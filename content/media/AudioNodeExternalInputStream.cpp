/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-*/
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "MediaStreamGraphImpl.h"
#include "AudioNodeEngine.h"
#include "AudioNodeExternalInputStream.h"
#include "speex/speex_resampler.h"

using namespace mozilla::dom;

namespace mozilla {

AudioNodeExternalInputStream::AudioNodeExternalInputStream(AudioNodeEngine* aEngine, TrackRate aSampleRate)
    : AudioNodeStream(aEngine, MediaStreamGraph::INTERNAL_STREAM, aSampleRate)
{
  MOZ_COUNT_CTOR(AudioNodeExternalInputStream);
}

AudioNodeExternalInputStream::~AudioNodeExternalInputStream()
{
  MOZ_COUNT_DTOR(AudioNodeExternalInputStream);

  // Destroy all speex resamplers -- one per input track.
  for (uint32_t i = 0; i < mTrackMap.Length(); ++i) {
    speex_resampler_destroy(mTrackMap[i].mResampler);
  }
}

AudioNodeExternalInputStream::TrackMapEntry*
AudioNodeExternalInputStream::GetTrackMap(const StreamBuffer::Track& aTrack)
{
  SpeexResamplerState* resampler;
  TrackMapEntry* map;

  // Check the map for an existing entry corresponding to the input track.
  for (uint32_t i = 0; i < mTrackMap.Length(); ++i) {
    map = &mTrackMap[i];
    if (map->mTrackID == aTrack.GetID()) {
      return map;
    }
  }

  // Grab the first valid AudioChunk from the track.
  AudioSegment::ChunkIterator ci(*aTrack.Get<AudioSegment>());
  while (!ci.IsEnded() && (*ci).IsNull()) {
    ci.Next();
  }

  if (ci.IsEnded()) {
    return nullptr;
  }

  // Create a speex resampler with the same sample rate and number of channels
  // as the track.
  resampler = speex_resampler_init((*ci).mChannelData.Length(),
                                   aTrack.GetRate(),
                                   mSampleRate,
                                   SPEEX_RESAMPLER_QUALITY_DEFAULT,
                                   nullptr);
  speex_resampler_skip_zeros(resampler);

  // Create a new map entry.
  map = mTrackMap.AppendElement();
  map->mResampler = resampler;
  map->mTrackID = aTrack.GetID();
  map->mLastInputTick = 0;
  map->mFrontChunkOffset = 0;

  return map;
}

void
AudioNodeExternalInputStream::CopyFromChunk(const AudioChunk& aInputChunk,
                                            const AudioChunk& aOutputChunk,
                                            uint32_t aInputOffset,
                                            uint32_t aOutputOffset,
                                            uint32_t& aActualOutputSamples)
{
  const uint32_t numberOfChannels = aInputChunk.mChannelData.Length();

  MOZ_ASSERT(aInputOffset <= aInputChunk.GetDuration(), "Input AudioChunk offset out of bounds.");
  uint32_t inputSamples = aInputChunk.GetDuration() - aInputOffset;

  MOZ_ASSERT(aOutputOffset <= WEBAUDIO_BLOCK_SIZE, "Output AudioChunk offset out of bounds.");
  uint32_t outputSamples = WEBAUDIO_BLOCK_SIZE - aOutputOffset;

  for (uint32_t i = 0; i < numberOfChannels; ++i) {
    float* finalData = static_cast<float*>(const_cast<void*>(aOutputChunk.mChannelData[i])) + aOutputOffset;
    if (aInputChunk.mBufferFormat == AUDIO_FORMAT_S16) {
      int16_t* inputData = static_cast<int16_t*>(const_cast<void*>(aInputChunk.mChannelData[i])) + aInputOffset;
      for (uint32_t k = 0; k < outputSamples && k < inputSamples; ++k) {
        finalData[k] = AudioSampleToFloat(inputData[k]);
      }
    } else {
      float* inputData = static_cast<float*>(const_cast<void*>(aInputChunk.mChannelData[i])) + aInputOffset;
      for (uint32_t k = 0; k < outputSamples && k < inputSamples; ++k) {
        finalData[k] = inputData[k];
      }
    }
  }

  aActualOutputSamples = std::min(outputSamples, inputSamples);
}

void
AudioNodeExternalInputStream::ResampleChunk(SpeexResamplerState* aResampler,
                                            TrackRate aInputRate,
                                            const AudioChunk& aInputChunk,
                                            const AudioChunk& aOutputChunk,
                                            uint32_t aInputOffset,
                                            uint32_t aOutputOffset,
                                            uint32_t& aActualInputSamples,
                                            uint32_t& aActualOutputSamples)
{
  // Make sure these values are copied as doubles for accurate use later.
  double inputPlaybackRate = aInputRate;
  double playbackRateRatio = inputPlaybackRate / mSampleRate;

  // Cache the number of channels in the input chunk.
  const uint32_t numberOfChannels = aInputChunk.mChannelData.Length();

  // The number of input samples available is the size of the input chunk minus the
  // amount of data we've already read from it.
  MOZ_ASSERT(aInputOffset <= aInputChunk.GetDuration(), "Input AudioChunk offset out of bounds.");
  uint32_t inputSamples = aInputChunk.GetDuration() - aInputOffset;

  // The number of output samples available is the size of the output chunk (WEBAUDIO_BLOCK_SIZE)
  // minus the amount of data we've already written to it.
  MOZ_ASSERT(aOutputOffset <= WEBAUDIO_BLOCK_SIZE, "Output AudioChunk offset out of bounds.");
  uint32_t outputSamples = WEBAUDIO_BLOCK_SIZE - aOutputOffset;

  // If there aren't enough samples left in the input chunk, compensate by decreasing the amount
  // of data we expect as output this time.
  if (inputSamples < outputSamples * playbackRateRatio) {
    outputSamples = ceil(inputSamples / playbackRateRatio);
  }

  for (uint32_t i = 0; i < numberOfChannels; ++i) {
    // Store input and output values because they'll change when given to speex for processing.
    uint32_t in = inputSamples;
    uint32_t out = outputSamples;

    // Point to the output chunk's channel data with an offset since this chunk could already
    // have data in it from a previous write.
    float* finalData = static_cast<float*>(const_cast<void*>(aOutputChunk.mChannelData[i])) + aOutputOffset;

    // Depending on the audio format, we need to cast input and output arrays appropriately.
    if (aInputChunk.mBufferFormat == AUDIO_FORMAT_S16) {
      int16_t outputData[sizeof(int16_t) * WEBAUDIO_BLOCK_SIZE];
      // Point at the beginning of input with an offset corresponding to the current position in the chunk,
      // since we may have started reading from it already in the past.
      int16_t* inputData = static_cast<int16_t*>(const_cast<void*>(aInputChunk.mChannelData[i])) + aInputOffset;

      // Finally, do some resampling!
      speex_resampler_process_int(aResampler, i,
                                  inputData, &in,
                                  outputData, &out);

      for (uint32_t k = 0; k < out; ++k) {
        finalData[k] = AudioSampleToFloat(outputData[k]);
      }
    } else {
      // Point at the beginning of input with an offset corresponding to the current position in the chunk,
      // since we may have started reading from it already in the past.
      float* inputData = static_cast<float*>(const_cast<void*>(aInputChunk.mChannelData[i])) + aInputOffset;

      // Finally, do some resampling!
      speex_resampler_process_float(aResampler, i,
                                    inputData, &in,
                                    finalData, &out);
    }

    // In the last iteration of the loop, since speex may not read all available input, record how many
    // samples were read and how many were written.
    if (i == aInputChunk.mChannelData.Length() - 1) {
      aActualOutputSamples = out;
      aActualInputSamples = in;
    }
  }
}

void
AudioNodeExternalInputStream::MixTracks(const nsTArray<AudioChunk>& aOutputChunks,
                                        AudioChunk& aDestinationChunk)
{
  for (uint32_t i = 0; i < aOutputChunks.Length(); ++i) {
    uint32_t numberOfChannels = aOutputChunks[i].mChannelData.Length();
    for (uint32_t j = 0; j < numberOfChannels; ++j) {
      float* outputChunkData = static_cast<float*>(const_cast<void*>(aOutputChunks[i].mChannelData[j]));
      float* aggregateChunkData = static_cast<float*>(const_cast<void*>(aDestinationChunk.mChannelData[j]));
      AudioBlockAddChannelWithScale(outputChunkData, aOutputChunks[i].mVolume, aggregateChunkData);
    }
  }
}

void
AudioNodeExternalInputStream::ConsumeDataFromTrack(AudioChunk& aOutputChunk,
                                                   const StreamBuffer::Track& aInputTrack,
                                                   TrackMapEntry& aTrackMap)
{
  uint32_t outputOffset = 0;

  while (outputOffset < WEBAUDIO_BLOCK_SIZE && !aTrackMap.mChunkQueue.empty()) {
    // These variables are passed to ResampleChunk so it can report how many samples
    // were actually read and written by speex.
    uint32_t in = 0, out = 0;

    AudioChunk& inputChunk = aTrackMap.mChunkQueue.front();

    if (inputChunk.IsNull()) {
      double inputPlaybackRate = aInputTrack.GetRate();
      double playbackRateRatio = inputPlaybackRate / mSampleRate;

      uint32_t inputSamples = inputChunk.GetDuration() - aTrackMap.mFrontChunkOffset;
      uint32_t outputSamples = WEBAUDIO_BLOCK_SIZE - outputOffset;

      if (inputSamples < outputSamples * playbackRateRatio) {
        outputSamples = ceil(inputSamples / playbackRateRatio);
      } else if (outputSamples < inputSamples / playbackRateRatio) {
        inputSamples = ceil(outputSamples * playbackRateRatio);
      }

      WriteZeroesToAudioBlock(&aOutputChunk, outputOffset, outputSamples);
      outputOffset += outputSamples;
      aTrackMap.mFrontChunkOffset += inputSamples;
    } else {
      // This will be an estimation since one output chunk maybe be comprised of several input chunks.
      aOutputChunk.mVolume = inputChunk.mVolume;

      // If input sample rate is equal to output sample rate, just put the input chunk
      // directly into the output chunk array. No need to resample.
      if (aInputTrack.GetRate() == mSampleRate) {
        CopyFromChunk(inputChunk, aOutputChunk,
                      aTrackMap.mFrontChunkOffset, outputOffset,
                      out);
        in = out;
      } else {
        // Write data from the current input chunk to the current output chunk.
        ResampleChunk(aTrackMap.mResampler,
                      aInputTrack.GetRate(),
                      inputChunk, aOutputChunk,
                      aTrackMap.mFrontChunkOffset, outputOffset,
                      in, out);
      }
    }

    outputOffset += out;

    aTrackMap.mFrontChunkOffset += in;
    if (aTrackMap.mFrontChunkOffset == inputChunk.GetDuration()) {
      aTrackMap.mFrontChunkOffset = 0;
      aTrackMap.mChunkQueue.pop_front();
    }
  }
}

void
AudioNodeExternalInputStream::BufferInputData(const StreamBuffer::Track& aInputTrack,
                                              TrackMapEntry& aTrackMap)
{
  // Create a temporary segment containing only the AudioChunks from which we might need data.
  AudioSegment& inputSegment = *aInputTrack.Get<AudioSegment>();
  AudioSegment tmpSegment;
  tmpSegment.AppendSlice(inputSegment, aTrackMap.mLastInputTick, aInputTrack.GetEnd());

  // Now, iterate over the input chunks from the input track.
  AudioSegment::ChunkIterator ci(tmpSegment);
  while (!ci.IsEnded()) {
    const AudioChunk& inputChunk = *ci;
    if (inputChunk.GetDuration() > 0) {
      aTrackMap.mChunkQueue.push_back(inputChunk);
      aTrackMap.mLastInputTick += inputChunk.GetDuration();
    }
    ci.Next();
  }
}

void
AudioNodeExternalInputStream::ProduceOutput(GraphTime aFrom, GraphTime aTo)
{
  MOZ_ASSERT(mInputs.Length() == 1);

  // According to spec, number of outputs is always 1.
  mLastChunks.SetLength(1);

  nsTArray<AudioChunk> outputChunks;
  uint32_t numChannels = 0;

  for (StreamBuffer::TrackIter tracks(mInputs[0]->GetSource()->mBuffer, MediaSegment::AUDIO);
       !tracks.IsEnded(); tracks.Next()) {
    const StreamBuffer::Track& inputTrack = *tracks;

    if (inputTrack.GetRate() == 0) { continue; }

    // As the track map for the entry corresponding to the input track. A new entry will be created
    // if one does not already exist. We'll need this map later to refer to the speex resampler created
    // for the track.
    TrackMapEntry* trackMap = GetTrackMap(inputTrack);

    // If something bizarre happened and we're beyond the end of the input track, bail.
    if (!trackMap || trackMap->mLastInputTick > inputTrack.GetEnd()) { continue; }

    // Start consuming data from the input track to write to the last known output position.
    BufferInputData(inputTrack, *trackMap);

    if (!trackMap->mChunkQueue.empty()) {
      AudioChunk* outputChunk = outputChunks.AppendElement();
      AllocateAudioBlock(trackMap->mChunkQueue.front().mChannelData.Length(), outputChunk);
      WriteZeroesToAudioBlock(outputChunk, 0, WEBAUDIO_BLOCK_SIZE);
      ConsumeDataFromTrack(*outputChunk, inputTrack, *trackMap);
      numChannels = std::max(numChannels, outputChunk->mChannelData.Length());
    }
  }

  if (!outputChunks.IsEmpty()) {
    MOZ_ASSERT(numChannels > 0, "Number of output channels shouldn't be 0.");
    AllocateAudioBlock(numChannels, &mLastChunks[0]);
    MixTracks(outputChunks, mLastChunks[0]);

  } else {
    // Otherwise, make no noise.
    mLastChunks[0].SetNull(WEBAUDIO_BLOCK_SIZE);
  }

  // Using AudioNodeStream's AdvanceOutputSegment to push the media stream graph along with null data.
  AdvanceOutputSegment();
}

}
