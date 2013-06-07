/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-*/
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "MediaStreamGraphImpl.h"
#include "AudioNodeEngine.h"
#include "AudioNodeExternalInputStream.h"
#include "speex/speex_resampler.h"

using namespace mozilla::dom;

// Keep the size of the output queue reasonable.
static const int MAX_QUEUE_LENGTH = 20;

namespace mozilla {

AudioNodeExternalInputStream::AudioNodeExternalInputStream(AudioNodeEngine* aEngine)
    : AudioNodeStream(aEngine, MediaStreamGraph::INTERNAL_STREAM, IdealAudioRate()),
      mLastChunkOffset(0)
{
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
  AudioNodeExternalInputStream::TrackMapEntry* map;

  // Check the map for an existing entry corresponding to the input track.
  for (uint32_t i = 0; i < mTrackMap.Length(); ++i) {
    map = &mTrackMap[i];
    if (map->mTrackID == aTrack.GetID()) {
      return map;
    }
  }

  // Grab the first valid AudioChunk from the track.
  AudioSegment::ChunkIterator ci(*aTrack.Get<AudioSegment>());
  while ((*ci).IsNull() && !ci.IsEnded()) {
    ci.Next();
  }

  // Create a speex resampler with the same sample rate and number of channels
  // as the track.
  resampler = speex_resampler_init((*ci).mChannelData.Length(),
                                   aTrack.GetRate(),
                                   IdealAudioRate(),
                                   SPEEX_RESAMPLER_QUALITY_DEFAULT,
                                   nullptr);
  // Create a new map entry.
  map = mTrackMap.AppendElement();
  map->mResampler = resampler;
  map->mTrackID = aTrack.GetID();
  map->mLastTick = 0;

  return map;
}

void
AudioNodeExternalInputStream::WriteToCurrentChunk(SpeexResamplerState* aResampler,
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
  double finalPlaybackRate = inputPlaybackRate / IdealAudioRate();

  // Cache the number of channels in the input chunk.
  const uint32_t numberOfChannels = aInputChunk.mChannelData.Length();

  // The number of input samples available is the size of the input chunk minus the
  // amount of data we've already read from it.
  uint32_t inputSamples = aInputChunk.GetDuration() - aInputOffset;
  // The number of output samples available is the size of the output chunk (WEBAUDIO_BLOCK_SIZE)
  // minus the amount of data we've already written to it.
  uint32_t outputSamples = WEBAUDIO_BLOCK_SIZE - aOutputOffset;

  // In case no data is written, initialize these to 0.
  aActualOutputSamples = 0;
  aActualInputSamples = 0;

  // If there aren't enough samples left in the input chunk, compensate by decreasing the amount
  // of data we expect as output this time.
  if (inputSamples < outputSamples * finalPlaybackRate) {
    outputSamples = ceil(inputSamples / finalPlaybackRate);
  }

  for (uint32_t i = 0; i < numberOfChannels; ++i) {
    // rawBuffer can hold either int16 or float audio data, depending on the format of
    // the input chunks.  We're using a raw byte buffer that can fit in a channel of
    // floats, which would be more than enough for int16.
    char rawBuffer[sizeof(float) * WEBAUDIO_BLOCK_SIZE];

    // Store input and output values because they'll change when given to speex for processing.
    uint32_t in = inputSamples;
    uint32_t out = outputSamples;

    // Point to the output chunk's channel data with an offset since this chunk could already 
    // have data in it from a previous write.
    float* finalData = static_cast<float*>(const_cast<void*>(aOutputChunk.mChannelData[i])) + aOutputOffset;

    // Depending on the audio format, we need to cast input and output arrays appropriately.
    if (aInputChunk.mBufferFormat == AUDIO_FORMAT_S16) {

      // Similar to `finalData`, point to `rawBuffer` with an offset so that writing to
      // `finalData` is easily aligned.
      int16_t* outputData = reinterpret_cast<int16_t*>(rawBuffer) + aOutputOffset;
      // And again, but with an offset corresponding to the current position in the current input chunk,
      // since we may have started reading from it already in the past.
      int16_t* inputData = static_cast<int16_t*>(const_cast<void*>(aInputChunk.mChannelData[i])) + aInputOffset;
      
      // Finally, do some resampling!
      speex_resampler_process_int(aResampler, i,
                                  inputData, &in,
                                  outputData, &out);

      // *Add* the result to `finalData`, since multiple tracks could be writing to this chunk in the same
      // time window. Use AudioSampleToFloat to ensure that 16bit int data becomes a float for `finalData`.
      for (uint32_t k = 0; k < out; ++k) {
        finalData[k] += AudioSampleToFloat(outputData[k]);
      }
    }
    else {
      // Similar to `finalData`, point to `rawBuffer` with an offset so that writing to
      // `finalData` is easily aligned.
      float* outputData = reinterpret_cast<float*>(rawBuffer) + aOutputOffset;
      // And again, but with an offset corresponding to the current position in the current input chunk,
      // since we may have started reading from it already in the past.
      float* inputData = static_cast<float*>(const_cast<void*>(aInputChunk.mChannelData[i])) + aInputOffset;

      // Finally, do some resampling!
      speex_resampler_process_float(aResampler, i,
                                    inputData, &in,
                                    outputData, &out);
      
      // *Add* the result to `finalData`, since multiple tracks could be writing to this chunk in the same
      // time window. No float conversion needs to be done here, since we should already be in float land.
      for (uint32_t k = 0; k < out; ++k) {
        finalData[k] += outputData[k];
      }
    }

    // In the last iteration of the loop, since speex may not read all available input, record how many
    // samples were read and how many were written.
    if (i == aInputChunk.mChannelData.Length() - 1) {
      aActualOutputSamples = out;
      aActualInputSamples = in;
    }
  }
}

bool
AudioNodeExternalInputStream::PrepareOutputChunkQueue(AudioNodeExternalInputStream::ChunkMetaData& aChunkMetaData)
{
  // If we filled up the current output chunk the last time we wrote
  // output data, then we'll need to see if we should extend the queue or
  // just move to the next AudioChunk waiting in line.
  if (aChunkMetaData.mOffset == WEBAUDIO_BLOCK_SIZE) {
    // If the current output chunk is indeed at the end of the queue,...
    if (aChunkMetaData.mIndex == mOutputChunkQueue.size() - 1) {
      // ...and the queue isn't at its maximum size,...
      if (mOutputChunkQueue.size() == MAX_QUEUE_LENGTH) {
        return false;
      }
      // ...then add a new chunk to the back.
      AudioChunk tmpChunk;
      mOutputChunkQueue.push_back(tmpChunk);
    }
    // Regardless of whether or not we needed to add a new chunk, increase
    // the index for current output to the next chunk in line.
    ++aChunkMetaData.mIndex;
    // And, reset the offset so we start writing output at the beginning of the
    // current chunk.
    aChunkMetaData.mOffset = 0;
  }
  return true;
}

void
AudioNodeExternalInputStream::ConsumeInputData(const StreamBuffer::Track& aInputTrack,
                                               AudioNodeExternalInputStream::ChunkMetaData& aChunkMetaData)
{
  // As the track map for the entry corresponding to the input track. A new entry will be created
  // if one does not already exist. We'll need this map later to refer to the speex resampler created
  // for the track.
  AudioNodeExternalInputStream::TrackMapEntry* trackMap = GetTrackMap(aInputTrack);

  // If something bizarre happened and we're beyond the end of the input track, bail.
  if (trackMap->mLastTick > aInputTrack.GetEnd()) { return; }
  
  // Create a temporary segment containing only the AudioChunks from which we might need data.
  AudioSegment& inputSegment = *aInputTrack.Get<AudioSegment>();
  AudioSegment tmpSegment;
  tmpSegment.AppendSlice(inputSegment, trackMap->mLastTick, aInputTrack.GetEnd());

  // We'll need to remember how much data from the current input chunk we've already consumed
  // so that, if we need to move to another output chunk, we can switch, and start writing again.
  uint32_t chunkInputConsumed = 0;

  // Now, iterate over the input chunks from the input track.
  AudioSegment::ChunkIterator ci(tmpSegment);
  while (!ci.IsEnded()) {
    const AudioChunk& currentInputChunk = *ci;

    // TODO: these are still ok
    if (currentInputChunk.IsNull()) {
      ci.Next();
      continue;
    }

    // Make sure that an output chunk is available to receive new data. If no more are available,
    // and no more can be created this time, bail.
    if (!PrepareOutputChunkQueue(aChunkMetaData)) {
      break;
    }

    AudioChunk* currentOutputChunk = &mOutputChunkQueue[aChunkMetaData.mIndex];

    // If the current output chunk is brand new, allocate space for its data, and write 0's to it
    // in case we find ourselves in a position where no more data is available.
    if (currentOutputChunk->IsNull()) {
      AllocateAudioBlock(currentInputChunk.mChannelData.Length(), currentOutputChunk);
      WriteZeroesToAudioBlock(currentOutputChunk, 0, WEBAUDIO_BLOCK_SIZE);
    }

    // These variables are passed to WriteToCurrentChunk so it can report how many samples
    // were actually read and written by speex.
    uint32_t in, out;

    // Write data from the current input chunk to the current output chunk.
    WriteToCurrentChunk(trackMap->mResampler,
                        aInputTrack.GetRate(),
                        currentInputChunk, *currentOutputChunk,
                        chunkInputConsumed,
                        aChunkMetaData.mOffset,
                        in, out);

    // Record how much data was read from input, and how much was written to output. Here, we advance
    // the `mLastTick` property of the current track map to remember the last `TrackTick` at which we read
    // data. We advance `chunkInputConsumed` similarly to see if we need to move to the next input chunk.
    // Lastly, in case the input chunk was completely consumed, advance the output chunk offset so that we
    // can continue to write to the same chunk again starting at the right position. 
    trackMap->mLastTick += in;
    chunkInputConsumed += in;
    aChunkMetaData.mOffset += out;

    // If the input chunk was completely consumed, move to the next input chunk and reset the consumption counter.
    if (chunkInputConsumed == currentInputChunk.GetDuration()) {
      ci.Next();
      chunkInputConsumed = 0;
    }
  }

}

void
AudioNodeExternalInputStream::ProduceOutput(GraphTime aFrom, GraphTime aTo)
{
  MOZ_ASSERT(mInputs.Length() == 1);

  uint16_t outputCount = std::max(uint16_t(1), mEngine->OutputCount());
  mLastChunks.SetLength(outputCount);

  // If there is room in the output buffer, try to consume more input data.
  if (mOutputChunkQueue.size() < MAX_QUEUE_LENGTH) {

    // We need to point to the correct position inside the newest chunk so we can start writing
    // data again. Here, we figure out where we were last time or just make sure the output queue
    // is ready to accept data if it's empty.
    AudioNodeExternalInputStream::ChunkMetaData lastChunkMetaData;
    if (mOutputChunkQueue.empty()) {
      AudioChunk tmpChunk;
      mOutputChunkQueue.push_back(tmpChunk);

      // If the queue was empty, it doesn't matter where we were last time. We need to start fresh.
      lastChunkMetaData.mIndex = 0;
      lastChunkMetaData.mOffset = 0;
    }
    else {
      // The last time we wrote data, it has to have been in the last chunk. Point to it, set the
      // output offset from the last time we wrote data to this chunk.
      lastChunkMetaData.mIndex = mOutputChunkQueue.size() - 1;
      lastChunkMetaData.mOffset = mLastChunkOffset;      
    }

    // Mulitple tracks additively write data to the same chunks, so we need to figure out which track wrote
    // the most data, and remember the offset of the last chunk the gets created.
    AudioNodeExternalInputStream::ChunkMetaData maxChunkMetaData;
    maxChunkMetaData.mIndex = 0;
    maxChunkMetaData.mOffset = 0;

    // Iterate over all the input tracks.
    for (StreamBuffer::TrackIter tracks(mInputs[0]->GetSource()->mBuffer, MediaSegment::AUDIO);
         !tracks.IsEnded(); tracks.Next()) {
      StreamBuffer::Track& inputTrack = *tracks;

      // To record what happens to the output chunk queue and offset during the resampling of an individual track,
      // we create a new ChunkMetaData object which can be used for comparison with other tracks' work when we're
      // finished collecting data from this track.
      AudioNodeExternalInputStream::ChunkMetaData chunkMetaData = lastChunkMetaData;

      // Start consuming data from the input track to write to the last known output position.
      ConsumeInputData(inputTrack, chunkMetaData);

      // As a result of consuming input data from this track, if we wrote out more data than for any other track
      // this time, remember the specifics.
      if (chunkMetaData.mIndex >= maxChunkMetaData.mIndex && chunkMetaData.mOffset > maxChunkMetaData.mOffset) {
        maxChunkMetaData = chunkMetaData;
      }
    }

    // If the maximum values were actually set (if data was actually written to output), save the last output chunk
    // offset for next time.
    if (maxChunkMetaData.mIndex > 0 && maxChunkMetaData.mOffset > 0) {
      mLastChunkOffset = maxChunkMetaData.mOffset;
    }
  }

  // If the output chunk queue is not empty, or the only chunk in the queue is actually full of data, then go ahead
  // and let the next audio node steal it by popping it from the queue and handing it to AudioNodeStream's
  // `mLastChunks` array.
  if (!mOutputChunkQueue.empty() && !(mOutputChunkQueue.size() == 1 && mLastChunkOffset < WEBAUDIO_BLOCK_SIZE)) {
    mLastChunks[0] = mOutputChunkQueue.front();
    mOutputChunkQueue.pop_front();
  }
  else {
    // Otherwise, make no noise.
    mLastChunks[0].SetNull(WEBAUDIO_BLOCK_SIZE);
  }

  // Using AudioNodeStream's AdvanceOutputSegment to push the media stream graph along with null data.
  AdvanceOutputSegment();
}

}
