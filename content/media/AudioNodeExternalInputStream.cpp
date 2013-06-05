/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-*/
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "MediaStreamGraphImpl.h"
#include "AudioNodeEngine.h"
#include "AudioNodeExternalInputStream.h"
#include "speex/speex_resampler.h"

using namespace mozilla::dom;

static const int MAX_QUEUE_LENGTH = 500;

namespace mozilla {

AudioNodeExternalInputStream::AudioNodeExternalInputStream(AudioNodeEngine* aEngine)
    : AudioNodeStream(aEngine, MediaStreamGraph::INTERNAL_STREAM, IdealAudioRate()),
      mLastChunkOffset(0)
{
}

AudioNodeExternalInputStream::~AudioNodeExternalInputStream()
{
  MOZ_COUNT_DTOR(AudioNodeExternalInputStream);
  for (uint32_t i = 0; i < mTrackMap.Length(); ++i) {
    speex_resampler_destroy(mTrackMap[i].mResampler);  
  }
}

AudioNodeExternalInputStream::TrackMapEntry*
AudioNodeExternalInputStream::GetTrackMap(const StreamBuffer::Track& aTrack)
{
  SpeexResamplerState* resampler;
  AudioNodeExternalInputStream::TrackMapEntry* map;

  for (uint32_t i = 0; i < mTrackMap.Length(); ++i) {
    map = &mTrackMap[i];
    if (map->mTrackID == aTrack.GetID()) {
      return map;
    }
  }

  AudioSegment::ChunkIterator ci(*aTrack.Get<AudioSegment>());
  while ((*ci).IsNull() && !ci.IsEnded()) {
    ci.Next();
  }

  resampler = speex_resampler_init((*ci).mChannelData.Length(),
                                   aTrack.GetRate(),
                                   IdealAudioRate(),
                                   SPEEX_RESAMPLER_QUALITY_DEFAULT,
                                   nullptr);
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
                                                  uint32_t aOutputOffset,
                                                  uint32_t& aActualIntputSamples,
                                                  uint32_t& aActualOutputSamples)
{
  double inputPlaybackRate = aInputRate;
  double finalPlaybackRate = inputPlaybackRate / IdealAudioRate();

  const uint32_t numberOfChannels = aInputChunk.mChannelData.Length();

  uint32_t inputSamples = aInputChunk.GetDuration();
  uint32_t outputSamples = WEBAUDIO_BLOCK_SIZE - aOutputOffset;

  aActualOutputSamples = 0;
  aActualIntputSamples = 0;

  if (inputSamples < outputSamples * finalPlaybackRate) {
    outputSamples = ceil(inputSamples / finalPlaybackRate);
  }

  for (uint32_t i = 0; i < numberOfChannels; ++i) {
    // rawBuffer can hold either int16 or float audio data, depending on the format of
    // the input chunks.  We're using a raw byte buffer that can fit in a channel of
    // floats, which would be more than enough for int16.
    char rawBuffer[sizeof(float) * WEBAUDIO_BLOCK_SIZE];

    uint32_t in = inputSamples;
    uint32_t out = outputSamples;
    float* finalData = static_cast<float*>(const_cast<void*>(aOutputChunk.mChannelData[i]));

    if (aInputChunk.mBufferFormat == AUDIO_FORMAT_S16) {
      int16_t* outputData = reinterpret_cast<int16_t*>(rawBuffer);
      int16_t* inputData = static_cast<int16_t*>(const_cast<void*>(aInputChunk.mChannelData[i]));
      speex_resampler_process_int(aResampler, i,
                                  inputData, &in,
                                  outputData, &out);

      for (uint32_t k = aOutputOffset; k < aOutputOffset + out; ++k) {
        finalData[k] += AudioSampleToFloat(outputData[k]);
      }
    }
    else {
      float* outputData = reinterpret_cast<float*>(rawBuffer);
      float* inputData = static_cast<float*>(const_cast<void*>(aInputChunk.mChannelData[i]));
      speex_resampler_process_float(aResampler, i,
                                    inputData, &in,
                                    outputData, &out);
      
      for (uint32_t k = aOutputOffset; k < aOutputOffset + out; ++k) {
        finalData[k] += outputData[k];
      }
    }

    if (i == aInputChunk.mChannelData.Length() - 1) {
      // In the last iteration of the loop, remember how many more frames we need to produce.
      // printf("input: %d => %d\toutput: %d => %d\n",
      //   inputSamples, in, outputSamples, out);
      aActualOutputSamples = out;
      aActualIntputSamples = in;
    }
  }
}

bool
AudioNodeExternalInputStream::PrepareOutputChunkQueue(AudioNodeExternalInputStream::ChunkMetaData& aChunkMetaData)
{
  if (aChunkMetaData.mOffset == WEBAUDIO_BLOCK_SIZE) {
    if (aChunkMetaData.mIndex == mOutputChunkQueue.size() - 1) {
      if (mOutputChunkQueue.size() == MAX_QUEUE_LENGTH) {
        return false;
      }
      AudioChunk tmpChunk;
      mOutputChunkQueue.push_back(tmpChunk);
    }
    ++aChunkMetaData.mIndex;
    aChunkMetaData.mOffset = 0;
  }
  return true;
}

void
AudioNodeExternalInputStream::ConsumeInputData(const StreamBuffer::Track& aInputTrack,
                                               AudioNodeExternalInputStream::ChunkMetaData& aChunkMetaData)
{
  AudioNodeExternalInputStream::TrackMapEntry* trackMap = GetTrackMap(aInputTrack);

  AudioSegment& inputSegment = *aInputTrack.Get<AudioSegment>();
  SpeexResamplerState* resampler = trackMap->mResampler;

  AudioSegment tmpSegment;
  if (trackMap->mLastTick > aInputTrack.GetEnd()) { return; }
  tmpSegment.AppendSlice(inputSegment, trackMap->mLastTick, aInputTrack.GetEnd());

  AudioSegment::ChunkIterator ci(tmpSegment);
  
  uint32_t chunkInputConsumed = 0;
  uint32_t in;
  uint32_t out;

  while (!ci.IsEnded()) {
    const AudioChunk& currentInputChunk = *ci;

    // TODO: these are still ok
    if (currentInputChunk.IsNull()) {
      ci.Next();
      continue;
    }

    if (!PrepareOutputChunkQueue(aChunkMetaData)) {
      break;
    }

    if (mOutputChunkQueue[aChunkMetaData.mIndex].IsNull()) {
      AllocateAudioBlock(currentInputChunk.mChannelData.Length(), &mOutputChunkQueue[aChunkMetaData.mIndex]);
      WriteZeroesToAudioBlock(&mOutputChunkQueue[aChunkMetaData.mIndex], 0, WEBAUDIO_BLOCK_SIZE);
    }

    WriteToCurrentChunk(resampler,
                        aInputTrack.GetRate(),
                        currentInputChunk, mOutputChunkQueue[aChunkMetaData.mIndex],
                        aChunkMetaData.mOffset,
                        in, out);

    chunkInputConsumed += in;
    trackMap->mLastTick += in;
    aChunkMetaData.mOffset += out;

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

  if (mOutputChunkQueue.size() < MAX_QUEUE_LENGTH) {
    AudioNodeExternalInputStream::ChunkMetaData lastChunkMetaData;
    if (mOutputChunkQueue.empty()) {
      AudioChunk tmpChunk;
      mOutputChunkQueue.push_back(tmpChunk);
      lastChunkMetaData.mIndex = 0;
      lastChunkMetaData.mOffset = 0;
    }
    else {
      lastChunkMetaData.mIndex = mOutputChunkQueue.size() - 1;
      lastChunkMetaData.mOffset = mLastChunkOffset;      
    }

    AudioNodeExternalInputStream::ChunkMetaData maxChunkMetaData = lastChunkMetaData;

    for (StreamBuffer::TrackIter tracks(mInputs[0]->GetSource()->mBuffer, MediaSegment::AUDIO);
         !tracks.IsEnded(); tracks.Next()) {
      StreamBuffer::Track& inputTrack = *tracks;

      AudioNodeExternalInputStream::ChunkMetaData chunkMetaData = lastChunkMetaData;

      ConsumeInputData(inputTrack, chunkMetaData);

      if (chunkMetaData.mIndex >= maxChunkMetaData.mIndex && chunkMetaData.mOffset > maxChunkMetaData.mOffset) {
        maxChunkMetaData = chunkMetaData;
      }
    }

    mLastChunkOffset = maxChunkMetaData.mOffset;
  }

  if (!mOutputChunkQueue.empty()) {
    mLastChunks[0] = mOutputChunkQueue.front();
    if (mLastChunks[0].IsNull()) {
      printf("WTF 1\n");
    }
    mOutputChunkQueue.pop_front();
  }
  else {
    printf("WTF 2\n");
    mLastChunks[0].SetNull(WEBAUDIO_BLOCK_SIZE);
  }

  AdvanceOutputSegment();
}

}
