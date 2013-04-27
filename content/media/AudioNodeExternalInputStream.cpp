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

/**
 * An AudioNodeStream produces a single audio track with ID
 * AUDIO_NODE_STREAM_TRACK_ID. This track has rate IdealAudioRate().
 * Each chunk in the track is a single block of WEBAUDIO_BLOCK_SIZE samples.
 */
static const int AUDIO_NODE_STREAM_TRACK_ID = 1;

AudioNodeExternalInputStream::AudioNodeExternalInputStream(AudioNodeEngine* aEngine,
                                                           MediaStreamGraph::AudioNodeStreamKind aKind,
                                                           uint32_t aNumberOfInputChannels)
  : AudioNodeStream(aEngine, aKind, aNumberOfInputChannels)
{
}


AudioNodeExternalInputStream::~AudioNodeExternalInputStream()
{
  MOZ_COUNT_DTOR(AudioNodeExternalInputStream);
  for (uint32_t i = 0; i < mResamplerMap.Length(); ++i) {
    speex_resampler_destroy(mResamplerMap[i].mResampler);  
  }
}

SpeexResamplerState*
AudioNodeExternalInputStream::GetTrackResampler(StreamBuffer::Track* aTrack)
{
  SpeexResamplerState* resampler;
  ResamplerMapEntry* map;

  for (uint32_t i = 0; i < mResamplerMap.Length(); ++i) {
    map = &mResamplerMap[i];
    if (map->mTrackID == aTrack->GetID()) {
      return map->mResampler;
    }
  }

  resampler = speex_resampler_init(mNumberOfInputChannels,
                                   aTrack->GetRate(),
                                   IdealAudioRate(),
                                   SPEEX_RESAMPLER_QUALITY_DEFAULT,
                                   nullptr);
  map = mResamplerMap.AppendElement();
  map->mResampler = resampler;
  map->mTrackID = aTrack->GetID();

  return resampler;
}

void
AudioNodeExternalInputStream::WriteDataToOutputChunk(StreamBuffer::Track* aInputTrack,
                                                     AudioChunk* aOutputChunk,
                                                     TrackRate aInputRate,
                                                     TrackTicks aStartTicks,
                                                     TrackTicks aEndTicks)
{
  uint32_t bufferOffset = 0;
  double finalSampleRate = aInputRate;
  double finalPlaybackRate = finalSampleRate / IdealAudioRate();
  uint32_t availableInOutputBuffer = WEBAUDIO_BLOCK_SIZE - bufferOffset;
  uint32_t availableInInputBuffer = 0;
  uint32_t inputSamples, outputSamples;

  AudioSegment* inputSegment = aInputTrack->Get<AudioSegment>();
  AudioSegment tmpSegment;

  AudioChunk tmpChunk;

  tmpSegment.AppendSlice(*inputSegment, aStartTicks, aEndTicks);

  nsTArray<AudioChunk*> currentChunks;
  // TrackTicks totalChunksDuration = 0;
  for (AudioSegment::ChunkIterator ci(tmpSegment); !ci.IsEnded(); ci.Next()) {
    AudioChunk& chunk = *ci;
    availableInInputBuffer += chunk.mDuration;
    //if (totalChunksDuration >= aStartTicks) {
    currentChunks.AppendElement(&chunk);
    //  availableInInputBuffer += chunk.mDuration;
    //}
  }

  if (currentChunks.Length()) {
    SpeexResamplerState* resampler = GetTrackResampler(aInputTrack);

    if (aOutputChunk->IsNull()) {
      AllocateAudioBlock(mNumberOfInputChannels, aOutputChunk);
      WriteZeroesToAudioBlock(aOutputChunk, 0, WEBAUDIO_BLOCK_SIZE);
    }

    AllocateAudioBlock(mNumberOfInputChannels, &tmpChunk);
    WriteZeroesToAudioBlock(&tmpChunk, 0, WEBAUDIO_BLOCK_SIZE);

    if (availableInInputBuffer < availableInOutputBuffer * finalPlaybackRate) {
      outputSamples = ceil(availableInInputBuffer / finalPlaybackRate);
      inputSamples = availableInInputBuffer;
    } else {
      inputSamples = ceil(availableInOutputBuffer * finalPlaybackRate);
      outputSamples = availableInOutputBuffer;
    }

    for (uint32_t j = 0; j < currentChunks.Length(); ++j) {
      AudioChunk& c = *currentChunks[j];
      for (uint32_t i = 0; i < c.mChannelData.Length() && i < mNumberOfInputChannels; ++i) {
        float* outputData = static_cast<float*>(const_cast<void*>(tmpChunk.mChannelData[i]));
        float* inputData = static_cast<float*>(const_cast<void*>(c.mChannelData[i]));
        float* finalData = static_cast<float*>(const_cast<void*>(aOutputChunk->mChannelData[i]));
        speex_resampler_process_float(resampler, i,
                                      inputData, &inputSamples,
                                      outputData, &outputSamples);

        // Add tmpChunk channel data to outputChunk. Note: Not a straight copy because we need to retain
        // data already in outputChunk.
        for (uint32_t j = 0; j < WEBAUDIO_BLOCK_SIZE; ++j) {
          finalData[j] += outputData[j];
        }
      }
    }
  }
}

void
AudioNodeExternalInputStream::ProduceOutput(GraphTime aFrom, GraphTime aTo)
{
  AudioChunk outputChunk;
  outputChunk.SetNull(WEBAUDIO_BLOCK_SIZE);

  MOZ_ASSERT(mInputs.Length() == 1);

  MediaInputPort* inputPort = mInputs[0];
  MediaStream* inputStream = inputPort->GetSource();

  StreamTime from = inputStream->GraphTimeToStreamTime(aFrom);
  StreamTime to = inputStream->GraphTimeToStreamTime(aTo);

  GraphTime next;

  for (StreamBuffer::TrackIter tracks(inputStream->mBuffer, MediaSegment::AUDIO);
       !tracks.IsEnded(); tracks.Next()) {
  
    StreamBuffer::Track& inputTrack = *tracks;

    for (GraphTime t = aFrom; t < aTo; t = next) {
      MediaInputPort::InputInterval interval = inputPort->GetNextInputInterval(t);
      interval.mEnd = std::min(interval.mEnd, aTo);
      if (interval.mStart >= interval.mEnd)
        break;
      next = interval.mEnd;

      TrackRate rate = inputTrack.GetRate();
      TrackTicks startTicks = TimeToTicksRoundDown(rate, from);
      TrackTicks endTicks = TimeToTicksRoundDown(rate, to);
      TrackTicks ticksDuration = endTicks - startTicks;

      StreamTime inputEnd = inputStream->GraphTimeToStreamTime(interval.mEnd);
      TrackTicks inputTrackEndPoint = TRACK_TICKS_MAX;

      if (inputTrack.IsEnded()) {
        TrackTicks inputEndTicks = inputTrack.TimeToTicksRoundDown(inputEnd);
        if (inputTrack.GetEnd() <= inputEndTicks) {
          inputTrackEndPoint = inputTrack.GetEnd();
          // track has ended
        }
      }

      if (interval.mInputIsBlocked) {
        // Maybe the input track ended?
        //outputSegment.AppendNullData(ticksDuration);
      }
      else {
        TrackTicks inputEndTicks = TimeToTicksRoundUp(rate, inputEnd);
        TrackTicks inputStartTicks = inputEndTicks - ticksDuration;

        WriteDataToOutputChunk(&inputTrack, &outputChunk,
                               rate,
                               inputStartTicks, std::min(inputTrackEndPoint, inputEndTicks));
      }

    }
  }

  mLastChunk = outputChunk;
  FinalizeProducedOutput(&outputChunk);
}

}
