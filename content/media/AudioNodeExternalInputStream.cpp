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
                                                           MediaStreamGraph::AudioNodeStreamKind aKind)
  : AudioNodeStream(aEngine, aKind)
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
AudioNodeExternalInputStream::GetTrackResampler(StreamBuffer::Track* aTrack,
                                                uint32_t aNumberOfInputChannels)
{
  SpeexResamplerState* resampler;
  ResamplerMapEntry* map;

  for (uint32_t i = 0; i < mResamplerMap.Length(); ++i) {
    map = &mResamplerMap[i];
    if (map->mTrackID == aTrack->GetID()) {
      return map->mResampler;
    }
  }

  resampler = speex_resampler_init(aNumberOfInputChannels,
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
  double finalPlaybackRate = aInputRate / IdealAudioRate();
  uint32_t availableInOutputBuffer = WEBAUDIO_BLOCK_SIZE;
  uint32_t availableInInputBuffer = 0;
  uint32_t inputSamples, outputSamples;

  AudioSegment* inputSegment = aInputTrack->Get<AudioSegment>();
  AudioSegment tmpSegment;

  tmpSegment.AppendSlice(*inputSegment, aStartTicks, aEndTicks);

  nsTArray<AudioChunk*> currentChunks;
  // TrackTicks totalChunksDuration = 0;
  for (AudioSegment::ChunkIterator ci(tmpSegment); !ci.IsEnded(); ci.Next()) {
    AudioChunk& chunk = *ci;
    availableInInputBuffer += chunk.GetDuration();
    //if (totalChunksDuration >= aStartTicks) {
    currentChunks.AppendElement(&chunk);
    //  availableInInputBuffer += chunk.mDuration;
    //}
  }

  if (!currentChunks.IsEmpty()) {
    bool newChunk = aOutputChunk->IsNull();
    const uint32_t numberOfChannels = currentChunks[0]->mChannelData.Length();
    if (newChunk) {
      AllocateAudioBlock(numberOfChannels, aOutputChunk);
    }

    SpeexResamplerState* resampler = GetTrackResampler(aInputTrack, numberOfChannels);

    for (uint32_t j = 0; j < currentChunks.Length(); ++j) {
      if (availableInInputBuffer < availableInOutputBuffer * finalPlaybackRate) {
        outputSamples = ceil(availableInInputBuffer / finalPlaybackRate);
        inputSamples = availableInInputBuffer;
      } else {
        inputSamples = ceil(availableInOutputBuffer * finalPlaybackRate);
        outputSamples = availableInOutputBuffer;
      }

      AudioChunk& c = *currentChunks[j];
      for (uint32_t i = 0; i < c.mChannelData.Length() && i < numberOfChannels; ++i) {
        // rawBuffer can hold either int16 or float audio data, depending on the format of
        // the input chunks.  We're using a raw byte buffer that can fit in a channel of
        // floats, which would be more than enough for int16.
        char rawBuffer[sizeof(float) * WEBAUDIO_BLOCK_SIZE];
        uint32_t currentOutputOffset = WEBAUDIO_BLOCK_SIZE - availableInOutputBuffer;

        uint32_t in = inputSamples;
        uint32_t out = outputSamples;
        float* finalData = static_cast<float*>(const_cast<void*>(aOutputChunk->mChannelData[i]));
        if (c.mBufferFormat == AUDIO_FORMAT_S16) {
          int16_t* outputData = reinterpret_cast<int16_t*>(rawBuffer);
          int16_t* inputData = static_cast<int16_t*>(const_cast<void*>(c.mChannelData[i]));
          speex_resampler_process_int(resampler, i,
                                      inputData, &in,
                                      outputData, &out);

          for (uint32_t k = currentOutputOffset; k < currentOutputOffset + out; ++k) {
            if (newChunk) {
              finalData[k] = AudioSampleToFloat(outputData[k]);
            } else {
              finalData[k] += AudioSampleToFloat(outputData[k]);
            }
          }
        } else {
          float* outputData = reinterpret_cast<float*>(rawBuffer);
          float* inputData = static_cast<float*>(const_cast<void*>(c.mChannelData[i]));
          speex_resampler_process_float(resampler, i,
                                        inputData, &in,
                                        outputData, &out);
          printf("input: %d => %d\toutput: %d => %d\n",
            inputSamples, in, outputSamples, out);
          
          for (uint32_t k = currentOutputOffset; k < currentOutputOffset + out; ++k) {
            if (newChunk) {
              finalData[k] = outputData[k];
            } else {
              finalData[k] += outputData[k];
            }
          }
        }

        if (i == c.mChannelData.Length() - 1) {
          // In the last iteration of the loop, remember how many more frames we need to produce.
          availableInOutputBuffer -= out;
          availableInInputBuffer -= in;
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
