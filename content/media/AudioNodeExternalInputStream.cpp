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

static const int DEFAULT_NUM_INPUT_CHANNELS = 2;

AudioNodeExternalInputStream::AudioNodeExternalInputStream(AudioNodeEngine* aEngine,
                                                           MediaStreamGraph::AudioNodeStreamKind aKind)
    : ProcessedMediaStream(nullptr),
      mEngine(aEngine)
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
AudioNodeExternalInputStream::GetTrackMap(StreamBuffer::Track* aTrack)
{
  SpeexResamplerState* resampler;
  AudioNodeExternalInputStream::TrackMapEntry* map;

  for (uint32_t i = 0; i < mTrackMap.Length(); ++i) {
    map = &mTrackMap[i];
    if (map->mTrackID == aTrack->GetID()) {
      return map;
    }
  }

  AudioSegment::ChunkIterator ci(*aTrack->Get<AudioSegment>());
  while ((*ci).IsNull() && !ci.IsEnded()) {
    ci.Next();
  }

  resampler = speex_resampler_init((*ci).mChannelData.Length(),
                                   aTrack->GetRate(),
                                   IdealAudioRate(),
                                   SPEEX_RESAMPLER_QUALITY_DEFAULT,
                                   nullptr);
  map = mTrackMap.AppendElement();
  map->mResampler = resampler;
  map->mTrackID = aTrack->GetID();
  map->mLastTick = 0;

  return map;
}

void
AudioNodeExternalInputStream::WriteDataToOutputChunk(StreamBuffer::Track* aInputTrack,
                                                     AudioChunk* aOutputChunk,
                                                     TrackRate aInputRate,
                                                     TrackTicks aStartTicks,
                                                     TrackTicks aEndTicks)
{
  double inputPlaybackRate = aInputRate;
  double finalPlaybackRate = inputPlaybackRate / IdealAudioRate();
  uint32_t availableInOutputBuffer = WEBAUDIO_BLOCK_SIZE;
  uint32_t inputSamples, outputSamples;

  bool newChunk = aOutputChunk->IsNull();

  AudioNodeExternalInputStream::TrackMapEntry* trackMap = GetTrackMap(aInputTrack);

  AudioSegment& inputSegment = *aInputTrack->Get<AudioSegment>();
  SpeexResamplerState* resampler = trackMap->mResampler;

  AudioSegment tmpSegment;
  tmpSegment.AppendSlice(inputSegment, trackMap->mLastTick, aInputTrack->GetEnd());

  // printf("%lld, %lld\n", tmpSegment.GetDuration(), aEndTicks - trackMap->mLastTick);

  AudioSegment::ChunkIterator ci(tmpSegment);
  
  while (availableInOutputBuffer > 0) {
    if (ci.IsEnded()) {
      printf("Haha it did happen!\n");
      break;
    }

    const AudioChunk& currentChunk = *ci;

    // TODO: these are still ok
    // if (currentChunk.IsNull()) {
    //   ci.Next();
    //   continue;
    // }

    const uint32_t numberOfChannels = currentChunk.mChannelData.Length();
    uint32_t availableInChunk = currentChunk.GetDuration();

    if (newChunk) {
      AllocateAudioBlock(numberOfChannels, aOutputChunk);
    }

    if (availableInChunk < availableInOutputBuffer * finalPlaybackRate) {
      outputSamples = ceil(availableInChunk / finalPlaybackRate);
      inputSamples = availableInChunk;
    } else {
      inputSamples = availableInChunk; // ceil(availableInOutputBuffer * finalPlaybackRate);
      outputSamples = availableInOutputBuffer;
    }

    //printf("[%f %f], %d => %d\n", inputPlaybackRate, finalPlaybackRate, inputSamples, outputSamples);

    for (uint32_t i = 0; i < numberOfChannels; ++i) {
      // rawBuffer can hold either int16 or float audio data, depending on the format of
      // the input chunks.  We're using a raw byte buffer that can fit in a channel of
      // floats, which would be more than enough for int16.
      char rawBuffer[sizeof(float) * WEBAUDIO_BLOCK_SIZE];
      uint32_t currentOutputOffset = WEBAUDIO_BLOCK_SIZE - availableInOutputBuffer;

      uint32_t in = inputSamples;
      uint32_t out = outputSamples;
      float* finalData = static_cast<float*>(const_cast<void*>(aOutputChunk->mChannelData[i]));
      // if (currentChunk.mBufferFormat == AUDIO_FORMAT_S16) {
      //   int16_t* outputData = reinterpret_cast<int16_t*>(rawBuffer);
      //   int16_t* inputData = static_cast<int16_t*>(const_cast<void*>(currentChunk.mChannelData[i]));
      //   speex_resampler_process_int(resampler, i,
      //                               inputData, &in,
      //                               outputData, &out);

      //   for (uint32_t k = currentOutputOffset; k < currentOutputOffset + out; ++k) {
      //     if (newChunk) {
      //       finalData[k] = AudioSampleToFloat(outputData[k]);
      //     } else {
      //       finalData[k] += AudioSampleToFloat(outputData[k]);
      //     }
      //   }
      // } else {
        float* outputData = reinterpret_cast<float*>(rawBuffer);
        float* inputData = static_cast<float*>(const_cast<void*>(currentChunk.mChannelData[i]));
        speex_resampler_process_float(resampler, i,
                                      inputData, &in,
                                      outputData, &out);
        
        for (uint32_t k = currentOutputOffset; k < currentOutputOffset + out; ++k) {
          if (newChunk) {
            finalData[k] = outputData[k];
          } else {
            finalData[k] += outputData[k];
          }
        }
      // }

      if (i == currentChunk.mChannelData.Length() - 1) {
        // In the last iteration of the loop, remember how many more frames we need to produce.
        printf("input: %d => %d\toutput: %d => %d\n",
          inputSamples, in, outputSamples, out);
        availableInOutputBuffer -= out;
        trackMap->mLastTick += in;
      }
    }
  
    ci.Next();

  }

  // If outputChunk has room left over, fill it with 0's. 
  // if (newChunk && availableInOutputBuffer > 0) {
  //  printf("residual out: %d\n", availableInOutputBuffer);
  //   for (uint32_t i = 0; i < aOutputChunk->mChannelData.Length(); ++i) {
  //     float* finalData = static_cast<float*>(const_cast<void*>(aOutputChunk->mChannelData[i]));
  //     for (uint32_t k = WEBAUDIO_BLOCK_SIZE - availableInOutputBuffer - 1; k < WEBAUDIO_BLOCK_SIZE; ++k) {
  //       finalData[k] = 0;
  //     }
  //   }
  // }
}

void
AudioNodeExternalInputStream::ProduceOutput(GraphTime aFrom, GraphTime aTo)
{
  mLastChunk.SetNull(0);

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

  FinalizeProducedOutput();
}

}
