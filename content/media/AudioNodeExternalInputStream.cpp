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

/**
 * An AudioNodeStream produces a single audio track with ID
 * AUDIO_NODE_STREAM_TRACK_ID. This track has rate IdealAudioRate().
 * Each chunk in the track is a single block of WEBAUDIO_BLOCK_SIZE samples.
 */
static const int AUDIO_NODE_STREAM_TRACK_ID = 1;

namespace mozilla {

AudioNodeExternalInputStream::AudioNodeExternalInputStream(AudioNodeEngine* aEngine)
    : ProcessedMediaStream(nullptr),
      mOutputIndex(0),
      mEngine(aEngine)
{
  mCurrentChunkMetaData.mIndex = 0;
  mCurrentChunkMetaData.mOffset = 0;
  mChunkQueue.AppendElement();
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

      for (uint32_t k = outputSamples; k < outputSamples + out; ++k) {
        finalData[k] += AudioSampleToFloat(outputData[k]);
      }
    }
    else {
      float* outputData = reinterpret_cast<float*>(rawBuffer);
      float* inputData = static_cast<float*>(const_cast<void*>(aInputChunk.mChannelData[i]));
      speex_resampler_process_float(aResampler, i,
                                    inputData, &in,
                                    outputData, &out);
      
      for (uint32_t k = outputSamples; k < outputSamples + out; ++k) {
        finalData[k] += outputData[k];
      }
    }

    if (i == aInputChunk.mChannelData.Length() - 1) {
      // In the last iteration of the loop, remember how many more frames we need to produce.
      // printf("input: %d => %d\toutput: %d => %d\n",
      //   inputSamples, in, outputSamples, out);
      aActualOutputSamples = out;
      aActualOutputSamples = in;
    }
  }
}

void
AudioNodeExternalInputStream::ConsumeInputData(StreamBuffer::Track* aInputTrack,
                                               TrackRate aInputRate,
                                               TrackTicks aStartTicks,
                                               TrackTicks aEndTicks)
{
  AudioNodeExternalInputStream::TrackMapEntry* trackMap = GetTrackMap(aInputTrack);

  AudioSegment& inputSegment = *aInputTrack->Get<AudioSegment>();
  SpeexResamplerState* resampler = trackMap->mResampler;

  AudioSegment tmpSegment;
  tmpSegment.AppendSlice(inputSegment, trackMap->mLastTick, aEndTicks);

  AudioSegment::ChunkIterator ci(tmpSegment);
  
  uint32_t chunkInputConsumed = 0;
  uint32_t in;
  uint32_t out;

  AudioNodeExternalInputStream::ChunkMetaData chunkMetaDataForTrack;
  chunkMetaDataForTrack.mIndex = mCurrentChunkMetaData.mIndex;
  chunkMetaDataForTrack.mOffset = mCurrentChunkMetaData.mOffset;

  while (!ci.IsEnded()) {
    const AudioChunk& currentInputChunk = *ci;

    // TODO: these are still ok
    if (currentInputChunk.IsNull()) {
      ci.Next();
      continue;
    }

    if (chunkMetaDataForTrack.mOffset == WEBAUDIO_BLOCK_SIZE) {
      if (chunkMetaDataForTrack.mIndex == MAX_QUEUE_LENGTH - 1) {
        break;
      }
      if (chunkMetaDataForTrack.mIndex == mChunkQueue.Length() - 1) {
        mChunkQueue.AppendElement();
      }
      ++chunkMetaDataForTrack.mIndex;
      chunkMetaDataForTrack.mOffset = 0;
    }

    if (mChunkQueue[chunkMetaDataForTrack.mIndex].IsNull()) {
      AllocateAudioBlock(currentInputChunk.mChannelData.Length(), &mChunkQueue[chunkMetaDataForTrack.mIndex]);
      WriteZeroesToAudioBlock(&mChunkQueue[chunkMetaDataForTrack.mIndex], 0, WEBAUDIO_BLOCK_SIZE);
    }

    WriteToCurrentChunk(resampler,
                        aInputRate,
                        currentInputChunk, mChunkQueue[chunkMetaDataForTrack.mIndex],
                        chunkMetaDataForTrack.mOffset,
                        in, out);

    chunkInputConsumed += in;
    trackMap->mLastTick += in;
    chunkMetaDataForTrack.mOffset += out;

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

        ConsumeInputData(&inputTrack, rate, inputStartTicks,
                         std::min(inputTrackEndPoint, inputEndTicks));
      }

    }
  }

  FinalizeProducedOutput();
}

StreamBuffer::Track*
AudioNodeExternalInputStream::EnsureTrack()
{
  StreamBuffer::Track* track = mBuffer.FindTrack(AUDIO_NODE_STREAM_TRACK_ID);
  if (!track) {
    nsAutoPtr<MediaSegment> segment(new AudioSegment());
    for (uint32_t j = 0; j < mListeners.Length(); ++j) {
      MediaStreamListener* l = mListeners[j];
      l->NotifyQueuedTrackChanges(Graph(), AUDIO_NODE_STREAM_TRACK_ID, IdealAudioRate(), 0,
                                  MediaStreamListener::TRACK_EVENT_CREATED,
                                  *segment);
    }
    track = &mBuffer.AddTrack(AUDIO_NODE_STREAM_TRACK_ID, IdealAudioRate(), 0, segment.forget());
  }
  return track;
}

void
AudioNodeExternalInputStream::FinalizeProducedOutput()
{
  StreamBuffer::Track* track = EnsureTrack();
  AudioSegment* segment = track->Get<AudioSegment>();

  segment->AppendNullData(mChunkQueue[mOutputIndex++].GetDuration());

  for (uint32_t j = 0; j < mListeners.Length(); ++j) {
    MediaStreamListener* l = mListeners[j];
    AudioChunk copyChunk = mChunkQueue[mOutputIndex];
    AudioSegment tmpSegment;
    tmpSegment.AppendAndConsumeChunk(&copyChunk);
    l->NotifyQueuedTrackChanges(Graph(), AUDIO_NODE_STREAM_TRACK_ID,
                                IdealAudioRate(), segment->GetDuration(), 0,
                                tmpSegment);
  }  
}

}
