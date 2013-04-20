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

AudioNodeExternalInputStream::~AudioNodeExternalInputStream()
{
  MOZ_COUNT_DTOR(AudioNodeExternalInputStream);
}

void
AudioNodeExternalInputStream::ProduceOutput(GraphTime aFrom, GraphTime aTo)
{
  AudioChunk outputChunk;
  outputChunk.SetNull(WEBAUDIO_BLOCK_SIZE);

  AudioSegment outputSegment;

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
      TrackTicks ticks = endTicks - startTicks;

      StreamTime inputEnd = inputStream->GraphTimeToStreamTime(interval.mEnd);
      TrackTicks inputTrackEndPoint = TRACK_TICKS_MAX;

      if (inputTrack.IsEnded()) {
        TrackTicks inputEndTicks = inputTrack.TimeToTicksRoundDown(inputEnd);
        if (inputTrack.GetEnd() <= inputEndTicks) {
          inputTrackEndPoint = inputTrack.GetEnd();
          // *aOutputTrackFinished = true;
        }
      }

      if (interval.mInputIsBlocked) {
        // Maybe the input track ended?
        outputSegment.AppendNullData(ticks);
      }
      else {
        TrackTicks inputEndTicks = TimeToTicksRoundUp(rate, inputEnd);
        TrackTicks inputStartTicks = inputEndTicks - ticks;
        outputSegment.AppendSlice(*inputTrack.GetSegment(),
                             std::min(inputTrackEndPoint, inputStartTicks),
                             std::min(inputTrackEndPoint, inputEndTicks));

      }

    }
  }

  // if (from < to) {
  //   for (StreamBuffer::TrackIter tracks(mExternalStream->mBuffer, MediaSegment::AUDIO);
  //        !tracks.IsEnded(); tracks.Next()) {
      
  //     StreamBuffer::Track& externalTrack = *tracks;

  //     TrackRate rate = externalTrack.GetRate();

  //     TrackTicks startTicks = TimeToTicksRoundDown(rate, from);
  //     TrackTicks endTicks = TimeToTicksRoundDown(rate, to);
  //     printf("Consuming %lld samples\n", endTicks - startTicks);

  //     AudioSegment* externalSegment = externalTrack.Get<AudioSegment>();
  //     outputSegment.AppendSlice(*externalSegment, startTicks, endTicks);

  //     for (AudioSegment::ChunkIterator chunks(outputSegment);
  //          !chunks.IsEnded(); chunks.Next()) {
  //       AudioChunk& externalChunk = *chunks;
  //       if (!externalChunk.IsNull()) {

  //         if (outputChunk.IsNull()) {
  //           AllocateAudioBlock(externalChunk.mChannelData.Length(), &outputChunk);
  //           WriteZeroesToAudioBlock(&outputChunk, 0, WEBAUDIO_BLOCK_SIZE);
  //         }
  //         for (uint32_t c = 0; c < externalChunk.mChannelData.Length(); ++c) {
  //           AudioBlockAddChannelWithScale(
  //             static_cast<float*>(const_cast<void*>(externalChunk.mChannelData[c])),
  //             externalChunk.mVolume,
  //             static_cast<float*>(const_cast<void*>(outputChunk.mChannelData[c])));
  //         }
  //       }
  //     }
  //   }
  // }

  mLastChunk = outputChunk;
  FinalizeProducedOutput(&outputChunk);
}

}
