/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-*/
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "MediaStreamGraphImpl.h"
#include "AudioNodeEngine.h"
#include "AudioNodeExternalInputStream.h"

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

uint32_t bloop = 0;

void
AudioNodeExternalInputStream::ProduceOutput(GraphTime aFrom, GraphTime aTo)
{
  AudioChunk outputChunk;
  outputChunk.SetNull(WEBAUDIO_BLOCK_SIZE);

  AudioSegment outputSegment;

  StreamTime from = mExternalStream->GraphTimeToStreamTime(aFrom);
  StreamTime to = mExternalStream->GraphTimeToStreamTime(aTo);

  if (from < to) {
    for (StreamBuffer::TrackIter tracks(mExternalStream->mBuffer, MediaSegment::AUDIO);
         !tracks.IsEnded(); tracks.Next()) {
      
      StreamBuffer::Track& externalTrack = *tracks;

      TrackRate rate = externalTrack.GetRate();

      TrackTicks startTicks = TimeToTicksRoundUp(rate, from);
      TrackTicks endTicks = TimeToTicksRoundUp(rate, to);

      AudioSegment* externalSegment = externalTrack.Get<AudioSegment>();
      outputSegment.AppendSlice(*externalSegment, startTicks, endTicks);

      for (AudioSegment::ChunkIterator chunks(outputSegment);
           !chunks.IsEnded(); chunks.Next()) {
        AudioChunk& externalChunk = *chunks;
        if (!externalChunk.IsNull()) {

          if (outputChunk.IsNull()) {
            AllocateAudioBlock(externalChunk.mChannelData.Length(), &outputChunk);
            WriteZeroesToAudioBlock(&outputChunk, 0, WEBAUDIO_BLOCK_SIZE);
          }
          for (uint32_t c = 0; c < externalChunk.mChannelData.Length(); ++c) {
            AudioBlockAddChannelWithScale((float*)(externalChunk.mChannelData[c]),
                                          externalChunk.mVolume,
                                          (float*)(outputChunk.mChannelData[c]));
          }
        }
      }
    }
  }

  mLastChunk = outputChunk;
  FinalizeProducedOutput(&outputChunk);
}

}
