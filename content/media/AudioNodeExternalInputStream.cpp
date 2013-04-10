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

void
AudioNodeExternalInputStream::ProduceOutput(GraphTime aFrom, GraphTime aTo)
{
  // uint32_t outputChannels = 0;

  StreamBuffer::Track* track = EnsureTrack();
  AudioSegment* segment = track->Get<AudioSegment>();

  AudioChunk outputChunk;
  outputChunk.SetNull(WEBAUDIO_BLOCK_SIZE);

  // AudioSegment* inputSegment;

  // StreamTime from = mExternalStream->GraphTimeToStreamTime(aFrom);
  // StreamTime to = mExternalStream->GraphTimeToStreamTime(aTo);

  // mExternalStream->mBuffer.FindTrack(2)->mSegment

  for (StreamBuffer::TrackIter tracks(mExternalStream->mBuffer);
       !tracks.IsEnded(); tracks.Next()) {
    StreamBuffer::Track& track = *tracks;
    AudioSegment* externalSegment = track.Get<AudioSegment>();
    for (AudioSegment::ChunkIterator chunks(*externalSegment);
         !chunks.IsEnded(); chunks.Next()) {
      AudioChunk& externalChunk = *chunks;
      if (!externalChunk.IsNull()) {

        if (outputChunk.IsNull()) {
          AllocateAudioBlock(externalChunk.mChannelData.Length(), &outputChunk);
          WriteZeroesToAudioBlock(&outputChunk, 0, WEBAUDIO_BLOCK_SIZE);
        }
        for (uint32_t c = 0; c < externalChunk.mChannelData.Length(); ++c) {
          AudioBlockAddChannelWithScale((float*)(externalChunk.mChannelData[c]),
                                        1.0f,
                                        (float*)(outputChunk.mChannelData[c]));
        }
      }
    }
  }


  // for (StreamBuffer::TrackIter tracks(mExternalStream->mBuffer, MediaSegment::AUDIO);
  //  !tracks.IsEnded(); tracks.Next()) {
  //   AudioSegment* audio = tracks->Get<AudioSegment>();
  //   AudioSegment::ChunkIterator chunkIter(*audio);

  //   while (!chunkIter.IsEnded()) {
  //     AudioChunk& inputChunk = *chunkIter;

  //     if (inputChunk.IsNull()) {
  //       continue;
  //     }

  //     if (outputChannels == 0) {
  //       AllocateAudioBlock(inputChunk.mChannelData.Length(), &outputChunk);
  //     }
  //     break;
  //   }

  //   // inputSegment.AppendSlice(*audio, from, to);
  //   break;
  // }


// +    AudioNodeStream* a = s->AsAudioNodeStream();^M
// +    if (!a) {^M
// +      StreamTime from = s->GraphTimeToStreamTime(aFrom);^M
// +      StreamTime to = s->GraphTimeToStreamTime(aTo);^M
// +^M
// +      AudioSegment inputSegment;^M
// +^M
// +      for (StreamBuffer::TrackIter tracks(s->mBuffer, MediaSegment::AUDIO);^M
// +       !tracks.IsEnded(); tracks.Next()) {^M
// +        AudioSegment* audio  = tracks->Get<AudioSegment>();^M
// +        inputSegment.AppendSlice(*audio, from, to);^M
// +        AudioSegment::ChunkIterator chunkIter(inputSegment);^M
// +        while (!chunkIter.IsEnded()) {^M
// +          inputChunks.AppendElement(*chunkIter);^M
// +        }^M
// +      }^M
// +^M
// +^M
// +      // AudioSegment inputSegment;^M
// +      // for (uint32_t i = 0; i < s->mAudioOutputStreams.Length(); ++i) {^M
// +      //   MediaStream::AudioOutputStream& audioOutput = s->mAudioOutputStreams[i];^M
// +      //   StreamBuffer::Track* track = s->mBuffer.FindTrack(audioOutput.mTrackID);^M
// +      //   AudioSegment* audio  = track->Get<AudioSegment>();^M
// +^M
// +      //   inputSegment.AppendSlice(*audio, from, to);^M
// +      //       AudioSegment::ChunkIterator iter(inputSegment);^M
// +      //       while (!iter.IsEnded()) {^M
// +      //     inputChunks.AppendElement(*iter);^M
// +      //       }^M
// +      // }^M
// +      break;^M




  mLastChunk = outputChunk;
  
  if (mKind == MediaStreamGraph::EXTERNAL_STREAM) {
    segment->AppendAndConsumeChunk(&outputChunk);
  } else {
    segment->AppendNullData(outputChunk.GetDuration());
  }

  for (uint32_t j = 0; j < mListeners.Length(); ++j) {
    MediaStreamListener* l = mListeners[j];
    AudioChunk copyChunk = outputChunk;
    AudioSegment tmpSegment;
    tmpSegment.AppendAndConsumeChunk(&copyChunk);
    l->NotifyQueuedTrackChanges(Graph(), AUDIO_NODE_STREAM_TRACK_ID,
                                IdealAudioRate(), segment->GetDuration(), 0,
                                tmpSegment);
  }
}

}
