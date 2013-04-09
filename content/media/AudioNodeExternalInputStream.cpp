/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-*/
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "MediaStreamGraphImpl.h"
#include "AudioNodeExternalInputStream.h"

using namespace mozilla::dom;

namespace mozilla {

AudioNodeExternalInputStream::~AudioNodeExternalInputStream()
{
  MOZ_COUNT_DTOR(AudioNodeExternalInputStream);
}

void
AudioNodeStream::ProduceOutput(GraphTime aFrom, GraphTime aTo)
{
  StreamBuffer::Track* track = EnsureTrack();

  AudioChunk outputChunk;
  AudioSegment* segment = track->Get<AudioSegment>();

  outputChunk.SetNull(0);

  AudioChunk tmpChunk;
  AudioChunk* inputChunk = ObtainInputBlock(&tmpChunk, aFrom, aTo);
  bool finished = false;
  //mEngine->ProduceAudioBlock(this, *inputChunk, &outputChunk, &finished);
  if (finished) {
    FinishOutput();
  }

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
