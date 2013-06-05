/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-*/
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef MOZILLA_AUDIONODEEXTERNALINPUTSTREAM_H_
#define MOZILLA_AUDIONODEEXTERNALINPUTSTREAM_H_

#include "MediaStreamGraph.h"
#include "AudioChannelFormat.h"
#include "AudioNodeEngine.h"
#include "AudioNodeStream.h"
#include "mozilla/dom/AudioParam.h"
#include <deque>

#ifdef PR_LOGGING
#define LOG(type, msg) PR_LOG(gMediaStreamGraphLog, type, msg)
#else
#define LOG(type, msg)
#endif

// Forward declaration of for Speex for mResamplerMap
typedef struct SpeexResamplerState_ SpeexResamplerState;

namespace mozilla {

class AudioNodeExternalInputStream : public ProcessedMediaStream {
public:

  AudioNodeExternalInputStream(AudioNodeEngine* aEngine);

  ~AudioNodeExternalInputStream();

  virtual void ProduceOutput(GraphTime aFrom, GraphTime aTo);

  virtual AudioNodeExternalInputStream* AsAudioNodeExternalInputStream() { return this; }

  // Any thread
  AudioNodeEngine* Engine() { return mEngine; }

  AudioChunk* GetNextOutputChunk() { return &mNextOutputChunk; }

private:
  struct TrackMapEntry {
    SpeexResamplerState* mResampler;
    TrackTicks mLastTick;
    TrackID mTrackID;
  };

  struct ChunkMetaData {
    uint32_t mIndex;
    uint32_t mOffset;
  };

  uint32_t mLastChunkOffset;

  std::deque<AudioChunk> mOutputChunkQueue;

  AudioChunk mNextOutputChunk;

  // The engine that will generate output for this node.
  nsAutoPtr<AudioNodeEngine> mEngine;

  StreamBuffer::Track* EnsureTrack();

  void FinalizeProducedOutput();

  nsTArray<TrackMapEntry> mTrackMap;

  TrackMapEntry* GetTrackMap(const StreamBuffer::Track& aTrack);

  void WriteToCurrentChunk(SpeexResamplerState* resampler,
                           TrackRate aInputRate,
                           const AudioChunk& aInputChunk,
                           const AudioChunk& aOutputChunk,
                           uint32_t aOutputOffset,
                           uint32_t& aActualIntputSamples,
                           uint32_t& aActualOutputSamples);

  void ConsumeInputData(const StreamBuffer::Track& aInputTrack,
                        AudioNodeExternalInputStream::ChunkMetaData& aChunkMetaData);

  bool PrepareOutputChunkQueue(AudioNodeExternalInputStream::ChunkMetaData& aChunkMetaData);
};

}

#endif /* MOZILLA_AUDIONODESTREAM_H_ */
