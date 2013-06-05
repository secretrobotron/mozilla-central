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

class AudioNodeExternalInputStream : public AudioNodeStream {
public:

  AudioNodeExternalInputStream(AudioNodeEngine* aEngine);
  ~AudioNodeExternalInputStream();

  virtual AudioNodeExternalInputStream* AsAudioNodeExternalInputStream() { return this; }
  AudioChunk* GetNextOutputChunk() { return &mNextOutputChunk; }
  virtual void ProduceOutput(GraphTime aFrom, GraphTime aTo);

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
  AudioChunk mNextOutputChunk;
  std::deque<AudioChunk> mOutputChunkQueue;
  nsTArray<TrackMapEntry> mTrackMap;

  void ConsumeInputData(const StreamBuffer::Track& aInputTrack,
                        AudioNodeExternalInputStream::ChunkMetaData& aChunkMetaData);
  TrackMapEntry* GetTrackMap(const StreamBuffer::Track& aTrack);
  bool PrepareOutputChunkQueue(AudioNodeExternalInputStream::ChunkMetaData& aChunkMetaData);
  void WriteToCurrentChunk(SpeexResamplerState* resampler,
                           TrackRate aInputRate,
                           const AudioChunk& aInputChunk,
                           const AudioChunk& aOutputChunk,
                           uint32_t aOutputOffset,
                           uint32_t& aActualIntputSamples,
                           uint32_t& aActualOutputSamples);
};

}

#endif /* MOZILLA_AUDIONODESTREAM_H_ */
