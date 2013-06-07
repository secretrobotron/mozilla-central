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

// Forward declaration for mResamplerMap
typedef struct SpeexResamplerState_ SpeexResamplerState;

namespace mozilla {

class AudioNodeExternalInputStream : public AudioNodeStream {
public:

  AudioNodeExternalInputStream(AudioNodeEngine* aEngine, TrackRate aSampleRate);
  ~AudioNodeExternalInputStream();

  virtual void ProduceOutput(GraphTime aFrom, GraphTime aTo) MOZ_OVERRIDE;

private:
  // For storing pointers and data about input tracks, like the last TrackTick which
  // was read, and the associated speex resampler.
  struct TrackMapEntry {
    SpeexResamplerState* mResampler;
    TrackTicks mLastInputTick;
    TrackID mTrackID;
    std::deque<AudioChunk> mChunkQueue;
    uint32_t mFrontChunkOffset;
  };

  // For easily pairing data about output chunks.
  struct ChunkMetaData {
    uint32_t mIndex;
    uint32_t mOffset;
  };

  std::deque<AudioChunk> mOutputChunkQueue;
  nsTArray<TrackMapEntry> mTrackMap;

  void BufferInputData(const StreamBuffer::Track& aInputTrack,
                       TrackMapEntry& aTrackMap);
  void ConsumeDataFromTrack(AudioChunk& aOutputChunk,
                            const StreamBuffer::Track& aInputTrack,
                            TrackMapEntry& aTrackMap);
  void CopyFromChunk(const AudioChunk& aInputChunk,
                     const AudioChunk& aOutputChunk,
                     uint32_t aInputOffset,
                     uint32_t aOutputOffset,
                     uint32_t& aActualOutputSamples);
  TrackMapEntry* GetTrackMap(const StreamBuffer::Track& aTrack);
  void MixTracks(const nsTArray<AudioChunk>& aOutputChunks,
                 AudioChunk& aDestinationChunk);
  void ResampleChunk(SpeexResamplerState* resampler,
                     TrackRate aInputRate,
                     const AudioChunk& aInputChunk,
                     const AudioChunk& aOutputChunk,
                     uint32_t aInputOffset,
                     uint32_t aOutputOffset,
                     uint32_t& aActualIntputSamples,
                     uint32_t& aActualOutputSamples);
};

}

#endif /* MOZILLA_AUDIONODESTREAM_H_ */
