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

  AudioNodeExternalInputStream(AudioNodeEngine* aEngine,
                               MediaStreamGraph::AudioNodeStreamKind aKind,
                               uint32_t aNumberOfInputChannels = 0);

  ~AudioNodeExternalInputStream();

  virtual void ProduceOutput(GraphTime aFrom, GraphTime aTo);

private:
  struct ResamplerMapEntry {
    SpeexResamplerState* mResampler;
    TrackID mTrackID;
  };

  nsTArray<ResamplerMapEntry> mResamplerMap;

  SpeexResamplerState* GetTrackResampler(StreamBuffer::Track* aTrack);
};

}

#endif /* MOZILLA_AUDIONODESTREAM_H_ */
