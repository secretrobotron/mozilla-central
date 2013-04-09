/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-*/
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef MOZILLA_AUDIONODESTREAM_H_
#define MOZILLA_AUDIONODESTREAM_H_

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

namespace mozilla {

namespace dom {

class AudioNodeExternalInputStream : public AudioNodeStream {
public:

  /**
   * Transfers ownership of aEngine to the new AudioNodeExternalInputStream.
   */
  AudioNodeExternalInputStream(AudioNodeEngine* aEngine,
                               MediaStreamGraph::AudioNodeStreamKind aKind)
    : AudioNodeStream(aEngine, aKind)
  {
  }
  ~AudioNodeExternalInputStream();

  virtual void ProduceOutput(GraphTime aFrom, GraphTime aTo);
};

}

#endif /* MOZILLA_AUDIONODESTREAM_H_ */
