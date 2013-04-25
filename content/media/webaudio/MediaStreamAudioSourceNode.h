/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef MediaStreamAudioSourceNode_h_
#define MediaStreamAudioSourceNode_h_

#include "AudioNode.h"

namespace mozilla {
namespace dom {

class MediaStreamAudioSourceNode : public AudioNode
{
public:
  MediaStreamAudioSourceNode(AudioContext* aContext, const DOMMediaStream* aMediaTream);
  virtual ~MediaStreamAudioSourceNode();

  NS_DECL_ISUPPORTS_INHERITED
  NS_DECL_CYCLE_COLLECTION_CLASS_INHERITED(MediaStreamAudioSourceNode, AudioNode)

  virtual JSObject* WrapObject(JSContext* aCx, JSObject* aScope);

  virtual bool SupportsMediaStreams() const MOZ_OVERRIDE
  {
    return true;
  }

  virtual uint32_t NumberOfInputs() const MOZ_FINAL MOZ_OVERRIDE
  {
    return 1;
  }

private:
  nsRefPtr<MediaInputPort> mInputPort;
};

}
}

#endif

