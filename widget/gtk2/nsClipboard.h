/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* vim:expandtab:shiftwidth=4:tabstop=4:
 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __nsClipboard_h_
#define __nsClipboard_h_

#include "nsIClipboard.h"
#include "nsClipboardPrivacyHandler.h"
#include "nsAutoPtr.h"
#include <gtk/gtk.h>

class nsClipboard : public nsIClipboard,
                    public nsIObserver
{
public:
    nsClipboard();
    virtual ~nsClipboard();
    
    NS_DECL_ISUPPORTS
    
    NS_DECL_NSICLIPBOARD
    NS_DECL_NSIOBSERVER

    // Make sure we are initialized, called from the factory
    // constructor
    nsresult  Init              (void);

    // Someone requested the selection
    void   SelectionGetEvent    (GtkClipboard     *aGtkClipboard,
                                 GtkSelectionData *aSelectionData);
    void   SelectionClearEvent  (GtkClipboard     *aGtkClipboard);

private:
    // Utility methods
    static GdkAtom               GetSelectionAtom (PRInt32 aWhichClipboard);
    static GtkSelectionData     *GetTargets       (GdkAtom aWhichClipboard);

    // Save global clipboard content to gtk
    nsresult                     Store            (void);

    // Get our hands on the correct transferable, given a specific
    // clipboard
    nsITransferable             *GetTransferable  (PRInt32 aWhichClipboard);

    // Hang on to our owners and transferables so we can transfer data
    // when asked.
    nsCOMPtr<nsIClipboardOwner>  mSelectionOwner;
    nsCOMPtr<nsIClipboardOwner>  mGlobalOwner;
    nsCOMPtr<nsITransferable>    mSelectionTransferable;
    nsCOMPtr<nsITransferable>    mGlobalTransferable;
    nsRefPtr<nsClipboardPrivacyHandler> mPrivacyHandler;

};

#endif /* __nsClipboard_h_ */
