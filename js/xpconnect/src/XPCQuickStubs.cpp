/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "mozilla/Util.h"

#include "jsapi.h"
#include "jsatom.h"
#include "jsfriendapi.h"
#include "nsCOMPtr.h"
#include "xpcprivate.h"
#include "XPCInlines.h"
#include "XPCQuickStubs.h"
#include "XPCWrapper.h"
#include "mozilla/dom/BindingUtils.h"

using namespace mozilla;

static inline QITableEntry *
GetOffsets(nsISupports *identity, XPCWrappedNativeProto* proto)
{
    QITableEntry* offsets = proto ? proto->GetOffsets() : nsnull;
    if (!offsets) {
        static NS_DEFINE_IID(kThisPtrOffsetsSID, NS_THISPTROFFSETS_SID);
        identity->QueryInterface(kThisPtrOffsetsSID, (void**)&offsets);
    }
    return offsets;
}

static inline QITableEntry *
GetOffsetsFromSlimWrapper(JSObject *obj)
{
    NS_ASSERTION(IS_SLIM_WRAPPER(obj), "What kind of object is this?");
    return GetOffsets(static_cast<nsISupports*>(xpc_GetJSPrivate(obj)),
                      GetSlimWrapperProto(obj));
}

static const xpc_qsHashEntry *
LookupEntry(PRUint32 tableSize, const xpc_qsHashEntry *table, const nsID &iid)
{
    size_t i;
    const xpc_qsHashEntry *p;

    i = iid.m0 % tableSize;
    do
    {
        p = table + i;
        if (p->iid.Equals(iid))
            return p;
        i = p->chain;
    } while (i != XPC_QS_NULL_INDEX);
    return nsnull;
}

static const xpc_qsHashEntry *
LookupInterfaceOrAncestor(PRUint32 tableSize, const xpc_qsHashEntry *table,
                          const nsID &iid)
{
    const xpc_qsHashEntry *entry = LookupEntry(tableSize, table, iid);
    if (!entry) {
        /*
         * On a miss, we have to search for every interface the object
         * supports, including ancestors.
         */
        nsCOMPtr<nsIInterfaceInfo> info;
        if (NS_FAILED(nsXPConnect::GetXPConnect()->GetInfoForIID(&iid, getter_AddRefs(info))))
            return nsnull;

        const nsIID *piid;
        for (;;) {
            nsCOMPtr<nsIInterfaceInfo> parent;
            if (NS_FAILED(info->GetParent(getter_AddRefs(parent))) ||
                !parent ||
                NS_FAILED(parent->GetIIDShared(&piid))) {
                break;
            }
            entry = LookupEntry(tableSize, table, *piid);
            if (entry)
                break;
            info.swap(parent);
        }
    }
    return entry;
}

static void
PointerFinalize(JSFreeOp *fop, JSObject *obj)
{
    JSPropertyOp *popp = static_cast<JSPropertyOp *>(JS_GetPrivate(obj));
    delete popp;
}

JSClass
PointerHolderClass = {
    "Pointer", JSCLASS_HAS_PRIVATE,
    JS_PropertyStub, JS_PropertyStub, JS_PropertyStub, JS_StrictPropertyStub,
    JS_EnumerateStub, JS_ResolveStub, JS_ConvertStub, PointerFinalize
};

static JSBool
ReifyPropertyOps(JSContext *cx, JSObject *obj, jsid id, unsigned orig_attrs,
                 JSPropertyOp getter, JSStrictPropertyOp setter,
                 JSObject **getterobjp, JSObject **setterobjp)
{
    // Generate both getter and setter and stash them in the prototype.
    jsval roots[2] = { JSVAL_NULL, JSVAL_NULL };
    JS::AutoArrayRooter tvr(cx, ArrayLength(roots), roots);

    unsigned attrs = JSPROP_SHARED | (orig_attrs & JSPROP_ENUMERATE);
    JSObject *getterobj;
    if (getter) {
        getterobj = GeneratePropertyOp(cx, obj, id, 0, getter);
        if (!getterobj)
            return false;
        roots[0] = OBJECT_TO_JSVAL(getterobj);
        attrs |= JSPROP_GETTER;
    } else
        getterobj = nsnull;

    JSObject *setterobj;
    if (setter) {
        setterobj = GeneratePropertyOp(cx, obj, id, 1, setter);
        if (!setterobj)
            return false;
        roots[1] = OBJECT_TO_JSVAL(setterobj);
        attrs |= JSPROP_SETTER;
    } else
        setterobj = nsnull;

    if (getterobjp)
        *getterobjp = getterobj;
    if (setterobjp)
        *setterobjp = setterobj;
    return JS_DefinePropertyById(cx, obj, id, JSVAL_VOID,
                                 JS_DATA_TO_FUNC_PTR(JSPropertyOp, getterobj),
                                 JS_DATA_TO_FUNC_PTR(JSStrictPropertyOp, setterobj),
                                 attrs);
}

static JSBool
LookupGetterOrSetter(JSContext *cx, JSBool wantGetter, unsigned argc, jsval *vp)
{
    XPC_QS_ASSERT_CONTEXT_OK(cx);

    if (argc == 0) {
        JS_SET_RVAL(cx, vp, JSVAL_VOID);
        return true;
    }

    JSObject *obj = JS_THIS_OBJECT(cx, vp);
    if (!obj)
        return false;

    jsval idval = JS_ARGV(cx, vp)[0];
    jsid id;
    JSPropertyDescriptor desc;
    if (!JS_ValueToId(cx, idval, &id) ||
        !JS_GetPropertyDescriptorById(cx, obj, id, JSRESOLVE_QUALIFIED, &desc))
        return false;

    // No property at all means no getters or setters possible.
    if (!desc.obj) {
        JS_SET_RVAL(cx, vp, JSVAL_VOID);
        return true;
    }

    // Inline obj_lookup[GS]etter here.
    if (wantGetter) {
        if (desc.attrs & JSPROP_GETTER) {
            JS_SET_RVAL(cx, vp,
                        OBJECT_TO_JSVAL(JS_FUNC_TO_DATA_PTR(JSObject *, desc.getter)));
            return true;
        }
    } else {
        if (desc.attrs & JSPROP_SETTER) {
            JS_SET_RVAL(cx, vp,
                        OBJECT_TO_JSVAL(JS_FUNC_TO_DATA_PTR(JSObject *, desc.setter)));
            return true;
        }
    }

    // Since XPConnect doesn't use JSPropertyOps in any other contexts,
    // ensuring that we have an XPConnect prototype object ensures that
    // we are only going to expose quickstubbed properties to script.
    // Also be careful not to overwrite existing properties!

    if (!JSID_IS_STRING(id) ||
        !IS_PROTO_CLASS(js::GetObjectClass(desc.obj)) ||
        (desc.attrs & (JSPROP_GETTER | JSPROP_SETTER)) ||
        !(desc.getter || desc.setter) ||
        desc.setter == js::GetObjectJSClass(desc.obj)->setProperty) {
        JS_SET_RVAL(cx, vp, JSVAL_VOID);
        return true;
    }

    JSObject *getterobj, *setterobj;
    if (!ReifyPropertyOps(cx, desc.obj, id, desc.attrs, desc.getter, desc.setter,
                          &getterobj, &setterobj)) {
        return false;
    }

    JSObject *wantedobj = wantGetter ? getterobj : setterobj;
    jsval v = wantedobj ? OBJECT_TO_JSVAL(wantedobj) : JSVAL_VOID;
    JS_SET_RVAL(cx, vp, v);
    return true;
}

static JSBool
SharedLookupGetter(JSContext *cx, unsigned argc, jsval *vp)
{
    return LookupGetterOrSetter(cx, true, argc, vp);
}

static JSBool
SharedLookupSetter(JSContext *cx, unsigned argc, jsval *vp)
{
    return LookupGetterOrSetter(cx, false, argc, vp);
}

static JSBool
DefineGetterOrSetter(JSContext *cx, unsigned argc, JSBool wantGetter, jsval *vp)
{
    unsigned attrs;
    JSBool found;
    JSPropertyOp getter;
    JSStrictPropertyOp setter;
    JSObject *obj2;
    jsval v;
    jsid id;

    XPC_QS_ASSERT_CONTEXT_OK(cx);
    JSObject *obj = JS_THIS_OBJECT(cx, vp);
    if (!obj)
        return false;
    JSNative forward = wantGetter ? js::obj_defineGetter : js::obj_defineSetter;
    jsval idval = (argc >= 1) ? JS_ARGV(cx, vp)[0] : JSVAL_VOID;
    if (!JSVAL_IS_STRING(idval))
        return forward(cx, argc, vp);

    if (!JS_ValueToId(cx, idval, &id) ||
        !JS_LookupPropertyWithFlagsById(cx, obj, id,
                                        JSRESOLVE_QUALIFIED, &obj2, &v) ||
        (obj2 &&
         !JS_GetPropertyAttrsGetterAndSetterById(cx, obj2, id, &attrs,
                                                 &found, &getter, &setter)))
        return false;

    // The property didn't exist, already has a getter or setter, or is not
    // our property, then just forward now.
    if (!obj2 ||
        (attrs & (JSPROP_GETTER | JSPROP_SETTER)) ||
        !(getter || setter) ||
        !IS_PROTO_CLASS(js::GetObjectClass(obj2)))
        return forward(cx, argc, vp);

    // Reify the getter and setter...
    if (!ReifyPropertyOps(cx, obj2, id, attrs, getter, setter, nsnull, nsnull))
        return false;

    return forward(cx, argc, vp);
}

static JSBool
SharedDefineGetter(JSContext *cx, unsigned argc, jsval *vp)
{
    return DefineGetterOrSetter(cx, argc, true, vp);
}

static JSBool
SharedDefineSetter(JSContext *cx, unsigned argc, jsval *vp)
{
    return DefineGetterOrSetter(cx, argc, false, vp);
}


JSBool
xpc_qsDefineQuickStubs(JSContext *cx, JSObject *proto, unsigned flags,
                       PRUint32 ifacec, const nsIID **interfaces,
                       PRUint32 tableSize, const xpc_qsHashEntry *table,
                       const xpc_qsPropertySpec *propspecs,
                       const xpc_qsFunctionSpec *funcspecs,
                       const char *stringTable)
{
    /*
     * Walk interfaces in reverse order to behave like XPConnect when a
     * feature is defined in more than one of the interfaces.
     *
     * XPCNativeSet::FindMethod returns the first matching feature it finds,
     * searching the interfaces forward.  Here, definitions toward the
     * front of 'interfaces' overwrite those toward the back.
     */
    bool definedProperty = false;
    for (uint32_t i = ifacec; i-- != 0;) {
        const nsID &iid = *interfaces[i];
        const xpc_qsHashEntry *entry =
            LookupInterfaceOrAncestor(tableSize, table, iid);

        if (entry) {
            for (;;) {
                // Define quick stubs for attributes.
                const xpc_qsPropertySpec *ps = propspecs + entry->prop_index;
                const xpc_qsPropertySpec *ps_end = ps + entry->n_props;
                for ( ; ps < ps_end; ++ps) {
                    definedProperty = true;
                    if (!JS_DefineProperty(cx, proto,
                                           stringTable + ps->name_index,
                                           JSVAL_VOID, ps->getter, ps->setter,
                                           flags | JSPROP_SHARED)) 
                        return false;
                }

                // Define quick stubs for methods.
                const xpc_qsFunctionSpec *fs = funcspecs + entry->func_index;
                const xpc_qsFunctionSpec *fs_end = fs + entry->n_funcs;
                for ( ; fs < fs_end; ++fs) {
                    if (!JS_DefineFunction(cx, proto,
                                           stringTable + fs->name_index,
                                           reinterpret_cast<JSNative>(fs->native),
                                           fs->arity, flags))
                        return false;
                }

                // Next.
                size_t j = entry->parentInterface;
                if (j == XPC_QS_NULL_INDEX)
                    break;
                entry = table + j;
            }
        }
    }

    static JSFunctionSpec getterfns[] = {
        JS_FN("__lookupGetter__", SharedLookupGetter, 1, 0),
        JS_FN("__lookupSetter__", SharedLookupSetter, 1, 0),
        JS_FN("__defineGetter__", SharedDefineGetter, 2, 0),
        JS_FN("__defineSetter__", SharedDefineSetter, 2, 0),
        JS_FS_END
    };

    if (definedProperty && !JS_DefineFunctions(cx, proto, getterfns))
        return false;

    return true;
}

JSBool
xpc_qsThrow(JSContext *cx, nsresult rv)
{
    XPCThrower::Throw(rv, cx);
    return false;
}

/**
 * Get the interface name and member name (for error messages).
 *
 * We could instead have each quick stub pass its name to the error-handling
 * functions, as that name is statically known.  But that would be redundant;
 * the information is handy at runtime anyway.  Also, this code often produces
 * a more specific error message, e.g. "[nsIDOMHTMLDocument.appendChild]"
 * rather than "[nsIDOMNode.appendChild]".
 */
static void
GetMemberInfo(JSObject *obj, jsid memberId, const char **ifaceName)
{
    // Get the interface name.  From DefinePropertyIfFound (in
    // xpcwrappednativejsops.cpp) and XPCThrower::Verbosify.
    //
    // We could instead make the quick stub could pass in its interface name,
    // but this code often produces a more specific error message, e.g.
    *ifaceName = "Unknown";

    NS_ASSERTION(IS_WRAPPER_CLASS(js::GetObjectClass(obj)) ||
                 js::GetObjectClass(obj) == &XPC_WN_Tearoff_JSClass,
                 "obj must be a wrapper");
    XPCWrappedNativeProto *proto;
    if (IS_SLIM_WRAPPER(obj)) {
        proto = GetSlimWrapperProto(obj);
    } else {
        XPCWrappedNative *wrapper = (XPCWrappedNative *) js::GetObjectPrivate(obj);
        proto = wrapper->GetProto();
    }
    if (proto) {
        XPCNativeSet *set = proto->GetSet();
        if (set) {
            XPCNativeMember *member;
            XPCNativeInterface *iface;

            if (set->FindMember(memberId, &member, &iface))
                *ifaceName = iface->GetNameString();
        }
    }
}

static void
GetMethodInfo(JSContext *cx, jsval *vp, const char **ifaceNamep, jsid *memberIdp)
{
    JSObject *funobj = JSVAL_TO_OBJECT(JS_CALLEE(cx, vp));
    NS_ASSERTION(JS_ObjectIsFunction(cx, funobj),
                 "JSNative callee should be Function object");
    JSString *str = JS_GetFunctionId(JS_GetObjectFunction(funobj));
    jsid methodId = str ? INTERNED_STRING_TO_JSID(cx, str) : JSID_VOID;
    GetMemberInfo(JSVAL_TO_OBJECT(vp[1]), methodId, ifaceNamep);
    *memberIdp = methodId;
}

static bool
ThrowCallFailed(JSContext *cx, nsresult rv,
                const char *ifaceName, jsid memberId, const char *memberName)
{
    /* Only one of memberId or memberName should be given. */
    JS_ASSERT(JSID_IS_VOID(memberId) != !memberName);

    // From XPCThrower::ThrowBadResult.
    char* sz;
    const char* format;
    const char* name;

    /*
     *  If there is a pending exception when the native call returns and
     *  it has the same error result as returned by the native call, then
     *  the native call may be passing through an error from a previous JS
     *  call. So we'll just throw that exception into our JS.
     */
    if (XPCThrower::CheckForPendingException(rv, cx))
        return false;

    // else...

    if (!nsXPCException::NameAndFormatForNSResult(NS_ERROR_XPC_NATIVE_RETURNED_FAILURE, nsnull, &format) ||
        !format) {
        format = "";
    }

    JSAutoByteString memberNameBytes;
    if (!memberName) {
        memberName = JSID_IS_STRING(memberId)
                     ? memberNameBytes.encode(cx, JSID_TO_STRING(memberId))
                     : "unknown";
    }
    if (nsXPCException::NameAndFormatForNSResult(rv, &name, nsnull)
        && name) {
        sz = JS_smprintf("%s 0x%x (%s) [%s.%s]",
                         format, rv, name, ifaceName, memberName);
    } else {
        sz = JS_smprintf("%s 0x%x [%s.%s]",
                         format, rv, ifaceName, memberName);
    }

    XPCThrower::BuildAndThrowException(cx, rv, sz);

    if (sz)
        JS_smprintf_free(sz);

    return false;
}

JSBool
xpc_qsThrowGetterSetterFailed(JSContext *cx, nsresult rv, JSObject *obj,
                              jsid memberId)
{
    const char *ifaceName;
    GetMemberInfo(obj, memberId, &ifaceName);
    return ThrowCallFailed(cx, rv, ifaceName, memberId, NULL);
}

JSBool
xpc_qsThrowMethodFailed(JSContext *cx, nsresult rv, jsval *vp)
{
    const char *ifaceName;
    jsid memberId;
    GetMethodInfo(cx, vp, &ifaceName, &memberId);
    return ThrowCallFailed(cx, rv, ifaceName, memberId, NULL);
}

JSBool
xpc_qsThrowMethodFailedWithCcx(XPCCallContext &ccx, nsresult rv)
{
    ThrowBadResult(rv, ccx);
    return false;
}

bool
xpc_qsThrowMethodFailedWithDetails(JSContext *cx, nsresult rv,
                                   const char *ifaceName,
                                   const char *memberName)
{
    return ThrowCallFailed(cx, rv, ifaceName, JSID_VOID, memberName);
}

static void
ThrowBadArg(JSContext *cx, nsresult rv, const char *ifaceName,
            jsid memberId, const char *memberName, unsigned paramnum)
{
    /* Only one memberId or memberName should be given. */
    JS_ASSERT(JSID_IS_VOID(memberId) != !memberName);

    // From XPCThrower::ThrowBadParam.
    char* sz;
    const char* format;

    if (!nsXPCException::NameAndFormatForNSResult(rv, nsnull, &format))
        format = "";

    JSAutoByteString memberNameBytes;
    if (!memberName) {
        memberName = JSID_IS_STRING(memberId)
                     ? memberNameBytes.encode(cx, JSID_TO_STRING(memberId))
                     : "unknown";
    }
    sz = JS_smprintf("%s arg %u [%s.%s]",
                     format, (unsigned int) paramnum, ifaceName, memberName);

    XPCThrower::BuildAndThrowException(cx, rv, sz);

    if (sz)
        JS_smprintf_free(sz);
}

void
xpc_qsThrowBadArg(JSContext *cx, nsresult rv, jsval *vp, unsigned paramnum)
{
    const char *ifaceName;
    jsid memberId;
    GetMethodInfo(cx, vp, &ifaceName, &memberId);
    ThrowBadArg(cx, rv, ifaceName, memberId, NULL, paramnum);
}

void
xpc_qsThrowBadArgWithCcx(XPCCallContext &ccx, nsresult rv, unsigned paramnum)
{
    XPCThrower::ThrowBadParam(rv, paramnum, ccx);
}

void
xpc_qsThrowBadArgWithDetails(JSContext *cx, nsresult rv, unsigned paramnum,
                             const char *ifaceName, const char *memberName)
{
    ThrowBadArg(cx, rv, ifaceName, JSID_VOID, memberName, paramnum);
}

void
xpc_qsThrowBadSetterValue(JSContext *cx, nsresult rv,
                          JSObject *obj, jsid propId)
{
    const char *ifaceName;
    GetMemberInfo(obj, propId, &ifaceName);
    ThrowBadArg(cx, rv, ifaceName, propId, NULL, 0);
}

JSBool
xpc_qsGetterOnlyPropertyStub(JSContext *cx, JSHandleObject obj, JSHandleId id, JSBool strict, jsval *vp)
{
    return JS_ReportErrorFlagsAndNumber(cx,
                                        JSREPORT_WARNING | JSREPORT_STRICT |
                                        JSREPORT_STRICT_MODE_ERROR,
                                        js_GetErrorMessage, NULL,
                                        JSMSG_GETTER_ONLY);
}

xpc_qsDOMString::xpc_qsDOMString(JSContext *cx, jsval v, jsval *pval,
                                 StringificationBehavior nullBehavior,
                                 StringificationBehavior undefinedBehavior)
{
    typedef implementation_type::char_traits traits;
    // From the T_DOMSTRING case in XPCConvert::JSData2Native.
    JSString *s = InitOrStringify<traits>(cx, v, pval, nullBehavior,
                                          undefinedBehavior);
    if (!s)
        return;

    size_t len;
    const jschar *chars = JS_GetStringCharsZAndLength(cx, s, &len);
    if (!chars) {
        mValid = false;
        return;
    }

    new(mBuf) implementation_type(chars, len);
    mValid = true;
}

xpc_qsACString::xpc_qsACString(JSContext *cx, jsval v, jsval *pval,
                               StringificationBehavior nullBehavior,
                               StringificationBehavior undefinedBehavior)
{
    typedef implementation_type::char_traits traits;
    // From the T_CSTRING case in XPCConvert::JSData2Native.
    JSString *s = InitOrStringify<traits>(cx, v, pval, nullBehavior,
                                          undefinedBehavior);
    if (!s)
        return;

    size_t len = JS_GetStringEncodingLength(cx, s);
    if (len == size_t(-1)) {
        mValid = false;
        return;
    }

    JSAutoByteString bytes(cx, s);
    if (!bytes) {
        mValid = false;
        return;
    }

    new(mBuf) implementation_type(bytes.ptr(), len);
    mValid = true;
}

xpc_qsAUTF8String::xpc_qsAUTF8String(JSContext *cx, jsval v, jsval *pval)
{
    typedef nsCharTraits<PRUnichar> traits;
    // From the T_UTF8STRING  case in XPCConvert::JSData2Native.
    JSString *s = InitOrStringify<traits>(cx, v, pval, eNull, eNull);
    if (!s)
        return;

    size_t len;
    const PRUnichar *chars = JS_GetStringCharsZAndLength(cx, s, &len);
    if (!chars) {
        mValid = false;
        return;
    }

    new(mBuf) implementation_type(chars, len);
    mValid = true;
}

static nsresult
getNative(nsISupports *idobj,
          QITableEntry* entries,
          JSObject *obj,
          const nsIID &iid,
          void **ppThis,
          nsISupports **pThisRef,
          jsval *vp)
{
    // Try using the QITableEntry to avoid the extra AddRef and Release.
    if (entries) {
        for (QITableEntry* e = entries; e->iid; e++) {
            if (e->iid->Equals(iid)) {
                *ppThis = (char*) idobj + e->offset - entries[0].offset;
                *vp = OBJECT_TO_JSVAL(obj);
                *pThisRef = nsnull;
                return NS_OK;
            }
        }
    }

    nsresult rv = idobj->QueryInterface(iid, ppThis);
    *pThisRef = static_cast<nsISupports*>(*ppThis);
    if (NS_SUCCEEDED(rv))
        *vp = OBJECT_TO_JSVAL(obj);
    return rv;
}

inline nsresult
getNativeFromWrapper(JSContext *cx,
                     XPCWrappedNative *wrapper,
                     const nsIID &iid,
                     void **ppThis,
                     nsISupports **pThisRef,
                     jsval *vp)
{
    return getNative(wrapper->GetIdentityObject(), wrapper->GetOffsets(),
                     wrapper->GetFlatJSObject(), iid, ppThis, pThisRef, vp);
}


nsresult
getWrapper(JSContext *cx,
           JSObject *obj,
           XPCWrappedNative **wrapper,
           JSObject **cur,
           XPCWrappedNativeTearOff **tearoff)
{
    // We can have at most three layers in need of unwrapping here:
    // * A (possible) security wrapper
    // * A (possible) Xray waiver
    // * A (possible) outer window
    //
    // If we pass stopAtOuter == false, we can handle all three with one call
    // to XPCWrapper::Unwrap.
    if (js::IsWrapper(obj)) {
        obj = XPCWrapper::Unwrap(cx, obj, false);

        // The safe unwrap might have failed for SCRIPT_ACCESS_ONLY objects. If it
        // didn't fail though, we should be done with wrappers.
        if (!obj)
            return NS_ERROR_XPC_SECURITY_MANAGER_VETO;
        MOZ_ASSERT(!js::IsWrapper(obj));
    }

    // Start with sane values.
    *wrapper = nsnull;
    *cur = nsnull;
    *tearoff = nsnull;

    js::Class* clasp = js::GetObjectClass(obj);
    if (dom::IsDOMClass(clasp) ||
        dom::binding::instanceIsProxy(obj)) {
        *cur = obj;

        return NS_OK;
    }

    // Handle tearoffs.
    //
    // If |obj| is of the tearoff class, that means we're dealing with a JS
    // object reflection of a particular interface (ie, |foo.nsIBar|). These
    // JS objects are parented to their wrapper, so we snag the tearoff object
    // along the way (if desired), and then set |obj| to its parent.
    if (clasp == &XPC_WN_Tearoff_JSClass) {
        *tearoff = (XPCWrappedNativeTearOff*) js::GetObjectPrivate(obj);
        obj = js::GetObjectParent(obj);
    }

    // If we've got a WN or slim wrapper, store things the way callers expect.
    // Otherwise, leave things null and return.
    if (IS_WRAPPER_CLASS(clasp)) {
        if (IS_WN_WRAPPER_OBJECT(obj))
            *wrapper = (XPCWrappedNative*) js::GetObjectPrivate(obj);
        else
            *cur = obj;
    }

    return NS_OK;
}

nsresult
castNative(JSContext *cx,
           XPCWrappedNative *wrapper,
           JSObject *cur,
           XPCWrappedNativeTearOff *tearoff,
           const nsIID &iid,
           void **ppThis,
           nsISupports **pThisRef,
           jsval *vp,
           XPCLazyCallContext *lccx)
{
    if (wrapper) {
        nsresult rv = getNativeFromWrapper(cx,wrapper, iid, ppThis, pThisRef,
                                           vp);

        if (lccx && NS_SUCCEEDED(rv))
            lccx->SetWrapper(wrapper, tearoff);

        if (rv != NS_ERROR_NO_INTERFACE)
            return rv;
    } else if (cur) {
        nsISupports *native;
        QITableEntry *entries;
        js::Class* clasp = js::GetObjectClass(cur);
        if (dom::IsDOMClass(clasp)) {
            native = dom::UnwrapDOMObject<nsISupports>(cur);
            entries = nsnull;
        } else if (dom::binding::instanceIsProxy(cur)) {
            native = static_cast<nsISupports*>(js::GetProxyPrivate(cur).toPrivate());
            entries = nsnull;
        } else if (IS_WRAPPER_CLASS(clasp) && IS_SLIM_WRAPPER_OBJECT(cur)) {
            native = static_cast<nsISupports*>(xpc_GetJSPrivate(cur));
            entries = GetOffsetsFromSlimWrapper(cur);
        } else {
            MOZ_NOT_REACHED("what kind of wrapper is this?");
        }

        if (NS_SUCCEEDED(getNative(native, entries, cur, iid, ppThis, pThisRef, vp))) {
            if (lccx) {
                // This only matters for unwrapping of this objects, so we
                // shouldn't end up here for the new DOM bindings.
                NS_ABORT_IF_FALSE(IS_SLIM_WRAPPER(cur),
                                  "what kind of wrapper is this?");
                lccx->SetWrapper(cur);
            }

            return NS_OK;
        }
    }

    *pThisRef = nsnull;
    return NS_ERROR_XPC_BAD_OP_ON_WN_PROTO;
}

JSBool
xpc_qsUnwrapThisFromCcxImpl(XPCCallContext &ccx,
                            const nsIID &iid,
                            void **ppThis,
                            nsISupports **pThisRef,
                            jsval *vp)
{
    nsISupports *native = ccx.GetIdentityObject();
    if (!native)
        return xpc_qsThrow(ccx.GetJSContext(), NS_ERROR_XPC_HAS_BEEN_SHUTDOWN);

    nsresult rv = getNative(native, GetOffsets(native, ccx.GetProto()),
                            ccx.GetFlattenedJSObject(), iid, ppThis, pThisRef,
                            vp);
    if (NS_FAILED(rv))
        return xpc_qsThrow(ccx.GetJSContext(), rv);
    return true;
}

JSObject*
xpc_qsUnwrapObj(JS::Value v, nsISupports **ppArgRef, nsresult *rv)
{
    if (v.isNullOrUndefined()) {
        *ppArgRef = nsnull;
        *rv = NS_OK;
        return nsnull;
    }

    if (!v.isObject()) {
        *ppArgRef = nsnull;
        *rv = ((v.isInt32() && v.toInt32() == 0)
               ? NS_ERROR_XPC_BAD_CONVERT_JS_ZERO_ISNOT_NULL
               : NS_ERROR_XPC_BAD_CONVERT_JS);
        return nsnull;
    }

    *rv = NS_OK;
    return &v.toObject();
}

nsresult
xpc_qsUnwrapArgImpl(JSContext *cx,
                    jsval v,
                    const nsIID &iid,
                    void **ppArg,
                    nsISupports **ppArgRef,
                    jsval *vp)
{
    nsresult rv;
    JSObject *src = xpc_qsUnwrapObj(v, ppArgRef, &rv);
    if (!src) {
        *ppArg = nsnull;

        return rv;
    }

    XPCWrappedNative *wrapper;
    XPCWrappedNativeTearOff *tearoff;
    JSObject *obj2;
    rv = getWrapper(cx, src, &wrapper, &obj2, &tearoff);
    NS_ENSURE_SUCCESS(rv, rv);

    if (wrapper || obj2) {
        if (NS_FAILED(castNative(cx, wrapper, obj2, tearoff, iid, ppArg,
                                 ppArgRef, vp, nsnull)))
            return NS_ERROR_XPC_BAD_CONVERT_JS;
        return NS_OK;
    }
    // else...
    // Slow path.

    // XXX E4X breaks the world. Don't try wrapping E4X objects!
    // This hack can be removed (or changed accordingly) when the
    // DOM <-> E4X bindings are complete, see bug 270553
    if (JS_TypeOfValue(cx, OBJECT_TO_JSVAL(src)) == JSTYPE_XML) {
        *ppArgRef = nsnull;
        return NS_ERROR_XPC_BAD_CONVERT_JS;
    }

    // Try to unwrap a slim wrapper.
    nsISupports *iface;
    if (XPCConvert::GetISupportsFromJSObject(src, &iface)) {
        if (!iface || NS_FAILED(iface->QueryInterface(iid, ppArg))) {
            *ppArgRef = nsnull;
            return NS_ERROR_XPC_BAD_CONVERT_JS;
        }

        *ppArgRef = static_cast<nsISupports*>(*ppArg);
        return NS_OK;
    }

    // Create the ccx needed for quick stubs.
    XPCCallContext ccx(JS_CALLER, cx);
    if (!ccx.IsValid()) {
        *ppArgRef = nsnull;
        return NS_ERROR_XPC_BAD_CONVERT_JS;
    }

    nsRefPtr<nsXPCWrappedJS> wrappedJS;
    rv = nsXPCWrappedJS::GetNewOrUsed(ccx, src, iid, nsnull,
                                      getter_AddRefs(wrappedJS));
    if (NS_FAILED(rv) || !wrappedJS) {
        *ppArgRef = nsnull;
        return rv;
    }

    // We need to go through the QueryInterface logic to make this return
    // the right thing for the various 'special' interfaces; e.g.
    // nsIPropertyBag. We must use AggregatedQueryInterface in cases where
    // there is an outer to avoid nasty recursion.
    rv = wrappedJS->QueryInterface(iid, ppArg);
    if (NS_SUCCEEDED(rv)) {
        *ppArgRef = static_cast<nsISupports*>(*ppArg);
        *vp = OBJECT_TO_JSVAL(wrappedJS->GetJSObject());
    }
    return rv;
}

JSBool
xpc_qsJsvalToCharStr(JSContext *cx, jsval v, JSAutoByteString *bytes)
{
    JSString *str;

    JS_ASSERT(!bytes->ptr());
    if (JSVAL_IS_STRING(v)) {
        str = JSVAL_TO_STRING(v);
    } else if (JSVAL_IS_VOID(v) || JSVAL_IS_NULL(v)) {
        return true;
    } else {
        if (!(str = JS_ValueToString(cx, v)))
            return false;
    }
    return !!bytes->encode(cx, str);
}

JSBool
xpc_qsJsvalToWcharStr(JSContext *cx, jsval v, jsval *pval, const PRUnichar **pstr)
{
    JSString *str;

    if (JSVAL_IS_STRING(v)) {
        str = JSVAL_TO_STRING(v);
    } else if (JSVAL_IS_VOID(v) || JSVAL_IS_NULL(v)) {
        *pstr = NULL;
        return true;
    } else {
        if (!(str = JS_ValueToString(cx, v)))
            return false;
        *pval = STRING_TO_JSVAL(str);  // Root the new string.
    }

    const jschar *chars = JS_GetStringCharsZ(cx, str);
    if (!chars)
        return false;

    *pstr = static_cast<const PRUnichar *>(chars);
    return true;
}

namespace xpc {

bool
StringToJsval(JSContext *cx, nsAString &str, JS::Value *rval)
{
    // From the T_DOMSTRING case in XPCConvert::NativeData2JS.
    if (str.IsVoid()) {
        *rval = JSVAL_NULL;
        return true;
    }
    return NonVoidStringToJsval(cx, str, rval);
}

bool
NonVoidStringToJsval(JSContext *cx, nsAString &str, JS::Value *rval)
{
    nsStringBuffer* sharedBuffer;
    jsval jsstr = XPCStringConvert::ReadableToJSVal(cx, str, &sharedBuffer);
    if (JSVAL_IS_NULL(jsstr))
        return false;
    *rval = jsstr;
    if (sharedBuffer) {
        // The string was shared but ReadableToJSVal didn't addref it.
        // Move the ownership from str to jsstr.
        str.ForgetSharedBuffer();
    }
    return true;
}

} // namespace xpc

JSBool
xpc_qsStringToJsstring(JSContext *cx, nsString &str, JSString **rval)
{
    // From the T_DOMSTRING case in XPCConvert::NativeData2JS.
    if (str.IsVoid()) {
        *rval = nsnull;
        return true;
    }

    nsStringBuffer* sharedBuffer;
    jsval jsstr = XPCStringConvert::ReadableToJSVal(cx, str, &sharedBuffer);
    if (JSVAL_IS_NULL(jsstr))
        return false;
    *rval = JSVAL_TO_STRING(jsstr);
    if (sharedBuffer) {
        // The string was shared but ReadableToJSVal didn't addref it.
        // Move the ownership from str to jsstr.
        str.ForgetSharedBuffer();
    }
    return true;
}

JSBool
xpc_qsXPCOMObjectToJsval(XPCLazyCallContext &lccx, qsObjectHelper &aHelper,
                         const nsIID *iid, XPCNativeInterface **iface,
                         jsval *rval)
{
    NS_PRECONDITION(iface, "Who did that and why?");

    // From the T_INTERFACE case in XPCConvert::NativeData2JS.
    // This is one of the slowest things quick stubs do.

    JSContext *cx = lccx.GetJSContext();

    nsresult rv;
    if (!XPCConvert::NativeInterface2JSObject(lccx, rval, nsnull,
                                              aHelper, iid, iface,
                                              true, &rv)) {
        // I can't tell if NativeInterface2JSObject throws JS exceptions
        // or not.  This is a sloppy stab at the right semantics; the
        // method really ought to be fixed to behave consistently.
        if (!JS_IsExceptionPending(cx))
            xpc_qsThrow(cx, NS_FAILED(rv) ? rv : NS_ERROR_UNEXPECTED);
        return false;
    }

#ifdef DEBUG
    JSObject* jsobj = JSVAL_TO_OBJECT(*rval);
    if (jsobj && !js::GetObjectParent(jsobj))
        NS_ASSERTION(js::GetObjectClass(jsobj)->flags & JSCLASS_IS_GLOBAL,
                     "Why did we recreate this wrapper?");
#endif

    return true;
}

JSBool
xpc_qsVariantToJsval(XPCLazyCallContext &lccx,
                     nsIVariant *p,
                     jsval *rval)
{
    // From the T_INTERFACE case in XPCConvert::NativeData2JS.
    // Error handling is in XPCWrappedNative::CallMethod.
    if (p) {
        nsresult rv;
        JSBool ok = XPCVariant::VariantDataToJS(lccx, p, &rv, rval);
        if (!ok)
            xpc_qsThrow(lccx.GetJSContext(), rv);
        return ok;
    }
    *rval = JSVAL_NULL;
    return true;
}

#ifdef DEBUG
void
xpc_qsAssertContextOK(JSContext *cx)
{
    XPCJSContextStack* stack = XPCJSRuntime::Get()->GetJSContextStack();

    JSContext *topJSContext = stack->Peek();

    // This is what we're actually trying to assert here.
    NS_ASSERTION(cx == topJSContext, "wrong context on XPCJSContextStack!");
}
#endif
