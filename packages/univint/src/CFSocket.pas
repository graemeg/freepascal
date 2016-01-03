{	CFSocket.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1999-2013, Apple Inc.  All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 1999-2009, Apple Inc.  All rights reserved.
}
{   Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	Copyright (c) 1999-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1999-2009, Apple Inc.  All rights reserved.
>>>>>>> origin/fixes_2.4
}
{   Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit CFSocket;
interface
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> graemeg/fixes_2_2
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> origin/fixes_2_2
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
>>>>>>> origin/fixes_2.4

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
>>>>>>> origin/fixes_2.4
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __x86_64__ and defined CPUX86_64}
	{$setc __x86_64__ := 1}
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __arm64__ and defined CPUAARCH64}
  {$setc __arm64__ := 1}
{$elsec}
  {$setc __arm64__ := 0}
{$endc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_PPC64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> origin/fixes_2.4
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
<<<<<<< HEAD
{$setc TARGET_CPU_PPC_64 := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ifc defined FPC_BIG_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := TRUE}
	{$setc TARGET_RT_LITTLE_ENDIAN := FALSE}
{$elifc defined FPC_LITTLE_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := FALSE}
	{$setc TARGET_RT_LITTLE_ENDIAN := TRUE}
{$elsec}
	{$error Neither FPC_BIG_ENDIAN nor FPC_LITTLE_ENDIAN are defined.}
{$endc}
{$setc ACCESSOR_CALLS_ARE_FUNCTIONS := TRUE}
{$setc CALL_NOT_IN_CARBON := FALSE}
{$setc OLDROUTINENAMES := FALSE}
{$setc OPAQUE_TOOLBOX_STRUCTS := TRUE}
{$setc OPAQUE_UPP_TYPES := TRUE}
{$setc OTCARBONAPPLICATION := TRUE}
{$setc OTKERNEL := FALSE}
{$setc PM_USE_SESSION_APIS := TRUE}
{$setc TARGET_API_MAC_CARBON := TRUE}
{$setc TARGET_API_MAC_OS8 := FALSE}
{$setc TARGET_API_MAC_OSX := TRUE}
{$setc TARGET_CARBON := TRUE}
{$setc TARGET_CPU_68K := FALSE}
{$setc TARGET_CPU_MIPS := FALSE}
{$setc TARGET_CPU_SPARC := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,CFBase,CFData,CFString,CFRunLoop,CFDate;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


type
	CFSocketNativeHandle = SInt32;


type
<<<<<<< HEAD
<<<<<<< HEAD
	CFSocketRef = ^__CFSocket; { an opaque type }
	__CFSocket = record end;
=======
	CFSocketRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFSocketRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2

{ A CFSocket contains a native socket within a structure that can 
be used to read from the socket in the background and make the data
thus read available using a runloop source.  The callback used for
this may be of three types, as specified by the callBackTypes
argument when creating the CFSocket.

If kCFSocketReadCallBack is used, then data will not be
automatically read, but the callback will be called when data
is available to be read, or a new child socket is waiting to be
accepted.
    
If kCFSocketAcceptCallBack is used, then new child sockets will be
accepted and passed to the callback, with the data argument being
a pointer to a CFSocketNativeHandle.  This is usable only with
connection rendezvous sockets.

If kCFSocketDataCallBack is used, then data will be read in chunks
in the background and passed to the callback, with the data argument
being a CFDataRef.

These three types are mutually exclusive, but any one of them may
have kCFSocketConnectCallBack added to it, if the socket will be
used to connect in the background.  Connect in the background occurs
if CFSocketConnectToAddress is called with a negative timeout
value, in which case the call returns immediately, and a
kCFSocketConnectCallBack is generated when the connect finishes.
In this case the data argument is either NULL, or a pointer to
an SInt32 error code if the connect failed.  kCFSocketConnectCallBack
will never be sent more than once for a given socket.

The callback types may also have kCFSocketWriteCallBack added to
them, if large amounts of data are to be sent rapidly over the 
socket and notification is desired when there is space in the
kernel buffers so that the socket is writable again.  

With a connection-oriented socket, if the connection is broken from the
other end, then one final kCFSocketReadCallBack or kCFSocketDataCallBack 
will occur.  In the case of kCFSocketReadCallBack, the underlying socket 
will have 0 bytes available to read.  In the case of kCFSocketDataCallBack,
the data argument will be a CFDataRef of length 0.

There are socket flags that may be set to control whether callbacks of 
a given type are automatically reenabled after they are triggered, and 
whether the underlying native socket will be closed when the CFSocket
is invalidated.  By default read, accept, and data callbacks are 
automatically reenabled; write callbacks are not, and connect callbacks
may not be, since they are sent once only.  Be careful about automatically
reenabling read and write callbacks, since this implies that the 
callbacks will be sent repeatedly if the socket remains readable or
writable respectively.  Be sure to set these flags only for callbacks
that your CFSocket actually possesses; the result of setting them for
other callback types is undefined.

Individual callbacks may also be enabled and disabled manually, whether 
they are automatically reenabled or not.  If they are not automatically 
reenabled, then they will need to be manually reenabled when the callback 
is ready to be received again (and not sooner).  Even if they are 
automatically reenabled, there may be occasions when it will be useful
to be able to manually disable them temporarily and then reenable them.
Be sure to enable and disable only callbacks that your CFSocket actually
possesses; the result of enabling and disabling other callback types is
undefined.

By default the underlying native socket will be closed when the CFSocket 
is invalidated, but it will not be if kCFSocketCloseOnInvalidate is 
turned off.  This can be useful in order to destroy a CFSocket but 
continue to use the underlying native socket.  The CFSocket must 
still be invalidated when it will no longer be used.  Do not in 
either case close the underlying native socket without invalidating 
the CFSocket.

Addresses are stored as CFDatas containing a struct sockaddr
appropriate for the protocol family; make sure that all fields are
filled in properly when passing in an address.  

}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFSocketError = CFIndex;
=======
	CFSocketError = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFSocketError = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFSocketError = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFSocketError = SIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	CFSocketError = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFSocketError = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFSocketError = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
    kCFSocketSuccess = 0;
    kCFSocketError = -1;
    kCFSocketTimeout = -2;

type
	CFSocketSignature = record
		protocolFamily: SInt32;
		socketType: SInt32;
		protocol: SInt32;
		address: CFDataRef;
	end;

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFSocketCallBackType = CFOptionFlags;
=======
	CFSocketCallBackType = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFSocketCallBackType = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFSocketCallBackType = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFSocketCallBackType = UNSIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	CFSocketCallBackType = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFSocketCallBackType = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFSocketCallBackType = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
    kCFSocketNoCallBack = 0;
    kCFSocketReadCallBack = 1;
    kCFSocketAcceptCallBack = 2;
    kCFSocketDataCallBack = 3;
    kCFSocketConnectCallBack = 4;
{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
    kCFSocketWriteCallBack = 8;
{#endif}

{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{ Socket flags }
const
	kCFSocketAutomaticallyReenableReadCallBack = 1;
	kCFSocketAutomaticallyReenableAcceptCallBack = 2;
	kCFSocketAutomaticallyReenableDataCallBack = 3;
	kCFSocketAutomaticallyReenableWriteCallBack = 8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{#if MAC_OS_X_VERSION_10_5 <= MAC_OS_X_VERSION_MAX_ALLOWED}
	kCFSocketLeaveErrors = 64;
{#endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFSocketCloseOnInvalidate = 128; (* CF_AVAILABLE_STARTING(10_5, 2_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kCFSocketCloseOnInvalidate = 128;
>>>>>>> graemeg/cpstrnew
=======
=======
{#if MAC_OS_X_VERSION_10_5 <= MAC_OS_X_VERSION_MAX_ALLOWED}
	kCFSocketLeaveErrors = 64;
{#endif}
>>>>>>> origin/fixes_2.4
	kCFSocketCloseOnInvalidate = 128;
>>>>>>> graemeg/fixes_2_2
=======
	kCFSocketCloseOnInvalidate = 128;
>>>>>>> origin/fixes_2_2
{#endif}

type
	CFSocketCallBack = procedure( s: CFSocketRef; typ: CFSocketCallBackType; address: CFDataRef; data: {const} UnivPtr; info: UnivPtr );
{ If the callback wishes to keep hold of address or data after the point that it returns, then it must copy them. }

type
	CFSocketContext = record
		version: CFIndex;
		info: UnivPtr;
		retain: function( info: {const} UnivPtr ): UnivPtr;
		release: procedure( info: {const} UnivPtr );
		copyDescription: function( info: {const} UnivPtr ): CFStringRef;
	end;

function CFSocketGetTypeID: CFTypeID; external name '_CFSocketGetTypeID';

function CFSocketCreate( allocator: CFAllocatorRef; protocolFamily: SInt32; socketType: SInt32; protocol: SInt32; callBackTypes: CFOptionFlags; callout: CFSocketCallBack; const (*var*) context: CFSocketContext ): CFSocketRef; external name '_CFSocketCreate';
function CFSocketCreateWithNative( allocator: CFAllocatorRef; sock: CFSocketNativeHandle; callBackTypes: CFOptionFlags; callout: CFSocketCallBack; const (*var*) context: CFSocketContext ): CFSocketRef; external name '_CFSocketCreateWithNative';
function CFSocketCreateWithSocketSignature( allocator: CFAllocatorRef; const (*var*) signature: CFSocketSignature; callBackTypes: CFOptionFlags; callout: CFSocketCallBack; const (*var*) context: CFSocketContext ): CFSocketRef; external name '_CFSocketCreateWithSocketSignature';
<<<<<<< HEAD
<<<<<<< HEAD
{ CFSocketCreateWithSocketSignature() creates a socket of the requested type and binds its address (using CFSocketSetAddress()) to the requested address.  If this fails, it returns NULL. }
function CFSocketCreateConnectedToSocketSignature( allocator: CFAllocatorRef; const (*var*) signature: CFSocketSignature; callBackTypes: CFOptionFlags; callout: CFSocketCallBack; const (*var*) context: CFSocketContext; timeout: CFTimeInterval ): CFSocketRef; external name '_CFSocketCreateConnectedToSocketSignature';
{ CFSocketCreateConnectedToSocketSignature() creates a socket suitable for connecting to the requested type and address, and connects it (using CFSocketConnectToAddress()).  If this fails, it returns NULL. }
=======
function CFSocketCreateConnectedToSocketSignature( allocator: CFAllocatorRef; const (*var*) signature: CFSocketSignature; callBackTypes: CFOptionFlags; callout: CFSocketCallBack; const (*var*) context: CFSocketContext; timeout: CFTimeInterval ): CFSocketRef; external name '_CFSocketCreateConnectedToSocketSignature';
{ CFSocketCreateWithSignature creates a socket of the requested type and binds its address (using CFSocketSetAddress) to the requested address.  If this fails, it returns NULL.  CFSocketCreateConnectedToSignature creates a socket suitable for connecting to the requested type and address, and connects it (using CFSocketConnectToAddress).  If this fails, it returns NULL. }
>>>>>>> graemeg/fixes_2_2
=======
function CFSocketCreateConnectedToSocketSignature( allocator: CFAllocatorRef; const (*var*) signature: CFSocketSignature; callBackTypes: CFOptionFlags; callout: CFSocketCallBack; const (*var*) context: CFSocketContext; timeout: CFTimeInterval ): CFSocketRef; external name '_CFSocketCreateConnectedToSocketSignature';
{ CFSocketCreateWithSignature creates a socket of the requested type and binds its address (using CFSocketSetAddress) to the requested address.  If this fails, it returns NULL.  CFSocketCreateConnectedToSignature creates a socket suitable for connecting to the requested type and address, and connects it (using CFSocketConnectToAddress).  If this fails, it returns NULL. }
>>>>>>> origin/fixes_2_2

function CFSocketSetAddress( s: CFSocketRef; address: CFDataRef ): CFSocketError; external name '_CFSocketSetAddress';
function CFSocketConnectToAddress( s: CFSocketRef; address: CFDataRef; timeout: CFTimeInterval ): CFSocketError; external name '_CFSocketConnectToAddress';
procedure CFSocketInvalidate( s: CFSocketRef ); external name '_CFSocketInvalidate';

function CFSocketIsValid( s: CFSocketRef ): Boolean; external name '_CFSocketIsValid';
function CFSocketCopyAddress( s: CFSocketRef ): CFDataRef; external name '_CFSocketCopyAddress';
function CFSocketCopyPeerAddress( s: CFSocketRef ): CFDataRef; external name '_CFSocketCopyPeerAddress';
procedure CFSocketGetContext( s: CFSocketRef; var context: CFSocketContext ); external name '_CFSocketGetContext';
function CFSocketGetNative( s: CFSocketRef ): CFSocketNativeHandle; external name '_CFSocketGetNative';

function CFSocketCreateRunLoopSource( allocator: CFAllocatorRef; s: CFSocketRef; order: CFIndex ): CFRunLoopSourceRef; external name '_CFSocketCreateRunLoopSource';

{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
function CFSocketGetSocketFlags( s: CFSocketRef ): CFOptionFlags; external name '_CFSocketGetSocketFlags';
procedure CFSocketSetSocketFlags( s: CFSocketRef; flags: CFOptionFlags ); external name '_CFSocketSetSocketFlags';
procedure CFSocketDisableCallBacks( s: CFSocketRef; callBackTypes: CFOptionFlags ); external name '_CFSocketDisableCallBacks';
procedure CFSocketEnableCallBacks( s: CFSocketRef; callBackTypes: CFOptionFlags ); external name '_CFSocketEnableCallBacks';
{#endif}

{ For convenience, a function is provided to send data using the socket with a timeout.  The timeout will be used only if the specified value is positive.  The address should be left NULL if the socket is already connected. }
function CFSocketSendData( s: CFSocketRef; address: CFDataRef; data: CFDataRef; timeout: CFTimeInterval ): CFSocketError; external name '_CFSocketSendData';

{ Generic name registry functionality (CFSocketRegisterValue, 
CFSocketCopyRegisteredValue) allows the registration of any property
list type.  Functions specific to CFSockets (CFSocketRegisterSocketData,
CFSocketCopyRegisteredSocketData) register a CFData containing the
components of a socket signature (protocol family, socket type,
protocol, and address).  In each function the nameServerSignature
may be NULL, or any component of it may be 0, to use default values
(TCP, INADDR_LOOPBACK, port as set).  Name registration servers might
not allow registration with other than TCP and INADDR_LOOPBACK.
The actual address of the server responding to a query may be obtained
by using the nameServerAddress argument.  This address, the address
returned by CFSocketCopyRegisteredSocketSignature, and the value
returned by CFSocketCopyRegisteredValue must (if non-null) be released
by the caller.  CFSocketUnregister removes any registration associated
with the specified name.
}

function CFSocketRegisterValue( const (*var*) nameServerSignature: CFSocketSignature; timeout: CFTimeInterval; name: CFStringRef; value: CFPropertyListRef ): CFSocketError; external name '_CFSocketRegisterValue';
function CFSocketCopyRegisteredValue( const (*var*) nameServerSignature: CFSocketSignature; timeout: CFTimeInterval; name: CFStringRef; var value: CFPropertyListRef; var nameServerAddress: CFDataRef ): CFSocketError; external name '_CFSocketCopyRegisteredValue';

function CFSocketRegisterSocketSignature( const (*var*) nameServerSignature: CFSocketSignature; timeout: CFTimeInterval; name: CFStringRef; const (*var*) signature: CFSocketSignature ): CFSocketError; external name '_CFSocketRegisterSocketSignature';
function CFSocketCopyRegisteredSocketSignature( const (*var*) nameServerSignature: CFSocketSignature; timeout: CFTimeInterval; name: CFStringRef; var signature: CFSocketSignature; var nameServerAddress: CFDataRef ): CFSocketError; external name '_CFSocketCopyRegisteredSocketSignature';

function CFSocketUnregister( const (*var*) nameServerSignature: CFSocketSignature; timeout: CFTimeInterval; name: CFStringRef ): CFSocketError; external name '_CFSocketUnregister';

procedure CFSocketSetDefaultNameRegistryPortNumber( port: UInt16 ); external name '_CFSocketSetDefaultNameRegistryPortNumber';
function CFSocketGetDefaultNameRegistryPortNumber: UInt16; external name '_CFSocketGetDefaultNameRegistryPortNumber';

{ Constants used in name registry server communications }
var kCFSocketCommandKey: CFStringRef; external name '_kCFSocketCommandKey'; (* attribute const *)
var kCFSocketNameKey: CFStringRef; external name '_kCFSocketNameKey'; (* attribute const *)
var kCFSocketValueKey: CFStringRef; external name '_kCFSocketValueKey'; (* attribute const *)
var kCFSocketResultKey: CFStringRef; external name '_kCFSocketResultKey'; (* attribute const *)
var kCFSocketErrorKey: CFStringRef; external name '_kCFSocketErrorKey'; (* attribute const *)
var kCFSocketRegisterCommand: CFStringRef; external name '_kCFSocketRegisterCommand'; (* attribute const *)
var kCFSocketRetrieveCommand: CFStringRef; external name '_kCFSocketRetrieveCommand'; (* attribute const *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======

end.
>>>>>>> graemeg/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
