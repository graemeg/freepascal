{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2003-2006, 2008-2010 Apple Inc. All rights reserved.
=======
 * Copyright (c) 2003-2006, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2003-2006, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2003-2006, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2003-2006, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> origin/cpstrnew
=======
 * Copyright (c) 2002-2003 Apple Computer, Inc. All rights reserved.
>>>>>>> graemeg/fixes_2_2
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
<<<<<<< HEAD
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
=======
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY of ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES of MERCHANTABILITY,
>>>>>>> graemeg/fixes_2_2
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 }
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }


{
    Modified for use with Free Pascal
    Version 210
    Please report any bugs to <gpc@microbizz.nl>
}

>>>>>>> graemeg/fixes_2_2
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit SCNetworkConnection;
interface
<<<<<<< HEAD
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> graemeg/fixes_2_2

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
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
=======
>>>>>>> graemeg/fixes_2_2
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
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

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}

=======
>>>>>>> graemeg/fixes_2_2

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
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
>>>>>>> origin/cpstrnew
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
>>>>>>> graemeg/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
	{$setc TARGET_CPU_X86 := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
{$elsec}
	{$error Neither __ppc__ nor __i386__ is defined.}
{$endc}
{$setc TARGET_CPU_PPC_64 := FALSE}
>>>>>>> graemeg/fixes_2_2

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
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
<<<<<<< HEAD
uses MacTypes,MacOSXPosix,CFBase,CFDictionary,CFRunLoop;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{!
	@header SCNetworkConnection
	@discussion The SCNetworkConnection API contains functions that allow
		an application to control connection-oriented services defined
		in the system and get connection-status information.

		The functions in the SCNetworkConnection API allow you to
		control and get information about existing services only.
		If you need to create, change, or remove services, you
		should use the SCNetworkConfiguration API instead.

		Note: Currently, only PPP services can be controlled.
=======
uses MacTypes,CFBase,CFDictionary,CFRunLoop;
{$ALIGN MAC68K}

{!
	@header SCNetworkConnection
	The SCNetworkConnectionXXX() APIs allow an application to
	control connection oriented services defined in the system.

	This is a set of control APIs only. Using these APIs, an
	application will be able to control existing services.
	To create, change, or remove services, SCPreferences APIs
	must be used.

	Note: Currently only PPP services can be controlled.
>>>>>>> graemeg/fixes_2_2
 }


{!
	@typedef SCNetworkConnectionRef
<<<<<<< HEAD
	@discussion This is the handle to manage a connection-oriented service.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCNetworkConnectionRef = ^__SCNetworkConnection; { an opaque type }
	__SCNetworkConnection = record end;
=======
	SCNetworkConnectionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkConnectionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkConnectionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkConnectionRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	@discussion This is the handle to manage a connection oriented service.
 }
type
	SCNetworkConnectionRef							= ^SInt32;
>>>>>>> graemeg/fixes_2_2


{!
	@typedef SCNetworkConnectionContext
<<<<<<< HEAD
	@discussion Structure containing user-specified data and callbacks
		for a SCNetworkConnection.
	@field version The version number of the structure type being passed
		in as a parameter to the SCNetworkConnectionCreateWithServiceID
		function.  This structure is version 0.
	@field info A C pointer to a user-specified block of data.
	@field retain The callback used to add a retain for the info field.
		If this parameter is not a pointer to a function of the correct
		prototype, the behavior is undefined.  The value may be NULL.
	@field release The calllback used to remove a retain previously added
		for the info field.  If this parameter is not a pointer to a
		function of the correct prototype, the behavior is undefined.
		The value may be NULL.
	@field copyDescription The callback used to provide a description of
		the info field.
=======
>>>>>>> graemeg/fixes_2_2
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
type SCNetworkConnectionContext = record
		version: CFIndex;
		info: Ptr;
<<<<<<< HEAD
		retain: function( info: UnivPtr ): UnivPtr;
		release: procedure( info: UnivPtr );
		copyDescription: function( info: UnivPtr ): CFStringRef;
=======
		retain: function( info: Ptr ): Ptr;
		release: procedure( info: Ptr );
		copyDescription: function( info: Ptr ): CFStringRef;
>>>>>>> graemeg/fixes_2_2
	end;
	SCNetworkConnectionContextPtr = ^SCNetworkConnectionContext;


{!
	@enum SCNetworkConnectionStatus
	@discussion Status of the network connection.
		This status is intended to be generic and high level.
		An extended status, specific to the type of network
		connection is also available for applications that
		need additonal information.
<<<<<<< HEAD
	@constant kSCNetworkConnectionInvalid
		The network connection refers to an invalid service.
	@constant kSCNetworkConnectionDisconnected
		The network connection is disconnected.
	@constant kSCNetworkConnectionConnecting
		The network connection is connecting.
	@constant kSCNetworkConnectionConnected
		The network connection is connected.
=======

	@constant kSCNetworkConnectionInvalid
		The network connection refers to an invalid service.

	@constant kSCNetworkConnectionDisconnected
		The network connection is disconnected.

	@constant kSCNetworkConnectionConnecting
		The network connection is connecting.

	@constant kSCNetworkConnectionConnected
		The network connection is connected.

>>>>>>> graemeg/fixes_2_2
	@constant kSCNetworkConnectionDisconnecting
		The network connection is disconnecting.
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
<<<<<<< HEAD
const
	kSCNetworkConnectionInvalid = -1;
	kSCNetworkConnectionDisconnected = 0;
	kSCNetworkConnectionConnecting = 1;
	kSCNetworkConnectionConnected = 2;
	kSCNetworkConnectionDisconnecting = 3;
type
	SCNetworkConnectionStatus = SInt32;
=======
type
	SCNetworkConnectionStatus = SInt32;
const
	kSCNetworkConnectionInvalid		=  -1;
	kSCNetworkConnectionDisconnected	=  0;
	kSCNetworkConnectionConnecting		=  1;
	kSCNetworkConnectionConnected		=  2;
	kSCNetworkConnectionDisconnecting	=  3;
>>>>>>> graemeg/fixes_2_2


{!
	@enum SCNetworkConnectionPPPStatus
<<<<<<< HEAD
	@discussion PPP-specific status of the network connection.
		This status is returned as part of the extended information
		for a PPP service.
		Note: additional status might be returned in the future.
		Your application should be prepared to receive an unknown value.
	@constant kSCNetworkConnectionPPPDisconnected
		PPP is disconnected.
	@constant kSCNetworkConnectionPPPInitializing
		PPP is initializing.
	@constant kSCNetworkConnectionPPPConnectingLink
		PPP is connecting the lower connection layer (for example,
		the modem is dialing out).
	@constant kSCNetworkConnectionPPPDialOnTraffic
		PPP is waiting for networking traffic to automatically
		establish the connection.
	@constant kSCNetworkConnectionPPPNegotiatingLink
		The PPP lower layer is connected and PPP is negotiating the
		link layer (LCP protocol).
	@constant kSCNetworkConnectionPPPAuthenticating
		PPP is authenticating to the server (PAP, CHAP, MS-CHAP or
		EAP protocols).
	@constant kSCNetworkConnectionPPPWaitingForCallBack
		PPP is waiting for the server to call back.
	@constant kSCNetworkConnectionPPPNegotiatingNetwork
		PPP is now authenticated and negotiating the networking
		layer (IPCP or IPv6CP protocols)
	@constant kSCNetworkConnectionPPPConnected
		PPP is now fully connected for at least one networking layer.
		Additional networking protocol might still be negotiating.
	@constant kSCNetworkConnectionPPPTerminating
		PPP networking and link protocols are terminating.
	@constant kSCNetworkConnectionPPPDisconnectingLink
		PPP is disconnecting the lower level (for example, the modem
		is hanging up).
	@constant kSCNetworkConnectionPPPHoldingLinkOff
		PPP is disconnected and maintaining the link temporarily off.
	@constant kSCNetworkConnectionPPPSuspended
		PPP is suspended as a result of the suspend command (for
		example, when a V.92 Modem is On Hold).
=======
	@discussion PPP specific status of the network connection.
		This status is PPP specific and returned as part of the extended information
		for a PPP service.
		Note: additional status might be returned in the future, and the application should
		be prepared to receive an unknown value.

	@constant kSCNetworkConnectionPPPDisconnected
		PPP is disconnected.

	@constant kSCNetworkConnectionPPPInitializing
		PPP is initializing.

	@constant kSCNetworkConnectionPPPConnectingLink
		PPP is connecting the lower connection layer (for example, the modem is dialing out).

	@constant kSCNetworkConnectionPPPDialOnTraffic
		PPP is waiting for networking traffic to automatically establish the connection.

	@constant kSCNetworkConnectionPPPNegotiatingLink
		PPP lower layer is connected and PPP is negotiating the link layer (LCP protocol).

	@constant kSCNetworkConnectionPPPAuthenticating
		PPP is authenticating to the server (PAP, CHAP, MS-CHAP or EAP protocols).

	@constant kSCNetworkConnectionPPPWaitingForCallBack
		PPP is waiting for server to call back.

	@constant kSCNetworkConnectionPPPNegotiatingNetwork
		PPP is now authenticated and negotiating the networking layer (IPCP or IPv6CP protocols)

	@constant kSCNetworkConnectionPPPConnected
		PPP is now fully connected for at least one of the networking layer.
		Additional networking protocol might still be negotiating.

	@constant kSCNetworkConnectionPPPTerminating
		PPP networking and link protocols are terminating.

	@constant kSCNetworkConnectionPPPDisconnectingLink
		PPP is disconnecting the lower level (for example, the modem is hanging up).

	@constant kSCNetworkConnectionPPPHoldingLinkOff
		PPP is disconnected and maintaining the link temporarily off.

	@constant kSCNetworkConnectionPPPSuspended
		PPP is suspended as a result of the suspend command (for example, when a V92 Modem is On Hold).

>>>>>>> graemeg/fixes_2_2
	@constant kSCNetworkConnectionPPPWaitingForRedial
		PPP has found a busy server and is waiting for redial.
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
<<<<<<< HEAD
const
	kSCNetworkConnectionPPPDisconnected = 0;
	kSCNetworkConnectionPPPInitializing = 1;
	kSCNetworkConnectionPPPConnectingLink = 2;
	kSCNetworkConnectionPPPDialOnTraffic = 3;
	kSCNetworkConnectionPPPNegotiatingLink = 4;
	kSCNetworkConnectionPPPAuthenticating = 5;
	kSCNetworkConnectionPPPWaitingForCallBack = 6;
	kSCNetworkConnectionPPPNegotiatingNetwork = 7;
	kSCNetworkConnectionPPPConnected = 8;
	kSCNetworkConnectionPPPTerminating = 9;
	kSCNetworkConnectionPPPDisconnectingLink = 10;
	kSCNetworkConnectionPPPHoldingLinkOff = 11;
	kSCNetworkConnectionPPPSuspended = 12;
	kSCNetworkConnectionPPPWaitingForRedial = 13;
type
	SCNetworkConnectionPPPStatus = SInt32;
=======
type
	SCNetworkConnectionPPPStatus = SInt32;
const
	kSCNetworkConnectionPPPDisconnected		=  0;
	kSCNetworkConnectionPPPInitializing		=  1;
	kSCNetworkConnectionPPPConnectingLink		=  2;
	kSCNetworkConnectionPPPDialOnTraffic		=  3;
	kSCNetworkConnectionPPPNegotiatingLink		=  4;
	kSCNetworkConnectionPPPAuthenticating		=  5;
	kSCNetworkConnectionPPPWaitingForCallBack	=  6;
	kSCNetworkConnectionPPPNegotiatingNetwork	=  7;
	kSCNetworkConnectionPPPConnected		=  8;
	kSCNetworkConnectionPPPTerminating		=  9;
	kSCNetworkConnectionPPPDisconnectingLink	=  10;
	kSCNetworkConnectionPPPHoldingLinkOff		=  11;
	kSCNetworkConnectionPPPSuspended		=  12;
	kSCNetworkConnectionPPPWaitingForRedial		=  13;
>>>>>>> graemeg/fixes_2_2


{!
	@typedef SCNetworkConnectionCallBack
	@discussion Type of the callback function used when a
		status event is delivered.
	@param status The connection status.
	@param connection The connection reference.
<<<<<<< HEAD
	@param info Application-specific information.
 }
type
  // AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
	SCNetworkConnectionCallBack = procedure( connection: SCNetworkConnectionRef; status: SCNetworkConnectionStatus; info: UnivPtr );
=======
	@param info ....
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
type SCNetworkConnectionCallBack = procedure( connection: SCNetworkConnectionRef; status: SCNetworkConnectionStatus; info: Ptr );
>>>>>>> graemeg/fixes_2_2


{
    Keys for the statistics dictionary
}

{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionBytesIn CFSTRP('BytesIn')}
{$endc}		{ CFNumber }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionBytesOut CFSTRP('BytesOut')}
{$endc}		{ CFNumber }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionPacketsIn CFSTRP('PacketsIn')}
{$endc}		{ CFNumber }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionPacketsOut CFSTRP('PacketsOut')}
{$endc}		{ CFNumber }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionErrorsIn CFSTRP('ErrorsIn')}
{$endc}		{ CFNumber }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionErrorsOut CFSTRP('ErrorsOut')}
{$endc}		{ CFNumber }


<<<<<<< HEAD
{
 Keys for the SCNetworkConnectionCopyUserPreferences() "selectionOptions"
 dictionary
 }

<<<<<<< HEAD
{!
	@define kSCNetworkConnectionSelectionOptionOnDemandHostName
	@discussion A host name that will be used to select the
		"best" SCNetworkConnection.
 }
// #if	(__MAC_OS_X_VERSION_MIN_REQUIRED >= 1060) || (__IPHONE_OS_VERSION_MIN_REQUIRED >= 30000) || TARGET_IPHONE_SIMULATOR
// Only on  Mac OS X 10.6+, iPhoneOS 3.0+, iPhoneSimulator
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionSelectionOptionOnDemandHostName CFSTRP('OnDemandHostName')}
{$endc}	{ CFString }

// #endif

{!
=======
{!
	@define kSCNetworkConnectionSelectionOptionOnDemandHostName
	@discussion A host name that will be used to select the
		"best" SCNetworkConnection.
 }
// #if	(__MAC_OS_X_VERSION_MIN_REQUIRED >= 1060) || (__IPHONE_OS_VERSION_MIN_REQUIRED >= 30000) || TARGET_IPHONE_SIMULATOR
// Only on  Mac OS X 10.6+, iPhoneOS 3.0+, iPhoneSimulator
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionSelectionOptionOnDemandHostName CFSTRP('OnDemandHostName')}
{$endc}	{ CFString }

// #endif

{!
>>>>>>> origin/cpstrnew
	@define kSCNetworkConnectionSelectionOptionOnDemandRetry
	@discussion A boolean value used to indicate whether a DNS query has
		already been issued for the specified OnDemand host name.
 }
// #if	(__MAC_OS_X_VERSION_MIN_REQUIRED >= 1060) || (__IPHONE_OS_VERSION_MIN_REQUIRED >= 30000) || TARGET_IPHONE_SIMULATOR
// Only on  Mac OS X 10.6+, iPhoneOS 3.0+, iPhoneSimulator
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kSCNetworkConnectionSelectionOptionOnDemandRetry CFSTRP('OnDemandRetry')}
{$endc}		{ CFBoolean }
// #endif


{ until __IPHONE_NA is automatically translated }
{$ifc TARGET_OS_MAC}

{!
	@function SCNetworkConnectionGetTypeID
	@discussion Returns the type identifier of all SCNetworkConnection
		instances.
 }
function SCNetworkConnectionGetTypeID: CFTypeID; external name '_SCNetworkConnectionGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)


{!
	@function SCNetworkConnectionCopyUserPreferences
	@discussion Provides the default service ID and a dictionary of user
		options for the connection.  Applications can use the
		returned serviceID and userOptions values to open a
		connection on the fly.
	@param selectionOptions Currently unimplemented. Pass NULL for this
		version.
	@param serviceID Reference to the default serviceID for starting
		connections, this value will be returned by the function.
	@param userOptions Reference to default userOptions for starting
		connections, this will be returned by the function.
	@result Returns TRUE if there is a valid service to dial;
		FALSE if the function was unable to retrieve a service to dial.
 }
function SCNetworkConnectionCopyUserPreferences( selectionOptions: CFDictionaryRef; var serviceID: CFStringRef; var userOptions: CFDictionaryRef ): Boolean; external name '_SCNetworkConnectionCopyUserPreferences';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
{!
	@function SCDynamicStoreGetTypeID
	Returns the type identifier of all SCNetworkConnection instances.
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionGetTypeID: CFTypeID; external name '_SCNetworkConnectionGetTypeID';


{!
    @function SCNetworkConnectionCopyUserPreferences
	@discussion Provides the default serviceID and a userOptions dictionary for the connection.
		Applications can use the serviceID and userOptions returned to open a connection on the fly.
	@param selectionOptions Currently unimplemented. Pass NULL for this version.
	@param serviceID Reference to the default serviceID for starting connections,
		this value will be returned by the function.
	@param userOptions Reference to default userOptions for starting connections,
		this will be returned by the function.
	@result TRUE if there is a valid service to dial.
		FALSE if function was unable to retrieve a service to dial.
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionCopyUserPreferences( selectionOptions: CFDictionaryRef; var serviceID: CFStringRef; var userOptions: CFDictionaryRef ): Boolean; external name '_SCNetworkConnectionCopyUserPreferences';
>>>>>>> graemeg/fixes_2_2


{!
	@function SCNetworkConnectionCreateWithServiceID
<<<<<<< HEAD
	@discussion Creates a new connection reference to use for getting
		the status or for connecting or disconnecting the associated
		service.
	@param allocator The CFAllocator that should be used to allocate
		memory for the connection structure.  This parameter may be
		NULL in which case the current default CFAllocator is used.
		If this reference is not a valid CFAllocator, the behavior
		is undefined.
	@param serviceID A string that defines the service identifier
		of the connection.  Service identifiers uniquely identify
		services in the system configuration database.
	@param callout The function to be called when the status
		of the connection changes.  If this parameter is NULL, the
		application will not receive notifications of status change
		and will need to poll for updates.
	@param context The SCNetworkConnectionContext associated with the
		callout.
	@result Returns a reference to the new SCNetworkConnection.
 }
function SCNetworkConnectionCreateWithServiceID( allocator: CFAllocatorRef; serviceID: CFStringRef; callout: SCNetworkConnectionCallBack; var context: SCNetworkConnectionContext ): SCNetworkConnectionRef; external name '_SCNetworkConnectionCreateWithServiceID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)


{!
	@function SCNetworkConnectionCopyServiceID
	@discussion Returns the service ID associated with the SCNetworkConnection.
	@param connection The SCNetworkConnection to obtain status from.
	@result Returns the service ID associated with the SCNetworkConnection.
 }
function SCNetworkConnectionCopyServiceID( connection: SCNetworkConnectionRef ): CFStringRef; external name '_SCNetworkConnectionCopyServiceID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
	@discussion Creates a new connection reference to use for getting the status,
		for connecting or for disconnecting the associated service.
	@param allocator The CFAllocator which should be used to allocate
		memory for the connection structure.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param serviceID A string that defines the service identifier
		of the connection. Service identifiers uniquely identify
		services in the system configuration database.
	@param callout The function to be called when the status
		of the connection changes.
		If this parameter is NULL, the application will not receive
		change of status notifications and will need to poll for updates.
	@param context The SCNetworkConnectionContext associated with the callout.
	@result A reference to the new SCNetworkConnection.
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionCreateWithServiceID( allocator: CFAllocatorRef; serviceID: CFStringRef; callout: SCNetworkConnectionCallBack; var context: SCNetworkConnectionContext ): SCNetworkConnectionRef; external name '_SCNetworkConnectionCreateWithServiceID';


{!
	@function SCNetworkConnectionCopyService
	@discussion Returns the service ID associated with the SCNetworkConnection.
	@param connection The SCNetworkConnection to obtained status from.
	Returns the service ID associated with the SCNetworkConnection.
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionCopyServiceID( connection: SCNetworkConnectionRef ): CFStringRef; external name '_SCNetworkConnectionCopyServiceID';
>>>>>>> graemeg/fixes_2_2


{!
	@function SCNetworkConnectionGetStatus
	@discussion Returns the status of the SCNetworkConnection.
<<<<<<< HEAD
		A status is one of the following values:
<pre>
@textblock
&#32
	kSCNetworkConnectionInvalid
	kSCNetworkConnectionDisconnected
	kSCNetworkConnectionConnecting
	kSCNetworkConnectionDisconnecting
	kSCNetworkConnectionConnected
@/textblock
</pre>
	@param connection The SCNetworkConnection to obtain status from.
	@result Returns the status value.
}
function SCNetworkConnectionGetStatus( connection: SCNetworkConnectionRef ): SCNetworkConnectionStatus; external name '_SCNetworkConnectionGetStatus';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
		A status is one of the following values :
		    kSCNetworkConnectionInvalid
		    kSCNetworkConnectionDisconnected
		    kSCNetworkConnectionConnecting
		    kSCNetworkConnectionDisconnecting
		    kSCNetworkConnectionConnected

	@param connection The SCNetworkConnection to obtain status from.
	@result The status value.
}
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionGetStatus( connection: SCNetworkConnectionRef ): SCNetworkConnectionStatus; external name '_SCNetworkConnectionGetStatus';
>>>>>>> graemeg/fixes_2_2


{!
	@function SCNetworkConnectionCopyExtendedStatus
	@discussion Returns the extended status of the connection.
		An extended status dictionary contains specific dictionaries
		describing the status for each subcomponent of the service.

<<<<<<< HEAD
		For example, a status dictionary will contain the following
		sub-dictionaries, keys, and values:
<pre>
@textblock
&#32
	IPv4  : Addresses      : the assigned IP address.
&#32
	PPP   : Status         : the PPP-specific status of type
				 SCNetworkConnectionPPPStatus.
&#32
		LastCause      : Available when the status is "Disconnected"
				 and contains the last error associated with
				 connecting or disconnecting.
&#32
		ConnectTime    : the time when the connection was
				 established.
&#32
	Modem : ConnectSpeed   : the speed of the modem connection
				 in bits/second.
&#32
	IPSec : Status         : the IPSec-specific status of type
				 SCNetworkConnectionIPSecStatus
&#32
		ConnectTime    : the time when the connection was
				 established.

@/textblock
</pre>
		Other dictionaries could be present for PPPoE, PPTP, and L2TP.

		The status dictionary may be extended in the future to contain
		additional information.
	@param connection The SCNetworkConnection to obtain status from.
	@result Returns the status dictionary.
		If NULL is returned, the error can be retrieved using the SCError function.
 }
function SCNetworkConnectionCopyExtendedStatus( connection: SCNetworkConnectionRef ): CFDictionaryRef; external name '_SCNetworkConnectionCopyExtendedStatus';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
		For example, a status dictionary will contain the following dictionaries:

		IPv4:
		    IPaddress: IP address used.

		PPP:
		    Status: PPP specific status of type SCNetworkConnectionPPPStatus.
		    LastCause: Available when status is Disconnected.
				Contains the last error of disconnection.
		    ConnectTime: time when the connection happened
		    MaxTime: maximum time for this connection

		Modem:
		    ConnectionSpeed:   Speed of the modem connection in bits/s

		Other dictionaries could be present for PPPoE, PPTP and L2TP.

		The status dictionary can be extended as needed in the future
		to contain additional information.

	@param connection The SCNetworkConnection to obtain status from.
	@result The status dictionary.
		If NULL is returned, the error can be retrieved with SCError().
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionCopyExtendedStatus( connection: SCNetworkConnectionRef ): CFDictionaryRef; external name '_SCNetworkConnectionCopyExtendedStatus';
>>>>>>> graemeg/fixes_2_2


{!
	@function SCNetworkConnectionCopyStatistics
	@discussion Returns the statistics of the SCNetworkConnection.
		A statistic dictionary contains specific dictionaries
<<<<<<< HEAD
		with statistics for each subcomponent of the service.

		For example, a statistics dictionary will contain the following
		sub-dictionaries, keys, and values:
<pre>
@textblock
&#32
	PPP : BytesIn    :
	PPP : BytesOut   : Contains the number of bytes going up into
			   (or coming out of) the network stack for
			   any networking protocol without the PPP
			   headers and trailers.
&#32
	PPP : PacketsIn  :
	PPP : PacketsOut : Contains the number of packets going up into
			   (or coming out of) the network stack for
			   any networking protocol without the PPP
			   headers and trailers.
&#32
	PPP : ErrorsIn   :
	PPP : ErrorsOut  : Contains the number of errors going up into
			   (or coming out of) the network stack for
			   any networking protocol without the PPP
			   headers and trailers.
@/textblock
</pre>
		The statistics dictionary may be extended in the future to
		contain additional information.
	@param connection The SCNetworkConnection to obtained statistics from.
	@result Returns the statistics dictionary.
		If NULL is returned, the error can be retrieved using the SCError function.
 }
function SCNetworkConnectionCopyStatistics( connection: SCNetworkConnectionRef ): CFDictionaryRef; external name '_SCNetworkConnectionCopyStatistics';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
		with statistics for each subcomponents of the service.

		For example, a statistic dictionary will contain the following dictionaries:

		PPP: (Bytes,Packets,Errors()In,Out):
		    Statistics at the Network level.
		    Contains the number of bytes, packets, and errors on the PPP interface.
		    For example, BytesIn contains the number of bytes going up
		    into the network stack, for any networking protocol,
		    without the PPP headers and trailers.

		The statistic dictionary can be extended as needed in the future
		to contain additional information.

	@param connection The SCNetworkConnection to obtained statistics from.
	@result The statistics dictionary.
		If NULL is returned, the error can be retrieved with SCError().
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionCopyStatistics( connection: SCNetworkConnectionRef ): CFDictionaryRef; external name '_SCNetworkConnectionCopyStatistics';
>>>>>>> graemeg/fixes_2_2


{!
	@function SCNetworkConnectionStart
<<<<<<< HEAD
	@discussion Starts the connection for the SCNetworkConnection.
		The connection process is asynchronous and the function will
		return immediately.  The connection status can be obtained
		by polling or by callback.  The connection is made with the
		default settings from the administrator.  Some of the settings
		can be overridden for the duration of the connection.  These
		are specified in an options dictionary.  The options dictionary
		uses the same format as a network service defined in the system
		configuration preferences schema.

		Note: Starting and stopping of connections is implicitly
		arbitrated.  Calling SCNetworkConnectionStart on a connection
		already started will indicate that the application has
		interest in the connection and it shouldn't be stopped by
		anyone else.
	@param connection The SCNetworkConnection to start.
	@param userOptions The options dictionary to start the connection with.
		If userOptions is NULL, the default settings will be used.
		If userOptions are specified, they must be in the same format
		as network services stored in the system configuration
		preferences schema.  The options will override the default
		settings defined for the service.

		For security reasons, not all options can be overridden; the
		appropriate merging of all settings will be done before the
		connection is established, and inappropriate options will be
		ignored.
	@param linger This parameter indicates whether or not the connection
		can stay around when the application no longer has interest
		in it.  A typical application should pass FALSE, and the
		connection will be automatically stopped when the reference
		is released or if the application quits.  If the application
		passes TRUE, the application can release the reference or
		exit and the connection will be maintained until a timeout
		event, until a specific stop request occurs, or until an
		error is encountered.
	@result Returns TRUE if the connection was correctly started (the
		actual connection is not established yet, and the connection
		status needs to be periodically checked); FALSE if the
		connection request was not started.  The error must be
		retrieved from the SCError function.
 }
function SCNetworkConnectionStart( connection: SCNetworkConnectionRef; userOptions: CFDictionaryRef; linger: Boolean ): Boolean; external name '_SCNetworkConnectionStart';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
	@discussion Start the connection for the SCNetworkConnection.
		The connection process is asynchronous and the function will
		return immediately. The connection status can be obtain by polling or
		by callback.
		The connection is done with the default settings from the administrator.
		Some of the settings can be overridden for the duration of
		the connection. They are given in an option dictionary.
		The options dictionary is in the format of a Network Service
		as described in SystemConfiguration.

		Note: Starting and stopping of connections is implicitely arbitrated.
		Calling Start on a connection already started will indicate
		that the application has interest in the connection and it shouldn't
		be stopped by anyone else.

	@param connection The SCNetworkConnection to start.
	@param userOptions The options dictionary to start the connection with.
		If userOptions is NULL, the default settings will be used.
		If userOptions are specified, they must be in the SystemConfiguration format.
		The options will override the default settings defined for the service.

		For security reasons, not all the options can be overridden, the appropriate merging
		of all the settings will be done before the connection is established,
		and inappropriate options will be ignored.

	@param linger This parameter indicates whether or not the connection can stay around
		when the application no longer has interest in it.
		Typical application should pass FALSE, and the Stop function will
		automatically be called when the reference is released or if the application quits.
		If the application passes TRUE, the application can release the reference
		or exit and the Stop function will not be called.

	@result TRUE if the connection was correctly started. The actual connection is not established yet,
		and the connection status needs to be periodically checked.
		FALSE if the connection request didn't start. Error must be taken
		from SCError().
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionStart( connection: SCNetworkConnectionRef; userOptions: CFDictionaryRef; linger: Boolean ): Boolean; external name '_SCNetworkConnectionStart';
>>>>>>> graemeg/fixes_2_2


{!
	@function SCNetworkConnectionStop
<<<<<<< HEAD
	@discussion Stops the connection for the SCNetworkConnection.
		The disconnection process is asynchronous and the function
		will return immediately.  The connection status can be
		obtained by polling or by callback.  This function performs
		an arbitrated stop of the connection.  If several applications
		have marked their interest in the connection, by calling
		SCNetworkConnectionStart, the call will succeed but the
		actual connection will be maintained until the last interested
		application calls SCNetworkConnectionStop.

		In certain cases, you might want to stop the connection anyway.
		In these cases, you set the forceDisconnect argument to TRUE.
	@param connection The SCNetworkConnection to stop.
	@result Returns TRUE if the disconnection request succeeded;
		FALSE if the disconnection request failed.
		The error must be retrieved from the SCError function.
 }
function SCNetworkConnectionStop( connection: SCNetworkConnectionRef; forceDisconnect: Boolean ): Boolean; external name '_SCNetworkConnectionStop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)


{!
	@function SCNetworkConnectionCopyUserOptions
	@discussion Copies the user options used to start the connection.
		This is a mechanism a client can use to retrieve the user options
		previously passed to the SCNetworkConnectionStart function.
	@param connection The SCNetworkConnection to obtain options from.
	@result Returns the service dictionary containing the connection options.
		The dictionary can be empty if no user options were used.
		If NULL is returned, the error can be retrieved using the SCError function.
 }
function SCNetworkConnectionCopyUserOptions( connection: SCNetworkConnectionRef ): CFDictionaryRef; external name '_SCNetworkConnectionCopyUserOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
	@discussion Stop the connection for the SCNetworkConnection.
		The disconnection process is asynchronous and the function will
		return immediately. The connection status can be obtain by polling or
		by callback.
		This function performs an arbitrated stop of the connection.
		If several applications have marked their interest in the connection,
		by calling SCNetworkConnectionStart, the call will succeed but the the actual
		connection will be maintained until the last interested application calls stop.

		In certain cases, you might want to stop the connection anyway, and
		SCNetworkConnectionStop with forceDisconnect argument can be used.

	@param connection The SCNetworkConnection to stop.
	@result TRUE if the disconnection request succeeded.
		FALSE if the disconnection request failed. Error must be taken from SCError().
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionStop( connection: SCNetworkConnectionRef; forceDisconnect: Boolean ): Boolean; external name '_SCNetworkConnectionStop';


{!
	@function SCNetworkConnectionCopyCurrentOptions
	@discussion Copy the user options used to start the connection.
		This is a mechanism for a client to retrieve the user options
		previously passed to the SCNetworkConnectionStart function.
	@param connection The SCNetworkConnection to obtain options from.
	@result The service dictionary containing the connection options.
		The dictionary can be empty if no user options were used.
		If NULL is returned, the error can be retrieved with SCError().
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionCopyUserOptions( connection: SCNetworkConnectionRef ): CFDictionaryRef; external name '_SCNetworkConnectionCopyUserOptions';
>>>>>>> graemeg/fixes_2_2


{!
	@function SCNetworkConnectionScheduleWithRunLoop
<<<<<<< HEAD
	@discussion Schedules a connection with the run loop.
	@param connection The SCNetworkConnection to schedule.
	@param runLoop The run loop to schedule with.
	@param runLoopMode The run loop mode.
	@result Returns TRUE if the connection is scheduled successfully;
		FALSE if the scheduling failed.
		The error can be retrieved using the SCError function.
 }
function SCNetworkConnectionScheduleWithRunLoop( connection: SCNetworkConnectionRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCNetworkConnectionScheduleWithRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
	@discussion Schedule a connection with the Run Loop.
	@param connection The SCNetworkConnection to schedule.
	@param runLoop The runloop to schedule with.
	@param runLoopMode The runloop mode.
	@result TRUE if success.
		FALSE if failed. The error can be retrieved with SCError().
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionScheduleWithRunLoop( connection: SCNetworkConnectionRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCNetworkConnectionScheduleWithRunLoop';
>>>>>>> graemeg/fixes_2_2


{!
	@function SCNetworkConnectionUnscheduleFromRunLoop
<<<<<<< HEAD
	@discussion Unschedules a connection from the run loop.
	@param connection The SCNetworkConnection to unschedule.
	@param runLoop The run loop to unschedule from.
	@param runLoopMode The run loop mode.
	@result Returns TRUE if the connection is unscheduled successfully;
		FALSE if the unscheduling failed.
		The error can be retrieved using the SCError function.
 }
function SCNetworkConnectionUnscheduleFromRunLoop( connection: SCNetworkConnectionRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCNetworkConnectionUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
{$ifc not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
{$ifc not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
{$ifc not TARGET_OS_IPHONE}
>>>>>>> origin/cpstrnew
{!
	 @function SCNetworkConnectionSetDispatchQueue
	 @discussion Caller provides a dispatch queue on which the callback contained in connection will run.
	 @param connection The SCNetworkConnection to notify.
	 @param queue The libdispatch queue to run the callback on.
		Pass NULL to disable notifications, and release queue.
	 @result Returns TRUE if the notifications have been enabled/disabled as desired;
		 FALSE if not.
		 The error can be retrieved using the SCError function.
 }
function SCNetworkConnectionSetDispatchQueue( connection: SCNetworkConnectionRef; queue: dispatch_queue_t ): Boolean; external name '_SCNetworkConnectionSetDispatchQueue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$endc} {not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_OS_IPHONE}
>>>>>>> origin/cpstrnew

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
	@discussion Unschedule a connection from the Run Loop.
	@param connection The SCNetworkConnection to unschedule.
	@param runLoop The runloop to unschedule from.
	@param runLoopMode The runloop mode.
	@result TRUE if success.
		FALSE if failed. The error can be retrieved with SCError().
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkConnectionUnscheduleFromRunLoop( connection: SCNetworkConnectionRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCNetworkConnectionUnscheduleFromRunLoop';

end.
>>>>>>> graemeg/fixes_2_2
