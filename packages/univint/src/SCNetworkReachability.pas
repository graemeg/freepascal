{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2003-2005, 2008-2010 Apple Inc. All rights reserved.
=======
 * Copyright (c) 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> origin/cpstrnew
=======
 * Copyright (c) 2003 Apple Computer, Inc. All rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
 * Copyright (c) 2003 Apple Computer, Inc. All rights reserved.
>>>>>>> origin/fixes_2_2
=======
 * Copyright (c) 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
=======
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY of ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES of MERCHANTABILITY,
>>>>>>> graemeg/fixes_2_2
=======
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY of ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES of MERCHANTABILITY,
>>>>>>> origin/fixes_2_2
=======
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
>>>>>>> origin/fixes_2.4
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 }
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2_2
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit SCNetworkReachability;
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
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
=======
>>>>>>> origin/cpstrnew
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
=======
>>>>>>> origin/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
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
	{$setc TARGET_CPU_ARM64 := FALSE}
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
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
>>>>>>> origin/fixes_2.4
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
uses MacTypes,CFBase,SCNetwork,MacOSXPosix,CFRunLoop;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{!
	@header SCNetworkReachability
	@discussion The SCNetworkReachability API allows an application to
		determine the status of a system's current network
		configuration and the reachability of a target host.
		In addition, reachability can be monitored with notifications
		that are sent when the status has changed.

		"Reachability" reflects whether a data packet, sent by
		an application into the network stack, can leave the local
		computer.
		Note that reachability does <i>not</i> guarantee that the data
		packet will actually be received by the host.
=======
=======
>>>>>>> origin/fixes_2_2
{$ALIGN MAC68K}

{!
	@header SCNetworkReachability
	The SCNetworkReachabilityXXX() APIs allow an application to determine the status
	of a system's current network configuration and the reachability
	of a target host.  In addition, the reachability can be monitored
	with a notification being provided when/if the status has changed.

	The term "reachable" reflects whether a data packet, sent by
	an application into the network stack, can be sent to the
	the target host/address.  Please note that there is no
	guarantee that the data packet will actually be received by
	the host.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{!
	@header SCNetworkReachability
	@discussion The SCNetworkReachability API allows an application to
		determine the status of a system's current network
		configuration and the reachability of a target host.
		In addition, reachability can be monitored with notifications
		that are sent when the status has changed.

		"Reachability" reflects whether a data packet, sent by
		an application into the network stack, can leave the local
		computer.
		Note that reachability does <i>not</i> guarantee that the data
		packet will actually be received by the host.
>>>>>>> origin/fixes_2.4
 }

{!
	@typedef SCNetworkReachabilityRef
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion This is the handle to a network address or name.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCNetworkReachabilityRef = ^__SCNetworkReachability; { an opaque type }
	__SCNetworkReachability = record end;
=======
	SCNetworkReachabilityRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkReachabilityRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkReachabilityRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkReachabilityRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
	@discussion This is the handle to a network address/name.
 }
type
	SCNetworkReachabilityRef    = ^SInt32; { an opaque 32-bit type }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	@discussion This is the handle to a network address or name.
 }
type
	SCNetworkReachabilityRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


{!
	@typedef SCNetworkReachabilityContext
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	Structure containing user-specified data and callbacks for SCNetworkReachability.
	@field version The version number of the structure type being passed
		in as a parameter to the SCDynamicStore creation function.
		This structure is version 0.
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
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
type
	SCNetworkReachabilityContext = record
    version: CFIndex;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    info: UnivPtr;
		retain: function( info: UnivPtr ): UnivPtr;
		release: procedure( info: UnivPtr );
		copyDescription: function( info: UnivPtr ): CFStringRef;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
    info: Ptr;
		retain: function( info: Ptr ): Ptr;
		release: procedure( info: Ptr );
		copyDescription: function( info: Ptr ): CFStringRef;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
	SCNetworkReachabilityContextPtr = ^SCNetworkReachabilityContext;

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	@enum SCNetworkReachabilityFlags
	@discussion Flags that indicate whether the specified network
		nodename or address is reachable, whether a connection is
		required, and whether some user intervention may be required
		when establishing a connection.
	@constant kSCNetworkReachabilityFlagsTransientConnection
		This flag indicates that the specified nodename or address can
		be reached via a transient connection, such as PPP.
	@constant kSCNetworkReachabilityFlagsReachable
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration.
	@constant kSCNetworkReachabilityFlagsConnectionRequired
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.

		As an example, this status would be returned for a dialup
		connection that was not currently active, but could handle
		network traffic for the target system.
	@constant kSCNetworkReachabilityFlagsConnectionOnTraffic
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.  Any traffic directed
		to the specified name or address will initiate the connection.

		Note: this flag was previously named kSCNetworkReachabilityFlagsConnectionAutomatic
	@constant kSCNetworkReachabilityFlagsInterventionRequired
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.  In addition, some
		form of user intervention will be required to establish this
		connection, such as providing a password, an authentication
		token, etc.

		Note: At the present time, this flag will only be returned
		in the case where you have a dial-on-traffic configuration
		(ConnectionOnTraffic), where an attempt to connect has
		already been made, and where some error (e.g. no dial tone,
		no answer, bad password, ...) was encountered during the
		automatic connection attempt.  In this case the PPP controller
		will stop attempting to establish a connection until the user
		has intervened.
	@constant kSCNetworkReachabilityFlagsConnectionOnDemand
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.
		The connection will be established "On Demand" by the
		CFSocketStream APIs.
		Other APIs will not establish the connection.
	@constant kSCNetworkReachabilityFlagsIsLocalAddress
		This flag indicates that the specified nodename or address
		is one associated with a network interface on the current
		system.
	@constant kSCNetworkReachabilityFlagsIsDirect
		This flag indicates that network traffic to the specified
		nodename or address will not go through a gateway, but is
		routed directly to one of the interfaces in the system.
#if	TARGET_OS_IPHONE
	@constant kSCNetworkReachabilityFlagsIsWWAN
		This flag indicates that the specified nodename or address can
		be reached via an EDGE, GPRS, or other "cell" connection.
#endif	// TARGET_OS_IPHONE
 }
const
	kSCNetworkReachabilityFlagsTransientConnection = 1 shl 0;
	kSCNetworkReachabilityFlagsReachable = 1 shl 1;
	kSCNetworkReachabilityFlagsConnectionRequired = 1 shl 2;
	kSCNetworkReachabilityFlagsConnectionOnTraffic = 1 shl 3;
	kSCNetworkReachabilityFlagsInterventionRequired = 1 shl 4;
// #if	(__MAC_OS_X_VERSION_MIN_REQUIRED >= 1060) || (__IPHONE_OS_VERSION_MIN_REQUIRED >= 30000) || TARGET_IPHONE_SIMULATOR
// Only on  Mac OS X 10.6+, iPhoneOS 3.0+, iPhoneSimulator
	kSCNetworkReachabilityFlagsConnectionOnDemand = 1 shl 5;
//#endif
	kSCNetworkReachabilityFlagsIsLocalAddress = 1 shl 16;
	kSCNetworkReachabilityFlagsIsDirect = 1 shl 17;
{$ifc	TARGET_OS_IPHONE}
	kSCNetworkReachabilityFlagsIsWWAN = 1 shl 18;
{$endc} {TARGET_OS_IPHONE}

	kSCNetworkReachabilityFlagsConnectionAutomatic = kSCNetworkReachabilityFlagsConnectionOnTraffic;
type
	SCNetworkReachabilityFlags = UInt32;

{!
<<<<<<< HEAD
	@typedef SCNetworkReachabilityCallBack
	@discussion Type of the callback function used when the
		reachability of a network address or name changes.
	@param target The SCNetworkReachability reference being monitored
		for changes.
	@param flags The new SCNetworkReachabilityFlags representing the
		reachability status of the network address/name.
	@param info A C pointer to a user-specified block of data.
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
type SCNetworkReachabilityCallBack = procedure( target: SCNetworkReachabilityRef; flags: SCNetworkReachabilityFlags; info: UnivPtr );
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	@typedef SCNetworkReachabilityCallBack
	@discussion Type of the callback function used when the
		reachability of a network address or name changes.
	@param target The SCNetworkReachability reference being monitored
		for changes.
	@param flags The new SCNetworkReachabilityFlags representing the
		reachability status of the network address/name.
	@param info A C pointer to a user-specified block of data.
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
<<<<<<< HEAD
type SCNetworkReachabilityCallBack = procedure( target: SCNetworkReachabilityRef; flags: SCNetworkConnectionFlags; info: UnivPtr );
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type SCNetworkReachabilityCallBack = procedure( target: SCNetworkReachabilityRef; flags: SCNetworkReachabilityFlags; info: UnivPtr );
>>>>>>> origin/fixes_2.4

{!
	@function SCNetworkReachabilityCreateWithAddress
	@discussion Creates a reference to the specified network
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		address.  This reference can be used later to monitor the
		reachability of the target host.
	@param address The address of the desired host.
	@result Returns a reference to the new immutable SCNetworkReachabilityRef.

		 You must release the returned value.
 }
function SCNetworkReachabilityCreateWithAddress( allocator: CFAllocatorRef;  address: sockaddr_ptr ): SCNetworkReachabilityRef; external name '_SCNetworkReachabilityCreateWithAddress';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
		address.  This reference can later be used to monitor
		the reachability of the target host.
=======
		address.  This reference can be used later to monitor the
		reachability of the target host.
>>>>>>> origin/fixes_2.4
	@param address The address of the desired host.
	@result Returns a reference to the new immutable SCNetworkReachabilityRef.

		 You must release the returned value.
 }
<<<<<<< HEAD
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkReachabilityCreateWithAddress( allocator: CFAllocatorRef; address: sockaddr_ptr ): SCNetworkReachabilityRef; external name '_SCNetworkReachabilityCreateWithAddress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SCNetworkReachabilityCreateWithAddress( allocator: CFAllocatorRef;  address: sockaddr_ptr ): SCNetworkReachabilityRef; external name '_SCNetworkReachabilityCreateWithAddress';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCNetworkReachabilityCreateWithAddressPair
	@discussion Creates a reference to the specified network
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		address.  This reference can be used later to monitor the
		reachability of the target host.
=======
		address.  This reference can later be used to monitor
		the reachability of the target host.
>>>>>>> graemeg/fixes_2_2
=======
		address.  This reference can later be used to monitor
		the reachability of the target host.
>>>>>>> origin/fixes_2_2
=======
		address.  This reference can be used later to monitor the
		reachability of the target host.
>>>>>>> origin/fixes_2.4
	@param localAddress The local address associated with a network
		connection.  If NULL, only the remote address is of interest.
	@param remoteAddress The remote address associated with a network
		connection.  If NULL, only the local address is of interest.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@result Returns a reference to the new immutable SCNetworkReachabilityRef.

		 You must release the returned value.
 }
function SCNetworkReachabilityCreateWithAddressPair( allocator: CFAllocatorRef; localAddress: sockaddr_ptr; remoteAddress: sockaddr_ptr ): SCNetworkReachabilityRef; external name '_SCNetworkReachabilityCreateWithAddressPair';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{!
	@function SCNetworkReachabilityCreateWithName
	@discussion Creates a reference to the specified network host or node
		name.  This reference can be used later to monitor the
		reachability of the target host.
	@param nodename The node name of the desired host.
		This name would be the same as that passed to the
		gethostbyname(3) or getaddrinfo(3) functions.
	@result Returns a reference to the new immutable SCNetworkReachabilityRef.

		You must release the returned value.
 }
function SCNetworkReachabilityCreateWithName( allocator: CFAllocatorRef; nodename: ConstCStringPtr ): SCNetworkReachabilityRef; external name '_SCNetworkReachabilityCreateWithName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{!
	@function SCNetworkReachabilityGetTypeID
	@discussion Returns the type identifier of all SCNetworkReachability
		instances.
 }
function SCNetworkReachabilityGetTypeID: CFTypeID; external name '_SCNetworkReachabilityGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@result A reference to the new immutable SCNetworkReachabilityRef.
=======
	@result Returns a reference to the new immutable SCNetworkReachabilityRef.
>>>>>>> origin/fixes_2.4

		 You must release the returned value.
 }
function SCNetworkReachabilityCreateWithAddressPair( allocator: CFAllocatorRef; localAddress: sockaddr_ptr; remoteAddress: sockaddr_ptr ): SCNetworkReachabilityRef; external name '_SCNetworkReachabilityCreateWithAddressPair';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{!
	@function SCNetworkReachabilityCreateWithName
	@discussion Creates a reference to the specified network host or node
		name.  This reference can be used later to monitor the
		reachability of the target host.
	@param nodename The node name of the desired host.
		This name would be the same as that passed to the
		gethostbyname(3) or getaddrinfo(3) functions.
	@result Returns a reference to the new immutable SCNetworkReachabilityRef.

		You must release the returned value.
 }
function SCNetworkReachabilityCreateWithName( allocator: CFAllocatorRef; nodename: ConstCStringPtr ): SCNetworkReachabilityRef; external name '_SCNetworkReachabilityCreateWithName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{!
	@function SCNetworkReachabilityGetTypeID
	@discussion Returns the type identifier of all SCNetworkReachability
		instances.
 }
function SCNetworkReachabilityGetTypeID: CFTypeID; external name '_SCNetworkReachabilityGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{!
	@function SCNetworkReachabilityGetFlags
	@discussion Determines if the given target is reachable using the
		current network configuration.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param target The network reference associated with the address or name
		to be checked for reachability.
	@param flags A pointer to memory that will be filled with the
		SCNetworkReachabilityFlags detailing the reachability
		of the specified target.
	@result Returns TRUE if the network connection flags are valid;
		FALSE if the status could not be determined.
 }
function SCNetworkReachabilityGetFlags( target: SCNetworkReachabilityRef; var flags: SCNetworkReachabilityFlags ): Boolean; external name '_SCNetworkReachabilityGetFlags';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@param target The network reference associated with the address/name
=======
	@param target The network reference associated with the address or name
>>>>>>> origin/fixes_2.4
		to be checked for reachability.
	@param flags A pointer to memory that will be filled with the
		SCNetworkReachabilityFlags detailing the reachability
		of the specified target.
	@result Returns TRUE if the network connection flags are valid;
		FALSE if the status could not be determined.
 }
<<<<<<< HEAD
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function SCNetworkReachabilityGetFlags( target: SCNetworkReachabilityRef; var flags: SCNetworkConnectionFlags ): Boolean; external name '_SCNetworkReachabilityGetFlags';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SCNetworkReachabilityGetFlags( target: SCNetworkReachabilityRef; var flags: SCNetworkReachabilityFlags ): Boolean; external name '_SCNetworkReachabilityGetFlags';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCNetworkReachabilitySetCallback
	@discussion Assigns a client to a target, which receives callbacks
		when the reachability of the target changes.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param target The network reference associated with the address or
		name to be checked for reachability.
	@param callout The function to be called when the reachability of the
		target changes.  If NULL, the current client for the target
		is removed.
	@param context The SCNetworkReachabilityContext associated with
		the callout.  The value may be NULL.
	@result Returns TRUE if the notification client was successfully set.
 }
function SCNetworkReachabilitySetCallback( target: SCNetworkReachabilityRef; callout: SCNetworkReachabilityCallBack; var context: SCNetworkReachabilityContext ): Boolean; external name '_SCNetworkReachabilitySetCallback';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{!
	@function SCNetworkReachabilityScheduleWithRunLoop
	@discussion Schedules the given target with the given run loop and mode.
	@param target The address or name that is set up for asynchronous
		notifications.  Must be non-NULL.
	@param runLoop A reference to a run loop on which the target should
		be scheduled.  Must be non-NULL.
	@param runLoopMode The mode on which to schedule the target.
		Must be non-NULL.
	@result Returns TRUE if the target is scheduled successfully;
		FALSE otherwise.
 }
function SCNetworkReachabilityScheduleWithRunLoop( target: SCNetworkReachabilityRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCNetworkReachabilityScheduleWithRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{!
	@function SCNetworkReachabilityUnscheduleFromRunLoop
	@discussion Unschedules the given target from the given run loop
		and mode.
	@param target The address or name that is set up for asynchronous
		notifications.  Must be non-NULL.
	@param runLoop A reference to a run loop from which the target
		should be unscheduled.  Must be non-NULL.
	@param runLoopMode The mode on which to unschedule the target.
		Must be non-NULL.
	@result Returns TRUE if the target is unscheduled successfully;
		FALSE otherwise.
 }
function SCNetworkReachabilityUnscheduleFromRunLoop( target: SCNetworkReachabilityRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCNetworkReachabilityUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
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
	@function SCNetworkReachabilitySetDispatchQueue
	@discussion Schedules callbacks for the given target on the given
		dispatch queue.
	@param target The address or name that is set up for asynchronous
		notifications.  Must be non-NULL.
	@param queue A libdispatch queue to run the callback on. Pass NULL to disable notifications, and release queue.
	@result Returns TRUE if the target is unscheduled successfully;
		FALSE otherwise.
 }
function SCNetworkReachabilitySetDispatchQueue( target: SCNetworkReachabilityRef; queue: dispatch_queue_t ): Boolean; external name '_SCNetworkReachabilitySetDispatchQueue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} { TARGET_OS_MAC}
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

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	@param target The network reference associated with the address/name
		to be checked for reachability.
	@param callout The function to be called when the reachability of
=======
	@param target The network reference associated with the address or
		name to be checked for reachability.
	@param callout The function to be called when the reachability of the
>>>>>>> origin/fixes_2.4
		target changes.  If NULL, the current client for the target
		is removed.
	@param context The SCNetworkReachabilityContext associated with
		the callout.  The value may be NULL.
	@result Returns TRUE if the notification client was successfully set.
 }
function SCNetworkReachabilitySetCallback( target: SCNetworkReachabilityRef; callout: SCNetworkReachabilityCallBack; var context: SCNetworkReachabilityContext ): Boolean; external name '_SCNetworkReachabilitySetCallback';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{!
	@function SCNetworkReachabilityScheduleWithRunLoop
	@discussion Schedules the given target with the given run loop and mode.
	@param target The address or name that is set up for asynchronous
		notifications.  Must be non-NULL.
	@param runLoop A reference to a run loop on which the target should
		be scheduled.  Must be non-NULL.
	@param runLoopMode The mode on which to schedule the target.
		Must be non-NULL.
	@result Returns TRUE if the target is scheduled successfully;
		FALSE otherwise.
 }
function SCNetworkReachabilityScheduleWithRunLoop( target: SCNetworkReachabilityRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCNetworkReachabilityScheduleWithRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{!
	@function SCNetworkReachabilityUnscheduleFromRunLoop
	@discussion Unschedules the given target from the given run loop
		and mode.
	@param target The address or name that is set up for asynchronous
		notifications.  Must be non-NULL.
	@param runLoop A reference to a run loop from which the target
		should be unscheduled.  Must be non-NULL.
	@param runLoopMode The mode on which to unschedule the target.
		Must be non-NULL.
	@result Returns TRUE if the target is unscheduled successfully;
		FALSE otherwise.
 }
function SCNetworkReachabilityUnscheduleFromRunLoop( target: SCNetworkReachabilityRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCNetworkReachabilityUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{$ifc not TARGET_OS_IPHONE}
{!
	@function SCNetworkReachabilitySetDispatchQueue
	@discussion Schedules callbacks for the given target on the given
		dispatch queue.
	@param target The address or name that is set up for asynchronous
		notifications.  Must be non-NULL.
	@param queue A libdispatch queue to run the callback on. Pass NULL to disable notifications, and release queue.
	@result Returns TRUE if the target is unscheduled successfully;
		FALSE otherwise.
 }
function SCNetworkReachabilitySetDispatchQueue( target: SCNetworkReachabilityRef; queue: dispatch_queue_t ): Boolean; external name '_SCNetworkReachabilitySetDispatchQueue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{$endc} {not TARGET_OS_IPHONE}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
