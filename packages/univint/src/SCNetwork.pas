{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000, 2001, 2003-2009 Apple Inc. All rights reserved.
=======
 * Copyright (c) 2000, 2001, 2003-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2003-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2003-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2003-2008 Apple Inc. All rights reserved.
>>>>>>> origin/cpstrnew
=======
 * Copyright (c) 2000-2003 Apple Computer, Inc. All rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
 * Copyright (c) 2000-2003 Apple Computer, Inc. All rights reserved.
>>>>>>> origin/fixes_2_2
=======
 * Copyright (c) 2000, 2001, 2003-2008 Apple Inc. All rights reserved.
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

unit SCNetwork;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
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
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
=======
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
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
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,CFBase,MacOSXPosix;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
{$ALIGN POWER}

{!
	@header SCNetwork
	@discussion The SCNetwork API contains functions an application can
		use to determine remote host reachability and notify the
		system of configuration changes.

		The two SCNetworkCheckReachability functions allow an
		application to determine the status of the system's current
		network configuration and the reachability of a target host
		or address.

		"Reachability" reflects whether a data packet, sent by an
		application into the network stack, can leave the local
		computer.  Note that reachability does <i>not</i> guarantee
		that the data packet will actually be received by the host.
=======
=======
>>>>>>> origin/fixes_2_2
{$ALIGN MAC68K}

{!
	@header SCNetwork

	SCNetworkCheckReachabilityXXX()
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ALIGN POWER}

{!
	@header SCNetwork
	@discussion The SCNetwork API contains functions an application can
		use to determine remote host reachability and notify the
		system of configuration changes.

		The two SCNetworkCheckReachability functions allow an
		application to determine the status of the system's current
		network configuration and the reachability of a target host
		or address.

<<<<<<< HEAD
	This API sends a notification to interested network configuration
	agents to retry	their configuraton immediately. For example, calling
	this API will cause the DHCP client to contact the DHCP server
	immediately rather than waiting until its timeout has expired.
	The utility of this API is to allow the caller to give a hint to
	the system that the network infrastructure/configuration has changed.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		"Reachability" reflects whether a data packet, sent by an
		application into the network stack, can leave the local
		computer.  Note that reachability does <i>not</i> guarantee
		that the data packet will actually be received by the host.
>>>>>>> origin/fixes_2.4
 }

{!
	@enum SCNetworkConnectionFlags
	@discussion Flags that indicate whether the specified network
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		nodename or address is reachable, whether a connection is
		required, and whether some user intervention may be required
		when establishing a connection.

		Note: the SCNetworkConnection flags have been deprecated
		in favor of the newer SCNetworkReachability flags defined
		in SCNetworkReachability.h.
	@constant kSCNetworkFlagsTransientConnection
		This flag indicates that the specified nodename or address can
		be reached via a transient connection, such as PPP.
	@constant kSCNetworkFlagsReachable
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration.
	@constant kSCNetworkFlagsConnectionRequired
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.

		As an example, this status would be returned for a dialup
		connection that was not currently active, but could handle
		network traffic for the target system.
	@constant kSCNetworkFlagsConnectionAutomatic
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.  Any traffic directed
		to the specified name or address will initiate the connection.
	@constant kSCNetworkFlagsInterventionRequired
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.  In addition, some
		form of user intervention will be required to establish this
		connection, such as providing a password, an authentication
		token, etc.

		Note: At the present time, this flag will only be returned
		in the case where you have a dial-on-traffic configuration
		(ConnectionAutomatic), where an attempt to connect has
		already been made, and where some error (e.g. no dial tone,
		no answer, bad password, ...) was encountered during the
		automatic connection attempt.  In this case the PPP controller
		will stop attempting to establish a connection until the user
		has intervened.
	@constant kSCNetworkFlagsIsLocalAddress
		This flag indicates that the specified nodename or address
		is one associated with a network interface on the current
		system.
	@constant kSCNetworkFlagsIsDirect
		This flag indicates that network traffic to the specified
		nodename or address will not go through a gateway, but is
		routed directly to one of the interfaces in the system.
 }
const
	kSCNetworkFlagsTransientConnection = 1 shl 0;
	kSCNetworkFlagsReachable = 1 shl 1;
	kSCNetworkFlagsConnectionRequired = 1 shl 2;
	kSCNetworkFlagsConnectionAutomatic = 1 shl 3;
	kSCNetworkFlagsInterventionRequired = 1 shl 4;
	kSCNetworkFlagsIsLocalAddress = 1 shl 16;
	kSCNetworkFlagsIsDirect = 1 shl 17;
type
	SCNetworkConnectionFlags = UInt32;

{ until the __IPHONE_NA are automatically translated }
{$ifc TARGET_OS_MAC}
=======
=======
>>>>>>> origin/fixes_2_2
		nodename/address is reachable, requires a connection,
		requires some user intervention in establishing the
		connection, and whether the calling application must
		initiate the connection using the (TBD???) API.
=======
		nodename or address is reachable, whether a connection is
		required, and whether some user intervention may be required
		when establishing a connection.
>>>>>>> origin/fixes_2.4

		Note: the SCNetworkConnection flags have been deprecated
		in favor of the newer SCNetworkReachability flags defined
		in SCNetworkReachability.h.
	@constant kSCNetworkFlagsTransientConnection
		This flag indicates that the specified nodename or address can
		be reached via a transient connection, such as PPP.
	@constant kSCNetworkFlagsReachable
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration.
	@constant kSCNetworkFlagsConnectionRequired
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.

		As an example, this status would be returned for a dialup
		connection that was not currently active, but could handle
		network traffic for the target system.
	@constant kSCNetworkFlagsConnectionAutomatic
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.  Any traffic directed
		to the specified name or address will initiate the connection.
	@constant kSCNetworkFlagsInterventionRequired
		This flag indicates that the specified nodename or address can
		be reached using the current network configuration, but a
		connection must first be established.  In addition, some
		form of user intervention will be required to establish this
		connection, such as providing a password, an authentication
		token, etc.

		Note: At the present time, this flag will only be returned
		in the case where you have a dial-on-traffic configuration
		(ConnectionAutomatic), where an attempt to connect has
		already been made, and where some error (e.g. no dial tone,
		no answer, bad password, ...) was encountered during the
		automatic connection attempt.  In this case the PPP controller
		will stop attempting to establish a connection until the user
		has intervened.
	@constant kSCNetworkFlagsIsLocalAddress
		This flag indicates that the specified nodename or address
		is one associated with a network interface on the current
		system.
	@constant kSCNetworkFlagsIsDirect
		This flag indicates that network traffic to the specified
		nodename or address will not go through a gateway, but is
		routed directly to one of the interfaces in the system.
 }
const
	kSCNetworkFlagsTransientConnection = 1 shl 0;
	kSCNetworkFlagsReachable = 1 shl 1;
	kSCNetworkFlagsConnectionRequired = 1 shl 2;
	kSCNetworkFlagsConnectionAutomatic = 1 shl 3;
	kSCNetworkFlagsInterventionRequired = 1 shl 4;
	kSCNetworkFlagsIsLocalAddress = 1 shl 16;
	kSCNetworkFlagsIsDirect = 1 shl 17;
type
	SCNetworkConnectionFlags = UInt32;
<<<<<<< HEAD
const
	kSCNetworkFlagsTransientConnection	= 1 shl 0;
	kSCNetworkFlagsReachable		= 1 shl 1;
	kSCNetworkFlagsConnectionRequired	= 1 shl 2;
	kSCNetworkFlagsConnectionAutomatic	= 1 shl 3;
	kSCNetworkFlagsInterventionRequired	= 1 shl 4;
	kSCNetworkFlagsIsLocalAddress		= 1 shl 16;
	kSCNetworkFlagsIsDirect			= 1 shl 17;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ until the __IPHONE_NA are automatically translated }
{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{!
	@function SCNetworkCheckReachabilityByAddress
	@discussion Determines if the given network address is
		reachable using the current network configuration.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

		Note: this API has been deprecated but you can
		      get equivalent results with :
<pre>
	SCNetworkReachabiltyRef   target;
	SCNetworkReachabiltyFlags flags = 0;
	Boolean                   ok;

	target = SCNetworkReachabilityCreateWithAddress(NULL, address);
	ok = SCNetworkReachabilityGetFlags(target, &flags);
	CFRelease(target);
</pre>
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	@param address The network address of the desired host.
	@param addrlen The length, in bytes, of the address.
	@param flags A pointer to memory that will be filled with a
		set of SCNetworkConnectionFlags detailing the reachability
		of the specified address.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@result Returns TRUE if the network connection flags are valid;
		FALSE if the status could not be determined.
 }
function SCNetworkCheckReachabilityByAddress( address: sockaddr_ptr; addrlen: socklen_t; var flags: SCNetworkConnectionFlags ): Boolean; external name '_SCNetworkCheckReachabilityByAddress';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{!
	@function SCNetworkCheckReachabilityByName
	@discussion Determines if the given network host or node name is
		reachable using the current network configuration.

		Note: this API has been deprecated but you can
		      get equivalent results with :
<pre>
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCNetworkReachabilityRef   target;
	SCNetworkReachabilityFlags flags = 0;
=======
	SCNetworkReachabiltyRef   target;
	SCNetworkReachabiltyFlags flags = 0;
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkReachabiltyRef   target;
	SCNetworkReachabiltyFlags flags = 0;
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkReachabiltyRef   target;
	SCNetworkReachabiltyFlags flags = 0;
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkReachabiltyRef   target;
	SCNetworkReachabiltyFlags flags = 0;
>>>>>>> origin/cpstrnew
	Boolean                   ok;

	target = SCNetworkReachabilityCreateWithName(NULL, name);
	ok = SCNetworkReachabilityGetFlags(target, &flags);
	CFRelease(target);
</pre>
	@param nodename The node name of the desired host. This name would
		be the same as that passed to the gethostbyname(3) or
		getaddrinfo(3) functions.
	@param flags A pointer to memory that will be filled with a
		set of SCNetworkConnectionFlags detailing the reachability
		of the specified node name.
	@result Returns TRUE if the network connection flags are valid;
		FALSE if the status could not be determined.
 }
function SCNetworkCheckReachabilityByName( nodename: ConstCStringPtr; var flags: SCNetworkConnectionFlags ): Boolean; external name '_SCNetworkCheckReachabilityByName';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

=======
=======
>>>>>>> origin/fixes_2_2
	@result TRUE if the network connection flags are valid; FALSE if the
		status could not be determined.
=======
	@result Returns TRUE if the network connection flags are valid;
		FALSE if the status could not be determined.
>>>>>>> origin/fixes_2.4
 }
function SCNetworkCheckReachabilityByAddress( address: sockaddr_ptr; addrlen: socklen_t; var flags: SCNetworkConnectionFlags ): Boolean; external name '_SCNetworkCheckReachabilityByAddress';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{!
	@function SCNetworkCheckReachabilityByName
	@discussion Determines if the given network host or node name is
		reachable using the current network configuration.

		Note: this API has been deprecated but you can
		      get equivalent results with :
<pre>
	SCNetworkReachabiltyRef   target;
	SCNetworkReachabiltyFlags flags = 0;
	Boolean                   ok;

	target = SCNetworkReachabilityCreateWithName(NULL, name);
	ok = SCNetworkReachabilityGetFlags(target, &flags);
	CFRelease(target);
</pre>
	@param nodename The node name of the desired host. This name would
		be the same as that passed to the gethostbyname(3) or
		getaddrinfo(3) functions.
	@param flags A pointer to memory that will be filled with a
		set of SCNetworkConnectionFlags detailing the reachability
		of the specified node name.
	@result Returns TRUE if the network connection flags are valid;
		FALSE if the status could not be determined.
 }
<<<<<<< HEAD
function SCNetworkCheckReachabilityByName( nodename: CStringPtr; var flags: SCNetworkConnectionFlags ): Boolean; external name '_SCNetworkCheckReachabilityByName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SCNetworkCheckReachabilityByName( nodename: ConstCStringPtr; var flags: SCNetworkConnectionFlags ): Boolean; external name '_SCNetworkCheckReachabilityByName';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

>>>>>>> origin/fixes_2.4
{!
	@function SCNetworkInterfaceRefreshConfiguration
	@discussion Sends a notification to interested configuration agents
		to have them immediately retry their configuration over a
		particular network interface.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		Note: This function must be invoked by root (uid == 0).
	@param ifName The BSD name of the network interface, such as
		CFSTR("en0").
	@result Returns TRUE if the notification was sent; FALSE otherwise.
	@deprecated in version 10.4. Replaced with SCNetworkInterfaceForceConfigurationRefresh.
 }
function SCNetworkInterfaceRefreshConfiguration( ifName: CFStringRef ): Boolean; external name '_SCNetworkInterfaceRefreshConfiguration';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)

{$endc} { TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
		Note: This API must be invoked by root (uid == 0).
=======
>>>>>>> origin/fixes_2.4

		Note: This function must be invoked by root (uid == 0).
	@param ifName The BSD name of the network interface, such as
		CFSTR("en0").
	@result Returns TRUE if the notification was sent; FALSE otherwise.
	@deprecated in version 10.4. Replaced with SCNetworkInterfaceForceConfigurationRefresh.
 }
function SCNetworkInterfaceRefreshConfiguration( ifName: CFStringRef ): Boolean; external name '_SCNetworkInterfaceRefreshConfiguration';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)

{$endc} { TARGET_OS_MAC}
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
