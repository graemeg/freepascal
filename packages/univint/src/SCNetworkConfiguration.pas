{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2004-2011 Apple Inc. All rights reserved.
=======
 * Copyright (c) 2004-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2004-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2004-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2004-2008 Apple Inc. All rights reserved.
>>>>>>> origin/cpstrnew
=======
 * Copyright (c) 2004-2008 Apple Inc. All rights reserved.
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
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/cpstrnew
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/fixes_2.4
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit SCNetworkConfiguration;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
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


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/fixes_2.4
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

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
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,CFBase,CFArray,CFDictionary,CFNumber,SCPreferences;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{!
	@header SCNetworkConfiguration
	@discussion The SCNetworkConfiguration API provides access to the
		stored network configuration.  The functions include
		providing access to the network capable devices on the
		system, the network sets, network services, and network
		protocols.

		Note: When using the SCNetworkConfiguraiton APIs you must
		keep in mind that in order for any of your changes to be
		committed to permanent storage a call must be made to the
		SCPreferencesCommitChanges function.
 }


{!
	@group Interface configuration
 }

//#pragma mark -
//#pragma mark SCNetworkInterface configuration (typedefs, consts)

{!
	@typedef SCNetworkInterfaceRef
	@discussion This is the type of a reference to an object that represents
		a network interface.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCNetworkInterfaceRef = ^__SCNetworkInterface; { an opaque type }
	__SCNetworkInterface = record end;
=======
	SCNetworkInterfaceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkInterfaceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkInterfaceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkInterfaceRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	SCNetworkInterfaceRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{ until __IPHONE_NA is translated automatically }
{$ifc TARGET_OS_MAC}

{!
	@const kSCNetworkInterfaceType6to4
 }
var kSCNetworkInterfaceType6to4: CFStringRef; external name '_kSCNetworkInterfaceType6to4'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeBluetooth
 }
var kSCNetworkInterfaceTypeBluetooth: CFStringRef; external name '_kSCNetworkInterfaceTypeBluetooth'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeBond
 }
var kSCNetworkInterfaceTypeBond: CFStringRef; external name '_kSCNetworkInterfaceTypeBond'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeEthernet
 }
var kSCNetworkInterfaceTypeEthernet: CFStringRef; external name '_kSCNetworkInterfaceTypeEthernet'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeFireWire
 }
var kSCNetworkInterfaceTypeFireWire: CFStringRef; external name '_kSCNetworkInterfaceTypeFireWire'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeIEEE80211
 }
var kSCNetworkInterfaceTypeIEEE80211: CFStringRef; external name '_kSCNetworkInterfaceTypeIEEE80211'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)	// IEEE 802.11, AirPort

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@const kSCNetworkInterfaceTypeIPSec
=======
 @const kSCNetworkInterfaceTypeIPSec
>>>>>>> graemeg/cpstrnew
=======
 @const kSCNetworkInterfaceTypeIPSec
>>>>>>> graemeg/cpstrnew
=======
 @const kSCNetworkInterfaceTypeIPSec
>>>>>>> graemeg/cpstrnew
=======
 @const kSCNetworkInterfaceTypeIPSec
>>>>>>> origin/cpstrnew
=======
 @const kSCNetworkInterfaceTypeIPSec
>>>>>>> origin/fixes_2.4
 }
var kSCNetworkInterfaceTypeIPSec: CFStringRef; external name '_kSCNetworkInterfaceTypeIPSec'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeIrDA
 }
var kSCNetworkInterfaceTypeIrDA: CFStringRef; external name '_kSCNetworkInterfaceTypeIrDA'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeL2TP
 }
var kSCNetworkInterfaceTypeL2TP: CFStringRef; external name '_kSCNetworkInterfaceTypeL2TP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeModem
 }
var kSCNetworkInterfaceTypeModem: CFStringRef; external name '_kSCNetworkInterfaceTypeModem'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypePPP
 }
var kSCNetworkInterfaceTypePPP: CFStringRef; external name '_kSCNetworkInterfaceTypePPP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypePPTP
 }
var kSCNetworkInterfaceTypePPTP: CFStringRef; external name '_kSCNetworkInterfaceTypePPTP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeSerial
 }
var kSCNetworkInterfaceTypeSerial: CFStringRef; external name '_kSCNetworkInterfaceTypeSerial'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeVLAN
 }
var kSCNetworkInterfaceTypeVLAN: CFStringRef; external name '_kSCNetworkInterfaceTypeVLAN'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceTypeWWAN
 }
var kSCNetworkInterfaceTypeWWAN: CFStringRef; external name '_kSCNetworkInterfaceTypeWWAN'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{ special network interfaces (and types) }

{!
	@const kSCNetworkInterfaceTypeIPv4
 }
var kSCNetworkInterfaceTypeIPv4: CFStringRef; external name '_kSCNetworkInterfaceTypeIPv4'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkInterfaceIPv4
	@discussion A network interface that can used for layering other
		interfaces (e.g. 6to4, IPSec, PPTP, L2TP) over an existing
		IPv4 network.
 }
var kSCNetworkInterfaceIPv4: SCNetworkInterfaceRef; external name '_kSCNetworkInterfaceIPv4'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@group Interface configuration (Bond)
 }

//#pragma mark -

{!
	@typedef SCBondInterfaceRef
	@discussion This is the type of a reference to an object that represents
		an Ethernet Bond interface.
 }
type
	SCBondInterfaceRef = SCNetworkInterfaceRef;

{!
	@typedef SCBondStatusRef
	@discussion This is the type of a reference to an object that represents
		the status of an Ethernet Bond interface.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCBondStatusRef = ^__SCBondStatus; { an opaque type }
	__SCBondStatus = record end;

=======
	SCBondStatusRef = ^SInt32; { an opaque type }

{$ifc not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
	SCBondStatusRef = ^SInt32; { an opaque type }

{$ifc not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
	SCBondStatusRef = ^SInt32; { an opaque type }

{$ifc not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew
=======
	SCBondStatusRef = ^SInt32; { an opaque type }

{$ifc not TARGET_OS_IPHONE}
>>>>>>> origin/cpstrnew
=======
	SCBondStatusRef = ^SInt32; { an opaque type }

{$ifc not TARGET_OS_IPHONE}
>>>>>>> origin/fixes_2.4
{!
	@enum Ethernet Bond Aggregation Status (kSCBondStatusDeviceAggregationStatus) codes
	@discussion Returned status codes.
	@constant kSCBondStatusOK		Enabled, active, running, ...
	@constant kSCBondStatusLinkInvalid	The link state was not valid (i.e. down, half-duplex, wrong speed)
	@constant kSCBondStatusNoPartner	The port on the switch that the device is connected doesn't seem to have 802.3ad Link Aggregation enabled
	@constant kSCBondStatusNotInActiveGroup	We're talking to a partner, but the link aggregation group is different from the one that's active
	@constant kSCBondStatusUnknown		Non-specific failure
 }
const
	kSCBondStatusOK = 0;	{ enabled, active, running, ... }
	kSCBondStatusLinkInvalid = 1;	{ The link state was not valid (i.e. down, half-duplex, wrong speed) }
	kSCBondStatusNoPartner = 2;	{ The port on the switch that the device is connected doesn't seem to have 802.3ad Link Aggregation enabled }
	kSCBondStatusNotInActiveGroup = 3;	{ We're talking to a partner, but the link aggregation group is different from the one that's active }
	kSCBondStatusUnknown = 999;	{ Non-specific failure }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{!
	@const kSCBondStatusDeviceAggregationStatus
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{$endc} {not TARGET_OS_IPHONE}

{!
  @const kSCBondStatusDeviceAggregationStatus
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
 }
var kSCBondStatusDeviceAggregationStatus	{ CFNumber }: CFStringRef; external name '_kSCBondStatusDeviceAggregationStatus'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@const kSCBondStatusDeviceCollecting
=======
  @const kSCBondStatusDeviceCollecting
>>>>>>> graemeg/cpstrnew
=======
  @const kSCBondStatusDeviceCollecting
>>>>>>> graemeg/cpstrnew
=======
  @const kSCBondStatusDeviceCollecting
>>>>>>> graemeg/cpstrnew
=======
  @const kSCBondStatusDeviceCollecting
>>>>>>> origin/cpstrnew
=======
  @const kSCBondStatusDeviceCollecting
>>>>>>> origin/fixes_2.4
 }
var kSCBondStatusDeviceCollecting		{ CFNumber (0 or 1) }: CFStringRef; external name '_kSCBondStatusDeviceCollecting'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@const kSCBondStatusDeviceDistributing
=======
  @const kSCBondStatusDeviceDistributing
>>>>>>> graemeg/cpstrnew
=======
  @const kSCBondStatusDeviceDistributing
>>>>>>> graemeg/cpstrnew
=======
  @const kSCBondStatusDeviceDistributing
>>>>>>> graemeg/cpstrnew
=======
  @const kSCBondStatusDeviceDistributing
>>>>>>> origin/cpstrnew
=======
  @const kSCBondStatusDeviceDistributing
>>>>>>> origin/fixes_2.4
 }
var kSCBondStatusDeviceDistributing	{ CFNumber (0 or 1) }: CFStringRef; external name '_kSCBondStatusDeviceDistributing'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@group Interface configuration (VLAN)
 }

//#pragma mark -

{!
	@typedef SCVLANInterfaceRef
	@discussion This is the type of a reference to an object that represents
		a Virtual LAN (VLAN) interface.
 }
type
	SCVLANInterfaceRef = SCNetworkInterfaceRef;


{!
	@group Protocol configuration
 }

//#pragma mark -
//#pragma mark SCNetworkProtocol configuration (typedefs, consts)

{!
	@typedef SCNetworkProtocolRef
	@discussion This is the type of a reference to an object that represents
		a network protocol.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCNetworkProtocolRef = ^__SCNetworkProtocol; { an opaque type }
	__SCNetworkProtocol = record end;
=======
	SCNetworkProtocolRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkProtocolRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkProtocolRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkProtocolRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	SCNetworkProtocolRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{ network "protocol" types }

{!
	@const kSCNetworkProtocolTypeAppleTalk
 }
var kSCNetworkProtocolTypeAppleTalk: CFStringRef; external name '_kSCNetworkProtocolTypeAppleTalk'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{!
	@const kSCNetworkProtocolTypeDNS
 }
var kSCNetworkProtocolTypeDNS: CFStringRef; external name '_kSCNetworkProtocolTypeDNS'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkProtocolTypeIPv4
 }
var kSCNetworkProtocolTypeIPv4: CFStringRef; external name '_kSCNetworkProtocolTypeIPv4'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkProtocolTypeIPv6
 }
var kSCNetworkProtocolTypeIPv6: CFStringRef; external name '_kSCNetworkProtocolTypeIPv6'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkProtocolTypeProxies
 }
var kSCNetworkProtocolTypeProxies: CFStringRef; external name '_kSCNetworkProtocolTypeProxies'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@const kSCNetworkProtocolTypeSMB
 }
var kSCNetworkProtocolTypeSMB: CFStringRef; external name '_kSCNetworkProtocolTypeSMB'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@group Service configuration
 }

//#pragma mark -
//#pragma mark SCNetworkService configuration (typedefs, consts)

{!
	@typedef SCNetworkServiceRef
	@discussion This is the type of a reference to an object that represents
		a network service.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCNetworkServiceRef = ^__SCNetworkService; { an opaque type }
	__SCNetworkService = record end;
=======
	SCNetworkServiceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkServiceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkServiceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCNetworkServiceRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	SCNetworkServiceRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


{!
	@group Set configuration
 }

//#pragma mark -
//#pragma mark SCNetworkSet configuration (typedefs, consts)

{!
	@typedef SCNetworkSetRef
	@discussion This is the type of a reference to an object that represents
		a network set.
 }
type
	__SCNetworkSet = record end;
	SCNetworkSetRef = ^__SCNetworkSet;



{ --------------------------------------------------------------------------------
 * INTERFACES
 * -------------------------------------------------------------------------------- }

{!
	@group Interface configuration
 }

//#pragma mark -
//#pragma mark SCNetworkInterface configuration (APIs)

{!
	@function SCNetworkInterfaceGetTypeID
	@discussion Returns the type identifier of all SCNetworkInterface instances.
 }
function SCNetworkInterfaceGetTypeID: CFTypeID; external name '_SCNetworkInterfaceGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceCopyAll
	@discussion Returns all network capable interfaces on the system.
	@result The list of interfaces on the system.
		You must release the returned value.
 }
function SCNetworkInterfaceCopyAll: CFArrayRef { of SCNetworkInterfaceRef's }; external name '_SCNetworkInterfaceCopyAll';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetSupportedInterfaceTypes
	@discussion Identify all of the network interface types (e.g. PPP) that
		can be layered on top of this interface.
	@param interface The network interface.
	@result The list of SCNetworkInterface types supported by the interface;
		NULL if no interface types are supported.
 }
function SCNetworkInterfaceGetSupportedInterfaceTypes( interface_: SCNetworkInterfaceRef ): CFArrayRef { of kSCNetworkInterfaceTypeXXX CFStringRef's }; external name '_SCNetworkInterfaceGetSupportedInterfaceTypes';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetSupportedProtocolTypes
	@discussion Identify all of the network protocol types (e.g. IPv4, IPv6) that
		can be layered on top of this interface.
	@param interface The network interface.
	@result The list of SCNetworkProtocol types supported by the interface;
		NULL if no protocol types are supported.
 }
function SCNetworkInterfaceGetSupportedProtocolTypes( interface_: SCNetworkInterfaceRef ): CFArrayRef { of kSCNetworkProtocolTypeXXX CFStringRef's }; external name '_SCNetworkInterfaceGetSupportedProtocolTypes';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceCreateWithInterface
	@discussion Create a new network interface layered on top of another.  For
		example, this function would be used to create a "PPP" interface
		on top of a "modem".
	@param interface The network interface.
	@param interfaceType The type of SCNetworkInterface to be layered on
		top of the provided interface.
	@result A reference to the new SCNetworkInterface.
		You must release the returned value.
 }
function SCNetworkInterfaceCreateWithInterface( interface_: SCNetworkInterfaceRef; interfaceType: CFStringRef ): SCNetworkInterfaceRef; external name '_SCNetworkInterfaceCreateWithInterface';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetBSDName
	@discussion Returns the BSD interface (en0) or device name (modem)
		for the interface.
	@param interface The network interface.
	@result The BSD name associated with the interface (e.g. "en0");
		NULL if no BSD name is available.
 }
function SCNetworkInterfaceGetBSDName( interface_: SCNetworkInterfaceRef ): CFStringRef; external name '_SCNetworkInterfaceGetBSDName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetConfiguration
	@discussion Returns the configuration settings associated with a interface.
	@param interface The network interface.
	@result The configuration settings associated with the interface;
		NULL if no configuration settings are associated with the interface
		or an error was encountered.
 }
function SCNetworkInterfaceGetConfiguration( interface_: SCNetworkInterfaceRef ): CFDictionaryRef; external name '_SCNetworkInterfaceGetConfiguration';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetExtendedConfiguration
	@discussion Returns the configuration settings associated with a interface.
	@param interface The network interface.
	@param extendedType A string representing the type of extended information (e.g. EAPOL).
	@result The configuration settings associated with the interface;
		NULL if no configuration settings are associated with the interface
		or an error was encountered.
 }
function SCNetworkInterfaceGetExtendedConfiguration( interface_: SCNetworkInterfaceRef; extendedType: CFStringRef ): CFDictionaryRef; external name '_SCNetworkInterfaceGetExtendedConfiguration';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetHardwareAddressString
	@discussion Returns a displayable link layer address for the interface.
	@param interface The network interface.
	@result A string representing the hardware (MAC) address for the interface.
 }
function SCNetworkInterfaceGetHardwareAddressString( interface_: SCNetworkInterfaceRef ): CFStringRef; external name '_SCNetworkInterfaceGetHardwareAddressString';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetInterface
	@discussion For layered network interfaces, return the underlying interface.
	@param interface The network interface.
	@result The underlying network interface;
		NULL if this is a leaf interface.
 }
function SCNetworkInterfaceGetInterface( interface_: SCNetworkInterfaceRef ): SCNetworkInterfaceRef; external name '_SCNetworkInterfaceGetInterface';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetInterfaceType
	@discussion Returns the associated network interface type.
	@param interface The network interface.
	@result The interface type.
 }
function SCNetworkInterfaceGetInterfaceType( interface_: SCNetworkInterfaceRef ): CFStringRef; external name '_SCNetworkInterfaceGetInterfaceType';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceGetLocalizedDisplayName
	@discussion Returns the localized name (e.g. "Ethernet", "FireWire") for
		the interface.
	@param interface The network interface.
	@result A localized, display name for the interface;
		NULL if no name is available.
 }
function SCNetworkInterfaceGetLocalizedDisplayName( interface_: SCNetworkInterfaceRef ): CFStringRef; external name '_SCNetworkInterfaceGetLocalizedDisplayName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceSetConfiguration
	@discussion Stores the configuration settings for the interface.
	@param interface The network interface.
	@param config The configuration settings to associate with this interface.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.
 }
function SCNetworkInterfaceSetConfiguration( interface_: SCNetworkInterfaceRef; config: CFDictionaryRef ): Boolean; external name '_SCNetworkInterfaceSetConfiguration';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceSetExtendedConfiguration
	@discussion Stores the configuration settings for the interface.
	@param interface The network interface.
	@param config The configuration settings to associate with this interface.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.
 }
function SCNetworkInterfaceSetExtendedConfiguration( interface_: SCNetworkInterfaceRef; extendedType: CFStringRef; config: CFDictionaryRef ): Boolean; external name '_SCNetworkInterfaceSetExtendedConfiguration';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

//#pragma mark -

{!
	@function SCNetworkInterfaceCopyMediaOptions
	@discussion For the specified network interface, returns information
		about the currently requested media options, the active media
		options, and the media options which are available.
	@param interface The desired network interface.
	@param current A pointer to memory that will be filled with a CFDictionaryRef
		representing the currently requested media options (subtype, options).
		If NULL, the current options will not be returned.
	@param active A pointer to memory that will be filled with a CFDictionaryRef
		representing the active media options (subtype, options).
		If NULL, the active options will not be returned.
	@param available A pointer to memory that will be filled with a CFArrayRef
		representing the possible media options (subtype, options).
		If NULL, the available options will not be returned.
	@param filter A boolean indicating whether the available options should be
		filtered to exclude those options which would not normally be
		requested by a user/admin (e.g. hw-loopback).
	@result TRUE if requested information has been returned.
 }
function SCNetworkInterfaceCopyMediaOptions( interface_: SCNetworkInterfaceRef; var current: CFDictionaryRef; var active: CFDictionaryRef; var available: CFArrayRef; filter: Boolean ): Boolean; external name '_SCNetworkInterfaceCopyMediaOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceCopyMediaSubTypes
	@discussion For the provided interface configuration options, return a list
		of available media subtypes.
	@param available The available options as returned by the
		SCNetworkInterfaceCopyMediaOptions function.
	@result An array of available media subtypes CFString's (e.g. 10BaseT/UTP,
		100baseTX, etc).  NULL if no subtypes are available.
 }
function SCNetworkInterfaceCopyMediaSubTypes( available: CFArrayRef ): CFArrayRef; external name '_SCNetworkInterfaceCopyMediaSubTypes';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceCopyMediaSubTypeOptions
	@discussion For the provided interface configuration options and specific
		subtype, return a list of available media options.
	@param available The available options as returned by the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		SCNetworkInterfaceCopyMediaOptions function.
=======
		NetworkInterfaceCopyMediaOptions function.
>>>>>>> graemeg/cpstrnew
=======
		NetworkInterfaceCopyMediaOptions function.
>>>>>>> graemeg/cpstrnew
=======
		NetworkInterfaceCopyMediaOptions function.
>>>>>>> graemeg/cpstrnew
=======
		NetworkInterfaceCopyMediaOptions function.
>>>>>>> origin/cpstrnew
=======
		NetworkInterfaceCopyMediaOptions function.
>>>>>>> origin/fixes_2.4
	@param subType The subtype
	@result An array of available media options.  Each of the available options
		is returned as an array of CFString's (e.g. <half-duplex>,
		<full-duplex,flow-control>).  NULL if no options are available.
 }
function SCNetworkInterfaceCopyMediaSubTypeOptions( available: CFArrayRef; subType: CFStringRef ): CFArrayRef; external name '_SCNetworkInterfaceCopyMediaSubTypeOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceCopyMTU
	@discussion For the specified network interface, returns information
		about the currently MTU setting and the range of allowable
		values.
	@param interface The desired network interface.
	@param mtu_cur A pointer to memory that will be filled with the current
		MTU setting for the interface.
	@param mtu_min A pointer to memory that will be filled with the minimum
		MTU setting for the interface.  If negative, the minimum setting
		could not be determined.
	@param mtu_max A pointer to memory that will be filled with the maximum
		MTU setting for the interface.  If negative, the maximum setting
		could not be determined.
	@result TRUE if requested information has been returned.
 }
function SCNetworkInterfaceCopyMTU( interface_: SCNetworkInterfaceRef; var mtu_cur: SInt32; var mtu_min: SInt32; var mtu_max: SInt32 ): Boolean; external name '_SCNetworkInterfaceCopyMTU';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceSetMediaOptions
	@discussion For the specified network interface, sets the requested
		media subtype and options.
	@param interface The desired network interface.
	@param subtype The desired media subtype (e.g. "autoselect", "100baseTX", ...).
	@param options The desired media options (e.g. "half-duplex", "full-duplex", ...).
		If NULL, the active options will not be returned.
	@result TRUE if the configuration was updated; FALSE if an error was encountered.
 }
function SCNetworkInterfaceSetMediaOptions( interface_: SCNetworkInterfaceRef; subtype: CFStringRef; options: CFArrayRef ): Boolean; external name '_SCNetworkInterfaceSetMediaOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceSetMTU
	@discussion For the specified network interface, sets the
		requested MTU setting.
	@param interface The desired network interface.
	@param mtu The desired MTU setting for the interface.
	@result TRUE if the configuration was updated; FALSE if an error was encountered.
 }
function SCNetworkInterfaceSetMTU( interface_: SCNetworkInterfaceRef; mtu: SInt32 ): Boolean; external name '_SCNetworkInterfaceSetMTU';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkInterfaceForceConfigurationRefresh
	@discussion Sends a notification to interested network configuration
		agents to immediately retry their configuration. For example,
		calling this function will cause the DHCP client to contact
		the DHCP server immediately rather than waiting until its
		timeout has expired.  The utility of this function is to
		allow the caller to give a hint to the system that the
		network infrastructure or configuration has changed.

		Note: This function requires root (euid==0) privilege or,
		alternatively, you may pass an SCNetworkInterface which
		is derived from a sequence of calls to :

			SCPreferencesCreateWithAuthorization
			SCNetworkSetCopy...
			SCNetworkServiceGetInterface
	@param interface The desired network interface.
	@result Returns TRUE if the notification was sent; FALSE otherwise.
 }
function SCNetworkInterfaceForceConfigurationRefresh( interface_: SCNetworkInterfaceRef ): Boolean; external name '_SCNetworkInterfaceForceConfigurationRefresh';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@group Interface configuration (Bond)
 }

//#pragma mark -
//#pragma mark SCBondInterface configuration (APIs)

{!
	@function SCBondInterfaceCopyAll
	@discussion Returns all Ethernet Bond interfaces on the system.
	@param prefs The "preferences" session.
	@result The list of Ethernet Bond interfaces on the system.
		You must release the returned value.
 }
function SCBondInterfaceCopyAll( prefs: SCPreferencesRef ): CFArrayRef { of SCBondInterfaceRef's }; external name '_SCBondInterfaceCopyAll';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondInterfaceCopyAvailableMemberInterfaces
	@discussion Returns all network capable devices on the system
		that can be added to an Ethernet Bond interface.
	@param prefs The "preferences" session.
	@result The list of interfaces.
		You must release the returned value.
 }
function SCBondInterfaceCopyAvailableMemberInterfaces( prefs: SCPreferencesRef ): CFArrayRef { of SCNetworkInterfaceRef's }; external name '_SCBondInterfaceCopyAvailableMemberInterfaces';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondInterfaceCreate
	@discussion Create a new SCBondInterface interface.
	@param prefs The "preferences" session.
	@result A reference to the new SCBondInterface.
		You must release the returned value.
 }
function SCBondInterfaceCreate( prefs: SCPreferencesRef ): SCBondInterfaceRef; external name '_SCBondInterfaceCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondInterfaceRemove
	@discussion Removes the SCBondInterface from the configuration.
	@param bond The SCBondInterface interface.
	@result TRUE if the interface was removed; FALSE if an error was encountered.
 }
function SCBondInterfaceRemove( bond: SCBondInterfaceRef ): Boolean; external name '_SCBondInterfaceRemove';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondInterfaceGetMemberInterfaces
	@discussion Returns the member interfaces for the specified Ethernet Bond interface.
	@param bond The SCBondInterface interface.
	@result The list of interfaces.
 }
function SCBondInterfaceGetMemberInterfaces( bond: SCBondInterfaceRef ): CFArrayRef { of SCNetworkInterfaceRef's }; external name '_SCBondInterfaceGetMemberInterfaces';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondInterfaceGetOptions
	@discussion Returns the configuration settings associated with a Ethernet Bond interface.
	@param bond The SCBondInterface interface.
	@result The configuration settings associated with the Ethernet Bond interface;
		NULL if no changes to the default configuration have been saved.
 }
function SCBondInterfaceGetOptions( bond: SCBondInterfaceRef ): CFDictionaryRef; external name '_SCBondInterfaceGetOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondInterfaceSetMemberInterfaces
	@discussion Sets the member interfaces for the specified Ethernet Bond interface.
	@param bond The SCBondInterface interface.
	@param members The desired member interfaces.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.
 }
function SCBondInterfaceSetMemberInterfaces (bond: SCBondInterfaceRef; members: CFArrayRef { of SCNetworkInterfaceRef's } ): Boolean; external name '_SCBondInterfaceSetMemberInterfaces'; 
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondInterfaceSetLocalizedDisplayName
	@discussion Sets the localized display name for the specified Ethernet Bond interface.
	@param bond The SCBondInterface interface.
	@param newName The new display name.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.
 }
function SCBondInterfaceSetLocalizedDisplayName( bond: SCBondInterfaceRef; newName: CFStringRef ): Boolean; external name '_SCBondInterfaceSetLocalizedDisplayName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondInterfaceSetOptions
	@discussion Sets the configuration settings for the specified Ethernet Bond interface.
	@param bond The SCBondInterface interface.
	@param newOptions The new configuration settings.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.
 }
function SCBondInterfaceSetOptions( bond: SCBondInterfaceRef; newOptions: CFDictionaryRef ): Boolean; external name '_SCBondInterfaceSetOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

//#pragma mark -

{!
	@function SCBondInterfaceCopyStatus
	@discussion Returns the status of the specified Ethernet Bond interface.
	@param bond The SCBondInterface interface.
	@result The status associated with the interface.
		You must release the returned value.
 }
function SCBondInterfaceCopyStatus( bond: SCBondInterfaceRef ): SCBondStatusRef; external name '_SCBondInterfaceCopyStatus';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondStatusGetTypeID
	@discussion Returns the type identifier of all SCBondStatus instances.
 }
function SCBondStatusGetTypeID: CFTypeID; external name '_SCBondStatusGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondStatusGetMemberInterfaces
	@discussion Returns the member interfaces that are represented with the
		Ethernet Bond interface.
	@param bondStatus The Ethernet Bond status.
	@result The list of interfaces.
 }
function SCBondStatusGetMemberInterfaces( bondStatus: SCBondStatusRef ): CFArrayRef { of SCNetworkInterfaceRef's }; external name '_SCBondStatusGetMemberInterfaces';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCBondStatusGetInterfaceStatus
	@discussion Returns the status of a specific member interface of an
		Ethernet Bond or the status of the bond as a whole.
	@param bondStatus The Ethernet Bond status.
	@param interface The specific member interface; NULL if you want the
		status of the Ethernet Bond.
	@result The interface status.

	Note: at present, no information about the status of the Ethernet
	      Bond is returned.  As long as one member interface is active
	      then the bond should be operational.
 }
function SCBondStatusGetInterfaceStatus( bondStatus: SCBondStatusRef; interface_: SCNetworkInterfaceRef ): CFDictionaryRef; external name '_SCBondStatusGetInterfaceStatus';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@group Interface configuration (VLAN)
 }

//#pragma mark -
//#pragma mark SCVLANInterface configuration (APIs)

{!
	@function SCVLANInterfaceCopyAll
	@discussion Returns all VLAN interfaces on the system.
	@result The list of VLAN interfaces on the system.
		You must release the returned value.
 }
function SCVLANInterfaceCopyAll( prefs: SCPreferencesRef ): CFArrayRef { of SCVLANInterfaceRef's }; external name '_SCVLANInterfaceCopyAll';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceCopyAvailablePhysicalInterfaces
	@discussion Returns the network capable devices on the system
		that can be associated with a VLAN interface.
	@result The list of interfaces.
		You must release the returned value.
 }
function SCVLANInterfaceCopyAvailablePhysicalInterfaces: CFArrayRef { of SCNetworkInterfaceRef's }; external name '_SCVLANInterfaceCopyAvailablePhysicalInterfaces';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceCreate
	@discussion Create a new SCVLANInterface interface.
	@param prefs The "preferences" session.
	@param physical The physical interface to associate with the VLAN.
	@param tag The tag to associate with the VLAN.
	@result A reference to the new SCVLANInterface.
		You must release the returned value.

	Note: the tag must be in the range (1 <= tag <= 4094)
 }
function SCVLANInterfaceCreate( prefs: SCPreferencesRef; physical: SCNetworkInterfaceRef; tag: CFNumberRef ): SCVLANInterfaceRef; external name '_SCVLANInterfaceCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceRemove
	@discussion Removes the SCVLANInterface from the configuration.
	@param vlan The SCVLANInterface interface.
	@result TRUE if the interface was removed; FALSE if an error was encountered.
 }
function SCVLANInterfaceRemove( vlan: SCVLANInterfaceRef ): Boolean; external name '_SCVLANInterfaceRemove';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceGetPhysicalInterface
	@discussion Returns the physical interface for the specified VLAN interface.
	@param vlan The SCVLANInterface interface.
	@result The list of interfaces.
 }
function SCVLANInterfaceGetPhysicalInterface( vlan: SCVLANInterfaceRef ): SCNetworkInterfaceRef; external name '_SCVLANInterfaceGetPhysicalInterface';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceGetTag
	@discussion Returns the tag for the specified VLAN interface.
	@param vlan The SCVLANInterface interface.
	@result The tag.
 }
function SCVLANInterfaceGetTag( vlan: SCVLANInterfaceRef ): CFNumberRef; external name '_SCVLANInterfaceGetTag';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceGetOptions
	@discussion Returns the configuration settings associated with the VLAN interface.
	@param vlan The SCVLANInterface interface.
	@result The configuration settings associated with the VLAN interface;
		NULL if no changes to the default configuration have been saved.
 }
function SCVLANInterfaceGetOptions( vlan: SCVLANInterfaceRef ): CFDictionaryRef; external name '_SCVLANInterfaceGetOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceSetPhysicalInterfaceAndTag
	@discussion Updates the specified VLAN interface.
	@param vlan The SCVLANInterface interface.
	@param physical The physical interface to associate with the VLAN.
	@param tag The tag to associate with the VLAN.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.

	Note: the tag must be in the range (1 <= tag <= 4094)
 }
function SCVLANInterfaceSetPhysicalInterfaceAndTag( vlan: SCVLANInterfaceRef; physical: SCNetworkInterfaceRef; tag: CFNumberRef ): Boolean; external name '_SCVLANInterfaceSetPhysicalInterfaceAndTag';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceSetLocalizedDisplayName
	@discussion Sets the localized display name for the specified VLAN interface.
	@param vlan The SCVLANInterface interface.
	@param newName The new display name.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.
 }
function SCVLANInterfaceSetLocalizedDisplayName( vlan: SCVLANInterfaceRef; newName: CFStringRef ): Boolean; external name '_SCVLANInterfaceSetLocalizedDisplayName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCVLANInterfaceSetOptions
	@discussion Sets the configuration settings for the specified VLAN interface.
	@param vlan The SCVLANInterface interface.
	@param newOptions The new configuration settings.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.
 }
function SCVLANInterfaceSetOptions( vlan: SCVLANInterfaceRef; newOptions: CFDictionaryRef ): Boolean; external name '_SCVLANInterfaceSetOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)


{ --------------------------------------------------------------------------------
 * PROTOCOLS
 * -------------------------------------------------------------------------------- }

{!
	@group Protocol configuration
 }

//#pragma mark -
//#pragma mark SCNetworkProtocol configuration (APIs)

{!
	@function SCNetworkProtocolGetTypeID
	@discussion Returns the type identifier of all SCNetworkProtocol instances.
 }
function SCNetworkProtocolGetTypeID: CFTypeID; external name '_SCNetworkProtocolGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkProtocolGetConfiguration
	@discussion Returns the configuration settings associated with the protocol.
	@param protocol The network protocol.
	@result The configuration settings associated with the protocol;
		NULL if no configuration settings are associated with the protocol
		or an error was encountered.
 }
function SCNetworkProtocolGetConfiguration( protocol: SCNetworkProtocolRef ): CFDictionaryRef; external name '_SCNetworkProtocolGetConfiguration';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkProtocolGetEnabled
	@discussion Returns whether this protocol has been enabled.
	@param protocol The network protocol.
	@result TRUE if the protocol is enabled.
 }
function SCNetworkProtocolGetEnabled( protocol: SCNetworkProtocolRef ): Boolean; external name '_SCNetworkProtocolGetEnabled';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkProtocolGetProtocolType
	@discussion Returns the associated network protocol type.
	@param protocol The network protocol.
	@result The protocol type.
 }
function SCNetworkProtocolGetProtocolType( protocol: SCNetworkProtocolRef ): CFStringRef; external name '_SCNetworkProtocolGetProtocolType';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkProtocolSetConfiguration
	@discussion Stores the configuration settings for the protocol.
	@param protocol The network protocol.
	@param config The configuration settings to associate with this protocol.
	@result TRUE if the configuration was stored; FALSE if an error was encountered.
 }
function SCNetworkProtocolSetConfiguration( protocol: SCNetworkProtocolRef; config: CFDictionaryRef ): Boolean; external name '_SCNetworkProtocolSetConfiguration';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkProtocolSetEnabled
	@discussion Enables or disables the protocol.
	@param protocol The network protocol.
	@param enabled TRUE if the protocol should be enabled.
	@result TRUE if the enabled status was saved; FALSE if an error was encountered.
 }
function SCNetworkProtocolSetEnabled( protocol: SCNetworkProtocolRef; enabled: Boolean ): Boolean; external name '_SCNetworkProtocolSetEnabled';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{ --------------------------------------------------------------------------------
 * SERVICES
 * -------------------------------------------------------------------------------- }

{!
	@group Service configuration
 }

//#pragma mark -
//#pragma mark SCNetworkService configuration (APIs)

{!
	@function SCNetworkServiceGetTypeID
	@discussion Returns the type identifier of all SCNetworkService instances.
 }
function SCNetworkServiceGetTypeID: CFTypeID; external name '_SCNetworkServiceGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceAddProtocolType
	@discussion Adds a network protocol of the specified type to the
		service.  The protocal configuration is set to default values
		that are appropriate for the interface associated with the
		service.
	@param service The network service.
	@param protocolType The type of SCNetworkProtocol to be added to the service.
	@result TRUE if the protocol was added to the service; FALSE if the
		protocol was already present or an error was encountered.
 }
function SCNetworkServiceAddProtocolType( service: SCNetworkServiceRef; protocolType: CFStringRef ): Boolean; external name '_SCNetworkServiceAddProtocolType';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceCopyAll
	@discussion Returns all available network services for the specified preferences.
	@param prefs The "preferences" session.
	@result The list of SCNetworkService services associated with the preferences.
		You must release the returned value.
 }
function SCNetworkServiceCopyAll( prefs: SCPreferencesRef ): CFArrayRef { of SCNetworkServiceRef's }; external name '_SCNetworkServiceCopyAll';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceCopyProtocols
	@discussion Returns all network protocols associated with the service.
	@param service The network service.
	@result The list of SCNetworkProtocol protocols associated with the service.
		You must release the returned value.
 }
function SCNetworkServiceCopyProtocols( service: SCNetworkServiceRef ): CFArrayRef { of SCNetworkProtocolRef's }; external name '_SCNetworkServiceCopyProtocols';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceCreate
	@discussion Create a new network service for the specified interface in the
		configuration.
	@param prefs The "preferences" session.
	@result A reference to the new SCNetworkService.
		You must release the returned value.
 }
function SCNetworkServiceCreate( prefs: SCPreferencesRef; interface_: SCNetworkInterfaceRef ): SCNetworkServiceRef; external name '_SCNetworkServiceCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceCopy
	@discussion Returns the network service with the specified identifier.
	@param prefs The "preferences" session.
	@param serviceID The unique identifier for the service.
	@result A reference to the SCNetworkService from the associated preferences;
		NULL if the serviceID does not exist in the preferences or if an
		error was encountered.
		You must release the returned value.
 }
function SCNetworkServiceCopy( prefs: SCPreferencesRef; serviceID: CFStringRef ): SCNetworkServiceRef; external name '_SCNetworkServiceCopy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceEstablishDefaultConfiguration
	@discussion Establishes the "default" configuration for a network
		service.  This configuration includes the addition of
		network protocols for the service (with "default"
		configuration options).
	@param service The network service.
	@result TRUE if the configuration was updated; FALSE if an error was encountered.
}
function SCNetworkServiceEstablishDefaultConfiguration( service: SCNetworkServiceRef ): Boolean; external name '_SCNetworkServiceEstablishDefaultConfiguration';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkServiceGetEnabled
	@discussion Returns whether this service has been enabled.
	@param service The network service.
	@result TRUE if the service is enabled.
 }
function SCNetworkServiceGetEnabled( service: SCNetworkServiceRef ): Boolean; external name '_SCNetworkServiceGetEnabled';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceGetInterface
	@discussion Returns the network interface associated with the service.
	@param service The network service.
	@result A reference to the SCNetworkInterface associated with the service;
		NULL if an error was encountered.
 }
function SCNetworkServiceGetInterface( service: SCNetworkServiceRef ): SCNetworkInterfaceRef; external name '_SCNetworkServiceGetInterface';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceGetName
	@discussion Returns the [user specified] name associated with the service.
	@param service The network service.
	@result The [user specified] name.
 }
function SCNetworkServiceGetName( service: SCNetworkServiceRef ): CFStringRef; external name '_SCNetworkServiceGetName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceCopyProtocol
	@discussion Returns the network protocol of the specified type for
		the service.
	@param service The network service.
	@result A reference to the SCNetworkProtocol associated with the service;
		NULL if this protocol has not been added or if an error was encountered.
		You must release the returned value.
 }
function SCNetworkServiceCopyProtocol( service: SCNetworkServiceRef; protocolType: CFStringRef ): SCNetworkProtocolRef; external name '_SCNetworkServiceCopyProtocol';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceGetServiceID
	@discussion Returns the identifier for the service.
	@param service The network service.
	@result The service identifier.
 }
function SCNetworkServiceGetServiceID( service: SCNetworkServiceRef ): CFStringRef; external name '_SCNetworkServiceGetServiceID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceRemove
	@discussion Removes the network service from the configuration.
	@param service The network service.
	@result TRUE if the service was removed; FALSE if an error was encountered.
 }
function SCNetworkServiceRemove( service: SCNetworkServiceRef ): Boolean; external name '_SCNetworkServiceRemove';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceRemoveProtocolType
	@discussion Removes the network protocol of the specified type from the service.
	@param service The network service.
	@param protocolType The type of SCNetworkProtocol to be removed from the service.
	@result TRUE if the protocol was removed to the service; FALSE if the
		protocol was not configured or an error was encountered.
 }
function SCNetworkServiceRemoveProtocolType( service: SCNetworkServiceRef; protocolType: CFStringRef ): Boolean; external name '_SCNetworkServiceRemoveProtocolType';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceSetEnabled
	@discussion Enables or disables the service.
	@param service The network service.
	@param enabled TRUE if the service should be enabled.
	@result TRUE if the enabled status was saved; FALSE if an error was encountered.
 }
function SCNetworkServiceSetEnabled( service: SCNetworkServiceRef; enabled: Boolean ): Boolean; external name '_SCNetworkServiceSetEnabled';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkServiceSetName
	@discussion Stores the [user specified] name for the service.
	@param service The network service.
	@param name The [user defined] name to associate with the service.
	@result TRUE if the name was saved; FALSE if an error was encountered.

	Note: although not technically required, the [user specified] names
	for all services within any given set should be unique.  As such, an
	error will be returned if you attemp to name two services with the
	same string.
 }
function SCNetworkServiceSetName( service: SCNetworkServiceRef; name: CFStringRef ): Boolean; external name '_SCNetworkServiceSetName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)


{ --------------------------------------------------------------------------------
 * SETS
 * -------------------------------------------------------------------------------- }

{!
	@group Set configuration
 }

//#pragma mark -
//#pragma mark SCNetworkSet configuration (APIs)

{!
	@function SCNetworkSetGetTypeID
	@discussion Returns the type identifier of all SCNetworkSet instances.
 }
function SCNetworkSetGetTypeID: CFTypeID; external name '_SCNetworkSetGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetAddService
	@discussion Adds the network service to the set.
	@param set The network set.
	@param service The service to be added.
	@result TRUE if the service was added to the set; FALSE if the
		service was already present or an error was encountered.

	Note: prior to Mac OS X 10.5, the Network Preferences UI
	did not support having a single service being a member of
	more than one set.  An error will be returned if you attempt
	to add a service to more than one set on a pre-10.5 system.
 }
function SCNetworkSetAddService( set_: SCNetworkSetRef; service: SCNetworkServiceRef ): Boolean; external name '_SCNetworkSetAddService';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetContainsInterface
	@discussion Checks if an interface is represented by at least one
		network service in the specified set.
	@param set The network set.
	@param interface The network interface.
	@result TRUE if the interface is represented in the set; FALSE if not.
 }
function SCNetworkSetContainsInterface( set_: SCNetworkSetRef; interface_: SCNetworkInterfaceRef ): Boolean; external name '_SCNetworkSetContainsInterface';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)

{!
	@function SCNetworkSetCopyAll
	@discussion Returns all available sets for the specified preferences.
	@param prefs The "preferences" session.
	@result The list of SCNetworkSet sets associated with the preferences.
		You must release the returned value.
 }
function SCNetworkSetCopyAll( prefs: SCPreferencesRef ): CFArrayRef { of SCNetworkSetRef's }; external name '_SCNetworkSetCopyAll';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetCopyCurrent
	@discussion Returns the "current" set.
	@param prefs The "preferences" session.
	@result The current set; NULL if no current set has been defined.
 }
function SCNetworkSetCopyCurrent( prefs: SCPreferencesRef ): SCNetworkSetRef; external name '_SCNetworkSetCopyCurrent';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetCopyServices
	@discussion Returns all network services associated with the set.
	@param set The network set.
	@result The list of SCNetworkService services associated with the set.
		You must release the returned value.
 }
function SCNetworkSetCopyServices( set_: SCNetworkSetRef ): CFArrayRef { of SCNetworkServiceRef's }; external name '_SCNetworkSetCopyServices';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetCreate
	@discussion Create a new set in the configuration.
	@param prefs The "preferences" session.
	@result A reference to the new SCNetworkSet.
		You must release the returned value.
 }
function SCNetworkSetCreate( prefs: SCPreferencesRef ): SCNetworkSetRef; external name '_SCNetworkSetCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetCopy
	@discussion Returns the set with the specified identifier.
	@param prefs The "preferences" session.
	@param setID The unique identifier for the set.
	@result A reference to the SCNetworkSet from the associated preferences;
		NULL if the setID does not exist in the preferences or if an
		error was encountered.
		You must release the returned value.
 }
function SCNetworkSetCopy( prefs: SCPreferencesRef; setID: CFStringRef ): SCNetworkSetRef; external name '_SCNetworkSetCopy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetGetName
	@discussion Returns the [user specified] name associated with the set.
	@param set The network set.
	@result The [user specified] name.
 }
function SCNetworkSetGetName( set_: SCNetworkSetRef ): CFStringRef; external name '_SCNetworkSetGetName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetGetSetID
	@discussion Returns the identifier for the set.
	@param set The network set.
	@result The set identifier.
 }
function SCNetworkSetGetSetID( set_: SCNetworkSetRef ): CFStringRef; external name '_SCNetworkSetGetSetID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetGetServiceOrder
	@discussion Returns the [user specified] ordering of network services
		within the set.
	@param set The network set.
	@result The ordered list of CFStringRef service identifiers associated
		with the set;
		NULL if no service order has been specified or if an error
		was encountered.
 }
function SCNetworkSetGetServiceOrder( set_: SCNetworkSetRef ): CFArrayRef { of serviceID CFStringRef's }; external name '_SCNetworkSetGetServiceOrder';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetRemove
	@discussion Removes the set from the configuration.
	@param set The network set.
	@result TRUE if the set was removed; FALSE if an error was encountered.
 }
function SCNetworkSetRemove( set_: SCNetworkSetRef ): Boolean; external name '_SCNetworkSetRemove';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetRemoveService
	@discussion Removes the network service from the set.
	@param set The network set.
	@param service The service to be removed.
	@result TRUE if the service was removed from the set; FALSE if the
		service was not already present or an error was encountered.
 }
function SCNetworkSetRemoveService( set_: SCNetworkSetRef; service: SCNetworkServiceRef ): Boolean; external name '_SCNetworkSetRemoveService';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetSetCurrent
	@discussion Specifies the set that should be the "current" set.
	@param set The network set.
	@result TRUE if the current set was updated;
		FALSE if an error was encountered.
 }
function SCNetworkSetSetCurrent( set_: SCNetworkSetRef ): Boolean; external name '_SCNetworkSetSetCurrent';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetSetName
	@discussion Stores the [user specified] name for the set.
	@param set The network set.
	@param name The [user defined] name to associate with the set.
	@result TRUE if the name was saved; FALSE if an error was encountered.

	Note: although not technically required, the [user specified] names
	for all set should be unique.  As such, an error will be returned if
	you attemp to name two sets with the same string.
 }
function SCNetworkSetSetName( set_: SCNetworkSetRef; name: CFStringRef ): Boolean; external name '_SCNetworkSetSetName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCNetworkSetSetServiceOrder
	@discussion Stores the [user specified] ordering of network services for the set.
	@param set The network set.
	@param newOrder The ordered list of CFStringRef service identifiers for the set.
	@result TRUE if the new service order was saved; FALSE if an error was encountered.
 }
function SCNetworkSetSetServiceOrder( set_: SCNetworkSetRef; newOrder: CFArrayRef ): Boolean; external name '_SCNetworkSetSetServiceOrder';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)	{ serviceID CFStringRef's }


{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
