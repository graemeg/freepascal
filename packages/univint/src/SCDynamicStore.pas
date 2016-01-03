{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000, 2001, 2003-2005, 2008-2010 Apple Inc. All rights reserved.
=======
 * Copyright (c) 2000, 2001, 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> origin/cpstrnew
=======
 * Copyright (c) 2000 Apple Computer, Inc. All rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
 * Copyright (c) 2000 Apple Computer, Inc. All rights reserved.
>>>>>>> origin/fixes_2_2
=======
 * Copyright (c) 2000, 2001, 2003-2005, 2008, 2009 Apple Inc. All rights reserved.
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

unit SCDynamicStore;
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
<<<<<<< HEAD
{$elsec}
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
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,MacOSXPosix,CFBase,CFArray,CFRunLoop,CFPropertyList,CFDictionary;
{$endc} {not MACOSALLINCLUDE}


{$ALIGN POWER}
<<<<<<< HEAD

{!
	@header SCDynamicStore
	@discussion The SCDynamicStore API provides access to the key-value
		pairs in the dynamic store of a running system.  The dynamic
		store contains, among other items, a copy of the configuration
		settings for the currently active set (which is sometimes
		refered to as the location) and information about the current
		network state.

		The functions in the SCDynamicStore API allow you to find
		key-value pairs, add or remove key-value pairs, add or change
		values, and request notifications.

		To use the functions of the SCDynamicStore API, you must first
		establish a dynamic store session using the SCDynamicStoreCreate
		function.  When you are finished with the session, use CFRelease
		to close it.
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,CFBase,CFArray,CFRunLoop,CFPropertyList,CFDictionary;
{$ALIGN MAC68K}

{!
	@header SCDynamicStore
	The SystemConfiguration framework provides access to the
	data used to configure a running system.  The APIs provided
	by this framework communicate with the "configd" daemon.

	The "configd" daemon manages a "dynamic store" reflecting the
	desired configuration settings as well as the current state
	of the system.  The daemon provides a notification mechanism
	for user-level processes that need to be aware of changes
	made to the data.  Lastly, the daemon loads a number of
	bundles (or plug-ins) that monitor low-level kernel events
	and, via a set of policy modules, keep the state data up
	to date.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{!
	@header SCDynamicStore
	@discussion The SCDynamicStore API provides access to the key-value
		pairs in the dynamic store of a running system.  The dynamic
		store contains, among other items, a copy of the configuration
		settings for the currently active set (which is sometimes
		refered to as the location) and information about the current
		network state.

		The functions in the SCDynamicStore API allow you to find
		key-value pairs, add or remove key-value pairs, add or change
		values, and request notifications.

		To use the functions of the SCDynamicStore API, you must first
		establish a dynamic store session using the SCDynamicStoreCreate
		function.  When you are finished with the session, use CFRelease
		to close it.
>>>>>>> origin/fixes_2.4
 }


{!
	@typedef SCDynamicStoreRef
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion This is the handle to an open a dynamic store session
		with the system configuration daemon.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCDynamicStoreRef = ^__SCDynamicStore; { an opaque type }
	__SCDynamicStore = record end;
=======
	SCDynamicStoreRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCDynamicStoreRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCDynamicStoreRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCDynamicStoreRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew

{!
	@typedef SCDynamicStoreContext
	Structure containing user-specified data and callbacks for an
	SCDynamicStore session.
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
=======
=======
>>>>>>> origin/fixes_2_2
	@discussion This is the handle to an open "dynamic store" session
=======
	@discussion This is the handle to an open a dynamic store session
>>>>>>> origin/fixes_2.4
		with the system configuration daemon.
 }
type
	SCDynamicStoreRef = ^SInt32; { an opaque type }

{!
	@typedef SCDynamicStoreContext
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	Structure containing user-specified data and callbacks for an
	SCDynamicStore session.
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
>>>>>>> origin/fixes_2.4
 }
type
	SCDynamicStoreContext = record
		version: CFIndex;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		info: UnivPtr;
=======
		info: Ptr;
>>>>>>> graemeg/fixes_2_2
=======
		info: Ptr;
>>>>>>> origin/fixes_2_2
=======
		info: UnivPtr;
>>>>>>> origin/fixes_2.4
		retain: function( info: Ptr ): Ptr;
		release: procedure( info: Ptr );
		copyDescription: function( info: Ptr ): CFStringRef;
	end;
	SCDynamicStoreContextPtr = ^SCDynamicStoreContext;

{!
	@typedef SCDynamicStoreCallBack
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion Type of callback function used when notification of
		changes to the dynamic store is delivered.
	@param store The dynamic store session.
	@param changedKeys The list of changed keys.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		The list includes any specific SCDynamicStore keys that
		changed (add, update, remove, notify) since the last call
		to SCDynamicStoreSetNotificationKeys or since the last
		notification callback. The list also includes any specific
		keys matching one of the pattern string(s) that changed.

		An empty list indicates that the SCDynamicStore server
		restarted and that any assumptions based on prior content
		of the SCDynamicStore should be disgarded.

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	@param info A C pointer to a user-specified block of data.
 }
type
	SCDynamicStoreCallBack = procedure( store: SCDynamicStoreRef; changedKeys: CFArrayRef; info: UnivPtr );

{ until the __IPHONE_NA is automatically handled }
{$ifc TARGET_OS_MAC}

{!
	@function SCDynamicStoreGetTypeID
	@discussion Returns the type identifier of all SCDynamicStore instances.
 }
function SCDynamicStoreGetTypeID: CFTypeID; external name '_SCDynamicStoreGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@discussion Type of the callback function used when a
		dynamic store change is delivered.
	@param store The "dynamic store" session.
=======
	@discussion Type of callback function used when notification of
		changes to the dynamic store is delivered.
	@param store The dynamic store session.
>>>>>>> origin/fixes_2.4
	@param changedKeys The list of changed keys.
	@param info A C pointer to a user-specified block of data.
 }
type
	SCDynamicStoreCallBack = procedure( store: SCDynamicStoreRef; changedKeys: CFArrayRef; info: UnivPtr );

{ until the __IPHONE_NA is automatically handled }
{$ifc TARGET_OS_MAC}

{!
	@function SCDynamicStoreGetTypeID
	@discussion Returns the type identifier of all SCDynamicStore instances.
 }
function SCDynamicStoreGetTypeID: CFTypeID; external name '_SCDynamicStoreGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4


{!
	@function SCDynamicStoreCreate
	@discussion Creates a new session used to interact with the dynamic
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		store maintained by the System Configuration server.
	@param allocator The CFAllocator that should be used to allocate
		memory for the local dynamic store object.
=======
=======
>>>>>>> origin/fixes_2_2
		store maintained by the SystemConfiguration server.
	@param allocator The CFAllocator which should be used to allocate
		memory for the local "dynamic store" and its storage for
		values.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param name A string that describes the name of the calling
		process or plug-in of the caller.
	@param callout The function to be called when a watched value
<<<<<<< HEAD
		in the dynamic store is changed.
		A NULL value can be specified if no callouts are
		desired.
	@param context The SCDynamicStoreContext associated with the callout.
	@result Returns a reference to the new SCDynamicStore session.
		You must release the returned value.
 }
function SCDynamicStoreCreate( allocator: CFAllocatorRef; name: CFStringRef; callout: SCDynamicStoreCallBack; var context: SCDynamicStoreContext ): SCDynamicStoreRef; external name '_SCDynamicStoreCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreCreateWithOptions
	@discussion Creates a new session used to interact with the dynamic
		store maintained by the System Configuration server.
	@param allocator The CFAllocator that should be used to allocate
		memory for the local dynamic store object.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2_2
=======
		store maintained by the System Configuration server.
	@param allocator The CFAllocator that should be used to allocate
		memory for the local dynamic store object.
>>>>>>> origin/fixes_2.4
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param name A string that describes the name of the calling
		process or plug-in of the caller.
<<<<<<< HEAD
	@param storeOptions A CFDictionary containing options for the
		dynamic store session (such as whether all keys added or set
		into the dynamic store should be per-session keys).

		Currently available options include:

		<TABLE BORDER>
		<TR>
			<TH>key</TD>
			<TH>value</TD>
		</TR>
		<TR>
			<TD>kSCDynamicStoreUseSessionKeys</TD>
			<TD>CFBooleanRef</TD>
		</TR>
		</TABLE>

		A NULL value can be specified if no options are desired.
	@param callout The function to be called when a watched value
		in the dynamic store is changed.
<<<<<<< HEAD
		A NULL value can be specified if no callouts are
		desired.
	@param context The SCDynamicStoreContext associated with the callout.
	@result Returns a reference to the new SCDynamicStore session.
		You must release the returned value.
 }
function SCDynamicStoreCreateWithOptions( allocator: CFAllocatorRef; name: CFStringRef; storeOptions: CFDictionaryRef; callout: SCDynamicStoreCallBack; var context: SCDynamicStoreContext ): SCDynamicStoreRef; external name '_SCDynamicStoreCreateWithOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

var kSCDynamicStoreUseSessionKeys: CFStringRef; external name '_kSCDynamicStoreUseSessionKeys'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)	{ CFBoolean }

{!
	@function SCDynamicStoreCreateRunLoopSource
	@discussion Creates a CFRunLoopSource object that can be added to the
		application's run loop.  All dynamic store notifications are
		delivered using this run loop source.
	@param allocator The CFAllocator that should be used to allocate
		memory for this run loop source.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
=======
=======
>>>>>>> graemeg/cpstrnew
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
=======
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
>>>>>>> origin/cpstrnew
	@param name A string that describes the name of the calling
		process or plug-in of the caller.
	@param storeOptions A CFDictionary containing options for the
		dynamic store session (such as whether all keys added or set
		into the dynamic store should be per-session keys).

		Currently available options include:

		<TABLE BORDER>
		<TR>
			<TH>key</TD>
			<TH>value</TD>
		</TR>
		<TR>
			<TD>kSCDynamicStoreUseSessionKeys</TD>
			<TD>CFBooleanRef</TD>
		</TR>
		</TABLE>

		A NULL value can be specified if no options are desired.
	@param callout The function to be called when a watched value
		in the dynamic store is changed.
		A NULL value can be specified if no callouts are
		desired.
	@param context The SCDynamicStoreContext associated with the callout.
	@result Returns a reference to the new SCDynamicStore session.
		You must release the returned value.
 }
function SCDynamicStoreCreateWithOptions( allocator: CFAllocatorRef; name: CFStringRef; storeOptions: CFDictionaryRef; callout: SCDynamicStoreCallBack; var context: SCDynamicStoreContext ): SCDynamicStoreRef; external name '_SCDynamicStoreCreateWithOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

var kSCDynamicStoreUseSessionKeys: CFStringRef; external name '_kSCDynamicStoreUseSessionKeys'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)	{ CFBoolean }

{!
	@function SCDynamicStoreCreateRunLoopSource
	@discussion Creates a CFRunLoopSource object that can be added to the
		application's run loop.  All dynamic store notifications are
		delivered using this run loop source.
	@param allocator The CFAllocator that should be used to allocate
		memory for this run loop source.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	@param store A reference to the dynamic store session.
	@param order On platforms which support it, for source versions
		which support it, this parameter determines the order in
		which the sources which are ready to be processed are
		handled. A lower order number causes processing before
		higher order number sources. It is inadvisable to depend
		on the order number for any architectural or design aspect
		of code. In the absence of any reason to do otherwise,
=======
=======
	@param callout The function to be called when a watched value
>>>>>>> origin/fixes_2_2
		in the "dynamic store" is changed.
=======
>>>>>>> origin/fixes_2.4
		A NULL value can be specified if no callouts are
		desired.
	@param context The SCDynamicStoreContext associated with the callout.
	@result Returns a reference to the new SCDynamicStore session.
		You must release the returned value.
 }
function SCDynamicStoreCreate( allocator: CFAllocatorRef; name: CFStringRef; callout: SCDynamicStoreCallBack; var context: SCDynamicStoreContext ): SCDynamicStoreRef; external name '_SCDynamicStoreCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreCreateWithOptions
	@discussion Creates a new session used to interact with the dynamic
		store maintained by the System Configuration server.
	@param allocator The CFAllocator that should be used to allocate
		memory for the local dynamic store object.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
<<<<<<< HEAD
	@param store The "dynamic store" session.
	@param order On platforms which support it, this parameter
		determines the order in which the sources which are
		ready to be processed are handled.  A lower order
		number causes processing before higher order number
		sources. It is inadvisable to depend on the order
		number for any architectural or design aspect of
		code. In the absence of any reason to do otherwise,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	@param name A string that describes the name of the calling
		process or plug-in of the caller.
	@param storeOptions A CFDictionary containing options for the
		dynamic store session (such as whether all keys added or set
		into the dynamic store should be per-session keys).

		Currently available options include:

		<TABLE BORDER>
		<TR>
			<TH>key</TD>
			<TH>value</TD>
		</TR>
		<TR>
			<TD>kSCDynamicStoreUseSessionKeys</TD>
			<TD>CFBooleanRef</TD>
		</TR>
		</TABLE>

		A NULL value can be specified if no options are desired.
	@param callout The function to be called when a watched value
		in the dynamic store is changed.
		A NULL value can be specified if no callouts are
		desired.
	@param context The SCDynamicStoreContext associated with the callout.
	@result Returns a reference to the new SCDynamicStore session.
		You must release the returned value.
 }
function SCDynamicStoreCreateWithOptions( allocator: CFAllocatorRef; name: CFStringRef; storeOptions: CFDictionaryRef; callout: SCDynamicStoreCallBack; var context: SCDynamicStoreContext ): SCDynamicStoreRef; external name '_SCDynamicStoreCreateWithOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

var kSCDynamicStoreUseSessionKeys: CFStringRef; external name '_kSCDynamicStoreUseSessionKeys'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)	{ CFBoolean }

{!
	@function SCDynamicStoreCreateRunLoopSource
	@discussion Creates a CFRunLoopSource object that can be added to the
		application's run loop.  All dynamic store notifications are
		delivered using this run loop source.
	@param allocator The CFAllocator that should be used to allocate
		memory for this run loop source.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param store A reference to the dynamic store session.
	@param order On platforms which support it, for source versions
		which support it, this parameter determines the order in
		which the sources which are ready to be processed are
		handled. A lower order number causes processing before
		higher order number sources. It is inadvisable to depend
		on the order number for any architectural or design aspect
		of code. In the absence of any reason to do otherwise,
>>>>>>> origin/fixes_2.4
		zero should be used.
	@result A reference to the new CFRunLoopSource.
		You must release the returned value.

 }
function SCDynamicStoreCreateRunLoopSource( allocator: CFAllocatorRef; store: SCDynamicStoreRef; order: CFIndex ): CFRunLoopSourceRef; external name '_SCDynamicStoreCreateRunLoopSource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

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
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{$ifc not TARGET_OS_IPHONE}
>>>>>>> origin/fixes_2.4
{!
	@function SCDynamicStoreSetDispatchQueue
	@discussion Initiates notifications for the Notification
		Keys in store to the callback contained in store.
	@param store A reference to the dynamic store session.
	@param queue The dispatch queue to run the callback function on.
		Pass NULL to disable notifications, and release the queue.
	@result Returns TRUE on success, FALSE on failure.

 }
function SCDynamicStoreSetDispatchQueue( store: SCDynamicStoreRef; queue: dispatch_queue_t ): Boolean; external name '_SCDynamicStoreSetDispatchQueue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_OS_IPHONE}
>>>>>>> origin/fixes_2.4

{!
	@function SCDynamicStoreCopyKeyList
	@discussion Returns an array of CFString keys representing the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		current dynamic store entries that match a specified pattern.
	@param store The dynamic store session.
	@param pattern A regex(3) regular expression pattern
		used to match the dynamic store keys.
	@result Returns the list of matching keys; NULL if an error was
		encountered.
<<<<<<< HEAD
		You must release the returned value.
 }
function SCDynamicStoreCopyKeyList( store: SCDynamicStoreRef; pattern: CFStringRef ): CFArrayRef; external name '_SCDynamicStoreCopyKeyList';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreAddValue
	@discussion Adds the key-value pair to the dynamic store if no
		such key already exists.
	@param store The dynamic store session.
	@param key The key of the value to add to the dynamic store.
	@param value The value to add to the dynamic store.
	@result Returns TRUE if the key was added; FALSE if the key was already
		present in the dynamic store or if an error was encountered.
 }
function SCDynamicStoreAddValue( store: SCDynamicStoreRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCDynamicStoreAddValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreAddTemporaryValue
	@discussion Temporarily adds the key-value pair to the dynamic store
		if no such key already exists.  Unless the key is updated by another
		session, the key-value pair will be removed automatically when the
		session is closed.
	@param store The dynamic store session.
	@param key The key of the value to add to the dynamic store.
	@param value The value to add to the dynamic store.
	@result Returns TRUE if the key was added; FALSE if the key was already
		present in the dynamic store or if an error was encountered.
 }
function SCDynamicStoreAddTemporaryValue( store: SCDynamicStoreRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCDynamicStoreAddTemporaryValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreCopyValue
	@discussion Gets the value of the specified key from the dynamic store.
	@param store The dynamic store session.
	@param key The key associated with the value you want to get.
	@result Returns the value from the dynamic store that is associated with the given
		key; NULL if no value was located or an error was encountered.
		You must release the returned value.
 }
function SCDynamicStoreCopyValue( store: SCDynamicStoreRef; key: CFStringRef ): CFPropertyListRef; external name '_SCDynamicStoreCopyValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreCopyMultiple
	@discussion Gets the values of multiple keys in the dynamic store.
	@param store The dynamic store session.
	@param keys The keys associated with the values you want to get; NULL if no specific
		keys are requested.
	@param patterns An array of regex(3) pattern strings used to match the keys; NULL
		if no key patterns are requested.
	@result Returns a dictionary containing the key-value pairs of specific keys and the
		key-value pairs of keys that matched the specified patterns;
		NULL if an error was encountered.
		You must release the returned value.
 }
function SCDynamicStoreCopyMultiple( store: SCDynamicStoreRef; keys: CFArrayRef; patterns: CFArrayRef ): CFDictionaryRef; external name '_SCDynamicStoreCopyMultiple';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreSetValue
	@discussion Adds or replaces a value in the dynamic store for
		the specified key.
	@param store The dynamic store session.
	@param key The key you want to set.
	@param value The value to add to or replace in the dynamic store.
	@result Returns TRUE if the key was updated; FALSE if an error was encountered.
 }
function SCDynamicStoreSetValue( store: SCDynamicStoreRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCDynamicStoreSetValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreSetMultiple
	@discussion Updates multiple values in the dynamic store.
	@param store The dynamic store session.
	@param keysToSet A dictionary of key-value pairs you want to set into the dynamic store.
	@param keysToRemove An array of keys you want to remove from the dynamic store.
	@param keysToNotify An array of keys to flag as changed (without changing their values).
	@result Returns TRUE if the dynamic store updates were successful; FALSE if an error was encountered.
 }
function SCDynamicStoreSetMultiple( store: SCDynamicStoreRef; keysToSet: CFDictionaryRef; keysToRemove: CFArrayRef; keysToNotify: CFArrayRef ): Boolean; external name '_SCDynamicStoreSetMultiple';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
		configuration "dynamic store" entries that match a
		specified pattern.
	@param store The "dynamic store" session.
	@param pattern A regex(3) regular expression pattern that
		will be used to match the "dynamic store" keys.
	@result The list of matching keys.
=======
>>>>>>> origin/fixes_2.4
		You must release the returned value.
 }
function SCDynamicStoreCopyKeyList( store: SCDynamicStoreRef; pattern: CFStringRef ): CFArrayRef; external name '_SCDynamicStoreCopyKeyList';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreAddValue
	@discussion Adds the key-value pair to the dynamic store if no
		such key already exists.
	@param store The dynamic store session.
	@param key The key of the value to add to the dynamic store.
	@param value The value to add to the dynamic store.
	@result Returns TRUE if the key was added; FALSE if the key was already
		present in the dynamic store or if an error was encountered.
 }
function SCDynamicStoreAddValue( store: SCDynamicStoreRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCDynamicStoreAddValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreAddTemporaryValue
	@discussion Temporarily adds the key-value pair to the dynamic store
		if no such key already exists.  Unless the key is updated by another
		session, the key-value pair will be removed automatically when the
		session is closed.
	@param store The dynamic store session.
	@param key The key of the value to add to the dynamic store.
	@param value The value to add to the dynamic store.
	@result Returns TRUE if the key was added; FALSE if the key was already
		present in the dynamic store or if an error was encountered.
 }
function SCDynamicStoreAddTemporaryValue( store: SCDynamicStoreRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCDynamicStoreAddTemporaryValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreCopyValue
	@discussion Gets the value of the specified key from the dynamic store.
	@param store The dynamic store session.
	@param key The key associated with the value you want to get.
	@result Returns the value from the dynamic store that is associated with the given
		key; NULL if no value was located or an error was encountered.
		You must release the returned value.
 }
function SCDynamicStoreCopyValue( store: SCDynamicStoreRef; key: CFStringRef ): CFPropertyListRef; external name '_SCDynamicStoreCopyValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreCopyMultiple
	@discussion Gets the values of multiple keys in the dynamic store.
	@param store The dynamic store session.
	@param keys The keys associated with the values you want to get; NULL if no specific
		keys are requested.
	@param patterns An array of regex(3) pattern strings used to match the keys; NULL
		if no key patterns are requested.
	@result Returns a dictionary containing the key-value pairs of specific keys and the
		key-value pairs of keys that matched the specified patterns;
		NULL if an error was encountered.
		You must release the returned value.
 }
function SCDynamicStoreCopyMultiple( store: SCDynamicStoreRef; keys: CFArrayRef; patterns: CFArrayRef ): CFDictionaryRef; external name '_SCDynamicStoreCopyMultiple';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreSetValue
	@discussion Adds or replaces a value in the dynamic store for
		the specified key.
	@param store The dynamic store session.
	@param key The key you want to set.
	@param value The value to add to or replace in the dynamic store.
	@result Returns TRUE if the key was updated; FALSE if an error was encountered.
 }
function SCDynamicStoreSetValue( store: SCDynamicStoreRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCDynamicStoreSetValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreSetMultiple
	@discussion Updates multiple values in the dynamic store.
	@param store The dynamic store session.
	@param keysToSet A dictionary of key-value pairs you want to set into the dynamic store.
	@param keysToRemove An array of keys you want to remove from the dynamic store.
	@param keysToNotify An array of keys to flag as changed (without changing their values).
	@result Returns TRUE if the dynamic store updates were successful; FALSE if an error was encountered.
 }
function SCDynamicStoreSetMultiple( store: SCDynamicStoreRef; keysToSet: CFDictionaryRef; keysToRemove: CFArrayRef; keysToNotify: CFArrayRef ): Boolean; external name '_SCDynamicStoreSetMultiple';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCDynamicStoreRemoveValue
	@discussion Removes the value of the specified key from the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		dynamic store.
	@param store The dynamic store session.
	@param key The key of the value you want to remove.
	@result Returns TRUE if the key was removed; FALSE if no value was
<<<<<<< HEAD
		located or an error was encountered.
 }
function SCDynamicStoreRemoveValue( store: SCDynamicStoreRef; key: CFStringRef ): Boolean; external name '_SCDynamicStoreRemoveValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
		"dynamic store".
	@param store The "dynamic store" session.
	@param key The key of the value you wish to remove.
	@result TRUE if the key was removed; FALSE if no value was
		located or an error was encountered.
 }
function SCDynamicStoreRemoveValue( store: SCDynamicStoreRef; key: CFStringRef ): Boolean; external name '_SCDynamicStoreRemoveValue';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		located or an error was encountered.
 }
function SCDynamicStoreRemoveValue( store: SCDynamicStoreRef; key: CFStringRef ): Boolean; external name '_SCDynamicStoreRemoveValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCDynamicStoreNotifyValue
	@discussion Triggers a notification to be delivered for the
		specified key in the dynamic store.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	@param store The dynamic store session.
	@param key The key that should be flagged as changed.  Any dynamic store sessions
		that are monitoring this key will received a notification.  Note that the
		key's value is not updated.
	@result Returns TRUE if the notification was processed; FALSE if an error was encountered.
<<<<<<< HEAD
 }
function SCDynamicStoreNotifyValue( store: SCDynamicStoreRef; key: CFStringRef ): Boolean; external name '_SCDynamicStoreNotifyValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@param store The "dynamic store" session.
	@param key The key which should be flagged as changed (without actually changing the data).
	@result TRUE if the value was updated; FALSE if an error was encountered.
 }
function SCDynamicStoreNotifyValue( store: SCDynamicStoreRef; key: CFStringRef ): Boolean; external name '_SCDynamicStoreNotifyValue';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 }
function SCDynamicStoreNotifyValue( store: SCDynamicStoreRef; key: CFStringRef ): Boolean; external name '_SCDynamicStoreNotifyValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCDynamicStoreSetNotificationKeys
	@discussion Specifies a set of specific keys and key patterns
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		that should be monitored for changes.
	@param store The dynamic store session being watched.
	@param keys An array of keys to be monitored; NULL if no specific keys
		are to be monitored.
	@param patterns An array of regex(3) pattern strings used to match keys to be monitored;
		NULL if no key patterns are to be monitored.
	@result Returns TRUE if the set of notification keys and patterns was successfully
		updated; FALSE if an error was encountered.
 }
function SCDynamicStoreSetNotificationKeys( store: SCDynamicStoreRef; keys: CFArrayRef; patterns: CFArrayRef ): Boolean; external name '_SCDynamicStoreSetNotificationKeys';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
		which should be monitored for changes.
	@param store The "dynamic store" session being watched.
	@param keys The keys to be monitored; NULL if no specific keys
=======
		that should be monitored for changes.
	@param store The dynamic store session being watched.
	@param keys An array of keys to be monitored; NULL if no specific keys
>>>>>>> origin/fixes_2.4
		are to be monitored.
	@param patterns An array of regex(3) pattern strings used to match keys to be monitored;
		NULL if no key patterns are to be monitored.
	@result Returns TRUE if the set of notification keys and patterns was successfully
		updated; FALSE if an error was encountered.
 }
function SCDynamicStoreSetNotificationKeys( store: SCDynamicStoreRef; keys: CFArrayRef; patterns: CFArrayRef ): Boolean; external name '_SCDynamicStoreSetNotificationKeys';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCDynamicStoreCopyNotifiedKeys
	@discussion Returns an array of CFString keys representing the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		dynamic store entries that have changed since this
		function was last called.  If possible, your application should
		use the notification functions instead of polling for the list
		of changed keys returned by this function.
	@param store The dynamic store session.
	@result Returns the list of changed keys;
		NULL if an error was encountered.
<<<<<<< HEAD
		You must release the returned value.
 }
function SCDynamicStoreCopyNotifiedKeys( store: SCDynamicStoreRef ): CFArrayRef; external name '_SCDynamicStoreCopyNotifiedKeys';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
		"dynamic store" entries that have changed since this
		function was last called.
	@param store The "dynamic store" session.
	@result The list of changed keys.
=======
>>>>>>> origin/fixes_2.4
		You must release the returned value.
 }
function SCDynamicStoreCopyNotifiedKeys( store: SCDynamicStoreRef ): CFArrayRef; external name '_SCDynamicStoreCopyNotifiedKeys';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}
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
