{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000-2004, 2006, 2008-2010 Apple Inc. All rights reserved.
=======
 * Copyright (c) 2000-2004, 2006, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000-2004, 2006, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000-2004, 2006, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000-2004, 2006, 2008, 2009 Apple Inc. All rights reserved.
>>>>>>> origin/cpstrnew
=======
 * Copyright (c) 2000-2003 Apple Computer, Inc. All rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
 * Copyright (c) 2000-2003 Apple Computer, Inc. All rights reserved.
>>>>>>> origin/fixes_2_2
=======
 * Copyright (c) 2000-2004, 2006, 2008, 2009 Apple Inc. All rights reserved.
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

unit SystemConfiguration;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
>>>>>>> origin/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,CFBase,CFError;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

<<<<<<< HEAD

{!
	@header SystemConfiguration
	@discussion The System Configuration framework provides access to the
		data used to configure a running system.  The APIs provided
		by this framework communicate with the configd daemon.

		The configd daemon manages a dynamic store reflecting the
		desired configuration settings as well as the current state
		of the system.  The daemon provides a notification mechanism
		for processes that need to be aware of changes made to the
		data.  Lastly, the daemon loads a number of bundles (or
		plug-ins) that monitor low-level kernel events and, through
		a set of policy modules, keep the state data up to date.
 }

{!
	@enum Error codes
	@discussion Returned error codes.
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,CFBase;
{$ALIGN MAC68K}
=======
>>>>>>> origin/fixes_2.4

{!
	@header SystemConfiguration
	@discussion The System Configuration framework provides access to the
		data used to configure a running system.  The APIs provided
		by this framework communicate with the configd daemon.

		The configd daemon manages a dynamic store reflecting the
		desired configuration settings as well as the current state
		of the system.  The daemon provides a notification mechanism
		for processes that need to be aware of changes made to the
		data.  Lastly, the daemon loads a number of bundles (or
		plug-ins) that monitor low-level kernel events and, through
		a set of policy modules, keep the state data up to date.
 }

{!
<<<<<<< HEAD
	@enum
	@discussion Returned status codes.

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	@enum Error codes
	@discussion Returned error codes.
>>>>>>> origin/fixes_2.4
	@constant kSCStatusOK			Success
	@constant kSCStatusFailed		Non-specific Failure
	@constant kSCStatusInvalidArgument	Invalid argument
	@constant kSCStatusAccessError		Permission denied
	@constant kSCStatusNoKey		No such key
	@constant kSCStatusKeyExists		Data associated with key already defined
	@constant kSCStatusLocked		Lock already held
	@constant kSCStatusNeedLock		Lock required for this operation
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@constant kSCStatusNoStoreSession	Configuration daemon session not active
	@constant kSCStatusNoStoreServer	Configuration daemon not (or no longer) available
	@constant kSCStatusNotifierActive	Notifier is currently active
	@constant kSCStatusNoPrefsSession	Preferences session not active
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
	@constant kSCStatusNoStoreSession	Configuration daemon session not active
	@constant kSCStatusNoStoreServer	Configuration daemon not (or no longer) available
	@constant kSCStatusNotifierActive	Notifier is currently active
<<<<<<< HEAD

	@constant kSCStatusNoPrefsSession	Preference session not active
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	@constant kSCStatusNoPrefsSession	Preferences session not active
>>>>>>> origin/fixes_2.4
	@constant kSCStatusPrefsBusy		Preferences update currently in progress
	@constant kSCStatusNoConfigFile		Configuration file not found
	@constant kSCStatusNoLink		No such link
	@constant kSCStatusStale		Write attempted on stale version of object
	@constant kSCStatusMaxLink		Maximum link count exceeded
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@constant kSCStatusReachabilityUnknown
		A determination could not be made regarding the reachability
		of the specified nodename or address.
	@constant kSCStatusConnectionNoService	Network service for connection not available
}
const
{
	 * Generic error codes
	 }
	kSCStatusOK = 0;	{ Success }
	kSCStatusFailed = 1001;	{ Non-specific failure }
	kSCStatusInvalidArgument = 1002;	{ Invalid argument }
	kSCStatusAccessError = 1003;	{ Permission denied
							   - must be root to obtain lock
							   - could not create access/create preferences
							 }
	kSCStatusNoKey = 1004;	{ No such key }
	kSCStatusKeyExists = 1005;	{ Key already defined }
	kSCStatusLocked = 1006;	{ Lock already held }
	kSCStatusNeedLock = 1007;	{ Lock required for this operation }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
	{
	 * SCDynamicStore error codes
	 }
	kSCStatusNoStoreSession = 2001;	{ Configuration daemon session not active }
	kSCStatusNoStoreServer = 2002;	{ Configuration daemon not (no longer) available }
	kSCStatusNotifierActive = 2003;	{ Notifier is currently active }
	{
	 * SCPreferences error codes
	 }
	kSCStatusNoPrefsSession = 3001;	{ Preference session not active }
	kSCStatusPrefsBusy = 3002;	{ Preferences update currently in progress }
	kSCStatusNoConfigFile = 3003;	{ Configuration file not found }
	kSCStatusNoLink = 3004;	{ No such link }
	kSCStatusStale = 3005;	{ Write attempted on stale version of object }
	kSCStatusMaxLink = 3006;	{ Maximum link count exceeded }
	{
	 * SCNetwork error codes
	 }
	kSCStatusReachabilityUnknown = 4001;	{ Network reachability cannot be determined }
	{
	 * SCNetworkConnection error codes
	 }
//#if	(__MAC_OS_X_VERSION_MIN_REQUIRED >= 1060) || (__IPHONE_OS_VERSION_MIN_REQUIRED >= 20000)
	kSCStatusConnectionNoService = 5001;	{ Network service for connection not available }
//#endif
<<<<<<< HEAD
<<<<<<< HEAD
=======


{ SCDynamicStore APIs }
>>>>>>> graemeg/cpstrnew

{ SCPreferences APIs }

{ Schema Definitions (for SCDynamicStore and SCPreferences) }

{ SCNetworkConfiguration APIs }

{ SCNetworkReachability and SCNetworkConnection APIs }

{!
	@const kCFErrorDomainSystemConfiguration
	@discussion CFError domain associated with errors reported by
		the SystemConfiguration.framework.
 }
var kCFErrorDomainSystemConfiguration: CFStringRef; external name '_kCFErrorDomainSystemConfiguration'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
=======


{ SCDynamicStore APIs }
>>>>>>> graemeg/cpstrnew

{ SCPreferences APIs }

{ Schema Definitions (for SCDynamicStore and SCPreferences) }

{ SCNetworkConfiguration APIs }

{ SCNetworkReachability and SCNetworkConnection APIs }

{!
	@const kCFErrorDomainSystemConfiguration
	@discussion CFError domain associated with errors reported by
		the SystemConfiguration.framework.
 }
var kCFErrorDomainSystemConfiguration: CFStringRef; external name '_kCFErrorDomainSystemConfiguration'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)


{!
	@function SCCopyLastError
	@discussion Returns the most recent status or error code generated
		as the result of calling a System Configuration framework API.
	@result Returns the last error encountered.
 }
function SCCopyLastError: CFErrorRef; external name '_SCCopyLastError';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{!
	@function SCCopyLastError
	@discussion Returns the most recent status or error code generated
		as the result of calling a System Configuration framework API.
	@result Returns the last error encountered.
 }
function SCCopyLastError: CFErrorRef; external name '_SCCopyLastError';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{ SCDynamicStore APIs }

=======
	{
	 * SCDynamicStore error codes
	 }
	kSCStatusNoStoreSession = 2001;	{ Configuration daemon session not active }
	kSCStatusNoStoreServer = 2002;	{ Configuration daemon not (no longer) available }
	kSCStatusNotifierActive = 2003;	{ Notifier is currently active }
	{
	 * SCPreferences error codes
	 }
	kSCStatusNoPrefsSession = 3001;	{ Preference session not active }
	kSCStatusPrefsBusy = 3002;	{ Preferences update currently in progress }
	kSCStatusNoConfigFile = 3003;	{ Configuration file not found }
	kSCStatusNoLink = 3004;	{ No such link }
	kSCStatusStale = 3005;	{ Write attempted on stale version of object }
	kSCStatusMaxLink = 3006;	{ Maximum link count exceeded }
	{
	 * SCNetwork error codes
	 }
	kSCStatusReachabilityUnknown = 4001;	{ Network reachability cannot be determined }
	{
	 * SCNetworkConnection error codes
	 }
//#if	(__MAC_OS_X_VERSION_MIN_REQUIRED >= 1060) || (__IPHONE_OS_VERSION_MIN_REQUIRED >= 20000)
	kSCStatusConnectionNoService = 5001;	{ Network service for connection not available }
//#endif


{ SCDynamicStore APIs }

>>>>>>> graemeg/cpstrnew
=======
	{
	 * SCDynamicStore error codes
	 }
	kSCStatusNoStoreSession = 2001;	{ Configuration daemon session not active }
	kSCStatusNoStoreServer = 2002;	{ Configuration daemon not (no longer) available }
	kSCStatusNotifierActive = 2003;	{ Notifier is currently active }
	{
	 * SCPreferences error codes
	 }
	kSCStatusNoPrefsSession = 3001;	{ Preference session not active }
	kSCStatusPrefsBusy = 3002;	{ Preferences update currently in progress }
	kSCStatusNoConfigFile = 3003;	{ Configuration file not found }
	kSCStatusNoLink = 3004;	{ No such link }
	kSCStatusStale = 3005;	{ Write attempted on stale version of object }
	kSCStatusMaxLink = 3006;	{ Maximum link count exceeded }
	{
	 * SCNetwork error codes
	 }
	kSCStatusReachabilityUnknown = 4001;	{ Network reachability cannot be determined }
	{
	 * SCNetworkConnection error codes
	 }
//#if	(__MAC_OS_X_VERSION_MIN_REQUIRED >= 1060) || (__IPHONE_OS_VERSION_MIN_REQUIRED >= 20000)
	kSCStatusConnectionNoService = 5001;	{ Network service for connection not available }
//#endif


{ SCDynamicStore APIs }

>>>>>>> origin/cpstrnew
{ SCPreferences APIs }

{ Schema Definitions (for SCDynamicStore and SCPreferences) }

{ SCNetworkConfiguration APIs }

{ SCNetworkReachability and SCNetworkConnection APIs }

{!
	@const kCFErrorDomainSystemConfiguration
	@discussion CFError domain associated with errors reported by
		the SystemConfiguration.framework.
 }
var kCFErrorDomainSystemConfiguration: CFStringRef; external name '_kCFErrorDomainSystemConfiguration'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)


{!
	@function SCCopyLastError
	@discussion Returns the most recent status or error code generated
		as the result of calling a System Configuration framework API.
	@result Returns the last error encountered.
 }
function SCCopyLastError: CFErrorRef; external name '_SCCopyLastError';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{!
	@function SCError
	@discussion Returns the most recent status or error code generated
		as the result of calling a System Configuration framework API.
	@result Returns the last error encountered.
 }
function SCError: SInt32; external name '_SCError';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)

{!
	@function SCErrorString
	@discussion Returns a pointer to the message string
		associated with the specified status or error.
	@param status The SCDynamicStoreStatus to be returned.
	@result Returns a pointer to the error message string.
 }
function SCErrorString( status: SInt32 ): ConstCStringPtr; external name '_SCErrorString';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
	@constant kSCStatusReachabilityUnknown
		A determination could not be made regarding the reachability
		of the specified nodename or address.
	@constant kSCStatusConnectionNoService	Network service for connection not available
}
const
{
	 * Generic error codes
	 }
	kSCStatusOK = 0;	{ Success }
	kSCStatusFailed = 1001;	{ Non-specific failure }
	kSCStatusInvalidArgument = 1002;	{ Invalid argument }
	kSCStatusAccessError = 1003;	{ Permission denied
							   - must be root to obtain lock
							   - could not create access/create preferences
							 }
	kSCStatusNoKey = 1004;	{ No such key }
	kSCStatusKeyExists = 1005;	{ Key already defined }
	kSCStatusLocked = 1006;	{ Lock already held }
	kSCStatusNeedLock = 1007;	{ Lock required for this operation }
	{
	 * SCDynamicStore error codes
	 }
	kSCStatusNoStoreSession = 2001;	{ Configuration daemon session not active }
	kSCStatusNoStoreServer = 2002;	{ Configuration daemon not (no longer) available }
	kSCStatusNotifierActive = 2003;	{ Notifier is currently active }
	{
	 * SCPreferences error codes
	 }
	kSCStatusNoPrefsSession = 3001;	{ Preference session not active }
	kSCStatusPrefsBusy = 3002;	{ Preferences update currently in progress }
	kSCStatusNoConfigFile = 3003;	{ Configuration file not found }
	kSCStatusNoLink = 3004;	{ No such link }
	kSCStatusStale = 3005;	{ Write attempted on stale version of object }
	kSCStatusMaxLink = 3006;	{ Maximum link count exceeded }
	{
	 * SCNetwork error codes
	 }
	kSCStatusReachabilityUnknown = 4001;	{ Network reachability cannot be determined }
	{
	 * SCNetworkConnection error codes
	 }
//#if	(__MAC_OS_X_VERSION_MIN_REQUIRED >= 1060) || (__IPHONE_OS_VERSION_MIN_REQUIRED >= 20000)
	kSCStatusConnectionNoService = 5001;	{ Network service for connection not available }
//#endif


{ SCDynamicStore APIs }

{ SCPreferences APIs }

{ Schema Definitions (for SCDynamicStore and SCPreferences) }

{ SCNetworkConfiguration APIs }

{ SCNetworkReachability and SCNetworkConnection APIs }

{!
	@const kCFErrorDomainSystemConfiguration
	@discussion CFError domain associated with errors reported by
		the SystemConfiguration.framework.
 }
var kCFErrorDomainSystemConfiguration: CFStringRef; external name '_kCFErrorDomainSystemConfiguration'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)


{!
	@function SCCopyLastError
	@discussion Returns the most recent status or error code generated
		as the result of calling a System Configuration framework API.
	@result Returns the last error encountered.
 }
function SCCopyLastError: CFErrorRef; external name '_SCCopyLastError';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{!
	@function SCError
	@discussion Returns the most recent status or error code generated
		as the result of calling a System Configuration framework API.
	@result Returns the last error encountered.
 }
function SCError: SInt32; external name '_SCError';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)

{!
	@function SCErrorString
	@discussion Returns a pointer to the message string
		associated with the specified status or error.
	@param status The SCDynamicStoreStatus to be returned.
	@result Returns a pointer to the error message string.
 }
function SCErrorString( status: SInt32 ): ConstCStringPtr; external name '_SCErrorString';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)

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
