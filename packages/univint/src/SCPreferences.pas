{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000, 2001, 2004, 2005, 2007-2010 Apple Inc. All rights reserved.
=======
 * Copyright (c) 2000, 2001, 2004, 2005, 2007-2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2004, 2005, 2007-2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2004, 2005, 2007-2009 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000, 2001, 2004, 2005, 2007-2009 Apple Inc. All rights reserved.
>>>>>>> origin/cpstrnew
=======
 * Copyright (c) 2000-2003 Apple Computer, Inc. All rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
 * Copyright (c) 2000-2003 Apple Computer, Inc. All rights reserved.
>>>>>>> origin/fixes_2_2
=======
 * Copyright (c) 2000, 2001, 2004, 2005, 2007-2009 Apple Inc. All rights reserved.
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

unit SCPreferences;
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
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
>>>>>>> origin/cpstrnew
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,MacOSXPosix,CFBase,SCDynamicStore,CFDate,CFPropertyList,CFArray,CFData,CFRunLoop,Authorization;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{$ifc not TARGET_OS_IPHONE}
{$elsec} {not TARGET_OS_IPHONE}
type
	AuthorizationOpaqueRef = record end;
	AuthorizationRef = ^AuthorizationOpaqueRef;
{$endc} {not TARGET_OS_IPHONE}
<<<<<<< HEAD

{!
	@header SCPreferences
	@discussion The SCPreferences API allows an application to load and
		store XML configuration data in a controlled manner and provide
		the necessary notifications to other applications that need to
		be aware of configuration changes.

		To access configuration preferences, you must first establish a
		preferences session using the SCPreferencesCreate function.
		To identify a specific set of preferences to access, you pass a
		value in the prefsID parameter.
		A NULL value indicates that the default system preferences are
		to be accessed.
		A string that starts with a leading "/" character specifies
		the absolute path to the file containing the preferences to
		be accessed.
		A string that does not start with a leading "/" character
		specifies a file relative to the default system preferences
		directory.

		When you are finished with the preferences session, use
		CFRelease to close it.
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,CFBase,SCDynamicStore,CFDate,CFPropertyList,CFArray,CFData;
{$ALIGN MAC68K}

{!
	@header SCPreferences
	The SCPreferencesXXX() APIs allow an application to load and
	store XML configuration data in a controlled manner and provide
	the necessary notifications to other applications that need to
	be aware of configuration changes.

	The stored XML configuration data is accessed using a prefsID. A
	NULL value indicates that the default system preferences are to
	be accessed.
	A string which starts with a leading "/" character specifies the
	path to the file containing te preferences to be accessed.
	A string which does not start with a leading "/" character
	specifies a file relative to the default system preferences
	directory.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{!
	@header SCPreferences
	@discussion The SCPreferences API allows an application to load and
		store XML configuration data in a controlled manner and provide
		the necessary notifications to other applications that need to
		be aware of configuration changes.

		To access configuration preferences, you must first establish a
		preferences session using the SCPreferencesCreate function.
		To identify a specific set of preferences to access, you pass a
		value in the prefsID parameter.
		A NULL value indicates that the default system preferences are
		to be accessed.
		A string that starts with a leading "/" character specifies
		the absolute path to the file containing the preferences to
		be accessed.
		A string that does not start with a leading "/" character
		specifies a file relative to the default system preferences
		directory.

		When you are finished with the preferences session, use
		CFRelease to close it.
>>>>>>> origin/fixes_2.4
 }


{!
	@typedef SCPreferencesRef
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@discussion This is the handle to an open preferences session for
		accessing system configuration preferences.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	SCPreferencesRef = ^__SCPreferences; { an opaque type }
	__SCPreferences = record end;
=======
	SCPreferencesRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCPreferencesRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCPreferencesRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	SCPreferencesRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew

{!
	@enum SCPreferencesNotification
	@discussion Used with the SCPreferencesCallBack callback
		to describe the type of notification.
	@constant kSCPreferencesNotificationCommit Indicates when new
		preferences have been saved.
	@constant kSCPreferencesNotificationApply Key Indicates when a
		request has been made to apply the currently saved
		preferences to the active system configuration.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kSCPreferencesNotificationCommit = 1 shl 0;	// __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA)
	kSCPreferencesNotificationApply = 1 shl 1;	// __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
//AVAILABLE_MAC_OS_X_10_4_OR_LATER
const
	kSCPreferencesNotificationCommit = 1 shl 0;
	kSCPreferencesNotificationApply = 1 shl 1;
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

type
	SCPreferencesNotification = UInt32;

{!
	@typedef SCPreferencesContext
	Structure containing user-specified data and callbacks for SCPreferences.
	@field version The version number of the structure type being passed
		in as a parameter to the SCPreferencesSetCallback function.
		This structure is version 0.
	@field info A C pointer to a user-specified block of data.
	@field retain The callback used to add a retain for the info field.
		If this parameter is not a pointer to a function of the correct
		prototype, the behavior is undefined.
		The value may be NULL.
	@field release The calllback used to remove a retain previously added
		for the info field.
		If this parameter is not a pointer to a function of the
		correct prototype, the behavior is undefined.
		The value may be NULL.
	@field copyDescription The callback used to provide a description of
		the info field.
 }
type
	SCPreferencesContext = record
		version: CFIndex;
		info: UnivPtr;
		retain: function( info: UnivPtr ): UnivPtr;
		release: procedure( info: UnivPtr );
		copyDescription: function( info: UnivPtr ): CFStringRef;
	end;

{!
	@typedef SCPreferencesCallBack
	@discussion Type of the callback function used when the
		preferences have been updated and/or applied.
	@param prefs The preferences session.
	@param notificationType The type of notification, such as changes
		committed, changes applied, etc.
	@param info A C pointer to a user-specified block of data.
 }
type
	SCPreferencesCallBack = procedure( prefs: SCPreferencesRef; notificationType: SCPreferencesNotification; info: UnivPtr );

{ until __IPHONE_NA is automatically translated }
{$ifc TARGET_OS_MAC}

{!
	@function SCPreferencesGetTypeID
	@discussion Returns the type identifier of all SCPreferences instances.
 }
function SCPreferencesGetTypeID: CFTypeID; external name '_SCPreferencesGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@discussion This is the handle to an open "session" for
=======
	@discussion This is the handle to an open preferences session for
>>>>>>> origin/fixes_2.4
		accessing system configuration preferences.
 }
type
	SCPreferencesRef = ^SInt32; { an opaque type }

{!
	@enum SCPreferencesNotification
	@discussion Used with the SCPreferencesCallBack callback
		to describe the type of notification.
	@constant kSCPreferencesNotificationCommit Indicates when new
		preferences have been saved.
	@constant kSCPreferencesNotificationApply Key Indicates when a
		request has been made to apply the currently saved
		preferences to the active system configuration.
 }
//AVAILABLE_MAC_OS_X_10_4_OR_LATER
const
	kSCPreferencesNotificationCommit = 1 shl 0;
	kSCPreferencesNotificationApply = 1 shl 1;

type
	SCPreferencesNotification = UInt32;

{!
	@typedef SCPreferencesContext
	Structure containing user-specified data and callbacks for SCPreferences.
	@field version The version number of the structure type being passed
		in as a parameter to the SCPreferencesSetCallback function.
		This structure is version 0.
	@field info A C pointer to a user-specified block of data.
	@field retain The callback used to add a retain for the info field.
		If this parameter is not a pointer to a function of the correct
		prototype, the behavior is undefined.
		The value may be NULL.
	@field release The calllback used to remove a retain previously added
		for the info field.
		If this parameter is not a pointer to a function of the
		correct prototype, the behavior is undefined.
		The value may be NULL.
	@field copyDescription The callback used to provide a description of
		the info field.
 }
type
	SCPreferencesContext = record
		version: CFIndex;
		info: UnivPtr;
		retain: function( info: UnivPtr ): UnivPtr;
		release: procedure( info: UnivPtr );
		copyDescription: function( info: UnivPtr ): CFStringRef;
	end;

{!
	@typedef SCPreferencesCallBack
	@discussion Type of the callback function used when the
		preferences have been updated and/or applied.
	@param prefs The preferences session.
	@param notificationType The type of notification, such as changes
		committed, changes applied, etc.
	@param info A C pointer to a user-specified block of data.
 }
type
	SCPreferencesCallBack = procedure( prefs: SCPreferencesRef; notificationType: SCPreferencesNotification; info: UnivPtr );

{ until __IPHONE_NA is automatically translated }
{$ifc TARGET_OS_MAC}

{!
	@function SCPreferencesGetTypeID
	@discussion Returns the type identifier of all SCPreferences instances.
 }
function SCPreferencesGetTypeID: CFTypeID; external name '_SCPreferencesGetTypeID';
<<<<<<< HEAD

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesCreate
	@discussion Initiates access to the per-system set of configuration
		preferences.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	@param allocator The CFAllocator that should be used to allocate
		memory for this preferences session.
		This parameter may be NULL in which case the current
		default CFAllocator is used.
		If this reference is not a valid CFAllocator, the behavior
		is undefined.
<<<<<<< HEAD
	@param name A string that describes the name of the calling
		process.
	@param prefsID A string that identifies the name of the
		group of preferences to be accessed or updated.
	@result Returns a reference to the new SCPreferences.
		You must release the returned value.
 }
function SCPreferencesCreate( allocator: CFAllocatorRef; name: CFStringRef; prefsID: CFStringRef ): SCPreferencesRef; external name '_SCPreferencesCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)


{!
	@function SCPreferencesCreateWithAuthorization
	@discussion Initiates access to the per-system set of configuration
		preferences.
	@param allocator The CFAllocator that should be used to allocate
		memory for this preferences session.
		This parameter may be NULL in which case the current
		default CFAllocator is used.
		If this reference is not a valid CFAllocator, the behavior
		is undefined.
	@param name A string that describes the name of the calling
		process.
	@param prefsID A string that identifies the name of the
		group of preferences to be accessed or updated.
	@param authorization An authorization reference that is used to
		authorize any access to the enhanced privileges needed
		to manage the preferences session.
	@result Returns a reference to the new SCPreferences.
		You must release the returned value.
 }
function SCPreferencesCreateWithAuthorization( allocator: CFAllocatorRef; name: CFStringRef; prefsID: CFStringRef; authorization: AuthorizationRef ): SCPreferencesRef; external name '_SCPreferencesCreateWithAuthorization';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@param allocator ...
=======
>>>>>>> origin/fixes_2.4
	@param name A string that describes the name of the calling
		process.
	@param prefsID A string that identifies the name of the
		group of preferences to be accessed or updated.
	@result Returns a reference to the new SCPreferences.
		You must release the returned value.
 }
function SCPreferencesCreate( allocator: CFAllocatorRef; name: CFStringRef; prefsID: CFStringRef ): SCPreferencesRef; external name '_SCPreferencesCreate';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)


{!
	@function SCPreferencesCreateWithAuthorization
	@discussion Initiates access to the per-system set of configuration
		preferences.
	@param allocator The CFAllocator that should be used to allocate
		memory for this preferences session.
		This parameter may be NULL in which case the current
		default CFAllocator is used.
		If this reference is not a valid CFAllocator, the behavior
		is undefined.
	@param name A string that describes the name of the calling
		process.
	@param prefsID A string that identifies the name of the
		group of preferences to be accessed or updated.
	@param authorization An authorization reference that is used to
		authorize any access to the enhanced privileges needed
		to manage the preferences session.
	@result Returns a reference to the new SCPreferences.
		You must release the returned value.
 }
function SCPreferencesCreateWithAuthorization( allocator: CFAllocatorRef; name: CFStringRef; prefsID: CFStringRef; authorization: AuthorizationRef ): SCPreferencesRef; external name '_SCPreferencesCreateWithAuthorization';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesLock
	@discussion Locks access to the configuration preferences.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		This function obtains exclusive access to the configuration
		preferences.  Clients attempting to obtain exclusive access
		to the preferences will either receive a kSCStatusPrefsBusy
		error or block waiting for the lock to be released.
	@param prefs The preferences session.
<<<<<<< HEAD
	@param wait A boolean flag indicating whether the calling process
		should block waiting for another process to complete its update
		operation and release its lock.
	@result Returns TRUE if the lock was obtained;
		FALSE if an error occurred.
 }
function SCPreferencesLock( prefs: SCPreferencesRef; wait: Boolean ): Boolean; external name '_SCPreferencesLock';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	This function obtains exclusive access to the configuration
	preferences associated with this prefsID. Clients attempting
	to obtain exclusive access to the preferences will either receive
	an kSCStatusPrefsBusy error or block waiting for the lock to be
	released.
	@param session An SCPreferencesRef handle that should be used for
		all API calls.
=======
>>>>>>> origin/fixes_2.4
	@param wait A boolean flag indicating whether the calling process
		should block waiting for another process to complete its update
		operation and release its lock.
	@result Returns TRUE if the lock was obtained;
		FALSE if an error occurred.
 }
<<<<<<< HEAD
function SCPreferencesLock( session: SCPreferencesRef; wait: Boolean ): Boolean; external name '_SCPreferencesLock';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SCPreferencesLock( prefs: SCPreferencesRef; wait: Boolean ): Boolean; external name '_SCPreferencesLock';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesCommitChanges
	@discussion Commits changes made to the configuration preferences to
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		persistent storage.

		This function commits any changes to permanent storage.
		Implicit calls to the SCPreferencesLock and SCPreferencesUnlock
		functions will be made if exclusive access has not already been
		established.

		Note: This routine commits changes to persistent storage.
		Call the SCPreferencesApplyChanges function to apply the
		changes to the running system.
	@param prefs The preferences session.
	@result Returns TRUE if the lock was obtained;
		FALSE if an error occurred.
<<<<<<< HEAD
 }
function SCPreferencesCommitChanges( prefs: SCPreferencesRef ): Boolean; external name '_SCPreferencesCommitChanges';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
		persitent storage.

		This function commits any changes to permanent storage. An
		implicit call to SCPreferencesLock/SCPreferencesUnlock will
		be made if exclusive access has not already been established.

		Note:  This routine commits changes to persistent storage.
		Call SCPreferencesApplyChanges() to apply the changes
		to the running system.
	@param session An SCPreferencesRef handle that should be used for
		all API calls.
	@result TRUE if the lock was obtained; FALSE if an error occurred.
 }
function SCPreferencesCommitChanges( session: SCPreferencesRef ): Boolean; external name '_SCPreferencesCommitChanges';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 }
function SCPreferencesCommitChanges( prefs: SCPreferencesRef ): Boolean; external name '_SCPreferencesCommitChanges';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesApplyChanges
	@discussion Requests that the currently stored configuration
		preferences be applied to the active configuration.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param prefs The preferences session.
	@result Returns TRUE if the lock was obtained;
		FALSE if an error occurred.
 }
function SCPreferencesApplyChanges( prefs: SCPreferencesRef ): Boolean; external name '_SCPreferencesApplyChanges';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@param session An SCPreferencesRef handle that should be used for
		all API calls.
	@result TRUE if the lock was obtained; FALSE if an error occurred.
 }
function SCPreferencesApplyChanges( session: SCPreferencesRef ): Boolean; external name '_SCPreferencesApplyChanges';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	@param prefs The preferences session.
	@result Returns TRUE if the lock was obtained;
		FALSE if an error occurred.
 }
function SCPreferencesApplyChanges( prefs: SCPreferencesRef ): Boolean; external name '_SCPreferencesApplyChanges';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesUnlock
	@discussion Releases exclusive access to the configuration preferences.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		This function releases the exclusive access lock to the
		preferences.  Other clients will be now be able to establish
		exclusive access to the preferences.
	@param prefs The preferences session.
	@result Returns TRUE if the lock was obtained;
		FALSE if an error occurred.
<<<<<<< HEAD
 }
function SCPreferencesUnlock( prefs: SCPreferencesRef ): Boolean; external name '_SCPreferencesUnlock';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
		This function releases the exclusive access "lock" for this prefsID.
		Other clients will be now be able to establish exclusive access to
		the preferences.
	@param session An SCPreferencesRef handle that should be used for
		all API calls.
	@result TRUE if the lock was obtained; FALSE if an error occurred.
 }
function SCPreferencesUnlock( session: SCPreferencesRef ): Boolean; external name '_SCPreferencesUnlock';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 }
function SCPreferencesUnlock( prefs: SCPreferencesRef ): Boolean; external name '_SCPreferencesUnlock';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesGetSignature
	@discussion Returns a sequence of bytes that can be used to determine
		if the saved configuration preferences have changed.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param prefs The preferences session.
	@result Returns a CFDataRef that reflects the signature of the configuration
		preferences at the time of the call to the SCPreferencesCreate function.
 }
function SCPreferencesGetSignature( prefs: SCPreferencesRef ): CFDataRef; external name '_SCPreferencesGetSignature';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@param session An SCPreferencesRef handle that should be used for
		all API calls.
	@param signature A pointer to a CFDataRef that will reflect
		the signature of the configuration preferences at the time
		of the call to SCPreferencesCreate().
	@result A CFDataRef that reflects the signature of the configuration
		preferences at the time of the call to SCPreferencesCreate().
 }
function SCPreferencesGetSignature( session: SCPreferencesRef ): CFDataRef; external name '_SCPreferencesGetSignature';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	@param prefs The preferences session.
	@result Returns a CFDataRef that reflects the signature of the configuration
		preferences at the time of the call to the SCPreferencesCreate function.
 }
function SCPreferencesGetSignature( prefs: SCPreferencesRef ): CFDataRef; external name '_SCPreferencesGetSignature';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesCopyKeyList
	@discussion Returns an array of currently defined preference keys.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param prefs The preferences session.
	@result Returns the list of keys.
		You must release the returned value.
 }
function SCPreferencesCopyKeyList( prefs: SCPreferencesRef ): CFArrayRef; external name '_SCPreferencesCopyKeyList';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	@param session An SCPreferencesRef handle that should be used for
		all API calls.
	@result The list of keys.  You must release the returned value.
 }
function SCPreferencesCopyKeyList( session: SCPreferencesRef ): CFArrayRef; external name '_SCPreferencesCopyKeyList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	@param prefs The preferences session.
	@result Returns the list of keys.
		You must release the returned value.
 }
function SCPreferencesCopyKeyList( prefs: SCPreferencesRef ): CFArrayRef; external name '_SCPreferencesCopyKeyList';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesGetValue
	@discussion Returns the data associated with a preference key.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		This function retrieves data associated with the specified
		key.

		Note: To avoid inadvertantly reading stale data, first call
		the SCPreferencesLock function.
	@param prefs The preferences session.
	@param key The preference key to be returned.
	@result Returns the value associated with the specified preference key;
		NULL if no value was located.
 }
function SCPreferencesGetValue( prefs: SCPreferencesRef; key: CFStringRef ): CFPropertyListRef; external name '_SCPreferencesGetValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
		This function retrieves data associated with a key for the prefsID.
=======
		This function retrieves data associated with the specified
		key.
>>>>>>> origin/fixes_2.4

		Note: To avoid inadvertantly reading stale data, first call
		the SCPreferencesLock function.
	@param prefs The preferences session.
	@param key The preference key to be returned.
	@result Returns the value associated with the specified preference key;
		NULL if no value was located.
 }
<<<<<<< HEAD
function SCPreferencesGetValue( session: SCPreferencesRef; key: CFStringRef ): CFPropertyListRef; external name '_SCPreferencesGetValue';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SCPreferencesGetValue( prefs: SCPreferencesRef; key: CFStringRef ): CFPropertyListRef; external name '_SCPreferencesGetValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesAddValue
	@discussion Adds data for a preference key.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		This function associates new data with the specified key.
		To commit these changes to permanent storage, a call must
		be made to the SCPreferencesCommitChanges function.
	@param prefs The preferences session.
<<<<<<< HEAD
	@param key The preference key to be updated.
	@param value The CFPropertyListRef object containing the
		value to be associated with the specified preference key.
	@result Returns TRUE if the value was added;
		FALSE if the key already exists or
		if an error occurred.
 }
function SCPreferencesAddValue( prefs: SCPreferencesRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCPreferencesAddValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	This function associates new data with the specified key. In order
	to commit these changes to permanent storage a call must be made to
	SCPreferencesCommitChanges().
	@param session The SCPreferencesRef handle that should be used to
		communicate with the APIs.
=======
>>>>>>> origin/fixes_2.4
	@param key The preference key to be updated.
	@param value The CFPropertyListRef object containing the
		value to be associated with the specified preference key.
	@result Returns TRUE if the value was added;
		FALSE if the key already exists or
		if an error occurred.
 }
<<<<<<< HEAD
function SCPreferencesAddValue( session: SCPreferencesRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCPreferencesAddValue';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SCPreferencesAddValue( prefs: SCPreferencesRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCPreferencesAddValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesSetValue
	@discussion Updates the data associated with a preference key.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		This function adds or replaces the value associated with the
		specified key.  To commit these changes to permanent storage
		a call must be made to the SCPreferencesCommitChanges function.
	@param prefs The preferences session.
<<<<<<< HEAD
	@param key The preference key to be updated.
	@param value The CFPropertyListRef object containing the
		data to be associated with the specified preference key.
	@result Returns TRUE if the value was set;
		FALSE if an error occurred.
 }
function SCPreferencesSetValue( prefs: SCPreferencesRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCPreferencesSetValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
	This function adds or replaces the value associated with the
	specified key. In order to commit these changes to permanent
	storage a call must be made to SCPreferencesCommitChanges().
	@param session The SCPreferencesRef handle that should be used to
		communicate with the APIs.
=======
>>>>>>> origin/fixes_2.4
	@param key The preference key to be updated.
	@param value The CFPropertyListRef object containing the
		data to be associated with the specified preference key.
	@result Returns TRUE if the value was set;
		FALSE if an error occurred.
 }
<<<<<<< HEAD
function SCPreferencesSetValue( session: SCPreferencesRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCPreferencesSetValue';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SCPreferencesSetValue( prefs: SCPreferencesRef; key: CFStringRef; value: CFPropertyListRef ): Boolean; external name '_SCPreferencesSetValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{!
	@function SCPreferencesRemoveValue
	@discussion Removes the data associated with a preference key.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		This function removes the data associated with the specified
		key.  To commit these changes to permanent storage a call must
		be made to the SCPreferencesCommitChanges function.
	@param prefs The preferences session.
<<<<<<< HEAD
	@param key The preference key to be removed.
	@result Returns TRUE if the value was removed;
		FALSE if the key did not exist or if an error occurred.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
function SCPreferencesRemoveValue( prefs: SCPreferencesRef; key: CFStringRef ): Boolean; external name '_SCPreferencesRemoveValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCPreferencesSetCallback
	@discussion Assigns a callback to a preferences session.  The function
		is called when the changes to the preferences have been
		committed or applied.
	@param prefs The preferences session.
	@param callout The function to be called when the preferences have
		been changed or applied.
		If NULL, the current callback is removed.
	@param context The SCPreferencesContext associated with
		the callout.
	@result Returns TRUE if the notification client was successfully set.
 }
function SCPreferencesSetCallback( prefs: SCPreferencesRef; callout: SCPreferencesCallBack; var context: SCPreferencesContext ): Boolean; external name '_SCPreferencesSetCallback';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesScheduleWithRunLoop
	@discussion Schedule commit and apply notifications for the specified
		preferences session using the specified run loop and mode.
	@param prefs The preferences session.
	@param runLoop A reference to a run loop on which the notification
		should be scheduled.
		Must be non-NULL.
	@param runLoopMode The mode on which to schedule the notification.
		Must be non-NULL.
	@result Returns TRUE if the notifications are successfully scheduled;
		FALSE otherwise.
 }
function SCPreferencesScheduleWithRunLoop( prefs: SCPreferencesRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCPreferencesScheduleWithRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesUnscheduleFromRunLoop
	@discussion Unschedule commit and apply notifications for the specified
		preferences session from the specified run loop and mode.
	@param prefs The preferences session.
	@param runLoop A reference to a run loop from which the notification
		should be unscheduled.
		Must be non-NULL.
	@param runLoopMode The mode on which to unschedule the notification.
		Must be non-NULL.
	@result Returns TRUE if the notifications are successfully unscheduled;
		FALSE otherwise.
 }
function SCPreferencesUnscheduleFromRunLoop( prefs: SCPreferencesRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCPreferencesUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesSetDispatchQueue
	@discussion Schedule commit and apply notifications for the specified
		preferences session.
	@param prefs The preferences session.
	@param queue The dispatch queue to run the callback function on.
	@result Returns TRUE if the notifications are successfully scheduled;
		FALSE otherwise.
 }
function SCPreferencesSetDispatchQueue( prefs: SCPreferencesRef; queue: dispatch_queue_t ): Boolean; external name '_SCPreferencesSetDispatchQueue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
=======
 }
function SCPreferencesRemoveValue( prefs: SCPreferencesRef; key: CFStringRef ): Boolean; external name '_SCPreferencesRemoveValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCPreferencesSetCallback
	@discussion Assigns a callback to a preferences session.  The function
		is called when the changes to the preferences have been
		committed or applied.
	@param prefs The preferences session.
	@param callout The function to be called when the preferences have
		been changed or applied.
		If NULL, the current callback is removed.
	@param context The SCPreferencesContext associated with
		the callout.
	@result Returns TRUE if the notification client was successfully set.
 }
function SCPreferencesSetCallback( prefs: SCPreferencesRef; callout: SCPreferencesCallBack; var context: SCPreferencesContext ): Boolean; external name '_SCPreferencesSetCallback';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesScheduleWithRunLoop
	@discussion Schedule commit and apply notifications for the specified
		preferences session using the specified run loop and mode.
	@param prefs The preferences session.
	@param runLoop A reference to a run loop on which the notification
		should be scheduled.
		Must be non-NULL.
	@param runLoopMode The mode on which to schedule the notification.
		Must be non-NULL.
	@result Returns TRUE if the notifications are successfully scheduled;
		FALSE otherwise.
 }
function SCPreferencesScheduleWithRunLoop( prefs: SCPreferencesRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCPreferencesScheduleWithRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesUnscheduleFromRunLoop
	@discussion Unschedule commit and apply notifications for the specified
		preferences session from the specified run loop and mode.
	@param prefs The preferences session.
	@param runLoop A reference to a run loop from which the notification
		should be unscheduled.
		Must be non-NULL.
	@param runLoopMode The mode on which to unschedule the notification.
		Must be non-NULL.
	@result Returns TRUE if the notifications are successfully unscheduled;
		FALSE otherwise.
 }
function SCPreferencesUnscheduleFromRunLoop( prefs: SCPreferencesRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCPreferencesUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{$ifc not TARGET_OS_IPHONE}
{!
	@function SCPreferencesSetDispatchQueue
	@discussion Schedule commit and apply notifications for the specified
		preferences session.
	@param prefs The preferences session.
	@param queue The dispatch queue to run the callback function on.
	@result Returns TRUE if the notifications are successfully scheduled;
		FALSE otherwise.
 }
function SCPreferencesSetDispatchQueue( prefs: SCPreferencesRef; queue: dispatch_queue_t ): Boolean; external name '_SCPreferencesSetDispatchQueue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{$endc} {not TARGET_OS_IPHONE}
>>>>>>> graemeg/cpstrnew

{!
	@function SCPreferencesSynchronize
	@discussion Synchronizes accessed preferences with committed changes.

		Any references to preference values returned by calls to the
		SCPreferencesGetValue function are no longer valid unless they
		were explicitly retained or copied.  Any preference values
		that were updated (add, set, remove) but not committed will
		be discarded.
	@param prefs The preferences session.
 }
=======
 }
=======
 }
>>>>>>> graemeg/cpstrnew
=======
 }
>>>>>>> origin/cpstrnew
function SCPreferencesRemoveValue( prefs: SCPreferencesRef; key: CFStringRef ): Boolean; external name '_SCPreferencesRemoveValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCPreferencesSetCallback
	@discussion Assigns a callback to a preferences session.  The function
		is called when the changes to the preferences have been
		committed or applied.
	@param prefs The preferences session.
	@param callout The function to be called when the preferences have
		been changed or applied.
		If NULL, the current callback is removed.
	@param context The SCPreferencesContext associated with
		the callout.
	@result Returns TRUE if the notification client was successfully set.
 }
function SCPreferencesSetCallback( prefs: SCPreferencesRef; callout: SCPreferencesCallBack; var context: SCPreferencesContext ): Boolean; external name '_SCPreferencesSetCallback';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesScheduleWithRunLoop
	@discussion Schedule commit and apply notifications for the specified
		preferences session using the specified run loop and mode.
	@param prefs The preferences session.
	@param runLoop A reference to a run loop on which the notification
		should be scheduled.
		Must be non-NULL.
	@param runLoopMode The mode on which to schedule the notification.
		Must be non-NULL.
	@result Returns TRUE if the notifications are successfully scheduled;
		FALSE otherwise.
 }
function SCPreferencesScheduleWithRunLoop( prefs: SCPreferencesRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCPreferencesScheduleWithRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesUnscheduleFromRunLoop
	@discussion Unschedule commit and apply notifications for the specified
		preferences session from the specified run loop and mode.
	@param prefs The preferences session.
	@param runLoop A reference to a run loop from which the notification
		should be unscheduled.
		Must be non-NULL.
	@param runLoopMode The mode on which to unschedule the notification.
		Must be non-NULL.
	@result Returns TRUE if the notifications are successfully unscheduled;
		FALSE otherwise.
 }
function SCPreferencesUnscheduleFromRunLoop( prefs: SCPreferencesRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCPreferencesUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{$ifc not TARGET_OS_IPHONE}
{!
	@function SCPreferencesSetDispatchQueue
	@discussion Schedule commit and apply notifications for the specified
		preferences session.
	@param prefs The preferences session.
	@param queue The dispatch queue to run the callback function on.
	@result Returns TRUE if the notifications are successfully scheduled;
		FALSE otherwise.
 }
function SCPreferencesSetDispatchQueue( prefs: SCPreferencesRef; queue: dispatch_queue_t ): Boolean; external name '_SCPreferencesSetDispatchQueue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{$endc} {not TARGET_OS_IPHONE}

{!
	@function SCPreferencesSynchronize
	@discussion Synchronizes accessed preferences with committed changes.

		Any references to preference values returned by calls to the
		SCPreferencesGetValue function are no longer valid unless they
		were explicitly retained or copied.  Any preference values
		that were updated (add, set, remove) but not committed will
		be discarded.
	@param prefs The preferences session.
 }
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure SCPreferencesSynchronize( prefs: SCPreferencesRef ); external name '_SCPreferencesSynchronize';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	This function removes the data associated with the specified
	key. In order to commit these changes to permanent storage a
	call must be made to SCPreferencesCommitChanges().
	@param session The SCPreferencesRef handle that should be used to
		communicate with the APIs.
=======
>>>>>>> origin/fixes_2.4
	@param key The preference key to be removed.
	@result Returns TRUE if the value was removed;
		FALSE if the key did not exist or if an error occurred.
 }
function SCPreferencesRemoveValue( prefs: SCPreferencesRef; key: CFStringRef ): Boolean; external name '_SCPreferencesRemoveValue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCPreferencesSetCallback
	@discussion Assigns a callback to a preferences session.  The function
		is called when the changes to the preferences have been
		committed or applied.
	@param prefs The preferences session.
	@param callout The function to be called when the preferences have
		been changed or applied.
		If NULL, the current callback is removed.
	@param context The SCPreferencesContext associated with
		the callout.
	@result Returns TRUE if the notification client was successfully set.
 }
function SCPreferencesSetCallback( prefs: SCPreferencesRef; callout: SCPreferencesCallBack; var context: SCPreferencesContext ): Boolean; external name '_SCPreferencesSetCallback';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesScheduleWithRunLoop
	@discussion Schedule commit and apply notifications for the specified
		preferences session using the specified run loop and mode.
	@param prefs The preferences session.
	@param runLoop A reference to a run loop on which the notification
		should be scheduled.
		Must be non-NULL.
	@param runLoopMode The mode on which to schedule the notification.
		Must be non-NULL.
	@result Returns TRUE if the notifications are successfully scheduled;
		FALSE otherwise.
 }
function SCPreferencesScheduleWithRunLoop( prefs: SCPreferencesRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCPreferencesScheduleWithRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{!
	@function SCPreferencesUnscheduleFromRunLoop
	@discussion Unschedule commit and apply notifications for the specified
		preferences session from the specified run loop and mode.
	@param prefs The preferences session.
	@param runLoop A reference to a run loop from which the notification
		should be unscheduled.
		Must be non-NULL.
	@param runLoopMode The mode on which to unschedule the notification.
		Must be non-NULL.
	@result Returns TRUE if the notifications are successfully unscheduled;
		FALSE otherwise.
 }
function SCPreferencesUnscheduleFromRunLoop( prefs: SCPreferencesRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ): Boolean; external name '_SCPreferencesUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

{$ifc not TARGET_OS_IPHONE}
{!
	@function SCPreferencesSetDispatchQueue
	@discussion Schedule commit and apply notifications for the specified
		preferences session.
	@param prefs The preferences session.
	@param queue The dispatch queue to run the callback function on.
	@result Returns TRUE if the notifications are successfully scheduled;
		FALSE otherwise.
 }
function SCPreferencesSetDispatchQueue( prefs: SCPreferencesRef; queue: dispatch_queue_t ): Boolean; external name '_SCPreferencesSetDispatchQueue';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{$endc} {not TARGET_OS_IPHONE}

{!
	@function SCPreferencesSynchronize
	@discussion Synchronizes accessed preferences with committed changes.

		Any references to preference values returned by calls to the
		SCPreferencesGetValue function are no longer valid unless they
		were explicitly retained or copied.  Any preference values
		that were updated (add, set, remove) but not committed will
		be discarded.
	@param prefs The preferences session.
 }
procedure SCPreferencesSynchronize( prefs: SCPreferencesRef ); external name '_SCPreferencesSynchronize';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_NA) *)

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
