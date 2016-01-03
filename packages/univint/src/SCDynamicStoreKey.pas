{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000-2002, 2004, 2005, 2008 Apple Inc. All rights reserved.
=======
 * Copyright (c) 2000-2002 Apple Computer, Inc. All rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
 * Copyright (c) 2000-2002 Apple Computer, Inc. All rights reserved.
>>>>>>> origin/fixes_2_2
=======
 * Copyright (c) 2000-2002, 2004, 2005, 2008 Apple Inc. All rights reserved.
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

unit SCDynamicStoreKey;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> origin/cpstrnew
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
>>>>>>> graemeg/cpstrnew
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
uses MacTypes,CFBase;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{ until the __IPHONE_NA is automatically handled}
{$ifc TARGET_OS_MAC}


{$ALIGN POWER}
<<<<<<< HEAD

{!
	@header SCDynamicStoreKey
	@discussion The SCDynamicStoreKey API provides convenience functions
		that an application can use to create a correctly formatted
		dynamic store key for accessing specific items in the dynamic
		store.  An application can then use the resulting string in
		any function that requires a dynamic store key.
 }


{
 * SCDynamicStoreKeyCreate
=======
=======
>>>>>>> origin/fixes_2_2
{$ALIGN MAC68K}
=======
>>>>>>> origin/fixes_2.4

{!
	@header SCDynamicStoreKey
	@discussion The SCDynamicStoreKey API provides convenience functions
		that an application can use to create a correctly formatted
		dynamic store key for accessing specific items in the dynamic
		store.  An application can then use the resulting string in
		any function that requires a dynamic store key.
 }


{
<<<<<<< HEAD
 * SCDynamicStoreKeyCreate*
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 * SCDynamicStoreKeyCreate
>>>>>>> origin/fixes_2.4
 * - convenience routines that create a CFString key for an item in the store
 }

{!
	@function SCDynamicStoreKeyCreate
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	@discussion Creates a dynamic store key using the given format.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param fmt A CFStringRef describing the format for this key.
	@result Returns a string containing the formatted key.
<<<<<<< HEAD
 }
function SCDynamicStoreKeyCreate( allocator: CFAllocatorRef; fmt: CFStringRef; ... ): CFStringRef; external name '_SCDynamicStoreKeyCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateNetworkGlobalEntity
	@discussion Creates a dynamic store key that can be used to access
		a specific global (as opposed to a per-service or per-interface)
		network configuration entity.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param domain A string specifying the desired domain, such as the
		requested configuration (kSCDynamicStoreDomainSetup) or the
		actual state (kSCDynamicStoreDomainState).
	@param entity A string containing the specific global entity, such
		as IPv4 (kSCEntNetIPv4) or DNS (kSCEntNetDNS).
	@result Returns a string containing the formatted key.

 }
function SCDynamicStoreKeyCreateNetworkGlobalEntity( allocator: CFAllocatorRef; domain: CFStringRef; entity: CFStringRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateNetworkGlobalEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateNetworkInterface
	@discussion Creates a dynamic store key that can be used to access
		the network interface configuration information stored in
		the dynamic store.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param domain A string specifying the desired domain, such as the
		requested configuration (kSCDynamicStoreDomainSetup) or the
		actual state (kSCDynamicStoreDomainState).
	@result Returns a string containing the formatted key.

 }
function SCDynamicStoreKeyCreateNetworkInterface( allocator: CFAllocatorRef; domain: CFStringRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateNetworkInterface';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateNetworkInterfaceEntity
	@discussion Creates a dynamic store key that can be used to access
		the per-interface network configuration information stored in
		the dynamic store.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param domain A string specifying the desired domain, such as the
		requested configuration (kSCDynamicStoreDomainSetup) or the
		actual state (kSCDynamicStoreDomainState).
	@param ifname A string containing the interface name or a regular
		expression pattern.
	@param entity A string containing the specific global entity, such
		as IPv4 (kSCEntNetIPv4) or DNS (kSCEntNetDNS).
	@result Returns a string containing the formatted key.

 }
function SCDynamicStoreKeyCreateNetworkInterfaceEntity( allocator: CFAllocatorRef; domain: CFStringRef; ifname: CFStringRef; entity: CFStringRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateNetworkInterfaceEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateNetworkServiceEntity
	@discussion Creates a dynamic store key that can be used to access
		the per-service network configuration information stored in
		the dynamic store.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param domain A string specifying the desired domain, such as the
		requested configuration (kSCDynamicStoreDomainSetup) or the
		actual state (kSCDynamicStoreDomainState).
	@param serviceID A string containing the service ID or a regular
		expression pattern.
	@param entity A string containing the specific global entity, such
		as IPv4 (kSCEntNetIPv4) or DNS (kSCEntNetDNS).
	@result Returns a string containing the formatted key.


 }
function SCDynamicStoreKeyCreateNetworkServiceEntity( allocator: CFAllocatorRef; domain: CFStringRef; serviceID: CFStringRef; entity: CFStringRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateNetworkServiceEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateComputerName
	@discussion Creates a key that can be used in conjuntion with
		SCDynamicStoreSetNotificationKeys function to receive
		notifications when the current computer name changes.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the current computer or
		host name.
}
function SCDynamicStoreKeyCreateComputerName( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateComputerName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateConsoleUser
	@discussion Creates a key that can be used in conjunction with
		SCDynamicStoreSetNotificationKeys function to receive
		notifications when the current console user changes.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the current console user.
}
function SCDynamicStoreKeyCreateConsoleUser( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateConsoleUser';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateHostNames
	@discussion Creates a key that can be used in conjunction with the
		SCDynamicStoreSetNotificationKeys function to receive
		notifications when the HostNames entity changes.  The
		HostNames entity includes the local host name.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the HostNames entity.
}
function SCDynamicStoreKeyCreateHostNames( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateHostNames';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateLocation
	@discussion Creates a key that can be used in conjunction with the
		SCDynamicStoreSetNotificationKeys function to receive
		notifications when the location identifier changes.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the current location
		identifier.
}
function SCDynamicStoreKeyCreateLocation( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateLocation';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateProxies
	@discussion Creates a key that can be used in conjunction with
		the SCDynamicStoreSetNotificationKeys function to receive
		notifications when the current network proxy settings
		(such as HTTP or FTP) are changed.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the current proxy settings.
}
function SCDynamicStoreKeyCreateProxies( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateProxies';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	@discussion Creates a store key using the given format.
=======
>>>>>>> origin/fixes_2.4
 }
function SCDynamicStoreKeyCreate( allocator: CFAllocatorRef; fmt: CFStringRef; ... ): CFStringRef; external name '_SCDynamicStoreKeyCreate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateNetworkGlobalEntity
	@discussion Creates a dynamic store key that can be used to access
		a specific global (as opposed to a per-service or per-interface)
		network configuration entity.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param domain A string specifying the desired domain, such as the
		requested configuration (kSCDynamicStoreDomainSetup) or the
		actual state (kSCDynamicStoreDomainState).
	@param entity A string containing the specific global entity, such
		as IPv4 (kSCEntNetIPv4) or DNS (kSCEntNetDNS).
	@result Returns a string containing the formatted key.

 }
function SCDynamicStoreKeyCreateNetworkGlobalEntity( allocator: CFAllocatorRef; domain: CFStringRef; entity: CFStringRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateNetworkGlobalEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateNetworkInterface
	@discussion Creates a dynamic store key that can be used to access
		the network interface configuration information stored in
		the dynamic store.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param domain A string specifying the desired domain, such as the
		requested configuration (kSCDynamicStoreDomainSetup) or the
		actual state (kSCDynamicStoreDomainState).
	@result Returns a string containing the formatted key.

 }
function SCDynamicStoreKeyCreateNetworkInterface( allocator: CFAllocatorRef; domain: CFStringRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateNetworkInterface';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateNetworkInterfaceEntity
	@discussion Creates a dynamic store key that can be used to access
		the per-interface network configuration information stored in
		the dynamic store.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param domain A string specifying the desired domain, such as the
		requested configuration (kSCDynamicStoreDomainSetup) or the
		actual state (kSCDynamicStoreDomainState).
	@param ifname A string containing the interface name or a regular
		expression pattern.
	@param entity A string containing the specific global entity, such
		as IPv4 (kSCEntNetIPv4) or DNS (kSCEntNetDNS).
	@result Returns a string containing the formatted key.

 }
function SCDynamicStoreKeyCreateNetworkInterfaceEntity( allocator: CFAllocatorRef; domain: CFStringRef; ifname: CFStringRef; entity: CFStringRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateNetworkInterfaceEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateNetworkServiceEntity
	@discussion Creates a dynamic store key that can be used to access
		the per-service network configuration information stored in
		the dynamic store.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@param domain A string specifying the desired domain, such as the
		requested configuration (kSCDynamicStoreDomainSetup) or the
		actual state (kSCDynamicStoreDomainState).
	@param serviceID A string containing the service ID or a regular
		expression pattern.
	@param entity A string containing the specific global entity, such
		as IPv4 (kSCEntNetIPv4) or DNS (kSCEntNetDNS).
	@result Returns a string containing the formatted key.


 }
function SCDynamicStoreKeyCreateNetworkServiceEntity( allocator: CFAllocatorRef; domain: CFStringRef; serviceID: CFStringRef; entity: CFStringRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateNetworkServiceEntity';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateComputerName
	@discussion Creates a key that can be used in conjuntion with
		SCDynamicStoreSetNotificationKeys function to receive
		notifications when the current computer name changes.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the current computer or
		host name.
}
function SCDynamicStoreKeyCreateComputerName( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateComputerName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateConsoleUser
	@discussion Creates a key that can be used in conjunction with
		SCDynamicStoreSetNotificationKeys function to receive
		notifications when the current console user changes.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the current console user.
}
function SCDynamicStoreKeyCreateConsoleUser( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateConsoleUser';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateHostNames
	@discussion Creates a key that can be used in conjunction with the
		SCDynamicStoreSetNotificationKeys function to receive
		notifications when the HostNames entity changes.  The
		HostNames entity includes the local host name.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the HostNames entity.
}
function SCDynamicStoreKeyCreateHostNames( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateHostNames';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateLocation
	@discussion Creates a key that can be used in conjunction with the
		SCDynamicStoreSetNotificationKeys function to receive
		notifications when the location identifier changes.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the current location
		identifier.
}
function SCDynamicStoreKeyCreateLocation( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateLocation';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_NA) *)

{!
	@function SCDynamicStoreKeyCreateProxies
	@discussion Creates a key that can be used in conjunction with
		the SCDynamicStoreSetNotificationKeys function to receive
		notifications when the current network proxy settings
		(such as HTTP or FTP) are changed.
	@param allocator The CFAllocator that should be used to allocate
		memory for this key.
		This parameter may be NULL in which case the current
		default CFAllocator is used. If this reference is not
		a valid CFAllocator, the behavior is undefined.
	@result Returns a notification string for the current proxy settings.
}
function SCDynamicStoreKeyCreateProxies( allocator: CFAllocatorRef ): CFStringRef; external name '_SCDynamicStoreKeyCreateProxies';
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
