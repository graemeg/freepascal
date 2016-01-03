{	CFBase.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1998-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 1998-2009, Apple, Inc. All rights reserved.
}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{ 		Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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
	Copyright (c) 1998-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1998-2009, Apple, Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{ 		Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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

unit CFBase;
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
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
=======
{$elsec}
>>>>>>> origin/cpstrnew
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

type
  { type moved here to avoid circular dependency between Files and CFURL }
	CFURLRef = ^SInt32; { an opaque 32-bit type }
	CFURLRefPtr = ^CFURLRef;

var kCFCoreFoundationVersionNumber: Float64; external name '_kCFCoreFoundationVersionNumber'; (* attribute const *)

{$ifc TARGET_OS_MAC}
const
	kCFCoreFoundationVersionNumber10_0 = 196.40;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
	kCFCoreFoundationVersionNumber10_0_3 = 196.50;
const
	kCFCoreFoundationVersionNumber10_1 = 226.00;
const
	kCFCoreFoundationVersionNumber10_1_1 = 226.00;
{ Note the next three do not follow the usual numbering policy from the base release }
<<<<<<< HEAD
<<<<<<< HEAD
const
	kCFCoreFoundationVersionNumber10_1_2 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_3 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_4 = 227.30;
const
	kCFCoreFoundationVersionNumber10_2 = 263.00;
const
	kCFCoreFoundationVersionNumber10_2_1 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_2 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_3 = 263.30;
const
=======
=======
>>>>>>> graemeg/cpstrnew
const
	kCFCoreFoundationVersionNumber10_0_3 = 196.50;
const
	kCFCoreFoundationVersionNumber10_1 = 226.00;
const
	kCFCoreFoundationVersionNumber10_1_1 = 226.00;
{ Note the next three do not follow the usual numbering policy from the base release }
const
	kCFCoreFoundationVersionNumber10_1_2 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_3 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_4 = 227.30;
const
	kCFCoreFoundationVersionNumber10_2 = 263.00;
const
	kCFCoreFoundationVersionNumber10_2_1 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_2 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_3 = 263.30;
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
const
	kCFCoreFoundationVersionNumber10_1_2 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_3 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_4 = 227.30;
const
	kCFCoreFoundationVersionNumber10_2 = 263.00;
const
	kCFCoreFoundationVersionNumber10_2_1 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_2 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_3 = 263.30;
const
>>>>>>> graemeg/cpstrnew
=======
const
	kCFCoreFoundationVersionNumber10_1_2 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_3 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_4 = 227.30;
const
	kCFCoreFoundationVersionNumber10_2 = 263.00;
const
	kCFCoreFoundationVersionNumber10_2_1 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_2 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_3 = 263.30;
const
>>>>>>> origin/cpstrnew
	kCFCoreFoundationVersionNumber10_2_4 = 263.30;
const
	kCFCoreFoundationVersionNumber10_2_5 = 263.50;
const
	kCFCoreFoundationVersionNumber10_2_6 = 263.50;
const
	kCFCoreFoundationVersionNumber10_2_7 = 263.50;
const
	kCFCoreFoundationVersionNumber10_2_8 = 263.50;
const
	kCFCoreFoundationVersionNumber10_3 = 299.00;
const
	kCFCoreFoundationVersionNumber10_3_1 = 299.00;
const
	kCFCoreFoundationVersionNumber10_3_2 = 299.00;
const
	kCFCoreFoundationVersionNumber10_3_3 = 299.30;
const
	kCFCoreFoundationVersionNumber10_3_4 = 299.31;
const
	kCFCoreFoundationVersionNumber10_3_5 = 299.31;
const
	kCFCoreFoundationVersionNumber10_3_6 = 299.32;
const
	kCFCoreFoundationVersionNumber10_3_7 = 299.33;
const
	kCFCoreFoundationVersionNumber10_3_8 = 299.33;
const
	kCFCoreFoundationVersionNumber10_3_9 = 299.35;
const
	kCFCoreFoundationVersionNumber10_4 = 368.00;
const
	kCFCoreFoundationVersionNumber10_4_1 = 368.10;
const
	kCFCoreFoundationVersionNumber10_4_2 = 368.11;
const
	kCFCoreFoundationVersionNumber10_4_3 = 368.18;
const
	kCFCoreFoundationVersionNumber10_4_4_Intel = 368.26;
const
	kCFCoreFoundationVersionNumber10_4_4_PowerPC = 368.25;
const
	kCFCoreFoundationVersionNumber10_4_5_Intel = 368.26;
const
	kCFCoreFoundationVersionNumber10_4_5_PowerPC = 368.25;
const
	kCFCoreFoundationVersionNumber10_4_6_Intel = 368.26;
const
	kCFCoreFoundationVersionNumber10_4_6_PowerPC = 368.25;
const
	kCFCoreFoundationVersionNumber10_4_7 = 368.27;
const
	kCFCoreFoundationVersionNumber10_4_8 = 368.27;
const
	kCFCoreFoundationVersionNumber10_4_9 = 368.28;
const
	kCFCoreFoundationVersionNumber10_4_10 = 368.28;
const
	kCFCoreFoundationVersionNumber10_4_11 = 368.31;
const
	kCFCoreFoundationVersionNumber10_5 = 476.00;
const
	kCFCoreFoundationVersionNumber10_5_1 = 476.00;
const
	kCFCoreFoundationVersionNumber10_5_2 = 476.10;
const
	kCFCoreFoundationVersionNumber10_5_3 = 476.13;
const
	kCFCoreFoundationVersionNumber10_5_4 = 476.14;
const
	kCFCoreFoundationVersionNumber10_5_5 = 476.15;
const
	kCFCoreFoundationVersionNumber10_5_6 = 476.17;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kCFCoreFoundationVersionNumber10_5_7 = 476.18;
const
	kCFCoreFoundationVersionNumber10_5_8 = 476.19;
const
	kCFCoreFoundationVersionNumber10_6 = 550.00;
const
	kCFCoreFoundationVersionNumber10_6_1 = 550.00;
const
	kCFCoreFoundationVersionNumber10_6_2 = 550.13;
const
	kCFCoreFoundationVersionNumber10_6_3 = 550.19;
const
	kCFCoreFoundationVersionNumber10_6_4 = 550.29;
const
	kCFCoreFoundationVersionNumber10_6_5 = 550.42;
const
	kCFCoreFoundationVersionNumber10_6_6 = 550.42;
const
	kCFCoreFoundationVersionNumber10_6_7 = 550.42;
const
	kCFCoreFoundationVersionNumber10_6_8 = 550.43;
const
	kCFCoreFoundationVersionNumber10_7 = 635.00;
const
	kCFCoreFoundationVersionNumber10_7_1 = 635.00;
const
	kCFCoreFoundationVersionNumber10_7_2 = 635.15;
const
	kCFCoreFoundationVersionNumber10_7_3 = 635.19;
const
	kCFCoreFoundationVersionNumber10_7_4 = 635.21;
const
	kCFCoreFoundationVersionNumber10_7_5 = 635.21;
const
	kCFCoreFoundationVersionNumber10_8 = 744.00;
const
	kCFCoreFoundationVersionNumber10_8_1 = 744.00;
const
	kCFCoreFoundationVersionNumber10_8_2 = 744.12;
const
	kCFCoreFoundationVersionNumber10_8_3 = 744.18;
const
	kCFCoreFoundationVersionNumber10_8_4 = 744.19;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endc}

{$ifc TARGET_OS_IPHONE}
const
	kCFCoreFoundationVersionNumber_iPhoneOS_2_0 = 478.23;
const
	kCFCoreFoundationVersionNumber_iPhoneOS_2_1 = 478.26;
const
	kCFCoreFoundationVersionNumber_iPhoneOS_2_2 = 478.29;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kCFCoreFoundationVersionNumber_iPhoneOS_3_0 = 478.47;
const
	kCFCoreFoundationVersionNumber_iPhoneOS_3_1 = 478.52;
const
	kCFCoreFoundationVersionNumber_iPhoneOS_3_2 = 478.61;
const
	kCFCoreFoundationVersionNumber_iOS_4_0 = 550.32;
const
	kCFCoreFoundationVersionNumber_iOS_4_1 = 550.38;
const
	kCFCoreFoundationVersionNumber_iOS_4_2 = 550.52;
const
	kCFCoreFoundationVersionNumber_iOS_4_3 = 550.52;
const
	kCFCoreFoundationVersionNumber_iOS_5_0 = 675.00;
const
	kCFCoreFoundationVersionNumber_iOS_5_1 = 690.10;
const
	kCFCoreFoundationVersionNumber_iOS_6_0 = 793.00;
const
	kCFCoreFoundationVersionNumber_iOS_6_1 = 793.00;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endc}

type
	CFTypeID = UNSIGNEDLONG;
	CFOptionFlags = UNSIGNEDLONG;
	CFHashCode = UNSIGNEDLONG;
	CFIndex = SIGNEDLONG;
	CFIndexPtr = ^CFIndex;

{ Base "type" of all "CF objects", and polymorphic functions on them }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFTypeRef = UnivPtr; { an opaque type }
=======
	CFTypeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTypeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTypeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTypeRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	
{ GK: We need it for passing open arrays of CFTypes in MDQuery.pas }
	CFTypeRefPtr = ^CFTypeRef;
type
	CFStringRef = ^SInt32; { an opaque type }
	CFStringRefPtr = ^CFStringRef;
	CFMutableStringRef = ^SInt32; { an opaque type }
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

type
  { type moved here to avoid circular dependency between Files and CFURL }
	CFURLRef = ^SInt32; { an opaque 32-bit type }
	CFURLRefPtr = ^CFURLRef;

var kCFCoreFoundationVersionNumber: Float64; external name '_kCFCoreFoundationVersionNumber'; (* attribute const *)

{$ifc TARGET_OS_MAC}
const
	kCFCoreFoundationVersionNumber10_0 = 196.40;
const
	kCFCoreFoundationVersionNumber10_0_3 = 196.50;
const
	kCFCoreFoundationVersionNumber10_1 = 226.00;
const
	kCFCoreFoundationVersionNumber10_1_1 = 226.00;
{ Note the next three do not follow the usual numbering policy from the base release }
const
	kCFCoreFoundationVersionNumber10_1_2 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_3 = 227.20;
const
	kCFCoreFoundationVersionNumber10_1_4 = 227.30;
const
	kCFCoreFoundationVersionNumber10_2 = 263.00;
const
	kCFCoreFoundationVersionNumber10_2_1 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_2 = 263.10;
const
	kCFCoreFoundationVersionNumber10_2_3 = 263.30;
const
	kCFCoreFoundationVersionNumber10_2_4 = 263.30;
const
	kCFCoreFoundationVersionNumber10_2_5 = 263.50;
const
	kCFCoreFoundationVersionNumber10_2_6 = 263.50;
const
	kCFCoreFoundationVersionNumber10_2_7 = 263.50;
const
	kCFCoreFoundationVersionNumber10_2_8 = 263.50;
const
	kCFCoreFoundationVersionNumber10_3 = 299.00;
const
	kCFCoreFoundationVersionNumber10_3_1 = 299.00;
const
	kCFCoreFoundationVersionNumber10_3_2 = 299.00;
const
	kCFCoreFoundationVersionNumber10_3_3 = 299.30;
const
	kCFCoreFoundationVersionNumber10_3_4 = 299.31;
const
	kCFCoreFoundationVersionNumber10_3_5 = 299.31;
const
	kCFCoreFoundationVersionNumber10_3_6 = 299.32;
const
	kCFCoreFoundationVersionNumber10_3_7 = 299.33;
const
	kCFCoreFoundationVersionNumber10_3_8 = 299.33;
const
	kCFCoreFoundationVersionNumber10_3_9 = 299.35;
const
	kCFCoreFoundationVersionNumber10_4 = 368.00;
const
	kCFCoreFoundationVersionNumber10_4_1 = 368.10;
const
	kCFCoreFoundationVersionNumber10_4_2 = 368.11;
const
	kCFCoreFoundationVersionNumber10_4_3 = 368.18;
const
	kCFCoreFoundationVersionNumber10_4_4_Intel = 368.26;
const
	kCFCoreFoundationVersionNumber10_4_4_PowerPC = 368.25;
const
	kCFCoreFoundationVersionNumber10_4_5_Intel = 368.26;
const
	kCFCoreFoundationVersionNumber10_4_5_PowerPC = 368.25;
const
	kCFCoreFoundationVersionNumber10_4_6_Intel = 368.26;
const
	kCFCoreFoundationVersionNumber10_4_6_PowerPC = 368.25;
const
	kCFCoreFoundationVersionNumber10_4_7 = 368.27;
const
	kCFCoreFoundationVersionNumber10_4_8 = 368.27;
const
	kCFCoreFoundationVersionNumber10_4_9 = 368.28;
const
	kCFCoreFoundationVersionNumber10_4_10 = 368.28;
const
	kCFCoreFoundationVersionNumber10_4_11 = 368.31;
const
	kCFCoreFoundationVersionNumber10_5 = 476.00;
const
	kCFCoreFoundationVersionNumber10_5_1 = 476.00;
const
	kCFCoreFoundationVersionNumber10_5_2 = 476.10;
const
	kCFCoreFoundationVersionNumber10_5_3 = 476.13;
const
	kCFCoreFoundationVersionNumber10_5_4 = 476.14;
const
	kCFCoreFoundationVersionNumber10_5_5 = 476.15;
const
	kCFCoreFoundationVersionNumber10_5_6 = 476.17;
{$endc}

{$ifc TARGET_OS_IPHONE}
const
	kCFCoreFoundationVersionNumber_iPhoneOS_2_0 = 478.23;
const
	kCFCoreFoundationVersionNumber_iPhoneOS_2_1 = 478.26;
const
	kCFCoreFoundationVersionNumber_iPhoneOS_2_2 = 478.29;
{$endc}

type
	CFTypeID = UNSIGNEDLONG;
	CFOptionFlags = UNSIGNEDLONG;
	CFHashCode = UNSIGNEDLONG;
	CFIndex = SIGNEDLONG;
	CFIndexPtr = ^CFIndex;

{ Base "type" of all "CF objects", and polymorphic functions on them }
type
	CFTypeRef = ^SInt32; { an opaque type }
	
{ GK: We need it for passing open arrays of CFTypes in MDQuery.pas }
	CFTypeRefPtr = ^CFTypeRef;
type
	CFStringRef = ^SInt32; { an opaque type }
	CFStringRefPtr = ^CFStringRef;
<<<<<<< HEAD
	CFMutableStringRef = ^SInt32; { an opaque 32-bit type }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	CFMutableStringRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CFMutableStringRefPtr = ^CFMutableStringRef;

{
        Type to mean any instance of a property list type;
        currently, CFString, CFData, CFNumber, CFBoolean, CFDate,
        CFArray, and CFDictionary.
}
type
	CFPropertyListRef = CFTypeRef;

{ Values returned from comparison functions }
	CFComparisonResult = SInt32;
const
	kCFCompareLessThan = -1;
	kCFCompareEqualTo = 0;
	kCFCompareGreaterThan = 1;

{ A standard comparison function }
type
	CFComparatorFunction = function( val1: {const} UnivPtr; val2: {const} UnivPtr; context: UnivPtr ): CFComparisonResult;

{ Constant used by some functions to indicate failed searches. }
{ This is of type CFIndex. }
const
	kCFNotFound = -1;


{ Range type }
type
	CFRangePtr = ^CFRange;
	CFRange = record
		location: CFIndex;
		length: CFIndex;
	end;

function CFRangeMake( loc: CFIndex; len: CFIndex ): CFRange; external name '___CFRangeMake';

{ Private; do not use }
function __CFRangeMake( loc: CFIndex; len: CFIndex ): CFRange; external name '___CFRangeMake';


{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{ Null representant }

type
	CFNullRef = ^SInt32; { an opaque 32-bit type }

function CFNullGetTypeID: CFTypeID; external name '_CFNullGetTypeID';

var kCFNull: CFNullRef; external name '_kCFNull'; (* attribute const *)	// the singleton null instance

{#endif}


{ Allocator API

   Most of the time when specifying an allocator to Create functions, the NULL
   argument indicates "use the default"; this is the same as using kCFAllocatorDefault
   or the return value from CFAllocatorGetDefault().  This assures that you will use
   the allocator in effect at that time.

   You should rarely use kCFAllocatorSystemDefault, the default default allocator.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
	CFAllocatorRef = ^__CFAllocator; { an opaque type }
	__CFAllocator = record end;
=======
	CFAllocatorRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFAllocatorRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
	CFAllocatorRefPtr = ^CFAllocatorRef;

{ This is a synonym for NULL, if you'd rather use a named constant. }
var kCFAllocatorDefault: CFAllocatorRef; external name '_kCFAllocatorDefault'; (* attribute const *)

{ Default system allocator; you rarely need to use this. }
var kCFAllocatorSystemDefault: CFAllocatorRef; external name '_kCFAllocatorSystemDefault'; (* attribute const *)

{ This allocator uses malloc(), realloc(), and free(). This should not be
   generally used; stick to kCFAllocatorDefault whenever possible. This
   allocator is useful as the "bytesDeallocator" in CFData or
   "contentsDeallocator" in CFString where the memory was obtained as a
   result of malloc() type functions.
}
var kCFAllocatorMalloc: CFAllocatorRef; external name '_kCFAllocatorMalloc'; (* attribute const *)

{ This allocator explicitly uses the default malloc zone, returned by
   malloc_default_zone(). It should only be used when an object is
   safe to be allocated in non-scanned memory.
 }
var kCFAllocatorMallocZone: CFAllocatorRef; external name '_kCFAllocatorMallocZone'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Null allocator which does nothing and allocates no memory. This allocator
   is useful as the "bytesDeallocator" in CFData or "contentsDeallocator"
   in CFString where the memory should not be freed. 
}
var kCFAllocatorNull: CFAllocatorRef; external name '_kCFAllocatorNull'; (* attribute const *)

{ Special allocator argument to CFAllocatorCreate() which means
   "use the functions given in the context to allocate the allocator
   itself as well". 
}
var kCFAllocatorUseContext: CFAllocatorRef; external name '_kCFAllocatorUseContext'; (* attribute const *)

type
	CFAllocatorRetainCallBack = function( info: {const} UnivPtr ): UnivPtr;
	CFAllocatorReleaseCallBack = procedure( info: {const} UnivPtr );
	CFAllocatorCopyDescriptionCallBack = function( info: {const} UnivPtr ): CFStringRef;
	CFAllocatorAllocateCallBack = function( allocSize: CFIndex; hint: CFOptionFlags; info: UnivPtr ): UnivPtr;
	CFAllocatorReallocateCallBack = function( ptr: UnivPtr; newsize: CFIndex; hint: CFOptionFlags; info: UnivPtr ): UnivPtr;
	CFAllocatorDeallocateCallBack = procedure( ptr: UnivPtr; info: UnivPtr );
	CFAllocatorPreferredSizeCallBack = function( size: CFIndex; hint: CFOptionFlags; info: UnivPtr ): CFIndex;
	CFAllocatorContextPtr = ^CFAllocatorContext;
	CFAllocatorContext = record
		version: CFIndex;
		info: UnivPtr;
		retain: CFAllocatorRetainCallBack;
		release: CFAllocatorReleaseCallBack;        
		copyDescription: CFAllocatorCopyDescriptionCallBack;
		allocate: CFAllocatorAllocateCallBack;
		reallocate: CFAllocatorReallocateCallBack;
		deallocate: CFAllocatorDeallocateCallBack;
		preferredSize: CFAllocatorPreferredSizeCallBack;
	end;

function CFAllocatorGetTypeID: CFTypeID; external name '_CFAllocatorGetTypeID';

{
	CFAllocatorSetDefault() sets the allocator that is used in the current
	thread whenever NULL is specified as an allocator argument. This means
	that most, if not all allocations will go through this allocator. It
	also means that any allocator set as the default needs to be ready to
	deal with arbitrary memory allocation requests; in addition, the size
	and number of requests will change between releases.

	An allocator set as the default will never be released, even if later
	another allocator replaces it as the default. Not only is it impractical
	for it to be released (as there might be caches created under the covers
	that refer to the allocator), in general it's also safer and more
	efficient to keep it around.

	If you wish to use a custom allocator in a context, it's best to provide
	it as the argument to the various creation functions rather than setting
	it as the default. Setting the default allocator is not encouraged.

	If you do set an allocator as the default, either do it for all time in
	your app, or do it in a nested fashion (by restoring the previous allocator
	when you exit your context). The latter might be appropriate for plug-ins
	or libraries that wish to set the default allocator.
}
procedure CFAllocatorSetDefault( allocator: CFAllocatorRef ); external name '_CFAllocatorSetDefault';

function CFAllocatorGetDefault: CFAllocatorRef; external name '_CFAllocatorGetDefault';

function CFAllocatorCreate( allocator: CFAllocatorRef; var context: CFAllocatorContext ): CFAllocatorRef; external name '_CFAllocatorCreate';

function CFAllocatorAllocate( allocator: CFAllocatorRef; size: CFIndex; hint: CFOptionFlags ): UnivPtr; external name '_CFAllocatorAllocate';

function CFAllocatorReallocate( allocator: CFAllocatorRef; ptr: UnivPtr; newsize: CFIndex; hint: CFOptionFlags ): UnivPtr; external name '_CFAllocatorReallocate';

procedure CFAllocatorDeallocate( allocator: CFAllocatorRef; ptr: UnivPtr ); external name '_CFAllocatorDeallocate';

function CFAllocatorGetPreferredSizeForSize( allocator: CFAllocatorRef; size: CFIndex; hint: CFOptionFlags ): CFIndex; external name '_CFAllocatorGetPreferredSizeForSize';

procedure CFAllocatorGetContext( allocator: CFAllocatorRef; var context: CFAllocatorContext ); external name '_CFAllocatorGetContext';


{ Polymorphic CF functions }

function CFGetTypeID( cf: CFTypeRef ): CFTypeID; external name '_CFGetTypeID';

function CFCopyTypeIDDescription( type_id: CFTypeID ): CFStringRef; external name '_CFCopyTypeIDDescription';

function CFRetain( cf: CFTypeRef ): CFTypeRef; external name '_CFRetain';

procedure CFRelease( cf: CFTypeRef ); external name '_CFRelease';

<<<<<<< HEAD
<<<<<<< HEAD
function CFAutorelease( arg: CFTypeRef {CF_RELEASES_ARGUMENT} ): CFTypeRef; external name '_CFAutorelease';
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)

function CFGetRetainCount( cf: CFTypeRef ): CFIndex; external name '_CFGetRetainCount';

=======
=======
>>>>>>> origin/fixes_2_2
function CFGetRetainCount( cf: CFTypeRef ): CFIndex; external name '_CFGetRetainCount';

function CFMakeCollectable( cf: CFTypeRef ): CFTypeRef; external name '_CFMakeCollectable';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function CFEqual( cf1: CFTypeRef; cf2: CFTypeRef ): Boolean; external name '_CFEqual';

function CFHash( cf: CFTypeRef ): CFHashCode; external name '_CFHash';

function CFCopyDescription( cf: CFTypeRef ): CFStringRef; external name '_CFCopyDescription';

function CFGetAllocator( cf: CFTypeRef ): CFAllocatorRef; external name '_CFGetAllocator';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// This function is unavailable in ARC mode. Use CFBridgingRelease instead.
{ CF_AUTOMATED_REFCOUNT_UNAVAILABLE }
function CFMakeCollectable( cf: CFTypeRef ): CFTypeRef; external name '_CFMakeCollectable';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *) 

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
