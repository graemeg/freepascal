{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QD/QuickdrawAPI.h
 
     Contains:   API Prototypes from the former Quickdraw.i
 
     Version:    Quickdraw-262~1
 
     Copyright:  © 2005-2008 by Apple Inc. all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       QD/Quickdraw.h
=======
     File:       QD/QuickdrawAPI.h
>>>>>>> origin/fixes_2.4
 
     Contains:   API Prototypes from the former Quickdraw.i
 
     Version:    Quickdraw-262~1
 
<<<<<<< HEAD
     Copyright:  © 1985-2006 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 2005-2008 by Apple Inc. all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
                     http://www.freepascal.org/bugs.html
 
}
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit Quickdraw;
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
{$ifc not defined __x86_64__ and defined CPUX86_64}
	{$setc __x86_64__ := 1}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}
{$ifc not defined __arm64__ and defined CPUAARCH64}
  {$setc __arm64__ := 1}
{$elsec}
  {$setc __arm64__ := 0}
{$endc}

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
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}

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
=======
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

=======
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

>>>>>>> graemeg/cpstrnew
{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
=======
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}

=======
>>>>>>> origin/fixes_2_2

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
<<<<<<< HEAD
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
{$elifc defined __arm__ and __arm__}
>>>>>>> graemeg/cpstrnew
=======
{$elifc defined __arm__ and __arm__}
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> origin/cpstrnew
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
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

>>>>>>> origin/cpstrnew
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
uses MacTypes,QDCMCommon,QuickdrawTypes,ColorSyncDeprecated,CGDirectDisplay,Components,MixedMode,QuickdrawText,CGContext;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{*********************************************************************************
// NOTE: Quickdraw has been deprecated for deployment targets 10.4 and later.
// The replacement API is Quartz (CoreGraphics).
// Because of the fundamental differences in the imaging models and design
// goals between Quickdraw and Quartz, there is no direct correspondence
// possible (or even desirable) between Quickdraw and Quartz APIs and concepts.
// For certain purposes, some Quickdraw functions may even still be needed during
// a transition period; nevertheless, they have all been tagged as deprecated
// to express the overriding goal of eliminating all Quickdraw usage in the future.
 |********************************************************************************}

=======
=======
>>>>>>> origin/fixes_2_2
	{$setc TARGET_CPU_X86 := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
{$elsec}
	{$error Neither __ppc__ nor __i386__ is defined.}
{$endc}
{$setc TARGET_CPU_PPC_64 := FALSE}

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
{$setc TARGET_OS_MAC := TRUE}
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,CMTypes,CGDirectDisplay,Components,MixedMode,QuickdrawText,CGContext;


{$ALIGN MAC68K}

{*********************************************************************************
// NOTE: Quickdraw has been deprecated for deployment targets 10.4 and later.
// The replacement API is Quartz (CoreGraphics).
// Because of the fundamental differences in the imaging models and design
// goals between Quickdraw and Quartz, there is no direct correspondence
// possible (or even desirable) between Quickdraw and Quartz APIs and concepts.
// For certain purposes, some Quickdraw functions may even still be needed during
// a transition period; nevertheless, they have all been tagged as deprecated
// to express the overriding goal of eliminating all Quickdraw usage in the future.
 |********************************************************************************}


const
	invalColReq					= -1;							{ invalid color table request }

																{  transfer modes  }
	srcCopy						= 0;							{ the 16 transfer modes }
	srcOr						= 1;
	srcXor						= 2;
	srcBic						= 3;
	notSrcCopy					= 4;
	notSrcOr					= 5;
	notSrcXor					= 6;
	notSrcBic					= 7;
	patCopy						= 8;
	patOr						= 9;
	patXor						= 10;
	patBic						= 11;
	notPatCopy					= 12;
	notPatOr					= 13;
	notPatXor					= 14;
	notPatBic					= 15;							{  Special Text Transfer Mode  }
	grayishTextOr				= 49;
	hilitetransfermode			= 50;
	hilite						= 50;							{  Arithmetic transfer modes  }
	blend						= 32;
	addPin						= 33;
	addOver						= 34;
	subPin						= 35;
	addMax						= 37;
	adMax						= 37;
	subOver						= 38;
	adMin						= 39;
	ditherCopy					= 64;							{  Transparent mode constant  }
	transparent					= 36;

	italicBit					= 1;
	ulineBit					= 2;
	outlineBit					= 3;
	shadowBit					= 4;
	condenseBit					= 5;
	extendBit					= 6;

																{  QuickDraw color separation constants  }
	normalBit					= 0;							{ normal screen mapping }
	inverseBit					= 1;							{ inverse screen mapping }
	redBit						= 4;							{ RGB additive mapping }
	greenBit					= 3;
	blueBit						= 2;
	cyanBit						= 8;							{ CMYBk subtractive mapping }
	magentaBit					= 7;
	yellowBit					= 6;
	blackBit					= 5;

	blackColor					= 33;							{ colors expressed in these mappings }
	whiteColor					= 30;
	redColor					= 205;
	greenColor					= 341;
	blueColor					= 409;
	cyanColor					= 273;
	magentaColor				= 137;
	yellowColor					= 69;

	picLParen					= 0;							{ standard picture comments }
	picRParen					= 1;
	clutType					= 0;							{ 0 if lookup table }
	fixedType					= 1;							{ 1 if fixed table }
	directType					= 2;							{ 2 if direct values }

    { gdFlags bits. Bits 1..10 are legacy, and currently unused }
	gdDevType					= 0;							{ 0 = monochrome 1 = color }

	interlacedDevice			= 2;							{  1 if single pixel lines look bad  }
	hwMirroredDevice			= 4;							{  1 if device is HW mirrored  }
	roundedDevice				= 5;							{  1 if device has been ÒroundedÓ into the GrayRgn  }
	hasAuxMenuBar				= 6;							{  1 if device has an aux menu bar on it  }
	burstDevice					= 7;
	ext32Device					= 8;
	ramInit						= 10;							{ 1 if initialized from 'scrn' resource }
	mainScreen					= 11;							{  1 if main screen  }
	allInit						= 12;							{  1 if all devices initialized  }
	screenDevice				= 13;							{ 1 if screen device [not used] }
	noDriver					= 14;							{  1 if no driver for this GDevice  }
	screenActive				= 15;							{ 1 if in use }
	hiliteBit					= 7;							{ flag bit in LMGet/SetHiliteMode }
	pHiliteBit					= 0;							{ flag bit in LMGet/SetHiliteMode when used with BitClr }

    { miscellaneous constants }
	defQDColors					= 127;							{ resource ID of clut for default QDColors } { (legacy - now unused) }
																{  pixel type  }
	RGBDirect					= 16;							{  16 & 32 bits/pixel pixelType value  }
																{  pmVersion values  }
	baseAddr32					= 4;							{ pixmap base address is 32-bit address }


	sysPatListID				= 0;
	iBeamCursor					= 1;
	crossCursor					= 2;
	plusCursor					= 3;
	watchCursor					= 4;

	kQDGrafVerbFrame			= 0;
	kQDGrafVerbPaint			= 1;
	kQDGrafVerbErase			= 2;
	kQDGrafVerbInvert			= 3;
	kQDGrafVerbFill				= 4;

{$ifc OLDROUTINENAMES}
	frame 						= kQDGrafVerbFrame;
	paint 						= kQDGrafVerbPaint;
	erase 						= kQDGrafVerbErase;
	invert 						= kQDGrafVerbInvert;
	fill 						= kQDGrafVerbFill;

{$endc}  {OLDROUTINENAMES}


type
	GrafVerb							= SInt8;

const
	chunky						= 0;
	chunkyPlanar				= 1;
	planar						= 2;


type
	PixelType							= SInt8;
	Bits16								= array [0..15] of SInt16;

	{	**************   IMPORTANT NOTE REGARDING Pattern  **************************************
	   Patterns were originally defined as:
	   
	        C:          typedef unsigned char Pattern[8];
	        Pascal:     Pattern = packed array [0..7] of 0..255;
	        
	   The old array definition of Pattern would cause 68000 based CPU's to crash in certain circum-
	   stances. The new struct definition is safe, but may require source code changes to compile.
	    
	********************************************************************************************	}
	PatternPtr = ^Pattern;
	Pattern = record
		pat:					packed array [0..7] of UInt8;
	end;

{
 ConstPatternParam is no longer needed.  It was first created when Pattern was an array.
 Now that Pattern is a struct, it is more straight forward to just add the "const" qualifier
 on the parameter type (e.g. "const Pattern * pat" instead of "ConstPatternParam pat").

	ConstPatternParam = PatternPtr;
}

	PatPtr								= ^Pattern;
	PatHandle							= ^PatPtr;
	QDByte								= SignedByte;
	QDPtr								= Ptr;
	QDHandle							= Handle;
	QDErr								= SInt16;

const
	singleDevicesBit			= 0;
	dontMatchSeedsBit			= 1;
	allDevicesBit				= 2;

	singleDevices				= $01;
	dontMatchSeeds				= $02;
	allDevices					= $04;


type
	DeviceLoopFlags						= UInt32;
	{	
	    PrinterStatusOpcode.  For communication with downloading and printing services.
		}
	PrinterStatusOpcode					= SInt32;

const
	kPrinterFontStatus			= 0;
	kPrinterScalingStatus		= 1;


type
	PrinterFontStatusPtr = ^PrinterFontStatus;
	PrinterFontStatus = record
		oResult:				SInt32;
		iFondID:				SInt16;
		iStyle:					Style;
	end;

	PrinterScalingStatusPtr = ^PrinterScalingStatus;
	PrinterScalingStatus = record
		oScalingFactors:		Point;
	end;

	BitMapPtr = ^BitMap;
	BitMap = record
		baseAddr:				Ptr;
		rowBytes:				SInt16;
		bounds:					Rect;
	end;

	BitMapHandle						= ^BitMapPtr;
	CursorPtr = ^Cursor;
	Cursor = record
		data:					Bits16;
		mask:					Bits16;
		hotSpot:				Point;
	end;

	CursPtr								= ^Cursor;
	CursHandle							= ^CursPtr;
	PenStatePtr = ^PenState;
	PenState = record
		pnLoc:					Point;
		pnSize:					Point;
		pnMode:					SInt16;
		pnPat:					Pattern;
	end;

{$ifc NOT OPAQUE_TOOLBOX_STRUCTS}
	MacRegionPtr = ^MacRegion;
	MacRegion = record
		rgnSize:				UInt16;									{  size in bytes; don't rely on it  }
		rgnBBox:				Rect;									{  enclosing rectangle; in Carbon use GetRegionBounds  }
	end;

	{
	   The type name "Region" has a name space collision on Win32.
	   Use MacRegion to be cross-platfrom safe.
	}
{$ifc TARGET_OS_MAC}
	Region								= MacRegion;
	RegionPtr 							= ^Region;
{$endc}  {TARGET_OS_MAC}
	RgnPtr								= ^MacRegion;
	RgnHandle							= ^RgnPtr;
{$elsec}
	RgnHandle    = ^SInt32; { an opaque 32-bit type }
	RgnHandlePtr = ^RgnHandle;  { when a var xx:RgnHandle parameter can be nil, it is changed to xx: RgnHandlePtr }
{$endc}

	PicturePtr = ^Picture;
	Picture = record
		picSize:				SInt16;
		picFrame:				Rect;
	end;

	PicPtr								= ^Picture;
	PicHandle							= ^PicPtr;
	MacPolygonPtr = ^MacPolygon;
	MacPolygon = record
		polySize:				SInt16;
		polyBBox:				Rect;
		polyPoints:				array [0..0] of Point;
	end;

	{
	   The type name "Polygon" has a name space collision on Win32.
	   Use MacPolygon to be cross-platfrom safe.
	}
{$ifc TARGET_OS_MAC}
	Polygon								= MacPolygon;
	PolygonPtr 							= ^Polygon;
{$endc}  {TARGET_OS_MAC}

	PolyPtr								= ^MacPolygon;
	PolyHandle							= ^PolyPtr;
{$ifc TYPED_FUNCTION_POINTERS}
	QDTextProcPtr = procedure(byteCount: SInt16; textBuf: UnivPtr; numer: Point; denom: Point);
{$elsec}
	QDTextProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	QDLineProcPtr = procedure(newPt: Point);
{$elsec}
	QDLineProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	QDRectProcPtr = procedure(verb: GrafVerb; const (*var*) r: Rect);
{$elsec}
	QDRectProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	QDRRectProcPtr = procedure(verb: GrafVerb; const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16);
{$elsec}
	QDRRectProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	QDOvalProcPtr = procedure(verb: GrafVerb; const (*var*) r: Rect);
{$elsec}
	QDOvalProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	QDArcProcPtr = procedure(verb: GrafVerb; const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16);
{$elsec}
	QDArcProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	QDPolyProcPtr = procedure(verb: GrafVerb; poly: PolyHandle);
{$elsec}
	QDPolyProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	QDRgnProcPtr = procedure(verb: GrafVerb; rgn: RgnHandle);
{$elsec}
	QDRgnProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	QDBitsProcPtr = procedure(const (*var*) srcBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle);
=======
{$ifc not defined __x86_64__ and defined CPUX86_64}
	{$setc __x86_64__ := 1}
>>>>>>> origin/fixes_2.4
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}

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
uses MacTypes,QDCMCommon,QuickdrawTypes,ColorSyncDeprecated,CGDirectDisplay,Components,MixedMode,QuickdrawText,CGContext;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{*********************************************************************************
// NOTE: Quickdraw has been deprecated for deployment targets 10.4 and later.
// The replacement API is Quartz (CoreGraphics).
// Because of the fundamental differences in the imaging models and design
// goals between Quickdraw and Quartz, there is no direct correspondence
// possible (or even desirable) between Quickdraw and Quartz APIs and concepts.
// For certain purposes, some Quickdraw functions may even still be needed during
// a transition period; nevertheless, they have all been tagged as deprecated
// to express the overriding goal of eliminating all Quickdraw usage in the future.
 |********************************************************************************}

<<<<<<< HEAD
{$ifc CALL_NOT_IN_CARBON}
{
 *  InitGraf()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure InitGraf(globalPtr: UnivPtr); external name '_InitGraf';
{
 *  OpenPort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure OpenPort(port: GrafPtr); external name '_OpenPort';
{
 *  InitPort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure InitPort(port: GrafPtr); external name '_InitPort';
{
 *  ClosePort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure ClosePort(port: GrafPtr); external name '_ClosePort';
{$endc}  {CALL_NOT_IN_CARBON}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
    General comments about thread-safety of Quickdraw
    -------------------------------------------------
    
    The original design and implementation principles of Quickdraw seriously
    conflict with the goal of making Quickdraw thread-safe. Many Quickdraw
    functions rely on globals instead of explicit parameters. Even though the 
    current port (and the current GDevice) are being maintained per-thread,
    a simple call like "MoveTo(x, y)" just doesn't make sense with preemptive threads,
    if two different threads use the same port. Also, as soon as a client replaces
    bottleneck procedures (SetStdCProcs), thread-safety is compromised.
    That's why we maintain by default the "Not thread safe" attribute in Quickdraw APIs,
    even though they may appear to be thread-safe.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Break a region up into rectangles.}

const
	kQDRegionToRectsMsgInit = 1;
	kQDRegionToRectsMsgParse = 2;
	kQDRegionToRectsMsgTerminate = 3;

const
	kQDParseRegionFromTop = 1 shl 0;
	kQDParseRegionFromBottom = 1 shl 1;
	kQDParseRegionFromLeft = 1 shl 2;
	kQDParseRegionFromRight = 1 shl 3;
	kQDParseRegionFromTopLeft = kQDParseRegionFromTop or kQDParseRegionFromLeft;
	kQDParseRegionFromBottomRight = kQDParseRegionFromBottom or kQDParseRegionFromRight;

type
	QDRegionParseDirection = SInt32;
	RegionToRectsProcPtr = function( message: UInt16; rgn: RgnHandle; const (*var*) rect_: Rect; refCon: UnivPtr ): OSStatus;
	RegionToRectsUPP = RegionToRectsProcPtr;
{
 *  NewRegionToRectsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewRegionToRectsUPP( userRoutine: RegionToRectsProcPtr ): RegionToRectsUPP; external name '_NewRegionToRectsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
=======
>>>>>>> origin/fixes_2_2
{
   These are Carbon only routines. They do nothing at all on
   Mac OS 8, but work flawlessly on Mac OS X.
}

{
 *  LockPortBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4
function LockPortBits(port: GrafPtr): OSErr; external name '_LockPortBits';

{
 *  UnlockPortBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4
function UnlockPortBits(port: GrafPtr): OSErr; external name '_UnlockPortBits';

{  Break a region up into rectangles. }


const
	kQDParseRegionFromTop		= $01;
	kQDParseRegionFromBottom	= $02;
	kQDParseRegionFromLeft		= $04;
	kQDParseRegionFromRight		= $08;
	kQDParseRegionFromTopLeft	= $05;
	kQDParseRegionFromBottomRight = $0A;


type
	QDRegionParseDirection				= SInt32;
=======
{ Break a region up into rectangles.}
>>>>>>> origin/fixes_2.4

const
	kQDRegionToRectsMsgInit = 1;
	kQDRegionToRectsMsgParse = 2;
	kQDRegionToRectsMsgTerminate = 3;

const
<<<<<<< HEAD
	uppRegionToRectsProcInfo = $00003FB1;
	{
	 *  NewRegionToRectsUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
	 	}
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function NewRegionToRectsUPP(userRoutine: RegionToRectsProcPtr): RegionToRectsUPP; external name '_NewRegionToRectsUPP'; { old name was NewRegionToRectsProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQDParseRegionFromTop = 1 shl 0;
	kQDParseRegionFromBottom = 1 shl 1;
	kQDParseRegionFromLeft = 1 shl 2;
	kQDParseRegionFromRight = 1 shl 3;
	kQDParseRegionFromTopLeft = kQDParseRegionFromTop or kQDParseRegionFromLeft;
	kQDParseRegionFromBottomRight = kQDParseRegionFromBottom or kQDParseRegionFromRight;
>>>>>>> origin/fixes_2.4

type
	QDRegionParseDirection = SInt32;
	RegionToRectsProcPtr = function( message: UInt16; rgn: RgnHandle; const (*var*) rect_: Rect; refCon: UnivPtr ): OSStatus;
	RegionToRectsUPP = RegionToRectsProcPtr;
{
 *  NewRegionToRectsUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeRegionToRectsUPP( userUPP: RegionToRectsUPP ); external name '_DisposeRegionToRectsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DisposeRegionToRectsUPP(userUPP: RegionToRectsUPP); external name '_DisposeRegionToRectsUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewRegionToRectsUPP( userRoutine: RegionToRectsProcPtr ): RegionToRectsUPP; external name '_NewRegionToRectsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeRegionToRectsUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeRegionToRectsUPP( message: UInt16; rgn: RgnHandle; const (*var*) rect_: Rect; refCon: UnivPtr; userUPP: RegionToRectsUPP ): OSStatus; external name '_InvokeRegionToRectsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function InvokeRegionToRectsUPP(message: UInt16; rgn: RgnHandle; const (*var*) rect_: Rect; refCon: UnivPtr; userRoutine: RegionToRectsUPP): OSStatus; external name '_InvokeRegionToRectsUPP'; { old name was CallRegionToRectsProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeRegionToRectsUPP( userUPP: RegionToRectsUPP ); external name '_DisposeRegionToRectsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeRegionToRectsUPP()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   not available
 }
function QDRegionToRects( rgn: RgnHandle; dir: QDRegionParseDirection; proc: RegionToRectsUPP; userData: UnivPtr ): OSStatus; external name '_QDRegionToRects';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  LockPortBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function LockPortBits( port: GrafPtr ): OSErr; external name '_LockPortBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  UnlockPortBits()   *** DEPRECATED ***
 *  
=======
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeRegionToRectsUPP( message: UInt16; rgn: RgnHandle; const (*var*) rect_: Rect; refCon: UnivPtr; userUPP: RegionToRectsUPP ): OSStatus; external name '_InvokeRegionToRectsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  QDRegionToRects()
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function UnlockPortBits( port: GrafPtr ): OSErr; external name '_UnlockPortBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$ifc TARGET_OS_WIN32}
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 }
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function QDRegionToRects(rgn: RgnHandle; dir: QDRegionParseDirection; proc: RegionToRectsUPP; userData: UnivPtr): OSStatus; external name '_QDRegionToRects';

{$ifc NOT TARGET_OS_MAC}
{$ifc CALL_NOT_IN_CARBON}
{
 *  UpdatePort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function UpdatePort(port: GrafPtr): OSErr; external name '_UpdatePort';

{
 *  GetPortNativeWindow()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function GetPortNativeWindow(macPort: GrafPtr): Ptr; external name '_GetPortNativeWindow';

{
 *  GetNativeWindowPort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function GetNativeWindowPort(nativeWindow: UnivPtr): GrafPtr; external name '_GetNativeWindowPort';
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDRegionToRects( rgn: RgnHandle; dir: QDRegionParseDirection; proc: RegionToRectsUPP; userData: UnivPtr ): OSStatus; external name '_QDRegionToRects';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{$ifc not TARGET_CPU_64}
{
 *  LockPortBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function LockPortBits( port: GrafPtr ): OSErr; external name '_LockPortBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  UnlockPortBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function UnlockPortBits( port: GrafPtr ): OSErr; external name '_UnlockPortBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$ifc TARGET_OS_WIN32}
<<<<<<< HEAD
{$ifc CALL_NOT_IN_CARBON}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  GetPortHWND()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetPortHWND(port: GrafPtr): Ptr; external name '_GetPortHWND';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  GetHWNDPort()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


// #define GetPortHWND(port)  (HWND)GetPortNativeWindow(port)
// #define GetHWNDPort(theHWND) GetNativeWindowPort(theHWND)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

// #define GetPortHWND(port)  (HWND)GetPortNativeWindow(port)
// #define GetHWNDPort(theHWND) GetNativeWindowPort(theHWND)
>>>>>>> origin/fixes_2.4
{
 *  GetPortHDC()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetPortHDC(port: GrafPtr): Ptr; external name '_GetPortHDC';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  GetPortHBITMAP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetPortHBITMAP(port: GrafPtr): Ptr; external name '_GetPortHBITMAP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  GetPortHPALETTE()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetPortHPALETTE(port: GrafPtr): Ptr; external name '_GetPortHPALETTE';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  GetPortHFONT()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetPortHFONT(port: GrafPtr): Ptr; external name '_GetPortHFONT';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  GetDIBFromPICT()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetDIBFromPICT(hPict: PicHandle): Ptr; external name '_GetDIBFromPICT';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  GetPICTFromDIB()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$endc} {TARGET_OS_WIN32}

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_WIN32}

{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  [Mac]SetPort()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacSetPort( port: GrafPtr ); external name '_SetPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure SetPort( port: GrafPtr ); external name '_SetPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetPort(port: GrafPtr); external name '_SetPort';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacSetPort( port: GrafPtr ); external name '_SetPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure SetPort( port: GrafPtr ); external name '_SetPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetPort()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetPort( var port: GrafPtr ); external name '_GetPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDSwapPort()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetPort( var port: GrafPtr ); external name '_GetPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  QDSwapPort()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDSwapPort()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Combines a GetPort(&savePort); SetPort(newPort) sequence.
 *  
 *  Discussion:
 *    On X, the GetPort/SetPort calls act on per-thread globals, and
 *    cost more processor cycles than in the past, where they were
 *    simple memory accessors. To optimize, use the QDSwapPort call
 *    which combines both, and returns a Boolean indicating if the port
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    actually did change. Typical usage: Boolean portChanged =
 *    QDSwapPort(newPort, &savePort); (... some drawing into newPort
 *    ...) if (portChanged) QDSwapPort(savePort, NULL);
=======
 *    actually did change. Typical usage: portChanged =
 *    QDSwapPort(newPort, &savePort); // some drawing into newPort if
 *    (portChanged) QDSwapPort(savePort, NULL);
>>>>>>> graemeg/fixes_2_2
=======
 *    actually did change. Typical usage: portChanged =
 *    QDSwapPort(newPort, &savePort); // some drawing into newPort if
 *    (portChanged) QDSwapPort(savePort, NULL);
>>>>>>> origin/fixes_2_2
=======
 *    actually did change. Typical usage: Boolean portChanged =
 *    QDSwapPort(newPort, &savePort); (... some drawing into newPort
 *    ...) if (portChanged) QDSwapPort(savePort, NULL);
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inNewPort:
 *      The new port to be set.
 *    
 *    outOldPort:
 *      Receives the previous port. Can be NULL.
 *  
 *  Result:
 *    A Boolean indicating whether the port was changed, i.e.
 *    (inNewPort != *outOldPort)
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function QDSwapPort( inNewPort: CGrafPtr; outOldPort: CGrafPtrPtr ): Boolean; external name '_QDSwapPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
//AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER;
function QDSwapPort(inNewPort: CGrafPtr; outOldPort: CGrafPtrPtr): boolean; external name '_QDSwapPort';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function QDSwapPort( inNewPort: CGrafPtr; outOldPort: CGrafPtrPtr ): Boolean; external name '_QDSwapPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GrafDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GrafDevice( device: SInt16 ); external name '_GrafDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GrafDevice(device: SInt16); external name '_GrafDevice';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GrafDevice( device: SInt16 ); external name '_GrafDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetPortBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortBits( const (*var*) bm: BitMap ); external name '_SetPortBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetPortBits(const (*var*) bm: BitMap); external name '_SetPortBits';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetPortBits( const (*var*) bm: BitMap ); external name '_SetPortBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PortSize()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PortSize( width: SInt16; height: SInt16 ); external name '_PortSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PortSize(width: SInt16; height: SInt16); external name '_PortSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PortSize( width: SInt16; height: SInt16 ); external name '_PortSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  MovePortTo()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MovePortTo( leftGlobal: SInt16; topGlobal: SInt16 ); external name '_MovePortTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure MovePortTo(leftGlobal: SInt16; topGlobal: SInt16); external name '_MovePortTo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MovePortTo( leftGlobal: SInt16; topGlobal: SInt16 ); external name '_MovePortTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetOrigin()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetOrigin( h: SInt16; v: SInt16 ); external name '_SetOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetOrigin(h: SInt16; v: SInt16); external name '_SetOrigin';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetOrigin( h: SInt16; v: SInt16 ); external name '_SetOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetClip()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetClip( rgn: RgnHandle ); external name '_SetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetClip(rgn: RgnHandle); external name '_SetClip';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetClip( rgn: RgnHandle ); external name '_SetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetClip()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetClip( rgn: RgnHandle ); external name '_GetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetClip(rgn: RgnHandle); external name '_GetClip';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GetClip( rgn: RgnHandle ); external name '_GetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ClipRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ClipRect( const (*var*) r: Rect ); external name '_ClipRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure ClipRect(const (*var*) r: Rect); external name '_ClipRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure ClipRect( const (*var*) r: Rect ); external name '_ClipRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  BackPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure BackPat( const (*var*) pat: Pattern ); external name '_BackPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  InitCursor()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure BackPat( const (*var*) pat: Pattern ); external name '_BackPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  InitCursor()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  InitCursor()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InitCursor; external name '_InitCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ NOTE: InitCursor used to be called to initialize Quickdraw. This is not necessary
on Mac OS X. Use SetThemeCursor (in Appearance.h) to set the cursor to one of the predefined
system cursors.
}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InitCursor; external name '_InitCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ NOTE: InitCursor used to be called to initialize Quickdraw. This is not necessary
on Mac OS X. Use SetThemeCursor (in Appearance.h) to set the cursor to one of the predefined
system cursors.
}
>>>>>>> origin/fixes_2.4
{
 *  [Mac]SetCursor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacSetCursor( const (*var*) crsr: Cursor ); external name '_SetCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure SetCursor( const (*var*) crsr: Cursor ); external name '_SetCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  HideCursor()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacSetCursor( const (*var*) crsr: Cursor ); external name '_SetCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure SetCursor( const (*var*) crsr: Cursor ); external name '_SetCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  HideCursor()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  HideCursor()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure HideCursor; external name '_HideCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]ShowCursor()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure HideCursor; external name '_HideCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]ShowCursor()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]ShowCursor()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacShowCursor__NAMED_ShowCursor; external name '_MacShowCursor__NAMED_ShowCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure ShowCursor; external name '_ShowCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ObscureCursor()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacShowCursor__NAMED_ShowCursor; external name '_MacShowCursor__NAMED_ShowCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure ShowCursor; external name '_ShowCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  ObscureCursor()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ObscureCursor()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ObscureCursor; external name '_ObscureCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ObscureCursor; external name '_ObscureCursor';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  HidePen()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure HidePen; external name '_HidePen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure HidePen; external name '_HidePen';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ShowPen()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ShowPen; external name '_ShowPen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ShowPen; external name '_ShowPen';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetPen()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetPen( var pt: Point ); external name '_GetPen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetPen(var pt: Point); external name '_GetPen';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GetPen( var pt: Point ); external name '_GetPen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetPenState()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetPenState( var pnState: PenState ); external name '_GetPenState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetPenState(var pnState: PenState); external name '_GetPenState';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GetPenState( var pnState: PenState ); external name '_GetPenState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetPenState()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPenState( const (*var*) pnState: PenState ); external name '_SetPenState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetPenState(const (*var*) pnState: PenState); external name '_SetPenState';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetPenState( const (*var*) pnState: PenState ); external name '_SetPenState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PenSize()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PenSize( width: SInt16; height: SInt16 ); external name '_PenSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PenSize(width: SInt16; height: SInt16); external name '_PenSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PenSize( width: SInt16; height: SInt16 ); external name '_PenSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PenMode()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PenMode( mode: SInt16 ); external name '_PenMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PenMode(mode: SInt16); external name '_PenMode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PenMode( mode: SInt16 ); external name '_PenMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PenPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PenPat( const (*var*) pat: Pattern ); external name '_PenPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PenPat(const (*var*) pat: Pattern); external name '_PenPat';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PenPat( const (*var*) pat: Pattern ); external name '_PenPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PenNormal()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PenNormal; external name '_PenNormal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PenNormal; external name '_PenNormal';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  MoveTo()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MoveTo( h: SInt16; v: SInt16 ); external name '_MoveTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure MoveTo(h: SInt16; v: SInt16); external name '_MoveTo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MoveTo( h: SInt16; v: SInt16 ); external name '_MoveTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  Move()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure Move( dh: SInt16; dv: SInt16 ); external name '_Move';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure Move(dh: SInt16; dv: SInt16); external name '_Move';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure Move( dh: SInt16; dv: SInt16 ); external name '_Move';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]LineTo()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacLineTo__NAMED_LineTo( h: SInt16; v: SInt16 ); external name '_MacLineTo__NAMED_LineTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure LineTo( h: SInt16; v: SInt16 ); external name '_LineTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure LineTo(h: SInt16; v: SInt16); external name '_LineTo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacLineTo__NAMED_LineTo( h: SInt16; v: SInt16 ); external name '_MacLineTo__NAMED_LineTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure LineTo( h: SInt16; v: SInt16 ); external name '_LineTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  Line()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure Line( dh: SInt16; dv: SInt16 ); external name '_Line';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure Line(dh: SInt16; dv: SInt16); external name '_Line';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure Line( dh: SInt16; dv: SInt16 ); external name '_Line';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ForeColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ForeColor( color: SIGNEDLONG ); external name '_ForeColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure ForeColor(color: SInt32); external name '_ForeColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure ForeColor( color: SIGNEDLONG ); external name '_ForeColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  BackColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure BackColor( color: SIGNEDLONG ); external name '_BackColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure BackColor(color: SInt32); external name '_BackColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure BackColor( color: SIGNEDLONG ); external name '_BackColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ColorBit()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ColorBit( whichBit: SInt16 ); external name '_ColorBit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  [Mac]SetRect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ColorBit( whichBit: SInt16 ); external name '_ColorBit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  [Mac]SetRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]SetRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacSetRect( var r: Rect; left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 ); external name '_SetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure SetRect( var r: Rect; left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 ); external name '_SetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]OffsetRect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacSetRect( var r: Rect; left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 ); external name '_SetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure SetRect( var r: Rect; left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 ); external name '_SetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]OffsetRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]OffsetRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacOffsetRect( var r: Rect; dh: SInt16; dv: SInt16 ); external name '_OffsetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure OffsetRect( var r: Rect; dh: SInt16; dv: SInt16 ); external name '_OffsetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]InsetRect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacOffsetRect( var r: Rect; dh: SInt16; dv: SInt16 ); external name '_OffsetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure OffsetRect( var r: Rect; dh: SInt16; dv: SInt16 ); external name '_OffsetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]InsetRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]InsetRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacInsetRect( var r: Rect; dh: SInt16; dv: SInt16 ); external name '_InsetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure InsetRect( var r: Rect; dh: SInt16; dv: SInt16 ); external name '_InsetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SectRect()
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
procedure MacInsetRect( var r: Rect; dh: SInt16; dv: SInt16 ); external name '_InsetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure InsetRect( var r: Rect; dh: SInt16; dv: SInt16 ); external name '_InsetRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SectRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SectRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SectRect( const (*var*) src1: Rect; const (*var*) src2: Rect; var dstRect: Rect ): Boolean; external name '_SectRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]UnionRect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SectRect( const (*var*) src1: Rect; const (*var*) src2: Rect; var dstRect: Rect ): Boolean; external name '_SectRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]UnionRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]UnionRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacUnionRect( const (*var*) src1: Rect; const (*var*) src2: Rect; var dstRect: Rect ); external name '_UnionRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure UnionRect( const (*var*) src1: Rect; const (*var*) src2: Rect; var dstRect: Rect ); external name '_UnionRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]EqualRect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacUnionRect( const (*var*) src1: Rect; const (*var*) src2: Rect; var dstRect: Rect ); external name '_UnionRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure UnionRect( const (*var*) src1: Rect; const (*var*) src2: Rect; var dstRect: Rect ); external name '_UnionRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]EqualRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]EqualRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacEqualRect( const (*var*) rect1: Rect; const (*var*) rect2: Rect ): Boolean; external name '_EqualRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function EqualRect( const (*var*) rect1: Rect; const (*var*) rect2: Rect ): Boolean; external name '_EqualRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EmptyRect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacEqualRect( const (*var*) rect1: Rect; const (*var*) rect2: Rect ): Boolean; external name '_EqualRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function EqualRect( const (*var*) rect1: Rect; const (*var*) rect2: Rect ): Boolean; external name '_EqualRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  EmptyRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  EmptyRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function EmptyRect( const (*var*) r: Rect ): Boolean; external name '_EmptyRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function EmptyRect( const (*var*) r: Rect ): Boolean; external name '_EmptyRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  [Mac]FrameRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacFrameRect( const (*var*) r: Rect ); external name '_FrameRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure FrameRect( const (*var*) r: Rect ); external name '_FrameRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FrameRect(const (*var*) r: Rect); external name '_FrameRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacFrameRect( const (*var*) r: Rect ); external name '_FrameRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure FrameRect( const (*var*) r: Rect ); external name '_FrameRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PaintRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PaintRect( const (*var*) r: Rect ); external name '_PaintRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PaintRect(const (*var*) r: Rect); external name '_PaintRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PaintRect( const (*var*) r: Rect ); external name '_PaintRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  EraseRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EraseRect( const (*var*) r: Rect ); external name '_EraseRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure EraseRect(const (*var*) r: Rect); external name '_EraseRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure EraseRect( const (*var*) r: Rect ); external name '_EraseRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]InvertRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacInvertRect( const (*var*) r: Rect ); external name '_InvertRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure InvertRect( const (*var*) r: Rect ); external name '_InvertRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure InvertRect(const (*var*) r: Rect); external name '_InvertRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacInvertRect( const (*var*) r: Rect ); external name '_InvertRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure InvertRect( const (*var*) r: Rect ); external name '_InvertRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]FillRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacFillRect( const (*var*) r: Rect; const (*var*) pat: Pattern ); external name '_FillRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure FillRect( const (*var*) r: Rect; const (*var*) pat: Pattern ); external name '_FillRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillRect(const (*var*) r: Rect; const (*var*) pat: Pattern); external name '_FillRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacFillRect( const (*var*) r: Rect; const (*var*) pat: Pattern ); external name '_FillRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure FillRect( const (*var*) r: Rect; const (*var*) pat: Pattern ); external name '_FillRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FrameOval()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FrameOval( const (*var*) r: Rect ); external name '_FrameOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FrameOval(const (*var*) r: Rect); external name '_FrameOval';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FrameOval( const (*var*) r: Rect ); external name '_FrameOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PaintOval()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PaintOval( const (*var*) r: Rect ); external name '_PaintOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PaintOval(const (*var*) r: Rect); external name '_PaintOval';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PaintOval( const (*var*) r: Rect ); external name '_PaintOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  EraseOval()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EraseOval( const (*var*) r: Rect ); external name '_EraseOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure EraseOval(const (*var*) r: Rect); external name '_EraseOval';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure EraseOval( const (*var*) r: Rect ); external name '_EraseOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  InvertOval()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvertOval( const (*var*) r: Rect ); external name '_InvertOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure InvertOval(const (*var*) r: Rect); external name '_InvertOval';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvertOval( const (*var*) r: Rect ); external name '_InvertOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillOval()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillOval( const (*var*) r: Rect; const (*var*) pat: Pattern ); external name '_FillOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillOval(const (*var*) r: Rect; const (*var*) pat: Pattern); external name '_FillOval';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FillOval( const (*var*) r: Rect; const (*var*) pat: Pattern ); external name '_FillOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FrameRoundRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FrameRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_FrameRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FrameRoundRect(const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16); external name '_FrameRoundRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FrameRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_FrameRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PaintRoundRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PaintRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_PaintRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PaintRoundRect(const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16); external name '_PaintRoundRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PaintRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_PaintRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  EraseRoundRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EraseRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_EraseRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure EraseRoundRect(const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16); external name '_EraseRoundRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure EraseRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_EraseRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  InvertRoundRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvertRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_InvertRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure InvertRoundRect(const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16); external name '_InvertRoundRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvertRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_InvertRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillRoundRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16; const (*var*) pat: Pattern ); external name '_FillRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillRoundRect(const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16; const (*var*) pat: Pattern); external name '_FillRoundRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FillRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16; const (*var*) pat: Pattern ); external name '_FillRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FrameArc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FrameArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_FrameArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FrameArc(const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16); external name '_FrameArc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FrameArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_FrameArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PaintArc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PaintArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_PaintArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PaintArc(const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16); external name '_PaintArc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PaintArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_PaintArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  EraseArc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EraseArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_EraseArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure EraseArc(const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16); external name '_EraseArc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure EraseArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_EraseArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  InvertArc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvertArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_InvertArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure InvertArc(const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16); external name '_InvertArc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvertArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_InvertArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillArc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16; const (*var*) pat: Pattern ); external name '_FillArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16; const (*var*) pat: Pattern ); external name '_FillArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  NewRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  NewRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewRgn: RgnHandle; external name '_NewRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewRgn: RgnHandle; external name '_NewRgn';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  OpenRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure OpenRgn; external name '_OpenRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure OpenRgn; external name '_OpenRgn';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CloseRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CloseRgn( dstRgn: RgnHandle ); external name '_CloseRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure CloseRgn(dstRgn: RgnHandle); external name '_CloseRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CloseRgn( dstRgn: RgnHandle ); external name '_CloseRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  BitMapToRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function BitMapToRegion( region: RgnHandle; const (*var*) bMap: BitMap ): OSErr; external name '_BitMapToRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function BitMapToRegion(region: RgnHandle; const (*var*) bMap: BitMap): OSErr; external name '_BitMapToRegion';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function BitMapToRegion( region: RgnHandle; const (*var*) bMap: BitMap ): OSErr; external name '_BitMapToRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  RgnToHandle()
 *  
 *  Summary:
 *    Allows to "flatten" the opaque region data, for persistent
 *    storage. HandleToRgn is the reciprocal call.
 *  
 *  Discussion:
 *    These calls were introduced to facilitate Carbonization of
 *    applications, that relied on the undocumented format of region
 *    data. Since the internal (opaque) region format changed in 10.3,
 *    the purpose of these functions became questionable at best. They
 *    should be considered deprecated. Applications that need to
 *    preserve region data within their documents should convert the
 *    regions to a sequence of rectangles, using QDRegionToRects. The
 *    original region can then be rebuilt using RectRgn() and
 *    UnionRgn() calls.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    region:
 *      The input RgnHandle
 *    
 *    flattenedRgnDataHdl:
 *      A valid Handle that gets resized and filled with the region
 *      data.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure RgnToHandle( region: RgnHandle; flattenedRgnDataHdl: Handle ); external name '_RgnToHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  HandleToRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure RgnToHandle( region: RgnHandle; flattenedRgnDataHdl: Handle ); external name '_RgnToHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
<<<<<<< HEAD
 *  HandleToRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  HandleToRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure HandleToRgn( oldRegion: Handle; region: RgnHandle ); external name '_HandleToRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure HandleToRgn( oldRegion: Handle; region: RgnHandle ); external name '_HandleToRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  DisposeRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DisposeRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeRgn( rgn: RgnHandle ); external name '_DisposeRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]CopyRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeRgn( rgn: RgnHandle ); external name '_DisposeRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]CopyRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]CopyRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacCopyRgn( srcRgn: RgnHandle; dstRgn: RgnHandle ); external name '_CopyRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure CopyRgn( srcRgn: RgnHandle; dstRgn: RgnHandle ); external name '_CopyRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetEmptyRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacCopyRgn( srcRgn: RgnHandle; dstRgn: RgnHandle ); external name '_CopyRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure CopyRgn( srcRgn: RgnHandle; dstRgn: RgnHandle ); external name '_CopyRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SetEmptyRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SetEmptyRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetEmptyRgn( rgn: RgnHandle ); external name '_SetEmptyRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]SetRectRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetEmptyRgn( rgn: RgnHandle ); external name '_SetEmptyRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]SetRectRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]SetRectRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacSetRectRgn( rgn: RgnHandle; left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 ); external name '_SetRectRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure SetRectRgn( rgn: RgnHandle; left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 ); external name '_SetRectRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RectRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacSetRectRgn( rgn: RgnHandle; left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 ); external name '_SetRectRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure SetRectRgn( rgn: RgnHandle; left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 ); external name '_SetRectRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  RectRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  RectRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure RectRgn( rgn: RgnHandle; const (*var*) r: Rect ); external name '_RectRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]OffsetRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure RectRgn( rgn: RgnHandle; const (*var*) r: Rect ); external name '_RectRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]OffsetRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]OffsetRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacOffsetRgn( rgn: RgnHandle; dh: SInt16; dv: SInt16 ); external name '_OffsetRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure OffsetRgn( rgn: RgnHandle; dh: SInt16; dv: SInt16 ); external name '_OffsetRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InsetRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacOffsetRgn( rgn: RgnHandle; dh: SInt16; dv: SInt16 ); external name '_OffsetRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure OffsetRgn( rgn: RgnHandle; dh: SInt16; dv: SInt16 ); external name '_OffsetRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  InsetRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  InsetRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InsetRgn( rgn: RgnHandle; dh: SInt16; dv: SInt16 ); external name '_InsetRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SectRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InsetRgn( rgn: RgnHandle; dh: SInt16; dv: SInt16 ); external name '_InsetRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SectRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SectRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SectRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_SectRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]UnionRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SectRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_SectRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]UnionRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]UnionRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacUnionRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_UnionRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure UnionRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_UnionRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DiffRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacUnionRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_UnionRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure UnionRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_UnionRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  DiffRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DiffRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DiffRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_DiffRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]XorRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DiffRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_DiffRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]XorRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]XorRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacXorRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_XorRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure XorRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_XorRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RectInRgn()
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacXorRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_XorRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure XorRgn( srcRgnA: RgnHandle; srcRgnB: RgnHandle; dstRgn: RgnHandle ); external name '_XorRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  RectInRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  RectInRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function RectInRgn( const (*var*) r: Rect; rgn: RgnHandle ): Boolean; external name '_RectInRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]EqualRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function RectInRgn( const (*var*) r: Rect; rgn: RgnHandle ): Boolean; external name '_RectInRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]EqualRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]EqualRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacEqualRgn( rgnA: RgnHandle; rgnB: RgnHandle ): Boolean; external name '_EqualRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function EqualRgn( rgnA: RgnHandle; rgnB: RgnHandle ): Boolean; external name '_EqualRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EmptyRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacEqualRgn( rgnA: RgnHandle; rgnB: RgnHandle ): Boolean; external name '_EqualRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function EqualRgn( rgnA: RgnHandle; rgnB: RgnHandle ): Boolean; external name '_EqualRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  EmptyRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  EmptyRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function EmptyRgn( rgn: RgnHandle ): Boolean; external name '_EmptyRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function EmptyRgn(rgn: RgnHandle): boolean; external name '_EmptyRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function EmptyRgn( rgn: RgnHandle ): Boolean; external name '_EmptyRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]FrameRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacFrameRgn( rgn: RgnHandle ); external name '_FrameRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure FrameRgn( rgn: RgnHandle ); external name '_FrameRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FrameRgn(rgn: RgnHandle); external name '_FrameRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacFrameRgn( rgn: RgnHandle ); external name '_FrameRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure FrameRgn( rgn: RgnHandle ); external name '_FrameRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]PaintRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacPaintRgn( rgn: RgnHandle ); external name '_PaintRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure PaintRgn( rgn: RgnHandle ); external name '_PaintRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PaintRgn(rgn: RgnHandle); external name '_PaintRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacPaintRgn( rgn: RgnHandle ); external name '_PaintRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure PaintRgn( rgn: RgnHandle ); external name '_PaintRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  EraseRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EraseRgn( rgn: RgnHandle ); external name '_EraseRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure EraseRgn(rgn: RgnHandle); external name '_EraseRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure EraseRgn( rgn: RgnHandle ); external name '_EraseRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]InvertRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacInvertRgn( rgn: RgnHandle ); external name '_InvertRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure InvertRgn( rgn: RgnHandle ); external name '_InvertRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure InvertRgn(rgn: RgnHandle); external name '_InvertRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacInvertRgn( rgn: RgnHandle ); external name '_InvertRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure InvertRgn( rgn: RgnHandle ); external name '_InvertRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]FillRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MacFillRgn( rgn: RgnHandle; const (*var*) pat: Pattern ); external name '_FillRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure FillRgn( rgn: RgnHandle; const (*var*) pat: Pattern ); external name '_FillRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillRgn(rgn: RgnHandle; const (*var*) pat: Pattern); external name '_FillRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MacFillRgn( rgn: RgnHandle; const (*var*) pat: Pattern ); external name '_FillRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
procedure FillRgn( rgn: RgnHandle; const (*var*) pat: Pattern ); external name '_FillRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ScrollRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ScrollRect( const (*var*) r: Rect; dh: SInt16; dv: SInt16; updateRgn: RgnHandle ); external name '_ScrollRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure ScrollRect(const (*var*) r: Rect; dh: SInt16; dv: SInt16; updateRgn: RgnHandle); external name '_ScrollRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure ScrollRect( const (*var*) r: Rect; dh: SInt16; dv: SInt16; updateRgn: RgnHandle ); external name '_ScrollRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CopyBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CopyBits( const (*var*) srcBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle { can be NULL } ); external name '_CopyBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CopyBits( const (*var*) srcBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle { can be NULL } ); external name '_CopyBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  SeedFill()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SeedFill( srcPtr: {const} UnivPtr; dstPtr: UnivPtr; srcRow: SInt16; dstRow: SInt16; height: SInt16; words: SInt16; seedH: SInt16; seedV: SInt16 ); external name '_SeedFill';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SeedFill(srcPtr: UnivPtr; dstPtr: UnivPtr; srcRow: SInt16; dstRow: SInt16; height: SInt16; words: SInt16; seedH: SInt16; seedV: SInt16); external name '_SeedFill';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SeedFill( srcPtr: {const} UnivPtr; dstPtr: UnivPtr; srcRow: SInt16; dstRow: SInt16; height: SInt16; words: SInt16; seedH: SInt16; seedV: SInt16 ); external name '_SeedFill';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CalcMask()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CalcMask( srcPtr: {const} UnivPtr; dstPtr: UnivPtr; srcRow: SInt16; dstRow: SInt16; height: SInt16; words: SInt16 ); external name '_CalcMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure CalcMask(srcPtr: UnivPtr; dstPtr: UnivPtr; srcRow: SInt16; dstRow: SInt16; height: SInt16; words: SInt16); external name '_CalcMask';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CalcMask( srcPtr: {const} UnivPtr; dstPtr: UnivPtr; srcRow: SInt16; dstRow: SInt16; height: SInt16; words: SInt16 ); external name '_CalcMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CopyMask()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CopyMask( const (*var*) srcBits: BitMap; const (*var*) maskBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) maskRect: Rect; const (*var*) dstRect: Rect ); external name '_CopyMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CopyMask( const (*var*) srcBits: BitMap; const (*var*) maskBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) maskRect: Rect; const (*var*) dstRect: Rect ); external name '_CopyMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  OpenPicture()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OpenPicture( const (*var*) picFrame: Rect ): PicHandle; external name '_OpenPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function OpenPicture(const (*var*) picFrame: Rect): PicHandle; external name '_OpenPicture';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OpenPicture( const (*var*) picFrame: Rect ): PicHandle; external name '_OpenPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PicComment()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PicComment( kind: SInt16; dataSize: SInt16; dataHandle: Handle ); external name '_PicComment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PicComment(kind: SInt16; dataSize: SInt16; dataHandle: Handle); external name '_PicComment';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PicComment( kind: SInt16; dataSize: SInt16; dataHandle: Handle ); external name '_PicComment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ClosePicture()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ClosePicture; external name '_ClosePicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDGetPictureBounds()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ClosePicture; external name '_ClosePicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  QDGetPictureBounds()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDGetPictureBounds()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function QDGetPictureBounds( picH: PicHandle; var outRect: Rect ): RectPtr; external name '_QDGetPictureBounds'; (* attribute ignoreable *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function QDGetPictureBounds( picH: PicHandle; var outRect: Rect ): RectPtr; external name '_QDGetPictureBounds'; (* attribute ignoreable *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  DrawPicture()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DrawPicture( myPicture: PicHandle; const (*var*) dstRect: Rect ); external name '_DrawPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DrawPicture(myPicture: PicHandle; const (*var*) dstRect: Rect); external name '_DrawPicture';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DrawPicture( myPicture: PicHandle; const (*var*) dstRect: Rect ); external name '_DrawPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  KillPicture()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure KillPicture( myPicture: PicHandle ); external name '_KillPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure KillPicture(myPicture: PicHandle); external name '_KillPicture';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure KillPicture( myPicture: PicHandle ); external name '_KillPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  OpenPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OpenPoly: PolyHandle; external name '_OpenPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OpenPoly: PolyHandle; external name '_OpenPoly';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ClosePoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ClosePoly; external name '_ClosePoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ClosePoly; external name '_ClosePoly';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  KillPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure KillPoly( poly: PolyHandle ); external name '_KillPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure KillPoly(poly: PolyHandle); external name '_KillPoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure KillPoly( poly: PolyHandle ); external name '_KillPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  OffsetPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure OffsetPoly( poly: PolyHandle; dh: SInt16; dv: SInt16 ); external name '_OffsetPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure OffsetPoly(poly: PolyHandle; dh: SInt16; dv: SInt16); external name '_OffsetPoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure OffsetPoly( poly: PolyHandle; dh: SInt16; dv: SInt16 ); external name '_OffsetPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FramePoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FramePoly( poly: PolyHandle ); external name '_FramePoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FramePoly(poly: PolyHandle); external name '_FramePoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FramePoly( poly: PolyHandle ); external name '_FramePoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PaintPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PaintPoly( poly: PolyHandle ); external name '_PaintPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PaintPoly(poly: PolyHandle); external name '_PaintPoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PaintPoly( poly: PolyHandle ); external name '_PaintPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ErasePoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ErasePoly( poly: PolyHandle ); external name '_ErasePoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure ErasePoly(poly: PolyHandle); external name '_ErasePoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure ErasePoly( poly: PolyHandle ); external name '_ErasePoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  InvertPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvertPoly( poly: PolyHandle ); external name '_InvertPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure InvertPoly(poly: PolyHandle); external name '_InvertPoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvertPoly( poly: PolyHandle ); external name '_InvertPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillPoly( poly: PolyHandle; const (*var*) pat: Pattern ); external name '_FillPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  SetPt()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillPoly( poly: PolyHandle; const (*var*) pat: Pattern ); external name '_FillPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  SetPt()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SetPt()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPt( var pt: Point; h: SInt16; v: SInt16 ); external name '_SetPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPt( var pt: Point; h: SInt16; v: SInt16 ); external name '_SetPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  LocalToGlobal()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LocalToGlobal( var pt: Point ); external name '_LocalToGlobal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure LocalToGlobal(var pt: Point); external name '_LocalToGlobal';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure LocalToGlobal( var pt: Point ); external name '_LocalToGlobal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GlobalToLocal()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GlobalToLocal( var pt: Point ); external name '_GlobalToLocal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ For a replacement, read the man pages of rand, random, arc4random in <stdlib.h> }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GlobalToLocal( var pt: Point ); external name '_GlobalToLocal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ For a replacement, read the man pages of rand, random, arc4random in <stdlib.h> }
>>>>>>> origin/fixes_2.4
{
 *  Random()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function Random: SInt16; external name '_Random';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function Random: SInt16; external name '_Random';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StuffHex()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StuffHex( thingPtr: UnivPtr; const (*var*) s: Str255 ); external name '_StuffHex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StuffHex(thingPtr: UnivPtr; const (*var*) s: Str255); external name '_StuffHex';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StuffHex( thingPtr: UnivPtr; const (*var*) s: Str255 ); external name '_StuffHex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]GetPixel()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacGetPixel( h: SInt16; v: SInt16 ): Boolean; external name '_GetPixel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
function GetPixel( h: SInt16; v: SInt16 ): Boolean; external name '_GetPixel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  ScalePt()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacGetPixel( h: SInt16; v: SInt16 ): Boolean; external name '_GetPixel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
function GetPixel( h: SInt16; v: SInt16 ): Boolean; external name '_GetPixel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  ScalePt()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ScalePt()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ScalePt( var pt: Point; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_ScalePt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MapPt()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ScalePt( var pt: Point; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_ScalePt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  MapPt()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  MapPt()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MapPt( var pt: Point; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_MapPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MapRect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MapPt( var pt: Point; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_MapPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  MapRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  MapRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MapRect( var r: Rect; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_MapRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  MapRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MapRect( var r: Rect; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_MapRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  MapRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  MapRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MapRgn( rgn: RgnHandle; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_MapRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure MapRgn(rgn: RgnHandle; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect); external name '_MapRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MapRgn( rgn: RgnHandle; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_MapRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  MapPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MapPoly( poly: PolyHandle; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_MapPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure MapPoly(poly: PolyHandle; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect); external name '_MapPoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MapPoly( poly: PolyHandle; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect ); external name '_MapPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetStdProcs()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetStdProcs( var procs: QDProcs ); external name '_SetStdProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetStdProcs(var procs: QDProcs); external name '_SetStdProcs';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetStdProcs( var procs: QDProcs ); external name '_SetStdProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdRect( verb: GrafVerb; const (*var*) r: Rect ); external name '_StdRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdRect(verb: GrafVerb; const (*var*) r: Rect); external name '_StdRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdRect( verb: GrafVerb; const (*var*) r: Rect ); external name '_StdRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdRRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdRRect( verb: GrafVerb; const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_StdRRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdRRect(verb: GrafVerb; const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16); external name '_StdRRect';
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdRRect(verb: GrafVerb; const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16); external name '_StdRRect';
>>>>>>> origin/fixes_2_2
=======
procedure StdRRect( verb: GrafVerb; const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 ); external name '_StdRRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdOval()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdOval( verb: GrafVerb; const (*var*) r: Rect ); external name '_StdOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdOval(verb: GrafVerb; const (*var*) r: Rect); external name '_StdOval';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdOval( verb: GrafVerb; const (*var*) r: Rect ); external name '_StdOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdArc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdArc( verb: GrafVerb; const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_StdArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdArc(verb: GrafVerb; const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16); external name '_StdArc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdArc( verb: GrafVerb; const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 ); external name '_StdArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdPoly( verb: GrafVerb; poly: PolyHandle ); external name '_StdPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdPoly(verb: GrafVerb; poly: PolyHandle); external name '_StdPoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdPoly( verb: GrafVerb; poly: PolyHandle ); external name '_StdPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdRgn( verb: GrafVerb; rgn: RgnHandle ); external name '_StdRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdRgn(verb: GrafVerb; rgn: RgnHandle); external name '_StdRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdRgn( verb: GrafVerb; rgn: RgnHandle ); external name '_StdRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdBits( const (*var*) srcBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle ); external name '_StdBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdBits(const (*var*) srcBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle); external name '_StdBits';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdBits( const (*var*) srcBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle ); external name '_StdBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdComment()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdComment( kind: SInt16; dataSize: SInt16; dataHandle: Handle ); external name '_StdComment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdComment(kind: SInt16; dataSize: SInt16; dataHandle: Handle); external name '_StdComment';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdComment( kind: SInt16; dataSize: SInt16; dataHandle: Handle ); external name '_StdComment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdGetPic()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdGetPic( dataPtr: UnivPtr; byteCount: SInt16 ); external name '_StdGetPic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdGetPic(dataPtr: UnivPtr; byteCount: SInt16); external name '_StdGetPic';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdGetPic( dataPtr: UnivPtr; byteCount: SInt16 ); external name '_StdGetPic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdPutPic()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdPutPic( dataPtr: {const} UnivPtr; byteCount: SInt16 ); external name '_StdPutPic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure StdPutPic(dataPtr: UnivPtr; byteCount: SInt16); external name '_StdPutPic';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure StdPutPic( dataPtr: {const} UnivPtr; byteCount: SInt16 ); external name '_StdPutPic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  StdOpcode()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdOpcode( const (*var*) fromRect: Rect; const (*var*) toRect: Rect; opcode: UInt16; version: SInt16 ); external name '_StdOpcode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  AddPt()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdOpcode( const (*var*) fromRect: Rect; const (*var*) toRect: Rect; opcode: UInt16; version: SInt16 ); external name '_StdOpcode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  AddPt()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  AddPt()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure AddPt( src: Point; var dst: Point ); external name '_AddPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EqualPt()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure AddPt( src: Point; var dst: Point ); external name '_AddPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  EqualPt()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  EqualPt()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function EqualPt( pt1: Point; pt2: Point ): Boolean; external name '_EqualPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]PtInRect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function EqualPt( pt1: Point; pt2: Point ): Boolean; external name '_EqualPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  [Mac]PtInRect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  [Mac]PtInRect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacPtInRect( pt: Point; const (*var*) r: Rect ): Boolean; external name '_PtInRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function PtInRect( pt: Point; const (*var*) r: Rect ): Boolean; external name '_PtInRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  Pt2Rect()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacPtInRect( pt: Point; const (*var*) r: Rect ): Boolean; external name '_PtInRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function PtInRect( pt: Point; const (*var*) r: Rect ): Boolean; external name '_PtInRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  Pt2Rect()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Pt2Rect()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure Pt2Rect( pt1: Point; pt2: Point; var dstRect: Rect ); external name '_Pt2Rect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PtToAngle()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure Pt2Rect( pt1: Point; pt2: Point; var dstRect: Rect ); external name '_Pt2Rect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PtToAngle()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  PtToAngle()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PtToAngle( const (*var*) r: Rect; pt: Point; var angle: SInt16 ); external name '_PtToAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SubPt()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PtToAngle( const (*var*) r: Rect; pt: Point; var angle: SInt16 ); external name '_PtToAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SubPt()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SubPt()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SubPt( src: Point; var dst: Point ); external name '_SubPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  PtInRgn()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SubPt( src: Point; var dst: Point ); external name '_SubPt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  PtInRgn()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  PtInRgn()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PtInRgn( pt: Point; rgn: RgnHandle ): Boolean; external name '_PtInRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function PtInRgn(pt: Point; rgn: RgnHandle): boolean; external name '_PtInRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PtInRgn( pt: Point; rgn: RgnHandle ): Boolean; external name '_PtInRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  StdLine()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdLine( newPt: Point ); external name '_StdLine';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure StdLine( newPt: Point ); external name '_StdLine';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
{$ifc CALL_NOT_IN_CARBON}
{
 *  OpenCPort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure OpenCPort(port: CGrafPtr); external name '_OpenCPort';
{
 *  InitCPort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure InitCPort(port: CGrafPtr); external name '_InitCPort';
{
 *  CloseCPort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure CloseCPort(port: CGrafPtr); external name '_CloseCPort';
{$endc}  {CALL_NOT_IN_CARBON}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  NewPixMap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewPixMap: PixMapHandle; external name '_NewPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewPixMap: PixMapHandle; external name '_NewPixMap';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  DisposePixMap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposePixMap( pm: PixMapHandle ); external name '_DisposePixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DisposePixMap(pm: PixMapHandle); external name '_DisposePixMap';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposePixMap( pm: PixMapHandle ); external name '_DisposePixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CopyPixMap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CopyPixMap( srcPM: PixMapHandle; dstPM: PixMapHandle ); external name '_CopyPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure CopyPixMap(srcPM: PixMapHandle; dstPM: PixMapHandle); external name '_CopyPixMap';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CopyPixMap( srcPM: PixMapHandle; dstPM: PixMapHandle ); external name '_CopyPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  NewPixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewPixPat: PixPatHandle; external name '_NewPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewPixPat: PixPatHandle; external name '_NewPixPat';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  DisposePixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposePixPat( pp: PixPatHandle ); external name '_DisposePixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DisposePixPat(pp: PixPatHandle); external name '_DisposePixPat';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposePixPat( pp: PixPatHandle ); external name '_DisposePixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CopyPixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CopyPixPat( srcPP: PixPatHandle; dstPP: PixPatHandle ); external name '_CopyPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure CopyPixPat(srcPP: PixPatHandle; dstPP: PixPatHandle); external name '_CopyPixPat';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CopyPixPat( srcPP: PixPatHandle; dstPP: PixPatHandle ); external name '_CopyPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  PenPixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PenPixPat( pp: PixPatHandle ); external name '_PenPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PenPixPat(pp: PixPatHandle); external name '_PenPixPat';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PenPixPat( pp: PixPatHandle ); external name '_PenPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  BackPixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure BackPixPat( pp: PixPatHandle ); external name '_BackPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure BackPixPat(pp: PixPatHandle); external name '_BackPixPat';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure BackPixPat( pp: PixPatHandle ); external name '_BackPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetPixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetPixPat( patID: SInt16 ): PixPatHandle; external name '_GetPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetPixPat(patID: SInt16): PixPatHandle; external name '_GetPixPat';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetPixPat( patID: SInt16 ): PixPatHandle; external name '_GetPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  MakeRGBPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MakeRGBPat( pp: PixPatHandle; const (*var*) myColor: RGBColor ); external name '_MakeRGBPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure MakeRGBPat(pp: PixPatHandle; const (*var*) myColor: RGBColor); external name '_MakeRGBPat';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MakeRGBPat( pp: PixPatHandle; const (*var*) myColor: RGBColor ); external name '_MakeRGBPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillCRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillCRect( const (*var*) r: Rect; pp: PixPatHandle ); external name '_FillCRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillCRect(const (*var*) r: Rect; pp: PixPatHandle); external name '_FillCRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FillCRect( const (*var*) r: Rect; pp: PixPatHandle ); external name '_FillCRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillCOval()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillCOval( const (*var*) r: Rect; pp: PixPatHandle ); external name '_FillCOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillCOval(const (*var*) r: Rect; pp: PixPatHandle); external name '_FillCOval';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FillCOval( const (*var*) r: Rect; pp: PixPatHandle ); external name '_FillCOval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillCRoundRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillCRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16; pp: PixPatHandle ); external name '_FillCRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillCRoundRect(const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16; pp: PixPatHandle); external name '_FillCRoundRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FillCRoundRect( const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16; pp: PixPatHandle ); external name '_FillCRoundRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillCArc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillCArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16; pp: PixPatHandle ); external name '_FillCArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillCArc(const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16; pp: PixPatHandle); external name '_FillCArc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FillCArc( const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16; pp: PixPatHandle ); external name '_FillCArc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillCRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillCRgn( rgn: RgnHandle; pp: PixPatHandle ); external name '_FillCRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillCRgn(rgn: RgnHandle; pp: PixPatHandle); external name '_FillCRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FillCRgn( rgn: RgnHandle; pp: PixPatHandle ); external name '_FillCRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  FillCPoly()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure FillCPoly( poly: PolyHandle; pp: PixPatHandle ); external name '_FillCPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure FillCPoly(poly: PolyHandle; pp: PixPatHandle); external name '_FillCPoly';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure FillCPoly( poly: PolyHandle; pp: PixPatHandle ); external name '_FillCPoly';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  RGBForeColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure RGBForeColor( const (*var*) color: RGBColor ); external name '_RGBForeColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure RGBForeColor(const (*var*) color: RGBColor); external name '_RGBForeColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure RGBForeColor( const (*var*) color: RGBColor ); external name '_RGBForeColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  RGBBackColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure RGBBackColor( const (*var*) color: RGBColor ); external name '_RGBBackColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure RGBBackColor(const (*var*) color: RGBColor); external name '_RGBBackColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure RGBBackColor( const (*var*) color: RGBColor ); external name '_RGBBackColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetCPixel()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetCPixel( h: SInt16; v: SInt16; const (*var*) cPix: RGBColor ); external name '_SetCPixel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetCPixel(h: SInt16; v: SInt16; const (*var*) cPix: RGBColor); external name '_SetCPixel';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetCPixel( h: SInt16; v: SInt16; const (*var*) cPix: RGBColor ); external name '_SetCPixel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetPortPix()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortPix( pm: PixMapHandle ); external name '_SetPortPix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetPortPix(pm: PixMapHandle); external name '_SetPortPix';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetPortPix( pm: PixMapHandle ); external name '_SetPortPix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetCPixel()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetCPixel( h: SInt16; v: SInt16; var cPix: RGBColor ); external name '_GetCPixel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetCPixel(h: SInt16; v: SInt16; var cPix: RGBColor); external name '_GetCPixel';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GetCPixel( h: SInt16; v: SInt16; var cPix: RGBColor ); external name '_GetCPixel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetForeColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetForeColor( var color: RGBColor ); external name '_GetForeColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetForeColor(var color: RGBColor); external name '_GetForeColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GetForeColor( var color: RGBColor ); external name '_GetForeColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetBackColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetBackColor( var color: RGBColor ); external name '_GetBackColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetBackColor(var color: RGBColor); external name '_GetBackColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GetBackColor( var color: RGBColor ); external name '_GetBackColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SeedCFill()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SeedCFill( const (*var*) srcBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; seedH: SInt16; seedV: SInt16; matchProc: ColorSearchUPP; matchData: SIGNEDLONG ); external name '_SeedCFill';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SeedCFill(const (*var*) srcBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; seedH: SInt16; seedV: SInt16; matchProc: ColorSearchUPP; matchData: SInt32); external name '_SeedCFill';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SeedCFill( const (*var*) srcBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; seedH: SInt16; seedV: SInt16; matchProc: ColorSearchUPP; matchData: SIGNEDLONG ); external name '_SeedCFill';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CalcCMask()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CalcCMask( const (*var*) srcBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; const (*var*) seedRGB: RGBColor; matchProc: ColorSearchUPP; matchData: SIGNEDLONG ); external name '_CalcCMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure CalcCMask(const (*var*) srcBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; const (*var*) seedRGB: RGBColor; matchProc: ColorSearchUPP; matchData: SInt32); external name '_CalcCMask';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CalcCMask( const (*var*) srcBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; const (*var*) seedRGB: RGBColor; matchProc: ColorSearchUPP; matchData: SIGNEDLONG ); external name '_CalcCMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  OpenCPicture()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OpenCPicture( const (*var*) newHeader: OpenCPicParams ): PicHandle; external name '_OpenCPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function OpenCPicture(const (*var*) newHeader: OpenCPicParams): PicHandle; external name '_OpenCPicture';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OpenCPicture( const (*var*) newHeader: OpenCPicParams ): PicHandle; external name '_OpenCPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  OpColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure OpColor( const (*var*) color: RGBColor ); external name '_OpColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure OpColor(const (*var*) color: RGBColor); external name '_OpColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure OpColor( const (*var*) color: RGBColor ); external name '_OpColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  HiliteColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure HiliteColor( const (*var*) color: RGBColor ); external name '_HiliteColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure HiliteColor(const (*var*) color: RGBColor); external name '_HiliteColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure HiliteColor( const (*var*) color: RGBColor ); external name '_HiliteColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  DisposeCTable()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeCTable( cTable: CTabHandle ); external name '_DisposeCTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DisposeCTable(cTable: CTabHandle); external name '_DisposeCTable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeCTable( cTable: CTabHandle ); external name '_DisposeCTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetCTable()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetCTable( ctID: SInt16 ): CTabHandle; external name '_GetCTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetCTable(ctID: SInt16): CTabHandle; external name '_GetCTable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetCTable( ctID: SInt16 ): CTabHandle; external name '_GetCTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetCCursor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetCCursor( crsrID: SInt16 ): CCrsrHandle; external name '_GetCCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetCCursor(crsrID: SInt16): CCrsrHandle; external name '_GetCCursor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetCCursor( crsrID: SInt16 ): CCrsrHandle; external name '_GetCCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetCCursor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetCCursor( cCrsr: CCrsrHandle ); external name '_SetCCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetCCursor(cCrsr: CCrsrHandle); external name '_SetCCursor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetCCursor( cCrsr: CCrsrHandle ); external name '_SetCCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  AllocCursor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure AllocCursor; external name '_AllocCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure AllocCursor; external name '_AllocCursor';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  DisposeCCursor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeCCursor( cCrsr: CCrsrHandle ); external name '_DisposeCCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ GetCIcon(), PlotCIcon(), and DisposeCIcon() moved to Icons.h}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeCCursor( cCrsr: CCrsrHandle ); external name '_DisposeCCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
{  GetCIcon(), PlotCIcon(), and DisposeCIcon() moved to Icons.h }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ GetCIcon(), PlotCIcon(), and DisposeCIcon() moved to Icons.h}
>>>>>>> origin/fixes_2.4

{
 *  SetStdCProcs()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetStdCProcs( var procs: CQDProcs ); external name '_SetStdCProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetStdCProcs(var procs: CQDProcs); external name '_SetStdCProcs';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetStdCProcs( var procs: CQDProcs ); external name '_SetStdCProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetMaxDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMaxDevice( const (*var*) globalRect: Rect ): GDHandle; external name '_GetMaxDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetMaxDevice(const (*var*) globalRect: Rect): GDHandle; external name '_GetMaxDevice';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetMaxDevice( const (*var*) globalRect: Rect ): GDHandle; external name '_GetMaxDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetCTSeed()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetCTSeed: SIGNEDLONG; external name '_GetCTSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Replacements for the following (as far as still needed in a QD-free world)
 * can be found in CoreGraphics/CGDirectDisplay.h:
 * CGGetActiveDisplayList, CGMainDisplayID, CGDisplayBounds, etc. 
 }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetCTSeed: SIGNEDLONG; external name '_GetCTSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Replacements for the following (as far as still needed in a QD-free world)
 * can be found in CoreGraphics/CGDirectDisplay.h:
 * CGGetActiveDisplayList, CGMainDisplayID, CGDisplayBounds, etc. 
 }
>>>>>>> origin/fixes_2.4
{
 *  GetDeviceList()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetDeviceList: GDHandle; external name '_GetDeviceList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetDeviceList: GDHandle; external name '_GetDeviceList';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetMainDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMainDevice: GDHandle; external name '_GetMainDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMainDevice: GDHandle; external name '_GetMainDevice';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetNextDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetNextDevice( curDevice: GDHandle ): GDHandle; external name '_GetNextDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetNextDevice(curDevice: GDHandle): GDHandle; external name '_GetNextDevice';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetNextDevice( curDevice: GDHandle ): GDHandle; external name '_GetNextDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  TestDeviceAttribute()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function TestDeviceAttribute( gdh: GDHandle; attribute: SInt16 ): Boolean; external name '_TestDeviceAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function TestDeviceAttribute(gdh: GDHandle; attribute: SInt16): boolean; external name '_TestDeviceAttribute';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TestDeviceAttribute( gdh: GDHandle; attribute: SInt16 ): Boolean; external name '_TestDeviceAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetDeviceAttribute()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetDeviceAttribute( gdh: GDHandle; attribute: SInt16; value: Boolean ); external name '_SetDeviceAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetDeviceAttribute(gdh: GDHandle; attribute: SInt16; value: boolean); external name '_SetDeviceAttribute';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetDeviceAttribute( gdh: GDHandle; attribute: SInt16; value: Boolean ); external name '_SetDeviceAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  InitGDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InitGDevice( qdRefNum: SInt16; mode: SIGNEDLONG; gdh: GDHandle ); external name '_InitGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure InitGDevice(qdRefNum: SInt16; mode: SInt32; gdh: GDHandle); external name '_InitGDevice';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InitGDevice( qdRefNum: SInt16; mode: SIGNEDLONG; gdh: GDHandle ); external name '_InitGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  NewGDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewGDevice( refNum: SInt16; mode: SIGNEDLONG ): GDHandle; external name '_NewGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function NewGDevice(refNum: SInt16; mode: SInt32): GDHandle; external name '_NewGDevice';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewGDevice( refNum: SInt16; mode: SIGNEDLONG ): GDHandle; external name '_NewGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  DisposeGDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeGDevice( gdh: GDHandle ); external name '_DisposeGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DisposeGDevice(gdh: GDHandle); external name '_DisposeGDevice';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeGDevice( gdh: GDHandle ); external name '_DisposeGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetGDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetGDevice( gd: GDHandle ); external name '_SetGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetGDevice(gd: GDHandle); external name '_SetGDevice';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetGDevice( gd: GDHandle ); external name '_SetGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetGDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetGDevice: GDHandle; external name '_GetGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetGDevice: GDHandle; external name '_GetGDevice';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  Color2Index()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function Color2Index( const (*var*) myColor: RGBColor ): SIGNEDLONG; external name '_Color2Index';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function Color2Index(const (*var*) myColor: RGBColor): SInt32; external name '_Color2Index';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function Color2Index( const (*var*) myColor: RGBColor ): SIGNEDLONG; external name '_Color2Index';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  Index2Color()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure Index2Color( index: SIGNEDLONG; var aColor: RGBColor ); external name '_Index2Color';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure Index2Color(index: SInt32; var aColor: RGBColor); external name '_Index2Color';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure Index2Color( index: SIGNEDLONG; var aColor: RGBColor ); external name '_Index2Color';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  InvertColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvertColor( var myColor: RGBColor ); external name '_InvertColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure InvertColor(var myColor: RGBColor); external name '_InvertColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvertColor( var myColor: RGBColor ); external name '_InvertColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  RealColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function RealColor( const (*var*) color: RGBColor ): Boolean; external name '_RealColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function RealColor(const (*var*) color: RGBColor): boolean; external name '_RealColor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RealColor( const (*var*) color: RGBColor ): Boolean; external name '_RealColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetSubTable()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetSubTable( myColors: CTabHandle; iTabRes: SInt16; targetTbl: CTabHandle ); external name '_GetSubTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetSubTable(myColors: CTabHandle; iTabRes: SInt16; targetTbl: CTabHandle); external name '_GetSubTable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure GetSubTable( myColors: CTabHandle; iTabRes: SInt16; targetTbl: CTabHandle ); external name '_GetSubTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  MakeITable()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure MakeITable( cTabH: CTabHandle; iTabH: ITabHandle; res: SInt16 ); external name '_MakeITable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure MakeITable(cTabH: CTabHandle; iTabH: ITabHandle; res: SInt16); external name '_MakeITable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure MakeITable( cTabH: CTabHandle; iTabH: ITabHandle; res: SInt16 ); external name '_MakeITable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  AddSearch()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure AddSearch( searchProc: ColorSearchUPP ); external name '_AddSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure AddSearch(searchProc: ColorSearchUPP); external name '_AddSearch';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure AddSearch( searchProc: ColorSearchUPP ); external name '_AddSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  AddComp()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure AddComp( compProc: ColorComplementUPP ); external name '_AddComp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure AddComp(compProc: ColorComplementUPP); external name '_AddComp';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure AddComp( compProc: ColorComplementUPP ); external name '_AddComp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  DelSearch()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DelSearch( searchProc: ColorSearchUPP ); external name '_DelSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DelSearch(searchProc: ColorSearchUPP); external name '_DelSearch';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DelSearch( searchProc: ColorSearchUPP ); external name '_DelSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  DelComp()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DelComp( compProc: ColorComplementUPP ); external name '_DelComp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DelComp(compProc: ColorComplementUPP); external name '_DelComp';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DelComp( compProc: ColorComplementUPP ); external name '_DelComp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetClientID()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetClientID( id: SInt16 ); external name '_SetClientID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetClientID(id: SInt16); external name '_SetClientID';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetClientID( id: SInt16 ); external name '_SetClientID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ProtectEntry()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ProtectEntry( index: SInt16; protect: Boolean ); external name '_ProtectEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure ProtectEntry(index: SInt16; protect: boolean); external name '_ProtectEntry';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure ProtectEntry( index: SInt16; protect: Boolean ); external name '_ProtectEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ReserveEntry()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ReserveEntry( index: SInt16; reserve: Boolean ); external name '_ReserveEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure ReserveEntry(index: SInt16; reserve: boolean); external name '_ReserveEntry';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure ReserveEntry( index: SInt16; reserve: Boolean ); external name '_ReserveEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetEntries()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetEntries( start: SInt16; count: SInt16; var aTable: CSpecArray ); external name '_SetEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SetEntries(start: SInt16; count: SInt16; var aTable: CSpecArray); external name '_SetEntries';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetEntries( start: SInt16; count: SInt16; var aTable: CSpecArray ); external name '_SetEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SaveEntries()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure SaveEntries( srcTable: CTabHandle; resultTable: CTabHandle; var selection: ReqListRec ); external name '_SaveEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure SaveEntries(srcTable: CTabHandle; resultTable: CTabHandle; var selection: ReqListRec); external name '_SaveEntries';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SaveEntries( srcTable: CTabHandle; resultTable: CTabHandle; var selection: ReqListRec ); external name '_SaveEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  RestoreEntries()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure RestoreEntries( srcTable: CTabHandle; dstTable: CTabHandle; var selection: ReqListRec ); external name '_RestoreEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure RestoreEntries(srcTable: CTabHandle; dstTable: CTabHandle; var selection: ReqListRec); external name '_RestoreEntries';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure RestoreEntries( srcTable: CTabHandle; dstTable: CTabHandle; var selection: ReqListRec ); external name '_RestoreEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  QDError()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDError: SInt16; external name '_QDError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDError: SInt16; external name '_QDError';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CopyDeepMask()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CopyDeepMask( const (*var*) srcBits: BitMap; const (*var*) maskBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) maskRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle { can be NULL } ); external name '_CopyDeepMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CopyDeepMask( const (*var*) srcBits: BitMap; const (*var*) maskBits: BitMap; const (*var*) dstBits: BitMap; const (*var*) srcRect: Rect; const (*var*) maskRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle { can be NULL } ); external name '_CopyDeepMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  DeviceLoop()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DeviceLoop( drawingRgn: RgnHandle; drawingProc: DeviceLoopDrawingUPP; userData: SIGNEDLONG; flags: DeviceLoopFlags ); external name '_DeviceLoop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure DeviceLoop(drawingRgn: RgnHandle; drawingProc: DeviceLoopDrawingUPP; userData: SInt32; flags: DeviceLoopFlags); external name '_DeviceLoop';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DeviceLoop( drawingRgn: RgnHandle; drawingProc: DeviceLoopDrawingUPP; userData: SIGNEDLONG; flags: DeviceLoopFlags ); external name '_DeviceLoop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetMaskTable()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMaskTable: Ptr; external name '_GetMaskTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMaskTable: Ptr; external name '_GetMaskTable';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetPattern()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetPattern( patternID: SInt16 ): PatHandle; external name '_GetPattern';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetPattern(patternID: SInt16): PatHandle; external name '_GetPattern';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetPattern( patternID: SInt16 ): PatHandle; external name '_GetPattern';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  [Mac]GetCursor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MacGetCursor( cursorID: SInt16 ): CursHandle; external name '_GetCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
function GetCursor( cursorID: SInt16 ): CursHandle; external name '_GetCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetCursor(cursorID: SInt16): CursHandle; external name '_GetCursor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MacGetCursor( cursorID: SInt16 ): CursHandle; external name '_GetCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
function GetCursor( cursorID: SInt16 ): CursHandle; external name '_GetCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  GetPicture()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetPicture( pictureID: SInt16 ): PicHandle; external name '_GetPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetPicture(pictureID: SInt16): PicHandle; external name '_GetPicture';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  DeltaPoint()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function DeltaPoint( ptA: Point; ptB: Point ): SIGNEDLONG; external name '_DeltaPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ShieldCursor()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function DeltaPoint(ptA: Point; ptB: Point): SInt32; external name '_DeltaPoint';

{
 *  ShieldCursor()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ShieldCursor( const (*var*) shieldRect: Rect; offsetPt: Point ); external name '_ShieldCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure ShieldCursor(const (*var*) shieldRect: Rect; offsetPt: Point); external name '_ShieldCursor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetPicture( pictureID: SInt16 ): PicHandle; external name '_GetPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  ScreenRes()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ScreenRes( var scrnHRes: SInt16; var scrnVRes: SInt16 ); external name '_ScreenRes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure ScreenRes(var scrnHRes: SInt16; var scrnVRes: SInt16); external name '_ScreenRes';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  GetIndPattern()   *** DEPRECATED ***
=======
function DeltaPoint( ptA: Point; ptB: Point ): SIGNEDLONG; external name '_DeltaPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ShieldCursor()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure GetIndPattern( var thePat: Pattern; patternListID: SInt16; index: SInt16 ); external name '_GetIndPattern';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure ShieldCursor( const (*var*) shieldRect: Rect; offsetPt: Point ); external name '_ShieldCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{
 *  deltapoint()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
// overloading not possible
// function deltapoint( var ptA: Point; var ptB: Point ): SIGNEDLONG;
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;


{$endc} {not TARGET_CPU_64}
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetIndPattern(var thePat: Pattern; patternListID: SInt16; index: SInt16); external name '_GetIndPattern';
=======
procedure ScreenRes( var scrnHRes: SInt16; var scrnVRes: SInt16 ); external name '_ScreenRes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{$ifc OLDROUTINENAMES}
{$ifc CALL_NOT_IN_CARBON}
{
 *  DisposPixMap()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure GetIndPattern(var thePat: Pattern; patternListID: SInt16; index: SInt16); external name '_GetIndPattern';

{$ifc OLDROUTINENAMES}
{$ifc CALL_NOT_IN_CARBON}
{
 *  DisposPixMap()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
procedure DisposPixMap(pm: PixMapHandle); external name '_DisposPixMap';
{
 *  DisposPixPat()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure DisposPixPat(pp: PixPatHandle); external name '_DisposPixPat';
{
 *  DisposCTable()
 *  
 *  Availability:
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 }
procedure GetIndPattern( var thePat: Pattern; patternListID: SInt16; index: SInt16 ); external name '_GetIndPattern';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{
 *  deltapoint()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
procedure DisposGDevice(gdh: GDHandle); external name '_DisposGDevice';
{$endc}  {CALL_NOT_IN_CARBON}
{$endc}  {OLDROUTINENAMES}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
// overloading not possible
// function deltapoint( var ptA: Point; var ptB: Point ): SIGNEDLONG;
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;


{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

{
    From ToolUtils.i
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  PackBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure PackBits( var srcPtr: Ptr; var dstPtr: Ptr; srcBytes: SInt16 ); external name '_PackBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
procedure PackBits(var srcPtr: Ptr; var dstPtr: Ptr; srcBytes: SInt16); external name '_PackBits';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PackBits( var srcPtr: Ptr; var dstPtr: Ptr; srcBytes: SInt16 ); external name '_PackBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  UnpackBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure UnpackBits( var srcPtr: Ptr; var dstPtr: Ptr; dstBytes: SInt16 ); external name '_UnpackBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SlopeFromAngle()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure UnpackBits( var srcPtr: Ptr; var dstPtr: Ptr; dstBytes: SInt16 ); external name '_UnpackBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  SlopeFromAngle()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SlopeFromAngle()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SlopeFromAngle( angle: SInt16 ): Fixed; external name '_SlopeFromAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AngleFromSlope()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SlopeFromAngle( angle: SInt16 ): Fixed; external name '_SlopeFromAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  AngleFromSlope()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  AngleFromSlope()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function AngleFromSlope( slope: Fixed ): SInt16; external name '_AngleFromSlope';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ New transfer modes }
{$endc} {not TARGET_CPU_64}

const
	colorXorXFer = 52;
	noiseXFer = 53;
	customXFer = 54;

{ Custom XFer flags }
const
	kXFer1PixelAtATime = $00000001; { 1 pixel passed to custom XFer proc}
	kXFerConvertPixelToRGB32 = $00000002; { All color depths converted to 32 bit RGB}

type
	CustomXFerRec = record
		version: UInt32;
		srcPixels: UnivPtr;
		destPixels: UnivPtr;
		resultPixels: UnivPtr;
		refCon: UInt32;
		pixelSize: UInt32;
		pixelCount: UInt32;
		firstPixelHV: Point;
		destBounds: Rect;
	end;
	CustomXFerRecPtr = ^CustomXFerRec;
type
	CustomXFerProcPtr = procedure( info: CustomXFerRecPtr );
{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function AngleFromSlope( slope: Fixed ): SInt16; external name '_AngleFromSlope';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ New transfer modes }
{$endc} {not TARGET_CPU_64}

const
	colorXorXFer = 52;
	noiseXFer = 53;
	customXFer = 54;

{ Custom XFer flags }
const
	kXFer1PixelAtATime = $00000001; { 1 pixel passed to custom XFer proc}
	kXFerConvertPixelToRGB32 = $00000002; { All color depths converted to 32 bit RGB}

type
	CustomXFerRec = record
		version: UInt32;
		srcPixels: UnivPtr;
		destPixels: UnivPtr;
		resultPixels: UnivPtr;
		refCon: UInt32;
		pixelSize: UInt32;
		pixelCount: UInt32;
		firstPixelHV: Point;
		destBounds: Rect;
	end;
<<<<<<< HEAD

{$ifc TYPED_FUNCTION_POINTERS}
	CustomXFerProcPtr = procedure(info: CustomXFerRecPtr);
{$elsec}
	CustomXFerProcPtr = ProcPtr;
{$endc}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	CustomXFerRecPtr = ^CustomXFerRec;
type
	CustomXFerProcPtr = procedure( info: CustomXFerRecPtr );
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  GetPortCustomXFerProc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetPortCustomXFerProc( port: CGrafPtr; var proc: CustomXFerProcPtr; var flags: UInt32; var refCon: UInt32 ): OSErr; external name '_GetPortCustomXFerProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function GetPortCustomXFerProc(port: CGrafPtr; var proc: CustomXFerProcPtr; var flags: UInt32; var refCon: UInt32): OSErr; external name '_GetPortCustomXFerProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetPortCustomXFerProc( port: CGrafPtr; var proc: CustomXFerProcPtr; var flags: UInt32; var refCon: UInt32 ): OSErr; external name '_GetPortCustomXFerProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetPortCustomXFerProc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetPortCustomXFerProc( port: CGrafPtr; proc: CustomXFerProcPtr; flags: UInt32; refCon: UInt32 ): OSErr; external name '_SetPortCustomXFerProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

const
	kCursorComponentsVersion = $00010001;

const
	kCursorComponentType = FourCharCode('curs');

{ Cursor Component capabilities flags }
const
	cursorDoesAnimate = 1 shl 0;
	cursorDoesHardware = 1 shl 1;
	cursorDoesUnreadableScreenBits = 1 shl 2;

{ Cursor Component output mode flags }
const
	kRenderCursorInHardware = 1 shl 0;
	kRenderCursorInSoftware = 1 shl 1;

{ Cursor Component Info }
type
	CursorInfoPtr = ^CursorInfo;
	CursorInfo = record
		version: SIGNEDLONG;                { use kCursorComponentsVersion }
		capabilities: SIGNEDLONG;
		animateDuration: SIGNEDLONG;        { approximate time between animate tickles }
		bounds: Rect;
		hotspot: Point;
		reserved: SIGNEDLONG;               { must set to zero }
	end;
{ Cursor Component Selectors }
const
	kCursorComponentInit = $0001;
	kCursorComponentGetInfo = $0002;
	kCursorComponentSetOutputMode = $0003;
	kCursorComponentSetData = $0004;
	kCursorComponentReconfigure = $0005;
	kCursorComponentDraw = $0006;
	kCursorComponentErase = $0007;
	kCursorComponentMove = $0008;
	kCursorComponentAnimate = $0009;
	kCursorComponentLastReserved = $0050;

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetPortCustomXFerProc( port: CGrafPtr; proc: CustomXFerProcPtr; flags: UInt32; refCon: UInt32 ): OSErr; external name '_SetPortCustomXFerProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

const
	kCursorComponentsVersion = $00010001;

const
	kCursorComponentType = FourCharCode('curs');

{ Cursor Component capabilities flags }
const
	cursorDoesAnimate = 1 shl 0;
	cursorDoesHardware = 1 shl 1;
	cursorDoesUnreadableScreenBits = 1 shl 2;

{ Cursor Component output mode flags }
const
	kRenderCursorInHardware = 1 shl 0;
	kRenderCursorInSoftware = 1 shl 1;

{ Cursor Component Info }
type
	CursorInfoPtr = ^CursorInfo;
	CursorInfo = record
		version: SIGNEDLONG;                { use kCursorComponentsVersion }
		capabilities: SIGNEDLONG;
		animateDuration: SIGNEDLONG;        { approximate time between animate tickles }
		bounds: Rect;
		hotspot: Point;
		reserved: SIGNEDLONG;               { must set to zero }
	end;
{ Cursor Component Selectors }
const
	kCursorComponentInit = $0001;
	kCursorComponentGetInfo = $0002;
	kCursorComponentSetOutputMode = $0003;
	kCursorComponentSetData = $0004;
	kCursorComponentReconfigure = $0005;
	kCursorComponentDraw = $0006;
	kCursorComponentErase = $0007;
	kCursorComponentMove = $0008;
	kCursorComponentAnimate = $0009;
	kCursorComponentLastReserved = $0050;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  OpenCursorComponent()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OpenCursorComponent( c: Component; var ci: ComponentInstance ): OSErr; external name '_OpenCursorComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function OpenCursorComponent(c: Component; var ci: ComponentInstance): OSErr; external name '_OpenCursorComponent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OpenCursorComponent( c: Component; var ci: ComponentInstance ): OSErr; external name '_OpenCursorComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CloseCursorComponent()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CloseCursorComponent( ci: ComponentInstance ): OSErr; external name '_CloseCursorComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function CloseCursorComponent(ci: ComponentInstance): OSErr; external name '_CloseCursorComponent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CloseCursorComponent( ci: ComponentInstance ): OSErr; external name '_CloseCursorComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetCursorComponent()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetCursorComponent( ci: ComponentInstance ): OSErr; external name '_SetCursorComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function SetCursorComponent(ci: ComponentInstance): OSErr; external name '_SetCursorComponent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SetCursorComponent( ci: ComponentInstance ): OSErr; external name '_SetCursorComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CursorComponentChanged()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CursorComponentChanged( ci: ComponentInstance ): OSErr; external name '_CursorComponentChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
//AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4;
function CursorComponentChanged(ci: ComponentInstance): OSErr; external name '_CursorComponentChanged';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CursorComponentChanged( ci: ComponentInstance ): OSErr; external name '_CursorComponentChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  CursorComponentSetData()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CursorComponentSetData( ci: ComponentInstance; data: SIGNEDLONG ): OSErr; external name '_CursorComponentSetData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Quickdraw-specific ColorSync matching }
{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CursorComponentSetData( ci: ComponentInstance; data: SIGNEDLONG ): OSErr; external name '_CursorComponentSetData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Quickdraw-specific ColorSync matching }
{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  CWMatchPixMap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CWMatchPixMap( cw: CMWorldRef; var myPixMap: PixMap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr ): CMError; external name '_CWMatchPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 1.0 and later]
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function CWMatchPixMap( cw: CMWorldRef; var myPixMap: PixMap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr ): CMError; external name '_CWMatchPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  CWCheckPixMap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CWCheckPixMap( cw: CMWorldRef; var myPixMap: PixMap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitMap: BitMap ): CMError; external name '_CWCheckPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 1.0 and later]
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function CWCheckPixMap( cw: CMWorldRef; var myPixMap: PixMap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitMap: BitMap ): CMError; external name '_CWCheckPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  NCMBeginMatching()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function NCMBeginMatching( src: CMProfileRef; dst: CMProfileRef; var myRef: CMMatchRef ): CMError; external name '_NCMBeginMatching';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  CMEndMatching()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
procedure CMEndMatching( myRef: CMMatchRef ); external name '_CMEndMatching';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  NCMDrawMatchedPicture()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
procedure NCMDrawMatchedPicture( myPicture: PicHandle; dst: CMProfileRef; var myRect: Rect ); external name '_NCMDrawMatchedPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  CMEnableMatchingComment()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
procedure CMEnableMatchingComment( enableIt: Boolean ); external name '_CMEnableMatchingComment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  NCMUseProfileComment()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        not available [in CarbonLib 1.0 and later]
 *    Non-Carbon CFM:   not available [in ColorSyncLib 2.0 and later]
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function NCMUseProfileComment( prof: CMProfileRef; flags: UInt32 ): CMError; external name '_NCMUseProfileComment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
<<<<<<< HEAD
=======
{ Available in CarbonLib... }
>>>>>>> graemeg/fixes_2_2
=======
{ Available in CarbonLib... }
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  IsValidPort()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Attempts to detect invalid ports
 *  
 *  Discussion:
 *    A grafPort structure contains many nested Handles. An attempt to
 *    guarantee that they are all valid is prohibitively costly. Since
 *    10.1, IsValidPort only compares the CGrafPtr parameter against
 *    the list of grafPorts maintained internally by Quickdraw. The
 *    function returns true if it is found, false otherwise. This is
 *    enough to detect ports belonging to windows that have been
 *    closed, or GWorlds that have been deallocated.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    port:
 *      The CGrafPtr in question.
 *  
 *  Result:
 *    If false, port is definitely invalid. If true, port is probably
 *    valid (unless memory has been clobbered)
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsValidPort( port: CGrafPtr ): Boolean; external name '_IsValidPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsValidPort( port: CGrafPtr ): Boolean; external name '_IsValidPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  IsValidRgnHandle()
 *  
 *  Summary:
 *    Tests a RgnHandle
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    rgn:
 *      The RgnHandle in question.
 *  
 *  Result:
 *    If false, either the RgnHandle parameter is NULL, or has been
 *    deallocated, or the region data are corrupt. The latter can
 *    happen when memory has been overwritten. If true, the RgnHandle
 *    is valid. NOTE: In 10.3, the internal (opaque) format of region
 *    data has changed, to overcome size and performance limitations.
 *    Do not attempt to access region data by dereferencing the
 *    RgnHandle.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function IsValidRgnHandle( rgn: RgnHandle ): Boolean; external name '_IsValidRgnHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$endc} {not TARGET_CPU_64}

{ GrafPort }
{ Getters }
{$ifc not TARGET_CPU_64}
=======
{$ifc ACCESSOR_CALLS_ARE_FUNCTIONS}
{ GrafPort }
{ Getters }
>>>>>>> graemeg/fixes_2_2
=======
{$ifc ACCESSOR_CALLS_ARE_FUNCTIONS}
{ GrafPort }
{ Getters }
>>>>>>> origin/fixes_2_2
=======

{$endc} {not TARGET_CPU_64}

{ GrafPort }
{ Getters }
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  GetPortPixMap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPixMap( port: CGrafPtr ): PixMapHandle; external name '_GetPortPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortBitMapForCopyBits()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPixMap( port: CGrafPtr ): PixMapHandle; external name '_GetPortPixMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortBitMapForCopyBits()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortBitMapForCopyBits()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    GetPortBitMapForCopyBits is provided for the specific purpose of
 *    using the return value as a parameter to CopyBits. The return
 *    value can be used as the srcBits or dstBits parameter to CopyBits
 *    regardless of whether the port is color. If the port parameter is
 *    a color port, however, the returned BitMapPtr does not actually
 *    point to a BitMap; it points to the PixMapHandle and other fields
 *    in the CGrafPort structure. You should not dereference the
 *    BitMapPtr or otherwise depend on its contents unless you've
 *    confirmed that this port is a non-color port.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0.2 and later
 }
function GetPortBitMapForCopyBits( port: CGrafPtr ): BitMapPtr; external name '_GetPortBitMapForCopyBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortBounds()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0.2 and later
 }
function GetPortBitMapForCopyBits( port: CGrafPtr ): BitMapPtr; external name '_GetPortBitMapForCopyBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortBounds()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortBounds()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortBounds( port: CGrafPtr; var rect_: Rect ): RectPtr; external name '_GetPortBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortForeColor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortBounds( port: CGrafPtr; var rect_: Rect ): RectPtr; external name '_GetPortBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortForeColor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortForeColor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortForeColor( port: CGrafPtr; var foreColor: RGBColor ): RGBColorPtr; external name '_GetPortForeColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortBackColor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortForeColor( port: CGrafPtr; var foreColor: RGBColor ): RGBColorPtr; external name '_GetPortForeColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortBackColor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortBackColor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortBackColor( port: CGrafPtr; var backColor: RGBColor ): RGBColorPtr; external name '_GetPortBackColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortOpColor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortBackColor( port: CGrafPtr; var backColor: RGBColor ): RGBColorPtr; external name '_GetPortBackColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortOpColor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortOpColor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortOpColor( port: CGrafPtr; var opColor: RGBColor ): RGBColorPtr; external name '_GetPortOpColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortHiliteColor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortOpColor( port: CGrafPtr; var opColor: RGBColor ): RGBColorPtr; external name '_GetPortOpColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortHiliteColor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortHiliteColor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortHiliteColor( port: CGrafPtr; var hiliteColor: RGBColor ): RGBColorPtr; external name '_GetPortHiliteColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortGrafProcs()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortHiliteColor( port: CGrafPtr; var hiliteColor: RGBColor ): RGBColorPtr; external name '_GetPortHiliteColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortGrafProcs()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortGrafProcs()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortGrafProcs( port: CGrafPtr ): CQDProcsPtr; external name '_GetPortGrafProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortTextFont()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortGrafProcs( port: CGrafPtr ): CQDProcsPtr; external name '_GetPortGrafProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortTextFont()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortTextFont()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortTextFont( port: CGrafPtr ): SInt16; external name '_GetPortTextFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortTextFace()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortTextFont( port: CGrafPtr ): SInt16; external name '_GetPortTextFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortTextFace()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortTextFace()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortTextFace( port: CGrafPtr ): ByteParameter; external name '_GetPortTextFace';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortTextMode()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortTextFace( port: CGrafPtr ): ByteParameter; external name '_GetPortTextFace';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortTextMode()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortTextMode()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortTextMode( port: CGrafPtr ): SInt16; external name '_GetPortTextMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortTextSize()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortTextMode( port: CGrafPtr ): SInt16; external name '_GetPortTextMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortTextSize()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortTextSize()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortTextSize( port: CGrafPtr ): SInt16; external name '_GetPortTextSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortChExtra()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortTextSize( port: CGrafPtr ): SInt16; external name '_GetPortTextSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortChExtra()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortChExtra()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortChExtra( port: CGrafPtr ): SInt16; external name '_GetPortChExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortFracHPenLocation()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortChExtra( port: CGrafPtr ): SInt16; external name '_GetPortChExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortFracHPenLocation()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortFracHPenLocation()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortFracHPenLocation( port: CGrafPtr ): SInt16; external name '_GetPortFracHPenLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortSpExtra()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortFracHPenLocation( port: CGrafPtr ): SInt16; external name '_GetPortFracHPenLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortSpExtra()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortSpExtra()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortSpExtra( port: CGrafPtr ): Fixed; external name '_GetPortSpExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortPenVisibility()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortSpExtra( port: CGrafPtr ): Fixed; external name '_GetPortSpExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortPenVisibility()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortPenVisibility()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenVisibility( port: CGrafPtr ): SInt16; external name '_GetPortPenVisibility';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortVisibleRegion()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenVisibility( port: CGrafPtr ): SInt16; external name '_GetPortPenVisibility';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortVisibleRegion()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortVisibleRegion()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortVisibleRegion( port: CGrafPtr; visRgn: RgnHandle ): RgnHandle; external name '_GetPortVisibleRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortClipRegion()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortVisibleRegion( port: CGrafPtr; visRgn: RgnHandle ): RgnHandle; external name '_GetPortVisibleRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortClipRegion()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortClipRegion()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortClipRegion( port: CGrafPtr; clipRgn: RgnHandle ): RgnHandle; external name '_GetPortClipRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortBackPixPat()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortClipRegion( port: CGrafPtr; clipRgn: RgnHandle ): RgnHandle; external name '_GetPortClipRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortBackPixPat()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortBackPixPat()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortBackPixPat( port: CGrafPtr; backPattern: PixPatHandle ): PixPatHandle; external name '_GetPortBackPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortPenPixPat()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortBackPixPat( port: CGrafPtr; backPattern: PixPatHandle ): PixPatHandle; external name '_GetPortBackPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortPenPixPat()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortPenPixPat()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenPixPat( port: CGrafPtr; penPattern: PixPatHandle ): PixPatHandle; external name '_GetPortPenPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortFillPixPat()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenPixPat( port: CGrafPtr; penPattern: PixPatHandle ): PixPatHandle; external name '_GetPortPenPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortFillPixPat()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortFillPixPat()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortFillPixPat( port: CGrafPtr; fillPattern: PixPatHandle ): PixPatHandle; external name '_GetPortFillPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortPenSize()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortFillPixPat( port: CGrafPtr; fillPattern: PixPatHandle ): PixPatHandle; external name '_GetPortFillPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortPenSize()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortPenSize()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenSize( port: CGrafPtr; var penSize: Point ): PointPtr; external name '_GetPortPenSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortPenMode()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenSize( port: CGrafPtr; var penSize: Point ): PointPtr; external name '_GetPortPenSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortPenMode()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortPenMode()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenMode( port: CGrafPtr ): SInt32; external name '_GetPortPenMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPortPenLocation()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenMode( port: CGrafPtr ): SInt32; external name '_GetPortPenMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  GetPortPenLocation()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetPortPenLocation()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenLocation( port: CGrafPtr; var penLocation: Point ): PointPtr; external name '_GetPortPenLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  IsPortRegionBeingDefined()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPortPenLocation( port: CGrafPtr; var penLocation: Point ): PointPtr; external name '_GetPortPenLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  IsPortRegionBeingDefined()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsPortRegionBeingDefined()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsPortRegionBeingDefined( port: CGrafPtr ): Boolean; external name '_IsPortRegionBeingDefined';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  IsPortPictureBeingDefined()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsPortRegionBeingDefined( port: CGrafPtr ): Boolean; external name '_IsPortRegionBeingDefined';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  IsPortPictureBeingDefined()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsPortPictureBeingDefined()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsPortPictureBeingDefined( port: CGrafPtr ): Boolean; external name '_IsPortPictureBeingDefined';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  IsPortPolyBeingDefined()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsPortPictureBeingDefined( port: CGrafPtr ): Boolean; external name '_IsPortPictureBeingDefined';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  IsPortPolyBeingDefined()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsPortPolyBeingDefined()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
function IsPortPolyBeingDefined( port: CGrafPtr ): Boolean; external name '_IsPortPolyBeingDefined';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  IsPortOffscreen()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
function IsPortPolyBeingDefined( port: CGrafPtr ): Boolean; external name '_IsPortPolyBeingDefined';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  IsPortOffscreen()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsPortOffscreen()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsPortOffscreen( port: CGrafPtr ): Boolean; external name '_IsPortOffscreen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  IsPortColor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsPortOffscreen( port: CGrafPtr ): Boolean; external name '_IsPortOffscreen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  IsPortColor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsPortColor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0.2 and later
 }
function IsPortColor( port: CGrafPtr ): Boolean; external name '_IsPortColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  IsPortVisibleRegionEmpty()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0.2 and later
 }
function IsPortColor( port: CGrafPtr ): Boolean; external name '_IsPortColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  IsPortVisibleRegionEmpty()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsPortVisibleRegionEmpty()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.1
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function IsPortVisibleRegionEmpty( port: CGrafPtr ): Boolean; external name '_IsPortVisibleRegionEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  IsPortClipRegionEmpty()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function IsPortVisibleRegionEmpty( port: CGrafPtr ): Boolean; external name '_IsPortVisibleRegionEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  IsPortClipRegionEmpty()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsPortClipRegionEmpty()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.1
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function IsPortClipRegionEmpty( port: CGrafPtr ): Boolean; external name '_IsPortClipRegionEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SectRegionWithPortClipRegion()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function IsPortClipRegionEmpty( port: CGrafPtr ): Boolean; external name '_IsPortClipRegionEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  SectRegionWithPortClipRegion()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SectRegionWithPortClipRegion()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure SectRegionWithPortClipRegion( port: CGrafPtr; ioRegion: RgnHandle ); external name '_SectRegionWithPortClipRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SectRegionWithPortVisibleRegion()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure SectRegionWithPortClipRegion( port: CGrafPtr; ioRegion: RgnHandle ); external name '_SectRegionWithPortClipRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  SectRegionWithPortVisibleRegion()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SectRegionWithPortVisibleRegion()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure SectRegionWithPortVisibleRegion( port: CGrafPtr; ioRegion: RgnHandle ); external name '_SectRegionWithPortVisibleRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure SectRegionWithPortVisibleRegion(port: CGrafPtr; ioRegion: RgnHandle); external name '_SectRegionWithPortVisibleRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure SectRegionWithPortVisibleRegion( port: CGrafPtr; ioRegion: RgnHandle ); external name '_SectRegionWithPortVisibleRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{ Swappers }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  SwapPortPicSaveHandle()   *** DEPRECATED ***
=======
 *  SwapPortPicSaveHandle()
>>>>>>> graemeg/fixes_2_2
=======
 *  SwapPortPicSaveHandle()
>>>>>>> origin/fixes_2_2
=======
 *  SwapPortPicSaveHandle()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets the port's picSave Handle, and returns the previous picSave
 *  
 *  Discussion:
 *    Historically, the picSave field in a GrafPort is != NULL if a
 *    Picture is being defined; and it has been documented that picture
 *    definition can be temporarily suspended by saving the current
 *    picSave Handle and setting picSave to NULL. Restoring the saved
 *    picSave Handle resumes picture definition.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    port:
 *      The port whose picSave field is being swapped.
 *    
 *    inPicSaveHdl:
 *      The picSave Handle to be set.
 *  
 *  Result:
 *    The previous picSave Handle in the port.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function SwapPortPicSaveHandle( port: CGrafPtr; inPicSaveHdl: Handle ): Handle; external name '_SwapPortPicSaveHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Similarly: }
{
 *  SwapPortPolySaveHandle()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function SwapPortPicSaveHandle( port: CGrafPtr; inPicSaveHdl: Handle ): Handle; external name '_SwapPortPicSaveHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Similarly: }
{
<<<<<<< HEAD
 *  SwapPortPolySaveHandle()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SwapPortPolySaveHandle()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function SwapPortPolySaveHandle( port: CGrafPtr; inPolySaveHdl: Handle ): Handle; external name '_SwapPortPolySaveHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SwapPortRegionSaveHandle()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function SwapPortPolySaveHandle( port: CGrafPtr; inPolySaveHdl: Handle ): Handle; external name '_SwapPortPolySaveHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  SwapPortRegionSaveHandle()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SwapPortRegionSaveHandle()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function SwapPortRegionSaveHandle( port: CGrafPtr; inRegionSaveHdl: Handle ): Handle; external name '_SwapPortRegionSaveHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function SwapPortRegionSaveHandle(port: CGrafPtr; inRegionSaveHdl: Handle): Handle; external name '_SwapPortRegionSaveHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function SwapPortRegionSaveHandle( port: CGrafPtr; inRegionSaveHdl: Handle ): Handle; external name '_SwapPortRegionSaveHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{ Setters }
{
 *  SetPortBounds()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortBounds( port: CGrafPtr; const (*var*) rect_: Rect ); external name '_SetPortBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortBounds( port: CGrafPtr; const (*var*) rect_: Rect ); external name '_SetPortBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortOpColor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortOpColor( port: CGrafPtr; const (*var*) opColor: RGBColor ); external name '_SetPortOpColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortOpColor( port: CGrafPtr; const (*var*) opColor: RGBColor ); external name '_SetPortOpColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortGrafProcs()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortGrafProcs( port: CGrafPtr; procs: CQDProcsPtr ); external name '_SetPortGrafProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetPortTextFont()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortGrafProcs( port: CGrafPtr; procs: CQDProcsPtr ); external name '_SetPortGrafProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  SetPortTextFont()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SetPortTextFont()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextFont( port: CGrafPtr; txFont: SInt16 ); external name '_SetPortTextFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetPortTextSize()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextFont( port: CGrafPtr; txFont: SInt16 ); external name '_SetPortTextFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  SetPortTextSize()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextSize( port: CGrafPtr; txSize: SInt16 ); external name '_SetPortTextSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetPortTextFace()   *** DEPRECATED ***
=======
=======
=======
 *  SetPortTextSize()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextSize( port: CGrafPtr; txSize: SInt16 ); external name '_SetPortTextSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  SetPortTextFace()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SetPortTextFace()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextFace( port: CGrafPtr; face: StyleParameter ); external name '_SetPortTextFace';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetPortTextMode()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextFace( port: CGrafPtr; face: StyleParameter ); external name '_SetPortTextFace';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  SetPortTextMode()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SetPortTextMode()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextMode( port: CGrafPtr; mode: SInt16 ); external name '_SetPortTextMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextMode(port: CGrafPtr; mode: SInt16); external name '_SetPortTextMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetPortTextMode( port: CGrafPtr; mode: SInt16 ); external name '_SetPortTextMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  SetPortVisibleRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortVisibleRegion( port: CGrafPtr; visRgn: RgnHandle ); external name '_SetPortVisibleRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortVisibleRegion( port: CGrafPtr; visRgn: RgnHandle ); external name '_SetPortVisibleRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortClipRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortClipRegion( port: CGrafPtr; clipRgn: RgnHandle ); external name '_SetPortClipRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortClipRegion( port: CGrafPtr; clipRgn: RgnHandle ); external name '_SetPortClipRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortPenPixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortPenPixPat( port: CGrafPtr; penPattern: PixPatHandle ); external name '_SetPortPenPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortPenPixPat( port: CGrafPtr; penPattern: PixPatHandle ); external name '_SetPortPenPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortFillPixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.2 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.2 and later
 }
procedure SetPortFillPixPat( port: CGrafPtr; penPattern: PixPatHandle ); external name '_SetPortFillPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.2 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.2 and later
 }
procedure SetPortFillPixPat( port: CGrafPtr; penPattern: PixPatHandle ); external name '_SetPortFillPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortBackPixPat()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortBackPixPat( port: CGrafPtr; backPattern: PixPatHandle ); external name '_SetPortBackPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortBackPixPat( port: CGrafPtr; backPattern: PixPatHandle ); external name '_SetPortBackPixPat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortPenSize()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortPenSize( port: CGrafPtr; penSize: Point ); external name '_SetPortPenSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortPenSize( port: CGrafPtr; penSize: Point ); external name '_SetPortPenSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortPenMode()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortPenMode( port: CGrafPtr; penMode: SInt32 ); external name '_SetPortPenMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortPenMode( port: CGrafPtr; penMode: SInt32 ); external name '_SetPortPenMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetPortFracHPenLocation()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortFracHPenLocation( port: CGrafPtr; pnLocHFrac: SInt16 ); external name '_SetPortFracHPenLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetPortFracHPenLocation( port: CGrafPtr; pnLocHFrac: SInt16 ); external name '_SetPortFracHPenLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{ PixMap }
{
 *  GetPixBounds()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPixBounds( pixMap: PixMapHandle; var bounds: Rect ): RectPtr; external name '_GetPixBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPixBounds( pixMap: PixMapHandle; var bounds: Rect ): RectPtr; external name '_GetPixBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetPixDepth()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPixDepth( pixMap: PixMapHandle ): SInt16; external name '_GetPixDepth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetPixDepth( pixMap: PixMapHandle ): SInt16; external name '_GetPixDepth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{ QDGlobals }
{ Getters }
{
 *  GetQDGlobalsRandomSeed()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsRandomSeed: SIGNEDLONG; external name '_GetQDGlobalsRandomSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsRandomSeed: SIGNEDLONG; external name '_GetQDGlobalsRandomSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetQDGlobalsScreenBits()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsScreenBits( var screenBits: BitMap ): BitMapPtr; external name '_GetQDGlobalsScreenBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsScreenBits( var screenBits: BitMap ): BitMapPtr; external name '_GetQDGlobalsScreenBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetQDGlobalsArrow()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsArrow( var arrow: Cursor ): CursorPtr; external name '_GetQDGlobalsArrow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsArrow( var arrow: Cursor ): CursorPtr; external name '_GetQDGlobalsArrow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetQDGlobalsDarkGray()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsDarkGray( var dkGray: Pattern ): PatternPtr; external name '_GetQDGlobalsDarkGray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsDarkGray( var dkGray: Pattern ): PatternPtr; external name '_GetQDGlobalsDarkGray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetQDGlobalsLightGray()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsLightGray( var ltGray: Pattern ): PatternPtr; external name '_GetQDGlobalsLightGray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsLightGray( var ltGray: Pattern ): PatternPtr; external name '_GetQDGlobalsLightGray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetQDGlobalsGray()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsGray( var gray: Pattern ): PatternPtr; external name '_GetQDGlobalsGray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsGray( var gray: Pattern ): PatternPtr; external name '_GetQDGlobalsGray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetQDGlobalsBlack()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsBlack( var black: Pattern ): PatternPtr; external name '_GetQDGlobalsBlack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsBlack( var black: Pattern ): PatternPtr; external name '_GetQDGlobalsBlack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetQDGlobalsWhite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsWhite( var white: Pattern ): PatternPtr; external name '_GetQDGlobalsWhite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsWhite( var white: Pattern ): PatternPtr; external name '_GetQDGlobalsWhite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  GetQDGlobalsThePort()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetQDGlobalsThePort: CGrafPtr; external name '_GetQDGlobalsThePort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{ Setters }
{
 *  SetQDGlobalsRandomSeed()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetQDGlobalsRandomSeed( randomSeed: SIGNEDLONG ); external name '_SetQDGlobalsRandomSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetQDGlobalsRandomSeed( randomSeed: SIGNEDLONG ); external name '_SetQDGlobalsRandomSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetQDGlobalsArrow()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetQDGlobalsArrow( const (*var*) arrow: Cursor ); external name '_SetQDGlobalsArrow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Regions }
{
 *  GetRegionBounds()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetQDGlobalsArrow( const (*var*) arrow: Cursor ); external name '_SetQDGlobalsArrow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Regions }
{
<<<<<<< HEAD
 *  GetRegionBounds()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetRegionBounds()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetRegionBounds( region: RgnHandle; var bounds: Rect ): RectPtr; external name '_GetRegionBounds'; (* attribute ignoreable *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsRegionRectangular()
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetRegionBounds( region: RgnHandle; var bounds: Rect ): RectPtr; external name '_GetRegionBounds'; (* attribute ignoreable *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  IsRegionRectangular()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsRegionRectangular()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsRegionRectangular( region: RgnHandle ): Boolean; external name '_IsRegionRectangular';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsRegionRectangular(region: RgnHandle): boolean; external name '_IsRegionRectangular';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function IsRegionRectangular( region: RgnHandle ): Boolean; external name '_IsRegionRectangular';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Utilities }
{ To prevent upward dependencies, GetWindowFromPort() is defined in Window Manager interface: }
{      pascal WindowRef        GetWindowFromPort(CGrafPtr port); }
{ NewPtr/OpenCPort doesn't work with opaque structures }
{
 *  CreateNewPort()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function CreateNewPort: CGrafPtr; external name '_CreateNewPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  DisposePort()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure DisposePort( port: CGrafPtr ); external name '_DisposePort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure DisposePort( port: CGrafPtr ); external name '_DisposePort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SetQDError()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0.2 and later
 }
procedure SetQDError( err: OSErr ); external name '_SetQDError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{ Helpful Carbon-only utilities (finally made public)}

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0.2 and later
 }
procedure SetQDError( err: OSErr ); external name '_SetQDError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{ Helpful Carbon-only utilities (finally made public)}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  QDLocalToGlobalPoint()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDLocalToGlobalPoint( port: CGrafPtr; var point_: Point ): PointPtr; external name '_QDLocalToGlobalPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDLocalToGlobalPoint( port: CGrafPtr; var point_: Point ): PointPtr; external name '_QDLocalToGlobalPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDGlobalToLocalPoint()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDGlobalToLocalPoint( port: CGrafPtr; var point_: Point ): PointPtr; external name '_QDGlobalToLocalPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDGlobalToLocalPoint( port: CGrafPtr; var point_: Point ): PointPtr; external name '_QDGlobalToLocalPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDLocalToGlobalRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDLocalToGlobalRect( port: CGrafPtr; var bounds: Rect ): RectPtr; external name '_QDLocalToGlobalRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDLocalToGlobalRect( port: CGrafPtr; var bounds: Rect ): RectPtr; external name '_QDLocalToGlobalRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDGlobalToLocalRect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDGlobalToLocalRect( port: CGrafPtr; var bounds: Rect ): RectPtr; external name '_QDGlobalToLocalRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDGlobalToLocalRect( port: CGrafPtr; var bounds: Rect ): RectPtr; external name '_QDGlobalToLocalRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDLocalToGlobalRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDLocalToGlobalRegion( port: CGrafPtr; region: RgnHandle ): RgnHandle; external name '_QDLocalToGlobalRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDLocalToGlobalRegion( port: CGrafPtr; region: RgnHandle ): RgnHandle; external name '_QDLocalToGlobalRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDGlobalToLocalRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDGlobalToLocalRegion( port: CGrafPtr; region: RgnHandle ): RgnHandle; external name '_QDGlobalToLocalRegion';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDGlobalToLocalRegion(port: CGrafPtr; region: RgnHandle): RgnHandle; external name '_QDGlobalToLocalRegion';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDGlobalToLocalRegion( port: CGrafPtr; region: RgnHandle ): RgnHandle; external name '_QDGlobalToLocalRegion';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
   Routines available on Mac OS X to flush buffered window ports...
   These calls do nothing on Mac OS 8/9. QDIsPortBuffered will always return false there.
}

{
 *  QDIsPortBuffered()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDIsPortBuffered( port: CGrafPtr ): Boolean; external name '_QDIsPortBuffered';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDIsPortBuffered( port: CGrafPtr ): Boolean; external name '_QDIsPortBuffered';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDIsPortBufferDirty()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDIsPortBufferDirty( port: CGrafPtr ): Boolean; external name '_QDIsPortBufferDirty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDIsPortBufferDirty( port: CGrafPtr ): Boolean; external name '_QDIsPortBufferDirty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDFlushPortBuffer()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
procedure QDFlushPortBuffer( port: CGrafPtr; region: RgnHandle { can be NULL } ); external name '_QDFlushPortBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
procedure QDFlushPortBuffer( port: CGrafPtr; region: RgnHandle { can be NULL } ); external name '_QDFlushPortBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDGetDirtyRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function QDGetDirtyRegion( port: CGrafPtr; rgn: RgnHandle ): OSStatus; external name '_QDGetDirtyRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function QDGetDirtyRegion( port: CGrafPtr; rgn: RgnHandle ): OSStatus; external name '_QDGetDirtyRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDSetDirtyRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function QDSetDirtyRegion( port: CGrafPtr; rgn: RgnHandle ): OSStatus; external name '_QDSetDirtyRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function QDSetDirtyRegion( port: CGrafPtr; rgn: RgnHandle ): OSStatus; external name '_QDSetDirtyRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDAddRectToDirtyRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function QDAddRectToDirtyRegion( inPort: CGrafPtr; const (*var*) inBounds: Rect ): OSStatus; external name '_QDAddRectToDirtyRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function QDAddRectToDirtyRegion( inPort: CGrafPtr; const (*var*) inBounds: Rect ): OSStatus; external name '_QDAddRectToDirtyRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDAddRegionToDirtyRegion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function QDAddRegionToDirtyRegion( inPort: CGrafPtr; inRegion: RgnHandle ): OSStatus; external name '_QDAddRegionToDirtyRegion';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function QDAddRegionToDirtyRegion(inPort: CGrafPtr; inRegion: RgnHandle): OSStatus; external name '_QDAddRegionToDirtyRegion';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function QDAddRegionToDirtyRegion( inPort: CGrafPtr; inRegion: RgnHandle ): OSStatus; external name '_QDAddRegionToDirtyRegion';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  CreateCGContextForPort()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateCGContextForPort( inPort: CGrafPtr; var outContext: CGContextRef ): OSStatus; external name '_CreateCGContextForPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateCGContextForPort( inPort: CGrafPtr; var outContext: CGContextRef ): OSStatus; external name '_CreateCGContextForPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  ClipCGContextToRegion()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set a CGContext's clip path to the equivalent of a RgnHandle
 *  
 *  Discussion:
 *    When Quickdraw and CoreGraphics drawing are being mixed, it is
 *    often necessary to convert a QD clipRgn to a clipPath in CG. In
 *    order to produce the expected outcome in ClipCGContextToRegion,
 *    this function needs to reset any CGContext clipPath before
 *    setting it to the converted region, in contrast to the usual
 *    behavior of CGContextClip which takes the intersection with the
 *    previous clip.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    gc:
 *      The CGContext
 *    
 *    portRect:
 *      A bounding box for the region (needed by conversion to
 *      clipPath). Can be the GrafPort bounds, or the region bounds
 *    
 *    region:
 *      The RgnHandle (usually a clipRgn) to be converted to the
 *      CGContextClip.
 *  
 *  Result:
 *    OSStatus noErr, or a memory error is some internal allocation
 *    failed.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ClipCGContextToRegion( gc: CGContextRef; const (*var*) portRect: Rect; region: RgnHandle ): OSStatus; external name '_ClipCGContextToRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ClipCGContextToRegion( gc: CGContextRef; const (*var*) portRect: Rect; region: RgnHandle ): OSStatus; external name '_ClipCGContextToRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  SyncCGContextOriginWithPort()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function SyncCGContextOriginWithPort( inContext: CGContextRef; port: CGrafPtr ): OSStatus; external name '_SyncCGContextOriginWithPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function SyncCGContextOriginWithPort( inContext: CGContextRef; port: CGrafPtr ): OSStatus; external name '_SyncCGContextOriginWithPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDBeginCGContext()
 *  
 *  Summary:
 *    Allow CoreGraphics drawing in a CGrafPort
 *  
 *  Discussion:
 *    So far, CreateCGContextForPort() was used to create a CGContext
 *    for CG drawing from a CGrafPort. However, if the current port is
 *    a printing port, the CreateCGContextForPort fails; consequently,
 *    there was no way to mix Quickdraw and CoreGraphics drawing and
 *    still print it. If, instead, the CoreGraphics drawing is
 *    bracketed by QDBeginCGContext/QDEndCGContext calls, the drawing
 *    will also appear in print. There are some restrictions on the
 *    usage of QDBeginCGContext/QDEndCGContext:
 *    - Between QDBeginCGContext and QDEndCGContext, Quickdraw drawing
 *    is disabled; only CoreGraphics drawing is allowed
 *    - QDBeginCGContext/QDEndCGContext calls can not be nested
 *    - QDEndCGContext releases the CGContext returned from
 *    QDBeginCGContext and sets it to NULL.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPort:
 *      The current port
 *    
 *    outContext:
 *      The CGContextRef to be used for CG drawing
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function QDBeginCGContext( inPort: CGrafPtr; var outContext: CGContextRef ): OSStatus; external name '_QDBeginCGContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function QDBeginCGContext( inPort: CGrafPtr; var outContext: CGContextRef ): OSStatus; external name '_QDBeginCGContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  QDEndCGContext()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function QDEndCGContext( inPort: CGrafPtr; var inoutContext: CGContextRef ): OSStatus; external name '_QDEndCGContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function QDEndCGContext( inPort: CGrafPtr; var inoutContext: CGContextRef ): OSStatus; external name '_QDEndCGContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
<<<<<<< HEAD
    The following routines are implemented in CarbonLib, and on Mac OS X in QD proper.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
    They save the pixel data of a region in a packed format for quick save/restore 
    without using a lot of memory to do a large, hollow region, such as the region
    used when drag hiliting (which is where this is used).
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

type
	QDRegionBitsRef = ^SInt32; { an opaque type }
	QDRegionBitsRefPtr = ^QDRegionBitsRef;  { when a var xx:QDRegionBitsRef parameter can be nil, it is changed to xx: QDRegionBitsRefPtr }
{$ifc not TARGET_CPU_64}
{
 *  QDSaveRegionBits()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

type
	QDRegionBitsRef = ^SInt32; { an opaque type }
	QDRegionBitsRefPtr = ^QDRegionBitsRef;  { when a var xx:QDRegionBitsRef parameter can be nil, it is changed to xx: QDRegionBitsRefPtr }
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  QDSaveRegionBits()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDSaveRegionBits()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Saves the pixel data of a region in a packed format for quick
 *    restore
 *  
 *  Discussion:
 *    Implemented in CarbonLib, and on Mac OS X in QD proper. Used in
 *    particular for drag hiliting. The packed format doesn't use too
 *    much memory for large hollow regions.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    region:
 *      The RgnHandle corresponding to the pixel data to be saved.
 *  
 *  Result:
 *    The QDRegionBitsRef to be passed later into QDRestoreRegionBits.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDSaveRegionBits( region: RgnHandle ): QDRegionBitsRef; external name '_QDSaveRegionBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDRestoreRegionBits()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDSaveRegionBits( region: RgnHandle ): QDRegionBitsRef; external name '_QDSaveRegionBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  QDRestoreRegionBits()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDRestoreRegionBits()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Restores the pixel data of a region, as returned from
 *    QDSaveRegionBits
 *  
 *  Discussion:
 *    Implemented in CarbonLib, and on Mac OS X in QD proper. Used in
 *    particular for drag hiliting. NOTE: QDRestoreRegionBits also
 *    calls QDDisposeRegionBits on the regionBits passed in - don't
 *    call it again!
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    region:
 *      The RgnHandle corresponding to the regionBits.
 *    
 *    regionBits:
 *      The QDRegionBitsRef returned from a preceding QDSaveRegionBits
 *      call
 *  
 *  Result:
 *    OSStatus noErr, or paramErr if a NULL parameter is passed in
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDRestoreRegionBits( region: RgnHandle; regionBits: QDRegionBitsRef ): OSStatus; external name '_QDRestoreRegionBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDDisposeRegionBits()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDRestoreRegionBits( region: RgnHandle; regionBits: QDRegionBitsRef ): OSStatus; external name '_QDRestoreRegionBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  QDDisposeRegionBits()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDDisposeRegionBits()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Disposes of data allocated in QDSaveRegionBits, when the
 *    QDRegionBitsRef needs to be discarded without being passed to
 *    QDRestoreRegionBits.
 *  
 *  Discussion:
 *    Implemented in CarbonLib, and on Mac OS X in QD proper. NOTE: If
 *    the QDRegionBitsRef has been passed to QDRestoreRegionBits, it
 *    has been deallocated already - don't call QDDisposeRegionBits,
 *    then.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    regionBits:
 *      The QDRegionBitsRef returned from a preceding QDSaveRegionBits
 *      call
 *  
 *  Result:
 *    OSStatus noErr, or paramErr if a NULL parameter is passed in
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDDisposeRegionBits( regionBits: QDRegionBitsRef ): OSStatus; external name '_QDDisposeRegionBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDDisposeRegionBits(regionBits: QDRegionBitsRef): OSStatus; external name '_QDDisposeRegionBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function QDDisposeRegionBits( regionBits: QDRegionBitsRef ): OSStatus; external name '_QDDisposeRegionBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
   Developers need a way to go from a CGDirectDisplay environment to Quickdraw.
   The following is equivalent to CreateNewPort(), but instead of taking the
   portPixMap from the current GDevice, it uses the GDevice corresponding to
   the CGSDisplayID passed in. If the CGSDisplayID is invalid, the mainDevice
   is used instead.
}
{
 *  CreateNewPortForCGDisplayID()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateNewPortForCGDisplayID( inCGDisplayID: UInt32 ): CGrafPtr; external name '_CreateNewPortForCGDisplayID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateNewPortForCGDisplayID( inCGDisplayID: UInt32 ): CGrafPtr; external name '_CreateNewPortForCGDisplayID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
   In Mac OS X, developers should be able to turn the WaitCursor (spinning wheel)
   on and off. QDDisplayWaitCursor() keeps track of nested calls.
   Passing FALSE will resume automatic wait cursor operation.
   Call this function only from an application in the foreground.
}
{
 *  QDDisplayWaitCursor()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure QDDisplayWaitCursor( forceWaitCursor: Boolean ); external name '_QDDisplayWaitCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDSetPatternOrigin()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure QDDisplayWaitCursor( forceWaitCursor: Boolean ); external name '_QDDisplayWaitCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  QDSetPatternOrigin()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDSetPatternOrigin()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets the pattern origin for the current port.
 *  
 *  Discussion:
 *    When a QuickDraw drawing operation uses a pattern (either a
 *    black&white pattern or a PixPat), the pattern's image is aligned
 *    with port origin, modified by the pattern origin of the port. For
 *    example, if the background pattern is a 8x8 image, and a
 *    rectangle with coordinates (3, 3, 8, 8) is filled with that
 *    pattern, then only the bottom right 5x5 portion of the pattern
 *    image will be drawn into the rectangle. When drawing a pattern,
 *    QuickDraw always starts with the port origin and then adjusts it
 *    by the pattern origin to determine the actual origin point of
 *    pattern drawing. QDSetPatternOrigin can be used to set the
 *    pattern origin relative to the port origin. It is often used in
 *    conjuction with SetOrigin to maintain the pattern alignment at
 *    (0,0) in a window's content area, regardless of the port origin;
 *    for example, after changing the port's origin to (10,10), an
 *    application might change the port's pattern origin to (-10, -10)
 *    so that patterns are still aligned with the window's content area.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    origin:
 *      The new pattern origin of the port.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in NQD 8.5 and later
 }
procedure QDSetPatternOrigin( origin: Point ); external name '_QDSetPatternOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDGetPatternOrigin()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in NQD 8.5 and later
 }
procedure QDSetPatternOrigin( origin: Point ); external name '_QDSetPatternOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  QDGetPatternOrigin()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDGetPatternOrigin()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns the pattern origin of the current port.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    origin:
 *      On exit, contains the current port's pattern origin.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in NQD 8.5 and later
 }
procedure QDGetPatternOrigin( var origin: Point ); external name '_QDGetPatternOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDIsNamedPixMapCursorRegistered()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in NQD 8.5 and later
 }
procedure QDGetPatternOrigin( var origin: Point ); external name '_QDGetPatternOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  QDIsNamedPixMapCursorRegistered()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDIsNamedPixMapCursorRegistered()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns whether a named cursor has already been registered.
 *  
 *  Discussion:
 *    The CoreGraphics Scalable Cursor Registry provides support for
 *    cursors based on PixMaps for crsrData and crsrMask, with sizes up
 *    to 64x64 pixels. Such cursors need to be registered via
 *    QDRegisterNamedPixMapCursor, and can then be set by
 *    QDSetNamedPixMapCursor.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    name:
 *      (see below at QDRegisterNamedPixMapCursor)
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDIsNamedPixMapCursorRegistered( name: ConstCStringPtr ): Boolean; external name '_QDIsNamedPixMapCursorRegistered';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDRegisterNamedPixMapCursor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  QDRegisterNamedPixMapCursor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDRegisterNamedPixMapCursor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Register a new cursor by name
 *  
 *  Discussion:
 *    In order to set a PixMapCursor, it needs to be registered first
 *    by name. This only succeeds if the system supports Hardware
 *    Cursors (depending on video circuits). There is also the obvious
 *    companion function QDUnregisterNamedPixMapCursor. NOTE: The
 *    original implementation of QDUnregisterNamedPixMapCursor was
 *    misspelled "QDUnregisterNamedPixMapCursur".
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    crsrData:
 *      A PixMapHandle representing the cursor pixels. If the
 *      pixelDepth is 32, the crsrMask PixMapHandle can be NULL; in
 *      this case, the alpha channel in the ARGB PixMap is used as
 *      mask. If a crsrMask is provided, the alpha channel in crsrData
 *      is ignored. The pixelDepth can be any of 1, 2, 4, 8, 16 or 32.
 *    
 *    crsrMask:
 *      A PixMapHandle representing the mask. It is assumed to be in
 *      8-bit deep grayScale format, although other depths are accepted
 *      and converted to 8-bit grayScale (using CopyBits). The
 *      (**crsrMask).bounds rectangle needs to match
 *      (**crsrData).bounds. If the crsrData are 32-bit deep, crsrMask
 *      can be NULL, and the alpha bytes in the crsrData ARGB pixels
 *      are used as mask.
 *    
 *    hotSpot:
 *      The usual cursor hotspot, in coordinates relativ to
 *      (**crsrData).bounds.topLeft.
 *    
 *    name:
 *      A naming convention involving the name of your application and
 *      descriptive cursor names or resource IDs is suggested. Cursor
 *      names are 0-terminated C-strings up to a length of 127. $result
 *              OSStatus: noErr = 0 for success, or (constants from
 *      MacErrors.h): kQDNoColorHWCursorSupport,
 *      kQDCursorAlreadyRegistered, paramErr, memFullErr, or a CGError
 *      as returned internally from CoreGraphics.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDRegisterNamedPixMapCursor( crsrData: PixMapHandle; crsrMask: PixMapHandle; hotSpot: Point; name: ConstCStringPtr ): OSStatus; external name '_QDRegisterNamedPixMapCursor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDUnregisterNamedPixMapCursur()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  QDUnregisterNamedPixMapCursur()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDUnregisterNamedPixMapCursur()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDUnregisterNamedPixMapCursur( name: ConstCStringPtr ): OSStatus; external name '_QDUnregisterNamedPixMapCursur';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDUnregisterNamedPixMapCursor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  QDUnregisterNamedPixMapCursor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDUnregisterNamedPixMapCursor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function QDUnregisterNamedPixMapCursor( name: ConstCStringPtr ): OSStatus; external name '_QDUnregisterNamedPixMapCursor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDSetNamedPixMapCursor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  QDSetNamedPixMapCursor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDSetNamedPixMapCursor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDSetNamedPixMapCursor( name: ConstCStringPtr ): OSStatus; external name '_QDSetNamedPixMapCursor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

const
	kQDXArrowCursor = 0;
	kQDXIBeamCursor = 1;
	kQDXIBeamXORCursor = 2;
	kQDXAliasCursor = 3;
	kQDXCopyCursor = 4;
	kQDXMoveCursor = 5;
	kQDXNumberOfSystemCursors = 6;     { Must be last }


type
	QDXSystemCursorID = UInt32;
{$ifc not TARGET_CPU_64}
{
 *  QDGetCursorNameForSystemCursor()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{$endc} {not TARGET_CPU_64}

const
	kQDXArrowCursor = 0;
	kQDXIBeamCursor = 1;
	kQDXIBeamXORCursor = 2;
	kQDXAliasCursor = 3;
	kQDXCopyCursor = 4;
	kQDXMoveCursor = 5;
	kQDXNumberOfSystemCursors = 6;     { Must be last }


type
	QDXSystemCursorID = UInt32;
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  QDGetCursorNameForSystemCursor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDGetCursorNameForSystemCursor()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Return the name of one of the predefined Mac OS X system cursors
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    cursorID:
 *      UInt32 in the range 0 ... kQDXNumberOfSystemCursors - 1 (see
 *      enum above)
 *  
 *  Result:
 *    const char* name, or NULL if 'cursorID' is out of range
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function QDGetCursorNameForSystemCursor( cursorID: QDXSystemCursorID ): ConstCStringPtr; external name '_QDGetCursorNameForSystemCursor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDSetCursorScale()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  QDSetCursorScale()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDSetCursorScale()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Set a scaling factor for the cursor.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    scale:
 *      Must be at least 0.5, and integer values (1.0, 2.0, 3.0, 4.0)
 *      are recommended. The scaling factor is system-wide (applies to
 *      all apps), and is intended for use in such things as assisting
 *      the visually impaired.  The scaling factor is treated as a hint
 *      to the system, and the exact scale applied may be limited by
 *      device driver capabilities and performance considerations.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDSetCursorScale( scale: Float32 ): OSStatus; external name '_QDSetCursorScale';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDGetCursorScale()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  QDGetCursorScale()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDGetCursorScale()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function QDGetCursorScale( outScale: Float32Ptr ): OSStatus; external name '_QDGetCursorScale';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDGetCursorData()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  QDGetCursorData()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDGetCursorData()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Allocate and return a PixMapHandle with the cursor data; also
 *    return the hotSpot. The caller is responsible for calling
 *    DisposePtr((**crsrData).baseAddr) and DisposePixMap(crsrData)
 *    when done with the crsrData returned.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    contextCursor:
 *      A Boolean; if true, return data for the current context cursor,
 *      if false, return data for the currently showing global cursor.
 *    
 *    crsrData:
 *      Allocates a PixMapHandle and pixelData in baseAddr,
 *      corresponding to the cursorData. The pixelData are in 32-bit
 *      ARGB format, with the mask contained in the alpha channel. This
 *      PixMapHandle can be passed as crsrData to
 *      QDRegisterNamedPixMapCursor, above (with crsrMask = NULL). If
 *      the return result indicates an error, NULL is returned.
 *    
 *    hotSpot:
 *      Contains the cursor hotSpot, if successful.
 *  
 *  Result:
 *    noErr if successful, or whatever error is returned from lower
 *    levels otherwise.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function QDGetCursorData( contextCursor: Boolean; var crsrData: PixMapHandle; var hotSpot: Point ): OSStatus; external name '_QDGetCursorData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}
<<<<<<< HEAD

const
	kQDUseDefaultTextRendering = 0;    { Sets whatever is specified in system defaults.}
                                        { Currently sets kQDUseTrueTypeScalerGlyphs if nothing is specified.}
	kQDUseTrueTypeScalerGlyphs = 1 shl 0; { bit 0}
	kQDUseCGTextRendering = 1 shl 1; { bit 1}
	kQDUseCGTextMetrics = 1 shl 2; { bit 2}
	kQDSupportedFlags = kQDUseTrueTypeScalerGlyphs or kQDUseCGTextRendering or kQDUseCGTextMetrics;
	kQDDontChangeFlags = -1; { to request the current state, without changing anything}


{$ifc not TARGET_CPU_64}
{
 *  QDSwapTextFlags()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
>>>>>>> origin/fixes_2.4

const
	kQDUseDefaultTextRendering = 0;    { Sets whatever is specified in system defaults.}
                                        { Currently sets kQDUseTrueTypeScalerGlyphs if nothing is specified.}
	kQDUseTrueTypeScalerGlyphs = 1 shl 0; { bit 0}
	kQDUseCGTextRendering = 1 shl 1; { bit 1}
	kQDUseCGTextMetrics = 1 shl 2; { bit 2}
	kQDSupportedFlags = kQDUseTrueTypeScalerGlyphs or kQDUseCGTextRendering or kQDUseCGTextMetrics;
	kQDDontChangeFlags = -1; { to request the current state, without changing anything}


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  QDSwapTextFlags()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  QDSwapTextFlags()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns current flags and optionally sets new flags.
 *  
 *  Discussion:
 *    Currently, only the flag bits in the enum above are supported.
 *    The typical usage is UInt32 savedFlags =
 *    QDSwapTextFlags(newFlags); // ... draw text under the conditions
 *    of "newFlags" ... (void)QDSwapTextFlags(savedFlags);  // restore
 *    previous setting
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    newFlags:
 *      Use the enums above; example "kQDUseCGTextRendering |
 *      kQDUseCGTextMetrics".
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDSwapTextFlags( newFlags: UInt32 ): UInt32; external name '_QDSwapTextFlags';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDSwapPortTextFlags()   *** DEPRECATED ***
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  QDSwapPortTextFlags()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  QDSwapPortTextFlags()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Same as QDSwapTextFlags, but per GrafPort.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    port:
 *      Settings per port override any global settings. If port ==
 *      NULL, the current port is used.
 *    
 *    newFlags:
 *      As in QDSwapTextFlags, above.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function QDSwapPortTextFlags( port: CGrafPtr; newFlags: UInt32 ): UInt32; external name '_QDSwapPortTextFlags';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{
 *  QDGetCGDirectDisplayID()
 *  
 *  Summary:
 *    Return the CGDirectDisplayID corresponding to a GDHandle
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inGDevice:
 *      The GDevice handle corresponding to the device for which the
 *      CGDirectDisplayID is requested
 *  
 *  Result:
 *    The CGDirectDisplayID, or 0 if the GDHandle does not represent a
 *    display.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function QDGetCGDirectDisplayID( inGDevice: GDHandle ): CGDirectDisplayID; external name '_QDGetCGDirectDisplayID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{ 
    LowMem accessor functions previously in LowMem.h
}
{
 *  LMGetScrVRes()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetScrVRes: SInt16; external name '_LMGetScrVRes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetScrVRes()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetScrVRes( value: SInt16 ); external name '_LMSetScrVRes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetScrVRes( value: SInt16 ); external name '_LMSetScrVRes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetScrHRes()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetScrHRes: SInt16; external name '_LMGetScrHRes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetScrHRes()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetScrHRes( value: SInt16 ); external name '_LMSetScrHRes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetScrHRes( value: SInt16 ); external name '_LMSetScrHRes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetMainDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetMainDevice: GDHandle; external name '_LMGetMainDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetMainDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetMainDevice( value: GDHandle ); external name '_LMSetMainDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetMainDevice( value: GDHandle ); external name '_LMSetMainDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetDeviceList()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetDeviceList: GDHandle; external name '_LMGetDeviceList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetDeviceList()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetDeviceList( value: GDHandle ); external name '_LMSetDeviceList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetDeviceList( value: GDHandle ); external name '_LMSetDeviceList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetQDColors()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetQDColors: Handle; external name '_LMGetQDColors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetQDColors()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetQDColors( value: Handle ); external name '_LMSetQDColors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetQDColors( value: Handle ); external name '_LMSetQDColors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetWidthListHand()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetWidthListHand: Handle; external name '_LMGetWidthListHand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetWidthListHand()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetWidthListHand( value: Handle ); external name '_LMSetWidthListHand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetWidthListHand( value: Handle ); external name '_LMSetWidthListHand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetHiliteMode()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function LMGetHiliteMode: UInt8; external name '_LMGetHiliteMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function LMGetHiliteMode: UInt8; external name '_LMGetHiliteMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetHiliteMode()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetHiliteMode( value: ByteParameter ); external name '_LMSetHiliteMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetHiliteMode( value: ByteParameter ); external name '_LMSetHiliteMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetWidthPtr()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetWidthPtr: Ptr; external name '_LMGetWidthPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetWidthPtr()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetWidthPtr( value: Ptr ); external name '_LMSetWidthPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetWidthPtr( value: Ptr ); external name '_LMSetWidthPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetWidthTabHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetWidthTabHandle: Handle; external name '_LMGetWidthTabHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetWidthTabHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetWidthTabHandle( value: Handle ); external name '_LMSetWidthTabHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetWidthTabHandle( value: Handle ); external name '_LMSetWidthTabHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetLastSPExtra()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetLastSPExtra: SInt32; external name '_LMGetLastSPExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetLastSPExtra()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetLastSPExtra( value: SInt32 ); external name '_LMSetLastSPExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetLastSPExtra( value: SInt32 ); external name '_LMSetLastSPExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetLastFOND()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetLastFOND: Handle; external name '_LMGetLastFOND';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetLastFOND()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetLastFOND( value: Handle ); external name '_LMSetLastFOND';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetLastFOND( value: Handle ); external name '_LMSetLastFOND';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetFractEnable()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function LMGetFractEnable: UInt8; external name '_LMGetFractEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function LMGetFractEnable: UInt8; external name '_LMGetFractEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetFractEnable()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetFractEnable( value: ByteParameter ); external name '_LMSetFractEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetFractEnable( value: ByteParameter ); external name '_LMSetFractEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMGetTheGDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function LMGetTheGDevice: GDHandle; external name '_LMGetTheGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetTheGDevice()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetTheGDevice( value: GDHandle ); external name '_LMSetTheGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetTheGDevice( value: GDHandle ); external name '_LMSetTheGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  LMGetHiliteRGB()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMGetHiliteRGB( var hiliteRGBValue: RGBColor ); external name '_LMGetHiliteRGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMGetHiliteRGB( var hiliteRGBValue: RGBColor ); external name '_LMGetHiliteRGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetHiliteRGB()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetHiliteRGB( const (*var*) hiliteRGBValue: RGBColor ); external name '_LMSetHiliteRGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetHiliteRGB( const (*var*) hiliteRGBValue: RGBColor ); external name '_LMSetHiliteRGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  LMGetCursorNew()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function LMGetCursorNew: Boolean; external name '_LMGetCursorNew';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function LMGetCursorNew: Boolean; external name '_LMGetCursorNew';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  LMSetCursorNew()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetCursorNew( value: Boolean ); external name '_LMSetCursorNew';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure LMSetCursorNew( value: Boolean ); external name '_LMSetCursorNew';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}


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
