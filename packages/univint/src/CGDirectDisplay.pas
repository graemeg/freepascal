<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CoreGraphics - CGDirectDisplay.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 2000-2011 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
   Copyright (c) 2000-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
   Copyright (c) 2000-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
   Copyright (c) 2000-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
   Copyright (c) 2000-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{
 *  CGDirectDisplay.h
 *  CoreGraphics
 *
 *  Copyright (c) 2000 Apple Computer, Inc. All rights reserved.
 *
 }
=======
{ CoreGraphics - CGDirectDisplay.h
   Copyright (c) 2000-2009 Apple Inc.
   All rights reserved. }
>>>>>>> origin/fixes_2.4
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit CGDirectDisplay;
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
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
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
	{$setc TARGET_CPU_ARM := FALSE}
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
<<<<<<< HEAD
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
<<<<<<< HEAD
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
uses MacTypes,CFBase,CFArray,CFDictionary,CGContext,CGBase,CGGeometry,CGErrors,CGImage;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


type
	CGDirectDisplayID = UInt32;
	CGDirectDisplayIDPtr = ^CGDirectDisplayID;  { when a var xx:CGDirectDisplayID parameter can be nil, it is changed to xx: CGDirectDisplayIDPtr }
	CGOpenGLDisplayMask = UInt32;
	CGRefreshRate = Float64;

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGDirectPaletteRef = ^OpaqueCGDirectPaletteRef; { an opaque type }
	OpaqueCGDirectPaletteRef = record end;
	CGDirectPaletteRefPtr = ^CGDirectPaletteRef;  { when a var xx:CGDirectPaletteRef parameter can be nil, it is changed to xx: CGDirectPaletteRefPtr }

type
	CGDisplayModeRef = ^OpaqueCGDisplayModeRef; { an opaque type }
	OpaqueCGDisplayModeRef = record end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	CGDirectPaletteRef = ^SInt32; { an opaque type }
	CGDirectPaletteRefPtr = ^CGDirectPaletteRef;  { when a var xx:CGDirectPaletteRef parameter can be nil, it is changed to xx: CGDirectPaletteRefPtr }

type
	CGDisplayModeRef = ^SInt32; { an opaque type }
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
	CGDisplayModeRefPtr = ^CGDisplayModeRef;

const
  kCGNullDirectDisplay = CGDirectDisplayID(0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc TARGET_OS_MAC}

{GPC-FPC-ONLY-START}
function kCGDirectMainDisplay: CGDirectDisplayID; external name '_CGMainDisplayID';
{GPC-FPC-ONLY-END}

=======

{$ifc TARGET_OS_MAC}

=======

{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======

{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======

{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
{GPC-FPC-ONLY-START}
function kCGDirectMainDisplay: CGDirectDisplayID; external name '_CGMainDisplayID';
{GPC-FPC-ONLY-END}

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
{ Return the display ID of the current main display. }

function CGMainDisplayID: CGDirectDisplayID; external name '_CGMainDisplayID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Mechanisms used to find screen IDs.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

   The following functions take an array length (`maxDisplays') and array of
   pointers to CGDirectDisplayIDs (`displays'). The array is filled in with
   the displays meeting the specified criteria; no more than `maxDisplays'.
   will be stored in `displays'. The number of displays meeting the criteria
   is returned in `matchingDisplayCount'.

   If the `displays' array is NULL, only the number of displays meeting the
   specified criteria is returned in `matchingDisplayCount'. }

function CGGetDisplaysWithPoint( point: CGPoint; maxDisplays: UInt32; displays: CGDirectDisplayIDPtr; var matchingDisplayCount: UInt32 ): CGError; external name '_CGGetDisplaysWithPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

function CGGetDisplaysWithRect( rect: CGRect; maxDisplays: UInt32; displays: CGDirectDisplayIDPtr; var matchingDisplayCount: UInt32 ): CGError; external name '_CGGetDisplaysWithRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

function CGGetDisplaysWithOpenGLDisplayMask( mask: CGOpenGLDisplayMask; maxDisplays: UInt32; displays: CGDirectDisplayIDPtr; var matchingDisplayCount: UInt32 ): CGError; external name '_CGGetDisplaysWithOpenGLDisplayMask';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
                            
{ Return a list of active displays.

=======

   The following functions take an array length (`maxDisplays') and array of
   pointers to CGDirectDisplayIDs (`displays'). The array is filled in with
   the displays meeting the specified criteria; no more than `maxDisplays'.
   will be stored in `displays'. The number of displays meeting the criteria
   is returned in `matchingDisplayCount'.

   If the `displays' array is NULL, only the number of displays meeting the
   specified criteria is returned in `matchingDisplayCount'. }

=======

   The following functions take an array length (`maxDisplays') and array of
   pointers to CGDirectDisplayIDs (`displays'). The array is filled in with
   the displays meeting the specified criteria; no more than `maxDisplays'.
   will be stored in `displays'. The number of displays meeting the criteria
   is returned in `matchingDisplayCount'.

   If the `displays' array is NULL, only the number of displays meeting the
   specified criteria is returned in `matchingDisplayCount'. }

>>>>>>> graemeg/cpstrnew
=======

   The following functions take an array length (`maxDisplays') and array of
   pointers to CGDirectDisplayIDs (`displays'). The array is filled in with
   the displays meeting the specified criteria; no more than `maxDisplays'.
   will be stored in `displays'. The number of displays meeting the criteria
   is returned in `matchingDisplayCount'.

   If the `displays' array is NULL, only the number of displays meeting the
   specified criteria is returned in `matchingDisplayCount'. }

>>>>>>> graemeg/cpstrnew
=======

   The following functions take an array length (`maxDisplays') and array of
   pointers to CGDirectDisplayIDs (`displays'). The array is filled in with
   the displays meeting the specified criteria; no more than `maxDisplays'.
   will be stored in `displays'. The number of displays meeting the criteria
   is returned in `matchingDisplayCount'.

   If the `displays' array is NULL, only the number of displays meeting the
   specified criteria is returned in `matchingDisplayCount'. }

>>>>>>> origin/cpstrnew
function CGGetDisplaysWithPoint( point: CGPoint; maxDisplays: UInt32; dspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetDisplaysWithPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

function CGGetDisplaysWithRect( rect: CGRect; maxDisplays: UInt32; dspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetDisplaysWithRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

function CGGetDisplaysWithOpenGLDisplayMask( mask: CGOpenGLDisplayMask; maxDisplays: UInt32; dspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetDisplaysWithOpenGLDisplayMask';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return a list of active displays.

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
   If `activeDisplays' is NULL, then `maxDisplays' is ignored, and
   `displayCount' is set to the number of displays. Otherwise, the list of
   active displays is stored in `activeDisplays'; no more than `maxDisplays'
   will be stored in `activeDisplays'.

   The first display returned in the list is the main display (the one with
   the menu bar). When mirroring, this will be the largest drawable display
   in the mirror set, or, if all displays are the same size, the one with
   the deepest pixel depth. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGGetActiveDisplayList( maxDisplays: UInt32; activeDisplays: CGDirectDisplayIDPtr; var displayCount: UInt32 ): CGError; external name '_CGGetActiveDisplayList';
=======
function CGGetActiveDisplayList( maxDisplays: UInt32; activeDspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetActiveDisplayList';
>>>>>>> graemeg/cpstrnew
=======
function CGGetActiveDisplayList( maxDisplays: UInt32; activeDspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetActiveDisplayList';
>>>>>>> graemeg/cpstrnew
=======
function CGGetActiveDisplayList( maxDisplays: UInt32; activeDspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetActiveDisplayList';
>>>>>>> graemeg/cpstrnew
=======
function CGGetActiveDisplayList( maxDisplays: UInt32; activeDspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetActiveDisplayList';
>>>>>>> origin/cpstrnew
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return a list of online displays.

   If `onlineDisplays' is NULL, then `maxDisplays' is ignored, and
   `displayCount' is set to the number of displays. Otherwise, the list of
   online displays is stored in `onlineDisplays'; no more than `maxDisplays'
   will be stored in `onlineDisplays'.

   With hardware mirroring, a display may be online but not necessarily
   active or drawable. Programs which manipulate display settings such as
   the palette or gamma tables need access to all displays in use, including
   hardware mirrors which are not drawable. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGGetOnlineDisplayList( maxDisplays: UInt32; onlineDisplays: CGDirectDisplayIDPtr; var displayCount: UInt32 ): CGError; external name '_CGGetOnlineDisplayList';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the OpenGL display mask for `display', or 0 if `display' is an
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function CGGetOnlineDisplayList( maxDisplays: UInt32; onlineDspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetOnlineDisplayList';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the OpenGL display mask for `display', or 0 is `display' is an
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
   invalid display. }

function CGDisplayIDToOpenGLDisplayMask( display: CGDirectDisplayID ): CGOpenGLDisplayMask; external name '_CGDisplayIDToOpenGLDisplayMask';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the display for the OpenGL display mask `mask', or
   `kCGNullDirectDisplay' if the bits set dont't match a display. A mask
   with multiple bits set returns an arbitrary match. }

function CGOpenGLDisplayMaskToDisplayID( mask: CGOpenGLDisplayMask ): CGDirectDisplayID; external name '_CGOpenGLDisplayMaskToDisplayID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the screen size and screen origin of `display' in global
   coordinates, or `CGRectZero' if `display' is invalid. }

function CGDisplayBounds( display: CGDirectDisplayID ): CGRect; external name '_CGDisplayBounds';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the width in pixels of `display'. }

function CGDisplayPixelsWide( display: CGDirectDisplayID ): size_t; external name '_CGDisplayPixelsWide';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the height in pixels of `display'. }

function CGDisplayPixelsHigh( display: CGDirectDisplayID ): size_t; external name '_CGDisplayPixelsHigh';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return an array of all modes for the specified display, or NULL if
   `display' is invalid. The "options" field is reserved for future
   expansion; pass NULL for now. }
  
function CGDisplayCopyAllDisplayModes( display: CGDirectDisplayID; options: CFDictionaryRef ): CFArrayRef; external name '_CGDisplayCopyAllDisplayModes';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCGDisplayShowDuplicateLowResolutionModes: CFStringRef; external name '_kCGDisplayShowDuplicateLowResolutionModes'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Return the current mode of the specified display, or NULL if `display'
   is invalid. }
   
function CGDisplayCopyDisplayMode( display: CGDirectDisplayID ): CGDisplayModeRef; external name '_CGDisplayCopyDisplayMode';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Switch the display mode of `display' to `mode'. The "options" field is
   reserved for future expansion; pass NULL for now.

   The selected display mode persists for the life of the program, and
   automatically reverts to the permanent setting when the program
   terminates.

   When changing display modes of displays in a mirroring set, other
   displays in the mirroring set will be set to a display mode capable of
   mirroring the bounds of the largest display being explicitly set.

   Note that after switching, display parameters and addresses may change. }

function CGDisplaySetDisplayMode( display: CGDirectDisplayID; mode: CGDisplayModeRef; options: CFDictionaryRef ): CGError; external name '_CGDisplaySetDisplayMode';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Return the width in points of the specified display mode. }
=======
{ Return the width in pixels of the specified display mode. }
>>>>>>> graemeg/cpstrnew
=======
{ Return the width in pixels of the specified display mode. }
>>>>>>> graemeg/cpstrnew
=======
{ Return the width in pixels of the specified display mode. }
>>>>>>> graemeg/cpstrnew
=======
{ Return the width in pixels of the specified display mode. }
>>>>>>> origin/cpstrnew

function CGDisplayModeGetWidth( mode: CGDisplayModeRef ): size_t; external name '_CGDisplayModeGetWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Return the height in points of the specified display mode. }
=======
{ Return the height in pixels of the specified display mode. }
>>>>>>> graemeg/cpstrnew
=======
{ Return the height in pixels of the specified display mode. }
>>>>>>> graemeg/cpstrnew
=======
{ Return the height in pixels of the specified display mode. }
>>>>>>> graemeg/cpstrnew
=======
{ Return the height in pixels of the specified display mode. }
>>>>>>> origin/cpstrnew

function CGDisplayModeGetHeight( mode: CGDisplayModeRef ): size_t; external name '_CGDisplayModeGetHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return a string representing the pixel encoding of the specified display
   mode, expressed as a CFString containing an IOKit graphics mode. }

function CGDisplayModeCopyPixelEncoding( mode: CGDisplayModeRef ): CFStringRef; external name '_CGDisplayModeCopyPixelEncoding';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the refresh rate of the specified display mode. }

function CGDisplayModeGetRefreshRate( mode: CGDisplayModeRef ): Float64; external name '_CGDisplayModeGetRefreshRate';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the IOKit flags of the specified display mode. }

function CGDisplayModeGetIOFlags( mode: CGDisplayModeRef ): UInt32; external name '_CGDisplayModeGetIOFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the IOKit display mode ID of the specified display mode. }

function CGDisplayModeGetIODisplayModeID( mode: CGDisplayModeRef ): SInt32; external name '_CGDisplayModeGetIODisplayModeID';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return true if the specified mode is usable for displaying the
   desktop GUI; false otherwise. }

function CGDisplayModeIsUsableForDesktopGUI( mode: CGDisplayModeRef ): CBool; external name '_CGDisplayModeIsUsableForDesktopGUI';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the CFTypeID for CGDisplayModeRefs. }

function CGDisplayModeGetTypeID: CFTypeID; external name '_CGDisplayModeGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Equivalent to `CFRetain(mode)', except it doesn't crash (as CFRetain
   does) if `mode' is NULL. }

function CGDisplayModeRetain( mode: CGDisplayModeRef ): CGDisplayModeRef; external name '_CGDisplayModeRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Equivalent to `CFRelease(mode)', except it doesn't crash (as CFRelease
   does) if `mode' is NULL. }

procedure CGDisplayModeRelease( mode: CGDisplayModeRef ); external name '_CGDisplayModeRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Return the width in pixels of the specified display mode. }

function CGDisplayModeGetPixelWidth( mode: CGDisplayModeRef ): size_t; external name '_CGDisplayModeGetPixelWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

{ Return the height in pixels of the specified display mode. }

function CGDisplayModeGetPixelHeight( mode: CGDisplayModeRef ): size_t; external name '_CGDisplayModeGetPixelHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Set the gamma function for `display' by specifying the coefficients of
   the gamma transfer function.

   Gamma values must be greater than 0. Minimum values must be in the
   interval [0, 1). Maximum values must be in the interval (0, 1]. Out of
   range values or maximum values greater than or equal to minimum values
   return `kCGErrorRangeCheck'.

   Values are computed by sampling a function for a range of indexes from 0
   to 1:
     value = Min + ((Max - Min) * pow(index, Gamma))
   The resulting values are converted to a machine-specific format and
   loaded into display hardware. }

type
	CGGammaValue = Float32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGGammaValuePtr = ^CGGammaValue;
=======
	CGGammaValuePtr					= ^CGGammaValue;
>>>>>>> graemeg/cpstrnew
=======
	CGGammaValuePtr					= ^CGGammaValue;
>>>>>>> graemeg/cpstrnew
=======
	CGGammaValuePtr					= ^CGGammaValue;
>>>>>>> graemeg/cpstrnew
=======
	CGGammaValuePtr					= ^CGGammaValue;
>>>>>>> origin/cpstrnew
                                              
function CGSetDisplayTransferByFormula( display: CGDirectDisplayID; redMin: CGGammaValue; redMax: CGGammaValue; redGamma: CGGammaValue; greenMin: CGGammaValue; greenMax: CGGammaValue; greenGamma: CGGammaValue; blueMin: CGGammaValue; blueMax: CGGammaValue; blueGamma: CGGammaValue ): CGError; external name '_CGSetDisplayTransferByFormula';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the coefficients of the gamma transfer function for `display'. }

function CGGetDisplayTransferByFormula( display: CGDirectDisplayID; var redMin: CGGammaValue; var redMax: CGGammaValue; var redGamma: CGGammaValue; var greenMin: CGGammaValue; var greenMax: CGGammaValue; var greenGamma: CGGammaValue; var blueMin: CGGammaValue; var blueMax: CGGammaValue; var blueGamma: CGGammaValue ): CGError; external name '_CGGetDisplayTransferByFormula';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the capacity, or number of entries, in the gamma table for
   `display', or 0 if 'display' is invalid. }

function CGDisplayGammaTableCapacity( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayGammaTableCapacity';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Set the gamma function for `display' by specifying the values in the RGB
   gamma tables.

   Values within each table should be in the interval [0, 1] The same table
   may be passed in for red, green, and blue channels. The number of entries
   in the tables is specified by `tableSize'. The tables are interpolated as
   needed to generate the number of samples needed by the display hardware. }

function CGSetDisplayTransferByTable( display: CGDirectDisplayID; tableSize: UInt32; {const var} redTable: CGGammaValuePtr; {const var} greenTable: CGGammaValuePtr; {const var} blueTable: CGGammaValuePtr ): CGError; external name '_CGSetDisplayTransferByTable';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Return the RGB gamma table values for `display'.

   The number of entries in each array is specified by `capacity'; no more
   than `capacity' entries will be written to each table. The number of
   entries written is stored in `sampleCount'. }

function CGGetDisplayTransferByTable( display: CGDirectDisplayID; capacity: UInt32; redTable: CGGammaValuePtr; greenTable: CGGammaValuePtr; blueTable: CGGammaValuePtr; var sampleCount: UInt32 ): CGError; external name '_CGGetDisplayTransferByTable';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Set the gamma function for `display' by specifying the values in the RGB
   gamma tables as bytes.

   Values within each table should be in the interval [0, 255] The same
   table may be passed in for red, green, and blue channels. The number of
   entries in the tables is specified by `tableSize'. The tables are
   interpolated as needed to generate the number of samples needed by the
   display hardware. }

function CGSetDisplayTransferByByteTable( display: CGDirectDisplayID; tableSize: UInt32; redTable: {const} UInt8Ptr; greenTable: {const} UInt8Ptr; blueTable: {const} UInt8Ptr ): CGError; external name '_CGSetDisplayTransferByByteTable';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Restore the gamma tables of all system displays to the values in the
   user's ColorSync display profile. }

procedure CGDisplayRestoreColorSyncSettings; external name '_CGDisplayRestoreColorSyncSettings';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Options used with `CGDisplayCaptureWithOptions' and
   `CGCaptureAllDisplaysWithOptions'. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kCGCaptureNoOptions = 0;	{ Default behavior. }
	kCGCaptureNoFill = 1 shl 0;	{ Disables fill with black on capture. }
type
	CGCaptureOptions = UInt32;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type
	CGCaptureOptions = UInt32;
const
	kCGCaptureNoOptions = 0;	{ Default behavior. }
	kCGCaptureNoFill = 1 shl 0;	{ Disables fill with black on capture. }
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

{ Return true if `display' is captured; false otherwise. }

function CGDisplayIsCaptured( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsCaptured';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_9, __IPHONE_NA, __IPHONE_NA) *)
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew

{ Capture `display' for exclusive use by an application. }

function CGDisplayCapture( display: CGDirectDisplayID ): CGError; external name '_CGDisplayCapture';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Capture `display' for exclusive use by an application, using the options
   specified by `options'. }

function CGDisplayCaptureWithOptions( display: CGDirectDisplayID; options: CGCaptureOptions ): CGError; external name '_CGDisplayCaptureWithOptions';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Release the captured display `display'. }

function CGDisplayRelease( display: CGDirectDisplayID ): CGError; external name '_CGDisplayRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Capture all displays. This operation provides an immersive environment
   for an appplication, and prevents other applications from trying to
   adjust to display changes. }

function CGCaptureAllDisplays: CGError; external name '_CGCaptureAllDisplays';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Capture all displays, using the options specified by `options'. This
   operation provides an immersive environment for an appplication, and
   prevents other applications from trying to adjust to display changes. }

function CGCaptureAllDisplaysWithOptions( options: CGCaptureOptions ): CGError; external name '_CGCaptureAllDisplaysWithOptions';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Release all captured displays and restore the display modes to the user's
   preferences. May be used in conjunction with `CGDisplayCapture' or
   `CGCaptureAllDisplays'. }

function CGReleaseAllDisplays: CGError; external name '_CGReleaseAllDisplays';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Returns window ID of the shield window for the captured display `display',
   or NULL if the display is not not shielded. }

function CGShieldingWindowID( display: CGDirectDisplayID ): UInt32; external name '_CGShieldingWindowID';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Returns the window level of the shield window for the captured display
   `display'. }

function CGShieldingWindowLevel: SInt32; external name '_CGShieldingWindowLevel';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return an image containing the contents of the display identified by
   `displayID'. }

function CGDisplayCreateImage( displayID: CGDirectDisplayID ): CGImageRef; external name '_CGDisplayCreateImage';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return an image containing the contents of the rectangle `rect',
   specified in display space, of the display identified by `displayID'. The
   actual rectangle used is the rectangle returned from
   `CGRectIntegral(rect)'. }

function CGDisplayCreateImageForRect( display: CGDirectDisplayID; rect: CGRect ): CGImageRef; external name '_CGDisplayCreateImageForRect';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Hide the mouse cursor and increment the hide cursor count. The `display'
   parameter is ignored. }

function CGDisplayHideCursor( display: CGDirectDisplayID ): CGError; external name '_CGDisplayHideCursor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Decrement the hide cursor count; show the cursor if the hide cursor count
   is zero. The `display' parameter is ignored. }

function CGDisplayShowCursor( display: CGDirectDisplayID ): CGError; external name '_CGDisplayShowCursor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Move the mouse cursor to the specified point relative to the origin (the
   upper-left corner) of `display'. No events are generated as a result of
   the move. Points that lie outside the desktop are clipped to the
   desktop. }

function CGDisplayMoveCursorToPoint( display: CGDirectDisplayID; point: CGPoint ): CGError; external name '_CGDisplayMoveCursorToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the mouse position change since with the last mouse move event
   received by the application. }

procedure CGGetLastMouseDelta( var deltaX: SInt32; var deltaY: SInt32 ); external name '_CGGetLastMouseDelta';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Return true if the current display mode of `display' supports palettes,
   false otherwise. }

function CGDisplayCanSetPalette( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayCanSetPalette';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Set the palette for `display' to `palette'. }

function CGDisplaySetPalette( display: CGDirectDisplayID; palette: CGDirectPaletteRef ): CGError; external name '_CGDisplaySetPalette';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Wait until the beam position moves outside the region specified by
   `upperScanLine' and `lowerScanLine' of `display'. If `upperScanLine' is
   greater than `lowerScanLine' or if `upperScanLine' and `lowerScanLine'
   encompass the entire display height, return an error.

   Some display systems may not use conventional video vertical and
   horizontal sweep. These displays report a `kCGDisplayRefreshRate' of 0 in
   the dictionary returned by `CGDisplayCurrentMode'. On such displays, this
   function returns at once. Also, some display device drivers may not
   implement support for this mechanism. On such displays, this function
   returns at once. }

function CGDisplayWaitForBeamPositionOutsideLines( display: CGDirectDisplayID; upperScanLine: UInt32; lowerScanLine: UInt32 ): CGError; external name '_CGDisplayWaitForBeamPositionOutsideLines';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the current beam position of `display', or 0 if `display' is
   invalid, if `display' does not implement conventional video vertical and
   horizontal sweep, or if the display device driver does not implement this
   functionality. }

function CGDisplayBeamPosition( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayBeamPosition';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

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
{ Return a CGContext suitable for drawing to the captured display
   `display', or NULL if `display' has not been captured. The context is
   owned by the device and should not be released by the caller.

   The context remains valid while the display is captured and while the
   display configuration is unchanged. Releasing the captured display or
   reconfiguring the display invalidates the drawing context.

   The determine when the display configuration is changing, use
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   `CGDisplayRegisterReconfigurationCallback'. }
=======
   `CGRegisterDisplayRegisterReconfigurationCallback'. }
>>>>>>> graemeg/cpstrnew
=======
   `CGRegisterDisplayRegisterReconfigurationCallback'. }
>>>>>>> graemeg/cpstrnew
=======
   `CGRegisterDisplayRegisterReconfigurationCallback'. }
>>>>>>> graemeg/cpstrnew
=======
   `CGRegisterDisplayRegisterReconfigurationCallback'. }
>>>>>>> origin/cpstrnew

function CGDisplayGetDrawingContext( display: CGDirectDisplayID ): CGContextRef; external name '_CGDisplayGetDrawingContext';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)


{
   Keys used in mode dictionaries.  Source C strings shown won't change.
   Some CFM environments cannot import data variables, and so
   the definitions are provided directly.
  
   These keys are used only within the scope of the mode dictionaries,
   so further uniquing, as by prefix, of the source string is not needed.
 }

{ These are deprecated; don't use them. }

{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayWidth CFSTRP('Width')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayHeight CFSTRP('Height')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayMode CFSTRP('Mode')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayBitsPerPixel CFSTRP('BitsPerPixel')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayBitsPerSample CFSTRP('BitsPerSample')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplaySamplesPerPixel CFSTRP('SamplesPerPixel')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayRefreshRate CFSTRP('RefreshRate')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeUsableForDesktopGUI CFSTRP('UsableForDesktopGUI')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayIOFlags CFSTRP('IOFlags')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayBytesPerRow CFSTRP('kCGDisplayBytesPerRow')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGIODisplayModeID CFSTRP('IODisplayModeID')}
{$endc}

{
 * Keys to describe optional properties of display modes.
 *
 * The key will only be present if the property applies,
 * and will be associated with a value of kCFBooleanTrue.
 * Keys not relevant to a particular display mode will not
 * appear in the mode dictionary.
 *
 * These strings must remain unchanged in future releases, of course.
 }

{ These are deprecated; don't use them. }

{ Set if display mode doesn't need a confirmation dialog to be set }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeIsSafeForHardware CFSTRP('kCGDisplayModeIsSafeForHardware')}
{$endc}

{ The following keys reflect interesting bits of the IOKit display mode flags }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeIsInterlaced CFSTRP('kCGDisplayModeIsInterlaced')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeIsStretched CFSTRP('kCGDisplayModeIsStretched')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeIsTelevisionOutput CFSTRP('kCGDisplayModeIsTelevisionOutput')}
{$endc}

{ These types are deprecated; don't use them. }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

type
	CGDisplayCount = UInt32;
	CGDisplayErr = CGError;
	CGBeamPosition = UInt32;
	CGByteValue = UInt8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    CGDisplayCoord = SInt32;
=======
  CGDisplayCoord = SInt32;
>>>>>>> graemeg/cpstrnew
=======
  CGDisplayCoord = SInt32;
>>>>>>> graemeg/cpstrnew
=======
  CGDisplayCoord = SInt32;
>>>>>>> origin/cpstrnew

type
	CGMouseDelta = SInt32;
	CGTableCount = UInt32;

const
  CGDisplayNoErr=kCGErrorSuccess;

{ These functions are deprecated; do not use them. }

{ Move the mouse cursor to the specified point relative to the origin (the
   upper-left corner) of `display'. No events are generated as a result of
   the move. Points that lie outside the desktop are clipped to the
   desktop. }

{ Use `CGDisplayCreateImage' instead. }
function CGDisplayBaseAddress( display: CGDirectDisplayID ): UnivPtr; external name '_CGDisplayBaseAddress';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6,__IPHONE_NA, __IPHONE_NA) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======

{
 * return address for X,Y in global coordinates;
 *	(0,0) represents the upper left corner of the main display.
 * returns NULL for an invalid display or out of bounds coordinates
 * If the display has not been captured, the returned address may refer
 * to read-only memory.
 }

{ Use `CGDisplayCreateImageForRect' instead. }
function CGDisplayAddressForPosition( display: CGDirectDisplayID; x: CGDisplayCoord; y: CGDisplayCoord ): UnivPtr; external name '_CGDisplayAddressForPosition';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Display mode selection
 * Display modes are represented as CFDictionaries
 * All dictionaries and arrays returned via these mechanisms are
 * owned by the framework and should not be released.  The framework
 * will not release them out from under your application.
 *
 * Values associated with the following keys are CFNumber types.
 * With CFNumberGetValue(), use kCFNumberLongType for best results.
 * kCGDisplayRefreshRate encodes a double value, so to get the fractional
 * refresh rate use kCFNumberDoubleType.
 }
 
{ Use `CGDisplayCreateImage' or `CGDisplayCreateImageForRect' instead. }
function CGDisplayBytesPerRow( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBytesPerRow';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

=======
=======
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
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
uses MacTypes,CFBase,CFArray,CFDictionary,CGContext,CGBase,CGGeometry,CGErrors;
{$ALIGN POWER}

=======
uses MacTypes,CFBase,CFArray,CFDictionary,CGContext,CGBase,CGGeometry,CGErrors,CGImage;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

{$ALIGN POWER}


type
	CGDirectDisplayID = UInt32;
	CGDirectDisplayIDPtr = ^CGDirectDisplayID;  { when a var xx:CGDirectDisplayID parameter can be nil, it is changed to xx: CGDirectDisplayIDPtr }
	CGOpenGLDisplayMask = UInt32;
	CGRefreshRate = Float64;

type
	CGDirectPaletteRef = ^SInt32; { an opaque type }
	CGDirectPaletteRefPtr = ^CGDirectPaletteRef;  { when a var xx:CGDirectPaletteRef parameter can be nil, it is changed to xx: CGDirectPaletteRefPtr }

type
	CGDisplayModeRef = ^SInt32; { an opaque type }
	CGDisplayModeRefPtr = ^CGDisplayModeRef;

const
  kCGNullDirectDisplay = CGDirectDisplayID(0);

{$ifc TARGET_OS_MAC}

{GPC-FPC-ONLY-START}
function kCGDirectMainDisplay: CGDirectDisplayID; external name '_CGMainDisplayID';
{GPC-FPC-ONLY-END}

{ Return the display ID of the current main display. }

function CGMainDisplayID: CGDirectDisplayID; external name '_CGMainDisplayID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Mechanisms used to find screen IDs.

   The following functions take an array length (`maxDisplays') and array of
   pointers to CGDirectDisplayIDs (`displays'). The array is filled in with
   the displays meeting the specified criteria; no more than `maxDisplays'.
   will be stored in `displays'. The number of displays meeting the criteria
   is returned in `matchingDisplayCount'.

   If the `displays' array is NULL, only the number of displays meeting the
   specified criteria is returned in `matchingDisplayCount'. }

function CGGetDisplaysWithPoint( point: CGPoint; maxDisplays: UInt32; dspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetDisplaysWithPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

function CGGetDisplaysWithRect( rect: CGRect; maxDisplays: UInt32; dspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetDisplaysWithRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

function CGGetDisplaysWithOpenGLDisplayMask( mask: CGOpenGLDisplayMask; maxDisplays: UInt32; dspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetDisplaysWithOpenGLDisplayMask';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return a list of active displays.

   If `activeDisplays' is NULL, then `maxDisplays' is ignored, and
   `displayCount' is set to the number of displays. Otherwise, the list of
   active displays is stored in `activeDisplays'; no more than `maxDisplays'
   will be stored in `activeDisplays'.

   The first display returned in the list is the main display (the one with
   the menu bar). When mirroring, this will be the largest drawable display
   in the mirror set, or, if all displays are the same size, the one with
   the deepest pixel depth. }

function CGGetActiveDisplayList( maxDisplays: UInt32; activeDspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetActiveDisplayList';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return a list of online displays.

   If `onlineDisplays' is NULL, then `maxDisplays' is ignored, and
   `displayCount' is set to the number of displays. Otherwise, the list of
   online displays is stored in `onlineDisplays'; no more than `maxDisplays'
   will be stored in `onlineDisplays'.

   With hardware mirroring, a display may be online but not necessarily
   active or drawable. Programs which manipulate display settings such as
   the palette or gamma tables need access to all displays in use, including
   hardware mirrors which are not drawable. }

function CGGetOnlineDisplayList( maxDisplays: UInt32; onlineDspys: CGDirectDisplayIDPtr; var dspyCnt: UInt32 ): CGError; external name '_CGGetOnlineDisplayList';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the OpenGL display mask for `display', or 0 is `display' is an
   invalid display. }

function CGDisplayIDToOpenGLDisplayMask( display: CGDirectDisplayID ): CGOpenGLDisplayMask; external name '_CGDisplayIDToOpenGLDisplayMask';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the display for the OpenGL display mask `mask', or
   `kCGNullDirectDisplay' if the bits set dont't match a display. A mask
   with multiple bits set returns an arbitrary match. }

function CGOpenGLDisplayMaskToDisplayID( mask: CGOpenGLDisplayMask ): CGDirectDisplayID; external name '_CGOpenGLDisplayMaskToDisplayID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the screen size and screen origin of `display' in global
   coordinates, or `CGRectZero' if `display' is invalid. }

function CGDisplayBounds( display: CGDirectDisplayID ): CGRect; external name '_CGDisplayBounds';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the width in pixels of `display'. }

function CGDisplayPixelsWide( display: CGDirectDisplayID ): size_t; external name '_CGDisplayPixelsWide';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the height in pixels of `display'. }

function CGDisplayPixelsHigh( display: CGDirectDisplayID ): size_t; external name '_CGDisplayPixelsHigh';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return an array of all modes for the specified display, or NULL if
   `display' is invalid. The "options" field is reserved for future
   expansion; pass NULL for now. }
  
function CGDisplayCopyAllDisplayModes( display: CGDirectDisplayID; options: CFDictionaryRef ): CFArrayRef; external name '_CGDisplayCopyAllDisplayModes';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the current mode of the specified display, or NULL if `display'
   is invalid. }
   
function CGDisplayCopyDisplayMode( display: CGDirectDisplayID ): CGDisplayModeRef; external name '_CGDisplayCopyDisplayMode';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Switch the display mode of `display' to `mode'. The "options" field is
   reserved for future expansion; pass NULL for now.

   The selected display mode persists for the life of the program, and
   automatically reverts to the permanent setting when the program
   terminates.

   When changing display modes of displays in a mirroring set, other
   displays in the mirroring set will be set to a display mode capable of
   mirroring the bounds of the largest display being explicitly set.

   Note that after switching, display parameters and addresses may change. }

function CGDisplaySetDisplayMode( display: CGDirectDisplayID; mode: CGDisplayModeRef; options: CFDictionaryRef ): CGError; external name '_CGDisplaySetDisplayMode';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the width in pixels of the specified display mode. }

function CGDisplayModeGetWidth( mode: CGDisplayModeRef ): size_t; external name '_CGDisplayModeGetWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the height in pixels of the specified display mode. }

function CGDisplayModeGetHeight( mode: CGDisplayModeRef ): size_t; external name '_CGDisplayModeGetHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return a string representing the pixel encoding of the specified display
   mode, expressed as a CFString containing an IOKit graphics mode. }

function CGDisplayModeCopyPixelEncoding( mode: CGDisplayModeRef ): CFStringRef; external name '_CGDisplayModeCopyPixelEncoding';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the refresh rate of the specified display mode. }

function CGDisplayModeGetRefreshRate( mode: CGDisplayModeRef ): Float64; external name '_CGDisplayModeGetRefreshRate';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the IOKit flags of the specified display mode. }

function CGDisplayModeGetIOFlags( mode: CGDisplayModeRef ): UInt32; external name '_CGDisplayModeGetIOFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the IOKit display mode ID of the specified display mode. }

function CGDisplayModeGetIODisplayModeID( mode: CGDisplayModeRef ): SInt32; external name '_CGDisplayModeGetIODisplayModeID';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return true if the specified mode is usable for displaying the
   desktop GUI; false otherwise. }

function CGDisplayModeIsUsableForDesktopGUI( mode: CGDisplayModeRef ): CBool; external name '_CGDisplayModeIsUsableForDesktopGUI';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the CFTypeID for CGDisplayModeRefs. }

function CGDisplayModeGetTypeID: CFTypeID; external name '_CGDisplayModeGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Equivalent to `CFRetain(mode)', except it doesn't crash (as CFRetain
   does) if `mode' is NULL. }

function CGDisplayModeRetain( mode: CGDisplayModeRef ): CGDisplayModeRef; external name '_CGDisplayModeRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Equivalent to `CFRelease(mode)', except it doesn't crash (as CFRelease
   does) if `mode' is NULL. }

procedure CGDisplayModeRelease( mode: CGDisplayModeRef ); external name '_CGDisplayModeRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Set the gamma function for `display' by specifying the coefficients of
   the gamma transfer function.

   Gamma values must be greater than 0. Minimum values must be in the
   interval [0, 1). Maximum values must be in the interval (0, 1]. Out of
   range values or maximum values greater than or equal to minimum values
   return `kCGErrorRangeCheck'.

   Values are computed by sampling a function for a range of indexes from 0
   to 1:
     value = Min + ((Max - Min) * pow(index, Gamma))
   The resulting values are converted to a machine-specific format and
   loaded into display hardware. }

type
	CGGammaValue = Float32;
	CGGammaValuePtr					= ^CGGammaValue;
                                              
function CGSetDisplayTransferByFormula( display: CGDirectDisplayID; redMin: CGGammaValue; redMax: CGGammaValue; redGamma: CGGammaValue; greenMin: CGGammaValue; greenMax: CGGammaValue; greenGamma: CGGammaValue; blueMin: CGGammaValue; blueMax: CGGammaValue; blueGamma: CGGammaValue ): CGError; external name '_CGSetDisplayTransferByFormula';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the coefficients of the gamma transfer function for `display'. }

function CGGetDisplayTransferByFormula( display: CGDirectDisplayID; var redMin: CGGammaValue; var redMax: CGGammaValue; var redGamma: CGGammaValue; var greenMin: CGGammaValue; var greenMax: CGGammaValue; var greenGamma: CGGammaValue; var blueMin: CGGammaValue; var blueMax: CGGammaValue; var blueGamma: CGGammaValue ): CGError; external name '_CGGetDisplayTransferByFormula';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the capacity, or number of entries, in the gamma table for
   `display', or 0 if 'display' is invalid. }

function CGDisplayGammaTableCapacity( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayGammaTableCapacity';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Return the RGB gamma table values for `display'.

   The number of entries in each array is specified by `capacity'; no more
   than `capacity' entries will be written to each table. The number of
   entries written is stored in `sampleCount'. }

function CGGetDisplayTransferByTable( display: CGDirectDisplayID; capacity: UInt32; redTable: CGGammaValuePtr; greenTable: CGGammaValuePtr; blueTable: CGGammaValuePtr; var sampleCount: UInt32 ): CGError; external name '_CGGetDisplayTransferByTable';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Set the gamma function for `display' by specifying the values in the RGB
   gamma tables as bytes.

   Values within each table should be in the interval [0, 255] The same
   table may be passed in for red, green, and blue channels. The number of
   entries in the tables is specified by `tableSize'. The tables are
   interpolated as needed to generate the number of samples needed by the
   display hardware. }

function CGSetDisplayTransferByByteTable( display: CGDirectDisplayID; tableSize: UInt32; redTable: {const} UInt8Ptr; greenTable: {const} UInt8Ptr; blueTable: {const} UInt8Ptr ): CGError; external name '_CGSetDisplayTransferByByteTable';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Restore the gamma tables of all system displays to the values in the
   user's ColorSync display profile. }

procedure CGDisplayRestoreColorSyncSettings; external name '_CGDisplayRestoreColorSyncSettings';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Options used with `CGDisplayCaptureWithOptions' and
   `CGCaptureAllDisplaysWithOptions'. }

type
	CGCaptureOptions = UInt32;
const
	kCGCaptureNoOptions = 0;	{ Default behavior. }
	kCGCaptureNoFill = 1 shl 0;	{ Disables fill with black on capture. }

{ Return true if `display' is captured; false otherwise. }

function CGDisplayIsCaptured( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsCaptured';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Capture `display' for exclusive use by an application. }

function CGDisplayCapture( display: CGDirectDisplayID ): CGError; external name '_CGDisplayCapture';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Capture `display' for exclusive use by an application, using the options
   specified by `options'. }

function CGDisplayCaptureWithOptions( display: CGDirectDisplayID; options: CGCaptureOptions ): CGError; external name '_CGDisplayCaptureWithOptions';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Release the captured display `display'. }

function CGDisplayRelease( display: CGDirectDisplayID ): CGError; external name '_CGDisplayRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Capture all displays. This operation provides an immersive environment
   for an appplication, and prevents other applications from trying to
   adjust to display changes. }

function CGCaptureAllDisplays: CGError; external name '_CGCaptureAllDisplays';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Capture all displays, using the options specified by `options'. This
   operation provides an immersive environment for an appplication, and
   prevents other applications from trying to adjust to display changes. }

function CGCaptureAllDisplaysWithOptions( options: CGCaptureOptions ): CGError; external name '_CGCaptureAllDisplaysWithOptions';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Release all captured displays and restore the display modes to the user's
   preferences. May be used in conjunction with `CGDisplayCapture' or
   `CGCaptureAllDisplays'. }

function CGReleaseAllDisplays: CGError; external name '_CGReleaseAllDisplays';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Returns window ID of the shield window for the captured display `display',
   or NULL if the display is not not shielded. }

function CGShieldingWindowID( display: CGDirectDisplayID ): UInt32; external name '_CGShieldingWindowID';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Returns the window level of the shield window for the captured display
   `display'. }

function CGShieldingWindowLevel: SInt32; external name '_CGShieldingWindowLevel';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return an image containing the contents of the display identified by
   `displayID'. }

function CGDisplayCreateImage( displayID: CGDirectDisplayID ): CGImageRef; external name '_CGDisplayCreateImage';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return an image containing the contents of the rectangle `rect',
   specified in display space, of the display identified by `displayID'. The
   actual rectangle used is the rectangle returned from
   `CGRectIntegral(rect)'. }

function CGDisplayCreateImageForRect( display: CGDirectDisplayID; rect: CGRect ): CGImageRef; external name '_CGDisplayCreateImageForRect';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Hide the mouse cursor and increment the hide cursor count. The `display'
   parameter is ignored. }

function CGDisplayHideCursor( display: CGDirectDisplayID ): CGError; external name '_CGDisplayHideCursor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Decrement the hide cursor count; show the cursor if the hide cursor count
   is zero. The `display' parameter is ignored. }

function CGDisplayShowCursor( display: CGDirectDisplayID ): CGError; external name '_CGDisplayShowCursor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Move the mouse cursor to the specified point relative to the origin (the
   upper-left corner) of `display'. No events are generated as a result of
   the move. Points that lie outside the desktop are clipped to the
   desktop. }

function CGDisplayMoveCursorToPoint( display: CGDirectDisplayID; point: CGPoint ): CGError; external name '_CGDisplayMoveCursorToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the mouse position change since with the last mouse move event
   received by the application. }

procedure CGGetLastMouseDelta( var deltaX: SInt32; var deltaY: SInt32 ); external name '_CGGetLastMouseDelta';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return true if the current display mode of `display' supports palettes,
   false otherwise. }

function CGDisplayCanSetPalette( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayCanSetPalette';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Set the palette for `display' to `palette'. }

function CGDisplaySetPalette( display: CGDirectDisplayID; palette: CGDirectPaletteRef ): CGError; external name '_CGDisplaySetPalette';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Wait until the beam position moves outside the region specified by
   `upperScanLine' and `lowerScanLine' of `display'. If `upperScanLine' is
   greater than `lowerScanLine' or if `upperScanLine' and `lowerScanLine'
   encompass the entire display height, return an error.

   Some display systems may not use conventional video vertical and
   horizontal sweep. These displays report a `kCGDisplayRefreshRate' of 0 in
   the dictionary returned by `CGDisplayCurrentMode'. On such displays, this
   function returns at once. Also, some display device drivers may not
   implement support for this mechanism. On such displays, this function
   returns at once. }

function CGDisplayWaitForBeamPositionOutsideLines( display: CGDirectDisplayID; upperScanLine: UInt32; lowerScanLine: UInt32 ): CGError; external name '_CGDisplayWaitForBeamPositionOutsideLines';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return the current beam position of `display', or 0 if `display' is
   invalid, if `display' does not implement conventional video vertical and
   horizontal sweep, or if the display device driver does not implement this
   functionality. }

function CGDisplayBeamPosition( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayBeamPosition';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Return a CGContext suitable for drawing to the captured display
   `display', or NULL if `display' has not been captured. The context is
   owned by the device and should not be released by the caller.

   The context remains valid while the display is captured and while the
   display configuration is unchanged. Releasing the captured display or
   reconfiguring the display invalidates the drawing context.

   The determine when the display configuration is changing, use
   `CGRegisterDisplayRegisterReconfigurationCallback'. }

function CGDisplayGetDrawingContext( display: CGDirectDisplayID ): CGContextRef; external name '_CGDisplayGetDrawingContext';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)


{
   Keys used in mode dictionaries.  Source C strings shown won't change.
   Some CFM environments cannot import data variables, and so
   the definitions are provided directly.
  
   These keys are used only within the scope of the mode dictionaries,
   so further uniquing, as by prefix, of the source string is not needed.
 }

{ These are deprecated; don't use them. }

{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayWidth CFSTRP('Width')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayHeight CFSTRP('Height')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayMode CFSTRP('Mode')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayBitsPerPixel CFSTRP('BitsPerPixel')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayBitsPerSample CFSTRP('BitsPerSample')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplaySamplesPerPixel CFSTRP('SamplesPerPixel')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayRefreshRate CFSTRP('RefreshRate')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeUsableForDesktopGUI CFSTRP('UsableForDesktopGUI')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayIOFlags CFSTRP('IOFlags')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayBytesPerRow CFSTRP('kCGDisplayBytesPerRow')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGIODisplayModeID CFSTRP('IODisplayModeID')}
{$endc}

{
 * Keys to describe optional properties of display modes.
 *
 * The key will only be present if the property applies,
 * and will be associated with a value of kCFBooleanTrue.
 * Keys not relevant to a particular display mode will not
 * appear in the mode dictionary.
 *
 * These strings must remain unchanged in future releases, of course.
 }

{ These are deprecated; don't use them. }

{ Set if display mode doesn't need a confirmation dialog to be set }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeIsSafeForHardware CFSTRP('kCGDisplayModeIsSafeForHardware')}
{$endc}

{ The following keys reflect interesting bits of the IOKit display mode flags }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeIsInterlaced CFSTRP('kCGDisplayModeIsInterlaced')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeIsStretched CFSTRP('kCGDisplayModeIsStretched')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGDisplayModeIsTelevisionOutput CFSTRP('kCGDisplayModeIsTelevisionOutput')}
{$endc}

{ These types are deprecated; don't use them. }

type
	CGDisplayCount = UInt32;
	CGDisplayErr = CGError;
	CGBeamPosition = UInt32;
	CGByteValue = UInt8;
  CGDisplayCoord = SInt32;

type
	CGMouseDelta = SInt32;
	CGTableCount = UInt32;

const
  CGDisplayNoErr=kCGErrorSuccess;

{ These functions are deprecated; do not use them. }

{ Move the mouse cursor to the specified point relative to the origin (the
   upper-left corner) of `display'. No events are generated as a result of
   the move. Points that lie outside the desktop are clipped to the
   desktop. }

{ Use `CGDisplayCreateImage' instead. }
function CGDisplayBaseAddress( display: CGDirectDisplayID ): UnivPtr; external name '_CGDisplayBaseAddress';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6,__IPHONE_NA, __IPHONE_NA) *)

{
 * return address for X,Y in global coordinates;
 *	(0,0) represents the upper left corner of the main display.
 * returns NULL for an invalid display or out of bounds coordinates
 * If the display has not been captured, the returned address may refer
 * to read-only memory.
 }

{ Use `CGDisplayCreateImageForRect' instead. }
function CGDisplayAddressForPosition( display: CGDirectDisplayID; x: CGDisplayCoord; y: CGDisplayCoord ): UnivPtr; external name '_CGDisplayAddressForPosition';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Display mode selection
 * Display modes are represented as CFDictionaries
 * All dictionaries and arrays returned via these mechanisms are
 * owned by the framework and should not be released.  The framework
 * will not release them out from under your application.
 *
 * Values associated with the following keys are CFNumber types.
 * With CFNumberGetValue(), use kCFNumberLongType for best results.
 * kCGDisplayRefreshRate encodes a double value, so to get the fractional
 * refresh rate use kCFNumberDoubleType.
 }
 
{ Use `CGDisplayCreateImage' or `CGDisplayCreateImageForRect' instead. }
function CGDisplayBytesPerRow( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBytesPerRow';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Return a CFArray of CFDictionaries describing all display modes.
 * Returns NULL if the display is invalid.
 }

{ Use the CGDisplayMode APIs instead. }
function CGDisplayAvailableModes( display: CGDirectDisplayID ): CFArrayRef; external name '_CGDisplayAvailableModes';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Try to find a display mode of specified depth with dimensions equal or greater than
 * specified.
 * If no depth match is found, try for the next larger depth with dimensions equal or greater
 * than specified.  If no luck, then just return the current mode.
 *
 * exactmatch, if not NULL, is set to 'true' if an exact match in width, height, and depth is found,
 * and 'false' otherwise.
 *
 * CGDisplayBestModeForParametersAndRefreshRateWithProperty searches the list, looking for
 * display modes with the specified property.  The property should be one of:
 *	kCGDisplayModeIsSafeForHardware;
 *	kCGDisplayModeIsInterlaced;
 *	kCGDisplayModeIsStretched;
 *	kCGDisplayModeIsTelevisionOutput
 *	
 * Returns NULL if display is invalid.
 }

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBestModeForParameters( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParameters';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBestModeForParametersAndRefreshRate( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; refresh: CGRefreshRate; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParametersAndRefreshRate';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBestModeForParametersAndRefreshRateWithProperty( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; refresh: CGRefreshRate; property: CFStringRef; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParametersAndRefreshRateWithProperty';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Return a CFDictionary describing the current display mode.
 * Returns NULL if display is invalid.
 }

{ Use the CGDisplayMode APIs instead. }
function CGDisplayCurrentMode( display: CGDirectDisplayID ): CFDictionaryRef; external name '_CGDisplayCurrentMode';

{
 * Switch display mode.  Note that after switching, 
 * display parameters and addresses may change.
 * The selected display mode persists for the life of the program, and automatically
 * reverts to the permanent setting made by Preferences when the program terminates.
 * The mode dictionary passed in must be a dictionary vended by other CGDirectDisplay
 * APIs such as CGDisplayBestModeForParameters() and CGDisplayAvailableModes().
 *
 * The mode dictionary passed in must be a dictionary vended by other CGDirectDisplay
 * APIs such as CGDisplayBestModeForParameters() and CGDisplayAvailableModes().
 *
 * When changing display modes of displays in a mirroring set, other displays in
 * the mirroring set will be set to a display mode capable of mirroring the bounds
 * of the largest display being explicitly set. 
 }
{ Use the CGDisplayMode APIs instead. }
function CGDisplaySwitchToMode( display: CGDirectDisplayID; mode: CFDictionaryRef ): CGError; external name '_CGDisplaySwitchToMode';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Query parameters for current mode }
{ Use the CGDisplayMode APIs instead. }
function CGDisplayBitsPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBitsPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

<<<<<<< HEAD
{
 * Set a display gamma/transfer function from a formula specifying
 * min and max values and a gamma for each channel.
 * Gamma values must be greater than 0.0.
 * To get an antigamma of 1.6, one would specify a value of (1.0 / 1.6)
 * Min values must be greater than or equal to 0.0 and less than 1.0.
 * Max values must be greater than 0.0 and less than or equal to 1.0.
 * Out of range values, or Max greater than or equal to Min result
 * in a kCGSRangeCheck error.
 *
 * Values are computed by sampling a function for a range of indices from 0 through 1:
 *	value = Min + ((Max - Min) * pow(index, Gamma))
 * The resulting values are converted to a machine specific format
 * and loaded into hardware.
 }
type
	CGGammaValue = Float32;
	CGGammaValuePtr					= ^CGGammaValue;
                                              
function CGSetDisplayTransferByFormula( display: CGDirectDisplayID; redMin: CGGammaValue; redMax: CGGammaValue; redGamma: CGGammaValue; greenMin: CGGammaValue; greenMax: CGGammaValue; greenGamma: CGGammaValue; blueMin: CGGammaValue; blueMax: CGGammaValue; blueGamma: CGGammaValue ): CGDisplayErr; external name '_CGSetDisplayTransferByFormula';
                                              
function CGGetDisplayTransferByFormula( display: CGDirectDisplayID; var redMin: CGGammaValue; var redMax: CGGammaValue; var redGamma: CGGammaValue; var greenMin: CGGammaValue; var greenMax: CGGammaValue; var greenGamma: CGGammaValue; var blueMin: CGGammaValue; var blueMax: CGGammaValue; var blueGamma: CGGammaValue ): CGDisplayErr; external name '_CGGetDisplayTransferByFormula';
{
 * Returns the capacity, or nunber of entries, in the camma table for the specified
 * display.  If 'display' is invalid, returns 0.
 }
function CGDisplayGammaTableCapacity( display: CGDirectDisplayID ): CGTableCount; external name '_CGDisplayGammaTableCapacity'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 * Set a display gamma/transfer function using tables of data for each channel.
 * Values within each table should have values in the range of 0.0 through 1.0.
 * The same table may be passed in for red, green, and blue channels. 'tableSize'
 * indicates the number of entries in each table.
 * The tables are interpolated as needed to generate the number of samples needed
 * by hardware.
 }
function CGSetDisplayTransferByTable( display: CGDirectDisplayID; tableSize: CGTableCount; redTable: {const} CGGammaValuePtr; greenTable: {const} CGGammaValuePtr; blueTable: {const} CGGammaValuePtr ): CGDisplayErr; external name '_CGSetDisplayTransferByTable';

{
 * Get transfer tables.  Capacity should contain the number of samples each
 * array can hold, and *sampleCount is filled in with the number of samples
 * actually copied in.
 }
function CGGetDisplayTransferByTable( display: CGDirectDisplayID; capacity: CGTableCount; redTable: CGGammaValuePtr; greenTable: CGGammaValuePtr; blueTable: CGGammaValuePtr; var sampleCount: CGTableCount ): CGDisplayErr; external name '_CGGetDisplayTransferByTable';

{ As a convenience, allow setting of the gamma table by byte values }
function CGSetDisplayTransferByByteTable( display: CGDirectDisplayID; tableSize: CGTableCount; redTable: {const} CGByteValuePtr; greenTable: {const} CGByteValuePtr; blueTable: {const} CGByteValuePtr ): CGDisplayErr; external name '_CGSetDisplayTransferByByteTable';

{ Restore gamma tables of system displays to the user's ColorSync specified values }
procedure CGDisplayRestoreColorSyncSettings; external name '_CGDisplayRestoreColorSyncSettings';

{
 * Options used with CGDisplayCaptureWithOptions and CGCaptureAllDisplaysWithOptions
 }
const
	kCGCaptureNoOptions = 0;	{ Default behavior }
	kCGCaptureNoFill = 1 shl 0;	{ Disables fill with black on display capture }

{ Display capture and release }
function CGDisplayIsCaptured( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsCaptured';
function CGDisplayCapture( display: CGDirectDisplayID ): CGDisplayErr; external name '_CGDisplayCapture';
function CGDisplayCaptureWithOptions( display: CGDirectDisplayID; options: CGCaptureOptions ): CGDisplayErr; external name '_CGDisplayCaptureWithOptions'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function CGDisplayRelease( display: CGDirectDisplayID ): CGDisplayErr; external name '_CGDisplayRelease';

{
 * Capture all displays; this has the nice effect of providing an immersive
 * environment, and preventing other apps from trying to adjust themselves
 * to display changes only needed by your app.
 }
function CGCaptureAllDisplays: CGDisplayErr; external name '_CGCaptureAllDisplays';
function CGCaptureAllDisplaysWithOptions( options: CGCaptureOptions ): CGDisplayErr; external name '_CGCaptureAllDisplaysWithOptions'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 * Release all captured displays, and restore the display modes to the
 * user's preferences.  May be used in conjunction with CGDisplayCapture()
 * or CGCaptureAllDisplays().
 }
function CGReleaseAllDisplays: CGDisplayErr; external name '_CGReleaseAllDisplays';

{
 * Returns CoreGraphics raw shield window ID or NULL if not shielded
 * This value may be used with drawing surface APIs.
 }
function CGShieldingWindowID( display: CGDirectDisplayID ): UnivPtr; external name '_CGShieldingWindowID';

{
 * Returns the window level used for the shield window.
 * This value may be used with Cocoa windows to position the
 * Cocoa window in the same window level as the shield window.
 }
function CGShieldingWindowLevel: SInt32; external name '_CGShieldingWindowLevel';

{
 * Returns base address of display or NULL for an invalid display.
 * If the display has not been captured, the returned address may refer
 * to read-only memory.
 }
function CGDisplayBaseAddress( display: CGDirectDisplayID ): UnivPtr; external name '_CGDisplayBaseAddress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
{
 * return address for X,Y in global coordinates;
 *	(0,0) represents the upper left corner of the main display.
 * returns NULL for an invalid display or out of bounds coordinates
 * If the display has not been captured, the returned address may refer
 * to read-only memory.
 }
<<<<<<< HEAD

<<<<<<< HEAD
{ Use `CGDisplayCreateImageForRect' instead. }
function CGDisplayAddressForPosition( display: CGDirectDisplayID; x: CGDisplayCoord; y: CGDisplayCoord ): UnivPtr; external name '_CGDisplayAddressForPosition';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Display mode selection
 * Display modes are represented as CFDictionaries
 * All dictionaries and arrays returned via these mechanisms are
 * owned by the framework and should not be released.  The framework
 * will not release them out from under your application.
 *
 * Values associated with the following keys are CFNumber types.
 * With CFNumberGetValue(), use kCFNumberLongType for best results.
 * kCGDisplayRefreshRate encodes a double value, so to get the fractional
 * refresh rate use kCFNumberDoubleType.
 }
 
{ Use `CGDisplayCreateImage' or `CGDisplayCreateImageForRect' instead. }
function CGDisplayBytesPerRow( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBytesPerRow';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew

{
 * return address for X,Y in global coordinates;
 *	(0,0) represents the upper left corner of the main display.
 * returns NULL for an invalid display or out of bounds coordinates
 * If the display has not been captured, the returned address may refer
 * to read-only memory.
 }

<<<<<<< HEAD
{ Use `CGDisplayCreateImageForRect' instead. }
function CGDisplayAddressForPosition( display: CGDirectDisplayID; x: CGDisplayCoord; y: CGDisplayCoord ): UnivPtr; external name '_CGDisplayAddressForPosition';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
=======
{ Use the CGDisplayMode APIs instead. }
function CGDisplayAvailableModes( display: CGDirectDisplayID ): CFArrayRef; external name '_CGDisplayAvailableModes';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
{ Use the CGDisplayMode APIs instead. }
function CGDisplayAvailableModes( display: CGDirectDisplayID ): CFArrayRef; external name '_CGDisplayAvailableModes';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew

{
 * Display mode selection
 * Display modes are represented as CFDictionaries
 * All dictionaries and arrays returned via these mechanisms are
 * owned by the framework and should not be released.  The framework
 * will not release them out from under your application.
 *
 * Values associated with the following keys are CFNumber types.
 * With CFNumberGetValue(), use kCFNumberLongType for best results.
 * kCGDisplayRefreshRate encodes a double value, so to get the fractional
 * refresh rate use kCFNumberDoubleType.
 }
 
{ Use `CGDisplayCreateImage' or `CGDisplayCreateImageForRect' instead. }
function CGDisplayBytesPerRow( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBytesPerRow';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew

{
 * return address for X,Y in global coordinates;
 *	(0,0) represents the upper left corner of the main display.
 * returns NULL for an invalid display or out of bounds coordinates
 * If the display has not been captured, the returned address may refer
 * to read-only memory.
 }

<<<<<<< HEAD
{ Use `CGDisplayCreateImageForRect' instead. }
function CGDisplayAddressForPosition( display: CGDirectDisplayID; x: CGDisplayCoord; y: CGDisplayCoord ): UnivPtr; external name '_CGDisplayAddressForPosition';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
=======
{ Use the CGDisplayMode APIs instead. }
function CGDisplayAvailableModes( display: CGDirectDisplayID ): CFArrayRef; external name '_CGDisplayAvailableModes';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew

{
 * Display mode selection
 * Display modes are represented as CFDictionaries
 * All dictionaries and arrays returned via these mechanisms are
 * owned by the framework and should not be released.  The framework
 * will not release them out from under your application.
 *
 * Values associated with the following keys are CFNumber types.
 * With CFNumberGetValue(), use kCFNumberLongType for best results.
 * kCGDisplayRefreshRate encodes a double value, so to get the fractional
 * refresh rate use kCFNumberDoubleType.
 }
 
{ Use `CGDisplayCreateImage' or `CGDisplayCreateImageForRect' instead. }
function CGDisplayBytesPerRow( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBytesPerRow';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Return a CFArray of CFDictionaries describing all display modes.
 * Returns NULL if the display is invalid.
 }

{ Use the CGDisplayMode APIs instead. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGDisplayAvailableModes( display: CGDirectDisplayID ): CFArrayRef; external name '_CGDisplayAvailableModes';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
function CGDisplayBestModeForParameters( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParameters';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
<<<<<<< HEAD
<<<<<<< HEAD
function CGDisplayBestModeForParametersAndRefreshRate( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; refreshRate: CGRefreshRate; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParametersAndRefreshRate';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBestModeForParametersAndRefreshRateWithProperty( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; refreshRate: CGRefreshRate; property: CFStringRef; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParametersAndRefreshRateWithProperty';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
function CGDisplayBestModeForParameters( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParameters';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
>>>>>>> origin/cpstrnew
function CGDisplayBestModeForParametersAndRefreshRate( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; refresh: CGRefreshRate; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParametersAndRefreshRate';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBestModeForParametersAndRefreshRateWithProperty( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; refresh: CGRefreshRate; property: CFStringRef; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParametersAndRefreshRateWithProperty';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Return a CFDictionary describing the current display mode.
 * Returns NULL if display is invalid.
 }

{ Use the CGDisplayMode APIs instead. }
function CGDisplayCurrentMode( display: CGDirectDisplayID ): CFDictionaryRef; external name '_CGDisplayCurrentMode';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{
 * Switch display mode.  Note that after switching, 
 * display parameters and addresses may change.
 * The selected display mode persists for the life of the program, and automatically
 * reverts to the permanent setting made by Preferences when the program terminates.
 * The mode dictionary passed in must be a dictionary vended by other CGDirectDisplay
 * APIs such as CGDisplayBestModeForParameters() and CGDisplayAvailableModes().
 *
 * The mode dictionary passed in must be a dictionary vended by other CGDirectDisplay
 * APIs such as CGDisplayBestModeForParameters() and CGDisplayAvailableModes().
 *
 * When changing display modes of displays in a mirroring set, other displays in
 * the mirroring set will be set to a display mode capable of mirroring the bounds
 * of the largest display being explicitly set. 
 }
{ Use the CGDisplayMode APIs instead. }
function CGDisplaySwitchToMode( display: CGDirectDisplayID; mode: CFDictionaryRef ): CGError; external name '_CGDisplaySwitchToMode';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Query parameters for current mode }
{ Use the CGDisplayMode APIs instead. }
function CGDisplayBitsPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBitsPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
<<<<<<< HEAD

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBitsPerSample( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBitsPerSample';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
<<<<<<< HEAD
<<<<<<< HEAD

{ Use the CGDisplayMode APIs instead. }
function CGDisplaySamplesPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplaySamplesPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

function CGDisplayCanSetPalette( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayCanSetPalette';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

function CGDisplaySetPalette( display: CGDirectDisplayID; palette: CGDirectPaletteRef ): CGError; external name '_CGDisplaySetPalette';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

function CGDisplayWaitForBeamPositionOutsideLines( display: CGDirectDisplayID; upperScanLine: UInt32; lowerScanLine: UInt32 ): CGError; external name '_CGDisplayWaitForBeamPositionOutsideLines';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

function CGDisplayBeamPosition( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayBeamPosition';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

=======

{ Use the CGDisplayMode APIs instead. }
function CGDisplaySamplesPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplaySamplesPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew

=======

{ Use the CGDisplayMode APIs instead. }
function CGDisplaySamplesPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplaySamplesPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

>>>>>>> graemeg/cpstrnew
=======

type
	CGDisplayCount = UInt32;
	CGDisplayErr = CGError;
	CGBeamPosition = UInt32;
	CGByteValue = UInt8;
  CGDisplayCoord = SInt32;

type
	CGMouseDelta = SInt32;
	CGTableCount = UInt32;

const
  CGDisplayNoErr=kCGErrorSuccess;

{ These functions are deprecated; do not use them. }

{ Move the mouse cursor to the specified point relative to the origin (the
   upper-left corner) of `display'. No events are generated as a result of
   the move. Points that lie outside the desktop are clipped to the
   desktop. }

{ Use `CGDisplayCreateImage' instead. }
function CGDisplayBaseAddress( display: CGDirectDisplayID ): UnivPtr; external name '_CGDisplayBaseAddress';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6,__IPHONE_NA, __IPHONE_NA) *)

=======
>>>>>>> origin/fixes_2_2
{
 * return address for X,Y in global coordinates;
 *	(0,0) represents the upper left corner of the main display.
 * returns NULL for an invalid display or out of bounds coordinates
 * If the display has not been captured, the returned address may refer
 * to read-only memory.
 }
<<<<<<< HEAD

{ Use `CGDisplayCreateImageForRect' instead. }
function CGDisplayAddressForPosition( display: CGDirectDisplayID; x: CGDisplayCoord; y: CGDisplayCoord ): UnivPtr; external name '_CGDisplayAddressForPosition';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Display mode selection
 * Display modes are represented as CFDictionaries
 * All dictionaries and arrays returned via these mechanisms are
 * owned by the framework and should not be released.  The framework
 * will not release them out from under your application.
 *
 * Values associated with the following keys are CFNumber types.
 * With CFNumberGetValue(), use kCFNumberLongType for best results.
 * kCGDisplayRefreshRate encodes a double value, so to get the fractional
 * refresh rate use kCFNumberDoubleType.
 }
 
{ Use `CGDisplayCreateImage' or `CGDisplayCreateImageForRect' instead. }
function CGDisplayBytesPerRow( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBytesPerRow';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Return a CFArray of CFDictionaries describing all display modes.
 * Returns NULL if the display is invalid.
 }

{ Use the CGDisplayMode APIs instead. }
function CGDisplayAvailableModes( display: CGDirectDisplayID ): CFArrayRef; external name '_CGDisplayAvailableModes';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Try to find a display mode of specified depth with dimensions equal or greater than
 * specified.
 * If no depth match is found, try for the next larger depth with dimensions equal or greater
 * than specified.  If no luck, then just return the current mode.
 *
 * exactmatch, if not NULL, is set to 'true' if an exact match in width, height, and depth is found,
 * and 'false' otherwise.
 *
 * CGDisplayBestModeForParametersAndRefreshRateWithProperty searches the list, looking for
 * display modes with the specified property.  The property should be one of:
 *	kCGDisplayModeIsSafeForHardware;
 *	kCGDisplayModeIsInterlaced;
 *	kCGDisplayModeIsStretched;
 *	kCGDisplayModeIsTelevisionOutput
 *	
 * Returns NULL if display is invalid.
 }

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBestModeForParameters( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParameters';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBestModeForParametersAndRefreshRate( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; refresh: CGRefreshRate; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParametersAndRefreshRate';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBestModeForParametersAndRefreshRateWithProperty( display: CGDirectDisplayID; bitsPerPixel: size_t; width: size_t; height: size_t; refresh: CGRefreshRate; property: CFStringRef; var exactMatch: boolean_t ): CFDictionaryRef; external name '_CGDisplayBestModeForParametersAndRefreshRateWithProperty';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{
 * Return a CFDictionary describing the current display mode.
 * Returns NULL if display is invalid.
 }

{ Use the CGDisplayMode APIs instead. }
function CGDisplayCurrentMode( display: CGDirectDisplayID ): CFDictionaryRef; external name '_CGDisplayCurrentMode';

{
 * Switch display mode.  Note that after switching, 
 * display parameters and addresses may change.
 * The selected display mode persists for the life of the program, and automatically
 * reverts to the permanent setting made by Preferences when the program terminates.
 * The mode dictionary passed in must be a dictionary vended by other CGDirectDisplay
 * APIs such as CGDisplayBestModeForParameters() and CGDisplayAvailableModes().
 *
 * The mode dictionary passed in must be a dictionary vended by other CGDirectDisplay
 * APIs such as CGDisplayBestModeForParameters() and CGDisplayAvailableModes().
 *
 * When changing display modes of displays in a mirroring set, other displays in
 * the mirroring set will be set to a display mode capable of mirroring the bounds
 * of the largest display being explicitly set. 
 }
{ Use the CGDisplayMode APIs instead. }
function CGDisplaySwitchToMode( display: CGDirectDisplayID; mode: CFDictionaryRef ): CGError; external name '_CGDisplaySwitchToMode';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Query parameters for current mode }
{ Use the CGDisplayMode APIs instead. }
function CGDisplayBitsPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBitsPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBitsPerSample( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBitsPerSample';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplaySamplesPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplaySamplesPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

>>>>>>> graemeg/cpstrnew
=======

{ Use the CGDisplayMode APIs instead. }
function CGDisplayBitsPerSample( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBitsPerSample';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{ Use the CGDisplayMode APIs instead. }
function CGDisplaySamplesPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplaySamplesPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

>>>>>>> origin/cpstrnew
{$endc}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
function CGDisplayAddressForPosition( display: CGDirectDisplayID; x: CGDisplayCoord; y: CGDisplayCoord ): UnivPtr; external name '_CGDisplayAddressForPosition';


{ Mouse Cursor controls }
function CGDisplayHideCursor( display: CGDirectDisplayID ): CGDisplayErr; external name '_CGDisplayHideCursor';	{ increments hide cursor count }
function CGDisplayShowCursor( display: CGDirectDisplayID ): CGDisplayErr; external name '_CGDisplayShowCursor';	{ decrements hide cursor count  }

{
 * Move the cursor to the specified point relative to the display origin
 * (the upper left corner of the display).  Returns CGDisplayNoErr on success.
 * No events are generated as a result of this move.
 * Points that would lie outside the desktop are clipped to the desktop.
 }
function CGDisplayMoveCursorToPoint( display: CGDirectDisplayID; point: CGPoint ): CGDisplayErr; external name '_CGDisplayMoveCursorToPoint';

{
 * Report the mouse position change associated with the last mouse move event
 * recieved by this application.
 }
procedure CGGetLastMouseDelta( var deltaX: CGMouseDelta; var deltaY: CGMouseDelta ); external name '_CGGetLastMouseDelta';


{ Palette controls (8 bit pseudocolor only) }

{
 * Returns TRUE if the current display mode supports palettes.
 * Display must not be a hardware mirror of another, and should
 * have a depth of 8 bits per pixel for this to return TRUE.
 }
function CGDisplayCanSetPalette( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayCanSetPalette';

{
 * Set a palette.  The current gamma function is applied to the palette
 * elements before being loaded into hardware.  The display must not be
 * a hardware mirror of another, and should have a depth of 8 bits per pixel.
 * Setting the palette on the active, or primary display in a hardware
 * mirroring set affects all displays in that set.
 }
function CGDisplaySetPalette( display: CGDirectDisplayID; palette: CGDirectPaletteRef ): CGDisplayErr; external name '_CGDisplaySetPalette';

{
 * Wait until the beam position is outside the range specified by upperScanLine and lowerScanLine.
 * Note that if upperScanLine and lowerScanLine encompass the entire display height,
 * the function returns an error.
 * lowerScanLine must be greater than or equal to upperScanLine.
 *
 * Some display systems may not conventional video vertical and horizontal sweep in painting.
 * These displays report a kCGDisplayRefreshRate of 0 in the CFDictionaryRef returned by
 * CGDisplayCurrentMode().  On such displays, this function returns at once.
 *
 * Some drivers may not implement support for this mechanism.
 * On such displays, this function returns at once.
 *
 * Returns CGDisplayNoErr on success, and an error if display or upperScanLine and
 * lowerScanLine are invalid.
 *
 * The app should set the values of upperScanLine and lowerScanLine to allow enough lead time
 * for the drawing operation to complete.  A common strategy is to wait for the beam to pass
 * the bottom of the drawing area, allowing almost a full vertical sweep period to perform drawing.
 * To do this, set upperScanLine to 0, and set lowerScanLine to the bottom of the bounding box:
 *	lowerScanLine = (CGBeamPosition)(cgrect.origin.y + cgrect.size.height);
 *
 * IOKit may implement this as a spin-loop on the beam position call used for CGDisplayBeamPosition().
 * On such system the function is CPU bound, and subject to all the usual scheduling pre-emption.
 * In particular, attempting to wait for the beam to hit a specific scanline may be an exercise in frustration.
 *
 * These functions are advisary in nature, and depend on IOKit and hardware specific drivers to implement
 * support. If you need extremely precise timing, or access to vertical blanking interrupts,
 * you should consider writing a device driver to tie into hardware-specific capabilities.
 }
function CGDisplayWaitForBeamPositionOutsideLines( display: CGDirectDisplayID; upperScanLine: CGBeamPosition; lowerScanLine: CGBeamPosition ): CGDisplayErr; external name '_CGDisplayWaitForBeamPositionOutsideLines';

{
 * Returns the current beam position on the display.  If display is invalid,
 * or the display does not implement conventional video vertical and horizontal
 * sweep in painting, or the driver does not implement this functionality, 0 is returned.
 }
function CGDisplayBeamPosition( display: CGDirectDisplayID ): CGBeamPosition; external name '_CGDisplayBeamPosition';
=======
{ Use the CGDisplayMode APIs instead. }
function CGDisplayBitsPerSample( display: CGDirectDisplayID ): size_t; external name '_CGDisplayBitsPerSample';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Use the CGDisplayMode APIs instead. }
function CGDisplaySamplesPerPixel( display: CGDirectDisplayID ): size_t; external name '_CGDisplaySamplesPerPixel';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6, __IPHONE_NA, __IPHONE_NA) *)

{$endc}
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
