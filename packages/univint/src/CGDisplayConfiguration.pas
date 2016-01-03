<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CoreGraphics - CGDisplayConfiguration.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 2002-2011 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
=======
>>>>>>> graemeg/cpstrnew
   Copyright (c) 2002-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
   Copyright (c) 2002-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
   Copyright (c) 2002-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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
 *  CGDisplayConfiguration.h
 *  CoreGraphics
 *
 *  Copyright (c) 2002 Apple Computer, Inc. All rights reserved.
 *
 }
=======
{ CoreGraphics - CGDisplayConfiguration.h
   Copyright (c) 2002-2009 Apple Inc.
   All rights reserved. }
>>>>>>> origin/fixes_2.4
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit CGDisplayConfiguration;
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CGBase,CGDirectDisplay,CGColorSpace,CGErrors,CFDictionary,CGGeometry;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{ The display reconfiguration process:
   - Call `CGBeginDisplayConfiguration' to start.
   - Make all desired changes for all displays.
   - Commit the changes using `CGPerformDisplayConfiguration', or cancel
     with `CGCancelDisplayConfiguration'.

   The resulting layout will be adjusted to remove gaps or overlaps from the
   requested layout, if needed. }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGDisplayConfigRef = ^_CGDisplayConfigRef; { an opaque type }
	_CGDisplayConfigRef = record end;

{$ifc TARGET_OS_MAC}
{ Begin a new set of display configuration changes. This function creates a
   display configuration which provides a context for a set of display
   configuration changes. Use `CGCompleteDisplayConfiguration' to apply the
   changes in a single transaction. }

function CGBeginDisplayConfiguration( var config: CGDisplayConfigRef ): CGError; external name '_CGBeginDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Configure the origin of a display in global display coordinates.

   The new origin of the display is placed as close as possible to the
   requested location, without overlapping or leaving a gap between
   displays.

=======
	CGDisplayConfigRef = ^SInt32; { an opaque type }

{$ifc TARGET_OS_MAC}

{ Begin a new set of display configuration changes. This function creates a
   display configuration which provides a context for a set of display
   configuration changes. Use `CGCompleteDisplayConfiguration' to apply the
   changes in a single transaction. }

function CGBeginDisplayConfiguration( var config: CGDisplayConfigRef ): CGError; external name '_CGBeginDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Configure the origin of a display in global display coordinates.

=======
	CGDisplayConfigRef = ^SInt32; { an opaque type }

{$ifc TARGET_OS_MAC}

{ Begin a new set of display configuration changes. This function creates a
   display configuration which provides a context for a set of display
   configuration changes. Use `CGCompleteDisplayConfiguration' to apply the
   changes in a single transaction. }

function CGBeginDisplayConfiguration( var config: CGDisplayConfigRef ): CGError; external name '_CGBeginDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Configure the origin of a display in global display coordinates.

>>>>>>> graemeg/cpstrnew
=======
	CGDisplayConfigRef = ^SInt32; { an opaque type }

{$ifc TARGET_OS_MAC}

{ Begin a new set of display configuration changes. This function creates a
   display configuration which provides a context for a set of display
   configuration changes. Use `CGCompleteDisplayConfiguration' to apply the
   changes in a single transaction. }

function CGBeginDisplayConfiguration( var config: CGDisplayConfigRef ): CGError; external name '_CGBeginDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Configure the origin of a display in global display coordinates.

>>>>>>> graemeg/cpstrnew
=======
	CGDisplayConfigRef = ^SInt32; { an opaque type }

{$ifc TARGET_OS_MAC}

{ Begin a new set of display configuration changes. This function creates a
   display configuration which provides a context for a set of display
   configuration changes. Use `CGCompleteDisplayConfiguration' to apply the
   changes in a single transaction. }

function CGBeginDisplayConfiguration( var config: CGDisplayConfigRef ): CGError; external name '_CGBeginDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Configure the origin of a display in global display coordinates.

>>>>>>> origin/cpstrnew
   The new origin of the display is placed as close as possible to the
   requested location, without overlapping or leaving a gap between
   displays.

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
   Any display whose origin is not explicitly set in a reconfiguration will
   be repositioned to a location as close as possible to its current
   location without overlapping or leaving a gap between displays.

   Note that setting the origin of a display which is mirroring another
   display will remove that display from any mirroring set. }

function CGConfigureDisplayOrigin( config: CGDisplayConfigRef; display: CGDirectDisplayID; x: SInt32; y: SInt32 ): CGError; external name '_CGConfigureDisplayOrigin';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Configure the display mode of a display. The "options" field is reserved
   for future expansion; pass NULL for now.

   A display mode is a set of properties such as width, height, pixel depth,
   and refresh rate, and options such as stretched LCD panel filling.

   If you use this function to change the mode of a display in a mirroring
   set, Quartz may adjust the bounds, resolutions, and depth of the other
   displays in the set to a safe mode, with matching depth and the smallest
   enclosing size. }

function CGConfigureDisplayWithDisplayMode( config: CGDisplayConfigRef; display: CGDirectDisplayID; mode: CGDisplayModeRef; options: CFDictionaryRef ): CGError; external name '_CGConfigureDisplayWithDisplayMode';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Enable or disable stereo operation for a display.

   Note that the system normally detects the presence of a stereo window,
   and will automatically switch a display containing a stereo window to
   stereo operation. This function provides a mechanism to force a display
   to stereo operation, and to set options (such as blue line sync signal)
   when in stereo operation.

   When in stereo operation, a display may need to generate a special stereo
   sync signal as part of the video output. The sync signal consists of a
   blue line which occupies the first 25% of the last scanline for the left
   eye view, and the first 75% of the last scanline for the right eye view.
   The remainder of the scanline is black. To force the display to generate
   this sync signal, pass true for `forceBlueLine'; otherwise, pass false.

   Returns `kCGErrorSuccess' on success, or `kCGErrorRangeCheck' if the
   display does not support the stereo operation settings requested.

   On success, the display resolution, mirroring mode, and available display
   modes may change due to hardware-specific capabilities and limitations.
   You should check these settings to verify that they are appropriate for
   your application. }

function CGConfigureDisplayStereoOperation( config: CGDisplayConfigRef; display: CGDirectDisplayID; stereo: boolean_t; forceBlueLine: boolean_t ): CGError; external name '_CGConfigureDisplayStereoOperation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Make a display a mirror of a master display.

   Pass `kCGNullDirectDisplay' for the master display to disable mirroring.
   Pass `CGMainDisplayID()' for the master display to mirror the main
   display.

   Display mirroring and display matte generation are implemented either in
   hardware (preferred) or software, at the discretion of the device driver.

   - Hardware mirroring

     With hardware mirroring enabled, all drawing is directed to the primary
     display --- see CGDisplayPrimaryDisplay.

     If the device driver selects hardware matte generation, the display
     bounds and rowbytes values are adjusted to reflect the active drawable
     area.

   - Software mirroring

     In this form of mirroring, identical content is drawn into each display
     in the mirroring set. Applications that use the window system need not
     be concerned about mirroring, as the window system takes care of all
     flushing of window content to the appropriate displays.

     Applications that draw directly to the display, as with display
     capture, must make sure to draw the same content to all mirrored
     displays in a software mirror set. When drawing to software mirrored
     displays using a full screen OpenGL context (not drawing through a
     window), you should create shared OpenGL contexts for each display and
     re-render for each display.

     You can use the function `CGGetActiveDisplayList' to determine which
     displays are active, or drawable. This automatically gives your
     application the correct view of the current displays. }

function CGConfigureDisplayMirrorOfDisplay( config: CGDisplayConfigRef; display: CGDirectDisplayID; master: CGDirectDisplayID ): CGError; external name '_CGConfigureDisplayMirrorOfDisplay';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Cancel a set of display configuration changes. On return, the
   configuration is cancelled and is no longer valid. }

function CGCancelDisplayConfiguration( config: CGDisplayConfigRef ): CGError; external name '_CGCancelDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Complete a set of display configuration changes. On return, the
   configuration is no longer valid.

   A configuration change can apply for the life of an application, the life
   of a login session, or permanently. If a request is made to make a change
   permanent, and the change cannot be supported by Mac OS X user interface,
   then the configuration change lasts only for the current login session.

   A permanent configuration change also becomes the current session's
   configuration.

   When the system reverts configurations at app termination, the
   configuration reverts to the session or permanent configuration setting.

   When the system reverts configurations at session termination, the
   configuration reverts to the permanent configuration setting.

   This operation may fail if an unsupported display mode is requested, or
   if another app is running in full-screen mode. }

=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,CGBase,CGDirectDisplay,CGErrors,CFDictionary,CGGeometry;
=======
uses MacTypes,CGBase,CGDirectDisplay,CGColorSpace,CGErrors,CFDictionary,CGGeometry;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{ The display reconfiguration process:
   - Call `CGBeginDisplayConfiguration' to start.
   - Make all desired changes for all displays.
   - Commit the changes using `CGPerformDisplayConfiguration', or cancel
     with `CGCancelDisplayConfiguration'.

   The resulting layout will be adjusted to remove gaps or overlaps from the
   requested layout, if needed. }

type
	CGDisplayConfigRef = ^SInt32; { an opaque type }

{$ifc TARGET_OS_MAC}

{ Begin a new set of display configuration changes. This function creates a
   display configuration which provides a context for a set of display
   configuration changes. Use `CGCompleteDisplayConfiguration' to apply the
   changes in a single transaction. }

function CGBeginDisplayConfiguration( var config: CGDisplayConfigRef ): CGError; external name '_CGBeginDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Configure the origin of a display in global display coordinates.

   The new origin of the display is placed as close as possible to the
   requested location, without overlapping or leaving a gap between
   displays.

   Any display whose origin is not explicitly set in a reconfiguration will
   be repositioned to a location as close as possible to its current
   location without overlapping or leaving a gap between displays.

   Note that setting the origin of a display which is mirroring another
   display will remove that display from any mirroring set. }

function CGConfigureDisplayOrigin( config: CGDisplayConfigRef; display: CGDirectDisplayID; x: SInt32; y: SInt32 ): CGError; external name '_CGConfigureDisplayOrigin';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Configure the display mode of a display. The "options" field is reserved
   for future expansion; pass NULL for now.

   A display mode is a set of properties such as width, height, pixel depth,
   and refresh rate, and options such as stretched LCD panel filling.

   If you use this function to change the mode of a display in a mirroring
   set, Quartz may adjust the bounds, resolutions, and depth of the other
   displays in the set to a safe mode, with matching depth and the smallest
   enclosing size. }

function CGConfigureDisplayWithDisplayMode( config: CGDisplayConfigRef; display: CGDirectDisplayID; mode: CGDisplayModeRef; options: CFDictionaryRef ): CGError; external name '_CGConfigureDisplayWithDisplayMode';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Enable or disable stereo operation for a display.

   Note that the system normally detects the presence of a stereo window,
   and will automatically switch a display containing a stereo window to
   stereo operation. This function provides a mechanism to force a display
   to stereo operation, and to set options (such as blue line sync signal)
   when in stereo operation.

   When in stereo operation, a display may need to generate a special stereo
   sync signal as part of the video output. The sync signal consists of a
   blue line which occupies the first 25% of the last scanline for the left
   eye view, and the first 75% of the last scanline for the right eye view.
   The remainder of the scanline is black. To force the display to generate
   this sync signal, pass true for `forceBlueLine'; otherwise, pass false.

   Returns `kCGErrorSuccess' on success, or `kCGErrorRangeCheck' if the
   display does not support the stereo operation settings requested.

   On success, the display resolution, mirroring mode, and available display
   modes may change due to hardware-specific capabilities and limitations.
   You should check these settings to verify that they are appropriate for
   your application. }

function CGConfigureDisplayStereoOperation( config: CGDisplayConfigRef; display: CGDirectDisplayID; stereo: boolean_t; forceBlueLine: boolean_t ): CGError; external name '_CGConfigureDisplayStereoOperation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Make a display a mirror of a master display.

   Pass `kCGNullDirectDisplay' for the master display to disable mirroring.
   Pass `CGMainDisplayID()' for the master display to mirror the main
   display.

   Display mirroring and display matte generation are implemented either in
   hardware (preferred) or software, at the discretion of the device driver.

   - Hardware mirroring

     With hardware mirroring enabled, all drawing is directed to the primary
     display --- see CGDisplayPrimaryDisplay.

     If the device driver selects hardware matte generation, the display
     bounds and rowbytes values are adjusted to reflect the active drawable
     area.

   - Software mirroring

     In this form of mirroring, identical content is drawn into each display
     in the mirroring set. Applications that use the window system need not
     be concerned about mirroring, as the window system takes care of all
     flushing of window content to the appropriate displays.

     Applications that draw directly to the display, as with display
     capture, must make sure to draw the same content to all mirrored
     displays in a software mirror set. When drawing to software mirrored
     displays using a full screen OpenGL context (not drawing through a
     window), you should create shared OpenGL contexts for each display and
     re-render for each display.

     You can use the function `CGGetActiveDisplayList' to determine which
     displays are active, or drawable. This automatically gives your
     application the correct view of the current displays. }

function CGConfigureDisplayMirrorOfDisplay( config: CGDisplayConfigRef; display: CGDirectDisplayID; master: CGDirectDisplayID ): CGError; external name '_CGConfigureDisplayMirrorOfDisplay';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Cancel a set of display configuration changes. On return, the
   configuration is cancelled and is no longer valid. }

function CGCancelDisplayConfiguration( config: CGDisplayConfigRef ): CGError; external name '_CGCancelDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Complete a set of display configuration changes. On return, the
   configuration is no longer valid.

   A configuration change can apply for the life of an application, the life
   of a login session, or permanently. If a request is made to make a change
   permanent, and the change cannot be supported by Mac OS X user interface,
   then the configuration change lasts only for the current login session.

   A permanent configuration change also becomes the current session's
   configuration.

   When the system reverts configurations at app termination, the
   configuration reverts to the session or permanent configuration setting.

   When the system reverts configurations at session termination, the
   configuration reverts to the permanent configuration setting.

   This operation may fail if an unsupported display mode is requested, or
   if another app is running in full-screen mode. }

<<<<<<< HEAD
{
 * Perform the requested reconfigurations and discard the configRef
 *
 * A configuration change can apply for the life of an app, the life of a login session, or
 * permanently. If a request is made to make a change permanent, and the change
 * cannot be supported by the Aqua UI (resolution and pixel depth constraints apply),
 * then the configuration  change is demoted to lasting the session.
 *
 * A permanent configuration change also becomes the current session's
 * configuration.
 *
 * When the system reverts confgurations at app termination, the
 * configuration always reverts to the session or permanent configuration setting.
 *
 * When the system reverts confgurations at session termination, the
 * configuration always reverts to the permanent configuration setting.
 *
 * This operation may fail if:
 *     An unsupported display mode is requested
 *     Another app is running in full-screen mode
 *
 }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kCGConfigureForAppOnly = 0;
	kCGConfigureForSession = 1;
	kCGConfigurePermanently = 2;
type
	CGConfigureOption = UInt32;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
function CGCompleteDisplayConfiguration( config: CGDisplayConfigRef; option: CGConfigureOption ): CGError; external name '_CGCompleteDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

{ Restore the permanent display configuration settings for the current
   user. }
<<<<<<< HEAD

procedure CGRestorePermanentDisplayConfiguration; external name '_CGRestorePermanentDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Display changes are reported via a callback mechanism.

   Callbacks are invoked when the app is listening for events, on the event
   processing thread, or from within the display reconfiguration function
   when in the program that is driving the reconfiguration.

   Callbacks should avoid changing display configurations, and should not
   raise exceptions or perform a non-local return such as calling longjmp().

   Before display reconfiguration, a callback fires to inform applications
   of a configuration change. The callback runs once for each on-line
   display. The flag is set to `kCGDisplayBeginConfigurationFlag'. This
   callback does not carry other per-display information, as details of how
   a reconfiguration affects a particular device rely on device-specific
   behaviors which may not be exposed by a device driver.

   After display reconfiguration, at the time the callback function is
   invoked, all display state reported by CoreGraphics, QuickDraw, and the
   Carbon Display Manager API will be up to date. This callback runs after
   the Carbon Display Manager notification callbacks. The callback runs once
   for each added, removed, and currently on-line display. Note that in the
   case of removed displays, calls into the CoreGraphics API with the
   removed display ID will fail. }
=======
=======
>>>>>>> origin/fixes_2_2
function CGCompleteDisplayConfiguration( configRef: CGDisplayConfigRef; option: CGConfigureOption ): CGError; external name '_CGCompleteDisplayConfiguration';
=======
>>>>>>> origin/fixes_2.4

procedure CGRestorePermanentDisplayConfiguration; external name '_CGRestorePermanentDisplayConfiguration';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

<<<<<<< HEAD
{
 * Applications may want to register for notifications of display changes.
 *
 * Display changes are reported via a callback mechanism.
 *
 * Callbacks are invoked when the app is listening for events,
 * on the event processing thread, or from within the display
 * reconfiguration function when in the program that is driving the
 * reconfiguration.
 *
 * Callbacks should avoid attempting to change display configurations,
 * and should not raise exceptions or perform a non-local return such as
 * calling longjmp().
 *
 * Before display reconfiguration, a callback fires to inform
 * applications of a pending configuration change. The callback runs
 * once for each on-line display.  The flags passed in are set to
 * kCGDisplayBeginConfigurationFlag.  This callback does not
 * carry other per-display information, as details of how a
 * reconfiguration affects a particular device rely on device-specific
 * behaviors which may not be exposed by a device driver.
 *
 * After display reconfiguration, at the time the callback function
 * is invoked, all display state reported by CoreGraphics, QuickDraw,
 * and the Carbon Display Manager API will be up to date.  This callback
 * runs after the Carbon Display Manager notification callbacks.
 * The callback runs once for each added, removed, and currently
 * on-line display.  Note that in the case of removed displays, calls into
 * the CoreGraphics API with the removed display ID will fail.
 }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Display changes are reported via a callback mechanism.

   Callbacks are invoked when the app is listening for events, on the event
   processing thread, or from within the display reconfiguration function
   when in the program that is driving the reconfiguration.

   Callbacks should avoid changing display configurations, and should not
   raise exceptions or perform a non-local return such as calling longjmp().

   Before display reconfiguration, a callback fires to inform applications
   of a configuration change. The callback runs once for each on-line
   display. The flag is set to `kCGDisplayBeginConfigurationFlag'. This
   callback does not carry other per-display information, as details of how
   a reconfiguration affects a particular device rely on device-specific
   behaviors which may not be exposed by a device driver.

   After display reconfiguration, at the time the callback function is
   invoked, all display state reported by CoreGraphics, QuickDraw, and the
   Carbon Display Manager API will be up to date. This callback runs after
   the Carbon Display Manager notification callbacks. The callback runs once
   for each added, removed, and currently on-line display. Note that in the
   case of removed displays, calls into the CoreGraphics API with the
   removed display ID will fail. }
>>>>>>> origin/fixes_2.4

const
	kCGDisplayBeginConfigurationFlag = 1 shl 0; { Set in pre-reconfiguration callback }
	kCGDisplayMovedFlag = 1 shl 1; { post-reconfiguration callback flag }
	kCGDisplaySetMainFlag = 1 shl 2; { post-reconfiguration callback flag }
	kCGDisplaySetModeFlag = 1 shl 3; { post-reconfiguration callback flag }
	kCGDisplayAddFlag = 1 shl 4; { post-reconfiguration callback flag }
	kCGDisplayRemoveFlag = 1 shl 5; { post-reconfiguration callback flag }
	kCGDisplayEnabledFlag = 1 shl 8; { post-reconfiguration callback flag }
	kCGDisplayDisabledFlag = 1 shl 9; { post-reconfiguration callback flag }
	kCGDisplayMirrorFlag = 1 shl 10;{ post-reconfiguration callback flag }
	kCGDisplayUnMirrorFlag = 1 shl 11; { post-reconfiguration callback flag }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCGDisplayDesktopShapeChangedFlag = 1 shl 12;
type
	CGDisplayChangeSummaryFlags = UInt32;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ A client-supplied callback function thatís invoked whenever the
=======
{ A client-supplied callback function that’s invoked whenever the
>>>>>>> graemeg/cpstrnew
=======
{ A client-supplied callback function that’s invoked whenever the
>>>>>>> graemeg/cpstrnew
=======
{ A client-supplied callback function that’s invoked whenever the
>>>>>>> graemeg/cpstrnew
=======
{ A client-supplied callback function that’s invoked whenever the
>>>>>>> origin/cpstrnew
   configuration of a local display is changed. }

type
	CGDisplayReconfigurationCallBack = procedure( display: CGDirectDisplayID; flags: CGDisplayChangeSummaryFlags; userInfo: UnivPtr );

{ Register a display reconfiguration callback procedure. The `userInfo'
   argument is passed back to the callback procedure each time it is
   invoked. }

function CGDisplayRegisterReconfigurationCallback( callback: CGDisplayReconfigurationCallBack; userInfo: UnivPtr ): CGError; external name '_CGDisplayRegisterReconfigurationCallback';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Remove a display reconfiguration callback procedure. }

function CGDisplayRemoveReconfigurationCallback( callback: CGDisplayReconfigurationCallBack; userInfo: UnivPtr ): CGError; external name '_CGDisplayRemoveReconfigurationCallback';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Specialized configuration changes should be done outside of the scope of
   a `CGBeginDisplayConfiguration'/`CGCompleteDisplayConfiguration' pair, as
   they may alter things such as the available display modes which a normal
   reconfiguration sequence might assume are invariant. }

{ Immediately enable or disable stereo operation for a display.

   Note that the system normally detects the presence of a stereo window,
   and will automatically switch a display containing a stereo window to
   stereo operation. This function provides a mechanism to force a display
   to stereo operation, and to set options (such as blue line sync signal)
   when in stereo operation.

   When in stereo operation, a display may need to generate a special stereo
   sync signal as part of the video output. The sync signal consists of a
   blue line which occupies the first 25% of the last scanline for the left
   eye view, and the first 75% of the last scanline for the right eye view.
   The remainder of the scanline is black. To force the display to generate
   this sync signal, pass true for `forceBlueLine'; otherwise, pass false.

   Returns `kCGErrorSuccess' on success, or `kCGErrorRangeCheck' if the
   display does not support the stereo operation settings requested.

   On success, the display resolution, mirroring mode, and available display
   modes may change due to hardware-specific capabilities and limitations.
   You should check these settings to verify that they are appropriate for
   your application. }

function CGDisplaySetStereoOperation( display: CGDirectDisplayID; stereo: boolean_t; forceBlueLine: boolean_t; option: CGConfigureOption ): CGError; external name '_CGDisplaySetStereoOperation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return true if the display is connected, awake, and available for
   drawing; false otherwise. }

function CGDisplayIsActive( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsActive';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is asleep (and is therefore not drawable);
   false otherwise. }

function CGDisplayIsAsleep( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsAsleep';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is connected or online; false otherwise. }

function CGDisplayIsOnline( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsOnline';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is the current main display; false
   otherwise. }

function CGDisplayIsMain( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsMain';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is built-in, such as the internal display in
   portable systems; false otherwise. }

function CGDisplayIsBuiltin( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsBuiltin';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is in a mirroring set; false otherwise. }

function CGDisplayIsInMirrorSet( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsInMirrorSet';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is always in a mirroring set and cannot be
   unmirrored; false otherwise. }

function CGDisplayIsAlwaysInMirrorSet( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsAlwaysInMirrorSet';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is in a hardware mirroring set; false
   otherwise. }

function CGDisplayIsInHWMirrorSet( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsInHWMirrorSet';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ For a secondary display in a mirror set, return the display being
   mirrored (the primary display), or `kCGNullDirectDisplay' if the display
   is the primary display or if the display is unmirrored. }

function CGDisplayMirrorsDisplay( display: CGDirectDisplayID ): CGDirectDisplayID; external name '_CGDisplayMirrorsDisplay';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is using OpenGL acceleration; false
   otherwise. }

function CGDisplayUsesOpenGLAcceleration( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayUsesOpenGLAcceleration';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is running in a stereo graphics mode; false
   otherwise. }

function CGDisplayIsStereo( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsStereo';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *) { 10.4.3 and later. }

{ Return the primary display in a hardware mirror set, or `display' if the
   display is not hardware-mirrored. }

function CGDisplayPrimaryDisplay( display: CGDirectDisplayID ): CGDirectDisplayID; external name '_CGDisplayPrimaryDisplay';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the logical unit number of a display. }

function CGDisplayUnitNumber( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayUnitNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the vendor number of a display's monitor. }

function CGDisplayVendorNumber( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayVendorNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the model number of a display's monitor. }

function CGDisplayModelNumber( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayModelNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the serial number of a display's monitor. }

function CGDisplaySerialNumber( display: CGDirectDisplayID ): UInt32; external name '_CGDisplaySerialNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the IOKit service port of a display. }
// uncomment when IOKit translated:
// function CGDisplayIOServicePort( display: CGDirectDisplayID ): io_service_t;
// CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA);

{ Return the width and height of a display in millimeters.

   If 'display' is not a valid display ID, the size returned has a width and
   height of 0.

   If Extended Display Identification Data (EDID) for the display device is
   not available, the size is estimated based on the device width and height
   in pixels from `CGDisplayBounds', with an assumed resolution of 2.835
   pixels/mm, or 72 DPI, a reasonable guess for displays predating EDID
   support. }

function CGDisplayScreenSize( display: CGDirectDisplayID ): CGSize; external name '_CGDisplayScreenSize';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Return the rotation angle of a display in degrees clockwise.

   A display rotation of 90∞ implies the display is rotated clockwise 90∞,
   such that what was the physical bottom of the display is now the left
   side, and what was the physical top is now the right side.

   If `display' is not a valid display ID, the rotation returned is 0. }

function CGDisplayRotation( display: CGDirectDisplayID ): Float64; external name '_CGDisplayRotation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

<<<<<<< HEAD
=======
{ Returns the IOKit service port for a display device }
// uncomment when IOKit translated:
// function CGDisplayIOServicePort( display: CGDirectDisplayID ): io_service_t;
// CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA);

{ Return the width and height of a display in millimeters.

   If 'display' is not a valid display ID, the size returned has a width and
   height of 0.

   If Extended Display Identification Data (EDID) for the display device is
   not available, the size is estimated based on the device width and height
   in pixels from `CGDisplayBounds', with an assumed resolution of 2.835
   pixels/mm, or 72 DPI, a reasonable guess for displays predating EDID
   support. }

function CGDisplayScreenSize( display: CGDirectDisplayID ): CGSize; external name '_CGDisplayScreenSize';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Return the rotation angle of a display in degrees clockwise.

   A display rotation of 90° implies the display is rotated clockwise 90°,
   such that what was the physical bottom of the display is now the left
   side, and what was the physical top is now the right side.

   If `display' is not a valid display ID, the rotation returned is 0. }

function CGDisplayRotation( display: CGDirectDisplayID ): Float64; external name '_CGDisplayRotation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

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
{ Return the color space of a display. }

function CGDisplayCopyColorSpace( display: CGDirectDisplayID ): CGColorSpaceRef; external name '_CGDisplayCopyColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ These functions are deprecated; do not use them. }

{
 * Set the display mode
 *
 * The mode dictionary passed in must be a dictionary vended by other CGDirectDisplay
 * APIs such as CGDisplayBestModeForParameters() and CGDisplayAvailableModes().
 *
 * When changing display modes of displays in a mirroring set, other displays in
 * the mirroring set whose mode is not explicitly changed will be set to a display
 * mode capable of mirroring the bounds of the largest display being explicitly set. 
 }

function CGConfigureDisplayMode( config: CGDisplayConfigRef; display: CGDirectDisplayID; mode: CFDictionaryRef ): CGError; external name '_CGConfigureDisplayMode';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6,__IPHONE_NA, __IPHONE_NA) *)

{$endc}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	kCGDisplayDesktopShapeChangedFlag = 1 shl 12;
>>>>>>> origin/fixes_2.4
type
	CGDisplayChangeSummaryFlags = UInt32;

{ A client-supplied callback function that’s invoked whenever the
   configuration of a local display is changed. }

type
	CGDisplayReconfigurationCallBack = procedure( display: CGDirectDisplayID; flags: CGDisplayChangeSummaryFlags; userInfo: UnivPtr );

{ Register a display reconfiguration callback procedure. The `userInfo'
   argument is passed back to the callback procedure each time it is
   invoked. }

function CGDisplayRegisterReconfigurationCallback( callback: CGDisplayReconfigurationCallBack; userInfo: UnivPtr ): CGError; external name '_CGDisplayRegisterReconfigurationCallback';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Remove a display reconfiguration callback procedure. }

function CGDisplayRemoveReconfigurationCallback( callback: CGDisplayReconfigurationCallBack; userInfo: UnivPtr ): CGError; external name '_CGDisplayRemoveReconfigurationCallback';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Specialized configuration changes should be done outside of the scope of
   a `CGBeginDisplayConfiguration'/`CGCompleteDisplayConfiguration' pair, as
   they may alter things such as the available display modes which a normal
   reconfiguration sequence might assume are invariant. }

{ Immediately enable or disable stereo operation for a display.

   Note that the system normally detects the presence of a stereo window,
   and will automatically switch a display containing a stereo window to
   stereo operation. This function provides a mechanism to force a display
   to stereo operation, and to set options (such as blue line sync signal)
   when in stereo operation.

   When in stereo operation, a display may need to generate a special stereo
   sync signal as part of the video output. The sync signal consists of a
   blue line which occupies the first 25% of the last scanline for the left
   eye view, and the first 75% of the last scanline for the right eye view.
   The remainder of the scanline is black. To force the display to generate
   this sync signal, pass true for `forceBlueLine'; otherwise, pass false.

   Returns `kCGErrorSuccess' on success, or `kCGErrorRangeCheck' if the
   display does not support the stereo operation settings requested.

   On success, the display resolution, mirroring mode, and available display
   modes may change due to hardware-specific capabilities and limitations.
   You should check these settings to verify that they are appropriate for
   your application. }

function CGDisplaySetStereoOperation( display: CGDirectDisplayID; stereo: boolean_t; forceBlueLine: boolean_t; option: CGConfigureOption ): CGError; external name '_CGDisplaySetStereoOperation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return true if the display is connected, awake, and available for
   drawing; false otherwise. }

function CGDisplayIsActive( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsActive';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is asleep (and is therefore not drawable);
   false otherwise. }

function CGDisplayIsAsleep( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsAsleep';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is connected or online; false otherwise. }

function CGDisplayIsOnline( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsOnline';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is the current main display; false
   otherwise. }

function CGDisplayIsMain( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsMain';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is built-in, such as the internal display in
   portable systems; false otherwise. }

function CGDisplayIsBuiltin( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsBuiltin';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is in a mirroring set; false otherwise. }

function CGDisplayIsInMirrorSet( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsInMirrorSet';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is always in a mirroring set and cannot be
   unmirrored; false otherwise. }

function CGDisplayIsAlwaysInMirrorSet( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsAlwaysInMirrorSet';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is in a hardware mirroring set; false
   otherwise. }

function CGDisplayIsInHWMirrorSet( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsInHWMirrorSet';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ For a secondary display in a mirror set, return the display being
   mirrored (the primary display), or `kCGNullDirectDisplay' if the display
   is the primary display or if the display is unmirrored. }

function CGDisplayMirrorsDisplay( display: CGDirectDisplayID ): CGDirectDisplayID; external name '_CGDisplayMirrorsDisplay';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is using OpenGL acceleration; false
   otherwise. }

function CGDisplayUsesOpenGLAcceleration( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayUsesOpenGLAcceleration';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if the display is running in a stereo graphics mode; false
   otherwise. }

function CGDisplayIsStereo( display: CGDirectDisplayID ): boolean_t; external name '_CGDisplayIsStereo';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *) { 10.4.3 and later. }

{ Return the primary display in a hardware mirror set, or `display' if the
   display is not hardware-mirrored. }

function CGDisplayPrimaryDisplay( display: CGDirectDisplayID ): CGDirectDisplayID; external name '_CGDisplayPrimaryDisplay';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the logical unit number of a display. }

function CGDisplayUnitNumber( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayUnitNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the vendor number of a display's monitor. }

function CGDisplayVendorNumber( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayVendorNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the model number of a display's monitor. }

function CGDisplayModelNumber( display: CGDirectDisplayID ): UInt32; external name '_CGDisplayModelNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return the serial number of a display's monitor. }

function CGDisplaySerialNumber( display: CGDirectDisplayID ): UInt32; external name '_CGDisplaySerialNumber';

{ Returns the IOKit service port for a display device }
// uncomment when IOKit translated:
// function CGDisplayIOServicePort( display: CGDirectDisplayID ): io_service_t;
// CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA);

{ Return the width and height of a display in millimeters.

   If 'display' is not a valid display ID, the size returned has a width and
   height of 0.

   If Extended Display Identification Data (EDID) for the display device is
   not available, the size is estimated based on the device width and height
   in pixels from `CGDisplayBounds', with an assumed resolution of 2.835
   pixels/mm, or 72 DPI, a reasonable guess for displays predating EDID
   support. }

function CGDisplayScreenSize( display: CGDirectDisplayID ): CGSize; external name '_CGDisplayScreenSize';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{ Return the rotation angle of a display in degrees clockwise.

   A display rotation of 90° implies the display is rotated clockwise 90°,
   such that what was the physical bottom of the display is now the left
   side, and what was the physical top is now the right side.

   If `display' is not a valid display ID, the rotation returned is 0. }

function CGDisplayRotation( display: CGDirectDisplayID ): Float64; external name '_CGDisplayRotation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return the color space of a display. }

function CGDisplayCopyColorSpace( display: CGDirectDisplayID ): CGColorSpaceRef; external name '_CGDisplayCopyColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ These functions are deprecated; do not use them. }

{
 * Set the display mode
 *
 * The mode dictionary passed in must be a dictionary vended by other CGDirectDisplay
 * APIs such as CGDisplayBestModeForParameters() and CGDisplayAvailableModes().
 *
 * When changing display modes of displays in a mirroring set, other displays in
 * the mirroring set whose mode is not explicitly changed will be set to a display
 * mode capable of mirroring the bounds of the largest display being explicitly set. 
 }

function CGConfigureDisplayMode( config: CGDisplayConfigRef; display: CGDirectDisplayID; mode: CFDictionaryRef ): CGError; external name '_CGConfigureDisplayMode';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6,__IPHONE_NA, __IPHONE_NA) *)

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
