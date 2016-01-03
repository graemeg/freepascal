{ CoreGraphics - CGColor.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2003-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
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
 * Copyright (c) 2003 Apple Computer, Inc.
 * All rights reserved.
 }
=======
 * Copyright (c) 2003-2008 Apple Inc.
 * All rights reserved. }
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

unit CGColor;
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
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
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
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
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
uses MacTypes,CFBase,CGBase,CGColorSpace;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


// CGColorRef defined in CGBase


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Create a color in the color space `space' with color components
   (including alpha) specified by `components'. `space' may be any color
   space except a pattern color space. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function CGColorCreate( space: CGColorSpaceRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
=======

function CGColorCreate( space: CGColorSpaceRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreate'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
{ Create a color in the "Generic" gray color space. }
=======

function CGColorCreate( space: CGColorSpaceRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreate'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$ifc TARGET_OS_MAC}
{ Create a color in the "Generic" gray color space. }

function CGColorCreateGenericGray(gray: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericGray'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Create a color in the "Generic" RGB color space. }

function CGColorCreateGenericRGB(red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericRGB'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Create a color in the "Generic" CMYK color space. }

function CGColorCreateGenericCMYK(cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericCMYK'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{$endc}

{ Return a constant color. As `CGColorGetConstantColor' is not a "Copy" or
   "Create" function, it does not necessarily return a new reference each
   time it's called. As a consequence, you should not release the returned
   value. However, colors returned from `CGColorGetConstantColor' can be
   retained and released in a properly nested fashion, just like any other
   CF type. }

function CGColorGetConstantColor(colorName: CFStringRef): CGColorRef; external name '_CGColorGetConstantColor'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew

<<<<<<< HEAD
function CGColorCreateGenericGray( gray: CGFloat; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateGenericGray';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
=======
function CGColorCreateGenericGray(gray: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericGray'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

<<<<<<< HEAD
{ Create a color in the "Generic" RGB color space. }

function CGColorCreateGenericRGB(red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericRGB'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Create a color in the "Generic" CMYK color space. }

function CGColorCreateGenericCMYK(cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericCMYK'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{$endc}

{ Return a constant color. As `CGColorGetConstantColor' is not a "Copy" or
   "Create" function, it does not necessarily return a new reference each
   time it's called. As a consequence, you should not release the returned
   value. However, colors returned from `CGColorGetConstantColor' can be
   retained and released in a properly nested fashion, just like any other
   CF type. }

function CGColorGetConstantColor(colorName: CFStringRef): CGColorRef; external name '_CGColorGetConstantColor'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{ Create a color in the "Generic" RGB color space. }

<<<<<<< HEAD
function CGColorCreateGenericRGB( red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateGenericRGB';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Create a color in the "Generic" CMYK color space. }

function CGColorCreateGenericCMYK( cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateGenericCMYK';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

=======

=======

>>>>>>> graemeg/cpstrnew
function CGColorCreate( space: CGColorSpaceRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreate'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$ifc TARGET_OS_MAC}
{ Create a color in the "Generic" gray color space. }

function CGColorCreateGenericGray(gray: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericGray'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
<<<<<<< HEAD
=======

{ Create a color in the "Generic" RGB color space. }

function CGColorCreateGenericRGB(red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericRGB'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Create a color in the "Generic" CMYK color space. }

function CGColorCreateGenericCMYK(cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericCMYK'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{$endc}

{ Return a constant color. As `CGColorGetConstantColor' is not a "Copy" or
   "Create" function, it does not necessarily return a new reference each
   time it's called. As a consequence, you should not release the returned
   value. However, colors returned from `CGColorGetConstantColor' can be
   retained and released in a properly nested fashion, just like any other
   CF type. }

function CGColorGetConstantColor(colorName: CFStringRef): CGColorRef; external name '_CGColorGetConstantColor'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{ Create a color in the "Generic" RGB color space. }

<<<<<<< HEAD
function CGColorCreateGenericRGB(red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericRGB'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Create a color in the "Generic" CMYK color space. }

function CGColorCreateGenericCMYK(cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericCMYK'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{$endc}

>>>>>>> graemeg/cpstrnew
{ Return a constant color. As `CGColorGetConstantColor' is not a "Copy" or
   "Create" function, it does not necessarily return a new reference each
   time it's called. As a consequence, you should not release the returned
   value. However, colors returned from `CGColorGetConstantColor' can be
   retained and released in a properly nested fashion, just like any other
   CF type. }
<<<<<<< HEAD
=======

function CGColorGetConstantColor(colorName: CFStringRef): CGColorRef; external name '_CGColorGetConstantColor'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew

function CGColorGetConstantColor( colorName: CFStringRef ): CGColorRef; external name '_CGColorGetConstantColor';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
{$endc}

{ Create a color in color space `space' with pattern `pattern' and
   components `components'. `space' must be a pattern color space. }

<<<<<<< HEAD
function CGColorCreateWithPattern( colorspace: CGColorSpaceRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreateWithPattern';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
=======
function CGColorCreateWithPattern( colorspace: CGColorSpaceRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreateWithPattern'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CGColorCreateWithPattern( colorspace: CGColorSpaceRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreateWithPattern'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CGColorCreateWithPattern( colorspace: CGColorSpaceRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreateWithPattern'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CGColorCreateWithPattern( colorspace: CGColorSpaceRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreateWithPattern'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew

{ Create a copy of `color'. }

function CGColorCreateCopy( color: CGColorRef ): CGColorRef; external name '_CGColorCreateCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Create a copy of `color' with alpha set to `alpha'. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGColorCreateCopyWithAlpha( color: CGColorRef; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateCopyWithAlpha';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
=======
function CGColorCreateCopyWithAlpha( color: CGColorRef; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateCopyWithAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CGColorCreateCopyWithAlpha( color: CGColorRef; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateCopyWithAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CGColorCreateCopyWithAlpha( color: CGColorRef; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateCopyWithAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CGColorCreateCopyWithAlpha( color: CGColorRef; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateCopyWithAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew

{ Equivalent to `CFRetain(color)', except it doesn't crash (as CFRetain
   does) if `color' is NULL. }

function CGColorRetain( color: CGColorRef ): CGColorRef; external name '_CGColorRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(color)', except it doesn't crash (as CFRelease
   does) if `color' is NULL. }

procedure CGColorRelease( color: CGColorRef ); external name '_CGColorRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return true if `color1' is equal to `color2'; false otherwise. }

function CGColorEqualToColor( color1: CGColorRef; color2: CGColorRef ): CBool; external name '_CGColorEqualToColor';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the number of color components (including alpha) associated with
   `color'. }

function CGColorGetNumberOfComponents( color: CGColorRef ): size_t; external name '_CGColorGetNumberOfComponents';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the color components (including alpha) associated with `color'. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGColorGetComponents( color: CGColorRef ): CGFloatPtr; external name '_CGColorGetComponents';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the alpha component associated with `color'. }

function CGColorGetAlpha( color: CGColorRef ): CGFloat; external name '_CGColorGetAlpha';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
=======
function CGColorGetComponents( color: CGColorRef ): CGFloatPtr; external name '_CGColorGetComponents'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
function CGColorGetComponents( color: CGColorRef ): CGFloatPtr; external name '_CGColorGetComponents'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return the alpha component associated with `color'. }

function CGColorGetAlpha( color: CGColorRef ): CGFloat; external name '_CGColorGetAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
{ Return the alpha component associated with `color'. }
=======
=======
>>>>>>> origin/cpstrnew
function CGColorGetComponents( color: CGColorRef ): CGFloatPtr; external name '_CGColorGetComponents'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return the alpha component associated with `color'. }

function CGColorGetAlpha( color: CGColorRef ): CGFloat; external name '_CGColorGetAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

function CGColorGetAlpha( color: CGColorRef ): CGFloat; external name '_CGColorGetAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Return the color space associated with `color'. }

function CGColorGetColorSpace( color: CGColorRef ): CGColorSpaceRef; external name '_CGColorGetColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the pattern associated with `color', if it's a color in a pattern
   color space; NULL otherwise. }

function CGColorGetPattern( color: CGColorRef ): CGPatternRef; external name '_CGColorGetPattern';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the CFTypeID for CGColors. }

function CGColorGetTypeID: CFTypeID; external name '_CGColorGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
{** Names of colors for use with `CGColorGetConstantColor'. **}

<<<<<<< HEAD
{ Colors in the "Generic" gray color space. }

<<<<<<< HEAD
var kCGColorWhite: CFStringRef; external name '_kCGColorWhite'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

var kCGColorBlack: CFStringRef; external name '_kCGColorBlack'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

var kCGColorClear: CFStringRef; external name '_kCGColorClear'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
{$endc}

<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Return the CFTypeID for CGColors. }

function CGColorGetTypeID: CFTypeID; external name '_CGColorGetTypeID'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$ifc TARGET_OS_MAC}
{** Names of colors for use with `CGColorGetConstantColor'. **}

{ Colors in the "Generic" gray color space. }

var kCGColorWhite: CFStringRef; external name '_kCGColorWhite'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var  kCGColorBlack: CFStringRef; external name '_kCGColorBlack'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kCGColorClear: CFStringRef; external name '_kCGColorClear'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{$endc}
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
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{ Return the CFTypeID for CGColors. }
=======
{ Create a color in the color space `space' with color components
   (including alpha) specified by `components'. `space' may be any color
   space except a pattern color space. }
>>>>>>> origin/fixes_2.4

function CGColorCreate( space: CGColorSpaceRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreate'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$ifc TARGET_OS_MAC}
{ Create a color in the "Generic" gray color space. }

function CGColorCreateGenericGray(gray: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericGray'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Create a color in the "Generic" RGB color space. }

function CGColorCreateGenericRGB(red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericRGB'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Create a color in the "Generic" CMYK color space. }

function CGColorCreateGenericCMYK(cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat): CGColorRef; external name '_CGColorCreateGenericCMYK'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{$endc}

{ Return a constant color. As `CGColorGetConstantColor' is not a "Copy" or
   "Create" function, it does not necessarily return a new reference each
   time it's called. As a consequence, you should not release the returned
   value. However, colors returned from `CGColorGetConstantColor' can be
   retained and released in a properly nested fashion, just like any other
   CF type. }

function CGColorGetConstantColor(colorName: CFStringRef): CGColorRef; external name '_CGColorGetConstantColor'; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Create a color in colorspace `colorspace' with pattern `pattern' and
 * components `components'.  `colorspace' must be a pattern colorspace. }

function CGColorCreateWithPattern( colorspace: CGColorSpaceRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ): CGColorRef; external name '_CGColorCreateWithPattern'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Create a copy of `color'. }

function CGColorCreateCopy( color: CGColorRef ): CGColorRef; external name '_CGColorCreateCopy'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Create a copy of `color' with alpha set to `alpha'. }

function CGColorCreateCopyWithAlpha( color: CGColorRef; alpha: CGFloat ): CGColorRef; external name '_CGColorCreateCopyWithAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Equivalent to `CFRetain(color)', except it doesn't crash (as CFRetain
 * does) if `color' is NULL. }

function CGColorRetain( color: CGColorRef ): CGColorRef; external name '_CGColorRetain'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Equivalent to `CFRelease(color)', except it doesn't crash (as CFRelease
 * does) if `color' is NULL. }

procedure CGColorRelease( color: CGColorRef ); external name '_CGColorRelease'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return true if `color1' is equal to `color2'; false otherwise. }

function CGColorEqualToColor( color1: CGColorRef; color2: CGColorRef ): CBool; external name '_CGColorEqualToColor'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return the number of color components (including alpha) associated with
 * `color'. }

function CGColorGetNumberOfComponents( color: CGColorRef ): size_t; external name '_CGColorGetNumberOfComponents'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return the color components (including alpha) associated with
 * `color'. }

function CGColorGetComponents( color: CGColorRef ): CGFloatPtr; external name '_CGColorGetComponents'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return the alpha component associated with `color'. }

function CGColorGetAlpha( color: CGColorRef ): CGFloat; external name '_CGColorGetAlpha'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return the colorspace associated with `color'. }

function CGColorGetColorSpace( color: CGColorRef ): CGColorSpaceRef; external name '_CGColorGetColorSpace'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return the pattern associated with `color', if it's a color in a pattern
 * colorspace; NULL otherwise. }

function CGColorGetPattern( color: CGColorRef ): CGPatternRef; external name '_CGColorGetPattern'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{ Return the CFTypeID for CGColors. }

function CGColorGetTypeID: CFTypeID; external name '_CGColorGetTypeID'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$ifc TARGET_OS_MAC}
{** Names of colors for use with `CGColorGetConstantColor'. **}

{ Colors in the "Generic" gray color space. }

var kCGColorWhite: CFStringRef; external name '_kCGColorWhite'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var  kCGColorBlack: CFStringRef; external name '_kCGColorBlack'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kCGColorClear: CFStringRef; external name '_kCGColorClear'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
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
