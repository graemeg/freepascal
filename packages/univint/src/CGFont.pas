{ CoreGraphics - CGFont.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 1999-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
 * Copyright (c) 1999-2003 Apple Computer, Inc.
 * All rights reserved.
 }
=======
   Copyright (c) 1999-2009 Apple Inc.
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

unit CGFont;
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
uses MacTypes,CFBase,CFData,CFDictionary,CFArray,CGBase,CGDataProvider,CGGeometry;
{$endc} {not MACOSALLINCLUDE}

=======
uses MacTypes,CFBase,CFData,CFDictionary,CFArray,CGBase;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CFData,CFDictionary,CFArray,CGBase;
>>>>>>> origin/fixes_2_2
=======
uses MacTypes,CFBase,CFData,CFDictionary,CFArray,CGBase,CGDataProvider,CGGeometry;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{ The type used to represent a CoreGraphics font. }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGFontRef = ^OpaqueCGFontRef; { an opaque type }
	OpaqueCGFontRef = record end;
=======
	CGFontRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGFontRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGFontRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGFontRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CGFontRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CGFontRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CGFontRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{ A type to represent indexes in a CGFontRef. }

type
	CGFontIndex = UInt16;

{ A type to represent glyph identifiers in a CGFontRef. }

type
	CGGlyph = CGFontIndex;
	CGGlyphPtr								= ^CGGlyph;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{ The format of a PostScript font subset. Type1 is documented in "Adobe
   Type 1 Font Format"; Type3 in "PostScript Language Reference, 3rd ed."
   and Type42 in "Adobe Technical Note 5012, The Type 42 Font Format
   Specification". }
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{ The format of a PostScript font subset. Type1 is documented in the
 * "Adobe Type 1 Font Format"; Type3 in the "PostScript Language Reference,
 * 3rd ed." and Type42 in "Adobe Technical Note 5012, The Type 42 Font
 * Format Specification". }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

type
	CGFontPostScriptFormat = SInt32;
const
	kCGFontPostScriptFormatType1 = 1;
	kCGFontPostScriptFormatType3 = 3;
	kCGFontPostScriptFormatType42 = 42;


const
{ The maximum allowed value of a CGFontIndex. Always <= USHRT_MAX - 1. }
	kCGFontIndexMax = (1 shl 16) - 2;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { A value representing an invalid CGFontIndex. Always <= USHRT_MAX. }
	kCGFontIndexInvalid = (1 shl 16) - 1;

  { The maximum allowed value of a CGGlyph. }
	kCGGlyphMax = kCGFontIndexMax;

{ Return the CFTypeID for CGFontRefs. }

function CGFontGetTypeID: CFTypeID; external name '_CGFontGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
{ Create a CGFontRef using `platformFontReference', a pointer to a
   platform-specific font reference. For MacOS X, `platformFontReference'
   should be a pointer to an ATSFontRef. }

function CGFontCreateWithPlatformFont( platformFontReference: UnivPtr ): CGFontRef; external name '_CGFontCreateWithPlatformFont';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6,
    __IPHONE_NA, __IPHONE_NA) *)
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ Return the font defined by the data provided by `provider', or NULL if
   the font can't be created. }

function CGFontCreateWithDataProvider( provider: CGDataProviderRef ): CGFontRef; external name '_CGFontCreateWithDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

=======

{ Return the font defined by the data provided by `provider', or NULL if
   the font can't be created. }

function CGFontCreateWithDataProvider( provider: CGDataProviderRef ): CGFontRef; external name '_CGFontCreateWithDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
{ Return the font identified by `name', corresponding to the font's
   PostScript name or its full name, or NULL if the font can't be
   created. }

function CGFontCreateWithFontName( name: CFStringRef ): CGFontRef; external name '_CGFontCreateWithFontName';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

=======
=======
>>>>>>> graemeg/cpstrnew

{ Return the font defined by the data provided by `provider', or NULL if
   the font can't be created. }

function CGFontCreateWithDataProvider( provider: CGDataProviderRef ): CGFontRef; external name '_CGFontCreateWithDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

=======

{ Return the font defined by the data provided by `provider', or NULL if
   the font can't be created. }

function CGFontCreateWithDataProvider( provider: CGDataProviderRef ): CGFontRef; external name '_CGFontCreateWithDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

>>>>>>> origin/cpstrnew
{ Return the font identified by `name', corresponding to the font's
   PostScript name or its full name, or NULL if the font can't be
   created. }

function CGFontCreateWithFontName( name: CFStringRef ): CGFontRef; external name '_CGFontCreateWithFontName';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Return a font based on `font' with the variation specification dictionary
   `variations' applied to `font'. A variation specification dictionary
   contains keys corresponding the variation axis names of the font. Each
   key is a variation axis name; the value for each key is the value
   specified for that particular variation axis represented as a
   CFNumberRef. If a variation axis name is not specified in `variations',
   then the current value from `font' is used. }

function CGFontCreateCopyWithVariations( font: CGFontRef; variations: CFDictionaryRef ): CGFontRef; external name '_CGFontCreateCopyWithVariations';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Equivalent to `CFRetain(font)', except it doesn't crash (as CFRetain
   does) if `font' is NULL. }

function CGFontRetain( font: CGFontRef ): CGFontRef; external name '_CGFontRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(font)', except it doesn't crash (as CFRelease
   does) if `font' is NULL. }

procedure CGFontRelease( font: CGFontRef ); external name '_CGFontRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the number of glyphs in `font'. }

function CGFontGetNumberOfGlyphs( font: CGFontRef ): size_t; external name '_CGFontGetNumberOfGlyphs';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the glyph space units/em for `font'. }

function CGFontGetUnitsPerEm( font: CGFontRef ): SInt32; external name '_CGFontGetUnitsPerEm';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the PostScript name of `font'. }

function CGFontCopyPostScriptName( font: CGFontRef ): CFStringRef; external name '_CGFontCopyPostScriptName';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the "full name" of `font'. }

function CGFontCopyFullName( font: CGFontRef ): CFStringRef; external name '_CGFontCopyFullName';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the ascent of `font'. The ascent is the maximum distance above the
   baseline of glyphs in a font. The value is specified in glyph space
   units. }

function CGFontGetAscent( font: CGFontRef ): SInt32; external name '_CGFontGetAscent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the descent of `font'. The descent is the maximum distance below
   the baseline of glyphs in a font. The value is specified in glyph space
   units. }

function CGFontGetDescent( font: CGFontRef ): SInt32; external name '_CGFontGetDescent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the leading of `font'. The leading is the spacing between
   consecutive lines of text in a font. The value is specified in glyph
   space units. }

function CGFontGetLeading( font: CGFontRef ): SInt32; external name '_CGFontGetLeading';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the cap height of `font'. The cap height is the distance above the
   baseline of the top of flat capital letters of glyphs in a font. The
   value is specified in glyph space units. }

function CGFontGetCapHeight( font: CGFontRef ): SInt32; external name '_CGFontGetCapHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the x-height of `font'. The x-height is the distance above the
   baseline of the top of flat, non-ascending lowercase letters (such as
   "x") of glyphs in a font. The value is specified in glyph space units. }

function CGFontGetXHeight( font: CGFontRef ): SInt32; external name '_CGFontGetXHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the font bounding box of `font'. The font bounding box is the
   union of all of the bounding boxes for all the glyphs in a font. The
   value is specified in glyph space units. }

function CGFontGetFontBBox( font: CGFontRef ): CGRect; external name '_CGFontGetFontBBox';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the italic angle of `font', measured in degrees counter-clockwise
   from the vertical. }

function CGFontGetItalicAngle( font: CGFontRef ): CGFloat; external name '_CGFontGetItalicAngle';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the thickness of the dominant vertical stems of glyphs in `font'.
   The value is specified in glyph space units. }

function CGFontGetStemV( font: CGFontRef ): CGFloat; external name '_CGFontGetStemV';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return an array of the variation axis dictionaries for `font'. Each
   variation axis dictionary contains values for the variation axis keys
   listed below. This function returns NULL if `font' doesn't support
   variations. }

function CGFontCopyVariationAxes( font: CGFontRef ): CFArrayRef; external name '_CGFontCopyVariationAxes';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the variation specification dictionary from `font'. This
   dictionary contains keys corresponding the variation axis names of the
   font. Each key is a variation axis name; the value for each key is the
   value specified for that particular variation axis represented as a
   CFNumberRef. This function returns NULL if `font' doesn't support
   variations. }

function CGFontCopyVariations( font: CGFontRef ): CFDictionaryRef; external name '_CGFontCopyVariations';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Get the advance of each glyph in `glyphs', an array of `count' glyphs,
   and return it in the corresponding entry of `advances', an array of
   `count' integers. The advances are specified in glyph space. Returns
   false if advances can't be retrieved for any reason; true otherwise. }
<<<<<<< HEAD

function CGFontGetGlyphAdvances( font: CGFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t; advances: {variable-size-array} SInt32Ptr ): CBool; external name '_CGFontGetGlyphAdvances';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Get the bounding box of each glyph in `glyphs', an array of `count'
   glyphs, and return it in the corresponding entry of `bboxes', an array of
   `count' rectangles. The bounding boxes are specified in glyph space.
   Returns false if bounding boxes can't be retrieved for any reason; true
   otherwise. }

function CGFontGetGlyphBBoxes( font: CGFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t; bboxes: {variable-size-array} CGRectPtr ): CBool; external name '_CGFontGetGlyphBBoxes';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ Return the glyph associated with `name' in `font'. If `name' isn't found
   in the font, return 0. }

=======
=======
>>>>>>> graemeg/cpstrnew

{ Return the glyph associated with `name' in `font'. If `name' isn't found
   in the font, return 0. }

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
function CGFontGetGlyphWithGlyphName( font: CGFontRef; name: CFStringRef ): CGGlyph; external name '_CGFontGetGlyphWithGlyphName';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the glyph name of `glyph' in `font', or NULL if `glyph' does not
   appear in `font'. }

=======

{ Return the glyph associated with `name' in `font'. If `name' isn't found
   in the font, return 0. }

=======

function CGFontGetGlyphAdvances( font: CGFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t; advances: {variable-size-array} SInt32Ptr ): CBool; external name '_CGFontGetGlyphAdvances';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Get the bounding box of each glyph in `glyphs', an array of `count'
   glyphs, and return it in the corresponding entry of `bboxes', an array of
   `count' rectangles. The bounding boxes are specified in glyph space.
   Returns false if bounding boxes can't be retrieved for any reason; true
   otherwise. }

function CGFontGetGlyphBBoxes( font: CGFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t; bboxes: {variable-size-array} CGRectPtr ): CBool; external name '_CGFontGetGlyphBBoxes';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the glyph associated with `name' in `font'. If `name' isn't found
   in the font, return 0. }

>>>>>>> origin/cpstrnew
function CGFontGetGlyphWithGlyphName( font: CGFontRef; name: CFStringRef ): CGGlyph; external name '_CGFontGetGlyphWithGlyphName';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the glyph name of `glyph' in `font', or NULL if `glyph' does not
   appear in `font'. }

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function CGFontCopyGlyphNameForGlyph( font: CGFontRef; glyph: CGGlyph ): CFStringRef; external name '_CGFontCopyGlyphNameForGlyph';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return true if a subset in the PostScript format `format' can be created
   for `font'; false otherwise. }

function CGFontCanCreatePostScriptSubset( font: CGFontRef; format: CGFontPostScriptFormat ): CBool; external name '_CGFontCanCreatePostScriptSubset';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a subset of `font' named `subsetName' in the PostScript format
   `format'. The subset will contain the glyphs specified by `glyphs', an
   array of `count' CGGlyphs. If non-NULL, `encoding' specifies the default
   encoding for the subset. }

type
	CGGlyph256Array = array[0..255] of CGGlyph;
function CGFontCreatePostScriptSubset( font: CGFontRef; subsetName: CFStringRef; format: CGFontPostScriptFormat; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t; const (*var*) encoding: CGGlyph256Array ): CFDataRef; external name '_CGFontCreatePostScriptSubset';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD

{ Return a PostScript encoding of `font' containing glyphs in
   `encoding'. }
<<<<<<< HEAD
<<<<<<< HEAD
=======

{ Return a PostScript encoding of `font' containing glyphs in
   `encoding'. }
>>>>>>> origin/cpstrnew

function CGFontCreatePostScriptEncoding( font: CGFontRef; const (*var*) encoding: CGGlyph256Array ): CFDataRef; external name '_CGFontCreatePostScriptEncoding';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return an array of font table tags in `font'. Each entry in the array is
   a four-byte value representing a single TrueType or OpenType font table
   tag. }

<<<<<<< HEAD
=======

function CGFontCreatePostScriptEncoding( font: CGFontRef; const (*var*) encoding: CGGlyph256Array ): CFDataRef; external name '_CGFontCreatePostScriptEncoding';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return an array of font table tags in `font'. Each entry in the array is
   a four-byte value representing a single TrueType or OpenType font table
   tag. }

>>>>>>> graemeg/cpstrnew
=======

function CGFontCreatePostScriptEncoding( font: CGFontRef; const (*var*) encoding: CGGlyph256Array ): CFDataRef; external name '_CGFontCreatePostScriptEncoding';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return an array of font table tags in `font'. Each entry in the array is
   a four-byte value representing a single TrueType or OpenType font table
   tag. }

>>>>>>> graemeg/cpstrnew
=======

{ Return a PostScript encoding of `font' containing glyphs in
   `encoding'. }

function CGFontCreatePostScriptEncoding( font: CGFontRef; const (*var*) encoding: CGGlyph256Array ): CFDataRef; external name '_CGFontCreatePostScriptEncoding';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return an array of font table tags in `font'. Each entry in the array is
   a four-byte value representing a single TrueType or OpenType font table
   tag. }

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function CGFontCopyTableTags( font: CGFontRef ): CFArrayRef; external name '_CGFontCopyTableTags';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the table in `font' corresponding to `tag', or NULL if no such
   table exists. }

function CGFontCopyTableForTag( font: CGFontRef; tag: UInt32 ): CFDataRef; external name '_CGFontCopyTableForTag';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
    { A value representing an invalid CGFontIndex. Always <= USHRT_MAX. }
=======
  { A value representing an invalid CGFontIndex. Always <= USHRT_MAX. }
>>>>>>> origin/fixes_2.4
	kCGFontIndexInvalid = (1 shl 16) - 1;

  { The maximum allowed value of a CGGlyph. }
	kCGGlyphMax = kCGFontIndexMax;

{ Return the CFTypeID for CGFontRefs. }

function CGFontGetTypeID: CFTypeID; external name '_CGFontGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
{ Create a CGFontRef using `platformFontReference', a pointer to a
   platform-specific font reference. For MacOS X, `platformFontReference'
   should be a pointer to an ATSFontRef. }

function CGFontCreateWithPlatformFont( platformFontReference: UnivPtr ): CGFontRef; external name '_CGFontCreateWithPlatformFont';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_6,
    __IPHONE_NA, __IPHONE_NA) *)
{$endc}

{ Return the font defined by the data provided by `provider', or NULL if
   the font can't be created. }

function CGFontCreateWithDataProvider( provider: CGDataProviderRef ): CGFontRef; external name '_CGFontCreateWithDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the font identified by `name', corresponding to the font's
   PostScript name or its full name, or NULL if the font can't be
   created. }

function CGFontCreateWithFontName( name: CFStringRef ): CGFontRef; external name '_CGFontCreateWithFontName';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return a font based on `font' with the variation specification dictionary
   `variations' applied to `font'. A variation specification dictionary
   contains keys corresponding the variation axis names of the font. Each
   key is a variation axis name; the value for each key is the value
   specified for that particular variation axis represented as a
   CFNumberRef. If a variation axis name is not specified in `variations',
   then the current value from `font' is used. }

function CGFontCreateCopyWithVariations( font: CGFontRef; variations: CFDictionaryRef ): CGFontRef; external name '_CGFontCreateCopyWithVariations';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Equivalent to `CFRetain(font)', except it doesn't crash (as CFRetain
   does) if `font' is NULL. }

function CGFontRetain( font: CGFontRef ): CGFontRef; external name '_CGFontRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(font)', except it doesn't crash (as CFRelease
   does) if `font' is NULL. }

procedure CGFontRelease( font: CGFontRef ); external name '_CGFontRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the number of glyphs in `font'. }

function CGFontGetNumberOfGlyphs( font: CGFontRef ): size_t; external name '_CGFontGetNumberOfGlyphs';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the glyph space units/em for `font'. }

function CGFontGetUnitsPerEm( font: CGFontRef ): SInt32; external name '_CGFontGetUnitsPerEm';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the PostScript name of `font'. }

function CGFontCopyPostScriptName( font: CGFontRef ): CFStringRef; external name '_CGFontCopyPostScriptName';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the "full name" of `font'. }

function CGFontCopyFullName( font: CGFontRef ): CFStringRef; external name '_CGFontCopyFullName';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the ascent of `font'. The ascent is the maximum distance above the
   baseline of glyphs in a font. The value is specified in glyph space
   units. }

function CGFontGetAscent( font: CGFontRef ): SInt32; external name '_CGFontGetAscent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the descent of `font'. The descent is the maximum distance below
   the baseline of glyphs in a font. The value is specified in glyph space
   units. }

function CGFontGetDescent( font: CGFontRef ): SInt32; external name '_CGFontGetDescent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the leading of `font'. The leading is the spacing between
   consecutive lines of text in a font. The value is specified in glyph
   space units. }

function CGFontGetLeading( font: CGFontRef ): SInt32; external name '_CGFontGetLeading';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the cap height of `font'. The cap height is the distance above the
   baseline of the top of flat capital letters of glyphs in a font. The
   value is specified in glyph space units. }

function CGFontGetCapHeight( font: CGFontRef ): SInt32; external name '_CGFontGetCapHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the x-height of `font'. The x-height is the distance above the
   baseline of the top of flat, non-ascending lowercase letters (such as
   "x") of glyphs in a font. The value is specified in glyph space units. }

function CGFontGetXHeight( font: CGFontRef ): SInt32; external name '_CGFontGetXHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the font bounding box of `font'. The font bounding box is the
   union of all of the bounding boxes for all the glyphs in a font. The
   value is specified in glyph space units. }

function CGFontGetFontBBox( font: CGFontRef ): CGRect; external name '_CGFontGetFontBBox';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the italic angle of `font', measured in degrees counter-clockwise
   from the vertical. }

function CGFontGetItalicAngle( font: CGFontRef ): CGFloat; external name '_CGFontGetItalicAngle';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the thickness of the dominant vertical stems of glyphs in `font'.
   The value is specified in glyph space units. }

function CGFontGetStemV( font: CGFontRef ): CGFloat; external name '_CGFontGetStemV';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return an array of the variation axis dictionaries for `font'. Each
   variation axis dictionary contains values for the variation axis keys
   listed below. This function returns NULL if `font' doesn't support
   variations. }

function CGFontCopyVariationAxes( font: CGFontRef ): CFArrayRef; external name '_CGFontCopyVariationAxes';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the variation specification dictionary from `font'. This
   dictionary contains keys corresponding the variation axis names of the
   font. Each key is a variation axis name; the value for each key is the
   value specified for that particular variation axis represented as a
   CFNumberRef. This function returns NULL if `font' doesn't support
   variations. }

function CGFontCopyVariations( font: CGFontRef ): CFDictionaryRef; external name '_CGFontCopyVariations';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Get the advance of each glyph in `glyphs', an array of `count' glyphs,
   and return it in the corresponding entry of `advances', an array of
   `count' integers. The advances are specified in glyph space. Returns
   false if advances can't be retrieved for any reason; true otherwise. }

function CGFontGetGlyphAdvances( font: CGFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t; advances: {variable-size-array} SInt32Ptr ): CBool; external name '_CGFontGetGlyphAdvances';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Get the bounding box of each glyph in `glyphs', an array of `count'
   glyphs, and return it in the corresponding entry of `bboxes', an array of
   `count' rectangles. The bounding boxes are specified in glyph space.
   Returns false if bounding boxes can't be retrieved for any reason; true
   otherwise. }

function CGFontGetGlyphBBoxes( font: CGFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t; bboxes: {variable-size-array} CGRectPtr ): CBool; external name '_CGFontGetGlyphBBoxes';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the glyph associated with `name' in `font'. If `name' isn't found
   in the font, return 0. }

function CGFontGetGlyphWithGlyphName( font: CGFontRef; name: CFStringRef ): CGGlyph; external name '_CGFontGetGlyphWithGlyphName';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the glyph name of `glyph' in `font', or NULL if `glyph' does not
   appear in `font'. }

function CGFontCopyGlyphNameForGlyph( font: CGFontRef; glyph: CGGlyph ): CFStringRef; external name '_CGFontCopyGlyphNameForGlyph';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return true if a subset in the PostScript format `format' can be created
   for `font'; false otherwise. }

function CGFontCanCreatePostScriptSubset( font: CGFontRef; format: CGFontPostScriptFormat ): CBool; external name '_CGFontCanCreatePostScriptSubset';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a subset of `font' named `subsetName' in the PostScript format
   `format'. The subset will contain the glyphs specified by `glyphs', an
   array of `count' CGGlyphs. If non-NULL, `encoding' specifies the default
   encoding for the subset. }

type
	CGGlyph256Array = array[0..255] of CGGlyph;
function CGFontCreatePostScriptSubset( font: CGFontRef; subsetName: CFStringRef; format: CGFontPostScriptFormat; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t; const (*var*) encoding: CGGlyph256Array ): CFDataRef; external name '_CGFontCreatePostScriptSubset';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return a PostScript encoding of `font' containing glyphs in
   `encoding'. }

function CGFontCreatePostScriptEncoding( font: CGFontRef; const (*var*) encoding: CGGlyph256Array ): CFDataRef; external name '_CGFontCreatePostScriptEncoding';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return an array of font table tags in `font'. Each entry in the array is
   a four-byte value representing a single TrueType or OpenType font table
   tag. }

<<<<<<< HEAD
function CGFontCreatePostScriptEncoding( font: CGFontRef; const (*var*) encoding: CGGlyph256Array ): CFDataRef; external name '_CGFontCreatePostScriptEncoding'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGFontCopyTableTags( font: CGFontRef ): CFArrayRef; external name '_CGFontCopyTableTags';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the table in `font' corresponding to `tag', or NULL if no such
   table exists. }

function CGFontCopyTableForTag( font: CGFontRef; tag: UInt32 ): CFDataRef; external name '_CGFontCopyTableForTag';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{** Keys for the font variation axis dictionary. **}

{ The key used to obtain the variation axis name from a variation axis
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   dictionary. The value obtained with this key is a CFStringRef specifying
   the name of the variation axis. }

var kCGFontVariationAxisName: CFStringRef; external name '_kCGFontVariationAxisName'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The key used to obtain the minimum variation axis value from a variation
   axis dictionary. The value obtained with this key is a CFNumberRef
   specifying the minimum value of the variation axis. }

var kCGFontVariationAxisMinValue: CFStringRef; external name '_kCGFontVariationAxisMinValue'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The key used to obtain the maximum variation axis value from a variation
   axis dictionary. The value obtained with this key is a CFNumberRef
   specifying the maximum value of the variation axis. }

var kCGFontVariationAxisMaxValue: CFStringRef; external name '_kCGFontVariationAxisMaxValue'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The key used to obtain the default variation axis value from a variation
   axis dictionary. The value obtained with this key is a CFNumberRef
   specifying the default value of the variation axis. }

var kCGFontVariationAxisDefaultValue: CFStringRef; external name '_kCGFontVariationAxisDefaultValue'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 * dictionary. The value obtained with this key is a CFStringRef specifying
 * the name of the variation axis. }
=======
   dictionary. The value obtained with this key is a CFStringRef specifying
   the name of the variation axis. }
>>>>>>> origin/fixes_2.4

var kCGFontVariationAxisName: CFStringRef; external name '_kCGFontVariationAxisName'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The key used to obtain the minimum variation axis value from a variation
   axis dictionary. The value obtained with this key is a CFNumberRef
   specifying the minimum value of the variation axis. }

var kCGFontVariationAxisMinValue: CFStringRef; external name '_kCGFontVariationAxisMinValue'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The key used to obtain the maximum variation axis value from a variation
   axis dictionary. The value obtained with this key is a CFNumberRef
   specifying the maximum value of the variation axis. }

var kCGFontVariationAxisMaxValue: CFStringRef; external name '_kCGFontVariationAxisMaxValue'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The key used to obtain the default variation axis value from a variation
   axis dictionary. The value obtained with this key is a CFNumberRef
   specifying the default value of the variation axis. }

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
var kCGFontVariationAxisDefaultValue: CFStringRef; external name '_kCGFontVariationAxisDefaultValue'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Obsolete; don't use these. }

const
	CGGlyphMin = 0;
	CGGlyphMax = kCGGlyphMax;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
