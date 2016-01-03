{ CoreGraphics - CGContext.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2015 }
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
 * Copyright (c) 2000-2003 Apple Computer, Inc.
 * All rights reserved.
 }
=======
 * Copyright (c) 2000-2008 Apple Inc.
 * All rights reserved. }
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

unit CGContext;
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
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
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
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
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
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CFBase,CGGeometry,CGBase,CFDictionary,CGAffineTransforms,CGColorSpace,CGFont,CGGradient,CGImage,CGPDFDocument,CGPath,CGColor,CGShading,CGPDFPage;
{$endc} {not MACOSALLINCLUDE}
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

=======
uses MacTypes,CFBase,CGGeometry,CGBase,CFDictionary,CGAffineTransforms,CGColorSpace,CGFont,CGImage,CGPDFDocument,CGPath,CGColor,CGShading,CGPDFPage;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CGGeometry,CGBase,CFDictionary,CGAffineTransforms,CGColorSpace,CGFont,CGImage,CGPDFDocument,CGPath,CGColor,CGShading,CGPDFPage;
>>>>>>> origin/fixes_2_2
{$ALIGN POWER}


type
<<<<<<< HEAD
<<<<<<< HEAD
	CGContextRef = ^SInt32; { an opaque type }
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
=======
uses MacTypes,CFBase,CGGeometry,CGBase,CFDictionary,CGAffineTransforms,CGColorSpace,CGFont,CGGradient,CGImage,CGPDFDocument,CGPath,CGColor,CGShading,CGPDFPage;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

{$ALIGN POWER}


type
<<<<<<< HEAD
	CGContextRef = ^OpaqueCGContextRef; { an opaque type }
	OpaqueCGContextRef = record end;

=======
=======
>>>>>>> origin/fixes_2_2
	CGContextRef = ^SInt32; { an opaque 32-bit type }
=======
	CGContextRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


{ Line join styles. }

type
	CGLineJoin = SInt32;
const
	kCGLineJoinMiter = 0;
	kCGLineJoinRound = 1;
	kCGLineJoinBevel = 2;

{ Line cap styles. }

type
	CGLineCap = SInt32;
const
	kCGLineCapButt = 0;
	kCGLineCapRound = 1;
	kCGLineCapSquare = 2;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ Drawing modes for paths. }

type
	CGPathDrawingMode = SInt32;
const
	kCGPathFill = 0;
	kCGPathEOFill = 1;
	kCGPathStroke = 2;
	kCGPathFillStroke = 3;
	kCGPathEOFillStroke = 4;

{ Drawing modes for text. }

type
	CGTextDrawingMode = SInt32;
const
	kCGTextFill = 0;
	kCGTextStroke = 1;
	kCGTextFillStroke = 2;
	kCGTextInvisible = 3;
	kCGTextFillClip = 4;
	kCGTextStrokeClip = 5;
	kCGTextFillStrokeClip = 6;
	kCGTextClip = 7;

{ Text encodings. }

type
	CGTextEncoding = SInt32;
const
	kCGEncodingFontSpecific = 0;
	kCGEncodingMacRoman = 1;

{ Interpolation quality. }

type
	CGInterpolationQuality = SInt32;
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCGInterpolationDefault = 0;	{ Let the context decide. }
	kCGInterpolationNone = 1;		{ Never interpolate. }
	kCGInterpolationLow = 2;		{ Low quality, fast interpolation. }
	kCGInterpolationMedium = 4;		{ Medium quality, slower than kCGInterpolationLow. Available in Mac OS X 10.6 & later. }
	kCGInterpolationHigh = 3;		{ Highest quality, slower than kCGInterpolationMedium. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Blend modes.

   The blend modes from kCGBlendModeNormal to kCGBlendModeLuminosity are
   supported in Mac OS X 10.4 and later. The Porter-Duff blend modes (from
   kCGBlendModeClear to kCGBlendModePlusLighter) are supported in Mac OS X
   10.5 and later. The names of the Porter-Duff blend modes are historical.

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew

{ Blend modes.

   The blend modes from kCGBlendModeNormal to kCGBlendModeLuminosity are
   supported in Mac OS X 10.4 and later. The Porter-Duff blend modes (from
   kCGBlendModeClear to kCGBlendModePlusLighter) are supported in Mac OS X
   10.5 and later. The names of the Porter-Duff blend modes are historical.

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   Note that the Porter-Duff blend modes are not necessarily supported in
   every context. In particular, they are only guaranteed to work in
   bitmap-based contexts, such as those created by CGBitmapContextCreate. It
   is your responsibility to make sure that they do what you want when you
   use them in a CGContext. }
=======
=======
>>>>>>> origin/fixes_2_2
	kCGInterpolationDefault = 0;		{ Let the context decide. }
=======
	kCGInterpolationDefault = 0;	{ Let the context decide. }
>>>>>>> origin/fixes_2.4
	kCGInterpolationNone = 1;		{ Never interpolate. }
	kCGInterpolationLow = 2;		{ Low quality, fast interpolation. }
	kCGInterpolationMedium = 4;		{ Medium quality, slower than kCGInterpolationLow. Available in Mac OS X 10.6 & later. }
	kCGInterpolationHigh = 3;		{ Highest quality, slower than kCGInterpolationMedium. }

<<<<<<< HEAD
{ Blend modes. }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Blend modes.

   The blend modes from kCGBlendModeNormal to kCGBlendModeLuminosity are
   supported in Mac OS X 10.4 and later. The Porter-Duff blend modes (from
   kCGBlendModeClear to kCGBlendModePlusLighter) are supported in Mac OS X
   10.5 and later. The names of the Porter-Duff blend modes are historical.

   Note that the Porter-Duff blend modes are not necessarily supported in
   every context. In particular, they are only guaranteed to work in
   bitmap-based contexts, such as those created by CGBitmapContextCreate. It
   is your responsibility to make sure that they do what you want when you
   use them in a CGContext. }
>>>>>>> origin/fixes_2.4

type
	CGBlendMode = SInt32;
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Available in Mac OS X 10.4 & later. }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Available in Mac OS X 10.4 & later. }
>>>>>>> origin/fixes_2.4
	kCGBlendModeNormal = 0;
	kCGBlendModeMultiply = 1;
	kCGBlendModeScreen = 2;
	kCGBlendModeOverlay = 3;
	kCGBlendModeDarken = 4;
	kCGBlendModeLighten = 5;
	kCGBlendModeColorDodge = 6;
	kCGBlendModeColorBurn = 7;
	kCGBlendModeSoftLight = 8;
	kCGBlendModeHardLight = 9;
	kCGBlendModeDifference = 10;
	kCGBlendModeExclusion = 11;
	kCGBlendModeHue = 12;
	kCGBlendModeSaturation = 13;
	kCGBlendModeColor = 14;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kCGBlendModeLuminosity = 15;

    { Available in Mac OS X 10.5 & later. R, S, and D are, respectively,
       premultiplied result, source, and destination colors with alpha; Ra,
       Sa, and Da are the alpha components of these colors.

       The Porter-Duff "source over" mode is called `kCGBlendModeNormal':
         R = S + D*(1 - Sa)

       Note that the Porter-Duff "XOR" mode is only titularly related to the
       classical bitmap XOR operation (which is unsupported by
       CoreGraphics). }

	kCGBlendModeClear = 16;			{ R = 0 }
	kCGBlendModeCopy = 17;			{ R = S }
	kCGBlendModeSourceIn = 18;		{ R = S*Da }
	kCGBlendModeSourceOut = 19;		{ R = S*(1 - Da) }
	kCGBlendModeSourceAtop = 20;		{ R = S*Da + D*(1 - Sa) }
	kCGBlendModeDestinationOver = 21;	{ R = S*(1 - Da) + D }
	kCGBlendModeDestinationIn = 22;		{ R = D*Sa }
	kCGBlendModeDestinationOut = 23;		{ R = D*(1 - Sa) }
	kCGBlendModeDestinationAtop = 24;	{ R = S*(1 - Da) + D*Sa }
	kCGBlendModeXOR = 25;			{ R = S*(1 - Da) + D*(1 - Sa) }
	kCGBlendModePlusDarker = 26;		{ R = MAX(0, (1 - D) + (1 - S)) }
	kCGBlendModePlusLighter = 27;		{ R = MIN(1, S + D) } { Available in Mac OS X 10.4 & later. }
<<<<<<< HEAD

{ Return the CFTypeID for CGContextRefs. }

function CGContextGetTypeID: CFTypeID; external name '_CGContextGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{* Graphics state functions. *}

{ Push a copy of the current graphics state onto the graphics state stack.
   Note that the path is not considered part of the graphics state, and is
   not saved. }

procedure CGContextSaveGState( c: CGContextRef ); external name '_CGContextSaveGState';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Restore the current graphics state from the one on the top of the
   graphics state stack, popping the graphics state stack in the process. }

procedure CGContextRestoreGState( c: CGContextRef ); external name '_CGContextRestoreGState';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
	kCGBlendModeLuminosity = 15; { Available in Mac OS X 10.4 & later. }

=======
>>>>>>> origin/fixes_2.4

{ Return the CFTypeID for CGContextRefs. }

function CGContextGetTypeID: CFTypeID; external name '_CGContextGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{* Graphics state functions. *}

{ Push a copy of the current graphics state onto the graphics state stack.
   Note that the path is not considered part of the graphics state, and is
   not saved. }

procedure CGContextSaveGState( c: CGContextRef ); external name '_CGContextSaveGState';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Restore the current graphics state from the one on the top of the
   graphics state stack, popping the graphics state stack in the process. }

procedure CGContextRestoreGState( c: CGContextRef ); external name '_CGContextRestoreGState';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Coordinate space transformations. *}

{ Scale the current graphics state's transformation matrix (the CTM) by
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   `(sx, sy)'. }

procedure CGContextScaleCTM( c: CGContextRef; sx: CGFloat; sy: CGFloat ); external name '_CGContextScaleCTM';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Translate the current graphics state's transformation matrix (the CTM) by
   `(tx, ty)'. }

procedure CGContextTranslateCTM( c: CGContextRef; tx: CGFloat; ty: CGFloat ); external name '_CGContextTranslateCTM';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Rotate the current graphics state's transformation matrix (the CTM) by
   `angle' radians. }

procedure CGContextRotateCTM( c: CGContextRef; angle: CGFloat ); external name '_CGContextRotateCTM';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Concatenate the current graphics state's transformation matrix (the CTM)
   with the affine transform `transform'. }

procedure CGContextConcatCTM( c: CGContextRef; transform: CGAffineTransform ); external name '_CGContextConcatCTM';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 * `(sx, sy)'. }
=======
   `(sx, sy)'. }
>>>>>>> origin/fixes_2.4

procedure CGContextScaleCTM( c: CGContextRef; sx: CGFloat; sy: CGFloat ); external name '_CGContextScaleCTM';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Translate the current graphics state's transformation matrix (the CTM) by
   `(tx, ty)'. }

procedure CGContextTranslateCTM( c: CGContextRef; tx: CGFloat; ty: CGFloat ); external name '_CGContextTranslateCTM';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Rotate the current graphics state's transformation matrix (the CTM) by
   `angle' radians. }

procedure CGContextRotateCTM( c: CGContextRef; angle: CGFloat ); external name '_CGContextRotateCTM';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Concatenate the current graphics state's transformation matrix (the CTM)
   with the affine transform `transform'. }

procedure CGContextConcatCTM( c: CGContextRef; transform: CGAffineTransform ); external name '_CGContextConcatCTM';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the current graphics state's transformation matrix. }

function CGContextGetCTM( c: CGContextRef ): CGAffineTransform; external name '_CGContextGetCTM';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Drawing attribute functions. *}

{ Set the line width in the current graphics state to `width'. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure CGContextSetLineWidth( c: CGContextRef; width: CGFloat ); external name '_CGContextSetLineWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
procedure CGContextSetLineWidth( c: CGContextRef; width: Float32 ); external name '_CGContextSetLineWidth';
>>>>>>> graemeg/fixes_2_2
=======
procedure CGContextSetLineWidth( c: CGContextRef; width: Float32 ); external name '_CGContextSetLineWidth';
>>>>>>> origin/fixes_2_2
=======
procedure CGContextSetLineWidth( c: CGContextRef; width: CGFloat ); external name '_CGContextSetLineWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Set the line cap in the current graphics state to `cap'. }

procedure CGContextSetLineCap( c: CGContextRef; cap: CGLineCap ); external name '_CGContextSetLineCap';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Set the line join in the current graphics state to `join'. }

procedure CGContextSetLineJoin( c: CGContextRef; join: CGLineJoin ); external name '_CGContextSetLineJoin';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the miter limit in the current graphics state to `limit'. }

procedure CGContextSetMiterLimit( c: CGContextRef; limit: CGFloat ); external name '_CGContextSetMiterLimit';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the line dash patttern in the current graphics state of `c'. }

procedure CGContextSetLineDash( c: CGContextRef; phase: CGFloat; {const} lengths: {variable-size-array} CGFloatPtr; count: size_t ); external name '_CGContextSetLineDash';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the path flatness parameter in the current graphics state of `c' to
   `flatness'. }

procedure CGContextSetFlatness( c: CGContextRef; flatness: CGFloat ); external name '_CGContextSetFlatness';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the alpha value in the current graphics state of `c' to `alpha'. }

procedure CGContextSetAlpha( c: CGContextRef; alpha: CGFloat ); external name '_CGContextSetAlpha';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the blend mode of `context' to `mode'. }

procedure CGContextSetBlendMode( context: CGContextRef; mode: CGBlendMode ); external name '_CGContextSetBlendMode';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Set the miter limit in the current graphics state to `limit'. }

procedure CGContextSetMiterLimit( c: CGContextRef; limit: CGFloat ); external name '_CGContextSetMiterLimit';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the line dash patttern in the current graphics state of `c'. }

procedure CGContextSetLineDash( c: CGContextRef; phase: CGFloat; {const} lengths: {variable-size-array} CGFloatPtr; count: size_t ); external name '_CGContextSetLineDash';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the path flatness parameter in the current graphics state of `c' to
   `flatness'. }

procedure CGContextSetFlatness( c: CGContextRef; flatness: CGFloat ); external name '_CGContextSetFlatness';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the alpha value in the current graphics state of `c' to `alpha'. }

procedure CGContextSetAlpha( c: CGContextRef; alpha: CGFloat ); external name '_CGContextSetAlpha';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the blend mode of `context' to `mode'. }

<<<<<<< HEAD
procedure CGContextSetBlendMode( context: CGContextRef; mode: CGBlendMode ); external name '_CGContextSetBlendMode'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CGContextSetBlendMode( context: CGContextRef; mode: CGBlendMode ); external name '_CGContextSetBlendMode';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Path construction functions. *}

{ Note that a context has a single path in use at any time: a path is not
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   part of the graphics state. }

{ Begin a new path. The old path is discarded. }

procedure CGContextBeginPath( c: CGContextRef ); external name '_CGContextBeginPath';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Start a new subpath at point `(x, y)' in the context's path. }

procedure CGContextMoveToPoint( c: CGContextRef; x: CGFloat; y: CGFloat ); external name '_CGContextMoveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Append a straight line segment from the current point to `(x, y)'. }

procedure CGContextAddLineToPoint( c: CGContextRef; x: CGFloat; y: CGFloat ); external name '_CGContextAddLineToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Append a cubic Bezier curve from the current point to `(x,y)', with
   control points `(cp1x, cp1y)' and `(cp2x, cp2y)'. }

procedure CGContextAddCurveToPoint( c: CGContextRef; cp1x: CGFloat; cp1y: CGFloat; cp2x: CGFloat; cp2y: CGFloat; x: CGFloat; y: CGFloat ); external name '_CGContextAddCurveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Append a quadratic curve from the current point to `(x, y)', with control
   point `(cpx, cpy)'. }

procedure CGContextAddQuadCurveToPoint( c: CGContextRef; cpx: CGFloat; cpy: CGFloat; x: CGFloat; y: CGFloat ); external name '_CGContextAddQuadCurveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 * part of the graphics state. }
=======
   part of the graphics state. }
>>>>>>> origin/fixes_2.4

{ Begin a new path. The old path is discarded. }

procedure CGContextBeginPath( c: CGContextRef ); external name '_CGContextBeginPath';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Start a new subpath at point `(x, y)' in the context's path. }

procedure CGContextMoveToPoint( c: CGContextRef; x: CGFloat; y: CGFloat ); external name '_CGContextMoveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Append a straight line segment from the current point to `(x, y)'. }

procedure CGContextAddLineToPoint( c: CGContextRef; x: CGFloat; y: CGFloat ); external name '_CGContextAddLineToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Append a cubic Bezier curve from the current point to `(x,y)', with
   control points `(cp1x, cp1y)' and `(cp2x, cp2y)'. }

procedure CGContextAddCurveToPoint( c: CGContextRef; cp1x: CGFloat; cp1y: CGFloat; cp2x: CGFloat; cp2y: CGFloat; x: CGFloat; y: CGFloat ); external name '_CGContextAddCurveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Append a quadratic curve from the current point to `(x, y)', with control
   point `(cpx, cpy)'. }

<<<<<<< HEAD
procedure CGContextAddQuadCurveToPoint( c: CGContextRef; cpx: Float32; cpy: Float32; x: Float32; y: Float32 ); external name '_CGContextAddQuadCurveToPoint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CGContextAddQuadCurveToPoint( c: CGContextRef; cpx: CGFloat; cpy: CGFloat; x: CGFloat; y: CGFloat ); external name '_CGContextAddQuadCurveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Close the current subpath of the context's path. }

procedure CGContextClosePath( c: CGContextRef ); external name '_CGContextClosePath';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Path construction convenience functions. *}

{ Add a single rect to the context's path. }

procedure CGContextAddRect( c: CGContextRef; rect: CGRect ); external name '_CGContextAddRect';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Add a set of rects to the context's path. }

procedure CGContextAddRects( c: CGContextRef; {const} rects: {variable-size-array} CGRectPtr; count: size_t ); external name '_CGContextAddRects';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Add a set of lines to the context's path. }

procedure CGContextAddLines( c: CGContextRef; {const} points: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextAddLines';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Add an ellipse inside `rect' to the current path of `context'. See the
   function `CGPathAddEllipseInRect' for more information on how the path
   for the ellipse is constructed. }

procedure CGContextAddEllipseInRect( context: CGContextRef; rect: CGRect ); external name '_CGContextAddEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Add an arc of a circle to the context's path, possibly preceded by a
   straight line segment. `(x, y)' is the center of the arc; `radius' is its
   radius; `startAngle' is the angle to the first endpoint of the arc;
   `endAngle' is the angle to the second endpoint of the arc; and
   `clockwise' is 1 if the arc is to be drawn clockwise, 0 otherwise.
   `startAngle' and `endAngle' are measured in radians. }

procedure CGContextAddArc( c: CGContextRef; x: CGFloat; y: CGFloat; radius: CGFloat; startAngle: CGFloat; endAngle: CGFloat; clockwise: SInt32 ); external name '_CGContextAddArc';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Add an arc of a circle to the context's path, possibly preceded by a
   straight line segment. `radius' is the radius of the arc. The arc is
   tangent to the line from the current point to `(x1, y1)', and the line
   from `(x1, y1)' to `(x2, y2)'. }

procedure CGContextAddArcToPoint( c: CGContextRef; x1: CGFloat; y1: CGFloat; x2: CGFloat; y2: CGFloat; radius: CGFloat ); external name '_CGContextAddArcToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Add `path' to the path of context. The points in `path' are transformed
   by the CTM of context before they are added. }

procedure CGContextAddPath( context: CGContextRef; path: CGPathRef ); external name '_CGContextAddPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{* Path stroking. *}

{ Replace the path in `context' with the stroked version of the path, using
   the parameters of `context' to calculate the stroked path. The resulting
   path is created such that filling it with the appropriate color will
   produce the same results as stroking the original path. You can use this
   path in the same way you can use the path of any context; for example,
   you can clip to the stroked version of a path by calling this function
   followed by a call to "CGContextClip". }

procedure CGContextReplacePathWithStrokedPath( c: CGContextRef ); external name '_CGContextReplacePathWithStrokedPath';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{* Path information functions. *}

{ Return true if the path of `context' contains no elements, false
   otherwise. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function CGContextIsPathEmpty( context: CGContextRef ): CBool; external name '_CGContextIsPathEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the current point of the current subpath of the path of
   `context'. }

function CGContextGetPathCurrentPoint( context: CGContextRef ): CGPoint; external name '_CGContextGetPathCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bounding box of the path of `context'. The bounding box is the
   smallest rectangle completely enclosing all points in the path, including
   control points for Bezier and quadratic curves. }

function CGContextGetPathBoundingBox( context: CGContextRef ): CGRect; external name '_CGContextGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of the path of `context'. The returned path is specified in
   the current user space of `context'. }

function CGContextCopyPath( context: CGContextRef ): CGPathRef; external name '_CGContextCopyPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `point' is contained in the current path of `context'. A
   point is contained within a context's path if it is inside the painted
   region when the path is stroked or filled with opaque colors using the
   path drawing mode `mode'. `point' is specified is user space. }

=======

function CGContextIsPathEmpty( context: CGContextRef ): CBool; external name '_CGContextIsPathEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the current point of the current subpath of the path of
   `context'. }

function CGContextGetPathCurrentPoint( context: CGContextRef ): CGPoint; external name '_CGContextGetPathCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bounding box of the path of `context'. The bounding box is the
   smallest rectangle completely enclosing all points in the path, including
   control points for Bezier and quadratic curves. }

function CGContextGetPathBoundingBox( context: CGContextRef ): CGRect; external name '_CGContextGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of the path of `context'. The returned path is specified in
   the current user space of `context'. }

function CGContextCopyPath( context: CGContextRef ): CGPathRef; external name '_CGContextCopyPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

=======

function CGContextIsPathEmpty( context: CGContextRef ): CBool; external name '_CGContextIsPathEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the current point of the current subpath of the path of
   `context'. }

function CGContextGetPathCurrentPoint( context: CGContextRef ): CGPoint; external name '_CGContextGetPathCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bounding box of the path of `context'. The bounding box is the
   smallest rectangle completely enclosing all points in the path, including
   control points for Bezier and quadratic curves. }

function CGContextGetPathBoundingBox( context: CGContextRef ): CGRect; external name '_CGContextGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of the path of `context'. The returned path is specified in
   the current user space of `context'. }

function CGContextCopyPath( context: CGContextRef ): CGPathRef; external name '_CGContextCopyPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
=======

function CGContextIsPathEmpty( context: CGContextRef ): CBool; external name '_CGContextIsPathEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the current point of the current subpath of the path of
   `context'. }

function CGContextGetPathCurrentPoint( context: CGContextRef ): CGPoint; external name '_CGContextGetPathCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bounding box of the path of `context'. The bounding box is the
   smallest rectangle completely enclosing all points in the path, including
   control points for Bezier and quadratic curves. }

function CGContextGetPathBoundingBox( context: CGContextRef ): CGRect; external name '_CGContextGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of the path of `context'. The returned path is specified in
   the current user space of `context'. }

function CGContextCopyPath( context: CGContextRef ): CGPathRef; external name '_CGContextCopyPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
=======

function CGContextIsPathEmpty( context: CGContextRef ): CBool; external name '_CGContextIsPathEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the current point of the current subpath of the path of
   `context'. }

function CGContextGetPathCurrentPoint( context: CGContextRef ): CGPoint; external name '_CGContextGetPathCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bounding box of the path of `context'. The bounding box is the
   smallest rectangle completely enclosing all points in the path, including
   control points for Bezier and quadratic curves. }

function CGContextGetPathBoundingBox( context: CGContextRef ): CGRect; external name '_CGContextGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of the path of `context'. The returned path is specified in
   the current user space of `context'. }

function CGContextCopyPath( context: CGContextRef ): CGPathRef; external name '_CGContextCopyPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

>>>>>>> origin/cpstrnew
{ Return true if `point' is contained in the current path of `context'. A
   point is contained within a context's path if it is inside the painted
   region when the path is stroked or filled with opaque colors using the
   path drawing mode `mode'. `point' is specified is user space. }

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
function CGContextPathContainsPoint( context: CGContextRef; point: CGPoint; mode: CGPathDrawingMode ): CBool; external name '_CGContextPathContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Add an ellipse inside `rect' to the current path of `context'. See the
   function `CGPathAddEllipseInRect' for more information on how the path
   for the ellipse is constructed. }

procedure CGContextAddEllipseInRect( context: CGContextRef; rect: CGRect ); external name '_CGContextAddEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Add an arc of a circle to the context's path, possibly preceded by a
   straight line segment. `(x, y)' is the center of the arc; `radius' is its
   radius; `startAngle' is the angle to the first endpoint of the arc;
   `endAngle' is the angle to the second endpoint of the arc; and
   `clockwise' is 1 if the arc is to be drawn clockwise, 0 otherwise.
   `startAngle' and `endAngle' are measured in radians. }

procedure CGContextAddArc( c: CGContextRef; x: CGFloat; y: CGFloat; radius: CGFloat; startAngle: CGFloat; endAngle: CGFloat; clockwise: SInt32 ); external name '_CGContextAddArc';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Add an arc of a circle to the context's path, possibly preceded by a
   straight line segment. `radius' is the radius of the arc. The arc is
   tangent to the line from the current point to `(x1, y1)', and the line
   from `(x1, y1)' to `(x2, y2)'. }

procedure CGContextAddArcToPoint( c: CGContextRef; x1: CGFloat; y1: CGFloat; x2: CGFloat; y2: CGFloat; radius: CGFloat ); external name '_CGContextAddArcToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Add `path' to the path of context. The points in `path' are transformed
   by the CTM of context before they are added. }

procedure CGContextAddPath( context: CGContextRef; path: CGPathRef ); external name '_CGContextAddPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{* Path stroking. *}

{ Replace the path in `context' with the stroked version of the path, using
   the parameters of `context' to calculate the stroked path. The resulting
   path is created such that filling it with the appropriate color will
   produce the same results as stroking the original path. You can use this
   path in the same way you can use the path of any context; for example,
   you can clip to the stroked version of a path by calling this function
   followed by a call to "CGContextClip". }

procedure CGContextReplacePathWithStrokedPath( c: CGContextRef ); external name '_CGContextReplacePathWithStrokedPath';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{* Path information functions. *}

{ Return true if the path of `context' contains no elements, false
   otherwise. }

function CGContextIsPathEmpty( context: CGContextRef ): CBool; external name '_CGContextIsPathEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the current point of the current subpath of the path of
   `context'. }

function CGContextGetPathCurrentPoint( context: CGContextRef ): CGPoint; external name '_CGContextGetPathCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bounding box of the path of `context'. The bounding box is the
   smallest rectangle completely enclosing all points in the path, including
   control points for Bezier and quadratic curves. }

function CGContextGetPathBoundingBox( context: CGContextRef ): CGRect; external name '_CGContextGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of the path of `context'. The returned path is specified in
   the current user space of `context'. }

<<<<<<< HEAD
function CGContextPathContainsPoint( context: CGContextRef; point: CGPoint; mode: CGPathDrawingMode ): CBool; external name '_CGContextPathContainsPoint'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGContextCopyPath( context: CGContextRef ): CGPathRef; external name '_CGContextCopyPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `point' is contained in the current path of `context'. A
   point is contained within a context's path if it is inside the painted
   region when the path is stroked or filled with opaque colors using the
   path drawing mode `mode'. `point' is specified is user space. }

function CGContextPathContainsPoint( context: CGContextRef; point: CGPoint; mode: CGPathDrawingMode ): CBool; external name '_CGContextPathContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Path drawing functions. *}

{ Draw the context's path using drawing mode `mode'. }

procedure CGContextDrawPath( c: CGContextRef; mode: CGPathDrawingMode ); external name '_CGContextDrawPath';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Path drawing convenience functions. *}

{ Fill the context's path using the winding-number fill rule. Any open
   subpath of the path is implicitly closed. }

procedure CGContextFillPath( c: CGContextRef ); external name '_CGContextFillPath';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Fill the context's path using the even-odd fill rule. Any open subpath of
   the path is implicitly closed. }

procedure CGContextEOFillPath( c: CGContextRef ); external name '_CGContextEOFillPath';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Path drawing convenience functions. *}

{ Fill the context's path using the winding-number fill rule. Any open
   subpath of the path is implicitly closed. }

procedure CGContextFillPath( c: CGContextRef ); external name '_CGContextFillPath';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Fill the context's path using the even-odd fill rule. Any open subpath of
   the path is implicitly closed. }

procedure CGContextEOFillPath( c: CGContextRef ); external name '_CGContextEOFillPath';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Stroke the context's path. }

procedure CGContextStrokePath( c: CGContextRef ); external name '_CGContextStrokePath';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Fill `rect' with the current fill color. }

procedure CGContextFillRect( c: CGContextRef; rect: CGRect ); external name '_CGContextFillRect';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Fill `rects', an array of `count' CGRects, with the current fill
   color. }

procedure CGContextFillRects( c: CGContextRef; {const} rects: {variable-size-array} CGRectPtr; count: size_t ); external name '_CGContextFillRects';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Fill `rects', an array of `count' CGRects, with the current fill
   color. }

procedure CGContextFillRects( c: CGContextRef; {const} rects: {variable-size-array} CGRectPtr; count: size_t ); external name '_CGContextFillRects';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Stroke `rect' with the current stroke color and the current linewidth. }

procedure CGContextStrokeRect( c: CGContextRef; rect: CGRect ); external name '_CGContextStrokeRect';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Stroke `rect' with the current stroke color, using `width' as the the
   line width. }

procedure CGContextStrokeRectWithWidth( c: CGContextRef; rect: CGRect; width: CGFloat ); external name '_CGContextStrokeRectWithWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Clear `rect' (that is, set the region within the rect to transparent). }

procedure CGContextClearRect( c: CGContextRef; rect: CGRect ); external name '_CGContextClearRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Fill an ellipse (an oval) inside `rect'. }

procedure CGContextFillEllipseInRect( context: CGContextRef; rect: CGRect ); external name '_CGContextFillEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Stroke an ellipse (an oval) inside `rect'. }

procedure CGContextStrokeEllipseInRect( context: CGContextRef; rect: CGRect ); external name '_CGContextStrokeEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Stroke a sequence of line segments one after another in `context'. The
   line segments are specified by `points', an array of `count' CGPoints.
   This function is equivalent to
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

     CGContextBeginPath(context);
     for (k = 0; k < count; k += 2) begin
         CGContextMoveToPoint(context, s[k].x, s[k].y);
         CGContextAddLineToPoint(context, s[k+1].x, s[k+1].y);
     end;
     CGContextStrokePath(context); }

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     CGContextBeginPath(context);
     for (k = 0; k < count; k += 2) begin
         CGContextMoveToPoint(context, s[k].x, s[k].y);
         CGContextAddLineToPoint(context, s[k+1].x, s[k+1].y);
     end;
     CGContextStrokePath(context); }

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
procedure CGContextStrokeLineSegments( c: CGContextRef; {const} points: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextStrokeLineSegments';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Stroke `rect' with the current stroke color, using `width' as the the
   line width. }

procedure CGContextStrokeRectWithWidth( c: CGContextRef; rect: CGRect; width: CGFloat ); external name '_CGContextStrokeRectWithWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Clear `rect' (that is, set the region within the rect to transparent). }

procedure CGContextClearRect( c: CGContextRef; rect: CGRect ); external name '_CGContextClearRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Fill an ellipse (an oval) inside `rect'. }

procedure CGContextFillEllipseInRect( context: CGContextRef; rect: CGRect ); external name '_CGContextFillEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Stroke an ellipse (an oval) inside `rect'. }

procedure CGContextStrokeEllipseInRect( context: CGContextRef; rect: CGRect ); external name '_CGContextStrokeEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Stroke a sequence of line segments one after another in `context'. The
   line segments are specified by `points', an array of `count' CGPoints.
   This function is equivalent to
     CGContextBeginPath(context);
     for (k = 0; k < count; k += 2) begin
         CGContextMoveToPoint(context, s[k].x, s[k].y);
         CGContextAddLineToPoint(context, s[k+1].x, s[k+1].y);
     end;
     CGContextStrokePath(context); }

<<<<<<< HEAD
procedure CGContextStrokeLineSegments( c: CGContextRef; {const} points: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextStrokeLineSegments'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CGContextStrokeLineSegments( c: CGContextRef; {const} points: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextStrokeLineSegments';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Clipping functions. *}

{ Intersect the context's path with the current clip path and use the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   resulting path as the clip path for subsequent rendering operations. Use
   the winding-number fill rule for deciding what's inside the path. }

procedure CGContextClip( c: CGContextRef ); external name '_CGContextClip';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Intersect the context's path with the current clip path and use the
   resulting path as the clip path for subsequent rendering operations. Use
   the even-odd fill rule for deciding what's inside the path. }

procedure CGContextEOClip( c: CGContextRef ); external name '_CGContextEOClip';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Add `mask' transformed to `rect' to the clipping area of `context'. The
   mask, which may be either an image mask or an image, is mapped into the
   specified rectangle and intersected with the current clipping area of the
   context.

   If `mask' is an image mask, then it clips in a manner identical to the
   behavior if it were used with "CGContextDrawImage": it indicates an area
   to be masked out (left unchanged) when drawing. The source samples of the
   image mask determine which points of the clipping area are changed,
   acting as an "inverse alpha": if the value of a source sample in the
   image mask is S, then the corresponding point in the current clipping
   area will be multiplied by an alpha of (1-S). (For example, if S is 1,
   then the point in the clipping area becomes clear, while if S is 0, the
   point in the clipping area is unchanged.

   If `mask' is an image, then it serves as alpha mask and is blended with
   the current clipping area. The source samples of mask determine which
   points of the clipping area are changed: if the value of the source
   sample in mask is S, then the corresponding point in the current clipping
   area will be multiplied by an alpha of S. (For example, if S is 0, then
   the point in the clipping area becomes clear, while if S is 1, the point
   in the clipping area is unchanged.

   If `mask' is an image, then it must be in the DeviceGray color space, may
   not have alpha, and may not be masked by an image mask or masking
   color. }

procedure CGContextClipToMask( c: CGContextRef; rect: CGRect; mask: CGImageRef ); external name '_CGContextClipToMask';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the bounding box of the clip path of `c' in user space. The
   bounding box is the smallest rectangle completely enclosing all points in
   the clip. }

function CGContextGetClipBoundingBox( c: CGContextRef ): CGRect; external name '_CGContextGetClipBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* Clipping convenience functions. *}

{ Intersect the current clipping path with `rect'. Note that this function
   resets the context's path to the empty path. }

procedure CGContextClipToRect( c: CGContextRef; rect: CGRect ); external name '_CGContextClipToRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Intersect the current clipping path with the clipping region formed by
   creating a path consisting of all rects in `rects'. Note that this
   function resets the context's path to the empty path. }

procedure CGContextClipToRects( c: CGContextRef; {const} rects: {variable-size-array} CGRectPtr; count: size_t ); external name '_CGContextClipToRects';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 * resulting path as the clip path for subsequent rendering operations.
 * Use the winding-number fill rule for deciding what's inside the path. }
=======
   resulting path as the clip path for subsequent rendering operations. Use
   the winding-number fill rule for deciding what's inside the path. }
>>>>>>> origin/fixes_2.4

procedure CGContextClip( c: CGContextRef ); external name '_CGContextClip';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Intersect the context's path with the current clip path and use the
   resulting path as the clip path for subsequent rendering operations. Use
   the even-odd fill rule for deciding what's inside the path. }

procedure CGContextEOClip( c: CGContextRef ); external name '_CGContextEOClip';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Add `mask' transformed to `rect' to the clipping area of `context'. The
   mask, which may be either an image mask or an image, is mapped into the
   specified rectangle and intersected with the current clipping area of the
   context.

   If `mask' is an image mask, then it clips in a manner identical to the
   behavior if it were used with "CGContextDrawImage": it indicates an area
   to be masked out (left unchanged) when drawing. The source samples of the
   image mask determine which points of the clipping area are changed,
   acting as an "inverse alpha": if the value of a source sample in the
   image mask is S, then the corresponding point in the current clipping
   area will be multiplied by an alpha of (1-S). (For example, if S is 1,
   then the point in the clipping area becomes clear, while if S is 0, the
   point in the clipping area is unchanged.

   If `mask' is an image, then it serves as alpha mask and is blended with
   the current clipping area. The source samples of mask determine which
   points of the clipping area are changed: if the value of the source
   sample in mask is S, then the corresponding point in the current clipping
   area will be multiplied by an alpha of S. (For example, if S is 0, then
   the point in the clipping area becomes clear, while if S is 1, the point
   in the clipping area is unchanged.

   If `mask' is an image, then it must be in the DeviceGray color space, may
   not have alpha, and may not be masked by an image mask or masking
   color. }

procedure CGContextClipToMask( c: CGContextRef; rect: CGRect; mask: CGImageRef ); external name '_CGContextClipToMask';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the bounding box of the clip path of `c' in user space. The
   bounding box is the smallest rectangle completely enclosing all points in
   the clip. }

function CGContextGetClipBoundingBox( c: CGContextRef ): CGRect; external name '_CGContextGetClipBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* Clipping convenience functions. *}

{ Intersect the current clipping path with `rect'. Note that this function
   resets the context's path to the empty path. }

procedure CGContextClipToRect( c: CGContextRef; rect: CGRect ); external name '_CGContextClipToRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Intersect the current clipping path with the clipping region formed by
   creating a path consisting of all rects in `rects'. Note that this
   function resets the context's path to the empty path. }

procedure CGContextClipToRects( c: CGContextRef; {const} rects: {variable-size-array} CGRectPtr; count: size_t ); external name '_CGContextClipToRects';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Primitive color functions. *}

{ Set the current fill color in the context `c' to `color'. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure CGContextSetFillColorWithColor( c: CGContextRef; color: CGColorRef ); external name '_CGContextSetFillColorWithColor';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Set the current stroke color in the context `c' to `color'. }

procedure CGContextSetStrokeColorWithColor( c: CGContextRef; color: CGColorRef ); external name '_CGContextSetStrokeColorWithColor';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* Color space functions. *}

{ Set the current fill color space in `context' to `space'. As a
   side-effect, set the fill color to a default value appropriate for the
   color space. }

procedure CGContextSetFillColorSpace( context: CGContextRef; space: CGColorSpaceRef ); external name '_CGContextSetFillColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current stroke color space in `context' to `space'. As a
   side-effect, set the stroke color to a default value appropriate for the
   color space. }

procedure CGContextSetStrokeColorSpace( context: CGContextRef; space: CGColorSpaceRef ); external name '_CGContextSetStrokeColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Color functions. *}

{ Set the components of the current fill color in `context' to the values
   specifed by `components'. The number of elements in `components' must be
   one greater than the number of components in the current fill color space
   (N color components + 1 alpha component). The current fill color space
   must not be a pattern color space. }

procedure CGContextSetFillColor( context: CGContextRef; {const} components: {variable-size-array} CGFloatPtr ); external name '_CGContextSetFillColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the components of the current stroke color in `context' to the values
   specifed by `components'. The number of elements in `components' must be
   one greater than the number of components in the current stroke color
   space (N color components + 1 alpha component). The current stroke color
   space must not be a pattern color space. }

procedure CGContextSetStrokeColor( context: CGContextRef; {const} components: {variable-size-array} CGFloatPtr ); external name '_CGContextSetStrokeColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Pattern functions. *}

{ Set the components of the current fill color in `context' to the values
   specifed by `components', and set the current fill pattern to `pattern'.
   The number of elements in `components' must be one greater than the
   number of components in the current fill color space (N color components
   + 1 alpha component). The current fill color space must be a pattern
   color space. }

procedure CGContextSetFillPattern( context: CGContextRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ); external name '_CGContextSetFillPattern';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the components of the current stroke color in `context' to the values
   specifed by `components', and set the current stroke pattern to
   `pattern'. The number of elements in `components' must be one greater
   than the number of components in the current stroke color space (N color
   components + 1 alpha component). The current stroke color space must be a
   pattern color space. }

procedure CGContextSetStrokePattern( context: CGContextRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ); external name '_CGContextSetStrokePattern';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the pattern phase in the current graphics state of `context' to
   `phase'. }

procedure CGContextSetPatternPhase( context: CGContextRef; phase: CGSize ); external name '_CGContextSetPatternPhase';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Color convenience functions. *}

{ Set the current fill color space in `context' to `DeviceGray' and set the
   components of the current fill color to `(gray, alpha)'. }

procedure CGContextSetGrayFillColor( context: CGContextRef; gray: CGFloat; alpha: CGFloat ); external name '_CGContextSetGrayFillColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current stroke color space in `context' to `DeviceGray' and set
   the components of the current stroke color to `(gray, alpha)'. }

procedure CGContextSetGrayStrokeColor( context: CGContextRef; gray: CGFloat; alpha: CGFloat ); external name '_CGContextSetGrayStrokeColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current fill color space in `context' to `DeviceRGB' and set the
   components of the current fill color to `(red, green, blue, alpha)'. }

procedure CGContextSetRGBFillColor( context: CGContextRef; red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat ); external name '_CGContextSetRGBFillColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current stroke color space in `context' to `DeviceRGB' and set
   the components of the current stroke color to `(red, green, blue,
   alpha)'. }

procedure CGContextSetRGBStrokeColor( context: CGContextRef; red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat ); external name '_CGContextSetRGBStrokeColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current fill color space in `context' to `DeviceCMYK' and set the
   components of the current fill color to `(cyan, magenta, yellow, black,
   alpha)'. }

procedure CGContextSetCMYKFillColor( context: CGContextRef; cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat ); external name '_CGContextSetCMYKFillColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current stroke color space in `context' to `DeviceCMYK' and set
   the components of the current stroke color to `(cyan, magenta, yellow,
   black, alpha)'. }

procedure CGContextSetCMYKStrokeColor( context: CGContextRef; cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat ); external name '_CGContextSetCMYKStrokeColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Rendering intent. *}

{ Set the rendering intent in the current graphics state of `context' to
   `intent'. }

procedure CGContextSetRenderingIntent( context: CGContextRef; intent: CGColorRenderingIntent ); external name '_CGContextSetRenderingIntent';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
procedure CGContextSetFillColorWithColor( c: CGContextRef; color: CGColorRef ); external name '_CGContextSetFillColorWithColor'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
procedure CGContextSetFillColorWithColor( c: CGContextRef; color: CGColorRef ); external name '_CGContextSetFillColorWithColor';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Set the current stroke color in the context `c' to `color'. }

procedure CGContextSetStrokeColorWithColor( c: CGContextRef; color: CGColorRef ); external name '_CGContextSetStrokeColorWithColor';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* Color space functions. *}

{ Set the current fill color space in `context' to `space'. As a
   side-effect, set the fill color to a default value appropriate for the
   color space. }

procedure CGContextSetFillColorSpace( context: CGContextRef; space: CGColorSpaceRef ); external name '_CGContextSetFillColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current stroke color space in `context' to `space'. As a
   side-effect, set the stroke color to a default value appropriate for the
   color space. }

procedure CGContextSetStrokeColorSpace( context: CGContextRef; space: CGColorSpaceRef ); external name '_CGContextSetStrokeColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Color functions. *}

{ Set the components of the current fill color in `context' to the values
   specifed by `components'. The number of elements in `components' must be
   one greater than the number of components in the current fill color space
   (N color components + 1 alpha component). The current fill color space
   must not be a pattern color space. }

procedure CGContextSetFillColor( context: CGContextRef; {const} components: {variable-size-array} CGFloatPtr ); external name '_CGContextSetFillColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the components of the current stroke color in `context' to the values
   specifed by `components'. The number of elements in `components' must be
   one greater than the number of components in the current stroke color
   space (N color components + 1 alpha component). The current stroke color
   space must not be a pattern color space. }

procedure CGContextSetStrokeColor( context: CGContextRef; {const} components: {variable-size-array} CGFloatPtr ); external name '_CGContextSetStrokeColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Pattern functions. *}

{ Set the components of the current fill color in `context' to the values
   specifed by `components', and set the current fill pattern to `pattern'.
   The number of elements in `components' must be one greater than the
   number of components in the current fill color space (N color components
   + 1 alpha component). The current fill color space must be a pattern
   color space. }

procedure CGContextSetFillPattern( context: CGContextRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ); external name '_CGContextSetFillPattern';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the components of the current stroke color in `context' to the values
   specifed by `components', and set the current stroke pattern to
   `pattern'. The number of elements in `components' must be one greater
   than the number of components in the current stroke color space (N color
   components + 1 alpha component). The current stroke color space must be a
   pattern color space. }

procedure CGContextSetStrokePattern( context: CGContextRef; pattern: CGPatternRef; {const} components: {variable-size-array} CGFloatPtr ); external name '_CGContextSetStrokePattern';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the pattern phase in the current graphics state of `context' to
   `phase'. }

procedure CGContextSetPatternPhase( context: CGContextRef; phase: CGSize ); external name '_CGContextSetPatternPhase';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Color convenience functions. *}

{ Set the current fill color space in `context' to `DeviceGray' and set the
   components of the current fill color to `(gray, alpha)'. }

procedure CGContextSetGrayFillColor( context: CGContextRef; gray: CGFloat; alpha: CGFloat ); external name '_CGContextSetGrayFillColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current stroke color space in `context' to `DeviceGray' and set
   the components of the current stroke color to `(gray, alpha)'. }

procedure CGContextSetGrayStrokeColor( context: CGContextRef; gray: CGFloat; alpha: CGFloat ); external name '_CGContextSetGrayStrokeColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current fill color space in `context' to `DeviceRGB' and set the
   components of the current fill color to `(red, green, blue, alpha)'. }

procedure CGContextSetRGBFillColor( context: CGContextRef; red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat ); external name '_CGContextSetRGBFillColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current stroke color space in `context' to `DeviceRGB' and set
   the components of the current stroke color to `(red, green, blue,
   alpha)'. }

procedure CGContextSetRGBStrokeColor( context: CGContextRef; red: CGFloat; green: CGFloat; blue: CGFloat; alpha: CGFloat ); external name '_CGContextSetRGBStrokeColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current fill color space in `context' to `DeviceCMYK' and set the
   components of the current fill color to `(cyan, magenta, yellow, black,
   alpha)'. }

procedure CGContextSetCMYKFillColor( context: CGContextRef; cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat ); external name '_CGContextSetCMYKFillColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the current stroke color space in `context' to `DeviceCMYK' and set
   the components of the current stroke color to `(cyan, magenta, yellow,
   black, alpha)'. }

procedure CGContextSetCMYKStrokeColor( context: CGContextRef; cyan: CGFloat; magenta: CGFloat; yellow: CGFloat; black: CGFloat; alpha: CGFloat ); external name '_CGContextSetCMYKStrokeColor';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Rendering intent. *}

{ Set the rendering intent in the current graphics state of `context' to
   `intent'. }

<<<<<<< HEAD
procedure CGContextSetRenderingIntent( c: CGContextRef; intent: CGColorRenderingIntent ); external name '_CGContextSetRenderingIntent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CGContextSetRenderingIntent( context: CGContextRef; intent: CGColorRenderingIntent ); external name '_CGContextSetRenderingIntent';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Image functions. *}

{ Draw `image' in the rectangular area specified by `rect' in the context
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   `c'. The image is scaled, if necessary, to fit into `rect'. }

procedure CGContextDrawImage( c: CGContextRef; rect: CGRect; image: CGImageRef ); external name '_CGContextDrawImage';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Draw `image' tiled in the context `c'. The image is scaled to the size
   specified by `rect' in user space, positioned at the origin of `rect' in
   user space, then replicated, stepping the width of `rect' horizontally
   and the height of `rect' vertically, to fill the current clip region.
   Unlike patterns, the image is tiled in user space, so transformations
   applied to the CTM affect the final result. }

procedure CGContextDrawTiledImage( c: CGContextRef; rect: CGRect; image: CGImageRef ); external name '_CGContextDrawTiledImage';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the interpolation quality for image rendering of `context'. The
   interpolation quality is a gstate parameter which controls the level of
   interpolation performed when an image is interpolated (for example, when
   scaling the image). Note that it is merely a hint to the context: not all
   contexts support all interpolation quality levels. }

function CGContextGetInterpolationQuality( context: CGContextRef ): CGInterpolationQuality; external name '_CGContextGetInterpolationQuality';
<<<<<<< HEAD
<<<<<<< HEAD
=======

{ Draw `image' tiled in the context `c'. The image is scaled to the size
   specified by `rect' in user space, positioned at the origin of `rect' in
   user space, then replicated, stepping the width of `rect' horizontally
   and the height of `rect' vertically, to fill the current clip region.
   Unlike patterns, the image is tiled in user space, so transformations
   applied to the CTM affect the final result. }

procedure CGContextDrawTiledImage( c: CGContextRef; rect: CGRect; image: CGImageRef ); external name '_CGContextDrawTiledImage';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the interpolation quality for image rendering of `context'. The
   interpolation quality is a gstate parameter which controls the level of
   interpolation performed when an image is interpolated (for example, when
   scaling the image). Note that it is merely a hint to the context: not all
   contexts support all interpolation quality levels. }

function CGContextGetInterpolationQuality( context: CGContextRef ): CGInterpolationQuality; external name '_CGContextGetInterpolationQuality';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the interpolation quality of `context' to `quality'. }

procedure CGContextSetInterpolationQuality( context: CGContextRef; quality: CGInterpolationQuality ); external name '_CGContextSetInterpolationQuality';
>>>>>>> graemeg/cpstrnew
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

=======

<<<<<<< HEAD
{ Draw `image' tiled in the context `c'. The image is scaled to the size
   specified by `rect' in user space, positioned at the origin of `rect' in
   user space, then replicated, stepping the width of `rect' horizontally
   and the height of `rect' vertically, to fill the current clip region.
   Unlike patterns, the image is tiled in user space, so transformations
   applied to the CTM affect the final result. }

procedure CGContextDrawTiledImage( c: CGContextRef; rect: CGRect; image: CGImageRef ); external name '_CGContextDrawTiledImage';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the interpolation quality for image rendering of `context'. The
   interpolation quality is a gstate parameter which controls the level of
   interpolation performed when an image is interpolated (for example, when
   scaling the image). Note that it is merely a hint to the context: not all
   contexts support all interpolation quality levels. }

function CGContextGetInterpolationQuality( context: CGContextRef ): CGInterpolationQuality; external name '_CGContextGetInterpolationQuality';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

>>>>>>> origin/cpstrnew
{ Set the interpolation quality of `context' to `quality'. }

procedure CGContextSetInterpolationQuality( context: CGContextRef; quality: CGInterpolationQuality ); external name '_CGContextSetInterpolationQuality';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Shadow support. *}

{ Set the shadow parameters in `context'. `offset' specifies a translation
   in base-space; `blur' is a non-negative number specifying the amount of
   blur; `color' specifies the color of the shadow, which may contain a
   non-opaque alpha value. If `color' is NULL, it's equivalent to specifying
   a fully transparent color. The shadow is a gstate parameter. After a
   shadow is specified, all objects drawn subsequently will be shadowed. To
   turn off shadowing, set the shadow color to a fully transparent color (or
   pass NULL as the color), or use the standard gsave/grestore mechanism. }

procedure CGContextSetShadowWithColor( context: CGContextRef; offset: CGSize; blur: CGFloat; color: CGColorRef ); external name '_CGContextSetShadowWithColor';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Equivalent to calling
     CGContextSetShadowWithColor(context, offset, blur, color)
   where color is black with 1/3 alpha (i.e., RGBA = (0, 0, 0, 1.0/3.0)) in
   the DeviceRGB color space. }

procedure CGContextSetShadow( context: CGContextRef; offset: CGSize; blur: CGFloat ); external name '_CGContextSetShadow';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* Gradient and shading functions. *}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Fill the current clipping region of `context' with a linear gradient from
   `startPoint' to `endPoint'. The location 0 of `gradient' corresponds to
   `startPoint'; the location 1 of `gradient' corresponds to `endPoint';
   colors are linearly interpolated between these two points based on the
   values of the gradient's locations. The option flags control whether the
   gradient is drawn before the start point or after the end point. }

procedure CGContextDrawLinearGradient( context: CGContextRef; gradient: CGGradientRef; startPoint: CGPoint; endPoint: CGPoint; options: CGGradientDrawingOptions ); external name '_CGContextDrawLinearGradient';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
<<<<<<< HEAD

{ Fill the current clipping region of `context' with a radial gradient
   between two circles defined by the center point and radius of each
   circle. The location 0 of `gradient' corresponds to a circle centered at
   `startCenter' with radius `startRadius'; the location 1 of `gradient'
   corresponds to a circle centered at `endCenter' with radius `endRadius';
   colors are linearly interpolated between these two circles based on the
   values of the gradient's locations. The option flags control whether the
   gradient is drawn before the start circle or after the end circle. }
<<<<<<< HEAD

procedure CGContextDrawRadialGradient( context: CGContextRef; gradient: CGGradientRef; startCenter: CGPoint; startRadius: CGFloat; endCenter: CGPoint; endRadius: CGFloat; options: CGGradientDrawingOptions ); external name '_CGContextDrawRadialGradient';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Fill the current clipping region of `context' with `shading'. }

=======

{ Fill the current clipping region of `context' with a linear gradient from
   `startPoint' to `endPoint'. The location 0 of `gradient' corresponds to
   `startPoint'; the location 1 of `gradient' corresponds to `endPoint';
   colors are linearly interpolated between these two points based on the
   values of the gradient's locations. The option flags control whether the
   gradient is drawn before the start point or after the end point. }

procedure CGContextDrawLinearGradient( context: CGContextRef; gradient: CGGradientRef; startPoint: CGPoint; endPoint: CGPoint; options: CGGradientDrawingOptions ); external name '_CGContextDrawLinearGradient';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Fill the current clipping region of `context' with a radial gradient
   between two circles defined by the center point and radius of each
   circle. The location 0 of `gradient' corresponds to a circle centered at
   `startCenter' with radius `startRadius'; the location 1 of `gradient'
   corresponds to a circle centered at `endCenter' with radius `endRadius';
   colors are linearly interpolated between these two circles based on the
   values of the gradient's locations. The option flags control whether the
   gradient is drawn before the start circle or after the end circle. }

=======
{ Set the shadow parameters in `context'. `offset' specifies a translation
   in base-space; `blur' is a non-negative number specifying the amount of
   blur; `color' specifies the color of the shadow, which may contain a
   non-opaque alpha value. If `color' is NULL, it's equivalent to specifying
   a fully transparent color. The shadow is a gstate parameter. After a
   shadow is specified, all objects drawn subsequently will be shadowed. To
   turn off shadowing, set the shadow color to a fully transparent color (or
   pass NULL as the color), or use the standard gsave/grestore mechanism. }

procedure CGContextSetShadowWithColor( context: CGContextRef; offset: CGSize; blur: CGFloat; color: CGColorRef ); external name '_CGContextSetShadowWithColor';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Equivalent to calling
     CGContextSetShadowWithColor(context, offset, blur, color)
   where color is black with 1/3 alpha (i.e., RGBA = (0, 0, 0, 1.0/3.0)) in
   the DeviceRGB color space. }

procedure CGContextSetShadow( context: CGContextRef; offset: CGSize; blur: CGFloat ); external name '_CGContextSetShadow';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* Gradient and shading functions. *}

{ Fill the current clipping region of `context' with a linear gradient from
   `startPoint' to `endPoint'. The location 0 of `gradient' corresponds to
   `startPoint'; the location 1 of `gradient' corresponds to `endPoint';
   colors are linearly interpolated between these two points based on the
   values of the gradient's locations. The option flags control whether the
   gradient is drawn before the start point or after the end point. }

procedure CGContextDrawLinearGradient( context: CGContextRef; gradient: CGGradientRef; startPoint: CGPoint; endPoint: CGPoint; options: CGGradientDrawingOptions ); external name '_CGContextDrawLinearGradient';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

=======

>>>>>>> origin/cpstrnew
{ Fill the current clipping region of `context' with a radial gradient
   between two circles defined by the center point and radius of each
   circle. The location 0 of `gradient' corresponds to a circle centered at
   `startCenter' with radius `startRadius'; the location 1 of `gradient'
   corresponds to a circle centered at `endCenter' with radius `endRadius';
   colors are linearly interpolated between these two circles based on the
   values of the gradient's locations. The option flags control whether the
   gradient is drawn before the start circle or after the end circle. }

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure CGContextDrawRadialGradient( context: CGContextRef; gradient: CGGradientRef; startCenter: CGPoint; startRadius: CGFloat; endCenter: CGPoint; endRadius: CGFloat; options: CGGradientDrawingOptions ); external name '_CGContextDrawRadialGradient';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Fill the current clipping region of `context' with `shading'. }

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
procedure CGContextDrawShading( context: CGContextRef; shading: CGShadingRef ); external name '_CGContextDrawShading';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{* Text functions. *}

{ Set the current character spacing in `context' to `spacing'. The
   character spacing is added to the displacement between the origin of one
   character and the origin of the next. }

procedure CGContextSetCharacterSpacing( context: CGContextRef; spacing: CGFloat ); external name '_CGContextSetCharacterSpacing';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the user-space point at which text will be drawn in the context `c'
   to `(x, y)'. }

procedure CGContextSetTextPosition( c: CGContextRef; x: CGFloat; y: CGFloat ); external name '_CGContextSetTextPosition';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the user-space point at which text will be drawn in `context'. }

function CGContextGetTextPosition( context: CGContextRef ): CGPoint; external name '_CGContextGetTextPosition';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 * `c'.  The image is scaled, if necessary, to fit into `rect'. }
=======
   `c'. The image is scaled, if necessary, to fit into `rect'. }
>>>>>>> origin/fixes_2.4

procedure CGContextDrawImage( c: CGContextRef; rect: CGRect; image: CGImageRef ); external name '_CGContextDrawImage';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `image' tiled in the context `c'. The image is scaled to the size
   specified by `rect' in user space, positioned at the origin of `rect' in
   user space, then replicated, stepping the width of `rect' horizontally
   and the height of `rect' vertically, to fill the current clip region.
   Unlike patterns, the image is tiled in user space, so transformations
   applied to the CTM affect the final result. }

procedure CGContextDrawTiledImage( c: CGContextRef; rect: CGRect; image: CGImageRef ); external name '_CGContextDrawTiledImage';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return the interpolation quality for image rendering of `context'. The
   interpolation quality is a gstate parameter which controls the level of
   interpolation performed when an image is interpolated (for example, when
   scaling the image). Note that it is merely a hint to the context: not all
   contexts support all interpolation quality levels. }

function CGContextGetInterpolationQuality( context: CGContextRef ): CGInterpolationQuality; external name '_CGContextGetInterpolationQuality';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the interpolation quality of `context' to `quality'. }

procedure CGContextSetInterpolationQuality( context: CGContextRef; quality: CGInterpolationQuality ); external name '_CGContextSetInterpolationQuality';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Shadow support. *}

{ Set the shadow parameters in `context'. `offset' specifies a translation
   in base-space; `blur' is a non-negative number specifying the amount of
   blur; `color' specifies the color of the shadow, which may contain a
   non-opaque alpha value. If `color' is NULL, it's equivalent to specifying
   a fully transparent color. The shadow is a gstate parameter. After a
   shadow is specified, all objects drawn subsequently will be shadowed. To
   turn off shadowing, set the shadow color to a fully transparent color (or
   pass NULL as the color), or use the standard gsave/grestore mechanism. }

procedure CGContextSetShadowWithColor( context: CGContextRef; offset: CGSize; blur: CGFloat; color: CGColorRef ); external name '_CGContextSetShadowWithColor';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Equivalent to calling
     CGContextSetShadowWithColor(context, offset, blur, color)
   where color is black with 1/3 alpha (i.e., RGBA = (0, 0, 0, 1.0/3.0)) in
   the DeviceRGB color space. }

procedure CGContextSetShadow( context: CGContextRef; offset: CGSize; blur: CGFloat ); external name '_CGContextSetShadow';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* Gradient and shading functions. *}

{ Fill the current clipping region of `context' with a linear gradient from
   `startPoint' to `endPoint'. The location 0 of `gradient' corresponds to
   `startPoint'; the location 1 of `gradient' corresponds to `endPoint';
   colors are linearly interpolated between these two points based on the
   values of the gradient's locations. The option flags control whether the
   gradient is drawn before the start point or after the end point. }

procedure CGContextDrawLinearGradient( context: CGContextRef; gradient: CGGradientRef; startPoint: CGPoint; endPoint: CGPoint; options: CGGradientDrawingOptions ); external name '_CGContextDrawLinearGradient';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Fill the current clipping region of `context' with a radial gradient
   between two circles defined by the center point and radius of each
   circle. The location 0 of `gradient' corresponds to a circle centered at
   `startCenter' with radius `startRadius'; the location 1 of `gradient'
   corresponds to a circle centered at `endCenter' with radius `endRadius';
   colors are linearly interpolated between these two circles based on the
   values of the gradient's locations. The option flags control whether the
   gradient is drawn before the start circle or after the end circle. }

procedure CGContextDrawRadialGradient( context: CGContextRef; gradient: CGGradientRef; startCenter: CGPoint; startRadius: CGFloat; endCenter: CGPoint; endRadius: CGFloat; options: CGGradientDrawingOptions ); external name '_CGContextDrawRadialGradient';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Fill the current clipping region of `context' with `shading'. }

procedure CGContextDrawShading( context: CGContextRef; shading: CGShadingRef ); external name '_CGContextDrawShading';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{* Text functions. *}

{ Set the current character spacing in `context' to `spacing'. The
   character spacing is added to the displacement between the origin of one
   character and the origin of the next. }

procedure CGContextSetCharacterSpacing( context: CGContextRef; spacing: CGFloat ); external name '_CGContextSetCharacterSpacing';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the user-space point at which text will be drawn in the context `c'
   to `(x, y)'. }

procedure CGContextSetTextPosition( c: CGContextRef; x: CGFloat; y: CGFloat ); external name '_CGContextSetTextPosition';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the user-space point at which text will be drawn in `context'. }

<<<<<<< HEAD
function CGContextGetTextPosition( c: CGContextRef ): CGPoint; external name '_CGContextGetTextPosition';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGContextGetTextPosition( context: CGContextRef ): CGPoint; external name '_CGContextGetTextPosition';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Set the text matrix in the context `c' to `t'. }

procedure CGContextSetTextMatrix( c: CGContextRef; t: CGAffineTransform ); external name '_CGContextSetTextMatrix';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the text matrix in the context `c'. }

function CGContextGetTextMatrix( c: CGContextRef ): CGAffineTransform; external name '_CGContextGetTextMatrix';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the text drawing mode in the current graphics state of the context
   `c' to `mode'. }

procedure CGContextSetTextDrawingMode( c: CGContextRef; mode: CGTextDrawingMode ); external name '_CGContextSetTextDrawingMode';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the font in the current graphics state of the context `c' to
   `font'. }

procedure CGContextSetFont( c: CGContextRef; font: CGFontRef ); external name '_CGContextSetFont';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the font size in the current graphics state of the context `c' to
   `size'. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure CGContextSetFontSize( c: CGContextRef; size: CGFloat ); external name '_CGContextSetFontSize';
=======

procedure CGContextSetFontSize( c: CGContextRef; size: CGFloat ); external name '_CGContextSetFontSize';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Attempts to find the font named `name' and, if successful, sets it as the
   font in the current graphics state of `c' and sets the font size in the
   current graphics state to `size'. `textEncoding' specifies how to
   translate from bytes to glyphs when displaying text. }

procedure CGContextSelectFont( c: CGContextRef; name: ConstCStringPtr; size: CGFloat; textEncoding: CGTextEncoding ); external name '_CGContextSelectFont';
>>>>>>> graemeg/cpstrnew
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the points specified by
   `positions'. Each element of `positions' specifies the position from the
   associated glyph; the positions are specified in user space. }
<<<<<<< HEAD

procedure CGContextShowGlyphsAtPositions( context: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} positions: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextShowGlyphsAtPositions';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew

procedure CGContextSetFontSize( c: CGContextRef; size: CGFloat ); external name '_CGContextSetFontSize';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Attempts to find the font named `name' and, if successful, sets it as the
   font in the current graphics state of `c' and sets the font size in the
   current graphics state to `size'. `textEncoding' specifies how to
   translate from bytes to glyphs when displaying text. }

procedure CGContextSelectFont( c: CGContextRef; name: ConstCStringPtr; size: CGFloat; textEncoding: CGTextEncoding ); external name '_CGContextSelectFont';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the points specified by
   `positions'. Each element of `positions' specifies the position from the
   associated glyph; the positions are specified in user space. }

procedure CGContextShowGlyphsAtPositions( context: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} positions: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextShowGlyphsAtPositions';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
<<<<<<< HEAD
=======

procedure CGContextShowGlyphsAtPositions( context: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} positions: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextShowGlyphsAtPositions';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew

{* Text convenience functions. *}

{ Draw `string', a string of `length' bytes, at the point specified by the
   text matrix in the context `c'. Each byte of the string is mapped through
   the encoding vector of the current font to obtain the glyph to
   display. }

procedure CGContextShowText( c: CGContextRef; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowText';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD
=======

procedure CGContextSetFontSize( c: CGContextRef; size: CGFloat ); external name '_CGContextSetFontSize';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Attempts to find the font named `name' and, if successful, sets it as the
   font in the current graphics state of `c' and sets the font size in the
   current graphics state to `size'. `textEncoding' specifies how to
   translate from bytes to glyphs when displaying text. }

procedure CGContextSelectFont( c: CGContextRef; name: ConstCStringPtr; size: CGFloat; textEncoding: CGTextEncoding ); external name '_CGContextSelectFont';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the points specified by
   `positions'. Each element of `positions' specifies the position from the
   associated glyph; the positions are specified in user space. }

procedure CGContextShowGlyphsAtPositions( context: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} positions: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextShowGlyphsAtPositions';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{* Text convenience functions. *}
>>>>>>> origin/cpstrnew

{ Draw `string', a string of `length' bytes, at the point `(x, y)',
   specified in user space, in the context `c'. Each byte of the string is
   mapped through the encoding vector of the current font to obtain the
   glyph to display. }

procedure CGContextShowTextAtPoint( c: CGContextRef; x: CGFloat; y: CGFloat; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowTextAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw the glyphs pointed to by `g', an array of `count' glyphs, at the
   point specified by the text matrix in the context `c'. }
>>>>>>> graemeg/cpstrnew

procedure CGContextShowGlyphs( c: CGContextRef; {const} g: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphs';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

<<<<<<< HEAD
{ Draw `string', a string of `length' bytes, at the point specified by the
   text matrix in the context `c'. Each byte of the string is mapped through
   the encoding vector of the current font to obtain the glyph to
   display. }

procedure CGContextShowText( c: CGContextRef; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowText';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD

{ Draw `string', a string of `length' bytes, at the point `(x, y)',
   specified in user space, in the context `c'. Each byte of the string is
   mapped through the encoding vector of the current font to obtain the
   glyph to display. }

procedure CGContextShowTextAtPoint( c: CGContextRef; x: CGFloat; y: CGFloat; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowTextAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw the glyphs pointed to by `g', an array of `count' glyphs, at the
   point specified by the text matrix in the context `c'. }

procedure CGContextShowGlyphs( c: CGContextRef; {const} g: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphs';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Display the glyphs pointed to by `glyphs', an array of `count' glyphs, at
   the point `(x, y)', specified in user space, in `context'. }

procedure CGContextShowGlyphsAtPoint( context: CGContextRef; x: CGFloat; y: CGFloat; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphsAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the current point
   specified by the text matrix. Each element of `advances' specifies the
   offset from the previous glyph's origin to the origin of the associated
   glyph; the advances are specified in user space. }

procedure CGContextShowGlyphsWithAdvances( c: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} advances: {variable-size-array} CGSizePtr; count: size_t ); external name '_CGContextShowGlyphsWithAdvances';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
>>>>>>> graemeg/cpstrnew
=======
{ Display the glyphs pointed to by `glyphs', an array of `count' glyphs, at
   the point `(x, y)', specified in user space, in `context'. }

procedure CGContextShowGlyphsAtPoint( context: CGContextRef; x: CGFloat; y: CGFloat; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphsAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the current point
   specified by the text matrix. Each element of `advances' specifies the
   offset from the previous glyph's origin to the origin of the associated
   glyph; the advances are specified in user space. }

procedure CGContextShowGlyphsWithAdvances( c: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} advances: {variable-size-array} CGSizePtr; count: size_t ); external name '_CGContextShowGlyphsWithAdvances';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
>>>>>>> graemeg/cpstrnew
=======

{ Draw `string', a string of `length' bytes, at the point `(x, y)',
   specified in user space, in the context `c'. Each byte of the string is
   mapped through the encoding vector of the current font to obtain the
   glyph to display. }

procedure CGContextShowTextAtPoint( c: CGContextRef; x: CGFloat; y: CGFloat; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowTextAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw the glyphs pointed to by `g', an array of `count' glyphs, at the
   point specified by the text matrix in the context `c'. }

procedure CGContextShowGlyphs( c: CGContextRef; {const} g: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphs';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Display the glyphs pointed to by `glyphs', an array of `count' glyphs, at
   the point `(x, y)', specified in user space, in `context'. }

procedure CGContextShowGlyphsAtPoint( context: CGContextRef; x: CGFloat; y: CGFloat; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphsAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the current point
   specified by the text matrix. Each element of `advances' specifies the
   offset from the previous glyph's origin to the origin of the associated
   glyph; the advances are specified in user space. }

procedure CGContextShowGlyphsWithAdvances( c: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} advances: {variable-size-array} CGSizePtr; count: size_t ); external name '_CGContextShowGlyphsWithAdvances';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
>>>>>>> graemeg/cpstrnew
=======

{ Draw `string', a string of `length' bytes, at the point `(x, y)',
   specified in user space, in the context `c'. Each byte of the string is
   mapped through the encoding vector of the current font to obtain the
   glyph to display. }

procedure CGContextShowTextAtPoint( c: CGContextRef; x: CGFloat; y: CGFloat; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowTextAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw the glyphs pointed to by `g', an array of `count' glyphs, at the
   point specified by the text matrix in the context `c'. }

procedure CGContextShowGlyphs( c: CGContextRef; {const} g: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphs';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Display the glyphs pointed to by `glyphs', an array of `count' glyphs, at
   the point `(x, y)', specified in user space, in `context'. }

procedure CGContextShowGlyphsAtPoint( context: CGContextRef; x: CGFloat; y: CGFloat; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphsAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the current point
   specified by the text matrix. Each element of `advances' specifies the
   offset from the previous glyph's origin to the origin of the associated
   glyph; the advances are specified in user space. }

procedure CGContextShowGlyphsWithAdvances( c: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} advances: {variable-size-array} CGSizePtr; count: size_t ); external name '_CGContextShowGlyphsWithAdvances';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Set the text drawing mode in the current graphics state of the context
   `c' to `mode'. }

procedure CGContextSetTextDrawingMode( c: CGContextRef; mode: CGTextDrawingMode ); external name '_CGContextSetTextDrawingMode';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the font in the current graphics state of the context `c' to
   `font'. }

procedure CGContextSetFont( c: CGContextRef; font: CGFontRef ); external name '_CGContextSetFont';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Set the font size in the current graphics state of the context `c' to
   `size'. }

procedure CGContextSetFontSize( c: CGContextRef; size: CGFloat ); external name '_CGContextSetFontSize';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Attempts to find the font named `name' and, if successful, sets it as the
   font in the current graphics state of `c' and sets the font size in the
   current graphics state to `size'. `textEncoding' specifies how to
   translate from bytes to glyphs when displaying text. }

procedure CGContextSelectFont( c: CGContextRef; name: ConstCStringPtr; size: CGFloat; textEncoding: CGTextEncoding ); external name '_CGContextSelectFont';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the points specified by
   `positions'. Each element of `positions' specifies the position from the
   associated glyph; the positions are specified in user space. }

procedure CGContextShowGlyphsAtPositions( context: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} positions: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGContextShowGlyphsAtPositions';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{* Text convenience functions. *}

{ Draw `string', a string of `length' bytes, at the point specified by the
   text matrix in the context `c'. Each byte of the string is mapped through
   the encoding vector of the current font to obtain the glyph to
   display. }

procedure CGContextShowText( c: CGContextRef; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowText';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `string', a string of `length' bytes, at the point `(x, y)',
   specified in user space, in the context `c'. Each byte of the string is
   mapped through the encoding vector of the current font to obtain the
   glyph to display. }

procedure CGContextShowTextAtPoint( c: CGContextRef; x: CGFloat; y: CGFloat; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowTextAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw the glyphs pointed to by `g', an array of `count' glyphs, at the
   point specified by the text matrix in the context `c'. }

procedure CGContextShowGlyphs( c: CGContextRef; {const} g: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphs';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Display the glyphs pointed to by `glyphs', an array of `count' glyphs, at
   the point `(x, y)', specified in user space, in `context'. }

procedure CGContextShowGlyphsAtPoint( context: CGContextRef; x: CGFloat; y: CGFloat; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphsAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Draw `glyphs', an array of `count' CGGlyphs, at the current point
   specified by the text matrix. Each element of `advances' specifies the
   offset from the previous glyph's origin to the origin of the associated
   glyph; the advances are specified in user space. }

<<<<<<< HEAD
procedure CGContextShowGlyphsAtPoint( c: CGContextRef; x: Float32; y: Float32; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphsAtPoint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CGContextShowGlyphsWithAdvances( c: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} advances: {variable-size-array} CGSizePtr; count: size_t ); external name '_CGContextShowGlyphsWithAdvances';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* PDF functions. *}

{ Draw `page' in the current user space of the context `c'. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure CGContextDrawPDFPage( c: CGContextRef; page: CGPDFPageRef ); external name '_CGContextDrawPDFPage';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
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

{$ifc TARGET_OS_MAC}
=======
procedure CGContextDrawPDFPage( c: CGContextRef; page: CGPDFPageRef ); external name '_CGContextDrawPDFPage'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> graemeg/fixes_2_2
=======
procedure CGContextDrawPDFPage( c: CGContextRef; page: CGPDFPageRef ); external name '_CGContextDrawPDFPage'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> origin/fixes_2_2
=======
procedure CGContextDrawPDFPage( c: CGContextRef; page: CGPDFPageRef ); external name '_CGContextDrawPDFPage';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4
{ DEPRECATED; use the CGPDFPage API instead.
 * Draw `page' in `document' in the rectangular area specified by `rect' in
 * the context `c'.  The media box of the page is scaled, if necessary, to
 * fit into `rect'. }

procedure CGContextDrawPDFDocument( c: CGContextRef; rect: CGRect; document: CGPDFDocumentRef; page: SInt32 ); external name '_CGContextDrawPDFDocument';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc}
>>>>>>> origin/fixes_2.4

{* Output page functions. *}

{ Begin a new page. }

procedure CGContextBeginPage( c: CGContextRef; const (*var*) mediaBox: CGRect ); external name '_CGContextBeginPage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ End the current page. }

procedure CGContextEndPage( c: CGContextRef ); external name '_CGContextEndPage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Context functions. *}

{ Equivalent to `CFRetain(c)'. }

function CGContextRetain( c: CGContextRef ): CGContextRef; external name '_CGContextRetain';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Equivalent to `CFRelease(c)'. }

procedure CGContextRelease( c: CGContextRef ); external name '_CGContextRelease';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Flush all drawing to the destination. }

procedure CGContextFlush( c: CGContextRef ); external name '_CGContextFlush';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Synchronized drawing. }

procedure CGContextSynchronize( c: CGContextRef ); external name '_CGContextSynchronize';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{* Antialiasing functions. *}

{ Turn on antialiasing if `shouldAntialias' is true; turn it off otherwise.
   This parameter is part of the graphics state. }

procedure CGContextSetShouldAntialias( context: CGContextRef; shouldAntialias: CBool ); external name '_CGContextSetShouldAntialias';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Allow antialiasing in `context' if `allowsAntialiasing' is true; don't
   allow it otherwise. This parameter is not part of the graphics state. A
   context will perform antialiasing if both `allowsAntialiasing' and the
   graphics state parameter `shouldAntialias' are true. }

procedure CGContextSetAllowsAntialiasing( context: CGContextRef; allowsAntialiasing: CBool ); external name '_CGContextSetAllowsAntialiasing';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{* Font display functions. *}

{ Turn on font smoothing if `shouldSmoothFonts' is true; turn it off
   otherwise. This parameter is part of the graphics state. Note that this
   doesn't guarantee that font smoothing will occur: not all destination
   contexts support font smoothing. }

procedure CGContextSetShouldSmoothFonts( context: CGContextRef; shouldSmoothFonts: CBool ); external name '_CGContextSetShouldSmoothFonts';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ If `allowsFontSmoothing' is true, then allow font smoothing when
   displaying text in `context'; otherwise, don't allow font smoothing. This
   parameter is not part of the graphics state. Fonts will be smoothed if
   they are antialiased when drawn and if both `allowsFontSmoothing' and the
   graphics state parameter `shouldSmoothFonts' are true. }
 
procedure CGContextSetAllowsFontSmoothing( context: CGContextRef; allowsFontSmoothing: CBool ); external name '_CGContextSetAllowsFontSmoothing';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ If `shouldSubpixelPositionFonts' is true, then glyphs may be placed at
   subpixel positions (if allowed) when displaying text in `context';
   otherwise, glyphs will be forced to integer pixel boundaries. This
   parameter is part of the graphics state. }

procedure CGContextSetShouldSubpixelPositionFonts( context: CGContextRef; shouldSubpixelPositionFonts: CBool ); external name '_CGContextSetShouldSubpixelPositionFonts';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ If `allowsFontSubpixelPositioning' is true, then allow font subpixel
   positioning when displaying text in `context'; otherwise, don't allow
   subpixel positioning. This parameter is not part of the graphics state. A
   context will place glyphs at subpixel positions if fonts will be
   antialiased when drawn and if both `allowsFontSubpixelPositioning' and
   the graphics state parameter `shouldSubpixelPositionFonts' are true. }

procedure CGContextSetAllowsFontSubpixelPositioning( context: CGContextRef; allowsFontSubpixelPositioning: CBool ); external name '_CGContextSetAllowsFontSubpixelPositioning';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ If `shouldSubpixelQuantizeFonts' is true, then quantize the subpixel
   positions of glyphs when displaying text in `context'; otherwise, don't
   quantize the subpixel positions. This parameter is part of the graphics
   state. }

procedure CGContextSetShouldSubpixelQuantizeFonts( context: CGContextRef; shouldSubpixelQuantizeFonts: CBool ); external name '_CGContextSetShouldSubpixelQuantizeFonts';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ If `allowsFontSubpixelQuantization' is true, then allow font subpixel
   quantization when displaying text in `context'; otherwise, don't allow
   subpixel quantization. This parameter is not part of the graphics state.
   A context quantizes subpixel positions if glyphs will be drawn at
   subpixel positions and `allowsFontSubpixelQuantization' and the graphics
   state parameter `shouldSubpixelQuantizeFonts' are both true. }

procedure CGContextSetAllowsFontSubpixelQuantization( context: CGContextRef; allowsFontSubpixelQuantization: CBool ); external name '_CGContextSetAllowsFontSubpixelQuantization';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{* Transparency layer support. *}

{ Begin a transparency layer in `context'. All subsequent drawing
   operations until a corresponding `CGContextEndTransparencyLayer' are
   composited into a fully transparent backdrop (which is treated as a
   separate destination buffer from the context). After the transparency
   layer is ended, the result is composited into the context using the
   global alpha and shadow state of the context. This operation respects the
   clipping region of the context. After a call to this function, all of the
   parameters in the graphics state remain unchanged with the exception of
   the following:
     - The global alpha is set to 1.
     - The shadow is turned off.
     - The blend mode is set to `kCGBlendModeNormal'.
   Ending the transparency layer restores these parameters to the values
   they had before `CGContextBeginTransparencyLayer' was called.
   Transparency layers may be nested. }

procedure CGContextBeginTransparencyLayer( context: CGContextRef; auxiliaryInfo: CFDictionaryRef ); external name '_CGContextBeginTransparencyLayer';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Begin a transparency layer in `context'. This function is identical to
  `CGContextBeginTransparencyLayer' except that the content of the
  transparency layer will be bounded by `rect' (specified in user space). }

procedure CGContextBeginTransparencyLayerWithRect( context: CGContextRef; rect: CGRect; auxiliaryInfo: CFDictionaryRef ); external name '_CGContextBeginTransparencyLayerWithRect';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ End a tranparency layer. }

procedure CGContextEndTransparencyLayer( context: CGContextRef ); external name '_CGContextEndTransparencyLayer';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* User space to device space tranformations. *}

{ Return the affine transform mapping the user space (abstract coordinates)
   of `context' to device space (pixels). }

function CGContextGetUserSpaceToDeviceSpaceTransform( context: CGContextRef ): CGAffineTransform; external name '_CGContextGetUserSpaceToDeviceSpaceTransform';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `point' from the user space of `context' to device space. }

function CGContextConvertPointToDeviceSpace( context: CGContextRef; point: CGPoint ): CGPoint; external name '_CGContextConvertPointToDeviceSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `point' from device space to the user space of `context'. }

function CGContextConvertPointToUserSpace( context: CGContextRef; point: CGPoint ): CGPoint; external name '_CGContextConvertPointToUserSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `size' from the user space of `context' to device space. }

function CGContextConvertSizeToDeviceSpace( context: CGContextRef; size: CGSize ): CGSize; external name '_CGContextConvertSizeToDeviceSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `size' from device space to the user space of `context'. }

function CGContextConvertSizeToUserSpace( context: CGContextRef; size: CGSize ): CGSize; external name '_CGContextConvertSizeToUserSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `rect' from the user space of `context' to device space. Since
   affine transforms do not preserve rectangles in general, this function
   returns the smallest rectangle which contains the transformed corner
   points of `rect'. }

function CGContextConvertRectToDeviceSpace( context: CGContextRef; rect: CGRect ): CGRect; external name '_CGContextConvertRectToDeviceSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `rect' from device space to the user space of `context'. Since
   affine transforms do not preserve rectangles in general, this function
   returns the smallest rectangle which contains the transformed corner
   points of `rect'. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function CGContextConvertRectToUserSpace( context: CGContextRef; rect: CGRect ): CGRect; external name '_CGContextConvertRectToUserSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{* Deprecated functions. *}

{ DEPRECATED; use the CoreText API instead. }

procedure CGContextSelectFont( c: CGContextRef; name: ConstCStringPtr; size: CGFloat; textEncoding: CGTextEncoding ); external name '_CGContextSelectFont';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_9, __IPHONE_2_0, __IPHONE_7_0) *)

{ DEPRECATED; use the CoreText API instead. }

procedure CGContextShowText( c: CGContextRef; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowText';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_9, __IPHONE_2_0, __IPHONE_7_0) *)

{ DEPRECATED; use the CoreText API instead. }

procedure CGContextShowTextAtPoint( c: CGContextRef; x: CGFloat; y: CGFloat; strng: ConstCStringPtr; length: size_t ); external name '_CGContextShowTextAtPoint';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_9, __IPHONE_2_0, __IPHONE_7_0) *)

{ DEPRECATED; use the CoreText API instead. }

procedure CGContextShowGlyphs( c: CGContextRef; {const} g: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphs';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_9, __IPHONE_2_0, __IPHONE_7_0) *)

{ DEPRECATED; use the CoreText API instead. }

procedure CGContextShowGlyphsAtPoint( context: CGContextRef; x: CGFloat; y: CGFloat; {const} glyphs: {variable-size-array} CGGlyphPtr; count: size_t ); external name '_CGContextShowGlyphsAtPoint';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_9, __IPHONE_2_0, __IPHONE_7_0) *)

{ DEPRECATED; use the CoreText API instead. }

procedure CGContextShowGlyphsWithAdvances( context: CGContextRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} advances: {variable-size-array} CGSizePtr; count: size_t ); external name '_CGContextShowGlyphsWithAdvances';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_3, __MAC_10_9, __IPHONE_2_0, __IPHONE_7_0) *)

{$ifc TARGET_OS_MAC}

{ DEPRECATED; use the CGPDFPage API instead. }

procedure CGContextDrawPDFDocument( c: CGContextRef; rect: CGRect; document: CGPDFDocumentRef; page: SInt32 ); external name '_CGContextDrawPDFDocument';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)

{$endc}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

function CGContextConvertRectToUserSpace( context: CGContextRef; rect: CGRect ): CGRect; external name '_CGContextConvertRectToUserSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

function CGContextConvertRectToUserSpace( context: CGContextRef; rect: CGRect ): CGRect; external name '_CGContextConvertRectToUserSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

>>>>>>> origin/cpstrnew
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{* Antialiasing functions. *}

{ Turn on antialiasing if `shouldAntialias' is true; turn it off otherwise.
   This parameter is part of the graphics state. }

procedure CGContextSetShouldAntialias( context: CGContextRef; shouldAntialias: CBool ); external name '_CGContextSetShouldAntialias';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Allow antialiasing in `context' if `allowsAntialiasing' is true; don't
   allow it otherwise. This parameter is not part of the graphics state. A
   context will perform antialiasing if both `allowsAntialiasing' and the
   graphics state parameter `shouldAntialias' are true. }

procedure CGContextSetAllowsAntialiasing( context: CGContextRef; allowsAntialiasing: CBool ); external name '_CGContextSetAllowsAntialiasing';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{* Font display functions. *}

{ Turn on font smoothing if `shouldSmoothFonts' is true; turn it off
   otherwise. This parameter is part of the graphics state. Note that this
   doesn't guarantee that font smoothing will occur: not all destination
   contexts support font smoothing. }

procedure CGContextSetShouldSmoothFonts( context: CGContextRef; shouldSmoothFonts: CBool ); external name '_CGContextSetShouldSmoothFonts';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ If `allowsFontSmoothing' is true, then allow font smoothing when
   displaying text in `context'; otherwise, don't allow font smoothing. This
   parameter is not part of the graphics state. Fonts will be smoothed if
   they are antialiased when drawn and if both `allowsFontSmoothing' and the
   graphics state parameter `shouldSmoothFonts' are true. }
 
procedure CGContextSetAllowsFontSmoothing( context: CGContextRef; allowsFontSmoothing: CBool ); external name '_CGContextSetAllowsFontSmoothing';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ If `shouldSubpixelPositionFonts' is true, then glyphs may be placed at
   subpixel positions (if allowed) when displaying text in `context';
   otherwise, glyphs will be forced to integer pixel boundaries. This
   parameter is part of the graphics state. }

procedure CGContextSetShouldSubpixelPositionFonts( context: CGContextRef; shouldSubpixelPositionFonts: CBool ); external name '_CGContextSetShouldSubpixelPositionFonts';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ If `allowsFontSubpixelPositioning' is true, then allow font subpixel
   positioning when displaying text in `context'; otherwise, don't allow
   subpixel positioning. This parameter is not part of the graphics state. A
   context will place glyphs at subpixel positions if fonts will be
   antialiased when drawn and if both `allowsFontSubpixelPositioning' and
   the graphics state parameter `shouldSubpixelPositionFonts' are true. }

procedure CGContextSetAllowsFontSubpixelPositioning( context: CGContextRef; allowsFontSubpixelPositioning: CBool ); external name '_CGContextSetAllowsFontSubpixelPositioning';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ If `shouldSubpixelQuantizeFonts' is true, then quantize the subpixel
   positions of glyphs when displaying text in `context'; otherwise, don't
   quantize the subpixel positions. This parameter is part of the graphics
   state. }

procedure CGContextSetShouldSubpixelQuantizeFonts( context: CGContextRef; shouldSubpixelQuantizeFonts: CBool ); external name '_CGContextSetShouldSubpixelQuantizeFonts';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ If `allowsFontSubpixelQuantization' is true, then allow font subpixel
   quantization when displaying text in `context'; otherwise, don't allow
   subpixel quantization. This parameter is not part of the graphics state.
   A context quantizes subpixel positions if glyphs will be drawn at
   subpixel positions and `allowsFontSubpixelQuantization' and the graphics
   state parameter `shouldSubpixelQuantizeFonts' are both true. }

procedure CGContextSetAllowsFontSubpixelQuantization( context: CGContextRef; allowsFontSubpixelQuantization: CBool ); external name '_CGContextSetAllowsFontSubpixelQuantization';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{* Transparency layer support. *}

{ Begin a transparency layer in `context'. All subsequent drawing
   operations until a corresponding `CGContextEndTransparencyLayer' are
   composited into a fully transparent backdrop (which is treated as a
   separate destination buffer from the context). After the transparency
   layer is ended, the result is composited into the context using the
   global alpha and shadow state of the context. This operation respects the
   clipping region of the context. After a call to this function, all of the
   parameters in the graphics state remain unchanged with the exception of
   the following:
     - The global alpha is set to 1.
     - The shadow is turned off.
     - The blend mode is set to `kCGBlendModeNormal'.
   Ending the transparency layer restores these parameters to the values
   they had before `CGContextBeginTransparencyLayer' was called.
   Transparency layers may be nested. }

procedure CGContextBeginTransparencyLayer( context: CGContextRef; auxiliaryInfo: CFDictionaryRef ); external name '_CGContextBeginTransparencyLayer';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Begin a transparency layer in `context'. This function is identical to
  `CGContextBeginTransparencyLayer' except that the content of the
  transparency layer will be bounded by `rect' (specified in user space). }

procedure CGContextBeginTransparencyLayerWithRect( context: CGContextRef; rect: CGRect; auxiliaryInfo: CFDictionaryRef ); external name '_CGContextBeginTransparencyLayerWithRect';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ End a tranparency layer. }

procedure CGContextEndTransparencyLayer( context: CGContextRef ); external name '_CGContextEndTransparencyLayer';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{* User space to device space tranformations. *}

{ Return the affine transform mapping the user space (abstract coordinates)
   of `context' to device space (pixels). }

function CGContextGetUserSpaceToDeviceSpaceTransform( context: CGContextRef ): CGAffineTransform; external name '_CGContextGetUserSpaceToDeviceSpaceTransform';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `point' from the user space of `context' to device space. }

function CGContextConvertPointToDeviceSpace( context: CGContextRef; point: CGPoint ): CGPoint; external name '_CGContextConvertPointToDeviceSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `point' from device space to the user space of `context'. }

function CGContextConvertPointToUserSpace( context: CGContextRef; point: CGPoint ): CGPoint; external name '_CGContextConvertPointToUserSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `size' from the user space of `context' to device space. }

function CGContextConvertSizeToDeviceSpace( context: CGContextRef; size: CGSize ): CGSize; external name '_CGContextConvertSizeToDeviceSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `size' from device space to the user space of `context'. }

function CGContextConvertSizeToUserSpace( context: CGContextRef; size: CGSize ): CGSize; external name '_CGContextConvertSizeToUserSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `rect' from the user space of `context' to device space. Since
   affine transforms do not preserve rectangles in general, this function
   returns the smallest rectangle which contains the transformed corner
   points of `rect'. }

function CGContextConvertRectToDeviceSpace( context: CGContextRef; rect: CGRect ): CGRect; external name '_CGContextConvertRectToDeviceSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `rect' from device space to the user space of `context'. Since
   affine transforms do not preserve rectangles in general, this function
   returns the smallest rectangle which contains the transformed corner
   points of `rect'. }

function CGContextConvertRectToUserSpace( context: CGContextRef; rect: CGRect ): CGRect; external name '_CGContextConvertRectToUserSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

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
