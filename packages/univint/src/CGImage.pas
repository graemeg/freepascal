{ CoreGraphics - CGImage.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
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
 * Copyright (c) 2000-2004 Apple Computer, Inc.
 * All rights reserved.
 }
=======
 * Copyright (c) 2000-2008 Apple Inc.
 * All rights reserved. }
>>>>>>> origin/fixes_2.4
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
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

unit CGImage;
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,CFBase,CGBase,CGGeometry,CGColorSpace,CGDataProvider;
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


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGImageRef = ^OpaqueCGImageRef; { an opaque type }
	OpaqueCGImageRef = record end;
=======
	CGImageRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CGImageRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CGImageRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CGImageRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


type
	CGImageAlphaInfo = SInt32;
const
	kCGImageAlphaNone = 0;               { For example, RGB. }
	kCGImageAlphaPremultipliedLast = 1;  { For example, premultiplied RGBA }
	kCGImageAlphaPremultipliedFirst = 2; { For example, premultiplied ARGB }
	kCGImageAlphaLast = 3;               { For example, non-premultiplied RGBA }
	kCGImageAlphaFirst = 4;              { For example, non-premultiplied ARGB }
	kCGImageAlphaNoneSkipLast = 5;       { For example, RBGX. }
	kCGImageAlphaNoneSkipFirst = 6;      { For example, XRGB. }
	kCGImageAlphaOnly = 7;                { No color data, alpha data only }

const
	kCGBitmapAlphaInfoMask = $1F;
	kCGBitmapFloatComponents = 1 shl 8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCGBitmapByteOrderMask = $7000;
	kCGBitmapByteOrderDefault = 0 shl 12;
=======
    
	kCGBitmapByteOrderMask = $7000;
>>>>>>> graemeg/fixes_2_2
=======
    
	kCGBitmapByteOrderMask = $7000;
>>>>>>> origin/fixes_2_2
=======
	kCGBitmapByteOrderMask = $7000;
	kCGBitmapByteOrderDefault = 0 shl 12;
>>>>>>> origin/fixes_2.4
	kCGBitmapByteOrder16Little = 1 shl 12;
	kCGBitmapByteOrder32Little = 2 shl 12;
	kCGBitmapByteOrder16Big = 3 shl 12;
	kCGBitmapByteOrder32Big = 4 shl 12;
type
	CGBitmapInfo = UInt32; { Available in MAC OS X 10.4 & later. }

const
{$ifc TARGET_RT_BIG_ENDIAN}
	kCGBitmapByteOrder16Host = kCGBitmapByteOrder16Big;
	kCGBitmapByteOrder32Host = kCGBitmapByteOrder32Big;
{$elsec}
	kCGBitmapByteOrder16Host = kCGBitmapByteOrder16Little;
	kCGBitmapByteOrder32Host = kCGBitmapByteOrder32Little;
{$endc}

{ Return the CFTypeID for CGImageRefs. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGImageGetTypeID: CFTypeID; external name '_CGImageGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create an image. }

function CGImageCreate( width: size_t; height: size_t; bitsPerComponent: size_t; bitsPerPixel: size_t; bytesPerRow: size_t; space: CGColorSpaceRef; bitmapInfo: CGBitmapInfo; provider: CGDataProviderRef; {const} decode: {variable-size-array} CGFloatPtr; shouldInterpolate: CBool; intent: CGColorRenderingIntent ): CGImageRef; external name '_CGImageCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Create an image mask. }

function CGImageMaskCreate( width: size_t; height: size_t; bitsPerComponent: size_t; bitsPerPixel: size_t; bytesPerRow: size_t; provider: CGDataProviderRef; {const} decode: {variable-size-array} CGFloatPtr; shouldInterpolate: CBool ): CGImageRef; external name '_CGImageMaskCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of `image'. Only the image structure itself is copied; the
   underlying data is not. }

function CGImageCreateCopy( image: CGImageRef ): CGImageRef; external name '_CGImageCreateCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create an image from `source', a data provider of JPEG-encoded data. }

function CGImageCreateWithJPEGDataProvider( source: CGDataProviderRef; {const} decode: {variable-size-array} CGFloatPtr; shouldInterpolate: CBool; intent: CGColorRenderingIntent ): CGImageRef; external name '_CGImageCreateWithJPEGDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_2_0) *)

{ Create an image using `source', a data provider for PNG-encoded data. }

function CGImageCreateWithPNGDataProvider( source: CGDataProviderRef; {const} decode: {variable-size-array} CGFloatPtr; shouldInterpolate: CBool; intent: CGColorRenderingIntent ): CGImageRef; external name '_CGImageCreateWithPNGDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create an image using the data contained within the subrectangle `rect'
   of `image'.

   The new image is created by
     1) adjusting `rect' to integral bounds by calling "CGRectIntegral";
     2) intersecting the result with a rectangle with origin (0, 0) and size
        equal to the size of `image';
     3) referencing the pixels within the resulting rectangle, treating the
        first pixel of the image data as the origin of the image.
   If the resulting rectangle is the null rectangle, this function returns
   NULL.

   If W and H are the width and height of image, respectively, then the
   point (0,0) corresponds to the first pixel of the image data; the point
   (W-1, 0) is the last pixel of the first row of the image data; (0, H-1)
   is the first pixel of the last row of the image data; and (W-1, H-1) is
   the last pixel of the last row of the image data.

   The resulting image retains a reference to the original image, so you may
   release the original image after calling this function. }

function CGImageCreateWithImageInRect( image: CGImageRef; rect: CGRect ): CGImageRef; external name '_CGImageCreateWithImageInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a new image from `image' masked by `mask', which may be an image
   mask or an image.

   If `mask' is an image mask, then it indicates which parts of the context
   are to be painted with the image when drawn in a context, and which are
   to be masked out (left unchanged). The source samples of the image mask
   determine which areas are painted, acting as an "inverse alpha": if the
   value of a source sample in the image mask is S, then the corresponding
   region in `image' is blended with the destination using an alpha of
   (1-S). (For example, if S is 1, then the region is not painted, while if
   S is 0, the region is fully painted.)

   If `mask' is an image, then it serves as alpha mask for blending the
   image onto the destination. The source samples of `mask' determine which
   areas are painted: if the value of the source sample in mask is S, then
   the corresponding region in image is blended with the destination with an
   alpha of S. (For example, if S is 0, then the region is not painted,
   while if S is 1, the region is fully painted.)

   The parameter `image' may not be an image mask and may not have an image
   mask or masking color associated with it.
  
   If `mask' is an image, then it must be in the DeviceGray color space, may
   not have alpha, and may not itself be masked by an image mask or a
   masking color. }

function CGImageCreateWithMask( image: CGImageRef; mask: CGImageRef ): CGImageRef; external name '_CGImageCreateWithMask';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a new image from `image' masked by `components', an array of 2N
   values ( min[1], max[1], ... min[N], max[N] ) where N is the number of
   components in color space of `image'. Any image sample with color value
   (c[1], ... c[N]) where min[i] <= c[i] <= max[i] for 1 <= i <= N is masked
   out (that is, not painted).

   Each value in `components' must be a valid image sample value: if `image'
   has integral pixel components, then each value of must be in the range
   [0..2**bitsPerComponent - 1] (where `bitsPerComponent' is the number of
   bits/component of `image'); if `image' has floating-point pixel
   components, then each value may be any floating-point number which is a
   valid color component.

   The parameter `image' may not be an image mask, and may not already have
   an image mask or masking color associated with it. }

function CGImageCreateWithMaskingColors( image: CGImageRef; {const} components: {variable-size-array} CGFloatPtr ): CGImageRef; external name '_CGImageCreateWithMaskingColors';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a copy of `image', replacing the image's color space with `space'.
   Returns NULL if `image' is an image mask, or if the number of components
   of `space' isn't the same as the number of components of the color space
   of `image'. }

function CGImageCreateCopyWithColorSpace( image: CGImageRef; space: CGColorSpaceRef ): CGImageRef; external name '_CGImageCreateCopyWithColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
function CGImageGetTypeID: CFTypeID; external name '_CGImageGetTypeID'; (* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
=======
function CGImageGetTypeID: CFTypeID; external name '_CGImageGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Create an image. }

function CGImageCreate( width: size_t; height: size_t; bitsPerComponent: size_t; bitsPerPixel: size_t; bytesPerRow: size_t; space: CGColorSpaceRef; bitmapInfo: CGBitmapInfo; provider: CGDataProviderRef; {const} decode: {variable-size-array} CGFloatPtr; shouldInterpolate: CBool; intent: CGColorRenderingIntent ): CGImageRef; external name '_CGImageCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Create an image mask. }

function CGImageMaskCreate( width: size_t; height: size_t; bitsPerComponent: size_t; bitsPerPixel: size_t; bytesPerRow: size_t; provider: CGDataProviderRef; {const} decode: {variable-size-array} CGFloatPtr; shouldInterpolate: CBool ): CGImageRef; external name '_CGImageMaskCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of `image'. Only the image structure itself is copied; the
   underlying data is not. }

function CGImageCreateCopy( image: CGImageRef ): CGImageRef; external name '_CGImageCreateCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create an image from `source', a data provider of JPEG-encoded data. }

function CGImageCreateWithJPEGDataProvider( source: CGDataProviderRef; {const} decode: {variable-size-array} CGFloatPtr; shouldInterpolate: CBool; intent: CGColorRenderingIntent ): CGImageRef; external name '_CGImageCreateWithJPEGDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_2_0) *)

{ Create an image using `source', a data provider for PNG-encoded data. }

function CGImageCreateWithPNGDataProvider( source: CGDataProviderRef; {const} decode: {variable-size-array} CGFloatPtr; shouldInterpolate: CBool; intent: CGColorRenderingIntent ): CGImageRef; external name '_CGImageCreateWithPNGDataProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create an image using the data contained within the subrectangle `rect'
   of `image'.

   The new image is created by
     1) adjusting `rect' to integral bounds by calling "CGRectIntegral";
     2) intersecting the result with a rectangle with origin (0, 0) and size
        equal to the size of `image';
     3) referencing the pixels within the resulting rectangle, treating the
        first pixel of the image data as the origin of the image.
   If the resulting rectangle is the null rectangle, this function returns
   NULL.

   If W and H are the width and height of image, respectively, then the
   point (0,0) corresponds to the first pixel of the image data; the point
   (W-1, 0) is the last pixel of the first row of the image data; (0, H-1)
   is the first pixel of the last row of the image data; and (W-1, H-1) is
   the last pixel of the last row of the image data.

   The resulting image retains a reference to the original image, so you may
   release the original image after calling this function. }

function CGImageCreateWithImageInRect( image: CGImageRef; rect: CGRect ): CGImageRef; external name '_CGImageCreateWithImageInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a new image from `image' masked by `mask', which may be an image
   mask or an image.

   If `mask' is an image mask, then it indicates which parts of the context
   are to be painted with the image when drawn in a context, and which are
   to be masked out (left unchanged). The source samples of the image mask
   determine which areas are painted, acting as an "inverse alpha": if the
   value of a source sample in the image mask is S, then the corresponding
   region in `image' is blended with the destination using an alpha of
   (1-S). (For example, if S is 1, then the region is not painted, while if
   S is 0, the region is fully painted.)

   If `mask' is an image, then it serves as alpha mask for blending the
   image onto the destination. The source samples of `mask' determine which
   areas are painted: if the value of the source sample in mask is S, then
   the corresponding region in image is blended with the destination with an
   alpha of S. (For example, if S is 0, then the region is not painted,
   while if S is 1, the region is fully painted.)

   The parameter `image' may not be an image mask and may not have an image
   mask or masking color associated with it.
  
   If `mask' is an image, then it must be in the DeviceGray color space, may
   not have alpha, and may not itself be masked by an image mask or a
   masking color. }

function CGImageCreateWithMask( image: CGImageRef; mask: CGImageRef ): CGImageRef; external name '_CGImageCreateWithMask';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a new image from `image' masked by `components', an array of 2N
<<<<<<< HEAD
 * values ( min[1], max[1], ... min[N], max[N] ) where N is the number of
 * components in color space of `image'. Any image sample with color value
 * (c[1], ... c[N]) where min[i] <= c[i] <= max[i] for 1 <= i <= N is
 * masked out (that is, not painted).
 *
 * Each value in `components' must be a valid image sample value: if
 * `image' has integral pixel components, then each value of must be in the
 * range [0 .. 2**bitsPerComponent - 1] (where `bitsPerComponent' is the
 * number of bits/component of `image'); if `image' has floating-point
 * pixel components, then each value may be any floating-point number which
 * is a valid color component.
 *
 * The parameter `image' may not be an image mask, and may not already have
 * an image mask or masking color associated with it. }

function CGImageCreateWithMaskingColors( image: CGImageRef; {const} components: {variable-size-array} Float32Ptr ): CGImageRef; external name '_CGImageCreateWithMaskingColors'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Create a copy of `image', replacing the image's colorspace with
 * `colorspace'.  Returns NULL if `image' is an image mask, or if the
 * number of components of `colorspace' isn't the same as the number of
 * components of the colorspace of `image'. }

function CGImageCreateCopyWithColorSpace( image: CGImageRef; colorspace: CGColorSpaceRef ): CGImageRef; external name '_CGImageCreateCopyWithColorSpace'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
   values ( min[1], max[1], ... min[N], max[N] ) where N is the number of
   components in color space of `image'. Any image sample with color value
   (c[1], ... c[N]) where min[i] <= c[i] <= max[i] for 1 <= i <= N is masked
   out (that is, not painted).

   Each value in `components' must be a valid image sample value: if `image'
   has integral pixel components, then each value of must be in the range
   [0..2**bitsPerComponent - 1] (where `bitsPerComponent' is the number of
   bits/component of `image'); if `image' has floating-point pixel
   components, then each value may be any floating-point number which is a
   valid color component.

   The parameter `image' may not be an image mask, and may not already have
   an image mask or masking color associated with it. }

function CGImageCreateWithMaskingColors( image: CGImageRef; {const} components: {variable-size-array} CGFloatPtr ): CGImageRef; external name '_CGImageCreateWithMaskingColors';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a copy of `image', replacing the image's color space with `space'.
   Returns NULL if `image' is an image mask, or if the number of components
   of `space' isn't the same as the number of components of the color space
   of `image'. }

function CGImageCreateCopyWithColorSpace( image: CGImageRef; space: CGColorSpaceRef ): CGImageRef; external name '_CGImageCreateCopyWithColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Equivalent to `CFRetain(image)'. }

function CGImageRetain( image: CGImageRef ): CGImageRef; external name '_CGImageRetain';
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

{ Equivalent to `CFRelease(image)'. }

procedure CGImageRelease( image: CGImageRef ); external name '_CGImageRelease';
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

{ Return true if `image' is an image mask, false otherwise. }

function CGImageIsMask( image: CGImageRef ): CBool; external name '_CGImageIsMask';
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

{ Return the width of `image'. }

function CGImageGetWidth( image: CGImageRef ): size_t; external name '_CGImageGetWidth';
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

{ Return the height of `image'. }

function CGImageGetHeight( image: CGImageRef ): size_t; external name '_CGImageGetHeight';
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

{ Return the number of bits/component of `image'. }

function CGImageGetBitsPerComponent( image: CGImageRef ): size_t; external name '_CGImageGetBitsPerComponent';
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

{ Return the number of bits/pixel of `image'. }

function CGImageGetBitsPerPixel( image: CGImageRef ): size_t; external name '_CGImageGetBitsPerPixel';
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

{ Return the number of bytes/row of `image'. }

function CGImageGetBytesPerRow( image: CGImageRef ): size_t; external name '_CGImageGetBytesPerRow';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the color space of `image', or NULL if `image' is an image
   mask. }

function CGImageGetColorSpace( image: CGImageRef ): CGColorSpaceRef; external name '_CGImageGetColorSpace';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the color space of `image', or NULL if `image' is an image
   mask. }

function CGImageGetColorSpace( image: CGImageRef ): CGColorSpaceRef; external name '_CGImageGetColorSpace';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the alpha info of `image'. }

function CGImageGetAlphaInfo( image: CGImageRef ): CGImageAlphaInfo; external name '_CGImageGetAlphaInfo';
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

{ Return the data provider of `image'. }

function CGImageGetDataProvider( image: CGImageRef ): CGDataProviderRef; external name '_CGImageGetDataProvider';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the decode array of `image'. }

function CGImageGetDecode( image: CGImageRef ): CGFloatPtr; external name '_CGImageGetDecode';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2

{ Return the decode array of `image'. }

function CGImageGetDecode( image: CGImageRef ): Float32Ptr; external name '_CGImageGetDecode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the decode array of `image'. }

function CGImageGetDecode( image: CGImageRef ): CGFloatPtr; external name '_CGImageGetDecode';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the interpolation parameter of `image'. }

function CGImageGetShouldInterpolate( image: CGImageRef ): CBool; external name '_CGImageGetShouldInterpolate';
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

{ Return the rendering intent of `image'. }

function CGImageGetRenderingIntent( image: CGImageRef ): CGColorRenderingIntent; external name '_CGImageGetRenderingIntent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bitmap info of `image'. }

function CGImageGetBitmapInfo( image: CGImageRef ): CGBitmapInfo; external name '_CGImageGetBitmapInfo';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the bitmap info of `image'. }

function CGImageGetBitmapInfo( image: CGImageRef ): CGBitmapInfo; external name '_CGImageGetBitmapInfo';
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
