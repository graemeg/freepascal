{
 *  CVPixelBuffer.h
 *  CoreVideo
 *
 *  Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
 *
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
{  Pascal Translation Update:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, August 2015 }
=======
 {	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
 {	 Pascal Translation Update:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }

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
 {	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
 {	 Pascal Translation Update:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }

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

unit CVPixelBuffer;
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
>>>>>>> origin/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
uses MacTypes, CFArray, CFBase, CFDictionary, CVBase, CVImageBuffer, CVReturns;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

 
  {! @header CVPixelBuffer.h
	@copyright 2004 Apple Computer, Inc. All rights reserved.
	@availability Mac OS X 10.4 or later
    @discussion CVPixelBuffers are CVImageBuffers that hold the pixels in main memory
		   
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
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
{
>>>>>>> origin/fixes_2.4
#if COREVIDEO_SUPPORTS_IOSURFACE
#endif // COREVIDEO_SUPPORTS_IOSURFACE
}

{ Right now we don't support IOSurface }
{$setc COREVIDEO_SUPPORTS_IOSURFACE := FALSE}

{
<<<<<<< HEAD
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
>>>>>>> origin/fixes_2.4
CoreVideo pixel format type constants.
CoreVideo does not provide support for all of these formats; this list just defines their names.
}
const
	kCVPixelFormatType_1Monochrome = $00000001; { 1 bit indexed }
	kCVPixelFormatType_2Indexed = $00000002; { 2 bit indexed }
	kCVPixelFormatType_4Indexed = $00000004; { 4 bit indexed }
	kCVPixelFormatType_8Indexed = $00000008; { 8 bit indexed }
	kCVPixelFormatType_1IndexedGray_WhiteIsZero = $00000021; { 1 bit indexed gray, white is zero }
	kCVPixelFormatType_2IndexedGray_WhiteIsZero = $00000022; { 2 bit indexed gray, white is zero }
	kCVPixelFormatType_4IndexedGray_WhiteIsZero = $00000024; { 4 bit indexed gray, white is zero }
	kCVPixelFormatType_8IndexedGray_WhiteIsZero = $00000028; { 8 bit indexed gray, white is zero }
	kCVPixelFormatType_16BE555 = $00000010; { 16 bit BE RGB 555 }
	kCVPixelFormatType_16LE555 = FourCharCode('L555');     { 16 bit LE RGB 555 }
	kCVPixelFormatType_16LE5551 = FourCharCode('5551');     { 16 bit LE RGB 5551 }
	kCVPixelFormatType_16BE565 = FourCharCode('B565');     { 16 bit BE RGB 565 }
	kCVPixelFormatType_16LE565 = FourCharCode('L565');     { 16 bit LE RGB 565 }
	kCVPixelFormatType_24RGB = $00000018; { 24 bit RGB }
	kCVPixelFormatType_24BGR = FourCharCode('24BG');     { 24 bit BGR }
	kCVPixelFormatType_32ARGB = $00000020; { 32 bit ARGB }
	kCVPixelFormatType_32BGRA = FourCharCode('BGRA');     { 32 bit BGRA }
	kCVPixelFormatType_32ABGR = FourCharCode('ABGR');     { 32 bit ABGR }
	kCVPixelFormatType_32RGBA = FourCharCode('RGBA');     { 32 bit RGBA }
	kCVPixelFormatType_64ARGB = FourCharCode('b64a');     { 64 bit ARGB, 16-bit big-endian samples }
	kCVPixelFormatType_48RGB = FourCharCode('b48r');     { 48 bit RGB, 16-bit big-endian samples }
	kCVPixelFormatType_32AlphaGray = FourCharCode('b32a');     { 32 bit AlphaGray, 16-bit big-endian samples, black is zero }
	kCVPixelFormatType_16Gray = FourCharCode('b16g');     { 16 bit Grayscale, 16-bit big-endian samples, black is zero }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCVPixelFormatType_30RGB = FourCharCode('R10k');     { 30 bit RGB, 10-bit big-endian samples, 2 unused padding bits (at least significant end). }
	kCVPixelFormatType_422YpCbCr8 = FourCharCode('2vuy');     { Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1 }
	kCVPixelFormatType_4444YpCbCrA8 = FourCharCode('v408');     { Component Y'CbCrA 8-bit 4:4:4:4, ordered Cb Y' Cr A }
	kCVPixelFormatType_4444YpCbCrA8R = FourCharCode('r408');     { Component Y'CbCrA 8-bit 4:4:4:4, rendering format. full range alpha, zero biased YUV, ordered A Y' Cb Cr }
	kCVPixelFormatType_4444AYpCbCr8 = FourCharCode('y408');     { Component Y'CbCrA 8-bit 4:4:4:4, ordered A Y' Cb Cr, full range alpha, video range Y'CbCr. }
	kCVPixelFormatType_4444AYpCbCr16 = FourCharCode('y416');     { Component Y'CbCrA 16-bit 4:4:4:4, ordered A Y' Cb Cr, full range alpha, video range Y'CbCr, 16-bit little-endian samples. }
=======
	kCVPixelFormatType_422YpCbCr8 = FourCharCode('2vuy');     { Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1 }
	kCVPixelFormatType_4444YpCbCrA8 = FourCharCode('v408');     { Component Y'CbCrA 8-bit 4:4:4:4, ordered Cb Y' Cr A }
	kCVPixelFormatType_4444YpCbCrA8R = FourCharCode('r408');     { Component Y'CbCrA 8-bit 4:4:4:4, rendering format. full range alpha, zero biased YUV, ordered A Y' Cb Cr }
>>>>>>> graemeg/cpstrnew
=======
	kCVPixelFormatType_422YpCbCr8 = FourCharCode('2vuy');     { Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1 }
	kCVPixelFormatType_4444YpCbCrA8 = FourCharCode('v408');     { Component Y'CbCrA 8-bit 4:4:4:4, ordered Cb Y' Cr A }
	kCVPixelFormatType_4444YpCbCrA8R = FourCharCode('r408');     { Component Y'CbCrA 8-bit 4:4:4:4, rendering format. full range alpha, zero biased YUV, ordered A Y' Cb Cr }
>>>>>>> graemeg/cpstrnew
=======
	kCVPixelFormatType_422YpCbCr8 = FourCharCode('2vuy');     { Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1 }
	kCVPixelFormatType_4444YpCbCrA8 = FourCharCode('v408');     { Component Y'CbCrA 8-bit 4:4:4:4, ordered Cb Y' Cr A }
	kCVPixelFormatType_4444YpCbCrA8R = FourCharCode('r408');     { Component Y'CbCrA 8-bit 4:4:4:4, rendering format. full range alpha, zero biased YUV, ordered A Y' Cb Cr }
>>>>>>> graemeg/cpstrnew
=======
	kCVPixelFormatType_422YpCbCr8 = FourCharCode('2vuy');     { Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1 }
	kCVPixelFormatType_4444YpCbCrA8 = FourCharCode('v408');     { Component Y'CbCrA 8-bit 4:4:4:4, ordered Cb Y' Cr A }
	kCVPixelFormatType_4444YpCbCrA8R = FourCharCode('r408');     { Component Y'CbCrA 8-bit 4:4:4:4, rendering format. full range alpha, zero biased YUV, ordered A Y' Cb Cr }
>>>>>>> origin/cpstrnew
=======
	kCVPixelFormatType_422YpCbCr8 = FourCharCode('2vuy');     { Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1 }
	kCVPixelFormatType_4444YpCbCrA8 = FourCharCode('v408');     { Component Y'CbCrA 8-bit 4:4:4:4, ordered Cb Y' Cr A }
	kCVPixelFormatType_4444YpCbCrA8R = FourCharCode('r408');     { Component Y'CbCrA 8-bit 4:4:4:4, rendering format. full range alpha, zero biased YUV, ordered A Y' Cb Cr }
>>>>>>> origin/fixes_2.4
	kCVPixelFormatType_444YpCbCr8 = FourCharCode('v308');     { Component Y'CbCr 8-bit 4:4:4 }
	kCVPixelFormatType_422YpCbCr16 = FourCharCode('v216');     { Component Y'CbCr 10,12,14,16-bit 4:2:2 }
	kCVPixelFormatType_422YpCbCr10 = FourCharCode('v210');     { Component Y'CbCr 10-bit 4:2:2 }
	kCVPixelFormatType_444YpCbCr10 = FourCharCode('v410');     { Component Y'CbCr 10-bit 4:4:4 }
	kCVPixelFormatType_420YpCbCr8Planar = FourCharCode('y420');   { Planar Component Y'CbCr 8-bit 4:2:0.  baseAddr points to a big-endian CVPlanarPixelBufferInfo_YCbCrPlanar struct }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCVPixelFormatType_420YpCbCr8PlanarFullRange = FourCharCode('f420');   { Planar Component Y'CbCr 8-bit 4:2:0, full range.  baseAddr points to a big-endian CVPlanarPixelBufferInfo_YCbCrPlanar struct }
	kCVPixelFormatType_422YpCbCr_4A_8BiPlanar = FourCharCode('a2vy'); { First plane: Video-range Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1; second plane: alpha 8-bit 0-255 }
	kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange = FourCharCode('420v'); { Bi-Planar Component Y'CbCr 8-bit 4:2:0, video-range (luma=[16,235] chroma=[16,240]).  baseAddr points to a big-endian CVPlanarPixelBufferInfo_YCbCrBiPlanar struct }
	kCVPixelFormatType_420YpCbCr8BiPlanarFullRange = FourCharCode('420f'); { Bi-Planar Component Y'CbCr 8-bit 4:2:0, full-range (luma=[0,255] chroma=[1,255]).  baseAddr points to a big-endian CVPlanarPixelBufferInfo_YCbCrBiPlanar struct } 
	kCVPixelFormatType_422YpCbCr8_yuvs = FourCharCode('yuvs');     { Component Y'CbCr 8-bit 4:2:2, ordered Y'0 Cb Y'1 Cr }
	kCVPixelFormatType_422YpCbCr8FullRange = FourCharCode('yuvf'); { Component Y'CbCr 8-bit 4:2:2, full range, ordered Y'0 Cb Y'1 Cr }
	kCVPixelFormatType_OneComponent8 = FourCharCode('L008');     { 8 bit one component, black is zero }
	kCVPixelFormatType_TwoComponent8 = FourCharCode('2C08');     { 8 bit two component, black is zero }
	kCVPixelFormatType_OneComponent16Half = FourCharCode('L00h');     { 16 bit one component IEEE half-precision float, 16-bit little-endian samples }
	kCVPixelFormatType_OneComponent32Float = FourCharCode('L00f');     { 32 bit one component IEEE float, 32-bit little-endian samples }
	kCVPixelFormatType_TwoComponent16Half = FourCharCode('2C0h');     { 16 bit two component IEEE half-precision float, 16-bit little-endian samples }
	kCVPixelFormatType_TwoComponent32Float = FourCharCode('2C0f');     { 32 bit two component IEEE float, 32-bit little-endian samples }
	kCVPixelFormatType_64RGBAHalf = FourCharCode('RGhA');     { 64 bit RGBA IEEE half-precision float, 16-bit little-endian samples }
	kCVPixelFormatType_128RGBAFloat = FourCharCode('RGfA');     { 128 bit RGBA IEEE float, 32-bit little-endian samples }
=======
	kCVPixelFormatType_422YpCbCr_4A_8BiPlanar = FourCharCode('a2vy'); { First plane: Video-range Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1; second plane: alpha 8-bit 0-255 }
>>>>>>> graemeg/cpstrnew
=======
	kCVPixelFormatType_422YpCbCr_4A_8BiPlanar = FourCharCode('a2vy'); { First plane: Video-range Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1; second plane: alpha 8-bit 0-255 }
>>>>>>> graemeg/cpstrnew
=======
	kCVPixelFormatType_422YpCbCr_4A_8BiPlanar = FourCharCode('a2vy'); { First plane: Video-range Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1; second plane: alpha 8-bit 0-255 }
>>>>>>> graemeg/cpstrnew
=======
	kCVPixelFormatType_422YpCbCr_4A_8BiPlanar = FourCharCode('a2vy'); { First plane: Video-range Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1; second plane: alpha 8-bit 0-255 }
>>>>>>> origin/cpstrnew
=======
	kCVPixelFormatType_422YpCbCr_4A_8BiPlanar = FourCharCode('a2vy'); { First plane: Video-range Component Y'CbCr 8-bit 4:2:2, ordered Cb Y'0 Cr Y'1; second plane: alpha 8-bit 0-255 }
>>>>>>> origin/fixes_2.4

	
{!
	@enum Pixel Buffer Locking Flags
	@discussion Flags to pass to CVPixelBufferLockBaseAddress() / CVPixelBufferUnlockBaseAddress()
	@constant kCVPixelBufferLock_ReadOnly
		If you are not going to modify the data while you hold the lock, you should set this flag
		to avoid potentially invalidating any existing caches of the buffer contents.  This flag
		should be passed both to the lock and unlock functions.  Non-symmetrical usage of this
		flag will result in undefined behavior.
}
type
	CVPixelBufferLockFlags = SInt32;
const
	kCVPixelBufferLock_ReadOnly = $00000001;

{
Planar pixel buffers have the following descriptor at their base address.  
Clients should generally use CVPixelBufferGetBaseAddressOfPlane, 
CVPixelBufferGetBytesPerRowOfPlane, etc. instead of accessing it directly.
}
type
	CVPlanarComponentInfoPtr = ^CVPlanarComponentInfo;
	CVPlanarComponentInfo = record
		offset: SInt32;    { offset from main base address to base address of this plane, big-endian }
		rowBytes: UInt32;  { bytes per row of this plane, big-endian }
	end;
type
	CVPlanarPixelBufferInfoPtr = ^CVPlanarPixelBufferInfo;
	CVPlanarPixelBufferInfo = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		componentInfo: array [0..1-1] of CVPlanarComponentInfo;
=======
	  componentInfo	: array[0..0] of CVPlanarComponentInfo;
>>>>>>> graemeg/cpstrnew
=======
	  componentInfo	: array[0..0] of CVPlanarComponentInfo;
>>>>>>> graemeg/cpstrnew
=======
	  componentInfo	: array[0..0] of CVPlanarComponentInfo;
>>>>>>> graemeg/cpstrnew
=======
	  componentInfo	: array[0..0] of CVPlanarComponentInfo;
>>>>>>> origin/cpstrnew
=======
	  componentInfo	: array[0..0] of CVPlanarComponentInfo;
>>>>>>> origin/fixes_2.4
	end;
type
	CVPlanarPixelBufferInfo_YCbCrPlanarPtr = ^CVPlanarPixelBufferInfo_YCbCrPlanar;
	CVPlanarPixelBufferInfo_YCbCrPlanar = record
		componentInfoY: CVPlanarComponentInfo;
		componentInfoCb: CVPlanarComponentInfo;
		componentInfoCr: CVPlanarComponentInfo;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
    CVPlanarPixelBufferInfo_YCbCrBiPlanarPtr = ^CVPlanarPixelBufferInfo_YCbCrBiPlanar;
	CVPlanarPixelBufferInfo_YCbCrBiPlanar = record
		componentInfoY: CVPlanarComponentInfo;
		componentInfoCbCr: CVPlanarComponentInfo;
	end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

//#pragma mark BufferAttributeKeys
var kCVPixelBufferPixelFormatTypeKey: CFStringRef; external name '_kCVPixelBufferPixelFormatTypeKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)		    // A single CFNumber or a CFArray of CFNumbers (OSTypes)
var kCVPixelBufferMemoryAllocatorKey: CFStringRef; external name '_kCVPixelBufferMemoryAllocatorKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)		    // CFAllocatorRef
var kCVPixelBufferWidthKey: CFStringRef; external name '_kCVPixelBufferWidthKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)			    // CFNumber
var kCVPixelBufferHeightKey: CFStringRef; external name '_kCVPixelBufferHeightKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)			    // CFNumber
var kCVPixelBufferExtendedPixelsLeftKey: CFStringRef; external name '_kCVPixelBufferExtendedPixelsLeftKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)	    // CFNumber
var kCVPixelBufferExtendedPixelsTopKey: CFStringRef; external name '_kCVPixelBufferExtendedPixelsTopKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)		    // CFNumber
var kCVPixelBufferExtendedPixelsRightKey: CFStringRef; external name '_kCVPixelBufferExtendedPixelsRightKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)	    // CFNumber
var kCVPixelBufferExtendedPixelsBottomKey: CFStringRef; external name '_kCVPixelBufferExtendedPixelsBottomKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)	    // CFNumber
var kCVPixelBufferBytesPerRowAlignmentKey: CFStringRef; external name '_kCVPixelBufferBytesPerRowAlignmentKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)	    // CFNumber
var kCVPixelBufferCGBitmapContextCompatibilityKey: CFStringRef; external name '_kCVPixelBufferCGBitmapContextCompatibilityKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)  // CFBoolean
var kCVPixelBufferCGImageCompatibilityKey: CFStringRef; external name '_kCVPixelBufferCGImageCompatibilityKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)	    // CFBoolean
var kCVPixelBufferOpenGLCompatibilityKey: CFStringRef; external name '_kCVPixelBufferOpenGLCompatibilityKey'; (* attribute const *)
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)	    // CFBoolean
var kCVPixelBufferPlaneAlignmentKey: CFStringRef; external name '_kCVPixelBufferPlaneAlignmentKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_4_0) *)		    // CFNumber
var kCVPixelBufferIOSurfacePropertiesKey: CFStringRef; external name '_kCVPixelBufferIOSurfacePropertiesKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_4_0) *)     // CFDictionary; presence requests buffer allocation via IOSurface
{$ifc TARGET_OS_IPHONE}
var kCVPixelBufferOpenGLESCompatibilityKey: CFStringRef; external name '_kCVPixelBufferOpenGLESCompatibilityKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_NA,__IPHONE_6_0) *)	    // CFBoolean
{$endc}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	    // CFBoolean
var kCVPixelBufferPlaneAlignmentKey: CFStringRef; external name '_kCVPixelBufferPlaneAlignmentKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)		    // CFNumber
var kCVPixelBufferIOSurfacePropertiesKey: CFStringRef; external name '_kCVPixelBufferIOSurfacePropertiesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)     // CFDictionary; presence requests buffer allocation via IOSurface
// Ensures that CGLTexImageIOSurface2D() will succeed in creating a valid texture object from the CVPixelBuffer's IOSurface.
var kCVPixelBufferIOSurfaceOpenGLTextureCompatibilityKey: CFStringRef; external name '_kCVPixelBufferIOSurfaceOpenGLTextureCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)	// CFBoolean
// Ensures that CGLTexImageIOSurface2D() will succeed in creating a valid texture object from the CVPixelBuffer's IOSurface AND that the resulting texture may be used as a color buffer attachment to a OpenGL frame buffer object.
var kCVPixelBufferIOSurfaceOpenGLFBOCompatibilityKey: CFStringRef; external name '_kCVPixelBufferIOSurfaceOpenGLFBOCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)	// CFBoolean
// Ensures that the CVPixelBuffer's IOSurfaceRef can be displayed in an CoreAnimation CALayer.
var kCVPixelBufferIOSurfaceCoreAnimationCompatibilityKey: CFStringRef; external name '_kCVPixelBufferIOSurfaceCoreAnimationCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)	// CFBoolean

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
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

//#pragma mark BufferAttributeKeys
var kCVPixelBufferPixelFormatTypeKey: CFStringRef; external name '_kCVPixelBufferPixelFormatTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		    // A single CFNumber or a CFArray of CFNumbers (OSTypes)
var kCVPixelBufferMemoryAllocatorKey: CFStringRef; external name '_kCVPixelBufferMemoryAllocatorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		    // CFAllocatorRef
var kCVPixelBufferWidthKey: CFStringRef; external name '_kCVPixelBufferWidthKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)			    // CFNumber
var kCVPixelBufferHeightKey: CFStringRef; external name '_kCVPixelBufferHeightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)			    // CFNumber
var kCVPixelBufferExtendedPixelsLeftKey: CFStringRef; external name '_kCVPixelBufferExtendedPixelsLeftKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	    // CFNumber
var kCVPixelBufferExtendedPixelsTopKey: CFStringRef; external name '_kCVPixelBufferExtendedPixelsTopKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		    // CFNumber
var kCVPixelBufferExtendedPixelsRightKey: CFStringRef; external name '_kCVPixelBufferExtendedPixelsRightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	    // CFNumber
var kCVPixelBufferExtendedPixelsBottomKey: CFStringRef; external name '_kCVPixelBufferExtendedPixelsBottomKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	    // CFNumber
var kCVPixelBufferBytesPerRowAlignmentKey: CFStringRef; external name '_kCVPixelBufferBytesPerRowAlignmentKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	    // CFNumber
var kCVPixelBufferCGBitmapContextCompatibilityKey: CFStringRef; external name '_kCVPixelBufferCGBitmapContextCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)  // CFBoolean
var kCVPixelBufferCGImageCompatibilityKey: CFStringRef; external name '_kCVPixelBufferCGImageCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	    // CFBoolean
var kCVPixelBufferOpenGLCompatibilityKey: CFStringRef; external name '_kCVPixelBufferOpenGLCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	    // CFBoolean
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
var kCVPixelBufferPlaneAlignmentKey: CFStringRef; external name '_kCVPixelBufferPlaneAlignmentKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)		    // CFNumber
var kCVPixelBufferIOSurfacePropertiesKey: CFStringRef; external name '_kCVPixelBufferIOSurfacePropertiesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)     // CFDictionary; presence requests buffer allocation via IOSurface
// Ensures that CGLTexImageIOSurface2D() will succeed in creating a valid texture object from the CVPixelBuffer's IOSurface.
var kCVPixelBufferIOSurfaceOpenGLTextureCompatibilityKey: CFStringRef; external name '_kCVPixelBufferIOSurfaceOpenGLTextureCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)	// CFBoolean
// Ensures that CGLTexImageIOSurface2D() will succeed in creating a valid texture object from the CVPixelBuffer's IOSurface AND that the resulting texture may be used as a color buffer attachment to a OpenGL frame buffer object.
var kCVPixelBufferIOSurfaceOpenGLFBOCompatibilityKey: CFStringRef; external name '_kCVPixelBufferIOSurfaceOpenGLFBOCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)	// CFBoolean
// Ensures that the CVPixelBuffer's IOSurfaceRef can be displayed in an CoreAnimation CALayer.
var kCVPixelBufferIOSurfaceCoreAnimationCompatibilityKey: CFStringRef; external name '_kCVPixelBufferIOSurfaceCoreAnimationCompatibilityKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)	// CFBoolean

>>>>>>> origin/fixes_2.4

{!
    @typedef	CVPixelBufferRef
    @abstract   Based on the image buffer type. The pixel buffer implements the memory storage for an image buffer.

}
type
	CVPixelBufferRef = CVImageBufferRef;

function CVPixelBufferGetTypeID: CFTypeID; external name '_CVPixelBufferGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferRetain
    @abstract   Retains a CVPixelBuffer object
    @discussion Equivalent to CFRetain, but NULL safe
    @param      buffer A CVPixelBuffer object that you want to retain.
    @result     A CVPixelBuffer object that is the same as the passed in buffer.
}
function CVPixelBufferRetain( texture: CVPixelBufferRef ): CVPixelBufferRef; external name '_CVPixelBufferRetain';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferRelease
    @abstract   Releases a CVPixelBuffer object
    @discussion Equivalent to CFRelease, but NULL safe
    @param      buffer A CVPixelBuffer object that you want to release.
}
procedure CVPixelBufferRelease( texture: CVPixelBufferRef ); external name '_CVPixelBufferRelease';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferCreateResolvedAttributesDictionary
    @abstract   Takes a CFArray of CFDictionary objects describing various pixel buffer attributes and tries to resolve them into a
                single dictionary.
    @discussion This is useful when you need to resolve multiple requirements between different potential clients of a buffer.
    @param      attributes CFArray of CFDictionaries containing kCVPixelBuffer key/value pairs.
    @param      resolvedDictionaryOut The resulting dictionary will be placed here.
    @result     Return value that may be useful in discovering why resolution failed.
}
function CVPixelBufferCreateResolvedAttributesDictionary( allocator: CFAllocatorRef; attributes: CFArrayRef; var resolvedDictionaryOut: CFDictionaryRef ): CVReturn; external name '_CVPixelBufferCreateResolvedAttributesDictionary';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2


{!
    @function   CVPixelBufferCreate
    @abstract   Call to create a single PixelBuffer for a given size and pixelFormatType.
    @discussion Creates a single PixelBuffer for a given size and pixelFormatType. It allocates the necessary memory based on the pixel dimensions, pixelFormatType and extended pixels described in the pixelBufferAttributes. Not all parameters of the pixelBufferAttributes will be used here.
    @param      width   Width of the PixelBuffer in pixels.
    @param      height  Height of the PixelBuffer in pixels.
    @param	pixelFormatType		Pixel format indentified by its respective OSType.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @param	pixelBufferAttributes      A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
=======
    @param	pixelBufferAttributes      A dictionary with additonal attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> graemeg/fixes_2_2
=======
    @param	pixelBufferAttributes      A dictionary with additonal attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> origin/fixes_2_2
=======
    @param	pixelBufferAttributes      A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> origin/fixes_2.4
    @param      pixelBufferOut          The new pixel buffer will be returned here
    @result	returns kCVReturnSuccess on success.
}    
function CVPixelBufferCreate( allocator: CFAllocatorRef; width: size_t; height: size_t; pixelFormatType: OSType; pixelBufferAttributes: CFDictionaryRef; var pixelBufferOut: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferCreate';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

type
	CVPixelBufferReleaseBytesCallback = procedure( releaseRefCon: UnivPtr; baseAddress: {const} UnivPtr );

{!
    @function   CVPixelBufferCreateWithBytes
    @abstract   Call to create a single PixelBuffer for a given size and pixelFormatType based on a passed in piece of memory.
    @discussion Creates a single PixelBuffer for a given size and pixelFormatType. Not all parameters of the pixelBufferAttributes will be used here. It requires a release callback function that will be called, when the PixelBuffer gets destroyed so that the owner of the pixels can free the memory.
    @param      width   Width of the PixelBuffer in pixels
    @param      height  Height of the PixelBuffer in pixels
    @param      pixelFormatType		Pixel format indentified by its respective OSType.
    @param      baseAddress		Address of the memory storing the pixels.
    @param      bytesPerRow		Row bytes of the pixel storage memory.
    @param      releaseCallback         CVPixelBufferReleaseBytePointerCallback function that gets called when the PixelBuffer gets destroyed.
    @param      releaseRefCon           User data identifying the PixelBuffer for the release callback.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @param      pixelBufferAttributes      A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
=======
    @param      pixelBufferAttributes      A dictionary with additonal attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> graemeg/fixes_2_2
=======
    @param      pixelBufferAttributes      A dictionary with additonal attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> origin/fixes_2_2
=======
    @param      pixelBufferAttributes      A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> origin/fixes_2.4
    @param      pixelBufferOut          The new pixel buffer will be returned here
    @result	returns kCVReturnSuccess on success.
}
function CVPixelBufferCreateWithBytes( allocator: CFAllocatorRef; width: size_t; height: size_t; pixelFormatType: OSType; baseAddress: UnivPtr; bytesPerRow: size_t; releaseCallback: CVPixelBufferReleaseBytesCallback; releaseRefCon: UnivPtr; pixelBufferAttributes: CFDictionaryRef; var pixelBufferOut: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferCreateWithBytes';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

type
	CVPixelBufferReleasePlanarBytesCallback = procedure( releaseRefCon: UnivPtr; dataPtr: {const} UnivPtr; dataSize: size_t; numberOfPlanes: size_t; {const} planeAddresses: {variable-size-array} UnivPtr );

{!
    @function   CVPixelBufferCreateWithPlanarBytes
    @abstract   Call to create a single PixelBuffer in planar format for a given size and pixelFormatType based on a passed in piece of memory.
    @discussion Creates a single PixelBuffer for a given size and pixelFormatType. Not all parameters of the pixelBufferAttributes will be used here. It requires a release callback function that will be called, when the PixelBuffer gets destroyed so that the owner of the pixels can free the memory.
    @param      width			Width of the PixelBuffer in pixels
    @param      height			Height of the PixelBuffer in pixels
    @param      pixelFormatType		Pixel format indentified by its respective OSType.
    @param	dataPtr			Pass a pointer to a plane descriptor block, or NULL.
    @param	dataSize		pass size if planes are contiguous, NULL if not.
    @param	numberOfPlanes		Number of planes.
    @param	planeBaseAddress	Array of base addresses for the planes.
    @param	planeWidth		Array of plane widths.
    @param	planeHeight		Array of plane heights.
    @param	planeBytesPerRow	Array of plane bytesPerRow values.
    @param	releaseCallback		CVPixelBufferReleaseBytePointerCallback function that gets called when the PixelBuffer gets destroyed.
    @param	releaseRefCon		User data identifying the PixelBuffer for the release callback.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @param	pixelBufferAttributes      A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
=======
    @param	pixelBufferAttributes      A dictionary with additonal attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> graemeg/fixes_2_2
=======
    @param	pixelBufferAttributes      A dictionary with additonal attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> origin/fixes_2_2
=======
    @param	pixelBufferAttributes      A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
>>>>>>> origin/fixes_2.4
    @param      pixelBufferOut          The new pixel buffer will be returned here
    @result	returns kCVReturnSuccess on success.
}
function CVPixelBufferCreateWithPlanarBytes( allocator: CFAllocatorRef; width: size_t; height: size_t; pixelFormatType: OSType; dataPtr: {pass a pointer to a plane descriptor block, or NULL} UnivPtr; dataSize: {pass size if planes are contiguous, NULL if not} size_t; numberOfPlanes: size_t; planeAddresses: {variable-size-array} UnivPtr; planeWidth: {variable-size-array} size_t_Ptr; planeHeight: {variable-size-array} size_t_Ptr; planeBytesPerRow: {variable-size-array} size_t_Ptr; releaseCallback: CVPixelBufferReleasePlanarBytesCallback; releaseRefCon: UnivPtr; pixelBufferAttributes: CFDictionaryRef; var pixelBufferOut: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferCreateWithPlanarBytes';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)


{!
	@function   CVPixelBufferLockBaseAddress
	@abstract   Description Locks the BaseAddress of the PixelBuffer to ensure that the memory is accessible.
	@param      pixelBuffer Target PixelBuffer.
	@param      lockFlags See CVPixelBufferLockFlags.
	@result     kCVReturnSuccess if the lock succeeded, or error code on failure
}
function CVPixelBufferLockBaseAddress( pixelBuffer: CVPixelBufferRef; lockFlags: CVOptionFlags ): CVReturn; external name '_CVPixelBufferLockBaseAddress';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)

{!
	@function   CVPixelBufferUnlockBaseAddress
	@abstract   Description Unlocks the BaseAddress of the PixelBuffer.
	@param      pixelBuffer Target PixelBuffer.
	@param      unlockFlags See CVPixelBufferLockFlags.
	@result     kCVReturnSuccess if the unlock succeeded, or error code on failure
}
function CVPixelBufferUnlockBaseAddress( pixelBuffer: CVPixelBufferRef; unlockFlags: CVOptionFlags ): CVReturn; external name '_CVPixelBufferUnlockBaseAddress';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{!
	@function   CVPixelBufferLockBaseAddress
	@abstract   Description Locks the BaseAddress of the PixelBuffer to ensure that the memory is accessible.
	@param      pixelBuffer Target PixelBuffer.
	@param      lockFlags See CVPixelBufferLockFlags.
	@result     kCVReturnSuccess if the lock succeeded, or error code on failure
}
function CVPixelBufferLockBaseAddress( pixelBuffer: CVPixelBufferRef; lockFlags: CVOptionFlags ): CVReturn; external name '_CVPixelBufferLockBaseAddress';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
	@function   CVPixelBufferUnlockBaseAddress
	@abstract   Description Unlocks the BaseAddress of the PixelBuffer.
	@param      pixelBuffer Target PixelBuffer.
	@param      unlockFlags See CVPixelBufferLockFlags.
	@result     kCVReturnSuccess if the unlock succeeded, or error code on failure
}
function CVPixelBufferUnlockBaseAddress( pixelBuffer: CVPixelBufferRef; unlockFlags: CVOptionFlags ): CVReturn; external name '_CVPixelBufferUnlockBaseAddress';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetWidth
    @abstract   Returns the width of the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @result     Width in pixels.
}
function CVPixelBufferGetWidth( pixelBuffer: CVPixelBufferRef ): size_t; external name '_CVPixelBufferGetWidth';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetHeight
    @abstract   Returns the height of the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @result     Height in pixels.
}
function CVPixelBufferGetHeight( pixelBuffer: CVPixelBufferRef ): size_t; external name '_CVPixelBufferGetHeight';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetPixelFormatType
    @abstract   Returns the PixelFormatType of the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @result     OSType identifying the pixel format by its type.
}
function CVPixelBufferGetPixelFormatType( pixelBuffer: CVPixelBufferRef ): OSType; external name '_CVPixelBufferGetPixelFormatType';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetBaseAddress
    @abstract   Returns the base address of the PixelBuffer.
    @discussion Retrieving the base address for a PixelBuffer requires that the buffer base address be locked
                via a successful call to CVPixelBufferLockBaseAddress.
    @param      pixelBuffer Target PixelBuffer.
    @result     Base address of the pixels.
		For chunky buffers, this will return a pointer to the pixel at 0,0 in the buffer
		For planar buffers this will return a pointer to a PlanarComponentInfo struct (defined in QuickTime).
}
function CVPixelBufferGetBaseAddress( pixelBuffer: CVPixelBufferRef ): UnivPtr; external name '_CVPixelBufferGetBaseAddress';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetBytesPerRow
    @abstract   Returns the rowBytes of the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @result     Bytes per row of the image data.   For planar buffers this will return a rowBytes value such that bytesPerRow * height
                will cover the entire image including all planes.
}
function CVPixelBufferGetBytesPerRow( pixelBuffer: CVPixelBufferRef ): size_t; external name '_CVPixelBufferGetBytesPerRow';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetDataSize
    @abstract   Returns the data size for contigous planes of the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @result     Data size used in CVPixelBufferCreateWithPlanarBytes.
}
function CVPixelBufferGetDataSize( pixelBuffer: CVPixelBufferRef ): size_t; external name '_CVPixelBufferGetDataSize';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferIsPlanar
    @abstract   Returns if the PixelBuffer is planar.
    @param      pixelBuffer Target PixelBuffer.
    @result     True if the PixelBuffer was created using CVPixelBufferCreateWithPlanarBytes.
}
function CVPixelBufferIsPlanar( pixelBuffer: CVPixelBufferRef ): Boolean; external name '_CVPixelBufferIsPlanar';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetPlaneCount
    @abstract   Returns number of planes of the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @result     Number of planes.  Returns 0 for non-planar CVPixelBufferRefs.
}
function CVPixelBufferGetPlaneCount( pixelBuffer: CVPixelBufferRef ): size_t; external name '_CVPixelBufferGetPlaneCount';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetWidthOfPlane
    @abstract   Returns the width of the plane at planeIndex in the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @param      planeIndex  Identifying the plane.
    @result     Width in pixels, or 0 for non-planar CVPixelBufferRefs.
}
function CVPixelBufferGetWidthOfPlane( pixelBuffer: CVPixelBufferRef; planeIndex: size_t ): size_t; external name '_CVPixelBufferGetWidthOfPlane';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetHeightOfPlane
    @abstract   Returns the height of the plane at planeIndex in the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @param      planeIndex  Identifying the plane.
    @result     Height in pixels, or 0 for non-planar CVPixelBufferRefs.
}
function CVPixelBufferGetHeightOfPlane( pixelBuffer: CVPixelBufferRef; planeIndex: size_t ): size_t; external name '_CVPixelBufferGetHeightOfPlane';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetBaseAddressOfPlane
    @abstract   Returns the base address of the plane at planeIndex in the PixelBuffer.
    @discussion Retrieving the base address for a PixelBuffer requires that the buffer base address be locked
                via a successful call to CVPixelBufferLockBaseAddress.
    @param      pixelBuffer Target PixelBuffer.
    @param      planeIndex  Identifying the plane.
    @result     Base address of the plane, or NULL for non-planar CVPixelBufferRefs.
}
function CVPixelBufferGetBaseAddressOfPlane( pixelBuffer: CVPixelBufferRef; planeIndex: size_t ): UnivPtr; external name '_CVPixelBufferGetBaseAddressOfPlane';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetBytesPerRowOfPlane
    @abstract   Returns the row bytes of the plane at planeIndex in the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @param      planeIndex  Identifying the plane.
    @result     Row bytes of the plane, or NULL for non-planar CVPixelBufferRefs.
}
function CVPixelBufferGetBytesPerRowOfPlane( pixelBuffer: CVPixelBufferRef; planeIndex: size_t ): size_t; external name '_CVPixelBufferGetBytesPerRowOfPlane';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2

{!
    @function   CVPixelBufferGetExtendedPixels
    @abstract   Returns the size of extended pixels of the PixelBuffer.
    @param      pixelBuffer Target PixelBuffer.
    @param      extraColumnsOnLeft Returns the pixel row padding to the left.  May be NULL.
    @param      extraRowsOnTop Returns the pixel row padding to the top.  May be NULL. 
    @param      extraColumnsOnRight Returns the pixel row padding to the right. May be NULL.
    @param      extraRowsOnBottom Returns the pixel row padding to the bottom. May be NULL.
}
procedure CVPixelBufferGetExtendedPixels( pixelBuffer: CVPixelBufferRef; var extraColumnsOnLeft: size_t; var extraColumnsOnRight: size_t; var extraRowsOnTop: size_t; var extraRowsOnBottom: size_t ); external name '_CVPixelBufferGetExtendedPixels';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)

{!
    @function   CVPixelBufferFillExtendedPixels
    @abstract   Fills the extended pixels of the PixelBuffer.   This function replicates edge pixels to fill the entire extended region of the image.
    @param      pixelBuffer Target PixelBuffer.
}
function CVPixelBufferFillExtendedPixels( pixelBuffer: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferFillExtendedPixels';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_4_0) *)

<<<<<<< HEAD
=======
{$ifc COREVIDEO_SUPPORTS_IOSURFACE}

{!
	@function   CVPixelBufferGetIOSurface
	@abstract   Returns the IOSurface backing the pixel buffer, or NULL if it is not backed by an IOSurface.
	@param      pixelBuffer Target PixelBuffer.
}
function CVPixelBufferGetIOSurface( pixelBuffer: CVPixelBufferRef ): IOSurfaceRef; external name '_CVPixelBufferGetIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
    @function   CVPixelBufferCreateWithIOSurface
    @abstract   Call to create a single CVPixelBuffer for a passed-in IOSurface.
    @discussion The CVPixelBuffer will retain the IOSurface.
    	IMPORTANT NOTE: If you are using IOSurface to share CVPixelBuffers between processes
    	and those CVPixelBuffers are allocated via a CVPixelBufferPool, it is important
    	that the CVPixelBufferPool does not reuse CVPixelBuffers whose IOSurfaces are still
    	in use in other processes.  
    	CoreVideo and IOSurface will take care of this for if you use IOSurfaceCreateMachPort 
    	and IOSurfaceLookupFromMachPort, but NOT if you pass IOSurfaceIDs.
    @param      surface		            The IOSurface to wrap.
    @param      pixelBufferAttributes   A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
    @param      pixelBufferOut          The new pixel buffer will be returned here
    @result     returns kCVReturnSuccess on success.
}
function CVPixelBufferCreateWithIOSurface( allocator: CFAllocatorRef; surface: IOSurfaceRef; pixelBufferAttributes: CFDictionaryRef; var pixelBufferOut: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferCreateWithIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{$endc} // COREVIDEO_SUPPORTS_IOSURFACE

<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$ifc COREVIDEO_SUPPORTS_IOSURFACE}

{!
	@function   CVPixelBufferGetIOSurface
	@abstract   Returns the IOSurface backing the pixel buffer, or NULL if it is not backed by an IOSurface.
	@param      pixelBuffer Target PixelBuffer.
}
function CVPixelBufferGetIOSurface( pixelBuffer: CVPixelBufferRef ): IOSurfaceRef; external name '_CVPixelBufferGetIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
    @function   CVPixelBufferCreateWithIOSurface
    @abstract   Call to create a single CVPixelBuffer for a passed-in IOSurface.
    @discussion The CVPixelBuffer will retain the IOSurface.
    	IMPORTANT NOTE: If you are using IOSurface to share CVPixelBuffers between processes
    	and those CVPixelBuffers are allocated via a CVPixelBufferPool, it is important
    	that the CVPixelBufferPool does not reuse CVPixelBuffers whose IOSurfaces are still
    	in use in other processes.  
    	CoreVideo and IOSurface will take care of this for if you use IOSurfaceCreateMachPort 
    	and IOSurfaceLookupFromMachPort, but NOT if you pass IOSurfaceIDs.
    @param      surface		            The IOSurface to wrap.
    @param      pixelBufferAttributes   A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
    @param      pixelBufferOut          The new pixel buffer will be returned here
    @result     returns kCVReturnSuccess on success.
}
function CVPixelBufferCreateWithIOSurface( allocator: CFAllocatorRef; surface: IOSurfaceRef; pixelBufferAttributes: CFDictionaryRef; var pixelBufferOut: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferCreateWithIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{$endc} // COREVIDEO_SUPPORTS_IOSURFACE

<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$ifc COREVIDEO_SUPPORTS_IOSURFACE}

{!
	@function   CVPixelBufferGetIOSurface
	@abstract   Returns the IOSurface backing the pixel buffer, or NULL if it is not backed by an IOSurface.
	@param      pixelBuffer Target PixelBuffer.
}
function CVPixelBufferGetIOSurface( pixelBuffer: CVPixelBufferRef ): IOSurfaceRef; external name '_CVPixelBufferGetIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
    @function   CVPixelBufferCreateWithIOSurface
    @abstract   Call to create a single CVPixelBuffer for a passed-in IOSurface.
    @discussion The CVPixelBuffer will retain the IOSurface.
    	IMPORTANT NOTE: If you are using IOSurface to share CVPixelBuffers between processes
    	and those CVPixelBuffers are allocated via a CVPixelBufferPool, it is important
    	that the CVPixelBufferPool does not reuse CVPixelBuffers whose IOSurfaces are still
    	in use in other processes.  
    	CoreVideo and IOSurface will take care of this for if you use IOSurfaceCreateMachPort 
    	and IOSurfaceLookupFromMachPort, but NOT if you pass IOSurfaceIDs.
    @param      surface		            The IOSurface to wrap.
    @param      pixelBufferAttributes   A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
    @param      pixelBufferOut          The new pixel buffer will be returned here
    @result     returns kCVReturnSuccess on success.
}
function CVPixelBufferCreateWithIOSurface( allocator: CFAllocatorRef; surface: IOSurfaceRef; pixelBufferAttributes: CFDictionaryRef; var pixelBufferOut: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferCreateWithIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{$endc} // COREVIDEO_SUPPORTS_IOSURFACE

<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$ifc COREVIDEO_SUPPORTS_IOSURFACE}

{!
	@function   CVPixelBufferGetIOSurface
	@abstract   Returns the IOSurface backing the pixel buffer, or NULL if it is not backed by an IOSurface.
	@param      pixelBuffer Target PixelBuffer.
}
function CVPixelBufferGetIOSurface( pixelBuffer: CVPixelBufferRef ): IOSurfaceRef; external name '_CVPixelBufferGetIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
    @function   CVPixelBufferCreateWithIOSurface
    @abstract   Call to create a single CVPixelBuffer for a passed-in IOSurface.
    @discussion The CVPixelBuffer will retain the IOSurface.
    	IMPORTANT NOTE: If you are using IOSurface to share CVPixelBuffers between processes
    	and those CVPixelBuffers are allocated via a CVPixelBufferPool, it is important
    	that the CVPixelBufferPool does not reuse CVPixelBuffers whose IOSurfaces are still
    	in use in other processes.  
    	CoreVideo and IOSurface will take care of this for if you use IOSurfaceCreateMachPort 
    	and IOSurfaceLookupFromMachPort, but NOT if you pass IOSurfaceIDs.
    @param      surface		            The IOSurface to wrap.
    @param      pixelBufferAttributes   A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
    @param      pixelBufferOut          The new pixel buffer will be returned here
    @result     returns kCVReturnSuccess on success.
}
function CVPixelBufferCreateWithIOSurface( allocator: CFAllocatorRef; surface: IOSurfaceRef; pixelBufferAttributes: CFDictionaryRef; var pixelBufferOut: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferCreateWithIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{$endc} // COREVIDEO_SUPPORTS_IOSURFACE

{$endc} {TARGET_OS_MAC}
>>>>>>> origin/cpstrnew
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
    @function   CVPixelBufferFillExtendedPixels
    @abstract   Fills the extended pixels of the PixelBuffer with Zero.   This function replicates edge pixels to fill the entire extended region of the image.
    @param      pixelBuffer Target PixelBuffer.
}
function CVPixelBufferFillExtendedPixels( pixelBuffer: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferFillExtendedPixels';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{$ifc COREVIDEO_SUPPORTS_IOSURFACE}

{!
	@function   CVPixelBufferGetIOSurface
	@abstract   Returns the IOSurface backing the pixel buffer, or NULL if it is not backed by an IOSurface.
	@param      pixelBuffer Target PixelBuffer.
}
function CVPixelBufferGetIOSurface( pixelBuffer: CVPixelBufferRef ): IOSurfaceRef; external name '_CVPixelBufferGetIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
    @function   CVPixelBufferCreateWithIOSurface
    @abstract   Call to create a single CVPixelBuffer for a passed-in IOSurface.
    @discussion The CVPixelBuffer will retain the IOSurface.
    	IMPORTANT NOTE: If you are using IOSurface to share CVPixelBuffers between processes
    	and those CVPixelBuffers are allocated via a CVPixelBufferPool, it is important
    	that the CVPixelBufferPool does not reuse CVPixelBuffers whose IOSurfaces are still
    	in use in other processes.  
    	CoreVideo and IOSurface will take care of this for if you use IOSurfaceCreateMachPort 
    	and IOSurfaceLookupFromMachPort, but NOT if you pass IOSurfaceIDs.
    @param      surface		            The IOSurface to wrap.
    @param      pixelBufferAttributes   A dictionary with additional attributes for a a pixel buffer. This parameter is optional. See PixelBufferAttributes for more details.
    @param      pixelBufferOut          The new pixel buffer will be returned here
    @result     returns kCVReturnSuccess on success.
}
function CVPixelBufferCreateWithIOSurface( allocator: CFAllocatorRef; surface: IOSurfaceRef; pixelBufferAttributes: CFDictionaryRef; var pixelBufferOut: CVPixelBufferRef ): CVReturn; external name '_CVPixelBufferCreateWithIOSurface';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{$endc} // COREVIDEO_SUPPORTS_IOSURFACE

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
