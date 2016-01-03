{
 * ImageIO - CGImageDestination.h
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2004-2010 Apple Inc. All rights reserved.
 *
 }

<<<<<<< HEAD
{  Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{	 Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
 * Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
 *
 }

{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{	 Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }

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

unit CGImageDestination;
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
<<<<<<< HEAD
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
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFError, CFURL, CGDataConsumer, CGImage, CGImageSource, CGImageMetadata;
{$endc} {not MACOSALLINCLUDE}


=======
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFURL, CGDataConsumer, CGImage, CGImageSource;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

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
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFURL, CGDataConsumer, CGImage, CGImageSource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFURL, CGDataConsumer, CGImage, CGImageSource;
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

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
	CGImageDestinationRef = ^OpaqueCGImageDestinationRef; { an opaque type }
	OpaqueCGImageDestinationRef = record end;
=======
	CGImageDestinationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageDestinationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageDestinationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageDestinationRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CGImageDestinationRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CGImageDestinationRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CGImageDestinationRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


{* Properties which may be passed to "CGImageDestinationAddImage"
 ** or "CGImageDestinationAddImageFromSource" to effect the output.
 ** The values apply to a single image of an image destination. *}


{ The desired compression quality to use when writing to an image 
 * destination. If present, the value of this key is a CFNumberRef 
 * in the range 0.0 to 1.0. A value of 1.0 implies lossless
 * compression is desired if destination format supports it. 
 * A value of 0.0 implies that that maximum compression is 
 * desired. }

var kCGImageDestinationLossyCompressionQuality: CFStringRef; external name '_kCGImageDestinationLossyCompressionQuality'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4


{ The desired background color to composite against when writing 
 * an image with alpha to a destination format that does not support 
 * alpha. If present, the value of this key is a CGColorRef without
 * any alpha component of its own.  If not present a white color
 * will be used if needed. }

var kCGImageDestinationBackgroundColor: CFStringRef; external name '_kCGImageDestinationBackgroundColor'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4


{ Return the CFTypeID for CGImageDestinations. }

function CGImageDestinationGetTypeID: CFTypeID; external name '_CGImageDestinationGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Return an array of supported type identifiers. }

function CGImageDestinationCopyTypeIdentifiers: CFArrayRef; external name '_CGImageDestinationCopyTypeIdentifiers';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Create an image destination writing to the data consumer `consumer'.
 * The parameter `type' specifies the type identifier of the resulting
 * image file.  The parameter `count' specifies number of images (not
 * including thumbnails) that the image file will contain. The `options'
 * dictionary is reserved for future use; currently, you should pass NULL
 * for this parameter. }

function CGImageDestinationCreateWithDataConsumer( consumer: CGDataConsumerRef; typ: CFStringRef; count: size_t; options: CFDictionaryRef ): CGImageDestinationRef; external name '_CGImageDestinationCreateWithDataConsumer';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Create an image destination writing to `data'. The parameter `type'
 * specifies the type identifier of the resulting image file.  The
 * parameter `count' specifies number of images (not including thumbnails)
 * that the image file will contain. The `options' dictionary is reserved
 * for future use; currently, you should pass NULL for this parameter. }

function CGImageDestinationCreateWithData( data: CFMutableDataRef; typ: CFStringRef; count: size_t; options: CFDictionaryRef ): CGImageDestinationRef; external name '_CGImageDestinationCreateWithData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Create an image destination writing to `url'. The parameter `type'
 * specifies the type identifier of the resulting image file.  The
 * parameter `count' specifies number of images (not including thumbnails)
 * that the image file will contain. The `options' dictionary is reserved
 * for future use; currently, you should pass NULL for this parameter.
 * Note that if `url' already exists, it will be overwritten. }

function CGImageDestinationCreateWithURL( url: CFURLRef; typ: CFStringRef; count: size_t; options: CFDictionaryRef ): CGImageDestinationRef; external name '_CGImageDestinationCreateWithURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Specify the dictionary `properties' of properties which apply to all
 * images in the image destination `idst'. }

procedure CGImageDestinationSetProperties( idst: CGImageDestinationRef; properties: CFDictionaryRef ); external name '_CGImageDestinationSetProperties';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Set the next image in the image destination `idst' to be `image' with
 * optional properties specified in `properties'.  An error is logged if
 * more images are added than specified in the original count of the image
 * destination. }

procedure CGImageDestinationAddImage( idst: CGImageDestinationRef; image: CGImageRef; properties: CFDictionaryRef ); external name '_CGImageDestinationAddImage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Set the next image in the image destination `idst' to be the image at
 * `index' in the image source `isrc'.  The index is zero-based. The
 * properties of the source image can be added to or overriden by supplying
 * additional keys/values in `properties'.  If a key in `properties' has
 * the value kCFNull, the corresponding property in the destination will be
 * removed. }

procedure CGImageDestinationAddImageFromSource( idst: CGImageDestinationRef; isrc: CGImageSourceRef; index: size_t; properties: CFDictionaryRef ); external name '_CGImageDestinationAddImageFromSource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ Write everything to the destination data, url or consumer of the image
 * destination `idst'.  You must call this function or the image
 * destination will not be valid.  After this function is called, no
 * additional data will be written to the image destination.  Return true
 * if the image was successfully written; false otherwise. }

function CGImageDestinationFinalize( idst: CGImageDestinationRef ): CBool; external name '_CGImageDestinationFinalize';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)

{$ifc TARGET_OS_MAC}
{ Set the next image in the image destination `idst' to be `image' with
 * metadata properties specified in `metadata'. An error is logged if more
 * images are added than specified in the original count of the image
 * destination. }
procedure CGImageDestinationAddImageAndMetadata( idst: CGImageDestinationRef; image: CGImageRef; metadata: CGImageMetadataRef; options: CFDictionaryRef ); external name '_CGImageDestinationAddImageAndMetadata';
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

{*
 ** Keys which may be used in the 'options' dictionary of
 ** "CGImageDestinationCopyImageSource" to effect the output.
 *}

{ Set the metadata tags for the image destination. If present, the value of
 * this key is a CGImageMetadataRef. By default, all EXIF, IPTC, and XMP tags
 * will be replaced. Use kCGImageDestinationMergeMetadata to merge the tags
 * with the existing tags in the image source.
 }
var kCGImageDestinationMetadata: CFStringRef; external name '_kCGImageDestinationMetadata'; (* attribute const *)
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

{ If true, The metadata will be copied from the source and merged with the tags
 * specified in kCGImageDestinationMetadata. If a tag does not exist in the 
 * source, it will be added. If the tag exists in the source, it will be 
 * updated. A metadata tag can be removed by setting the tag's value to 
 * kCFNull. If present, the value of this key is a CFBoooleanRef. The default
 * is kCFBooleanFalse.
 } 
var kCGImageDestinationMergeMetadata: CFStringRef; external name '_kCGImageDestinationMergeMetadata'; (* attribute const *)
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

{ XMP data will not be written to the destination. If used in conjunction with 
 * kCGImageDestinationMetadata, EXIF and/or IPTC tags will be preserved, but 
 * an XMP packet will not be written to the file. If present, the value for 
 * this key is a CFBooleanRef. The default is kCFBooleanFalse.
 }
var kCGImageMetadataShouldExcludeXMP: CFStringRef; external name '_kCGImageMetadataShouldExcludeXMP'; (* attribute const *)
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

{ Updates the DateTime parameters of the image metadata. Only values
 * present in the original image will updated. If present, the value should
 * be a CFStringRef or a CFDateRef. If CFString, the value must be in 
 * Exif DateTime or ISO 8601 DateTime format. This option is mutually
 * exclusive with kCGImageDestinationMetadata.
 }
var kCGImageDestinationDateTime: CFStringRef; external name '_kCGImageDestinationDateTime'; (* attribute const *)
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

{ Updates the orientation in the image metadata. The image data itself will
 * not be rotated. If present, the value should be a CFNumberRef from 1 to 8. 
 * This option is mutually exclusive with kCGImageDestinationMetadata.
 }
var kCGImageDestinationOrientation: CFStringRef; external name '_kCGImageDestinationOrientation'; (* attribute const *)
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

{ Losslessly copies the contents of the image source, 'isrc', to the 
 * destination, 'idst'. The image data will not be modified. The image's 
 * metadata can be modified by adding the keys and values defined above to 
 * 'options'. No other images should be added to the image destination. 
 * CGImageDestinationFinalize() should not be called afterward -
 * the result is saved to the destination when this function returns. 
 * The image type of the destination must match the image source. Returns true
 * if the operation was successful. If an error occurs, false will be returned 
 * and 'err' will be set to a CFErrorRef. Not all image formats are supported 
 * for this operation. }
function CGImageDestinationCopyImageSource( idst: CGImageDestinationRef; isrc: CGImageSourceRef; options: CFDictionaryRef; var err: CFErrorRef ): CBool; external name '_CGImageDestinationCopyImageSource';
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_NA) *)

=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

>>>>>>> graemeg/cpstrnew
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

>>>>>>> origin/cpstrnew
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

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
