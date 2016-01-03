{
 * ImageIO - CGImageSource.h
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2004-2010 Apple Inc. All rights reserved.
 *
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{	 Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{	 Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{	 Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{	 Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit CGImageSource;
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
=======
>>>>>>> origin/fixes_2.4
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFURL, CGDataProvider, CGImage, CGImageMetadata;
{$endc} {not MACOSALLINCLUDE}


=======
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFURL, CGDataProvider, CGImage;
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
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFURL, CGDataProvider, CGImage;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFURL, CGDataProvider, CGImage;
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
	CGImageSourceRef = ^OpaqueCGImageSourceRef; { an opaque type }
	OpaqueCGImageSourceRef = record end;
=======
	CGImageSourceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageSourceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageSourceRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGImageSourceRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CGImageSourceRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CGImageSourceRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CGImageSourceRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


type
	CGImageSourceStatus = SInt32;
const
	kCGImageStatusUnexpectedEOF = -5;
	kCGImageStatusInvalidData = -4;
	kCGImageStatusUnknownType = -3;
	kCGImageStatusReadingHeader = -2;
	kCGImageStatusIncomplete = -1;
	kCGImageStatusComplete = 0;

{* Keys for the options dictionary when creating a CGImageSourceRef. *}

{ Specifies the "best guess" of the type identifier for the format of the
 * image source file. If specified, the value of this key must be a
 * CFStringRef. For more information about type identifiers, see "UTType.h"
 * in the Application Services framework. }

var kCGImageSourceTypeIdentifierHint: CFStringRef; external name '_kCGImageSourceTypeIdentifierHint'; (* attribute const *)
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

{* Keys for the options dictionary of "CGImageSourceCopyPropertiesAtIndex"
 ** and "CGImageSourceCreateImageAtIndex". *}

{ Specifies whether the image should be cached in a decoded form. The
 * value of this key must be a CFBooleanRef; the default value is
 * kCFBooleanFalse. }

var kCGImageSourceShouldCache: CFStringRef; external name '_kCGImageSourceShouldCache'; (* attribute const *)
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

{ Specifies whether the image should be returned as a floating
 * point CGImageRef if supported by the file format. Extended
 * range floating point CGImageRef may require additional
 * processing  to render pleasingly.  The value of this key must
 * be a CFBooleanRef; the default value is kCFBooleanFalse. }

var kCGImageSourceShouldAllowFloat: CFStringRef; external name '_kCGImageSourceShouldAllowFloat'; (* attribute const *)
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


{* Keys for the options dictionary of
 ** "CGImageSourceCreateThumbnailAtIndex". *}

{ Specifies whether a thumbnail should be automatically created for an
 * image if a thumbnail isn't present in the image source file.  The
 * thumbnail will be created from the full image, subject to the limit
 * specified by kCGImageSourceThumbnailMaxPixelSize---if a maximum pixel
 * size isn't specified, then the thumbnail will be the size of the full
 * image, which probably isn't what you want. The value of this key must be
 * a CFBooleanRef; the default value of this key is kCFBooleanFalse. }

var kCGImageSourceCreateThumbnailFromImageIfAbsent: CFStringRef; external name '_kCGImageSourceCreateThumbnailFromImageIfAbsent'; (* attribute const *)
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

{ Specifies whether a thumbnail should be created from the full image even
 * if a thumbnail is present in the image source file. The thumbnail will
 * be created from the full image, subject to the limit specified by
 * kCGImageSourceThumbnailMaxPixelSize---if a maximum pixel size isn't
 * specified, then the thumbnail will be the size of the full image, which
 * probably isn't what you want. The value of this key must be a
 * CFBooleanRef; the default value of this key is kCFBooleanFalse. }

var kCGImageSourceCreateThumbnailFromImageAlways: CFStringRef; external name '_kCGImageSourceCreateThumbnailFromImageAlways'; (* attribute const *)
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

{ Specifies the maximum width and height in pixels of a thumbnail.  If
 * this this key is not specified, the width and height of a thumbnail is
 * not limited and thumbnails may be as big as the image itself.  If
 * present, this value of this key must be a CFNumberRef. }

var kCGImageSourceThumbnailMaxPixelSize: CFStringRef; external name '_kCGImageSourceThumbnailMaxPixelSize'; (* attribute const *)
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

{ Specifies whether the thumbnail should be rotated and scaled according
 * to the orientation and pixel aspect ratio of the full image. The value
 * of this key must be a CFBooleanRef; the default value of this key is 
 * kCFBooleanFalse. }

var kCGImageSourceCreateThumbnailWithTransform: CFStringRef; external name '_kCGImageSourceCreateThumbnailWithTransform'; (* attribute const *)
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


{ Return the CFTypeID for CGImageSources. }

function CGImageSourceGetTypeID: CFTypeID; external name '_CGImageSourceGetTypeID';
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

function CGImageSourceCopyTypeIdentifiers: CFArrayRef; external name '_CGImageSourceCopyTypeIdentifiers';
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

{ Create an image source reading from the data provider `provider'. The
 * `options' dictionary may be used to request additional creation options;
 * see the list of keys above for more information. }

function CGImageSourceCreateWithDataProvider( provider: CGDataProviderRef; options: CFDictionaryRef ): CGImageSourceRef; external name '_CGImageSourceCreateWithDataProvider';
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

{ Create an image source reading from `data'.  The `options' dictionary
 * may be used to request additional creation options; see the list of keys
 * above for more information. }

function CGImageSourceCreateWithData( data: CFDataRef; options: CFDictionaryRef ): CGImageSourceRef; external name '_CGImageSourceCreateWithData';
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

{ Create an image source reading from `url'. The `options' dictionary may
 * be used to request additional creation options; see the list of keys
 * above for more information. }

function CGImageSourceCreateWithURL( url: CFURLRef; options: CFDictionaryRef ): CGImageSourceRef; external name '_CGImageSourceCreateWithURL';
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

{ Return the type identifier of the image source `isrc'.  This type is the
 * type of the source "container", which is not necessarily the type of the
 * image(s) in the container.  For example, the .icns format supports
 * embedded JPEG2000 but the source type will be "com.apple.icns". }

function CGImageSourceGetType( isrc: CGImageSourceRef ): CFStringRef; external name '_CGImageSourceGetType';
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

{ Return the number of images (not including thumbnails) in the image
 * source `isrc'. }

function CGImageSourceGetCount( isrc: CGImageSourceRef ): size_t; external name '_CGImageSourceGetCount';
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

{ Return the properties of the image source `isrc'.  These properties
 * apply to the container in general but not necessarily to any individual
 * image that it contains. }

function CGImageSourceCopyProperties( isrc: CGImageSourceRef; options: CFDictionaryRef ): CFDictionaryRef; external name '_CGImageSourceCopyProperties';
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

{ Return the properties of the image at `index' in the image source
 * `isrc'.  The index is zero-based. The `options' dictionary may be used
 * to request additional options; see the list of keys above for more
 * information. }

function CGImageSourceCopyPropertiesAtIndex( isrc: CGImageSourceRef; index: size_t; options: CFDictionaryRef ): CFDictionaryRef; external name '_CGImageSourceCopyPropertiesAtIndex';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)

{$ifc TARGET_OS_MAC}
{ Return the metadata of the image at `index' in the image source
 * `isrc'. The index is zero-based. The `options' dictionary may be used
 * to request additional options; see the list of keys above for more
 * information. Please refer to CGImageMetadata.h for usage of metadata. }
function CGImageSourceCopyMetadataAtIndex( isrc: CGImageSourceRef; index: size_t; options: CFDictionaryRef ): CGImageMetadataRef; external name '_CGImageSourceCopyMetadataAtIndex';
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_8,__IPHONE_NA) *)
{$endc} { TARGET_OS_MAC }
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

{ Return the image at `index' in the image source `isrc'.  The index is
 * zero-based. The `options' dictionary may be used to request additional
 * creation options; see the list of keys above for more information. }

function CGImageSourceCreateImageAtIndex( isrc: CGImageSourceRef; index: size_t; options: CFDictionaryRef ): CGImageRef; external name '_CGImageSourceCreateImageAtIndex';
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

{ Return the thumbnail of the image at `index' in the image source `isrc'.
 * The index is zero-based. The `options' dictionary may be used to request
 * additional thumbnail creation options; see the list of keys above for
 * more information. }

function CGImageSourceCreateThumbnailAtIndex( isrc: CGImageSourceRef; index: size_t; options: CFDictionaryRef ): CGImageRef; external name '_CGImageSourceCreateThumbnailAtIndex';
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

{ Create an incremental image source. No data is provided at creation
 * time; it is assumed that data will eventually be provided using
 * "CGImageSourceUpdateDataProvider" or "CGImageSourceUpdateData".  The
 * `options' dictionary may be used to request additional creation options;
 * see the list of keys above for more information. }

function CGImageSourceCreateIncremental( options: CFDictionaryRef ): CGImageSourceRef; external name '_CGImageSourceCreateIncremental';
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

{ Update the incremental image source `isrc' with new data.  The new data
 * must include all the previous data plus any additional new data. The
 * `final' parameter should be true when the final set of data is provided;
 * false otherwise. }

procedure CGImageSourceUpdateData( isrc: CGImageSourceRef; data: CFDataRef; final: CBool ); external name '_CGImageSourceUpdateData';
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

{ Update the incremental image source `isrc' with a new data provider.
 * The new data provider must provide all the previous data plus any
 * additional new data. The `final' parameter should be true when the final
 * set of data is provided; false otherwise. }

procedure CGImageSourceUpdateDataProvider( isrc: CGImageSourceRef; provider: CGDataProviderRef; final: CBool ); external name '_CGImageSourceUpdateDataProvider';
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

{ Return the overall status of the image source `isrc'.  The status is
 * particularly informative for incremental image sources, but may be used
 * by clients providing non-incremental data as well. }

function CGImageSourceGetStatus( isrc: CGImageSourceRef ): CGImageSourceStatus; external name '_CGImageSourceGetStatus';
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

{ Return the current status of the image at `index' in the image source
 * `isrc'. The index is zero-based. The returned status is particularly
 * informative for incremental image sources but may used by clients
 * providing non-incremental data as well. }

function CGImageSourceGetStatusAtIndex( isrc: CGImageSourceRef; index: size_t ): CGImageSourceStatus; external name '_CGImageSourceGetStatusAtIndex';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* IMAGEIO_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_4_0) *)

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)


{$endc} {TARGET_OS_MAC}
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
