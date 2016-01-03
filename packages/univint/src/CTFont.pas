{
 *  CTFont.h
 *  CoreText
 *
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Copyright (c) 2006-2012 Apple Inc. All rights reserved.
 *
 }
{  Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 *  Copyright (c) 2006-2008 Apple Inc. All rights reserved.
 *
 }
{       Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit CTFont;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
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


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CTFontDescriptor,CTFontTraits,ATSTypes,CFBase,CFArray,CFCharacterSet,CFData,CFDictionary,CFString,CGBase,CGAffineTransforms,CGContext,CGFont,CGGeometry,CGPath;
{$endc} {not MACOSALLINCLUDE}


=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,CTFontDescriptor,CTFontTraits,ATSTypes,CFBase,CFArray,CFCharacterSet,CFData,CFDictionary,CFString,CGBase,CGAffineTransforms,CGFont,CGGeometry,CGPath;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

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
{$ALIGN POWER}


{!
    @header

    Thread Safety Information

    All functions in this header are thread safe unless otherwise specified.
}


{! --------------------------------------------------------------------------
    @group Font Types
}//--------------------------------------------------------------------------

{!
    @typedef    CTFontRef
    @abstract   The Core Text Font reference.
    @discussion This is a opaque reference to a core font object.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CTFontRef = ^__CTFont; { an opaque type }
	__CTFont = record end;
=======
	CTFontRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFontRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFontRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFontRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CTFontRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetTypeID
    @abstract   Returns the type identifier for Core Text font references.
    @result     The identifier for the opaque type CTFontRef.
}
function CTFontGetTypeID: CFTypeID; external name '_CTFontGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Constants
}//--------------------------------------------------------------------------

// Name specifier constants
{!
    @defined    kCTFontCopyrightNameKey
    @abstract   The name specifier for the copyright name.
}
var kCTFontCopyrightNameKey: CFStringRef; external name '_kCTFontCopyrightNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFamilyNameKey
    @abstract   The name specifier for the family name.
}
var kCTFontFamilyNameKey: CFStringRef; external name '_kCTFontFamilyNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontSubFamilyNameKey
    @abstract   The name specifier for the subfamily name.
}
var kCTFontSubFamilyNameKey: CFStringRef; external name '_kCTFontSubFamilyNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontStyleNameKey
    @abstract   The name specifier for the style name.
}
var kCTFontStyleNameKey: CFStringRef; external name '_kCTFontStyleNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontUniqueNameKey
    @abstract   The name specifier for the unique name.
    @discussion Note that this name is often not unique and should not be
                assumed to be truly unique.
}
var kCTFontUniqueNameKey: CFStringRef; external name '_kCTFontUniqueNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFullNameKey
    @abstract   The name specifier for the full name.
}
var kCTFontFullNameKey: CFStringRef; external name '_kCTFontFullNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontVersionNameKey
    @abstract   The name specifier for the version name.
}
var kCTFontVersionNameKey: CFStringRef; external name '_kCTFontVersionNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontPostScriptNameKey
    @abstract   The name specifier for the Postscript name.
}
var kCTFontPostScriptNameKey: CFStringRef; external name '_kCTFontPostScriptNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontTrademarkNameKey
    @abstract   The name specifier for the trademark name.
}
var kCTFontTrademarkNameKey: CFStringRef; external name '_kCTFontTrademarkNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontManufacturerNameKey
    @abstract   The name specifier for the manufacturer name.
}
var kCTFontManufacturerNameKey: CFStringRef; external name '_kCTFontManufacturerNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontDesignerNameKey
    @abstract   The name specifier for the designer name.
}
var kCTFontDesignerNameKey: CFStringRef; external name '_kCTFontDesignerNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontDescriptionNameKey
    @abstract   The name specifier for the description name.
}
var kCTFontDescriptionNameKey: CFStringRef; external name '_kCTFontDescriptionNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontVendorURLNameKey
    @abstract   The name specifier for the vendor url name.
}
var kCTFontVendorURLNameKey: CFStringRef; external name '_kCTFontVendorURLNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontDesignerURLNameKey
    @abstract   The name specifier for the designer url name.
}
var kCTFontDesignerURLNameKey: CFStringRef; external name '_kCTFontDesignerURLNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontLicenseNameKey
    @abstract   The name specifier for the license name.
}
var kCTFontLicenseNameKey: CFStringRef; external name '_kCTFontLicenseNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontLicenseURLNameKey
    @abstract   The name specifier for the license url name.
}
var kCTFontLicenseURLNameKey: CFStringRef; external name '_kCTFontLicenseURLNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontSampleTextNameKey
    @abstract   The name specifier for the sample text name string.
}
var kCTFontSampleTextNameKey: CFStringRef; external name '_kCTFontSampleTextNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontPostScriptCIDNameKey
    @abstract   The name specifier for the Postscript CID name.
}
var kCTFontPostScriptCIDNameKey: CFStringRef; external name '_kCTFontPostScriptCIDNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Creation
}//--------------------------------------------------------------------------

{!
    @function   CTFontCreateWithName
    @abstract   Returns a new font reference for the given name.

    @param      name
                The font name for which you wish to create a new font reference. A valid PostScript name is preferred, although other font name types will be matched in a fallback manner.

    @param      size
                The point size for the font reference. If 0.0 is specified, the default font size of 12.0 will be used.

    @param      matrix
                The transformation matrix for the font. If unspecified, the identity matrix will be used. Optional.

    @result     This function will return a CTFontRef that best matches the name provided with size and matrix attributes. The name parameter is the only required parameters, and default values will be used for unspecified parameters. A best match will be found if all parameters cannot be matched identically.
}
function CTFontCreateWithName( name: CFStringRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null} ): CTFontRef; external name '_CTFontCreateWithName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreateWithFontDescriptor
    @abstract   Returns a new font reference that best matches the font descriptor.

    @param      descriptor
                A font descriptor containing attributes that specify the requested font.

    @param      size
                The point size for the font reference. If 0.0 is specified, the default font size of 12.0 will be used.

    @param      matrix
                The transformation matrix for the font. If unspecified, the identity matrix will be used. Optional.

    @result     This function will return a CTFontRef that best matches the attributes provided with the font descriptor. The size and matrix parameters will override any specified in the font descriptor, unless they are unspecified. A best match font will always be returned, and default values will be used for any unspecified.
}
function CTFontCreateWithFontDescriptor( descriptor: CTFontDescriptorRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null} ): CTFontRef; external name '_CTFontCreateWithFontDescriptor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @enum       CTFontOptions
    @abstract   Options for descriptor match and font creation.
    @constant   kCTFontOptionsPreventAutoActivation
                Prevents automatic font activation from taking place.
    @constant   kCTFontOptionsPreferSystemFont
                Font matching will prefer to match Apple system fonts.
}
const
	kCTFontOptionsDefault = 0;
	kCTFontOptionsPreventAutoActivation = 1 shl 0;
	kCTFontOptionsPreferSystemFont = 1 shl 2;
type
	CTFontOptions = CFOptionFlags;


{!
    @function   CTFontCreateWithNameAndOptions
    @abstract   Returns a new font reference for the given name.

    @param      name
                The font name for which you wish to create a new font reference. A valid PostScript name is preferred, although other font name types will be matched in a fallback manner.

    @param      size
                The point size for the font reference. If 0.0 is specified, the default font size of 12.0 will be used.

    @param      matrix
                The transformation matrix for the font. If unspecified, the identity matrix will be used. Optional.
                
    @param      options
                Options flags.

    @result     This function will return a CTFontRef that best matches the name provided with size and matrix attributes. The name parameter is the only required parameters, and default values will be used for unspecified parameters. A best match will be found if all parameters cannot be matched identically.
}
function CTFontCreateWithNameAndOptions( name: CFStringRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null}; options: CTFontOptions ): CTFontRef; external name '_CTFontCreateWithNameAndOptions';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreateWithFontDescriptorAndOptions
    @abstract   Returns a new font reference that best matches the font descriptor.

    @param      descriptor
                A font descriptor containing attributes that specify the requested font.

    @param      size
                The point size for the font reference. If 0.0 is specified, the default font size of 12.0 will be used.

    @param      matrix
                The transformation matrix for the font. If unspecified, the identity matrix will be used. Optional.

    @param      options
                Options flags.

    @result     This function will return a CTFontRef that best matches the attributes provided with the font descriptor. The size and matrix parameters will override any specified in the font descriptor, unless they are unspecified. A best match font will always be returned, and default values will be used for any unspecified.
}
function CTFontCreateWithFontDescriptorAndOptions( descriptor: CTFontDescriptorRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null}; options: CTFontOptions ): CTFontRef; external name '_CTFontCreateWithFontDescriptorAndOptions';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @enum       UI Type constants
    @abstract   These constants represent the specific user interface purpose to specify for font creation.
    @discussion Use these constants with CTFontCreateUIFontForLanguage to indicate the intended user interface usage of the font reference to be created.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    kCTFontUIFontNone                           = $ffffffff;
    kCTFontUIFontUser                           =  0;
    kCTFontUIFontUserFixedPitch                 =  1;
    kCTFontUIFontSystem                         =  2;
    kCTFontUIFontEmphasizedSystem               =  3;
    kCTFontUIFontSmallSystem                    =  4;
    kCTFontUIFontSmallEmphasizedSystem          =  5;
    kCTFontUIFontMiniSystem                     =  6;
    kCTFontUIFontMiniEmphasizedSystem           =  7;
    kCTFontUIFontViews                          =  8;
    kCTFontUIFontApplication                    =  9;
    kCTFontUIFontLabel                          = 10;
    kCTFontUIFontMenuTitle                      = 11;
    kCTFontUIFontMenuItem                       = 12;
    kCTFontUIFontMenuItemMark                   = 13;
    kCTFontUIFontMenuItemCmdKey                 = 14;
    kCTFontUIFontWindowTitle                    = 15;
    kCTFontUIFontPushButton                     = 16;
    kCTFontUIFontUtilityWindowTitle             = 17;
    kCTFontUIFontAlertHeader                    = 18;
    kCTFontUIFontSystemDetail                   = 19;
    kCTFontUIFontEmphasizedSystemDetail         = 20;
    kCTFontUIFontToolbar                        = 21;
    kCTFontUIFontSmallToolbar                   = 22;
    kCTFontUIFontMessage                        = 23;
    kCTFontUIFontPalette                        = 24;
    kCTFontUIFontToolTip                        = 25;
    kCTFontUIFontControlContent                 = 26;

    kCTFontNoFontType = kCTFontUIFontNone;
    kCTFontUserFontType = kCTFontUIFontUser;
    kCTFontUserFixedPitchFontType = kCTFontUIFontUserFixedPitch;
    kCTFontSystemFontType = kCTFontUIFontSystem;
    kCTFontEmphasizedSystemFontType = kCTFontUIFontEmphasizedSystem;
    kCTFontSmallSystemFontType = kCTFontUIFontSmallSystem;
    kCTFontSmallEmphasizedSystemFontType = kCTFontUIFontSmallEmphasizedSystem;
    kCTFontMiniSystemFontType = kCTFontUIFontMiniSystem;
    kCTFontMiniEmphasizedSystemFontType = kCTFontUIFontMiniEmphasizedSystem;
    kCTFontViewsFontType = kCTFontUIFontViews;
    kCTFontApplicationFontType = kCTFontUIFontApplication;
    kCTFontLabelFontType = kCTFontUIFontLabel;
    kCTFontMenuTitleFontType = kCTFontUIFontMenuTitle;
    kCTFontMenuItemFontType = kCTFontUIFontMenuItem;
    kCTFontMenuItemMarkFontType = kCTFontUIFontMenuItemMark;
    kCTFontMenuItemCmdKeyFontType = kCTFontUIFontMenuItemCmdKey;
    kCTFontWindowTitleFontType = kCTFontUIFontWindowTitle;
    kCTFontPushButtonFontType = kCTFontUIFontPushButton;
    kCTFontUtilityWindowTitleFontType = kCTFontUIFontUtilityWindowTitle;
    kCTFontAlertHeaderFontType = kCTFontUIFontAlertHeader;
    kCTFontSystemDetailFontType = kCTFontUIFontSystemDetail;
    kCTFontEmphasizedSystemDetailFontType = kCTFontUIFontEmphasizedSystemDetail;
    kCTFontToolbarFontType = kCTFontUIFontToolbar;
    kCTFontSmallToolbarFontType = kCTFontUIFontSmallToolbar;
    kCTFontMessageFontType = kCTFontUIFontMessage;
    kCTFontPaletteFontType = kCTFontUIFontPalette;
    kCTFontToolTipFontType = kCTFontUIFontToolTip;
    kCTFontControlContentFontType = kCTFontUIFontControlContent;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	kCTFontNoFontType = $ffffffff;
	kCTFontUserFontType = 0;
	kCTFontUserFixedPitchFontType = 1;
	kCTFontSystemFontType = 2;
	kCTFontEmphasizedSystemFontType = 3;
	kCTFontSmallSystemFontType = 4;
	kCTFontSmallEmphasizedSystemFontType = 5;
	kCTFontMiniSystemFontType = 6;
	kCTFontMiniEmphasizedSystemFontType = 7;
	kCTFontViewsFontType = 8;
	kCTFontApplicationFontType = 9;
	kCTFontLabelFontType = 10;
	kCTFontMenuTitleFontType = 11;
	kCTFontMenuItemFontType = 12;
	kCTFontMenuItemMarkFontType = 13;
	kCTFontMenuItemCmdKeyFontType = 14;
	kCTFontWindowTitleFontType = 15;
	kCTFontPushButtonFontType = 16;
	kCTFontUtilityWindowTitleFontType = 17;
	kCTFontAlertHeaderFontType = 18;
	kCTFontSystemDetailFontType = 19;
	kCTFontEmphasizedSystemDetailFontType = 20;
	kCTFontToolbarFontType = 21;
	kCTFontSmallToolbarFontType = 22;
	kCTFontMessageFontType = 23;
	kCTFontPaletteFontType = 24;
	kCTFontToolTipFontType = 25;
	kCTFontControlContentFontType = 26;
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
type
	CTFontUIFontType = UInt32;

{!
    @function   CTFontCreateUIFontForLanguage
    @abstract   Returns the special UI font for the given language and UI type.

    @param      uiType
                A uiType constant specifying the intended UI use for the requested font reference.

    @param      size
                The point size for the font reference. If 0.0 is specified, the default size for the requested uiType is used.

    @param      language
                Language specifier string to select a font for a particular localization. If unspecified, the current system language is used. The format of the language identifier should conform to the BCP 47 standard.

    @result     This function returns the correct font for various UI uses. The only required parameter is the uiType selector, unspecified optional parameters will use default values.
}
function CTFontCreateUIFontForLanguage( uiType: CTFontUIFontType; size: CGFloat; language: CFStringRef ): CTFontRef; external name '_CTFontCreateUIFontForLanguage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreateCopyWithAttributes
    @abstract   Returns a new font with additional attributes based on the original font.

    @discussion This function provides a mechanism to quickly change attributes on a given font reference in response to user actions. For instance, the size can be changed in response to a user manipulating a size slider.

    @param      font
                Original font reference to base new font on.

    @param      size
                The point size for the font reference. If 0.0 is specified, the original font's size will be preserved.

    @param      matrix
                The transformation matrix for the font. If unspecified, the original font matrix will be preserved. Optional.

    @param      attributes
                A font descriptor containing additional attributes that the new font should contain.

    @result     Returns a new font reference converted from the original with the specified attributes.
}
function CTFontCreateCopyWithAttributes( font: CTFontRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null}; attributes: CTFontDescriptorRef ): CTFontRef; external name '_CTFontCreateCopyWithAttributes';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreateCopyWithSymbolicTraits
    @abstract   Returns a new font based on the original font with the specified symbolic traits.

    @param      font
                Original font reference on which to base the new font.

    @param      size
                The point size for the font reference. If 0.0 is specified, the original font's size will be preserved.

    @param      matrix
                The transformation matrix for the font. If unspecified, the original font matrix will be preserved. Optional.

    @param      symTraitValue
                The value of the symbolic traits. This bitfield is used to indicate the desired value for the traits specified by the symTraitMask parameter. Used in conjunction, they can allow for trait removal as well as addition.

    @param      symTraitMask
                The mask bits of the symbolic traits. This bitfield is used to indicate the traits that should be changed.

    @result     Returns a new font reference in the same family with the given symbolic traits, or NULL if none found in the system.
}
function CTFontCreateCopyWithSymbolicTraits( font: CTFontRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null}; symTraitValue: CTFontSymbolicTraits; symTraitMask: CTFontSymbolicTraits ): CTFontRef; external name '_CTFontCreateCopyWithSymbolicTraits';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreateCopyWithFamily
    @abstract   Returns a new font in the specified family based on the traits of the original font.

    @param      font
                Original font reference to base new font on.

    @param      size
                The point size for the font reference. If 0.0 is specified, the original font's size will be preserved.

    @param      matrix
                The transformation matrix for the font. If unspecified, the original font matrix will be preserved. Optional.

    @param      family
                The name of the desired family.

    @result     Returns a new font reference with the original traits in the given family. NULL if non found in the system.
}
function CTFontCreateCopyWithFamily( font: CTFontRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null}; family: CFStringRef ): CTFontRef; external name '_CTFontCreateCopyWithFamily';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Cascading
}//--------------------------------------------------------------------------

{!
    @function   CTFontCreateForString
    @abstract   Returns a new font reference that can best map the given string range based on the current font.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @discussion This function is to be used when the current font does not cover the given range of the string. The current font itself will not be returned, but preference is given to fonts in its cascade list.

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
    @param      currentFont
                The current font that contains a valid cascade list.

    @param      string
                A unicode string containing characters that cannot be encoded by the current font.

    @param      range
                A CFRange specifying the range of the string that needs to be mapped.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @result     This function returns the best substitute font that can encode the specified string range.

    @seealso    CTFontCopyCharacterSet
    @seealso    CTFontGetGlyphsForCharacters
    @seealso    kCTFontCascadeListAttribute
}
function CTFontCreateForString( currentFont: CTFontRef; strng: CFStringRef; range: CFRange ): CTFontRef; external name '_CTFontCreateForString';
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
    @result     This function returns the best substitute font from the cascade list of the current font that can encode the specified string range. If the current font is capable of encoding the string range then it will be retained and returned.
}
function CTFontCreateForString( currentFont: CTFontRef; strng: CFStringRef; range: CFRange ): CTFontRef; external name '_CTFontCreateForString';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
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

{! --------------------------------------------------------------------------
    @group Font Accessors
}//--------------------------------------------------------------------------

{!
    @function   CTFontCopyFontDescriptor
    @abstract   Returns the normalized font descriptors for the given font reference.

    @param      font
                The font reference.

    @result     This function returns a normalized font descriptor for a font. The font descriptor contains enough information to recreate this font at a later time.
}
function CTFontCopyFontDescriptor( font: CTFontRef ): CTFontDescriptorRef; external name '_CTFontCopyFontDescriptor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyAttribute
    @abstract   Returns the value associated with an arbitrary attribute.

    @param      font
                The font reference.

    @param      attribute
                The requested attribute.

    @result     This function returns a retained reference to an arbitrary attribute. If the requested attribute is not present, NULL is returned. Refer to the attribute definitions for documentation as to how each attribute is packaged as a CFType.
}
function CTFontCopyAttribute( font: CTFontRef; attribute: CFStringRef ): CFTypeRef; external name '_CTFontCopyAttribute';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetSize
    @abstract   Returns the point size of the font reference.

    @param      font
                The font reference.

    @result     This function returns the point size of the given font reference. This is the point size provided when the font was created.
}
function CTFontGetSize( font: CTFontRef ): CGFloat; external name '_CTFontGetSize';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetMatrix
    @abstract   Returns the transformation matrix of the font.

    @param      font
                The font reference.

    @result     This function returns the transformation matrix for this given font reference. This is the matrix that was provided when the font was created.
}
function CTFontGetMatrix( font: CTFontRef ): CGAffineTransform; external name '_CTFontGetMatrix';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetSymbolicTraits
    @abstract   Returns the symbolic font traits.

    @param      font
                The font reference.

    @result     This function returns the symbolic traits of the font. This is equivalent to the kCTFontSymbolicTrait of traits dictionary. See CTFontTraits.h for a definition of the font traits.
}
function CTFontGetSymbolicTraits( font: CTFontRef ): CTFontSymbolicTraits; external name '_CTFontGetSymbolicTraits';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyTraits
    @abstract   Returns the font traits dictionary.

    @param      font
                The font reference.

    @result     This function returns a retained reference to the font traits dictionary. Individual traits can be accessed with the trait key constants. See CTFontTraits.h for a definition of the font traits.
}
function CTFontCopyTraits( font: CTFontRef ): CFDictionaryRef; external name '_CTFontCopyTraits';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Names
}//--------------------------------------------------------------------------

{!
    @function   CTFontCopyPostScriptName
    @abstract   Returns the PostScript name.

    @param      font
                The font reference.

    @result     This function returns a retained reference to the PostScript name of the font.
}
function CTFontCopyPostScriptName( font: CTFontRef ): CFStringRef; external name '_CTFontCopyPostScriptName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyFamilyName
    @abstract   Returns the family name.

    @param      font
                The font reference.

    @result     This function returns a retained reference to the family name of the font.
}
function CTFontCopyFamilyName( font: CTFontRef ): CFStringRef; external name '_CTFontCopyFamilyName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyFullName
    @abstract   Returns the display name.

    @param      font
                The font reference.

    @result     This function returns a retained reference to the full name of the font.
}
function CTFontCopyFullName( font: CTFontRef ): CFStringRef; external name '_CTFontCopyFullName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyDisplayName
    @abstract   Returns the display name.

    @param      font
                The font reference.

    @result     This function returns a retained reference to the localized display name of the font.
}
function CTFontCopyDisplayName( font: CTFontRef ): CFStringRef; external name '_CTFontCopyDisplayName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyName
    @abstract   Returns a reference to the requested name.

    @param      font
                The font reference.

    @param      nameKey
                The name specifier. See name specifier constants.

    @result     This function creates the requested name for the font, or NULL if the font does not have an entry for the requested name. The Unicode version of the name will be preferred, otherwise the first available will be used.
}
function CTFontCopyName( font: CTFontRef; nameKey: CFStringRef ): CFStringRef; external name '_CTFontCopyName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyLocalizedName
    @abstract   Returns a reference to a localized font name.

    @param      font
                The font reference.

    @param      nameKey
                The name specifier. See name specifier constants.

    @param      language
                Pointer to a CFStringRef to receive the language string of the returned name string. The format of the language identifier will conform to the BCP 47 standard.

    @result     This function returns a specific localized name from the font reference. The name is localized based on the user's global language precedence. If the font does not have an entry for the requested name, NULL will be returned. The matched language will be returned in the caller's buffer.
}
function CTFontCopyLocalizedName( font: CTFontRef; nameKey: CFStringRef; var language: CFStringRef ): CFStringRef; external name '_CTFontCopyLocalizedName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Encoding
}//--------------------------------------------------------------------------

{!
    @function   CTFontCopyCharacterSet
    @abstract   Returns the Unicode character set of the font.

    @param      font
                The font reference.

    @result     This function returns a retained reference to the font's character set. This character set covers the nominal referenced by the font's Unicode cmap table (or equivalent).
}
function CTFontCopyCharacterSet( font: CTFontRef ): CFCharacterSetRef; external name '_CTFontCopyCharacterSet';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetStringEncoding
    @abstract   Returns the best string encoding for legacy format support.

    @param      font
                The font reference.

    @result     This function returns the best string encoding for the font.
}
function CTFontGetStringEncoding( font: CTFontRef ): CFStringEncoding; external name '_CTFontGetStringEncoding';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopySupportedLanguages
    @abstract   Returns an array of languages supported by the font.

    @param      font
                The font reference.

    @result     This function returns a retained reference to an array of languages supported by the font. The array contains language identifier strings as CFStringRefs. The format of the language identifier will conform to the BCP 47 standard.
}
function CTFontCopySupportedLanguages( font: CTFontRef ): CFArrayRef; external name '_CTFontCopySupportedLanguages';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetGlyphsForCharacters
    @abstract   Performs basic character-to-glyph mapping.

    @discussion This function only provides the nominal mapping as specified by the font's Unicode cmap (or equivalent); such mapping does not constitute proper Unicode layout: it is the caller's responsibility to handle the Unicode properties of the characters.

    @param      font
                The font reference.

    @param      characters
                An array of characters (UTF-16 code units). Non-BMP characters must be encoded as surrogate pairs.

    @param      glyphs
                A pointer to a buffer to receive the glyphs. Glyphs for non-BMP characters are sparse: the first glyph corresponds to the full character and the second glyph will be 0.

    @param      count
                The capacity of both the characters and glyphs arrays.

    @result     The return value indicates whether all provided characters were successfully mapped. A return value of true indicates that the font mapped all characters. A return value of false indicates that some or all of the characters were not mapped; glyphs for unmapped characters will be 0 (with the exception of those corresponding non-BMP characters as described above).

    @seealso    CTFontCopyCharacterSet
}
function CTFontGetGlyphsForCharacters( font: CTFontRef; {const} characters: {variable-size-array} UniCharPtr; glyphs: {variable-size-array} CGGlyphPtr; count: CFIndex ): CBool; external name '_CTFontGetGlyphsForCharacters';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Metrics
}//--------------------------------------------------------------------------

{!
    @function   CTFontGetAscent
    @abstract   Returns the scaled font ascent metric.

    @param      font
                The font reference.

    @result     This function returns the font ascent metric scaled based on the point size and matrix of the font reference.
}
function CTFontGetAscent( font: CTFontRef ): CGFloat; external name '_CTFontGetAscent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetDescent
    @abstract   Returns the scaled font descent metric.

    @param      font
                The font reference.

    @result     This function returns the font descent metric scaled based on the point size and matrix of the font reference.
}
function CTFontGetDescent( font: CTFontRef ): CGFloat; external name '_CTFontGetDescent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetLeading
    @abstract   Returns the scaled font leading metric.

    @param      font
                The font reference.

    @result     This function returns the font leading metric scaled based on the point size and matrix of the font reference.
}
function CTFontGetLeading( font: CTFontRef ): CGFloat; external name '_CTFontGetLeading';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetUnitsPerEm
    @abstract   Returns the units per em metric.

    @param      font
                The font reference.

    @result     This function returns the units per em of the font.
}
function CTFontGetUnitsPerEm( font: CTFontRef ): UInt32; external name '_CTFontGetUnitsPerEm';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetGlyphCount
    @abstract   Returns the number of glyphs.

    @param      font
                The font reference.

    @result     This function returns the number of glyphs in the font.
}
function CTFontGetGlyphCount( font: CTFontRef ): CFIndex; external name '_CTFontGetGlyphCount';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetBoundingBox
    @abstract   Returns the scaled bounding box.

    @param      font
                The font reference.

    @result     This will return the design bounding box of the font, which is the rectangle defined by xMin, yMin, xMax, and yMax values for the font.
}
function CTFontGetBoundingBox( font: CTFontRef ): CGRect; external name '_CTFontGetBoundingBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetUnderlinePosition
    @abstract   Returns the scaled underline position.

    @param      font
                The font reference.

    @result     This function returns the font underline position metric scaled based on the point size and matrix of the font reference.
}
function CTFontGetUnderlinePosition( font: CTFontRef ): CGFloat; external name '_CTFontGetUnderlinePosition';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetUnderlineThickness
    @abstract   Returns the scaled underline thickness metric.

    @param      font
                The font reference.

    @result     This function returns the font underline thickness metric scaled based on the point size and matrix of the font reference.
}
function CTFontGetUnderlineThickness( font: CTFontRef ): CGFloat; external name '_CTFontGetUnderlineThickness';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetSlantAngle
    @abstract   Returns the slant angle of the font.

    @param      font
                The font reference.

    @result     This function returns the transformed slant angle of the font. This is equivalent to the italic or caret angle with any skew from the transformation matrix applied.
}
function CTFontGetSlantAngle( font: CTFontRef ): CGFloat; external name '_CTFontGetSlantAngle';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetCapHeight
    @abstract   Returns the cap height metric.

    @param      font
                The font reference.

    @result     This function returns the font cap height metric scaled based on the point size and matrix of the font reference.
}
function CTFontGetCapHeight( font: CTFontRef ): CGFloat; external name '_CTFontGetCapHeight';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetXHeight
    @abstract   Returns the X height metric.

    @param      font
                The font reference.

    @result     This function returns the font X height metric scaled based on the point size and matrix of the font reference.
}
function CTFontGetXHeight( font: CTFontRef ): CGFloat; external name '_CTFontGetXHeight';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Glyphs
}//--------------------------------------------------------------------------

{!
    @function   CTFontGetGlyphWithName
    @abstract   Returns the CGGlyph for the specified glyph name.

    @param      font
                The font reference.

    @param      glyphName
                The glyph name as a CFString.

    @result     The glyph with the specified name or 0 if the name is not recognized; this glyph can be used with other Core Text glyph data accessors or with Quartz.
}
function CTFontGetGlyphWithName( font: CTFontRef; glyphName: CFStringRef ): CGGlyph; external name '_CTFontGetGlyphWithName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetBoundingRectsForGlyphs
    @abstract   Calculates the bounding rects for an array of glyphs and returns the overall bounding rect for the run.

    @param      font
                The font reference.

    @param      orientation
                The intended drawing orientation of the glyphs. Used to determined which glyph metrics to return.

    @param      glyphs
                An array of count number of glyphs.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @param      boundingRects
=======
    @param      boundingRect
>>>>>>> graemeg/cpstrnew
=======
    @param      boundingRect
>>>>>>> graemeg/cpstrnew
=======
    @param      boundingRect
>>>>>>> graemeg/cpstrnew
=======
    @param      boundingRect
>>>>>>> origin/cpstrnew
=======
    @param      boundingRect
>>>>>>> origin/fixes_2.4
                An array of count number of CGRects to receive the computed glyph rects. Can be NULL, in which case only the overall bounding rect is calculated.

    @param      count
                The capacity of the glyphs and boundingRects buffers.

    @result     This function returns the overall bounding rectangle for an array or run of glyphs. The bounding rects of the individual glyphs are returned through the boundingRects parameter. These are the design metrics from the font transformed in font space.
}
function CTFontGetBoundingRectsForGlyphs( font: CTFontRef; orientation: CTFontOrientation; {const} glyphs: {variable-size-array} CGGlyphPtr; boundingRects: {variable-size-array} CGRectPtr; count: CFIndex ): CGRect; external name '_CTFontGetBoundingRectsForGlyphs';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{!
    @function   CTFontGetOpticalBoundsForGlyphs
    @abstract   Calculates the optical bounding rects for an array of glyphs and returns the overall optical bounding rect for the run.

    @discussion Fonts may specify the optical edges of glyphs that can be used to make the edges of lines of text line up in a more visually pleasing way. This function returns bounding rects corresponding to this information if present in a font, otherwise it returns typographic bounding rects (composed of the font's ascent and descent and a glyph's advance width).

    @param      font
                The font reference.

    @param      glyphs
                An array of count number of glyphs.

    @param      boundingRects
                An array of count number of CGRects to receive the computed glyph rects. Can be NULL, in which case only the overall bounding rect is calculated.

    @param      count
                The capacity of the glyphs and boundingRects buffers.

    @param      options
                Reserved, set to zero.

    @result     This function returns the overall bounding rectangle for an array or run of glyphs. The bounding rects of the individual glyphs are returned through the boundingRects parameter. These are the design metrics from the font transformed in font space.
}
function CTFontGetOpticalBoundsForGlyphs( font: CTFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; boundingRects: {variable-size-array} CGRectPtr; count: CFIndex; options: CFOptionFlags ): CGRect; external name '_CTFontGetOpticalBoundsForGlyphs';
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetAdvancesForGlyphs
    @abstract   Calculates the advances for an array of glyphs and returns the summed advance.

    @param      font
                The font reference.

    @param      orientation
                The intended drawing orientation of the glyphs. Used to determined which glyph metrics to return.

    @param      glyphs
                An array of count number of glyphs.

    @param      advances
                An array of count number of CGSize to receive the computed glyph advances. Can be NULL, in which case only the overall advance is calculated.

    @param      count
                The capacity of the glyphs and advances buffers.

    @result     This function returns the summed glyph advance of an array of glyphs. Individual glyph advances are passed back via the advances parameter. These are the ideal metrics for each glyph scaled and transformed in font space.
}
function CTFontGetAdvancesForGlyphs( font: CTFontRef; orientation: CTFontOrientation; {const} glyphs: {variable-size-array} CGGlyphPtr; advances: {variable-size-array} CGSizePtr; count: CFIndex ): Float64; external name '_CTFontGetAdvancesForGlyphs';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontGetVerticalTranslationsForGlyphs
    @abstract   Calculates the offset from the default (horizontal) origin to the vertical origin for an array of glyphs.

    @param      font
                The font reference.

    @param      glyphs
                An array of count number of glyphs.

    @param      translations
                An array of count number of CGSize to receive the computed origin offsets.

    @param      count
                The capacity of the glyphs and translations buffers.
}
procedure CTFontGetVerticalTranslationsForGlyphs( font: CTFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; translations: {variable-size-array} CGSizePtr; count: CFIndex ); external name '_CTFontGetVerticalTranslationsForGlyphs';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreatePathForGlyph
    @abstract   Creates a path for the specified glyph.

    @discussion Creates a path from the outlines of the glyph for the specified font. The path will reflect the font point size, matrix, and transform parameter, in that order. The transform parameter will most commonly be used to provide a translation to the desired glyph origin.

    @param      font
                The font reference.

    @param      glyph
                The glyph.

    @param      transform
                An affine transform applied to the path. Can be NULL, in which case CGAffineTransformIdentity will be used.

    @result     A retained CGPath reference containing the glyph outlines or NULL if an error occurred.
}
function CTFontCreatePathForGlyph( font: CTFontRef; glyph: CGGlyph; transform: {const} CGAffineTransformPtr {can be null} ): CGPathRef; external name '_CTFontCreatePathForGlyph';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{! --------------------------------------------------------------------------
    @group Font Variations        (this functionality is not supported on iOS)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{! --------------------------------------------------------------------------
    @group Font Variations
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
}//--------------------------------------------------------------------------

{!
    @defined    kCTFontVariationAxisIdentifierKey
    @abstract   Key to get the variation axis identifier.
    @discussion This key is used with a variation axis dictionary to get the axis identifier value as a CFNumberRef.
}
var kCTFontVariationAxisIdentifierKey: CFStringRef; external name '_kCTFontVariationAxisIdentifierKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontVariationAxisMinimumValueKey
    @abstract   Key to get the variation axis minimum value.
    @discussion This key is used with a variation axis dictionary to get the minimum axis value as a CFNumberRef.
}
var kCTFontVariationAxisMinimumValueKey: CFStringRef; external name '_kCTFontVariationAxisMinimumValueKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontVariationAxisMaximumValueKey
    @abstract   Key to get the variation axis maximum value.
    @discussion This key is used with a variation axis dictionary to get the maximum axis value as a CFNumberRef.
}
var kCTFontVariationAxisMaximumValueKey: CFStringRef; external name '_kCTFontVariationAxisMaximumValueKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontVariationAxisDefaultValueKey
    @abstract   Key to get the variation axis default value.
    @discussion This key is used with a variation axis dictionary to get the default axis value as a CFNumberRef.
}
var kCTFontVariationAxisDefaultValueKey: CFStringRef; external name '_kCTFontVariationAxisDefaultValueKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontVariationAxisNameKey
    @abstract   Key to get the variation axis name string.
    @discussion This key is used with a variation axis dictionary to get the localized variation axis name.
}
var kCTFontVariationAxisNameKey: CFStringRef; external name '_kCTFontVariationAxisNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyVariationAxes
    @abstract   Returns an array of variation axes.

    @param      font
                The font reference.

    @result     This function returns an array of variation axis dictionaries. Each variation axis dictionary contains the five variation axis keys above.
}
function CTFontCopyVariationAxes( font: CTFontRef ): CFArrayRef; external name '_CTFontCopyVariationAxes';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyVariation
    @abstract   Returns a variation dictionary from the font reference.

    @param      font
                The font reference.

    @result     This function returns the current variation instance as a dictionary. The keys for each variation correspond to the variation identifier obtained via kCTVariationAxisIdentifierKey which represents the axis' four character code as a CFNumber.
}
function CTFontCopyVariation( font: CTFontRef ): CFDictionaryRef; external name '_CTFontCopyVariation';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{! --------------------------------------------------------------------------
    @group Font Features              (only AAT features are supported on iOS)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{! --------------------------------------------------------------------------
    @group Font Features
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
}//--------------------------------------------------------------------------

{!
    @defined    kCTFontFeatureTypeIdentifierKey
    @abstract   Key to get the font feature type value.
    @discussion This key can be used with a font feature dictionary to get the type identifier as a CFNumberRef.
}
var kCTFontFeatureTypeIdentifierKey: CFStringRef; external name '_kCTFontFeatureTypeIdentifierKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFeatureTypeNameKey
    @abstract   Key to get the font feature name.
    @discussion This key can be used with a font feature dictionary to get the localized type name string as a CFString.
}
var kCTFontFeatureTypeNameKey: CFStringRef; external name '_kCTFontFeatureTypeNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFeatureTypeExclusiveKey
    @abstract   Key to get the font feature exclusive setting.
    @discussion This key can be used with a font feature dictionary to get the the exclusive setting of the feature as a CFBoolean. The value associated with this key indicates whether the feature selectors associated with this type should be mutually exclusive.
}
var kCTFontFeatureTypeExclusiveKey: CFStringRef; external name '_kCTFontFeatureTypeExclusiveKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFeatureTypeSelectorsKey
    @abstract   Key to get the font feature selectors.
    @discussion This key can be used with a font feature dictionary to get the array of font feature selectors as a CFArrayRef. This is an array of selector dictionaries that contain the values for the following selector keys.
}
var kCTFontFeatureTypeSelectorsKey: CFStringRef; external name '_kCTFontFeatureTypeSelectorsKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFeatureSelectorIdentifierKey
    @abstract   Key to get the font feature selector identifier.
    @discussion This key can be used with a selector dictionary corresponding to a feature type to obtain the selector identifier value as a CFNumberRef.
}
var kCTFontFeatureSelectorIdentifierKey: CFStringRef; external name '_kCTFontFeatureSelectorIdentifierKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFeatureSelectorNameKey
    @abstract   Key to get the font feature selector name.
    @discussion This key is used with a selector dictionary to get the localized name string for the selector as a CFStringRef.
}
var kCTFontFeatureSelectorNameKey: CFStringRef; external name '_kCTFontFeatureSelectorNameKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFeatureSelectorDefaultKey
    @abstract   Key to get the font feature selector default setting value.
    @discussion This key is used with a selector dictionary to get the default indicator for the selector. This value is a CFBooleanRef which if present and true indicates that this selector is the default setting for the current feature type.
}
var kCTFontFeatureSelectorDefaultKey: CFStringRef; external name '_kCTFontFeatureSelectorDefaultKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4
{!
    @defined    kCTFontFeatureSelectorSettingKey
    @abstract   Key to get or specify the current feature setting.
    @discussion This key is used with a selector dictionary to get or specify the current setting for the selector. This value is a CFBooleanRef to indicate whether this selector is on or off. If this key is not present, the default setting is used.
}
var kCTFontFeatureSelectorSettingKey: CFStringRef; external name '_kCTFontFeatureSelectorSettingKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyFeatures
    @abstract   Returns an array of font features

    @param      font
                The font reference.

    @result     This function returns an array of font feature dictionaries for the font reference.
}
function CTFontCopyFeatures( font: CTFontRef ): CFArrayRef; external name '_CTFontCopyFeatures';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyFeatureSettings
    @abstract   Returns an array of font feature setting tuples

    @discussion A setting tuple is a dictionary of a kCTFontFeatureTypeIdentifierKey key-value pair and a kCTFontFeatureSelectorIdentifierKey key-value pair. Each tuple corresponds to an enabled non-default setting. It is the caller's responsibility to handle exclusive and non-exclusive settings as necessary.

    @param      font
                The font reference.

    @result     This function returns a normalized array of font feature setting dictionaries. The array will only contain the non-default settings that should be applied to the font, or NULL if the default settings should be used.
}
function CTFontCopyFeatureSettings( font: CTFontRef ): CFArrayRef; external name '_CTFontCopyFeatureSettings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Conversion
}//--------------------------------------------------------------------------

{!
    @function   CTFontCopyGraphicsFont
    @abstract   Returns a CGFontRef and attributes.

    @param      font
                The font reference.

    @param      attributes
                A pointer to a CTFontDescriptorRef to receive a font descriptor containing additional attributes. Can be NULL. Must be released by caller.

    @result     This function returns a CGFontRef for the given font reference. Additional attributes from the font will be passed back as a font descriptor via the attributes parameter. The result must be released by the caller.
}
function CTFontCopyGraphicsFont( font: CTFontRef; attributes: CTFontDescriptorRefPtr { can be NULL } ): CGFontRef; external name '_CTFontCopyGraphicsFont';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreateWithGraphicsFont
    @abstract   Creates a new font reference from a CGFontRef.

    @param      graphicsFont
                A valid CGFontRef.

    @param      size
                The point size for the font reference. If 0.0 is specified, the default font size of 12.0 will be used.

    @param      matrix
                The transformation matrix for the font. If unspecified, the identity matrix will be used. Optional.

    @param      attributes
                A CTFontDescriptorRef containing additional attributes that should be matched. Optional.

    @result     This function returns a new font reference for an existing CGFontRef with the specified size, matrix, and additional attributes.
}
function CTFontCreateWithGraphicsFont( graphicsFont: CGFontRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null}; attributes: CTFontDescriptorRef ): CTFontRef; external name '_CTFontCreateWithGraphicsFont';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{$ifc TARGET_OS_MAC}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4
{!
    @function   CTFontGetPlatformFont
    @abstract   Returns the ATSFontRef and attributes.

    @param      font
                The font reference.

    @param      attributes
                A pointer to a CTFontDescriptorRef to receive a font descriptor containing additional attributes. Can be NULL. Must be released by caller.

    @result     This function returns a an ATSFontRef for the given font reference. Additional attributes from the font will be passed back as a font descriptor via the attributes parameter.
}
function CTFontGetPlatformFont( font: CTFontRef; attributes: CTFontDescriptorRefPtr {can be null} ): ATSFontRef; external name '_CTFontGetPlatformFont';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreateWithPlatformFont
    @abstract   Creates a new font reference from an ATSFontRef.

    @param      platformFont
                A valid ATSFontRef.

    @param      size
                The point size for the font reference. If 0.0 is specified, the default font size of 12.0 will be used.

    @param      matrix
                The transformation matrix for the font. If unspecified, the identity matrix will be used. Optional.

    @param      attributes
                A CTFontDescriptorRef containing additional attributes that should be matched. Optional.

    @result     This function returns a new font reference for an ATSFontRef with the specified size, matrix, and additional attributes.
}
function CTFontCreateWithPlatformFont( platformFont: ATSFontRef; size: CGFloat; matrix: {const} CGAffineTransformPtr {can be null}; attributes: CTFontDescriptorRef ): CTFontRef; external name '_CTFontCreateWithPlatformFont';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCreateWithQuickdrawInstance
    @abstract   Returns a font reference for the given Quickdraw instance.

    @discussion This function is provided for compatibility support between Core Text and clients needing to support Quickdraw font references.

    @param      name
                The Quickdraw font name. If NULL or zero length, an identifier must be specified instead.

    @param      identifier
                The Quickdraw font identifier. If 0, a name must be specified instead.

    @param      style
                The Quickdraw font style.

    @param      size
                The point size for the font reference. If 0.0 is specified, the default size of 12.0 is used.

    @result     This function returns the best font instance matching the Quickdraw instance information.
}
function CTFontCreateWithQuickdrawInstance( name: StringPtr {can be null}; identifier: SInt16; style: UInt8; size: CGFloat ): CTFontRef; external name '_CTFontCreateWithQuickdrawInstance';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_NA) *)
{$endc} { TARGET_OS_MAC }
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{! --------------------------------------------------------------------------
    @group Font Tables
}//--------------------------------------------------------------------------

const
	kCTFontTableBASE = FourCharCode('BASE');   // Baseline
	kCTFontTableCFF = FourCharCode('CFF ');   // PostScript font program
	kCTFontTableDSIG = FourCharCode('DSIG');   // Digital signature
	kCTFontTableEBDT = FourCharCode('EBDT');   // Embedded bitmap
	kCTFontTableEBLC = FourCharCode('EBLC');   // Embedded bitmap location
	kCTFontTableEBSC = FourCharCode('EBSC');   // Embedded bitmap scaling
	kCTFontTableGDEF = FourCharCode('GDEF');   // Glyph definition
	kCTFontTableGPOS = FourCharCode('GPOS');   // Glyph positioning
	kCTFontTableGSUB = FourCharCode('GSUB');   // Glyph substitution
	kCTFontTableJSTF = FourCharCode('JSTF');   // Justification
	kCTFontTableLTSH = FourCharCode('LTSH');   // Linear threshold
	kCTFontTableOS2 = FourCharCode('OS/2');   // OS/2 and Windows specific metrics
	kCTFontTablePCLT = FourCharCode('PCLT');   // PCL 5 data
	kCTFontTableVDMX = FourCharCode('VDMX');   // Vertical device metrics
	kCTFontTableVORG = FourCharCode('VORG');   // Vertical origin
	kCTFontTableZapf = FourCharCode('Zapf');   // Glyph reference
	kCTFontTableAcnt = FourCharCode('acnt');   // Accent attachment
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCTFontTableAnkr = FourCharCode('ankr');   // Anchor points
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
	kCTFontTableAvar = FourCharCode('avar');   // Axis variation
	kCTFontTableBdat = FourCharCode('bdat');   // Bitmap data
	kCTFontTableBhed = FourCharCode('bhed');   // Bitmap font header
	kCTFontTableBloc = FourCharCode('bloc');   // Bitmap location
	kCTFontTableBsln = FourCharCode('bsln');   // Baseline
	kCTFontTableCmap = FourCharCode('cmap');   // Character to glyph mapping
	kCTFontTableCvar = FourCharCode('cvar');   // CVT variation
	kCTFontTableCvt = FourCharCode('cvt ');   // Control value table
	kCTFontTableFdsc = FourCharCode('fdsc');   // Font descriptor
	kCTFontTableFeat = FourCharCode('feat');   // Layout feature
	kCTFontTableFmtx = FourCharCode('fmtx');   // Font metrics
	kCTFontTableFpgm = FourCharCode('fpgm');   // Font program
	kCTFontTableFvar = FourCharCode('fvar');   // Font variation
	kCTFontTableGasp = FourCharCode('gasp');   // Grid-fitting/Scan-conversion
	kCTFontTableGlyf = FourCharCode('glyf');   // Glyph data
	kCTFontTableGvar = FourCharCode('gvar');   // Glyph variation
	kCTFontTableHdmx = FourCharCode('hdmx');   // Horizontal device metrics
	kCTFontTableHead = FourCharCode('head');   // Font header
	kCTFontTableHhea = FourCharCode('hhea');   // Horizontal header
	kCTFontTableHmtx = FourCharCode('hmtx');   // Horizontal metrics
	kCTFontTableHsty = FourCharCode('hsty');   // Horizontal style
	kCTFontTableJust = FourCharCode('just');   // Justification
	kCTFontTableKern = FourCharCode('kern');   // Kerning
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCTFontTableKerx = FourCharCode('kerx');   // Extended kerning
	kCTFontTableLcar = FourCharCode('lcar');   // Ligature caret
	kCTFontTableLtag = FourCharCode('ltag');   // Language tags
=======
	kCTFontTableLcar = FourCharCode('lcar');   // Ligature caret
>>>>>>> graemeg/cpstrnew
=======
	kCTFontTableLcar = FourCharCode('lcar');   // Ligature caret
>>>>>>> graemeg/cpstrnew
=======
	kCTFontTableLcar = FourCharCode('lcar');   // Ligature caret
>>>>>>> graemeg/cpstrnew
=======
	kCTFontTableLcar = FourCharCode('lcar');   // Ligature caret
>>>>>>> origin/cpstrnew
=======
	kCTFontTableLcar = FourCharCode('lcar');   // Ligature caret
>>>>>>> origin/fixes_2.4
	kCTFontTableLoca = FourCharCode('loca');   // Index to location
	kCTFontTableMaxp = FourCharCode('maxp');   // Maximum profile
	kCTFontTableMort = FourCharCode('mort');   // Morph
	kCTFontTableMorx = FourCharCode('morx');   // Extended morph
	kCTFontTableName = FourCharCode('name');   // Naming table
	kCTFontTableOpbd = FourCharCode('opbd');   // Optical bounds
	kCTFontTablePost = FourCharCode('post');   // PostScript information
	kCTFontTablePrep = FourCharCode('prep');   // CVT program
	kCTFontTableProp = FourCharCode('prop');   // Properties
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCTFontTableSbit = FourCharCode('sbit');   // Bitmap data
	kCTFontTableSbix = FourCharCode('sbix');   // Extended bitmap data
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
	kCTFontTableTrak = FourCharCode('trak');   // Tracking
	kCTFontTableVhea = FourCharCode('vhea');   // Vertical header
	kCTFontTableVmtx = FourCharCode('vmtx');    // Vertical metrics
type
	CTFontTableTag = UInt32;

const
	kCTFontTableOptionNoOptions = 0;
	kCTFontTableOptionExcludeSynthetic = 1 shl 0;
type
	CTFontTableOptions = UInt32;

{!
    @function   CTFontCopyAvailableTables
    @abstract   Returns an array of font table tags.

    @param      font
                The font reference.

    @param      options

    @result     This function returns an array of CTFontTableTag values for the given font and the supplied options. The returned set will contain unboxed values, which may be extracted like so:
                <code>CTFontTableTag tag = (CTFontTableTag)(uintptr_t)CFArrayGetValueAtIndex(tags, index);</code>
}
function CTFontCopyAvailableTables( font: CTFontRef; options: CTFontTableOptions ): CFArrayRef; external name '_CTFontCopyAvailableTables';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCopyTable
    @abstract   Returns a reference to the font table data.

    @param      font
                The font reference.

    @param      table
                The font table identifier as a CTFontTableTag.

    @param      options

    @result     This function returns a retained reference to the font table data as CFDataRef. The table data is not actually copied, however the data reference must be released.
}
function CTFontCopyTable( font: CTFontRef; table: CTFontTableTag; options: CTFontTableOptions ): CFDataRef; external name '_CTFontCopyTable';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @function   CTFontDrawGlyphs
    @abstract   Renders the given glyphs from the CTFont at the given positions in the CGContext.
    @discussion This function will modify the CGContext's font, text size, and text matrix if specified in the CTFont. These attributes will not be restored.
        The given glyphs should be the result of proper Unicode text layout operations (such as CTLine). Results from CTFontGetGlyphsForCharacters (or similar APIs) do not perform any Unicode text layout.

    @param      font
                The font to render glyphs from. If the font has a size or matrix attribute, the CGContext will be set with these values.

    @param      glyphs
                The glyphs to be rendered. See above discussion of how the glyphs should be derived.

    @param      positions
                The positions (origins) for each glyph. The positions are in user space. The number of positions passed in must be equivalent to the number of glyphs.

    @param      count
                The number of glyphs to be rendered from the glyphs array.

    @param      context
                CGContext used to render the glyphs.

    @result     void
}
procedure CTFontDrawGlyphs( font: CTFontRef; {const} glyphs: {variable-size-array} CGGlyphPtr; {const} positions: {variable-size-array} CGPointPtr; count: size_t; context: CGContextRef ); external name '_CTFontDrawGlyphs';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_4_2) *)


{!
    @function   CTFontGetLigatureCaretPositions
    @abstract   Returns caret positions within a glyph.
    @discussion This function is used to obtain caret positions for a specific glyph.
                The return value is the max number of positions possible, and the function
                will populate the caller's positions buffer with available positions if possible.
                This function may not be able to produce positions if the font does not
                have the appropriate data, in which case it will return 0.
 
    @param      font
                The font reference.
    
    @param      glyph
                The glyph.
 
    @param      positions
                A buffer of at least maxPositions to receive the ligature caret positions for
                the glyph.
 
    @param      maxPositions
                The maximum number of positions to return.
 
    @result     Returns the number of caret positions for the specified glyph.
}
function CTFontGetLigatureCaretPositions( font: CTFontRef; glyph: CGGlyph; positions: {variable-size-array} CGFloatPtr; maxPositions: CFIndex ): CFIndex; external name '_CTFontGetLigatureCaretPositions';
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{! --------------------------------------------------------------------------
    @group Baseline Alignment
 }//--------------------------------------------------------------------------

{!
    @defined    kCTBaselineClassRoman

    @abstract   Key to reference the Roman baseline class.
 
    @discussion This key can be used with a baseline info dictionary to offset to the Roman baseline as a CFNumberRef float. It can also be used as the value for kCTBaselineClassAttributeName.
}
var kCTBaselineClassRoman: CFStringRef; external name '_kCTBaselineClassRoman'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)

{!
    @defined    kCTBaselineClassIdeographicCentered
 
    @abstract   Key to reference the Ideographic Centered baseline class.
 
    @discussion This key can be used with a baseline info dictionary to offset to the Ideographic Centered baseline as a CFNumberRef float. It can also be used as the value for kCTBaselineClassAttributeName.
}
var kCTBaselineClassIdeographicCentered: CFStringRef; external name '_kCTBaselineClassIdeographicCentered'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)
    
{!
    @defined    kCTBaselineClassIdeographicLow
 
    @abstract   Key to reference the Ideographic Low baseline class.
 
    @discussion This key can be used with a baseline info dictionary to offset to the Ideographic Low baseline as a CFNumberRef float. It can also be used as the value for kCTBaselineClassAttributeName.
}
var kCTBaselineClassIdeographicLow: CFStringRef; external name '_kCTBaselineClassIdeographicLow'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)
    
{!
    @defined    kCTBaselineClassIdeographicHigh

    @abstract   Key to reference the Ideographic High baseline class.

    @discussion This key can be used with a baseline info dictionary to offset to the Ideographic High baseline as a CFNumberRef float. It can also be used as the value for kCTBaselineClassAttributeName.
}
var kCTBaselineClassIdeographicHigh: CFStringRef; external name '_kCTBaselineClassIdeographicHigh'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)

{!
    @defined    kCTBaselineClassHanging
 
    @abstract   Key to reference the Hanging baseline class.
 
    @discussion This key can be used with a baseline info dictionary to offset to the Hanging baseline as a CFNumberRef float. It can also be used as the value for kCTBaselineClassAttributeName.
}
var kCTBaselineClassHanging: CFStringRef; external name '_kCTBaselineClassHanging'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)

{!
    @defined    kCTBaselineClassMathKey

    @abstract   Key to reference the Math baseline class.

    @discussion This key can be used with a baseline info dictionary to offset to the Math baseline as a CFNumberRef float. It can also be used as the value for kCTBaselineClassAttributeName.
}
var kCTBaselineClassMath: CFStringRef; external name '_kCTBaselineClassMath'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)

{!
    @defined    kCTBaselineReferenceFont
 
    @abstract   Key to reference a font for the reference baseline.
 
    @discussion This key can be used to specify a font for the reference baseline. The value is a CTFontRef or the kCTBaselineOriginalFont constant.
}
var kCTBaselineReferenceFont: CFStringRef; external name '_kCTBaselineReferenceFont'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)

{!
    @defined    kCTBaselineOriginalFont
    
    @abstract   Use the original font for setting the reference baseline.
 
    @discussion This constant can be used as the value for kCTBaselineReferenceFont to specify that the original font should be used for the reference baseline.
}
var kCTBaselineOriginalFont: CFStringRef; external name '_kCTBaselineOriginalFont'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)

{!
    @function   CTFontCopyDefaultCascadeListForLanguages
    @abstract   Return an ordered list of CTFontDescriptorRef's for font fallback derived from the system default fallback region according to the given language preferences. The style of the given is also matched as well as the weight and width of the font is not one of the system UI font, otherwise the UI font fallback is applied.

    @param      font
                The font reference.
 
    @param      languagePrefList
                The language preference list - ordered array of CFStringRef's of ISO language codes.

    @result     The ordered list of fallback fonts - ordered array of CTFontDescriptors.
}
function CTFontCopyDefaultCascadeListForLanguages( font: CTFontRef; languagePrefList: CFArrayRef ): CFArrayRef; external name '_CTFontCopyDefaultCascadeListForLanguages';
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
