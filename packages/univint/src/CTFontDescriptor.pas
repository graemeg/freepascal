{
 *  CTFontDescriptor.h
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

unit CTFontDescriptor;
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
uses MacTypes,CTFontTraits,CFBase,CFArray,CFCharacterSet,CFData,CFDictionary,CFNumber,CFSet,CGBase,CGAffineTransforms;
{$endc} {not MACOSALLINCLUDE}


<<<<<<< HEAD
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
{$ifc TARGET_OS_MAC}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{!
    @header

    Thread Safety Information

    All functions in this header are thread safe unless otherwise specified.
}


{! --------------------------------------------------------------------------
    @group Descriptor Types
}//--------------------------------------------------------------------------

{!
    @typedef    CTFontDescriptorRef
    @abstract   The Core Text Font Descriptor reference.
    @discussion This is a opaque reference to a font descriptor.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CTFontDescriptorRef = ^__CTFontDescriptor; { an opaque type }
	__CTFontDescriptor = record end;
=======
	CTFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CTFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CTFontDescriptorRefPtr = ^CTFontDescriptorRef;

{!
    @function   CTFontDescriptorGetTypeID
    @abstract   Returns the type identifier for Core Text font descriptor
                references.
    @result     The identifier for the opaque type CTFontDescriptorRef.
}
function CTFontDescriptorGetTypeID: CFTypeID; external name '_CTFontDescriptorGetTypeID';
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
    @group Descriptor Constants
}//--------------------------------------------------------------------------

{!
    @defined    kCTFontURLAttribute
    @abstract   The font URL.
    @discussion This is the key for accessing the font URL from the font descriptor. The value associated with this key is a CFURLRef.
}
var kCTFontURLAttribute: CFStringRef; external name '_kCTFontURLAttribute'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)
{!
    @defined    kCTFontNameAttribute
    @abstract   The PostScript name.
    @discussion This is the key for retrieving the PostScript name from the font descriptor. When matching, this is treated more generically: the system first tries to find fonts with this PostScript name. If none is found, the system tries to find fonts with this family name, and, finally, if still nothing, tries to find fonts with this display name. The value associated with this key is a CFStringRef. If unspecified, defaults to "Helvetica", if unavailable falls back to global font cascade list.
}
var kCTFontNameAttribute: CFStringRef; external name '_kCTFontNameAttribute'; (* attribute const *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
{!
    @defined    kCTFontNameAttribute
    @abstract   The PostScript name.
    @discussion This is the key for accessing the PostScript name from the font descriptor. The value associated with this key is a CFStringRef. If unspecified, defaults to "Helvetica", if unavailable falls back to global font cascade list.
}
var kCTFontNameAttribute: CFStringRef; external name '_kCTFontNameAttribute'; (* attribute const *)
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
{!
    @defined    kCTFontDisplayNameAttribute
    @abstract   The display name.
    @discussion This is the key for accessing the name used to display the font. Most commonly this is the full name. The value associated with this key is a CFStringRef.
}
var kCTFontDisplayNameAttribute: CFStringRef; external name '_kCTFontDisplayNameAttribute'; (* attribute const *)
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
    @defined    kCTFontFamilyNameAttribute
    @abstract   The family name.
    @discussion This is the key for accessing the family name from the font descriptor. The value associated with this key is a CFStringRef.
}
var kCTFontFamilyNameAttribute: CFStringRef; external name '_kCTFontFamilyNameAttribute'; (* attribute const *)
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
    @defined    kCTFontStyleNameAttribute
    @abstract   The style name.
    @discussion This is the key for accessing the style name of the font. This name represents the designer's description of the font's style. The value associated with this key is a CFStringRef.
}
var kCTFontStyleNameAttribute: CFStringRef; external name '_kCTFontStyleNameAttribute'; (* attribute const *)
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
    @defined    kCTFontTraitsAttribute
    @abstract   The font traits dictionary.
    @discussion This is the key for accessing the dictionary of font traits for stylistic information. See CTFontTraits.h for the list of font traits. The value associated with this key is a CFDictionaryRef.
}
var kCTFontTraitsAttribute: CFStringRef; external name '_kCTFontTraitsAttribute'; (* attribute const *)
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
    @defined    kCTFontVariationAttribute
    @abstract   The font variation dictionary.
    @discussion This key is used to obtain the font variation instance as a CFDictionaryRef. If specified in a font descriptor, fonts with the specified axes will be primary match candidates, if no such fonts exist, this attribute will be ignored.
}
var kCTFontVariationAttribute: CFStringRef; external name '_kCTFontVariationAttribute'; (* attribute const *)
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
    @defined    kCTFontSizeAttribute
    @abstract   The font point size.
    @discussion This key is used to obtain or specify the font point size. Creating a font with this unspecified will default to a point size of 12.0. The value for this key is represented as a CFNumberRef.
}
var kCTFontSizeAttribute: CFStringRef; external name '_kCTFontSizeAttribute'; (* attribute const *)
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
    @defined    kCTFontMatrixAttribute
    @abstract   The font transformation matrix.
    @discussion This key is used to specify the font transformation matrix when creating a font. The default value is CGAffineTransformIdentity. The value for this key is a CFDataRef containing a CGAffineTransform, of which only the a, b, c, and d fields are used.
}
var kCTFontMatrixAttribute: CFStringRef; external name '_kCTFontMatrixAttribute'; (* attribute const *)
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
    @defined    kCTFontCascadeListAttribute
    @abstract   The font cascade list.
    @discussion This key is used to specify or obtain the cascade list used for a font reference. The cascade list is a CFArrayRef containing CTFontDescriptorRefs. If unspecified, the global cascade list is used.
}
var kCTFontCascadeListAttribute: CFStringRef; external name '_kCTFontCascadeListAttribute'; (* attribute const *)
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
    @defined    kCTFontCharacterSetAttribute
    @abstract   The font unicode character coverage set.
    @discussion This key is used to specify or obtain the character set for a font reference. This value for this key is a CFCharacterSetRef. If specified this can be used to restrict the font to a subset of its actual character set. If unspecified this attribute is ignored and the actual character set is used.
}
var kCTFontCharacterSetAttribute: CFStringRef; external name '_kCTFontCharacterSetAttribute'; (* attribute const *)
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
    @defined    kCTFontLanguagesAttribute
    @abstract   The list of supported languages.
    @discussion This key is used to specify or obtain a list of covered languages for a font reference. The value for this key is a CFArrayRef of CFStringRefs. If specified this restricts the search to matching fonts that support the specified languages. The language identifier string should conform to the BCP 47 standard. If unspecified this attribute is ignored.
}
var kCTFontLanguagesAttribute: CFStringRef; external name '_kCTFontLanguagesAttribute'; (* attribute const *)
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
    @defined    kCTFontBaselineAdjustAttribute
    @abstract   The baseline adjustment to apply to font metrics.
    @discussion This key is used to specify or obtain the baseline adjustment for a font reference. This is primary used when defining font descriptors for a cascade list to keep the baseline of all fonts even. The value associated with this is a float represented as a CFNumberRef.
}
var kCTFontBaselineAdjustAttribute: CFStringRef; external name '_kCTFontBaselineAdjustAttribute'; (* attribute const *)
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
    @defined    kCTFontMacintoshEncodingsAttribute
    @abstract   The macintosh encodings attribute.
    @discussion This key is used to specify or obtain the macintosh encodings for a font reference. The value associated with this key is a CFNumberRef containing a bitfield of the Macintosh encodings. This attribute is provided for legacy compatibility.
}
var kCTFontMacintoshEncodingsAttribute: CFStringRef; external name '_kCTFontMacintoshEncodingsAttribute'; (* attribute const *)
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
    @defined    kCTFontFeaturesAttribute
    @abstract   The array of font features.
    @discussion This key is used to specify or obtain the font features for a font reference. The value associated with this key is a CFArrayRef of font feature dictionaries. This features list contains the feature information from the 'feat' table of the font. See the CTFontCopyFeatures() API in   CTFont.h.
}
var kCTFontFeaturesAttribute: CFStringRef; external name '_kCTFontFeaturesAttribute'; (* attribute const *)
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
    @defined    kCTFontFeatureSettingsAttribute
    @abstract   The array of font settings.
    @discussion This key is used to specify or obtain the font features settings for a font reference. The value associated with this key is a CFArrayRef of font feature setting dictionaries. A setting dictionary contains a tuple of a kCTFontFeatureTypeIdentifierKey key-value pair and a kCTFontFeatureSelectorIdentifierKey key-value pair. Each setting dictionary indicates which setting should be turned on. In the case of duplicate or conflicting setting the last setting in the list will take precedence. It is the caller's responsibility to handle exclusive and non-exclusive settings as necessary.
}
var kCTFontFeatureSettingsAttribute: CFStringRef; external name '_kCTFontFeatureSettingsAttribute'; (* attribute const *)
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
    @defined    kCTFontFixedAdvanceAttribute
    @abstract   Specifies advance width.
    @discussion This key is used to specify a constant advance width, which affects the glyph metrics of any font instance created with this key; it overrides font values and the font transformation matrix, if any. The value associated with this key must be a CFNumberRef.
}
var kCTFontFixedAdvanceAttribute: CFStringRef; external name '_kCTFontFixedAdvanceAttribute'; (* attribute const *)
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
    @defined    kCTFontOrientationAttribute
    @abstract   The orientation attribute.
    @discussion This key is used to specify a particular orientation for the glyphs of the font. The value associated with this key is a int as a CFNumberRef. If you want to receive vertical metrics from a font for vertical rendering, specify kCTFontVerticalOrientation. If unspecified, the font will use its native orientation.
}
var kCTFontOrientationAttribute: CFStringRef; external name '_kCTFontOrientationAttribute'; (* attribute const *)
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
    @enum       CTFontOrientation
    @abstract   Specifies the intended rendering orientation of the font for obtaining glyph metrics.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCTFontOrientationDefault = 0;
	kCTFontOrientationHorizontal = 1;
	kCTFontOrientationVertical = 2;

    kCTFontDefaultOrientation = kCTFontOrientationDefault;
    kCTFontHorizontalOrientation = kCTFontOrientationHorizontal;
    kCTFontVerticalOrientation = kCTFontOrientationVertical;
=======
	kCTFontDefaultOrientation = 0;
	kCTFontHorizontalOrientation = 1;
	kCTFontVerticalOrientation = 2;
>>>>>>> graemeg/cpstrnew
=======
	kCTFontDefaultOrientation = 0;
	kCTFontHorizontalOrientation = 1;
	kCTFontVerticalOrientation = 2;
>>>>>>> graemeg/cpstrnew
=======
	kCTFontDefaultOrientation = 0;
	kCTFontHorizontalOrientation = 1;
	kCTFontVerticalOrientation = 2;
>>>>>>> graemeg/cpstrnew
=======
	kCTFontDefaultOrientation = 0;
	kCTFontHorizontalOrientation = 1;
	kCTFontVerticalOrientation = 2;
>>>>>>> origin/cpstrnew
=======
	kCTFontDefaultOrientation = 0;
	kCTFontHorizontalOrientation = 1;
	kCTFontVerticalOrientation = 2;
>>>>>>> origin/fixes_2.4
type
	CTFontOrientation = UInt32;

{!
    @defined    kCTFontFormatAttribute
    @abstract   Specifies the recognized format of the font.
    @discussion The attribute is used to specify or obtain the format of the font. The returned value is a CFNumber containing one of the constants defined below.
}
var kCTFontFormatAttribute: CFStringRef; external name '_kCTFontFormatAttribute'; (* attribute const *)
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
    @constant   kCTFontFormatUnrecognized
                The font is not a recognized format
    @constant   kCTFontFormatOpenTypePostScript
                The font is an OpenType format containing PostScript data
    @constant   kCTFontFormatOpenTypeTrueType
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                The font is an OpenType format containing TrueType data
=======
    			The font is an OpenType format containing TrueType data
>>>>>>> graemeg/cpstrnew
=======
    			The font is an OpenType format containing TrueType data
>>>>>>> graemeg/cpstrnew
=======
    			The font is an OpenType format containing TrueType data
>>>>>>> graemeg/cpstrnew
=======
    			The font is an OpenType format containing TrueType data
>>>>>>> origin/cpstrnew
=======
    			The font is an OpenType format containing TrueType data
>>>>>>> origin/fixes_2.4
    @constant   kCTFontFormatTrueType
                The font is a recognized TrueType format
    @constant   kCTFontFormatPostScript
                The font is a recognized PostScript format
    @constant   kCTFontFormatBitmap
                The font is a bitmap only format
}
const
	kCTFontFormatUnrecognized = 0;
	kCTFontFormatOpenTypePostScript = 1;
	kCTFontFormatOpenTypeTrueType = 2;
	kCTFontFormatTrueType = 3;
	kCTFontFormatPostScript = 4;
	kCTFontFormatBitmap = 5;
type
	CTFontFormat = UInt32;

{!
    @defined    kCTFontRegistrationScopeAttribute
    @abstract   Specifies the font descriptor's registration scope.
    @discussion The attribute is used to specify or obtain the font registration scope. The value returned is a CFNumberRef containing one of the CTFontManagerScope enumerated values. A value of NULL can be returned for font descriptors that are not registered.
}
var kCTFontRegistrationScopeAttribute: CFStringRef; external name '_kCTFontRegistrationScopeAttribute'; (* attribute const *)
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
    @defined    kCTFontPriorityAttribute
    @abstract   The font descriptors priority when resolving duplicates and sorting match results.
    @discussion This key is used to obtain or specify the font priority. The value returned is a CFNumberRef containing an integer value as defined below. The higher the value, the higher the priority of the font. Only registered fonts will have a priority. Unregistered font descriptors will return NULL.
}
var kCTFontPriorityAttribute: CFStringRef; external name '_kCTFontPriorityAttribute'; (* attribute const *)
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
    @constant   kCTFontPrioritySystem
                Priority of system fonts (located in /System/Library/Fonts).
    @constant   kCTFontPriorityNetwork
                Priority of network fonts (located in /Network/Library/Fonts).
    @constant   kCTFontPriorityComputer
                Priority of computer local fonts (located in /Library/Fonts).
    @constant   kCTFontPriorityUser
                Priority of local fonts (located in user's Library/Fonts).
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @constant   kCTFontPriorityDynamic
=======
    @constant   kCTFontPrioritySession
>>>>>>> graemeg/cpstrnew
=======
    @constant   kCTFontPrioritySession
>>>>>>> graemeg/cpstrnew
=======
    @constant   kCTFontPrioritySession
>>>>>>> graemeg/cpstrnew
=======
    @constant   kCTFontPrioritySession
>>>>>>> origin/cpstrnew
=======
    @constant   kCTFontPrioritySession
>>>>>>> origin/fixes_2.4
                Priority of fonts registered dynamically, not located in a standard location (either kCTFontManagerScopeUser, or kCTFontManagerScopeSession).
    @constant   kCTFontPriorityProcess
                Priority of fonts registered for the process (kCTFontManagerScopeProcess).
}
const
	kCTFontPrioritySystem = 10000;
	kCTFontPriorityNetwork = 20000;
	kCTFontPriorityComputer = 30000;
	kCTFontPriorityUser = 40000;
	kCTFontPriorityDynamic = 50000;
	kCTFontPriorityProcess = 60000;
type
	CTFontPriority = UInt32;

{!
    @defined    kCTFontEnabledAttribute
    @abstract   The font enabled state.
    @discussion This key is used to obtain the font state. The returned value is a CFNumberRef representing a boolean value. Unregistered font descriptors will return NULL, which is equivalent to false.
}
var kCTFontEnabledAttribute: CFStringRef; external name '_kCTFontEnabledAttribute'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)

{!
	@defined    kCTFontDownloadableAttribute
	@abstract   The font downloadable state.
	@discussion The value associated with this key is a CFBoolean.  If it is kCFBooleanTrue, CoreText attempts to download a font if necessary when matching a descriptor.
}
var kCTFontDownloadableAttribute: CFStringRef; external name '_kCTFontDownloadableAttribute'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)

{$ifc TARGET_OS_IPHONE}
{!
    @defined    kCTFontDownloadedAttribute
    @abstract   The download state.
    @discussion The value associated with this key is a CFBoolean.  If it is kCFBooleanTrue, corresponding FontAsset has been downloaded. (but still it may be necessary to call appropriate API in order to use the font in the FontAsset.)
}
var kCTFontDownloadedAttribute: CFStringRef; external name '_kCTFontDownloadedAttribute'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_NA, __IPHONE_7_0) *)
{$endc} {TARGET_OS_IPHONE}
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

{! --------------------------------------------------------------------------
    @group Descriptor Creation
}//--------------------------------------------------------------------------

{!
    @function   CTFontDescriptorCreateWithNameAndSize
    @abstract   Creates a new font descriptor with the provided PostScript name and size.

    @param      name
                The PostScript name to be used for the font descriptor as a CFStringRef.

    @param      size
                The point size. If 0.0, the kCTFontSizeAttribute will be omitted from the font descriptor.

    @result     This function creates a new font descriptor reference with the given PostScript name and point size.
}
function CTFontDescriptorCreateWithNameAndSize( name: CFStringRef; size: CGFloat ): CTFontDescriptorRef; external name '_CTFontDescriptorCreateWithNameAndSize';
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
    @function   CTFontDescriptorCreateWithAttributes
    @abstract   Creates a new font descriptor reference from a dictionary of attributes.

    @param      attributes
                A CFDictionaryRef of arbitrary attributes.

    @result     This function creates a new font descriptor with the attributes specified. This dictionary can contain arbitrary attributes that will be preserved, however unrecognized attributes will be ignored on font creation and and may not be preserved over the round trip (descriptor -> font -> descriptor).
}
function CTFontDescriptorCreateWithAttributes( attributes: CFDictionaryRef ): CTFontDescriptorRef; external name '_CTFontDescriptorCreateWithAttributes';
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
    @function   CTFontDescriptorCreateCopyWithAttributes
    @abstract   Creates a copy of the original font descriptor with new attributes.

    @param      original
                The original font descriptor reference.

    @param      attributes
                A CFDictionaryRef of arbitrary attributes.

    @result     This function creates a new copy of the original font descriptor with attributes augmented by those specified. If there are conflicts between attributes, the new attributes will replace existing ones, except for kCTFontVariationAttribute and kCTFontFeatureSettingsAttribute which will be merged.
}
function CTFontDescriptorCreateCopyWithAttributes( original: CTFontDescriptorRef; attributes: CFDictionaryRef ): CTFontDescriptorRef; external name '_CTFontDescriptorCreateCopyWithAttributes';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{!
    @function   CTFontCreateCopyWithFamily
    @abstract   Returns a new font descriptor in the specified family based on the traits of the original descriptor.

    @param      original
                The original font descriptor reference.

    @param		family
                The name of the desired family.

    @result     Returns a new font reference with the original traits in the given family, or NULL if none found in the system.
}
function CTFontDescriptorCreateCopyWithFamily( original: CTFontDescriptorRef; family: CFStringRef ): CTFontDescriptorRef; external name '_CTFontDescriptorCreateCopyWithFamily';
(* CT_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)

{!
    @function   CTFontDescriptorCreateCopyWithSymbolicTraits
    @abstract   Returns a new font descriptor based on the original descriptor having the specified symbolic traits.

    @param      original
                The original font descriptor reference.

    @param		symTraitValue
                The value of the symbolic traits. This bitfield is used to indicate the desired value for the traits specified by the symTraitMask parameter. Used in conjunction, they can allow for trait removal as well as addition.

    @param		symTraitMask
                The mask bits of the symbolic traits. This bitfield is used to indicate the traits that should be changed.

    @result     Returns a new font descriptor reference in the same family with the given symbolic traits, or NULL if none found in the system.
}
function CTFontDescriptorCreateCopyWithSymbolicTraits( original: CTFontDescriptorRef; symTraitValue: CTFontSymbolicTraits; symTraitMask: CTFontSymbolicTraits ): CTFontDescriptorRef; external name '_CTFontDescriptorCreateCopyWithSymbolicTraits';
(* CT_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)

{!
    @function   CTFontDescriptorCreateCopyWithVariation
    @abstract   Creates a copy of the original font descriptor with a new variation instance.
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

{!
    @function   CTFontDescriptorCreateCopyWithVariation
    @abstract   Creates a copy of the original font descriptor with a new  variation instance.
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

    @param      original
                The original font descriptor reference.

    @param      variationIdentifier
                The variation axis identifier. This is the four character code of the variation axis as a CFNumberRef.

    @param      variationValue
                The value corresponding with the variation instance.

    @result     This function returns a copy of the original font descriptor with a new variation instance. This is a convenience method for easily creating new variation font instances.
}
function CTFontDescriptorCreateCopyWithVariation( original: CTFontDescriptorRef; variationIdentifier: CFNumberRef; variationValue: CGFloat ): CTFontDescriptorRef; external name '_CTFontDescriptorCreateCopyWithVariation';
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
    @function   CTFontDescriptorCreateCopyWithFeature
    @abstract   Copies a font descriptor with new feature setting.

    @discussion This is a convenience method to more easily toggle the state of individual features.

    @param      original
                The original font descriptor reference.

    @param      featureTypeIdentifier
                The feature type identifier.

    @param      featureSelectorIdentifier
                The feature selector identifier.

    @result     A copy of the original font descriptor modified with the given feature settings.
}
function CTFontDescriptorCreateCopyWithFeature( original: CTFontDescriptorRef; featureTypeIdentifier: CFNumberRef; featureSelectorIdentifier: CFNumberRef ): CTFontDescriptorRef; external name '_CTFontDescriptorCreateCopyWithFeature';
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
    @function   CTFontDescriptorCreateMatchingFontDescriptors
    @abstract   Returns an array of font normalized font descriptors matching the provided descriptor.

    @param      descriptor
                The font descriptor reference.

    @param      mandatoryAttributes
                A set of attribute keys which are required to be identically matched in any returned font descriptors. Optional.

    @result     This function returns a retained array of normalized font descriptors matching the attributes present in descriptor. If descriptor itself is normalized then the array will contain only one item, the original descriptor.
}
function CTFontDescriptorCreateMatchingFontDescriptors( descriptor: CTFontDescriptorRef; mandatoryAttributes: CFSetRef ): CFArrayRef; external name '_CTFontDescriptorCreateMatchingFontDescriptors';
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
    @function   CTFontDescriptorCreateMatchingFontDescriptor
    @abstract   Returns an the single preferred matching font descriptor based on the original descriptor and system precedence.

    @param      descriptor
                The font descriptor reference.

    @param      mandatoryAttributes
                A set of attribute keys which are required to be identically matched in any returned font descriptors. Optional.

    @result     This function returns a retained normalized font descriptor matching the attributes present in descriptor. The original descriptor may be returned in normalized form.
}
function CTFontDescriptorCreateMatchingFontDescriptor( descriptor: CTFontDescriptorRef; mandatoryAttributes: CFSetRef ): CTFontDescriptorRef; external name '_CTFontDescriptorCreateMatchingFontDescriptor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{!
    Progress state
 }

type
  CTFontDescriptorMatchingState = SInt32;
const
    kCTFontDescriptorMatchingDidBegin = 0;              // called once at the beginning.
    kCTFontDescriptorMatchingDidFinish = 1;             // called once at the end.
    
    kCTFontDescriptorMatchingWillBeginQuerying = 2;     // called once before talking to the server.  Skipped if not necessary.
    kCTFontDescriptorMatchingStalled = 3;               // called when stalled. (e.g. while waiting for server response.)
    
    // Downloading and activating are repeated for each descriptor.
    kCTFontDescriptorMatchingWillBeginDownloading = 4;  // Downloading part may be skipped if all the assets are already downloaded
    kCTFontDescriptorMatchingDownloading = 5;
    kCTFontDescriptorMatchingDidFinishDownloading = 6;
    kCTFontDescriptorMatchingDidMatch = 7;              // called when font descriptor is matched.
    
    kCTFontDescriptorMatchingDidFailWithError = 8;       // called when an error occurred.  (may be called multiple times.)

{!
    keys for progressParameter dictionary.
 }

{ CTFontDescriptorRef; The current font descriptor.   Valid when state is kCTFontDescriptorMatchingDidMatch. }
var kCTFontDescriptorMatchingSourceDescriptor: CFStringRef; external name '_kCTFontDescriptorMatchingSourceDescriptor'; (* attribute const *)
(* CT_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)

{ CFArray; Array of descriptors to be queried.   Valid while downloading or when state is kCTFontDescriptorMatchingWillBeginQuerying. }
var kCTFontDescriptorMatchingDescriptors: CFStringRef; external name '_kCTFontDescriptorMatchingDescriptors'; (* attribute const *)
(* CT_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)

{ CFArray; Array of matched font descriptors.   Valid when state is kCTFontDescriptorMatchingDidMatch or CTFontDescriptorMatchingEnd. }
var kCTFontDescriptorMatchingResult: CFStringRef; external name '_kCTFontDescriptorMatchingResult'; (* attribute const *)
(* CT_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)

{ CFNumber; Progress in 0 - 100 for the current descriptor.   Valid during Downloading state. }
var kCTFontDescriptorMatchingPercentage: CFStringRef; external name '_kCTFontDescriptorMatchingPercentage'; (* attribute const *)
(* CT_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)

{ CFNumber; Byte size to download for the current descriptor.   Valid during Downloading state. }
var kCTFontDescriptorMatchingCurrentAssetSize: CFStringRef; external name '_kCTFontDescriptorMatchingCurrentAssetSize'; (* attribute const *)
(* CT_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)

{ CFNumber; Total downloaded byte size.   Valid during Downloading state. }
var kCTFontDescriptorMatchingTotalDownloadedSize: CFStringRef; external name '_kCTFontDescriptorMatchingTotalDownloadedSize'; (* attribute const *)
(* CT_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)

{ CFNumber; Total byte size to download.   Always valid, but may be Zero when information is not available. }
var kCTFontDescriptorMatchingTotalAssetSize: CFStringRef; external name '_kCTFontDescriptorMatchingTotalAssetSize'; (* attribute const *)
(* CT_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)

{ CFError; Valid when state kCTFontDescriptorMatchingDidFailWithError. }
var kCTFontDescriptorMatchingError: CFStringRef; external name '_kCTFontDescriptorMatchingError'; (* attribute const *)
(* CT_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)
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
    @group Descriptor Accessors
}//--------------------------------------------------------------------------

{!
    @function   CTFontDescriptorCopyAttributes
    @abstract   Returns the attributes dictionary of the font descriptor.

    @param      descriptor
                The font descriptor reference.

    @result     A retained reference to the font descriptor attributes dictionary. This dictionary will contain the minimum number of attributes to fully specify this particular font descriptor.
}
function CTFontDescriptorCopyAttributes( descriptor: CTFontDescriptorRef ): CFDictionaryRef; external name '_CTFontDescriptorCopyAttributes';
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
    @function   CTFontDescriptorCopyAttribute
    @abstract   Returns the value associated with an arbitrary attribute.

    @param      descriptor
                The font descriptor.

    @param      attribute
                The requested attribute.

    @result     A retained reference to the requested attribute, or NULL if the requested attribute is not present. Refer to the attribute definitions for documentation as to how each attribute is packaged as a CFType.
}
function CTFontDescriptorCopyAttribute( descriptor: CTFontDescriptorRef; attribute: CFStringRef ): CFTypeRef; external name '_CTFontDescriptorCopyAttribute';
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
    function    CTFontDescriptorCopyLocalizedAttribute
    @abstract   Returns a localized value for the requested attribute if available.

    @discussion This function returns a localized attribute based on the global language list. If localization is not possible for the attribute the behavior matches CTFontDescriptorCopyAttribute(). Generally, localization of attributes is only applicable to name attributes of a normalized font descriptor.

    @param      descriptor
                The font descriptor reference.

    @param      attribute
                The requested font attribute.

    @param      language
                If non-NULL, this will be receive a retained reference to the matched language. The language identifier will conform to the BCP 47 standard.

    @result     A retained reference to the requested attribute, or NULL if the requested attribute is not present. Refer to the attribute definitions for documentation as to how each attribute is packaged as a CFType.
}
function CTFontDescriptorCopyLocalizedAttribute( descriptor: CTFontDescriptorRef; attribute: CFStringRef; language: CFStringRefPtr {can be null} ): CFTypeRef; external name '_CTFontDescriptorCopyLocalizedAttribute';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
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
