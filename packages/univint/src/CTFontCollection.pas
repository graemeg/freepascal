{
 *  CTFontCollection.h
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

unit CTFontCollection;
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
>>>>>>> origin/fixes_2.4
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
uses MacTypes,CTFontDescriptor,CFBase,CFArray,CFDictionary,CFSet;
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
uses MacTypes,CTFontDescriptor,CFBase,CFArray,CFDictionary;
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
    @group Collection Types
}//--------------------------------------------------------------------------

{!
    @typedef    CTFontCollectionRef
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract   The Core Text font collection reference.
    @discussion An opaque reference to an immutable font collection.
}
type
	CTFontCollectionRef = ^__CTFontCollection; { an opaque type }
	__CTFontCollection = record end;
	CTFontCollectionRefPTr = ^CTFontCollectionRef;

{!
    @typedef    CTMutableFontCollectionRef
    @abstract   The Core Text mutable font collection reference.
    @discussion An opaque reference to a mutable font collection.
}
type
	CTMutableFontCollectionRef = ^__CTFontCollection;

{!
    @function   CTFontCollectionGetTypeID
    @abstract   Returns the type identifier for Core Text font collection references.
    @result     The identifier for the opaque types CTFontCollectionRef or CTMutableFontCollectionRef.
}
function CTFontCollectionGetTypeID: CFTypeID; external name '_CTFontCollectionGetTypeID';
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
    @abstract   The Core Text Font Collection reference.
    @discussion An opaque reference to a font collection.
}
type
	CTFontCollectionRef = ^SInt32; { an opaque type }
	CTFontCollectionRefPTr = ^CTFontCollectionRef;

{!
    @function   CTFontCollectionGetTypeID
    @abstract   Returns the type identifier for Core Text font collection references.
    @result     The identifier for the opaque type CTFontCollectionRef.
}
function CTFontCollectionGetTypeID: CFTypeID; external name '_CTFontCollectionGetTypeID';
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
    @typedef    CTFontCollectionSortDescriptorsCallback
    @abstract   Collection sorting callback.
    @discussion This callback can be specified to obtain the matching font descriptors of a collection in sorted order. Return the appropriate comparison result of first descriptor to second descriptor.
}
type
	CTFontCollectionSortDescriptorsCallback = function( first: CTFontDescriptorRef; second: CTFontDescriptorRef; refCon: UnivPtr ): CFComparisonResult;

{! --------------------------------------------------------------------------
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @group Collection Matching Options
=======
    @group Collection Constants
>>>>>>> graemeg/cpstrnew
=======
    @group Collection Constants
>>>>>>> graemeg/cpstrnew
=======
    @group Collection Constants
>>>>>>> graemeg/cpstrnew
=======
    @group Collection Constants
>>>>>>> origin/cpstrnew
=======
    @group Collection Constants
>>>>>>> origin/fixes_2.4
}//--------------------------------------------------------------------------

{!
    @defined    kCTFontCollectionRemoveDuplicatesOption
    @abstract   Option key to specify filtering of duplicates.
    @discussion Specify this option key in the options dictionary with a non- zero value to enable automatic filtering of duplicate font descriptors.
}
var kCTFontCollectionRemoveDuplicatesOption: CFStringRef; external name '_kCTFontCollectionRemoveDuplicatesOption'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{$ifc TARGET_OS_MAC}
{!
    @defined    kCTFontCollectionIncludeDisabledOption
    @abstract   Option key to include disabled fonts in the matching results.
    @discussion Specify this option key in the options dictionary with a non-zero value to enable matching of disabled fonts. You can pass font descriptors specifying disabled fonts to CTFontManagerEnableFontDescriptors, but you cannot use such a font descriptor to query font attributes from the system database or create a CTFontRef.
}
var kCTFontCollectionIncludeDisabledFontsOption: CFStringRef; external name '_kCTFontCollectionIncludeDisabledFontsOption'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)

{!
    @defined    kCTFontCollectionDisallowAutoActivationOption
    @abstract   Option key to avoid auto-activating fonts.
    @discussion Specify this option key in the options dictionary with a non-zero value to disallow searches for missing fonts (font descriptors returning no results).
}
var kCTFontCollectionDisallowAutoActivationOption: CFStringRef; external name '_kCTFontCollectionDisallowAutoActivationOption'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)
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
    @group Collection Creation
}//--------------------------------------------------------------------------

{!
    @function   CTFontCollectionCreateFromAvailableFonts
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @abstract   Returns a new font collection matching all available fonts.
=======
    @abstract   Returns a new font collection containing all available fonts.
>>>>>>> graemeg/cpstrnew
=======
    @abstract   Returns a new font collection containing all available fonts.
>>>>>>> graemeg/cpstrnew
=======
    @abstract   Returns a new font collection containing all available fonts.
>>>>>>> graemeg/cpstrnew
=======
    @abstract   Returns a new font collection containing all available fonts.
>>>>>>> origin/cpstrnew
=======
    @abstract   Returns a new font collection containing all available fonts.
>>>>>>> origin/fixes_2.4

    @param      options
                The options dictionary. See constant option keys.

    @result     This function creates a new collection containing all fonts available to the current application.
}
function CTFontCollectionCreateFromAvailableFonts( options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateFromAvailableFonts';
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
    @function   CTFontCollectionCreateWithFontDescriptors
    @abstract   Returns a new collection based on the array of font descriptors.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @param      queryDescriptors
                An array of font descriptors to use for matching. May be NULL, in which case the matching descriptors will be NULL.
=======
    @param      descriptors
                An array of font descriptors.
>>>>>>> graemeg/cpstrnew
=======
    @param      descriptors
                An array of font descriptors.
>>>>>>> graemeg/cpstrnew
=======
    @param      descriptors
                An array of font descriptors.
>>>>>>> graemeg/cpstrnew
=======
    @param      descriptors
                An array of font descriptors.
>>>>>>> origin/cpstrnew
=======
    @param      descriptors
                An array of font descriptors.
>>>>>>> origin/fixes_2.4

    @param      options
                The options dictionary. See constant option keys.

    @result     This function creates a new collection based on the provided font descriptors. The contents of this collection is defined by matching the provided descriptors against all available font descriptors.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CTFontCollectionCreateWithFontDescriptors( queryDescriptors: CFArrayRef; options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateWithFontDescriptors';
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
function CTFontCollectionCreateWithFontDescriptors( descriptors: CFArrayRef; options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateWithFontDescriptors';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTFontCollectionCreateWithFontDescriptors( descriptors: CFArrayRef; options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateWithFontDescriptors';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTFontCollectionCreateWithFontDescriptors( descriptors: CFArrayRef; options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateWithFontDescriptors';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTFontCollectionCreateWithFontDescriptors( descriptors: CFArrayRef; options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateWithFontDescriptors';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
function CTFontCollectionCreateWithFontDescriptors( descriptors: CFArrayRef; options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateWithFontDescriptors';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontCollectionCreateCopyWithFontDescriptors
    @abstract   Returns a copy of the original collection augmented with the new font descriptors.

    @param      original
                The original font collection reference.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @param      queryDescriptors
=======
    @param      descriptor
>>>>>>> graemeg/cpstrnew
=======
    @param      descriptor
>>>>>>> graemeg/cpstrnew
=======
    @param      descriptor
>>>>>>> graemeg/cpstrnew
=======
    @param      descriptor
>>>>>>> origin/cpstrnew
=======
    @param      descriptor
>>>>>>> origin/fixes_2.4
                An array of font descriptors to augment those of the original collection.

    @param      options
                The options dictionary. See constant option keys.

    @result     This function creates a copy of the original font collection augmented by the new font descriptors and options. The new font descriptors are merged with the existing descriptors to create a single set.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CTFontCollectionCreateCopyWithFontDescriptors( original: CTFontCollectionRef; queryDescriptors: CFArrayRef; options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateCopyWithFontDescriptors';
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{$ifc TARGET_OS_MAC}
{!
    @function   CTFontCollectionCreateMutableCopy
    @abstract   Returns a mutable copy of the original collection.

    @param      original
                The original font collection reference.

    @result     This function creates a mutable copy of the original font collection.
}
function CTFontCollectionCreateMutableCopy( original: CTFontCollectionRef ): CTMutableFontCollectionRef; external name '_CTFontCollectionCreateMutableCopy';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)

{! --------------------------------------------------------------------------
    @group Editing the query descriptors
}//--------------------------------------------------------------------------

{!
    @function   CTFontCollectionCopyQueryDescriptors
    @abstract   Returns the array of descriptors to match.

    @param      collection
                The font collection reference.

    @result     This function returns a retained reference to the array of descriptors to be used to query (match) the system font database. The return value is undefined if CTFontCollectionCreateFromAvailableFonts was used to create the collection.
}
function CTFontCollectionCopyQueryDescriptors( collection: CTFontCollectionRef ): CFArrayRef; external name '_CTFontCollectionCopyQueryDescriptors';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)

{!
    @function   CTFontCollectionSetQueryDescriptors
    @abstract   Replaces the array of descriptors to match.

    @param      collection
                The font collection reference.

    @param      descriptors
                An array of CTFontDescriptorRef. May be NULL to represent an empty collection, in which case the matching descriptors will also be NULL.
}
procedure CTFontCollectionSetQueryDescriptors( collection: CTMutableFontCollectionRef; descriptors: CFArrayRef ); external name '_CTFontCollectionSetQueryDescriptors';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)

{!
    @function   CTFontCollectionCopyExclusionDescriptors
    @abstract   Returns the array of descriptors to exclude from the match.

    @param      collection
                The font collection reference.

    @result     This function returns a retained reference to the array of descriptors to be used to query (match) the system font database.
}
function CTFontCollectionCopyExclusionDescriptors( collection: CTFontCollectionRef ): CFArrayRef; external name '_CTFontCollectionCopyExclusionDescriptors';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)

{!
    @function   CTFontCollectionSetExclusionDescriptors
    @abstract   Replaces the array of descriptors to exclude from the match.

    @param      collection
                The font collection reference.

    @param      descriptors
                An array of CTFontDescriptorRef. May be NULL.
}
procedure CTFontCollectionSetExclusionDescriptors( collection: CTMutableFontCollectionRef; descriptors: CFArrayRef ); external name '_CTFontCollectionSetExclusionDescriptors';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)
{$endc} { TARGET_OS_MAC }
{! --------------------------------------------------------------------------
    @group Retrieving Matching Descriptors
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
function CTFontCollectionCreateCopyWithFontDescriptors( original: CTFontCollectionRef; descriptors: CFArrayRef; options: CFDictionaryRef ): CTFontCollectionRef; external name '_CTFontCollectionCreateCopyWithFontDescriptors';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{! --------------------------------------------------------------------------
    @group Collection Access
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
    @function   CTFontCollectionCreateMatchingFontDescriptors
    @abstract   Returns an array of font descriptors matching the collection.

    @param      collection
                The font collection reference.

    @result     This function returns a retained reference to an array of normalized font descriptors   matching the collection definition.
}
function CTFontCollectionCreateMatchingFontDescriptors( collection: CTFontCollectionRef ): CFArrayRef; external name '_CTFontCollectionCreateMatchingFontDescriptors';
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
    @function   CTFontCollectionCreateMatchingFontDescriptorsSortedWithCallback
    @abstract   Returns the array of matching font descriptors sorted with the callback function.

    @param      collection
                The collection reference.

    @param      sortCallback
                The sorting callback function that defines the sort order.

    @param      refCon
                Pointer to client data define context for the callback.

    @result     This function returns an array of font descriptors matching the criteria of the collection and sorted by the results of the sorting callback function.
}
function CTFontCollectionCreateMatchingFontDescriptorsSortedWithCallback( collection: CTFontCollectionRef; sortCallback: CTFontCollectionSortDescriptorsCallback; refCon: UnivPtr ): CFArrayRef; external name '_CTFontCollectionCreateMatchingFontDescriptorsSortedWithCallback';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{$ifc TARGET_OS_MAC}
{!
    @function   CTFontCollectionCreateMatchingFontDescriptorsWithOptions
    @abstract   Returns an array of font descriptors matching the collection.

    @param      collection
                The font collection reference.

    @param      options
                The options dictionary. See constant option keys. May be NULL, in which case this call returns the same results as CTFontCollectionCreateMatchingFontDescriptors, using the options passed in when the collection was created.

    @result     This function returns a retained reference to an array of normalized font descriptors   matching the collection definition.
}
function CTFontCollectionCreateMatchingFontDescriptorsWithOptions( collection: CTFontCollectionRef; options: CFDictionaryRef ): CFArrayRef; external name '_CTFontCollectionCreateMatchingFontDescriptorsWithOptions';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)

{!
    @function   CTFontCollectionCreateMatchingFontDescriptorsForFamily
    @abstract   Returns an array of font descriptors matching the specified family, one descriptor for each style in the collection.

    @param      collection
                The font collection reference.

    @param      familyName
                The font family name

    @result     This function returns a retained reference to an array of normalized font descriptors matching the collection definition.
}
function CTFontCollectionCreateMatchingFontDescriptorsForFamily( collection: CTFontCollectionRef; familyName: CFStringRef; options: CFDictionaryRef ): CFArrayRef; external name '_CTFontCollectionCreateMatchingFontDescriptorsForFamily';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)

{! --------------------------------------------------------------------------
    @group Bulk attribute access
}//--------------------------------------------------------------------------
{!
    @enum       CTFontCollectionCopyOptions
    @abstract   Option bits for use with CTFontCollectionCopyFontAttribute(s).

    @constant   kCTFontCollectionCopyStandardSort
                Passing this option indicates that the return values should be sorted in standard UI order, suitable for display to the user. This is the same sorting behavior used by NSFontPanel and Font Book.

    @constant   kCTFontCollectionCopyUnique
                Passing this option indicates that duplicate values should be removed from the results.
 }
const
	kCTFontCollectionCopyDefaultOptions = 0;
	kCTFontCollectionCopyUnique = 1 shl 0;
	kCTFontCollectionCopyStandardSort = 1 shl 1;
type
	CTFontCollectionCopyOptions = UInt32;

{!
    @function   CTFontCollectionCopyFontAttribute
    @abstract   Returns an array of font descriptor attribute values.

    @param      collection
                The font collection reference.

    @param      attributeName
                The attribute to retrieve for each descriptor in the collection.

    @param      options
                Options to alter the return value.

    @result     This function returns a retained reference to a CFArray, or NULL on error. The caller is reponsible for releasing the array. The array contains one value for each descriptor, in the same order as the results from CTFontCollectionCreateMatchingDescriptors. When the kCTFontCollectionCopyUnique is set, duplicate values will be removed. When kCTFontCollectionCopyStandardSort is set, the values will be sorted in standard UI order.
 }
function CTFontCollectionCopyFontAttribute( collection: CTFontCollectionRef; attributeName: CFStringRef; options: CTFontCollectionCopyOptions ): CFArrayRef; external name '_CTFontCollectionCopyFontAttribute';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)

{!
    @function   CTFontCollectionCopyFontAttributes
    @abstract   Returns an array of dictionaries containing font descriptor attribute values.
 
    @param      collection
                The font collection reference.

    @param      attributeNames
                The attributes to retrieve for each descriptor in the collection.

    @param      options
                Options to alter the return value.

    @result     This function returns a retained reference to a CFArray, or NULL on error. The caller is reponsible for releasing the array. The array contains one value for each descriptor, in the same order as the results from CTFontCollectionCreateMatchingDescriptors. When the kCTFontCollectionCopyUnique is set, duplicate values will be removed. When kCTFontCollectionCopyStandardSort is set, the values will be sorted in standard UI order.
 }
function CTFontCollectionCopyFontAttributes( collection: CTFontCollectionRef; attributeNames: CFSetRef; options: CTFontCollectionCopyOptions ): CFArrayRef; external name '_CTFontCollectionCopyFontAttributes';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)
{$endc} { TARGET_OS_MAC }
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
