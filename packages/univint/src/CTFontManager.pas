{
 *  CTFontManager.h
 *  CoreText
 *
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Copyright (c) 2008-2012 Apple Inc. All rights reserved.
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
 *  Copyright (c) 2008 Apple Inc. All rights reserved.
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

unit CTFontManager;
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
>>>>>>> origin/fixes_2.4
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
uses MacTypes,CTFontDescriptor,CTFontManagerErrors,CFBase,CFArray,CFData,CFError,CGFont;
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
uses MacTypes,CTFontDescriptor,CTFontManagerErrors,CFBase,CFArray,CFError;
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


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
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
{!
	@constant	CTRegisterBundleFonts
	@discussion If this key is defined in the application bundle info dictionary with a boolean value of true, CTFontManager will register all fonts in the Fonts subdirectory of the bundle's Resources directory in the process scope.  
 }

{!
	@function   CTFontManagerCopyAvailablePostScriptNames
	@abstract   Returns an array of unique PostScript font names.
 
	@result     This function returns a retained reference to a CFArray of CFString references, or NULL on error. The caller is responsible for releasing the array.
 }
function CTFontManagerCopyAvailablePostScriptNames: CFArrayRef; external name '_CTFontManagerCopyAvailablePostScriptNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
    @function   CTFontManagerCopyAvailableFontFamilyNames
    @abstract   Returns an array of visible font family names sorted for UI display.

    @result     This function returns a retained reference to a CFArray of CFString references, or NULL on error. The caller is responsible for releasing the array.
}
function CTFontManagerCopyAvailableFontFamilyNames: CFArrayRef; external name '_CTFontManagerCopyAvailableFontFamilyNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{!
    @function   CTFontManagerCopyAvailableFontURLs
    @abstract   Returns an array of font URLs.

    @result     This function returns a retained reference to a CFArray of CFURL references, or NULL on error. The caller is responsible for releasing the array.
}
function CTFontManagerCopyAvailableFontURLs: CFArrayRef; external name '_CTFontManagerCopyAvailableFontURLs';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
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
    @function   CTFontManagerCompareFontFamilyNames
    @abstract   A CFComparatorFunction to compare font family names and sort them according to Apple guidelines.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @discussion This function compares font family names and sorts them in the preferred order for display in user interfaces.
=======
    @discussion This function compares font family names and sorts them in the Apple preferred order, accounting for foundry prefix. Family names with recognized prefixes are sorted after the un-prefixed names in prefix order.
>>>>>>> graemeg/cpstrnew
=======
    @discussion This function compares font family names and sorts them in the Apple preferred order, accounting for foundry prefix. Family names with recognized prefixes are sorted after the un-prefixed names in prefix order.
>>>>>>> graemeg/cpstrnew
=======
    @discussion This function compares font family names and sorts them in the Apple preferred order, accounting for foundry prefix. Family names with recognized prefixes are sorted after the un-prefixed names in prefix order.
>>>>>>> graemeg/cpstrnew
=======
    @discussion This function compares font family names and sorts them in the Apple preferred order, accounting for foundry prefix. Family names with recognized prefixes are sorted after the un-prefixed names in prefix order.
>>>>>>> origin/cpstrnew
=======
    @discussion This function compares font family names and sorts them in the Apple preferred order, accounting for foundry prefix. Family names with recognized prefixes are sorted after the un-prefixed names in prefix order.
>>>>>>> origin/fixes_2.4
    @param      family1
                The first localized font family name, as CFStringRef.
    @param      family2
                The second localized font family name, as CFStringRef.
    @param      context
                Unused. Can be NULL.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @result     A CFComparisonResult value indicating the sort order for the two family names. kCFComparisonResultGreaterThan if family1 is greater than family2, kCFComparisonResultLessThan if family1 is less than family2, and kCFComparisonResultEqualTo if they are equal.
}
function CTFontManagerCompareFontFamilyNames( family1: {const} UnivPtr; family2: {const} UnivPtr; context: UnivPtr ): CFComparisonResult; external name '_CTFontManagerCompareFontFamilyNames';
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
    @result     A CFComparisonResult value indicating the sort order for the two family names. kCFComparisonResultGreatherThan if family1 is greater than family2, kCFComparisonResultLessThan if family1 is less than family2, and kCFComparisonResultEqualTo if they are equal.
}
function CTFontManagerCompareFontFamilyNames( family1: {const} UnivPtr; family2: {const} UnivPtr; context: UnivPtr ): CFComparisonResult; external name '_CTFontManagerCompareFontFamilyNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
    @function   CTFontManagerCreateFontDescriptorsFromURL
    @abstract   Returns an array of font descriptors representing each of the fonts in the specified URL.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                Note: these font descriptors are not available through font descriptor matching.
=======
				Note: these font descriptors are not availabe through font descriptor matching.
>>>>>>> graemeg/cpstrnew
=======
				Note: these font descriptors are not availabe through font descriptor matching.
>>>>>>> graemeg/cpstrnew
=======
				Note: these font descriptors are not availabe through font descriptor matching.
>>>>>>> graemeg/cpstrnew
=======
				Note: these font descriptors are not availabe through font descriptor matching.
>>>>>>> origin/cpstrnew
=======
				Note: these font descriptors are not availabe through font descriptor matching.
>>>>>>> origin/fixes_2.4

    @param      fileURL
                A file system URL referencing a valid font file.

    @result     This function returns a retained reference to a CFArray, or NULL on error. The caller is responsible for releasing the array.
}
function CTFontManagerCreateFontDescriptorsFromURL( fileURL: CFURLRef ): CFArrayRef; external name '_CTFontManagerCreateFontDescriptorsFromURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)

{!
    @function   CTFontManagerCreateFontDescriptorFromData
    @abstract   Returns a font descriptor representing the font in the supplied data.
                Note: the font descriptor is not available through font descriptor matching.

    @param      data
                A CFData containing font data.

    @result     A font descriptor created from the data, or NULL on error.
}
function CTFontManagerCreateFontDescriptorFromData( data: CFDataRef ): CTFontDescriptorRef; external name '_CTFontManagerCreateFontDescriptorFromData';
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_NA) *)
{$endc} { TARGET_OS_MAC }
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
    @enum       CTFontManagerScope
    @abstract   Scope for font registration.
    @constant   kCTFontManagerScopeProcess
                The font is available to the current process for the duration of the process unless directly unregistered.
    @constant   kCTFontManagerScopeUser
                The font is available to all processes for the current user session and will be available in subsequent sessions unless unregistered.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                User scope is unsupported in iOS.
    @constant   kCTFontManagerScopeSession
                The font is available to the current user session, and will not be available in subsequent sessions.
                User scope is unsupported in iOS.
=======
    @constant   kCTFontManagerScopeSession
                The font is available to the current user session, and will not be available in subsequent sessions.
>>>>>>> graemeg/cpstrnew
=======
    @constant   kCTFontManagerScopeSession
                The font is available to the current user session, and will not be available in subsequent sessions.
>>>>>>> graemeg/cpstrnew
=======
    @constant   kCTFontManagerScopeSession
                The font is available to the current user session, and will not be available in subsequent sessions.
>>>>>>> graemeg/cpstrnew
=======
    @constant   kCTFontManagerScopeSession
                The font is available to the current user session, and will not be available in subsequent sessions.
>>>>>>> origin/cpstrnew
=======
    @constant   kCTFontManagerScopeSession
                The font is available to the current user session, and will not be available in subsequent sessions.
>>>>>>> origin/fixes_2.4
}
const
	kCTFontManagerScopeNone = 0;
	kCTFontManagerScopeProcess = 1;
	kCTFontManagerScopeUser = 2;
	kCTFontManagerScopeSession = 3;
type
	CTFontManagerScope = UInt32;

{!
    @function   CTFontManagerRegisterFontsForURL
    @abstract   Registers fonts from the specified font URL with the font manager. Registered fonts are discoverable through font descriptor matching.

    @param      fontURL
                Font URL.

    @param      scope
                Scope constant defining the availability and lifetime of the registration. See scope constants for more details.

    @param      error
                Pointer to receive CFError in the case of failed registration.

    @result     Returns true if registration of the fonts was successful.
}
function CTFontManagerRegisterFontsForURL( fontURL: CFURLRef; scope: CTFontManagerScope; var error: CFErrorRef ): CBool; external name '_CTFontManagerRegisterFontsForURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_4_1) *)
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
    @function   CTFontManagerUnregisterFontsForURL
    @abstract   Unregisters fonts from the specified font URL with the font manager. Unregistered fonts are no longer discoverable through font descriptor matching.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                iOS note: only fonts registered with CTFontManagerRegisterFontsForURL or CTFontManagerRegisterFontsForURLs can be unregistered with this API.
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

    @param      fontURL
                Font URL.

    @param      scope
                Scope constant defining the availability and lifetime of the registration. Should match the scope the fonts are registered in. See scope constants for more details.

    @param      error
                Pointer to receive CFError in the case of failed unregistration.

    @result     Returns true if unregistration of the fonts was successful.

}
function CTFontManagerUnregisterFontsForURL( fontURL: CFURLRef; scope: CTFontManagerScope; var error: CFErrorRef ): CBool; external name '_CTFontManagerUnregisterFontsForURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_4_1) *)

{!
    @function   CTFontManagerRegisterGraphicsFont
    @abstract   Registers the specified graphics font with the font manager. Registered fonts are discoverable through font descriptor matching.
                Attempts to register a font that is either already registered or contains the same Postscript of an already registered font will fail.
                This functionality is useful for fonts that may be embedded in documents or present/constructed in memory. A graphics font is obtained
                by calling CGFontCreateWithDataProvider. Fonts that are backed by files should be registered using CTFontManagerRegisterFontsForURL.
 
    @param      font
                Graphics font to be registered.
 
    @param      error
                Pointer to receive CFError in the case of failed registration.
 
    @result     Returns true if registration of the fonts was successful.
}
function CTFontManagerRegisterGraphicsFont( font: CGFontRef; var error: CFErrorRef ): CBool; external name '_CTFontManagerRegisterGraphicsFont';
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_4_1) *)
    
{!
    @function   CTFontManagerUnregisterGraphicsFont
    @abstract   Unregisters the specified graphics font with the font manager. Unregistered fonts are no longer discoverable through font descriptor matching.
 
    @param      font
                Graphics font to be unregistered.
 
    @param      error
                Pointer to receive CFError in the case of failed unregistration.
 
    @result     Returns true if unregistration of the font was successful.
}
function CTFontManagerUnregisterGraphicsFont( font: CGFontRef; var error: CFErrorRef ): CBool; external name '_CTFontManagerUnregisterGraphicsFont';
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_4_1) *)
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
    @function   CTFontManagerRegisterFontsForURLs
    @abstract   Registers fonts from the specified font URLs with the font manager. Registered fonts are discoverable through font descriptor matching.

    @param      fontURLs
                Array of font URLs.

    @param      scope
                Scope constant defining the availability and lifetime of the registration. See scope constants for more details.

    @param      errors
                Pointer to CFArrayRef to receive array of CFError references. Each error will contain a CFArray of font URLs corresponding to kCTFontManagerErrorFontURLsKey. These URLs represent the font files that caused the error, and were not successfully registered. Must be released by caller. Can be NULL.

    @result     Returns true if registration of all font URLs was successful. Otherwise false.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CTFontManagerRegisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; var errors: CFArrayRef ): CBool; external name '_CTFontManagerRegisterFontsForURLs';
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_4_1) *)
=======
function CTFontManagerRegisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerRegisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTFontManagerRegisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerRegisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTFontManagerRegisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerRegisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTFontManagerRegisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerRegisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
function CTFontManagerRegisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerRegisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> origin/fixes_2.4

{!
    @function   CTFontManagerUnregisterFontsForURLs
    @abstract   Unregisters fonts from the specified font URLs with the font manager. Unregistered fonts are no longer discoverable through font descriptor matching.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                iOS note: only fonts registered with CTFontManagerRegisterFontsForURL or CTFontManagerRegisterFontsForURLs can be unregistered with this API.
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

    @param      fontURLs
                Array of font URLs.

    @param      scope
                Scope constant defining the availability and lifetime of the registration. Should match the scope the fonts are registered in. See scope constants for more details.

    @param      errors
                Pointer to CFArrayRef to receive array of CFError references. Each error will contain a CFArray of font URLs corresponding to kCTFontManagerErrorFontURLsKey. These URLs represent the font files that caused the error, and were not successfully unregistered. Must be released by caller. Can be NULL.

    @result     Returns true if unregistration of all font URLs was successful. Otherwise false.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CTFontManagerUnregisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; var errors: CFArrayRef ): CBool; external name '_CTFontManagerUnregisterFontsForURLs';
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_4_1) *)

{$ifc TARGET_OS_MAC}
=======
function CTFontManagerUnregisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerUnregisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function CTFontManagerUnregisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerUnregisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function CTFontManagerUnregisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerUnregisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function CTFontManagerUnregisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerUnregisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
function CTFontManagerUnregisterFontsForURLs( fontURLs: CFArrayRef; scope: CTFontManagerScope; errors: CFArrayRefPtr {can be null} ): CBool; external name '_CTFontManagerUnregisterFontsForURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

>>>>>>> origin/fixes_2.4
{!
    @function   CTFontManagerEnableFontDescriptors
    @abstract   Enables or disables the matching font descriptors for font descriptor matching.

    @param      descriptors
                Array of font descriptors.

    @param      enable
                Boolean value indicating whether the fonts matching descriptors should be enabled for font descriptor matching.
}
procedure CTFontManagerEnableFontDescriptors( descriptors: CFArrayRef; enable: CBool ); external name '_CTFontManagerEnableFontDescriptors';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
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
    @function   CTFontManagerGetScopeForURL
    @abstract   Returns the registration scope of the specified URL.

    @param      fontURL
                Font URL.

    @result     Returns the registration scope of the specified URL, will return kCTFontManagerScopeNone if not currently registered.
}
function CTFontManagerGetScopeForURL( fontURL: CFURLRef ): CTFontManagerScope; external name '_CTFontManagerGetScopeForURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
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
    @function   CTFontManagerIsSupportedFontFile
    @abstract   Determines whether the referenced font data (usually by file URL) is supported on the current platform.

    @param      fontURL
                A URL to font data.

    @result     This function returns true if the URL represents a valid font that can be used on the current platform.
}
function CTFontManagerIsSupportedFont( fontURL: CFURLRef ): CBool; external name '_CTFontManagerIsSupportedFont';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
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
    @group Manager Auto-Activation
}//--------------------------------------------------------------------------


{!
    @const      kCTFontManagerBundleIdentifier
    @abstract   CTFontManage bundle identifier
    @discussion The CTFontManager bundle identifier to be used with get or set global auto-activation settings.
}
var kCTFontManagerBundleIdentifier: CFStringRef; external name '_kCTFontManagerBundleIdentifier'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
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
    @enum
    @abstract   Auto-activation settings.
    @constant   kCTFontManagerAutoActivationDefault
                Default auto-activation setting. When specified, the application will use the global setting.
    @constant   kCTFontManagerAutoActivationDisabled
                Disables auto-activation.
    @constant   kCTFontManagerAutoActivationEnabled
                Enables auto-activation.
    @constant   kCTFontManagerAutoActivationPromptUser
                Requires user input for auto-activation. A dialog will be presented to the user to confirm auto
                activation of the font.
}
const
	kCTFontManagerAutoActivationDefault = 0;
	kCTFontManagerAutoActivationDisabled = 1;
	kCTFontManagerAutoActivationEnabled = 2;
	kCTFontManagerAutoActivationPromptUser = 3;
type
	CTFontManagerAutoActivationSetting = UInt32;

{!
    @function   CTFontManagerSetAutoActivationSetting
    @abstract   Sets the auto-activation for the specified bundle identifier.
    @param      bundleIdentifier
                The bundle identifier. Used to specify a particular application bundle. If NULL,
                the current application bundle will be used. If kCTFontManagerBundleIdentifier is specified,
                will set the global auto-activation settings.
    @param      setting
                The new setting.
    @result     Function will apply the setting to the appropriate preferences location.
}
procedure CTFontManagerSetAutoActivationSetting( bundleIdentifier: CFStringRef; setting: CTFontManagerAutoActivationSetting ); external name '_CTFontManagerSetAutoActivationSetting';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
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
    @function   CTFontManagerGetAutoActivationSetting
    @abstract   Accessor for the auto-activation setting.
    @param      bundleIdentifier
                The bundle identifier. Used to specify a particular application bundle. If NULL,
                the current application bundle will be used. If kCTFontManagerBundleIdentifier is specified,
                will set the global auto-activation settings.
    @result     Will return the auto-activation setting for specified bundle identifier.
}
function CTFontManagerGetAutoActivationSetting( bundleIdentifier: CFStringRef ): CTFontManagerAutoActivationSetting; external name '_CTFontManagerGetAutoActivationSetting';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
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
    @group Manager Notifications
}//--------------------------------------------------------------------------

{!
    @constant   kCTFontManagerRegisteredFontsChangedNotification
    @abstract   Notification name for font registry changes.
    @discussion This is the string to use as the notification name when subscribing
                to CTFontManager notifications. This notification will be posted
                when fonts are added to the font registry. The client is responsible for
                registered with the distributed notification center to receive notifications
                for changes to the session or user scopes, and with a local notification
                for changes to the process scope.
}
var kCTFontManagerRegisteredFontsChangedNotification: CFStringRef; external name '_kCTFontManagerRegisteredFontsChangedNotification'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_NA) *)
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

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
