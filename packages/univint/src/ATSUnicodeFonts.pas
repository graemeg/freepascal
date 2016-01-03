{
     File:       QD/ATSUnicodeFonts.h
 
     Contains:   ATSUI font handling functions.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    Quickdraw-285~150
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 2003-2008 by Apple Inc. all rights reserved.
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    Quickdraw-262~1
 
     Copyright:  © 2003-2008 by Apple Inc. all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
{  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
                     http://www.freepascal.org/bugs.html
 
}
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit ATSUnicodeFonts;
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> graemeg/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
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
uses MacTypes,ATSUnicodeTypes,SFNTTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

<<<<<<< HEAD
=======
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ ---------------------------------------------------------------------------- }
{  Font features                                                               }
{ ---------------------------------------------------------------------------- }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{
 *  ATSUSetFontFeatures()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithFeature,
 *    CTFontDescriptorCreateWithAttributes instead.
=======
{
 *  ATSUSetFontFeatures()
>>>>>>> graemeg/fixes_2_2
=======
{
 *  ATSUSetFontFeatures()
>>>>>>> origin/fixes_2_2
=======

{
 *  ATSUSetFontFeatures()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithFeature,
 *    CTFontDescriptorCreateWithAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets font features in a style object.
 *  
 *  Discussion:
 *    This function enables you to set multiple font features for a
 *    style object. Any unset font features retain their font-defined
 *    default values. To set style attributes and font variations for a
 *    style object, call the functions ATSUSetAttributes and
 *    ATSUSetVariations, respectively. The constants that represent
 *    font feature types are defined in the header file
 *    SFNTLayoutTypes.h. When you use ATSUI to access and set font
 *    features, you must use the constants defined in this header file,
 *    which are described in "Inside Mac OS X: Rendering Unicode Text
 *    With ATSUI". As feature types can be added at any time, you
 *    should check Apple's font feature registry website for the most
 *    up-to-date list of font feature types and selectors:
 *    http://developer.apple.com/fonts/Registry/index.html.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style object for which to set font features.
 *    
 *    iFeatureCount:
 *      The number of font features to set. This value should
 *      correspond to the number of elements in the iType and iSelector
 *      arrays.
 *    
 *    iType:
 *      An array of feature types. Each element in the array must
 *      contain a valid feature type that corresponds to a feature
 *      selector in the iSelector array. To obtain the valid feature
 *      types for a font, call the function ATSUGetFontFeatureTypes .
 *    
 *    iSelector:
 *      An array of feature selectors. Each element in the array must
 *      contain a valid feature selector that corresponds to a feature
 *      type in the iType array. To obtain the valid feature selectors
 *      for a font, call the function ATSUGetFontFeatureSelectors .
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUSetFontFeatures( iStyle: ATSUStyle; iFeatureCount: ItemCount; {const} iType: {variable-size-array} ATSUFontFeatureTypePtr; {const} iSelector: {variable-size-array} ATSUFontFeatureSelectorPtr ): OSStatus; external name '_ATSUSetFontFeatures';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
 *  ATSUGetFontFeature()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUSetFontFeatures( iStyle: ATSUStyle; iFeatureCount: ItemCount; {const} iType: {variable-size-array} ATSUFontFeatureTypePtr; {const} iSelector: {variable-size-array} ATSUFontFeatureSelectorPtr ): OSStatus; external name '_ATSUSetFontFeatures';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUGetFontFeature()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontFeature()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the font feature corresponding to an index into an array
 *    of font features for a style object.
 *  
 *  Discussion:
 *    You might typically call ATSUGetFontFeature if you need to obtain
 *    one previously set feature after another within your program's
 *    processing loop. To obtain all previously set font features for a
 *    given style object, you can call the function
 *    ATSUGetAllFontFeatures. Before calling ATSUGetFontFeature, you
 *    should call the function ATSUGetAllFontFeatures to obtain a count
 *    of the font features that are set in the style object. You can
 *    then pass the index for the feature whose setting you want to
 *    obtain in the iTag and iMaximumValueSize parameters of
 *    ATSUGetFontFeature.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style you wish to obtain font feature information for.
 *    
 *    iFeatureIndex:
 *      An index into the array of font features for the style object.
 *      This index identifies the font feature to examine. Because this
 *      index is zero-based, you must pass a value between 0 and one
 *      less than the value produced in the oActualFeatureCount
 *      parameter of the function ATSUGetAllFontFeatures.
 *    
 *    oFeatureType:
 *      On return, the value identifies the font feature type
 *      corresponding to the index passed in the iFeatureIndex
 *      parameter. You must allocate space for ATSUGetFontFeature to
 *      store this value.
 *    
 *    oFeatureSelector:
 *      On return, the value identifies the font feature selector that
 *      corresponds to the feature type produced in the oFeatureType
 *      parameter. ou must allocate space for ATSUGetFontFeature to
 *      store this value.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontFeature( iStyle: ATSUStyle; iFeatureIndex: ItemCount; oFeatureType: ATSUFontFeatureTypePtr; oFeatureSelector: ATSUFontFeatureSelectorPtr ): OSStatus; external name '_ATSUGetFontFeature';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetAllFontFeatures()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontFeature( iStyle: ATSUStyle; iFeatureIndex: ItemCount; oFeatureType: ATSUFontFeatureTypePtr; oFeatureSelector: ATSUFontFeatureSelectorPtr ): OSStatus; external name '_ATSUGetFontFeature';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetAllFontFeatures()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetAllFontFeatures()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the font features of a style object that are not at
 *    default settings.
 *  
 *  Discussion:
 *    The ATSUGetAllFontFeatures function obtains all of a style
 *    object's font features that are not at default settings. Font
 *    features are grouped into categories called feature types, within
 *    which individual feature selectors define particular feature
 *    settings. The arrays produced by ATSUGetAllFontFeatures contain
 *    constants identifying the object's font types and their
 *    corresponding font selectors. Typically you use the function
 *    ATSUGetAllFontFeatures by calling it twice, as follows: (1) Pass
 *    a reference to the style object to examine in the iStyle
 *    parameter, a valid pointer to an ItemCount value in the
 *    oActualFeatureCount parameter, NULL for the oFeatureType and
 *    oFeatureSelector parameters, and 0 for the iMaximumFeatureCount
 *    parameter. ATSUGetAllFontFeatures returns the size in the
 *    oActualFeatureCount parameter to use for the feature type and
 *    selector arrays. (2) Allocate enough space for arrays of the
 *    returned size, then call ATSUGetAllFontFeatures again, passing a
 *    pointer to the arrays in the oFeatureType and oFeatureSelector
 *    parameters. On return, the arrays contain the font feature types
 *    and selectors, respectively, for the style object.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style for which you wish to obtain font feature information.
 *    
 *    iMaximumFeatureCount:
 *      The maximum number of feature types and selectors to obtain for
 *      the style object. Typically, this is equivalent to the number
 *      of ATSUFontFeatureType and ATSUFontFeatureSelector values for
 *      which you have allocated memory in the oFeatureType and
 *      oFeatureSelector parameters, respectively. To determine this
 *      value, see the Discussion.
 *    
 *    oFeatureType:
 *      On return, the array contains constants identifying each type
 *      of font feature that is at a nondefault setting in the style
 *      object. If you are uncertain of how much memory to allocate for
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      this array, see the Discussion.
=======
 *      this array, see the Discussion. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      this array, see the Discussion. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      this array, see the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    oFeatureSelector:
 *      On return, the array contains constants identifying the feature
 *      selectors that are at nondefault settings in the style object.
 *      Each selector determines the setting for a corresponding
 *      feature type produced in the oFeatureType parameter. If you are
 *      uncertain of how much memory to allocate for this array, see
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      the Discussion.
=======
 *      the Discussion.  can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      the Discussion.  can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    oActualFeatureCount:
 *      On return, the value specifies the actual number of font
 *      feature types and selectors in the style object. This may be
 *      greater than the value you specified in the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      iMaximumFeatureCount parameter.
=======
 *      iMaximumFeatureCount parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      iMaximumFeatureCount parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      iMaximumFeatureCount parameter.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetAllFontFeatures( iStyle: ATSUStyle; iMaximumFeatureCount: ItemCount; oFeatureType: {variable-size-array} ATSUFontFeatureTypePtr { can be NULL }; oFeatureSelector: {variable-size-array} ATSUFontFeatureSelectorPtr { can be NULL }; oActualFeatureCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAllFontFeatures';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUClearFontFeatures()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetAllFontFeatures( iStyle: ATSUStyle; iMaximumFeatureCount: ItemCount; oFeatureType: {variable-size-array} ATSUFontFeatureTypePtr { can be NULL }; oFeatureSelector: {variable-size-array} ATSUFontFeatureSelectorPtr { can be NULL }; oActualFeatureCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAllFontFeatures';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUClearFontFeatures()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUClearFontFeatures()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Restores default settings to the specified font features of a
 *    style object.
 *  
 *  Discussion:
 *    This function removes those font features that are identified by
 *    the feature selector and type constants in the iSelector and
 *    iType arrays and replaces them with their font-defined default
 *    values. Note that if you pass ATSUClearFontFeatures a font
 *    feature and selector that are already at default settings, the
 *    function does not return an error. To restore default font
 *    variations to a style object, call the function
 *    ATSUClearFontVariations. To restore default style attributes to a
 *    style object, call ATSUClearAttributes. To restore all default
 *    settings to a style object (for font features, variations, and
 *    style attributes), call the function ATSUClearStyle.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      A style whose font features you wish to clear.
 *    
 *    iFeatureCount:
 *      The number of font features to restore to default settings.
 *      This value should correspond to the number of elements in the
 *      iType and iSelector arrays. To restore default settings to all
 *      the font features in the specified style object, pass the
 *      constant kATSUClearAll in this parameter. In this case, the
 *      values in the iType and iSelector parameters are ignored.
 *    
 *    iType:
 *      An array of feature types. Each value should identify a font
 *      feature to restore to its default setting. To obtain all
 *      previously set font features for a given style object, you can
 *      call the function ATSUGetAllFontFeatures. You may pass NULL for
 *      this parameter if you are passing kATSUClearAll for the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      iFeatureCount parameter.
=======
 *      iFeatureCount parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      iFeatureCount parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      iFeatureCount parameter.
>>>>>>> origin/fixes_2.4
 *    
 *    iSelector:
 *      An array of feature selectors. Each element in the array must
 *      contain a valid feature selector corresponding to a font
 *      feature you provide in the iType parameter. To obtain all
 *      previously set feature selectors for a given style object, you
 *      can call the function ATSUGetAllFontFeatures. You may pass NULL
 *      for this parameter if you are passing kATSUClearAll for the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      iFeatureCount parameter.
=======
 *      iFeatureCount parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      iFeatureCount parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      iFeatureCount parameter.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUClearFontFeatures( iStyle: ATSUStyle; iFeatureCount: ItemCount; {const} iType: {variable-size-array} ATSUFontFeatureTypePtr { can be NULL }; {const} iSelector: {variable-size-array} ATSUFontFeatureSelectorPtr { can be NULL } ): OSStatus; external name '_ATSUClearFontFeatures';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUClearFontFeatures( iStyle: ATSUStyle; iFeatureCount: ItemCount; iType: ATSUFontFeatureTypePtr; iSelector: ATSUFontFeatureSelectorPtr ): OSStatus; external name '_ATSUClearFontFeatures';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUClearFontFeatures( iStyle: ATSUStyle; iFeatureCount: ItemCount; {const} iType: {variable-size-array} ATSUFontFeatureTypePtr { can be NULL }; {const} iSelector: {variable-size-array} ATSUFontFeatureSelectorPtr { can be NULL } ): OSStatus; external name '_ATSUClearFontFeatures';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  Font variations                                                             }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUSetVariations()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithVariation,
 *    CTFontDescriptorCreateWithAttributes instead.
<<<<<<< HEAD
=======
 *  ATSUSetVariations()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUSetVariations()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets font variation axes and values in a style object.
 *  
 *  Discussion:
 *    If you supply font variation axes and values to the
 *    ATSUSetVariations function, you can change the appearance of a
 *    style object's font accordingly. You may specify any number of
 *    variation axes and values in a style object. Any of the font's
 *    variations that you do not set retain their font-defined default
 *    values. You can also use the ATSUSetVariations function to supply
 *    your own value within any variation axes defined for the font.
 *    However, if the font does not support the variation axis you
 *    specify, your custom variation has no visual effect. By calling
 *    the function ATSUGetIndFontVariation, you can obtain a variation
 *    axis and its maximum, minimum, and default values for a font.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style object for which to set font variation values.
 *    
 *    iVariationCount:
 *      The number of font variation values to set. This value should
 *      correspond to the number of elements in the iAxes and iValue
 *      arrays.
 *    
 *    iAxes:
 *      An array of font variation axes. Each element in the array must
 *      represent a valid variation axis tag that corresponds to a
 *      variation value in the iValue array. To obtain a valid
 *      variation axis tag for a font, you can call the functions
 *      ATSUGetIndFontVariation or ATSUGetFontInstance.
 *    
 *    iValue:
 *      An array of font variation values. Each element in the array
 *      must contain a value that is valid for the corresponding
 *      variation axis in the iAxes parameter. You can obtain a font's
 *      maximum, minimum, and default values for a given variation axis
 *      by calling the function ATSUGetIndFontVariation . You can
 *      obtain the font variation axis values for a font instance by
 *      calling ATSUGetFontInstance.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUSetVariations( iStyle: ATSUStyle; iVariationCount: ItemCount; {const} iAxes: {variable-size-array} ATSUFontVariationAxisPtr; {const} iValue: {variable-size-array} ATSUFontVariationValuePtr ): OSStatus; external name '_ATSUSetVariations';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetFontVariationValue()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUSetVariations( iStyle: ATSUStyle; iVariationCount: ItemCount; {const} iAxes: {variable-size-array} ATSUFontVariationAxisPtr; {const} iValue: {variable-size-array} ATSUFontVariationValuePtr ): OSStatus; external name '_ATSUSetVariations';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetFontVariationValue()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontVariationValue()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the current value for a single font variation axis in a
 *    style object.
 *  
 *  Discussion:
 *    This function obtains the setting for a specified font variation
 *    axis in a style object. You might typically call
 *    ATSUGetFontVariationValue if you need to obtain one previously
 *    set variation axis value after another within your program's
 *    processing loop. To obtain all nondefault font variation axis
 *    values for a given style object, you can call the function
 *    ATSUGetAllFontVariations. Before calling
 *    ATSUGetFontVariationValue, call the function
 *    ATSUGetAllFontVariations to obtain the font variation axes that
 *    are set for the style object.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style for which you want to obtain a variation value.
 *    
 *    iFontVariationAxis:
 *      A tag specifying the style object's variation axis to examine.
 *      You can obtain a list of variation axis tags that are set to
 *      non-default values in a particular style object from the
 *      function ATSUGetAllFontVariations.
 *    
 *    oFontVariationValue:
 *      On return, ATSUGetFontVariationValue produces the currently set
 *      value for the style object's specified variation axis. If this
 *      value has not been set, ATSUGetFontVariationValue produces the
 *      font-defined default value.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontVariationValue( iStyle: ATSUStyle; iFontVariationAxis: ATSUFontVariationAxis; oFontVariationValue: ATSUFontVariationValuePtr ): OSStatus; external name '_ATSUGetFontVariationValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetAllFontVariations()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontVariationValue( iStyle: ATSUStyle; iFontVariationAxis: ATSUFontVariationAxis; oFontVariationValue: ATSUFontVariationValuePtr ): OSStatus; external name '_ATSUGetFontVariationValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetAllFontVariations()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetAllFontVariations()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a style object's font variation values that are not at
 *    default settings.
 *  
 *  Discussion:
 *    This function obtains all of a style object's font variation axes
 *    that are not at default settings, as well as the current values
 *    for the axes. Typically you use the function
 *    ATSUGetAllFontVariations by calling it twice, as follows: (1)
 *    Pass a reference to the style object to examine in the iStyle
 *    parameter, a pointer to an ItemCount value in the
 *    oActualVariationCount parameter, NULL for the oVariationAxes and
 *    oFontVariationValues parameters, and 0 for the iVariationCount
 *    parameter. ATSUGetAllFontVariations returns the size to use for
 *    the variation axes and value arrays in the oActualVariationCount
 *    parameter. (2) Allocate enough space for arrays of the returned
 *    size, then call ATSUGetAllFontVariations again, passing a pointer
 *    to the arrays in the oVariationAxes and oFontVariationValues
 *    parameters. On return, the arrays contain the font variation axes
 *    and their corresponding values, respectively, for the style
 *    object.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      A style for which you wish to obtain information about current
 *      variation settings.
 *    
 *    iVariationCount:
 *      The maximum number of font variation values to obtain for the
 *      style object. Typically, this is equivalent to the number of
 *      ATSUFontVariationAxis and ATSUFontVariationValue values for
 *      which you have allocated memory in the oVariationAxes and
 *      oFontVariationValues parameters, respectively. To determine
 *      this value, see the Discussion.
 *    
 *    oVariationAxes:
 *      On return, the array contains the current font variation values
 *      for the font variation axes produced in the oVariationAxes
 *      array. If you are uncertain of how much memory to allocate for
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      this array, see the Discussion.
=======
 *      this array, see the Discussion. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      this array, see the Discussion. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      this array, see the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    oFontVariationValues:
 *      On return, the value specifies the actual number of nondefault
 *      font variation values in the style object. This may be greater
 *      than the value you passed in the iVariationCount parameter. If
 *      you are uncertain of how much memory to allocate for this
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      array, see the Discussion.
=======
 *      array, see the Discussion. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      array, see the Discussion. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      array, see the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    oActualVariationCount:
 *      On return, the value specifies the actual number of nondefault
 *      font variation values in the style object. This may be greater
 *      than the value you passed in the iVariationCount parameter.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
 *       can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *       can be NULL
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetAllFontVariations( iStyle: ATSUStyle; iVariationCount: ItemCount; oVariationAxes: {variable-size-array} ATSUFontVariationAxisPtr { can be NULL }; oFontVariationValues: {variable-size-array} ATSUFontVariationValuePtr { can be NULL }; oActualVariationCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAllFontVariations';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUClearFontVariations()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetAllFontVariations( iStyle: ATSUStyle; iVariationCount: ItemCount; oVariationAxes: {variable-size-array} ATSUFontVariationAxisPtr { can be NULL }; oFontVariationValues: {variable-size-array} ATSUFontVariationValuePtr { can be NULL }; oActualVariationCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAllFontVariations';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUClearFontVariations()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUClearFontVariations()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Restores default values to the specified font variation axes of a
 *    style object.
 *  
 *  Discussion:
 *    The ATSUClearFontVariations function removes those font variation
 *    axis values identified by variation axis tags in the iAxis array
 *    and replaces them with their font-defined default values. You can
 *    remove unset font variation values from a style object without a
 *    function error. To restore default font features to a style
 *    object, call the function ATSUClearFontFeatures. To restore
 *    default style attributes, call ATSUClearAttributes. To restore
 *    all default settings to a style object (for font features,
 *    variations, and style attributes), call the function
 *    ATSUClearStyle.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style in which you wish to clear font variation settings.
 *    
 *    iAxisCount:
 *      The number of font variation axes to restore to default
 *      settings. This value should correspond to the number of
 *      elements in the iAxis array. To restore default values to all
 *      the font variation axes in the style object, pass the constant
 *      kATSUClearAll in this parameter. If you pass kATSUClearAll the
 *      value in the iAxis parameter is ignored.
 *    
 *    iAxis:
 *      An array of font variation axes. Each element in the array must
 *      contain a valid tag that corresponds to a font variation axis
 *      to restore to its default setting. You can obtain variation
 *      axis tags for a style object from the function
 *      ATSUGetAllFontVariations. You may pass NULL for this parameter
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      if you are passing kATSUClearAll for the iAxisCount parameter.
=======
 *      if you are passing kATSUClearAll for the iAxisCount parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      if you are passing kATSUClearAll for the iAxisCount parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      if you are passing kATSUClearAll for the iAxisCount parameter.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUClearFontVariations( iStyle: ATSUStyle; iAxisCount: ItemCount; {const} iAxis: {variable-size-array} ATSUFontVariationAxisPtr { can be NULL } ): OSStatus; external name '_ATSUClearFontVariations';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUClearFontVariations( iStyle: ATSUStyle; iAxisCount: ItemCount; iAxis: ATSUFontVariationAxisPtr ): OSStatus; external name '_ATSUClearFontVariations';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUClearFontVariations( iStyle: ATSUStyle; iAxisCount: ItemCount; {const} iAxis: {variable-size-array} ATSUFontVariationAxisPtr { can be NULL } ): OSStatus; external name '_ATSUClearFontVariations';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  Font ID's                                                                   }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUFontCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCollectionCreateFromAvailableFonts instead.
<<<<<<< HEAD
=======
 *  ATSUFontCount()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUFontCount()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the number of ATSUI-compatible fonts installed on a
 *    user's system.
 *  
 *  Discussion:
 *    The ATSUFontCount function obtains the number of fonts on a
 *    user's system that are compatible with ATSUI. Incompatible fonts
 *    include those that cannot be used to represent Unicode, the
 *    missing-character glyph font, and fonts whose names begin with a
 *    period or a percent sign. You can use the count produced in the
 *    oFontCount parameter to determine the amount of memory to
 *    allocate for the oFontIDs array in the function ATSUGetFontIDs.
 *    It is important to note that the set of installed
 *    ATSUI-compatible fonts may change while your application is
 *    running. In Mac OS X, the set of installed fonts may change at
 *    any time. Although in Mac OS 9, fonts cannot be removed from the
 *    Fonts folder while an application other than the Finder is
 *    running, they can be removed from other locations, and it is
 *    possible for fonts to be added. Additionally, just because the
 *    number of fonts stays the same between two successive calls to
 *    ATSUFontCount , this does not mean that the font lists are the
 *    same. It is possible for a font to be added and another removed
 *    between two successive calls to ATSUFontCount , leaving the total
 *    number unchanged.
 *  
 *  Parameters:
 *    
 *    oFontCount:
 *      On return, the number of ATSUI-compatible fonts installed on a
 *      user's system.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFontCount( var oFontCount: ItemCount ): OSStatus; external name '_ATSUFontCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetFontIDs()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCollectionCreateFromAvailableFonts
 *    CTFontCollectionCreateWithFontDescriptors, or
 *    CTFontCollectionCreateCopyWithFontDescriptors instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFontCount( var oFontCount: ItemCount ): OSStatus; external name '_ATSUFontCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetFontIDs()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontIDs()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCollectionCreateFromAvailableFonts
 *    CTFontCollectionCreateWithFontDescriptors, or
 *    CTFontCollectionCreateCopyWithFontDescriptors instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a list of all the ATSUI-compatible fonts installed on the
 *    user's system.
 *  
 *  Discussion:
 *    Use the function ATSUFontCount to determine how much memory to
 *    allocate before calling this function. Also see the discussion
 *    for the ATSUFontCount function.
 *  
 *  Parameters:
 *    
 *    oFontIDs:
 *      On return, the array contains unique identifiers for each of
 *      the ATSUI-compatible fonts installed on the user's system. You
 *      should allocate enough memory to contain an array the size of
 *      the count produced by the function ATSUFontCount.
 *    
 *    iArraySize:
 *      The maximum number of fonts to obtain. Typically, this is
 *      equivalent to the number of ATSUFontID values for which you
 *      have allocated memory in the oFontIDs parameter.
 *    
 *    oFontCount:
 *      On return, the value specifies the actual number of
 *      ATSUI-compatible fonts installed on the user's system. This may
 *      be greater than the value you specified in the iArraySize
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      parameter.
=======
 *      parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      parameter.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontIDs( oFontIDs: {variable-size-array} ATSUFontIDPtr; iArraySize: ItemCount; oFontCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetFontIDs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUFONDtoFontID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontIDs( oFontIDs: {variable-size-array} ATSUFontIDPtr; iArraySize: ItemCount; oFontCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetFontIDs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUFONDtoFontID()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUFONDtoFontID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Finds the ATSUI font ID that corresponds to a font family number,
 *    if one exists.
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    This function is not recommended. It is specifically associated
 *    with old QD data types (i.e. the QD font family or 'FOND'
 *    identifier) that has no equivalent in the newer API sets. The
 *    concept of FOND ID and font family instances are all QD-specific
 *    concepts that are not supported in Cocoa, Quartz, and the
 *    non-deprecated parts of ATS and ATSUI.
<<<<<<< HEAD
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFONDtoFontID( iFONDNumber: SInt16; iFONDStyle: Style; var oFontID: ATSUFontID ): OSStatus; external name '_ATSUFONDtoFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUFontIDtoFOND()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    This function is not recommended. Instead, use the function
 *    FMGetFontFromFontFamilyInstance (see Fonts.h). FMFont values are
 *    equivalent to ATSUFontID values.
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFONDtoFontID( iFONDNumber: SInt16; iFONDStyle: Style; var oFontID: ATSUFontID ): OSStatus; external name '_ATSUFONDtoFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUFontIDtoFOND()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUFontIDtoFOND()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Finds the font family number and style that correspond to an
 *    ATSUI font ID, if these exist.
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    This function is not recommended. It is specifically associated
 *    with old QD data types (i.e. the QD font family or 'FOND'
 *    identifier) that has no equivalent in the newer API sets. The
 *    concept of FOND ID and font family instances are all QD-specific
 *    concepts that are not supported in Cocoa, Quartz, and the
 *    non-deprecated parts of ATS and ATSUI.
<<<<<<< HEAD
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFontIDtoFOND( iFontID: ATSUFontID; var oFONDNumber: SInt16; var oFONDStyle: Style ): OSStatus; external name '_ATSUFontIDtoFOND';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    This function is not recommended. Instead, use the function
 *    FMGetFontFamilyInstanceFromFont (see Fonts.h). FMFont values are
 *    equivalent to ATSUFontID values.
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFontIDtoFOND( iFontID: ATSUFontID; var oFONDNumber: SInt16; var oFONDStyle: Style ): OSStatus; external name '_ATSUFontIDtoFOND';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  Font names                                                                  }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCountFontNames()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyPostScriptName, CTFontCopyFamilyName,
 *    CTFontCopyFullName, CTFontCopyDisplayName, CTFontCopyName, or
 *    CTFontCopyLocalizedName instead.
<<<<<<< HEAD
=======
 *  ATSUCountFontNames()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCountFontNames()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the number of font names that correspond to a given ATSUI
 *    font ID.
 *  
 *  Discussion:
 *    This function obtains the number of font names defined in a font
 *    name table for a given ATSUI font ID. This number includes
 *    repetitions of the same name in different platforms, languages,
 *    and scripts; names of font features, variations, tracking
 *    settings, and instances for the font; and font names identified
 *    by name code constants. You can pass an index value based on this
 *    count to the function ATSUGetIndFontName to obtain a name string,
 *    name code, platform, script, and language for a given ATSUI font
 *    ID.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which you wish to obtain the font name count.
 *    
 *    oFontNameCount:
 *      On return, the value specifies the number of entries in the
 *      font name table corresponding to the given ATSUI font ID.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontNames( iFontID: ATSUFontID; var oFontNameCount: ItemCount ): OSStatus; external name '_ATSUCountFontNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetIndFontName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyPostScriptName, CTFontCopyFamilyName,
 *    CTFontCopyFullName, CTFontCopyDisplayName, CTFontCopyName, or
 *    CTFontCopyLocalizedName instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontNames( iFontID: ATSUFontID; var oFontNameCount: ItemCount ): OSStatus; external name '_ATSUCountFontNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetIndFontName()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetIndFontName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyPostScriptName, CTFontCopyFamilyName,
 *    CTFontCopyFullName, CTFontCopyDisplayName, CTFontCopyName, or
 *    CTFontCopyLocalizedName instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a name string, name code, platform, script, and language
 *    for the font that matches an ATSUI font ID and name table index
 *    value.
 *  
 *  Discussion:
 *    Typically you use the ATSUGetIndFontName function by calling it
 *    twice, as follows: (1) Pass valid values for the iFontID,
 *    iFontNameIndex, and oActualNameLength parameters, 0 for the
 *    iMaximumNameLength parameter, and NULL for the other parameters.
 *    ATSUGetIndFontName returns the length of the font name string in
 *    the oActualNameLength parameter. (2) Allocate enough space for a
 *    buffer of the returned size, then call the function again,
 *    passing a valid pointer to the buffer in the oName parameter. On
 *    return, the buffer contains the font name string. To find a name
 *    string and index value for the first font in a name table that
 *    matches an ATSUI font ID and the specified font parameters, call
 *    the function ATSUFindFontName. To obtain an ATSUI font ID for the
 *    first font in a name table that matches the specified name
 *    string, name code, platform, script, and/or language, call the
 *    function ATSUFindFontFromName.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain information. Note that because
 *      Apple Type Services assigns ATSUFontID values systemwide at
 *      runtime, font IDs can change across system restarts.
 *    
 *    iFontNameIndex:
 *      An index to the font for which to obtain information. Because
 *      this index must be 0-based, you should pass a value between 0
 *      and one less than the count produced by the function
 *      ATSUCountFontNames.
 *    
 *    iMaximumNameLength:
 *      The maximum length of the font name string to obtain.
 *      Typically, this is equivalent to the size of the buffer that
 *      you have allocated in the oName parameter. To determine this
 *      length, see the Discussion.
 *    
 *    oName:
 *      On return, the buffer contains the name string of the font
 *      matching the ATSUI font ID and name table index value being
 *      passed. If the buffer you allocate is not large enough to
 *      contain the name string, ATSUGetIndFontName produces a partial
 *      string. If you are unsure how much memory to allocate for this
 *      parameter, see the Discussion.
 *    
 *    oActualNameLength:
 *      On return, the value specifies the actual length of the
 *      complete name string. This may be greater than the value passed
 *      in the iMaximumNameLength parameter. You should check this
 *      value to ensure that you have allocated sufficient memory and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      therefore obtained the complete name string for the font.
=======
 *      therefore obtained the complete name string for the font. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      therefore obtained the complete name string for the font. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      therefore obtained the complete name string for the font.
>>>>>>> origin/fixes_2.4
 *    
 *    oFontNameCode:
 *      On return, a value specifying the type of name returned (i.e.,
 *      full name, postscript name) of the font. See SFNTTypes.h for a
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      list of possible values.
 *    
 *    oFontNamePlatform:
 *      On return, a value specifying the encoding of the font. See
 *      SFNTTypes.h for a list of possible values.
 *    
 *    oFontNameScript:
 *      On return, a value specifying the script of the font. See
 *      SFNTTypes.h for a list of possible values.
 *    
 *    oFontNameLanguage:
 *      On return, a value specifying the language of the font. See
 *      SFNTTypes.h for a list of possible values.
=======
=======
>>>>>>> origin/fixes_2_2
 *      list of possible values. can be NULL
=======
 *      list of possible values.
>>>>>>> origin/fixes_2.4
 *    
 *    oFontNamePlatform:
 *      On return, a value specifying the encoding of the font. See
 *      SFNTTypes.h for a list of possible values.
 *    
 *    oFontNameScript:
 *      On return, a value specifying the script of the font. See
 *      SFNTTypes.h for a list of possible values.
 *    
 *    oFontNameLanguage:
 *      On return, a value specifying the language of the font. See
<<<<<<< HEAD
 *      SFNTTypes.h for a list of possible values. can be NULL
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *      SFNTTypes.h for a list of possible values.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetIndFontName( iFontID: ATSUFontID; iFontNameIndex: ItemCount; iMaximumNameLength: ByteCount; oName: Ptr; oActualNameLength: ByteCountPtr { can be NULL }; oFontNameCode: FontNameCodePtr { can be NULL }; oFontNamePlatform: FontPlatformCodePtr { can be NULL }; oFontNameScript: FontScriptCodePtr { can be NULL }; oFontNameLanguage: FontLanguageCodePtr { can be NULL } ): OSStatus; external name '_ATSUGetIndFontName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUFindFontName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyPostScriptName, CTFontCopyFamilyName,
 *    CTFontCopyFullName, CTFontCopyDisplayName, CTFontCopyName, or
 *    CTFontCopyLocalizedName instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetIndFontName( iFontID: ATSUFontID; iFontNameIndex: ItemCount; iMaximumNameLength: ByteCount; oName: Ptr; oActualNameLength: ByteCountPtr { can be NULL }; oFontNameCode: FontNameCodePtr { can be NULL }; oFontNamePlatform: FontPlatformCodePtr { can be NULL }; oFontNameScript: FontScriptCodePtr { can be NULL }; oFontNameLanguage: FontLanguageCodePtr { can be NULL } ): OSStatus; external name '_ATSUGetIndFontName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUFindFontName()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUFindFontName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyPostScriptName, CTFontCopyFamilyName,
 *    CTFontCopyFullName, CTFontCopyDisplayName, CTFontCopyName, or
 *    CTFontCopyLocalizedName instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a name string and index value for the first font in a
 *    name table that matches the specified ATSUI font ID, name code,
 *    platform, script, and/or language.
 *  
 *  Discussion:
 *    Typically you use the ATSUFindFontName function by calling it
 *    twice, as follows: (1) Pass NULL for the oName and oFontNameIndex
 *    parameters, 0 for the iMaximumNameLength parameter, and valid
 *    values for the other parameters. ATSUFindFontName returns the
 *    length of the font name string in the oActualNameLength
 *    parameter. (2) Allocate enough space for a buffer of the returned
 *    size, then call the function again, passing a valid pointer to
 *    the buffer in the oName parameter. On return, the buffer contains
 *    the font name string. To obtain an ATSUI font ID for the first
 *    font in a name table that matches the specified name string, name
 *    code, platform, script, and/or language, call the function
 *    ATSUFindFontFromName. To obtain the font name string, name code,
 *    platform, script, and language for the font that matches an ATSUI
 *    font ID and name table index, call the function
 *    ATSUGetIndFontName. Although they will each accept NULL on input
 *    individually, you must pass a vaild pointer to at least one of
 *    the three parameters oName, oActualNameLength, or oFontNameIndex,
 *    or ATSUFindFontName will return paramErr.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain a name string. Note that because
 *      Apple Type Services assigns ATSUFontID values systemwide at
 *      runtime, font IDs can change across system restarts.
 *    
 *    iFontNameCode:
 *      A constant specifying the FontNameCode value of the font for
 *      which to obtain a name string. See the SFNTTypes.h header file
 *      for a definition of the FontNameCode type and a list of
 *      possible values.
 *    
 *    iFontNamePlatform:
 *      A constant specifying the encoding of the font. See SFNTTypes.h
 *      for possible values to pass for this parameter. If you pass the
 *      kFontNoPlatformCode constant, ATSUFindFontName produces the
 *      first font in the name table matching the other specified
 *      parameters.
 *    
 *    iFontNameScript:
 *      A constant specifying the script of the font. See SFNTTypes.h
 *      for possible values to pass for this parameter. If you pass the
 *      kFontNoScriptCode constant, ATSUFindFontName produces the first
 *      font in the name table matching the other specified parameters.
 *    
 *    iFontNameLanguage:
 *      A constant specifying the language of the font you are
 *      searching for. See SFNTLayoutTypes.h for possible values to
 *      pass for this parameter.
 *    
 *    iMaximumNameLength:
 *      The maximum size of string you want ATSUFindFontName to return.
 *      Typically, this value is equal to the size of the buffer you
 *      have allocated for the oName parameter. To determine this
 *      length, see the Discussion.
 *    
 *    oName:
 *      On return, the name string of the first font in the font name
 *      table matching your specified parameters. If the buffer you
 *      allocate is not large enough, ATSUFindFontName produces a
 *      partial string. If you are unsure how much space to allocate
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      for this parameter, see the Discussion.
=======
 *      for this parameter, see the Discussion. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      for this parameter, see the Discussion. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      for this parameter, see the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    oActualNameLength:
 *      On return, specifies the actual length of the complete name
 *      string. This may be greater than the value passed in the
 *      iMaximumNameLength parameter. You should check this value to
 *      ensure that you have allocated sufficient memory and therefore
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      obtained the complete name string for the font.
 *    
 *    oFontNameIndex:
 *      On return, the value provides a 0-based index to the font name
 *      in the font name table.
=======
=======
>>>>>>> origin/fixes_2_2
 *      obtained the complete name string for the font. can be NULL
 *    
 *    oFontNameIndex:
 *      On return, the value provides a 0-based index to the font name
 *      in the font name table. can be NULL
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *      obtained the complete name string for the font.
 *    
 *    oFontNameIndex:
 *      On return, the value provides a 0-based index to the font name
 *      in the font name table.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFindFontName( iFontID: ATSUFontID; iFontNameCode: FontNameCode; iFontNamePlatform: FontPlatformCode; iFontNameScript: FontScriptCode; iFontNameLanguage: FontLanguageCode; iMaximumNameLength: ByteCount; oName: Ptr { can be NULL }; oActualNameLength: ByteCountPtr { can be NULL }; oFontNameIndex: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUFindFontName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUFindFontFromName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreateWithName instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFindFontName( iFontID: ATSUFontID; iFontNameCode: FontNameCode; iFontNamePlatform: FontPlatformCode; iFontNameScript: FontScriptCode; iFontNameLanguage: FontLanguageCode; iMaximumNameLength: ByteCount; oName: Ptr { can be NULL }; oActualNameLength: ByteCountPtr { can be NULL }; oFontNameIndex: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUFindFontName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUFindFontFromName()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUFindFontFromName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreateWithName instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains an ATSUI font ID for the first font in a name table that
 *    matches the specified name string, name code, platform, script,
 *    and/or language.
 *  
 *  Discussion:
 *    Because ATSUI cannot guarantee the uniqueness of names among
 *    installed fonts, ATSUFindFontFromName does not necessarily find
 *    the only font that matches these parameters. As a result, you may
 *    want to create a more sophisticated name-matching algorithm or
 *    guarantee the uniqueness of names among installed fonts.
 *  
 *  Parameters:
 *    
 *    iName:
 *      A pointer to a buffer containing the name string of the font
 *      for which to obtain an ATSUI font ID.
 *    
 *    iNameLength:
 *      The length, in bytes, of the name string provided in the iName
 *      parameter.
 *    
 *    iFontNameCode:
 *      A constant specifying the type of name to search for (i.e.,
 *      full name, postcript name). See SFNTTypes.h for a list possible
 *      values to pass for this parameter.
 *    
 *    iFontNamePlatform:
 *      A constant specifying the encoding of the font you are
 *      searching for. See SFNTTypes.h for possible values to pass for
 *      this parameter. Pass kFontNoPlatformCode if you do not want to
 *      limit your search to a particular encoding.
 *    
 *    iFontNameScript:
 *      A constant specifying the script of the font you are searching
 *      for. See SFNTTypes.h for possible values to pass for this
 *      parameter. Pass kFontNoScriptCode if you do not want to limit
 *      your search to a particular script.
 *    
 *    iFontNameLanguage:
 *      A constant specifying the language of the font you are
 *      searching for. See SFNTTypes.h for possible values to pass for
 *      this parameter. Pass kFontNoLanguageCode if you do not want to
 *      limit your search to a particular language.
 *    
 *    oFontID:
 *      On return, the value provides a unique identifier for the
 *      specified font. If no installed font matches the specified
 *      parameters, kATSUInvalidFontID is returned for this parameter.
 *  
 *  Result:
 *    On success, noErr is returned. If the font cannot be found,
 *    kATSUInvalidFontErr is returned. See MacErrors.h for other
 *    possible error codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFindFontFromName( iName: {const} UnivPtr; iNameLength: ByteCount; iFontNameCode: FontNameCode; iFontNamePlatform: FontPlatformCode; iFontNameScript: FontScriptCode; iFontNameLanguage: FontLanguageCode; var oFontID: ATSUFontID ): OSStatus; external name '_ATSUFindFontFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUFindFontFromName( iName: Ptr; iNameLength: ByteCount; iFontNameCode: FontNameCode; iFontNamePlatform: FontPlatformCode; iFontNameScript: FontScriptCode; iFontNameLanguage: FontLanguageCode; var oFontID: ATSUFontID ): OSStatus; external name '_ATSUFindFontFromName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUFindFontFromName( iName: {const} UnivPtr; iNameLength: ByteCount; iFontNameCode: FontNameCode; iFontNamePlatform: FontPlatformCode; iFontNameScript: FontScriptCode; iFontNameLanguage: FontLanguageCode; var oFontID: ATSUFontID ): OSStatus; external name '_ATSUFindFontFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  Font features                                                               }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCountFontFeatureTypes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
<<<<<<< HEAD
=======
 *  ATSUCountFontFeatureTypes()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCountFontFeatureTypes()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the number of available feature types in a font.
 *  
 *  Discussion:
 *    This function function obtains the total number of feature types
 *    defined for a font. You can use the count produced by
 *    ATSUCountFontFeatureTypes to determine how much memory to
 *    allocate for the oTypes array in the function
 *    ATSUGetFontFeatureTypes.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain a count of feature types.
 *    
 *    oTypeCount:
 *      On return, the actual number of feature types defined for the
 *      font.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontFeatureTypes( iFontID: ATSUFontID; var oTypeCount: ItemCount ): OSStatus; external name '_ATSUCountFontFeatureTypes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUCountFontFeatureSelectors()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontFeatureTypes( iFontID: ATSUFontID; var oTypeCount: ItemCount ): OSStatus; external name '_ATSUCountFontFeatureTypes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUCountFontFeatureSelectors()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUCountFontFeatureSelectors()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the number of available feature selectors for a given
 *    feature type in a font.
 *  
 *  Discussion:
 *    This function obtains the total number of feature selectors
 *    defined for a given feature type in the font. You can use the
 *    count produced by ATSUCountFontFeatureSelectors to determine how
 *    much memory to allocate for the oSelectors array in the function
 *    ATSUGetFontFeatureSelectors.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain feature selector information.
 *    
 *    iType:
 *      A value specifying one of the font's supported feature types.
 *      To obtain the available feature types for a font, call the
 *      function ATSUGetFontFeatureTypes.
 *    
 *    oSelectorCount:
 *      On return, specifies the actual number of feature selectors
 *      defined for the feature type by the font.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontFeatureSelectors( iFontID: ATSUFontID; iType: ATSUFontFeatureType; var oSelectorCount: ItemCount ): OSStatus; external name '_ATSUCountFontFeatureSelectors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetFontFeatureTypes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontFeatureSelectors( iFontID: ATSUFontID; iType: ATSUFontFeatureType; var oSelectorCount: ItemCount ): OSStatus; external name '_ATSUCountFontFeatureSelectors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetFontFeatureTypes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontFeatureTypes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the available feature types of a font.
 *  
 *  Discussion:
 *    A given font may not support all possible feature types and
 *    selectors. If you select features that are not available in a
 *    font, you won't see a change in the glyph's appearance. To
 *    determine the available features of a font, you can call the
 *    functions ATSUGetFontFeatureTypes and
 *    ATSUGetFontFeatureSelectors. The ATSUGetFontFeatureTypes function
 *    reads the font data table for the specified font and obtains its
 *    supported feature types. You can then use this information both
 *    to present the user a list of font features from which to select
 *    and to call such functions as ATSUSetFontFeatures with more
 *    accuracy.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain information about feature types.
 *    
 *    iMaximumTypes:
 *      The maximum number of feature types to obtain for the font.
 *      Typically, this is equivalent to the number of elements in the
 *      oTypes array.
 *    
 *    oTypes:
 *      A pointer to memory you have allocated for an array of
 *      ATSUFontFeatureType values.  You can call the function
 *      ATSUCountFontFeatureTypes to obtain the number of available
 *      feature types for a given font and thus determine the amount of
 *      memory to allocate. On return, the array contains constants
 *      identifying each type of feature that is defined for the font.
 *      The constants that represent font feature types are defined in
 *      the header file SFNTLayoutTypes.h and are described in the
 *      official ATSUI documentation, available on the Apple developer
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      website.
=======
 *      website. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      website. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      website.
>>>>>>> origin/fixes_2.4
 *    
 *    oActualTypeCount:
 *      On return, the actual number of feature types defined in the
 *      font. This may be greater than the value you specify in the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      iMaximumTypes parameter.
=======
 *      iMaximumTypes parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      iMaximumTypes parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      iMaximumTypes parameter.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontFeatureTypes( iFontID: ATSUFontID; iMaximumTypes: ItemCount; oTypes: {variable-size-array} ATSUFontFeatureTypePtr { can be NULL }; oActualTypeCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetFontFeatureTypes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetFontFeatureSelectors()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontFeatureTypes( iFontID: ATSUFontID; iMaximumTypes: ItemCount; oTypes: {variable-size-array} ATSUFontFeatureTypePtr { can be NULL }; oActualTypeCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetFontFeatureTypes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetFontFeatureSelectors()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontFeatureSelectors()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the available feature selectors for a given feature type
 *    in a font.
 *  
 *  Discussion:
 *    A given font may not support all possible feature types and
 *    selectors. If you select features that are not available in a
 *    font, you won't see a change in the glyph's appearance. To
 *    determine the available features of a font, you can call the
 *    functions ATSUGetFontFeatureTypes and
 *    ATSUGetFontFeatureSelectors. The ATSUGetFontFeatureSelectors
 *    function reads the font data table for the specified font and
 *    obtains its supported feature selectors for the given feature
 *    types. You can then use this information both to present the user
 *    a list of font features from which to select and to call such
 *    functions as ATSUSetFontFeatures with more accuracy.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain feature selectors.
 *    
 *    iType:
 *      An ATSUFontFeatureType value specifying one of the font's
 *      supported feature types. To obtain the available feature types
 *      for a font, call the function ATSUGetFontFeatureTypes.
 *    
 *    iMaximumSelectors:
 *      An ItemCount value specifying the maximum number of feature
 *      selectors to obtain for the font's specified feature type.
 *      Typically, this is equivalent to the number of elements in the
 *      oSelectors array.
 *    
 *    oSelectors:
 *      A pointer to memory you have allocated for an array of
 *      ATSUFontFeatureSelector values.  You can call the function
 *      ATSUCountFontFeatureSelectors to obtain the number of available
 *      feature selectors for a given font feature type and thus
 *      determine the amount of memory to allocate. On return, the
 *      array contains constants identifying each available feature
 *      selector for the given feature type. The constants that
 *      represent font feature selectors are defined in the header file
 *      SFNTLayoutTypes.h and are described in the official ATSUI
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      documentation, available on the Apple developer website.
=======
 *      documentation, available on the Apple developer website. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      documentation, available on the Apple developer website. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      documentation, available on the Apple developer website.
>>>>>>> origin/fixes_2.4
 *    
 *    oSelectorIsOnByDefault:
 *      A pointer to memory you have allocated for an array of Boolean
 *      values. The number of elements in this array should correspond
 *      to the number of elements in the oSelectors array. On return,
 *      the array contains Boolean values indicating whether the
 *      corresponding feature selector in the oSelectors array is on or
 *      off. If true, the feature selector is on by default; if false,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      off.
=======
 *      off. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      off. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      off.
>>>>>>> origin/fixes_2.4
 *    
 *    oActualSelectorCount:
 *      On return, the value specifies the actual number of feature
 *      selectors defined for the given feature type. This value may be
 *      greater than the value you specify in the iMaximumSelectors
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      parameter.
=======
 *      parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      parameter.
>>>>>>> origin/fixes_2.4
 *    
 *    oIsMutuallyExclusive:
 *      On return, the value indicates whether the feature selectors
 *      for the given feature type are exclusive or nonexclusive. If a
 *      feature type is exclusive you can choose only one of its
 *      available feature selectors at a time, such as whether to
 *      display numbers as proportional or fixed-width. If a feature
 *      type is nonexclusive, you can enable any number of feature
 *      selectors at once. If true , the feature type is exclusive and
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      only one selector can be used at a time.
=======
 *      only one selector can be used at a time. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      only one selector can be used at a time. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      only one selector can be used at a time.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontFeatureSelectors( iFontID: ATSUFontID; iType: ATSUFontFeatureType; iMaximumSelectors: ItemCount; oSelectors: {variable-size-array} ATSUFontFeatureSelectorPtr { can be NULL }; oSelectorIsOnByDefault: {variable-size-array} BooleanPtr { can be NULL }; oActualSelectorCount: ItemCountPtr { can be NULL }; oIsMutuallyExclusive: BooleanPtr { can be NULL } ): OSStatus; external name '_ATSUGetFontFeatureSelectors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetFontFeatureNameCode()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontFeatureSelectors( iFontID: ATSUFontID; iType: ATSUFontFeatureType; iMaximumSelectors: ItemCount; oSelectors: {variable-size-array} ATSUFontFeatureSelectorPtr { can be NULL }; oSelectorIsOnByDefault: {variable-size-array} BooleanPtr { can be NULL }; oActualSelectorCount: ItemCountPtr { can be NULL }; oIsMutuallyExclusive: BooleanPtr { can be NULL } ): OSStatus; external name '_ATSUGetFontFeatureSelectors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetFontFeatureNameCode()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontFeatureNameCode()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyFeatures, CTFontCopyFeatureSettings instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    btains the name code for a font's feature type or selector that
 *    matches an ASTUI font ID, feature type, and feature selector.
 *  
 *  Discussion:
 *    This function obtains the name code for a font's feature type or
 *    selector that matches an ASTUI font ID, feature type and feature
 *    selector values. By default, ATSUGetFontFeatureNameCode function
 *    obtains the name code of a feature selector. To determine the
 *    name code of a feature type, pass the constant kATSUNoSelector in
 *    the iSelector parameter. You can use the function
 *    ATSUFindFontName to obtain the localized name string for the name
 *    code produced by ATSUGetFontFeatureNameCode.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain the name code for a feature type
 *      or selector.
 *    
 *    iType:
 *      A constant identifying a valid feature type. To obtain the
 *      valid feature types for a font, call the function
 *      ATSUGetFontFeatureTypes.
 *    
 *    iSelector:
 *      A constant identifying a valid feature selector that
 *      corresponds to the feature type passed in the iType parameter.
 *      If you pass the constant kATSUNoSelector, the name code
 *      produced by ATSUGetFontFeatureNameCode is that of the feature
 *      type, not the feature selector. To obtain the valid feature
 *      selectors for a font, call the function
 *      ATSUGetFontFeatureSelectors.
 *    
 *    oNameCode:
 *      On return, the value contains the name code for the font
 *      feature selector or type. See the SFNTTypes.h header file for a
 *      definition of the FontNameCode type and a list of possible
 *      values.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontFeatureNameCode( iFontID: ATSUFontID; iType: ATSUFontFeatureType; iSelector: ATSUFontFeatureSelector; var oNameCode: FontNameCode ): OSStatus; external name '_ATSUGetFontFeatureNameCode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontFeatureNameCode( iFontID: ATSUFontID; iType: ATSUFontFeatureType; iSelector: ATSUFontFeatureSelector; var oNameCode: FontNameCode ): OSStatus; external name '_ATSUGetFontFeatureNameCode';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  Font tracking value & names                                                 }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCountFontTracking()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyTable instead.
<<<<<<< HEAD
=======
 *  ATSUCountFontTracking()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCountFontTracking()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the number of entries in the font tracking table that
 *    correspond to a given ATSUI font ID and glyph orientation.
 *  
 *  Discussion:
 *    This function obtains the number of font tracking entries defined
 *    in a font tracking table for a given ATSUI font ID and glyph
 *    orientation. You can pass an index value based on this count to
 *    the function ATSUGetIndFontTracking to obtain the name code and
 *    tracking value of a font tracking.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain tracking table information.
 *    
 *    iCharacterOrientation:
 *      A constant identifying the glyph orientation of the font
 *      tracking entries. See the definition of
 *      ATSUVerticalCharacterType for a list of possible values.
 *    
 *    oTrackingCount:
 *      On return, the number of entries in the font tracking table
 *      corresponding to the given ATSUI font ID and glyph orientation.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUCountFontTracking( iFontID: ATSUFontID; iCharacterOrientation: ATSUVerticalCharacterType; var oTrackingCount: ItemCount ): OSStatus; external name '_ATSUCountFontTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetIndFontTracking()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyTable instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUCountFontTracking( iFontID: ATSUFontID; iCharacterOrientation: ATSUVerticalCharacterType; var oTrackingCount: ItemCount ): OSStatus; external name '_ATSUCountFontTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetIndFontTracking()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetIndFontTracking()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyTable instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the name code and tracking value for the font tracking
 *    that matches an ASTUI font ID, glyph orientation, and tracking
 *    table index.
 *  
 *  Discussion:
 *    You can call the ATSUGetIndFontTracking function to obtain the
 *    name code and tracking value that matches the specified ATSUI
 *    font ID, glyph orientation, and tracking table index value. This
 *    information allows you to manipulate tracking settings for a
 *    style using this font via the kATSUTrackingTag attribute. You can
 *    use the function ATSUFindFontName to obtain the localized name
 *    string for the name code produced by ATSUGetIndFontTracking.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain tracking information.
 *    
 *    iCharacterOrientation:
 *      A constant identifying the glyph orientation of the font
 *      tracking entries. See the definition of
 *      ATSUVerticalCharacterType for a list of possible values.
 *    
 *    iTrackIndex:
 *      An index to the font tracking for which to obtain information.
 *      Because this index must be 0-based, you should pass a value
 *      between 0 and one less than the count produced by the function
 *      ATSUCountFontTracking.
 *    
 *    oFontTrackingValue:
 *      On return, the value contains the font tracking value.
 *    
 *    oNameCode:
 *      On return, the value contains the name code for the font
 *      tracking. See the SFNTTypes.h header file for a definition of
 *      the FontNameCode type and a list of possible values.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUGetIndFontTracking( iFontID: ATSUFontID; iCharacterOrientation: ATSUVerticalCharacterType; iTrackIndex: ItemCount; var oFontTrackingValue: Fixed; var oNameCode: FontNameCode ): OSStatus; external name '_ATSUGetIndFontTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUGetIndFontTracking( iFontID: ATSUFontID; iCharacterOrientation: ATSUVerticalCharacterType; iTrackIndex: ItemCount; var oFontTrackingValue: Fixed; var oNameCode: FontNameCode ): OSStatus; external name '_ATSUGetIndFontTracking';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  Font variations                                                             }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCountFontVariations()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
<<<<<<< HEAD
=======
 *  ATSUCountFontVariations()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCountFontVariations()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the number of defined variation axes in a font.
 *  
 *  Discussion:
 *    This function function obtains the total number of variation axes
 *    defined for a font. You can use the count produced by
 *    ATSUCountFontVariations to get information about a specific font
 *    variation axis from the function ATSUGetIndFontVariation.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain a count of variation axes.
 *    
 *    oVariationCount:
 *      On return, a count of the number of variation axes defined for
 *      the font.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontVariations( iFontID: ATSUFontID; var oVariationCount: ItemCount ): OSStatus; external name '_ATSUCountFontVariations';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetIndFontVariation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontVariations( iFontID: ATSUFontID; var oVariationCount: ItemCount ): OSStatus; external name '_ATSUCountFontVariations';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetIndFontVariation()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetIndFontVariation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a variation axis and its value range for a font.
 *  
 *  Discussion:
 *    By calling this function, you can obtain a variation axis and its
 *    maximum, minimum, and default values for a font. If you supply
 *    font variation axes and values to the function ATSUSetVariations,
 *    you can change the appearance of a style object's font
 *    accordingly. Note that while you may pass NULL for any of the
 *    output parameters, at least one must be non-NULL or paramErr will
 *    be returned.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      A font for which to obtain variation information for.
 *    
 *    iVariationIndex:
 *      A value specifying an index into the array of variation axes
 *      for the font. This index identifies the font variation axis to
 *      examine. Because this index is zero-based, you must pass a
 *      value between 0 and one less than the value produced in the
 *      oVariationCount parameter of the function
 *      ATSUCountFontVariations.
 *    
 *    oATSUFontVariationAxis:
 *      On return, a four-character code identifying the font variation
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      axis corresponding to the specified index.
 *    
 *    oMinimumValue:
 *      On return, the variation axis minimum.
 *    
 *    oMaximumValue:
 *      On return, the variation axis maximum.
 *    
 *    oDefaultValue:
 *      On return, the variation axis default.
=======
=======
>>>>>>> origin/fixes_2_2
 *      axis corresponding to the specified index. can be NULL
=======
 *      axis corresponding to the specified index.
>>>>>>> origin/fixes_2.4
 *    
 *    oMinimumValue:
 *      On return, the variation axis minimum.
 *    
 *    oMaximumValue:
 *      On return, the variation axis maximum.
 *    
 *    oDefaultValue:
<<<<<<< HEAD
 *      On return, the variation axis default. can be NULL
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *      On return, the variation axis default.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetIndFontVariation( iFontID: ATSUFontID; iVariationIndex: ItemCount; oATSUFontVariationAxis: ATSUFontVariationAxisPtr { can be NULL }; oMinimumValue: ATSUFontVariationValuePtr { can be NULL }; oMaximumValue: ATSUFontVariationValuePtr { can be NULL }; oDefaultValue: ATSUFontVariationValuePtr { can be NULL } ): OSStatus; external name '_ATSUGetIndFontVariation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetFontVariationNameCode()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetIndFontVariation( iFontID: ATSUFontID; iVariationIndex: ItemCount; oATSUFontVariationAxis: ATSUFontVariationAxisPtr { can be NULL }; oMinimumValue: ATSUFontVariationValuePtr { can be NULL }; oMaximumValue: ATSUFontVariationValuePtr { can be NULL }; oDefaultValue: ATSUFontVariationValuePtr { can be NULL } ): OSStatus; external name '_ATSUGetIndFontVariation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetFontVariationNameCode()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontVariationNameCode()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the name code for the font variation that matches an
 *    ASTUI font ID and font variation axis.
 *  
 *  Discussion:
 *    This function function obtains the name code for the font
 *    variation that matches an ASTUI font ID and font variation axis
 *    tag. You can use the function ATSUFindFontName to obtain the
 *    localized name string for the name code produced by
 *    ATSUGetFontVariationNameCode.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain a font variation name code.
 *    
 *    iAxis:
 *      An ATSUFontVariationAxis value representing a valid variation
 *      axis tag. To obtain a valid variation axis tag for a font, you
 *      can call the functions ATSUGetIndFontVariation or
 *      ATSUGetFontInstance.
 *    
 *    oNameCode:
 *      On return, the value contains the name code for the font
 *      variation. See the SFNTTypes.h header file for a definition of
 *      the FontNameCode type and a list of possible values.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontVariationNameCode( iFontID: ATSUFontID; iAxis: ATSUFontVariationAxis; var oNameCode: FontNameCode ): OSStatus; external name '_ATSUGetFontVariationNameCode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontVariationNameCode( iFontID: ATSUFontID; iAxis: ATSUFontVariationAxis; var oNameCode: FontNameCode ): OSStatus; external name '_ATSUGetFontVariationNameCode';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  Font Instances                                                              }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCountFontInstances()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
<<<<<<< HEAD
=======
 *  ATSUCountFontInstances()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCountFontInstances()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the number of defined font instances in a font.
 *  
 *  Discussion:
 *    This function obtains the total number of font instances defined
 *    in a font. You can use an index value derived from this count to
 *    get information about a specific font instance by calling the
 *    function ATSUGetFontInstance.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain a count of defined instances.
 *    
 *    oInstances:
 *      On return, the value specifies the number of font instances
 *      defined for the font.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontInstances( iFontID: ATSUFontID; var oInstances: ItemCount ): OSStatus; external name '_ATSUCountFontInstances';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetFontInstance()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCountFontInstances( iFontID: ATSUFontID; var oInstances: ItemCount ): OSStatus; external name '_ATSUCountFontInstances';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetFontInstance()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontInstance()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the font variation axis values for a font instance.
 *  
 *  Discussion:
 *    with a minimum value of 0.0, a default of 0.5, and a maximum of
 *    1.0. Additionally, the variation axis 'wdth' is also defined for
 *    the font, with a similar value range. The type designer can then
 *    choose to declare a font instance for a set of specific values
 *    within these axes, such as “Demibold” for a value of 0.8 for the
 *    'wght' axis and 0.5 for the 'wdth' axis. By calling the function
 *    ATSUGetFontInstance , you can obtain the variation axis values
 *    for a given index into an array of font instances. Typically you
 *    use the function ATSUGetFontInstance by calling it twice, as
 *    follows: (1) Pass the ID of the font to examine in the iFont
 *    parameter, a valid pointer to an ItemCount value in the
 *    oActualVariationCount parameter, NULL for the oAxes and oValues
 *    parameters, and 0 for the other parameters. ATSUGetFontInstance
 *    returns the size to use for the oAxes and oValues arrays in the
 *    oActualVariationCount parameter. (2) Allocate enough space for
 *    arrays of the returned size, then call the ATSUGetFontInstance
 *    again, passing pointers to the arrays in the oAxes and oValues
 *    parameters. On return, the arrays contain the font variation axes
 *    and their corresponding values, respectively, for the font
 *    instance.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain instance information.
 *    
 *    iFontInstanceIndex:
 *      An index into an array of instances for the font. This index
 *      identifies the font instance to examine. Because this index is
 *      zero-based, you must pass a value between 0 and one less than
 *      the value produced in the oInstances parameter of the function
 *      ATSUCountFontInstances.
 *    
 *    iMaximumVariations:
 *      The maximum number of font variation axes to obtain for the
 *      font instance. Typically, this is equivalent to the number of
 *      ATSUFontVariationAxis and ATSUFontVariationValue values for
 *      which you have allocated memory in the oAxes and oValues
 *      parameters, respectively. To determine this value, see the
 *      Discussion.
 *    
 *    oAxes:
 *      On return, the array contains tags identifying the font
 *      variation axes that constitute the font instance. If you are
 *      uncertain of how much memory to allocate for this array, see
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      the Discussion.
=======
 *      the Discussion. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      the Discussion. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    oValues:
 *      On return, the array contains the defined values for the font
 *      variation axes produced in the oAxes array. If you are
 *      uncertain of how much memory to allocate for this array, see
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      the Discussion.
=======
 *      the Discussion. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      the Discussion. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    oActualVariationCount:
 *      On return, the actual number of font variation axes that
 *      constitute the font instance. This may be greater than the
 *      value you passed in the iMaximumVariations parameter.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontInstance( iFontID: ATSUFontID; iFontInstanceIndex: ItemCount; iMaximumVariations: ItemCount; oAxes: {variable-size-array} ATSUFontVariationAxisPtr { can be NULL }; oValues: {variable-size-array} ATSUFontVariationValuePtr { can be NULL }; var oActualVariationCount: ItemCount ): OSStatus; external name '_ATSUGetFontInstance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetFontInstanceNameCode()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontInstance( iFontID: ATSUFontID; iFontInstanceIndex: ItemCount; iMaximumVariations: ItemCount; oAxes: {variable-size-array} ATSUFontVariationAxisPtr { can be NULL }; oValues: {variable-size-array} ATSUFontVariationValuePtr { can be NULL }; var oActualVariationCount: ItemCount ): OSStatus; external name '_ATSUGetFontInstance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetFontInstanceNameCode()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetFontInstanceNameCode()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyVariationAxes, CTFontCopyVariation instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the name code for the font instance that matches an ASTUI
 *    font ID and font instance index value.
 *  
 *  Discussion:
 *    A font instance consists of a named set of values for each
 *    variation axis in a font. The ATSUGetFontInstanceNameCode
 *    function obtains the name code for the font instance that matches
 *    an ASTUI font ID and font instance index value. You can use the
 *    function ATSUFindFontName to obtain the localized name string for
 *    the name code produced by ATSUGetFontInstanceNameCode. You can
 *    obtain the font variation axis values for a font instance by
 *    calling the function ATSUGetFontInstance.
 *  
 *  Parameters:
 *    
 *    iFontID:
 *      The font for which to obtain a font instance name code.
 *    
 *    iInstanceIndex:
 *      An index to the font instance for which to obtain a name code.
 *      Because this index must be 0-based, you should pass a value
 *      between 0 and one less than the count produced by the function
 *      ATSUCountFontInstances.
 *    
 *    oNameCode:
 *      On return, the name code for the font instance. See the
 *      SFNTTypes.h header file for a definition of the FontNameCode
 *      type and a list of possible values.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontInstanceNameCode( iFontID: ATSUFontID; iInstanceIndex: ItemCount; var oNameCode: FontNameCode ): OSStatus; external name '_ATSUGetFontInstanceNameCode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetFontInstanceNameCode( iFontID: ATSUFontID; iInstanceIndex: ItemCount; var oNameCode: FontNameCode ): OSStatus; external name '_ATSUGetFontInstanceNameCode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}

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
