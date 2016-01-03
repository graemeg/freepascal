{
     File:       QD/ATSUnicodeObjects.h
 
     Contains:   ATSUI object manipulation functions.
 
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

unit ATSUnicodeObjects;
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
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
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
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
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
uses MacTypes,ATSUnicodeTypes,TextCommon,SFNTLayoutTypes;
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
{  ATSUI basic style functions                                                 }
{ ---------------------------------------------------------------------------- }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{
 *  ATSUCreateStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyGraphicsFont, CTFontCreateWithGraphicsFont,
 *    CTFontCreateWithPlatformFont, CTFontCreateWithQuickdrawInstance,
 *    CTFontCreateUIFontForLanguage, CTFontCreateCopyWithAttributes,
 *    CTFontCreateCopyWithSymbolicTraits, CTFontCreateCopyWithFamily,
 *    CTFontCreateForString, CTFontCreateWithName, or
 *    CTFontCreateWithFontDescriptor instead.
=======
{
 *  ATSUCreateStyle()
>>>>>>> graemeg/fixes_2_2
=======
{
 *  ATSUCreateStyle()
>>>>>>> origin/fixes_2_2
=======

{
 *  ATSUCreateStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyGraphicsFont, CTFontCreateWithGraphicsFont,
 *    CTFontCreateWithPlatformFont, CTFontCreateWithQuickdrawInstance,
 *    CTFontCreateUIFontForLanguage, CTFontCreateCopyWithAttributes,
 *    CTFontCreateCopyWithSymbolicTraits, CTFontCreateCopyWithFamily,
 *    CTFontCreateForString, CTFontCreateWithName, or
 *    CTFontCreateWithFontDescriptor instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Creates an ATSUStyle object with default settings.
 *  
 *  Discussion:
 *    ATSUStyle objects created by this function have a default set of
 *    values for all attributes. The attributes include settings such
 *    as font, point size, color and so on. You can change the
 *    attributes of a style object by calling the function
 *    ATSUSetAttributes. You can also change font features and
 *    variations set in an ATSUStyle by calling the functions
 *    ATSUSetFontFeatures and ATSUSetVariations, respectively.
 *    ATSUStyle objects are used by associating them with a run of
 *    characters in an ATSUTextLayout object. You can do this by
 *    calling functions such as ATSUSetRunStyle or
 *    ATSUCreateTextLayoutWithTextPtr. You are responsible for freeing
 *    memory assoicated with an ATSUStyle object by calling
 *    ATSUDisposeStyle.
 *  
 *  Parameters:
 *    
 *    oStyle:
 *      On return, a reference to an ATSUStyle object with default
 *      settings.
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
function ATSUCreateStyle( var oStyle: ATSUStyle ): OSStatus; external name '_ATSUCreateStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
 *  ATSUCreateAndCopyStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyGraphicsFont, CTFontCreateWithGraphicsFont,
 *    CTFontCreateWithPlatformFont, CTFontCreateWithQuickdrawInstance,
 *    CTFontCreateUIFontForLanguage, CTFontCreateCopyWithAttributes,
 *    CTFontCreateCopyWithSymbolicTraits, CTFontCreateCopyWithFamily,
 *    CTFontCreateForString, CTFontCreateWithName, or
 *    CTFontCreateWithFontDescriptor instead.
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
function ATSUCreateStyle( var oStyle: ATSUStyle ): OSStatus; external name '_ATSUCreateStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUCreateAndCopyStyle()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUCreateAndCopyStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCopyGraphicsFont, CTFontCreateWithGraphicsFont,
 *    CTFontCreateWithPlatformFont, CTFontCreateWithQuickdrawInstance,
 *    CTFontCreateUIFontForLanguage, CTFontCreateCopyWithAttributes,
 *    CTFontCreateCopyWithSymbolicTraits, CTFontCreateCopyWithFamily,
 *    CTFontCreateForString, CTFontCreateWithName, or
 *    CTFontCreateWithFontDescriptor instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Creates a new ATSUStyle object with the same attributes, font
 *    features, and font variation settings as the input style.
 *  
 *  Discussion:
 *    All attributes, font features, and font variation settings of the
 *    input ATSUStyle object are copied over to a newly created
 *    ATSUStyle object. Note that reference constants are not copied.
 *    You are responsible for freeing memory assoicated with the
 *    returned ATSUStyle object by calling ATSUDisposeStyle.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The ATSUStyle object you want to copy.
 *    
 *    oStyle:
 *      On return, a newly created ATSUStyle object. This will be an
 *      exact copy of iStyle, except for the reference constant (if
 *      set).
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
function ATSUCreateAndCopyStyle( iStyle: ATSUStyle; var oStyle: ATSUStyle ): OSStatus; external name '_ATSUCreateAndCopyStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}

=======


>>>>>>> graemeg/cpstrnew
{$endc} {not TARGET_CPU_64}
=======


{$endc} {not TARGET_CPU_64}
>>>>>>> origin/cpstrnew

{
 *  ATSUDisposeStyle()   *** DEPRECATED ***
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
function ATSUCreateAndCopyStyle( iStyle: ATSUStyle; var oStyle: ATSUStyle ): OSStatus; external name '_ATSUCreateAndCopyStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  ATSUDisposeStyle()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUDisposeStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Disposes of the memory associated with a style object.
 *  
 *  Discussion:
 *    The ATSUDisposeStyle function frees the memory associated with
 *    the specified style object and its internal structures, including
 *    style run attributes. It does not dispose of the memory pointed
 *    to by application-defined style run attributes or reference
 *    constants. You are responsible for doing so. You should call this
 *    function after calling the function ATSUDisposeTextLayout to
 *    dispose of any text layout objects associated with the style
 *    object. For best performance, once you create a style object, you
 *    should keep it and use it as often as needed. You should dispose
 *    of the style object only when it is no longer needed in your
 *    application.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style you want to dispose of.
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
function ATSUDisposeStyle( iStyle: ATSUStyle ): OSStatus; external name '_ATSUDisposeStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
 *  ATSUSetStyleRefCon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttributes instead.
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
function ATSUDisposeStyle( iStyle: ATSUStyle ): OSStatus; external name '_ATSUDisposeStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUSetStyleRefCon()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUSetStyleRefCon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets a reference constant for an ATSUStyle object.
 *  
 *  Discussion:
 *    Reference constants are any 32-bit value you wish to associate
 *    with an object. It can be a pointer to application-specific data,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    a integer value, or anything you like. If you copy or clear a
=======
 *    a SInt16 value, or anything you like. If you copy or clear a
>>>>>>> graemeg/fixes_2_2
=======
 *    a SInt16 value, or anything you like. If you copy or clear a
>>>>>>> origin/fixes_2_2
=======
 *    a integer value, or anything you like. If you copy or clear a
>>>>>>> origin/fixes_2.4
 *    style object that contains a reference constant, the reference
 *    constant is neither copied nor removed. To obtain the reference
 *    constant for a particular ATSUStyle object after it has been set,
 *    use the function ATSUGetStyleRefCon.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      An ATSUStyle object you want to set the reference constant for.
 *    
 *    iRefCon:
 *      Any arbitrary 32-bit value containing or referring to
 *      application-specific data.
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
function ATSUSetStyleRefCon( iStyle: ATSUStyle; iRefCon: URefCon ): OSStatus; external name '_ATSUSetStyleRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetStyleRefCon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttributes instead.
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
function ATSUSetStyleRefCon( iStyle: ATSUStyle; iRefCon: URefCon ): OSStatus; external name '_ATSUSetStyleRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetStyleRefCon()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetStyleRefCon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns the reference constant for an ATSUStyle object.
 *  
 *  Discussion:
 *    Together with ATSUSetStyleRefCon, this function provides a
 *    mechanism for keeping application-specific data associated with
 *    ATSUStyle objects. Note that if an ATSUStyle object is copied or
 *    cleared, its associated reference constant, if any, is not copied
 *    or cleared.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style object for which to obtain application-specific data.
 *    
 *    oRefCon:
 *      On return, the reference constant for iStyle.
 *  
 *  Result:
 *    On success, noErr is returned. If no reference constant is set in
 *    iStyle, paramErr is returned. See MacErrors.h for other possible
 *    error codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetStyleRefCon( iStyle: ATSUStyle; var oRefCon: URefCon ): OSStatus; external name '_ATSUGetStyleRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUGetStyleRefCon( iStyle: ATSUStyle; var oRefCon: UInt32 ): OSStatus; external name '_ATSUGetStyleRefCon';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetStyleRefCon( iStyle: ATSUStyle; var oRefCon: URefCon ): OSStatus; external name '_ATSUGetStyleRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI style comparison                                                      }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCompareStyles()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateMatchingFontDescriptor instead.
<<<<<<< HEAD
=======
 *  ATSUCompareStyles()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCompareStyles()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Compares two ATSUStyleObjects.
 *  
 *  Discussion:
 *    The ATSUCompareStyles function compares the contents of two style
 *    objects, including their style attributes, font features, and
 *    font variations. It does not consider reference constants or
 *    application-defined style attributes in the comparison. Note that
 *    order is important, as the ATSUStyleComparison constants that can
 *    be returned indicate "contains" vs. "contained by" based on which
 *    style is considered first in the comparsion.
 *  
 *  Parameters:
 *    
 *    iFirstStyle:
 *      The first style to be compared.
 *    
 *    iSecondStyle:
 *      The second style to be compared.
 *    
 *    oComparison:
 *      On return, the value contains the results of the comparison and
 *      indicates whether the two style objects are the same,
 *      different, or if one is a subset of the other. See the
 *      definition of the ATSUStyleComparison type for more information
 *      on possible values returned for this parameter.
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
function ATSUCompareStyles( iFirstStyle: ATSUStyle; iSecondStyle: ATSUStyle; var oComparison: ATSUStyleComparison ): OSStatus; external name '_ATSUCompareStyles';
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
function ATSUCompareStyles( iFirstStyle: ATSUStyle; iSecondStyle: ATSUStyle; var oComparison: ATSUStyleComparison ): OSStatus; external name '_ATSUCompareStyles';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI style attribute manipulation                                          }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCopyAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
<<<<<<< HEAD
=======
 *  ATSUCopyAttributes()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCopyAttributes()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Copies attributes from one style to another.
 *  
 *  Discussion:
 *    There are three types of settings in a style: attributes, font
 *    features, and font variations. This function copies only the
 *    first. To copy all three types of settings, use the function
 *    ATSUCreateAndCopyStyle. Also note that this function does not
 *    copy reference constants.
 *  
 *  Parameters:
 *    
 *    iSourceStyle:
 *      The style whose attributes you are copying from.
 *    
 *    iDestinationStyle:
 *      The style whose attributes you are copying to.
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
function ATSUCopyAttributes( iSourceStyle: ATSUStyle; iDestinationStyle: ATSUStyle ): OSStatus; external name '_ATSUCopyAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUOverwriteAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
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
function ATSUCopyAttributes( iSourceStyle: ATSUStyle; iDestinationStyle: ATSUStyle ): OSStatus; external name '_ATSUCopyAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUOverwriteAttributes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUOverwriteAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Copies to a destination style object the nondefault style
 *    attribute settings of a source style object.
 *  
 *  Discussion:
 *    The ATSUOverwriteAttributes function copies all nondefault style
 *    attribute values from a source style object to a destination
 *    style object. The source object's nondefault values are applied
 *    to the destination object whether or not the destination object
 *    also has nondefault values for the copied attributes. All other
 *    settings in the destination style object are left unchanged.
 *    ATSUOverwriteAttributes does not copy the contents of memory
 *    referenced by pointers within custom style attributes or within
 *    reference constants. You are responsible for ensuring that this
 *    memory remains valid until both the source and destination style
 *    objects are disposed of. To create a style object that contains
 *    all the contents of another style object, call the function
 *    ATSUCreateAndCopyStyle. To copy all the style attributes
 *    (including any default settings) of a style object into an
 *    existing style object, call the function ATSUCopyAttributes. To
 *    copy style attributes that are set in the source but not in the
 *    destination style object, call the function
 *    ATSUUnderwriteAttributes.
 *  
 *  Parameters:
 *    
 *    iSourceStyle:
 *      An ATSUStyle value specifying the style object from which to
 *      copy nondefault style attributes.
 *    
 *    iDestinationStyle:
 *      An ATSUStyle value specifying the style object containing the
 *      style attributes to be overwritten.
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
function ATSUOverwriteAttributes( iSourceStyle: ATSUStyle; iDestinationStyle: ATSUStyle ): OSStatus; external name '_ATSUOverwriteAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUUnderwriteAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
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
function ATSUOverwriteAttributes( iSourceStyle: ATSUStyle; iDestinationStyle: ATSUStyle ): OSStatus; external name '_ATSUOverwriteAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUUnderwriteAttributes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUUnderwriteAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Copies to a destination style object only those nondefault style
 *    attribute settings of a source style object that are at default
 *    settings in the destination object.
 *  
 *  Discussion:
 *    The ATSUUnderwriteAttributes function copies to a destination
 *    style object only those nondefault style attribute values of a
 *    source style object that are not currently set in a destination
 *    style object. Note that the corresponding value in the
 *    destination object must not be set in order for a copied value to
 *    be applied. All other quantities in the destination style object
 *    are left unchanged. ATSUUnderwriteAttributes does not copy the
 *    contents of memory referenced by pointers within custom style
 *    attributes or within reference constants. You are responsible for
 *    ensuring that this memory remains valid until both the source and
 *    destination style objects are disposed of. To create a style
 *    object that contains all the contents of another style object,
 *    call the function ATSUCreateAndCopyStyle. To copy all the style
 *    attributes (including any default settings) of a style object
 *    into an existing style object, call the function
 *    ATSUCopyAttributes. To copy style attributes that are set in the
 *    source whether or not they are set in the destination style
 *    object, call the function ATSUOverwriteAttributes.
 *  
 *  Parameters:
 *    
 *    iSourceStyle:
 *      An ATSUStyle value specifying the style object from which to
 *      copy nondefault style attributes.
 *    
 *    iDestinationStyle:
 *      An ATSUStyle value specifying the style object containing style
 *      attribute values to be set.
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
function ATSUUnderwriteAttributes( iSourceStyle: ATSUStyle; iDestinationStyle: ATSUStyle ): OSStatus; external name '_ATSUUnderwriteAttributes';
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
function ATSUUnderwriteAttributes( iSourceStyle: ATSUStyle; iDestinationStyle: ATSUStyle ): OSStatus; external name '_ATSUUnderwriteAttributes';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  Empty ATSUI styles                                                          }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUClearStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
<<<<<<< HEAD
=======
 *  ATSUClearStyle()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUClearStyle()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Restores default values to a style object.
 *  
 *  Discussion:
 *    Clears a style object of all style attributes (including any
 *    application-defined attributes), font features, and font
 *    variations and returns these values to their default settings. To
 *    clear attributes, font features, or font variations individually,
 *    use the functions ATSUClearAttributes, ATSUClearFontVariations,
 *    or ATSUClearFontFeatures, respectively. Note that ATSUClearStyle
 *    does not affect Reference constants.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style to be cleared.
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
function ATSUClearStyle( iStyle: ATSUStyle ): OSStatus; external name '_ATSUClearStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUStyleIsEmpty()   *** DEPRECATED ***
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
function ATSUClearStyle( iStyle: ATSUStyle ): OSStatus; external name '_ATSUClearStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUStyleIsEmpty()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUStyleIsEmpty()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Indicates whether a style object contains only default values.
 *  
 *  Discussion:
 *    You can call the ATSUStyleIsEmpty function to determine whether a
 *    style object contains only default values for style attributes,
 *    font features, and font variations. ATSUStyleIsEmpty does not
 *    consider reference constants in its evaluation.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      An ATSUStyle value specifying the style object to examine.
 *    
 *    oIsClear:
 *      On return, the value is set to true if the style object
 *      contains only default values for style attributes, font
 *      features, and font variations. If false , the style object
 *      contains one or more nondefault values for style attributes,
 *      font features, or font variations. Reference constants do not
 *      affect this result.
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
function ATSUStyleIsEmpty( iStyle: ATSUStyle; var oIsClear: Boolean ): OSStatus; external name '_ATSUStyleIsEmpty';
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
function ATSUStyleIsEmpty( iStyle: ATSUStyle; var oIsClear: Boolean ): OSStatus; external name '_ATSUStyleIsEmpty';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI style attribute getters and setters                                   }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCalculateBaselineDeltas()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
<<<<<<< HEAD
=======
 *  ATSUCalculateBaselineDeltas()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCalculateBaselineDeltas()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the optimal baseline positions for glyphs in a style run.
 *  
 *  Discussion:
 *    Depending on the writing system, a baseline may be above, below,
 *    or through the centers of glyphs. In general, a style run has a
 *    default baseline, to which all glyphs are visually aligned when
 *    the text is laid out. For example, in a run of Roman text, the
 *    default baseline is the Roman baseline, upon which glyphs sit
 *    (except for descenders, which extend below the baseline). You can
 *    call the ATSUCalculateBaselineDeltas function to obtain the
 *    distances from a specified baseline type to that of other
 *    baseline types for a given style object.
 *    ATSUCalculateBaselineDeltas takes into account font and text size
 *    when performing these calculations. ATSUI uses these distances to
 *    determine the cross-stream shifting to apply when aligning glyphs
 *    in a style run. You can use the resulting array to set or obtain
 *    the optimal baseline positions of glyphs in a style run. You can
 *    also set various baseline values to create special effects such
 *    as drop capitals. The functions ATSUSetLineControls and
 *    ATSUSetLayoutControls allow you to set baseline offset values at
 *    the line or layout level, respectively, using the
 *    kATSULineBaselineValuesTag control attribute tag.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      An ATSUStyle value specifying the style object to examine.
 *    
 *    iBaselineClass:
 *      A BslnBaselineClass constant identifying the primary baseline
 *      from which to measure other baselines. See SFNTLayoutTypes.h
 *      for an enumeration of possible values. Pass the constant
 *      kBSLNNoBaselineOverride to use the standard baseline value from
 *      the current font.
 *    
 *    oBaselineDeltas:
 *      On return, an array that contains baseline offsets, specifying
 *      distances measured in points, from the default baseline to each
 *      of the other baseline types in the style object. Positive
 *      values indicate baselines above the default baseline and
 *      negative values indicate baselines below it. See
 *      SFNTLayoutTypes.h for a description of the BslnBaselineRecord
 *      type.
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
function ATSUCalculateBaselineDeltas( iStyle: ATSUStyle; iBaselineClass: BslnBaselineClass; oBaselineDeltas: BslnBaselineRecord ): OSStatus; external name '_ATSUCalculateBaselineDeltas';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}
=======
>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}
=======
>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{
 *  ATSUSetAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateWithNameAndSize,
 *    CTFontDescriptorCreateWithAttributes, or
 *    CTFontDescriptorCreateCopyWithAttributes instead.
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
function ATSUCalculateBaselineDeltas( iStyle: ATSUStyle; iBaselineClass: BslnBaselineClass; oBaselineDeltas: BslnBaselineRecord ): OSStatus; external name '_ATSUCalculateBaselineDeltas';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  ATSUSetAttributes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUSetAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateWithNameAndSize,
 *    CTFontDescriptorCreateWithAttributes, or
 *    CTFontDescriptorCreateCopyWithAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets style attribute values in a style object.
 *  
 *  Discussion:
 *    Style attributes are a collection of values and settings that
 *    specify information about a style such as font, point size, and
 *    color. To specify a style attribute, ATSUI uses a "triple"
 *    consisting of (1) an attribute tag, (2) a value for that tag, and
 *    (3) the size of the value. For a list of possible tags and their
 *    default values, see the ATSUI documentation, or the definition of
 *    ATSUAttributeTag elsewhere in this header file. When you call
 *    ATSUSetAttributes, any style attributes that you do not set
 *    retain their previous values. To set font features and font
 *    variations, call the functions ATSUSetFontFeatures and
 *    ATSUSetVariations, respectively.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      A style in which to set attributes.
 *    
 *    iAttributeCount:
 *      An ItemCount value specifying the number of attributes to set.
 *      This value should correspond to the number of elements in the
 *      iTag, iValueSize, and iValue arrays.
 *    
 *    iTag:
 *      An array of attribute tags. The number of elements in this
 *      array must not be less than iAttributeCount. Each element in
 *      the array must contain a valid style attribute tag (see the
 *      definition of ATSUAttributeTag for possible values).
 *    
 *    iValueSize:
 *      An array of ByteCount values. The number of elements in this
 *      array must not be less than iAttributeCount. Each ByteCount
 *      value corresoponds to the size of an element referred to by a
 *      pointer in the iValue array.
 *    
 *    iValue:
 *      An array of pointers of type ATSUAttributeValuePtr. Each
 *      pointer referrs to a value that corresponds to a tag specified
 *      by the iTag array. The size of the data referred to is
 *      determined by a corresponding element in the iValueSize array.
 *      The number of elements in this array must not be less than
 *      iAttributeCount.
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
function ATSUSetAttributes( iStyle: ATSUStyle; iAttributeCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr; {const} iValueSize: {variable-size-array} ByteCountPtr; {const} iValue: {variable-size-array} ATSUAttributeValuePtrPtr ): OSStatus; external name '_ATSUSetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
 *  ATSUGetAttribute()   *** DEPRECATED ***
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
function ATSUSetAttributes( iStyle: ATSUStyle; iAttributeCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr; {const} iValueSize: {variable-size-array} ByteCountPtr; {const} iValue: {variable-size-array} ATSUAttributeValuePtrPtr ): OSStatus; external name '_ATSUSetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUGetAttribute()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetAttribute()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttribute instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a single attribute value for a style object.
 *  
 *  Discussion:
 *    The ATSUGetAttribute function obtains the value of a specified
 *    style attribute for a given style object. Before calling
 *    ATSUGetAttribute, you should call the function
 *    ATSUGetAllAttributes to obtain an array of nondefault style
 *    attribute tags and value sizes for the style object. You can then
 *    pass ATSUGetAttribute the tag and value size for the attribute
 *    value to obtain. This function may return kATSUNotSetErr for some
 *    attributes that have not been set to a non-default via a call to
 *    ATSUSetAttributes.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style object you with to retrieve an attribute value from.
 *    
 *    iTag:
 *      The tag you wish to obtain the value of.
 *    
 *    iExpectedValueSize:
 *      The size of the buffer pointed to by oValue.
 *    
 *    oValue:
 *      On input, a buffer you have allocated to retain the value of
 *      the specified attribute. On return, the value of the requested
 *      attribute will be placed here. You may pass NULL for this
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      parameter.
 *    
 *    oActualValueSize:
 *      On return, the actual number of bytes written to oValue is
 *      placed here. You may pass NULL for this parameter.
=======
=======
>>>>>>> origin/fixes_2_2
 *      parameter. can be NULL
 *    
 *    oActualValueSize:
 *      On return, the actual number of bytes written to oValue is
 *      placed here. You may pass NULL for this parameter. can be NULL
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *      parameter.
 *    
 *    oActualValueSize:
 *      On return, the actual number of bytes written to oValue is
 *      placed here. You may pass NULL for this parameter.
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
function ATSUGetAttribute( iStyle: ATSUStyle; iTag: ATSUAttributeTag; iExpectedValueSize: ByteCount; oValue: ATSUAttributeValuePtr { can be NULL }; oActualValueSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetAllAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttributes instead.
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
function ATSUGetAttribute( iStyle: ATSUStyle; iTag: ATSUAttributeTag; iExpectedValueSize: ByteCount; oValue: ATSUAttributeValuePtr { can be NULL }; oActualValueSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetAllAttributes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetAllAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCopyAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains an array of style attribute tags and value sizes for a
 *    style object.
 *  
 *  Discussion:
 *    This function returns information as to which attributes have had
 *    non-default values set in a particular ATSUStyle object. It will
 *    also return the size in bytes of the values of these attributes.
 *    Using this information, you can then call ATSUGetAttribute to
 *    obtain the value of a given attribute. Typically you use the
 *    function ATSUGetAllAttributes by calling it twice, as follows:
 *    (1) Pass a reference to the style object to examine in the iStyle
 *    parameter, a valid pointer to an ItemCount value in the
 *    oTagValuePairCount parameter, NULL for the oAttributeInfoArray
 *    parameter, and 0 for the iTagValuePairArraySize parameter.
 *    ATSUGetAllAttributes returns the size of the tag and value-size
 *    arrays in the oTagValuePairCount parameter. (2) Allocate enough
 *    space for an array of the returned size, then call the
 *    ATSUGetAllAttributes function again, passing a valid pointer in
 *    the oAttributeInfoArray parameter. On return, the pointer refers
 *    to an array of the style attribute tag and value-size pairs
 *    contained in the style object.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      The style object you wish to retrieve a list of attribute tags
 *      from.
 *    
 *    oAttributeInfoArray:
 *      On return, an array of ATSUAttributeInfo structures. Each
 *      structure contains information about an attribute in iStyle
 *      that has a non-default value. You must allocate space for this
 *      array. If you are unsure how much space to allocate, you may
 *      pass NULL for this parameter and use the oTagValuePairCount
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      parameter to determine how much space to allocate.
 *    
 *    iTagValuePairArraySize:
 *      The size of the array you allocated and are passing in for the
 *      oAttributeInfoArray parameter.
 *    
 *    oTagValuePairCount:
 *      On return, the number of attributes whose information was
 *      stored in the oAttributeInfoArray parameter.
=======
=======
>>>>>>> origin/fixes_2_2
 *      parameter to determine how much space to allocate. can be NULL
=======
 *      parameter to determine how much space to allocate.
>>>>>>> origin/fixes_2.4
 *    
 *    iTagValuePairArraySize:
 *      The size of the array you allocated and are passing in for the
 *      oAttributeInfoArray parameter.
 *    
 *    oTagValuePairCount:
 *      On return, the number of attributes whose information was
<<<<<<< HEAD
 *      stored in the oAttributeInfoArray parameter. can be NULL
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *      stored in the oAttributeInfoArray parameter.
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
function ATSUGetAllAttributes( iStyle: ATSUStyle; oAttributeInfoArray: {variable-size-array} ATSUAttributeInfoPtr { can be NULL }; iTagValuePairArraySize: ItemCount; oTagValuePairCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAllAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUClearAttributes()   *** DEPRECATED ***
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
function ATSUGetAllAttributes( iStyle: ATSUStyle; oAttributeInfoArray: {variable-size-array} ATSUAttributeInfoPtr { can be NULL }; iTagValuePairArraySize: ItemCount; oTagValuePairCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAllAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUClearAttributes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUClearAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Restores default values to the specified style attributes of a
 *    style object.
 *  
 *  Discussion:
 *    Removes those style attribute values identified by the tag
 *    constants in the iTag array and replaces them with the default
 *    values. For a list of possible tags and their default values, see
 *    the ATSUI documentation, or the definition of ATSUAttributeTag
 *    elsewhere in this header file. If you specify that any currently
 *    unset attribute values be removed, ATSUClearAttributes does not
 *    return an error. Note this function only deals with attributes.
 *    To remove all previously set style attributes as well as font
 *    features and font variations from a style object, call the
 *    function ATSUClearStyle.
 *  
 *  Parameters:
 *    
 *    iStyle:
 *      A style whose attributes you want to clear.
 *    
 *    iTagCount:
 *      The number of tags you are passing in via the iTag parameter.
 *      Pass kATSUClearAll to clear all attributes.
 *    
 *    iTag:
 *      An array of ATSUAttributeTag indicating which attributes to
 *      clear. You may pass NULL for this parameter if you are passing
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      kATSUClearAll for the iTagCount parameter.
=======
 *      kATSUClearAll for the iTagCount parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      kATSUClearAll for the iTagCount parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      kATSUClearAll for the iTagCount parameter.
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
function ATSUClearAttributes( iStyle: ATSUStyle; iTagCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr { can be NULL } ): OSStatus; external name '_ATSUClearAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUClearAttributes( iStyle: ATSUStyle; iTagCount: ItemCount; iTag: ATSUAttributeTagPtr ): OSStatus; external name '_ATSUClearAttributes';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUClearAttributes( iStyle: ATSUStyle; iTagCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr { can be NULL } ): OSStatus; external name '_ATSUClearAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI basic text layout functions                                           }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCreateTextLayout()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTTypesetterCreateWithAttributedString,
 *    CTTypesetterCreateWithAttributedStringAndOptions,
 *    CTLineCreateWithAttributedString, CTLineCreateTruncatedLine, or
 *    CTLineCreateJustifiedLine instead.
<<<<<<< HEAD
=======
 *  ATSUCreateTextLayout()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCreateTextLayout()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Creates an opaque text layout object containing only default text
 *    layout attributes.
 *  
 *  Discussion:
 *    This function creates a empty text layout object that has no
 *    styles or text buffer associated with it. Most ATSUI functions
 *    that operate on text layout objects require that the objects be
 *    associated with style information and text. To associate style
 *    objects and text with an empty text layout object, you can call
 *    the functions ATSUSetRunStyle and ATSUSetTextPointerLocation .
 *    Or, to create a text layout object and associate style objects
 *    and text with it at the same time, you can call the function
 *    ATSUCreateTextLayoutWithTextPtr. To provide nondefault line or
 *    layout attributes for a text layout object, you can call the
 *    functions ATSUSetLineControls or ATSUSetLayoutControls . After
 *    setting text attributes, call ATSUDrawText to draw the text. Text
 *    layout objects are readily reusable and should be cached for
 *    later use, if possible. You can reuse a text layout object even
 *    if the text associated with it is altered. Call the functions
 *    ATSUSetTextPointerLocation, ATSUTextDeleted, or ATSUTextInserted
 *    to manage the altered text.
 *  
 *  Parameters:
 *    
 *    oTextLayout:
 *      On return, the value refers to an empty text layout object.
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
function ATSUCreateTextLayout( var oTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCreateTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUCreateAndCopyTextLayout()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTTypesetterCreateWithAttributedString,
 *    CTTypesetterCreateWithAttributedStringAndOptions,
 *    CTLineCreateWithAttributedString, CTLineCreateTruncatedLine, or
 *    CTLineCreateJustifiedLine instead.
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
function ATSUCreateTextLayout( var oTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCreateTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUCreateAndCopyTextLayout()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUCreateAndCopyTextLayout()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTTypesetterCreateWithAttributedString,
 *    CTTypesetterCreateWithAttributedStringAndOptions,
 *    CTLineCreateWithAttributedString, CTLineCreateTruncatedLine, or
 *    CTLineCreateJustifiedLine instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Creates a copy of a text layout object.
 *  
 *  Discussion:
 *    This function creates a copy of the source text layout object's
 *    style runs (including references to the associated text buffer
 *    and style objects), line attributes, layout attributes, and
 *    layout caches. ATSUCreateAndCopyTextLayout does not copy
 *    reference constants. To create a text layout object without
 *    copying a source object, you can the function
 *    ATSUCreateTextLayout or the function
 *    ATSUCreateTextLayoutWithTextPtr.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout to be copied.
 *    
 *    oTextLayout:
 *      On return, a reference to a layout object which is a copy of
 *      iTextLayout.
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
function ATSUCreateAndCopyTextLayout( iTextLayout: ATSUTextLayout; var oTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCreateAndCopyTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}
=======

>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}

{
 *  ATSUCreateTextLayoutWithTextPtr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTTypesetterCreateWithAttributedString,
 *    CTTypesetterCreateWithAttributedStringAndOptions,
 *    CTLineCreateWithAttributedString, CTLineCreateTruncatedLine, or
 *    CTLineCreateJustifiedLine instead.
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
function ATSUCreateAndCopyTextLayout( iTextLayout: ATSUTextLayout; var oTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCreateAndCopyTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  ATSUCreateTextLayoutWithTextPtr()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUCreateTextLayoutWithTextPtr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTTypesetterCreateWithAttributedString,
 *    CTTypesetterCreateWithAttributedStringAndOptions,
 *    CTLineCreateWithAttributedString, CTLineCreateTruncatedLine, or
 *    CTLineCreateJustifiedLine instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Creates an opaque text layout object containing default text
 *    layout attributes as well as associated text and text styles.
 *  
 *  Discussion:
 *    This function creates a text layout object and associates the
 *    specified text buffer and style runs with it. All layout
 *    attributes are set to their default values. To provide nondefault
 *    line or layout attributes for a text layout object, you can call
 *    the functions ATSUSetLineControls or ATSUSetLayoutControls. After
 *    setting text attributes, call ATSUDrawText to draw the text.
 *    Because the only way that ATSUI interacts with text is via the
 *    memory references you associate with a text layout object, you
 *    are responsible for keeping these references updated through use
 *    of the functions ATSUTextInserted, ATSUTextDeleted,
 *    ATSUTextMoved, and ATSUSetTextPointerLocation. Note that, because
 *    ATSUI objects retain state information, doing superfluous calling
 *    can degrade performance. For example, you could call
 *    ATSUSetTextPointerLocation rather than ATSUTextInserted when the
 *    user inserts text, but there would be a performance penalty, as
 *    all the layout caches are flushed when you call
 *    ATSUSetTextPointerLocation , rather than just the affected ones.
 *    Text layout objects are readily reusable and should themselves be
 *    cached for later use, if possible. Text objects are thread-safe
 *    starting with ATSUI version 2.4.
 *  
 *  Parameters:
 *    
 *    iText:
 *      A text buffer containing UTF-16Ðencoded text. ATSUI associates
 *      this buffer with the new text layout object and analyzes the
 *      complete text of the buffer when obtaining the layout context
 *      for the current text range. Thus, for paragraph-format text, if
 *      you specify a buffer containing less than a complete paragraph,
 *      some of ATSUI's layout results are not guaranteed to be
 *      accurate. For example, with a buffer of less than a full
 *      paragraph, ATSUI can neither reliably obtain the context for
 *      bidirectional processing nor reliably generate accent
 *      attachments and ligature formations for Roman text.
 *    
 *    iTextOffset:
 *      The offset from the beginning of the text buffer to the first
 *      character of the range to include in the layout. To indicate
 *      that the specified text range starts at the beginning of the
 *      text buffer, you can pass the constant kATSUFromTextBeginning.
 *      To specify the entire text buffer, pass kATSUFromTextBeginning
 *      in this parameter and kATSUToTextEnd in the iTextLength
 *      parameter. For best results, use one layout for each paragraph
 *      within the text buffer.
 *    
 *    iTextLength:
 *      The length of the text range. Note that the sum of iTextOffset
 *      and iTextLength must be less than or equal to the value of the
 *      iTextTotalLength parameter. If you want the range of text to
 *      extend to the end of the text buffer, you can pass the constant
 *      kATSUToTextEnd. For best results, use one layout for each
 *      paragraph within the text buffer.
 *    
 *    iTextTotalLength:
 *      The length of the entire text buffer referred to by iText. This
 *      value should be greater than or equal to the range of text
 *      defined by the iTextLength parameter.
 *    
 *    iNumberOfRuns:
 *      The number of text style runs you want to define within the
 *      overall text range. The number of style objects and style run
 *      lengths passed in the iStyles and iRunLengths parameters,
 *      respectively, should be equal to the number of runs specified
 *      here.
 *    
 *    iRunLengths:
 *      An array providing ATSUI with the lengths of each of the text's
 *      style runs. You can pass kATSUToTextEnd for the last style run
 *      length if you want the style run to extend to the end of the
 *      text range. If the sum of the style run lengths is less than
 *      the total length of the text range, the remaining characters
 *      are assigned to the last style run.
 *    
 *    iStyles:
 *      An array of styles, each corresponding to a style run defined
 *      in iRunLengths. The same ATSUStyle object may be referred to
 *      more than once in this array. The number of elements in this
 *      array must be equal to the value specified by the iNumberOfRuns
 *      parameter.
 *    
 *    oTextLayout:
 *      A valid pointer to an ATSUTextLayout value. On return, the
 *      value refers to the newly created text layout object.
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
function ATSUCreateTextLayoutWithTextPtr( iText: ConstUniCharArrayPtr; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount; iNumberOfRuns: ItemCount; {const} iRunLengths: {variable-size-array} UniCharCountPtr; iStyles: {variable-size-array} ATSUStylePtr; var oTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCreateTextLayoutWithTextPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUClearLayoutCache()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
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
function ATSUCreateTextLayoutWithTextPtr( iText: ConstUniCharArrayPtr; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount; iNumberOfRuns: ItemCount; {const} iRunLengths: {variable-size-array} UniCharCountPtr; iStyles: {variable-size-array} ATSUStylePtr; var oTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCreateTextLayoutWithTextPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUClearLayoutCache()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUClearLayoutCache()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Clears the layout cache of a line or an entire text layout object.
 *  
 *  Discussion:
 *    The layout cache contains all the layout information ATSUI
 *    calculates and needs to draw a range of text in a text layout
 *    object. This includes caret positions, the memory locations of
 *    glyphs, and other information needed to lay out the glyphs. ATSUI
 *    uses information in the layout cache to avoid laying out the text
 *    again, thereby improving performance. When you clear the layout
 *    cache of a line or block of text, ATSUI takes longer to redraw a
 *    line, since it must perform the calculations that support glyph
 *    layout again. You should call the function ATSUClearLayoutCache
 *    when you need to decrease the amount of memory your application
 *    uses. This function reclaims memory at the cost of optimal
 *    performance. By default, the ATSUClearLayoutCache function
 *    removes the layout cache of a single line. To clear the layout
 *    cache for multiple lines, you should call ATSUClearLayoutCache
 *    for each line. To clear the layout cache of an entire text layout
 *    object, pass the constant kATSUFromTextBeginning in the
 *    iLineStart parameter. Note that ATSUClearLayoutCache does not
 *    produce a function error if lines do not have a layout cache. The
 *    ATSUClearLayoutCache function flushes the layout cache but does
 *    not alter previously set text layout attributes, soft line break
 *    positions, or the text memory location. If you do not want to
 *    retain these values, you should dispose of the text layout object
 *    by calling the ATSUDisposeTextLayout function.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout for which to clear the layout caches.
 *    
 *    iLineStart:
 *      The offset from the beginning of the text buffer to the
 *      beginning of the line for which to discard the layout cache. If
 *      the range of text spans multiple lines, you should call
 *      ATSUClearLayoutCache for each line, passing the offset
 *      corresponding to the beginning of the new line to draw with
 *      each call. To clear the layout cache of the entire text layout
 *      object, you can pass the constant kATSUFromTextBeginning.
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
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUClearLayoutCache( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset ): OSStatus; external name '_ATSUClearLayoutCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUDisposeTextLayout()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUClearLayoutCache( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset ): OSStatus; external name '_ATSUClearLayoutCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUDisposeTextLayout()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUDisposeTextLayout()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFRelease instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Disposes of the memory associated with a text layout object.
 *  
 *  Discussion:
 *    This function frees the memory associated with the specified text
 *    layout object and its internal structures, including line and
 *    layout control attributes, style runs, and soft line breaks.
 *    ATSUDisposeTextLayout does not dispose of any memory that may be
 *    allocated for the text buffer, style objects, or reference
 *    constants associated with the text layout object. You are
 *    responsible for doing so. For best performance, text layout
 *    objects are readily reusable and should be cached for later use,
 *    if possible. You can reuse a text layout object even if the text
 *    associated with it is altered. Call the functions
 *    ATSUSetTextPointerLocation, ATSUTextDeleted, or ATSUTextInserted
 *    to manage the altered text, rather than disposing of the text
 *    layout object and creating a new one.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout object to dispose of.
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
function ATSUDisposeTextLayout( iTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUDisposeTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUSetTextLayoutRefCon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CFAttributedStringSetAttributes instead.
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
function ATSUDisposeTextLayout( iTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUDisposeTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUSetTextLayoutRefCon()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUSetTextLayoutRefCon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CFAttributedStringSetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets application-specific data for a text layout object.
 *  
 *  Discussion:
 *    This function associates a reference constant (that is,
 *    application-specific data) with a text layout object. You might
 *    typically use ATSUSetTextLayoutRefCon to track user preferences
 *    that can effect layout, for example. If you copy or clear a text
 *    layout object containing a reference constant, the reference
 *    constant is not copied or removed. When you dispose of a text
 *    layout object that contains a reference constant, you are
 *    responsible for freeing any memory allocated for the reference
 *    constant. Calling the function ATSUDisposeTextLayout does not do
 *    so.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      A layout for which you wish to set a reference constant.
 *    
 *    iRefCon:
 *      Any arbitrary 32-bit value you wish to store in association
 *      with iTextLayout.
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
function ATSUSetTextLayoutRefCon( iTextLayout: ATSUTextLayout; iRefCon: URefCon ): OSStatus; external name '_ATSUSetTextLayoutRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
 *  ATSUGetTextLayoutRefCon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CFAttributedStringSetAttributes instead.
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
function ATSUSetTextLayoutRefCon( iTextLayout: ATSUTextLayout; iRefCon: URefCon ): OSStatus; external name '_ATSUSetTextLayoutRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUGetTextLayoutRefCon()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetTextLayoutRefCon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CFAttributedStringSetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains application-specific data for a text layout object.
 *  
 *  Discussion:
 *    This function obtains a reference constant (that is,
 *    application-specific data) associated with a text layout object.
 *    To associate a reference constant with a text layout object, call
 *    the function ATSUSetTextLayoutRefCon.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      A layout for which you wish to retreive the reference constant.
 *    
 *    oRefCon:
 *      On return, the reference constant associated with iTextLayout.
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
function ATSUGetTextLayoutRefCon( iTextLayout: ATSUTextLayout; var oRefCon: URefCon ): OSStatus; external name '_ATSUGetTextLayoutRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUGetTextLayoutRefCon( iTextLayout: ATSUTextLayout; var oRefCon: UInt32 ): OSStatus; external name '_ATSUGetTextLayoutRefCon';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUGetTextLayoutRefCon( iTextLayout: ATSUTextLayout; var oRefCon: URefCon ): OSStatus; external name '_ATSUGetTextLayoutRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI text buffer manipulation                                              }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUSetTextPointerLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
<<<<<<< HEAD
=======
 *  ATSUSetTextPointerLocation()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUSetTextPointerLocation()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Associates a text buffer with a text layout object or updates
 *    previously associated text.
 *  
 *  Discussion:
 *    For ATSUI to render your text, you must associate the text with
 *    both a text layout object and style information. Some functions,
 *    such as ATSUCreateTextLayoutWithTextPtr, create a text layout
 *    object and associate text with it concurrently. However, if you
 *    use the function ATSUCreateTextLayout to create a text layout
 *    object, you must assign text to the object prior to attempting
 *    most ATSUI operations. You can use the function
 *    ATSUSetTextPointerLocation to associate text with a layout
 *    object. When you call this function, you are both assigning a
 *    text buffer to a text layout object and specifying the current
 *    text subrange within the buffer to include in the layout. If
 *    there is already text associated with a text layout object,
 *    calling ATSUSetTextPointerLocation overrides the previously
 *    associated text, as well as clearing the object's layout caches.
 *    You would typically only call this function for a text layout
 *    object with existing associated text if either (a) both the
 *    buffer itself is relocated and a subrange of the buffer's text is
 *    deleted or inserted or (b) when associating an entirely different
 *    buffer with a text layout object. Note that, because ATSUI
 *    objects retain state, doing superfluous calling can degrade
 *    performance. For example, you could call
 *    ATSUSetTextPointerLocation rather than ATSUTextInserted when the
 *    user simply inserts a subrange of text within a text buffer, but
 *    there would be a performance penalty, as all the layout caches
 *    are flushed by ATSUSetTextPointerLocation, rather than just the
 *    affected ones. Similarly, you should not call
 *    ATSUSetTextPointerLocation, when an entire text buffer associated
 *    with a text layout object is relocated, but no other changes have
 *    occurred that would affect the buffer's current subrange.
 *    Instead, you should call ATSUTextMoved, which is a more focused
 *    function and therefore more efficient. After associating text
 *    with a text layout object, use ATSUSetRunStyle to associate style
 *    information with the text. You can then call the function
 *    ATSUDrawText to display the text or a subrange of the text.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout object for which you wish to associate a text buffer.
 *    
 *    iText:
 *      A pointer to a buffer of Unicode text in UTF-16 format. This is
 *      the text that will be associated with iTextLayout.
 *    
 *    iTextOffset:
 *      The starting offset of the subrange of the text buffer you wish
 *      to associate with iTextLayout. To indicate that the specified
 *      text range starts at the beginning of the text buffer, you can
 *      pass the constant kATSUFromTextBeginning . To specify the
 *      entire text buffer, pass kATSUFromTextBeginning in this
 *      parameter and kATSUToTextEnd in the iTextLength parameter.
 *    
 *    iTextLength:
 *      The length of the subrage of the text buffer you wish to
 *      associate with iTextLayout. Note that the sum of iTextOffset
 *      and iTextLength must be less than or equal to the value of the
 *      iTextTotalLength parameter. If you want the range of text to
 *      extend to the end of the text buffer, you can pass the constant
 *      kATSUToTextEnd.
 *    
 *    iTextTotalLength:
 *      The length of the entire text buffer. This value should be
 *      greater than or equal to the range of text defined by the
 *      iTextLength parameter.
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
function ATSUSetTextPointerLocation( iTextLayout: ATSUTextLayout; iText: ConstUniCharArrayPtr; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount ): OSStatus; external name '_ATSUSetTextPointerLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetTextLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUSetTextPointerLocation( iTextLayout: ATSUTextLayout; iText: ConstUniCharArrayPtr; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount ): OSStatus; external name '_ATSUSetTextPointerLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetTextLocation()
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUSetTextPointerLocation( iTextLayout: ATSUTextLayout; iText: ConstUniCharArrayPtr; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount ): OSStatus; external name '_ATSUSetTextPointerLocation';


{
 *  ATSUGetTextLocation()
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetTextLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns information about the Unicode text buffer associated with
 *    a layout.
 *  
 *  Discussion:
 *    For a given layout, ATSUGetTextLocation will return information
 *    about the Unicode text buffer associated with it, including its
 *    memory location, its size, and whether it is stored in a handle.
 *    Note that since a layout may refer to a subrange within a text
 *    buffer, parameters defining this subrange are included. oOffset
 *    and oTextLength give information about the subrange, while oText
 *    and oTextTotalLength give information about the entire text
 *    buffer. You may pass NULL for any parameters you are not
 *    interested in. Only iTextLayout is required.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      A text layout whose text buffer you want information regarding.
 *    
 *    oText:
 *      A pointer to data of any type. On return, the pointer is set to
 *      either a pointer or a handle that refers to the text buffer for
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      the specified text layout object.
=======
 *      the specified text layout object. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      the specified text layout object. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      the specified text layout object.
>>>>>>> origin/fixes_2.4
 *    
 *    oTextIsStoredInHandle:
 *      On return, the value is set to true if the text buffer referred
 *      to by the oText parameter is accessed by a handle; if false, a
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      pointer.
 *    
 *    oOffset:
 *      On return, the offset from the beginning of the text buffer to
 *      the first character of the layout's current text range.
 *    
 *    oTextLength:
 *      On return, the value specifies the length of the layout's
 *      current text range.
=======
=======
>>>>>>> origin/fixes_2_2
 *      pointer. can be NULL
=======
 *      pointer.
>>>>>>> origin/fixes_2.4
 *    
 *    oOffset:
 *      On return, the offset from the beginning of the text buffer to
 *      the first character of the layout's current text range.
 *    
 *    oTextLength:
 *      On return, the value specifies the length of the layout's
<<<<<<< HEAD
 *      current text range. can be NULL
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *      current text range.
>>>>>>> origin/fixes_2.4
 *    
 *    oTextTotalLength:
 *      On return, the total length of the text buffer. Note this is
 *      not necessarily the same as the length of the layout's current
 *      range. (A layout may refer to only a subrange within a text
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      buffer.)
=======
 *      buffer.) can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      buffer.) can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      buffer.)
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
function ATSUGetTextLocation( iTextLayout: ATSUTextLayout; oText: UnivPtrPtr { can be NULL }; oTextIsStoredInHandle: BooleanPtr { can be NULL }; oOffset: UniCharArrayOffsetPtr { can be NULL }; oTextLength: UniCharCountPtr { can be NULL }; oTextTotalLength: UniCharCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetTextLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUTextDeleted()   *** DEPRECATED ***
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
function ATSUGetTextLocation( iTextLayout: ATSUTextLayout; oText: UnivPtrPtr { can be NULL }; oTextIsStoredInHandle: BooleanPtr { can be NULL }; oOffset: UniCharArrayOffsetPtr { can be NULL }; oTextLength: UniCharCountPtr { can be NULL }; oTextTotalLength: UniCharCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetTextLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUTextDeleted()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUTextDeleted()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Informs ATSUI of the location and length of a text deletion.
 *  
 *  Discussion:
 *    When you call the ATSUTextDeleted function to inform ATSUI of a
 *    text deletion, it shortens the style run(s) containing the
 *    deleted text by the amount of the deletion. If a style run
 *    corresponds entirely to a range of deleted text, that style run
 *    is removed. If the deletion point is between two style runs, the
 *    first style run is shortened (or removed). The ATSUTextDeleted
 *    function also shortens the total length of the text buffer
 *    containing the deleted text by the amount of the deletion. That
 *    is, it shifts the memory location of the text following the
 *    deleted text by iDeletedRangeLength .ATSUTextDeleted also removes
 *    any soft line breaks that fall within the deleted text and
 *    updates affected drawing caches. The ATSUTextDeleted function
 *    does not change the actual memory location of the affected text.
 *    You are responsible for deleting the corresponding text is from
 *    the text buffer. You are also responsible for calling the
 *    function ATSUDisposeStyle to dispose of the memory associated
 *    with any style runs that have been removed. Note that calling the
 *    function ATSUTextDeleted automatically removes previously-set
 *    soft line breaks if the line breaks are within the range of text
 *    that is deleted.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout containing the deleted text.
 *    
 *    iDeletedRangeStart:
 *      The starting location of the deleted text. To specify a
 *      deletion point at the beginning of the text buffer, you can
 *      pass the constant kATSUFromTextBeginning. To specify that the
 *      entire text buffer has been deleted, pass
 *      kATSUFromTextBeginning in this parameter and kATSUToTextEnd in
 *      the iDeletedRangeLength parameter.
 *    
 *    iDeletedRangeLength:
 *      The length of the deleted text. To specify a deletion length
 *      extending to the end of the text buffer, you can pass the
 *      constant kATSUToTextEnd.
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
function ATSUTextDeleted( iTextLayout: ATSUTextLayout; iDeletedRangeStart: UniCharArrayOffset; iDeletedRangeLength: UniCharCount ): OSStatus; external name '_ATSUTextDeleted';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUTextInserted()   *** DEPRECATED ***
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
function ATSUTextDeleted( iTextLayout: ATSUTextLayout; iDeletedRangeStart: UniCharArrayOffset; iDeletedRangeLength: UniCharCount ): OSStatus; external name '_ATSUTextDeleted';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUTextInserted()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUTextInserted()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Informs ATSUI of the location and length of a text insertion.
 *  
 *  Discussion:
 *    When you call the ATSUTextInserted function to inform ATSUI of a
 *    text insertion, it extends the style run containing the insertion
 *    point by the amount of the inserted text. If the insertion point
 *    is between two style runs, the first style run is extended to
 *    include the new text. The ATSUTextInserted function also extends
 *    the total length of the text buffer containing the inserted text
 *    by the amount of the inserted text. That is, it shifts the memory
 *    location of the text following the inserted text by
 *    iInsertionLength. ATSUTextInserted then updates drawing caches.
 *    Note that the ATSUTextInserted function does not change the
 *    actual memory location of the inserted text. You are responsible
 *    for placing the inserted text into the text buffer at the
 *    appropriate location. The ATSUTextInserted function does not
 *    insert style runs or line breaks; to do so, call the functions
 *    ATSUSetRunStyle and ATSUSetSoftLineBreak, respectively. Break
 *    line operations should be redone after you call ATSUTextInserted.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout in which the text insertion is taking place.
 *    
 *    iInsertionLocation:
 *      The offset corresponding to the beginning of the inserted text.
 *    
 *    iInsertionLength:
 *      The length of the inserted text.
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
function ATSUTextInserted( iTextLayout: ATSUTextLayout; iInsertionLocation: UniCharArrayOffset; iInsertionLength: UniCharCount ): OSStatus; external name '_ATSUTextInserted';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}

=======


>>>>>>> graemeg/cpstrnew
{$endc} {not TARGET_CPU_64}
=======


{$endc} {not TARGET_CPU_64}
>>>>>>> graemeg/cpstrnew
=======


{$endc} {not TARGET_CPU_64}
>>>>>>> origin/cpstrnew

{
 *  ATSUTextMoved()   *** DEPRECATED ***
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
function ATSUTextInserted( iTextLayout: ATSUTextLayout; iInsertionLocation: UniCharArrayOffset; iInsertionLength: UniCharCount ): OSStatus; external name '_ATSUTextInserted';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  ATSUTextMoved()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUTextMoved()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Informs ATSUI of the new memory location of relocated text.
 *  
 *  Discussion:
 *    You should call the ATSUTextMoved function when a range of text
 *    consisting of less than an entire text buffer has been moved. The
 *    ATSUTextMoved function informs ATSUI of the new memory location
 *    of the text. You are responsible for moving the text. The text
 *    buffer should remain otherwise unchanged.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout containing the moved text.
 *    
 *    iNewLocation:
 *      The new memory location of the moved text.
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
function ATSUTextMoved( iTextLayout: ATSUTextLayout; iNewLocation: ConstUniCharArrayPtr ): OSStatus; external name '_ATSUTextMoved';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
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
function ATSUTextMoved( iTextLayout: ATSUTextLayout; iNewLocation: ConstUniCharArrayPtr ): OSStatus; external name '_ATSUTextMoved';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI layout controls                                                       }
{ ---------------------------------------------------------------------------- }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  ATSUCopyLayoutControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTParagraphStyleCreate or CTLineGetPenOffsetForFlush instead.
=======
{
 *  ATSUCopyLayoutControls()
>>>>>>> graemeg/fixes_2_2
=======
{
 *  ATSUCopyLayoutControls()
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
{
 *  ATSUCopyLayoutControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTParagraphStyleCreate or CTLineGetPenOffsetForFlush instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Copies all layout control attribute settings from a source text
 *    layout object to a destination text layout object.
 *  
 *  Discussion:
 *    This function copies all layout control attribute values to a
 *    destination text layout object from a source text layout object,
 *    including any default (unset) values in the source object. For a
 *    list of tags and their default values, see the definition of
 *    ATSUAttributeTag. Reference constants and the contents of memory
 *    referenced by pointers within custom layout attributes are not
 *    copied. You are responsible for ensuring that this memory remains
 *    valid until both the source and destination text layout objects
 *    are disposed. To copy line control attribute values from one text
 *    layout object to another, call the function ATSUCopyLineControls.
 *  
 *  Parameters:
 *    
 *    iSourceTextLayout:
 *      The text layout to copy layout controls from.
 *    
 *    iDestTextLayout:
 *      The text layout to copy layout controls to.
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
function ATSUCopyLayoutControls( iSourceTextLayout: ATSUTextLayout; iDestTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCopyLayoutControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{
 *  ATSUSetLayoutControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
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
function ATSUCopyLayoutControls( iSourceTextLayout: ATSUTextLayout; iDestTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCopyLayoutControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  ATSUSetLayoutControls()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUSetLayoutControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets layout control attribute values in a text layout object.
 *  
 *  Discussion:
 *    When you use ATSUI to image your text, you can control the text's
 *    display and formatting at a number of different levels: layout,
 *    line, and run. The level affected by this function is the layout
 *    level, which is that of the entire text range associated with
 *    your text layout object. Attributes at this level affect the
 *    width of the text area from margin to margin, the alignment of
 *    the text, its justification, rotation, and direction, as well as
 *    other layout options. See ATSUSetLineControls for information
 *    about controling text and the line level. Similar to style
 *    attributes, you use a "triple" to specify a line or layout
 *    control attribute. That is, (1) an attribute tag, (2) the size
 *    (in bytes) of the attribute value, and (3) the value of the
 *    attribute it sets. Attribute tags are constants supplied by
 *    ATSUI. Attribute values may be a scalar, a structure, or a
 *    pointer. And as with style attributes, you can also create a
 *    custom attribute for a line or layout attribute for which ATSUI
 *    does not provide a tag. For a list of layout control tags defined
 *    by ATSUI and their default values, see the definition of
 *    ATSUAttributeTag.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout in which to set layout-level controls.
 *    
 *    iAttributeCount:
 *      The number of attributes to set. This value should correspond
 *      to the number of elements in the iTag, iValueSize, and iValue
 *      arrays.
 *    
 *    iTag:
 *      An array of attribute tags to set. For a list of layout control
 *      tags defined by ATSUI and their default values, see the
 *      definition of ATSUAttributeTag.
 *    
 *    iValueSize:
 *      An array of values indicating the sizes of the values pointed
 *      to by the elements in the iValue array.
 *    
 *    iValue:
 *      An array of attribute value pointers. Each value in the array
 *      must correspond to a tag in the iTag array and be a legal value
 *      for that tag.
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
function ATSUSetLayoutControls( iTextLayout: ATSUTextLayout; iAttributeCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr; {const} iValueSize: {variable-size-array} ByteCountPtr; {const} iValue: {variable-size-array} ATSUAttributeValuePtrPtr ): OSStatus; external name '_ATSUSetLayoutControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
 *  ATSUGetLayoutControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringGetAttributes instead.
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
function ATSUSetLayoutControls( iTextLayout: ATSUTextLayout; iAttributeCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr; {const} iValueSize: {variable-size-array} ByteCountPtr; {const} iValue: {variable-size-array} ATSUAttributeValuePtrPtr ): OSStatus; external name '_ATSUSetLayoutControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUGetLayoutControl()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetLayoutControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringGetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a single layout control attribute value for a text layout
 *    object.
 *  
 *  Discussion:
 *    Before calling ATSUGetLayoutControl, you should call the function
 *    ATSUGetAllLayoutControls to obtain an array of nondefault layout
 *    control attribute tags and value sizes for the text layout
 *    object. You can then pass the tag and value size for the
 *    attribute value to obtain to ATSUGetLayoutControl. Typically you
 *    use the function ATSUGetLayoutControl by calling it twice, as
 *    follows: (1) Pass a reference to the text layout object to
 *    examine in the iTextLayout parameter, NULL for the oValue
 *    parameter, 0 for the iExpectedValueSize parameter.
 *    ATSUGetLayoutControl returns the actual size of the attribute
 *    value in the oActualValueSize parameter. (2) Allocate enough
 *    space for an array of the returned size, then call the
 *    ATSUGetLayoutControl function again, passing a valid pointer in
 *    the oValue parameter. On return, the pointer refers to the actual
 *    attribute value contained in the text layout object. For a list
 *    of layout control tags defined by ATSUI and their default values,
 *    see the definition of ATSUAttributeTag.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout for which you wish to obtain a single layout
 *      control value.
 *    
 *    iTag:
 *      An attribute tag specifying the layout control value you wish
 *      to obtain. For a list of layout control tags defined by ATSUI
 *      and their default values, see the definition of
 *      ATSUAttributeTag.
 *    
 *    iExpectedValueSize:
 *      The size in bytes of the buffer you have allocated for the
 *      oValue parameter.
 *    
 *    oValue:
 *      On return, the value assocaited with the layout tag specified
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      by the iTag parameter.
=======
 *      by the iTag parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      by the iTag parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      by the iTag parameter.
>>>>>>> origin/fixes_2.4
 *    
 *    oActualValueSize:
 *      On return, the value contains the actual size (in bytes) of the
 *      attribute value. You should examine this parameter if you are
 *      unsure of the size of the attribute value being obtained, as in
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      the case of custom layout control attributes.
=======
 *      the case of custom layout control attributes. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      the case of custom layout control attributes. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      the case of custom layout control attributes.
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
function ATSUGetLayoutControl( iTextLayout: ATSUTextLayout; iTag: ATSUAttributeTag; iExpectedValueSize: ByteCount; oValue: ATSUAttributeValuePtr { can be NULL }; oActualValueSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetLayoutControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetAllLayoutControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringGetAttributes instead.
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
function ATSUGetLayoutControl( iTextLayout: ATSUTextLayout; iTag: ATSUAttributeTag; iExpectedValueSize: ByteCount; oValue: ATSUAttributeValuePtr { can be NULL }; oActualValueSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetLayoutControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetAllLayoutControls()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetAllLayoutControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringGetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains an array of non-default layout control attribute tags and
 *    value sizes for a text layout object.
 *  
 *  Discussion:
 *    This function function obtains all nondefault layout control
 *    attribute tags and their values sizes for a text layout object.
 *    You can pass a tag and value size pair obtained from
 *    ATSUGetAllLayoutControls to the function ATSUGetLayoutControl to
 *    determine the corresponding attribute value. Typically you use
 *    the function ATSUGetAllLayoutControls by calling it twice, as
 *    follows: (1) Pass a reference to the text layout object to
 *    examine in the iTextLayout parameter, NULL for the
 *    oAttributeInfoArray parameter, a pointer to an ItemCount value in
 *    the oTagValuePairCount parameter, and 0 for the
 *    iTagValuePairArraySize parameter. ATSUGetAllLayoutControls
 *    returns the size of the tag and value size arrays in the
 *    oTagValuePairCount parameter. (2) Allocate enough space for an
 *    array of the returned size, then call the
 *    ATSUGetAllLayoutControls function again, passing a valid pointer
 *    in the oAttributeInfoArray parameter. On return, the pointer
 *    refers to an array of the layout control attribute tag and value
 *    size pairs contained in the text layout object.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout for which you wish to obtain the set of non-default
 *      layout tags.
 *    
 *    oAttributeInfoArray:
 *      On return, this array contains pairs of tags and value sizes
 *      for the object's layout control attributes that are not at
 *      default values. If you are uncertain of how much memory to
 *      allocate for this parameter, see the Discussion.
 *    
 *    iTagValuePairArraySize:
 *      A value specifying the maximum number of tag and value size
 *      pairs to obtain for the text layout object. Typically, this is
 *      equivalent to the number of ATSUAttributeInfo structures for
 *      which you have allocated memory in the oAttributeInfoArray
 *      parameter. To determine this value, see the Discussion.
 *    
 *    oTagValuePairCount:
 *      On return, the value specifies the actual number of
 *      ATSUAttributeInfo structures in the text layout object. This
 *      may be greater than the value you specified in the
 *      iTagValuePairArraySize parameter.
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
function ATSUGetAllLayoutControls( iTextLayout: ATSUTextLayout; oAttributeInfoArray: {variable-size-array} ATSUAttributeInfoPtr; iTagValuePairArraySize: ItemCount; var oTagValuePairCount: ItemCount ): OSStatus; external name '_ATSUGetAllLayoutControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUClearLayoutControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
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
function ATSUGetAllLayoutControls( iTextLayout: ATSUTextLayout; oAttributeInfoArray: {variable-size-array} ATSUAttributeInfoPtr; iTagValuePairArraySize: ItemCount; var oTagValuePairCount: ItemCount ): OSStatus; external name '_ATSUGetAllLayoutControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUClearLayoutControls()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUClearLayoutControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Restores default values to the specified layout control
 *    attributes of a text layout object.
 *  
 *  Discussion:
 *    This function removes those layout control attribute values
 *    identified by the tag constants in the iTag array and replaces
 *    them with the default values. If you specify that any currently
 *    unset attribute values be removed, the function does not return
 *    an error. For a list of layout control tags defined by ATSUI and
 *    their default values, see the definition of ATSUAttributeTag.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout in which you wish to clear layout controls.
 *    
 *    iTagCount:
 *      The number of tags you wish to clear. This value should
 *      correspond to the nuumber of elements in the iTag array. Pass
 *      kATSUClearAll for this parameter if you wish to clear all
 *      layout controls.
 *    
 *    iTag:
 *      An array of layout control tags to be cleared. For a list of
 *      layout control tags defined by ATSUI and their default values,
 *      see the definition of ATSUAttributeTag. You may pass NULL for
 *      this parameter if you are passing kATSUClearAll for the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      iTagCount parameter.
=======
 *      iTagCount parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      iTagCount parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      iTagCount parameter.
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
function ATSUClearLayoutControls( iTextLayout: ATSUTextLayout; iTagCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr { can be NULL } ): OSStatus; external name '_ATSUClearLayoutControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUClearLayoutControls( iTextLayout: ATSUTextLayout; iTagCount: ItemCount; iTag: ATSUAttributeTagPtr ): OSStatus; external name '_ATSUClearLayoutControls';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUClearLayoutControls( iTextLayout: ATSUTextLayout; iTagCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr { can be NULL } ): OSStatus; external name '_ATSUClearLayoutControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI line controls                                                         }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCopyLineControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
<<<<<<< HEAD
=======
 *  ATSUCopyLineControls()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCopyLineControls()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Copies line control attribute settings from a line in a source
 *    text layout object to a line in a destination text layout object.
 *  
 *  Discussion:
 *    This function copies all line control attribute values to a line
 *    in a destination text layout object from a line in a source text
 *    layout object, including any default (unset) values in the source
 *    line. Unset line control attributes are assigned the default
 *    values. ATSUCopyLineControls does not copy the contents of memory
 *    referenced by pointers within custom line attributes or within
 *    reference constants. You are responsible for ensuring that this
 *    memory remains valid until the source text layout object is
 *    disposed.
 *  
 *  Parameters:
 *    
 *    iSourceTextLayout:
 *      The text layout object from which to copy line control
 *      attributes.
 *    
 *    iSourceLineStart:
 *      The start of the line from which to copy line control
 *      attributes.
 *    
 *    iDestTextLayout:
 *      The text layout object for which to set line control
 *      attributes. This can be the same text layout object passed in
 *      the iSourceTextLayout parameter if you want to copy line
 *      control attributes from one line to another within a text
 *      layout object.
 *    
 *    iDestLineStart:
 *      The start of the line to which to copy line control attributes.
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
function ATSUCopyLineControls( iSourceTextLayout: ATSUTextLayout; iSourceLineStart: UniCharArrayOffset; iDestTextLayout: ATSUTextLayout; iDestLineStart: UniCharArrayOffset ): OSStatus; external name '_ATSUCopyLineControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUSetLineControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
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
function ATSUCopyLineControls( iSourceTextLayout: ATSUTextLayout; iSourceLineStart: UniCharArrayOffset; iDestTextLayout: ATSUTextLayout; iDestLineStart: UniCharArrayOffset ): OSStatus; external name '_ATSUCopyLineControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUSetLineControls()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUSetLineControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets one or more line control values for a specified line in a
 *    text layout.
 *  
 *  Discussion:
 *    When you use ATSUI to image your text, you can control the text's
 *    display and formatting at a number of different levels: layout,
 *    line, and run. The level affected by this function is the line
 *    level. These attributes are similar to those that you can apply
 *    on a full-layout basis, but each affects only an individual text
 *    line. Note that setting line control attributes overrides the
 *    corresponding layout-level settings. Also, from a performance
 *    standpoint, it is preferable to work from the layout level and
 *    not specify such controls line by line unless necessary. Lines
 *    are determined by soft breaks that may be set in your layout. You
 *    can specify a line by giving a starting offset into the text
 *    buffer. Attributes at this level affect the width of the text
 *    area from margin to margin, the alignment of the text, its
 *    justification, rotation, and direction, as well as other layout
 *    options. Similar to style attributes, you use a "triple" to
 *    specify a line or layout control attribute. That is, (1) an
 *    attribute tag, (2) the size (in bytes) of the attribute value,
 *    and (3) the value of the attribute it sets. Attribute tags are
 *    constants supplied by ATSUI. Attribute values may be a scalar, a
 *    structure, or a pointer. And as with style attributes, you can
 *    also create a custom attribute for a line or layout attribute for
 *    which ATSUI does not provide a tag. For a list of line control
 *    tags defined by ATSUI and their default values, see the
 *    definition of ATSUAttributeTag.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout in which you wish to set line controls.
 *    
 *    iLineStart:
 *      The starting offset of the line for which you wish to set
 *      controls.
 *    
 *    iAttributeCount:
 *      The number of attributes to set. This value should correspond
 *      to the number of elements in the iTag, iValueSize, and iValue
 *      arrays.
 *    
 *    iTag:
 *      An array of attribute tags to set. For a list of line control
 *      tags defined by ATSUI and their default values, see the
 *      definition of ATSUAttributeTag.
 *    
 *    iValueSize:
 *      An array of values indicating the sizes of the values pointed
 *      to by the elements in the iValue array.
 *    
 *    iValue:
 *      An array of attribute value pointers. Each value in the array
 *      must correspond to a tag in the iTag array and be a legal value
 *      for that tag.
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
function ATSUSetLineControls( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iAttributeCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr; {const} iValueSize: {variable-size-array} ByteCountPtr; {const} iValue: {variable-size-array} ATSUAttributeValuePtrPtr ): OSStatus; external name '_ATSUSetLineControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetLineControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
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
function ATSUSetLineControls( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iAttributeCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr; {const} iValueSize: {variable-size-array} ByteCountPtr; {const} iValue: {variable-size-array} ATSUAttributeValuePtrPtr ): OSStatus; external name '_ATSUSetLineControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetLineControl()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetLineControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a single line control attribute value for a line in a
 *    text layout object.
 *  
 *  Discussion:
 *    Before calling ATSUGetLineControl, you should call the function
 *    ATSUGetAllLineControls to obtain an array of nondefault line
 *    control attribute tags and value sizes for the line. You can then
 *    pass the tag and value size for the attribute value to obtain to
 *    ATSUGetLineControl. Typically you use the function
 *    ATSUGetLineControl by calling it twice, as follows: (1) Pass a
 *    reference to the text layout object to examine in the iTextLayout
 *    parameter, NULL for the oValue parameter, 0 for the
 *    iExpectedValueSize parameter. ATSUGetLineControl returns the
 *    actual size of the attribute value in the oActualValueSize
 *    parameter. (2) Allocate enough space for an array of the returned
 *    size, then call the ATSUGetLineControl function again, passing a
 *    valid pointer in the oValue parameter. On return, the pointer
 *    refers to the actual attribute value contained for the line in
 *    the text layout object.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout for which to obtain a line control value.
 *    
 *    iLineStart:
 *      The start of the line for which to obtain a line control value.
 *    
 *    iTag:
 *      A tag specifying the line control value to be obtained. For a
 *      list of line control tags defined by ATSUI and their default
 *      values, see the definition of ATSUAttributeTag.
 *    
 *    iExpectedValueSize:
 *      The expected size (in bytes) of the value to obtain.
 *    
 *    oValue:
 *      On return, the actual attribute value. If you are uncertain of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      how much memory to allocate, see the Discussion.
=======
 *      how much memory to allocate, see the Discussion. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      how much memory to allocate, see the Discussion. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      how much memory to allocate, see the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    oActualValueSize:
 *      On return, the value contains the actual size (in bytes) of the
 *      attribute value. You should examine this parameter if you are
 *      unsure of the size of the attribute value being obtained, as in
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      the case of custom line control attributes.
=======
 *      the case of custom line control attributes. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      the case of custom line control attributes. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      the case of custom line control attributes.
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
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUGetLineControl( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iTag: ATSUAttributeTag; iExpectedValueSize: ByteCount; oValue: ATSUAttributeValuePtr { can be NULL }; oActualValueSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetLineControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetAllLineControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringGetAttributes instead.
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
function ATSUGetLineControl( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iTag: ATSUAttributeTag; iExpectedValueSize: ByteCount; oValue: ATSUAttributeValuePtr { can be NULL }; oActualValueSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetLineControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetAllLineControls()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetAllLineControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringGetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains an array of line control attribute tags and value sizes
 *    for a line in a text layout object.
 *  
 *  Discussion:
 *    This function obtains all nondefault line control attribute tags
 *    and their values sizes for a line in a text layout object. You
 *    can pass a tag and value size pair obtained from
 *    ATSUGetAllLineControls to the function ATSUGetLineControl to
 *    determine the corresponding attribute value. Typically you use
 *    the function ATSUGetAllLineControls by calling it twice, as
 *    follows: (1) Pass a reference to the text layout object to
 *    examine in the iTextLayout parameter, the appropriate
 *    UniCharArrayOffset value in the iLineStart parameter, NULL for
 *    the oAttributeInfoArray parameter, a pointer to an ItemCount
 *    value in the oTagValuePairCount parameter, and 0 for the
 *    iTagValuePairArraySize parameter. ATSUGetAllLineControls returns
 *    the size of the tag and value size arrays in the
 *    oTagValuePairCount parameter. (2) Allocate enough space for an
 *    array of the returned size, then call the ATSUGetAllLineControls
 *    function again, passing a valid pointer in the
 *    oAttributeInfoArray parameter. On return, the pointer refers to
 *    an array of the line control attribute tag and value size pairs
 *    contained in the specified line. To obtain the nondefault layout
 *    control attribute tags and value sizes for a text layout object,
 *    call the function ATSUGetAllLayoutControls.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout for which you wish to obtain line control
 *      information.
 *    
 *    iLineStart:
 *      The beginning of the line for which you wish to obtain line
 *      control information.
 *    
 *    oAttributeInfoArray:
 *      On return, this array contains pairs of tags and value sizes
 *      for the object's line control attributes that are not at
 *      default values. If you are uncertain of how much memory to
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      allocate for this array, see the Discussion.
=======
 *      allocate for this array, see the Discussion. can be NULL 
>>>>>>> graemeg/fixes_2_2
=======
 *      allocate for this array, see the Discussion. can be NULL 
>>>>>>> origin/fixes_2_2
=======
 *      allocate for this array, see the Discussion.
>>>>>>> origin/fixes_2.4
 *    
 *    iTagValuePairArraySize:
 *      The size of of the array you allocated for the
 *      oAttributeInfoArray parameter.
 *    
 *    oTagValuePairCount:
 *      On return, the value specifies the actual number of
 *      ATSUAttributeInfo structures in the line. This may be greater
 *      than the value you specified in the iTagValuePairArraySize
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
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUGetAllLineControls( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; oAttributeInfoArray: {variable-size-array} ATSUAttributeInfoPtr { can be NULL }; iTagValuePairArraySize: ItemCount; oTagValuePairCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAllLineControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUClearLineControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
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
function ATSUGetAllLineControls( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; oAttributeInfoArray: {variable-size-array} ATSUAttributeInfoPtr { can be NULL }; iTagValuePairArraySize: ItemCount; oTagValuePairCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetAllLineControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUClearLineControls()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUClearLineControls()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Restores default values to the specified line control attributes
 *    of a text layout object.
 *  
 *  Discussion:
 *    This function removes those line control attribute values
 *    identified by the tag constants in the iTag array and replaces
 *    them with the default values. If you specify that any currently
 *    unset attribute values be removed, the function does not return
 *    an error. For a list of line control tags defined by ATSUI and
 *    their default values, see the definition of ATSUAttributeTag.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout in which you wish to clear line controls.
 *    
 *    iLineStart:
 *      The start of the line in which to clear line controls.
 *    
 *    iTagCount:
 *      The number of tags you wish to clear. This value should
 *      correspond to the nuumber of elements in the iTag array. Pass
 *      kATSUClearAll to clear all line controls.
 *    
 *    iTag:
 *      An array of line control tags to be cleared. For a list of line
 *      control tags defined by ATSUI and their default values, see the
 *      definition of ATSUAttributeTag. You may pass NULL for this
 *      parameter if you are passing kATSUClearAll for the iTagCount
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
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUClearLineControls( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iTagCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr { can be NULL } ): OSStatus; external name '_ATSUClearLineControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUClearLineControls( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iTagCount: ItemCount; iTag: ATSUAttributeTagPtr ): OSStatus; external name '_ATSUClearLineControls';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUClearLineControls( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iTagCount: ItemCount; {const} iTag: {variable-size-array} ATSUAttributeTagPtr { can be NULL } ): OSStatus; external name '_ATSUClearLineControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI style run processing                                                  }
{ ---------------------------------------------------------------------------- }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{
 *  ATSUSetRunStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
=======
{
 *  ATSUSetRunStyle()
>>>>>>> graemeg/fixes_2_2
=======
{
 *  ATSUSetRunStyle()
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

{
 *  ATSUSetRunStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Defines a style run by associating style information with a run
 *    of text.
 *  
 *  Discussion:
 *    A text run consists of one or more characters that are contiguous
 *    in memory. If you associate these characters with a distinct
 *    style, you define a style run. You can use the ATSUSetRunStyle
 *    function to define a style run, by associating a style object
 *    with a run of text in a text layout object. Each text run must be
 *    assigned a style object, which may or may not differ from other
 *    style objects assigned to other text runs in a given text layout
 *    object. After calling ATSUSetRunStyle, you can call the function
 *    ATSUDrawText to display the styled text. When you call
 *    ATSUDrawText, if you have not previously assigned styles to all
 *    the characters you request to be drawn, ATSUI automatically does
 *    so. Specifically, ATSUI extends the first style it locates
 *    immediately prior (in storage order) to the unstyled characters
 *    to include those unassigned characters. If the unstyled
 *    characters are at the beginning of the text stream, ATSUI finds
 *    the first style run in the stream and extends it backward to the
 *    first character. You should call ATSUSetRunStyle whenever you
 *    create a new text layout object without any associated styles, as
 *    by using the function ATSUCreateTextLayout. You should also call
 *    ATSUSetRunStyle to assign a style to a text run in response to a
 *    user action, such as when the user selects a run of text and
 *    changes the font. You do not need to call ATSUSetRunStyle when
 *    you change style attributes or text layout attributes. In such
 *    cases, ATSUI automatically updates the layout of the text as
 *    appropriate.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout in which you wish to set the style run.
 *    
 *    iStyle:
 *      The style to be assigned to the run of characters.
 *    
 *    iRunStart:
 *      The start of the run of characters. To specify the beginning of
 *      the text buffer, pass kATSUFromTextBeginning for this parameter.
 *    
 *    iRunLength:
 *      The end of the run of characters. To specify a run that
 *      continues to the end of the text buffer, pass kATSUToTextEnd
 *      for this parameter.
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
function ATSUSetRunStyle( iTextLayout: ATSUTextLayout; iStyle: ATSUStyle; iRunStart: UniCharArrayOffset; iRunLength: UniCharCount ): OSStatus; external name '_ATSUSetRunStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
 *  ATSUGetRunStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTLineGetGlyphRuns instead.
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
function ATSUSetRunStyle( iTextLayout: ATSUTextLayout; iStyle: ATSUStyle; iRunStart: UniCharArrayOffset; iRunLength: UniCharCount ): OSStatus; external name '_ATSUSetRunStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUGetRunStyle()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetRunStyle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTLineGetGlyphRuns instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains style run information for a character offset in a run of
 *    text.
 *  
 *  Discussion:
 *    You can use the ATSUGetRunStyle function to obtain the style
 *    object assigned to a given text offset. ATSUGetRunStyle also
 *    produces the encompassing text range that shares the style object
 *    with the offset. Note that the style object contains those
 *    previously set style attributes, font features, and font
 *    variations that are continuous for the range of text that
 *    includes the specified text offset. If you want to obtain all
 *    shared style information for a style run, including any unset
 *    attributes, call the function ATSUGetContinuousAttributes
 *    instead. If only one style run is set in the text layout object,
 *    and it does not cover the entire text layout object,
 *    ATSUGetRunStyle uses the style run information for the iOffset
 *    parameter to set the style run information for the remaining text.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout for which to obtain style run information.
 *    
 *    iOffset:
 *      The beginning character for which you want to obtain style run
 *      information.
 *    
 *    oStyle:
 *      On return, the style object assigned to the range of text
 *      containing the character at iOffset. Note that if you pass an
 *      offset in the iOffset parameter that is at a style run
 *      boundary, ATSUGetRunStyle produces style run information for
 *      the following, not preceding, style run.
 *    
 *    oRunStart:
 *      On return, the offset from the beginning of the text buffer to
 *      the first character of the style run containing the character
 *      at iOffset. Note that the entire style run does not necessarily
 *      share the same unset attribute values as the character at
 *      iOffset.
 *    
 *    oRunLength:
 *      On return, the length of the style run containing the character
 *      at iOffset.
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
function ATSUGetRunStyle( iTextLayout: ATSUTextLayout; iOffset: UniCharArrayOffset; var oStyle: ATSUStyle; var oRunStart: UniCharArrayOffset; var oRunLength: UniCharCount ): OSStatus; external name '_ATSUGetRunStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetContinuousAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTParagraphStyleCreate instead.
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
function ATSUGetRunStyle( iTextLayout: ATSUTextLayout; iOffset: UniCharArrayOffset; var oStyle: ATSUStyle; var oRunStart: UniCharArrayOffset; var oRunLength: UniCharCount ): OSStatus; external name '_ATSUGetRunStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetContinuousAttributes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetContinuousAttributes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTParagraphStyleCreate instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the style attribute values that are continuous over a
 *    given text range.
 *  
 *  Discussion:
 *    This function examines the specified text range to obtain the
 *    style attribute values (including those at default values) that
 *    remain consistent for the entire text range. You should call
 *    ATSUGetContinuousAttributes to determine the style information
 *    that remains constant over text that has been selected by the
 *    user.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout for which you wish to obtain style run information.
 *    
 *    iOffset:
 *      The starting character for which to examine style run
 *      attributes. To specify the beginning of the text buffer, pass
 *      kATSUFromTextBeginning for this parameter.
 *    
 *    iLength:
 *      The length of the range of characters to examine. To specify a
 *      range that continues to the end of the text buffer, pass
 *      kATSUToTextEnd for this parameter.
 *    
 *    oStyle:
 *      On return, a style object containing those attributes which are
 *      the same for the entire text range specified by the iOffset and
 *      iLength parameters.
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
function ATSUGetContinuousAttributes( iTextLayout: ATSUTextLayout; iOffset: UniCharArrayOffset; iLength: UniCharCount; oStyle: ATSUStyle ): OSStatus; external name '_ATSUGetContinuousAttributes';
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
function ATSUGetContinuousAttributes( iTextLayout: ATSUTextLayout; iOffset: UniCharArrayOffset; iLength: UniCharCount; oStyle: ATSUStyle ): OSStatus; external name '_ATSUGetContinuousAttributes';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI tab support                                                           }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUSetTabArray()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTParagraphStyleCreate instead.
<<<<<<< HEAD
=======
 *  ATSUSetTabArray()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUSetTabArray()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets a tab ruler for a text layout object.
 *  
 *  Discussion:
 *    When a tab ruler is set for a text layout object, ATSUI
 *    automatically aligns text such that any tabs characters in the
 *    text are laid out to follow the tab ruler's specifications. If
 *    you want to use tabs in your text and you also want to use the
 *    function ATSUBatchBreakLines, then you must set tabs by calling
 *    the function ATSUSetTabArray. See the definition of ATSUTab for
 *    more information about setting up a tab ruler.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The layout in which to set the tab array.
 *    
 *    iTabs:
 *      An array of tabstops. See the definition of ATSUTab for more
 *      inforamation about specifying tabs.
 *    
 *    iTabCount:
 *      The number of tab stops in the iTabs array.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUSetTabArray( iTextLayout: ATSUTextLayout; {const} iTabs: {variable-size-array} ATSUTabPtr; iTabCount: ItemCount ): OSStatus; external name '_ATSUSetTabArray';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetTabArray()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTParagraphStyleCreate instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUSetTabArray( iTextLayout: ATSUTextLayout; {const} iTabs: {variable-size-array} ATSUTabPtr; iTabCount: ItemCount ): OSStatus; external name '_ATSUSetTabArray';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetTabArray()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetTabArray()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTParagraphStyleCreate instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Retrieves the tab ruler associated with a text layout object.
 *  
 *  Discussion:
 *    This function can be used to retrieve all the tabs that were
 *    previously set for a text layout object, using the function
 *    ATSUSetTabArray . All the returned tabs will be in order of
 *    position along the line.Typically you use the ATSUGetTabArray
 *    function by calling it twice, as follows: (1) Pass NULL for the
 *    oTabs parameter, 0 for the iMaxTabCount parameter, and valid
 *    values for the other parameters. The ATSUGetTabArray function
 *    returns the actual number of tabs in the oTabCount parameter. (2)
 *    Allocate enough space for a buffer of the returned size, then
 *    call the function again, passing a valid pointer to the buffer in
 *    the oTabs parameter. On return, the buffer contains the tab
 *    values in order of position along the line from left to right.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout for which to retrieve the tab ruler.
 *    
 *    iMaxTabCount:
 *      The size of the array you have allocated for the oTabs
 *      parameter. If you are unsure what to pass for this parameter,
 *      see the Discussion.
 *    
 *    oTabs:
 *      On return, an array of ATSUTab structures specifying the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      currently set tab ruler for this layout.
=======
 *      currently set tab ruler for this layout. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      currently set tab ruler for this layout. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      currently set tab ruler for this layout.
>>>>>>> origin/fixes_2.4
 *    
 *    oTabCount:
 *      On return, the number of tabs currently set in this layout.
 *      Note that this may be greater than the value you have passed
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      for iMaxTabCount.
=======
 *      for iMaxTabCount. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      for iMaxTabCount. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      for iMaxTabCount.
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
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUGetTabArray( iTextLayout: ATSUTextLayout; iMaxTabCount: ItemCount; oTabs: {variable-size-array} ATSUTabPtr { can be NULL }; oTabCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetTabArray';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSUGetTabArray( iTextLayout: ATSUTextLayout; iMaxTabCount: ItemCount; oTabs: ATSUTabPtr; oTabCount: ItemCountPtr ): OSStatus; external name '_ATSUGetTabArray';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUGetTabArray( iTextLayout: ATSUTextLayout; iMaxTabCount: ItemCount; oTabs: {variable-size-array} ATSUTabPtr { can be NULL }; oTabCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetTabArray';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{ ATSUI font fallback object functions                                         }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCreateFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
<<<<<<< HEAD
=======
 *  ATSUCreateFontFallbacks()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCreateFontFallbacks()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Creates an opaque object that can be set to contain a font list
 *    and a font-search method.
 *  
 *  Discussion:
 *    After using this fucntion to create an ATSUFontFallbacks object,
 *    you can then use ATSUSetObjFontFallbacks to set the fallback
 *    method for this object, and then use the
 *    kATSULineFontFallbacksTag to apply the object to a layout. You
 *    may then either call ATSUMatchFontsToText to manually perform
 *    font substitution, or call ATSUSetTransientFontMatching to
 *    perform automatic font subtitution.
 *  
 *  Parameters:
 *    
 *    oFontFallback:
 *      On return, a reference to a newly created ATSUFontFallbacks
 *      object. You are responsible for freeing this object with
 *      ATSUDisposeFontFallbacks.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUCreateFontFallbacks( var oFontFallback: ATSUFontFallbacks ): OSStatus; external name '_ATSUCreateFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUDisposeFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUCreateFontFallbacks( var oFontFallback: ATSUFontFallbacks ): OSStatus; external name '_ATSUCreateFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUDisposeFontFallbacks()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUDisposeFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Disposes of an ATSUDisposeFontFallbacks object.
 *  
 *  Discussion:
 *    This function will only dispose of the ATSUDisposeFontFallbacks
 *    itself. If you have allocated an array of ATSUFontIDs for use
 *    with this ATSUFontFallbacks object, you are responsible for
 *    freeing it separately.
 *  
 *  Parameters:
 *    
 *    iFontFallbacks:
 *      The ATSUFontFallbacks object to be disposed of.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDisposeFontFallbacks( iFontFallbacks: ATSUFontFallbacks ): OSStatus; external name '_ATSUDisposeFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUSetObjFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDisposeFontFallbacks( iFontFallbacks: ATSUFontFallbacks ): OSStatus; external name '_ATSUDisposeFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUSetObjFontFallbacks()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUSetObjFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Assigns a font-search method and a font list to a font fallback
 *    object.
 *  
 *  Discussion:
 *    This function allows you to define the settings for a font
 *    fallback object. These settings determine the method ATSUI uses
 *    to perform font fallbacks, as well as the font search list, if
 *    one is needed. Not all fallback methods require a search list.
 *    See the definition of ATSUFontFallbackMethod for more infomation
 *    about the different font fallback methods. Once you have called
 *    this function, you typically will want to associate the font
 *    fallback object with a text layout using ATSUSetLayoutControls
 *    and the kATSULineFontFallbacksTag attribute.
 *  
 *  Parameters:
 *    
 *    iFontFallbacks:
 *      The fallback object for which you wish to set or change
 *      settings.
 *    
 *    iFontFallbacksCount:
 *      The number of fonts contained in the iFonts array. Some font
 *      fallbacks methods do not require such a list. In such cases,
 *      you may pass zero for this paramter.
 *    
 *    iFonts:
 *      A list of fonts for ATSUI to search through when performing
 *      fallbacks. Some font fallbacks methods do not require such a
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      list. In such cases, you may pass NULL for this parameter.
=======
 *      list. In such cases, you may pass NULL for this parameter. can be NUL
>>>>>>> graemeg/fixes_2_2
=======
 *      list. In such cases, you may pass NULL for this parameter. can be NUL
>>>>>>> origin/fixes_2_2
=======
 *      list. In such cases, you may pass NULL for this parameter.
>>>>>>> origin/fixes_2.4
 *    
 *    iFontFallbackMethod:
 *      The font fallback method for ATSUI to use. See the definition
 *      of ATSUFontFallbackMethod for a list of possible constants to
 *      pass in for this paramater. Note that some fallback modes
 *      require a list of fonts for ATSUI to search. In such cases, use
 *      the iFonts and iFontFallbacksCount parameters to specify this
 *      list.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUSetObjFontFallbacks( iFontFallbacks: ATSUFontFallbacks; iFontFallbacksCount: ItemCount; {const} iFonts: {variable-size-array} ATSUFontIDPtr { can be NULL }; iFontFallbackMethod: ATSUFontFallbackMethod ): OSStatus; external name '_ATSUSetObjFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGetObjFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUSetObjFontFallbacks( iFontFallbacks: ATSUFontFallbacks; iFontFallbacksCount: ItemCount; {const} iFonts: {variable-size-array} ATSUFontIDPtr { can be NULL }; iFontFallbackMethod: ATSUFontFallbackMethod ): OSStatus; external name '_ATSUSetObjFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGetObjFontFallbacks()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetObjFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontDescriptorCreateCopyWithAttributes instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns information about the current settings in an
 *    ATSUFontFallbacks object.
 *  
 *  Discussion:
 *    Information returned includes the font-search method, and the
 *    font search list, if one is set. Note that some font fallback
 *    modes do not have a client-specified search list. You must
 *    allocate space for this list.
 *  
 *  Parameters:
 *    
 *    iFontFallbacks:
 *      The font fallback object you want to know the current settings
 *      of.
 *    
 *    iMaxFontFallbacksCount:
 *      For this parameter, pass in the size of the array you are
 *      passing in for the oFonts parameter.
 *    
 *    oFonts:
 *      On input, a buffer you have allocated for storing the font
 *      search list. On return, ATSUGetObjFontFallbacks will populate
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      the list up to iMaxFontFallbacksCount items.
=======
 *      the list up to iMaxFontFallbacksCount items. can be NUL
>>>>>>> graemeg/fixes_2_2
=======
 *      the list up to iMaxFontFallbacksCount items. can be NUL
>>>>>>> origin/fixes_2_2
=======
 *      the list up to iMaxFontFallbacksCount items.
>>>>>>> origin/fixes_2.4
 *    
 *    oFontFallbackMethod:
 *      On return, the font fallback method currently set for this
 *      object. See the definition of ATSUFontFallbackMethod for more
 *      information regarding the different font fallback modes.
 *    
 *    oActualFallbacksCount:
 *      On return, the size of the font search list. You can use this
 *      parameter to determine how much space to allocate for the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      oFonts parameter.
=======
 *      oFonts parameter. can be NUL
>>>>>>> graemeg/fixes_2_2
=======
 *      oFonts parameter. can be NUL
>>>>>>> origin/fixes_2_2
=======
 *      oFonts parameter.
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
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUGetObjFontFallbacks( iFontFallbacks: ATSUFontFallbacks; iMaxFontFallbacksCount: ItemCount; oFonts: {variable-size-array} ATSUFontIDPtr { can be NULL }; var oFontFallbackMethod: ATSUFontFallbackMethod; oActualFallbacksCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetObjFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
// AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER
function ATSUGetObjFontFallbacks( iFontFallbacks: ATSUFontFallbacks; iMaxFontFallbacksCount: ItemCount; oFonts: ATSUFontIDPtr; var oFontFallbackMethod: ATSUFontFallbackMethod; oActualFallbacksCount: ItemCountPtr ): OSStatus; external name '_ATSUGetObjFontFallbacks';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUGetObjFontFallbacks( iFontFallbacks: ATSUFontFallbacks; iMaxFontFallbacksCount: ItemCount; oFonts: {variable-size-array} ATSUFontIDPtr { can be NULL }; var oFontFallbackMethod: ATSUFontFallbackMethod; oActualFallbacksCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_ATSUGetObjFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{  ATSUI font matching                                                         }
{ ---------------------------------------------------------------------------- }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{
 *  ATSUMatchFontsToText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreateForString instead.
=======
=======
>>>>>>> origin/fixes_2_2
{
 *  ATSUSetFontFallbacks()
 *  
 *  Summary:
 *    Sets font fallback behavior on a global basis.
 *  
 *  Discussion:
 *    Control of font fallback behavior on a global basis is no longer
 *    recommended. Object based font fallbacks are preferred. See the
 *    functions ATSUCreateFontFallbacks, ATSUDisposeFontFallbacks,
 *    ATSUSetObjFontFallbacks, and ATSUGetObjFontFallbacks, as well as
 *    the kATSULineFontFallbacksTag attribute for more information
 *    about object based font fallbacks.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUSetFontFallbacks( iFontFallbacksCount: ItemCount; iFontIDs: ATSUFontIDPtr; iFontFallbackMethod: ATSUFontFallbackMethod ): OSStatus; external name '_ATSUSetFontFallbacks';

=======
{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

{
 *  ATSUMatchFontsToText()   *** DEPRECATED ***
 *  
<<<<<<< HEAD
 *  Summary:
 *    Gets the current global font fallback behavior.
 *  
 *  Discussion:
 *    Control of font fallback behavior on a global basis is no longer
 *    recommended. Object based font fallbacks are preferred. See the
 *    functions ATSUCreateFontFallbacks, ATSUDisposeFontFallbacks,
 *    ATSUSetObjFontFallbacks, and ATSUGetObjFontFallbacks, as well as
 *    the kATSULineFontFallbacksTag attribute for more information
 *    about object based font fallbacks.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUGetFontFallbacks( iMaxFontFallbacksCount: ItemCount; oFontIDs: ATSUFontIDPtr; var oFontFallbackMethod: ATSUFontFallbackMethod; var oActualFallbacksCount: ItemCount ): OSStatus; external name '_ATSUGetFontFallbacks';


{
 *  ATSUMatchFontsToText()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Deprecated:
 *    Use CTFontCreateForString instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Examines a text range for characters that cannot be drawn with
 *    the current font and suggests a substitute font, if necessary.
 *  
 *  Discussion:
 *    When you call the ATSUMatchFontsToText function, ATSUI scans the
 *    given range of text for characters that cannot be drawn with the
 *    currently assigned font. When ATSUI finds such a character, it
 *    identifies a substitute font for drawing the character. ATSUI
 *    then continues scanning the text range for subsequent characters
 *    that cannot be drawn, stopping when it finds a character that can
 *    be drawn with the currently assigned font, or finds a character
 *    that cannot be drawn with either the currently assigned font or
 *    the substitute font, or reaches the end of the text range you
 *    have specified. ATSUI's default behavior for finding a substitute
 *    font is to recommend the first valid font that it finds when
 *    scanning the fonts in the user's system. ATSUI first searches in
 *    the standard application fonts for various languages. If that
 *    fails, ATSUI searches through the remaining fonts on the system
 *    in the order in which the Font Manager returns the fonts. After
 *    ATSUI has searched all the fonts in the system, any unmatched
 *    text is drawn using the last-resort font. That is, missing glyphs
 *    are represented by and empty box to indicate to the user that a
 *    valid font for that character is not installed on their system.
 *    You can alter ATSUI's default search behavior by calling the
 *    function ATSUCreateFontFallbacks and defining your own font
 *    fallback settings for the text layout object. Because ATSUI does
 *    not necessarily completely scan the text range you specify with
 *    each call to ATSUMatchFontsToText, if ATSUI does find any
 *    characters that cannot be rendered with their current font, you
 *    should call ATSUMatchFontsToText again and update the input range
 *    to check that all the subsequent characters in the range can be
 *    drawn. For that reason, you should call ATSUMatchFontsToText from
 *    within a loop to assure that the entire range of text is checked.
 *    Note that calling ATSUMatchFontsToText does not cause the
 *    suggested font substitution to be performed. If you want ATSUI to
 *    perform font substitution automatically, you can call the
 *    function ATSUSetTransientFontMatching.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The text layout object to examine.
 *    
 *    iTextStart:
 *      The first character of the range to examine. To start at the
 *      beginning of the text buffer, pass the constant
 *      kATSUFromTextBeginning.
 *    
 *    iTextLength:
 *      The length of the text range to examine. If you want the range
 *      of text to extend to the end of the text buffer, you can pass
 *      the constant kATSUToTextEnd.
 *    
 *    oFontID:
 *      On return, the value provides a font ID for the suggested
 *      substitute font or kATSUInvalidFontID, if no substitute font is
 *      available.
 *    
 *    oChangedOffset:
 *      On return, this value specifies the offset from the beginning
 *      of the text buffer to the first character that cannot be drawn
 *      with the current font.
 *    
 *    oChangedLength:
 *      On return, this value specifies the length of the text range
 *      that cannot be drawn with the current font.
 *  
 *  Result:
 *    The result code noErr indicates that all the characters in the
 *    given range can be rendered with their current font(s) and no
 *    font substitution is needed. If you receive either of the result
 *    codes kATSUFontsMatched or kATSUFontsNotMatched, you should
 *    update the input range and call ATSUMatchFontsToText again to
 *    ensure that all the characters in the range can be drawn. See
 *    MacErrors.h for other possible error codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUMatchFontsToText( iTextLayout: ATSUTextLayout; iTextStart: UniCharArrayOffset; iTextLength: UniCharCount; var oFontID: ATSUFontID; var oChangedOffset: UniCharArrayOffset; var oChangedLength: UniCharCount ): OSStatus; external name '_ATSUMatchFontsToText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUSetTransientFontMatching()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
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
function ATSUMatchFontsToText( iTextLayout: ATSUTextLayout; iTextStart: UniCharArrayOffset; iTextLength: UniCharCount; var oFontID: ATSUFontID; var oChangedOffset: UniCharArrayOffset; var oChangedLength: UniCharCount ): OSStatus; external name '_ATSUMatchFontsToText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUSetTransientFontMatching()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUSetTransientFontMatching()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Sets the current transient font matching state for a given layout.
 *  
 *  Discussion:
 *    Transient font matching allows ATSUI to automatically substitute
 *    glyphs from other fonts if the specified styles do not contain
 *    glyphs for all the characters in the text. You can change the
 *    behavior of this font substitution by calling the function
 *    ATSUCreateFontFallbacks and defining your own font fallback
 *    settings for the text layout object.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      A layout for which to set the current transient font matching
 *      state.
 *    
 *    iTransientFontMatching:
 *      A boolean value indicating if the current transient font
 *      matching state to set.
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
function ATSUSetTransientFontMatching( iTextLayout: ATSUTextLayout; iTransientFontMatching: Boolean ): OSStatus; external name '_ATSUSetTransientFontMatching';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
 *  ATSUGetTransientFontMatching()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
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
function ATSUSetTransientFontMatching( iTextLayout: ATSUTextLayout; iTransientFontMatching: Boolean ): OSStatus; external name '_ATSUSetTransientFontMatching';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUGetTransientFontMatching()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGetTransientFontMatching()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains the current transient font matching state for a given
 *    layout.
 *  
 *  Discussion:
 *    Transient font matching allows ATSUI to automatically substitute
 *    glyphs from other fonts if the specified styles do not contain
 *    glyphs for all the characters in the text. You can change the
 *    behavior of this font substitution by calling the function
 *    ATSUCreateFontFallbacks and defining your own font fallback
 *    settings for the text layout object.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      A layout for which to obtain the current transient font
 *      matching state.
 *    
 *    oTransientFontMatching:
 *      On return, a boolean value indicating the current transient
 *      font matching state.
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
function ATSUGetTransientFontMatching( iTextLayout: ATSUTextLayout; var oTransientFontMatching: Boolean ): OSStatus; external name '_ATSUGetTransientFontMatching';
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
function ATSUGetTransientFontMatching( iTextLayout: ATSUTextLayout; var oTransientFontMatching: Boolean ): OSStatus; external name '_ATSUGetTransientFontMatching';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ Functions listed beyond this point are either deprecated or not recommended }

{ ---------------------------------------------------------------------------- }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ ATSUI global font fallback functions                                        }
{ ---------------------------------------------------------------------------- }
{
 *  ATSUSetFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
 *  
 *  Summary:
 *    Sets font fallback behavior on a global basis.
 *  
 *  Discussion:
 *    Control of font fallback behavior on a global basis is no longer
 *    recommended. Object based font fallbacks are preferred. See the
 *    functions ATSUCreateFontFallbacks, ATSUDisposeFontFallbacks,
 *    ATSUSetObjFontFallbacks, and ATSUGetObjFontFallbacks, as well as
 *    the kATSULineFontFallbacksTag attribute for more information
 *    about object based font fallbacks.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUSetFontFallbacks( iFontFallbacksCount: ItemCount; {const} iFontIDs: {variable-size-array} ATSUFontIDPtr; iFontFallbackMethod: ATSUFontFallbackMethod ): OSStatus; external name '_ATSUSetFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  ATSUGetFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
 *  
 *  Summary:
 *    Gets the current global font fallback behavior.
 *  
 *  Discussion:
 *    Control of font fallback behavior on a global basis is no longer
 *    recommended. Object based font fallbacks are preferred. See the
 *    functions ATSUCreateFontFallbacks, ATSUDisposeFontFallbacks,
 *    ATSUSetObjFontFallbacks, and ATSUGetObjFontFallbacks, as well as
 *    the kATSULineFontFallbacksTag attribute for more information
 *    about object based font fallbacks.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUGetFontFallbacks( iMaxFontFallbacksCount: ItemCount; oFontIDs: {variable-size-array} ATSUFontIDPtr; var oFontFallbackMethod: ATSUFontFallbackMethod; var oActualFallbacksCount: ItemCount ): OSStatus; external name '_ATSUGetFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{ ---------------------------------------------------------------------------- }
{  Handle-based functions                                                      }
{ ---------------------------------------------------------------------------- }
{
 *  ATSUCreateTextLayoutWithTextHandle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTTypesetterCreateWithAttributedString,
 *    CTTypesetterCreateWithAttributedStringAndOptions,
 *    CTLineCreateWithAttributedString, CTLineCreateTruncatedLine, or
 *    CTLineCreateJustifiedLine instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    ATSUCreateTextLayoutWithTextPtr instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCreateTextLayoutWithTextHandle( iText: UniCharArrayHandle; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount; iNumberOfRuns: ItemCount; {const} iRunLengths: {variable-size-array} UniCharCountPtr; iStyles: {variable-size-array} ATSUStylePtr; var oTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCreateTextLayoutWithTextHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)


{
 *  ATSUSetTextHandleLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    ATSUSetTextPointerLocation instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUSetTextHandleLocation( iTextLayout: ATSUTextLayout; iText: UniCharArrayHandle; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount ): OSStatus; external name '_ATSUSetTextHandleLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)


{ ---------------------------------------------------------------------------- }
{  ATSUI idle processing (deprecated)                                          }
{ ---------------------------------------------------------------------------- }
{
 *  ATSUIdle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Summary:
 *    Performs background processing.
 *  
 *  Discussion:
 *    Current versions of ATSUI do not implement background processing
 *    for text layout objects. In Mac OS X, the function ATSUIdle does
 *    nothing.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUIdle( iTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{  Handle-based functions                                                      }
=======
{ ATSUI global font fallback functions                                        }
>>>>>>> origin/fixes_2.4
{ ---------------------------------------------------------------------------- }
{
 *  ATSUSetFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
 *  
 *  Summary:
 *    Sets font fallback behavior on a global basis.
 *  
 *  Discussion:
 *    Control of font fallback behavior on a global basis is no longer
 *    recommended. Object based font fallbacks are preferred. See the
 *    functions ATSUCreateFontFallbacks, ATSUDisposeFontFallbacks,
 *    ATSUSetObjFontFallbacks, and ATSUGetObjFontFallbacks, as well as
 *    the kATSULineFontFallbacksTag attribute for more information
 *    about object based font fallbacks.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUSetFontFallbacks( iFontFallbacksCount: ItemCount; {const} iFontIDs: {variable-size-array} ATSUFontIDPtr; iFontFallbackMethod: ATSUFontFallbackMethod ): OSStatus; external name '_ATSUSetFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  ATSUGetFontFallbacks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
 *  
 *  Summary:
 *    Gets the current global font fallback behavior.
 *  
 *  Discussion:
 *    Control of font fallback behavior on a global basis is no longer
 *    recommended. Object based font fallbacks are preferred. See the
 *    functions ATSUCreateFontFallbacks, ATSUDisposeFontFallbacks,
 *    ATSUSetObjFontFallbacks, and ATSUGetObjFontFallbacks, as well as
 *    the kATSULineFontFallbacksTag attribute for more information
 *    about object based font fallbacks.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.6 and later
 }
function ATSUGetFontFallbacks( iMaxFontFallbacksCount: ItemCount; oFontIDs: {variable-size-array} ATSUFontIDPtr; var oFontFallbackMethod: ATSUFontFallbackMethod; var oActualFallbacksCount: ItemCount ): OSStatus; external name '_ATSUGetFontFallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{ ---------------------------------------------------------------------------- }
{  Handle-based functions                                                      }
{ ---------------------------------------------------------------------------- }
{
 *  ATSUCreateTextLayoutWithTextHandle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTTypesetterCreateWithAttributedString,
 *    CTTypesetterCreateWithAttributedStringAndOptions,
 *    CTLineCreateWithAttributedString, CTLineCreateTruncatedLine, or
 *    CTLineCreateJustifiedLine instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    ATSUCreateTextLayoutWithTextPtr instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCreateTextLayoutWithTextHandle( iText: UniCharArrayHandle; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount; iNumberOfRuns: ItemCount; {const} iRunLengths: {variable-size-array} UniCharCountPtr; iStyles: {variable-size-array} ATSUStylePtr; var oTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUCreateTextLayoutWithTextHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)


{
 *  ATSUSetTextHandleLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API and CFAttributedStringSetAttributes instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    ATSUSetTextPointerLocation instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUSetTextHandleLocation( iTextLayout: ATSUTextLayout; iText: UniCharArrayHandle; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iTextTotalLength: UniCharCount ): OSStatus; external name '_ATSUSetTextHandleLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)


{ ---------------------------------------------------------------------------- }
{  ATSUI idle processing (deprecated)                                          }
{ ---------------------------------------------------------------------------- }
{
 *  ATSUIdle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed on MacOS X.
 *  
 *  Summary:
 *    Performs background processing.
 *  
 *  Discussion:
 *    Current versions of ATSUI do not implement background processing
 *    for text layout objects. In Mac OS X, the function ATSUIdle does
 *    nothing.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUIdle( iTextLayout: ATSUTextLayout ): OSStatus; external name '_ATSUIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)


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
