{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/MacLocales.h
 
     Contains:   Types & prototypes for locale functions
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1998-2012 by Apple Inc. All rights reserved.
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1998-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1998-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1998-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1998-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       MacLocales.p
=======
     File:       CarbonCore/MacLocales.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Types & prototypes for locale functions
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1998-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1998-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit MacLocales;
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
	{$setc TARGET_CPU_ARM64 := FALSE}
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
{$elifc defined __arm64__ and __arm64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

=======


>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}
=======
uses MacTypes,MacErrors;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,MacErrors;

>>>>>>> origin/fixes_2_2
=======
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}


{
   -------------------------------------------------------------------------------------------------
   TYPES & CONSTANTS
   -------------------------------------------------------------------------------------------------
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	LocaleRef = ^OpaqueLocaleRef; { an opaque type }
	OpaqueLocaleRef = record end;
=======
	LocaleRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	LocaleRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	LocaleRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	LocaleRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	LocalePartMask = UInt32;
const
{ bit set requests the following:}
	kLocaleLanguageMask = 1 shl 0; { ISO 639-1 or -2 language code (2 or 3 letters)}
	kLocaleLanguageVariantMask = 1 shl 1; { custom string for language variant}
	kLocaleScriptMask = 1 shl 2; { ISO 15924 script code (2 letters)}
	kLocaleScriptVariantMask = 1 shl 3; { custom string for script variant}
	kLocaleRegionMask = 1 shl 4; { ISO 3166 country/region code (2 letters)}
	kLocaleRegionVariantMask = 1 shl 5; { custom string for region variant}
	kLocaleAllPartsMask = $0000003F; { all of the above}

type
	LocaleOperationClass = FourCharCode;
{ constants for LocaleOperationClass are in UnicodeUtilities interfaces}
type
	LocaleAndVariantPtr = ^LocaleAndVariant;
	LocaleOperationVariant = FourCharCode;
	LocaleAndVariant = record
		locale: LocaleRef;
		opVariant: LocaleOperationVariant;
	end;

type
	LocaleNameMask = UInt32;
const
{ bit set requests the following:}
	kLocaleNameMask = 1 shl 0; { name of locale}
	kLocaleOperationVariantNameMask = 1 shl 1; { name of LocaleOperationVariant}
	kLocaleAndVariantNameMask = $00000003; { all of the above}

{
   -------------------------------------------------------------------------------------------------
   FUNCTION PROTOTYPES
   -------------------------------------------------------------------------------------------------
}

{ Convert to or from LocaleRefs (and related utilities)}

{
 *  LocaleRefFromLangOrRegionCode()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleRefFromLangOrRegionCode( lang: LangCode; region: RegionCode; var locale: LocaleRef ): OSStatus; external name '_LocaleRefFromLangOrRegionCode';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
type
	LocaleRef = ^SInt32; { an opaque type }
	LocalePartMask = UInt32;
const
{ bit set requests the following:}
	kLocaleLanguageMask = 1 shl 0; { ISO 639-1 or -2 language code (2 or 3 letters)}
	kLocaleLanguageVariantMask = 1 shl 1; { custom string for language variant}
	kLocaleScriptMask = 1 shl 2; { ISO 15924 script code (2 letters)}
	kLocaleScriptVariantMask = 1 shl 3; { custom string for script variant}
	kLocaleRegionMask = 1 shl 4; { ISO 3166 country/region code (2 letters)}
	kLocaleRegionVariantMask = 1 shl 5; { custom string for region variant}
	kLocaleAllPartsMask = $0000003F; { all of the above}

type
	LocaleOperationClass = FourCharCode;
{ constants for LocaleOperationClass are in UnicodeUtilities interfaces}
type
	LocaleAndVariantPtr = ^LocaleAndVariant;
	LocaleOperationVariant = FourCharCode;
	LocaleAndVariant = record
		locale: LocaleRef;
		opVariant: LocaleOperationVariant;
	end;

type
	LocaleNameMask = UInt32;
const
<<<<<<< HEAD
																{  bit set requests the following: }
	kLocaleNameMask				= $00000001;					{  name of locale }
	kLocaleOperationVariantNameMask = $00000002;				{  name of LocaleOperationVariant }
	kLocaleAndVariantNameMask	= $00000003;					{  all of the above }

	{
	   -------------------------------------------------------------------------------------------------
	   function PROTOTYPES
	   -------------------------------------------------------------------------------------------------
	}

	{  Convert to or from LocaleRefs (and related utilities) }

	{
	 *  LocaleRefFromLangOrRegionCode()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function LocaleRefFromLangOrRegionCode(lang: LangCode; region: RegionCode; var locale: LocaleRef): OSStatus; external name '_LocaleRefFromLangOrRegionCode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ bit set requests the following:}
	kLocaleNameMask = 1 shl 0; { name of locale}
	kLocaleOperationVariantNameMask = 1 shl 1; { name of LocaleOperationVariant}
	kLocaleAndVariantNameMask = $00000003; { all of the above}
>>>>>>> origin/fixes_2.4

{
   -------------------------------------------------------------------------------------------------
   FUNCTION PROTOTYPES
   -------------------------------------------------------------------------------------------------
}

{ Convert to or from LocaleRefs (and related utilities)}

{
 *  LocaleRefFromLangOrRegionCode()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleRefFromLocaleString( localeString: ConstCStringPtr; var locale: LocaleRef ): OSStatus; external name '_LocaleRefFromLocaleString';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleRefFromLangOrRegionCode( lang: LangCode; region: RegionCode; var locale: LocaleRef ): OSStatus; external name '_LocaleRefFromLangOrRegionCode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LocaleRefFromLocaleString()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
<<<<<<< HEAD
function LocaleRefFromLocaleString(localeString: ConstCStringPtr; var locale: LocaleRef): OSStatus; external name '_LocaleRefFromLocaleString';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function LocaleRefFromLocaleString( localeString: ConstCStringPtr; var locale: LocaleRef ): OSStatus; external name '_LocaleRefFromLocaleString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  LocaleRefGetPartString()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleRefGetPartString( locale: LocaleRef; partMask: LocalePartMask; maxStringLen: ByteCount; partString: CStringPtr ): OSStatus; external name '_LocaleRefGetPartString';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
<<<<<<< HEAD
function LocaleRefGetPartString(locale: LocaleRef; partMask: LocalePartMask; maxStringLen: ByteCount; var partString: char): OSStatus; external name '_LocaleRefGetPartString';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function LocaleRefGetPartString( locale: LocaleRef; partMask: LocalePartMask; maxStringLen: ByteCount; partString: CStringPtr ): OSStatus; external name '_LocaleRefGetPartString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  LocaleStringToLangAndRegionCodes()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Map a CFLocale-style locale string to old Script Manager LangCode
 *    and RegionCode values.
 *  
 *  Parameters:
 *    
 *    localeString:
 *      A null-terminated C-string version of a CFLocale-style locale
 *      identifier of the sort that could be passed to
 *      CFLocaleCreateCanonicalLocaleIdentifierFromString, based on BCP
 *      47 language tags: <http://www.rfc-editor.org/rfc/bcp/bcp47.txt>.
 *    
 *    lang:
 *      A pointer to a LangCode to be set from the locale identifier;
 *      will be set to langUnspecified or -1 if no language code can be
 *      derived from the identifier. May be NULL if region is not also
 *      NULL.
 *    
 *    region:
 *      A pointer to a RegionCode to be set from the locale identifier;
 *      will be set to -1 if no language code can be derived from the
 *      identifier. May be NULL if lang is not also NULL.
 *  
 *  Result:
 *    OSStatus, noErr if operation successful, otherwise paramErr or
 *    possibly other errors.
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 9.0 and later
 }
function LocaleStringToLangAndRegionCodes( localeString: ConstCStringPtr; var lang: LangCode; var region: RegionCode ): OSStatus; external name '_LocaleStringToLangAndRegionCodes';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{ Enumerate locales for a LocaleOperationClass }

{
 *  LocaleOperationCountLocales()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFLocaleCopyAvailableLocaleIdentifiers instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFLocaleCopyAvailableLocaleIdentifiers instead.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleOperationCountLocales( opClass: LocaleOperationClass; var localeCount: ItemCount ): OSStatus; external name '_LocaleOperationCountLocales';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  LocaleOperationGetLocales()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFXxxxx instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFLocaleCopyAvailableLocaleIdentifiers instead.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleOperationGetLocales( opClass: LocaleOperationClass; maxLocaleCount: ItemCount; var actualLocaleCount: ItemCount; localeVariantList: {variable-size-array} LocaleAndVariantPtr ): OSStatus; external name '_LocaleOperationGetLocales';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/cpstrnew
{ Get names for a locale (or a region's language)}

{
 *  LocaleGetName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFLocaleCopyDisplayNameForPropertyValue instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFLocaleCopyDisplayNameForPropertyValue instead.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleGetName( locale: LocaleRef; opVariant: LocaleOperationVariant; nameMask: LocaleNameMask; displayLocale: LocaleRef; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: {variable-size-array} UniCharPtr ): OSStatus; external name '_LocaleGetName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  LocaleCountNames()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFLocaleCopyAvailableLocaleIdentifiers instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFLocaleCopyAvailableLocaleIdentifiers instead.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleCountNames( locale: LocaleRef; opVariant: LocaleOperationVariant; nameMask: LocaleNameMask; var nameCount: ItemCount ): OSStatus; external name '_LocaleCountNames';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  LocaleGetIndName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFLocaleCopyAvailableLocaleIdentifiers and
 *    CFLocaleCopyDisplayNameForPropertyValue instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFLocaleCopyAvailableLocaleIdentifiers and
 *    CFLocaleCopyDisplayNameForPropertyValue instead.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleGetIndName( locale: LocaleRef; opVariant: LocaleOperationVariant; nameMask: LocaleNameMask; nameIndex: ItemCount; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: {variable-size-array} UniCharPtr; var displayLocale: LocaleRef ): OSStatus; external name '_LocaleGetIndName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/cpstrnew
{$ifc TARGET_CPU_64}
{
 *  LocaleGetRegionLanguageName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFLocaleCreateCanonicalLocaleIdentifierFromScriptManagerCodes
 *    and CFLocaleCopyDisplayNameForPropertyValue instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFLocaleCreateCanonicalLocaleIdentifierFromScriptManagerCodes and
 *    CFLocaleCopyDisplayNameForPropertyValue instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 9.0 and later
 }
function LocaleGetRegionLanguageName( region: RegionCode; var languageName: Str255 ): OSStatus; external name '_LocaleGetRegionLanguageName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{ Get names for a LocaleOperationClass}

{$endc} {TARGET_CPU_64}

=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 9.0 and later
 }
function LocaleStringToLangAndRegionCodes( localeString: ConstCStringPtr; var lang: LangCode; var region: RegionCode ): OSStatus; external name '_LocaleStringToLangAndRegionCodes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Enumerate locales for a LocaleOperationClass }

{
 *  LocaleOperationCountLocales()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleOperationCountLocales( opClass: LocaleOperationClass; var localeCount: ItemCount ): OSStatus; external name '_LocaleOperationCountLocales';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LocaleOperationGetLocales()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleOperationGetLocales( opClass: LocaleOperationClass; maxLocaleCount: ItemCount; var actualLocaleCount: ItemCount; localeVariantList: {variable-size-array} LocaleAndVariantPtr ): OSStatus; external name '_LocaleOperationGetLocales';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Get names for a locale (or a region's language)}

{
 *  LocaleGetName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleGetName( locale: LocaleRef; opVariant: LocaleOperationVariant; nameMask: LocaleNameMask; displayLocale: LocaleRef; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: {variable-size-array} UniCharPtr ): OSStatus; external name '_LocaleGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LocaleCountNames()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleCountNames( locale: LocaleRef; opVariant: LocaleOperationVariant; nameMask: LocaleNameMask; var nameCount: ItemCount ): OSStatus; external name '_LocaleCountNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LocaleGetIndName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleGetIndName( locale: LocaleRef; opVariant: LocaleOperationVariant; nameMask: LocaleNameMask; nameIndex: ItemCount; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: {variable-size-array} UniCharPtr; var displayLocale: LocaleRef ): OSStatus; external name '_LocaleGetIndName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc TARGET_CPU_64}
{
 *  LocaleGetRegionLanguageName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFLocaleCreateCanonicalLocaleIdentifierFromScriptManagerCodes
 *    and CFLocaleCopyDisplayNameForPropertyValue instead.
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    CFLocaleCreateCanonicalLocaleIdentifierFromScriptManagerCodes and
 *    CFLocaleCopyDisplayNameForPropertyValue instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 9.0 and later
 }
function LocaleGetRegionLanguageName( region: RegionCode; var languageName: Str255 ): OSStatus; external name '_LocaleGetRegionLanguageName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Get names for a LocaleOperationClass}

{$endc} {TARGET_CPU_64}

<<<<<<< HEAD
{  Get names for a LocaleOperationClass }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  LocaleOperationGetName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleOperationGetName( opClass: LocaleOperationClass; displayLocale: LocaleRef; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: {variable-size-array} UniCharPtr ): OSStatus; external name '_LocaleOperationGetName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
<<<<<<< HEAD
function LocaleOperationGetName(opClass: LocaleOperationClass; displayLocale: LocaleRef; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: UniCharPtr): OSStatus; external name '_LocaleOperationGetName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function LocaleOperationGetName( opClass: LocaleOperationClass; displayLocale: LocaleRef; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: {variable-size-array} UniCharPtr ): OSStatus; external name '_LocaleOperationGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  LocaleOperationCountNames()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleOperationCountNames( opClass: LocaleOperationClass; var nameCount: ItemCount ): OSStatus; external name '_LocaleOperationCountNames';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
<<<<<<< HEAD
function LocaleOperationCountNames(opClass: LocaleOperationClass; var nameCount: ItemCount): OSStatus; external name '_LocaleOperationCountNames';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function LocaleOperationCountNames( opClass: LocaleOperationClass; var nameCount: ItemCount ): OSStatus; external name '_LocaleOperationCountNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  LocaleOperationGetIndName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleOperationGetIndName( opClass: LocaleOperationClass; nameIndex: ItemCount; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: {variable-size-array} UniCharPtr; var displayLocale: LocaleRef ): OSStatus; external name '_LocaleOperationGetIndName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in LocalesLib 8.6 and later
 }
function LocaleOperationGetIndName( opClass: LocaleOperationClass; nameIndex: ItemCount; maxNameLen: UniCharCount; var actualNameLen: UniCharCount; displayName: {variable-size-array} UniCharPtr; var displayLocale: LocaleRef ): OSStatus; external name '_LocaleOperationGetIndName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
