{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       ATS/ATSFont.h
 
     Contains:   Public interface to the font access and data management functions of ATS.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    ATS
 
     Copyright:  © 2000-2012 by Apple Inc., all rights reserved.
=======
     DRI:        Julio Gonzalez
 
     Copyright:  © 2000-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}

{  Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
     DRI:        Julio Gonzalez
 
     Copyright:  © 2000-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
>>>>>>> graemeg/cpstrnew
=======
     DRI:        Julio Gonzalez
 
     Copyright:  © 2000-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
>>>>>>> graemeg/cpstrnew
=======
     DRI:        Julio Gonzalez
 
     Copyright:  © 2000-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
>>>>>>> origin/cpstrnew
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                ATSFont.i
                     Revision:        
                     Dated:           
                     Last change by:  
                     Last comment:    
 
     Bugs:       Report bugs to Radar component "System Interfaces", "Latest"
                 List the version information (from above) in the Problem Description.
 
}

{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

>>>>>>> graemeg/cpstrnew
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
     File:       ATSFont.p
=======
     File:       ATS/ATSFont.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Public interface to the font access and data management functions of ATS.
 
     DRI:        Julio Gonzalez
 
     Copyright:  © 2000-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                ATSFont.i
                     Revision:        
                     Dated:           
                     Last change by:  
                     Last comment:    
 
     Bugs:       Report bugs to Radar component "System Interfaces", "Latest"
                 List the version information (from above) in the Problem Description.
 
}

{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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

unit ATSFont;
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
=======
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
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
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
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
=======
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CFBase,CFRunLoop,CFPropertyList,ATSTypes,CFString,CFURL,Files,TextCommon,SFNTTypes;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}
=======
{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}
>>>>>>> origin/cpstrnew


const
	kATSOptionFlagsDefault = 0;
	kATSOptionFlagsComposeFontPostScriptName = 1 shl 0; { ATSFontGetPostScriptName }
	kATSOptionFlagsUseDataForkAsResourceFork = 1 shl 8; { ATSFontActivateFromFileSpecification }
	kATSOptionFlagsUseResourceFork = 2 shl 8;
	kATSOptionFlagsUseDataFork = 3 shl 8;
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,CFBase,CFRunLoop,CFPropertyList,ATSTypes,CFString,Files,TextCommon,SFNTTypes;
=======
uses MacTypes,CFBase,CFRunLoop,CFPropertyList,ATSTypes,CFString,CFURL,Files,TextCommon,SFNTTypes;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


const
	kATSOptionFlagsDefault		= 0;
	kATSOptionFlagsComposeFontPostScriptName = $01;				{  ATSFontGetPostScriptName  }
	kATSOptionFlagsUseDataForkAsResourceFork = $0100;			{  ATSFontActivateFromFileSpecification  }
	kATSOptionFlagsUseResourceFork = $0200;
	kATSOptionFlagsUseDataFork	= $0300;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

	kATSIterationCompleted		= -980;
	kATSInvalidFontFamilyAccess	= -981;
	kATSInvalidFontAccess		= -982;
	kATSIterationScopeModified	= -983;
	kATSInvalidFontTableAccess	= -984;
	kATSInvalidFontContainerAccess = -985;
<<<<<<< HEAD
<<<<<<< HEAD
    kATSInvalidGlyphAccess        = -986;


{ Activation Option Flags }
type
	ATSFontContext = UInt32;
const
	kATSFontContextUnspecified = 0;
	kATSFontContextGlobal = 1;
	kATSFontContextLocal = 2;

const
	kATSOptionFlagsActivateDisabled = $00000001 shl 5; { Used by activate to activate fonts in the disabled state }
	kATSOptionFlagsProcessSubdirectories = $00000001 shl 6; { Used by activation/deactivation & iteration }
	kATSOptionFlagsDoNotNotify = $00000001 shl 7; { Do not notify after global activation/deactivation }
	kATSOptionFlagsRecordPersistently = $00000001 shl 18; { Used by activated/deactivation to record/remove font references in persistent store to be remembered at next login }

{ Iteration Option Flags }
const
	kATSOptionFlagsIterateByPrecedenceMask = $00000001 shl 5; { Fonts returned from highest to lowest precedece }
	kATSOptionFlagsIncludeDisabledMask = $00000001 shl 7; { Disabled Fonts will show up in iteration, also valid for ATSFontFindFromContainer }
	kATSOptionFlagsIterationScopeMask = $00000007 shl 12; { Mask option bits 12-14 for iteration scopes }
	kATSOptionFlagsDefaultScope = $00000000 shl 12;
	kATSOptionFlagsUnRestrictedScope = $00000001 shl 12;
	kATSOptionFlagsRestrictedScope = $00000002 shl 12;

type
	ATSFontFamilyApplierFunction = function( iFamily: ATSFontFamilyRef; iRefCon: UnivPtr ): OSStatus;
	ATSFontApplierFunction = function( iFont: ATSFontRef; iRefCon: UnivPtr ): OSStatus;
	ATSFontFamilyIterator_ = record end;
	ATSFontFamilyIterator = ^ATSFontFamilyIterator_;
	ATSFontIterator_ = record end;
	ATSFontIterator = ^ATSFontIterator_;
const
	kATSFontFilterCurrentVersion = 0;

type
	ATSFontFilterSelector = SInt32;
=======
=======
>>>>>>> origin/fixes_2_2
  kATSInvalidGlyphAccess        = -986;


{ Activation Option Flags }
type
	ATSFontContext = UInt32;
const
	kATSFontContextUnspecified = 0;
	kATSFontContextGlobal = 1;
	kATSFontContextLocal = 2;

const
	kATSOptionFlagsActivateDisabled = $00000001 shl 5; { Used by activate to activate fonts in the disabled state }
	kATSOptionFlagsProcessSubdirectories = $00000001 shl 6; { Used by activation/deactivation & iteration }
	kATSOptionFlagsDoNotNotify = $00000001 shl 7; { Do not notify after global activation/deactivation }
	kATSOptionFlagsRecordPersistently = $00000001 shl 18; { Used by activated/deactivation to record/remove font references in persistent store to be remembered at next login }

{ Iteration Option Flags }
const
	kATSOptionFlagsIterateByPrecedenceMask = $00000001 shl 5; { Fonts returned from highest to lowest precedece }
	kATSOptionFlagsIncludeDisabledMask = $00000001 shl 7; { Disabled Fonts will show up in iteration, also valid for ATSFontFindFromContainer }
	kATSOptionFlagsIterationScopeMask = $00000007 shl 12; { Mask option bits 12-14 for iteration scopes }
	kATSOptionFlagsDefaultScope = $00000000 shl 12;
	kATSOptionFlagsUnRestrictedScope = $00000001 shl 12;
	kATSOptionFlagsRestrictedScope = $00000002 shl 12;

type
	ATSFontFamilyApplierFunction = function( iFamily: ATSFontFamilyRef; iRefCon: UnivPtr ): OSStatus;
	ATSFontApplierFunction = function( iFont: ATSFontRef; iRefCon: UnivPtr ): OSStatus;
	ATSFontFamilyIterator_ = record end;
	ATSFontFamilyIterator = ^ATSFontFamilyIterator_;
	ATSFontIterator_ = record end;
	ATSFontIterator = ^ATSFontIterator_;
const
	kATSFontFilterCurrentVersion = 0;

type
<<<<<<< HEAD
	ATSFontFilterSelector 		= SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	ATSFontFilterSelector = SInt32;
>>>>>>> origin/fixes_2.4
const
	kATSFontFilterSelectorUnspecified = 0;
	kATSFontFilterSelectorGeneration = 3;
	kATSFontFilterSelectorFontFamily = 7;
	kATSFontFilterSelectorFontFamilyApplierFunction = 8;
	kATSFontFilterSelectorFontApplierFunction = 9;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSFileReferenceFilterSelector = 10;
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
	kATSFileReferenceFilterSelector = 10;
>>>>>>> origin/fixes_2.4

type
	ATSFontFilterPtr = ^ATSFontFilter;
	ATSFontFilter = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		version: UInt32;
		filterSelector: ATSFontFilterSelector;
=======
		version:				UInt32;
		filterSelector:			ATSFontFilterSelector;
>>>>>>> graemeg/fixes_2_2
=======
		version:				UInt32;
		filterSelector:			ATSFontFilterSelector;
>>>>>>> origin/fixes_2_2
=======
		version: UInt32;
		filterSelector: ATSFontFilterSelector;
>>>>>>> origin/fixes_2.4
		case SInt16 of
		0: (
			generationFilter:	ATSGeneration;
			);
		1: (
			fontFamilyFilter:	ATSFontFamilyRef;
			);
		2: (
			fontFamilyApplierFunctionFilter: ATSFontFamilyApplierFunction;
			);
		3: (
			fontApplierFunctionFilter: ATSFontApplierFunction;
			);
<<<<<<< HEAD
<<<<<<< HEAD
		4: (
			fontFileRefFilter: {const} FSRefPtr;
			);
	end;
{ Notification related }

	ATSFontNotificationRef_ = record end;
	ATSFontNotificationRef = ^ATSFontNotificationRef_;
	ATSFontNotificationRefPtr = ^ATSFontNotificationRef;
	ATSFontNotificationInfoRef_ = record end;
	ATSFontNotificationInfoRef = ^ATSFontNotificationInfoRef_;
=======
=======
>>>>>>> origin/fixes_2_2
	end;
{ Notification related }

	ATSFontNotificationRef_ = record end;
	ATSFontNotificationRef = ^ATSFontNotificationRef_;
	ATSFontNotificationRefPtr = ^ATSFontNotificationRef;
<<<<<<< HEAD
	ATSFontNotificationInfoRef    = ^SInt32; { an opaque 32-bit type }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	ATSFontNotificationInfoRef_ = record end;
	ATSFontNotificationInfoRef = ^ATSFontNotificationInfoRef_;
>>>>>>> origin/fixes_2.4
	ATSFontNotificationInfoRefPtr = ^ATSFontNotificationInfoRef;

{
 *  ATSFontNotifyOption
 *  
 *  Discussion:
 *    Options used with ATSFontNotificationSubscribe.  Any of the
 *    options that follow may be used together in order to alter the
 *    default behavior of ATS notifications.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	ATSFontNotifyOption = SInt32;
const
{
   * Default behavior of ATSFontNotificationSubscribe.
   }
	kATSFontNotifyOptionDefault = 0;
=======
=======
>>>>>>> origin/fixes_2_2
type
=======

>>>>>>> origin/fixes_2.4
	ATSFontNotifyOption = SInt32;
const
{
   * Default behavior of ATSFontNotificationSubscribe.
   }
<<<<<<< HEAD
  kATSFontNotifyOptionDefault   = 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSFontNotifyOptionDefault = 0;
>>>>>>> origin/fixes_2.4

  {
   * Normally applications will only receive ATS notifications while in
   * the foreground.   If suspended, the notification will be delivered
   * when then application comes to the foreground.  This is the
   * default.  You should set this option if you are a server or tool
   * that performs font management functions and require immediate
   * notification when something changes.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSFontNotifyOptionReceiveWhileSuspended = 1 shl 0;

=======
  kATSFontNotifyOptionReceiveWhileSuspended = 1 shl 0;
>>>>>>> graemeg/fixes_2_2
=======
  kATSFontNotifyOptionReceiveWhileSuspended = 1 shl 0;
>>>>>>> origin/fixes_2_2
=======
	kATSFontNotifyOptionReceiveWhileSuspended = 1 shl 0;

>>>>>>> origin/fixes_2.4

{
 *  ATSFontNotifyAction
 *  
 *  Discussion:
 *    Used with ATSFontNotify.   The following is a list of actions you
 *    might wish the ATS server to perform and notify clients if
 *    appropriate.
 }
type
	ATSFontNotifyAction = SInt32;
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
=======

  {
>>>>>>> graemeg/fixes_2_2
=======

  {
>>>>>>> origin/fixes_2_2
=======
{
>>>>>>> origin/fixes_2.4
   * Used after a batch (de)activation of fonts occurs.   Typically the
   * caller has exercised multiple global (De)Activation calls with the
   * kATSOptionFlagsDoNotNotify set. Once all calls are completed, one
   * may use ATSFontNotify with this action to ask ATS to notify all
   * clients.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSFontNotifyActionFontsChanged = 1;
=======
  kATSFontNotifyActionFontsChanged = 1;
>>>>>>> graemeg/fixes_2_2
=======
  kATSFontNotifyActionFontsChanged = 1;
>>>>>>> origin/fixes_2_2
=======
	kATSFontNotifyActionFontsChanged = 1;
>>>>>>> origin/fixes_2.4

  {
   * The ATS system with the help of the Finder keeps track of changes
   * to any of the font directories in the system domains ( System,
   * Local, Network, User, & Classic). However, one may wish to
   * add/remove fonts to these locations programmatically. This action
   * is used to let ATS server to rescan these directories and post
   * notifications if necessary.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSFontNotifyActionDirectoriesChanged = 2;
=======
  kATSFontNotifyActionDirectoriesChanged = 2;
>>>>>>> graemeg/fixes_2_2
=======
  kATSFontNotifyActionDirectoriesChanged = 2;
>>>>>>> origin/fixes_2_2
=======
	kATSFontNotifyActionDirectoriesChanged = 2;
>>>>>>> origin/fixes_2.4


{
 *  ATSNotificationCallback
 *  
 *  Discussion:
 *    Callback delivered for ATS notifications.
 *  
 *  Parameters:
 *    
 *    info:
 *      Parameter is placed here for future improvements.  Initially
 *      the contents of this parameter will be NULL.
 *    
 *    refCon:
 *      User data/state to be supplied to callback function
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
type
	ATSNotificationCallback = procedure( info: ATSFontNotificationInfoRef; refCon: UnivPtr );
{ ----------------------------------------------------------------------------------------- }
{ Font container                                                                            }
{ ----------------------------------------------------------------------------------------- }
{
 *  ATSGetGeneration()
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
  Register for kCTFontManagerRegisteredFontsChangedNotification notifications
}
function ATSGetGeneration: ATSGeneration; external name '_ATSGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)


=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ATSGetGeneration: ATSGeneration; external name '_ATSGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc not TARGET_CPU_64}
{
 *  ATSFontActivateFromFileSpecification()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontActivateFromFileReference instead.
 *  
 *  Summary:
 *    Activates one or more fonts from a file specification.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the file specification that specifies the name and
 *      location of a file or directory that contains the font data you
 *      want to activate.
 *    
 *    iContext:
 *      A value that specifies the context of the activated font.
 *    
 *    iFormat:
 *      A value that represents the format identifier of the font. Pass
 *      kATSFontFormatUnspecified as the system automatically
 *      determines the format of the font.
 *    
 *    iReserved:
 *      This parameter is currently reserved for future use, so you
 *      should pass NULL.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that is activated
 *      from the file specification. You need this reference when you
 *      deactivate the font by calling the function ATSFontDeactivate.
 *  
 *  Result:
 *    noErr Activated successfully
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
<<<<<<< HEAD
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
{
  Use CTFontManagerRegisterFontsForURL() or CTFontManagerRegisterFontsForURLs()
}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
function ATSFontActivateFromFileSpecification( const (*var*) iFile: FSSpec; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromFileSpecification';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc}  { TARGET_CPU_64 }
=======
function ATSGetGeneration: ATSGeneration; external name '_ATSGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSFontActivateFromFileReference()
 *  
 *  Summary:
 *    Activates one or more fonts from a file reference.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
 *  ATSFontActivateFromFileSpecification()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontActivateFromFileReference instead.
 *  
 *  Summary:
 *    Activates one or more fonts from a file specification.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the file specification that specifies the name and
 *      location of a file or directory that contains the font data you
 *      want to activate.
 *    
 *    iContext:
 *      A value that specifies the context of the activated font.
 *    
 *    iFormat:
 *      A value that represents the format identifier of the font. Pass
 *      kATSFontFormatUnspecified as the system automatically
 *      determines the format of the font.
 *    
<<<<<<< HEAD
 *    iRefCon:
=======
 *    iReserved:
>>>>>>> graemeg/cpstrnew
 *      This parameter is currently reserved for future use, so you
 *      should pass NULL.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that is activated
 *      from the file specification. You need this reference when you
 *      deactivate the font by calling the function ATSFontDeactivate.
 *  
 *  Result:
 *    noErr Activated successfully
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
{
   Use CTFontManagerRegisterFontsForURL() or CTFontManagerRegisterFontsForURLs()
}
function ATSFontActivateFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iFormat: ATSFontFormat; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontActivateFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iFormat: ATSFontFormat; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontActivateFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iFormat: ATSFontFormat; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew

=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
=======
>>>>>>> origin/cpstrnew
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontActivateFromFileSpecification( const (*var*) iFile: FSSpec; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromFileSpecification';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc}  { TARGET_CPU_64 }
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{
 *  ATSFontActivateFromFileReference()
 *  
 *  Summary:
 *    Activates one or more fonts from a file reference.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the file specification that specifies the name and
 *      location of a file or directory that contains the font data you
 *      want to activate.
 *    
 *    iContext:
 *      A value that specifies the context of the activated font.
 *    
 *    iFormat:
 *      A value that represents the format identifier of the font. Pass
 *      kATSFontFormatUnspecified as the system automatically
 *      determines the format of the font.
 *    
 *    iRefCon:
 *      This parameter is currently reserved for future use, so you
 *      should pass NULL.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that is activated
 *      from the file specification. You need this reference when you
 *      deactivate the font by calling the function ATSFontDeactivate.
 *  
 *  Result:
 *    noErr Activated successfully
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
<<<<<<< HEAD
=======
=======
 *  
 *  Availability:
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontActivateFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iFormat: ATSFontFormat; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
type ATSNotificationCallback = procedure( info: ATSFontNotificationInfoRef; refCon: UnivPtr );

	{	 ----------------------------------------------------------------------------------------- 	}
	{	 Font container                                                                            	}
	{	 ----------------------------------------------------------------------------------------- 	}
	{
	 *  ATSGetGeneration()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   not available
	 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
=======
>>>>>>> origin/fixes_2.4
function ATSGetGeneration: ATSGeneration; external name '_ATSGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  ATSFontActivateFromFileSpecification()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontActivateFromFileReference instead.
 *  
 *  Summary:
 *    Activates one or more fonts from a file specification.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the file specification that specifies the name and
 *      location of a file or directory that contains the font data you
 *      want to activate.
 *    
 *    iContext:
 *      A value that specifies the context of the activated font.
 *    
 *    iFormat:
 *      A value that represents the format identifier of the font. Pass
 *      kATSFontFormatUnspecified as the system automatically
 *      determines the format of the font.
 *    
 *    iReserved:
 *      This parameter is currently reserved for future use, so you
 *      should pass NULL.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that is activated
 *      from the file specification. You need this reference when you
 *      deactivate the font by calling the function ATSFontDeactivate.
 *  
 *  Result:
 *    noErr Activated successfully
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontActivateFromFileSpecification(const (*var*) iFile: FSSpec; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef): OSStatus; external name '_ATSFontActivateFromFileSpecification';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontActivateFromFileSpecification( const (*var*) iFile: FSSpec; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromFileSpecification';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc}  { TARGET_CPU_64 }
>>>>>>> origin/fixes_2.4

{
 *  ATSFontActivateFromFileReference()
 *  
 *  Summary:
 *    Activates one or more fonts from a file reference.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the file specification that specifies the name and
 *      location of a file or directory that contains the font data you
 *      want to activate.
 *    
 *    iContext:
 *      A value that specifies the context of the activated font.
 *    
 *    iFormat:
 *      A value that represents the format identifier of the font. Pass
 *      kATSFontFormatUnspecified as the system automatically
 *      determines the format of the font.
 *    
 *    iRefCon:
 *      This parameter is currently reserved for future use, so you
 *      should pass NULL.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that is activated
 *      from the file specification. You need this reference when you
 *      deactivate the font by calling the function ATSFontDeactivate.
 *  
 *  Result:
 *    noErr Activated successfully
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
  Use CTFontManagerRegisterFontForData() or CGFontCreateWithDataProvider() w/ CTFontManagerRegisterGraphicsFont()
}
function ATSFontActivateFromMemory( iData: LogicalAddress; iLength: ByteCount; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontActivateFromMemory( iData: LogicalAddress; iLength: ByteCount; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontActivateFromMemory( iData: LogicalAddress; iLength: ByteCount; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontActivateFromMemory( iData: LogicalAddress; iLength: ByteCount; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontActivateFromMemory( iData: LogicalAddress; iLength: ByteCount; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ATSFontActivateFromMemory(iData: LogicalAddress; iLength: ByteCount; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef): OSStatus; external name '_ATSFontActivateFromMemory';
>>>>>>> origin/fixes_2_2

{
 *  ATSFontDeactivate()
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
{ "Use CTFontManagerUnregisterFontsForURL() or CTFontManagerUnregisterFontsForURLs()" }
function ATSFontDeactivate( iContainer: ATSFontContainerRef; iRefCon: UnivPtr; iOptions: ATSOptionFlags ): OSStatus; external name '_ATSFontDeactivate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)


{
 *  ATSFontGetContainerFromFileReference()
 *  
 *  Summary:
 *    Get the font container reference associated with an activated
 *    file reference.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the valid file reference that specificies the
 *      activated font file for which to get the container.
 *    
 *    iContext:
 *      The context that the font file is accessible too. If
 *      kATSFontContextGlobal is specified this function will return
 *      only a valid font container reference that was activated in
 *      kATSFontContextGlobal. If kATSFontContextDefault or
 *      kATSFontContextLocal is used, the container returned will
 *      adhere to precedence rules. In which case a container activated
 *      in kATSFontContextLocal will be preferred over one activated in
 *      kATSFontContextGlobal.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container representing the
 *      file reference activated in the specified context. On error or
 *      for a file that is not activated, this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    paramErr One or more parameters are invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
{ "Use CTFontCopyAttribute() with kCTFontURLAttribute." }
function ATSFontGetContainerFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainerFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)


{
 *  ATSFontGetContainer()
 *  
 *  Summary:
 *    Gets the font container reference for the font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that was used to
 *      activate the font reference. On error this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    kATSInvalidFontContainerAccess The font container is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
{ "Use CTFontCopyAttribute() with kCTFontURLAttribute." }
function ATSFontGetContainer( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
=======
>>>>>>> graemeg/cpstrnew
 }
function ATSFontDeactivate( iContainer: ATSFontContainerRef; iRefCon: UnivPtr; iOptions: ATSOptionFlags ): OSStatus; external name '_ATSFontDeactivate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ATSFontGetContainerFromFileReference()
 *  
 *  Summary:
 *    Get the font container reference associated with an activated
 *    file reference.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the valid file reference that specificies the
 *      activated font file for which to get the container.
 *    
 *    iContext:
 *      The context that the font file is accessible too. If
 *      kATSFontContextGlobal is specified this function will return
 *      only a valid font container reference that was activated in
 *      kATSFontContextGlobal. If kATSFontContextDefault or
 *      kATSFontContextLocal is used, the container returned will
 *      adhere to precedence rules. In which case a container activated
 *      in kATSFontContextLocal will be preferred over one activated in
 *      kATSFontContextGlobal.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container representing the
 *      file reference activated in the specified context. On error or
 *      for a file that is not activated, this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    paramErr One or more parameters are invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetContainerFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainerFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontGetContainer()
 *  
 *  Summary:
 *    Gets the font container reference for the font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that was used to
 *      activate the font reference. On error this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    kATSInvalidFontContainerAccess The font container is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 }
function ATSFontGetContainer( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  ATSFontSetEnabled()
 *  
 *  Summary:
 *    Sets a font's state to enabled or disabled.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    iEnabled:
 *      The state to set the font to. True for enabled, false for
 *      disabled.
 *  
 *  Result:
 *    kATSInvalidFontAccess The font reference is invalid in the
 *    current application context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
{ "Use CTFontManagerEnableFontDescriptors()" }
function ATSFontSetEnabled( iFont: ATSFontRef; iOptions: ATSOptionFlags; iEnabled: Boolean ): OSStatus; external name '_ATSFontSetEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontSetEnabled( iFont: ATSFontRef; iOptions: ATSOptionFlags; iEnabled: Boolean ): OSStatus; external name '_ATSFontSetEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
 }
=======
 }
function ATSFontDeactivate( iContainer: ATSFontContainerRef; iRefCon: UnivPtr; iOptions: ATSOptionFlags ): OSStatus; external name '_ATSFontDeactivate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ATSFontGetContainerFromFileReference()
 *  
 *  Summary:
 *    Get the font container reference associated with an activated
 *    file reference.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the valid file reference that specificies the
 *      activated font file for which to get the container.
 *    
 *    iContext:
 *      The context that the font file is accessible too. If
 *      kATSFontContextGlobal is specified this function will return
 *      only a valid font container reference that was activated in
 *      kATSFontContextGlobal. If kATSFontContextDefault or
 *      kATSFontContextLocal is used, the container returned will
 *      adhere to precedence rules. In which case a container activated
 *      in kATSFontContextLocal will be preferred over one activated in
 *      kATSFontContextGlobal.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container representing the
 *      file reference activated in the specified context. On error or
 *      for a file that is not activated, this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    paramErr One or more parameters are invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetContainerFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainerFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontGetContainer()
 *  
 *  Summary:
 *    Gets the font container reference for the font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that was used to
 *      activate the font reference. On error this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    kATSInvalidFontContainerAccess The font container is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
>>>>>>> graemeg/cpstrnew
=======
 }
function ATSFontDeactivate( iContainer: ATSFontContainerRef; iRefCon: UnivPtr; iOptions: ATSOptionFlags ): OSStatus; external name '_ATSFontDeactivate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ATSFontGetContainerFromFileReference()
 *  
 *  Summary:
 *    Get the font container reference associated with an activated
 *    file reference.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the valid file reference that specificies the
 *      activated font file for which to get the container.
 *    
 *    iContext:
 *      The context that the font file is accessible too. If
 *      kATSFontContextGlobal is specified this function will return
 *      only a valid font container reference that was activated in
 *      kATSFontContextGlobal. If kATSFontContextDefault or
 *      kATSFontContextLocal is used, the container returned will
 *      adhere to precedence rules. In which case a container activated
 *      in kATSFontContextLocal will be preferred over one activated in
 *      kATSFontContextGlobal.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container representing the
 *      file reference activated in the specified context. On error or
 *      for a file that is not activated, this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    paramErr One or more parameters are invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetContainerFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainerFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontGetContainer()
 *  
 *  Summary:
 *    Gets the font container reference for the font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that was used to
 *      activate the font reference. On error this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    kATSInvalidFontContainerAccess The font container is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
>>>>>>> origin/cpstrnew
function ATSFontGetContainer( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontSetEnabled()
 *  
 *  Summary:
 *    Sets a font's state to enabled or disabled.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    iEnabled:
 *      The state to set the font to. True for enabled, false for
 *      disabled.
 *  
 *  Result:
 *    kATSInvalidFontAccess The font reference is invalid in the
 *    current application context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontSetEnabled( iFont: ATSFontRef; iOptions: ATSOptionFlags; iEnabled: Boolean ): OSStatus; external name '_ATSFontSetEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{
 *  ATSFontIsEnabled()
 *  
 *  Summary:
 *    Returns true if the font is enabled.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference
 *  
 *  Result:
 *    false The font is disabled.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCopyAttribute() with kCTFontEnabledAttribute" }
function ATSFontIsEnabled( iFont: ATSFontRef ): Boolean; external name '_ATSFontIsEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontIsEnabled( iFont: ATSFontRef ): Boolean; external name '_ATSFontIsEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontIsEnabled( iFont: ATSFontRef ): Boolean; external name '_ATSFontIsEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontIsEnabled( iFont: ATSFontRef ): Boolean; external name '_ATSFontIsEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontIsEnabled( iFont: ATSFontRef ): Boolean; external name '_ATSFontIsEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontActivateFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iFormat: ATSFontFormat; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontActivateFromMemory()
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontActivateFromMemory( iData: LogicalAddress; iLength: ByteCount; iContext: ATSFontContext; iFormat: ATSFontFormat; iReserved: UnivPtr; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontActivateFromMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ATSFontDeactivate()
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontDeactivate( iContainer: ATSFontContainerRef; iRefCon: UnivPtr; iOptions: ATSOptionFlags ): OSStatus; external name '_ATSFontDeactivate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ATSFontGetContainerFromFileReference()
 *  
 *  Summary:
 *    Get the font container reference associated with an activated
 *    file reference.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFile:
 *      A pointer to the valid file reference that specificies the
 *      activated font file for which to get the container.
 *    
 *    iContext:
 *      The context that the font file is accessible too. If
 *      kATSFontContextGlobal is specified this function will return
 *      only a valid font container reference that was activated in
 *      kATSFontContextGlobal. If kATSFontContextDefault or
 *      kATSFontContextLocal is used, the container returned will
 *      adhere to precedence rules. In which case a container activated
 *      in kATSFontContextLocal will be preferred over one activated in
 *      kATSFontContextGlobal.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container representing the
 *      file reference activated in the specified context. On error or
 *      for a file that is not activated, this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    paramErr One or more parameters are invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetContainerFromFileReference( const (*var*) iFile: FSRef; iContext: ATSFontContext; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainerFromFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontGetContainer()
 *  
 *  Summary:
 *    Gets the font container reference for the font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    oContainer:
 *      On output, a reference to the font container that was used to
 *      activate the font reference. On error this will be set to
 *      kATSFontContainerRefUnspecified.
 *  
 *  Result:
 *    kATSInvalidFontContainerAccess The font container is invalid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontDeactivate(iContainer: ATSFontContainerRef; iRefCon: UnivPtr; iOptions: ATSOptionFlags): OSStatus; external name '_ATSFontDeactivate';
>>>>>>> graemeg/fixes_2_2
=======
function ATSFontGetContainer( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oContainer: ATSFontContainerRef ): OSStatus; external name '_ATSFontGetContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontSetEnabled()
 *  
 *  Summary:
 *    Sets a font's state to enabled or disabled.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference.
 *    
 *    iOptions:
 *      An options flag.  See developer documentation for appropriate
 *      flags.
 *    
 *    iEnabled:
 *      The state to set the font to. True for enabled, false for
 *      disabled.
 *  
 *  Result:
 *    kATSInvalidFontAccess The font reference is invalid in the
 *    current application context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontSetEnabled( iFont: ATSFontRef; iOptions: ATSOptionFlags; iEnabled: Boolean ): OSStatus; external name '_ATSFontSetEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontIsEnabled()
 *  
 *  Summary:
 *    Returns true if the font is enabled.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      The font reference
 *  
 *  Result:
 *    false The font is disabled.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontIsEnabled( iFont: ATSFontRef ): Boolean; external name '_ATSFontIsEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ ----------------------------------------------------------------------------------------- }
{ Font family                                                                               }
{ ----------------------------------------------------------------------------------------- }
{
 *  ATSFontFamilyApplyFunction()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontManagerCopyAvailableFontFamilyNames()" }
function ATSFontFamilyApplyFunction( iFunction: ATSFontFamilyApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontFamilyApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFamilyApplyFunction( iFunction: ATSFontFamilyApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontFamilyApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyApplyFunction( iFunction: ATSFontFamilyApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontFamilyApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyApplyFunction( iFunction: ATSFontFamilyApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontFamilyApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyApplyFunction( iFunction: ATSFontFamilyApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontFamilyApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
 *  Availability:
=======
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontDeactivate(iContainer: ATSFontContainerRef; iRefCon: UnivPtr; iOptions: ATSOptionFlags): OSStatus; external name '_ATSFontDeactivate';

{ ----------------------------------------------------------------------------------------- }
{ Font family                                                                               }
{ ----------------------------------------------------------------------------------------- }
{
 *  ATSFontFamilyApplyFunction()
 *  
 *  Availability:
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ATSFontFamilyApplyFunction(iFunction: ATSFontFamilyApplierFunction; iRefCon: UnivPtr): OSStatus; external name '_ATSFontFamilyApplyFunction';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyApplyFunction( iFunction: ATSFontFamilyApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontFamilyApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyIteratorCreate()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontManagerCopyAvailableFontFamilyNames()" }
function ATSFontFamilyIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFamilyIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyIteratorCreate(iContext: ATSFontContext; iFilter: {Const}ATSFontFilterPtr; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator): OSStatus; external name '_ATSFontFamilyIteratorCreate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyIteratorRelease()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontFamilyIteratorRelease( var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorRelease';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
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
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyIteratorRelease(var ioIterator: ATSFontFamilyIterator): OSStatus; external name '_ATSFontFamilyIteratorRelease';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyIteratorRelease( var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyIteratorReset()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontFamilyIteratorReset( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorReset';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
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
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyIteratorReset(iContext: ATSFontContext; iFilter: {Const}ATSFontFilterPtr; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator): OSStatus; external name '_ATSFontFamilyIteratorReset';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyIteratorReset( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontFamilyIterator ): OSStatus; external name '_ATSFontFamilyIteratorReset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyIteratorNext()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontFamilyIteratorNext( iIterator: ATSFontFamilyIterator; var oFamily: ATSFontFamilyRef ): OSStatus; external name '_ATSFontFamilyIteratorNext';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
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
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyIteratorNext(iIterator: ATSFontFamilyIterator; var oFamily: ATSFontFamilyRef): OSStatus; external name '_ATSFontFamilyIteratorNext';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyIteratorNext( iIterator: ATSFontFamilyIterator; var oFamily: ATSFontFamilyRef ): OSStatus; external name '_ATSFontFamilyIteratorNext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyFindFromName()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontDescriptorCreateMatchingFontDescriptors() with kCTFontFamilyNameAttribute" }
function ATSFontFamilyFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFamilyFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyFindFromName(iName: CFStringRef; iOptions: ATSOptionFlags): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyGetGeneration()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Register for kCTFontManagerRegisteredFontsChangedNotification notifications" }
function ATSFontFamilyGetGeneration( iFamily: ATSFontFamilyRef ): ATSGeneration; external name '_ATSFontFamilyGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFamilyGetGeneration( iFamily: ATSFontFamilyRef ): ATSGeneration; external name '_ATSFontFamilyGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetGeneration( iFamily: ATSFontFamilyRef ): ATSGeneration; external name '_ATSFontFamilyGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetGeneration( iFamily: ATSFontFamilyRef ): ATSGeneration; external name '_ATSFontFamilyGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetGeneration( iFamily: ATSFontFamilyRef ): ATSGeneration; external name '_ATSFontFamilyGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyGetGeneration(iFamily: ATSFontFamilyRef): ATSGeneration; external name '_ATSFontFamilyGetGeneration';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyGetGeneration( iFamily: ATSFontFamilyRef ): ATSGeneration; external name '_ATSFontFamilyGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyGetName()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCopyFamilyName()" }
function ATSFontFamilyGetName( iFamily: ATSFontFamilyRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontFamilyGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFamilyGetName( iFamily: ATSFontFamilyRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontFamilyGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetName( iFamily: ATSFontFamilyRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontFamilyGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetName( iFamily: ATSFontFamilyRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontFamilyGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetName( iFamily: ATSFontFamilyRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontFamilyGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyGetName(iFamily: ATSFontFamilyRef; iOptions: ATSOptionFlags; var oName: CFStringRef): OSStatus; external name '_ATSFontFamilyGetName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyGetName( iFamily: ATSFontFamilyRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontFamilyGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyGetEncoding()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontGetStringEncoding()" }
function ATSFontFamilyGetEncoding( iFamily: ATSFontFamilyRef ): TextEncoding; external name '_ATSFontFamilyGetEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFamilyGetEncoding( iFamily: ATSFontFamilyRef ): TextEncoding; external name '_ATSFontFamilyGetEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetEncoding( iFamily: ATSFontFamilyRef ): TextEncoding; external name '_ATSFontFamilyGetEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetEncoding( iFamily: ATSFontFamilyRef ): TextEncoding; external name '_ATSFontFamilyGetEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetEncoding( iFamily: ATSFontFamilyRef ): TextEncoding; external name '_ATSFontFamilyGetEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyGetEncoding(iFamily: ATSFontFamilyRef): TextEncoding; external name '_ATSFontFamilyGetEncoding';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFamilyGetEncoding( iFamily: ATSFontFamilyRef ): TextEncoding; external name '_ATSFontFamilyGetEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ ----------------------------------------------------------------------------------------- }
{ Font                                                                                      }
{ ----------------------------------------------------------------------------------------- }
{
 *  ATSFontApplyFunction()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCollectionCreateFromAvailableFonts()" }
function ATSFontApplyFunction( iFunction: ATSFontApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontApplyFunction( iFunction: ATSFontApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontApplyFunction( iFunction: ATSFontApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontApplyFunction( iFunction: ATSFontApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontApplyFunction( iFunction: ATSFontApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontApplyFunction(iFunction: ATSFontApplierFunction; iRefCon: UnivPtr): OSStatus; external name '_ATSFontApplyFunction';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontApplyFunction( iFunction: ATSFontApplierFunction; iRefCon: UnivPtr ): OSStatus; external name '_ATSFontApplyFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontIteratorCreate()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCollectionCreateFromAvailableFonts()" }
function ATSFontIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontIteratorCreate(iContext: ATSFontContext; iFilter: {Const}ATSFontFilterPtr; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator): OSStatus; external name '_ATSFontIteratorCreate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontIteratorCreate( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontIteratorRelease()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontIteratorRelease( var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorRelease';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
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
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontIteratorRelease(var ioIterator: ATSFontIterator): OSStatus; external name '_ATSFontIteratorRelease';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontIteratorRelease( var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontIteratorReset()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontIteratorReset( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorReset';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
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
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontIteratorReset(iContext: ATSFontContext; iFilter: {Const}ATSFontFilterPtr; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator): OSStatus; external name '_ATSFontIteratorReset';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontIteratorReset( iContext: ATSFontContext; {const} iFilter: ATSFontFilterPtr { can be NULL }; iRefCon: UnivPtr; iOptions: ATSOptionFlags; var ioIterator: ATSFontIterator ): OSStatus; external name '_ATSFontIteratorReset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontIteratorNext()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontIteratorNext( iIterator: ATSFontIterator; var oFont: ATSFontRef ): OSStatus; external name '_ATSFontIteratorNext';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
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
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontIteratorNext(iIterator: ATSFontIterator; var oFont: ATSFontRef): OSStatus; external name '_ATSFontIteratorNext';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontIteratorNext( iIterator: ATSFontIterator; var oFont: ATSFontRef ): OSStatus; external name '_ATSFontIteratorNext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFindFromName()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCreateWithName()" }
function ATSFontFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFindFromName(iName: CFStringRef; iOptions: ATSOptionFlags): ATSFontRef; external name '_ATSFontFindFromName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFindFromName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFindFromPostScriptName()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCreateWithName()" }
function ATSFontFindFromPostScriptName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFindFromPostScriptName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromPostScriptName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromPostScriptName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromPostScriptName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFindFromPostScriptName(iName: CFStringRef; iOptions: ATSOptionFlags): ATSFontRef; external name '_ATSFontFindFromPostScriptName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

{
 *  ATSFontFindFromContainer()
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontDescriptorCreateMatchingFontDescriptors() with kCTFontURLAttribute or use CTFontManagerCreateFontDescriptorsFromURL()" }
function ATSFontFindFromContainer( iContainer: ATSFontContainerRef; iOptions: ATSOptionFlags; iCount: ItemCount; ioArray: {variable-size-array} ATSFontRefPtr; var oCount: ItemCount ): OSStatus; external name '_ATSFontFindFromContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFindFromContainer( iContainer: ATSFontContainerRef; iOptions: ATSOptionFlags; iCount: ItemCount; ioArray: {variable-size-array} ATSFontRefPtr; var oCount: ItemCount ): OSStatus; external name '_ATSFontFindFromContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromContainer( iContainer: ATSFontContainerRef; iOptions: ATSOptionFlags; iCount: ItemCount; ioArray: {variable-size-array} ATSFontRefPtr; var oCount: ItemCount ): OSStatus; external name '_ATSFontFindFromContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromContainer( iContainer: ATSFontContainerRef; iOptions: ATSOptionFlags; iCount: ItemCount; ioArray: {variable-size-array} ATSFontRefPtr; var oCount: ItemCount ): OSStatus; external name '_ATSFontFindFromContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFindFromContainer( iContainer: ATSFontContainerRef; iOptions: ATSOptionFlags; iCount: ItemCount; ioArray: {variable-size-array} ATSFontRefPtr; var oCount: ItemCount ): OSStatus; external name '_ATSFontFindFromContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
=======
function ATSFontFindFromPostScriptName( iName: CFStringRef; iOptions: ATSOptionFlags ): ATSFontRef; external name '_ATSFontFindFromPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFindFromContainer()
 *  
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFindFromContainer(iContainer: ATSFontContainerRef; iOptions: ATSOptionFlags; iCount: ItemCount; var ioArray: ATSFontRef; var oCount: ItemCount): OSStatus; external name '_ATSFontFindFromContainer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontFindFromContainer( iContainer: ATSFontContainerRef; iOptions: ATSOptionFlags; iCount: ItemCount; ioArray: {variable-size-array} ATSFontRefPtr; var oCount: ItemCount ): OSStatus; external name '_ATSFontFindFromContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontGetGeneration()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Register for kCTFontManagerRegisteredFontsChangedNotification notifications" }
function ATSFontGetGeneration( iFont: ATSFontRef ): ATSGeneration; external name '_ATSFontGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetGeneration( iFont: ATSFontRef ): ATSGeneration; external name '_ATSFontGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetGeneration( iFont: ATSFontRef ): ATSGeneration; external name '_ATSFontGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetGeneration( iFont: ATSFontRef ): ATSGeneration; external name '_ATSFontGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetGeneration( iFont: ATSFontRef ): ATSGeneration; external name '_ATSFontGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontGetGeneration(iFont: ATSFontRef): ATSGeneration; external name '_ATSFontGetGeneration';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontGetGeneration( iFont: ATSFontRef ): ATSGeneration; external name '_ATSFontGetGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontGetName()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCopyFullName()" }
function ATSFontGetName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontGetName(iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef): OSStatus; external name '_ATSFontGetName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontGetName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontGetPostScriptName()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCopyPostScriptName()" }
function ATSFontGetPostScriptName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetPostScriptName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetPostScriptName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetPostScriptName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetPostScriptName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontGetPostScriptName(iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef): OSStatus; external name '_ATSFontGetPostScriptName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontGetPostScriptName( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oName: CFStringRef ): OSStatus; external name '_ATSFontGetPostScriptName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontGetTableDirectory()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCopyAvailableTables()" }
function ATSFontGetTableDirectory( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTableDirectory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetTableDirectory( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTableDirectory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetTableDirectory( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTableDirectory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetTableDirectory( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTableDirectory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetTableDirectory( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTableDirectory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontGetTableDirectory(iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr): OSStatus; external name '_ATSFontGetTableDirectory';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontGetTableDirectory( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTableDirectory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontGetTable()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCopyTable()" }
function ATSFontGetTable( iFont: ATSFontRef; iTag: FourCharCode; iOffset: ByteOffset; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetTable( iFont: ATSFontRef; iTag: FourCharCode; iOffset: ByteOffset; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetTable( iFont: ATSFontRef; iTag: FourCharCode; iOffset: ByteOffset; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetTable( iFont: ATSFontRef; iTag: FourCharCode; iOffset: ByteOffset; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetTable( iFont: ATSFontRef; iTag: FourCharCode; iOffset: ByteOffset; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ATSFontGetTable(iFont: ATSFontRef; iTag: FourCharCode; iOffset: ByteOffset; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr): OSStatus; external name '_ATSFontGetTable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  ATSFontGetHorizontalMetrics()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontGetXHeight(), CTFontGetAscent(), and friends to find a specific metric." }
function ATSFontGetHorizontalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetHorizontalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetHorizontalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetHorizontalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetHorizontalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetHorizontalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetHorizontalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetHorizontalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetHorizontalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetHorizontalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ATSFontGetHorizontalMetrics(iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics): OSStatus; external name '_ATSFontGetHorizontalMetrics';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  ATSFontGetVerticalMetrics()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontGetXHeight(), CTFontGetAscent(), and friends to find a specific metric." }
function ATSFontGetVerticalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetVerticalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetVerticalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetVerticalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetVerticalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetVerticalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetVerticalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetVerticalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetVerticalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetVerticalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew

{ ----------------------------------------------------------------------------------------- }
{ Compatibility                                                                             }
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ATSFontGetVerticalMetrics(iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics): OSStatus; external name '_ATSFontGetVerticalMetrics';

{ ----------------------------------------------------------------------------------------- }
{ Compatibiity                                                                              }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ----------------------------------------------------------------------------------------- }
{
 *  ATSFontFamilyFindFromQuickDrawName()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCreateWithQuickdrawInstance()" }
function ATSFontFamilyFindFromQuickDrawName( const (*var*) iName: Str255 ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFamilyFindFromQuickDrawName( const (*var*) iName: Str255 ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyFindFromQuickDrawName( const (*var*) iName: Str255 ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyFindFromQuickDrawName( const (*var*) iName: Str255 ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyFindFromQuickDrawName( const (*var*) iName: Str255 ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontFamilyFindFromQuickDrawName(const (*var*) iName: Str255): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromQuickDrawName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontGetTable( iFont: ATSFontRef; iTag: FourCharCode; iOffset: ByteOffset; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontFamilyGetQuickDrawName()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "QuickDraw is deprecated" }
function ATSFontFamilyGetQuickDrawName( iFamily: ATSFontFamilyRef; var oName: Str255 ): OSStatus; external name '_ATSFontFamilyGetQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontFamilyGetQuickDrawName( iFamily: ATSFontFamilyRef; var oName: Str255 ): OSStatus; external name '_ATSFontFamilyGetQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetQuickDrawName( iFamily: ATSFontFamilyRef; var oName: Str255 ): OSStatus; external name '_ATSFontFamilyGetQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetQuickDrawName( iFamily: ATSFontFamilyRef; var oName: Str255 ): OSStatus; external name '_ATSFontFamilyGetQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontFamilyGetQuickDrawName( iFamily: ATSFontFamilyRef; var oName: Str255 ): OSStatus; external name '_ATSFontFamilyGetQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
function ATSFontGetHorizontalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetHorizontalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc TARGET_CPU_64}
{
 *  ATSFontGetFileSpecification()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontGetFileReference instead.
 *  
 *  Summary:
 *    Obtains the file specification for a font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Parameters:
 *    
 *    iFont:
 *      A reference to the font whose file specification you want to
 *      obtain.
 *    
 *    oFile:
 *      On output, points to the file specification that specifies the
 *      name and location of a file or directory that contains the font
 *      data specified by the iFont parameter.
 *  
 *  Result:
 *    noErr File specification obtained successfully.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCopyAttribute() with kCTFontURLAttribute." }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ATSFontGetFileSpecification( iFont: ATSFontRef; var oFile: ATSFSSpec ): OSStatus; external name '_ATSFontGetFileSpecification';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc}	{ TARGET_CPU_64 }

=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetVerticalMetrics( iFont: ATSFontRef; iOptions: ATSOptionFlags; var oMetrics: ATSFontMetrics ): OSStatus; external name '_ATSFontGetVerticalMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ ----------------------------------------------------------------------------------------- }
{ Compatibility                                                                             }
{ ----------------------------------------------------------------------------------------- }
>>>>>>> origin/fixes_2.4
{
 *  ATSFontGetFileReference()
 *  
 *  Summary:
 *    Obtains the file reference for a font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      A reference to the font whose file specification you want to
 *      obtain.
 *    
 *    oFile:
 *      On output, points to the file reference that specifies the name
 *      and location of a file or directory that contains the font data
 *      specified by the iFont parameter.
 *  
 *  Result:
 *    noErr File specification obtained successfully.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontCopyAttribute() with kCTFontURLAttribute." }
function ATSFontGetFileReference( iFont: ATSFontRef; var oFile: FSRef ): OSStatus; external name '_ATSFontGetFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetFileReference( iFont: ATSFontRef; var oFile: FSRef ): OSStatus; external name '_ATSFontGetFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetFileReference( iFont: ATSFontRef; var oFile: FSRef ): OSStatus; external name '_ATSFontGetFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetFileReference( iFont: ATSFontRef; var oFile: FSRef ): OSStatus; external name '_ATSFontGetFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetFileReference( iFont: ATSFontRef; var oFile: FSRef ): OSStatus; external name '_ATSFontGetFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontFamilyFindFromQuickDrawName( const (*var*) iName: Str255 ): ATSFontFamilyRef; external name '_ATSFontFamilyFindFromQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ATSFontFamilyGetQuickDrawName()
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontFamilyGetQuickDrawName( iFamily: ATSFontFamilyRef; var oName: Str255 ): OSStatus; external name '_ATSFontFamilyGetQuickDrawName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc TARGET_CPU_64}
{
 *  ATSFontGetFileSpecification()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ATSFontGetFileReference instead.
 *  
 *  Summary:
 *    Obtains the file specification for a font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Parameters:
 *    
 *    iFont:
 *      A reference to the font whose file specification you want to
 *      obtain.
 *    
 *    oFile:
 *      On output, points to the file specification that specifies the
 *      name and location of a file or directory that contains the font
 *      data specified by the iFont parameter.
 *  
 *  Result:
 *    noErr File specification obtained successfully.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetFileSpecification( iFont: ATSFontRef; var oFile: ATSFSSpec ): OSStatus; external name '_ATSFontGetFileSpecification';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc}	{ TARGET_CPU_64 }

{
 *  ATSFontGetFileReference()
 *  
 *  Summary:
 *    Obtains the file reference for a font.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iFont:
 *      A reference to the font whose file specification you want to
 *      obtain.
 *    
 *    oFile:
 *      On output, points to the file reference that specifies the name
 *      and location of a file or directory that contains the font data
 *      specified by the iFont parameter.
 *  
 *  Result:
 *    noErr File specification obtained successfully.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontGetFileSpecification(iFont: ATSFontRef; var oFile: FSSpec): OSStatus; external name '_ATSFontGetFileSpecification';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontGetFileReference( iFont: ATSFontRef; var oFile: FSRef ): OSStatus; external name '_ATSFontGetFileReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ATSFontGetFontFamilyResource()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "QuickDraw is deprecated" }
function ATSFontGetFontFamilyResource( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetFontFamilyResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetFontFamilyResource( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetFontFamilyResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetFontFamilyResource( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetFontFamilyResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetFontFamilyResource( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetFontFamilyResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontGetFontFamilyResource( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetFontFamilyResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ATSFontGetFontFamilyResource(iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr): OSStatus; external name '_ATSFontGetFontFamilyResource';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ATSFontGetFontFamilyResource( iFont: ATSFontRef; iBufferSize: ByteCount; ioBuffer: UnivPtr; oSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_ATSFontGetFontFamilyResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ ----------------------------------------------------------------------------------------- }
{ Notification                                                                              }
{ ----------------------------------------------------------------------------------------- }
{
 *  ATSFontNotify()
 *  
 *  Summary:
 *    Used to alert ATS that an action which may require notification
 *    to clients has occurred.
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Parameters:
 *    
 *    action:
 *      Action that should be taken by the ATS Server
 *    
 *    info:
 *      Any required or optional information that may be required by
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      the action taken.
=======
 *      the action taken. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      the action taken. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      the action taken.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    noErr Action successfully reported paramErr Invalid action passed
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Register for kCTFontManagerRegisteredFontsChangedNotification notifications" }
function ATSFontNotify( action: ATSFontNotifyAction; info: UnivPtr { can be NULL } ): OSStatus; external name '_ATSFontNotify';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
=======
>>>>>>> graemeg/cpstrnew
function ATSFontNotify( action: ATSFontNotifyAction; info: UnivPtr { can be NULL } ): OSStatus; external name '_ATSFontNotify';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontNotify( action: ATSFontNotifyAction; info: UnivPtr { can be NULL } ): OSStatus; external name '_ATSFontNotify';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontNotify( action: ATSFontNotifyAction; info: UnivPtr { can be NULL } ): OSStatus; external name '_ATSFontNotify';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSFontNotify( action: ATSFontNotifyAction; info: UnivPtr ): OSStatus; external name '_ATSFontNotify';

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSFontNotify( action: ATSFontNotifyAction; info: UnivPtr ): OSStatus; external name '_ATSFontNotify';
=======
function ATSFontNotify( action: ATSFontNotifyAction; info: UnivPtr { can be NULL } ): OSStatus; external name '_ATSFontNotify';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4

>>>>>>> origin/fixes_2_2

{
 *  ATSFontNotificationSubscribe()
 *  
 *  Summary:
 *    Ask the ATS System to notify caller when certain events have
 *    occurred.  Note that your application must have a CFRunLoop in
 *    order to receive notifications. Any Appkit or Carbon event loop
 *    based application will have one by default.
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Parameters:
 *    
 *    callback:
 *      Function that will be called by the ATS system whenever an
 *      event of interest takes place.
 *    
 *    options:
 *      Set the wanted ATSFontNotificationOptions to modify the default
 *      behavior of ATS Notifications.
 *    
 *    iRefcon:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      User data/state which will be passed to the callback funtion
 *    
 *    oNotificationRef:
 *      You may use this reference to un-subscribe to this notification.
=======
=======
>>>>>>> origin/fixes_2_2
 *      User data/state which will be passed to the callback funtion. can be NULL
 *    
 *    oNotificationRef:
 *      You may use this reference to un-subscribe to this notification. can be NULL
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *      User data/state which will be passed to the callback funtion
 *    
 *    oNotificationRef:
 *      You may use this reference to un-subscribe to this notification.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    noErr Subscribed successfully paramErr NULL callback was passed.
 *    memFullErr Could not allocate enough memory for internal data
 *    structures.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Register for kCTFontManagerRegisteredFontsChangedNotification notifications" }
function ATSFontNotificationSubscribe( callback: ATSNotificationCallback; options: ATSFontNotifyOption; iRefcon: UnivPtr { can be NULL }; oNotificationRef: ATSFontNotificationRefPtr { can be NULL } ): OSStatus; external name '_ATSFontNotificationSubscribe';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
=======
>>>>>>> graemeg/cpstrnew
function ATSFontNotificationSubscribe( callback: ATSNotificationCallback; options: ATSFontNotifyOption; iRefcon: UnivPtr { can be NULL }; oNotificationRef: ATSFontNotificationRefPtr { can be NULL } ): OSStatus; external name '_ATSFontNotificationSubscribe';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontNotificationSubscribe( callback: ATSNotificationCallback; options: ATSFontNotifyOption; iRefcon: UnivPtr { can be NULL }; oNotificationRef: ATSFontNotificationRefPtr { can be NULL } ): OSStatus; external name '_ATSFontNotificationSubscribe';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
function ATSFontNotificationSubscribe( callback: ATSNotificationCallback; options: ATSFontNotifyOption; iRefcon: UnivPtr { can be NULL }; oNotificationRef: ATSFontNotificationRefPtr { can be NULL } ): OSStatus; external name '_ATSFontNotificationSubscribe';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSFontNotificationSubscribe( callback: ATSNotificationCallback; options: ATSFontNotifyOption; iRefcon: UnivPtr; oNotificationRef: ATSFontNotificationRefPtr ): OSStatus; external name '_ATSFontNotificationSubscribe';
=======
function ATSFontNotificationSubscribe( callback: ATSNotificationCallback; options: ATSFontNotifyOption; iRefcon: UnivPtr { can be NULL }; oNotificationRef: ATSFontNotificationRefPtr { can be NULL } ): OSStatus; external name '_ATSFontNotificationSubscribe';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSFontNotificationSubscribe( callback: ATSNotificationCallback; options: ATSFontNotifyOption; iRefcon: UnivPtr; oNotificationRef: ATSFontNotificationRefPtr ): OSStatus; external name '_ATSFontNotificationSubscribe';

>>>>>>> origin/fixes_2_2

{
 *  ATSFontNotificationUnsubscribe()
 *  
 *  Summary:
 *    Release subscription and stop receiving notifications for a given
 *    reference.
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Parameters:
 *    
 *    notificationRef:
 *      Notification reference for which you want to stop receiving
 *      notifications. Note, if more than one notification has been
 *      requested of ATS, you will still receive notifications on those
 *      requests.
 *  
 *  Result:
 *    noErr Unsubscribed successfully paramErr NULL/invalid
 *    notificationRef passed
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Unregister for kCTFontManagerRegisteredFontsChangedNotification notifications" }
function ATSFontNotificationUnsubscribe( notificationRef: ATSFontNotificationRef ): OSStatus; external name '_ATSFontNotificationUnsubscribe';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
=======
>>>>>>> graemeg/cpstrnew
function ATSFontNotificationUnsubscribe( notificationRef: ATSFontNotificationRef ): OSStatus; external name '_ATSFontNotificationUnsubscribe';
=======
function ATSFontNotificationUnsubscribe( notificationRef: ATSFontNotificationRef ): OSStatus; external name '_ATSFontNotificationUnsubscribe';
>>>>>>> graemeg/cpstrnew
=======
function ATSFontNotificationUnsubscribe( notificationRef: ATSFontNotificationRef ): OSStatus; external name '_ATSFontNotificationUnsubscribe';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSFontNotificationUnsubscribe( notificationRef: ATSFontNotificationRef ): OSStatus; external name '_ATSFontNotificationUnsubscribe';

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
=======
>>>>>>> origin/fixes_2.4
function ATSFontNotificationUnsubscribe( notificationRef: ATSFontNotificationRef ): OSStatus; external name '_ATSFontNotificationUnsubscribe';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> origin/fixes_2_2

{ ----------------------------------------------------------------------------------------- }
{ Font query message hooks                                                                  }
{ ----------------------------------------------------------------------------------------- }

{
 *  ATSFontQuerySourceContext
 *  
 *  Summary:
 *    A parameter block for client information to be retained by ATS
 *    and passed back to an ATSFontQueryCallback function.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ATSFontQuerySourceContextPtr = ^ATSFontQuerySourceContext;
	ATSFontQuerySourceContext = record
{
=======
	ATSFontQuerySourceContext = record

  {
>>>>>>> graemeg/fixes_2_2
=======
	ATSFontQuerySourceContext = record

  {
>>>>>>> origin/fixes_2_2
=======
	ATSFontQuerySourceContextPtr = ^ATSFontQuerySourceContext;
	ATSFontQuerySourceContext = record
{
>>>>>>> origin/fixes_2.4
   * A 32-bit unsigned integer that indicates the version of this
   * structure. This should be set to 0.
   }
		version: UInt32;

  {
   * A pointer-sized client datum that should be passed back to an
   * ATSFontQueryCallback function.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		refCon: UnivPtr;
=======
		refCon: Ptr;
>>>>>>> graemeg/fixes_2_2
=======
		refCon: Ptr;
>>>>>>> origin/fixes_2_2
=======
		refCon: UnivPtr;
>>>>>>> origin/fixes_2.4

  {
   * The callback used to add a retain to the refCon.
   }
		retain: CFAllocatorRetainCallBack;

  {
   * The callback used to remove a retain to the refCon.
   }
		release: CFAllocatorReleaseCallBack;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
	ATSFontQuerySourceContextPtr = ^ATSFontQuerySourceContext;
>>>>>>> graemeg/fixes_2_2
=======
	ATSFontQuerySourceContextPtr = ^ATSFontQuerySourceContext;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  ATSFontQueryMessageID
 *  
 *  Discussion:
 *    Constants for ATS font query message types.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	ATSFontQueryMessageID = SInt32;
const
{
=======
=======
>>>>>>> origin/fixes_2_2
type
	ATSFontQueryMessageID = SInt32;
const

  {
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

	ATSFontQueryMessageID = SInt32;
const
{
>>>>>>> origin/fixes_2.4
   * The message ID for a font request query. The data for a message
   * with this ID is a flattened CFDictionaryRef with keys and values
   * as decribed below. A query dictionary may have any or all of these
   * entries.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSQueryActivateFontMessage = FourCharCode('atsa');
=======
  kATSQueryActivateFontMessage  = FourCharCode('atsa');
>>>>>>> graemeg/fixes_2_2
=======
  kATSQueryActivateFontMessage  = FourCharCode('atsa');
>>>>>>> origin/fixes_2_2
=======
	kATSQueryActivateFontMessage = FourCharCode('atsa');
>>>>>>> origin/fixes_2.4


{
 *  ATSFontQueryCallback
 *  
 *  Summary:
 *    Callback for receiving font-related queries from ATS.
 *  
 *  Parameters:
 *    
 *    msgid:
 *      An ATSFontQueryMessageID that identifies the message type.
 *    
 *    data:
 *      A CFPropertyListRef that represents the query. The content is
 *      message type-specific.
 *    
 *    refCon:
 *      A pointer-sized client datum that was optionally provided to
 *      ATSCreateFontQueryRunLoopSource.
 *  
 *  Result:
 *    A CFPropertyListRef that represents the message type-specific
 *    response to the query. May be NULL.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSFontQueryCallback = function( msgid: ATSFontQueryMessageID; data: CFPropertyListRef; refCon: UnivPtr ): CFPropertyListRef;
=======
type ATSFontQueryCallback = function( msgid: ATSFontQueryMessageID; data: CFPropertyListRef; refCon: UnivPtr ): CFPropertyListRef;
>>>>>>> graemeg/fixes_2_2
=======
type ATSFontQueryCallback = function( msgid: ATSFontQueryMessageID; data: CFPropertyListRef; refCon: UnivPtr ): CFPropertyListRef;
>>>>>>> origin/fixes_2_2
=======
type
	ATSFontQueryCallback = function( msgid: ATSFontQueryMessageID; data: CFPropertyListRef; refCon: UnivPtr ): CFPropertyListRef;
>>>>>>> origin/fixes_2.4
{
 *  ATSCreateFontQueryRunLoopSource()
 *  
 *  Summary:
 *    Creates a CFRunLoopSourceRef that will be used to convey font
 *    queries from ATS.
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version Jaguar
 *  
>>>>>>> origin/fixes_2.4
 *  Parameters:
 *    
 *    queryOrder:
 *      A CFIndex that specifies the priority of this query receiver
 *      relative to others. When ATS makes a font query, it will send
 *      the query to each receiver in priority order, from highest to
 *      lowest. "Normal" priority is 0.
 *    
 *    sourceOrder:
 *      The order of the created run loop source.
 *    
 *    callout:
 *      A function pointer of type ATSFontQueryCallback that will be
 *      called to process a font query.
 *    
 *    context:
 *      An ATSFontQuerySourceContext parameter block that provides a
 *      pointer-sized client datum which will be retained by ATS and
 *      passed to the callout function. May be NULL.
 *  
 *  Result:
 *    A CFRunLoopSourceRef. To stop receiving queries, invalidate this
 *    run loop source.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ "Use CTFontManagerCreateFontRequestRunLoopSource()" }
function ATSCreateFontQueryRunLoopSource( queryOrder: CFIndex; sourceOrder: CFIndex; callout: ATSFontQueryCallback; {const} context: ATSFontQuerySourceContextPtr { can be NULL } ): CFRunLoopSourceRef; external name '_ATSCreateFontQueryRunLoopSource';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSCreateFontQueryRunLoopSource( queryOrder: CFIndex; sourceOrder: CFIndex; callout: ATSFontQueryCallback; {const} context: ATSFontQuerySourceContextPtr { can be NULL } ): CFRunLoopSourceRef; external name '_ATSCreateFontQueryRunLoopSource';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSCreateFontQueryRunLoopSource( queryOrder: CFIndex; sourceOrder: CFIndex; callout: ATSFontQueryCallback; {const} context: ATSFontQuerySourceContextPtr { can be NULL } ): CFRunLoopSourceRef; external name '_ATSCreateFontQueryRunLoopSource';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSCreateFontQueryRunLoopSource( queryOrder: CFIndex; sourceOrder: CFIndex; callout: ATSFontQueryCallback; {const} context: ATSFontQuerySourceContextPtr { can be NULL } ): CFRunLoopSourceRef; external name '_ATSCreateFontQueryRunLoopSource';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function ATSCreateFontQueryRunLoopSource( queryOrder: CFIndex; sourceOrder: CFIndex; callout: ATSFontQueryCallback; {const} context: ATSFontQuerySourceContextPtr { can be NULL } ): CFRunLoopSourceRef; external name '_ATSCreateFontQueryRunLoopSource';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSCreateFontQueryRunLoopSource( queryOrder: CFIndex; sourceOrder: CFIndex; callout: ATSFontQueryCallback; context: ATSFontQuerySourceContextPtr ): CFRunLoopSourceRef; external name '_ATSCreateFontQueryRunLoopSource';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSCreateFontQueryRunLoopSource( queryOrder: CFIndex; sourceOrder: CFIndex; callout: ATSFontQueryCallback; context: ATSFontQuerySourceContextPtr ): CFRunLoopSourceRef; external name '_ATSCreateFontQueryRunLoopSource';
>>>>>>> origin/fixes_2_2
=======
function ATSCreateFontQueryRunLoopSource( queryOrder: CFIndex; sourceOrder: CFIndex; callout: ATSFontQueryCallback; {const} context: ATSFontQuerySourceContextPtr { can be NULL } ): CFRunLoopSourceRef; external name '_ATSCreateFontQueryRunLoopSource';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ ----------------------------------------------------------------------------------------- }
{ Font request query message content                                                        }
{ ----------------------------------------------------------------------------------------- }
{ Keys in a font request query dictionary. }
{ These keys appear in the dictionary for a kATSQueryActivateFontMessage query. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
//const
>>>>>>> graemeg/fixes_2_2
=======
//const
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  kATSQueryClientPID
 *  
 *  Discussion:
 *    The process ID of the application making the query. The
 *    corresponding value is a CFNumberRef that contains a pid_t.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSQueryClientPID CFSTRP('ATS client pid')}
{$endc}

{
 *  kATSQueryQDFamilyName
 *  
 *  Discussion:
 *    The Quickdraw-style family name of the font being requested, e.g.
 *    the name passed to GetFNum. The corresponding value is a
 *    CFStringRef.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSQueryQDFamilyName CFSTRP('font family name')}
{$endc}

{
 *  kATSQueryFontName
 *  
 *  Discussion:
 *    The name of the font being requested. The corresponding value is
 *    a CFStringRef suitable as an argument to ATSFontFindFromName().
 *    This should match a candidate font's unique or full name.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSQueryFontName CFSTRP('font name')}
{$endc}

{
 *  kATSQueryFontPostScriptName
 *  
 *  Discussion:
 *    The PostScript name of the font being requested. The
 *    corresponding value is a CFStringRef suitable as an argument to
 *    ATSFontFindFromPostScriptName(). This should match either the
 *    PostScript name derived from the font's FOND resource or its sfnt
 *    name table, with preference given to the FOND PostScript name.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSQueryFontPostScriptName CFSTRP('font PS name')}
{$endc}

{
 *  kATSQueryFontNameTableEntries
 *  
 *  Discussion:
 *    A descriptor for sfnt name table entries that the requested font
 *    must have. The corresponding value is a CFArrayRef of
 *    CFDictionaryRefs that describe name table entries. A font must
 *    have all of the specified entries to be considered a match.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSQueryFontNameTableEntries CFSTRP('font name table entries')}
{$endc}
{ Keys in a font raw name descriptor dictionary. }

{
 *  kATSFontNameTableCode
 *  
 *  Discussion:
 *    The font name's name code. The corresponding value is a
 *    CFNumberRef. If missing, assume kFontNoNameCode.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSFontNameTableCode CFSTRP('font name code')}
{$endc}

{
 *  kATSFontNameTablePlatform
 *  
 *  Discussion:
 *    The font name's platform code. The corresponding value is a
 *    CFNumberRef. If missing, assume kFontNoPlatformCode.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSFontNameTablePlatform CFSTRP('font platform code')}
{$endc}

{
 *  kATSFontNameTableScript
 *  
 *  Discussion:
 *    The font name's script code. The corresponding value is a
 *    CFNumberRef. If missing, assume kFontNoScriptCode.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSFontNameTableScript CFSTRP('font script code')}
{$endc}

{
 *  kATSFontNameTableLanguage
 *  
 *  Discussion:
 *    The font name's language code. The corresponding value is a
 *    CFNumberRef. If missing, assume kFontNoLanguageCode.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSFontNameTableLanguage CFSTRP('font language code')}
{$endc}

{
 *  kATSFontNameTableBytes
 *  
 *  Discussion:
 *    The raw bytes of the font name. The corresponding value is a
 *    CFDataRef that contains the raw name bytes.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kATSFontNameTableBytes CFSTRP('font name table bytes')}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ ----------------------------------------------------------------------------------------- }
{ Auto activation settings                                                                  }
{ ----------------------------------------------------------------------------------------- }

{
 *  ATSAutoActivationSetting
 *  
 *  Summary:
 *    Values for auto activation settings.
 }
const
{
   * Resets the setting the the default state. For application settings
   * this clears the setting. For the global setting it will revert to
   * the initial system setting, kATSFontAutoActivationEnabled.
   }
	kATSFontAutoActivationDefault = 0;
	kATSFontAutoActivationDisabled = 1;
	kATSFontAutoActivationEnabled = 2;

  {
   * Asks the user before auto-activating fonts requested by the
   * application.
   }
	kATSFontAutoActivationAsk = 4;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


type
	ATSFontAutoActivationSetting = UInt32;
{
 *  ATSFontSetGlobalAutoActivationSetting()
 *  
 *  Summary:
 *    Sets the user's global auto-activation setting.
 *  
 *  Discussion:
 *    This function can be used to set the user's global
 *    auto-activation setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iSetting:
 *      One of the enumerated constants above specifying the setting
 *      for font auto-activation. Will return paramErr on invalid input.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontSetGlobalAutoActivationSetting( iSetting: ATSFontAutoActivationSetting ): OSStatus; external name '_ATSFontSetGlobalAutoActivationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontGetGlobalAutoActivationSetting()
 *  
 *  Summary:
 *    Gets the user's global auto-activation setting.
 *  
 *  Discussion:
 *    This function can be used to get the user's global
 *    auto-activation setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetGlobalAutoActivationSetting: ATSFontAutoActivationSetting; external name '_ATSFontGetGlobalAutoActivationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontSetAutoActivationSettingForApplication()
 *  
 *  Summary:
 *    Sets the auto-activation setting for the specified application
 *    bundle.
 *  
 *  Discussion:
 *    This function can be used to set the auto-activation setting for
 *    the specified application. The per-application setting overrides
 *    the global setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iSetting:
 *      One of the enumerated constants above specifying the setting
 *      for font auto-activation. Specifying
 *      kATSFontAutoActivationDefault will clear the application
 *      specific setting and the global setting will be used.
 *    
 *    iApplicationFileURL:
 *      A valid file URL for an application. Passing NULL for this
 *      parameter indicates the current process.
 *  
 *  Result:
 *    An OSStatus code. Will return noErr on success, and paramErr on
 *    any invalid input. May return memFullErr if unable to allocate
 *    temporary structures.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontSetAutoActivationSettingForApplication( iSetting: ATSFontAutoActivationSetting; iApplicationFileURL: CFURLRef ): OSStatus; external name '_ATSFontSetAutoActivationSettingForApplication';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontGetAutoActivationSettingForApplication()
 *  
 *  Summary:
 *    Query the activation setting for the specified application.
 *  
 *  Discussion:
 *    This function can be used to query the auto-activation setting
 *    for the specified application. The setting is the app-specific
 *    setting if available, otherwise it is
 *    kATSFontAutoActivationDefault.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iApplicationFileURL:
 *      A valid file URL for an application. Passing NULL for this
 *      parameter indicates the current process.
 *  
 *  Result:
 *    Returns the setting that will be used for the specified
 *    application. If this returns kATSFontAutoActivationDefault the
 *    global setting will be used.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetAutoActivationSettingForApplication( iApplicationFileURL: CFURLRef ): ATSFontAutoActivationSetting; external name '_ATSFontGetAutoActivationSettingForApplication';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

type
	ATSFontAutoActivationSetting = UInt32;
{
 *  ATSFontSetGlobalAutoActivationSetting()
 *  
 *  Summary:
 *    Sets the user's global auto-activation setting.
 *  
 *  Discussion:
 *    This function can be used to set the user's global
 *    auto-activation setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iSetting:
 *      One of the enumerated constants above specifying the setting
 *      for font auto-activation. Will return paramErr on invalid input.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
{ "Use CTFontManagerSetAutoActivationSetting() with kCTFontManagerBundleIdentifier" }
function ATSFontSetGlobalAutoActivationSetting( iSetting: ATSFontAutoActivationSetting ): OSStatus; external name '_ATSFontSetGlobalAutoActivationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontSetGlobalAutoActivationSetting( iSetting: ATSFontAutoActivationSetting ): OSStatus; external name '_ATSFontSetGlobalAutoActivationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew

{
 *  ATSFontGetGlobalAutoActivationSetting()
 *  
 *  Summary:
 *    Gets the user's global auto-activation setting.
 *  
 *  Discussion:
 *    This function can be used to get the user's global
 *    auto-activation setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
{ "Use CTFontManagerGetAutoActivationSetting() with kCTFontManagerBundleIdentifier" }
function ATSFontGetGlobalAutoActivationSetting: ATSFontAutoActivationSetting; external name '_ATSFontGetGlobalAutoActivationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetGlobalAutoActivationSetting: ATSFontAutoActivationSetting; external name '_ATSFontGetGlobalAutoActivationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew

{
 *  ATSFontSetAutoActivationSettingForApplication()
 *  
 *  Summary:
 *    Sets the auto-activation setting for the specified application
 *    bundle.
 *  
 *  Discussion:
 *    This function can be used to set the auto-activation setting for
 *    the specified application. The per-application setting overrides
 *    the global setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iSetting:
 *      One of the enumerated constants above specifying the setting
 *      for font auto-activation. Specifying
 *      kATSFontAutoActivationDefault will clear the application
 *      specific setting and the global setting will be used.
 *    
 *    iApplicationFileURL:
 *      A valid file URL for an application. Passing NULL for this
 *      parameter indicates the current process.
 *  
 *  Result:
 *    An OSStatus code. Will return noErr on success, and paramErr on
 *    any invalid input. May return memFullErr if unable to allocate
 *    temporary structures.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
{ "Use CTFontManagerSetAutoActivationSetting()" }
function ATSFontSetAutoActivationSettingForApplication( iSetting: ATSFontAutoActivationSetting; iApplicationFileURL: CFURLRef ): OSStatus; external name '_ATSFontSetAutoActivationSettingForApplication';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontSetAutoActivationSettingForApplication( iSetting: ATSFontAutoActivationSetting; iApplicationFileURL: CFURLRef ): OSStatus; external name '_ATSFontSetAutoActivationSettingForApplication';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew

{
 *  ATSFontGetAutoActivationSettingForApplication()
 *  
 *  Summary:
 *    Query the activation setting for the specified application.
 *  
 *  Discussion:
 *    This function can be used to query the auto-activation setting
 *    for the specified application. The setting is the app-specific
 *    setting if available, otherwise it is
 *    kATSFontAutoActivationDefault.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iApplicationFileURL:
 *      A valid file URL for an application. Passing NULL for this
 *      parameter indicates the current process.
 *  
 *  Result:
 *    Returns the setting that will be used for the specified
 *    application. If this returns kATSFontAutoActivationDefault the
 *    global setting will be used.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
{ "Use CTFontManagerGetAutoActivationSetting()" }
function ATSFontGetAutoActivationSettingForApplication( iApplicationFileURL: CFURLRef ): ATSFontAutoActivationSetting; external name '_ATSFontGetAutoActivationSettingForApplication';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_8 *)
=======
function ATSFontGetAutoActivationSettingForApplication( iApplicationFileURL: CFURLRef ): ATSFontAutoActivationSetting; external name '_ATSFontGetAutoActivationSettingForApplication';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew

=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{ ----------------------------------------------------------------------------------------- }
{ Auto activation settings                                                                  }
{ ----------------------------------------------------------------------------------------- }
>>>>>>> origin/fixes_2.4

{
 *  ATSAutoActivationSetting
 *  
 *  Summary:
 *    Values for auto activation settings.
 }
const
{
   * Resets the setting the the default state. For application settings
   * this clears the setting. For the global setting it will revert to
   * the initial system setting, kATSFontAutoActivationEnabled.
   }
	kATSFontAutoActivationDefault = 0;
	kATSFontAutoActivationDisabled = 1;
	kATSFontAutoActivationEnabled = 2;

  {
   * Asks the user before auto-activating fonts requested by the
   * application.
   }
	kATSFontAutoActivationAsk = 4;


type
	ATSFontAutoActivationSetting = UInt32;
{
 *  ATSFontSetGlobalAutoActivationSetting()
 *  
 *  Summary:
 *    Sets the user's global auto-activation setting.
 *  
 *  Discussion:
 *    This function can be used to set the user's global
 *    auto-activation setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iSetting:
 *      One of the enumerated constants above specifying the setting
 *      for font auto-activation. Will return paramErr on invalid input.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontSetGlobalAutoActivationSetting( iSetting: ATSFontAutoActivationSetting ): OSStatus; external name '_ATSFontSetGlobalAutoActivationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontGetGlobalAutoActivationSetting()
 *  
 *  Summary:
 *    Gets the user's global auto-activation setting.
 *  
 *  Discussion:
 *    This function can be used to get the user's global
 *    auto-activation setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetGlobalAutoActivationSetting: ATSFontAutoActivationSetting; external name '_ATSFontGetGlobalAutoActivationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontSetAutoActivationSettingForApplication()
 *  
 *  Summary:
 *    Sets the auto-activation setting for the specified application
 *    bundle.
 *  
 *  Discussion:
 *    This function can be used to set the auto-activation setting for
 *    the specified application. The per-application setting overrides
 *    the global setting.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iSetting:
 *      One of the enumerated constants above specifying the setting
 *      for font auto-activation. Specifying
 *      kATSFontAutoActivationDefault will clear the application
 *      specific setting and the global setting will be used.
 *    
 *    iApplicationFileURL:
 *      A valid file URL for an application. Passing NULL for this
 *      parameter indicates the current process.
 *  
 *  Result:
 *    An OSStatus code. Will return noErr on success, and paramErr on
 *    any invalid input. May return memFullErr if unable to allocate
 *    temporary structures.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontSetAutoActivationSettingForApplication( iSetting: ATSFontAutoActivationSetting; iApplicationFileURL: CFURLRef ): OSStatus; external name '_ATSFontSetAutoActivationSettingForApplication';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  ATSFontGetAutoActivationSettingForApplication()
 *  
 *  Summary:
 *    Query the activation setting for the specified application.
 *  
 *  Discussion:
 *    This function can be used to query the auto-activation setting
 *    for the specified application. The setting is the app-specific
 *    setting if available, otherwise it is
 *    kATSFontAutoActivationDefault.
 *  
 *  Mac OS X threading:
 *    Thread safe since version Leopard
 *  
 *  Parameters:
 *    
 *    iApplicationFileURL:
 *      A valid file URL for an application. Passing NULL for this
 *      parameter indicates the current process.
 *  
 *  Result:
 *    Returns the setting that will be used for the specified
 *    application. If this returns kATSFontAutoActivationDefault the
 *    global setting will be used.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ATSFontGetAutoActivationSettingForApplication( iApplicationFileURL: CFURLRef ): ATSFontAutoActivationSetting; external name '_ATSFontGetAutoActivationSettingForApplication';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

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
