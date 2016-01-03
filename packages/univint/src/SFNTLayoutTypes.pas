{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       ATS/SFNTLayoutTypes.h
 
     Contains:   SFNT file layout structures and constants.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    ATS
 
     Copyright:  © 1994-2012 by Apple Inc., all rights reserved.
=======
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
>>>>>>> graemeg/cpstrnew
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                SFNTLayoutTypes.i
                     Revision:        1.3
                     Dated:           2007/01/15 23:28:27
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}

{  Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
=======
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                SFNTLayoutTypes.i
                     Revision:        1.3
                     Dated:           2007/01/15 23:28:27
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                SFNTLayoutTypes.i
                     Revision:        1.3
                     Dated:           2007/01/15 23:28:27
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                SFNTLayoutTypes.i
                     Revision:        1.3
                     Dated:           2007/01/15 23:28:27
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
>>>>>>> origin/cpstrnew
     Bugs:       Report bugs to Radar component "System Interfaces", "Latest"
                 List the version information (from above) in the Problem Description.
 
}

{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
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
     File:       SFNTLayoutTypes.p
=======
     File:       ATS/SFNTLayoutTypes.h
>>>>>>> origin/fixes_2.4
 
     Contains:   SFNT file layout structures and constants.
 
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                SFNTLayoutTypes.i
                     Revision:        1.3
                     Dated:           2007/01/15 23:28:27
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
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

unit SFNTLayoutTypes;
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
{$elsec}
=======
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
{$elsec}
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
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
uses MacTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
//#pragma pack(push, 2)
=======


>>>>>>> graemeg/fixes_2_2
=======


>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_OS_MAC}

//#pragma pack(push, 2)
>>>>>>> origin/fixes_2.4
{$ALIGN MAC68K}

{ ----------------------------------------------------------------------------------------- }
{ CONSTANTS }
{
    The following values can be used to set run feature values. Note that unless the
    feature is defaulted differently in different fonts, the zero value for the
<<<<<<< HEAD
<<<<<<< HEAD
    selectors represents the default value. Consult the following URL for further info:
    <http://developer.apple.com/fonts/registry/>
=======
    selectors represents the default value.  Check the following URL site for further info:
    "http://fonts.apple.com/Registry"
>>>>>>> graemeg/fixes_2_2
=======
    selectors represents the default value.  Check the following URL site for further info:
    "http://fonts.apple.com/Registry"
>>>>>>> origin/fixes_2_2
}


{
 *  Summary:
 *    Feature types
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAllTypographicFeaturesType = 0;
	kLigaturesType = 1;
	kCursiveConnectionType = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kLetterCaseType = 3;    { deprecated - use kLowerCaseType or kUpperCaseType instead }
=======
	kLetterCaseType = 3;
>>>>>>> graemeg/cpstrnew
=======
	kLetterCaseType = 3;
>>>>>>> graemeg/cpstrnew
=======
	kLetterCaseType = 3;
>>>>>>> graemeg/cpstrnew
=======
	kLetterCaseType = 3;
>>>>>>> origin/cpstrnew
	kVerticalSubstitutionType = 4;
	kLinguisticRearrangementType = 5;
	kNumberSpacingType = 6;
	kSmartSwashType = 8;
	kDiacriticsType = 9;
	kVerticalPositionType = 10;
	kFractionsType = 11;
	kOverlappingCharactersType = 13;
	kTypographicExtrasType = 14;
	kMathematicalExtrasType = 15;
	kOrnamentSetsType = 16;
	kCharacterAlternativesType = 17;
	kDesignComplexityType = 18;
	kStyleOptionsType = 19;
	kCharacterShapeType = 20;
	kNumberCaseType = 21;
	kTextSpacingType = 22;
	kTransliterationType = 23;
	kAnnotationType = 24;
	kKanaSpacingType = 25;
	kIdeographicSpacingType = 26;
	kUnicodeDecompositionType = 27;
	kRubyKanaType = 28;
	kCJKSymbolAlternativesType = 29;
	kIdeographicAlternativesType = 30;
	kCJKVerticalRomanPlacementType = 31;
	kItalicCJKRomanType = 32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCaseSensitiveLayoutType = 33;
	kAlternateKanaType = 34;
	kStylisticAlternativesType = 35;
	kContextualAlternatesType = 36;
	kLowerCaseType = 37;
	kUpperCaseType = 38;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kCJKRomanSpacingType = 103;
	kLastFeatureType = -1;


{
 *  Summary:
 *    Selectors for feature type kAllTypographicFeaturesType
 }
const
	kAllTypeFeaturesOnSelector = 0;
	kAllTypeFeaturesOffSelector = 1;


{
 *  Summary:
 *    Selectors for feature type kLigaturesType
 }
const
	kRequiredLigaturesOnSelector = 0;
	kRequiredLigaturesOffSelector = 1;
	kCommonLigaturesOnSelector = 2;
	kCommonLigaturesOffSelector = 3;
	kRareLigaturesOnSelector = 4;
	kRareLigaturesOffSelector = 5;
	kLogosOnSelector = 6;
	kLogosOffSelector = 7;
	kRebusPicturesOnSelector = 8;
	kRebusPicturesOffSelector = 9;
	kDiphthongLigaturesOnSelector = 10;
	kDiphthongLigaturesOffSelector = 11;
	kSquaredLigaturesOnSelector = 12;
	kSquaredLigaturesOffSelector = 13;
	kAbbrevSquaredLigaturesOnSelector = 14;
	kAbbrevSquaredLigaturesOffSelector = 15;
	kSymbolLigaturesOnSelector = 16;
	kSymbolLigaturesOffSelector = 17;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kContextualLigaturesOnSelector = 18;
	kContextualLigaturesOffSelector = 19;
	kHistoricalLigaturesOnSelector = 20;
	kHistoricalLigaturesOffSelector = 21;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{
 *  Summary:
 *    Selectors for feature type kCursiveConnectionType
 }
const
	kUnconnectedSelector = 0;
	kPartiallyConnectedSelector = 1;
	kCursiveSelector = 2;


{
 *  Summary:
 *    Selectors for feature type kLetterCaseType
 }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kUpperAndLowerCaseSelector = 0;    { deprecated }
	kAllCapsSelector = 1;    { deprecated }
	kAllLowerCaseSelector = 2;    { deprecated }
	kSmallCapsSelector = 3;    { deprecated }
	kInitialCapsSelector = 4;    { deprecated }
	kInitialCapsAndSmallCapsSelector = 5;  { deprecated }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kUpperAndLowerCaseSelector = 0;
	kAllCapsSelector = 1;
	kAllLowerCaseSelector = 2;
	kSmallCapsSelector = 3;
	kInitialCapsSelector = 4;
	kInitialCapsAndSmallCapsSelector = 5;
>>>>>>> graemeg/cpstrnew


{
 *  Summary:
 *    Selectors for feature type kVerticalSubstitutionType
 }
const
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
const
	kAllTypographicFeaturesType = 0;
	kLigaturesType = 1;
	kCursiveConnectionType = 2;
	kLetterCaseType = 3;
	kVerticalSubstitutionType = 4;
	kLinguisticRearrangementType = 5;
	kNumberSpacingType = 6;
	kSmartSwashType = 8;
	kDiacriticsType = 9;
	kVerticalPositionType = 10;
	kFractionsType = 11;
	kOverlappingCharactersType = 13;
	kTypographicExtrasType = 14;
	kMathematicalExtrasType = 15;
	kOrnamentSetsType = 16;
	kCharacterAlternativesType = 17;
	kDesignComplexityType = 18;
	kStyleOptionsType = 19;
	kCharacterShapeType = 20;
	kNumberCaseType = 21;
	kTextSpacingType = 22;
	kTransliterationType = 23;
	kAnnotationType = 24;
	kKanaSpacingType = 25;
	kIdeographicSpacingType = 26;
	kUnicodeDecompositionType = 27;
	kRubyKanaType = 28;
	kCJKSymbolAlternativesType = 29;
	kIdeographicAlternativesType = 30;
	kCJKVerticalRomanPlacementType = 31;
	kItalicCJKRomanType = 32;
	kCJKRomanSpacingType = 103;
	kLastFeatureType = -1;


{
 *  Summary:
 *    Selectors for feature type kAllTypographicFeaturesType
 }
const
	kAllTypeFeaturesOnSelector = 0;
	kAllTypeFeaturesOffSelector = 1;


{
 *  Summary:
 *    Selectors for feature type kLigaturesType
 }
const
	kRequiredLigaturesOnSelector = 0;
	kRequiredLigaturesOffSelector = 1;
	kCommonLigaturesOnSelector = 2;
	kCommonLigaturesOffSelector = 3;
	kRareLigaturesOnSelector = 4;
	kRareLigaturesOffSelector = 5;
	kLogosOnSelector = 6;
	kLogosOffSelector = 7;
	kRebusPicturesOnSelector = 8;
	kRebusPicturesOffSelector = 9;
	kDiphthongLigaturesOnSelector = 10;
	kDiphthongLigaturesOffSelector = 11;
	kSquaredLigaturesOnSelector = 12;
	kSquaredLigaturesOffSelector = 13;
	kAbbrevSquaredLigaturesOnSelector = 14;
	kAbbrevSquaredLigaturesOffSelector = 15;
	kSymbolLigaturesOnSelector = 16;
	kSymbolLigaturesOffSelector = 17;


{
 *  Summary:
 *    Selectors for feature type kCursiveConnectionType
 }
const
	kUnconnectedSelector = 0;
	kPartiallyConnectedSelector = 1;
	kCursiveSelector = 2;


{
 *  Summary:
 *    Selectors for feature type kLetterCaseType
 }
const
	kUpperAndLowerCaseSelector = 0;
	kAllCapsSelector = 1;
	kAllLowerCaseSelector = 2;
	kSmallCapsSelector = 3;
	kInitialCapsSelector = 4;
	kInitialCapsAndSmallCapsSelector = 5;


<<<<<<< HEAD
	{
	 *  Summary:
	 *    Selectors for feature type kVerticalSubstitutionType
	 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  Summary:
 *    Selectors for feature type kVerticalSubstitutionType
 }
const
>>>>>>> origin/fixes_2.4
	kSubstituteVerticalFormsOnSelector = 0;
	kSubstituteVerticalFormsOffSelector = 1;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Selectors for feature type kLinguisticRearrangementType
 }
const
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  Summary:
	 *    Selectors for feature type kLinguisticRearrangementType
	 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kLinguisticRearrangementOnSelector = 0;
	kLinguisticRearrangementOffSelector = 1;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Selectors for feature type kNumberSpacingType
 }
const
	kMonospacedNumbersSelector = 0;
<<<<<<< HEAD
	kProportionalNumbersSelector = 1;
	kThirdWidthNumbersSelector = 2;
	kQuarterWidthNumbersSelector = 3;


{
 *  Summary:
 *    Selectors for feature type kSmartSwashType
 }
const
	kWordInitialSwashesOnSelector = 0;
	kWordInitialSwashesOffSelector = 1;
	kWordFinalSwashesOnSelector = 2;
	kWordFinalSwashesOffSelector = 3;
	kLineInitialSwashesOnSelector = 4;
	kLineInitialSwashesOffSelector = 5;
	kLineFinalSwashesOnSelector = 6;
	kLineFinalSwashesOffSelector = 7;
	kNonFinalSwashesOnSelector = 8;
	kNonFinalSwashesOffSelector = 9;


{
 *  Summary:
 *    Selectors for feature type kDiacriticsType
 }
const
	kShowDiacriticsSelector = 0;
	kHideDiacriticsSelector = 1;
	kDecomposeDiacriticsSelector = 2;


{
 *  Summary:
 *    Selectors for feature type kVerticalPositionType
 }
const
	kNormalPositionSelector = 0;
	kSuperiorsSelector = 1;
	kInferiorsSelector = 2;
	kOrdinalsSelector = 3;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kScientificInferiorsSelector = 4;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{
 *  Summary:
 *    Selectors for feature type kFractionsType
 }
const
	kNoFractionsSelector = 0;
	kVerticalFractionsSelector = 1;
	kDiagonalFractionsSelector = 2;


{
 *  Summary:
 *    Selectors for feature type kOverlappingCharactersType
 }
const
	kPreventOverlapOnSelector = 0;
	kPreventOverlapOffSelector = 1;


{
 *  Summary:
 *    Selectors for feature type kTypographicExtrasType
 }
const
	kHyphensToEmDashOnSelector = 0;
	kHyphensToEmDashOffSelector = 1;
	kHyphenToEnDashOnSelector = 2;
	kHyphenToEnDashOffSelector = 3;
	kSlashedZeroOnSelector = 4;
	kSlashedZeroOffSelector = 5;
	kFormInterrobangOnSelector = 6;
	kFormInterrobangOffSelector = 7;
	kSmartQuotesOnSelector = 8;
	kSmartQuotesOffSelector = 9;
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  Summary:
	 *    Selectors for feature type kNumberSpacingType
	 	}
	kMonospacedNumbersSelector	= 0;
=======
>>>>>>> origin/fixes_2.4
	kProportionalNumbersSelector = 1;
	kThirdWidthNumbersSelector = 2;
	kQuarterWidthNumbersSelector = 3;


{
 *  Summary:
 *    Selectors for feature type kSmartSwashType
 }
const
	kWordInitialSwashesOnSelector = 0;
	kWordInitialSwashesOffSelector = 1;
	kWordFinalSwashesOnSelector = 2;
	kWordFinalSwashesOffSelector = 3;
	kLineInitialSwashesOnSelector = 4;
	kLineInitialSwashesOffSelector = 5;
	kLineFinalSwashesOnSelector = 6;
	kLineFinalSwashesOffSelector = 7;
	kNonFinalSwashesOnSelector = 8;
	kNonFinalSwashesOffSelector = 9;


{
 *  Summary:
 *    Selectors for feature type kDiacriticsType
 }
const
	kShowDiacriticsSelector = 0;
	kHideDiacriticsSelector = 1;
	kDecomposeDiacriticsSelector = 2;


<<<<<<< HEAD
	{
	 *  Summary:
	 *    Selectors for feature type kVerticalPositionType
	 	}
	kNormalPositionSelector		= 0;
	kSuperiorsSelector			= 1;
	kInferiorsSelector			= 2;
	kOrdinalsSelector			= 3;


	{
	 *  Summary:
	 *    Selectors for feature type kFractionsType
	 	}
	kNoFractionsSelector		= 0;
	kVerticalFractionsSelector	= 1;
	kDiagonalFractionsSelector	= 2;


	{
	 *  Summary:
	 *    Selectors for feature type kOverlappingCharactersType
	 	}
	kPreventOverlapOnSelector	= 0;
	kPreventOverlapOffSelector	= 1;


	{
	 *  Summary:
	 *    Selectors for feature type kTypographicExtrasType
	 	}
	kHyphensToEmDashOnSelector	= 0;
	kHyphensToEmDashOffSelector	= 1;
	kHyphenToEnDashOnSelector	= 2;
	kHyphenToEnDashOffSelector	= 3;
	kSlashedZeroOnSelector		= 4;
	kSlashedZeroOffSelector		= 5;
	kFormInterrobangOnSelector	= 6;
	kFormInterrobangOffSelector	= 7;
	kSmartQuotesOnSelector		= 8;
	kSmartQuotesOffSelector		= 9;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  Summary:
 *    Selectors for feature type kVerticalPositionType
 }
const
	kNormalPositionSelector = 0;
	kSuperiorsSelector = 1;
	kInferiorsSelector = 2;
	kOrdinalsSelector = 3;


{
 *  Summary:
 *    Selectors for feature type kFractionsType
 }
const
	kNoFractionsSelector = 0;
	kVerticalFractionsSelector = 1;
	kDiagonalFractionsSelector = 2;


{
 *  Summary:
 *    Selectors for feature type kOverlappingCharactersType
 }
const
	kPreventOverlapOnSelector = 0;
	kPreventOverlapOffSelector = 1;


{
 *  Summary:
 *    Selectors for feature type kTypographicExtrasType
 }
const
	kHyphensToEmDashOnSelector = 0;
	kHyphensToEmDashOffSelector = 1;
	kHyphenToEnDashOnSelector = 2;
	kHyphenToEnDashOffSelector = 3;
	kSlashedZeroOnSelector = 4;
	kSlashedZeroOffSelector = 5;
	kFormInterrobangOnSelector = 6;
	kFormInterrobangOffSelector = 7;
	kSmartQuotesOnSelector = 8;
	kSmartQuotesOffSelector = 9;
>>>>>>> origin/fixes_2.4
	kPeriodsToEllipsisOnSelector = 10;
	kPeriodsToEllipsisOffSelector = 11;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Selectors for feature type kMathematicalExtrasType
 }
const
	kHyphenToMinusOnSelector = 0;
	kHyphenToMinusOffSelector = 1;
	kAsteriskToMultiplyOnSelector = 2;
	kAsteriskToMultiplyOffSelector = 3;
	kSlashToDivideOnSelector = 4;
	kSlashToDivideOffSelector = 5;
	kInequalityLigaturesOnSelector = 6;
	kInequalityLigaturesOffSelector = 7;
	kExponentsOnSelector = 8;
	kExponentsOffSelector = 9;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kMathematicalGreekOnSelector = 10;
	kMathematicalGreekOffSelector = 11;
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


{
 *  Summary:
 *    Selectors for feature type kOrnamentSetsType
 }
const
	kNoOrnamentsSelector = 0;
	kDingbatsSelector = 1;
	kPiCharactersSelector = 2;
	kFleuronsSelector = 3;
	kDecorativeBordersSelector = 4;
	kInternationalSymbolsSelector = 5;
	kMathSymbolsSelector = 6;


{
 *  Summary:
 *    Selectors for feature type kCharacterAlternativesType
 }
const
	kNoAlternatesSelector = 0;


{
 *  Summary:
 *    Selectors for feature type kDesignComplexityType
 }
const
	kDesignLevel1Selector = 0;
	kDesignLevel2Selector = 1;
	kDesignLevel3Selector = 2;
	kDesignLevel4Selector = 3;
	kDesignLevel5Selector = 4;


{
 *  Summary:
 *    Selectors for feature type kStyleOptionsType
 }
const
	kNoStyleOptionsSelector = 0;
	kDisplayTextSelector = 1;
	kEngravedTextSelector = 2;
	kIlluminatedCapsSelector = 3;
	kTitlingCapsSelector = 4;
	kTallCapsSelector = 5;


{
 *  Summary:
 *    Selectors for feature type kCharacterShapeType
 }
const
<<<<<<< HEAD
	kTraditionalCharactersSelector = 0;
	kSimplifiedCharactersSelector = 1;
	kJIS1978CharactersSelector = 2;
	kJIS1983CharactersSelector = 3;
	kJIS1990CharactersSelector = 4;
	kTraditionalAltOneSelector = 5;
	kTraditionalAltTwoSelector = 6;
	kTraditionalAltThreeSelector = 7;
	kTraditionalAltFourSelector = 8;
	kTraditionalAltFiveSelector = 9;
	kExpertCharactersSelector = 10;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kJIS2004CharactersSelector = 11;
	kHojoCharactersSelector = 12;
	kNLCCharactersSelector = 13;
	kTraditionalNamesCharactersSelector = 14;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{
 *  Summary:
 *    Selectors for feature type kNumberCaseType
 }
const
	kLowerCaseNumbersSelector = 0;
	kUpperCaseNumbersSelector = 1;


{
 *  Summary:
 *    Selectors for feature type kTextSpacingType
 }
const
	kProportionalTextSelector = 0;
	kMonospacedTextSelector = 1;
	kHalfWidthTextSelector = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kThirdWidthTextSelector = 3;
	kQuarterWidthTextSelector = 4;
	kAltProportionalTextSelector = 5;
	kAltHalfWidthTextSelector = 6;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{
 *  Summary:
 *    Selectors for feature type kTransliterationType
 }
const
	kNoTransliterationSelector = 0;
	kHanjaToHangulSelector = 1;
	kHiraganaToKatakanaSelector = 2;
	kKatakanaToHiraganaSelector = 3;
	kKanaToRomanizationSelector = 4;
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  Summary:
	 *    Selectors for feature type kMathematicalExtrasType
	 	}
	kHyphenToMinusOnSelector	= 0;
	kHyphenToMinusOffSelector	= 1;
	kAsteriskToMultiplyOnSelector = 2;
	kAsteriskToMultiplyOffSelector = 3;
	kSlashToDivideOnSelector	= 4;
	kSlashToDivideOffSelector	= 5;
	kInequalityLigaturesOnSelector = 6;
	kInequalityLigaturesOffSelector = 7;
	kExponentsOnSelector		= 8;
	kExponentsOffSelector		= 9;


	{
	 *  Summary:
	 *    Selectors for feature type kOrnamentSetsType
	 	}
	kNoOrnamentsSelector		= 0;
	kDingbatsSelector			= 1;
	kPiCharactersSelector		= 2;
	kFleuronsSelector			= 3;
	kDecorativeBordersSelector	= 4;
	kInternationalSymbolsSelector = 5;
	kMathSymbolsSelector		= 6;


	{
	 *  Summary:
	 *    Selectors for feature type kCharacterAlternativesType
	 	}
	kNoAlternatesSelector		= 0;


	{
	 *  Summary:
	 *    Selectors for feature type kDesignComplexityType
	 	}
	kDesignLevel1Selector		= 0;
	kDesignLevel2Selector		= 1;
	kDesignLevel3Selector		= 2;
	kDesignLevel4Selector		= 3;
	kDesignLevel5Selector		= 4;


	{
	 *  Summary:
	 *    Selectors for feature type kStyleOptionsType
	 	}
	kNoStyleOptionsSelector		= 0;
	kDisplayTextSelector		= 1;
	kEngravedTextSelector		= 2;
	kIlluminatedCapsSelector	= 3;
	kTitlingCapsSelector		= 4;
	kTallCapsSelector			= 5;


	{
	 *  Summary:
	 *    Selectors for feature type kCharacterShapeType
	 	}
=======
>>>>>>> origin/fixes_2.4
	kTraditionalCharactersSelector = 0;
	kSimplifiedCharactersSelector = 1;
	kJIS1978CharactersSelector = 2;
	kJIS1983CharactersSelector = 3;
	kJIS1990CharactersSelector = 4;
	kTraditionalAltOneSelector = 5;
	kTraditionalAltTwoSelector = 6;
	kTraditionalAltThreeSelector = 7;
<<<<<<< HEAD
	kTraditionalAltFourSelector	= 8;
	kTraditionalAltFiveSelector	= 9;
	kExpertCharactersSelector	= 10;


	{
	 *  Summary:
	 *    Selectors for feature type kNumberCaseType
	 	}
	kLowerCaseNumbersSelector	= 0;
	kUpperCaseNumbersSelector	= 1;


	{
	 *  Summary:
	 *    Selectors for feature type kTextSpacingType
	 	}
	kProportionalTextSelector	= 0;
	kMonospacedTextSelector		= 1;
	kHalfWidthTextSelector		= 2;
	kNormallySpacedTextSelector	= 3;


	{
	 *  Summary:
	 *    Selectors for feature type kTransliterationType
	 	}
	kNoTransliterationSelector	= 0;
	kHanjaToHangulSelector		= 1;
	kHiraganaToKatakanaSelector	= 2;
	kKatakanaToHiraganaSelector	= 3;
	kKanaToRomanizationSelector	= 4;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kTraditionalAltFourSelector = 8;
	kTraditionalAltFiveSelector = 9;
	kExpertCharactersSelector = 10;


{
 *  Summary:
 *    Selectors for feature type kNumberCaseType
 }
const
	kLowerCaseNumbersSelector = 0;
	kUpperCaseNumbersSelector = 1;


{
 *  Summary:
 *    Selectors for feature type kTextSpacingType
 }
const
	kProportionalTextSelector = 0;
	kMonospacedTextSelector = 1;
	kHalfWidthTextSelector = 2;


{
 *  Summary:
 *    Selectors for feature type kTransliterationType
 }
const
	kNoTransliterationSelector = 0;
	kHanjaToHangulSelector = 1;
	kHiraganaToKatakanaSelector = 2;
	kKatakanaToHiraganaSelector = 3;
	kKanaToRomanizationSelector = 4;
>>>>>>> origin/fixes_2.4
	kRomanizationToHiraganaSelector = 5;
	kRomanizationToKatakanaSelector = 6;
	kHanjaToHangulAltOneSelector = 7;
	kHanjaToHangulAltTwoSelector = 8;
	kHanjaToHangulAltThreeSelector = 9;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Selectors for feature type kAnnotationType
 }
const
	kNoAnnotationSelector = 0;
	kBoxAnnotationSelector = 1;
<<<<<<< HEAD
	kRoundedBoxAnnotationSelector = 2;
	kCircleAnnotationSelector = 3;
	kInvertedCircleAnnotationSelector = 4;
	kParenthesisAnnotationSelector = 5;
	kPeriodAnnotationSelector = 6;
	kRomanNumeralAnnotationSelector = 7;
	kDiamondAnnotationSelector = 8;
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  Summary:
	 *    Selectors for feature type kAnnotationType
	 	}
	kNoAnnotationSelector		= 0;
	kBoxAnnotationSelector		= 1;
=======
>>>>>>> origin/fixes_2.4
	kRoundedBoxAnnotationSelector = 2;
	kCircleAnnotationSelector = 3;
	kInvertedCircleAnnotationSelector = 4;
	kParenthesisAnnotationSelector = 5;
	kPeriodAnnotationSelector = 6;
	kRomanNumeralAnnotationSelector = 7;
<<<<<<< HEAD
	kDiamondAnnotationSelector	= 8;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kDiamondAnnotationSelector = 8;
>>>>>>> origin/fixes_2.4
	kInvertedBoxAnnotationSelector = 9;
	kInvertedRoundedBoxAnnotationSelector = 10;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Selectors for feature type kKanaSpacingType
 }
const
	kFullWidthKanaSelector = 0;
	kProportionalKanaSelector = 1;
<<<<<<< HEAD


{
 *  Summary:
 *    Selectors for feature type kIdeographicSpacingType
 }
const
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  Summary:
	 *    Selectors for feature type kKanaSpacingType
	 	}
	kFullWidthKanaSelector		= 0;
	kProportionalKanaSelector	= 1;


	{
	 *  Summary:
	 *    Selectors for feature type kIdeographicSpacingType
	 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


{
 *  Summary:
 *    Selectors for feature type kIdeographicSpacingType
 }
const
>>>>>>> origin/fixes_2.4
	kFullWidthIdeographsSelector = 0;
	kProportionalIdeographsSelector = 1;
	kHalfWidthIdeographsSelector = 2;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Selectors for feature type kUnicodeDecompositionType
 }
const
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  Summary:
	 *    Selectors for feature type kUnicodeDecompositionType
	 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kCanonicalCompositionOnSelector = 0;
	kCanonicalCompositionOffSelector = 1;
	kCompatibilityCompositionOnSelector = 2;
	kCompatibilityCompositionOffSelector = 3;
	kTranscodingCompositionOnSelector = 4;
	kTranscodingCompositionOffSelector = 5;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Selectors for feature type kRubyKanaType
 }
const
	kNoRubyKanaSelector = 0;    { deprecated - use kRubyKanaOffSelector instead }
	kRubyKanaSelector = 1;    { deprecated - use kRubyKanaOnSelector instead }
	kRubyKanaOnSelector = 2;
	kRubyKanaOffSelector = 3;
<<<<<<< HEAD


{
 *  Summary:
 *    Selectors for feature type kCJKSymbolAlternativesType
 }
const
	kNoCJKSymbolAlternativesSelector = 0;
	kCJKSymbolAltOneSelector = 1;
	kCJKSymbolAltTwoSelector = 2;
	kCJKSymbolAltThreeSelector = 3;
	kCJKSymbolAltFourSelector = 4;
	kCJKSymbolAltFiveSelector = 5;


{
 *  Summary:
 *    Selectors for feature type kIdeographicAlternativesType
 }
const
	kNoIdeographicAlternativesSelector = 0;
	kIdeographicAltOneSelector = 1;
	kIdeographicAltTwoSelector = 2;
	kIdeographicAltThreeSelector = 3;
	kIdeographicAltFourSelector = 4;
	kIdeographicAltFiveSelector = 5;


{
 *  Summary:
 *    Selectors for feature type kCJKVerticalRomanPlacementType
 }
const
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  Summary:
	 *    Selectors for feature type kRubyKanaType
	 	}
	kNoRubyKanaSelector			= 0;
	kRubyKanaSelector			= 1;
=======
>>>>>>> origin/fixes_2.4


{
 *  Summary:
 *    Selectors for feature type kCJKSymbolAlternativesType
 }
const
	kNoCJKSymbolAlternativesSelector = 0;
	kCJKSymbolAltOneSelector = 1;
	kCJKSymbolAltTwoSelector = 2;
	kCJKSymbolAltThreeSelector = 3;
	kCJKSymbolAltFourSelector = 4;
	kCJKSymbolAltFiveSelector = 5;


{
 *  Summary:
 *    Selectors for feature type kIdeographicAlternativesType
 }
const
	kNoIdeographicAlternativesSelector = 0;
	kIdeographicAltOneSelector = 1;
	kIdeographicAltTwoSelector = 2;
	kIdeographicAltThreeSelector = 3;
	kIdeographicAltFourSelector = 4;
	kIdeographicAltFiveSelector = 5;


<<<<<<< HEAD
	{
	 *  Summary:
	 *    Selectors for feature type kCJKVerticalRomanPlacementType
	 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  Summary:
 *    Selectors for feature type kCJKVerticalRomanPlacementType
 }
const
>>>>>>> origin/fixes_2.4
	kCJKVerticalRomanCenteredSelector = 0;
	kCJKVerticalRomanHBaselineSelector = 1;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Selectors for feature type kItalicCJKRomanType
 }
const
	kNoCJKItalicRomanSelector = 0;    { deprecated - use kCJKItalicRomanOffSelector instead }
	kCJKItalicRomanSelector = 1;    { deprecated - use kCJKItalicRomanOnSelector instead }
	kCJKItalicRomanOnSelector = 2;
	kCJKItalicRomanOffSelector = 3;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{
 *  Summary:
 *    Selectors for feature type kCaseSensitiveLayoutType
 }
const
	kCaseSensitiveLayoutOnSelector = 0;
	kCaseSensitiveLayoutOffSelector = 1;
	kCaseSensitiveSpacingOnSelector = 2;
	kCaseSensitiveSpacingOffSelector = 3;


{
 *  Summary:
 *    Selectors for feature type kAlternateKanaType
 }
const
	kAlternateHorizKanaOnSelector = 0;
	kAlternateHorizKanaOffSelector = 1;
	kAlternateVertKanaOnSelector = 2;
	kAlternateVertKanaOffSelector = 3;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew


{
 *  Summary:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Selectors for feature type kStylisticAlternativesType
 }
const
	kNoStylisticAlternatesSelector = 0;
	kStylisticAltOneOnSelector = 2;
	kStylisticAltOneOffSelector = 3;
	kStylisticAltTwoOnSelector = 4;
	kStylisticAltTwoOffSelector = 5;
	kStylisticAltThreeOnSelector = 6;
	kStylisticAltThreeOffSelector = 7;
	kStylisticAltFourOnSelector = 8;
	kStylisticAltFourOffSelector = 9;
	kStylisticAltFiveOnSelector = 10;
	kStylisticAltFiveOffSelector = 11;
	kStylisticAltSixOnSelector = 12;
	kStylisticAltSixOffSelector = 13;
	kStylisticAltSevenOnSelector = 14;
	kStylisticAltSevenOffSelector = 15;
	kStylisticAltEightOnSelector = 16;
	kStylisticAltEightOffSelector = 17;
	kStylisticAltNineOnSelector = 18;
	kStylisticAltNineOffSelector = 19;
	kStylisticAltTenOnSelector = 20;
	kStylisticAltTenOffSelector = 21;
	kStylisticAltElevenOnSelector = 22;
	kStylisticAltElevenOffSelector = 23;
	kStylisticAltTwelveOnSelector = 24;
	kStylisticAltTwelveOffSelector = 25;
	kStylisticAltThirteenOnSelector = 26;
	kStylisticAltThirteenOffSelector = 27;
	kStylisticAltFourteenOnSelector = 28;
	kStylisticAltFourteenOffSelector = 29;
	kStylisticAltFifteenOnSelector = 30;
	kStylisticAltFifteenOffSelector = 31;
	kStylisticAltSixteenOnSelector = 32;
	kStylisticAltSixteenOffSelector = 33;
	kStylisticAltSeventeenOnSelector = 34;
	kStylisticAltSeventeenOffSelector = 35;
	kStylisticAltEighteenOnSelector = 36;
	kStylisticAltEighteenOffSelector = 37;
	kStylisticAltNineteenOnSelector = 38;
	kStylisticAltNineteenOffSelector = 39;
	kStylisticAltTwentyOnSelector = 40;
	kStylisticAltTwentyOffSelector = 41;


{
 *  Summary:
 *    Selectors for feature type kContextualAlternatesType
 }
const
	kContextualAlternatesOnSelector = 0;
	kContextualAlternatesOffSelector = 1;
	kSwashAlternatesOnSelector = 2;
	kSwashAlternatesOffSelector = 3;
	kContextualSwashAlternatesOnSelector = 4;
	kContextualSwashAlternatesOffSelector = 5;


{
 *  Summary:
 *    Selectors for feature type kLowerCaseType
 }
const
	kDefaultLowerCaseSelector = 0;
	kLowerCaseSmallCapsSelector = 1;
	kLowerCasePetiteCapsSelector = 2;


{
 *  Summary:
 *    Selectors for feature type kUpperCaseType
 }
const
	kDefaultUpperCaseSelector = 0;
	kUpperCaseSmallCapsSelector = 1;
	kUpperCasePetiteCapsSelector = 2;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{
 *  Summary:
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    Selectors for feature type kCJKRomanSpacingType
 }
const
	kHalfWidthCJKRomanSelector = 0;
	kProportionalCJKRomanSelector = 1;
	kDefaultCJKRomanSelector = 2;
	kFullWidthCJKRomanSelector = 3;

{ --------------------------------------------------------------------------- }
{ ---------------- Table Specific Typedefs and Constants -------------------- }
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: lookup tables - used within various other tables }
const
	kSFNTLookupSimpleArray = 0;    { a simple array indexed by glyph code }
	kSFNTLookupSegmentSingle = 2;    { segment mapping to single value }
	kSFNTLookupSegmentArray = 4;    { segment mapping to lookup array }
	kSFNTLookupSingleTable = 6;    { sorted list of glyph, value pairs }
	kSFNTLookupTrimmedArray = 8;     { a simple trimmed array indexed by glyph code }

type
	SFNTLookupTableFormat = UInt16;
	SFNTLookupValue = UInt16;
	SFNTLookupOffset = UInt16;
	SFNTLookupKind = UInt32;
{
    A BinarySearchHeader defines the five standard fields needed to perform quick
    lookups in a lookup table (note that using UInt16s, and not ItemCounts or
    similar types, is important here, since these tables are in fonts, and the
    documented font formats specify 16-bit quantities).
}
type
	SFNTLookupBinarySearchHeaderPtr = ^SFNTLookupBinarySearchHeader;
	SFNTLookupBinarySearchHeader = record
		unitSize: UInt16;               { size of a unit in bytes }
		nUnits: UInt16;                 { number of units in table }
		searchRange: UInt16;            { (largest power of two <= nUnits) * unitSize }
		entrySelector: UInt16;          { log2 (largest power of two <= nUnits) }
		rangeShift: UInt16;             { (nUnits - largest power of two <= nUnits) * unitSize }
	end;
{ A format 0 lookup table maps all glyphs in the font to lookup values }
type
	SFNTLookupArrayHeaderPtr = ^SFNTLookupArrayHeader;
	SFNTLookupArrayHeader = record
		lookupValues : array[0..0] of SFNTLookupValue;
	end;
{ A format 8 lookup table maps some range of glyphs in the font to lookup values }
type
	SFNTLookupTrimmedArrayHeaderPtr = ^SFNTLookupTrimmedArrayHeader;
	SFNTLookupTrimmedArrayHeader = record
		firstGlyph: UInt16;
		count: UInt16;
		valueArray : array[0..0] of SFNTLookupValue;
	end;
{
    Format 2 and format 4 lookup tables map ranges of glyphs to either single lookup
    values (format 2), or per-glyph lookup values (format 4). Since both formats
    use the same kind of data, only one unified set of segment-related structures
    is defined.
}
type
	SFNTLookupSegmentPtr = ^SFNTLookupSegment;
	SFNTLookupSegment = record
		lastGlyph: UInt16;
		firstGlyph: UInt16;
		value: array[0..0] of UInt16;
	end;
type
	SFNTLookupSegmentHeaderPtr = ^SFNTLookupSegmentHeader;
	SFNTLookupSegmentHeader = record
		binSearch: SFNTLookupBinarySearchHeader;
		segments: array[0..0] of SFNTLookupSegment;
	end;
{ A format 6 lookup table maps single glyphs to lookup values. }
type
	SFNTLookupSinglePtr = ^SFNTLookupSingle;
	SFNTLookupSingle = record
		glyph: UInt16;
		value: array[0..0] of UInt16;
	end;
type
	SFNTLookupSingleHeaderPtr = ^SFNTLookupSingleHeader;
	SFNTLookupSingleHeader = record
		binSearch: SFNTLookupBinarySearchHeader;
		entries: array[0..0] of SFNTLookupSingle;
	end;
{ The format-specific part of the subtable header }
	SFNTLookupFormatSpecificHeaderPtr = ^SFNTLookupFormatSpecificHeader;
 	SFNTLookupFormatSpecificHeader = record
 		case SInt16 of
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  Summary:
	 *    Selectors for feature type kItalicCJKRomanType
	 	}
	kNoCJKItalicRomanSelector	= 0;
	kCJKItalicRomanSelector		= 1;
=======
>>>>>>> origin/fixes_2.4


{
 *  Summary:
 *    Selectors for feature type kCJKRomanSpacingType
 }
const
	kHalfWidthCJKRomanSelector = 0;
	kProportionalCJKRomanSelector = 1;
	kDefaultCJKRomanSelector = 2;
	kFullWidthCJKRomanSelector = 3;

{ --------------------------------------------------------------------------- }
{ ---------------- Table Specific Typedefs and Constants -------------------- }
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: lookup tables - used within various other tables }
const
	kSFNTLookupSimpleArray = 0;    { a simple array indexed by glyph code }
	kSFNTLookupSegmentSingle = 2;    { segment mapping to single value }
	kSFNTLookupSegmentArray = 4;    { segment mapping to lookup array }
	kSFNTLookupSingleTable = 6;    { sorted list of glyph, value pairs }
	kSFNTLookupTrimmedArray = 8;     { a simple trimmed array indexed by glyph code }

type
	SFNTLookupTableFormat = UInt16;
	SFNTLookupValue = UInt16;
	SFNTLookupOffset = UInt16;
	SFNTLookupKind = UInt32;
{
    A BinarySearchHeader defines the five standard fields needed to perform quick
    lookups in a lookup table (note that using UInt16s, and not ItemCounts or
    similar types, is important here, since these tables are in fonts, and the
    documented font formats specify 16-bit quantities).
}
type
	SFNTLookupBinarySearchHeaderPtr = ^SFNTLookupBinarySearchHeader;
	SFNTLookupBinarySearchHeader = record
		unitSize: UInt16;               { size of a unit in bytes }
		nUnits: UInt16;                 { number of units in table }
		searchRange: UInt16;            { (largest power of two <= nUnits) * unitSize }
		entrySelector: UInt16;          { log2 (largest power of two <= nUnits) }
		rangeShift: UInt16;             { (nUnits - largest power of two <= nUnits) * unitSize }
	end;
{ A format 0 lookup table maps all glyphs in the font to lookup values }
type
	SFNTLookupArrayHeaderPtr = ^SFNTLookupArrayHeader;
	SFNTLookupArrayHeader = record
		lookupValues : array[0..0] of SFNTLookupValue;
	end;
{ A format 8 lookup table maps some range of glyphs in the font to lookup values }
type
	SFNTLookupTrimmedArrayHeaderPtr = ^SFNTLookupTrimmedArrayHeader;
	SFNTLookupTrimmedArrayHeader = record
		firstGlyph: UInt16;
		count: UInt16;
		valueArray : array[0..0] of SFNTLookupValue;
	end;
<<<<<<< HEAD

	{	
	    Format 2 and format 4 lookup tables map ranges of glyphs to either single lookup
	    values (format 2), or per-glyph lookup values (format 4). Since both formats
	    use the same kind of data, only one unified set of segment-related structures
	    is defined.
		}
	SFNTLookupSegmentPtr = ^SFNTLookupSegment;
	SFNTLookupSegment = record
		lastGlyph:				UInt16;
		firstGlyph:				UInt16;
<<<<<<< HEAD
		value:					array [0..0] of UInt16;
	end;

	SFNTLookupSegmentHeaderPtr = ^SFNTLookupSegmentHeader;
	SFNTLookupSegmentHeader = record
		binSearch:				SFNTLookupBinarySearchHeader;
		segments:				array [0..0] of SFNTLookupSegment;
	end;

	{	 A format 6 lookup table maps single glyphs to lookup values. 	}
	SFNTLookupSinglePtr = ^SFNTLookupSingle;
	SFNTLookupSingle = record
		glyph:					UInt16;
		value:					array [0..0] of UInt16;
	end;

	SFNTLookupSingleHeaderPtr = ^SFNTLookupSingleHeader;
	SFNTLookupSingleHeader = record
		binSearch:				SFNTLookupBinarySearchHeader;
		entries:				array [0..0] of SFNTLookupSingle;
	end;

	{	 The format-specific part of the subtable header 	}
	SFNTLookupFormatSpecificHeaderPtr = ^SFNTLookupFormatSpecificHeader;
	SFNTLookupFormatSpecificHeader = record
		case SInt16 of
>>>>>>> graemeg/fixes_2_2
		0: (
			theArray:			SFNTLookupArrayHeader;
			);
		1: (
			segment:			SFNTLookupSegmentHeader;
			);
		2: (
			single:				SFNTLookupSingleHeader;
			);
		3: (
			trimmedArray:		SFNTLookupTrimmedArrayHeader;
			);
	end;

<<<<<<< HEAD
{ The overall subtable header }
type
	SFNTLookupTable = record
		format: SFNTLookupTableFormat;              { table format }
		fsHeader: SFNTLookupFormatSpecificHeader;   { format specific header }
	end;
	SFNTLookupTablePtr = ^SFNTLookupTable;
	SFNTLookupTableHandle = ^SFNTLookupTablePtr;
{ --------------------------------------------------------------------------- }
{ GENERAL FORMATS FOR STATE TABLES -- prefix "ST" }
const
	kSTClassEndOfText = 0;
	kSTClassOutOfBounds = 1;
	kSTClassDeletedGlyph = 2;
	kSTClassEndOfLine = 3;
	kSTSetMark = $8000;
	kSTNoAdvance = $4000;
	kSTMarkEnd = $2000;
	kSTLigActionMask = $3FFF;
	kSTRearrVerbMask = $000F;

type
	STClass = UInt8;
	STEntryIndex = UInt8;
	STHeaderPtr = ^STHeader;
	STHeader = record
		filler: UInt8;
		nClasses: STClass;
		classTableOffset: UInt16;
		stateArrayOffset: UInt16;
		entryTableOffset: UInt16;
	end;
type
	STClassTablePtr = ^STClassTable;
	STClassTable = record
		firstGlyph: UInt16;
		nGlyphs: UInt16;
		classes: array[0..0] of STClass;
	end;
type
	STEntryZeroPtr = ^STEntryZero;
	STEntryZero = record
		newState: UInt16;
		flags: UInt16;
	end;
type
	STEntryOnePtr = ^STEntryOne;
	STEntryOne = record
		newState: UInt16;
		flags: UInt16;
		offset1: UInt16;
	end;
type
	STEntryTwoPtr = ^STEntryTwo;
	STEntryTwo = record
		newState: UInt16;
		flags: UInt16;
		offset1: UInt16;
		offset2: UInt16;
	end;
{ --------------------------------------------------------------------------- }
{ GENERAL FORMATS FOR STATE TABLES to be used with 'morx' tables -- prefix "STX" }
const
	kSTXHasLigAction = $2000;


type
	STXClass = UInt16;
	STXStateIndex = UInt16;
	STXEntryIndex = UInt16;
	STXHeaderPtr = ^STXHeader;
	STXHeader = record
		nClasses: UInt32;
		classTableOffset: UInt32;
		stateArrayOffset: UInt32;
		entryTableOffset: UInt32;
	end;

type
	STXClassTable = SFNTLookupTable;
	STXEntryZeroPtr = ^STXEntryZero;
	STXEntryZero = record
		newState: STXStateIndex;
		flags: UInt16;
	end;
type
	STXEntryOnePtr = ^STXEntryOne;
	STXEntryOne = record
		newState: STXStateIndex;
		flags: UInt16;
		index1: UInt16;
	end;
type
	STXEntryTwoPtr = ^STXEntryTwo;
	STXEntryTwo = record
		newState: STXStateIndex;
		flags: UInt16;
		index1: UInt16;
		index2: UInt16;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'lcar' }
{ CONSTANTS }
const
	kLCARTag = $6C636172; { 'lcar' }
	kLCARCurrentVersion = $00010000; { current version number for 'lcar' table }
	kLCARLinearFormat = 0;
	kLCARCtlPointFormat = 1;

{ TYPES }
type
	LcarCaretClassEntryPtr = ^LcarCaretClassEntry;
	LcarCaretClassEntry = record
		count: UInt16;
		partials: array[0..0] of UInt16;            { these are either FUnits or control-point numbers }
	end;
type
	LcarCaretTable = record
		version: Fixed;
		format: UInt16;
		lookup: SFNTLookupTable;
	end;
	LcarCaretTablePtr = ^LcarCaretTable;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'just' }
{ CONSTANTS }
const
	kJUSTTag = $6A757374; { 'just' }
	kJUSTCurrentVersion = $00010000;
	kJUSTStandardFormat = 0;
	kJUSTnoGlyphcode = $FFFF; { used in a pcConditionalAddAction }
	kJUSTpcDecompositionAction = 0;
	kJUSTpcUnconditionalAddAction = 1;
	kJUSTpcConditionalAddAction = 2;
	kJUSTpcGlyphStretchAction = 3;
	kJUSTpcDuctilityAction = 4;
	kJUSTpcGlyphRepeatAddAction = 5;

{ Justification priority levels }
const
	kJUSTKashidaPriority = 0;
	kJUSTSpacePriority = 1;
	kJUSTLetterPriority = 2;
	kJUSTNullPriority = 3;
	kJUSTPriorityCount = 4;

{ Justification flags }
const
	kJUSTOverridePriority = $8000;
	kJUSTOverrideLimits = $4000;
	kJUSTOverrideUnlimited = $2000;
	kJUSTUnlimited = $1000;
	kJUSTPriorityMask = $0003;

{ TYPES }
type
	JustPCActionType = UInt16;
	JustificationFlags = UInt16;
{ A JustPCDecompositionAction defines a ligature decomposition action. }
type
	JustPCDecompositionActionPtr = ^JustPCDecompositionAction;
	JustPCDecompositionAction = record
		lowerLimit: Fixed;
		upperLimit: Fixed;
		order: UInt16;
		count: UInt16;
		glyphs: array[0..0] of UInt16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	end;
{ A JUSTPCUnconditionalAddAction defines an unconditional glyph add action. }

type
	JustPCUnconditionalAddAction = UInt16;
{
    A JUSTPCConditionalAddAction defines a glyph substitution and add action. If the addGlyph
    is equal to kJUSTnoGlyphcode, then no glyph will be added, and the justification for
    the line will be redone.
}
type
	JustPCConditionalAddActionPtr = ^JustPCConditionalAddAction;
	JustPCConditionalAddAction = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		substThreshold: Fixed;        { threshold of growth factor at which subst occurs }
=======
		substThreshhold: Fixed;        { threshhold of growth factor at which subst occurs }
>>>>>>> graemeg/cpstrnew
		addGlyph: UInt16;
		substGlyph: UInt16;
	end;
=======
	end;
{ A JUSTPCUnconditionalAddAction defines an unconditional glyph add action. }

type
	JustPCUnconditionalAddAction = UInt16;
{
    A JUSTPCConditionalAddAction defines a glyph substitution and add action. If the addGlyph
    is equal to kJUSTnoGlyphcode, then no glyph will be added, and the justification for
    the line will be redone.
}
type
	JustPCConditionalAddActionPtr = ^JustPCConditionalAddAction;
	JustPCConditionalAddAction = record
		substThreshhold: Fixed;        { threshhold of growth factor at which subst occurs }
		addGlyph: UInt16;
		substGlyph: UInt16;
	end;
>>>>>>> graemeg/cpstrnew
=======
		substThreshhold: Fixed;        { threshhold of growth factor at which subst occurs }
		addGlyph: UInt16;
		substGlyph: UInt16;
	end;
>>>>>>> graemeg/cpstrnew
=======
		substThreshhold: Fixed;        { threshhold of growth factor at which subst occurs }
		addGlyph: UInt16;
		substGlyph: UInt16;
	end;
>>>>>>> origin/cpstrnew
{ A PCDuctilityAction defines a ductile axis along which the glyph will be varied. }
type
	JustPCDuctilityActionPtr = ^JustPCDuctilityAction;
	JustPCDuctilityAction = record
		ductilityAxis: UInt32;
		minimumLimit: Fixed;
		noStretchValue: Fixed;
		maximumLimit: Fixed;
	end;
{
    A PCGlyphRepetitionAction defines a glyph which will not be stretched or otherwise
    transformed, but rather which will be emplaced however many times are needed to fill
    the needed gap.
}
type
	JustPCGlyphRepeatAddActionPtr = ^JustPCGlyphRepeatAddAction;
	JustPCGlyphRepeatAddAction = record
		flags: UInt16;
		glyph: UInt16;
	end;
{ PCActionSubrecords contain the actual postcompensation actions. }
type
	JustPCActionSubrecordPtr = ^JustPCActionSubrecord;
	JustPCActionSubrecord = record
		theClass: UInt16;               { justification class value associated with this rec }
		theType: JustPCActionType;
		length: UInt32;
		data: UInt32;                   { not really a UInt32; cast as ptr to appropriate action }
	end;
{ The set of postcompensation records is defined in a PCAction struct. }
type
	JustPCActionPtr = ^JustPCAction;
	JustPCAction = record
		actionCount: UInt32;            { long for alignment purposes }
		actions: array[0..0] of JustPCActionSubrecord;
	end;
{
    JustWidthDeltaEntry is the justification table entry structure.  The justClass value (which is
    actually limited to 7 bits by the state table structure) is defined as a long for PPC alignment reasons.
}
type
	JustWidthDeltaEntryPtr = ^JustWidthDeltaEntry;
	JustWidthDeltaEntry = record
		justClass: UInt32;
		beforeGrowLimit: Fixed;        { ems AW can grow by at most on LT }
		beforeShrinkLimit: Fixed;      { ems AW can shrink by at most on LT }
		afterGrowLimit: Fixed;         { ems AW can grow by at most on RB }
		afterShrinkLimit: Fixed;       { ems AW can shrink by at most on RB }
		growFlags: JustificationFlags;              { flags controlling grow case }
		shrinkFlags: JustificationFlags;            { flags controlling shrink case }
	end;
type
	JustWidthDeltaGroupPtr = ^JustWidthDeltaGroup;
	JustWidthDeltaGroup = record
		count: UInt32;
		entries: array[0..0] of JustWidthDeltaEntry;
	end;
{ Overall structure of a postcompensation table is defined in PostcompTable. }
type
	JustPostcompTablePtr = ^JustPostcompTable;
	JustPostcompTable = record
		lookupTable: SFNTLookupTable;
                                              { action records here }
	end;
type
	JustDirectionTablePtr = ^JustDirectionTable;
	JustDirectionTable = record
		justClass: UInt16;              { offset to state table (0=none) }
		widthDeltaClusters: UInt16;     { offset to clusters }
		postcomp: UInt16;               { offset to postcomp table (0=none) }
		lookup: SFNTLookupTable;
	end;
type
	JustTablePtr = ^JustTable;
	JustTable = record
		version: Fixed;
		format: UInt16;
		horizHeaderOffset: UInt16;
		vertHeaderOffset: UInt16;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'opbd' }
{ CONSTANTS }
const
	kOPBDTag = $6F706264; { 'opbd' }
	kOPBDCurrentVersion = $00010000;
	kOPBDDistanceFormat = 0;
	kOPBDControlPointFormat = 1;

{ TYPES }

type
	OpbdTableFormat = UInt16;
{
    The OpbdSideValues struct is the lookup result from the FindSingle call for the
    optical tables. It contains the 4 FUnit values that are relevant to the specified
    glyph, or the 4 control gxPoint values.
}
type
	OpbdSideValuesPtr = ^OpbdSideValues;
	OpbdSideValues = record
		leftSideShift: SInt16;
		topSideShift: SInt16;
		rightSideShift: SInt16;
		bottomSideShift: SInt16;
	end;
type
	OpbdTablePtr = ^OpbdTable;
	OpbdTable = record
		version: Fixed;
		format: OpbdTableFormat;
		lookupTable: SFNTLookupTable;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'mort' }
{ CONSTANTS }
const
	kMORTTag = $6D6F7274; { 'mort' }
	kMORTCurrentVersion = $00010000; { current version number for 'mort' table }
                                        { Coverage masks }
	kMORTCoverVertical = $8000;
	kMORTCoverDescending = $4000;
	kMORTCoverIgnoreVertical = $2000;
	kMORTCoverTypeMask = $000F; { Subtable types }
	kMORTRearrangementType = 0;
	kMORTContextualType = 1;
	kMORTLigatureType = 2;
	kMORTSwashType = 4;
	kMORTInsertionType = 5;    { Ligature subtable constants }
	kMORTLigLastAction = $80000000;
	kMORTLigStoreLigature = $40000000;
	kMORTLigFormOffsetMask = $3FFFFFFF;
	kMORTLigFormOffsetShift = 2;    { Rearrangement subtable actions }
	kMORTraNoAction = 0;    {    no action   }
	kMORTraxA = 1;    {      Ax => xA    }
	kMORTraDx = 2;    {      xD => Dx    }
	kMORTraDxA = 3;    {     AxD => DxA   }
	kMORTraxAB = 4;    {   ABx => xAB   }
	kMORTraxBA = 5;    {   ABx => xBA   }
	kMORTraCDx = 6;    {   xCD => CDx   }
	kMORTraDCx = 7;    {   xCD => DCx   }
	kMORTraCDxA = 8;    {  AxCD => CDxA  }
	kMORTraDCxA = 9;    {  AxCD => DCxA  }
	kMORTraDxAB = 10;   {  ABxD => DxAB  }
	kMORTraDxBA = 11;   {  ABxD => DxBA  }
	kMORTraCDxAB = 12;   { ABxCD => CDxAB }
	kMORTraCDxBA = 13;   { ABxCD => CDxBA }
	kMORTraDCxAB = 14;   { ABxCD => DCxAB }
	kMORTraDCxBA = 15;   { ABxCD => DCxBA }
                                        { Insertion subtable constants }
	kMORTDoInsertionsBefore = $80;
	kMORTIsSplitVowelPiece = $40;
	kMORTInsertionsCountMask = $3F;
	kMORTCurrInsertKashidaLike = $2000;
	kMORTMarkInsertKashidaLike = $1000;
	kMORTCurrInsertBefore = $0800;
	kMORTMarkInsertBefore = $0400;
	kMORTMarkJustTableCountMask = $3F80;
	kMORTMarkJustTableCountShift = 7;    { JustTableIndex for marked character }
	kMORTCurrJustTableCountMask = $007F;
	kMORTCurrJustTableCountShift = 0;    { JustTableIndex for current character }
	kMORTCurrInsertCountMask = $03E0;
	kMORTCurrInsertCountShift = 5;    { count to insert after current glyphRec }
	kMORTMarkInsertCountMask = $001F;
	kMORTMarkInsertCountShift = 0;     { count to insert after marked glyphRec }

{ TYPES }

type
	MortSubtableMaskFlags = UInt32;
	MortLigatureActionEntry = UInt32;
	MortRearrangementSubtablePtr = ^MortRearrangementSubtable;
	MortRearrangementSubtable = record
		header: STHeader;
=======
=======
		value:					array [0..0] of UInt16;
=======
{
    Format 2 and format 4 lookup tables map ranges of glyphs to either single lookup
    values (format 2), or per-glyph lookup values (format 4). Since both formats
    use the same kind of data, only one unified set of segment-related structures
    is defined.
}
type
	SFNTLookupSegmentPtr = ^SFNTLookupSegment;
	SFNTLookupSegment = record
		lastGlyph: UInt16;
		firstGlyph: UInt16;
		value: array[0..0] of UInt16;
>>>>>>> origin/fixes_2.4
	end;
type
	SFNTLookupSegmentHeaderPtr = ^SFNTLookupSegmentHeader;
	SFNTLookupSegmentHeader = record
		binSearch: SFNTLookupBinarySearchHeader;
		segments: array[0..0] of SFNTLookupSegment;
	end;
{ A format 6 lookup table maps single glyphs to lookup values. }
type
	SFNTLookupSinglePtr = ^SFNTLookupSingle;
	SFNTLookupSingle = record
		glyph: UInt16;
		value: array[0..0] of UInt16;
	end;
type
	SFNTLookupSingleHeaderPtr = ^SFNTLookupSingleHeader;
	SFNTLookupSingleHeader = record
		binSearch: SFNTLookupBinarySearchHeader;
		entries: array[0..0] of SFNTLookupSingle;
	end;
{ The format-specific part of the subtable header }
	SFNTLookupFormatSpecificHeaderPtr = ^SFNTLookupFormatSpecificHeader;
 	SFNTLookupFormatSpecificHeader = record
 		case SInt16 of
		0: (
			theArray:			SFNTLookupArrayHeader;
			);
		1: (
			segment:			SFNTLookupSegmentHeader;
			);
		2: (
			single:				SFNTLookupSingleHeader;
			);
		3: (
			trimmedArray:		SFNTLookupTrimmedArrayHeader;
			);
	end;

<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
	{	 The overall subtable header 	}
	SFNTLookupTablePtr = ^SFNTLookupTable;
=======
{ The overall subtable header }
type
>>>>>>> origin/fixes_2.4
	SFNTLookupTable = record
		format: SFNTLookupTableFormat;              { table format }
		fsHeader: SFNTLookupFormatSpecificHeader;   { format specific header }
	end;
	SFNTLookupTablePtr = ^SFNTLookupTable;
	SFNTLookupTableHandle = ^SFNTLookupTablePtr;
{ --------------------------------------------------------------------------- }
{ GENERAL FORMATS FOR STATE TABLES -- prefix "ST" }
const
	kSTClassEndOfText = 0;
	kSTClassOutOfBounds = 1;
	kSTClassDeletedGlyph = 2;
	kSTClassEndOfLine = 3;
	kSTSetMark = $8000;
	kSTNoAdvance = $4000;
	kSTMarkEnd = $2000;
	kSTLigActionMask = $3FFF;
	kSTRearrVerbMask = $000F;

type
	STClass = UInt8;
	STEntryIndex = UInt8;
	STHeaderPtr = ^STHeader;
	STHeader = record
		filler: UInt8;
		nClasses: STClass;
		classTableOffset: UInt16;
		stateArrayOffset: UInt16;
		entryTableOffset: UInt16;
	end;
type
	STClassTablePtr = ^STClassTable;
	STClassTable = record
		firstGlyph: UInt16;
		nGlyphs: UInt16;
		classes: array[0..0] of STClass;
	end;
type
	STEntryZeroPtr = ^STEntryZero;
	STEntryZero = record
		newState: UInt16;
		flags: UInt16;
	end;
type
	STEntryOnePtr = ^STEntryOne;
	STEntryOne = record
		newState: UInt16;
		flags: UInt16;
		offset1: UInt16;
	end;
type
	STEntryTwoPtr = ^STEntryTwo;
	STEntryTwo = record
		newState: UInt16;
		flags: UInt16;
		offset1: UInt16;
		offset2: UInt16;
	end;
{ --------------------------------------------------------------------------- }
{ GENERAL FORMATS FOR STATE TABLES to be used with 'morx' tables -- prefix "STX" }
const
	kSTXHasLigAction = $2000;


type
	STXClass = UInt16;
	STXStateIndex = UInt16;
	STXEntryIndex = UInt16;
	STXHeaderPtr = ^STXHeader;
	STXHeader = record
		nClasses: UInt32;
		classTableOffset: UInt32;
		stateArrayOffset: UInt32;
		entryTableOffset: UInt32;
	end;

type
	STXClassTable = SFNTLookupTable;
	STXEntryZeroPtr = ^STXEntryZero;
	STXEntryZero = record
		newState: STXStateIndex;
		flags: UInt16;
	end;
type
	STXEntryOnePtr = ^STXEntryOne;
	STXEntryOne = record
		newState: STXStateIndex;
		flags: UInt16;
		index1: UInt16;
	end;
type
	STXEntryTwoPtr = ^STXEntryTwo;
	STXEntryTwo = record
		newState: STXStateIndex;
		flags: UInt16;
		index1: UInt16;
		index2: UInt16;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'lcar' }
{ CONSTANTS }
const
	kLCARTag = $6C636172; { 'lcar' }
	kLCARCurrentVersion = $00010000; { current version number for 'lcar' table }
	kLCARLinearFormat = 0;
	kLCARCtlPointFormat = 1;

{ TYPES }
type
	LcarCaretClassEntryPtr = ^LcarCaretClassEntry;
	LcarCaretClassEntry = record
		count: UInt16;
		partials: array[0..0] of UInt16;            { these are either FUnits or control-point numbers }
	end;
type
	LcarCaretTable = record
		version: Fixed;
		format: UInt16;
		lookup: SFNTLookupTable;
	end;
	LcarCaretTablePtr = ^LcarCaretTable;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'just' }
{ CONSTANTS }
const
	kJUSTTag = $6A757374; { 'just' }
	kJUSTCurrentVersion = $00010000;
	kJUSTStandardFormat = 0;
	kJUSTnoGlyphcode = $FFFF; { used in a pcConditionalAddAction }
	kJUSTpcDecompositionAction = 0;
	kJUSTpcUnconditionalAddAction = 1;
	kJUSTpcConditionalAddAction = 2;
	kJUSTpcGlyphStretchAction = 3;
	kJUSTpcDuctilityAction = 4;
	kJUSTpcGlyphRepeatAddAction = 5;

{ Justification priority levels }
const
	kJUSTKashidaPriority = 0;
	kJUSTSpacePriority = 1;
	kJUSTLetterPriority = 2;
	kJUSTNullPriority = 3;
	kJUSTPriorityCount = 4;

{ Justification flags }
const
	kJUSTOverridePriority = $8000;
	kJUSTOverrideLimits = $4000;
	kJUSTOverrideUnlimited = $2000;
	kJUSTUnlimited = $1000;
	kJUSTPriorityMask = $0003;

{ TYPES }
type
	JustPCActionType = UInt16;
	JustificationFlags = UInt16;
{ A JustPCDecompositionAction defines a ligature decomposition action. }
type
	JustPCDecompositionActionPtr = ^JustPCDecompositionAction;
	JustPCDecompositionAction = record
		lowerLimit: Fixed;
		upperLimit: Fixed;
		order: UInt16;
		count: UInt16;
		glyphs: array[0..0] of UInt16;
	end;
{ A JUSTPCUnconditionalAddAction defines an unconditional glyph add action. }

type
	JustPCUnconditionalAddAction = UInt16;
{
    A JUSTPCConditionalAddAction defines a glyph substitution and add action. If the addGlyph
    is equal to kJUSTnoGlyphcode, then no glyph will be added, and the justification for
    the line will be redone.
}
type
	JustPCConditionalAddActionPtr = ^JustPCConditionalAddAction;
	JustPCConditionalAddAction = record
		substThreshhold: Fixed;        { threshhold of growth factor at which subst occurs }
		addGlyph: UInt16;
		substGlyph: UInt16;
	end;
{ A PCDuctilityAction defines a ductile axis along which the glyph will be varied. }
type
	JustPCDuctilityActionPtr = ^JustPCDuctilityAction;
	JustPCDuctilityAction = record
		ductilityAxis: UInt32;
		minimumLimit: Fixed;
		noStretchValue: Fixed;
		maximumLimit: Fixed;
	end;
{
    A PCGlyphRepetitionAction defines a glyph which will not be stretched or otherwise
    transformed, but rather which will be emplaced however many times are needed to fill
    the needed gap.
}
type
	JustPCGlyphRepeatAddActionPtr = ^JustPCGlyphRepeatAddAction;
	JustPCGlyphRepeatAddAction = record
		flags: UInt16;
		glyph: UInt16;
	end;
{ PCActionSubrecords contain the actual postcompensation actions. }
type
	JustPCActionSubrecordPtr = ^JustPCActionSubrecord;
	JustPCActionSubrecord = record
		theClass: UInt16;               { justification class value associated with this rec }
		theType: JustPCActionType;
		length: UInt32;
		data: UInt32;                   { not really a UInt32; cast as ptr to appropriate action }
	end;
{ The set of postcompensation records is defined in a PCAction struct. }
type
	JustPCActionPtr = ^JustPCAction;
	JustPCAction = record
		actionCount: UInt32;            { long for alignment purposes }
		actions: array[0..0] of JustPCActionSubrecord;
	end;
{
    JustWidthDeltaEntry is the justification table entry structure.  The justClass value (which is
    actually limited to 7 bits by the state table structure) is defined as a long for PPC alignment reasons.
}
type
	JustWidthDeltaEntryPtr = ^JustWidthDeltaEntry;
	JustWidthDeltaEntry = record
		justClass: UInt32;
		beforeGrowLimit: Fixed;        { ems AW can grow by at most on LT }
		beforeShrinkLimit: Fixed;      { ems AW can shrink by at most on LT }
		afterGrowLimit: Fixed;         { ems AW can grow by at most on RB }
		afterShrinkLimit: Fixed;       { ems AW can shrink by at most on RB }
		growFlags: JustificationFlags;              { flags controlling grow case }
		shrinkFlags: JustificationFlags;            { flags controlling shrink case }
	end;
type
	JustWidthDeltaGroupPtr = ^JustWidthDeltaGroup;
	JustWidthDeltaGroup = record
		count: UInt32;
		entries: array[0..0] of JustWidthDeltaEntry;
	end;
{ Overall structure of a postcompensation table is defined in PostcompTable. }
type
	JustPostcompTablePtr = ^JustPostcompTable;
	JustPostcompTable = record
		lookupTable: SFNTLookupTable;
                                              { action records here }
	end;
type
	JustDirectionTablePtr = ^JustDirectionTable;
	JustDirectionTable = record
		justClass: UInt16;              { offset to state table (0=none) }
		widthDeltaClusters: UInt16;     { offset to clusters }
		postcomp: UInt16;               { offset to postcomp table (0=none) }
		lookup: SFNTLookupTable;
	end;
type
	JustTablePtr = ^JustTable;
	JustTable = record
		version: Fixed;
		format: UInt16;
		horizHeaderOffset: UInt16;
		vertHeaderOffset: UInt16;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'opbd' }
{ CONSTANTS }
const
	kOPBDTag = $6F706264; { 'opbd' }
	kOPBDCurrentVersion = $00010000;
	kOPBDDistanceFormat = 0;
	kOPBDControlPointFormat = 1;

{ TYPES }

type
	OpbdTableFormat = UInt16;
{
    The OpbdSideValues struct is the lookup result from the FindSingle call for the
    optical tables. It contains the 4 FUnit values that are relevant to the specified
    glyph, or the 4 control gxPoint values.
}
type
	OpbdSideValuesPtr = ^OpbdSideValues;
	OpbdSideValues = record
		leftSideShift: SInt16;
		topSideShift: SInt16;
		rightSideShift: SInt16;
		bottomSideShift: SInt16;
	end;
type
	OpbdTablePtr = ^OpbdTable;
	OpbdTable = record
		version: Fixed;
		format: OpbdTableFormat;
		lookupTable: SFNTLookupTable;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'mort' }
{ CONSTANTS }
const
	kMORTTag = $6D6F7274; { 'mort' }
	kMORTCurrentVersion = $00010000; { current version number for 'mort' table }
                                        { Coverage masks }
	kMORTCoverVertical = $8000;
	kMORTCoverDescending = $4000;
	kMORTCoverIgnoreVertical = $2000;
	kMORTCoverTypeMask = $000F; { Subtable types }
	kMORTRearrangementType = 0;
	kMORTContextualType = 1;
	kMORTLigatureType = 2;
	kMORTSwashType = 4;
	kMORTInsertionType = 5;    { Ligature subtable constants }
	kMORTLigLastAction = $80000000;
	kMORTLigStoreLigature = $40000000;
	kMORTLigFormOffsetMask = $3FFFFFFF;
	kMORTLigFormOffsetShift = 2;    { Rearrangement subtable actions }
	kMORTraNoAction = 0;    {    no action   }
	kMORTraxA = 1;    {      Ax => xA    }
	kMORTraDx = 2;    {      xD => Dx    }
	kMORTraDxA = 3;    {     AxD => DxA   }
	kMORTraxAB = 4;    {   ABx => xAB   }
	kMORTraxBA = 5;    {   ABx => xBA   }
	kMORTraCDx = 6;    {   xCD => CDx   }
	kMORTraDCx = 7;    {   xCD => DCx   }
	kMORTraCDxA = 8;    {  AxCD => CDxA  }
	kMORTraDCxA = 9;    {  AxCD => DCxA  }
	kMORTraDxAB = 10;   {  ABxD => DxAB  }
	kMORTraDxBA = 11;   {  ABxD => DxBA  }
	kMORTraCDxAB = 12;   { ABxCD => CDxAB }
	kMORTraCDxBA = 13;   { ABxCD => CDxBA }
	kMORTraDCxAB = 14;   { ABxCD => DCxAB }
	kMORTraDCxBA = 15;   { ABxCD => DCxBA }
                                        { Insertion subtable constants }
	kMORTDoInsertionsBefore = $80;
	kMORTIsSplitVowelPiece = $40;
	kMORTInsertionsCountMask = $3F;
	kMORTCurrInsertKashidaLike = $2000;
	kMORTMarkInsertKashidaLike = $1000;
	kMORTCurrInsertBefore = $0800;
	kMORTMarkInsertBefore = $0400;
	kMORTMarkJustTableCountMask = $3F80;
	kMORTMarkJustTableCountShift = 7;    { JustTableIndex for marked character }
	kMORTCurrJustTableCountMask = $007F;
	kMORTCurrJustTableCountShift = 0;    { JustTableIndex for current character }
	kMORTCurrInsertCountMask = $03E0;
	kMORTCurrInsertCountShift = 5;    { count to insert after current glyphRec }
	kMORTMarkInsertCountMask = $001F;
	kMORTMarkInsertCountShift = 0;     { count to insert after marked glyphRec }

{ TYPES }

type
	MortSubtableMaskFlags = UInt32;
	MortLigatureActionEntry = UInt32;
	MortRearrangementSubtablePtr = ^MortRearrangementSubtable;
	MortRearrangementSubtable = record
<<<<<<< HEAD
		header:					STHeader;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		header: STHeader;
>>>>>>> origin/fixes_2.4
	end;

	MortContextualSubtablePtr = ^MortContextualSubtable;
	MortContextualSubtable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		header: STHeader;
=======
		header:					STHeader;
>>>>>>> graemeg/fixes_2_2
=======
		header:					STHeader;
>>>>>>> origin/fixes_2_2
=======
		header: STHeader;
>>>>>>> origin/fixes_2.4
		substitutionTableOffset: UInt16;
	end;

	MortLigatureSubtablePtr = ^MortLigatureSubtable;
	MortLigatureSubtable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		header: STHeader;
		ligatureActionTableOffset: UInt16;
		componentTableOffset: UInt16;
		ligatureTableOffset: UInt16;
	end;
	
	MortSwashSubtablePtr = ^MortSwashSubtable;
	MortSwashSubtable = record
		lookup: SFNTLookupTable;
=======
=======
>>>>>>> origin/fixes_2_2
		header:					STHeader;
=======
		header: STHeader;
>>>>>>> origin/fixes_2.4
		ligatureActionTableOffset: UInt16;
		componentTableOffset: UInt16;
		ligatureTableOffset: UInt16;
	end;
	
	MortSwashSubtablePtr = ^MortSwashSubtable;
	MortSwashSubtable = record
<<<<<<< HEAD
		lookup:					SFNTLookupTable;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		lookup: SFNTLookupTable;
>>>>>>> origin/fixes_2.4
	end;

	MortInsertionSubtablePtr = ^MortInsertionSubtable;
	MortInsertionSubtable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		header: STHeader;
	end;
	
=======
		header:					STHeader;
	end;

>>>>>>> graemeg/fixes_2_2
=======
		header:					STHeader;
	end;

>>>>>>> origin/fixes_2_2
=======
		header: STHeader;
	end;
	
>>>>>>> origin/fixes_2.4
	MortSpecificSubtablePtr = ^MortSpecificSubtable;
	MortSpecificSubtable = record
		case SInt16 of
		0: (
			rearrangement:		MortRearrangementSubtable;
			);
		1: (
			contextual:			MortContextualSubtable;
			);
		2: (
			ligature:			MortLigatureSubtable;
			);
		3: (
			swash:				MortSwashSubtable;
			);
		4: (
			insertion:			MortInsertionSubtable;
			);
	end;

	MortSubtablePtr = ^MortSubtable;
	MortSubtable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		length: UInt16;
		coverage: UInt16;
		flags: MortSubtableMaskFlags;
		u: MortSpecificSubtable;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		length:					UInt16;
		coverage:				UInt16;
		flags:					MortSubtableMaskFlags;
		u:						MortSpecificSubtable;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;

	MortFeatureEntryPtr = ^MortFeatureEntry;
	MortFeatureEntry = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		featureType: UInt16;
		featureSelector: UInt16;
		enableFlags: MortSubtableMaskFlags;
		disableFlags: MortSubtableMaskFlags;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		featureType:			UInt16;
		featureSelector:		UInt16;
		enableFlags:			MortSubtableMaskFlags;
		disableFlags:			MortSubtableMaskFlags;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;

	MortChainPtr = ^MortChain;
	MortChain = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		defaultFlags: MortSubtableMaskFlags;        { default flags for this chain }
		length: UInt32;                 { byte length of this chain }
		nFeatures: UInt16;              { number of feature entries }
		nSubtables: UInt16;             { number of subtables }
		featureEntries: array[0..0] of MortFeatureEntry;
                                              { the subtables follow }
<<<<<<< HEAD
	end;
	
	MortTablePtr = ^MortTable;
	MortTable = record
		version: Fixed;
		nChains: UInt32;
		chains: array[0..0] of MortChain;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'morx' (version 2 and beyond metamorphosis tables) }
{ CONSTANTS }
const
	kMORXTag = $6D6F7278; { 'morx' }
	kMORXCurrentVersion = $00020000; { version number for current 'morx' table }
                                        { Coverage masks }
	kMORXCoverVertical = $80000000;
	kMORXCoverDescending = $40000000;
	kMORXCoverIgnoreVertical = $20000000;
	kMORXCoverTypeMask = $000000FF;

{ TYPES }
type
	MorxRearrangementSubtablePtr = ^MorxRearrangementSubtable;
	MorxRearrangementSubtable = record
		header: STXHeader;
=======
=======
>>>>>>> origin/fixes_2_2
		defaultFlags:			MortSubtableMaskFlags;					{  default flags for this chain  }
		length:					UInt32;									{  byte length of this chain  }
		nFeatures:				UInt16;									{  number of feature entries  }
		nSubtables:				UInt16;									{  number of subtables  }
		featureEntries:			array [0..0] of MortFeatureEntry;
																		{  the subtables follow  }
=======
>>>>>>> origin/fixes_2.4
	end;
	
	MortTablePtr = ^MortTable;
	MortTable = record
		version: Fixed;
		nChains: UInt32;
		chains: array[0..0] of MortChain;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'morx' (version 2 and beyond metamorphosis tables) }
{ CONSTANTS }
const
	kMORXTag = $6D6F7278; { 'morx' }
	kMORXCurrentVersion = $00020000; { version number for current 'morx' table }
                                        { Coverage masks }
	kMORXCoverVertical = $80000000;
	kMORXCoverDescending = $40000000;
	kMORXCoverIgnoreVertical = $20000000;
	kMORXCoverTypeMask = $000000FF;

{ TYPES }
type
	MorxRearrangementSubtablePtr = ^MorxRearrangementSubtable;
	MorxRearrangementSubtable = record
<<<<<<< HEAD
		header:					STXHeader;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		header: STXHeader;
>>>>>>> origin/fixes_2.4
	end;

	MorxContextualSubtablePtr = ^MorxContextualSubtable;
	MorxContextualSubtable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		header: STXHeader;
=======
		header:					STXHeader;
>>>>>>> graemeg/fixes_2_2
=======
		header:					STXHeader;
>>>>>>> origin/fixes_2_2
=======
		header: STXHeader;
>>>>>>> origin/fixes_2.4
		substitutionTableOffset: UInt32;
	end;

	MorxLigatureSubtablePtr = ^MorxLigatureSubtable;
	MorxLigatureSubtable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		header: STXHeader;
		ligatureActionTableOffset: UInt32;
		componentTableOffset: UInt32;
		ligatureTableOffset: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2
		header:					STXHeader;
		ligatureActionTableOffset: UInt32;
		componentTableOffset:	UInt32;
		ligatureTableOffset:	UInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		header: STXHeader;
		ligatureActionTableOffset: UInt32;
		componentTableOffset: UInt32;
		ligatureTableOffset: UInt32;
>>>>>>> origin/fixes_2.4
	end;

	MorxInsertionSubtablePtr = ^MorxInsertionSubtable;
	MorxInsertionSubtable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		header: STXHeader;
		insertionGlyphTableOffset: UInt32;
	end;
	
=======
=======
>>>>>>> origin/fixes_2_2
		header:					STXHeader;
		insertionGlyphTableOffset: UInt32;
	end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		header: STXHeader;
		insertionGlyphTableOffset: UInt32;
	end;
	
>>>>>>> origin/fixes_2.4
	MorxSpecificSubtablePtr = ^MorxSpecificSubtable;
	MorxSpecificSubtable = record
		case SInt16 of
		0: (
			rearrangement:		MorxRearrangementSubtable;
			);
		1: (
			contextual:			MorxContextualSubtable;
			);
		2: (
			ligature:			MorxLigatureSubtable;
			);
		3: (
			swash:				MortSwashSubtable;
			);
		4: (
			insertion:			MorxInsertionSubtable;
			);
	end;

	MorxSubtablePtr = ^MorxSubtable;
	MorxSubtable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		length: UInt32;
		coverage: UInt32;
		flags: MortSubtableMaskFlags;
		u: MorxSpecificSubtable;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		length:					UInt32;
		coverage:				UInt32;
		flags:					MortSubtableMaskFlags;
		u:						MorxSpecificSubtable;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;

	MorxChainPtr = ^MorxChain;
	MorxChain = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		defaultFlags: MortSubtableMaskFlags;        { default flags for this chain }
		length: UInt32;                 { byte length of this chain }
		nFeatures: UInt32;              { number of feature entries }
		nSubtables: UInt32;             { number of subtables }
		featureEntries: array[0..0] of MortFeatureEntry;
                                              { the subtables follow }
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		defaultFlags:			MortSubtableMaskFlags;					{  default flags for this chain  }
		length:					UInt32;									{  byte length of this chain  }
		nFeatures:				UInt32;									{  number of feature entries  }
		nSubtables:				UInt32;									{  number of subtables  }
		featureEntries:			array [0..0] of MortFeatureEntry;
																		{  the subtables follow  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;

	MorxTablePtr = ^MorxTable;
	MorxTable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		version: Fixed;
		nChains: UInt32;
		chains: array[0..0] of MorxChain;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'prop' }
{ CONSTANTS }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
	kPROPTag = $70726F70; { 'prop' }
	kPROPCurrentVersion = $00030000; { current version number for 'prop' table }
	kPROPPairOffsetShift = 8;
	kPROPPairOffsetSign = 7;
	kPROPIsFloaterMask = $8000; { glyph is floater }
	kPROPCanHangLTMask = $4000; { glyph can hang left/top }
	kPROPCanHangRBMask = $2000; { glyph can hang right/bottom }
	kPROPUseRLPairMask = $1000; { if glyph lands in RL streak, use paired glyph }
	kPROPPairOffsetMask = $0F00; { 4-bit signed offset to other pair member }
	kPROPRightConnectMask = $0080; { glyph connects to glyph on right }
	kPROPZeroReserved = $0060; { must be zero }
	kPROPDirectionMask = $001F; { direction bits }

{ These are the Unicode direction classes (plus the Special European Number class). }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
const
	kPROPTag = $70726F70; { 'prop' }
	kPROPCurrentVersion = $00030000; { current version number for 'prop' table }
	kPROPPairOffsetShift = 8;
	kPROPPairOffsetSign = 7;
	kPROPIsFloaterMask = $8000; { glyph is floater }
	kPROPCanHangLTMask = $4000; { glyph can hang left/top }
	kPROPCanHangRBMask = $2000; { glyph can hang right/bottom }
	kPROPUseRLPairMask = $1000; { if glyph lands in RL streak, use paired glyph }
	kPROPPairOffsetMask = $0F00; { 4-bit signed offset to other pair member }
	kPROPRightConnectMask = $0080; { glyph connects to glyph on right }
	kPROPZeroReserved = $0060; { must be zero }
	kPROPDirectionMask = $001F; { direction bits }

{ These are the Unicode direction classes (plus the Special European Number class). }
const
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kPROPLDirectionClass = 0;    { Left-to-Right }
	kPROPRDirectionClass = 1;    { Right-to-Left }
	kPROPALDirectionClass = 2;    { Right-to-Left Arabic Letter }
	kPROPENDirectionClass = 3;    { European Number }
	kPROPESDirectionClass = 4;    { European Number Seperator }
	kPROPETDirectionClass = 5;    { European Number Terminator }
	kPROPANDirectionClass = 6;    { Arabic Number }
	kPROPCSDirectionClass = 7;    { Common Number Seperator }
	kPROPPSDirectionClass = 8;    { Paragraph Seperator (also referred to as Block Separator) }
	kPROPSDirectionClass = 9;    { Segment Seperator }
	kPROPWSDirectionClass = 10;   { Whitespace }
	kPROPONDirectionClass = 11;   { Other Neutral }
	kPROPSENDirectionClass = 12;   { Special European Number (not a Unicode class) }
	kPROPLREDirectionClass = 13;   { Left-to-Right Embeding }
	kPROPLRODirectionClass = 14;   { Left-to-Right Override }
	kPROPRLEDirectionClass = 15;   { Right-to-Left Embeding }
	kPROPRLODirectionClass = 16;   { Right-to-Left Override }
	kPROPPDFDirectionClass = 17;   { Pop Directional Format }
	kPROPNSMDirectionClass = 18;   { Non-Spacing Mark }
	kPROPBNDirectionClass = 19;   { Boundary Neutral }
	kPROPNumDirectionClasses = 20;    { Number of Unicode directional types + Special European Number }

{ TYPES }

type
	PropCharProperties = UInt16;
	PropTablePtr = ^PropTable;
	PropTable = record
		version: Fixed;
		format: UInt16;
		defaultProps: PropCharProperties;
		lookup: SFNTLookupTable;
=======
=======
>>>>>>> origin/fixes_2_2
		version:				Fixed;
		nChains:				UInt32;
		chains:					array [0..0] of MorxChain;
=======
		version: Fixed;
		nChains: UInt32;
		chains: array[0..0] of MorxChain;
>>>>>>> origin/fixes_2.4
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'prop' }
{ CONSTANTS }
const
	kPROPTag = $70726F70; { 'prop' }
	kPROPCurrentVersion = $00030000; { current version number for 'prop' table }
	kPROPPairOffsetShift = 8;
	kPROPPairOffsetSign = 7;
	kPROPIsFloaterMask = $8000; { glyph is floater }
	kPROPCanHangLTMask = $4000; { glyph can hang left/top }
	kPROPCanHangRBMask = $2000; { glyph can hang right/bottom }
	kPROPUseRLPairMask = $1000; { if glyph lands in RL streak, use paired glyph }
	kPROPPairOffsetMask = $0F00; { 4-bit signed offset to other pair member }
	kPROPRightConnectMask = $0080; { glyph connects to glyph on right }
	kPROPZeroReserved = $0060; { must be zero }
	kPROPDirectionMask = $001F; { direction bits }

{ These are the Unicode direction classes (plus the Special European Number class). }
const
	kPROPLDirectionClass = 0;    { Left-to-Right }
	kPROPRDirectionClass = 1;    { Right-to-Left }
	kPROPALDirectionClass = 2;    { Right-to-Left Arabic Letter }
	kPROPENDirectionClass = 3;    { European Number }
	kPROPESDirectionClass = 4;    { European Number Seperator }
	kPROPETDirectionClass = 5;    { European Number Terminator }
	kPROPANDirectionClass = 6;    { Arabic Number }
	kPROPCSDirectionClass = 7;    { Common Number Seperator }
	kPROPPSDirectionClass = 8;    { Paragraph Seperator (also referred to as Block Separator) }
	kPROPSDirectionClass = 9;    { Segment Seperator }
	kPROPWSDirectionClass = 10;   { Whitespace }
	kPROPONDirectionClass = 11;   { Other Neutral }
	kPROPSENDirectionClass = 12;   { Special European Number (not a Unicode class) }
	kPROPLREDirectionClass = 13;   { Left-to-Right Embeding }
	kPROPLRODirectionClass = 14;   { Left-to-Right Override }
	kPROPRLEDirectionClass = 15;   { Right-to-Left Embeding }
	kPROPRLODirectionClass = 16;   { Right-to-Left Override }
	kPROPPDFDirectionClass = 17;   { Pop Directional Format }
	kPROPNSMDirectionClass = 18;   { Non-Spacing Mark }
	kPROPBNDirectionClass = 19;   { Boundary Neutral }
	kPROPNumDirectionClasses = 20;    { Number of Unicode directional types + Special European Number }

{ TYPES }

type
	PropCharProperties = UInt16;
	PropTablePtr = ^PropTable;
	PropTable = record
<<<<<<< HEAD
		version:				Fixed;
		format:					UInt16;
		defaultProps:			PropCharProperties;
		lookup:					SFNTLookupTable;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		version: Fixed;
		format: UInt16;
		defaultProps: PropCharProperties;
		lookup: SFNTLookupTable;
>>>>>>> origin/fixes_2.4
	end;

	PropLookupSegmentPtr = ^PropLookupSegment;
	PropLookupSegment = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		lastGlyph: UInt16;
		firstGlyph: UInt16;
		value: UInt16;
=======
		lastGlyph:				UInt16;
		firstGlyph:				UInt16;
		value:					UInt16;
>>>>>>> graemeg/fixes_2_2
=======
		lastGlyph:				UInt16;
		firstGlyph:				UInt16;
		value:					UInt16;
>>>>>>> origin/fixes_2_2
=======
		lastGlyph: UInt16;
		firstGlyph: UInt16;
		value: UInt16;
>>>>>>> origin/fixes_2.4
	end;

	PropLookupSinglePtr = ^PropLookupSingle;
	PropLookupSingle = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		glyph: UInt16;
		props: PropCharProperties;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'trak' }
{ CONSTANTS }
const
	kTRAKTag = $7472616B; { 'trak' }
	kTRAKCurrentVersion = $00010000; { current version number for 'trak' table }
	kTRAKUniformFormat = 0;     {    kTRAKPerGlyphFormat         = 2}

{ TYPES }

type
	TrakValue = SInt16;
	TrakTableEntryPtr = ^TrakTableEntry;
	TrakTableEntry = record
		track: Fixed;
		nameTableIndex: UInt16;
		sizesOffset: UInt16;            { offset to array of TrackingValues }
=======
=======
>>>>>>> origin/fixes_2_2
		glyph:					UInt16;
		props:					PropCharProperties;
=======
		glyph: UInt16;
		props: PropCharProperties;
>>>>>>> origin/fixes_2.4
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'trak' }
{ CONSTANTS }
const
	kTRAKTag = $7472616B; { 'trak' }
	kTRAKCurrentVersion = $00010000; { current version number for 'trak' table }
	kTRAKUniformFormat = 0;     {    kTRAKPerGlyphFormat         = 2}

{ TYPES }

type
	TrakValue = SInt16;
	TrakTableEntryPtr = ^TrakTableEntry;
	TrakTableEntry = record
<<<<<<< HEAD
		track:					Fixed;
		nameTableIndex:			UInt16;
		sizesOffset:			UInt16;									{  offset to array of TrackingValues  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		track: Fixed;
		nameTableIndex: UInt16;
		sizesOffset: UInt16;            { offset to array of TrackingValues }
>>>>>>> origin/fixes_2.4
	end;

	TrakTableDataPtr = ^TrakTableData;
	TrakTableData = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		nTracks: UInt16;
		nSizes: UInt16;
		sizeTableOffset: UInt32;
		trakTable: array[0..0] of TrakTableEntry;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		nTracks:				UInt16;
		nSizes:					UInt16;
		sizeTableOffset:		UInt32;
		trakTable:				array [0..0] of TrakTableEntry;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;

	TrakTablePtr = ^TrakTable;
	TrakTable = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		version: Fixed;
		format: UInt16;
		horizOffset: UInt16;
		vertOffset: UInt16;
<<<<<<< HEAD
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'kern' }
{ CONSTANTS }
const
	kKERNTag = $6B65726E; { 'kern' }
	kKERNCurrentVersion = $00010000;
	kKERNVertical = $8000; { set if this table has vertical kerning information }
	kKERNResetCrossStream = $8000; { this value in a cross-stream table means reset to zero }
	kKERNCrossStream = $4000; { set if this table contains cross-stream kerning values }
	kKERNVariation = $2000; { set if this table contains variation kerning values }
	kKERNUnusedBits = $1F00; { UNUSED, MUST BE ZERO }
	kKERNFormatMask = $00FF; { format of this subtable }

const
	kKERNOrderedList = 0;    { ordered list of kerning pairs }
	kKERNStateTable = 1;    { state table for n-way contextual kerning }
	kKERNSimpleArray = 2;    { simple n X m array of kerning values }
	kKERNIndexArray = 3;     { modifed version of SimpleArray }

{ Message Type Flags }
const
	kKERNLineStart = $00000001; { Array of glyphs starts a line }
	kKERNLineEndKerning = $00000002; { Array of glyphs ends a line }
	kKERNNoCrossKerning = $00000004; { Prohibit cross kerning }
	kKERNNotesRequested = $00000008; { Caller wants kerning notes }
	kKERNNoStakeNote = 1;    { Indicates a glyph was involved in a kerning pair/group }
	kKERNCrossStreamResetNote = 2;    { Indicates a return-to-baseline in cross-stream kerning }
	kKERNNotApplied = $00000001; { All kerning values were zero, kerning call had no effect }

{ TYPES }

type
	KernTableFormat = UInt8;
	KernSubtableInfo = UInt16;
	KernKerningValue = SInt16;
	KernArrayOffset = UInt16;
{ header for version 0 kerning table }

	KernVersion0HeaderPtr = ^KernVersion0Header;
	KernVersion0Header = record
		version: UInt16;                { font version number (will be 0!) }
		nTables: UInt16;                { number of subtables present }
		firstSubtable: array[0..0] of UInt16;       { first subtable starts here }
	end;

	KernTableHeaderPtr = ^KernTableHeader;
	KernTableHeader = record
		version: Fixed;                { font version number (currently 1.0) }
		nTables: SInt32;                { number of subtables present }
		firstSubtable: array[0..0] of UInt16;       { first subtable starts here }
	end;
	KernTableHeaderHandle = ^KernTableHeaderPtr;
{
    F O R M A T   S P E C I F I C   D E F I N I T I O N S

    kernOrderedList:
    
    The table is a sorted list of [left glyph, right glyph, value] triples.
    There's enough information in the header so that the list can be
    efficiently binary searched. 
}
{ defines a single kerning pair of Glyphcodes  }
	KernKerningPairPtr = ^KernKerningPair;
	KernKerningPair = record
		left: UInt16;
		right: UInt16;
	end;
{ a single list entry }

	KernOrderedListEntryPtr = ^KernOrderedListEntry;
	KernOrderedListEntry = record
		pair: KernKerningPair;                   { the kerning pair }
		value: KernKerningValue;                  { the kerning value for the above pair }
	end;

{ the header information for binary searching the list }
	KernOrderedListHeaderPtr = ^KernOrderedListHeader;
	KernOrderedListHeader = record
		nPairs: UInt16;                 { number of kerning pairs in table }
		searchRange: UInt16;            { (largest power of two <= nPairs) * entry size }
		entrySelector: UInt16;          { log2 (largest power of two <= nPairs) }
		rangeShift: UInt16;             { (nPairs - largest power of two <= nPairs) * entry size }
		table: array[0..0] of UInt16;               { entries are first glyph, second glyph, and value }
	end;
{ KernStateTable: like the the generic state tables }
	KernStateHeaderPtr = ^KernStateHeader;
	KernStateHeader = record
		header: STHeader;                 { state table header }
		valueTable: UInt16;             { offset to kerning value table }
		firstTable: array[0..0] of UInt8;          { first table starts here }
=======
=======
>>>>>>> origin/fixes_2_2
		version:				Fixed;
		format:					UInt16;
		horizOffset:			UInt16;
		vertOffset:				UInt16;
=======
>>>>>>> origin/fixes_2.4
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'kern' }
{ CONSTANTS }
const
	kKERNTag = $6B65726E; { 'kern' }
	kKERNCurrentVersion = $00010000;
	kKERNVertical = $8000; { set if this table has vertical kerning information }
	kKERNResetCrossStream = $8000; { this value in a cross-stream table means reset to zero }
	kKERNCrossStream = $4000; { set if this table contains cross-stream kerning values }
	kKERNVariation = $2000; { set if this table contains variation kerning values }
	kKERNUnusedBits = $1F00; { UNUSED, MUST BE ZERO }
	kKERNFormatMask = $00FF; { format of this subtable }

const
	kKERNOrderedList = 0;    { ordered list of kerning pairs }
	kKERNStateTable = 1;    { state table for n-way contextual kerning }
	kKERNSimpleArray = 2;    { simple n X m array of kerning values }
	kKERNIndexArray = 3;     { modifed version of SimpleArray }

{ Message Type Flags }
const
	kKERNLineStart = $00000001; { Array of glyphs starts a line }
	kKERNLineEndKerning = $00000002; { Array of glyphs ends a line }
	kKERNNoCrossKerning = $00000004; { Prohibit cross kerning }
	kKERNNotesRequested = $00000008; { Caller wants kerning notes }
	kKERNNoStakeNote = 1;    { Indicates a glyph was involved in a kerning pair/group }
	kKERNCrossStreamResetNote = 2;    { Indicates a return-to-baseline in cross-stream kerning }
	kKERNNotApplied = $00000001; { All kerning values were zero, kerning call had no effect }

{ TYPES }

type
	KernTableFormat = UInt8;
	KernSubtableInfo = UInt16;
	KernKerningValue = SInt16;
	KernArrayOffset = UInt16;
{ header for version 0 kerning table }

	KernVersion0HeaderPtr = ^KernVersion0Header;
	KernVersion0Header = record
		version: UInt16;                { font version number (will be 0!) }
		nTables: UInt16;                { number of subtables present }
		firstSubtable: array[0..0] of UInt16;       { first subtable starts here }
	end;

	KernTableHeaderPtr = ^KernTableHeader;
	KernTableHeader = record
		version: Fixed;                { font version number (currently 1.0) }
		nTables: SInt32;                { number of subtables present }
		firstSubtable: array[0..0] of UInt16;       { first subtable starts here }
	end;
	KernTableHeaderHandle = ^KernTableHeaderPtr;
{
    F O R M A T   S P E C I F I C   D E F I N I T I O N S

    kernOrderedList:
    
    The table is a sorted list of [left glyph, right glyph, value] triples.
    There's enough information in the header so that the list can be
    efficiently binary searched. 
}
{ defines a single kerning pair of Glyphcodes  }
	KernKerningPairPtr = ^KernKerningPair;
	KernKerningPair = record
		left: UInt16;
		right: UInt16;
	end;
{ a single list entry }

	KernOrderedListEntryPtr = ^KernOrderedListEntry;
	KernOrderedListEntry = record
		pair: KernKerningPair;                   { the kerning pair }
		value: KernKerningValue;                  { the kerning value for the above pair }
	end;

{ the header information for binary searching the list }
	KernOrderedListHeaderPtr = ^KernOrderedListHeader;
	KernOrderedListHeader = record
		nPairs: UInt16;                 { number of kerning pairs in table }
		searchRange: UInt16;            { (largest power of two <= nPairs) * entry size }
		entrySelector: UInt16;          { log2 (largest power of two <= nPairs) }
		rangeShift: UInt16;             { (nPairs - largest power of two <= nPairs) * entry size }
		table: array[0..0] of UInt16;               { entries are first glyph, second glyph, and value }
	end;
{ KernStateTable: like the the generic state tables }
	KernStateHeaderPtr = ^KernStateHeader;
	KernStateHeader = record
<<<<<<< HEAD
		header:					STHeader;								{  state table header  }
		valueTable:				UInt16;									{  offset to kerning value table  }
		firstTable:				SInt8;									{  first table starts here  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		header: STHeader;                 { state table header }
		valueTable: UInt16;             { offset to kerning value table }
		firstTable: array[0..0] of UInt8;          { first table starts here }
>>>>>>> origin/fixes_2.4
	end;

	KernStateEntryPtr = ^KernStateEntry;
	KernStateEntry = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		newState: UInt16;
		flags: UInt16;                  { flags per above enum }
	end;
{
    Kern offset table header.
    The offset table is a trimmed array from firstGlyph to limitGlyph.
    Glyphs outside of this range should get zero for right-hand glyphs
    and the offset of the beginning of the kerning array for left-hand glyphs.
}

	KernOffsetTable = record
		firstGlyph: UInt16;             { first glyph in class range }
		nGlyphs: UInt16;                { number of glyphs in class range }
		offsetTable: array[0..0] of KernArrayOffset;         { offset table starts here }
	end;
	KernOffsetTablePtr = ^KernOffsetTable;
{ Header information for accessing offset tables and kerning array }
{
    KernSimpleArray:
    
    The array is an nXm array of kenring values. Each row in the array
    represents one left-hand glyph, and each column one right-hand glyph.
    The zeroth row and column always represent glyphs that are out of bounds
    and will always contain zero.
    
    A pair is looked up by indexing the left-hand glyph through the left
    offset table, the right-hand glyph through the right offset table,
    adding both offsets to the starting address of the kerning array,
    and fetching the kerning value pointed to.
}
{ Kern offset table header. }
{ The offset table is a trimmed array from firstGlyph to limitGlyph. }
{ Glyphs outside of this range should get zero for right-hand glyphs }
{ and the offset of the beginning of the kerning array for left- }
{ hand glyphs. }
type
	KernSimpleArrayHeaderPtr = ^KernSimpleArrayHeader;
	KernSimpleArrayHeader = record
		rowWidth: UInt16;               { width, in bytes, of a row in the table }
		leftOffsetTable: UInt16;        { offset to left-hand offset table }
		rightOffsetTable: UInt16;       { offset to right-hand offset table }
		theArray: KernArrayOffset;               { offset to start of kerning array }
		firstTable: array[0..0] of UInt16;          { first offset table starts here... }
	end;
{ Index Array }
	
	KernIndexArrayHeaderPtr = ^KernIndexArrayHeader;
	KernIndexArrayHeader = record
		glyphCount: UInt16;
		kernValueCount: UInt8;
		leftClassCount: UInt8;
		rightClassCount: UInt8;
		flags: UInt8;                  { set to 0 for now }
		kernValue: array[0..0] of SInt16;           { actual kerning values reference by index in kernIndex }
		leftClass: array[0..0] of UInt8;           { maps left glyph to offset into kern index }
		rightClass: array[0..0] of UInt8;          { maps right glyph to offset into kern index }
		kernIndex: array[0..0] of UInt8;           { contains indicies into kernValue }
	end;
{ format specific part of subtable header }
=======
=======
>>>>>>> origin/fixes_2_2
		newState:				UInt16;
		flags:					UInt16;									{  flags per above enum  }
=======
		newState: UInt16;
		flags: UInt16;                  { flags per above enum }
>>>>>>> origin/fixes_2.4
	end;
{
    Kern offset table header.
    The offset table is a trimmed array from firstGlyph to limitGlyph.
    Glyphs outside of this range should get zero for right-hand glyphs
    and the offset of the beginning of the kerning array for left-hand glyphs.
}

	KernOffsetTable = record
		firstGlyph: UInt16;             { first glyph in class range }
		nGlyphs: UInt16;                { number of glyphs in class range }
		offsetTable: array[0..0] of KernArrayOffset;         { offset table starts here }
	end;
	KernOffsetTablePtr = ^KernOffsetTable;
{ Header information for accessing offset tables and kerning array }
{
    KernSimpleArray:
    
    The array is an nXm array of kenring values. Each row in the array
    represents one left-hand glyph, and each column one right-hand glyph.
    The zeroth row and column always represent glyphs that are out of bounds
    and will always contain zero.
    
    A pair is looked up by indexing the left-hand glyph through the left
    offset table, the right-hand glyph through the right offset table,
    adding both offsets to the starting address of the kerning array,
    and fetching the kerning value pointed to.
}
{ Kern offset table header. }
{ The offset table is a trimmed array from firstGlyph to limitGlyph. }
{ Glyphs outside of this range should get zero for right-hand glyphs }
{ and the offset of the beginning of the kerning array for left- }
{ hand glyphs. }
type
	KernSimpleArrayHeaderPtr = ^KernSimpleArrayHeader;
	KernSimpleArrayHeader = record
		rowWidth: UInt16;               { width, in bytes, of a row in the table }
		leftOffsetTable: UInt16;        { offset to left-hand offset table }
		rightOffsetTable: UInt16;       { offset to right-hand offset table }
		theArray: KernArrayOffset;               { offset to start of kerning array }
		firstTable: array[0..0] of UInt16;          { first offset table starts here... }
	end;
{ Index Array }
	
	KernIndexArrayHeaderPtr = ^KernIndexArrayHeader;
	KernIndexArrayHeader = record
<<<<<<< HEAD
		glyphCount:				UInt16;
		kernValueCount:			SInt8;
		leftClassCount:			SInt8;
		rightClassCount:		SInt8;
		flags:					SInt8;									{  set to 0 for now  }
		kernValue:				array [0..0] of SInt16;					{  actual kerning values reference by index in kernIndex  }
		leftClass:				SInt8;									{  maps left glyph to offset into kern index  }
		rightClass:				SInt8;									{  maps right glyph to offset into kern index  }
		kernIndex:				SInt8;									{  contains indicies into kernValue  }
	end;

	{	 format specific part of subtable header 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		glyphCount: UInt16;
		kernValueCount: UInt8;
		leftClassCount: UInt8;
		rightClassCount: UInt8;
		flags: UInt8;                  { set to 0 for now }
		kernValue: array[0..0] of SInt16;           { actual kerning values reference by index in kernIndex }
		leftClass: array[0..0] of UInt8;           { maps left glyph to offset into kern index }
		rightClass: array[0..0] of UInt8;          { maps right glyph to offset into kern index }
		kernIndex: array[0..0] of UInt8;           { contains indicies into kernValue }
	end;
{ format specific part of subtable header }
>>>>>>> origin/fixes_2.4
	KernFormatSpecificHeaderPtr = ^KernFormatSpecificHeader;
	KernFormatSpecificHeader = record
		case SInt16 of
		0: (
			orderedList:		KernOrderedListHeader;
			);
		1: (
			stateTable:			KernStateHeader;
			);
		2: (
			simpleArray:		KernSimpleArrayHeader;
			);
		3: (
			indexArray:			KernIndexArrayHeader;
			);
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ version 0 subtable header }

	KernVersion0SubtableHeaderPtr = ^KernVersion0SubtableHeader;
	KernVersion0SubtableHeader = record
		version: UInt16;                { kerning table version number }
		length: UInt16;                 { length in bytes (including this header) }
		stInfo: KernSubtableInfo;                 { sub-table info }
		fsHeader: KernFormatSpecificHeader;         { format specific sub-header }
	end;
{ Overall Subtable header format }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	KernSubtableHeaderPtr = ^KernSubtableHeader;
>>>>>>> graemeg/cpstrnew
	KernSubtableHeader = record
		length: SInt32;                 { length in bytes (including this header) }
		stInfo: KernSubtableInfo;                 { subtable info }
		tupleIndex: SInt16;             { tuple index for variation subtables }
		fsHeader: KernFormatSpecificHeader;         { format specific sub-header }
	end;
<<<<<<< HEAD
	KernSubtableHeaderPtr = ^KernSubtableHeader;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'kerx' }
{ CONSTANTS }
const
	kKERXTag = $6B657278; { 'kerx' }
	kKERXCurrentVersion = $00020000;
	kKERXVertical = $80000000; { set if this table has vertical kerning information }
	kKERXResetCrossStream = $8000; { this value in a cross-stream table means reset to zero }
	kKERXCrossStream = $40000000; { set if this table contains cross-stream kerning values }
	kKERXVariation = $20000000; { set if this table contains variation kerning values }
	kKERXUnusedBits = $1FFFFF00; { UNUSED, MUST BE ZERO }
	kKERXFormatMask = $000000FF; { format of this subtable }

<<<<<<< HEAD
const
	kKERXOrderedList = 0;    { ordered list of kerning pairs }
	kKERXStateTable = 1;    { state table for n-way contextual kerning }
	kKERXSimpleArray = 2;    { simple n X m array of kerning values }
	kKERXIndexArray = 3;    { modifed version of SimpleArray }
	kKERXControlPoint = 4;     { state table for control point positioning }

<<<<<<< HEAD
{ Message Type Flags }
const
	kKERXLineStart = $00000001; { Array of glyphs starts a line }
	kKERXLineEndKerning = $00000002; { Array of glyphs ends a line }
	kKERXNoCrossKerning = $00000004; { Prohibit cross kerning }
	kKERXNotesRequested = $00000008; { Caller wants kerning notes }
	kKERXNoStakeNote = 1;    { Indicates a glyph was involved in a kerning pair/group }
	kKERXCrossStreamResetNote = 2;    { Indicates a return-to-baseline in cross-stream kerning }
	kKERXNotApplied = $00000001; { All kerning values were zero, kerning call had no effect }

{ Flags in KerxControlPointHeader }
const
	kKERXActionTypeMask = 3 shl 30; { Mask to extract action type }
	kKERXActionTypeControlPoints = 0 shl 30; { Actions have control point numbers }
	kKERXActionTypeAnchorPoints = 1 shl 30; { Actions have anchor point numbers }
	kKERXActionTypeCoordinates = 2 shl 30; { Actions have control point coordinates }
	kKERXUnusedFlags = $3F000000; { Unused, must be zero }
	kKERXActionOffsetMask = $00FFFFFF; { Mask to extract offset to action table }

{ TYPES }
type
	KerxSubtableCoverage = UInt32;
	KerxArrayOffset = UInt32;
{ Header for an extended kerning table }
type
	KerxTableHeader = record
		version: Fixed;                { font version number (currently 1.0) }
		nTables: UInt32;                { number of subtables present }
		firstSubtable: array [0..1-1] of UInt32;       { first subtable starts here }
	end;
	KerxTableHeaderPtr = ^KerxTableHeader;
type
	KerxTableHeaderHandle = ^KerxTableHeaderPtr;
{
 F O R M A T   S P E C I F I C   D E F I N I T I O N S
 
 kerxOrderedList:
 
 The table is a sorted list of [left glyph, right glyph, value] triples.
 There's enough information in the header so that the list can be
 efficiently binary searched. 
 }
{ defines a single kerning pair of Glyphcodes  }
type
	KerxKerningPair = record
		left: UInt16;
		right: UInt16;
	end;
{ a single list entry }
type
	KerxOrderedListEntry = record
		pair: KerxKerningPair;                   { the kerning pair }
		value: KernKerningValue;                  { the kerning value for the above pair }
	end;
	KerxOrderedListEntryPtr = ^KerxOrderedListEntry;
{ the header information for binary searching the list }
type
	KerxOrderedListHeader = record
		nPairs: UInt32;                 { number of kerning pairs in table }
		searchRange: UInt32;            { (largest power of two <= nPairs) * entry size }
		entrySelector: UInt32;          { log2 (largest power of two <= nPairs) }
		rangeShift: UInt32;             { (nPairs - largest power of two <= nPairs) * entry size }
		table: array [0..1-1] of UInt32;               { entries are first glyph, second glyph, and value }
	end;
{ KernStateTable: like the the generic state tables }
type
	KerxStateHeader = record
		header: STXHeader;                 { state table header }
		valueTable: UInt32;             { offset to kerning value table }
		firstTable: array [0..1-1] of UInt8;          { first table starts here }
	end;
type
	KerxStateEntry = record
		newState: UInt16;
		flags: UInt16;                  { flags per above enum }
		valueIndex: UInt16;
	end;
{ KerxControlPointTable: like the the generic state tables }
type
	KerxControlPointHeader = record
		header: STXHeader;                 { state table header }
		flags: UInt32;                  { flags }
		firstTable: array [0..1-1] of UInt8;          { first table starts here }
	end;
type
	KerxControlPointEntry = record
		newState: UInt16;
		flags: UInt16;                  { flags per above enum }
		actionIndex: UInt16;
	end;
type
	KerxControlPointAction = record
		markControlPoint: UInt16;
		currControlPoint: UInt16;
	end;
type
	KerxAnchorPointAction = record
		markAnchorPoint: UInt16;
		currAnchorPoint: UInt16;
	end;
type
	KerxCoordinateAction = record
		markX: UInt16;
		markY: UInt16;
		currX: UInt16;
		currY: UInt16;
	end;
{
 Kern offset table header.
 The offset table is a trimmed array from firstGlyph to limitGlyph.
 Glyphs outside of this range should get zero for right-hand glyphs
 and the offset of the beginning of the kerning array for left-hand glyphs.
 }
type
	KerxOffsetTable = record
		firstGlyph: UInt16;             { first glyph in class range }
		nGlyphs: UInt16;                { number of glyphs in class range }
		offsetTable: array [0..1-1] of KerxArrayOffset;         { offset table starts here }
	end;
	KerxOffsetTablePtr = ^KerxOffsetTable;
{ Header information for accessing offset tables and kerning array }
{
 KerxSimpleArray:
 
 The array is an nXm array of kenring values. Each row in the array
 represents one left-hand glyph, and each column one right-hand glyph.
 The zeroth row and column always represent glyphs that are out of bounds
 and will always contain zero.
 
 A pair is looked up by indexing the left-hand glyph through the left
 offset table, the right-hand glyph through the right offset table,
 adding both offsets to the starting address of the kerning array,
 and fetching the kerning value pointed to.
 }
{ Kern offset table header. }
{ The offset table is a trimmed array from firstGlyph to limitGlyph. }
{ Glyphs outside of this range should get zero for right-hand glyphs }
{ and the offset of the beginning of the kerning array for left- }
{ hand glyphs. }
type
	KerxSimpleArrayHeader = record
		rowWidth: UInt32;               { width, in bytes, of a row in the table }
		leftOffsetTable: UInt32;        { offset to left-hand offset table }
		rightOffsetTable: UInt32;       { offset to right-hand offset table }
		theArray: KerxArrayOffset;               { offset to start of kerning array }
		firstTable: array [0..1-1] of UInt32;          { first offset table starts here... }
	end;
{ Index Array }
type
	KerxIndexArrayHeader = record
		glyphCount: UInt16;
		kernValueCount: UInt16;
		leftClassCount: UInt16;
		rightClassCount: UInt16;
		flags: UInt16;                  { set to 0 for now }
		kernValue: array [0..1-1] of SInt16;           { actual kerning values reference by index in kernIndex }
		leftClass: array [0..1-1] of UInt16;           { maps left glyph to offset into kern index }
		rightClass: array [0..1-1] of UInt16;          { maps right glyph to offset into kern index }
		kernIndex: array [0..1-1] of UInt16;           { contains indicies into kernValue }
	end;
{ format specific part of subtable header }
type
  KerxFormatSpecificHeader = record
    case SInt16 of
      0: (
        orderedList: KerxOrderedListHeader;
        );
      1: (
        stateTable: KerxStateHeader;
        );
      2: (
        simpleArray: KerxSimpleArrayHeader;
        );
      3: (
        indexArray: KerxIndexArrayHeader;
        );
      4: (
        controlPoint: KerxControlPointHeader;
        );
  end;
  KerxFormatSpecificHeaderPtr = ^KerxFormatSpecificHeader;

{ Overall Subtable header format }
type
	KerxSubtableHeader = record
		length: UInt32;                 { length in bytes (including this header) }
		stInfo: KerxSubtableCoverage;               { subtable converage }
		tupleIndex: UInt32;             { tuple index for variation subtables }
		fsHeader: KerxFormatSpecificHeader;         { format specific sub-header }
	end;
	KerxSubtableHeaderPtr = ^KerxSubtableHeader;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'bsln' }
{ CONSTANTS }
const
=======
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'bsln' }
{ CONSTANTS }
const
>>>>>>> graemeg/cpstrnew
=======
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'bsln' }
{ CONSTANTS }
const
>>>>>>> graemeg/cpstrnew
=======

{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'bsln' }
{ CONSTANTS }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kBSLNTag = $62736C6E; { 'bsln' }
	kBSLNCurrentVersion = $00010000; { current version number for 'bsln' table }
	kBSLNDistanceFormatNoMap = 0;
	kBSLNDistanceFormatWithMap = 1;
	kBSLNControlPointFormatNoMap = 2;
	kBSLNControlPointFormatWithMap = 3;

{ Baseline classes and constants }
const
	kBSLNRomanBaseline = 0;
	kBSLNIdeographicCenterBaseline = 1;
	kBSLNIdeographicLowBaseline = 2;
	kBSLNHangingBaseline = 3;
	kBSLNMathBaseline = 4;
	kBSLNLastBaseline = 31;
	kBSLNNumBaselineClasses = kBSLNLastBaseline + 1;
	kBSLNNoBaselineOverride = 255;

{ TYPES }
type
	BslnBaselineClass = UInt32;
	BslnBaselineClassPtr = ^BslnBaselineClass;
	
{ The BslnBaselineRecord array defines the baseline deltas for the line. }
	BslnBaselineRecord = array[0..31] of Fixed;
	BslnBaselineRecordPtr = ^BslnBaselineRecord;
{
    BslnFormat0Part is the format-specific data for a distance table with no mapping (i.e.
    all the glyphs belong to the defaultBaseline).
}

	BslnFormat0PartPtr = ^BslnFormat0Part;
	BslnFormat0Part = record
		deltas: array[0..31] of SInt16;
	end;
{ BslnFormat1Part is the format-specific data for a distance table with a gxMapping. }
	BslnFormat1PartPtr = ^BslnFormat1Part;
	BslnFormat1Part = record
		deltas: array[0..31] of SInt16;
		mappingData: SFNTLookupTable;
	end;
{
    BslnFormat2Part is the format-specific data for a control-point table with no
    mapping (i.e. all the glyphs belong to the defaultBaseline). It specifies a single
    glyph to use and the set of control points in that glyph that designate each of
    the baselines.
}
	BslnFormat2PartPtr = ^BslnFormat2Part;
	BslnFormat2Part = record
		stdGlyph: UInt16;
		ctlPoints: array[0..31] of SInt16;
	end;
{
    BslnFormat3Part is the format-specific data for a distance table with a mapping. Like
    format 2, it contains a single glyph and its set of control-point values for each
    of the baselines.
}

	BslnFormat3PartPtr = ^BslnFormat3Part;
	BslnFormat3Part = record
		stdGlyph: UInt16;
		ctlPoints: array[0..31] of SInt16;
		mappingData: SFNTLookupTable;
	end;
{ The BslnFormatUnion is a union containing the format-specific parts of the baseline table. }
=======
=======
>>>>>>> origin/fixes_2_2
=======
{ version 0 subtable header }
>>>>>>> origin/fixes_2.4

	KernVersion0SubtableHeaderPtr = ^KernVersion0SubtableHeader;
	KernVersion0SubtableHeader = record
		version: UInt16;                { kerning table version number }
		length: UInt16;                 { length in bytes (including this header) }
		stInfo: KernSubtableInfo;                 { sub-table info }
		fsHeader: KernFormatSpecificHeader;         { format specific sub-header }
	end;
{ Overall Subtable header format }
	KernSubtableHeaderPtr = ^KernSubtableHeader;
	KernSubtableHeader = record
		length: SInt32;                 { length in bytes (including this header) }
		stInfo: KernSubtableInfo;                 { subtable info }
		tupleIndex: SInt16;             { tuple index for variation subtables }
		fsHeader: KernFormatSpecificHeader;         { format specific sub-header }
	end;

{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'bsln' }
{ CONSTANTS }
const
	kBSLNTag = $62736C6E; { 'bsln' }
	kBSLNCurrentVersion = $00010000; { current version number for 'bsln' table }
	kBSLNDistanceFormatNoMap = 0;
	kBSLNDistanceFormatWithMap = 1;
	kBSLNControlPointFormatNoMap = 2;
	kBSLNControlPointFormatWithMap = 3;

{ Baseline classes and constants }
const
	kBSLNRomanBaseline = 0;
	kBSLNIdeographicCenterBaseline = 1;
	kBSLNIdeographicLowBaseline = 2;
	kBSLNHangingBaseline = 3;
	kBSLNMathBaseline = 4;
	kBSLNLastBaseline = 31;
	kBSLNNumBaselineClasses = kBSLNLastBaseline + 1;
	kBSLNNoBaselineOverride = 255;

{ TYPES }
type
	BslnBaselineClass = UInt32;
	BslnBaselineClassPtr = ^BslnBaselineClass;
	
{ The BslnBaselineRecord array defines the baseline deltas for the line. }
	BslnBaselineRecord = array[0..31] of Fixed;
	BslnBaselineRecordPtr = ^BslnBaselineRecord;
{
    BslnFormat0Part is the format-specific data for a distance table with no mapping (i.e.
    all the glyphs belong to the defaultBaseline).
}

	BslnFormat0PartPtr = ^BslnFormat0Part;
	BslnFormat0Part = record
		deltas: array[0..31] of SInt16;
	end;
{ BslnFormat1Part is the format-specific data for a distance table with a gxMapping. }
	BslnFormat1PartPtr = ^BslnFormat1Part;
	BslnFormat1Part = record
		deltas: array[0..31] of SInt16;
		mappingData: SFNTLookupTable;
	end;
{
    BslnFormat2Part is the format-specific data for a control-point table with no
    mapping (i.e. all the glyphs belong to the defaultBaseline). It specifies a single
    glyph to use and the set of control points in that glyph that designate each of
    the baselines.
}
	BslnFormat2PartPtr = ^BslnFormat2Part;
	BslnFormat2Part = record
		stdGlyph: UInt16;
		ctlPoints: array[0..31] of SInt16;
	end;
{
    BslnFormat3Part is the format-specific data for a distance table with a mapping. Like
    format 2, it contains a single glyph and its set of control-point values for each
    of the baselines.
}

	BslnFormat3PartPtr = ^BslnFormat3Part;
	BslnFormat3Part = record
		stdGlyph: UInt16;
		ctlPoints: array[0..31] of SInt16;
		mappingData: SFNTLookupTable;
	end;
<<<<<<< HEAD

	{	 The BslnFormatUnion is a union containing the format-specific parts of the baseline table. 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ The BslnFormatUnion is a union containing the format-specific parts of the baseline table. }
>>>>>>> origin/fixes_2.4
	BslnFormatUnionPtr = ^BslnFormatUnion;
	BslnFormatUnion = record
		case SInt16 of
		0: (
			fmt0Part:			BslnFormat0Part;
			);
		1: (
			fmt1Part:			BslnFormat1Part;
			);
		2: (
			fmt2Part:			BslnFormat2Part;
			);
		3: (
			fmt3Part:			BslnFormat3Part;
			);
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ The table format used in BaselineTable }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	BslnTableFormat = UInt16;
{ BaselineTable defines the top-level format of the baseline table in the font. }

	BslnTablePtr = ^BslnTable;
	BslnTable = record
		version: Fixed;
		format: BslnTableFormat;
		defaultBaseline: UInt16;
		parts: BslnFormatUnion;
	end;

{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'ALMX' }
{ TYPES }
	ALMXHeaderPtr = ^ALMXHeader;
	ALMXHeader = record
		Version: Fixed;                { 16.16 format 1.0 by default }
		Flags: UInt16;
		NMasters: UInt16;
		FirstGlyph: UInt16;
		LastGlyph: UInt16;

		lookup: SFNTLookupTable;                 { lookup table }
	end;

	ALMXGlyphEntryPtr = ^ALMXGlyphEntry;
	ALMXGlyphEntry = record
{ lookup data for ALMX table }
		GlyphIndexOffset: SInt16;
		HorizontalAdvance: SInt16;
		XOffsetToHOrigin: SInt16;
		VerticalAdvance: SInt16;
		YOffsetToVOrigin: SInt16;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'ROTA' }
{ TYPES }
	ROTAHeaderPtr = ^ROTAHeader;
	ROTAHeader = record
		Version: Fixed;                { 16.16 format 1.0 by default }
		Flags: UInt16;
		NMasters: UInt16;
		FirstGlyph: UInt16;
		LastGlyph: UInt16;

		lookup: SFNTLookupTable;                 { lookup table }
	end;

	ROTAGlyphEntryPtr = ^ROTAGlyphEntry;
	ROTAGlyphEntry = record
{ lookup data for ROTA table }
		GlyphIndexOffset: SInt16;
		HBaselineOffset: SInt16;        { y offset to the rotated horizontal baseline }
		VBaselineOffset: SInt16;        { x offset to the rotated vertical baseline }
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE 'ankr' }
{ CONSTANTS }
const
	kANKRCurrentVersion = 0;

{ TYPES }
type
	AnchorPoint = record
		x: SInt16;                     { x coordinate of anchor point }
		y: SInt16;                     { y coordinate of anchor point }
	end;

type
	AnchorPointTable = record
		nPoints: UInt32;                { number of anchor points defined for this glyph }
		points: array [0..1-1] of AnchorPoint;              { first anchor point starts here }
	end;

type
	AnkrTable = record
		version: UInt16;                { 1 }
		flags: UInt16;                  { never leave home without them (see 'Zapf') }
		lookupTableOffset: UInt32;      { Byte offset to lookup table mapping glyphs to offset into anchor point table }
		anchorPointTableOffset: UInt32; { Byte offset to start of anchor point table }
	end;
{ --------------------------------------------------------------------------- }

=======

	BslnTableFormat = UInt16;
{ BaselineTable defines the top-level format of the baseline table in the font. }

=======

	BslnTableFormat = UInt16;
{ BaselineTable defines the top-level format of the baseline table in the font. }

>>>>>>> graemeg/cpstrnew
=======

	BslnTableFormat = UInt16;
{ BaselineTable defines the top-level format of the baseline table in the font. }

>>>>>>> graemeg/cpstrnew
=======

	BslnTableFormat = UInt16;
{ BaselineTable defines the top-level format of the baseline table in the font. }

>>>>>>> origin/cpstrnew
	BslnTablePtr = ^BslnTable;
	BslnTable = record
		version: Fixed;
		format: BslnTableFormat;
		defaultBaseline: UInt16;
		parts: BslnFormatUnion;
	end;

{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'ALMX' }
{ TYPES }
	ALMXHeaderPtr = ^ALMXHeader;
	ALMXHeader = record
		Version: Fixed;                { 16.16 format 1.0 by default }
		Flags: UInt16;
		NMasters: UInt16;
		FirstGlyph: UInt16;
		LastGlyph: UInt16;

		lookup: SFNTLookupTable;                 { lookup table }
	end;

	ALMXGlyphEntryPtr = ^ALMXGlyphEntry;
	ALMXGlyphEntry = record
{ lookup data for ALMX table }
		GlyphIndexOffset: SInt16;
		HorizontalAdvance: SInt16;
		XOffsetToHOrigin: SInt16;
		VerticalAdvance: SInt16;
		YOffsetToVOrigin: SInt16;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'ROTA' }
{ TYPES }
	ROTAHeaderPtr = ^ROTAHeader;
	ROTAHeader = record
		Version: Fixed;                { 16.16 format 1.0 by default }
		Flags: UInt16;
		NMasters: UInt16;
		FirstGlyph: UInt16;
		LastGlyph: UInt16;

		lookup: SFNTLookupTable;                 { lookup table }
	end;

	ROTAGlyphEntryPtr = ^ROTAGlyphEntry;
	ROTAGlyphEntry = record
{ lookup data for ROTA table }
		GlyphIndexOffset: SInt16;
		HBaselineOffset: SInt16;        { y offset to the rotated horizontal baseline }
		VBaselineOffset: SInt16;        { x offset to the rotated vertical baseline }
	end;
{ --------------------------------------------------------------------------- }

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
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	{	 The table format used in BaselineTable 	}
	BslnTableFormat						= UInt16;
	{	 BaselineTable defines the top-level format of the baseline table in the font. 	}
	BslnTablePtr = ^BslnTable;
	BslnTable = record
		version:				Fixed;
		format:					BslnTableFormat;
		defaultBaseline:		UInt16;
		parts:					BslnFormatUnion;
	end;
=======
{ The table format used in BaselineTable }
>>>>>>> origin/fixes_2.4

	BslnTableFormat = UInt16;
{ BaselineTable defines the top-level format of the baseline table in the font. }

	BslnTablePtr = ^BslnTable;
	BslnTable = record
		version: Fixed;
		format: BslnTableFormat;
		defaultBaseline: UInt16;
		parts: BslnFormatUnion;
	end;

{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'ALMX' }
{ TYPES }
	ALMXHeaderPtr = ^ALMXHeader;
	ALMXHeader = record
		Version: Fixed;                { 16.16 format 1.0 by default }
		Flags: UInt16;
		NMasters: UInt16;
		FirstGlyph: UInt16;
		LastGlyph: UInt16;

		lookup: SFNTLookupTable;                 { lookup table }
	end;

	ALMXGlyphEntryPtr = ^ALMXGlyphEntry;
	ALMXGlyphEntry = record
{ lookup data for ALMX table }
		GlyphIndexOffset: SInt16;
		HorizontalAdvance: SInt16;
		XOffsetToHOrigin: SInt16;
		VerticalAdvance: SInt16;
		YOffsetToVOrigin: SInt16;
	end;
{ --------------------------------------------------------------------------- }
{ FORMATS FOR TABLE: 'ROTA' }
{ TYPES }
	ROTAHeaderPtr = ^ROTAHeader;
	ROTAHeader = record
		Version: Fixed;                { 16.16 format 1.0 by default }
		Flags: UInt16;
		NMasters: UInt16;
		FirstGlyph: UInt16;
		LastGlyph: UInt16;

		lookup: SFNTLookupTable;                 { lookup table }
	end;

	ROTAGlyphEntryPtr = ^ROTAGlyphEntry;
	ROTAGlyphEntry = record
{ lookup data for ROTA table }
		GlyphIndexOffset: SInt16;
		HBaselineOffset: SInt16;        { y offset to the rotated horizontal baseline }
		VBaselineOffset: SInt16;        { x offset to the rotated vertical baseline }
	end;
{ --------------------------------------------------------------------------- }

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
