{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/Finder.h
 
     Contains:   Finder flags and container types.
                 The contents of this header file are deprecated.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1990-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1990-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       Finder.p
=======
     File:       CarbonCore/Finder.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Finder flags and container types.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1990-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1990-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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


=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/fixes_2.4
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

unit Finder;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
>>>>>>> origin/cpstrnew
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
>>>>>>> graemeg/cpstrnew
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
=======

>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}

{ Creator and type of clipping files }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const
	kClippingCreator = FourCharCode('drag');
	kClippingPictureType = FourCharCode('clpp');
	kClippingTextType = FourCharCode('clpt');
	kClippingSoundType = FourCharCode('clps');
	kClippingUnknownType = FourCharCode('clpu');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

{ Creator and type of Internet Location files }
const
	kInternetLocationCreator = FourCharCode('drag');
	kInternetLocationHTTP = FourCharCode('ilht');
	kInternetLocationFTP = FourCharCode('ilft');
	kInternetLocationFile = FourCharCode('ilfi');
	kInternetLocationMail = FourCharCode('ilma');
	kInternetLocationNNTP = FourCharCode('ilnw');
	kInternetLocationAFP = FourCharCode('ilaf');
	kInternetLocationAppleTalk = FourCharCode('ilat');
	kInternetLocationNSL = FourCharCode('ilns');
	kInternetLocationGeneric = FourCharCode('ilge');
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew

{ Creator and type of Internet Location files }
const
<<<<<<< HEAD
	kCustomIconResource = -16455; { Custom icon family resource ID }
=======
	kInternetLocationCreator = FourCharCode('drag');
	kInternetLocationHTTP = FourCharCode('ilht');
	kInternetLocationFTP = FourCharCode('ilft');
	kInternetLocationFile = FourCharCode('ilfi');
	kInternetLocationMail = FourCharCode('ilma');
	kInternetLocationNNTP = FourCharCode('ilnw');
	kInternetLocationAFP = FourCharCode('ilaf');
	kInternetLocationAppleTalk = FourCharCode('ilat');
	kInternetLocationNSL = FourCharCode('ilns');
	kInternetLocationGeneric = FourCharCode('ilge');
>>>>>>> graemeg/cpstrnew

{ In order to specify any of the information described in the }
{ CustomBadgeResource data structure you must clear the kExtendedFlagsAreInvalid }
{ and set kExtendedFlagHasCustomBadge of the FXInfo.fdXFlags or DXInfo.frXFlags field, }
{ and add a resource of type kCustomBadgeResourceType and ID kCustomBadgeResourceID to }
{ the file or to the "Icon/n" file for a folder }
const
	kCustomBadgeResourceType = FourCharCode('badg');
	kCustomBadgeResourceID = kCustomIconResource;
	kCustomBadgeResourceVersion = 0;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Creator and type of Internet Location files }
const
<<<<<<< HEAD
	kCustomIconResource = -16455; { Custom icon family resource ID }
=======
	kInternetLocationCreator = FourCharCode('drag');
	kInternetLocationHTTP = FourCharCode('ilht');
	kInternetLocationFTP = FourCharCode('ilft');
	kInternetLocationFile = FourCharCode('ilfi');
	kInternetLocationMail = FourCharCode('ilma');
	kInternetLocationNNTP = FourCharCode('ilnw');
	kInternetLocationAFP = FourCharCode('ilaf');
	kInternetLocationAppleTalk = FourCharCode('ilat');
	kInternetLocationNSL = FourCharCode('ilns');
	kInternetLocationGeneric = FourCharCode('ilge');
>>>>>>> graemeg/cpstrnew

{ In order to specify any of the information described in the }
{ CustomBadgeResource data structure you must clear the kExtendedFlagsAreInvalid }
{ and set kExtendedFlagHasCustomBadge of the FXInfo.fdXFlags or DXInfo.frXFlags field, }
{ and add a resource of type kCustomBadgeResourceType and ID kCustomBadgeResourceID to }
{ the file or to the "Icon/n" file for a folder }
const
	kCustomBadgeResourceType = FourCharCode('badg');
	kCustomBadgeResourceID = kCustomIconResource;
	kCustomBadgeResourceVersion = 0;
=======
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
{ Creator and type of Internet Location files }
const
<<<<<<< HEAD
	kCustomIconResource = -16455; { Custom icon family resource ID }
=======
	kInternetLocationCreator = FourCharCode('drag');
	kInternetLocationHTTP = FourCharCode('ilht');
	kInternetLocationFTP = FourCharCode('ilft');
	kInternetLocationFile = FourCharCode('ilfi');
	kInternetLocationMail = FourCharCode('ilma');
	kInternetLocationNNTP = FourCharCode('ilnw');
	kInternetLocationAFP = FourCharCode('ilaf');
	kInternetLocationAppleTalk = FourCharCode('ilat');
	kInternetLocationNSL = FourCharCode('ilns');
	kInternetLocationGeneric = FourCharCode('ilge');
>>>>>>> graemeg/cpstrnew
=======
const
	kCustomIconResource = -16455; { Custom icon family resource ID }
>>>>>>> origin/cpstrnew

{ In order to specify any of the information described in the }
{ CustomBadgeResource data structure you must clear the kExtendedFlagsAreInvalid }
{ and set kExtendedFlagHasCustomBadge of the FXInfo.fdXFlags or DXInfo.frXFlags field, }
{ and add a resource of type kCustomBadgeResourceType and ID kCustomBadgeResourceID to }
{ the file or to the "Icon/n" file for a folder }
const
	kCustomBadgeResourceType = FourCharCode('badg');
	kCustomBadgeResourceID = kCustomIconResource;
	kCustomBadgeResourceVersion = 0;

<<<<<<< HEAD
const
	kCustomIconResource = -16455; { Custom icon family resource ID }

{ In order to specify any of the information described in the }
{ CustomBadgeResource data structure you must clear the kExtendedFlagsAreInvalid }
{ and set kExtendedFlagHasCustomBadge of the FXInfo.fdXFlags or DXInfo.frXFlags field, }
{ and add a resource of type kCustomBadgeResourceType and ID kCustomBadgeResourceID to }
{ the file or to the "Icon/n" file for a folder }
const
	kCustomBadgeResourceType = FourCharCode('badg');
	kCustomBadgeResourceID = kCustomIconResource;
	kCustomBadgeResourceVersion = 0;

type
	CustomBadgeResource = record
		version: SInt16;                { This is version kCustomBadgeResourceVersion}
		customBadgeResourceID: SInt16;  { If not 0, the ID of a resource to use on top}
                                              { of the icon for this file or folder}
		customBadgeType: OSType;        { If not 0, the type and creator of an icon}
		customBadgeCreator: OSType;     { to use on top of the icon}
		windowBadgeType: OSType;        { If not 0, the type and creator of an icon}
		windowBadgeCreator: OSType;     { to display in the header of the window for this }
                                              { file or folder}
		overrideType: OSType;           { If not 0, the type and creator of an icon to}
		overrideCreator: OSType;        { use INSTEAD of the icon for this file or folder}
	end;
	CustomBadgeResourcePtr = ^CustomBadgeResource;
type
	CustomBadgeResourceHandle = ^CustomBadgeResourcePtr;
{ You can specify routing information for a file by including a 'rout' 0 
    resource in it and setting the kExtendedFlagHasRoutingInfo bit in the extended 
    Finder flags. 
    The 'rout' resource is an array of RoutingResourceEntry. Each entry is considered
    in turn. The first matching entry is used.
    If the creator and fileType match the file being dropped and targetFolder match
    the folder ID of the folder being dropped onto, then the file is rerouted 
    into the specified destination folder.
    The only target folder currently supported is the system folder, 
    kSystemFolderType = 'macs'.
}
const
=======
type
	CustomBadgeResource = record
		version: SInt16;                { This is version kCustomBadgeResourceVersion}
		customBadgeResourceID: SInt16;  { If not 0, the ID of a resource to use on top}
                                              { of the icon for this file or folder}
		customBadgeType: OSType;        { If not 0, the type and creator of an icon}
		customBadgeCreator: OSType;     { to use on top of the icon}
		windowBadgeType: OSType;        { If not 0, the type and creator of an icon}
		windowBadgeCreator: OSType;     { to display in the header of the window for this }
                                              { file or folder}
		overrideType: OSType;           { If not 0, the type and creator of an icon to}
		overrideCreator: OSType;        { use INSTEAD of the icon for this file or folder}
	end;
	CustomBadgeResourcePtr = ^CustomBadgeResource;
type
	CustomBadgeResourceHandle = ^CustomBadgeResourcePtr;
{ You can specify routing information for a file by including a 'rout' 0 
    resource in it and setting the kExtendedFlagHasRoutingInfo bit in the extended 
    Finder flags. 
    The 'rout' resource is an array of RoutingResourceEntry. Each entry is considered
    in turn. The first matching entry is used.
    If the creator and fileType match the file being dropped and targetFolder match
    the folder ID of the folder being dropped onto, then the file is rerouted 
    into the specified destination folder.
    The only target folder currently supported is the system folder, 
    kSystemFolderType = 'macs'.
}
const
>>>>>>> origin/cpstrnew
	kRoutingResourceType = FourCharCode('rout');
	kRoutingResourceID = 0;
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ Creator and type of Internet Location files }
const
	kInternetLocationCreator = FourCharCode('drag');
	kInternetLocationHTTP = FourCharCode('ilht');
	kInternetLocationFTP = FourCharCode('ilft');
	kInternetLocationFile = FourCharCode('ilfi');
	kInternetLocationMail = FourCharCode('ilma');
	kInternetLocationNNTP = FourCharCode('ilnw');
	kInternetLocationAFP = FourCharCode('ilaf');
	kInternetLocationAppleTalk = FourCharCode('ilat');
	kInternetLocationNSL = FourCharCode('ilns');
	kInternetLocationGeneric = FourCharCode('ilge');


const
	kCustomIconResource = -16455; { Custom icon family resource ID }

{ In order to specify any of the information described in the }
{ CustomBadgeResource data structure you must clear the kExtendedFlagsAreInvalid }
{ and set kExtendedFlagHasCustomBadge of the FXInfo.fdXFlags or DXInfo.frXFlags field, }
{ and add a resource of type kCustomBadgeResourceType and ID kCustomBadgeResourceID to }
{ the file or to the "Icon/n" file for a folder }
const
	kCustomBadgeResourceType = FourCharCode('badg');
	kCustomBadgeResourceID = kCustomIconResource;
	kCustomBadgeResourceVersion = 0;

type
	CustomBadgeResource = record
		version: SInt16;                { This is version kCustomBadgeResourceVersion}
		customBadgeResourceID: SInt16;  { If not 0, the ID of a resource to use on top}
                                              { of the icon for this file or folder}
		customBadgeType: OSType;        { If not 0, the type and creator of an icon}
		customBadgeCreator: OSType;     { to use on top of the icon}
		windowBadgeType: OSType;        { If not 0, the type and creator of an icon}
		windowBadgeCreator: OSType;     { to display in the header of the window for this }
                                              { file or folder}
		overrideType: OSType;           { If not 0, the type and creator of an icon to}
		overrideCreator: OSType;        { use INSTEAD of the icon for this file or folder}
	end;
	CustomBadgeResourcePtr = ^CustomBadgeResource;
type
	CustomBadgeResourceHandle = ^CustomBadgeResourcePtr;
{ You can specify routing information for a file by including a 'rout' 0 
    resource in it and setting the kExtendedFlagHasRoutingInfo bit in the extended 
    Finder flags. 
    The 'rout' resource is an array of RoutingResourceEntry. Each entry is considered
    in turn. The first matching entry is used.
    If the creator and fileType match the file being dropped and targetFolder match
    the folder ID of the folder being dropped onto, then the file is rerouted 
    into the specified destination folder.
    The only target folder currently supported is the system folder, 
    kSystemFolderType = 'macs'.
}
const
<<<<<<< HEAD
	kRoutingResourceType		= FourCharCode('rout');
	kRoutingResourceID			= 0;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kRoutingResourceType = FourCharCode('rout');
	kRoutingResourceID = 0;
>>>>>>> origin/fixes_2.4

type
	RoutingResourceEntryPtr = ^RoutingResourceEntry;
	RoutingResourceEntry = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		creator: OSType;                { Use '****' or 0 to match any creator }
		fileType: OSType;               { Use '****' or 0 to match any file type }
		targetFolder: OSType;           { Folder ID of the folder this file was dropped onto }
		destinationFolder: OSType;      { Folder that the source will be routed to }
		reservedField: OSType;          { Set to 0 }
<<<<<<< HEAD
	end;
type
	RoutingResourcePtr = RoutingResourceEntryPtr;
	RoutingResourceHandle = ^RoutingResourcePtr;

{ Types for special container aliases }
<<<<<<< HEAD
const
	kContainerFolderAliasType = FourCharCode('fdrp'); { type for folder aliases }
	kContainerTrashAliasType = FourCharCode('trsh'); { type for trash folder aliases }
	kContainerHardDiskAliasType = FourCharCode('hdsk'); { type for hard disk aliases }
	kContainerFloppyAliasType = FourCharCode('flpy'); { type for floppy aliases }
	kContainerServerAliasType = FourCharCode('srvr'); { type for server aliases }
	kApplicationAliasType = FourCharCode('adrp'); { type for application aliases }
	kContainerAliasType = FourCharCode('drop'); { type for all other containers }
	kDesktopPrinterAliasType = FourCharCode('dtpa'); { type for Desktop Printer alias }
	kContainerCDROMAliasType = FourCharCode('cddr'); { type for CD-ROM alias }
	kApplicationCPAliasType = FourCharCode('acdp'); { type for application control panel alias }
	kApplicationDAAliasType = FourCharCode('addp'); { type for application DA alias }
	kPackageAliasType = FourCharCode('fpka'); { type for plain package alias }
	kAppPackageAliasType = FourCharCode('fapa'); { type for application package alias }

{ Types for Special folder aliases }
const
=======
const
	kContainerFolderAliasType = FourCharCode('fdrp'); { type for folder aliases }
	kContainerTrashAliasType = FourCharCode('trsh'); { type for trash folder aliases }
	kContainerHardDiskAliasType = FourCharCode('hdsk'); { type for hard disk aliases }
	kContainerFloppyAliasType = FourCharCode('flpy'); { type for floppy aliases }
	kContainerServerAliasType = FourCharCode('srvr'); { type for server aliases }
	kApplicationAliasType = FourCharCode('adrp'); { type for application aliases }
	kContainerAliasType = FourCharCode('drop'); { type for all other containers }
	kDesktopPrinterAliasType = FourCharCode('dtpa'); { type for Desktop Printer alias }
	kContainerCDROMAliasType = FourCharCode('cddr'); { type for CD-ROM alias }
	kApplicationCPAliasType = FourCharCode('acdp'); { type for application control panel alias }
	kApplicationDAAliasType = FourCharCode('addp'); { type for application DA alias }
	kPackageAliasType = FourCharCode('fpka'); { type for plain package alias }
	kAppPackageAliasType = FourCharCode('fapa'); { type for application package alias }

{ Types for Special folder aliases }
const
>>>>>>> origin/cpstrnew
	kSystemFolderAliasType = FourCharCode('fasy');
	kAppleMenuFolderAliasType = FourCharCode('faam');
	kStartupFolderAliasType = FourCharCode('fast');
	kPrintMonitorDocsFolderAliasType = FourCharCode('fapn');
	kPreferencesFolderAliasType = FourCharCode('fapf');
	kControlPanelFolderAliasType = FourCharCode('fact');
	kExtensionFolderAliasType = FourCharCode('faex');

{ Types for AppleShare folder aliases }
const
	kExportedFolderAliasType = FourCharCode('faet');
	kDropFolderAliasType = FourCharCode('fadr');
	kSharedFolderAliasType = FourCharCode('fash');
	kMountedFolderAliasType = FourCharCode('famn');

{ Finder flags (finderFlags, fdFlags and frFlags) }
{ Any flag reserved or not specified should be set to 0. }
{ If a flag applies to a file, but not to a folder, make sure to check }
{ that the item is not a folder by checking ((ParamBlockRec.ioFlAttrib & ioDirMask) == 0) }
const
	kIsOnDesk = $0001; { Files and folders (System 6) }
	kColor = $000E; { Files and folders }
                                        { bit 0x0020 was kRequireSwitchLaunch, but is now reserved for future use}
	kIsShared = $0040; { Files only (Applications only) }
                                        { If clear, the application needs to write to }
                                        { its resource fork, and therefore cannot be }
                                        { shared on a server }
	kHasNoINITs = $0080; { Files only (Extensions/Control Panels only) }
                                        { This file contains no INIT resource }
	kHasBeenInited = $0100; { Files only }
                                        { Clear if the file contains desktop database }
                                        { resources ('BNDL', 'FREF', 'open', 'kind'...) }
                                        { that have not been added yet. Set only by the Finder }
                                        { Reserved for folders - make sure this bit is cleared for folders }
                                        { bit 0x0200 was the letter bit for AOCE, but is now reserved for future use }
	kHasCustomIcon = $0400; { Files and folders }
	kIsStationery = $0800; { Files only }
	kNameLocked = $1000; { Files and folders }
	kHasBundle = $2000; { Files and folders }
                                        { Indicates that a file has a BNDL resource }
                                        { Indicates that a folder is displayed as a package }
	kIsInvisible = $4000; { Files and folders }
	kIsAlias = $8000; { Files only }

{ Obsolete. Use names defined above. }
const
	fOnDesk = kIsOnDesk;
	fHasBundle = kHasBundle;
	fInvisible = kIsInvisible;

{ Obsolete }
const
	fTrash = -3;
	fDesktop = -2;
	fDisk = 0;

{$ifc OLDROUTINENAMES}
const
	kIsStationary = kIsStationery;

{$endc} {OLDROUTINENAMES}

{ Extended flags (extendedFinderFlags, fdXFlags and frXFlags) }
{ Any flag not specified should be set to 0. }
const
	kExtendedFlagsAreInvalid = $8000; { If set the other extended flags are ignored }
	kExtendedFlagHasCustomBadge = $0100; { Set if the file or folder has a badge resource }
	kExtendedFlagObjectIsBusy = $0080; { Set if the object is marked as busy/incomplete }
	kExtendedFlagHasRoutingInfo = $0004; { Set if the file contains routing info resource }


{ Use a filetype in this range to indicate that a file is temporarily busy }
{ (while it is being downloaded or installed, for example).  This prevents }
{ Finder 8.5 and later from trying to change the item's attributes before it }
{ is fully created. -- If you provide a series of 'BNDL' icons for your creator }
{ and some of these filetypes, you can achieve limited icon animation while }
{ the file creation progresses. }
const
	kFirstMagicBusyFiletype = FourCharCode('bzy ');
	kLastMagicBusyFiletype = FourCharCode('bzy?');

{ Use this date as a file's or folder's creation date to indicate that it is }
{ temporarily busy (while it is being downloaded or installed, for example). }
{ This prevents Finder from trying to change the item's attributes before it }
{ is fully created (Finder 8.5 and 8.6 check file creation dates; later Finders }
{ may check folder creation dates as well). }
const
	kMagicBusyCreationDate = $4F3AFDB0;


{------------------------------------------------------------------------}
{
   The following data structures are binary compatible with FInfo, DInfo,
   FXInfo and DXInfo but represent the Mac OS 8 semantic of the fields.
   Use these data structures preferably to FInfo, etc...
}
{------------------------------------------------------------------------}
=======
=======
>>>>>>> origin/fixes_2_2
		creator:				OSType;									{  Use '****' or 0 to match any creator  }
		fileType:				OSType;									{  Use '****' or 0 to match any file type  }
		targetFolder:			OSType;									{  Folder ID of the folder this file was dropped onto  }
		destinationFolder:		OSType;									{  Folder that the source will be routed to  }
		reservedField:			OSType;									{  Set to 0  }
=======
>>>>>>> origin/fixes_2.4
	end;
type
	RoutingResourcePtr = RoutingResourceEntryPtr;
	RoutingResourceHandle = ^RoutingResourcePtr;

{ Types for special container aliases }
const
	kContainerFolderAliasType = FourCharCode('fdrp'); { type for folder aliases }
	kContainerTrashAliasType = FourCharCode('trsh'); { type for trash folder aliases }
	kContainerHardDiskAliasType = FourCharCode('hdsk'); { type for hard disk aliases }
	kContainerFloppyAliasType = FourCharCode('flpy'); { type for floppy aliases }
	kContainerServerAliasType = FourCharCode('srvr'); { type for server aliases }
	kApplicationAliasType = FourCharCode('adrp'); { type for application aliases }
	kContainerAliasType = FourCharCode('drop'); { type for all other containers }
	kDesktopPrinterAliasType = FourCharCode('dtpa'); { type for Desktop Printer alias }
	kContainerCDROMAliasType = FourCharCode('cddr'); { type for CD-ROM alias }
	kApplicationCPAliasType = FourCharCode('acdp'); { type for application control panel alias }
	kApplicationDAAliasType = FourCharCode('addp'); { type for application DA alias }
	kPackageAliasType = FourCharCode('fpka'); { type for plain package alias }
	kAppPackageAliasType = FourCharCode('fapa'); { type for application package alias }

{ Types for Special folder aliases }
const
	kSystemFolderAliasType = FourCharCode('fasy');
	kAppleMenuFolderAliasType = FourCharCode('faam');
	kStartupFolderAliasType = FourCharCode('fast');
	kPrintMonitorDocsFolderAliasType = FourCharCode('fapn');
	kPreferencesFolderAliasType = FourCharCode('fapf');
	kControlPanelFolderAliasType = FourCharCode('fact');
	kExtensionFolderAliasType = FourCharCode('faex');

{ Types for AppleShare folder aliases }
const
	kExportedFolderAliasType = FourCharCode('faet');
	kDropFolderAliasType = FourCharCode('fadr');
	kSharedFolderAliasType = FourCharCode('fash');
	kMountedFolderAliasType = FourCharCode('famn');

{ Finder flags (finderFlags, fdFlags and frFlags) }
{ Any flag reserved or not specified should be set to 0. }
{ If a flag applies to a file, but not to a folder, make sure to check }
{ that the item is not a folder by checking ((ParamBlockRec.ioFlAttrib & ioDirMask) == 0) }
const
	kIsOnDesk = $0001; { Files and folders (System 6) }
	kColor = $000E; { Files and folders }
                                        { bit 0x0020 was kRequireSwitchLaunch, but is now reserved for future use}
	kIsShared = $0040; { Files only (Applications only) }
                                        { If clear, the application needs to write to }
                                        { its resource fork, and therefore cannot be }
                                        { shared on a server }
	kHasNoINITs = $0080; { Files only (Extensions/Control Panels only) }
                                        { This file contains no INIT resource }
	kHasBeenInited = $0100; { Files only }
                                        { Clear if the file contains desktop database }
                                        { resources ('BNDL', 'FREF', 'open', 'kind'...) }
                                        { that have not been added yet. Set only by the Finder }
                                        { Reserved for folders - make sure this bit is cleared for folders }
                                        { bit 0x0200 was the letter bit for AOCE, but is now reserved for future use }
	kHasCustomIcon = $0400; { Files and folders }
	kIsStationery = $0800; { Files only }
	kNameLocked = $1000; { Files and folders }
	kHasBundle = $2000; { Files and folders }
                                        { Indicates that a file has a BNDL resource }
                                        { Indicates that a folder is displayed as a package }
	kIsInvisible = $4000; { Files and folders }
	kIsAlias = $8000; { Files only }

{ Obsolete. Use names defined above. }
const
	fOnDesk = kIsOnDesk;
	fHasBundle = kHasBundle;
	fInvisible = kIsInvisible;

{ Obsolete }
const
	fTrash = -3;
	fDesktop = -2;
	fDisk = 0;

{$ifc OLDROUTINENAMES}
const
	kIsStationary = kIsStationery;

{$endc} {OLDROUTINENAMES}

{ Extended flags (extendedFinderFlags, fdXFlags and frXFlags) }
{ Any flag not specified should be set to 0. }
const
	kExtendedFlagsAreInvalid = $8000; { If set the other extended flags are ignored }
	kExtendedFlagHasCustomBadge = $0100; { Set if the file or folder has a badge resource }
	kExtendedFlagObjectIsBusy = $0080; { Set if the object is marked as busy/incomplete }
	kExtendedFlagHasRoutingInfo = $0004; { Set if the file contains routing info resource }


{ Use a filetype in this range to indicate that a file is temporarily busy }
{ (while it is being downloaded or installed, for example).  This prevents }
{ Finder 8.5 and later from trying to change the item's attributes before it }
{ is fully created. -- If you provide a series of 'BNDL' icons for your creator }
{ and some of these filetypes, you can achieve limited icon animation while }
{ the file creation progresses. }
const
	kFirstMagicBusyFiletype = FourCharCode('bzy ');
	kLastMagicBusyFiletype = FourCharCode('bzy?');

{ Use this date as a file's or folder's creation date to indicate that it is }
{ temporarily busy (while it is being downloaded or installed, for example). }
{ This prevents Finder from trying to change the item's attributes before it }
{ is fully created (Finder 8.5 and 8.6 check file creation dates; later Finders }
{ may check folder creation dates as well). }
const
	kMagicBusyCreationDate = $4F3AFDB0;


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{------------------------------------------------------------------------}
{
   The following data structures are binary compatible with FInfo, DInfo,
   FXInfo and DXInfo but represent the Mac OS 8 semantic of the fields.
   Use these data structures preferably to FInfo, etc...
}
{------------------------------------------------------------------------}
>>>>>>> origin/fixes_2.4

type
	FileInfoPtr = ^FileInfo;
	FileInfo = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		fileType: OSType;               { The type of the file }
		fileCreator: OSType;            { The file's creator }
		finderFlags: UInt16;            { ex: kHasBundle, kIsInvisible... }
		location: Point;               { File's location in the folder }
                                              { If set to (0, 0), the Finder will place the item automatically }
		reservedField: UInt16;          { (set to 0) }
<<<<<<< HEAD
	end;
type
	FolderInfoPtr = ^FolderInfo;
	FolderInfo = record
		windowBounds: Rect;           { The position and dimension of the folder's window }
		finderFlags: UInt16;            { ex. kIsInvisible, kNameLocked, etc.}
		location: Point;               { Folder's location in the parent folder }
                                              { If set to (0, 0), the Finder will place the item automatically }
		reservedField: UInt16;          { (set to 0) }
	end;
type
	ExtendedFileInfoPtr = ^ExtendedFileInfo;
	ExtendedFileInfo = record
		reserved1: array [0..3] of SInt16;           { Reserved (set to 0) }
		extendedFinderFlags: UInt16;    { Extended flags (custom badge, routing info...) }
		reserved2: SInt16;              { Reserved (set to 0). Comment ID if high-bit is clear }
		putAwayFolderID: SInt32;        { Put away folder ID }
	end;
type
	ExtendedFolderInfoPtr = ^ExtendedFolderInfo;
	ExtendedFolderInfo = record
		scrollPosition: Point;         { Scroll position (for icon views) }
		reserved1: SInt32;              { Reserved (set to 0) }
		extendedFinderFlags: UInt16;    { Extended flags (custom badge, routing info...) }
		reserved2: SInt16;              { Reserved (set to 0). Comment ID if high-bit is clear }
		putAwayFolderID: SInt32;        { Put away folder ID }
	end;
{------------------------------------------------------------------------}
{
   The following data structures are here for compatibility.
   Use the new data structures replacing them if possible (i.e. FileInfo 
   instead of FInfo, etc...)
}
{------------------------------------------------------------------------}
{ File info }
{
     IMPORTANT:
     In MacOS 8, the fdFldr field has become reserved for the Finder.
}
type
	FInfoPtr = ^FInfo;
	FInfo = record
		fdType: OSType;                 { The type of the file }
		fdCreator: OSType;              { The file's creator }
		fdFlags: UInt16;                { Flags ex. kHasBundle, kIsInvisible, etc. }
		fdLocation: Point;             { File's location in folder. }
                                              { If set to (0, 0), the Finder will place the item automatically }
		fdFldr: SInt16;                 { Reserved (set to 0) }
	end;
{ Extended file info }
{
     IMPORTANT:
     In MacOS 8, the fdIconID and fdComment fields were changed
     to become reserved fields for the Finder.
     The fdScript has become an extended flag.
}
type
	FXInfoPtr = ^FXInfo;
	FXInfo = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		fdIconID: SInt16;              { Reserved (set to 0) }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
		fdReserved: array [0..2] of SInt16;          { Reserved (set to 0) }
		fdScript: SInt8;               { Extended flags. Script code if high-bit is set }
		fdXFlags: SInt8;               { Extended flags }
		fdComment: SInt16;              { Reserved (set to 0). Comment ID if high-bit is clear }
		fdPutAway: SInt32;              { Put away folder ID }
	end;
{ Folder info }
{
     IMPORTANT:
     In MacOS 8, the frView field was changed to become reserved 
     field for the Finder.
}
type
	DInfoPtr = ^DInfo;
	DInfo = record
		frRect: Rect;                 { Folder's window bounds }
		frFlags: UInt16;                { Flags ex. kIsInvisible, kNameLocked, etc.}
		frLocation: Point;             { Folder's location in parent folder }
                                              { If set to (0, 0), the Finder will place the item automatically }
		frView: SInt16;                 { Reserved (set to 0) }
	end;
{ Extended folder info }
{
     IMPORTANT:
     In MacOS 8, the frOpenChain and frComment fields were changed
     to become reserved fields for the Finder.
     The frScript has become an extended flag.
}
type
	DXInfoPtr = ^DXInfo;
	DXInfo = record
		frScroll: Point;               { Scroll position }
		frOpenChain: SInt32;            { Reserved (set to 0) }
		frScript: SInt8;               { Extended flags. Script code if high-bit is set }
		frXFlags: SInt8;               { Extended flags }
		frComment: SInt16;              { Reserved (set to 0). Comment ID if high-bit is clear }
		frPutAway: SInt32;              { Put away folder ID }
	end;
{ ControlPanelDefProcPtr and cdev constants have all been moved to Processes.i}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
		fileType:				OSType;									{  The type of the file  }
		fileCreator:			OSType;									{  The file's creator  }
		finderFlags:			UInt16;									{  ex: kHasBundle, kIsInvisible...  }
		location:				Point;									{  File's location in the folder  }
																		{  If set to (0, 0), the Finder will place the item automatically  }
		reservedField:			UInt16;									{  (set to 0)  }
=======
>>>>>>> origin/fixes_2.4
	end;
type
	FolderInfoPtr = ^FolderInfo;
	FolderInfo = record
		windowBounds: Rect;           { The position and dimension of the folder's window }
		finderFlags: UInt16;            { ex. kIsInvisible, kNameLocked, etc.}
		location: Point;               { Folder's location in the parent folder }
                                              { If set to (0, 0), the Finder will place the item automatically }
		reservedField: UInt16;          { (set to 0) }
	end;
type
	ExtendedFileInfoPtr = ^ExtendedFileInfo;
	ExtendedFileInfo = record
		reserved1: array [0..3] of SInt16;           { Reserved (set to 0) }
		extendedFinderFlags: UInt16;    { Extended flags (custom badge, routing info...) }
		reserved2: SInt16;              { Reserved (set to 0). Comment ID if high-bit is clear }
		putAwayFolderID: SInt32;        { Put away folder ID }
	end;
type
	ExtendedFolderInfoPtr = ^ExtendedFolderInfo;
	ExtendedFolderInfo = record
		scrollPosition: Point;         { Scroll position (for icon views) }
		reserved1: SInt32;              { Reserved (set to 0) }
		extendedFinderFlags: UInt16;    { Extended flags (custom badge, routing info...) }
		reserved2: SInt16;              { Reserved (set to 0). Comment ID if high-bit is clear }
		putAwayFolderID: SInt32;        { Put away folder ID }
	end;
{------------------------------------------------------------------------}
{
   The following data structures are here for compatibility.
   Use the new data structures replacing them if possible (i.e. FileInfo 
   instead of FInfo, etc...)
}
{------------------------------------------------------------------------}
{ File info }
{
     IMPORTANT:
     In MacOS 8, the fdFldr field has become reserved for the Finder.
}
type
	FInfoPtr = ^FInfo;
	FInfo = record
		fdType: OSType;                 { The type of the file }
		fdCreator: OSType;              { The file's creator }
		fdFlags: UInt16;                { Flags ex. kHasBundle, kIsInvisible, etc. }
		fdLocation: Point;             { File's location in folder. }
                                              { If set to (0, 0), the Finder will place the item automatically }
		fdFldr: SInt16;                 { Reserved (set to 0) }
	end;
{ Extended file info }
{
     IMPORTANT:
     In MacOS 8, the fdIconID and fdComment fields were changed
     to become reserved fields for the Finder.
     The fdScript has become an extended flag.
}
type
	FXInfoPtr = ^FXInfo;
	FXInfo = record
		fdReserved: array [0..2] of SInt16;          { Reserved (set to 0) }
		fdScript: SInt8;               { Extended flags. Script code if high-bit is set }
		fdXFlags: SInt8;               { Extended flags }
		fdComment: SInt16;              { Reserved (set to 0). Comment ID if high-bit is clear }
		fdPutAway: SInt32;              { Put away folder ID }
	end;
{ Folder info }
{
     IMPORTANT:
     In MacOS 8, the frView field was changed to become reserved 
     field for the Finder.
}
type
	DInfoPtr = ^DInfo;
	DInfo = record
		frRect: Rect;                 { Folder's window bounds }
		frFlags: UInt16;                { Flags ex. kIsInvisible, kNameLocked, etc.}
		frLocation: Point;             { Folder's location in parent folder }
                                              { If set to (0, 0), the Finder will place the item automatically }
		frView: SInt16;                 { Reserved (set to 0) }
	end;
{ Extended folder info }
{
     IMPORTANT:
     In MacOS 8, the frOpenChain and frComment fields were changed
     to become reserved fields for the Finder.
     The frScript has become an extended flag.
}
type
	DXInfoPtr = ^DXInfo;
	DXInfo = record
		frScroll: Point;               { Scroll position }
		frOpenChain: SInt32;            { Reserved (set to 0) }
		frScript: SInt8;               { Extended flags. Script code if high-bit is set }
		frXFlags: SInt8;               { Extended flags }
		frComment: SInt16;              { Reserved (set to 0). Comment ID if high-bit is clear }
		frPutAway: SInt32;              { Put away folder ID }
	end;
{ ControlPanelDefProcPtr and cdev constants have all been moved to Processes.i}

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
