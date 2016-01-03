{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/QuickTimeMusic.h
 
     Contains:   QuickTime Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    QuickTime 7.7.1
 
     Copyright:  © 1990-2012 by Apple Inc., all rights reserved
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       QuickTimeMusic.p
=======
     File:       QuickTime/QuickTimeMusic.h
>>>>>>> origin/fixes_2.4
 
     Contains:   QuickTime Interfaces.
 
     Version:    QuickTime 7.6.3
 
<<<<<<< HEAD
     Copyright:  © 1990-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
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
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit QuickTimeMusic;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,Components,Dialogs,Endian,Files,ImageCompression,Movies,QuickdrawTypes;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
=======

>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifc not TARGET_CPU_64}

const
	kaiToneDescType = FourCharCode('tone');
	kaiNoteRequestInfoType = FourCharCode('ntrq');
	kaiKnobListType = FourCharCode('knbl');
	kaiKeyRangeInfoType = FourCharCode('sinf');
	kaiSampleDescType = FourCharCode('sdsc');
	kaiSampleInfoType = FourCharCode('smin');
	kaiSampleDataType = FourCharCode('sdat');
	kaiSampleDataQUIDType = FourCharCode('quid');
	kaiInstInfoType = FourCharCode('iinf');
	kaiPictType = FourCharCode('pict');
	kaiWriterType = FourCharCode('©wrt');
	kaiCopyrightType = FourCharCode('©cpy');
	kaiOtherStrType = FourCharCode('str ');
	kaiInstrumentRefType = FourCharCode('iref');
	kaiInstGMQualityType = FourCharCode('qual');
	kaiLibraryInfoType = FourCharCode('linf');
	kaiLibraryDescType = FourCharCode('ldsc');
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

const
	kaiToneDescType = FourCharCode('tone');
	kaiNoteRequestInfoType = FourCharCode('ntrq');
	kaiKnobListType = FourCharCode('knbl');
	kaiKeyRangeInfoType = FourCharCode('sinf');
	kaiSampleDescType = FourCharCode('sdsc');
	kaiSampleInfoType = FourCharCode('smin');
	kaiSampleDataType = FourCharCode('sdat');
	kaiSampleDataQUIDType = FourCharCode('quid');
	kaiInstInfoType = FourCharCode('iinf');
	kaiPictType = FourCharCode('pict');
	kaiWriterType = FourCharCode('©wrt');
	kaiCopyrightType = FourCharCode('©cpy');
	kaiOtherStrType = FourCharCode('str ');
	kaiInstrumentRefType = FourCharCode('iref');
	kaiInstGMQualityType = FourCharCode('qual');
	kaiLibraryInfoType = FourCharCode('linf');
	kaiLibraryDescType = FourCharCode('ldsc');
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Dialogs,Files,Components,Quickdraw,Video,MacMemory,Sound,Endian,ImageCompression,Movies;
=======
uses MacTypes,Components,Dialogs,Endian,Files,ImageCompression,Movies,QuickdrawTypes;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

const
	kaiToneDescType = FourCharCode('tone');
	kaiNoteRequestInfoType = FourCharCode('ntrq');
	kaiKnobListType = FourCharCode('knbl');
	kaiKeyRangeInfoType = FourCharCode('sinf');
	kaiSampleDescType = FourCharCode('sdsc');
	kaiSampleInfoType = FourCharCode('smin');
	kaiSampleDataType = FourCharCode('sdat');
	kaiSampleDataQUIDType = FourCharCode('quid');
	kaiInstInfoType = FourCharCode('iinf');
	kaiPictType = FourCharCode('pict');
	kaiWriterType = FourCharCode('©wrt');
	kaiCopyrightType = FourCharCode('©cpy');
	kaiOtherStrType = FourCharCode('str ');
	kaiInstrumentRefType = FourCharCode('iref');
	kaiInstGMQualityType = FourCharCode('qual');
	kaiLibraryInfoType = FourCharCode('linf');
	kaiLibraryDescType = FourCharCode('ldsc');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type
	InstLibDescRecPtr = ^InstLibDescRec;
	InstLibDescRec = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		libIDName: Str31;
	end;
type
	InstKnobRecPtr = ^InstKnobRec;
	InstKnobRec = record
		number: BigEndianLong;
		value: BigEndianLong;
	end;
const
	kInstKnobMissingUnknown = 0;
	kInstKnobMissingDefault = 1 shl 0;
=======
=======
>>>>>>> origin/fixes_2_2
		libIDName:				Str31;
=======
		libIDName: Str31;
>>>>>>> origin/fixes_2.4
	end;
type
	InstKnobRecPtr = ^InstKnobRec;
	InstKnobRec = record
		number: BigEndianLong;
		value: BigEndianLong;
	end;
const
<<<<<<< HEAD
	kInstKnobMissingUnknown		= 0;
	kInstKnobMissingDefault		= $01;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kInstKnobMissingUnknown = 0;
	kInstKnobMissingDefault = 1 shl 0;
>>>>>>> origin/fixes_2.4

type
	InstKnobListPtr = ^InstKnobList;
	InstKnobList = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		knobCount: BigEndianLong;
		knobFlags: BigEndianLong;
		knob: array [0..0] of InstKnobRec;
	end;
const
	kMusicLoopTypeNormal = 0;
	kMusicLoopTypePalindrome = 1;     { back & forth}

const
	instSamplePreProcessFlag = 1 shl 0;
=======
=======
>>>>>>> origin/fixes_2_2
		knobCount:				BigEndianLong;
		knobFlags:				BigEndianLong;
		knob:					array [0..0] of InstKnobRec;
=======
		knobCount: BigEndianLong;
		knobFlags: BigEndianLong;
		knob: array [0..0] of InstKnobRec;
>>>>>>> origin/fixes_2.4
	end;
const
	kMusicLoopTypeNormal = 0;
	kMusicLoopTypePalindrome = 1;     { back & forth}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	instSamplePreProcessFlag = 1 shl 0;
>>>>>>> origin/fixes_2.4

type
	InstSampleDescRecPtr = ^InstSampleDescRec;
	InstSampleDescRec = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		dataFormat: BigEndianOSType;
		numChannels: BigEndianShort;
		sampleSize: BigEndianShort;
		sampleRate: BigEndianUnsignedFixed;
		sampleDataID: BigEndianShort;
		offset: BigEndianLong;                 { offset within SampleData - this could be just for internal use}
		numSamples: BigEndianLong;             { this could also just be for internal use, we'll see}

		loopType: BigEndianLong;
		loopStart: BigEndianLong;
		loopEnd: BigEndianLong;

		pitchNormal: BigEndianLong;
		pitchLow: BigEndianLong;
		pitchHigh: BigEndianLong;
<<<<<<< HEAD
	end;

type
	AtomicInstrument = Handle;
	AtomicInstrumentPtr = Ptr;
const
	kQTMIDIComponentType = FourCharCode('midi');

const
	kOMSComponentSubType = FourCharCode('OMS ');
	kFMSComponentSubType = FourCharCode('FMS ');
	kMIDIManagerComponentSubType = FourCharCode('mmgr');
	kOSXMIDIComponentSubType = FourCharCode('osxm');

type
	QTMIDIComponent = ComponentInstance;
const
	kMusicPacketPortLost = 1;    { received when application loses the default input port }
	kMusicPacketPortFound = 2;    { received when application gets it back out from under someone else's claim }
	kMusicPacketTimeGap = 3;     { data[0] = number of milliseconds to keep the MIDI line silent }

const
	kAppleSysexID = $11; { apple sysex is followed by 2-byte command. 0001 is the command for samplesize }
	kAppleSysexCmdSampleSize = $0001; { 21 bit number in 3 midi bytes follows sysex ID and 2 cmd bytes }
	kAppleSysexCmdSampleBreak = $0002; { specifies that the sample should break right here }
	kAppleSysexCmdAtomicInstrument = $0010; { contents of atomic instrument handle }
	kAppleSysexCmdDeveloper = $7F00; { F0 11 7F 00 ww xx yy zz ... F7 is available for non-Apple developers, where wxyz is unique app signature with 8th bit cleared, unique to developer, and 00 and 7f are reserved }

type
	MusicMIDIPacket = record
		length: UInt16;
		reserved: UNSIGNEDLONG;               { if length zero, then reserved = above enum }
		data: packed array [0..248] of UInt8;
	end;
type
	MusicMIDISendProcPtr = function( self: ComponentInstance; refCon: SIGNEDLONG; var mmp: MusicMIDIPacket ): ComponentResult;
	MusicMIDISendUPP = MusicMIDISendProcPtr;
const
	kSynthesizerConnectionFMS = 1;    { this connection imported from FMS }
	kSynthesizerConnectionMMgr = 2;    { this connection imported from the MIDI Mgr }
	kSynthesizerConnectionOMS = 4;    { this connection imported from OMS }
	kSynthesizerConnectionQT = 8;    { this connection is a QuickTime-only port }
	kSynthesizerConnectionOSXMIDI = 16;   { this connection is an OS X CoreMIDI port }
                                        { lowest five bits are mutually exclusive; combinations reserved for future use.}
	kSynthesizerConnectionUnavailable = 256; { port exists, but cannot be used just now }

{
    The sampleBankFile field of this structure can be used to pass in a pointer to an FSSpec
    that represents a SoundFont 2 or DLS file (otherwise set it to NULL ).
    
    You then pass in a structure with this field set (all other fields should be zero) to
    NARegisterMusicDevice:
        - with synthType as kSoftSynthComponentSubType 
        - with name being used to return to the application the "name" of the synth 
        that should be used in the synthesiserName field of the ToneDescription structure
        and is also used to retrieve a particular MusicComponent with the
        NAGetRegisteredMusicDevice call
    
    This call will create a MusicComponent of kSoftSynthComponentSubType, with the specified
    sound bank as the sample data source.

    This field requires QuickTime 5.0 or later and should be set to NULL for prior versions.
}
type
	SynthesizerConnectionsPtr = ^SynthesizerConnections;
	SynthesizerConnections = record
		clientID: OSType;
		inputPortID: OSType;            { terminology death: this port is used to SEND to the midi synth }
		outputPortID: OSType;           { terminology death: this port receives from a keyboard or other control device }
		midiChannel: SIGNEDLONG;            { The system channel; others are configurable (or the nubus slot number) }
		flags: SIGNEDLONG;
		unique: SIGNEDLONG;                 { unique id may be used instead of index, to getinfo and unregister calls }
		sampleBankFile: FSSpecPtr;         {  see notes above }
		reserved2: SIGNEDLONG;              { should be zero }
	end;
type
	QTMIDIPortPtr = ^QTMIDIPort;
	QTMIDIPort = record
		portConnections: SynthesizerConnections;
		portName: Str63;
	end;
type
	QTMIDIPortList = record
		portCount: SInt16;
		port: array [0..0] of QTMIDIPort;
	end;
	QTMIDIPortListPtr = ^QTMIDIPortList;
type
	QTMIDIPortListHandle = ^QTMIDIPortListPtr;
{
 *  QTMIDIGetMIDIPorts()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTMIDIGetMIDIPorts( ci: QTMIDIComponent; var inputPorts: QTMIDIPortListHandle; var outputPorts: QTMIDIPortListHandle ): ComponentResult; external name '_QTMIDIGetMIDIPorts';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  QTMIDIUseSendPort()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTMIDIUseSendPort( ci: QTMIDIComponent; portIndex: SIGNEDLONG; inUse: SIGNEDLONG ): ComponentResult; external name '_QTMIDIUseSendPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  QTMIDISendMIDI()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTMIDISendMIDI( ci: QTMIDIComponent; portIndex: SIGNEDLONG; var mp: MusicMIDIPacket ): ComponentResult; external name '_QTMIDISendMIDI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


const
	kMusicComponentType = FourCharCode('musi');
	kInstrumentComponentType = FourCharCode('inst');

const
	kSoftSynthComponentSubType = FourCharCode('ss  ');
	kGMSynthComponentSubType = FourCharCode('gm  ');


type
	MusicComponent = ComponentInstance;
{ MusicSynthesizerFlags}
const
	kSynthesizerDynamicVoice = 1 shl 0; { can assign voices on the fly (else, polyphony is very important }
	kSynthesizerUsesMIDIPort = 1 shl 1; { must be patched through MIDI Manager }
	kSynthesizerMicrotone = 1 shl 2; { can play microtonal scales }
	kSynthesizerHasSamples = 1 shl 3; { synthesizer has some use for sampled data }
	kSynthesizerMixedDrums = 1 shl 4; { any part can play drum parts, total = instrument parts }
	kSynthesizerSoftware = 1 shl 5; { implemented in main CPU software == uses cpu cycles }
	kSynthesizerHardware = 1 shl 6; { is a hardware device (such as nubus, or maybe DSP?) }
	kSynthesizerDynamicChannel = 1 shl 7; { can move any part to any channel or disable each part. (else we assume it lives on all channels in masks) }
	kSynthesizerHogsSystemChannel = 1 shl 8; { can be channelwise dynamic, but always responds on its system channel }
	kSynthesizerHasSystemChannel = 1 shl 9; { has some "system channel" notion to distinguish it from multiple instances of the same device (GM devices dont) }
	kSynthesizerSlowSetPart = 1 shl 10; { SetPart() and SetPartInstrumentNumber() calls do not have rapid response, may glitch notes }
	kSynthesizerOffline = 1 shl 12; { can enter an offline synthesis mode }
	kSynthesizerGM = 1 shl 14; { synth is a GM device }
	kSynthesizerDLS = 1 shl 15; { synth supports DLS level 1 }
	kSynthesizerSoundLocalization = 1 shl 16; { synth supports extremely baroque, nonstandard, and proprietary "apple game sprockets" localization parameter set }

{
 * Note that these controller numbers
 * are _not_ identical to the MIDI controller numbers.
 * These are _signed_ 8.8 values, and the LSB's are
 * always sent to a MIDI device. Controllers 32-63 are
 * reserved (for MIDI, they are LSB's for 0-31, but we
 * always send both).
 *
 * The full range, therefore, is -128.00 to 127.7f.
 *
 * _Excepting_ _volume_, all controls default to zero.
 *
 * Pitch bend is specified in fractional semitones! No
 * more "pitch bend range" nonsense. You can bend as far
 * as you want, any time you want.
 }
type
	MusicController = SInt32;
const
	kControllerModulationWheel = 1;
	kControllerBreath = 2;
	kControllerFoot = 4;
	kControllerPortamentoTime = 5;    { time in 8.8 seconds, portamento on/off is omitted, 0 time = 'off' }
	kControllerVolume = 7;    { main volume control }
	kControllerBalance = 8;
	kControllerPan = 10;   { 0 - "default", 1 - n: positioned in output 1-n (incl fractions) }
	kControllerExpression = 11;   { secondary volume control }
	kControllerLever1 = 16;   { general purpose controllers }
	kControllerLever2 = 17;   { general purpose controllers }
	kControllerLever3 = 18;   { general purpose controllers }
	kControllerLever4 = 19;   { general purpose controllers }
	kControllerLever5 = 80;   { general purpose controllers }
	kControllerLever6 = 81;   { general purpose controllers }
	kControllerLever7 = 82;   { general purpose controllers }
	kControllerLever8 = 83;   { general purpose controllers }
	kControllerPitchBend = 32;   { positive & negative semitones, with 8 bits fraction, same units as transpose controllers}
	kControllerAfterTouch = 33;   { aka channel pressure }
	kControllerPartTranspose = 40;   { identical to pitchbend, for overall part xpose }
	kControllerTuneTranspose = 41;   { another pitchbend, for "song global" pitch offset }
	kControllerPartVolume = 42;   { another volume control, passed right down from note allocator part volume }
	kControllerTuneVolume = 43;   { another volume control, used for "song global" volume - since we share one synthesizer across multiple tuneplayers}
	kControllerSustain = 64;   { boolean - positive for on, 0 or negative off }
	kControllerPortamento = 65;   { boolean}
	kControllerSostenuto = 66;   { boolean }
	kControllerSoftPedal = 67;   { boolean }
	kControllerReverb = 91;
	kControllerTremolo = 92;
	kControllerChorus = 93;
	kControllerCeleste = 94;
	kControllerPhaser = 95;
	kControllerEditPart = 113;  { last 16 controllers 113-128 and above are global controllers which respond on part zero }
	kControllerMasterTune = 114;
	kControllerMasterTranspose = 114;  { preferred}
	kControllerMasterVolume = 115;
	kControllerMasterCPULoad = 116;
	kControllerMasterPolyphony = 117;
	kControllerMasterFeatures = 118;


{ ID's of knobs supported by the QuickTime Music Synthesizer built into QuickTime}

const
	kQTMSKnobStartID = $02000000;
	kQTMSKnobVolumeAttackTimeID = $02000001;
	kQTMSKnobVolumeDecayTimeID = $02000002;
=======
=======
>>>>>>> origin/fixes_2_2
		dataFormat:				BigEndianOSType;
		numChannels:			BigEndianShort;
		sampleSize:				BigEndianShort;
		sampleRate:				BigEndianUnsignedFixed;
		sampleDataID:			BigEndianShort;
		offset:					BigEndianLong;							{  offset within SampleData - this could be just for internal use }
		numSamples:				BigEndianLong;							{  this could also just be for internal use, we'll see }
		loopType:				BigEndianLong;
		loopStart:				BigEndianLong;
		loopEnd:				BigEndianLong;
		pitchNormal:			BigEndianLong;
		pitchLow:				BigEndianLong;
		pitchHigh:				BigEndianLong;
=======
>>>>>>> origin/fixes_2.4
	end;

type
	AtomicInstrument = Handle;
	AtomicInstrumentPtr = Ptr;
const
	kQTMIDIComponentType = FourCharCode('midi');

const
	kOMSComponentSubType = FourCharCode('OMS ');
	kFMSComponentSubType = FourCharCode('FMS ');
	kMIDIManagerComponentSubType = FourCharCode('mmgr');
	kOSXMIDIComponentSubType = FourCharCode('osxm');

type
	QTMIDIComponent = ComponentInstance;
const
	kMusicPacketPortLost = 1;    { received when application loses the default input port }
	kMusicPacketPortFound = 2;    { received when application gets it back out from under someone else's claim }
	kMusicPacketTimeGap = 3;     { data[0] = number of milliseconds to keep the MIDI line silent }

const
	kAppleSysexID = $11; { apple sysex is followed by 2-byte command. 0001 is the command for samplesize }
	kAppleSysexCmdSampleSize = $0001; { 21 bit number in 3 midi bytes follows sysex ID and 2 cmd bytes }
	kAppleSysexCmdSampleBreak = $0002; { specifies that the sample should break right here }
	kAppleSysexCmdAtomicInstrument = $0010; { contents of atomic instrument handle }
	kAppleSysexCmdDeveloper = $7F00; { F0 11 7F 00 ww xx yy zz ... F7 is available for non-Apple developers, where wxyz is unique app signature with 8th bit cleared, unique to developer, and 00 and 7f are reserved }

type
	MusicMIDIPacket = record
		length: UInt16;
		reserved: UNSIGNEDLONG;               { if length zero, then reserved = above enum }
		data: packed array [0..248] of UInt8;
	end;
type
	MusicMIDISendProcPtr = function( self: ComponentInstance; refCon: SIGNEDLONG; var mmp: MusicMIDIPacket ): ComponentResult;
	MusicMIDISendUPP = MusicMIDISendProcPtr;
const
	kSynthesizerConnectionFMS = 1;    { this connection imported from FMS }
	kSynthesizerConnectionMMgr = 2;    { this connection imported from the MIDI Mgr }
	kSynthesizerConnectionOMS = 4;    { this connection imported from OMS }
	kSynthesizerConnectionQT = 8;    { this connection is a QuickTime-only port }
	kSynthesizerConnectionOSXMIDI = 16;   { this connection is an OS X CoreMIDI port }
                                        { lowest five bits are mutually exclusive; combinations reserved for future use.}
	kSynthesizerConnectionUnavailable = 256; { port exists, but cannot be used just now }

{
    The sampleBankFile field of this structure can be used to pass in a pointer to an FSSpec
    that represents a SoundFont 2 or DLS file (otherwise set it to NULL ).
    
    You then pass in a structure with this field set (all other fields should be zero) to
    NARegisterMusicDevice:
        - with synthType as kSoftSynthComponentSubType 
        - with name being used to return to the application the "name" of the synth 
        that should be used in the synthesiserName field of the ToneDescription structure
        and is also used to retrieve a particular MusicComponent with the
        NAGetRegisteredMusicDevice call
    
    This call will create a MusicComponent of kSoftSynthComponentSubType, with the specified
    sound bank as the sample data source.

    This field requires QuickTime 5.0 or later and should be set to NULL for prior versions.
}
type
	SynthesizerConnectionsPtr = ^SynthesizerConnections;
	SynthesizerConnections = record
		clientID: OSType;
		inputPortID: OSType;            { terminology death: this port is used to SEND to the midi synth }
		outputPortID: OSType;           { terminology death: this port receives from a keyboard or other control device }
		midiChannel: SIGNEDLONG;            { The system channel; others are configurable (or the nubus slot number) }
		flags: SIGNEDLONG;
		unique: SIGNEDLONG;                 { unique id may be used instead of index, to getinfo and unregister calls }
		sampleBankFile: FSSpecPtr;         {  see notes above }
		reserved2: SIGNEDLONG;              { should be zero }
	end;
type
	QTMIDIPortPtr = ^QTMIDIPort;
	QTMIDIPort = record
		portConnections: SynthesizerConnections;
		portName: Str63;
	end;
type
	QTMIDIPortList = record
		portCount: SInt16;
		port: array [0..0] of QTMIDIPort;
	end;
	QTMIDIPortListPtr = ^QTMIDIPortList;
type
	QTMIDIPortListHandle = ^QTMIDIPortListPtr;
{
 *  QTMIDIGetMIDIPorts()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTMIDIGetMIDIPorts( ci: QTMIDIComponent; var inputPorts: QTMIDIPortListHandle; var outputPorts: QTMIDIPortListHandle ): ComponentResult; external name '_QTMIDIGetMIDIPorts';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  QTMIDIUseSendPort()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTMIDIUseSendPort( ci: QTMIDIComponent; portIndex: SIGNEDLONG; inUse: SIGNEDLONG ): ComponentResult; external name '_QTMIDIUseSendPort';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  QTMIDISendMIDI()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTMIDISendMIDI( ci: QTMIDIComponent; portIndex: SIGNEDLONG; var mp: MusicMIDIPacket ): ComponentResult; external name '_QTMIDISendMIDI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


const
	kMusicComponentType = FourCharCode('musi');
	kInstrumentComponentType = FourCharCode('inst');

const
	kSoftSynthComponentSubType = FourCharCode('ss  ');
	kGMSynthComponentSubType = FourCharCode('gm  ');


type
	MusicComponent = ComponentInstance;
{ MusicSynthesizerFlags}
const
	kSynthesizerDynamicVoice = 1 shl 0; { can assign voices on the fly (else, polyphony is very important }
	kSynthesizerUsesMIDIPort = 1 shl 1; { must be patched through MIDI Manager }
	kSynthesizerMicrotone = 1 shl 2; { can play microtonal scales }
	kSynthesizerHasSamples = 1 shl 3; { synthesizer has some use for sampled data }
	kSynthesizerMixedDrums = 1 shl 4; { any part can play drum parts, total = instrument parts }
	kSynthesizerSoftware = 1 shl 5; { implemented in main CPU software == uses cpu cycles }
	kSynthesizerHardware = 1 shl 6; { is a hardware device (such as nubus, or maybe DSP?) }
	kSynthesizerDynamicChannel = 1 shl 7; { can move any part to any channel or disable each part. (else we assume it lives on all channels in masks) }
	kSynthesizerHogsSystemChannel = 1 shl 8; { can be channelwise dynamic, but always responds on its system channel }
	kSynthesizerHasSystemChannel = 1 shl 9; { has some "system channel" notion to distinguish it from multiple instances of the same device (GM devices dont) }
	kSynthesizerSlowSetPart = 1 shl 10; { SetPart() and SetPartInstrumentNumber() calls do not have rapid response, may glitch notes }
	kSynthesizerOffline = 1 shl 12; { can enter an offline synthesis mode }
	kSynthesizerGM = 1 shl 14; { synth is a GM device }
	kSynthesizerDLS = 1 shl 15; { synth supports DLS level 1 }
	kSynthesizerSoundLocalization = 1 shl 16; { synth supports extremely baroque, nonstandard, and proprietary "apple game sprockets" localization parameter set }

{
 * Note that these controller numbers
 * are _not_ identical to the MIDI controller numbers.
 * These are _signed_ 8.8 values, and the LSB's are
 * always sent to a MIDI device. Controllers 32-63 are
 * reserved (for MIDI, they are LSB's for 0-31, but we
 * always send both).
 *
 * The full range, therefore, is -128.00 to 127.7f.
 *
 * _Excepting_ _volume_, all controls default to zero.
 *
 * Pitch bend is specified in fractional semitones! No
 * more "pitch bend range" nonsense. You can bend as far
 * as you want, any time you want.
 }
type
	MusicController = SInt32;
const
	kControllerModulationWheel = 1;
	kControllerBreath = 2;
	kControllerFoot = 4;
	kControllerPortamentoTime = 5;    { time in 8.8 seconds, portamento on/off is omitted, 0 time = 'off' }
	kControllerVolume = 7;    { main volume control }
	kControllerBalance = 8;
	kControllerPan = 10;   { 0 - "default", 1 - n: positioned in output 1-n (incl fractions) }
	kControllerExpression = 11;   { secondary volume control }
	kControllerLever1 = 16;   { general purpose controllers }
	kControllerLever2 = 17;   { general purpose controllers }
	kControllerLever3 = 18;   { general purpose controllers }
	kControllerLever4 = 19;   { general purpose controllers }
	kControllerLever5 = 80;   { general purpose controllers }
	kControllerLever6 = 81;   { general purpose controllers }
	kControllerLever7 = 82;   { general purpose controllers }
	kControllerLever8 = 83;   { general purpose controllers }
	kControllerPitchBend = 32;   { positive & negative semitones, with 8 bits fraction, same units as transpose controllers}
	kControllerAfterTouch = 33;   { aka channel pressure }
	kControllerPartTranspose = 40;   { identical to pitchbend, for overall part xpose }
	kControllerTuneTranspose = 41;   { another pitchbend, for "song global" pitch offset }
	kControllerPartVolume = 42;   { another volume control, passed right down from note allocator part volume }
	kControllerTuneVolume = 43;   { another volume control, used for "song global" volume - since we share one synthesizer across multiple tuneplayers}
	kControllerSustain = 64;   { boolean - positive for on, 0 or negative off }
	kControllerPortamento = 65;   { boolean}
	kControllerSostenuto = 66;   { boolean }
	kControllerSoftPedal = 67;   { boolean }
	kControllerReverb = 91;
	kControllerTremolo = 92;
	kControllerChorus = 93;
	kControllerCeleste = 94;
	kControllerPhaser = 95;
	kControllerEditPart = 113;  { last 16 controllers 113-128 and above are global controllers which respond on part zero }
	kControllerMasterTune = 114;
	kControllerMasterTranspose = 114;  { preferred}
	kControllerMasterVolume = 115;
	kControllerMasterCPULoad = 116;
	kControllerMasterPolyphony = 117;
	kControllerMasterFeatures = 118;


{ ID's of knobs supported by the QuickTime Music Synthesizer built into QuickTime}

const
<<<<<<< HEAD
	kControllerModulationWheel	= 1;
	kControllerBreath			= 2;
	kControllerFoot				= 4;
	kControllerPortamentoTime	= 5;							{  time in 8.8 seconds, portamento on/off is omitted, 0 time = 'off'  }
	kControllerVolume			= 7;							{  main volume control  }
	kControllerBalance			= 8;
	kControllerPan				= 10;							{  0 - "default", 1 - n: positioned in output 1-n (incl fractions)  }
	kControllerExpression		= 11;							{  secondary volume control  }
	kControllerLever1			= 16;							{  general purpose controllers  }
	kControllerLever2			= 17;							{  general purpose controllers  }
	kControllerLever3			= 18;							{  general purpose controllers  }
	kControllerLever4			= 19;							{  general purpose controllers  }
	kControllerLever5			= 80;							{  general purpose controllers  }
	kControllerLever6			= 81;							{  general purpose controllers  }
	kControllerLever7			= 82;							{  general purpose controllers  }
	kControllerLever8			= 83;							{  general purpose controllers  }
	kControllerPitchBend		= 32;							{  positive & negative semitones, with 8 bits fraction, same units as transpose controllers }
	kControllerAfterTouch		= 33;							{  aka channel pressure  }
	kControllerPartTranspose	= 40;							{  identical to pitchbend, for overall part xpose  }
	kControllerTuneTranspose	= 41;							{  another pitchbend, for "song global" pitch offset  }
	kControllerPartVolume		= 42;							{  another volume control, passed right down from note allocator part volume  }
	kControllerTuneVolume		= 43;							{  another volume control, used for "song global" volume - since we share one synthesizer across multiple tuneplayers }
	kControllerSustain			= 64;							{  boolean - positive for on, 0 or negative off  }
	kControllerPortamento		= 65;							{  boolean }
	kControllerSostenuto		= 66;							{  boolean  }
	kControllerSoftPedal		= 67;							{  boolean  }
	kControllerReverb			= 91;
	kControllerTremolo			= 92;
	kControllerChorus			= 93;
	kControllerCeleste			= 94;
	kControllerPhaser			= 95;
	kControllerEditPart			= 113;							{  last 16 controllers 113-128 and above are global controllers which respond on part zero  }
	kControllerMasterTune		= 114;
	kControllerMasterTranspose	= 114;							{  preferred }
	kControllerMasterVolume		= 115;
	kControllerMasterCPULoad	= 116;
	kControllerMasterPolyphony	= 117;
	kControllerMasterFeatures	= 118;


	{  ID's of knobs supported by the QuickTime Music Synthesizer built into QuickTime }

	kQTMSKnobStartID			= $02000000;
	kQTMSKnobVolumeAttackTimeID	= $02000001;
	kQTMSKnobVolumeDecayTimeID	= $02000002;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTMSKnobStartID = $02000000;
	kQTMSKnobVolumeAttackTimeID = $02000001;
	kQTMSKnobVolumeDecayTimeID = $02000002;
>>>>>>> origin/fixes_2.4
	kQTMSKnobVolumeSustainLevelID = $02000003;
	kQTMSKnobVolumeRelease1RateID = $02000004;
	kQTMSKnobVolumeDecayKeyScalingID = $02000005;
	kQTMSKnobVolumeReleaseTimeID = $02000006;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kQTMSKnobVolumeLFODelayID = $02000007;
	kQTMSKnobVolumeLFORampTimeID = $02000008;
	kQTMSKnobVolumeLFOPeriodID = $02000009;
	kQTMSKnobVolumeLFOShapeID = $0200000A;
	kQTMSKnobVolumeLFODepthID = $0200000B;
	kQTMSKnobVolumeOverallID = $0200000C;
	kQTMSKnobVolumeVelocity127ID = $0200000D;
	kQTMSKnobVolumeVelocity96ID = $0200000E;
	kQTMSKnobVolumeVelocity64ID = $0200000F;
	kQTMSKnobVolumeVelocity32ID = $02000010;
	kQTMSKnobVolumeVelocity16ID = $02000011; { Pitch related knobs}
	kQTMSKnobPitchTransposeID = $02000012;
	kQTMSKnobPitchLFODelayID = $02000013;
	kQTMSKnobPitchLFORampTimeID = $02000014;
	kQTMSKnobPitchLFOPeriodID = $02000015;
	kQTMSKnobPitchLFOShapeID = $02000016;
	kQTMSKnobPitchLFODepthID = $02000017;
	kQTMSKnobPitchLFOQuantizeID = $02000018; { Stereo related knobs}
	kQTMSKnobStereoDefaultPanID = $02000019;
	kQTMSKnobStereoPositionKeyScalingID = $0200001A;
	kQTMSKnobPitchLFOOffsetID = $0200001B;
	kQTMSKnobExclusionGroupID = $0200001C; { Misc knobs, late additions}
	kQTMSKnobSustainTimeID = $0200001D;
	kQTMSKnobSustainInfiniteID = $0200001E;
	kQTMSKnobVolumeLFOStereoID = $0200001F;
	kQTMSKnobVelocityLowID = $02000020;
	kQTMSKnobVelocityHighID = $02000021;
	kQTMSKnobVelocitySensitivityID = $02000022;
	kQTMSKnobPitchSensitivityID = $02000023;
	kQTMSKnobVolumeLFODepthFromWheelID = $02000024;
	kQTMSKnobPitchLFODepthFromWheelID = $02000025; { Volume Env again}
	kQTMSKnobVolumeExpOptionsID = $02000026; { Env1}
	kQTMSKnobEnv1AttackTimeID = $02000027;
	kQTMSKnobEnv1DecayTimeID = $02000028;
	kQTMSKnobEnv1SustainLevelID = $02000029;
	kQTMSKnobEnv1SustainTimeID = $0200002A;
	kQTMSKnobEnv1SustainInfiniteID = $0200002B;
	kQTMSKnobEnv1ReleaseTimeID = $0200002C;
	kQTMSKnobEnv1ExpOptionsID = $0200002D; { Env2}
	kQTMSKnobEnv2AttackTimeID = $0200002E;
	kQTMSKnobEnv2DecayTimeID = $0200002F;
	kQTMSKnobEnv2SustainLevelID = $02000030;
	kQTMSKnobEnv2SustainTimeID = $02000031;
	kQTMSKnobEnv2SustainInfiniteID = $02000032;
	kQTMSKnobEnv2ReleaseTimeID = $02000033;
	kQTMSKnobEnv2ExpOptionsID = $02000034; { Pitch Env}
	kQTMSKnobPitchEnvelopeID = $02000035;
	kQTMSKnobPitchEnvelopeDepthID = $02000036; { Filter}
	kQTMSKnobFilterKeyFollowID = $02000037;
	kQTMSKnobFilterTransposeID = $02000038;
	kQTMSKnobFilterQID = $02000039;
	kQTMSKnobFilterFrequencyEnvelopeID = $0200003A;
	kQTMSKnobFilterFrequencyEnvelopeDepthID = $0200003B;
	kQTMSKnobFilterQEnvelopeID = $0200003C;
	kQTMSKnobFilterQEnvelopeDepthID = $0200003D; { Reverb Threshhold}
	kQTMSKnobReverbThresholdID = $0200003E;
	kQTMSKnobVolumeAttackVelScalingID = $0200003F;
	kQTMSKnobLastIDPlus1 = $02000040;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

=======

<<<<<<< HEAD
<<<<<<< HEAD

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
const
	kControllerMaximum = $00007FFF; { +01111111.11111111 }
	kControllerMinimum = $FFFF8000; { -10000000.00000000 }

type
	SynthesizerDescription = record
		synthesizerType: OSType;        { synthesizer type (must be same as component subtype) }
		name: Str31;                   { text name of synthesizer type }
		flags: UNSIGNEDLONG;                  { from the above enum }
		voiceCount: UNSIGNEDLONG;             { maximum polyphony }

		partCount: UNSIGNEDLONG;              { maximum multi-timbrality (and midi channels) }
		instrumentCount: UNSIGNEDLONG;        { non gm, built in (rom) instruments only }
		modifiableInstrumentCount: UNSIGNEDLONG; { plus n-more are user modifiable }
		channelMask: UNSIGNEDLONG;            { (midi device only) which channels device always uses }

		drumPartCount: UNSIGNEDLONG;          { maximum multi-timbrality of drum parts }
		drumCount: UNSIGNEDLONG;              { non gm, built in (rom) drumkits only }
		modifiableDrumCount: UNSIGNEDLONG;    { plus n-more are user modifiable }
		drumChannelMask: UNSIGNEDLONG;        { (midi device only) which channels device always uses }

		outputCount: UNSIGNEDLONG;            { number of audio outputs (usually two) }
		latency: UNSIGNEDLONG;                { response time in µSec }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		controllers: array [0..4-1] of UNSIGNEDLONG;         { array of 128 bits }
		gmInstruments: array [0..4-1] of UNSIGNEDLONG;       { array of 128 bits }
		gmDrums: array [0..4-1] of UNSIGNEDLONG;             { array of 128 bits }
=======
		controllers: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmInstruments: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmDrums: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
>>>>>>> graemeg/cpstrnew
=======
		controllers: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmInstruments: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmDrums: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
>>>>>>> graemeg/cpstrnew
=======
		controllers: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmInstruments: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmDrums: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
>>>>>>> graemeg/cpstrnew
=======
		controllers: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmInstruments: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmDrums: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
>>>>>>> origin/cpstrnew
	end;
const
	kVoiceCountDynamic = -1;    { constant to use to specify dynamic voicing }
=======
=======
>>>>>>> origin/fixes_2_2
	kQTMSKnobVolumeLFODelayID	= $02000007;
=======
	kQTMSKnobVolumeLFODelayID = $02000007;
>>>>>>> origin/fixes_2.4
	kQTMSKnobVolumeLFORampTimeID = $02000008;
	kQTMSKnobVolumeLFOPeriodID = $02000009;
	kQTMSKnobVolumeLFOShapeID = $0200000A;
	kQTMSKnobVolumeLFODepthID = $0200000B;
	kQTMSKnobVolumeOverallID = $0200000C;
	kQTMSKnobVolumeVelocity127ID = $0200000D;
	kQTMSKnobVolumeVelocity96ID = $0200000E;
	kQTMSKnobVolumeVelocity64ID = $0200000F;
	kQTMSKnobVolumeVelocity32ID = $02000010;
	kQTMSKnobVolumeVelocity16ID = $02000011; { Pitch related knobs}
	kQTMSKnobPitchTransposeID = $02000012;
	kQTMSKnobPitchLFODelayID = $02000013;
	kQTMSKnobPitchLFORampTimeID = $02000014;
	kQTMSKnobPitchLFOPeriodID = $02000015;
	kQTMSKnobPitchLFOShapeID = $02000016;
	kQTMSKnobPitchLFODepthID = $02000017;
	kQTMSKnobPitchLFOQuantizeID = $02000018; { Stereo related knobs}
	kQTMSKnobStereoDefaultPanID = $02000019;
	kQTMSKnobStereoPositionKeyScalingID = $0200001A;
	kQTMSKnobPitchLFOOffsetID = $0200001B;
	kQTMSKnobExclusionGroupID = $0200001C; { Misc knobs, late additions}
	kQTMSKnobSustainTimeID = $0200001D;
	kQTMSKnobSustainInfiniteID = $0200001E;
	kQTMSKnobVolumeLFOStereoID = $0200001F;
	kQTMSKnobVelocityLowID = $02000020;
	kQTMSKnobVelocityHighID = $02000021;
	kQTMSKnobVelocitySensitivityID = $02000022;
	kQTMSKnobPitchSensitivityID = $02000023;
	kQTMSKnobVolumeLFODepthFromWheelID = $02000024;
	kQTMSKnobPitchLFODepthFromWheelID = $02000025; { Volume Env again}
	kQTMSKnobVolumeExpOptionsID = $02000026; { Env1}
	kQTMSKnobEnv1AttackTimeID = $02000027;
	kQTMSKnobEnv1DecayTimeID = $02000028;
	kQTMSKnobEnv1SustainLevelID = $02000029;
	kQTMSKnobEnv1SustainTimeID = $0200002A;
	kQTMSKnobEnv1SustainInfiniteID = $0200002B;
	kQTMSKnobEnv1ReleaseTimeID = $0200002C;
	kQTMSKnobEnv1ExpOptionsID = $0200002D; { Env2}
	kQTMSKnobEnv2AttackTimeID = $0200002E;
	kQTMSKnobEnv2DecayTimeID = $0200002F;
	kQTMSKnobEnv2SustainLevelID = $02000030;
	kQTMSKnobEnv2SustainTimeID = $02000031;
	kQTMSKnobEnv2SustainInfiniteID = $02000032;
	kQTMSKnobEnv2ReleaseTimeID = $02000033;
	kQTMSKnobEnv2ExpOptionsID = $02000034; { Pitch Env}
	kQTMSKnobPitchEnvelopeID = $02000035;
	kQTMSKnobPitchEnvelopeDepthID = $02000036; { Filter}
	kQTMSKnobFilterKeyFollowID = $02000037;
	kQTMSKnobFilterTransposeID = $02000038;
	kQTMSKnobFilterQID = $02000039;
	kQTMSKnobFilterFrequencyEnvelopeID = $0200003A;
	kQTMSKnobFilterFrequencyEnvelopeDepthID = $0200003B;
	kQTMSKnobFilterQEnvelopeID = $0200003C;
	kQTMSKnobFilterQEnvelopeDepthID = $0200003D; { Reverb Threshhold}
	kQTMSKnobReverbThresholdID = $0200003E;
	kQTMSKnobVolumeAttackVelScalingID = $0200003F;
	kQTMSKnobLastIDPlus1 = $02000040;


const
	kControllerMaximum = $00007FFF; { +01111111.11111111 }
	kControllerMinimum = $FFFF8000; { -10000000.00000000 }

type
	SynthesizerDescription = record
		synthesizerType: OSType;        { synthesizer type (must be same as component subtype) }
		name: Str31;                   { text name of synthesizer type }
		flags: UNSIGNEDLONG;                  { from the above enum }
		voiceCount: UNSIGNEDLONG;             { maximum polyphony }

		partCount: UNSIGNEDLONG;              { maximum multi-timbrality (and midi channels) }
		instrumentCount: UNSIGNEDLONG;        { non gm, built in (rom) instruments only }
		modifiableInstrumentCount: UNSIGNEDLONG; { plus n-more are user modifiable }
		channelMask: UNSIGNEDLONG;            { (midi device only) which channels device always uses }

		drumPartCount: UNSIGNEDLONG;          { maximum multi-timbrality of drum parts }
		drumCount: UNSIGNEDLONG;              { non gm, built in (rom) drumkits only }
		modifiableDrumCount: UNSIGNEDLONG;    { plus n-more are user modifiable }
		drumChannelMask: UNSIGNEDLONG;        { (midi device only) which channels device always uses }

		outputCount: UNSIGNEDLONG;            { number of audio outputs (usually two) }
		latency: UNSIGNEDLONG;                { response time in µSec }

		controllers: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmInstruments: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
		gmDrums: array [0..3] of UNSIGNEDLONG;					{  array of 128 bits  }
	end;
const
<<<<<<< HEAD
	kVoiceCountDynamic			= -1;							{  constant to use to specify dynamic voicing  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kVoiceCountDynamic = -1;    { constant to use to specify dynamic voicing }
>>>>>>> origin/fixes_2.4


type
	ToneDescriptionPtr = ^ToneDescription;
	ToneDescription = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		synthesizerType: BigEndianOSType;        { synthesizer type }
		synthesizerName: Str31;        { name of instantiation of synth }
		instrumentName: Str31;         { preferred name for human use }
		instrumentNumber: BigEndianLong;       { inst-number used if synth-name matches }
		gmNumber: BigEndianLong;               { Best matching general MIDI number }
<<<<<<< HEAD
	end;
const
	kFirstGMInstrument = $00000001;
	kLastGMInstrument = $00000080;
	kFirstGSInstrument = $00000081;
	kLastGSInstrument = $00003FFF;
	kFirstDrumkit = $00004000; { (first value is "no drum". instrument numbers from 16384->16384+128 are drumkits, and for GM they are _defined_ drumkits! }
	kLastDrumkit = $00004080;
	kFirstROMInstrument = $00008000;
	kLastROMInstrument = $0000FFFF;
	kFirstUserInstrument = $00010000;
	kLastUserInstrument = $0001FFFF;

{ InstrumentMatch}
const
	kInstrumentMatchSynthesizerType = 1;
	kInstrumentMatchSynthesizerName = 2;
	kInstrumentMatchName = 4;
	kInstrumentMatchNumber = 8;
	kInstrumentMatchGMNumber = 16;
	kInstrumentMatchGSNumber = 32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ KnobFlags}
const
	kKnobBasic = 8;    { knob shows up in certain simplified lists of knobs }
	kKnobReadOnly = 16;   { knob value cannot be changed by user or with a SetKnob call }
	kKnobInterruptUnsafe = 32;   { only alter this knob from foreground task time (may access toolbox) }
	kKnobKeyrangeOverride = 64;   { knob can be overridden within a single keyrange (software synth only) }
	kKnobGroupStart = 128;  { knob is first in some logical group of knobs }
	kKnobFixedPoint8 = 1024;
	kKnobFixedPoint16 = 2048; { One of these may be used at a time. }
	kKnobTypeNumber = 0 shl 12;
	kKnobTypeGroupName = 1 shl 12; { "knob" is really a group name for display purposes }
	kKnobTypeBoolean = 2 shl 12; { if range is greater than 1, its a multi-checkbox field }
	kKnobTypeNote = 3 shl 12; { knob range is equivalent to MIDI keys }
	kKnobTypePan = 4 shl 12; { range goes left/right (lose this? ) }
	kKnobTypeInstrument = 5 shl 12; { knob value = reference to another instrument number }
	kKnobTypeSetting = 6 shl 12; { knob value is 1 of n different things (eg, fm algorithms) popup menu }
	kKnobTypeMilliseconds = 7 shl 12; { knob is a millisecond time range }
	kKnobTypePercentage = 8 shl 12; { knob range is displayed as a Percentage }
	kKnobTypeHertz = 9 shl 12; { knob represents frequency }
	kKnobTypeButton = 10 shl 12; { momentary trigger push button }


<<<<<<< HEAD
<<<<<<< HEAD
=======

{ KnobFlags}
const
	kKnobBasic = 8;    { knob shows up in certain simplified lists of knobs }
	kKnobReadOnly = 16;   { knob value cannot be changed by user or with a SetKnob call }
	kKnobInterruptUnsafe = 32;   { only alter this knob from foreground task time (may access toolbox) }
	kKnobKeyrangeOverride = 64;   { knob can be overridden within a single keyrange (software synth only) }
	kKnobGroupStart = 128;  { knob is first in some logical group of knobs }
	kKnobFixedPoint8 = 1024;
	kKnobFixedPoint16 = 2048; { One of these may be used at a time. }
	kKnobTypeNumber = 0 shl 12;
	kKnobTypeGroupName = 1 shl 12; { "knob" is really a group name for display purposes }
	kKnobTypeBoolean = 2 shl 12; { if range is greater than 1, its a multi-checkbox field }
	kKnobTypeNote = 3 shl 12; { knob range is equivalent to MIDI keys }
	kKnobTypePan = 4 shl 12; { range goes left/right (lose this? ) }
	kKnobTypeInstrument = 5 shl 12; { knob value = reference to another instrument number }
	kKnobTypeSetting = 6 shl 12; { knob value is 1 of n different things (eg, fm algorithms) popup menu }
	kKnobTypeMilliseconds = 7 shl 12; { knob is a millisecond time range }
	kKnobTypePercentage = 8 shl 12; { knob range is displayed as a Percentage }
	kKnobTypeHertz = 9 shl 12; { knob represents frequency }
	kKnobTypeButton = 10 shl 12; { momentary trigger push button }


>>>>>>> graemeg/cpstrnew
=======

{ KnobFlags}
const
	kKnobBasic = 8;    { knob shows up in certain simplified lists of knobs }
	kKnobReadOnly = 16;   { knob value cannot be changed by user or with a SetKnob call }
	kKnobInterruptUnsafe = 32;   { only alter this knob from foreground task time (may access toolbox) }
	kKnobKeyrangeOverride = 64;   { knob can be overridden within a single keyrange (software synth only) }
	kKnobGroupStart = 128;  { knob is first in some logical group of knobs }
	kKnobFixedPoint8 = 1024;
	kKnobFixedPoint16 = 2048; { One of these may be used at a time. }
	kKnobTypeNumber = 0 shl 12;
	kKnobTypeGroupName = 1 shl 12; { "knob" is really a group name for display purposes }
	kKnobTypeBoolean = 2 shl 12; { if range is greater than 1, its a multi-checkbox field }
	kKnobTypeNote = 3 shl 12; { knob range is equivalent to MIDI keys }
	kKnobTypePan = 4 shl 12; { range goes left/right (lose this? ) }
	kKnobTypeInstrument = 5 shl 12; { knob value = reference to another instrument number }
	kKnobTypeSetting = 6 shl 12; { knob value is 1 of n different things (eg, fm algorithms) popup menu }
	kKnobTypeMilliseconds = 7 shl 12; { knob is a millisecond time range }
	kKnobTypePercentage = 8 shl 12; { knob range is displayed as a Percentage }
	kKnobTypeHertz = 9 shl 12; { knob represents frequency }
	kKnobTypeButton = 10 shl 12; { momentary trigger push button }


>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
	kUnknownKnobValue = $7FFFFFFF; { a knob with this value means, we don't know it. }
	kDefaultKnobValue = $7FFFFFFE; { used to SET a knob to its default value. }
=======
=======
>>>>>>> origin/fixes_2_2
		synthesizerType:		BigEndianOSType;						{  synthesizer type  }
		synthesizerName:		Str31;									{  name of instantiation of synth  }
		instrumentName:			Str31;									{  preferred name for human use  }
		instrumentNumber:		BigEndianLong;							{  inst-number used if synth-name matches  }
		gmNumber:				BigEndianLong;							{  Best matching general MIDI number  }
=======
>>>>>>> origin/fixes_2.4
	end;
const
	kFirstGMInstrument = $00000001;
	kLastGMInstrument = $00000080;
	kFirstGSInstrument = $00000081;
	kLastGSInstrument = $00003FFF;
	kFirstDrumkit = $00004000; { (first value is "no drum". instrument numbers from 16384->16384+128 are drumkits, and for GM they are _defined_ drumkits! }
	kLastDrumkit = $00004080;
	kFirstROMInstrument = $00008000;
	kLastROMInstrument = $0000FFFF;
	kFirstUserInstrument = $00010000;
	kLastUserInstrument = $0001FFFF;

{ InstrumentMatch}
const
	kInstrumentMatchSynthesizerType = 1;
	kInstrumentMatchSynthesizerName = 2;
	kInstrumentMatchName = 4;
	kInstrumentMatchNumber = 8;
	kInstrumentMatchGMNumber = 16;
	kInstrumentMatchGSNumber = 32;

{ KnobFlags}
const
	kKnobBasic = 8;    { knob shows up in certain simplified lists of knobs }
	kKnobReadOnly = 16;   { knob value cannot be changed by user or with a SetKnob call }
	kKnobInterruptUnsafe = 32;   { only alter this knob from foreground task time (may access toolbox) }
	kKnobKeyrangeOverride = 64;   { knob can be overridden within a single keyrange (software synth only) }
	kKnobGroupStart = 128;  { knob is first in some logical group of knobs }
	kKnobFixedPoint8 = 1024;
	kKnobFixedPoint16 = 2048; { One of these may be used at a time. }
	kKnobTypeNumber = 0 shl 12;
	kKnobTypeGroupName = 1 shl 12; { "knob" is really a group name for display purposes }
	kKnobTypeBoolean = 2 shl 12; { if range is greater than 1, its a multi-checkbox field }
	kKnobTypeNote = 3 shl 12; { knob range is equivalent to MIDI keys }
	kKnobTypePan = 4 shl 12; { range goes left/right (lose this? ) }
	kKnobTypeInstrument = 5 shl 12; { knob value = reference to another instrument number }
	kKnobTypeSetting = 6 shl 12; { knob value is 1 of n different things (eg, fm algorithms) popup menu }
	kKnobTypeMilliseconds = 7 shl 12; { knob is a millisecond time range }
	kKnobTypePercentage = 8 shl 12; { knob range is displayed as a Percentage }
	kKnobTypeHertz = 9 shl 12; { knob represents frequency }
	kKnobTypeButton = 10 shl 12; { momentary trigger push button }


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kUnknownKnobValue = $7FFFFFFF; { a knob with this value means, we don't know it. }
	kDefaultKnobValue = $7FFFFFFE; { used to SET a knob to its default value. }
>>>>>>> origin/fixes_2.4

type
	KnobDescriptionPtr = ^KnobDescription;
	KnobDescription = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		name: Str63;
		lowValue: SIGNEDLONG;
		highValue: SIGNEDLONG;
		defaultValue: SIGNEDLONG;           { a default instrument is made of all default values }
		flags: SIGNEDLONG;
		knobID: SIGNEDLONG;
	end;
type
	GCInstrumentData = record
		tone: ToneDescription;
		knobCount: SIGNEDLONG;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		knob: array [0..1-1] of SIGNEDLONG;
=======
		knob: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		knob: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		knob: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		knob: array [0..0] of SInt32;
>>>>>>> origin/cpstrnew
=======
		knob: array [0..0] of SInt32;
>>>>>>> origin/fixes_2.4
	end;
	GCInstrumentDataPtr = ^GCInstrumentData;
type
	GCInstrumentDataHandle = ^GCInstrumentDataPtr;
	InstrumentAboutInfoPtr = ^InstrumentAboutInfo;
	InstrumentAboutInfo = record
		p: PicHandle;
		author: Str255;
		copyright: Str255;
		other: Str255;
	end;

const
  notImplementedMusicErr = $8000F7E9;
  cantSendToSynthesizerErr = $8000F7E8;
  cantReceiveFromSynthesizerErr = $8000F7E7;
  illegalVoiceAllocationErr = $8000F7E6;
  illegalPartErr = $8000F7E5;
  illegalChannelErr = $8000F7E4;
  illegalKnobErr = $8000F7E3;
  illegalKnobValueErr = $8000F7E2;
  illegalInstrumentErr = $8000F7E1;
  illegalControllerErr = $8000F7E0;
  midiManagerAbsentErr = $8000F7DF;
  synthesizerNotRespondingErr = $8000F7DE;
  synthesizerErr = $8000F7DD;
  illegalNoteChannelErr = $8000F7DC;
  noteChannelNotAllocatedErr = $8000F7DB;
  tunePlayerFullErr = $8000F7DA;
  tuneParseErr = $8000F7D9;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

const
	kGetAtomicInstNoExpandedSamples = 1 shl 0;
	kGetAtomicInstNoOriginalSamples = 1 shl 1;
	kGetAtomicInstNoSamples = kGetAtomicInstNoExpandedSamples or kGetAtomicInstNoOriginalSamples;
	kGetAtomicInstNoKnobList = 1 shl 2;
	kGetAtomicInstNoInstrumentInfo = 1 shl 3;
	kGetAtomicInstOriginalKnobList = 1 shl 4;
	kGetAtomicInstAllKnobs = 1 shl 5; { return even those that are set to default}

{
   For non-gm instruments, instrument number of tone description == 0
   If you want to speed up while running, slam the inst num with what Get instrument number returns
   All missing knobs are slammed to the default value
}
const
	kSetAtomicInstKeepOriginalInstrument = 1 shl 0;
	kSetAtomicInstShareAcrossParts = 1 shl 1; { inst disappears when app goes away}
	kSetAtomicInstCallerTosses = 1 shl 2; { the caller isn't keeping a copy around (for NASetAtomicInstrument)}
	kSetAtomicInstCallerGuarantees = 1 shl 3; { the caller guarantees a copy is around}
	kSetAtomicInstInterruptSafe = 1 shl 4; { dont move memory at this time (but process at next task time)}
	kSetAtomicInstDontPreprocess = 1 shl 7; { perform no further preprocessing because either 1)you know the instrument is digitally clean, or 2) you got it from a GetPartAtomic}

const
	kInstrumentNamesModifiable = 1;
	kInstrumentNamesBoth = 2;

{
 * Structures specific to the GenericMusicComponent
 }
<<<<<<< HEAD

=======

const
	kGetAtomicInstNoExpandedSamples = 1 shl 0;
	kGetAtomicInstNoOriginalSamples = 1 shl 1;
	kGetAtomicInstNoSamples = kGetAtomicInstNoExpandedSamples or kGetAtomicInstNoOriginalSamples;
	kGetAtomicInstNoKnobList = 1 shl 2;
	kGetAtomicInstNoInstrumentInfo = 1 shl 3;
	kGetAtomicInstOriginalKnobList = 1 shl 4;
	kGetAtomicInstAllKnobs = 1 shl 5; { return even those that are set to default}

{
   For non-gm instruments, instrument number of tone description == 0
   If you want to speed up while running, slam the inst num with what Get instrument number returns
   All missing knobs are slammed to the default value
}
const
	kSetAtomicInstKeepOriginalInstrument = 1 shl 0;
	kSetAtomicInstShareAcrossParts = 1 shl 1; { inst disappears when app goes away}
	kSetAtomicInstCallerTosses = 1 shl 2; { the caller isn't keeping a copy around (for NASetAtomicInstrument)}
	kSetAtomicInstCallerGuarantees = 1 shl 3; { the caller guarantees a copy is around}
	kSetAtomicInstInterruptSafe = 1 shl 4; { dont move memory at this time (but process at next task time)}
	kSetAtomicInstDontPreprocess = 1 shl 7; { perform no further preprocessing because either 1)you know the instrument is digitally clean, or 2) you got it from a GetPartAtomic}

const
	kInstrumentNamesModifiable = 1;
	kInstrumentNamesBoth = 2;

{
 * Structures specific to the GenericMusicComponent
 }

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
const
	kGenericMusicComponentSubtype = FourCharCode('gene');

type
	GenericKnobDescriptionPtr = ^GenericKnobDescription;
	GenericKnobDescription = record
		kd: KnobDescription;
		hw1: SIGNEDLONG;                    { driver defined }
		hw2: SIGNEDLONG;                    { driver defined }
		hw3: SIGNEDLONG;                    { driver defined }
		settingsID: SIGNEDLONG;             { resource ID list for boolean and popup names }
	end;
type
	GenericKnobDescriptionList = record
		knobCount: SIGNEDLONG;
		knob: array [0..0] of GenericKnobDescription;
	end;
	GenericKnobDescriptionListPtr = ^GenericKnobDescriptionList;
type
	GenericKnobDescriptionListHandle = ^GenericKnobDescriptionListPtr;
{ knobTypes for MusicDerivedSetKnob }
const
	kGenericMusicKnob = 1;
	kGenericMusicInstrumentKnob = 2;
	kGenericMusicDrumKnob = 3;
	kGenericMusicGlobalController = 4;


const
	kGenericMusicResFirst = 0;
	kGenericMusicResMiscStringList = 1;   { STR# 1: synth name, 2:about author,3:aboutcopyright,4:aboutother }
	kGenericMusicResMiscLongList = 2;    { Long various params, see list below }
	kGenericMusicResInstrumentList = 3;   { NmLs of names and shorts, categories prefixed by '¥¥' }
	kGenericMusicResDrumList = 4;    { NmLs of names and shorts }
	kGenericMusicResInstrumentKnobDescriptionList = 5; { Knob }
	kGenericMusicResDrumKnobDescriptionList = 6; { Knob }
	kGenericMusicResKnobDescriptionList = 7; { Knob }
	kGenericMusicResBitsLongList = 8;    { Long back to back bitmaps of controllers, gminstruments, and drums }
	kGenericMusicResModifiableInstrumentHW = 9; { Shrt same as the hw shorts trailing the instrument names, a shortlist }
	kGenericMusicResGMTranslation = 10;   { Long 128 long entries, 1 for each gm inst, of local instrument numbers 1-n (not hw numbers) }
	kGenericMusicResROMInstrumentData = 11; { knob lists for ROM instruments, so the knob values may be known }
	kGenericMusicResAboutPICT = 12;   { picture for aboutlist. must be present for GetAbout call to work }
	kGenericMusicResLast = 13;

{ elements of the misc long list }
const
	kGenericMusicMiscLongFirst = 0;
=======
=======
>>>>>>> origin/fixes_2_2
		name:					Str63;
		lowValue:				SInt32;
		highValue:				SInt32;
		defaultValue:			SInt32;								{  a default instrument is made of all default values  }
		flags:					SInt32;
		knobID:					SInt32;
	end;

	GCInstrumentDataPtr = ^GCInstrumentData;
	GCInstrumentData = record
		tone:					ToneDescription;
		knobCount:				SInt32;
		knob:					array [0..0] of SInt32;
	end;

	GCInstrumentDataHandle				= ^GCInstrumentDataPtr;
	InstrumentAboutInfoPtr = ^InstrumentAboutInfo;
	InstrumentAboutInfo = record
		p:						PicHandle;
		author:					Str255;
		copyright:				Str255;
		other:					Str255;
	end;


const
	notImplementedMusicErr		= $8000F7E9;
	cantSendToSynthesizerErr	= $8000F7E8;
	cantReceiveFromSynthesizerErr = $8000F7E7;
	illegalVoiceAllocationErr	= $8000F7E6;
	illegalPartErr				= $8000F7E5;
	illegalChannelErr			= $8000F7E4;
	illegalKnobErr				= $8000F7E3;
	illegalKnobValueErr			= $8000F7E2;
	illegalInstrumentErr		= $8000F7E1;
	illegalControllerErr		= $8000F7E0;
	midiManagerAbsentErr		= $8000F7DF;
	synthesizerNotRespondingErr	= $8000F7DE;
	synthesizerErr				= $8000F7DD;
	illegalNoteChannelErr		= $8000F7DC;
	noteChannelNotAllocatedErr	= $8000F7DB;
	tunePlayerFullErr			= $8000F7DA;
	tuneParseErr				= $8000F7D9;

	kGetAtomicInstNoExpandedSamples = $01;
	kGetAtomicInstNoOriginalSamples = $02;
	kGetAtomicInstNoSamples		= $03;
	kGetAtomicInstNoKnobList	= $04;
	kGetAtomicInstNoInstrumentInfo = $08;
	kGetAtomicInstOriginalKnobList = $10;
	kGetAtomicInstAllKnobs		= $20;							{  return even those that are set to default }

	{
	   For non-gm instruments, instrument number of tone description == 0
	   If you want to speed up while running, slam the inst num with what Get instrument number returns
	   All missing knobs are slammed to the default value
	}
	kSetAtomicInstKeepOriginalInstrument = $01;
	kSetAtomicInstShareAcrossParts = $02;						{  inst disappears when app goes away }
	kSetAtomicInstCallerTosses	= $04;							{  the caller isn't keeping a copy around (for NASetAtomicInstrument) }
	kSetAtomicInstCallerGuarantees = $08;						{  the caller guarantees a copy is around }
	kSetAtomicInstInterruptSafe	= $10;							{  dont move memory at this time (but process at next task time) }
	kSetAtomicInstDontPreprocess = $80;							{  perform no further preprocessing because either 1)you know the instrument is digitally clean, or 2) you got it from a GetPartAtomic }

	kInstrumentNamesModifiable	= 1;
	kInstrumentNamesBoth		= 2;

	{	
	 * Structures specific to the GenericMusicComponent
	 	}
=======

const
	kGetAtomicInstNoExpandedSamples = 1 shl 0;
	kGetAtomicInstNoOriginalSamples = 1 shl 1;
	kGetAtomicInstNoSamples = kGetAtomicInstNoExpandedSamples or kGetAtomicInstNoOriginalSamples;
	kGetAtomicInstNoKnobList = 1 shl 2;
	kGetAtomicInstNoInstrumentInfo = 1 shl 3;
	kGetAtomicInstOriginalKnobList = 1 shl 4;
	kGetAtomicInstAllKnobs = 1 shl 5; { return even those that are set to default}

{
   For non-gm instruments, instrument number of tone description == 0
   If you want to speed up while running, slam the inst num with what Get instrument number returns
   All missing knobs are slammed to the default value
}
const
	kSetAtomicInstKeepOriginalInstrument = 1 shl 0;
	kSetAtomicInstShareAcrossParts = 1 shl 1; { inst disappears when app goes away}
	kSetAtomicInstCallerTosses = 1 shl 2; { the caller isn't keeping a copy around (for NASetAtomicInstrument)}
	kSetAtomicInstCallerGuarantees = 1 shl 3; { the caller guarantees a copy is around}
	kSetAtomicInstInterruptSafe = 1 shl 4; { dont move memory at this time (but process at next task time)}
	kSetAtomicInstDontPreprocess = 1 shl 7; { perform no further preprocessing because either 1)you know the instrument is digitally clean, or 2) you got it from a GetPartAtomic}

const
	kInstrumentNamesModifiable = 1;
	kInstrumentNamesBoth = 2;
>>>>>>> origin/fixes_2.4

{
 * Structures specific to the GenericMusicComponent
 }

const
	kGenericMusicComponentSubtype = FourCharCode('gene');

type
	GenericKnobDescriptionPtr = ^GenericKnobDescription;
	GenericKnobDescription = record
		kd: KnobDescription;
		hw1: SIGNEDLONG;                    { driver defined }
		hw2: SIGNEDLONG;                    { driver defined }
		hw3: SIGNEDLONG;                    { driver defined }
		settingsID: SIGNEDLONG;             { resource ID list for boolean and popup names }
	end;
type
	GenericKnobDescriptionList = record
		knobCount: SIGNEDLONG;
		knob: array [0..0] of GenericKnobDescription;
	end;
	GenericKnobDescriptionListPtr = ^GenericKnobDescriptionList;
type
	GenericKnobDescriptionListHandle = ^GenericKnobDescriptionListPtr;
{ knobTypes for MusicDerivedSetKnob }
const
	kGenericMusicKnob = 1;
	kGenericMusicInstrumentKnob = 2;
	kGenericMusicDrumKnob = 3;
	kGenericMusicGlobalController = 4;


<<<<<<< HEAD
	kGenericMusicResFirst		= 0;
	kGenericMusicResMiscStringList = 1;							{  STR# 1: synth name, 2:about author,3:aboutcopyright,4:aboutother  }
	kGenericMusicResMiscLongList = 2;							{  Long various params, see list below  }
	kGenericMusicResInstrumentList = 3;							{  NmLs of names and shorts, categories prefixed by '¥¥'  }
	kGenericMusicResDrumList	= 4;							{  NmLs of names and shorts  }
	kGenericMusicResInstrumentKnobDescriptionList = 5;			{  Knob  }
	kGenericMusicResDrumKnobDescriptionList = 6;				{  Knob  }
	kGenericMusicResKnobDescriptionList = 7;					{  Knob  }
	kGenericMusicResBitsLongList = 8;							{  Long back to back bitmaps of controllers, gminstruments, and drums  }
	kGenericMusicResModifiableInstrumentHW = 9;					{  Shrt same as the hw shorts trailing the instrument names, a shortlist  }
	kGenericMusicResGMTranslation = 10;							{  Long 128 long entries, 1 for each gm inst, of local instrument numbers 1-n (not hw numbers)  }
	kGenericMusicResROMInstrumentData = 11;						{  knob lists for ROM instruments, so the knob values may be known  }
	kGenericMusicResAboutPICT	= 12;							{  picture for aboutlist. must be present for GetAbout call to work  }
	kGenericMusicResLast		= 13;

	{	 elements of the misc long list 	}
	kGenericMusicMiscLongFirst	= 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kGenericMusicResFirst = 0;
	kGenericMusicResMiscStringList = 1;   { STR# 1: synth name, 2:about author,3:aboutcopyright,4:aboutother }
	kGenericMusicResMiscLongList = 2;    { Long various params, see list below }
	kGenericMusicResInstrumentList = 3;   { NmLs of names and shorts, categories prefixed by '¥¥' }
	kGenericMusicResDrumList = 4;    { NmLs of names and shorts }
	kGenericMusicResInstrumentKnobDescriptionList = 5; { Knob }
	kGenericMusicResDrumKnobDescriptionList = 6; { Knob }
	kGenericMusicResKnobDescriptionList = 7; { Knob }
	kGenericMusicResBitsLongList = 8;    { Long back to back bitmaps of controllers, gminstruments, and drums }
	kGenericMusicResModifiableInstrumentHW = 9; { Shrt same as the hw shorts trailing the instrument names, a shortlist }
	kGenericMusicResGMTranslation = 10;   { Long 128 long entries, 1 for each gm inst, of local instrument numbers 1-n (not hw numbers) }
	kGenericMusicResROMInstrumentData = 11; { knob lists for ROM instruments, so the knob values may be known }
	kGenericMusicResAboutPICT = 12;   { picture for aboutlist. must be present for GetAbout call to work }
	kGenericMusicResLast = 13;

{ elements of the misc long list }
const
	kGenericMusicMiscLongFirst = 0;
>>>>>>> origin/fixes_2.4
	kGenericMusicMiscLongVoiceCount = 1;
	kGenericMusicMiscLongPartCount = 2;
	kGenericMusicMiscLongModifiableInstrumentCount = 3;
	kGenericMusicMiscLongChannelMask = 4;
	kGenericMusicMiscLongDrumPartCount = 5;
	kGenericMusicMiscLongModifiableDrumCount = 6;
	kGenericMusicMiscLongDrumChannelMask = 7;
	kGenericMusicMiscLongOutputCount = 8;
	kGenericMusicMiscLongLatency = 9;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kGenericMusicMiscLongFlags = 10;
	kGenericMusicMiscLongFirstGMHW = 11;  { number to add to locate GM main instruments }
	kGenericMusicMiscLongFirstGMDrumHW = 12; { number to add to locate GM drumkits }
	kGenericMusicMiscLongFirstUserHW = 13; { First hw number of user instruments (presumed sequential) }
	kGenericMusicMiscLongLast = 14;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	kGenericMusicMiscLongFlags	= 10;
	kGenericMusicMiscLongFirstGMHW = 11;						{  number to add to locate GM main instruments  }
	kGenericMusicMiscLongFirstGMDrumHW = 12;					{  number to add to locate GM drumkits  }
	kGenericMusicMiscLongFirstUserHW = 13;						{  First hw number of user instruments (presumed sequential)  }
	kGenericMusicMiscLongLast	= 14;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

type
	GCPartPtr = ^GCPart;
	GCPart = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		hwInstrumentNumber: SIGNEDLONG;     { internal number of recalled instrument }
		controller: array [0..127] of SInt16;				{  current values for all controllers  }
		volume: SIGNEDLONG;                 { ctrl 7 is special case }
		polyphony: SIGNEDLONG;
		midiChannel: SIGNEDLONG;            { 1-16 if in use }
		id: GCInstrumentData;                     { ToneDescription & knoblist, uncertain length }
	end;
{
 * Calls specific to the GenericMusicComponent
 }
const
	kMusicGenericRange = $0100;
	kMusicDerivedRange = $0200;

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 * Calls specific to the GenericMusicComponent
 }
const
	kMusicGenericRange = $0100;
	kMusicDerivedRange = $0200;

{
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
 * Flags in GenericMusicConfigure call
 }
const
=======
 * Calls specific to the GenericMusicComponent
 }
const
	kMusicGenericRange = $0100;
	kMusicDerivedRange = $0200;

{
 * Flags in GenericMusicConfigure call
 }
const
>>>>>>> origin/fixes_2.4
	kGenericMusicDoMIDI = 1 shl 0; { implement normal MIDI messages for note, controllers, and program changes 0-127 }
	kGenericMusicBank0 = 1 shl 1; { implement instrument bank changes on controller 0 }
	kGenericMusicBank32 = 1 shl 2; { implement instrument bank changes on controller 32 }
	kGenericMusicErsatzMIDI = 1 shl 3; { construct MIDI packets, but send them to the derived component }
	kGenericMusicCallKnobs = 1 shl 4; { call the derived component with special knob format call }
	kGenericMusicCallParts = 1 shl 5; { call the derived component with special part format call }
	kGenericMusicCallInstrument = 1 shl 6; { call MusicDerivedSetInstrument for MusicSetInstrument calls }
	kGenericMusicCallNumber = 1 shl 7; { call MusicDerivedSetPartInstrumentNumber for MusicSetPartInstrumentNumber calls, & don't send any C0 or bank stuff }
	kGenericMusicCallROMInstrument = 1 shl 8; { call MusicSetInstrument for MusicSetPartInstrumentNumber for "ROM" instruments, passing params from the ROMi resource }
	kGenericMusicAllDefaults = 1 shl 9; { indicates that when a new instrument is recalled, all knobs are reset to DEFAULT settings. True for GS modules }


type
	MusicOfflineDataProcPtr = function( SoundData: Ptr; numBytes: SIGNEDLONG; myRefCon: SIGNEDLONG ): ComponentResult;
	MusicOfflineDataUPP = MusicOfflineDataProcPtr;
	OfflineSampleTypePtr = ^OfflineSampleType;
	OfflineSampleType = record
		numChannels: UNSIGNEDLONG;            {number of channels,  ie mono = 1}
		sampleRate: UnsignedFixed;             {sample rate in Apples Fixed point representation}
		sampleSize: UInt16;             {number of bits in sample}
	end;
type
	InstrumentInfoRecordPtr = ^InstrumentInfoRecord;
	InstrumentInfoRecord = record
		instrumentNumber: SIGNEDLONG;       { instrument number (if 0, name is a catagory)}
		flags: SIGNEDLONG;                  { show in picker, etc.}
		toneNameIndex: SIGNEDLONG;          { index in toneNames (1 based)}
		itxtNameAtomID: SIGNEDLONG;         { index in itxtNames (itxt/name by index)}
	end;
type
	InstrumentInfoList = record
		recordCount: SIGNEDLONG;
		toneNames: Handle;              { name from tone description}
		itxtNames: QTAtomContainer;              { itxt/name atoms for instruments}
		info: array [0..0] of InstrumentInfoRecord;
	end;
	InstrumentInfoListPtr = ^InstrumentInfoList;
type
	InstrumentInfoListHandle = ^InstrumentInfoListPtr;
{
 *  MusicGetDescription()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetDescription( mc: MusicComponent; var sd: SynthesizerDescription ): ComponentResult; external name '_MusicGetDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetPart()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPart( mc: MusicComponent; part: SIGNEDLONG; var midiChannel: SIGNEDLONG; var polyphony: SIGNEDLONG ): ComponentResult; external name '_MusicGetPart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetPart()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPart( mc: MusicComponent; part: SIGNEDLONG; midiChannel: SIGNEDLONG; polyphony: SIGNEDLONG ): ComponentResult; external name '_MusicSetPart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetPartInstrumentNumber()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartInstrumentNumber( mc: MusicComponent; part: SIGNEDLONG; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_MusicSetPartInstrumentNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetPartInstrumentNumber()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartInstrumentNumber( mc: MusicComponent; part: SIGNEDLONG ): ComponentResult; external name '_MusicGetPartInstrumentNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicStorePartInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicStorePartInstrument( mc: MusicComponent; part: SIGNEDLONG; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_MusicStorePartInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetPartAtomicInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartAtomicInstrument( mc: MusicComponent; part: SIGNEDLONG; var ai: AtomicInstrument; flags: SIGNEDLONG ): ComponentResult; external name '_MusicGetPartAtomicInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetPartAtomicInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartAtomicInstrument( mc: MusicComponent; part: SIGNEDLONG; aiP: AtomicInstrumentPtr; flags: SIGNEDLONG ): ComponentResult; external name '_MusicSetPartAtomicInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetPartKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartKnob( mc: MusicComponent; part: SIGNEDLONG; knobID: SIGNEDLONG ): ComponentResult; external name '_MusicGetPartKnob';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetPartKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartKnob( mc: MusicComponent; part: SIGNEDLONG; knobID: SIGNEDLONG; knobValue: SIGNEDLONG ): ComponentResult; external name '_MusicSetPartKnob';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetKnob( mc: MusicComponent; knobID: SIGNEDLONG ): ComponentResult; external name '_MusicGetKnob';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetKnob( mc: MusicComponent; knobID: SIGNEDLONG; knobValue: SIGNEDLONG ): ComponentResult; external name '_MusicSetKnob';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetPartName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartName( mc: MusicComponent; part: SIGNEDLONG; name: StringPtr ): ComponentResult; external name '_MusicGetPartName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetPartName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartName( mc: MusicComponent; part: SIGNEDLONG; name: StringPtr ): ComponentResult; external name '_MusicSetPartName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicFindTone()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicFindTone( mc: MusicComponent; var td: ToneDescription; var libraryIndexOut: SIGNEDLONG; fit: UNSIGNEDLONGPtr ): ComponentResult; external name '_MusicFindTone';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicPlayNote()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicPlayNote( mc: MusicComponent; part: SIGNEDLONG; pitch: SIGNEDLONG; velocity: SIGNEDLONG ): ComponentResult; external name '_MusicPlayNote';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicResetPart()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicResetPart( mc: MusicComponent; part: SIGNEDLONG ): ComponentResult; external name '_MusicResetPart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetPartController()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartController( mc: MusicComponent; part: SIGNEDLONG; controllerNumber: MusicController; controllerValue: SIGNEDLONG ): ComponentResult; external name '_MusicSetPartController';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetPartController()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartController( mc: MusicComponent; part: SIGNEDLONG; controllerNumber: MusicController ): ComponentResult; external name '_MusicGetPartController';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetMIDIProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetMIDIProc( mc: MusicComponent; var midiSendProc: MusicMIDISendUPP; var refCon: SIGNEDLONG ): ComponentResult; external name '_MusicGetMIDIProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetMIDIProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetMIDIProc( mc: MusicComponent; midiSendProc: MusicMIDISendUPP; refCon: SIGNEDLONG ): ComponentResult; external name '_MusicSetMIDIProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetInstrumentNames()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInstrumentNames( mc: MusicComponent; modifiableInstruments: SIGNEDLONG; var instrumentNames: Handle; var instrumentCategoryLasts: Handle; var instrumentCategoryNames: Handle ): ComponentResult; external name '_MusicGetInstrumentNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetDrumNames()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetDrumNames( mc: MusicComponent; modifiableInstruments: SIGNEDLONG; var instrumentNumbers: Handle; var instrumentNames: Handle ): ComponentResult; external name '_MusicGetDrumNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetMasterTune()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetMasterTune( mc: MusicComponent ): ComponentResult; external name '_MusicGetMasterTune';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetMasterTune()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetMasterTune( mc: MusicComponent; masterTune: SIGNEDLONG ): ComponentResult; external name '_MusicSetMasterTune';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetInstrumentAboutInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInstrumentAboutInfo( mc: MusicComponent; part: SIGNEDLONG; var iai: InstrumentAboutInfo ): ComponentResult; external name '_MusicGetInstrumentAboutInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetDeviceConnection()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetDeviceConnection( mc: MusicComponent; index: SIGNEDLONG; var id1: SIGNEDLONG; var id2: SIGNEDLONG ): ComponentResult; external name '_MusicGetDeviceConnection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicUseDeviceConnection()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicUseDeviceConnection( mc: MusicComponent; id1: SIGNEDLONG; id2: SIGNEDLONG ): ComponentResult; external name '_MusicUseDeviceConnection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetKnobSettingStrings()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetKnobSettingStrings( mc: MusicComponent; knobIndex: SIGNEDLONG; isGlobal: SIGNEDLONG; var settingsNames: Handle; var settingsCategoryLasts: Handle; var settingsCategoryNames: Handle ): ComponentResult; external name '_MusicGetKnobSettingStrings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetMIDIPorts()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetMIDIPorts( mc: MusicComponent; var inputPortCount: SIGNEDLONG; var outputPortCount: SIGNEDLONG ): ComponentResult; external name '_MusicGetMIDIPorts';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSendMIDI()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSendMIDI( mc: MusicComponent; portIndex: SIGNEDLONG; var mp: MusicMIDIPacket ): ComponentResult; external name '_MusicSendMIDI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicStartOffline()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicStartOffline( mc: MusicComponent; numChannels: UNSIGNEDLONGPtr; var sampleRate: UnsignedFixed; sampleSize: UInt16Ptr; dataProc: MusicOfflineDataUPP; dataProcRefCon: SIGNEDLONG ): ComponentResult; external name '_MusicStartOffline';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetOfflineTimeTo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetOfflineTimeTo( mc: MusicComponent; newTimeStamp: SIGNEDLONG ): ComponentResult; external name '_MusicSetOfflineTimeTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetInstrumentKnobDescription()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInstrumentKnobDescription( mc: MusicComponent; knobIndex: SIGNEDLONG; var mkd: KnobDescription ): ComponentResult; external name '_MusicGetInstrumentKnobDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetDrumKnobDescription()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetDrumKnobDescription( mc: MusicComponent; knobIndex: SIGNEDLONG; var mkd: KnobDescription ): ComponentResult; external name '_MusicGetDrumKnobDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetKnobDescription()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetKnobDescription( mc: MusicComponent; knobIndex: SIGNEDLONG; var mkd: KnobDescription ): ComponentResult; external name '_MusicGetKnobDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGetInfoText()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInfoText( mc: MusicComponent; selector: SIGNEDLONG; var textH: Handle; var styleH: Handle ): ComponentResult; external name '_MusicGetInfoText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


const
	kGetInstrumentInfoNoBuiltIn = 1 shl 0;
	kGetInstrumentInfoMidiUserInst = 1 shl 1;
	kGetInstrumentInfoNoIText = 1 shl 2;

{
 *  MusicGetInstrumentInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInstrumentInfo( mc: MusicComponent; getInstrumentInfoFlags: SIGNEDLONG; var infoListH: InstrumentInfoListHandle ): ComponentResult; external name '_MusicGetInstrumentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicTask()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicTask( mc: MusicComponent ): ComponentResult; external name '_MusicTask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetPartInstrumentNumberInterruptSafe()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartInstrumentNumberInterruptSafe( mc: MusicComponent; part: SIGNEDLONG; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_MusicSetPartInstrumentNumberInterruptSafe';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicSetPartSoundLocalization()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartSoundLocalization( mc: MusicComponent; part: SIGNEDLONG; data: Handle ): ComponentResult; external name '_MusicSetPartSoundLocalization';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGenericConfigure()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGenericConfigure( mc: MusicComponent; mode: SIGNEDLONG; flags: SIGNEDLONG; baseResID: SIGNEDLONG ): ComponentResult; external name '_MusicGenericConfigure';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGenericGetPart()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGenericGetPart( mc: MusicComponent; partNumber: SIGNEDLONG; var part: GCPartPtr ): ComponentResult; external name '_MusicGenericGetPart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGenericGetKnobList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGenericGetKnobList( mc: MusicComponent; knobType: SIGNEDLONG; var gkdlH: GenericKnobDescriptionListHandle ): ComponentResult; external name '_MusicGenericGetKnobList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicGenericSetResourceNumbers()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGenericSetResourceNumbers( mc: MusicComponent; resourceIDH: Handle ): ComponentResult; external name '_MusicGenericSetResourceNumbers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedMIDISend()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedMIDISend( mc: MusicComponent; var packet: MusicMIDIPacket ): ComponentResult; external name '_MusicDerivedMIDISend';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedSetKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
=======
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedSetKnob( mc: MusicComponent; knobType: SIGNEDLONG; knobNumber: SIGNEDLONG; knobValue: SIGNEDLONG; partNumber: SIGNEDLONG; var p: GCPart; var gkd: GenericKnobDescription ): ComponentResult; external name '_MusicDerivedSetKnob';
=======
function MusicDerivedMIDISend( mc: MusicComponent; var packet: MusicMIDIPacket ): ComponentResult; external name '_MusicDerivedMIDISend';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  MusicDerivedSetPart()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedSetPart( mc: MusicComponent; partNumber: SIGNEDLONG; var p: GCPart ): ComponentResult; external name '_MusicDerivedSetPart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedSetInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedSetInstrument( mc: MusicComponent; partNumber: SIGNEDLONG; var p: GCPart ): ComponentResult; external name '_MusicDerivedSetInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedSetPartInstrumentNumber()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedSetPartInstrumentNumber( mc: MusicComponent; partNumber: SIGNEDLONG; var p: GCPart ): ComponentResult; external name '_MusicDerivedSetPartInstrumentNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedSetMIDI()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedSetMIDI( mc: MusicComponent; midiProc: MusicMIDISendUPP; refcon: SIGNEDLONG; midiChannel: SIGNEDLONG ): ComponentResult; external name '_MusicDerivedSetMIDI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedStorePartInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedStorePartInstrument( mc: MusicComponent; partNumber: SIGNEDLONG; var p: GCPart; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_MusicDerivedStorePartInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedOpenResFile()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedOpenResFile( mc: MusicComponent ): ComponentResult; external name '_MusicDerivedOpenResFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedCloseResFile()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedCloseResFile( mc: MusicComponent; resRefNum: SInt16 ): ComponentResult; external name '_MusicDerivedCloseResFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{--------------------------
    Types
--------------------------}
const
	kNoteRequestNoGM = 1;    { don't degrade to a GM synth }
	kNoteRequestNoSynthType = 2;    { don't degrade to another synth of same type but different name }
	kNoteRequestSynthMustMatch = 4;     { synthType must be a match, including kGMSynthComponentSubType }


const
	kNoteRequestSpecifyMIDIChannel = $80;

type
	NoteAllocator = ComponentInstance;
{
    The midiChannelAssignment field of this structure is used to assign a MIDI channel 
    when a NoteChannel is created from a NoteRequest.
    A value of 0 indicates a MIDI channel has *not* been assigned
    A value of (kNoteRequestSpecifyMIDIChannel | 1->16) is a MIDI channel assignment

    This field requires QuickTime 5.0 or later and should be set to 0 for prior versions.
}
type
	NoteRequestMIDIChannel = UInt8;
	NoteRequestInfoPtr = ^NoteRequestInfo;
	NoteRequestInfo = record
		flags: UInt8;                  { 1: dont accept GM match, 2: dont accept same-synth-type match }
		midiChannelAssignment: NoteRequestMIDIChannel; { (kNoteRequestSpecifyMIDIChannel | 1->16) as MIDI Channel assignement or zero - see notes above  }
		polyphony: BigEndianShort;              { Maximum number of voices }
		typicalPolyphony: BigEndianFixed;       { Hint for level mixing }
	end;
type
	NoteRequestPtr = ^NoteRequest;
	NoteRequest = record
		info: NoteRequestInfo;
		tone: ToneDescription;
	end;

type
	NoteChannel = SIGNEDLONG;

const
	kPickDontMix = 1;    { dont mix instruments with drum sounds }
	kPickSameSynth = 2;    { only allow the same device that went in, to come out }
	kPickUserInsts = 4;    { show user insts in addition to ROM voices }
	kPickEditAllowEdit = 8;    { lets user switch over to edit mode }
	kPickEditAllowPick = 16;   { lets the user switch over to pick mode }
	kPickEditSynthGlobal = 32;   { edit the global knobs of the synth }
	kPickEditControllers = 64;    { edit the controllers of the notechannel }


const
	kNoteAllocatorComponentType = FourCharCode('nota');


{--------------------------------
    Note Allocator Prototypes
--------------------------------}
{
 *  NARegisterMusicDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NARegisterMusicDevice( na: NoteAllocator; synthType: OSType; var name: Str31; var connections: SynthesizerConnections ): ComponentResult; external name '_NARegisterMusicDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAUnregisterMusicDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAUnregisterMusicDevice( na: NoteAllocator; index: SIGNEDLONG ): ComponentResult; external name '_NAUnregisterMusicDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetRegisteredMusicDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetRegisteredMusicDevice( na: NoteAllocator; index: SIGNEDLONG; var synthType: OSType; name: Str31; var connections: SynthesizerConnections; var mc: MusicComponent ): ComponentResult; external name '_NAGetRegisteredMusicDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASaveMusicConfiguration()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASaveMusicConfiguration( na: NoteAllocator ): ComponentResult; external name '_NASaveMusicConfiguration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NANewNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NANewNoteChannel( na: NoteAllocator; var noteRequest_: NoteRequest; var outChannel: NoteChannel ): ComponentResult; external name '_NANewNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NADisposeNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NADisposeNoteChannel( na: NoteAllocator; noteChannel_: NoteChannel ): ComponentResult; external name '_NADisposeNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetNoteChannelInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetNoteChannelInfo( na: NoteAllocator; noteChannel_: NoteChannel; var index: SIGNEDLONG; var part: SIGNEDLONG ): ComponentResult; external name '_NAGetNoteChannelInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPrerollNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPrerollNoteChannel( na: NoteAllocator; noteChannel_: NoteChannel ): ComponentResult; external name '_NAPrerollNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAUnrollNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAUnrollNoteChannel( na: NoteAllocator; noteChannel_: NoteChannel ): ComponentResult; external name '_NAUnrollNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetNoteChannelVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetNoteChannelVolume( na: NoteAllocator; noteChannel_: NoteChannel; volume: Fixed ): ComponentResult; external name '_NASetNoteChannelVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAResetNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAResetNoteChannel( na: NoteAllocator; noteChannel_: NoteChannel ): ComponentResult; external name '_NAResetNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPlayNote()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPlayNote( na: NoteAllocator; noteChannel_: NoteChannel; pitch: SIGNEDLONG; velocity: SIGNEDLONG ): ComponentResult; external name '_NAPlayNote';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetController()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetController( na: NoteAllocator; noteChannel_: NoteChannel; controllerNumber: SIGNEDLONG; controllerValue: SIGNEDLONG ): ComponentResult; external name '_NASetController';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetKnob( na: NoteAllocator; noteChannel_: NoteChannel; knobNumber: SIGNEDLONG; knobValue: SIGNEDLONG ): ComponentResult; external name '_NASetKnob';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAFindNoteChannelTone()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAFindNoteChannelTone( na: NoteAllocator; noteChannel_: NoteChannel; var td: ToneDescription; var instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_NAFindNoteChannelTone';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetInstrumentNumber()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetInstrumentNumber( na: NoteAllocator; noteChannel_: NoteChannel; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_NASetInstrumentNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPickInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPickInstrument( na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; var sd: ToneDescription; flags: UNSIGNEDLONG; refCon: SIGNEDLONG; reserved1: SIGNEDLONG; reserved2: SIGNEDLONG ): ComponentResult; external name '_NAPickInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPickArrangement()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPickArrangement( na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; zero1: SIGNEDLONG; zero2: SIGNEDLONG; t: Track; songName: StringPtr ): ComponentResult; external name '_NAPickArrangement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAStuffToneDescription()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAStuffToneDescription( na: NoteAllocator; gmNumber: SIGNEDLONG; var td: ToneDescription ): ComponentResult; external name '_NAStuffToneDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NACopyrightDialog()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NACopyrightDialog( na: NoteAllocator; p: PicHandle; author: StringPtr; copyright: StringPtr; other: StringPtr; title: StringPtr; filterProc: ModalFilterUPP; refCon: SIGNEDLONG ): ComponentResult; external name '_NACopyrightDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


=======
		hwInstrumentNumber:		SInt32;								{  internal number of recalled instrument  }
		controller:				array [0..127] of SInt16;				{  current values for all controllers  }
		volume:					SInt32;								{  ctrl 7 is special case  }
		polyphony:				SInt32;
		midiChannel:			SInt32;								{  1-16 if in use  }
		id:						GCInstrumentData;						{  ToneDescription & knoblist, uncertain length  }
	end;

	{	
	 * Calls specific to the GenericMusicComponent
	 	}

const
	kMusicGenericRange			= $0100;
	kMusicDerivedRange			= $0200;

	{	
	 * Flags in GenericMusicConfigure call
	 	}
	kGenericMusicDoMIDI			= $01;							{  implement normal MIDI messages for note, controllers, and program changes 0-127  }
	kGenericMusicBank0			= $02;							{  implement instrument bank changes on controller 0  }
	kGenericMusicBank32			= $04;							{  implement instrument bank changes on controller 32  }
	kGenericMusicErsatzMIDI		= $08;							{  construct MIDI packets, but send them to the derived component  }
	kGenericMusicCallKnobs		= $10;							{  call the derived component with special knob format call  }
	kGenericMusicCallParts		= $20;							{  call the derived component with special part format call  }
	kGenericMusicCallInstrument	= $40;							{  call MusicDerivedSetInstrument for MusicSetInstrument calls  }
	kGenericMusicCallNumber		= $80;							{  call MusicDerivedSetPartInstrumentNumber for MusicSetPartInstrumentNumber calls, & don't send any C0 or bank stuff  }
	kGenericMusicCallROMInstrument = $0100;						{  call MusicSetInstrument for MusicSetPartInstrumentNumber for "ROM" instruments, passing params from the ROMi resource  }
	kGenericMusicAllDefaults	= $0200;						{  indicates that when a new instrument is recalled, all knobs are reset to DEFAULT settings. True for GS modules  }


type
{$ifc TYPED_FUNCTION_POINTERS}
	MusicOfflineDataProcPtr = function(SoundData: Ptr; numBytes: SInt32; myRefCon: SInt32): ComponentResult;
{$elsec}
	MusicOfflineDataProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	MusicOfflineDataUPP = ^SInt32; { an opaque UPP }
{$elsec}
	MusicOfflineDataUPP = UniversalProcPtr;
{$endc}	
	OfflineSampleTypePtr = ^OfflineSampleType;
	OfflineSampleType = record
		numChannels:			UInt32;									{ number of channels,  ie mono = 1 }
		sampleRate:				UnsignedFixed;							{ sample rate in Apples Fixed point representation }
		sampleSize:				UInt16;									{ number of bits in sample }
	end;

	InstrumentInfoRecordPtr = ^InstrumentInfoRecord;
	InstrumentInfoRecord = record
		instrumentNumber:		SInt32;								{  instrument number (if 0, name is a catagory) }
		flags:					SInt32;								{  show in picker, etc. }
		toneNameIndex:			SInt32;								{  index in toneNames (1 based) }
		itxtNameAtomID:			SInt32;								{  index in itxtNames (itxt/name by index) }
	end;

	InstrumentInfoListPtr = ^InstrumentInfoList;
	InstrumentInfoList = record
		recordCount:			SInt32;
		toneNames:				Handle;									{  name from tone description }
		itxtNames:				QTAtomContainer;						{  itxt/name atoms for instruments }
		info:					array [0..0] of InstrumentInfoRecord;
	end;

	InstrumentInfoListHandle			= ^InstrumentInfoListPtr;
	{
	 *  MusicGetDescription()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function MusicGetDescription(mc: MusicComponent; var sd: SynthesizerDescription): ComponentResult; external name '_MusicGetDescription';
{
 *  MusicGetPart()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPart(mc: MusicComponent; part: SInt32; var midiChannel: SInt32; var polyphony: SInt32): ComponentResult; external name '_MusicGetPart';
{
 *  MusicSetPart()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPart(mc: MusicComponent; part: SInt32; midiChannel: SInt32; polyphony: SInt32): ComponentResult; external name '_MusicSetPart';
{
 *  MusicSetPartInstrumentNumber()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartInstrumentNumber(mc: MusicComponent; part: SInt32; instrumentNumber: SInt32): ComponentResult; external name '_MusicSetPartInstrumentNumber';
{
 *  MusicGetPartInstrumentNumber()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartInstrumentNumber(mc: MusicComponent; part: SInt32): ComponentResult; external name '_MusicGetPartInstrumentNumber';
{
 *  MusicStorePartInstrument()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicStorePartInstrument(mc: MusicComponent; part: SInt32; instrumentNumber: SInt32): ComponentResult; external name '_MusicStorePartInstrument';
{
 *  MusicGetPartAtomicInstrument()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartAtomicInstrument(mc: MusicComponent; part: SInt32; var ai: AtomicInstrument; flags: SInt32): ComponentResult; external name '_MusicGetPartAtomicInstrument';
{
 *  MusicSetPartAtomicInstrument()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartAtomicInstrument(mc: MusicComponent; part: SInt32; aiP: AtomicInstrumentPtr; flags: SInt32): ComponentResult; external name '_MusicSetPartAtomicInstrument';
{
 *  MusicGetPartKnob()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartKnob(mc: MusicComponent; part: SInt32; knobID: SInt32): ComponentResult; external name '_MusicGetPartKnob';
{
 *  MusicSetPartKnob()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartKnob(mc: MusicComponent; part: SInt32; knobID: SInt32; knobValue: SInt32): ComponentResult; external name '_MusicSetPartKnob';
{
 *  MusicGetKnob()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetKnob(mc: MusicComponent; knobID: SInt32): ComponentResult; external name '_MusicGetKnob';
{
 *  MusicSetKnob()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetKnob(mc: MusicComponent; knobID: SInt32; knobValue: SInt32): ComponentResult; external name '_MusicSetKnob';
{
 *  MusicGetPartName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartName(mc: MusicComponent; part: SInt32; name: StringPtr): ComponentResult; external name '_MusicGetPartName';
{
 *  MusicSetPartName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartName(mc: MusicComponent; part: SInt32; name: StringPtr): ComponentResult; external name '_MusicSetPartName';
{
 *  MusicFindTone()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicFindTone(mc: MusicComponent; var td: ToneDescription; var libraryIndexOut: SInt32; var fit: UInt32): ComponentResult; external name '_MusicFindTone';
{
 *  MusicPlayNote()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicPlayNote(mc: MusicComponent; part: SInt32; pitch: SInt32; velocity: SInt32): ComponentResult; external name '_MusicPlayNote';
{
 *  MusicResetPart()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicResetPart(mc: MusicComponent; part: SInt32): ComponentResult; external name '_MusicResetPart';
{
 *  MusicSetPartController()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartController(mc: MusicComponent; part: SInt32; controllerNumber: MusicController; controllerValue: SInt32): ComponentResult; external name '_MusicSetPartController';
{
 *  MusicGetPartController()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetPartController(mc: MusicComponent; part: SInt32; controllerNumber: MusicController): ComponentResult; external name '_MusicGetPartController';
{
 *  MusicGetMIDIProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetMIDIProc(mc: MusicComponent; var midiSendProc: MusicMIDISendUPP; var refCon: SInt32): ComponentResult; external name '_MusicGetMIDIProc';
{
 *  MusicSetMIDIProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetMIDIProc(mc: MusicComponent; midiSendProc: MusicMIDISendUPP; refCon: SInt32): ComponentResult; external name '_MusicSetMIDIProc';
{
 *  MusicGetInstrumentNames()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInstrumentNames(mc: MusicComponent; modifiableInstruments: SInt32; var instrumentNames: Handle; var instrumentCategoryLasts: Handle; var instrumentCategoryNames: Handle): ComponentResult; external name '_MusicGetInstrumentNames';
{
 *  MusicGetDrumNames()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetDrumNames(mc: MusicComponent; modifiableInstruments: SInt32; var instrumentNumbers: Handle; var instrumentNames: Handle): ComponentResult; external name '_MusicGetDrumNames';
{
 *  MusicGetMasterTune()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetMasterTune(mc: MusicComponent): ComponentResult; external name '_MusicGetMasterTune';
{
 *  MusicSetMasterTune()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetMasterTune(mc: MusicComponent; masterTune: SInt32): ComponentResult; external name '_MusicSetMasterTune';
{
 *  MusicGetInstrumentAboutInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInstrumentAboutInfo(mc: MusicComponent; part: SInt32; var iai: InstrumentAboutInfo): ComponentResult; external name '_MusicGetInstrumentAboutInfo';
{
 *  MusicGetDeviceConnection()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetDeviceConnection(mc: MusicComponent; index: SInt32; var id1: SInt32; var id2: SInt32): ComponentResult; external name '_MusicGetDeviceConnection';
{
 *  MusicUseDeviceConnection()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicUseDeviceConnection(mc: MusicComponent; id1: SInt32; id2: SInt32): ComponentResult; external name '_MusicUseDeviceConnection';
{
 *  MusicGetKnobSettingStrings()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetKnobSettingStrings(mc: MusicComponent; knobIndex: SInt32; isGlobal: SInt32; var settingsNames: Handle; var settingsCategoryLasts: Handle; var settingsCategoryNames: Handle): ComponentResult; external name '_MusicGetKnobSettingStrings';
{
 *  MusicGetMIDIPorts()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetMIDIPorts(mc: MusicComponent; var inputPortCount: SInt32; var outputPortCount: SInt32): ComponentResult; external name '_MusicGetMIDIPorts';
{
 *  MusicSendMIDI()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSendMIDI(mc: MusicComponent; portIndex: SInt32; var mp: MusicMIDIPacket): ComponentResult; external name '_MusicSendMIDI';
{
 *  MusicStartOffline()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicStartOffline(mc: MusicComponent; var numChannels: UInt32; var sampleRate: UnsignedFixed; var sampleSize: UInt16; dataProc: MusicOfflineDataUPP; dataProcRefCon: SInt32): ComponentResult; external name '_MusicStartOffline';
{
 *  MusicSetOfflineTimeTo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetOfflineTimeTo(mc: MusicComponent; newTimeStamp: SInt32): ComponentResult; external name '_MusicSetOfflineTimeTo';
{
 *  MusicGetInstrumentKnobDescription()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInstrumentKnobDescription(mc: MusicComponent; knobIndex: SInt32; var mkd: KnobDescription): ComponentResult; external name '_MusicGetInstrumentKnobDescription';
{
 *  MusicGetDrumKnobDescription()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetDrumKnobDescription(mc: MusicComponent; knobIndex: SInt32; var mkd: KnobDescription): ComponentResult; external name '_MusicGetDrumKnobDescription';
{
 *  MusicGetKnobDescription()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetKnobDescription(mc: MusicComponent; knobIndex: SInt32; var mkd: KnobDescription): ComponentResult; external name '_MusicGetKnobDescription';
{
 *  MusicGetInfoText()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInfoText(mc: MusicComponent; selector: SInt32; var textH: Handle; var styleH: Handle): ComponentResult; external name '_MusicGetInfoText';
const
	kGetInstrumentInfoNoBuiltIn	= $01;
	kGetInstrumentInfoMidiUserInst = $02;
	kGetInstrumentInfoNoIText	= $04;

	{
	 *  MusicGetInstrumentInfo()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function MusicGetInstrumentInfo(mc: MusicComponent; getInstrumentInfoFlags: SInt32; var infoListH: InstrumentInfoListHandle): ComponentResult; external name '_MusicGetInstrumentInfo';
{
 *  MusicTask()
=======
		hwInstrumentNumber:		SInt32;								{  internal number of recalled instrument  }
		controller:				array [0..127] of SInt16;				{  current values for all controllers  }
		volume:					SInt32;								{  ctrl 7 is special case  }
		polyphony:				SInt32;
		midiChannel:			SInt32;								{  1-16 if in use  }
		id:						GCInstrumentData;						{  ToneDescription & knoblist, uncertain length  }
	end;

	{	
	 * Calls specific to the GenericMusicComponent
	 	}

const
	kMusicGenericRange			= $0100;
	kMusicDerivedRange			= $0200;

	{	
	 * Flags in GenericMusicConfigure call
	 	}
	kGenericMusicDoMIDI			= $01;							{  implement normal MIDI messages for note, controllers, and program changes 0-127  }
	kGenericMusicBank0			= $02;							{  implement instrument bank changes on controller 0  }
	kGenericMusicBank32			= $04;							{  implement instrument bank changes on controller 32  }
	kGenericMusicErsatzMIDI		= $08;							{  construct MIDI packets, but send them to the derived component  }
	kGenericMusicCallKnobs		= $10;							{  call the derived component with special knob format call  }
	kGenericMusicCallParts		= $20;							{  call the derived component with special part format call  }
	kGenericMusicCallInstrument	= $40;							{  call MusicDerivedSetInstrument for MusicSetInstrument calls  }
	kGenericMusicCallNumber		= $80;							{  call MusicDerivedSetPartInstrumentNumber for MusicSetPartInstrumentNumber calls, & don't send any C0 or bank stuff  }
	kGenericMusicCallROMInstrument = $0100;						{  call MusicSetInstrument for MusicSetPartInstrumentNumber for "ROM" instruments, passing params from the ROMi resource  }
	kGenericMusicAllDefaults	= $0200;						{  indicates that when a new instrument is recalled, all knobs are reset to DEFAULT settings. True for GS modules  }


type
{$ifc TYPED_FUNCTION_POINTERS}
	MusicOfflineDataProcPtr = function(SoundData: Ptr; numBytes: SInt32; myRefCon: SInt32): ComponentResult;
{$elsec}
	MusicOfflineDataProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	MusicOfflineDataUPP = ^SInt32; { an opaque UPP }
{$elsec}
	MusicOfflineDataUPP = UniversalProcPtr;
{$endc}	
	OfflineSampleTypePtr = ^OfflineSampleType;
	OfflineSampleType = record
		numChannels:			UInt32;									{ number of channels,  ie mono = 1 }
		sampleRate:				UnsignedFixed;							{ sample rate in Apples Fixed point representation }
		sampleSize:				UInt16;									{ number of bits in sample }
	end;

	InstrumentInfoRecordPtr = ^InstrumentInfoRecord;
	InstrumentInfoRecord = record
		instrumentNumber:		SInt32;								{  instrument number (if 0, name is a catagory) }
		flags:					SInt32;								{  show in picker, etc. }
		toneNameIndex:			SInt32;								{  index in toneNames (1 based) }
		itxtNameAtomID:			SInt32;								{  index in itxtNames (itxt/name by index) }
	end;

	InstrumentInfoListPtr = ^InstrumentInfoList;
	InstrumentInfoList = record
		recordCount:			SInt32;
		toneNames:				Handle;									{  name from tone description }
		itxtNames:				QTAtomContainer;						{  itxt/name atoms for instruments }
		info:					array [0..0] of InstrumentInfoRecord;
	end;

	InstrumentInfoListHandle			= ^InstrumentInfoListPtr;
	{
	 *  MusicGetDescription()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function MusicGetDescription(mc: MusicComponent; var sd: SynthesizerDescription): ComponentResult; external name '_MusicGetDescription';
{
 *  MusicGetPart()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicTask(mc: MusicComponent): ComponentResult; external name '_MusicTask';
{
 *  MusicSetPartInstrumentNumberInterruptSafe()
=======
function MusicGetPart(mc: MusicComponent; part: SInt32; var midiChannel: SInt32; var polyphony: SInt32): ComponentResult; external name '_MusicGetPart';
{
 *  MusicSetPart()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicSetPartInstrumentNumberInterruptSafe(mc: MusicComponent; part: SInt32; instrumentNumber: SInt32): ComponentResult; external name '_MusicSetPartInstrumentNumberInterruptSafe';
{
 *  MusicSetPartSoundLocalization()
=======
function MusicSetPart(mc: MusicComponent; part: SInt32; midiChannel: SInt32; polyphony: SInt32): ComponentResult; external name '_MusicSetPart';
{
 *  MusicSetPartInstrumentNumber()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicSetPartSoundLocalization(mc: MusicComponent; part: SInt32; data: Handle): ComponentResult; external name '_MusicSetPartSoundLocalization';
{
 *  MusicGenericConfigure()
=======
function MusicSetPartInstrumentNumber(mc: MusicComponent; part: SInt32; instrumentNumber: SInt32): ComponentResult; external name '_MusicSetPartInstrumentNumber';
{
 *  MusicGetPartInstrumentNumber()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicGenericConfigure(mc: MusicComponent; mode: SInt32; flags: SInt32; baseResID: SInt32): ComponentResult; external name '_MusicGenericConfigure';
{
 *  MusicGenericGetPart()
=======
function MusicGetPartInstrumentNumber(mc: MusicComponent; part: SInt32): ComponentResult; external name '_MusicGetPartInstrumentNumber';
{
 *  MusicStorePartInstrument()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicGenericGetPart(mc: MusicComponent; partNumber: SInt32; var part: UnivPtr): ComponentResult; external name '_MusicGenericGetPart';
{
 *  MusicGenericGetKnobList()
=======
function MusicStorePartInstrument(mc: MusicComponent; part: SInt32; instrumentNumber: SInt32): ComponentResult; external name '_MusicStorePartInstrument';
{
 *  MusicGetPartAtomicInstrument()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicGenericGetKnobList(mc: MusicComponent; knobType: SInt32; var gkdlH: GenericKnobDescriptionListHandle): ComponentResult; external name '_MusicGenericGetKnobList';
{
 *  MusicGenericSetResourceNumbers()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
function MusicGetPartAtomicInstrument(mc: MusicComponent; part: SInt32; var ai: AtomicInstrument; flags: SInt32): ComponentResult; external name '_MusicGetPartAtomicInstrument';
{
 *  MusicSetPartAtomicInstrument()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicGenericSetResourceNumbers(mc: MusicComponent; resourceIDH: Handle): ComponentResult; external name '_MusicGenericSetResourceNumbers';
{
 *  MusicDerivedMIDISend()
=======
function MusicSetPartAtomicInstrument(mc: MusicComponent; part: SInt32; aiP: AtomicInstrumentPtr; flags: SInt32): ComponentResult; external name '_MusicSetPartAtomicInstrument';
{
 *  MusicGetPartKnob()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedMIDISend(mc: MusicComponent; var packet: MusicMIDIPacket): ComponentResult; external name '_MusicDerivedMIDISend';
{
 *  MusicDerivedSetKnob()
=======
function MusicGetPartKnob(mc: MusicComponent; part: SInt32; knobID: SInt32): ComponentResult; external name '_MusicGetPartKnob';
{
 *  MusicSetPartKnob()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedSetKnob(mc: MusicComponent; knobType: SInt32; knobNumber: SInt32; knobValue: SInt32; partNumber: SInt32; var p: GCPart; var gkd: GenericKnobDescription): ComponentResult; external name '_MusicDerivedSetKnob';
{
 *  MusicDerivedSetPart()
=======
function MusicSetPartKnob(mc: MusicComponent; part: SInt32; knobID: SInt32; knobValue: SInt32): ComponentResult; external name '_MusicSetPartKnob';
{
 *  MusicGetKnob()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedSetPart(mc: MusicComponent; partNumber: SInt32; var p: GCPart): ComponentResult; external name '_MusicDerivedSetPart';
{
 *  MusicDerivedSetInstrument()
=======
function MusicGetKnob(mc: MusicComponent; knobID: SInt32): ComponentResult; external name '_MusicGetKnob';
{
 *  MusicSetKnob()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedSetInstrument(mc: MusicComponent; partNumber: SInt32; var p: GCPart): ComponentResult; external name '_MusicDerivedSetInstrument';
{
 *  MusicDerivedSetPartInstrumentNumber()
=======
function MusicSetKnob(mc: MusicComponent; knobID: SInt32; knobValue: SInt32): ComponentResult; external name '_MusicSetKnob';
{
 *  MusicGetPartName()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedSetPartInstrumentNumber(mc: MusicComponent; partNumber: SInt32; var p: GCPart): ComponentResult; external name '_MusicDerivedSetPartInstrumentNumber';
{
 *  MusicDerivedSetMIDI()
=======
function MusicGetPartName(mc: MusicComponent; part: SInt32; name: StringPtr): ComponentResult; external name '_MusicGetPartName';
{
 *  MusicSetPartName()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedSetMIDI(mc: MusicComponent; midiProc: MusicMIDISendUPP; refcon: SInt32; midiChannel: SInt32): ComponentResult; external name '_MusicDerivedSetMIDI';
{
 *  MusicDerivedStorePartInstrument()
=======
function MusicSetPartName(mc: MusicComponent; part: SInt32; name: StringPtr): ComponentResult; external name '_MusicSetPartName';
{
 *  MusicFindTone()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedStorePartInstrument(mc: MusicComponent; partNumber: SInt32; var p: GCPart; instrumentNumber: SInt32): ComponentResult; external name '_MusicDerivedStorePartInstrument';
{
 *  MusicDerivedOpenResFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
function MusicFindTone(mc: MusicComponent; var td: ToneDescription; var libraryIndexOut: SInt32; var fit: UInt32): ComponentResult; external name '_MusicFindTone';
{
 *  MusicPlayNote()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedOpenResFile(mc: MusicComponent): ComponentResult; external name '_MusicDerivedOpenResFile';
{
 *  MusicDerivedCloseResFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
function MusicPlayNote(mc: MusicComponent; part: SInt32; pitch: SInt32; velocity: SInt32): ComponentResult; external name '_MusicPlayNote';
{
 *  MusicResetPart()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MusicDerivedCloseResFile(mc: MusicComponent; resRefNum: SInt16): ComponentResult; external name '_MusicDerivedCloseResFile';
{--------------------------
    Types
--------------------------}

const
	kNoteRequestNoGM			= 1;							{  don't degrade to a GM synth  }
	kNoteRequestNoSynthType		= 2;							{  don't degrade to another synth of same type but different name  }
	kNoteRequestSynthMustMatch	= 4;							{  synthType must be a match, including kGMSynthComponentSubType  }


	kNoteRequestSpecifyMIDIChannel = $80;


type
	NoteAllocator						= ComponentInstance;
	{	
	    The midiChannelAssignment field of this structure is used to assign a MIDI channel 
	    when a NoteChannel is created from a NoteRequest.
	    A value of 0 indicates a MIDI channel has *not* been assigned
	    A value of (kNoteRequestSpecifyMIDIChannel | 1->16) is a MIDI channel assignment
	
	    This field requires QuickTime 5.0 or later and should be set to 0 for prior versions.
		}
	NoteRequestMIDIChannel				= UInt8;
	NoteRequestInfoPtr = ^NoteRequestInfo;
	NoteRequestInfo = record
		flags:					SInt8;									{  1: dont accept GM match, 2: dont accept same-synth-type match  }
		midiChannelAssignment:	SInt8;									{  (kNoteRequestSpecifyMIDIChannel | 1->16) as MIDI Channel assignement or zero - see notes above   }
		polyphony:				BigEndianShort;							{  Maximum number of voices  }
		typicalPolyphony:		BigEndianFixed;							{  Hint for level mixing  }
	end;

	NoteRequestPtr = ^NoteRequest;
	NoteRequest = record
		info:					NoteRequestInfo;
		tone:					ToneDescription;
	end;

	NoteChannel    = ^SInt32; { an opaque 32-bit type }
	NoteChannelPtr = ^NoteChannel;  { when a var xx:NoteChannel parameter can be nil, it is changed to xx: NoteChannelPtr }


const
	kPickDontMix				= 1;							{  dont mix instruments with drum sounds  }
	kPickSameSynth				= 2;							{  only allow the same device that went in, to come out  }
	kPickUserInsts				= 4;							{  show user insts in addition to ROM voices  }
	kPickEditAllowEdit			= 8;							{  lets user switch over to edit mode  }
	kPickEditAllowPick			= 16;							{  lets the user switch over to pick mode  }
	kPickEditSynthGlobal		= 32;							{  edit the global knobs of the synth  }
	kPickEditControllers		= 64;							{  edit the controllers of the notechannel  }


	kNoteAllocatorComponentType	= FourCharCode('nota');


	{	--------------------------------
	    Note Allocator Prototypes
	--------------------------------	}
	{
	 *  NARegisterMusicDevice()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function NARegisterMusicDevice(na: NoteAllocator; synthType: OSType; var name: Str31; var connections: SynthesizerConnections): ComponentResult; external name '_NARegisterMusicDevice';
{
 *  NAUnregisterMusicDevice()
=======
function MusicResetPart(mc: MusicComponent; part: SInt32): ComponentResult; external name '_MusicResetPart';
{
 *  MusicSetPartController()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAUnregisterMusicDevice(na: NoteAllocator; index: SInt32): ComponentResult; external name '_NAUnregisterMusicDevice';
{
 *  NAGetRegisteredMusicDevice()
=======
function MusicSetPartController(mc: MusicComponent; part: SInt32; controllerNumber: MusicController; controllerValue: SInt32): ComponentResult; external name '_MusicSetPartController';
{
 *  MusicGetPartController()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAGetRegisteredMusicDevice(na: NoteAllocator; index: SInt32; var synthType: OSType; var name: Str31; var connections: SynthesizerConnections; var mc: MusicComponent): ComponentResult; external name '_NAGetRegisteredMusicDevice';
{
 *  NASaveMusicConfiguration()
=======
function MusicGetPartController(mc: MusicComponent; part: SInt32; controllerNumber: MusicController): ComponentResult; external name '_MusicGetPartController';
{
 *  MusicGetMIDIProc()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASaveMusicConfiguration(na: NoteAllocator): ComponentResult; external name '_NASaveMusicConfiguration';
{
 *  NANewNoteChannel()
=======
function MusicGetMIDIProc(mc: MusicComponent; var midiSendProc: MusicMIDISendUPP; var refCon: SInt32): ComponentResult; external name '_MusicGetMIDIProc';
{
 *  MusicSetMIDIProc()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NANewNoteChannel(na: NoteAllocator; var noteRequest_: NoteRequest; var outChannel: NoteChannel): ComponentResult; external name '_NANewNoteChannel';
{
 *  NADisposeNoteChannel()
=======
function MusicSetMIDIProc(mc: MusicComponent; midiSendProc: MusicMIDISendUPP; refCon: SInt32): ComponentResult; external name '_MusicSetMIDIProc';
{
 *  MusicGetInstrumentNames()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NADisposeNoteChannel(na: NoteAllocator; noteChannel_: NoteChannel): ComponentResult; external name '_NADisposeNoteChannel';
{
 *  NAGetNoteChannelInfo()
=======
function MusicGetInstrumentNames(mc: MusicComponent; modifiableInstruments: SInt32; var instrumentNames: Handle; var instrumentCategoryLasts: Handle; var instrumentCategoryNames: Handle): ComponentResult; external name '_MusicGetInstrumentNames';
{
 *  MusicGetDrumNames()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAGetNoteChannelInfo(na: NoteAllocator; noteChannel_: NoteChannel; var index: SInt32; var part: SInt32): ComponentResult; external name '_NAGetNoteChannelInfo';
{
 *  NAPrerollNoteChannel()
=======
function MusicGetDrumNames(mc: MusicComponent; modifiableInstruments: SInt32; var instrumentNumbers: Handle; var instrumentNames: Handle): ComponentResult; external name '_MusicGetDrumNames';
{
 *  MusicGetMasterTune()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAPrerollNoteChannel(na: NoteAllocator; noteChannel_: NoteChannel): ComponentResult; external name '_NAPrerollNoteChannel';
{
 *  NAUnrollNoteChannel()
=======
function MusicGetMasterTune(mc: MusicComponent): ComponentResult; external name '_MusicGetMasterTune';
{
 *  MusicSetMasterTune()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAUnrollNoteChannel(na: NoteAllocator; noteChannel_: NoteChannel): ComponentResult; external name '_NAUnrollNoteChannel';
{
 *  NASetNoteChannelVolume()
=======
function MusicSetMasterTune(mc: MusicComponent; masterTune: SInt32): ComponentResult; external name '_MusicSetMasterTune';
{
 *  MusicGetInstrumentAboutInfo()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASetNoteChannelVolume(na: NoteAllocator; noteChannel_: NoteChannel; volume: Fixed): ComponentResult; external name '_NASetNoteChannelVolume';
{
 *  NAResetNoteChannel()
=======
function MusicGetInstrumentAboutInfo(mc: MusicComponent; part: SInt32; var iai: InstrumentAboutInfo): ComponentResult; external name '_MusicGetInstrumentAboutInfo';
{
 *  MusicGetDeviceConnection()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAResetNoteChannel(na: NoteAllocator; noteChannel_: NoteChannel): ComponentResult; external name '_NAResetNoteChannel';
{
 *  NAPlayNote()
=======
function MusicGetDeviceConnection(mc: MusicComponent; index: SInt32; var id1: SInt32; var id2: SInt32): ComponentResult; external name '_MusicGetDeviceConnection';
{
 *  MusicUseDeviceConnection()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAPlayNote(na: NoteAllocator; noteChannel_: NoteChannel; pitch: SInt32; velocity: SInt32): ComponentResult; external name '_NAPlayNote';
{
 *  NASetController()
=======
function MusicUseDeviceConnection(mc: MusicComponent; id1: SInt32; id2: SInt32): ComponentResult; external name '_MusicUseDeviceConnection';
{
 *  MusicGetKnobSettingStrings()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASetController(na: NoteAllocator; noteChannel_: NoteChannel; controllerNumber: SInt32; controllerValue: SInt32): ComponentResult; external name '_NASetController';
{
 *  NASetKnob()
=======
function MusicGetKnobSettingStrings(mc: MusicComponent; knobIndex: SInt32; isGlobal: SInt32; var settingsNames: Handle; var settingsCategoryLasts: Handle; var settingsCategoryNames: Handle): ComponentResult; external name '_MusicGetKnobSettingStrings';
{
 *  MusicGetMIDIPorts()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASetKnob(na: NoteAllocator; noteChannel_: NoteChannel; knobNumber: SInt32; knobValue: SInt32): ComponentResult; external name '_NASetKnob';
{
 *  NAFindNoteChannelTone()
=======
function MusicGetMIDIPorts(mc: MusicComponent; var inputPortCount: SInt32; var outputPortCount: SInt32): ComponentResult; external name '_MusicGetMIDIPorts';
{
 *  MusicSendMIDI()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAFindNoteChannelTone(na: NoteAllocator; noteChannel_: NoteChannel; var td: ToneDescription; var instrumentNumber: SInt32): ComponentResult; external name '_NAFindNoteChannelTone';
{
 *  NASetInstrumentNumber()
=======
function MusicSendMIDI(mc: MusicComponent; portIndex: SInt32; var mp: MusicMIDIPacket): ComponentResult; external name '_MusicSendMIDI';
{
 *  MusicStartOffline()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASetInstrumentNumber(na: NoteAllocator; noteChannel_: NoteChannel; instrumentNumber: SInt32): ComponentResult; external name '_NASetInstrumentNumber';
{
 *  NAPickInstrument()
=======
function MusicStartOffline(mc: MusicComponent; var numChannels: UInt32; var sampleRate: UnsignedFixed; var sampleSize: UInt16; dataProc: MusicOfflineDataUPP; dataProcRefCon: SInt32): ComponentResult; external name '_MusicStartOffline';
{
 *  MusicSetOfflineTimeTo()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAPickInstrument(na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; var sd: ToneDescription; flags: UInt32; refCon: SInt32; reserved1: SInt32; reserved2: SInt32): ComponentResult; external name '_NAPickInstrument';
{
 *  NAPickArrangement()
=======
function MusicSetOfflineTimeTo(mc: MusicComponent; newTimeStamp: SInt32): ComponentResult; external name '_MusicSetOfflineTimeTo';
{
 *  MusicGetInstrumentKnobDescription()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAPickArrangement(na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; zero1: SInt32; zero2: SInt32; t: Track; songName: StringPtr): ComponentResult; external name '_NAPickArrangement';
{
 *  NAStuffToneDescription()
=======
function MusicGetInstrumentKnobDescription(mc: MusicComponent; knobIndex: SInt32; var mkd: KnobDescription): ComponentResult; external name '_MusicGetInstrumentKnobDescription';
{
 *  MusicGetDrumKnobDescription()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAStuffToneDescription(na: NoteAllocator; gmNumber: SInt32; var td: ToneDescription): ComponentResult; external name '_NAStuffToneDescription';
{
 *  NACopyrightDialog()
=======
function MusicGetDrumKnobDescription(mc: MusicComponent; knobIndex: SInt32; var mkd: KnobDescription): ComponentResult; external name '_MusicGetDrumKnobDescription';
{
 *  MusicGetKnobDescription()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NACopyrightDialog(na: NoteAllocator; p: PicHandle; author: StringPtr; copyright: StringPtr; other: StringPtr; title: StringPtr; filterProc: ModalFilterUPP; refCon: SInt32): ComponentResult; external name '_NACopyrightDialog';
>>>>>>> graemeg/fixes_2_2
{
    kNADummyOneSelect = 29
    kNADummyTwoSelect = 30
}

{
<<<<<<< HEAD
 *  NAGetIndNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetIndNoteChannel( na: NoteAllocator; index: SIGNEDLONG; var nc: NoteChannel; var seed: SIGNEDLONG ): ComponentResult; external name '_NAGetIndNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetMIDIPorts()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetMIDIPorts( na: NoteAllocator; var inputPorts: QTMIDIPortListHandle; var outputPorts: QTMIDIPortListHandle ): ComponentResult; external name '_NAGetMIDIPorts';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetNoteRequest()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetNoteRequest( na: NoteAllocator; noteChannel_: NoteChannel; var nrOut: NoteRequest ): ComponentResult; external name '_NAGetNoteRequest';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASendMIDI()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASendMIDI( na: NoteAllocator; noteChannel_: NoteChannel; var mp: MusicMIDIPacket ): ComponentResult; external name '_NASendMIDI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPickEditInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPickEditInstrument( na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; refCon: SIGNEDLONG; nc: NoteChannel; ai: AtomicInstrument; flags: SIGNEDLONG ): ComponentResult; external name '_NAPickEditInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NANewNoteChannelFromAtomicInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NANewNoteChannelFromAtomicInstrument( na: NoteAllocator; instrument: AtomicInstrumentPtr; flags: SIGNEDLONG; var outChannel: NoteChannel ): ComponentResult; external name '_NANewNoteChannelFromAtomicInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetAtomicInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetAtomicInstrument( na: NoteAllocator; noteChannel_: NoteChannel; instrument: AtomicInstrumentPtr; flags: SIGNEDLONG ): ComponentResult; external name '_NASetAtomicInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetKnob( na: NoteAllocator; noteChannel_: NoteChannel; knobNumber: SIGNEDLONG; var knobValue: SIGNEDLONG ): ComponentResult; external name '_NAGetKnob';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NATask()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NATask( na: NoteAllocator ): ComponentResult; external name '_NATask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetNoteChannelBalance()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetNoteChannelBalance( na: NoteAllocator; noteChannel_: NoteChannel; balance: SIGNEDLONG ): ComponentResult; external name '_NASetNoteChannelBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetInstrumentNumberInterruptSafe()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetInstrumentNumberInterruptSafe( na: NoteAllocator; noteChannel_: NoteChannel; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_NASetInstrumentNumberInterruptSafe';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetNoteChannelSoundLocalization()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetNoteChannelSoundLocalization( na: NoteAllocator; noteChannel_: NoteChannel; data: Handle ): ComponentResult; external name '_NASetNoteChannelSoundLocalization';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetController()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetController( na: NoteAllocator; noteChannel_: NoteChannel; controllerNumber: SIGNEDLONG; var controllerValue: SIGNEDLONG ): ComponentResult; external name '_NAGetController';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


const
	kTuneQueueDepth = 8;     { Deepest you can queue tune segments }
=======
 *  NAGetIndNoteChannel()
=======
function MusicGetKnobDescription(mc: MusicComponent; knobIndex: SInt32; var mkd: KnobDescription): ComponentResult; external name '_MusicGetKnobDescription';
{
 *  MusicGetInfoText()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGetInfoText(mc: MusicComponent; selector: SInt32; var textH: Handle; var styleH: Handle): ComponentResult; external name '_MusicGetInfoText';
const
	kGetInstrumentInfoNoBuiltIn	= $01;
	kGetInstrumentInfoMidiUserInst = $02;
	kGetInstrumentInfoNoIText	= $04;

	{
	 *  MusicGetInstrumentInfo()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function MusicGetInstrumentInfo(mc: MusicComponent; getInstrumentInfoFlags: SInt32; var infoListH: InstrumentInfoListHandle): ComponentResult; external name '_MusicGetInstrumentInfo';
{
 *  MusicTask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicTask(mc: MusicComponent): ComponentResult; external name '_MusicTask';
{
 *  MusicSetPartInstrumentNumberInterruptSafe()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartInstrumentNumberInterruptSafe(mc: MusicComponent; part: SInt32; instrumentNumber: SInt32): ComponentResult; external name '_MusicSetPartInstrumentNumberInterruptSafe';
{
 *  MusicSetPartSoundLocalization()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicSetPartSoundLocalization(mc: MusicComponent; part: SInt32; data: Handle): ComponentResult; external name '_MusicSetPartSoundLocalization';
{
 *  MusicGenericConfigure()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGenericConfigure(mc: MusicComponent; mode: SInt32; flags: SInt32; baseResID: SInt32): ComponentResult; external name '_MusicGenericConfigure';
{
 *  MusicGenericGetPart()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGenericGetPart(mc: MusicComponent; partNumber: SInt32; var part: UnivPtr): ComponentResult; external name '_MusicGenericGetPart';
{
 *  MusicGenericGetKnobList()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGenericGetKnobList(mc: MusicComponent; knobType: SInt32; var gkdlH: GenericKnobDescriptionListHandle): ComponentResult; external name '_MusicGenericGetKnobList';
{
 *  MusicGenericSetResourceNumbers()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicGenericSetResourceNumbers(mc: MusicComponent; resourceIDH: Handle): ComponentResult; external name '_MusicGenericSetResourceNumbers';
{
 *  MusicDerivedMIDISend()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedMIDISend(mc: MusicComponent; var packet: MusicMIDIPacket): ComponentResult; external name '_MusicDerivedMIDISend';
{
 *  MusicDerivedSetKnob()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedSetKnob(mc: MusicComponent; knobType: SInt32; knobNumber: SInt32; knobValue: SInt32; partNumber: SInt32; var p: GCPart; var gkd: GenericKnobDescription): ComponentResult; external name '_MusicDerivedSetKnob';
{
 *  MusicDerivedSetPart()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedSetPart(mc: MusicComponent; partNumber: SInt32; var p: GCPart): ComponentResult; external name '_MusicDerivedSetPart';
{
 *  MusicDerivedSetInstrument()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAGetIndNoteChannel(na: NoteAllocator; index: SInt32; var nc: NoteChannel; var seed: SInt32): ComponentResult; external name '_NAGetIndNoteChannel';
{
 *  NAGetMIDIPorts()
=======
function MusicDerivedSetInstrument(mc: MusicComponent; partNumber: SInt32; var p: GCPart): ComponentResult; external name '_MusicDerivedSetInstrument';
{
 *  MusicDerivedSetPartInstrumentNumber()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAGetMIDIPorts(na: NoteAllocator; var inputPorts: QTMIDIPortListHandle; var outputPorts: QTMIDIPortListHandle): ComponentResult; external name '_NAGetMIDIPorts';
{
 *  NAGetNoteRequest()
=======
function MusicDerivedSetPartInstrumentNumber(mc: MusicComponent; partNumber: SInt32; var p: GCPart): ComponentResult; external name '_MusicDerivedSetPartInstrumentNumber';
{
 *  MusicDerivedSetMIDI()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAGetNoteRequest(na: NoteAllocator; noteChannel_: NoteChannel; var nrOut: NoteRequest): ComponentResult; external name '_NAGetNoteRequest';
{
 *  NASendMIDI()
=======
function MusicDerivedSetMIDI(mc: MusicComponent; midiProc: MusicMIDISendUPP; refcon: SInt32; midiChannel: SInt32): ComponentResult; external name '_MusicDerivedSetMIDI';
{
 *  MusicDerivedStorePartInstrument()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASendMIDI(na: NoteAllocator; noteChannel_: NoteChannel; var mp: MusicMIDIPacket): ComponentResult; external name '_NASendMIDI';
{
 *  NAPickEditInstrument()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
function MusicDerivedStorePartInstrument(mc: MusicComponent; partNumber: SInt32; var p: GCPart; instrumentNumber: SInt32): ComponentResult; external name '_MusicDerivedStorePartInstrument';
{
 *  MusicDerivedOpenResFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAPickEditInstrument(na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; refCon: SInt32; nc: NoteChannel; ai: AtomicInstrument; flags: SInt32): ComponentResult; external name '_NAPickEditInstrument';
{
 *  NANewNoteChannelFromAtomicInstrument()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
function MusicDerivedOpenResFile(mc: MusicComponent): ComponentResult; external name '_MusicDerivedOpenResFile';
{
 *  MusicDerivedCloseResFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NANewNoteChannelFromAtomicInstrument(na: NoteAllocator; instrument: AtomicInstrumentPtr; flags: SInt32; var outChannel: NoteChannel): ComponentResult; external name '_NANewNoteChannelFromAtomicInstrument';
{
 *  NASetAtomicInstrument()
=======
function MusicDerivedCloseResFile(mc: MusicComponent; resRefNum: SInt16): ComponentResult; external name '_MusicDerivedCloseResFile';
{--------------------------
    Types
--------------------------}

const
	kNoteRequestNoGM			= 1;							{  don't degrade to a GM synth  }
	kNoteRequestNoSynthType		= 2;							{  don't degrade to another synth of same type but different name  }
	kNoteRequestSynthMustMatch	= 4;							{  synthType must be a match, including kGMSynthComponentSubType  }


	kNoteRequestSpecifyMIDIChannel = $80;


type
	NoteAllocator						= ComponentInstance;
	{	
	    The midiChannelAssignment field of this structure is used to assign a MIDI channel 
	    when a NoteChannel is created from a NoteRequest.
	    A value of 0 indicates a MIDI channel has *not* been assigned
	    A value of (kNoteRequestSpecifyMIDIChannel | 1->16) is a MIDI channel assignment
	
	    This field requires QuickTime 5.0 or later and should be set to 0 for prior versions.
		}
	NoteRequestMIDIChannel				= UInt8;
	NoteRequestInfoPtr = ^NoteRequestInfo;
	NoteRequestInfo = record
		flags:					SInt8;									{  1: dont accept GM match, 2: dont accept same-synth-type match  }
		midiChannelAssignment:	SInt8;									{  (kNoteRequestSpecifyMIDIChannel | 1->16) as MIDI Channel assignement or zero - see notes above   }
		polyphony:				BigEndianShort;							{  Maximum number of voices  }
		typicalPolyphony:		BigEndianFixed;							{  Hint for level mixing  }
	end;

	NoteRequestPtr = ^NoteRequest;
	NoteRequest = record
		info:					NoteRequestInfo;
		tone:					ToneDescription;
	end;

	NoteChannel    = ^SInt32; { an opaque 32-bit type }
	NoteChannelPtr = ^NoteChannel;  { when a var xx:NoteChannel parameter can be nil, it is changed to xx: NoteChannelPtr }


const
	kPickDontMix				= 1;							{  dont mix instruments with drum sounds  }
	kPickSameSynth				= 2;							{  only allow the same device that went in, to come out  }
	kPickUserInsts				= 4;							{  show user insts in addition to ROM voices  }
	kPickEditAllowEdit			= 8;							{  lets user switch over to edit mode  }
	kPickEditAllowPick			= 16;							{  lets the user switch over to pick mode  }
	kPickEditSynthGlobal		= 32;							{  edit the global knobs of the synth  }
	kPickEditControllers		= 64;							{  edit the controllers of the notechannel  }


	kNoteAllocatorComponentType	= FourCharCode('nota');


	{	--------------------------------
	    Note Allocator Prototypes
	--------------------------------	}
	{
	 *  NARegisterMusicDevice()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function NARegisterMusicDevice(na: NoteAllocator; synthType: OSType; var name: Str31; var connections: SynthesizerConnections): ComponentResult; external name '_NARegisterMusicDevice';
{
 *  NAUnregisterMusicDevice()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASetAtomicInstrument(na: NoteAllocator; noteChannel_: NoteChannel; instrument: AtomicInstrumentPtr; flags: SInt32): ComponentResult; external name '_NASetAtomicInstrument';
{
 *  NAGetKnob()
=======
function NAUnregisterMusicDevice(na: NoteAllocator; index: SInt32): ComponentResult; external name '_NAUnregisterMusicDevice';
{
 *  NAGetRegisteredMusicDevice()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAGetKnob(na: NoteAllocator; noteChannel_: NoteChannel; knobNumber: SInt32; var knobValue: SInt32): ComponentResult; external name '_NAGetKnob';
{
 *  NATask()
=======
function NAGetRegisteredMusicDevice(na: NoteAllocator; index: SInt32; var synthType: OSType; var name: Str31; var connections: SynthesizerConnections; var mc: MusicComponent): ComponentResult; external name '_NAGetRegisteredMusicDevice';
{
 *  NASaveMusicConfiguration()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NATask(na: NoteAllocator): ComponentResult; external name '_NATask';
{
 *  NASetNoteChannelBalance()
=======
function NASaveMusicConfiguration(na: NoteAllocator): ComponentResult; external name '_NASaveMusicConfiguration';
{
 *  NANewNoteChannel()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASetNoteChannelBalance(na: NoteAllocator; noteChannel_: NoteChannel; balance: SInt32): ComponentResult; external name '_NASetNoteChannelBalance';
{
 *  NASetInstrumentNumberInterruptSafe()
=======
function NANewNoteChannel(na: NoteAllocator; var noteRequest_: NoteRequest; var outChannel: NoteChannel): ComponentResult; external name '_NANewNoteChannel';
{
 *  NADisposeNoteChannel()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASetInstrumentNumberInterruptSafe(na: NoteAllocator; noteChannel_: NoteChannel; instrumentNumber: SInt32): ComponentResult; external name '_NASetInstrumentNumberInterruptSafe';
{
 *  NASetNoteChannelSoundLocalization()
=======
function NADisposeNoteChannel(na: NoteAllocator; noteChannel_: NoteChannel): ComponentResult; external name '_NADisposeNoteChannel';
{
 *  NAGetNoteChannelInfo()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NASetNoteChannelSoundLocalization(na: NoteAllocator; noteChannel_: NoteChannel; data: Handle): ComponentResult; external name '_NASetNoteChannelSoundLocalization';
{
 *  NAGetController()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
function NAGetNoteChannelInfo(na: NoteAllocator; noteChannel_: NoteChannel; var index: SInt32; var part: SInt32): ComponentResult; external name '_NAGetNoteChannelInfo';
{
 *  NAPrerollNoteChannel()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function NAGetController(na: NoteAllocator; noteChannel_: NoteChannel; controllerNumber: SInt32; var controllerValue: SInt32): ComponentResult; external name '_NAGetController';
const
	kTuneQueueDepth				= 8;							{  Deepest you can queue tune segments  }
>>>>>>> graemeg/fixes_2_2


type
	TuneStatusPtr = ^TuneStatus;
	TuneStatus = record
<<<<<<< HEAD
		tune: UNSIGNEDLONGPtr;                   { currently playing tune }
		tunePtr: UNSIGNEDLONGPtr;                { position within currently playing piece }
		time: TimeValue;                   { current tune time }
		queueCount: SInt16;             { how many pieces queued up? }
		queueSpots: SInt16;             { How many more tunepieces can be queued }
		queueTime: TimeValue;              { How much time is queued up? (can be very inaccurate) }
		reserved: array [0..2] of SIGNEDLONG;
	end;

	TuneCallBackProcPtr = procedure( const (*var*) status: TuneStatus; refCon: SIGNEDLONG);
	TunePlayCallBackProcPtr = procedure( var event: UNSIGNEDLONG; seed: SIGNEDLONG; refCon: SIGNEDLONG );
	TuneCallBackUPP = TuneCallBackProcPtr;
	TunePlayCallBackUPP = TunePlayCallBackProcPtr;

type
	TunePlayer = ComponentInstance;
const
	kTunePlayerComponentType = FourCharCode('tune');


{
 *  TuneSetHeader()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetHeader( tp: TunePlayer; var header: UNSIGNEDLONG ): ComponentResult; external name '_TuneSetHeader';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetTimeBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetTimeBase( tp: TunePlayer; var tb: TimeBase ): ComponentResult; external name '_TuneGetTimeBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetTimeScale()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetTimeScale( tp: TunePlayer; scale: TimeScale ): ComponentResult; external name '_TuneSetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetTimeScale()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetTimeScale( tp: TunePlayer; var scale: TimeScale ): ComponentResult; external name '_TuneGetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetIndexedNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetIndexedNoteChannel( tp: TunePlayer; i: SIGNEDLONG; var nc: NoteChannel ): ComponentResult; external name '_TuneGetIndexedNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
<<<<<<< HEAD
<<<<<<< HEAD
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
{ Values for when to start. }
const
	kTuneStartNow = 1;    { start after buffer is implied }
	kTuneDontClipNotes = 2;    { allow notes to finish their durations outside sample }
	kTuneExcludeEdgeNotes = 4;    { dont play notes that start at end of tune }
	kTuneQuickStart = 8;    { Leave all the controllers where they are, ignore start time }
	kTuneLoopUntil = 16;   { loop a queued tune if there's nothing else in the queue}
	kTunePlayDifference = 32;   { by default, the tune difference is skipped}
	kTunePlayConcurrent = 64;   { dont block the next tune sequence with this one}
	kTuneStartNewMaster = 16384;

{
 *  TuneQueue()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneQueue( tp: TunePlayer; var tune: UNSIGNEDLONG; tuneRate: Fixed; tuneStartPosition: UNSIGNEDLONG; tuneStopPosition: UNSIGNEDLONG; queueFlags: UNSIGNEDLONG; callBackProc: TuneCallBackUPP; refCon: SIGNEDLONG ): ComponentResult; external name '_TuneQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneInstant()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneInstant( tp: TunePlayer; var tune: UNSIGNEDLONG; tunePosition: UNSIGNEDLONG ): ComponentResult; external name '_TuneInstant';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetStatus()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetStatus( tp: TunePlayer; var status: TuneStatus ): ComponentResult; external name '_TuneGetStatus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
{ Values for stopping. }
const
	kTuneStopFade = 1;    { do a quick, synchronous fadeout }
	kTuneStopSustain = 2;    { don't silece notes }
	kTuneStopInstant = 4;    { silence notes fast (else, decay them) }
	kTuneStopReleaseChannels = 8;     { afterwards, let the channels go }

{
 *  TuneStop()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneStop( tp: TunePlayer; stopFlags: SIGNEDLONG ): ComponentResult; external name '_TuneStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetVolume( tp: TunePlayer; volume: Fixed ): ComponentResult; external name '_TuneSetVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetVolume()   *** DEPRECATED ***
=======
 *  MusicDerivedSetKnob()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneGetVolume( tp: TunePlayer ): ComponentResult; external name '_TuneGetVolume';
=======
function MusicDerivedSetKnob( mc: MusicComponent; knobType: SIGNEDLONG; knobNumber: SIGNEDLONG; knobValue: SIGNEDLONG; partNumber: SIGNEDLONG; var p: GCPart; var gkd: GenericKnobDescription ): ComponentResult; external name '_MusicDerivedSetKnob';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TunePreroll()   *** DEPRECATED ***
=======
 *  MusicDerivedSetPart()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TunePreroll( tp: TunePlayer ): ComponentResult; external name '_TunePreroll';
=======
function MusicDerivedSetPart( mc: MusicComponent; partNumber: SIGNEDLONG; var p: GCPart ): ComponentResult; external name '_MusicDerivedSetPart';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneUnroll()   *** DEPRECATED ***
=======
 *  MusicDerivedSetInstrument()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneUnroll( tp: TunePlayer ): ComponentResult; external name '_TuneUnroll';
=======
function MusicDerivedSetInstrument( mc: MusicComponent; partNumber: SIGNEDLONG; var p: GCPart ): ComponentResult; external name '_MusicDerivedSetInstrument';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneSetNoteChannels()   *** DEPRECATED ***
=======
 *  MusicDerivedSetPartInstrumentNumber()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneSetNoteChannels( tp: TunePlayer; count: UNSIGNEDLONG; var noteChannelList: NoteChannel; playCallBackProc: TunePlayCallBackUPP; refCon: SIGNEDLONG ): ComponentResult; external name '_TuneSetNoteChannels';
=======
function MusicDerivedSetPartInstrumentNumber( mc: MusicComponent; partNumber: SIGNEDLONG; var p: GCPart ): ComponentResult; external name '_MusicDerivedSetPartInstrumentNumber';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneSetPartTranspose()   *** DEPRECATED ***
=======
 *  MusicDerivedSetMIDI()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneSetPartTranspose( tp: TunePlayer; part: UNSIGNEDLONG; transpose: SIGNEDLONG; velocityShift: SIGNEDLONG ): ComponentResult; external name '_TuneSetPartTranspose';
=======
function MusicDerivedSetMIDI( mc: MusicComponent; midiProc: MusicMIDISendUPP; refcon: SIGNEDLONG; midiChannel: SIGNEDLONG ): ComponentResult; external name '_MusicDerivedSetMIDI';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneGetNoteAllocator()   *** DEPRECATED ***
=======
 *  MusicDerivedStorePartInstrument()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneGetNoteAllocator( tp: TunePlayer ): NoteAllocator; external name '_TuneGetNoteAllocator';
=======
function MusicDerivedStorePartInstrument( mc: MusicComponent; partNumber: SIGNEDLONG; var p: GCPart; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_MusicDerivedStorePartInstrument';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneSetSofter()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetSofter( tp: TunePlayer; softer: SIGNEDLONG ): ComponentResult; external name '_TuneSetSofter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneTask()   *** DEPRECATED ***
=======
 *  MusicDerivedOpenResFile()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedOpenResFile( mc: MusicComponent ): ComponentResult; external name '_MusicDerivedOpenResFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MusicDerivedCloseResFile()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MusicDerivedCloseResFile( mc: MusicComponent; resRefNum: SInt16 ): ComponentResult; external name '_MusicDerivedCloseResFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{--------------------------
    Types
--------------------------}
const
	kNoteRequestNoGM = 1;    { don't degrade to a GM synth }
	kNoteRequestNoSynthType = 2;    { don't degrade to another synth of same type but different name }
	kNoteRequestSynthMustMatch = 4;     { synthType must be a match, including kGMSynthComponentSubType }


const
	kNoteRequestSpecifyMIDIChannel = $80;

type
	NoteAllocator = ComponentInstance;
{
    The midiChannelAssignment field of this structure is used to assign a MIDI channel 
    when a NoteChannel is created from a NoteRequest.
    A value of 0 indicates a MIDI channel has *not* been assigned
    A value of (kNoteRequestSpecifyMIDIChannel | 1->16) is a MIDI channel assignment

    This field requires QuickTime 5.0 or later and should be set to 0 for prior versions.
}
type
	NoteRequestMIDIChannel = UInt8;
	NoteRequestInfoPtr = ^NoteRequestInfo;
	NoteRequestInfo = record
		flags: UInt8;                  { 1: dont accept GM match, 2: dont accept same-synth-type match }
		midiChannelAssignment: NoteRequestMIDIChannel; { (kNoteRequestSpecifyMIDIChannel | 1->16) as MIDI Channel assignement or zero - see notes above  }
		polyphony: BigEndianShort;              { Maximum number of voices }
		typicalPolyphony: BigEndianFixed;       { Hint for level mixing }
	end;
type
	NoteRequestPtr = ^NoteRequest;
	NoteRequest = record
		info: NoteRequestInfo;
		tone: ToneDescription;
	end;

type
	NoteChannel = SIGNEDLONG;

const
	kPickDontMix = 1;    { dont mix instruments with drum sounds }
	kPickSameSynth = 2;    { only allow the same device that went in, to come out }
	kPickUserInsts = 4;    { show user insts in addition to ROM voices }
	kPickEditAllowEdit = 8;    { lets user switch over to edit mode }
	kPickEditAllowPick = 16;   { lets the user switch over to pick mode }
	kPickEditSynthGlobal = 32;   { edit the global knobs of the synth }
	kPickEditControllers = 64;    { edit the controllers of the notechannel }


const
	kNoteAllocatorComponentType = FourCharCode('nota');


{--------------------------------
    Note Allocator Prototypes
--------------------------------}
{
 *  NARegisterMusicDevice()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneTask( tp: TunePlayer ): ComponentResult; external name '_TuneTask';
=======
function NARegisterMusicDevice( na: NoteAllocator; synthType: OSType; var name: Str31; var connections: SynthesizerConnections ): ComponentResult; external name '_NARegisterMusicDevice';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneSetBalance()   *** DEPRECATED ***
=======
 *  NAUnregisterMusicDevice()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneSetBalance( tp: TunePlayer; balance: SIGNEDLONG ): ComponentResult; external name '_TuneSetBalance';
=======
function NAUnregisterMusicDevice( na: NoteAllocator; index: SIGNEDLONG ): ComponentResult; external name '_NAUnregisterMusicDevice';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneSetSoundLocalization()   *** DEPRECATED ***
=======
 *  NAGetRegisteredMusicDevice()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneSetSoundLocalization( tp: TunePlayer; data: Handle ): ComponentResult; external name '_TuneSetSoundLocalization';
=======
function NAGetRegisteredMusicDevice( na: NoteAllocator; index: SIGNEDLONG; var synthType: OSType; name: Str31; var connections: SynthesizerConnections; var mc: MusicComponent ): ComponentResult; external name '_NAGetRegisteredMusicDevice';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneSetHeaderWithSize()   *** DEPRECATED ***
=======
 *  NASaveMusicConfiguration()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function TuneSetHeaderWithSize( tp: TunePlayer; var header: UNSIGNEDLONG; size: UNSIGNEDLONG ): ComponentResult; external name '_TuneSetHeaderWithSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
<<<<<<< HEAD
<<<<<<< HEAD
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
{ flags for part mix. }
const
	kTuneMixMute = 1;    { disable a part }
	kTuneMixSolo = 2;     { if any parts soloed, play only soloed parts }


{
 *  TuneSetPartMix()   *** DEPRECATED ***
=======
function NASaveMusicConfiguration( na: NoteAllocator ): ComponentResult; external name '_NASaveMusicConfiguration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NANewNoteChannel()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetPartMix( tp: TunePlayer; partNumber: UNSIGNEDLONG; volume: SIGNEDLONG; balance: SIGNEDLONG; mixFlags: SIGNEDLONG ): ComponentResult; external name '_TuneSetPartMix';
=======
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NANewNoteChannel( na: NoteAllocator; var noteRequest_: NoteRequest; var outChannel: NoteChannel ): ComponentResult; external name '_NANewNoteChannel';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TuneGetPartMix()   *** DEPRECATED ***
=======
 *  NADisposeNoteChannel()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetPartMix( tp: TunePlayer; partNumber: UNSIGNEDLONG; var volumeOut: SIGNEDLONG; var balanceOut: SIGNEDLONG; var mixFlagsOut: SIGNEDLONG ): ComponentResult; external name '_TuneGetPartMix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


type
	MusicOpWord = UNSIGNEDLONG;
	MusicOpWordPtr = ^MusicOpWord;
{    QuickTime Music Track Event Formats:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew

    At this time, QuickTime music tracks support 5 different event types -- REST events,
    short NOTE events, short CONTROL events, short GENERAL events, Long NOTE events, 
    long CONTROL events, and variable GENERAL events.
 
        ¥ REST Event (4 bytes/event):
    
            (0 0 0) (5-bit UNUSED) (24-bit Rest Duration)
        
        ¥ÊShort NOTE Events (4 bytes/event):
    
            (0 0 1) (5-bit Part) (6-bit Pitch) (7-bit Volume) (11-bit Duration)
        
            where:  Pitch is offset by 32 (Actual pitch = pitch field + 32)

        ¥ÊShort CONTROL Events (4 bytes/event):
    
            (0 1 0) (5-bit Part) (8-bit Controller) (1-bit UNUSED) (1-bit Sign) (7-bit MSB) (7-bit LSB)
                                                                         ( or 15-bit Signed Value)
        ¥ Short GENERAL Event (4 bytes/event):
    
            (0 1 1) (1-bit UNUSED) (12-bit Sub-Type) (16-bit Value)
    
        ¥ Long NOTE Events (8 bytes/event):
    
            (1 0 0 1) (12-bit Part) (1-bit UNUSED) (7-bit Pitch) (1-bit UNUSED) (7-bit Volume)
            (1 0) (8-bit UNUSED) (22-bit Duration)
        
        ¥ÊLong CONTROL Event (8 bytes/event):
        
            (1 0 1 0) (12-bit Part) (16-bit Value MSB) 
            (1 0) (14-bit Controller) (16-bit Value LSB)
    
        ¥ÊLong KNOB Event (8 bytes/event):
    
            (1 0 1 1) (12-bit Sub-Type) (16-bit Value MSB)
            (1 0) (14-bit KNOB) (16-bit Value LSB)
    
        ¥ÊVariable GENERAL Length Events (N bytes/event):
    
            (1 1 1 1) (12-bit Sub-Type) (16-bit Length)
                :
            (32-bit Data values)
                :
            (1 1) (14-bit UNUSED) (16-bit Length)
    
            where:  Length field is the number of LONG words in the record.
                    Lengths include the first and last long words (Minimum length = 2)
                
    The following event type values have not been used yet and are reserved for 
    future expansion:
        
        ¥ (1 0 0 0)     (8 bytes/event)
        ¥ (1 1 0 0)     (N bytes/event)
        ¥ (1 1 0 1)     (N bytes/event)
        ¥ (1 1 1 0)     (N bytes/event)
        
    For all events, the following generalizations apply:
    
        -   All duration values are specified in Millisecond units.
        -   Pitch values are intended to map directly to the MIDI key numbers.
        -   Controllers from 0 to 127 correspond to the standard MIDI controllers.
            Controllers greater than 127 correspond to other controls (i.e., Pitch Bend, 
            Key Pressure, and Channel Pressure).    
}

<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    At this time, QuickTime music tracks support 5 different event types -- REST events,
    short NOTE events, short CONTROL events, short GENERAL events, Long NOTE events, 
    long CONTROL events, and variable GENERAL events.
 
        ¥ REST Event (4 bytes/event):
    
            (0 0 0) (5-bit UNUSED) (24-bit Rest Duration)
        
        ¥ÊShort NOTE Events (4 bytes/event):
    
            (0 0 1) (5-bit Part) (6-bit Pitch) (7-bit Volume) (11-bit Duration)
        
            where:  Pitch is offset by 32 (Actual pitch = pitch field + 32)

        ¥ÊShort CONTROL Events (4 bytes/event):
    
            (0 1 0) (5-bit Part) (8-bit Controller) (1-bit UNUSED) (1-bit Sign) (7-bit MSB) (7-bit LSB)
                                                                         ( or 15-bit Signed Value)
        ¥ Short GENERAL Event (4 bytes/event):
    
            (0 1 1) (1-bit UNUSED) (12-bit Sub-Type) (16-bit Value)
    
        ¥ Long NOTE Events (8 bytes/event):
    
            (1 0 0 1) (12-bit Part) (1-bit UNUSED) (7-bit Pitch) (1-bit UNUSED) (7-bit Volume)
            (1 0) (8-bit UNUSED) (22-bit Duration)
        
        ¥ÊLong CONTROL Event (8 bytes/event):
        
            (1 0 1 0) (12-bit Part) (16-bit Value MSB) 
            (1 0) (14-bit Controller) (16-bit Value LSB)
    
        ¥ÊLong KNOB Event (8 bytes/event):
    
            (1 0 1 1) (12-bit Sub-Type) (16-bit Value MSB)
            (1 0) (14-bit KNOB) (16-bit Value LSB)
    
        ¥ÊVariable GENERAL Length Events (N bytes/event):
    
            (1 1 1 1) (12-bit Sub-Type) (16-bit Length)
                :
            (32-bit Data values)
                :
            (1 1) (14-bit UNUSED) (16-bit Length)
    
            where:  Length field is the number of LONG words in the record.
                    Lengths include the first and last long words (Minimum length = 2)
                
    The following event type values have not been used yet and are reserved for 
    future expansion:
        
        ¥ (1 0 0 0)     (8 bytes/event)
        ¥ (1 1 0 0)     (N bytes/event)
        ¥ (1 1 0 1)     (N bytes/event)
        ¥ (1 1 1 0)     (N bytes/event)
        
    For all events, the following generalizations apply:
    
        -   All duration values are specified in Millisecond units.
        -   Pitch values are intended to map directly to the MIDI key numbers.
        -   Controllers from 0 to 127 correspond to the standard MIDI controllers.
            Controllers greater than 127 correspond to other controls (i.e., Pitch Bend, 
            Key Pressure, and Channel Pressure).    
}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Defines for the implemented music event data fields}
const
	kRestEventType = $00000000; { lower 3-bits }
	kNoteEventType = $00000001; { lower 3-bits }
	kControlEventType = $00000002; { lower 3-bits }
	kMarkerEventType = $00000003; { lower 3-bits }
	kUndefined1EventType = $00000008; { 4-bits }
	kXNoteEventType = $00000009; { 4-bits }
	kXControlEventType = $0000000A; { 4-bits }
	kKnobEventType = $0000000B; { 4-bits }
	kUndefined2EventType = $0000000C; { 4-bits }
	kUndefined3EventType = $0000000D; { 4-bits }
	kUndefined4EventType = $0000000E; { 4-bits }
	kGeneralEventType = $0000000F; { 4-bits }
	kXEventLengthBits = $00000002; { 2 bits: indicates 8-byte event record }
	kGeneralEventLengthBits = $00000003; { 2 bits: indicates variable length event record }
	kEventLen = 1;    { length of events in long words }
	kXEventLen = 2;
	kRestEventLen = kEventLen; { length of events in long words }
	kNoteEventLen = kEventLen;
	kControlEventLen = kEventLen;
	kMarkerEventLen = kEventLen;
	kXNoteEventLen = kXEventLen;
	kXControlEventLen = kXEventLen;
	kGeneralEventLen = kXEventLen; { 2 or more, however }
                                        { Universal Event Defines}
	kEventLengthFieldPos = 30;   { by looking at these two bits of the 1st or last word         }
	kEventLengthFieldWidth = 2;    { of an event you can determine the event length                }
                                        { length field: 0 & 1 => 1 long; 2 => 2 longs; 3 => variable length }
	kEventTypeFieldPos = 29;   { event type field for short events }
	kEventTypeFieldWidth = 3;    { short type is 3 bits }
	kXEventTypeFieldPos = 28;   { event type field for extended events }
	kXEventTypeFieldWidth = 4;    { extended type is 4 bits }
	kEventPartFieldPos = 24;
	kEventPartFieldWidth = 5;
	kXEventPartFieldPos = 16;   { in the 1st long word }
	kXEventPartFieldWidth = 12;   { Rest Events}
	kRestEventDurationFieldPos = 0;
	kRestEventDurationFieldWidth = 24;
	kRestEventDurationMax = ((1 shl kRestEventDurationFieldWidth) - 1); { Note Events}
	kNoteEventPitchFieldPos = 18;
	kNoteEventPitchFieldWidth = 6;
	kNoteEventPitchOffset = 32;   { add to value in pitch field to get actual pitch }
	kNoteEventVolumeFieldPos = 11;
	kNoteEventVolumeFieldWidth = 7;
	kNoteEventVolumeOffset = 0;    { add to value in volume field to get actual volume }
	kNoteEventDurationFieldPos = 0;
	kNoteEventDurationFieldWidth = 11;
	kNoteEventDurationMax = ((1 shl kNoteEventDurationFieldWidth) - 1);
	kXNoteEventPitchFieldPos = 0;    { in the 1st long word }
	kXNoteEventPitchFieldWidth = 16;
	kXNoteEventDurationFieldPos = 0;    { in the 2nd long word }
	kXNoteEventDurationFieldWidth = 22;
	kXNoteEventDurationMax = ((1 shl kXNoteEventDurationFieldWidth) - 1);
	kXNoteEventVolumeFieldPos = 22;   { in the 2nd long word }
	kXNoteEventVolumeFieldWidth = 7;    { Control Events}
	kControlEventControllerFieldPos = 16;
	kControlEventControllerFieldWidth = 8;
	kControlEventValueFieldPos = 0;
	kControlEventValueFieldWidth = 16;
	kXControlEventControllerFieldPos = 0; { in the 2nd long word }
	kXControlEventControllerFieldWidth = 16;
	kXControlEventValueFieldPos = 0;    { in the 1st long word }
	kXControlEventValueFieldWidth = 16;   { Knob Events}
	kKnobEventValueHighFieldPos = 0;    { 1st long word }
	kKnobEventValueHighFieldWidth = 16;
	kKnobEventKnobFieldPos = 16;   { 2nd long word }
	kKnobEventKnobFieldWidth = 14;
	kKnobEventValueLowFieldPos = 0;    { 2nd long word }
	kKnobEventValueLowFieldWidth = 16;   { Marker Events}
	kMarkerEventSubtypeFieldPos = 16;
	kMarkerEventSubtypeFieldWidth = 8;
	kMarkerEventValueFieldPos = 0;
	kMarkerEventValueFieldWidth = 16;   { General Events}
	kGeneralEventSubtypeFieldPos = 16;   { in the last long word }
	kGeneralEventSubtypeFieldWidth = 14;
	kGeneralEventLengthFieldPos = 0;    { in the 1st & last long words }
	kGeneralEventLengthFieldWidth = 16;

{$ifc TARGET_RT_LITTLE_ENDIAN}
const
	kEndMarkerValue = $00000060;

{$elsec}
const
	kEndMarkerValue = $60000000;

{$endc}  {TARGET_RT_LITTLE_ENDIAN}

{ macros for extracting various fields from the QuickTime event records}


{ General Event Defined Types}
const
	kGeneralEventNoteRequest = 1;    { Encapsulates NoteRequest data structure }
	kGeneralEventPartKey = 4;
	kGeneralEventTuneDifference = 5;    { Contains a standard sequence, with end marker, for the tune difference of a sequence piece (halts QuickTime 2.0 Music) }
	kGeneralEventAtomicInstrument = 6;    { Encapsulates AtomicInstrument record }
	kGeneralEventKnob = 7;    { knobID/knobValue pairs; smallest event is 4 longs }
	kGeneralEventMIDIChannel = 8;    { used in tune header, one longword identifies the midi channel it originally came from }
	kGeneralEventPartChange = 9;    { used in tune sequence, one longword identifies the tune part which can now take over this part's note channel (similar to program change) (halts QuickTime 2.0 Music)}
	kGeneralEventNoOp = 10;   { guaranteed to do nothing and be ignored. (halts QuickTime 2.0 Music) }
	kGeneralEventUsedNotes = 11;   { four longwords specifying which midi notes are actually used, 0..127 msb to lsb }
	kGeneralEventPartMix = 12;    { three longwords: Fixed volume, long balance, long flags }

{ Marker Event Defined Types       // marker is 60 ee vv vv in hex, where e = event type, and v = value}
const
	kMarkerEventEnd = 0;    { marker type 0 means: value 0 - stop, value != 0 - ignore}
	kMarkerEventBeat = 1;    { value 0 = single beat; anything else is 65536ths-of-a-beat (quarter note)}
	kMarkerEventTempo = 2;     { value same as beat marker, but indicates that a tempo event should be computed (based on where the next beat or tempo marker is) and emitted upon export}

const
	kCurrentlyNativeEndian = 1;
	kCurrentlyNotNativeEndian = 2;

{ UPP call backs }
{
 *  NewMusicMIDISendUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewMusicMIDISendUPP( userRoutine: MusicMIDISendProcPtr ): MusicMIDISendUPP; external name '_NewMusicMIDISendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
function NAPrerollNoteChannel(na: NoteAllocator; noteChannel_: NoteChannel): ComponentResult; external name '_NAPrerollNoteChannel';
=======
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NADisposeNoteChannel( na: NoteAllocator; noteChannel_: NoteChannel ): ComponentResult; external name '_NADisposeNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetNoteChannelInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetNoteChannelInfo( na: NoteAllocator; noteChannel_: NoteChannel; var index: SIGNEDLONG; var part: SIGNEDLONG ): ComponentResult; external name '_NAGetNoteChannelInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


>>>>>>> origin/fixes_2.4
{
 *  NAPrerollNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPrerollNoteChannel( na: NoteAllocator; noteChannel_: NoteChannel ): ComponentResult; external name '_NAPrerollNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAUnrollNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAUnrollNoteChannel( na: NoteAllocator; noteChannel_: NoteChannel ): ComponentResult; external name '_NAUnrollNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetNoteChannelVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetNoteChannelVolume( na: NoteAllocator; noteChannel_: NoteChannel; volume: Fixed ): ComponentResult; external name '_NASetNoteChannelVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAResetNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAResetNoteChannel( na: NoteAllocator; noteChannel_: NoteChannel ): ComponentResult; external name '_NAResetNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPlayNote()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPlayNote( na: NoteAllocator; noteChannel_: NoteChannel; pitch: SIGNEDLONG; velocity: SIGNEDLONG ): ComponentResult; external name '_NAPlayNote';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetController()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetController( na: NoteAllocator; noteChannel_: NoteChannel; controllerNumber: SIGNEDLONG; controllerValue: SIGNEDLONG ): ComponentResult; external name '_NASetController';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetKnob( na: NoteAllocator; noteChannel_: NoteChannel; knobNumber: SIGNEDLONG; knobValue: SIGNEDLONG ): ComponentResult; external name '_NASetKnob';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAFindNoteChannelTone()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAFindNoteChannelTone( na: NoteAllocator; noteChannel_: NoteChannel; var td: ToneDescription; var instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_NAFindNoteChannelTone';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetInstrumentNumber()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetInstrumentNumber( na: NoteAllocator; noteChannel_: NoteChannel; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_NASetInstrumentNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPickInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPickInstrument( na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; var sd: ToneDescription; flags: UNSIGNEDLONG; refCon: SIGNEDLONG; reserved1: SIGNEDLONG; reserved2: SIGNEDLONG ): ComponentResult; external name '_NAPickInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPickArrangement()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPickArrangement( na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; zero1: SIGNEDLONG; zero2: SIGNEDLONG; t: Track; songName: StringPtr ): ComponentResult; external name '_NAPickArrangement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAStuffToneDescription()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAStuffToneDescription( na: NoteAllocator; gmNumber: SIGNEDLONG; var td: ToneDescription ): ComponentResult; external name '_NAStuffToneDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NACopyrightDialog()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NACopyrightDialog( na: NoteAllocator; p: PicHandle; author: StringPtr; copyright: StringPtr; other: StringPtr; title: StringPtr; filterProc: ModalFilterUPP; refCon: SIGNEDLONG ): ComponentResult; external name '_NACopyrightDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
    kNADummyOneSelect = 29
    kNADummyTwoSelect = 30
}

{
 *  NAGetIndNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetIndNoteChannel( na: NoteAllocator; index: SIGNEDLONG; var nc: NoteChannel; var seed: SIGNEDLONG ): ComponentResult; external name '_NAGetIndNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetMIDIPorts()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetMIDIPorts( na: NoteAllocator; var inputPorts: QTMIDIPortListHandle; var outputPorts: QTMIDIPortListHandle ): ComponentResult; external name '_NAGetMIDIPorts';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetNoteRequest()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetNoteRequest( na: NoteAllocator; noteChannel_: NoteChannel; var nrOut: NoteRequest ): ComponentResult; external name '_NAGetNoteRequest';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASendMIDI()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASendMIDI( na: NoteAllocator; noteChannel_: NoteChannel; var mp: MusicMIDIPacket ): ComponentResult; external name '_NASendMIDI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAPickEditInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAPickEditInstrument( na: NoteAllocator; filterProc: ModalFilterUPP; prompt: StringPtr; refCon: SIGNEDLONG; nc: NoteChannel; ai: AtomicInstrument; flags: SIGNEDLONG ): ComponentResult; external name '_NAPickEditInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NANewNoteChannelFromAtomicInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NANewNoteChannelFromAtomicInstrument( na: NoteAllocator; instrument: AtomicInstrumentPtr; flags: SIGNEDLONG; var outChannel: NoteChannel ): ComponentResult; external name '_NANewNoteChannelFromAtomicInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetAtomicInstrument()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetAtomicInstrument( na: NoteAllocator; noteChannel_: NoteChannel; instrument: AtomicInstrumentPtr; flags: SIGNEDLONG ): ComponentResult; external name '_NASetAtomicInstrument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetKnob()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetKnob( na: NoteAllocator; noteChannel_: NoteChannel; knobNumber: SIGNEDLONG; var knobValue: SIGNEDLONG ): ComponentResult; external name '_NAGetKnob';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NATask()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NATask( na: NoteAllocator ): ComponentResult; external name '_NATask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetNoteChannelBalance()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetNoteChannelBalance( na: NoteAllocator; noteChannel_: NoteChannel; balance: SIGNEDLONG ): ComponentResult; external name '_NASetNoteChannelBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetInstrumentNumberInterruptSafe()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetInstrumentNumberInterruptSafe( na: NoteAllocator; noteChannel_: NoteChannel; instrumentNumber: SIGNEDLONG ): ComponentResult; external name '_NASetInstrumentNumberInterruptSafe';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NASetNoteChannelSoundLocalization()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NASetNoteChannelSoundLocalization( na: NoteAllocator; noteChannel_: NoteChannel; data: Handle ): ComponentResult; external name '_NASetNoteChannelSoundLocalization';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NAGetController()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function NAGetController( na: NoteAllocator; noteChannel_: NoteChannel; controllerNumber: SIGNEDLONG; var controllerValue: SIGNEDLONG ): ComponentResult; external name '_NAGetController';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


const
	kTuneQueueDepth = 8;     { Deepest you can queue tune segments }


type
	TuneStatusPtr = ^TuneStatus;
	TuneStatus = record
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
		tune:					SInt32Ptr;								{  currently playing tune  }
		tunePtr:				SInt32Ptr;								{  position within currently playing piece  }
		time:					TimeValue;								{  current tune time  }
		queueCount:				SInt16;								{  how many pieces queued up?  }
		queueSpots:				SInt16;								{  How many more tunepieces can be queued  }
		queueTime:				TimeValue;								{  How much time is queued up? (can be very inaccurate)  }
		reserved:				array [0..2] of SInt32;
=======
		tune: UNSIGNEDLONGPtr;                   { currently playing tune }
		tunePtr: UNSIGNEDLONGPtr;                { position within currently playing piece }
		time: TimeValue;                   { current tune time }
		queueCount: SInt16;             { how many pieces queued up? }
		queueSpots: SInt16;             { How many more tunepieces can be queued }
		queueTime: TimeValue;              { How much time is queued up? (can be very inaccurate) }
		reserved: array [0..2] of SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;

	TuneCallBackProcPtr = procedure( const (*var*) status: TuneStatus; refCon: SIGNEDLONG);
	TunePlayCallBackProcPtr = procedure( var event: UNSIGNEDLONG; seed: SIGNEDLONG; refCon: SIGNEDLONG );
	TuneCallBackUPP = TuneCallBackProcPtr;
	TunePlayCallBackUPP = TunePlayCallBackProcPtr;

type
	TunePlayer = ComponentInstance;
const
	kTunePlayerComponentType = FourCharCode('tune');


{
 *  TuneSetHeader()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetHeader( tp: TunePlayer; var header: UNSIGNEDLONG ): ComponentResult; external name '_TuneSetHeader';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetTimeBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetTimeBase( tp: TunePlayer; var tb: TimeBase ): ComponentResult; external name '_TuneGetTimeBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetTimeScale()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetTimeScale( tp: TunePlayer; scale: TimeScale ): ComponentResult; external name '_TuneSetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetTimeScale()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetTimeScale( tp: TunePlayer; var scale: TimeScale ): ComponentResult; external name '_TuneGetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetIndexedNoteChannel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetIndexedNoteChannel( tp: TunePlayer; i: SIGNEDLONG; var nc: NoteChannel ): ComponentResult; external name '_TuneGetIndexedNoteChannel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Values for when to start. }
const
	kTuneStartNow = 1;    { start after buffer is implied }
	kTuneDontClipNotes = 2;    { allow notes to finish their durations outside sample }
	kTuneExcludeEdgeNotes = 4;    { dont play notes that start at end of tune }
	kTuneQuickStart = 8;    { Leave all the controllers where they are, ignore start time }
	kTuneLoopUntil = 16;   { loop a queued tune if there's nothing else in the queue}
	kTunePlayDifference = 32;   { by default, the tune difference is skipped}
	kTunePlayConcurrent = 64;   { dont block the next tune sequence with this one}
	kTuneStartNewMaster = 16384;

{
 *  TuneQueue()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneQueue( tp: TunePlayer; var tune: UNSIGNEDLONG; tuneRate: Fixed; tuneStartPosition: UNSIGNEDLONG; tuneStopPosition: UNSIGNEDLONG; queueFlags: UNSIGNEDLONG; callBackProc: TuneCallBackUPP; refCon: SIGNEDLONG ): ComponentResult; external name '_TuneQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneInstant()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneInstant( tp: TunePlayer; var tune: UNSIGNEDLONG; tunePosition: UNSIGNEDLONG ): ComponentResult; external name '_TuneInstant';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetStatus()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetStatus( tp: TunePlayer; var status: TuneStatus ): ComponentResult; external name '_TuneGetStatus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Values for stopping. }
const
	kTuneStopFade = 1;    { do a quick, synchronous fadeout }
	kTuneStopSustain = 2;    { don't silece notes }
	kTuneStopInstant = 4;    { silence notes fast (else, decay them) }
	kTuneStopReleaseChannels = 8;     { afterwards, let the channels go }

{
 *  TuneStop()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneStop( tp: TunePlayer; stopFlags: SIGNEDLONG ): ComponentResult; external name '_TuneStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetVolume( tp: TunePlayer; volume: Fixed ): ComponentResult; external name '_TuneSetVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetVolume( tp: TunePlayer ): ComponentResult; external name '_TuneGetVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TunePreroll()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TunePreroll( tp: TunePlayer ): ComponentResult; external name '_TunePreroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneUnroll()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneUnroll( tp: TunePlayer ): ComponentResult; external name '_TuneUnroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetNoteChannels()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetNoteChannels( tp: TunePlayer; count: UNSIGNEDLONG; var noteChannelList: NoteChannel; playCallBackProc: TunePlayCallBackUPP; refCon: SIGNEDLONG ): ComponentResult; external name '_TuneSetNoteChannels';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetPartTranspose()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetPartTranspose( tp: TunePlayer; part: UNSIGNEDLONG; transpose: SIGNEDLONG; velocityShift: SIGNEDLONG ): ComponentResult; external name '_TuneSetPartTranspose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetNoteAllocator()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetNoteAllocator( tp: TunePlayer ): NoteAllocator; external name '_TuneGetNoteAllocator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetSofter()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetSofter( tp: TunePlayer; softer: SIGNEDLONG ): ComponentResult; external name '_TuneSetSofter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneTask()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneTask( tp: TunePlayer ): ComponentResult; external name '_TuneTask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetBalance()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetBalance( tp: TunePlayer; balance: SIGNEDLONG ): ComponentResult; external name '_TuneSetBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetSoundLocalization()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetSoundLocalization( tp: TunePlayer; data: Handle ): ComponentResult; external name '_TuneSetSoundLocalization';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneSetHeaderWithSize()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetHeaderWithSize( tp: TunePlayer; var header: UNSIGNEDLONG; size: UNSIGNEDLONG ): ComponentResult; external name '_TuneSetHeaderWithSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ flags for part mix. }
const
	kTuneMixMute = 1;    { disable a part }
	kTuneMixSolo = 2;     { if any parts soloed, play only soloed parts }


{
 *  TuneSetPartMix()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneSetPartMix( tp: TunePlayer; partNumber: UNSIGNEDLONG; volume: SIGNEDLONG; balance: SIGNEDLONG; mixFlags: SIGNEDLONG ): ComponentResult; external name '_TuneSetPartMix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TuneGetPartMix()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function TuneGetPartMix( tp: TunePlayer; partNumber: UNSIGNEDLONG; var volumeOut: SIGNEDLONG; var balanceOut: SIGNEDLONG; var mixFlagsOut: SIGNEDLONG ): ComponentResult; external name '_TuneGetPartMix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


type
	MusicOpWord = UNSIGNEDLONG;
	MusicOpWordPtr = ^MusicOpWord;
{    QuickTime Music Track Event Formats:

    At this time, QuickTime music tracks support 5 different event types -- REST events,
    short NOTE events, short CONTROL events, short GENERAL events, Long NOTE events, 
    long CONTROL events, and variable GENERAL events.
 
        ¥ REST Event (4 bytes/event):
    
            (0 0 0) (5-bit UNUSED) (24-bit Rest Duration)
        
        ¥ÊShort NOTE Events (4 bytes/event):
    
            (0 0 1) (5-bit Part) (6-bit Pitch) (7-bit Volume) (11-bit Duration)
        
            where:  Pitch is offset by 32 (Actual pitch = pitch field + 32)

        ¥ÊShort CONTROL Events (4 bytes/event):
    
            (0 1 0) (5-bit Part) (8-bit Controller) (1-bit UNUSED) (1-bit Sign) (7-bit MSB) (7-bit LSB)
                                                                         ( or 15-bit Signed Value)
        ¥ Short GENERAL Event (4 bytes/event):
    
            (0 1 1) (1-bit UNUSED) (12-bit Sub-Type) (16-bit Value)
    
        ¥ Long NOTE Events (8 bytes/event):
    
            (1 0 0 1) (12-bit Part) (1-bit UNUSED) (7-bit Pitch) (1-bit UNUSED) (7-bit Volume)
            (1 0) (8-bit UNUSED) (22-bit Duration)
        
        ¥ÊLong CONTROL Event (8 bytes/event):
        
            (1 0 1 0) (12-bit Part) (16-bit Value MSB) 
            (1 0) (14-bit Controller) (16-bit Value LSB)
    
        ¥ÊLong KNOB Event (8 bytes/event):
    
            (1 0 1 1) (12-bit Sub-Type) (16-bit Value MSB)
            (1 0) (14-bit KNOB) (16-bit Value LSB)
    
        ¥ÊVariable GENERAL Length Events (N bytes/event):
    
            (1 1 1 1) (12-bit Sub-Type) (16-bit Length)
                :
            (32-bit Data values)
                :
            (1 1) (14-bit UNUSED) (16-bit Length)
    
            where:  Length field is the number of LONG words in the record.
                    Lengths include the first and last long words (Minimum length = 2)
                
    The following event type values have not been used yet and are reserved for 
    future expansion:
        
        ¥ (1 0 0 0)     (8 bytes/event)
        ¥ (1 1 0 0)     (N bytes/event)
        ¥ (1 1 0 1)     (N bytes/event)
        ¥ (1 1 1 0)     (N bytes/event)
        
    For all events, the following generalizations apply:
    
        -   All duration values are specified in Millisecond units.
        -   Pitch values are intended to map directly to the MIDI key numbers.
        -   Controllers from 0 to 127 correspond to the standard MIDI controllers.
            Controllers greater than 127 correspond to other controls (i.e., Pitch Bend, 
            Key Pressure, and Channel Pressure).    
}

{ Defines for the implemented music event data fields}
const
	kRestEventType = $00000000; { lower 3-bits }
	kNoteEventType = $00000001; { lower 3-bits }
	kControlEventType = $00000002; { lower 3-bits }
	kMarkerEventType = $00000003; { lower 3-bits }
	kUndefined1EventType = $00000008; { 4-bits }
	kXNoteEventType = $00000009; { 4-bits }
	kXControlEventType = $0000000A; { 4-bits }
	kKnobEventType = $0000000B; { 4-bits }
	kUndefined2EventType = $0000000C; { 4-bits }
	kUndefined3EventType = $0000000D; { 4-bits }
	kUndefined4EventType = $0000000E; { 4-bits }
	kGeneralEventType = $0000000F; { 4-bits }
	kXEventLengthBits = $00000002; { 2 bits: indicates 8-byte event record }
	kGeneralEventLengthBits = $00000003; { 2 bits: indicates variable length event record }
	kEventLen = 1;    { length of events in long words }
	kXEventLen = 2;
	kRestEventLen = kEventLen; { length of events in long words }
	kNoteEventLen = kEventLen;
	kControlEventLen = kEventLen;
	kMarkerEventLen = kEventLen;
	kXNoteEventLen = kXEventLen;
	kXControlEventLen = kXEventLen;
	kGeneralEventLen = kXEventLen; { 2 or more, however }
                                        { Universal Event Defines}
	kEventLengthFieldPos = 30;   { by looking at these two bits of the 1st or last word         }
	kEventLengthFieldWidth = 2;    { of an event you can determine the event length                }
                                        { length field: 0 & 1 => 1 long; 2 => 2 longs; 3 => variable length }
	kEventTypeFieldPos = 29;   { event type field for short events }
	kEventTypeFieldWidth = 3;    { short type is 3 bits }
	kXEventTypeFieldPos = 28;   { event type field for extended events }
	kXEventTypeFieldWidth = 4;    { extended type is 4 bits }
	kEventPartFieldPos = 24;
	kEventPartFieldWidth = 5;
	kXEventPartFieldPos = 16;   { in the 1st long word }
	kXEventPartFieldWidth = 12;   { Rest Events}
	kRestEventDurationFieldPos = 0;
	kRestEventDurationFieldWidth = 24;
	kRestEventDurationMax = ((1 shl kRestEventDurationFieldWidth) - 1); { Note Events}
	kNoteEventPitchFieldPos = 18;
	kNoteEventPitchFieldWidth = 6;
	kNoteEventPitchOffset = 32;   { add to value in pitch field to get actual pitch }
	kNoteEventVolumeFieldPos = 11;
	kNoteEventVolumeFieldWidth = 7;
	kNoteEventVolumeOffset = 0;    { add to value in volume field to get actual volume }
	kNoteEventDurationFieldPos = 0;
	kNoteEventDurationFieldWidth = 11;
	kNoteEventDurationMax = ((1 shl kNoteEventDurationFieldWidth) - 1);
	kXNoteEventPitchFieldPos = 0;    { in the 1st long word }
	kXNoteEventPitchFieldWidth = 16;
	kXNoteEventDurationFieldPos = 0;    { in the 2nd long word }
	kXNoteEventDurationFieldWidth = 22;
	kXNoteEventDurationMax = ((1 shl kXNoteEventDurationFieldWidth) - 1);
	kXNoteEventVolumeFieldPos = 22;   { in the 2nd long word }
	kXNoteEventVolumeFieldWidth = 7;    { Control Events}
	kControlEventControllerFieldPos = 16;
	kControlEventControllerFieldWidth = 8;
	kControlEventValueFieldPos = 0;
	kControlEventValueFieldWidth = 16;
	kXControlEventControllerFieldPos = 0; { in the 2nd long word }
	kXControlEventControllerFieldWidth = 16;
	kXControlEventValueFieldPos = 0;    { in the 1st long word }
	kXControlEventValueFieldWidth = 16;   { Knob Events}
	kKnobEventValueHighFieldPos = 0;    { 1st long word }
	kKnobEventValueHighFieldWidth = 16;
	kKnobEventKnobFieldPos = 16;   { 2nd long word }
	kKnobEventKnobFieldWidth = 14;
	kKnobEventValueLowFieldPos = 0;    { 2nd long word }
	kKnobEventValueLowFieldWidth = 16;   { Marker Events}
	kMarkerEventSubtypeFieldPos = 16;
	kMarkerEventSubtypeFieldWidth = 8;
	kMarkerEventValueFieldPos = 0;
	kMarkerEventValueFieldWidth = 16;   { General Events}
	kGeneralEventSubtypeFieldPos = 16;   { in the last long word }
	kGeneralEventSubtypeFieldWidth = 14;
	kGeneralEventLengthFieldPos = 0;    { in the 1st & last long words }
	kGeneralEventLengthFieldWidth = 16;

{$ifc TARGET_RT_LITTLE_ENDIAN}
const
	kEndMarkerValue = $00000060;

{$elsec}
const
	kEndMarkerValue = $60000000;

{$endc}  {TARGET_RT_LITTLE_ENDIAN}

<<<<<<< HEAD
	{  macros for extracting various fields from the QuickTime event records }


	{  General Event Defined Types }
	kGeneralEventNoteRequest	= 1;							{  Encapsulates NoteRequest data structure  }
	kGeneralEventPartKey		= 4;
	kGeneralEventTuneDifference	= 5;							{  Contains a standard sequence, with end marker, for the tune difference of a sequence piece (halts QuickTime 2.0 Music)  }
	kGeneralEventAtomicInstrument = 6;							{  Encapsulates AtomicInstrument record  }
	kGeneralEventKnob			= 7;							{  knobID/knobValue pairs; smallest event is 4 longs  }
	kGeneralEventMIDIChannel	= 8;							{  used in tune header, one longword identifies the midi channel it originally came from  }
	kGeneralEventPartChange		= 9;							{  used in tune sequence, one longword identifies the tune part which can now take over this part's note channel (similar to program change) (halts QuickTime 2.0 Music) }
	kGeneralEventNoOp			= 10;							{  guaranteed to do nothing and be ignored. (halts QuickTime 2.0 Music)  }
	kGeneralEventUsedNotes		= 11;							{  four longwords specifying which midi notes are actually used, 0..127 msb to lsb  }
	kGeneralEventPartMix		= 12;							{  three longwords: Fixed volume, long balance, long flags  }

	{  Marker Event Defined Types       // marker is 60 ee vv vv in hex, where e = event type, and v = value }
	kMarkerEventEnd				= 0;							{  marker type 0 means: value 0 - stop, value != 0 - ignore }
	kMarkerEventBeat			= 1;							{  value 0 = single beat; anything else is 65536ths-of-a-beat (quarter note) }
	kMarkerEventTempo			= 2;							{  value same as beat marker, but indicates that a tempo event should be computed (based on where the next beat or tempo marker is) and emitted upon export }

	kCurrentlyNativeEndian		= 1;
	kCurrentlyNotNativeEndian	= 2;

	{	 UPP call backs 	}
	uppMusicMIDISendProcInfo = $00000FF0;
	uppMusicOfflineDataProcInfo = $00000FF0;
	uppTuneCallBackProcInfo = $000003C0;
	uppTunePlayCallBackProcInfo = $00000FC0;
	{
	 *  NewMusicMIDISendUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewMusicMIDISendUPP(userRoutine: MusicMIDISendProcPtr): MusicMIDISendUPP; external name '_NewMusicMIDISendUPP'; { old name was NewMusicMIDISendProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ macros for extracting various fields from the QuickTime event records}


{ General Event Defined Types}
const
	kGeneralEventNoteRequest = 1;    { Encapsulates NoteRequest data structure }
	kGeneralEventPartKey = 4;
	kGeneralEventTuneDifference = 5;    { Contains a standard sequence, with end marker, for the tune difference of a sequence piece (halts QuickTime 2.0 Music) }
	kGeneralEventAtomicInstrument = 6;    { Encapsulates AtomicInstrument record }
	kGeneralEventKnob = 7;    { knobID/knobValue pairs; smallest event is 4 longs }
	kGeneralEventMIDIChannel = 8;    { used in tune header, one longword identifies the midi channel it originally came from }
	kGeneralEventPartChange = 9;    { used in tune sequence, one longword identifies the tune part which can now take over this part's note channel (similar to program change) (halts QuickTime 2.0 Music)}
	kGeneralEventNoOp = 10;   { guaranteed to do nothing and be ignored. (halts QuickTime 2.0 Music) }
	kGeneralEventUsedNotes = 11;   { four longwords specifying which midi notes are actually used, 0..127 msb to lsb }
	kGeneralEventPartMix = 12;    { three longwords: Fixed volume, long balance, long flags }

{ Marker Event Defined Types       // marker is 60 ee vv vv in hex, where e = event type, and v = value}
const
	kMarkerEventEnd = 0;    { marker type 0 means: value 0 - stop, value != 0 - ignore}
	kMarkerEventBeat = 1;    { value 0 = single beat; anything else is 65536ths-of-a-beat (quarter note)}
	kMarkerEventTempo = 2;     { value same as beat marker, but indicates that a tempo event should be computed (based on where the next beat or tempo marker is) and emitted upon export}

const
	kCurrentlyNativeEndian = 1;
	kCurrentlyNotNativeEndian = 2;

{ UPP call backs }
>>>>>>> origin/fixes_2.4
{
 *  NewMusicMIDISendUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewMusicOfflineDataUPP( userRoutine: MusicOfflineDataProcPtr ): MusicOfflineDataUPP; external name '_NewMusicOfflineDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewMusicMIDISendUPP( userRoutine: MusicMIDISendProcPtr ): MusicMIDISendUPP; external name '_NewMusicMIDISendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewMusicOfflineDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewMusicOfflineDataUPP(userRoutine: MusicOfflineDataProcPtr): MusicOfflineDataUPP; external name '_NewMusicOfflineDataUPP'; { old name was NewMusicOfflineDataProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewMusicOfflineDataUPP( userRoutine: MusicOfflineDataProcPtr ): MusicOfflineDataUPP; external name '_NewMusicOfflineDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  NewTuneCallBackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTuneCallBackUPP( userRoutine: TuneCallBackProcPtr ): TuneCallBackUPP; external name '_NewTuneCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewTuneCallBackUPP(userRoutine: TuneCallBackProcPtr): TuneCallBackUPP; external name '_NewTuneCallBackUPP'; { old name was NewTuneCallBackProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewTuneCallBackUPP( userRoutine: TuneCallBackProcPtr ): TuneCallBackUPP; external name '_NewTuneCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  NewTunePlayCallBackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTunePlayCallBackUPP( userRoutine: TunePlayCallBackProcPtr ): TunePlayCallBackUPP; external name '_NewTunePlayCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewTunePlayCallBackUPP(userRoutine: TunePlayCallBackProcPtr): TunePlayCallBackUPP; external name '_NewTunePlayCallBackUPP'; { old name was NewTunePlayCallBackProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewTunePlayCallBackUPP( userRoutine: TunePlayCallBackProcPtr ): TunePlayCallBackUPP; external name '_NewTunePlayCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeMusicMIDISendUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeMusicMIDISendUPP( userUPP: MusicMIDISendUPP ); external name '_DisposeMusicMIDISendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeMusicMIDISendUPP(userUPP: MusicMIDISendUPP); external name '_DisposeMusicMIDISendUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeMusicMIDISendUPP( userUPP: MusicMIDISendUPP ); external name '_DisposeMusicMIDISendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeMusicOfflineDataUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeMusicOfflineDataUPP( userUPP: MusicOfflineDataUPP ); external name '_DisposeMusicOfflineDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeMusicOfflineDataUPP(userUPP: MusicOfflineDataUPP); external name '_DisposeMusicOfflineDataUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeMusicOfflineDataUPP( userUPP: MusicOfflineDataUPP ); external name '_DisposeMusicOfflineDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeTuneCallBackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTuneCallBackUPP( userUPP: TuneCallBackUPP ); external name '_DisposeTuneCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeTuneCallBackUPP(userUPP: TuneCallBackUPP); external name '_DisposeTuneCallBackUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeTuneCallBackUPP( userUPP: TuneCallBackUPP ); external name '_DisposeTuneCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeTunePlayCallBackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTunePlayCallBackUPP( userUPP: TunePlayCallBackUPP ); external name '_DisposeTunePlayCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeTunePlayCallBackUPP(userUPP: TunePlayCallBackUPP); external name '_DisposeTunePlayCallBackUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeTunePlayCallBackUPP( userUPP: TunePlayCallBackUPP ); external name '_DisposeTunePlayCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeMusicMIDISendUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeMusicMIDISendUPP( self: ComponentInstance; refCon: SIGNEDLONG; var mmp: MusicMIDIPacket; userUPP: MusicMIDISendUPP ): ComponentResult; external name '_InvokeMusicMIDISendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeMusicMIDISendUPP(self: ComponentInstance; refCon: SInt32; var mmp: MusicMIDIPacket; userRoutine: MusicMIDISendUPP): ComponentResult; external name '_InvokeMusicMIDISendUPP'; { old name was CallMusicMIDISendProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeMusicMIDISendUPP( self: ComponentInstance; refCon: SIGNEDLONG; var mmp: MusicMIDIPacket; userUPP: MusicMIDISendUPP ): ComponentResult; external name '_InvokeMusicMIDISendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeMusicOfflineDataUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeMusicOfflineDataUPP( SoundData: Ptr; numBytes: SIGNEDLONG; myRefCon: SIGNEDLONG; userUPP: MusicOfflineDataUPP ): ComponentResult; external name '_InvokeMusicOfflineDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeMusicOfflineDataUPP(SoundData: Ptr; numBytes: SInt32; myRefCon: SInt32; userRoutine: MusicOfflineDataUPP): ComponentResult; external name '_InvokeMusicOfflineDataUPP'; { old name was CallMusicOfflineDataProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeMusicOfflineDataUPP( SoundData: Ptr; numBytes: SIGNEDLONG; myRefCon: SIGNEDLONG; userUPP: MusicOfflineDataUPP ): ComponentResult; external name '_InvokeMusicOfflineDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeTuneCallBackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeTuneCallBackUPP( const (*var*) status: TuneStatus; refCon: SIGNEDLONG; userUPP: TuneCallBackUPP ); external name '_InvokeTuneCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure InvokeTuneCallBackUPP(const (*var*) status: TuneStatus; refCon: SInt32; userRoutine: TuneCallBackUPP); external name '_InvokeTuneCallBackUPP'; { old name was CallTuneCallBackProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvokeTuneCallBackUPP( const (*var*) status: TuneStatus; refCon: SIGNEDLONG; userUPP: TuneCallBackUPP ); external name '_InvokeTuneCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeTunePlayCallBackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeTunePlayCallBackUPP( var event: UNSIGNEDLONG; seed: SIGNEDLONG; refCon: SIGNEDLONG; userUPP: TunePlayCallBackUPP ); external name '_InvokeTunePlayCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ selectors for component calls }
const
	kQTMIDIGetMIDIPortsSelect = $0001;
	kQTMIDIUseSendPortSelect = $0002;
	kQTMIDISendMIDISelect = $0003;
	kMusicGetDescriptionSelect = $0001;
	kMusicGetPartSelect = $0002;
	kMusicSetPartSelect = $0003;
	kMusicSetPartInstrumentNumberSelect = $0004;
	kMusicGetPartInstrumentNumberSelect = $0005;
	kMusicStorePartInstrumentSelect = $0006;
	kMusicGetPartAtomicInstrumentSelect = $0009;
	kMusicSetPartAtomicInstrumentSelect = $000A;
	kMusicGetPartKnobSelect = $0010;
	kMusicSetPartKnobSelect = $0011;
	kMusicGetKnobSelect = $0012;
	kMusicSetKnobSelect = $0013;
	kMusicGetPartNameSelect = $0014;
	kMusicSetPartNameSelect = $0015;
	kMusicFindToneSelect = $0016;
	kMusicPlayNoteSelect = $0017;
	kMusicResetPartSelect = $0018;
	kMusicSetPartControllerSelect = $0019;
	kMusicGetPartControllerSelect = $001A;
	kMusicGetMIDIProcSelect = $001B;
	kMusicSetMIDIProcSelect = $001C;
	kMusicGetInstrumentNamesSelect = $001D;
	kMusicGetDrumNamesSelect = $001E;
	kMusicGetMasterTuneSelect = $001F;
	kMusicSetMasterTuneSelect = $0020;
	kMusicGetInstrumentAboutInfoSelect = $0022;
	kMusicGetDeviceConnectionSelect = $0023;
	kMusicUseDeviceConnectionSelect = $0024;
	kMusicGetKnobSettingStringsSelect = $0025;
	kMusicGetMIDIPortsSelect = $0026;
	kMusicSendMIDISelect = $0027;
	kMusicStartOfflineSelect = $0029;
	kMusicSetOfflineTimeToSelect = $002A;
	kMusicGetInstrumentKnobDescriptionSelect = $002B;
	kMusicGetDrumKnobDescriptionSelect = $002C;
	kMusicGetKnobDescriptionSelect = $002D;
	kMusicGetInfoTextSelect = $002E;
	kMusicGetInstrumentInfoSelect = $002F;
	kMusicTaskSelect = $0031;
	kMusicSetPartInstrumentNumberInterruptSafeSelect = $0032;
	kMusicSetPartSoundLocalizationSelect = $0033;
	kMusicGenericConfigureSelect = $0100;
	kMusicGenericGetPartSelect = $0101;
	kMusicGenericGetKnobListSelect = $0102;
	kMusicGenericSetResourceNumbersSelect = $0103;
	kMusicDerivedMIDISendSelect = $0200;
	kMusicDerivedSetKnobSelect = $0201;
	kMusicDerivedSetPartSelect = $0202;
	kMusicDerivedSetInstrumentSelect = $0203;
	kMusicDerivedSetPartInstrumentNumberSelect = $0204;
	kMusicDerivedSetMIDISelect = $0205;
	kMusicDerivedStorePartInstrumentSelect = $0206;
	kMusicDerivedOpenResFileSelect = $0207;
	kMusicDerivedCloseResFileSelect = $0208;
	kNARegisterMusicDeviceSelect = $0000;
	kNAUnregisterMusicDeviceSelect = $0001;
	kNAGetRegisteredMusicDeviceSelect = $0002;
	kNASaveMusicConfigurationSelect = $0003;
	kNANewNoteChannelSelect = $0004;
	kNADisposeNoteChannelSelect = $0005;
	kNAGetNoteChannelInfoSelect = $0006;
	kNAPrerollNoteChannelSelect = $0007;
	kNAUnrollNoteChannelSelect = $0008;
	kNASetNoteChannelVolumeSelect = $000B;
	kNAResetNoteChannelSelect = $000C;
	kNAPlayNoteSelect = $000D;
	kNASetControllerSelect = $000E;
	kNASetKnobSelect = $000F;
	kNAFindNoteChannelToneSelect = $0010;
	kNASetInstrumentNumberSelect = $0011;
	kNAPickInstrumentSelect = $0012;
	kNAPickArrangementSelect = $0013;
	kNAStuffToneDescriptionSelect = $001B;
	kNACopyrightDialogSelect = $001C;
	kNAGetIndNoteChannelSelect = $001F;
	kNAGetMIDIPortsSelect = $0021;
	kNAGetNoteRequestSelect = $0022;
	kNASendMIDISelect = $0023;
	kNAPickEditInstrumentSelect = $0024;
	kNANewNoteChannelFromAtomicInstrumentSelect = $0025;
	kNASetAtomicInstrumentSelect = $0026;
	kNAGetKnobSelect = $0028;
	kNATaskSelect = $0029;
	kNASetNoteChannelBalanceSelect = $002A;
	kNASetInstrumentNumberInterruptSafeSelect = $002B;
	kNASetNoteChannelSoundLocalizationSelect = $002C;
	kNAGetControllerSelect = $002D;
	kTuneSetHeaderSelect = $0004;
	kTuneGetTimeBaseSelect = $0005;
	kTuneSetTimeScaleSelect = $0006;
	kTuneGetTimeScaleSelect = $0007;
	kTuneGetIndexedNoteChannelSelect = $0008;
	kTuneQueueSelect = $000A;
	kTuneInstantSelect = $000B;
	kTuneGetStatusSelect = $000C;
	kTuneStopSelect = $000D;
	kTuneSetVolumeSelect = $0010;
	kTuneGetVolumeSelect = $0011;
	kTunePrerollSelect = $0012;
	kTuneUnrollSelect = $0013;
	kTuneSetNoteChannelsSelect = $0014;
	kTuneSetPartTransposeSelect = $0015;
	kTuneGetNoteAllocatorSelect = $0017;
	kTuneSetSofterSelect = $0018;
	kTuneTaskSelect = $0019;
	kTuneSetBalanceSelect = $001A;
	kTuneSetSoundLocalizationSelect = $001B;
	kTuneSetHeaderWithSizeSelect = $001C;
	kTuneSetPartMixSelect = $001D;
	kTuneGetPartMixSelect = $001E;

{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeTunePlayCallBackUPP( var event: UNSIGNEDLONG; seed: SIGNEDLONG; refCon: SIGNEDLONG; userUPP: TunePlayCallBackUPP ); external name '_InvokeTunePlayCallBackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ selectors for component calls }
const
	kQTMIDIGetMIDIPortsSelect = $0001;
	kQTMIDIUseSendPortSelect = $0002;
	kQTMIDISendMIDISelect = $0003;
	kMusicGetDescriptionSelect = $0001;
	kMusicGetPartSelect = $0002;
	kMusicSetPartSelect = $0003;
	kMusicSetPartInstrumentNumberSelect = $0004;
	kMusicGetPartInstrumentNumberSelect = $0005;
	kMusicStorePartInstrumentSelect = $0006;
	kMusicGetPartAtomicInstrumentSelect = $0009;
	kMusicSetPartAtomicInstrumentSelect = $000A;
	kMusicGetPartKnobSelect = $0010;
	kMusicSetPartKnobSelect = $0011;
	kMusicGetKnobSelect = $0012;
	kMusicSetKnobSelect = $0013;
	kMusicGetPartNameSelect = $0014;
	kMusicSetPartNameSelect = $0015;
	kMusicFindToneSelect = $0016;
	kMusicPlayNoteSelect = $0017;
	kMusicResetPartSelect = $0018;
	kMusicSetPartControllerSelect = $0019;
	kMusicGetPartControllerSelect = $001A;
	kMusicGetMIDIProcSelect = $001B;
	kMusicSetMIDIProcSelect = $001C;
	kMusicGetInstrumentNamesSelect = $001D;
	kMusicGetDrumNamesSelect = $001E;
	kMusicGetMasterTuneSelect = $001F;
	kMusicSetMasterTuneSelect = $0020;
	kMusicGetInstrumentAboutInfoSelect = $0022;
	kMusicGetDeviceConnectionSelect = $0023;
	kMusicUseDeviceConnectionSelect = $0024;
	kMusicGetKnobSettingStringsSelect = $0025;
	kMusicGetMIDIPortsSelect = $0026;
	kMusicSendMIDISelect = $0027;
	kMusicStartOfflineSelect = $0029;
	kMusicSetOfflineTimeToSelect = $002A;
	kMusicGetInstrumentKnobDescriptionSelect = $002B;
	kMusicGetDrumKnobDescriptionSelect = $002C;
	kMusicGetKnobDescriptionSelect = $002D;
	kMusicGetInfoTextSelect = $002E;
	kMusicGetInstrumentInfoSelect = $002F;
	kMusicTaskSelect = $0031;
	kMusicSetPartInstrumentNumberInterruptSafeSelect = $0032;
	kMusicSetPartSoundLocalizationSelect = $0033;
	kMusicGenericConfigureSelect = $0100;
	kMusicGenericGetPartSelect = $0101;
	kMusicGenericGetKnobListSelect = $0102;
	kMusicGenericSetResourceNumbersSelect = $0103;
	kMusicDerivedMIDISendSelect = $0200;
	kMusicDerivedSetKnobSelect = $0201;
	kMusicDerivedSetPartSelect = $0202;
	kMusicDerivedSetInstrumentSelect = $0203;
	kMusicDerivedSetPartInstrumentNumberSelect = $0204;
	kMusicDerivedSetMIDISelect = $0205;
	kMusicDerivedStorePartInstrumentSelect = $0206;
	kMusicDerivedOpenResFileSelect = $0207;
	kMusicDerivedCloseResFileSelect = $0208;
	kNARegisterMusicDeviceSelect = $0000;
	kNAUnregisterMusicDeviceSelect = $0001;
	kNAGetRegisteredMusicDeviceSelect = $0002;
	kNASaveMusicConfigurationSelect = $0003;
	kNANewNoteChannelSelect = $0004;
	kNADisposeNoteChannelSelect = $0005;
	kNAGetNoteChannelInfoSelect = $0006;
	kNAPrerollNoteChannelSelect = $0007;
	kNAUnrollNoteChannelSelect = $0008;
	kNASetNoteChannelVolumeSelect = $000B;
	kNAResetNoteChannelSelect = $000C;
	kNAPlayNoteSelect = $000D;
	kNASetControllerSelect = $000E;
	kNASetKnobSelect = $000F;
	kNAFindNoteChannelToneSelect = $0010;
	kNASetInstrumentNumberSelect = $0011;
	kNAPickInstrumentSelect = $0012;
	kNAPickArrangementSelect = $0013;
	kNAStuffToneDescriptionSelect = $001B;
	kNACopyrightDialogSelect = $001C;
	kNAGetIndNoteChannelSelect = $001F;
	kNAGetMIDIPortsSelect = $0021;
	kNAGetNoteRequestSelect = $0022;
	kNASendMIDISelect = $0023;
	kNAPickEditInstrumentSelect = $0024;
	kNANewNoteChannelFromAtomicInstrumentSelect = $0025;
	kNASetAtomicInstrumentSelect = $0026;
	kNAGetKnobSelect = $0028;
	kNATaskSelect = $0029;
	kNASetNoteChannelBalanceSelect = $002A;
	kNASetInstrumentNumberInterruptSafeSelect = $002B;
	kNASetNoteChannelSoundLocalizationSelect = $002C;
	kNAGetControllerSelect = $002D;
	kTuneSetHeaderSelect = $0004;
	kTuneGetTimeBaseSelect = $0005;
	kTuneSetTimeScaleSelect = $0006;
	kTuneGetTimeScaleSelect = $0007;
	kTuneGetIndexedNoteChannelSelect = $0008;
	kTuneQueueSelect = $000A;
	kTuneInstantSelect = $000B;
	kTuneGetStatusSelect = $000C;
	kTuneStopSelect = $000D;
	kTuneSetVolumeSelect = $0010;
	kTuneGetVolumeSelect = $0011;
	kTunePrerollSelect = $0012;
	kTuneUnrollSelect = $0013;
	kTuneSetNoteChannelsSelect = $0014;
	kTuneSetPartTransposeSelect = $0015;
	kTuneGetNoteAllocatorSelect = $0017;
	kTuneSetSofterSelect = $0018;
	kTuneTaskSelect = $0019;
	kTuneSetBalanceSelect = $001A;
	kTuneSetSoundLocalizationSelect = $001B;
	kTuneSetHeaderWithSizeSelect = $001C;
	kTuneSetPartMixSelect = $001D;
	kTuneGetPartMixSelect = $001E;

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
