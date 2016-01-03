{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/Collections.h
 
     Contains:   Collection Manager Interfaces
                 The contents of this header file are deprecated.
                 Use Foundation or CoreFoundation collection objects instead.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1989-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1989-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       Collections.p
=======
     File:       CarbonCore/Collections.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Collection Manager Interfaces
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1989-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1989-2008 by Apple Computer, Inc., all rights reserved
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

unit Collections;
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

<<<<<<< HEAD
{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

=======
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,MixedMode;
=======
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

<<<<<<< HEAD
{$ALIGN MAC68K}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{***********}
{ Constants }
{***********}
{ Convenience constants for functions which optionally return values }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kCollectionDontWantTag = 0;
	kCollectionDontWantId = 0;
	kCollectionDontWantSize = 0;
	kCollectionDontWantAttributes = 0;
	kCollectionDontWantIndex = 0;
	kCollectionDontWantData = 0;


{ attributes bits }
const
	kCollectionNoAttributes = $00000000; { no attributes bits set }
	kCollectionAllAttributes = -1; { all attributes bits set }
	kCollectionUserAttributes = $0000FFFF; { user attributes bits }
	kCollectionDefaultAttributes = $40000000; { default attributes - unlocked, persistent }
<<<<<<< HEAD
<<<<<<< HEAD


{ 
    Attribute bits 0 through 15 (entire low word) are reserved for use by the application.
    Attribute bits 16 through 31 (entire high word) are reserved for use by the Collection Manager.
    Only bits 31 (kCollectionLockBit) and 30 (kCollectionPersistenceBit) currently have meaning.
}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
const
	kCollectionUser0Bit = 0;
	kCollectionUser1Bit = 1;
	kCollectionUser2Bit = 2;
	kCollectionUser3Bit = 3;
	kCollectionUser4Bit = 4;
	kCollectionUser5Bit = 5;
	kCollectionUser6Bit = 6;
	kCollectionUser7Bit = 7;
	kCollectionUser8Bit = 8;
	kCollectionUser9Bit = 9;
	kCollectionUser10Bit = 10;
	kCollectionUser11Bit = 11;
	kCollectionUser12Bit = 12;
	kCollectionUser13Bit = 13;
	kCollectionUser14Bit = 14;
	kCollectionUser15Bit = 15;
	kCollectionReserved0Bit = 16;
	kCollectionReserved1Bit = 17;
	kCollectionReserved2Bit = 18;
	kCollectionReserved3Bit = 19;
	kCollectionReserved4Bit = 20;
	kCollectionReserved5Bit = 21;
	kCollectionReserved6Bit = 22;
	kCollectionReserved7Bit = 23;
	kCollectionReserved8Bit = 24;
	kCollectionReserved9Bit = 25;
	kCollectionReserved10Bit = 26;
	kCollectionReserved11Bit = 27;
	kCollectionReserved12Bit = 28;
	kCollectionReserved13Bit = 29;
	kCollectionPersistenceBit = 30;
	kCollectionLockBit = 31;


{ attribute masks }
<<<<<<< HEAD
const
=======
const
	kCollectionUser0Bit = 0;
	kCollectionUser1Bit = 1;
	kCollectionUser2Bit = 2;
	kCollectionUser3Bit = 3;
	kCollectionUser4Bit = 4;
	kCollectionUser5Bit = 5;
	kCollectionUser6Bit = 6;
	kCollectionUser7Bit = 7;
	kCollectionUser8Bit = 8;
	kCollectionUser9Bit = 9;
	kCollectionUser10Bit = 10;
	kCollectionUser11Bit = 11;
	kCollectionUser12Bit = 12;
	kCollectionUser13Bit = 13;
	kCollectionUser14Bit = 14;
	kCollectionUser15Bit = 15;
	kCollectionReserved0Bit = 16;
	kCollectionReserved1Bit = 17;
	kCollectionReserved2Bit = 18;
	kCollectionReserved3Bit = 19;
	kCollectionReserved4Bit = 20;
	kCollectionReserved5Bit = 21;
	kCollectionReserved6Bit = 22;
	kCollectionReserved7Bit = 23;
	kCollectionReserved8Bit = 24;
	kCollectionReserved9Bit = 25;
	kCollectionReserved10Bit = 26;
	kCollectionReserved11Bit = 27;
	kCollectionReserved12Bit = 28;
	kCollectionReserved13Bit = 29;
	kCollectionPersistenceBit = 30;
	kCollectionLockBit = 31;


{ attribute masks }
const
>>>>>>> graemeg/cpstrnew
=======
const
>>>>>>> graemeg/cpstrnew
=======


{ 
    Attribute bits 0 through 15 (entire low word) are reserved for use by the application.
    Attribute bits 16 through 31 (entire high word) are reserved for use by the Collection Manager.
    Only bits 31 (kCollectionLockBit) and 30 (kCollectionPersistenceBit) currently have meaning.
}
const
	kCollectionUser0Bit = 0;
	kCollectionUser1Bit = 1;
	kCollectionUser2Bit = 2;
	kCollectionUser3Bit = 3;
	kCollectionUser4Bit = 4;
	kCollectionUser5Bit = 5;
	kCollectionUser6Bit = 6;
	kCollectionUser7Bit = 7;
	kCollectionUser8Bit = 8;
	kCollectionUser9Bit = 9;
	kCollectionUser10Bit = 10;
	kCollectionUser11Bit = 11;
	kCollectionUser12Bit = 12;
	kCollectionUser13Bit = 13;
	kCollectionUser14Bit = 14;
	kCollectionUser15Bit = 15;
	kCollectionReserved0Bit = 16;
	kCollectionReserved1Bit = 17;
	kCollectionReserved2Bit = 18;
	kCollectionReserved3Bit = 19;
	kCollectionReserved4Bit = 20;
	kCollectionReserved5Bit = 21;
	kCollectionReserved6Bit = 22;
	kCollectionReserved7Bit = 23;
	kCollectionReserved8Bit = 24;
	kCollectionReserved9Bit = 25;
	kCollectionReserved10Bit = 26;
	kCollectionReserved11Bit = 27;
	kCollectionReserved12Bit = 28;
	kCollectionReserved13Bit = 29;
	kCollectionPersistenceBit = 30;
	kCollectionLockBit = 31;


{ attribute masks }
const
>>>>>>> graemeg/cpstrnew
=======


{ 
    Attribute bits 0 through 15 (entire low word) are reserved for use by the application.
    Attribute bits 16 through 31 (entire high word) are reserved for use by the Collection Manager.
    Only bits 31 (kCollectionLockBit) and 30 (kCollectionPersistenceBit) currently have meaning.
}
const
	kCollectionUser0Bit = 0;
	kCollectionUser1Bit = 1;
	kCollectionUser2Bit = 2;
	kCollectionUser3Bit = 3;
	kCollectionUser4Bit = 4;
	kCollectionUser5Bit = 5;
	kCollectionUser6Bit = 6;
	kCollectionUser7Bit = 7;
	kCollectionUser8Bit = 8;
	kCollectionUser9Bit = 9;
	kCollectionUser10Bit = 10;
	kCollectionUser11Bit = 11;
	kCollectionUser12Bit = 12;
	kCollectionUser13Bit = 13;
	kCollectionUser14Bit = 14;
	kCollectionUser15Bit = 15;
	kCollectionReserved0Bit = 16;
	kCollectionReserved1Bit = 17;
	kCollectionReserved2Bit = 18;
	kCollectionReserved3Bit = 19;
	kCollectionReserved4Bit = 20;
	kCollectionReserved5Bit = 21;
	kCollectionReserved6Bit = 22;
	kCollectionReserved7Bit = 23;
	kCollectionReserved8Bit = 24;
	kCollectionReserved9Bit = 25;
	kCollectionReserved10Bit = 26;
	kCollectionReserved11Bit = 27;
	kCollectionReserved12Bit = 28;
	kCollectionReserved13Bit = 29;
	kCollectionPersistenceBit = 30;
	kCollectionLockBit = 31;


{ attribute masks }
const
>>>>>>> origin/cpstrnew
	kCollectionUser0Mask = 1 shl kCollectionUser0Bit;
	kCollectionUser1Mask = 1 shl kCollectionUser1Bit;
	kCollectionUser2Mask = 1 shl kCollectionUser2Bit;
	kCollectionUser3Mask = 1 shl kCollectionUser3Bit;
	kCollectionUser4Mask = 1 shl kCollectionUser4Bit;
	kCollectionUser5Mask = 1 shl kCollectionUser5Bit;
	kCollectionUser6Mask = 1 shl kCollectionUser6Bit;
	kCollectionUser7Mask = 1 shl kCollectionUser7Bit;
	kCollectionUser8Mask = 1 shl kCollectionUser8Bit;
	kCollectionUser9Mask = 1 shl kCollectionUser9Bit;
	kCollectionUser10Mask = 1 shl kCollectionUser10Bit;
	kCollectionUser11Mask = 1 shl kCollectionUser11Bit;
	kCollectionUser12Mask = 1 shl kCollectionUser12Bit;
	kCollectionUser13Mask = 1 shl kCollectionUser13Bit;
	kCollectionUser14Mask = 1 shl kCollectionUser14Bit;
	kCollectionUser15Mask = 1 shl kCollectionUser15Bit;
	kCollectionReserved0Mask = 1 shl kCollectionReserved0Bit;
	kCollectionReserved1Mask = 1 shl kCollectionReserved1Bit;
	kCollectionReserved2Mask = 1 shl kCollectionReserved2Bit;
	kCollectionReserved3Mask = 1 shl kCollectionReserved3Bit;
	kCollectionReserved4Mask = 1 shl kCollectionReserved4Bit;
	kCollectionReserved5Mask = 1 shl kCollectionReserved5Bit;
	kCollectionReserved6Mask = 1 shl kCollectionReserved6Bit;
	kCollectionReserved7Mask = 1 shl kCollectionReserved7Bit;
	kCollectionReserved8Mask = 1 shl kCollectionReserved8Bit;
	kCollectionReserved9Mask = 1 shl kCollectionReserved9Bit;
	kCollectionReserved10Mask = 1 shl kCollectionReserved10Bit;
	kCollectionReserved11Mask = 1 shl kCollectionReserved11Bit;
	kCollectionReserved12Mask = 1 shl kCollectionReserved12Bit;
	kCollectionReserved13Mask = 1 shl kCollectionReserved13Bit;
	kCollectionPersistenceMask = 1 shl kCollectionPersistenceBit;
	kCollectionLockMask = 1 shl kCollectionLockBit;


{*********}
{ Types   }
{*********}
{ abstract data type for a collection }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Collection = ^OpaqueCollection; { an opaque type }
	OpaqueCollection = record end;
=======
	Collection = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	Collection = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	Collection = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	Collection = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
{ collection member 4 byte tag }
type
	CollectionTag = FourCharCode;
	CollectionFlattenProcPtr = function( size: SInt32; data: UnivPtr; refCon: UnivPtr ): OSErr;
	CollectionExceptionProcPtr = function( c: Collection; status: OSErr ): OSErr;
	CollectionFlattenUPP = CollectionFlattenProcPtr;
	CollectionExceptionUPP = CollectionExceptionProcPtr;
{
 *  NewCollectionFlattenUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCollectionFlattenUPP( userRoutine: CollectionFlattenProcPtr ): CollectionFlattenUPP; external name '_NewCollectionFlattenUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
const
	kCollectionDontWantTag = 0;
	kCollectionDontWantId = 0;
	kCollectionDontWantSize = 0;
	kCollectionDontWantAttributes = 0;
	kCollectionDontWantIndex = 0;
	kCollectionDontWantData = 0;


{ attributes bits }
const
	kCollectionNoAttributes = $00000000; { no attributes bits set }
	kCollectionAllAttributes = -1; { all attributes bits set }
	kCollectionUserAttributes = $0000FFFF; { user attributes bits }
	kCollectionDefaultAttributes = $40000000; { default attributes - unlocked, persistent }


{ 
    Attribute bits 0 through 15 (entire low word) are reserved for use by the application.
    Attribute bits 16 through 31 (entire high word) are reserved for use by the Collection Manager.
    Only bits 31 (kCollectionLockBit) and 30 (kCollectionPersistenceBit) currently have meaning.
}
const
	kCollectionUser0Bit = 0;
	kCollectionUser1Bit = 1;
	kCollectionUser2Bit = 2;
	kCollectionUser3Bit = 3;
	kCollectionUser4Bit = 4;
	kCollectionUser5Bit = 5;
	kCollectionUser6Bit = 6;
	kCollectionUser7Bit = 7;
	kCollectionUser8Bit = 8;
	kCollectionUser9Bit = 9;
	kCollectionUser10Bit = 10;
	kCollectionUser11Bit = 11;
	kCollectionUser12Bit = 12;
	kCollectionUser13Bit = 13;
	kCollectionUser14Bit = 14;
	kCollectionUser15Bit = 15;
	kCollectionReserved0Bit = 16;
	kCollectionReserved1Bit = 17;
	kCollectionReserved2Bit = 18;
	kCollectionReserved3Bit = 19;
	kCollectionReserved4Bit = 20;
	kCollectionReserved5Bit = 21;
	kCollectionReserved6Bit = 22;
	kCollectionReserved7Bit = 23;
	kCollectionReserved8Bit = 24;
	kCollectionReserved9Bit = 25;
	kCollectionReserved10Bit = 26;
	kCollectionReserved11Bit = 27;
	kCollectionReserved12Bit = 28;
	kCollectionReserved13Bit = 29;
	kCollectionPersistenceBit = 30;
	kCollectionLockBit = 31;


{ attribute masks }
const
<<<<<<< HEAD
	uppCollectionFlattenProcInfo = $00000FE0;
	uppCollectionExceptionProcInfo = $000002E0;
	{
	 *  NewCollectionFlattenUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewCollectionFlattenUPP(userRoutine: CollectionFlattenProcPtr): CollectionFlattenUPP; external name '_NewCollectionFlattenUPP'; { old name was NewCollectionFlattenProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kCollectionUser0Mask = 1 shl kCollectionUser0Bit;
	kCollectionUser1Mask = 1 shl kCollectionUser1Bit;
	kCollectionUser2Mask = 1 shl kCollectionUser2Bit;
	kCollectionUser3Mask = 1 shl kCollectionUser3Bit;
	kCollectionUser4Mask = 1 shl kCollectionUser4Bit;
	kCollectionUser5Mask = 1 shl kCollectionUser5Bit;
	kCollectionUser6Mask = 1 shl kCollectionUser6Bit;
	kCollectionUser7Mask = 1 shl kCollectionUser7Bit;
	kCollectionUser8Mask = 1 shl kCollectionUser8Bit;
	kCollectionUser9Mask = 1 shl kCollectionUser9Bit;
	kCollectionUser10Mask = 1 shl kCollectionUser10Bit;
	kCollectionUser11Mask = 1 shl kCollectionUser11Bit;
	kCollectionUser12Mask = 1 shl kCollectionUser12Bit;
	kCollectionUser13Mask = 1 shl kCollectionUser13Bit;
	kCollectionUser14Mask = 1 shl kCollectionUser14Bit;
	kCollectionUser15Mask = 1 shl kCollectionUser15Bit;
	kCollectionReserved0Mask = 1 shl kCollectionReserved0Bit;
	kCollectionReserved1Mask = 1 shl kCollectionReserved1Bit;
	kCollectionReserved2Mask = 1 shl kCollectionReserved2Bit;
	kCollectionReserved3Mask = 1 shl kCollectionReserved3Bit;
	kCollectionReserved4Mask = 1 shl kCollectionReserved4Bit;
	kCollectionReserved5Mask = 1 shl kCollectionReserved5Bit;
	kCollectionReserved6Mask = 1 shl kCollectionReserved6Bit;
	kCollectionReserved7Mask = 1 shl kCollectionReserved7Bit;
	kCollectionReserved8Mask = 1 shl kCollectionReserved8Bit;
	kCollectionReserved9Mask = 1 shl kCollectionReserved9Bit;
	kCollectionReserved10Mask = 1 shl kCollectionReserved10Bit;
	kCollectionReserved11Mask = 1 shl kCollectionReserved11Bit;
	kCollectionReserved12Mask = 1 shl kCollectionReserved12Bit;
	kCollectionReserved13Mask = 1 shl kCollectionReserved13Bit;
	kCollectionPersistenceMask = 1 shl kCollectionPersistenceBit;
	kCollectionLockMask = 1 shl kCollectionLockBit;


{*********}
{ Types   }
{*********}
{ abstract data type for a collection }
type
	Collection = ^SInt32; { an opaque type }
{ collection member 4 byte tag }
type
	CollectionTag = FourCharCode;
	CollectionFlattenProcPtr = function( size: SInt32; data: UnivPtr; refCon: UnivPtr ): OSErr;
	CollectionExceptionProcPtr = function( c: Collection; status: OSErr ): OSErr;
	CollectionFlattenUPP = CollectionFlattenProcPtr;
	CollectionExceptionUPP = CollectionExceptionProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewCollectionFlattenUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCollectionExceptionUPP( userRoutine: CollectionExceptionProcPtr ): CollectionExceptionUPP; external name '_NewCollectionExceptionUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCollectionFlattenUPP( userRoutine: CollectionFlattenProcPtr ): CollectionFlattenUPP; external name '_NewCollectionFlattenUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewCollectionExceptionUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewCollectionExceptionUPP(userRoutine: CollectionExceptionProcPtr): CollectionExceptionUPP; external name '_NewCollectionExceptionUPP'; { old name was NewCollectionExceptionProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewCollectionExceptionUPP( userRoutine: CollectionExceptionProcPtr ): CollectionExceptionUPP; external name '_NewCollectionExceptionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeCollectionFlattenUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeCollectionFlattenUPP( userUPP: CollectionFlattenUPP ); external name '_DisposeCollectionFlattenUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeCollectionFlattenUPP(userUPP: CollectionFlattenUPP); external name '_DisposeCollectionFlattenUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeCollectionFlattenUPP( userUPP: CollectionFlattenUPP ); external name '_DisposeCollectionFlattenUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeCollectionExceptionUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeCollectionExceptionUPP( userUPP: CollectionExceptionUPP ); external name '_DisposeCollectionExceptionUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeCollectionExceptionUPP(userUPP: CollectionExceptionUPP); external name '_DisposeCollectionExceptionUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeCollectionExceptionUPP( userUPP: CollectionExceptionUPP ); external name '_DisposeCollectionExceptionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeCollectionFlattenUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeCollectionFlattenUPP( size: SInt32; data: UnivPtr; refCon: UnivPtr; userUPP: CollectionFlattenUPP ): OSErr; external name '_InvokeCollectionFlattenUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeCollectionFlattenUPP(size: SInt32; data: UnivPtr; refCon: UnivPtr; userRoutine: CollectionFlattenUPP): OSErr; external name '_InvokeCollectionFlattenUPP'; { old name was CallCollectionFlattenProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeCollectionFlattenUPP( size: SInt32; data: UnivPtr; refCon: UnivPtr; userUPP: CollectionFlattenUPP ): OSErr; external name '_InvokeCollectionFlattenUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeCollectionExceptionUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeCollectionExceptionUPP( c: Collection; status: OSErr; userUPP: CollectionExceptionUPP ): OSErr; external name '_InvokeCollectionExceptionUPP';
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

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeCollectionExceptionUPP(c: Collection; status: OSErr; userRoutine: CollectionExceptionUPP): OSErr; external name '_InvokeCollectionExceptionUPP'; { old name was CallCollectionExceptionProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeCollectionExceptionUPP( c: Collection; status: OSErr; userUPP: CollectionExceptionUPP ): OSErr; external name '_InvokeCollectionExceptionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{*******************************************}
{************ Public interfaces ************}
{*******************************************}
{
 *  NewCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function NewCollection: Collection; external name '_NewCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function NewCollection: Collection; external name '_NewCollection';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  DisposeCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
procedure DisposeCollection( c: Collection ); external name '_DisposeCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
procedure DisposeCollection(c: Collection); external name '_DisposeCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeCollection( c: Collection ); external name '_DisposeCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CloneCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function CloneCollection( c: Collection ): Collection; external name '_CloneCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function CloneCollection(c: Collection): Collection; external name '_CloneCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CloneCollection( c: Collection ): Collection; external name '_CloneCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CountCollectionOwners()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function CountCollectionOwners( c: Collection ): SInt32; external name '_CountCollectionOwners';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function CountCollectionOwners(c: Collection): SInt32; external name '_CountCollectionOwners';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CountCollectionOwners( c: Collection ): SInt32; external name '_CountCollectionOwners';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RetainCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RetainCollection( c: Collection ): OSStatus; external name '_RetainCollection';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function RetainCollection(c: Collection): OSStatus; external name '_RetainCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RetainCollection( c: Collection ): OSStatus; external name '_RetainCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  ReleaseCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function ReleaseCollection( c: Collection ): OSStatus; external name '_ReleaseCollection';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function ReleaseCollection(c: Collection): OSStatus; external name '_ReleaseCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ReleaseCollection( c: Collection ): OSStatus; external name '_ReleaseCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  GetCollectionRetainCount()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetCollectionRetainCount( c: Collection ): ItemCount; external name '_GetCollectionRetainCount';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetCollectionRetainCount(c: Collection): ItemCount; external name '_GetCollectionRetainCount';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetCollectionRetainCount( c: Collection ): ItemCount; external name '_GetCollectionRetainCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CopyCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function CopyCollection( srcCollection: Collection; dstCollection: Collection ): Collection; external name '_CopyCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function CopyCollection(srcCollection: Collection; dstCollection: Collection): Collection; external name '_CopyCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CopyCollection( srcCollection: Collection; dstCollection: Collection ): Collection; external name '_CopyCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetCollectionDefaultAttributes()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetCollectionDefaultAttributes( c: Collection ): SInt32; external name '_GetCollectionDefaultAttributes';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetCollectionDefaultAttributes(c: Collection): SInt32; external name '_GetCollectionDefaultAttributes';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetCollectionDefaultAttributes( c: Collection ): SInt32; external name '_GetCollectionDefaultAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  SetCollectionDefaultAttributes()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
procedure SetCollectionDefaultAttributes( c: Collection; whichAttributes: SInt32; newAttributes: SInt32 ); external name '_SetCollectionDefaultAttributes';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
procedure SetCollectionDefaultAttributes(c: Collection; whichAttributes: SInt32; newAttributes: SInt32); external name '_SetCollectionDefaultAttributes';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetCollectionDefaultAttributes( c: Collection; whichAttributes: SInt32; newAttributes: SInt32 ); external name '_SetCollectionDefaultAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CountCollectionItems()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function CountCollectionItems( c: Collection ): SInt32; external name '_CountCollectionItems';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function CountCollectionItems(c: Collection): SInt32; external name '_CountCollectionItems';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CountCollectionItems( c: Collection ): SInt32; external name '_CountCollectionItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  AddCollectionItem()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function AddCollectionItem( c: Collection; tag: CollectionTag; id: SInt32; itemSize: SInt32; itemData: {const} UnivPtr ): OSErr; external name '_AddCollectionItem';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function AddCollectionItem(c: Collection; tag: CollectionTag; id: SInt32; itemSize: SInt32; itemData: UnivPtr): OSErr; external name '_AddCollectionItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AddCollectionItem( c: Collection; tag: CollectionTag; id: SInt32; itemSize: SInt32; itemData: {const} UnivPtr ): OSErr; external name '_AddCollectionItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetCollectionItem()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetCollectionItem( c: Collection; tag: CollectionTag; id: SInt32; var itemSize: SInt32; itemData: UnivPtr ): OSErr; external name '_GetCollectionItem';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetCollectionItem(c: Collection; tag: CollectionTag; id: SInt32; var itemSize: SInt32; itemData: UnivPtr): OSErr; external name '_GetCollectionItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetCollectionItem( c: Collection; tag: CollectionTag; id: SInt32; var itemSize: SInt32; itemData: UnivPtr ): OSErr; external name '_GetCollectionItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RemoveCollectionItem()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function RemoveCollectionItem( c: Collection; tag: CollectionTag; id: SInt32 ): OSErr; external name '_RemoveCollectionItem';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function RemoveCollectionItem(c: Collection; tag: CollectionTag; id: SInt32): OSErr; external name '_RemoveCollectionItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RemoveCollectionItem( c: Collection; tag: CollectionTag; id: SInt32 ): OSErr; external name '_RemoveCollectionItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  SetCollectionItemInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function SetCollectionItemInfo( c: Collection; tag: CollectionTag; id: SInt32; whichAttributes: SInt32; newAttributes: SInt32 ): OSErr; external name '_SetCollectionItemInfo';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function SetCollectionItemInfo(c: Collection; tag: CollectionTag; id: SInt32; whichAttributes: SInt32; newAttributes: SInt32): OSErr; external name '_SetCollectionItemInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SetCollectionItemInfo( c: Collection; tag: CollectionTag; id: SInt32; whichAttributes: SInt32; newAttributes: SInt32 ): OSErr; external name '_SetCollectionItemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetCollectionItemInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetCollectionItemInfo( c: Collection; tag: CollectionTag; id: SInt32; var itemIndex: SInt32; var itemSize: SInt32; var attributes: SInt32 ): OSErr; external name '_GetCollectionItemInfo';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetCollectionItemInfo(c: Collection; tag: CollectionTag; id: SInt32; var index: SInt32; var itemSize: SInt32; var attributes: SInt32): OSErr; external name '_GetCollectionItemInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetCollectionItemInfo( c: Collection; tag: CollectionTag; id: SInt32; var itemIndex: SInt32; var itemSize: SInt32; var attributes: SInt32 ): OSErr; external name '_GetCollectionItemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ReplaceIndexedCollectionItem()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function ReplaceIndexedCollectionItem( c: Collection; itemIndex: SInt32; itemSize: SInt32; itemData: {const} UnivPtr ): OSErr; external name '_ReplaceIndexedCollectionItem';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function ReplaceIndexedCollectionItem(c: Collection; index: SInt32; itemSize: SInt32; itemData: UnivPtr): OSErr; external name '_ReplaceIndexedCollectionItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ReplaceIndexedCollectionItem( c: Collection; itemIndex: SInt32; itemSize: SInt32; itemData: {const} UnivPtr ): OSErr; external name '_ReplaceIndexedCollectionItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetIndexedCollectionItem()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetIndexedCollectionItem( c: Collection; itemIndex: SInt32; var itemSize: SInt32; itemData: UnivPtr ): OSErr; external name '_GetIndexedCollectionItem';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetIndexedCollectionItem(c: Collection; index: SInt32; var itemSize: SInt32; itemData: UnivPtr): OSErr; external name '_GetIndexedCollectionItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetIndexedCollectionItem( c: Collection; itemIndex: SInt32; var itemSize: SInt32; itemData: UnivPtr ): OSErr; external name '_GetIndexedCollectionItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RemoveIndexedCollectionItem()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function RemoveIndexedCollectionItem( c: Collection; itemIndex: SInt32 ): OSErr; external name '_RemoveIndexedCollectionItem';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function RemoveIndexedCollectionItem(c: Collection; index: SInt32): OSErr; external name '_RemoveIndexedCollectionItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RemoveIndexedCollectionItem( c: Collection; itemIndex: SInt32 ): OSErr; external name '_RemoveIndexedCollectionItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  SetIndexedCollectionItemInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function SetIndexedCollectionItemInfo( c: Collection; itemIndex: SInt32; whichAttributes: SInt32; newAttributes: SInt32 ): OSErr; external name '_SetIndexedCollectionItemInfo';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function SetIndexedCollectionItemInfo(c: Collection; index: SInt32; whichAttributes: SInt32; newAttributes: SInt32): OSErr; external name '_SetIndexedCollectionItemInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SetIndexedCollectionItemInfo( c: Collection; itemIndex: SInt32; whichAttributes: SInt32; newAttributes: SInt32 ): OSErr; external name '_SetIndexedCollectionItemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetIndexedCollectionItemInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetIndexedCollectionItemInfo( c: Collection; itemIndex: SInt32; var tag: CollectionTag; var id: SInt32; var itemSize: SInt32; var attributes: SInt32 ): OSErr; external name '_GetIndexedCollectionItemInfo';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetIndexedCollectionItemInfo(c: Collection; index: SInt32; var tag: CollectionTag; var id: SInt32; var itemSize: SInt32; var attributes: SInt32): OSErr; external name '_GetIndexedCollectionItemInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetIndexedCollectionItemInfo( c: Collection; itemIndex: SInt32; var tag: CollectionTag; var id: SInt32; var itemSize: SInt32; var attributes: SInt32 ): OSErr; external name '_GetIndexedCollectionItemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CollectionTagExists()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function CollectionTagExists( c: Collection; tag: CollectionTag ): Boolean; external name '_CollectionTagExists';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function CollectionTagExists(c: Collection; tag: CollectionTag): boolean; external name '_CollectionTagExists';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CollectionTagExists( c: Collection; tag: CollectionTag ): Boolean; external name '_CollectionTagExists';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CountCollectionTags()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function CountCollectionTags( c: Collection ): SInt32; external name '_CountCollectionTags';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function CountCollectionTags(c: Collection): SInt32; external name '_CountCollectionTags';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CountCollectionTags( c: Collection ): SInt32; external name '_CountCollectionTags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetIndexedCollectionTag()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetIndexedCollectionTag( c: Collection; tagIndex: SInt32; var tag: CollectionTag ): OSErr; external name '_GetIndexedCollectionTag';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetIndexedCollectionTag(c: Collection; tagIndex: SInt32; var tag: CollectionTag): OSErr; external name '_GetIndexedCollectionTag';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetIndexedCollectionTag( c: Collection; tagIndex: SInt32; var tag: CollectionTag ): OSErr; external name '_GetIndexedCollectionTag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CountTaggedCollectionItems()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function CountTaggedCollectionItems( c: Collection; tag: CollectionTag ): SInt32; external name '_CountTaggedCollectionItems';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function CountTaggedCollectionItems(c: Collection; tag: CollectionTag): SInt32; external name '_CountTaggedCollectionItems';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CountTaggedCollectionItems( c: Collection; tag: CollectionTag ): SInt32; external name '_CountTaggedCollectionItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetTaggedCollectionItem()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetTaggedCollectionItem( c: Collection; tag: CollectionTag; whichItem: SInt32; var itemSize: SInt32; itemData: UnivPtr ): OSErr; external name '_GetTaggedCollectionItem';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetTaggedCollectionItem(c: Collection; tag: CollectionTag; whichItem: SInt32; var itemSize: SInt32; itemData: UnivPtr): OSErr; external name '_GetTaggedCollectionItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetTaggedCollectionItem( c: Collection; tag: CollectionTag; whichItem: SInt32; var itemSize: SInt32; itemData: UnivPtr ): OSErr; external name '_GetTaggedCollectionItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetTaggedCollectionItemInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetTaggedCollectionItemInfo( c: Collection; tag: CollectionTag; whichItem: SInt32; var id: SInt32; var itemIndex: SInt32; var itemSize: SInt32; var attributes: SInt32 ): OSErr; external name '_GetTaggedCollectionItemInfo';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetTaggedCollectionItemInfo(c: Collection; tag: CollectionTag; whichItem: SInt32; var id: SInt32; var index: SInt32; var itemSize: SInt32; var attributes: SInt32): OSErr; external name '_GetTaggedCollectionItemInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetTaggedCollectionItemInfo( c: Collection; tag: CollectionTag; whichItem: SInt32; var id: SInt32; var itemIndex: SInt32; var itemSize: SInt32; var attributes: SInt32 ): OSErr; external name '_GetTaggedCollectionItemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  PurgeCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
procedure PurgeCollection( c: Collection; whichAttributes: SInt32; matchingAttributes: SInt32 ); external name '_PurgeCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
procedure PurgeCollection(c: Collection; whichAttributes: SInt32; matchingAttributes: SInt32); external name '_PurgeCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PurgeCollection( c: Collection; whichAttributes: SInt32; matchingAttributes: SInt32 ); external name '_PurgeCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  PurgeCollectionTag()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
procedure PurgeCollectionTag( c: Collection; tag: CollectionTag ); external name '_PurgeCollectionTag';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
procedure PurgeCollectionTag(c: Collection; tag: CollectionTag); external name '_PurgeCollectionTag';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure PurgeCollectionTag( c: Collection; tag: CollectionTag ); external name '_PurgeCollectionTag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  EmptyCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
procedure EmptyCollection( c: Collection ); external name '_EmptyCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
procedure EmptyCollection(c: Collection); external name '_EmptyCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure EmptyCollection( c: Collection ); external name '_EmptyCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  FlattenCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function FlattenCollection( c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr ): OSErr; external name '_FlattenCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function FlattenCollection(c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr): OSErr; external name '_FlattenCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function FlattenCollection( c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr ): OSErr; external name '_FlattenCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  FlattenPartialCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function FlattenPartialCollection( c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr; whichAttributes: SInt32; matchingAttributes: SInt32 ): OSErr; external name '_FlattenPartialCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function FlattenPartialCollection(c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr; whichAttributes: SInt32; matchingAttributes: SInt32): OSErr; external name '_FlattenPartialCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function FlattenPartialCollection( c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr; whichAttributes: SInt32; matchingAttributes: SInt32 ): OSErr; external name '_FlattenPartialCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  UnflattenCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function UnflattenCollection( c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr ): OSErr; external name '_UnflattenCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function UnflattenCollection(c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr): OSErr; external name '_UnflattenCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UnflattenCollection( c: Collection; flattenProc: CollectionFlattenUPP; refCon: UnivPtr ): OSErr; external name '_UnflattenCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetCollectionExceptionProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetCollectionExceptionProc( c: Collection ): CollectionExceptionUPP; external name '_GetCollectionExceptionProc';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetCollectionExceptionProc(c: Collection): CollectionExceptionUPP; external name '_GetCollectionExceptionProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetCollectionExceptionProc( c: Collection ): CollectionExceptionUPP; external name '_GetCollectionExceptionProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  SetCollectionExceptionProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
procedure SetCollectionExceptionProc( c: Collection; exceptionProc: CollectionExceptionUPP ); external name '_SetCollectionExceptionProc';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
procedure SetCollectionExceptionProc(c: Collection; exceptionProc: CollectionExceptionUPP); external name '_SetCollectionExceptionProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure SetCollectionExceptionProc( c: Collection; exceptionProc: CollectionExceptionUPP ); external name '_SetCollectionExceptionProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetNewCollection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetNewCollection( collectionID: SInt16 ): Collection; external name '_GetNewCollection';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetNewCollection(collectionID: SInt16): Collection; external name '_GetNewCollection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetNewCollection( collectionID: SInt16 ): Collection; external name '_GetNewCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{********************************************************************}
{************* Utility routines for handle-based access *************}
{********************************************************************}
{
 *  AddCollectionItemHdl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function AddCollectionItemHdl( aCollection: Collection; tag: CollectionTag; id: SInt32; itemData: Handle ): OSErr; external name '_AddCollectionItemHdl';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function AddCollectionItemHdl(aCollection: Collection; tag: CollectionTag; id: SInt32; itemData: Handle): OSErr; external name '_AddCollectionItemHdl';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AddCollectionItemHdl( aCollection: Collection; tag: CollectionTag; id: SInt32; itemData: Handle ): OSErr; external name '_AddCollectionItemHdl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetCollectionItemHdl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetCollectionItemHdl( aCollection: Collection; tag: CollectionTag; id: SInt32; itemData: Handle ): OSErr; external name '_GetCollectionItemHdl';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetCollectionItemHdl(aCollection: Collection; tag: CollectionTag; id: SInt32; itemData: Handle): OSErr; external name '_GetCollectionItemHdl';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetCollectionItemHdl( aCollection: Collection; tag: CollectionTag; id: SInt32; itemData: Handle ): OSErr; external name '_GetCollectionItemHdl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ReplaceIndexedCollectionItemHdl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function ReplaceIndexedCollectionItemHdl( aCollection: Collection; itemIndex: SInt32; itemData: Handle ): OSErr; external name '_ReplaceIndexedCollectionItemHdl';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function ReplaceIndexedCollectionItemHdl(aCollection: Collection; index: SInt32; itemData: Handle): OSErr; external name '_ReplaceIndexedCollectionItemHdl';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ReplaceIndexedCollectionItemHdl( aCollection: Collection; itemIndex: SInt32; itemData: Handle ): OSErr; external name '_ReplaceIndexedCollectionItemHdl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetIndexedCollectionItemHdl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function GetIndexedCollectionItemHdl( aCollection: Collection; itemIndex: SInt32; itemData: Handle ): OSErr; external name '_GetIndexedCollectionItemHdl';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function GetIndexedCollectionItemHdl(aCollection: Collection; index: SInt32; itemData: Handle): OSErr; external name '_GetIndexedCollectionItemHdl';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetIndexedCollectionItemHdl( aCollection: Collection; itemIndex: SInt32; itemData: Handle ): OSErr; external name '_GetIndexedCollectionItemHdl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  FlattenCollectionToHdl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function FlattenCollectionToHdl( aCollection: Collection; flattened: Handle ): OSErr; external name '_FlattenCollectionToHdl';
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


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
<<<<<<< HEAD
function FlattenCollectionToHdl(aCollection: Collection; flattened: Handle): OSErr; external name '_FlattenCollectionToHdl';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function FlattenCollectionToHdl( aCollection: Collection; flattened: Handle ): OSErr; external name '_FlattenCollectionToHdl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  UnflattenCollectionFromHdl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function UnflattenCollectionFromHdl( aCollection: Collection; flattened: Handle ): OSErr; external name '_UnflattenCollectionFromHdl';
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


{$ifc OLDROUTINENAMES}
const
	dontWantTag = kCollectionDontWantTag;
	dontWantId = kCollectionDontWantId;
	dontWantSize = kCollectionDontWantSize;
	dontWantAttributes = kCollectionDontWantAttributes;
	dontWantIndex = kCollectionDontWantIndex;
	dontWantData = kCollectionDontWantData;

const
	noCollectionAttributes = kCollectionNoAttributes;
	allCollectionAttributes = kCollectionAllAttributes;
	userCollectionAttributes = kCollectionUserAttributes;
	defaultCollectionAttributes = kCollectionDefaultAttributes;

const
	collectionUser0Bit = kCollectionUser0Bit;
	collectionUser1Bit = kCollectionUser1Bit;
	collectionUser2Bit = kCollectionUser2Bit;
	collectionUser3Bit = kCollectionUser3Bit;
	collectionUser4Bit = kCollectionUser4Bit;
	collectionUser5Bit = kCollectionUser5Bit;
	collectionUser6Bit = kCollectionUser6Bit;
	collectionUser7Bit = kCollectionUser7Bit;
	collectionUser8Bit = kCollectionUser8Bit;
	collectionUser9Bit = kCollectionUser9Bit;
	collectionUser10Bit = kCollectionUser10Bit;
	collectionUser11Bit = kCollectionUser11Bit;
	collectionUser12Bit = kCollectionUser12Bit;
	collectionUser13Bit = kCollectionUser13Bit;
	collectionUser14Bit = kCollectionUser14Bit;
	collectionUser15Bit = kCollectionUser15Bit;
	collectionReserved0Bit = kCollectionReserved0Bit;
	collectionReserved1Bit = kCollectionReserved1Bit;
	collectionReserved2Bit = kCollectionReserved2Bit;
	collectionReserved3Bit = kCollectionReserved3Bit;
	collectionReserved4Bit = kCollectionReserved4Bit;
	collectionReserved5Bit = kCollectionReserved5Bit;
	collectionReserved6Bit = kCollectionReserved6Bit;
	collectionReserved7Bit = kCollectionReserved7Bit;
	collectionReserved8Bit = kCollectionReserved8Bit;
	collectionReserved9Bit = kCollectionReserved9Bit;
	collectionReserved10Bit = kCollectionReserved10Bit;
	collectionReserved11Bit = kCollectionReserved11Bit;
	collectionReserved12Bit = kCollectionReserved12Bit;
	collectionReserved13Bit = kCollectionReserved13Bit;
	collectionPersistenceBit = kCollectionPersistenceBit;
	collectionLockBit = kCollectionLockBit;

const
	collectionUser0Mask = kCollectionUser0Mask;
	collectionUser1Mask = kCollectionUser1Mask;
	collectionUser2Mask = kCollectionUser2Mask;
	collectionUser3Mask = kCollectionUser3Mask;
	collectionUser4Mask = kCollectionUser4Mask;
	collectionUser5Mask = kCollectionUser5Mask;
	collectionUser6Mask = kCollectionUser6Mask;
	collectionUser7Mask = kCollectionUser7Mask;
	collectionUser8Mask = kCollectionUser8Mask;
	collectionUser9Mask = kCollectionUser9Mask;
	collectionUser10Mask = kCollectionUser10Mask;
	collectionUser11Mask = kCollectionUser11Mask;
	collectionUser12Mask = kCollectionUser12Mask;
	collectionUser13Mask = kCollectionUser13Mask;
	collectionUser14Mask = kCollectionUser14Mask;
	collectionUser15Mask = kCollectionUser15Mask;
	collectionReserved0Mask = kCollectionReserved0Mask;
	collectionReserved1Mask = kCollectionReserved1Mask;
	collectionReserved2Mask = kCollectionReserved2Mask;
	collectionReserved3Mask = kCollectionReserved3Mask;
	collectionReserved4Mask = kCollectionReserved4Mask;
	collectionReserved5Mask = kCollectionReserved5Mask;
	collectionReserved6Mask = kCollectionReserved6Mask;
	collectionReserved7Mask = kCollectionReserved7Mask;
	collectionReserved8Mask = kCollectionReserved8Mask;
	collectionReserved9Mask = kCollectionReserved9Mask;
	collectionReserved10Mask = kCollectionReserved10Mask;
	collectionReserved11Mask = kCollectionReserved11Mask;
	collectionReserved12Mask = kCollectionReserved12Mask;
	collectionReserved13Mask = kCollectionReserved13Mask;
	collectionPersistenceMask = kCollectionPersistenceMask;
	collectionLockMask = kCollectionLockMask;

{$endc}  {OLDROUTINENAMES}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CollectionsLib 1.0 and later
 }
function UnflattenCollectionFromHdl( aCollection: Collection; flattened: Handle ): OSErr; external name '_UnflattenCollectionFromHdl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc OLDROUTINENAMES}
const
	dontWantTag = kCollectionDontWantTag;
	dontWantId = kCollectionDontWantId;
	dontWantSize = kCollectionDontWantSize;
	dontWantAttributes = kCollectionDontWantAttributes;
	dontWantIndex = kCollectionDontWantIndex;
	dontWantData = kCollectionDontWantData;

const
	noCollectionAttributes = kCollectionNoAttributes;
	allCollectionAttributes = kCollectionAllAttributes;
	userCollectionAttributes = kCollectionUserAttributes;
	defaultCollectionAttributes = kCollectionDefaultAttributes;

const
	collectionUser0Bit = kCollectionUser0Bit;
	collectionUser1Bit = kCollectionUser1Bit;
	collectionUser2Bit = kCollectionUser2Bit;
	collectionUser3Bit = kCollectionUser3Bit;
	collectionUser4Bit = kCollectionUser4Bit;
	collectionUser5Bit = kCollectionUser5Bit;
	collectionUser6Bit = kCollectionUser6Bit;
	collectionUser7Bit = kCollectionUser7Bit;
	collectionUser8Bit = kCollectionUser8Bit;
	collectionUser9Bit = kCollectionUser9Bit;
	collectionUser10Bit = kCollectionUser10Bit;
	collectionUser11Bit = kCollectionUser11Bit;
	collectionUser12Bit = kCollectionUser12Bit;
	collectionUser13Bit = kCollectionUser13Bit;
	collectionUser14Bit = kCollectionUser14Bit;
	collectionUser15Bit = kCollectionUser15Bit;
	collectionReserved0Bit = kCollectionReserved0Bit;
	collectionReserved1Bit = kCollectionReserved1Bit;
	collectionReserved2Bit = kCollectionReserved2Bit;
	collectionReserved3Bit = kCollectionReserved3Bit;
	collectionReserved4Bit = kCollectionReserved4Bit;
	collectionReserved5Bit = kCollectionReserved5Bit;
	collectionReserved6Bit = kCollectionReserved6Bit;
	collectionReserved7Bit = kCollectionReserved7Bit;
	collectionReserved8Bit = kCollectionReserved8Bit;
	collectionReserved9Bit = kCollectionReserved9Bit;
	collectionReserved10Bit = kCollectionReserved10Bit;
	collectionReserved11Bit = kCollectionReserved11Bit;
	collectionReserved12Bit = kCollectionReserved12Bit;
	collectionReserved13Bit = kCollectionReserved13Bit;
	collectionPersistenceBit = kCollectionPersistenceBit;
	collectionLockBit = kCollectionLockBit;

const
	collectionUser0Mask = kCollectionUser0Mask;
	collectionUser1Mask = kCollectionUser1Mask;
	collectionUser2Mask = kCollectionUser2Mask;
	collectionUser3Mask = kCollectionUser3Mask;
	collectionUser4Mask = kCollectionUser4Mask;
	collectionUser5Mask = kCollectionUser5Mask;
	collectionUser6Mask = kCollectionUser6Mask;
	collectionUser7Mask = kCollectionUser7Mask;
	collectionUser8Mask = kCollectionUser8Mask;
	collectionUser9Mask = kCollectionUser9Mask;
	collectionUser10Mask = kCollectionUser10Mask;
	collectionUser11Mask = kCollectionUser11Mask;
	collectionUser12Mask = kCollectionUser12Mask;
	collectionUser13Mask = kCollectionUser13Mask;
	collectionUser14Mask = kCollectionUser14Mask;
	collectionUser15Mask = kCollectionUser15Mask;
	collectionReserved0Mask = kCollectionReserved0Mask;
	collectionReserved1Mask = kCollectionReserved1Mask;
	collectionReserved2Mask = kCollectionReserved2Mask;
	collectionReserved3Mask = kCollectionReserved3Mask;
	collectionReserved4Mask = kCollectionReserved4Mask;
	collectionReserved5Mask = kCollectionReserved5Mask;
	collectionReserved6Mask = kCollectionReserved6Mask;
	collectionReserved7Mask = kCollectionReserved7Mask;
	collectionReserved8Mask = kCollectionReserved8Mask;
	collectionReserved9Mask = kCollectionReserved9Mask;
	collectionReserved10Mask = kCollectionReserved10Mask;
	collectionReserved11Mask = kCollectionReserved11Mask;
	collectionReserved12Mask = kCollectionReserved12Mask;
	collectionReserved13Mask = kCollectionReserved13Mask;
	collectionPersistenceMask = kCollectionPersistenceMask;
	collectionLockMask = kCollectionLockMask;

{$endc}  {OLDROUTINENAMES}

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
