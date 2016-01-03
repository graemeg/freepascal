{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/IntlResources.h
 
     Contains:   International Resource definitions.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1983-2011 by Apple Inc. All rights reserved.
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1983-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1983-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1983-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1983-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       IntlResources.p
=======
     File:       CarbonCore/IntlResources.h
>>>>>>> origin/fixes_2.4
 
     Contains:   International Resource definitions.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1983-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1983-2008 by Apple Computer, Inc., all rights reserved.
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

unit IntlResources;
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
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
uses MacTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
<<<<<<< HEAD

{$ALIGN MAC68K}

const
{ Bits in the itlcFlags byte }
	itlcShowIcon = 7;    {Show icon even if only one script}
	itlcDualCaret = 6;    {Use dual caret for mixed direction text}
                                        { Bits in the itlcSysFlags word }
	itlcSysDirection = 15;    {System direction - left to right/right to left}

const
{ One more flag in the itlcFlags byte }
	itlcDisableKeyScriptSync = 3;     {Disable font and keyboard script synchrinozation}

const
{ We should define masks, too. }
	itlcDisableKeyScriptSyncMask = 1 shl itlcDisableKeyScriptSync; {Disable font and keyboard script synchrinozation mask}


const
	tokLeftQuote = 1;    { NumberParts.data[] enumerators                             }
	tokRightQuote = 2;    {  In general, these are NOT to be considered indices into the data[] array }
	tokLeadPlacer = 3;
	tokLeader = 4;
	tokNonLeader = 5;
	tokZeroLead = 6;
	tokPercent = 7;
	tokPlusSign = 8;
	tokMinusSign = 9;
	tokThousands = 10;
	tokReserved = 11;   { 11 is reserved field }
	tokSeparator = 12;
	tokEscape = 13;
	tokDecPoint = 14;
	tokEPlus = 15;
	tokEMinus = 16;
	tokMaxSymbols = 31;
	curNumberPartsVersion = 1;     {current version of NumberParts record}

const
	currSymLead = 16;
	currNegSym = 32;
	currTrailingZ = 64;
	currLeadingZ = 128;

const
	mdy = 0;
	dmy = 1;
	ymd = 2;
	myd = 3;
	dym = 4;
	ydm = 5;

=======

{$ALIGN MAC68K}

const
{ Bits in the itlcFlags byte }
	itlcShowIcon = 7;    {Show icon even if only one script}
	itlcDualCaret = 6;    {Use dual caret for mixed direction text}
                                        { Bits in the itlcSysFlags word }
	itlcSysDirection = 15;    {System direction - left to right/right to left}

const
{ One more flag in the itlcFlags byte }
	itlcDisableKeyScriptSync = 3;     {Disable font and keyboard script synchrinozation}

const
{ We should define masks, too. }
	itlcDisableKeyScriptSyncMask = 1 shl itlcDisableKeyScriptSync; {Disable font and keyboard script synchrinozation mask}


const
	tokLeftQuote = 1;    { NumberParts.data[] enumerators                             }
	tokRightQuote = 2;    {  In general, these are NOT to be considered indices into the data[] array }
	tokLeadPlacer = 3;
	tokLeader = 4;
	tokNonLeader = 5;
	tokZeroLead = 6;
	tokPercent = 7;
	tokPlusSign = 8;
	tokMinusSign = 9;
	tokThousands = 10;
	tokReserved = 11;   { 11 is reserved field }
	tokSeparator = 12;
	tokEscape = 13;
	tokDecPoint = 14;
	tokEPlus = 15;
	tokEMinus = 16;
	tokMaxSymbols = 31;
	curNumberPartsVersion = 1;     {current version of NumberParts record}

const
	currSymLead = 16;
	currNegSym = 32;
	currTrailingZ = 64;
	currLeadingZ = 128;

const
	mdy = 0;
	dmy = 1;
	ymd = 2;
	myd = 3;
	dym = 4;
	ydm = 5;

type
	DateOrders = SInt8;
const
	timeCycle24 = 0;    {time sequence 0:00 - 23:59}
	timeCycleZero = 1;    {time sequence 0:00-11:59, 0:00 - 11:59}
	timeCycle12 = 255;  {time sequence 12:00 - 11:59, 12:00 - 11:59}
	zeroCycle = 1;    {old name for timeCycleZero}
	longDay = 0;    {day of the month}
	longWeek = 1;    {day of the week}
	longMonth = 2;    {month of the year}
	longYear = 3;    {year}
	supDay = 1;    {suppress day of month}
	supWeek = 2;    {suppress day of week}
	supMonth = 4;    {suppress month}
	supYear = 8;    {suppress year}
	dayLdingZ = 32;
	mntLdingZ = 64;
	century = 128;
	secLeadingZ = 32;
	minLeadingZ = 64;
	hrLeadingZ = 128;

{ moved OffsetTable back here from QuickdrawText }
>>>>>>> graemeg/cpstrnew
type
	DateOrders = SInt8;
const
=======

{$ALIGN MAC68K}

const
{ Bits in the itlcFlags byte }
	itlcShowIcon = 7;    {Show icon even if only one script}
	itlcDualCaret = 6;    {Use dual caret for mixed direction text}
                                        { Bits in the itlcSysFlags word }
	itlcSysDirection = 15;    {System direction - left to right/right to left}

const
{ One more flag in the itlcFlags byte }
	itlcDisableKeyScriptSync = 3;     {Disable font and keyboard script synchrinozation}

const
{ We should define masks, too. }
	itlcDisableKeyScriptSyncMask = 1 shl itlcDisableKeyScriptSync; {Disable font and keyboard script synchrinozation mask}


const
	tokLeftQuote = 1;    { NumberParts.data[] enumerators                             }
	tokRightQuote = 2;    {  In general, these are NOT to be considered indices into the data[] array }
	tokLeadPlacer = 3;
	tokLeader = 4;
	tokNonLeader = 5;
	tokZeroLead = 6;
	tokPercent = 7;
	tokPlusSign = 8;
	tokMinusSign = 9;
	tokThousands = 10;
	tokReserved = 11;   { 11 is reserved field }
	tokSeparator = 12;
	tokEscape = 13;
	tokDecPoint = 14;
	tokEPlus = 15;
	tokEMinus = 16;
	tokMaxSymbols = 31;
	curNumberPartsVersion = 1;     {current version of NumberParts record}

const
	currSymLead = 16;
	currNegSym = 32;
	currTrailingZ = 64;
	currLeadingZ = 128;

const
	mdy = 0;
	dmy = 1;
	ymd = 2;
	myd = 3;
	dym = 4;
	ydm = 5;

type
	DateOrders = SInt8;
const
>>>>>>> origin/cpstrnew
	timeCycle24 = 0;    {time sequence 0:00 - 23:59}
	timeCycleZero = 1;    {time sequence 0:00-11:59, 0:00 - 11:59}
	timeCycle12 = 255;  {time sequence 12:00 - 11:59, 12:00 - 11:59}
	zeroCycle = 1;    {old name for timeCycleZero}
	longDay = 0;    {day of the month}
	longWeek = 1;    {day of the week}
	longMonth = 2;    {month of the year}
	longYear = 3;    {year}
	supDay = 1;    {suppress day of month}
	supWeek = 2;    {suppress day of week}
	supMonth = 4;    {suppress month}
	supYear = 8;    {suppress year}
	dayLdingZ = 32;
	mntLdingZ = 64;
	century = 128;
	secLeadingZ = 32;
	minLeadingZ = 64;
	hrLeadingZ = 128;

{ moved OffsetTable back here from QuickdrawText }
type
	OffPairPtr = ^OffPair;
	OffPair = record
		offFirst: SInt16;
		offSecond: SInt16;
	end;
type
	OffsetTable = array [0..2] of OffPair;
type
	Intl0RecPtr = ^Intl0Rec;
	Intl0Rec =  record
		decimalPt: char;              {decimal point character}
		thousSep: char;               {thousands separator character}
		listSep: char;                {list separator character}
		currSym1: char;               {currency symbol}
		currSym2: char;
		currSym3: char;
		currFmt: UInt8;                {currency format flags}
		dateOrder: UInt8;              {order of short date elements: mdy, dmy, etc.}
		shrtDateFmt: UInt8;            {format flags for each short date element}
		dateSep: char;                {date separator character}
		timeCycle: UInt8;              {specifies time cycle: 0..23, 1..12, or 0..11}
		timeFmt: UInt8;                {format flags for each time element}
		mornStr: array [1..4] of char;             {trailing string for AM if 12-hour cycle}
		eveStr: array [1..4] of char;              {trailing string for PM if 12-hour cycle}
		timeSep: char;                {time separator character}
		time1Suff: char;              {trailing string for AM if 24-hour cycle}
		time2Suff: char;
		time3Suff: char;
		time4Suff: char;
		time5Suff: char;              {trailing string for PM if 24-hour cycle}
		time6Suff: char;
		time7Suff: char;
		time8Suff: char;
		metricSys: UInt8;              {255 if metric, 0 if inches etc.}
		intl0Vers: SInt16;              {region code (hi byte) and version (lo byte)}
	end;
type
	Intl0Ptr = ^Intl0Rec;
	Intl0Hndl = ^Intl0Ptr;
	Intl1RecPtr = ^Intl1Rec;
	Intl1Rec =  record
		days: array [1..7] of Str15;                {day names}
		months: array [1..12] of Str15;             {month names}
		suppressDay: UInt8;            {255 for no day, or flags to suppress any element}
		lngDateFmt: UInt8;             {order of long date elements}
		dayLeading0: UInt8;            {255 for leading 0 in day number}
		abbrLen: UInt8;                {length for abbreviating names}
		st0: array [1..4] of char;                 { separator strings for long date format }
		st1: array [1..4] of char;
		st2: array [1..4] of char;
		st3: array [1..4] of char;
		st4: array [1..4] of char;
		intl1Vers: SInt16;              { region code (hi byte) and version (lo byte) }
		localRtn: array [0..0] of SInt16;            { now a flag for opt extension }
	end;
type
	Intl1Ptr = ^Intl1Rec;
	Intl1Hndl = ^Intl1Ptr;
{fields for optional itl1 extension}
type
	Itl1ExtRecPtr = ^Itl1ExtRec;
	Itl1ExtRec = record
		base: Intl1Rec;                   {un-extended Intl1Rec}
		version: SInt16;
		format: SInt16;
		calendarCode: SInt16;           {calendar code for this itl1 resource}
		extraDaysTableOffset: SInt32;   {offset in itl1 to extra days table}
		extraDaysTableLength: SInt32;   {length of extra days table}
		extraMonthsTableOffset: SInt32; {offset in itl1 to extra months table}
		extraMonthsTableLength: SInt32; {length of extra months table}
		abbrevDaysTableOffset: SInt32;  {offset in itl1 to abbrev days table}
		abbrevDaysTableLength: SInt32;  {length of abbrev days table}
		abbrevMonthsTableOffset: SInt32; {offset in itl1 to abbrev months table}
		abbrevMonthsTableLength: SInt32; {length of abbrev months table}
		extraSepsTableOffset: SInt32;   {offset in itl1 to extra seps table}
		extraSepsTableLength: SInt32;   {length of extra seps table}
		tables: array [0..0] of SInt16;              {now a flag for opt extension}
	end;
type
	UntokenTable = record
		len: SInt16;
		lastToken: SInt16;
		index: array [0..255] of SInt16;             {index table; last = lastToken}
	end;
	UntokenTablePtr = ^UntokenTable;
type
	UntokenTableHandle = ^UntokenTablePtr;
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

const
{ Bits in the itlcFlags byte }
	itlcShowIcon = 7;    {Show icon even if only one script}
	itlcDualCaret = 6;    {Use dual caret for mixed direction text}
                                        { Bits in the itlcSysFlags word }
	itlcSysDirection = 15;    {System direction - left to right/right to left}

const
{ One more flag in the itlcFlags byte }
	itlcDisableKeyScriptSync = 3;     {Disable font and keyboard script synchrinozation}

const
{ We should define masks, too. }
	itlcDisableKeyScriptSyncMask = 1 shl itlcDisableKeyScriptSync; {Disable font and keyboard script synchrinozation mask}


const
	tokLeftQuote = 1;    { NumberParts.data[] enumerators                             }
	tokRightQuote = 2;    {  In general, these are NOT to be considered indices into the data[] array }
	tokLeadPlacer = 3;
	tokLeader = 4;
	tokNonLeader = 5;
	tokZeroLead = 6;
	tokPercent = 7;
	tokPlusSign = 8;
	tokMinusSign = 9;
	tokThousands = 10;
	tokReserved = 11;   { 11 is reserved field }
	tokSeparator = 12;
	tokEscape = 13;
	tokDecPoint = 14;
	tokEPlus = 15;
	tokEMinus = 16;
	tokMaxSymbols = 31;
	curNumberPartsVersion = 1;     {current version of NumberParts record}

const
	currSymLead = 16;
	currNegSym = 32;
	currTrailingZ = 64;
	currLeadingZ = 128;

const
	mdy = 0;
	dmy = 1;
	ymd = 2;
	myd = 3;
	dym = 4;
	ydm = 5;

type
	DateOrders = SInt8;
const
	timeCycle24 = 0;    {time sequence 0:00 - 23:59}
	timeCycleZero = 1;    {time sequence 0:00-11:59, 0:00 - 11:59}
	timeCycle12 = 255;  {time sequence 12:00 - 11:59, 12:00 - 11:59}
	zeroCycle = 1;    {old name for timeCycleZero}
	longDay = 0;    {day of the month}
	longWeek = 1;    {day of the week}
	longMonth = 2;    {month of the year}
	longYear = 3;    {year}
	supDay = 1;    {suppress day of month}
	supWeek = 2;    {suppress day of week}
	supMonth = 4;    {suppress month}
	supYear = 8;    {suppress year}
	dayLdingZ = 32;
	mntLdingZ = 64;
	century = 128;
	secLeadingZ = 32;
	minLeadingZ = 64;
	hrLeadingZ = 128;

{ moved OffsetTable back here from QuickdrawText }
type
	OffPairPtr = ^OffPair;
	OffPair = record
		offFirst: SInt16;
		offSecond: SInt16;
	end;
type
	OffsetTable = array [0..2] of OffPair;
type
	Intl0RecPtr = ^Intl0Rec;
	Intl0Rec =  record
		decimalPt: char;              {decimal point character}
		thousSep: char;               {thousands separator character}
		listSep: char;                {list separator character}
		currSym1: char;               {currency symbol}
		currSym2: char;
		currSym3: char;
		currFmt: UInt8;                {currency format flags}
		dateOrder: UInt8;              {order of short date elements: mdy, dmy, etc.}
		shrtDateFmt: UInt8;            {format flags for each short date element}
		dateSep: char;                {date separator character}
		timeCycle: UInt8;              {specifies time cycle: 0..23, 1..12, or 0..11}
		timeFmt: UInt8;                {format flags for each time element}
		mornStr: array [1..4] of char;             {trailing string for AM if 12-hour cycle}
		eveStr: array [1..4] of char;              {trailing string for PM if 12-hour cycle}
		timeSep: char;                {time separator character}
		time1Suff: char;              {trailing string for AM if 24-hour cycle}
		time2Suff: char;
		time3Suff: char;
		time4Suff: char;
		time5Suff: char;              {trailing string for PM if 24-hour cycle}
		time6Suff: char;
		time7Suff: char;
		time8Suff: char;
		metricSys: UInt8;              {255 if metric, 0 if inches etc.}
		intl0Vers: SInt16;              {region code (hi byte) and version (lo byte)}
	end;
type
	Intl0Ptr = ^Intl0Rec;
	Intl0Hndl = ^Intl0Ptr;
	Intl1RecPtr = ^Intl1Rec;
	Intl1Rec =  record
		days: array [1..7] of Str15;                {day names}
		months: array [1..12] of Str15;             {month names}
		suppressDay: UInt8;            {255 for no day, or flags to suppress any element}
		lngDateFmt: UInt8;             {order of long date elements}
		dayLeading0: UInt8;            {255 for leading 0 in day number}
		abbrLen: UInt8;                {length for abbreviating names}
		st0: array [1..4] of char;                 { separator strings for long date format }
		st1: array [1..4] of char;
		st2: array [1..4] of char;
		st3: array [1..4] of char;
		st4: array [1..4] of char;
		intl1Vers: SInt16;              { region code (hi byte) and version (lo byte) }
		localRtn: array [0..0] of SInt16;            { now a flag for opt extension }
	end;
type
	Intl1Ptr = ^Intl1Rec;
	Intl1Hndl = ^Intl1Ptr;
{fields for optional itl1 extension}
type
	Itl1ExtRecPtr = ^Itl1ExtRec;
	Itl1ExtRec = record
		base: Intl1Rec;                   {un-extended Intl1Rec}
		version: SInt16;
		format: SInt16;
		calendarCode: SInt16;           {calendar code for this itl1 resource}
		extraDaysTableOffset: SInt32;   {offset in itl1 to extra days table}
		extraDaysTableLength: SInt32;   {length of extra days table}
		extraMonthsTableOffset: SInt32; {offset in itl1 to extra months table}
		extraMonthsTableLength: SInt32; {length of extra months table}
		abbrevDaysTableOffset: SInt32;  {offset in itl1 to abbrev days table}
		abbrevDaysTableLength: SInt32;  {length of abbrev days table}
		abbrevMonthsTableOffset: SInt32; {offset in itl1 to abbrev months table}
		abbrevMonthsTableLength: SInt32; {length of abbrev months table}
		extraSepsTableOffset: SInt32;   {offset in itl1 to extra seps table}
		extraSepsTableLength: SInt32;   {length of extra seps table}
		tables: array [0..0] of SInt16;              {now a flag for opt extension}
	end;
type
	UntokenTable = record
		len: SInt16;
		lastToken: SInt16;
		index: array [0..255] of SInt16;             {index table; last = lastToken}
	end;
<<<<<<< HEAD

	UntokenTableHandle					= ^UntokenTablePtr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	UntokenTablePtr = ^UntokenTable;
type
	UntokenTableHandle = ^UntokenTablePtr;
>>>>>>> origin/fixes_2.4
	WideCharPtr = ^WideChar;
	WideChar = packed record
		case SInt16 of
		0: (
			a: packed array [0..1] of char;			{ 0 is the high order character [for PPC] }
			);
		2: (
{$ifc TARGET_RT_BIG_ENDIAN}
			hi: char;
			lo: char;
{$elsec}
			lo: char;
			hi: char;
{$endc}
			);
		1: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			b: SInt16;
			);
	end;
type
	WideCharArrPtr = ^WideCharArr;
	WideCharArr = record
		size: SInt16;
		data: array [0..9] of WideChar;
	end;
type
	NumberParts = record
		version: SInt16;
		data: array [0..30] of WideChar;               { index by [tokLeftQuote..tokMaxSymbols] }
		pePlus: WideCharArr;
		peMinus: WideCharArr;
		peMinusPlus: WideCharArr;
		altNumTable: WideCharArr;
		reserved: packed array [0..19] of char;
	end;
	NumberPartsPtr = ^NumberParts;


type
	Itl4RecPtr = ^Itl4Rec;
	Itl4Rec = record
		flags: SInt16;                  {reserved}
		resourceType: SInt32;           {contains 'itl4'}
		resourceNum: SInt16;            {resource ID}
		version: SInt16;                {version number}
		resHeader1: SInt32;             {reserved}
		resHeader2: SInt32;             {reserved}
		numTables: SInt16;              {number of tables, one-based}
		mapOffset: SInt32;              {offset to table that maps byte to token}
		strOffset: SInt32;              {offset to routine that copies canonical string}
		fetchOffset: SInt32;            {offset to routine that gets next byte of character}
		unTokenOffset: SInt32;          {offset to table that maps token to canonical string}
		defPartsOffset: SInt32;         {offset to default number parts table}
		resOffset6: SInt32;             {reserved}
		resOffset7: SInt32;             {reserved}
		resOffset8: SInt32;             {reserved}
	end;
type
	Itl4Ptr = Itl4RecPtr;
	Itl4Handle = ^Itl4Ptr;
{ New NItl4Rec for System 7.0: }
type
	NItl4RecPtr = ^NItl4Rec;
	NItl4Rec = record
		flags: SInt16;                  {reserved}
		resourceType: SInt32;           {contains 'itl4'}
		resourceNum: SInt16;            {resource ID}
		version: SInt16;                {version number}
		format: SInt16;                 {format code}
		resHeader: SInt16;              {reserved}
		resHeader2: SInt32;             {reserved}
		numTables: SInt16;              {number of tables, one-based}
		mapOffset: SInt32;              {offset to table that maps byte to token}
		strOffset: SInt32;              {offset to routine that copies canonical string}
		fetchOffset: SInt32;            {offset to routine that gets next byte of character}
		unTokenOffset: SInt32;          {offset to table that maps token to canonical string}
		defPartsOffset: SInt32;         {offset to default number parts table}
		whtSpListOffset: SInt32;        {offset to white space code list}
		resOffset7: SInt32;             {reserved}
		resOffset8: SInt32;             {reserved}
		resLength1: SInt16;             {reserved}
		resLength2: SInt16;             {reserved}
		resLength3: SInt16;             {reserved}
		unTokenLength: SInt16;          {length of untoken table}
		defPartsLength: SInt16;         {length of default number parts table}
		whtSpListLength: SInt16;        {length of white space code list}
		resLength7: SInt16;             {reserved}
		resLength8: SInt16;             {reserved}
	end;
type
	NItl4Ptr = NItl4RecPtr;
	NItl4Handle = ^NItl4Ptr;

type
	TableDirectoryRecordPtr = ^TableDirectoryRecord;
	TableDirectoryRecord = record
		tableSignature: OSType;         {4 byte long table name }
		reserved: UInt32;               {Reserved for internal use }
		tableStartOffset: UInt32;       {Table start offset in byte}
		tableSize: UInt32;              {Table size in byte}
	end;
type
	Itl5RecordPtr = ^Itl5Record;
	Itl5Record = record
		versionNumber: Fixed;          {itl5 resource version number }
		numberOfTables: UInt16;         {Number of tables it contains }
		reserved: array [0..2] of UInt16;            {Reserved for internal use }
		tableDirectory: array [0..0] of TableDirectoryRecord;    {Table directory records }
	end;
type
	RuleBasedTrslRecordPtr = ^RuleBasedTrslRecord;
	RuleBasedTrslRecord = record
		sourceType: SInt16;             {Transliterate target type for the LHS of the rule }
		targetType: SInt16;             {Transliterate target type for the RHS of the rule }
		formatNumber: SInt16;           {Transliterate resource format number }
		propertyFlag: SInt16;           {Transliterate property flags }
		numberOfRules: SInt16;          {Number of rules following this field }
	end;

type
	ItlcRecordPtr = ^ItlcRecord;
	ItlcRecord = record
		itlcSystem: SInt16;             {default system script}
		itlcReserved: SInt16;           {reserved}
		itlcFontForce: SInt8;          {default font force flag}
		itlcIntlForce: SInt8;          {default intl force flag}
		itlcOldKybd: SInt8;            {MacPlus intl keybd flag}
		itlcFlags: SInt8;              {general flags}
		itlcIconOffset: SInt16;         {keyboard icon offset; not used in 7.0}
		itlcIconSide: SInt8;           {keyboard icon side; not used in 7.0}
		itlcIconRsvd: SInt8;           {rsvd for other icon info}
		itlcRegionCode: SInt16;         {preferred verXxx code}
		itlcSysFlags: SInt16;           {flags for setting system globals}
		itlcReserved4: array [0..31] of SInt8;      { for future use }
	end;
type
	ItlbRecordPtr = ^ItlbRecord;
	ItlbRecord = record
		itlbNumber: SInt16;             {itl0 id number}
		itlbDate: SInt16;               {itl1 id number}
		itlbSort: SInt16;               {itl2 id number}
		itlbFlags: SInt16;              {Script flags}
		itlbToken: SInt16;              {itl4 id number}
		itlbEncoding: SInt16;           {itl5 ID # (optional; char encoding)}
		itlbLang: SInt16;               {current language for script }
		itlbNumRep: SInt8;             {number representation code}
		itlbDateRep: SInt8;            {date representation code }
		itlbKeys: SInt16;               {KCHR id number}
		itlbIcon: SInt16;               {ID # of SICN or kcs#/kcs4/kcs8 suite.}
	end;
{ New ItlbExtRecord structure for System 7.0 }
type
	ItlbExtRecordPtr = ^ItlbExtRecord;
	ItlbExtRecord = record
		base: ItlbRecord;                   {un-extended ItlbRecord}
		itlbLocalSize: SInt32;          {size of script's local record}
		itlbMonoFond: SInt16;           {default monospace FOND ID}
		itlbMonoSize: SInt16;           {default monospace font size}
		itlbPrefFond: SInt16;           {preferred FOND ID}
		itlbPrefSize: SInt16;           {preferred font size}
		itlbSmallFond: SInt16;          {default small FOND ID}
		itlbSmallSize: SInt16;          {default small font size}
		itlbSysFond: SInt16;            {default system FOND ID}
		itlbSysSize: SInt16;            {default system font size}
		itlbAppFond: SInt16;            {default application FOND ID}
		itlbAppSize: SInt16;            {default application font size}
		itlbHelpFond: SInt16;           {default Help Mgr FOND ID}
		itlbHelpSize: SInt16;           {default Help Mgr font size}
		itlbValidStyles: Style;        {set of valid styles for script}
		itlbAliasStyle: Style;         {style (set) to mark aliases}
	end;


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
			b:					SInt16;
=======
			b: SInt16;
>>>>>>> origin/fixes_2.4
			);
	end;
type
	WideCharArrPtr = ^WideCharArr;
	WideCharArr = record
		size: SInt16;
		data: array [0..9] of WideChar;
	end;
type
	NumberParts = record
		version: SInt16;
		data: array [0..30] of WideChar;               { index by [tokLeftQuote..tokMaxSymbols] }
		pePlus: WideCharArr;
		peMinus: WideCharArr;
		peMinusPlus: WideCharArr;
		altNumTable: WideCharArr;
		reserved: packed array [0..19] of char;
	end;
	NumberPartsPtr = ^NumberParts;


type
	Itl4RecPtr = ^Itl4Rec;
	Itl4Rec = record
		flags: SInt16;                  {reserved}
		resourceType: SInt32;           {contains 'itl4'}
		resourceNum: SInt16;            {resource ID}
		version: SInt16;                {version number}
		resHeader1: SInt32;             {reserved}
		resHeader2: SInt32;             {reserved}
		numTables: SInt16;              {number of tables, one-based}
		mapOffset: SInt32;              {offset to table that maps byte to token}
		strOffset: SInt32;              {offset to routine that copies canonical string}
		fetchOffset: SInt32;            {offset to routine that gets next byte of character}
		unTokenOffset: SInt32;          {offset to table that maps token to canonical string}
		defPartsOffset: SInt32;         {offset to default number parts table}
		resOffset6: SInt32;             {reserved}
		resOffset7: SInt32;             {reserved}
		resOffset8: SInt32;             {reserved}
	end;
type
	Itl4Ptr = Itl4RecPtr;
	Itl4Handle = ^Itl4Ptr;
{ New NItl4Rec for System 7.0: }
type
	NItl4RecPtr = ^NItl4Rec;
	NItl4Rec = record
		flags: SInt16;                  {reserved}
		resourceType: SInt32;           {contains 'itl4'}
		resourceNum: SInt16;            {resource ID}
		version: SInt16;                {version number}
		format: SInt16;                 {format code}
		resHeader: SInt16;              {reserved}
		resHeader2: SInt32;             {reserved}
		numTables: SInt16;              {number of tables, one-based}
		mapOffset: SInt32;              {offset to table that maps byte to token}
		strOffset: SInt32;              {offset to routine that copies canonical string}
		fetchOffset: SInt32;            {offset to routine that gets next byte of character}
		unTokenOffset: SInt32;          {offset to table that maps token to canonical string}
		defPartsOffset: SInt32;         {offset to default number parts table}
		whtSpListOffset: SInt32;        {offset to white space code list}
		resOffset7: SInt32;             {reserved}
		resOffset8: SInt32;             {reserved}
		resLength1: SInt16;             {reserved}
		resLength2: SInt16;             {reserved}
		resLength3: SInt16;             {reserved}
		unTokenLength: SInt16;          {length of untoken table}
		defPartsLength: SInt16;         {length of default number parts table}
		whtSpListLength: SInt16;        {length of white space code list}
		resLength7: SInt16;             {reserved}
		resLength8: SInt16;             {reserved}
	end;
type
	NItl4Ptr = NItl4RecPtr;
	NItl4Handle = ^NItl4Ptr;

type
	TableDirectoryRecordPtr = ^TableDirectoryRecord;
	TableDirectoryRecord = record
		tableSignature: OSType;         {4 byte long table name }
		reserved: UInt32;               {Reserved for internal use }
		tableStartOffset: UInt32;       {Table start offset in byte}
		tableSize: UInt32;              {Table size in byte}
	end;
type
	Itl5RecordPtr = ^Itl5Record;
	Itl5Record = record
		versionNumber: Fixed;          {itl5 resource version number }
		numberOfTables: UInt16;         {Number of tables it contains }
		reserved: array [0..2] of UInt16;            {Reserved for internal use }
		tableDirectory: array [0..0] of TableDirectoryRecord;    {Table directory records }
	end;
type
	RuleBasedTrslRecordPtr = ^RuleBasedTrslRecord;
	RuleBasedTrslRecord = record
		sourceType: SInt16;             {Transliterate target type for the LHS of the rule }
		targetType: SInt16;             {Transliterate target type for the RHS of the rule }
		formatNumber: SInt16;           {Transliterate resource format number }
		propertyFlag: SInt16;           {Transliterate property flags }
		numberOfRules: SInt16;          {Number of rules following this field }
	end;

type
	ItlcRecordPtr = ^ItlcRecord;
	ItlcRecord = record
		itlcSystem: SInt16;             {default system script}
		itlcReserved: SInt16;           {reserved}
		itlcFontForce: SInt8;          {default font force flag}
		itlcIntlForce: SInt8;          {default intl force flag}
		itlcOldKybd: SInt8;            {MacPlus intl keybd flag}
		itlcFlags: SInt8;              {general flags}
		itlcIconOffset: SInt16;         {keyboard icon offset; not used in 7.0}
		itlcIconSide: SInt8;           {keyboard icon side; not used in 7.0}
		itlcIconRsvd: SInt8;           {rsvd for other icon info}
		itlcRegionCode: SInt16;         {preferred verXxx code}
		itlcSysFlags: SInt16;           {flags for setting system globals}
		itlcReserved4: array [0..31] of SInt8;      { for future use }
	end;
type
	ItlbRecordPtr = ^ItlbRecord;
	ItlbRecord = record
		itlbNumber: SInt16;             {itl0 id number}
		itlbDate: SInt16;               {itl1 id number}
		itlbSort: SInt16;               {itl2 id number}
		itlbFlags: SInt16;              {Script flags}
		itlbToken: SInt16;              {itl4 id number}
		itlbEncoding: SInt16;           {itl5 ID # (optional; char encoding)}
		itlbLang: SInt16;               {current language for script }
		itlbNumRep: SInt8;             {number representation code}
		itlbDateRep: SInt8;            {date representation code }
		itlbKeys: SInt16;               {KCHR id number}
		itlbIcon: SInt16;               {ID # of SICN or kcs#/kcs4/kcs8 suite.}
	end;
{ New ItlbExtRecord structure for System 7.0 }
type
	ItlbExtRecordPtr = ^ItlbExtRecord;
	ItlbExtRecord = record
		base: ItlbRecord;                   {un-extended ItlbRecord}
		itlbLocalSize: SInt32;          {size of script's local record}
		itlbMonoFond: SInt16;           {default monospace FOND ID}
		itlbMonoSize: SInt16;           {default monospace font size}
		itlbPrefFond: SInt16;           {preferred FOND ID}
		itlbPrefSize: SInt16;           {preferred font size}
		itlbSmallFond: SInt16;          {default small FOND ID}
		itlbSmallSize: SInt16;          {default small font size}
		itlbSysFond: SInt16;            {default system FOND ID}
		itlbSysSize: SInt16;            {default system font size}
		itlbAppFond: SInt16;            {default application FOND ID}
		itlbAppSize: SInt16;            {default application font size}
		itlbHelpFond: SInt16;           {default Help Mgr FOND ID}
		itlbHelpSize: SInt16;           {default Help Mgr font size}
		itlbValidStyles: Style;        {set of valid styles for script}
		itlbAliasStyle: Style;         {style (set) to mark aliases}
	end;


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
