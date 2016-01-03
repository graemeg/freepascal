{
     File:       CarbonCore/OSUtils.h
 
     Contains:   OS Utilities Interfaces.
<<<<<<< HEAD
<<<<<<< HEAD
                 The contents of this header file are deprecated.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1985-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1985-2005 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
{      Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{      Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{      Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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

unit OSUtils;
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
=======
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
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
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,CFBase;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
=======
uses MacTypes,CFBase,MixedMode,MacMemory,DateTimeUtils,CFString,Endian;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,MixedMode,MacMemory,DateTimeUtils,CFString,Endian;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{  HandToHand and other memory utilties were moved to MacMemory.h }

{  Date and Time utilties were moved to DateTimeUtils.h }


{$ALIGN MAC68K}

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	useFree = 0;
	useATalk = 1;
	useAsync = 2;
	useExtClk = 3;    {Externally clocked}
	useMIDI = 4;

const
	false32b = 0;    {24 bit addressing error}
	true32b = 1;     {32 bit addressing error}

const
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ result types for RelString Call }
	sortsBefore = -1;   {first string < second string}
	sortsEqual = 0;    {first string = second string}
	sortsAfter = 1;     {first string > second string}

const
	dummyType = 0;
	vType = 1;
	ioQType = 2;
	drvQType = 3;
	evType = 4;
	fsQType = 5;
	sIQType = 6;
	dtQType = 7;
	nmType = 8;

type
	QTypes = SignedByte;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
type
	SysParmTypePtr = ^SysParmType;
	SysParmType = packed record
		valid: UInt8;
		aTalkA: UInt8;
		aTalkB: UInt8;
		config: UInt8;
		portA: SInt16;
		portB: SInt16;
		alarm: SInt32;
		font: SInt16;
		kbdPrint: SInt16;
		volClik: SInt16;
		misc: SInt16;
	end;
type
	SysPPtr = SysParmTypePtr;

type
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	QElemPtr = ^QElem;
	QElem = record
		qLink: QElemPtr;
		qType: SInt16;
		qData: array [0..0] of SInt16;
	end;
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	QHdr = record
		qFlags: {volatile} SInt16;
		qHead: {volatile} QElemPtr;
		qTail: {volatile} QElemPtr;
	end;
	QHdrPtr = ^QHdr;
=======
=======
>>>>>>> origin/fixes_2_2
	QHdrPtr = ^QHdr;
=======
>>>>>>> origin/fixes_2.4
	QHdr = record
		qFlags: {volatile} SInt16;
		qHead: {volatile} QElemPtr;
		qTail: {volatile} QElemPtr;
	end;
<<<<<<< HEAD
	DeferredTaskPtr = ^DeferredTask;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	QHdrPtr = ^QHdr;
>>>>>>> origin/fixes_2.4
{ 
    In order for MachineLocation to be endian-safe, a new member 
    has been added to the 'u' union in the structure. You are 
    encouraged to use the new member instead of the old one.
    
    If your code looked like this:
    
        MachineLocation.u.dlsDelta = isDLS? 0x80: 0x00;
    
    you should change it to this:
    
        MachineLocation.u.dls.Delta = isDLS? 0x80: 0x00;
    
    to be endian safe. The gmtDelta remains the same; the low 24-bits
    are used. Remember that order of assignment DOES matter:
    
    This will overwrite results:
    
        MachineLocation.u.dls.Delta = 0xAA;         // u = 0xAAGGGGGG; G=Garbage
        MachineLocation.u.gmtDelta = 0xBBBBBB;      // u = 0x00BBBBBB;
    
    when in fact reversing the assignment would have preserved the values:

        MachineLocation.u.gmtDelta = 0xBBBBBB;      // u = 0x00BBBBBB;  
        MachineLocation.u.dls.Delta = 0xAA;         // u = 0xAABBBBBB;
        
    NOTE:   The information regarding dlsDelta in Inside Mac is INCORRECT.
            It's always 0x80 for daylight-saving time or 0x00 for standard time.
}
type
	MachineLocationPtr = ^MachineLocation;
	MachineLocation = packed record
		latitude: Fract;
		longitude: Fract;
		case SInt16 of
{$ifc TARGET_RT_BIG_ENDIAN}
		0: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			dlsDelta: SInt8;									{ signed byte; daylight savings delta }
			);
{$endc}
		1: (
			gmtDelta: SIGNEDLONG;           { use low 24-bits only }
=======
=======
>>>>>>> origin/fixes_2_2
			dlsDelta: SInt8;									{  signed byte; daylight savings delta  }
			);
{$endc}
		1: (
			gmtDelta: SInt32;								{  use low 24-bits only  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
			dlsDelta: SInt8;									{ signed byte; daylight savings delta }
			);
{$endc}
		1: (
			gmtDelta: SIGNEDLONG;           { use low 24-bits only }
>>>>>>> origin/fixes_2.4
			);
		2: (
{$ifc TARGET_RT_LITTLE_ENDIAN}
			pad0,pad1,pad2: SInt8;
{$endc}
			Delta: SInt8;
			);
	end;
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  IsMetric()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CFLocaleGetValue() and the property kCFLocaleUsesMetricSystem
 *    to determine this value.
 *  
 *  Summary:
 *    Verifies whether the current script system is using the metric
 *    system or the English system of measurement.
 *  
 *  Discussion:
 *    The IsMetric function examines the metricSys field of the
 *    numeric-format resource (resource type 'itl0') to determine if
 *    the current script is using the metric system. A value of 255 in
 *    the metricSys field indicates that the metric system
 *    (centimeters, kilometers, milligrams, degrees Celsius, and so on)
 *    is being used. A value of 0 in the metricSys field indicates that
 *    the English system of measurement (inches, miles, ounces, degrees
 *    Fahrenheit, and so on) is used.
 *    If you want to use units of measurement different from that of
 *    the current script, you need to override the value of the
 *    metricSys field in the current numeric-format resource. You can
 *    do this by using your own version of the numeric-format resource
 *    instead of the current script system’s default international
 *    resource.
 *    The IsMetric function is the same as the IUMetric function, which
 *    was previously available with the International Utilities Package.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    TRUE if the metric system is being used; FALSE if the English
 *    system is being used.
 *  
 *  Summary:
 *    Verifies whether the current script system is using the metric
 *    system or the English system of measurement.
 *  
 *  Discussion:
 *    The IsMetric function examines the metricSys field of the
 *    numeric-format resource (resource type 'itl0') to determine if
 *    the current script is using the metric system. A value of 255 in
 *    the metricSys field indicates that the metric system
 *    (centimeters, kilometers, milligrams, degrees Celsius, and so on)
 *    is being used. A value of 0 in the metricSys field indicates that
 *    the English system of measurement (inches, miles, ounces, degrees
 *    Fahrenheit, and so on) is used.
 *    If you want to use units of measurement different from that of
 *    the current script, you need to override the value of the
 *    metricSys field in the current numeric-format resource. You can
 *    do this by using your own version of the numeric-format resource
 *    instead of the current script system’s default international
 *    resource.
 *    The IsMetric function is the same as the IUMetric function, which
 *    was previously available with the International Utilities Package.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    TRUE if the metric system is being used; FALSE if the English
 *    system is being used.
 *  
 *  Summary:
 *    Verifies whether the current script system is using the metric
 *    system or the English system of measurement.
 *  
 *  Discussion:
 *    The IsMetric function examines the metricSys field of the
 *    numeric-format resource (resource type 'itl0') to determine if
 *    the current script is using the metric system. A value of 255 in
 *    the metricSys field indicates that the metric system
 *    (centimeters, kilometers, milligrams, degrees Celsius, and so on)
 *    is being used. A value of 0 in the metricSys field indicates that
 *    the English system of measurement (inches, miles, ounces, degrees
 *    Fahrenheit, and so on) is used.
 *    If you want to use units of measurement different from that of
 *    the current script, you need to override the value of the
 *    metricSys field in the current numeric-format resource. You can
 *    do this by using your own version of the numeric-format resource
 *    instead of the current script system’s default international
 *    resource.
 *    The IsMetric function is the same as the IUMetric function, which
 *    was previously available with the International Utilities Package.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    TRUE if the metric system is being used; FALSE if the English
 *    system is being used.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
=======
 *  IsMetric()
 *  
 *  Summary:
 *    Verifies whether the current script system is using the metric
 *    system or the English system of measurement.
 *  
 *  Discussion:
 *    The IsMetric function examines the metricSys field of the
 *    numeric-format resource (resource type 'itl0') to determine if
 *    the current script is using the metric system. A value of 255 in
 *    the metricSys field indicates that the metric system
 *    (centimeters, kilometers, milligrams, degrees Celsius, and so on)
 *    is being used. A value of 0 in the metricSys field indicates that
 *    the English system of measurement (inches, miles, ounces, degrees
 *    Fahrenheit, and so on) is used.
 *    If you want to use units of measurement different from that of
 *    the current script, you need to override the value of the
 *    metricSys field in the current numeric-format resource. You can
 *    do this by using your own version of the numeric-format resource
 *    instead of the current script system’s default international
 *    resource.
 *    The IsMetric function is the same as the IUMetric function, which
 *    was previously available with the International Utilities Package.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    TRUE if the metric system is being used; FALSE if the English
 *    system is being used.
=======
 *  IsMetric()
 *  
 *  Summary:
 *    Verifies whether the current script system is using the metric
 *    system or the English system of measurement.
 *  
 *  Discussion:
 *    The IsMetric function examines the metricSys field of the
 *    numeric-format resource (resource type 'itl0') to determine if
 *    the current script is using the metric system. A value of 255 in
 *    the metricSys field indicates that the metric system
 *    (centimeters, kilometers, milligrams, degrees Celsius, and so on)
 *    is being used. A value of 0 in the metricSys field indicates that
 *    the English system of measurement (inches, miles, ounces, degrees
 *    Fahrenheit, and so on) is used.
 *    If you want to use units of measurement different from that of
 *    the current script, you need to override the value of the
 *    metricSys field in the current numeric-format resource. You can
 *    do this by using your own version of the numeric-format resource
 *    instead of the current script system’s default international
 *    resource.
 *    The IsMetric function is the same as the IUMetric function, which
 *    was previously available with the International Utilities Package.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    TRUE if the metric system is being used; FALSE if the English
 *    system is being used.
=======
=======
>>>>>>> origin/fixes_2_2
 *  IsMetric()
 *  
 *  Summary:
 *    Verifies whether the current script system is using the metric
 *    system or the English system of measurement.
 *  
 *  Discussion:
 *    The IsMetric function examines the metricSys field of the
 *    numeric-format resource (resource type 'itl0') to determine if
 *    the current script is using the metric system. A value of 255 in
 *    the metricSys field indicates that the metric system
 *    (centimeters, kilometers, milligrams, degrees Celsius, and so on)
 *    is being used. A value of 0 in the metricSys field indicates that
 *    the English system of measurement (inches, miles, ounces, degrees
 *    Fahrenheit, and so on) is used.
 *    If you want to use units of measurement different from that of
 *    the current script, you need to override the value of the
 *    metricSys field in the current numeric-format resource. You can
 *    do this by using your own version of the numeric-format resource
 *    instead of the current script system’s default international
 *    resource.
 *    The IsMetric function is the same as the IUMetric function, which
 *    was previously available with the International Utilities Package.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    TRUE if the metric system is being used; FALSE if the English
 *    system is being used.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function IsMetric: Boolean; external name '_IsMetric';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  Delay()
 *  
 *  Summary:
 *    Delays execture for the specified amount of time.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    numTicks:
 *      the number of ticks to delay  for
 *    
 *    finalTicks:
 *      on return, if not NULL, will contain the value of TickCount()
 *      at the end of the delay period
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure Delay( numTicks: UNSIGNEDLONG; var finalTicks: UNSIGNEDLONG ); external name '_Delay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  Enqueue()
 *  
 *  Summary:
 *    Atomically adds a queue element to the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Enqueue inserts the given element into the queue in a
 *    multi-thread safe way.  If the element is already in the queue,
 *    or in some other queue, the data structures will be corrupted and
 *    will likely cause a crash or infinite loop.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be inserted
 *    
 *    qHeader:
 *      a pointer to the queue header.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure Enqueue( qElement: QElemPtr; qHeader: QHdrPtr ); external name '_Enqueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  Dequeue()
 *  
 *  Summary:
 *    Atomically removes a queue element from the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Dequeue removes the given element from the queue in a
 *    multi-thread safe way.  If the element is not in the queue, qErr
 *    is returned.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be removed
 *    
 *    qHeader:
 *      a pointer to the queue header.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function Dequeue( qElement: QElemPtr; qHeader: QHdrPtr ): OSErr; external name '_Dequeue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  Delay()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  MakeDataExecutable()
 *  
 *  Summary:
 *    Notifies the system that the specified data is subject to
 *    execution.
 *  
 *  Discussion:
 *    On some computer architectures it is necessary to tell the
 *    processor that an area of memory should be made executable.  This
 *    function does the necessary operations ( if possible ) to make it
 *    possible to execute code in the given address range.
 *    MakeDataExecutable is not supported for 64-bit applications. Use
 *    sys_icache_invalidate(3) and/or mprotect(2) as appropriate.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    baseAddress:
 *      the starting address to be made executable
 *    
 *    length:
 *      the length of the data pointed to by the baseAddress parameter.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function IsMetric: Boolean; external name '_IsMetric';
=======
procedure Delay( numTicks: UInt32; var finalTicks: UInt32 ); external name '_Delay';
>>>>>>> graemeg/fixes_2_2
=======
procedure Delay( numTicks: UInt32; var finalTicks: UInt32 ); external name '_Delay';
>>>>>>> origin/fixes_2_2
=======
procedure MakeDataExecutable( baseAddress: UnivPtr; length: UNSIGNEDLONG ); external name '_MakeDataExecutable';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Delay()
 *  
 *  Summary:
 *    Delays execture for the specified amount of time.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    numTicks:
 *      the number of ticks to delay  for
 *    
 *    finalTicks:
 *      on return, if not NULL, will contain the value of TickCount()
 *      at the end of the delay period
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure Delay( numTicks: UNSIGNEDLONG; var finalTicks: UNSIGNEDLONG ); external name '_Delay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 *  WriteParam()   *** DEPRECATED ***
=======
 *  ReadLocation()
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains information about a geographic location or time zone.
 *  
 *  Discussion:
 *    The latitude and longitude are stored as Fract values, giving
 *    accuracy to within one foot. For example, a Fract value of 1.0
 *    equals 90 degrees –1.0 equals –90 degrees and –2.0 equals –180
 *    degrees.
 *    To convert these values to a degrees format, you need to convert
 *    the Fract values first to the Fixed data type, then to the
 *    LongInt data type. Use the Mathematical and Logical Utilities
 *    functions Fract2Fix and Fix2Long to accomplish this task.
 *    The DST value is a signed byte value that specifies the offset
 *    for the hour field—whether to add one hour, subtract one hour, or
 *    make no change at all.
 *    The GMT value is in seconds east of GMT. For example, San
 *    Francisco is at –28,800 seconds (8 hours * 3,600 seconds per
 *    hour) east of GMT. The gmtDelta field is a 3-byte value contained
 *    in a long word, so you must take care to get it properly.
 *    For more information on the Fract data type and the conversion
 *    routines Long2Fix, Fix2Fract, Fract2Fix, and Fix2Long, see
 *    Mathematical and Logical Utilities.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    loc:
 *      On return, the fields of the geographic location structure
 *      containing the geographic location and the time-zone
 *      information.
 *      You can get values for the latitude, longitude, daylight
 *      savings time (DST), or Greenwich mean time (GMT). If the
 *      geographic location record has never been set, all fields
 *      contain 0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function WriteParam: OSErr; external name '_WriteParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure ReadLocation( var loc: MachineLocation ); external name '_ReadLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  TickCount()
 *  
 *  Summary:
 *    Obtains the current number of ticks (a tick is approximately 1/60
 *    of a second) approximately since the system last started up.
 *  
 *  Discussion:
 *    The TickCount function returns an unsigned 32-bit integer that
 *    indicates the current number of ticks since the system last
 *    started up. You can use this value to compare the number of ticks
 *    that have elapsed since a given event or other action occurred.
 *    For example, you could compare the current value returned by
 *    TickCount with the value of the when field of an event
 *    structure.
 *    The tick count rolls over in approximately 2 years 3 months,
 *    which means you should not use this to time intervals which may
 *    exceed ( or even approach ) this interval.
 *    Do not rely on the tick count being exact; it is usually accurate
 *    to within one tick, but this level of accuracy is not guaranteed.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the tick count
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Atomically adds a queue element to the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Enqueue inserts the given element into the queue in a
 *    multi-thread safe way.  If the element is already in the queue,
 *    or in some other queue, the data structures will be corrupted and
 *    will likely cause a crash or infinite loop.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be inserted
 *    
 *    qHeader:
 *      a pointer to the queue header.
 *  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TickCount: UInt32; external name '_TickCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CSCopyUserName()
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the user name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    useShortName:
 *      A Boolean value that specifies whether to return the short name
 *      or full name of the user.
 *  
 *  Result:
 *    the requested name in a CFStringRef.  You should release this
 *    when you are done with it.
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Atomically removes a queue element from the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Dequeue removes the given element from the queue in a
 *    multi-thread safe way.  If the element is not in the queue, qErr
 *    is returned.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be removed
 *    
 *    qHeader:
 *      a pointer to the queue header.
 *  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function CSCopyUserName( useShortName: Boolean ): CFStringRef; external name '_CSCopyUserName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  MakeDataExecutable()
 *  
 *  Summary:
 *    Notifies the system that the specified data is subject to
 *    execution.
 *  
 *  Discussion:
 *    On some computer architectures it is necessary to tell the
 *    processor that an area of memory should be made executable.  This
 *    function does the necessary operations ( if possible ) to make it
 *    possible to execute code in the given address range.
 *    MakeDataExecutable is not supported for 64-bit applications. Use
 *    sys_icache_invalidate(3) and/or mprotect(2) as appropriate.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    baseAddress:
 *      the starting address to be made executable
 *    
 *    length:
 *      the length of the data pointed to by the baseAddress parameter.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
<<<<<<< HEAD
function IsMetric: Boolean; external name '_IsMetric';
=======
procedure MakeDataExecutable( baseAddress: UnivPtr; length: UNSIGNEDLONG ); external name '_MakeDataExecutable';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  Delay()
 *  
 *  Summary:
 *    Delays execture for the specified amount of time.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    numTicks:
 *      the number of ticks to delay  for
 *    
 *    finalTicks:
 *      on return, if not NULL, will contain the value of TickCount()
 *      at the end of the delay period
=======
 *  ReadLocation()
 *  
 *  Summary:
 *    Obtains information about a geographic location or time zone.
 *  
 *  Discussion:
 *    The latitude and longitude are stored as Fract values, giving
 *    accuracy to within one foot. For example, a Fract value of 1.0
 *    equals 90 degrees –1.0 equals –90 degrees and –2.0 equals –180
 *    degrees.
 *    To convert these values to a degrees format, you need to convert
 *    the Fract values first to the Fixed data type, then to the
 *    LongInt data type. Use the Mathematical and Logical Utilities
 *    functions Fract2Fix and Fix2Long to accomplish this task.
 *    The DST value is a signed byte value that specifies the offset
 *    for the hour field—whether to add one hour, subtract one hour, or
 *    make no change at all.
 *    The GMT value is in seconds east of GMT. For example, San
 *    Francisco is at –28,800 seconds (8 hours * 3,600 seconds per
 *    hour) east of GMT. The gmtDelta field is a 3-byte value contained
 *    in a long word, so you must take care to get it properly.
 *    For more information on the Fract data type and the conversion
 *    routines Long2Fix, Fix2Fract, Fract2Fix, and Fix2Long, see
 *    Mathematical and Logical Utilities.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    loc:
 *      On return, the fields of the geographic location structure
 *      containing the geographic location and the time-zone
 *      information.
 *      You can get values for the latitude, longitude, daylight
 *      savings time (DST), or Greenwich mean time (GMT). If the
 *      geographic location record has never been set, all fields
 *      contain 0.
>>>>>>> origin/cpstrnew
 *  
=======
{
 *  CSCopyMachineName()
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the computer
 *    name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the name of this machine in a CFStringRef.  You should release
 *    this when you are done with it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function CSCopyMachineName: CFStringRef; external name '_CSCopyMachineName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


const
	useFree = 0;
	useATalk = 1;
	useAsync = 2;
	useExtClk = 3;    {Externally clocked}
	useMIDI = 4;

const
	false32b = 0;    {24 bit addressing error}
	true32b = 1;     {32 bit addressing error}

type
	SysPPtr = UnivPtr;

function GetMMUMode: SInt8; inline;
procedure SwapMMUMode( var mode: SInt8 ); inline;

{$ifc not TARGET_CPU_64}
{
 *  GetSysPPtr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Don't use this function; it always returns NULL on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetSysPPtr: SysPPtr; external name '_GetSysPPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
    NOTE: SysBeep() has been moved to Sound.h.  
 We could not automatically #include Sound.h in this file
 because Sound.h indirectly #include's OSUtils.h which
 would make a circular include.
 }
{$endc} {not TARGET_CPU_64}

type
	DeferredTaskProcPtr = procedure( dtParam: SIGNEDLONG );
	DeferredTaskUPP = DeferredTaskProcPtr;
{
 *  NewDeferredTaskUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDeferredTaskUPP( userRoutine: DeferredTaskProcPtr ): DeferredTaskUPP; external name '_NewDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDeferredTaskUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDeferredTaskUPP( userUPP: DeferredTaskUPP ); external name '_DisposeDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDeferredTaskUPP()
 *  
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure Delay( numTicks: UNSIGNEDLONG; var finalTicks: UNSIGNEDLONG ); external name '_Delay';
=======
procedure ReadLocation( var loc: MachineLocation ); external name '_ReadLocation';
>>>>>>> origin/cpstrnew
=======
procedure ReadLocation( var loc: MachineLocation ); external name '_ReadLocation';
>>>>>>> origin/fixes_2_2
=======
procedure InvokeDeferredTaskUPP( dtParam: SIGNEDLONG; userUPP: DeferredTaskUPP ); external name '_InvokeDeferredTaskUPP';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

type
	DeferredTask = record
		qLink: {volatile} QElemPtr;
		qType: SInt16;
		dtFlags: {volatile} SInt16;
		dtAddr: DeferredTaskUPP;
		dtParam: SIGNEDLONG;
		dtReserved: SIGNEDLONG;
	end;
	DeferredTaskPtr = ^DeferredTask;
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  TickCount()
 *  
 *  Summary:
 *    Obtains the current number of ticks (a tick is approximately 1/60
 *    of a second) approximately since the system last started up.
 *  
 *  Discussion:
 *    The TickCount function returns an unsigned 32-bit integer that
 *    indicates the current number of ticks since the system last
 *    started up. You can use this value to compare the number of ticks
 *    that have elapsed since a given event or other action occurred.
 *    For example, you could compare the current value returned by
 *    TickCount with the value of the when field of an event
 *    structure.
 *    The tick count rolls over in approximately 2 years 3 months,
 *    which means you should not use this to time intervals which may
 *    exceed ( or even approach ) this interval.
 *    Do not rely on the tick count being exact; it is usually accurate
 *    to within one tick, but this level of accuracy is not guaranteed.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the tick count
 *  
 *  Summary:
 *    Atomically adds a queue element to the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Enqueue inserts the given element into the queue in a
 *    multi-thread safe way.  If the element is already in the queue,
 *    or in some other queue, the data structures will be corrupted and
 *    will likely cause a crash or infinite loop.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be inserted
 *    
 *    qHeader:
 *      a pointer to the queue header.
=======
 *  WriteLocation()   *** DEPRECATED ***
=======
 *  DTInstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DTInstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTInstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  TickCount()
>>>>>>> origin/fixes_2_2
=======
 *  DTUninstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function DTUninstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTUninstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetCurrentA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetCurrentA5() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetCurrentA5: SIGNEDLONG; external name '_SetCurrentA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetA5() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
<<<<<<< HEAD
 *  Summary:
 *    Returns a reference to the CFString that represents the user name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    useShortName:
 *      A Boolean value that specifies whether to return the short name
 *      or full name of the user.
 *  
 *  Result:
 *    the requested name in a CFStringRef.  You should release this
 *    when you are done with it.
 *  
 *  Summary:
 *    Atomically removes a queue element from the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Dequeue removes the given element from the queue in a
 *    multi-thread safe way.  If the element is not in the queue, qErr
 *    is returned.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be removed
 *    
 *    qHeader:
 *      a pointer to the queue header.
 *  
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetA5( newA5: SIGNEDLONG ): SIGNEDLONG; external name '_SetA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  MakeDataExecutable()
 *  
 *  Summary:
 *    Notifies the system that the specified data is subject to
 *    execution.
 *  
 *  Discussion:
 *    On some computer architectures it is necessary to tell the
 *    processor that an area of memory should be made executable.  This
 *    function does the necessary operations ( if possible ) to make it
 *    possible to execute code in the given address range.
 *    MakeDataExecutable is not supported for 64-bit applications. Use
 *    sys_icache_invalidate(3) and/or mprotect(2) as appropriate.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    baseAddress:
 *      the starting address to be made executable
 *    
 *    length:
 *      the length of the data pointed to by the baseAddress parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only]
>>>>>>> graemeg/cpstrnew
=======
 *  CSCopyMachineName()
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the computer
 *    name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the name of this machine in a CFStringRef.  You should release
 *    this when you are done with it.
 *  
=======
{
 *  InitUtil()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    It is not necessary to call InitUtil on Mac OS X.  You should
 *    remove all calls to this from your code.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function InitUtil: OSErr; external name '_InitUtil';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


<<<<<<< HEAD
const
	useFree = 0;
	useATalk = 1;
	useAsync = 2;
	useExtClk = 3;    {Externally clocked}
	useMIDI = 4;

const
	false32b = 0;    {24 bit addressing error}
	true32b = 1;     {32 bit addressing error}

type
	SysPPtr = UnivPtr;

function GetMMUMode: SInt8; inline;
procedure SwapMMUMode( var mode: SInt8 ); inline;

{$ifc not TARGET_CPU_64}
{
 *  GetSysPPtr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Don't use this function; it always returns NULL on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function IsMetric: Boolean; external name '_IsMetric';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
procedure MakeDataExecutable( baseAddress: UnivPtr; length: UNSIGNEDLONG ); external name '_MakeDataExecutable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

=======
function GetSysPPtr: SysPPtr; external name '_GetSysPPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
<<<<<<< HEAD
 *  Delay()
 *  
 *  Summary:
 *    Delays execture for the specified amount of time.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    numTicks:
 *      the number of ticks to delay  for
 *    
 *    finalTicks:
 *      on return, if not NULL, will contain the value of TickCount()
 *      at the end of the delay period
=======
 *  ReadLocation()
 *  
 *  Summary:
 *    Obtains information about a geographic location or time zone.
 *  
 *  Discussion:
 *    The latitude and longitude are stored as Fract values, giving
 *    accuracy to within one foot. For example, a Fract value of 1.0
 *    equals 90 degrees –1.0 equals –90 degrees and –2.0 equals –180
 *    degrees.
 *    To convert these values to a degrees format, you need to convert
 *    the Fract values first to the Fixed data type, then to the
 *    LongInt data type. Use the Mathematical and Logical Utilities
 *    functions Fract2Fix and Fix2Long to accomplish this task.
 *    The DST value is a signed byte value that specifies the offset
 *    for the hour field—whether to add one hour, subtract one hour, or
 *    make no change at all.
 *    The GMT value is in seconds east of GMT. For example, San
 *    Francisco is at –28,800 seconds (8 hours * 3,600 seconds per
 *    hour) east of GMT. The gmtDelta field is a 3-byte value contained
 *    in a long word, so you must take care to get it properly.
 *    For more information on the Fract data type and the conversion
 *    routines Long2Fix, Fix2Fract, Fract2Fix, and Fix2Long, see
 *    Mathematical and Logical Utilities.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    loc:
 *      On return, the fields of the geographic location structure
 *      containing the geographic location and the time-zone
 *      information.
 *      You can get values for the latitude, longitude, daylight
 *      savings time (DST), or Greenwich mean time (GMT). If the
 *      geographic location record has never been set, all fields
 *      contain 0.
>>>>>>> graemeg/cpstrnew
=======
    NOTE: SysBeep() has been moved to Sound.h.  
 We could not automatically #include Sound.h in this file
 because Sound.h indirectly #include's OSUtils.h which
 would make a circular include.
 }
{$endc} {not TARGET_CPU_64}

type
	DeferredTaskProcPtr = procedure( dtParam: SIGNEDLONG );
	DeferredTaskUPP = DeferredTaskProcPtr;
{
 *  NewDeferredTaskUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure Delay( numTicks: UNSIGNEDLONG; var finalTicks: UNSIGNEDLONG ); external name '_Delay';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
procedure ReadLocation( var loc: MachineLocation ); external name '_ReadLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  TickCount()
 *  
 *  Summary:
 *    Obtains the current number of ticks (a tick is approximately 1/60
 *    of a second) approximately since the system last started up.
 *  
 *  Discussion:
 *    The TickCount function returns an unsigned 32-bit integer that
 *    indicates the current number of ticks since the system last
 *    started up. You can use this value to compare the number of ticks
 *    that have elapsed since a given event or other action occurred.
 *    For example, you could compare the current value returned by
 *    TickCount with the value of the when field of an event
 *    structure.
 *    The tick count rolls over in approximately 2 years 3 months,
 *    which means you should not use this to time intervals which may
 *    exceed ( or even approach ) this interval.
 *    Do not rely on the tick count being exact; it is usually accurate
 *    to within one tick, but this level of accuracy is not guaranteed.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the tick count
 *  
 *  Summary:
 *    Atomically adds a queue element to the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Enqueue inserts the given element into the queue in a
 *    multi-thread safe way.  If the element is already in the queue,
 *    or in some other queue, the data structures will be corrupted and
 *    will likely cause a crash or infinite loop.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be inserted
 *    
 *    qHeader:
 *      a pointer to the queue header.
=======
function NewDeferredTaskUPP( userRoutine: DeferredTaskProcPtr ): DeferredTaskUPP; external name '_NewDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDeferredTaskUPP()
>>>>>>> origin/cpstrnew
=======
{
 *  SetCurrentA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetCurrentA5() on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetCurrentA5: SInt32; external name '_SetCurrentA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetA5() on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetA5( newA5: SInt32 ): SInt32; external name '_SetA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  InitUtil()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function InitUtil: OSErr; external name '_InitUtil';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  MakeDataExecutable()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
procedure MakeDataExecutable( baseAddress: UnivPtr; length: UInt32 ); external name '_MakeDataExecutable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ReadLocation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure ReadLocation( var loc: MachineLocation ); external name '_ReadLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  WriteLocation()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure WriteLocation( const (*var*) loc: MachineLocation ); external name '_WriteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)


{
 *  TickCount()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure Enqueue( qElement: QElemPtr; qHeader: QHdrPtr ); external name '_Enqueue';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
=======
function TickCount: UInt32; external name '_TickCount';
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDeferredTaskUPP( userUPP: DeferredTaskUPP ); external name '_DisposeDeferredTaskUPP';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
<<<<<<< HEAD
 *  CSCopyUserName()
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the user name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    useShortName:
 *      A Boolean value that specifies whether to return the short name
 *      or full name of the user.
 *  
 *  Result:
 *    the requested name in a CFStringRef.  You should release this
 *    when you are done with it.
 *  
 *  Summary:
 *    Atomically removes a queue element from the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Dequeue removes the given element from the queue in a
 *    multi-thread safe way.  If the element is not in the queue, qErr
 *    is returned.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be removed
 *    
 *    qHeader:
 *      a pointer to the queue header.
 *  
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TickCount: UInt32; external name '_TickCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CSCopyUserName()
 *  
>>>>>>> graemeg/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
function Dequeue( qElement: QElemPtr; qHeader: QHdrPtr ): OSErr; external name '_Dequeue';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
=======
function CSCopyUserName( useShortName: Boolean ): CFStringRef; external name '_CSCopyUserName';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  MakeDataExecutable()
 *  
 *  Summary:
 *    Notifies the system that the specified data is subject to
 *    execution.
 *  
 *  Discussion:
 *    On some computer architectures it is necessary to tell the
 *    processor that an area of memory should be made executable.  This
 *    function does the necessary operations ( if possible ) to make it
 *    possible to execute code in the given address range.
 *    MakeDataExecutable is not supported for 64-bit applications. Use
 *    sys_icache_invalidate(3) and/or mprotect(2) as appropriate.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    baseAddress:
 *      the starting address to be made executable
 *    
 *    length:
 *      the length of the data pointed to by the baseAddress parameter.
=======
 *  CSCopyMachineName()
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the computer
 *    name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the name of this machine in a CFStringRef.  You should release
 *    this when you are done with it.
 *  
=======
function CSCopyUserName( useShortName: Boolean ): CFStringRef; external name '_CSCopyUserName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CSCopyMachineName()
 *  
>>>>>>> graemeg/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function CSCopyMachineName: CFStringRef; external name '_CSCopyMachineName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
const
	useFree = 0;
	useATalk = 1;
	useAsync = 2;
	useExtClk = 3;    {Externally clocked}
	useMIDI = 4;

const
	false32b = 0;    {24 bit addressing error}
	true32b = 1;     {32 bit addressing error}

type
	SysPPtr = UnivPtr;

function GetMMUMode: SInt8; inline;
procedure SwapMMUMode( var mode: SInt8 ); inline;

{$ifc not TARGET_CPU_64}
{
 *  GetSysPPtr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Don't use this function; it always returns NULL on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetSysPPtr: SysPPtr; external name '_GetSysPPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
    NOTE: SysBeep() has been moved to Sound.h.  
 We could not automatically #include Sound.h in this file
 because Sound.h indirectly #include's OSUtils.h which
 would make a circular include.
 }
{$endc} {not TARGET_CPU_64}

type
	DeferredTaskProcPtr = procedure( dtParam: SIGNEDLONG );
	DeferredTaskUPP = DeferredTaskProcPtr;
{
 *  NewDeferredTaskUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDeferredTaskUPP( userRoutine: DeferredTaskProcPtr ): DeferredTaskUPP; external name '_NewDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDeferredTaskUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure MakeDataExecutable( baseAddress: UnivPtr; length: UNSIGNEDLONG ); external name '_MakeDataExecutable';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
=======
procedure DisposeDeferredTaskUPP( userUPP: DeferredTaskUPP ); external name '_DisposeDeferredTaskUPP';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD

{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/cpstrnew
{
 *  InvokeDeferredTaskUPP()
 *  
 *  Summary:
 *    Obtains information about a geographic location or time zone.
 *  
 *  Discussion:
 *    The latitude and longitude are stored as Fract values, giving
 *    accuracy to within one foot. For example, a Fract value of 1.0
 *    equals 90 degrees –1.0 equals –90 degrees and –2.0 equals –180
 *    degrees.
 *    To convert these values to a degrees format, you need to convert
 *    the Fract values first to the Fixed data type, then to the
 *    LongInt data type. Use the Mathematical and Logical Utilities
 *    functions Fract2Fix and Fix2Long to accomplish this task.
 *    The DST value is a signed byte value that specifies the offset
 *    for the hour field—whether to add one hour, subtract one hour, or
 *    make no change at all.
 *    The GMT value is in seconds east of GMT. For example, San
 *    Francisco is at –28,800 seconds (8 hours * 3,600 seconds per
 *    hour) east of GMT. The gmtDelta field is a 3-byte value contained
 *    in a long word, so you must take care to get it properly.
 *    For more information on the Fract data type and the conversion
 *    routines Long2Fix, Fix2Fract, Fract2Fix, and Fix2Long, see
 *    Mathematical and Logical Utilities.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    loc:
 *      On return, the fields of the geographic location structure
 *      containing the geographic location and the time-zone
 *      information.
 *      You can get values for the latitude, longitude, daylight
 *      savings time (DST), or Greenwich mean time (GMT). If the
 *      geographic location record has never been set, all fields
 *      contain 0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure ReadLocation( var loc: MachineLocation ); external name '_ReadLocation';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
procedure InvokeDeferredTaskUPP( dtParam: SIGNEDLONG; userUPP: DeferredTaskUPP ); external name '_InvokeDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

type
	DeferredTask = record
		qLink: {volatile} QElemPtr;
		qType: SInt16;
		dtFlags: {volatile} SInt16;
		dtAddr: DeferredTaskUPP;
		dtParam: SIGNEDLONG;
		dtReserved: SIGNEDLONG;
	end;
	DeferredTaskPtr = ^DeferredTask;
{$ifc not TARGET_CPU_64}
{
 *  DTInstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DTInstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTInstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  DTUninstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Summary:
 *    Obtains the current number of ticks (a tick is approximately 1/60
 *    of a second) approximately since the system last started up.
 *  
 *  Discussion:
 *    The TickCount function returns an unsigned 32-bit integer that
 *    indicates the current number of ticks since the system last
 *    started up. You can use this value to compare the number of ticks
 *    that have elapsed since a given event or other action occurred.
 *    For example, you could compare the current value returned by
 *    TickCount with the value of the when field of an event
 *    structure.
 *    The tick count rolls over in approximately 2 years 3 months,
 *    which means you should not use this to time intervals which may
 *    exceed ( or even approach ) this interval.
 *    Do not rely on the tick count being exact; it is usually accurate
 *    to within one tick, but this level of accuracy is not guaranteed.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the tick count
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function DTUninstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTUninstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetCurrentA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetCurrentA5() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function TickCount: UInt32; external name '_TickCount';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function SetCurrentA5: SIGNEDLONG; external name '_SetCurrentA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  SetA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetA5() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the user name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    useShortName:
 *      A Boolean value that specifies whether to return the short name
 *      or full name of the user.
 *  
 *  Result:
 *    the requested name in a CFStringRef.  You should release this
 *    when you are done with it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function CSCopyUserName( useShortName: Boolean ): CFStringRef; external name '_CSCopyUserName';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function SetA5( newA5: SIGNEDLONG ): SIGNEDLONG; external name '_SetA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  InitUtil()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    It is not necessary to call InitUtil on Mac OS X.  You should
 *    remove all calls to this from your code.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the computer
 *    name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the name of this machine in a CFStringRef.  You should release
 *    this when you are done with it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function CSCopyMachineName: CFStringRef; external name '_CSCopyMachineName';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


const
	useFree = 0;
	useATalk = 1;
	useAsync = 2;
	useExtClk = 3;    {Externally clocked}
	useMIDI = 4;

const
	false32b = 0;    {24 bit addressing error}
	true32b = 1;     {32 bit addressing error}
<<<<<<< HEAD

type
	SysPPtr = UnivPtr;

function GetMMUMode: SInt8; inline;
procedure SwapMMUMode( var mode: SInt8 ); inline;

=======

type
	SysPPtr = UnivPtr;

function GetMMUMode: SInt8; inline;
procedure SwapMMUMode( var mode: SInt8 ); inline;

>>>>>>> graemeg/cpstrnew
{$ifc not TARGET_CPU_64}
{
 *  Delay()
 *  
 *  Summary:
 *    Delays execture for the specified amount of time.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    numTicks:
 *      the number of ticks to delay  for
 *    
 *    finalTicks:
 *      on return, if not NULL, will contain the value of TickCount()
 *      at the end of the delay period
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetSysPPtr: SysPPtr; external name '_GetSysPPtr';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
    NOTE: SysBeep() has been moved to Sound.h.  
 We could not automatically #include Sound.h in this file
 because Sound.h indirectly #include's OSUtils.h which
 would make a circular include.
 }
{$endc} {not TARGET_CPU_64}

type
	DeferredTaskProcPtr = procedure( dtParam: SIGNEDLONG );
	DeferredTaskUPP = DeferredTaskProcPtr;
{
 *  NewDeferredTaskUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDeferredTaskUPP( userRoutine: DeferredTaskProcPtr ): DeferredTaskUPP; external name '_NewDeferredTaskUPP';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
 *  DisposeDeferredTaskUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDeferredTaskUPP( userUPP: DeferredTaskUPP ); external name '_DisposeDeferredTaskUPP';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
 *  InvokeDeferredTaskUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
<<<<<<< HEAD
 }
procedure InvokeDeferredTaskUPP( dtParam: SIGNEDLONG; userUPP: DeferredTaskUPP ); external name '_InvokeDeferredTaskUPP';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

type
	DeferredTask = record
		qLink: {volatile} QElemPtr;
		qType: SInt16;
		dtFlags: {volatile} SInt16;
		dtAddr: DeferredTaskUPP;
		dtParam: SIGNEDLONG;
		dtReserved: SIGNEDLONG;
	end;
	DeferredTaskPtr = ^DeferredTask;
{$ifc not TARGET_CPU_64}
{
 *  DTInstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure Delay( numTicks: UNSIGNEDLONG; var finalTicks: UNSIGNEDLONG ); external name '_Delay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  Enqueue()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Atomically adds a queue element to the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Enqueue inserts the given element into the queue in a
 *    multi-thread safe way.  If the element is already in the queue,
 *    or in some other queue, the data structures will be corrupted and
 *    will likely cause a crash or infinite loop.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be inserted
 *    
 *    qHeader:
 *      a pointer to the queue header.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DTInstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTInstall';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
 }
procedure InvokeDeferredTaskUPP( dtParam: SIGNEDLONG; userUPP: DeferredTaskUPP ); external name '_InvokeDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure Enqueue( qElement: QElemPtr; qHeader: QHdrPtr ); external name '_Enqueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew

type
	DeferredTask = record
		qLink: {volatile} QElemPtr;
		qType: SInt16;
		dtFlags: {volatile} SInt16;
		dtAddr: DeferredTaskUPP;
		dtParam: SIGNEDLONG;
		dtReserved: SIGNEDLONG;
	end;
	DeferredTaskPtr = ^DeferredTask;
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  DTUninstall()   *** DEPRECATED ***
=======
 *  DTInstall()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
=======
 *  Dequeue()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Atomically removes a queue element from the given queue
 *  
 *  Discussion:
 *    A queue ( represented by a QHdrPtr ) is a singly linked list of
 *    elements.  Dequeue removes the given element from the queue in a
 *    multi-thread safe way.  If the element is not in the queue, qErr
 *    is returned.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    qElement:
 *      a pointer to the element to be removed
 *    
 *    qHeader:
 *      a pointer to the queue header.
 *  
 *  Mac OS X threading:
 *    Thread safe
<<<<<<< HEAD
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function DTUninstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTUninstall';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetCurrentA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetCurrentA5() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function Dequeue( qElement: QElemPtr; qHeader: QHdrPtr ): OSErr; external name '_Dequeue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  MakeDataExecutable()
 *  
 *  Summary:
 *    Notifies the system that the specified data is subject to
 *    execution.
 *  
 *  Discussion:
 *    On some computer architectures it is necessary to tell the
 *    processor that an area of memory should be made executable.  This
 *    function does the necessary operations ( if possible ) to make it
 *    possible to execute code in the given address range.
 *    MakeDataExecutable is not supported for 64-bit applications. Use
 *    sys_icache_invalidate(3) and/or mprotect(2) as appropriate.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    baseAddress:
 *      the starting address to be made executable
 *    
 *    length:
 *      the length of the data pointed to by the baseAddress parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only]
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function SetCurrentA5: SIGNEDLONG; external name '_SetCurrentA5';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
procedure MakeDataExecutable( baseAddress: UnivPtr; length: UNSIGNEDLONG ); external name '_MakeDataExecutable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  SetA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetA5() on Mac OS X.
=======
function DTInstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTInstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DTUninstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe
=======
 *  ReadLocation()
=======
 *  InvokeDeferredTaskUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Obtains information about a geographic location or time zone.
 *  
 *  Discussion:
 *    The latitude and longitude are stored as Fract values, giving
 *    accuracy to within one foot. For example, a Fract value of 1.0
 *    equals 90 degrees –1.0 equals –90 degrees and –2.0 equals –180
 *    degrees.
 *    To convert these values to a degrees format, you need to convert
 *    the Fract values first to the Fixed data type, then to the
 *    LongInt data type. Use the Mathematical and Logical Utilities
 *    functions Fract2Fix and Fix2Long to accomplish this task.
 *    The DST value is a signed byte value that specifies the offset
 *    for the hour field—whether to add one hour, subtract one hour, or
 *    make no change at all.
 *    The GMT value is in seconds east of GMT. For example, San
 *    Francisco is at –28,800 seconds (8 hours * 3,600 seconds per
 *    hour) east of GMT. The gmtDelta field is a 3-byte value contained
 *    in a long word, so you must take care to get it properly.
 *    For more information on the Fract data type and the conversion
 *    routines Long2Fix, Fix2Fract, Fract2Fix, and Fix2Long, see
 *    Mathematical and Logical Utilities.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    loc:
 *      On return, the fields of the geographic location structure
 *      containing the geographic location and the time-zone
 *      information.
 *      You can get values for the latitude, longitude, daylight
 *      savings time (DST), or Greenwich mean time (GMT). If the
 *      geographic location record has never been set, all fields
 *      contain 0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure ReadLocation( var loc: MachineLocation ); external name '_ReadLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TickCount()
 *  
 *  Summary:
 *    Obtains the current number of ticks (a tick is approximately 1/60
 *    of a second) approximately since the system last started up.
 *  
 *  Discussion:
 *    The TickCount function returns an unsigned 32-bit integer that
 *    indicates the current number of ticks since the system last
 *    started up. You can use this value to compare the number of ticks
 *    that have elapsed since a given event or other action occurred.
 *    For example, you could compare the current value returned by
 *    TickCount with the value of the when field of an event
 *    structure.
 *    The tick count rolls over in approximately 2 years 3 months,
 *    which means you should not use this to time intervals which may
 *    exceed ( or even approach ) this interval.
 *    Do not rely on the tick count being exact; it is usually accurate
 *    to within one tick, but this level of accuracy is not guaranteed.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the tick count
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function SetA5( newA5: SIGNEDLONG ): SIGNEDLONG; external name '_SetA5';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)


{
 *  InitUtil()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    It is not necessary to call InitUtil on Mac OS X.  You should
 *    remove all calls to this from your code.
=======
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function DTUninstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTUninstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
function TickCount: UInt32; external name '_TickCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CSCopyUserName()
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the user name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    useShortName:
 *      A Boolean value that specifies whether to return the short name
 *      or full name of the user.
 *  
 *  Result:
 *    the requested name in a CFStringRef.  You should release this
 *    when you are done with it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function CSCopyUserName( useShortName: Boolean ): CFStringRef; external name '_CSCopyUserName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  CSCopyMachineName()
 *  
 *  Summary:
 *    Returns a reference to the CFString that represents the computer
 *    name.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    the name of this machine in a CFStringRef.  You should release
 *    this when you are done with it.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetCurrentA5: SIGNEDLONG; external name '_SetCurrentA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function CSCopyMachineName: CFStringRef; external name '_CSCopyMachineName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


const
	useFree = 0;
	useATalk = 1;
	useAsync = 2;
	useExtClk = 3;    {Externally clocked}
	useMIDI = 4;

const
	false32b = 0;    {24 bit addressing error}
	true32b = 1;     {32 bit addressing error}

type
	SysPPtr = UnivPtr;
>>>>>>> graemeg/cpstrnew

function GetMMUMode: SInt8; inline;
procedure SwapMMUMode( var mode: SInt8 ); inline;

{$ifc not TARGET_CPU_64}
{
 *  GetSysPPtr()   *** DEPRECATED ***
 *  
 *  Deprecated:
<<<<<<< HEAD
 *    You no longer need to use SetA5() on Mac OS X.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function InitUtil: OSErr; external name '_InitUtil';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)


=======
function InitUtil: OSErr; external name '_InitUtil';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


>>>>>>> graemeg/cpstrnew
{
 *  WriteParam()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function no longer does anything on Mac OS X; you should
 *    remove all calls to it from your code.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
<<<<<<< HEAD
=======
 *    Don't use this function; it always returns NULL on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetSysPPtr: SysPPtr; external name '_GetSysPPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
    NOTE: SysBeep() has been moved to Sound.h.  
 We could not automatically #include Sound.h in this file
 because Sound.h indirectly #include's OSUtils.h which
 would make a circular include.
 }
{$endc} {not TARGET_CPU_64}

type
	DeferredTaskProcPtr = procedure( dtParam: SIGNEDLONG );
	DeferredTaskUPP = DeferredTaskProcPtr;
{
 *  NewDeferredTaskUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function WriteParam: OSErr; external name '_WriteParam';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)


{
 *  WriteLocation()   *** DEPRECATED ***
=======
function NewDeferredTaskUPP( userRoutine: DeferredTaskProcPtr ): DeferredTaskUPP; external name '_NewDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDeferredTaskUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    WriteLocation can not be used to set the geographic information
 *    on Mac OS X.  You should remove all calls to this function from
 *    your code.
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetA5( newA5: SIGNEDLONG ): SIGNEDLONG; external name '_SetA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDeferredTaskUPP( userUPP: DeferredTaskUPP ); external name '_DisposeDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
<<<<<<< HEAD
 *  InitUtil()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    It is not necessary to call InitUtil on Mac OS X.  You should
 *    remove all calls to this from your code.
 *  
 *  Mac OS X threading:
 *    Thread safe
=======
 *  InvokeDeferredTaskUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InitUtil: OSErr; external name '_InitUtil';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
=======
=======
 *    Non-Carbon CFM:   available as macro/inline
 }
>>>>>>> origin/cpstrnew
procedure InvokeDeferredTaskUPP( dtParam: SIGNEDLONG; userUPP: DeferredTaskUPP ); external name '_InvokeDeferredTaskUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

type
	DeferredTask = record
		qLink: {volatile} QElemPtr;
		qType: SInt16;
		dtFlags: {volatile} SInt16;
		dtAddr: DeferredTaskUPP;
		dtParam: SIGNEDLONG;
		dtReserved: SIGNEDLONG;
	end;
	DeferredTaskPtr = ^DeferredTask;
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  WriteParam()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function no longer does anything on Mac OS X; you should
 *    remove all calls to it from your code.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure WriteLocation( const (*var*) loc: MachineLocation ); external name '_WriteLocation';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_1, __IPHONE_NA, __IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}
{$endc} {TARGET_OS_MAC}

=======
=======
 *  DTInstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DTInstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTInstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DTUninstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function DTUninstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTUninstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetCurrentA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetCurrentA5() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
>>>>>>> graemeg/cpstrnew
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function WriteParam: OSErr; external name '_WriteParam';
=======
function SetCurrentA5: SIGNEDLONG; external name '_SetCurrentA5';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

type
	DeferredTask = record
		qLink: {volatile} QElemPtr;
		qType: SInt16;
		dtFlags: {volatile} SInt16;
		dtAddr: DeferredTaskUPP;
		dtParam: SIGNEDLONG;
		dtReserved: SIGNEDLONG;
	end;
	DeferredTaskPtr = ^DeferredTask;
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  WriteLocation()   *** DEPRECATED ***
=======
 *  DTInstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
>>>>>>> origin/cpstrnew
 *  
 *  Deprecated:
 *    WriteLocation can not be used to set the geographic information
 *    on Mac OS X.  You should remove all calls to this function from
 *    your code.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.0
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DTInstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTInstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}
{$endc} {TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
 *  SetA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetA5() on Mac OS X.
=======
{
 *  DTUninstall()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Deferred Task Manager is deprecated.  Look into restructuring
 *    your code to use threads, or MPTasks, or some other threading
 *    solution.
 *  
 *  Summary:
 *    Adds the specified task record to the deferred-task queue.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetA5( newA5: SIGNEDLONG ): SIGNEDLONG; external name '_SetA5';
=======
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function DTUninstall( dtTaskPtr: DeferredTaskPtr ): OSErr; external name '_DTUninstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  SetCurrentA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetCurrentA5() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetCurrentA5: SIGNEDLONG; external name '_SetCurrentA5';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  InitUtil()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    It is not necessary to call InitUtil on Mac OS X.  You should
 *    remove all calls to this from your code.
=======
 *  SetA5()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    You no longer need to use SetA5() on Mac OS X.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function InitUtil: OSErr; external name '_InitUtil';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  WriteParam()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function no longer does anything on Mac OS X; you should
 *    remove all calls to it from your code.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function WriteParam: OSErr; external name '_WriteParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  WriteLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    WriteLocation can not be used to set the geographic information
 *    on Mac OS X.  You should remove all calls to this function from
 *    your code.
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetA5( newA5: SIGNEDLONG ): SIGNEDLONG; external name '_SetA5';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  InitUtil()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    It is not necessary to call InitUtil on Mac OS X.  You should
 *    remove all calls to this from your code.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function InitUtil: OSErr; external name '_InitUtil';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  WriteParam()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function no longer does anything on Mac OS X; you should
 *    remove all calls to it from your code.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure WriteLocation( const (*var*) loc: MachineLocation ); external name '_WriteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)

=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function WriteParam: OSErr; external name '_WriteParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  WriteLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    WriteLocation can not be used to set the geographic information
 *    on Mac OS X.  You should remove all calls to this function from
 *    your code.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure WriteLocation( const (*var*) loc: MachineLocation ); external name '_WriteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)

>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function WriteParam: OSErr; external name '_WriteParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  WriteLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    WriteLocation can not be used to set the geographic information
 *    on Mac OS X.  You should remove all calls to this function from
 *    your code.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure WriteLocation( const (*var*) loc: MachineLocation ); external name '_WriteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)

>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}
{$endc} {TARGET_OS_MAC}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

{$ifc TARGET_OS_MAC}

{$R-}

function GetMMUMode: SInt8; inline;
begin
	GetMMUMode:= true32b
end;

procedure SwapMMUMode( var mode: SInt8 ); inline;
begin
	mode := true32b;
end;


{$endc} {TARGET_OS_MAC}

end.

{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{
 *  WriteParam()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function no longer does anything on Mac OS X; you should
 *    remove all calls to it from your code.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function WriteParam: OSErr; external name '_WriteParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  WriteLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    WriteLocation can not be used to set the geographic information
 *    on Mac OS X.  You should remove all calls to this function from
 *    your code.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.0
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure WriteLocation( const (*var*) loc: MachineLocation ); external name '_WriteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED *)


{$endc} {not TARGET_CPU_64}
{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

{$ifc TARGET_OS_MAC}

{$R-}

function GetMMUMode: SInt8; inline;
begin
	GetMMUMode:= true32b
end;

procedure SwapMMUMode( var mode: SInt8 ); inline;
begin
	mode := true32b;
end;


{$endc} {TARGET_OS_MAC}

end.
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
