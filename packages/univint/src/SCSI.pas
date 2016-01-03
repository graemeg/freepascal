{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       OSServices/SCSI.h
 
     Contains:   SCSI Family Interfaces.
 
     Version:    OSServices-352~2
 
     Copyright:  © 1986-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       SCSI.p
=======
     File:       OSServices/SCSI.h
>>>>>>> origin/fixes_2.4
 
     Contains:   SCSI Family Interfaces.
 
     Version:    OSServices-352~2
 
<<<<<<< HEAD
     Copyright:  © 1986-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1986-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit SCSI;
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
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
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
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
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
uses MacTypes,MixedMode,AppleDiskPartitions;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC and not TARGET_CPU_64}

{$ALIGN MAC68K}

{ TIB opcodes }
const
	scInc = 1;
	scNoInc = 2;
	scAdd = 3;
	scMove = 4;
	scLoop = 5;
	scNop = 6;
	scStop = 7;
	scComp = 8;

{ TIB instruction }
type
	SCSIInstrPtr = ^SCSIInstr;
	SCSIInstr = record
		scOpcode: UInt16;
		scParam1: SIGNEDLONG;
		scParam2: SIGNEDLONG;
	end;
{
 *  SCSIReset()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIGet()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSISelect()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSICmd()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIRead()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIRBlind()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIWrite()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIWBlind()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIComplete()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIStat()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSISelAtn()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIMsgIn()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIMsgOut()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


const
	scsiVERSION = 43;


{
 * SCSI Completion routine callback for SCSIAction.
 }
type
	SCSICallbackProcPtr = procedure( scsiPB: UnivPtr );
	SCSICallbackUPP = SCSICallbackProcPtr;
{
 *  NewSCSICallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewSCSICallbackUPP( userRoutine: SCSICallbackProcPtr ): SCSICallbackUPP; external name '_NewSCSICallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)

{
 *  DisposeSCSICallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeSCSICallbackUPP( userUPP: SCSICallbackUPP ); external name '_DisposeSCSICallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC and not TARGET_CPU_64}

{$ALIGN MAC68K}

{ TIB opcodes }
const
	scInc = 1;
	scNoInc = 2;
	scAdd = 3;
	scMove = 4;
	scLoop = 5;
	scNop = 6;
	scStop = 7;
	scComp = 8;

{ TIB instruction }
type
	SCSIInstrPtr = ^SCSIInstr;
	SCSIInstr = record
		scOpcode: UInt16;
		scParam1: SIGNEDLONG;
		scParam2: SIGNEDLONG;
	end;
{
 *  SCSIReset()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIGet()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSISelect()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSICmd()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIRead()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIRBlind()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIWrite()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIWBlind()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIComplete()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIStat()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSISelAtn()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIMsgIn()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SCSIMsgOut()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


const
	scsiVERSION = 43;


{
 * SCSI Completion routine callback for SCSIAction.
 }
type
	SCSICallbackProcPtr = procedure( scsiPB: UnivPtr );
	SCSICallbackUPP = SCSICallbackProcPtr;
{
 *  NewSCSICallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewSCSICallbackUPP( userRoutine: SCSICallbackProcPtr ): SCSICallbackUPP; external name '_NewSCSICallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)

{
 *  DisposeSCSICallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeSCSICallbackUPP(userUPP: SCSICallbackUPP); external name '_DisposeSCSICallbackUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeSCSICallbackUPP( userUPP: SCSICallbackUPP ); external name '_DisposeSCSICallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeSCSICallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeSCSICallbackUPP( scsiPB: UnivPtr; userUPP: SCSICallbackUPP ); external name '_InvokeSCSICallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)

{ 
   SCSI Manager 4.3 function codes 
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
const
	SCSINop = $00; { Execute nothing                          }
	SCSIExecIO = $01; { Execute the specified IO                  }
	SCSIBusInquiry = $03; { Get parameters for entire path of HBAs           }
	SCSIReleaseQ = $04; { Release the frozen SIM queue for particular LUN      }
	SCSIAbortCommand = $10; { Abort the selected Control Block             }
	SCSIResetBus = $11; { Reset the SCSI bus                         }
	SCSIResetDevice = $12; { Reset the SCSI device                      }
	SCSITerminateIO = $13;  { Terminate any pending IO                    }
<<<<<<< HEAD
<<<<<<< HEAD

const
	vendorUnique = $C0;  { 0xC0 thru 0xFF }
=======
>>>>>>> graemeg/cpstrnew


{ Allocation length defines for some of the fields }
const
<<<<<<< HEAD
	handshakeDataLength = 8;    { Handshake data length }
	maxCDBLength = 16;   { Space for the CDB bytes/pointer }
	vendorIDLength = 16;    { ASCII string len for Vendor ID  }

=======
	vendorUnique = $C0;  { 0xC0 thru 0xFF }


{ Allocation length defines for some of the fields }
const
	handshakeDataLength = 8;    { Handshake data length }
	maxCDBLength = 16;   { Space for the CDB bytes/pointer }
	vendorIDLength = 16;    { ASCII string len for Vendor ID  }

>>>>>>> graemeg/cpstrnew
=======

const
	vendorUnique = $C0;  { 0xC0 thru 0xFF }


{ Allocation length defines for some of the fields }
const
	handshakeDataLength = 8;    { Handshake data length }
	maxCDBLength = 16;   { Space for the CDB bytes/pointer }
	vendorIDLength = 16;    { ASCII string len for Vendor ID  }

>>>>>>> origin/cpstrnew
{ Define DeviceIdent structure }
type
	DeviceIdentPtr = ^DeviceIdent;
	DeviceIdent = record
		diReserved: UInt8;             { reserved           }
		bus: UInt8;                    { SCSI - Bus Number   }
		targetID: UInt8;               { SCSI - Target SCSI ID  }
		LUN: UInt8;                    { SCSI - LUN            }
	end;
{ Constants for the diReserved field of DeviceIdent }
{ used to distinguish whether the DeviceIdent holds }
{ information about a SCSI device (kBusTypeSCSI)    }
{ or an ATA device (kBusTypeATA).  The other        }
{ constants are pretty much deprecated.  Let me     }
{ know if you see any.                              }
<<<<<<< HEAD
const
=======
const
	SCSINop = $00; { Execute nothing                          }
	SCSIExecIO = $01; { Execute the specified IO                  }
	SCSIBusInquiry = $03; { Get parameters for entire path of HBAs           }
	SCSIReleaseQ = $04; { Release the frozen SIM queue for particular LUN      }
	SCSIAbortCommand = $10; { Abort the selected Control Block             }
	SCSIResetBus = $11; { Reset the SCSI bus                         }
	SCSIResetDevice = $12; { Reset the SCSI device                      }
	SCSITerminateIO = $13;  { Terminate any pending IO                    }

const
	vendorUnique = $C0;  { 0xC0 thru 0xFF }


{ Allocation length defines for some of the fields }
const
	handshakeDataLength = 8;    { Handshake data length }
	maxCDBLength = 16;   { Space for the CDB bytes/pointer }
	vendorIDLength = 16;    { ASCII string len for Vendor ID  }

{ Define DeviceIdent structure }
type
	DeviceIdentPtr = ^DeviceIdent;
	DeviceIdent = record
		diReserved: UInt8;             { reserved           }
		bus: UInt8;                    { SCSI - Bus Number   }
		targetID: UInt8;               { SCSI - Target SCSI ID  }
		LUN: UInt8;                    { SCSI - LUN            }
	end;
{ Constants for the diReserved field of DeviceIdent }
{ used to distinguish whether the DeviceIdent holds }
{ information about a SCSI device (kBusTypeSCSI)    }
{ or an ATA device (kBusTypeATA).  The other        }
{ constants are pretty much deprecated.  Let me     }
{ know if you see any.                              }
const
>>>>>>> graemeg/cpstrnew
=======
const
>>>>>>> origin/cpstrnew
	kBusTypeSCSI = 0;
	kBusTypeATA = 1;
	kBusTypePCMCIA = 2;
	kBusTypeMediaBay = 3;

{ If diReserved indicates that a DeviceIdent is     }
{ really for ATA, you can cast it to DeviceIdentATA }
{ to get at the important fields.                   }
type
	DeviceIdentATAPtr = ^DeviceIdentATA;
	DeviceIdentATA = record
		diReserved: UInt8;
		busNum: UInt8;
		devNum: UInt8;
		diReserved2: UInt8;
	end;
<<<<<<< HEAD
<<<<<<< HEAD


{ Command Descriptor Block structure }
type
=======
const
	SCSINop = $00; { Execute nothing                          }
	SCSIExecIO = $01; { Execute the specified IO                  }
	SCSIBusInquiry = $03; { Get parameters for entire path of HBAs           }
	SCSIReleaseQ = $04; { Release the frozen SIM queue for particular LUN      }
	SCSIAbortCommand = $10; { Abort the selected Control Block             }
	SCSIResetBus = $11; { Reset the SCSI bus                         }
	SCSIResetDevice = $12; { Reset the SCSI device                      }
	SCSITerminateIO = $13;  { Terminate any pending IO                    }

const
	vendorUnique = $C0;  { 0xC0 thru 0xFF }


{ Allocation length defines for some of the fields }
const
	handshakeDataLength = 8;    { Handshake data length }
	maxCDBLength = 16;   { Space for the CDB bytes/pointer }
	vendorIDLength = 16;    { ASCII string len for Vendor ID  }

{ Define DeviceIdent structure }
type
	DeviceIdentPtr = ^DeviceIdent;
	DeviceIdent = record
		diReserved: UInt8;             { reserved           }
		bus: UInt8;                    { SCSI - Bus Number   }
		targetID: UInt8;               { SCSI - Target SCSI ID  }
		LUN: UInt8;                    { SCSI - LUN            }
	end;
{ Constants for the diReserved field of DeviceIdent }
{ used to distinguish whether the DeviceIdent holds }
{ information about a SCSI device (kBusTypeSCSI)    }
{ or an ATA device (kBusTypeATA).  The other        }
{ constants are pretty much deprecated.  Let me     }
{ know if you see any.                              }
const
	kBusTypeSCSI = 0;
	kBusTypeATA = 1;
	kBusTypePCMCIA = 2;
	kBusTypeMediaBay = 3;

{ If diReserved indicates that a DeviceIdent is     }
{ really for ATA, you can cast it to DeviceIdentATA }
{ to get at the important fields.                   }
type
	DeviceIdentATAPtr = ^DeviceIdentATA;
	DeviceIdentATA = record
		diReserved: UInt8;
		busNum: UInt8;
		devNum: UInt8;
		diReserved2: UInt8;
	end;


{ Command Descriptor Block structure }
type
>>>>>>> graemeg/cpstrnew
=======


{ Command Descriptor Block structure }
type
>>>>>>> graemeg/cpstrnew
=======


{ Command Descriptor Block structure }
type
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeSCSICallbackUPP( scsiPB: UnivPtr; userUPP: SCSICallbackUPP ); external name '_InvokeSCSICallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)

{ 
   SCSI Manager 4.3 function codes 
 }
const
	SCSINop = $00; { Execute nothing                          }
	SCSIExecIO = $01; { Execute the specified IO                  }
	SCSIBusInquiry = $03; { Get parameters for entire path of HBAs           }
	SCSIReleaseQ = $04; { Release the frozen SIM queue for particular LUN      }
	SCSIAbortCommand = $10; { Abort the selected Control Block             }
	SCSIResetBus = $11; { Reset the SCSI bus                         }
	SCSIResetDevice = $12; { Reset the SCSI device                      }
	SCSITerminateIO = $13;  { Terminate any pending IO                    }

const
	vendorUnique = $C0;  { 0xC0 thru 0xFF }


{ Allocation length defines for some of the fields }
const
	handshakeDataLength = 8;    { Handshake data length }
	maxCDBLength = 16;   { Space for the CDB bytes/pointer }
	vendorIDLength = 16;    { ASCII string len for Vendor ID  }

{ Define DeviceIdent structure }
type
	DeviceIdentPtr = ^DeviceIdent;
	DeviceIdent = record
		diReserved: UInt8;             { reserved           }
		bus: UInt8;                    { SCSI - Bus Number   }
		targetID: UInt8;               { SCSI - Target SCSI ID  }
		LUN: UInt8;                    { SCSI - LUN            }
	end;
{ Constants for the diReserved field of DeviceIdent }
{ used to distinguish whether the DeviceIdent holds }
{ information about a SCSI device (kBusTypeSCSI)    }
{ or an ATA device (kBusTypeATA).  The other        }
{ constants are pretty much deprecated.  Let me     }
{ know if you see any.                              }
const
	kBusTypeSCSI = 0;
	kBusTypeATA = 1;
	kBusTypePCMCIA = 2;
	kBusTypeMediaBay = 3;

{ If diReserved indicates that a DeviceIdent is     }
{ really for ATA, you can cast it to DeviceIdentATA }
{ to get at the important fields.                   }
type
	DeviceIdentATAPtr = ^DeviceIdentATA;
	DeviceIdentATA = record
		diReserved: UInt8;
		busNum: UInt8;
		devNum: UInt8;
		diReserved2: UInt8;
	end;


{ Command Descriptor Block structure }
type
<<<<<<< HEAD
	PatchListPtr = ^PatchList;
	PatchList = record
		numPatchBlocks:			UInt16;									{  The number of disk blocks         }
																		{  to hold patch descriptions        }
		numPatches:				UInt16;									{  The number of patches             }
		thePatch:				array [0..0] of PatchDescriptor;		{  An array with one patch           }
																		{  per element                       }
	end;

	{  signature of a Patch entry point }
{$ifc TYPED_FUNCTION_POINTERS}
	PatchEntryPoint = function(var myPatch: PatchDescriptor; myDevID: DeviceIdent): OSErr;
{$elsec}
	PatchEntryPoint = ProcPtr;
{$endc}


	{	 Command Descriptor Block structure 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	CDBPtr = ^CDB;
	CDB = record
		case SInt16 of
		0: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			cdbPtr: BytePtr;                 { pointer to the CDB, or }
			);
		1: (
			cdbBytes: packed array [0..15] of UInt8;           { the actual CDB to send }
			);
	end;

	{ Scatter/gather list element (Deprecated for MacOS8) }

type
	SGRecordPtr = ^SGRecord;
	SGRecord = record
		SGAddr: Ptr;
		SGCount: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2
			cdbPtr:				BytePtr;								{  pointer to the CDB, or  }
=======
			cdbPtr: BytePtr;                 { pointer to the CDB, or }
>>>>>>> origin/fixes_2.4
			);
		1: (
			cdbBytes: packed array [0..15] of UInt8;           { the actual CDB to send }
			);
	end;

	{ Scatter/gather list element (Deprecated for MacOS8) }

type
	SGRecordPtr = ^SGRecord;
	SGRecord = record
<<<<<<< HEAD
		SGAddr:					Ptr;
		SGCount:				UInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		SGAddr: Ptr;
		SGCount: UInt32;
>>>>>>> origin/fixes_2.4
	end;

	SCSIHdrPtr = ^SCSIHdr;
	SCSIHdr = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		qLink: SCSIHdrPtr;								{  (internal use, must be nil on entry)    }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
<<<<<<< HEAD
	end;
type
	SCSI_PBPtr = ^SCSI_PB;
	SCSI_PB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
	end;
type
	SCSI_IOPtr = ^SCSI_IO;
	SCSI_IO = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }

		scsiResultFlags: UInt16;        { <- Flags which modify the scsiResult field     }
		scsiReserved3pt5: UInt16;       { -> Reserved                           }
		scsiDataPtr: BytePtr;            { -> Pointer to the data buffer or the S/G list      }
		scsiDataLength: UInt32;         { -> Data transfer length                   }
		scsiSensePtr: BytePtr;           { -> Ptr to autosense data buffer            }
		scsiSenseLength: UInt8;        { -> size of the autosense buffer              }
		scsiCDBLength: UInt8;          { -> Number of bytes for the CDB               }
		scsiSGListCount: UInt16;        { -> num of scatter gather list entries           }
		scsiReserved4: UInt32;          { <-     reserved for output                   }
		scsiSCSIstatus: UInt8;         { <- Returned scsi device status               }
		scsiSenseResidual: SInt8;      { <- Autosense residual length             }
		scsiReserved5: UInt16;          { <-     reserved for output                 }
		scsiDataResidual: SIGNEDLONG;       { <- Returned Transfer residual length          }
		scsiCDB: CDB;                { -> Actual CDB or pointer to CDB            }
		scsiTimeout: SIGNEDLONG;            { -> Timeout value (Time Mgr format) (CAM timeout) }
		scsiReserved5pt5: BytePtr;       { -> Reserved                           }
		scsiReserved5pt6: UInt16;       { -> Reserved                           }
		scsiIOFlags: UInt16;            { -> additional I/O flags                      }
		scsiTagAction: UInt8;          { -> What to do for tag queuing                }
		scsiReserved6: UInt8;          { ->     reserved for input                   }
		scsiReserved7: UInt16;          { ->     reserved for input                   }
		scsiSelectTimeout: UInt16;      { -> Select timeout value                 }
		scsiDataType: UInt8;           { -> Data description type (i.e. buffer, TIB, S/G)   }
		scsiTransferType: UInt8;       { -> Transfer type (i.e. Blind vs Polled)       }
		scsiReserved8: UInt32;          { ->     reserved for input                  }
		scsiReserved9: UInt32;          { ->     reserved for input                  }
		scsiHandshake: array [0..7] of UInt16;       { -> handshaking points (null term'd)    }
		scsiReserved10: UInt32;         { ->     reserved for input                  }
		scsiReserved11: UInt32;         { ->   reserved for input                   }
		scsiCommandLink: SCSI_IOPtr;								{ -> Ptr to the next PB in linked cmd chain      }
		scsiSIMpublics: packed array [0..7] of UInt8;      { ->     reserved for input to 3rd-party SIMs     }
		scsiAppleReserved6: packed array [0..7] of UInt8;  { -> reserved for input                     }

                                              { XPT layer privates (for old-API emulation) }

		scsiCurrentPhase: UInt16;       { <- phase upon completing old call            }
		scsiSelector: SInt16;           { -> selector specified in old calls            }
		scsiOldCallResult: OSErr;      { <- result of old call                     }
		scsiSCSImessage: UInt8;        { <- Returned scsi device message (for SCSIComplete)}
		XPTprivateFlags: UInt8;        { <> various flags                       }
		XPTextras: packed array [0..11] of UInt8;          {                              }
=======
=======
>>>>>>> origin/fixes_2_2
		qLink:					SCSIHdrPtr;								{  (internal use, must be nil on entry)    }
		scsiReserved1:			SInt16;								{  ->     reserved for input           }
		scsiPBLength:			UInt16;									{  -> Length of the entire PB         }
		scsiFunctionCode:		SInt8;									{  -> function selector            }
		scsiReserved2:			SInt8;									{  <-     reserved for output           }
		scsiResult:				OSErr;									{  <- Returned result                }
		scsiDevice:				DeviceIdent;							{  -> Device Identifier (bus+target+lun) }
		scsiCompletion:			SCSICallbackUPP;						{  -> Callback on completion function      }
		scsiFlags:				UInt32;									{  -> assorted flags             }
		scsiDriverStorage:		BytePtr;								{  <> Ptr for driver private use    }
		scsiXPTprivate:			Ptr;									{  private field for use in XPT       }
		scsiReserved3:			SInt32;								{  reserved                     }
=======
>>>>>>> origin/fixes_2.4
	end;
type
	SCSI_PBPtr = ^SCSI_PB;
	SCSI_PB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
	end;
type
	SCSI_IOPtr = ^SCSI_IO;
	SCSI_IO = record
<<<<<<< HEAD
		qLink:					SCSIHdrPtr;								{  (internal use, must be nil on entry)    }
		scsiReserved1:			SInt16;								{  ->     reserved for input           }
		scsiPBLength:			UInt16;									{  -> Length of the entire PB         }
		scsiFunctionCode:		SInt8;									{  -> function selector            }
		scsiReserved2:			SInt8;									{  <-     reserved for output           }
		scsiResult:				OSErr;									{  <- Returned result                }
		scsiDevice:				DeviceIdent;							{  -> Device Identifier (bus+target+lun) }
		scsiCompletion:			SCSICallbackUPP;						{  -> Callback on completion function      }
		scsiFlags:				UInt32;									{  -> assorted flags             }
		scsiDriverStorage:		BytePtr;								{  <> Ptr for driver private use    }
		scsiXPTprivate:			Ptr;									{  private field for use in XPT       }
		scsiReserved3:			SInt32;								{  reserved                     }
		scsiResultFlags:		UInt16;									{  <- Flags which modify the scsiResult field      }
		scsiReserved3pt5:		UInt16;									{  -> Reserved                            }
		scsiDataPtr:			BytePtr;								{  -> Pointer to the data buffer or the S/G list       }
		scsiDataLength:			UInt32;									{  -> Data transfer length                    }
		scsiSensePtr:			BytePtr;								{  -> Ptr to autosense data buffer             }
		scsiSenseLength:		SInt8;									{  -> size of the autosense buffer               }
		scsiCDBLength:			SInt8;									{  -> Number of bytes for the CDB                }
		scsiSGListCount:		UInt16;									{  -> num of scatter gather list entries            }
		scsiReserved4:			UInt32;									{  <-     reserved for output                    }
		scsiSCSIstatus:			SInt8;									{  <- Returned scsi device status                }
		scsiSenseResidual:		SInt8;									{  <- Autosense residual length              }
		scsiReserved5:			UInt16;									{  <-     reserved for output                  }
		scsiDataResidual:		SInt32;								{  <- Returned Transfer residual length           }
		scsiCDB:				CDB;									{  -> Actual CDB or pointer to CDB             }
		scsiTimeout:			SInt32;								{  -> Timeout value (Time Mgr format) (CAM timeout)  }
		scsiReserved5pt5:		BytePtr;								{  -> Reserved                            }
		scsiReserved5pt6:		UInt16;									{  -> Reserved                            }
		scsiIOFlags:			UInt16;									{  -> additional I/O flags                       }
		scsiTagAction:			SInt8;									{  -> What to do for tag queuing                 }
		scsiReserved6:			SInt8;									{  ->     reserved for input                    }
		scsiReserved7:			UInt16;									{  ->     reserved for input                    }
		scsiSelectTimeout:		UInt16;									{  -> Select timeout value                  }
		scsiDataType:			SInt8;									{  -> Data description type (i.e. buffer, TIB, S/G)    }
		scsiTransferType:		SInt8;									{  -> Transfer type (i.e. Blind vs Polled)        }
		scsiReserved8:			UInt32;									{  ->     reserved for input                   }
		scsiReserved9:			UInt32;									{  ->     reserved for input                   }
		scsiHandshake:			array [0..7] of UInt16;					{  -> handshaking points (null term'd)     }
		scsiReserved10:			UInt32;									{  ->     reserved for input                   }
		scsiReserved11:			UInt32;									{  ->   reserved for input                    }
		scsiCommandLink:		SCSI_IOPtr;								{  -> Ptr to the next PB in linked cmd chain       }
		scsiSIMpublics:			packed array [0..7] of UInt8;			{  ->     reserved for input to 3rd-party SIMs      }
		scsiAppleReserved6:		packed array [0..7] of UInt8;			{  -> reserved for input                      }
																		{  XPT layer privates (for old-API emulation)  }
		scsiCurrentPhase:		UInt16;									{  <- phase upon completing old call             }
		scsiSelector:			SInt16;								{  -> selector specified in old calls             }
		scsiOldCallResult:		OSErr;									{  <- result of old call                      }
		scsiSCSImessage:		SInt8;									{  <- Returned scsi device message (for SCSIComplete) }
		XPTprivateFlags:		SInt8;									{  <> various flags                        }
		XPTextras:				packed array [0..11] of UInt8;			{                                }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }

		scsiResultFlags: UInt16;        { <- Flags which modify the scsiResult field     }
		scsiReserved3pt5: UInt16;       { -> Reserved                           }
		scsiDataPtr: BytePtr;            { -> Pointer to the data buffer or the S/G list      }
		scsiDataLength: UInt32;         { -> Data transfer length                   }
		scsiSensePtr: BytePtr;           { -> Ptr to autosense data buffer            }
		scsiSenseLength: UInt8;        { -> size of the autosense buffer              }
		scsiCDBLength: UInt8;          { -> Number of bytes for the CDB               }
		scsiSGListCount: UInt16;        { -> num of scatter gather list entries           }
		scsiReserved4: UInt32;          { <-     reserved for output                   }
		scsiSCSIstatus: UInt8;         { <- Returned scsi device status               }
		scsiSenseResidual: SInt8;      { <- Autosense residual length             }
		scsiReserved5: UInt16;          { <-     reserved for output                 }
		scsiDataResidual: SIGNEDLONG;       { <- Returned Transfer residual length          }
		scsiCDB: CDB;                { -> Actual CDB or pointer to CDB            }
		scsiTimeout: SIGNEDLONG;            { -> Timeout value (Time Mgr format) (CAM timeout) }
		scsiReserved5pt5: BytePtr;       { -> Reserved                           }
		scsiReserved5pt6: UInt16;       { -> Reserved                           }
		scsiIOFlags: UInt16;            { -> additional I/O flags                      }
		scsiTagAction: UInt8;          { -> What to do for tag queuing                }
		scsiReserved6: UInt8;          { ->     reserved for input                   }
		scsiReserved7: UInt16;          { ->     reserved for input                   }
		scsiSelectTimeout: UInt16;      { -> Select timeout value                 }
		scsiDataType: UInt8;           { -> Data description type (i.e. buffer, TIB, S/G)   }
		scsiTransferType: UInt8;       { -> Transfer type (i.e. Blind vs Polled)       }
		scsiReserved8: UInt32;          { ->     reserved for input                  }
		scsiReserved9: UInt32;          { ->     reserved for input                  }
		scsiHandshake: array [0..7] of UInt16;       { -> handshaking points (null term'd)    }
		scsiReserved10: UInt32;         { ->     reserved for input                  }
		scsiReserved11: UInt32;         { ->   reserved for input                   }
		scsiCommandLink: SCSI_IOPtr;								{ -> Ptr to the next PB in linked cmd chain      }
		scsiSIMpublics: packed array [0..7] of UInt8;      { ->     reserved for input to 3rd-party SIMs     }
		scsiAppleReserved6: packed array [0..7] of UInt8;  { -> reserved for input                     }

                                              { XPT layer privates (for old-API emulation) }

		scsiCurrentPhase: UInt16;       { <- phase upon completing old call            }
		scsiSelector: SInt16;           { -> selector specified in old calls            }
		scsiOldCallResult: OSErr;      { <- result of old call                     }
		scsiSCSImessage: UInt8;        { <- Returned scsi device message (for SCSIComplete)}
		XPTprivateFlags: UInt8;        { <> various flags                       }
		XPTextras: packed array [0..11] of UInt8;          {                              }
>>>>>>> origin/fixes_2.4
	end;

	SCSIExecIOPB						= SCSI_IO;
	SCSIExecIOPBPtr 					= ^SCSIExecIOPB;
	{	 Bus inquiry PB 	}
	SCSIBusInquiryPBPtr = ^SCSIBusInquiryPB;
	SCSIBusInquiryPB = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }

		scsiEngineCount: UInt16;        { <- Number of engines on HBA                }
		scsiMaxTransferType: UInt16;    { <- Number of transfer types for this HBA      }

		scsiDataTypes: UInt32;          { <- which data types are supported by this SIM  }

		scsiIOpbSize: UInt16;           { <- Size of SCSI_IO PB for this SIM/HBA          }
		scsiMaxIOpbSize: UInt16;        { <- Size of max SCSI_IO PB for all SIM/HBAs        }

		scsiFeatureFlags: UInt32;       { <- Supported features flags field           }

		scsiVersionNumber: UInt8;      { <- Version number for the SIM/HBA           }
		scsiHBAInquiry: UInt8;         { <- Mimic of INQ byte 7 for the HBA           }
		scsiTargetModeFlags: UInt8;    { <- Flags for target mode support           }
		scsiScanFlags: UInt8;          { <- Scan related feature flags            }

		scsiSIMPrivatesPtr: UInt32;     { <- Ptr to SIM private data area              }
		scsiSIMPrivatesSize: UInt32;    { <- Size of SIM private data area           }
		scsiAsyncFlags: UInt32;         { <- Event cap. for Async Callback           }

		scsiHiBusID: UInt8;            { <- Highest path ID in the subsystem         }
		scsiInitiatorID: UInt8;        { <- ID of the HBA on the SCSI bus           }
		scsiBIReserved0: UInt16;        {                                  }

		scsiBIReserved1: UInt32;        { <-                                }
		scsiFlagsSupported: UInt32;     { <- which scsiFlags are supported           }

		scsiIOFlagsSupported: UInt16;   { <- which scsiIOFlags are supported           }
		scsiWeirdStuff: UInt16;         { <-                               }
		scsiMaxTarget: UInt16;          { <- maximum Target number supported           }
		scsiMaxLUN: UInt16;             { <- maximum Logical Unit number supported    }

		scsiSIMVendor: packed array [0..15] of char;			{ <- Vendor ID of SIM (or XPT if bus<FF)        }
		scsiHBAVendor: packed array [0..15] of char;			{ <- Vendor ID of the HBA                }
		scsiControllerFamily:	packed array [0..15] of char;			{ <- Family of SCSI Controller           }
		scsiControllerType: packed array [0..15] of char;			{ <- Specific Model of SCSI Controller used  }

		scsiXPTversion: packed array [0..3] of char;			{ <- version number of XPT              }
		scsiSIMversion: packed array [0..3] of char;			{ <- version number of SIM              }
		scsiHBAversion: packed array [0..3] of char;			{ <- version number of HBA              }

		scsiHBAslotType: UInt8;        { <- type of "slot" that this HBA is in       }
		scsiHBAslotNumber: UInt8;      { <- slot number of this HBA                  }
		scsiSIMsRsrcID: UInt16;         { <- resource ID of this SIM                  }

		scsiBIReserved3: UInt16;        { <-                               }
		scsiAdditionalLength: UInt16;   { <- additional BusInquiry PB len              }
<<<<<<< HEAD
	end;
{ Abort SIM Request PB }
type
	SCSIAbortCommandPBPtr = ^SCSIAbortCommandPB;
	SCSIAbortCommandPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiIOptr: SCSI_IOPtr;              { Pointer to the PB to abort        }
	end;
{ Terminate I/O Process Request PB }
type
	SCSITerminateIOPBPtr = ^SCSITerminateIOPB;
	SCSITerminateIOPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiIOptr: SCSI_IOPtr;              { Pointer to the PB to terminate        }
	end;
{ Reset SCSI Bus PB }
type
	SCSIResetBusPBPtr = ^SCSIResetBusPB;
	SCSIResetBusPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
	end;
{ Reset SCSI Device PB }
type
	SCSIResetDevicePBPtr = ^SCSIResetDevicePB;
	SCSIResetDevicePB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
	end;
{ Release SIM Queue PB }
type
	SCSIReleaseQPBPtr = ^SCSIReleaseQPB;
	SCSIReleaseQPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
	end;
{ SCSI Get Virtual ID Info PB }
type
	SCSIGetVirtualIDInfoPBPtr = ^SCSIGetVirtualIDInfoPB;
	SCSIGetVirtualIDInfoPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: Ptr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiOldCallID: UInt16;          { -> SCSI ID of device in question   }
		scsiExists: Boolean;             { <- true if device exists        }
		filler: SInt8;
	end;
{ Create/Lookup/Remove RefNum for Device PB }
type
	SCSIDriverPBPtr = ^SCSIDriverPB;
	SCSIDriverPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: Ptr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiDriver: SInt16;             { -> DriverRefNum, For SetDriver, <- For GetNextDriver }
		scsiDriverFlags: UInt16;        { <> Details of driver/device       }
		scsiNextDevice: DeviceIdent;         { <- DeviceIdent of the NEXT Item in the list  }
	end;
{ Load Driver PB }
type
	SCSILoadDriverPBPtr = ^SCSILoadDriverPB;
	SCSILoadDriverPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: Ptr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiLoadedRefNum: SInt16;       { <- SIM returns refnum of driver      }
		scsiDiskLoadFailed: Boolean;     { -> if true, indicates call after failure to load }
		filler: SInt8;
	end;

{ Defines for the scsiTransferType field }
const
	scsiTransferBlind = 0;
	scsiTransferPolled = 1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

const
	scsiErrorBase = -7936;

const
	scsiRequestInProgress = 1;    { 1   = PB request is in progress          }
                                        { Execution failed  00-2F }
	scsiRequestAborted = scsiErrorBase + 2; { -7934 = PB request aborted by the host        }
	scsiUnableToAbort = scsiErrorBase + 3; { -7933 = Unable to Abort PB request          }
	scsiNonZeroStatus = scsiErrorBase + 4; { -7932 = PB request completed with an err   }
	scsiUnused05 = scsiErrorBase + 5; { -7931 =                        }
	scsiUnused06 = scsiErrorBase + 6; { -7930 =                        }
	scsiUnused07 = scsiErrorBase + 7; { -7929 =                        }
	scsiUnused08 = scsiErrorBase + 8; { -7928 =                        }
	scsiUnableToTerminate = scsiErrorBase + 9; { -7927 = Unable to Terminate I/O PB req        }
	scsiSelectTimeout = scsiErrorBase + 10; { -7926 = Target selection timeout        }
	scsiCommandTimeout = scsiErrorBase + 11; { -7925 = Command timeout              }
	scsiIdentifyMessageRejected = scsiErrorBase + 12; { -7924 =                        }
	scsiMessageRejectReceived = scsiErrorBase + 13; { -7923 = Message reject received           }
	scsiSCSIBusReset = scsiErrorBase + 14; { -7922 = SCSI bus reset sent/received    }
	scsiParityError = scsiErrorBase + 15; { -7921 = Uncorrectable parity error occured     }
	scsiAutosenseFailed = scsiErrorBase + 16; { -7920 = Autosense: Request sense cmd fail  }
	scsiUnused11 = scsiErrorBase + 17; { -7919 =                        }
	scsiDataRunError = scsiErrorBase + 18; { -7918 = Data overrun/underrun error     }
	scsiUnexpectedBusFree = scsiErrorBase + 19; { -7917 = Unexpected BUS free              }
	scsiSequenceFailed = scsiErrorBase + 20; { -7916 = Target bus phase sequence failure  }
	scsiWrongDirection = scsiErrorBase + 21; { -7915 = Data phase was in wrong direction  }
	scsiUnused16 = scsiErrorBase + 22; { -7914 =                        }
	scsiBDRsent = scsiErrorBase + 23; { -7913 = A SCSI BDR msg was sent to target  }
	scsiTerminated = scsiErrorBase + 24; { -7912 = PB request terminated by the host  }
	scsiNoNexus = scsiErrorBase + 25; { -7911 = Nexus is not established        }
	scsiCDBReceived = scsiErrorBase + 26; { -7910 = The SCSI CDB has been received        }
                                        { Couldn't begin execution  30-3F }
	scsiTooManyBuses = scsiErrorBase + 48; { -7888 = Register failed because we're full }
	scsiBusy = scsiErrorBase + 49; { -7887 = SCSI subsystem is busy           }
	scsiProvideFail = scsiErrorBase + 50; { -7886 = Unable to provide requ. capability }
	scsiDeviceNotThere = scsiErrorBase + 51; { -7885 = SCSI device not installed/there    }
	scsiNoHBA = scsiErrorBase + 52; { -7884 = No HBA detected Error           }
	scsiDeviceConflict = scsiErrorBase + 53; { -7883 = sorry, max 1 refNum per DeviceIdent    }
	scsiNoSuchXref = scsiErrorBase + 54; { -7882 = no such RefNum xref              }
	scsiQLinkInvalid = scsiErrorBase + 55; { -7881 = pre-linked PBs not supported      }
                                        {   (The QLink field was nonzero)          }
                                        { Parameter errors  40-7F }
	scsiPBLengthError = scsiErrorBase + 64; { -7872 = (scsiPBLength is insuf'ct/invalid  }
	scsiFunctionNotAvailable = scsiErrorBase + 65; { -7871 = The requ. func is not available    }
	scsiRequestInvalid = scsiErrorBase + 66; { -7870 = PB request is invalid           }
	scsiBusInvalid = scsiErrorBase + 67; { -7869 = Bus ID supplied is invalid        }
	scsiTIDInvalid = scsiErrorBase + 68; { -7868 = Target ID supplied is invalid      }
	scsiLUNInvalid = scsiErrorBase + 69; { -7867 = LUN supplied is invalid         }
	scsiIDInvalid = scsiErrorBase + 70; { -7866 = The initiator ID is invalid     }
	scsiDataTypeInvalid = scsiErrorBase + 71; { -7865 = scsiDataType requested not supported }
	scsiTransferTypeInvalid = scsiErrorBase + 72; { -7864 = scsiTransferType field is too high     }
	scsiCDBLengthInvalid = scsiErrorBase + 73; { -7863 = scsiCDBLength field is too big        }

{ New errors for SCSI Family         }
const
	scsiUnused74 = scsiErrorBase + 74; { -7862 =                          }
	scsiUnused75 = scsiErrorBase + 75; { -7861 =                          }
	scsiBadDataLength = scsiErrorBase + 76; { -7860 = a zero data length in PB        }
	scsiPartialPrepared = scsiErrorBase + 77; { -7859 = could not do full prepare mem for I/O}
	scsiInvalidMsgType = scsiErrorBase + 78; { -7858 = Invalid message type (internal)       }
	scsiUnused79 = scsiErrorBase + 79; { -7857 =                              }
	scsiBadConnID = scsiErrorBase + 80; { -7856 = Bad Connection ID                }
	scsiUnused81 = scsiErrorBase + 81; { -7855 =                          }
	scsiIOInProgress = scsiErrorBase + 82; { -7854 = Can't close conn, IO in prog      }
	scsiTargetReserved = scsiErrorBase + 83; { -7853 = Target already reserved          }
	scsiUnused84 = scsiErrorBase + 84; { -7852 =                          }
	scsiUnused85 = scsiErrorBase + 85; { -7851 =                          }
	scsiBadConnType = scsiErrorBase + 86; { -7850 = Bad connection type              }
	scsiCannotLoadPlugin = scsiErrorBase + 87; { -7849 = No matching service category      }

{ +++ }
{
 * scsiFamilyInternalError and scsiPluginInternalError are intended to handle consistency check failures.
 * For example, if the family stores a record on a lookaside queue, but does not find that record
 * it can use this error to report this failure. SCSI Manager 4.3 uses dsIOCoreErr in a few places,
 * but this is probably not the best error. In general, internal errors should be reported as bugs.
 *
 * The following range of errors is provided for third-party (non-Apple) SCSI SIM and device driver vendors.
 * In general, they would be used for error conditions that are not covered by the standardized errors.
 * They should not normally be conveyed to normal applications, but might be used for communication between
 * a plug-in and a vendor-provided device driver (for example, to manage RAID hot-swapping).
 *
 * Note: I don't know how many SCSI errors are reserved in the error code architecture. Don't assume that
 * we'll actually get sixteen, but we should reserve at least one.
 }
const
	scsiFamilyInternalError = scsiErrorBase + 87; { -7849 = Internal consistency check failed  }
	scsiPluginInternalError = scsiErrorBase + 88; { -7848 = Internal consistency check failed  }
	scsiVendorSpecificErrorBase = scsiErrorBase + 128; { ??    = Start of third-party error range     }
	scsiVendorSpecificErrorCount = 16;    { Number of third-party errors             }

=======

const
	scsiErrorBase = -7936;

const
	scsiRequestInProgress = 1;    { 1   = PB request is in progress          }
                                        { Execution failed  00-2F }
	scsiRequestAborted = scsiErrorBase + 2; { -7934 = PB request aborted by the host        }
	scsiUnableToAbort = scsiErrorBase + 3; { -7933 = Unable to Abort PB request          }
	scsiNonZeroStatus = scsiErrorBase + 4; { -7932 = PB request completed with an err   }
	scsiUnused05 = scsiErrorBase + 5; { -7931 =                        }
	scsiUnused06 = scsiErrorBase + 6; { -7930 =                        }
	scsiUnused07 = scsiErrorBase + 7; { -7929 =                        }
	scsiUnused08 = scsiErrorBase + 8; { -7928 =                        }
	scsiUnableToTerminate = scsiErrorBase + 9; { -7927 = Unable to Terminate I/O PB req        }
	scsiSelectTimeout = scsiErrorBase + 10; { -7926 = Target selection timeout        }
	scsiCommandTimeout = scsiErrorBase + 11; { -7925 = Command timeout              }
	scsiIdentifyMessageRejected = scsiErrorBase + 12; { -7924 =                        }
	scsiMessageRejectReceived = scsiErrorBase + 13; { -7923 = Message reject received           }
	scsiSCSIBusReset = scsiErrorBase + 14; { -7922 = SCSI bus reset sent/received    }
	scsiParityError = scsiErrorBase + 15; { -7921 = Uncorrectable parity error occured     }
	scsiAutosenseFailed = scsiErrorBase + 16; { -7920 = Autosense: Request sense cmd fail  }
	scsiUnused11 = scsiErrorBase + 17; { -7919 =                        }
	scsiDataRunError = scsiErrorBase + 18; { -7918 = Data overrun/underrun error     }
	scsiUnexpectedBusFree = scsiErrorBase + 19; { -7917 = Unexpected BUS free              }
	scsiSequenceFailed = scsiErrorBase + 20; { -7916 = Target bus phase sequence failure  }
	scsiWrongDirection = scsiErrorBase + 21; { -7915 = Data phase was in wrong direction  }
	scsiUnused16 = scsiErrorBase + 22; { -7914 =                        }
	scsiBDRsent = scsiErrorBase + 23; { -7913 = A SCSI BDR msg was sent to target  }
	scsiTerminated = scsiErrorBase + 24; { -7912 = PB request terminated by the host  }
	scsiNoNexus = scsiErrorBase + 25; { -7911 = Nexus is not established        }
	scsiCDBReceived = scsiErrorBase + 26; { -7910 = The SCSI CDB has been received        }
                                        { Couldn't begin execution  30-3F }
	scsiTooManyBuses = scsiErrorBase + 48; { -7888 = Register failed because we're full }
	scsiBusy = scsiErrorBase + 49; { -7887 = SCSI subsystem is busy           }
	scsiProvideFail = scsiErrorBase + 50; { -7886 = Unable to provide requ. capability }
	scsiDeviceNotThere = scsiErrorBase + 51; { -7885 = SCSI device not installed/there    }
	scsiNoHBA = scsiErrorBase + 52; { -7884 = No HBA detected Error           }
	scsiDeviceConflict = scsiErrorBase + 53; { -7883 = sorry, max 1 refNum per DeviceIdent    }
	scsiNoSuchXref = scsiErrorBase + 54; { -7882 = no such RefNum xref              }
	scsiQLinkInvalid = scsiErrorBase + 55; { -7881 = pre-linked PBs not supported      }
                                        {   (The QLink field was nonzero)          }
                                        { Parameter errors  40-7F }
	scsiPBLengthError = scsiErrorBase + 64; { -7872 = (scsiPBLength is insuf'ct/invalid  }
	scsiFunctionNotAvailable = scsiErrorBase + 65; { -7871 = The requ. func is not available    }
	scsiRequestInvalid = scsiErrorBase + 66; { -7870 = PB request is invalid           }
	scsiBusInvalid = scsiErrorBase + 67; { -7869 = Bus ID supplied is invalid        }
	scsiTIDInvalid = scsiErrorBase + 68; { -7868 = Target ID supplied is invalid      }
	scsiLUNInvalid = scsiErrorBase + 69; { -7867 = LUN supplied is invalid         }
	scsiIDInvalid = scsiErrorBase + 70; { -7866 = The initiator ID is invalid     }
	scsiDataTypeInvalid = scsiErrorBase + 71; { -7865 = scsiDataType requested not supported }
	scsiTransferTypeInvalid = scsiErrorBase + 72; { -7864 = scsiTransferType field is too high     }
	scsiCDBLengthInvalid = scsiErrorBase + 73; { -7863 = scsiCDBLength field is too big        }

{ New errors for SCSI Family         }
const
	scsiUnused74 = scsiErrorBase + 74; { -7862 =                          }
	scsiUnused75 = scsiErrorBase + 75; { -7861 =                          }
	scsiBadDataLength = scsiErrorBase + 76; { -7860 = a zero data length in PB        }
	scsiPartialPrepared = scsiErrorBase + 77; { -7859 = could not do full prepare mem for I/O}
	scsiInvalidMsgType = scsiErrorBase + 78; { -7858 = Invalid message type (internal)       }
	scsiUnused79 = scsiErrorBase + 79; { -7857 =                              }
	scsiBadConnID = scsiErrorBase + 80; { -7856 = Bad Connection ID                }
	scsiUnused81 = scsiErrorBase + 81; { -7855 =                          }
	scsiIOInProgress = scsiErrorBase + 82; { -7854 = Can't close conn, IO in prog      }
	scsiTargetReserved = scsiErrorBase + 83; { -7853 = Target already reserved          }
	scsiUnused84 = scsiErrorBase + 84; { -7852 =                          }
	scsiUnused85 = scsiErrorBase + 85; { -7851 =                          }
	scsiBadConnType = scsiErrorBase + 86; { -7850 = Bad connection type              }
	scsiCannotLoadPlugin = scsiErrorBase + 87; { -7849 = No matching service category      }

{ +++ }
{
 * scsiFamilyInternalError and scsiPluginInternalError are intended to handle consistency check failures.
 * For example, if the family stores a record on a lookaside queue, but does not find that record
 * it can use this error to report this failure. SCSI Manager 4.3 uses dsIOCoreErr in a few places,
 * but this is probably not the best error. In general, internal errors should be reported as bugs.
 *
 * The following range of errors is provided for third-party (non-Apple) SCSI SIM and device driver vendors.
 * In general, they would be used for error conditions that are not covered by the standardized errors.
 * They should not normally be conveyed to normal applications, but might be used for communication between
 * a plug-in and a vendor-provided device driver (for example, to manage RAID hot-swapping).
 *
 * Note: I don't know how many SCSI errors are reserved in the error code architecture. Don't assume that
 * we'll actually get sixteen, but we should reserve at least one.
 }
const
	scsiFamilyInternalError = scsiErrorBase + 87; { -7849 = Internal consistency check failed  }
	scsiPluginInternalError = scsiErrorBase + 88; { -7848 = Internal consistency check failed  }
	scsiVendorSpecificErrorBase = scsiErrorBase + 128; { ??    = Start of third-party error range     }
	scsiVendorSpecificErrorCount = 16;    { Number of third-party errors             }

>>>>>>> origin/cpstrnew
{ --- }
const
	scsiExecutionErrors = scsiErrorBase;
	scsiNotExecutedErrors = scsiTooManyBuses;
	scsiParameterErrors = scsiPBLengthError;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

const
	scsiErrorBase = -7936;
>>>>>>> graemeg/cpstrnew

{ Defines for the scsiResultFlags field }
const
<<<<<<< HEAD
	scsiSIMQFrozen = $0001; { The SIM queue is frozen w/this err        }
	scsiAutosenseValid = $0002; { Autosense data valid for target         }
	scsiBusNotFree = $0004; { At time of callback, SCSI bus is not free  }

{ Defines for the bit numbers of the scsiFlags field in the PB header for the SCSIExecIO function }
const
	kbSCSIDisableAutosense = 29;   { Disable auto sense feature               }
	kbSCSIFlagReservedA = 28;   {                             }
	kbSCSIFlagReserved0 = 27;   {                             }
	kbSCSICDBLinked = 26;   { The PB contains a linked CDB             }
	kbSCSIQEnable = 25;   { Target queue actions are enabled          }
	kbSCSICDBIsPointer = 24;   { The CDB field contains a pointer          }
	kbSCSIFlagReserved1 = 23;   {                               }
	kbSCSIInitiateSyncData = 22;   { Attempt Sync data xfer and SDTR          }
	kbSCSIDisableSyncData = 21;   { Disable sync, go to async            }
	kbSCSISIMQHead = 20;   { Place PB at the head of SIM Q          }
	kbSCSISIMQFreeze = 19;   { Return the SIM Q to frozen state          }
	kbSCSISIMQNoFreeze = 18;   { Disallow SIM Q freezing                 }
	kbSCSIDoDisconnect = 17;   { Definitely do disconnect               }
	kbSCSIDontDisconnect = 16;   { Definitely don't disconnect              }
	kbSCSIDataReadyForDMA = 15;   { Data buffer(s) are ready for DMA          }
	kbSCSIFlagReserved3 = 14;   {                               }
	kbSCSIDataPhysical = 13;   { SG/Buffer data ptrs are physical          }
	kbSCSISensePhysical = 12;   { Autosense buffer ptr is physical          }
	kbSCSIFlagReserved5 = 11;   {                               }
	kbSCSIFlagReserved6 = 10;   {                               }
	kbSCSIFlagReserved7 = 9;    {                               }
	kbSCSIFlagReserved8 = 8;    {                               }
	kbSCSIDataBufferValid = 7;    { Data buffer valid                 }
	kbSCSIStatusBufferValid = 6;    { Status buffer valid                 }
	kbSCSIMessageBufferValid = 5;    { Message buffer valid                }
	kbSCSIFlagReserved9 = 4;     {                             }

{ Defines for the bit masks of the scsiFlags field }
const
	scsiDirectionMask = $C0000000; { Data direction mask                 }
	scsiDirectionNone = $C0000000; { Data direction (11: no data)          }
	scsiDirectionReserved = $00000000; { Data direction (00: reserved)       }
	scsiDirectionOut = $80000000; { Data direction (10: DATA OUT)       }
	scsiDirectionIn = $40000000; { Data direction (01: DATA IN)          }
	scsiDisableAutosense = $20000000; { Disable auto sense feature          }
	scsiFlagReservedA = $10000000; {                           }
	scsiFlagReserved0 = $08000000; {                           }
	scsiCDBLinked = $04000000; { The PB contains a linked CDB          }
	scsiQEnable = $02000000; { Target queue actions are enabled      }
	scsiCDBIsPointer = $01000000; { The CDB field contains a pointer      }
	scsiFlagReserved1 = $00800000; {                           }
	scsiInitiateSyncData = $00400000; { Attempt Sync data xfer and SDTR         }
	scsiDisableSyncData = $00200000; { Disable sync, go to async          }
	scsiSIMQHead = $00100000; { Place PB at the head of SIM Q       }
	scsiSIMQFreeze = $00080000; { Return the SIM Q to frozen state      }
	scsiSIMQNoFreeze = $00040000; { Disallow SIM Q freezing              }
	scsiDoDisconnect = $00020000; { Definitely do disconnect             }
	scsiDontDisconnect = $00010000; { Definitely don't disconnect          }
	scsiDataReadyForDMA = $00008000; { Data buffer(s) are ready for DMA      }
	scsiFlagReserved3 = $00004000; {  }
	scsiDataPhysical = $00002000; { SG/Buffer data ptrs are physical      }
	scsiSensePhysical = $00001000; { Autosense buffer ptr is physical      }
	scsiFlagReserved5 = $00000800; {                         }
	scsiFlagReserved6 = $00000400; {                           }
	scsiFlagReserved7 = $00000200; {                           }
	scsiFlagReserved8 = $00000100; {                           }

{ bit masks for the scsiIOFlags field in SCSIExecIOPB }
const
	scsiNoParityCheck = $0002; { disable parity checking                 }
	scsiDisableSelectWAtn = $0004; { disable select w/Atn                    }
	scsiSavePtrOnDisconnect = $0008; { do SaveDataPointer upon Disconnect msg          }
	scsiNoBucketIn = $0010; { don’t bit bucket in during this I/O           }
	scsiNoBucketOut = $0020; { don’t bit bucket out during this I/O        }
	scsiDisableWide = $0040; { disable wide transfer negotiation           }
	scsiInitiateWide = $0080; { initiate wide transfer negotiation           }
	scsiRenegotiateSense = $0100; { renegotiate sync/wide before issuing autosense     }
	scsiDisableDiscipline = $0200; { disable parameter checking on SCSIExecIO calls }
	scsiIOFlagReserved0080 = $0080; {                                }
	scsiIOFlagReserved8000 = $8000; {                                  }

{ Defines for the Bus Inquiry PB fields. }
{ scsiHBAInquiry field bits }
const
	scsiBusMDP = $80; { Supports Modify Data Pointer message               }
	scsiBusWide32 = $40; { Supports 32 bit wide SCSI                    }
	scsiBusWide16 = $20; { Supports 16 bit wide SCSI                    }
	scsiBusSDTR = $10; { Supports Sync Data Transfer Req message              }
	scsiBusLinkedCDB = $08; { Supports linked CDBs                         }
	scsiBusTagQ = $02; { Supports tag queue message                    }
	scsiBusSoftReset = $01;  { Supports soft reset                           }

{ Defines for the scsiDataType field }
const
	scsiDataBuffer = 0;    { single contiguous buffer supplied            }
	scsiDataTIB = 1;    { TIB supplied (ptr in scsiDataPtr)           }
	scsiDataSG = 2;    { scatter/gather list supplied             }
	scsiDataIOTable = 3;     {#(7/11/95) Prepared by Block Storage         }

{ scsiDataTypes field bits  }
{  bits 0->15 Apple-defined, 16->30 3rd-party unique, 31 = reserved }
const
	scsiBusDataTIB = 1 shl scsiDataTIB; { TIB supplied (ptr in scsiDataPtr)   }
	scsiBusDataBuffer = 1 shl scsiDataBuffer; { single contiguous buffer supplied      }
	scsiBusDataSG = 1 shl scsiDataSG; { scatter/gather list supplied        }
	scsiBusDataIOTable = 1 shl scsiDataIOTable; { (2/6/95) Prepare Memory for IO}
	scsiBusDataReserved = $80000000; {                            }

{ scsiScanFlags field bits }
const
	scsiBusScansDevices = $80; { Bus scans for and maintains device list         }
	scsiBusScansOnInit = $40; { Bus scans performed at power-up/reboot        }
	scsiBusLoadsROMDrivers = $20;  { may load ROM drivers to support targets       }

{ scsiFeatureFlags field bits }
const
	scsiBusLVD = $00000400; { HBA is Low Voltage Differential Bus         }
	scsiBusUltra3SCSI = $00000200; { HBA supports Ultra3 SCSI               }
	scsiBusUltra2SCSI = $00000100; { HBA supports Ultra2 SCSI               }
	scsiBusInternalExternalMask = $000000C0; { bus internal/external mask           }
	scsiBusInternalExternalUnknown = $00000000; { not known whether bus is inside or outside     }
	scsiBusInternalExternal = $000000C0; { bus goes inside and outside the box       }
	scsiBusInternal = $00000080; { bus goes inside the box                }
	scsiBusExternal = $00000040; { bus goes outside the box             }
	scsiBusCacheCoherentDMA = $00000020; { DMA is cache coherent                }
	scsiBusOldCallCapable = $00000010; { SIM is old call capable                }
	scsiBusUltraSCSI = $00000008; { HBA supports Ultra SCSI                 }
	scsiBusDifferential = $00000004; { Single Ended (0) or Differential (1)    }
	scsiBusFastSCSI = $00000002; { HBA supports fast SCSI                  }
	scsiBusDMAavailable = $00000001; { DMA is available                  }

{ scsiWeirdStuff field bits }
const
=======

{ Defines for the scsiResultFlags field }
const
	scsiSIMQFrozen = $0001; { The SIM queue is frozen w/this err        }
	scsiAutosenseValid = $0002; { Autosense data valid for target         }
	scsiBusNotFree = $0004; { At time of callback, SCSI bus is not free  }

{ Defines for the bit numbers of the scsiFlags field in the PB header for the SCSIExecIO function }
const
	kbSCSIDisableAutosense = 29;   { Disable auto sense feature               }
	kbSCSIFlagReservedA = 28;   {                             }
	kbSCSIFlagReserved0 = 27;   {                             }
	kbSCSICDBLinked = 26;   { The PB contains a linked CDB             }
	kbSCSIQEnable = 25;   { Target queue actions are enabled          }
	kbSCSICDBIsPointer = 24;   { The CDB field contains a pointer          }
	kbSCSIFlagReserved1 = 23;   {                               }
	kbSCSIInitiateSyncData = 22;   { Attempt Sync data xfer and SDTR          }
	kbSCSIDisableSyncData = 21;   { Disable sync, go to async            }
	kbSCSISIMQHead = 20;   { Place PB at the head of SIM Q          }
	kbSCSISIMQFreeze = 19;   { Return the SIM Q to frozen state          }
	kbSCSISIMQNoFreeze = 18;   { Disallow SIM Q freezing                 }
	kbSCSIDoDisconnect = 17;   { Definitely do disconnect               }
	kbSCSIDontDisconnect = 16;   { Definitely don't disconnect              }
	kbSCSIDataReadyForDMA = 15;   { Data buffer(s) are ready for DMA          }
	kbSCSIFlagReserved3 = 14;   {                               }
	kbSCSIDataPhysical = 13;   { SG/Buffer data ptrs are physical          }
	kbSCSISensePhysical = 12;   { Autosense buffer ptr is physical          }
	kbSCSIFlagReserved5 = 11;   {                               }
	kbSCSIFlagReserved6 = 10;   {                               }
	kbSCSIFlagReserved7 = 9;    {                               }
	kbSCSIFlagReserved8 = 8;    {                               }
	kbSCSIDataBufferValid = 7;    { Data buffer valid                 }
	kbSCSIStatusBufferValid = 6;    { Status buffer valid                 }
	kbSCSIMessageBufferValid = 5;    { Message buffer valid                }
	kbSCSIFlagReserved9 = 4;     {                             }

{ Defines for the bit masks of the scsiFlags field }
const
	scsiDirectionMask = $C0000000; { Data direction mask                 }
	scsiDirectionNone = $C0000000; { Data direction (11: no data)          }
	scsiDirectionReserved = $00000000; { Data direction (00: reserved)       }
	scsiDirectionOut = $80000000; { Data direction (10: DATA OUT)       }
	scsiDirectionIn = $40000000; { Data direction (01: DATA IN)          }
	scsiDisableAutosense = $20000000; { Disable auto sense feature          }
	scsiFlagReservedA = $10000000; {                           }
	scsiFlagReserved0 = $08000000; {                           }
	scsiCDBLinked = $04000000; { The PB contains a linked CDB          }
	scsiQEnable = $02000000; { Target queue actions are enabled      }
	scsiCDBIsPointer = $01000000; { The CDB field contains a pointer      }
	scsiFlagReserved1 = $00800000; {                           }
	scsiInitiateSyncData = $00400000; { Attempt Sync data xfer and SDTR         }
	scsiDisableSyncData = $00200000; { Disable sync, go to async          }
	scsiSIMQHead = $00100000; { Place PB at the head of SIM Q       }
	scsiSIMQFreeze = $00080000; { Return the SIM Q to frozen state      }
	scsiSIMQNoFreeze = $00040000; { Disallow SIM Q freezing              }
	scsiDoDisconnect = $00020000; { Definitely do disconnect             }
	scsiDontDisconnect = $00010000; { Definitely don't disconnect          }
	scsiDataReadyForDMA = $00008000; { Data buffer(s) are ready for DMA      }
	scsiFlagReserved3 = $00004000; {  }
	scsiDataPhysical = $00002000; { SG/Buffer data ptrs are physical      }
	scsiSensePhysical = $00001000; { Autosense buffer ptr is physical      }
	scsiFlagReserved5 = $00000800; {                         }
	scsiFlagReserved6 = $00000400; {                           }
	scsiFlagReserved7 = $00000200; {                           }
	scsiFlagReserved8 = $00000100; {                           }

{ bit masks for the scsiIOFlags field in SCSIExecIOPB }
const
	scsiNoParityCheck = $0002; { disable parity checking                 }
	scsiDisableSelectWAtn = $0004; { disable select w/Atn                    }
	scsiSavePtrOnDisconnect = $0008; { do SaveDataPointer upon Disconnect msg          }
	scsiNoBucketIn = $0010; { don’t bit bucket in during this I/O           }
	scsiNoBucketOut = $0020; { don’t bit bucket out during this I/O        }
	scsiDisableWide = $0040; { disable wide transfer negotiation           }
	scsiInitiateWide = $0080; { initiate wide transfer negotiation           }
	scsiRenegotiateSense = $0100; { renegotiate sync/wide before issuing autosense     }
	scsiDisableDiscipline = $0200; { disable parameter checking on SCSIExecIO calls }
	scsiIOFlagReserved0080 = $0080; {                                }
	scsiIOFlagReserved8000 = $8000; {                                  }

{ Defines for the Bus Inquiry PB fields. }
{ scsiHBAInquiry field bits }
const
	scsiBusMDP = $80; { Supports Modify Data Pointer message               }
	scsiBusWide32 = $40; { Supports 32 bit wide SCSI                    }
	scsiBusWide16 = $20; { Supports 16 bit wide SCSI                    }
	scsiBusSDTR = $10; { Supports Sync Data Transfer Req message              }
	scsiBusLinkedCDB = $08; { Supports linked CDBs                         }
	scsiBusTagQ = $02; { Supports tag queue message                    }
	scsiBusSoftReset = $01;  { Supports soft reset                           }

{ Defines for the scsiDataType field }
const
=======

=======

>>>>>>> origin/cpstrnew
{ Defines for the scsiResultFlags field }
const
	scsiSIMQFrozen = $0001; { The SIM queue is frozen w/this err        }
	scsiAutosenseValid = $0002; { Autosense data valid for target         }
	scsiBusNotFree = $0004; { At time of callback, SCSI bus is not free  }

{ Defines for the bit numbers of the scsiFlags field in the PB header for the SCSIExecIO function }
const
	kbSCSIDisableAutosense = 29;   { Disable auto sense feature               }
	kbSCSIFlagReservedA = 28;   {                             }
	kbSCSIFlagReserved0 = 27;   {                             }
	kbSCSICDBLinked = 26;   { The PB contains a linked CDB             }
	kbSCSIQEnable = 25;   { Target queue actions are enabled          }
	kbSCSICDBIsPointer = 24;   { The CDB field contains a pointer          }
	kbSCSIFlagReserved1 = 23;   {                               }
	kbSCSIInitiateSyncData = 22;   { Attempt Sync data xfer and SDTR          }
	kbSCSIDisableSyncData = 21;   { Disable sync, go to async            }
	kbSCSISIMQHead = 20;   { Place PB at the head of SIM Q          }
	kbSCSISIMQFreeze = 19;   { Return the SIM Q to frozen state          }
	kbSCSISIMQNoFreeze = 18;   { Disallow SIM Q freezing                 }
	kbSCSIDoDisconnect = 17;   { Definitely do disconnect               }
	kbSCSIDontDisconnect = 16;   { Definitely don't disconnect              }
	kbSCSIDataReadyForDMA = 15;   { Data buffer(s) are ready for DMA          }
	kbSCSIFlagReserved3 = 14;   {                               }
	kbSCSIDataPhysical = 13;   { SG/Buffer data ptrs are physical          }
	kbSCSISensePhysical = 12;   { Autosense buffer ptr is physical          }
	kbSCSIFlagReserved5 = 11;   {                               }
	kbSCSIFlagReserved6 = 10;   {                               }
	kbSCSIFlagReserved7 = 9;    {                               }
	kbSCSIFlagReserved8 = 8;    {                               }
	kbSCSIDataBufferValid = 7;    { Data buffer valid                 }
	kbSCSIStatusBufferValid = 6;    { Status buffer valid                 }
	kbSCSIMessageBufferValid = 5;    { Message buffer valid                }
	kbSCSIFlagReserved9 = 4;     {                             }

{ Defines for the bit masks of the scsiFlags field }
const
	scsiDirectionMask = $C0000000; { Data direction mask                 }
	scsiDirectionNone = $C0000000; { Data direction (11: no data)          }
	scsiDirectionReserved = $00000000; { Data direction (00: reserved)       }
	scsiDirectionOut = $80000000; { Data direction (10: DATA OUT)       }
	scsiDirectionIn = $40000000; { Data direction (01: DATA IN)          }
	scsiDisableAutosense = $20000000; { Disable auto sense feature          }
	scsiFlagReservedA = $10000000; {                           }
	scsiFlagReserved0 = $08000000; {                           }
	scsiCDBLinked = $04000000; { The PB contains a linked CDB          }
	scsiQEnable = $02000000; { Target queue actions are enabled      }
	scsiCDBIsPointer = $01000000; { The CDB field contains a pointer      }
	scsiFlagReserved1 = $00800000; {                           }
	scsiInitiateSyncData = $00400000; { Attempt Sync data xfer and SDTR         }
	scsiDisableSyncData = $00200000; { Disable sync, go to async          }
	scsiSIMQHead = $00100000; { Place PB at the head of SIM Q       }
	scsiSIMQFreeze = $00080000; { Return the SIM Q to frozen state      }
	scsiSIMQNoFreeze = $00040000; { Disallow SIM Q freezing              }
	scsiDoDisconnect = $00020000; { Definitely do disconnect             }
	scsiDontDisconnect = $00010000; { Definitely don't disconnect          }
	scsiDataReadyForDMA = $00008000; { Data buffer(s) are ready for DMA      }
	scsiFlagReserved3 = $00004000; {  }
	scsiDataPhysical = $00002000; { SG/Buffer data ptrs are physical      }
	scsiSensePhysical = $00001000; { Autosense buffer ptr is physical      }
	scsiFlagReserved5 = $00000800; {                         }
	scsiFlagReserved6 = $00000400; {                           }
	scsiFlagReserved7 = $00000200; {                           }
	scsiFlagReserved8 = $00000100; {                           }

{ bit masks for the scsiIOFlags field in SCSIExecIOPB }
const
	scsiNoParityCheck = $0002; { disable parity checking                 }
	scsiDisableSelectWAtn = $0004; { disable select w/Atn                    }
	scsiSavePtrOnDisconnect = $0008; { do SaveDataPointer upon Disconnect msg          }
	scsiNoBucketIn = $0010; { don’t bit bucket in during this I/O           }
	scsiNoBucketOut = $0020; { don’t bit bucket out during this I/O        }
	scsiDisableWide = $0040; { disable wide transfer negotiation           }
	scsiInitiateWide = $0080; { initiate wide transfer negotiation           }
	scsiRenegotiateSense = $0100; { renegotiate sync/wide before issuing autosense     }
	scsiDisableDiscipline = $0200; { disable parameter checking on SCSIExecIO calls }
	scsiIOFlagReserved0080 = $0080; {                                }
	scsiIOFlagReserved8000 = $8000; {                                  }

{ Defines for the Bus Inquiry PB fields. }
{ scsiHBAInquiry field bits }
const
	scsiBusMDP = $80; { Supports Modify Data Pointer message               }
	scsiBusWide32 = $40; { Supports 32 bit wide SCSI                    }
	scsiBusWide16 = $20; { Supports 16 bit wide SCSI                    }
	scsiBusSDTR = $10; { Supports Sync Data Transfer Req message              }
	scsiBusLinkedCDB = $08; { Supports linked CDBs                         }
	scsiBusTagQ = $02; { Supports tag queue message                    }
	scsiBusSoftReset = $01;  { Supports soft reset                           }

{ Defines for the scsiDataType field }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	scsiDataBuffer = 0;    { single contiguous buffer supplied            }
	scsiDataTIB = 1;    { TIB supplied (ptr in scsiDataPtr)           }
	scsiDataSG = 2;    { scatter/gather list supplied             }
	scsiDataIOTable = 3;     {#(7/11/95) Prepared by Block Storage         }

{ scsiDataTypes field bits  }
{  bits 0->15 Apple-defined, 16->30 3rd-party unique, 31 = reserved }
const
	scsiBusDataTIB = 1 shl scsiDataTIB; { TIB supplied (ptr in scsiDataPtr)   }
	scsiBusDataBuffer = 1 shl scsiDataBuffer; { single contiguous buffer supplied      }
	scsiBusDataSG = 1 shl scsiDataSG; { scatter/gather list supplied        }
	scsiBusDataIOTable = 1 shl scsiDataIOTable; { (2/6/95) Prepare Memory for IO}
	scsiBusDataReserved = $80000000; {                            }

{ scsiScanFlags field bits }
const
	scsiBusScansDevices = $80; { Bus scans for and maintains device list         }
	scsiBusScansOnInit = $40; { Bus scans performed at power-up/reboot        }
	scsiBusLoadsROMDrivers = $20;  { may load ROM drivers to support targets       }

{ scsiFeatureFlags field bits }
const
	scsiBusLVD = $00000400; { HBA is Low Voltage Differential Bus         }
	scsiBusUltra3SCSI = $00000200; { HBA supports Ultra3 SCSI               }
	scsiBusUltra2SCSI = $00000100; { HBA supports Ultra2 SCSI               }
	scsiBusInternalExternalMask = $000000C0; { bus internal/external mask           }
	scsiBusInternalExternalUnknown = $00000000; { not known whether bus is inside or outside     }
	scsiBusInternalExternal = $000000C0; { bus goes inside and outside the box       }
	scsiBusInternal = $00000080; { bus goes inside the box                }
	scsiBusExternal = $00000040; { bus goes outside the box             }
	scsiBusCacheCoherentDMA = $00000020; { DMA is cache coherent                }
	scsiBusOldCallCapable = $00000010; { SIM is old call capable                }
	scsiBusUltraSCSI = $00000008; { HBA supports Ultra SCSI                 }
	scsiBusDifferential = $00000004; { Single Ended (0) or Differential (1)    }
	scsiBusFastSCSI = $00000002; { HBA supports fast SCSI                  }
	scsiBusDMAavailable = $00000001; { DMA is available                  }

{ scsiWeirdStuff field bits }
const
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	scsiOddDisconnectUnsafeRead1 = $0001; { Disconnects on odd byte boundries are unsafe with DMA and/or blind reads }
	scsiOddDisconnectUnsafeWrite1 = $0002; { Disconnects on odd byte boundries are unsafe with DMA and/or blind writes }
	scsiBusErrorsUnsafe = $0004; { Non-handshaked delays or disconnects during blind transfers may cause a crash }
	scsiRequiresHandshake = $0008; { Non-handshaked delays or disconnects during blind transfers may cause data corruption }
	scsiTargetDrivenSDTRSafe = $0010; { Targets which initiate synchronous negotiations are supported }
	scsiOddCountForPhysicalUnsafe = $0020; { If using physical addrs all counts must be even, and disconnects must be on even boundries }
	scsiAbortCmdFixed = $0040; { Set if abort command is fixed to properly make callbacks }
	scsiMeshACKTimingFixed = $0080; { Set if bug allowing Mesh to release ACK prematurely is fixed }

{ scsiHBAslotType values }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
	scsiRequestInProgress = 1;    { 1   = PB request is in progress          }
                                        { Execution failed  00-2F }
	scsiRequestAborted = scsiErrorBase + 2; { -7934 = PB request aborted by the host        }
	scsiUnableToAbort = scsiErrorBase + 3; { -7933 = Unable to Abort PB request          }
	scsiNonZeroStatus = scsiErrorBase + 4; { -7932 = PB request completed with an err   }
	scsiUnused05 = scsiErrorBase + 5; { -7931 =                        }
	scsiUnused06 = scsiErrorBase + 6; { -7930 =                        }
	scsiUnused07 = scsiErrorBase + 7; { -7929 =                        }
	scsiUnused08 = scsiErrorBase + 8; { -7928 =                        }
	scsiUnableToTerminate = scsiErrorBase + 9; { -7927 = Unable to Terminate I/O PB req        }
	scsiSelectTimeout = scsiErrorBase + 10; { -7926 = Target selection timeout        }
	scsiCommandTimeout = scsiErrorBase + 11; { -7925 = Command timeout              }
	scsiIdentifyMessageRejected = scsiErrorBase + 12; { -7924 =                        }
	scsiMessageRejectReceived = scsiErrorBase + 13; { -7923 = Message reject received           }
	scsiSCSIBusReset = scsiErrorBase + 14; { -7922 = SCSI bus reset sent/received    }
	scsiParityError = scsiErrorBase + 15; { -7921 = Uncorrectable parity error occured     }
	scsiAutosenseFailed = scsiErrorBase + 16; { -7920 = Autosense: Request sense cmd fail  }
	scsiUnused11 = scsiErrorBase + 17; { -7919 =                        }
	scsiDataRunError = scsiErrorBase + 18; { -7918 = Data overrun/underrun error     }
	scsiUnexpectedBusFree = scsiErrorBase + 19; { -7917 = Unexpected BUS free              }
	scsiSequenceFailed = scsiErrorBase + 20; { -7916 = Target bus phase sequence failure  }
	scsiWrongDirection = scsiErrorBase + 21; { -7915 = Data phase was in wrong direction  }
	scsiUnused16 = scsiErrorBase + 22; { -7914 =                        }
	scsiBDRsent = scsiErrorBase + 23; { -7913 = A SCSI BDR msg was sent to target  }
	scsiTerminated = scsiErrorBase + 24; { -7912 = PB request terminated by the host  }
	scsiNoNexus = scsiErrorBase + 25; { -7911 = Nexus is not established        }
	scsiCDBReceived = scsiErrorBase + 26; { -7910 = The SCSI CDB has been received        }
                                        { Couldn't begin execution  30-3F }
	scsiTooManyBuses = scsiErrorBase + 48; { -7888 = Register failed because we're full }
	scsiBusy = scsiErrorBase + 49; { -7887 = SCSI subsystem is busy           }
	scsiProvideFail = scsiErrorBase + 50; { -7886 = Unable to provide requ. capability }
	scsiDeviceNotThere = scsiErrorBase + 51; { -7885 = SCSI device not installed/there    }
	scsiNoHBA = scsiErrorBase + 52; { -7884 = No HBA detected Error           }
	scsiDeviceConflict = scsiErrorBase + 53; { -7883 = sorry, max 1 refNum per DeviceIdent    }
	scsiNoSuchXref = scsiErrorBase + 54; { -7882 = no such RefNum xref              }
	scsiQLinkInvalid = scsiErrorBase + 55; { -7881 = pre-linked PBs not supported      }
                                        {   (The QLink field was nonzero)          }
                                        { Parameter errors  40-7F }
	scsiPBLengthError = scsiErrorBase + 64; { -7872 = (scsiPBLength is insuf'ct/invalid  }
	scsiFunctionNotAvailable = scsiErrorBase + 65; { -7871 = The requ. func is not available    }
	scsiRequestInvalid = scsiErrorBase + 66; { -7870 = PB request is invalid           }
	scsiBusInvalid = scsiErrorBase + 67; { -7869 = Bus ID supplied is invalid        }
	scsiTIDInvalid = scsiErrorBase + 68; { -7868 = Target ID supplied is invalid      }
	scsiLUNInvalid = scsiErrorBase + 69; { -7867 = LUN supplied is invalid         }
	scsiIDInvalid = scsiErrorBase + 70; { -7866 = The initiator ID is invalid     }
	scsiDataTypeInvalid = scsiErrorBase + 71; { -7865 = scsiDataType requested not supported }
	scsiTransferTypeInvalid = scsiErrorBase + 72; { -7864 = scsiTransferType field is too high     }
	scsiCDBLengthInvalid = scsiErrorBase + 73; { -7863 = scsiCDBLength field is too big        }

{ New errors for SCSI Family         }
const
	scsiUnused74 = scsiErrorBase + 74; { -7862 =                          }
	scsiUnused75 = scsiErrorBase + 75; { -7861 =                          }
	scsiBadDataLength = scsiErrorBase + 76; { -7860 = a zero data length in PB        }
	scsiPartialPrepared = scsiErrorBase + 77; { -7859 = could not do full prepare mem for I/O}
	scsiInvalidMsgType = scsiErrorBase + 78; { -7858 = Invalid message type (internal)       }
	scsiUnused79 = scsiErrorBase + 79; { -7857 =                              }
	scsiBadConnID = scsiErrorBase + 80; { -7856 = Bad Connection ID                }
	scsiUnused81 = scsiErrorBase + 81; { -7855 =                          }
	scsiIOInProgress = scsiErrorBase + 82; { -7854 = Can't close conn, IO in prog      }
	scsiTargetReserved = scsiErrorBase + 83; { -7853 = Target already reserved          }
	scsiUnused84 = scsiErrorBase + 84; { -7852 =                          }
	scsiUnused85 = scsiErrorBase + 85; { -7851 =                          }
	scsiBadConnType = scsiErrorBase + 86; { -7850 = Bad connection type              }
	scsiCannotLoadPlugin = scsiErrorBase + 87; { -7849 = No matching service category      }

{ +++ }
{
 * scsiFamilyInternalError and scsiPluginInternalError are intended to handle consistency check failures.
 * For example, if the family stores a record on a lookaside queue, but does not find that record
 * it can use this error to report this failure. SCSI Manager 4.3 uses dsIOCoreErr in a few places,
 * but this is probably not the best error. In general, internal errors should be reported as bugs.
 *
 * The following range of errors is provided for third-party (non-Apple) SCSI SIM and device driver vendors.
 * In general, they would be used for error conditions that are not covered by the standardized errors.
 * They should not normally be conveyed to normal applications, but might be used for communication between
 * a plug-in and a vendor-provided device driver (for example, to manage RAID hot-swapping).
 *
 * Note: I don't know how many SCSI errors are reserved in the error code architecture. Don't assume that
 * we'll actually get sixteen, but we should reserve at least one.
 }
const
	scsiFamilyInternalError = scsiErrorBase + 87; { -7849 = Internal consistency check failed  }
	scsiPluginInternalError = scsiErrorBase + 88; { -7848 = Internal consistency check failed  }
	scsiVendorSpecificErrorBase = scsiErrorBase + 128; { ??    = Start of third-party error range     }
	scsiVendorSpecificErrorCount = 16;    { Number of third-party errors             }

{ --- }
const
	scsiExecutionErrors = scsiErrorBase;
	scsiNotExecutedErrors = scsiTooManyBuses;
	scsiParameterErrors = scsiPBLengthError;

{ Defines for the scsiResultFlags field }
const
	scsiSIMQFrozen = $0001; { The SIM queue is frozen w/this err        }
	scsiAutosenseValid = $0002; { Autosense data valid for target         }
	scsiBusNotFree = $0004; { At time of callback, SCSI bus is not free  }

{ Defines for the bit numbers of the scsiFlags field in the PB header for the SCSIExecIO function }
const
	kbSCSIDisableAutosense = 29;   { Disable auto sense feature               }
	kbSCSIFlagReservedA = 28;   {                             }
	kbSCSIFlagReserved0 = 27;   {                             }
	kbSCSICDBLinked = 26;   { The PB contains a linked CDB             }
	kbSCSIQEnable = 25;   { Target queue actions are enabled          }
	kbSCSICDBIsPointer = 24;   { The CDB field contains a pointer          }
	kbSCSIFlagReserved1 = 23;   {                               }
	kbSCSIInitiateSyncData = 22;   { Attempt Sync data xfer and SDTR          }
	kbSCSIDisableSyncData = 21;   { Disable sync, go to async            }
	kbSCSISIMQHead = 20;   { Place PB at the head of SIM Q          }
	kbSCSISIMQFreeze = 19;   { Return the SIM Q to frozen state          }
	kbSCSISIMQNoFreeze = 18;   { Disallow SIM Q freezing                 }
	kbSCSIDoDisconnect = 17;   { Definitely do disconnect               }
	kbSCSIDontDisconnect = 16;   { Definitely don't disconnect              }
	kbSCSIDataReadyForDMA = 15;   { Data buffer(s) are ready for DMA          }
	kbSCSIFlagReserved3 = 14;   {                               }
	kbSCSIDataPhysical = 13;   { SG/Buffer data ptrs are physical          }
	kbSCSISensePhysical = 12;   { Autosense buffer ptr is physical          }
	kbSCSIFlagReserved5 = 11;   {                               }
	kbSCSIFlagReserved6 = 10;   {                               }
	kbSCSIFlagReserved7 = 9;    {                               }
	kbSCSIFlagReserved8 = 8;    {                               }
	kbSCSIDataBufferValid = 7;    { Data buffer valid                 }
	kbSCSIStatusBufferValid = 6;    { Status buffer valid                 }
	kbSCSIMessageBufferValid = 5;    { Message buffer valid                }
	kbSCSIFlagReserved9 = 4;     {                             }

{ Defines for the bit masks of the scsiFlags field }
const
	scsiDirectionMask = $C0000000; { Data direction mask                 }
	scsiDirectionNone = $C0000000; { Data direction (11: no data)          }
	scsiDirectionReserved = $00000000; { Data direction (00: reserved)       }
	scsiDirectionOut = $80000000; { Data direction (10: DATA OUT)       }
	scsiDirectionIn = $40000000; { Data direction (01: DATA IN)          }
	scsiDisableAutosense = $20000000; { Disable auto sense feature          }
	scsiFlagReservedA = $10000000; {                           }
	scsiFlagReserved0 = $08000000; {                           }
	scsiCDBLinked = $04000000; { The PB contains a linked CDB          }
	scsiQEnable = $02000000; { Target queue actions are enabled      }
	scsiCDBIsPointer = $01000000; { The CDB field contains a pointer      }
	scsiFlagReserved1 = $00800000; {                           }
	scsiInitiateSyncData = $00400000; { Attempt Sync data xfer and SDTR         }
	scsiDisableSyncData = $00200000; { Disable sync, go to async          }
	scsiSIMQHead = $00100000; { Place PB at the head of SIM Q       }
	scsiSIMQFreeze = $00080000; { Return the SIM Q to frozen state      }
	scsiSIMQNoFreeze = $00040000; { Disallow SIM Q freezing              }
	scsiDoDisconnect = $00020000; { Definitely do disconnect             }
	scsiDontDisconnect = $00010000; { Definitely don't disconnect          }
	scsiDataReadyForDMA = $00008000; { Data buffer(s) are ready for DMA      }
	scsiFlagReserved3 = $00004000; {  }
	scsiDataPhysical = $00002000; { SG/Buffer data ptrs are physical      }
	scsiSensePhysical = $00001000; { Autosense buffer ptr is physical      }
	scsiFlagReserved5 = $00000800; {                         }
	scsiFlagReserved6 = $00000400; {                           }
	scsiFlagReserved7 = $00000200; {                           }
	scsiFlagReserved8 = $00000100; {                           }

{ bit masks for the scsiIOFlags field in SCSIExecIOPB }
const
	scsiNoParityCheck = $0002; { disable parity checking                 }
	scsiDisableSelectWAtn = $0004; { disable select w/Atn                    }
	scsiSavePtrOnDisconnect = $0008; { do SaveDataPointer upon Disconnect msg          }
	scsiNoBucketIn = $0010; { don’t bit bucket in during this I/O           }
	scsiNoBucketOut = $0020; { don’t bit bucket out during this I/O        }
	scsiDisableWide = $0040; { disable wide transfer negotiation           }
	scsiInitiateWide = $0080; { initiate wide transfer negotiation           }
	scsiRenegotiateSense = $0100; { renegotiate sync/wide before issuing autosense     }
	scsiDisableDiscipline = $0200; { disable parameter checking on SCSIExecIO calls }
	scsiIOFlagReserved0080 = $0080; {                                }
	scsiIOFlagReserved8000 = $8000; {                                  }

{ Defines for the Bus Inquiry PB fields. }
{ scsiHBAInquiry field bits }
const
	scsiBusMDP = $80; { Supports Modify Data Pointer message               }
	scsiBusWide32 = $40; { Supports 32 bit wide SCSI                    }
	scsiBusWide16 = $20; { Supports 16 bit wide SCSI                    }
	scsiBusSDTR = $10; { Supports Sync Data Transfer Req message              }
	scsiBusLinkedCDB = $08; { Supports linked CDBs                         }
	scsiBusTagQ = $02; { Supports tag queue message                    }
	scsiBusSoftReset = $01;  { Supports soft reset                           }

{ Defines for the scsiDataType field }
const
	scsiDataBuffer = 0;    { single contiguous buffer supplied            }
	scsiDataTIB = 1;    { TIB supplied (ptr in scsiDataPtr)           }
	scsiDataSG = 2;    { scatter/gather list supplied             }
	scsiDataIOTable = 3;     {#(7/11/95) Prepared by Block Storage         }

{ scsiDataTypes field bits  }
{  bits 0->15 Apple-defined, 16->30 3rd-party unique, 31 = reserved }
const
	scsiBusDataTIB = 1 shl scsiDataTIB; { TIB supplied (ptr in scsiDataPtr)   }
	scsiBusDataBuffer = 1 shl scsiDataBuffer; { single contiguous buffer supplied      }
	scsiBusDataSG = 1 shl scsiDataSG; { scatter/gather list supplied        }
	scsiBusDataIOTable = 1 shl scsiDataIOTable; { (2/6/95) Prepare Memory for IO}
	scsiBusDataReserved = $80000000; {                            }

{ scsiScanFlags field bits }
const
	scsiBusScansDevices = $80; { Bus scans for and maintains device list         }
	scsiBusScansOnInit = $40; { Bus scans performed at power-up/reboot        }
	scsiBusLoadsROMDrivers = $20;  { may load ROM drivers to support targets       }

{ scsiFeatureFlags field bits }
const
	scsiBusLVD = $00000400; { HBA is Low Voltage Differential Bus         }
	scsiBusUltra3SCSI = $00000200; { HBA supports Ultra3 SCSI               }
	scsiBusUltra2SCSI = $00000100; { HBA supports Ultra2 SCSI               }
	scsiBusInternalExternalMask = $000000C0; { bus internal/external mask           }
	scsiBusInternalExternalUnknown = $00000000; { not known whether bus is inside or outside     }
	scsiBusInternalExternal = $000000C0; { bus goes inside and outside the box       }
	scsiBusInternal = $00000080; { bus goes inside the box                }
	scsiBusExternal = $00000040; { bus goes outside the box             }
	scsiBusCacheCoherentDMA = $00000020; { DMA is cache coherent                }
	scsiBusOldCallCapable = $00000010; { SIM is old call capable                }
	scsiBusUltraSCSI = $00000008; { HBA supports Ultra SCSI                 }
	scsiBusDifferential = $00000004; { Single Ended (0) or Differential (1)    }
	scsiBusFastSCSI = $00000002; { HBA supports fast SCSI                  }
	scsiBusDMAavailable = $00000001; { DMA is available                  }

{ scsiWeirdStuff field bits }
const
	scsiOddDisconnectUnsafeRead1 = $0001; { Disconnects on odd byte boundries are unsafe with DMA and/or blind reads }
	scsiOddDisconnectUnsafeWrite1 = $0002; { Disconnects on odd byte boundries are unsafe with DMA and/or blind writes }
	scsiBusErrorsUnsafe = $0004; { Non-handshaked delays or disconnects during blind transfers may cause a crash }
	scsiRequiresHandshake = $0008; { Non-handshaked delays or disconnects during blind transfers may cause data corruption }
	scsiTargetDrivenSDTRSafe = $0010; { Targets which initiate synchronous negotiations are supported }
	scsiOddCountForPhysicalUnsafe = $0020; { If using physical addrs all counts must be even, and disconnects must be on even boundries }
	scsiAbortCmdFixed = $0040; { Set if abort command is fixed to properly make callbacks }
	scsiMeshACKTimingFixed = $0080; { Set if bug allowing Mesh to release ACK prematurely is fixed }

{ scsiHBAslotType values }
const
>>>>>>> graemeg/cpstrnew
	scsiMotherboardBus = $00; { A built in Apple supplied bus            }
	scsiNuBus = $01; { A SIM on a NuBus card                   }
	scsiPDSBus = $03; {    "  on a PDS card                    }
	scsiPCIBus = $04; {    "  on a PCI bus card                   }
	scsiPCMCIABus = $05; {    "  on a PCMCIA card                  }
	scsiFireWireBridgeBus = $06; {    "  connected through a FireWire bridge   }
	scsiUSBBus = $07;  {    "  connected on a USB bus               }

{ Defines for the scsiDriverFlags field (in SCSIDriverPB) }
const
	scsiDeviceSensitive = $0001; { Only driver should access this device          }
	scsiDeviceNoOldCallAccess = $0002; { no old call access to this device            }

{
=======
	scsiMotherboardBus = $00; { A built in Apple supplied bus            }
	scsiNuBus = $01; { A SIM on a NuBus card                   }
	scsiPDSBus = $03; {    "  on a PDS card                    }
	scsiPCIBus = $04; {    "  on a PCI bus card                   }
	scsiPCMCIABus = $05; {    "  on a PCMCIA card                  }
	scsiFireWireBridgeBus = $06; {    "  connected through a FireWire bridge   }
	scsiUSBBus = $07;  {    "  connected on a USB bus               }

{ Defines for the scsiDriverFlags field (in SCSIDriverPB) }
const
	scsiDeviceSensitive = $0001; { Only driver should access this device          }
	scsiDeviceNoOldCallAccess = $0002; { no old call access to this device            }

{
>>>>>>> graemeg/cpstrnew
=======
	scsiMotherboardBus = $00; { A built in Apple supplied bus            }
	scsiNuBus = $01; { A SIM on a NuBus card                   }
	scsiPDSBus = $03; {    "  on a PDS card                    }
	scsiPCIBus = $04; {    "  on a PCI bus card                   }
	scsiPCMCIABus = $05; {    "  on a PCMCIA card                  }
	scsiFireWireBridgeBus = $06; {    "  connected through a FireWire bridge   }
	scsiUSBBus = $07;  {    "  connected on a USB bus               }

{ Defines for the scsiDriverFlags field (in SCSIDriverPB) }
const
	scsiDeviceSensitive = $0001; { Only driver should access this device          }
	scsiDeviceNoOldCallAccess = $0002; { no old call access to this device            }

{
>>>>>>> graemeg/cpstrnew
=======
	scsiMotherboardBus = $00; { A built in Apple supplied bus            }
	scsiNuBus = $01; { A SIM on a NuBus card                   }
	scsiPDSBus = $03; {    "  on a PDS card                    }
	scsiPCIBus = $04; {    "  on a PCI bus card                   }
	scsiPCMCIABus = $05; {    "  on a PCMCIA card                  }
	scsiFireWireBridgeBus = $06; {    "  connected through a FireWire bridge   }
	scsiUSBBus = $07;  {    "  connected on a USB bus               }

{ Defines for the scsiDriverFlags field (in SCSIDriverPB) }
const
	scsiDeviceSensitive = $0001; { Only driver should access this device          }
	scsiDeviceNoOldCallAccess = $0002; { no old call access to this device            }

{
>>>>>>> origin/cpstrnew
 * SCSI bus status. These values are returned by the SCSI target in the status phase.
 * They are not related to Macintosh status values (except that values other than
 * scsiStatusGood will result in scsiResult set to scsiNonZeroStatus).
 }
const
	scsiStatGood = $00; { Good Status}
	scsiStatCheckCondition = $02; { Check Condition}
	scsiStatConditionMet = $04; { Condition Met}
	scsiStatBusy = $08; { Busy}
	scsiStatIntermediate = $10; { Intermediate}
	scsiStatIntermedMet = $14; { Intermediate - Condition Met}
	scsiStatResvConflict = $18; { Reservation conflict}
	scsiStatTerminated = $22; { Command terminated}
	scsiStatQFull = $28;  { Queue full}

{ SCSI messages}
const
	kCmdCompleteMsg = 0;
	kExtendedMsg = 1;    { 0x01}
	kSaveDataPointerMsg = 2;    { 0x02}
	kRestorePointersMsg = 3;    { 0x03}
	kDisconnectMsg = 4;    { 0x04}
	kInitiatorDetectedErrorMsg = 5;    { 0x05}
	kAbortMsg = 6;    { 0x06}
	kMsgRejectMsg = 7;    { 0x07}
	kNoOperationMsg = 8;    { 0x08}
	kMsgParityErrorMsg = 9;    { 0x09}
	kLinkedCmdCompleteMsg = 10;   { 0x0a}
	kLinkedCmdCompleteWithFlagMsg = 11;   { 0x0b}
	kBusDeviceResetMsg = 12;   { 0x0c}
	kAbortTagMsg = 13;   { 0x0d}
	kClearQueueMsg = 14;   { 0x0e}
	kInitiateRecoveryMsg = 15;   { 0x0f}
	kReleaseRecoveryMsg = 16;   { 0x10}
	kTerminateIOProcessMsg = 17;   { 0x11}
	kSimpleQueueTag = $20; { 0x20}
	kHeadOfQueueTagMsg = $21; { 0x21}
	kOrderedQueueTagMsg = $22; { 0x22}
	kIgnoreWideResidueMsg = $23;  { 0x23}


{$ifc not TARGET_CPU_64}
{
 *  SCSIAction()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use the SCSITaskUserClient API instead.
 *  
 *  Discussion:
 *    This routine is deprecated. It is exported and callable, but it
 *    is no longer being maintained.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function SCSIAction( var parameterBlock: SCSI_PB ): OSErr; external name '_SCSIAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)


{$endc} {not TARGET_CPU_64}

=======
=======
>>>>>>> origin/fixes_2_2
		qLink:					SCSIHdrPtr;								{  (internal use, must be nil on entry)    }
		scsiReserved1:			SInt16;								{  ->     reserved for input           }
		scsiPBLength:			UInt16;									{  -> Length of the entire PB         }
		scsiFunctionCode:		SInt8;									{  -> function selector            }
		scsiReserved2:			SInt8;									{  <-     reserved for output           }
		scsiResult:				OSErr;									{  <- Returned result                }
		scsiDevice:				DeviceIdent;							{  -> Device Identifier (bus+target+lun) }
		scsiCompletion:			SCSICallbackUPP;						{  -> Callback on completion function      }
		scsiFlags:				UInt32;									{  -> assorted flags             }
		scsiDriverStorage:		BytePtr;								{  <> Ptr for driver private use    }
		scsiXPTprivate:			Ptr;									{  private field for use in XPT       }
		scsiReserved3:			SInt32;								{  reserved                     }
		scsiEngineCount:		UInt16;									{  <- Number of engines on HBA                 }
		scsiMaxTransferType:	UInt16;									{  <- Number of transfer types for this HBA       }
		scsiDataTypes:			UInt32;									{  <- which data types are supported by this SIM   }
		scsiIOpbSize:			UInt16;									{  <- Size of SCSI_IO PB for this SIM/HBA           }
		scsiMaxIOpbSize:		UInt16;									{  <- Size of max SCSI_IO PB for all SIM/HBAs         }
		scsiFeatureFlags:		UInt32;									{  <- Supported features flags field            }
		scsiVersionNumber:		SInt8;									{  <- Version number for the SIM/HBA            }
		scsiHBAInquiry:			SInt8;									{  <- Mimic of INQ byte 7 for the HBA            }
		scsiTargetModeFlags:	SInt8;									{  <- Flags for target mode support            }
		scsiScanFlags:			SInt8;									{  <- Scan related feature flags             }
		scsiSIMPrivatesPtr:		UInt32;									{  <- Ptr to SIM private data area               }
		scsiSIMPrivatesSize:	UInt32;									{  <- Size of SIM private data area            }
		scsiAsyncFlags:			UInt32;									{  <- Event cap. for Async Callback            }
		scsiHiBusID:			SInt8;									{  <- Highest path ID in the subsystem          }
		scsiInitiatorID:		SInt8;									{  <- ID of the HBA on the SCSI bus            }
		scsiBIReserved0:		UInt16;									{                                    }
		scsiBIReserved1:		UInt32;									{  <-                                 }
		scsiFlagsSupported:		UInt32;									{  <- which scsiFlags are supported            }
		scsiIOFlagsSupported:	UInt16;									{  <- which scsiIOFlags are supported            }
		scsiWeirdStuff:			UInt16;									{  <-                                }
		scsiMaxTarget:			UInt16;									{  <- maximum Target number supported            }
		scsiMaxLUN:				UInt16;									{  <- maximum Logical Unit number supported     }
		scsiSIMVendor:			packed array [0..15] of char;			{  <- Vendor ID of SIM (or XPT if bus<FF)         }
		scsiHBAVendor:			packed array [0..15] of char;			{  <- Vendor ID of the HBA                 }
		scsiControllerFamily:	packed array [0..15] of char;			{  <- Family of SCSI Controller            }
		scsiControllerType:		packed array [0..15] of char;			{  <- Specific Model of SCSI Controller used   }
		scsiXPTversion:			packed array [0..3] of char;			{  <- version number of XPT               }
		scsiSIMversion:			packed array [0..3] of char;			{  <- version number of SIM               }
		scsiHBAversion:			packed array [0..3] of char;			{  <- version number of HBA               }
		scsiHBAslotType:		SInt8;									{  <- type of "slot" that this HBA is in        }
		scsiHBAslotNumber:		SInt8;									{  <- slot number of this HBA                   }
		scsiSIMsRsrcID:			UInt16;									{  <- resource ID of this SIM                   }
		scsiBIReserved3:		UInt16;									{  <-                                }
		scsiAdditionalLength:	UInt16;									{  <- additional BusInquiry PB len               }
=======
>>>>>>> origin/fixes_2.4
	end;
{ Abort SIM Request PB }
type
	SCSIAbortCommandPBPtr = ^SCSIAbortCommandPB;
	SCSIAbortCommandPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiIOptr: SCSI_IOPtr;              { Pointer to the PB to abort        }
	end;
{ Terminate I/O Process Request PB }
type
	SCSITerminateIOPBPtr = ^SCSITerminateIOPB;
	SCSITerminateIOPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiIOptr: SCSI_IOPtr;              { Pointer to the PB to terminate        }
	end;
{ Reset SCSI Bus PB }
type
	SCSIResetBusPBPtr = ^SCSIResetBusPB;
	SCSIResetBusPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
	end;
{ Reset SCSI Device PB }
type
	SCSIResetDevicePBPtr = ^SCSIResetDevicePB;
	SCSIResetDevicePB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
	end;
{ Release SIM Queue PB }
type
	SCSIReleaseQPBPtr = ^SCSIReleaseQPB;
	SCSIReleaseQPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: BytePtr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
	end;
{ SCSI Get Virtual ID Info PB }
type
	SCSIGetVirtualIDInfoPBPtr = ^SCSIGetVirtualIDInfoPB;
	SCSIGetVirtualIDInfoPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: Ptr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiOldCallID: UInt16;          { -> SCSI ID of device in question   }
		scsiExists: Boolean;             { <- true if device exists        }
		filler: SInt8;
	end;
{ Create/Lookup/Remove RefNum for Device PB }
type
	SCSIDriverPBPtr = ^SCSIDriverPB;
	SCSIDriverPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: Ptr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiDriver: SInt16;             { -> DriverRefNum, For SetDriver, <- For GetNextDriver }
		scsiDriverFlags: UInt16;        { <> Details of driver/device       }
		scsiNextDevice: DeviceIdent;         { <- DeviceIdent of the NEXT Item in the list  }
	end;
{ Load Driver PB }
type
	SCSILoadDriverPBPtr = ^SCSILoadDriverPB;
	SCSILoadDriverPB = record
		qLink: SCSIHdrPtr;                  { (internal use, must be nil on entry)   }
		scsiReserved1: SInt16;          { ->     reserved for input          }
		scsiPBLength: UInt16;           { -> Length of the entire PB        }
		scsiFunctionCode: UInt8;       { -> function selector           }
		scsiReserved2: UInt8;          { <-     reserved for output          }
		scsiResult: {volatile} OSErr;             { <- Returned result               }
		scsiDevice: DeviceIdent;             { -> Device Identifier (bus+target+lun)}
		scsiCompletion: SCSICallbackUPP;         { -> Callback on completion function     }
		scsiFlags: UInt32;              { -> assorted flags            }
		scsiDriverStorage: Ptr;      { <> Ptr for driver private use   }
		scsiXPTprivate: Ptr;         { private field for use in XPT      }
		scsiReserved3: SIGNEDLONG;          { reserved                    }
		scsiLoadedRefNum: SInt16;       { <- SIM returns refnum of driver      }
		scsiDiskLoadFailed: Boolean;     { -> if true, indicates call after failure to load }
		filler: SInt8;
	end;

{ Defines for the scsiTransferType field }
const
	scsiTransferBlind = 0;
	scsiTransferPolled = 1;

const
	scsiErrorBase = -7936;

const
	scsiRequestInProgress = 1;    { 1   = PB request is in progress          }
                                        { Execution failed  00-2F }
	scsiRequestAborted = scsiErrorBase + 2; { -7934 = PB request aborted by the host        }
	scsiUnableToAbort = scsiErrorBase + 3; { -7933 = Unable to Abort PB request          }
	scsiNonZeroStatus = scsiErrorBase + 4; { -7932 = PB request completed with an err   }
	scsiUnused05 = scsiErrorBase + 5; { -7931 =                        }
	scsiUnused06 = scsiErrorBase + 6; { -7930 =                        }
	scsiUnused07 = scsiErrorBase + 7; { -7929 =                        }
	scsiUnused08 = scsiErrorBase + 8; { -7928 =                        }
	scsiUnableToTerminate = scsiErrorBase + 9; { -7927 = Unable to Terminate I/O PB req        }
	scsiSelectTimeout = scsiErrorBase + 10; { -7926 = Target selection timeout        }
	scsiCommandTimeout = scsiErrorBase + 11; { -7925 = Command timeout              }
	scsiIdentifyMessageRejected = scsiErrorBase + 12; { -7924 =                        }
	scsiMessageRejectReceived = scsiErrorBase + 13; { -7923 = Message reject received           }
	scsiSCSIBusReset = scsiErrorBase + 14; { -7922 = SCSI bus reset sent/received    }
	scsiParityError = scsiErrorBase + 15; { -7921 = Uncorrectable parity error occured     }
	scsiAutosenseFailed = scsiErrorBase + 16; { -7920 = Autosense: Request sense cmd fail  }
	scsiUnused11 = scsiErrorBase + 17; { -7919 =                        }
	scsiDataRunError = scsiErrorBase + 18; { -7918 = Data overrun/underrun error     }
	scsiUnexpectedBusFree = scsiErrorBase + 19; { -7917 = Unexpected BUS free              }
	scsiSequenceFailed = scsiErrorBase + 20; { -7916 = Target bus phase sequence failure  }
	scsiWrongDirection = scsiErrorBase + 21; { -7915 = Data phase was in wrong direction  }
	scsiUnused16 = scsiErrorBase + 22; { -7914 =                        }
	scsiBDRsent = scsiErrorBase + 23; { -7913 = A SCSI BDR msg was sent to target  }
	scsiTerminated = scsiErrorBase + 24; { -7912 = PB request terminated by the host  }
	scsiNoNexus = scsiErrorBase + 25; { -7911 = Nexus is not established        }
	scsiCDBReceived = scsiErrorBase + 26; { -7910 = The SCSI CDB has been received        }
                                        { Couldn't begin execution  30-3F }
	scsiTooManyBuses = scsiErrorBase + 48; { -7888 = Register failed because we're full }
	scsiBusy = scsiErrorBase + 49; { -7887 = SCSI subsystem is busy           }
	scsiProvideFail = scsiErrorBase + 50; { -7886 = Unable to provide requ. capability }
	scsiDeviceNotThere = scsiErrorBase + 51; { -7885 = SCSI device not installed/there    }
	scsiNoHBA = scsiErrorBase + 52; { -7884 = No HBA detected Error           }
	scsiDeviceConflict = scsiErrorBase + 53; { -7883 = sorry, max 1 refNum per DeviceIdent    }
	scsiNoSuchXref = scsiErrorBase + 54; { -7882 = no such RefNum xref              }
	scsiQLinkInvalid = scsiErrorBase + 55; { -7881 = pre-linked PBs not supported      }
                                        {   (The QLink field was nonzero)          }
                                        { Parameter errors  40-7F }
	scsiPBLengthError = scsiErrorBase + 64; { -7872 = (scsiPBLength is insuf'ct/invalid  }
	scsiFunctionNotAvailable = scsiErrorBase + 65; { -7871 = The requ. func is not available    }
	scsiRequestInvalid = scsiErrorBase + 66; { -7870 = PB request is invalid           }
	scsiBusInvalid = scsiErrorBase + 67; { -7869 = Bus ID supplied is invalid        }
	scsiTIDInvalid = scsiErrorBase + 68; { -7868 = Target ID supplied is invalid      }
	scsiLUNInvalid = scsiErrorBase + 69; { -7867 = LUN supplied is invalid         }
	scsiIDInvalid = scsiErrorBase + 70; { -7866 = The initiator ID is invalid     }
	scsiDataTypeInvalid = scsiErrorBase + 71; { -7865 = scsiDataType requested not supported }
	scsiTransferTypeInvalid = scsiErrorBase + 72; { -7864 = scsiTransferType field is too high     }
	scsiCDBLengthInvalid = scsiErrorBase + 73; { -7863 = scsiCDBLength field is too big        }

{ New errors for SCSI Family         }
const
	scsiUnused74 = scsiErrorBase + 74; { -7862 =                          }
	scsiUnused75 = scsiErrorBase + 75; { -7861 =                          }
	scsiBadDataLength = scsiErrorBase + 76; { -7860 = a zero data length in PB        }
	scsiPartialPrepared = scsiErrorBase + 77; { -7859 = could not do full prepare mem for I/O}
	scsiInvalidMsgType = scsiErrorBase + 78; { -7858 = Invalid message type (internal)       }
	scsiUnused79 = scsiErrorBase + 79; { -7857 =                              }
	scsiBadConnID = scsiErrorBase + 80; { -7856 = Bad Connection ID                }
	scsiUnused81 = scsiErrorBase + 81; { -7855 =                          }
	scsiIOInProgress = scsiErrorBase + 82; { -7854 = Can't close conn, IO in prog      }
	scsiTargetReserved = scsiErrorBase + 83; { -7853 = Target already reserved          }
	scsiUnused84 = scsiErrorBase + 84; { -7852 =                          }
	scsiUnused85 = scsiErrorBase + 85; { -7851 =                          }
	scsiBadConnType = scsiErrorBase + 86; { -7850 = Bad connection type              }
	scsiCannotLoadPlugin = scsiErrorBase + 87; { -7849 = No matching service category      }

{ +++ }
{
 * scsiFamilyInternalError and scsiPluginInternalError are intended to handle consistency check failures.
 * For example, if the family stores a record on a lookaside queue, but does not find that record
 * it can use this error to report this failure. SCSI Manager 4.3 uses dsIOCoreErr in a few places,
 * but this is probably not the best error. In general, internal errors should be reported as bugs.
 *
 * The following range of errors is provided for third-party (non-Apple) SCSI SIM and device driver vendors.
 * In general, they would be used for error conditions that are not covered by the standardized errors.
 * They should not normally be conveyed to normal applications, but might be used for communication between
 * a plug-in and a vendor-provided device driver (for example, to manage RAID hot-swapping).
 *
 * Note: I don't know how many SCSI errors are reserved in the error code architecture. Don't assume that
 * we'll actually get sixteen, but we should reserve at least one.
 }
const
	scsiFamilyInternalError = scsiErrorBase + 87; { -7849 = Internal consistency check failed  }
	scsiPluginInternalError = scsiErrorBase + 88; { -7848 = Internal consistency check failed  }
	scsiVendorSpecificErrorBase = scsiErrorBase + 128; { ??    = Start of third-party error range     }
	scsiVendorSpecificErrorCount = 16;    { Number of third-party errors             }

{ --- }
const
	scsiExecutionErrors = scsiErrorBase;
	scsiNotExecutedErrors = scsiTooManyBuses;
	scsiParameterErrors = scsiPBLengthError;

{ Defines for the scsiResultFlags field }
const
	scsiSIMQFrozen = $0001; { The SIM queue is frozen w/this err        }
	scsiAutosenseValid = $0002; { Autosense data valid for target         }
	scsiBusNotFree = $0004; { At time of callback, SCSI bus is not free  }

{ Defines for the bit numbers of the scsiFlags field in the PB header for the SCSIExecIO function }
const
	kbSCSIDisableAutosense = 29;   { Disable auto sense feature               }
	kbSCSIFlagReservedA = 28;   {                             }
	kbSCSIFlagReserved0 = 27;   {                             }
	kbSCSICDBLinked = 26;   { The PB contains a linked CDB             }
	kbSCSIQEnable = 25;   { Target queue actions are enabled          }
	kbSCSICDBIsPointer = 24;   { The CDB field contains a pointer          }
	kbSCSIFlagReserved1 = 23;   {                               }
	kbSCSIInitiateSyncData = 22;   { Attempt Sync data xfer and SDTR          }
	kbSCSIDisableSyncData = 21;   { Disable sync, go to async            }
	kbSCSISIMQHead = 20;   { Place PB at the head of SIM Q          }
	kbSCSISIMQFreeze = 19;   { Return the SIM Q to frozen state          }
	kbSCSISIMQNoFreeze = 18;   { Disallow SIM Q freezing                 }
	kbSCSIDoDisconnect = 17;   { Definitely do disconnect               }
	kbSCSIDontDisconnect = 16;   { Definitely don't disconnect              }
	kbSCSIDataReadyForDMA = 15;   { Data buffer(s) are ready for DMA          }
	kbSCSIFlagReserved3 = 14;   {                               }
	kbSCSIDataPhysical = 13;   { SG/Buffer data ptrs are physical          }
	kbSCSISensePhysical = 12;   { Autosense buffer ptr is physical          }
	kbSCSIFlagReserved5 = 11;   {                               }
	kbSCSIFlagReserved6 = 10;   {                               }
	kbSCSIFlagReserved7 = 9;    {                               }
	kbSCSIFlagReserved8 = 8;    {                               }
	kbSCSIDataBufferValid = 7;    { Data buffer valid                 }
	kbSCSIStatusBufferValid = 6;    { Status buffer valid                 }
	kbSCSIMessageBufferValid = 5;    { Message buffer valid                }
	kbSCSIFlagReserved9 = 4;     {                             }

{ Defines for the bit masks of the scsiFlags field }
const
	scsiDirectionMask = $C0000000; { Data direction mask                 }
	scsiDirectionNone = $C0000000; { Data direction (11: no data)          }
	scsiDirectionReserved = $00000000; { Data direction (00: reserved)       }
	scsiDirectionOut = $80000000; { Data direction (10: DATA OUT)       }
	scsiDirectionIn = $40000000; { Data direction (01: DATA IN)          }
	scsiDisableAutosense = $20000000; { Disable auto sense feature          }
	scsiFlagReservedA = $10000000; {                           }
	scsiFlagReserved0 = $08000000; {                           }
	scsiCDBLinked = $04000000; { The PB contains a linked CDB          }
	scsiQEnable = $02000000; { Target queue actions are enabled      }
	scsiCDBIsPointer = $01000000; { The CDB field contains a pointer      }
	scsiFlagReserved1 = $00800000; {                           }
	scsiInitiateSyncData = $00400000; { Attempt Sync data xfer and SDTR         }
	scsiDisableSyncData = $00200000; { Disable sync, go to async          }
	scsiSIMQHead = $00100000; { Place PB at the head of SIM Q       }
	scsiSIMQFreeze = $00080000; { Return the SIM Q to frozen state      }
	scsiSIMQNoFreeze = $00040000; { Disallow SIM Q freezing              }
	scsiDoDisconnect = $00020000; { Definitely do disconnect             }
	scsiDontDisconnect = $00010000; { Definitely don't disconnect          }
	scsiDataReadyForDMA = $00008000; { Data buffer(s) are ready for DMA      }
	scsiFlagReserved3 = $00004000; {  }
	scsiDataPhysical = $00002000; { SG/Buffer data ptrs are physical      }
	scsiSensePhysical = $00001000; { Autosense buffer ptr is physical      }
	scsiFlagReserved5 = $00000800; {                         }
	scsiFlagReserved6 = $00000400; {                           }
	scsiFlagReserved7 = $00000200; {                           }
	scsiFlagReserved8 = $00000100; {                           }

{ bit masks for the scsiIOFlags field in SCSIExecIOPB }
const
	scsiNoParityCheck = $0002; { disable parity checking                 }
	scsiDisableSelectWAtn = $0004; { disable select w/Atn                    }
	scsiSavePtrOnDisconnect = $0008; { do SaveDataPointer upon Disconnect msg          }
	scsiNoBucketIn = $0010; { don’t bit bucket in during this I/O           }
	scsiNoBucketOut = $0020; { don’t bit bucket out during this I/O        }
	scsiDisableWide = $0040; { disable wide transfer negotiation           }
	scsiInitiateWide = $0080; { initiate wide transfer negotiation           }
	scsiRenegotiateSense = $0100; { renegotiate sync/wide before issuing autosense     }
	scsiDisableDiscipline = $0200; { disable parameter checking on SCSIExecIO calls }
	scsiIOFlagReserved0080 = $0080; {                                }
	scsiIOFlagReserved8000 = $8000; {                                  }

{ Defines for the Bus Inquiry PB fields. }
{ scsiHBAInquiry field bits }
const
	scsiBusMDP = $80; { Supports Modify Data Pointer message               }
	scsiBusWide32 = $40; { Supports 32 bit wide SCSI                    }
	scsiBusWide16 = $20; { Supports 16 bit wide SCSI                    }
	scsiBusSDTR = $10; { Supports Sync Data Transfer Req message              }
	scsiBusLinkedCDB = $08; { Supports linked CDBs                         }
	scsiBusTagQ = $02; { Supports tag queue message                    }
	scsiBusSoftReset = $01;  { Supports soft reset                           }

{ Defines for the scsiDataType field }
const
<<<<<<< HEAD
	xptSCSIAction				= $0001;
	xptSCSIRegisterBus			= $0002;
	xptSCSIDeregisterBus		= $0003;
	xptSCSIReregisterBus		= $0004;
	xptSCSIKillXPT				= $0005;						{  kills Mini-XPT after transition  }
	xptSCSIInitialize			= $000A;						{  Initialize the SCSI manager  }

	{	
	 * SCSI bus status. These values are returned by the SCSI target in the status phase.
	 * They are not related to Macintosh status values (except that values other than
	 * scsiStatusGood will result in scsiResult set to scsiNonZeroStatus).
	 	}
	scsiStatGood				= $00;							{  Good Status }
	scsiStatCheckCondition		= $02;							{  Check Condition }
	scsiStatConditionMet		= $04;							{  Condition Met }
	scsiStatBusy				= $08;							{  Busy }
	scsiStatIntermediate		= $10;							{  Intermediate }
	scsiStatIntermedMet			= $14;							{  Intermediate - Condition Met }
	scsiStatResvConflict		= $18;							{  Reservation conflict }
	scsiStatTerminated			= $22;							{  Command terminated }
	scsiStatQFull				= $28;							{  Queue full }

	{  SCSI messages }
	kCmdCompleteMsg				= 0;
	kExtendedMsg				= 1;							{  0x01 }
	kSaveDataPointerMsg			= 2;							{  0x02 }
	kRestorePointersMsg			= 3;							{  0x03 }
	kDisconnectMsg				= 4;							{  0x04 }
	kInitiatorDetectedErrorMsg	= 5;							{  0x05 }
	kAbortMsg					= 6;							{  0x06 }
	kMsgRejectMsg				= 7;							{  0x07 }
	kNoOperationMsg				= 8;							{  0x08 }
	kMsgParityErrorMsg			= 9;							{  0x09 }
	kLinkedCmdCompleteMsg		= 10;							{  0x0a }
	kLinkedCmdCompleteWithFlagMsg = 11;							{  0x0b }
	kBusDeviceResetMsg			= 12;							{  0x0c }
	kAbortTagMsg				= 13;							{  0x0d }
	kClearQueueMsg				= 14;							{  0x0e }
	kInitiateRecoveryMsg		= 15;							{  0x0f }
	kReleaseRecoveryMsg			= 16;							{  0x10 }
	kTerminateIOProcessMsg		= 17;							{  0x11 }
	kSimpleQueueTag				= $20;							{  0x20 }
	kHeadOfQueueTagMsg			= $21;							{  0x21 }
	kOrderedQueueTagMsg			= $22;							{  0x22 }
	kIgnoreWideResidueMsg		= $23;							{  0x23 }


	{	 moveq #kSCSIx, D0;  _SCSIAtomic 	}
	{
	 *  SCSIAction()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function SCSIAction(var parameterBlock: SCSI_PB): OSErr; external name '_SCSIAction';
{$ifc CALL_NOT_IN_CARBON}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	scsiDataBuffer = 0;    { single contiguous buffer supplied            }
	scsiDataTIB = 1;    { TIB supplied (ptr in scsiDataPtr)           }
	scsiDataSG = 2;    { scatter/gather list supplied             }
	scsiDataIOTable = 3;     {#(7/11/95) Prepared by Block Storage         }

{ scsiDataTypes field bits  }
{  bits 0->15 Apple-defined, 16->30 3rd-party unique, 31 = reserved }
const
	scsiBusDataTIB = 1 shl scsiDataTIB; { TIB supplied (ptr in scsiDataPtr)   }
	scsiBusDataBuffer = 1 shl scsiDataBuffer; { single contiguous buffer supplied      }
	scsiBusDataSG = 1 shl scsiDataSG; { scatter/gather list supplied        }
	scsiBusDataIOTable = 1 shl scsiDataIOTable; { (2/6/95) Prepare Memory for IO}
	scsiBusDataReserved = $80000000; {                            }

{ scsiScanFlags field bits }
const
	scsiBusScansDevices = $80; { Bus scans for and maintains device list         }
	scsiBusScansOnInit = $40; { Bus scans performed at power-up/reboot        }
	scsiBusLoadsROMDrivers = $20;  { may load ROM drivers to support targets       }

{ scsiFeatureFlags field bits }
const
	scsiBusLVD = $00000400; { HBA is Low Voltage Differential Bus         }
	scsiBusUltra3SCSI = $00000200; { HBA supports Ultra3 SCSI               }
	scsiBusUltra2SCSI = $00000100; { HBA supports Ultra2 SCSI               }
	scsiBusInternalExternalMask = $000000C0; { bus internal/external mask           }
	scsiBusInternalExternalUnknown = $00000000; { not known whether bus is inside or outside     }
	scsiBusInternalExternal = $000000C0; { bus goes inside and outside the box       }
	scsiBusInternal = $00000080; { bus goes inside the box                }
	scsiBusExternal = $00000040; { bus goes outside the box             }
	scsiBusCacheCoherentDMA = $00000020; { DMA is cache coherent                }
	scsiBusOldCallCapable = $00000010; { SIM is old call capable                }
	scsiBusUltraSCSI = $00000008; { HBA supports Ultra SCSI                 }
	scsiBusDifferential = $00000004; { Single Ended (0) or Differential (1)    }
	scsiBusFastSCSI = $00000002; { HBA supports fast SCSI                  }
	scsiBusDMAavailable = $00000001; { DMA is available                  }

{ scsiWeirdStuff field bits }
const
	scsiOddDisconnectUnsafeRead1 = $0001; { Disconnects on odd byte boundries are unsafe with DMA and/or blind reads }
	scsiOddDisconnectUnsafeWrite1 = $0002; { Disconnects on odd byte boundries are unsafe with DMA and/or blind writes }
	scsiBusErrorsUnsafe = $0004; { Non-handshaked delays or disconnects during blind transfers may cause a crash }
	scsiRequiresHandshake = $0008; { Non-handshaked delays or disconnects during blind transfers may cause data corruption }
	scsiTargetDrivenSDTRSafe = $0010; { Targets which initiate synchronous negotiations are supported }
	scsiOddCountForPhysicalUnsafe = $0020; { If using physical addrs all counts must be even, and disconnects must be on even boundries }
	scsiAbortCmdFixed = $0040; { Set if abort command is fixed to properly make callbacks }
	scsiMeshACKTimingFixed = $0080; { Set if bug allowing Mesh to release ACK prematurely is fixed }

{ scsiHBAslotType values }
const
	scsiMotherboardBus = $00; { A built in Apple supplied bus            }
	scsiNuBus = $01; { A SIM on a NuBus card                   }
	scsiPDSBus = $03; {    "  on a PDS card                    }
	scsiPCIBus = $04; {    "  on a PCI bus card                   }
	scsiPCMCIABus = $05; {    "  on a PCMCIA card                  }
	scsiFireWireBridgeBus = $06; {    "  connected through a FireWire bridge   }
	scsiUSBBus = $07;  {    "  connected on a USB bus               }

{ Defines for the scsiDriverFlags field (in SCSIDriverPB) }
const
	scsiDeviceSensitive = $0001; { Only driver should access this device          }
	scsiDeviceNoOldCallAccess = $0002; { no old call access to this device            }

>>>>>>> origin/fixes_2.4
{
 * SCSI bus status. These values are returned by the SCSI target in the status phase.
 * They are not related to Macintosh status values (except that values other than
 * scsiStatusGood will result in scsiResult set to scsiNonZeroStatus).
 }
const
	scsiStatGood = $00; { Good Status}
	scsiStatCheckCondition = $02; { Check Condition}
	scsiStatConditionMet = $04; { Condition Met}
	scsiStatBusy = $08; { Busy}
	scsiStatIntermediate = $10; { Intermediate}
	scsiStatIntermedMet = $14; { Intermediate - Condition Met}
	scsiStatResvConflict = $18; { Reservation conflict}
	scsiStatTerminated = $22; { Command terminated}
	scsiStatQFull = $28;  { Queue full}

{ SCSI messages}
const
	kCmdCompleteMsg = 0;
	kExtendedMsg = 1;    { 0x01}
	kSaveDataPointerMsg = 2;    { 0x02}
	kRestorePointersMsg = 3;    { 0x03}
	kDisconnectMsg = 4;    { 0x04}
	kInitiatorDetectedErrorMsg = 5;    { 0x05}
	kAbortMsg = 6;    { 0x06}
	kMsgRejectMsg = 7;    { 0x07}
	kNoOperationMsg = 8;    { 0x08}
	kMsgParityErrorMsg = 9;    { 0x09}
	kLinkedCmdCompleteMsg = 10;   { 0x0a}
	kLinkedCmdCompleteWithFlagMsg = 11;   { 0x0b}
	kBusDeviceResetMsg = 12;   { 0x0c}
	kAbortTagMsg = 13;   { 0x0d}
	kClearQueueMsg = 14;   { 0x0e}
	kInitiateRecoveryMsg = 15;   { 0x0f}
	kReleaseRecoveryMsg = 16;   { 0x10}
	kTerminateIOProcessMsg = 17;   { 0x11}
	kSimpleQueueTag = $20; { 0x20}
	kHeadOfQueueTagMsg = $21; { 0x21}
	kOrderedQueueTagMsg = $22; { 0x22}
	kIgnoreWideResidueMsg = $23;  { 0x23}


{$ifc not TARGET_CPU_64}
{
 *  SCSIAction()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use the SCSITaskUserClient API instead.
 *  
 *  Discussion:
 *    This routine is deprecated. It is exported and callable, but it
 *    is no longer being maintained.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.2
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function SCSIAction( var parameterBlock: SCSI_PB ): OSErr; external name '_SCSIAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)


{$endc} {not TARGET_CPU_64}

{
 *  SCSIRegisterBus()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
<<<<<<< HEAD
function SCSIRegisterBus(var parameterBlock: SIMInitInfo): OSErr; external name '_SCSIRegisterBus';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


>>>>>>> origin/fixes_2.4
{
 *  SCSIDeregisterBus()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
<<<<<<< HEAD
function SCSIDeregisterBus(var parameterBlock: SCSI_PB): OSErr; external name '_SCSIDeregisterBus';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


>>>>>>> origin/fixes_2.4
{
 *  SCSIReregisterBus()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
<<<<<<< HEAD
function SCSIReregisterBus(var parameterBlock: SIMInitInfo): OSErr; external name '_SCSIReregisterBus';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


>>>>>>> origin/fixes_2.4
{
 *  SCSIKillXPT()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }


{$endc} {TARGET_OS_MAC and not TARGET_CPU_64}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }


{$endc} {TARGET_OS_MAC and not TARGET_CPU_64}
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
