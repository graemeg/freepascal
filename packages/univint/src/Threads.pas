{
     File:       CarbonCore/Threads.h
 
     Contains:   Thread Manager Interfaces.
<<<<<<< HEAD
<<<<<<< HEAD
                 The contents of this header file are deprecated.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1991-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1991-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1991-2005 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1991-2008 by Apple Computer, Inc., all rights reserved.
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

unit Threads;
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
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

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{********************************************************************************************
 
 The Thread Manager is deprecated.  Callers should use blocks, libDispatch, or pthreads.
 
 No exact replacement exists for the 'cooperative' threads model, but converting all of the
 former work done in cooperative threads into blocks and scheduling them on the same serial
 dispatch queue ( or the main queue ) is essentially the same.
 
********************************************************************************************}

=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,MixedMode,MacErrors;
=======
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ Thread states}
type
	ThreadState = UInt16;
const
	kReadyThreadState = 0;
	kStoppedThreadState = 1;
	kRunningThreadState = 2;

{ Error codes have been moved to Errors.(pah)}

{ Thread environment characteristics}
type
	ThreadTaskRef = UnivPtr;
{ Thread characteristics}
type
	ThreadStyle = UInt32;
const
	kCooperativeThread = 1 shl 0;
	kPreemptiveThread = 1 shl 1;

{ Thread identifiers}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ThreadID = UNSIGNEDLONG;
=======
	ThreadID = UInt32;
>>>>>>> graemeg/fixes_2_2
=======
	ThreadID = UInt32;
>>>>>>> origin/fixes_2_2
=======
	ThreadID = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kNoThreadID = 0;
	kCurrentThreadID = 1;
	kApplicationThreadID = 2;

{ Options when creating a thread}
type
	ThreadOptions = UInt32;
const
	kNewSuspend = 1 shl 0;
	kUsePremadeThread = 1 shl 1;
	kCreateIfNeeded = 1 shl 2;
	kFPUNotNeeded = 1 shl 3;
	kExactMatchThread = 1 shl 4;

{ Information supplied to the custom scheduler}
type
	SchedulerInfoRec = record
		InfoRecSize: UInt32;
		CurrentThreadID: ThreadID;
		SuggestedThreadID: ThreadID;
		InterruptedCoopThreadID: ThreadID;
	end;
	SchedulerInfoRecPtr = ^SchedulerInfoRec;

{
    The following ProcPtrs cannot be interchanged with UniversalProcPtrs because
    of differences between 680x0 and PowerPC runtime architectures with regard to
    the implementation of the Thread Manager.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	voidPtr = UnivPtr;
=======
	voidPtr = Ptr;
>>>>>>> graemeg/fixes_2_2
=======
	voidPtr = Ptr;
>>>>>>> origin/fixes_2_2
=======
	voidPtr = UnivPtr;
>>>>>>> origin/fixes_2.4
{ Prototype for thread's entry (main) routine}
type
	ThreadEntryProcPtr = function( threadParam: UnivPtr ): voidPtr;
{ Prototype for custom thread scheduler routine}
type
	ThreadSchedulerProcPtr = function( schedulerInfo: SchedulerInfoRecPtr ): ThreadID;
{ Prototype for custom thread switcher routine}
type
	ThreadSwitchProcPtr = procedure( threadBeingSwitched: ThreadID; switchProcParam: UnivPtr );
{ Prototype for thread termination notification routine}
type
	ThreadTerminationProcPtr = procedure( threadTerminated: ThreadID; terminationProcParam: UnivPtr );
{ Prototype for debugger NewThread notification}
type
	DebuggerNewThreadProcPtr = procedure( threadCreated: ThreadID );
{ Prototype for debugger DisposeThread notification}
type
	DebuggerDisposeThreadProcPtr = procedure( threadDeleted: ThreadID );
{ Prototype for debugger schedule notification}
type
	DebuggerThreadSchedulerProcPtr = function( schedulerInfo: SchedulerInfoRecPtr ): ThreadID;
type
	ThreadEntryUPP = ThreadEntryProcPtr;
	ThreadSchedulerUPP = ThreadSchedulerProcPtr;
	ThreadSwitchUPP = ThreadSwitchProcPtr;
	ThreadTerminationUPP = ThreadTerminationProcPtr;
	DebuggerNewThreadUPP = DebuggerNewThreadProcPtr;
	DebuggerDisposeThreadUPP = DebuggerDisposeThreadProcPtr;
	DebuggerThreadSchedulerUPP = DebuggerThreadSchedulerProcPtr;
{
 *  NewThreadEntryUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewThreadEntryUPP( userRoutine: ThreadEntryProcPtr ): ThreadEntryUPP; external name '_NewThreadEntryUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  NewThreadSchedulerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewThreadSchedulerUPP( userRoutine: ThreadSchedulerProcPtr ): ThreadSchedulerUPP; external name '_NewThreadSchedulerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  NewThreadSwitchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewThreadSwitchUPP( userRoutine: ThreadSwitchProcPtr ): ThreadSwitchUPP; external name '_NewThreadSwitchUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  NewThreadTerminationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewThreadTerminationUPP( userRoutine: ThreadTerminationProcPtr ): ThreadTerminationUPP; external name '_NewThreadTerminationUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  NewDebuggerNewThreadUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDebuggerNewThreadUPP( userRoutine: DebuggerNewThreadProcPtr ): DebuggerNewThreadUPP; external name '_NewDebuggerNewThreadUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  NewDebuggerDisposeThreadUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDebuggerDisposeThreadUPP( userRoutine: DebuggerDisposeThreadProcPtr ): DebuggerDisposeThreadUPP; external name '_NewDebuggerDisposeThreadUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  NewDebuggerThreadSchedulerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDebuggerThreadSchedulerUPP( userRoutine: DebuggerThreadSchedulerProcPtr ): DebuggerThreadSchedulerUPP; external name '_NewDebuggerThreadSchedulerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeThreadEntryUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeThreadEntryUPP( userUPP: ThreadEntryUPP ); external name '_DisposeThreadEntryUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeThreadSchedulerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeThreadSchedulerUPP( userUPP: ThreadSchedulerUPP ); external name '_DisposeThreadSchedulerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeThreadSwitchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeThreadSwitchUPP( userUPP: ThreadSwitchUPP ); external name '_DisposeThreadSwitchUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeThreadTerminationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeThreadTerminationUPP( userUPP: ThreadTerminationUPP ); external name '_DisposeThreadTerminationUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeDebuggerNewThreadUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDebuggerNewThreadUPP( userUPP: DebuggerNewThreadUPP ); external name '_DisposeDebuggerNewThreadUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeDebuggerDisposeThreadUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDebuggerDisposeThreadUPP( userUPP: DebuggerDisposeThreadUPP ); external name '_DisposeDebuggerDisposeThreadUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeDebuggerThreadSchedulerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDebuggerThreadSchedulerUPP( userUPP: DebuggerThreadSchedulerUPP ); external name '_DisposeDebuggerThreadSchedulerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeThreadEntryUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeThreadEntryUPP( threadParam: UnivPtr; userUPP: ThreadEntryUPP ): voidPtr; external name '_InvokeThreadEntryUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeThreadSchedulerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeThreadSchedulerUPP( schedulerInfo: SchedulerInfoRecPtr; userUPP: ThreadSchedulerUPP ): ThreadID; external name '_InvokeThreadSchedulerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeThreadSwitchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeThreadSwitchUPP( threadBeingSwitched: ThreadID; switchProcParam: UnivPtr; userUPP: ThreadSwitchUPP ); external name '_InvokeThreadSwitchUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeThreadTerminationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeThreadTerminationUPP( threadTerminated: ThreadID; terminationProcParam: UnivPtr; userUPP: ThreadTerminationUPP ); external name '_InvokeThreadTerminationUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeDebuggerNewThreadUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDebuggerNewThreadUPP( threadCreated: ThreadID; userUPP: DebuggerNewThreadUPP ); external name '_InvokeDebuggerNewThreadUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeDebuggerDisposeThreadUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDebuggerDisposeThreadUPP( threadDeleted: ThreadID; userUPP: DebuggerDisposeThreadUPP ); external name '_InvokeDebuggerDisposeThreadUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeDebuggerThreadSchedulerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDebuggerThreadSchedulerUPP( schedulerInfo: SchedulerInfoRecPtr; userUPP: DebuggerThreadSchedulerUPP ): ThreadID; external name '_InvokeDebuggerThreadSchedulerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
   Thread Manager function pointers (TPP):
   on classic 68k use raw function pointers (same as UPP's)
   on classic PowerPC, use raw function pointers
   on classic PowerPC with OPAQUE_UPP_TYPES=1, use UPP's
   on CFM-68K, use UPP's
   on Carbon, use UPP's
}

{ use UPP's}
type
	ThreadEntryTPP = ThreadEntryUPP;
	ThreadSchedulerTPP = ThreadSchedulerUPP;
	ThreadSwitchTPP = ThreadSwitchUPP;
	ThreadTerminationTPP = ThreadTerminationUPP;
	DebuggerNewThreadTPP = DebuggerNewThreadUPP;
	DebuggerDisposeThreadTPP = DebuggerDisposeThreadUPP;
	DebuggerThreadSchedulerTPP = DebuggerThreadSchedulerUPP;
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  NewThread()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
=======
=======
>>>>>>> origin/fixes_2_2
 *  NewThread()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function NewThread( threadStyle_: ThreadStyle; threadEntry: ThreadEntryTPP; threadParam: UnivPtr; stackSize: Size; options: ThreadOptions; threadResult: UnivPtrPtr { can be NULL }; var threadMade: ThreadID ): OSErr; external name '_NewThread';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetThreadScheduler()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThreadScheduler()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadScheduler( threadScheduler: ThreadSchedulerTPP ): OSErr; external name '_SetThreadScheduler';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetThreadSwitcher()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThreadSwitcher()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadSwitcher( thread: ThreadID; threadSwitcher: ThreadSwitchTPP; switchProcParam: UnivPtr; inOrOut: Boolean ): OSErr; external name '_SetThreadSwitcher';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetThreadTerminator()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThreadTerminator()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadTerminator( thread: ThreadID; threadTerminator: ThreadTerminationTPP; terminationProcParam: UnivPtr ): OSErr; external name '_SetThreadTerminator';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetDebuggerNotificationProcs()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDebuggerNotificationProcs()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetDebuggerNotificationProcs( notifyNewThread: DebuggerNewThreadTPP; notifyDisposeThread: DebuggerDisposeThreadTPP; notifyThreadScheduler: DebuggerThreadSchedulerTPP ): OSErr; external name '_SetDebuggerNotificationProcs';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  CreateThreadPool()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CreateThreadPool()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function CreateThreadPool( threadStyle_: ThreadStyle; numToCreate: SInt16; stackSize: Size ): OSErr; external name '_CreateThreadPool';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  GetDefaultThreadStackSize()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetDefaultThreadStackSize( threadStyle_: ThreadStyle; var stackSize: Size ): OSErr; external name '_GetDefaultThreadStackSize';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  ThreadCurrentStackSpace()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function ThreadCurrentStackSpace( thread: ThreadID; var freeStack: ByteCount ): OSErr; external name '_ThreadCurrentStackSpace';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  DisposeThread()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  GetDefaultThreadStackSize()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function DisposeThread( threadToDump: ThreadID; threadResult: UnivPtr; recycleThread: Boolean ): OSErr; external name '_DisposeThread';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  YieldToThread()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function YieldToThread( suggestedThread: ThreadID ): OSErr; external name '_YieldToThread';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function ThreadCurrentStackSpace( thread: ThreadID; var freeStack: ByteCount ): OSErr; external name '_ThreadCurrentStackSpace';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  YieldToAnyThread()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function YieldToAnyThread: OSErr; external name '_YieldToAnyThread';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  [Mac]GetCurrentThread()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetCurrentThread( var currentThreadID: ThreadID ): OSErr; external name '_GetCurrentThread';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)
function MacGetCurrentThread( var currentThreadID: ThreadID ): OSErr; external name '_GetCurrentThread';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  GetThreadState()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetThreadState( threadToGet: ThreadID; var threadState_: ThreadState ): OSErr; external name '_GetThreadState';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetThreadState()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadState( threadToSet: ThreadID; newState: ThreadState; suggestedThread: ThreadID ): OSErr; external name '_SetThreadState';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetThreadStateEndCritical()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadStateEndCritical( threadToSet: ThreadID; newState: ThreadState; suggestedThread: ThreadID ): OSErr; external name '_SetThreadStateEndCritical';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  ThreadBeginCritical()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function ThreadBeginCritical: OSErr; external name '_ThreadBeginCritical';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  ThreadEndCritical()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function ThreadEndCritical: OSErr; external name '_ThreadEndCritical';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  GetThreadCurrentTaskRef()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetThreadCurrentTaskRef( var threadTRef: ThreadTaskRef ): OSErr; external name '_GetThreadCurrentTaskRef';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  GetThreadStateGivenTaskRef()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetThreadStateGivenTaskRef( threadTRef: ThreadTaskRef; threadToGet: ThreadID; var threadState_: ThreadState ): OSErr; external name '_GetThreadStateGivenTaskRef';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetThreadReadyGivenTaskRef()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadReadyGivenTaskRef( threadTRef: ThreadTaskRef; threadToSet: ThreadID ): OSErr; external name '_SetThreadReadyGivenTaskRef';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)


{$ifc not TARGET_CPU_64}
{ This routine was never implemented on Mac OS X.}
{
 *  GetFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetFreeThreadCount( threadStyle_: ThreadStyle; var freeCount: SInt16 ): OSErr; external name '_GetFreeThreadCount';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)


{ This routine was never implemented on Mac OS X.}
{
 *  GetSpecificFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetSpecificFreeThreadCount( threadStyle_: ThreadStyle; stackSize: Size; var freeCount: SInt16 ): OSErr; external name '_GetSpecificFreeThreadCount';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)


<<<<<<< HEAD
=======
{$ifc not TARGET_CPU_64}
{ This routine was never implemented on Mac OS X.}
{
 *  GetFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetFreeThreadCount( threadStyle_: ThreadStyle; var freeCount: SInt16 ): OSErr; external name '_GetFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{ This routine was never implemented on Mac OS X.}
{
 *  GetSpecificFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetSpecificFreeThreadCount( threadStyle_: ThreadStyle; stackSize: Size; var freeCount: SInt16 ): OSErr; external name '_GetSpecificFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$ifc not TARGET_CPU_64}
{ This routine was never implemented on Mac OS X.}
{
 *  GetFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetFreeThreadCount( threadStyle_: ThreadStyle; var freeCount: SInt16 ): OSErr; external name '_GetFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{ This routine was never implemented on Mac OS X.}
{
 *  GetSpecificFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetSpecificFreeThreadCount( threadStyle_: ThreadStyle; stackSize: Size; var freeCount: SInt16 ): OSErr; external name '_GetSpecificFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$ifc not TARGET_CPU_64}
{ This routine was never implemented on Mac OS X.}
{
 *  GetFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetFreeThreadCount( threadStyle_: ThreadStyle; var freeCount: SInt16 ): OSErr; external name '_GetFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{ This routine was never implemented on Mac OS X.}
{
 *  GetSpecificFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetSpecificFreeThreadCount( threadStyle_: ThreadStyle; stackSize: Size; var freeCount: SInt16 ): OSErr; external name '_GetSpecificFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$ifc not TARGET_CPU_64}
{ This routine was never implemented on Mac OS X.}
{
 *  GetFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetFreeThreadCount( threadStyle_: ThreadStyle; var freeCount: SInt16 ): OSErr; external name '_GetFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{ This routine was never implemented on Mac OS X.}
{
 *  GetSpecificFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetSpecificFreeThreadCount( threadStyle_: ThreadStyle; stackSize: Size; var freeCount: SInt16 ): OSErr; external name '_GetSpecificFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


>>>>>>> origin/cpstrnew
{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDefaultThreadStackSize()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetDefaultThreadStackSize( threadStyle_: ThreadStyle; var stackSize: Size ): OSErr; external name '_GetDefaultThreadStackSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ThreadCurrentStackSpace()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function ThreadCurrentStackSpace( thread: ThreadID; var freeStack: ByteCount ): OSErr; external name '_ThreadCurrentStackSpace';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeThread()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function DisposeThread( threadToDump: ThreadID; threadResult: UnivPtr; recycleThread: Boolean ): OSErr; external name '_DisposeThread';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  YieldToThread()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function YieldToThread( suggestedThread: ThreadID ): OSErr; external name '_YieldToThread';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  YieldToAnyThread()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function YieldToAnyThread: OSErr; external name '_YieldToAnyThread';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]GetCurrentThread()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetCurrentThread( var currentThreadID: ThreadID ): OSErr; external name '_GetCurrentThread';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function MacGetCurrentThread( var currentThreadID: ThreadID ): OSErr; external name '_GetCurrentThread';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThreadState()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetThreadState( threadToGet: ThreadID; var threadState_: ThreadState ): OSErr; external name '_GetThreadState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThreadState()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadState( threadToSet: ThreadID; newState: ThreadState; suggestedThread: ThreadID ): OSErr; external name '_SetThreadState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThreadStateEndCritical()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadStateEndCritical( threadToSet: ThreadID; newState: ThreadState; suggestedThread: ThreadID ): OSErr; external name '_SetThreadStateEndCritical';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ThreadBeginCritical()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function ThreadBeginCritical: OSErr; external name '_ThreadBeginCritical';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ThreadEndCritical()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function ThreadEndCritical: OSErr; external name '_ThreadEndCritical';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThreadCurrentTaskRef()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetThreadCurrentTaskRef( var threadTRef: ThreadTaskRef ): OSErr; external name '_GetThreadCurrentTaskRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThreadStateGivenTaskRef()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetThreadStateGivenTaskRef( threadTRef: ThreadTaskRef; threadToGet: ThreadID; var threadState_: ThreadState ): OSErr; external name '_GetThreadStateGivenTaskRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThreadReadyGivenTaskRef()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function SetThreadReadyGivenTaskRef( threadTRef: ThreadTaskRef; threadToSet: ThreadID ): OSErr; external name '_SetThreadReadyGivenTaskRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{ This routine was never implemented on Mac OS X.}
{
 *  GetFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetFreeThreadCount( threadStyle_: ThreadStyle; var freeCount: SInt16 ): OSErr; external name '_GetFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{ This routine was never implemented on Mac OS X.}
{
 *  GetSpecificFreeThreadCount()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ThreadsLib 1.0 and later
 }
function GetSpecificFreeThreadCount( threadStyle_: ThreadStyle; stackSize: Size; var freeCount: SInt16 ): OSErr; external name '_GetSpecificFreeThreadCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


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
