{	CFRunLoop.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1998-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 1998-2009, Apple Inc. All rights reserved.
}
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
	Copyright (c) 1998-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1998-2009, Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit CFRunLoop;
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,CFBase,CFArray,CFDate,CFString,MacOSXPosix;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{!
	@header CFRunLoop
	CFRunLoops monitor sources of input to a task and dispatch control
	when sources become ready for processing. Examples of input sources
	might include user input devices, network connections, periodic
	or time-delayed events, and asynchronous callbacks. Input sources
	are registered with a run loop, and when a run loop is "run",
	callback functions associated with each source are called when
	the sources have some activity.

	There is one run loop per thread. Each run loop has different
	sets of input sources, called modes, which are named with strings.
	A run loop is run -- in a named mode -- to have it monitor the
	sources that have been registered in that mode, and the run loop
	blocks there until something happens. Examples of modes include
	the default mode, which a process would normally spend most of
	its time in, and a modal panel mode, which might be run when
	a modal panel is up, to restrict the set of input sources that
	are allowed to "fire". This is not to the granularity of, for
	example, what type of user input events are interesting, however.
	That sort of finer-grained granularity is given by UI-level
	frameworks with "get next event matching mask" or similar
	functionality.

	The CFRunLoopSource type is an abstraction of input sources that
	can be put in a run loop. An input source type would normally
	define an API for creating and operating on instances of the type,
	as if it were a separate entity from the run loop, then provide a
	function to create a CFRunLoopSource for an instance. The
	CFRunLoopSource can then be registered with the run loop,
	represents the input source to the run loop, and acts as
	intermediary between the run loop and the actual input source
	type instance. Examples include CFMachPort and CFSocket.

	A CFRunLoopTimer is a specialization of run loop sources, a way
	to generate either a one-shot delayed action, or a recurrent
	action.

	While being run, a run loop goes through a cycle of activities.
	Input sources are checked, timers which need firing are fired,
	and then the run loop blocks, waiting for something to happen 
	(or in the case of timers, waiting for it to be time for
	something to happen). When something does happen, the run loop
	wakes up, processes the activity (usually by calling a callback
	function for an input source), checks other sources, fires timers,
	and goes back to sleep. And so on. CFRunLoopObservers can be
	used to do processing at special points in this cycle.


}


{!
	@typedef CFRunLoopRef
	This is the type of a reference to a run loop.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
	CFRunLoopRef = ^__CFRunLoop; { an opaque type }
	__CFRunLoop = record end;
=======
	CFRunLoopRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFRunLoopRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2

{!
	@typedef CFRunLoopSourceRef
	This is the type of a reference to general run loop input sources.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
	CFRunLoopSourceRef = ^__CFRunLoopSource; { an opaque type }
	__CFRunLoopSource = record end;
=======
	CFRunLoopSourceRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFRunLoopSourceRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2

{!
	@typedef CFRunLoopObserverRef
	This is the type of a reference to a run loop observer.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
	CFRunLoopObserverRef = ^__CFRunLoopObserver; { an opaque type }
	__CFRunLoopObserver = record end;
=======
	CFRunLoopObserverRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFRunLoopObserverRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2

{!
	@typedef CFRunLoopTimerRef
	This is the type of a reference to a run loop timer.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
	CFRunLoopTimerRef = ^__CFRunLoopTimer; { an opaque type }
	__CFRunLoopTimer = record end;
=======
	CFRunLoopTimerRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFRunLoopTimerRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2

{ Reasons for CFRunLoopRunInMode() to Return }
const
	kCFRunLoopRunFinished = 1;
	kCFRunLoopRunStopped = 2;
	kCFRunLoopRunTimedOut = 3;
	kCFRunLoopRunHandledSource = 4;

{ Run Loop Observer Activities }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFRunLoopActivity = CFOptionFlags;
=======
	CFRunLoopActivity = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFRunLoopActivity = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFRunLoopActivity = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFRunLoopActivity = UNSIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	CFRunLoopActivity = UInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFRunLoopActivity = UInt32;
>>>>>>> origin/fixes_2_2
=======
	CFRunLoopActivity = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFRunLoopEntry = 1 shl 0;
    kCFRunLoopBeforeTimers = 1 shl 1;
    kCFRunLoopBeforeSources = 1 shl 2;
    kCFRunLoopBeforeWaiting = 1 shl 5;
    kCFRunLoopAfterWaiting = 1 shl 6;
    kCFRunLoopExit = 1 shl 7;
    kCFRunLoopAllActivities = $0FFFFFFF;

var kCFRunLoopDefaultMode: CFStringRef; external name '_kCFRunLoopDefaultMode'; (* attribute const *)
var kCFRunLoopCommonModes: CFStringRef; external name '_kCFRunLoopCommonModes'; (* attribute const *)

{!
	@function CFRunLoopGetTypeID
	Returns the type identifier of all CFRunLoop instances.
}
function CFRunLoopGetTypeID: CFTypeID; external name '_CFRunLoopGetTypeID';

{!
	@function CFRunLoopGetCurrent
	Returns the run loop for the current thread. There is exactly
	one run loop per thread.
}
function CFRunLoopGetCurrent: CFRunLoopRef; external name '_CFRunLoopGetCurrent';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFRunLoopGetMain: CFRunLoopRef; external name '_CFRunLoopGetMain';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CFRunLoopGetMain: CFRunLoopRef; external name '_CFRunLoopGetMain';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4
{!
	@function CFRunLoopCopyCurrentMode
	Returns the name of the mode in which the run loop is running.
	NULL is returned if the run loop is not running.
	@param rl The run loop for which the current mode should be
		reported.
}
function CFRunLoopCopyCurrentMode( rl: CFRunLoopRef ): CFStringRef; external name '_CFRunLoopCopyCurrentMode';

{!
	@function CFRunLoopCopyAllModes
	Returns an array of all the names of the modes known to the run
	loop.
	@param rl The run loop for which the mode list should be returned.
}
function CFRunLoopCopyAllModes( rl: CFRunLoopRef ): CFArrayRef; external name '_CFRunLoopCopyAllModes';

{!
	@function CFRunLoopAddCommonMode
	Makes the named mode a "common mode" for the run loop. The set of
	common modes are collectively accessed with the global constant
	kCFRunLoopCommonModes. Input sources previously added to the
	common modes are added to the new common mode.
	@param rl The run loop for which the mode should be made common.
	@param mode The name of the mode to mark as a common mode.
}
procedure CFRunLoopAddCommonMode( rl: CFRunLoopRef; mode: CFStringRef ); external name '_CFRunLoopAddCommonMode';

{!
	@function CFRunLoopGetNextTimerFireDate
	Returns the time at which the next timer will fire.
	@param rl The run loop for which the next timer fire date should
		be reported.
	@param mode The name of the mode to query.
}
function CFRunLoopGetNextTimerFireDate( rl: CFRunLoopRef; mode: CFStringRef ): CFAbsoluteTime; external name '_CFRunLoopGetNextTimerFireDate';

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
procedure CFRunLoopRun; external name '_CFRunLoopRun';
function CFRunLoopRunInMode( mode: CFStringRef; seconds: CFTimeInterval; returnAfterSourceHandled: Boolean ): SInt32; external name '_CFRunLoopRunInMode';
function CFRunLoopIsWaiting( rl: CFRunLoopRef ): Boolean; external name '_CFRunLoopIsWaiting';
procedure CFRunLoopWakeUp( rl: CFRunLoopRef ); external name '_CFRunLoopWakeUp';
procedure CFRunLoopStop( rl: CFRunLoopRef ); external name '_CFRunLoopStop';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{#if __BLOCKS__ && MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{ requires support for blocks
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
extern void CFRunLoopPerformBlock(CFRunLoopRef rl, CFTypeRef mode, void (^block)(void)) CF_AVAILABLE_STARTING(10_6, 4_0); 
=======
extern void CFRunLoopPerformBlock(CFRunLoopRef rl, CFTypeRef mode, void (^block)(void)) AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER; 
>>>>>>> graemeg/cpstrnew
=======
extern void CFRunLoopPerformBlock(CFRunLoopRef rl, CFTypeRef mode, void (^block)(void)) AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER; 
>>>>>>> graemeg/cpstrnew
=======
extern void CFRunLoopPerformBlock(CFRunLoopRef rl, CFTypeRef mode, void (^block)(void)) AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER; 
>>>>>>> graemeg/cpstrnew
=======
extern void CFRunLoopPerformBlock(CFRunLoopRef rl, CFTypeRef mode, void (^block)(void)) AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER; 
>>>>>>> origin/cpstrnew
}
{#endif}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{#if __BLOCKS__ && MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{ requires support for blocks
extern void CFRunLoopPerformBlock(CFRunLoopRef rl, CFTypeRef mode, void (^block)(void)) AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER; 
}
{#endif}

>>>>>>> origin/fixes_2.4
function CFRunLoopContainsSource( rl: CFRunLoopRef; source: CFRunLoopSourceRef; mode: CFStringRef ): Boolean; external name '_CFRunLoopContainsSource';
procedure CFRunLoopAddSource( rl: CFRunLoopRef; source: CFRunLoopSourceRef; mode: CFStringRef ); external name '_CFRunLoopAddSource';
procedure CFRunLoopRemoveSource( rl: CFRunLoopRef; source: CFRunLoopSourceRef; mode: CFStringRef ); external name '_CFRunLoopRemoveSource';

function CFRunLoopContainsObserver( rl: CFRunLoopRef; observer: CFRunLoopObserverRef; mode: CFStringRef ): Boolean; external name '_CFRunLoopContainsObserver';
procedure CFRunLoopAddObserver( rl: CFRunLoopRef; observer: CFRunLoopObserverRef; mode: CFStringRef ); external name '_CFRunLoopAddObserver';
procedure CFRunLoopRemoveObserver( rl: CFRunLoopRef; observer: CFRunLoopObserverRef; mode: CFStringRef ); external name '_CFRunLoopRemoveObserver';

function CFRunLoopContainsTimer( rl: CFRunLoopRef; timer: CFRunLoopTimerRef; mode: CFStringRef ): Boolean; external name '_CFRunLoopContainsTimer';
procedure CFRunLoopAddTimer( rl: CFRunLoopRef; timer: CFRunLoopTimerRef; mode: CFStringRef ); external name '_CFRunLoopAddTimer';
procedure CFRunLoopRemoveTimer( rl: CFRunLoopRef; timer: CFRunLoopTimerRef; mode: CFStringRef ); external name '_CFRunLoopRemoveTimer';

{!
	@typedef CFRunLoopSourceContext
	Structure containing the callbacks of a CFRunLoopSource.
	@field version The version number of the structure type being
		passed in as a parameter to the CFArray creation
		functions. Valid version numbers are currently 0 and 1.
		Version 0 sources are fairly generic, but may require a
		bit more implementation, or may require a separate
		thread as part of the implementation, for a complex
		source. Version 1 sources are available on Mach and Windows,
		and have performance advantages when the source type can
		be described with this style.
	@field info An arbitrary pointer to client-defined data, which
		can be associated with the source at creation time, and
		is passed to the callbacks.
	@field retain The callback used to add a retain for the source on
		the info pointer for the life of the source, and may be
		used for temporary references the source needs to take.
		This callback returns the actual info pointer to store in
		the source, almost always just the pointer passed as the
		parameter.
	@field release The callback used to remove a retain previously
		added for the source on the info pointer. 
	@field copyDescription The callback used to create a descriptive
		string representation of the info pointer (or the data
		pointed to by the info pointer) for debugging purposes.
		This is used by the CFCopyDescription() function.
	@field equal The callback used to compare the info pointers of
		two sources, to determine equality of sources.
	@field hash The callback used to compute a hash code for the info
		pointer for the source. The source uses this hash code
		information to produce its own hash code.
	@field schedule For a version 0 source, this callback is called
		whenever the source is added to a run loop mode. This
		information is often needed to implement complex sources.
	@field cancel For a version 0 source, this callback is called
		whenever the source is removed from a run loop mode. This
		information is often needed to implement complex sources.
	@field getPort Defined in version 1 sources, this function returns
		the Mach port or Windows HANDLE of a kernel object to
		represent the source to the run loop.  This function
		must return the same result every time it is called, for the
                lifetime of the source, and should be quick.
	@field perform This callback is the workhorse of a run loop source.
		It is called when the source needs to be "handled" or
		processing is needed for input or conditions relating to
		the source. For version 0 sources, this function is called
		when the source has been marked "signaled" with the
		CFRunLoopSourceSignal() function, and should do whatever
		handling is required for the source. For a version 1 source
		on Mach, this function is called when a Mach message arrives
		on the source's Mach port, with the message, its
		length, an allocator, and the source's info pointer. A
		version 1 source performs whatever processing is required
		on the Mach message, then can return a pointer to a Mach
		message (or NULL if none) to be sent (usually this is a
		"reply" message), which should be allocated with the
		allocator (and will be deallocated by the run loop after
		sending).  For a version 1 source on Windows the function
		is called when the kernel object is in the signaled state.
}
type
	CFRunLoopSourceContext = record
		version: CFIndex;
		info: UnivPtr;
		retain: function( info: {const} UnivPtr ): UnivPtr;
		release: procedure( info: {const} UnivPtr );
		copyDescription: function( info: {const} UnivPtr ): CFStringRef;
		equal: function( info1: {const} UnivPtr; info2: {const} UnivPtr ): Boolean;
		hash: function( info: {const} UnivPtr ): CFHashCode;
		schedule: function( info: {const} UnivPtr; rl: CFRunLoopRef; mode: CFStringRef ): CFHashCode;
		cancel: function( info: {const} UnivPtr; rl: CFRunLoopRef; mode: CFStringRef ): CFHashCode;
		perform: procedure( info: {const} UnivPtr );
	end;

type
	CFRunLoopSourceContext1 = record
		version: CFIndex;
		info: UnivPtr;
		retain: function( info: {const} UnivPtr ): UnivPtr;
		release: procedure( info: {const} UnivPtr );
		copyDescription: function( info: {const} UnivPtr ): CFStringRef;
		equal: function( info1: {const} UnivPtr; info2: {const} UnivPtr ): Boolean;
		hash: function( info: {const} UnivPtr ): CFHashCode;
		getPort: function( info: {const} UnivPtr ): mach_port_t;
		perform: function( msg: UnivPtr; size: CFIndex; allocator: CFAllocatorRef; info: UnivPtr ): UnivPtr;
	end;

{!
	@function CFRunLoopSourceGetTypeID
	Returns the type identifier of all CFRunLoopSource instances.
}
function CFRunLoopSourceGetTypeID: CFTypeID; external name '_CFRunLoopSourceGetTypeID';

{!
	@function CFRunLoopSourceCreate
	Creates a new run loop source with the given context.
	@param allocator The CFAllocator which should be used to allocate
		memory for the array and its storage for values. If this
		reference is not a valid CFAllocator, the behavior is
		undefined.
	@param order On platforms which support it, for source versions
		which support it, this parameter determines the order in
		which the sources which are ready to be processed are
		handled. A lower order number causes processing before
		higher order number sources. It is inadvisable to depend
		on the order number for any architectural or design aspect
		of code. In the absence of any reason to do otherwise,
		zero should be used.
	@param context A pointer to the context structure for the source.
}
function CFRunLoopSourceCreate( allocator: CFAllocatorRef; order: CFIndex; var context: CFRunLoopSourceContext ): CFRunLoopSourceRef; external name '_CFRunLoopSourceCreate';

{!
	@function CFRunLoopSourceGetOrder
	Returns the ordering parameter of the run loop source.
	@param source The run loop source for which the order number
		should be returned.
}
function CFRunLoopSourceGetOrder( source: CFRunLoopSourceRef ): CFIndex; external name '_CFRunLoopSourceGetOrder';

{!
	@function CFRunLoopSourceInvalidate
	Invalidates the run loop source. The run loop source is never
	performed again after it becomes invalid, and will automatically
	be removed from any run loops and modes which contain it. The
	source is not destroyed by this operation, however -- the memory
	is still valid; only the release of all references on the source
	through the reference counting system can do that. But note, that
	if the only retains on the source were held by run loops, those
	retains may all be released by the time this function returns,
	and the source may actually be destroyed through that process.
	@param source The run loop source which should be invalidated.
}
procedure CFRunLoopSourceInvalidate( source: CFRunLoopSourceRef ); external name '_CFRunLoopSourceInvalidate';

{!
	@function CFRunLoopSourceIsValid
	Reports whether or not the source is valid.
	@param source The run loop source for which the validity should
		be returned.
}
function CFRunLoopSourceIsValid( source: CFRunLoopSourceRef ): Boolean; external name '_CFRunLoopSourceIsValid';

{!
	@function CFRunLoopSourceGetContext
	Fills the memory pointed to by the context parameter with the
	context structure of the source.
	@param source The run loop source for which the context structure
		should be returned.
	@param context A pointer to a context structure to be filled.
}
procedure CFRunLoopSourceGetContext( source: CFRunLoopSourceRef; var context: CFRunLoopSourceContext ); external name '_CFRunLoopSourceGetContext';

{!
	@function CFRunLoopSourceSignal
	Marks the source as signalled, ready for handling by the run loop.
	Has no effect on version 1 sources, which are automatically
	handled when Mach messages for them come in.
	@param source The run loop source which should be signalled.
}
procedure CFRunLoopSourceSignal( source: CFRunLoopSourceRef ); external name '_CFRunLoopSourceSignal';

type
	CFRunLoopObserverContext = record
		version: CFIndex;
		info: UnivPtr;
		retain: function( info: {const} UnivPtr ): UnivPtr;
		release: procedure( info: {const} UnivPtr );
		copyDescription: function( info: {const} UnivPtr ): CFStringRef;
	end;

type
	CFRunLoopObserverCallBack = procedure( observer: CFRunLoopObserverRef; activity: CFRunLoopActivity; info: UnivPtr );

{!
	@function CFRunLoopObserverGetTypeID
	Returns the type identifier of all CFRunLoopObserver instances.
}
function CFRunLoopObserverGetTypeID: CFTypeID; external name '_CFRunLoopObserverGetTypeID';

function CFRunLoopObserverCreate( allocator: CFAllocatorRef; activities: CFOptionFlags; repeats: Boolean; order: CFIndex; callout: CFRunLoopObserverCallBack; var context: CFRunLoopObserverContext ): CFRunLoopObserverRef; external name '_CFRunLoopObserverCreate';

function CFRunLoopObserverGetActivities( observer: CFRunLoopObserverRef ): CFOptionFlags; external name '_CFRunLoopObserverGetActivities';
function CFRunLoopObserverDoesRepeat( observer: CFRunLoopObserverRef ): Boolean; external name '_CFRunLoopObserverDoesRepeat';
function CFRunLoopObserverGetOrder( observer: CFRunLoopObserverRef ): CFIndex; external name '_CFRunLoopObserverGetOrder';
procedure CFRunLoopObserverInvalidate( observer: CFRunLoopObserverRef ); external name '_CFRunLoopObserverInvalidate';
function CFRunLoopObserverIsValid( observer: CFRunLoopObserverRef ): Boolean; external name '_CFRunLoopObserverIsValid';
procedure CFRunLoopObserverGetContext( observer: CFRunLoopObserverRef; var context: CFRunLoopObserverContext ); external name '_CFRunLoopObserverGetContext';

type
	CFRunLoopTimerContext = record
		version: CFIndex;
		info: UnivPtr;
		retain: function( info: {const} UnivPtr ): UnivPtr;
		release: procedure( info: {const} UnivPtr );
		copyDescription: function( info: {const} UnivPtr ): CFStringRef;
	end;

type
	CFRunLoopTimerCallBack = procedure( timer: CFRunLoopTimerRef; info: UnivPtr );

{!
	@function CFRunLoopTimerGetTypeID
	Returns the type identifier of all CFRunLoopTimer instances.
}
function CFRunLoopTimerGetTypeID: CFTypeID; external name '_CFRunLoopTimerGetTypeID';

function CFRunLoopTimerCreate( allocator: CFAllocatorRef; fireDate: CFAbsoluteTime; interval: CFTimeInterval; flags: CFOptionFlags; order: CFIndex; callout: CFRunLoopTimerCallBack; var context: CFRunLoopTimerContext ): CFRunLoopTimerRef; external name '_CFRunLoopTimerCreate';
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef __BLOCKS__}
extern CFRunLoopTimerRef CFRunLoopTimerCreateWithHandler(CFAllocatorRef allocator, CFAbsoluteTime fireDate, CFTimeInterval interval, CFOptionFlags flags, CFIndex order, void (^block) (CFRunLoopTimerRef timer)) CF_AVAILABLE_STARTING(10_7, 5_0);
{$endif} { __BLOCKS__ }

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function CFRunLoopTimerGetNextFireDate( timer: CFRunLoopTimerRef ): CFAbsoluteTime; external name '_CFRunLoopTimerGetNextFireDate';
procedure CFRunLoopTimerSetNextFireDate( timer: CFRunLoopTimerRef; fireDate: CFAbsoluteTime ); external name '_CFRunLoopTimerSetNextFireDate';
function CFRunLoopTimerGetInterval( timer: CFRunLoopTimerRef ): CFTimeInterval; external name '_CFRunLoopTimerGetInterval';
function CFRunLoopTimerDoesRepeat( timer: CFRunLoopTimerRef ): Boolean; external name '_CFRunLoopTimerDoesRepeat';
function CFRunLoopTimerGetOrder( timer: CFRunLoopTimerRef ): CFIndex; external name '_CFRunLoopTimerGetOrder';
procedure CFRunLoopTimerInvalidate( timer: CFRunLoopTimerRef ); external name '_CFRunLoopTimerInvalidate';
function CFRunLoopTimerIsValid( timer: CFRunLoopTimerRef ): Boolean; external name '_CFRunLoopTimerIsValid';
procedure CFRunLoopTimerGetContext( timer: CFRunLoopTimerRef; var context: CFRunLoopTimerContext ); external name '_CFRunLoopTimerGetContext';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
Setting a tolerance for a timer allows it to fire later than the scheduled fire date, improving the ability of the system to optimize for increased power savings and responsiveness. The timer may fire at any time between its scheduled fire date and the scheduled fire date plus the tolerance. The timer will not fire before the scheduled fire date. For repeating timers, the next fire date is calculated from the original fire date regardless of tolerance applied at individual fire times, to avoid drift. The default value is zero, which means no additional tolerance is applied. The system reserves the right to apply a small amount of tolerance to certain timers regardless of the value of this property.

 As the user of the timer, you will have the best idea of what an appropriate tolerance for a timer may be. A general rule of thumb, though, is to set the tolerance to at least 10% of the interval, for a repeating timer. Even a small amount of tolerance will have a significant positive impact on the power usage of your application. The system may put a maximum value of the tolerance.
}
function CFRunLoopTimerGetTolerance( timer: CFRunLoopTimerRef ): CFTimeInterval; external name '_CFRunLoopTimerGetTolerance';
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)
procedure CFRunLoopTimerSetTolerance( timer: CFRunLoopTimerRef; tolerance: CFTimeInterval ); external name '_CFRunLoopTimerSetTolerance';
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======

end.
>>>>>>> graemeg/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
