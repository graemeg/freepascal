{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/Timer.h
 
     Contains:   Time Manager interfaces.
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
     File:       Timer.p
=======
     File:       CarbonCore/Timer.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Time Manager interfaces.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1985-2002 by Apple Computer, Inc., all rights reserved
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

unit Timer;
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes,ConditionalMacros,OSUtils;
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

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/cpstrnew

=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


{
 *  Microseconds()
 *  
 *  Summary:
 *    Determines the number of microseconds that have elapsed since
 *    system startup time.
 *  
 *  Discussion:
 *    Return a value representing the number of microseconds since some
 *    point in time, usually since the system was booted.  One
 *    microsecond is 1 * 10^-6 seconds, and so there are one million (
 *    1,000,000 ) microseconds per second.  For reference, in one
 *    microsecond light can travel about 850 feet in a vacuum.
 *    
 *    Microseconds() doesn't necessarily advance while the computer is
 *    asleep, so it should not be used for long duration timings.
 *  
 *  Parameters:
 *    
 *    microTickCount:
 *      The number of microseconds elapsed since system startup.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure Microseconds( var microTickCount: UnsignedWide ); external name '_Microseconds';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{***************************************************************************

    The remaining functions in this file have all been deprecated on Mac
    OS X 10.4.  There are other solutions which perform better, and consume
    fewer system resources which are recommended.  The Time Manager on
    Mac OS X does not have exactly the same behavior as it did on Mac OS 9
    and earlier, especially in a multithreaded process.
    
    Instead of using the Time Manager functions, you should consider
    the following:

=======
=======
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{***************************************************************************
<<<<<<< HEAD

=======
=======
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
>>>>>>> origin/cpstrnew

    The remaining functions in this file have all been deprecated on Mac
    OS X 10.4.  There are other solutions which perform better, and consume
    fewer system resources which are recommended.  The Time Manager on
    Mac OS X does not have exactly the same behavior as it did on Mac OS 9
    and earlier, especially in a multithreaded process.
    
    Instead of using the Time Manager functions, you should consider
    the following:

    1.  If you want a function to be called periodically from an
        application, then look at using a CFRunLoopTimer ( in CFRunLoop.h ).

    2.  In a Cocoa application, you can use the NSTimer object to get
        both absolute and interval-based periodic callbacks.
    
    3.  If you need something to happen periodically, and don't have a
        CFRunLoop in your application, you can create a thread which sleeps
        for whatever interval you want, then does something and sleeps again
        ( or exits, or whatever. )
        Conceptually, this is all the current Time Manager does -- when you
        prime a task, a thread is created which sleeps on a semaphore for the
        amount of time remaining before the next earliest Time Manager task,
        then calls the tmTask function for that task and then calculates the
        time until the next task, ad nauseum.
        
    4. If you just want to delay for some period of time, and don't have
        other threads or data which may require synchronization, you can call
        Delay(), sleep(), usleep(), or nanosleep().
        
    5. 
    - 
 ***************************************************************************}
const
{ high bit of qType is set if task is active }
	kTMTaskActive = 1 shl 15;

{***************************************************************************

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    The remaining functions in this file have all been deprecated on Mac
    OS X 10.4.  There are other solutions which perform better, and consume
    fewer system resources which are recommended.  The Time Manager on
    Mac OS X does not have exactly the same behavior as it did on Mac OS 9
    and earlier, especially in a multithreaded process.
    
    Instead of using the Time Manager functions, you should consider
    the following:

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    1.  If you want a function to be called periodically from an
        application, then look at using a CFRunLoopTimer ( in CFRunLoop.h ).

    2.  In a Cocoa application, you can use the NSTimer object to get
        both absolute and interval-based periodic callbacks.
    
    3.  If you need something to happen periodically, and don't have a
        CFRunLoop in your application, you can create a thread which sleeps
        for whatever interval you want, then does something and sleeps again
        ( or exits, or whatever. )
        Conceptually, this is all the current Time Manager does -- when you
        prime a task, a thread is created which sleeps on a semaphore for the
        amount of time remaining before the next earliest Time Manager task,
        then calls the tmTask function for that task and then calculates the
        time until the next task, ad nauseum.
        
    4. If you just want to delay for some period of time, and don't have
        other threads or data which may require synchronization, you can call
        Delay(), sleep(), usleep(), or nanosleep().
        
    5. 
    - 
 ***************************************************************************}
const
{ high bit of qType is set if task is active }
	kTMTaskActive = 1 shl 15;

type
	TMTaskPtr = ^TMTask;
	TimerProcPtr = procedure( tmTaskPtr_: TMTaskPtr );
	TimerUPP = TimerProcPtr;
	TMTask = record
		qLink: QElemPtr;
		qType: SInt16;
		tmAddr: TimerUPP;
		tmCount: SIGNEDLONG;
		tmWakeUp: SIGNEDLONG;
		tmReserved: SIGNEDLONG;
	end;

{
 *  InsTime()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use InstallTimeTask() instead of this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsTime( tmTaskPtr: QElemPtr ); external name '_InsTime';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  InsXTime()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use InstallXTimeTask() instead of this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsXTime( tmTaskPtr: QElemPtr ); external name '_InsXTime';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PrimeTime()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PrimeTimeTask() instead of this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure PrimeTime( tmTaskPtr: QElemPtr; count: SIGNEDLONG ); external name '_PrimeTime';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  RmvTime()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use RemoveTimeTask() instead of this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure RmvTime( tmTaskPtr: QElemPtr ); external name '_RmvTime';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  InstallTimeTask()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Installs a task, taking advantage of the drift-free,
 *    fixed-frequency timing services of the extended Time Manager.
 *  
 *  Discussion:
 *    The InstallXTimeTask function adds the Time Manager task
 *    structure specified by tmTaskPtr to the Time Manager queue. Use
 *    InstallXTimeTask only if you wish to use the drift-free,
 *    fixed-frequency timing services of the extended Time Manager; use
 *    InstallTimeTask in all other cases. The tmTaskPtr parameter must
 *    point to an extended Time Manager task structure. Your
 *    application must fill in the tmAddr field of that task. You
 *    should set the tmWakeUp and tmReserved fields to 0 the first time
 *    you call InsXTime.
 *    With the extended Time Manager, you can set tmAddr to NULL if you
 *    do not want a task to execute when the delay passed to PrimeTime
 *    expires. Also, InsXTime resets the high-order bit of the qType
 *    field to 0.
 *    The InstallXTimeTask function, which returns a value of type
 *    OSErr, takes the place of InsXTime.
 *  
 *  Parameters:
 *    
 *    tmTaskPtr:
 *      A pointer to an extended task structure to be installed in the
 *      queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function InstallTimeTask( tmTaskPtr: QElemPtr ): OSErr; external name '_InstallTimeTask';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  InstallXTimeTask()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Installs a task, taking advantage of the drift-free,
 *    fixed-frequency timing services of the extended Time Manager.
 *  
 *  Discussion:
 *    The InstallXTimeTask function adds the Time Manager task
 *    structure specified by tmTaskPtr to the Time Manager queue. Use
 *    InstallXTimeTask only if you wish to use the drift-free,
 *    fixed-frequency timing services of the extended Time Manager; use
 *    InstallTimeTask in all other cases. The tmTaskPtr parameter must
 *    point to an extended Time Manager task structure. Your
 *    application must fill in the tmAddr field of that task. You
 *    should set the tmWakeUp and tmReserved fields to 0 the first time
 *    you call InsXTime.
 *    With the extended Time Manager, you can set tmAddr to NULL if you
 *    do not want a task to execute when the delay passed to PrimeTime
 *    expires. Also, InsXTime resets the high-order bit of the qType
 *    field to 0.
 *    The InstallXTimeTask function, which returns a value of type
 *    OSErr, takes the place of InsXTime.
 *    
 *    
 *    
 *    
 *    
 *    
 *    *************************************************************
 *     See the discussion at the top of this file for information about
 *    why this call is deprecated on Mac OS X, and what other system
 *    facilities you can use in your code instead of the Time Manager.
 *    *****************************************************************
 *  
 *  Parameters:
 *    
 *    tmTaskPtr:
 *      A pointer to an extended task structure to be installed in the
 *      queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function InstallXTimeTask( tmTaskPtr: QElemPtr ): OSErr; external name '_InstallXTimeTask';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PrimeTimeTask()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Activates a task in the Time Manager queue.
 *  
 *  Discussion:
 *    The PrimeTimeTask function schedules the task specified by the
 *    tmAddr field of the structure pointed to by the tmTaskPtr
 *    parameter for execution after the delay specified by the count
 *    parameter has elapsed.
 *    
 *    If the count parameter is a positive value, it is interpreted as
 *    milliseconds. If count is a negative value, it is interpreted in
 *    negated microseconds. Microsecond delays are allowable only in
 *    the revised and extended Time Managers.
 *    
 *    The task record specified by the tmTaskPtr parameter must already
 *    be installed in the queue (by a previous call to the functions
 *    InstallTimeTask or InstallXTimeTask) before your application
 *    calls the PrimeTimeTask function. The PrimeTimeTask function
 *    returns immediately, and the specified task is executed after the
 *    specified delay has elapsed. If you call the PrimeTimeTask
 *    function with a time delay of 0, the task runs as soon as
 *    interrupts are enabled.
 *    
 *    In the revised and extended Time Managers, the PrimeTimeTask
 *    function sets the high-order bit of the qType field to 1. In
 *    addition, any value of the count parameter that exceeds the
 *    maximum millisecond delay is reduced to the maximum. If you stop
 *    an unexpired task (by calling the function RemoveTimeTask) and
 *    then reinstall it (by calling the InstallXTimeTask function), you
 *    can continue the previous delay by calling the PrimeTimeTask
 *    function with the count parameter set to 0.
 *    
 *    
 *    
 *    
 *    
 *    
 *    *************************************************************
 *     See the discussion at the top of this file for information about
 *    why this call is deprecated on Mac OS X, and what other system
 *    facilities you can use in your code instead of the Time Manager.
 *    *****************************************************************
 *  
 *  Parameters:
 *    
 *    tmTaskPtr:
 *      A pointer to a task structure already installed in the queue.
 *    
 *    count:
 *      The desired delay before execution of the task.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function PrimeTimeTask( tmTaskPtr: QElemPtr; count: SIGNEDLONG ): OSErr; external name '_PrimeTimeTask';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  RemoveTimeTask()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Removes a task from the Time Manager queue.
 *  
 *  Discussion:
 *    The RemoveTimeTask function removes the Time Manager task
 *    structure specified by the tmTaskPtr parameter from the Time
 *    Manager queue. In both the revised and extended Time Managers, if
 *    the specified task record is active (that is, if it has been
 *    activated but the specified time has not yet elapsed), the
 *    tmCount field of the task structure returns the amount of time
 *    remaining. To provide the greatest accuracy, the unused time is
 *    reported as negated microseconds if that value is small enough to
 *    fit into the tmCount field (even if the delay was originally
 *    specified in milliseconds); otherwise, the unused time is
 *    reported in positive milliseconds. If the time has already
 *    expired, the tmCount field contains 0.
 *    
 *    In the revised and extended Time Managers, the RemoveTimeTask
 *    function sets the high-order bit of the qType field to 0.
 *     
 *    
 *    
 *    
 *    
 *    
 *    *************************************************************
 *     See the discussion at the top of this file for information about
 *    why this call is deprecated on Mac OS X, and what other system
 *    facilities you can use in your code instead of the Time Manager.
 *    *****************************************************************
 *  
 *  Parameters:
 *    
 *    tmTaskPtr:
 *      A pointer to a task structure to be removed from the queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function RemoveTimeTask( tmTaskPtr: QElemPtr ): OSErr; external name '_RemoveTimeTask';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  NewTimerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTimerUPP( userRoutine: TimerProcPtr ): TimerUPP; external name '_NewTimerUPP';
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
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


{
 *  Microseconds()
 *  
 *  Summary:
 *    Determines the number of microseconds that have elapsed since
 *    system startup time.
 *  
 *  Discussion:
 *    Return a value representing the number of microseconds since some
 *    point in time, usually since the system was booted.  One
 *    microsecond is 1 * 10^-6 seconds, and so there are one million (
 *    1,000,000 ) microseconds per second.  For reference, in one
 *    microsecond light can travel about 850 feet in a vacuum.
 *    
 *    Microseconds() doesn't necessarily advance while the computer is
 *    asleep, so it should not be used for long duration timings.
 *  
 *  Parameters:
 *    
 *    microTickCount:
 *      The number of microseconds elapsed since system startup.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure Microseconds( var microTickCount: UnsignedWide ); external name '_Microseconds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{***************************************************************************

    The remaining functions in this file have all been deprecated on Mac
    OS X 10.4.  There are other solutions which perform better, and consume
    fewer system resources which are recommended.  The Time Manager on
    Mac OS X does not have exactly the same behavior as it did on Mac OS 9
    and earlier, especially in a multithreaded process.
    
    Instead of using the Time Manager functions, you should consider
    the following:

    1.  If you want a function to be called periodically from an
        application, then look at using a CFRunLoopTimer ( in CFRunLoop.h ).

    2.  In a Cocoa application, you can use the NSTimer object to get
        both absolute and interval-based periodic callbacks.
    
    3.  If you need something to happen periodically, and don't have a
        CFRunLoop in your application, you can create a thread which sleeps
        for whatever interval you want, then does something and sleeps again
        ( or exits, or whatever. )
        Conceptually, this is all the current Time Manager does -- when you
        prime a task, a thread is created which sleeps on a semaphore for the
        amount of time remaining before the next earliest Time Manager task,
        then calls the tmTask function for that task and then calculates the
        time until the next task, ad nauseum.
        
    4. If you just want to delay for some period of time, and don't have
        other threads or data which may require synchronization, you can call
        Delay(), sleep(), usleep(), or nanosleep().
        
    5. 
    - 
 ***************************************************************************}
const
{ high bit of qType is set if task is active }
	kTMTaskActive = 1 shl 15;

type
	TMTaskPtr = ^TMTask;
	TimerProcPtr = procedure( tmTaskPtr_: TMTaskPtr );
	TimerUPP = TimerProcPtr;
	TMTask = record
		qLink: QElemPtr;
		qType: SInt16;
		tmAddr: TimerUPP;
		tmCount: SIGNEDLONG;
		tmWakeUp: SIGNEDLONG;
		tmReserved: SIGNEDLONG;
	end;

{
 *  InsTime()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use InstallTimeTask() instead of this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsTime( tmTaskPtr: QElemPtr ); external name '_InsTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  InsXTime()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use InstallXTimeTask() instead of this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsXTime( tmTaskPtr: QElemPtr ); external name '_InsXTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PrimeTime()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PrimeTimeTask() instead of this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure PrimeTime( tmTaskPtr: QElemPtr; count: SIGNEDLONG ); external name '_PrimeTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RmvTime()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use RemoveTimeTask() instead of this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure RmvTime( tmTaskPtr: QElemPtr ); external name '_RmvTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  InstallTimeTask()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Installs a task, taking advantage of the drift-free,
 *    fixed-frequency timing services of the extended Time Manager.
 *  
 *  Discussion:
 *    The InstallXTimeTask function adds the Time Manager task
 *    structure specified by tmTaskPtr to the Time Manager queue. Use
 *    InstallXTimeTask only if you wish to use the drift-free,
 *    fixed-frequency timing services of the extended Time Manager; use
 *    InstallTimeTask in all other cases. The tmTaskPtr parameter must
 *    point to an extended Time Manager task structure. Your
 *    application must fill in the tmAddr field of that task. You
 *    should set the tmWakeUp and tmReserved fields to 0 the first time
 *    you call InsXTime.
 *    With the extended Time Manager, you can set tmAddr to NULL if you
 *    do not want a task to execute when the delay passed to PrimeTime
 *    expires. Also, InsXTime resets the high-order bit of the qType
 *    field to 0.
 *    The InstallXTimeTask function, which returns a value of type
 *    OSErr, takes the place of InsXTime.
 *  
 *  Parameters:
 *    
 *    tmTaskPtr:
 *      A pointer to an extended task structure to be installed in the
 *      queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function InstallTimeTask( tmTaskPtr: QElemPtr ): OSErr; external name '_InstallTimeTask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  InstallXTimeTask()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Installs a task, taking advantage of the drift-free,
 *    fixed-frequency timing services of the extended Time Manager.
 *  
 *  Discussion:
 *    The InstallXTimeTask function adds the Time Manager task
 *    structure specified by tmTaskPtr to the Time Manager queue. Use
 *    InstallXTimeTask only if you wish to use the drift-free,
 *    fixed-frequency timing services of the extended Time Manager; use
 *    InstallTimeTask in all other cases. The tmTaskPtr parameter must
 *    point to an extended Time Manager task structure. Your
 *    application must fill in the tmAddr field of that task. You
 *    should set the tmWakeUp and tmReserved fields to 0 the first time
 *    you call InsXTime.
 *    With the extended Time Manager, you can set tmAddr to NULL if you
 *    do not want a task to execute when the delay passed to PrimeTime
 *    expires. Also, InsXTime resets the high-order bit of the qType
 *    field to 0.
 *    The InstallXTimeTask function, which returns a value of type
 *    OSErr, takes the place of InsXTime.
 *    
 *    
 *    
 *    
 *    
 *    
 *    *************************************************************
 *     See the discussion at the top of this file for information about
 *    why this call is deprecated on Mac OS X, and what other system
 *    facilities you can use in your code instead of the Time Manager.
 *    *****************************************************************
 *  
 *  Parameters:
 *    
 *    tmTaskPtr:
 *      A pointer to an extended task structure to be installed in the
 *      queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function InstallXTimeTask( tmTaskPtr: QElemPtr ): OSErr; external name '_InstallXTimeTask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PrimeTimeTask()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Activates a task in the Time Manager queue.
 *  
 *  Discussion:
 *    The PrimeTimeTask function schedules the task specified by the
 *    tmAddr field of the structure pointed to by the tmTaskPtr
 *    parameter for execution after the delay specified by the count
 *    parameter has elapsed.
 *    
 *    If the count parameter is a positive value, it is interpreted as
 *    milliseconds. If count is a negative value, it is interpreted in
 *    negated microseconds. Microsecond delays are allowable only in
 *    the revised and extended Time Managers.
 *    
 *    The task record specified by the tmTaskPtr parameter must already
 *    be installed in the queue (by a previous call to the functions
 *    InstallTimeTask or InstallXTimeTask) before your application
 *    calls the PrimeTimeTask function. The PrimeTimeTask function
 *    returns immediately, and the specified task is executed after the
 *    specified delay has elapsed. If you call the PrimeTimeTask
 *    function with a time delay of 0, the task runs as soon as
 *    interrupts are enabled.
 *    
 *    In the revised and extended Time Managers, the PrimeTimeTask
 *    function sets the high-order bit of the qType field to 1. In
 *    addition, any value of the count parameter that exceeds the
 *    maximum millisecond delay is reduced to the maximum. If you stop
 *    an unexpired task (by calling the function RemoveTimeTask) and
 *    then reinstall it (by calling the InstallXTimeTask function), you
 *    can continue the previous delay by calling the PrimeTimeTask
 *    function with the count parameter set to 0.
 *    
 *    
 *    
 *    
 *    
 *    
 *    *************************************************************
 *     See the discussion at the top of this file for information about
 *    why this call is deprecated on Mac OS X, and what other system
 *    facilities you can use in your code instead of the Time Manager.
 *    *****************************************************************
 *  
 *  Parameters:
 *    
 *    tmTaskPtr:
 *      A pointer to a task structure already installed in the queue.
 *    
 *    count:
 *      The desired delay before execution of the task.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function PrimeTimeTask( tmTaskPtr: QElemPtr; count: SIGNEDLONG ): OSErr; external name '_PrimeTimeTask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RemoveTimeTask()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Removes a task from the Time Manager queue.
 *  
 *  Discussion:
 *    The RemoveTimeTask function removes the Time Manager task
 *    structure specified by the tmTaskPtr parameter from the Time
 *    Manager queue. In both the revised and extended Time Managers, if
 *    the specified task record is active (that is, if it has been
 *    activated but the specified time has not yet elapsed), the
 *    tmCount field of the task structure returns the amount of time
 *    remaining. To provide the greatest accuracy, the unused time is
 *    reported as negated microseconds if that value is small enough to
 *    fit into the tmCount field (even if the delay was originally
 *    specified in milliseconds); otherwise, the unused time is
 *    reported in positive milliseconds. If the time has already
 *    expired, the tmCount field contains 0.
 *    
 *    In the revised and extended Time Managers, the RemoveTimeTask
 *    function sets the high-order bit of the qType field to 0.
 *     
 *    
 *    
 *    
 *    
 *    
 *    *************************************************************
 *     See the discussion at the top of this file for information about
 *    why this call is deprecated on Mac OS X, and what other system
 *    facilities you can use in your code instead of the Time Manager.
 *    *****************************************************************
 *  
 *  Parameters:
 *    
 *    tmTaskPtr:
 *      A pointer to a task structure to be removed from the queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function RemoveTimeTask( tmTaskPtr: QElemPtr ): OSErr; external name '_RemoveTimeTask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewTimerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure Microseconds(var microTickCount: UnsignedWide); external name '_Microseconds';
const
	uppTimerProcInfo = $0000B802;
	{
	 *  NewTimerUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewTimerUPP(userRoutine: TimerProcPtr): TimerUPP; external name '_NewTimerUPP'; { old name was NewTimerProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewTimerUPP( userRoutine: TimerProcPtr ): TimerUPP; external name '_NewTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeTimerUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTimerUPP( userUPP: TimerUPP ); external name '_DisposeTimerUPP';
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
procedure DisposeTimerUPP(userUPP: TimerUPP); external name '_DisposeTimerUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeTimerUPP( userUPP: TimerUPP ); external name '_DisposeTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeTimerUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeTimerUPP( tmTaskPtr_: TMTaskPtr; userUPP: TimerUPP ); external name '_InvokeTimerUPP';
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


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
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
procedure InvokeTimerUPP( tmTaskPtr_: TMTaskPtr; userUPP: TimerUPP ); external name '_InvokeTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
