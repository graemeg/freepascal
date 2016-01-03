{
     File:       HIToolbox/CarbonEventsCore.h
 
     Contains:   Carbon Event Manager
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    HIToolbox-624~3
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 1999-2008 by Apple Inc., all rights reserved.
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1999-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1999-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 1999-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
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
                     http://www.freepascal.org/bugs.html
 
}
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit CarbonEventsCore;
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
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
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
uses MacTypes,CFBase,CGEventTypes,HIGeometry;
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



>>>>>>> graemeg/cpstrnew
=======



>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}

=======
uses MacTypes,CFBase;


>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase;


>>>>>>> origin/fixes_2_2
=======
uses MacTypes,CFBase,CGEventTypes,HIGeometry;
{$endc} {not MACOSALLINCLUDE}



{$ifc TARGET_OS_MAC}

>>>>>>> origin/fixes_2.4
{$ALIGN MAC68K}

{======================================================================================}
{  The core data structure of the Carbon Event system                                  }
{======================================================================================}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	EventRef = ^OpaqueEventRef; { an opaque type }
	OpaqueEventRef = record end;
=======
	EventRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	EventRefPtr = ^EventRef;
=======
	EventRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	EventRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	EventRef = ^SInt32; { an opaque type }
	EventRefPtr = ^EventRef;
>>>>>>> origin/fixes_2.4
{======================================================================================}
{  EVENT COMMON                                                                        }
{======================================================================================}


{
 *  Discussion:
 *    The following are all errors which can be returned from the
 *    routines contained in this file.
 }
const
{
   * This is returned from PostEventToQueue if the event in question is
   * already in the queue you are posting it to (or any other queue).
   }
	eventAlreadyPostedErr = -9860;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * You are attempting to modify a target that is currently in use,
=======
   * You are attemtping to modify a target that is currently in use,
>>>>>>> graemeg/fixes_2_2
=======
   * You are attemtping to modify a target that is currently in use,
>>>>>>> origin/fixes_2_2
=======
   * You are attempting to modify a target that is currently in use,
>>>>>>> origin/fixes_2.4
   * such as when dispatching.
   }
	eventTargetBusyErr = -9861;

  {
   * This is obsolete and will be removed.
   }
	eventClassInvalidErr = -9862;

  {
   * This is obsolete and will be removed.
   }
	eventClassIncorrectErr = -9864;

  {
   * This is what you should return from a kEventClassAccessibility
   * event handler when asked to process a directly dispached event
   * that would cause your handler proc to not return after more than a
   * split-second. This will cause the accessibility engine to defer
   * the event until a later time when your handler will be able to
   * take all the time it needs without causing timeout problems for
   * the assistive application. See kEventClassAccessibility and
   * kEventAccessiblePerformNamedAction for more information. You
   * should only return this on Mac OS X 10.3 and later; earlier
   * versions will treat this like a true failure, which prevents
   * assistive applications from getting the functionality they need.
   }
	eventDeferAccessibilityEventErr = -9865;

  {
   * Returned from InstallEventHandler if the handler proc you pass is
   * already installed for a given event type you are trying to
   * register.
   }
	eventHandlerAlreadyInstalledErr = -9866;

  {
   * A generic error.
   }
	eventInternalErr = -9868;

  {
   * This is obsolete and will be removed.
   }
	eventKindIncorrectErr = -9869;

  {
   * The piece of data you are requesting from an event is not present.
   }
	eventParameterNotFoundErr = -9870;

  {
   * This is what you should return from an event handler when your
   * handler has received an event it doesn't currently want to (or
   * isn't able to) handle. If you handle an event, you should return
   * noErr from your event handler. Any return value other than
   * eventNotHandledErr will cause event handling to stop; the event
   * will not be sent to any other event handler, and the return value
   * will be provided to the original caller of SendEventToTarget.
   }
	eventNotHandledErr = -9874;

  {
   * The event loop has timed out. This can be returned from calls to
   * ReceiveNextEvent or RunCurrentEventLoop.
   }
	eventLoopTimedOutErr = -9875;

  {
   * The event loop was quit, probably by a call to QuitEventLoop. This
   * can be returned from ReceiveNextEvent or RunCurrentEventLoop.
   }
	eventLoopQuitErr = -9876;

  {
   * Returned from RemoveEventFromQueue when trying to remove an event
   * that's not in any queue.
   }
	eventNotInQueueErr = -9877;

  {
   * Returned from RegisterEventHotKey when an attempt is made to
   * register a hotkey that is already registered in the current
   * process. (Note that it is not an error to register the same hotkey
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
   * in multiple processes.) Also returned if an attempt is made to
   * register a hotkey using the kEventHotKeyExclusive option when
   * another process has already registered the same hotkey with the
   * kEventHotKeyExclusive option.
<<<<<<< HEAD
=======
   * in multiple processes.)
>>>>>>> graemeg/fixes_2_2
=======
   * in multiple processes.)
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
   }
	eventHotKeyExistsErr = -9878;

  {
   * This error code is not currently used.
   }
	eventHotKeyInvalidErr = -9879;

  {
   * When returned from an event handler, causes the event dispatcher
   * to abandon event dispatching on this target, and pass the event to
   * the first handler on the next event target. Any event handlers
   * installed beneath the current handler on the current target will
   * not receive the event. Although newly documented in Mac OS X 10.3,
   * this error code is actually available on Mac OS X 10.0 and
   * CarbonLib 1.3 and later.
   }
	eventPassToNextTargetErr = -9880;

{======================================================================================}
{  EVENT CORE                                                                          }
{======================================================================================}
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Core Event Parameters                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kEventParamDirectObject = FourCharCode('----'); { type varies depending on event}
	kEventParamDragRef = FourCharCode('drag'); { typeDragRef}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Core Event Types                                                                  }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
    Some Carbon event data types will be retained when added to an EventRef in
    Mac OS X 10.2 and later, and released when the EventRef is destroyed.
    
        Retained in 10.2 and later:
        
            typeCFStringRef
            typeCFMutableStringRef
            typeCFMutableArrayRef
            typeCFTypeRef
            typeHIAccessibleObjectRef
            
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        Retained in 10.3 and later:
=======
        Retained in Panther and later:
>>>>>>> graemeg/fixes_2_2
=======
        Retained in Panther and later:
>>>>>>> origin/fixes_2_2
=======
        Retained in 10.3 and later:
>>>>>>> origin/fixes_2.4
        
            typeEventRef
            typeCFArrayRef
            typeCFDictionaryRef:
            typeCFMutableDictionaryRef
            
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        Retained in 10.4 and later:
=======
        Retained in Tiger and later:
>>>>>>> graemeg/fixes_2_2
=======
        Retained in Tiger and later:
>>>>>>> origin/fixes_2_2
=======
        Retained in 10.4 and later:
>>>>>>> origin/fixes_2.4
        
            typeHIShapeRef
            typeMenuRef
            
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
        Retained in 10.5 and later:
        
            typeCTFontRef
            typeCTGlyphInfoRef
            typeCFAttributedStringRef
            
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
    Note that other data types may be retained in future releases of Mac OS X.
    Apple recommends that if you need to know whether a particular data type
    (other than the ones documented here) is retained, that you check the retain
    count of an instance of that data type before and after adding it to an EventRef.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	typeDragRef = FourCharCode('drag'); { DragRef}
	typeCTFontRef = FourCharCode('ctfr'); { CTFontRef}
	typeCTGlyphInfoRef = FourCharCode('ctgi'); { CTGlyphInfoRef}
=======
=======
>>>>>>> origin/fixes_2_2
	typeCFStringRef = FourCharCode('cfst'); { CFStringRef}
	typeCFMutableStringRef = FourCharCode('cfms'); { CFMutableStringRef}
	typeCFArrayRef = FourCharCode('cfar'); { CFArrayRef}
	typeCFMutableArrayRef = FourCharCode('cfma'); { CFMutableArrayRef}
	typeCFDictionaryRef = FourCharCode('cfdc'); { CFDictionaryRef}
	typeCFMutableDictionaryRef = FourCharCode('cfmd'); { CFMutableDictionaryRef}
	typeCFTypeRef = FourCharCode('cfty'); { CFTypeRef}
	typeDragRef = FourCharCode('drag'); { DragRef}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	typeDragRef = FourCharCode('drag'); { DragRef}
	typeCTFontRef = FourCharCode('ctfr'); { CTFontRef}
	typeCTGlyphInfoRef = FourCharCode('ctgi'); { CTGlyphInfoRef}
>>>>>>> origin/fixes_2.4

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Event Flags, options                                                              }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  EventPriority
 *  
 *  Discussion:
 *    These values define the relative priority of an event, and are
 *    used when posting events with PostEventToQueue. In general events
 *    are pulled from the queue in order of first posted to last
 *    posted. These priorities are a way to alter that when posting
 *    events. You can post a standard priority event and then a high
 *    priority event and the high priority event will be pulled from
 *    the queue first.
 }
type
	EventPriority = SInt16;
const
{
   * Lowest priority. Currently only window update events are posted at
   * this priority.
   }
	kEventPriorityLow = 0;

  {
   * Normal priority of events. Most events are standard priority.
   }
	kEventPriorityStandard = 1;

  {
   * Highest priority.
   }
	kEventPriorityHigh = 2;

const
	kEventLeaveInQueue = false;
	kEventRemoveFromQueue = true;

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ ¥ Event Times                                                                        }
{ EventTime is in seconds since boot. Use the constants to make life easy.             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
type
	EventTime = Float64;
	EventTimePtr = ^EventTime;
type
	EventTimeout = EventTime;
	EventTimeoutPtr = ^EventTimeout;
type
	EventTimerInterval = EventTime;
	EventTimerIntervalPtr = ^EventTimerInterval;

const
	kEventDurationSecond		= 1.0;
	kEventDurationMillisecond	= kEventDurationSecond/1000;
	kEventDurationMicrosecond	= kEventDurationSecond/1000000;
	kEventDurationNanosecond	= kEventDurationSecond/1000000000;
	kEventDurationMinute		= kEventDurationSecond*60;
	kEventDurationHour			= kEventDurationMinute*60;
	kEventDurationDay			= kEventDurationHour*24;
	kEventDurationNoWait		= 0.0;
	kEventDurationForever		= -1.0;

{ Helpful doodads to convert to and from ticks and event times}
//    #define TicksToEventTime( t )   ((EventTime)( (t) / 60.0 ))
//    #define EventTimeToTicks( t )   ((UInt32)( ((t) * 60) + 0.5 ))


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ EventTypeSpec structure                                                              }
{ This structure is used in many routines to pass a list of event types to a function. }
{ You typically would declare a const array of these types to pass in.                 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  EventTypeSpec
 *  
 *  Discussion:
 *    This structure is used to specify an event. Typically, a static
 *    array of EventTypeSpecs are passed into functions such as
 *    InstallEventHandler, as well as routines such as
 *    FlushEventsMatchingListFromQueue.
 }
type
	EventTypeSpec = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		eventClass: OSType;
=======
		eventClass: UInt32;
>>>>>>> graemeg/fixes_2_2
=======
		eventClass: UInt32;
>>>>>>> origin/fixes_2_2
=======
		eventClass: OSType;
>>>>>>> origin/fixes_2.4
		eventKind: UInt32;
	end;
	EventTypeSpecPtr = ^EventTypeSpec;
{A helpful macro for dealing with EventTypeSpecs }
// #define GetEventTypeCount( t )  (sizeof( (t) ) / sizeof( EventTypeSpec ))

type
	EventParamName = OSType;
	EventParamNamePtr = ^EventParamName;
	EventParamType = OSType;
	EventParamTypePtr = ^EventParamType;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ EventLoop                                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  EventLoopRef
 *  
 *  Discussion:
 *    An EventLoopRef represents an 'event loop', which is the
 *    conceptual entity that you 'run' to fetch events from hardware
 *    and other sources and also fires timers that might be installed
 *    with InstallEventLoopTimer. The term 'run' is a bit of a
 *    misnomer, as the event loop's goal is to stay as blocked as
 *    possible to minimize CPU usage for the current application. The
 *    event loop is run implicitly thru APIs like ReceiveNextEvent,
 *    RunApplicationEventLoop, or even WaitNextEvent. It can also be
 *    run explicitly thru a call to RunCurrentEventLoop. Each
 *    preemptive thread can have an event loop. Cooperative threads
 *    share the main thread's event loop.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	EventLoopRef = ^OpaqueEventLoopRef; { an opaque type }
	OpaqueEventLoopRef = record end;
=======
	EventLoopRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventLoopRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventLoopRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventLoopRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	EventLoopRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	EventLoopRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	EventLoopRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
{
 *  GetCurrentEventLoop()
 *  
 *  Discussion:
 *    Returns the current event loop for the current thread. If the
 *    current thread is a cooperative thread, the main event loop is
 *    returned.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    An event loop reference.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetCurrentEventLoop: EventLoopRef; external name '_GetCurrentEventLoop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMainEventLoop()
 *  
 *  Discussion:
 *    Returns the event loop object for the main application thread.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    An event loop reference.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMainEventLoop: EventLoopRef; external name '_GetMainEventLoop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RunCurrentEventLoop()
 *  
 *  Discussion:
 *    This routine 'runs' the event loop, returning only if aborted or
 *    the timeout specified is reached. The event loop is mostly
 *    blocked while in this function, occasionally waking up to fire
 *    timers or pick up events. The typical use of this function is to
 *    cause the current thread to wait for some operation to complete,
 *    most likely on another thread of execution.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inTimeout:
 *      The time to wait until returning (can be kEventDurationForever).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RunCurrentEventLoop( inTimeout: EventTimeout ): OSStatus; external name '_RunCurrentEventLoop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QuitEventLoop()
 *  
 *  Discussion:
 *    Causes a specific event loop to terminate. Usage of this is
 *    similar to WakeUpProcess, in that it causes the eventloop
 *    specified to return immediately (as opposed to timing out).
 *    Typically this call is used in conjunction with
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    RunCurrentEventLoop. 
 *    
 *    Note that this call is meant to be used while the event loop is
 *    running; i.e., you would typically call this API from a timer
 *    callback or some other callback that is invoked by
 *    RunCurrentEventLoop or ReceiveNextEvent. This API has no effect
 *    if it is called while you are not inside the event loop.
<<<<<<< HEAD
=======
 *    RunCurrentEventLoop.
>>>>>>> graemeg/fixes_2_2
=======
 *    RunCurrentEventLoop.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEventLoop:
 *      The event loop to terminate.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function QuitEventLoop( inEventLoop: EventLoopRef ): OSStatus; external name '_QuitEventLoop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetCFRunLoopFromEventLoop()
 *  
 *  Discussion:
 *    Returns the corresponding CFRunLoopRef for the given EventLoop.
 *    This is not necessarily a one-to-one mapping, hence the need for
 *    this function. In Carbon, all cooperative threads use the same
 *    run loop under the covers, so using CFRunLoopGetCurrent might
 *    yield the wrong result. In general, you would only need to use
 *    this function if you wished to add your own sources to the run
 *    loop. If you don't know what I'm talking about, then you probably
 *    don't need to use this.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEventLoop:
 *      The event loop to get the CFRunLoop for.
 *  
 *  Result:
 *    The CFRunLoopRef for inEventLoop.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetCFRunLoopFromEventLoop( inEventLoop: EventLoopRef ): CFTypeRef; external name '_GetCFRunLoopFromEventLoop';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Low-level event fetching                                                          }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  ReceiveNextEvent()
 *  
 *  Discussion:
 *    This routine tries to fetch the next event of a specified type.
 *    If no events in the event queue match, this routine will run the
 *    current event loop until an event that matches arrives, or the
 *    timeout expires. Except for timers firing, your application is
 *    blocked waiting for events to arrive when inside this function.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inNumTypes:
 *      The number of event types we are waiting for (0 if any event
 *      should cause this routine to return).
 *    
 *    inList:
 *      The list of event types we are waiting for (pass NULL if any
 *      event should cause this routine to return).
 *    
 *    inTimeout:
 *      The time to wait (passing kEventDurationForever is preferred).
 *    
 *    inPullEvent:
 *      Pass true for this parameter to actually remove the next
 *      matching event from the queue.
 *    
 *    outEvent:
 *      The next event that matches the list passed in. If inPullEvent
 *      is true, the event is owned by you, and you will need to
 *      release it when done.
 *  
 *  Result:
 *    A result indicating whether an event was received, the timeout
 *    expired, or the current event loop was quit.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function ReceiveNextEvent( inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr; inTimeout: EventTimeout; inPullEvent: Boolean; var outEvent: EventRef ): OSStatus; external name '_ReceiveNextEvent';
=======
function ReceiveNextEvent( inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr; inTimeout: EventTimeout; inPullEvent: Boolean; var outEvent: EventRef ): OSStatus; external name '_ReceiveNextEvent';
>>>>>>> graemeg/fixes_2_2
=======
function ReceiveNextEvent( inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr; inTimeout: EventTimeout; inPullEvent: Boolean; var outEvent: EventRef ): OSStatus; external name '_ReceiveNextEvent';
>>>>>>> origin/fixes_2_2
=======
function ReceiveNextEvent( inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr; inTimeout: EventTimeout; inPullEvent: Boolean; var outEvent: EventRef ): OSStatus; external name '_ReceiveNextEvent';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Core event lifetime APIs                                                          }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  EventAttributes
 *  
 *  Summary:
 *    Attributes of a Carbon event.
 }
type
	EventAttributes = UInt32;
const
{
   * The absence of any attributes.
   }
	kEventAttributeNone = 0;

  {
   * Indicates that the event is considered user input; for example, a
   * mouse event or keyboard event. Not appropriate for higher-level
   * events such as a window update or activate.
   }
	kEventAttributeUserEvent = 1 shl 0;

  {
   * This event was not originally targeted to this process, but has
   * been provided to this process because someone has installed an
   * event handler for this event type on the event monitoring target.
   * Events with this attribute are sent directly to the event monitor
   * target by the event dispatcher.
   }
	kEventAttributeMonitored = 1 shl 3;

{
 *  [Mac]CreateEvent()
 *  
 *  Summary:
 *    Creates a new Carbon event.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inAllocator:
 *      The CFAllocator to use to allocate the event data. You can pass
 *      NULL or kCFAllocatorDefault to use the standard allocator.
 *    
 *    inClassID:
 *      The event class for the event.
 *    
 *    inKind:
 *      The event kind for the event.
 *    
 *    inWhen:
 *      The event timestamp to be recorded in the event. You may pass 0
 *      to indicate the current time.
 *    
 *    inAttributes:
 *      The event attributes. Typically this should be
 *      kEventAttributeNone.
 *    
 *    outEvent:
 *      On exit, contains the new event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CreateEvent( inAllocator: CFAllocatorRef { can be NULL }; inClassID: OSType; inKind: UInt32; inWhen: EventTime; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_CreateEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function MacCreateEvent__NAMED_CreateEvent( inAllocator: CFAllocatorRef { can be NULL }; inClassID: OSType; inKind: UInt32; inWhen: EventTime; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_MacCreateEvent__NAMED_CreateEvent';
=======
function CreateEvent( inAllocator: CFAllocatorRef { can be NULL }; inClassID: UInt32; inKind: UInt32; inWhen: EventTime; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_CreateEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function MacCreateEvent__NAMED_CreateEvent( inAllocator: CFAllocatorRef { can be NULL }; inClassID: UInt32; inKind: UInt32; inWhen: EventTime; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_MacCreateEvent__NAMED_CreateEvent';
>>>>>>> graemeg/fixes_2_2
=======
function CreateEvent( inAllocator: CFAllocatorRef { can be NULL }; inClassID: UInt32; inKind: UInt32; inWhen: EventTime; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_CreateEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function MacCreateEvent__NAMED_CreateEvent( inAllocator: CFAllocatorRef { can be NULL }; inClassID: UInt32; inKind: UInt32; inWhen: EventTime; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_MacCreateEvent__NAMED_CreateEvent';
>>>>>>> origin/fixes_2_2
=======
function CreateEvent( inAllocator: CFAllocatorRef { can be NULL }; inClassID: OSType; inKind: UInt32; inWhen: EventTime; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_CreateEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function MacCreateEvent__NAMED_CreateEvent( inAllocator: CFAllocatorRef { can be NULL }; inClassID: OSType; inKind: UInt32; inWhen: EventTime; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_MacCreateEvent__NAMED_CreateEvent';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyEvent()
 *  
 *  Summary:
 *    Copies an existing event.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inOther:
 *      The original event to copy.
 *  
 *  Result:
 *    The newly created event, or NULL if either the input event was
 *    NULL or the memory for the event could not be allocated. The new
 *    event is allocated using the same allocator as the original event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyEvent( inOther: EventRef ): EventRef; external name '_CopyEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyEventAs()
 *  
 *  Summary:
 *    Copies an existing event, allowing you to change the class and
 *    kind of the event.
 *  
 *  Discussion:
 *    CopyEventAs may be useful during event flow and transformation.
 *    For example, this API is used when upgrading a raw mouse down to
 *    a window click event, to ensure that the window click event has
 *    exactly the same parameters as the original mouse down event.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inAllocator:
 *      The CFAllocator to use to allocate the event data. You can pass
 *      NULL or kCFAllocatorDefault to use the standard allocator.
 *    
 *    inOther:
 *      The original event to copy.
 *    
 *    inEventClass:
 *      The new event class for the copy of the event.
 *    
 *    inEventKind:
 *      The new event kind for the copy of the event.
 *  
 *  Result:
 *    The newly created event, or NULL if either the input event was
 *    NULL or the memory for the event could not be allocated.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CopyEventAs( inAllocator: CFAllocatorRef { can be NULL }; inOther: EventRef; inEventClass: OSType; inEventKind: UInt32 ): EventRef; external name '_CopyEventAs';
=======
function CopyEventAs( inAllocator: CFAllocatorRef { can be NULL }; inOther: EventRef; inEventClass: UInt32; inEventKind: UInt32 ): EventRef; external name '_CopyEventAs';
>>>>>>> graemeg/fixes_2_2
=======
function CopyEventAs( inAllocator: CFAllocatorRef { can be NULL }; inOther: EventRef; inEventClass: UInt32; inEventKind: UInt32 ): EventRef; external name '_CopyEventAs';
>>>>>>> origin/fixes_2_2
=======
function CopyEventAs( inAllocator: CFAllocatorRef { can be NULL }; inOther: EventRef; inEventClass: OSType; inEventKind: UInt32 ): EventRef; external name '_CopyEventAs';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  RetainEvent()
 *  
 *  Summary:
 *    Increments the retain count of an event.
 *  
 *  Discussion:
 *    Note that EventRefs are not CoreFoundation types, and therefore
 *    you cannot use CFRetain on an EventRef; you must use RetainEvent.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to retain.
 *  
 *  Result:
 *    The event that was retained.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RetainEvent( inEvent: EventRef ): EventRef; external name '_RetainEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetEventRetainCount()
 *  
 *  Summary:
 *    Returns the retain count of an event.
 *  
 *  Discussion:
 *    Note that EventRefs are not CoreFoundation types, and therefore
 *    you cannot use CFGetRetainCount on an EventRef; you must use
 *    GetEventRetainCount.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event whose retain count to return.
 *  
 *  Result:
 *    The event's retain count.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function GetEventRetainCount( inEvent: EventRef ): ItemCount; external name '_GetEventRetainCount';
=======
function GetEventRetainCount( inEvent: EventRef ): UInt32; external name '_GetEventRetainCount';
>>>>>>> graemeg/fixes_2_2
=======
function GetEventRetainCount( inEvent: EventRef ): UInt32; external name '_GetEventRetainCount';
>>>>>>> origin/fixes_2_2
=======
function GetEventRetainCount( inEvent: EventRef ): ItemCount; external name '_GetEventRetainCount';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ReleaseEvent()
 *  
 *  Summary:
 *    Decrements the retain count of an event. If the retain count
 *    reaches zero, the event is destroyed.
 *  
 *  Discussion:
 *    Note that EventRefs are not CoreFoundation types, and therefore
 *    you cannot use CFRelease on an EventRef; you must use
 *    ReleaseEvent.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure ReleaseEvent( inEvent: EventRef ); external name '_ReleaseEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetEventParameter()
 *  
 *  Summary:
 *    Sets a piece of data for the given event.
 *  
 *  Discussion:
 *    SetEventParameter is thread-safe to the extent of allowing
 *    multiple threads to each modify a separate event, but it is not
 *    safe to have multiple threads call SetEventParameter on the same
 *    event. Multiple threads modifying the same event can cause
 *    corruption of the event data.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to set the data for.
 *    
 *    inName:
 *      The symbolic name of the parameter.
 *    
 *    inType:
 *      The symbolic type of the parameter.
 *    
 *    inSize:
 *      The size of the parameter data.
 *    
 *    inDataPtr:
 *      The pointer to the parameter data.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SetEventParameter( inEvent: EventRef; inName: EventParamName; inType: EventParamType; inSize: ByteCount; inDataPtr: {const} UnivPtr ): OSStatus; external name '_SetEventParameter';
=======
function SetEventParameter( inEvent: EventRef; inName: EventParamName; inType: EventParamType; inSize: UInt32; inDataPtr: {const} UnivPtr ): OSStatus; external name '_SetEventParameter';
>>>>>>> graemeg/fixes_2_2
=======
function SetEventParameter( inEvent: EventRef; inName: EventParamName; inType: EventParamType; inSize: UInt32; inDataPtr: {const} UnivPtr ): OSStatus; external name '_SetEventParameter';
>>>>>>> origin/fixes_2_2
=======
function SetEventParameter( inEvent: EventRef; inName: EventParamName; inType: EventParamType; inSize: ByteCount; inDataPtr: {const} UnivPtr ): OSStatus; external name '_SetEventParameter';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetEventParameter()
 *  
 *  Discussion:
 *    Gets a piece of data from the given event, if it exists.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to get the parameter from.
 *    
 *    inName:
 *      The symbolic name of the parameter.
 *    
 *    inDesiredType:
 *      The desired type of the parameter. The Carbon Event Manager
 *      will automatically use AppleEvent coercion handlers to convert
 *      the data in the event into the desired type, if possible. You
 *      may also pass typeWildCard to request that the data be returned
 *      in its original format.
 *    
 *    outActualType:
 *      The actual type of the parameter, or NULL if you are not
 *      interested in receiving this information.
 *    
 *    inBufferSize:
 *      The size of the output buffer specified by ioBuffer. You may
 *      pass zero for this parameter and NULL for the outData parameter
 *      if you don't want the data returned.
 *    
 *    outActualSize:
 *      The actual size of the data, or NULL if you don't want this
 *      information.
 *    
 *    outData:
 *      The pointer to the buffer which will receive the parameter
 *      data, or NULL if you don't want the data returned. If you pass
 *      NULL, you must also pass zero for the inBufferSize parameter.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function GetEventParameter( inEvent: EventRef; inName: EventParamName; inDesiredType: EventParamType; outActualType: EventParamTypePtr { can be NULL }; inBufferSize: ByteCount; outActualSize: ByteCountPtr { can be NULL }; outData: UnivPtr { can be NULL } ): OSStatus; external name '_GetEventParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveEventParameter()
 *  
 *  Summary:
 *    Removes a piece of data from the given event, if it exists.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to remove the data from.
 *    
 *    inName:
 *      The symbolic name of the parameter.
 *  
 *  Result:
 *    An operating system result code. eventParameterNotFoundErr is
 *    returned if the specified parameter is not present in the event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RemoveEventParameter( inEvent: EventRef; inName: EventParamName ): OSStatus; external name '_RemoveEventParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
function GetEventParameter( inEvent: EventRef; inName: EventParamName; inDesiredType: EventParamType; outActualType: EventParamTypePtr { can be NULL }; inBufferSize: UInt32; outActualSize: UInt32Ptr { can be NULL }; outData: UnivPtr { can be NULL } ): OSStatus; external name '_GetEventParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetEventParameter( inEvent: EventRef; inName: EventParamName; inDesiredType: EventParamType; outActualType: EventParamTypePtr { can be NULL }; inBufferSize: ByteCount; outActualSize: ByteCountPtr { can be NULL }; outData: UnivPtr { can be NULL } ): OSStatus; external name '_GetEventParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveEventParameter()
 *  
 *  Summary:
 *    Removes a piece of data from the given event, if it exists.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to remove the data from.
 *    
 *    inName:
 *      The symbolic name of the parameter.
 *  
 *  Result:
 *    An operating system result code. eventParameterNotFoundErr is
 *    returned if the specified parameter is not present in the event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RemoveEventParameter( inEvent: EventRef; inName: EventParamName ): OSStatus; external name '_RemoveEventParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/fixes_2.4
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Getters for 'base-class' event info                                               }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  GetEventClass()
 *  
 *  Discussion:
 *    Returns the class of the given event, such as mouse, keyboard,
 *    etc.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event in question.
 *  
 *  Result:
 *    The class ID of the event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function GetEventClass( inEvent: EventRef ): OSType; external name '_GetEventClass';
=======
function GetEventClass( inEvent: EventRef ): UInt32; external name '_GetEventClass';
>>>>>>> graemeg/fixes_2_2
=======
function GetEventClass( inEvent: EventRef ): UInt32; external name '_GetEventClass';
>>>>>>> origin/fixes_2_2
=======
function GetEventClass( inEvent: EventRef ): OSType; external name '_GetEventClass';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetEventKind()
 *  
 *  Discussion:
 *    Returns the kind of the given event (mousedown, etc.). Event
 *    kinds overlap between event classes, e.g. kEventMouseDown and
 *    kEventAppActivated have the same value (1). The combination of
 *    class and kind is what determines an event signature.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event in question.
 *  
 *  Result:
 *    The kind of the event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetEventKind( inEvent: EventRef ): UInt32; external name '_GetEventKind';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetEventTime()
 *  
 *  Discussion:
 *    Returns the time the event specified occurred, specified in
 *    EventTime, which is a floating point number representing seconds
 *    since the last system startup.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event in question.
 *  
 *  Result:
 *    The time the event occurred.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetEventTime( inEvent: EventRef ): EventTime; external name '_GetEventTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Setters for 'base-class' event info                                               }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  SetEventTime()
 *  
 *  Discussion:
 *    This routine allows you to set the time of a given event, if you
 *    so desire. In general, you would never use this routine, except
 *    for those special cases where you reuse an event from time to
 *    time instead of creating a new event each time.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event in question.
 *    
 *    inTime:
 *      The new time.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetEventTime( inEvent: EventRef; inTime: EventTime ): OSStatus; external name '_SetEventTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{  ¥ CGEventRef support                                                                }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  CreateEventWithCGEvent()
 *  
 *  Summary:
 *    Creates a Carbon event using the contents of a CGEventRef. The
 *    event class and kind are determined by the type of CGEventRef
 *    that is passed, and cannot be specified by the caller. The event
 *    timestamp is copied from the CGEventRef timestamp.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inAllocator:
 *      The CFAllocator to use to allocate the event data. You can pass
 *      NULL or kCFAllocatorDefault to use the standard allocator.
 *    
 *    inEvent:
 *      The CGEventRef from which the Carbon event should be created.
 *      This parameter will be retained by the Carbon event, and
 *      released when the Carbon event is released.
 *    
 *    inAttributes:
 *      The event attributes. Typically this should be
 *      kEventAttributeNone.
 *    
 *    outEvent:
 *      On exit, if this function returns noErr, then the new event
 *      will be written to this location.
 *  
 *  Result:
 *    An operating system result code, including paramErr if the
 *    CGEventRef is not of a type that can be converted into a Carbon
 *    event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CreateEventWithCGEvent( inAllocator: CFAllocatorRef; inEvent: CGEventRef; inAttributes: EventAttributes; var outEvent: EventRef ): OSStatus; external name '_CreateEventWithCGEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  CopyEventCGEvent()
 *  
 *  Summary:
 *    Returns the CGEventRef associated with a Carbon event, or NULL if
 *    the event has no CGEventRef.
 *  
 *  Discussion:
 *    This API returns the CGEventRef associated with a Carbon event if
 *    the Carbon event was originally created using a CGEventRef.
 *    Typically, this only applies to user-input Carbon events such as
 *    keyboard and mouse events. Carbon events that were created using
 *    CreateEvent do not have a CGEventRef associated with them; for
 *    example, kEventWindowUpdate has no CGEventRef. For such events,
 *    this API returns NULL.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The Carbon event whose CGEventRef you would like.
 *  
 *  Result:
 *    The CGEventRef associated with the Carbon event, or NULL if the
 *    event has no CGEventRef. If a CGEventRef is returned, then it has
 *    been retained by this function, and should be released by the
 *    caller.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CopyEventCGEvent( inEvent: EventRef ): CGEventRef; external name '_CopyEventCGEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{  ¥ Event Queue routines (posting, finding, flushing)                                 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	EventQueueRef = ^OpaqueEventQueueRef; { an opaque type }
	OpaqueEventQueueRef = record end;
=======
	EventQueueRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventQueueRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventQueueRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventQueueRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	EventQueueRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	EventQueueRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	EventQueueRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
{
 *  GetCurrentEventQueue()
 *  
 *  Discussion:
 *    Returns the current event queue for the current thread. If the
 *    current thread is a cooperative thread, the main event queue is
 *    returned.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    An event queue reference.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetCurrentEventQueue: EventQueueRef; external name '_GetCurrentEventQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMainEventQueue()
 *  
 *  Discussion:
 *    Returns the event queue object for the main application thread.
 *    
 *    
 *    GetMainEventQueue is threadsafe in Mac OS X 10.4 and later. On
 *    earlier versions of Mac OS X, you should call GetMainEventQueue
 *    once before creating any other threads if those other threads
 *    will be calling GetMainEventQueue themselves.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Result:
 *    An event queue reference.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMainEventQueue: EventQueueRef; external name '_GetMainEventQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EventComparatorProcPtr
 *  
 *  Discussion:
 *    Type of a callback function used by queue searches.
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to compare.
 *    
 *    inCompareData:
 *      The data used to compare the event.
 *  
 *  Result:
 *    A boolean value indicating whether the event matches (true) or
 *    not (false).
 }
type
	EventComparatorProcPtr = function( inEvent: EventRef; inCompareData: UnivPtr ): Boolean;
type
	EventComparatorUPP = EventComparatorProcPtr;
{
 *  NewEventComparatorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewEventComparatorUPP( userRoutine: EventComparatorProcPtr ): EventComparatorUPP; external name '_NewEventComparatorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeEventComparatorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeEventComparatorUPP( userUPP: EventComparatorUPP ); external name '_DisposeEventComparatorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeEventComparatorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeEventComparatorUPP( inEvent: EventRef; inCompareData: UnivPtr; userUPP: EventComparatorUPP ): Boolean; external name '_InvokeEventComparatorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PostEventToQueue()
 *  
 *  Discussion:
 *    Posts an event to the specified queue and increments the event's
 *    retain count. This automatically wakes up the event loop of the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    thread to which the queue belongs. After posting the event, you
 *    may release the event, since it is retained by the queue. If the
 *    event is already contained in any event queue,
 *    eventAlreadyPostedErr will be returned and the event will not be
 *    posted. 
 *    
 *    If the event is posted to the main event queue, then the event
 *    will be retrieved and dispatched by a subsequent call to the
 *    event loop by the main thread. If the event is posted to an event
 *    queue of a non-main thread, then that thread must be running its
 *    own event loop (calling ReceiveNextEvent and dispatching the
 *    event) for the event to be removed and dispatched. 
 *    
 *    If the event uses a standard event class (such as
 *    kEventClassWindow), then the event dispatcher will send the event
 *    to an appropriate event target (such as the specified window); if
 *    the event uses a custom event class, then the event dispatcher
 *    will send the event to the application target. 
 *    
 *    For custom event classes, you may specify a destination event
 *    target other than the application target by adding the
 *    kEventParamPostTarget event parameter to the event before posting
 *    it. The parameter should contain the event target to which the
 *    event should be sent. You may specify custom event target sending
 *    options by adding the kEventParamPostOptions event parameter to
 *    the event.
<<<<<<< HEAD
=======
 *    thread the queue belongs to. After posting the event, you may
 *    release the event, since it is retained by the queue.
>>>>>>> graemeg/fixes_2_2
=======
 *    thread the queue belongs to. After posting the event, you may
 *    release the event, since it is retained by the queue.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The event queue to post the event onto.
 *    
 *    inEvent:
 *      The event to post.
 *    
 *    inPriority:
 *      The priority of the event.
 *  
 *  Result:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    An operating system result code. eventAlreadyPostedErr is
 *    returned if the event is already contained in any event queue,
 *    and in this case the event will not be posted.
=======
 *    An operating system result code.
>>>>>>> graemeg/fixes_2_2
=======
 *    An operating system result code.
>>>>>>> origin/fixes_2_2
=======
 *    An operating system result code. eventAlreadyPostedErr is
 *    returned if the event is already contained in any event queue,
 *    and in this case the event will not be posted.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PostEventToQueue( inQueue: EventQueueRef; inEvent: EventRef; inPriority: EventPriority ): OSStatus; external name '_PostEventToQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FlushEventsMatchingListFromQueue()
 *  
 *  Discussion:
 *    Flushes events matching a specified list of classes and kinds
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    from an event queue. 
 *    
 *    This API may be safely used by any thread to flush the events
 *    from that thread's event queue. Prior to Mac OS X 10.5, it is
 *    unsafe to call this API from any thread other than the main
 *    thread when flushing the main event queue. The main event queue
 *    may be flushed from any thread in Mac OS X 10.5 and later.
<<<<<<< HEAD
=======
 *    from an event queue.
>>>>>>> graemeg/fixes_2_2
=======
 *    from an event queue.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The event queue to flush events from.
 *    
 *    inNumTypes:
 *      The number of event kinds to flush.
 *    
 *    inList:
 *      The list of event classes and kinds to flush from the queue.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function FlushEventsMatchingListFromQueue( inQueue: EventQueueRef; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_FlushEventsMatchingListFromQueue';
=======
function FlushEventsMatchingListFromQueue( inQueue: EventQueueRef; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_FlushEventsMatchingListFromQueue';
>>>>>>> graemeg/fixes_2_2
=======
function FlushEventsMatchingListFromQueue( inQueue: EventQueueRef; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_FlushEventsMatchingListFromQueue';
>>>>>>> origin/fixes_2_2
=======
function FlushEventsMatchingListFromQueue( inQueue: EventQueueRef; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_FlushEventsMatchingListFromQueue';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FlushSpecificEventsFromQueue()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Flushes events that match a comparator function. 
 *    
 *    This API may be safely used by any thread to flush the events
 *    from that thread's event queue. Prior to Mac OS X 10.5, it is
 *    unsafe to call this API from any thread other than the main
 *    thread when flushing the main event queue. The main event queue
 *    may be flushed from any thread in Mac OS X 10.5 and later.
<<<<<<< HEAD
=======
 *    Flushes events that match a comparator function.
>>>>>>> graemeg/fixes_2_2
=======
 *    Flushes events that match a comparator function.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The event queue to flush events from.
 *    
 *    inComparator:
 *      The comparison function to invoke for each event in the queue.
 *    
 *    inCompareData:
 *      The data you wish to pass to your comparison function.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function FlushSpecificEventsFromQueue( inQueue: EventQueueRef; inComparator: EventComparatorUPP; inCompareData: UnivPtr ): OSStatus; external name '_FlushSpecificEventsFromQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FlushEventQueue()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Flushes all events from an event queue. 
 *    
 *    This API may be safely used by any thread to flush the events
 *    from that thread's event queue. Prior to Mac OS X 10.5, it is
 *    unsafe to call this API from any thread other than the main
 *    thread when flushing the main event queue. The main event queue
 *    may be flushed from any thread in Mac OS X 10.5 and later.
<<<<<<< HEAD
=======
 *    Flushes all events from an event queue.
>>>>>>> graemeg/fixes_2_2
=======
 *    Flushes all events from an event queue.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The event queue to flush.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function FlushEventQueue( inQueue: EventQueueRef ): OSStatus; external name '_FlushEventQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FindSpecificEventInQueue()
 *  
 *  Discussion:
 *    Returns the first event that matches a comparator function, or
 *    NULL if no events match.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The event queue to search.
 *    
 *    inComparator:
 *      The comparison function to invoke for each event in the queue.
 *    
 *    inCompareData:
 *      The data you wish to pass to your comparison function.
 *  
 *  Result:
 *    An event reference. The event is still in the queue when
 *    FindSpecificEventInQueue returns; you can remove it from the
 *    queue with RemoveEventFromQueue. The returned event does not need
 *    to be released by the caller.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function FindSpecificEventInQueue( inQueue: EventQueueRef; inComparator: EventComparatorUPP; inCompareData: UnivPtr ): EventRef; external name '_FindSpecificEventInQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetNumEventsInQueue()
 *  
 *  Discussion:
 *    Returns the number of events in an event queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The event queue to query.
 *  
 *  Result:
 *    The number of items in the queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function GetNumEventsInQueue( inQueue: EventQueueRef ): ItemCount; external name '_GetNumEventsInQueue';
=======
function GetNumEventsInQueue( inQueue: EventQueueRef ): UInt32; external name '_GetNumEventsInQueue';
>>>>>>> graemeg/fixes_2_2
=======
function GetNumEventsInQueue( inQueue: EventQueueRef ): UInt32; external name '_GetNumEventsInQueue';
>>>>>>> origin/fixes_2_2
=======
function GetNumEventsInQueue( inQueue: EventQueueRef ): ItemCount; external name '_GetNumEventsInQueue';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveEventFromQueue()
 *  
 *  Discussion:
 *    Removes the given event from the specified queue and decrements
 *    the event's retain count. If it was your intention to hold onto
 *    the event, you must retain the event before removing it from the
 *    queue.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The queue to remove the event from.
 *    
 *    inEvent:
 *      The event to remove.
 *  
 *  Result:
 *    An operating system result code. eventNotInQueueErr is returned
 *    if the event is not actually contained in the specified queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveEventFromQueue( inQueue: EventQueueRef; inEvent: EventRef ): OSStatus; external name '_RemoveEventFromQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsEventInQueue()
 *  
 *  Discussion:
 *    Returns true if the specified event is posted to a queue.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The queue to check.
 *    
 *    inEvent:
 *      The event in question.
 *  
 *  Result:
 *    A boolean value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsEventInQueue( inQueue: EventQueueRef; inEvent: EventRef ): Boolean; external name '_IsEventInQueue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


const
{
   * Currently, the only value you can pass to
   * AcquireFirstMatchingEventInQueue in its inOptions parameter.
   }
	kEventQueueOptionsNone = 0;

{
 *  AcquireFirstMatchingEventInQueue()
 *  
 *  Discussion:
 *    Returns the first event that matches the list of event classes
 *    and kinds passed in. This call does not call the event loop, and
 *    hence no timers will fire nor will any window flushing occur when
 *    this API is called. New events will be pulled from the window
 *    server, however. Overall this API should have better performance
 *    characteristics than the older EventAvail API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inQueue:
 *      The queue to check.
 *    
 *    inNumTypes:
 *      The number of event kinds to search for. You may pass zero for
 *      this parameter if you also pass NULL for inList.
 *    
 *    inList:
 *      The list of event classes and kinds to search for in the queue.
 *      You may pass NULL for this parameter if you also pass zero for
 *      inNumTypes. This effectively matches ANY event in the queue,
 *      and will merely return the first event in the queue.
 *    
 *    inOptions:
 *      Currently, you must pass kEventQueueOptionsNone for this
 *      parameter.
 *  
 *  Result:
 *    An event reference, or NULL if no events match the list passed.
 *    The event returned has had its refcount incremented (i.e. it has
 *    been retained). As a result, you must release this value
 *    (assuming it's non-NULL). The event is not removed from the queue
 *    by this API; you should call RemoveEventFromQueue if necessary.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AcquireFirstMatchingEventInQueue( inQueue: EventQueueRef; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr; inOptions: OptionBits ): EventRef; external name '_AcquireFirstMatchingEventInQueue';
=======
function AcquireFirstMatchingEventInQueue( inQueue: EventQueueRef; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr; inOptions: OptionBits ): EventRef; external name '_AcquireFirstMatchingEventInQueue';
>>>>>>> graemeg/fixes_2_2
=======
function AcquireFirstMatchingEventInQueue( inQueue: EventQueueRef; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr; inOptions: OptionBits ): EventRef; external name '_AcquireFirstMatchingEventInQueue';
>>>>>>> origin/fixes_2_2
=======
function AcquireFirstMatchingEventInQueue( inQueue: EventQueueRef; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr; inOptions: OptionBits ): EventRef; external name '_AcquireFirstMatchingEventInQueue';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Queue-synchronized event and input device state                                     }
=======
{  Queue-synchronized event state                                                      }
>>>>>>> graemeg/fixes_2_2
=======
{  Queue-synchronized event state                                                      }
>>>>>>> origin/fixes_2_2
=======
{  Queue-synchronized event and input device state                                     }
>>>>>>> origin/fixes_2.4
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  GetCurrentEvent()
 *  
 *  Summary:
 *    Returns the user input event currently being handled.
 *  
 *  Discussion:
 *    When an event with kEventAttributeUserEvent is dispatched by the
 *    event dispatcher target, it is recorded internally by the Event
 *    Manager. At any time during the handling of that event (or of any
 *    other event which is created and sent during the handling of the
 *    original event), GetCurrentEvent may be used to retrieve the
 *    original EventRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    The user input (mouse or keyboard) event currently being handled.
 *    May be NULL if no event is currently being handled, or if the
 *    current event was not a user input event. The returned event is
 *    not retained, and its lifetime should be considered to be no
 *    longer than the current function; if you need to keep the event
 *    alive past that time, you should retain it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function GetCurrentEvent: EventRef; external name '_GetCurrentEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  GetCurrentEventButtonState()
 *  
 *  Summary:
 *    Returns the current queue-synchronized mouse button state on the
 *    primary input device.
 *  
 *  Discussion:
 *    At any point in the handling of user input, there are two
 *    different mouse button states: the queue-synchronized state and
 *    the hardware state. The hardware state reflects the actual
 *    current state of the mouse attached to the user's machine. The
 *    queue-synchronized state reflects the state according to the
 *    events that have been processed at that point by the application.
 *    These two states may be different if there are unprocessed events
 *    in the event queue, or if events are being artificially
 *    introduced into the event queue from an outside source.
 *    GetCurrentEventButtonState returns the queue-synchronized button
 *    state. This state is determined by user input events that are
 *    sent through the event dispatcher target; whenever a user input
 *    event (mouse or keyboard) is handled by the Carbon event
 *    dispatcher, its button state is recorded, and that button state
 *    will be returned by GetCurrentEventButtonState. 
 *    
 *    The "current event" referenced in the API name is the event most
 *    recently dispatched through the event dispatcher target, which is
 *    not necessarily the event that your event handler is handling.
 *    For example, if a mouse-down event occurs, and you have a handler
 *    for the kEventWindowHandleContentClick event that is generated
 *    from the mouse-down, then the button state will be that which was
 *    attached to the mouse-down. The ContentClick event itself does
 *    also have MouseButton and MouseChord parameters, which are copied
 *    from the the mouse-down event, but GetCurrentEventButtonState
 *    returns the button state from the mouse-down, not from the
 *    ContentClick event, since it was the mouse-down that was most
 *    recently dispatched through the event dispatcher. Usually, this
 *    is the behavior that you want anyways. 
 *    
 *    Note that events that are not sent through the event dispatcher
 *    target will not update the current event button state. Also, note
 *    that events arriving from outside the application, such as an
 *    AppleEvent or an Accessibility event, also will not update the
 *    modifiers. If your application modifies its behavior based on
 *    button state, we recommend that you parameterize your core code
 *    with the event buttons, and determine the button state based on
 *    the origin of the behavior request. For a request that originates
 *    directly from user input, you can use GetCurrentEventButtonState,
 *    but for a request that originates from an AppleEvent or
 *    Accessibility event, you would probably use no button state, or
 *    perhaps just left-button-pressed. 
 *    
 *    It is generally better to use this API than to use the Button
 *    function or the GetCurrentButtonState function (which return the
 *    hardware state). This gives a more consistent user experience
 *    when the user input queue is being remoted controlled or
 *    manipulated via non-hardware event sources such as speech or
 *    AppleEvents; using GetCurrentEventButtonState is also much faster
 *    than using Button or GetCurrentButtonState.
 *    
 *    Note that GetCurrentEventButtonState only returns a valid button
 *    state if your application is the active application. If your
 *    application is not active, then user input events are not flowing
 *    through the event dispatcher and the queue-synchronized state is
 *    not updated.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    The queue-synchronized state of the mouse buttons. Bit zero
 *    indicates the state of the primary button, bit one the state of
 *    the secondary button, and so on.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function GetCurrentEventButtonState: UInt32; external name '_GetCurrentEventButtonState';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  GetCurrentEventKeyModifiers()
 *  
 *  Summary:
 *    Returns the current queue-synchronized keyboard modifier state.
 *  
 *  Discussion:
 *    At any point in the handling of user input, there are two
 *    different keyboard modifier states: the queue-synchronized state
 *    and the hardware state. The hardware state reflects the actual
 *    current state of the keyboard attached to the user's machine. The
 *    queue-synchronized state reflects the state according to the
 *    events that have been processed at that point by the application.
 *    These two states may be different if there are unprocessed events
 *    in the event queue, or if events are being artificially
 *    introduced into the event queue from an outside source.
 *    GetCurrentEventKeyModifiers returns the queue-synchronized
 *    modifier state. This state is determined by user input events
 *    that are sent through the event dispatcher target; whenever a
 *    user input event (mouse or keyboard) is handled by the Carbon
 *    event dispatcher, its modifiers are recorded, and those modifiers
 *    will be returned by GetCurrentEventKeyModifiers. 
 *    
 *    The "current event" referenced in the API name is the event most
 *    recently dispatched through the event dispatcher target, which is
 *    not necessarily the event that your event handler is handling.
 *    For example, if a mouse-down event occurs, and you have a handler
 *    for the kEventWindowHandleContentClick event that is generated
 *    from the mouse-down, then the modifiers will be those that were
 *    attached to the mouse-down. The ContentClick event itself does
 *    also have a KeyModifiers parameter, which is copied from the
 *    mouse-down event, but GetCurrentEventKeyModifiers returns the
 *    modifiers from the mouse-down, not from the ContentClick event,
 *    since it was the mouse-down that was most recently dispatched
 *    through the event dispatcher. Usually, this is the behavior that
 *    you want anyways. 
 *    
 *    Note that events that are not sent through the event dispatcher
 *    target will not update the current event key modifiers. Also,
 *    note that events arriving from outside the application, such as
 *    an AppleEvent or an Accessibility event, also will not update the
 *    modifiers. If your application modifies its behavior based on
 *    modifier state, we recommend that you parameterize your core code
 *    with the event modifiers, and determine the modifiers based on
 *    the origin of the behavior request. For a request that originates
 *    directly from user input, you can use
 *    GetCurrentEventKeyModifiers, but for a request that originates
 *    from an AppleEvent or Accessibility event, you would probably use
 *    no modifiers. 
 *    BR> It is generally better to use this API than to use the
 *    GetCurrentKeyModifiers API (which returns the hardware state).
 *    This gives a more consistent user experience when the user input
 *    queue is being remoted controlled or manipulated via non-hardware
 *    event sources such as speech or AppleEvents; using
 *    GetCurrentEventKeyModifiers is also much faster than using
 *    EventAvail(0, &eventRecord) or GetCurrentKeyModifiers.
 *    
 *    Note that GetCurrentEventKeyModifiers only returns a valid
 *    modifier state if your application is the active application. If
 *    your application is not active, then user input events are not
 *    flowing through the event dispatcher and the queue-synchronized
 *    state is not updated.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    The queue-synchronized state of the keyboard modifiers. The
 *    format of the return value is the same as the modifiers field of
 *    an EventRecord (but only includes keyboard modifiers and not the
 *    other modifier flags included in an EventRecord).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function GetCurrentEventKeyModifiers: UInt32; external name '_GetCurrentEventKeyModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Non-synchronized input device state                                                 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  HIGetMousePosition()
 *  
 *  Summary:
 *    Returns the position of the mouse relative to the given object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSpace:
 *      The HICoordinateSpace constant specifying the desired
 *      coordinate space that the mouse position is to be relative to.
 *    
 *    inObject:
 *      A specific object defining the destination coordinate space
 *      that the point is to be returned in. You might pass a WindowRef
 *      or an HIViewRef. If no object is necessary, you must pass NULL.
 *      See the HICoordinateSpace documentation for details on which
 *      HICoordinateSpaces require objects.
 *    
 *    outPoint:
 *      A pointer to an HIPoint that will contain the mouse position on
 *      exit. If any parameter is invalid, this will be returned as the
 *      zero point.
 *  
 *  Result:
 *    A pointer to the HIPoint passed in outPoint.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.5 and later
 *    Non-Carbon CFM:   not available
 }
function HIGetMousePosition( inSpace: HICoordinateSpace; inObject: UnivPtr; var outPoint: HIPoint ): HIPointPtr; external name '_HIGetMousePosition';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  GetGlobalMouse()
 *  
 *  Summary:
 *    Returns the position of the mouse in global coordinates.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    globalMouse:
 *      On exit, contains the mouse position in global coordinates.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure GetGlobalMouse( var globalMouse: Point ); external name '_GetGlobalMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

=======
{  Multiple-button support                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
>>>>>>> graemeg/fixes_2_2
=======
{  Multiple-button support                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
>>>>>>> origin/fixes_2_2
=======
{  Non-synchronized input device state                                                 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  HIGetMousePosition()
 *  
 *  Summary:
 *    Returns the position of the mouse relative to the given object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSpace:
 *      The HICoordinateSpace constant specifying the desired
 *      coordinate space that the mouse position is to be relative to.
 *    
 *    inObject:
 *      A specific object defining the destination coordinate space
 *      that the point is to be returned in. You might pass a WindowRef
 *      or an HIViewRef. If no object is necessary, you must pass NULL.
 *      See the HICoordinateSpace documentation for details on which
 *      HICoordinateSpaces require objects.
 *    
 *    outPoint:
 *      A pointer to an HIPoint that will contain the mouse position on
 *      exit. If any parameter is invalid, this will be returned as the
 *      zero point.
 *  
 *  Result:
 *    A pointer to the HIPoint passed in outPoint.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.5 and later
 *    Non-Carbon CFM:   not available
 }
function HIGetMousePosition( inSpace: HICoordinateSpace; inObject: UnivPtr; var outPoint: HIPoint ): HIPointPtr; external name '_HIGetMousePosition';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  GetGlobalMouse()
 *  
 *  Summary:
 *    Returns the position of the mouse in global coordinates.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    globalMouse:
 *      On exit, contains the mouse position in global coordinates.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure GetGlobalMouse( var globalMouse: Point ); external name '_GetGlobalMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  GetCurrentButtonState()
 *  
 *  Summary:
 *    Returns the current hardware mouse button state on the primary
 *    input device.
 *  
 *  Discussion:
 *    In most cases, you should not use GetCurrentButtonState, but
 *    should use the GetCurrentEventButtonState function instead.
 *    GetCurrentEventButtonState is much faster than
 *    GetCurrentButtonState because it returns the locally cached
 *    button state; GetCurrentButtonState must get the mouse button
 *    state from the window server, which is slower. Using
 *    GetCurrentButtonState also can prevent your application from
 *    being operated by remote posting of events, since the hardware
 *    input device is not actually changing state in that case. Most
 *    commonly, you might need to use GetCurrentButtonState when your
 *    application is not the active application (as determined by the
 *    Process Manager function GetFrontProcess). In that case, the
 *    cached button state returned by GetCurrentEventButtonState is not
 *    valid because mouse button events are not flowing to your
 *    application, and you must use GetCurrentButtonState to determine
 *    the current hardware state.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    The state of the mouse buttons on the mouse hardware. Bit zero
 *    indicates the state of the primary button, bit one the state of
 *    the secondary button, and so on.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function GetCurrentButtonState: UInt32; external name '_GetCurrentButtonState';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  GetCurrentKeyModifiers()
 *  
 *  Summary:
 *    Returns the current hardware keyboard modifier state.
 *  
 *  Discussion:
 *    In most cases, you should not use GetCurrentKeyModifiers, but
 *    should use the GetCurrentEventKeyModifiers function instead.
 *    GetCurrentEventKeyModifiers is much faster than
 *    GetCurrentKeyModifiers because it returns the locally cached
 *    modifier state; GetCurrentKeyModifiers must get the modifier
 *    state from the window server, which is slower. Using
 *    GetCurrentKeyModifiers also can prevent your application from
 *    being operated by remote posting of events, since the hardware
 *    input device is not actually changing state in that case. Most
 *    commonly, you might need to use GetCurrentKeyModifiers when your
 *    application is not the active application (as determined by the
 *    Process Manager function GetFrontProcess). In that case, the
 *    cached modifier state returned by GetCurrentEventKeyModifiers is
 *    not valid because modifier-changed events are not flowing to your
 *    application, and you must use GetCurrentKeyModifiers to determine
 *    the current hardware state.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    The hardware state of the keyboard modifiers. The format of the
 *    return value is the same as the modifiers field of an EventRecord
 *    (but only includes keyboard modifiers and not the other modifier
 *    flags included in an EventRecord).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetCurrentKeyModifiers: UInt32; external name '_GetCurrentKeyModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Helpful utilities                                                                 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  GetCurrentEventTime()
 *  
 *  Discussion:
 *    Returns the current time since last system startup in seconds.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    EventTime.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetCurrentEventTime: EventTime; external name '_GetCurrentEventTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Timers                                                                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  EventLoopTimerRef
 *  
 *  Discussion:
 *    An EventLoopTimerRef represents what we term a 'timer'. A timer
 *    is a function that is called either once or at regular intervals.
 *    It executes at task level and should not be confused with Time
 *    Manager Tasks or any other interrupt-level callback. This means
 *    you can call Toolbox routines, allocate memory and draw. When a
 *    timer 'fires', it calls a callback that you specify when the
 *    timer is installed. Timers in general have two uses - as a
 *    timeout mechanism and as a periodic task. An everyday example of
 *    using a timer for a timeout might be a light that goes out if no
 *    motion is detected in a room for 5 minutes. For this, you might
 *    install a timer which will fire in 5 minutes. If motion is
 *    detected, you would reset the timer fire time and let the clock
 *    start over. If no motion is detected for the full 5 minutes, the
 *    timer will fire and you could power off the light. A periodic
 *    timer is one that fires at regular intervals (say every second or
 *    so). You might use such a timer to blink the insertion point in
 *    your editor, etc. One advantage of timers is that you can install
 *    the timer right from the code that wants the time. For example,
 *    the standard Toolbox Edit Text control can install a timer to
 *    blink the cursor when it's active, meaning that IdleControls is a
 *    no-op for that control and doesn't need to be called. When the
 *    control is inactive, it removes its timer and doesn't waste CPU
 *    time in that state. NOTE: Currently, if you do decide to draw
 *    when your timer is called, be sure to save and restore the
 *    current port so that calling your timer doesn't inadvertently
 *    change the port out from under someone.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	EventLoopTimerRef = ^__EventLoopTimer; { an opaque type }
	__EventLoopTimer = record end;
=======
	EventLoopTimerRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventLoopTimerRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventLoopTimerRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventLoopTimerRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	EventLoopTimerRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	EventLoopTimerRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	EventLoopTimerRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{
 *  EventLoopTimerProcPtr
 *  
 *  Discussion:
 *    Called when a timer fires.
 *  
 *  Parameters:
 *    
 *    inTimer:
 *      The timer that fired.
 *    
 *    inUserData:
 *      The data passed into InstallEventLoopTimer.
 }
type
	EventLoopTimerProcPtr = procedure( inTimer: EventLoopTimerRef; inUserData: UnivPtr );

{
 *  Discussion:
 *    Event Loop Idle Timer Messages
 }
const
{
   * The user has gone idle (not touched an input device) for the
   * duration specified in your idle timer. This is the first message
   * you will receive. Start your engines!
   }
	kEventLoopIdleTimerStarted = 1;

  {
   * If you specified an interval on your idle timer, your idle timer
   * proc will be called with this message, letting you know it is
   * merely firing at the interval specified. You will receive this
   * message for the first time at the specified interval after you
   * receive kEventLoopIdleTimerStarted. If you did not specify an
   * interval, this message is not sent.
   }
	kEventLoopIdleTimerIdling = 2;

  {
   * The user is back! Stop everything! This is your cue to stop any
   * processing if you need to.
   }
	kEventLoopIdleTimerStopped = 3;

type
	EventLoopIdleTimerMessage = UInt16;

{
 *  EventLoopIdleTimerProcPtr
 *  
 *  Discussion:
 *    Called when an idle timer fires.
 *  
 *  Parameters:
 *    
 *    inTimer:
 *      The timer that fired.
 *    
 *    inState:
 *      The current state of the timer.
 *    
 *    inUserData:
 *      The data passed into InstallEventLoopTimer.
 }
type
	EventLoopIdleTimerProcPtr = procedure( inTimer: EventLoopTimerRef; inState: EventLoopIdleTimerMessage; inUserData: UnivPtr );
type
	EventLoopTimerUPP = EventLoopTimerProcPtr;
type
	EventLoopIdleTimerUPP = EventLoopIdleTimerProcPtr;
{
 *  NewEventLoopTimerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewEventLoopTimerUPP( userRoutine: EventLoopTimerProcPtr ): EventLoopTimerUPP; external name '_NewEventLoopTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewEventLoopIdleTimerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewEventLoopIdleTimerUPP( userRoutine: EventLoopIdleTimerProcPtr ): EventLoopIdleTimerUPP; external name '_NewEventLoopIdleTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{
 *  DisposeEventLoopTimerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeEventLoopTimerUPP( userUPP: EventLoopTimerUPP ); external name '_DisposeEventLoopTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeEventLoopIdleTimerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeEventLoopIdleTimerUPP( userUPP: EventLoopIdleTimerUPP ); external name '_DisposeEventLoopIdleTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{
 *  InvokeEventLoopTimerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeEventLoopTimerUPP( inTimer: EventLoopTimerRef; inUserData: UnivPtr; userUPP: EventLoopTimerUPP ); external name '_InvokeEventLoopTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeEventLoopIdleTimerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeEventLoopIdleTimerUPP( inTimer: EventLoopTimerRef; inState: EventLoopIdleTimerMessage; inUserData: UnivPtr; userUPP: EventLoopIdleTimerUPP ); external name '_InvokeEventLoopIdleTimerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{
 *  InstallEventLoopTimer()
 *  
 *  Discussion:
 *    Installs a timer onto the event loop specified. The timer can
 *    either fire once or repeatedly at a specified interval depending
 *    on the parameters passed to this function.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEventLoop:
 *      The event loop to add the timer.
 *    
 *    inFireDelay:
 *      The delay before first firing this timer (can be 0, to request
 *      that the timer be fired as soon as control returns to your
 *      event loop). In Mac OS X and CarbonLib 1.5 and later, you may
 *      pass kEventDurationForever to stop the timer from firing at all
 *      until SetEventLoopTimerNextFireTime is used to start it; in
 *      earlier CarbonLibs, to achieve the same effect, just pass zero
 *      and then immediately call SetEventLoopTimerNextFireTime( timer,
 *      kEventDurationForever ) before returning control to your event
 *      loop.
 *    
 *    inInterval:
 *      The timer interval (pass 0 for a one-shot timer, which executes
 *      once but does not repeat). In Mac OS X and CarbonLib 1.5 and
 *      later, you may also pass kEventDurationForever to create a
 *      one-shot timer.
 *    
 *    inTimerProc:
 *      The routine to call when the timer fires.
 *    
 *    inTimerData:
 *      Data to pass to the timer proc when called.
 *    
 *    outTimer:
 *      A reference to the newly installed timer.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InstallEventLoopTimer( inEventLoop: EventLoopRef; inFireDelay: EventTimerInterval; inInterval: EventTimerInterval; inTimerProc: EventLoopTimerUPP; inTimerData: UnivPtr; var outTimer: EventLoopTimerRef ): OSStatus; external name '_InstallEventLoopTimer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  InstallEventLoopIdleTimer()
 *  
 *  Discussion:
 *    Installs a timer onto the event loop specified. Idle timers are
 *    only called when there is no user activity occuring in the
 *    application. This means that the user is not actively
 *    clicking/typing, and is also not in the middle of tracking a
 *    control, menu, or window. TrackMouseLocation actually disables
 *    all idle timers automatically for you.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inEventLoop:
 *      The event loop to add the timer.
 *    
 *    inDelay:
 *      The delay before firing this timer after a user input event has
 *      come in. For example, if you want to start your timer 2 seconds
 *      after the user stops typing, etc. you would pass 2.0 into this
 *      parameter. Each time the user types a key (or whatever), this
 *      timer is reset. If we are considered to be idle when an idle
 *      timer is installed, the first time it fires will be inDelay
 *      seconds from the time it is installed. So if you installed it
 *      in the middle of control tracking, say, it wouldn't fire until
 *      the user stopped tracking. But if you installed it at app
 *      startup and the user hasn't typed/clicked, it would fire in
 *      inDelay seconds. On Mac OS X 10.3 and earlier, the delay must
 *      be greater than zero. On Mac OS X 10.4 and later, the delay
 *      must be greather than or equal to zero. You cannot use
 *      kEventDurationForever for the delay.
 *    
 *    inInterval:
 *      The timer interval (pass 0 for a one-shot timer, which executes
 *      once but does not repeat). You may also pass
 *      kEventDurationForever to create a one-shot timer.
 *    
 *    inTimerProc:
 *      The routine to call when the timer fires.
 *    
 *    inTimerData:
 *      Data to pass to the timer proc when called.
 *    
 *    outTimer:
 *      A reference to the newly installed timer.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function InstallEventLoopIdleTimer( inEventLoop: EventLoopRef; inDelay: EventTimerInterval; inInterval: EventTimerInterval; inTimerProc: EventLoopIdleTimerUPP; inTimerData: UnivPtr; var outTimer: EventLoopTimerRef ): OSStatus; external name '_InstallEventLoopIdleTimer';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  RemoveEventLoopTimer()
 *  
 *  Discussion:
 *    Removes a timer that was previously installed by a call to
 *    InstallEventLoopTimer. You call this function when you are done
 *    using a timer.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inTimer:
 *      The timer to remove.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveEventLoopTimer( inTimer: EventLoopTimerRef ): OSStatus; external name '_RemoveEventLoopTimer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetEventLoopTimerNextFireTime()
 *  
 *  Discussion:
 *    This routine is used to 'reset' a timer. It controls the next
 *    time the timer fires. This will override any interval you might
 *    have set. For example, if you have a timer that fires every
 *    second, and you call this function setting the next time to five
 *    seconds from now, the timer will sleep for five seconds, then
 *    fire. It will then resume its one-second interval after that. It
 *    is as if you removed the timer and reinstalled it with a new
 *    first-fire delay.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    inTimer:
 *      The timer to adjust
 *    
 *    inNextFire:
 *      The interval from the current time to wait until firing the
 *      timer again. You may pass kEventDurationForever to stop the
 *      timer from firing at all.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetEventLoopTimerNextFireTime( inTimer: EventLoopTimerRef; inNextFire: EventTimerInterval ): OSStatus; external name '_SetEventLoopTimerNextFireTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{======================================================================================}
{  EVENT HANDLERS                                                                      }
{======================================================================================}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	EventHandlerRef = ^OpaqueEventHandlerRef; { an opaque type }
	OpaqueEventHandlerRef = record end;
	EventHandlerRefPtr = ^EventHandlerRef;
	EventHandlerCallRef = ^OpaqueEventHandlerCallRef; { an opaque type }
	OpaqueEventHandlerCallRef = record end;
=======
	EventHandlerRef = ^SInt32; { an opaque type }
	EventHandlerRefPtr = ^EventHandlerRef;
	EventHandlerCallRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventHandlerRef = ^SInt32; { an opaque type }
	EventHandlerRefPtr = ^EventHandlerRef;
	EventHandlerCallRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventHandlerRef = ^SInt32; { an opaque type }
	EventHandlerRefPtr = ^EventHandlerRef;
	EventHandlerCallRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventHandlerRef = ^SInt32; { an opaque type }
	EventHandlerRefPtr = ^EventHandlerRef;
	EventHandlerCallRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	EventHandlerRef = ^SInt32; { an opaque 32-bit type }
	EventHandlerRefPtr = ^EventHandlerRef;
	EventHandlerCallRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	EventHandlerRef = ^SInt32; { an opaque 32-bit type }
	EventHandlerRefPtr = ^EventHandlerRef;
	EventHandlerCallRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	EventHandlerRef = ^SInt32; { an opaque type }
	EventHandlerRefPtr = ^EventHandlerRef;
	EventHandlerCallRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	EventHandlerCallRefPtr = ^EventHandlerCallRef;

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ EventHandler specification                                                        }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ C++ Methods as Event Handlers                                                     }
{ To use a C++ method as an Event Handler callback, it must be declared in its class   }
{ as a static method.  Otherwise, the implicit "this" parameter will make the function }
{ not match the EventHandlerProcPtr prototype.                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  EventHandlerProcPtr
 *  
 *  Discussion:
 *    Callback for receiving events sent to a target this callback is
 *    installed on.
 *  
 *  Parameters:
 *    
 *    inHandlerCallRef:
 *      A reference to the current handler call chain. This is sent to
 *      your handler so that you can call CallNextEventHandler if you
 *      need to.
 *    
 *    inEvent:
 *      The Event.
 *    
 *    inUserData:
 *      The app-specified data you passed in a call to
 *      InstallEventHandler.
 *  
 *  Result:
 *    An operating system result code. Returning noErr indicates you
 *    handled the event. Returning eventNotHandledErr indicates you did
 *    not handle the event and perhaps the toolbox should take other
 *    action.
 }
type
	EventHandlerProcPtr = function( inHandlerCallRef: EventHandlerCallRef; inEvent: EventRef; inUserData: UnivPtr ): OSStatus;
type
	EventHandlerUPP = EventHandlerProcPtr;
{
 *  NewEventHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewEventHandlerUPP( userRoutine: EventHandlerProcPtr ): EventHandlerUPP; external name '_NewEventHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeEventHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeEventHandlerUPP( userUPP: EventHandlerUPP ); external name '_DisposeEventHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeEventHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeEventHandlerUPP( inHandlerCallRef: EventHandlerCallRef; inEvent: EventRef; inUserData: UnivPtr; userUPP: EventHandlerUPP ): OSStatus; external name '_InvokeEventHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	EventTargetRef = ^OpaqueEventTargetRef; { an opaque type }
	OpaqueEventTargetRef = record end;
=======
	EventTargetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventTargetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventTargetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	EventTargetRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	EventTargetRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	EventTargetRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	EventTargetRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Installing Event Handlers                                                         }
{ Use these routines to install event handlers for a specific toolbox object. You may  }
{ pass zero for inNumTypes and NULL for inList if you need to be in a situation where  }
{ you know you will be receiving events, but not exactly which ones at the time you    }
{ are installing the handler. Later, your application can call the Add/Remove routines }
{ listed below this section.                                                           }
{ You can only install a specific handler once. The combination of inHandler and       }
{ inUserData is considered the 'signature' of a handler. Any attempt to install a new  }
{ handler with the same proc and user data as an already-installed handler will result }
{ in eventHandlerAlreadyInstalledErr. Installing the same proc and user data on a      }
{ different object is legal.                                                           }
{ Upon successful completion of this routine, you are returned an EventHandlerRef,     }
{ which you can use in various other calls. It is not possible to retrieve any         }
{ information from an EventHandlerRef about which object the handler is attached to;   }
{ to keep track of the target object of an event handler, use the inUserData paramter  }
{ to InstallEventHandler to specify the object.                                        }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  InstallEventHandler()
 *  
 *  Discussion:
 *    Installs an event handler on a specified target. Your handler
 *    proc will be called with the events you registered with when an
 *    event of the corresponding type and class are send to the target
 *    you are installing your handler on.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTarget:
 *      The target to register your handler with.
 *    
 *    inHandler:
 *      A pointer to your handler function.
 *    
 *    inNumTypes:
 *      The number of events you are registering for.
 *    
 *    inList:
 *      A pointer to an array of EventTypeSpec entries representing the
 *      events you are interested in.
 *    
 *    inUserData:
 *      The value passed in this parameter is passed on to your event
 *      handler proc when it is called.
 *    
 *    outRef:
 *      Receives an EventHandlerRef, which you can use later to remove
 *      the handler. You can pass null if you don't want the reference
 *      - when the target is disposed, the handler will be disposed as
 *      well.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InstallEventHandler( inTarget: EventTargetRef; inHandler: EventHandlerUPP; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr; inUserData: UnivPtr; outRef: EventHandlerRefPtr { can be NULL } ): OSStatus; external name '_InstallEventHandler';
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveEventHandler()
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======
 *  
 *  Summary:
 *    Removes an event handler from the target it was bound to.
 *  
 *  Discussion:
 *    As of Mac OS X 10.1, it is safe to remove an event handler from
 *    inside the handler function. This is not safe to do in CarbonLib
 *    or earlier releases of Mac OS X.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inHandlerRef:
 *      The handler ref to remove (returned in a call to
 *      InstallEventHandler). After you call this function, the handler
 *      ref is considered to be invalid and can no longer be used.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveEventHandler( inHandlerRef: EventHandlerRef ): OSStatus; external name '_RemoveEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  InstallStandardEventHandler()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Removes an event handler from the target it was bound to.
 *  
 *  Discussion:
<<<<<<< HEAD
 *    As of Mac OS X 10.1, it is safe to remove an event handler from
 *    inside the handler function. This is not safe to do in CarbonLib
 *    or earlier releases of Mac OS X.
=======
 *    All event targets have default handlers installed on them by the
 *    toolbox to perform certain basic operations common to that type
 *    of target. Some targets also have standard handlers which are not
 *    installed by default, but may be requested. A standard handler
 *    typically provides higher-level behavior for its target. Prior to
 *    Mac OS X 10.5, only  window event targets have a standard
 *    handler; the window standard event hander may also be installed
 *    by setting the kWindowStandardHandlerAttribute flag. In Mac OS X
 *    10.5 and later, the application and menubar event targets also
 *    support standard event handlers. Calling
 *    InstallStandardEventHandler on any other type of target (control,
 *    menu, etc.) has no effect. 
 *    
 *    In Mac OS X 10.5 and later, InstallStandardEventHandler records
 *    the number of installation requests, and
 *    RemoveStandardEventHandler does not actually remove the standard
 *    handler until the count has been reduced to zero.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inHandlerRef:
 *      The handler ref to remove (returned in a call to
 *      InstallEventHandler). After you call this function, the handler
 *      ref is considered to be invalid and can no longer be used.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveEventHandler( inHandlerRef: EventHandlerRef ): OSStatus; external name '_RemoveEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  InstallStandardEventHandler()
>>>>>>> graemeg/cpstrnew
=======
 *  RemoveStandardEventHandler()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Removes the standard event handler (if any) for an event target.
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
 *    As of Mac OS X 10.1, it is safe to remove an event handler from
 *    inside the handler function. This is not safe to do in CarbonLib
 *    or earlier releases of Mac OS X.
=======
=======
=======
>>>>>>> origin/fixes_2_2
function InstallEventHandler( inTarget: EventTargetRef; inHandler: EventHandlerUPP; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr; inUserData: UnivPtr; outRef: EventHandlerRefPtr { can be NULL } ): OSStatus; external name '_InstallEventHandler';
=======
function InstallEventHandler( inTarget: EventTargetRef; inHandler: EventHandlerUPP; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr; inUserData: UnivPtr; outRef: EventHandlerRefPtr { can be NULL } ): OSStatus; external name '_InstallEventHandler';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveEventHandler()
 *  
 *  Summary:
 *    Removes an event handler from the target it was bound to.
 *  
 *  Discussion:
 *    As of Mac OS X 10.1, it is safe to remove an event handler from
 *    inside the handler function. This is not safe to do in CarbonLib
 *    or earlier releases of Mac OS X.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inHandlerRef:
 *      The handler ref to remove (returned in a call to
 *      InstallEventHandler). After you call this function, the handler
 *      ref is considered to be invalid and can no longer be used.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveEventHandler( inHandlerRef: EventHandlerRef ): OSStatus; external name '_RemoveEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  InstallStandardEventHandler()
 *  
 *  Summary:
 *    Installs the standard event handler (if any) for an event target.
 *  
 *  Discussion:
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *    All event targets have default handlers installed on them by the
 *    toolbox to perform certain basic operations common to that type
 *    of target. Some targets also have standard handlers which are not
 *    installed by default, but may be requested. A standard handler
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    typically provides higher-level behavior for its target. Prior to
 *    Mac OS X 10.5, only  window event targets have a standard
 *    handler; the window standard event hander may also be installed
 *    by setting the kWindowStandardHandlerAttribute flag. In Mac OS X
 *    10.5 and later, the application and menubar event targets also
 *    support standard event handlers. Calling
 *    InstallStandardEventHandler on any other type of target (control,
 *    menu, etc.) has no effect. 
 *    
 *    In Mac OS X 10.5 and later, InstallStandardEventHandler records
 *    the number of installation requests, and
 *    RemoveStandardEventHandler does not actually remove the standard
 *    handler until the count has been reduced to zero.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
 *    InstallStandardEventHandler records the number of installation
 *    requests, and RemoveStandardEventHandler does not actually remove
 *    the standard handler until the count has been reduced to zero.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    typically provides higher-level behavior for its target.
 *    Currently, only window event targets have a standard handler; the
 *    window standard event hander may also be installed by setting the
 *    kWindowStandardHandlerAttribute flag. Calling
 *    InstallStandardEventHandler on any other type of target
 *    (application, control, menu, etc.) has no effect.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTarget:
<<<<<<< HEAD
<<<<<<< HEAD
 *      The target whose standard handler should be removed.
=======
 *      The target whose standard handler should be installed.
>>>>>>> graemeg/fixes_2_2
=======
 *      The target whose standard handler should be installed.
>>>>>>> origin/fixes_2_2
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveEventHandler( inHandlerRef: EventHandlerRef ): OSStatus; external name '_RemoveEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  InstallStandardEventHandler()
>>>>>>> graemeg/cpstrnew
=======
 *  RemoveStandardEventHandler()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Removes the standard event handler (if any) for an event target.
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
 *    As of Mac OS X 10.1, it is safe to remove an event handler from
 *    inside the handler function. This is not safe to do in CarbonLib
 *    or earlier releases of Mac OS X.
=======
 *    All event targets have default handlers installed on them by the
 *    toolbox to perform certain basic operations common to that type
 *    of target. Some targets also have standard handlers which are not
 *    installed by default, but may be requested. A standard handler
 *    typically provides higher-level behavior for its target. Prior to
 *    Mac OS X 10.5, only  window event targets have a standard
 *    handler; the window standard event hander may also be installed
 *    by setting the kWindowStandardHandlerAttribute flag. In Mac OS X
 *    10.5 and later, the application and menubar event targets also
 *    support standard event handlers. Calling
 *    InstallStandardEventHandler on any other type of target (control,
 *    menu, etc.) has no effect. 
 *    
 *    In Mac OS X 10.5 and later, InstallStandardEventHandler records
 *    the number of installation requests, and
 *    RemoveStandardEventHandler does not actually remove the standard
 *    handler until the count has been reduced to zero.
>>>>>>> graemeg/cpstrnew
=======
 *    InstallStandardEventHandler records the number of installation
 *    requests, and RemoveStandardEventHandler does not actually remove
 *    the standard handler until the count has been reduced to zero.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InstallStandardEventHandler( inTarget: EventTargetRef ): OSStatus; external name '_InstallStandardEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveStandardEventHandler()
 *  
 *  Summary:
 *    Removes the standard event handler (if any) for an event target.
 *  
 *  Discussion:
<<<<<<< HEAD
 *    As of Mac OS X 10.1, it is safe to remove an event handler from
 *    inside the handler function. This is not safe to do in CarbonLib
 *    or earlier releases of Mac OS X.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    InstallStandardEventHandler records the number of installation
 *    requests, and RemoveStandardEventHandler does not actually remove
 *    the standard handler until the count has been reduced to zero.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inTarget:
 *      The target whose standard handler should be removed.
=======
=======
>>>>>>> origin/fixes_2_2
 *    inHandlerRef:
 *      The handler ref to remove (returned in a call to
 *      InstallEventHandler). After you call this function, the handler
 *      ref is considered to be invalid and can no longer be used.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    inTarget:
 *      The target whose standard handler should be removed.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveEventHandler( inHandlerRef: EventHandlerRef ): OSStatus; external name '_RemoveEventHandler';
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RemoveStandardEventHandler( inTarget: EventTargetRef ): OSStatus; external name '_RemoveStandardEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{$ifc not TARGET_CPU_64}
{
 *  RemoveStandardEventHandler()
 *  
 *  Summary:
 *    Removes the standard event handler (if any) for an event target.
 *  
 *  Discussion:
 *    InstallStandardEventHandler records the number of installation
 *    requests, and RemoveStandardEventHandler does not actually remove
 *    the standard handler until the count has been reduced to zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTarget:
 *      The target whose standard handler should be removed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RemoveStandardEventHandler( inTarget: EventTargetRef ): OSStatus; external name '_RemoveStandardEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  InstallStandardEventHandler()
 *  
 *  Summary:
 *    Installs the standard event handler (if any) for an event target.
 *  
 *  Discussion:
 *    All event targets have default handlers installed on them by the
 *    toolbox to perform certain basic operations common to that type
 *    of target. Some targets also have standard handlers which are not
 *    installed by default, but may be requested. A standard handler
 *    typically provides higher-level behavior for its target. Prior to
 *    Mac OS X 10.5, only  window event targets have a standard
 *    handler; the window standard event hander may also be installed
 *    by setting the kWindowStandardHandlerAttribute flag. In Mac OS X
 *    10.5 and later, the application and menubar event targets also
 *    support standard event handlers. Calling
 *    InstallStandardEventHandler on any other type of target (control,
 *    menu, etc.) has no effect. 
 *    
 *    In Mac OS X 10.5 and later, InstallStandardEventHandler records
 *    the number of installation requests, and
 *    RemoveStandardEventHandler does not actually remove the standard
 *    handler until the count has been reduced to zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTarget:
 *      The target whose standard handler should be installed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InstallStandardEventHandler( inTarget: EventTargetRef ): OSStatus; external name '_InstallStandardEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RemoveStandardEventHandler( inTarget: EventTargetRef ): OSStatus; external name '_RemoveStandardEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  RemoveStandardEventHandler()
 *  
 *  Summary:
 *    Removes the standard event handler (if any) for an event target.
 *  
 *  Discussion:
 *    InstallStandardEventHandler records the number of installation
 *    requests, and RemoveStandardEventHandler does not actually remove
 *    the standard handler until the count has been reduced to zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTarget:
 *      The target whose standard handler should be removed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RemoveStandardEventHandler( inTarget: EventTargetRef ): OSStatus; external name '_RemoveStandardEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RemoveStandardEventHandler( inTarget: EventTargetRef ): OSStatus; external name '_RemoveStandardEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Adjusting set of event types after a handler is created                           }
{ After installing a handler with the routine above, you can adjust the event type     }
{ list telling the toolbox what events to send to that handler by calling the two      }
{ routines below. If you add an event type twice for the same handler, your handler    }
{ will only be called once, but it will take two RemoveEventType calls to stop your    }
{ handler from being called with that event type. In other words, the install count    }
{ for each event type is maintained by the toolbox. This might allow you, for example  }
{ to have subclasses of a window object register for types without caring if the base  }
{ class has already registered for that type. When the subclass removes its types, it  }
{ can successfully do so without affecting the base class's reception of its event     }
{ types, yielding eternal bliss.                                                       }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  AddEventTypesToHandler()
 *  
 *  Discussion:
 *    Adds additional events to an event handler that has already been
 *    installed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inHandlerRef:
 *      The event handler to add the additional events to.
 *    
 *    inNumTypes:
 *      The number of events to add.
 *    
 *    inList:
 *      A pointer to an array of EventTypeSpec entries.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AddEventTypesToHandler( inHandlerRef: EventHandlerRef; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_AddEventTypesToHandler';
=======
function AddEventTypesToHandler( inHandlerRef: EventHandlerRef; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_AddEventTypesToHandler';
>>>>>>> graemeg/fixes_2_2
=======
function AddEventTypesToHandler( inHandlerRef: EventHandlerRef; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_AddEventTypesToHandler';
>>>>>>> origin/fixes_2_2
=======
function AddEventTypesToHandler( inHandlerRef: EventHandlerRef; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_AddEventTypesToHandler';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveEventTypesFromHandler()
 *  
 *  Discussion:
 *    Removes events from an event handler that has already been
 *    installed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inHandlerRef:
 *      The event handler to remove the events from.
 *    
 *    inNumTypes:
 *      The number of events to remove.
 *    
 *    inList:
 *      A pointer to an array of EventTypeSpec entries.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function RemoveEventTypesFromHandler( inHandlerRef: EventHandlerRef; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_RemoveEventTypesFromHandler';
=======
function RemoveEventTypesFromHandler( inHandlerRef: EventHandlerRef; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_RemoveEventTypesFromHandler';
>>>>>>> graemeg/fixes_2_2
=======
function RemoveEventTypesFromHandler( inHandlerRef: EventHandlerRef; inNumTypes: UInt32; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_RemoveEventTypesFromHandler';
>>>>>>> origin/fixes_2_2
=======
function RemoveEventTypesFromHandler( inHandlerRef: EventHandlerRef; inNumTypes: ItemCount; {const} inList: {variable-size-array} EventTypeSpecPtr ): OSStatus; external name '_RemoveEventTypesFromHandler';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Explicit Propagation                                                              }
{  CallNextEventHandler can be used to call thru to all handlers below the current     }
{  handler being called. You pass the EventHandlerCallRef passed to your EventHandler  }
{  into this call so that we know how to properly forward the event. The result of     }
{  this function should normally be the result of your own handler that you called     }
{  this API from. The typical use of this routine would be to allow the toolbox to do  }
{  its standard processing and then follow up with some type of embellishment.         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  CallNextEventHandler()
 *  
 *  Discussion:
 *    Calls thru to the event handlers below you in the event handler
 *    stack of the target to which your handler is bound. You might use
 *    this to call thru to the default toolbox handling in order to
 *    post-process the event. You can only call this routine from
 *    within an event handler.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inCallRef:
 *      The event handler call ref passed into your event handler.
 *    
 *    inEvent:
 *      The event to pass thru.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CallNextEventHandler( inCallRef: EventHandlerCallRef; inEvent: EventRef ): OSStatus; external name '_CallNextEventHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Sending Events                                                                    }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  Summary:
 *    Options for the SendEventToEventTargetWithOptions API.
 }
const
{
   * The event should be sent to the target given only, and should not
   * propagate to any other target. CallNextEventHandler, when passed
   * an event sent with this option, will only call other event
   * handlers installed on the current event target; it will not
   * propagate the event to other event targets.
   }
	kEventTargetDontPropagate = 1 shl 0;

  {
   * The event is a notification-style event, and should be received by
   * all handlers. The result is usually meaningless when sent in this
   * manner, though we do maintain the strongest result code while the
   * event falls through each handler. This means that if the first
   * handler to receive the event returned noErr, and the next returned
   * eventNotHandledErr, the result returned would actually be noErr.
   * No handler can stop this event from propagating; i.e., the result
   * code does not alter event flow.
   }
	kEventTargetSendToAllHandlers = 1 shl 1;

{
 *  SendEventToEventTarget()
 *  
 *  Discussion:
 *    Sends an event to the specified event target.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to send.
 *    
 *    inTarget:
 *      The target to send it to.
 *  
 *  Result:
 *    An operating system result code. The result is determined by both
 *    the SendEventToEventTarget API and also the event handlers that
 *    receive the event. SendEventToEventTarget will return paramErr if
 *    the event or the target are invalid, or eventNotHandledErr if the
 *    event is not wanted by any handler. If the event is received by a
 *    handler, however, then the result code returned by the API is
 *    determined by the handler; a handler may return any error code,
 *    and your code should not make any assumptions about exactly which
 *    errors will be returned by SendEventToEventTarget.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SendEventToEventTarget( inEvent: EventRef; inTarget: EventTargetRef ): OSStatus; external name '_SendEventToEventTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SendEventToEventTargetWithOptions()
 *  
 *  Discussion:
 *    Sends an event to the specified event target, optionally
 *    controlling how the event propagates. See the discussion of the
 *    event send options above for more detail.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to send.
 *    
 *    inTarget:
 *      The target to send it to.
 *    
 *    inOptions:
 *      The options to modify the send behavior. Passing zero for this
 *      makes it behave just like SendEventToEventTarget.
 *  
 *  Result:
 *    An operating system result code. The result is determined by both
 *    the SendEventToEventTargetWithOptions API and also the event
 *    handlers that receive the event.
 *    SendEventToEventTargetWithOptions will return paramErr if the
 *    event or the target are invalid, or eventNotHandledErr if the
 *    event is not wanted by any handler. If the event is received by a
 *    handler, however, then the result code returned by the API is
 *    determined by the handler; a handler may return any error code,
 *    and your code should not make any assumptions about exactly which
 *    errors will be returned by SendEventToEventTargetWithOptions.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function SendEventToEventTargetWithOptions( inEvent: EventRef; inTarget: EventTargetRef; inOptions: OptionBits ): OSStatus; external name '_SendEventToEventTargetWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Secure Event Input                                                                }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  EnableSecureEventInput()
 *  
 *  Summary:
 *    Enables secure event input mode.
 *  
 *  Discussion:
 *    When secure event input is enabled, keyboard input will only go
 *    to the application with keyboard focus, and will not be echoed to
 *    other applications that might be using the event monitor target
 *    to watch keyboard input. The EditText and EditUnicodeText
 *    controls automatically enter secure input mode when a password
 *    control has the focus; if your application implements its own
 *    password entry, you should enable secure event input while the
 *    user is entering text.
 *    
 *    This API maintains a count of the number of times that it has
 *    been called. Secure event input is not disabled until
 *    DisableSecureEventInput has been called the same number of
 *    times.
 *    
 *    Be sure to disable secure event input if your application becomes
 *    inactive. If your application crashes, secure event input will
 *    automatically be disabled if no other application has enabled it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function EnableSecureEventInput: OSStatus; external name '_EnableSecureEventInput';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  DisableSecureEventInput()
 *  
 *  Summary:
 *    Disables secure event input mode.
 *  
 *  Discussion:
 *    When secure event input is enabled, keyboard input will only go
 *    to the application with keyboard focus, and will not be echoed to
 *    other applications that might be using the event monitor target
 *    to watch keyboard input. The EditText and EditUnicodeText
 *    controls automatically enter secure input mode when a password
 *    control has the focus; if your application implements its own
 *    password entry, you should enable secure event input while the
 *    user is entering text.
 *    
 *    The EnableSecureEventInput API maintains a count of the number of
 *    times that it has been called. Secure event input is not disabled
 *    until this API has been called the same number of times.
 *    
 *    Be sure to disable secure event input if your application becomes
 *    inactive. If your application crashes, secure event input will
 *    automatically be disabled if no other application has enabled it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function DisableSecureEventInput: OSStatus; external name '_DisableSecureEventInput';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  IsSecureEventInputEnabled()
 *  
 *  Summary:
 *    Indicates whether secure event input is currently enabled.
 *  
 *  Discussion:
 *    This API returns whether secure event input is enabled by any
 *    process, not just the current process. Secure event input may be
 *    disabled in the current process but enabled in some other
 *    process; in that case, this API will return true.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function IsSecureEventInputEnabled: Boolean; external name '_IsSecureEventInputEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

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
