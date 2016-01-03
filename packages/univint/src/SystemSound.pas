<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
     File:       OSServices/SystemSound.h
 
<<<<<<< HEAD
     Contains:   *** DEPRECATED *** SystemSound include file
 
     Copyright:  (c) 2000-2011 Apple Inc. All rights reserved.
=======
     Contains:   SystemSound include file
 
     Version:    OSServices-352~2
 
     Copyright:  © 2000-2008 by Apple Computer, Inc., all rights reserved.
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
=======
=======
>>>>>>> origin/fixes_2_2
(*
=======
{
>>>>>>> origin/fixes_2.4
     File:       OSServices/SystemSound.h
 
     Contains:   SystemSound include file
 
     Version:    OSServices-352~2
 
<<<<<<< HEAD
     Copyright:  © 2000-2005 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 2000-2008 by Apple Computer, Inc., all rights reserved.
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
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2005 }
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
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2005 }
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

unit SystemSound;
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
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,Files,CFBase,CFRunLoop;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
{$ALIGN POWER}


{ ================================================================================ }
{ Errors                                                                           }
{ ================================================================================ }
=======
=======
>>>>>>> origin/fixes_2_2
{$ALIGN POWER}


(* ================================================================================ *)
(* Errors                                                                           *)
(* ================================================================================ *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ALIGN POWER}


{ ================================================================================ }
{ Errors                                                                           }
{ ================================================================================ }
>>>>>>> origin/fixes_2.4
const
	kSystemSoundNoError = 0;
	kSystemSoundUnspecifiedError = -1500;
	kSystemSoundClientTimedOutError = -1501;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ ================================================================================ }
{ Types                                                                            }
{ ================================================================================ }
type
	SystemSoundActionID = UInt32;
	SystemSoundCompletionProcPtr = function( actionID: SystemSoundActionID; userData: UnivPtr ): OSStatus;
type
	SystemSoundCompletionUPP = SystemSoundCompletionProcPtr;
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  NewSystemSoundCompletionUPP()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  NewSystemSoundCompletionUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewSystemSoundCompletionUPP( userRoutine: SystemSoundCompletionProcPtr ): SystemSoundCompletionUPP; external name '_NewSystemSoundCompletionUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)

{
 *  DisposeSystemSoundCompletionUPP()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
>>>>>>> origin/cpstrnew
 *  DisposeSystemSoundCompletionUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeSystemSoundCompletionUPP( userUPP: SystemSoundCompletionUPP ); external name '_DisposeSystemSoundCompletionUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)

{
 *  InvokeSystemSoundCompletionUPP()   *** DEPRECATED ***
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
>>>>>>> origin/cpstrnew
 *  InvokeSystemSoundCompletionUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeSystemSoundCompletionUPP( actionID: SystemSoundActionID; userData: UnivPtr; userUPP: SystemSoundCompletionUPP ): OSStatus; external name '_InvokeSystemSoundCompletionUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew

{ ================================================================================ }
{ Public APIs                                                                      }
{ ================================================================================ }
{
 *  AlertSoundPlay()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesPlayAlertSound(). Found in
 *    <AudioToolbox/AudioServices.h>
=======
=======
>>>>>>> origin/fixes_2_2
(* ================================================================================ *)
(* Types                                                                            *)
(* ================================================================================ *)
=======
{ ================================================================================ }
{ Types                                                                            }
{ ================================================================================ }
>>>>>>> origin/fixes_2.4
type
	SystemSoundActionID = UInt32;
	SystemSoundCompletionProcPtr = function( actionID: SystemSoundActionID; userData: UnivPtr ): OSStatus;
type
	SystemSoundCompletionUPP = SystemSoundCompletionProcPtr;
{
 *  NewSystemSoundCompletionUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewSystemSoundCompletionUPP( userRoutine: SystemSoundCompletionProcPtr ): SystemSoundCompletionUPP; external name '_NewSystemSoundCompletionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeSystemSoundCompletionUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeSystemSoundCompletionUPP( userUPP: SystemSoundCompletionUPP ); external name '_DisposeSystemSoundCompletionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  InvokeSystemSoundCompletionUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeSystemSoundCompletionUPP( actionID: SystemSoundActionID; userData: UnivPtr; userUPP: SystemSoundCompletionUPP ): OSStatus; external name '_InvokeSystemSoundCompletionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

<<<<<<< HEAD
(* ================================================================================ *)
(* Public APIs                                                                      *)
(* ================================================================================ *)
(*
 *  AlertSoundPlay()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ ================================================================================ }
{ Public APIs                                                                      }
{ ================================================================================ }
{
 *  AlertSoundPlay()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesPlayAlertSound(). Found in
 *    <AudioToolbox/AudioServices.h>
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Play an Alert Sound
 *  
 *  Discussion:
 *    Play the user's current alert sound, interrupting any previously
 *    playing alert sound.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
procedure AlertSoundPlay; external name '_AlertSoundPlay';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  AlertSoundPlayCustomSound()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesPlayAlertSound(). Found in
 *    <AudioToolbox/AudioServices.h>
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
procedure AlertSoundPlay; external name '_AlertSoundPlay';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


<<<<<<< HEAD
(*
 *  AlertSoundPlayCustomSound()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  AlertSoundPlayCustomSound()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesPlayAlertSound(). Found in
 *    <AudioToolbox/AudioServices.h>
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Play a User designated Alert Sound
 *  
 *  Discussion:
 *    Play a sound, designated by a SystemSoundActionID, with the
 *    behavior of AlertSoundPlay().
 *  
 *  Parameters:
 *    
 *    inAction:
 *      A SystemSoundActionID indicating the desired Sound to be played
 *      with AlertSound behavior.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
procedure AlertSoundPlayCustomSound( inAction: SystemSoundActionID ); external name '_AlertSoundPlayCustomSound';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  SystemSoundPlay()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesPlaySystemSound(). Found in
 *    <AudioToolbox/AudioServices.h>
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
procedure AlertSoundPlayCustomSound( inAction: SystemSoundActionID ); external name '_AlertSoundPlayCustomSound';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


<<<<<<< HEAD
(*
 *  SystemSoundPlay()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  SystemSoundPlay()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesPlaySystemSound(). Found in
 *    <AudioToolbox/AudioServices.h>
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Play a System Sound
 *  
 *  Discussion:
 *    Immediately play the sound designated by actionID. Use for one
 *    time actions that do not require a duration or modification
 *    during playback. Sustain loops in the sound will be ignored.
 *  
 *  Parameters:
 *    
 *    inAction:
 *      A SystemSoundActionID indicating the desired System Sound to be
 *      played.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
procedure SystemSoundPlay( inAction: SystemSoundActionID ); external name '_SystemSoundPlay';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  SystemSoundGetActionID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesCreateSystemSoundID(). Found in
 *    <AudioToolbox/AudioServices.h>
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
procedure SystemSoundPlay( inAction: SystemSoundActionID ); external name '_SystemSoundPlay';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


<<<<<<< HEAD
(*
 *  SystemSoundGetActionID()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  SystemSoundGetActionID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesCreateSystemSoundID(). Found in
 *    <AudioToolbox/AudioServices.h>
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Create a 'custom' System Sound by providing an audio file.
 *  
 *  Discussion:
 *    If the user wants to add a sound that can be played via
 *    SystemSoundPlay(), an FSRef for an audio file can be passed and a
 *    SystemSoundActionID, which can be passed to SystemSoundPlay() or
 *    AlertSoundPlayCustomSound(), will be returned. It is important
 *    that SystemSoundRemoveActionID() be called when the action is no
 *    longer needed by the client application so the System Sound
 *    Server can release any resources dedicated to the returned action
 *    id.
 *  
 *  Parameters:
 *    
 *    userFile:
 *      An const FSRef * for the audio file to be used as a System
 *      Sound. Any audio file supported by the AudioFile APIs in the
 *      AudioToolbox framework may be used.
 *    
 *    outAction:
 *      If successful, a SystemSoundActionID will be returned, which in
 *      turn can be passed to SystemSoundPlay().
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SystemSoundGetActionID( const (*var*) userFile: FSRef; var outAction: SystemSoundActionID ): OSStatus; external name '_SystemSoundGetActionID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  SystemSoundRemoveActionID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesDisposeSystemSoundID(). Found in
 *    <AudioToolbox/AudioServices.h>
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SystemSoundGetActionID( const (*var*) userFile: FSRef; var outAction: SystemSoundActionID ): OSStatus; external name '_SystemSoundGetActionID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


<<<<<<< HEAD
(*
 *  SystemSoundRemoveActionID()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  SystemSoundRemoveActionID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesDisposeSystemSoundID(). Found in
 *    <AudioToolbox/AudioServices.h>
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Remove a 'custom' System Sound.
 *  
 *  Discussion:
 *    If the user no longer needs to use the custom system sound that
 *    was created via SystemSoundGetActionID, this function should be
 *    called so the SystemSoundServer can release resources that are no
 *    longer needed.
 *  
 *  Parameters:
 *    
 *    inAction:
 *      A SystemSoundActionID indicating the desired System Sound to be
 *      removed.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SystemSoundRemoveActionID( inAction: SystemSoundActionID ): OSStatus; external name '_SystemSoundRemoveActionID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  SystemSoundSetCompletionRoutine()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesAddSystemSoundCompletion(). Found in
 *    <AudioToolbox/AudioServices.h>
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SystemSoundRemoveActionID( inAction: SystemSoundActionID ): OSStatus; external name '_SystemSoundRemoveActionID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


<<<<<<< HEAD
(*
 *  SystemSoundSetCompletionRoutine()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  SystemSoundSetCompletionRoutine()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesAddSystemSoundCompletion(). Found in
 *    <AudioToolbox/AudioServices.h>
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Call the provided Completion Routine when the provided
 *    SystemSoundActionID finishes playing in the server.
 *  
 *  Discussion:
 *    Once set, the System Sound Server will send a message to the
 *    System Sound Client indicating which SystemSoundActionID has
 *    finished playing.
 *  
 *  Parameters:
 *    
 *    inAction:
 *      The SystemSoundActionID that the completion routine will be
 *      associated with.
 *    
 *    inRunLoop:
 *      A CFRunLoopRef indicating the desired run loop the completion
 *      routine should be run on. Pass NULL for the main run loop.
 *    
 *    inRunLoopMode:
 *      A CFStringRef indicating the run loop mode for the runloop mode
 *      for the runloop where the completion routine will be executed.
 *      Pass NULL to use kCFRunLoopDefaultMode.
 *    
 *    inCompletionRoutine:
 *      A SystemSoundCompletionProc for the completion routine proc to
 *      be called when the provided SystemSoundActionID has completed
 *      playing in the server.
 *    
 *    inUserData:
 *      A void * to pass user data to the completion routine.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SystemSoundSetCompletionRoutine( inAction: SystemSoundActionID; inRunLoop: CFRunLoopRef; inRunLoopMode: CFStringRef; inCompletionRoutine: SystemSoundCompletionUPP; inUserData: UnivPtr ): OSStatus; external name '_SystemSoundSetCompletionRoutine';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  SystemSoundRemoveCompletionRoutine()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesRemoveSystemSoundCompletion(). Found in
 *    <AudioToolbox/AudioServices.h>
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SystemSoundSetCompletionRoutine( inAction: SystemSoundActionID; inRunLoop: CFRunLoopRef; inRunLoopMode: CFStringRef; inCompletionRoutine: SystemSoundCompletionUPP; inUserData: UnivPtr ): OSStatus; external name '_SystemSoundSetCompletionRoutine';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


<<<<<<< HEAD
(*
 *  SystemSoundRemoveCompletionRoutine()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  SystemSoundRemoveCompletionRoutine()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AudioServicesRemoveSystemSoundCompletion(). Found in
 *    <AudioToolbox/AudioServices.h>
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Remove the Completion Routine being used for the provided
 *    SystemSoundActionID.
 *  
 *  Discussion:
 *    To be called when it is no longer desired for the Completion
 *    Routine to be called when a System Sound action has finished
 *    playing.
 *  
 *  Parameters:
 *    
 *    inAction:
 *      A SystemSoundActionID that currently has an associated
 *      completion routine.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
procedure SystemSoundRemoveCompletionRoutine( inAction: SystemSoundActionID ); external name '_SystemSoundRemoveCompletionRoutine';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
procedure SystemSoundRemoveCompletionRoutine( inAction: SystemSoundActionID ); external name '_SystemSoundRemoveCompletionRoutine';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

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
