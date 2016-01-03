{
     File:       HIToolbox/AEInteraction.h
 
     Contains:   AppleEvent functions that deal with Events and interacting with user
 
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
 
     Copyright:  © 2000-2008 by Apple Computer, Inc., all rights reserved.
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 2000-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 2000-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 2000-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit AEInteraction;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
<<<<<<< HEAD
{$elsec}
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
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,QuickdrawTypes,AEDataModel,Notification,Events,CarbonEventsCore;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
=======
uses MacTypes,Quickdraw,AEDataModel,Notification,Events;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,Quickdraw,AEDataModel,Notification,Events;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ALIGN MAC68K}


{*************************************************************************
  AppleEvent callbacks. 
*************************************************************************}
type
	AEIdleProcPtr = function( var theEvent: EventRecord; var sleepTime: SInt32; var mouseRgn: RgnHandle ): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AEFilterProcPtr = function( var theEvent: EventRecord; returnID: SInt32; transactionID: AETransactionID; const (*var*) sender: AEAddressDesc ): Boolean;
	AEIdleUPP = AEIdleProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type
	AEFilterProcPtr = function( var theEvent: EventRecord; returnID: SInt32; transactionID: SInt32; const (*var*) sender: AEAddressDesc ): Boolean;
type
	AEIdleUPP = AEIdleProcPtr;
type
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	AEFilterProcPtr = function( var theEvent: EventRecord; returnID: SInt32; transactionID: AETransactionID; const (*var*) sender: AEAddressDesc ): Boolean;
	AEIdleUPP = AEIdleProcPtr;
>>>>>>> origin/fixes_2.4
	AEFilterUPP = AEFilterProcPtr;

{*************************************************************************
  The next couple of calls are basic routines used to create, send,
  and process AppleEvents. 
*************************************************************************}
{
 *  AESend()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESend( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; sendMode: AESendMode; sendPriority: AESendPriority; timeOutInTicks: SInt32; idleProc: AEIdleUPP { can be NULL }; filterProc: AEFilterUPP { can be NULL } ): OSErr; external name '_AESend';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEProcessAppleEvent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEProcessAppleEvent( const (*var*) theEventRecord: EventRecord ): OSErr; external name '_AEProcessAppleEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  AEProcessEvent()
 *  
 *  Summary:
 *    Dispatches a Carbon event of type kEventAppleEvent to the
 *    appropriate AppleEvent handlers.
 *  
 *  Discussion:
 *    This API is similar to AEProcessAppleEvent, but does not require
 *    the Carbon event to be converted to an EventRecord. Also, unlike
 *    AEProcessAppleEvent, this API does not require that an event be
 *    removed from its event queue before processing; the AppleEvent
 *    will be correctly dispatched even if the Carbon event is still in
 *    its event queue. Of course, you should still remove the Carbon
 *    event from its event queue later once you're done handling it,
 *    and it is also acceptable to remove it from the event queue
 *    before calling this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      A Carbon event of class kEventClassAppleEvent and kind
 *      kEventAppleEvent.
 *  
 *  Result:
 *    The operating system result code returned by the AppleEvent
 *    handler, or paramErr if the event passed to this API is not of
 *    the correct class and kind.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function AEProcessEvent( inEvent: EventRef ): OSStatus; external name '_AEProcessEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ 
 Note: during event processing, an event handler may realize that it is likely
 to exceed the client's timeout limit. Passing the reply to this
 routine causes a wait event to be generated that asks the client
 for more time. 
}
{
 *  AEResetTimer()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEResetTimer( const (*var*) reply: AppleEvent ): OSErr; external name '_AEResetTimer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{*************************************************************************
  The following three calls are used to allow applications to behave
  courteously when a user interaction such as a dialog box is needed. 
*************************************************************************}

type
	AEInteractAllowed = SInt8;
const
	kAEInteractWithSelf = 0;
	kAEInteractWithLocal = 1;
	kAEInteractWithAll = 2;

{
 *  AEGetInteractionAllowed()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetInteractionAllowed( var level: AEInteractAllowed ): OSErr; external name '_AEGetInteractionAllowed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AESetInteractionAllowed()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESetInteractionAllowed( level: AEInteractAllowed ): OSErr; external name '_AESetInteractionAllowed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEInteractWithUser()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEInteractWithUser( timeOutInTicks: SInt32; nmReqPtr: NMRecPtr; idleProc: AEIdleUPP ): OSErr; external name '_AEInteractWithUser';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{*************************************************************************
 The following four calls are available for applications which need more
 sophisticated control over when and how events are processed. Applications
 which implement multi-session servers or which implement their own
 internal event queueing will probably be the major clients of these
 routines. They can be called from within a handler to prevent the AEM from
 disposing of the AppleEvent when the handler returns. They can be used to
 asynchronously process the event (as MacApp does).
*************************************************************************}
{
 *  AESuspendTheCurrentEvent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESuspendTheCurrentEvent( const (*var*) theAppleEvent: AppleEvent ): OSErr; external name '_AESuspendTheCurrentEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ 
 Note: The following routine tells the AppleEvent manager that processing
 is either about to resume or has been completed on a previously suspended
 event. The procPtr passed in as the dispatcher parameter will be called to
 attempt to redispatch the event. Several constants for the dispatcher
 parameter allow special behavior. They are:
    - kAEUseStandardDispatch means redispatch as if the event was just
      received, using the standard AppleEvent dispatch mechanism.
    - kAENoDispatch means ignore the parameter.
      Use this in the case where the event has been handled and no
      redispatch is needed.
    - non nil means call the routine which the dispatcher points to.
}
{ Constants for Refcon in AEResumeTheCurrentEvent with kAEUseStandardDispatch }
const
	kAEDoNotIgnoreHandler = $00000000;
	kAEIgnoreAppPhacHandler = $00000001; { available only in vers 1.0.1 and greater }
	kAEIgnoreAppEventHandler = $00000002; { available only in vers 1.0.1 and greater }
	kAEIgnoreSysPhacHandler = $00000004; { available only in vers 1.0.1 and greater }
	kAEIgnoreSysEventHandler = $00000008; { available only in vers 1.0.1 and greater }
	kAEIngoreBuiltInEventHandler = $00000010; { available only in vers 1.0.1 and greater }
	kAEDontDisposeOnResume = $80000000; { available only in vers 1.0.1 and greater }

{ Constants for AEResumeTheCurrentEvent }
const
	kAENoDispatch = 0;    { dispatch parameter to AEResumeTheCurrentEvent takes a pointer to a dispatch }
	kAEUseStandardDispatch = $FFFFFFFF; { table, or one of these two constants }

{
 *  AEResumeTheCurrentEvent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AEResumeTheCurrentEvent( const (*var*) theAppleEvent: AppleEvent; const (*var*) reply: AppleEvent; dispatcher: AEEventHandlerUPP { can be NULL }; handlerRefcon: SRefCon ): OSErr; external name '_AEResumeTheCurrentEvent';
=======
function AEResumeTheCurrentEvent( const (*var*) theAppleEvent: AppleEvent; const (*var*) reply: AppleEvent; dispatcher: AEEventHandlerUPP { can be NULL }; handlerRefcon: SInt32 ): OSErr; external name '_AEResumeTheCurrentEvent';
>>>>>>> graemeg/fixes_2_2
=======
function AEResumeTheCurrentEvent( const (*var*) theAppleEvent: AppleEvent; const (*var*) reply: AppleEvent; dispatcher: AEEventHandlerUPP { can be NULL }; handlerRefcon: SInt32 ): OSErr; external name '_AEResumeTheCurrentEvent';
>>>>>>> origin/fixes_2_2
=======
function AEResumeTheCurrentEvent( const (*var*) theAppleEvent: AppleEvent; const (*var*) reply: AppleEvent; dispatcher: AEEventHandlerUPP { can be NULL }; handlerRefcon: SRefCon ): OSErr; external name '_AEResumeTheCurrentEvent';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEGetTheCurrentEvent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetTheCurrentEvent( var theAppleEvent: AppleEvent ): OSErr; external name '_AEGetTheCurrentEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AESetTheCurrentEvent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESetTheCurrentEvent( const (*var*) theAppleEvent: AppleEvent ): OSErr; external name '_AESetTheCurrentEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{*************************************************************************
  AppleEvent callbacks. 
*************************************************************************}
{
 *  NewAEIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAEIdleUPP( userRoutine: AEIdleProcPtr ): AEIdleUPP; external name '_NewAEIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewAEFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAEFilterUPP( userRoutine: AEFilterProcPtr ): AEFilterUPP; external name '_NewAEFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeAEIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAEIdleUPP( userUPP: AEIdleUPP ); external name '_DisposeAEIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeAEFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAEFilterUPP( userUPP: AEFilterUPP ); external name '_DisposeAEFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeAEIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAEIdleUPP( var theEvent: EventRecord; var sleepTime: SInt32; var mouseRgn: RgnHandle; userUPP: AEIdleUPP ): Boolean; external name '_InvokeAEIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeAEFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeAEFilterUPP( var theEvent: EventRecord; returnID: SInt32; transactionID: AETransactionID; const (*var*) sender: AEAddressDesc; userUPP: AEFilterUPP ): Boolean; external name '_InvokeAEFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
function InvokeAEFilterUPP( var theEvent: EventRecord; returnID: SInt32; transactionID: SInt32; const (*var*) sender: AEAddressDesc; userUPP: AEFilterUPP ): Boolean; external name '_InvokeAEFilterUPP';
=======
function InvokeAEFilterUPP( var theEvent: EventRecord; returnID: SInt32; transactionID: AETransactionID; const (*var*) sender: AEAddressDesc; userUPP: AEFilterUPP ): Boolean; external name '_InvokeAEFilterUPP';
>>>>>>> origin/fixes_2.4
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
