<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CoreGraphics - CGEventSource.h
   Copyright (c) 2004-2008 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
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
{
*  CGEventSource.h
*  CoreGraphics
*
*  Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
*
}
=======
{ CoreGraphics - CGEventSource.h
   Copyright (c) 2004-2008 Apple Inc.
   All rights reserved. }
>>>>>>> origin/fixes_2.4
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

unit CGEventSource;
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
=======
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
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
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
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
uses MacTypes,CFBase,CFDate,CGRemoteOperation,CGEventTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{$ifc TARGET_OS_MAC}

{ An event source contains accumulated state related to event generation
   and event posting, allowing for customized event generation and
   processing.

   A source state, represented by a `CGEventSourceStateID', refers to a
   global event state table. These tables contain accumulated information on
   modifier flag state, keyboard key state, mouse button state, and related
   internal parameters placed in effect by posting events with associated
   sources.

   Two pre-existing event state tables are defined:

     The `kCGEventSourceStateCombinedSessionState' table reflects the
     combined state of all event sources posting to the current user login
     session. If your program is posting events from within a login session,
     you should use this source state when you create an event source.

     The `kCGEventSourceStateHIDSystemState' table reflects the combined
     state of all hardware event sources posting from the HID system. If
     your program is a daemon or a user space device driver interpreting
     hardware state and generating events, you should use this source state
     when you create an event source.

   Very specialized applications such as remote control programs may want to
   generate and track event source state independent of other processes.
   These programs should use the `kCGEventSourceStatePrivate' value in
   creating their event source. An independent state table and unique source
   state ID (`CGEventSourceStateID') are created to track the event source's
   state. This independent sate table is owned by the creating event source
   and released with it. }

{ Return the CFTypeID for CGEventSourceRefs. }

function CGEventSourceGetTypeID: CFTypeID; external name '_CGEventSourceGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a Quartz event source created with a specified source state. }

function CGEventSourceCreate( stateID: CGEventSourceStateID ): CGEventSourceRef; external name '_CGEventSourceCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the keyboard type to be used with a Quartz event source. }

function CGEventSourceGetKeyboardType( source: CGEventSourceRef ): CGEventSourceKeyboardType; external name '_CGEventSourceGetKeyboardType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the keyboard type to be used with a Quartz event source. }

procedure CGEventSourceSetKeyboardType( source: CGEventSourceRef; keyboardType: CGEventSourceKeyboardType ); external name '_CGEventSourceSetKeyboardType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the scale of pixels per line in a scrolling event source.

   This function returns the scale of pixels per line in the specified event
   source. For example, if the scale in the event source is 10.5 pixels per
   line, this function would return 10.5. Every scrolling event can be
   interpreted to be scrolling by pixel or by line. By default, the scale is
   about ten pixels per line. }

function CGEventSourceGetPixelsPerLine( source: CGEventSourceRef ): Float64; external name '_CGEventSourceGetPixelsPerLine';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Set the scale of pixels per line in a scrolling event source.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

   This function sets the scale of pixels per line in the specified event
   source. For example, if you pass the value 12 as the `pixelsPerLine'
   parameter, the scale of pixels per line in the event source would be
   changed to 12. Every scrolling event can be interpreted to be scrolling
   by pixel or by line. By default, the scale is about ten pixels per
   line. }

procedure CGEventSourceSetPixelsPerLine( source: CGEventSourceRef; pixelsPerLine: Float64 ); external name '_CGEventSourceSetPixelsPerLine';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return the source state associated with a Quartz event source.

   For event sources created with the `kCGEventSourceStatePrivate' source
   state, this function returns the ID of the private source state table
   created for the event source. This unique ID may be passed to the
   `CGEventSourceCreate' function to create a second event source sharing
   the same state table. This may be useful, for example, in creating
   separate mouse and keyboard sources which share a common private state. }

function CGEventSourceGetSourceStateID( source: CGEventSourceRef ): CGEventSourceStateID; external name '_CGEventSourceGetSourceStateID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a Boolean value indicating the current button state of a Quartz
   event source. If true, the button is down; if false, the button is up. }

function CGEventSourceButtonState( stateID: CGEventSourceStateID; button: CGMouseButton ): CBool; external name '_CGEventSourceButtonState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a Boolean value indicating the current keyboard state of a Quartz
   event source. If true, the key is down; if false, the key is up. }

function CGEventSourceKeyState( stateID: CGEventSourceStateID; key: CGKeyCode ): CBool; external name '_CGEventSourceKeyState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the current flags of a Quartz event source. If true, the key is
   down; if false, the key is up. }

function CGEventSourceFlagsState( stateID: CGEventSourceStateID ): CGEventFlags; external name '_CGEventSourceFlagsState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the elapsed time since the last event for a Quartz event source.

   To get the elapsed time since the previous input event --- keyboard,
   mouse, or tablet --- specify `kCGAnyInputEventType'. }

function CGEventSourceSecondsSinceLastEventType( stateID: CGEventSourceStateID; eventType: CGEventType ): CFTimeInterval; external name '_CGEventSourceSecondsSinceLastEventType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a count of events of a given type seen since the Window Server
   started.

   Modifier keys produce `kCGEventFlagsChanged' events, not `kCGEventKeyDown'
   events, and do so both on press and release. The volume, brightness, and
   CD eject keys on some keyboards (both desktop and laptop) do not generate
   key up or key down events.

   For various reasons, the number of key up and key down events may not be
   the same when all keyboard keys are up. As a result, a mismatch does not
   necessarily indicate that some keys are down.

   Key autorepeat events are not counted. }

function CGEventSourceCounterForEventType( stateID: CGEventSourceStateID; eventType: CGEventType ): UInt32; external name '_CGEventSourceCounterForEventType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

=======

=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
   This function sets the scale of pixels per line in the specified event
   source. For example, if you pass the value 12 as the `pixelsPerLine'
   parameter, the scale of pixels per line in the event source would be
   changed to 12. Every scrolling event can be interpreted to be scrolling
   by pixel or by line. By default, the scale is about ten pixels per
   line. }

procedure CGEventSourceSetPixelsPerLine( source: CGEventSourceRef; pixelsPerLine: Float64 ); external name '_CGEventSourceSetPixelsPerLine';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return the source state associated with a Quartz event source.

   For event sources created with the `kCGEventSourceStatePrivate' source
   state, this function returns the ID of the private source state table
   created for the event source. This unique ID may be passed to the
   `CGEventSourceCreate' function to create a second event source sharing
   the same state table. This may be useful, for example, in creating
   separate mouse and keyboard sources which share a common private state. }

function CGEventSourceGetSourceStateID( source: CGEventSourceRef ): CGEventSourceStateID; external name '_CGEventSourceGetSourceStateID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a Boolean value indicating the current button state of a Quartz
   event source. If true, the button is down; if false, the button is up. }

function CGEventSourceButtonState( stateID: CGEventSourceStateID; button: CGMouseButton ): CBool; external name '_CGEventSourceButtonState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a Boolean value indicating the current keyboard state of a Quartz
   event source. If true, the key is down; if false, the key is up. }

function CGEventSourceKeyState( stateID: CGEventSourceStateID; key: CGKeyCode ): CBool; external name '_CGEventSourceKeyState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the current flags of a Quartz event source. If true, the key is
   down; if false, the key is up. }

function CGEventSourceFlagsState( stateID: CGEventSourceStateID ): CGEventFlags; external name '_CGEventSourceFlagsState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the elapsed time since the last event for a Quartz event source.

   To get the elapsed time since the previous input event --- keyboard,
   mouse, or tablet --- specify `kCGAnyInputEventType'. }

function CGEventSourceSecondsSinceLastEventType( stateID: CGEventSourceStateID; eventType: CGEventType ): CFTimeInterval; external name '_CGEventSourceSecondsSinceLastEventType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a count of events of a given type seen since the Window Server
   started.

   Modifier keys produce `kCGEventFlagsChanged' events, not `kCGEventKeyDown'
   events, and do so both on press and release. The volume, brightness, and
   CD eject keys on some keyboards (both desktop and laptop) do not generate
   key up or key down events.

   For various reasons, the number of key up and key down events may not be
   the same when all keyboard keys are up. As a result, a mismatch does not
   necessarily indicate that some keys are down.

   Key autorepeat events are not counted. }

function CGEventSourceCounterForEventType( stateID: CGEventSourceStateID; eventType: CGEventType ): UInt32; external name '_CGEventSourceCounterForEventType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

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
{ Set the 64-bit user-specified data for a Quartz event source.

   Each input event includes 64 bits of user-specified data. This function
   sets the user-specified data for all events created by the specified
   event source. This data may also be set per event using the
   `CGEventGetIntegerValueField' function. }

procedure CGEventSourceSetUserData( source: CGEventSourceRef; userData: SInt64 ); external name '_CGEventSourceSetUserData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the 64-bit user-specified data for a Quartz event source.

   Each input event includes 64 bits of user-specified data. This function
   gets the user-specified data for all events created by the specified
   event source. This data may also be obtained per event using the
   `CGEventGetIntegerValueField' function. }

function CGEventSourceGetUserData( source: CGEventSourceRef ): SInt64; external name '_CGEventSourceGetUserData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the mask that indicates which classes of local hardware events are
   enabled during event suppression.

   By default, the system does not suppress local hardware events from the
   keyboard or mouse during a short interval after a Quartz event is posted
   --- see `CGEventSourceSetLocalEventsSuppressionInterval' --- and during a
   synthetic mouse drag (mouse movement with the left or only mouse button
   down).

   Some applications may want to disable events from some of the local
   hardware during this interval. For example, if you post mouse events
   only, you may wish to suppress local mouse events and permit local
   keyboard events to pass through. This function lets you specify an event
   source, a suppression state (event suppression interval or mouse drag),
   and a filter mask of event classes to be passed through. The new local
   events filter takes effect with the next Quartz event you post using this
   event source. }

procedure CGEventSourceSetLocalEventsFilterDuringSuppressionState( source: CGEventSourceRef; filter: CGEventFilterMask; state: CGEventSuppressionState ); external name '_CGEventSourceSetLocalEventsFilterDuringSuppressionState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)


{ Return the mask that indicates which classes of local hardware events are
   enabled during event suppression.

   You can configure the system to suppress local hardware events from the
   keyboard or mouse during a short interval after a Quartz event is posted
   or during a synthetic mouse drag (mouse movement with the left or only
   mouse button down). For information about setting this local events
   filter, see `CGEventSourceSetLocalEventsFilterDuringSuppressionState'.

   This function lets you specify an event source and a suppression state
   (event suppression interval or mouse drag), and returns a filter mask of
   event categories to be passed through during suppression. }

function CGEventSourceGetLocalEventsFilterDuringSuppressionState( source: CGEventSourceRef; state: CGEventSuppressionState ): CGEventFilterMask; external name '_CGEventSourceGetLocalEventsFilterDuringSuppressionState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the interval that local hardware events may be suppressed following
   the posting of a Quartz event.

   This function sets the period of time in seconds that local hardware
   events may be suppressed after posting a Quartz event created with the
   specified event source. The default suppression interval is 0.25
   seconds. }

procedure CGEventSourceSetLocalEventsSuppressionInterval( source: CGEventSourceRef; seconds: CFTimeInterval ); external name '_CGEventSourceSetLocalEventsSuppressionInterval';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the interval that local hardware events may be suppressed
   following the posting of a Quartz event. }

function CGEventSourceGetLocalEventsSuppressionInterval( source: CGEventSourceRef ): CFTimeInterval; external name '_CGEventSourceGetLocalEventsSuppressionInterval';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{$endc}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

{$ifc TARGET_OS_MAC}

{ An event source contains accumulated state related to event generation
   and event posting, allowing for customized event generation and
   processing.

   A source state, represented by a `CGEventSourceStateID', refers to a
   global event state table. These tables contain accumulated information on
   modifier flag state, keyboard key state, mouse button state, and related
   internal parameters placed in effect by posting events with associated
   sources.

   Two pre-existing event state tables are defined:

     The `kCGEventSourceStateCombinedSessionState' table reflects the
     combined state of all event sources posting to the current user login
     session. If your program is posting events from within a login session,
     you should use this source state when you create an event source.

     The `kCGEventSourceStateHIDSystemState' table reflects the combined
     state of all hardware event sources posting from the HID system. If
     your program is a daemon or a user space device driver interpreting
     hardware state and generating events, you should use this source state
     when you create an event source.

   Very specialized applications such as remote control programs may want to
   generate and track event source state independent of other processes.
   These programs should use the `kCGEventSourceStatePrivate' value in
   creating their event source. An independent state table and unique source
   state ID (`CGEventSourceStateID') are created to track the event source's
   state. This independent sate table is owned by the creating event source
   and released with it. }

{ Return the CFTypeID for CGEventSourceRefs. }

function CGEventSourceGetTypeID: CFTypeID; external name '_CGEventSourceGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a Quartz event source created with a specified source state. }

function CGEventSourceCreate( stateID: CGEventSourceStateID ): CGEventSourceRef; external name '_CGEventSourceCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the keyboard type to be used with a Quartz event source. }

function CGEventSourceGetKeyboardType( source: CGEventSourceRef ): CGEventSourceKeyboardType; external name '_CGEventSourceGetKeyboardType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the keyboard type to be used with a Quartz event source. }

procedure CGEventSourceSetKeyboardType( source: CGEventSourceRef; keyboardType: CGEventSourceKeyboardType ); external name '_CGEventSourceSetKeyboardType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the scale of pixels per line in a scrolling event source.

   This function returns the scale of pixels per line in the specified event
   source. For example, if the scale in the event source is 10.5 pixels per
   line, this function would return 10.5. Every scrolling event can be
   interpreted to be scrolling by pixel or by line. By default, the scale is
   about ten pixels per line. }

function CGEventSourceGetPixelsPerLine( source: CGEventSourceRef ): Float64; external name '_CGEventSourceGetPixelsPerLine';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Set the scale of pixels per line in a scrolling event source.

   This function sets the scale of pixels per line in the specified event
   source. For example, if you pass the value 12 as the `pixelsPerLine'
   parameter, the scale of pixels per line in the event source would be
   changed to 12. Every scrolling event can be interpreted to be scrolling
   by pixel or by line. By default, the scale is about ten pixels per
   line. }

procedure CGEventSourceSetPixelsPerLine( source: CGEventSourceRef; pixelsPerLine: Float64 ); external name '_CGEventSourceSetPixelsPerLine';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return the source state associated with a Quartz event source.

   For event sources created with the `kCGEventSourceStatePrivate' source
   state, this function returns the ID of the private source state table
   created for the event source. This unique ID may be passed to the
   `CGEventSourceCreate' function to create a second event source sharing
   the same state table. This may be useful, for example, in creating
   separate mouse and keyboard sources which share a common private state. }

function CGEventSourceGetSourceStateID( source: CGEventSourceRef ): CGEventSourceStateID; external name '_CGEventSourceGetSourceStateID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a Boolean value indicating the current button state of a Quartz
   event source. If true, the button is down; if false, the button is up. }

function CGEventSourceButtonState( stateID: CGEventSourceStateID; button: CGMouseButton ): CBool; external name '_CGEventSourceButtonState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a Boolean value indicating the current keyboard state of a Quartz
   event source. If true, the key is down; if false, the key is up. }

function CGEventSourceKeyState( stateID: CGEventSourceStateID; key: CGKeyCode ): CBool; external name '_CGEventSourceKeyState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the current flags of a Quartz event source. If true, the key is
   down; if false, the key is up. }

function CGEventSourceFlagsState( stateID: CGEventSourceStateID ): CGEventFlags; external name '_CGEventSourceFlagsState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the elapsed time since the last event for a Quartz event source.

   To get the elapsed time since the previous input event --- keyboard,
   mouse, or tablet --- specify `kCGAnyInputEventType'. }

function CGEventSourceSecondsSinceLastEventType( stateID: CGEventSourceStateID; eventType: CGEventType ): CFTimeInterval; external name '_CGEventSourceSecondsSinceLastEventType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a count of events of a given type seen since the Window Server
   started.

   Modifier keys produce `kCGEventFlagsChanged' events, not `kCGEventKeyDown'
   events, and do so both on press and release. The volume, brightness, and
   CD eject keys on some keyboards (both desktop and laptop) do not generate
   key up or key down events.

   For various reasons, the number of key up and key down events may not be
   the same when all keyboard keys are up. As a result, a mismatch does not
   necessarily indicate that some keys are down.

   Key autorepeat events are not counted. }

function CGEventSourceCounterForEventType( stateID: CGEventSourceStateID; eventType: CGEventType ): UInt32; external name '_CGEventSourceCounterForEventType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the 64-bit user-specified data for a Quartz event source.

   Each input event includes 64 bits of user-specified data. This function
   sets the user-specified data for all events created by the specified
   event source. This data may also be set per event using the
   `CGEventGetIntegerValueField' function. }

procedure CGEventSourceSetUserData( source: CGEventSourceRef; userData: SInt64 ); external name '_CGEventSourceSetUserData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the 64-bit user-specified data for a Quartz event source.

   Each input event includes 64 bits of user-specified data. This function
   gets the user-specified data for all events created by the specified
   event source. This data may also be obtained per event using the
   `CGEventGetIntegerValueField' function. }

function CGEventSourceGetUserData( source: CGEventSourceRef ): SInt64; external name '_CGEventSourceGetUserData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the mask that indicates which classes of local hardware events are
   enabled during event suppression.

   By default, the system does not suppress local hardware events from the
   keyboard or mouse during a short interval after a Quartz event is posted
   --- see `CGEventSourceSetLocalEventsSuppressionInterval' --- and during a
   synthetic mouse drag (mouse movement with the left or only mouse button
   down).

   Some applications may want to disable events from some of the local
   hardware during this interval. For example, if you post mouse events
   only, you may wish to suppress local mouse events and permit local
   keyboard events to pass through. This function lets you specify an event
   source, a suppression state (event suppression interval or mouse drag),
   and a filter mask of event classes to be passed through. The new local
   events filter takes effect with the next Quartz event you post using this
   event source. }

procedure CGEventSourceSetLocalEventsFilterDuringSuppressionState( source: CGEventSourceRef; filter: CGEventFilterMask; state: CGEventSuppressionState ); external name '_CGEventSourceSetLocalEventsFilterDuringSuppressionState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)


{ Return the mask that indicates which classes of local hardware events are
   enabled during event suppression.

   You can configure the system to suppress local hardware events from the
   keyboard or mouse during a short interval after a Quartz event is posted
   or during a synthetic mouse drag (mouse movement with the left or only
   mouse button down). For information about setting this local events
   filter, see `CGEventSourceSetLocalEventsFilterDuringSuppressionState'.

   This function lets you specify an event source and a suppression state
   (event suppression interval or mouse drag), and returns a filter mask of
   event categories to be passed through during suppression. }

function CGEventSourceGetLocalEventsFilterDuringSuppressionState( source: CGEventSourceRef; state: CGEventSuppressionState ): CGEventFilterMask; external name '_CGEventSourceGetLocalEventsFilterDuringSuppressionState';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the interval that local hardware events may be suppressed following
   the posting of a Quartz event.

   This function sets the period of time in seconds that local hardware
   events may be suppressed after posting a Quartz event created with the
   specified event source. The default suppression interval is 0.25
   seconds. }

procedure CGEventSourceSetLocalEventsSuppressionInterval( source: CGEventSourceRef; seconds: CFTimeInterval ); external name '_CGEventSourceSetLocalEventsSuppressionInterval';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the interval that local hardware events may be suppressed
   following the posting of a Quartz event. }

function CGEventSourceGetLocalEventsSuppressionInterval( source: CGEventSourceRef ): CFTimeInterval; external name '_CGEventSourceGetLocalEventsSuppressionInterval';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{$endc}
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
