<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CoreGraphics - CGEvent.h
 * Copyright (c) 2004-2008 Apple Inc.
 * All rights reserved. }
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
 *  CGEvent.h
 *  CoreGraphics
 *
 *  Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
 *
 }
=======
{ CoreGraphics - CGEvent.h
 * Copyright (c) 2004-2008 Apple Inc.
 * All rights reserved. }
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

unit CGEvent;
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
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
uses MacTypes,CFBase,CFData,CFMachPort,CGBase,CGDirectDisplay,CGEventTypes,CGGeometry,CGErrors,CGRemoteOperation,CGEventSource;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

{ Return the type identifier for the opaque type `CGEventRef'. }

function CGEventGetTypeID: CFTypeID; external name '_CGEventGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new event using the event source `source'. If `source' is NULL,
   the default source is used. }

function CGEventCreate( source: CGEventSourceRef ): CGEventRef; external name '_CGEventCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a "flattened" data representation of an event. }

function CGEventCreateData( allocator: CFAllocatorRef; event: CGEventRef ): CFDataRef; external name '_CGEventCreateData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return an event created from a "flattened" data representation of the
   event. }

function CGEventCreateFromData( allocator: CFAllocatorRef; data: CFDataRef ): CGEventRef; external name '_CGEventCreateFromData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new mouse event.
<<<<<<< HEAD

   The event source may be taken from another event, or may be NULL.
   `mouseType' should be one of the mouse event types. `mouseCursorPosition'
   should be the position of the mouse cursor in global coordinates.
   `mouseButton' should be the button that's changing state; `mouseButton'
   is ignored unless `mouseType' is one of `kCGEventOtherMouseDown',
   `kCGEventOtherMouseDragged', or `kCGEventOtherMouseUp'.

   The current implemementation of the event system supports a maximum of
   thirty-two buttons. Mouse button 0 is the primary button on the mouse.
   Mouse button 1 is the secondary mouse button (right). Mouse button 2 is
   the center button, and the remaining buttons are in USB device order. }

function CGEventCreateMouseEvent( source: CGEventSourceRef; mouseType: CGEventType; mouseCursorPosition: CGPoint; mouseButton: CGMouseButton ): CGEventRef; external name '_CGEventCreateMouseEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new keyboard event.

   The event source may be taken from another event, or may be NULL. Based
   on the virtual key code values entered, the appropriate key down, key up,
   or flags changed events are generated.

   All keystrokes needed to generate a character must be entered, including
   SHIFT, CONTROL, OPTION, and COMMAND keys. For example, to produce a 'Z',
   the SHIFT key must be down, the 'z' key must go down, and then the SHIFT
   and 'z' key must be released:

     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, true);  // shift down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, true);  // 'z' down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, false); // 'z' up
     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, false); // 'shift up }

function CGEventCreateKeyboardEvent( source: CGEventSourceRef; virtualKey: CGKeyCode; keyDown: CBool ): CGEventRef; external name '_CGEventCreateKeyboardEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
                                                 
{ Return a new scrollwheel event.

   The event source may be taken from another event, or may be NULL. The
   scrolling units may be specified in lines using `kCGScrollEventUnitLine'
   or in pixels using `kCGScrollEventUnitPixel'. `kCGScrollEventUnitPixel'
   will produce an event that most applications interpret as a smooth
   scrolling event.
<<<<<<< HEAD

   One or more wheels must be specified. The current implementation supports
   up to three wheels.

   Every scrollwheel event can be interpreted to be scrolling by pixel or by
   line. The scale between the two is about 10 pixels per line by default.
   The scale can be altered by setting a custom value for the event source,
   using `CGEventSourceSetPixelsPerLine'. }
 
function CGEventCreateScrollWheelEvent( source: CGEventSourceRef; units: CGScrollEventUnit; wheelCount: UInt32; wheel1: SInt32; ... ): CGEventRef; external name '_CGEventCreateScrollWheelEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return a copy of `event'. }

=======

   One or more wheels must be specified. The current implementation supports
   up to three wheels.

   Every scrollwheel event can be interpreted to be scrolling by pixel or by
   line. The scale between the two is about 10 pixels per line by default.
   The scale can be altered by setting a custom value for the event source,
   using `CGEventSourceSetPixelsPerLine'. }
 
function CGEventCreateScrollWheelEvent( source: CGEventSourceRef; units: CGScrollEventUnit; wheelCount: UInt32; wheel1: SInt32; ... ): CGEventRef; external name '_CGEventCreateScrollWheelEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return a copy of `event'. }

>>>>>>> graemeg/cpstrnew
=======

   The event source may be taken from another event, or may be NULL.
   `mouseType' should be one of the mouse event types. `mouseCursorPosition'
   should be the position of the mouse cursor in global coordinates.
   `mouseButton' should be the button that's changing state; `mouseButton'
   is ignored unless `mouseType' is one of `kCGEventOtherMouseDown',
   `kCGEventOtherMouseDragged', or `kCGEventOtherMouseUp'.

   The current implemementation of the event system supports a maximum of
   thirty-two buttons. Mouse button 0 is the primary button on the mouse.
   Mouse button 1 is the secondary mouse button (right). Mouse button 2 is
   the center button, and the remaining buttons are in USB device order. }

function CGEventCreateMouseEvent( source: CGEventSourceRef; mouseType: CGEventType; mouseCursorPosition: CGPoint; mouseButton: CGMouseButton ): CGEventRef; external name '_CGEventCreateMouseEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new keyboard event.

   The event source may be taken from another event, or may be NULL. Based
   on the virtual key code values entered, the appropriate key down, key up,
   or flags changed events are generated.

   All keystrokes needed to generate a character must be entered, including
   SHIFT, CONTROL, OPTION, and COMMAND keys. For example, to produce a 'Z',
   the SHIFT key must be down, the 'z' key must go down, and then the SHIFT
   and 'z' key must be released:

     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, true);  // shift down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, true);  // 'z' down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, false); // 'z' up
     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, false); // 'shift up }

function CGEventCreateKeyboardEvent( source: CGEventSourceRef; virtualKey: CGKeyCode; keyDown: CBool ): CGEventRef; external name '_CGEventCreateKeyboardEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
                                                 
{ Return a new scrollwheel event.

   The event source may be taken from another event, or may be NULL. The
   scrolling units may be specified in lines using `kCGScrollEventUnitLine'
   or in pixels using `kCGScrollEventUnitPixel'. `kCGScrollEventUnitPixel'
   will produce an event that most applications interpret as a smooth
   scrolling event.

   One or more wheels must be specified. The current implementation supports
   up to three wheels.

   Every scrollwheel event can be interpreted to be scrolling by pixel or by
   line. The scale between the two is about 10 pixels per line by default.
   The scale can be altered by setting a custom value for the event source,
   using `CGEventSourceSetPixelsPerLine'. }
 
function CGEventCreateScrollWheelEvent( source: CGEventSourceRef; units: CGScrollEventUnit; wheelCount: UInt32; wheel1: SInt32; ... ): CGEventRef; external name '_CGEventCreateScrollWheelEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return a copy of `event'. }

>>>>>>> origin/cpstrnew
function CGEventCreateCopy( event: CGEventRef ): CGEventRef; external name '_CGEventCreateCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
{ 
 * CFRetain() and CFRelease() may be used to retain and release CGEventRefs.
 }

function CGEventGetSource( event: CGEventRef ): CGEventSourceRef; external name '_CGEventGetSource'; (* DEPRECATED_IN_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Return an event source created from an existing event.

   Event filters may use the event source to generate events that are
   compatible with an event being filtered.

   Note that `CGEventCreateSourceFromEvent' may return NULL if the event
   was generated with a private CGEventSourceStateID owned by another
   process.  Such events should be filtered based on the public state. }

function CGEventCreateSourceFromEvent( event: CGEventRef ): CGEventSourceRef; external name '_CGEventCreateSourceFromEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event source of an event. }

procedure CGEventSetSource( event: CGEventRef; source: CGEventSourceRef ); external name '_CGEventSetSource';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the event type of an event (left mouse down, for example). }

function CGEventGetType( event: CGEventRef ): CGEventType; external name '_CGEventGetType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event type of an event. }

procedure CGEventSetType( event: CGEventRef; typ: CGEventType ); external name '_CGEventSetType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the timestamp of an event. }

function CGEventGetTimestamp( event: CGEventRef ): CGEventTimestamp; external name '_CGEventGetTimestamp';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the timestamp of an event. }

procedure CGEventSetTimestamp( event: CGEventRef; timestamp: CGEventTimestamp ); external name '_CGEventSetTimestamp';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the location of an event in global display coordinates. }

function CGEventGetLocation( event: CGEventRef ): CGPoint; external name '_CGEventGetLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the location of an event relative to the lower-left corner of the
   main display. }

function CGEventGetUnflippedLocation( event: CGEventRef ): CGPoint; external name '_CGEventGetUnflippedLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
<<<<<<< HEAD

{ Set the location of an event in global display coordinates. }

procedure CGEventSetLocation( event: CGEventRef; location: CGPoint ); external name '_CGEventSetLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the event flags of an event. }

function CGEventGetFlags( event: CGEventRef ): CGEventFlags; external name '_CGEventGetFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event flags of an event. }

procedure CGEventSetFlags( event: CGEventRef; flags: CGEventFlags ); external name '_CGEventSetFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the Unicode string associated with a keyboard event.

   When you call this function with a NULL string or a maximum string length
   of 0, the function still returns the actual count of Unicode characters
   in the event. }

procedure CGEventKeyboardGetUnicodeString( event: CGEventRef; maxStringLength: UniCharCount; var actualStringLength: UniCharCount; unicodeString: {variable-size-array} UniCharPtr ); external name '_CGEventKeyboardGetUnicodeString';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the Unicode string associated with a keyboard event.

   By default, the system translates the virtual key code in a keyboard
   event into a Unicode string based on the keyboard ID in the event source.
   This function allows you to manually override this string. Note that
   application frameworks may ignore the Unicode string in a keyboard event
   and do their own translation based on the virtual keycode and perceived
   event state. }

procedure CGEventKeyboardSetUnicodeString( event: CGEventRef; stringLength: UniCharCount; {const} unicodeString: {variable-size-array} UniCharPtr ); external name '_CGEventKeyboardSetUnicodeString';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

=======

{ Return the type identifier for the opaque type `CGEventRef'. }

function CGEventGetTypeID: CFTypeID; external name '_CGEventGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new event using the event source `source'. If `source' is NULL,
   the default source is used. }

=======

{ Return the type identifier for the opaque type `CGEventRef'. }

function CGEventGetTypeID: CFTypeID; external name '_CGEventGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new event using the event source `source'. If `source' is NULL,
   the default source is used. }

>>>>>>> graemeg/cpstrnew
function CGEventCreate( source: CGEventSourceRef ): CGEventRef; external name '_CGEventCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a "flattened" data representation of an event. }

function CGEventCreateData( allocator: CFAllocatorRef; event: CGEventRef ): CFDataRef; external name '_CGEventCreateData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return an event created from a "flattened" data representation of the
   event. }

function CGEventCreateFromData( allocator: CFAllocatorRef; data: CFDataRef ): CGEventRef; external name '_CGEventCreateFromData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new mouse event.

   The event source may be taken from another event, or may be NULL.
   `mouseType' should be one of the mouse event types. `mouseCursorPosition'
   should be the position of the mouse cursor in global coordinates.
   `mouseButton' should be the button that's changing state; `mouseButton'
   is ignored unless `mouseType' is one of `kCGEventOtherMouseDown',
   `kCGEventOtherMouseDragged', or `kCGEventOtherMouseUp'.

   The current implemementation of the event system supports a maximum of
   thirty-two buttons. Mouse button 0 is the primary button on the mouse.
   Mouse button 1 is the secondary mouse button (right). Mouse button 2 is
   the center button, and the remaining buttons are in USB device order. }

function CGEventCreateMouseEvent( source: CGEventSourceRef; mouseType: CGEventType; mouseCursorPosition: CGPoint; mouseButton: CGMouseButton ): CGEventRef; external name '_CGEventCreateMouseEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new keyboard event.

   The event source may be taken from another event, or may be NULL. Based
   on the virtual key code values entered, the appropriate key down, key up,
   or flags changed events are generated.
<<<<<<< HEAD

   All keystrokes needed to generate a character must be entered, including
   SHIFT, CONTROL, OPTION, and COMMAND keys. For example, to produce a 'Z',
   the SHIFT key must be down, the 'z' key must go down, and then the SHIFT
   and 'z' key must be released:

     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, true);  // shift down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, true);  // 'z' down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, false); // 'z' up
     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, false); // 'shift up }

function CGEventCreateKeyboardEvent( source: CGEventSourceRef; virtualKey: CGKeyCode; keyDown: CBool ): CGEventRef; external name '_CGEventCreateKeyboardEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
                                                 
{ Return a new scrollwheel event.

   The event source may be taken from another event, or may be NULL. The
   scrolling units may be specified in lines using `kCGScrollEventUnitLine'
   or in pixels using `kCGScrollEventUnitPixel'. `kCGScrollEventUnitPixel'
   will produce an event that most applications interpret as a smooth
   scrolling event.

   One or more wheels must be specified. The current implementation supports
   up to three wheels.

   Every scrollwheel event can be interpreted to be scrolling by pixel or by
   line. The scale between the two is about 10 pixels per line by default.
   The scale can be altered by setting a custom value for the event source,
   using `CGEventSourceSetPixelsPerLine'. }
 
function CGEventCreateScrollWheelEvent( source: CGEventSourceRef; units: CGScrollEventUnit; wheelCount: UInt32; wheel1: SInt32; ... ): CGEventRef; external name '_CGEventCreateScrollWheelEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return a copy of `event'. }

=======

   All keystrokes needed to generate a character must be entered, including
   SHIFT, CONTROL, OPTION, and COMMAND keys. For example, to produce a 'Z',
   the SHIFT key must be down, the 'z' key must go down, and then the SHIFT
   and 'z' key must be released:

     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, true);  // shift down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, true);  // 'z' down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, false); // 'z' up
     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, false); // 'shift up }

function CGEventCreateKeyboardEvent( source: CGEventSourceRef; virtualKey: CGKeyCode; keyDown: CBool ): CGEventRef; external name '_CGEventCreateKeyboardEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
                                                 
{ Return a new scrollwheel event.

   The event source may be taken from another event, or may be NULL. The
   scrolling units may be specified in lines using `kCGScrollEventUnitLine'
   or in pixels using `kCGScrollEventUnitPixel'. `kCGScrollEventUnitPixel'
   will produce an event that most applications interpret as a smooth
   scrolling event.

   One or more wheels must be specified. The current implementation supports
   up to three wheels.

   Every scrollwheel event can be interpreted to be scrolling by pixel or by
   line. The scale between the two is about 10 pixels per line by default.
   The scale can be altered by setting a custom value for the event source,
   using `CGEventSourceSetPixelsPerLine'. }
 
function CGEventCreateScrollWheelEvent( source: CGEventSourceRef; units: CGScrollEventUnit; wheelCount: UInt32; wheel1: SInt32; ... ): CGEventRef; external name '_CGEventCreateScrollWheelEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return a copy of `event'. }

>>>>>>> graemeg/cpstrnew
function CGEventCreateCopy( event: CGEventRef ): CGEventRef; external name '_CGEventCreateCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

{$ifc TARGET_OS_MAC}

{ Return the type identifier for the opaque type `CGEventRef'. }

function CGEventGetTypeID: CFTypeID; external name '_CGEventGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new event using the event source `source'. If `source' is NULL,
   the default source is used. }

function CGEventCreate( source: CGEventSourceRef ): CGEventRef; external name '_CGEventCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a "flattened" data representation of an event. }

function CGEventCreateData( allocator: CFAllocatorRef; event: CGEventRef ): CFDataRef; external name '_CGEventCreateData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return an event created from a "flattened" data representation of the
   event. }

function CGEventCreateFromData( allocator: CFAllocatorRef; data: CFDataRef ): CGEventRef; external name '_CGEventCreateFromData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new mouse event.

   The event source may be taken from another event, or may be NULL.
   `mouseType' should be one of the mouse event types. `mouseCursorPosition'
   should be the position of the mouse cursor in global coordinates.
   `mouseButton' should be the button that's changing state; `mouseButton'
   is ignored unless `mouseType' is one of `kCGEventOtherMouseDown',
   `kCGEventOtherMouseDragged', or `kCGEventOtherMouseUp'.

   The current implemementation of the event system supports a maximum of
   thirty-two buttons. Mouse button 0 is the primary button on the mouse.
   Mouse button 1 is the secondary mouse button (right). Mouse button 2 is
   the center button, and the remaining buttons are in USB device order. }

function CGEventCreateMouseEvent( source: CGEventSourceRef; mouseType: CGEventType; mouseCursorPosition: CGPoint; mouseButton: CGMouseButton ): CGEventRef; external name '_CGEventCreateMouseEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return a new keyboard event.

   The event source may be taken from another event, or may be NULL. Based
   on the virtual key code values entered, the appropriate key down, key up,
   or flags changed events are generated.

   All keystrokes needed to generate a character must be entered, including
   SHIFT, CONTROL, OPTION, and COMMAND keys. For example, to produce a 'Z',
   the SHIFT key must be down, the 'z' key must go down, and then the SHIFT
   and 'z' key must be released:

     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, true);  // shift down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, true);  // 'z' down
     CGEventCreateKeyboardEvent(source, (CGKeyCode) 6, false); // 'z' up
     CGEventCreateKeyboardEvent(source, (CGKeyCode)56, false); // 'shift up }

function CGEventCreateKeyboardEvent( source: CGEventSourceRef; virtualKey: CGKeyCode; keyDown: CBool ): CGEventRef; external name '_CGEventCreateKeyboardEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
                                                 
{ Return a new scrollwheel event.

   The event source may be taken from another event, or may be NULL. The
   scrolling units may be specified in lines using `kCGScrollEventUnitLine'
   or in pixels using `kCGScrollEventUnitPixel'. `kCGScrollEventUnitPixel'
   will produce an event that most applications interpret as a smooth
   scrolling event.

   One or more wheels must be specified. The current implementation supports
   up to three wheels.

   Every scrollwheel event can be interpreted to be scrolling by pixel or by
   line. The scale between the two is about 10 pixels per line by default.
   The scale can be altered by setting a custom value for the event source,
   using `CGEventSourceSetPixelsPerLine'. }
 
function CGEventCreateScrollWheelEvent( source: CGEventSourceRef; units: CGScrollEventUnit; wheelCount: UInt32; wheel1: SInt32; ... ): CGEventRef; external name '_CGEventCreateScrollWheelEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return a copy of `event'. }

<<<<<<< HEAD
function CGEventCreateCopy( event: CGEventRef ): CGEventRef; external name '_CGEventCreateCopy'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGEventCreateCopy( event: CGEventRef ): CGEventRef; external name '_CGEventCreateCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4
{ 
 * CFRetain() and CFRelease() may be used to retain and release CGEventRefs.
 }

function CGEventGetSource( event: CGEventRef ): CGEventSourceRef; external name '_CGEventGetSource'; (* DEPRECATED_IN_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ Return an event source created from an existing event.

   Event filters may use the event source to generate events that are
   compatible with an event being filtered.

   Note that `CGEventCreateSourceFromEvent' may return NULL if the event
   was generated with a private CGEventSourceStateID owned by another
   process.  Such events should be filtered based on the public state. }

function CGEventCreateSourceFromEvent( event: CGEventRef ): CGEventSourceRef; external name '_CGEventCreateSourceFromEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event source of an event. }

procedure CGEventSetSource( event: CGEventRef; source: CGEventSourceRef ); external name '_CGEventSetSource';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the event type of an event (left mouse down, for example). }

function CGEventGetType( event: CGEventRef ): CGEventType; external name '_CGEventGetType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event type of an event. }

procedure CGEventSetType( event: CGEventRef; typ: CGEventType ); external name '_CGEventSetType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the timestamp of an event. }

function CGEventGetTimestamp( event: CGEventRef ): CGEventTimestamp; external name '_CGEventGetTimestamp';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the timestamp of an event. }

procedure CGEventSetTimestamp( event: CGEventRef; timestamp: CGEventTimestamp ); external name '_CGEventSetTimestamp';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the location of an event in global display coordinates. }

function CGEventGetLocation( event: CGEventRef ): CGPoint; external name '_CGEventGetLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the location of an event relative to the lower-left corner of the
   main display. }

function CGEventGetUnflippedLocation( event: CGEventRef ): CGPoint; external name '_CGEventGetUnflippedLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Set the location of an event in global display coordinates. }

procedure CGEventSetLocation( event: CGEventRef; location: CGPoint ); external name '_CGEventSetLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the event flags of an event. }

function CGEventGetFlags( event: CGEventRef ): CGEventFlags; external name '_CGEventGetFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event flags of an event. }

=======

{ Set the location of an event in global display coordinates. }

procedure CGEventSetLocation( event: CGEventRef; location: CGPoint ); external name '_CGEventSetLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the event flags of an event. }

function CGEventGetFlags( event: CGEventRef ): CGEventFlags; external name '_CGEventGetFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event flags of an event. }

>>>>>>> origin/cpstrnew
procedure CGEventSetFlags( event: CGEventRef; flags: CGEventFlags ); external name '_CGEventSetFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the Unicode string associated with a keyboard event.

   When you call this function with a NULL string or a maximum string length
   of 0, the function still returns the actual count of Unicode characters
   in the event. }

procedure CGEventKeyboardGetUnicodeString( event: CGEventRef; maxStringLength: UniCharCount; var actualStringLength: UniCharCount; unicodeString: {variable-size-array} UniCharPtr ); external name '_CGEventKeyboardGetUnicodeString';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the Unicode string associated with a keyboard event.

   By default, the system translates the virtual key code in a keyboard
   event into a Unicode string based on the keyboard ID in the event source.
   This function allows you to manually override this string. Note that
   application frameworks may ignore the Unicode string in a keyboard event
   and do their own translation based on the virtual keycode and perceived
   event state. }

procedure CGEventKeyboardSetUnicodeString( event: CGEventRef; stringLength: UniCharCount; {const} unicodeString: {variable-size-array} UniCharPtr ); external name '_CGEventKeyboardSetUnicodeString';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Return the integer value of a field in an event. }

function CGEventGetIntegerValueField( event: CGEventRef; field: CGEventField ): SInt64; external name '_CGEventGetIntegerValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

{ Set the integer value of a field in an event.

   Before calling this function, the event type must be set using a typed
   event creation function such as `CGEventCreateMouseEvent', or by calling
   `CGEventSetType'.

   If you are creating a mouse event generated by a tablet, call this
   function and specify the field `kCGMouseEventSubtype' with a value of
   `kCGEventMouseSubtypeTabletPoint' or `kCGEventMouseSubtypeTabletProximity'
   before setting other parameters. }

procedure CGEventSetIntegerValueField( event: CGEventRef; field: CGEventField; value: SInt64 ); external name '_CGEventSetIntegerValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the floating-point value of a field in an event.
<<<<<<< HEAD

   In cases where the field value is represented within the event by a fixed
   point number or an integer, the result is scaled to the appropriate range
   as part of creating the floating-point representation. }

function CGEventGetDoubleValueField( event: CGEventRef; field: CGEventField ): Float64; external name '_CGEventGetDoubleValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the floating-point value of a field in an event.

   Before calling this function, the event type must be set using a typed
   event creation function such as `CGEventCreateMouseEvent', or by calling
   `CGEventSetType'.

<<<<<<< HEAD
<<<<<<< HEAD
   In cases where the fieldís value is represented within the event by a
=======
   In cases where the field’s value is represented within the event by a
>>>>>>> graemeg/cpstrnew
=======
   In cases where the field’s value is represented within the event by a
>>>>>>> graemeg/cpstrnew
   fixed point number or integer, the value parameter is scaled as needed
   and converted to the appropriate type. }

procedure CGEventSetDoubleValueField( event: CGEventRef; field: CGEventField; value: Float64 ); external name '_CGEventSetDoubleValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
<<<<<<< HEAD

{ Event taps.

   A function registers an event tap, taking a pointer to the program's tap
   function and an arbitrary reference to be passed to the tap function, and
   returning a `CFMachPortRef' the program can add to the appropriate run
   loop by creating a surce and using `CFRunLoopAddSource'.

   Taps may be placed at the point where HIDSystem events enter the server,
   at the point where HIDSystem and remote control events enter a session,
   at the point where events have been annotated to flow to a specific
   application, or at the point where events are delivered to the
   application. Taps may be inserted at a specified point at the head of
   pre-existing filters, or appended after any pre-existing filters.

=======

{ Event taps.

   A function registers an event tap, taking a pointer to the program's tap
   function and an arbitrary reference to be passed to the tap function, and
   returning a `CFMachPortRef' the program can add to the appropriate run
   loop by creating a surce and using `CFRunLoopAddSource'.

   Taps may be placed at the point where HIDSystem events enter the server,
   at the point where HIDSystem and remote control events enter a session,
   at the point where events have been annotated to flow to a specific
   application, or at the point where events are delivered to the
   application. Taps may be inserted at a specified point at the head of
   pre-existing filters, or appended after any pre-existing filters.

>>>>>>> graemeg/cpstrnew
=======

{ Set the integer value of a field in an event.

   Before calling this function, the event type must be set using a typed
   event creation function such as `CGEventCreateMouseEvent', or by calling
   `CGEventSetType'.

   If you are creating a mouse event generated by a tablet, call this
   function and specify the field `kCGMouseEventSubtype' with a value of
   `kCGEventMouseSubtypeTabletPoint' or `kCGEventMouseSubtypeTabletProximity'
   before setting other parameters. }

procedure CGEventSetIntegerValueField( event: CGEventRef; field: CGEventField; value: SInt64 ); external name '_CGEventSetIntegerValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the floating-point value of a field in an event.

   In cases where the field value is represented within the event by a fixed
   point number or an integer, the result is scaled to the appropriate range
   as part of creating the floating-point representation. }

function CGEventGetDoubleValueField( event: CGEventRef; field: CGEventField ): Float64; external name '_CGEventGetDoubleValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the floating-point value of a field in an event.

   Before calling this function, the event type must be set using a typed
   event creation function such as `CGEventCreateMouseEvent', or by calling
   `CGEventSetType'.

   In cases where the field’s value is represented within the event by a
   fixed point number or integer, the value parameter is scaled as needed
   and converted to the appropriate type. }

procedure CGEventSetDoubleValueField( event: CGEventRef; field: CGEventField; value: Float64 ); external name '_CGEventSetDoubleValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Event taps.

   A function registers an event tap, taking a pointer to the program's tap
   function and an arbitrary reference to be passed to the tap function, and
   returning a `CFMachPortRef' the program can add to the appropriate run
   loop by creating a surce and using `CFRunLoopAddSource'.

   Taps may be placed at the point where HIDSystem events enter the server,
   at the point where HIDSystem and remote control events enter a session,
   at the point where events have been annotated to flow to a specific
   application, or at the point where events are delivered to the
   application. Taps may be inserted at a specified point at the head of
   pre-existing filters, or appended after any pre-existing filters.

>>>>>>> graemeg/cpstrnew
=======

   In cases where the field value is represented within the event by a fixed
   point number or an integer, the result is scaled to the appropriate range
   as part of creating the floating-point representation. }

function CGEventGetDoubleValueField( event: CGEventRef; field: CGEventField ): Float64; external name '_CGEventGetDoubleValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the floating-point value of a field in an event.

   Before calling this function, the event type must be set using a typed
   event creation function such as `CGEventCreateMouseEvent', or by calling
   `CGEventSetType'.

   In cases where the field’s value is represented within the event by a
   fixed point number or integer, the value parameter is scaled as needed
   and converted to the appropriate type. }

procedure CGEventSetDoubleValueField( event: CGEventRef; field: CGEventField; value: Float64 ); external name '_CGEventSetDoubleValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Event taps.

   A function registers an event tap, taking a pointer to the program's tap
   function and an arbitrary reference to be passed to the tap function, and
   returning a `CFMachPortRef' the program can add to the appropriate run
   loop by creating a surce and using `CFRunLoopAddSource'.

   Taps may be placed at the point where HIDSystem events enter the server,
   at the point where HIDSystem and remote control events enter a session,
   at the point where events have been annotated to flow to a specific
   application, or at the point where events are delivered to the
   application. Taps may be inserted at a specified point at the head of
   pre-existing filters, or appended after any pre-existing filters.

>>>>>>> origin/cpstrnew
   Taps may be passive event listeners, or active filters. An active filter
   may pass an event through unmodified, modify an event, or discard an
   event. When a tap is registered, it identifies the set of events to be
   observed with a mask, and indicates if it is a passive or active event
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   filter. Multiple event type bitmasks may be ORed together.
=======
   filter. Multiple event type bitmasks may be "OR"ed together.
>>>>>>> graemeg/cpstrnew
=======
   filter. Multiple event type bitmasks may be "OR"ed together.
>>>>>>> graemeg/cpstrnew
=======
   filter. Multiple event type bitmasks may be "OR"ed together.
>>>>>>> graemeg/cpstrnew
=======
   filter. Multiple event type bitmasks may be "OR"ed together.
>>>>>>> origin/cpstrnew

   Taps may only be placed at `kCGHIDEventTap' by a process running as the
   root user. NULL is returned for other users.

   Taps placed at `kCGHIDEventTap', `kCGSessionEventTap',
   `kCGAnnotatedSessionEventTap', or on a specific process may only receive
   key up and down events if access for assistive devices is enabled
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   (Preferences Accessibility panel, Keyboard view) or the caller is enabled
   for assistive device access, as by `AXMakeProcessTrusted'. If the tap is
   not permitted to monitor these events when the tap is created, then the
   appropriate bits in the mask are cleared. If that results in an empty
   mask, then NULL is returned.

   An event tap is represented as a `CFMachPortRef'. The event tap may be
   added to an appropriate run loop by calling `CFRunLoopAddSource'.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   (Preferences Universal Access panel, Keyboard view) or the caller is
   enabled for assistive device access, as by `AXMakeProcessTrusted'. If the
   tap is not permitted to monitor these events when the tap is created,
   then the appropriate bits in the mask are cleared. If that results in an
   empty mask, then NULL is returned.
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

   Releasing the CFMachPortRef will release the tap.

   A `CGEventTapProxy' is an opaque reference to state within the client
   application associated with the tap. The tap function may pass this
   reference to other functions, such as the event-posting routines.

   The event tap callback runs from the CFRunLoop to which the tap
   CFMachPort is added as a source. Thread safety is defined by the
   CFRunLoop and its environment. }

{ Create an event tap. }

function CGEventTapCreate( tap: CGEventTapLocation; place: CGEventTapPlacement; options: CGEventTapOptions; eventsOfInterest: CGEventMask; callback: CGEventTapCallBack; userInfo: UnivPtr ): CFMachPortRef; external name '_CGEventTapCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Create an event tap for a specified process.

   Events routed to individual applications may be tapped using another
   function. `CGEventTapCreateForPSN' will report all events routed to the
   specified application. }

function CGEventTapCreateForPSN( processSerialNumber: UnivPtr; place: CGEventTapPlacement; options: CGEventTapOptions; eventsOfInterest: CGEventMask; callback: CGEventTapCallBack; userInfo: UnivPtr ): CFMachPortRef; external name '_CGEventTapCreateForPSN';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Enable or disable an event tap.

   Taps are normally enabled when created. If a tap becomes unresponsive or
   a user requests taps be disabled, an appropriate `kCGEventTapDisabled...'
   event is passed to the registered CGEventTapCallBack function. An event
   tap may be re-enabled by calling this function. }

procedure CGEventTapEnable( tap: CFMachPortRef; enable: CBool ); external name '_CGEventTapEnable';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return true if `tap' is enabled; false otherwise. }

function CGEventTapIsEnabled( tap: CFMachPortRef ): CBool; external name '_CGEventTapIsEnabled';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Post an event from an event tap into the event stream.

   You can use this function to post a new event at the same point to which
   an event returned from an event tap callback function would be posted.
   The new event enters the system before the event returned by the callback
   enters the system. Events posted into the system will be seen by all taps
   placed after the tap posting the event. }

procedure CGEventTapPostEvent( proxy: CGEventTapProxy; event: CGEventRef ); external name '_CGEventTapPostEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Post an event into the event stream at a specified location.

   This function posts the specified event immediately before any event taps
   instantiated for that location, and the event passes through any such
   taps. }

procedure CGEventPost( tap: CGEventTapLocation; event: CGEventRef ); external name '_CGEventPost';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Post an event into the event stream for a specific application.

   This function makes it possible for an application to establish
   an event routing policy, for example, by tapping events at the
   `kCGAnnotatedSessionEventTap' location and then posting the events
   to another desired process.

   This function posts the specified event immediately before any event taps
   instantiated for the specified process, and the event passes through any
   such taps. }

procedure CGEventPostToPSN( processSerialNumber: UnivPtr; event: CGEventRef ); external name '_CGEventPostToPSN';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Gets a list of currently installed event taps.

   `tapList' is an array of event tap information structures of length
   `maxNumberOfTaps'. You are responsible for allocating storage for this
   array. On return, your array contains a list of currently installed event
   taps. On return, the number of event taps that are currently installed is
   stored in `eventTapCount'. If you pass NULL in this parameter, the
   `maxNumberOfTaps' parameter is ignored, and the number of event taps that
   are currently installed is stored in `eventTapCount'.

   Each call to this function has the side effect of resetting the minimum
   and maximum latencies in the `tapList' parameter to the corresponding
   average values. Values reported in these fields reflect the minimum and
   maximum values seen since the preceding call, or the instantiation of the
   tap. This allows a monitoring tool to evaluate the best and worst case
   latency over time and under various operating conditions. }

function CGGetEventTapList( maxNumberOfTaps: UInt32; tapList: {variable-size-array} CGEventTapInformationPtr; var eventTapCount: UInt32 ): CGError; external name '_CGGetEventTapList';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{$endc}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{
 * These functions provide access to the event source for an event.
 * Event filters may use these to generate events that are compatible
 * with an event being filtered.
 *
 * Note that CGEventCreateSourceFromEvent may return NULL if the event
 * was generated with a private CGEventSourceStateID owned by another
 * process.  Such events should be filtered based on the public state.
 }
function CGEventCreateSourceFromEvent( event: CGEventRef ): CGEventSourceRef; external name '_CGEventCreateSourceFromEvent'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
procedure CGEventSetSource( event: CGEventRef; source: CGEventSourceRef ); external name '_CGEventSetSource'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
=======
>>>>>>> origin/fixes_2.4

{ Return an event source created from an existing event.

   Event filters may use the event source to generate events that are
   compatible with an event being filtered.

   Note that `CGEventCreateSourceFromEvent' may return NULL if the event
   was generated with a private CGEventSourceStateID owned by another
   process.  Such events should be filtered based on the public state. }

function CGEventCreateSourceFromEvent( event: CGEventRef ): CGEventSourceRef; external name '_CGEventCreateSourceFromEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event source of an event. }

procedure CGEventSetSource( event: CGEventRef; source: CGEventSourceRef ); external name '_CGEventSetSource';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the event type of an event (left mouse down, for example). }

function CGEventGetType( event: CGEventRef ): CGEventType; external name '_CGEventGetType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event type of an event. }

procedure CGEventSetType( event: CGEventRef; typ: CGEventType ); external name '_CGEventSetType';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the timestamp of an event. }

function CGEventGetTimestamp( event: CGEventRef ): CGEventTimestamp; external name '_CGEventGetTimestamp';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the timestamp of an event. }

procedure CGEventSetTimestamp( event: CGEventRef; timestamp: CGEventTimestamp ); external name '_CGEventSetTimestamp';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the location of an event in global display coordinates. }

function CGEventGetLocation( event: CGEventRef ): CGPoint; external name '_CGEventGetLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the location of an event relative to the lower-left corner of the
   main display. }

function CGEventGetUnflippedLocation( event: CGEventRef ): CGPoint; external name '_CGEventGetUnflippedLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Set the location of an event in global display coordinates. }

procedure CGEventSetLocation( event: CGEventRef; location: CGPoint ); external name '_CGEventSetLocation';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the event flags of an event. }

function CGEventGetFlags( event: CGEventRef ): CGEventFlags; external name '_CGEventGetFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the event flags of an event. }

procedure CGEventSetFlags( event: CGEventRef; flags: CGEventFlags ); external name '_CGEventSetFlags';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the Unicode string associated with a keyboard event.

   When you call this function with a NULL string or a maximum string length
   of 0, the function still returns the actual count of Unicode characters
   in the event. }

procedure CGEventKeyboardGetUnicodeString( event: CGEventRef; maxStringLength: UniCharCount; var actualStringLength: UniCharCount; unicodeString: {variable-size-array} UniCharPtr ); external name '_CGEventKeyboardGetUnicodeString';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the Unicode string associated with a keyboard event.

   By default, the system translates the virtual key code in a keyboard
   event into a Unicode string based on the keyboard ID in the event source.
   This function allows you to manually override this string. Note that
   application frameworks may ignore the Unicode string in a keyboard event
   and do their own translation based on the virtual keycode and perceived
   event state. }

procedure CGEventKeyboardSetUnicodeString( event: CGEventRef; stringLength: UniCharCount; {const} unicodeString: {variable-size-array} UniCharPtr ); external name '_CGEventKeyboardSetUnicodeString';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the integer value of a field in an event. }

function CGEventGetIntegerValueField( event: CGEventRef; field: CGEventField ): SInt64; external name '_CGEventGetIntegerValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the integer value of a field in an event.

   Before calling this function, the event type must be set using a typed
   event creation function such as `CGEventCreateMouseEvent', or by calling
   `CGEventSetType'.

   If you are creating a mouse event generated by a tablet, call this
   function and specify the field `kCGMouseEventSubtype' with a value of
   `kCGEventMouseSubtypeTabletPoint' or `kCGEventMouseSubtypeTabletProximity'
   before setting other parameters. }

procedure CGEventSetIntegerValueField( event: CGEventRef; field: CGEventField; value: SInt64 ); external name '_CGEventSetIntegerValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return the floating-point value of a field in an event.

   In cases where the field value is represented within the event by a fixed
   point number or an integer, the result is scaled to the appropriate range
   as part of creating the floating-point representation. }

function CGEventGetDoubleValueField( event: CGEventRef; field: CGEventField ): Float64; external name '_CGEventGetDoubleValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Set the floating-point value of a field in an event.

   Before calling this function, the event type must be set using a typed
   event creation function such as `CGEventCreateMouseEvent', or by calling
   `CGEventSetType'.

   In cases where the field’s value is represented within the event by a
   fixed point number or integer, the value parameter is scaled as needed
   and converted to the appropriate type. }

procedure CGEventSetDoubleValueField( event: CGEventRef; field: CGEventField; value: Float64 ); external name '_CGEventSetDoubleValueField';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Event taps.

   A function registers an event tap, taking a pointer to the program's tap
   function and an arbitrary reference to be passed to the tap function, and
   returning a `CFMachPortRef' the program can add to the appropriate run
   loop by creating a surce and using `CFRunLoopAddSource'.

   Taps may be placed at the point where HIDSystem events enter the server,
   at the point where HIDSystem and remote control events enter a session,
   at the point where events have been annotated to flow to a specific
   application, or at the point where events are delivered to the
   application. Taps may be inserted at a specified point at the head of
   pre-existing filters, or appended after any pre-existing filters.

   Taps may be passive event listeners, or active filters. An active filter
   may pass an event through unmodified, modify an event, or discard an
   event. When a tap is registered, it identifies the set of events to be
   observed with a mask, and indicates if it is a passive or active event
   filter. Multiple event type bitmasks may be "OR"ed together.

   Taps may only be placed at `kCGHIDEventTap' by a process running as the
   root user. NULL is returned for other users.

   Taps placed at `kCGHIDEventTap', `kCGSessionEventTap',
   `kCGAnnotatedSessionEventTap', or on a specific process may only receive
   key up and down events if access for assistive devices is enabled
   (Preferences Universal Access panel, Keyboard view) or the caller is
   enabled for assistive device access, as by `AXMakeProcessTrusted'. If the
   tap is not permitted to monitor these events when the tap is created,
   then the appropriate bits in the mask are cleared. If that results in an
   empty mask, then NULL is returned.

   Releasing the CFMachPortRef will release the tap.

   A `CGEventTapProxy' is an opaque reference to state within the client
   application associated with the tap. The tap function may pass this
   reference to other functions, such as the event-posting routines.

   The event tap callback runs from the CFRunLoop to which the tap
   CFMachPort is added as a source. Thread safety is defined by the
   CFRunLoop and its environment. }

{ Create an event tap. }

function CGEventTapCreate( tap: CGEventTapLocation; place: CGEventTapPlacement; options: CGEventTapOptions; eventsOfInterest: CGEventMask; callback: CGEventTapCallBack; userInfo: UnivPtr ): CFMachPortRef; external name '_CGEventTapCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Create an event tap for a specified process.

   Events routed to individual applications may be tapped using another
   function. `CGEventTapCreateForPSN' will report all events routed to the
   specified application. }

function CGEventTapCreateForPSN( processSerialNumber: UnivPtr; place: CGEventTapPlacement; options: CGEventTapOptions; eventsOfInterest: CGEventMask; callback: CGEventTapCallBack; userInfo: UnivPtr ): CFMachPortRef; external name '_CGEventTapCreateForPSN';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Enable or disable an event tap.

   Taps are normally enabled when created. If a tap becomes unresponsive or
   a user requests taps be disabled, an appropriate `kCGEventTapDisabled...'
   event is passed to the registered CGEventTapCallBack function. An event
   tap may be re-enabled by calling this function. }

procedure CGEventTapEnable( tap: CFMachPortRef; enable: CBool ); external name '_CGEventTapEnable';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Return true if `tap' is enabled; false otherwise. }

function CGEventTapIsEnabled( tap: CFMachPortRef ): CBool; external name '_CGEventTapIsEnabled';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Post an event from an event tap into the event stream.

   You can use this function to post a new event at the same point to which
   an event returned from an event tap callback function would be posted.
   The new event enters the system before the event returned by the callback
   enters the system. Events posted into the system will be seen by all taps
   placed after the tap posting the event. }

procedure CGEventTapPostEvent( proxy: CGEventTapProxy; event: CGEventRef ); external name '_CGEventTapPostEvent';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Post an event into the event stream at a specified location.

   This function posts the specified event immediately before any event taps
   instantiated for that location, and the event passes through any such
   taps. }

procedure CGEventPost( tap: CGEventTapLocation; event: CGEventRef ); external name '_CGEventPost';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Post an event into the event stream for a specific application.

   This function makes it possible for an application to establish
   an event routing policy, for example, by tapping events at the
   `kCGAnnotatedSessionEventTap' location and then posting the events
   to another desired process.

   This function posts the specified event immediately before any event taps
   instantiated for the specified process, and the event passes through any
   such taps. }

procedure CGEventPostToPSN( processSerialNumber: UnivPtr; event: CGEventRef ); external name '_CGEventPostToPSN';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ Gets a list of currently installed event taps.

   `tapList' is an array of event tap information structures of length
   `maxNumberOfTaps'. You are responsible for allocating storage for this
   array. On return, your array contains a list of currently installed event
   taps. On return, the number of event taps that are currently installed is
   stored in `eventTapCount'. If you pass NULL in this parameter, the
   `maxNumberOfTaps' parameter is ignored, and the number of event taps that
   are currently installed is stored in `eventTapCount'.

   Each call to this function has the side effect of resetting the minimum
   and maximum latencies in the `tapList' parameter to the corresponding
   average values. Values reported in these fields reflect the minimum and
   maximum values seen since the preceding call, or the instantiation of the
   tap. This allows a monitoring tool to evaluate the best and worst case
   latency over time and under various operating conditions. }

function CGGetEventTapList( maxNumberOfTaps: UInt32; tapList: {variable-size-array} CGEventTapInformationPtr; var eventTapCount: UInt32 ): CGError; external name '_CGGetEventTapList';
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
