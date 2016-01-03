{
     File:       HIToolbox/Events.h
 
     Contains:   Event Manager Interfaces.
 
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
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1985-2005 by Apple Computer, Inc., all rights reserved
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1985-2005 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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

unit Events;
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
uses MacTypes,OSUtils,QuickdrawTypes,Endian;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======


{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}
>>>>>>> origin/cpstrnew

=======
uses MacTypes,OSUtils,Quickdraw,Endian;


>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,OSUtils,Quickdraw,Endian;
=======
uses MacTypes,OSUtils,QuickdrawTypes,Endian;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

>>>>>>> origin/fixes_2_2
{$ALIGN MAC68K}

type
	EventKind = UInt16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
type
>>>>>>> graemeg/fixes_2_2
=======
type
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	EventMask = UInt16;
const
	nullEvent = 0;
	mouseDown = 1;
	mouseUp = 2;
	keyDown = 3;
	keyUp = 4;
	autoKey = 5;
	updateEvt = 6;
	diskEvt = 7;    { Not sent in Carbon. See kEventClassVolume in CarbonEvents.h}
	activateEvt = 8;
	osEvt = 15;
	kHighLevelEvent = 23;

const
	mDownMask = 1 shl mouseDown; { mouse button pressed}
	mUpMask = 1 shl mouseUp; { mouse button released}
	keyDownMask = 1 shl keyDown; { key pressed}
	keyUpMask = 1 shl keyUp; { key released}
	autoKeyMask = 1 shl autoKey; { key repeatedly held down}
	updateMask = 1 shl updateEvt; { window needs updating}
	diskMask = 1 shl diskEvt; { disk inserted}
	activMask = 1 shl activateEvt; { activate/deactivate window}
	highLevelEventMask = $0400; { high-level events (includes AppleEvents)}
	osMask = 1 shl osEvt; { operating system events (suspend, resume)}
	everyEvent = $FFFF; { all of the above}

const
	charCodeMask = $000000FF;
	keyCodeMask = $0000FF00;
	adbAddrMask = $00FF0000;
	osEvtMessageMask = $FF000000;

const
{ OS event messages.  Event (sub)code is in the high byte of the message field.}
	mouseMovedMessage = $00FA;
	suspendResumeMessage = $0001;

const
	resumeFlag = 1;     { Bit 0 of message indicates resume vs suspend}


{
    CARBON ALERT! BATTLESTATIONS!
    
    The EventModifiers bits defined here are also used in the newer Carbon Event
    key modifiers parameters. There are two main differences:
    
    1)  The Carbon key modifiers parameter is a UInt32, not a UInt16. Never try to
        extract the key modifiers parameter from a Carbon Event into an EventModifiers
        type. You will probably get your stack trashed.
    2)  The Carbon key modifiers is just that: key modifiers. That parameter will
        never contain the button state bit.
}
type
	EventModifiers = UInt16;
const
{ modifiers }
	activeFlagBit = 0;    { activate? (activateEvt and mouseDown)}
	btnStateBit = 7;    { state of button?}
	cmdKeyBit = 8;    { command key down?}
	shiftKeyBit = 9;    { shift key down?}
	alphaLockBit = 10;   { alpha lock down?}
	optionKeyBit = 11;   { option key down?}
	controlKeyBit = 12;   { control key down?}
	rightShiftKeyBit = 13;   { right shift key down? Not supported on Mac OS X.}
	rightOptionKeyBit = 14;   { right Option key down? Not supported on Mac OS X.}
	rightControlKeyBit = 15;    { right Control key down? Not supported on Mac OS X.}

const
	activeFlag = 1 shl activeFlagBit;
	btnState = 1 shl btnStateBit;
	cmdKey = 1 shl cmdKeyBit;
	shiftKey = 1 shl shiftKeyBit;
	alphaLock = 1 shl alphaLockBit;
	optionKey = 1 shl optionKeyBit;
	controlKey = 1 shl controlKeyBit;
	rightShiftKey = 1 shl rightShiftKeyBit; { Not supported on Mac OS X.}
	rightOptionKey = 1 shl rightOptionKeyBit; { Not supported on Mac OS X.}
	rightControlKey = 1 shl rightControlKeyBit; { Not supported on Mac OS X.}

{ MacRoman character codes}
const
	kNullCharCode = 0;
	kHomeCharCode = 1;
	kEnterCharCode = 3;
	kEndCharCode = 4;
	kHelpCharCode = 5;
	kBellCharCode = 7;
	kBackspaceCharCode = 8;
	kTabCharCode = 9;
	kLineFeedCharCode = 10;
	kVerticalTabCharCode = 11;
	kPageUpCharCode = 11;
	kFormFeedCharCode = 12;
	kPageDownCharCode = 12;
	kReturnCharCode = 13;
	kFunctionKeyCharCode = 16;
	kCommandCharCode = 17;   { glyph available only in system fonts}
	kCheckCharCode = 18;   { glyph available only in system fonts}
	kDiamondCharCode = 19;   { glyph available only in system fonts}
	kAppleLogoCharCode = 20;   { glyph available only in system fonts}
	kEscapeCharCode = 27;
	kClearCharCode = 27;
	kLeftArrowCharCode = 28;
	kRightArrowCharCode = 29;
	kUpArrowCharCode = 30;
	kDownArrowCharCode = 31;
	kSpaceCharCode = 32;
	kDeleteCharCode = 127;
	kBulletCharCode = 165;
	kNonBreakingSpaceCharCode = 202;

{ useful Unicode code points}
const
	kShiftUnicode = $21E7; { Unicode UPWARDS WHITE ARROW}
	kControlUnicode = $2303; { Unicode UP ARROWHEAD}
	kOptionUnicode = $2325; { Unicode OPTION KEY}
	kCommandUnicode = $2318; { Unicode PLACE OF INTEREST SIGN}
	kPencilUnicode = $270E; { Unicode LOWER RIGHT PENCIL; actually pointed left until Mac OS X 10.3}
	kPencilLeftUnicode = $F802; { Unicode LOWER LEFT PENCIL; available in Mac OS X 10.3 and later}
	kCheckUnicode = $2713; { Unicode CHECK MARK}
	kDiamondUnicode = $25C6; { Unicode BLACK DIAMOND}
	kBulletUnicode = $2022; { Unicode BULLET}
	kAppleLogoUnicode = $F8FF; { Unicode APPLE LOGO}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

{
 *  Summary:
 *    Virtual keycodes
 *  
 *  Discussion:
 *    These constants are the virtual keycodes defined originally in
 *    Inside Mac Volume V, pg. V-191. They identify physical keys on a
 *    keyboard. Those constants with "ANSI" in the name are labeled
 *    according to the key position on an ANSI-standard US keyboard.
 *    For example, kVK_ANSI_A indicates the virtual keycode for the key
 *    with the letter 'A' in the US keyboard layout. Other keyboard
 *    layouts may have the 'A' key label on a different physical key;
 *    in this case, pressing 'A' will generate a different virtual
 *    keycode.
 }
const
	kVK_ANSI_A = $00;
	kVK_ANSI_S = $01;
	kVK_ANSI_D = $02;
	kVK_ANSI_F = $03;
	kVK_ANSI_H = $04;
	kVK_ANSI_G = $05;
	kVK_ANSI_Z = $06;
	kVK_ANSI_X = $07;
	kVK_ANSI_C = $08;
	kVK_ANSI_V = $09;
	kVK_ANSI_B = $0B;
	kVK_ANSI_Q = $0C;
	kVK_ANSI_W = $0D;
	kVK_ANSI_E = $0E;
	kVK_ANSI_R = $0F;
	kVK_ANSI_Y = $10;
	kVK_ANSI_T = $11;
	kVK_ANSI_1 = $12;
	kVK_ANSI_2 = $13;
	kVK_ANSI_3 = $14;
	kVK_ANSI_4 = $15;
	kVK_ANSI_6 = $16;
	kVK_ANSI_5 = $17;
	kVK_ANSI_Equal = $18;
	kVK_ANSI_9 = $19;
	kVK_ANSI_7 = $1A;
	kVK_ANSI_Minus = $1B;
	kVK_ANSI_8 = $1C;
	kVK_ANSI_0 = $1D;
	kVK_ANSI_RightBracket = $1E;
	kVK_ANSI_O = $1F;
	kVK_ANSI_U = $20;
	kVK_ANSI_LeftBracket = $21;
	kVK_ANSI_I = $22;
	kVK_ANSI_P = $23;
	kVK_ANSI_L = $25;
	kVK_ANSI_J = $26;
	kVK_ANSI_Quote = $27;
	kVK_ANSI_K = $28;
	kVK_ANSI_Semicolon = $29;
	kVK_ANSI_Backslash = $2A;
	kVK_ANSI_Comma = $2B;
	kVK_ANSI_Slash = $2C;
	kVK_ANSI_N = $2D;
	kVK_ANSI_M = $2E;
	kVK_ANSI_Period = $2F;
	kVK_ANSI_Grave = $32;
	kVK_ANSI_KeypadDecimal = $41;
	kVK_ANSI_KeypadMultiply = $43;
	kVK_ANSI_KeypadPlus = $45;
	kVK_ANSI_KeypadClear = $47;
	kVK_ANSI_KeypadDivide = $4B;
	kVK_ANSI_KeypadEnter = $4C;
	kVK_ANSI_KeypadMinus = $4E;
	kVK_ANSI_KeypadEquals = $51;
	kVK_ANSI_Keypad0 = $52;
	kVK_ANSI_Keypad1 = $53;
	kVK_ANSI_Keypad2 = $54;
	kVK_ANSI_Keypad3 = $55;
	kVK_ANSI_Keypad4 = $56;
	kVK_ANSI_Keypad5 = $57;
	kVK_ANSI_Keypad6 = $58;
	kVK_ANSI_Keypad7 = $59;
	kVK_ANSI_Keypad8 = $5B;
	kVK_ANSI_Keypad9 = $5C;

{ keycodes for keys that are independent of keyboard layout}
const
	kVK_Return = $24;
	kVK_Tab = $30;
	kVK_Space = $31;
	kVK_Delete = $33;
	kVK_Escape = $35;
	kVK_Command = $37;
	kVK_Shift = $38;
	kVK_CapsLock = $39;
	kVK_Option = $3A;
	kVK_Control = $3B;
	kVK_RightShift = $3C;
	kVK_RightOption = $3D;
	kVK_RightControl = $3E;
	kVK_Function = $3F;
	kVK_F17 = $40;
	kVK_VolumeUp = $48;
	kVK_VolumeDown = $49;
	kVK_Mute = $4A;
	kVK_F18 = $4F;
	kVK_F19 = $50;
	kVK_F20 = $5A;
	kVK_F5 = $60;
	kVK_F6 = $61;
	kVK_F7 = $62;
	kVK_F3 = $63;
	kVK_F8 = $64;
	kVK_F9 = $65;
	kVK_F11 = $67;
	kVK_F13 = $69;
	kVK_F16 = $6A;
	kVK_F14 = $6B;
	kVK_F10 = $6D;
	kVK_F12 = $6F;
	kVK_F15 = $71;
	kVK_Help = $72;
	kVK_Home = $73;
	kVK_PageUp = $74;
	kVK_ForwardDelete = $75;
	kVK_F4 = $76;
	kVK_End = $77;
	kVK_F2 = $78;
	kVK_PageDown = $79;
	kVK_F1 = $7A;
	kVK_LeftArrow = $7B;
	kVK_RightArrow = $7C;
	kVK_DownArrow = $7D;
	kVK_UpArrow = $7E;

{ ISO keyboards only}
const
	kVK_ISO_Section = $0A;

{ JIS keyboards only}
const
	kVK_JIS_Yen = $5D;
	kVK_JIS_Underscore = $5E;
	kVK_JIS_KeypadComma = $5F;
	kVK_JIS_Eisu = $66;
	kVK_JIS_Kana = $68;

<<<<<<< HEAD
type
	EventRecord = record
		what: EventKind;
		message: UNSIGNEDLONG;
=======
=======
>>>>>>> origin/fixes_2_2
type
	EventRecord = record
		what: EventKind;
		message: UInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	EventRecord = record
		what: EventKind;
		message: UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
		when: UInt32;
		where: Point;
		modifiers: EventModifiers;
	end;
	EventRecordPtr = ^EventRecord;
type
	FKEYProcPtr = procedure;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
type
>>>>>>> graemeg/fixes_2_2
=======
type
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	FKEYUPP = FKEYProcPtr;
{
 *  NewFKEYUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  DisposeFKEYUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  InvokeFKEYUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  GetMouse()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use HIGetMousePosition instead.
=======
{
 *  GetMouse()
>>>>>>> graemeg/fixes_2_2
=======
{
 *  GetMouse()
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
{
 *  GetMouse()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use HIGetMousePosition instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetMouse( var mouseLoc: Point ); external name '_GetMouse';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{
 *  Button()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetCurrentButtonState or GetCurrentEventButtonState instead.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  Button()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Button()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetCurrentButtonState or GetCurrentEventButtonState instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.6
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function Button: Boolean; external name '_Button';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  StillDown()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
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
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function StillDown: Boolean; external name '_StillDown';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  WaitMouseUp()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
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
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function WaitMouseUp: Boolean; external name '_WaitMouseUp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  KeyTranslate()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use UCKeyTranslate instead.
<<<<<<< HEAD
=======
 *  KeyTranslate()
>>>>>>> graemeg/fixes_2_2
=======
 *  KeyTranslate()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function KeyTranslate( transData: {const} UnivPtr; keycode: UInt16; var state: UInt32 ): UInt32; external name '_KeyTranslate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{
 *  GetCaretTime()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
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
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetCaretTime: UInt32; external name '_GetCaretTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======

{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{$ifc not TARGET_CPU_64 or not defined MACOSALLINCLUDE or not MACOSALLINCLUDE) }
{ GetKeys is available in 64 bit mode, but because we call it from
  an implemented routine, this means that 64 bit apps using MacOSAll
  always have to depend on the Carbon framework unless smart linking
  is used. This is undesirable (since Carbon is unsupported for 64 bit),
  so we put it in a separate unit.
}

<<<<<<< HEAD
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
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ 
    QuickTime 3.0 supports GetKeys() on unix and win32
}
type
    KeyMap = packed array [0..127] of boolean;
{
 *  GetKeys()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure __GetKeys( var theKeys: KeyMap ); external name '_GetKeys';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
		With GPC and FPC on PowerPC, the bytes of the KeyMap must be swapped
}

procedure GetKeys( var theKeys: KeyMap );

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64 or not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64 or not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}


>>>>>>> origin/fixes_2.4
{ Obsolete event types & masks }
const
	networkEvt = 10;
	driverEvt = 11;
	app1Evt = 12;
	app2Evt = 13;
	app3Evt = 14;
	app4Evt = 15;
	networkMask = $0400;
	driverMask = $0800;
	app1Mask = $1000;
	app2Mask = $2000;
	app3Mask = $4000;
	app4Mask = $8000;

type
	EvQEl = record
		qLink: QElemPtr;
		qType: SInt16;
		evtQWhat: EventKind;               { this part is identical to the EventRecord as defined above }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		evtQMessage: UNSIGNEDLONG;
=======
		evtQMessage: UInt32;
>>>>>>> graemeg/fixes_2_2
=======
		evtQMessage: UInt32;
>>>>>>> origin/fixes_2_2
=======
		evtQMessage: UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
		evtQWhen: UInt32;
		evtQWhere: Point;
		evtQModifiers: EventModifiers;
	end;
	EvQElPtr = ^EvQEl;
type
	GetNextEventFilterProcPtr = procedure( var theEvent: EventRecord; var result: Boolean );
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
type
>>>>>>> graemeg/fixes_2_2
=======
type
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	GetNextEventFilterUPP = GetNextEventFilterProcPtr;
{
 *  NewGetNextEventFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  DisposeGetNextEventFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  InvokeGetNextEventFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

type
	GNEFilterUPP = GetNextEventFilterUPP;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  GetDblTime()
 *  
 *  Summary:
 *    Returns the maximum time (in units of 1/60th of a second) allowed
 *    between two consecutive mouse-down events in order for the second
 *    click to be considered a double-click.
 *  
 *  Discussion:
 *    In 64-bit applications, you may replace calls to this API with
 *    calls to NXClickTime (declared in
 *    <IOKit/hidsystem/event_status_driver.h>) or with +[NSEvent
 *    doubleClickInterval] (available in Mac OS X 10.6 and later).
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    The maximum time between mouse-downs allowed for a double-click.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  GetDblTime()
 *  
 *  Summary:
 *    Returns the maximum time (in units of 1/60th of a second) allowed
 *    between two consecutive mouse-down events in order for the second
 *    click to be considered a double-click.
 *  
 *  Discussion:
 *    In 64-bit applications, you may replace calls to this API with
 *    calls to NXClickTime (declared in
 *    <IOKit/hidsystem/event_status_driver.h>) or with +[NSEvent
 *    doubleClickInterval] (available in Mac OS X 10.6 and later).
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    The maximum time between mouse-downs allowed for a double-click.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetDblTime: UInt32; external name '_GetDblTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetEventMask()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
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
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetEventMask( value: EventMask ); external name '_SetEventMask';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  GetNextEvent()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ReceiveNextEvent instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *  GetEvQHdr()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  PPostEvent()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  GetNextEvent()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetNextEvent()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ReceiveNextEvent instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetNextEvent( eventMask_: EventMask; var theEvent: EventRecord ): Boolean; external name '_GetNextEvent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  WaitNextEvent()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ReceiveNextEvent instead.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  WaitNextEvent()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  WaitNextEvent()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use ReceiveNextEvent instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function WaitNextEvent( eventMask_: EventMask; var theEvent: EventRecord; sleep: UInt32; mouseRgn: RgnHandle { can be NULL } ): Boolean; external name '_WaitNextEvent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  EventAvail()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FindSpecificEventInQueue instead.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EventAvail()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  EventAvail()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FindSpecificEventInQueue instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function EventAvail( eventMask_: EventMask; var theEvent: EventRecord ): Boolean; external name '_EventAvail';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  PostEvent()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PostEventToQueue or CGEventPost instead.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PostEvent()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  PostEvent()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PostEventToQueue or CGEventPost instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PostEvent( eventNum: EventKind; eventMsg: UInt32 ): OSErr; external name '_PostEvent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
<<<<<<< HEAD


{$endc} {not TARGET_CPU_64}

=======


{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
{
 *  FlushEvents()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FlushEventsMatchingListFromQueue,
 *    FlushSpecificEventsFromQueue, or FlushEventQueue instead.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{$endc} {not TARGET_CPU_64}

{
 *  FlushEvents()   *** DEPRECATED ***
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  GetOSEvent()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  FlushEvents()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Deprecated:
 *    Use FlushEventsMatchingListFromQueue,
 *    FlushSpecificEventsFromQueue, or FlushEventQueue instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.6
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure FlushEvents( whichMask: EventMask; stopMask: EventMask ); external name '_FlushEvents';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
<<<<<<< HEAD


=======


>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SystemClick()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  SystemTask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
(*
#if OLDROUTINENAMES
#define KeyTrans(transData, keycode, state) KeyTranslate(transData, keycode, state)
#endif  { OLDROUTINENAMES }
*)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{$ifc not TARGET_CPU_64}
{
 *  KeyScript()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TISSelectInputSource API for positive verbs (ScriptCode).
 *     Use TSMDocument properties to restrict input sources:
 *     kTSMDocumentEnabledInputSourcesPropertyTag
 *     kTSMDocumentInputSourceOverridePropertyTag
 *  
 *  Summary:
 *    Switch to the specified script's default (last used) input source.
=======
=======
>>>>>>> origin/fixes_2_2
{
    GetGlobalMouse, GetCurrentKeyModifiers, and CheckEventQueueForUserCancel
    are only available as part of the Carbon API.
}

{
 *  GetGlobalMouse()
 *  
 *  Summary:
 *    Returns the position of the mouse in global coordinates.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure KeyScript( code: SInt16 ); external name '_KeyScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}
=======

>>>>>>> graemeg/cpstrnew

=======


>>>>>>> origin/cpstrnew
{$endc} {not TARGET_CPU_64}

{
 *  IsCmdChar()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use IsUserCancelEventRef or CheckEventQueueForUserCancel instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    globalMouse:
 *      On exit, contains the mouse position in global coordinates.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure GetGlobalMouse( var globalMouse: Point ); external name '_GetGlobalMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
{
 *  KeyScript()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TISSelectInputSource API for positive verbs (ScriptCode).
 *     Use TSMDocument properties to restrict input sources:
 *     kTSMDocumentEnabledInputSourcesPropertyTag
 *     kTSMDocumentInputSourceOverridePropertyTag
 *  
 *  Summary:
 *    Switch to the specified script's default (last used) input source.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure KeyScript( code: SInt16 ); external name '_KeyScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  IsCmdChar()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  IsCmdChar()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use IsUserCancelEventRef or CheckEventQueueForUserCancel instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.6
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function IsCmdChar( const (*var*) event: EventRecord; test: SInt16 ): Boolean; external name '_IsCmdChar';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ 
    LowMem accessor functions previously in LowMem.h
}
{
 *  LMGetKeyThresh()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetKeyThresh: SInt16; external name '_LMGetKeyThresh';
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
 *  LMSetKeyThresh()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
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
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetKeyThresh( value: SInt16 ); external name '_LMSetKeyThresh';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
 *  LMGetKeyRepThresh()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetKeyRepThresh: SInt16; external name '_LMGetKeyRepThresh';
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
 *  LMSetKeyRepThresh()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
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
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetKeyRepThresh( value: SInt16 ); external name '_LMSetKeyRepThresh';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
 *  LMGetKbdLast()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetKbdLast: UInt8; external name '_LMGetKbdLast';
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
 *  LMSetKbdLast()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
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
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetKbdLast( value: UInt8 ); external name '_LMSetKbdLast';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
 *  LMGetKbdType()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetKbdType: UInt8; external name '_LMGetKbdType';
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
 *  LMSetKbdType()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
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
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetKbdType( value: UInt8 ); external name '_LMSetKbdType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}
{$endc} {TARGET_OS_MAC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

{$ifc TARGET_OS_MAC}
=======

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation
>>>>>>> graemeg/cpstrnew
=======

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation
>>>>>>> graemeg/cpstrnew

=======

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

>>>>>>> graemeg/cpstrnew
=======

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}
=======
implementation


>>>>>>> graemeg/fixes_2_2
=======
implementation


>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}
{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ifc TARGET_RT_BIG_ENDIAN}

procedure GetKeys( var theKeys: KeyMap );
var
	theReverseKeys: KeyMap;
	theKey: 0..127;
begin
	__GetKeys( theReverseKeys);
	for theKey:= 0 to 127 do
		theKeys[ theKey]:= theReverseKeys[ ((theKey div 8) * 8) + (7 - (theKey mod 8))]
end;

{$elsec}

procedure GetKeys( var theKeys: KeyMap );
begin
	__GetKeys( theKeys)
end;

{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$endc} {TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

end.

{$endc} {not MACOSALLINCLUDE}
=======

end.
>>>>>>> graemeg/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_MAC}

end.

{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
