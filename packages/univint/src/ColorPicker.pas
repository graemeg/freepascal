{
<<<<<<< HEAD
     File:       CommonPanels/ColorPicker.h
 
     Contains:   Color Picker package Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    CommonPanels-94~602
=======
     Version:    CommonPanels-91~177
>>>>>>> graemeg/cpstrnew
=======
     Version:    CommonPanels-91~177
>>>>>>> graemeg/cpstrnew
=======
     Version:    CommonPanels-91~177
>>>>>>> graemeg/cpstrnew
=======
     Version:    CommonPanels-91~177
>>>>>>> origin/cpstrnew
 
     Copyright:  � 1987-2008 by Apple Computer, Inc., all rights reserved
=======
     File:       ColorPicker.p
 
     Contains:   Color Picker package Interfaces.
 
     Version:    Technology: System 7.5
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  � 1987-2002 by Apple Computer, Inc., all rights reserved
>>>>>>> graemeg/fixes_2_2
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/cpstrnew
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
                     http://www.freepascal.org/bugs.html
 
}


{
    Modified for use with Free Pascal
    Version 210
    Please report any bugs to <gpc@microbizz.nl>
}

>>>>>>> graemeg/fixes_2_2
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit ColorPicker;
interface
<<<<<<< HEAD
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> graemeg/fixes_2_2

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
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
=======
>>>>>>> graemeg/fixes_2_2
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
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

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}

=======
>>>>>>> graemeg/fixes_2_2

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
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
	{$setc TARGET_CPU_X86 := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
{$elsec}
	{$error Neither __ppc__ nor __i386__ is defined.}
{$endc}
{$setc TARGET_CPU_PPC_64 := FALSE}
>>>>>>> graemeg/fixes_2_2

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
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
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
uses MacTypes,ColorSyncDeprecated,MixedMode,QuickdrawTypes,Events;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{$ifc not TARGET_CPU_64}
const
{Maximum small fract value, as long}
	kMaximumSmallFract = $0000FFFF;

{$endc} {not TARGET_CPU_64}

{ These are legacy constants. The Color Picker on OS X uses the Cocoa NSColorPanel. }
const
	kDefaultColorPickerWidth = 383;
	kDefaultColorPickerHeight = 238;

type
	DialogPlacementSpec = SInt16;
const
	kAtSpecifiedOrigin = 0;
	kDeepestColorScreen = 1;
	kCenterOnMainScreen = 2;

{ Since OS X uses the Cocoa NSColorPanel, the flags below are no longer used. }
const
	kColorPickerDialogIsMoveable = 1;
	kColorPickerDialogIsModal = 2;
	kColorPickerCanModifyPalette = 4;
	kColorPickerCanAnimatePalette = 8;
	kColorPickerAppIsColorSyncAware = 16;
	kColorPickerInSystemDialog = 32;
	kColorPickerInApplicationDialog = 64;
	kColorPickerInPickerDialog = 128;
	kColorPickerDetachedFromChoices = 256;
	kColorPickerCallColorProcLive = 512;

{$ifc OLDROUTINENAMES}
{$ifc not TARGET_CPU_64}
const
{Maximum small fract value, as long}
	MaxSmallFract = $0000FFFF;

{$endc} {not TARGET_CPU_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
	kDefaultWidth = 383;
	kDefaultHeight = 238;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew

const
	kDefaultWidth = 383;
	kDefaultHeight = 238;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Since OS X uses the Cocoa NSColorPanel, the flags below are no longer used. }
const
	DialogIsMoveable = 1;
	DialogIsModal = 2;
	CanModifyPalette = 4;
	CanAnimatePalette = 8;
	AppIsColorSyncAware = 16;
	InSystemDialog = 32;
	InApplicationDialog = 64;
	InPickerDialog = 128;
	DetachedFromChoices = 256;
	CallColorProcLive = 512;

{$endc} {OLDROUTINENAMES}

{$ifc not TARGET_CPU_64}
{ A SmallFract value is just the fractional part of a Fixed number,
which is the low order word.  They can be
assigned directly to and from INTEGERs. }
{ Unsigned fraction between 0 and 1 }
type
	SmallFract = UInt16;
	HSVColorPtr = ^HSVColor;
	HSVColor = record
		hue: SmallFract;                    { Fraction of circle, red at 0 }
		saturation: SmallFract;             { 0-1, 0 for gray, 1 for pure color }
		value: SmallFract;                  { 0-1, 0 for black, 1 for max intensity }
	end;
type
	HSLColorPtr = ^HSLColor;
	HSLColor = record
		hue: SmallFract;                    { Fraction of circle, red at 0 }
		saturation: SmallFract;             { 0-1, 0 for gray, 1 for pure color }
		lightness: SmallFract;              { 0-1, 0 for black, 1 for white }
	end;
type
	CMYColorPtr = ^CMYColor;
	CMYColor = record
		cyan: SmallFract;
		magenta: SmallFract;
		yellow: SmallFract;
	end;
type
	PMColor = record
		profile: CMProfileHandle;
		color: CMColor;
	end;
	PMColorPtr = ^PMColor;
{$endc} {not TARGET_CPU_64}

type
	NPMColor = record
		profile: CMProfileRef;
		color: CMColor;
	end;
	NPMColorPtr = ^NPMColor;
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Picker = ^OpaquePicker; { an opaque type }
	OpaquePicker = record end;
=======
	Picker = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	Picker = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	Picker = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	Picker = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	PickerPtr = ^Picker;  { when a var xx:Picker parameter can be nil, it is changed to xx: PickerPtr }
{ Since OS X uses the Cocoa NSColorPanel, the struct below is no longer used. }
type
	PickerMenuItemInfoPtr = ^PickerMenuItemInfo;
	PickerMenuItemInfo = record
		editMenuID: SInt16;
		cutItem: SInt16;
		copyItem: SInt16;
		pasteItem: SInt16;
		clearItem: SInt16;
		undoItem: SInt16;
	end;
{ The following proc ptr is the only supported way to communicate with the Cocoa NSColorPanel on OS X. }
type
	NColorChangedProcPtr = procedure( userData: SRefCon; var newColor: NPMColor );
	NColorChangedUPP = NColorChangedProcPtr;
{
 *  NewNColorChangedUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewNColorChangedUPP( userRoutine: NColorChangedProcPtr ): NColorChangedUPP; external name '_NewNColorChangedUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeNColorChangedUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeNColorChangedUPP( userUPP: NColorChangedUPP ); external name '_DisposeNColorChangedUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeNColorChangedUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeNColorChangedUPP( userData: SRefCon; var newColor: NPMColor; userUPP: NColorChangedUPP ); external name '_InvokeNColorChangedUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc TARGET_CPU_64}

type
	ColorChangedUPP = UnivPtr;
	UserEventUPP = UnivPtr;
{$elsec}
type
	ColorChangedProcPtr = procedure( userData: SInt32; var newColor: PMColor );
	UserEventProcPtr = function( var event: EventRecord ): Boolean;
	ColorChangedUPP = ColorChangedProcPtr;
	UserEventUPP = UserEventProcPtr;
{
 *  NewColorChangedUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewColorChangedUPP( userRoutine: ColorChangedProcPtr ): ColorChangedUPP; external name '_NewColorChangedUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewUserEventUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewUserEventUPP( userRoutine: UserEventProcPtr ): UserEventUPP; external name '_NewUserEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeColorChangedUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeColorChangedUPP( userUPP: ColorChangedUPP ); external name '_DisposeColorChangedUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeUserEventUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeUserEventUPP( userUPP: UserEventUPP ); external name '_DisposeUserEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeColorChangedUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeColorChangedUPP( userData: SInt32; var newColor: PMColor; userUPP: ColorChangedUPP ); external name '_InvokeColorChangedUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeUserEventUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeUserEventUPP( var event: EventRecord; userUPP: UserEventUPP ): Boolean; external name '_InvokeUserEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
type
	ColorPickerInfo = record
		theColor: PMColor;
		dstProfile: CMProfileHandle;
		flags: UInt32;
		placeWhere: DialogPlacementSpec;
		dialogOrigin: Point;
		pickerType: OSType;
		eventProc: UserEventUPP;
		colorProc: ColorChangedUPP;
		colorProcData: UInt32;
		prompt: Str255;
		mInfo: PickerMenuItemInfo;
		newColorChosen: Boolean;
		filler: SInt8;
	end;
{$endc} {not TARGET_CPU_64}

type
	NColorPickerInfo = record
		theColor: NPMColor;
		dstProfile: CMProfileRef;             { Currently ignored }
		flags: UInt32;                  { Currently ignored }
		placeWhere: DialogPlacementSpec;            { Currently ignored }
		dialogOrigin: Point;           { Currently ignored }
		pickerType: OSType;             { Currently ignored }
{$ifc not TARGET_CPU_64}

		eventProc: UserEventUPP;              { Ignored }
{$endc} {not TARGET_CPU_64}

		colorProc: NColorChangedUPP;
		colorProcData: URefCon;
		prompt: Str255;                 { Currently ignored }
		mInfo: PickerMenuItemInfo;                  { Ignored }
		newColorChosen: Boolean;
		reserved: UInt8;               { Must be 0 }
	end;

{$ifc not TARGET_CPU_64}
{
 *  Fix2SmallFract()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function Fix2SmallFract( f: Fixed ): SmallFract; external name '_Fix2SmallFract';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SmallFract2Fix()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SmallFract2Fix( s: SmallFract ): Fixed; external name '_SmallFract2Fix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CMY2RGB()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure CMY2RGB( const (*var*) cColor: CMYColor; var rColor: RGBColor ); external name '_CMY2RGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RGB2CMY()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure RGB2CMY( const (*var*) rColor: RGBColor; var cColor: CMYColor ); external name '_RGB2CMY';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HSL2RGB()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure HSL2RGB( const (*var*) hColor: HSLColor; var rColor: RGBColor ); external name '_HSL2RGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RGB2HSL()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure RGB2HSL( const (*var*) rColor: RGBColor; var hColor: HSLColor ); external name '_RGB2HSL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HSV2RGB()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure HSV2RGB( const (*var*) hColor: HSVColor; var rColor: RGBColor ); external name '_HSV2RGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RGB2HSV()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure RGB2HSV( const (*var*) rColor: RGBColor; var hColor: HSVColor ); external name '_RGB2HSV';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{
 *  GetColor()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetColor( where: Point; const (*var*) prompt: Str255; const (*var*) inColor: RGBColor; var outColor: RGBColor ): Boolean; external name '_GetColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  PickColor()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ColorPickerLib 2.0 and later
 }
function PickColor( var theColorInfo: ColorPickerInfo ): OSErr; external name '_PickColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{
 *  NPickColor()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ColorPickerLib 2.1 and later
 }
function NPickColor( var theColorInfo: NColorPickerInfo ): OSErr; external name '_NPickColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
uses MacTypes,CMTypes,CMICCProfile,MixedMode,Quickdraw,CMApplication,Events;


{$ALIGN MAC68K}


const
																{ Maximum small fract value, as long }
	kMaximumSmallFract			= $0000FFFF;

	kDefaultColorPickerWidth	= 383;
	kDefaultColorPickerHeight	= 238;


type
	DialogPlacementSpec 		= SInt16;
const
	kAtSpecifiedOrigin			= 0;
	kDeepestColorScreen			= 1;
	kCenterOnMainScreen			= 2;


	{	 These are for the flags field in the structs below (for example ColorPickerInfo). 	}
	kColorPickerDialogIsMoveable = 1;
	kColorPickerDialogIsModal	= 2;
	kColorPickerCanModifyPalette = 4;
	kColorPickerCanAnimatePalette = 8;
	kColorPickerAppIsColorSyncAware = 16;
	kColorPickerInSystemDialog	= 32;
	kColorPickerInApplicationDialog = 64;
	kColorPickerInPickerDialog	= 128;
	kColorPickerDetachedFromChoices = 256;
	kColorPickerCallColorProcLive = 512;


{$ifc OLDROUTINENAMES}
																{ Maximum small fract value, as long }
	MaxSmallFract				= $0000FFFF;

	kDefaultWidth				= 383;
	kDefaultHeight				= 238;

	{	 These are for the flags field in the structs below (for example ColorPickerInfo). 	}
	DialogIsMoveable			= 1;
	DialogIsModal				= 2;
	CanModifyPalette			= 4;
	CanAnimatePalette			= 8;
	AppIsColorSyncAware			= 16;
	InSystemDialog				= 32;
	InApplicationDialog			= 64;
	InPickerDialog				= 128;
	DetachedFromChoices			= 256;
	CallColorProcLive			= 512;

{$endc}  {OLDROUTINENAMES}

	{	 A SmallFract value is just the fractional part of a Fixed number,
	which is the low order word.  SmallFracts are used to save room,
	and to be compatible with Quickdraw's RGBColor.  They can be
	assigned directly to and from INTEGERs. 	}
	{	 Unsigned fraction between 0 and 1 	}

type
	SmallFract							= UInt16;
	{	 For developmental simplicity in switching between the HLS and HSV
	models, HLS is reordered into HSL. Thus both models start with
	hue and saturation values; value/lightness/brightness is last. 	}

	HSVColorPtr = ^HSVColor;
	HSVColor = record
		hue:					SmallFract;								{ Fraction of circle, red at 0 }
		saturation:				SmallFract;								{ 0-1, 0 for gray, 1 for pure color }
		value:					SmallFract;								{ 0-1, 0 for black, 1 for max intensity }
	end;

	HSLColorPtr = ^HSLColor;
	HSLColor = record
		hue:					SmallFract;								{ Fraction of circle, red at 0 }
		saturation:				SmallFract;								{ 0-1, 0 for gray, 1 for pure color }
		lightness:				SmallFract;								{ 0-1, 0 for black, 1 for white }
	end;

	CMYColorPtr = ^CMYColor;
	CMYColor = record
		cyan:					SmallFract;
		magenta:				SmallFract;
		yellow:					SmallFract;
	end;

	PMColorPtr = ^PMColor;
	PMColor = record
		profile:				CMProfileHandle;
		color:					CMColor;
	end;

	NPMColorPtr = ^NPMColor;
	NPMColor = record
		profile:				CMProfileRef;
		color:					CMColor;
	end;

	Picker    = ^SInt32; { an opaque 32-bit type }
	PickerPtr = ^Picker;  { when a var xx:Picker parameter can be nil, it is changed to xx: PickerPtr }
	PickerMenuItemInfoPtr = ^PickerMenuItemInfo;
	PickerMenuItemInfo = record
		editMenuID:				SInt16;
		cutItem:				SInt16;
		copyItem:				SInt16;
		pasteItem:				SInt16;
		clearItem:				SInt16;
		undoItem:				SInt16;
	end;

	{	 Structs related to deprecated API's have been pulled from this file. 	}
	{	 Those structs necessary for developers writing their own color pickers... 	}
	{	 have been moved to ColorPickerComponents.h. 	}

{$ifc TYPED_FUNCTION_POINTERS}
	ColorChangedProcPtr = procedure(userData: SInt32; var newColor: PMColor);
{$elsec}
	ColorChangedProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	NColorChangedProcPtr = procedure(userData: SInt32; var newColor: NPMColor);
{$elsec}
	NColorChangedProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	UserEventProcPtr = function(var event: EventRecord): boolean;
{$elsec}
	UserEventProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	ColorChangedUPP = ^SInt32; { an opaque UPP }
{$elsec}
	ColorChangedUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	NColorChangedUPP = ^SInt32; { an opaque UPP }
{$elsec}
	NColorChangedUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	UserEventUPP = ^SInt32; { an opaque UPP }
{$elsec}
	UserEventUPP = UniversalProcPtr;
{$endc}	
	ColorPickerInfoPtr = ^ColorPickerInfo;
	ColorPickerInfo = record
		theColor:				PMColor;
		dstProfile:				CMProfileHandle;
		flags:					UInt32;
		placeWhere:				DialogPlacementSpec;
		dialogOrigin:			Point;
		pickerType:				OSType;
		eventProc:				UserEventUPP;
		colorProc:				ColorChangedUPP;
		colorProcData:			UInt32;
		prompt:					Str255;
		mInfo:					PickerMenuItemInfo;
		newColorChosen:			boolean;
		filler:					SInt8;
	end;

	NColorPickerInfoPtr = ^NColorPickerInfo;
	NColorPickerInfo = record
		theColor:				NPMColor;
		dstProfile:				CMProfileRef;
		flags:					UInt32;
		placeWhere:				DialogPlacementSpec;
		dialogOrigin:			Point;
		pickerType:				OSType;
		eventProc:				UserEventUPP;
		colorProc:				NColorChangedUPP;
		colorProcData:			UInt32;
		prompt:					Str255;
		mInfo:					PickerMenuItemInfo;
		newColorChosen:			boolean;
		reserved:				SInt8;									{ Must be 0 }
	end;


	{   Below are the color conversion routines. }
	{
	 *  Fix2SmallFract()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         not available
	 	}
function Fix2SmallFract(f: Fixed): SmallFract; external name '_Fix2SmallFract';
{
 *  SmallFract2Fix()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
function SmallFract2Fix(s: SmallFract): Fixed; external name '_SmallFract2Fix';
{
 *  CMY2RGB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure CMY2RGB(const (*var*) cColor: CMYColor; var rColor: RGBColor); external name '_CMY2RGB';
{
 *  RGB2CMY()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure RGB2CMY(const (*var*) rColor: RGBColor; var cColor: CMYColor); external name '_RGB2CMY';
{
 *  HSL2RGB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure HSL2RGB(const (*var*) hColor: HSLColor; var rColor: RGBColor); external name '_HSL2RGB';
{
 *  RGB2HSL()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure RGB2HSL(const (*var*) rColor: RGBColor; var hColor: HSLColor); external name '_RGB2HSL';
{
 *  HSV2RGB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure HSV2RGB(const (*var*) hColor: HSVColor; var rColor: RGBColor); external name '_HSV2RGB';
{
 *  RGB2HSV()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure RGB2HSV(const (*var*) rColor: RGBColor; var hColor: HSVColor); external name '_RGB2HSV';
{   GetColor() works with or without the Color Picker extension. }
{
 *  GetColor()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
function GetColor(where: Point; const (*var*) prompt: Str255; const (*var*) inColor: RGBColor; var outColor: RGBColor): boolean; external name '_GetColor';
{   PickColor() requires the Color Picker extension (version 2.0 or greater). }
{
 *  PickColor()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in ColorPickerLib 2.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
function PickColor(var theColorInfo: ColorPickerInfo): OSErr; external name '_PickColor';
{   NPickColor() requires the Color Picker extension (version 2.1 or greater). }
{
 *  NPickColor()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in ColorPickerLib 2.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
function NPickColor(var theColorInfo: NColorPickerInfo): OSErr; external name '_NPickColor';
{ A suite of mid-level API calls have been deprecated.  Likely you never...  }
{ used them anyway.  They were removed from this file and should not be... }
{ used in the future as they are not gauranteed to be supported. }

const
	uppColorChangedProcInfo = $000003C0;
	uppNColorChangedProcInfo = $000003C0;
	uppUserEventProcInfo = $000000D0;
	{
	 *  NewColorChangedUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         not available
	 	}
function NewColorChangedUPP(userRoutine: ColorChangedProcPtr): ColorChangedUPP; external name '_NewColorChangedUPP'; { old name was NewColorChangedProc }
{
 *  NewNColorChangedUPP()
 *  
 *  Availability:
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
function NewNColorChangedUPP(userRoutine: NColorChangedProcPtr): NColorChangedUPP; external name '_NewNColorChangedUPP'; { old name was NewNColorChangedProc }
{
 *  NewUserEventUPP()
 *  
 *  Availability:
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
function NewUserEventUPP(userRoutine: UserEventProcPtr): UserEventUPP; external name '_NewUserEventUPP'; { old name was NewUserEventProc }
{
 *  DisposeColorChangedUPP()
 *  
 *  Availability:
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure DisposeColorChangedUPP(userUPP: ColorChangedUPP); external name '_DisposeColorChangedUPP';
{
 *  DisposeNColorChangedUPP()
 *  
 *  Availability:
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure DisposeNColorChangedUPP(userUPP: NColorChangedUPP); external name '_DisposeNColorChangedUPP';
{
 *  DisposeUserEventUPP()
 *  
 *  Availability:
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure DisposeUserEventUPP(userUPP: UserEventUPP); external name '_DisposeUserEventUPP';
{
 *  InvokeColorChangedUPP()
 *  
 *  Availability:
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure InvokeColorChangedUPP(userData: SInt32; var newColor: PMColor; userRoutine: ColorChangedUPP); external name '_InvokeColorChangedUPP'; { old name was CallColorChangedProc }
{
 *  InvokeNColorChangedUPP()
 *  
 *  Availability:
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure InvokeNColorChangedUPP(userData: SInt32; var newColor: NPMColor; userRoutine: NColorChangedUPP); external name '_InvokeNColorChangedUPP'; { old name was CallNColorChangedProc }
{
 *  InvokeUserEventUPP()
 *  
 *  Availability:
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
function InvokeUserEventUPP(var event: EventRecord; userRoutine: UserEventUPP): boolean; external name '_InvokeUserEventUPP'; { old name was CallUserEventProc }
{$ALIGN MAC68K}


end.
>>>>>>> graemeg/fixes_2_2
