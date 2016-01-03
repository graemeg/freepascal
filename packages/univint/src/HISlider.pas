{
     File:       HIToolbox/HISlider.h
 
     Contains:   API and type definitions related to HISlider.
 
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
=======
     Version:    HIToolbox-437~1
>>>>>>> origin/fixes_2.4
 
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                     http://www.freepascal.org/bugs.html
 
}
{       Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
>>>>>>> origin/fixes_2.4
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit HISlider;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
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


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

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
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,HIView,MacWindows,Controls,QuickdrawTypes,HIObject;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
 *  Discussion:
 *    There are several variants that control the behavior of the
 *    slider control. Any combination of the following three constants
 *    can be added to the basic CDEF ID (kSliderProc). 
 *    
 *    Mac OS X has a "Scroll to here" option in the General pane of
 *    System Preferences which allows users to click in the page
 *    up/down regions of a slider and have the thumb/indicator jump
 *    directly to the clicked position, which alters the value of the
 *    slider and moves any associated content appropriately. As long as
 *    the mouse button is held down, the click is treated as though the
 *    user had clicked in the thumb/indicator in the first place.
 *    
 *    
 *    If you want the sliders in your application to work with the
 *    "Scroll to here" option, you must do the following: 
 *    
 *    1. Create live-tracking sliders, not sliders that show a "ghost"
 *    thumb when you click on it. You can request live-tracking sliders
 *    by passing true in the liveTracking parameter to
 *    CreateSliderControl. If you create sliders with NewControl, use
 *    the kControlSliderLiveFeedback variant. 
 *    
 *    2. Write an appropriate ControlActionProc and associate it with
 *    your slider via the SetControlAction API. This allows your
 *    application to update its content appropriately when the
 *    live-tracking slider is clicked. 
 *    
 *    3. When calling HandleControlClick or TrackControl, pass -1 in
 *    the action proc parameter. This is a request for the Control
 *    Manager to use the action proc you associated with your control
 *    in step 2. If you rely on the standard window event handler to do
 *    your control tracking, this step is handled for you automatically.
 }
const
	kControlSliderProc = 48;
	kControlSliderLiveFeedback = 1 shl 0;
	kControlSliderHasTickMarks = 1 shl 1;
	kControlSliderReverseDirection = 1 shl 2;
	kControlSliderNonDirectional = 1 shl 3;


{
 *  ControlSliderOrientation
 *  
 *  Discussion:
 *    Slider Orientation
 }
type
	ControlSliderOrientation = UInt16;
const
{
   * If horizontal, the thumb of the slider points downwards. If
   * vertical, the thumb of the slider points to the right. This
   * orientation of slider can show tick marks.
   }
	kControlSliderPointsDownOrRight = 0;

  {
   * If horizontal, the thumb of the slider points upwards. If
   * vertical, the thumb of the slider points to the left. This
   * orientation of slider can show tick marks.
   }
	kControlSliderPointsUpOrLeft = 1;

  {
   * The slider thumb is non-directional. This orientation of slider is
   * not capable of showing tick marks.
   }
	kControlSliderDoesNotPoint = 2;

{ Control Kind Tag }
const
	kControlKindSlider = FourCharCode('sldr');

{ The HIObject class ID for the HISlider class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHISliderClassID CFSTRP('com.apple.HISlider')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreateSliderControl()
 *  
 *  Summary:
 *    Creates a slider control with the specified settings.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window in which to create the slider.
 *    
 *    boundsRect:
 *      The bounds with which to create the slider.
 *    
 *    value:
 *      The initial value of the slider. Should be in the range of the
 *      minimum and maximum values.
 *    
 *    minimum:
 *      The minimum value of the slider. Should be less than the
 *      maximum.
 *    
 *    maximum:
 *      The maximum value of the slider. Should be greater than the
 *      minimum.
 *    
 *    orientation:
 *      The orientation of the slider.
 *    
 *    numTickMarks:
 *      The number of tick marks of the slider. The orientation must
 *      not support tick marks if this value is non-zero.
 *    
 *    liveTracking:
 *    
 *    liveTrackingProc:
 *    
 *    outControl:
 *      A pointer to a ControlRef in which to store a reference to the
 *      created slider.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateSliderControl( window: WindowRef; const (*var*) boundsRect: Rect; value: SInt32; minimum: SInt32; maximum: SInt32; orientation: ControlSliderOrientation; numTickMarks: UInt16; liveTracking: Boolean; liveTrackingProc: ControlActionUPP; var outControl: ControlRef ): OSStatus; external name '_CreateSliderControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HISliderGetThumbOrientation()
 *  
 *  Summary:
 *    Get the orientation of the thumb of a slider.
 *  
 *  Discussion:
 *    Available only on Mac OS X 10.5 and later.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSlider:
 *      A reference to the slider from which to retrieve the thumb
 *      orientation.
 *  
 *  Result:
 *    The current thumb orientation of the specified slider. The
 *    incoming slider reference must be valid or the return value is
 *    undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HISliderGetThumbOrientation( inSlider: HIViewRef ): ControlSliderOrientation; external name '_HISliderGetThumbOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HISliderSetThumbOrientation()
 *  
 *  Summary:
 *    Set the orientation of the thumb of a slider.
 *  
 *  Discussion:
 *    Available only on Mac OS X 10.5 and later. This call may change
 *    the bounds needed to draw the slider fully. If changing the
 *    orientation so that tick marks will appear/disappear, be sure to
 *    resize the control to its optimal size if necessary.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSlider:
 *      A reference to the slider from which to retrieve the thumb
 *      orientation.
 *    
 *    inOrientation:
 *      A ControlSliderOrientation for the desired thumb orientation.
 *  
 *  Result:
 *    An OSStatus code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HISliderSetThumbOrientation( inSlider: HIViewRef; inOrientation: ControlSliderOrientation ): OSStatus; external name '_HISliderSetThumbOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HISliderGetTickMarkCount()
 *  
 *  Summary:
 *    Get the tick mark count of a slider.
 *  
 *  Discussion:
 *    Available only on Mac OS X 10.5 and later.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSlider:
 *      A reference to the slider for which to get the tick mark count.
 *  
 *  Result:
 *    The current tick mark count of the specified slider. The incoming
 *    slider reference must be valid or the return value is undefined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HISliderGetTickMarkCount( inSlider: HIViewRef ): ItemCount; external name '_HISliderGetTickMarkCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HISliderSetTickMarkCount()
 *  
 *  Summary:
 *    Set the tick mark count of a slider. Tick marks will not appear
 *    on a slider unless its orientation is pointy (i.e.
 *    kControlSliderPointsDownOrRight/UpOrLeft) This call may change
 *    the bounds needed to draw the slider fully. If changing the tick
 *    mark count so that tick marks will appear/disappear, be sure to
 *    resize the control to its optimal size if necessary.
 *  
 *  Discussion:
 *    Available only on Mac OS X 10.5 and later.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSlider:
 *      A reference to the slider for which to set the tick mark count.
 *    
 *    inNumTicks:
 *      The desired number of tick marks.
 *  
 *  Result:
 *    An OSStatus code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HISliderSetTickMarkCount( inSlider: HIViewRef; inNumTicks: ItemCount ): OSStatus; external name '_HISliderSetTickMarkCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
