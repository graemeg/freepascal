{
     File:       HIToolbox/HIButtonViews.h
 
     Contains:   Definitions of the button views provided by HIToolbox.
 
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
 
     Copyright:  © 2006-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/fixes_2.4
 
}
{       Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
<<<<<<< HEAD
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

unit HIButtonViews;
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
>>>>>>> origin/fixes_2.4
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
uses MacTypes,Appearance,CarbonEvents,Controls,IconsCore,Icons,Menus,QuickdrawTypes,TextEdit,CFBase,HIObject;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
 *  HIButtonViews.h
 *  
 *  Discussion:
 *    API definitions for the simple button views provided by
 *    HIToolbox: pushbutton, checkbox, radio button and radio group,
 *    bevel button, and round button.
 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{   PUSH BUTTON (CDEF 23)                                                              }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Two new variants of the standard pushbutton have been added to the standard control }
{  suite that draw a color icon next to the control title. One variant draws the icon  }
{  on the left side, the other draws it on the right side (when the system justifica-  }
{  tion is right to left, these are reversed).                                         }
{  When either of the icon pushbuttons are created, the contrlMax field of the control }
{  record is used to determine the ID of the 'cicn' resource drawn in the pushbutton.  }
{  In addition, a push button can now be told to draw with a default outline using the }
{  SetControlData routine with the kControlPushButtonDefaultTag below.                 }
{  A push button may also be marked using the kControlPushButtonCancelTag. This has    }
{  no visible representation, but does cause the button to play the CancelButton theme }
{  sound instead of the regular pushbutton theme sound when pressed.                   }
{ Push Button proc IDs }
const
	kControlPushButtonProc = 368;
	kControlPushButLeftIconProc = 374;  { Standard pushbutton with left-side icon}
	kControlPushButRightIconProc = 375;   { Standard pushbutton with right-side icon}

{ Push Button Icon Alignments }
type
	ControlPushButtonIconAlignment = UInt16;
const
	kControlPushButtonIconOnLeft = 6;
	kControlPushButtonIconOnRight = 7;

{ Control Kind Tag }
const
	kControlKindPushButton = FourCharCode('push');
	kControlKindPushIconButton = FourCharCode('picn');

{ The HIObject class ID for the HIPushButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIPushButtonClassID CFSTRP('com.apple.HIPushButton')}
{$endc}
{ Creation APIs: Carbon Only }
{$ifc not TARGET_CPU_64}
{
 *  CreatePushButtonControl()
 *  
 *  Summary:
 *    Creates a push button control.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window that should contain the control. May be NULL on 10.3
 *      and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    title:
 *      The control title. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePushButtonControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreatePushButtonControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CreatePushButtonWithIconControl()
 *  
 *  Summary:
 *    Creates a push button control containing an icon or other
 *    graphical content.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window that should contain the control. May be NULL on 10.3
 *      and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    title:
 *      The control title. May be NULL.
 *    
 *    icon:
 *      The control graphic content.
 *    
 *    iconAlignment:
 *      The alignment of the control graphic content.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePushButtonWithIconControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef { can be NULL }; var icon: ControlButtonContentInfo; iconAlignment: ControlPushButtonIconAlignment; var outControl: ControlRef ): OSStatus; external name '_CreatePushButtonWithIconControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}


{
 *  Summary:
 *    Tagged data supported by standard buttons
 }
const
{
   * Data is a Boolean indicating if a push button is a default button.
   * If so, the button will draw with the appropriate appearance.
   * Available in Mac OS 8.5 and later.
   }
	kControlPushButtonDefaultTag = FourCharCode('dflt');

  {
   * Data is a Boolean indicating if a push button is a cancel button.
   * If so, the button will draw with an appropriate appearance.
   * Available in Mac OS 8.5 and later.
   }
	kControlPushButtonCancelTag = FourCharCode('cncl');

  {
   * Data is a ControlButtonContentInfo or HIViewContentInfo structure.
   * May be used to set or retrieve a button's image content. Available
   * in Mac OS X 10.4 and later. In Mac OS X 10.4, the push button
   * supports the kControlContentCIconRes and kControlContentCGImageRef
   * content types. In Mac OS X 10.5 and later, the push button also
   * supports IconRef content.
   }
	kControlPushButtonContentTag = kControlContentTag;

  {
   * Data is a ControlPushButtonIconAlignment indicating the desired
   * alignment for the button's image content. Applies to all types of
   * image content that may be associated with the push button, not
   * just icons. Available in Mac OS X 10.4 and later.
   }
	kControlPushButtonIconAlignmentTag = FourCharCode('cpia');

  {
   * Data is a Boolean indicating if a push button should animate. Will
   * be True by default even if the button is not a default button and
   * is not currently pulsing. Available in Mac OS X 10.5 and later.
   }
	kControlPushButtonAnimatingTag = FourCharCode('anim');


{
 *  Summary:
 *    Tagged data supported by push buttons
 *  
 *  Discussion:
 *    This new tag is available on Mac OS X 10.4 and later. The
 *    constant is not in the Mac OS X 10.4 and Mac OS X 10.5 headers,
 *    but the constant value is functional on both releases.
 }
const
{
   * Data is a Boolean indicating if a push button is a textured push
   * button, to be drawn on a textured window. Textured was previously
   * referred to as "metal". This attribute is only to be set on push
   * buttons being used in composited windows.
   }
	kControlPushButtonIsTexturedTag = FourCharCode('metl');

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{   CHECKBOX (CDEF 23)                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  The standard checkbox view supports a "mixed" value that indicates that the         }
{  current setting contains a mixed set of on and off values. The control value used   }
{  to display this indication is defined below:                                        }
{      kControlCheckBoxMixedValue = 2                                                  }
{ Check Box proc ID }
const
	kControlCheckBoxProc = 369;

{ Variants with Appearance 1.1 or later }
const
	kControlCheckBoxAutoToggleProc = 371;

{ Control Kind Tag }
const
	kControlKindCheckBox = FourCharCode('cbox');

{ The HIObject class ID for the HICheckBox class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHICheckBoxClassID CFSTRP('com.apple.HICheckBox')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreateCheckBoxControl()
 *  
 *  Summary:
 *    Creates a checkbox control.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window that should contain the control. May be NULL on 10.3
 *      and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    title:
 *      The control title. May be NULL.
 *    
 *    initialValue:
 *      The initial value of the control. Should be zero (off), one
 *      (on), or two (mixed). The control is automatically given a
 *      minimum value of zero and a maximum value of two.
 *    
 *    autoToggle:
 *      Whether this control should have auto-toggle behavior. If true,
 *      the control will automatically toggle between on and off states
 *      when clicked.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateCheckBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef { can be NULL }; initialValue: SInt32; autoToggle: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateCheckBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Check Box Values }
{$endc} {not TARGET_CPU_64}

const
	kControlCheckBoxUncheckedValue = 0;
	kControlCheckBoxCheckedValue = 1;
	kControlCheckBoxMixedValue = 2;


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{   RADIO BUTTON (CDEF 23)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  The standard radio button control supports a "mixed" value that indicates that the  }
{  current setting contains a mixed set of on and off values. The control value used   }
{  to display this indication is defined below:                                        }
{      kControlRadioButtonMixedValue = 2                                               }
{ Radio Button proc IDs }
const
	kControlRadioButtonProc = 370;

{ Variants with Appearance 1.1 or later }
const
	kControlRadioButtonAutoToggleProc = 372;

{ Radio Button Values }
const
	kControlRadioButtonUncheckedValue = 0;
	kControlRadioButtonCheckedValue = 1;
	kControlRadioButtonMixedValue = 2;

{ Control Kind Tag }
const
	kControlKindRadioButton = FourCharCode('rdio');

{ The HIObject class ID for the HIRadioButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIRadioButtonClassID CFSTRP('com.apple.HIRadioButton')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreateRadioButtonControl()
 *  
 *  Summary:
 *    Creates a radio button control.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window that should contain the control. May be NULL on 10.3
 *      and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    title:
 *      The control title. May be NULL.
 *    
 *    initialValue:
 *      The initial value of the control. Should be zero (off), one
 *      (on), or two (mixed). The control is automatically given a
 *      minimum value of zero and a maximum value of two.
 *    
 *    autoToggle:
 *      Whether this control should have auto-toggle behavior. If true,
 *      the control will automatically toggle between on and off states
 *      when clicked. This parameter should be false if the control
 *      will be embedded into a radio group control; in that case, the
 *      radio group will handle setting the correct control value in
 *      response to a click.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateRadioButtonControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef { can be NULL }; initialValue: SInt32; autoToggle: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateRadioButtonControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{   RADIO GROUP (CDEF 26)                                                              }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements a radio group. It is an embedding control and can therefore }
{  only be used when a control hierarchy is established for its owning window. You     }
{  should only embed controls that have radio button behavior - currently radio buttons}
{  and bevel buttons. Specifically, you can embed controls that return the feature bit }
{  kControlHasRadioBehavior. As controls are embedded into it, the group sets up its   }
{  value, min, and max to represent the number of embedded items.                      }
{  The current value of the control is the index of the sub-control that is the current}
{  'on' radio button. To get the current radio button control handle, you can use the  }
{  control manager call GetIndexedSubControl, passing in the value of the radio group. }
{  Note that when creating radio buttons for use in a radio group control, you should  }
{  not use the autoToggle version of the radio button. The radio group control will    }
{  handling toggling the radio button values itself; auto-toggle radio buttons do not  }
{  work properly in a radio group control on Mac OS 9.                                 }
{  NOTE: This control is only available with Appearance 1.0.1.                         }
{ Radio Group Proc ID }
{$endc} {not TARGET_CPU_64}

const
	kControlRadioGroupProc = 416;

{ Control Kind Tag }
const
	kControlKindRadioGroup = FourCharCode('rgrp');

{ The HIObject class ID for the HIRadioGroup class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIRadioGroupClassID CFSTRP('com.apple.HIRadioGroup')}
{$endc}
{ Creation API: Carbon Only }
{$ifc not TARGET_CPU_64}
{
 *  CreateRadioGroupControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateRadioGroupControl( window: WindowRef; const (*var*) boundsRect: Rect; var outControl: ControlRef ): OSStatus; external name '_CreateRadioGroupControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ
     BEVEL BUTTON INTERFACE (CDEF 2)
  
    Bevel buttons allow you to control the content type (pict/icon/etc.), the behavior
   (pushbutton/toggle/sticky), and the bevel size. You also have the option of
    attaching a menu to it. When a menu is present, you can specify which way the
    popup arrow is facing (down or right).

    This is all made possible by overloading the Min, Max, and Value parameters for the
    control, as well as adjusting the variant. Here's the breakdown of what goes where:

    Parameter                   What Goes Here
                
    Min                         Hi Byte = Behavior, Lo Byte = content type.
    Max                         ResID for resource-based content types.
    Value                       MenuID to attach, 0 = no menu, please.

    The variant is broken down into two halves. The low 2 bits control the bevel type.
    Bit 2 controls the popup arrow direction (if a menu is present) and bit 3 controls
    whether or not to use the control's owning window's font.

    Constants for all you need to put this together are below. The values for behaviors
    are set up so that you can simply add them to the content type for use in the Min
    parameter of NewControl.

    An example call:

    control = NewControl( window, &bounds, "\p", true, 0, kControlContentIconSuiteRes +
                            kBehaviorToggles, myIconSuiteID, bevelButtonSmallBevelProc,
                            0L );

    Attaching a menu:

    control = NewControl( window, &bounds, "\p", true, kMyMenuID,
            kControlContentIconSuiteRes, myIconSuiteID, bevelButtonSmallBevelProc +
            kBevelButtonMenuOnRight, 0L );

    This will attach menu ID kMyMenuID to the button, with the popup arrow facing right.
    This also puts the menu up to the right of the button. You can also specify that a
    menu can have multiple items checked at once by adding kBehaviorMultiValueMenus
    into the Min parameter. If you do use multivalue menus, the GetBevelButtonMenuValue
    helper function will return the last item chosen from the menu, whether or not it
    was checked.

    NOTE:   Bevel buttons with menus actually have *two* values. The value of the
            button (on/off), and the value of the menu. The menu value can be gotten
            with the GetBevelButtonMenuValue helper function.

    Handle-based Content
    
    You can create your control and then set the content to an existing handle to an
    icon suite, etc. using the macros below. Please keep in mind that resource-based
    content is owned by the control, handle-based content is owned by you. The CDEF will
    not try to dispose of handle-based content. If you are changing the content type of
    the button on the fly, you must make sure that if you are replacing a handle-
    based content with a resource-based content to properly dispose of the handle,
    else a memory leak will ensue.

    Textual Content
    
    Please note that if a bevel button gets its textual content from the title
    of the control. To alter the textual content of a bevel button, use the
    SetControlTitle[WithCFString] API.

    Implicit Menu Arrow Sizing
    
    Bevel buttons can now have implicit popup menu arrow sizes on Mac OS X 10.5 and later. Use
    SetControlData(..., kControlSizeTag, ...), where the default control size is
    kControlSizeAuto. kControlSizeAuto has the bevel button render its popup menu arrow
    at a size that is dependent on the size of the bevel button -- this is the behavior
    on Tiger and earlier. kControlSizeNormal has it render the normal size arrow and
    kControlSizeSmall has it render the arrow small. All other sizes are invalid.
}
{ Bevel Button Proc IDs }
{$endc} {not TARGET_CPU_64}

const
	kControlBevelButtonSmallBevelProc = 32;
	kControlBevelButtonNormalBevelProc = 33;
	kControlBevelButtonLargeBevelProc = 34;

{ Add these variant codes to kBevelButtonSmallBevelProc to change the type of button }
const
	kControlBevelButtonSmallBevelVariant = 0;
	kControlBevelButtonNormalBevelVariant = 1 shl 0;
	kControlBevelButtonLargeBevelVariant = 1 shl 1;
	kControlBevelButtonMenuOnRightVariant = 1 shl 2;

{ Bevel Thicknesses }
type
	ControlBevelThickness = UInt16;
const
	kControlBevelButtonSmallBevel = 0;
	kControlBevelButtonNormalBevel = 1;
	kControlBevelButtonLargeBevel = 2;

{ Behaviors of bevel buttons. These are set up so you can add  }
{ them together with the content types for use in the Min      }
{ parameter of NewControl. Note that the behavior of a bevel   }
{ button cannot be changed after the button is created.        }
const
	kControlBehaviorPushbutton = 0;
	kControlBehaviorToggles = $0100;
	kControlBehaviorSticky = $0200;
	kControlBehaviorOffsetContents = $8000;
	kControlBehaviorSingleValueMenu = 0;
	kControlBehaviorMultiValueMenu = $4000; { only makes sense when a menu is attached.}

{ Behaviors for 1.0.1 or later }
const
	kControlBehaviorCommandMenu = $2000; { menu holds commands, not choices. Overrides multi-value bit.}

type
	ControlBevelButtonBehavior = UInt16;
	ControlBevelButtonMenuBehavior = UInt16;
{ Bevel Button Menu Placements }
type
	ControlBevelButtonMenuPlacement = UInt16;
const
	kControlBevelButtonMenuOnBottom = 0;
	kControlBevelButtonMenuOnRight = 1 shl 2;

{ Control Kind Tag }
const
	kControlKindBevelButton = FourCharCode('bevl');

{ The HIObject class ID for the HIBevelButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIBevelButtonClassID CFSTRP('com.apple.HIBevelButton')}
{$endc}
{ Creation API: Carbon Only }
{$ifc not TARGET_CPU_64}
{
 *  CreateBevelButtonControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateBevelButtonControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef; thickness: ControlBevelThickness; behavior: ControlBevelButtonBehavior; info: ControlButtonContentInfoPtr; menuID_: MenuID; menuBehavior: ControlBevelButtonMenuBehavior; menuPlacement: ControlBevelButtonMenuPlacement; var outControl: ControlRef ): OSStatus; external name '_CreateBevelButtonControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Graphic Alignments }
{$endc} {not TARGET_CPU_64}

type
	ControlButtonGraphicAlignment = SInt16;
const
	kControlBevelButtonAlignSysDirection = -1; { only left or right}
	kControlBevelButtonAlignCenter = 0;
	kControlBevelButtonAlignLeft = 1;
	kControlBevelButtonAlignRight = 2;
	kControlBevelButtonAlignTop = 3;
	kControlBevelButtonAlignBottom = 4;
	kControlBevelButtonAlignTopLeft = 5;
	kControlBevelButtonAlignBottomLeft = 6;
	kControlBevelButtonAlignTopRight = 7;
	kControlBevelButtonAlignBottomRight = 8;

{ Text Alignments }
type
	ControlButtonTextAlignment = SInt16;
const
	kControlBevelButtonAlignTextSysDirection = teFlushDefault;
	kControlBevelButtonAlignTextCenter = teCenter;
	kControlBevelButtonAlignTextFlushRight = teFlushRight;
	kControlBevelButtonAlignTextFlushLeft = teFlushLeft;

{ Text Placements }
type
	ControlButtonTextPlacement = SInt16;
const
	kControlBevelButtonPlaceSysDirection = -1; { if graphic on right, then on left}
	kControlBevelButtonPlaceNormally = 0;
	kControlBevelButtonPlaceToRightOfGraphic = 1;
	kControlBevelButtonPlaceToLeftOfGraphic = 2;
	kControlBevelButtonPlaceBelowGraphic = 3;
	kControlBevelButtonPlaceAboveGraphic = 4;


{ Data tags supported by the bevel button controls }
const
	kControlBevelButtonContentTag = kControlContentTag; { ControlImageContentInfo}
	kControlBevelButtonTransformTag = FourCharCode('tran'); { IconTransformType}
	kControlBevelButtonTextAlignTag = FourCharCode('tali'); { ButtonTextAlignment}
	kControlBevelButtonTextOffsetTag = FourCharCode('toff'); { SInt16}
	kControlBevelButtonGraphicAlignTag = FourCharCode('gali'); { ButtonGraphicAlignment}
	kControlBevelButtonGraphicOffsetTag = FourCharCode('goff'); { Point}
	kControlBevelButtonTextPlaceTag = FourCharCode('tplc'); { ButtonTextPlacement}
	kControlBevelButtonMenuValueTag = FourCharCode('mval'); { SInt16}
	kControlBevelButtonMenuHandleTag = FourCharCode('mhnd'); { MenuRef}
	kControlBevelButtonMenuRefTag = FourCharCode('mhnd'); { MenuRef}
	kControlBevelButtonCenterPopupGlyphTag = FourCharCode('pglc'); { Boolean: true = center, false = bottom right}

{ These are tags in 1.0.1 or later }
const
	kControlBevelButtonLastMenuTag = FourCharCode('lmnu'); { SInt16: menuID of last menu item selected from}
	kControlBevelButtonMenuDelayTag = FourCharCode('mdly'); { SInt32: ticks to delay before menu appears}

{ tags available with Appearance 1.1 or later }
const
{ Boolean: True = if an icon of the ideal size for}
                                        { the button isn't available, scale a larger or}
                                        { smaller icon to the ideal size. False = don't}
                                        { scale; draw a smaller icon or clip a larger icon.}
                                        { Default is false. Only applies to IconSuites and}
	kControlBevelButtonScaleIconTag = FourCharCode('scal'); { IconRefs.}

{ tags available in Mac OS X and later }
const
	kControlBevelButtonOwnedMenuRefTag = FourCharCode('omrf'); { MenuRef (control will dispose)}
	kControlBevelButtonKindTag = FourCharCode('bebk'); { ThemeButtonKind ( kTheme[Small,Medium,Large,Rounded]BevelButton )}


{
 *  Summary:
 *    Tags available with Mac OS X 10.3 or later
 }
const
{
   * Passed data is an Boolean.  Gets or sets whether or not the
   * associated menu is a multi-value menu or not.  True means that the
   * menu can have multiple selections.
   }
	kControlBevelButtonIsMultiValueMenuTag = FourCharCode('mult');

{ Helper routines are available only thru the shared library/glue. }
{$ifc not TARGET_CPU_64}
{
 *  GetBevelButtonMenuValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetBevelButtonMenuValue( inButton: ControlRef; var outValue: MenuItemIndex ): OSErr; external name '_GetBevelButtonMenuValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetBevelButtonMenuValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonMenuValue( inButton: ControlRef; inValue: MenuItemIndex ): OSErr; external name '_SetBevelButtonMenuValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetBevelButtonMenuHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetBevelButtonMenuHandle( inButton: ControlRef; var outHandle: MenuHandle ): OSErr; external name '_GetBevelButtonMenuHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function GetBevelButtonMenuRef__NAME__GetBevelButtonMenuHandle( possibleWindow: WindowRef ): Boolean; external name '_GetBevelButtonMenuRef__NAME__GetBevelButtonMenuHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetBevelButtonContentInfo()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetBevelButtonContentInfo( inButton: ControlRef; outContent: ControlButtonContentInfoPtr ): OSErr; external name '_GetBevelButtonContentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetBevelButtonContentInfo()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonContentInfo( inButton: ControlRef; inContent: ControlButtonContentInfoPtr ): OSErr; external name '_SetBevelButtonContentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetBevelButtonTransform()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonTransform( inButton: ControlRef; transform: IconTransformType ): OSErr; external name '_SetBevelButtonTransform';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetBevelButtonGraphicAlignment()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonGraphicAlignment( inButton: ControlRef; inAlign: ControlButtonGraphicAlignment; inHOffset: SInt16; inVOffset: SInt16 ): OSErr; external name '_SetBevelButtonGraphicAlignment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetBevelButtonTextAlignment()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonTextAlignment( inButton: ControlRef; inAlign: ControlButtonTextAlignment; inHOffset: SInt16 ): OSErr; external name '_SetBevelButtonTextAlignment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetBevelButtonTextPlacement()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonTextPlacement( inButton: ControlRef; inWhere: ControlButtonTextPlacement ): OSErr; external name '_SetBevelButtonTextPlacement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{   ROUND BUTTON                                                                       }
{  (CDEF 31)                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{$endc} {not TARGET_CPU_64}


{
 *  ControlRoundButtonSize
 *  
 *  Discussion:
 *    Button Sizes
 }
type
	ControlRoundButtonSize = SInt16;
const
{
   * A 20 pixel diameter button.
   }
	kControlRoundButtonNormalSize = kControlSizeNormal;

  {
   * A 25 pixel diameter button.
   }
	kControlRoundButtonLargeSize = kControlSizeLarge;

{ Data tags supported by the round button controls }
const
	kControlRoundButtonContentTag = kControlContentTag; { ControlImageContentInfo}
	kControlRoundButtonSizeTag = kControlSizeTag; { ControlRoundButtonSize}

{ Control Kind Tag }
const
	kControlKindRoundButton = FourCharCode('rndb');

{ The HIObject class ID for the HIRoundButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIRoundButtonClassID CFSTRP('com.apple.HIRoundButton')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreateRoundButtonControl()
 *  
 *  Summary:
 *    Creates a new instance of the Round Button Control.
 *  
 *  Discussion:
 *    CreateRoundButtonControl is preferred over NewControl because it
 *    allows you to specify the exact set of parameters required to
 *    create the control without overloading parameter semantics.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inWindow:
 *      The WindowRef in which to create the control. May be NULL in
 *      10.3 and later.
 *    
 *    inBoundsRect:
 *      The bounding rectangle for the control. The height and width of
 *      the control is fixed (specified by the ControlRoundButtonSize
 *      parameter) and the control will be centered within the
 *      rectangle you specify.
 *    
 *    inSize:
 *      The button size; either kControlRoundButtonNormalSize or
 *      kControlRoundButtonLargeSize.
 *    
 *    inContent:
 *      Any optional content displayed in the button. Currently only
 *      kControlContentIconRef is supported. May be NULL.
 *    
 *    outControl:
 *      On successful exit, this will contain the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateRoundButtonControl( inWindow: WindowRef { can be NULL }; const (*var*) inBoundsRect: Rect; inSize: ControlRoundButtonSize; inContent: ControlButtonContentInfoPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateRoundButtonControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{$ifc OLDROUTINENAMES}
const
	kControlCheckboxUncheckedValue = kControlCheckBoxUncheckedValue;
	kControlCheckboxCheckedValue = kControlCheckBoxCheckedValue;
	kControlCheckboxMixedValue = kControlCheckBoxMixedValue;

{$endc}  { OLDROUTINENAMES }

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
