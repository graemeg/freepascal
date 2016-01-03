{
     File:       HIToolbox/HIContainerViews.h
 
     Contains:   Definition of the container views provided by HIToolbox.
 
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

unit HIContainerViews;
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
uses MacTypes,Appearance,CarbonEvents,Controls,Menus,QuickdrawTypes,CFBase,HIObject;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
 *  HIContainerViews.h
 *  
 *  Discussion:
 *    API definitions for the views that can contain other views: group
 *    box, placard, window header, and user pane.
 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ GROUP BOX (CDEF 10)                                                               }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  The group box CDEF can be use in several ways. It can have no title, a text title,  }
{  a check box as the title, or a popup button as a title. There are two versions of   }
{  group boxes, primary and secondary, which look slightly different.                  }
{ Group Box proc IDs }
const
	kControlGroupBoxTextTitleProc = 160;
	kControlGroupBoxCheckBoxProc = 161;
	kControlGroupBoxPopupButtonProc = 162;
	kControlGroupBoxSecondaryTextTitleProc = 164;
	kControlGroupBoxSecondaryCheckBoxProc = 165;
	kControlGroupBoxSecondaryPopupButtonProc = 166;

{ Control Kind Tag }
const
	kControlKindGroupBox = FourCharCode('grpb');
	kControlKindCheckGroupBox = FourCharCode('cgrp');
	kControlKindPopupGroupBox = FourCharCode('pgrp');

{ The HIObject class ID for the HIGroupBox class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIGroupBoxClassID CFSTRP('com.apple.HIGroupBox')}
{$endc}
{ The HIObject class ID for the HICheckBoxGroup class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHICheckBoxGroupClassID CFSTRP('com.apple.HICheckBoxGroup')}
{$endc}
{ Creation APIs: Carbon only }
{$ifc not TARGET_CPU_64}
{
 *  CreateGroupBoxControl()
 *  
 *  Summary:
 *    Creates a group box control.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window that should contain the control.
 *    
 *    boundsRect:
 *      The bounding box of the control.
 *    
 *    title:
 *      The title of the control.
 *    
 *    primary:
 *      Whether to create a primary or secondary group box.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateGroupBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef; primary: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateGroupBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CreateCheckGroupBoxControl()
 *  
 *  Summary:
 *    Creates a group box control that has a check box as its title.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window that should contain the control.
 *    
 *    boundsRect:
 *      The bounding box of the control.
 *    
 *    title:
 *      The title of the control (used as the title of the check box).
 *    
 *    initialValue:
 *      The initial value of the check box.
 *    
 *    primary:
 *      Whether to create a primary or secondary group box.
 *    
 *    autoToggle:
 *      Whether to create an auto-toggling check box. Auto-toggling
 *      check box titles are only supported on Mac OS X; this parameter
 *      must be false when used with CarbonLib.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateCheckGroupBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef; initialValue: SInt32; primary: Boolean; autoToggle: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateCheckGroupBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CreatePopupGroupBoxControl()
 *  
 *  Summary:
 *    Creates a group box control that has a popup button as its title.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window that should contain the control.
 *    
 *    boundsRect:
 *      The bounding box of the control.
 *    
 *    title:
 *      The title of the control (used as the title of the popup
 *      button).
 *    
 *    primary:
 *      Whether to create a primary or secondary group box.
 *    
 *    menuID:
 *      The menu ID of the menu to be displayed by the popup button.
 *    
 *    variableWidth:
 *      Whether the popup button should have a variable-width title.
 *      Fixed-width titles are only supported by Mac OS X; this
 *      parameter must be true when used with CarbonLib.
 *    
 *    titleWidth:
 *      The width in pixels of the popup button title.
 *    
 *    titleJustification:
 *      The justification of the popup button title. Use one of the
 *      TextEdit justification constants here (teFlushDefault,
 *      teCenter, teFlushRight, or teFlushLeft).
 *    
 *    titleStyle:
 *      The QuickDraw text style of the popup button title.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePopupGroupBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef; primary: Boolean; menuID_: MenuID; variableWidth: Boolean; titleWidth: SInt16; titleJustification: SInt16; titleStyle: Style; var outControl: ControlRef ): OSStatus; external name '_CreatePopupGroupBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by group box }
{$endc} {not TARGET_CPU_64}

const
	kControlGroupBoxMenuHandleTag = FourCharCode('mhan'); { MenuRef (popup title only)}
	kControlGroupBoxMenuRefTag = FourCharCode('mhan'); { MenuRef (popup title only)}
	kControlGroupBoxFontStyleTag = kControlFontStyleTag; { ControlFontStyleRec}

{ tags available with Appearance 1.1 or later }
const
	kControlGroupBoxTitleRectTag = FourCharCode('trec'); { Rect. Rectangle that the title text/control is drawn in. (get only)}


{
 *  Summary:
 *    Tags available with Mac OS X 10.3 or later
 }
const
{
   * Passed data is a Rect.  Returns the full rectangle that content is
   * drawn in (get only). This is slightly different than the content
   * region, as it also includes the frame drawn around the content.
   }
	kControlGroupBoxFrameRectTag = FourCharCode('frec');


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PLACARD (CDEF 14)                                                                 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Placard proc IDs }
const
	kControlPlacardProc = 224;

{ Control Kind Tag }
const
	kControlKindPlacard = FourCharCode('plac');

{ The HIObject class ID for the HIPlacardView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIPlacardViewClassID CFSTRP('com.apple.HIPlacardView')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreatePlacardControl()
 *  
 *  Summary:
 *    Creates a placard control.
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
 *      The bounding box of the control.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePlacardControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; var outControl: ControlRef ): OSStatus; external name '_CreatePlacardControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ WINDOW HEADER (CDEF 21)                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Window Header proc IDs }
{$endc} {not TARGET_CPU_64}

const
	kControlWindowHeaderProc = 336;  { normal header}
	kControlWindowListViewHeaderProc = 337; { variant for list views - no bottom line}

{ Control Kind Tag }
const
	kControlKindWindowHeader = FourCharCode('whed');


{
 *  Summary:
 *    Tags available with Mac OS X 10.3 or later
 }
const
{
   * Passed data is a Boolean.  Set to true if the control is to draw
   * as a list header.
   }
	kControlWindowHeaderIsListHeaderTag = FourCharCode('islh');

{ The HIObject class ID for the HIWindowHeaderView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIWindowHeaderViewClassID CFSTRP('com.apple.HIWindowHeaderView')}
{$endc}
{ Creation API: Carbon Only }
{$ifc not TARGET_CPU_64}
{
 *  CreateWindowHeaderControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateWindowHeaderControl( window: WindowRef; const (*var*) boundsRect: Rect; isListHeader: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateWindowHeaderControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ USER PANE (CDEF 16)                                                               }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  User panes have two primary purposes: to allow easy implementation of a custom      }
{  control by the developer, and to provide a generic container for embedding other    }
{  controls.                                                                           }
{  In Carbon, with the advent of Carbon-event-based controls, you may find it easier   }
{  to simply write a new control from scratch than to customize a user pane control.   }
{  The set of callbacks provided by the user pane will not be extended to support      }
{  new Control Manager features; instead, you should just write a real control.        }
{  User panes do not, by default, support embedding. If you try to embed a control     }
{  into a user pane, you will get back errControlIsNotEmbedder. You can make a user    }
{  pane support embedding by passing the kControlSupportsEmbedding flag in the 'value' }
{  parameter when you create the control.                                              }
{  User panes support the following overloaded control initialization options:         }
{  Parameter                   What Goes Here                                          }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ         ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ      }
{  Value                       Control feature flags                                   }

{ User Pane proc IDs }
{$endc} {not TARGET_CPU_64}

const
	kControlUserPaneProc = 256;

{ Control Kind Tag }
const
	kControlKindUserPane = FourCharCode('upan');

{ The HIObject class ID for the HIUserPane class. Valid in Mac OS X 10.4 and later. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIUserPaneClassID CFSTRP('com.apple.HIUserPane')}
{$endc}
{ Creation API: Carbon only }
{$ifc not TARGET_CPU_64}
{
 *  CreateUserPaneControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateUserPaneControl( window: WindowRef; const (*var*) boundsRect: Rect; features: UInt32; var outControl: ControlRef ): OSStatus; external name '_CreateUserPaneControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by user panes }
{ Currently, they are all proc ptrs for doing things like drawing and hit testing, etc. }
{$endc} {not TARGET_CPU_64}

const
	kControlUserItemDrawProcTag = FourCharCode('uidp'); { UserItemUPP}
	kControlUserPaneDrawProcTag = FourCharCode('draw'); { ControlUserPaneDrawUPP}
	kControlUserPaneHitTestProcTag = FourCharCode('hitt'); { ControlUserPaneHitTestUPP}
	kControlUserPaneTrackingProcTag = FourCharCode('trak'); { ControlUserPaneTrackingUPP}
	kControlUserPaneIdleProcTag = FourCharCode('idle'); { ControlUserPaneIdleUPP}
	kControlUserPaneKeyDownProcTag = FourCharCode('keyd'); { ControlUserPaneKeyDownUPP}
	kControlUserPaneActivateProcTag = FourCharCode('acti'); { ControlUserPaneActivateUPP}
	kControlUserPaneFocusProcTag = FourCharCode('foci'); { ControlUserPaneFocusUPP}
	kControlUserPaneBackgroundProcTag = FourCharCode('back'); { ControlUserPaneBackgroundUPP (32-bit only)}

type
	ControlUserPaneDrawProcPtr = procedure( control: ControlRef; part: ControlPartCode );
	ControlUserPaneHitTestProcPtr = function( control: ControlRef; where: Point ): ControlPartCode;
	ControlUserPaneTrackingProcPtr = function( control: ControlRef; startPt: Point; actionProc: ControlActionUPP ): ControlPartCode;
	ControlUserPaneIdleProcPtr = procedure( control: ControlRef );
	ControlUserPaneKeyDownProcPtr = function( control: ControlRef; keyCode: SInt16; charCode: SInt16; modifiers: SInt16 ): ControlPartCode;
	ControlUserPaneActivateProcPtr = procedure( control: ControlRef; activating: Boolean );
	ControlUserPaneFocusProcPtr = function( control: ControlRef; action: ControlFocusPart ): ControlPartCode;
	ControlUserPaneDrawUPP = ControlUserPaneDrawProcPtr;
	ControlUserPaneHitTestUPP = ControlUserPaneHitTestProcPtr;
	ControlUserPaneTrackingUPP = ControlUserPaneTrackingProcPtr;
	ControlUserPaneIdleUPP = ControlUserPaneIdleProcPtr;
	ControlUserPaneKeyDownUPP = ControlUserPaneKeyDownProcPtr;
	ControlUserPaneActivateUPP = ControlUserPaneActivateProcPtr;
	ControlUserPaneFocusUPP = ControlUserPaneFocusProcPtr;
{
 *  NewControlUserPaneDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlUserPaneDrawUPP( userRoutine: ControlUserPaneDrawProcPtr ): ControlUserPaneDrawUPP; external name '_NewControlUserPaneDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewControlUserPaneHitTestUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlUserPaneHitTestUPP( userRoutine: ControlUserPaneHitTestProcPtr ): ControlUserPaneHitTestUPP; external name '_NewControlUserPaneHitTestUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewControlUserPaneTrackingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlUserPaneTrackingUPP( userRoutine: ControlUserPaneTrackingProcPtr ): ControlUserPaneTrackingUPP; external name '_NewControlUserPaneTrackingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewControlUserPaneIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlUserPaneIdleUPP( userRoutine: ControlUserPaneIdleProcPtr ): ControlUserPaneIdleUPP; external name '_NewControlUserPaneIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewControlUserPaneKeyDownUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlUserPaneKeyDownUPP( userRoutine: ControlUserPaneKeyDownProcPtr ): ControlUserPaneKeyDownUPP; external name '_NewControlUserPaneKeyDownUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewControlUserPaneActivateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlUserPaneActivateUPP( userRoutine: ControlUserPaneActivateProcPtr ): ControlUserPaneActivateUPP; external name '_NewControlUserPaneActivateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewControlUserPaneFocusUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlUserPaneFocusUPP( userRoutine: ControlUserPaneFocusProcPtr ): ControlUserPaneFocusUPP; external name '_NewControlUserPaneFocusUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlUserPaneDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlUserPaneDrawUPP( userUPP: ControlUserPaneDrawUPP ); external name '_DisposeControlUserPaneDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlUserPaneHitTestUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlUserPaneHitTestUPP( userUPP: ControlUserPaneHitTestUPP ); external name '_DisposeControlUserPaneHitTestUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlUserPaneTrackingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlUserPaneTrackingUPP( userUPP: ControlUserPaneTrackingUPP ); external name '_DisposeControlUserPaneTrackingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlUserPaneIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlUserPaneIdleUPP( userUPP: ControlUserPaneIdleUPP ); external name '_DisposeControlUserPaneIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlUserPaneKeyDownUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlUserPaneKeyDownUPP( userUPP: ControlUserPaneKeyDownUPP ); external name '_DisposeControlUserPaneKeyDownUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlUserPaneActivateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlUserPaneActivateUPP( userUPP: ControlUserPaneActivateUPP ); external name '_DisposeControlUserPaneActivateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlUserPaneFocusUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlUserPaneFocusUPP( userUPP: ControlUserPaneFocusUPP ); external name '_DisposeControlUserPaneFocusUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlUserPaneDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeControlUserPaneDrawUPP( control: ControlRef; part: ControlPartCode; userUPP: ControlUserPaneDrawUPP ); external name '_InvokeControlUserPaneDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlUserPaneHitTestUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeControlUserPaneHitTestUPP( control: ControlRef; where: Point; userUPP: ControlUserPaneHitTestUPP ): ControlPartCode; external name '_InvokeControlUserPaneHitTestUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlUserPaneTrackingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeControlUserPaneTrackingUPP( control: ControlRef; startPt: Point; actionProc: ControlActionUPP; userUPP: ControlUserPaneTrackingUPP ): ControlPartCode; external name '_InvokeControlUserPaneTrackingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlUserPaneIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeControlUserPaneIdleUPP( control: ControlRef; userUPP: ControlUserPaneIdleUPP ); external name '_InvokeControlUserPaneIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlUserPaneKeyDownUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeControlUserPaneKeyDownUPP( control: ControlRef; keyCode: SInt16; charCode: SInt16; modifiers: SInt16; userUPP: ControlUserPaneKeyDownUPP ): ControlPartCode; external name '_InvokeControlUserPaneKeyDownUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlUserPaneActivateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeControlUserPaneActivateUPP( control: ControlRef; activating: Boolean; userUPP: ControlUserPaneActivateUPP ); external name '_InvokeControlUserPaneActivateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlUserPaneFocusUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeControlUserPaneFocusUPP( control: ControlRef; action: ControlFocusPart; userUPP: ControlUserPaneFocusUPP ): ControlPartCode; external name '_InvokeControlUserPaneFocusUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc not TARGET_CPU_64}
type
	ControlUserPaneBackgroundProcPtr = procedure( control: ControlRef; info: ControlBackgroundPtr );
	ControlUserPaneBackgroundUPP = ControlUserPaneBackgroundProcPtr;
{
 *  NewControlUserPaneBackgroundUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlUserPaneBackgroundUPP( userRoutine: ControlUserPaneBackgroundProcPtr ): ControlUserPaneBackgroundUPP; external name '_NewControlUserPaneBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlUserPaneBackgroundUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlUserPaneBackgroundUPP( userUPP: ControlUserPaneBackgroundUPP ); external name '_DisposeControlUserPaneBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlUserPaneBackgroundUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeControlUserPaneBackgroundUPP( control: ControlRef; info: ControlBackgroundPtr; userUPP: ControlUserPaneBackgroundUPP ); external name '_InvokeControlUserPaneBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
