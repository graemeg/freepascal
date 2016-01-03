{
     File:       HIToolbox/ControlDefinitions.h
 
     Contains:   Definitions of controls provided by the Control Manager
 
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
 
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
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
 
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
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

unit ControlDefinitions;
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> origin/fixes_2.4
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
uses MacTypes,TextEdit,AXUIElement,AEDataModel,CFBase,Events,QuickdrawTypes,IconsCore,CFData,CFDictionary,CFString,DateTimeUtils,Drag,TextCommon,Appearance,CarbonEvents,Controls,Lists,MacHelp,Menus,HIObject;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

=======
=======

>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
{$ifc TARGET_OS_MAC}
=======
uses MacTypes,TextEdit,AXUIElement,AEDataModel,CFBase,Events,Quickdraw,Icons,CFData,CFDictionary,DateTimeUtils,Drag,TextCommon,Appearance,CarbonEvents,Controls,Lists,MacHelp,Menus,CFString;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,TextEdit,AXUIElement,AEDataModel,CFBase,Events,Quickdraw,Icons,CFData,CFDictionary,DateTimeUtils,Drag,TextCommon,Appearance,CarbonEvents,Controls,Lists,MacHelp,Menus,CFString;

>>>>>>> origin/fixes_2_2
=======
uses MacTypes,TextEdit,AXUIElement,AEDataModel,CFBase,Events,QuickdrawTypes,IconsCore,CFData,CFDictionary,CFString,DateTimeUtils,Drag,TextCommon,Appearance,CarbonEvents,Controls,Lists,MacHelp,Menus,HIObject;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}


{
 *  ControlDefinitions.h
 *  
 *  Discussion:
 *    System software supplies a variety of controls for your
 *    applications to use. They are described herein.
 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{  ¥ Resource Types                                                                                    }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

const
	kControlTabListResType = FourCharCode('tab#'); { used for tab control (Appearance 1.0 and later)}
	kControlListDescResType = FourCharCode('ldes'); { used for list box control (Appearance 1.0 and later)}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Check Box Values                                                                  }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kControlCheckBoxUncheckedValue = 0;
	kControlCheckBoxCheckedValue = 1;
	kControlCheckBoxMixedValue = 2;

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Radio Button Values                                                               }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kControlRadioButtonUncheckedValue = 0;
	kControlRadioButtonCheckedValue = 1;
	kControlRadioButtonMixedValue = 2;

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Pop-Up Menu Control Constants                                                     }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Variant codes for the System 7 pop-up menu}
const
	popupFixedWidth = 1 shl 0;
	popupVariableWidth = 1 shl 1;
	popupUseAddResMenu = 1 shl 2;
	popupUseWFont = 1 shl 3;

{ Menu label styles for the System 7 pop-up menu}
const
	popupTitleBold = 1 shl 8;
	popupTitleItalic = 1 shl 9;
	popupTitleUnderline = 1 shl 10;
	popupTitleOutline = 1 shl 11;
	popupTitleShadow = 1 shl 12;
	popupTitleCondense = 1 shl 13;
	popupTitleExtend = 1 shl 14;
	popupTitleNoStyle = 1 shl 15;

{ Menu label justifications for the System 7 pop-up menu}
const
	popupTitleLeftJust = $00000000;
	popupTitleCenterJust = $00000001;
	popupTitleRightJust = $000000FF;

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{  ¥ Control Definition IDÕs                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Standard System 7 procIDs}

const
	pushButProc = 0;
	checkBoxProc = 1;
	radioButProc = 2;
	scrollBarProc = 16;
	popupMenuProc = 1008;

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Control Part Codes                                                                }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kControlLabelPart = 1;
	kControlMenuPart = 2;
	kControlTrianglePart = 4;
	kControlEditTextPart = 5;    { Appearance 1.0 and later}
	kControlPicturePart = 6;    { Appearance 1.0 and later}
	kControlIconPart = 7;    { Appearance 1.0 and later}
	kControlClockPart = 8;    { Appearance 1.0 and later}
	kControlListBoxPart = 24;   { Appearance 1.0 and later}
	kControlListBoxDoubleClickPart = 25;  { Appearance 1.0 and later}
	kControlImageWellPart = 26;   { Appearance 1.0 and later}
	kControlRadioGroupPart = 27;   { Appearance 1.0.2 and later}
	kControlButtonPart = 10;
	kControlCheckBoxPart = 11;
	kControlRadioButtonPart = 11;
	kControlUpButtonPart = kAppearancePartUpButton;
	kControlDownButtonPart = kAppearancePartDownButton;
	kControlPageUpPart = kAppearancePartPageUpArea;
	kControlPageDownPart = kAppearancePartPageDownArea;
	kControlClockHourDayPart = 9;    { Appearance 1.1 and later}
	kControlClockMinuteMonthPart = 10;   { Appearance 1.1 and later}
	kControlClockSecondYearPart = 11;   { Appearance 1.1 and later}
	kControlClockAMPMPart = 12;   { Appearance 1.1 and later}
	kControlDataBrowserPart = 24;   { CarbonLib 1.0 and later}
	kControlDataBrowserDraggedPart = 25;   { CarbonLib 1.0 and later}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{--------------------------------------------------------------------------------------}
{  ¥ DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ
    ¥ EDIT TEXT (CDEF 17)
  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ
    Use the EditUnicodeText control to replace the Edit Text control. It uses the same
    data tags as the Edit Text control.
}
<<<<<<< HEAD

{ Edit Text proc IDs }
<<<<<<< HEAD
<<<<<<< HEAD
const
	kControlEditTextProc = 272;
	kControlEditTextPasswordProc = 274;

{ proc IDs available with Appearance 1.1 or later }
const
	kControlEditTextInlineInputProc = 276; { Can't combine with the other variants}

{ Control Kind Tag }
const
	kControlKindEditText = FourCharCode('etxt');
<<<<<<< HEAD

{$ifc not TARGET_CPU_64}
{
 *  CreateEditTextControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateEditUnicodeTextControl API instead.
 *  
 *  Summary:
 *    Creates a new edit text control.
 *  
 *  Discussion:
 *    This control is a legacy control. It is deprecated in favor of
 *    the EditUnicodeText control, which handles Unicode and draws its
 *    text using antialiasing.
=======
const
	kControlEditTextProc = 272;
	kControlEditTextPasswordProc = 274;

{ proc IDs available with Appearance 1.1 or later }
const
	kControlEditTextInlineInputProc = 276; { Can't combine with the other variants}

{ Control Kind Tag }
const
	kControlKindEditText = FourCharCode('etxt');

{$ifc not TARGET_CPU_64}
{
 *  CreateEditTextControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateEditUnicodeTextControl API instead.
 *  
 *  Summary:
 *    Creates a new edit text control.
 *  
 *  Discussion:
 *    This control is a legacy control. It is deprecated in favor of
 *    the EditUnicodeText control, which handles Unicode and draws its
 *    text using antialiasing.
=======
const
	kControlEditTextProc = 272;
	kControlEditTextPasswordProc = 274;

{ proc IDs available with Appearance 1.1 or later }
const
	kControlEditTextInlineInputProc = 276; { Can't combine with the other variants}

{ Control Kind Tag }
const
	kControlKindEditText = FourCharCode('etxt');

{$ifc not TARGET_CPU_64}
{
 *  CreateEditTextControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateEditUnicodeTextControl API instead.
 *  
 *  Summary:
 *    Creates a new edit text control.
 *  
 *  Discussion:
 *    This control is a legacy control. It is deprecated in favor of
 *    the EditUnicodeText control, which handles Unicode and draws its
 *    text using antialiasing.
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ Edit Text proc IDs }
const
	kControlEditTextProc = 272;
	kControlEditTextPasswordProc = 274;

{ proc IDs available with Appearance 1.1 or later }
const
	kControlEditTextInlineInputProc = 276; { Can't combine with the other variants}

{ Control Kind Tag }
const
	kControlKindEditText = FourCharCode('etxt');

{$ifc not TARGET_CPU_64}
{
 *  CreateEditTextControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateEditUnicodeTextControl API instead.
 *  
 *  Summary:
 *    Creates a new edit text control.
 *  
 *  Discussion:
 *    This control is a legacy control. It is deprecated in favor of
 *    the EditUnicodeText control, which handles Unicode and draws its
 *    text using antialiasing.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    isPassword:
 *      A Boolean indicating whether the field is to be used as a
 *      password field. Passing false indicates that the field is to
 *      display entered text normally. True means that the field will
 *      be used as a password field and any text typed into the field
 *      will be displayed only as bullets.
 *    
 *    useInlineInput:
 *      A Boolean indicating whether or not the control is to accept
 *      inline input. Pass true to to accept inline input, otherwise
 *      pass false.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control (if noErr is returned as the
 *      result code).
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateEditTextControl( window: WindowRef; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; isPassword: Boolean; useInlineInput: Boolean; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateEditTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ ControlData tags supported only by the classic EditText control}
{$endc} {not TARGET_CPU_64}

const
	kControlEditTextTEHandleTag = FourCharCode('than'); { The TEHandle of the text edit record}
	kControlEditTextInlinePreUpdateProcTag = FourCharCode('prup'); { TSMTEPreUpdateUPP and TSMTEPostUpdateUpp. For use with inline input variant...}
	kControlEditTextInlinePostUpdateProcTag = FourCharCode('poup'); { ...The refCon parameter will contain the ControlRef.}

{
    The classic EditText control also supports these tags defined for the EditUnicodeText control:
    
        kControlEditTextLockedTag
        kControlEditTextStyleTag
        kControlEditTextFixedTextTag
        kControlEditTextTextTag
        kControlEditTextKeyFilterTag
        kControlEditTextValidationProcTag
        kControlEditTextSelectionTag
        kControlEditTextKeyScriptBehaviorTag
        kControlEditTextCFStringTag
        kControlEditTextPasswordTag
        kControlEditTextPasswordCFStringTag
}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PICTURE CONTROL (CDEF 19)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Value parameter should contain the ID of the picture you wish to display when       }
{  creating controls of this type. If you don't want the control tracked at all, use   }
{  the 'no track' variant.                                                             }
{ Picture control proc IDs }
const
	kControlPictureProc = 304;
	kControlPictureNoTrackProc = 305;   { immediately returns kControlPicturePart}

{ Control Kind Tag }
const
	kControlKindPicture = FourCharCode('pict');

{ The HIObject class ID for the HIPictureView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIPictureViewClassID CFSTRP('com.apple.HIPictureView')}
{$endc}
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  CreateBevelButtonControl()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  CreatePictureControl()
 *  
 *  Summary:
 *    Creates a picture control.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Parameters:
 *    
 *    window:
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    isPassword:
 *      A Boolean indicating whether the field is to be used as a
 *      password field. Passing false indicates that the field is to
 *      display entered text normally. True means that the field will
 *      be used as a password field and any text typed into the field
 *      will be displayed only as bullets.
 *    
 *    useInlineInput:
 *      A Boolean indicating whether or not the control is to accept
 *      inline input. Pass true to to accept inline input, otherwise
 *      pass false.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control (if noErr is returned as the
 *      result code).
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateEditTextControl( window: WindowRef; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; isPassword: Boolean; useInlineInput: Boolean; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateEditTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ ControlData tags supported only by the classic EditText control}
{$endc} {not TARGET_CPU_64}

const
	kControlEditTextTEHandleTag = FourCharCode('than'); { The TEHandle of the text edit record}
	kControlEditTextInlinePreUpdateProcTag = FourCharCode('prup'); { TSMTEPreUpdateUPP and TSMTEPostUpdateUpp. For use with inline input variant...}
	kControlEditTextInlinePostUpdateProcTag = FourCharCode('poup'); { ...The refCon parameter will contain the ControlRef.}

{
    The classic EditText control also supports these tags defined for the EditUnicodeText control:
    
        kControlEditTextLockedTag
        kControlEditTextStyleTag
        kControlEditTextFixedTextTag
        kControlEditTextTextTag
        kControlEditTextKeyFilterTag
        kControlEditTextValidationProcTag
        kControlEditTextSelectionTag
        kControlEditTextKeyScriptBehaviorTag
        kControlEditTextCFStringTag
        kControlEditTextPasswordTag
        kControlEditTextPasswordCFStringTag
}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PICTURE CONTROL (CDEF 19)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Value parameter should contain the ID of the picture you wish to display when       }
{  creating controls of this type. If you don't want the control tracked at all, use   }
{  the 'no track' variant.                                                             }
{ Picture control proc IDs }
const
	kControlPictureProc = 304;
	kControlPictureNoTrackProc = 305;   { immediately returns kControlPicturePart}

{ Control Kind Tag }
const
	kControlKindPicture = FourCharCode('pict');

{ The HIObject class ID for the HIPictureView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIPictureViewClassID CFSTRP('com.apple.HIPictureView')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreatePictureControl()
 *  
 *  Summary:
 *    Creates a picture control.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Parameters:
 *    
 *    window:
 *      The window that should contain the control. May be NULL on 10.3
 *      and later.
 *    
 *    boundsRect:
 *      The bounding box of the control.
 *    
 *    content:
 *      The descriptor for the picture you want the control to display.
 *    
 *    dontTrack:
 *      A Boolean value indicating whether the control should hilite
 *      when it is clicked on. False means hilite and track the mouse.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePictureControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; const (*var*) content: ControlButtonContentInfo; dontTrack: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreatePictureControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by picture controls }
{$endc} {not TARGET_CPU_64}

const
	kControlPictureHandleTag = FourCharCode('pich'); { PicHandle}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ LIST BOX (CDEF 22)                                                                }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Lists use an auxiliary resource to define their format. The resource type used is   }
{  'ldes' and a definition for it can be found in Appearance.r. The resource ID for    }
{  the ldes is passed in the 'value' parameter when creating the control. You may pass }
{  zero in value. This tells the List Box control to not use a resource. The list will }
{  be created with default values, and will use the standard LDEF (0). You can change  }
{  the list by getting the list handle. You can set the LDEF to use by using the tag   }
{  below (kControlListBoxLDEFTag)                                                      }
{ List Box proc IDs }
const
	kControlListBoxProc = 352;
	kControlListBoxAutoSizeProc = 353;

{ Control Kind Tag }
const
	kControlKindListBox = FourCharCode('lbox');

{$ifc not TARGET_CPU_64}
{
 *  CreateListBoxControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateListBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; autoSize: Boolean; numRows: SInt16; numColumns: SInt16; horizScroll: Boolean; vertScroll: Boolean; cellHeight: SInt16; cellWidth: SInt16; hasGrowSpace: Boolean; const (*var*) listDef: ListDefSpec; var outControl: ControlRef ): OSStatus; external name '_CreateListBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by list box }
{$endc} {not TARGET_CPU_64}

const
	kControlListBoxListHandleTag = FourCharCode('lhan'); { ListHandle}
	kControlListBoxKeyFilterTag = kControlKeyFilterTag; { ControlKeyFilterUPP}
	kControlListBoxFontStyleTag = kControlFontStyleTag; { ControlFontStyleRec}

{ New tags in 1.0.1 or later }
const
	kControlListBoxDoubleClickTag = FourCharCode('dblc'); { Boolean. Was last click a double-click?}
	kControlListBoxLDEFTag = FourCharCode('ldef'); { SInt16. ID of LDEF to use.}

{ Resource Types }
const
	kControlListDescResType = FourCharCode('ldes'); { used for list box control (Appearance 1.0 and later)}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ SCROLL TEXT BOX (CDEF 27)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements a scrolling box of (non-editable) text. This is useful for  }
{  credits in about boxes, etc.                                                        }
{  The standard version of this control has a scroll bar, but the autoscrolling        }
{  variant does not. The autoscrolling variant needs two pieces of information to      }
{  work: delay (in ticks) before the scrolling starts, and time (in ticks) between     }
{  scrolls. It will scroll one pixel at a time, unless changed via SetControlData.     }
{  Parameter                   What Goes Here                                          }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ         ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ    }
{  Value                       Resource ID of 'TEXT'/'styl' content.                   }
{  Min                         Scroll start delay (in ticks)                       .   }
{  Max                         Delay (in ticks) between scrolls.                       }
{  NOTE: This control is only available with Appearance 1.1.                           }
{ Scroll Text Box Proc IDs }
const
	kControlScrollTextBoxProc = 432;
	kControlScrollTextBoxAutoScrollProc = 433;

{ Control Kind Tag }
const
	kControlKindScrollingTextBox = FourCharCode('stbx');

{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  GetBevelButtonMenuValue()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  CreateScrollingTextBoxControl()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Parameters:
 *    
 *    window:
<<<<<<< HEAD
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    isPassword:
 *      A Boolean indicating whether the field is to be used as a
 *      password field. Passing false indicates that the field is to
 *      display entered text normally. True means that the field will
 *      be used as a password field and any text typed into the field
 *      will be displayed only as bullets.
 *    
 *    useInlineInput:
 *      A Boolean indicating whether or not the control is to accept
 *      inline input. Pass true to to accept inline input, otherwise
 *      pass false.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control (if noErr is returned as the
 *      result code).
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateEditTextControl( window: WindowRef; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; isPassword: Boolean; useInlineInput: Boolean; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateEditTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *      The window that should contain the control. May be NULL on 10.3
 *      and later.
 *    
 *    boundsRect:
 *      The bounding box of the control.
 *    
 *    content:
 *      The descriptor for the picture you want the control to display.
 *    
 *    dontTrack:
 *      A Boolean value indicating whether the control should hilite
 *      when it is clicked on. False means hilite and track the mouse.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePictureControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; const (*var*) content: ControlButtonContentInfo; dontTrack: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreatePictureControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by picture controls }
{$endc} {not TARGET_CPU_64}

const
	kControlPictureHandleTag = FourCharCode('pich'); { PicHandle}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ LIST BOX (CDEF 22)                                                                }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Lists use an auxiliary resource to define their format. The resource type used is   }
{  'ldes' and a definition for it can be found in Appearance.r. The resource ID for    }
{  the ldes is passed in the 'value' parameter when creating the control. You may pass }
{  zero in value. This tells the List Box control to not use a resource. The list will }
{  be created with default values, and will use the standard LDEF (0). You can change  }
{  the list by getting the list handle. You can set the LDEF to use by using the tag   }
{  below (kControlListBoxLDEFTag)                                                      }
{ List Box proc IDs }
const
	kControlListBoxProc = 352;
	kControlListBoxAutoSizeProc = 353;

{ Control Kind Tag }
const
	kControlKindListBox = FourCharCode('lbox');

{$ifc not TARGET_CPU_64}
{
 *  CreateListBoxControl()
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetBevelButtonMenuValue( inButton: ControlRef; var outValue: SInt16 ): OSErr; external name '_GetBevelButtonMenuValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetBevelButtonMenuValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonMenuValue( inButton: ControlRef; inValue: SInt16 ): OSErr; external name '_SetBevelButtonMenuValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetBevelButtonMenuHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonContentInfo( inButton: ControlRef; inContent: ControlButtonContentInfoPtr ): OSErr; external name '_SetBevelButtonContentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetBevelButtonTransform()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateListBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; autoSize: Boolean; numRows: SInt16; numColumns: SInt16; horizScroll: Boolean; vertScroll: Boolean; cellHeight: SInt16; cellWidth: SInt16; hasGrowSpace: Boolean; const (*var*) listDef: ListDefSpec; var outControl: ControlRef ): OSStatus; external name '_CreateListBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

{ ControlData tags supported only by the classic EditText control}
{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
const
	kControlEditTextTEHandleTag = FourCharCode('than'); { The TEHandle of the text edit record}
	kControlEditTextInlinePreUpdateProcTag = FourCharCode('prup'); { TSMTEPreUpdateUPP and TSMTEPostUpdateUpp. For use with inline input variant...}
	kControlEditTextInlinePostUpdateProcTag = FourCharCode('poup'); { ...The refCon parameter will contain the ControlRef.}

{
    The classic EditText control also supports these tags defined for the EditUnicodeText control:
    
        kControlEditTextLockedTag
        kControlEditTextStyleTag
        kControlEditTextFixedTextTag
        kControlEditTextTextTag
        kControlEditTextKeyFilterTag
        kControlEditTextValidationProcTag
        kControlEditTextSelectionTag
        kControlEditTextKeyScriptBehaviorTag
        kControlEditTextCFStringTag
        kControlEditTextPasswordTag
        kControlEditTextPasswordCFStringTag
}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PICTURE CONTROL (CDEF 19)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Value parameter should contain the ID of the picture you wish to display when       }
{  creating controls of this type. If you don't want the control tracked at all, use   }
{  the 'no track' variant.                                                             }
{ Picture control proc IDs }
const
	kControlPictureProc = 304;
	kControlPictureNoTrackProc = 305;   { immediately returns kControlPicturePart}

{ Control Kind Tag }
const
	kControlKindPicture = FourCharCode('pict');

{ The HIObject class ID for the HIPictureView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIPictureViewClassID CFSTRP('com.apple.HIPictureView')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreatePictureControl()
 *  
 *  Summary:
 *    Creates a picture control.
>>>>>>> graemeg/cpstrnew
=======
{ Tagged data supported by list box }
{$endc} {not TARGET_CPU_64}

const
	kControlListBoxListHandleTag = FourCharCode('lhan'); { ListHandle}
	kControlListBoxKeyFilterTag = kControlKeyFilterTag; { ControlKeyFilterUPP}
	kControlListBoxFontStyleTag = kControlFontStyleTag; { ControlFontStyleRec}

{ New tags in 1.0.1 or later }
const
	kControlListBoxDoubleClickTag = FourCharCode('dblc'); { Boolean. Was last click a double-click?}
	kControlListBoxLDEFTag = FourCharCode('ldef'); { SInt16. ID of LDEF to use.}

{ Resource Types }
const
	kControlListDescResType = FourCharCode('ldes'); { used for list box control (Appearance 1.0 and later)}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ SCROLL TEXT BOX (CDEF 27)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements a scrolling box of (non-editable) text. This is useful for  }
{  credits in about boxes, etc.                                                        }
{  The standard version of this control has a scroll bar, but the autoscrolling        }
{  variant does not. The autoscrolling variant needs two pieces of information to      }
{  work: delay (in ticks) before the scrolling starts, and time (in ticks) between     }
{  scrolls. It will scroll one pixel at a time, unless changed via SetControlData.     }
{  Parameter                   What Goes Here                                          }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ         ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ    }
{  Value                       Resource ID of 'TEXT'/'styl' content.                   }
{  Min                         Scroll start delay (in ticks)                       .   }
{  Max                         Delay (in ticks) between scrolls.                       }
{  NOTE: This control is only available with Appearance 1.1.                           }
{ Scroll Text Box Proc IDs }
const
	kControlScrollTextBoxProc = 432;
	kControlScrollTextBoxAutoScrollProc = 433;

{ Control Kind Tag }
const
	kControlKindScrollingTextBox = FourCharCode('stbx');

{$ifc not TARGET_CPU_64}
{
 *  CreateScrollingTextBoxControl()
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetBevelButtonTextPlacement( inButton: ControlRef; inWhere: ControlButtonTextPlacement ): OSErr; external name '_SetBevelButtonTextPlacement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ SLIDER (CDEF 3)                                                                   }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  There are several variants that control the behavior of the slider control. Any     }
{  combination of the following three constants can be added to the basic CDEF ID      }
{  (kSliderProc).                                                                      }
{  Variants:                                                                           }
{      kSliderLiveFeedback     Slider does not use "ghosted" indicator when tracking.  }
{                              ActionProc is called (set via SetControlAction) as the  }
{                              indicator is dragged. The value is updated so that the  }
{                              actionproc can adjust some other property based on the  }
{                              value each time the action proc is called. If no action }
{                              proc is installed, it reverts to the ghost indicator.   }
{      kSliderHasTickMarks     Slider is drawn with 'tick marks'. The control          }
{                              rectangle must be large enough to accomidate the tick   }
{                              marks.                                                  }
{      kSliderReverseDirection Slider thumb points in opposite direction than normal.  }
{                              If the slider is vertical, the thumb will point to the  }
{                              left, if the slider is horizontal, the thumb will point }
{                              upwards.                                                }
{      kSliderNonDirectional   This option overrides the kSliderReverseDirection and   }
{                              kSliderHasTickMarks variants. It creates an indicator   }
{                              which is rectangular and doesn't point in any direction }
{                              like the normal indicator does.                         }
{  Mac OS X has a "Scroll to here" option in the General pane of System Preferences    }
{  which allows users to click in the page up/down regions of a slider and have the    }
{  thumb/indicator jump directly to the clicked position, which alters the value of    }
{  the slider and moves any associated content appropriately. As long as the mouse     }
{  button is held down, the click is treated as though the user had clicked in the     }
{  thumb/indicator in the first place.                                                 }
{  If you want the sliders in your application to work with the "Scroll to here"       }
{  option, you must do the following:                                                  }
{  1. Create live-tracking sliders, not sliders that show a "ghost" thumb when you     }
{  click on it. You can request live-tracking sliders by passing true in the           }
{  liveTracking parameter to CreateSliderControl. If you create sliders with           }
{  NewControl, use the kControlSliderLiveFeedback variant.                             }
{  2. Write an appropriate ControlActionProc and associate it with your slider via     }
{  the SetControlAction API. This allows your application to update its content        }
{  appropriately when the live-tracking slider is clicked.                             }
{  3. When calling HandleControlClick or TrackControl, pass -1 in the action proc      }
{  parameter. This is a request for the Control Manager to use the action proc you     }
{  associated with your control in step 2. If you rely on the standard window event    }
{  handler to do your control tracking, this step is handled for you automatically.    }
{ Slider proc ID and variants }
const
	kControlSliderProc = 48;
	kControlSliderLiveFeedback = 1 shl 0;
	kControlSliderHasTickMarks = 1 shl 1;
	kControlSliderReverseDirection = 1 shl 2;
	kControlSliderNonDirectional = 1 shl 3;

{ Slider Orientation }
type
	ControlSliderOrientation = UInt16;
const
	kControlSliderPointsDownOrRight = 0;
	kControlSliderPointsUpOrLeft = 1;
	kControlSliderDoesNotPoint = 2;

{ Control Kind Tag }
const
	kControlKindSlider = FourCharCode('sldr');

{ The HIObject class ID for the HISlider class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHISliderClassID CFSTRP('com.apple.HISlider')}
{$endc}
{ Creation API: Carbon Only }
{
 *  CreateSliderControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateSliderControl( window: WindowRef; const (*var*) boundsRect: Rect; value: SInt32; minimum: SInt32; maximum: SInt32; orientation: ControlSliderOrientation; numTickMarks: UInt16; liveTracking: Boolean; liveTrackingProc: ControlActionUPP; var outControl: ControlRef ): OSStatus; external name '_CreateSliderControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ DISCLOSURE TRIANGLE (CDEF 4)                                                      }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control can be used as either left or right facing. It can also handle its own }
{  tracking if you wish. This means that when the 'autotoggle' variant is used, if the }
{  user clicks the control, it's state will change automatically from open to closed   }
{  and vice-versa depending on its initial state. After a successful call to Track-    }
{  Control, you can just check the current value to see what state it was switched to. }
{ Triangle proc IDs }
const
	kControlTriangleProc = 64;
	kControlTriangleLeftFacingProc = 65;
	kControlTriangleAutoToggleProc = 66;
	kControlTriangleLeftFacingAutoToggleProc = 67;

type
	ControlDisclosureTriangleOrientation = UInt16;
const
	kControlDisclosureTrianglePointDefault = 0; { points right on a left-to-right script system (Mac OS X and later or CarbonLib 1.5 and later only)}
	kControlDisclosureTrianglePointRight = 1;
	kControlDisclosureTrianglePointLeft = 2;

{ Control Kind Tag }
const
	kControlKindDisclosureTriangle = FourCharCode('dist');

{ The HIObject class ID for the HIDisclosureTriangle class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIDisclosureTriangleClassID CFSTRP('com.apple.HIDisclosureTriangle')}
{$endc}
{
 *  CreateDisclosureTriangleControl()
 *  
 *  Summary:
 *    Creates a Disclosure Triangle control at a specific position in
 *    the specified window.
 *  
 *  Discussion:
 *    Disclosure Triangles are small controls that give the user a way
 *    to toggle the visibility of information or other user interface.
 *    When information is in a hidden state, a Disclosure Triangle is
 *    considered "closed" and should point to the right (or sometimes
 *    to the left). When the user clicks on it, the Disclosure Triangle
 *    rotates downwards into the "open" state. The application should
 *    repond by revealing the appropriate information or interface. On
 *    Mac OS X, a root control will be created for the window if one
 *    does not already exist. If a root control exists for the window,
 *    the Disclosure Triangle control will be embedded into it.
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
<<<<<<< HEAD
 *  Parameters:
 *    
 *    window:
<<<<<<< HEAD
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    isPassword:
 *      A Boolean indicating whether the field is to be used as a
 *      password field. Passing false indicates that the field is to
 *      display entered text normally. True means that the field will
 *      be used as a password field and any text typed into the field
 *      will be displayed only as bullets.
 *    
 *    useInlineInput:
 *      A Boolean indicating whether or not the control is to accept
 *      inline input. Pass true to to accept inline input, otherwise
 *      pass false.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control (if noErr is returned as the
 *      result code).
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateEditTextControl( window: WindowRef; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; isPassword: Boolean; useInlineInput: Boolean; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateEditTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ ControlData tags supported only by the classic EditText control}
{$endc} {not TARGET_CPU_64}

=======

{$ifc not TARGET_CPU_64}
{
 *  CreateEditTextControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateEditUnicodeTextControl API instead.
 *  
 *  Summary:
 *    Creates a new edit text control.
 *  
 *  Discussion:
 *    This control is a legacy control. It is deprecated in favor of
 *    the EditUnicodeText control, which handles Unicode and draws its
 *    text using antialiasing.
=======
 *      The window that should contain the control. May be NULL on 10.3
 *      and later.
 *    
 *    boundsRect:
 *      The bounding box of the control.
 *    
 *    content:
 *      The descriptor for the picture you want the control to display.
 *    
 *    dontTrack:
 *      A Boolean value indicating whether the control should hilite
 *      when it is clicked on. False means hilite and track the mouse.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePictureControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; const (*var*) content: ControlButtonContentInfo; dontTrack: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreatePictureControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by picture controls }
{$endc} {not TARGET_CPU_64}

const
	kControlPictureHandleTag = FourCharCode('pich'); { PicHandle}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ LIST BOX (CDEF 22)                                                                }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Lists use an auxiliary resource to define their format. The resource type used is   }
{  'ldes' and a definition for it can be found in Appearance.r. The resource ID for    }
{  the ldes is passed in the 'value' parameter when creating the control. You may pass }
{  zero in value. This tells the List Box control to not use a resource. The list will }
{  be created with default values, and will use the standard LDEF (0). You can change  }
{  the list by getting the list handle. You can set the LDEF to use by using the tag   }
{  below (kControlListBoxLDEFTag)                                                      }
{ List Box proc IDs }
const
	kControlListBoxProc = 352;
	kControlListBoxAutoSizeProc = 353;

{ Control Kind Tag }
const
	kControlKindListBox = FourCharCode('lbox');

{$ifc not TARGET_CPU_64}
{
 *  CreateListBoxControl()
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inWindow:
 *      The WindowRef into which the Disclosure Triangle will be
 *      created.
 *    
 *    inBoundsRect:
 *      The desired position (in coordinates local to the window's
 *      port) for the Disclosure Triangle.
 *    
 *    inOrientation:
 *      The direction the Disclosure Triangle should point when it is
 *      "closed". Passing kControlDisclosureTrianglePointDefault is
 *      only legal as of Mac OS X and CarbonLib 1.5.
 *    
 *    inTitle:
 *      The title for the Disclosure Triangle. The title will only be
 *      displayed if the inDrawTitle parameter is true. Title display
 *      only works on Mac OS X.
 *    
 *    inInitialValue:
 *      The starting value determines whether the Disclosure Triangle
 *      is initially in its "open" or "closed" state. The value 0
 *      represents the "closed" state and 1 represents the "open" state.
 *    
 *    inDrawTitle:
 *      A Boolean indicating whether the Disclosure Triangle should
 *      draw its title next to the widget. Title display only works on
 *      Mac OS X.
 *    
 *    inAutoToggles:
 *      A Boolean indicating whether the Disclosure Triangle should
 *      change its own value (from "open" to "closed" and vice-versa)
<<<<<<< HEAD
 *      automatically when it is clicked on.
 *    
 *    outControl:
 *      On successful output, outControl will contain a reference to
 *      the Disclosure Triangle control.
 *  
 *  Result:
 *    An OSStatus code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateDisclosureTriangleControl( inWindow: WindowRef; const (*var*) inBoundsRect: Rect; inOrientation: ControlDisclosureTriangleOrientation; inTitle: CFStringRef; inInitialValue: SInt32; inDrawTitle: Boolean; inAutoToggles: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateDisclosureTriangleControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by disclosure triangles }
const
	kControlTriangleLastValueTag = FourCharCode('last'); { SInt16}

{ Helper routines are available only thru the shared library/glue. }
{
 *  SetDisclosureTriangleLastValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetDisclosureTriangleLastValue( inTabControl: ControlRef; inValue: SInt16 ): OSErr; external name '_SetDisclosureTriangleLastValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PROGRESS INDICATOR (CDEF 5)                                                       }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This CDEF implements both determinate and indeterminate progress bars. To switch,   }
{  just use SetControlData to set the indeterminate flag to make it indeterminate. Any }
{  animation will automatically be handled by the toolbox on an event timer.           }
{  We also use this same CDEF for Relevance bars. At this time this control does not   }
{  idle.                                                                               }
{ Progress Bar proc IDs }
const
	kControlProgressBarProc = 80;
	kControlRelevanceBarProc = 81;

{ Control Kind Tag }
const
	kControlKindProgressBar = FourCharCode('prgb');
	kControlKindRelevanceBar = FourCharCode('relb');

{ The HIObject class ID for the HIProgressBar class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIProgressBarClassID CFSTRP('com.apple.HIProgressBar')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateProgressBarControl()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Parameters:
 *    
 *    window:
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    isPassword:
 *      A Boolean indicating whether the field is to be used as a
 *      password field. Passing false indicates that the field is to
 *      display entered text normally. True means that the field will
 *      be used as a password field and any text typed into the field
 *      will be displayed only as bullets.
 *    
 *    useInlineInput:
 *      A Boolean indicating whether or not the control is to accept
 *      inline input. Pass true to to accept inline input, otherwise
 *      pass false.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control (if noErr is returned as the
 *      result code).
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateEditTextControl( window: WindowRef; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; isPassword: Boolean; useInlineInput: Boolean; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateEditTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ ControlData tags supported only by the classic EditText control}
{$endc} {not TARGET_CPU_64}

>>>>>>> graemeg/cpstrnew
const
	kControlEditTextTEHandleTag = FourCharCode('than'); { The TEHandle of the text edit record}
	kControlEditTextInlinePreUpdateProcTag = FourCharCode('prup'); { TSMTEPreUpdateUPP and TSMTEPostUpdateUpp. For use with inline input variant...}
	kControlEditTextInlinePostUpdateProcTag = FourCharCode('poup'); { ...The refCon parameter will contain the ControlRef.}

{
    The classic EditText control also supports these tags defined for the EditUnicodeText control:
    
        kControlEditTextLockedTag
        kControlEditTextStyleTag
        kControlEditTextFixedTextTag
        kControlEditTextTextTag
        kControlEditTextKeyFilterTag
        kControlEditTextValidationProcTag
        kControlEditTextSelectionTag
        kControlEditTextKeyScriptBehaviorTag
        kControlEditTextCFStringTag
        kControlEditTextPasswordTag
        kControlEditTextPasswordCFStringTag
}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PICTURE CONTROL (CDEF 19)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Value parameter should contain the ID of the picture you wish to display when       }
{  creating controls of this type. If you don't want the control tracked at all, use   }
{  the 'no track' variant.                                                             }
{ Picture control proc IDs }
const
	kControlPictureProc = 304;
	kControlPictureNoTrackProc = 305;   { immediately returns kControlPicturePart}

{ Control Kind Tag }
const
	kControlKindPicture = FourCharCode('pict');

{ The HIObject class ID for the HIPictureView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIPictureViewClassID CFSTRP('com.apple.HIPictureView')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreatePictureControl()
 *  
 *  Summary:
 *    Creates a picture control.
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateListBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; autoSize: Boolean; numRows: SInt16; numColumns: SInt16; horizScroll: Boolean; vertScroll: Boolean; cellHeight: SInt16; cellWidth: SInt16; hasGrowSpace: Boolean; const (*var*) listDef: ListDefSpec; var outControl: ControlRef ): OSStatus; external name '_CreateListBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by list box }
{$endc} {not TARGET_CPU_64}

const
	kControlListBoxListHandleTag = FourCharCode('lhan'); { ListHandle}
	kControlListBoxKeyFilterTag = kControlKeyFilterTag; { ControlKeyFilterUPP}
	kControlListBoxFontStyleTag = kControlFontStyleTag; { ControlFontStyleRec}

{ New tags in 1.0.1 or later }
const
	kControlListBoxDoubleClickTag = FourCharCode('dblc'); { Boolean. Was last click a double-click?}
	kControlListBoxLDEFTag = FourCharCode('ldef'); { SInt16. ID of LDEF to use.}

{ Resource Types }
const
	kControlListDescResType = FourCharCode('ldes'); { used for list box control (Appearance 1.0 and later)}
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateScrollingTextBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; contentResID: SInt16; autoScroll: Boolean; delayBeforeAutoScroll: UInt32; delayBetweenAutoScroll: UInt32; autoScrollAmount: UInt16; var outControl: ControlRef ): OSStatus; external name '_CreateScrollingTextBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by Scroll Text Box }
{$endc} {not TARGET_CPU_64}

const
	kControlScrollTextBoxDelayBeforeAutoScrollTag = FourCharCode('stdl'); { UInt32 (ticks until autoscrolling starts)}
	kControlScrollTextBoxDelayBetweenAutoScrollTag = FourCharCode('scdl'); { UInt32 (ticks between scrolls)}
	kControlScrollTextBoxAutoScrollAmountTag = FourCharCode('samt'); { UInt16 (pixels per scroll) -- defaults to 1}
	kControlScrollTextBoxContentsTag = FourCharCode('tres'); { SInt16 (resource ID of 'TEXT'/'styl') -- write only!}
	kControlScrollTextBoxAnimatingTag = FourCharCode('anim'); { Boolean (whether the text box should auto-scroll)}
>>>>>>> origin/cpstrnew

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ SCROLL TEXT BOX (CDEF 27)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements a scrolling box of (non-editable) text. This is useful for  }
{  credits in about boxes, etc.                                                        }
{  The standard version of this control has a scroll bar, but the autoscrolling        }
{  variant does not. The autoscrolling variant needs two pieces of information to      }
{  work: delay (in ticks) before the scrolling starts, and time (in ticks) between     }
{  scrolls. It will scroll one pixel at a time, unless changed via SetControlData.     }
{  Parameter                   What Goes Here                                          }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ         ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ    }
{  Value                       Resource ID of 'TEXT'/'styl' content.                   }
{  Min                         Scroll start delay (in ticks)                       .   }
{  Max                         Delay (in ticks) between scrolls.                       }
{  NOTE: This control is only available with Appearance 1.1.                           }
{ Scroll Text Box Proc IDs }
const
	kControlScrollTextBoxProc = 432;
	kControlScrollTextBoxAutoScrollProc = 433;

{ Control Kind Tag }
const
	kControlKindScrollingTextBox = FourCharCode('stbx');

{$ifc not TARGET_CPU_64}
{
 *  CreateScrollingTextBoxControl()
>>>>>>> graemeg/cpstrnew
=======
=======
 *      automatically when it is clicked on.
 *    
 *    outControl:
 *      On successful output, outControl will contain a reference to
 *      the Disclosure Triangle control.
 *  
 *  Result:
 *    An OSStatus code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateDisclosureTriangleControl( inWindow: WindowRef; const (*var*) inBoundsRect: Rect; inOrientation: ControlDisclosureTriangleOrientation; inTitle: CFStringRef; inInitialValue: SInt32; inDrawTitle: Boolean; inAutoToggles: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateDisclosureTriangleControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by disclosure triangles }
const
	kControlTriangleLastValueTag = FourCharCode('last'); { SInt16}

{ Helper routines are available only thru the shared library/glue. }
{
 *  SetDisclosureTriangleLastValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetDisclosureTriangleLastValue( inTabControl: ControlRef; inValue: SInt16 ): OSErr; external name '_SetDisclosureTriangleLastValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PROGRESS INDICATOR (CDEF 5)                                                       }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This CDEF implements both determinate and indeterminate progress bars. To switch,   }
{  just use SetControlData to set the indeterminate flag to make it indeterminate. Any }
{  animation will automatically be handled by the toolbox on an event timer.           }
{  We also use this same CDEF for Relevance bars. At this time this control does not   }
{  idle.                                                                               }
{ Progress Bar proc IDs }
const
	kControlProgressBarProc = 80;
	kControlRelevanceBarProc = 81;

{ Control Kind Tag }
const
	kControlKindProgressBar = FourCharCode('prgb');
	kControlKindRelevanceBar = FourCharCode('relb');

{ The HIObject class ID for the HIProgressBar class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIProgressBarClassID CFSTRP('com.apple.HIProgressBar')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateProgressBarControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateProgressBarControl( window: WindowRef; const (*var*) boundsRect: Rect; value: SInt32; minimum: SInt32; maximum: SInt32; indeterminate: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateProgressBarControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ The HIObject class ID for the HIRelevanceBar class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIRelevanceBarClassID CFSTRP('com.apple.HIRelevanceBar')}
{$endc}
{
 *  CreateRelevanceBarControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateRelevanceBarControl( window: WindowRef; const (*var*) boundsRect: Rect; value: SInt32; minimum: SInt32; maximum: SInt32; var outControl: ControlRef ): OSStatus; external name '_CreateRelevanceBarControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by progress bars }
const
	kControlProgressBarIndeterminateTag = FourCharCode('inde'); { Boolean}
	kControlProgressBarAnimatingTag = FourCharCode('anim'); { Boolean}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ LITTLE ARROWS (CDEF 6)                                                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements the little up and down arrows you'd see in the Memory       }
{  control panel for adjusting the cache size.                                         }
{ Little Arrows proc IDs }
const
	kControlLittleArrowsProc = 96;

{ Control Kind Tag }
const
	kControlKindLittleArrows = FourCharCode('larr');


{
 *  Summary:
 *    Tags available with Mac OS X 10.3 or later
 }
const
{
   * Passed data is an SInt32.  Gets or sets the increment value of the
   * control.
   }
	kControlLittleArrowsIncrementValueTag = FourCharCode('incr');

{ The HIObject class ID for the HILittleArrows class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHILittleArrowsClassID CFSTRP('com.apple.HILittleArrows')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateLittleArrowsControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateLittleArrowsControl( window: WindowRef; const (*var*) boundsRect: Rect; value: SInt32; minimum: SInt32; maximum: SInt32; increment: SInt32; var outControl: ControlRef ): OSStatus; external name '_CreateLittleArrowsControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ CHASING ARROWS (CDEF 7)                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control will automatically animate via an event loop timer.                    }
{ Chasing Arrows proc IDs }
const
	kControlChasingArrowsProc = 112;

{ Control Kind Tag }
const
	kControlKindChasingArrows = FourCharCode('carr');

{ The HIObject class ID for the HIChasingArrows class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIChasingArrowsClassID CFSTRP('com.apple.HIChasingArrows')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateChasingArrowsControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateChasingArrowsControl( window: WindowRef; const (*var*) boundsRect: Rect; var outControl: ControlRef ): OSStatus; external name '_CreateChasingArrowsControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by the Chasing Arrows control }
const
	kControlChasingArrowsAnimatingTag = FourCharCode('anim'); { Boolean}


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ TABS (CDEF 8)                                                                     }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Tabs use an auxiliary resource (tab#) to hold tab information such as the tab name  }
{  and an icon suite ID for each tab.                                                  }
{  The ID of the tab# resource that you wish to associate with a tab control should    }
{  be passed in as the Value parameter of the control. If you are using GetNewControl, }
{  then the Value slot in the CNTL resource should have the ID of the 'tab#' resource  }
{  on creation.                                                                        }
{  Passing zero in for the tab# resource tells the control not to read in a tab# res.  }
{  You can then use SetControlMaximum to add tabs, followed by a call to SetControlData}
{  with the kControlTabInfoTag, passing in a pointer to a ControlTabInfoRec. This sets }
{  the name and optionally an icon for a tab. Pass the 1-based tab index as the part   }
{  code parameter to SetControlData to indicate the tab that you want to modify.       }
{  Accessibility Notes: Those of you who wish to customize the accessibility           }
{  information provided for individual tabs of a tabs control -- by handling various   }
{  kEventClassAccessibility Carbon Events, by calling                                  }
{  HIObjectSetAuxiliaryAccessibilityAttribute, etc. -- need to know how to interpret   }
{  and/or build AXUIElementRefs that represent individual tabs. The AXUIElement        }
{  representing an individual tab will/must be constructed using the tab control's     }
{  ControlRef and the UInt64 identifier of the one-based index of the tab the element  }
{  refers to. As usual, a UInt64 identifier of zero represents the tabs control as a   }
{  whole. You must neither interpret nor create tab control elements whose identifiers }
{  are greater than the count of tabs in the tabs control.                             }
{ Tabs proc IDs }
const
	kControlTabLargeProc = 128;  { Large tab size, north facing   }
	kControlTabSmallProc = 129;  { Small tab size, north facing   }
	kControlTabLargeNorthProc = 128;  { Large tab size, north facing   }
	kControlTabSmallNorthProc = 129;  { Small tab size, north facing   }
	kControlTabLargeSouthProc = 130;  { Large tab size, south facing   }
	kControlTabSmallSouthProc = 131;  { Small tab size, south facing   }
	kControlTabLargeEastProc = 132;  { Large tab size, east facing    }
	kControlTabSmallEastProc = 133;  { Small tab size, east facing    }
	kControlTabLargeWestProc = 134;  { Large tab size, west facing    }
	kControlTabSmallWestProc = 135;   { Small tab size, west facing    }

{ Tab Directions }
type
	ControlTabDirection = UInt16;
const
	kControlTabDirectionNorth = 0;
	kControlTabDirectionSouth = 1;
	kControlTabDirectionEast = 2;
	kControlTabDirectionWest = 3;

{ Tab Sizes }
type
	ControlTabSize = UInt16;
const
	kControlTabSizeLarge = kControlSizeNormal;
	kControlTabSizeSmall = kControlSizeSmall;
	kControlTabSizeMini = kControlSizeMini;

{ Control Tab Entry - used during creation                             }
{ Note that the client is responsible for allocating/providing         }
{ the ControlButtonContentInfo and string storage for this             }
{ structure.                                                           }
type
	ControlTabEntry = record
		icon: ControlButtonContentInfoPtr;
		name: CFStringRef;
		enabled: Boolean;
	end;
{ Control Kind Tag }
const
	kControlKindTabs = FourCharCode('tabs');

{ The HIObject class ID for the HITabbedView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHITabbedViewClassID CFSTRP('com.apple.HITabbedView')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateTabsControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateTabsControl( window: WindowRef; const (*var*) boundsRect: Rect; size: ControlTabSize; direction: ControlTabDirection; numTabs: UInt16; const (*var*) tabArray: ControlTabEntry; var outControl: ControlRef ): OSStatus; external name '_CreateTabsControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by tabs }
const
	kControlTabContentRectTag = FourCharCode('rect'); { Rect}
	kControlTabEnabledFlagTag = FourCharCode('enab'); { Boolean}
	kControlTabFontStyleTag = kControlFontStyleTag; { ControlFontStyleRec}

{ New tags in 1.0.1 or later }
const
	kControlTabInfoTag = FourCharCode('tabi'); { ControlTabInfoRec}

{ New tags in X 10.1 or later }
const
	kControlTabImageContentTag = FourCharCode('cont'); { ControlButtonContentInfo}

const
	kControlTabInfoVersionZero = 0;    { ControlTabInfoRec}
	kControlTabInfoVersionOne = 1;     { ControlTabInfoRecV1}

type
	ControlTabInfoRecPtr = ^ControlTabInfoRec;
	ControlTabInfoRec = record
		version: SInt16;                { version of this structure.}
		iconSuiteID: SInt16;            { icon suite to use. Zero indicates no icon}
		name: Str255;                   { name to be displayed on the tab}
	end;
type
	ControlTabInfoRecV1Ptr = ^ControlTabInfoRecV1;
	ControlTabInfoRecV1 = record
		version: SInt16;                { version of this structure. == kControlTabInfoVersionOne}
		iconSuiteID: SInt16;            { icon suite to use. Zero indicates no icon}
		name: CFStringRef;                   { name to be displayed on the tab. Will be retained so caller}
                                              { should always release it.}
	end;
{ Helper routines are available only thru the shared library/glue. }
{
 *  GetTabContentRect()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetTabContentRect( inTabControl: ControlRef; var outContentRect: Rect ): OSErr; external name '_GetTabContentRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetTabEnabled()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetTabEnabled( inTabControl: ControlRef; inTabToHilite: SInt16; inEnabled: Boolean ): OSErr; external name '_SetTabEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ VISUAL SEPARATOR (CDEF 9)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Separator lines determine their orientation (horizontal or vertical) automatically  }
{  based on the relative height and width of their contrlRect.                         }
{ Visual separator proc IDs }
const
	kControlSeparatorLineProc = 144;

{ Control Kind Tag }
const
	kControlKindSeparator = FourCharCode('sepa');

{ The HIObject class ID for the HIVisualSeparator class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIVisualSeparatorClassID CFSTRP('com.apple.HIVisualSeparator')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateSeparatorControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateSeparatorControl( window: WindowRef; const (*var*) boundsRect: Rect; var outControl: ControlRef ): OSStatus; external name '_CreateSeparatorControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePopupGroupBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef; primary: Boolean; menuID: SInt16; variableWidth: Boolean; titleWidth: SInt16; titleJustification: SInt16; titleStyle: Style; var outControl: ControlRef ): OSStatus; external name '_CreatePopupGroupBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by group box }
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
{  ¥ IMAGE WELL (CDEF 11)                                                              }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Image Wells allow you to control the content type (pict/icon/etc.) shown in the     }
{  well.                                                                               }
{  This is made possible by overloading the Min and Value parameters for the control.  }
{  Parameter                   What Goes Here                                          }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ         ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ      }
{  Min                         content type (see constants for bevel buttons)          }
{  Value                       Resource ID of content type, if resource-based.         }
{  Handle-based Content                                                                }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ                                                                }
{  You can create your control and then set the content to an existing handle to an    }
{  icon suite, etc. using the macros below. Please keep in mind that resource-based    }
{  content is owned by the control, handle-based content is owned by you. The CDEF will}
{  not try to dispose of handle-based content. If you are changing the content type of }
{  the button on the fly, you must make sure that if you are replacing a handle-       }
{  based content with a resource-based content to properly dispose of the handle,      }
{  else a memory leak will ensue.                                                      }
{ Image Well proc IDs }
const
	kControlImageWellProc = 176;

{ Control Kind Tag }
const
	kControlKindImageWell = FourCharCode('well');

{ The HIObject class ID for the HIImageWell class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIImageWellClassID CFSTRP('com.apple.HIImageWell')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateImageWellControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateImageWellControl( window: WindowRef; const (*var*) boundsRect: Rect; const (*var*) info: ControlButtonContentInfo; var outControl: ControlRef ): OSStatus; external name '_CreateImageWellControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by image wells }
const
	kControlImageWellContentTag = FourCharCode('cont'); { ButtonContentInfo}
	kControlImageWellTransformTag = FourCharCode('tran'); { IconTransformType}
	kControlImageWellIsDragDestinationTag = FourCharCode('drag'); { Boolean}

{ Helper routines are available only thru the shared library/glue. }
{
 *  GetImageWellContentInfo()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetImageWellContentInfo( inButton: ControlRef; outContent: ControlButtonContentInfoPtr ): OSErr; external name '_GetImageWellContentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetImageWellContentInfo()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetImageWellContentInfo( inButton: ControlRef; inContent: ControlButtonContentInfoPtr ): OSErr; external name '_SetImageWellContentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetImageWellTransform()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetImageWellTransform( inButton: ControlRef; inTransform: IconTransformType ): OSErr; external name '_SetImageWellTransform';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ POPUP ARROW (CDEF 12)                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  The popup arrow CDEF is used to draw the small arrow normally associated with a     }
{  popup control. The arrow can point in four directions, and a small or large version }
{  can be used. This control is provided to allow clients to draw the arrow in a       }
{  normalized fashion which will take advantage of themes automatically.               }
{ Popup Arrow proc IDs }
const
	kControlPopupArrowEastProc = 192;
	kControlPopupArrowWestProc = 193;
	kControlPopupArrowNorthProc = 194;
	kControlPopupArrowSouthProc = 195;
	kControlPopupArrowSmallEastProc = 196;
	kControlPopupArrowSmallWestProc = 197;
	kControlPopupArrowSmallNorthProc = 198;
	kControlPopupArrowSmallSouthProc = 199;

{ Popup Arrow Orientations }
const
	kControlPopupArrowOrientationEast = 0;
	kControlPopupArrowOrientationWest = 1;
	kControlPopupArrowOrientationNorth = 2;
	kControlPopupArrowOrientationSouth = 3;


type
	ControlPopupArrowOrientation = UInt16;
{ Popup Arrow Size }
const
	kControlPopupArrowSizeNormal = 0;
	kControlPopupArrowSizeSmall = 1;

type
	ControlPopupArrowSize = UInt16;
{ Control Kind Tag }
const
	kControlKindPopupArrow = FourCharCode('parr');

{
 *  CreatePopupArrowControl()
 *  
 *  Summary:
 *    Creates a popup arrow control.
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
 *    orientation:
 *      The orientation of the control.
 *    
 *    size:
 *      The size of the control.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePopupArrowControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; orientation: ControlPopupArrowOrientation; size: ControlPopupArrowSize; var outControl: ControlRef ): OSStatus; external name '_CreatePopupArrowControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePlacardControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; var outControl: ControlRef ): OSStatus; external name '_CreatePlacardControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ CLOCK (CDEF 15)                                                                   }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  NOTE:   You can specify more options in the Value paramter when creating the clock. }
{          See below.                                                                  }
{  NOTE:   Under Appearance 1.1, the clock control knows and returns more part codes.  }
{          The new clock-specific part codes are defined with the other control parts. }
{          Besides these clock-specific parts, we also return kControlUpButtonPart     }
{          and kControlDownButtonPart when they hit the up and down arrows.            }
{          The new part codes give you more flexibility for focusing and hit testing.  }
{          The original kControlClockPart is still valid. When hit testing, it means   }
{          that some non-editable area of the clock's whitespace has been clicked.     }
{          When focusing a currently unfocused clock, it changes the focus to the      }
{          first part; it is the same as passing kControlFocusNextPart. When           }
{          re-focusing a focused clock, it will not change the focus at all.           }
{ Clock proc IDs }
const
	kControlClockTimeProc = 240;
	kControlClockTimeSecondsProc = 241;
	kControlClockDateProc = 242;
	kControlClockMonthYearProc = 243;

{ Clock Types }
type
	ControlClockType = UInt16;
const
	kControlClockTypeHourMinute = 0;
	kControlClockTypeHourMinuteSecond = 1;
	kControlClockTypeMonthDayYear = 2;
	kControlClockTypeMonthYear = 3;

{ Clock Flags }
{  These flags can be passed into 'value' field on creation of the control.            }
{  Value is set to 0 after control is created.                                         }
type
	ControlClockFlags = UInt32;
const
	kControlClockFlagStandard = 0;    { editable, non-live}
	kControlClockNoFlags = 0;
	kControlClockFlagDisplayOnly = 1;    { add this to become non-editable}
	kControlClockIsDisplayOnly = 1;
	kControlClockFlagLive = 2;    { automatically shows current time on idle. only valid with display only.}
	kControlClockIsLive = 2;

{ Control Kind Tag }
const
	kControlKindClock = FourCharCode('clck');

{ The HIObject class ID for the HIClock class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIClockViewClassID CFSTRP('com.apple.HIClock')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateClockControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateClockControl( window: WindowRef; const (*var*) boundsRect: Rect; clockType: ControlClockType; clockFlags: ControlClockFlags; var outControl: ControlRef ): OSStatus; external name '_CreateClockControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by clocks }
const
	kControlClockLongDateTag = FourCharCode('date'); { LongDateRec}
	kControlClockFontStyleTag = kControlFontStyleTag; { ControlFontStyleRec}
	kControlClockAnimatingTag = FourCharCode('anim'); { Boolean}

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
const
	kControlUserPaneProc = 256;

{ Control Kind Tag }
const
	kControlKindUserPane = FourCharCode('upan');

{ The HIObject class ID for the HIUserPane class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIUserPaneClassID CFSTRP('com.apple.HIUserPane')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateUserPaneControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateUserPaneControl( window: WindowRef; const (*var*) boundsRect: Rect; features: UInt32; var outControl: ControlRef ): OSStatus; external name '_CreateUserPaneControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by user panes }
{ Currently, they are all proc ptrs for doing things like drawing and hit testing, etc. }
const
	kControlUserItemDrawProcTag = FourCharCode('uidp'); { UserItemUPP}
	kControlUserPaneDrawProcTag = FourCharCode('draw'); { ControlUserPaneDrawUPP}
	kControlUserPaneHitTestProcTag = FourCharCode('hitt'); { ControlUserPaneHitTestUPP}
	kControlUserPaneTrackingProcTag = FourCharCode('trak'); { ControlUserPaneTrackingUPP}
	kControlUserPaneIdleProcTag = FourCharCode('idle'); { ControlUserPaneIdleUPP}
	kControlUserPaneKeyDownProcTag = FourCharCode('keyd'); { ControlUserPaneKeyDownUPP}
	kControlUserPaneActivateProcTag = FourCharCode('acti'); { ControlUserPaneActivateUPP}
	kControlUserPaneFocusProcTag = FourCharCode('foci'); { ControlUserPaneFocusUPP}
	kControlUserPaneBackgroundProcTag = FourCharCode('back'); { ControlUserPaneBackgroundUPP}

type
	ControlUserPaneDrawProcPtr = procedure( control: ControlRef; part: SInt16 );
type
	ControlUserPaneHitTestProcPtr = function( control: ControlRef; where: Point ): ControlPartCode;
type
	ControlUserPaneTrackingProcPtr = function( control: ControlRef; startPt: Point; actionProc: ControlActionUPP ): ControlPartCode;
type
	ControlUserPaneIdleProcPtr = procedure( control: ControlRef );
type
	ControlUserPaneKeyDownProcPtr = function( control: ControlRef; keyCode: SInt16; charCode: SInt16; modifiers: SInt16 ): ControlPartCode;
type
	ControlUserPaneActivateProcPtr = procedure( control: ControlRef; activating: Boolean );
type
	ControlUserPaneFocusProcPtr = function( control: ControlRef; action: ControlFocusPart ): ControlPartCode;
type
	ControlUserPaneBackgroundProcPtr = procedure( control: ControlRef; info: ControlBackgroundPtr );
type
	ControlUserPaneDrawUPP = ControlUserPaneDrawProcPtr;
type
	ControlUserPaneHitTestUPP = ControlUserPaneHitTestProcPtr;
type
	ControlUserPaneTrackingUPP = ControlUserPaneTrackingProcPtr;
type
	ControlUserPaneIdleUPP = ControlUserPaneIdleProcPtr;
type
	ControlUserPaneKeyDownUPP = ControlUserPaneKeyDownProcPtr;
type
	ControlUserPaneActivateUPP = ControlUserPaneActivateProcPtr;
type
	ControlUserPaneFocusUPP = ControlUserPaneFocusProcPtr;
type
	ControlUserPaneBackgroundUPP = ControlUserPaneBackgroundProcPtr;
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
 *  InvokeControlUserPaneDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeControlUserPaneDrawUPP( control: ControlRef; part: SInt16; userUPP: ControlUserPaneDrawUPP ); external name '_InvokeControlUserPaneDrawUPP';
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

{
  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ
    ¥ EDIT TEXT (CDEF 17)
  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ
}
{ Edit Text proc IDs }
const
	kControlEditTextProc = 272;
	kControlEditTextPasswordProc = 274;

{ proc IDs available with Appearance 1.1 or later }
const
	kControlEditTextInlineInputProc = 276; { Can't combine with the other variants}

{ Control Kind Tag }
const
	kControlKindEditText = FourCharCode('etxt');

{
 *  CreateEditTextControl()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateEditUnicodeTextControl API instead.
 *  
 *  Summary:
 *    Creates a new edit text control.
 *  
 *  Discussion:
 *    This control is a legacy control. It is deprecated in favor of
 *    the EditUnicodeText control, which handles Unicode and draws its
 *    text using antialiasing.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    isPassword:
 *      A Boolean indicating whether the field is to be used as a
 *      password field. Passing false indicates that the field is to
 *      display entered text normally. True means that the field will
 *      be used as a password field and any text typed into the field
 *      will be displayed only as bullets.
 *    
 *    useInlineInput:
 *      A Boolean indicating whether or not the control is to accept
 *      inline input. Pass true to to accept inline input, otherwise
 *      pass false.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control (if noErr is returned as the
 *      result code).
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateEditTextControl( window: WindowRef; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; isPassword: Boolean; useInlineInput: Boolean; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateEditTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Tagged data supported by edit text }
const
	kControlEditTextStyleTag = kControlFontStyleTag; { ControlFontStyleRec}
	kControlEditTextTextTag = FourCharCode('text'); { Buffer of chars - you supply the buffer}
	kControlEditTextTEHandleTag = FourCharCode('than'); { The TEHandle of the text edit record}
	kControlEditTextKeyFilterTag = kControlKeyFilterTag;
	kControlEditTextSelectionTag = FourCharCode('sele'); { ControlEditTextSelectionRec}
	kControlEditTextPasswordTag = FourCharCode('pass'); { The clear text password text}
	kControlEditTextCharCount = FourCharCode('chrc'); { Count of characters in the control's text}

{ tags available with Appearance 1.1 or later }
const
	kControlEditTextKeyScriptBehaviorTag = FourCharCode('kscr'); { ControlKeyScriptBehavior. Defaults to "PrefersRoman" for password fields,}
                                        {       or "AllowAnyScript" for non-password fields.}
	kControlEditTextLockedTag = FourCharCode('lock'); { Boolean. Locking disables editability.}
	kControlEditTextFixedTextTag = FourCharCode('ftxt'); { Like the normal text tag, but fixes inline input first}
	kControlEditTextValidationProcTag = FourCharCode('vali'); { ControlEditTextValidationUPP. Called when a key filter can't be: after cut, paste, etc.}
	kControlEditTextInlinePreUpdateProcTag = FourCharCode('prup'); { TSMTEPreUpdateUPP and TSMTEPostUpdateUpp. For use with inline input variant...}
	kControlEditTextInlinePostUpdateProcTag = FourCharCode('poup'); { ...The refCon parameter will contain the ControlRef.}


{
 *  Discussion:
 *    EditText ControlData tags available with MacOSX and later.
 }
const
{
   * Extract the content of the edit text field as a CFString.  Don't
   * forget that you own the returned CFStringRef and are responsible
   * for CFReleasing it.
   }
	kControlEditTextCFStringTag = FourCharCode('cfst'); { CFStringRef (Also available on CarbonLib 1.5)}

  {
   * Extract the content of the edit text field as a CFString, if it is
   * a password field.  Don't forget that you own the returned
   * CFStringRef and are responsible for CFReleasing it.
   }
	kControlEditTextPasswordCFStringTag = FourCharCode('pwcf'); { CFStringRef}


{ Structure for getting the edit text selection }
type
	ControlEditTextSelectionRecPtr = ^ControlEditTextSelectionRec;
	ControlEditTextSelectionRec = record
		selStart: SInt16;
		selEnd: SInt16;
	end;
type
	ControlEditTextSelectionPtr = ControlEditTextSelectionRecPtr;
type
	ControlEditTextValidationProcPtr = procedure( control: ControlRef );
type
	ControlEditTextValidationUPP = ControlEditTextValidationProcPtr;
{
 *  NewControlEditTextValidationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlEditTextValidationUPP( userRoutine: ControlEditTextValidationProcPtr ): ControlEditTextValidationUPP; external name '_NewControlEditTextValidationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlEditTextValidationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlEditTextValidationUPP( userUPP: ControlEditTextValidationUPP ); external name '_DisposeControlEditTextValidationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlEditTextValidationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeControlEditTextValidationUPP( control: ControlRef; userUPP: ControlEditTextValidationUPP ); external name '_InvokeControlEditTextValidationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ STATIC TEXT (CDEF 18)                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Static Text proc IDs }
const
	kControlStaticTextProc = 288;

{ Control Kind Tag }
const
	kControlKindStaticText = FourCharCode('stxt');

{ The HIObject class ID for the HIStaticTextView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIStaticTextViewClassID CFSTRP('com.apple.HIStaticTextView')}
{$endc}
{ Creation API: Carbon only }
{
 *  CreateStaticTextControl()
 *  
 *  Summary:
 *    Creates a new static text control.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateStaticTextControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateStaticTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  Summary:
 *    Tagged data supported by the static text control
 }
const
{
   * Used to get or set the control's current text style. Data is of
   * type ControlFontStyleRec. Available with Appearance Manager 1.0
   * (Mac OS 8.0) and later.
   }
	kControlStaticTextStyleTag = kControlFontStyleTag;

  {
   * Used to get or set the control's current text. Data is an array of
   * chars. Generally you should used GetControlDataSize to determine
   * the length of the text, and allocate a buffer of that length,
   * before calling GetControlData with this selector. Deprecated in
   * Carbon in favor of kControlStaticTextCFStringTag. Available with
   * Appearance Manager 1.0 (Mac OS 8.0) and later.
   }
	kControlStaticTextTextTag = FourCharCode('text');

  {
   * Used to get the height of the control's text. May not be used with
   * SetControlData. Data is of type SInt16. Available with Appearance
   * Manager 1.0 (Mac OS 8.0) and later.
   }
	kControlStaticTextTextHeightTag = FourCharCode('thei');

  {
   * Used to get or set the control's text truncation style. Data is of
   * type TruncCode; pass a truncation code of -1 to indication no
   * truncation. Available with Appearance Manager 1.1 (Mac OS 8.5) and
   * later. Truncation will not occur unless
   * kControlStaticTextIsMultilineTag is set to false.
   }
	kControlStaticTextTruncTag = FourCharCode('trun');

  {
   * Used to get or set the control's current text. Data is of type
   * CFStringRef. When setting the text, the control will retain the
   * string, so you may release the string after calling
   * SetControlData; if the string is mutable, the control will make a
   * copy of the string, so any changes to the string after calling
   * SetControlData will not affect the control. When getting the text,
   * the control retains the string before returning it to you, so you
   * must release the string after you are done with it. Available in
   * CarbonLib 1.5 and Mac OS X 10.0 and later.
   }
	kControlStaticTextCFStringTag = FourCharCode('cfst');

  {
   * Used to get or set whether the control draws its text in multiple
   * lines if the text is too wide for the control bounds. If false,
   * then the control always draws the text in a single line. Data is
   * of type Boolean. Available in Mac OS X 10.1 and later.
   }
	kControlStaticTextIsMultilineTag = FourCharCode('stim');


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PICTURE CONTROL (CDEF 19)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Value parameter should contain the ID of the picture you wish to display when       }
{  creating controls of this type. If you don't want the control tracked at all, use   }
{  the 'no track' variant.                                                             }
{ Picture control proc IDs }
const
	kControlPictureProc = 304;
	kControlPictureNoTrackProc = 305;   { immediately returns kControlPicturePart}

{ Control Kind Tag }
const
	kControlKindPicture = FourCharCode('pict');

{
 *  CreatePictureControl()
 *  
 *  Summary:
 *    Creates a picture control.
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
 *    content:
 *      The descriptor for the picture you want the control to display.
 *    
 *    dontTrack:
 *      A Boolean value indicating whether the control should hilite
 *      when it is clicked on. False means hilite and track the mouse.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePictureControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; const (*var*) content: ControlButtonContentInfo; dontTrack: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreatePictureControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by picture controls }
const
	kControlPictureHandleTag = FourCharCode('pich'); { PicHandle}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ ICON CONTROL (CDEF 20)                                                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Value parameter should contain the ID of the ICON or cicn you wish to display when  }
{  creating controls of this type. If you don't want the control tracked at all, use   }
{  the 'no track' variant.                                                             }
{ Icon control proc IDs }
const
	kControlIconProc = 320;
	kControlIconNoTrackProc = 321;  { immediately returns kControlIconPart}
	kControlIconSuiteProc = 322;
	kControlIconSuiteNoTrackProc = 323;   { immediately returns kControlIconPart}

const
{ icon ref controls may have either an icon, color icon, icon suite, or icon ref.}
                                        { for data other than icon, you must set the data by passing a}
                                        { ControlButtonContentInfo to SetControlData}
	kControlIconRefProc = 324;
	kControlIconRefNoTrackProc = 325;   { immediately returns kControlIconPart}

{ Control Kind Tag }
const
	kControlKindIcon = FourCharCode('icon');

{ The HIObject class ID for the HIIconView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIIconViewClassID CFSTRP('com.apple.HIIconView')}
{$endc}
{
 *  CreateIconControl()
 *  
 *  Summary:
 *    Creates an Icon control at a specific position in the specified
 *    window.
 *  
 *  Discussion:
 *    Icon controls display an icon that (optionally) hilites when
 *    clicked on. On Mac OS X, a root control will be created for the
 *    window if one does not already exist. If a root control exists
 *    for the window, the Icon control will be embedded into it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inWindow:
 *      The WindowRef into which the Icon control will be created. May
 *      be NULL on 10.3 and later.
 *    
 *    inBoundsRect:
 *      The desired position (in coordinates local to the window's
 *      port) for the Icon control.
 *    
 *    inIconContent:
 *      The descriptor for the icon you want the control to display.
 *      Mac OS X and CarbonLib 1.5 (and beyond) support all of the icon
 *      content types. Prior to CarbonLib 1.5, the only content types
 *      that are properly respected are kControlContentIconSuiteRes,
 *      kControlContentCIconRes, and kControlContentICONRes.
 *    
 *    inDontTrack:
 *      A Boolean value indicating whether the control should hilite
 *      when it is clicked on. False means hilite and track the mouse.
 *    
 *    outControl:
 *      On successful output, outControl will contain a reference to
 *      the Icon control.
 *  
 *  Result:
 *    An OSStatus code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateIconControl( inWindow: WindowRef { can be NULL }; const (*var*) inBoundsRect: Rect; const (*var*) inIconContent: ControlButtonContentInfo; inDontTrack: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateIconControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by icon controls }
const
	kControlIconTransformTag = FourCharCode('trfm'); { IconTransformType}
	kControlIconAlignmentTag = FourCharCode('algn'); { IconAlignmentType}

{ Tags available with appearance 1.1 or later }
const
	kControlIconResourceIDTag = FourCharCode('ires'); { SInt16 resource ID of icon to use}
	kControlIconContentTag = FourCharCode('cont'); { accepts a ControlButtonContentInfo}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ WINDOW HEADER (CDEF 21)                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Window Header proc IDs }
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
{
 *  CreateWindowHeaderControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateWindowHeaderControl( window: WindowRef; const (*var*) boundsRect: Rect; isListHeader: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateWindowHeaderControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ LIST BOX (CDEF 22)                                                                }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Lists use an auxiliary resource to define their format. The resource type used is   }
{  'ldes' and a definition for it can be found in Appearance.r. The resource ID for    }
{  the ldes is passed in the 'value' parameter when creating the control. You may pass }
{  zero in value. This tells the List Box control to not use a resource. The list will }
{  be created with default values, and will use the standard LDEF (0). You can change  }
{  the list by getting the list handle. You can set the LDEF to use by using the tag   }
{  below (kControlListBoxLDEFTag)                                                      }
{ List Box proc IDs }
const
	kControlListBoxProc = 352;
	kControlListBoxAutoSizeProc = 353;

{ Control Kind Tag }
const
	kControlKindListBox = FourCharCode('lbox');

{ Creation API: Carbon Only }
{
 *  CreateListBoxControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateListBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; autoSize: Boolean; numRows: SInt16; numColumns: SInt16; horizScroll: Boolean; vertScroll: Boolean; cellHeight: SInt16; cellWidth: SInt16; hasGrowSpace: Boolean; const (*var*) listDef: ListDefSpec; var outControl: ControlRef ): OSStatus; external name '_CreateListBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by list box }
const
	kControlListBoxListHandleTag = FourCharCode('lhan'); { ListHandle}
	kControlListBoxKeyFilterTag = kControlKeyFilterTag; { ControlKeyFilterUPP}
	kControlListBoxFontStyleTag = kControlFontStyleTag; { ControlFontStyleRec}

{ New tags in 1.0.1 or later }
const
	kControlListBoxDoubleClickTag = FourCharCode('dblc'); { Boolean. Was last click a double-click?}
	kControlListBoxLDEFTag = FourCharCode('ldef'); { SInt16. ID of LDEF to use.}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ PUSH BUTTON (CDEF 23)                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  The new standard checkbox and radio button controls support a "mixed" value that    }
{  indicates that the current setting contains a mixed set of on and off values. The   }
{  control value used to display this indication is defined in Controls.h:             }
{      kControlCheckBoxMixedValue = 2                                                  }
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
{ Theme Push Button/Check Box/Radio Button proc IDs }
const
	kControlPushButtonProc = 368;
	kControlCheckBoxProc = 369;
	kControlRadioButtonProc = 370;
	kControlPushButLeftIconProc = 374;  { Standard pushbutton with left-side icon}
	kControlPushButRightIconProc = 375;   { Standard pushbutton with right-side icon}

{ Variants with Appearance 1.1 or later }
const
	kControlCheckBoxAutoToggleProc = 371;
	kControlRadioButtonAutoToggleProc = 372;

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
	kControlKindRadioButton = FourCharCode('rdio');
	kControlKindCheckBox = FourCharCode('cbox');

{ The HIObject class ID for the HIPushButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIPushButtonClassID CFSTRP('com.apple.HIPushButton')}
{$endc}
{ The HIObject class ID for the HICheckBox class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHICheckBoxClassID CFSTRP('com.apple.HICheckBox')}
{$endc}
{ The HIObject class ID for the HIRadioButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIRadioButtonClassID CFSTRP('com.apple.HIRadioButton')}
{$endc}
{ Creation APIs: Carbon Only }
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePushButtonWithIconControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef { can be NULL }; var icon: ControlButtonContentInfo; iconAlignment: ControlPushButtonIconAlignment; var outControl: ControlRef ): OSStatus; external name '_CreatePushButtonWithIconControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateRadioButtonControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef { can be NULL }; initialValue: SInt32; autoToggle: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateRadioButtonControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateCheckBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; title: CFStringRef { can be NULL }; initialValue: SInt32; autoToggle: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateCheckBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by standard buttons }
const
	kControlPushButtonDefaultTag = FourCharCode('dflt'); { default ring flag}
	kControlPushButtonCancelTag = FourCharCode('cncl'); { cancel button flag (1.1 and later)}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ SCROLL BAR (CDEF 24)                                                              }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This is the new Appearance scroll bar.                                              }
{  Mac OS X has a "Scroll to here" option in the General pane of System Preferences    }
{  which allows users to click in the page up/down regions of a scroll bar and have    }
{  the thumb/indicator jump directly to the clicked position, which alters the value   }
{  of the scroll bar and moves the scrolled content appropriately. As long as the      }
{  mouse button is held down, the click is treated as though the user had clicked in   }
{  the thumb/indicator in the first place.                                             }
{  If you want the scroll bars in your application to work with the "Scroll to here"   }
{  option, you must do the following:                                                  }
{  1. Create live-tracking scroll bars, not scroll bars that show a "ghost" thumb      }
{  when you click on it. You can request live-tracking scroll bars by passing true     }
{  in the liveTracking parameter to CreateScrollBarControl. If you create scroll bars  }
{  with NewControl, use the kControlScrollBarLiveProc.                                 }
{  2. Write an appropriate ControlActionProc and associate it with your scroll bar     }
{  via the SetControlAction API. This allows your application to update its content    }
{  appropriately when the live-tracking scroll bar is clicked.                         }
{  3. When calling HandleControlClick or TrackControl, pass -1 in the action proc      }
{  parameter. This is a request for the Control Manager to use the action proc you     }
{  associated with your control in step 2. If you rely on the standard window event    }
{  handler to do your control tracking, this step is handled for you automatically.    }
{ Theme Scroll Bar proc IDs }
const
	kControlScrollBarProc = 384;  { normal scroll bar}
	kControlScrollBarLiveProc = 386;   { live scrolling variant}

{ Control Kind Tag }
const
	kControlKindScrollBar = FourCharCode('sbar');

{ The HIObject class ID for the HIScrollBar class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIScrollBarClassID CFSTRP('com.apple.HIScrollBar')}
{$endc}
{
 *  CreateScrollBarControl()
 *  
 *  Summary:
 *    Creates a scroll bar control.
 *  
 *  Discussion:
 *    This creation API is available in Carbon only.
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
 *    value:
 *      The initial value of the control.
 *    
 *    minimum:
 *      The minimum value of the control.
 *    
 *    maximum:
 *      The maximum value of the control.
 *    
 *    viewSize:
 *      The size of the visible area of the scroll bar content.
 *    
 *    liveTracking:
 *      A Boolean indicating whether or not live tracking is enabled
 *      for this scroll bar. If set to true and a valid
 *      liveTrackingProc is also passed in, the callback will be called
 *      repeatedly as the thumb is moved during tracking.  If set to
 *      false, a semi-transparent thumb called a "ghost thumb" will
 *      draw and no live tracking will occur.
 *    
 *    liveTrackingProc:
 *      If liveTracking is on, a ControlActionUPP callback to be called
 *      as the control live tracks.  This callback is called repeatedly
 *      as the scroll thumb is moved during tracking.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateScrollBarControl( window: WindowRef; const (*var*) boundsRect: Rect; value: SInt32; minimum: SInt32; maximum: SInt32; viewSize: SInt32; liveTracking: Boolean; liveTrackingProc: ControlActionUPP; var outControl: ControlRef ): OSStatus; external name '_CreateScrollBarControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ These tags are available in Mac OS X or later }
const
	kControlScrollBarShowsArrowsTag = FourCharCode('arro'); { Boolean whether or not to draw the scroll arrows}


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ POPUP BUTTON (CDEF 25)                                                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This is the new Appearance Popup Button. It takes the same variants and does the    }
{  same overloading as the previous popup menu control. There are some differences:    }
{  Passing in a menu ID of -12345 causes the popup not to try and get the menu from a  }
{  resource. Instead, you can build the menu and later stuff the MenuRef field in      }
{  the popup data information.                                                         }
{  You can pass -1 in the Max parameter to have the control calculate the width of the }
{  title on its own instead of guessing and then tweaking to get it right. It adds the }
{  appropriate amount of space between the title and the popup.                        }
{ Theme Popup Button proc IDs }
const
	kControlPopupButtonProc = 400;
	kControlPopupFixedWidthVariant = 1 shl 0;
	kControlPopupVariableWidthVariant = 1 shl 1;
	kControlPopupUseAddResMenuVariant = 1 shl 2;
	kControlPopupUseWFontVariant = kControlUsesOwningWindowsFontVariant;

{ Control Kind Tag }
const
	kControlKindPopupButton = FourCharCode('popb');

{ The HIObject class ID for the HIPopupButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIPopupButtonClassID CFSTRP('com.apple.HIPopupButton')}
{$endc}
{
 *  CreatePopupButtonControl()
 *  
 *  Summary:
 *    Creates a popup button control.
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
 *    title:
 *      The title of the control.
 *    
 *    menuID:
 *      The ID of a menu that should be used by the control. A menu
 *      with this ID should be inserted into the menubar with
 *      InsertMenu(menu, kInsertHierarchicalMenu). You can also pass
 *      -12345 to have the control delay its acquisition of a menu; in
 *      this case, you can build the menu and later provide it to the
 *      control with SetControlData and kControlPopupButtonMenuRefTag
 *      or kControlPopupButtonOwnedMenuRefTag.
 *    
 *    variableWidth:
 *      Whether the width of the control is allowed to vary according
 *      to the width of the selected menu item text, or should remain
 *      fixed to the original control bounds width.
 *    
 *    titleWidth:
 *      The width of the title.
 *    
 *    titleJustification:
 *      The justification of the title.
 *    
 *    titleStyle:
 *      A QuickDraw style bitfield indicating the font style of the
 *      title.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreatePopupButtonControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; title: CFStringRef; menuID: SInt16; variableWidth: Boolean; titleWidth: SInt16; titleJustification: SInt16; titleStyle: Style; var outControl: ControlRef ): OSStatus; external name '_CreatePopupButtonControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ These tags are available in 1.0.1 or later of Appearance }
const
	kControlPopupButtonMenuHandleTag = FourCharCode('mhan'); { MenuRef}
	kControlPopupButtonMenuRefTag = FourCharCode('mhan'); { MenuRef}
	kControlPopupButtonMenuIDTag = FourCharCode('mnid'); { SInt16}

{ These tags are available in 1.1 or later of Appearance }
const
	kControlPopupButtonExtraHeightTag = FourCharCode('exht'); { SInt16 - extra vertical whitespace within the button}
	kControlPopupButtonOwnedMenuRefTag = FourCharCode('omrf'); { MenuRef}

{ These tags are available in Mac OS X }
const
	kControlPopupButtonCheckCurrentTag = FourCharCode('chck'); { Boolean    - whether the popup puts a checkmark next to the current item (defaults to true)}


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ RADIO GROUP (CDEF 26)                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements a radio group. It is an embedding control and can therefore }
{  only be used when a control hierarchy is established for its owning window. You     }
{  should only embed radio buttons within it. As radio buttons are embedded into it,   }
{  the group sets up its value, min, and max to represent the number of embedded items.}
{  The current value of the control is the index of the sub-control that is the current}
{  'on' radio button. To get the current radio button control handle, you can use the  }
{  control manager call GetIndSubControl, passing in the value of the radio group.     }
{  Note that when creating radio buttons for use in a radio group control, you should  }
{  not use the autoToggle version of the radio button. The radio group control will    }
{  handling toggling the radio button values itself; auto-toggle radio buttons do not  }
{  work properly in a radio group control on Mac OS 9.                                 }
{  NOTE: This control is only available with Appearance 1.0.1.                         }
{ Radio Group Proc ID }
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
{
 *  CreateRadioGroupControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateRadioGroupControl( window: WindowRef; const (*var*) boundsRect: Rect; var outControl: ControlRef ): OSStatus; external name '_CreateRadioGroupControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ SCROLL TEXT BOX (CDEF 27)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements a scrolling box of (non-editable) text. This is useful for  }
{  credits in about boxes, etc.                                                        }
{  The standard version of this control has a scroll bar, but the autoscrolling        }
{  variant does not. The autoscrolling variant needs two pieces of information to      }
{  work: delay (in ticks) before the scrolling starts, and time (in ticks) between     }
{  scrolls. It will scroll one pixel at a time, unless changed via SetControlData.     }
{  Parameter                   What Goes Here                                          }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ         ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ    }
{  Value                       Resource ID of 'TEXT'/'styl' content.                   }
{  Min                         Scroll start delay (in ticks)                       .   }
{  Max                         Delay (in ticks) between scrolls.                       }
{  NOTE: This control is only available with Appearance 1.1.                           }
{ Scroll Text Box Proc IDs }
const
	kControlScrollTextBoxProc = 432;
	kControlScrollTextBoxAutoScrollProc = 433;

{ Control Kind Tag }
const
	kControlKindScrollingTextBox = FourCharCode('stbx');

{ Creation API: Carbon Only }
{
 *  CreateScrollingTextBoxControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateScrollingTextBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; contentResID: SInt16; autoScroll: Boolean; delayBeforeAutoScroll: UInt32; delayBetweenAutoScroll: UInt32; autoScrollAmount: UInt16; var outControl: ControlRef ): OSStatus; external name '_CreateScrollingTextBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by Scroll Text Box }
const
	kControlScrollTextBoxDelayBeforeAutoScrollTag = FourCharCode('stdl'); { UInt32 (ticks until autoscrolling starts)}
	kControlScrollTextBoxDelayBetweenAutoScrollTag = FourCharCode('scdl'); { UInt32 (ticks between scrolls)}
	kControlScrollTextBoxAutoScrollAmountTag = FourCharCode('samt'); { UInt16 (pixels per scroll) -- defaults to 1}
	kControlScrollTextBoxContentsTag = FourCharCode('tres'); { SInt16 (resource ID of 'TEXT'/'styl') -- write only!}
	kControlScrollTextBoxAnimatingTag = FourCharCode('anim'); { Boolean (whether the text box should auto-scroll)}


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ DISCLOSURE BUTTON                                                                 }
{  (CDEF 30)                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ The HIObject class ID for the HIDisclosureButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIDisclosureButtonClassID CFSTRP('com.apple.HIDisclosureButton')}
{$endc}
{
 *  CreateDisclosureButtonControl()
 *  
 *  Summary:
 *    Creates a new instance of the Disclosure Button Control.
 *  
 *  Discussion:
 *    CreateDisclosureButtonControl is preferred over NewControl
 *    because it allows you to specify the exact set of parameters
 *    required to create the control without overloading parameter
 *    semantics. The initial minimum of the Disclosure Button will be
 *    kControlDisclosureButtonClosed, and the maximum will be
 *    kControlDisclosureButtonDisclosed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inWindow:
 *      The WindowRef in which to create the control.
 *    
 *    inBoundsRect:
 *      The bounding rectangle for the control. The height of the
 *      control is fixed and the control will be centered vertically
 *      within the rectangle you specify.
 *    
 *    inValue:
 *      The initial value; either kControlDisclosureButtonClosed or
 *      kControlDisclosureButtonDisclosed.
 *    
 *    inAutoToggles:
 *      A boolean value indicating whether its value should change
 *      automatically after tracking the mouse.
 *    
 *    outControl:
 *      On successful exit, this will contain the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateDisclosureButtonControl( inWindow: WindowRef; const (*var*) inBoundsRect: Rect; inValue: SInt32; inAutoToggles: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreateDisclosureButtonControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Control Kind Tag }
const
	kControlKindDisclosureButton = FourCharCode('disb');


{
 *  Discussion:
 *    Disclosure Button Values
 }
const
{
   * The control be drawn suggesting a closed state.
   }
	kControlDisclosureButtonClosed = 0;

  {
   * The control will be drawn suggesting an open state.
   }
	kControlDisclosureButtonDisclosed = 1;


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ ROUND BUTTON                                                                      }
{  (CDEF 31)                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

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
	kControlRoundButtonContentTag = FourCharCode('cont'); { ControlButtonContentInfo}
	kControlRoundButtonSizeTag = kControlSizeTag; { ControlRoundButtonSize}

{ Control Kind Tag }
const
	kControlKindRoundButton = FourCharCode('rndb');

{ The HIObject class ID for the HIRoundButton class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIRoundButtonClassID CFSTRP('com.apple.HIRoundButton')}
{$endc}
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateRoundButtonControl( inWindow: WindowRef { can be NULL }; const (*var*) inBoundsRect: Rect; inSize: ControlRoundButtonSize; inContent: ControlButtonContentInfoPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateRoundButtonControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{    ¥ DATA BROWSER                                                                    }
{     (CDEF 29)                                                                        }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements a user interface component for browsing (optionally)        }
{  hiearchical data structures. The browser supports multiple presentation styles      }
{  including, but not limited to:                                                      }
{      kDataBrowserListView   - items and item properties in                           }
{                               multi-column (optionally outline) format               }
{      kDataBrowserColumnView - in-place browsing using fixed navigation columns       }
{  The browser manages all view styles through a single high-level interface.          }
{  The high-level interface makes the following assumptions:                           }
{      - Items have unique 32-bit identifiers (0 is reserved)                          }
{      - Items have two kinds of named and typed properties:                           }
{           - Predefined attribute properties ( < 1024 )                               }
{             (including some display properties)                                      }
{           - Client-defined display properties ( >= 1024 )                            }
{      - Some items are containers of other items                                      }
{      - Items may be sorted by any property                                           }
{  Because a browser doesn't know all details about the type of objects it manages,    }
{  some implementation responsibility is best handled by its client. The client must   }
{  provide a set of callback routines which define the item hierarchy and help to      }
{  populate the browser with items. The client may also provide callbacks for handling }
{  custom data types and doing low-level event management.                             }
{  The API is subdivided into a "universal" set of routines that applies to all view   }
{  styles, and a set of routines unique to each view style. kDataBrowserListView and   }
{  kDataBrowserColumnView share an (internal) TableView abstract base class. The       }
{  TableView formatting options and API applies to both of these view styles.          }
{  NOTE: This control is only available with CarbonLib 1.1.                            }
{  NOTE: This control must be created with the CreateDataBrowserControl API in         }
{        CarbonLib 1.1 through 1.4. In Mac OS X and CarbonLib 1.5 and later, you       }
{        may use the control's procID (29) to create the control with NewControl       }
{        or with a 'CNTL' resource.                                                    }
{ The HIObject class ID for the HIDataBrowser class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIDataBrowserClassID CFSTRP('com.apple.HIDataBrowser')}
{$endc}
{ Control Kind Tag }
const
	kControlKindDataBrowser = FourCharCode('datb');

{ Error Codes }
const
	errDataBrowserNotConfigured = -4970;
	errDataBrowserItemNotFound = -4971;
	errDataBrowserItemNotAdded = -4975;
	errDataBrowserPropertyNotFound = -4972;
	errDataBrowserInvalidPropertyPart = -4973;
	errDataBrowserInvalidPropertyData = -4974;
	errDataBrowserPropertyNotSupported = -4979; { Return from DataBrowserGetSetItemDataProc }

const
{ Generic Control Tags }
	kControlDataBrowserIncludesFrameAndFocusTag = FourCharCode('brdr'); { Boolean }
	kControlDataBrowserKeyFilterTag = kControlEditTextKeyFilterTag;
	kControlDataBrowserEditTextKeyFilterTag = kControlDataBrowserKeyFilterTag;
	kControlDataBrowserEditTextValidationProcTag = kControlEditTextValidationProcTag;

{ Data Browser View Styles }
type
	DataBrowserViewStyle = OSType;
const
	kDataBrowserNoView = FourCharCode('????'); {  Error State  }
	kDataBrowserListView = FourCharCode('lstv');
	kDataBrowserColumnView = FourCharCode('clmv');

{ Selection Flags }
type
	DataBrowserSelectionFlags = UInt32;
const
	kDataBrowserDragSelect = 1 shl 0; { Å ListMgr lNoRect }
	kDataBrowserSelectOnlyOne = 1 shl 1; { Å ListMgr lOnlyOne }
	kDataBrowserResetSelection = 1 shl 2; { Å ListMgr lNoExtend }
	kDataBrowserCmdTogglesSelection = 1 shl 3; { Å ListMgr lUseSense }
	kDataBrowserNoDisjointSelection = 1 shl 4; { Å ListMgr lNoDisjoint }
	kDataBrowserAlwaysExtendSelection = 1 shl 5; { Å ListMgr lExtendDrag }
	kDataBrowserNeverEmptySelectionSet = 1 shl 6; { Å ListMgr lNoNilHilite }

{ Data Browser Sorting }
type
	DataBrowserSortOrder = UInt16;
const
	kDataBrowserOrderUndefined = 0;    { Not currently supported }
	kDataBrowserOrderIncreasing = 1;
	kDataBrowserOrderDecreasing = 2;

{ Data Browser Item Management }
type
	DataBrowserItemID = UInt32;
	DataBrowserItemIDPtr = ^DataBrowserItemID;
const
	kDataBrowserNoItem = 0;    { Reserved DataBrowserItemID }

type
	DataBrowserItemState = UInt32;
const
	kDataBrowserItemNoState = 0;
	kDataBrowserItemAnyState = $FFFFFFFF;
	kDataBrowserItemIsSelected = 1 shl 0;
	kDataBrowserContainerIsOpen = 1 shl 1;
	kDataBrowserItemIsDragTarget = 1 shl 2; { During a drag operation }

{ Options for use with RevealDataBrowserItem }
type
	DataBrowserRevealOptions = UInt8;
const
	kDataBrowserRevealOnly = 0;
	kDataBrowserRevealAndCenterInView = 1 shl 0;
	kDataBrowserRevealWithoutSelecting = 1 shl 1;

{ Set operations for use with SetDataBrowserSelectedItems }
type
	DataBrowserSetOption = UInt32;
const
	kDataBrowserItemsAdd = 0;    { add specified items to existing set }
	kDataBrowserItemsAssign = 1;    { assign destination set to specified items }
	kDataBrowserItemsToggle = 2;    { toggle membership state of specified items }
	kDataBrowserItemsRemove = 3;     { remove specified items from existing set }

{ Commands for use with MoveDataBrowserSelectionAnchor }
type
	DataBrowserSelectionAnchorDirection = UInt32;
const
	kDataBrowserSelectionAnchorUp = 0;
	kDataBrowserSelectionAnchorDown = 1;
	kDataBrowserSelectionAnchorLeft = 2;
	kDataBrowserSelectionAnchorRight = 3;

{ Edit menu command IDs for use with Enable/ExecuteDataBrowserEditCommand }
type
	DataBrowserEditCommand = UInt32;
const
	kDataBrowserEditMsgUndo = kHICommandUndo;
	kDataBrowserEditMsgRedo = kHICommandRedo;
	kDataBrowserEditMsgCut = kHICommandCut;
	kDataBrowserEditMsgCopy = kHICommandCopy;
	kDataBrowserEditMsgPaste = kHICommandPaste;
	kDataBrowserEditMsgClear = kHICommandClear;
	kDataBrowserEditMsgSelectAll = kHICommandSelectAll;

{ Notifications used in DataBrowserItemNotificationProcPtr }
type
	DataBrowserItemNotification = UInt32;
const
	kDataBrowserItemAdded = 1;    { The specified item has been added to the browser }
	kDataBrowserItemRemoved = 2;    { The specified item has been removed from the browser }
	kDataBrowserEditStarted = 3;    { Starting an EditText session for specified item }
	kDataBrowserEditStopped = 4;    { Stopping an EditText session for specified item }
	kDataBrowserItemSelected = 5;    { Item has just been added to the selection set }
	kDataBrowserItemDeselected = 6;    { Item has just been removed from the selection set }
	kDataBrowserItemDoubleClicked = 7;
	kDataBrowserContainerOpened = 8;    { Container is open }
	kDataBrowserContainerClosing = 9;    { Container is about to close (and will real soon now, y'all) }
	kDataBrowserContainerClosed = 10;   { Container is closed (y'all come back now!) }
	kDataBrowserContainerSorting = 11;   { Container is about to be sorted (lock any volatile properties) }
	kDataBrowserContainerSorted = 12;   { Container has been sorted (you may release any property locks) }
	kDataBrowserUserToggledContainer = 16; { _User_ requested container open/close state to be toggled }
	kDataBrowserTargetChanged = 15;   { The target has changed to the specified item }
	kDataBrowserUserStateChanged = 13;   { The user has reformatted the view for the target }
	kDataBrowserSelectionSetChanged = 14;  { The selection set has been modified (net result may be the same) }


{
 *  DataBrowserPropertyID
 *  
 *  Discussion:
 *    Properties with values 0 through 1023 are reserved for Apple's
 *    use. Values greater than or equal to 1024 are for client use.
 }
type
	DataBrowserPropertyID = UInt32;
const
{ Predefined attribute properties, optional & non-display unless otherwise stated }
	kDataBrowserItemNoProperty = 0;   { The anti-property (no associated data) }
	kDataBrowserItemIsActiveProperty = 1; { Boolean typed data (defaults to true) }
	kDataBrowserItemIsSelectableProperty = 2; { Boolean typed data (defaults to true) }
	kDataBrowserItemIsEditableProperty = 3; { Boolean typed data (defaults to false, used for editable properties) }
	kDataBrowserItemIsContainerProperty = 4; { Boolean typed data (defaults to false) }
	kDataBrowserContainerIsOpenableProperty = 5; { Boolean typed data (defaults to true) }
	kDataBrowserContainerIsClosableProperty = 6; { Boolean typed data (defaults to true) }
	kDataBrowserContainerIsSortableProperty = 7; { Boolean typed data (defaults to true) }
	kDataBrowserItemSelfIdentityProperty = 8; { kDataBrowserIconAndTextType (display property; ColumnView only) }

  {
   * kDataBrowserContainerAliasIDProperty is sent to your
   * DataBrowserItemDataProcPtr callback to give you a chance to follow
   * an alias or symlink that the item might represent. If the incoming
   * item is an alias to another item, you can call
   * SetDataBrowserItemDataItemID to let Data Browser know which other
   * item the incoming item points to. 
   * 
   * This is only sent from column view, and your support for it is
   * optional. It allows Data Browser to be more memory efficient with
   * its internal storage. If a given container item is an alias to an
   * item whose contents are already displayed in an existing column
   * view column, the contents can be shared between those two columns.
   }
	kDataBrowserContainerAliasIDProperty = 9; { DataBrowserItemID (alias/symlink an item to a container item) }

  {
   * kDataBrowserColumnViewPreviewProperty is sent to various callbacks
   * to give you a chance to draw or track in the preview column of
   * column view. 
   * 
   * You can also pass kDataBrowserColumnViewPreviewProperty in the
   * property parameter of RevealDataBrowserItem in conjunction with
   * the appropriate DataBrowserItemID of the item whose preview is
   * being displayed when you want to make sure the preview column is
   * visible to the user. 
   * 
   * kDataBrowserColumnViewPreviewProperty is only supported in column
   * view.
   }
	kDataBrowserColumnViewPreviewProperty = 10; { kDataBrowserCustomType (display property; ColumnView only) }

  {
   * kDataBrowserItemParentContainerProperty is sent to your
   * DataBrowserItemDataProcPtr callback when Data Browser needs to
   * know the parent container item for a given item. 
   * 
   * In column view, this allows the internals of SetDataBrowserTarget
   * to work. The target is the container whose contents you wish to
   * display, which is the rightmost column in column view. However,
   * unlike SetDataBrowserColumnViewPath, SetDataBrowserTarget doesn't
   * offer a way for you to communicate the DataBrowserItemIDs of the
   * rest of the column containers, so SetDataBrowserTarget needs to
   * ask for them explicitly by asking for the container's parent, then
   * the container's parent's parent, and so on. 
   * 
   * In list view, this allows you to pass a non-container to
   * SetDataBrowserTarget. In this situation, Data Browser will ask you
   * for the parent of the target so it knows which container to
   * display the contents of in the list view. 
   * 
   * In both list and column views, your DataBrowserItemDataProcPtr
   * callback might be called with
   * kDataBrowserItemParentContainerProperty at a variety of other
   * times, so you should be sure to support this property if your Data
   * Browser displays a containment hierarchy.
   }
	kDataBrowserItemParentContainerProperty = 11; { DataBrowserItemID (the parent of the specified item, used by ColumnView) }

{ DataBrowser Property Types (for display properties; i.e. ListView columns) }
{      These are primarily presentation types (or styles) although         }
{      they also imply a particular set of primitive types or structures.  }
type
	DataBrowserPropertyType = OSType;
const
{ == Corresponding data type or structure == }
	kDataBrowserCustomType = $3F3F3F3F; { No associated data, custom callbacks used }
	kDataBrowserIconType = FourCharCode('icnr'); { IconRef, IconTransformType, RGBColor }
	kDataBrowserTextType = FourCharCode('text'); { CFStringRef }
	kDataBrowserDateTimeType = FourCharCode('date'); { DateTime or LongDateTime }
	kDataBrowserSliderType = FourCharCode('sldr'); { Min, Max, Value }
	kDataBrowserCheckboxType = FourCharCode('chbx'); { ThemeButtonValue }
	kDataBrowserProgressBarType = FourCharCode('prog'); { Min, Max, Value }
	kDataBrowserRelevanceRankType = FourCharCode('rank'); { Min, Max, Value }
	kDataBrowserPopupMenuType = FourCharCode('menu'); { MenuRef, Value }
	kDataBrowserIconAndTextType = FourCharCode('ticn'); { IconRef, CFStringRef, etc }

{ DataBrowser Property Parts }
{      Visual components of a property type.      }
{      For use with GetDataBrowserItemPartBounds. }
type
	DataBrowserPropertyPart = OSType;
const
	kDataBrowserPropertyEnclosingPart = 0;
	kDataBrowserPropertyContentPart = FourCharCode('----');
	kDataBrowserPropertyDisclosurePart = FourCharCode('disc');
	kDataBrowserPropertyTextPart = kDataBrowserTextType;
	kDataBrowserPropertyIconPart = kDataBrowserIconType;
	kDataBrowserPropertySliderPart = kDataBrowserSliderType;
	kDataBrowserPropertyCheckboxPart = kDataBrowserCheckboxType;
	kDataBrowserPropertyProgressBarPart = kDataBrowserProgressBarType;
	kDataBrowserPropertyRelevanceRankPart = kDataBrowserRelevanceRankType;

{ Modify appearance/behavior of display properties }
type
	DataBrowserPropertyFlags = UInt32;
{ Low 8 bits apply to all property types }
const
	kDataBrowserUniversalPropertyFlagsMask = $FF;
	kDataBrowserPropertyIsMutable = 1 shl 0;
	kDataBrowserDefaultPropertyFlags = 0 shl 0;
	kDataBrowserUniversalPropertyFlags = kDataBrowserUniversalPropertyFlagsMask; { support for an old name}
	kDataBrowserPropertyIsEditable = kDataBrowserPropertyIsMutable; { support for an old name}

{ Next 8 bits contain property-specific modifiers }

{
 *  Summary:
 *    Data Browser Property Flags
 }
const
	kDataBrowserPropertyFlagsOffset = 8;
	kDataBrowserPropertyFlagsMask = $FF shl kDataBrowserPropertyFlagsOffset;
	kDataBrowserCheckboxTriState = 1 shl kDataBrowserPropertyFlagsOffset; { kDataBrowserCheckboxType}
	kDataBrowserDateTimeRelative = 1 shl (kDataBrowserPropertyFlagsOffset); { kDataBrowserDateTimeType }
	kDataBrowserDateTimeDateOnly = 1 shl (kDataBrowserPropertyFlagsOffset + 1); { kDataBrowserDateTimeType }
	kDataBrowserDateTimeTimeOnly = 1 shl (kDataBrowserPropertyFlagsOffset + 2); { kDataBrowserDateTimeType }
	kDataBrowserDateTimeSecondsToo = 1 shl (kDataBrowserPropertyFlagsOffset + 3); { kDataBrowserDateTimeType }
	kDataBrowserSliderPlainThumb = kThemeThumbPlain shl kDataBrowserPropertyFlagsOffset; { kDataBrowserSliderType }
	kDataBrowserSliderUpwardThumb = kThemeThumbUpward shl kDataBrowserPropertyFlagsOffset; { kDataBrowserSliderType }
	kDataBrowserSliderDownwardThumb = kThemeThumbDownward shl kDataBrowserPropertyFlagsOffset; { kDataBrowserSliderType }
	kDataBrowserDoNotTruncateText = 3 shl kDataBrowserPropertyFlagsOffset; { kDataBrowserTextType && kDataBrowserIconAndTextType }
	kDataBrowserTruncateTextAtEnd = 2 shl kDataBrowserPropertyFlagsOffset; { kDataBrowserTextType && kDataBrowserIconAndTextType }
	kDataBrowserTruncateTextMiddle = 0 shl kDataBrowserPropertyFlagsOffset; { kDataBrowserTextType && kDataBrowserIconAndTextType }
	kDataBrowserTruncateTextAtStart = 1 shl kDataBrowserPropertyFlagsOffset; { kDataBrowserTextType && kDataBrowserIconAndTextType }

  {
   * This flag is only for use with columns of type
   * kDataBrowserPopupMenuType. This flag indicates that the popup be
   * drawn in a sleek buttonless fashion. The text will be drawn next
   * to a popup glyph, and the whole cell will be clickable. Available
   * on 10.4 and later.
   }
	kDataBrowserPopupMenuButtonless = 1 shl kDataBrowserPropertyFlagsOffset; { kDataBrowserPopupMenuType}
	kDataBrowserPropertyModificationFlags = kDataBrowserPropertyFlagsMask; { support for an old name}
	kDataBrowserRelativeDateTime = kDataBrowserDateTimeRelative; { support for an old name}

{
   Next 8 bits contain viewStyle-specific modifiers 
   See individual ViewStyle sections below for flag definitions 
}
const
	kDataBrowserViewSpecificFlagsOffset = 16;
	kDataBrowserViewSpecificFlagsMask = $FF shl kDataBrowserViewSpecificFlagsOffset;
	kDataBrowserViewSpecificPropertyFlags = kDataBrowserViewSpecificFlagsMask; { support for an old name}

{ High 8 bits are reserved for client application use }
const
	kDataBrowserClientPropertyFlagsOffset = 24;
	kDataBrowserClientPropertyFlagsMask = $FF000000;

{ Client defined property description }
type
	DataBrowserPropertyDescPtr = ^DataBrowserPropertyDesc;
	DataBrowserPropertyDesc = record
		propertyID: DataBrowserPropertyID;
		propertyType: DataBrowserPropertyType;
		propertyFlags: DataBrowserPropertyFlags;
	end;
{ Callback definition for use with ForEachDataBrowserItem }
type
	DataBrowserItemProcPtr = procedure( item: DataBrowserItemID; state: DataBrowserItemState; clientData: UnivPtr );
type
	DataBrowserItemUPP = DataBrowserItemProcPtr;
{
 *  NewDataBrowserItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemUPP( userRoutine: DataBrowserItemProcPtr ): DataBrowserItemUPP; external name '_NewDataBrowserItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemUPP( userUPP: DataBrowserItemUPP ); external name '_DisposeDataBrowserItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserItemUPP( item: DataBrowserItemID; state: DataBrowserItemState; clientData: UnivPtr; userUPP: DataBrowserItemUPP ); external name '_InvokeDataBrowserItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{ Creation/Configuration }
{
 *  CreateDataBrowserControl()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateDataBrowserControl( window: WindowRef; const (*var*) boundsRect: Rect; style: DataBrowserViewStyle; var outControl: ControlRef ): OSStatus; external name '_CreateDataBrowserControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserViewStyle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserViewStyle( browser: ControlRef; var style: DataBrowserViewStyle ): OSStatus; external name '_GetDataBrowserViewStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserViewStyle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserViewStyle( browser: ControlRef; style: DataBrowserViewStyle ): OSStatus; external name '_SetDataBrowserViewStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  Summary:
 *    Data Browser attributes
 *  
 *  Discussion:
 *    For use with DataBrowserChangeAttributes and
 *    DataBrowserGetAttributes. Available in Mac OS X 10.4 and later.
 }
const
{
   * A constant with value zero; the lack of any attributes.
   }
	kDataBrowserAttributeNone = 0;

  {
   * In Column View, this Data Browser is allowed to resize the owning
   * window whenever necessary. This includes, but is not necessarily
   * limited to, situations where column resize operations need more
   * visible space in the window. If you turn this attribute on, your
   * window must tolerate being resized behind your app's back. If your
   * window needs to react to bounds changes, use a
   * kEventWindowBoundsChanged event handler. If you need to constrain
   * your window's minimum and maximum bounds, use
   * kEventWindowGetMinimum/MaximumSize handlers, the
   * SetWindowResizeLimits API, or something similar.
   }
	kDataBrowserAttributeColumnViewResizeWindow = 1 shl 0;

  {
   * In List View, this Data Browser should draw alternating row
   * background colors.
   }
	kDataBrowserAttributeListViewAlternatingRowColors = 1 shl 1;

  {
   * In List View, this Data Browser should draw a vertical line
   * between the columns.
   }
	kDataBrowserAttributeListViewDrawColumnDividers = 1 shl 2;

{
 *  DataBrowserChangeAttributes()
 *  
 *  Summary:
 *    Set the attributes for the given Data Browser.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDataBrowser:
 *      The Data Browser whose attributes to change.
 *    
 *    inAttributesToSet:
 *      The attributes to set.
 *    
 *    inAttributesToClear:
 *      The attributes to clear.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function DataBrowserChangeAttributes( inDataBrowser: ControlRef; inAttributesToSet: OptionBits; inAttributesToClear: OptionBits ): OSStatus; external name '_DataBrowserChangeAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  DataBrowserGetAttributes()
 *  
 *  Summary:
 *    Returns the attributes of a given Data Browser.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDataBrowser:
 *      The Data Browser whose attributes to query.
 *    
 *    outAttributes:
 *      On exit, will contain the attributes of the Data Browser. This
 *      parameter cannot be NULL.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function DataBrowserGetAttributes( inDataBrowser: ControlRef; var outAttributes: OptionBits ): OSStatus; external name '_DataBrowserGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  Summary:
 *    DataBrowserMetric values
 *  
 *  Discussion:
 *    For use with DataBrowserSetMetric.
 }
const
{
   * The content (icon, text, etc.) within a cell is drawn a certain
   * amount in from the left & right edges of the cell. This metric
   * governs the amount of inset.
   }
	kDataBrowserMetricCellContentInset = 1;

  {
   * This metric controls the space between the icon and text within a
   * column of type kDataBrowserIconAndTextType.
   }
	kDataBrowserMetricIconAndTextGap = 2;

  {
   * In List View only, this metric is similar to
   * kDataBrowserMetricCellContentInset, but it only affects the
   * disclosure column and it only affects the side of the cell that
   * displays the disclosure triangle. In other words, this metric is
   * used instead of (not in addition to)
   * DataBrowserMetricCellContentInset for one side of the disclosure
   * column.
   }
	kDataBrowserMetricDisclosureColumnEdgeInset = 3;

  {
   * In List View only, this metric controls the amount of space
   * between the disclosure triangle and the cell's content.
   }
	kDataBrowserMetricDisclosureTriangleAndContentGap = 4;

  {
   * In List View only, this metric controls the amount of space in the
   * disclosure column for each level of indentation in progressively
   * deeper hierarchies of disclosed items.
   }
	kDataBrowserMetricDisclosureColumnPerDepthGap = 5;
	kDataBrowserMetricLast = kDataBrowserMetricDisclosureColumnPerDepthGap;


type
	DataBrowserMetric = UInt32;
{
 *  DataBrowserSetMetric()
 *  
 *  Summary:
 *    Sets a value for a specified Data Browser metric.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDataBrowser:
 *      The Data Browser instance whose metric is being changed.
 *    
 *    inMetric:
 *      The DataBrowserMetric whose value is being changed.
 *    
 *    inUseDefaultValue:
 *      A Boolean indicating whether you want the Data Browser instance
 *      to revert to the default value for the metric. If you pass
 *      true, inValue will be ignored and a suitable default value will
 *      be used. If you pass false, inValue will be used as the value
 *      of the metric.
 *    
 *    inValue:
 *      When you pass false for inUseDefaultValue, this parameter is
 *      the value to use for the metric.
 *  
 *  Result:
 *    An operating system status code. If the incoming ControlRef isn't
 *    a Data Browser instance, or if the incoming metric isn't known,
 *    this function will return paramErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function DataBrowserSetMetric( inDataBrowser: ControlRef; inMetric: DataBrowserMetric; inUseDefaultValue: Boolean; inValue: Float32 ): OSStatus; external name '_DataBrowserSetMetric';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  DataBrowserGetMetric()
 *  
 *  Summary:
 *    Gets the value for a specified Data Browser metric.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDataBrowser:
 *      The Data Browser instance whose metric value to get.
 *    
 *    inMetric:
 *      The DataBrowserMetric value to get.
 *    
 *    outUsingDefaultValue:
 *      On exit, this is a Boolean indicating whether the metric's
 *      value is determined by Data Browser's default values. You may
 *      pass NULL if you don't need this information.
 *    
 *    outValue:
 *      On exit, this is the value of the metric.
 *  
 *  Result:
 *    An operating system status code. If the incoming ControlRef isn't
 *    a Data Browser instance, or if the incoming metric isn't known,
 *    this function will return paramErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function DataBrowserGetMetric( inDataBrowser: ControlRef; inMetric: DataBrowserMetric; outUsingDefaultValue: BooleanPtr { can be NULL }; var outValue: Float32 ): OSStatus; external name '_DataBrowserGetMetric';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ Item Manipulation }
{ Passing NULL for "items" argument to RemoveDataBrowserItems and }
{ UpdateDataBrowserItems refers to all items in the specified container. }
{ Passing NULL for "items" argument to AddDataBrowserItems means }
{ "generate IDs starting from 1." }
{
 *  AddDataBrowserItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AddDataBrowserItems( browser: ControlRef; container: DataBrowserItemID; numItems: UInt32; {const} items: DataBrowserItemIDPtr { can be NULL }; preSortProperty: DataBrowserPropertyID ): OSStatus; external name '_AddDataBrowserItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveDataBrowserItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveDataBrowserItems( browser: ControlRef; container: DataBrowserItemID; numItems: UInt32; {const} items: DataBrowserItemIDPtr { can be NULL }; preSortProperty: DataBrowserPropertyID ): OSStatus; external name '_RemoveDataBrowserItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateDataBrowserItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateDataBrowserItems( browser: ControlRef; container: DataBrowserItemID; numItems: UInt32; {const} items: DataBrowserItemIDPtr { can be NULL }; preSortProperty: DataBrowserPropertyID; propertyID: DataBrowserPropertyID ): OSStatus; external name '_UpdateDataBrowserItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Edit Menu Enabling and Handling }
{
 *  EnableDataBrowserEditCommand()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function EnableDataBrowserEditCommand( browser: ControlRef; command: DataBrowserEditCommand ): Boolean; external name '_EnableDataBrowserEditCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ExecuteDataBrowserEditCommand()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function ExecuteDataBrowserEditCommand( browser: ControlRef; command: DataBrowserEditCommand ): OSStatus; external name '_ExecuteDataBrowserEditCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserSelectionAnchor()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserSelectionAnchor( browser: ControlRef; var first: DataBrowserItemID; var last: DataBrowserItemID ): OSStatus; external name '_GetDataBrowserSelectionAnchor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MoveDataBrowserSelectionAnchor()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function MoveDataBrowserSelectionAnchor( browser: ControlRef; direction: DataBrowserSelectionAnchorDirection; extendSelection: Boolean ): OSStatus; external name '_MoveDataBrowserSelectionAnchor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Container Manipulation }
{
 *  OpenDataBrowserContainer()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function OpenDataBrowserContainer( browser: ControlRef; container: DataBrowserItemID ): OSStatus; external name '_OpenDataBrowserContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CloseDataBrowserContainer()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CloseDataBrowserContainer( browser: ControlRef; container: DataBrowserItemID ): OSStatus; external name '_CloseDataBrowserContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SortDataBrowserContainer()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SortDataBrowserContainer( browser: ControlRef; container: DataBrowserItemID; sortChildren: Boolean ): OSStatus; external name '_SortDataBrowserContainer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Aggregate Item Access and Iteration }
{
 *  GetDataBrowserItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItems( browser: ControlRef; container: DataBrowserItemID; recurse: Boolean; state: DataBrowserItemState; items: Handle ): OSStatus; external name '_GetDataBrowserItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemCount()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemCount( browser: ControlRef; container: DataBrowserItemID; recurse: Boolean; state: DataBrowserItemState; var numItems: UInt32 ): OSStatus; external name '_GetDataBrowserItemCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ForEachDataBrowserItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function ForEachDataBrowserItem( browser: ControlRef; container: DataBrowserItemID; recurse: Boolean; state: DataBrowserItemState; callback: DataBrowserItemUPP; clientData: UnivPtr ): OSStatus; external name '_ForEachDataBrowserItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Individual Item Access and Display }
{
 *  IsDataBrowserItemSelected()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsDataBrowserItemSelected( browser: ControlRef; item: DataBrowserItemID ): Boolean; external name '_IsDataBrowserItemSelected';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemState()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemState( browser: ControlRef; item: DataBrowserItemID; var state: DataBrowserItemState ): OSStatus; external name '_GetDataBrowserItemState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RevealDataBrowserItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RevealDataBrowserItem( browser: ControlRef; item: DataBrowserItemID; propertyID: DataBrowserPropertyID; options: DataBrowserRevealOptions ): OSStatus; external name '_RevealDataBrowserItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Selection Set Manipulation }
{
 *  SetDataBrowserSelectedItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserSelectedItems( browser: ControlRef; numItems: UInt32; items: DataBrowserItemIDPtr; operation: DataBrowserSetOption ): OSStatus; external name '_SetDataBrowserSelectedItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ DataBrowser Attribute Manipulation }
{ The user customizable portion of the current view style settings }
{
 *  SetDataBrowserUserState()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserUserState( browser: ControlRef; stateInfo: CFDictionaryRef ): OSStatus; external name '_SetDataBrowserUserState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserUserState()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserUserState( browser: ControlRef; var stateInfo: CFDictionaryRef ): OSStatus; external name '_GetDataBrowserUserState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ All items are active/enabled or not }
{
 *  SetDataBrowserActiveItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserActiveItems( browser: ControlRef; active: Boolean ): OSStatus; external name '_SetDataBrowserActiveItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserActiveItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserActiveItems( browser: ControlRef; var active: Boolean ): OSStatus; external name '_GetDataBrowserActiveItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Inset the scrollbars within the DataBrowser bounds }
{
 *  SetDataBrowserScrollBarInset()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserScrollBarInset( browser: ControlRef; var insetRect: Rect ): OSStatus; external name '_SetDataBrowserScrollBarInset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserScrollBarInset()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserScrollBarInset( browser: ControlRef; var insetRect: Rect ): OSStatus; external name '_GetDataBrowserScrollBarInset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ The "user focused" item }
{ For the ListView, this means the root container }
{ For the ColumnView, this means the rightmost container column }
{
 *  SetDataBrowserTarget()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTarget( browser: ControlRef; target: DataBrowserItemID ): OSStatus; external name '_SetDataBrowserTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTarget()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTarget( browser: ControlRef; var target: DataBrowserItemID ): OSStatus; external name '_GetDataBrowserTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Current sort ordering }
{ ListView tracks this per-column }
{
 *  SetDataBrowserSortOrder()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserSortOrder( browser: ControlRef; order: DataBrowserSortOrder ): OSStatus; external name '_SetDataBrowserSortOrder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserSortOrder()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserSortOrder( browser: ControlRef; var order: DataBrowserSortOrder ): OSStatus; external name '_GetDataBrowserSortOrder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Scrollbar values }
{
 *  SetDataBrowserScrollPosition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserScrollPosition( browser: ControlRef; top: UInt32; left: UInt32 ): OSStatus; external name '_SetDataBrowserScrollPosition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserScrollPosition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserScrollPosition( browser: ControlRef; var top: UInt32; var left: UInt32 ): OSStatus; external name '_GetDataBrowserScrollPosition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Show/Hide each scrollbar }
{
 *  SetDataBrowserHasScrollBars()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserHasScrollBars( browser: ControlRef; horiz: Boolean; vert: Boolean ): OSStatus; external name '_SetDataBrowserHasScrollBars';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserHasScrollBars()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserHasScrollBars( browser: ControlRef; var horiz: Boolean; var vert: Boolean ): OSStatus; external name '_GetDataBrowserHasScrollBars';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Property passed to sort callback (ListView sort column) }
{
 *  SetDataBrowserSortProperty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserSortProperty( browser: ControlRef; property: DataBrowserPropertyID ): OSStatus; external name '_SetDataBrowserSortProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserSortProperty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserSortProperty( browser: ControlRef; var property: DataBrowserPropertyID ): OSStatus; external name '_GetDataBrowserSortProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Modify selection behavior }
{
 *  SetDataBrowserSelectionFlags()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserSelectionFlags( browser: ControlRef; selectionFlags: DataBrowserSelectionFlags ): OSStatus; external name '_SetDataBrowserSelectionFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserSelectionFlags()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserSelectionFlags( browser: ControlRef; var selectionFlags: DataBrowserSelectionFlags ): OSStatus; external name '_GetDataBrowserSelectionFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Dynamically modify property appearance/behavior }
{
 *  SetDataBrowserPropertyFlags()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserPropertyFlags( browser: ControlRef; property: DataBrowserPropertyID; flags: DataBrowserPropertyFlags ): OSStatus; external name '_SetDataBrowserPropertyFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserPropertyFlags()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserPropertyFlags( browser: ControlRef; property: DataBrowserPropertyID; var flags: DataBrowserPropertyFlags ): OSStatus; external name '_GetDataBrowserPropertyFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Text of current in-place edit session }
{
 *  SetDataBrowserEditText()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserEditText( browser: ControlRef; text: CFStringRef ): OSStatus; external name '_SetDataBrowserEditText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyDataBrowserEditText()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function CopyDataBrowserEditText( browser: ControlRef; var text: CFStringRef ): OSStatus; external name '_CopyDataBrowserEditText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserEditText()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserEditText( browser: ControlRef; text: CFMutableStringRef ): OSStatus; external name '_GetDataBrowserEditText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Item/property currently being edited }
{
 *  SetDataBrowserEditItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserEditItem( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID ): OSStatus; external name '_SetDataBrowserEditItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserEditItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserEditItem( browser: ControlRef; var item: DataBrowserItemID; var property: DataBrowserPropertyID ): OSStatus; external name '_GetDataBrowserEditItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Get the current bounds of a visual part of an item's property }
{
 *  GetDataBrowserItemPartBounds()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemPartBounds( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; part: DataBrowserPropertyPart; var bounds: Rect ): OSStatus; external name '_GetDataBrowserItemPartBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ DataBrowser ItemData Accessors (used within DataBrowserItemData callback) }
type
	DataBrowserItemDataRef = UnivPtr;
{
 *  SetDataBrowserItemDataIcon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataIcon( itemData: DataBrowserItemDataRef; theData: IconRef ): OSStatus; external name '_SetDataBrowserItemDataIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataIcon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataIcon( itemData: DataBrowserItemDataRef; var theData: IconRef ): OSStatus; external name '_GetDataBrowserItemDataIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataText()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataText( itemData: DataBrowserItemDataRef; theData: CFStringRef ): OSStatus; external name '_SetDataBrowserItemDataText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataText()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataText( itemData: DataBrowserItemDataRef; var theData: CFStringRef ): OSStatus; external name '_GetDataBrowserItemDataText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataValue( itemData: DataBrowserItemDataRef; theData: SInt32 ): OSStatus; external name '_SetDataBrowserItemDataValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataValue( itemData: DataBrowserItemDataRef; var theData: SInt32 ): OSStatus; external name '_GetDataBrowserItemDataValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataMinimum()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataMinimum( itemData: DataBrowserItemDataRef; theData: SInt32 ): OSStatus; external name '_SetDataBrowserItemDataMinimum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataMinimum()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataMinimum( itemData: DataBrowserItemDataRef; var theData: SInt32 ): OSStatus; external name '_GetDataBrowserItemDataMinimum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataMaximum()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataMaximum( itemData: DataBrowserItemDataRef; theData: SInt32 ): OSStatus; external name '_SetDataBrowserItemDataMaximum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataMaximum()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataMaximum( itemData: DataBrowserItemDataRef; var theData: SInt32 ): OSStatus; external name '_GetDataBrowserItemDataMaximum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataBooleanValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataBooleanValue( itemData: DataBrowserItemDataRef; theData: Boolean ): OSStatus; external name '_SetDataBrowserItemDataBooleanValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataBooleanValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataBooleanValue( itemData: DataBrowserItemDataRef; var theData: Boolean ): OSStatus; external name '_GetDataBrowserItemDataBooleanValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataMenuRef()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataMenuRef( itemData: DataBrowserItemDataRef; theData: MenuRef ): OSStatus; external name '_SetDataBrowserItemDataMenuRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataMenuRef()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataMenuRef( itemData: DataBrowserItemDataRef; var theData: MenuRef ): OSStatus; external name '_GetDataBrowserItemDataMenuRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataRGBColor()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataRGBColor( itemData: DataBrowserItemDataRef; const (*var*) theData: RGBColor ): OSStatus; external name '_SetDataBrowserItemDataRGBColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataRGBColor()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataRGBColor( itemData: DataBrowserItemDataRef; var theData: RGBColor ): OSStatus; external name '_GetDataBrowserItemDataRGBColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataDrawState()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataDrawState( itemData: DataBrowserItemDataRef; theData: ThemeDrawState ): OSStatus; external name '_SetDataBrowserItemDataDrawState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataDrawState()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataDrawState( itemData: DataBrowserItemDataRef; var theData: ThemeDrawState ): OSStatus; external name '_GetDataBrowserItemDataDrawState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataButtonValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataButtonValue( itemData: DataBrowserItemDataRef; theData: ThemeButtonValue ): OSStatus; external name '_SetDataBrowserItemDataButtonValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataButtonValue()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataButtonValue( itemData: DataBrowserItemDataRef; var theData: ThemeButtonValue ): OSStatus; external name '_GetDataBrowserItemDataButtonValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataIconTransform()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataIconTransform( itemData: DataBrowserItemDataRef; theData: IconTransformType ): OSStatus; external name '_SetDataBrowserItemDataIconTransform';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataIconTransform()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataIconTransform( itemData: DataBrowserItemDataRef; var theData: IconTransformType ): OSStatus; external name '_GetDataBrowserItemDataIconTransform';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataDateTime()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataDateTime( itemData: DataBrowserItemDataRef; theData: SInt32 ): OSStatus; external name '_SetDataBrowserItemDataDateTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataDateTime()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataDateTime( itemData: DataBrowserItemDataRef; var theData: SInt32 ): OSStatus; external name '_GetDataBrowserItemDataDateTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataLongDateTime()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataLongDateTime( itemData: DataBrowserItemDataRef; (*const*) var theData: LongDateTime ): OSStatus; external name '_SetDataBrowserItemDataLongDateTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataLongDateTime()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataLongDateTime( itemData: DataBrowserItemDataRef; var theData: LongDateTime ): OSStatus; external name '_GetDataBrowserItemDataLongDateTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserItemDataItemID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserItemDataItemID( itemData: DataBrowserItemDataRef; theData: DataBrowserItemID ): OSStatus; external name '_SetDataBrowserItemDataItemID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataItemID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataItemID( itemData: DataBrowserItemDataRef; var theData: DataBrowserItemID ): OSStatus; external name '_GetDataBrowserItemDataItemID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserItemDataProperty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserItemDataProperty( itemData: DataBrowserItemDataRef; var theData: DataBrowserPropertyID ): OSStatus; external name '_GetDataBrowserItemDataProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Standard DataBrowser Callbacks }

{ Basic Item Management & Manipulation }
type
	DataBrowserItemDataProcPtr = function( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; itemData: DataBrowserItemDataRef; setValue: Boolean ): OSStatus;
type
	DataBrowserItemDataUPP = DataBrowserItemDataProcPtr;

{ Item Comparison }
type
	DataBrowserItemCompareProcPtr = function( browser: ControlRef; itemOne: DataBrowserItemID; itemTwo: DataBrowserItemID; sortProperty: DataBrowserPropertyID ): Boolean;
type
	DataBrowserItemCompareUPP = DataBrowserItemCompareProcPtr;

{ ItemEvent Notification }
{  A Very Important Note about DataBrowserItemNotificationProcPtr:                                     }
{  Under all currently shipping versions of CarbonLib (eg. up through 1.3), your callback is called    }
{  just as the prototype appears in this header. It should only be expecting three parameters because  }
{  DataBrowser will only pass three parameters.                                                        }
{  Under Mac OS X, your callback is called with an additional parameter. If you wish to interpret      }
{  the additional parameter, your callback should have the same prototype as the                       }
{  DataBrowserItemNotificationWithItemProcPtr (below). You may freely take a callback with this        }
{  prototype and pass it to NewDataBrowserItemNotificationUPP in order to generate a                   }
{  DataBrowserItemNotificationUPP that you can use just like any other DataBrowserItemNotificationUPP. }
{  If you use this technique under CarbonLib, you will *not* receive valid data in the fourth          }
{  parameter, and any attempt to use the invalid data will probably result in a crash.                 }
type
	DataBrowserItemNotificationWithItemProcPtr = procedure( browser: ControlRef; item: DataBrowserItemID; message: DataBrowserItemNotification; itemData: DataBrowserItemDataRef );
type
	DataBrowserItemNotificationProcPtr = procedure( browser: ControlRef; item: DataBrowserItemID; message: DataBrowserItemNotification );
type
	DataBrowserItemNotificationWithItemUPP = DataBrowserItemNotificationWithItemProcPtr;
type
	DataBrowserItemNotificationUPP = DataBrowserItemNotificationProcPtr;


{ Drag & Drop Processing }
type
	DataBrowserAddDragItemProcPtr = function( browser: ControlRef; theDrag: DragReference; item: DataBrowserItemID; var itemRef: ItemReference ): Boolean;
type
	DataBrowserAcceptDragProcPtr = function( browser: ControlRef; theDrag: DragReference; item: DataBrowserItemID ): Boolean;
type
	DataBrowserReceiveDragProcPtr = function( browser: ControlRef; theDrag: DragReference; item: DataBrowserItemID ): Boolean;
type
	DataBrowserPostProcessDragProcPtr = procedure( browser: ControlRef; theDrag: DragReference; trackDragResult: OSStatus );
type
	DataBrowserAddDragItemUPP = DataBrowserAddDragItemProcPtr;
type
	DataBrowserAcceptDragUPP = DataBrowserAcceptDragProcPtr;
type
	DataBrowserReceiveDragUPP = DataBrowserReceiveDragProcPtr;
type
	DataBrowserPostProcessDragUPP = DataBrowserPostProcessDragProcPtr;

{ Contextual Menu Support }
type
	DataBrowserGetContextualMenuProcPtr = procedure( browser: ControlRef; var menu: MenuRef; var helpType: UInt32; var helpItemString: CFStringRef; var selection: AEDesc );
type
	DataBrowserSelectContextualMenuProcPtr = procedure( browser: ControlRef; menu: MenuRef; selectionType: UInt32; menuID: SInt16; menuItem: MenuItemIndex );
type
	DataBrowserGetContextualMenuUPP = DataBrowserGetContextualMenuProcPtr;
type
	DataBrowserSelectContextualMenuUPP = DataBrowserSelectContextualMenuProcPtr;

{ Help Manager Support }
type
	DataBrowserItemHelpContentProcPtr = procedure( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr );
type
	DataBrowserItemHelpContentUPP = DataBrowserItemHelpContentProcPtr;
{
 *  NewDataBrowserItemDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemDataUPP( userRoutine: DataBrowserItemDataProcPtr ): DataBrowserItemDataUPP; external name '_NewDataBrowserItemDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserItemCompareUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemCompareUPP( userRoutine: DataBrowserItemCompareProcPtr ): DataBrowserItemCompareUPP; external name '_NewDataBrowserItemCompareUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserItemNotificationWithItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemNotificationWithItemUPP( userRoutine: DataBrowserItemNotificationWithItemProcPtr ): DataBrowserItemNotificationWithItemUPP; external name '_NewDataBrowserItemNotificationWithItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  NewDataBrowserItemNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemNotificationUPP( userRoutine: DataBrowserItemNotificationProcPtr ): DataBrowserItemNotificationUPP; external name '_NewDataBrowserItemNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserAddDragItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserAddDragItemUPP( userRoutine: DataBrowserAddDragItemProcPtr ): DataBrowserAddDragItemUPP; external name '_NewDataBrowserAddDragItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserAcceptDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserAcceptDragUPP( userRoutine: DataBrowserAcceptDragProcPtr ): DataBrowserAcceptDragUPP; external name '_NewDataBrowserAcceptDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserReceiveDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserReceiveDragUPP( userRoutine: DataBrowserReceiveDragProcPtr ): DataBrowserReceiveDragUPP; external name '_NewDataBrowserReceiveDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserPostProcessDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserPostProcessDragUPP( userRoutine: DataBrowserPostProcessDragProcPtr ): DataBrowserPostProcessDragUPP; external name '_NewDataBrowserPostProcessDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserGetContextualMenuUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserGetContextualMenuUPP( userRoutine: DataBrowserGetContextualMenuProcPtr ): DataBrowserGetContextualMenuUPP; external name '_NewDataBrowserGetContextualMenuUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserSelectContextualMenuUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserSelectContextualMenuUPP( userRoutine: DataBrowserSelectContextualMenuProcPtr ): DataBrowserSelectContextualMenuUPP; external name '_NewDataBrowserSelectContextualMenuUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDataBrowserItemHelpContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemHelpContentUPP( userRoutine: DataBrowserItemHelpContentProcPtr ): DataBrowserItemHelpContentUPP; external name '_NewDataBrowserItemHelpContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserItemDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemDataUPP( userUPP: DataBrowserItemDataUPP ); external name '_DisposeDataBrowserItemDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserItemCompareUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemCompareUPP( userUPP: DataBrowserItemCompareUPP ); external name '_DisposeDataBrowserItemCompareUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserItemNotificationWithItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemNotificationWithItemUPP( userUPP: DataBrowserItemNotificationWithItemUPP ); external name '_DisposeDataBrowserItemNotificationWithItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeDataBrowserItemNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemNotificationUPP( userUPP: DataBrowserItemNotificationUPP ); external name '_DisposeDataBrowserItemNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserAddDragItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserAddDragItemUPP( userUPP: DataBrowserAddDragItemUPP ); external name '_DisposeDataBrowserAddDragItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserAcceptDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserAcceptDragUPP( userUPP: DataBrowserAcceptDragUPP ); external name '_DisposeDataBrowserAcceptDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserReceiveDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserReceiveDragUPP( userUPP: DataBrowserReceiveDragUPP ); external name '_DisposeDataBrowserReceiveDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserPostProcessDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserPostProcessDragUPP( userUPP: DataBrowserPostProcessDragUPP ); external name '_DisposeDataBrowserPostProcessDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserGetContextualMenuUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserGetContextualMenuUPP( userUPP: DataBrowserGetContextualMenuUPP ); external name '_DisposeDataBrowserGetContextualMenuUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserSelectContextualMenuUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserSelectContextualMenuUPP( userUPP: DataBrowserSelectContextualMenuUPP ); external name '_DisposeDataBrowserSelectContextualMenuUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDataBrowserItemHelpContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemHelpContentUPP( userUPP: DataBrowserItemHelpContentUPP ); external name '_DisposeDataBrowserItemHelpContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserItemDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserItemDataUPP( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; itemData: DataBrowserItemDataRef; setValue: Boolean; userUPP: DataBrowserItemDataUPP ): OSStatus; external name '_InvokeDataBrowserItemDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserItemCompareUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserItemCompareUPP( browser: ControlRef; itemOne: DataBrowserItemID; itemTwo: DataBrowserItemID; sortProperty: DataBrowserPropertyID; userUPP: DataBrowserItemCompareUPP ): Boolean; external name '_InvokeDataBrowserItemCompareUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserItemNotificationWithItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserItemNotificationWithItemUPP( browser: ControlRef; item: DataBrowserItemID; message: DataBrowserItemNotification; itemData: DataBrowserItemDataRef; userUPP: DataBrowserItemNotificationWithItemUPP ); external name '_InvokeDataBrowserItemNotificationWithItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeDataBrowserItemNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserItemNotificationUPP( browser: ControlRef; item: DataBrowserItemID; message: DataBrowserItemNotification; userUPP: DataBrowserItemNotificationUPP ); external name '_InvokeDataBrowserItemNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserAddDragItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserAddDragItemUPP( browser: ControlRef; theDrag: DragReference; item: DataBrowserItemID; var itemRef: ItemReference; userUPP: DataBrowserAddDragItemUPP ): Boolean; external name '_InvokeDataBrowserAddDragItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserAcceptDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserAcceptDragUPP( browser: ControlRef; theDrag: DragReference; item: DataBrowserItemID; userUPP: DataBrowserAcceptDragUPP ): Boolean; external name '_InvokeDataBrowserAcceptDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserReceiveDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserReceiveDragUPP( browser: ControlRef; theDrag: DragReference; item: DataBrowserItemID; userUPP: DataBrowserReceiveDragUPP ): Boolean; external name '_InvokeDataBrowserReceiveDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserPostProcessDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserPostProcessDragUPP( browser: ControlRef; theDrag: DragReference; trackDragResult: OSStatus; userUPP: DataBrowserPostProcessDragUPP ); external name '_InvokeDataBrowserPostProcessDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserGetContextualMenuUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserGetContextualMenuUPP( browser: ControlRef; var menu: MenuRef; var helpType: UInt32; var helpItemString: CFStringRef; var selection: AEDesc; userUPP: DataBrowserGetContextualMenuUPP ); external name '_InvokeDataBrowserGetContextualMenuUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserSelectContextualMenuUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserSelectContextualMenuUPP( browser: ControlRef; menu: MenuRef; selectionType: UInt32; menuID: SInt16; menuItem: MenuItemIndex; userUPP: DataBrowserSelectContextualMenuUPP ); external name '_InvokeDataBrowserSelectContextualMenuUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDataBrowserItemHelpContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserItemHelpContentUPP( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr; userUPP: DataBrowserItemHelpContentUPP ); external name '_InvokeDataBrowserItemHelpContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{ Standard Callback (vtable) Structure }
const
	kDataBrowserLatestCallbacks = 0;

type
	DataBrowserCallbacksPtr = ^DataBrowserCallbacks;
	DataBrowserCallbacks = record
		version: UInt32;                { Use kDataBrowserLatestCallbacks }
		case SInt16 of
		0: (
			itemDataCallback:	DataBrowserItemDataUPP;
			itemCompareCallback: DataBrowserItemCompareUPP;
			itemNotificationCallback: DataBrowserItemNotificationUPP;
			addDragItemCallback: DataBrowserAddDragItemUPP;
			acceptDragCallback:	DataBrowserAcceptDragUPP;
			receiveDragCallback: DataBrowserReceiveDragUPP;
			postProcessDragCallback: DataBrowserPostProcessDragUPP;
			itemHelpContentCallback: DataBrowserItemHelpContentUPP;
			getContextualMenuCallback: DataBrowserGetContextualMenuUPP;
			selectContextualMenuCallback: DataBrowserSelectContextualMenuUPP;
		   );
	end;
{
 *  InitDataBrowserCallbacks()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InitDataBrowserCallbacks( var callbacks: DataBrowserCallbacks ): OSStatus; external name '_InitDataBrowserCallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Macro for initializing callback structure }
// #define InitializeDataBrowserCallbacks(callbacks, vers) \
{ (callbacks)->version = (vers); InitDataBrowserCallbacks(callbacks); }

{
 *  GetDataBrowserCallbacks()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserCallbacks( browser: ControlRef; var callbacks: DataBrowserCallbacks ): OSStatus; external name '_GetDataBrowserCallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserCallbacks()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserCallbacks( browser: ControlRef; const (*var*) callbacks: DataBrowserCallbacks ): OSStatus; external name '_SetDataBrowserCallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Custom Format Callbacks (kDataBrowserCustomType display properties) }

type
	DataBrowserDragFlags = UInt32;
type
	DataBrowserTrackingResult = SInt16;
const
	kDataBrowserContentHit = 1;
	kDataBrowserNothingHit = 0;
	kDataBrowserStopTracking = -1;

type
	DataBrowserDrawItemProcPtr = procedure( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; itemState: DataBrowserItemState; const (*var*) theRect: Rect; gdDepth: SInt16; colorDevice: Boolean );
type
	DataBrowserEditItemProcPtr = function( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; theString: CFStringRef; var maxEditTextRect: Rect; var shrinkToFit: Boolean ): Boolean;
type
	DataBrowserHitTestProcPtr = function( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; const (*var*) theRect: Rect; const (*var*) mouseRect: Rect ): Boolean;
type
	DataBrowserTrackingProcPtr = function( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; const (*var*) theRect: Rect; startPt: Point; modifiers: EventModifiers ): DataBrowserTrackingResult;
type
	DataBrowserItemDragRgnProcPtr = procedure( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; const (*var*) theRect: Rect; dragRgn: RgnHandle );
type
	DataBrowserItemAcceptDragProcPtr = function( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; const (*var*) theRect: Rect; theDrag: DragReference ): DataBrowserDragFlags;
type
	DataBrowserItemReceiveDragProcPtr = function( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; dragFlags: DataBrowserDragFlags; theDrag: DragReference ): Boolean;
type
	DataBrowserDrawItemUPP = DataBrowserDrawItemProcPtr;
type
	DataBrowserEditItemUPP = DataBrowserEditItemProcPtr;
type
	DataBrowserHitTestUPP = DataBrowserHitTestProcPtr;
type
	DataBrowserTrackingUPP = DataBrowserTrackingProcPtr;
type
	DataBrowserItemDragRgnUPP = DataBrowserItemDragRgnProcPtr;
type
	DataBrowserItemAcceptDragUPP = DataBrowserItemAcceptDragProcPtr;
type
	DataBrowserItemReceiveDragUPP = DataBrowserItemReceiveDragProcPtr;
{
 *  NewDataBrowserDrawItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserDrawItemUPP( userRoutine: DataBrowserDrawItemProcPtr ): DataBrowserDrawItemUPP; external name '_NewDataBrowserDrawItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  NewDataBrowserEditItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserEditItemUPP( userRoutine: DataBrowserEditItemProcPtr ): DataBrowserEditItemUPP; external name '_NewDataBrowserEditItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  NewDataBrowserHitTestUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserHitTestUPP( userRoutine: DataBrowserHitTestProcPtr ): DataBrowserHitTestUPP; external name '_NewDataBrowserHitTestUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  NewDataBrowserTrackingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserTrackingUPP( userRoutine: DataBrowserTrackingProcPtr ): DataBrowserTrackingUPP; external name '_NewDataBrowserTrackingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  NewDataBrowserItemDragRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemDragRgnUPP( userRoutine: DataBrowserItemDragRgnProcPtr ): DataBrowserItemDragRgnUPP; external name '_NewDataBrowserItemDragRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  NewDataBrowserItemAcceptDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemAcceptDragUPP( userRoutine: DataBrowserItemAcceptDragProcPtr ): DataBrowserItemAcceptDragUPP; external name '_NewDataBrowserItemAcceptDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  NewDataBrowserItemReceiveDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewDataBrowserItemReceiveDragUPP( userRoutine: DataBrowserItemReceiveDragProcPtr ): DataBrowserItemReceiveDragUPP; external name '_NewDataBrowserItemReceiveDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeDataBrowserDrawItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserDrawItemUPP( userUPP: DataBrowserDrawItemUPP ); external name '_DisposeDataBrowserDrawItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeDataBrowserEditItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserEditItemUPP( userUPP: DataBrowserEditItemUPP ); external name '_DisposeDataBrowserEditItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeDataBrowserHitTestUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserHitTestUPP( userUPP: DataBrowserHitTestUPP ); external name '_DisposeDataBrowserHitTestUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeDataBrowserTrackingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserTrackingUPP( userUPP: DataBrowserTrackingUPP ); external name '_DisposeDataBrowserTrackingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeDataBrowserItemDragRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemDragRgnUPP( userUPP: DataBrowserItemDragRgnUPP ); external name '_DisposeDataBrowserItemDragRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeDataBrowserItemAcceptDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemAcceptDragUPP( userUPP: DataBrowserItemAcceptDragUPP ); external name '_DisposeDataBrowserItemAcceptDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeDataBrowserItemReceiveDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeDataBrowserItemReceiveDragUPP( userUPP: DataBrowserItemReceiveDragUPP ); external name '_DisposeDataBrowserItemReceiveDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeDataBrowserDrawItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserDrawItemUPP( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; itemState: DataBrowserItemState; const (*var*) theRect: Rect; gdDepth: SInt16; colorDevice: Boolean; userUPP: DataBrowserDrawItemUPP ); external name '_InvokeDataBrowserDrawItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeDataBrowserEditItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserEditItemUPP( browser: ControlRef; item: DataBrowserItemID; property: DataBrowserPropertyID; theString: CFStringRef; var maxEditTextRect: Rect; var shrinkToFit: Boolean; userUPP: DataBrowserEditItemUPP ): Boolean; external name '_InvokeDataBrowserEditItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeDataBrowserHitTestUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserHitTestUPP( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; const (*var*) theRect: Rect; const (*var*) mouseRect: Rect; userUPP: DataBrowserHitTestUPP ): Boolean; external name '_InvokeDataBrowserHitTestUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeDataBrowserTrackingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserTrackingUPP( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; const (*var*) theRect: Rect; startPt: Point; modifiers: EventModifiers; userUPP: DataBrowserTrackingUPP ): DataBrowserTrackingResult; external name '_InvokeDataBrowserTrackingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeDataBrowserItemDragRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokeDataBrowserItemDragRgnUPP( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; const (*var*) theRect: Rect; dragRgn: RgnHandle; userUPP: DataBrowserItemDragRgnUPP ); external name '_InvokeDataBrowserItemDragRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeDataBrowserItemAcceptDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserItemAcceptDragUPP( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; const (*var*) theRect: Rect; theDrag: DragReference; userUPP: DataBrowserItemAcceptDragUPP ): DataBrowserDragFlags; external name '_InvokeDataBrowserItemAcceptDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeDataBrowserItemReceiveDragUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeDataBrowserItemReceiveDragUPP( browser: ControlRef; itemID: DataBrowserItemID; property: DataBrowserPropertyID; dragFlags: DataBrowserDragFlags; theDrag: DragReference; userUPP: DataBrowserItemReceiveDragUPP ): Boolean; external name '_InvokeDataBrowserItemReceiveDragUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{ Custom Callback (vtable) Structure }
const
	kDataBrowserLatestCustomCallbacks = 0;

type
	DataBrowserCustomCallbacksPtr = ^DataBrowserCustomCallbacks;
	DataBrowserCustomCallbacks = record
		version: UInt32;                { Use kDataBrowserLatestCustomCallbacks }
		case SInt16 of
		0: (
			drawItemCallback:	DataBrowserDrawItemUPP;
			editTextCallback:	DataBrowserEditItemUPP;
			hitTestCallback:	DataBrowserHitTestUPP;
			trackingCallback:	DataBrowserTrackingUPP;
			dragRegionCallback:	DataBrowserItemDragRgnUPP;
			acceptDragCallback:	DataBrowserItemAcceptDragUPP;
			receiveDragCallback: DataBrowserItemReceiveDragUPP;
		   );
	end;
{
 *  InitDataBrowserCustomCallbacks()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InitDataBrowserCustomCallbacks( var callbacks: DataBrowserCustomCallbacks ): OSStatus; external name '_InitDataBrowserCustomCallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Macro for initializing custom callback structure }
// #define InitializeDataBrowserCustomCallbacks(callbacks, vers) \
{ (callbacks)->version = (vers); InitDataBrowserCustomCallbacks(callbacks); }

{
 *  GetDataBrowserCustomCallbacks()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserCustomCallbacks( browser: ControlRef; var callbacks: DataBrowserCustomCallbacks ): OSStatus; external name '_GetDataBrowserCustomCallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserCustomCallbacks()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserCustomCallbacks( browser: ControlRef; const (*var*) callbacks: DataBrowserCustomCallbacks ): OSStatus; external name '_SetDataBrowserCustomCallbacks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ TableView Formatting }
type
	DataBrowserTableViewHiliteStyle = UInt32;
const
	kDataBrowserTableViewMinimalHilite = 0;
	kDataBrowserTableViewFillHilite = 1;

type
	DataBrowserTableViewPropertyFlags = UInt32;
const
{ kDataBrowserTableView DataBrowserPropertyFlags }
	kDataBrowserTableViewSelectionColumn = 1 shl kDataBrowserViewSpecificFlagsOffset;

{ The row and column indicies are zero-based }

type
	DataBrowserTableViewRowIndex = UInt32;
type
	DataBrowserTableViewColumnIndex = UInt32;
type
	DataBrowserTableViewColumnID = DataBrowserPropertyID;
type
	DataBrowserTableViewColumnDesc = DataBrowserPropertyDesc;
	DataBrowserTableViewColumnDescPtr = ^DataBrowserTableViewColumnDesc;

{ TableView API }
{ Use when setting column position }
const
	kDataBrowserTableViewLastColumn = $FFFFFFFF;

{
 *  RemoveDataBrowserTableViewColumn()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveDataBrowserTableViewColumn( browser: ControlRef; column: DataBrowserTableViewColumnID ): OSStatus; external name '_RemoveDataBrowserTableViewColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewColumnCount()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewColumnCount( browser: ControlRef; var numColumns: UInt32 ): OSStatus; external name '_GetDataBrowserTableViewColumnCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserTableViewHiliteStyle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTableViewHiliteStyle( browser: ControlRef; hiliteStyle: DataBrowserTableViewHiliteStyle ): OSStatus; external name '_SetDataBrowserTableViewHiliteStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewHiliteStyle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewHiliteStyle( browser: ControlRef; var hiliteStyle: DataBrowserTableViewHiliteStyle ): OSStatus; external name '_GetDataBrowserTableViewHiliteStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserTableViewRowHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTableViewRowHeight( browser: ControlRef; height: UInt16 ): OSStatus; external name '_SetDataBrowserTableViewRowHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewRowHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewRowHeight( browser: ControlRef; var height: UInt16 ): OSStatus; external name '_GetDataBrowserTableViewRowHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserTableViewColumnWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTableViewColumnWidth( browser: ControlRef; width: UInt16 ): OSStatus; external name '_SetDataBrowserTableViewColumnWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewColumnWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewColumnWidth( browser: ControlRef; var width: UInt16 ): OSStatus; external name '_GetDataBrowserTableViewColumnWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserTableViewItemRowHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTableViewItemRowHeight( browser: ControlRef; item: DataBrowserItemID; height: UInt16 ): OSStatus; external name '_SetDataBrowserTableViewItemRowHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewItemRowHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewItemRowHeight( browser: ControlRef; item: DataBrowserItemID; var height: UInt16 ): OSStatus; external name '_GetDataBrowserTableViewItemRowHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserTableViewNamedColumnWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTableViewNamedColumnWidth( browser: ControlRef; column: DataBrowserTableViewColumnID; width: UInt16 ): OSStatus; external name '_SetDataBrowserTableViewNamedColumnWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewNamedColumnWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewNamedColumnWidth( browser: ControlRef; column: DataBrowserTableViewColumnID; var width: UInt16 ): OSStatus; external name '_GetDataBrowserTableViewNamedColumnWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserTableViewGeometry()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTableViewGeometry( browser: ControlRef; variableWidthColumns: Boolean; variableHeightRows: Boolean ): OSStatus; external name '_SetDataBrowserTableViewGeometry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewGeometry()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewGeometry( browser: ControlRef; var variableWidthColumns: Boolean; var variableHeightRows: Boolean ): OSStatus; external name '_GetDataBrowserTableViewGeometry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewItemID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewItemID( browser: ControlRef; row: DataBrowserTableViewRowIndex; var item: DataBrowserItemID ): OSStatus; external name '_GetDataBrowserTableViewItemID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserTableViewItemRow()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTableViewItemRow( browser: ControlRef; item: DataBrowserItemID; row: DataBrowserTableViewRowIndex ): OSStatus; external name '_SetDataBrowserTableViewItemRow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewItemRow()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewItemRow( browser: ControlRef; item: DataBrowserItemID; var row: DataBrowserTableViewRowIndex ): OSStatus; external name '_GetDataBrowserTableViewItemRow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserTableViewColumnPosition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserTableViewColumnPosition( browser: ControlRef; column: DataBrowserTableViewColumnID; position: DataBrowserTableViewColumnIndex ): OSStatus; external name '_SetDataBrowserTableViewColumnPosition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewColumnPosition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewColumnPosition( browser: ControlRef; column: DataBrowserTableViewColumnID; var position: DataBrowserTableViewColumnIndex ): OSStatus; external name '_GetDataBrowserTableViewColumnPosition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserTableViewColumnProperty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserTableViewColumnProperty( browser: ControlRef; column: DataBrowserTableViewColumnIndex; var property: DataBrowserTableViewColumnID ): OSStatus; external name '_GetDataBrowserTableViewColumnProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ kDataBrowserListView Formatting }

{
 *  Discussion:
 *    DataBrowserPropertyFlags that are specific to kDataBrowserListView
 }
const
	kDataBrowserListViewSelectionColumn = kDataBrowserTableViewSelectionColumn;
	kDataBrowserListViewMovableColumn = 1 shl (kDataBrowserViewSpecificFlagsOffset + 1);
	kDataBrowserListViewSortableColumn = 1 shl (kDataBrowserViewSpecificFlagsOffset + 2);

  {
   * kDataBrowserListViewTypeSelectColumn marks a column as
   * type-selectable. If one or more of your list view columns are
   * marked as type-selectable, Data Browser will do type-selection for
   * you automatically. Data Browser applies the typing to the first
   * column (in the system direction) with this property flag. This
   * flag only intended for use with columns of type
   * kDataBrowserTextType, kDataBrowserIconAndTextType, and
   * kDataBrowserDateTimeType; if you set it for a column of another
   * type, the type-selection behavior is undefined. Turning on this
   * flag also causes Data Browser to gather all keyboard input via a
   * carbon event handler instead of relying on calls to
   * HandleControlKey; therefore, you will never see these keyboard
   * events come out of WaitNextEvent. Only available on 10.3 and later.
   }
	kDataBrowserListViewTypeSelectColumn = 1 shl (kDataBrowserViewSpecificFlagsOffset + 3);

  {
   * Normally the text in a header button for a column of type
   * kDataBrowserIconAndTextType is aligned as though it has an icon
   * next to it even if no icon is specified for the header button; in
   * other words, space is reserved for an icon in the header button
   * even if no icon is displayed. However, this flag indicates that
   * space should not be reserved for an icon if no icon is provided
   * for the header button. This flag allows a client to justify the
   * left edge of the text in a header button to the left edge of the
   * icon in the cells beneath it. Available on 10.4 and later.
   }
	kDataBrowserListViewNoGapForIconInHeaderButton = 1 shl (kDataBrowserViewSpecificFlagsOffset + 4);
	kDataBrowserListViewDefaultColumnFlags = kDataBrowserListViewMovableColumn + kDataBrowserListViewSortableColumn;

type
	DataBrowserListViewPropertyFlags = DataBrowserPropertyFlags;
const
	kDataBrowserListViewLatestHeaderDesc = 0;

type
	DataBrowserListViewHeaderDescPtr = ^DataBrowserListViewHeaderDesc;
	DataBrowserListViewHeaderDesc = record
		version: UInt32;                { Use kDataBrowserListViewLatestHeaderDesc }

		minimumWidth: UInt16;
		maximumWidth: UInt16;

		titleOffset: SInt16;
		titleString: CFStringRef;
		initialOrder: DataBrowserSortOrder;
		btnFontStyle: ControlFontStyleRec;
		btnContentInfo: ControlButtonContentInfo;
	end;
type
	DataBrowserListViewColumnDescPtr = ^DataBrowserListViewColumnDesc;
	DataBrowserListViewColumnDesc = record
		propertyDesc: DataBrowserTableViewColumnDesc;
		headerBtnDesc: DataBrowserListViewHeaderDesc;
	end;
{ kDataBrowserListView API }
const
	kDataBrowserListViewAppendColumn = kDataBrowserTableViewLastColumn;

{
 *  AutoSizeDataBrowserListViewColumns()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AutoSizeDataBrowserListViewColumns( browser: ControlRef ): OSStatus; external name '_AutoSizeDataBrowserListViewColumns';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AddDataBrowserListViewColumn()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AddDataBrowserListViewColumn( browser: ControlRef; var columnDesc: DataBrowserListViewColumnDesc; position: DataBrowserTableViewColumnIndex ): OSStatus; external name '_AddDataBrowserListViewColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserListViewHeaderDesc()
 *  
 *  Summary:
 *    Returns information about a specified column header in a list
 *    view.
 *  
 *  Discussion:
 *    Note that this API does not correctly use CoreFoundation naming
 *    conventions. Although the API name begins with "Get", implying
 *    that you do not need to release the CFStringRef and IconRef
 *    returned by this API, in fact you do actually need to release
 *    these objects.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    browser:
 *      The data browser for which you need header information.
 *    
 *    column:
 *      The column ID for which you need header information.
 *    
 *    desc:
 *      On exit, contains header information for the specified column.
 *      You must release the CFStringRef and IconRef contained in this
 *      structure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserListViewHeaderDesc( browser: ControlRef; column: DataBrowserTableViewColumnID; var desc: DataBrowserListViewHeaderDesc ): OSStatus; external name '_GetDataBrowserListViewHeaderDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  SetDataBrowserListViewHeaderDesc()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserListViewHeaderDesc( browser: ControlRef; column: DataBrowserTableViewColumnID; var desc: DataBrowserListViewHeaderDesc ): OSStatus; external name '_SetDataBrowserListViewHeaderDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  SetDataBrowserListViewHeaderBtnHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserListViewHeaderBtnHeight( browser: ControlRef; height: UInt16 ): OSStatus; external name '_SetDataBrowserListViewHeaderBtnHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserListViewHeaderBtnHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserListViewHeaderBtnHeight( browser: ControlRef; var height: UInt16 ): OSStatus; external name '_GetDataBrowserListViewHeaderBtnHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserListViewUsePlainBackground()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserListViewUsePlainBackground( browser: ControlRef; usePlainBackground: Boolean ): OSStatus; external name '_SetDataBrowserListViewUsePlainBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserListViewUsePlainBackground()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserListViewUsePlainBackground( browser: ControlRef; var usePlainBackground: Boolean ): OSStatus; external name '_GetDataBrowserListViewUsePlainBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserListViewDisclosureColumn()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserListViewDisclosureColumn( browser: ControlRef; column: DataBrowserTableViewColumnID; expandableRows: Boolean ): OSStatus; external name '_SetDataBrowserListViewDisclosureColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserListViewDisclosureColumn()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserListViewDisclosureColumn( browser: ControlRef; var column: DataBrowserTableViewColumnID; var expandableRows: Boolean ): OSStatus; external name '_GetDataBrowserListViewDisclosureColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ kDataBrowserColumnView API }
{
 *  GetDataBrowserColumnViewPath()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserColumnViewPath( browser: ControlRef; path: Handle ): OSStatus; external name '_GetDataBrowserColumnViewPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserColumnViewPathLength()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserColumnViewPathLength( browser: ControlRef; var pathLength: UInt32 ): OSStatus; external name '_GetDataBrowserColumnViewPathLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserColumnViewPath()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserColumnViewPath( browser: ControlRef; length: UInt32; path: DataBrowserItemIDPtr ): OSStatus; external name '_SetDataBrowserColumnViewPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDataBrowserColumnViewDisplayType()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDataBrowserColumnViewDisplayType( browser: ControlRef; propertyType: DataBrowserPropertyType ): OSStatus; external name '_SetDataBrowserColumnViewDisplayType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDataBrowserColumnViewDisplayType()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDataBrowserColumnViewDisplayType( browser: ControlRef; var propertyType: DataBrowserPropertyType ): OSStatus; external name '_GetDataBrowserColumnViewDisplayType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ DataBrowser UPP macros }
{
    Customizing Data Browser Accessibility Information
    
    Warning: The following assumes you already understand how to handle the
    Accessibility Carbon Events described in CarbonEvents.h.
    
    Data Browser automatically handles the various Accessibility Carbon
    Events to provide a large amount of Accessibility information. However,
    your application may need to override or augment the default information
    that Data Browser provides.
    
    Though it is already possible for your application to install various
    Accessibility Carbon Event handlers on a Data Browser instance, it is
    impossible to interpret the AXUIElementRefs contained in the events
    without the help of the Data Browser. A given AXUIElementRef that is
    passed to Data Browser list view in an Accessibility Carbon Event could
    represent a row, a cell, or the list view as a whole. If your
    application needs to add an attribute to only the rows in a list view,
    your application will need to ask Data Browser what any given
    AXUIElementRef represents. The AXUIElementGetDataBrowserItemInfo allows
    your application to ask that question.
    
    Additionally, your application may want to generate its own AXUIElementRefs
    that represent children of or point to various rows or cells of a Data Browser
    instance. The AXUIElementCreateWithDataBrowserAndItemInfo API allows your
    application to manufacture AXUIElementRefs that represent certain parts of a
    Data Browser so you can provide them in your Accessibility Carbon Event
    handlers.
    
    Typical Usage Scenario: You want to add an Accessibility attribute to
    all rows in a Data Browser list view.
    
        Step 1: Install the appropriate Accessibility Carbon Event handlers
        on your Data Browser instance. Call InstallEventHandler or a similar
        API to install a handler onto your Data Browser ControlRef for the
        kEventAccessibleGetAllAttributeNames,
        kEventAccessibleGetNamedAttribute, and other appropriate events.
        
        Step 2: Your handler should find out what part of the Data Browser
        is being asked for its accessibility information. Extract the
        kEventParamAccessibleObject parameter out of the Carbon Event and
        pass it to AXUIElementGetDataBrowserItemInfo. See that API
        description for more usage information and calling requirements.
        Examine the DataBrowserAccessibilityItemInfo structure that is
        filled out to determine whether it represents the part of the Data
        Browser you are interested in adding an attribute to. In this case,
        you are looking for a row, so you would make sure the item field is
        not kDataBrowserNoItem, and that the columnProperty is
        kDataBrowserItemNoProperty.
        
        Step 3: Your event handler should call CallNextEventHandler to allow
        the Data Browser to do the default handling of the event. This is
        particularly important if the AXUIElementRef did not represent a
        row, since you don't want to disrupt the Data Browser's handling of
        the event for parts other than rows.
        
        Step 4: If you determined that the part was a row in step 2, your
        handler should now do whatever custom work it deems necessary. For
        the kEventAccessibleGetAllAttributeNames, your handler would extract
        the kEventParamAccessibleAttributeNames parameter out of the event
        and add your custom attribute name to the array. For the
        kEventAccessibleGetNamedAttribute event, your handler would test the
        kEventParamAccessibleAttributeName parameter to see if it matches
        your custom attribute name; if so, your handler would put its custom
        data in the kEventParamAccessibleAttributeValue parameter. Any other
        events would be handled similarly.
        
        Step 5: Your event handler should return an appropriate result code.
        In cases where the AXUIElementRef does not represent a row or when
        the attribute name is not your custom attribute, your handler can
        return the same result code that was returned by
        CallNextEventHandler in step 3. In cases where your handler decided
        to augment or override the default handling of the event, your
        handler will typically want to return noErr. See the Carbon Event
        documentation for more details on the meanings of result codes
        returned by event handlers.
}

{
 *  DataBrowserAccessibilityItemInfoV0
 *  
 *  Summary:
 *    A specific description of Data Browser accessibility item
 *    information.
 *  
 *  Discussion:
 *    If you fill this structure as part of a
 *    DataBrowserAccessibilityItemInfo, you must set the
 *    DataBrowserAccessibilityItemInfo's version field to zero.
 }
type
	DataBrowserAccessibilityItemInfoV0 = record
{
   * The DataBrowserItemID of the container the AXUIElementRef
   * represents or lives within. Even kDataBrowserNoItem might be
   * meaningful, since it is the root container ID if you haven't
   * overridden it via SetDataBrowserTarget. In list view, the
   * container helps narrow down the AXUIElementRef to either a
   * disclosed child of another row, or the list as a whole. In column
   * view, the container helps narrow down the AXUIElementRef to a
   * column; also see the columnProperty description below.
   }
		container: DataBrowserItemID;

  {
   * The DataBrowserItemID of the item the AXUIElementRef represents or
   * lives within. If item is kDataBrowserNoItem, the AXUIElementRef
   * represents just the container. In list view, the item helps narrow
   * down the AXUIElementRef to either a row, or the root container as
   * a whole. In column view, the item helps narrow down the
   * AXUIElementRef to either a cell, or a column as a whole; also see
   * the columnProperty description below.
   }
		item: DataBrowserItemID;

  {
   * The DataBrowserPropertyID of the column the AXUIElementRef
   * represents or lives within. If columnProperty is
   * kDataBrowserItemNoProperty and item is not kDataBrowserNoItem, the
   * AXUIElementRef represents a whole row. In list view, this field
   * helps narrow down the AXUIElementRef to either a cell, or a row as
   * a whole. In column view, the columnProperty will/must always be
   * set to kDataBrowserItemNoProperty unless the AXUIElementRef
   * represents the preview column. When the AXUIElementRef represents
   * the preview column, the columnProperty will/must always be set to
   * kDataBrowserColumnViewPreviewProperty, and the other fields of
   * this structure will/must be set to zero or the equivalent constant.
   }
		columnProperty: DataBrowserPropertyID;

  {
   * The DataBrowserPropertyPart of the sub-cell part the
   * AXUIElementRef represents. Examples include the disclosure
   * triangle in a cell, the text in a cell, and the check box in a
   * cell. If propertyPart is kDataBrowserPropertyEnclosingPart and
   * columnProperty is not kDataBrowserItemNoProperty, the
   * AXUIElementRef represents the cell as a whole. In both list view
   * and column view, this field helps narrow down the AXUIElementRef
   * to either a sub-cell part, or a cell as a whole. For column view,
   * also see the columnProperty description above.
   }
		propertyPart: DataBrowserPropertyPart;
	end;

{
 *  DataBrowserAccessibilityItemInfoV1
 *  
 *  Summary:
 *    A specific description of Data Browser accessibility item
 *    information.
 *  
 *  Discussion:
 *    If you fill this structure as part of a
 *    DataBrowserAccessibilityItemInfo, you must set the
 *    DataBrowserAccessibilityItemInfo's version field to one. 
 *     
 *    This structure is identical to the V0 structure except for the
 *    inclusion of row and column indicies. These indicies may be
 *    useful to clients who call AXUIElementGetDataBrowserItemInfo.
 *    
 *    If your Data Browser instance allows a given item and/or
 *    container to be displayed more than once at a given point in
 *    time, you can use the row and column indicies to differentiate
 *    the particular visual occurances of that item when calling
 *    AXUIElementCreateWithDataBrowserAndItemInfo. See the additional
 *    details in the rowIndex and columnIndex discussions below.
 }
type
	DataBrowserAccessibilityItemInfoV1 = record
{
   * The DataBrowserItemID of the container the AXUIElementRef
   * represents or lives within. Even kDataBrowserNoItem might be
   * meaningful, since it is the root container ID if you haven't
   * overridden it via SetDataBrowserTarget. In list view, the
   * container helps narrow down the AXUIElementRef to either a
   * disclosed child of another row, or the list as a whole. In column
   * view, the container helps narrow down the AXUIElementRef to a
   * column; also see the columnProperty description below.
   }
		container: DataBrowserItemID;

  {
   * The DataBrowserItemID of the item the AXUIElementRef represents or
   * lives within. If item is kDataBrowserNoItem, the AXUIElementRef
   * represents just the container. In list view, the item helps narrow
   * down the AXUIElementRef to either a row, or the root container as
   * a whole. In column view, the item helps narrow down the
   * AXUIElementRef to either a cell, or a column as a whole; also see
   * the columnProperty description below.
   }
		item: DataBrowserItemID;

  {
   * The DataBrowserPropertyID of the column the AXUIElementRef
   * represents or lives within. If columnProperty is
   * kDataBrowserItemNoProperty and item is not kDataBrowserNoItem, the
   * AXUIElementRef represents a whole row. In list view, this field
   * helps narrow down the AXUIElementRef to either a cell, or a row as
   * a whole. In column view, the columnProperty will/must always be
   * set to kDataBrowserItemNoProperty unless the AXUIElementRef
   * represents the preview column. When the AXUIElementRef represents
   * the preview column, the columnProperty will/must always be set to
   * kDataBrowserColumnViewPreviewProperty, and the other fields of
   * this structure will/must be set to zero or the equivalent constant.
   }
		columnProperty: DataBrowserPropertyID;

  {
   * The DataBrowserPropertyPart of the sub-cell part the
   * AXUIElementRef represents. Examples include the disclosure
   * triangle in a cell, the text in a cell, and the check box in a
   * cell. If propertyPart is kDataBrowserPropertyEnclosingPart and
   * columnProperty is not kDataBrowserItemNoProperty, the
   * AXUIElementRef represents the cell as a whole. In both list view
   * and column view, this field helps narrow down the AXUIElementRef
   * to either a sub-cell part, or a cell as a whole. For column view,
   * also see the columnProperty description above.
   }
		propertyPart: DataBrowserPropertyPart;

  {
   * The zero-based DataBrowserTableViewRowIndex of the row specified
   * by the other parts of this structure. If the other parts of this
   * structure do not specify a row or a part thereof, this field
   * will/must be set to zero; because this field is zero based, you
   * must test the other parts this structure to see whether this field
   * is meaningful. In list view, when the other parts of this
   * structure specify an item or part thereof, this field will/must be
   * set to the row index at which the specified item can be found. In
   * column view, when the other parts of this structure specify a cell
   * or part thereof, this field will/must be set to the row index at
   * which the specified cell can be found.
   }
		rowIndex: DataBrowserTableViewRowIndex;

  {
   * The zero-based DataBrowserTableViewColumnIndex of the column
   * specified by the other parts of this structure. If the other parts
   * of this structure do not specify a column or a part thereof, this
   * field will/must be set to zero; because this field is zero based,
   * you must test the other parts this structure to see whether this
   * field is meaningful. In list view, when the other parts of this
   * structure specify a cell or part thereof, this field will/must be
   * set to the column index at which the specified cell can be found.
   * In column view, when the other parts of this structure specify a
   * column or part thereof, this field will/must be set to the column
   * index at which the specified cell can be found.
   }
		columnIndex: DataBrowserTableViewColumnIndex;
	end;

{
 *  DataBrowserAccessibilityItemInfo
 *  
 *  Summary:
 *    A generalized description of Data Browser accessibility item
 *    information.
 *  
 *  Discussion:
 *    Pass this structure to AXUIElementGetDataBrowserItemInfo or
 *    AXUIElementCreateWithDataBrowserAndItemInfo.
 }
type
	DataBrowserAccessibilityItemInfo = record
{
   * A UInt32 which identifies how to interpret the following union.
   * Set this field to zero if you fill out the union's data in the
   * form of a DataBrowserAccessibilityItemInfoV0 structure. Set this
   * field to one if you fill out the union's data in the form of a
   * DataBrowserAccessibilityItemInfoV1 structure.
   }
		version: UInt32;
		case SInt16 of
		0: (
			v0: DataBrowserAccessibilityItemInfoV0;
		);
		1: (
			v1: DataBrowserAccessibilityItemInfoV1;
		);
	end;
{
 *  AXUIElementGetDataBrowserItemInfo()
 *  
 *  Summary:
 *    Gets a description of the part of a Data Browser represented by a
 *    given AXUIElementRef.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
 *    window:
 *      The window that should contain the control. May be NULL on 10.3
 *      and later.
 *    
 *    boundsRect:
 *      The bounding box of the control.
 *    
 *    content:
 *      The descriptor for the picture you want the control to display.
 *    
 *    dontTrack:
 *      A Boolean value indicating whether the control should hilite
 *      when it is clicked on. False means hilite and track the mouse.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function CreatePictureControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; const (*var*) content: ControlButtonContentInfo; dontTrack: Boolean; var outControl: ControlRef ): OSStatus; external name '_CreatePictureControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by picture controls }
{$endc} {not TARGET_CPU_64}

const
	kControlPictureHandleTag = FourCharCode('pich'); { PicHandle}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ LIST BOX (CDEF 22)                                                                }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Lists use an auxiliary resource to define their format. The resource type used is   }
{  'ldes' and a definition for it can be found in Appearance.r. The resource ID for    }
{  the ldes is passed in the 'value' parameter when creating the control. You may pass }
{  zero in value. This tells the List Box control to not use a resource. The list will }
{  be created with default values, and will use the standard LDEF (0). You can change  }
{  the list by getting the list handle. You can set the LDEF to use by using the tag   }
{  below (kControlListBoxLDEFTag)                                                      }
{ List Box proc IDs }
const
	kControlListBoxProc = 352;
	kControlListBoxAutoSizeProc = 353;

{ Control Kind Tag }
const
	kControlKindListBox = FourCharCode('lbox');

{$ifc not TARGET_CPU_64}
{
 *  CreateListBoxControl()
=======
=======
>>>>>>> origin/fixes_2_2
 *    inElement:
 *      An AXUIElementRef representing part of a Data Browser.
 *    
 *    inDataBrowser:
 *      A Data Browser ControlRef.
 *    
 *    inDesiredInfoVersion:
 *      A UInt32 indicating the the version you want the ioInfo
 *      structure passed back as. Currently, the only supported version
 *      is zero, so you must pass zero in the inDesiredInfoVersion
 *      parameter.
 *    
 *    outInfo:
 *      A DataBrowserAccessibilityItemInfo that will be filled in with
 *      a description of the part of the Data Browser that the
 *      AXUIElementRef represents.
 *  
 *  Result:
 *    An OSStatus result code. The function will return noErr if it was
 *    able to generate a description of the AXUIElementRef. If the
 *    AXUIElementRef does not represent the Data Browser you passed in,
 *    the function will return paramErr. If the AXUIElementRef
 *    represents some non-item part of the Data Browser, the function
 *    will return errDataBrowserItemNotFound.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function AXUIElementGetDataBrowserItemInfo( inElement: AXUIElementRef; inDataBrowser: ControlRef; inDesiredInfoVersion: UInt32; var outInfo: DataBrowserAccessibilityItemInfo ): OSStatus; external name '_AXUIElementGetDataBrowserItemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  AXUIElementCreateWithDataBrowserAndItemInfo()
 *  
 *  Summary:
 *    Creates an AXUIElementRef to represent some part of a Data
 *    Browser accessibility hierarchy.
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateListBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; autoSize: Boolean; numRows: SInt16; numColumns: SInt16; horizScroll: Boolean; vertScroll: Boolean; cellHeight: SInt16; cellWidth: SInt16; hasGrowSpace: Boolean; const (*var*) listDef: ListDefSpec; var outControl: ControlRef ): OSStatus; external name '_CreateListBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by list box }
{$endc} {not TARGET_CPU_64}

const
	kControlListBoxListHandleTag = FourCharCode('lhan'); { ListHandle}
	kControlListBoxKeyFilterTag = kControlKeyFilterTag; { ControlKeyFilterUPP}
	kControlListBoxFontStyleTag = kControlFontStyleTag; { ControlFontStyleRec}

{ New tags in 1.0.1 or later }
const
	kControlListBoxDoubleClickTag = FourCharCode('dblc'); { Boolean. Was last click a double-click?}
	kControlListBoxLDEFTag = FourCharCode('ldef'); { SInt16. ID of LDEF to use.}

{ Resource Types }
const
	kControlListDescResType = FourCharCode('ldes'); { used for list box control (Appearance 1.0 and later)}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ SCROLL TEXT BOX (CDEF 27)                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  This control implements a scrolling box of (non-editable) text. This is useful for  }
{  credits in about boxes, etc.                                                        }
{  The standard version of this control has a scroll bar, but the autoscrolling        }
{  variant does not. The autoscrolling variant needs two pieces of information to      }
{  work: delay (in ticks) before the scrolling starts, and time (in ticks) between     }
{  scrolls. It will scroll one pixel at a time, unless changed via SetControlData.     }
{  Parameter                   What Goes Here                                          }
{  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ         ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ    }
{  Value                       Resource ID of 'TEXT'/'styl' content.                   }
{  Min                         Scroll start delay (in ticks)                       .   }
{  Max                         Delay (in ticks) between scrolls.                       }
{  NOTE: This control is only available with Appearance 1.1.                           }
{ Scroll Text Box Proc IDs }
const
	kControlScrollTextBoxProc = 432;
	kControlScrollTextBoxAutoScrollProc = 433;

{ Control Kind Tag }
const
	kControlKindScrollingTextBox = FourCharCode('stbx');

{$ifc not TARGET_CPU_64}
{
 *  CreateScrollingTextBoxControl()
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inDataBrowser:
 *      A Data Browser ControlRef.
 *    
 *    inInfo:
 *      A DataBrowserAccessibilityItemInfo describing the part of the
 *      Data Browser for which you want to create an AXUIElementRef.
 *  
 *  Result:
 *    An AXUIElementRef representing the part, or NULL if one cannot be
 *    created to represent the part you specified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function AXUIElementCreateWithDataBrowserAndItemInfo( inDataBrowser: ControlRef; const (*var*) inInfo: DataBrowserAccessibilityItemInfo ): AXUIElementRef; external name '_AXUIElementCreateWithDataBrowserAndItemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{---------------------------------------------------------------------------------------}
{ EditUnicodeText Control                                                               }
{---------------------------------------------------------------------------------------}
{ This control is only available in Mac OS X.  It is super similar to Edit Text control }
{ Use all the same Get/Set tags.  But don't ask for the TEHandle.                       }
{---------------------------------------------------------------------------------------}
{ This callback supplies the functionality of the TSMTEPostUpdateProcPtr that is used }
{ in the EditText control.  A client should supply this call if they want to look at  }
{ inline text that has been fixed before it is included in the actual body text       }
{ if the new text (i.e. the text in the handle) should be included in the body text    }
{ the client should return true.  If the client wants to block the inclusion of the    }
{ text they should return false.                                                       }
type
	EditUnicodePostUpdateProcPtr = function( uniText: UniCharArrayHandle; uniTextLength: UniCharCount; iStartOffset: UniCharArrayOffset; iEndOffset: UniCharArrayOffset; refcon: UnivPtr ): Boolean;
type
	EditUnicodePostUpdateUPP = EditUnicodePostUpdateProcPtr;
{
 *  NewEditUnicodePostUpdateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewEditUnicodePostUpdateUPP( userRoutine: EditUnicodePostUpdateProcPtr ): EditUnicodePostUpdateUPP; external name '_NewEditUnicodePostUpdateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeEditUnicodePostUpdateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeEditUnicodePostUpdateUPP( userUPP: EditUnicodePostUpdateUPP ); external name '_DisposeEditUnicodePostUpdateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeEditUnicodePostUpdateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeEditUnicodePostUpdateUPP( uniText: UniCharArrayHandle; uniTextLength: UniCharCount; iStartOffset: UniCharArrayOffset; iEndOffset: UniCharArrayOffset; refcon: UnivPtr; userUPP: EditUnicodePostUpdateUPP ): Boolean; external name '_InvokeEditUnicodePostUpdateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

const
	kControlEditUnicodeTextProc = 912;
	kControlEditUnicodeTextPasswordProc = 914;

{ Control Kind Tag }
const
	kControlKindEditUnicodeText = FourCharCode('eutx');

{ The HIObject class ID for the HITextField class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHITextFieldClassID CFSTRP('com.apple.HITextField')}
{$endc}
{
 *  CreateEditUnicodeTextControl()
 *  
 *  Summary:
 *    Creates a new edit text control.
 *  
 *  Discussion:
 *    This is the preferred edit text control. Use it instead of the
 *    EditText control. This control handles Unicode and draws its text
 *    using antialiasing, which the other control does not.
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
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateScrollingTextBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; contentResID: SInt16; autoScroll: Boolean; delayBeforeAutoScroll: UInt32; delayBetweenAutoScroll: UInt32; autoScrollAmount: UInt16; var outControl: ControlRef ): OSStatus; external name '_CreateScrollingTextBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
=======
function CreateScrollingTextBoxControl( window: WindowRef; const (*var*) boundsRect: Rect; contentResID: SInt16; autoScroll: Boolean; delayBeforeAutoScroll: UInt32; delayBetweenAutoScroll: UInt32; autoScrollAmount: UInt16; var outControl: ControlRef ): OSStatus; external name '_CreateScrollingTextBoxControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/cpstrnew
{ Tagged data supported by Scroll Text Box }
{$endc} {not TARGET_CPU_64}

const
<<<<<<< HEAD
	kControlScrollTextBoxDelayBeforeAutoScrollTag = FourCharCode('stdl'); { UInt32 (ticks until autoscrolling starts)}
	kControlScrollTextBoxDelayBetweenAutoScrollTag = FourCharCode('scdl'); { UInt32 (ticks between scrolls)}
	kControlScrollTextBoxAutoScrollAmountTag = FourCharCode('samt'); { UInt16 (pixels per scroll) -- defaults to 1}
	kControlScrollTextBoxContentsTag = FourCharCode('tres'); { SInt16 (resource ID of 'TEXT'/'styl') -- write only!}
	kControlScrollTextBoxAnimatingTag = FourCharCode('anim'); { Boolean (whether the text box should auto-scroll)}
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    window:
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    isPassword:
 *      A Boolean indicating whether the field is to be used as a
 *      password field. Passing false indicates that the field is to
 *      display entered text normally. True means that the field will
 *      be used as a password field and any text typed into the field
 *      will be displayed only as bullets.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control (if noErr is returned as the
 *      result code).
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateEditUnicodeTextControl( window: WindowRef; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; isPassword: Boolean; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateEditUnicodeTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
    The EditUnicodeText control supports these tags previously defined for the EditText control:
    
        kControlEditTextLockedTag
        kControlFontStyleTag
        kControlEditTextFixedTextTag
        kControlEditTextTextTag
        kControlEditTextKeyFilterTag
        kControlEditTextValidationProcTag
        kControlEditTextSelectionTag
        kControlEditTextKeyScriptBehaviorTag
        kControlEditTextCFStringTag
        kControlEditTextPasswordTag
        kControlEditTextPasswordCFStringTag
}
{ Tagged data supported by EditUnicodeText control only }
const
	kControlEditTextSingleLineTag = FourCharCode('sglc'); { data is a Boolean; indicates whether the control should always be single-line}
	kControlEditTextInsertTextBufferTag = FourCharCode('intx'); { data is an array of char; get or set the control's text as WorldScript-encoded text}
	kControlEditTextInsertCFStringRefTag = FourCharCode('incf'); { data is a CFStringRef; get or set the control's text as a CFStringRef. Caller should release CFString if getting.}
	kControlEditUnicodeTextPostUpdateProcTag = FourCharCode('upup'); { data is a UnicodePostUpdateUPP; get or set the post-update proc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Tagged data supported by Scroll Text Box }
{$endc} {not TARGET_CPU_64}

const
	kControlScrollTextBoxDelayBeforeAutoScrollTag = FourCharCode('stdl'); { UInt32 (ticks until autoscrolling starts)}
	kControlScrollTextBoxDelayBetweenAutoScrollTag = FourCharCode('scdl'); { UInt32 (ticks between scrolls)}
	kControlScrollTextBoxAutoScrollAmountTag = FourCharCode('samt'); { UInt16 (pixels per scroll) -- defaults to 1}
	kControlScrollTextBoxContentsTag = FourCharCode('tres'); { SInt16 (resource ID of 'TEXT'/'styl') -- write only!}
	kControlScrollTextBoxAnimatingTag = FourCharCode('anim'); { Boolean (whether the text box should auto-scroll)}
>>>>>>> origin/fixes_2.4

{$ifc OLDROUTINENAMES}
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ OLDROUTINENAMES                                                                   }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
	kControlCheckboxUncheckedValue = kControlCheckBoxUncheckedValue;
	kControlCheckboxCheckedValue = kControlCheckBoxCheckedValue;
	kControlCheckboxMixedValue = kControlCheckBoxMixedValue;

const
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	inLabel = kControlLabelPart;
	inMenu = kControlMenuPart;
	inTriangle = kControlTrianglePart;
	inButton = kControlButtonPart;
	inCheckBox = kControlCheckBoxPart;
	inUpButton = kControlUpButtonPart;
	inDownButton = kControlDownButtonPart;
	inPageUp = kControlPageUpPart;
	inPageDown = kControlPageDownPart;

const
	kInLabelControlPart = kControlLabelPart;
	kInMenuControlPart = kControlMenuPart;
	kInTriangleControlPart = kControlTrianglePart;
	kInButtonControlPart = kControlButtonPart;
	kInCheckBoxControlPart = kControlCheckBoxPart;
	kInUpButtonControlPart = kControlUpButtonPart;
	kInDownButtonControlPart = kControlDownButtonPart;
	kInPageUpControlPart = kControlPageUpPart;
	kInPageDownControlPart = kControlPageDownPart;


{$endc}  {OLDROUTINENAMES}


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
{$ALIGN MAC68K}


end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
