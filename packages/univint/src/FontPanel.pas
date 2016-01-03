{
     File:       CommonPanels/FontPanel.h
 
     Contains:   Carbon Font Panel package Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
 
     Copyright:  © 2002-2008 by Apple Computer, Inc., all rights reserved
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{  Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                     http://www.freepascal.org/bugs.html
 
}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
     Version:    CommonPanels-73~983
=======
     Version:    CommonPanels-91~177
>>>>>>> origin/fixes_2.4
 
     Copyright:  © 2002-2008 by Apple Computer, Inc., all rights reserved
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit FontPanel;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
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
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> origin/fixes_2.4
uses MacTypes, AEDataModel, AERegistry, ATSTypes, CarbonEventsCore, CFArray, CFBase, CFDictionary, QuickdrawTypes,CGBase;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
=======
uses MacTypes, AEDataModel, AERegistry, ATSTypes, CarbonEventsCore, CFArray, CFBase, CFDictionary, Quickdraw;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes, AEDataModel, AERegistry, ATSTypes, CarbonEventsCore, CFArray, CFBase, CFDictionary, Quickdraw;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}

{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Font Panel-Related Events
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}

{
 *  Discussion:
 *    Event classes
 }
const
{
   * Events related to font selection or handling.
   }
	kEventClassFont = FourCharCode('font');


{
 *  Summary:
 *    Common command IDs
 }
const
{
   * The state of the Font Panel should be toggled, displaying it or
   * hiding it as necessary. If the user closes the Font Panel directly
   * from the window, the application will receive a
   * kEventFontPanelClosed event.
   }
	kHICommandShowHideFontPanel = FourCharCode('shfp');

{ Font Events }

{
 *  Summary:
 *    Font events (kEventClassFont)
 *  
 *  Discussion:
 *    When the user closes the Font Panel, a kEventWindowClosed event
 *    will be detected by the Carbon event handler installed by the
 *    system. The system then notifies the application that the Font
 *    Panel has closed by posting a Carbon Event Manager event. This
 *    allows the application to update any menu items or other controls
 *    whose state may have to change because the Font Panel has closed.
 *    kEventWindowClosed has no parameters. When the user selects an
 *    item in the Font Panel, the system will send a
 *    kEventFontSelection event to the event target specified when the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    application called SetFontInfoForSelection(). kEventFontSelection
 *    will contain parameters reflecting the current Font Panel
 *    selection in all supported formats. Font events are available
 *    after Mac OS X 10.2 in the Carbon framework.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *    application called SetFontPanelInfo(). kEventFontSelection will
 *    contain parameters reflecting the current Font Panel selection in
 *    all supported formats. Font events are available after Mac OS X
 *    10.2 in the Carbon framework.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 }
const
{
   * The Font Panel has been closed. The application should update its
   * corresponding UI element (e.g., a menu item) accordingly.
   }
	kEventFontPanelClosed = 1;

  {
   * The user has specified font settings in the Font Panel. The
   * application can obtain these settings from the event, in which
   * they are stored as parameters. Not all parameters are guaranteed
   * to be present; the application should check for all those which it
   * recognizes and apply the ones found as appropriate to the target
   * text.
   }
	kEventFontSelection = 2;

{
    Parameters for font events:

    kEventFontPanelClosed
        None.
        
    kEventFontSelection
        -->     kEventParamATSUFontID               typeATSUFontID
        -->     kEventParamATSUFontSize             typeATSUSize
        -->     kEventParamFMFontFamily             typeFMFontFamily
        -->     kEventParamFMFontSize               typeFMFontSize
        -->     kEventParamFontColor                typeFontColor
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
        -->     kEventParamDictionary               typeCFDictionaryRef 
        -->     kEventParamViewAttributesDictionary typeCFDictionaryRef
                A dictionary containing attributes that can be applied to an entire text view.  An example of this is the background color to 
                apply to the view.
<<<<<<< HEAD
}
const
	typeCTFontDescriptorRef = typeCFTypeRef; { CTFontDescriptor reference.}
=======
        -->     kEventParamDictionary               typeCFDictionary 
}
const
>>>>>>> graemeg/fixes_2_2
=======
        -->     kEventParamDictionary               typeCFDictionary 
}
const
>>>>>>> origin/fixes_2_2
=======
}
const
	typeCTFontDescriptorRef = typeCFTypeRef; { CTFontDescriptor reference.}
>>>>>>> origin/fixes_2.4
	typeATSUFontID = typeUInt32; { ATSUI font ID.}
	typeATSUSize = typeFixed; { ATSUI font size.}
	typeFMFontFamily = typeSInt16; { Font family reference.}
	typeFMFontStyle = typeSInt16; { Quickdraw font style}
	typeFMFontSize = typeSInt16; { Integer font size.}
	typeFontColor = typeRGBColor; { Font color spec (optional).}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kEventParamCTFontDescriptor = FourCharCode('ctfd'); { typeCTFontDescriptorRef}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kEventParamCTFontDescriptor = FourCharCode('ctfd'); { typeCTFontDescriptorRef}
>>>>>>> origin/fixes_2.4
	kEventParamATSUFontID = FourCharCode('auid'); { typeATSUFontID}
	kEventParamATSUFontSize = FourCharCode('ausz'); { typeATSUSize}
	kEventParamFMFontFamily = FourCharCode('fmfm'); { typeFMFontFamily}
	kEventParamFMFontStyle = FourCharCode('fmst'); { typeFMFontStyle}
	kEventParamFMFontSize = FourCharCode('fmsz'); { typeFMFontSize}
	kEventParamFontColor = FourCharCode('fclr'); { typeFontColor}
	kEventParamDictionary = FourCharCode('dict'); {    typeCFDictionaryRef}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kEventParamViewAttributesDictionary = FourCharCode('dadc'); {    typeCFDictionaryRef}

{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Key constants to be used to access data inside the dictionary that may
    be contained in the kEventFontSelection dictionary. (kEventParamDictionary)
=======
=======
>>>>>>> origin/fixes_2_2

{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Key constants to be used to access data inside the dictionary that may
    be contained in the kEventFontSelection dictionary.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kEventParamViewAttributesDictionary = FourCharCode('dadc'); {    typeCFDictionaryRef}

{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Key constants to be used to access data inside the dictionary that may
    be contained in the kEventFontSelection dictionary. (kEventParamDictionary)
>>>>>>> origin/fixes_2.4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
{
 *  kFontPanelATSUFontIDKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelATSUFontIDKey: CFStringRef; external name '_kFontPanelATSUFontIDKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{Value is a CFNumber containing the ATSU Font ID}
{
 *  kFontPanelVariationAxesKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelVariationAxesKey: CFStringRef; external name '_kFontPanelVariationAxesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{ Value is a CFDataRef containing one or more ATSUI Variation Axes}
{
 *  kFontPanelVariationValuesKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelVariationValuesKey: CFStringRef; external name '_kFontPanelVariationValuesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{Value is a CFDataRef containing one or more ATSU Variation values}
{
 *  kFontPanelFeatureTypesKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelFeatureTypesKey: CFStringRef; external name '_kFontPanelFeatureTypesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{  Value is a CFDataRef containing one or more ATSUI feature types}
{
 *  kFontPanelFeatureSelectorsKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelFeatureSelectorsKey: CFStringRef; external name '_kFontPanelFeatureSelectorsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{  Value is a CFDataRef containing one or more ATSUI feature selectors}
{
 *  kFontPanelAttributesKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelAttributesKey: CFStringRef; external name '_kFontPanelAttributesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
  const <CFString> string kFontPanelAttributesKey           =   "FontAttributes";
    Value is a CFDictionaryRef containing three keyed values.  Each value is
    a CFDataRef.  One CFDataRef contains one or more ATSUAttributeTags.
    One CFDataRef contains one or more value sizes for each tag.  And the last
    CFDataRef contains the actual values.  It is important to understand that
    these are the actual values and not value ptrs.  To pass these values to
    ATSUI they must be converted into ptrs.  The following code fragment demonstrates
    one technique
    CFDataRef       values;
    CFDataRef       tags;
    CFDataRef       sizes;
    if (    CFDictionaryGetValueIfPresent( attributesDict, kFontPanelAttributeValuesKey, &values ) &&
            CFDictionaryGetValueIfPresent( attributesDict, kFontPanelAttributeTagsKey, &tags )
            CFDictionaryGetValueIfPresent( attributesDict, kFontPanelAttributeSizesKey, &sizes ))
    (
        ItemCount               count = CFDataGetLength( tags )/sizeof(ATSUAttributeTag);
        CFIndex                 index;
        ATSUAttributeValuePtr   valuePtrs = malloc( count * sizeof(ATSUAttributeValuePtr) );
        UInt32*                 sizePtr = (UInt32*)CFDataGetBytePtr(sizes);
        UInt32*                 bytePtr = (UInt32*)CFDataGetBytePtr(values);
        for ( index = 0; index < count; index++ )
        (
            valuePtrs[index] = bytePtr;
            bytePtr = (UInt32*)( (UInt8*)bytePtr + sizePtr[index]);
        )
        verify_noerr( ATSUSetAttributes( someATSUStyle, count, (ATSUAttributeTag*)CFDataGetBytePtr(tags),sizePtr, valuePtrs ) );
        free( valuePtrs );
}
{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Keys to access the CFDataRefs inside the attributes dictionary (see above)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
{
 *  kFontPanelAttributeTagsKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelAttributeTagsKey: CFStringRef; external name '_kFontPanelAttributeTagsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{Value is a CFDataRef containing one or more style attribute tags}
{
 *  kFontPanelAttributeSizesKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelAttributeSizesKey: CFStringRef; external name '_kFontPanelAttributeSizesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{Value is a CFDataRef containing one or more style attribute sizes}
{
 *  kFontPanelAttributeValuesKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelAttributeValuesKey: CFStringRef; external name '_kFontPanelAttributeValuesKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{Value is a CFDataRef containing one or more style values}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Keys to access to access the optional mouse tracking state if the font attribute/feature control is tracking
An application can look for this optional value to aid in supporting undo/redo for a font attribute/feature that is represented by
a control that tracks such as a slider.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
{
 *  kFontPanelMouseTrackingState
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelMouseTrackingState: CFStringRef; external name '_kFontPanelMouseTrackingState'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
{
   The value referenced by this key is a CFNumberRef that will contain one of the following values
  from CarbonEvents.h
   kEventMouseDown
   kEventMouseUp
   kEventMouseDragged
}

{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Keys to access the data from the document attributes dictionary (kEventParamViewAttributesDictionary)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
{
 *  kFontPanelBackgroundColorAttributeName
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFontPanelBackgroundColorAttributeName: CFStringRef; external name '_kFontPanelBackgroundColorAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Other Font Panel Constants
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
{
Error codes (Font Panel codes in range [-8880,-8899]).
}
const
	fontPanelShowErr = -8880; { Can't display the Font Panel.}
	fontPanelSelectionStyleErr = -8881; { Bad font selection style info.}
	fontPanelFontSelectionQDStyleVersionErr = -8882; { Unsupported record version.}

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Type of font information passed in SetFontInfoForSelection(). If the client is
=======
Type of font information passed in SetFontPanelInfo(). If the client is
>>>>>>> graemeg/fixes_2_2
=======
Type of font information passed in SetFontPanelInfo(). If the client is
>>>>>>> origin/fixes_2_2
=======
Type of font information passed in SetFontInfoForSelection(). If the client is
>>>>>>> origin/fixes_2.4
sending ATSUI style data, it specifies kFontSelectionATSUIType; if it is
sending Quickdraw style data, it specifies kFontSelectionQDType.
}
const
	kFontSelectionATSUIType = FourCharCode('astl'); { Use ATSUIStyle collection.}
	kFontSelectionQDType = FourCharCode('qstl'); { Use FontSelectionQDStyle record.}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kFontSelectionCoreTextType = FourCharCode('ctfd'); { Use CTFontDescriptorRef.}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kFontSelectionCoreTextType = FourCharCode('ctfd'); { Use CTFontDescriptorRef.}
>>>>>>> origin/fixes_2.4

{
Supported versions of the FontSelectionQDStyle record. Clients should always set
the <version> field to one of these values.
}
const
	kFontSelectionQDStyleVersionZero = 0;


{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Font Panel Types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
{
Record specifying the font information to be specified in the Font
Panel. This record is used if the client is sending Quickdraw style data
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(i.e., it specified kFontSelectionQDType in SetFontInfoForSelection()).
=======
(i.e., it specified kFontSelectionQDType in SetFontPanelInfo()).
>>>>>>> graemeg/fixes_2_2
=======
(i.e., it specified kFontSelectionQDType in SetFontPanelInfo()).
>>>>>>> origin/fixes_2_2
=======
(i.e., it specified kFontSelectionQDType in SetFontInfoForSelection()).
>>>>>>> origin/fixes_2.4
}
type
	FontSelectionQDStyle = record
		version: UInt32;                { Version number of struct.}
		instance: FMFontFamilyInstance;             { Font instance data.}
		size: FMFontSize;                   { Size of font in points.}
		hasColor: Boolean;               { true if color info supplied.}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		reserved: UInt8;               { Filler byte.}
=======
		reserved: SInt8;               { Filler byte.}
>>>>>>> graemeg/fixes_2_2
=======
		reserved: SInt8;               { Filler byte.}
>>>>>>> origin/fixes_2_2
=======
		reserved: UInt8;               { Filler byte.}
>>>>>>> origin/fixes_2.4
		color: RGBColor;                  { Color specification for font.}
	end;
	FontSelectionQDStylePtr = ^FontSelectionQDStyle;
{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Font Panel Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
{
 *  FPIsFontPanelVisible()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function FPIsFontPanelVisible: Boolean; external name '_FPIsFontPanelVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FPShowHideFontPanel()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function FPShowHideFontPanel: OSStatus; external name '_FPShowHideFontPanel';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  SetFontInfoForSelection()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SetFontInfoForSelection( iStyleType: OSType; iNumStyles: UInt32; iStyles: UnivPtr; iFPEventTarget: EventTargetRef ): OSStatus; external name '_SetFontInfoForSelection';
=======
function SetFontInfoForSelection( iStyleType: OSType; iNumStyles: UInt32; iStyles: {variable-size-array} UnivPtr; iFPEventTarget: EventTargetRef ): OSStatus; external name '_SetFontInfoForSelection';
>>>>>>> graemeg/fixes_2_2
=======
function SetFontInfoForSelection( iStyleType: OSType; iNumStyles: UInt32; iStyles: {variable-size-array} UnivPtr; iFPEventTarget: EventTargetRef ): OSStatus; external name '_SetFontInfoForSelection';
>>>>>>> origin/fixes_2_2
=======
function SetFontInfoForSelection( iStyleType: OSType; iNumStyles: UInt32; iStyles: UnivPtr; iFPEventTarget: EventTargetRef ): OSStatus; external name '_SetFontInfoForSelection';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Font Collection Functions

    In 10.3 the ability to create and modify font collections is available.  Font
    collections are files containing font descriptions.  Font descriptions are
    encapsulated in the opaque object FCFontDescriptorRef. A FCFontDescriptroRef
    is a CFType.  To release one call CFRelease.
    
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Font Collection Types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	FCFontDescriptorRef = ^OpaqueFCFontDescriptorRef; { an opaque type }
	OpaqueFCFontDescriptorRef = record end;
=======
	FCFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	FCFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	FCFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	FCFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	FCFontDescriptorRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	FCFontDescriptorRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	FCFontDescriptorRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
{
 *  FCCopyCollectionNames()
 *  
 *  Discussion:
 *    FCCopyCollectionNames returns a copy of the CFArrayRef containing
 *    the displayable names of every font collection available to the
 *    current user.
 *  
 *  Result:
 *    A CFArrayRef containing CFStringRefs where each CFStringRef
 *    contains a font collection's displayable name. Callers are
 *    responsible for releasing the returned CFArrayRef.  If the
 *    operation is not successful NULL is returned.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FCCopyCollectionNames: CFArrayRef; external name '_FCCopyCollectionNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  FCCopyFontDescriptorsInCollection()
 *  
 *  Discussion:
 *    FCCopyFontDescriptorsInCollection copies the fontDescriptors in a
 *    named collection into an array.
 *  
 *  Parameters:
 *    
 *    iCollection:
 *      The name of a collection that descriptors should be copied from.
 *  
 *  Result:
 *    A CFArrayRef containing copies of the FCFontDescriptorRefs
 *    contained in the name collection.  Callers are responsible for
 *    releasing the returned CFArrayRef.  The FCFontDescriptorRefs are
 *    retained when added to the array and released when the array is
 *    destroyed.  You can access a font descriptor in the array in the
 *    following manner: fd =
 *    (FCFontDescriptorRef)CFArrayGetValueAtIndex(array, i);
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FCCopyFontDescriptorsInCollection( iCollection: CFStringRef ): CFArrayRef; external name '_FCCopyFontDescriptorsInCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  FCAddCollection()
 *  
 *  Discussion:
 *    Add a collection to the font descriptor collections available to
 *    the current user. If the collection is successfully added noErr
 *    is returned.  If the collection is not added an error code is
 *    returned.
 *  
 *  Parameters:
 *    
 *    iCollection:
 *      the name of the collection to add.
 *    
 *    iCollectionOptions:
 *      currently there are no options.  Set to kNilOptions.  This
 *      parameter is available for future expansion.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FCAddCollection( iCollection: CFStringRef; iCollectionOptions: OptionBits ): OSStatus; external name '_FCAddCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  FCRemoveCollection()
 *  
 *  Discussion:
 *    Remove a named collection from the font descriptor collections
 *    available to the current user. Returns noErr if the collection
 *    was successfully removed.  An appropriate error code is returned
 *    if the operation was not successful.
 *  
 *  Parameters:
 *    
 *    iCollection:
 *      the name of the collection to remove.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FCRemoveCollection( iCollection: CFStringRef ): OSStatus; external name '_FCRemoveCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  FCAddFontDescriptorToCollection()
 *  
 *  Discussion:
 *    Add a font descriptor to the named collection.  noErr is returned
 *    if the font descriptor is added. An error code describing the
 *    failure is returned if the descriptor is not added.
 *  
 *  Parameters:
 *    
 *    iDescriptor:
 *      the font descriptor that should be added.  The
 *      FCFontDescriptorRef is retained when it is added to the
 *      collection.  After calling this function the caller may release
 *      their copy.
 *    
 *    iCollection:
 *      the name of the collection to which the font descriptor should
 *      be added.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FCAddFontDescriptorToCollection( iDescriptor: FCFontDescriptorRef; iCollection: CFStringRef ): OSStatus; external name '_FCAddFontDescriptorToCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  FCRemoveFontDescriptorFromCollection()
 *  
 *  Discussion:
 *    Remove a font descriptor from the named collection.  An error is
 *    returned if the font descriptor can not be removed.  noErr is
 *    returned if the descriptor is removed.
 *  
 *  Parameters:
 *    
 *    iDescriptor:
 *      the descriptor that should be removed.
 *    
 *    iCollection:
 *      the name of the collection that the descriptor should be
 *      removed from.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FCRemoveFontDescriptorFromCollection( iDescriptor: FCFontDescriptorRef; iCollection: CFStringRef ): OSStatus; external name '_FCRemoveFontDescriptorFromCollection';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    Font Descriptor Attribute Keys
    
    Font Descriptors contain font attributes that are set and accessed via a set of 
    keys.  The keys are all constant CFStringRefs.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~}

{
 *  kFCFontFamilyAttribute
 *  
 *  Discussion:
 *    The key for a CFStringRef that contains a font family name (e.g.
 *    Baskerville).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFCFontFamilyAttribute: CFStringRef; external name '_kFCFontFamilyAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kFCFontNameAttribute
 *  
 *  Discussion:
 *    The key for a CFStringRef containing a font name (e.g.
 *    Baskerville-Italic).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFCFontNameAttribute: CFStringRef; external name '_kFCFontNameAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kFCFontFaceAttribute
 *  
 *  Discussion:
 *    The key for a CFStringRef containing a face name (e.g. Italic).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFCFontFaceAttribute: CFStringRef; external name '_kFCFontFaceAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kFCFontSizeAttribute
 *  
 *  Discussion:
 *    The key for a CFNumber containg the font size (e.g. 12).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFCFontSizeAttribute: CFStringRef; external name '_kFCFontSizeAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kFCFontVisibleNameAttribute
 *  
 *  Discussion:
 *    The Key for a CFStringRef containing the name that should be used
 *    in a UI to describe the font.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFCFontVisibleNameAttribute: CFStringRef; external name '_kFCFontVisibleNameAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kFCFontCGColorAttribute
 *  
 *  Discussion:
 *    The Key for a CGColorRef containing the fonts color.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kFCFontCGColorAttribute: CFStringRef; external name '_kFCFontCGColorAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  FCFontDescriptorCreateWithFontAttributes()
 *  
 *  Discussion:
 *    Create a font descriptor using the attributes contained in the
 *    dictionary.
 *  
 *  Parameters:
 *    
 *    iAttributes:
 *      a dictionary containing one or more of the attributes described
 *      above.
 *  
 *  Result:
 *    A valid FCFontDescriptorRef.  NULL if one cannot be created.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FCFontDescriptorCreateWithFontAttributes( iAttributes: CFDictionaryRef ): FCFontDescriptorRef; external name '_FCFontDescriptorCreateWithFontAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  FCFontDescriptorCreateWithName()
 *  
 *  Discussion:
 *    Create a font descriptor using a fontname and font size.
 *  
 *  Parameters:
 *    
 *    iFontName:
 *      The name of the font (e.g. Baskerville-Italic).
 *    
 *    iSize:
 *      the size of the font. (e.g. 12.0).
 *  
 *  Result:
 *    A valid FCFontDescriptorRef.  NULL if one cannot be created.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function FCFontDescriptorCreateWithName( iFontName: CFStringRef; iSize: CGFloat ): FCFontDescriptorRef; external name '_FCFontDescriptorCreateWithName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
function FCFontDescriptorCreateWithName( iFontName: CFStringRef; iSize: Float32 ): FCFontDescriptorRef; external name '_FCFontDescriptorCreateWithName';
=======
function FCFontDescriptorCreateWithName( iFontName: CFStringRef; iSize: CGFloat ): FCFontDescriptorRef; external name '_FCFontDescriptorCreateWithName';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


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
