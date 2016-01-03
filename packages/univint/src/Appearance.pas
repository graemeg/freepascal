{
     File:       HIToolbox/Appearance.h
 
     Contains:   Appearance Manager Interfaces.
 
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
 
     Copyright:  © 1994-2008 by Apple Computer, Inc., all rights reserved
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1994-2005 by Apple Computer, Inc., all rights reserved
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1994-2005 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 1994-2008 by Apple Computer, Inc., all rights reserved
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

unit Appearance;
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
uses MacTypes,CFBase,CGContext,Collections,QuickdrawText,QuickdrawTypes,MacErrors;
{$endc} {not MACOSALLINCLUDE}

=======
uses MacTypes,CFBase,CGContext,Collections,Processes,QuickdrawText,TextCommon,Quickdraw,TextEdit,QDOffscreen,MacErrors,TextUtils,CFString;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CGContext,Collections,Processes,QuickdrawText,TextCommon,Quickdraw,TextEdit,QDOffscreen,MacErrors,TextUtils,CFString;
>>>>>>> origin/fixes_2_2
=======
uses MacTypes,CFBase,CGContext,Collections,QuickdrawText,QuickdrawTypes,MacErrors;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Appearance Manager constants, etc.                                               }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Appearance Manager Apple Events (1.1 and later)              }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}

const
	kAppearanceEventClass = FourCharCode('appr'); { Event Class }
	kAEAppearanceChanged = FourCharCode('thme'); { Appearance changed (e.g. platinum to hi-tech) }
	kAESystemFontChanged = FourCharCode('sysf'); { system font changed }
	kAESmallSystemFontChanged = FourCharCode('ssfn'); { small system font changed }
	kAEViewsFontChanged = FourCharCode('vfnt'); { views font changed }

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Appearance Manager file types                                                    }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeDataFileType = FourCharCode('thme'); { file type for theme files }
	kThemePlatinumFileType = FourCharCode('pltn'); { file type for platinum appearance }
	kThemeCustomThemesFileType = FourCharCode('scen'); { file type for user themes }
	kThemeSoundTrackFileType = FourCharCode('tsnd');

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Appearance Manager Supported Themes                                              }
{ Use CopyThemeIdentifier to get the current theme ID                              }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kThemeAppearancePlatinum CFSTRP('com.apple.theme.appearance.platinum')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kThemeAppearanceAqua CFSTRP('com.apple.theme.appearance.aqua')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kThemeAppearanceAquaBlue CFSTRP('com.apple.theme.appearance.aqua.blue')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kThemeAppearanceAquaGraphite CFSTRP('com.apple.theme.appearance.aqua.graphite')}
{$endc}


{
 *  AppearancePartCode
 *  
 *  Summary:
 *    These are part codes returned by a few of the hit testing
 *    Appearance APIs. Many of the Control Manager's ControlPartCodes
 *    are based on these part codes.
 }
type
	AppearancePartCode = SInt16;
const
{
   * This represents the lack of a part. It will be returned when the
   * Appearance Manager's hit testing logic determines that the input
   * point is not in any part of the widget.
   }
	kAppearancePartMetaNone = 0;

  {
   * This represents a widget which is not currently clickable because
   * it is disabled.
   }
	kAppearancePartMetaDisabled = 254;

  {
   * This represents a widget which is inactive, presumably because it
   * is in a window that is inactive.
   }
	kAppearancePartMetaInactive = 255;

  {
   * The part of a widget which indicates the widget's value. Scroll
   * bar thumbs and slider thumbs are the two main examples.
   }
	kAppearancePartIndicator = 129;

  {
   * The part of a widget which moves its value visually upward. Scroll
   * bar up arrows are the main example.
   }
	kAppearancePartUpButton = 20;

  {
   * The part of a widget which moves its value visually downward.
   * Scroll bar down arrows are the main example.
   }
	kAppearancePartDownButton = 21;

  {
   * The part of a widget which moves its value visually leftward.
   * Scroll bar left arrows are the main example.
   }
	kAppearancePartLeftButton = kAppearancePartUpButton;

  {
   * The part of a widget which moves its value visually rightward.
   * Scroll bar right arrows are the main example.
   }
	kAppearancePartRightButton = kAppearancePartDownButton;

  {
   * The part of a widget which moves its value visually upward one
   * whole page. Scroll bar page up areas are the main example.
   }
	kAppearancePartPageUpArea = 22;

  {
   * The part of a widget which moves its value visually downward one
   * whole page. Scroll bar page down areas are the main example.
   }
	kAppearancePartPageDownArea = 23;

  {
   * The part of a widget which moves its value visually leftward one
   * whole page. Scroll bar page left areas are the main example.
   }
	kAppearancePartPageLeftArea = kAppearancePartPageUpArea;

  {
   * The part of a widget which moves its value visually rightward one
   * whole page. Scroll bar page right areas are the main example.
   }
	kAppearancePartPageRightArea = kAppearancePartPageDownArea;


{
 *  AppearanceRegionCode
 *  
 *  Summary:
 *    These are region codes used by a few of window-related Appearance
 *    APIs. Many of the Window Manager's WindowRegionCodes are based on
 *    these region codes.
 }
type
	AppearanceRegionCode = UInt16;
const
	kAppearanceRegionTitleBar = 0;
	kAppearanceRegionTitleText = 1;
	kAppearanceRegionCloseBox = 2;
	kAppearanceRegionZoomBox = 3;
	kAppearanceRegionDrag = 5;
	kAppearanceRegionGrow = 6;
	kAppearanceRegionCollapseBox = 7;
	kAppearanceRegionTitleProxyIcon = 8;  { Mac OS 8.5 forward}
	kAppearanceRegionStructure = 32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAppearanceRegionContent = 33;   { Content area of the window; empty when the window is collapsed}
	kAppearanceRegionToolbarButton = 41;
=======
	kAppearanceRegionContent = 33;    { Content area of the window; empty when the window is collapsed}
>>>>>>> graemeg/fixes_2_2
=======
	kAppearanceRegionContent = 33;    { Content area of the window; empty when the window is collapsed}
>>>>>>> origin/fixes_2_2
=======
	kAppearanceRegionContent = 33;   { Content area of the window; empty when the window is collapsed}
	kAppearanceRegionToolbarButton = 41;
>>>>>>> origin/fixes_2.4


{
 *  Discussion:
 *    ThemeBrushes
 }
const
{
   * Use with kModalWindowClass.
   }
	kThemeBrushDialogBackgroundActive = 1;

  {
   * Use with kModalWindowClass.
   }
	kThemeBrushDialogBackgroundInactive = 2;

  {
   * Use with kAlertWindowClass and kMovableAlertWindowClass.
   }
	kThemeBrushAlertBackgroundActive = 3;

  {
   * Use with kAlertWindowClass and kMovableAlertWindowClass.
   }
	kThemeBrushAlertBackgroundInactive = 4;

  {
   * Use with kDocumentWindowClass.
   }
	kThemeBrushModelessDialogBackgroundActive = 5;

  {
   * Use with kDocumentWindowClass.
   }
	kThemeBrushModelessDialogBackgroundInactive = 6;

  {
   * Use with kFloatingWindowClass and kUtilityWindowClass.
   }
	kThemeBrushUtilityWindowBackgroundActive = 7;

  {
   * Use with kFloatingWindowClass and kUtilityWindowClass.
   }
	kThemeBrushUtilityWindowBackgroundInactive = 8;

  {
   * The background used for Finder list views.
   }
	kThemeBrushListViewSortColumnBackground = 9;
	kThemeBrushListViewBackground = 10;
	kThemeBrushIconLabelBackground = 11;
	kThemeBrushListViewSeparator = 12;
	kThemeBrushChasingArrows = 13;
	kThemeBrushDragHilite = 14;

  {
   * Use with kDocumentWindowClass.
   }
	kThemeBrushDocumentWindowBackground = 15;
	kThemeBrushFinderWindowBackground = 16;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushScrollBarDelimiterActive = 17;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushScrollBarDelimiterInactive = 18;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushFocusHighlight = 19;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushPopupArrowActive = 20;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushPopupArrowPressed = 21;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushPopupArrowInactive = 22;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushAppleGuideCoachmark = 23;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushIconLabelBackgroundSelected = 24;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushStaticAreaFill = 25;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushActiveAreaFill = 26;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFrameActive = 27;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFrameInactive = 28;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFaceActive = 29;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFaceInactive = 30;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFacePressed = 31;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonActiveDarkShadow = 32;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonActiveDarkHighlight = 33;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonActiveLightShadow = 34;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonActiveLightHighlight = 35;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonInactiveDarkShadow = 36;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonInactiveDarkHighlight = 37;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonInactiveLightShadow = 38;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonInactiveLightHighlight = 39;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonPressedDarkShadow = 40;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonPressedDarkHighlight = 41;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonPressedLightShadow = 42;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonPressedLightHighlight = 43;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushBevelActiveLight = 44;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushBevelActiveDark = 45;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushBevelInactiveLight = 46;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushBevelInactiveDark = 47;

  {
   * Available in Appearance 1.1.1 (Mac OS 9.0) and later.
   }
	kThemeBrushNotificationWindowBackground = 48;

  {
=======
=======
>>>>>>> origin/fixes_2_2

{
 *  Discussion:
 *    ThemeBrushes introduced in Appearance 1.1 (Mac OS 8.5) and later
 }
const
=======
  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
>>>>>>> origin/fixes_2.4
	kThemeBrushScrollBarDelimiterActive = 17;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushScrollBarDelimiterInactive = 18;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushFocusHighlight = 19;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushPopupArrowActive = 20;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushPopupArrowPressed = 21;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushPopupArrowInactive = 22;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushAppleGuideCoachmark = 23;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushIconLabelBackgroundSelected = 24;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushStaticAreaFill = 25;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushActiveAreaFill = 26;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFrameActive = 27;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFrameInactive = 28;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFaceActive = 29;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFaceInactive = 30;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonFacePressed = 31;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonActiveDarkShadow = 32;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonActiveDarkHighlight = 33;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonActiveLightShadow = 34;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonActiveLightHighlight = 35;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonInactiveDarkShadow = 36;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonInactiveDarkHighlight = 37;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonInactiveLightShadow = 38;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonInactiveLightHighlight = 39;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonPressedDarkShadow = 40;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonPressedDarkHighlight = 41;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonPressedLightShadow = 42;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushButtonPressedLightHighlight = 43;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushBevelActiveLight = 44;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushBevelActiveDark = 45;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushBevelInactiveLight = 46;

  {
   * Available in Appearance 1.1 (Mac OS 8.5) and later.
   }
	kThemeBrushBevelInactiveDark = 47;

  {
   * Available in Appearance 1.1.1 (Mac OS 9.0) and later.
   }
	kThemeBrushNotificationWindowBackground = 48;

<<<<<<< HEAD

{
 *  Discussion:
 *    ThemeBrushes introduced in Carbon. Available in Mac OS X, and
 *    Carbon Lib 1.3 and later.
 }
const
{
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  {
>>>>>>> origin/fixes_2.4
   * Use with kMovableModalWindowClass windows. Available in Mac OS X,
   * and CarbonLib 1.3 and later.
   }
	kThemeBrushMovableModalBackground = 49;

  {
   * Use with kSheetWindowClass and kSheetAlertWindowClass. This is the
   * backwardly compatible sheet background.
   * kThemeBrushSheetBackgroundTransparent is preferred. Available in
   * Mac OS X, and CarbonLib 1.3 and later.
   }
	kThemeBrushSheetBackgroundOpaque = 50;

  {
   * Use with kDrawerWindowClass. Available in Mac OS X, and CarbonLib
   * 1.3 and later.
   }
	kThemeBrushDrawerBackground = 51;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {
   * Use with kToolbarWindowClass. Available in Mac OS X, and Carbon
   * Lib 1.6 and later.
   }
	kThemeBrushToolbarBackground = 52;

  {
=======
=======
>>>>>>> origin/fixes_2_2

{
 *  Discussion:
 *    ThemeBrushes introduced in Carbon. Available in Mac OS X, and
 *    Carbon Lib 1.6 and later.
 }
const
{
   * Use with kToolbarWindowClass.
   }
	kThemeBrushToolbarBackground = 52;


{
 *  Discussion:
 *    ThemeBrushes introduced in Mac OS X 10.1. Available in Mac OS X
 *    10.1 and CarbonLib 1.6, and later.
 }
const
{
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  {
   * Use with kToolbarWindowClass. Available in Mac OS X, and Carbon
   * Lib 1.6 and later.
   }
	kThemeBrushToolbarBackground = 52;

  {
>>>>>>> origin/fixes_2.4
   * Use with kSheetWindowClass and kSheetAlertWindowClass. Not fully
   * transparent -- this brush is the semi-transparent background seen
   * with modern sheets. Available in Mac OS X 10.1 and CarbonLib 1.6,
   * and later.
   }
	kThemeBrushSheetBackgroundTransparent = 53;

  {
   * Available in Mac OS X 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushMenuBackground = 54;

  {
   * Available in Mac OS X 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushMenuBackgroundSelected = 55;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {
   * For lists that use different colors as the background for odd and
   * even rows (like iTunes), this brush is the background for the odd
   * numbered rows. Available in Mac OS X 10.3 and later.
=======
=======
>>>>>>> origin/fixes_2_2

{
 *  Discussion:
 *    ThemeBrushes introduced in Mac OS X 10.3.
 }
const
{
   * For lists that use different colors as the background for odd and
   * even rows (like iTunes), this brush is the background for the odd
   * numbered rows. Available in Mac OS X 10.4 and later.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  {
   * For lists that use different colors as the background for odd and
   * even rows (like iTunes), this brush is the background for the odd
   * numbered rows. Available in Mac OS X 10.3 and later.
>>>>>>> origin/fixes_2.4
   }
	kThemeBrushListViewOddRowBackground = 56;

  {
   * For lists that use different colors as the background for odd and
   * even rows (like iTunes), this brush is the background for the even
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * numbered rows. Available in Mac OS X 10.3 and later.
=======
   * numbered rows. Available in Mac OS X 10.4 and later.
>>>>>>> graemeg/fixes_2_2
=======
   * numbered rows. Available in Mac OS X 10.4 and later.
>>>>>>> origin/fixes_2_2
=======
   * numbered rows. Available in Mac OS X 10.3 and later.
>>>>>>> origin/fixes_2.4
   }
	kThemeBrushListViewEvenRowBackground = 57;

  {
   * The color for the divider lines drawn between columns in some list
   * views. This brush may have an alpha value associated with it.
   * Drawing this brush with QuickDraw or fetching it as an RGBColor
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * will not necessarily yield satisfactory results. Available in Mac
   * OS X 10.3 and later.
   }
	kThemeBrushListViewColumnDivider = 58;

  {
   * kThemeBrushSheetBackgroundOpaque synonym.
   * kThemeBrushSheetBackgroundOpaque is preferred
   }
	kThemeBrushSheetBackground = kThemeBrushSheetBackgroundOpaque;

  {
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
   * Available in Mac OS 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushBlack = -1;

  {
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
   * Available in Mac OS 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushWhite = -2;

  {
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
=======
=======
>>>>>>> origin/fixes_2_2
   * will not necessarily yield satisfactory results.
=======
   * will not necessarily yield satisfactory results. Available in Mac
   * OS X 10.3 and later.
>>>>>>> origin/fixes_2.4
   }
	kThemeBrushListViewColumnDivider = 58;

  {
   * kThemeBrushSheetBackgroundOpaque synonym.
   * kThemeBrushSheetBackgroundOpaque is preferred
   }
	kThemeBrushSheetBackground = kThemeBrushSheetBackgroundOpaque;

  {
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
   * Available in Mac OS 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushBlack = -1;

  {
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
   * Available in Mac OS 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushWhite = -2;

  {
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
>>>>>>> origin/fixes_2.4
   * Available in Mac OS 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushPrimaryHighlightColor = -3;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
>>>>>>> origin/fixes_2.4
   * Available in Mac OS 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushSecondaryHighlightColor = -4;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
   * This is a theme meta-brush. It is a specific color that does not
   * change from theme to theme. Use it instead of a direct RGB value.
>>>>>>> origin/fixes_2.4
   * Available in Mac OS 10.1 and CarbonLib 1.6, and later.
   }
	kThemeBrushAlternatePrimaryHighlightColor = -5;

type
	ThemeBrush = SInt16;
const
	kThemeTextColorDialogActive = 1;
	kThemeTextColorDialogInactive = 2;
	kThemeTextColorAlertActive = 3;
	kThemeTextColorAlertInactive = 4;
	kThemeTextColorModelessDialogActive = 5;
	kThemeTextColorModelessDialogInactive = 6;
	kThemeTextColorWindowHeaderActive = 7;
	kThemeTextColorWindowHeaderInactive = 8;
	kThemeTextColorPlacardActive = 9;
	kThemeTextColorPlacardInactive = 10;
	kThemeTextColorPlacardPressed = 11;
	kThemeTextColorPushButtonActive = 12;
	kThemeTextColorPushButtonInactive = 13;
	kThemeTextColorPushButtonPressed = 14;
	kThemeTextColorBevelButtonActive = 15;
	kThemeTextColorBevelButtonInactive = 16;
	kThemeTextColorBevelButtonPressed = 17;
	kThemeTextColorPopupButtonActive = 18;
	kThemeTextColorPopupButtonInactive = 19;
	kThemeTextColorPopupButtonPressed = 20;
	kThemeTextColorIconLabel = 21;
	kThemeTextColorListView = 22;

{ Text Colors available in Appearance 1.0.1 or later }
const
	kThemeTextColorDocumentWindowTitleActive = 23;
	kThemeTextColorDocumentWindowTitleInactive = 24;
	kThemeTextColorMovableModalWindowTitleActive = 25;
	kThemeTextColorMovableModalWindowTitleInactive = 26;
	kThemeTextColorUtilityWindowTitleActive = 27;
	kThemeTextColorUtilityWindowTitleInactive = 28;
	kThemeTextColorPopupWindowTitleActive = 29;
	kThemeTextColorPopupWindowTitleInactive = 30;
	kThemeTextColorRootMenuActive = 31;
	kThemeTextColorRootMenuSelected = 32;
	kThemeTextColorRootMenuDisabled = 33;
	kThemeTextColorMenuItemActive = 34;
	kThemeTextColorMenuItemSelected = 35;
	kThemeTextColorMenuItemDisabled = 36;
	kThemeTextColorPopupLabelActive = 37;
	kThemeTextColorPopupLabelInactive = 38;


{ Text colors available in Appearance 1.1 or later }
const
	kThemeTextColorTabFrontActive = 39;
	kThemeTextColorTabNonFrontActive = 40;
	kThemeTextColorTabNonFrontPressed = 41;
	kThemeTextColorTabFrontInactive = 42;
	kThemeTextColorTabNonFrontInactive = 43;
	kThemeTextColorIconLabelSelected = 44;
	kThemeTextColorBevelButtonStickyActive = 45;
	kThemeTextColorBevelButtonStickyInactive = 46;

{ Text colors available in Appearance 1.1.1 or later }
const
	kThemeTextColorNotification = 47;


{ Text colors only available later than OS X 10.1.3 }
const
	kThemeTextColorSystemDetail = 48;

{ These values are specific colors that do not change from             }
{ theme to theme. You can use them instead of using direct RGB values. }
const
	kThemeTextColorBlack = -1;
	kThemeTextColorWhite = -2;

type
	ThemeTextColor = SInt16;
{ States to draw primitives: disabled, active, and pressed (hilited) }
const
	kThemeStateInactive = 0;
	kThemeStateActive = 1;
	kThemeStatePressed = 2;
	kThemeStateRollover = 6;
	kThemeStateUnavailable = 7;
	kThemeStateUnavailableInactive = 8;

{ obsolete name }
const
	kThemeStateDisabled = 0;

const
	kThemeStatePressedUp = 2;    { draw with up pressed     (increment/decrement buttons) }
	kThemeStatePressedDown = 3;     { draw with down pressed (increment/decrement buttons) }

type
	ThemeDrawState = UInt32;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Theme cursor selectors available in Appearance 1.1 or later                      }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

const
	kThemeArrowCursor = 0;
	kThemeCopyArrowCursor = 1;
	kThemeAliasArrowCursor = 2;
	kThemeContextualMenuArrowCursor = 3;
	kThemeIBeamCursor = 4;
	kThemeCrossCursor = 5;
	kThemePlusCursor = 6;
	kThemeWatchCursor = 7;    { Can Animate }
	kThemeClosedHandCursor = 8;
	kThemeOpenHandCursor = 9;
	kThemePointingHandCursor = 10;
	kThemeCountingUpHandCursor = 11;   { Can Animate }
	kThemeCountingDownHandCursor = 12;   { Can Animate }
	kThemeCountingUpAndDownHandCursor = 13; { Can Animate }
	kThemeSpinningCursor = 14;   { Can Animate }
	kThemeResizeLeftCursor = 15;
	kThemeResizeRightCursor = 16;
	kThemeResizeLeftRightCursor = 17;
	kThemeNotAllowedCursor = 18;   { available on Mac OS X 10.2 and later }

  {
   * Available in Mac OS X 10.3 or later.
   }
	kThemeResizeUpCursor = 19;

  {
   * Available in Mac OS X 10.3 or later.
   }
	kThemeResizeDownCursor = 20;

  {
   * Available in Mac OS X 10.3 or later.
   }
	kThemeResizeUpDownCursor = 21;

  {
   * A special cursor to indicate that letting up the mouse will cause
   * a dragged item to go away. When the item goes away, a poof cloud
   * animation should occur. This cursor should be updated dynamically
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * depending on whether the mouse up action will remove the item.
=======
   * dependeding on whether the mouse up action will remove the item.
>>>>>>> graemeg/fixes_2_2
=======
   * dependeding on whether the mouse up action will remove the item.
>>>>>>> origin/fixes_2_2
=======
   * depending on whether the mouse up action will remove the item.
>>>>>>> origin/fixes_2.4
   * Available in Mac OS X 10.3 or later.
   }
	kThemePoofCursor = 22;

type
	ThemeCursor = UInt32;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Theme menu bar drawing states                                                    }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeMenuBarNormal = 0;
	kThemeMenuBarSelected = 1;

type
	ThemeMenuBarState = UInt16;
{ attributes }
const
	kThemeMenuSquareMenuBar = 1 shl 0;

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Theme menu drawing states                                                        }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeMenuActive = 0;
	kThemeMenuSelected = 1;
	kThemeMenuDisabled = 3;

type
	ThemeMenuState = UInt16;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ MenuType: add kThemeMenuTypeInactive to menu type for DrawThemeMenuBackground if entire  }
{ menu is inactive                                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeMenuTypePullDown = 0;
	kThemeMenuTypePopUp = 1;
	kThemeMenuTypeHierarchical = 2;
	kThemeMenuTypeInactive = $0100;

type
	ThemeMenuType = UInt16;
const
	kThemeMenuItemPlain = 0;
	kThemeMenuItemHierarchical = 1;    { item has hierarchical arrow}
	kThemeMenuItemScrollUpArrow = 2;    { for scrollable menus, indicates item is scroller}
	kThemeMenuItemScrollDownArrow = 3;
	kThemeMenuItemAtTop = $0100; { indicates item is being drawn at top of menu}
	kThemeMenuItemAtBottom = $0200; { indicates item is being drawn at bottom of menu}
	kThemeMenuItemHierBackground = $0400; { item is within a hierarchical menu}
	kThemeMenuItemPopUpBackground = $0800; { item is within a popped up menu}
	kThemeMenuItemHasIcon = $8000; { add into non-arrow type when icon present}
	kThemeMenuItemNoBackground = $4000; { don't draw the menu background while drawing this item (Mac OS X only)}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kThemeMenuItemAlignRight = $2000; { right-align the content of this menu item (SnowLeopard and later)}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kThemeMenuItemAlignRight = $2000; { right-align the content of this menu item (SnowLeopard and later)}
>>>>>>> origin/fixes_2.4

type
	ThemeMenuItemType = UInt16;

{
 *  Discussion:
 *    ThemeBackgroundKinds
 }
const
{
   * The theme background used to draw the fill of a tab pane.
   }
	kThemeBackgroundTabPane = 1;

  {
   * The theme background used to draw the fill of a window placard.
   }
	kThemeBackgroundPlacard = 2;

  {
   * The theme background used to draw the window header of a window of
   * a window that does not contain content that has a hard-lined top
   * edge. An example of this would be icon view in the non side panel
   * Finder.
   }
	kThemeBackgroundWindowHeader = 3;

  {
   * The theme background used to draw the window header of a window of
   * a window that does contain content that has a hard-lined top edge.
   * An example of this would be list view in the non side panel Finder.
   }
	kThemeBackgroundListViewWindowHeader = 4;

  {
   * The theme background used to draw the fill of a secondary group
   * box.
   }
	kThemeBackgroundSecondaryGroupBox = 5;

  {
   * A special theme brush for drawing metal backgrounds. Currently,
   * this brush only works with HIThemeDrawBackground.
   }
	kThemeBackgroundMetal = 6;

type
	ThemeBackgroundKind = UInt32;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Theme Collection tags for Get/SetTheme                                                   }
{  X ALERT: Please note that Get/SetTheme are severely neutered under Mac OS X at present. }
{           The first group of tags below are available to get under both 9 and X. The     }
{           second group is 9 only. None of the tags can be used in SetTheme on X, as it   }
{           is completely inert on X, and will return unimpErr.                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeNameTag = FourCharCode('name'); { Str255}
	kThemeVariantNameTag = FourCharCode('varn'); { Str255}
	kThemeVariantBaseTintTag = FourCharCode('tint'); { RGBColor (10.1 and later)}
	kThemeHighlightColorTag = FourCharCode('hcol'); { RGBColor}
	kThemeScrollBarArrowStyleTag = FourCharCode('sbar'); { ThemeScrollBarArrowStyle}
	kThemeScrollBarThumbStyleTag = FourCharCode('sbth'); { ThemeScrollBarThumbStyle}
	kThemeSoundsEnabledTag = FourCharCode('snds'); { Boolean}
	kThemeDblClickCollapseTag = FourCharCode('coll'); { Boolean}

const
	kThemeAppearanceFileNameTag = FourCharCode('thme'); { Str255}
	kThemeSystemFontTag = FourCharCode('lgsf'); { Str255}
	kThemeSmallSystemFontTag = FourCharCode('smsf'); { Str255}
	kThemeViewsFontTag = FourCharCode('vfnt'); { Str255}
	kThemeViewsFontSizeTag = FourCharCode('vfsz'); { SInt16}
	kThemeDesktopPatternNameTag = FourCharCode('patn'); { Str255}
	kThemeDesktopPatternTag = FourCharCode('patt'); { <variable-length data> (flattened pattern)}
	kThemeDesktopPictureNameTag = FourCharCode('dpnm'); { Str255}
	kThemeDesktopPictureAliasTag = FourCharCode('dpal'); { <alias handle>}
	kThemeDesktopPictureAlignmentTag = FourCharCode('dpan'); { UInt32 (see the Picture Alignments below)}
	kThemeHighlightColorNameTag = FourCharCode('hcnm'); { Str255}
	kThemeExamplePictureIDTag = FourCharCode('epic'); { SInt16}
	kThemeSoundTrackNameTag = FourCharCode('sndt'); { Str255}
	kThemeSoundMaskTag = FourCharCode('smsk'); { UInt32}
	kThemeUserDefinedTag = FourCharCode('user'); { Boolean (this should _always_ be true if present - used by Control Panel).}
	kThemeSmoothFontEnabledTag = FourCharCode('smoo'); { Boolean}
	kThemeSmoothFontMinSizeTag = FourCharCode('smos'); { UInt16 (must be >= 12 and <= 24)}

{ Picture Aligmnents that might be reported in the data for kThemeDesktopPictureAlignmentTag}
const
	kTiledOnScreen = 1;    { draws picture repeatedly}
	kCenterOnScreen = 2;    { "actual size", shows pattern on sides or clips picture if necessary}
	kFitToScreen = 3;    { shrinks if necessary}
	kFillScreen = 4;    { messes up aspect ratio if necessary}
	kUseBestGuess = 5;     { heuristically determines the best way to display the picture based on picture and monitor sizes}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Theme Control Settings                                                                   }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeCheckBoxClassicX = 0;    { check box with an 'X'}
	kThemeCheckBoxCheckMark = 1;     { check box with a real check mark}

type
	ThemeCheckBoxStyle = UInt16;
const
	kThemeScrollBarArrowsSingle = 0;    { single arrow on each end}
	kThemeScrollBarArrowsLowerRight = 1;   { double arrows only on right or bottom}

type
	ThemeScrollBarArrowStyle = UInt16;
const
	kThemeScrollBarThumbNormal = 0;    { normal, classic thumb size}
	kThemeScrollBarThumbProportional = 1;  { proportional thumbs}

type
	ThemeScrollBarThumbStyle = UInt16;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Font constants                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  Summary:
 *    A ThemeFontID value is a virtual font ID which can be passed to
 *    one of the Appearance Manager's text-related routines. Within
 *    those routines, the ThemeFontID is mapped into the appropriate
 *    real font (or fonts), size, and style based on the system
 *    appearance (Platinum on Mac OS 9, Aqua on Mac OS X), the string
 *    to be rendered (if any), the language/ script that the app is
 *    running in, and possibly other factors. The ThemeFontIDs allow
 *    you to get the correct text appearance for the platform your app
 *    is currently running on.
 }
const
{
   * The font used to draw most interface elements. If you can't find a
   * more appropriate font from the list below, you should use this
   * one. This font is suitable for drawing titles on most custom
   * widgets/buttons, as well as most static text in dialogs and
   * windows.
   }
	kThemeSystemFont = 0;

  {
   * The font used to draw interface elements when space is at a
   * premium. It draws a slightly smaller font compared to
   * kThemeSystemFont.
   }
	kThemeSmallSystemFont = 1;

  {
   * Identical to kThemeSmallSystemFont, except it draws bolded (or
   * otherwise emphasized in some fashion appropriate to your
   * application's language/script).
   }
	kThemeSmallEmphasizedSystemFont = 2;

  {
   * The font used to draw file and folder names in Finder windows or
   * other browsable lists.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kThemeViewsFont = 3;
=======
	kThemeViewsFont = 3;    { The following ID's are only available with MacOS X or CarbonLib 1.3 and later}
>>>>>>> graemeg/fixes_2_2
=======
	kThemeViewsFont = 3;    { The following ID's are only available with MacOS X or CarbonLib 1.3 and later}
>>>>>>> origin/fixes_2_2
=======
	kThemeViewsFont = 3;
>>>>>>> origin/fixes_2.4

  {
   * Identical to kThemeSystemFont, except it draws bolded (or
   * otherwise emphasized in some fashion appropriate to your
   * application's language/script). Only available on Mac OS X or
   * CarbonLib 1.3 or later.
   }
	kThemeEmphasizedSystemFont = 4;

  {
   * An analog to the Script Manager's notion of the Application Font.
   * This font is a suitable default choice for your application's
   * document-style text editing areas. Only available on Mac OS X or
   * CarbonLib 1.3 or later.
   }
	kThemeApplicationFont = 5;

  {
   * Generally smaller than kThemeSmallSystemFont, this font is
   * appropriate for drawing text labels next to image content that
   * reinforces the text's meaning (such as on a bevel button). Only
   * available on Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeLabelFont = 6;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kThemeSystemFontDetail = 7;
	kThemeSystemFontDetailEmphasized = 8;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kThemeSystemFontDetail = 7;
	kThemeSystemFontDetailEmphasized = 8;
>>>>>>> origin/fixes_2.4

  {
   * The font used to draw menu titles in the menu bar. Only available
   * on Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeMenuTitleFont = 100;

  {
   * The font used to draw menu items in the menus. Only available on
   * Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeMenuItemFont = 101;

  {
   * The font used to draw menu item marks in the menus. Only available
   * on Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeMenuItemMarkFont = 102;

  {
   * The font used to draw menu item command key equivalents in the
   * menus. Only available on Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeMenuItemCmdKeyFont = 103;

  {
   * The font used to draw text in most window title bars. Only
   * available on Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeWindowTitleFont = 104;

  {
   * The font used to draw text labels on push buttons. Only available
   * on Mac OS X or CarbonLib 1.3 or later.
   }
	kThemePushButtonFont = 105;

  {
   * The font used to draw text in utility window title bars. Only
   * available on Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeUtilityWindowTitleFont = 106;

  {
   * The font used to draw the first (and most important) message of an
   * alert window. Only available on Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeAlertHeaderFont = 107;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

  {
   * The font used to draw the label of a toolbar item. Available in
   * Mac OS X 10.2 or later.
   }
	kThemeToolbarFont = 108;

  {
   * The appropriate system font for mini-sized controls. Available in
   * Mac OS X 10.3 or later.
   }
	kThemeMiniSystemFont = 109;
<<<<<<< HEAD
=======
	kThemeSystemFontDetail = 7;
	kThemeSystemFontDetailEmphasized = 8;
>>>>>>> graemeg/fixes_2_2
=======
	kThemeSystemFontDetail = 7;
	kThemeSystemFontDetailEmphasized = 8;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

  {
   * Unlike the other ThemeFontIDs, this one doesn't map to a font
   * appropriate to your application's language or script. It maps
   * directly to the font, size, and style of the current Quickdraw
   * port. This allows you to get somewhat customized behavior out of
   * the APIs which take ThemeFontIDs. Note, however, that
   * kThemeCurrentPortFont does not (and will never) support all
   * Quickdraw styles on all platforms; in particular, outline and
   * shadow style are not supported on Mac OS X. Additionally,
   * kThemeCurrentPortFont is not (and will never be) completely
   * unicode savvy; use of kThemeCurrentPortFont may result in errors
   * having to do with the current port's font not being appropriate
   * for rendering or measuring all glyphs in a given unicode string.
   * Because of overhead associated with gathering Quickdraw font
   * information and converting it to the native font format on Mac OS
   * X, use of kThemeCurrentPortFont may slow down your text drawing
   * and measuring significantly compared to other ThemeFontIDs.
   * Instead of using kThemeCurrentPortFont, your application will
   * probably be better served by using one of the other ThemeFontIDs;
   * use kThemeCurrentPortFont only as a last resort. Only available on
   * Mac OS X or CarbonLib 1.3 or later.
   }
	kThemeCurrentPortFont = 200;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
   * Only to be used as a fontID parameter in the HIThemeTextInfo
   * structure. It allows the caller to measure and draw text using a
   * CTFontRef and is the correct method for using custom fonts due to
   * QuickDraw deprecation. When the fontID parameter of the
   * HIThemeTextInfo structure is kThemeSpecifiedFont, the version
   * field must be kHIThemeTextInfoVersionOne and the font field must
   * be a valid CTFontRef. Available in Mac OS X 10.5 or later.
<<<<<<< HEAD
   }
	kThemeSpecifiedFont = 201;

{ This is the total of the PUBLIC ThemeFontIDs!}
const
	kPublicThemeFontCount = 21;
=======
=======
>>>>>>> origin/fixes_2_2
   * The font used to draw the label of a toolbar item. Available in
   * Mac OS X 10.2 or later.
   }
	kThemeToolbarFont = 108;

  {
   * The appropriate system font for mini-sized controls. Available in
   * Mac OS X 10.3 or later.
=======
>>>>>>> origin/fixes_2.4
   }
	kThemeSpecifiedFont = 201;

{ This is the total of the PUBLIC ThemeFontIDs!}
const
<<<<<<< HEAD
	kPublicThemeFontCount = 20;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kPublicThemeFontCount = 21;
>>>>>>> origin/fixes_2.4

type
	ThemeFontID = UInt16;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Tab constants                                                                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeTabNonFront = 0;
	kThemeTabNonFrontPressed = 1;
	kThemeTabNonFrontInactive = 2;
	kThemeTabFront = 3;
	kThemeTabFrontInactive = 4;
	kThemeTabNonFrontUnavailable = 5;
	kThemeTabFrontUnavailable = 6;

type
	ThemeTabStyle = UInt16;
const
	kThemeTabNorth = 0;
	kThemeTabSouth = 1;
	kThemeTabEast = 2;
	kThemeTabWest = 3;

type
	ThemeTabDirection = UInt16;

{
 *  Summary:
 *    Deprecated tab height and overlap constants.
 *  
 *  Discussion:
 *    These constants have been deprecated in favor of theme metrics.
 *    Please do not use them anymore. These constants will be removed
 *    in the next major release of OS X.
 }
const
{
   * Deprecated. Use kThemeMetricSmallTabHeight.
   }
	kThemeSmallTabHeight = 16;

  {
   * Deprecated. Use kThemeMetricLargeTabHeight.
   }
	kThemeLargeTabHeight = 21;

  {
   * Deprecated. Use kThemeMetricTabFrameOverlap.
   }
	kThemeTabPaneOverlap = 3;

  {
   * Deprecated. Use kThemeMetricSmallTabHeight and
   * kThemeMetricSmallTabFrameOverlap.
   }
	kThemeSmallTabHeightMax = 19;

  {
   * Deprecated. Use metric kThemeMetricLargeTabHeight and
   * kThemeMetricTabFrameOverlap.
   }
	kThemeLargeTabHeightMax = 24;


{
 *  Discussion:
 *    ThemeTrackKinds
 }
const
{
   * This is the primitive used to draw the normal variant of the
   * scroll bar control.
   }
	kThemeScrollBarMedium = 0;

  {
   * This is the primitive used to draw the small variant of the scroll
   * bar control.
   }
	kThemeScrollBarSmall = 1;

  {
   * This is the primitive used to draw the normal variant of the
   * slider control.
   }
	kThemeSliderMedium = 2;

  {
   * This is the primitive used to draw the normal variant of the
   * progress bar control.
   }
	kThemeProgressBarMedium = 3;

  {
   * This is the primitive used to draw the normal variant of the
   * indeterminate progress bar control.
   }
	kThemeIndeterminateBarMedium = 4;

  {
   * This is the primitive used to draw the normal variant of the
   * relevance bar control.
   }
	kThemeRelevanceBar = 5;

  {
   * This is the primitive used to draw the small variant of the slider
   * control.
   }
	kThemeSliderSmall = 6;

  {
   * This is the primitive used to draw the large variant of the
   * progress bar control.
   }
	kThemeProgressBarLarge = 7;

  {
   * This is the primitive used to draw the large variant of the
   * indeterminate progress bar control.
   }
	kThemeIndeterminateBarLarge = 8;


{
 *  Discussion:
 *    New ThemeTrackKinds on Mac OS X 10.3 and later. Not all of them
 *    are implemented.
 }
const
{
   * Not implemented. Will return paramErr if used.
   }
	kThemeScrollBarMini = 9;

  {
   * A miniature version of the slider.
   }
	kThemeSliderMini = 10;

  {
   * Not implemented. Will return paramErr if used.
   }
	kThemeProgressBarMini = 11;

  {
   * Not implemented. Will return paramErr if used.
   }
	kThemeIndeterminateBarMini = 12;


{
 *  Discussion:
 *    These are legacy synonyms for previously defined ThemeTrackKinds.
 *    Please use the modern constant names.
 }
const
	kThemeMediumScrollBar = kThemeScrollBarMedium;
	kThemeSmallScrollBar = kThemeScrollBarSmall;
	kThemeMediumSlider = kThemeSliderMedium;
	kThemeMediumProgressBar = kThemeProgressBarMedium;
	kThemeMediumIndeterminateBar = kThemeIndeterminateBarMedium;
	kThemeSmallSlider = kThemeSliderSmall;
	kThemeLargeProgressBar = kThemeProgressBarLarge;
	kThemeLargeIndeterminateBar = kThemeIndeterminateBarLarge;
	kThemeMiniScrollBar = kThemeScrollBarMini;
	kThemeMiniSlider = kThemeSliderMini;
	kThemeMiniProgressBar = kThemeProgressBarMini;
	kThemeMiniIndeterminateBar = kThemeIndeterminateBarMini;

type
	ThemeTrackKind = UInt16;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Track enable states                                                                      }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
{ track states }
	kThemeTrackActive = 0;
	kThemeTrackDisabled = 1;
	kThemeTrackNothingToScroll = 2;
	kThemeTrackInactive = 3;

type
	ThemeTrackEnableState = UInt8;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Track pressed states                                                                     }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
{ press states (ignored unless track is active) }
	kThemeLeftOutsideArrowPressed = $01;
	kThemeLeftInsideArrowPressed = $02;
	kThemeLeftTrackPressed = $04;
	kThemeThumbPressed = $08;
	kThemeRightTrackPressed = $10;
	kThemeRightInsideArrowPressed = $20;
	kThemeRightOutsideArrowPressed = $40;
	kThemeTopOutsideArrowPressed = kThemeLeftOutsideArrowPressed;
	kThemeTopInsideArrowPressed = kThemeLeftInsideArrowPressed;
	kThemeTopTrackPressed = kThemeLeftTrackPressed;
	kThemeBottomTrackPressed = kThemeRightTrackPressed;
	kThemeBottomInsideArrowPressed = kThemeRightInsideArrowPressed;
	kThemeBottomOutsideArrowPressed = kThemeRightOutsideArrowPressed;

type
	ThemeTrackPressState = UInt8;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Thumb directions                                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
{ thumb direction }
	kThemeThumbPlain = 0;
	kThemeThumbUpward = 1;
	kThemeThumbDownward = 2;

type
	ThemeThumbDirection = UInt8;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Track attributes                                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  Discussion:
 *    Theme track attributes control the look of the track elements as
 *    drawn by the DrawThemeTrackFoo as well as the region returned by
 *    GetThemeTrackFoo.
 }
const
{
   * The track is drawn horizontally.
   }
	kThemeTrackHorizontal = 1 shl 0;

  {
   * The track progresses from right to left.
   }
	kThemeTrackRightToLeft = 1 shl 1;

  {
   * The track's thumb should be drawn.
   }
	kThemeTrackShowThumb = 1 shl 2;

  {
   * The provided thumbRgn should be drawn opaque, not as a ghost.
   }
	kThemeTrackThumbRgnIsNotGhost = 1 shl 3;

  {
   * The track scroll bar doesn't have arrows.  This attribute
   * currently has no effect
   }
	kThemeTrackNoScrollBarArrows = 1 shl 4;

  {
   * The thumb has focus.  This attribute currently has effect only on
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * sliders.  Available only in Mac OS X 10.2 and later.
   }
	kThemeTrackHasFocus = 1 shl 5;

  {
   * The track's track (i.e. the groove for a slider or the
   * gutter+arrows of a scroll bar) should not be drawn. Available only
   * in Mac OS X 10.5 and later.
   }
	kThemeTrackHideTrack = 1 shl 6;

=======
=======
>>>>>>> origin/fixes_2_2
   * sliders.  Available only in Mac OS X after 10.1.
   }
	kThemeTrackHasFocus = 1 shl 5;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
   * sliders.  Available only in Mac OS X 10.2 and later.
   }
	kThemeTrackHasFocus = 1 shl 5;

  {
   * The track's track (i.e. the groove for a slider or the
   * gutter+arrows of a scroll bar) should not be drawn. Available only
   * in Mac OS X 10.5 and later.
   }
	kThemeTrackHideTrack = 1 shl 6;

>>>>>>> origin/fixes_2.4
type
	ThemeTrackAttributes = UInt16;

{
 *  ScrollBarTrackInfo
 *  
 *  Summary:
 *    Drawing parameters passed to scroll bar drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    Used in both the old theme APIs and the newer HITheme APIs.
 }
type
	ScrollBarTrackInfoPtr = ^ScrollBarTrackInfo;
	ScrollBarTrackInfo = record
{
   * The current view range size.
   }
		viewsize: SInt32;               { current view range size }

  {
   * ThemeTrackPressState bits that indicate the pressed parts state.
   }
		pressState: SInt8 {ThemeTrackPressState};           { pressed parts state }
		pad: SInt8
	end;

{
 *  SliderTrackInfo
 *  
 *  Summary:
 *    Drawing parameters passed to slider drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    Used in both the old theme APIs and the newer HITheme APIs.
 }
type
	SliderTrackInfoPtr = ^SliderTrackInfo;
	SliderTrackInfo = record
{
   * A ThemeThumbDirection indicating the thumb direction.
   }
		thumbDir: SInt8 {ThemeThumbDirection};              { thumb direction }

  {
   * ThemeTrackPressState bits that indicate the pressed parts state.
   }
		pressState: SInt8 {ThemeTrackPressState};           { pressed parts state }
	end;

{
 *  ProgressTrackInfo
 *  
 *  Summary:
 *    Drawing parameters passed to (indeterminate) progress bar drawing
 *    and measuring theme APIs.
 *  
 *  Discussion:
 *    Used in both the old theme APIs and the newer HITheme APIs.
 }
type
	ProgressTrackInfoPtr = ^ProgressTrackInfo;
	ProgressTrackInfo = record
{
   * A UInt8 indicating the current phase of the track fill.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		phase: UInt8;                  { phase for indeterminate progress }
=======
		phase: SInt8;                  { phase for indeterminate progress }
>>>>>>> graemeg/fixes_2_2
=======
		phase: SInt8;                  { phase for indeterminate progress }
>>>>>>> origin/fixes_2_2
=======
		phase: UInt8;                  { phase for indeterminate progress }
>>>>>>> origin/fixes_2.4
		pad: SInt8
	end;

{
 *  ThemeTrackDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to track drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    Use the newer HITheme APIs.
 }
type
	ThemeTrackDrawInfoPtr = ^ThemeTrackDrawInfo;
	ThemeTrackDrawInfo = record
{
   * The ThemeTrackKind of the track being drawn or measured.
   }
		kind: ThemeTrackKind;

  {
   * The bounds of the track is being drawn or measured.
   }
		bounds: Rect;

  {
   * The minimum allowable value for the track being drawn or measured.
   }
		min: SInt32;

  {
   * The maximum allowable value for the track being drawn or measured.
   }
		max: SInt32;

  {
   * The value for the track being drawn or measured.
   }
		value: SInt32;

  {
   * Leave this reserved field set to 0.
   }
		reserved: UInt32;


  {
   * A set of ThemeTrackAttributes for the track to be drawn or
   * measured.
   }
		attributes: ThemeTrackAttributes;

  {
   * A ThemeTrackEnableState describing the state of the track to be
   * drawn or measured.
   }
		enableState: SInt8 {ThemeTrackEnableState};

  {
   * Leave this reserved field set to 0.
   }
		filler1: SInt8;
		case SInt16 of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
			0: (
				scrollbar: ScrollBarTrackInfo;
				);
			1: (
				slider: SliderTrackInfo;
				);
			2: (
				progress: ProgressTrackInfo;
				);
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		0: (
			scrollbar:			ScrollBarTrackInfo;
			);
		1: (
			slider:				SliderTrackInfo;
			);
		2: (
			progress:			ProgressTrackInfo;
			);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;

{
 *  Summary:
 *    ThemeWindowAttributes
 *  
 *  Discussion:
 *    Theme window attributes control the look of the window elements
 *    as drawn by the DrawThemeWindowFrame.
 }
const
{
   * This bit means that the size of the window can be changed by the
   * user. The window is drawn with a grow box.
   }
	kThemeWindowHasGrow = 1 shl 0;

  {
   * This bit means that the window can be zoomed horizontally. The
   * window is drawn with a zoom box.
   }
	kThemeWindowHasHorizontalZoom = 1 shl 3;

  {
   * This bit means that the window can be zoomed vertically. The
   * window is drawn with a zoom box.
   }
	kThemeWindowHasVerticalZoom = 1 shl 4;

  {
   * This bit means that the window can be zoomed horizontally and
   * vertically. The window is drawn with a zoom box.
   }
	kThemeWindowHasFullZoom = kThemeWindowHasHorizontalZoom + kThemeWindowHasVerticalZoom;

  {
   * This bit means that the window can be closed. The window is drawn
   * with a close box.
   }
	kThemeWindowHasCloseBox = 1 shl 5;

  {
   * This bit means that the window can be collapsed. The window is
   * drawn with a collapse box.
   }
	kThemeWindowHasCollapseBox = 1 shl 6;

  {
   * This bit means that the window has title text and/or a title icon.
   }
	kThemeWindowHasTitleText = 1 shl 7;

  {
   * This bit means that the window has been collapsed.
   }
	kThemeWindowIsCollapsed = 1 shl 8;

  {
   * This bit means that the window has dirty content. Get your mind
   * out of the gutter. That means that the content of the window has
   * changed and needs to be saved. If the window is drawing a close
   * box, it will be drawn dirty.
   }
	kThemeWindowHasDirty = 1 shl 9;

  {
   * This bit means that the window has a toolbar toggling button.
   }
	kThemeWindowHasToolbarButton = 1 shl 11;


type
	ThemeWindowAttributes = UInt32;

{
 *  Summary:
 *    ThemeWindowTypes
 *  
 *  Discussion:
 *    Window Types Supported by the Appearance Manager.
 }
const
	kThemeDocumentWindow = 0;
	kThemeDialogWindow = 1;
	kThemeMovableDialogWindow = 2;
	kThemeAlertWindow = 3;
	kThemeMovableAlertWindow = 4;
	kThemePlainDialogWindow = 5;
	kThemeShadowDialogWindow = 6;
	kThemePopupWindow = 7;
	kThemeUtilityWindow = 8;
	kThemeUtilitySideWindow = 9;
	kThemeSheetWindow = 10;
	kThemeDrawerWindow = 11;

type
	ThemeWindowType = UInt16;

{
 *  Summary:
 *    ThemeTitleBarWidgets
 *  
 *  Discussion:
 *    Window Widgets Supported by the Appearance Manager.
 }
const
	kThemeWidgetCloseBox = 0;
	kThemeWidgetZoomBox = 1;
	kThemeWidgetCollapseBox = 2;
	kThemeWidgetDirtyCloseBox = 6;
<<<<<<< HEAD
<<<<<<< HEAD
	kThemeWidgetToolbarButton = 7;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

const
{ Old/Obsolete name to be removed}
	kThemeWidgetABox = 3;
	kThemeWidgetBBox = 4;
	kThemeWidgetBOffBox = 5;

type
	ThemeTitleBarWidget = UInt16;

{
 *  Summary:
 *    ThemeArrowOrientation
 *  
 *  Discussion:
 *    Popup arrow orientations.
 }
const
	kThemeArrowLeft = 0;
	kThemeArrowDown = 1;
	kThemeArrowRight = 2;
	kThemeArrowUp = 3;

type
	ThemeArrowOrientation = UInt16;

{
 *  Summary:
 *    ThemePopupArrowSizes
 *  
 *  Discussion:
 *    Popup arrow sizes.
 }
const
	kThemeArrow3pt = 0;
	kThemeArrow5pt = 1;
	kThemeArrow7pt = 2;
	kThemeArrow9pt = 3;

type
	ThemePopupArrowSize = UInt16;

{
 *  Summary:
 *    ThemeGrowDirections
 *  
 *  Discussion:
 *    Grow box directions.
 }
const
	kThemeGrowLeft = 1 shl 0; { can grow to the left }
	kThemeGrowRight = 1 shl 1; { can grow to the right }
	kThemeGrowUp = 1 shl 2; { can grow up }
	kThemeGrowDown = 1 shl 3; { can grow down }

type
	ThemeGrowDirection = UInt16;

{
 *  Discussion:
 *    ThemeButtonKinds
 }
const
{
   * Dynamically-sized push button kind. Prior to Mac OS X 10.3 all
   * push button sizes could be determined dynamically: either they
   * were smaller than normal size and therefore small or they were
   * normal size. This constant will never describe a mini push button,
   * regardless of associated bounds. Please use the explicitly-sized
   * kThemePushButton(Normal,Small,Mini) constants.
   }
	kThemePushButton = 0;
	kThemeCheckBox = 1;
	kThemeRadioButton = 2;

  {
   * Bevel button (obsolete)
   }
	kThemeBevelButton = 3;

  {
   * Popup button without text (no label). See ThemeButtonAdornment for
   * glyphs. The arrow button theme name is somewhat confusing. This is
   * the primitive used to draw the control known as the disclosure
   * button.
   }
	kThemeArrowButton = 4;

  {
   * Dynamically-sized popup button kind. Prior to Mac OS X 10.3 all
   * popup button sizes could be determined dynamically: either they
   * were smaller than normal size and therefore small or they were
   * normal size. This constant will never describe a mini popup
   * button, regardless of associated bounds. Please use the
   * explicitly-sized kThemePopupButton(Normal,Small,Mini) constants.
   }
	kThemePopupButton = 5;

  {
   * This is a disclosure triangle with a label beside it. Used to be
   * confusingly named kThemeDisclosureTriangle.
   }
	kThemeDisclosureTriangle = 6;

  {
   * Increment/decrement buttons (no label). This is the primitive used
   * to draw the LittleArrows control.
   }
	kThemeIncDecButton = 7;

  {
   * Small-shadow bevel button
   }
	kThemeBevelButtonSmall = 8;

  {
   * Med-shadow bevel button
   }
	kThemeBevelButtonMedium = 3;

  {
   * Large-shadow bevel button
   }
	kThemeBevelButtonLarge = 9;

  {
   * Sort button for top of a list. This is the theme primitive used to
   * draw the top of the columns in the data browser.
   }
	kThemeListHeaderButton = 10;

  {
   * This is the primitive used to draw the normal variant of the round
   * button control.
   }
	kThemeRoundButton = 11;

  {
   * This is the primitive used to draw the large variant of the round
   * button control.
   }
	kThemeRoundButtonLarge = 12;

  {
   * This is the primitive used to draw the small variant of the check
   * box control.
   }
	kThemeCheckBoxSmall = 13;

  {
   * This is the primitive used to draw the small variant of the radio
   * button control.
   }
	kThemeRadioButtonSmall = 14;

  {
   * This is the primitive used to draw the rounded-corner variant of
   * the bevel button control.
   }
	kThemeRoundedBevelButton = 15;

  {
   * This is the primitive used to draw the normal variant of the combo
   * box control.
   }
	kThemeComboBox = 16;

  {
   * This is the primitive used to draw the small variant of the combo
   * box control.
   }
	kThemeComboBoxSmall = 17;


{
 *  Discussion:
 *    New ThemeButtonKinds available on Mac OS X 10.3 and later.
 }
const
{
   * This is the primitive used to draw the mini variant of the combo
   * box control.
   }
	kThemeComboBoxMini = 18;

  {
   * This is the primitive used to draw the mini variant of the check
   * box control.
   }
	kThemeCheckBoxMini = 19;

  {
   * This is the primitive used to draw the mini variant of the radio
   * button control.
   }
	kThemeRadioButtonMini = 20;

  {
   * This is the primitive used to draw the small variant of the
   * LittleArrows control.
   }
	kThemeIncDecButtonSmall = 21;

  {
   * This is the primitive used to draw the mini variant of the
   * LittleArrows control.
   }
	kThemeIncDecButtonMini = 22;

  {
   * The arrow button theme name is somewhat confusing. This is the
   * primitive used to draw the small variant of the control known as
   * the disclosure button.
   }
	kThemeArrowButtonSmall = 23;

  {
   * The arrow button theme name is somewhat confusing. This is the
   * primitive used to draw the mini variant of the control known as
   * the disclosure button.
   }
	kThemeArrowButtonMini = 24;

  {
   * Explicitly-sized normal push button kind. Prior to Mac OS X 10.3
   * all push button sizes could be determined dynamically: either they
   * were smaller than normal size and therefore small or they were
   * normal size. Using this constant, an explicitly-sized normal push
   * button can be specified.
   }
	kThemePushButtonNormal = 25;

  {
   * Explicitly-sized small push button kind. Prior to Mac OS X 10.3
   * all push button sizes could be determined dynamically: either they
   * were smaller than normal size and therefore small or they were
   * normal size. Using this constant, an explicitly-sized small push
   * button can be specified.
   }
	kThemePushButtonSmall = 26;

  {
   * Explicitly-sized mini push button kind. Prior to Mac OS X 10.3 all
   * push button sizes could be determined dynamically: either they
   * were smaller than normal size and therefore small or they were
   * normal size. Since a mini variant was introduced in Mac OS X 10.3,
   * smaller than normal size is can also mean mini. To avoid confusion
   * with existing code, the mini variant will never be implicitly
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * determined and must be explicitly requested with the
=======
   * determined and must be explicity requested with the
>>>>>>> graemeg/fixes_2_2
=======
   * determined and must be explicity requested with the
>>>>>>> origin/fixes_2_2
=======
   * determined and must be explicitly requested with the
>>>>>>> origin/fixes_2.4
   * kThemePushButtonMini constant.
   }
	kThemePushButtonMini = 27;

  {
   * Explicitly-sized normal popup button kind. Prior to Mac OS X 10.3
   * all popup button sizes could be determined dynamically: either
   * they were smaller than normal size and therefore small or they
   * were normal size. Using this constant, an explicitly-sized normal
   * popup button can be specified.
   }
	kThemePopupButtonNormal = 28;

  {
   * Explicitly-sized small popup button kind. Prior to Mac OS X 10.3
   * all popup button sizes could be determined dynamically: either
   * they were smaller than normal size and therefore small or they
   * were normal size. Using this constant, an explicitly-sized small
   * popup button can be specified.
   }
	kThemePopupButtonSmall = 29;

  {
   * Explicitly-sized mini popup button kind. Prior to Mac OS X 10.3
   * all popup button sizes could be determined dynamically: either
   * they were smaller than normal size and therefore small or they
   * were normal size. Since a mini variant was introduced in Mac OS X
   * 10.3, smaller than normal size is can also mean mini. To avoid
   * confusion with existing code, the mini variant will never be
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * implicitly determined and must be explicitly requested with the
=======
   * implicitly determined and must be explicity requested with the
>>>>>>> graemeg/fixes_2_2
=======
   * implicitly determined and must be explicity requested with the
>>>>>>> origin/fixes_2_2
=======
   * implicitly determined and must be explicitly requested with the
>>>>>>> origin/fixes_2.4
   * kThemePopupButtonMini constant.
   }
	kThemePopupButtonMini = 30;


{
 *  Discussion:
 *    New ThemeButtonKinds available on Mac OS X 10.4 and later.
 }
const
{
   * This is the primitive used to draw the inset variant of the bevel
   * button control.
   }
	kThemeBevelButtonInset = 31;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * This is the primitive used to draw the textured variant of the
   * push button. Similar to kThemePushButtonNormal, but textured, to
   * be rendered on a textured window.
   }
	kThemePushButtonTextured = 32;

  {
   * This is the primitive used to draw the small, textured variant of
   * the push button. Similar to kThemePushButtonNormal, but textured,
   * to be rendered on a textured window.
   }
	kThemePushButtonTexturedSmall = 33;
=======
=======
>>>>>>> origin/fixes_2_2
   * This is the primitive used to draw the inset variant of the push
   * button. Similar to kThemePushButtonNormal, but inset.
=======
   * This is the primitive used to draw the textured variant of the
   * push button. Similar to kThemePushButtonNormal, but textured, to
   * be rendered on a textured window.
>>>>>>> origin/fixes_2.4
   }
	kThemePushButtonTextured = 32;

  {
   * This is the primitive used to draw the small, textured variant of
   * the push button. Similar to kThemePushButtonNormal, but textured,
   * to be rendered on a textured window.
   }
<<<<<<< HEAD
	kThemePushButtonInsetSmall = 33;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kThemePushButtonTexturedSmall = 33;
>>>>>>> origin/fixes_2.4

  {
   * This is the primitive used to draw the help variant of the round
   * button.
   }
	kThemeRoundButtonHelp = 34;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  {
   * This is a synonym for kThemePushButtonTextured for code
   * compatibility. Please use kThemePushButtonTextured instead.
   }
	kThemePushButtonInset = kThemePushButtonTextured;

  {
   * This is a synonym for kThemePushButtonTexturedSmall for code
   * compatibility. Please use kThemePushButtonTexturedSmall instead.
   }
	kThemePushButtonInsetSmall = kThemePushButtonTexturedSmall;

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  Discussion:
 *    These are legacy synonyms for previously defined
 *    ThemeButtonKinds. Please use the modern constant names.
 }
const
	kThemeNormalCheckBox = kThemeCheckBox;
	kThemeNormalRadioButton = kThemeRadioButton;
	kThemeLargeBevelButton = kThemeBevelButtonLarge;
	kThemeMediumBevelButton = kThemeBevelButtonMedium;
	kThemeMiniCheckBox = kThemeCheckBoxMini;
	kThemeMiniRadioButton = kThemeRadioButtonMini;
	kThemeSmallBevelButton = kThemeBevelButtonSmall;
	kThemeSmallCheckBox = kThemeCheckBoxSmall;
	kThemeSmallRadioButton = kThemeRadioButtonSmall;
	kThemeLargeRoundButton = kThemeRoundButtonLarge;
	kThemeDisclosureButton = kThemeDisclosureTriangle;

type
	ThemeButtonKind = UInt16;

{
 *  Summary:
 *    ThemeButtonValues
 *  
 *  Discussion:
 *    Common button values
 }
const
	kThemeButtonOff = 0;
	kThemeButtonOn = 1;
	kThemeButtonMixed = 2;
	kThemeDisclosureRight = 0;
	kThemeDisclosureDown = 1;
	kThemeDisclosureLeft = 2;

type
	ThemeButtonValue = UInt16;


{
 *  Summary:
 *    ThemeButtonAdornments
 *  
 *  Discussion:
 *    Use these adornments when drawing buttons using the theme
 *    primitive. Note that the bits starting at (1 << 6) are recycled
 *    and have different meanings depending on ThemeButtonKind.
 }
const
{
   * No adornments.
   }
	kThemeAdornmentNone = 0;

  {
   * Draw default ornamentation (for push button and generic well).
   }
	kThemeAdornmentDefault = 1 shl 0;

  {
   * Draw focus.
   }
	kThemeAdornmentFocus = 1 shl 2;

  {
   * Draw right to left label.
   }
	kThemeAdornmentRightToLeft = 1 shl 4;

  {
   * Don't draw or erase label (radio, check, disclosure).
   }
	kThemeAdornmentDrawIndicatorOnly = 1 shl 5;

  {
   * Draw the left border of the button as selected (list header button
   * only).
   }
	kThemeAdornmentHeaderButtonLeftNeighborSelected = 1 shl 6;

  {
   * Draw the right border of the button (list header button only).
   }
	kThemeAdornmentHeaderButtonRightNeighborSelected = 1 shl 7;

  {
   * Draw the sort indicator pointing upward (list header button only).
   }
	kThemeAdornmentHeaderButtonSortUp = 1 shl 8;

  {
   * Draw as a header menu button (list header button only).
   }
	kThemeAdornmentHeaderMenuButton = 1 shl 9;

  {
   * Draw the non-shadow area of the button (list header button only).
   }
	kThemeAdornmentHeaderButtonNoShadow = 1 shl 10;

  {
   * Draw the only the shadow area of the button (list header button
   * only).
   }
	kThemeAdornmentHeaderButtonShadowOnly = 1 shl 11;

  {
   * When drawn selected, do not draw the sort direction arrow (list
   * header button only). Mac OS X 10.4 and later.
   }
	kThemeAdornmentHeaderButtonNoSortArrow = 1 shl 12;

  {
   * Draw a left arrow on the arrow button or draw left disclosure
   * triangle. Can be combined with kThemeAdornmentArrowDownArrow to
   * draw an intermediate disclosure triangle.
   }
	kThemeAdornmentArrowLeftArrow = 1 shl 6;

  {
   * Draw a down arrow on the arrow button or draw right disclosure
   * triangle. Can be combined with kThemeAdornmentArrowLeftArrow to
   * draw an intermediate disclosure triangle.
   }
	kThemeAdornmentArrowDownArrow = 1 shl 7;

  {
   * Draw a double arrow on the arrow button.
   }
	kThemeAdornmentArrowDoubleArrow = 1 shl 8;

  {
   * Draw a up arrow on the arrow button.
   }
	kThemeAdornmentArrowUpArrow = 1 shl 9;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  {
   * Draw a right disclosure triangle. Can be combined with
   * kThemeAdornmentArrowDownArrow to draw an intermediate disclosure
   * triangle.
   }
	kThemeAdornmentArrowRightArrow = 1 shl 10;

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  Discussion:
 *    These are legacy synonyms for previously defined
 *    ThemeButtonAdornments. Please use the modern constant names.
 }
const
	kThemeAdornmentNoShadow = kThemeAdornmentHeaderButtonNoShadow;
	kThemeAdornmentShadowOnly = kThemeAdornmentHeaderButtonShadowOnly;

type
	ThemeButtonAdornment = UInt16;

{
 *  ThemeButtonDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to button drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    Use the newer HITheme APIs.
 }
type
	ThemeButtonDrawInfo = record
{
   * The ThemeDrawState of the button being drawn or measured.
   }
		state: ThemeDrawState;

  {
   * The ThemeButtonValue of the button being drawn or measured.
   }
		value: ThemeButtonValue;

  {
   * The ThemeButtonAdornment(s) with which the button is being drawn
   * or measured.
   }
		adornment: ThemeButtonAdornment;
	end;
	ThemeButtonDrawInfoPtr = ^ThemeButtonDrawInfo;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Sound Support                                                                            }
{  X ALERT: Please note that none of the theme sound APIs currently function on X.         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Sound classes                                                                            }
{ You can use the constants below to set what sounds are active using the SetTheme API.    }
{ Use these with the kThemeSoundMask tag.                                                  }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeNoSounds = 0;
	kThemeWindowSoundsMask = 1 shl 0;
	kThemeMenuSoundsMask = 1 shl 1;
	kThemeControlSoundsMask = 1 shl 2;
	kThemeFinderSoundsMask = 1 shl 3;


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Drag Sounds                                                                              }
{ Drag sounds are looped for the duration of the drag.                                     }
{ Call BeginThemeDragSound at the start of the drag.                                       }
{ Call EndThemeDragSound when the drag has finished.                                       }
{ Note that in order to maintain a consistent user experience, only one drag sound may     }
{ occur at a time.  The sound should be attached to a mouse action, start after the        }
{ mouse goes down and stop when the mouse is released.                                     }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeDragSoundNone = 0;
	kThemeDragSoundMoveWindow = FourCharCode('wmov');
	kThemeDragSoundGrowWindow = FourCharCode('wgro');
	kThemeDragSoundMoveUtilWindow = FourCharCode('umov');
	kThemeDragSoundGrowUtilWindow = FourCharCode('ugro');
	kThemeDragSoundMoveDialog = FourCharCode('dmov');
	kThemeDragSoundMoveAlert = FourCharCode('amov');
	kThemeDragSoundMoveIcon = FourCharCode('imov');
	kThemeDragSoundSliderThumb = FourCharCode('slth');
	kThemeDragSoundSliderGhost = FourCharCode('slgh');
	kThemeDragSoundScrollBarThumb = FourCharCode('sbth');
	kThemeDragSoundScrollBarGhost = FourCharCode('sbgh');
	kThemeDragSoundScrollBarArrowDecreasing = FourCharCode('sbad');
	kThemeDragSoundScrollBarArrowIncreasing = FourCharCode('sbai');
	kThemeDragSoundDragging = FourCharCode('drag');

type
	ThemeDragSoundKind = OSType;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ State-change sounds                                                      }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ State-change sounds are played asynchronously as a one-shot.             }
=======
{ State-change sounds are played asynchonously as a one-shot.              }
>>>>>>> graemeg/fixes_2_2
=======
{ State-change sounds are played asynchonously as a one-shot.              }
>>>>>>> origin/fixes_2_2
=======
{ State-change sounds are played asynchronously as a one-shot.             }
>>>>>>> origin/fixes_2.4
{ Call PlayThemeSound to play the sound.  The sound will play              }
{ asynchronously until complete, then stop automatically.                  }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeSoundNone = 0;
	kThemeSoundMenuOpen = FourCharCode('mnuo'); { menu sounds }
	kThemeSoundMenuClose = FourCharCode('mnuc');
	kThemeSoundMenuItemHilite = FourCharCode('mnui');
	kThemeSoundMenuItemRelease = FourCharCode('mnus');
	kThemeSoundWindowClosePress = FourCharCode('wclp'); { window sounds }
	kThemeSoundWindowCloseEnter = FourCharCode('wcle');
	kThemeSoundWindowCloseExit = FourCharCode('wclx');
	kThemeSoundWindowCloseRelease = FourCharCode('wclr');
	kThemeSoundWindowZoomPress = FourCharCode('wzmp');
	kThemeSoundWindowZoomEnter = FourCharCode('wzme');
	kThemeSoundWindowZoomExit = FourCharCode('wzmx');
	kThemeSoundWindowZoomRelease = FourCharCode('wzmr');
	kThemeSoundWindowCollapsePress = FourCharCode('wcop');
	kThemeSoundWindowCollapseEnter = FourCharCode('wcoe');
	kThemeSoundWindowCollapseExit = FourCharCode('wcox');
	kThemeSoundWindowCollapseRelease = FourCharCode('wcor');
	kThemeSoundWindowDragBoundary = FourCharCode('wdbd');
	kThemeSoundUtilWinClosePress = FourCharCode('uclp'); { utility window sounds }
	kThemeSoundUtilWinCloseEnter = FourCharCode('ucle');
	kThemeSoundUtilWinCloseExit = FourCharCode('uclx');
	kThemeSoundUtilWinCloseRelease = FourCharCode('uclr');
	kThemeSoundUtilWinZoomPress = FourCharCode('uzmp');
	kThemeSoundUtilWinZoomEnter = FourCharCode('uzme');
	kThemeSoundUtilWinZoomExit = FourCharCode('uzmx');
	kThemeSoundUtilWinZoomRelease = FourCharCode('uzmr');
	kThemeSoundUtilWinCollapsePress = FourCharCode('ucop');
	kThemeSoundUtilWinCollapseEnter = FourCharCode('ucoe');
	kThemeSoundUtilWinCollapseExit = FourCharCode('ucox');
	kThemeSoundUtilWinCollapseRelease = FourCharCode('ucor');
	kThemeSoundUtilWinDragBoundary = FourCharCode('udbd');
	kThemeSoundWindowOpen = FourCharCode('wopn'); { window close and zoom action }
	kThemeSoundWindowClose = FourCharCode('wcls');
	kThemeSoundWindowZoomIn = FourCharCode('wzmi');
	kThemeSoundWindowZoomOut = FourCharCode('wzmo');
	kThemeSoundWindowCollapseUp = FourCharCode('wcol');
	kThemeSoundWindowCollapseDown = FourCharCode('wexp');
	kThemeSoundWindowActivate = FourCharCode('wact');
	kThemeSoundUtilWindowOpen = FourCharCode('uopn');
	kThemeSoundUtilWindowClose = FourCharCode('ucls');
	kThemeSoundUtilWindowZoomIn = FourCharCode('uzmi');
	kThemeSoundUtilWindowZoomOut = FourCharCode('uzmo');
	kThemeSoundUtilWindowCollapseUp = FourCharCode('ucol');
	kThemeSoundUtilWindowCollapseDown = FourCharCode('uexp');
	kThemeSoundUtilWindowActivate = FourCharCode('uact');
	kThemeSoundDialogOpen = FourCharCode('dopn');
	kThemeSoundDialogClose = FourCharCode('dlgc');
	kThemeSoundAlertOpen = FourCharCode('aopn');
	kThemeSoundAlertClose = FourCharCode('altc');
	kThemeSoundPopupWindowOpen = FourCharCode('pwop');
	kThemeSoundPopupWindowClose = FourCharCode('pwcl');
	kThemeSoundButtonPress = FourCharCode('btnp'); { button }
	kThemeSoundButtonEnter = FourCharCode('btne');
	kThemeSoundButtonExit = FourCharCode('btnx');
	kThemeSoundButtonRelease = FourCharCode('btnr');
	kThemeSoundDefaultButtonPress = FourCharCode('dbtp'); { default button }
	kThemeSoundDefaultButtonEnter = FourCharCode('dbte');
	kThemeSoundDefaultButtonExit = FourCharCode('dbtx');
	kThemeSoundDefaultButtonRelease = FourCharCode('dbtr');
	kThemeSoundCancelButtonPress = FourCharCode('cbtp'); { cancel button }
	kThemeSoundCancelButtonEnter = FourCharCode('cbte');
	kThemeSoundCancelButtonExit = FourCharCode('cbtx');
	kThemeSoundCancelButtonRelease = FourCharCode('cbtr');
	kThemeSoundCheckboxPress = FourCharCode('chkp'); { checkboxes }
	kThemeSoundCheckboxEnter = FourCharCode('chke');
	kThemeSoundCheckboxExit = FourCharCode('chkx');
	kThemeSoundCheckboxRelease = FourCharCode('chkr');
	kThemeSoundRadioPress = FourCharCode('radp'); { radio buttons }
	kThemeSoundRadioEnter = FourCharCode('rade');
	kThemeSoundRadioExit = FourCharCode('radx');
	kThemeSoundRadioRelease = FourCharCode('radr');
	kThemeSoundScrollArrowPress = FourCharCode('sbap'); { scroll bars }
	kThemeSoundScrollArrowEnter = FourCharCode('sbae');
	kThemeSoundScrollArrowExit = FourCharCode('sbax');
	kThemeSoundScrollArrowRelease = FourCharCode('sbar');
	kThemeSoundScrollEndOfTrack = FourCharCode('sbte');
	kThemeSoundScrollTrackPress = FourCharCode('sbtp');
	kThemeSoundSliderEndOfTrack = FourCharCode('slte'); { sliders }
	kThemeSoundSliderTrackPress = FourCharCode('sltp');
	kThemeSoundBalloonOpen = FourCharCode('blno'); { help balloons }
	kThemeSoundBalloonClose = FourCharCode('blnc');
	kThemeSoundBevelPress = FourCharCode('bevp'); { Bevel buttons }
	kThemeSoundBevelEnter = FourCharCode('beve');
	kThemeSoundBevelExit = FourCharCode('bevx');
	kThemeSoundBevelRelease = FourCharCode('bevr');
	kThemeSoundLittleArrowUpPress = FourCharCode('laup'); { Little Arrows }
	kThemeSoundLittleArrowDnPress = FourCharCode('ladp');
	kThemeSoundLittleArrowEnter = FourCharCode('lare');
	kThemeSoundLittleArrowExit = FourCharCode('larx');
	kThemeSoundLittleArrowUpRelease = FourCharCode('laur');
	kThemeSoundLittleArrowDnRelease = FourCharCode('ladr');
	kThemeSoundPopupPress = FourCharCode('popp'); { Popup Buttons }
	kThemeSoundPopupEnter = FourCharCode('pope');
	kThemeSoundPopupExit = FourCharCode('popx');
	kThemeSoundPopupRelease = FourCharCode('popr');
	kThemeSoundDisclosurePress = FourCharCode('dscp'); { Disclosure Buttons }
	kThemeSoundDisclosureEnter = FourCharCode('dsce');
	kThemeSoundDisclosureExit = FourCharCode('dscx');
	kThemeSoundDisclosureRelease = FourCharCode('dscr');
	kThemeSoundTabPressed = FourCharCode('tabp'); { Tabs }
	kThemeSoundTabEnter = FourCharCode('tabe');
	kThemeSoundTabExit = FourCharCode('tabx');
	kThemeSoundTabRelease = FourCharCode('tabr');
	kThemeSoundDragTargetHilite = FourCharCode('dthi'); { drag manager }
	kThemeSoundDragTargetUnhilite = FourCharCode('dtuh');
	kThemeSoundDragTargetDrop = FourCharCode('dtdr');
	kThemeSoundEmptyTrash = FourCharCode('ftrs'); { finder }
	kThemeSoundSelectItem = FourCharCode('fsel');
	kThemeSoundNewItem = FourCharCode('fnew');
	kThemeSoundReceiveDrop = FourCharCode('fdrp');
	kThemeSoundCopyDone = FourCharCode('fcpd');
	kThemeSoundResolveAlias = FourCharCode('fral');
	kThemeSoundLaunchApp = FourCharCode('flap');
	kThemeSoundDiskInsert = FourCharCode('dski');
	kThemeSoundDiskEject = FourCharCode('dske');
	kThemeSoundFinderDragOnIcon = FourCharCode('fdon');
	kThemeSoundFinderDragOffIcon = FourCharCode('fdof');

type
	ThemeSoundKind = OSType;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Window Metrics                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Window metrics are used by the Appearance manager to fill in the blanks necessary to    }
{  draw windows. If a value is not appropriate for the type of window, be sure to fill in  }
{  the slot in the structure with zero.    For the popupTabOffset parameter, you can pass a}
{  real value based on the left edge of the window. This value might be interpreted in a   }
{  different manner when depending on the value of the popupTabPosition field. The values  }
{  you can pass into popupTabPosition are:                                                 }
{  kThemePopupTabNormalPosition                                                            }
{      Starts the tab left edge at the position indicated by the popupTabOffset field.     }
{  kThemePopupTabCenterOnWindow                                                            }
{      tells us to ignore the offset field and instead simply center the width of the      }
{      handle on the window.                                                               }
{  kThemePopupTabCenterOnOffset                                                            }
{      tells us to center the width of the handle around the value passed in offset.       }
{  The Appearance Manager will try its best to accomodate the requested placement, but may }
{  move the handle slightly to make it fit correctly.                                      }
const
	kThemePopupTabNormalPosition = 0;
	kThemePopupTabCenterOnWindow = 1;
	kThemePopupTabCenterOnOffset = 2;

type
	ThemeWindowMetrics = record
		metricSize: UInt16;             { should be always be sizeof( ThemeWindowMetrics )}
		titleHeight: SInt16;
		titleWidth: SInt16;
		popupTabOffset: SInt16;
		popupTabWidth: SInt16;
		popupTabPosition: UInt16;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ThemeWindowMetricsPtr = ^ThemeWindowMetrics;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Drawing State                                                                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ThemeDrawingState = ^OpaqueThemeDrawingState; { an opaque type }
	OpaqueThemeDrawingState = record end;
=======
	ThemeDrawingState = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ThemeDrawingState = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ThemeDrawingState = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ThemeDrawingState = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	ThemeDrawingStatePtr = ^ThemeDrawingState;  { when a var xx:ThemeDrawingState parameter can be nil, it is changed to xx: ThemeDrawingStatePtr }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Callback procs                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  ThemeTabTitleDrawProcPtr
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Summary:
 *    Callback function for drawing the label text of a tab.
 }
type
	ThemeTabTitleDrawProcPtr = procedure( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: URefCon );
<<<<<<< HEAD

{
 *  ThemeEraseProcPtr
 *  
 *  Summary:
 *    Callback function for erasing the background behind a theme
 *    primitive.
 }
type
	ThemeEraseProcPtr = procedure( const (*var*) bounds: Rect; eraseData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  ThemeButtonDrawProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the label text of a button.
 }
type
	ThemeButtonDrawProcPtr = procedure( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  WindowTitleDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the title text of a window.
 }
type
	WindowTitleDrawingProcPtr = procedure( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: URefCon );

{
=======

{
 *  ThemeEraseProcPtr
 *  
 *  Summary:
 *    Callback function for erasing the background behind a theme
 *    primitive.
 }
type
	ThemeEraseProcPtr = procedure( const (*var*) bounds: Rect; eraseData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  ThemeButtonDrawProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the label text of a button.
 }
type
	ThemeButtonDrawProcPtr = procedure( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  WindowTitleDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the title text of a window.
 }
type
	WindowTitleDrawingProcPtr = procedure( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: URefCon );

{
>>>>>>> graemeg/cpstrnew
 *  ThemeIteratorProcPtr
 *  
 *  Summary:
 *    Callback function for iterating over all known themes.
 }
type
=======
 *  
 *  Summary:
 *    Callback function for drawing the label text of a tab.
 }
type
	ThemeTabTitleDrawProcPtr = procedure( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: URefCon );

{
 *  ThemeEraseProcPtr
 *  
 *  Summary:
 *    Callback function for erasing the background behind a theme
 *    primitive.
 }
type
	ThemeEraseProcPtr = procedure( const (*var*) bounds: Rect; eraseData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  ThemeButtonDrawProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the label text of a button.
 }
type
	ThemeButtonDrawProcPtr = procedure( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  WindowTitleDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the title text of a window.
 }
type
	WindowTitleDrawingProcPtr = procedure( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: URefCon );

{
 *  ThemeIteratorProcPtr
 *  
 *  Summary:
 *    Callback function for iterating over all known themes.
 }
type
>>>>>>> graemeg/cpstrnew
=======
 *  
 *  Summary:
 *    Callback function for drawing the label text of a tab.
 }
type
	ThemeTabTitleDrawProcPtr = procedure( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: URefCon );

{
 *  ThemeEraseProcPtr
 *  
 *  Summary:
 *    Callback function for erasing the background behind a theme
 *    primitive.
 }
type
	ThemeEraseProcPtr = procedure( const (*var*) bounds: Rect; eraseData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  ThemeButtonDrawProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the label text of a button.
 }
type
	ThemeButtonDrawProcPtr = procedure( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  WindowTitleDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the title text of a window.
 }
type
	WindowTitleDrawingProcPtr = procedure( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: URefCon );

{
 *  ThemeIteratorProcPtr
 *  
 *  Summary:
 *    Callback function for iterating over all known themes.
 }
type
>>>>>>> graemeg/cpstrnew
=======
 *  
 *  Summary:
 *    Callback function for drawing the label text of a tab.
 }
type
	ThemeTabTitleDrawProcPtr = procedure( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: URefCon );

{
 *  ThemeEraseProcPtr
 *  
 *  Summary:
 *    Callback function for erasing the background behind a theme
 *    primitive.
 }
type
	ThemeEraseProcPtr = procedure( const (*var*) bounds: Rect; eraseData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  ThemeButtonDrawProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the label text of a button.
 }
type
	ThemeButtonDrawProcPtr = procedure( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  WindowTitleDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the title text of a window.
 }
type
	WindowTitleDrawingProcPtr = procedure( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: URefCon );

{
 *  ThemeIteratorProcPtr
 *  
 *  Summary:
 *    Callback function for iterating over all known themes.
 }
type
>>>>>>> origin/cpstrnew
	ThemeIteratorProcPtr = function( const (*var*) inFileName: Str255; resID: SInt16; inThemeSettings: Collection; inUserData: PRefCon ): Boolean;
=======
type
	ThemeWindowMetricsPtr = ^ThemeWindowMetrics;

{
 *  Summary:
 *    Theme metrics allow you to find out sizes of things in the
 *    current environment, such as how wide a scroll bar is, etc.
 *  
 *  Discussion:
 *    ThemeMetrics
=======
type
=======
>>>>>>> origin/fixes_2.4
	ThemeWindowMetricsPtr = ^ThemeWindowMetrics;
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Drawing State                                                                            }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
type
	ThemeDrawingState = ^SInt32; { an opaque type }
	ThemeDrawingStatePtr = ^ThemeDrawingState;  { when a var xx:ThemeDrawingState parameter can be nil, it is changed to xx: ThemeDrawingStatePtr }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Callback procs                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  ThemeTabTitleDrawProcPtr
 *  
<<<<<<< HEAD
 *  Discussion:
 *    ThemeMetrics
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Callback function for drawing the label text of a tab.
>>>>>>> origin/fixes_2.4
 }
type
	ThemeTabTitleDrawProcPtr = procedure( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: URefCon );

{
 *  ThemeEraseProcPtr
 *  
 *  Summary:
 *    Callback function for erasing the background behind a theme
 *    primitive.
 }
type
	ThemeEraseProcPtr = procedure( const (*var*) bounds: Rect; eraseData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  ThemeButtonDrawProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the label text of a button.
 }
type
	ThemeButtonDrawProcPtr = procedure( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: URefCon; depth: SInt16; isColorDev: Boolean );

{
 *  WindowTitleDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the title text of a window.
 }
type
	WindowTitleDrawingProcPtr = procedure( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: URefCon );

{
 *  ThemeIteratorProcPtr
 *  
 *  Summary:
 *    Callback function for iterating over all known themes.
 }
type
<<<<<<< HEAD
	ThemeEraseProcPtr = procedure( const (*var*) bounds: Rect; eraseData: UInt32; depth: SInt16; isColorDev: Boolean );
type
	ThemeButtonDrawProcPtr = procedure( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: UInt32; depth: SInt16; isColorDev: Boolean );
type
	WindowTitleDrawingProcPtr = procedure( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: UInt32 );
type
	ThemeIteratorProcPtr = function( const (*var*) inFileName: Str255; resID: SInt16; inThemeSettings: Collection; inUserData: UnivPtr ): Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	ThemeIteratorProcPtr = function( const (*var*) inFileName: Str255; resID: SInt16; inThemeSettings: Collection; inUserData: PRefCon ): Boolean;
>>>>>>> origin/fixes_2.4
type
	ThemeTabTitleDrawUPP = ThemeTabTitleDrawProcPtr;
type
	ThemeEraseUPP = ThemeEraseProcPtr;
type
	ThemeButtonDrawUPP = ThemeButtonDrawProcPtr;
type
	WindowTitleDrawingUPP = WindowTitleDrawingProcPtr;
type
	ThemeIteratorUPP = ThemeIteratorProcPtr;
{
 *  NewThemeTabTitleDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewThemeTabTitleDrawUPP( userRoutine: ThemeTabTitleDrawProcPtr ): ThemeTabTitleDrawUPP; external name '_NewThemeTabTitleDrawUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  NewThemeEraseUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewThemeEraseUPP( userRoutine: ThemeEraseProcPtr ): ThemeEraseUPP; external name '_NewThemeEraseUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  NewThemeButtonDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewThemeButtonDrawUPP( userRoutine: ThemeButtonDrawProcPtr ): ThemeButtonDrawUPP; external name '_NewThemeButtonDrawUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  NewWindowTitleDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewWindowTitleDrawingUPP( userRoutine: WindowTitleDrawingProcPtr ): WindowTitleDrawingUPP; external name '_NewWindowTitleDrawingUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  NewThemeIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewThemeIteratorUPP( userRoutine: ThemeIteratorProcPtr ): ThemeIteratorUPP; external name '_NewThemeIteratorUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeThemeTabTitleDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeThemeTabTitleDrawUPP( userUPP: ThemeTabTitleDrawUPP ); external name '_DisposeThemeTabTitleDrawUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeThemeEraseUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeThemeEraseUPP( userUPP: ThemeEraseUPP ); external name '_DisposeThemeEraseUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeThemeButtonDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeThemeButtonDrawUPP( userUPP: ThemeButtonDrawUPP ); external name '_DisposeThemeButtonDrawUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeWindowTitleDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeWindowTitleDrawingUPP( userUPP: WindowTitleDrawingUPP ); external name '_DisposeWindowTitleDrawingUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeThemeIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeThemeIteratorUPP( userUPP: ThemeIteratorUPP ); external name '_DisposeThemeIteratorUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeThemeTabTitleDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure InvokeThemeTabTitleDrawUPP( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: URefCon; userUPP: ThemeTabTitleDrawUPP ); external name '_InvokeThemeTabTitleDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
procedure InvokeThemeTabTitleDrawUPP( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: UInt32; userUPP: ThemeTabTitleDrawUPP ); external name '_InvokeThemeTabTitleDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
procedure InvokeThemeTabTitleDrawUPP( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: UInt32; userUPP: ThemeTabTitleDrawUPP ); external name '_InvokeThemeTabTitleDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
procedure InvokeThemeTabTitleDrawUPP( const (*var*) bounds: Rect; style: ThemeTabStyle; direction: ThemeTabDirection; depth: SInt16; isColorDev: Boolean; userData: URefCon; userUPP: ThemeTabTitleDrawUPP ); external name '_InvokeThemeTabTitleDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeThemeEraseUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure InvokeThemeEraseUPP( const (*var*) bounds: Rect; eraseData: URefCon; depth: SInt16; isColorDev: Boolean; userUPP: ThemeEraseUPP ); external name '_InvokeThemeEraseUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
procedure InvokeThemeEraseUPP( const (*var*) bounds: Rect; eraseData: UInt32; depth: SInt16; isColorDev: Boolean; userUPP: ThemeEraseUPP ); external name '_InvokeThemeEraseUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
procedure InvokeThemeEraseUPP( const (*var*) bounds: Rect; eraseData: UInt32; depth: SInt16; isColorDev: Boolean; userUPP: ThemeEraseUPP ); external name '_InvokeThemeEraseUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
procedure InvokeThemeEraseUPP( const (*var*) bounds: Rect; eraseData: URefCon; depth: SInt16; isColorDev: Boolean; userUPP: ThemeEraseUPP ); external name '_InvokeThemeEraseUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeThemeButtonDrawUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure InvokeThemeButtonDrawUPP( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: URefCon; depth: SInt16; isColorDev: Boolean; userUPP: ThemeButtonDrawUPP ); external name '_InvokeThemeButtonDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
procedure InvokeThemeButtonDrawUPP( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: UInt32; depth: SInt16; isColorDev: Boolean; userUPP: ThemeButtonDrawUPP ); external name '_InvokeThemeButtonDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
procedure InvokeThemeButtonDrawUPP( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: UInt32; depth: SInt16; isColorDev: Boolean; userUPP: ThemeButtonDrawUPP ); external name '_InvokeThemeButtonDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
procedure InvokeThemeButtonDrawUPP( const (*var*) bounds: Rect; kind: ThemeButtonKind; const (*var*) info: ThemeButtonDrawInfo; userData: URefCon; depth: SInt16; isColorDev: Boolean; userUPP: ThemeButtonDrawUPP ); external name '_InvokeThemeButtonDrawUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeWindowTitleDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure InvokeWindowTitleDrawingUPP( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: URefCon; userUPP: WindowTitleDrawingUPP ); external name '_InvokeWindowTitleDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
procedure InvokeWindowTitleDrawingUPP( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: UInt32; userUPP: WindowTitleDrawingUPP ); external name '_InvokeWindowTitleDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
procedure InvokeWindowTitleDrawingUPP( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: UInt32; userUPP: WindowTitleDrawingUPP ); external name '_InvokeWindowTitleDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
procedure InvokeWindowTitleDrawingUPP( const (*var*) bounds: Rect; depth: SInt16; colorDevice: Boolean; userData: URefCon; userUPP: WindowTitleDrawingUPP ); external name '_InvokeWindowTitleDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeThemeIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
<<<<<<< HEAD
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeThemeIteratorUPP( const (*var*) inFileName: Str255; resID: SInt16; inThemeSettings: Collection; inUserData: PRefCon; userUPP: ThemeIteratorUPP ): Boolean; external name '_InvokeThemeIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Menu Drawing callbacks                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  MenuTitleDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the title text of a menu.
 }
type
	MenuTitleDrawingProcPtr = procedure( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SRefCon );

{
 *  MenuItemDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the text of a menu item.
 }
type
	MenuItemDrawingProcPtr = procedure( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SRefCon );
=======
=======
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeThemeIteratorUPP( const (*var*) inFileName: Str255; resID: SInt16; inThemeSettings: Collection; inUserData: PRefCon; userUPP: ThemeIteratorUPP ): Boolean; external name '_InvokeThemeIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Menu Drawing callbacks                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
 *  MenuTitleDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the title text of a menu.
 }
type
	MenuTitleDrawingProcPtr = procedure( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SRefCon );

{
 *  MenuItemDrawingProcPtr
 *  
 *  Summary:
 *    Callback function for drawing the text of a menu item.
 }
type
<<<<<<< HEAD
	MenuItemDrawingProcPtr = procedure( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SInt32 );
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	MenuItemDrawingProcPtr = procedure( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SRefCon );
>>>>>>> origin/fixes_2.4
type
	MenuTitleDrawingUPP = MenuTitleDrawingProcPtr;
type
	MenuItemDrawingUPP = MenuItemDrawingProcPtr;
{
 *  NewMenuTitleDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewMenuTitleDrawingUPP( userRoutine: MenuTitleDrawingProcPtr ): MenuTitleDrawingUPP; external name '_NewMenuTitleDrawingUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  NewMenuItemDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewMenuItemDrawingUPP( userRoutine: MenuItemDrawingProcPtr ): MenuItemDrawingUPP; external name '_NewMenuItemDrawingUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeMenuTitleDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeMenuTitleDrawingUPP( userUPP: MenuTitleDrawingUPP ); external name '_DisposeMenuTitleDrawingUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeMenuItemDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeMenuItemDrawingUPP( userUPP: MenuItemDrawingUPP ); external name '_DisposeMenuItemDrawingUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeMenuTitleDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure InvokeMenuTitleDrawingUPP( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SRefCon; userUPP: MenuTitleDrawingUPP ); external name '_InvokeMenuTitleDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
procedure InvokeMenuTitleDrawingUPP( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SInt32; userUPP: MenuTitleDrawingUPP ); external name '_InvokeMenuTitleDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
procedure InvokeMenuTitleDrawingUPP( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SInt32; userUPP: MenuTitleDrawingUPP ); external name '_InvokeMenuTitleDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
procedure InvokeMenuTitleDrawingUPP( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SRefCon; userUPP: MenuTitleDrawingUPP ); external name '_InvokeMenuTitleDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeMenuItemDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure InvokeMenuItemDrawingUPP( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SRefCon; userUPP: MenuItemDrawingUPP ); external name '_InvokeMenuItemDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
procedure InvokeMenuItemDrawingUPP( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SInt32; userUPP: MenuItemDrawingUPP ); external name '_InvokeMenuItemDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
procedure InvokeMenuItemDrawingUPP( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SInt32; userUPP: MenuItemDrawingUPP ); external name '_InvokeMenuItemDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
procedure InvokeMenuItemDrawingUPP( const (*var*) inBounds: Rect; inDepth: SInt16; inIsColorDevice: Boolean; inUserData: SRefCon; userUPP: MenuItemDrawingUPP ); external name '_InvokeMenuItemDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  Appearance Manager APIs                                                         }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Registering Appearance-Savvy Applications }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  RegisterAppearanceClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function does nothing on Mac OS X. Do not call it.
 *  
 *  Summary:
 *    Registers a process as a client of the Appearance Manager.
=======
{
 *  RegisterAppearanceClient()
>>>>>>> graemeg/fixes_2_2
=======
{
 *  RegisterAppearanceClient()
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
{
 *  RegisterAppearanceClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function does nothing on Mac OS X. Do not call it.
 *  
 *  Summary:
 *    Registers a process as a client of the Appearance Manager.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function RegisterAppearanceClient: OSStatus; external name '_RegisterAppearanceClient';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  UnregisterAppearanceClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function does nothing on Mac OS X. Do not call it.
 *  
 *  Summary:
 *    Unregisters a process as a client of the Appearance Manager.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UnregisterAppearanceClient()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  UnregisterAppearanceClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function does nothing on Mac OS X. Do not call it.
 *  
 *  Summary:
 *    Unregisters a process as a client of the Appearance Manager.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function UnregisterAppearanceClient: OSStatus; external name '_UnregisterAppearanceClient';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IsAppearanceClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function always returns true on Mac OS X. Do not call it.
 *  
 *  Summary:
 *    Indicates whether a process is registered with the Appearance
 *    Manager.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsAppearanceClient()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IsAppearanceClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function always returns true on Mac OS X. Do not call it.
 *  
 *  Summary:
 *    Indicates whether a process is registered with the Appearance
 *    Manager.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function IsAppearanceClient( const (*var*) process: ProcessSerialNumber ): Boolean; external name '_IsAppearanceClient';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{****************************************************************************
    NOTES ON THEME BRUSHES
    Theme brushes can be either colors or patterns, depending on the theme.
    Because of this, you should be prepared to handle the case where a brush
    is a pattern and save and restore the pnPixPat and bkPixPat fields of
    your GrafPorts when saving the fore and back colors. Also, since patterns
    in bkPixPat override the background color of the window, you should use
    BackPat to set your background pattern to a normal white pattern. This
    will ensure that you can use RGBBackColor to set your back color to white,
    call EraseRect and get the expected results.
****************************************************************************}

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  SetThemePen()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.4 and later, use HIThemeSetStroke, and draw using
 *    CoreGraphics.
 *  
 *  Summary:
 *    Sets the foreground color and/or pen pattern of the current
 *    GrafPort to allow drawing with a specified theme brush.
<<<<<<< HEAD
=======
 *  SetThemePen()
>>>>>>> graemeg/fixes_2_2
=======
 *  SetThemePen()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetThemePen( inBrush: ThemeBrush; inDepth: SInt16; inIsColorDevice: Boolean ): OSStatus; external name '_SetThemePen';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetThemeBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.4 and later, use HIThemeSetFill, and draw using
 *    CoreGraphics.
 *  
 *  Summary:
 *    Sets the background color and/or fill pattern of the current
 *    GrafPort to allow drawing with a specified theme brush.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThemeBackground()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetThemeBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.4 and later, use HIThemeSetFill, and draw using
 *    CoreGraphics.
 *  
 *  Summary:
 *    Sets the background color and/or fill pattern of the current
 *    GrafPort to allow drawing with a specified theme brush.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetThemeBackground( inBrush: ThemeBrush; inDepth: SInt16; inIsColorDevice: Boolean ): OSStatus; external name '_SetThemeBackground';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetThemeTextColor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.4 and later, use HIThemeTextFill, and draw with
 *    CoreGraphics, ATSUI, or HIThemeDrawTextBox.
 *  
 *  Summary:
 *    Sets the foreground color and/or pen pattern of the current
 *    GrafPort to allow drawing with a specified theme text color.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThemeTextColor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetThemeTextColor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.4 and later, use HIThemeTextFill, and draw with
 *    CoreGraphics, ATSUI, or HIThemeDrawTextBox.
 *  
 *  Summary:
 *    Sets the foreground color and/or pen pattern of the current
 *    GrafPort to allow drawing with a specified theme text color.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetThemeTextColor( inColor: ThemeTextColor; inDepth: SInt16; inIsColorDevice: Boolean ): OSStatus; external name '_SetThemeTextColor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Window Placards, Headers and Frames }
{
 *  DrawThemeWindowHeader()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawHeader.
 *  
 *  Summary:
 *    Draws a window header theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ Window Placards, Headers and Frames }
{
<<<<<<< HEAD
 *  DrawThemeWindowHeader()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DrawThemeWindowHeader()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawHeader.
 *  
 *  Summary:
 *    Draws a window header theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeWindowHeader( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemeWindowHeader';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeWindowListViewHeader()   *** DEPRECATED ***
<<<<<<< HEAD
=======
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawHeader.
 *  
 *  Summary:
 *    Draws a window header theme primitive that is suitable for use at
 *    the top of a list view.
>>>>>>> origin/fixes_2.4
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawHeader.
 *  
 *  Summary:
 *    Draws a window header theme primitive that is suitable for use at
 *    the top of a list view.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeWindowListViewHeader()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeWindowListViewHeader( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemeWindowListViewHeader';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemePlacard()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawPlacard.
 *  
 *  Summary:
 *    Draws a placard theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemePlacard()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemePlacard()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawPlacard.
 *  
 *  Summary:
 *    Draws a placard theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemePlacard( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemePlacard';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeEditTextFrame()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawFrame.
 *  
 *  Summary:
 *    Draws a frame theme primitive suitable for use around an edit
 *    text control.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeEditTextFrame()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeEditTextFrame()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawFrame.
 *  
 *  Summary:
 *    Draws a frame theme primitive suitable for use around an edit
 *    text control.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeEditTextFrame( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemeEditTextFrame';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeListBoxFrame()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawFrame.
 *  
 *  Summary:
 *    Draws a frame theme primitive suitable for use around a list box.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeListBoxFrame()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeListBoxFrame()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawFrame.
 *  
 *  Summary:
 *    Draws a frame theme primitive suitable for use around a list box.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeListBoxFrame( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemeListBoxFrame';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ Keyboard Focus Drawing }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  DrawThemeFocusRect()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawFocusRect. This API
 *    erases the focus ring when called with inHasFocus equal to false,
 *    but HIThemeDrawFocsuRect does not; to simulate the behavior of
 *    DrawThemeFocusRect in this case, simply erase the desired area
 *    yourself.
 *  
 *  Summary:
 *    Draws a focus ring around a specified rectangle.
<<<<<<< HEAD
=======
 *  DrawThemeFocusRect()
>>>>>>> graemeg/fixes_2_2
=======
 *  DrawThemeFocusRect()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeFocusRect( const (*var*) inRect: Rect; inHasFocus: Boolean ): OSStatus; external name '_DrawThemeFocusRect';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ Dialog Group Boxes and Separators }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  DrawThemePrimaryGroup()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawGroupBox.
 *  
 *  Summary:
 *    Draws a primary group theme primitive.
<<<<<<< HEAD
=======
 *  DrawThemePrimaryGroup()
>>>>>>> graemeg/fixes_2_2
=======
 *  DrawThemePrimaryGroup()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemePrimaryGroup( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemePrimaryGroup';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeSecondaryGroup()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawGroupBox.
 *  
 *  Summary:
 *    Draws a secondary group theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeSecondaryGroup()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeSecondaryGroup()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawGroupBox.
 *  
 *  Summary:
 *    Draws a secondary group theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeSecondaryGroup( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemeSecondaryGroup';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeSeparator()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawSeparator.
 *  
 *  Summary:
 *    Draws a separator theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeSeparator()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeSeparator()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawSeparator.
 *  
 *  Summary:
 *    Draws a separator theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeSeparator( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemeSeparator';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ BEGIN APPEARANCE 1.0.1 ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ The following Appearance Manager APIs are only available in Appearance 1.0.1 or later }
{
 *  DrawThemeModelessDialogFrame()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawWindowFrame.
 *  
 *  Summary:
 *    Draws a modeless dialog window frame theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ BEGIN APPEARANCE 1.0.1 ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ The following Appearance Manager APIs are only available in Appearance 1.0.1 or later }
{
<<<<<<< HEAD
 *  DrawThemeModelessDialogFrame()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DrawThemeModelessDialogFrame()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawWindowFrame.
 *  
 *  Summary:
 *    Draws a modeless dialog window frame theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeModelessDialogFrame( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemeModelessDialogFrame';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeGenericWell()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawGenericWell.
 *  
 *  Summary:
 *    Draws an image well theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeGenericWell()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeGenericWell()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawGenericWell.
 *  
 *  Summary:
 *    Draws an image well theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeGenericWell( const (*var*) inRect: Rect; inState: ThemeDrawState; inFillCenter: Boolean ): OSStatus; external name '_DrawThemeGenericWell';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeFocusRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawFocusRect. This API
 *    erases the focus ring when called with inHasFocus equal to false,
 *    but HIThemeDrawFocsuRect does not; to simulate the behavior of
 *    DrawThemeFocusRect in this case, simply erase the desired area
 *    yourself.
 *  
 *  Summary:
 *    Draws a focus ring around a specified region.
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeFocusRegion()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{
 *  DrawThemeFocusRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawFocusRect. This API
 *    erases the focus ring when called with inHasFocus equal to false,
 *    but HIThemeDrawFocsuRect does not; to simulate the behavior of
 *    DrawThemeFocusRect in this case, simply erase the desired area
 *    yourself.
 *  
 *  Summary:
 *    Draws a focus ring around a specified region.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeFocusRegion( inRegion: RgnHandle; inHasFocus: Boolean ): OSStatus; external name '_DrawThemeFocusRegion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IsThemeInColor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X, the Aqua theme is always drawn in color, and this
 *    API always returns true.
 *  
 *  Summary:
 *    Indicates whether the current theme would be drawn in color or
 *    black and white.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsThemeInColor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IsThemeInColor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X, the Aqua theme is always drawn in color, and this
 *    API always returns true.
 *  
 *  Summary:
 *    Indicates whether the current theme would be drawn in color or
 *    black and white.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function IsThemeInColor( inDepth: SInt16; inIsColorDevice: Boolean ): Boolean; external name '_IsThemeInColor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeAccentColors()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function always returns appearanceThemeHasNoAccents on Mac
 *    OS X. Do not call it.
 *  
 *  Summary:
 *    Returns the accent colors for the current theme.
 *  
 *  Discussion:
 *    GetThemeAccentColors will only work in the platinum theme on Mac
 *    OS 8.x and 9.x. It returns an error on Mac OS X, since the Aqua
 *    theme does not support accent colors.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{
<<<<<<< HEAD
 *  GetThemeAccentColors()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetThemeAccentColors()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function always returns appearanceThemeHasNoAccents on Mac
 *    OS X. Do not call it.
 *  
 *  Summary:
 *    Returns the accent colors for the current theme.
 *  
 *  Discussion:
 *    GetThemeAccentColors will only work in the platinum theme on Mac
 *    OS 8.x and 9.x. It returns an error on Mac OS X, since the Aqua
 *    theme does not support accent colors.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeAccentColors( var outColors: CTabHandle ): OSStatus; external name '_GetThemeAccentColors';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeMenuBarBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuBarBackground.
 *  
 *  Summary:
 *    Draws a menubar background theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeMenuBarBackground()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeMenuBarBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuBarBackground.
 *  
 *  Summary:
 *    Draws a menubar background theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuBarBackground( const (*var*) inBounds: Rect; inState: ThemeMenuBarState; inAttributes: UInt32 ): OSStatus; external name '_DrawThemeMenuBarBackground';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuTitle.
 *  
 *  Summary:
 *    Draws a menu title theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeMenuTitle()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuTitle.
 *  
 *  Summary:
 *    Draws a menu title theme primitive.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuTitle( const (*var*) inMenuBarRect: Rect; const (*var*) inTitleRect: Rect; inState: ThemeMenuState; inAttributes: UInt32; inTitleProc: MenuTitleDrawingUPP { can be NULL }; inTitleData: URefCon ): OSStatus; external name '_DrawThemeMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuTitle( const (*var*) inMenuBarRect: Rect; const (*var*) inTitleRect: Rect; inState: ThemeMenuState; inAttributes: UInt32; inTitleProc: MenuTitleDrawingUPP { can be NULL }; inTitleData: UInt32 ): OSStatus; external name '_DrawThemeMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuTitle( const (*var*) inMenuBarRect: Rect; const (*var*) inTitleRect: Rect; inState: ThemeMenuState; inAttributes: UInt32; inTitleProc: MenuTitleDrawingUPP { can be NULL }; inTitleData: URefCon ): OSStatus; external name '_DrawThemeMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{
 *  GetThemeMenuBarHeight()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Returns the height of the menubar, in points.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outHeight:
 *      On exit, contains the height of the menubar, in points.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Returns the height of the menubar, in points.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outHeight:
 *      On exit, contains the height of the menubar, in points.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuBarHeight( var outHeight: SInt16 ): OSStatus; external name '_GetThemeMenuBarHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  DrawThemeMenuBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuBackground.
 *  
 *  Summary:
 *    Draws a menu background theme primitive.
<<<<<<< HEAD
=======
 *  DrawThemeMenuBackground()
>>>>>>> graemeg/fixes_2_2
=======
 *  DrawThemeMenuBackground()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuBackground( const (*var*) inMenuRect: Rect; inMenuType: ThemeMenuType ): OSStatus; external name '_DrawThemeMenuBackground';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeMenuBackgroundRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeGetMenuBackgroundShape.
 *  
 *  Summary:
 *    Constructs a region that covers the entire area of a menu.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{
 *  GetThemeMenuBackgroundRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeGetMenuBackgroundShape.
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuBackgroundRegion( const (*var*) inMenuRect: Rect; menuType: ThemeMenuType; region: RgnHandle ): OSStatus; external name '_GetThemeMenuBackgroundRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeMenuItem()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Constructs a region that covers the entire area of a menu.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuBackgroundRegion( const (*var*) inMenuRect: Rect; menuType: ThemeMenuType; region: RgnHandle ): OSStatus; external name '_GetThemeMenuBackgroundRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeMenuItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuItem.
 *  
 *  Summary:
 *    Draws a menu item theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuBackgroundRegion( const (*var*) inMenuRect: Rect; menuType: ThemeMenuType; region: RgnHandle ): OSStatus; external name '_GetThemeMenuBackgroundRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeMenuItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuItem.
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuSeparator( const (*var*) inItemRect: Rect ): OSStatus; external name '_DrawThemeMenuSeparator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeMenuSeparatorHeight()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Draws a menu item theme primitive.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuItem( const (*var*) inMenuRect: Rect; const (*var*) inItemRect: Rect; inVirtualMenuTop: SInt16; inVirtualMenuBottom: SInt16; inState: ThemeMenuState; inItemType: ThemeMenuItemType; inDrawProc: MenuItemDrawingUPP { can be NULL }; inUserData: URefCon ): OSStatus; external name '_DrawThemeMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeMenuSeparator()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuSeparator.
 *  
 *  Summary:
 *    Draws a menu item separator theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuItem( const (*var*) inMenuRect: Rect; const (*var*) inItemRect: Rect; inVirtualMenuTop: SInt16; inVirtualMenuBottom: SInt16; inState: ThemeMenuState; inItemType: ThemeMenuItemType; inDrawProc: MenuItemDrawingUPP { can be NULL }; inUserData: URefCon ): OSStatus; external name '_DrawThemeMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeMenuSeparator()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawMenuSeparator.
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuItemExtra( inItemType: ThemeMenuItemType; var outHeight: SInt16; var outWidth: SInt16 ): OSStatus; external name '_GetThemeMenuItemExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeMenuTitleExtra()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Draws a menu item separator theme primitive.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuSeparator( const (*var*) inItemRect: Rect ): OSStatus; external name '_DrawThemeMenuSeparator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuTitleExtra( var outWidth: SInt16; inIsSquished: Boolean ): OSStatus; external name '_GetThemeMenuTitleExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function DrawThemeMenuSeparator( const (*var*) inItemRect: Rect ): OSStatus; external name '_DrawThemeMenuSeparator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ BEGIN APPEARANCE 1.1 ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ THEME SWITCHING ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  X ALERT: Please note that Get/SetTheme are severely neutered under Mac OS X at present.         }
{           See the note above regarding what collection tags are supported under X.               }

{
 *  GetTheme()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Returns information about the current theme.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    ioCollection:
 *      On entry, a valid empty collection. On exit, contains
 *      information about the current theme. See Theme Collection Tags
 *      for the set of collection tags that are used to retrieve
 *      information from this collection.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Returns information about the current theme.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    ioCollection:
 *      On entry, a valid empty collection. On exit, contains
 *      information about the current theme. See Theme Collection Tags
 *      for the set of collection tags that are used to retrieve
 *      information from this collection.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetTheme( ioCollection: Collection ): OSStatus; external name '_GetTheme';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  SetTheme()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This API does not modify the current theme in any way on Mac OS X.
 *  
 *  Summary:
 *    Sets information about the current theme.
<<<<<<< HEAD
=======
 *  SetTheme()
>>>>>>> graemeg/fixes_2_2
=======
 *  SetTheme()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function SetTheme( ioCollection: Collection ): OSStatus; external name '_SetTheme';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IterateThemes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This API does nothing on Mac OS X; it does not call the theme
 *    iterator callback function.
 *  
 *  Summary:
 *    Iterates over all known themes.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IterateThemes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IterateThemes()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This API does nothing on Mac OS X; it does not call the theme
 *    iterator callback function.
 *  
 *  Summary:
 *    Iterates over all known themes.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function IterateThemes( inProc: ThemeIteratorUPP; inUserData: PRefCon { can be NULL } ): OSStatus; external name '_IterateThemes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function IterateThemes( inProc: ThemeIteratorUPP; inUserData: UnivPtr { can be NULL } ): OSStatus; external name '_IterateThemes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function IterateThemes( inProc: ThemeIteratorUPP; inUserData: PRefCon { can be NULL } ): OSStatus; external name '_IterateThemes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ TABS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  DrawThemeTabPane()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTabPane.
 *  
 *  Summary:
 *    Draws a tab pane theme primitive.
<<<<<<< HEAD
=======
 *  DrawThemeTabPane()
>>>>>>> graemeg/fixes_2_2
=======
 *  DrawThemeTabPane()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTabPane( const (*var*) inRect: Rect; inState: ThemeDrawState ): OSStatus; external name '_DrawThemeTabPane';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeTab()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTab.
 *  
 *  Summary:
 *    Draws a tab theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeTab()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeTab()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTab.
 *  
 *  Summary:
 *    Draws a tab theme primitive.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTab( const (*var*) inRect: Rect; inStyle: ThemeTabStyle; inDirection: ThemeTabDirection; labelProc: ThemeTabTitleDrawUPP { can be NULL }; userData: URefCon ): OSStatus; external name '_DrawThemeTab';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTabRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTabDrawShape.
 *  
 *  Summary:
 *    Constructs a region that covers the area of a tab.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTab( const (*var*) inRect: Rect; inStyle: ThemeTabStyle; inDirection: ThemeTabDirection; labelProc: ThemeTabTitleDrawUPP { can be NULL }; userData: URefCon ): OSStatus; external name '_DrawThemeTab';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetThemeTabRegion()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetThemeTabRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTabDrawShape.
 *  
 *  Summary:
 *    Constructs a region that covers the area of a tab.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeTabRegion( const (*var*) inRect: Rect; inStyle: ThemeTabStyle; inDirection: ThemeTabDirection; ioRgn: RgnHandle ): OSStatus; external name '_GetThemeTabRegion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ CURSORS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  SetThemeCursor()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Sets the current cursor to a specified theme-compliant cursor.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inCursor:
 *      The cursor to set.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Sets the current cursor to a specified theme-compliant cursor.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inCursor:
 *      The cursor to set.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function SetThemeCursor( inCursor: ThemeCursor ): OSStatus; external name '_SetThemeCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetAnimatedThemeCursor()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Sets the current cursor to the next frame of a specified
 *    theme-compliant cursor.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inCursor:
 *      The cursor to set.
 *    
 *    inAnimationStep:
 *      A value indicating which frame of animation should be used. The
 *      exact value does not matter, but the value should increase by 1
 *      before each call to this API. Not all cursors support
 *      animation; those that do not will ignore this parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inCursor:
 *      The cursor to set.
 *    
 *    inAnimationStep:
 *      A value indicating which frame of animation should be used. The
 *      exact value does not matter, but the value should increase by 1
 *      before each call to this API. Not all cursors support
 *      animation; those that do not will ignore this parameter.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function SetAnimatedThemeCursor( inCursor: ThemeCursor; inAnimationStep: UInt32 ): OSStatus; external name '_SetAnimatedThemeCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ CONTROL STYLE SETTINGS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  GetThemeScrollBarThumbStyle()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Retrieves the current scrollbar thumb style, which may be
 *    proportional or non-proportional.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outStyle:
 *      On exit, contains the current scrollbar thumb style.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outStyle:
 *      On exit, contains the current scrollbar thumb style.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeScrollBarThumbStyle( var outStyle: ThemeScrollBarThumbStyle ): OSStatus; external name '_GetThemeScrollBarThumbStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeScrollBarArrowStyle()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Retrieves the current scrollbar arrow style. Arrows may be single
 *    at each end of the scrollbar, double at one end, or double at
 *    both ends.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outStyle:
 *      On exit, contains the current scrollbar arrow style.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outStyle:
 *      On exit, contains the current scrollbar arrow style.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeScrollBarArrowStyle( var outStyle: ThemeScrollBarArrowStyle ): OSStatus; external name '_GetThemeScrollBarArrowStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeCheckBoxStyle()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Retrieves the current checkbox style, which may use an X or a
 *    checkmark.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outStyle:
 *      On exit, contains the current checkbox style.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outStyle:
 *      On exit, contains the current checkbox style.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeCheckBoxStyle( var outStyle: ThemeCheckBoxStyle ): OSStatus; external name '_GetThemeCheckBoxStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ FONTS/TEXT ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  UseThemeFont()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use HIThemeDrawTextBox instead.
 *  
 *  Summary:
 *    Sets the font of the current GrafPort to allow drawing with a
 *    specified theme font.
<<<<<<< HEAD
=======
 *  UseThemeFont()
>>>>>>> graemeg/fixes_2_2
=======
 *  UseThemeFont()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function UseThemeFont( inFontID: ThemeFontID; inScript: ScriptCode ): OSStatus; external name '_UseThemeFont';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeFont()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Some theme fonts cannot be drawn using QuickDraw. Use
 *    HIThemeDrawTextBox instead.
 *  
 *  Summary:
 *    Retrieves the QuickDraw font name, size, and style for a
 *    specified theme font.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeFont()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeFont()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Some theme fonts cannot be drawn using QuickDraw. Use
 *    HIThemeDrawTextBox instead.
 *  
 *  Summary:
 *    Retrieves the QuickDraw font name, size, and style for a
 *    specified theme font.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeFont( inFontID: ThemeFontID; inScript: ScriptCode; outFontName: StringPtr { can be NULL }; var outFontSize: SInt16; var outStyle: Style ): OSStatus; external name '_GetThemeFont';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeTextBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTextBox.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeTextBox()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeTextBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTextBox.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Draws text into the area you specify.
 *  
 *  Discussion:
 *    DrawThemeTextBox allows you to draw theme-savvy (ie. Aqua-savvy
 *    on Mac OS X) text. It is unicode savvy (although only partially
 *    so under CarbonLib), and allows you to customize certain text
 *    rendering characteristics such as the font, wrapping behavior,
 *    and justification. The text is drawn into the CGContextRef you
 *    provide, or into the current Quickdraw port if no CGContextRef is
 *    provided. None of DrawThemeTextBox's parameters imply a color, so
 *    you must set up the desired text color separately before calling
 *    DrawThemeTextBox. If you provide a CGContextRef, its fill color
 *    will be used to draw the text. If you do not provide a
 *    CGContextRef, a color based on the current Quickdraw port's
 *    foreground color and the grayishTextOr mode (if set) will be used
 *    to draw the text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inString:
 *      A CFStringRef containing the unicode characters you wish to
 *      render. You MUST NOT pass in a CFStringRef that was allocated
 *      with any of the "NoCopy" CFString creation APIs; a string
 *      created with a "NoCopy" API has transient storage which is
 *      incompatible with DrawThemeTextBox's caches.
 *    
 *    inFontID:
 *      The ThemeFontID describing the font you'd like to render the
 *      text with. See the discussion of ThemeFontIDs elsewhere in this
 *      header.
 *    
 *    inState:
 *      The ThemeDrawState describing the the state of the interface
 *      element you are drawing the text for. If, for example, you are
 *      drawing text for an inactive window, you would pass
 *      kThemeStateInactive. The ThemeDrawState is generally only used
 *      to determine the shadow characteristics for the text on Mac OS
 *      X. Note that the ThemeDrawState does NOT imply a color. It is
 *      NOT used as a mechanism for graying the text. If you wish to
 *      draw grayed text, you must set up the desired gray color and
 *      apply it to either the current Quickdraw port or the
 *      CGContextRef as appropriate.
 *    
 *    inWrapToWidth:
 *      A Boolean indicating whether you want to draw multiple lines of
 *      text wrapped to a bounding box. False indicates that only one
 *      line of text should be drawn without any sort of wrapping.
 *    
 *    inBoundingBox:
 *      The rectangle (in coordinates relative to the current Quickdraw
 *      port) describing the area to draw the text within. The first
 *      line of text will be top-justified to this rectangle. Wrapping
 *      (if desired) will happen at the horizontal extent of this
 *      rectangle. Regardless of the amount of text in your
 *      CFStringRef, all drawn text will be clipped to this rectangle.
 *    
 *    inJust:
 *      The horizontal justification you would like for your text. You
 *      can use one of the standard justification constants from
 *      TextEdit.h.
 *    
 *    inContext:
 *      The CGContextRef into which you would like to draw the text. On
 *      Mac OS X, all text drawing happens in CGContextRefs; if you
 *      pass NULL, a transient CGContextRef will be allocated and
 *      deallocated for use within the single API call. Relying on the
 *      system behavior if transiently creating CGContextRefs may
 *      result in performance problems. On Mac OS 9, the CGContextRef
 *      parameter is ignored.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function DrawThemeTextBox( inString: CFStringRef; inFontID: ThemeFontID; inState: ThemeDrawState; inWrapToWidth: Boolean; const (*var*) inBoundingBox: Rect; inJust: SInt16; inContext: CGContextRef ): OSStatus; external name '_DrawThemeTextBox';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TruncateThemeText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTextDimensions or
 *    HIThemeDrawTextBox. There is currently no replacement API that
 *    actually alters the string.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function DrawThemeTextBox( inString: CFStringRef; inFontID: ThemeFontID; inState: ThemeDrawState; inWrapToWidth: Boolean; const (*var*) inBoundingBox: Rect; inJust: SInt16; inContext: CGContextRef ): OSStatus; external name '_DrawThemeTextBox';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  TruncateThemeText()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  TruncateThemeText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTextDimensions or
 *    HIThemeDrawTextBox. There is currently no replacement API that
 *    actually alters the string.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Truncates text to fit within the width you specify.
 *  
 *  Discussion:
 *    TruncateThemeText alters a unicode string to fit within a width
 *    that you specify. It is unicode savvy (although only partially so
 *    under CarbonLib), and makes its calculations (and any subsequent
 *    string alteration) based on the font and state you specify. If
 *    the string needs to be truncated, it will be reduced to the
 *    maximum number of characters which (with the addition of an
 *    ellipsis character) fits within the specified width.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inString:
 *      A CFMutableStringRef containing the unicode characters you wish
 *      to truncate. On output, inString may have been altered to fit
 *      within the specified width. You MUST NOT pass in a CFStringRef
 *      that was allocated with any of the "NoCopy" CFString creation
 *      APIs (see note in DrawThemeTextBox above).
 *    
 *    inFontID:
 *      The ThemeFontID to use for text measurements. See the
 *      discussion of ThemeFontIDs elsewhere in this header.
 *    
 *    inState:
 *      The ThemeDrawState which matches the state you will ultimately
 *      render the string width. This may affect text measurement
 *      during truncation, so you should be sure the value you pass to
 *      TruncateThemeText matches the value you will eventually use for
 *      drawing.
 *    
 *    inPixelWidthLimit:
 *      The maximum width (in pixels) that the resulting truncated
 *      string may have.
 *    
 *    inTruncWhere:
 *      A TruncCode indicating where you would like truncation to occur.
 *    
 *    outTruncated:
 *      On output, this Boolean value indicates whether the string was
 *      truncated. True means the string was truncated. False means the
 *      string was not (and did not need to be) truncated.
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
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function TruncateThemeText( inString: CFMutableStringRef; inFontID: ThemeFontID; inState: ThemeDrawState; inPixelWidthLimit: SInt16; inTruncWhere: TruncCode; outTruncated: BooleanPtr { can be NULL } ): OSStatus; external name '_TruncateThemeText';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTextDimensions()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTextDimensions.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeTextDimensions()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTextDimensions()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTextDimensions.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Tells you the height, width, and baseline for a string.
 *  
 *  Discussion:
 *    GetThemeTextDimensions measures the given string using a font and
 *    state you specify. It always reports the actual height and
 *    baseline. It sometimes reports the actual width (see below). It
 *    can measure a string that wraps. It is unicode savvy (although
 *    only partially so under CarbonLib).
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inString:
 *      A CFStringRef containing the unicode characters you wish to
 *      measure. You MUST NOT pass in a CFStringRef that was allocated
 *      with any of the "NoCopy" CFString creation APIs (see note in
 *      DrawThemeTextBox above).
 *    
 *    inFontID:
 *      The ThemeFontID describing the font you'd like to measure the
 *      text with. See the discussion of ThemeFontIDs elsewhere in this
 *      header.
 *    
 *    inState:
 *      The ThemeDrawState which matches the state you will ultimately
 *      render the string width. This may affect text measurement, so
 *      you should be sure the value you pass to TruncateThemeText
 *      matches the value you will eventually use for drawing.
 *    
 *    inWrapToWidth:
 *      A Boolean indicating whether you want the measurements based on
 *      wrapping the text to a specific width. If you pass true, you
 *      must specify the desired width in ioBounds->h.
 *    
 *    ioBounds:
 *      On output, ioBounds->v contains the height of the text. If you
 *      pass false to inWrapToWidth, ioBounds->h will contain the width
 *      of the text on output. If you pass true to inWrapToWidth,
 *      ioBounds->h must (on input) contain the desired width for
 *      wrapping; on output, ioBounds->h contains the same value you
 *      specified on input.
 *    
 *    outBaseline:
 *      On output, outBaseline contains the offset (in Quickdraw space)
 *      from the bottom edge of the last line of text to the baseline
 *      of the first line of text. outBaseline will generally be a
 *      negative value. On Mac OS X 10.2 and later, you may pass NULL
 *      if you don't want this information.
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
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function GetThemeTextDimensions( inString: CFStringRef; inFontID: ThemeFontID; inState: ThemeDrawState; inWrapToWidth: Boolean; var ioBounds: Point; outBaseline: SInt16Ptr { can be NULL } ): OSStatus; external name '_GetThemeTextDimensions';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{
 *  GetThemeTextShadowOutset()
 *  
 *  Summary:
 *    Tells you the amount of space taken up by the shadow for a given
 *    font/state combination.
 *  
 *  Discussion:
 *    GetThemeTextShadowOutset passes back the maximum amount of space
 *    the shadow will take up for text drawn in the specified font and
 *    state. While GetThemeTextDimensions tells you how much space is
 *    taken up by the character glyphs themselves, it does not
 *    incorporate the font/state shadow into its calculations. If you
 *    need to know how much total space including the shadow will be
 *    taken up, call GetThemeTextDimensions followed by
 *    GetThemeTextShadowOutset.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inFontID:
 *      The ThemeFontID describing the font you'd like the shadow
 *      characteristics of. Font and state both determine the amount of
 *      shadow that will be used on rendered text. See the discussion
 *      of ThemeFontIDs elsewhere in this header.
 *    
 *    inState:
 *      The ThemeDrawState which matches the state you'd like the
 *      shadow characteristics of. Font and state both determine the
 *      amount of shadow that will be used on rendered text.
 *    
 *    outOutset:
 *      On output, outOutset contains the amount of space the shadow
 *      will take up beyond each edge of the text bounding rectangle
 *      returned by GetThemeTextDimensions. The fields of outOutset
 *      will either be positive values or zero.
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
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function GetThemeTextShadowOutset( inFontID: ThemeFontID; inState: ThemeDrawState; var outOutset: Rect ): OSStatus; external name '_GetThemeTextShadowOutset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ TRACKS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  DrawThemeTrack()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTrack.
 *  
 *  Summary:
 *    Draws a track theme primitive, such as a slider or scrollbar.
<<<<<<< HEAD
=======
 *  DrawThemeTrack()
>>>>>>> graemeg/fixes_2_2
=======
 *  DrawThemeTrack()
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTrack( const (*var*) drawInfo: ThemeTrackDrawInfo; rgnGhost: RgnHandle { can be NULL }; eraseProc: ThemeEraseUPP { can be NULL }; eraseData: URefCon ): OSStatus; external name '_DrawThemeTrack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  HitTestThemeTrack()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeHitTestTrack.
 *  
 *  Summary:
 *    Determines the part of a track that is is under a specified point.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTrack( const (*var*) drawInfo: ThemeTrackDrawInfo; rgnGhost: RgnHandle { can be NULL }; eraseProc: ThemeEraseUPP { can be NULL }; eraseData: URefCon ): OSStatus; external name '_DrawThemeTrack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  HitTestThemeTrack()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  HitTestThemeTrack()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeHitTestTrack.
 *  
 *  Summary:
 *    Determines the part of a track that is is under a specified point.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function HitTestThemeTrack( const (*var*) drawInfo: ThemeTrackDrawInfo; mousePoint: Point; var partHit: AppearancePartCode ): Boolean; external name '_HitTestThemeTrack';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTrackBounds.
 *  
 *  Summary:
 *    Retrieves the bounds of a track theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeTrackBounds()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTrackBounds.
 *  
 *  Summary:
 *    Retrieves the bounds of a track theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeTrackBounds( const (*var*) drawInfo: ThemeTrackDrawInfo; var bounds: Rect ): OSStatus; external name '_GetThemeTrackBounds';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackThumbRgn()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTrackThumbShape.
 *  
 *  Summary:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Constructs a region that covers the area of a track thumb.
=======
 *    Constructs a region that covers the ara of a track thumb.
>>>>>>> graemeg/cpstrnew
=======
 *    Constructs a region that covers the ara of a track thumb.
>>>>>>> graemeg/cpstrnew
=======
 *    Constructs a region that covers the ara of a track thumb.
>>>>>>> graemeg/cpstrnew
=======
 *    Constructs a region that covers the ara of a track thumb.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeTrackThumbRgn()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackThumbRgn()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTrackThumbShape.
 *  
 *  Summary:
 *    Constructs a region that covers the ara of a track thumb.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeTrackThumbRgn( const (*var*) drawInfo: ThemeTrackDrawInfo; thumbRgn: RgnHandle ): OSStatus; external name '_GetThemeTrackThumbRgn';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackDragRect()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTrackDragRect.
 *  
 *  Summary:
 *    Retrieves the bounds of a track dragging area.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeTrackDragRect()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackDragRect()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTrackDragRect.
 *  
 *  Summary:
 *    Retrieves the bounds of a track dragging area.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeTrackDragRect( const (*var*) drawInfo: ThemeTrackDrawInfo; var dragRect: Rect ): OSStatus; external name '_GetThemeTrackDragRect';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeTrackTickMarks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTrackTickMarks.
 *  
 *  Summary:
 *    Draws the tick marks for a track theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeTrackTickMarks()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeTrackTickMarks()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTrackTickMarks.
 *  
 *  Summary:
 *    Draws the tick marks for a track theme primitive.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTrackTickMarks( const (*var*) drawInfo: ThemeTrackDrawInfo; numTicks: ItemCount; eraseProc: ThemeEraseUPP { can be NULL }; eraseData: URefCon ): OSStatus; external name '_DrawThemeTrackTickMarks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackThumbPositionFromOffset()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use
 *    HIThemeGetTrackThumbPositionFromOffset.
 *  
 *  Summary:
 *    Retrieves a track's thumb position based on its offset in the
 *    track bounds.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTrackTickMarks( const (*var*) drawInfo: ThemeTrackDrawInfo; numTicks: ItemCount; eraseProc: ThemeEraseUPP { can be NULL }; eraseData: URefCon ): OSStatus; external name '_DrawThemeTrackTickMarks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetThemeTrackThumbPositionFromOffset()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetThemeTrackThumbPositionFromOffset()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use
 *    HIThemeGetTrackThumbPositionFromOffset.
 *  
 *  Summary:
 *    Retrieves a track's thumb position based on its offset in the
 *    track bounds.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeTrackThumbPositionFromOffset( const (*var*) drawInfo: ThemeTrackDrawInfo; thumbOffset: Point; var relativePosition: SInt32 ): OSStatus; external name '_GetThemeTrackThumbPositionFromOffset';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackThumbPositionFromRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use
 *    HIThemeGetTrackThumbPositionFromBounds.
 *  
 *  Summary:
 *    Retrieves a track's thumb position based on a region covering the
 *    thumb.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeTrackThumbPositionFromRegion()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackThumbPositionFromRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use
 *    HIThemeGetTrackThumbPositionFromBounds.
 *  
 *  Summary:
 *    Retrieves a track's thumb position based on a region covering the
 *    thumb.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeTrackThumbPositionFromRegion( const (*var*) drawInfo: ThemeTrackDrawInfo; thumbRgn: RgnHandle; var relativePosition: SInt32 ): OSStatus; external name '_GetThemeTrackThumbPositionFromRegion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackLiveValue()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTrackLiveValue.
 *  
 *  Summary:
 *    Retrieves the value that a track would have based on a thumb
 *    position.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeTrackLiveValue()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeTrackLiveValue()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetTrackLiveValue.
 *  
 *  Summary:
 *    Retrieves the value that a track would have based on a thumb
 *    position.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeTrackLiveValue( const (*var*) drawInfo: ThemeTrackDrawInfo; relativePosition: SInt32; var value: SInt32 ): OSStatus; external name '_GetThemeTrackLiveValue';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ SCROLLBAR ARROWS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  DrawThemeScrollBarArrows()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTrack, which will draw
 *    the entire scrollbar, including both the track and arrows.
 *  
 *  Summary:
 *    Draws the arrows at each or both ends of a scrollbar theme
 *    primitive.
<<<<<<< HEAD
=======
 *  DrawThemeScrollBarArrows()
>>>>>>> graemeg/fixes_2_2
=======
 *  DrawThemeScrollBarArrows()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeScrollBarArrows( const (*var*) bounds: Rect; enableState: ThemeTrackEnableState; pressState: ThemeTrackPressState; isHoriz: Boolean; var trackBounds: Rect ): OSStatus; external name '_DrawThemeScrollBarArrows';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeScrollBarTrackRect()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetScrollBarTrackRect.
 *  
 *  Summary:
 *    Retrieves the bounds of a scrollbar track.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeScrollBarTrackRect()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeScrollBarTrackRect()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetScrollBarTrackRect.
 *  
 *  Summary:
 *    Retrieves the bounds of a scrollbar track.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeScrollBarTrackRect( const (*var*) bounds: Rect; enableState: ThemeTrackEnableState; pressState: ThemeTrackPressState; isHoriz: Boolean; var trackBounds: Rect ): OSStatus; external name '_GetThemeScrollBarTrackRect';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  HitTestThemeScrollBarArrows()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeHitTestScrollBarArrows.
 *  
 *  Summary:
 *    Determines which scrollbar arrow is under a specified point.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HitTestThemeScrollBarArrows()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  HitTestThemeScrollBarArrows()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeHitTestScrollBarArrows.
 *  
 *  Summary:
 *    Determines which scrollbar arrow is under a specified point.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function HitTestThemeScrollBarArrows( const (*var*) scrollBarBounds: Rect; enableState: ThemeTrackEnableState; pressState: ThemeTrackPressState; isHoriz: Boolean; ptHit: Point; var trackBounds: Rect; var partcode: AppearancePartCode ): Boolean; external name '_HitTestThemeScrollBarArrows';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ WINDOWS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  GetThemeWindowRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetWindowShape.
 *  
 *  Summary:
 *    Constructs a region that covers a specified part of a window.
<<<<<<< HEAD
=======
 *  GetThemeWindowRegion()
>>>>>>> graemeg/fixes_2_2
=======
 *  GetThemeWindowRegion()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeWindowRegion( flavor: ThemeWindowType; const (*var*) contRect: Rect; state: ThemeDrawState; const (*var*) metrics: ThemeWindowMetrics; attributes: ThemeWindowAttributes; winRegion: AppearanceRegionCode; rgn: RgnHandle ): OSStatus; external name '_GetThemeWindowRegion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeWindowFrame()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawWindowFrame.
 *  
 *  Summary:
 *    Draws a window frame theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeWindowFrame()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeWindowFrame()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawWindowFrame.
 *  
 *  Summary:
 *    Draws a window frame theme primitive.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeWindowFrame( flavor: ThemeWindowType; const (*var*) contRect: Rect; state: ThemeDrawState; const (*var*) metrics: ThemeWindowMetrics; attributes: ThemeWindowAttributes; titleProc: WindowTitleDrawingUPP { can be NULL }; titleData: URefCon ): OSStatus; external name '_DrawThemeWindowFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeTitleBarWidget()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTitleBarWidget.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeWindowFrame( flavor: ThemeWindowType; const (*var*) contRect: Rect; state: ThemeDrawState; const (*var*) metrics: ThemeWindowMetrics; attributes: ThemeWindowAttributes; titleProc: WindowTitleDrawingUPP { can be NULL }; titleData: URefCon ): OSStatus; external name '_DrawThemeWindowFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  DrawThemeTitleBarWidget()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DrawThemeTitleBarWidget()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTitleBarWidget.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Draws the requested theme title bar widget.
 *  
 *  Discussion:
 *    DrawThemeTitleBarWidget renders the requested theme title bar
 *    widget in the proper location of a window.  A common
 *    misconception when using this API is that the client must specify
 *    the exact location of the widget in the window. The widget will
 *    locate itself in the window based relative to the content rect
 *    passed in content rectangle -- the contRect parameter.  Another
 *    common problem is to ignore the window's attributes.  The
 *    attributes must be set up properly to describe the window for
 *    which the widget is to be drawn.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    flavor:
 *      A valid ThemeWindowtype describing the type of theme window for
 *      which you would like to draw a widget.
 *    
 *    contRect:
 *      A rectangle describing the window's content area.  The widget
 *      is drawn relative to the content rectangle of the window, so
 *      this parameter does not describe the actual widget bounds, it
 *      describes the window's content rectangle.
 *    
 *    state:
 *      A valid ThemeDrawState which describes the state of the window
 *      for which the widget is to be drawn.
 *    
 *    metrics:
 *      A pointer to a set of valid ThemeWindowMetrics.  At this time,
 *      none of the fields of the metrics are pertinent to the widgets,
 *      so the only important field is the metricSize field to mark the
 *      structure as valid.
 *    
 *    attributes:
 *      A valid ThemeWindowAttributes set which describes the window
 *      for which the widget is to be drawn.
 *    
 *    widget:
 *      A valid ThemeTitleBarWidget set which describes which widget to
 *      draw.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTitleBarWidget( flavor: ThemeWindowType; const (*var*) contRect: Rect; state: ThemeDrawState; const (*var*) metrics: ThemeWindowMetrics; attributes: ThemeWindowAttributes; widget: ThemeTitleBarWidget ): OSStatus; external name '_DrawThemeTitleBarWidget';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeWindowRegionHit()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetWindowRegionHit.
 *  
 *  Summary:
 *    Determines the window part that is under a specified point.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeWindowRegionHit()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeWindowRegionHit()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetWindowRegionHit.
 *  
 *  Summary:
 *    Determines the window part that is under a specified point.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeWindowRegionHit( flavor: ThemeWindowType; const (*var*) inContRect: Rect; state: ThemeDrawState; const (*var*) metrics: ThemeWindowMetrics; inAttributes: ThemeWindowAttributes; inPoint: Point; var outRegionHit: AppearanceRegionCode ): Boolean; external name '_GetThemeWindowRegionHit';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeScrollBarDelimiters()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawScrollBarDelimiters.
 *  
 *  Summary:
 *    Draws the scrollbar delimeters in a window frame.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeScrollBarDelimiters()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeScrollBarDelimiters()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use HIThemeDrawScrollBarDelimiters.
 *  
 *  Summary:
 *    Draws the scrollbar delimeters in a window frame.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeScrollBarDelimiters( flavor: ThemeWindowType; const (*var*) inContRect: Rect; state: ThemeDrawState; attributes: ThemeWindowAttributes ): OSStatus; external name '_DrawThemeScrollBarDelimiters';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ BUTTONS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  DrawThemeButton()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawButton.
 *  
 *  Summary:
 *    Draws a button theme primitive.
<<<<<<< HEAD
=======
 *  DrawThemeButton()
>>>>>>> graemeg/fixes_2_2
=======
 *  DrawThemeButton()
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeButton( const (*var*) inBounds: Rect; inKind: ThemeButtonKind; const (*var*) inNewInfo: ThemeButtonDrawInfo; {const} inPrevInfo: ThemeButtonDrawInfoPtr { can be NULL }; inEraseProc: ThemeEraseUPP { can be NULL }; inLabelProc: ThemeButtonDrawUPP { can be NULL }; inUserData: URefCon ): OSStatus; external name '_DrawThemeButton';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeButtonRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetButtonShape.
 *  
 *  Summary:
 *    Constructs a region that covers the area of a button.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeButton( const (*var*) inBounds: Rect; inKind: ThemeButtonKind; const (*var*) inNewInfo: ThemeButtonDrawInfo; {const} inPrevInfo: ThemeButtonDrawInfoPtr { can be NULL }; inEraseProc: ThemeEraseUPP { can be NULL }; inLabelProc: ThemeButtonDrawUPP { can be NULL }; inUserData: URefCon ): OSStatus; external name '_DrawThemeButton';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetThemeButtonRegion()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetThemeButtonRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetButtonShape.
 *  
 *  Summary:
 *    Constructs a region that covers the area of a button.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeButtonRegion( const (*var*) inBounds: Rect; inKind: ThemeButtonKind; const (*var*) inNewInfo: ThemeButtonDrawInfo; outRegion: RgnHandle ): OSStatus; external name '_GetThemeButtonRegion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeButtonContentBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetButtonContentBounds.
 *  
 *  Summary:
 *    Retrieves the bounds of a button's content.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeButtonContentBounds()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeButtonContentBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetButtonContentBounds.
 *  
 *  Summary:
 *    Retrieves the bounds of a button's content.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeButtonContentBounds( const (*var*) inBounds: Rect; inKind: ThemeButtonKind; const (*var*) inDrawInfo: ThemeButtonDrawInfo; var outBounds: Rect ): OSStatus; external name '_GetThemeButtonContentBounds';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeButtonBackgroundBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetButtonBackgroundBounds.
 *  
 *  Summary:
 *    Retrieves the bounds of a button's background.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeButtonBackgroundBounds()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeButtonBackgroundBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetButtonBackgroundBounds.
 *  
 *  Summary:
 *    Retrieves the bounds of a button's background.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeButtonBackgroundBounds( const (*var*) inBounds: Rect; inKind: ThemeButtonKind; const (*var*) inDrawInfo: ThemeButtonDrawInfo; var outBounds: Rect ): OSStatus; external name '_GetThemeButtonBackgroundBounds';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ INTERFACE SOUNDS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  X ALERT: Please note that the sound APIs do not work on Mac OS X at present.                    }
{
 *  PlayThemeSound()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Plays a theme-compliant sound. Note that this API does not play
 *    sound on any version of Mac OS X.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    kind:
 *      The kind of sound to play.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    kind:
 *      The kind of sound to play.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function PlayThemeSound( kind: ThemeSoundKind ): OSStatus; external name '_PlayThemeSound';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  BeginThemeDragSound()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Begins playing a theme-compliant sound to indicate that a drag is
 *    occuring. Note that this API does not play sound on any version
 *    of Mac OS X.
 *  
 *  Discussion:
 *    You should call EndThemeDragSound when the drag is complete.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    kind:
 *      The kind of drag sound to play.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    kind:
 *      The kind of drag sound to play.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function BeginThemeDragSound( kind: ThemeDragSoundKind ): OSStatus; external name '_BeginThemeDragSound';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EndThemeDragSound()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Stops playing a theme-compliant sound to indicate that a drag is
 *    occuring.
 *  
 *  Discussion:
 *    This API should only be called after BeginThemeDragSound has been
 *    called.
 *  
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function EndThemeDragSound: OSStatus; external name '_EndThemeDragSound';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ PRIMITIVES ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  DrawThemeTickMark()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawTickMark.
 *  
 *  Summary:
 *    Draws a tick mark theme primitive.
<<<<<<< HEAD
=======
 *  DrawThemeTickMark()
>>>>>>> graemeg/fixes_2_2
=======
 *  DrawThemeTickMark()
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeTickMark( const (*var*) bounds: Rect; state: ThemeDrawState ): OSStatus; external name '_DrawThemeTickMark';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeChasingArrows()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawChasingArrows.
 *  
 *  Summary:
 *    Draws a chasing arrows theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeChasingArrows()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeChasingArrows()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawChasingArrows.
 *  
 *  Summary:
 *    Draws a chasing arrows theme primitive.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeChasingArrows( const (*var*) bounds: Rect; theIndex: UInt32; state: ThemeDrawState; eraseProc: ThemeEraseUPP { can be NULL }; eraseData: URefCon ): OSStatus; external name '_DrawThemeChasingArrows';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemePopupArrow()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawPopupArrow.
 *  
 *  Summary:
 *    Draws a popup arrow theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeChasingArrows( const (*var*) bounds: Rect; theIndex: UInt32; state: ThemeDrawState; eraseProc: ThemeEraseUPP { can be NULL }; eraseData: URefCon ): OSStatus; external name '_DrawThemeChasingArrows';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  DrawThemePopupArrow()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DrawThemePopupArrow()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawPopupArrow.
 *  
 *  Summary:
 *    Draws a popup arrow theme primitive.
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemePopupArrow( const (*var*) bounds: Rect; orientation: ThemeArrowOrientation; size: ThemePopupArrowSize; state: ThemeDrawState; eraseProc: ThemeEraseUPP { can be NULL }; eraseData: URefCon ): OSStatus; external name '_DrawThemePopupArrow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeStandaloneGrowBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawGrowBox.
 *  
 *  Summary:
 *    Draws a grow box theme primitive that is designed to be detached
 *    from the window frame.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemePopupArrow( const (*var*) bounds: Rect; orientation: ThemeArrowOrientation; size: ThemePopupArrowSize; state: ThemeDrawState; eraseProc: ThemeEraseUPP { can be NULL }; eraseData: URefCon ): OSStatus; external name '_DrawThemePopupArrow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  DrawThemeStandaloneGrowBox()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DrawThemeStandaloneGrowBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawGrowBox.
 *  
 *  Summary:
 *    Draws a grow box theme primitive that is designed to be detached
 *    from the window frame.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeStandaloneGrowBox( origin: Point; growDirection: ThemeGrowDirection; isSmall: Boolean; state: ThemeDrawState ): OSStatus; external name '_DrawThemeStandaloneGrowBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeStandaloneNoGrowBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawGrowBox.
 *  
 *  Summary:
 *    Draws a blank area that can be used to fill in the corner of two
 *    scrollbars, when no grow box is required.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DrawThemeStandaloneNoGrowBox()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DrawThemeStandaloneNoGrowBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeDrawGrowBox.
 *  
 *  Summary:
 *    Draws a blank area that can be used to fill in the corner of two
 *    scrollbars, when no grow box is required.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DrawThemeStandaloneNoGrowBox( origin: Point; growDirection: ThemeGrowDirection; isSmall: Boolean; state: ThemeDrawState ): OSStatus; external name '_DrawThemeStandaloneNoGrowBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeStandaloneGrowBoxBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetGrowBoxBounds.
 *  
 *  Summary:
 *    Retrieves the bounds of a standalone grow box theme primitive.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeStandaloneGrowBoxBounds()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetThemeStandaloneGrowBoxBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeGetGrowBoxBounds.
 *  
 *  Summary:
 *    Retrieves the bounds of a standalone grow box theme primitive.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeStandaloneGrowBoxBounds( origin: Point; growDirection: ThemeGrowDirection; isSmall: Boolean; var bounds: Rect ): OSStatus; external name '_GetThemeStandaloneGrowBoxBounds';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ DRAWING STATE ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ The following routines help you save and restore the drawing state in a theme-savvy manner. With }
{ these weapons in your arsenal, there is no grafport you cannot tame. Use ThemeGetDrawingState to }
{ get the current drawing settings for the current port. It will return an opaque object for you   }
{ to pass into SetThemeDrawingState later on. When you are finished with the state, call the       }
{ DisposeThemeDrawingState routine. You can alternatively pass true into the inDisposeNow          }
{ parameter of the SetThemeDrawingState routine.  You can use this routine to copy the drawing     }
{ state from one port to another as well.                                                          }
{ As of this writing (Mac OS 9.1 and Mac OS X), Get/SetThemeDrawingState will save and restore     }
{ this data in the port:                                                                           }
{      pen size                                                                                    }
{      pen location                                                                                }
{      pen mode                                                                                    }
{      pen Pattern and PixPat                                                                      }
{      background Pattern and PixPat                                                               }
{      RGB foreground and background colors                                                        }
{      text mode                                                                                   }
{      pattern origin                                                                              }
{ Get/SetThemeDrawingState may save and restore additional port state in the future, but you can   }
{ rely on them to always save at least this port state.                                            }
{
 *  NormalizeThemeDrawingState()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Normalizes the drawing state of the current GrafPort.
 *  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Normalizes the drawing state of the current GrafPort.
 *  
>>>>>>> origin/fixes_2.4
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function NormalizeThemeDrawingState: OSStatus; external name '_NormalizeThemeDrawingState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeDrawingState()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Retrieves the drawing state of the current port.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outState:
 *      On exit, contains the saved drawing state. This state must be
 *      released by calling either DisposeThemeDrawingState or
 *      SetThemeDrawingState with inDisposeNow equal to true.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Retrieves the drawing state of the current port.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outState:
 *      On exit, contains the saved drawing state. This state must be
 *      released by calling either DisposeThemeDrawingState or
 *      SetThemeDrawingState with inDisposeNow equal to true.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeDrawingState( var outState: ThemeDrawingState ): OSStatus; external name '_GetThemeDrawingState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThemeDrawingState()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Restores saved drawing state of the current port.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inState:
 *      The state to restore.
 *    
 *    inDisposeNow:
 *      Whether the drawing state should be disposed by this API.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Restores saved drawing state of the current port.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inState:
 *      The state to restore.
 *    
 *    inDisposeNow:
 *      Whether the drawing state should be disposed by this API.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function SetThemeDrawingState( inState: ThemeDrawingState; inDisposeNow: Boolean ): OSStatus; external name '_SetThemeDrawingState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeThemeDrawingState()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Releases the memory used by a saved drawing state object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inState:
 *      The state to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Releases the memory used by a saved drawing state object.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inState:
 *      The state to release.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function DisposeThemeDrawingState( inState: ThemeDrawingState ): OSStatus; external name '_DisposeThemeDrawingState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ MISCELLANEOUS ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
 *  ApplyThemeBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeApplyBackground.
 *  
 *  Summary:
 *    Sets up the background color and/or pattern in the current
 *    GrafPort so that an embedded control may erase its content bounds
 *    appropriately for its containing control.
 *  
 *  Discussion:
 *    This API is normally called by embedding controls in response to
 *    a kEventControlApplyBackground Carbon event. It is only needed
 *    when an embedding control is contained in a non-composited
 *    window. The standard controls call this API to ensure a correct
 *    background for the current theme prior to erasing their content
 *    bounds (in a non-composited window). You should pass in the same
 *    rectangle you would use if you were calling the drawing primitive
 *    that corresponds to the ThemeBackgroundKind that you provide to
 *    this API.
=======
=======
>>>>>>> origin/fixes_2_2
{ ApplyThemeBackground is used to set up the background for embedded controls  }
{ It is normally called by controls that are embedders. The standard controls  }
{ call this API to ensure a correct background for the current theme. You pass }
{ in the same rectangle you would if you were calling the drawing primitive.   }
{
 *  ApplyThemeBackground()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  ApplyThemeBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X 10.3 and later, use HIThemeApplyBackground.
 *  
 *  Summary:
 *    Sets up the background color and/or pattern in the current
 *    GrafPort so that an embedded control may erase its content bounds
 *    appropriately for its containing control.
 *  
 *  Discussion:
 *    This API is normally called by embedding controls in response to
 *    a kEventControlApplyBackground Carbon event. It is only needed
 *    when an embedding control is contained in a non-composited
 *    window. The standard controls call this API to ensure a correct
 *    background for the current theme prior to erasing their content
 *    bounds (in a non-composited window). You should pass in the same
 *    rectangle you would use if you were calling the drawing primitive
 *    that corresponds to the ThemeBackgroundKind that you provide to
 *    this API.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Parameters:
 *    
 *    inKind:
 *      The type of theme background for which to set the current
 *      GrafPort's background color and/or pattern.
 *    
 *    bounds:
 *      The bounds of the area that would be covered by the theme
 *      background. This parameter is used to determine the appropriate
 *      pattern offset, if necessary.
 *    
 *    inState:
 *      The theme drawing state that should be used by the theme
 *      background.
 *    
 *    inDepth:
 *      The depth of the GDevice containing the current GrafPort. On
 *      Mac OS X, this parameter is ignored, and you should always pass
 *      32.
 *    
 *    inColorDev:
 *      Whether the GDevice containing the current GrafPort is color or
 *      black and white. On Mac OS X, this parameter is ignored, and
 *      you should always pass true.
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function ApplyThemeBackground( inKind: ThemeBackgroundKind; const (*var*) bounds: Rect; inState: ThemeDrawState; inDepth: SInt16; inColorDev: Boolean ): OSStatus; external name '_ApplyThemeBackground';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IsValidAppearanceFileType()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This API always returns false on Mac OS X.
 *  
 *  Summary:
 *    Indicates whether a specified file type is used for theme files.
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetThemeTextColorForWindow() has moved to MacWindows.h
 }
{
 *  IsValidAppearanceFileType()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IsValidAppearanceFileType()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This API always returns false on Mac OS X.
 *  
 *  Summary:
 *    Indicates whether a specified file type is used for theme files.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function IsValidAppearanceFileType( fileType: OSType ): Boolean; external name '_IsValidAppearanceFileType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{
 *  GetThemeBrushAsColor()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Retrieves the color that most closely matches a theme brush.
 *  
 *  Discussion:
 *    Note that the color may not exactly reproduce the theme brush,
 *    since many brushes are drawn using a pattern rather than a solid
 *    color.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBrush:
 *      The brush whose color to retrieve.
 *    
 *    inDepth:
 *      The depth of the GDevice containing the current GrafPort. On
 *      Mac OS X, this parameter is ignored, and you should always pass
 *      32.
 *    
 *    inColorDev:
 *      Whether the GDevice containing the current GrafPort is color or
 *      black and white. On Mac OS X, this parameter is ignored, and
 *      you should always pass true.
 *    
 *    outColor:
 *      On exit, contains the color that corresponds to the specified
 *      brush.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBrush:
 *      The brush whose color to retrieve.
 *    
 *    inDepth:
 *      The depth of the GDevice containing the current GrafPort. On
 *      Mac OS X, this parameter is ignored, and you should always pass
 *      32.
 *    
 *    inColorDev:
 *      Whether the GDevice containing the current GrafPort is color or
 *      black and white. On Mac OS X, this parameter is ignored, and
 *      you should always pass true.
 *    
 *    outColor:
 *      On exit, contains the color that corresponds to the specified
 *      brush.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeBrushAsColor( inBrush: ThemeBrush; inDepth: SInt16; inColorDev: Boolean; var outColor: RGBColor ): OSStatus; external name '_GetThemeBrushAsColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeTextColor()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Retrieves the color that is used to draw a specified type of text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inColor:
 *      The text color to retrieve.
 *    
 *    inDepth:
 *      The depth of the GDevice containing the current GrafPort. On
 *      Mac OS X, this parameter is ignored, and you should always pass
 *      32.
 *    
 *    inColorDev:
 *      Whether the GDevice containing the current GrafPort is color or
 *      black and white. On Mac OS X, this parameter is ignored, and
 *      you should always pass true.
 *    
 *    outColor:
 *      On exit, contains the specified text color.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Retrieves the color that is used to draw a specified type of text.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inColor:
 *      The text color to retrieve.
 *    
 *    inDepth:
 *      The depth of the GDevice containing the current GrafPort. On
 *      Mac OS X, this parameter is ignored, and you should always pass
 *      32.
 *    
 *    inColorDev:
 *      Whether the GDevice containing the current GrafPort is color or
 *      black and white. On Mac OS X, this parameter is ignored, and
 *      you should always pass true.
 *    
 *    outColor:
 *      On exit, contains the specified text color.
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
 *    Non-Carbon CFM:   in AppearanceLib 1.1 and later
 }
function GetThemeTextColor( inColor: ThemeTextColor; inDepth: SInt16; inColorDev: Boolean; var outColor: RGBColor ): OSStatus; external name '_GetThemeTextColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{--------------------------------------- MOVED ROUTINES -------------------------------------------}
{ The following routines have been moved to HITheme.h                                              }
{ GetThemeMenuSeparatorHeight                                                                      }
{ GetThemeMenuItemExtra                                                                            }
{ GetThemeMenuTitleExtra                                                                           }
{ GetThemeMetric                                                                                   }
{ CopyThemeIdentifier                                                                              }
{--------------------------------------------------------------------------------------------------}
<<<<<<< HEAD

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ BEGIN CARBON ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
=======
=======
>>>>>>> origin/fixes_2_2
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ BEGIN CARBON ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{
 *  GetThemeMetric()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetThemeMetric( inMetric: ThemeMetric; var outMetric: SInt32 ): OSStatus; external name '_GetThemeMetric';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyThemeIdentifier()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function CopyThemeIdentifier( var outIdentifier: CFStringRef ): OSStatus; external name '_CopyThemeIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ BEGIN CARBON ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
>>>>>>> origin/fixes_2.4
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Obsolete symbolic names                                                                          }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Obsolete error codes - use the new ones, s'il vous plait / kudasai }
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
const
	appearanceBadBrushIndexErr = themeInvalidBrushErr; { pattern index invalid }
	appearanceProcessRegisteredErr = themeProcessRegisteredErr;
	appearanceProcessNotRegisteredErr = themeProcessNotRegisteredErr;
	appearanceBadTextColorIndexErr = themeBadTextColorErr;
	appearanceThemeHasNoAccents = themeHasNoAccentsErr;
	appearanceBadCursorIndexErr = themeBadCursorIndexErr;

const
	kThemeActiveDialogBackgroundBrush = kThemeBrushDialogBackgroundActive;
	kThemeInactiveDialogBackgroundBrush = kThemeBrushDialogBackgroundInactive;
	kThemeActiveAlertBackgroundBrush = kThemeBrushAlertBackgroundActive;
	kThemeInactiveAlertBackgroundBrush = kThemeBrushAlertBackgroundInactive;
	kThemeActiveModelessDialogBackgroundBrush = kThemeBrushModelessDialogBackgroundActive;
	kThemeInactiveModelessDialogBackgroundBrush = kThemeBrushModelessDialogBackgroundInactive;
	kThemeActiveUtilityWindowBackgroundBrush = kThemeBrushUtilityWindowBackgroundActive;
	kThemeInactiveUtilityWindowBackgroundBrush = kThemeBrushUtilityWindowBackgroundInactive;
	kThemeListViewSortColumnBackgroundBrush = kThemeBrushListViewSortColumnBackground;
	kThemeListViewBackgroundBrush = kThemeBrushListViewBackground;
	kThemeIconLabelBackgroundBrush = kThemeBrushIconLabelBackground;
	kThemeListViewSeparatorBrush = kThemeBrushListViewSeparator;
	kThemeChasingArrowsBrush = kThemeBrushChasingArrows;
	kThemeDragHiliteBrush = kThemeBrushDragHilite;
	kThemeDocumentWindowBackgroundBrush = kThemeBrushDocumentWindowBackground;
	kThemeFinderWindowBackgroundBrush = kThemeBrushFinderWindowBackground;

const
	kThemeActiveScrollBarDelimiterBrush = kThemeBrushScrollBarDelimiterActive;
	kThemeInactiveScrollBarDelimiterBrush = kThemeBrushScrollBarDelimiterInactive;
	kThemeFocusHighlightBrush = kThemeBrushFocusHighlight;
	kThemeActivePopupArrowBrush = kThemeBrushPopupArrowActive;
	kThemePressedPopupArrowBrush = kThemeBrushPopupArrowPressed;
	kThemeInactivePopupArrowBrush = kThemeBrushPopupArrowInactive;
	kThemeAppleGuideCoachmarkBrush = kThemeBrushAppleGuideCoachmark;

const
	kThemeActiveDialogTextColor = kThemeTextColorDialogActive;
	kThemeInactiveDialogTextColor = kThemeTextColorDialogInactive;
	kThemeActiveAlertTextColor = kThemeTextColorAlertActive;
	kThemeInactiveAlertTextColor = kThemeTextColorAlertInactive;
	kThemeActiveModelessDialogTextColor = kThemeTextColorModelessDialogActive;
	kThemeInactiveModelessDialogTextColor = kThemeTextColorModelessDialogInactive;
	kThemeActiveWindowHeaderTextColor = kThemeTextColorWindowHeaderActive;
	kThemeInactiveWindowHeaderTextColor = kThemeTextColorWindowHeaderInactive;
	kThemeActivePlacardTextColor = kThemeTextColorPlacardActive;
	kThemeInactivePlacardTextColor = kThemeTextColorPlacardInactive;
	kThemePressedPlacardTextColor = kThemeTextColorPlacardPressed;
	kThemeActivePushButtonTextColor = kThemeTextColorPushButtonActive;
	kThemeInactivePushButtonTextColor = kThemeTextColorPushButtonInactive;
	kThemePressedPushButtonTextColor = kThemeTextColorPushButtonPressed;
	kThemeActiveBevelButtonTextColor = kThemeTextColorBevelButtonActive;
	kThemeInactiveBevelButtonTextColor = kThemeTextColorBevelButtonInactive;
	kThemePressedBevelButtonTextColor = kThemeTextColorBevelButtonPressed;
	kThemeActivePopupButtonTextColor = kThemeTextColorPopupButtonActive;
	kThemeInactivePopupButtonTextColor = kThemeTextColorPopupButtonInactive;
	kThemePressedPopupButtonTextColor = kThemeTextColorPopupButtonPressed;
	kThemeIconLabelTextColor = kThemeTextColorIconLabel;
	kThemeListViewTextColor = kThemeTextColorListView;

const
	kThemeActiveDocumentWindowTitleTextColor = kThemeTextColorDocumentWindowTitleActive;
	kThemeInactiveDocumentWindowTitleTextColor = kThemeTextColorDocumentWindowTitleInactive;
	kThemeActiveMovableModalWindowTitleTextColor = kThemeTextColorMovableModalWindowTitleActive;
	kThemeInactiveMovableModalWindowTitleTextColor = kThemeTextColorMovableModalWindowTitleInactive;
	kThemeActiveUtilityWindowTitleTextColor = kThemeTextColorUtilityWindowTitleActive;
	kThemeInactiveUtilityWindowTitleTextColor = kThemeTextColorUtilityWindowTitleInactive;
	kThemeActivePopupWindowTitleColor = kThemeTextColorPopupWindowTitleActive;
	kThemeInactivePopupWindowTitleColor = kThemeTextColorPopupWindowTitleInactive;
	kThemeActiveRootMenuTextColor = kThemeTextColorRootMenuActive;
	kThemeSelectedRootMenuTextColor = kThemeTextColorRootMenuSelected;
	kThemeDisabledRootMenuTextColor = kThemeTextColorRootMenuDisabled;
	kThemeActiveMenuItemTextColor = kThemeTextColorMenuItemActive;
	kThemeSelectedMenuItemTextColor = kThemeTextColorMenuItemSelected;
	kThemeDisabledMenuItemTextColor = kThemeTextColorMenuItemDisabled;
	kThemeActivePopupLabelTextColor = kThemeTextColorPopupLabelActive;
	kThemeInactivePopupLabelTextColor = kThemeTextColorPopupLabelInactive;

const
	kAEThemeSwitch = kAEAppearanceChanged; { Event ID's: Theme Switched }

const
	kThemeNoAdornment = kThemeAdornmentNone;
	kThemeDefaultAdornment = kThemeAdornmentDefault;
	kThemeFocusAdornment = kThemeAdornmentFocus;
	kThemeRightToLeftAdornment = kThemeAdornmentRightToLeft;
	kThemeDrawIndicatorOnly = kThemeAdornmentDrawIndicatorOnly;

const
	kThemeBrushPassiveAreaFill = kThemeBrushStaticAreaFill;

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	kThemeMetricCheckBoxGlyphHeight = kThemeMetricCheckBoxHeight;
	kThemeMetricRadioButtonGlyphHeight = kThemeMetricRadioButtonHeight;
	kThemeMetricDisclosureButtonSize = kThemeMetricDisclosureButtonHeight;
	kThemeMetricBestListHeaderHeight = kThemeMetricListHeaderHeight;
	kThemeMetricSmallProgressBarThickness = kThemeMetricNormalProgressBarThickness; { obsolete }
	kThemeMetricProgressBarThickness = kThemeMetricLargeProgressBarThickness; { obsolete }

const
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kThemeScrollBar = kThemeMediumScrollBar;
	kThemeSlider = kThemeMediumSlider;
	kThemeProgressBar = kThemeMediumProgressBar;
	kThemeIndeterminateBar = kThemeMediumIndeterminateBar;

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
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

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
