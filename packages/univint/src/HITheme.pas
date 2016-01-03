{
     File:       HIToolbox/HITheme.h
 
     Contains:   HIToolbox HITheme interfaces.
 
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
 
     Copyright:  © 1994-2008 by Apple Computer, Inc., all rights reserved.
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1994-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1994-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 1994-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/fixes_2_2
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/fixes_2_2
 
}
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/fixes_2.4
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

unit HITheme;
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
uses MacTypes,CFBase,CGBase,Appearance,HIShape,HIGeometry,Drag,CFDate,CGContext,MacWindows,Controls,CTFont;
{$endc} {not MACOSALLINCLUDE}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,CFBase,CGBase,Appearance,HIShape,HIGeometry,Drag,CFDate,CGContext,MacWindows,Controls;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CGBase,Appearance,HIShape,HIGeometry,Drag,CFDate,CGContext,MacWindows,Controls;
=======
uses MacTypes,CFBase,CGBase,Appearance,HIShape,HIGeometry,Drag,CFDate,CGContext,MacWindows,Controls,CTFont;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ -------------------------------------------------------------------------- }
{  HIThemeOrientation information                                            }
{ -------------------------------------------------------------------------- }

const
{
   * The passed context has an origin at the top left. This is the
   * default of a context passed to you by HIToolbox.
   }
	kHIThemeOrientationNormal = 0;

  {
   * The passed context has an origin at the bottom left. This is the
   * default for a context you create.
   }
	kHIThemeOrientationInverted = 1;

type
	HIThemeOrientation = UInt32;
{ -------------------------------------------------------------------------- }
{  Splitter types                                                            }
{ -------------------------------------------------------------------------- }

const
{
   * Draw the splitter with its normal appearance.
   }
	kHIThemeSplitterAdornmentNone = 0;

  {
   * Draw the splitter with its metal appearance.
   }
	kHIThemeSplitterAdornmentMetal = 1;

type
	HIThemeSplitterAdornment = UInt32;
{ -------------------------------------------------------------------------- }
{  Window Grow Box                                                           }
{ -------------------------------------------------------------------------- }

const
{
   * The grow box corner for a window that has no scroll bars.
   }
	kHIThemeGrowBoxKindNormal = 0;

  {
   * The grow box corner for a window that has no grow box. This sounds
   * paradoxical, but this type of grow box, formerly known as the
   * "NoGrowBox" is used to fill in the corner left blank by the
   * intersection of a horizontal and a vertical scroll bar.
   }
	kHIThemeGrowBoxKindNone = 1;

type
	HIThemeGrowBoxKind = UInt32;

const
{
   * Draw the grow box for normal windows.
   }
	kHIThemeGrowBoxSizeNormal = 0;

  {
   * Draw the smaller grow box for utility or floating windows.
   }
	kHIThemeGrowBoxSizeSmall = 1;

type
	HIThemeGrowBoxSize = UInt32;

const
{
   * The group box is drawn with the primary variant.
   }
	kHIThemeGroupBoxKindPrimary = 0;

  {
   * The group box is drawn with the secondary variant.
   }
	kHIThemeGroupBoxKindSecondary = 1;

  {
   * The group box is drawn with the primary variant. This group box
   * draws opaque. This does not match the Mac OS X 10.3 appearance
   * 100%, as the boxes should be transparent, but draws this way for
   * the sake of compatibility. Please update to use the newer
   * transparent variant.
   }
	kHIThemeGroupBoxKindPrimaryOpaque = 3;

  {
   * The group box is drawn with the secondary variant. This group box
   * draws opaque. This does not match the Mac OS X 10.3 appearance
   * 100%, as the boxes should be transparent, but draws this way for
   * the sake of compatibility. Please update to use the newer
   * transparent variant.
   }
	kHIThemeGroupBoxKindSecondaryOpaque = 4;

type
	HIThemeGroupBoxKind = UInt32;

const
{
   * A header drawn above window content that has no top border of its
   * own. (i.e. the same as the status bar in an icon view Finder
   * window).
   }
	kHIThemeHeaderKindWindow = 0;

  {
   * A header drawn above window content that has a top border of its
   * own. (i.e. the same as the status bar in an list view Finder
   * window).
   }
	kHIThemeHeaderKindList = 1;

type
	HIThemeHeaderKind = UInt32;

const
{
   * The default sized square text field (like Edit Text).
   }
	kHIThemeFrameTextFieldSquare = 0;
	kHIThemeFrameListBox = 1;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  {
   * The standard sized round text field, as typically used for search
   * fields. Available on Mac OS X 10.3 and later.
   }
	kHIThemeFrameTextFieldRound = 1000;

  {
   * The small size round text field, as typically used for search
   * fields. Available on Mac OS X 10.3 and later.
   }
	kHIThemeFrameTextFieldRoundSmall = 1001;

  {
   * The mini size round text field, as typically used for search
   * fields. Available on Mac OS X 10.3 and later.
   }
	kHIThemeFrameTextFieldRoundMini = 1002;

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
type
	HIThemeFrameKind = UInt32;

const
{
   * Indicates that a menu title should be drawn in a condensed
   * appearance. This constant is used in the
   * HIThemeMenuTitleDrawInfo.attributes field.
   }
	kHIThemeMenuTitleDrawCondensed = 1 shl 0;

{ -------------------------------------------------------------------------- }
{  DrawInfo                                                                  }
{ -------------------------------------------------------------------------- }

{
 *  HIScrollBarTrackInfo
 *  
 *  Summary:
 *    Drawing parameters passed to scroll bar drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIScrollBarTrackInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeTrackEnableState for the scroll bar to be drawn.
   }
		enableState: SInt8 {ThemeTrackEnableState};

  {
   * The ThemeTrackPressState for the scroll bar to be drawn.
   }
		pressState: SInt8 {ThemeTrackPressState};

  {
   * The view range size.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		viewsize: CGFloat;
=======
		viewsize: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		viewsize: Float32;
>>>>>>> origin/fixes_2_2
=======
		viewsize: CGFloat;
>>>>>>> origin/fixes_2.4
	end;
	HIScrollBarTrackInfoPtr = ^HIScrollBarTrackInfo;

{
 *  HIThemeTrackDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to track drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3, but based on legacy TrackDrawInfo.
 }
type
	HIThemeTrackDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;                { current version is 0 }

  {
   * The ThemeTrackKind of the track being drawn or measured.
   }
		kind: ThemeTrackKind;                   { what kind of track this info is for }

  {
   * An HIRect describing the bounds of the track being drawn or
   * measured.
   }
		bounds: HIRect;                 { track basis rectangle }

  {
   * The minimum allowable value for the track being drawn or measured.
   }
		min: SInt32;                    { min track value }

  {
   * The maximum allowable value for the track being drawn or measured.
   }
		max: SInt32;                    { max track value }

  {
   * The value for the track being drawn or measured.
   }
		value: SInt32;                  { current thumb value }

  {
   * Leave this reserved field set to 0.
   }
		reserved: UInt32;


  {
   * A set of ThemeTrackAttributes for the track to be drawn or
   * measured.
   }
		attributes: ThemeTrackAttributes;           { various track attributes }

  {
   * A ThemeTrackEnableState describing the state of the track to be
   * drawn or measured.
   }
		enableState: SInt8 {ThemeTrackEnableState};         { enable state }

  {
   * Leave this reserved field set to 0.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		filler1: UInt8;

		case SInt16 of
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
		filler1: SInt8;

	case SInt16 of
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
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
	HIThemeTrackDrawInfoPtr = ^HIThemeTrackDrawInfo;

{
 *  HIThemeAnimationTimeInfo
 *  
 *  Summary:
 *    Time parameters passed to button drawing and measuring theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeAnimationTimeInfo = record
{
   * The CFAbsoluteTime of the beginning of the animation of the
   * button.  This only applies to buttons that animate -- currently
   * only kThemePushButton.  All other buttons will ignore this field. 
   * If there is to be no animation, set this field to 0.
   }
		start: CFAbsoluteTime;

  {
   * The CFAbsoluteTime of the current animation frame of the button. 
   * This only applies to buttons that animate -- currently only
   * kThemePushButton.  All other buttons will ignore this field.  If
   * there is to be no animation, set this field to 0.
   }
		current: CFAbsoluteTime;
	end;
	HIThemeAnimationTimeInfoPtr = ^HIThemeAnimationTimeInfo;

{
 *  HIThemeAnimationFrameInfo
 *  
 *  Summary:
 *    Frame parameters passed to button drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeAnimationFrameInfo = record
{
   * The index of the frame of the animation to draw. If the index is
   * greater that the maximum number of animation frames, it will be
   * modded to calculate which frame to draw.
   }
		index: UInt32;
	end;
	HIThemeAnimationFrameInfoPtr = ^HIThemeAnimationFrameInfo;

{
 *  HIThemeButtonDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to button drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeButtonDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState of the button being drawn or measured.
   }
		state: ThemeDrawState;

  {
   * A ThemeButtonKind indicating the type of button to be drawn.
   }
		kind: ThemeButtonKind;

  {
   * The ThemeButtonValue of the button being drawn or measured.
   }
		value: ThemeButtonValue;

  {
   * The ThemeButtonAdornment(s) with which the button is being drawn
   * or measured.
   }
		adornment: ThemeButtonAdornment;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		case SInt16 of
			0: (
					time: HIThemeAnimationTimeInfo;
			   );
			1: (
					frame: HIThemeAnimationFrameInfo;
				);
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	case SInt16 of
		0: (
				time: HIThemeAnimationTimeInfo;
		   );
		1: (
				frame: HIThemeAnimationFrameInfo;
			);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
	HIThemeButtonDrawInfoPtr = ^HIThemeButtonDrawInfo;

{
 *  HIThemeSplitterDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to splitter drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeSplitterDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState of the splitter being drawn or measured.
   }
		state: ThemeDrawState;

  {
   * The HIThemeSplitterAdornments of the splitter being drawn or
   * measured.
   }
		adornment: HIThemeSplitterAdornment;
	end;
	HIThemeSplitterDrawInfoPtr = ^HIThemeSplitterDrawInfo;

const
{
   * No tab adornments are to be drawn.
   }
	kHIThemeTabAdornmentNone = 0;

  {
   * A focus ring is to be drawn around the tab.
   }
	kHIThemeTabAdornmentFocus = 1 shl 2; { to match button focus adornment }

  {
   * If available, a leading separator is to be drawn on the tab,
   * either to the left or above, depending on orientation. Note that
   * tabs overlap and if the separators are drawn over top each other,
   * the shadows multiply undesirably. New in Mac OS X 10.4.
   }
	kHIThemeTabAdornmentLeadingSeparator = 1 shl 3;

  {
   * If available, a right separator is to be drawn on the tab, either
   * to the right or below, depending on the orientation. Note that
   * tabs overlap and if the separators are drawn over top each other,
   * the shadows multiply undesirably. New in Mac OS X 10.4.
   }
	kHIThemeTabAdornmentTrailingSeparator = 1 shl 4;

type
	HIThemeTabAdornment = UInt32;

{
 *  Summary:
 *    These values are similar to kControlSize constants for
 *    convenience.
 }
const
{
   * The tabs are normal (large) sized.
   }
	kHIThemeTabSizeNormal = 0;

  {
   * The tabs are drawn as the small variant.
   }
	kHIThemeTabSizeSmall = 1;

  {
   * The tabs are drawn as the mini variant.
   }
	kHIThemeTabSizeMini = 3;

type
	HIThemeTabSize = UInt32;

{
 *  Summary:
 *    Available values for HIThemeTabPosition. These are positions of
 *    the tabs within the tab control. New in Mac OS X 10.4.
 }
const
{
   * The first position of a tab control. Left or top tab.
   }
	kHIThemeTabPositionFirst = 0;

  {
   * A middle tab.
   }
	kHIThemeTabPositionMiddle = 1;

  {
   * The last position of a tab control. Right or bottom tab.
   }
	kHIThemeTabPositionLast = 2;

  {
   * The only position of a tab control. It is simultaneously first and
   * last. You know, only. There is only one tab. It looks pretty much
   * like a button. Please don't use this if you can avoid it. It's
   * ugly.
   }
	kHIThemeTabPositionOnly = 3;

type
	HIThemeTabPosition = UInt32;

{
 *  Summary:
 *    Available values for HIThemeTabKind.
 }
const
	kHIThemeTabKindNormal = 0;

type
	HIThemeTabKind = UInt32;

{
 *  HIThemeTabDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to tab drawing and measuring theme APIs.
 *  
 *  Discussion:
 *    In Mac OS X 10.4, added kind and position fields.
 }
type
	HIThemeTabDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 1.
   }
		version: UInt32;

  {
   * A ThemeTabStyle describing the style of the tab to be drawn.
   }
		style: ThemeTabStyle;

  {
   * A ThemeTabDirection describing the side on which the tab is being
   * drawn.
   }
		direction: ThemeTabDirection;

  {
   * An HIThemeTabSize indicating what size of tab to draw.
   }
		size: HIThemeTabSize;

  {
   * An HIThemeTabAdornment describing any additional adornments that
   * are to be drawn on the tab.
   }
		adornment: HIThemeTabAdornment;             { various tab attributes }

  {
   * An HIThemeTabKind indicating what kind of tab to draw.
   }
		kind: HIThemeTabKind;

  {
   * The HIThemeTabPositions of the tab to be drawn or measured.
   }
		position: HIThemeTabPosition;
	end;
	HIThemeTabDrawInfoPtr = ^HIThemeTabDrawInfo;

{
 *  HIThemeTabDrawInfoVersionZero
 *  
 *  Summary:
 *    This structure is left here as a reference to the previous
 *    version of the tab drawing parameters. Please use the current
 *    version.
 *  
 *  Discussion:
 *    Shipped with Mac OS X 10.3.
 }
type
	HIThemeTabDrawInfoVersionZero = record
		version: UInt32;
		style: ThemeTabStyle;
		direction: ThemeTabDirection;
		size: HIThemeTabSize;
		adornment: HIThemeTabAdornment;             { various tab attributes }
	end;

{
 *  Summary:
 *    Values for HIThemeTabPaneAdornment.
 }
const
	kHIThemeTabPaneAdornmentNormal = 0;


type
	HIThemeTabPaneAdornment = UInt32;

{
 *  HIThemeTabPaneDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to tab pane drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    In Mac OS X 10.4, added kind and adornment fields.
 }
type
	HIThemeTabPaneDrawInfo = record
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * The version of this data structure.  Currently, it is always 1.
=======
   * The version of this data structure.  Currently, it is always 0.
>>>>>>> graemeg/fixes_2_2
=======
   * The version of this data structure.  Currently, it is always 0.
>>>>>>> origin/fixes_2_2
=======
   * The version of this data structure.  Currently, it is always 1.
>>>>>>> origin/fixes_2.4
   }
		version: UInt32;

  {
   * The ThemeDrawState for the tab pane to be drawn.
   }
		state: ThemeDrawState;

  {
   * A ThemeTabDirection describing on which side of the pane the tabs
   * will be drawn.
   }
		direction: ThemeTabDirection;

  {
   * An HIThemeTabSize indicating what size of tab pane to draw.
   }
		size: HIThemeTabSize;

  {
   * An HIThemeTabKind indicating what kind of tab to draw this pane
   * for.
   }
		kind: HIThemeTabKind;

  {
   * An HIThemeTabPaneAdornment describing any additional adornments
   * that are to be drawn on the tab pane.
   }
		adornment: HIThemeTabPaneAdornment;
	end;

{
 *  HIThemeTabPaneDrawInfoVersionZero
 *  
 *  Summary:
 *    This structure is left here as a reference to the previous
 *    version of the tab pane drawing parameters. Please use the
 *    current version.
 *  
 *  Discussion:
 *    Shipped with Mac OS X 10.3.
 }
type
	HIThemeTabPaneDrawInfoVersionZero = record
		version: UInt32;
		state: ThemeDrawState;
		direction: ThemeTabDirection;
		size: HIThemeTabSize;
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const
{
   * Available in Mac OS X 10.3 and later. Valid fields for this
   * version are version and menuType.
   }
	kHIThemeMenuDrawInfoVersionZero = 0;

  {
   * Available in Mac OS X 10.5 and later. Valid fields for this
   * version are all those in the zero version as well as the
   * menuDirection field.
   }
	kHIThemeMenuDrawInfoVersionOne = 1001;


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  HIThemeMenuDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to menu drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    New in Mac OS X 10.3; revised in Mac OS X 10.5.
=======
 *    New in Mac OS X 10.3.
>>>>>>> graemeg/fixes_2_2
=======
 *    New in Mac OS X 10.3.
>>>>>>> origin/fixes_2_2
=======
 *    New in Mac OS X 10.3; revised in Mac OS X 10.5.
>>>>>>> origin/fixes_2.4
 }
type
	HIThemeMenuDrawInfo = record
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * The version of this data structure. Use
   * kHIThemeMenuDrawInfoVersionZero or kHIThemeMenuDrawInfoVersionOne.
=======
   * The version of this data structure.  Currently, it is always 0.
>>>>>>> graemeg/fixes_2_2
=======
   * The version of this data structure.  Currently, it is always 0.
>>>>>>> origin/fixes_2_2
=======
   * The version of this data structure. Use
   * kHIThemeMenuDrawInfoVersionZero or kHIThemeMenuDrawInfoVersionOne.
>>>>>>> origin/fixes_2.4
   }
		version: UInt32;

  {
   * A ThemeMenuType indicating which type of menu is to be drawn.
   }
		menuType: ThemeMenuType;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

  {
   * Must be zero.
   }
		reserved1: UNSIGNEDLONG;

  {
   * Must be zero.
   }
		reserved2: CGFloat;

  {
   * kHIMenuRightDirection or kHIMenuLeftDirection as declared in
   * <CarbonEvents.h>. Only interpreted if the version is
   * kHIThemeMenuDrawInfoVersionOne and the menu type is hierarchical.
   }
		menuDirection: UInt32;

  {
   * Must be zero.
   }
		reserved3: CGFloat;

  {
   * Must be zero.
   }
		reserved4: CGFloat;
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
	HIThemeMenuDrawInfoPtr = ^HIThemeMenuDrawInfo;

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  HIThemeMenuDrawInfoVersionZero
 *  
 *  Summary:
 *    Drawing parameters passed to menu drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeMenuDrawInfoVersionZero = record
{
   * The version of this data structure. Always 0.
   }
		version: UInt32;

  {
   * A ThemeMenuType indicating which type of menu is to be drawn.
   }
		menuType: ThemeMenuType;
	end;
	HIThemeMenuDrawInfoVersionZeroPtr = ^HIThemeMenuDrawInfoVersionZero;

{
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  HIThemeMenuItemDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to menu item drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeMenuItemDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * A ThemeMenuItemType indicating what type of menu item is to be
   * drawn.
   }
		itemType: ThemeMenuItemType;

  {
   * The ThemeMenuState of the menu item to be drawn.
   }
		state: ThemeMenuState;
	end;
	HIThemeMenuItemDrawInfoPtr = ^HIThemeMenuItemDrawInfo;

{
 *  HIThemeFrameDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to frame drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeFrameDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The HIThemeFrameKind of the frame to be drawn.
   }
		kind: HIThemeFrameKind;

  {
   * The ThemeDrawState of the frame to be drawn.
   }
		state: ThemeDrawState;

  {
   * A Boolean indicating whether the frame is to be drawn with focus
   * or without.
   }
		isFocused: Boolean;
	end;
	HIThemeFrameDrawInfoPtr = ^HIThemeFrameDrawInfo;

{
 *  HIThemeGroupBoxDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to group box drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeGroupBoxDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the group box to be drawn.
   }
		state: ThemeDrawState;

  {
   * An HIThemeGroupBoxKind indicating which type of group box is to be
   * drawn.
   }
		kind: HIThemeGroupBoxKind;
	end;
	HIThemeGroupBoxDrawInfoPtr = ^HIThemeGroupBoxDrawInfo;

{
 *  HIThemeGrabberDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to grabber drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeGrabberDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the grabber to be drawn.
   }
		state: ThemeDrawState;
	end;
	HIThemeGrabberDrawInfoPtr = ^HIThemeGrabberDrawInfo;

{
 *  HIThemePlacardDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to placard drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemePlacardDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the placard to be drawn.
   }
		state: ThemeDrawState;
	end;
	HIThemePlacardDrawInfoPtr = ^HIThemePlacardDrawInfo;

{
 *  HIThemeHeaderDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to header drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeHeaderDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the header to be drawn.
   }
		state: ThemeDrawState;

  {
   * The HIThemeHeaderKind for the header to be drawn.
   }
		kind: HIThemeHeaderKind;
	end;
	HIThemeHeaderDrawInfoPtr = ^HIThemeHeaderDrawInfo;

{
 *  HIThemeMenuBarDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to HIThemeDrawMenuBarBackground.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeMenuBarDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeMenuBarState for the menu bar to be drawn.
   }
		state: ThemeMenuBarState;

  {
   * The attributes of the menu bar to be drawn.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		attributes: OptionBits;
=======
		attributes: UInt32;
>>>>>>> graemeg/fixes_2_2
=======
		attributes: UInt32;
>>>>>>> origin/fixes_2_2
=======
		attributes: OptionBits;
>>>>>>> origin/fixes_2.4
	end;
	HIThemeMenuBarDrawInfoPtr = ^HIThemeMenuBarDrawInfo;

{
 *  HIThemeMenuTitleDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to HIThemeDrawMenuTitle.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeMenuTitleDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeMenuState for the menu title to be drawn.
   }
		state: ThemeMenuState;

  {
   * The attributes of the menu title to be drawn. Must be either 0 or
   * kHIThemeMenuTitleDrawCondensed.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		attributes: OptionBits;
=======
		attributes: UInt32;
>>>>>>> graemeg/fixes_2_2
=======
		attributes: UInt32;
>>>>>>> origin/fixes_2_2
=======
		attributes: OptionBits;
>>>>>>> origin/fixes_2.4

  {
   * The border space between the menu title rect and the menu title
   * text when the menu title spacing is being condensed. This field is
   * only observed by the Appearance Manager when the attributes field
   * contains kHIThemeMenuTitleDrawCondensed. The valid values for this
   * field range from the value returned by GetThemeMenuTitleExtra(
   * &extra, false ) to the value returned by GetThemeMenuTitleExtra(
   * &extra, true ). You may pass 0 in this field to use the minimum
   * condensed title extra.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		condensedTitleExtra: CGFloat;
=======
		condensedTitleExtra: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		condensedTitleExtra: Float32;
>>>>>>> origin/fixes_2_2
=======
		condensedTitleExtra: CGFloat;
>>>>>>> origin/fixes_2.4
	end;
	HIThemeMenuTitleDrawInfoPtr = ^HIThemeMenuTitleDrawInfo;

{
 *  HIThemeTickMarkDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to tick mark drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeTickMarkDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the tick mark to be drawn.
   }
		state: ThemeDrawState;
	end;
	HIThemeTickMarkDrawInfoPtr = ^HIThemeTickMarkDrawInfo;

{
 *  HIThemeWindowDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to window drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3, but based on legacy ThemeWindowMetrics.
 }
type
	HIThemeWindowDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * A ThemeDrawState which describes the state of the window to be
   * drawn.
   }
		state: ThemeDrawState;

  {
   * A ThemeWindowType specifying the type of window to be drawn.
   }
		windowType: ThemeWindowType;

  {
   * The ThemeWindowAttributes describing the window to be drawn.
   }
		attributes: ThemeWindowAttributes;

  {
   * The height of the title of the window.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		titleHeight: CGFloat;
=======
		titleHeight: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		titleHeight: Float32;
>>>>>>> origin/fixes_2_2
=======
		titleHeight: CGFloat;
>>>>>>> origin/fixes_2.4

  {
   * The width of the title of the window.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		titleWidth: CGFloat;
=======
		titleWidth: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		titleWidth: Float32;
>>>>>>> origin/fixes_2_2
=======
		titleWidth: CGFloat;
>>>>>>> origin/fixes_2.4
	end;
	HIThemeWindowDrawInfoPtr = ^HIThemeWindowDrawInfo;

{
 *  HIThemeWindowWidgetDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to window widget drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3, but based on legacy ThemeWindowMetrics.
 }
type
	HIThemeWindowWidgetDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * A ThemeDrawState which describes the state of the widget to be
   * drawn.
   }
		widgetState: ThemeDrawState;

  {
   * A ThemeTitleBarWidget specifying the type of window widget to be
   * drawn.
   }
		widgetType: ThemeTitleBarWidget;

  {
   * A ThemeDrawState which describes the state of the window for which
   * the widget is to be drawn.
   }
		windowState: ThemeDrawState;

  {
   * A ThemeWindowType specifying the type of window to be drawn.
   }
		windowType: ThemeWindowType;

  {
   * The ThemeWindowAttributes describing the window to be drawn.
   }
		attributes: ThemeWindowAttributes;

  {
   * The height of the title of the window.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		titleHeight: CGFloat;
=======
		titleHeight: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		titleHeight: Float32;
>>>>>>> origin/fixes_2_2
=======
		titleHeight: CGFloat;
>>>>>>> origin/fixes_2.4

  {
   * The width of the title of the window.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		titleWidth: CGFloat;
=======
		titleWidth: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		titleWidth: Float32;
>>>>>>> origin/fixes_2_2
=======
		titleWidth: CGFloat;
>>>>>>> origin/fixes_2.4
	end;
	HIThemeWindowWidgetDrawInfoPtr = ^HIThemeWindowWidgetDrawInfo;

{
 *  HIThemeSeparatorDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to separator drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeSeparatorDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the separator to be drawn.
   }
		state: ThemeDrawState;
	end;
	HIThemeSeparatorDrawInfoPtr = ^HIThemeSeparatorDrawInfo;

{
 *  HIThemeScrollBarDelimitersDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to separator drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeScrollBarDelimitersDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the separator to be drawn.
   }
		state: ThemeDrawState;

  {
   * A ThemeWindowType specifying the type of window for which to draw
   * the delimiters.
   }
		windowType: ThemeWindowType;

  {
   * The ThemeWindowAttributes of the window for which the scroll bar
   * delimters are to be drawn.
   }
		attributes: ThemeWindowAttributes;
	end;
	HIThemeScrollBarDelimitersDrawInfoPtr = ^HIThemeScrollBarDelimitersDrawInfo;

{
 *  HIThemeChasingArrowsDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to chasing arrows drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeChasingArrowsDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the chasing arrows to be drawn.
   }
		state: ThemeDrawState;

  {
   * A UInt32 used to calculate which frame of the chasing arrow
   * animation is to be drawn.
   }
		index: UInt32;
	end;
	HIThemeChasingArrowsDrawInfoPtr = ^HIThemeChasingArrowsDrawInfo;

{
 *  HIThemePopupArrowDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to popup arrow drawing and measuring
 *    theme APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemePopupArrowDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the popup arrow to be drawn.
   }
		state: ThemeDrawState;

  {
   * A ThemeArrowOrientation for the orientation of the popup arrow to
   * be drawn.
   }
		orientation: ThemeArrowOrientation;

  {
   * A ThemePopupArrowSize for the size of the popup arrow to be drawn.
   }
		size: ThemePopupArrowSize;
	end;
	HIThemePopupArrowDrawInfoPtr = ^HIThemePopupArrowDrawInfo;

{
 *  HIThemeGrowBoxDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to grow box drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3.
 }
type
	HIThemeGrowBoxDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the grow box to be drawn.
   }
		state: ThemeDrawState;

  {
   * A ThemeGrowBoxKind indicating in which kind of grow box to draw.
   }
		kind: HIThemeGrowBoxKind;

  {
   * A ThemeGrowDirection indicating in which direction the window will
   * grow.
   }
		direction: ThemeGrowDirection;

  {
   * An HIThemeGrowBoxSize describing the size of the grow box to draw.
   }
		size: HIThemeGrowBoxSize;
	end;
	HIThemeGrowBoxDrawInfoPtr = ^HIThemeGrowBoxDrawInfo;

{
 *  HIThemeBackgroundDrawInfo
 *  
 *  Discussion:
 *    New in Mac OS X 10.3, but based on legacy TrackDrawInfo.
 }
type
	HIThemeBackgroundDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * The ThemeDrawState of the background to be drawn.
=======
   * The ThemeDrawState of the background to be drawn. Currently,
   * HIThemeDrawBackground backgrounds do not have state, so this field
   * has no meaning. Set it to kThemeStateActive.
>>>>>>> graemeg/fixes_2_2
=======
   * The ThemeDrawState of the background to be drawn. Currently,
   * HIThemeDrawBackground backgrounds do not have state, so this field
   * has no meaning. Set it to kThemeStateActive.
>>>>>>> origin/fixes_2_2
=======
   * The ThemeDrawState of the background to be drawn.
>>>>>>> origin/fixes_2.4
   }
		state: ThemeDrawState;

  {
   * The ThemeBackgroundKind with which to fill the background.
   }
		kind: ThemeBackgroundKind;
	end;
	HIThemeBackgroundDrawInfoPtr = ^HIThemeBackgroundDrawInfo;
{ -------------------------------------------------------------------------- }
{  Buttons                                                                   }
{ -------------------------------------------------------------------------- }
{
 *  HIThemeDrawButton()
 *  
 *  Summary:
 *    Draw a themed button.
 *  
 *  Discussion:
 *    This generic button drawing theme primitive draws not just a push
 *    button, but all of the kinds of buttons described by
 *    ThemeButtonKind.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      The HIRect in which to draw.  Note that this API may draw
 *      outside of its bounds.
 *    
 *    inDrawInfo:
 *      An HIThemeButtonDrawInfo describing the button that will be
 *      drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *    
 *    outLabelRect:
 *      A pointer to an HIRect into which to put the bounds of the
 *      label rect.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawButton( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeButtonDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation; outLabelRect: HIRectPtr { can be NULL } ): OSStatus; external name '_HIThemeDrawButton';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetButtonShape()
 *  
 *  Summary:
 *    Get a shape of a themed button.
 *  
 *  Discussion:
 *    This API was mistakenly named as a "Get" API. It behaves as
 *    "Copy" API. THE CALLER IS RESPONSIBLE FOR RELEASING THE RETURNED
 *    SHAPE.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIRect indicating where the button would be drawn.
 *    
 *    inDrawInfo:
 *      An HIThemeButtonDrawInfo describing the button that would be
=======
=======
>>>>>>> origin/fixes_2_2
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      An HIThemeButtonDrawInfo describing the button that will be
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *      drawn.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
<<<<<<< HEAD
<<<<<<< HEAD
 *      the button that would be drawn. It needs to be released by the
 *      caller.
=======
 *      the button. Needs to be released by caller.
>>>>>>> graemeg/fixes_2_2
=======
 *      the button. Needs to be released by caller.
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetButtonShape( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeButtonDrawInfo; var outShape: HIShapeRef ): OSStatus; external name '_HIThemeGetButtonShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetButtonContentBounds()
 *  
 *  Summary:
 *    Get the bounds of a themed button's content.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIRect indicating where the button would be drawn.
 *    
 *    inDrawInfo:
 *      An HIThemeButtonDrawInfo describing the button that would be
=======
=======
>>>>>>> origin/fixes_2_2
 *      An HIRect indicating where the button is to be drawn.
 *    
 *    inDrawInfo:
 *      An HIThemeButtonDrawInfo describing the button that will be
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *      drawn.
 *    
 *    outBounds:
 *      A pointer to an HIRect in which will be returned the rectangle
 *      of the button content bounds.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetButtonContentBounds( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeButtonDrawInfo; var outBounds: HIRect ): OSStatus; external name '_HIThemeGetButtonContentBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetButtonBackgroundBounds()
 *  
 *  Summary:
 *    Get the bounds of the background of a themed button.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIRect indicating where the button would be drawn.
 *    
 *    inDrawInfo:
 *      An HIThemeButtonDrawInfo describing the button that would be
=======
=======
>>>>>>> origin/fixes_2_2
 *      An HIRect indicating where the button is to be drawn.
 *    
 *    inDrawInfo:
 *      An HIThemeButtonDrawInfo describing the button that will be
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *      drawn.
 *    
 *    outBounds:
 *      A pointer to an HIRect in which will be returned the rectangle
 *      of the button background bounds.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetButtonBackgroundBounds( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeButtonDrawInfo; var outBounds: HIRect ): OSStatus; external name '_HIThemeGetButtonBackgroundBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawChasingArrows()
 *  
 *  Summary:
 *    Draw themed chasing arrows.
 *  
 *  Discussion:
 *    Draw a frame from the chasing arrows animation.  The animation
 *    frame is based on a modulo value calculated from the index.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      An HIRect indicating where the chasing arrows are to be drawn.
 *    
 *    inDrawInfo:
 *      An HIThemeChasingArrowsDrawInfo describing the chasing arrows
 *      to be drawn or measured.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawChasingArrows( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeChasingArrowsDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawChasingArrows';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawPopupArrow()
 *  
 *  Summary:
 *    Draws a themed popup arrow.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      An HIThemePopupArrowDrawInfo describing the popup arrow to be
 *      drawn or measured.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawPopupArrow( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemePopupArrowDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawPopupArrow';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ -------------------------------------------------------------------------- }
{  Menus                                                                     }
{ -------------------------------------------------------------------------- }
{
 *  HIThemeDrawMenuBarBackground()
 *  
 *  Summary:
 *    Draws the menu bar background for a given area.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      The HIThemeMenuBarDrawInfo of the menu bar to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawMenuBarBackground( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeMenuBarDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawMenuBarBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawMenuTitle()
 *  
 *  Summary:
 *    Draws the menu title background for a menu.
 *  
 *  Discussion:
 *    This API draws the background of a menu title. It does not draw
 *    the menu title text; it is the caller's responsibility to draw
 *    the text after this API has returned. The text should be drawn
 *    into the bounds returned in the outLabelRect parameter; the
 *    caller should ensure that the text is not drawn outside of those
 *    bounds, either by truncating or clipping the text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenuBarRect:
 *      An HIRect indicating the bounds of the whole menu bar for which
 *      the menu title is to be drawn.
 *    
 *    inTitleRect:
 *      An HIRect for the bounds of the menu title to be drawn.
 *    
 *    inDrawInfo:
 *      The HIThemeMenuTitleDrawInfo of the menu title to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *    
 *    outLabelRect:
 *      On exit, contains the bounds in which the menu title text
 *      should be drawn. May be NULL if you don't need this information.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawMenuTitle( const (*var*) inMenuBarRect: HIRect; const (*var*) inTitleRect: HIRect; const (*var*) inDrawInfo: HIThemeMenuTitleDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation; outLabelRect: HIRectPtr { can be NULL } ): OSStatus; external name '_HIThemeDrawMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawMenuBackground()
 *  
 *  Summary:
 *    Draws the theme menu background in a rectangle.  This API may
 *    draw outside of the specified rectangle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenuRect:
 *      An HIRect indicating the bounds of the whole menu for which the
 *      background is to be drawn.
 *    
 *    inMenuDrawInfo:
 *      An HIThemeMenuDrawInfo describing the menu to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawMenuBackground( const (*var*) inMenuRect: HIRect; const (*var*) inMenuDrawInfo: HIThemeMenuDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawMenuBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawMenuItem()
 *  
 *  Summary:
 *    Draws a themed menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenuRect:
 *      An HIRect indicating the bounds of the whole menu for which the
 *      menu item is to be drawn.
 *    
 *    inItemRect:
 *      An HIRect for the bounds of the menu item to be drawn.
 *    
 *    inItemDrawInfo:
 *      An HIThemeMenuItemDrawInfo describing the drawing
 *      characteristics of the menu item to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *    
 *    outContentRect:
 *      An HIRect that will be filled with the rectangle describing
 *      where the menu item content is to be drawn.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawMenuItem( const (*var*) inMenuRect: HIRect; const (*var*) inItemRect: HIRect; const (*var*) inItemDrawInfo: HIThemeMenuItemDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation; var outContentRect: HIRect ): OSStatus; external name '_HIThemeDrawMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawMenuSeparator()
 *  
 *  Summary:
 *    Draws a themed menu separator.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenuRect:
 *      An HIRect indicating the bounds of the whole menu for which the
 *      menu separator is to be drawn.
 *    
 *    inItemRect:
 *      An HIRect for the bounds of the menu separator to be drawn.
 *    
 *    inItemDrawInfo:
 *      An HIThemeMenuItemDrawInfo describing the drawing
 *      characteristics of the menu item to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawMenuSeparator( const (*var*) inMenuRect: HIRect; const (*var*) inItemRect: HIRect; const (*var*) inItemDrawInfo: HIThemeMenuItemDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawMenuSeparator';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetMenuBackgroundShape()
 *  
 *  Summary:
 *    Gets the shape of the background for a themed menu.
 *  
 *  Discussion:
 *    This shape can extend outside of the bounds of the specified
 *    rectangle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenuRect:
 *      An HIRect indicating the bounds of the menu for which the menu
<<<<<<< HEAD
<<<<<<< HEAD
 *      background is to be retrieved.
=======
 *      background is to be drawn.
>>>>>>> graemeg/fixes_2_2
=======
 *      background is to be drawn.
>>>>>>> origin/fixes_2_2
 *    
 *    inMenuDrawInfo:
 *      An HIThemeMenuDrawInfo describing the menu to be measured.
 *    
 *    outShape:
 *      A valid HIShape that will be cleared and filled with the shape
<<<<<<< HEAD
<<<<<<< HEAD
 *      of the menu background. It needs to be released by the caller.
=======
 *      of the menu background.
>>>>>>> graemeg/fixes_2_2
=======
 *      of the menu background.
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetMenuBackgroundShape( const (*var*) inMenuRect: HIRect; const (*var*) inMenuDrawInfo: HIThemeMenuDrawInfo; var outShape: HIShapeRef ): OSStatus; external name '_HIThemeGetMenuBackgroundShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ -------------------------------------------------------------------------- }
{  Segments                                                                  }
{ -------------------------------------------------------------------------- }

{
 *  Summary:
 *    Available values for HIThemeSegmentPosition. These are positions
 *    of the segments within the segmented view.
 }
const
{
   * The first position of a segmented view.
   }
	kHIThemeSegmentPositionFirst = 0;

  {
   * A middle segment.
   }
	kHIThemeSegmentPositionMiddle = 1;

  {
   * The last position of a segmented view.
   }
	kHIThemeSegmentPositionLast = 2;

  {
   * The only position of a segmented view. It is simultaneously first
   * and last. You know, only. There is only one segment. It looks
   * pretty much like a button. Please don't use this if you can avoid
   * it. It's ugly.
   }
	kHIThemeSegmentPositionOnly = 3;

type
	HIThemeSegmentPosition = UInt32;

{
 *  Summary:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Available values for HIThemeSegmentKind available on Mac OS X
 *    10.4 a later.
 }
const
{
   * The segment to use on non-textured windows. Do not use on textured
=======
=======
>>>>>>> origin/fixes_2_2
 *    Available values for HIThemeSegmentKind.
 }
const
{
   * The outset looking segment. Do not use on metal or metal-like
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Available values for HIThemeSegmentKind available on Mac OS X
 *    10.4 a later.
 }
const
{
   * The segment to use on non-textured windows. Do not use on textured
>>>>>>> origin/fixes_2.4
   * windows.
   }
	kHIThemeSegmentKindNormal = 0;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
   * The textured segment. Use on textured windows.
   }
	kHIThemeSegmentKindTextured = 1;

  {
   * This is a synonym for kHIThemeSegmentKindTextured for code
   * compatibility. Please use kHIThemeSegmentKindTextured instead.
<<<<<<< HEAD
   }
	kHIThemeSegmentKindInset = kHIThemeSegmentKindTextured;
=======
   * The inset segment. Use on metal or metal-like windows.
   }
	kHIThemeSegmentKindInset = 1;
>>>>>>> graemeg/fixes_2_2
=======
   * The inset segment. Use on metal or metal-like windows.
   }
	kHIThemeSegmentKindInset = 1;
>>>>>>> origin/fixes_2_2
=======
   }
	kHIThemeSegmentKindInset = kHIThemeSegmentKindTextured;
>>>>>>> origin/fixes_2.4

type
	HIThemeSegmentKind = UInt32;

{
 *  Summary:
 *    Available values for HIThemeSegmentSize.
 }
const
{
   * The normally sized segment.
   }
	kHIThemeSegmentSizeNormal = 0;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * The small segment. Not available as textured.
=======
   * The small segment. Not available with as inset.
>>>>>>> graemeg/fixes_2_2
=======
   * The small segment. Not available with as inset.
>>>>>>> origin/fixes_2_2
=======
   * The small segment. Not available as textured.
>>>>>>> origin/fixes_2.4
   }
	kHIThemeSegmentSizeSmall = 1;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * The mini segment. Not available as textured.
=======
   * The mini segment. Not available with as inset.
>>>>>>> graemeg/fixes_2_2
=======
   * The mini segment. Not available with as inset.
>>>>>>> origin/fixes_2_2
=======
   * The mini segment. Not available as textured.
>>>>>>> origin/fixes_2.4
   }
	kHIThemeSegmentSizeMini = 3;

type
	HIThemeSegmentSize = UInt32;

const
{
   * No segment adornments are to be drawn.
   }
	kHIThemeSegmentAdornmentNone = 0;

  {
   * A focus ring is to be drawn around the segment.
   }
	kHIThemeSegmentAdornmentFocus = 1 shl 2; { to match button focus adornment }

  {
   * If available, a leading separator is to be drawn on the segment.
   * Note that segments overlap and if the separators are drawn over
   * top each other, the shadows multiply undesirably.
   }
	kHIThemeSegmentAdornmentLeadingSeparator = 1 shl 3;

  {
   * If available, a trailing separator is to be drawn on the segment.
   * Note that segments overlap and if the separators are drawn over
   * top each other, the shadows multiply undesirably.
   }
	kHIThemeSegmentAdornmentTrailingSeparator = 1 shl 4;

type
	HIThemeSegmentAdornment = UInt32;

{
 *  HIThemeSegmentDrawInfo
 *  
 *  Summary:
 *    Drawing parameters passed to segment drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.4.
 }
type
	HIThemeSegmentDrawInfo = record
{
   * The version of this data structure.  Currently, it is always 0.
   }
		version: UInt32;

  {
   * The ThemeDrawState for the segment to be drawn or measured.
   }
		state: ThemeDrawState;

  {
   * The ThemeButtonValue of the segment to be drawn or measured.
   }
		value: ThemeButtonValue;
		size: HIThemeSegmentSize;

  {
   * The HIThemeSegmentKind of the segment to be drawn or measured.
   }
		kind: HIThemeSegmentKind;

  {
   * The HIThemeSegmentPositions of the segment to be drawn or measured.
   }
		position: HIThemeSegmentPosition;

  {
   * The HIThemeSegmentAdornment of the segment to be drawn or measured.
   }
		adornment: HIThemeSegmentAdornment;
	end;
	HIThemeSegmentDrawInfoPtr = ^HIThemeSegmentDrawInfo;
{
 *  HIThemeDrawSegment()
 *  
 *  Summary:
 *    Draw a piece of a segmented view.
 *  
 *  Discussion:
 *    New in Mac OS X 10.4. Please note that segments can draw a
 *    separator outside of the specified bounds and the adornments of
 *    the individual segments must coordinate their drawing of
 *    separators (with the adornment field of the passed in
 *    HIThemeSegmentDrawInfo) to avoid overdrawing.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      The bounds of the segment to be drawn.
 *    
 *    inDrawInfo:
 *      A HIThemeSegmentDrawInfo describing the segment to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawSegment( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeSegmentDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawSegment';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ -------------------------------------------------------------------------- }
{  Tabs                                                                      }
{ -------------------------------------------------------------------------- }
{
 *  HIThemeDrawTabPane()
 *  
 *  Summary:
 *    Draws a themed tab pane.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw the pane.
 *    
 *    inDrawInfo:
 *      The HIThemeTabPaneDrawInfo of the tab pane to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawTabPane( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeTabPaneDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawTabPane';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawTab()
 *  
 *  Summary:
 *    Draw a themed tab.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      An HIThemeTabDrawInfo describing the tab to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *    
 *    outLabelRect:
 *      An HIRect into which to put the bounds of the label rect.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawTab( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeTabDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation; outLabelRect: HIRectPtr { can be NULL } ): OSStatus; external name '_HIThemeDrawTab';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTabPaneDrawShape()
 *  
 *  Summary:
 *    Gets the shape of the draw area relative to the full bounds of
 *    the tab+pane.
 *  
 *  Discussion:
 *    This API was mistakenly named as a "Get" API. It behaves as
 *    "Copy" API. THE CALLER IS RESPONSIBLE FOR RELEASING THE RETURNED
 *    SHAPE.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIRect indicating the entire tabs area for which the tab
 *      pane shape is to be retrieved.
 *    
 *    inDirection:
 *      A ThemeTabDirection describing on which side of the pane the
 *      tabs would be drawn.
 *    
 *    inTabSize:
 *      An HIThemeTabSize indicating the size of tab pane that would be
 *      drawn.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
 *      the draw area. It needs to be released by the caller.
=======
=======
>>>>>>> origin/fixes_2_2
 *      The HIRect for which to get the shape.
 *    
 *    inDirection:
 *      A ThemeTabDirection describing on which side of the pane the
 *      tabs will be drawn.
 *    
 *    inTabSize:
 *      An HIThemeTabSize indicating what size of tab pane to draw.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
 *      the draw area. Needs to be released by caller.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTabPaneDrawShape( const (*var*) inRect: HIRect; inDirection: ThemeTabDirection; inTabSize: HIThemeTabSize; var outShape: HIShapeRef ): OSStatus; external name '_HIThemeGetTabPaneDrawShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTabPaneContentShape()
 *  
 *  Summary:
 *    Gets the shape of the content area relative to the full bounds of
 *    the tab+pane.
 *  
 *  Discussion:
 *    This API was mistakenly named as a "Get" API. It behaves as
 *    "Copy" API. THE CALLER IS RESPONSIBLE FOR RELEASING THE RETURNED
 *    SHAPE.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIRect indicating the entire tabs area for which the tab
 *      content shape is to be retrieved.
 *    
 *    inDirection:
 *      A ThemeTabDirection describing on which side of the pane the
 *      tabs would be drawn.
 *    
 *    inTabSize:
 *      An HIThemeTabSize indicating what size of tab pane that would
 *      be drawn.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
 *      the draw content. It needs to be released by the caller.
=======
=======
>>>>>>> origin/fixes_2_2
 *      The HIRect for which to get the shape.
 *    
 *    inDirection:
 *      A ThemeTabDirection describing on which side of the pane the
 *      tabs will be drawn.
 *    
 *    inTabSize:
 *      An HIThemeTabSize indicating what size of tab pane to draw.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
 *      the draw content. Needs to be released by caller.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTabPaneContentShape( const (*var*) inRect: HIRect; inDirection: ThemeTabDirection; inTabSize: HIThemeTabSize; var outShape: HIShapeRef ): OSStatus; external name '_HIThemeGetTabPaneContentShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTabDrawShape()
 *  
 *  Summary:
 *    Gets the shape of the tab drawing area relative to the full
 *    bounds of the tab+pane.
 *  
 *  Discussion:
 *    This API was mistakenly named as a "Get" API. It behaves as
 *    "Copy" API. THE CALLER IS RESPONSIBLE FOR RELEASING THE RETURNED
 *    SHAPE.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIRect indicating the entire tab+pane area for which the tab
 *      shape is to be retrieved.
 *    
 *    inDrawInfo:
 *      An HIThemeTabDrawInfo describing the tab that would be drawn.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
 *      the tab drawing area. It needs to be released by the caller.
=======
=======
>>>>>>> origin/fixes_2_2
 *      The HIRect for which to get the shape.
 *    
 *    inDrawInfo:
 *      An HIThemeTabDrawInfo describing the tab that will be drawn.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
 *      the tab drawing area. Needs to be released by caller.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTabDrawShape( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeTabDrawInfo; var outShape: HIShapeRef ): OSStatus; external name '_HIThemeGetTabDrawShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTabShape()
 *  
 *  Summary:
 *    Gets the shape for a themed tab.
 *  
 *  Discussion:
 *    This API was mistakenly named as a "Get" API. It behaves as
 *    "Copy" API. THE CALLER IS RESPONSIBLE FOR RELEASING THE RETURNED
 *    SHAPE.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      An HIRect indicating the entire tabs area for which the tab
 *      shape is to be retrieved.
 *    
 *    inDrawInfo:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIThemeTabDrawInfo describing the tab that would be drawn.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
 *      the tab. It needs to be released by the caller.
=======
=======
>>>>>>> origin/fixes_2_2
 *      An HIThemeTabDrawInfo describing the tab that will be drawn.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
 *      the tab. Needs to be released by caller.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTabShape( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeTabDrawInfo; var outShape: HIShapeRef ): OSStatus; external name '_HIThemeGetTabShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ -------------------------------------------------------------------------- }
{  Text                                                                      }
{ -------------------------------------------------------------------------- }

const
{
   * Don't truncate the measured or drawn text.
   }
	kHIThemeTextTruncationNone = 0;

  {
   * During measure or draw, if the text will not fit within the
   * available bounds, truncate the text in the middle of the last
   * visible line.
   }
	kHIThemeTextTruncationMiddle = 1;

  {
   * During measure or draw, if the text will not fit within the
   * available bounds, truncate the text at the end of the last visible
   * line.
   }
	kHIThemeTextTruncationEnd = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kHIThemeTextTruncationDefault = 3;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kHIThemeTextTruncationDefault = 3;
>>>>>>> origin/fixes_2.4

type
	HIThemeTextTruncation = UInt32;

const
{
   * The text will be drawn flush with the left side of the bounding
   * box.
   }
	kHIThemeTextHorizontalFlushLeft = 0;

  {
   * The text will be centered within the bounding box.
   }
	kHIThemeTextHorizontalFlushCenter = 1;

  {
   * The text will be drawn flush with the right side of the bounding
   * box.
   }
	kHIThemeTextHorizontalFlushRight = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kHIThemeTextHorizontalFlushDefault = 3;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kHIThemeTextHorizontalFlushDefault = 3;
>>>>>>> origin/fixes_2.4

type
	HIThemeTextHorizontalFlush = UInt32;

const
{
   * Draw the text vertically flush with the top of the box
   }
	kHIThemeTextVerticalFlushTop = 0;

  {
   * Vertically center the text
   }
	kHIThemeTextVerticalFlushCenter = 1;

  {
   * Draw the text vertically flush with the bottom of the box
   }
	kHIThemeTextVerticalFlushBottom = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kHIThemeTextVerticalFlushDefault = 3;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kHIThemeTextVerticalFlushDefault = 3;
>>>>>>> origin/fixes_2.4

type
	HIThemeTextVerticalFlush = UInt32;

const
	kHIThemeTextBoxOptionNone = 0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

  {
   * Is the text strongly vertical? This option bit is not correctly
   * respected and will have no effect if used.
   }
<<<<<<< HEAD
	kHIThemeTextBoxOptionStronglyVertical = 1 shl 1;

  {
   * Draw the text with an engraved look, suitable for use on the Mac
   * OS X 10.5 dark window backgrounds or on the bodies of some
   * controls.
   }
	kHIThemeTextBoxOptionEngraved = 1 shl 2;

  {
   * By default, HIThemeDrawTextBox will clip the text to the rectangle
   * specified by the inBounds parameter. With some fonts or styles,
   * text may draw outside of the drawing rectangle and will be
   * clipped. If this bit is set, HIThemeDrawTextBox will not clip
   * drawing to its inBounds parameter. Available in Mac OS X 10.4 and
   * later.
   }
	kHIThemeTextBoxOptionDontClip = 1 shl 18;

type
	HIThemeTextBoxOptions = OptionBits;

const
{
   * Available in Mac OS X 10.3 and later. Valid fields for this
   * version are version, state, fontID, horizontalFlushness,
   * verticalFlushness, options, truncationPosition, truncationMaxLines
   * and truncationHappened.
   }
	kHIThemeTextInfoVersionZero = 0;

  {
   * Available in Mac OS X 10.5 and later. Valid fields for this
   * version are all those in the zero version as well as the font
   * field.
   }
	kHIThemeTextInfoVersionOne = 1;

=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kHIThemeTextBoxOptionStronglyVertical = 1 shl 1;

  {
   * Draw the text with an engraved look, suitable for use on the Mac
   * OS X 10.5 dark window backgrounds or on the bodies of some
   * controls.
   }
	kHIThemeTextBoxOptionEngraved = 1 shl 2;

  {
   * By default, HIThemeDrawTextBox will clip the text to the rectangle
   * specified by the inBounds parameter. With some fonts or styles,
   * text may draw outside of the drawing rectangle and will be
   * clipped. If this bit is set, HIThemeDrawTextBox will not clip
   * drawing to its inBounds parameter. Available in Mac OS X 10.4 and
   * later.
   }
	kHIThemeTextBoxOptionDontClip = 1 shl 18;

type
	HIThemeTextBoxOptions = OptionBits;

const
{
   * Available in Mac OS X 10.3 and later. Valid fields for this
   * version are version, state, fontID, horizontalFlushness,
   * verticalFlushness, options, truncationPosition, truncationMaxLines
   * and truncationHappened.
   }
	kHIThemeTextInfoVersionZero = 0;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  {
   * Available in Mac OS X 10.5 and later. Valid fields for this
   * version are all those in the zero version as well as the font
   * field.
   }
	kHIThemeTextInfoVersionOne = 1;

>>>>>>> origin/fixes_2.4

{
 *  HIThemeTextInfo
 *  
 *  Summary:
 *    Drawing parameters passed to text drawing and measuring theme
 *    APIs.
 *  
 *  Discussion:
 *    New in Mac OS X 10.3, this structure is used for measuring and
 *    drawing text with the HIThemeGetTextDimensions and
 *    HIThemeDrawTextBox APIs. If truncationPosition is
 *    kHIThemeTextTruncationNone, the other fields with the truncation
 *    prefix are ignored.
 }
type
	HIThemeTextInfo = record
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * The version of this data structure. Currently, it is always 1.
   }
		version: UInt32;
=======
   * The version of this data structure. Currently, it is always 0.
   }
		version: UInt32;                { current version is 0 }
>>>>>>> graemeg/fixes_2_2
=======
   * The version of this data structure. Currently, it is always 0.
   }
		version: UInt32;                { current version is 0 }
>>>>>>> origin/fixes_2_2
=======
   * The version of this data structure. Currently, it is always 1.
   }
		version: UInt32;
>>>>>>> origin/fixes_2.4

  {
   * The theme draw state in which to draw the string.
   }
		state: ThemeDrawState;

  {
   * The font in which to draw the string.
   }
		fontID: ThemeFontID;

  {
   * The horizontal flushness of the text. One of the
   * kHIThemeTextHorizontalFlush[Left/Center/Right] constants. When
   * this structure is used for HIThemeGetTextDimensions, this field
   * has no effect on the returned dimensions. However, providing the
   * same flushness that will be used with a subsequent draw will
   * trigger a performance optimization.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		horizontalFlushness: HIThemeTextHorizontalFlush;
=======
		horizontalFlushness: HIThemeTextHorizontalFlush; { kHIThemeTextHorizontalFlush[Left/Center/Right] }
>>>>>>> graemeg/fixes_2_2
=======
		horizontalFlushness: HIThemeTextHorizontalFlush; { kHIThemeTextHorizontalFlush[Left/Center/Right] }
>>>>>>> origin/fixes_2_2
=======
		horizontalFlushness: HIThemeTextHorizontalFlush;
>>>>>>> origin/fixes_2.4

  {
   * The vertical flushness of the text. One of the
   * kHIThemeTextVerticalFlush[Top/Center/Bottom] constants. When this
   * paramblock is used for HIThemeGetTextDimensions, this field has no
   * effect on the returned dimensions. However, providing the same
   * flushness that will be used with a subsequent draw will trigger a
   * performance optimization.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		verticalFlushness: HIThemeTextVerticalFlush;
=======
		verticalFlushness: HIThemeTextVerticalFlush; { kHIThemeTextVerticalFlush[Top/Center/Bottom] }
>>>>>>> graemeg/fixes_2_2
=======
		verticalFlushness: HIThemeTextVerticalFlush; { kHIThemeTextVerticalFlush[Top/Center/Bottom] }
>>>>>>> origin/fixes_2_2
=======
		verticalFlushness: HIThemeTextVerticalFlush;
>>>>>>> origin/fixes_2.4

  {
   * Currently, the only option available is for strongly vertical text
   * with the kThemeTextBoxOptionStronglyVertical option bit.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		options: HIThemeTextBoxOptions;

  {
   * Specifies where truncation should occur. If this field is
   * kHIThemeTextTruncationNone, no truncation will occur, and all
   * fields with the truncation prefix will be ignored.
   }
		truncationPosition: HIThemeTextTruncation;

  {
   * The maximum number of lines to measure or draw before truncation
   * occurs. Ignored if truncationPosition is
   * kHIThemeTextTruncationNone.
   }
		truncationMaxLines: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2
		options: HIThemeTextBoxOptions;             { includes kHIThemeTextBoxOptionStronglyVertical }
=======
		options: HIThemeTextBoxOptions;
>>>>>>> origin/fixes_2.4

  {
   * Specifies where truncation should occur. If this field is
   * kHIThemeTextTruncationNone, no truncation will occur, and all
   * fields with the truncation prefix will be ignored.
   }
		truncationPosition: HIThemeTextTruncation;

  {
   * The maximum number of lines to measure or draw before truncation
   * occurs. Ignored if truncationPosition is
   * kHIThemeTextTruncationNone.
   }
<<<<<<< HEAD
		truncationMaxLines: UInt32;     { the maximum number of lines before truncation occurs }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		truncationMaxLines: UInt32;
>>>>>>> origin/fixes_2.4

  {
   * On output, if the text has been truncated, this is set to true. If
   * the text fit completely within the parameters specified and the
   * text was not truncated, this is set to false.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		truncationHappened: Boolean;
		filler1: UInt8;

  {
   * If fontID is kThemeSpecifiedFont and the version is 1, this
   * CTFontRef will be used for measuring and rendering of the string.
   * If the fontID is anything other than kThemeSpecifiedFont or the
   * version is not 1, this field is ignored.
   }
		font: CTFontRef;
<<<<<<< HEAD
=======
		truncationHappened: Boolean;     { on output, whether truncation needed to happen }
>>>>>>> graemeg/fixes_2_2
=======
		truncationHappened: Boolean;     { on output, whether truncation needed to happen }
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
	HIThemeTextInfoPtr = ^HIThemeTextInfo;
{
 *  HIThemeGetTextDimensions()
 *  
 *  Summary:
 *    Get text dimensions of a string
 *  
 *  Discussion:
 *    This allows you to get various dimension characteristics of a
 *    string bound to certain criteria that you specify. It allows you
 *    to get the absolute bounds of a string laid out in a single line,
 *    or the bounds of a string constrained to a given width.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inString:
 *      A CFStringRef containing the unicode characters you wish to
 *      measure. You MUST NOT pass in a CFStringRef that was allocated
 *      with any of the "NoCopy" CFString creation APIs; a string
 *      created with a "NoCopy" API has transient storage which is
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *      incompatible with HIThemeGetTextDimensions's caches. 
 *      
 *      In Mac OS X 10.5 and later, this API may also be passed a
 *      CFAttributedStringRef.
<<<<<<< HEAD
=======
 *      incompatible with HIThemeGetTextDimensions's caches.
>>>>>>> graemeg/fixes_2_2
=======
 *      incompatible with HIThemeGetTextDimensions's caches.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *    
 *    inWidth:
 *      The width to constrain the text before wrapping. If inWidth is
 *      0, the text will not wrap and will be laid out as a single
 *      line, unless it contains embedded carriage return or linefeed
 *      characters; CR/LF will cause the text to wrap and the resulting
 *      measurements will include space for multiple lines of text. If
 *      inWidth is not 0, the text will wrap at the given width and the
 *      measurements will be returned from the multi-line layout.
 *      
 *      
 *      To force single-line layout even in the presence of embedded
 *      CR/LF characters, pass FLT_MAX for inWidth,
 *      kHIThemeTextTruncationEnd for inTextInfo.truncationPosition,
 *      and 1 for inTextInfo.truncationMaxLines.
 *    
 *    inTextInfo:
 *      The HIThemeTextInfo parameter block specifying additional
 *      options for flushness and truncation. The truncationHappened
 *      field is the only field that will be written to by this API
 *      (and the reason for inTextInfo not being const).
 *    
 *    outWidth:
 *      On output, will contain the width of the string. This width may
 *      be smaller than the constrain inWidth parameter if the text has
 *      wrapped. It will return the true bounding width of the layout.
 *      Can be NULL.
 *    
 *    outHeight:
 *      On output, will contain the height of the string. Can be NULL.
 *    
 *    outBaseline:
 *      On output, will contain the baseline of the string. This is the
 *      delta from the top of the text to the baseline of the first
 *      line. Can be NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function HIThemeGetTextDimensions( inString: CFStringRef; inWidth: CGFloat; var inTextInfo: HIThemeTextInfo; outWidth: CGFloatPtr { can be NULL }; outHeight: CGFloatPtr { can be NULL }; outBaseline: CGFloatPtr { can be NULL } ): OSStatus; external name '_HIThemeGetTextDimensions';
=======
function HIThemeGetTextDimensions( inString: CFStringRef; inWidth: Float32; var inTextInfo: HIThemeTextInfo; outWidth: Float32Ptr { can be NULL }; outHeight: Float32Ptr { can be NULL }; outBaseline: Float32Ptr { can be NULL } ): OSStatus; external name '_HIThemeGetTextDimensions';
>>>>>>> graemeg/fixes_2_2
=======
function HIThemeGetTextDimensions( inString: CFStringRef; inWidth: Float32; var inTextInfo: HIThemeTextInfo; outWidth: Float32Ptr { can be NULL }; outHeight: Float32Ptr { can be NULL }; outBaseline: Float32Ptr { can be NULL } ): OSStatus; external name '_HIThemeGetTextDimensions';
>>>>>>> origin/fixes_2_2
=======
function HIThemeGetTextDimensions( inString: CFStringRef; inWidth: CGFloat; var inTextInfo: HIThemeTextInfo; outWidth: CGFloatPtr { can be NULL }; outHeight: CGFloatPtr { can be NULL }; outBaseline: CGFloatPtr { can be NULL } ): OSStatus; external name '_HIThemeGetTextDimensions';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawTextBox()
 *  
 *  Summary:
 *    Draw the string into the given bounding box
 *  
 *  Discussion:
 *    Draw the string into the bounding box given. You can specify
 *    options such as truncation and justification as well as
 *    determining whether the text was truncated when it was drawn.
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *      incompatible with HIThemeDrawTextBox's caches. 
 *      
 *      In Mac OS X 10.5 and later, this API may also be passed a
 *      CFAttributedStringRef.
<<<<<<< HEAD
=======
 *      incompatible with HIThemeDrawTextBox's caches.
>>>>>>> graemeg/fixes_2_2
=======
 *      incompatible with HIThemeDrawTextBox's caches.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *    
 *    inBounds:
 *      The HIRect that bounds where the text is to be drawn
 *    
 *    inTextInfo:
 *      The HIThemeTextInfo parameter block specifying additional
 *      options for truncation and flushness. You can control the
 *      number of lines drawn by specifying a truncation of
 *      kHIThemeTextTruncationMiddle or kHIThemeTextTruncationEnd for
 *      the truncationPosition field and then specifying a maximum
 *      number of lines to draw before truncation occurs in the
 *      truncationMaxLines field. The truncationHappened field is the
 *      only field that will be written to by this API (and the reason
 *      for inTextInfo not being const).
 *    
 *    inContext:
 *      The CGContextRef into which to draw the text.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawTextBox( inString: CFStringRef; const (*var*) inBounds: HIRect; var inTextInfo: HIThemeTextInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawTextBox';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  HIThemeGetUIFontType()
 *  
 *  Summary:
 *    Returns the CTFontUIFontType for a ThemeFontID
 *  
 *  Discussion:
 *    It is possible to create a CTFontRef that represents a
 *    ThemeFontID by using this API in conjunction with
 *    CTFontCreateUIFontForLanguage. 
 *    
 *    Suggested usage: 
 *    CTFontRef font = CTFontCreateUIFontForLanguage(
 *    HIThemeGetUIFontType( inFontID ), 0, NULL );
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inFontID:
 *      The ThemeFontID to map to a CTFontUIFontType.
 *  
 *  Result:
 *    The CTFontUIFontType that represents the ThemeFontID or
 *    kCTFontNoFontType if there is an error.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetUIFontType( inFontID: ThemeFontID ): CTFontUIFontType; external name '_HIThemeGetUIFontType';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ -------------------------------------------------------------------------- }
{  Tracks                                                                    }
{ -------------------------------------------------------------------------- }
{
 *  HIThemeDrawTrack()
 *  
 *  Summary:
 *    Draw a themed track item.
 *  
 *  Discussion:
 *    Used to draw any tracked element including sliders and scroll
 *    bars.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track that will be drawn.
 *    
 *    inGhostRect:
 *      An HIRect describing the location of the ghost indicator to be
 *      drawn. Generally, this should be NULL and the control using
 *      this primitive should support live feeback.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawTrack( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; {const} inGhostRect: HIRectPtr { can be NULL }; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawTrack';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawTrackTickMarks()
 *  
 *  Summary:
 *    Draws the tick marks for a slider track.
 *  
 *  Discussion:
 *    This primitive only draws evenly distributed tick marks. 
 *    Internally, it calls HIThemeDrawTickMark to do the actual tick
 *    mark drawing, and any custom (non-even distribution) drawing of
 *    tick marks should be done with HIThemeDrawTickMark.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track tick marks that
 *      will be drawn.
 *    
 *    inNumTicks:
 *      A value indicating the number of tick marks to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawTrackTickMarks( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; inNumTicks: ItemCount; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawTrackTickMarks';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawTickMark()
 *  
 *  Summary:
 *    Draws a single tick mark.
 *  
 *  Discussion:
 *    This primitive draws a single tick mark and can be used to draw
 *    custom tick marks that are not easily drawn by
 *    HIThemeDrawTrackTickMarks, which only draws evenly distributed
 *    tick marks.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      The HIThemeTickMarkDrawInfo of the tick mark to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawTickMark( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeTickMarkDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawTickMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTrackThumbShape()
 *  
 *  Summary:
 *    Get the thumb shape of a themed track.
 *  
 *  Discussion:
 *    This API was mistakenly named as a "Get" API. It behaves as
 *    "Copy" API. THE CALLER IS RESPONSIBLE FOR RELEASING THE RETURNED
 *    SHAPE.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track to be measured.
 *    
 *    outThumbShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
<<<<<<< HEAD
<<<<<<< HEAD
 *      the themed track's thumb. It needs to be released by the caller.
=======
 *      the themed track's thumb. Needs to be released by caller.
>>>>>>> graemeg/fixes_2_2
=======
 *      the themed track's thumb. Needs to be released by caller.
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTrackThumbShape( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; var outThumbShape: HIShapeRef ): OSStatus; external name '_HIThemeGetTrackThumbShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeHitTestTrack()
 *  
 *  Summary:
 *    Hit test the themed track.
 *  
 *  Discussion:
 *    Returns true if the track was hit and fills in outPartHit. 
 *    Otherwise, returns false.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTabDrawInfo describing the tab that will be drawn.
 *    
 *    inMousePoint:
 *      An HIPoint which will be location basis for the test.
 *    
 *    outPartHit:
 *      A pointer to a ControlPartCode that will be filled with the
 *      part hit by the incoming point.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeHitTestTrack( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inMousePoint: HIPoint; var outPartHit: ControlPartCode ): Boolean; external name '_HIThemeHitTestTrack';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTrackBounds()
 *  
 *  Summary:
 *    Gets the track bounds of a themed track.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track that will be drawn.
 *    
 *    outBounds:
 *      A pointer to an HIRect in which will be returned the rectangle
 *      of the track bounds.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTrackBounds( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; var outBounds: HIRect ): OSStatus; external name '_HIThemeGetTrackBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTrackPartBounds()
 *  
 *  Summary:
 *    Returns measurements for the bounds of the a track part,
 *    according to the specifics of that track as specified in the
 *    incoming draw info record.
 *  
 *  Discussion:
 *    HIThemeGetTrackPartBounds allows you to get the boundaries of
 *    individual pieces of a track's theme layout.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track to be measured.
 *    
 *    inPartCode:
 *      A ControlPartCode describing which part to measure.  A list of
 *      available ControlPartCodes can be retrieved using
 *      HIThemeGetTrackParts.
 *    
 *    outPartBounds:
 *      The bounds of the specified part relative to the start
 *      rectangle specified in inDrawInfo.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTrackPartBounds( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; inPartCode: ControlPartCode; var outPartBounds: HIRect ): OSStatus; external name '_HIThemeGetTrackPartBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTrackParts()
 *  
 *  Summary:
 *    Counts the number of parts that make up a track.  Optionally
 *    returns an array of ControlPartCodes that describe each of the
 *    counted parts.
 *  
 *  Discussion:
 *    HIThemeGetTrackParts allows you to count the number of parts that
 *    make up a track.  This is useful if you need to iterate through
 *    the parts of a track and get information about them, i.e. using
 *    HIThemeGetTrackPartBounds.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track to be measured.
 *    
 *    outNumberOfParts:
 *      A pointer to a UInt32 in which to return the number of counted
 *      parts.
 *    
 *    inMaxParts:
 *      The maximum number of ControlPartCodes that can be copied into
 *      the supplied ioPartsBuffer.  This value is ignored if
 *      ioPartsBuffer is NULL.
 *    
 *    ioPartsBuffer:
 *      An pointer to an array into which HIThemeGetTrackPartBounds
 *      will copy ControlPartCodes that describe each of the counted
 *      parts.  This pointer can be NULL if you are just counting parts.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTrackParts( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; var outNumberOfParts: UInt32; inMaxParts: UInt32; ioPartsBuffer: ControlPartCodePtr { can be NULL } ): OSStatus; external name '_HIThemeGetTrackParts';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTrackDragRect()
 *  
 *  Summary:
 *    Get the rectangle of the drag area of a themed track.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track to be measured.
 *    
 *    outDragRect:
 *      A pointer to an HIRect in which will be returned the rectangle
 *      of the drag area of the track.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTrackDragRect( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; var outDragRect: HIRect ): OSStatus; external name '_HIThemeGetTrackDragRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTrackThumbPositionFromOffset()
 *  
 *  Summary:
 *    Get the track's relative thumb position based on the offset.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track to be measured.
 *    
 *    inThumbOffset:
 *      An HIPoint describing the position of the thumb as an offset
 *      from the track bounds.
 *    
 *    outRelativePosition:
 *      On output, the track-relative position calculated from the
 *      thumb offset.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function HIThemeGetTrackThumbPositionFromOffset( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inThumbOffset: HIPoint; var outRelativePosition: CGFloat ): OSStatus; external name '_HIThemeGetTrackThumbPositionFromOffset';
=======
function HIThemeGetTrackThumbPositionFromOffset( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inThumbOffset: HIPoint; var outRelativePosition: Float32 ): OSStatus; external name '_HIThemeGetTrackThumbPositionFromOffset';
>>>>>>> graemeg/fixes_2_2
=======
function HIThemeGetTrackThumbPositionFromOffset( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inThumbOffset: HIPoint; var outRelativePosition: Float32 ): OSStatus; external name '_HIThemeGetTrackThumbPositionFromOffset';
>>>>>>> origin/fixes_2_2
=======
function HIThemeGetTrackThumbPositionFromOffset( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inThumbOffset: HIPoint; var outRelativePosition: CGFloat ): OSStatus; external name '_HIThemeGetTrackThumbPositionFromOffset';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTrackThumbPositionFromBounds()
 *  
 *  Summary:
 *    Get the themed track thumb position from its bounds.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      A pointer to an HIThemeTrackDrawInfo describing the track to be
 *      measured.
 *    
 *    inThumbBounds:
 *      The bounds of the thumb from which the postion is to be
 *      calculated.
 *    
 *    outRelativePosition:
 *      On output, the track-relative position calculated from the
 *      thumb location.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function HIThemeGetTrackThumbPositionFromBounds( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inThumbBounds: HIRect; var outRelativePosition: CGFloat ): OSStatus; external name '_HIThemeGetTrackThumbPositionFromBounds';
=======
function HIThemeGetTrackThumbPositionFromBounds( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inThumbBounds: HIRect; var outRelativePosition: Float32 ): OSStatus; external name '_HIThemeGetTrackThumbPositionFromBounds';
>>>>>>> graemeg/fixes_2_2
=======
function HIThemeGetTrackThumbPositionFromBounds( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inThumbBounds: HIRect; var outRelativePosition: Float32 ): OSStatus; external name '_HIThemeGetTrackThumbPositionFromBounds';
>>>>>>> origin/fixes_2_2
=======
function HIThemeGetTrackThumbPositionFromBounds( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; const (*var*) inThumbBounds: HIRect; var outRelativePosition: CGFloat ): OSStatus; external name '_HIThemeGetTrackThumbPositionFromBounds';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetTrackLiveValue()
 *  
 *  Summary:
 *    Get the themed track live value.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrawInfo:
 *      An HIThemeTrackDrawInfo describing the track to be measured.
 *    
 *    inRelativePosition:
 *      An HIPoint describing the position of the thumb as an offset
 *      from the track bounds.
 *    
 *    outValue:
 *      On output, the track value as calculated from the relative
 *      postion of the thumb.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function HIThemeGetTrackLiveValue( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; inRelativePosition: CGFloat; var outValue: SInt32 ): OSStatus; external name '_HIThemeGetTrackLiveValue';
=======
function HIThemeGetTrackLiveValue( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; inRelativePosition: Float32; var outValue: SInt32 ): OSStatus; external name '_HIThemeGetTrackLiveValue';
>>>>>>> graemeg/fixes_2_2
=======
function HIThemeGetTrackLiveValue( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; inRelativePosition: Float32; var outValue: SInt32 ): OSStatus; external name '_HIThemeGetTrackLiveValue';
>>>>>>> origin/fixes_2_2
=======
function HIThemeGetTrackLiveValue( const (*var*) inDrawInfo: HIThemeTrackDrawInfo; inRelativePosition: CGFloat; var outValue: SInt32 ): OSStatus; external name '_HIThemeGetTrackLiveValue';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetScrollBarTrackRect()
 *  
 *  Summary:
 *    Gets the rectangle of the tracking area of a themed scroll bar.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIRect indicating the area in which the scroll bar would be
 *      drawn.
 *    
 *    inTrackInfo:
 *      An HIScrollBarTrackInfo for the scroll bar that would be drawn.
 *      Currently, only the pressState and enableState fields are used.
 *    
 *    inIsHoriz:
 *      A Boolean where true means that the scroll bar would be
 *      horizontal and false means that the scroll bar would be
=======
=======
>>>>>>> origin/fixes_2_2
 *      An HIRect indicating the area in which the scroll bar will be
 *      drawn.
 *    
 *    inTrackInfo:
 *      An HIScrollBarTrackInfo for the scroll bar to be drawn.
 *      Currently, only the pressState and enableState fields are used.
 *    
 *    inIsHoriz:
 *      A Boolean where true means that the scroll bar is to be
 *      horizontal and false means that the scroll bar is to be
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *      vertical.
 *    
 *    outTrackBounds:
 *      A pointer to an HIRect in which will be returned the rectangle
 *      of the track area of the scroll bar.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetScrollBarTrackRect( const (*var*) inBounds: HIRect; const (*var*) inTrackInfo: HIScrollBarTrackInfo; inIsHoriz: Boolean; var outTrackBounds: HIRect ): OSStatus; external name '_HIThemeGetScrollBarTrackRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeHitTestScrollBarArrows()
 *  
 *  Summary:
 *    Hit test the theme scroll bar arrows to determine where the hit
 *    occurred.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inScrollBarBounds:
 *      An HIRect indicating the bounds of the scroll bar that will be
 *      hit tested.
 *    
 *    inTrackInfo:
 *      An HIScrollBarTrackInfo for the scroll bar to be drawn.
 *      Currently, only the version, pressState and enableState fields
 *      are used.
 *    
 *    inIsHoriz:
 *      A Boolean where true means that the scroll bar is to be
 *      horizontal and false means that the scroll bar is to be
 *      vertical.
 *    
 *    inPtHit:
 *      An HIPoint indicating where the control was hit and which will
 *      be used for hit testing.
 *    
 *    outTrackBounds:
 *      A pointer to an HIRect in which will be returned the rectangle
 *      of the track area of the scroll bar.  Can be NULL.
 *    
 *    outPartCode:
 *      A pointer to a ControlPartCode in which the part code of the
 *      hit part will be returned.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeHitTestScrollBarArrows( const (*var*) inScrollBarBounds: HIRect; const (*var*) inTrackInfo: HIScrollBarTrackInfo; inIsHoriz: Boolean; const (*var*) inPtHit: HIPoint; outTrackBounds: HIRectPtr { can be NULL }; var outPartCode: ControlPartCode ): Boolean; external name '_HIThemeHitTestScrollBarArrows';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawScrollBarDelimiters()
 *  
 *  Summary:
 *    Draw themed scroll bar delimiters.
 *  
 *  Discussion:
 *    Draws the grow lines delimiting the scroll bar areas.  Does not
 *    draw the size box.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inContRect:
 *      An HIRect indicating the rectangle of the content area of the
 *      window to be drawn.
 *    
 *    inDrawInfo:
 *      The HIThemeScrollBarDelimitersDrawInfo of the delimiters to be
 *      drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawScrollBarDelimiters( const (*var*) inContRect: HIRect; const (*var*) inDrawInfo: HIThemeScrollBarDelimitersDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawScrollBarDelimiters';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ -------------------------------------------------------------------------- }
{  Windows                                                                   }
{ -------------------------------------------------------------------------- }
{
 *  HIThemeDrawWindowFrame()
 *  
 *  Summary:
 *    Draws a themed window frame.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inContRect:
 *      An HIRect indicating the rectangle of the content area of the
 *      window to be drawn.
 *    
 *    inDrawInfo:
 *      The HIThemeWindowDrawInfo of the window frame to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *    
 *    outTitleRect:
 *      A pointer to an HIRect into which to put the bounds of the
 *      title rect.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawWindowFrame( const (*var*) inContRect: HIRect; const (*var*) inDrawInfo: HIThemeWindowDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation; outTitleRect: HIRectPtr { can be NULL } ): OSStatus; external name '_HIThemeDrawWindowFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawTitleBarWidget()
 *  
 *  Summary:
 *    Draws the requested theme title bar widget.
 *  
 *  Discussion:
 *    HIThemeDrawTitleBarWidget renders the requested theme title bar
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
 *    inContRect:
 *      A rectangle describing the window's content area.  The widget
 *      is drawn relative to the content rectangle of the window, so
 *      this parameter does not describe the actual widget bounds, it
 *      describes the window's content rectangle.
 *    
 *    inDrawInfo:
 *      The HIThemeWindowWidgetDrawInfo of the window widget to be
 *      drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawTitleBarWidget( const (*var*) inContRect: HIRect; const (*var*) inDrawInfo: HIThemeWindowWidgetDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawTitleBarWidget';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawGrowBox()
 *  
 *  Summary:
 *    Draws a theme grow box.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inOrigin:
 *      The origin from which to draw the grow box.
 *    
 *    inDrawInfo:
 *      An HIThemeGrowBoxDrawInfo describing the grow box to be drawn
 *      or measured.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawGrowBox( const (*var*) inOrigin: HIPoint; const (*var*) inDrawInfo: HIThemeGrowBoxDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawGrowBox';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetGrowBoxBounds()
 *  
 *  Summary:
 *    Gets the bounds for a grow box.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inOrigin:
 *      The origin from which to draw the grow box.
 *    
 *    inDrawInfo:
<<<<<<< HEAD
<<<<<<< HEAD
 *      An HIThemeGrowBoxDrawInfo describing the grow box to be
 *      measured. The state field is ignored.
=======
 *      An HIThemeGrowBoxDrawInfo describing the grow box to be drawn
 *      or measured. The state field is ignored.
>>>>>>> graemeg/fixes_2_2
=======
 *      An HIThemeGrowBoxDrawInfo describing the grow box to be drawn
 *      or measured. The state field is ignored.
>>>>>>> origin/fixes_2_2
 *    
 *    outBounds:
 *      A pointer to an HIRect in which will be returned the rectangle
 *      of the standalone grow box bounds.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetGrowBoxBounds( const (*var*) inOrigin: HIPoint; const (*var*) inDrawInfo: HIThemeGrowBoxDrawInfo; var outBounds: HIRect ): OSStatus; external name '_HIThemeGetGrowBoxBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetWindowShape()
 *  
 *  Summary:
 *    Obtains the specified window shape.
 *  
 *  Discussion:
 *    This API was mistakenly named as a "Get" API. It behaves as
 *    "Copy" API. THE CALLER IS RESPONSIBLE FOR RELEASING THE RETURNED
 *    SHAPE.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inContRect:
 *      An HIRect indicating the rectangle of the content area of the
<<<<<<< HEAD
<<<<<<< HEAD
 *      window that would be drawn.
=======
 *      window to be drawn.
>>>>>>> graemeg/fixes_2_2
=======
 *      window to be drawn.
>>>>>>> origin/fixes_2_2
 *    
 *    inDrawInfo:
 *      The HIThemeWindowDrawInfo of the window frame to be measured.
 *    
 *    inWinRegion:
 *      A WindowRegionCode indicating the desired region for which to
 *      return the shape.
 *    
 *    outShape:
 *      A pointer to an HIShapeRef which will be set to the shape of
<<<<<<< HEAD
<<<<<<< HEAD
 *      the requested window region. It needs to be released by the
 *      caller.
=======
 *      the requested window region. Needs to be released by caller.
>>>>>>> graemeg/fixes_2_2
=======
 *      the requested window region. Needs to be released by caller.
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetWindowShape( const (*var*) inContRect: HIRect; const (*var*) inDrawInfo: HIThemeWindowDrawInfo; inWinRegion: WindowRegionCode; var outShape: HIShapeRef ): OSStatus; external name '_HIThemeGetWindowShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeGetWindowRegionHit()
 *  
 *  Summary:
 *    Get the window region hit in a themed window.
 *  
 *  Discussion:
 *    Not that this call does not return a region, but a region code.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inContRect:
 *      An HIRect indicating the rectangle of the content area of the
<<<<<<< HEAD
<<<<<<< HEAD
 *      window that would be drawn.
=======
 *      window to be drawn.
>>>>>>> graemeg/fixes_2_2
=======
 *      window to be drawn.
>>>>>>> origin/fixes_2_2
 *    
 *    inDrawInfo:
 *      The HIThemeWindowDrawInfo of the window frame to be measured.
 *    
 *    inPoint:
 *      An HIPoint against which the test will occur.
 *    
 *    outRegionHit:
 *      The output WindowRegionCode of hit window region.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetWindowRegionHit( const (*var*) inContRect: HIRect; const (*var*) inDrawInfo: HIThemeWindowDrawInfo; const (*var*) inPoint: HIPoint; var outRegionHit: WindowRegionCode ): Boolean; external name '_HIThemeGetWindowRegionHit';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ -------------------------------------------------------------------------- }
{  Miscellaneous                                                             }
{ -------------------------------------------------------------------------- }
{
 *  HIThemeDrawFrame()
 *  
 *  Summary:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Draws a variety of frames.
=======
 *    Draws a variety of frames frame.
>>>>>>> graemeg/fixes_2_2
=======
 *    Draws a variety of frames frame.
>>>>>>> origin/fixes_2_2
=======
 *    Draws a variety of frames.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      An HIThemeFrameDrawInfo describing the frame to be drawn or
 *      measured.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawFrame( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeFrameDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawGroupBox()
 *  
 *  Summary:
 *    Draws a themed primary group box.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      An HIThemeGroupBoxDrawInfo describing the group box to be drawn
 *      or measured.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawGroupBox( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeGroupBoxDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawGroupBox';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawGenericWell()
 *  
 *  Summary:
 *    Draws a themed generic well.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      An HIThemeButtonDrawInfo that describes attributes of the well
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      to be drawn. Set the kThemeAdornmentDefault bit of the
 *      adornment field of this structure to also draw the center of
 *      the well.
=======
 *      to be drawn.
>>>>>>> graemeg/fixes_2_2
=======
 *      to be drawn.
>>>>>>> origin/fixes_2_2
=======
 *      to be drawn. Set the kThemeAdornmentDefault bit of the
 *      adornment field of this structure to also draw the center of
 *      the well.
>>>>>>> origin/fixes_2.4
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawGenericWell( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeButtonDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawGenericWell';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawPaneSplitter()
 *  
 *  Summary:
 *    Draws a themed pane splitter.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      The HIThemeSplitterDrawInfo of the pane splitter to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawPaneSplitter( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeSplitterDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawPaneSplitter';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawGrabber()
 *  
 *  Summary:
 *    Draws a themed grabber.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      The HIThemeGrabberDrawInfo of the grabber to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawGrabber( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeGrabberDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawGrabber';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawPlacard()
 *  
 *  Summary:
 *    Draws a themed placard.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      The HIThemePlacardDrawInfo of the placard to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawPlacard( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemePlacardDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawPlacard';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawHeader()
 *  
 *  Summary:
 *    Draws a themed window header in the specified rectangle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      The HIThemeHeaderDrawInfo of the window frame to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawHeader( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeHeaderDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawHeader';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawFocusRect()
 *  
 *  Summary:
 *    Draws a themed focus rectangle in the specified rectangle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inHasFocus:
 *      Pass in true to draw focus. Passing false effectively makes
 *      this API a no-op.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawFocusRect( const (*var*) inRect: HIRect; inHasFocus: Boolean; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawFocusRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const
{
   * Draw the visual focus only, and not any of the draw operations
   * that form its shape.
   }
	kHIThemeFocusRingOnly = 0;

  {
   * Draw the visual focus above the results of the draw operations
   * that form its shape.
   }
	kHIThemeFocusRingAbove = 1;

  {
   * Draw the visual focus below the results of the draw operations
   * that form its shape.
   }
	kHIThemeFocusRingBelow = 2;


type
	HIThemeFocusRing = UInt32;
{
 *  HIThemeBeginFocus()
 *  
 *  Summary:
 *    Begin focus drawing.
 *  
 *  Discussion:
 *    Call HIThemeBeginFocus to begin focus drawing. All drawing
 *    operations in the specified context after this call will be drawn
 *    with a visual representation of focus. Currently, this is a
 *    theme-tinted halo resembling a glow. Note that nothing will be
 *    drawn in the specified context until HIThemeEndFocus is called. A
 *    call to HIThemeBeginFocus must always be paired with an
 *    HIThemeEndFocus call. Nesting these calls will not crash but the
 *    results will be odd and is definitely not recommended.
 *    HIThemeBegin/EndFocus is designed to replace
 *    DrawThemeFocusRegion. For efficiency, clipping the context to the
 *    bounds that will be drawn into is highly desirable.
 *    HIThemeBeginFocus may do some allocations that are affected by
 *    the size of the context's clip at the time it is called -- so an
 *    extremely large clip or an unset clip may cause a large,
 *    inefficient allocation.
<<<<<<< HEAD
=======
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inContext:
 *      The CG context in which the focus is to be drawn.
 *    
 *    inRing:
 *      An HIThemeFocusRing indicating which type of focus is to be
 *      drawn.
 *    
 *    inReserved:
 *      Always pass NULL for this parameter.
 *  
 *  Result:
 *    A result code indicating success or failure. Don't call
 *    HIThemeEndFocus on the context if HIThemeBeginFocus fails.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIThemeBeginFocus( inContext: CGContextRef; inRing: HIThemeFocusRing; inReserved: UnivPtr ): OSStatus; external name '_HIThemeBeginFocus';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIThemeEndFocus()
 *  
 *  Summary:
 *    End focus drawing.
 *  
 *  Discussion:
 *    See HIThemeBeginFocus for focus drawing details. Calling
 *    HIThemeEndFocus indicates that the drawing operations to be
 *    focused are complete.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inContext:
 *      The CG context in which the focus is to be drawn. This needs to
 *      be the same context passed to the HIThemeBeginFocus call with
 *      which this call to HIThemeEndFocus is paired.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIThemeEndFocus( inContext: CGContextRef ): OSStatus; external name '_HIThemeEndFocus';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  HIThemeDrawSeparator()
 *  
 *  Summary:
 *    Draw a themed separator element.
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inContext:
 *      The CG context in which the focus is to be drawn.
 *    
 *    inRing:
 *      An HIThemeFocusRing indicating which type of focus is to be
 *      drawn.
 *    
 *    inReserved:
 *      Always pass NULL for this parameter.
 *  
 *  Result:
 *    A result code indicating success or failure. Don't call
 *    HIThemeEndFocus on the context if HIThemeBeginFocus fails.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIThemeBeginFocus( inContext: CGContextRef; inRing: HIThemeFocusRing; inReserved: UnivPtr ): OSStatus; external name '_HIThemeBeginFocus';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIThemeEndFocus()
 *  
 *  Summary:
 *    End focus drawing.
 *  
 *  Discussion:
 *    See HIThemeBeginFocus for focus drawing details. Calling
 *    HIThemeEndFocus indicates that the drawing operations to be
 *    focused are complete.
=======
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      The HIThemeSeparatorDrawInfo of the window frame to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawSeparator( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeSeparatorDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawSeparator';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeSetFill()
 *  
 *  Summary:
 *    Set the context fill color to that specified by the requested
 *    brush.
 *  
 *  Discussion:
 *    Note that this call does not actually draw anything. It sets the
 *    passed context's fill color to that of the specified theme brush.
 *    Subsequent fills in the context will be with the color specified
 *    by the theme brush.
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inContext:
 *      The CG context in which the focus is to be drawn. This needs to
 *      be the same context passed to the HIThemeBeginFocus call with
 *      which this call to HIThemeEndFocus is paired.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIThemeEndFocus( inContext: CGContextRef ): OSStatus; external name '_HIThemeEndFocus';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


=======
>>>>>>> graemeg/fixes_2_2
{
 *  HIThemeDrawSeparator()
 *  
 *  Summary:
 *    Draw a themed separator element.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The HIRect in which to draw.
 *    
 *    inDrawInfo:
 *      The HIThemeSeparatorDrawInfo of the window frame to be drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawSeparator( const (*var*) inRect: HIRect; const (*var*) inDrawInfo: HIThemeSeparatorDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawSeparator';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeSetFill()
 *  
 *  Summary:
 *    Set the context fill color to that specified by the requested
 *    brush.
 *  
 *  Discussion:
 *    Note that this call does not actually draw anything. It sets the
 *    passed context's fill color to that of the specified theme brush.
 *    Subsequent fills in the context will be with the color specified
 *    by the theme brush.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBrush:
 *      The ThemeBrush describing the requested fill color.
 *    
 *    inInfo:
 *      Not used. Should always be NULL.
 *    
=======
 *    inBrush:
 *      The ThemeBrush describing the requested fill color.
 *    
 *    inInfo:
 *      Not used. Should always be NULL.
 *    
>>>>>>> origin/fixes_2_2
 *    inContext:
 *      The CG context for which the fill color is to be set.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeSetFill( inBrush: ThemeBrush; inInfo: UnivPtr; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeSetFill';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HIThemeSetStroke()
 *  
 *  Summary:
 *    Set the context stroke color to that specified by the requested
 *    brush.
 *  
 *  Discussion:
 *    Note that this call does not actually draw anything. It sets the
 *    passed context's stroke color to that of the specified theme
 *    brush. Subsequent strokes in the context will be with the color
 *    specified by the theme brush.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBrush:
 *      The ThemeBrush describing the requested stroke color.
 *    
 *    inInfo:
 *      Not used. Should always be NULL.
 *    
 *    inContext:
 *      The CG context for which the stroke color is to be set.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeSetStroke( inBrush: ThemeBrush; inInfo: UnivPtr; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeSetStroke';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HIThemeSetTextFill()
 *  
 *  Summary:
 *    Set the context text fill color to that specified by the
 *    requested brush.
 *  
 *  Discussion:
 *    Note that this call does not actually draw anything. It sets the
 *    passed context's text fill color to that of the specified
 *    ThemeTextColor. Subsequent text drawing in the context will be
 *    with the color specified by the ThemeTextColor.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inColor:
 *      A ThemeTextColor describing the requested text fill color.
 *    
 *    inInfo:
 *      Not used. Should always be NULL.
 *    
 *    inContext:
 *      The CG context for which the fill color is to be set.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeSetTextFill( inColor: ThemeTextColor; inInfo: UnivPtr; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeSetTextFill';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HIThemeApplyBackground()
 *  
 *  Summary:
 *    Apply a themed background for a rectangle.
 *  
 *  Discussion:
 *    Note that this call does not actually draw anything. It sets the
 *    passed context's fill color to the requested theme background.
 *    Subsequent fills in the context will fill with the theme
 *    background.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      An HIRect enclosing the whole background. This rectangle is
 *      used to calculate the pattern phase (if there is one) of the
 *      background as it is set up in the context.
 *    
 *    inDrawInfo:
 *      An HIThemeBackgroundDrawInfo describing the background.
 *    
 *    inContext:
 *      The CG context for which the background is to be set.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeApplyBackground( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeBackgroundDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeApplyBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeDrawBackground()
 *  
 *  Summary:
 *    Draw a themed background for a rectangle.
 *  
 *  Discussion:
 *    Currently, this call only works with kThemeBackgroundMetal.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBounds:
 *      An HIRect indicating the bounds to fill with the background.
 *      For backgrounds that need pattern continuity, such as
 *      kThemeBackgroundMetal, this rectangle is the full bounds of the
 *      rectangle for which the filling is to occur. If drawing a
 *      sub-rectangle of that background, set the clip and draw the
 *      full rectangle. This routine has been optimized to not perform
 *      calculations on the non-clip part of the drawing bounds.
 *    
 *    inDrawInfo:
 *      An HIThemeBackgroundDrawInfo describing the background to be
 *      drawn.
 *    
 *    inContext:
 *      The CG context in which the drawing is to be done.
 *    
 *    inOrientation:
 *      An HIThemeOrientation that describes the orientation of the
 *      passed in context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeDrawBackground( const (*var*) inBounds: HIRect; const (*var*) inDrawInfo: HIThemeBackgroundDrawInfo; inContext: CGContextRef; inOrientation: HIThemeOrientation ): OSStatus; external name '_HIThemeDrawBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIThemeBrushCreateCGColor()
 *  
 *  Summary:
 *    Create a CGColor for a ThemeBrush.
 *  
 *  Discussion:
 *    Color is an ambiguous term. The color may be a pattern.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBrush:
 *      The ThemeBrush describing the requested color.
 *    
 *    outColor:
 *      A pointer to a CGColorRef that will be set to the newly created
 *      CGColor.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIThemeBrushCreateCGColor( inBrush: ThemeBrush; var outColor: CGColorRef ): OSStatus; external name '_HIThemeBrushCreateCGColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  HIThemeGetTextColorForThemeBrush()
 *  
 *  Summary:
 *    Returns an appropriate ThemeTextColor that matches a ThemeBrush.
 *  
 *  Discussion:
 *    Creates a ThemeTextColor for use with HIThemeSetTextFill.
 *    ThemeTextColors are currently availabe for these theme brushes:
 *    
 *    
 *    kThemeBrushDialogBackgroundActive/Inactive 
 *    
 *    kThemeBrushAlertBackgroundActive/Inactive 
 *    
 *    kThemeBrushModelessDialogBackgroundActive/Inactive 
 *    <BR> kThemeBrushNotificationWindowBackground
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBrush:
 *      The ThemeBrush describing the requested color.
 *    
 *    inWindowIsActive:
 *      Whether the text color should indicate an active or inactive
 *      state.
 *    
 *    outColor:
 *      A pointer to a ThemeTextColor that will be set to the matched
 *      color.
 *  
 *  Result:
 *    An operating system result code. themeNoAppropriateBrushErr will
 *    be returned if no matching ThemeTextColor exists.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIThemeGetTextColorForThemeBrush( inBrush: ThemeBrush; inWindowIsActive: Boolean; var outColor: ThemeTextColor ): OSStatus; external name '_HIThemeGetTextColorForThemeBrush';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{ The following routines were in Appearance.h prior to 10.6 }
{
 *  GetThemeMenuSeparatorHeight()
 *  
 *  Summary:
 *    Returns the height of a menu item separator, in points.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outHeight:
 *      On exit, contains the height of a menu item separator, in
 *      points.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuSeparatorHeight( var outHeight: SInt16 ): OSStatus; external name '_GetThemeMenuSeparatorHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeMenuItemExtra()
 *  
 *  Summary:
 *    Returns the extra width and height required for a menu item
 *    beyond the height of the text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inItemType:
 *      The type of menu item. These are defined in Appearance.h.
 *    
 *    outHeight:
 *      Extra height, in points, for this item type.
 *    
 *    outWidth:
 *      Extra width, in points, for this item type.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuItemExtra( inItemType: ThemeMenuItemType; var outHeight: SInt16; var outWidth: SInt16 ): OSStatus; external name '_GetThemeMenuItemExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetThemeMenuTitleExtra()
 *  
 *  Summary:
 *    Returns the extra width for a menu title, in points.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outWidth:
 *      On exit, contains the extra menu title width, in points.
 *    
 *    inIsSquished:
 *      Indicates whether the menu title is being drawn with a
 *      condensed appearance.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetThemeMenuTitleExtra( var outWidth: SInt16; inIsSquished: Boolean ): OSStatus; external name '_GetThemeMenuTitleExtra';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ The ThemeMetric values are defined in Appearance.h }

{
 *  Summary:
 *    Theme metrics allow you to find out sizes of things in the
 *    current environment, such as how wide a scroll bar is, etc.
 *  
 *  Discussion:
 *    ThemeMetrics
 }
const
{
   * The width (or height if horizontal) of a scroll bar.
   }
	kThemeMetricScrollBarWidth = 0;

  {
   * The width (or height if horizontal) of a small scroll bar.
   }
	kThemeMetricSmallScrollBarWidth = 1;

  {
   * The height of the non-label part of a check box control.
   }
	kThemeMetricCheckBoxHeight = 2;

  {
   * The height of the non-label part of a radio button control.
   }
	kThemeMetricRadioButtonHeight = 3;

  {
   * The amount of white space surrounding the text Rect of the text
   * inside of an Edit Text control.  If you select all of the text in
   * an Edit Text control, you can see the white space. The metric is
   * the number of pixels, per side, that the text Rect is outset to
   * create the whitespace Rect.
   }
	kThemeMetricEditTextWhitespace = 4;

  {
   * The thickness of the Edit Text frame that surrounds the whitespace
   * Rect (that is surrounding the text Rect). The metric is the number
   * of pixels, per side, that the frame Rect is outset from the
   * whitespace Rect.
   }
	kThemeMetricEditTextFrameOutset = 5;

  {
   * The number of pixels that the list box frame is outset from the
   * content of the list box.
   }
	kThemeMetricListBoxFrameOutset = 6;

  {
   * Describes how far outside of the input rect DrawThemeFocusRect and
   * HIThemeDrawFocusRect APIs will draw the focus.
   }
	kThemeMetricFocusRectOutset = 7;

  {
   * The thickness of the frame drawn by DrawThemeGenericWell.
   }
	kThemeMetricImageWellThickness = 8;

  {
   * The number of pixels a scrollbar should overlap (actually
   * underlap) any bounding box which surrounds it and scrollable
   * content. This also includes the window frame when a scrolbar is
   * along an edge of the window.
   }
	kThemeMetricScrollBarOverlap = 9;

  {
   * The height of the large tab of a tab control.
   }
	kThemeMetricLargeTabHeight = 10;

  {
   * The width of the caps (end pieces) of the large tabs of a tab
   * control.
   }
	kThemeMetricLargeTabCapsWidth = 11;

  {
   * The amount to add to the tab height (kThemeMetricLargeTabHeight)
   * to find out the rectangle height to use with the various Tab
   * drawing primitives. This amount is also the amount that each tab
   * overlaps the tab pane.
   }
	kThemeMetricTabFrameOverlap = 12;

  {
   * If less than zero, this indicates that the text should be centered
   * on each tab. If greater than zero, the text should be justified
   * (according to the system script direction) and the amount is the
   * offset from the appropriate edge at which the text should start
   * drawing.
   }
	kThemeMetricTabIndentOrStyle = 13;

  {
   * The amount of space that every tab's drawing rectangle overlaps
   * the one on either side of it.
   }
	kThemeMetricTabOverlap = 14;

  {
   * The height of the small tab of a tab control.  This includes the
   * pixels that overlap the tab pane and/or tab pane bar.
   }
	kThemeMetricSmallTabHeight = 15;

  {
   * The width of the caps (end pieces) of the small tabs of a tab
   * control.
   }
	kThemeMetricSmallTabCapsWidth = 16;

  {
   * The height of the push button control.
   }
	kThemeMetricPushButtonHeight = 19;

  {
   * The height of the list header field of the data browser control.
   }
	kThemeMetricListHeaderHeight = 20;

  {
   * The height of a disclosure triangle control.  This triangle is the
   * not the center of the disclosure button, but its own control.
   }
	kThemeMetricDisclosureTriangleHeight = 25;

  {
   * The width of a disclosure triangle control.
   }
	kThemeMetricDisclosureTriangleWidth = 26;

  {
   * The height of a little arrows control.
   }
	kThemeMetricLittleArrowsHeight = 27;

  {
   * The width of a little arrows control.
   }
	kThemeMetricLittleArrowsWidth = 28;

  {
   * The height of a popup button control.
   }
	kThemeMetricPopupButtonHeight = 30;

  {
   * The height of a small popup button control.
   }
	kThemeMetricSmallPopupButtonHeight = 31;

  {
   * The height of the large progress bar, not including its shadow.
   }
	kThemeMetricLargeProgressBarThickness = 32;

  {
   * This metric is not used.
   }
	kThemeMetricPullDownHeight = 33;

  {
   * This metric is not used.
   }
	kThemeMetricSmallPullDownHeight = 34;

  {
   * The height of the window grow box control.
   }
	kThemeMetricResizeControlHeight = 38;

  {
   * The height of the small grow box control, such as on utility
   * windows.
   }
	kThemeMetricSmallResizeControlHeight = 39;

  {
   * The height of the horizontal slider control.
   }
	kThemeMetricHSliderHeight = 41;

  {
   * The height of the tick marks for a horizontal slider control.
   }
	kThemeMetricHSliderTickHeight = 42;

  {
   * The width of the vertical slider control.
   }
	kThemeMetricVSliderWidth = 45;

  {
   * The width of the tick marks for a vertical slider control.
   }
	kThemeMetricVSliderTickWidth = 46;

  {
   * The height of the title bar widgets (grow, close, and zoom boxes)
   * for a document window.
   }
	kThemeMetricTitleBarControlsHeight = 49;

  {
   * The width of the non-label part of a check box control.
   }
	kThemeMetricCheckBoxWidth = 50;

  {
   * The width of the non-label part of a radio button control.
   }
	kThemeMetricRadioButtonWidth = 52;

  {
   * The height of the normal bar, not including its shadow.
   }
	kThemeMetricNormalProgressBarThickness = 58;

  {
   * The number of pixels of shadow depth drawn below the progress bar.
   }
	kThemeMetricProgressBarShadowOutset = 59;

  {
   * The number of pixels of shadow depth drawn below the small
   * progress bar.
   }
	kThemeMetricSmallProgressBarShadowOutset = 60;

  {
   * The number of pixels that the content of a primary group box is
   * from the bounds of the control.
   }
	kThemeMetricPrimaryGroupBoxContentInset = 61;

  {
   * The number of pixels that the content of a secondary group box is
   * from the bounds of the control.
   }
	kThemeMetricSecondaryGroupBoxContentInset = 62;

  {
   * Width allocated to draw the mark character in a menu.
   }
	kThemeMetricMenuMarkColumnWidth = 63;

  {
   * Width allocated for the mark character in a menu item when the
   * menu has kMenuAttrExcludesMarkColumn.
   }
	kThemeMetricMenuExcludedMarkColumnWidth = 64;

  {
   * Indent into the interior of the mark column at which the mark
   * character is drawn.
   }
	kThemeMetricMenuMarkIndent = 65;

  {
   * Whitespace at the leading edge of menu item text.
   }
	kThemeMetricMenuTextLeadingEdgeMargin = 66;

  {
   * Whitespace at the trailing edge of menu item text.
   }
	kThemeMetricMenuTextTrailingEdgeMargin = 67;

  {
   * Width per indent level (set by SetMenuItemIndent) of a menu item.
   }
	kThemeMetricMenuIndentWidth = 68;

  {
   * Whitespace at the trailing edge of a menu icon (if the item also
   * has text).
   }
	kThemeMetricMenuIconTrailingEdgeMargin = 69;


{
 *  Discussion:
 *    The following metrics are only available in OS X.
 }
const
{
   * The height of a disclosure button.
   }
	kThemeMetricDisclosureButtonHeight = 17;

  {
   * The height and the width of the round button control.
   }
	kThemeMetricRoundButtonSize = 18;

  {
   * The height of the non-label part of a small check box control.
   }
	kThemeMetricSmallCheckBoxHeight = 21;

  {
   * The width of a disclosure button.
   }
	kThemeMetricDisclosureButtonWidth = 22;

  {
   * The height of a small disclosure button.
   }
	kThemeMetricSmallDisclosureButtonHeight = 23;

  {
   * The width of a small disclosure button.
   }
	kThemeMetricSmallDisclosureButtonWidth = 24;

  {
   * The height (or width if vertical) of a pane splitter.
   }
	kThemeMetricPaneSplitterHeight = 29;

  {
   * The height of the small push button control.
   }
	kThemeMetricSmallPushButtonHeight = 35;

  {
   * The height of the non-label part of a small radio button control.
   }
	kThemeMetricSmallRadioButtonHeight = 36;

  {
   * The height of the relevance indicator control.
   }
	kThemeMetricRelevanceIndicatorHeight = 37;

  {
   * The height and the width of the large round button control.
   }
	kThemeMetricLargeRoundButtonSize = 40;

  {
   * The height of the small, horizontal slider control.
   }
	kThemeMetricSmallHSliderHeight = 43;

  {
   * The height of the tick marks for a small, horizontal slider
   * control.
   }
	kThemeMetricSmallHSliderTickHeight = 44;

  {
   * The width of the small, vertical slider control.
   }
	kThemeMetricSmallVSliderWidth = 47;

  {
   * The width of the tick marks for a small, vertical slider control.
   }
	kThemeMetricSmallVSliderTickWidth = 48;

  {
   * The width of the non-label part of a small check box control.
   }
	kThemeMetricSmallCheckBoxWidth = 51;

  {
   * The width of the non-label part of a small radio button control.
   }
	kThemeMetricSmallRadioButtonWidth = 53;

  {
   * The minimum width of the thumb of a small, horizontal slider
   * control.
   }
	kThemeMetricSmallHSliderMinThumbWidth = 54;

  {
   * The minimum width of the thumb of a small, vertical slider control.
   }
	kThemeMetricSmallVSliderMinThumbHeight = 55;

  {
   * The offset of the tick marks from the appropriate side of a small
   * horizontal slider control.
   }
	kThemeMetricSmallHSliderTickOffset = 56;

  {
   * The offset of the tick marks from the appropriate side of a small
   * vertical slider control.
   }
	kThemeMetricSmallVSliderTickOffset = 57;


{
 *  Discussion:
 *    The following metrics are only available in Mac OS X 10.3 and
 *    later.
 }
const
	kThemeMetricComboBoxLargeBottomShadowOffset = 70;
	kThemeMetricComboBoxLargeRightShadowOffset = 71;
	kThemeMetricComboBoxSmallBottomShadowOffset = 72;
	kThemeMetricComboBoxSmallRightShadowOffset = 73;
	kThemeMetricComboBoxLargeDisclosureWidth = 74;
	kThemeMetricComboBoxSmallDisclosureWidth = 75;
	kThemeMetricRoundTextFieldContentInsetLeft = 76;
	kThemeMetricRoundTextFieldContentInsetRight = 77;
	kThemeMetricRoundTextFieldContentInsetBottom = 78;
	kThemeMetricRoundTextFieldContentInsetTop = 79;
	kThemeMetricRoundTextFieldContentHeight = 80;
	kThemeMetricComboBoxMiniBottomShadowOffset = 81;
	kThemeMetricComboBoxMiniDisclosureWidth = 82;
	kThemeMetricComboBoxMiniRightShadowOffset = 83;
	kThemeMetricLittleArrowsMiniHeight = 84;
	kThemeMetricLittleArrowsMiniWidth = 85;
	kThemeMetricLittleArrowsSmallHeight = 86;
	kThemeMetricLittleArrowsSmallWidth = 87;
	kThemeMetricMiniCheckBoxHeight = 88;
	kThemeMetricMiniCheckBoxWidth = 89;
	kThemeMetricMiniDisclosureButtonHeight = 90;
	kThemeMetricMiniDisclosureButtonWidth = 91;
	kThemeMetricMiniHSliderHeight = 92;
	kThemeMetricMiniHSliderMinThumbWidth = 93;
	kThemeMetricMiniHSliderTickHeight = 94;
	kThemeMetricMiniHSliderTickOffset = 95;
	kThemeMetricMiniPopupButtonHeight = 96;
	kThemeMetricMiniPullDownHeight = 97;
	kThemeMetricMiniPushButtonHeight = 98;
	kThemeMetricMiniRadioButtonHeight = 99;
	kThemeMetricMiniRadioButtonWidth = 100;
	kThemeMetricMiniTabCapsWidth = 101;
	kThemeMetricMiniTabFrameOverlap = 102;
	kThemeMetricMiniTabHeight = 103;
	kThemeMetricMiniTabOverlap = 104;
	kThemeMetricMiniVSliderMinThumbHeight = 105;
	kThemeMetricMiniVSliderTickOffset = 106;
	kThemeMetricMiniVSliderTickWidth = 107;
	kThemeMetricMiniVSliderWidth = 108;
	kThemeMetricRoundTextFieldContentInsetWithIconLeft = 109;
	kThemeMetricRoundTextFieldContentInsetWithIconRight = 110;
	kThemeMetricRoundTextFieldMiniContentHeight = 111;
	kThemeMetricRoundTextFieldMiniContentInsetBottom = 112;
	kThemeMetricRoundTextFieldMiniContentInsetLeft = 113;
	kThemeMetricRoundTextFieldMiniContentInsetRight = 114;
	kThemeMetricRoundTextFieldMiniContentInsetTop = 115;
	kThemeMetricRoundTextFieldMiniContentInsetWithIconLeft = 116;
	kThemeMetricRoundTextFieldMiniContentInsetWithIconRight = 117;
	kThemeMetricRoundTextFieldSmallContentHeight = 118;
	kThemeMetricRoundTextFieldSmallContentInsetBottom = 119;
	kThemeMetricRoundTextFieldSmallContentInsetLeft = 120;
	kThemeMetricRoundTextFieldSmallContentInsetRight = 121;
	kThemeMetricRoundTextFieldSmallContentInsetTop = 122;
	kThemeMetricRoundTextFieldSmallContentInsetWithIconLeft = 123;
	kThemeMetricRoundTextFieldSmallContentInsetWithIconRight = 124;
	kThemeMetricSmallTabFrameOverlap = 125;
	kThemeMetricSmallTabOverlap = 126;

  {
   * The height of a small pane splitter. Should only be used in a
   * window with thick borders, like a metal window.
   }
	kThemeMetricSmallPaneSplitterHeight = 127;


{
 *  Discussion:
 *    The following metrics are only available in Mac OS X 10.4 and
 *    later.
 }
const
{
   * The horizontal start offset for the first tick mark on a
   * horizontal slider.
   }
	kThemeMetricHSliderTickOffset = 128;

  {
   * The vertical start offset for the first tick mark on a vertical
   * slider.
   }
	kThemeMetricVSliderTickOffset = 129;

  {
   * The minimum thumb height for a thumb on a slider.
   }
	kThemeMetricSliderMinThumbHeight = 130;
	kThemeMetricSliderMinThumbWidth = 131;

  {
   * The minimum thumb height for a thumb on a scroll bar.
   }
	kThemeMetricScrollBarMinThumbHeight = 132;

  {
   * The minimum thumb width for a thumb on a scroll bar.
   }
	kThemeMetricScrollBarMinThumbWidth = 133;

  {
   * The minimum thumb height for a thumb on a small scroll bar.
   }
	kThemeMetricSmallScrollBarMinThumbHeight = 134;

  {
   * The minimum thumb width for a thumb on a small scroll bar.
   }
	kThemeMetricSmallScrollBarMinThumbWidth = 135;

  {
   * The height of the round-ended button. (For example, the Kind
   * button in a Finder Search query.)
   }
	kThemeMetricButtonRoundedHeight = 136;

  {
   * The height of the inset round-ended button. (For example, the
   * Servers button in a Finder Search query.)
   }
	kThemeMetricButtonRoundedRecessedHeight = 137;


{
 *  Discussion:
 *    The following metrics are only available in Mac OS X 10.5 and
 *    later.
 }
const
{
   * This metric refers to the appearance of the separator control.
   * That separator is drawn with the HIThemeDrawSeparator theme
   * primitive. This metric is the height of a horizontal separator or
   * the width of a vertical separator.
   }
	kThemeMetricSeparatorSize = 138;

  {
   * The height of the push button control variant that is designed to
   * be used in a textured window.
   }
	kThemeMetricTexturedPushButtonHeight = 139;

  {
   * The height of the small push button control variant that is
   * designed to be used in a textured window.
   }
	kThemeMetricTexturedSmallPushButtonHeight = 140;

type
	ThemeMetric = UInt32;
{
 *  GetThemeMetric()
 *  
 *  Summary:
 *    Returns a measurement in points for a specified type of user
 *    interface element.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMetric:
 *      The metric to retrieve.
 *    
 *    outMetric:
 *      The size of the specified user interface element, in points.
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
 *  Summary:
 *    Retrieves a string identifying the current theme variant, which
 *    may be Aqua or Graphite.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outIdentifier:
 *      On exit, contains the theme variant identifier. This string
 *      must be released by the caller.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function CopyThemeIdentifier( var outIdentifier: CFStringRef ): OSStatus; external name '_CopyThemeIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Obsolete symbolic names                                                                          }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kThemeMetricCheckBoxGlyphHeight = kThemeMetricCheckBoxHeight;
	kThemeMetricRadioButtonGlyphHeight = kThemeMetricRadioButtonHeight;
	kThemeMetricDisclosureButtonSize = kThemeMetricDisclosureButtonHeight;
	kThemeMetricBestListHeaderHeight = kThemeMetricListHeaderHeight;
	kThemeMetricSmallProgressBarThickness = kThemeMetricNormalProgressBarThickness; { obsolete }
	kThemeMetricProgressBarThickness = kThemeMetricLargeProgressBarThickness; { obsolete }
<<<<<<< HEAD

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======


end.
>>>>>>> graemeg/fixes_2_2
=======
=======
>>>>>>> origin/fixes_2.4

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
