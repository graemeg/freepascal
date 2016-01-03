{
     File:       HIToolbox/Menus.h
 
     Contains:   Menu Manager Interfaces.
 
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
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved.
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1985-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1985-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved.
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
<<<<<<< HEAD
    Version 210
    Please report any bugs to <gpc@microbizz.nl>
}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit Menus;
interface
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
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> origin/fixes_2_2
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2_2

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
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
	{$setc TARGET_CPU_X86 := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
{$elsec}
	{$error Neither __ppc__ nor __i386__ is defined.}
{$endc}
{$setc TARGET_CPU_PPC_64 := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> origin/fixes_2_2
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
<<<<<<< HEAD
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
<<<<<<< HEAD
uses MacTypes,AEDataModel,Appearance,CFBase,CFArray,CFAttributedString,CGContext,CTFont,ATSTypes,Events,QuickdrawTypes,TextCommon,Collections,CarbonEventsCore,HIObject;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,AEDataModel,CFBase,CGContext,ATSTypes,Events,Quickdraw,Fonts,TextCommon,Processes,AppleEvents,Collections,MacErrors,CFString,CFUUID,CarbonEventsCore;

>>>>>>> graemeg/fixes_2_2

{$ALIGN MAC68K}


{
 *  Menu Manager
 }
{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Constants                                                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
<<<<<<< HEAD

const
{
   * Proc ID for a normal text menu. This constant is not typically
   * used.
   }
	textMenuProc = 0;

  {
   * This value may be passed to InsertMenuItem, InsertMenuItemText,
   * and InsertMenuItemTextWithCFString to indicate that the new item
   * should be inserted at the end of the menu. Note that you can also
   * just call AppendMenu[ItemText][WithCFString].
   }
	kHIMenuAppendItem = $0000FFFF;

const
	noMark = 0;     { mark symbol for SetItemMark; other mark symbols are defined in Fonts.h }

{ Constants for use with MacOS 8.0 (Appearance 1.0) and later}
const
	kMenuStdMenuProc = 63;
	kMenuStdMenuBarProc = 63;

{ For use with Get/SetMenuItemModifiers}
const
	kMenuNoModifiers = 0;    { Mask for no modifiers}
	kMenuShiftModifier = 1 shl 0; { Mask for shift key modifier}
	kMenuOptionModifier = 1 shl 1; { Mask for option key modifier}
	kMenuControlModifier = 1 shl 2; { Mask for control key modifier}
	kMenuNoCommandModifier = 1 shl 3; { Mask for no command key modifier}

{ For use with Get/SetMenuItemIconHandle}
const
	kMenuNoIcon = 0;    { No icon}
	kMenuIconType = 1;    { Type for ICON}
	kMenuShrinkIconType = 2;    { Type for ICON plotted 16 x 16}
	kMenuSmallIconType = 3;    { Type for SICN}
	kMenuColorIconType = 4;    { Type for cicn}
	kMenuIconSuiteType = 5;    { Type for Icon Suite}
	kMenuIconRefType = 6;    { Type for Icon Ref}
	kMenuCGImageRefType = 7;    { Type for a CGImageRef (Mac OS X only)}
	kMenuSystemIconSelectorType = 8;    { Type for an OSType identifying an IconRef registered with Icon Services under kSystemIconsCreator (Mac OS X 10.1 and later only)}
	kMenuIconResourceType = 9;     { Type for a CFStringRef with the full name of a .icns resource in the main bundle of the process (Mac OS X 10.1 and later only)}

{ For use with Get/SetMenuItemKeyGlyph}
const
	kMenuNullGlyph = $00; { Null (always glyph 1)}
	kMenuTabRightGlyph = $02; { Tab to the right key (for left-to-right script systems)}
	kMenuTabLeftGlyph = $03; { Tab to the left key (for right-to-left script systems)}
	kMenuEnterGlyph = $04; { Enter key}
	kMenuShiftGlyph = $05; { Shift key}
	kMenuControlGlyph = $06; { Control key}
	kMenuOptionGlyph = $07; { Option key}
	kMenuSpaceGlyph = $09; { Space (always glyph 3) key}
	kMenuDeleteRightGlyph = $0A; { Delete to the right key (for right-to-left script systems)}
	kMenuReturnGlyph = $0B; { Return key (for left-to-right script systems)}
	kMenuReturnR2LGlyph = $0C; { Return key (for right-to-left script systems)}
	kMenuNonmarkingReturnGlyph = $0D; { Nonmarking return key}
	kMenuPencilGlyph = $0F; { Pencil key}
	kMenuDownwardArrowDashedGlyph = $10; { Downward dashed arrow key}
	kMenuCommandGlyph = $11; { Command key}
	kMenuCheckmarkGlyph = $12; { Checkmark key}
	kMenuDiamondGlyph = $13; { Diamond key}
	kMenuAppleLogoFilledGlyph = $14; { Apple logo key (filled)}
	kMenuParagraphKoreanGlyph = $15; { Unassigned (paragraph in Korean)}
	kMenuDeleteLeftGlyph = $17; { Delete to the left key (for left-to-right script systems)}
	kMenuLeftArrowDashedGlyph = $18; { Leftward dashed arrow key}
	kMenuUpArrowDashedGlyph = $19; { Upward dashed arrow key}
	kMenuRightArrowDashedGlyph = $1A; { Rightward dashed arrow key}
	kMenuEscapeGlyph = $1B; { Escape key}
	kMenuClearGlyph = $1C; { Clear key}
	kMenuLeftDoubleQuotesJapaneseGlyph = $1D; { Unassigned (left double quotes in Japanese)}
	kMenuRightDoubleQuotesJapaneseGlyph = $1E; { Unassigned (right double quotes in Japanese)}
	kMenuTrademarkJapaneseGlyph = $1F; { Unassigned (trademark in Japanese)}
	kMenuBlankGlyph = $61; { Blank key}
	kMenuPageUpGlyph = $62; { Page up key}
	kMenuCapsLockGlyph = $63; { Caps lock key}
	kMenuLeftArrowGlyph = $64; { Left arrow key}
	kMenuRightArrowGlyph = $65; { Right arrow key}
	kMenuNorthwestArrowGlyph = $66; { Northwest arrow key}
	kMenuHelpGlyph = $67; { Help key}
	kMenuUpArrowGlyph = $68; { Up arrow key}
	kMenuSoutheastArrowGlyph = $69; { Southeast arrow key}
	kMenuDownArrowGlyph = $6A; { Down arrow key}
	kMenuPageDownGlyph = $6B; { Page down key}
	kMenuAppleLogoOutlineGlyph = $6C; { Apple logo key (outline)}
	kMenuContextualMenuGlyph = $6D; { Contextual menu key}
	kMenuPowerGlyph = $6E; { Power key}
	kMenuF1Glyph = $6F; { F1 key}
	kMenuF2Glyph = $70; { F2 key}
	kMenuF3Glyph = $71; { F3 key}
	kMenuF4Glyph = $72; { F4 key}
	kMenuF5Glyph = $73; { F5 key}
	kMenuF6Glyph = $74; { F6 key}
	kMenuF7Glyph = $75; { F7 key}
	kMenuF8Glyph = $76; { F8 key}
	kMenuF9Glyph = $77; { F9 key}
	kMenuF10Glyph = $78; { F10 key}
	kMenuF11Glyph = $79; { F11 key}
	kMenuF12Glyph = $7A; { F12 key}
	kMenuF13Glyph = $87; { F13 key}
	kMenuF14Glyph = $88; { F14 key}
	kMenuF15Glyph = $89; { F15 key}
	kMenuControlISOGlyph = $8A; { Control key (ISO standard)}
	kMenuEjectGlyph = $8C; { Eject key (available on Mac OS X 10.2 and later)}
	kMenuEisuGlyph = $8D; { Japanese eisu key (available in Mac OS X 10.4 and later)}
	kMenuKanaGlyph = $8E; { Japanese kana key (available in Mac OS X 10.4 and later)}
	kMenuF16Glyph = $8F; { F16 key (available in SnowLeopard and later)}
	kMenuF17Glyph = $90; { F17 key (available in SnowLeopard and later)}
	kMenuF18Glyph = $91; { F18 key (available in SnowLeopard and later)}
	kMenuF19Glyph = $92;  { F19 key (available in SnowLeopard and later)}


{
 *  MenuAttributes
 *  
 *  Summary:
 *    Menu attributes control behavior of the entire menu. They are
 *    used with the Get/ChangeMenuAttributes APIs.
 }
type
	MenuAttributes = UInt32;
const
{
   * No column space is allocated for the mark character when this menu
   * is drawn.
   }
	kMenuAttrExcludesMarkColumn = 1 shl 0;

  {
   * The menu title is automatically disabled when all items are
   * disabled.
   }
	kMenuAttrAutoDisable = 1 shl 2;

  {
   * The pencil glyph from the Keyboard font (kMenuPencilGlyph) is used
   * to draw the Control modifier key in menu keyboard equivalents.
   * This appearance is typically used only by Japanese input method
   * menus.
   }
	kMenuAttrUsePencilGlyph = 1 shl 3;

  {
   * The menu title is not drawn in the menubar, even when the menu is
   * inserted in the menubar. Useful for adding command keys that don't
   * correspond to a visible menu item; menu items with the desired
   * command keys can be added to the menu and inserted in the menubar
   * without making the menu visible. This attribute is available in
   * Mac OS X 10.2 and later.
   }
	kMenuAttrHidden = 1 shl 4;

  {
   * If menu item separators are present at the beginning or end of the
   * menu, or if multiple contiguous separators are present, the extra
   * separator items are marked as hidden to avoid extra blank space in
   * the menu. The menu is examined for extra separators whenever the
   * menu size is recalculated. This attribute is available in Mac OS X
   * 10.3 and later.
   }
	kMenuAttrCondenseSeparators = 1 shl 5;

  {
   * Disables automatic caching of the menu image by the Menu Manager.
   * Automatic caching is provided for all menus that use an HIView to
   * draw their content. Setting this attribute will prevent the Menu
   * Manager from caching the menu image; instead, the menu will be
   * drawn using the standard HIView drawing mechanism each time that
   * it is displayed. This attribute is available in Mac OS X 10.3 and
   * later.
   }
	kMenuAttrDoNotCacheImage = 1 shl 6;

  {
   * Disables substitution of command keys from the
   * NSUserKeyEquivalents dictionary. By default, all menu items are
   * checked for a match in the dictionary. Note that this attribute,
   * to be effective, should be added at the time that the menu is
   * created; once the menu has been searched for user command keys
   * (which occurs in CalcMenuSize, in GetItemCmd and
   * GetMenuItemCommandKey, and before command key matching), the
   * original command keys are replaced by the user command keys and
   * cannot be retrieved. For this reason, it is also not useful to
   * clear this attribute; the original command keys cannot be
   * restored. This attribute is available in Mac OS X 10.3 and later.
   }
	kMenuAttrDoNotUseUserCommandKeys = 1 shl 7;


{
 *  MenuItemAttributes
 *  
 *  Summary:
 *    Menu item attributes control behavior of individual menu items.
 *    They are used with the Get/ChangeMenuItemAttributes APIs.
 }
type
	MenuItemAttributes = UInt32;
const
{
   * This item is disabled.
   }
	kMenuItemAttrDisabled = 1 shl 0;

  {
   * This item's icon is disabled.
   }
	kMenuItemAttrIconDisabled = 1 shl 1;

  {
   * Allows the parent item of a submenu to be selectable.
   }
	kMenuItemAttrSubmenuParentChoosable = 1 shl 2;

  {
   * This item changes dynamically based on modifier key state.
   }
	kMenuItemAttrDynamic = 1 shl 3;

  {
   * This item is not part of the same dynamic group as the previous
   * item.
   }
	kMenuItemAttrNotPreviousAlternate = 1 shl 4;

  {
   * This item is not drawn when the menu is displayed. It is also not
   * included in command key matching, unless the item also has either
   * the Dynamic or IncludeInCmdKeyMatching attributes.
   }
	kMenuItemAttrHidden = 1 shl 5;

  {
   * This item is a separator; the text of the item is ignored.
   }
	kMenuItemAttrSeparator = 1 shl 6;

  {
   * This item is a menu section header; it is disabled and
   * unselectable.
   }
	kMenuItemAttrSectionHeader = 1 shl 7;

  {
   * Metacharacters in the text of this item (such as the dash) are
   * ignored.
   }
	kMenuItemAttrIgnoreMeta = 1 shl 8;

  {
   * This item is recognized by IsMenuKeyEvent when it is passed an
   * auto-repeat keyboard event.
   }
	kMenuItemAttrAutoRepeat = 1 shl 9;

  {
   * When MenuEvent and IsMenuKeyEvent compare this item's keyboard
   * equivalent against a keyboard event, they use the item's virtual
   * keycode equivalent rather than its character code equivalent.
   }
	kMenuItemAttrUseVirtualKey = 1 shl 10;

  {
   * This item is drawn in a customized fashion by the application.
   * Causes custom menu item drawing Carbon events to be sent. This
   * attribute is available in CarbonLib 1.4 and Mac OS X 10.1, and
   * later.
   }
	kMenuItemAttrCustomDraw = 1 shl 11;

  {
   * This item is examined during command key matching by MenuKey,
   * MenuEvent, and IsMenuKeyEvent. Normally, visible menu items are
   * included in command key matching, but hidden menu items are
   * excluded (unless the item also has the Dynamic menu item
   * attribute). The IncludeInCmdKeyMatching attribute can be used to
   * force a hidden, non-dynamic menu item to be included in command
   * key matching when it normally wouldn't. This attribute is
   * available in CarbonLib 1.6 and Mac OS X 10.2 and later.
   }
	kMenuItemAttrIncludeInCmdKeyMatching = 1 shl 12;

  {
   * This item is automatically disabled if, when
   * kEventCommandUpdateStatus is sent for this item, no handler is
   * installed or all handlers return eventNotHandledErr. A return
   * value from any handler of any value other than eventNotHandledErr
   * will prevent automatic disabling. This attribute is useful for
   * applications that use kEventCommandUpdateStatus events for all
   * menu item enabling; previously, an application would need to
   * install UpdateStatus handlers on its application target to disable
   * all items in the application that were unnecessary when no
   * document windows were open. By setting this attribute, all menu
   * items will be disabled automatically unless specifically enabled
   * by an UpdateStatus handler on a window, control, or application
   * target. This attribute is available in Mac OS X 10.3 and later.
   }
	kMenuItemAttrAutoDisable = 1 shl 13;

  {
   * During command key matching, the Menu Manager uses a cache of the
   * available command keys to locate the menu item that matches an
   * event. Before returning this item, the Menu Manager sends a
   * kEventMenuEnableItems event to the menu containing the item, and a
   * kEventCommandUpdateStatus event to each item in the menu, so that
   * the item can be properly enabled or disabled. For some
   * applications, updating the item status for each item in the menu
   * is quite expensive, and also unnecessary since only a single item
   * actually needs to be updated. Setting this attribute indicates to
   * the Menu Manager that it only needs to send a
   * kEventCommandUpdateStatus event to this menu item before returning
   * it from command key matching; kEventMenuEnableItems will not be
   * sent to the menu, and no other menu item will receive
   * kEventCommandUpdateStatus. This attribute is available in Mac OS X
   * 10.3 and later.
   }
	kMenuItemAttrUpdateSingleItem = 1 shl 14;


{
 *  MenuTrackingMode
 *  
 *  Summary:
 *    A menu tracking mode constant is part of the
 *    kEventMenuBeginTracking and kEventMenuChangeTrackingMode Carbon
 *    events. It indicates whether menus are being tracked using the
 *    mouse or keyboard.
 }
type
	MenuTrackingMode = UInt32;
const
{
   * Menus are being tracked using the mouse.
   }
	kMenuTrackingModeMouse = 1;

  {
   * Menus are being tracked using the keyboard.
   }
	kMenuTrackingModeKeyboard = 2;


{
 *  MenuEventOptions
 *  
 *  Summary:
 *    Menu event options control how the menus are searched for an item
 *    matching a particular keyboard event. They are used with the
 *    IsMenuKeyEvent API.
 }
type
	MenuEventOptions = UInt32;
const
{
   * Disabled items are examined for a match.
   }
	kMenuEventIncludeDisabledItems = $0001;

  {
   * Don't hilite the menu title if a match is found.
   }
	kMenuEventQueryOnly = $0002;

  {
   * Don't look for a match in submenus of the starting menu.
   }
	kMenuEventDontCheckSubmenus = $0004;

{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Types                                                                        }
{——————————————————————————————————————————————————————————————————————————————————————}
type
	MenuID = SInt16;
	MenuItemIndex = UInt16;
	MenuItemIndex_fix = MenuItemIndex; { used as field type when a record declaration contains a MenuItemIndex field identifier }
	MenuItemIndexPtr = ^MenuItemIndex;
	MenuCommand = UInt32;
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	MenuRef = ^OpaqueMenuRef; { an opaque type }
	OpaqueMenuRef = record end;
=======
	MenuRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MenuRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MenuRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	MenuRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	MenuRef_fix = MenuRef; { used as field type when a record declaration contains a MenuRef field identifier }
	MenuRefPtr = ^MenuRef;
{ MenuHandle is old name for MenuRef}
type
	MenuHandle = MenuRef;

{
   A MenuBarHandle is a handle to a MenuBarHeader. An instance of this structure is returned
   by the GetMenuBar and GetNewMBar APIs. It is typedef'd to a plain Handle to retain
   source compatibility with previous versions of this header file.
}
type
	MenuBarHandle = Handle;

{
 *  MenuBarHeader
 *  
 *  Summary:
 *    This structure is contained in a MenuBarHandle. It contains a
 *    list of the non-hierarchical menus that have been inserted into
 *    the menubar.
 *  
 *  Discussion:
 *    The MenuBarHandle is a dynamically sized object which cannot be
 *    directly expressed as a C or Pascal structure. First is the
 *    MenuBarHeader structure, followed by a dynamically sized array of
 *    MenuBarMenus, one for each menu. This array is followed by the
 *    HMenuBarHeader, followed by another dynamically sized array of
 *    HMenuBarMenus, one for each hierarchical menu.
 }
type
	MenuBarHeaderPtr = ^MenuBarHeader;
	MenuBarHeader = record
{
   * Offset in bytes from the start of the header to the last menu in
   * the array of MenuBarMenus.
   }
		lastMenu: UInt16;

  {
   * Global coordinate of the right edge of the rightmost menu; unused
   * in a MenuBarHandle returned by GetMenuBar or GetNewMBar.
   }
		lastRight: SInt16;

  {
   * The MBDF resource ID; unused in a MenuBarHandle returned by
   * GetMenuBar or GetNewMBar.
   }
		mbResID: SInt16;
	end;

{
 *  HMenuBarHeader
 *  
 *  Summary:
 *    This structure is contained in a MenuBarHandle. It contains a
 *    list of the hierarchical menus that have been inserted into the
 *    menubar with InsertMenu( menu, -1 ).
 *  
 *  Discussion:
 *    The hierarchical portion of the menubar follows the
 *    non-hierarchical portion in a menubar handle. The hierarchical
 *    portion begins with the HMenuBarHeader structure, followed by a
 *    dynamically sized array of HMenuBarMenus.
 }
type
	HMenuBarHeaderPtr = ^HMenuBarHeader;
	HMenuBarHeader = record
{
   * Offset in bytes from the start of the header to the last menu in
   * the array of HMenuBarMenus.
   }
		lastHMenu: UInt16;

  {
   * Saved bits behind the hilited menu title; unused in a
   * MenuBarHandle returned by GetMenuBar or GetNewMBar.
   }
		menuTitleBits: PixMapHandle;
	end;

{
 *  MenuBarMenu
 *  
 *  Summary:
 *    This structure contains a single menu in the menubar. It is an
 *    element in an array in the MenuBarHeader data strucuture.
 }
type
	MenuBarMenuPtr = ^MenuBarMenu;
	MenuBarMenu = record
{
   * A menu in the menubar.
   }
		menu: MenuRef;

  {
   * The global coordinate of the left edge of the menu title; unused
   * in a MenuBarHandle returned by GetMenuBar or GetNewMBar.
   }
		menuLeft: SInt16;
	end;

{
 *  HMenuBarMenu
 *  
 *  Summary:
 *    This structure contains a single hierarchical menu in the
 *    menubar. It is an element in an array in the HMenuBarHeader data
 *    strucuture.
 }
type
	HMenuBarMenuPtr = ^HMenuBarMenu;
	HMenuBarMenu = record
{
   * An hierarchical menu in the menubar.
   }
		menu: MenuRef;

  {
   * This field is currently unused.
   }
		reserved: SInt16;
	end;
{
 *  MenuTrackingData
 *  
 *  Summary:
 *    The MenuTrackingData structure contains information about a menu
 *    currently being displayed. It is used with the
 *    GetMenuTrackingData API.
 }
type
	MenuTrackingData = record
		menu: MenuRef;
		itemSelected: MenuItemIndex;
		itemUnderMouse: MenuItemIndex;
		itemRect: Rect;
		virtualMenuTop: SInt32;
		virtualMenuBottom: SInt32;
	end;
	MenuTrackingDataPtr = ^MenuTrackingData;

{
 *  Summary:
 *    A MenuItemDataFlags value indicates which fields of a
 *    MenuItemDataRec structure should be used by the
 *    Copy/SetMenuItemData APIs. All MenuItemDataFlags may be used when
 *    getting or setting the contents of a menu item; some may also be
 *    used when getting or setting information about the menu itself,
 *    if the item index given to Copy/SetMenuItemData is 0.
 }
const
{
   * Set or return the Str255 text of a menu using the
   * MenuItemDataRec.text field. If getting the text, the text field
   * must be initialized with a pointer to a Str255 variable before
   * calling CopyMenuItemData. If both kMenuItemDataText and
   * kMenuItemCFString are set on entry to CopyMenuItemData, the API
   * will determine whether the menu text was most recently set using a
   * Str255 or CFString, and return only that text format; the flags
   * value for the other format will be cleared. Valid for both menu
   * items and the menu title (if item number is 0). 
   * 
   * Note that this option is not supported in the 64-bit HIToolbox API.
   }
	kMenuItemDataText = 1 shl 0;

  {
   * Set or return the mark character of a menu item using the
   * MenuItemDataRec.mark field. Valid only for menu items.
   }
	kMenuItemDataMark = 1 shl 1;

  {
   * Set or return the command key of a menu item using the
   * MenuItemDataRec.cmdKey field. Valid only for menu items.
   }
	kMenuItemDataCmdKey = 1 shl 2;

  {
   * Set or return the command key glyph of a menu item using the
   * MenuItemDataRec.cmdKeyGlyph field. Valid only for menu items.
   }
	kMenuItemDataCmdKeyGlyph = 1 shl 3;

  {
   * Set or return the command key modifiers of a menu item using the
   * MenuItemDataRec.cmdKeyModifiers field. Valid only for menu items.
   }
	kMenuItemDataCmdKeyModifiers = 1 shl 4;

  {
   * Set or return the QuickDraw text style of a menu item using the
   * MenuItemDataRec.style field. Valid only for menu items.
   }
	kMenuItemDataStyle = 1 shl 5;

  {
   * Set or return the enable state of a menu using the
   * MenuItemDataRec.enabled field. Valid for both menu items and the
   * menu itself (if item number is 0).
   }
	kMenuItemDataEnabled = 1 shl 6;

  {
   * Set or return the enable state of a menu item icon using the
   * MenuItemDataRec.iconEnabled field. Valid only for menu items.
   }
	kMenuItemDataIconEnabled = 1 shl 7;

  {
   * Set or return the icon resource ID of a menu item using the
   * MenuItemDataRec.iconID field. Valid only for menu items. 
   * 
   * Note that this option is not supported in the 64-bit HIToolbox API.
   }
	kMenuItemDataIconID = 1 shl 8;

  {
   * Set or return the icon handle of a menu item using the
   * MenuItemDataRec.iconType and MenuItemDataRec.iconHandle fields.
   * Both fields must be initialized if setting the icon handle; both
   * fields will be returned when getting the handle. The iconType
   * field should contain one of the constants kMenuIconType,
   * kMenuShrinkIconType, kMenuSmallIconType, kMenuColorIconType,
   * kMenuIconSuiteType, kMenuIconRefType, kMenuCGImageRefType,
   * kMenuSystemIconSelectorType, or kMenuIconResourceType. An icon
   * handle may be a handle to an ICON resource, a SICN resource, a
   * cicn resource, an IconSuite, an IconRef, a CGImageRef, an OSType,
   * or a CFStringRef. Valid only for menu items. When used with
   * CopyMenuItemData, the caller does not need to release the returned
   * icon handle.
   }
	kMenuItemDataIconHandle = 1 shl 9;

  {
   * Set or return the command ID of a menu item using the
   * MenuItemDataRec.cmdID field. Valid only for menu items.
   }
	kMenuItemDataCommandID = 1 shl 10;

  {
   * Set or return the text encoding of a menu item using the
   * MenuItemDataRec.encoding field. Valid only for menu items.
   * 
   * 
   * Note that this option is not supported in the 64-bit HIToolbox API.
   }
	kMenuItemDataTextEncoding = 1 shl 11;

  {
   * Set or return the menu ID of a item's submenu using the
   * MenuItemDataRec.submenuID field. Valid only for menu items.
   }
	kMenuItemDataSubmenuID = 1 shl 12;

  {
   * Set or return the menu handle of an item's submenu using the
   * MenuItemDataRec.submenuHandle field. Valid only for menu items.
   * When used with CopyMenuItemData, the caller does not need to
   * release the returned MenuRef.
   }
	kMenuItemDataSubmenuHandle = 1 shl 13;

  {
   * Set or return the QuickDraw font ID of a menu item using the
   * MenuItemDataRec.fontID field. Valid only for menu items.
   }
	kMenuItemDataFontID = 1 shl 14;

  {
   * Set or return the refcon of a menu using the
   * MenuItemDataRec.refcon field. Valid for both menu items and the
   * menu itself (if item number is 0).
   }
	kMenuItemDataRefcon = 1 shl 15;

  {
   * Set or return the attributes of a menu using the
   * MenuItemDataRec.attr field. Valid for both menu items, in which
   * case the attr field should contain values from the
   * MenuItemAttributes enumeration, and for the menu itself (if item
   * number is 0), in which case the attr field should contain values
   * from the MenuAttributes enumeration.
   }
	kMenuItemDataAttributes = 1 shl 16;

  {
   * Set or return the CFString text of a menu using the
   * MenuItemDataRec.cfText field. If setting the CFString text, the
   * Menu Manager increments the refcount of the text; the caller may
   * safely release the text after setting it. If getting the CFString
   * text, the caller receives a copy of the text owned by the Menu
   * Manager; modifications on the text by the caller will have no
   * effect on the text of the menu, and the caller should release the
   * text when finished with it. If both kMenuItemDataText and
   * kMenuItemCFString are set on entry to CopyMenuItemData, the API
   * will determine whether the menu text was most recently set using a
   * Str255 or CFString, and return only that text format; the flags
   * value for the other format will be cleared. Valid for both menu
   * items and the menu title (if item number is 0). When used with
   * CopyMenuItemData, the caller must release the returned CFStringRef.
   }
	kMenuItemDataCFString = 1 shl 17;

  {
   * Set or return the properties of a menu using the
   * MenuItemDataRec.properties field. If setting the properties, the
   * properties field should contain a collection with the new
   * properties; existing menu properties with the same collection
   * creator and tag will be replaced by the new properties. If getting
   * the properties, the properties field should either be set to NULL
   * or to a valid Collection. If NULL, a new collection is allocated
   * by the CopyMenuItemData and returned in the properties field. If
   * not NULL, the entire contents of the collection are replaced by
   * the properties of the menu. Valid for both menu items and the menu
   * itself (if item number is 0).
   }
	kMenuItemDataProperties = 1 shl 18;

  {
   * Set or return the item indent level of a menu item using the
   * MenuItemDataRec.indent field. Valid only for menu items.
   }
	kMenuItemDataIndent = 1 shl 19;

  {
   * Set or return the virtual key code keyboard equivalent of a menu
   * item using the MenuItemDataRec.cmdVirtualKey field. Valid only for
   * menu items. On output, only valid if the item has the
   * kMenuItemAttrUseVirtualKeyCode attribute. Available in Mac OS X
   * and CarbonLib 1.1 and later.
   }
	kMenuItemDataCmdVirtualKey = 1 shl 20;

  {
   * Set or return the attributed text of a menu item using the
   * MenuItemDataRec.attributedText field. If setting the attributed
   * text, the Menu Manager increments the refcount of the text; the
   * caller may safely release the text after setting it. If getting
   * the attributed text, the caller receives a copy of the text owned
   * by the Menu Manager; modifications on the text by the caller will
   * have no effect on the text of the menu, and the caller should
   * release the text when finished with it. Valid only for menu items.
   * Available in Mac OS X 10.5 and later.
   }
	kMenuItemDataAttributedText = 1 shl 21;

  {
   * Set or return the CTFontRef of a menu using the
   * MenuItemDataRec.font field. If setting the font, the Menu Manager
   * increments the refcount of the font; the caller may safely release
   * the font after setting it. If getting the font, the caller
   * receives a retained instance of the font owned by the Menu
   * Manager; the caller should release the font when finished with it.
   * Valid for both menu items and the menu title (if item number is
   * 0). Available in Mac OS X 10.5 and later.
   }
	kMenuItemDataFont = 1 shl 22;
	kMenuItemDataAllDataVersionOne = $000FFFFF;
	kMenuItemDataAllDataVersionTwo = kMenuItemDataAllDataVersionOne or kMenuItemDataCmdVirtualKey;
	kMenuItemDataAllDataVersionThree = kMenuItemDataAllDataVersionTwo or kMenuItemDataAttributedText or kMenuItemDataFont;

type
	MenuItemDataFlags = UInt64;

{
 *  MenuItemDataRec
 *  
 *  Summary:
 *    The MenuItemDataRec structure is used to get and change aspects
 *    of a menu item. It is used with the Copy/SetMenuItemData APIs.
 *  
 *  Discussion:
 *    When using this structure with Copy/SetMenuItemData, the caller
 *    must first set the whichData field to a combination of
 *    MenuItemDataFlags indicating which specific data should be
 *    retrieved or set. Some fields also require initialization before
 *    calling CopyMenuItemData; see the individual MenuItemDataFlags
 *    documentation for details. When used with CopyMenuItemData, the
 *    whichData field will be updated on exit to indicate which data
 *    fields have actually been returned. If the caller requests data
 *    that does not exist for that menu or menu item, the corresponding
 *    bit in the whichData field will be cleared, and the field in the
 *    MenuItemDataRec for that data will be set to zero.
 }
type
	MenuItemDataRecPtr = ^MenuItemDataRec;
	MenuItemDataRec = record
		whichData: MenuItemDataFlags;
		text: StringPtr;
		mark: UniChar;
		cmdKey: UniChar;
		cmdKeyGlyph: UInt32;
		cmdKeyModifiers: UInt32;
		style: Style_fix;
		enabled: Boolean;
		iconEnabled: Boolean;
		filler1: UInt8;
		iconID: SInt32;
		iconType: UInt32;
		iconHandle: Handle;
		cmdID: MenuCommand;
		encoding: TextEncoding;
		submenuID: MenuID;
		submenuHandle: MenuRef;
		fontID: SInt32;
		refcon: URefCon;
		attr: OptionBits;
		cfText: CFStringRef;
		properties: Collection;
		indent: UInt32;
		cmdVirtualKey: UInt16;
		attributedText: CFAttributedStringRef;
		font: CTFontRef;
	end;
type
	MenuItemDataPtr = MenuItemDataRecPtr;
	MenuItemID = UInt32;

{
 *  MenuDefType
 *  
 *  Summary:
 *    Types of custom menu definitions.
 }
const
{
   * A custom menu definition using a function pointer based on the
   * pre-Carbon MDEF model. This type of custom menu definition is not
   * supported by 64-bit HIToolbox.
   }
	kMenuDefProcPtr = 0;

  {
   * A custom menu definition using an HIView subclass. Available in
   * Mac OS X 10.3 and later.
   }
	kMenuDefClassID = 1;

type
	MenuDefType = UInt32;
{$ifc TARGET_CPU_64}
type
	MenuDefUPP = UnivPtr;
{$elsec} {TARGET_CPU_64}

{
 *  MenuDefProcPtr
 *  
 *  Summary:
 *    Callback function for a custom menu definition.
 }
type
	MenuDefProcPtr = procedure( message: SInt16; theMenu: MenuRef; var menuRect: Rect; hitPt: Point; var whichItem: SInt16 );
	MenuDefUPP = MenuDefProcPtr;
{
 *  NewMenuDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewMenuDefUPP( userRoutine: MenuDefProcPtr ): MenuDefUPP; external name '_NewMenuDefUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeMenuDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeMenuDefUPP( userUPP: MenuDefUPP ); external name '_DisposeMenuDefUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  InvokeMenuDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeMenuDefUPP( message: SInt16; theMenu: MenuRef; var menuRect: Rect; hitPt: Point; var whichItem: SInt16; userUPP: MenuDefUPP ); external name '_InvokeMenuDefUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{$endc} {TARGET_CPU_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{
 *  MenuDefSpec
 *  
 *  Summary:
 *    Specifies a custom menu definition.
 }
type
	MenuDefSpec = record
{
   * The type of menu definition: either kMenuDefProcPtr or
   * kMenuDefClassID. kMenuDefClassID may only be used in Mac OS X 10.3
   * and later. On 64-bit HIToolbox, kMenuDefProcPtr is not supported.
   }
		defType: MenuDefType;
		case SInt16 of
		0: (
			defProc: MenuDefUPP;
			);
		1: (
			classID: CFStringRef;
			initEvent: EventRef;
			);
	end;
	MenuDefSpecPtr = ^MenuDefSpec;
{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Manipulation                                                                 }
{——————————————————————————————————————————————————————————————————————————————————————}
{$ifc not TARGET_CPU_64}
{
 *  [Mac]GetMenu()
 *  
 *  Summary:
 *    Creates a menu using the contents of a 'MENU' resource.
<<<<<<< HEAD
 *  
 *  Discussion:
 *    Apple recommends that you use Interface Builder to design your
 *    menus and other user interface components. Interface Builder
 *    stores its output in nib files rather then Resource Manager
 *    resources. Use the CreateMenuFromNib API to load a menu from a
 *    nib file.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    resourceID:
 *      The resource ID of the 'MENU' resource.
 *  
 *  Result:
 *    The newly created MenuRef. In CarbonLib and Mac OS X, a new menu
 *    will be created by each call to the GetMenu API; this is a change
 *    from the classic Mac OS API, in which GetMenu would return an
 *    existing menu if the menu had previously been created from the
 *    same resource.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
{$ifc TARGET_OS_MAC}
function MacGetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
{$endc} {TARGET_OS_MAC}
function GetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DisposeMenu( theMenu: MenuRef ); external name '_DisposeMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuID()
=======
 *  
 *  Discussion:
 *    Apple recommends that you use Interface Builder to design your
 *    menus and other user interface components. Interface Builder
 *    stores its output in nib files rather then Resource Manager
 *    resources. Use the CreateMenuFromNib API to load a menu from a
 *    nib file.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuID( menu: MenuRef ): MenuID; external name '_GetMenuID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuWidth()
=======
 *  Parameters:
 *    
 *    resourceID:
 *      The resource ID of the 'MENU' resource.
>>>>>>> graemeg/cpstrnew
 *  
 *  Result:
 *    The newly created MenuRef. In CarbonLib and Mac OS X, a new menu
 *    will be created by each call to the GetMenu API; this is a change
 *    from the classic Mac OS API, in which GetMenu would return an
 *    existing menu if the menu had previously been created from the
 *    same resource.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
<<<<<<< HEAD
function GetMenuWidth( menu: MenuRef ): SInt16; external name '_GetMenuWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuHeight( menu: MenuRef ): SInt16; external name '_GetMenuHeight';
=======
{$ifc TARGET_OS_MAC}
function MacGetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
{$endc} {TARGET_OS_MAC}
function GetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuDefinition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuDefinition( menu: MenuRef; outDefSpec: MenuDefSpecPtr ): OSStatus; external name '_GetMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SetMenuID()
=======
 *  GetMenuID()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
<<<<<<< HEAD
procedure SetMenuID( menu: MenuRef; menuID_: MenuID ); external name '_SetMenuID';
=======
function GetMenuID( menu: MenuRef ): MenuID; external name '_GetMenuID';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
>>>>>>> graemeg/cpstrnew


{
<<<<<<< HEAD
 *  SetMenuWidth()
=======
 *  GetMenuWidth()
>>>>>>> graemeg/cpstrnew
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
<<<<<<< HEAD
 }
procedure SetMenuWidth( menu: MenuRef; width: SInt16 ); external name '_SetMenuWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuHeight( menu: MenuRef; height: SInt16 ); external name '_SetMenuHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{
 *  SetMenuDefinition()
=======
 *  Summary:
 *    Specifies a custom menu definition.
 }
type
	MenuDefSpec = record
{
   * The type of menu definition: either kMenuDefProcPtr or
   * kMenuDefClassID. kMenuDefClassID may only be used in Mac OS X 10.3
   * and later. On 64-bit HIToolbox, kMenuDefProcPtr is not supported.
   }
		defType: MenuDefType;
		case SInt16 of
		0: (
			defProc: MenuDefUPP;
			);
		1: (
			classID: CFStringRef;
			initEvent: EventRef;
			);
	end;
	MenuDefSpecPtr = ^MenuDefSpec;
{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Manipulation                                                                 }
{——————————————————————————————————————————————————————————————————————————————————————}
{$ifc not TARGET_CPU_64}
{
 *  [Mac]GetMenu()
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Creates a menu using the contents of a 'MENU' resource.
 *  
 *  Discussion:
 *    Apple recommends that you use Interface Builder to design your
 *    menus and other user interface components. Interface Builder
 *    stores its output in nib files rather then Resource Manager
 *    resources. Use the CreateMenuFromNib API to load a menu from a
 *    nib file.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuDefinition( menu: MenuRef; const (*var*) defSpec: MenuDefSpec ): OSStatus; external name '_SetMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CalcMenuSize()
=======
=======
>>>>>>> origin/cpstrnew
 *  Parameters:
 *    
 *    resourceID:
 *      The resource ID of the 'MENU' resource.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  
 *  Result:
 *    The newly created MenuRef. In CarbonLib and Mac OS X, a new menu
 *    will be created by each call to the GetMenu API; this is a change
 *    from the classic Mac OS API, in which GetMenu would return an
 *    existing menu if the menu had previously been created from the
 *    same resource.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure CalcMenuSize( theMenu: MenuRef ); external name '_CalcMenuSize';
=======
 }
function GetMenuWidth( menu: MenuRef ): SInt16; external name '_GetMenuWidth';
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}
function MacGetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
{$endc} {TARGET_OS_MAC}
function GetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
{$endc} {TARGET_OS_MAC}
function GetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuHeight()
=======
 *  
 *  Summary:
 *    Creates a menu using the contents of a 'MENU' resource.
 *  
 *  Discussion:
 *    Apple recommends that you use Interface Builder to design your
 *    menus and other user interface components. Interface Builder
 *    stores its output in nib files rather then Resource Manager
 *    resources. Use the CreateMenuFromNib API to load a menu from a
 *    nib file.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
=======
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
>>>>>>> graemeg/cpstrnew
 }
function GetMenuHeight( menu: MenuRef ): SInt16; external name '_GetMenuHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  GetMenuDefinition()
=======
 *  GetMenuID()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID and size used to draw the items
 *    in a menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuFont.
=======
 *  GetMenuID()
>>>>>>> graemeg/cpstrnew
=======
 *  Parameters:
 *    
 *    resourceID:
 *      The resource ID of the 'MENU' resource.
>>>>>>> graemeg/cpstrnew
 *  
 *  Result:
 *    The newly created MenuRef. In CarbonLib and Mac OS X, a new menu
 *    will be created by each call to the GetMenu API; this is a change
 *    from the classic Mac OS API, in which GetMenu would return an
 *    existing menu if the menu had previously been created from the
 *    same resource.
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu's default font, or zero if the font
 *      has not been customized.
 *    
 *    outFontSize:
 *      On exit, contains the menu's default font size, or zero if the
 *      font size has not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetMenuDefinition( menu: MenuRef; outDefSpec: MenuDefSpecPtr ): OSStatus; external name '_GetMenuDefinition';
=======
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuID( menu: MenuRef ): MenuID; external name '_GetMenuID';
>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}
function MacGetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
{$endc} {TARGET_OS_MAC}
function GetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
=======
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuID( menu: MenuRef ): MenuID; external name '_GetMenuID';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  SetMenuID()
 *  
 *  Summary:
 *    Sets the QuickDraw font ID and size used to draw the items in a
 *    menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to set.
 *    
 *    inFontID:
 *      The menu's default font, or zero to use the standard font for
 *      menu items.
 *    
 *    inFontSize:
 *      The menu's default font size, or zero to use the standard font
 *      size for menu items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuID( menu: MenuRef; menuID_: MenuID ); external name '_SetMenuID';
=======
 *  GetMenuWidth()
=======
 *  GetMenuWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuWidth( menu: MenuRef ): SInt16; external name '_GetMenuWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuHeight()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
<<<<<<< HEAD
function GetMenuWidth( menu: MenuRef ): SInt16; external name '_GetMenuWidth';
>>>>>>> graemeg/cpstrnew
=======
function GetMenuHeight( menu: MenuRef ): SInt16; external name '_GetMenuHeight';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  SetMenuWidth()
=======
 *  GetMenuHeight()
>>>>>>> graemeg/cpstrnew
=======
 *  GetMenuID()
>>>>>>> graemeg/cpstrnew
=======
 *  GetMenuDefinition()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure SetMenuWidth( menu: MenuRef; width: SInt16 ); external name '_SetMenuWidth';
=======
function GetMenuHeight( menu: MenuRef ): SInt16; external name '_GetMenuHeight';
>>>>>>> graemeg/cpstrnew
=======
function GetMenuID( menu: MenuRef ): MenuID; external name '_GetMenuID';
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   not available
 }
function GetMenuDefinition( menu: MenuRef; outDefSpec: MenuDefSpecPtr ): OSStatus; external name '_GetMenuDefinition';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  SetMenuHeight()
=======
 *  GetMenuDefinition()
>>>>>>> graemeg/cpstrnew
=======
 *  GetMenuWidth()
>>>>>>> graemeg/cpstrnew
=======
 *  SetMenuID()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuID( menu: MenuRef; menuID_: MenuID ); external name '_SetMenuID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuWidth( menu: MenuRef; width: SInt16 ); external name '_SetMenuWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuHeight( menu: MenuRef; height: SInt16 ); external name '_SetMenuHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuDefinition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuDefinition( menu: MenuRef; const (*var*) defSpec: MenuDefSpec ): OSStatus; external name '_SetMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CalcMenuSize()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure CalcMenuSize( theMenu: MenuRef ); external name '_CalcMenuSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CountMenuItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
function CountMenuItems( theMenu: MenuRef ): UInt16; external name '_CountMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Routines available in Mac OS 8.5 and later, and on Mac OS 8.1 and later using CarbonLib 1.1 and later}

{
 *  GetMenuFont()
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID and size used to draw the items
 *    in a menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu's default font, or zero if the font
 *      has not been customized.
 *    
 *    outFontSize:
 *      On exit, contains the menu's default font size, or zero if the
 *      font size has not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuFont( menu: MenuRef; var outFontID: SInt16; var outFontSize: UInt16 ): OSStatus; external name '_GetMenuFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuFont()
 *  
 *  Summary:
 *    Sets the QuickDraw font ID and size used to draw the items in a
 *    menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to set.
 *    
 *    inFontID:
 *      The menu's default font, or zero to use the standard font for
 *      menu items.
 *    
 *    inFontSize:
 *      The menu's default font size, or zero to use the standard font
 *      size for menu items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuHeight( menu: MenuRef; height: SInt16 ); external name '_SetMenuHeight';
=======
 *    Non-Carbon CFM:   not available
 }
function GetMenuDefinition( menu: MenuRef; outDefSpec: MenuDefSpecPtr ): OSStatus; external name '_GetMenuDefinition';
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuWidth( menu: MenuRef ): SInt16; external name '_GetMenuWidth';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  SetMenuDefinition()
=======
 *  SetMenuID()
>>>>>>> graemeg/cpstrnew
=======
 *  GetMenuHeight()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Returns whether the layout of a menu includes or excludes space
 *    for menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the GetMenuAttributes API and check
 *    for the kMenuAttrExcludesMarkColumn attribute instead of using
 *    this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to examine.
 *  
 *  Result:
 *    True if the menu layout does not include space for menu item
 *    marks, or false if it does include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuID( menu: MenuRef; menuID_: MenuID ); external name '_SetMenuID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuWidth()
 *  
 *  Summary:
 *    Sets whether the layout of a menu includes or excludes space for
 *    menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the ChangeMenuAttributes API with
 *    the kMenuAttrExcludesMarkColumn attribute instead of using this
 *    API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to modify.
 *    
 *    excludesMark:
 *      True if the menu layout should not include space for menu item
 *      marks, or false if it should include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuWidth( menu: MenuRef; width: SInt16 ); external name '_SetMenuWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuHeight()
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuHeight( menu: MenuRef ): SInt16; external name '_GetMenuHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuDefinition()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuHeight( menu: MenuRef; height: SInt16 ); external name '_SetMenuHeight';
=======
 *    Non-Carbon CFM:   not available
 }
function GetMenuDefinition( menu: MenuRef; outDefSpec: MenuDefSpecPtr ): OSStatus; external name '_GetMenuDefinition';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SetMenuDefinition()
=======
 *  SetMenuID()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuID( menu: MenuRef; menuID_: MenuID ); external name '_SetMenuID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuWidth( menu: MenuRef; width: SInt16 ); external name '_SetMenuWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuHeight( menu: MenuRef; height: SInt16 ); external name '_SetMenuHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuDefinition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuDefinition( menu: MenuRef; const (*var*) defSpec: MenuDefSpec ): OSStatus; external name '_SetMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CalcMenuSize()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure CalcMenuSize( theMenu: MenuRef ); external name '_CalcMenuSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CountMenuItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
function CountMenuItems( theMenu: MenuRef ): UInt16; external name '_CountMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Routines available in Mac OS 8.5 and later, and on Mac OS 8.1 and later using CarbonLib 1.1 and later}

{
<<<<<<< HEAD
 *  GetMenuFont()
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID and size used to draw the items
 *    in a menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu's default font, or zero if the font
 *      has not been customized.
 *    
 *    outFontSize:
 *      On exit, contains the menu's default font size, or zero if the
 *      font size has not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuFont( menu: MenuRef; var outFontID: SInt16; var outFontSize: UInt16 ): OSStatus; external name '_GetMenuFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuFont()
 *  
 *  Summary:
 *    Sets the QuickDraw font ID and size used to draw the items in a
 *    menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to set.
 *    
 *    inFontID:
 *      The menu's default font, or zero to use the standard font for
 *      menu items.
 *    
 *    inFontSize:
 *      The menu's default font size, or zero to use the standard font
 *      size for menu items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuDefinition( menu: MenuRef; const (*var*) defSpec: MenuDefSpec ): OSStatus; external name '_SetMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CalcMenuSize()
 *  
 *  Summary:
 *    Returns whether the layout of a menu includes or excludes space
 *    for menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the GetMenuAttributes API and check
 *    for the kMenuAttrExcludesMarkColumn attribute instead of using
 *    this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to examine.
 *  
 *  Result:
 *    True if the menu layout does not include space for menu item
 *    marks, or false if it does include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure CalcMenuSize( theMenu: MenuRef ); external name '_CalcMenuSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CountMenuItems()
=======
 *  DuplicateMenu()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Sets whether the layout of a menu includes or excludes space for
 *    menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the ChangeMenuAttributes API with
 *    the kMenuAttrExcludesMarkColumn attribute instead of using this
 *    API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to modify.
 *    
 *    excludesMark:
 *      True if the menu layout should not include space for menu item
 *      marks, or false if it should include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
function CountMenuItems( theMenu: MenuRef ): UInt16; external name '_CountMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Routines available in Mac OS 8.5 and later, and on Mac OS 8.1 and later using CarbonLib 1.1 and later}

{
 *  GetMenuFont()
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID and size used to draw the items
 *    in a menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu's default font, or zero if the font
 *      has not been customized.
 *    
 *    outFontSize:
 *      On exit, contains the menu's default font size, or zero if the
 *      font size has not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
function GetMenuFont( menu: MenuRef; var outFontID: SInt16; var outFontSize: UInt16 ): OSStatus; external name '_GetMenuFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuFont()
 *  
 *  Summary:
 *    Sets the QuickDraw font ID and size used to draw the items in a
 *    menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to set.
 *    
 *    inFontID:
 *      The menu's default font, or zero to use the standard font for
 *      menu items.
 *    
 *    inFontSize:
 *      The menu's default font size, or zero to use the standard font
 *      size for menu items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function SetMenuDefinition( menu: MenuRef; const (*var*) defSpec: MenuDefSpec ): OSStatus; external name '_SetMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CalcMenuSize()
 *  
 *  Summary:
 *    Returns whether the layout of a menu includes or excludes space
 *    for menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the GetMenuAttributes API and check
 *    for the kMenuAttrExcludesMarkColumn attribute instead of using
 *    this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to examine.
 *  
 *  Result:
 *    True if the menu layout does not include space for menu item
 *    marks, or false if it does include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
procedure CalcMenuSize( theMenu: MenuRef ); external name '_CalcMenuSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CountMenuItems()
 *  
 *  Summary:
 *    Sets whether the layout of a menu includes or excludes space for
 *    menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the ChangeMenuAttributes API with
 *    the kMenuAttrExcludesMarkColumn attribute instead of using this
 *    API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to modify.
 *    
 *    excludesMark:
 *      True if the menu layout should not include space for menu item
 *      marks, or false if it should include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
function CountMenuItems( theMenu: MenuRef ): UInt16; external name '_CountMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Routines available in Mac OS 8.5 and later, and on Mac OS 8.1 and later using CarbonLib 1.1 and later}

{
 *  GetMenuFont()
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID and size used to draw the items
 *    in a menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu's default font, or zero if the font
 *      has not been customized.
 *    
 *    outFontSize:
 *      On exit, contains the menu's default font size, or zero if the
 *      font size has not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
function GetMenuFont( menu: MenuRef; var outFontID: SInt16; var outFontSize: UInt16 ): OSStatus; external name '_GetMenuFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuFont()
 *  
 *  Summary:
 *    Sets the QuickDraw font ID and size used to draw the items in a
 *    menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to set.
 *    
 *    inFontID:
 *      The menu's default font, or zero to use the standard font for
 *      menu items.
 *    
 *    inFontSize:
 *      The menu's default font size, or zero to use the standard font
 *      size for menu items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
function SetMenuFont( menu: MenuRef; inFontID: SInt16; inFontSize: UInt16 ): OSStatus; external name '_SetMenuFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  GetMenuExcludesMarkColumn()
 *  
 *  Summary:
 *    Returns whether the layout of a menu includes or excludes space
 *    for menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the GetMenuAttributes API and check
 *    for the kMenuAttrExcludesMarkColumn attribute instead of using
 *    this API.
=======
 *  HIMenuGetContentView()
 *  
 *  Summary:
 *    Returns the HIViewRef that will be used to draw menu content for
 *    this menu, if any.
 *  
 *  Discussion:
 *    If the content view has not yet been created, the Menu Manager
 *    will create the content view using the view class ID and
 *    initialization event associated with the menu. Note that the menu
 *    content view is not the same as the window content view; the menu
 *    content view is embedded inside the window content view. If the
 *    menu uses an MDEF instead of an HIView to draw its content, noErr
 *    is returned but the output HIViewRef is set to NULL.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    menu:
 *      The menu to examine.
 *  
 *  Result:
 *    True if the menu layout does not include space for menu item
 *    marks, or false if it does include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuExcludesMarkColumn( menu: MenuRef ): Boolean; external name '_GetMenuExcludesMarkColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuExcludesMarkColumn()
 *  
 *  Summary:
 *    Sets whether the layout of a menu includes or excludes space for
 *    menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the ChangeMenuAttributes API with
 *    the kMenuAttrExcludesMarkColumn attribute instead of using this
 *    API.
=======
 *    inMenu:
 *      The menu.
 *    
 *    inMenuType:
 *      The type of menu for which the menu content view should be
 *      returned. The same MenuRef may have multiple content views,
 *      depending on the menu type being displayed.
 *    
 *    outView:
 *      On exit, contains the view. May be set to NULL if the menu does
 *      not use an HIView to draw its content. The caller should not
 *      release this view.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIMenuGetContentView( inMenu: MenuRef; inMenuType: ThemeMenuType; var outView: HIViewRef ): OSStatus; external name '_HIMenuGetContentView';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIMenuSetFont()
 *  
 *  Summary:
 *    Sets the font to be used to draw a menu or a menu item.
 *  
 *  Discussion:
 *    Pass zero for the inItem parameter to set the default font for
 *    all of the items in a menu. Pass a non-zero value for the inItem
 *    parameter to set a font for a specific item; this overrides the
 *    default font for the menu. If a font is set for an entire menu,
 *    it will also apply to all submenus of the menu. Setting a font
 *    for a specific item does not cause that font to apply to the
 *    item's submenu.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    menu:
 *      The menu to modify.
 *    
 *    excludesMark:
 *      True if the menu layout should not include space for menu item
 *      marks, or false if it should include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
=======
 *    inMenu:
 *      The menu whose font to set.
 *    
 *    inItem:
 *      The item whose font to set, or zero to customze the font for
 *      the entire menu.
 *    
 *    inFont:
 *      The font to use for the specified menu or item. May be NULL to
 *      remove the menu or item's current custom font. The font will be
 *      retained by the Menu Manager and released when the menu or item
 *      is destroyed, or when the font for the menu or item is changed
 *      to a different value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
>>>>>>> graemeg/cpstrnew
 }
<<<<<<< HEAD
function SetMenuExcludesMarkColumn( menu: MenuRef; excludesMark: Boolean ): OSStatus; external name '_SetMenuExcludesMarkColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  RegisterMenuDefinition()
 *  
 *  Summary:
 *    Registers or unregisters a binding between a resource ID and a
 *    menu definition function.
 *  
 *  Discussion:
 *    In the classic Mac OS Menu Manager, a 'MENU' resource can contain
 *    an embedded MDEF procID that is used by the Menu Manager as the
 *    resource ID of an 'MDEF' resource to measure and draw the menu.
 *    The 'MDEF' resource is loaded by the Menu Manager when you load
 *    the menu with GetMenu. Since MDEFs can no longer be packaged as
 *    code resources on Carbon, the procID can no longer refer directly
 *    to an MDEF resource. However, using RegisterMenuDefinition you
 *    can instead specify a UniversalProcPtr pointing to code in your
 *    application code fragment.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inResID:
 *      An MDEF proc ID, as used in a 'MENU' resource.
 *    
 *    inDefSpec:
 *      Specifies the MenuDefUPP that should be used for menus with the
 *      given MDEF proc ID. Passing NULL allows you to unregister the
 *      menu definition that had been associated with the given MDEF
 *      proc ID.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function RegisterMenuDefinition( inResID: SInt16; inDefSpec: MenuDefSpecPtr ): OSStatus; external name '_RegisterMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CreateNewMenu()
 *  
 *  Summary:
 *    Creates a new, untitled, empty menu.
 *  
 *  Discussion:
 *    CreateNewMenu is preferred over NewMenu because it allows you to
 *    specify the menu's attributes and it does not require you to
 *    specify a Str255-based menu title. To set the menu title, you can
 *    use either SetMenuTitle or SetMenuTitleWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenuID:
 *      The menu ID to use for the new menu. Zero is a valid menu ID in
 *      Carbon.
 *    
 *    inMenuAttributes:
 *      The menu attributes to use for the new menu.
 *    
 *    outMenuRef:
 *      On exit, contains the new menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateNewMenu( inMenuID: MenuID; inMenuAttributes: MenuAttributes; var outMenuRef: MenuRef ): OSStatus; external name '_CreateNewMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CreateCustomMenu()
 *  
 *  Summary:
 *    Creates a new, untitled, empty menu using a custom menu
 *    definition function.
 *  
 *  Discussion:
 *    Similar to CreateNewMenu, but also allows you to specify a custom
 *    menu definition function.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDefSpec:
 *      Specifies a custom menu definition function. defSpec->defType
 *      must be kMenuDefProcPtr or, on Mac OS X 10.3 and later,
 *      kMenuDefClassID.
 *    
 *    inMenuID:
 *      The menu ID to use for the new menu.
 *    
 *    inMenuAttributes:
 *      The menu attributes to use for the new menu.
 *    
 *    outMenuRef:
 *      On exit, contains the new menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateCustomMenu( const (*var*) inDefSpec: MenuDefSpec; inMenuID: MenuID; inMenuAttributes: MenuAttributes; var outMenuRef: MenuRef ): OSStatus; external name '_CreateCustomMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsValidMenu()
 *  
 *  Summary:
 *    Determines if a menu is valid.
=======
function HIMenuSetFont( inMenu: MenuRef; inItem: MenuItemIndex; inFont: CTFontRef { can be NULL } ): OSStatus; external name '_HIMenuSetFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyFont()
 *  
 *  Summary:
 *    Retrieves the custom font that is used to draw a menu or a menu
 *    item.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
<<<<<<< HEAD
 *      The menu to check for validity.
=======
 *      The menu whose font to retrieve.
 *    
 *    inItem:
 *      The item whose font to retrieve, or zero to retrieve the font
 *      for the entire menu.
 *    
 *    outFont:
 *      On exit, contains the font used for the specified menu or item.
 *      If the menu or item does not have a custom font, the resulting
 *      CTFontRef is set to NULL. The caller must release the
 *      CTFontRef, if it is non-NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyFont( inMenu: MenuRef; inItem: MenuItemIndex; var outFont: CTFontRef ): OSStatus; external name '_HIMenuCopyFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Insertion                                                               }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  DeleteMenuItem()
>>>>>>> origin/cpstrnew
 *  
 *  Result:
 *    Indicates whether the menu is valid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
function IsValidMenu( inMenu: MenuRef ): Boolean; external name '_IsValidMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
 *  DuplicateMenu()
 *  
 *  Summary:
 *    Creates a new menu that is a copy of another menu.
 *  
 *  Discussion:
 *    Unlike RetainMenu, DuplicateMenu creates an entirely new menu
 *    that is an exact copy of the original menu. The MDEF for the new
 *    menu will receive an init message after the menu has been fully
 *    created.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSourceMenu:
 *      The menu to duplicate.
 *    
 *    outMenu:
 *      On exit, a copy of the source menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function DuplicateMenu( inSourceMenu: MenuRef; var outMenu: MenuRef ): OSStatus; external name '_DuplicateMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuTitleAsCFString()
 *  
 *  Summary:
 *    Returns a CFString containing the title of a menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose title to return.
 *    
 *    outString:
 *      On exit, a CFString containing the menu's title. This string
 *      must be released by the caller.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuTitleAsCFString( inMenu: MenuRef; var outString: CFStringRef ): OSStatus; external name '_CopyMenuTitleAsCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuTitleWithCFString()
 *  
 *  Summary:
 *    Sets the title of a menu to the text contained in a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    SetMenuTitleWithCFString, depending on whether the string is
 *    mutable or immutable. If the string is mutable, modifying the
 *    string after calling SetMenuTitleWithCFString will have no effect
 *    on the menu's actual title. The caller may release the string
 *    after calling SetMenuTitleWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose title to set.
 *    
 *    inString:
 *      The string containing the new menu title text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuTitleWithCFString( inMenu: MenuRef; inString: CFStringRef ): OSStatus; external name '_SetMenuTitleWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuTitleIcon()
=======
 *  CopyMenuItems()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
=======
 *  DuplicateMenu()
 *  
 *  Summary:
 *    Creates a new menu that is a copy of another menu.
 *  
 *  Discussion:
 *    Unlike RetainMenu, DuplicateMenu creates an entirely new menu
 *    that is an exact copy of the original menu. The MDEF for the new
 *    menu will receive an init message after the menu has been fully
 *    created.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSourceMenu:
 *      The menu to duplicate.
 *    
 *    outMenu:
 *      On exit, a copy of the source menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function DuplicateMenu( inSourceMenu: MenuRef; var outMenu: MenuRef ): OSStatus; external name '_DuplicateMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuTitleAsCFString()
 *  
 *  Summary:
 *    Returns a CFString containing the title of a menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose title to return.
 *    
 *    outString:
 *      On exit, a CFString containing the menu's title. This string
 *      must be released by the caller.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuTitleAsCFString( inMenu: MenuRef; var outString: CFStringRef ): OSStatus; external name '_CopyMenuTitleAsCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuTitleWithCFString()
 *  
 *  Summary:
 *    Sets the title of a menu to the text contained in a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    SetMenuTitleWithCFString, depending on whether the string is
 *    mutable or immutable. If the string is mutable, modifying the
 *    string after calling SetMenuTitleWithCFString will have no effect
 *    on the menu's actual title. The caller may release the string
 *    after calling SetMenuTitleWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose title to set.
 *    
 *    inString:
 *      The string containing the new menu title text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuTitleWithCFString( inMenu: MenuRef; inString: CFStringRef ): OSStatus; external name '_SetMenuTitleWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuTitleIcon()
 *  
 *  Summary:
>>>>>>> graemeg/cpstrnew
 *    Sets the title of a menu to be an icon.
 *  
 *  Discussion:
 *    The Menu Manager takes ownership of the supplied icon after this
 *    call. When a menu with an title icon is disposed, the Menu
 *    Manager will dispose the icon also; the Menu Manager will also
 *    dispose of the current title icon when a new text or icon title
 *    is supplied for a menu. If an IconRef or CGImageRef is specified,
 *    the Menu Manager will increment its refcount, so you may freely
 *    release your reference to the icon or image without invalidating
 *    the Menu Manager's copy. The menubar will be invalidated by this
 *    call, and redrawn at the next opportunity.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose title to set.
 *    
 *    inType:
 *      The type of icon being used to specify the icon title; use
 *      kMenuNoIcon to remove the icon from the menu title. In Mac OS X
 *      10.2 and earlier, the supported types are kMenuIconSuiteType
 *      and kMenuIconRefType; Mac OS X 10.3 also supports
 *      kMenuCGImageRefType.
 *    
 *    inIcon:
 *      The icon; must be NULL if inType is kMenuNoIcon. The supported
 *      icon formats are IconSuiteRef, IconRef, and in Mac OS X 10.3
 *      and later, CGImageRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuTitleIcon( inMenu: MenuRef; inType: UInt32; inIcon: UnivPtr ): OSStatus; external name '_SetMenuTitleIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  GetMenuTitleIcon()
 *  
 *  Summary:
 *    Retrieves the icon, if any, being used as the title of a menu.
 *  
 *  Discussion:
 *    This API does not increment a refcount on the returned icon. The
 *    caller should not release the icon.
=======
 *  HIMenuSetSelection()
 *  
 *  Summary:
 *    Sets the selected item in a menu.
 *  
 *  Discussion:
 *    The HIMenuSetSelection API may be called either before opening a
 *    menu, or while a menu is open. If called before opening a menu,
 *    when the menu is next opened, the specified item will be
 *    selected; this case only works for menus that use an HIView to
 *    display menu content, and does not work for MDEF-based menus. If
 *    called while a menu is open, the menu's selection will be changed
 *    to the specified item. Note that changing the selection of an
 *    open menu is not standard UI and may be confusing to the user;
 *    also, changing the selected item does not currently affect the
 *    state of hierarchical submenus of the previously selected or
 *    newly selected item, so Apple recommends that you do not change
 *    the selected state of an item with a hierarchical submenu.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
<<<<<<< HEAD
 *      The menu whose icon title to retrieve.
 *    
 *    outType:
 *      On exit, contains the type of icon being used as the title of
 *      the menu. Contains kMenuNoIcon if the menu does not have an
 *      icon title.
 *    
 *    outIcon:
 *      On exit, contains the IconSuiteRef, IconRef, or CGImageRef
 *      being used as the title of the menu, or NULL if the menu does
 *      not have an icon title. May be NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuTitleIcon( inMenu: MenuRef; outType: UInt32Ptr { can be NULL }; outIcon: UnivPtrPtr { can be NULL } ): OSStatus; external name '_GetMenuTitleIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *      The menu whose selected item to set.
 *    
 *    inMenuType:
 *      The type of menu for which to set the selected item. Ignored
 *      unless the menu is already open; use GetMenuType to determine
 *      the menu's current type.
 *    
 *    inItem:
 *      The menu item that should be selected.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetSelection( inMenu: MenuRef; inMenuType: ThemeMenuType; inItem: MenuItemIndex ): OSStatus; external name '_HIMenuSetSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  InvalidateMenuSize()
 *  
 *  Summary:
 *    Invalidates the menu size so that it will be recalculated when
 *    next displayed.
 *  
 *  Discussion:
 *    The pre-Carbon technique for invalidating the menu size was to
 *    set the width and height to -1. Although this technique still
 *    works, for best compatibility it's preferable to use the
 *    InvalidateMenuSize API so that the Menu Manager has explicit
 *    notification that the menu is invalid.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose size to invalidate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
function InvalidateMenuSize( inMenu: MenuRef ): OSStatus; external name '_InvalidateMenuSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuSizeInvalid()
 *  
 *  Summary:
 *    Determines if a menu's size is invalid and should be recalculated.
 *  
 *  Discussion:
 *    The pre-Carbon technique for determining if a menu's size is
 *    invalid was to check if the width or height was -1. This
 *    technique is not always reliable on Carbon due to implementation
 *    changes in the Menu Manager. You should now use IsMenuSizeInvalid
 *    instead.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inMenu:
 *      The menu whose size to examine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    menu:
 *      The menu to display.
 *    
 *    top:
 *      The vertical position, in global coordinates, of the top left
 *      corner of the selected item when the menu is opened.
 *    
 *    left:
 *      The horizontal position, in global coordinates, of the top left
 *      corner of the selected item when the menu is opened.
 *    
 *    popUpItem:
 *      The item that should be positioned at the global point
 *      specified by the top and left parameters. May be zero, in which
 *      case item one is positioned at the specified global point.
 *  
 *  Result:
 *    A 32-bit value whose high 16-bit word is the menu ID and whose
 *    low 16-bit word is the index of the menu item that was selected,
 *    or zero if no item was selected.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
function IsMenuSizeInvalid( inMenu: MenuRef ): Boolean; external name '_IsMenuSizeInvalid';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HIMenuGetContentView()
 *  
 *  Summary:
 *    Returns the HIViewRef that will be used to draw menu content for
 *    this menu, if any.
 *  
 *  Discussion:
 *    If the content view has not yet been created, the Menu Manager
 *    will create the content view using the view class ID and
 *    initialization event associated with the menu. Note that the menu
 *    content view is not the same as the window content view; the menu
 *    content view is embedded inside the window content view. If the
 *    menu uses an MDEF instead of an HIView to draw its content, noErr
 *    is returned but the output HIViewRef is set to NULL.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu.
 *    
 *    inMenuType:
 *      The type of menu for which the menu content view should be
 *      returned. The same MenuRef may have multiple content views,
 *      depending on the menu type being displayed.
 *    
 *    outView:
 *      On exit, contains the view. May be set to NULL if the menu does
 *      not use an HIView to draw its content. The caller should not
 *      release this view.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
function HIMenuGetContentView( inMenu: MenuRef; inMenuType: ThemeMenuType; var outView: HIViewRef ): OSStatus; external name '_HIMenuGetContentView';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIMenuSetFont()
 *  
 *  Summary:
 *    Sets the font to be used to draw a menu or a menu item.
 *  
 *  Discussion:
 *    Pass zero for the inItem parameter to set the default font for
 *    all of the items in a menu. Pass a non-zero value for the inItem
 *    parameter to set a font for a specific item; this overrides the
 *    default font for the menu. If a font is set for an entire menu,
 *    it will also apply to all submenus of the menu. Setting a font
 *    for a specific item does not cause that font to apply to the
 *    item's submenu.
 *  
 *  Summary:
 *    Determines whether an event should result in the selection of a
 *    menu item.
 *  
 *  Discussion:
 *    Apple recommends that you use the IsMenuKeyEvent API rather than
 *    this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inMenu:
 *      The menu whose font to set.
 *    
 *    inItem:
 *      The item whose font to set, or zero to customze the font for
 *      the entire menu.
 *    
 *    inFont:
 *      The font to use for the specified menu or item. May be NULL to
 *      remove the menu or item's current custom font. The font will be
 *      retained by the Menu Manager and released when the menu or item
 *      is destroyed, or when the font for the menu or item is changed
 *      to a different value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
=======
 *    inEvent:
 *      The event to examine. Typically, this is a keyboard event.
 *  
 *  Result:
 *    If the keyboard event corresponds to a menu item's keyboard
 *    equivalent, then the result is a 32-bit value whose high 16-bit
 *    word is the menu ID and whose low 16-bit word is the index of the
 *    menu item that was found. Zero is returned if no item matched the
 *    event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
>>>>>>> origin/cpstrnew
 }
function HIMenuSetFont( inMenu: MenuRef; inItem: MenuItemIndex; inFont: CTFontRef { can be NULL } ): OSStatus; external name '_HIMenuSetFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD


{
 *  HIMenuCopyFont()
 *  
 *  Summary:
 *    Retrieves the custom font that is used to draw a menu or a menu
 *    item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose font to retrieve.
 *    
 *    inItem:
 *      The item whose font to retrieve, or zero to retrieve the font
 *      for the entire menu.
 *    
 *    outFont:
 *      On exit, contains the font used for the specified menu or item.
 *      If the menu or item does not have a custom font, the resulting
 *      CTFontRef is set to NULL. The caller must release the
 *      CTFontRef, if it is non-NULL.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyFont( inMenu: MenuRef; inItem: MenuItemIndex; var outFont: CTFontRef ): OSStatus; external name '_HIMenuCopyFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Insertion                                                               }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  DeleteMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DeleteMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_DeleteMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuItems()
 *  
 *  Summary:
 *    Copies menu items from one menu to another.
=======
=======
>>>>>>> graemeg/cpstrnew


{
 *  HIMenuCopyFont()
 *  
 *  Summary:
 *    Retrieves the custom font that is used to draw a menu or a menu
 *    item.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
 *    inMenu:
 *      The menu whose font to retrieve.
 *    
 *    inItem:
 *      The item whose font to retrieve, or zero to retrieve the font
 *      for the entire menu.
 *    
 *    outFont:
 *      On exit, contains the font used for the specified menu or item.
 *      If the menu or item does not have a custom font, the resulting
 *      CTFontRef is set to NULL. The caller must release the
 *      CTFontRef, if it is non-NULL.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.3 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyFont( inMenu: MenuRef; inItem: MenuItemIndex; var outFont: CTFontRef ): OSStatus; external name '_HIMenuCopyFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


<<<<<<< HEAD
{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Insertion                                                               }
{——————————————————————————————————————————————————————————————————————————————————————}
=======
{$endc} {not TARGET_CPU_64}


>>>>>>> origin/cpstrnew
{
 *  DeleteMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DeleteMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_DeleteMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  CopyMenuItems()
 *  
 *  Summary:
 *    Copies menu items from one menu to another.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *    inSourceMenu:
 *      The menu from which to copy items.
 *    
 *    inFirstItem:
 *      The first item to copy.
 *    
<<<<<<< HEAD
 *    inNumItems:
 *      The number of items to copy.
 *    
 *    inDestMenu:
 *      The menu to which to copy items.
 *    
 *    inInsertAfter:
 *      The menu item in the destination menu after which to insert the
 *      copied items. Pass 0 to insert the items at the beginning of
 *      the menu. This value of this parameter must not exceed the
 *      number of items in the destination menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
=======
 *    inDismissalReason:
 *      Why the menu is being dismissed; this value will be added to
 *      the kEventMenuEndTracking event. On Mac OS X 10.5 and later,
 *      you may pass zero to indicate that
 *      kHIMenuDismissedByCancelMenuTracking should be passed to the
 *      EndTracking event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function CopyMenuItems( inSourceMenu: MenuRef; inFirstItem: MenuItemIndex; inNumItems: ItemCount; inDestMenu: MenuRef; inInsertAfter: MenuItemIndex ): OSStatus; external name '_CopyMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DeleteMenuItems()
 *  
 *  Summary:
 *    Deletes multiple menu items.
 *  
 *  Discussion:
 *    This API is more efficient than calling DeleteMenuItem multiple
 *    times.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu from which to delete items.
 *    
 *    inFirstItem:
 *      The first item to delete.
 *    
 *    inNumItems:
 *      The number of items to delete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
function DeleteMenuItems( inMenu: MenuRef; inFirstItem: MenuItemIndex; inNumItems: ItemCount ): OSStatus; external name '_DeleteMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AppendMenuItemTextWithCFString()
 *  
 *  Summary:
 *    Appends a new menu item with text from a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    AppendMenuItemWithTextCFString, depending on whether the string
 *    is mutable or immutable. If the string is mutable, modifying the
 *    string after calling AppendMenuItemTextWithCFString will have no
 *    effect on the menu item's actual text. The caller may release the
 *    string after calling AppendMenuItemTextWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to which to append the new item.
 *    
 *    inString:
 *      The text of the new item.
 *    
 *    inAttributes:
 *      The attributes of the new item.
 *    
 *    inCommandID:
 *      The command ID of the new item.
 *    
 *    outNewItem:
 *      On exit, the index of the new item. May be NULL if the caller
 *      does not need this information.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AppendMenuItemTextWithCFString( inMenu: MenuRef; inString: CFStringRef; inAttributes: MenuItemAttributes; inCommandID: MenuCommand; outNewItem: MenuItemIndexPtr { can be NULL } ): OSStatus; external name '_AppendMenuItemTextWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InsertMenuItemTextWithCFString()
 *  
 *  Summary:
 *    Inserts a new menu item with text from a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    InsertMenuItemWithCFString, depending on whether the string is
 *    mutable or immutable. If the string is mutable, modifying the
 *    string after calling InsertMenuItemWithCFString will have no
 *    effect on the menu item's actual text. The caller may release the
 *    string after calling InsertMenuItemWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to insert the new item.
 *    
 *    inString:
 *      The text of the new item.
 *    
 *    inAfterItem:
 *      The item after which to insert the new item. Pass 0 to insert
 *      the item at the beginning of the menu. If inAfterItem is
 *      greater than the number of items in the menu, the item is
 *      inserted at the end of the menu.
 *    
 *    inAttributes:
 *      The attributes of the new item.
 *    
 *    inCommandID:
 *      The command ID of the new item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
function InsertMenuItemTextWithCFString( inMenu: MenuRef; inString: CFStringRef; inAfterItem: MenuItemIndex; inAttributes: MenuItemAttributes; inCommandID: MenuCommand ): OSStatus; external name '_InsertMenuItemTextWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Events                                                                       }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  HIMenuSetSelection()
 *  
 *  Summary:
 *    Sets the selected item in a menu.
 *  
 *  Discussion:
 *    The HIMenuSetSelection API may be called either before opening a
 *    menu, or while a menu is open. If called before opening a menu,
 *    when the menu is next opened, the specified item will be
 *    selected; this case only works for menus that use an HIView to
 *    display menu content, and does not work for MDEF-based menus. If
 *    called while a menu is open, the menu's selection will be changed
 *    to the specified item. Note that changing the selection of an
 *    open menu is not standard UI and may be confusing to the user;
 *    also, changing the selected item does not currently affect the
 *    state of hierarchical submenus of the previously selected or
 *    newly selected item, so Apple recommends that you do not change
 *    the selected state of an item with a hierarchical submenu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose selected item to set.
 *    
 *    inMenuType:
 *      The type of menu for which to set the selected item. Ignored
 *      unless the menu is already open; use GetMenuType to determine
 *      the menu's current type.
 *    
 *    inItem:
 *      The menu item that should be selected.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetSelection( inMenu: MenuRef; inMenuType: ThemeMenuType; inItem: MenuItemIndex ): OSStatus; external name '_HIMenuSetSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  MenuSelect()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MenuSelect( startPt: Point ): SInt32; external name '_MenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PopUpMenuSelect()
 *  
 *  Summary:
 *    Displays a pop-up menu at a specified location.
 *  
 *  Discussion:
 *    In Mac OS 9 and earlier, PopUpMenuSelect requires that the menu
 *    be inserted into the menubar using InsertMenu( menuRef,
 *    kInsertHierarchicalMenu ). CarbonLib 1.1 and later, and Mac OS X,
 *    do not have this requirement; a menu can be displayed by
 *    PopUpMenuSelect even if it is not inserted in the menubar.
 *  
 *  Summary:
 *    Creates multiple menus using the contents of a 'MBAR' resource.
 *  
 *  Discussion:
 *    Apple recommends that you use Interface Builder to design your
 *    menus and other user interface components. Interface Builder
 *    stores its output in nib files rather then Resource Manager
 *    resources. Use the CreateMenuBarFromNib API to load a menu from a
 *    nib file.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    menu:
 *      The menu to display.
 *    
 *    top:
 *      The vertical position, in global coordinates, of the top left
 *      corner of the selected item when the menu is opened.
 *    
 *    left:
 *      The horizontal position, in global coordinates, of the top left
 *      corner of the selected item when the menu is opened.
 *    
 *    popUpItem:
 *      The item that should be positioned at the global point
 *      specified by the top and left parameters. May be zero, in which
 *      case item one is positioned at the specified global point.
 *  
 *  Result:
 *    A 32-bit value whose high 16-bit word is the menu ID and whose
 *    low 16-bit word is the index of the menu item that was selected,
 *    or zero if no item was selected.
=======
 *    menuBarID:
 *      The resource ID of the 'MBAR' resource.
 *  
 *  Result:
 *    The newly created menubar handle. Use SetMenuBar to place the
 *    contents of the menubar handle into the menubar. You must release
 *    the menubar handle with DisposeMenuBar when you are done with it.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PopUpMenuSelect( menu: MenuRef; top: SInt16; left: SInt16; popUpItem: MenuItemIndex ): SInt32; external name '_PopUpMenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MenuChoice()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MenuChoice: SInt32; external name '_MenuChoice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MenuEvent()
 *  
 *  Summary:
 *    Determines whether an event should result in the selection of a
 *    menu item.
 *  
 *  Discussion:
 *    Apple recommends that you use the IsMenuKeyEvent API rather than
 *    this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine. Typically, this is a keyboard event.
 *  
 *  Result:
 *    If the keyboard event corresponds to a menu item's keyboard
 *    equivalent, then the result is a 32-bit value whose high 16-bit
 *    word is the menu ID and whose low 16-bit word is the index of the
 *    menu item that was found. Zero is returned if no item matched the
 *    event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
function MenuEvent( const (*var*) inEvent: EventRecord ): UInt32; external name '_MenuEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuKeyEvent()
 *  
 *  Summary:
 *    Determines if an event corresponds to a menu command key.
 *  
 *  Discussion:
 *    By default, IsMenuKeyEvent searches the menus in the current menu
 *    bar and hilites the menu title of the menu containing the
 *    selected item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inStartMenu:
 *      IsMenuKeyEvent searches for matching menu items in this menu
 *      and all of its submenus. May be NULL to search the current menu
 *      bar contents.
 *    
 *    inEvent:
 *      The event to match against. Non-keyboard events are ignored.
 *    
 *    inOptions:
 *      Options controlling how to search. Pass kNilOptions for the
 *      default behavior.
 *    
 *    outMenu:
 *      On exit, the menu containing the matching item. May be NULL.
 *    
 *    outMenuItem:
 *      On exit, the menu item that matched. May be NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.1 and later
=======
 *    CarbonLib:        in CarbonLib 1.0.2 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function IsMenuKeyEvent( inStartMenu: MenuRef; inEvent: EventRef; inOptions: MenuEventOptions; outMenu: MenuRefPtr { can be NULL }; outMenuItem: MenuItemIndexPtr { can be NULL } ): Boolean; external name '_IsMenuKeyEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalidateMenuEnabling()
 *  
 *  Summary:
 *    Causes the menu enable state to be recalculated at the next
 *    convenient opportunity.
 *  
 *  Discussion:
 *    It is common for state changes in an application (for example,
 *    selection of text) to cause a change in the enabling of items in
 *    the application's menu (for example, the Copy menu item might
 *    become enabled). In a Carbon-event-savvy application, menu items
 *    are enabled or disabled in response to an
 *    kEventCommandUpdateStatus event; however, this event is normally
 *    only sent before a command key press or a click in the menubar.
 *    You can request an explicit recalculation of a menu's enable
 *    state with the InvalidateMenuEnabling API. The Carbon Event
 *    Manager will automatically invalidate the enable state of all
 *    top-level menus when a user event is dispatched, the user focus
 *    changes, or the active window changes, so in many cases you will
 *    not need to explicitly invalidate the menu enabling state.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      A menu to re-enable, or NULL if all menus in the root menu
 *      should be re-enabled.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.3 and later
=======
 *    CarbonLib:        in CarbonLib 1.0.2 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function InvalidateMenuEnabling( inMenu: MenuRef ): OSStatus; external name '_InvalidateMenuEnabling';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}


{
 *  Summary:
 *    Menu dismissal causation constants
 }
const
{
   * The menu was dismissed by the selection of a menu item.
   }
	kHIMenuDismissedBySelection = 1;

  {
   * The menu was dismissed because the user canceled menu tracking.
   }
	kHIMenuDismissedByUserCancel = 2;

  {
   * The menu was dismissed by a mouse-down somewhere that did not
   * result in menu item selection.
   }
	kHIMenuDismissedByMouseDown = 3;

  {
   * The menu was dismissed by a mouse-up.
   }
	kHIMenuDismissedByMouseUp = 4;

  {
   * The menu was dismissed by a keyboard event.
   }
	kHIMenuDismissedByKeyEvent = 5;

  {
   * The menu was dismissed because the current application was no
   * longer frontmost.
   }
	kHIMenuDismissedByAppSwitch = 6;

  {
   * The menu was dismissed because menu tracking mode timed out.
   }
	kHIMenuDismissedByTimeout = 7;

  {
   * The menu was dismissed by the CancelMenuTracking API.
   }
	kHIMenuDismissedByCancelMenuTracking = 8;

  {
   * The menu was dismissed because the active window changed.
   }
	kHIMenuDismissedByActivationChange = 9;

  {
   * The menu was dismissed bcause the user focus window changed, or
   * because keyboard focus was removed from the current process.
   }
	kHIMenuDismissedByFocusChange = 10;

{$ifc not TARGET_CPU_64}
{
 *  CancelMenuTracking()
 *  
 *  Summary:
 *    Cancels a menu tracking session.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRootMenu:
 *      The root menu of the menu tracking session that should be
 *      dismissed. For menubar tracking, use the result of AcquireRoot
 *      menu; for popup menu tracking, use the menu that was passed to
 *      PopUpMenuSelect.
 *    
 *    inImmediate:
 *      Whether the open menus should disappear immediately or fade out.
 *    
 *    inDismissalReason:
 *      Why the menu is being dismissed; this value will be added to
 *      the kEventMenuEndTracking event. On Mac OS X 10.5 and later,
 *      you may pass zero to indicate that
 *      kHIMenuDismissedByCancelMenuTracking should be passed to the
 *      EndTracking event.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
function CancelMenuTracking( inRootMenu: MenuRef; inImmediate: Boolean; inDismissalReason: UInt32 ): OSStatus; external name '_CancelMenuTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Bar                                                                          }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  GetMBarHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMBarHeight: SInt16; external name '_GetMBarHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]DrawMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DrawMenuBar; external name '_DrawMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacDrawMenuBar; external name '_DrawMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InvalMenuBar; external name '_InvalMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  IsMenuBarInvalid()
 *  
 *  Summary:
 *    Determines if the menubar is invalid and should be redrawn.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    rootMenu:
 *      The root menu for the menubar to be examined. Pass NULL to
 *      check the state of the current menubar.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuBarInvalid( rootMenu: MenuRef ): Boolean; external name '_IsMenuBarInvalid';
=======
 *  FlashMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure FlashMenuBar( menuID_: MenuID ); external name '_FlashMenuBar';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  HiliteMenu()
=======
 *  IsMenuBarVisible()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure HiliteMenu( menuID_: MenuID ); external name '_HiliteMenu';
=======
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function IsMenuBarVisible: Boolean; external name '_IsMenuBarVisible';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  GetNewMBar()
 *  
 *  Summary:
 *    Creates multiple menus using the contents of a 'MBAR' resource.
 *  
 *  Discussion:
 *    Apple recommends that you use Interface Builder to design your
 *    menus and other user interface components. Interface Builder
 *    stores its output in nib files rather then Resource Manager
 *    resources. Use the CreateMenuBarFromNib API to load a menu from a
 *    nib file.
=======
 *  ShowMenuBar()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menuBarID:
 *      The resource ID of the 'MBAR' resource.
 *  
 *  Result:
 *    The newly created menubar handle. Use SetMenuBar to place the
 *    contents of the menubar handle into the menubar. You must release
 *    the menubar handle with DisposeMenuBar when you are done with it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
<<<<<<< HEAD
function GetNewMBar( menuBarID: SInt16 ): MenuBarHandle; external name '_GetNewMBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMenuBar: MenuBarHandle; external name '_GetMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMenuBar( mbar: MenuBarHandle ); external name '_SetMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DuplicateMenuBar()
 *  
 *  Summary:
 *    Duplicates a menubar handle.
 *  
 *  Discussion:
 *    This API should be used in Carbon applications when duplicating a
 *    handle returned from GetMenuBar or GetNewMBar. You should not use
 *    Memory Manager APIs (HandToHand, NewHandle, etc) to duplicate
 *    such a handle. This is necessary in Carbon so that the refcounts
 *    of the menus in the menubar handle can be incremented when the
 *    handle is duplicated.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMbar:
 *      The menubar handle to duplicate.
 *    
 *    outMbar:
 *      On exit, contains the new menubar handle.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function DuplicateMenuBar( inMbar: MenuBarHandle; var outMbar: MenuBarHandle ): OSStatus; external name '_DuplicateMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeMenuBar()
 *  
 *  Summary:
 *    Releases a menubar handle.
 *  
 *  Discussion:
 *    This API should be used in Carbon applications when releasing a
 *    handle returned from GetMenuBar or GetNewMBar. You should not use
 *    DisposeHandle to release such a handle. This is necessary in
 *    Carbon so that the refcounts of the menus in the menubar handle
 *    can be decremented when the handle is released.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMbar:
 *      The menubar handle to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function DisposeMenuBar( inMbar: MenuBarHandle ): OSStatus; external name '_DisposeMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMenuHandle( menuID_: MenuID ): MenuRef; external name '_GetMenuHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function GetMenuRef( menuID_: MenuID ): MenuRef; external name '_GetMenuHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]InsertMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertMenu( theMenu: MenuRef; beforeID: MenuID ); external name '_InsertMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacInsertMenu( theMenu: MenuRef; beforeID: MenuID ); external name '_InsertMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]DeleteMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DeleteMenu( menuID_: MenuID ); external name '_DeleteMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacDeleteMenu( menuID_: MenuID ); external name '_DeleteMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ClearMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure ClearMenuBar; external name '_ClearMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FlashMenuBar()
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure FlashMenuBar( menuID_: MenuID ); external name '_FlashMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuBarVisible()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function IsMenuBarVisible: Boolean; external name '_IsMenuBarVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ShowMenuBar()
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure FlashMenuBar( menuID_: MenuID ); external name '_FlashMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuBarVisible()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function IsMenuBarVisible: Boolean; external name '_IsMenuBarVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ShowMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure ShowMenuBar; external name '_ShowMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HideMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure HideMenuBar; external name '_HideMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AcquireRootMenu()
 *  
 *  Summary:
 *    Get the menu whose contents are displayed in the menubar.
 *  
 *  Discussion:
 *    The refcount of the root menu is incremented by this API. The
 *    caller should release a refcount with ReleaseMenu when it’s done
 *    with the menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AcquireRootMenu: MenuRef; external name '_AcquireRootMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetRootMenu()
 *  
 *  Summary:
 *    Sets the menu whose contents are displayed in the menubar.
 *  
 *  Discussion:
 *    The refcount of the root menu is incremented by this API. The
 *    caller may release the menu after calling SetRootMenu.
 *    
 *    A root menu should contain one menu item for each top-level menu
 *    that should be displayed in the menubar. Each menu item should
 *    have a submenu that was installed with
 *    SetMenuItemHierarchicalMenu.
 *    
 *    SetRootMenu also sets the contents of the hierarchical portion of
 *    the menulist (the set of menus that were inserted with
 *    InsertMenu( menu, kInsertHierarchicalMenu). If a menu that was
 *    returned by AcquireRootMenu is passed to SetRootMenu, the
 *    hierarchical menulist is changed to include the menus that were
 *    in the hierarchical menulist when AcquireRootMenu was called. If
 *    a newly created menu is passed to SetRootMenu, the hierarchical
 *    menulist is cleared and has no menus in it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The new root menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetRootMenu( inMenu: MenuRef ): OSStatus; external name '_SetRootMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Accessors                                                               }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  [Mac]CheckMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
procedure CheckMenuItem( theMenu: MenuRef; item: MenuItemIndex; checked: Boolean ); external name '_CheckMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacCheckMenuItem( theMenu: MenuRef; item: MenuItemIndex; checked: Boolean ); external name '_CheckMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetItemMark()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemMark( theMenu: MenuRef; item: MenuItemIndex; markChar: CharParameter ); external name '_SetItemMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetItemMark()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }

procedure __GetItemMark( theMenu: MenuRef; item: MenuItemIndex; var markChar: UInt16 ); external name '_GetItemMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
		With GPC and FPC on Intel we have an endian issue, because markChar is expected to be 16-bit
}

procedure GetItemMark( theMenu: MenuRef; item: MenuItemIndex; var markChar: CharParameter ); inline;


{
 *  SetItemCmd()
 *  
 *  Summary:
 *    Sets a menu item's keyboard equivalent to a specified character
 *    in the application's text encoding.
 *  
 *  Discussion:
 *    Apple recommends that you use the SetMenuItemCommandKey API
 *    instead of this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose item to modify.
 *    
 *    item:
 *      The item whose keyboard equivalent to set.
 *    
 *    cmdChar:
 *      A character in the application's text encoding, or zero to
 *      remove the item's keyboard equivalent.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemCmd( theMenu: MenuRef; item: MenuItemIndex; cmdChar: CharParameter ); external name '_SetItemCmd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetItemCmd()
 *  
 *  Summary:
 *    Returns a menu item's keyboard equivalent in the application's
 *    text encoding.
 *  
 *  Discussion:
 *    Apple recommends that you use the GetMenuItemCommandKey API
 *    instead of this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose item to query.
 *    
 *    item:
 *      The item whose keyboard equivalent to return.
 *    
 *    cmdChar:
 *      On exit, contains the menu item's keyboard equivalent, or zero
 *      if the item has no keyboard equivalent.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure __GetItemCmd( theMenu: MenuRef; item: MenuItemIndex; var cmdChar: UInt16 ); external name '_GetItemCmd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
		With GPC and FPC on Intel we have an endian issue, because cmdChar is expected to be 16-bit
}

procedure GetItemCmd( theMenu: MenuRef; item: MenuItemIndex; var cmdChar: CharParameter ); inline;


{
 *  SetItemStyle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemStyle( theMenu: MenuRef; item: MenuItemIndex; chStyle: StyleParameter ); external name '_SetItemStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetItemStyle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetItemStyle( theMenu: MenuRef; item: MenuItemIndex; var chStyle: Style ); external name '_GetItemStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemCommandID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemCommandID( inMenu: MenuRef; inItem: MenuItemIndex; inCommandID: MenuCommand ): OSErr; external name '_SetMenuItemCommandID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemCommandID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemCommandID( inMenu: MenuRef; inItem: MenuItemIndex; var outCommandID: MenuCommand ): OSErr; external name '_GetMenuItemCommandID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemModifiers()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemModifiers( inMenu: MenuRef; inItem: MenuItemIndex; inModifiers: SInt8 ): OSErr; external name '_SetMenuItemModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemModifiers()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemModifiers( inMenu: MenuRef; inItem: MenuItemIndex; var outModifiers: SInt8 ): OSErr; external name '_GetMenuItemModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemIconHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemIconHandle( inMenu: MenuRef; inItem: MenuItemIndex; inIconType: SInt8; inIconHandle: Handle ): OSErr; external name '_SetMenuItemIconHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemIconHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemIconHandle( inMenu: MenuRef; inItem: MenuItemIndex; var outIconType: SInt8; var outIconHandle: Handle ): OSErr; external name '_GetMenuItemIconHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{
 *  SetMenuItemFontID()
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{
 *  SetMenuItemFontID()
 *  
 *  Summary:
 *    Sets the QuickDraw font ID used to draw a menu item.
 *  
 *  Discussion:
 *    The font ID specified with this API overrides the font ID
 *    specified with the SetMenuFont API. 
 *    
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuItemFontID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item whose font to set.
 *    
 *    inItem:
 *      The menu item whose font to set.
 *    
 *    inFontID:
 *      The menu item's font, or zero to use the standard font for menu
 *      items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemFontID( inMenu: MenuRef; inItem: MenuItemIndex; inFontID: SInt16 ): OSErr; external name '_SetMenuItemFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemFontID()
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID used to draw a menu item.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuItemFontID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item whose font to retrieve.
 *    
 *    inItem:
 *      The menu item whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu item's font, or zero if the font has
 *      not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemFontID( inMenu: MenuRef; inItem: MenuItemIndex; var outFontID: SInt16 ): OSErr; external name '_GetMenuItemFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemRefCon()
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Sets the QuickDraw font ID used to draw a menu item.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    The font ID specified with this API overrides the font ID
 *    specified with the SetMenuFont API. 
 *    
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuItemFontID.
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemRefCon( inMenu: MenuRef; inItem: MenuItemIndex; inRefCon: URefCon ): OSErr; external name '_SetMenuItemRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemRefCon()
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item whose font to set.
 *    
 *    inItem:
 *      The menu item whose font to set.
 *    
 *    inFontID:
 *      The menu item's font, or zero to use the standard font for menu
 *      items.
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemRefCon( inMenu: MenuRef; inItem: MenuItemIndex; var outRefCon: URefCon ): OSErr; external name '_GetMenuItemRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemKeyGlyph()
 *  
 *  Summary:
 *    Sets the command key glyph code for a menu item.
 *  
 *  Discussion:
 *    A menu item's command key may be customized using a key glyph
 *    code; these codes are the kMenu*Glyph constants documented in
 *    Menus.h. In classic Mac OS, a glyph code is only used for
 *    display; it does not affect command key matching. In Carbon, a
 *    menu item's glyph code is used for command key matching if the
 *    menu item does not have a command key character or virtual
 *    keycode assigned to it. 
 *    
 *    In CarbonLib 1.2 and Mac OS X 10.0 and later, the Menu Manager
 *    will automatically draw the appropriate glyph for a menu item
 *    that has a virtual keycode command key assigned to it; it is not
 *    necessary to set both the virtual keycode and the glyph for an
 *    item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to change.
 *    
 *    inItem:
 *      The menu item to change.
 *    
 *    inGlyph:
 *      The new glyph code for the item, or zero to remove the item's
 *      glyph code.
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to change.
 *    
 *    inItem:
 *      The menu item to change.
 *    
 *    inGlyph:
 *      The new glyph code for the item, or zero to remove the item's
 *      glyph code.
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to change.
 *    
 *    inItem:
 *      The menu item to change.
 *    
 *    inGlyph:
 *      The new glyph code for the item, or zero to remove the item's
 *      glyph code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemKeyGlyph( inMenu: MenuRef; inItem: MenuItemIndex; inGlyph: SInt16 ): OSErr; external name '_SetMenuItemKeyGlyph';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemKeyGlyph()
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID used to draw a menu item.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuItemFontID.
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item whose font to retrieve.
 *    
 *    inItem:
 *      The menu item whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu item's font, or zero if the font has
 *      not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemKeyGlyph( inMenu: MenuRef; inItem: MenuItemIndex; var outGlyph: SInt16 ): OSErr; external name '_GetMenuItemKeyGlyph';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Routines available in Mac OS 8.5 and later (supporting enabling/disabling of > 31 items)}

{
 *  [Mac]EnableMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemRefCon( inMenu: MenuRef; inItem: MenuItemIndex; inRefCon: URefCon ): OSErr; external name '_SetMenuItemRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemRefCon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemRefCon( inMenu: MenuRef; inItem: MenuItemIndex; var outRefCon: URefCon ): OSErr; external name '_GetMenuItemRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemKeyGlyph()
 *  
 *  Summary:
 *    Sets the command key glyph code for a menu item.
 *  
 *  Discussion:
 *    A menu item's command key may be customized using a key glyph
 *    code; these codes are the kMenu*Glyph constants documented in
 *    Menus.h. In classic Mac OS, a glyph code is only used for
 *    display; it does not affect command key matching. In Carbon, a
 *    menu item's glyph code is used for command key matching if the
 *    menu item does not have a command key character or virtual
 *    keycode assigned to it. 
 *    
 *    In CarbonLib 1.2 and Mac OS X 10.0 and later, the Menu Manager
 *    will automatically draw the appropriate glyph for a menu item
 *    that has a virtual keycode command key assigned to it; it is not
 *    necessary to set both the virtual keycode and the glyph for an
 *    item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to change.
 *    
 *    inItem:
 *      The menu item to change.
 *    
 *    inGlyph:
 *      The new glyph code for the item, or zero to remove the item's
 *      glyph code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemKeyGlyph( inMenu: MenuRef; inItem: MenuItemIndex; inGlyph: SInt16 ): OSErr; external name '_SetMenuItemKeyGlyph';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemKeyGlyph()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemKeyGlyph( inMenu: MenuRef; inItem: MenuItemIndex; var outGlyph: SInt16 ): OSErr; external name '_GetMenuItemKeyGlyph';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Routines available in Mac OS 8.5 and later (supporting enabling/disabling of > 31 items)}

{
 *  [Mac]EnableMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure EnableMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_EnableMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacEnableMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_EnableMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisableMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure DisableMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_DisableMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuItemEnabled()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function IsMenuItemEnabled( menu: MenuRef; item: MenuItemIndex ): Boolean; external name '_IsMenuItemEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EnableMenuItemIcon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure EnableMenuItemIcon( theMenu: MenuRef; item: MenuItemIndex ); external name '_EnableMenuItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisableMenuItemIcon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure DisableMenuItemIcon( theMenu: MenuRef; item: MenuItemIndex ); external name '_DisableMenuItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuItemIconEnabled()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function IsMenuItemIconEnabled( menu: MenuRef; item: MenuItemIndex ): Boolean; external name '_IsMenuItemIconEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemHierarchicalMenu()
 *  
 *  Summary:
 *    Attaches a submenu to a menu item.
 *  
 *  Discussion:
 *    Using SetMenuItemHierarchicalMenu, it is possible to directly
 *    specify the submenu for a menu item without specifying its menu
 *    ID. It is not necessary to insert the submenu into the
 *    hierarchical portion of the menubar, and it is not necessary for
 *    the submenu to have a unique menu ID; it is recommended that you
 *    use 0 as the menu ID for the submenu, and identify selections
 *    from the menu by command ID. The Menu Manager will increment the
 *    refcount of the submenu that you specify, and the submenu's
 *    refcount will be decremented automatically when the parent menu
 *    item is deleted or the parent menu is disposed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The parent menu.
 *    
 *    inItem:
 *      The parent item.
 *    
 *    inHierMenu:
 *      The submenu. You may pass NULL to remove any existing submenu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemHierarchicalMenu( inMenu: MenuRef; inItem: MenuItemIndex; inHierMenu: MenuRef { can be NULL } ): OSStatus; external name '_SetMenuItemHierarchicalMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemHierarchicalMenu()
 *  
 *  Summary:
 *    Returns the submenu attached to a menu item.
 *  
 *  Discussion:
 *    GetMenuItemHierarchicalMenu will return the submenu attached to a
 *    menu item regardless of how the submenu was specified. If the
 *    submenu was specified by menu ID (using SetItemCmd or
 *    SetMenuItemHierarchicalID), GetMenuItemHierarchicalMenu will
 *    return the currently installed menu with that ID, if any. The
 *    only case where GetMenuItemHierarchicalMenu will fail to return
 *    the item's submenu is when the submenu is specified by menu ID,
 *    but the submenu is not currently inserted in the menu bar.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The parent menu.
 *    
 *    inItem:
 *      The parent item.
 *    
 *    outHierMenu:
 *      On exit, the item's submenu, or NULL if it does not have one.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemHierarchicalMenu( inMenu: MenuRef; inItem: MenuItemIndex; var outHierMenu: MenuRef ): OSStatus; external name '_GetMenuItemHierarchicalMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HIMenuCopyParents()
 *  
 *  Summary:
 *    Returns the parent menus of a menu.
 *  
 *  Discussion:
 *    A menu may be a submenu of one or more other menus; for example,
 *    a Spelling menu might be a submenu of both an Edit menu contained
 *    in the menubar, and a contextual menu displayed when the user
 *    clicks in a document. HIMenuCopyParents returns an array of
 *    MenuRefs that are the parent menus of the specified menu. If the
 *    menu has only a single parent, the array will have only that
 *    single parent menu in it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose parent menus to return.
 *    
 *    outParents:
 *      On exit, contains an array of MenuRefs that are parents of the
 *      specified menu. If the menu does not have any parents, the
 *      array on exit will be NULL. If not NULL, the caller is
 *      responsible for releasing the array.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyParents( inMenu: MenuRef; var outParents: CFArrayRef ): OSStatus; external name '_HIMenuCopyParents';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  CopyMenuItemTextAsCFString()
 *  
 *  Summary:
 *    Returns a CFString containing the text of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose text to return.
 *    
 *    outString:
 *      On exit, a CFString containing the item's text. This string
 *      must be released by the caller.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuItemTextAsCFString( inMenu: MenuRef; inItem: MenuItemIndex; var outString: CFStringRef ): OSStatus; external name '_CopyMenuItemTextAsCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemTextWithCFString()
 *  
 *  Summary:
 *    Sets the text of a menu item to the text contained in a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    SetMenuItemTextWithCFString, depending on whether the string is
 *    mutable or immutable. If the string is mutable, modifying the
 *    string after calling SetMenuItemTextWithCFString will have no
 *    effect on the menu item's actual text. The caller may release the
 *    string after calling SetMenuItemTextWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose text to set.
 *    
 *    inString:
 *      The string containing the new menu item text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemTextWithCFString( inMenu: MenuRef; inItem: MenuItemIndex; inString: CFStringRef ): OSStatus; external name '_SetMenuItemTextWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HIMenuSetAttributedItemText()
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======
 *  
 *  Summary:
 *    Sets the attributed text of a menu item.
 *  
 *  Discussion:
 *    A menu item may have two strings attached to it: a regular
 *    CFString and an attributed CFString. The regular string is
 *    modified by the SetMenuItemTextAsCFString API, and the attributed
 *    string is modified using this API. If an attributed string is
 *    provided, it will be used when measuring and drawing the menu
 *    item text. However, the regular string (if provided) is always
 *    used for type-selection; this allows the attributed string to
 *    contain extra characters that should be ignored during
 *    type-selection.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to set.
 *    
 *    inText:
 *      The new attributed text for the menu item. May be NULL if the
 *      item no longer need attributed text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; inText: CFAttributedStringRef { can be NULL } ): OSStatus; external name '_HIMenuSetAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyAttributedItemText()
 *  
 *  Summary:
 *    Retrieves the attributed text of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to retrieve.
 *    
 *    outText:
 *      On exit, contains the item's attributed text, or NULL if the
 *      item has no attributed text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; var outText: CFAttributedStringRef ): OSStatus; external name '_HIMenuCopyAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  GetMenuItemIndent()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Sets the attributed text of a menu item.
 *  
 *  Discussion:
 *    A menu item may have two strings attached to it: a regular
 *    CFString and an attributed CFString. The regular string is
 *    modified by the SetMenuItemTextAsCFString API, and the attributed
 *    string is modified using this API. If an attributed string is
 *    provided, it will be used when measuring and drawing the menu
 *    item text. However, the regular string (if provided) is always
 *    used for type-selection; this allows the attributed string to
 *    contain extra characters that should be ignored during
 *    type-selection.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to set.
 *    
 *    inText:
 *      The new attributed text for the menu item. May be NULL if the
 *      item no longer need attributed text.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; inText: CFAttributedStringRef { can be NULL } ): OSStatus; external name '_HIMenuSetAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyAttributedItemText()
 *  
 *  Summary:
 *    Retrieves the attributed text of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to retrieve.
 *    
 *    outText:
 *      On exit, contains the item's attributed text, or NULL if the
 *      item has no attributed text.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; var outText: CFAttributedStringRef ): OSStatus; external name '_HIMenuCopyAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  GetMenuItemIndent()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Sets the attributed text of a menu item.
 *  
 *  Discussion:
 *    A menu item may have two strings attached to it: a regular
 *    CFString and an attributed CFString. The regular string is
 *    modified by the SetMenuItemTextAsCFString API, and the attributed
 *    string is modified using this API. If an attributed string is
 *    provided, it will be used when measuring and drawing the menu
 *    item text. However, the regular string (if provided) is always
 *    used for type-selection; this allows the attributed string to
 *    contain extra characters that should be ignored during
 *    type-selection.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to set.
 *    
 *    inText:
 *      The new attributed text for the menu item. May be NULL if the
 *      item no longer need attributed text.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> graemeg/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; inText: CFAttributedStringRef { can be NULL } ): OSStatus; external name '_HIMenuSetAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyAttributedItemText()
 *  
 *  Summary:
 *    Retrieves the attributed text of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to retrieve.
 *    
 *    outText:
 *      On exit, contains the item's attributed text, or NULL if the
 *      item has no attributed text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; var outText: CFAttributedStringRef ): OSStatus; external name '_HIMenuCopyAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  GetMenuItemIndent()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Sets the attributed text of a menu item.
 *  
 *  Discussion:
 *    A menu item may have two strings attached to it: a regular
 *    CFString and an attributed CFString. The regular string is
 *    modified by the SetMenuItemTextAsCFString API, and the attributed
 *    string is modified using this API. If an attributed string is
 *    provided, it will be used when measuring and drawing the menu
 *    item text. However, the regular string (if provided) is always
 *    used for type-selection; this allows the attributed string to
 *    contain extra characters that should be ignored during
 *    type-selection.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to set.
 *    
 *    inText:
 *      The new attributed text for the menu item. May be NULL if the
 *      item no longer need attributed text.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> graemeg/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; inText: CFAttributedStringRef { can be NULL } ): OSStatus; external name '_HIMenuSetAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyAttributedItemText()
 *  
 *  Summary:
 *    Retrieves the attributed text of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to retrieve.
 *    
 *    outText:
 *      On exit, contains the item's attributed text, or NULL if the
 *      item has no attributed text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; var outText: CFAttributedStringRef ): OSStatus; external name '_HIMenuCopyAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  GetMenuItemIndent()
 *  
 *  Summary:
 *    Gets the indent level of a menu item.
 *  
 *  Discussion:
 *    The indent level of an item is an amount of extra space added to
 *    the left of the item's icon or checkmark. The level is simply a
 *    number, starting at zero, which the Menu Manager multiplies by a
 *    constant to get the indent in pixels. The default indent level is
 *    zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose indent to retrieve.
 *    
 *    outIndent:
 *      On exit, the indent level of the item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemIndent( inMenu: MenuRef; inItem: MenuItemIndex; var outIndent: UInt32 ): OSStatus; external name '_GetMenuItemIndent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemIndent()
 *  
 *  Summary:
 *    Sets the indent level of a menu item.
 *  
 *  Discussion:
 *    The indent level of an item is an amount of extra space added to
 *    the left of the item's icon or checkmark. The level is simply a
 *    number, starting at zero, which the Menu Manager multiplies by a
 *    constant to get the indent in pixels. The default indent level is
 *    zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose indent to set.
 *    
 *    inIndent:
 *      The new indent level of the item.
 *  
 *  Availability:
=======
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemIndent( inMenu: MenuRef; inItem: MenuItemIndex; inIndent: UInt32 ): OSStatus; external name '_SetMenuItemIndent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemCommandKey()
 *  
 *  Summary:
 *    Gets the keyboard equivalent of a menu item.
 *  
 *  Discussion:
 *    A menu item's keyboard equivalent may be either a character code
 *    or a virtual keycode. An item's character code and virtual
 *    keycode are stored separately and may contain different values,
 *    but only one is used by the Menu Manager at any given time. When
 *    requesting a menu item's virtual keycode equivalent, you should
 *    first check that the item is using a virtual keycode by testing
 *    the kMenuItemAttrUseVirtualKey attribute for that item. If this
 *    attribute is not set, the item's virtual keycode is ignored by
 *    the Menu Manager. Note that zero is a valid virtual keycode, so
 *    you cannot test the returned keycode against zero to determine if
 *    the item is using a virtual keycode equivalent. You must test the
 *    kMenuItemAttrUseVirtualKey attribute.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose keyboard equivalent to retrieve.
 *    
 *    inGetVirtualKey:
 *      Indicates whether to retrieve the item's character code or
 *      virtual keycode equivalent.
 *    
 *    outKey:
 *      On exit, the keyboard equivalent of the item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemCommandKey( inMenu: MenuRef; inItem: MenuItemIndex; inGetVirtualKey: Boolean; var outKey: UInt16 ): OSStatus; external name '_GetMenuItemCommandKey';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemCommandKey()
 *  
 *  Summary:
 *    Sets the keyboard equivalent of a menu item.
 *  
 *  Discussion:
 *    A menu item's keyboard equivalent may be either a character code
 *    or a virtual keycode. The character code is always used to draw
 *    the item's keyboard equivalent in the menu, but either a char
 *    code or a virtual key code may be used for keyboard equivalent
 *    matching by MenuEvent and IsMenuKeyEvent, depending on whether
 *    the kMenuItemAttrUseVirtualKey item attribute is set. If
 *    SetMenuItemCommandKey is used to set the virtual keycode
 *    equivalent for a menu item, it also automatically sets the
 *    kMenuItemAttrUseVirtualKey item attribute. To make the menu item
 *    stop using the virtual keycode equivalent and use the character
 *    code equivalent instead, use ChangeMenuItemAttributes to clear
 *    the kMenuItemAttrUseVirtualKey item attribute.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose keyboard equivalent to set.
 *    
 *    inSetVirtualKey:
 *      Indicates whether to set the item's character code or virtual
 *      keycode equivalent. If setting a character code, the character
 *      is assumed to be in the application's text encoding.
 *    
 *    inKey:
 *      The item's new character code or virtual keycode equivalent.
 *      Note that the character code is not a Unicode value; rather,
 *      the character code is assumed to be expressed in the
 *      application's text encoding. In Mac OS X 10.3 and later, the
 *      Menu Manager also supports the following metavalues when
 *      inSetVirtualKey is false. When receiving one of these values,
 *      the Menu Manager automatically sets the item's keyboard
 *      equivalent glyph code to an appropriate value using
 *      SetMenuItemKeyGlyph. 
 *      
 *      NSF1FunctionKey..NSF15FunctionKey 
 *      NSHomeFunctionKey 
 *      kEnterCharCode 
 *      kEndCharCode 
 *      NSEndFunctionKey 
 *      kHelpCharCode 
 *      NSHelpFunctionKey 
 *      kBackspaceCharCode 
 *      kTabCharCode 
 *      kLineFeedCharCode 
 *      kReturnCharCode 
 *      kPageUpCharCode 
 *      NSPageUpFunctionKey 
 *      kPageDownCharCode 
 *      NSPageDownFunctionKey 
 *      NSBackTabCharacter 
 *      kEscapeCharCode 
 *      NSClearDisplayFunctionKey 
 *      kLeftArrowCharCode 
 *      NSLeftArrowFunctionKey 
 *      kRightArrowCharCode 
 *      NSRightArrowFunctionKey 
 *      kUpArrowCharCode 
 *      NSUpArrowFunctionKey 
 *      kDownArrowCharCode 
 *      NSDownArrowFunctionKey 
 *      kDeleteCharCode 
 *      
 *      In Mac OS X 10.4 and later, the Menu Manager also supports
 *      these values: 
 *      
 *      0x2190..0x2193 (left, up, right, and down-arrow)
 *       kSpaceCharCode 
 *      
 *      In Mac OS X 10.6 and later, the Menu Manager also supports
 *      these values: 
 *      NSF16FunctionKey..NSF19FunctionKey, NSDeleteFunctionKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemCommandKey( inMenu: MenuRef; inItem: MenuItemIndex; inSetVirtualKey: Boolean; inKey: UInt16 ): OSStatus; external name '_SetMenuItemCommandKey';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{ • Menu item views                                                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  HIMenuSetItemViewClass()
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======
 *  
 *  Summary:
 *    Sets the HIObject class ID and initialization event that will be
 *    used to create a custom HIView for a menu item.
 *  
 *  Discussion:
 *    When a menu item has a custom HIObject class ID associated with
 *    it, the standard menu view automatically creates an HIView using
 *    that class ID as necessary and inserts the view as a subview of
 *    the standard menu view. The item view is send a
 *    kEventControlGetOptimalBounds event to request that it calculate
 *    its size, and if this size is larger than the size that the item
 *    would normally have, it is used instead. The view's frame is
 *    automatically positioned to match the frame of the item. 
 *    
 *    An item view should be an ordinary subclass of HIView. The item
 *    view is responsible for entirely replacing the standard menu item
 *    content; the standard content will not be drawn. When created, an
 *    item view's kEventHIObjectInitialize event will have two extra
 *    parameters, kEventParamMenuRef and kEventParamMenuItemIndex, if
 *    the view is intended for use in a menu. Note that a view should
 *    not cache the item index from the Init event, because the item
 *    index can change over time if menu items are inserted or deleted;
 *    instead, a view can determine its current menu item index at any
 *    time (except during Construct or Init handlers) by calling
 *    HIMenuItemViewGetEnclosingMenuItem. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Sets the HIObject class ID and initialization event that will be
 *    used to create a custom HIView for a menu item.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    When a menu item has a custom HIObject class ID associated with
 *    it, the standard menu view automatically creates an HIView using
 *    that class ID as necessary and inserts the view as a subview of
 *    the standard menu view. The item view is send a
 *    kEventControlGetOptimalBounds event to request that it calculate
 *    its size, and if this size is larger than the size that the item
 *    would normally have, it is used instead. The view's frame is
 *    automatically positioned to match the frame of the item. 
 *    
 *    An item view should be an ordinary subclass of HIView. The item
 *    view is responsible for entirely replacing the standard menu item
 *    content; the standard content will not be drawn. When created, an
 *    item view's kEventHIObjectInitialize event will have two extra
 *    parameters, kEventParamMenuRef and kEventParamMenuItemIndex, if
 *    the view is intended for use in a menu. Note that a view should
 *    not cache the item index from the Init event, because the item
 *    index can change over time if menu items are inserted or deleted;
 *    instead, a view can determine its current menu item index at any
 *    time (except during Construct or Init handlers) by calling
 *    HIMenuItemViewGetEnclosingMenuItem. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
>>>>>>> graemeg/cpstrnew
=======
 *  Parameters:
 *    
 *    inMenu:
 *      The menu for which to set the item view class.
 *    
 *    inItem:
 *      The menu item for which to set the item view class.
 *    
 *    inClassID:
 *      The class ID of the menu item's custom view. May be NULL to
 *      clear the item's custom view class ID; however, any item view
 *      that had already been created for the item will remain embedded
 *      in the menu content view and will be used to measure and draw
 *      the item.
 *    
 *    inInitEvent:
 *      The initialization event of the menu item's custom view. May be
 *      NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; inClassID: CFStringRef { can be NULL }; inInitEvent: EventRef { can be NULL } ): OSStatus; external name '_HIMenuSetItemViewClass';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyItemViewClass()
 *  
 *  Summary:
 *    Returns the HIObject class ID and initialization event that are
 *    used to create a custom HIView for a menu item.
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Sets the HIObject class ID and initialization event that will be
 *    used to create a custom HIView for a menu item.
 *  
 *  Discussion:
 *    When a menu item has a custom HIObject class ID associated with
 *    it, the standard menu view automatically creates an HIView using
 *    that class ID as necessary and inserts the view as a subview of
 *    the standard menu view. The item view is send a
 *    kEventControlGetOptimalBounds event to request that it calculate
 *    its size, and if this size is larger than the size that the item
 *    would normally have, it is used instead. The view's frame is
 *    automatically positioned to match the frame of the item. 
 *    
 *    An item view should be an ordinary subclass of HIView. The item
 *    view is responsible for entirely replacing the standard menu item
 *    content; the standard content will not be drawn. When created, an
 *    item view's kEventHIObjectInitialize event will have two extra
 *    parameters, kEventParamMenuRef and kEventParamMenuItemIndex, if
 *    the view is intended for use in a menu. Note that a view should
 *    not cache the item index from the Init event, because the item
 *    index can change over time if menu items are inserted or deleted;
 *    instead, a view can determine its current menu item index at any
 *    time (except during Construct or Init handlers) by calling
 *    HIMenuItemViewGetEnclosingMenuItem. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Sets the HIObject class ID and initialization event that will be
 *    used to create a custom HIView for a menu item.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    When a menu item has a custom HIObject class ID associated with
 *    it, the standard menu view automatically creates an HIView using
 *    that class ID as necessary and inserts the view as a subview of
 *    the standard menu view. The item view is send a
 *    kEventControlGetOptimalBounds event to request that it calculate
 *    its size, and if this size is larger than the size that the item
 *    would normally have, it is used instead. The view's frame is
 *    automatically positioned to match the frame of the item. 
 *    
 *    An item view should be an ordinary subclass of HIView. The item
 *    view is responsible for entirely replacing the standard menu item
 *    content; the standard content will not be drawn. When created, an
 *    item view's kEventHIObjectInitialize event will have two extra
 *    parameters, kEventParamMenuRef and kEventParamMenuItemIndex, if
 *    the view is intended for use in a menu. Note that a view should
 *    not cache the item index from the Init event, because the item
 *    index can change over time if menu items are inserted or deleted;
 *    instead, a view can determine its current menu item index at any
 *    time (except during Construct or Init handlers) by calling
 *    HIMenuItemViewGetEnclosingMenuItem. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
=======
 *  Parameters:
 *    
 *    inMenu:
 *      The menu for which to set the item view class.
 *    
 *    inItem:
 *      The menu item for which to set the item view class.
 *    
 *    inClassID:
 *      The class ID of the menu item's custom view. May be NULL to
 *      clear the item's custom view class ID; however, any item view
 *      that had already been created for the item will remain embedded
 *      in the menu content view and will be used to measure and draw
 *      the item.
 *    
 *    inInitEvent:
 *      The initialization event of the menu item's custom view. May be
 *      NULL.
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu for which to set the item view class.
 *    
 *    inItem:
 *      The menu item for which to set the item view class.
 *    
 *    inClassID:
 *      The class ID of the menu item's custom view. May be NULL to
 *      clear the item's custom view class ID; however, any item view
 *      that had already been created for the item will remain embedded
 *      in the menu content view and will be used to measure and draw
 *      the item.
 *    
 *    inInitEvent:
 *      The initialization event of the menu item's custom view. May be
 *      NULL.
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu for which to retrieve the item view class.
 *    
 *    inItem:
 *      The menu item for which to retrieve the item view class.
 *    
 *    outClassID:
 *      On exit, contains the item's custom view class ID, or NULL if
 *      none. The caller should release this string.
 *    
 *    outInitEvent:
 *      On exit, contains the item's custom view initialization event,
 *      or NULL if none. The caller should release this event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function HIMenuSetItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; inClassID: CFStringRef { can be NULL }; inInitEvent: EventRef { can be NULL } ): OSStatus; external name '_HIMenuSetItemViewClass';
=======
function HIMenuCopyItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; var outClassID: CFStringRef; outInitEvent: EventRefPtr { can be NULL } ): OSStatus; external name '_HIMenuCopyItemViewClass';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
<<<<<<< HEAD
 *  HIMenuCopyItemViewClass()
 *  
 *  Summary:
 *    Returns the HIObject class ID and initialization event that are
 *    used to create a custom HIView for a menu item.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
 *  HIMenuGetItemView()
 *  
 *  Summary:
 *    Returns the HIViewRef that will be used to draw a menu item, if
 *    any.
 *  
 *  Discussion:
 *    If the item view has not yet been created, the Menu Manager will
 *    create the item view using the view class ID and initialization
 *    event specified by the HIMenuSetItemViewClass API. If the menu
 *    item does not have a custom view associated with it, noErr is
 *    returned but outView is set to NULL. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      The menu for which to set the item view class.
 *    
 *    inItem:
 *      The menu item for which to set the item view class.
 *    
 *    inClassID:
 *      The class ID of the menu item's custom view. May be NULL to
 *      clear the item's custom view class ID; however, any item view
 *      that had already been created for the item will remain embedded
 *      in the menu content view and will be used to measure and draw
 *      the item.
 *    
 *    inInitEvent:
 *      The initialization event of the menu item's custom view. May be
 *      NULL.
=======
=======
>>>>>>> graemeg/cpstrnew
 *      The menu for which to retrieve the item view class.
 *    
 *    inItem:
 *      The menu item for which to retrieve the item view class.
 *    
 *    outClassID:
 *      On exit, contains the item's custom view class ID, or NULL if
 *      none. The caller should release this string.
 *    
 *    outInitEvent:
 *      On exit, contains the item's custom view initialization event,
 *      or NULL if none. The caller should release this event.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
 *      The menu.
 *    
 *    inItem:
 *      The menu item.
 *    
 *    inMenuType:
 *    
 *    outView:
 *      On exit, contains the view. May be set to NULL if the item does
 *      not have a custom item view. The caller should not release this
 *      view.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function HIMenuSetItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; inClassID: CFStringRef { can be NULL }; inInitEvent: EventRef { can be NULL } ): OSStatus; external name '_HIMenuSetItemViewClass';
=======
function HIMenuCopyItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; var outClassID: CFStringRef; outInitEvent: EventRefPtr { can be NULL } ): OSStatus; external name '_HIMenuCopyItemViewClass';
>>>>>>> graemeg/cpstrnew
=======
function HIMenuCopyItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; var outClassID: CFStringRef; outInitEvent: EventRefPtr { can be NULL } ): OSStatus; external name '_HIMenuCopyItemViewClass';
>>>>>>> graemeg/cpstrnew
=======
function HIMenuGetItemView( inMenu: MenuRef; inItem: MenuItemIndex; inMenuType: ThemeMenuType; var outView: HIViewRef ): OSStatus; external name '_HIMenuGetItemView';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  HIMenuCopyItemViewClass()
 *  
 *  Summary:
 *    Returns the HIObject class ID and initialization event that are
 *    used to create a custom HIView for a menu item.
=======
=======
>>>>>>> graemeg/cpstrnew
 *  HIMenuGetItemView()
 *  
 *  Summary:
 *    Returns the HIViewRef that will be used to draw a menu item, if
 *    any.
 *  
 *  Discussion:
 *    If the item view has not yet been created, the Menu Manager will
 *    create the item view using the view class ID and initialization
 *    event specified by the HIMenuSetItemViewClass API. If the menu
 *    item does not have a custom view associated with it, noErr is
 *    returned but outView is set to NULL. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
 *  HIMenuItemViewGetEnclosingMenuItem()
 *  
 *  Summary:
 *    Locates the menu item that contains a menu item view or one of
 *    its subviews.
 *  
 *  Discussion:
 *    A custom HIView may behave differently depending on whether it's
 *    embedded in a menu or a regular window. This API will tell the
 *    view whether it's contained in a menu or a window. Note that this
 *    API will always return false if called from a view's
 *    kEventHIObjectConstruct or kEventHIObjectInitialize event
 *    handler, even if the view is being created for use in a menu; at
 *    Construct and Init time, the Menu Manager has not yet created the
 *    binding between the view and the item. A view can check for
 *    kEventParamMenuRef and kEventParamMenuItemIndex parameters in its
 *    kEventHIObjectInitialize handler to determine at Init time
 *    whether the view is being placed into a menu. 
 *    
 *    This API may be passed a menu item view or any subview of the
 *    item view. If passed a subview, it will search upwards through
 *    the view hierarchy to look for an item view.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inMenu:
<<<<<<< HEAD
<<<<<<< HEAD
 *      The menu for which to retrieve the item view class.
 *    
 *    inItem:
 *      The menu item for which to retrieve the item view class.
 *    
 *    outClassID:
 *      On exit, contains the item's custom view class ID, or NULL if
 *      none. The caller should release this string.
 *    
 *    outInitEvent:
 *      On exit, contains the item's custom view initialization event,
 *      or NULL if none. The caller should release this event.
=======
=======
>>>>>>> graemeg/cpstrnew
 *      The menu.
 *    
 *    inItem:
 *      The menu item.
 *    
 *    inMenuType:
 *    
 *    outView:
 *      On exit, contains the view. May be set to NULL if the item does
 *      not have a custom item view. The caller should not release this
 *      view.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
function HIMenuCopyItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; var outClassID: CFStringRef; outInitEvent: EventRefPtr { can be NULL } ): OSStatus; external name '_HIMenuCopyItemViewClass';
=======
function HIMenuGetItemView( inMenu: MenuRef; inItem: MenuItemIndex; inMenuType: ThemeMenuType; var outView: HIViewRef ): OSStatus; external name '_HIMenuGetItemView';
>>>>>>> graemeg/cpstrnew
=======
function HIMenuGetItemView( inMenu: MenuRef; inItem: MenuItemIndex; inMenuType: ThemeMenuType; var outView: HIViewRef ): OSStatus; external name '_HIMenuGetItemView';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  HIMenuGetItemView()
 *  
 *  Summary:
 *    Returns the HIViewRef that will be used to draw a menu item, if
 *    any.
 *  
 *  Discussion:
 *    If the item view has not yet been created, the Menu Manager will
 *    create the item view using the view class ID and initialization
 *    event specified by the HIMenuSetItemViewClass API. If the menu
 *    item does not have a custom view associated with it, noErr is
 *    returned but outView is set to NULL. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
=======
=======
>>>>>>> graemeg/cpstrnew
 *  HIMenuItemViewGetEnclosingMenuItem()
 *  
 *  Summary:
 *    Locates the menu item that contains a menu item view or one of
 *    its subviews.
 *  
 *  Discussion:
 *    A custom HIView may behave differently depending on whether it's
 *    embedded in a menu or a regular window. This API will tell the
 *    view whether it's contained in a menu or a window. Note that this
 *    API will always return false if called from a view's
 *    kEventHIObjectConstruct or kEventHIObjectInitialize event
 *    handler, even if the view is being created for use in a menu; at
 *    Construct and Init time, the Menu Manager has not yet created the
 *    binding between the view and the item. A view can check for
 *    kEventParamMenuRef and kEventParamMenuItemIndex parameters in its
 *    kEventHIObjectInitialize handler to determine at Init time
 *    whether the view is being placed into a menu. 
 *    
 *    This API may be passed a menu item view or any subview of the
 *    item view. If passed a subview, it will search upwards through
 *    the view hierarchy to look for an item view.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
 *    inMenu:
 *      The menu.
 *    
 *    inItem:
 *      The menu item.
 *    
 *    inMenuType:
 *    
 *    outView:
 *      On exit, contains the view. May be set to NULL if the item does
 *      not have a custom item view. The caller should not release this
 *      view.
=======
 *    inItemView:
 *      The item view.
 *    
 *    outMenu:
 *      On exit, contains the menu in which the view is embedded. May
 *      be NULL if you don't need the menu.
 *    
 *    outItem:
 *      On exit, contains the menu item that the view is implementing.
 *      May be NULL if you don't need the item. Note that the item
 *      index that is returned may change over time if menu items are
 *      inserted or deleted; generally, a view should not cache the
 *      item index, but should call this API each time that it needs to
 *      know its current item.
 *  
 *  Result:
 *    true if the view is embedded in a menu, or false if not.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function HIMenuGetItemView( inMenu: MenuRef; inItem: MenuItemIndex; inMenuType: ThemeMenuType; var outView: HIViewRef ): OSStatus; external name '_HIMenuGetItemView';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuItemViewGetEnclosingMenuItem()
 *  
 *  Summary:
 *    Locates the menu item that contains a menu item view or one of
 *    its subviews.
 *  
 *  Discussion:
 *    A custom HIView may behave differently depending on whether it's
 *    embedded in a menu or a regular window. This API will tell the
 *    view whether it's contained in a menu or a window. Note that this
 *    API will always return false if called from a view's
 *    kEventHIObjectConstruct or kEventHIObjectInitialize event
 *    handler, even if the view is being created for use in a menu; at
 *    Construct and Init time, the Menu Manager has not yet created the
 *    binding between the view and the item. A view can check for
 *    kEventParamMenuRef and kEventParamMenuItemIndex parameters in its
 *    kEventHIObjectInitialize handler to determine at Init time
 *    whether the view is being placed into a menu. 
 *    
 *    This API may be passed a menu item view or any subview of the
 *    item view. If passed a subview, it will search upwards through
 *    the view hierarchy to look for an item view.
=======
 *    inItemView:
 *      The item view.
 *    
 *    outMenu:
 *      On exit, contains the menu in which the view is embedded. May
 *      be NULL if you don't need the menu.
 *    
 *    outItem:
 *      On exit, contains the menu item that the view is implementing.
 *      May be NULL if you don't need the item. Note that the item
 *      index that is returned may change over time if menu items are
 *      inserted or deleted; generally, a view should not cache the
 *      item index, but should call this API each time that it needs to
 *      know its current item.
>>>>>>> graemeg/cpstrnew
 *  
 *  Result:
 *    true if the view is embedded in a menu, or false if not.
 *  
 *  Parameters:
 *    
 *    inItemView:
 *      The item view.
 *    
 *    outMenu:
 *      On exit, contains the menu in which the view is embedded. May
 *      be NULL if you don't need the menu.
 *    
 *    outItem:
 *      On exit, contains the menu item that the view is implementing.
 *      May be NULL if you don't need the item. Note that the item
 *      index that is returned may change over time if menu items are
 *      inserted or deleted; generally, a view should not cache the
 *      item index, but should call this API each time that it needs to
 *      know its current item.
 *  
=======
 *    inItemView:
 *      The item view.
 *    
 *    outMenu:
 *      On exit, contains the menu in which the view is embedded. May
 *      be NULL if you don't need the menu.
 *    
 *    outItem:
 *      On exit, contains the menu item that the view is implementing.
 *      May be NULL if you don't need the item. Note that the item
 *      index that is returned may change over time if menu items are
 *      inserted or deleted; generally, a view should not cache the
 *      item index, but should call this API each time that it needs to
 *      know its current item.
 *  
>>>>>>> graemeg/cpstrnew
 *  Result:
 *    true if the view is embedded in a menu, or false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
=======
>>>>>>> origin/cpstrnew
function HIMenuItemViewGetEnclosingMenuItem( inItemView: HIViewRef; outMenu: MenuRefPtr { can be NULL }; outItem: MenuItemIndexPtr { can be NULL } ): Boolean; external name '_HIMenuItemViewGetEnclosingMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuItemViewCancelMenuTracking()
 *  
 *  Summary:
 *    Cancels menu tracking for the menu containing an item view.
 *  
 *  Discussion:
 *    In order to call the CancelMenuTracking API, you must know the
 *    menu that is the root for the menu tracking session. This root
 *    menu is often not available to a custom menu item view. This API
 *    allows a menu item view to cancel tracking without knowing the
 *    root menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inItemView:
 *      The item view.
 *    
 *    inImmediate:
 *      Whether the open menus should disappear immediately or fade out.
 *    
 *    inDismissalReason:
 *      Why the menu is being dismissed; this value will be added to
 *      the kEventMenuEndTracking event. If zero,
 *      kHIMenuDismissedByCancelMenuTracking is added to the
 *      EndTracking event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuItemViewCancelMenuTracking( inItemView: HIViewRef; inImmediate: Boolean; inDismissalReason: UInt32 ): OSStatus; external name '_HIMenuItemViewCancelMenuTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{ • Properties  (Mac OS 8.5 and later)                                                 }
{ With the following property APIs, you can attach any piece of data you'd like to a   }
{ menu or menu item. Passing zero for the item number parameter indicates you'd like   }
{ to attach the data to the menu itself, and not to any specific menu item.            }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

const
	kMenuPropertyPersistent = $00000001; { whether this property gets saved when archiving the menu; however, ignored in Mac OS X 10.4}

{$ifc not TARGET_CPU_64}
{
 *  GetMenuItemProperty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuItemProperty( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; bufferSize: ByteCount; var actualSize: ByteCount; propertyBuffer: UnivPtr ): OSStatus; external name '_GetMenuItemProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemPropertySize()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuItemPropertySize( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; var size: ByteCount ): OSStatus; external name '_GetMenuItemPropertySize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemProperty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function SetMenuItemProperty( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; propertySize: ByteCount; propertyData: {const} UnivPtr ): OSStatus; external name '_SetMenuItemProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveMenuItemProperty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function RemoveMenuItemProperty( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType ): OSStatus; external name '_RemoveMenuItemProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemPropertyAttributes()
 *  
 *  Summary:
 *    Gets the attributes of a menu item property.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    item:
 *      The menu item.
 *    
 *    propertyCreator:
 *      The creator code of the property.
 *    
 *    propertyTag:
 *      The property tag.
 *    
 *    attributes:
 *      On exit, contains the attributes of the property.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemPropertyAttributes( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; var attributes: OptionBits ): OSStatus; external name '_GetMenuItemPropertyAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ChangeMenuItemPropertyAttributes()
 *  
 *  Summary:
 *    Changes the attributes of a menu item property.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    item:
 *      The menu item.
 *    
 *    propertyCreator:
 *      The creator code of the property.
 *    
 *    propertyTag:
 *      The property tag.
 *    
 *    attributesToSet:
 *      The attributes to add to the menu item property.
 *    
 *    attributesToClear:
 *      The attributes to remove from the menu item property.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ChangeMenuItemPropertyAttributes( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; attributesToSet: OptionBits; attributesToClear: OptionBits ): OSStatus; external name '_ChangeMenuItemPropertyAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Attributes (Carbon and later)                                                     }
{  Each menu and menu item has attribute flags.                                        }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  GetMenuAttributes()
 *  
 *  Summary:
 *    Gets the attributes of a menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    outAttributes:
 *      On exit, contains the attributes of the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuAttributes( menu: MenuRef; var outAttributes: MenuAttributes ): OSStatus; external name '_GetMenuAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ChangeMenuAttributes()
 *  
 *  Summary:
 *    Changes the attributes of a menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    setTheseAttributes:
 *      The attributes to add to the menu.
 *    
 *    clearTheseAttributes:
 *      The attributes to remove from the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ChangeMenuAttributes( menu: MenuRef; setTheseAttributes: MenuAttributes; clearTheseAttributes: MenuAttributes ): OSStatus; external name '_ChangeMenuAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemAttributes()
 *  
 *  Summary:
 *    Gets the attributes of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    item:
 *      The menu item.
 *    
 *    outAttributes:
 *      On exit, contains the attributes of the menu item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemAttributes( menu: MenuRef; item: MenuItemIndex; var outAttributes: MenuItemAttributes ): OSStatus; external name '_GetMenuItemAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ChangeMenuItemAttributes()
 *  
 *  Summary:
 *    Changes the attributes of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    item:
 *      The menu item.
 *    
 *    setTheseAttributes:
 *      The attributes to add to the menu item.
 *    
 *    clearTheseAttributes:
 *      The attributes to remove from the menu item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ChangeMenuItemAttributes( menu: MenuRef; item: MenuItemIndex; setTheseAttributes: MenuItemAttributes; clearTheseAttributes: MenuItemAttributes ): OSStatus; external name '_ChangeMenuItemAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Mass menu item enabling and disabling (Carbon and later)                          }
{  Useful when rewriting code that modifies the enableFlags field directly.            }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  DisableAllMenuItems()
 *  
 *  Summary:
 *    Disables all items in a menu.
 *  
 *  Discussion:
 *    This API is equivalent to pre-Carbon code that masked the
 *    enableFlags field of the MenuInfo with 0x01. It disables all
 *    items (including items past item 31) but does not affect the
 *    state of the menu title.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose items to disable.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisableAllMenuItems( theMenu: MenuRef ); external name '_DisableAllMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EnableAllMenuItems()
 *  
 *  Summary:
 *    Enables all items in a menu.
 *  
 *  Discussion:
 *    This API is equivalent to pre-Carbon code that or'd the
 *    enableFlags field of the MenuInfo with 0xFFFFFFFE. It enables all
 *    items (including items past item 31) but does not affect the
 *    state of the menu title.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose items to enable.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EnableAllMenuItems( theMenu: MenuRef ); external name '_EnableAllMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MenuHasEnabledItems()
 *  
 *  Summary:
 *    Determines if any items in a menu are enabled.
 *  
 *  Discussion:
 *    This API is equivalent to pre-Carbon code that compared the
 *    enableFlags field of the MenuInfo with 0. It checks the enable
 *    state of all items to see if any are enabled, but ignores the
 *    state of the menu title. It will return true even if the menu
 *    title is disabled.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose items to examine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MenuHasEnabledItems( theMenu: MenuRef ): Boolean; external name '_MenuHasEnabledItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu tracking status (Carbon and later)                                           }
{  Get info about the selected menu item during menu tracking. Replaces direct access  }
{  to low-mem globals that previously held this info.                                  }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

{
 *  GetMenuTrackingData()
 *  
 *  Summary:
 *    Gets information about the menu currently selected by the user.
 *  
 *  Discussion:
 *    This API replaces direct access to the low-memory globals
 *    TopMenuItem, AtMenuBottom, MenuDisable, and mbSaveLoc. It is only
 *    valid to call this API while menu tracking is occurring. This API
 *    will most commonly be used by custom MDEFs.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu about which to get tracking information. May be NULL
 *      to get information about the menu that the user is currently
 *      selecting. If the menu is not currently open, menuNotFoundErr
 *      is returned.
 *    
 *    outData:
 *      On exit, contains tracking data about the menu. On CarbonLib,
 *      the itemRect field is not supported and is always set to an
 *      empty rect.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuTrackingData( theMenu: MenuRef; var outData: MenuTrackingData ): OSStatus; external name '_GetMenuTrackingData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  GetMenuType()
 *  
 *  Summary:
 *    Gets the display type (pulldown, hierarchical, or popup) of a
 *    menu.
 *  
 *  Discussion:
 *    This API may only be called when the menu is displayed. If the
 *    menu is not currently open, an error is returned. The display
 *    type of a menu may vary from one menu tracking session to
 *    another; for example, the same menu might be displayed as a
 *    pulldown menu and as a popup menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose type to get.
 *    
 *    outType:
 *      On exit, the type of the menu. The returned value will be one
 *      of the ThemeMenuType constants: kThemeMenuTypePullDown, PopUp,
 *      or Hierarchical. The kThemeMenuTypeInactive bit will never be
 *      set.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuType( theMenu: MenuRef; var outType: UInt16 ): OSStatus; external name '_GetMenuType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Universal command ID access (Carbon and later)                                    }
{  These APIs allow you to operate on menu items strictly by command ID, with no       }
{  knowledge of a menu item's index.                                                   }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  CountMenuItemsWithCommandID()
 *  
 *  Summary:
 *    Counts the menu items with a specified command ID.
 *  
 *  Discussion:
 *    In CarbonLib 1.0.x and 1.1, this API always returns zero or one;
 *    it stops after finding the first menu item with the specified
 *    command ID. In CarbonLib 1.2 and Mac OS X 10.0 and later, it
 *    counts all menu items with the specified command ID. In Mac OS X
 *    10.0 and CarbonLib 1.0 through 1.4, this API only searches
 *    top-level menus (menus visible in the menubar) and submenus of
 *    top-level menus. It does not search hierarchical menus that are
 *    inserted in the menubar but are not submenus of a top-level menus
 *    (for example, it does not search menus that are inserted for use
 *    in a popup menu control). In Mac OS X 10.1 and CarbonLib 1.5 and
 *    later, this API also searches inserted hierarchical menus.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for items with the
 *      specified command ID. Pass NULL to hegin searching with the
 *      root menu. The search will descend into all submenus of this
 *      menu.
 *    
 *    inCommandID:
 *      The command ID for which to search.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CountMenuItemsWithCommandID( inMenu: MenuRef; inCommandID: MenuCommand ): ItemCount; external name '_CountMenuItemsWithCommandID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetIndMenuItemWithCommandID()
 *  
 *  Summary:
 *    Finds a menu item with a specified command ID.
 *  
 *  Discussion:
 *    This API searches the specified menu and its submenus for the
 *    n'th menu item with the specified command ID. In CarbonLib 1.0.x
 *    and 1.1, only the first menu item is returned. In CarbonLib 1.2
 *    and Mac OS X 10.0 and later, this API iterates over all menu
 *    items with the specified command ID. In Mac OS X 10.0 and
 *    CarbonLib 1.0 through 1.4, this API only searches top-level menus
 *    (menus visible in the menubar) and submenus of top-level menus.
 *    It does not search hierarchical menus that are inserted in the
 *    menubar but are not submenus of a top-level menus (for example,
 *    it does not search menus that are inserted for use in a popup
 *    menu control). In Mac OS X 10.1 and CarbonLib 1.5 and later, this
 *    API also searches inserted hierarchical menus.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for items with the
 *      specified command ID. Pass NULL to hegin searching with the
 *      root menu. The search will descend into all submenus of this
 *      menu.
 *    
 *    inCommandID:
 *      The command ID for which to search.
 *    
 *    inItemIndex:
 *      The 1-based index of the menu item to retrieve. In CarbonLib
 *      1.0.x and 1.1, this parameter must be 1. In CarbonLib 1.2 and
 *      Mac OS X 10.0, this parameter may vary from 1 to the number of
 *      menu items with the specified command ID.
 *    
 *    outMenu:
 *      On exit, the menu containing the menu item with the specified
 *      command ID.
 *    
 *    outIndex:
 *      On exit, the item index of the menu item with the specified
 *      command ID.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetIndMenuItemWithCommandID( inMenu: MenuRef; inCommandID: MenuCommand; inItemIndex: UInt32; outMenu: MenuRefPtr { can be NULL }; outIndex: MenuItemIndexPtr { can be NULL } ): OSStatus; external name '_GetIndMenuItemWithCommandID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EnableMenuCommand()
 *  
 *  Summary:
 *    Enables the menu item with a specified command ID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to be enabled. If more than one
 *      item has this command ID, only the first will be enabled.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EnableMenuCommand( inMenu: MenuRef; inCommandID: MenuCommand ); external name '_EnableMenuCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisableMenuCommand()
 *  
 *  Summary:
 *    Disables the menu item with a specified command ID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to be disabled. If more than
 *      one item has this command ID, only the first will be disabled.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisableMenuCommand( inMenu: MenuRef; inCommandID: MenuCommand ); external name '_DisableMenuCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuCommandEnabled()
 *  
 *  Summary:
 *    Determines if the menu item with a specified command ID is
 *    enabled.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to examine. If more than one
 *      item has this command ID, only the first will be examined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuCommandEnabled( inMenu: MenuRef; inCommandID: MenuCommand ): Boolean; external name '_IsMenuCommandEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuCommandMark()
 *  
 *  Summary:
 *    Locates the menu item with a specified command ID and sets its
 *    mark character.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to be modified. If more than
 *      one item has this command ID, only the first will be modified.
 *    
 *    inMark:
 *      The new mark character. Although the type of this parameter is
 *      UniChar, currently only the low byte of this character will be
 *      used as the mark character, and it is interpreted using the
 *      application’s text encoding.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuCommandMark( inMenu: MenuRef; inCommandID: MenuCommand; inMark: UniChar ): OSStatus; external name '_SetMenuCommandMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuCommandMark()
 *  
 *  Summary:
 *    Locates the menu item with a specified command ID and returns its
 *    mark character.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to be examined. If more than
 *      one item has this command ID, only the first will be examined.
 *    
 *    outMark:
 *      On exit, the menu item's mark character.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuCommandMark( inMenu: MenuRef; inCommandID: MenuCommand; var outMark: UniChar ): OSStatus; external name '_GetMenuCommandMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuCommandProperty()
 *  
 *  Summary:
 *    Retrives property data for a menu item with a specified command
 *    ID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item containing the property. If
 *      more than one item has this command ID, only the first will be
 *      used.
 *    
 *    inPropertyCreator:
 *      The property creator.
 *    
 *    inPropertyTag:
 *      The property tag.
 *    
 *    inBufferSize:
 *      The size of the output buffer, in bytes.
 *    
 *    outActualSize:
 *      On exit, contains the actual size of the property data. May be
 *      NULL if you do not need this information.
 *    
 *    inPropertyBuffer:
 *      The address of a buffer in which to place the property data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuCommandProperty( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType; inBufferSize: ByteCount; outActualSize: ByteCountPtr { can be NULL }; inPropertyBuffer: UnivPtr ): OSStatus; external name '_GetMenuCommandProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuCommandPropertySize()
 *  
 *  Summary:
 *    Retrives the size of property data for a menu item with a
 *    specified command ID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item containing the property. If
 *      more than one item has this command ID, only the first will be
 *      used.
 *    
 *    inPropertyCreator:
 *      The property creator.
 *    
 *    inPropertyTag:
 *      The property tag.
 *    
 *    outSize:
 *      On exit, contains the size of the property data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuCommandPropertySize( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType; var outSize: ByteCount ): OSStatus; external name '_GetMenuCommandPropertySize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuCommandProperty()
 *  
 *  Summary:
 *    Sets property data for a menu item with a specified command ID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item that will receive the property.
 *      If more than one item has this command ID, only the first will
 *      be modified.
 *    
 *    inPropertyCreator:
 *      The property creator.
 *    
 *    inPropertyTag:
 *      The property tag.
 *    
 *    inPropertySize:
 *      The size of the property data, in bytes.
 *    
 *    inPropertyData:
 *      The address of the property data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuCommandProperty( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType; inPropertySize: ByteCount; inPropertyData: {const} UnivPtr ): OSStatus; external name '_SetMenuCommandProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveMenuCommandProperty()
 *  
 *  Summary:
 *    Removes a property from a menu item with a specified command ID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item from which the property will be
 *      removed. If more than one item has this command ID, only the
 *      first will be modified.
 *    
 *    inPropertyCreator:
 *      The property creator.
 *    
 *    inPropertyTag:
 *      The property tag.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveMenuCommandProperty( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType ): OSStatus; external name '_RemoveMenuCommandProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuItemData()
 *  
 *  Summary:
 *    Returns multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to retrieve many attributes of a menu item
 *    simultaneously; for example, it might be used by a menu
 *    definition function that needs to know how to draw a menu item.
 *    It is more efficient to use this function than to use the
 *    accessor functions for the individual attributes of the menu.
 *    This function returns a copy of the data in the menu, so any data
 *    in the MenuItemDataRec that is dynamically allocated (for
 *    example, the CFString item text) should be released by the caller.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu from which to copy data. If inIsCommandID is true, you
 *      may pass NULL for this parameter to search for an item in the
 *      root menu; if inIsCommandID is false, this parameter must be a
 *      valid MenuRef.
 *    
 *    inItem:
 *      The item or command ID from which to copy data.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and data is copied from the first item in the
 *      menu with that command ID. If inIsCommandID is false, the
 *      inItem parameter is interpreted as a menu item index, and data
 *      is copied for that item in the specified menu.
 *    
 *    ioData:
 *      Data is copied from the item and placed here. On entry, the
 *      whichData field of this structure should be initialized to
 *      indicate which data the caller would like returned. Individual
 *      fields of the MenuItemDataRec structure may require
 *      pre-initialization also; see the individual MenuItemDataFlags
 *      documentation for details.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuItemData( inMenu: MenuRef { can be NULL }; inItem: MenuItemID; inIsCommandID: Boolean; ioData: MenuItemDataPtr ): OSStatus; external name '_CopyMenuItemData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemData()
 *  
 *  Summary:
 *    Sets multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to set many attributes of a menu item
 *    simultaneously. It is more efficient to use this function than to
 *    use the accessor functions for the individual attributes of the
 *    menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to modify.
 *    
 *    inItem:
 *      The item or command ID to modify.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and the first item in the menu with that
 *      command ID. is modified. If inIsCommandID is false, the inItem
 *      parameter is interpreted as a menu item index, and the item
 *      with that index in the specified menu is modified.
 *    
 *    inData:
 *      The data to set. The caller should set the whichData field of
 *      this structure to indicate which data should be set. Only the
 *      fields of the structure corresponding to the non-zero whichData
 *      flags must be initialized; other fields are ignored.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemData( inMenu: MenuRef; inItem: MenuItemID; inIsCommandID: Boolean; const (*var*) inData: MenuItemDataRec ): OSStatus; external name '_SetMenuItemData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Dynamic menu item support (available in CarbonLib 1.1 and Mac OS X)               }
{  Dynamic menu item support allows a menu item to be redrawn while the menu is open   }
{  and visible to the user. Carbon contains automatic support for dynamic items based  }
{  on keyboard modifier state. If you need to implement your own variable item state   }
{  based on other system state, you can use these APIs to implement it.                }
{  To use the built-in support for dynamic items, you should create a menu containing  }
{  several contiguous items with the same command key but different text and modifier  }
{  keys. For example, you might have:                                                  }
{      Close       cmd-W                                                               }
{      Close All   cmd-option-W                                                        }
{  The Menu Manager automatically determines a dynamic menu group using the base       }
{  command key of a dynamic menu item ('W' in this case). Only a single item from      }
{  a dynamic group is visible at any time; the other items are hidden. The Menu        }
{  Manager uses the current keyboard modifiers to determine which item is visible.     }
{  It is also possible to have a dynamic menu item group that does not have command    }
{  keys, but only modifiers; for example, in the Finder's View menu, the Clean Up      }
{  and Clean Up Selection items have no command key, but are a dynamic menu item       }
{  group that changes visibility based on the Option modifier key.                     }
{  In this example, in your MENU resource, you would create the Close and Close All    }
{  items and give them each the letter 'W' as the command key; using an associated     }
{  xmnu resource, you would specify kMenuOptionModifier as the modifier for the        }
{  Close All item. You can do the same thing using InterfacerBuilder and nib-based     }
{  menus.                                                                              }
{  After loading your menu from the resource, you must set the kMenuItemAttrDynamic    }
{  flag for each dynamic item. In this example, you would use:                         }
{      ChangeMenuItemAttributes( menu, kCloseItem, kMenuItemAttrDynamic, 0 );          }
{      ChangeMenuItemAttributes( menu, kCloseAllItem, kMenuItemAttrDynamic, 0 );       }
{  The Menu Manager will now automatically display the correct item depending on       }
{  whether the Option key is pressed. The result from MenuSelect will be the item      }
{  number of the item that was visible when the menu closed.                           }
{  Note that:                                                                          }
{      - If you use InterfacerBuilder, you can set the kMenuItemAttrDynamic attribute  }
{        directly in the menu object in the nib, using the Inspector window for the    }
{        menu. You don't need to add the attribute after creating the menu.            }
{      - If your application requires Mac OS X 10.2 or CarbonLib 1.6 or later, you     }
{        can also use a version-one-format 'xmnu' resource, which allows settting      }
{        the menu item attributes in the 'xmnu'. Using a version one resource, you     }
{        can include the kMenuItemAttrDynamic attribute in the resource rather than    }
{        adding it after creating the menu.                                            }
{  If the Menu Manager's built-in support is not sufficient, you can also change the   }
{  attributes of an item yourself and use the UpdateInvalidMenuItems API to cause      }
{  the menu to redraw. Changes to a menu item (changing text, command key, style,      }
{  etc.) that occur while the menu is open will cause the menu item to be invalidated, }
{  but not redrawn. If you need to invalidate the item explicitly yourself, perhaps    }
{  because you have a custom MDEF that depends on state not accessed using Menu        }
{  Manager APIs, you can use the InvalidateMenuItems API. UpdateInvalidMenuItems will  }
{  scan the menu for invalid items and redraw each, clearing its invalid flag          }
{  afterwards.                                                                         }
{  If you need to change menu contents based on modifier key state without using the   }
{  built-in support in the Menu Manager, we recommend that you install a Carbon event  }
{  handler on your menu for the [kEventClassKeyboard, kEventRawKeyModifiersChanged]    }
{  event. Modifier key events are passed to the currently open menu before being sent  }
{  to the user focus target.                                                           }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  IsMenuItemInvalid()
 *  
 *  Summary:
 *    Determines if a menu item is invalid and should be redrawn.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose item to examine.
 *    
 *    inItem:
 *      The item to examine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuItemInvalid( inMenu: MenuRef; inItem: MenuItemIndex ): Boolean; external name '_IsMenuItemInvalid';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalidateMenuItems()
 *  
 *  Summary:
 *    Invalidates a group of menu items so that they will be redrawn
 *    when UpdateInvalidMenuItems is next called.
 *  
 *  Discussion:
 *    Menu items are automatically invalidated when their contents are
 *    changed using Menu Manager APIs while the menu is open. However,
 *    you might need to use this API if you have a custom MDEF that
 *    draws using state not contained in the menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose items to invalidate.
 *    
 *    inFirstItem:
 *      The first item to invalidate.
 *    
 *    inNumItems:
 *      The number of items to invalidate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvalidateMenuItems( inMenu: MenuRef; inFirstItem: MenuItemIndex; inNumItems: ItemCount ): OSStatus; external name '_InvalidateMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateInvalidMenuItems()
 *  
 *  Summary:
 *    Redraws the invalid items of an open menu.
 *  
 *  Discussion:
 *    It is not necessary to use UpdateInvalidMenuItems if you are
 *    using Carbon's built-in support for dynamic items based on
 *    modifier key state. However, if you are modifying items
 *    dynamically using your own implementation, you should call
 *    UpdateInvalidMenuItems after completing your modifications for a
 *    single menu. It will redraw any items that have been marked as
 *    invalid, and clear the invalid flag for those items.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to update.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateInvalidMenuItems( inMenu: MenuRef ): OSStatus; external name '_UpdateInvalidMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Standard font menu (Carbon and later)                                             }
{  These APIs allow you to create and use the standard font menu.                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

const
	kHierarchicalFontMenuOption = $00000001;

{$ifc not TARGET_CPU_64}
{
 *  CreateStandardFontMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateStandardFontMenu( menu: MenuRef; afterItem: MenuItemIndex; firstHierMenuID: MenuID; options: OptionBits; outHierMenuCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_CreateStandardFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateStandardFontMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateStandardFontMenu( menu: MenuRef; outHierMenuCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_UpdateStandardFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetFontFamilyFromMenuSelection()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetFontFamilyFromMenuSelection( menu: MenuRef; item: MenuItemIndex; var outFontFamily: FMFontFamily; var outStyle: FMFontStyle ): OSStatus; external name '_GetFontFamilyFromMenuSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Contextual Menu routines and constants                                            }
{  available with Conxtextual Menu extension 1.0 and later                             }
{——————————————————————————————————————————————————————————————————————————————————————}
{ Gestalt Selector for classic 68K apps only. }
{ CFM apps should weak link and check the symbols. }
{$endc} {not TARGET_CPU_64}

const
	gestaltContextualMenuAttr = FourCharCode('cmnu');
	gestaltContextualMenuUnusedBit = 0;
	gestaltContextualMenuTrapAvailable = 1;
	gestaltContextualMenuHasAttributeAndModifierKeys = 2; { Contextual Menu Manager supports keyContextualMenuAttributes and keyContextualMenuModifiers }
	gestaltContextualMenuHasUnicodeSupport = 3; { Contextual Menu Manager supports typeUnicodeText and typeCFStringRef }


{
 *  Summary:
 *    Values indicating what kind of help the application supports
 }
const
{
   * The application does not support any help. The Menu Manager will
   * put an appropriate help string into the contextual menu and
   * disable the Help item.
   }
	kCMHelpItemNoHelp = 0;

  {
   * The application supports Apple Guide help. The Menu Manager will
   * put the name of the main Guide file into the contextual menu and
   * enable the Help item.
   }
	kCMHelpItemAppleGuide = 1;

  {
   * The application supports some other form of help. In this case,
   * the application must also pass a valid string into the
   * inHelpItemString parameter of ContextualMenuSelect. This string
   * will be the text of the Help item in the contextual menu, and the
   * Help item will be enabled.
   }
	kCMHelpItemOtherHelp = 2;

  {
   * The application does not support any help. The Menu Manager will
   * remove the Help item from the contextual menu. This constant is
   * available in Mac OS X and CarbonLib 1.6, and later; however, in
   * CarbonLib it is translated to kCMHelpItemNoHelp, and the Help item
   * is only disabled, not removed.
   }
	kCMHelpItemRemoveHelp = 3;


{
 *  Summary:
 *    Values indicating what was chosen from a contextual menu
 }
const
{
   * The user did not choose an item from the contextual menu and the
   * application should do no further processing of the event.
   }
	kCMNothingSelected = 0;

  {
   * The user chose one of the application's items from the menu. The
   * application can examine the outMenuID and outMenuItem parameters
   * of ContextualMenuSelect to see what the menu selection was, and it
   * should then handle the selection appropriately.
   }
	kCMMenuItemSelected = 1;

  {
   * The user chose the Help item from the menu. The application should
   * open an Apple Guide database to a section appropriate for the
   * selection. If the application supports some other form of help, it
   * should be presented instead.
   }
	kCMShowHelpSelected = 3;


{
 *  Summary:
 *    AERecord keywords used by the ExamineContext method of a
 *    Contextual Menu plugin to specify menu item contents.
 }
const
{
   * Specifies the text of an item in a contextual menu. Data for this
   * parameter can be in one of several formats. In Mac OS 7/8/9.x and
   * Mac OS X 10.0 and 10.1, typeChar and typeIntlText are supported.
   * In Mac OS X 10.2 and later, typeStyledText, typeAEText,
   * typeUnicodeText, and typeCFStringRef are also supported. If you
   * provide data as typeCFStringRef, the Contextual Menu Manager will
   * automatically release the CFStringRef once the menu has been
   * displayed. If you need the CFStringRef to have a longer timetime,
   * your plugin should retain the CFStringRef before inserting it into
   * the AERecord.
   }
	keyContextualMenuName = FourCharCode('pnam');

  {
   * Specifies the command ID of an item in a contextual menu. Data for
   * this parameter should be typeSInt32.
   }
	keyContextualMenuCommandID = FourCharCode('cmcd');

  {
   * Specifies a contextual menu item with a submenu. Typically used
   * with AEPutKeyDesc to add an entire AEDesc containing the submenu
   * as the data for the parameter.
   }
	keyContextualMenuSubmenu = FourCharCode('cmsb');

  {
   * Specifies the menu item attributes of an item in a contextual
   * menu. Data for this parameter should be typeSInt32. Available in
   * Mac OS X 10.2 and later.
   }
	keyContextualMenuAttributes = FourCharCode('cmat');

  {
   * Specifies the modifier keys of an item in a contextual menu (see
   * kMenuShiftModifier, kMenuControlModifier, etc.) Data for this
   * parameter should be typeSInt32. Using this parameter together with
   * the keyContextualMenuAttributes parameter, it is possible to
   * create a contextual menu with dynamic items which change according
   * to the modifier keys pressed by the user. Available in Mac OS X
   * 10.2 and later.
   }
	keyContextualMenuModifiers = FourCharCode('cmmd');

{$ifc not TARGET_CPU_64}
{
 *  IsShowContextualMenuClick()
 *  
 *  Summary:
 *    Determines whether a particular EventRecord could invoke a
 *    contextual menu.
 *  
 *  Discussion:
 *    Applications should call IsShowContextualMenuClick when they
 *    receive non-null events. If IsShowContextualMenuClick returns
 *    true, your application should generate its own menu and Apple
 *    Event descriptor (AEDesc), and then call ContextualMenuSelect to
 *    display and track the contextual menu, and then handle the user's
 *    choice. 
 *    
 *    Apple recommends that you use the IsShowContextualMenuEvent API
 *    instead of this API. If you use the standard window Carbon event
 *    handler on your windows, you can also use a
 *    kEventControlContextualMenuClick event handler to be notified
 *    when a contextual menu click has occured; in this case, you do
 *    not need to call this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine.
 *  
 *  Result:
 *    Returns true if the application should display a contextual menu,
 *    false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function IsShowContextualMenuClick( const (*var*) inEvent: EventRecord ): Boolean; external name '_IsShowContextualMenuClick';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsShowContextualMenuEvent()
 *  
 *  Summary:
 *    Determines whether a particular EventRef could invoke a
 *    contextual menu.
 *  
 *  Discussion:
 *    This API is similar to IsShowContextualMenuClick, but takes a
 *    Carbon EventRef as its parameter instead of an EventRecord.
 *    
 *    
 *    In Mac OS X 10.3 and earlier, this API always returned false if
 *    the event kind was not kEventMouseDown,
 *    kEventWindowClickContentRgn, kEventWindowClickStructureRgn, or
 *    kEventWindowHandleContentClick. In Mac OS X 10.4 and later, this
 *    API no longer requires a specific event kind; it only requires
 *    that the event contain kEventParamMouseButton and
 *    kEventParamKeyModifiers parameters. 
 *    
 *    If you use the standard window Carbon event handler on your
 *    windows, you can also use a kEventControlContextualMenuClick
 *    event handler to be notified when a contextual menu click has
 *    occured; in this case, you do not need to call this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine.
 *  
 *  Result:
 *    Returns true if the application should display a contextual menu,
 *    false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsShowContextualMenuEvent( inEvent: EventRef ): Boolean; external name '_IsShowContextualMenuEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ContextualMenuSelect()
 *  
 *  Summary:
 *    Displays a contextual menu.
 *  
 *  Discussion:
 *    If IsShowContextualMenuClick returns true, you should call the
 *    ContextualMenuSelect API after generating your own menu and
 *    preparing an Apple Event descriptor (AEDesc) that describes the
 *    item for which your application is displaying a contextual menu.
 *    This descriptor may contain an object specifier or raw data and
 *    will be passed to all contextual menu plug-ins. 
 *    
 *    The system will add other items before displaying the contextual
 *    menu, and it will remove those items before returning, leaving
 *    the menu in its original state. 
 *    
 *    After all the system commands are added, the contextual menu is
 *    displayed and tracked. If the user selects one of the system
 *    items, it is handled by the system and the call returns as though
 *    the user didn't select anything from the menu. If the user
 *    selects any other item (or no item at all), the Menu Manager
 *    passes back appropriate values in the parameters
 *    outUserSelectionType, outMenuID, and outMenuItem. 
 *    
 *    Your application should provide visual feedback indicating the
 *    item that was clicked upon. For example, a click on an icon
 *    should highlight the icon, while a click on editable text should
 *    not eliminate the current selection. 
 *    
 *    If the outUserSelectionType parameter contains
 *    kCMMenuItemSelected, you should look at the outMenuID and
 *    outMenuItem parameters to determine what menu item the user chose
 *    and handle it appropriately. If the user selected
 *    kCMHelpItemSelected, you should open the proper Apple Guide
 *    sequence or other form of custom help. 
 *    
 *    There is a bug in ContextualMenuSelect in versions of Mac OS X
 *    prior to Mac OS X 10.4 that causes the value returned in the
 *    outUserSelectionType parameter to be set to kCMShowHelpSelected
 *    instead of kCMMenuItemSelected if the menu has a menu ID of zero
 *    and item three is selected. To avoid this problem on all versions
 *    of Mac OS X, simply use a non-zero menu ID. Note that when a new
 *    menu is created in InterfaceBuilder, its ID is automatically set
 *    to zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      A menu containing application commands to display. The caller
 *      creates this menu based on the current context, the mouse
 *      location, and the current selection (if it was the target of
 *      the mouse). If you pass nil, only system commands will be
 *      displayed. The menu should be added to the menu list as a
 *      pop-up menu (using the InsertMenu function).
 *    
 *    inGlobalLocation:
 *      The location (in global coordinates) of the mouse near which
 *      the menu is to be displayed.
 *    
 *    inReserved:
 *      Reserved for future use. Pass false for this parameter.
 *    
 *    inHelpType:
 *      An identifier specifying the type of help to be provided by the
 *      application; see kCMHelpItem constants.
 *    
 *    inHelpItemString:
 *      A string containing the text to be displayed for the help menu
 *      item. This string is unused unless you also pass the constant
 *      kCMHelpItemOtherHelp in the inHelpType parameter.
 *    
 *    inSelection:
 *      An object specifier for the current selection. This allows he
 *      system to examine the selection and add special system commands
 *      accordingly. Passing a value of nil indicates that no selection
 *      should be examined, and most likely, no special system actions
 *      will be included.
 *    
 *    outUserSelectionType:
 *      On exit, the value indicates what the user selected from the
 *      contextual menu; see kCMNothingSelected, kCMMenuItemSelected,
 *      and kCMShowHelpSelected.
 *    
 *    outMenuID:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu ID of the chosen item.
 *    
 *    outMenuItem:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu item chosen.
 *  
 *  Result:
 *    An OSStatus result code. ContextualMenuSelect returns the result
 *    code userCanceledErr and sets outUserSelectionType to
 *    kCMNothingSelected to indicate that the user did not select
 *    anything from the contextual menu and no further processing is
 *    needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function ContextualMenuSelect( inMenu: MenuRef; inGlobalLocation: Point; inReserved: Boolean; inHelpType: UInt32; {const} inHelpItemString:ConstStringPtr { can be NULL }; {const} inSelection: AEDescPtr { can be NULL }; var outUserSelectionType: UInt32; var outMenuID: MenuID; var outMenuItem: MenuItemIndex ): OSStatus; external name '_ContextualMenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {NOT TARGET_CPU_64}

{——————————————————————————————————————————————————————————————————————————————————————}
{  Contextual Menu Plugin Interface                                                    }
{  For Mac OS X 10.1, we support a new type of Contextual Menu Plugin: the CFPlugIn    }
{  based plugin.  Each plugin must be in a CFPlugIn in the Contextual Menu Items       }
{  folder in one of these paths:                                                       }
{      /System/Library/Contextual Menu Items/                                          }
{      /Library/Contextual Menu Items/                                                 }
{      ~/Library/Contextual Menu Items/                                                }
{  It must export the following functions using the following interface or a C++       }
{  interface inheriting from IUnknown and including similar functions.                 }
{——————————————————————————————————————————————————————————————————————————————————————}
(*
{ The Contextual Menu Manager will only load CFPlugIns of type kContextualMenuTypeID }
#define kContextualMenuTypeID ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x2F, 0x65, 0x22, 0xE9, 0x3E, 0x66, 0x11, 0xD5, \
  0x80, 0xA7, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 2F6522E9-3E66-11D5-80A7-003065B300BC }

{ Contextual Menu Plugins must implement this Contexual Menu Plugin Interface }
#define kContextualMenuInterfaceID    ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x32, 0x99, 0x7B, 0x62, 0x3E, 0x66, 0x11, 0xD5, \
  0xBE, 0xAB, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 32997B62-3E66-11D5-BEAB-003065B300BC }

#define CM_IUNKNOWN_C_GUTS \
   void *_reserved; \
 SInt32 (*QueryInterface)(void *thisPointer, CFUUIDBytes iid, void ** ppv); \
   UInt32 (*AddRef)(void *thisPointer); \
 UInt32 (*Release)(void *thisPointer)

{ The function table for the interface }
type
	ContextualMenuInterfaceStruct = record
CM_IUNKNOWN_C_GUTS;
    OSStatus ( *ExamineContext )(
          void*               thisInstance,
          const AEDesc*       inContext,
         AEDescList*         outCommandPairs );
 OSStatus ( *HandleSelection )(
         void*               thisInstance,
          AEDesc*             inContext,
         SInt32              inCommandID );
 void ( *PostMenuCleanup )(
         void*               thisInstance );
	end;
*)
{
 *  CMPluginExamineContext()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginExamineContext( thisInstance: UnivPtr; const (*var*) inContext: AEDesc; var outCommandPairs: AEDescList ): OSStatus; external name '_CMPluginExamineContext';


{
 *  CMPluginHandleSelection()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginHandleSelection( thisInstance: UnivPtr; var inContext: AEDesc; inCommandID: SInt32 ): OSStatus; external name '_CMPluginHandleSelection';


{
 *  CMPluginPostMenuCleanup()
 *  
 *  Availability:
 *    Implemented by client
 }
procedure CMPluginPostMenuCleanup( thisInstance: UnivPtr ); external name '_CMPluginPostMenuCleanup';


{ previously in LowMem.h.  This functions return the menu ID of the hilited menu }
{$ifc not TARGET_CPU_64}
{
 *  LMGetTheMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetTheMenu: MenuID; external name '_LMGetTheMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  • DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
    A Short Course on Menu Definition Functions
    
    A menu definition function is used to implement a custom visual appearance for a menu.
    Menu definition functions are still supported in Carbon, but the messages sent to a
    menu definition function in Carbon are different than for a non-Carbon application.
    
    In general, Apple recommends using the system menu definition whenever possible.
    Menu definition functions will continue to be supported, but it is not easy to write
    a correct menu definition, especially one that attempts to imitate the standard system
    menu appearance. If you require special features in your menu that the system menu
    definition does not support, please mail <toolbox@apple.com> and describe your requirements;
    we would much rather enhance the system menu definition than have you write a custom one.
    The use of menu definition functions is deprecated in Mac OS X; in Mac OS X 10.3 and
    later, Apple recommends that you use a custom HIView to draw menu content.
    
    Menu definition functions before Carbon used the following messages:
    
        kMenuDrawMsg
        kMenuChooseMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuDrawItemMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    kMenuChooseMsg and kMenuDrawItemMsg are not supported in Carbon and are not sent to
    Carbon menu definitions. In Carbon, kMenuChooseMsg is replaced by kMenuFindItemMsg and
    kMenuHiliteItemMsg. Menu definition functions in Carbon use the following messages:
    
        kMenuInitMsg
        kMenuDisposeMsg
        kMenuFindItemMsg
        kMenuHiliteItemMsg
        kMenuDrawItemsMsg
        kMenuDrawMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    The rest of this documentation will focus on Carbon menu definitions only.
    
    Menu Definition Messages
    
        Carbon menu definition functions should support the following messages:
        
        kMenuInitMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       OSErr*
        
            Sent when a menu is created. This message gives the menu definition an opportunity
            to initialize its own state. If the menu definition encounters an error while
            initializing, it should set *whichItem to a non-zero error code; this will cause the
            Menu Manager to destroy the menu and return an error back from the API that was used
            to create the menu.
        
        kMenuDisposeMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       unused
            
            Sent when a menu is destroyed. This message gives the menu definition an opportunity
            to destroy its own data.
            
        kMenuFindItemMsg
            
            menuRect        menu bounds
            hitPt           point to hit-test
            whichItem       MDEFFindItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to know what item is under
            the mouse. The whichItem parameter is actually a pointer to a MenuTrackingData structure.
            On entry, the menu, virtualMenuTop, and virtualMenuBottom fields of this structure are
            valid. The menu definition should determine which item containst the given point, if any,
            and fill in the itemUnderMouse, itemSelected, and itemRect fields. If an item is found,
            the menu definition should always fill in the itemUnderMouse and itemRect fields. The
            menu definition should only fill in the itemSelected field if the item is available for
            selection; if it is unavailable (because it is disabled, or for some other reason), the
            menu definition should set the itemSelected field to zero.
            
            The values placed in the itemUnderMouse and itemSelected fields should be less than or
            equal to the number of items returned by CountMenuItems on this menu. The values placed
            in these two fields should be identical if both are non-zero. itemUnderMouse should always
            be non-zero if the mouse is actually over an item.
            
            The menu definition should not hilite the found item during this message. The Menu 
            Manager will send a separate kMenuHiliteItemMsg to request hiliting of the item.
            
            If the menu definition supports scrolling, it should scroll the menu during this message,
            and update the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData to
            indicate the menu's new scrolled position.
            
            If the menu definition uses QuickDraw to draw while scrolling, it should draw into the
            current port.
    
            If the menu definition uses CoreGraphics to draw while scrolling, it should use the
            CGContextRef passed in the context field of the MDEFHiliteItemData structure.
            
            Menu definitions must use the ScrollMenuImage API, if available, to scroll the menu contents.
            This API is available in CarbonLib 1.5 and later, and in Mac OS X 10.1 and later. ScrollMenuImage
            properly supports scrolling the alpha channel in the menu's image data. Use of QuickDraw's
            ScrollRect API to scroll the menu contents will result in the alpha channel being set to 0xFF
            (opaque) and the menu will no longer be transparent.
            
            The menu definition should not modify the menu field of the MenuTrackingData.
            
        kMenuHiliteItemMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFHiliteItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to hilite a newly selected
            item. The whichItem parameter is actually a pointer to a MDEFHiliteItemData structure.
            The menu definition should unhilite the item in the previousItem field, if non-zero,
            and hilite the item in the newItem field.
            
            Menu definitions should use the EraseMenuBackground API to erase the old menu contents
            before unhiliting a menu item, if the menu definition is using the Appearance Manager's
            menu drawing APIs. This is necessary because the background of a menu is transparent on
            Aqua, and if the old hilite is not erased first, it will show through the new unhilited
            menu background.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFHiliteItemData structure.
            
        kMenuDrawItemsMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawItemsData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw a portion of the
            menu. This message is used by the dynamic menu item support code in the Menu Manager;
            for example, if items five and six in a menu are a dynamic group, the Menu Manager will
            send a DrawItems message when the group's modifier key is pressed or released to redraw
            the appropriate item, but no other items in the menu.
            
            The whichItem parameter for this message is actually a pointer to a MDEFDrawItemsData
            structure. The menu definition should redraw the items starting with firstItem and
            ending with lastItem, inclusive.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawItemsData structure.
            
        kMenuDrawMsg
            
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw the entire menu.
            The whichItem parameter is actually a pointer to a MenuTrackingData structure. On entry,
            the menu field of this structure is valid. The menu definition should draw the menu and,
            if it supports scrolling, should also fill in the virtualMenuTop and virtualMenuBottom
            fields of the structure to indicate the menu's initial unscrolled position; typically, 
            virtualMenuTop would be set to the same value as the top coordinate of the menu rect,
            and virtualMenuBottom would be set to virtualMenuTop plus the virtual height of the menu.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawData structure.
            
        kMenuSizeMsg
        
            menuRect        unused
            hitPt           maximum width and height of the menu
            whichItem       unused
            
            Sent when the Menu Manager needs to determine the size of a menu. The menu definition
            should calculate the width and height of the menu and store the sizes into the menu with
            SetMenuWidth and SetMenuHeight.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the hitPt parameter to this message is the maximum width (hitPt.h) and height
            (hitPt.v) of the menu. The menu definition should ensure that the width and height that it
            places in the menu do not exceed these values. If the gestalt bit is not set, the menu
            definition should just use the main GDevice's width and height as constraints on the menu's
            width and height.
            
        kMenuPopUpMsg
        
            menuRect        on entry, constraints on the menu's position; on exit, menu bounds
            hitPt           requested menu location, with swapped coordinates
            whichItem       on entry, requested initial selection; on exit, virtual menu top
            
            Sent when the Menu Manager is about to display a popup menu. The menu definition should
            calculate the appropriate menu bounds to contain the menu based on the requested menu
            location and selected item. It should write the menuBounds into the rect given by the
            menuRect parameter.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the menuRect parameter on entry to this message contains a constraint rect,
            in global coordinates, outside of which the popup menu should not be positioned. The menu
            definition should take this constraint rect into account as it calculates the menu bounds.
            If the gestalt bit is not set, the menu definition should use the bounds of the GDevice
            containing the menu's top left corner as a constraint on the menu's position.
            
            The hitPt parameter is a requested location for the top left corner of the menu. The
            coordinates of this parameter are swapped from their normal order; the h field of the
            hitPt parameter contains the vertical coordinate, and the v field of hitPt contains
            the horizontal coordinate.
            
            On entry, the whichItem parameter points at a menu item index which is requested to be
            the initial selection when the menu is displayed. After calculating the menu's bounds,
            the menu definition should write the menu's virtual top coordinate into the location
            pointed at by the whichItem parameter. If displaying the menu at the requested location
            does not require scrolling, the virtual top will be the same as the menu bounds top;
            if the menu must scroll to fit in the requested location, the virtual top may be different.
            
        kMenuCalcItemMsg
        
            menuRect        on exit, item bounds
            hitPt           unused
            whichItem       the item whose rect to calculate
            
            Sent when the Menu Manager needs to know the bounds of a menu item. The menu definition
            should calculate the size of the item specified by the whichItem parameter, and store 
            the bounds in the rect specified by the menuRect parameter.
            
            Some sample menu definition code provided by Apple has previously shown an implementation
            of this message that always sets the top left corner of the item bounds to (0,0), regardless
            of the item's actual position in the menu. For best future compatibility, menu definitions
            should begin storing an item bounds that gives the item's actual position in the menu based
            on the menu's current virtual top. For example, if the virtual menu top starts at 20, then
            the menu definition would calculate an item bounds for the first item that starts at (0,20),
            an item bounds for the second item that starts at (0,40), and so on. The menu definition
            should call GetMenuTrackingData to get the menu's current virtual position, and use zero
            for the menu top if GetMenuTrackingData returns an error.
            
        kMenuThemeSavvyMsg
        
            menuRect        unused
            hitPt           unused
            whichItem       on exit, indicates theme-savvyness of the menu definition
            
            Sent by the Menu Manager to determine whether the MDEF uses the Appearance Manager 
            menu-drawing functions to draw its menu. If it does, the menu definition should return
            kThemeSavvyMenuResponse in the location pointed to by whichItem. If the menu definition
            draws its own custom content without using the Appearance Manager menu-drawing functions,
            it should ignore this message.
    
    Low-memory Global Replacements
    
        Pre-Carbon menu definitions needed to use several low-memory globals to communicate with the
        Menu Manager. These globals have all been replaced or made obsolete in Carbon, as follows:
        
        MenuDisable
            
            MenuDisable is now set automatically by the Menu Manager using the value returned in the
            itemUnderMouse field of the MenuTrackingData structure passed to kMenuFindItemMsg.
            
        TopMenuItem
        AtMenuBottom
        
            TopMenuItem and AtMenuBottom are now set automatically by the Menu Manager using the
            values returned in the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData
            structure passed to kMenuDrawMsg and kMenuFindItemMsg.
            
        mbSaveLoc
    
            This undocumented low-memory global was used by pre-Carbon menu definitions to store
            the bounding rect of the currently selected item and to avoid drawing glitches while
            the menu definition was scrolling the contents of a menu that had submenus. The Menu
            Manager now automatically sets the selected item bounds using the value returned in
            the itemRect field of the MenuTrackingData structure passed to kMenuFindItemMsg. In
            order to correctly support scrolling of menus with submenus, a menu definition should
            verify, before scrolling the menu contents, that no submenus of the scrolling menu are
            currently visible. A menu definition can use GetMenuTrackingData to verify this condition,
            as follows:
            
                Boolean SafeToScroll( MenuRef menuBeingScrolled )
                (
                    MenuTrackingData lastMenuData;
                    return GetMenuTrackingData( NULL, &lastMenuData ) == noErr
                           && lastMenuData.menu == menuBeingScrolled;
                )
            
            If SafeToScroll returns false, the menu definition should not scroll the menu.
}
{$endc} {not TARGET_CPU_64}

const
{ menu defProc messages }
	kMenuDrawMsg = 0;
	kMenuSizeMsg = 2;
	kMenuPopUpMsg = 3;    { position the popup menu rect appropriately }
	kMenuCalcItemMsg = 5;
	kMenuThemeSavvyMsg = 7;    { is your MDEF theme-savvy?  If so, return kThemeSavvyMenuResponse in the whichItem parameter}
	kMenuInitMsg = 8;    { Return an error code in *whichItem to indicate success or failure. Only supported in Carbon. }
	kMenuDisposeMsg = 9;    { The menu is being destroyed. Only supported in Carbon.}
	kMenuFindItemMsg = 10;   { Determine which item is under the mouse. Only supported in Carbon.}
	kMenuHiliteItemMsg = 11;   { Hilite the specified item. Only supported in Carbon.}
	kMenuDrawItemsMsg = 12;   { Draw a range of items. Only supported in Carbon.}
	mDrawMsg = kMenuDrawMsg; { obsolete constant name}
	mSizeMsg = kMenuSizeMsg; { obsolete constant name}
	mPopUpMsg = kMenuPopUpMsg; { obsolete constant name}
	mCalcItemMsg = kMenuCalcItemMsg; { obsolete constant name}


const
	kThemeSavvyMenuResponse = $7473; { should be returned in *whichItem when handling kMenuThemeSavvyMsg}


{
 *  MDEFHiliteItemData
 *  
 *  Summary:
 *    The MDEFHiliteItemData structure contains information about which
 *    menu items should be hilited and unhilited as the user moves
 *    through the menus. It is used by menu definition functions, which
 *    receive a pointer to an MDEFHiliteItemData structure as the
 *    whichItem parameter during kMenuHiliteItemMsg.
 }
type
	MDEFHiliteItemData = record
{
   * The item that was previously selected. It should be redrawn in an
   * unhilited state. May be zero if no item was previously selected.
   }
		previousItem: MenuItemIndex;

  {
   * The item that is now selected. It should be redrawn in a hilited
   * state. May be zero if no item is now selected.
   }
		newItem: MenuItemIndex;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFHiliteItemDataPtr = ^MDEFHiliteItemData;
type
	HiliteMenuItemData = MDEFHiliteItemData;
	HiliteMenuItemDataPtr = MDEFHiliteItemDataPtr;

{
 *  MDEFDrawData
 *  
 *  Summary:
 *    The MDEFDrawData structure contains information needed to draw a
 *    menu. It is used by menu definition functions, which receive a
 *    pointer to an MDEFDrawData structure as the whichItem parameter
 *    during kMenuDrawMsg.
 }
type
	MDEFDrawData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the virtualMenuTop and virtualMenuBottom fields of this structure
   * while drawing the menu.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFDrawDataPtr = ^MDEFDrawData;

{
 *  MDEFFindItemData
 *  
 *  Summary:
 *    The MDEFFindItemData structure contains information needed to
 *    determine which item is currently selected by the user. It is
 *    used by menu definition functions, which receive a pointer to an
 *    MDEFDrawData structure as the whichItem parameter during
 *    kMenuFindItemMsg.
 }
type
	MDEFFindItemData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the itemSelected, itemUnderMouse, and itemRect fields of this
   * structure after determining which item is at the specified point.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into if it needs
   * to scroll the menu during the FindItem message. The Menu Manager
   * will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFFindItemDataPtr = ^MDEFFindItemData;

{
 *  MDEFDrawItemsData
 *  
 *  Summary:
 *    The MDEFDrawItemsData structure contains information about which
 *    menu items to redraw. It is used by menu definition functions,
 *    which receive a pointer to an MDEFDrawItemsData structure as the
 *    whichItem parameter during kMenuDrawItemsMsg.
 }
type
	MDEFDrawItemsData = record
{
   * The first item to draw.
   }
		firstItem: MenuItemIndex;

  {
   * The last item to draw.
   }
		lastItem: MenuItemIndex;

  {
   * Information about the menu's tracking state. The virtualMenuTop
   * and virtualMenuBottom fields in this structure will be the most
   * useful in handling the DrawItems message.
   }
		trackingData: MenuTrackingDataPtr;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF returns.
   }
		context: UnivPtr;
	end;
	MDEFDrawItemsDataPtr = ^MDEFDrawItemsData;
{ obsolete menu color table constants}
const
	mctAllItems = -98;  {search for all Items for the given ID}
	mctLastIDIndic = -99;   {last color table entry has this in ID field}

type
	MCEntry = record
		mctID: MenuID;                  {menu ID.  ID = 0 is the menu bar}
		mctItem: SInt16;                {menu Item. Item = 0 is a title}
		mctRGB1: RGBColor;                {usage depends on ID and Item}
		mctRGB2: RGBColor;                {usage depends on ID and Item}
		mctRGB3: RGBColor;                {usage depends on ID and Item}
		mctRGB4: RGBColor;                {usage depends on ID and Item}
		mctReserved: SInt16;            {reserved for internal use}
	end;
	MCEntryPtr = ^MCEntry;
type
	MCTable = array [0..0] of MCEntry;	{ARRAY [1..numEntries] of MCEntry}
	MCTablePtr = ^MCTable;
	MCTableHandle = ^MCTablePtr;
type
	MenuCRsrc = record
		numEntries: SInt16;             {number of entries}
		mcEntryRecs: MCTable;            {ARRAY [1..numEntries] of MCEntry}
	end;
	MenuCRsrcPtr = ^MenuCRsrc;
type
	MenuCRsrcHandle = ^MenuCRsrcPtr;


const
{
   * Menu item command ID to indicate a hierarchical menu; the item
   * icon ID is the hierarchical menu ID. This constant is deprecated.
   * Use SetMenuItemHierarchicalID or SetMenuItemHierarchicalMenu
   * instead of using this constant.
   }
	hMenuCmd = 27;

  {
   * A menu ID used with InsertMenu to insert a menu into the
   * hierarchical portion of the menubar. Rather than inserting menus
   * into the hierarchical portion of the menubar, we recommend that
   * your application use SetMenuItemHierarchicalMenu to directly
   * specify the submenu's MenuRef.
   }
	kInsertHierarchicalMenu = -1;

  {
   * The old name for kInsertHierarchicalMenu. This constant is
   * deprecated.
   }
	hierMenu = -1;

{$ifc not TARGET_CPU_64}
{
 *  NewMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateNewMenu instead of NewMenu.
 *  
 *  Summary:
 *    Creates a new menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewMenu( menuID_: MenuID; const (*var*) menuTitle: Str255 ): MenuRef; external name '_NewMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuTitleAsCFString instead of GetMenuTitle.
 *  
 *  Summary:
 *    Retrieves the title of a menu as a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuTitle( menu: MenuRef; var title: Str255 ): StringPtr; external name '_GetMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuTitleWithCFString instead of SetMenuTitle.
 *  
 *  Summary:
 *    Sets the title of a menu to a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function SetMenuTitle( menu: MenuRef; const (*var*) title: Str255 ): OSStatus; external name '_SetMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  [Mac]AppendMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AppendMenuItemTextWithCFString instead of AppendMenu. Note
 *    that unlike AppendMenu, AppendMenuItemTextWithCFString does not
 *    interpret metacharacters in the text.
 *  
 *  Summary:
 *    Appends a new menu item to a menu, using a Pascal string for the
 *    item's text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
procedure AppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacAppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  [Mac]InsertMenuItem()
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu into which to insert the item.
 *    
 *    itemString:
 *      The text of the new item. This string is parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
procedure InsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacInsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AppendMenuItemText()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
>>>>>>> origin/cpstrnew
 }
function AppendMenuItemText( menu: MenuRef; const (*var*) inString: Str255 ): OSStatus; external name '_AppendMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use InsertMenuItemTextWithCFString instead of InsertMenuItemText.
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu into which to insert the item.
 *    
 *    inString:
 *      The text of the new item. This string is not parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh; the new item's text becomes exactly the
 *      specified text.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
>>>>>>> origin/cpstrnew
 }
function InsertMenuItemText( menu: MenuRef; const (*var*) inString: Str255; afterItem: MenuItemIndex ): OSStatus; external name '_InsertMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of SetMenuItemText.
 *  
 *  Summary:
 *    Sets the text of a menu item using a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> origin/cpstrnew
 }
procedure SetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; const (*var*) itemString: Str255 ); external name '_SetMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

<<<<<<< HEAD
{——————————————————————————————————————————————————————————————————————————————————————}
{  • Standard font menu (Carbon and later)                                             }
{  These APIs allow you to create and use the standard font menu.                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

const
	kHierarchicalFontMenuOption = $00000001;
=======
>>>>>>> origin/cpstrnew

{$ifc not TARGET_CPU_64}
{
 *  GetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of GetMenuItemText.
 *  
 *  Summary:
 *    Retrieves the text of a menu item as a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; var itemString: Str255 ); external name '_GetMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of setting an item's text
 *    as a Str255.
 *  
 *  Summary:
 *    Sets the text encoding that should be used to interpret a menu
 *    item's Pascal string-based text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; inScriptID: TextEncoding ): OSErr; external name '_SetMenuItemTextEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of retrieving a menu
 *    item's text as a Str255.
 *  
 *  Summary:
 *    Retrieves the text encoding that should be used to interpret a
 *    menu item's Pascal string-based text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
<<<<<<< HEAD
function GetFontFamilyFromMenuSelection( menu: MenuRef; item: MenuItemIndex; var outFontFamily: FMFontFamily; var outStyle: FMFontStyle ): OSStatus; external name '_GetFontFamilyFromMenuSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Contextual Menu routines and constants                                            }
{  available with Conxtextual Menu extension 1.0 and later                             }
{——————————————————————————————————————————————————————————————————————————————————————}
{ Gestalt Selector for classic 68K apps only. }
{ CFM apps should weak link and check the symbols. }
{$endc} {not TARGET_CPU_64}

const
	gestaltContextualMenuAttr = FourCharCode('cmnu');
	gestaltContextualMenuUnusedBit = 0;
	gestaltContextualMenuTrapAvailable = 1;
	gestaltContextualMenuHasAttributeAndModifierKeys = 2; { Contextual Menu Manager supports keyContextualMenuAttributes and keyContextualMenuModifiers }
	gestaltContextualMenuHasUnicodeSupport = 3; { Contextual Menu Manager supports typeUnicodeText and typeCFStringRef }
=======
function GetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; var outScriptID: TextEncoding ): OSErr; external name '_GetMenuItemTextEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  SetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemIconHandle instead of SetItemIcon.
 *  
 *  Summary:
 *    Sets the resource ID of an icon resource to be displayed in a
 *    menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemIcon( theMenu: MenuRef; item: MenuItemIndex; iconIndex: SInt16 ); external name '_SetItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemIconHandle instead of GetItemIcon.
 *  
 *  Summary:
 *    Retrieves the resource ID of an icon resource to be displayed in
 *    a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetItemIcon( theMenu: MenuRef; item: MenuItemIndex; var iconIndex: SInt16 ); external name '_GetItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemHierarchicalMenu instead of
 *    SetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Sets the menu ID of the submenu of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
<<<<<<< HEAD
const
{
   * Specifies the text of an item in a contextual menu. Data for this
   * parameter can be in one of several formats. In Mac OS 7/8/9.x and
   * Mac OS X 10.0 and 10.1, typeChar and typeIntlText are supported.
   * In Mac OS X 10.2 and later, typeStyledText, typeAEText,
   * typeUnicodeText, and typeCFStringRef are also supported. If you
   * provide data as typeCFStringRef, the Contextual Menu Manager will
   * automatically release the CFStringRef once the menu has been
   * displayed. If you need the CFStringRef to have a longer timetime,
   * your plugin should retain the CFStringRef before inserting it into
   * the AERecord.
   }
	keyContextualMenuName = FourCharCode('pnam');

  {
   * Specifies the command ID of an item in a contextual menu. Data for
   * this parameter should be typeSInt32.
   }
	keyContextualMenuCommandID = FourCharCode('cmcd');

  {
   * Specifies a contextual menu item with a submenu. Typically used
   * with AEPutKeyDesc to add an entire AEDesc containing the submenu
   * as the data for the parameter.
   }
	keyContextualMenuSubmenu = FourCharCode('cmsb');

  {
   * Specifies the menu item attributes of an item in a contextual
   * menu. Data for this parameter should be typeSInt32. Available in
   * Mac OS X 10.2 and later.
   }
	keyContextualMenuAttributes = FourCharCode('cmat');

  {
   * Specifies the modifier keys of an item in a contextual menu (see
   * kMenuShiftModifier, kMenuControlModifier, etc.) Data for this
   * parameter should be typeSInt32. Using this parameter together with
   * the keyContextualMenuAttributes parameter, it is possible to
   * create a contextual menu with dynamic items which change according
   * to the modifier keys pressed by the user. Available in Mac OS X
   * 10.2 and later.
   }
	keyContextualMenuModifiers = FourCharCode('cmmd');

{$ifc not TARGET_CPU_64}
=======
function SetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; inHierID: MenuID ): OSErr; external name '_SetMenuItemHierarchicalID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemHierarchicalMenu instead of
 *    GetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Retrieves the menu ID of the submenu of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; var outHierID: MenuID ): OSErr; external name '_GetMenuItemHierarchicalID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


>>>>>>> origin/cpstrnew
{
 *  GetMenuRetainCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFGetRetainCount instead of
 *    GetMenuRetainCount.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Applications should call IsShowContextualMenuClick when they
 *    receive non-null events. If IsShowContextualMenuClick returns
 *    true, your application should generate its own menu and Apple
 *    Event descriptor (AEDesc), and then call ContextualMenuSelect to
 *    display and track the contextual menu, and then handle the user's
 *    choice. 
 *    
 *    Apple recommends that you use the IsShowContextualMenuEvent API
 *    instead of this API. If you use the standard window Carbon event
 *    handler on your windows, you can also use a
 *    kEventControlContextualMenuClick event handler to be notified
 *    when a contextual menu click has occured; in this case, you do
 *    not need to call this API.
=======
 *  Summary:
 *    Returns the retain count of a menu.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to return.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
function GetMenuRetainCount( inMenu: MenuRef ): ItemCount; external name '_GetMenuRetainCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RetainMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRetain instead of RetainMenu.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    This API is similar to IsShowContextualMenuClick, but takes a
 *    Carbon EventRef as its parameter instead of an EventRecord.
 *    
 *    
 *    In Mac OS X 10.3 and earlier, this API always returned false if
 *    the event kind was not kEventMouseDown,
 *    kEventWindowClickContentRgn, kEventWindowClickStructureRgn, or
 *    kEventWindowHandleContentClick. In Mac OS X 10.4 and later, this
 *    API no longer requires a specific event kind; it only requires
 *    that the event contain kEventParamMouseButton and
 *    kEventParamKeyModifiers parameters. 
 *    
 *    If you use the standard window Carbon event handler on your
 *    windows, you can also use a kEventControlContextualMenuClick
 *    event handler to be notified when a contextual menu click has
 *    occured; in this case, you do not need to call this API.
=======
 *  Summary:
 *    Increments the retain count of a menu.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to increment.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsShowContextualMenuEvent( inEvent: EventRef ): Boolean; external name '_IsShowContextualMenuEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ContextualMenuSelect()
 *  
 *  Summary:
 *    Displays a contextual menu.
 *  
 *  Discussion:
 *    If IsShowContextualMenuClick returns true, you should call the
 *    ContextualMenuSelect API after generating your own menu and
 *    preparing an Apple Event descriptor (AEDesc) that describes the
 *    item for which your application is displaying a contextual menu.
 *    This descriptor may contain an object specifier or raw data and
 *    will be passed to all contextual menu plug-ins. 
 *    
 *    The system will add other items before displaying the contextual
 *    menu, and it will remove those items before returning, leaving
 *    the menu in its original state. 
 *    
 *    After all the system commands are added, the contextual menu is
 *    displayed and tracked. If the user selects one of the system
 *    items, it is handled by the system and the call returns as though
 *    the user didn't select anything from the menu. If the user
 *    selects any other item (or no item at all), the Menu Manager
 *    passes back appropriate values in the parameters
 *    outUserSelectionType, outMenuID, and outMenuItem. 
 *    
 *    Your application should provide visual feedback indicating the
 *    item that was clicked upon. For example, a click on an icon
 *    should highlight the icon, while a click on editable text should
 *    not eliminate the current selection. 
 *    
 *    If the outUserSelectionType parameter contains
 *    kCMMenuItemSelected, you should look at the outMenuID and
 *    outMenuItem parameters to determine what menu item the user chose
 *    and handle it appropriately. If the user selected
 *    kCMHelpItemSelected, you should open the proper Apple Guide
 *    sequence or other form of custom help. 
 *    
 *    There is a bug in ContextualMenuSelect in versions of Mac OS X
 *    prior to Mac OS X 10.4 that causes the value returned in the
 *    outUserSelectionType parameter to be set to kCMShowHelpSelected
 *    instead of kCMMenuItemSelected if the menu has a menu ID of zero
 *    and item three is selected. To avoid this problem on all versions
 *    of Mac OS X, simply use a non-zero menu ID. Note that when a new
 *    menu is created in InterfaceBuilder, its ID is automatically set
 *    to zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      A menu containing application commands to display. The caller
 *      creates this menu based on the current context, the mouse
 *      location, and the current selection (if it was the target of
 *      the mouse). If you pass nil, only system commands will be
 *      displayed. The menu should be added to the menu list as a
 *      pop-up menu (using the InsertMenu function).
 *    
 *    inGlobalLocation:
 *      The location (in global coordinates) of the mouse near which
 *      the menu is to be displayed.
 *    
 *    inReserved:
 *      Reserved for future use. Pass false for this parameter.
 *    
 *    inHelpType:
 *      An identifier specifying the type of help to be provided by the
 *      application; see kCMHelpItem constants.
 *    
 *    inHelpItemString:
 *      A string containing the text to be displayed for the help menu
 *      item. This string is unused unless you also pass the constant
 *      kCMHelpItemOtherHelp in the inHelpType parameter.
 *    
 *    inSelection:
 *      An object specifier for the current selection. This allows he
 *      system to examine the selection and add special system commands
 *      accordingly. Passing a value of nil indicates that no selection
 *      should be examined, and most likely, no special system actions
 *      will be included.
 *    
 *    outUserSelectionType:
 *      On exit, the value indicates what the user selected from the
 *      contextual menu; see kCMNothingSelected, kCMMenuItemSelected,
 *      and kCMShowHelpSelected.
 *    
 *    outMenuID:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu ID of the chosen item.
 *    
 *    outMenuItem:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu item chosen.
 *  
 *  Result:
 *    An OSStatus result code. ContextualMenuSelect returns the result
 *    code userCanceledErr and sets outUserSelectionType to
 *    kCMNothingSelected to indicate that the user did not select
 *    anything from the contextual menu and no further processing is
 *    needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function ContextualMenuSelect( inMenu: MenuRef; inGlobalLocation: Point; inReserved: Boolean; inHelpType: UInt32; {const} inHelpItemString:ConstStringPtr { can be NULL }; {const} inSelection: AEDescPtr { can be NULL }; var outUserSelectionType: UInt32; var outMenuID: MenuID; var outMenuItem: MenuItemIndex ): OSStatus; external name '_ContextualMenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {NOT TARGET_CPU_64}

{——————————————————————————————————————————————————————————————————————————————————————}
{  Contextual Menu Plugin Interface                                                    }
{  For Mac OS X 10.1, we support a new type of Contextual Menu Plugin: the CFPlugIn    }
{  based plugin.  Each plugin must be in a CFPlugIn in the Contextual Menu Items       }
{  folder in one of these paths:                                                       }
{      /System/Library/Contextual Menu Items/                                          }
{      /Library/Contextual Menu Items/                                                 }
{      ~/Library/Contextual Menu Items/                                                }
{  It must export the following functions using the following interface or a C++       }
{  interface inheriting from IUnknown and including similar functions.                 }
{——————————————————————————————————————————————————————————————————————————————————————}
(*
{ The Contextual Menu Manager will only load CFPlugIns of type kContextualMenuTypeID }
#define kContextualMenuTypeID ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x2F, 0x65, 0x22, 0xE9, 0x3E, 0x66, 0x11, 0xD5, \
  0x80, 0xA7, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 2F6522E9-3E66-11D5-80A7-003065B300BC }

{ Contextual Menu Plugins must implement this Contexual Menu Plugin Interface }
#define kContextualMenuInterfaceID    ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x32, 0x99, 0x7B, 0x62, 0x3E, 0x66, 0x11, 0xD5, \
  0xBE, 0xAB, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 32997B62-3E66-11D5-BEAB-003065B300BC }

#define CM_IUNKNOWN_C_GUTS \
   void *_reserved; \
 SInt32 (*QueryInterface)(void *thisPointer, CFUUIDBytes iid, void ** ppv); \
   UInt32 (*AddRef)(void *thisPointer); \
 UInt32 (*Release)(void *thisPointer)

{ The function table for the interface }
type
	ContextualMenuInterfaceStruct = record
CM_IUNKNOWN_C_GUTS;
    OSStatus ( *ExamineContext )(
          void*               thisInstance,
          const AEDesc*       inContext,
         AEDescList*         outCommandPairs );
 OSStatus ( *HandleSelection )(
         void*               thisInstance,
          AEDesc*             inContext,
         SInt32              inCommandID );
 void ( *PostMenuCleanup )(
         void*               thisInstance );
	end;
*)
{
 *  CMPluginExamineContext()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginExamineContext( thisInstance: UnivPtr; const (*var*) inContext: AEDesc; var outCommandPairs: AEDescList ): OSStatus; external name '_CMPluginExamineContext';


{
 *  CMPluginHandleSelection()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginHandleSelection( thisInstance: UnivPtr; var inContext: AEDesc; inCommandID: SInt32 ): OSStatus; external name '_CMPluginHandleSelection';


{
 *  CMPluginPostMenuCleanup()
 *  
 *  Availability:
 *    Implemented by client
 }
procedure CMPluginPostMenuCleanup( thisInstance: UnivPtr ); external name '_CMPluginPostMenuCleanup';


{ previously in LowMem.h.  This functions return the menu ID of the hilited menu }
{$ifc not TARGET_CPU_64}
{
 *  LMGetTheMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetTheMenu: MenuID; external name '_LMGetTheMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  • DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
    A Short Course on Menu Definition Functions
    
    A menu definition function is used to implement a custom visual appearance for a menu.
    Menu definition functions are still supported in Carbon, but the messages sent to a
    menu definition function in Carbon are different than for a non-Carbon application.
    
    In general, Apple recommends using the system menu definition whenever possible.
    Menu definition functions will continue to be supported, but it is not easy to write
    a correct menu definition, especially one that attempts to imitate the standard system
    menu appearance. If you require special features in your menu that the system menu
    definition does not support, please mail <toolbox@apple.com> and describe your requirements;
    we would much rather enhance the system menu definition than have you write a custom one.
    The use of menu definition functions is deprecated in Mac OS X; in Mac OS X 10.3 and
    later, Apple recommends that you use a custom HIView to draw menu content.
    
    Menu definition functions before Carbon used the following messages:
    
        kMenuDrawMsg
        kMenuChooseMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuDrawItemMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    kMenuChooseMsg and kMenuDrawItemMsg are not supported in Carbon and are not sent to
    Carbon menu definitions. In Carbon, kMenuChooseMsg is replaced by kMenuFindItemMsg and
    kMenuHiliteItemMsg. Menu definition functions in Carbon use the following messages:
    
        kMenuInitMsg
        kMenuDisposeMsg
        kMenuFindItemMsg
        kMenuHiliteItemMsg
        kMenuDrawItemsMsg
        kMenuDrawMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    The rest of this documentation will focus on Carbon menu definitions only.
    
    Menu Definition Messages
    
        Carbon menu definition functions should support the following messages:
        
        kMenuInitMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       OSErr*
        
            Sent when a menu is created. This message gives the menu definition an opportunity
            to initialize its own state. If the menu definition encounters an error while
            initializing, it should set *whichItem to a non-zero error code; this will cause the
            Menu Manager to destroy the menu and return an error back from the API that was used
            to create the menu.
        
        kMenuDisposeMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       unused
            
            Sent when a menu is destroyed. This message gives the menu definition an opportunity
            to destroy its own data.
            
        kMenuFindItemMsg
            
            menuRect        menu bounds
            hitPt           point to hit-test
            whichItem       MDEFFindItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to know what item is under
            the mouse. The whichItem parameter is actually a pointer to a MenuTrackingData structure.
            On entry, the menu, virtualMenuTop, and virtualMenuBottom fields of this structure are
            valid. The menu definition should determine which item containst the given point, if any,
            and fill in the itemUnderMouse, itemSelected, and itemRect fields. If an item is found,
            the menu definition should always fill in the itemUnderMouse and itemRect fields. The
            menu definition should only fill in the itemSelected field if the item is available for
            selection; if it is unavailable (because it is disabled, or for some other reason), the
            menu definition should set the itemSelected field to zero.
            
            The values placed in the itemUnderMouse and itemSelected fields should be less than or
            equal to the number of items returned by CountMenuItems on this menu. The values placed
            in these two fields should be identical if both are non-zero. itemUnderMouse should always
            be non-zero if the mouse is actually over an item.
            
            The menu definition should not hilite the found item during this message. The Menu 
            Manager will send a separate kMenuHiliteItemMsg to request hiliting of the item.
            
            If the menu definition supports scrolling, it should scroll the menu during this message,
            and update the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData to
            indicate the menu's new scrolled position.
            
            If the menu definition uses QuickDraw to draw while scrolling, it should draw into the
            current port.
    
            If the menu definition uses CoreGraphics to draw while scrolling, it should use the
            CGContextRef passed in the context field of the MDEFHiliteItemData structure.
            
            Menu definitions must use the ScrollMenuImage API, if available, to scroll the menu contents.
            This API is available in CarbonLib 1.5 and later, and in Mac OS X 10.1 and later. ScrollMenuImage
            properly supports scrolling the alpha channel in the menu's image data. Use of QuickDraw's
            ScrollRect API to scroll the menu contents will result in the alpha channel being set to 0xFF
            (opaque) and the menu will no longer be transparent.
            
            The menu definition should not modify the menu field of the MenuTrackingData.
            
        kMenuHiliteItemMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFHiliteItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to hilite a newly selected
            item. The whichItem parameter is actually a pointer to a MDEFHiliteItemData structure.
            The menu definition should unhilite the item in the previousItem field, if non-zero,
            and hilite the item in the newItem field.
            
            Menu definitions should use the EraseMenuBackground API to erase the old menu contents
            before unhiliting a menu item, if the menu definition is using the Appearance Manager's
            menu drawing APIs. This is necessary because the background of a menu is transparent on
            Aqua, and if the old hilite is not erased first, it will show through the new unhilited
            menu background.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFHiliteItemData structure.
            
        kMenuDrawItemsMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawItemsData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw a portion of the
            menu. This message is used by the dynamic menu item support code in the Menu Manager;
            for example, if items five and six in a menu are a dynamic group, the Menu Manager will
            send a DrawItems message when the group's modifier key is pressed or released to redraw
            the appropriate item, but no other items in the menu.
            
            The whichItem parameter for this message is actually a pointer to a MDEFDrawItemsData
            structure. The menu definition should redraw the items starting with firstItem and
            ending with lastItem, inclusive.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawItemsData structure.
            
        kMenuDrawMsg
            
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw the entire menu.
            The whichItem parameter is actually a pointer to a MenuTrackingData structure. On entry,
            the menu field of this structure is valid. The menu definition should draw the menu and,
            if it supports scrolling, should also fill in the virtualMenuTop and virtualMenuBottom
            fields of the structure to indicate the menu's initial unscrolled position; typically, 
            virtualMenuTop would be set to the same value as the top coordinate of the menu rect,
            and virtualMenuBottom would be set to virtualMenuTop plus the virtual height of the menu.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawData structure.
            
        kMenuSizeMsg
        
            menuRect        unused
            hitPt           maximum width and height of the menu
            whichItem       unused
            
            Sent when the Menu Manager needs to determine the size of a menu. The menu definition
            should calculate the width and height of the menu and store the sizes into the menu with
            SetMenuWidth and SetMenuHeight.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the hitPt parameter to this message is the maximum width (hitPt.h) and height
            (hitPt.v) of the menu. The menu definition should ensure that the width and height that it
            places in the menu do not exceed these values. If the gestalt bit is not set, the menu
            definition should just use the main GDevice's width and height as constraints on the menu's
            width and height.
            
        kMenuPopUpMsg
        
            menuRect        on entry, constraints on the menu's position; on exit, menu bounds
            hitPt           requested menu location, with swapped coordinates
            whichItem       on entry, requested initial selection; on exit, virtual menu top
            
            Sent when the Menu Manager is about to display a popup menu. The menu definition should
            calculate the appropriate menu bounds to contain the menu based on the requested menu
            location and selected item. It should write the menuBounds into the rect given by the
            menuRect parameter.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the menuRect parameter on entry to this message contains a constraint rect,
            in global coordinates, outside of which the popup menu should not be positioned. The menu
            definition should take this constraint rect into account as it calculates the menu bounds.
            If the gestalt bit is not set, the menu definition should use the bounds of the GDevice
            containing the menu's top left corner as a constraint on the menu's position.
            
            The hitPt parameter is a requested location for the top left corner of the menu. The
            coordinates of this parameter are swapped from their normal order; the h field of the
            hitPt parameter contains the vertical coordinate, and the v field of hitPt contains
            the horizontal coordinate.
            
            On entry, the whichItem parameter points at a menu item index which is requested to be
            the initial selection when the menu is displayed. After calculating the menu's bounds,
            the menu definition should write the menu's virtual top coordinate into the location
            pointed at by the whichItem parameter. If displaying the menu at the requested location
            does not require scrolling, the virtual top will be the same as the menu bounds top;
            if the menu must scroll to fit in the requested location, the virtual top may be different.
            
        kMenuCalcItemMsg
        
            menuRect        on exit, item bounds
            hitPt           unused
            whichItem       the item whose rect to calculate
            
            Sent when the Menu Manager needs to know the bounds of a menu item. The menu definition
            should calculate the size of the item specified by the whichItem parameter, and store 
            the bounds in the rect specified by the menuRect parameter.
            
            Some sample menu definition code provided by Apple has previously shown an implementation
            of this message that always sets the top left corner of the item bounds to (0,0), regardless
            of the item's actual position in the menu. For best future compatibility, menu definitions
            should begin storing an item bounds that gives the item's actual position in the menu based
            on the menu's current virtual top. For example, if the virtual menu top starts at 20, then
            the menu definition would calculate an item bounds for the first item that starts at (0,20),
            an item bounds for the second item that starts at (0,40), and so on. The menu definition
            should call GetMenuTrackingData to get the menu's current virtual position, and use zero
            for the menu top if GetMenuTrackingData returns an error.
            
        kMenuThemeSavvyMsg
        
            menuRect        unused
            hitPt           unused
            whichItem       on exit, indicates theme-savvyness of the menu definition
            
            Sent by the Menu Manager to determine whether the MDEF uses the Appearance Manager 
            menu-drawing functions to draw its menu. If it does, the menu definition should return
            kThemeSavvyMenuResponse in the location pointed to by whichItem. If the menu definition
            draws its own custom content without using the Appearance Manager menu-drawing functions,
            it should ignore this message.
    
    Low-memory Global Replacements
    
        Pre-Carbon menu definitions needed to use several low-memory globals to communicate with the
        Menu Manager. These globals have all been replaced or made obsolete in Carbon, as follows:
        
        MenuDisable
            
            MenuDisable is now set automatically by the Menu Manager using the value returned in the
            itemUnderMouse field of the MenuTrackingData structure passed to kMenuFindItemMsg.
            
        TopMenuItem
        AtMenuBottom
        
            TopMenuItem and AtMenuBottom are now set automatically by the Menu Manager using the
            values returned in the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData
            structure passed to kMenuDrawMsg and kMenuFindItemMsg.
            
        mbSaveLoc
    
            This undocumented low-memory global was used by pre-Carbon menu definitions to store
            the bounding rect of the currently selected item and to avoid drawing glitches while
            the menu definition was scrolling the contents of a menu that had submenus. The Menu
            Manager now automatically sets the selected item bounds using the value returned in
            the itemRect field of the MenuTrackingData structure passed to kMenuFindItemMsg. In
            order to correctly support scrolling of menus with submenus, a menu definition should
            verify, before scrolling the menu contents, that no submenus of the scrolling menu are
            currently visible. A menu definition can use GetMenuTrackingData to verify this condition,
            as follows:
            
                Boolean SafeToScroll( MenuRef menuBeingScrolled )
                (
                    MenuTrackingData lastMenuData;
                    return GetMenuTrackingData( NULL, &lastMenuData ) == noErr
                           && lastMenuData.menu == menuBeingScrolled;
                )
            
            If SafeToScroll returns false, the menu definition should not scroll the menu.
}
{$endc} {not TARGET_CPU_64}

const
{ menu defProc messages }
	kMenuDrawMsg = 0;
	kMenuSizeMsg = 2;
	kMenuPopUpMsg = 3;    { position the popup menu rect appropriately }
	kMenuCalcItemMsg = 5;
	kMenuThemeSavvyMsg = 7;    { is your MDEF theme-savvy?  If so, return kThemeSavvyMenuResponse in the whichItem parameter}
	kMenuInitMsg = 8;    { Return an error code in *whichItem to indicate success or failure. Only supported in Carbon. }
	kMenuDisposeMsg = 9;    { The menu is being destroyed. Only supported in Carbon.}
	kMenuFindItemMsg = 10;   { Determine which item is under the mouse. Only supported in Carbon.}
	kMenuHiliteItemMsg = 11;   { Hilite the specified item. Only supported in Carbon.}
	kMenuDrawItemsMsg = 12;   { Draw a range of items. Only supported in Carbon.}
	mDrawMsg = kMenuDrawMsg; { obsolete constant name}
	mSizeMsg = kMenuSizeMsg; { obsolete constant name}
	mPopUpMsg = kMenuPopUpMsg; { obsolete constant name}
	mCalcItemMsg = kMenuCalcItemMsg; { obsolete constant name}


const
	kThemeSavvyMenuResponse = $7473; { should be returned in *whichItem when handling kMenuThemeSavvyMsg}


{
 *  MDEFHiliteItemData
 *  
 *  Summary:
 *    The MDEFHiliteItemData structure contains information about which
 *    menu items should be hilited and unhilited as the user moves
 *    through the menus. It is used by menu definition functions, which
 *    receive a pointer to an MDEFHiliteItemData structure as the
 *    whichItem parameter during kMenuHiliteItemMsg.
 }
type
	MDEFHiliteItemData = record
{
   * The item that was previously selected. It should be redrawn in an
   * unhilited state. May be zero if no item was previously selected.
   }
		previousItem: MenuItemIndex;

  {
   * The item that is now selected. It should be redrawn in a hilited
   * state. May be zero if no item is now selected.
   }
		newItem: MenuItemIndex;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFHiliteItemDataPtr = ^MDEFHiliteItemData;
type
	HiliteMenuItemData = MDEFHiliteItemData;
	HiliteMenuItemDataPtr = MDEFHiliteItemDataPtr;

{
 *  MDEFDrawData
 *  
 *  Summary:
 *    The MDEFDrawData structure contains information needed to draw a
 *    menu. It is used by menu definition functions, which receive a
 *    pointer to an MDEFDrawData structure as the whichItem parameter
 *    during kMenuDrawMsg.
 }
type
	MDEFDrawData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the virtualMenuTop and virtualMenuBottom fields of this structure
   * while drawing the menu.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFDrawDataPtr = ^MDEFDrawData;

{
 *  MDEFFindItemData
 *  
 *  Summary:
 *    The MDEFFindItemData structure contains information needed to
 *    determine which item is currently selected by the user. It is
 *    used by menu definition functions, which receive a pointer to an
 *    MDEFDrawData structure as the whichItem parameter during
 *    kMenuFindItemMsg.
 }
type
	MDEFFindItemData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the itemSelected, itemUnderMouse, and itemRect fields of this
   * structure after determining which item is at the specified point.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into if it needs
   * to scroll the menu during the FindItem message. The Menu Manager
   * will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFFindItemDataPtr = ^MDEFFindItemData;

{
 *  MDEFDrawItemsData
 *  
 *  Summary:
 *    The MDEFDrawItemsData structure contains information about which
 *    menu items to redraw. It is used by menu definition functions,
 *    which receive a pointer to an MDEFDrawItemsData structure as the
 *    whichItem parameter during kMenuDrawItemsMsg.
 }
type
	MDEFDrawItemsData = record
{
   * The first item to draw.
   }
		firstItem: MenuItemIndex;

  {
   * The last item to draw.
   }
		lastItem: MenuItemIndex;

  {
   * Information about the menu's tracking state. The virtualMenuTop
   * and virtualMenuBottom fields in this structure will be the most
   * useful in handling the DrawItems message.
   }
		trackingData: MenuTrackingDataPtr;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF returns.
   }
		context: UnivPtr;
	end;
	MDEFDrawItemsDataPtr = ^MDEFDrawItemsData;
{ obsolete menu color table constants}
const
	mctAllItems = -98;  {search for all Items for the given ID}
	mctLastIDIndic = -99;   {last color table entry has this in ID field}

type
	MCEntry = record
		mctID: MenuID;                  {menu ID.  ID = 0 is the menu bar}
		mctItem: SInt16;                {menu Item. Item = 0 is a title}
		mctRGB1: RGBColor;                {usage depends on ID and Item}
		mctRGB2: RGBColor;                {usage depends on ID and Item}
		mctRGB3: RGBColor;                {usage depends on ID and Item}
		mctRGB4: RGBColor;                {usage depends on ID and Item}
		mctReserved: SInt16;            {reserved for internal use}
	end;
	MCEntryPtr = ^MCEntry;
type
	MCTable = array [0..0] of MCEntry;	{ARRAY [1..numEntries] of MCEntry}
	MCTablePtr = ^MCTable;
	MCTableHandle = ^MCTablePtr;
type
	MenuCRsrc = record
		numEntries: SInt16;             {number of entries}
		mcEntryRecs: MCTable;            {ARRAY [1..numEntries] of MCEntry}
	end;
	MenuCRsrcPtr = ^MenuCRsrc;
type
	MenuCRsrcHandle = ^MenuCRsrcPtr;


const
{
   * Menu item command ID to indicate a hierarchical menu; the item
   * icon ID is the hierarchical menu ID. This constant is deprecated.
   * Use SetMenuItemHierarchicalID or SetMenuItemHierarchicalMenu
   * instead of using this constant.
   }
	hMenuCmd = 27;

  {
   * A menu ID used with InsertMenu to insert a menu into the
   * hierarchical portion of the menubar. Rather than inserting menus
   * into the hierarchical portion of the menubar, we recommend that
   * your application use SetMenuItemHierarchicalMenu to directly
   * specify the submenu's MenuRef.
   }
	kInsertHierarchicalMenu = -1;

  {
   * The old name for kInsertHierarchicalMenu. This constant is
   * deprecated.
   }
	hierMenu = -1;

{$ifc not TARGET_CPU_64}
{
 *  NewMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateNewMenu instead of NewMenu.
 *  
 *  Summary:
 *    Creates a new menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewMenu( menuID_: MenuID; const (*var*) menuTitle: Str255 ): MenuRef; external name '_NewMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuTitleAsCFString instead of GetMenuTitle.
 *  
 *  Summary:
 *    Retrieves the title of a menu as a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuTitle( menu: MenuRef; var title: Str255 ): StringPtr; external name '_GetMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuTitleWithCFString instead of SetMenuTitle.
 *  
 *  Summary:
 *    Sets the title of a menu to a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function SetMenuTitle( menu: MenuRef; const (*var*) title: Str255 ): OSStatus; external name '_SetMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  [Mac]AppendMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AppendMenuItemTextWithCFString instead of AppendMenu. Note
 *    that unlike AppendMenu, AppendMenuItemTextWithCFString does not
 *    interpret metacharacters in the text.
 *  
 *  Summary:
 *    Appends a new menu item to a menu, using a Pascal string for the
 *    item's text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/cpstrnew
 }
procedure AppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacAppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  [Mac]InsertMenuItem()
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu into which to insert the item.
 *    
 *    itemString:
 *      The text of the new item. This string is parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/cpstrnew
 }
procedure InsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacInsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AppendMenuItemText()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
>>>>>>> graemeg/cpstrnew
 }
function AppendMenuItemText( menu: MenuRef; const (*var*) inString: Str255 ): OSStatus; external name '_AppendMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use InsertMenuItemTextWithCFString instead of InsertMenuItemText.
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu into which to insert the item.
 *    
 *    inString:
 *      The text of the new item. This string is not parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh; the new item's text becomes exactly the
 *      specified text.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
>>>>>>> graemeg/cpstrnew
 }
function InsertMenuItemText( menu: MenuRef; const (*var*) inString: Str255; afterItem: MenuItemIndex ): OSStatus; external name '_InsertMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of SetMenuItemText.
 *  
 *  Summary:
 *    Sets the text of a menu item using a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/cpstrnew
 }
procedure SetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; const (*var*) itemString: Str255 ); external name '_SetMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

<<<<<<< HEAD
{——————————————————————————————————————————————————————————————————————————————————————}
{  • Standard font menu (Carbon and later)                                             }
{  These APIs allow you to create and use the standard font menu.                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

const
	kHierarchicalFontMenuOption = $00000001;
=======
>>>>>>> graemeg/cpstrnew

{$ifc not TARGET_CPU_64}
{
 *  GetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of GetMenuItemText.
 *  
 *  Summary:
 *    Retrieves the text of a menu item as a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; var itemString: Str255 ); external name '_GetMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of setting an item's text
 *    as a Str255.
 *  
 *  Summary:
 *    Sets the text encoding that should be used to interpret a menu
 *    item's Pascal string-based text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; inScriptID: TextEncoding ): OSErr; external name '_SetMenuItemTextEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of retrieving a menu
 *    item's text as a Str255.
 *  
 *  Summary:
 *    Retrieves the text encoding that should be used to interpret a
 *    menu item's Pascal string-based text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
<<<<<<< HEAD
function GetFontFamilyFromMenuSelection( menu: MenuRef; item: MenuItemIndex; var outFontFamily: FMFontFamily; var outStyle: FMFontStyle ): OSStatus; external name '_GetFontFamilyFromMenuSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Contextual Menu routines and constants                                            }
{  available with Conxtextual Menu extension 1.0 and later                             }
{——————————————————————————————————————————————————————————————————————————————————————}
{ Gestalt Selector for classic 68K apps only. }
{ CFM apps should weak link and check the symbols. }
{$endc} {not TARGET_CPU_64}

const
	gestaltContextualMenuAttr = FourCharCode('cmnu');
	gestaltContextualMenuUnusedBit = 0;
	gestaltContextualMenuTrapAvailable = 1;
	gestaltContextualMenuHasAttributeAndModifierKeys = 2; { Contextual Menu Manager supports keyContextualMenuAttributes and keyContextualMenuModifiers }
	gestaltContextualMenuHasUnicodeSupport = 3; { Contextual Menu Manager supports typeUnicodeText and typeCFStringRef }
=======
function GetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; var outScriptID: TextEncoding ): OSErr; external name '_GetMenuItemTextEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  SetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemIconHandle instead of SetItemIcon.
 *  
 *  Summary:
 *    Sets the resource ID of an icon resource to be displayed in a
 *    menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemIcon( theMenu: MenuRef; item: MenuItemIndex; iconIndex: SInt16 ); external name '_SetItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemIconHandle instead of GetItemIcon.
 *  
 *  Summary:
 *    Retrieves the resource ID of an icon resource to be displayed in
 *    a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetItemIcon( theMenu: MenuRef; item: MenuItemIndex; var iconIndex: SInt16 ); external name '_GetItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemHierarchicalMenu instead of
 *    SetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Sets the menu ID of the submenu of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
<<<<<<< HEAD
const
{
   * Specifies the text of an item in a contextual menu. Data for this
   * parameter can be in one of several formats. In Mac OS 7/8/9.x and
   * Mac OS X 10.0 and 10.1, typeChar and typeIntlText are supported.
   * In Mac OS X 10.2 and later, typeStyledText, typeAEText,
   * typeUnicodeText, and typeCFStringRef are also supported. If you
   * provide data as typeCFStringRef, the Contextual Menu Manager will
   * automatically release the CFStringRef once the menu has been
   * displayed. If you need the CFStringRef to have a longer timetime,
   * your plugin should retain the CFStringRef before inserting it into
   * the AERecord.
   }
	keyContextualMenuName = FourCharCode('pnam');

  {
   * Specifies the command ID of an item in a contextual menu. Data for
   * this parameter should be typeSInt32.
   }
	keyContextualMenuCommandID = FourCharCode('cmcd');

  {
   * Specifies a contextual menu item with a submenu. Typically used
   * with AEPutKeyDesc to add an entire AEDesc containing the submenu
   * as the data for the parameter.
   }
	keyContextualMenuSubmenu = FourCharCode('cmsb');

  {
   * Specifies the menu item attributes of an item in a contextual
   * menu. Data for this parameter should be typeSInt32. Available in
   * Mac OS X 10.2 and later.
   }
	keyContextualMenuAttributes = FourCharCode('cmat');

  {
   * Specifies the modifier keys of an item in a contextual menu (see
   * kMenuShiftModifier, kMenuControlModifier, etc.) Data for this
   * parameter should be typeSInt32. Using this parameter together with
   * the keyContextualMenuAttributes parameter, it is possible to
   * create a contextual menu with dynamic items which change according
   * to the modifier keys pressed by the user. Available in Mac OS X
   * 10.2 and later.
   }
	keyContextualMenuModifiers = FourCharCode('cmmd');

{$ifc not TARGET_CPU_64}
=======
function SetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; inHierID: MenuID ): OSErr; external name '_SetMenuItemHierarchicalID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemHierarchicalMenu instead of
 *    GetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Retrieves the menu ID of the submenu of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; var outHierID: MenuID ): OSErr; external name '_GetMenuItemHierarchicalID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


>>>>>>> graemeg/cpstrnew
{
 *  GetMenuRetainCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFGetRetainCount instead of
 *    GetMenuRetainCount.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Applications should call IsShowContextualMenuClick when they
 *    receive non-null events. If IsShowContextualMenuClick returns
 *    true, your application should generate its own menu and Apple
 *    Event descriptor (AEDesc), and then call ContextualMenuSelect to
 *    display and track the contextual menu, and then handle the user's
 *    choice. 
 *    
 *    Apple recommends that you use the IsShowContextualMenuEvent API
 *    instead of this API. If you use the standard window Carbon event
 *    handler on your windows, you can also use a
 *    kEventControlContextualMenuClick event handler to be notified
 *    when a contextual menu click has occured; in this case, you do
 *    not need to call this API.
=======
 *  Summary:
 *    Returns the retain count of a menu.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to return.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> graemeg/cpstrnew
 }
function GetMenuRetainCount( inMenu: MenuRef ): ItemCount; external name '_GetMenuRetainCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RetainMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRetain instead of RetainMenu.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    This API is similar to IsShowContextualMenuClick, but takes a
 *    Carbon EventRef as its parameter instead of an EventRecord.
 *    
 *    
 *    In Mac OS X 10.3 and earlier, this API always returned false if
 *    the event kind was not kEventMouseDown,
 *    kEventWindowClickContentRgn, kEventWindowClickStructureRgn, or
 *    kEventWindowHandleContentClick. In Mac OS X 10.4 and later, this
 *    API no longer requires a specific event kind; it only requires
 *    that the event contain kEventParamMouseButton and
 *    kEventParamKeyModifiers parameters. 
 *    
 *    If you use the standard window Carbon event handler on your
 *    windows, you can also use a kEventControlContextualMenuClick
 *    event handler to be notified when a contextual menu click has
 *    occured; in this case, you do not need to call this API.
=======
 *  Summary:
 *    Increments the retain count of a menu.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to increment.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RetainMenu( inMenu: MenuRef ): OSStatus; external name '_RetainMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ReleaseMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRelease instead of ReleaseMenu.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    If IsShowContextualMenuClick returns true, you should call the
 *    ContextualMenuSelect API after generating your own menu and
 *    preparing an Apple Event descriptor (AEDesc) that describes the
 *    item for which your application is displaying a contextual menu.
 *    This descriptor may contain an object specifier or raw data and
 *    will be passed to all contextual menu plug-ins. 
 *    
 *    The system will add other items before displaying the contextual
 *    menu, and it will remove those items before returning, leaving
 *    the menu in its original state. 
 *    
 *    After all the system commands are added, the contextual menu is
 *    displayed and tracked. If the user selects one of the system
 *    items, it is handled by the system and the call returns as though
 *    the user didn't select anything from the menu. If the user
 *    selects any other item (or no item at all), the Menu Manager
 *    passes back appropriate values in the parameters
 *    outUserSelectionType, outMenuID, and outMenuItem. 
 *    
 *    Your application should provide visual feedback indicating the
 *    item that was clicked upon. For example, a click on an icon
 *    should highlight the icon, while a click on editable text should
 *    not eliminate the current selection. 
 *    
 *    If the outUserSelectionType parameter contains
 *    kCMMenuItemSelected, you should look at the outMenuID and
 *    outMenuItem parameters to determine what menu item the user chose
 *    and handle it appropriately. If the user selected
 *    kCMHelpItemSelected, you should open the proper Apple Guide
 *    sequence or other form of custom help. 
 *    
 *    There is a bug in ContextualMenuSelect in versions of Mac OS X
 *    prior to Mac OS X 10.4 that causes the value returned in the
 *    outUserSelectionType parameter to be set to kCMShowHelpSelected
 *    instead of kCMMenuItemSelected if the menu has a menu ID of zero
 *    and item three is selected. To avoid this problem on all versions
 *    of Mac OS X, simply use a non-zero menu ID. Note that when a new
 *    menu is created in InterfaceBuilder, its ID is automatically set
 *    to zero.
=======
 *  Summary:
 *    Decrements the retain count of a menu.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to decrement. If the retain count
 *      falls to zero, the menu is destroyed.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> graemeg/cpstrnew
 }
function ReleaseMenu( inMenu: MenuRef ): OSStatus; external name '_ReleaseMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


<<<<<<< HEAD
{$endc} {NOT TARGET_CPU_64}

{——————————————————————————————————————————————————————————————————————————————————————}
{  Contextual Menu Plugin Interface                                                    }
{  For Mac OS X 10.1, we support a new type of Contextual Menu Plugin: the CFPlugIn    }
{  based plugin.  Each plugin must be in a CFPlugIn in the Contextual Menu Items       }
{  folder in one of these paths:                                                       }
{      /System/Library/Contextual Menu Items/                                          }
{      /Library/Contextual Menu Items/                                                 }
{      ~/Library/Contextual Menu Items/                                                }
{  It must export the following functions using the following interface or a C++       }
{  interface inheriting from IUnknown and including similar functions.                 }
{——————————————————————————————————————————————————————————————————————————————————————}
(*
{ The Contextual Menu Manager will only load CFPlugIns of type kContextualMenuTypeID }
#define kContextualMenuTypeID ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x2F, 0x65, 0x22, 0xE9, 0x3E, 0x66, 0x11, 0xD5, \
  0x80, 0xA7, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 2F6522E9-3E66-11D5-80A7-003065B300BC }

{ Contextual Menu Plugins must implement this Contexual Menu Plugin Interface }
#define kContextualMenuInterfaceID    ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x32, 0x99, 0x7B, 0x62, 0x3E, 0x66, 0x11, 0xD5, \
  0xBE, 0xAB, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 32997B62-3E66-11D5-BEAB-003065B300BC }
=======
{
 *  InsertResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex ); external name '_InsertResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  AppendResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Appends menu items containing the names of resources of a
 *    specified resource type.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Implemented by client
 }
function CMPluginExamineContext( thisInstance: UnivPtr; const (*var*) inContext: AEDesc; var outCommandPairs: AEDescList ): OSStatus; external name '_CMPluginExamineContext';


{
 *  CMPluginHandleSelection()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginHandleSelection( thisInstance: UnivPtr; var inContext: AEDesc; inCommandID: SInt32 ): OSStatus; external name '_CMPluginHandleSelection';


{
 *  CMPluginPostMenuCleanup()
 *  
 *  Availability:
 *    Implemented by client
 }
procedure CMPluginPostMenuCleanup( thisInstance: UnivPtr ); external name '_CMPluginPostMenuCleanup';


{ previously in LowMem.h.  This functions return the menu ID of the hilited menu }
{$ifc not TARGET_CPU_64}
{
 *  LMGetTheMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetTheMenu: MenuID; external name '_LMGetTheMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  • DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
    A Short Course on Menu Definition Functions
    
    A menu definition function is used to implement a custom visual appearance for a menu.
    Menu definition functions are still supported in Carbon, but the messages sent to a
    menu definition function in Carbon are different than for a non-Carbon application.
    
    In general, Apple recommends using the system menu definition whenever possible.
    Menu definition functions will continue to be supported, but it is not easy to write
    a correct menu definition, especially one that attempts to imitate the standard system
    menu appearance. If you require special features in your menu that the system menu
    definition does not support, please mail <toolbox@apple.com> and describe your requirements;
    we would much rather enhance the system menu definition than have you write a custom one.
    The use of menu definition functions is deprecated in Mac OS X; in Mac OS X 10.3 and
    later, Apple recommends that you use a custom HIView to draw menu content.
    
    Menu definition functions before Carbon used the following messages:
    
        kMenuDrawMsg
        kMenuChooseMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuDrawItemMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    kMenuChooseMsg and kMenuDrawItemMsg are not supported in Carbon and are not sent to
    Carbon menu definitions. In Carbon, kMenuChooseMsg is replaced by kMenuFindItemMsg and
    kMenuHiliteItemMsg. Menu definition functions in Carbon use the following messages:
    
        kMenuInitMsg
        kMenuDisposeMsg
        kMenuFindItemMsg
        kMenuHiliteItemMsg
        kMenuDrawItemsMsg
        kMenuDrawMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    The rest of this documentation will focus on Carbon menu definitions only.
    
    Menu Definition Messages
    
        Carbon menu definition functions should support the following messages:
        
        kMenuInitMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       OSErr*
        
            Sent when a menu is created. This message gives the menu definition an opportunity
            to initialize its own state. If the menu definition encounters an error while
            initializing, it should set *whichItem to a non-zero error code; this will cause the
            Menu Manager to destroy the menu and return an error back from the API that was used
            to create the menu.
        
        kMenuDisposeMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       unused
            
            Sent when a menu is destroyed. This message gives the menu definition an opportunity
            to destroy its own data.
            
        kMenuFindItemMsg
            
            menuRect        menu bounds
            hitPt           point to hit-test
            whichItem       MDEFFindItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to know what item is under
            the mouse. The whichItem parameter is actually a pointer to a MenuTrackingData structure.
            On entry, the menu, virtualMenuTop, and virtualMenuBottom fields of this structure are
            valid. The menu definition should determine which item containst the given point, if any,
            and fill in the itemUnderMouse, itemSelected, and itemRect fields. If an item is found,
            the menu definition should always fill in the itemUnderMouse and itemRect fields. The
            menu definition should only fill in the itemSelected field if the item is available for
            selection; if it is unavailable (because it is disabled, or for some other reason), the
            menu definition should set the itemSelected field to zero.
            
            The values placed in the itemUnderMouse and itemSelected fields should be less than or
            equal to the number of items returned by CountMenuItems on this menu. The values placed
            in these two fields should be identical if both are non-zero. itemUnderMouse should always
            be non-zero if the mouse is actually over an item.
            
            The menu definition should not hilite the found item during this message. The Menu 
            Manager will send a separate kMenuHiliteItemMsg to request hiliting of the item.
            
            If the menu definition supports scrolling, it should scroll the menu during this message,
            and update the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData to
            indicate the menu's new scrolled position.
            
            If the menu definition uses QuickDraw to draw while scrolling, it should draw into the
            current port.
    
            If the menu definition uses CoreGraphics to draw while scrolling, it should use the
            CGContextRef passed in the context field of the MDEFHiliteItemData structure.
            
            Menu definitions must use the ScrollMenuImage API, if available, to scroll the menu contents.
            This API is available in CarbonLib 1.5 and later, and in Mac OS X 10.1 and later. ScrollMenuImage
            properly supports scrolling the alpha channel in the menu's image data. Use of QuickDraw's
            ScrollRect API to scroll the menu contents will result in the alpha channel being set to 0xFF
            (opaque) and the menu will no longer be transparent.
            
            The menu definition should not modify the menu field of the MenuTrackingData.
            
        kMenuHiliteItemMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFHiliteItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to hilite a newly selected
            item. The whichItem parameter is actually a pointer to a MDEFHiliteItemData structure.
            The menu definition should unhilite the item in the previousItem field, if non-zero,
            and hilite the item in the newItem field.
            
            Menu definitions should use the EraseMenuBackground API to erase the old menu contents
            before unhiliting a menu item, if the menu definition is using the Appearance Manager's
            menu drawing APIs. This is necessary because the background of a menu is transparent on
            Aqua, and if the old hilite is not erased first, it will show through the new unhilited
            menu background.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFHiliteItemData structure.
            
        kMenuDrawItemsMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawItemsData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw a portion of the
            menu. This message is used by the dynamic menu item support code in the Menu Manager;
            for example, if items five and six in a menu are a dynamic group, the Menu Manager will
            send a DrawItems message when the group's modifier key is pressed or released to redraw
            the appropriate item, but no other items in the menu.
            
            The whichItem parameter for this message is actually a pointer to a MDEFDrawItemsData
            structure. The menu definition should redraw the items starting with firstItem and
            ending with lastItem, inclusive.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawItemsData structure.
            
        kMenuDrawMsg
            
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw the entire menu.
            The whichItem parameter is actually a pointer to a MenuTrackingData structure. On entry,
            the menu field of this structure is valid. The menu definition should draw the menu and,
            if it supports scrolling, should also fill in the virtualMenuTop and virtualMenuBottom
            fields of the structure to indicate the menu's initial unscrolled position; typically, 
            virtualMenuTop would be set to the same value as the top coordinate of the menu rect,
            and virtualMenuBottom would be set to virtualMenuTop plus the virtual height of the menu.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawData structure.
            
        kMenuSizeMsg
        
            menuRect        unused
            hitPt           maximum width and height of the menu
            whichItem       unused
            
            Sent when the Menu Manager needs to determine the size of a menu. The menu definition
            should calculate the width and height of the menu and store the sizes into the menu with
            SetMenuWidth and SetMenuHeight.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the hitPt parameter to this message is the maximum width (hitPt.h) and height
            (hitPt.v) of the menu. The menu definition should ensure that the width and height that it
            places in the menu do not exceed these values. If the gestalt bit is not set, the menu
            definition should just use the main GDevice's width and height as constraints on the menu's
            width and height.
            
        kMenuPopUpMsg
        
            menuRect        on entry, constraints on the menu's position; on exit, menu bounds
            hitPt           requested menu location, with swapped coordinates
            whichItem       on entry, requested initial selection; on exit, virtual menu top
            
            Sent when the Menu Manager is about to display a popup menu. The menu definition should
            calculate the appropriate menu bounds to contain the menu based on the requested menu
            location and selected item. It should write the menuBounds into the rect given by the
            menuRect parameter.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the menuRect parameter on entry to this message contains a constraint rect,
            in global coordinates, outside of which the popup menu should not be positioned. The menu
            definition should take this constraint rect into account as it calculates the menu bounds.
            If the gestalt bit is not set, the menu definition should use the bounds of the GDevice
            containing the menu's top left corner as a constraint on the menu's position.
            
            The hitPt parameter is a requested location for the top left corner of the menu. The
            coordinates of this parameter are swapped from their normal order; the h field of the
            hitPt parameter contains the vertical coordinate, and the v field of hitPt contains
            the horizontal coordinate.
            
            On entry, the whichItem parameter points at a menu item index which is requested to be
            the initial selection when the menu is displayed. After calculating the menu's bounds,
            the menu definition should write the menu's virtual top coordinate into the location
            pointed at by the whichItem parameter. If displaying the menu at the requested location
            does not require scrolling, the virtual top will be the same as the menu bounds top;
            if the menu must scroll to fit in the requested location, the virtual top may be different.
            
        kMenuCalcItemMsg
        
            menuRect        on exit, item bounds
            hitPt           unused
            whichItem       the item whose rect to calculate
            
            Sent when the Menu Manager needs to know the bounds of a menu item. The menu definition
            should calculate the size of the item specified by the whichItem parameter, and store 
            the bounds in the rect specified by the menuRect parameter.
            
            Some sample menu definition code provided by Apple has previously shown an implementation
            of this message that always sets the top left corner of the item bounds to (0,0), regardless
            of the item's actual position in the menu. For best future compatibility, menu definitions
            should begin storing an item bounds that gives the item's actual position in the menu based
            on the menu's current virtual top. For example, if the virtual menu top starts at 20, then
            the menu definition would calculate an item bounds for the first item that starts at (0,20),
            an item bounds for the second item that starts at (0,40), and so on. The menu definition
            should call GetMenuTrackingData to get the menu's current virtual position, and use zero
            for the menu top if GetMenuTrackingData returns an error.
            
        kMenuThemeSavvyMsg
        
            menuRect        unused
            hitPt           unused
            whichItem       on exit, indicates theme-savvyness of the menu definition
            
            Sent by the Menu Manager to determine whether the MDEF uses the Appearance Manager 
            menu-drawing functions to draw its menu. If it does, the menu definition should return
            kThemeSavvyMenuResponse in the location pointed to by whichItem. If the menu definition
            draws its own custom content without using the Appearance Manager menu-drawing functions,
            it should ignore this message.
    
    Low-memory Global Replacements
    
        Pre-Carbon menu definitions needed to use several low-memory globals to communicate with the
        Menu Manager. These globals have all been replaced or made obsolete in Carbon, as follows:
        
        MenuDisable
            
            MenuDisable is now set automatically by the Menu Manager using the value returned in the
            itemUnderMouse field of the MenuTrackingData structure passed to kMenuFindItemMsg.
            
        TopMenuItem
        AtMenuBottom
        
            TopMenuItem and AtMenuBottom are now set automatically by the Menu Manager using the
            values returned in the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData
            structure passed to kMenuDrawMsg and kMenuFindItemMsg.
            
        mbSaveLoc
    
            This undocumented low-memory global was used by pre-Carbon menu definitions to store
            the bounding rect of the currently selected item and to avoid drawing glitches while
            the menu definition was scrolling the contents of a menu that had submenus. The Menu
            Manager now automatically sets the selected item bounds using the value returned in
            the itemRect field of the MenuTrackingData structure passed to kMenuFindItemMsg. In
            order to correctly support scrolling of menus with submenus, a menu definition should
            verify, before scrolling the menu contents, that no submenus of the scrolling menu are
            currently visible. A menu definition can use GetMenuTrackingData to verify this condition,
            as follows:
            
                Boolean SafeToScroll( MenuRef menuBeingScrolled )
                (
                    MenuTrackingData lastMenuData;
                    return GetMenuTrackingData( NULL, &lastMenuData ) == noErr
                           && lastMenuData.menu == menuBeingScrolled;
                )
            
            If SafeToScroll returns false, the menu definition should not scroll the menu.
}
{$endc} {not TARGET_CPU_64}

const
{ menu defProc messages }
	kMenuDrawMsg = 0;
	kMenuSizeMsg = 2;
	kMenuPopUpMsg = 3;    { position the popup menu rect appropriately }
	kMenuCalcItemMsg = 5;
	kMenuThemeSavvyMsg = 7;    { is your MDEF theme-savvy?  If so, return kThemeSavvyMenuResponse in the whichItem parameter}
	kMenuInitMsg = 8;    { Return an error code in *whichItem to indicate success or failure. Only supported in Carbon. }
	kMenuDisposeMsg = 9;    { The menu is being destroyed. Only supported in Carbon.}
	kMenuFindItemMsg = 10;   { Determine which item is under the mouse. Only supported in Carbon.}
	kMenuHiliteItemMsg = 11;   { Hilite the specified item. Only supported in Carbon.}
	kMenuDrawItemsMsg = 12;   { Draw a range of items. Only supported in Carbon.}
	mDrawMsg = kMenuDrawMsg; { obsolete constant name}
	mSizeMsg = kMenuSizeMsg; { obsolete constant name}
	mPopUpMsg = kMenuPopUpMsg; { obsolete constant name}
	mCalcItemMsg = kMenuCalcItemMsg; { obsolete constant name}


const
	kThemeSavvyMenuResponse = $7473; { should be returned in *whichItem when handling kMenuThemeSavvyMsg}


{
 *  MDEFHiliteItemData
 *  
 *  Summary:
 *    The MDEFHiliteItemData structure contains information about which
 *    menu items should be hilited and unhilited as the user moves
 *    through the menus. It is used by menu definition functions, which
 *    receive a pointer to an MDEFHiliteItemData structure as the
 *    whichItem parameter during kMenuHiliteItemMsg.
 }
type
	MDEFHiliteItemData = record
{
   * The item that was previously selected. It should be redrawn in an
   * unhilited state. May be zero if no item was previously selected.
   }
		previousItem: MenuItemIndex;

  {
   * The item that is now selected. It should be redrawn in a hilited
   * state. May be zero if no item is now selected.
   }
		newItem: MenuItemIndex;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFHiliteItemDataPtr = ^MDEFHiliteItemData;
type
	HiliteMenuItemData = MDEFHiliteItemData;
	HiliteMenuItemDataPtr = MDEFHiliteItemDataPtr;

{
 *  MDEFDrawData
 *  
 *  Summary:
 *    The MDEFDrawData structure contains information needed to draw a
 *    menu. It is used by menu definition functions, which receive a
 *    pointer to an MDEFDrawData structure as the whichItem parameter
 *    during kMenuDrawMsg.
 }
type
	MDEFDrawData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the virtualMenuTop and virtualMenuBottom fields of this structure
   * while drawing the menu.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFDrawDataPtr = ^MDEFDrawData;

{
 *  MDEFFindItemData
 *  
 *  Summary:
 *    The MDEFFindItemData structure contains information needed to
 *    determine which item is currently selected by the user. It is
 *    used by menu definition functions, which receive a pointer to an
 *    MDEFDrawData structure as the whichItem parameter during
 *    kMenuFindItemMsg.
 }
type
	MDEFFindItemData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the itemSelected, itemUnderMouse, and itemRect fields of this
   * structure after determining which item is at the specified point.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into if it needs
   * to scroll the menu during the FindItem message. The Menu Manager
   * will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFFindItemDataPtr = ^MDEFFindItemData;

{
 *  MDEFDrawItemsData
 *  
 *  Summary:
 *    The MDEFDrawItemsData structure contains information about which
 *    menu items to redraw. It is used by menu definition functions,
 *    which receive a pointer to an MDEFDrawItemsData structure as the
 *    whichItem parameter during kMenuDrawItemsMsg.
 }
type
	MDEFDrawItemsData = record
{
   * The first item to draw.
   }
		firstItem: MenuItemIndex;

  {
   * The last item to draw.
   }
		lastItem: MenuItemIndex;

  {
   * Information about the menu's tracking state. The virtualMenuTop
   * and virtualMenuBottom fields in this structure will be the most
   * useful in handling the DrawItems message.
   }
		trackingData: MenuTrackingDataPtr;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF returns.
   }
		context: UnivPtr;
	end;
	MDEFDrawItemsDataPtr = ^MDEFDrawItemsData;
{ obsolete menu color table constants}
const
	mctAllItems = -98;  {search for all Items for the given ID}
	mctLastIDIndic = -99;   {last color table entry has this in ID field}

type
	MCEntry = record
		mctID: MenuID;                  {menu ID.  ID = 0 is the menu bar}
		mctItem: SInt16;                {menu Item. Item = 0 is a title}
		mctRGB1: RGBColor;                {usage depends on ID and Item}
		mctRGB2: RGBColor;                {usage depends on ID and Item}
		mctRGB3: RGBColor;                {usage depends on ID and Item}
		mctRGB4: RGBColor;                {usage depends on ID and Item}
		mctReserved: SInt16;            {reserved for internal use}
	end;
	MCEntryPtr = ^MCEntry;
type
	MCTable = array [0..0] of MCEntry;	{ARRAY [1..numEntries] of MCEntry}
	MCTablePtr = ^MCTable;
	MCTableHandle = ^MCTablePtr;
type
	MenuCRsrc = record
		numEntries: SInt16;             {number of entries}
		mcEntryRecs: MCTable;            {ARRAY [1..numEntries] of MCEntry}
	end;
	MenuCRsrcPtr = ^MenuCRsrc;
type
	MenuCRsrcHandle = ^MenuCRsrcPtr;


const
{
   * Menu item command ID to indicate a hierarchical menu; the item
   * icon ID is the hierarchical menu ID. This constant is deprecated.
   * Use SetMenuItemHierarchicalID or SetMenuItemHierarchicalMenu
   * instead of using this constant.
   }
	hMenuCmd = 27;

  {
   * A menu ID used with InsertMenu to insert a menu into the
   * hierarchical portion of the menubar. Rather than inserting menus
   * into the hierarchical portion of the menubar, we recommend that
   * your application use SetMenuItemHierarchicalMenu to directly
   * specify the submenu's MenuRef.
   }
	kInsertHierarchicalMenu = -1;

  {
   * The old name for kInsertHierarchicalMenu. This constant is
   * deprecated.
   }
	hierMenu = -1;

{$ifc not TARGET_CPU_64}
{
 *  NewMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateNewMenu instead of NewMenu.
 *  
 *  Summary:
 *    Creates a new menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewMenu( menuID_: MenuID; const (*var*) menuTitle: Str255 ): MenuRef; external name '_NewMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuTitleAsCFString instead of GetMenuTitle.
 *  
 *  Summary:
 *    Retrieves the title of a menu as a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuTitle( menu: MenuRef; var title: Str255 ): StringPtr; external name '_GetMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuTitleWithCFString instead of SetMenuTitle.
 *  
 *  Summary:
 *    Sets the title of a menu to a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
<<<<<<< HEAD
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
<<<<<<< HEAD
function RemoveMenuCommandProperty( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType ): OSStatus; external name '_RemoveMenuCommandProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuItemData()
 *  
 *  Summary:
 *    Returns multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to retrieve many attributes of a menu item
 *    simultaneously; for example, it might be used by a menu
 *    definition function that needs to know how to draw a menu item.
 *    It is more efficient to use this function than to use the
 *    accessor functions for the individual attributes of the menu.
 *    This function returns a copy of the data in the menu, so any data
 *    in the MenuItemDataRec that is dynamically allocated (for
 *    example, the CFString item text) should be released by the caller.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu from which to copy data. If inIsCommandID is true, you
 *      may pass NULL for this parameter to search for an item in the
 *      root menu; if inIsCommandID is false, this parameter must be a
 *      valid MenuRef.
 *    
 *    inItem:
 *      The item or command ID from which to copy data.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and data is copied from the first item in the
 *      menu with that command ID. If inIsCommandID is false, the
 *      inItem parameter is interpreted as a menu item index, and data
 *      is copied for that item in the specified menu.
 *    
 *    ioData:
 *      Data is copied from the item and placed here. On entry, the
 *      whichData field of this structure should be initialized to
 *      indicate which data the caller would like returned. Individual
 *      fields of the MenuItemDataRec structure may require
 *      pre-initialization also; see the individual MenuItemDataFlags
 *      documentation for details.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuItemData( inMenu: MenuRef { can be NULL }; inItem: MenuItemID; inIsCommandID: Boolean; ioData: MenuItemDataPtr ): OSStatus; external name '_CopyMenuItemData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemData()
 *  
 *  Summary:
 *    Sets multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to set many attributes of a menu item
 *    simultaneously. It is more efficient to use this function than to
 *    use the accessor functions for the individual attributes of the
 *    menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to modify.
 *    
 *    inItem:
 *      The item or command ID to modify.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and the first item in the menu with that
 *      command ID. is modified. If inIsCommandID is false, the inItem
 *      parameter is interpreted as a menu item index, and the item
 *      with that index in the specified menu is modified.
 *    
 *    inData:
 *      The data to set. The caller should set the whichData field of
 *      this structure to indicate which data should be set. Only the
 *      fields of the structure corresponding to the non-zero whichData
 *      flags must be initialized; other fields are ignored.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemData( inMenu: MenuRef; inItem: MenuItemID; inIsCommandID: Boolean; const (*var*) inData: MenuItemDataRec ): OSStatus; external name '_SetMenuItemData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Dynamic menu item support (available in CarbonLib 1.1 and Mac OS X)               }
{  Dynamic menu item support allows a menu item to be redrawn while the menu is open   }
{  and visible to the user. Carbon contains automatic support for dynamic items based  }
{  on keyboard modifier state. If you need to implement your own variable item state   }
{  based on other system state, you can use these APIs to implement it.                }
{  To use the built-in support for dynamic items, you should create a menu containing  }
{  several contiguous items with the same command key but different text and modifier  }
{  keys. For example, you might have:                                                  }
{      Close       cmd-W                                                               }
{      Close All   cmd-option-W                                                        }
{  The Menu Manager automatically determines a dynamic menu group using the base       }
{  command key of a dynamic menu item ('W' in this case). Only a single item from      }
{  a dynamic group is visible at any time; the other items are hidden. The Menu        }
{  Manager uses the current keyboard modifiers to determine which item is visible.     }
{  It is also possible to have a dynamic menu item group that does not have command    }
{  keys, but only modifiers; for example, in the Finder's View menu, the Clean Up      }
{  and Clean Up Selection items have no command key, but are a dynamic menu item       }
{  group that changes visibility based on the Option modifier key.                     }
{  In this example, in your MENU resource, you would create the Close and Close All    }
{  items and give them each the letter 'W' as the command key; using an associated     }
{  xmnu resource, you would specify kMenuOptionModifier as the modifier for the        }
{  Close All item. You can do the same thing using InterfacerBuilder and nib-based     }
{  menus.                                                                              }
{  After loading your menu from the resource, you must set the kMenuItemAttrDynamic    }
{  flag for each dynamic item. In this example, you would use:                         }
{      ChangeMenuItemAttributes( menu, kCloseItem, kMenuItemAttrDynamic, 0 );          }
{      ChangeMenuItemAttributes( menu, kCloseAllItem, kMenuItemAttrDynamic, 0 );       }
{  The Menu Manager will now automatically display the correct item depending on       }
{  whether the Option key is pressed. The result from MenuSelect will be the item      }
{  number of the item that was visible when the menu closed.                           }
{  Note that:                                                                          }
{      - If you use InterfacerBuilder, you can set the kMenuItemAttrDynamic attribute  }
{        directly in the menu object in the nib, using the Inspector window for the    }
{        menu. You don't need to add the attribute after creating the menu.            }
{      - If your application requires Mac OS X 10.2 or CarbonLib 1.6 or later, you     }
{        can also use a version-one-format 'xmnu' resource, which allows settting      }
{        the menu item attributes in the 'xmnu'. Using a version one resource, you     }
{        can include the kMenuItemAttrDynamic attribute in the resource rather than    }
{        adding it after creating the menu.                                            }
{  If the Menu Manager's built-in support is not sufficient, you can also change the   }
{  attributes of an item yourself and use the UpdateInvalidMenuItems API to cause      }
{  the menu to redraw. Changes to a menu item (changing text, command key, style,      }
{  etc.) that occur while the menu is open will cause the menu item to be invalidated, }
{  but not redrawn. If you need to invalidate the item explicitly yourself, perhaps    }
{  because you have a custom MDEF that depends on state not accessed using Menu        }
{  Manager APIs, you can use the InvalidateMenuItems API. UpdateInvalidMenuItems will  }
{  scan the menu for invalid items and redraw each, clearing its invalid flag          }
{  afterwards.                                                                         }
{  If you need to change menu contents based on modifier key state without using the   }
{  built-in support in the Menu Manager, we recommend that you install a Carbon event  }
{  handler on your menu for the [kEventClassKeyboard, kEventRawKeyModifiersChanged]    }
{  event. Modifier key events are passed to the currently open menu before being sent  }
{  to the user focus target.                                                           }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  IsMenuItemInvalid()
 *  
 *  Summary:
 *    Determines if a menu item is invalid and should be redrawn.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose item to examine.
 *    
 *    inItem:
 *      The item to examine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuItemInvalid( inMenu: MenuRef; inItem: MenuItemIndex ): Boolean; external name '_IsMenuItemInvalid';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalidateMenuItems()
 *  
 *  Summary:
 *    Invalidates a group of menu items so that they will be redrawn
 *    when UpdateInvalidMenuItems is next called.
 *  
 *  Discussion:
 *    Menu items are automatically invalidated when their contents are
 *    changed using Menu Manager APIs while the menu is open. However,
 *    you might need to use this API if you have a custom MDEF that
 *    draws using state not contained in the menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose items to invalidate.
 *    
 *    inFirstItem:
 *      The first item to invalidate.
 *    
 *    inNumItems:
 *      The number of items to invalidate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvalidateMenuItems( inMenu: MenuRef; inFirstItem: MenuItemIndex; inNumItems: ItemCount ): OSStatus; external name '_InvalidateMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateInvalidMenuItems()
 *  
 *  Summary:
 *    Redraws the invalid items of an open menu.
 *  
 *  Discussion:
 *    It is not necessary to use UpdateInvalidMenuItems if you are
 *    using Carbon's built-in support for dynamic items based on
 *    modifier key state. However, if you are modifying items
 *    dynamically using your own implementation, you should call
 *    UpdateInvalidMenuItems after completing your modifications for a
 *    single menu. It will redraw any items that have been marked as
 *    invalid, and clear the invalid flag for those items.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to update.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateInvalidMenuItems( inMenu: MenuRef ): OSStatus; external name '_UpdateInvalidMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Standard font menu (Carbon and later)                                             }
{  These APIs allow you to create and use the standard font menu.                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

const
	kHierarchicalFontMenuOption = $00000001;

{$ifc not TARGET_CPU_64}
{
 *  CreateStandardFontMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateStandardFontMenu( menu: MenuRef; afterItem: MenuItemIndex; firstHierMenuID: MenuID; options: OptionBits; outHierMenuCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_CreateStandardFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateStandardFontMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateStandardFontMenu( menu: MenuRef; outHierMenuCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_UpdateStandardFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetFontFamilyFromMenuSelection()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetFontFamilyFromMenuSelection( menu: MenuRef; item: MenuItemIndex; var outFontFamily: FMFontFamily; var outStyle: FMFontStyle ): OSStatus; external name '_GetFontFamilyFromMenuSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Contextual Menu routines and constants                                            }
{  available with Conxtextual Menu extension 1.0 and later                             }
{——————————————————————————————————————————————————————————————————————————————————————}
{ Gestalt Selector for classic 68K apps only. }
{ CFM apps should weak link and check the symbols. }
{$endc} {not TARGET_CPU_64}

const
	gestaltContextualMenuAttr = FourCharCode('cmnu');
	gestaltContextualMenuUnusedBit = 0;
	gestaltContextualMenuTrapAvailable = 1;
	gestaltContextualMenuHasAttributeAndModifierKeys = 2; { Contextual Menu Manager supports keyContextualMenuAttributes and keyContextualMenuModifiers }
	gestaltContextualMenuHasUnicodeSupport = 3; { Contextual Menu Manager supports typeUnicodeText and typeCFStringRef }


{
 *  Summary:
 *    Values indicating what kind of help the application supports
 }
const
{
   * The application does not support any help. The Menu Manager will
   * put an appropriate help string into the contextual menu and
   * disable the Help item.
   }
	kCMHelpItemNoHelp = 0;

  {
   * The application supports Apple Guide help. The Menu Manager will
   * put the name of the main Guide file into the contextual menu and
   * enable the Help item.
   }
	kCMHelpItemAppleGuide = 1;

  {
   * The application supports some other form of help. In this case,
   * the application must also pass a valid string into the
   * inHelpItemString parameter of ContextualMenuSelect. This string
   * will be the text of the Help item in the contextual menu, and the
   * Help item will be enabled.
   }
	kCMHelpItemOtherHelp = 2;

  {
   * The application does not support any help. The Menu Manager will
   * remove the Help item from the contextual menu. This constant is
   * available in Mac OS X and CarbonLib 1.6, and later; however, in
   * CarbonLib it is translated to kCMHelpItemNoHelp, and the Help item
   * is only disabled, not removed.
   }
	kCMHelpItemRemoveHelp = 3;


{
 *  Summary:
 *    Values indicating what was chosen from a contextual menu
 }
const
{
   * The user did not choose an item from the contextual menu and the
   * application should do no further processing of the event.
   }
	kCMNothingSelected = 0;

  {
   * The user chose one of the application's items from the menu. The
   * application can examine the outMenuID and outMenuItem parameters
   * of ContextualMenuSelect to see what the menu selection was, and it
   * should then handle the selection appropriately.
   }
	kCMMenuItemSelected = 1;

  {
   * The user chose the Help item from the menu. The application should
   * open an Apple Guide database to a section appropriate for the
   * selection. If the application supports some other form of help, it
   * should be presented instead.
   }
	kCMShowHelpSelected = 3;


{
 *  Summary:
 *    AERecord keywords used by the ExamineContext method of a
 *    Contextual Menu plugin to specify menu item contents.
 }
const
{
   * Specifies the text of an item in a contextual menu. Data for this
   * parameter can be in one of several formats. In Mac OS 7/8/9.x and
   * Mac OS X 10.0 and 10.1, typeChar and typeIntlText are supported.
   * In Mac OS X 10.2 and later, typeStyledText, typeAEText,
   * typeUnicodeText, and typeCFStringRef are also supported. If you
   * provide data as typeCFStringRef, the Contextual Menu Manager will
   * automatically release the CFStringRef once the menu has been
   * displayed. If you need the CFStringRef to have a longer timetime,
   * your plugin should retain the CFStringRef before inserting it into
   * the AERecord.
   }
	keyContextualMenuName = FourCharCode('pnam');

  {
   * Specifies the command ID of an item in a contextual menu. Data for
   * this parameter should be typeSInt32.
   }
	keyContextualMenuCommandID = FourCharCode('cmcd');

  {
   * Specifies a contextual menu item with a submenu. Typically used
   * with AEPutKeyDesc to add an entire AEDesc containing the submenu
   * as the data for the parameter.
   }
	keyContextualMenuSubmenu = FourCharCode('cmsb');

  {
   * Specifies the menu item attributes of an item in a contextual
   * menu. Data for this parameter should be typeSInt32. Available in
   * Mac OS X 10.2 and later.
   }
	keyContextualMenuAttributes = FourCharCode('cmat');

  {
   * Specifies the modifier keys of an item in a contextual menu (see
   * kMenuShiftModifier, kMenuControlModifier, etc.) Data for this
   * parameter should be typeSInt32. Using this parameter together with
   * the keyContextualMenuAttributes parameter, it is possible to
   * create a contextual menu with dynamic items which change according
   * to the modifier keys pressed by the user. Available in Mac OS X
   * 10.2 and later.
   }
	keyContextualMenuModifiers = FourCharCode('cmmd');

{$ifc not TARGET_CPU_64}
{
 *  IsShowContextualMenuClick()
 *  
 *  Summary:
 *    Determines whether a particular EventRecord could invoke a
 *    contextual menu.
 *  
 *  Discussion:
 *    Applications should call IsShowContextualMenuClick when they
 *    receive non-null events. If IsShowContextualMenuClick returns
 *    true, your application should generate its own menu and Apple
 *    Event descriptor (AEDesc), and then call ContextualMenuSelect to
 *    display and track the contextual menu, and then handle the user's
 *    choice. 
 *    
 *    Apple recommends that you use the IsShowContextualMenuEvent API
 *    instead of this API. If you use the standard window Carbon event
 *    handler on your windows, you can also use a
 *    kEventControlContextualMenuClick event handler to be notified
 *    when a contextual menu click has occured; in this case, you do
 *    not need to call this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine.
 *  
 *  Result:
 *    Returns true if the application should display a contextual menu,
 *    false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function IsShowContextualMenuClick( const (*var*) inEvent: EventRecord ): Boolean; external name '_IsShowContextualMenuClick';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsShowContextualMenuEvent()
 *  
 *  Summary:
 *    Determines whether a particular EventRef could invoke a
 *    contextual menu.
 *  
 *  Discussion:
 *    This API is similar to IsShowContextualMenuClick, but takes a
 *    Carbon EventRef as its parameter instead of an EventRecord.
 *    
 *    
 *    In Mac OS X 10.3 and earlier, this API always returned false if
 *    the event kind was not kEventMouseDown,
 *    kEventWindowClickContentRgn, kEventWindowClickStructureRgn, or
 *    kEventWindowHandleContentClick. In Mac OS X 10.4 and later, this
 *    API no longer requires a specific event kind; it only requires
 *    that the event contain kEventParamMouseButton and
 *    kEventParamKeyModifiers parameters. 
 *    
 *    If you use the standard window Carbon event handler on your
 *    windows, you can also use a kEventControlContextualMenuClick
 *    event handler to be notified when a contextual menu click has
 *    occured; in this case, you do not need to call this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine.
 *  
 *  Result:
 *    Returns true if the application should display a contextual menu,
 *    false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsShowContextualMenuEvent( inEvent: EventRef ): Boolean; external name '_IsShowContextualMenuEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ContextualMenuSelect()
 *  
 *  Summary:
 *    Displays a contextual menu.
 *  
 *  Discussion:
 *    If IsShowContextualMenuClick returns true, you should call the
 *    ContextualMenuSelect API after generating your own menu and
 *    preparing an Apple Event descriptor (AEDesc) that describes the
 *    item for which your application is displaying a contextual menu.
 *    This descriptor may contain an object specifier or raw data and
 *    will be passed to all contextual menu plug-ins. 
 *    
 *    The system will add other items before displaying the contextual
 *    menu, and it will remove those items before returning, leaving
 *    the menu in its original state. 
 *    
 *    After all the system commands are added, the contextual menu is
 *    displayed and tracked. If the user selects one of the system
 *    items, it is handled by the system and the call returns as though
 *    the user didn't select anything from the menu. If the user
 *    selects any other item (or no item at all), the Menu Manager
 *    passes back appropriate values in the parameters
 *    outUserSelectionType, outMenuID, and outMenuItem. 
 *    
 *    Your application should provide visual feedback indicating the
 *    item that was clicked upon. For example, a click on an icon
 *    should highlight the icon, while a click on editable text should
 *    not eliminate the current selection. 
 *    
 *    If the outUserSelectionType parameter contains
 *    kCMMenuItemSelected, you should look at the outMenuID and
 *    outMenuItem parameters to determine what menu item the user chose
 *    and handle it appropriately. If the user selected
 *    kCMHelpItemSelected, you should open the proper Apple Guide
 *    sequence or other form of custom help. 
 *    
 *    There is a bug in ContextualMenuSelect in versions of Mac OS X
 *    prior to Mac OS X 10.4 that causes the value returned in the
 *    outUserSelectionType parameter to be set to kCMShowHelpSelected
 *    instead of kCMMenuItemSelected if the menu has a menu ID of zero
 *    and item three is selected. To avoid this problem on all versions
 *    of Mac OS X, simply use a non-zero menu ID. Note that when a new
 *    menu is created in InterfaceBuilder, its ID is automatically set
 *    to zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      A menu containing application commands to display. The caller
 *      creates this menu based on the current context, the mouse
 *      location, and the current selection (if it was the target of
 *      the mouse). If you pass nil, only system commands will be
 *      displayed. The menu should be added to the menu list as a
 *      pop-up menu (using the InsertMenu function).
 *    
 *    inGlobalLocation:
 *      The location (in global coordinates) of the mouse near which
 *      the menu is to be displayed.
 *    
 *    inReserved:
 *      Reserved for future use. Pass false for this parameter.
 *    
 *    inHelpType:
 *      An identifier specifying the type of help to be provided by the
 *      application; see kCMHelpItem constants.
 *    
 *    inHelpItemString:
 *      A string containing the text to be displayed for the help menu
 *      item. This string is unused unless you also pass the constant
 *      kCMHelpItemOtherHelp in the inHelpType parameter.
 *    
 *    inSelection:
 *      An object specifier for the current selection. This allows he
 *      system to examine the selection and add special system commands
 *      accordingly. Passing a value of nil indicates that no selection
 *      should be examined, and most likely, no special system actions
 *      will be included.
 *    
 *    outUserSelectionType:
 *      On exit, the value indicates what the user selected from the
 *      contextual menu; see kCMNothingSelected, kCMMenuItemSelected,
 *      and kCMShowHelpSelected.
 *    
 *    outMenuID:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu ID of the chosen item.
 *    
 *    outMenuItem:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu item chosen.
 *  
 *  Result:
 *    An OSStatus result code. ContextualMenuSelect returns the result
 *    code userCanceledErr and sets outUserSelectionType to
 *    kCMNothingSelected to indicate that the user did not select
 *    anything from the contextual menu and no further processing is
 *    needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function ContextualMenuSelect( inMenu: MenuRef; inGlobalLocation: Point; inReserved: Boolean; inHelpType: UInt32; {const} inHelpItemString:ConstStringPtr { can be NULL }; {const} inSelection: AEDescPtr { can be NULL }; var outUserSelectionType: UInt32; var outMenuID: MenuID; var outMenuItem: MenuItemIndex ): OSStatus; external name '_ContextualMenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {NOT TARGET_CPU_64}

{——————————————————————————————————————————————————————————————————————————————————————}
{  Contextual Menu Plugin Interface                                                    }
{  For Mac OS X 10.1, we support a new type of Contextual Menu Plugin: the CFPlugIn    }
{  based plugin.  Each plugin must be in a CFPlugIn in the Contextual Menu Items       }
{  folder in one of these paths:                                                       }
{      /System/Library/Contextual Menu Items/                                          }
{      /Library/Contextual Menu Items/                                                 }
{      ~/Library/Contextual Menu Items/                                                }
{  It must export the following functions using the following interface or a C++       }
{  interface inheriting from IUnknown and including similar functions.                 }
{——————————————————————————————————————————————————————————————————————————————————————}
(*
{ The Contextual Menu Manager will only load CFPlugIns of type kContextualMenuTypeID }
#define kContextualMenuTypeID ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x2F, 0x65, 0x22, 0xE9, 0x3E, 0x66, 0x11, 0xD5, \
  0x80, 0xA7, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 2F6522E9-3E66-11D5-80A7-003065B300BC }

{ Contextual Menu Plugins must implement this Contexual Menu Plugin Interface }
#define kContextualMenuInterfaceID    ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x32, 0x99, 0x7B, 0x62, 0x3E, 0x66, 0x11, 0xD5, \
  0xBE, 0xAB, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 32997B62-3E66-11D5-BEAB-003065B300BC }

#define CM_IUNKNOWN_C_GUTS \
   void *_reserved; \
 SInt32 (*QueryInterface)(void *thisPointer, CFUUIDBytes iid, void ** ppv); \
   UInt32 (*AddRef)(void *thisPointer); \
 UInt32 (*Release)(void *thisPointer)

{ The function table for the interface }
type
	ContextualMenuInterfaceStruct = record
CM_IUNKNOWN_C_GUTS;
    OSStatus ( *ExamineContext )(
          void*               thisInstance,
          const AEDesc*       inContext,
         AEDescList*         outCommandPairs );
 OSStatus ( *HandleSelection )(
         void*               thisInstance,
          AEDesc*             inContext,
         SInt32              inCommandID );
 void ( *PostMenuCleanup )(
         void*               thisInstance );
	end;
*)
{
 *  CMPluginExamineContext()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginExamineContext( thisInstance: UnivPtr; const (*var*) inContext: AEDesc; var outCommandPairs: AEDescList ): OSStatus; external name '_CMPluginExamineContext';


{
 *  CMPluginHandleSelection()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginHandleSelection( thisInstance: UnivPtr; var inContext: AEDesc; inCommandID: SInt32 ): OSStatus; external name '_CMPluginHandleSelection';


{
 *  CMPluginPostMenuCleanup()
 *  
 *  Availability:
 *    Implemented by client
 }
procedure CMPluginPostMenuCleanup( thisInstance: UnivPtr ); external name '_CMPluginPostMenuCleanup';


{ previously in LowMem.h.  This functions return the menu ID of the hilited menu }
{$ifc not TARGET_CPU_64}
{
 *  LMGetTheMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetTheMenu: MenuID; external name '_LMGetTheMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  • DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
    A Short Course on Menu Definition Functions
    
    A menu definition function is used to implement a custom visual appearance for a menu.
    Menu definition functions are still supported in Carbon, but the messages sent to a
    menu definition function in Carbon are different than for a non-Carbon application.
    
    In general, Apple recommends using the system menu definition whenever possible.
    Menu definition functions will continue to be supported, but it is not easy to write
    a correct menu definition, especially one that attempts to imitate the standard system
    menu appearance. If you require special features in your menu that the system menu
    definition does not support, please mail <toolbox@apple.com> and describe your requirements;
    we would much rather enhance the system menu definition than have you write a custom one.
    The use of menu definition functions is deprecated in Mac OS X; in Mac OS X 10.3 and
    later, Apple recommends that you use a custom HIView to draw menu content.
    
    Menu definition functions before Carbon used the following messages:
    
        kMenuDrawMsg
        kMenuChooseMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuDrawItemMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    kMenuChooseMsg and kMenuDrawItemMsg are not supported in Carbon and are not sent to
    Carbon menu definitions. In Carbon, kMenuChooseMsg is replaced by kMenuFindItemMsg and
    kMenuHiliteItemMsg. Menu definition functions in Carbon use the following messages:
    
        kMenuInitMsg
        kMenuDisposeMsg
        kMenuFindItemMsg
        kMenuHiliteItemMsg
        kMenuDrawItemsMsg
        kMenuDrawMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    The rest of this documentation will focus on Carbon menu definitions only.
    
    Menu Definition Messages
    
        Carbon menu definition functions should support the following messages:
        
        kMenuInitMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       OSErr*
        
            Sent when a menu is created. This message gives the menu definition an opportunity
            to initialize its own state. If the menu definition encounters an error while
            initializing, it should set *whichItem to a non-zero error code; this will cause the
            Menu Manager to destroy the menu and return an error back from the API that was used
            to create the menu.
        
        kMenuDisposeMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       unused
            
            Sent when a menu is destroyed. This message gives the menu definition an opportunity
            to destroy its own data.
            
        kMenuFindItemMsg
            
            menuRect        menu bounds
            hitPt           point to hit-test
            whichItem       MDEFFindItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to know what item is under
            the mouse. The whichItem parameter is actually a pointer to a MenuTrackingData structure.
            On entry, the menu, virtualMenuTop, and virtualMenuBottom fields of this structure are
            valid. The menu definition should determine which item containst the given point, if any,
            and fill in the itemUnderMouse, itemSelected, and itemRect fields. If an item is found,
            the menu definition should always fill in the itemUnderMouse and itemRect fields. The
            menu definition should only fill in the itemSelected field if the item is available for
            selection; if it is unavailable (because it is disabled, or for some other reason), the
            menu definition should set the itemSelected field to zero.
            
            The values placed in the itemUnderMouse and itemSelected fields should be less than or
            equal to the number of items returned by CountMenuItems on this menu. The values placed
            in these two fields should be identical if both are non-zero. itemUnderMouse should always
            be non-zero if the mouse is actually over an item.
            
            The menu definition should not hilite the found item during this message. The Menu 
            Manager will send a separate kMenuHiliteItemMsg to request hiliting of the item.
            
            If the menu definition supports scrolling, it should scroll the menu during this message,
            and update the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData to
            indicate the menu's new scrolled position.
            
            If the menu definition uses QuickDraw to draw while scrolling, it should draw into the
            current port.
    
            If the menu definition uses CoreGraphics to draw while scrolling, it should use the
            CGContextRef passed in the context field of the MDEFHiliteItemData structure.
            
            Menu definitions must use the ScrollMenuImage API, if available, to scroll the menu contents.
            This API is available in CarbonLib 1.5 and later, and in Mac OS X 10.1 and later. ScrollMenuImage
            properly supports scrolling the alpha channel in the menu's image data. Use of QuickDraw's
            ScrollRect API to scroll the menu contents will result in the alpha channel being set to 0xFF
            (opaque) and the menu will no longer be transparent.
            
            The menu definition should not modify the menu field of the MenuTrackingData.
            
        kMenuHiliteItemMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFHiliteItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to hilite a newly selected
            item. The whichItem parameter is actually a pointer to a MDEFHiliteItemData structure.
            The menu definition should unhilite the item in the previousItem field, if non-zero,
            and hilite the item in the newItem field.
            
            Menu definitions should use the EraseMenuBackground API to erase the old menu contents
            before unhiliting a menu item, if the menu definition is using the Appearance Manager's
            menu drawing APIs. This is necessary because the background of a menu is transparent on
            Aqua, and if the old hilite is not erased first, it will show through the new unhilited
            menu background.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFHiliteItemData structure.
            
        kMenuDrawItemsMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawItemsData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw a portion of the
            menu. This message is used by the dynamic menu item support code in the Menu Manager;
            for example, if items five and six in a menu are a dynamic group, the Menu Manager will
            send a DrawItems message when the group's modifier key is pressed or released to redraw
            the appropriate item, but no other items in the menu.
            
            The whichItem parameter for this message is actually a pointer to a MDEFDrawItemsData
            structure. The menu definition should redraw the items starting with firstItem and
            ending with lastItem, inclusive.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawItemsData structure.
            
        kMenuDrawMsg
            
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw the entire menu.
            The whichItem parameter is actually a pointer to a MenuTrackingData structure. On entry,
            the menu field of this structure is valid. The menu definition should draw the menu and,
            if it supports scrolling, should also fill in the virtualMenuTop and virtualMenuBottom
            fields of the structure to indicate the menu's initial unscrolled position; typically, 
            virtualMenuTop would be set to the same value as the top coordinate of the menu rect,
            and virtualMenuBottom would be set to virtualMenuTop plus the virtual height of the menu.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawData structure.
            
        kMenuSizeMsg
        
            menuRect        unused
            hitPt           maximum width and height of the menu
            whichItem       unused
            
            Sent when the Menu Manager needs to determine the size of a menu. The menu definition
            should calculate the width and height of the menu and store the sizes into the menu with
            SetMenuWidth and SetMenuHeight.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the hitPt parameter to this message is the maximum width (hitPt.h) and height
            (hitPt.v) of the menu. The menu definition should ensure that the width and height that it
            places in the menu do not exceed these values. If the gestalt bit is not set, the menu
            definition should just use the main GDevice's width and height as constraints on the menu's
            width and height.
            
        kMenuPopUpMsg
        
            menuRect        on entry, constraints on the menu's position; on exit, menu bounds
            hitPt           requested menu location, with swapped coordinates
            whichItem       on entry, requested initial selection; on exit, virtual menu top
            
            Sent when the Menu Manager is about to display a popup menu. The menu definition should
            calculate the appropriate menu bounds to contain the menu based on the requested menu
            location and selected item. It should write the menuBounds into the rect given by the
            menuRect parameter.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the menuRect parameter on entry to this message contains a constraint rect,
            in global coordinates, outside of which the popup menu should not be positioned. The menu
            definition should take this constraint rect into account as it calculates the menu bounds.
            If the gestalt bit is not set, the menu definition should use the bounds of the GDevice
            containing the menu's top left corner as a constraint on the menu's position.
            
            The hitPt parameter is a requested location for the top left corner of the menu. The
            coordinates of this parameter are swapped from their normal order; the h field of the
            hitPt parameter contains the vertical coordinate, and the v field of hitPt contains
            the horizontal coordinate.
            
            On entry, the whichItem parameter points at a menu item index which is requested to be
            the initial selection when the menu is displayed. After calculating the menu's bounds,
            the menu definition should write the menu's virtual top coordinate into the location
            pointed at by the whichItem parameter. If displaying the menu at the requested location
            does not require scrolling, the virtual top will be the same as the menu bounds top;
            if the menu must scroll to fit in the requested location, the virtual top may be different.
            
        kMenuCalcItemMsg
        
            menuRect        on exit, item bounds
            hitPt           unused
            whichItem       the item whose rect to calculate
            
            Sent when the Menu Manager needs to know the bounds of a menu item. The menu definition
            should calculate the size of the item specified by the whichItem parameter, and store 
            the bounds in the rect specified by the menuRect parameter.
            
            Some sample menu definition code provided by Apple has previously shown an implementation
            of this message that always sets the top left corner of the item bounds to (0,0), regardless
            of the item's actual position in the menu. For best future compatibility, menu definitions
            should begin storing an item bounds that gives the item's actual position in the menu based
            on the menu's current virtual top. For example, if the virtual menu top starts at 20, then
            the menu definition would calculate an item bounds for the first item that starts at (0,20),
            an item bounds for the second item that starts at (0,40), and so on. The menu definition
            should call GetMenuTrackingData to get the menu's current virtual position, and use zero
            for the menu top if GetMenuTrackingData returns an error.
            
        kMenuThemeSavvyMsg
        
            menuRect        unused
            hitPt           unused
            whichItem       on exit, indicates theme-savvyness of the menu definition
            
            Sent by the Menu Manager to determine whether the MDEF uses the Appearance Manager 
            menu-drawing functions to draw its menu. If it does, the menu definition should return
            kThemeSavvyMenuResponse in the location pointed to by whichItem. If the menu definition
            draws its own custom content without using the Appearance Manager menu-drawing functions,
            it should ignore this message.
    
    Low-memory Global Replacements
    
        Pre-Carbon menu definitions needed to use several low-memory globals to communicate with the
        Menu Manager. These globals have all been replaced or made obsolete in Carbon, as follows:
        
        MenuDisable
            
            MenuDisable is now set automatically by the Menu Manager using the value returned in the
            itemUnderMouse field of the MenuTrackingData structure passed to kMenuFindItemMsg.
            
        TopMenuItem
        AtMenuBottom
        
            TopMenuItem and AtMenuBottom are now set automatically by the Menu Manager using the
            values returned in the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData
            structure passed to kMenuDrawMsg and kMenuFindItemMsg.
            
        mbSaveLoc
    
            This undocumented low-memory global was used by pre-Carbon menu definitions to store
            the bounding rect of the currently selected item and to avoid drawing glitches while
            the menu definition was scrolling the contents of a menu that had submenus. The Menu
            Manager now automatically sets the selected item bounds using the value returned in
            the itemRect field of the MenuTrackingData structure passed to kMenuFindItemMsg. In
            order to correctly support scrolling of menus with submenus, a menu definition should
            verify, before scrolling the menu contents, that no submenus of the scrolling menu are
            currently visible. A menu definition can use GetMenuTrackingData to verify this condition,
            as follows:
            
                Boolean SafeToScroll( MenuRef menuBeingScrolled )
                (
                    MenuTrackingData lastMenuData;
                    return GetMenuTrackingData( NULL, &lastMenuData ) == noErr
                           && lastMenuData.menu == menuBeingScrolled;
                )
            
            If SafeToScroll returns false, the menu definition should not scroll the menu.
}
{$endc} {not TARGET_CPU_64}

const
{ menu defProc messages }
	kMenuDrawMsg = 0;
	kMenuSizeMsg = 2;
	kMenuPopUpMsg = 3;    { position the popup menu rect appropriately }
	kMenuCalcItemMsg = 5;
	kMenuThemeSavvyMsg = 7;    { is your MDEF theme-savvy?  If so, return kThemeSavvyMenuResponse in the whichItem parameter}
	kMenuInitMsg = 8;    { Return an error code in *whichItem to indicate success or failure. Only supported in Carbon. }
	kMenuDisposeMsg = 9;    { The menu is being destroyed. Only supported in Carbon.}
	kMenuFindItemMsg = 10;   { Determine which item is under the mouse. Only supported in Carbon.}
	kMenuHiliteItemMsg = 11;   { Hilite the specified item. Only supported in Carbon.}
	kMenuDrawItemsMsg = 12;   { Draw a range of items. Only supported in Carbon.}
	mDrawMsg = kMenuDrawMsg; { obsolete constant name}
	mSizeMsg = kMenuSizeMsg; { obsolete constant name}
	mPopUpMsg = kMenuPopUpMsg; { obsolete constant name}
	mCalcItemMsg = kMenuCalcItemMsg; { obsolete constant name}


const
	kThemeSavvyMenuResponse = $7473; { should be returned in *whichItem when handling kMenuThemeSavvyMsg}


{
 *  MDEFHiliteItemData
 *  
 *  Summary:
 *    The MDEFHiliteItemData structure contains information about which
 *    menu items should be hilited and unhilited as the user moves
 *    through the menus. It is used by menu definition functions, which
 *    receive a pointer to an MDEFHiliteItemData structure as the
 *    whichItem parameter during kMenuHiliteItemMsg.
 }
type
	MDEFHiliteItemData = record
{
   * The item that was previously selected. It should be redrawn in an
   * unhilited state. May be zero if no item was previously selected.
   }
		previousItem: MenuItemIndex;

  {
   * The item that is now selected. It should be redrawn in a hilited
   * state. May be zero if no item is now selected.
   }
		newItem: MenuItemIndex;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFHiliteItemDataPtr = ^MDEFHiliteItemData;
type
	HiliteMenuItemData = MDEFHiliteItemData;
	HiliteMenuItemDataPtr = MDEFHiliteItemDataPtr;

{
 *  MDEFDrawData
 *  
 *  Summary:
 *    The MDEFDrawData structure contains information needed to draw a
 *    menu. It is used by menu definition functions, which receive a
 *    pointer to an MDEFDrawData structure as the whichItem parameter
 *    during kMenuDrawMsg.
 }
type
	MDEFDrawData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the virtualMenuTop and virtualMenuBottom fields of this structure
   * while drawing the menu.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFDrawDataPtr = ^MDEFDrawData;

{
 *  MDEFFindItemData
 *  
 *  Summary:
 *    The MDEFFindItemData structure contains information needed to
 *    determine which item is currently selected by the user. It is
 *    used by menu definition functions, which receive a pointer to an
 *    MDEFDrawData structure as the whichItem parameter during
 *    kMenuFindItemMsg.
 }
type
	MDEFFindItemData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the itemSelected, itemUnderMouse, and itemRect fields of this
   * structure after determining which item is at the specified point.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into if it needs
   * to scroll the menu during the FindItem message. The Menu Manager
   * will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFFindItemDataPtr = ^MDEFFindItemData;

{
 *  MDEFDrawItemsData
 *  
 *  Summary:
 *    The MDEFDrawItemsData structure contains information about which
 *    menu items to redraw. It is used by menu definition functions,
 *    which receive a pointer to an MDEFDrawItemsData structure as the
 *    whichItem parameter during kMenuDrawItemsMsg.
 }
type
	MDEFDrawItemsData = record
{
   * The first item to draw.
   }
		firstItem: MenuItemIndex;

  {
   * The last item to draw.
   }
		lastItem: MenuItemIndex;

  {
   * Information about the menu's tracking state. The virtualMenuTop
   * and virtualMenuBottom fields in this structure will be the most
   * useful in handling the DrawItems message.
   }
		trackingData: MenuTrackingDataPtr;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF returns.
   }
		context: UnivPtr;
	end;
	MDEFDrawItemsDataPtr = ^MDEFDrawItemsData;
{ obsolete menu color table constants}
const
	mctAllItems = -98;  {search for all Items for the given ID}
	mctLastIDIndic = -99;   {last color table entry has this in ID field}

type
	MCEntry = record
		mctID: MenuID;                  {menu ID.  ID = 0 is the menu bar}
		mctItem: SInt16;                {menu Item. Item = 0 is a title}
		mctRGB1: RGBColor;                {usage depends on ID and Item}
		mctRGB2: RGBColor;                {usage depends on ID and Item}
		mctRGB3: RGBColor;                {usage depends on ID and Item}
		mctRGB4: RGBColor;                {usage depends on ID and Item}
		mctReserved: SInt16;            {reserved for internal use}
	end;
	MCEntryPtr = ^MCEntry;
type
	MCTable = array [0..0] of MCEntry;	{ARRAY [1..numEntries] of MCEntry}
	MCTablePtr = ^MCTable;
	MCTableHandle = ^MCTablePtr;
type
	MenuCRsrc = record
		numEntries: SInt16;             {number of entries}
		mcEntryRecs: MCTable;            {ARRAY [1..numEntries] of MCEntry}
	end;
	MenuCRsrcPtr = ^MenuCRsrc;
type
	MenuCRsrcHandle = ^MenuCRsrcPtr;


const
{
   * Menu item command ID to indicate a hierarchical menu; the item
   * icon ID is the hierarchical menu ID. This constant is deprecated.
   * Use SetMenuItemHierarchicalID or SetMenuItemHierarchicalMenu
   * instead of using this constant.
   }
	hMenuCmd = 27;

  {
   * A menu ID used with InsertMenu to insert a menu into the
   * hierarchical portion of the menubar. Rather than inserting menus
   * into the hierarchical portion of the menubar, we recommend that
   * your application use SetMenuItemHierarchicalMenu to directly
   * specify the submenu's MenuRef.
   }
	kInsertHierarchicalMenu = -1;

  {
   * The old name for kInsertHierarchicalMenu. This constant is
   * deprecated.
   }
	hierMenu = -1;

{$ifc not TARGET_CPU_64}
{
 *  NewMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateNewMenu instead of NewMenu.
 *  
 *  Summary:
 *    Creates a new menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewMenu( menuID_: MenuID; const (*var*) menuTitle: Str255 ): MenuRef; external name '_NewMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuTitleAsCFString instead of GetMenuTitle.
 *  
 *  Summary:
 *    Retrieves the title of a menu as a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
=======
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
<<<<<<< HEAD
=======
function RemoveMenuCommandProperty( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType ): OSStatus; external name '_RemoveMenuCommandProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuItemData()
 *  
 *  Summary:
 *    Returns multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to retrieve many attributes of a menu item
 *    simultaneously; for example, it might be used by a menu
 *    definition function that needs to know how to draw a menu item.
 *    It is more efficient to use this function than to use the
 *    accessor functions for the individual attributes of the menu.
 *    This function returns a copy of the data in the menu, so any data
 *    in the MenuItemDataRec that is dynamically allocated (for
 *    example, the CFString item text) should be released by the caller.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu from which to copy data. If inIsCommandID is true, you
 *      may pass NULL for this parameter to search for an item in the
 *      root menu; if inIsCommandID is false, this parameter must be a
 *      valid MenuRef.
 *    
 *    inItem:
 *      The item or command ID from which to copy data.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and data is copied from the first item in the
 *      menu with that command ID. If inIsCommandID is false, the
 *      inItem parameter is interpreted as a menu item index, and data
 *      is copied for that item in the specified menu.
 *    
 *    ioData:
 *      Data is copied from the item and placed here. On entry, the
 *      whichData field of this structure should be initialized to
 *      indicate which data the caller would like returned. Individual
 *      fields of the MenuItemDataRec structure may require
 *      pre-initialization also; see the individual MenuItemDataFlags
 *      documentation for details.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuItemData( inMenu: MenuRef { can be NULL }; inItem: MenuItemID; inIsCommandID: Boolean; ioData: MenuItemDataPtr ): OSStatus; external name '_CopyMenuItemData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemData()
 *  
 *  Summary:
 *    Sets multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to set many attributes of a menu item
 *    simultaneously. It is more efficient to use this function than to
 *    use the accessor functions for the individual attributes of the
 *    menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to modify.
 *    
 *    inItem:
 *      The item or command ID to modify.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and the first item in the menu with that
 *      command ID. is modified. If inIsCommandID is false, the inItem
 *      parameter is interpreted as a menu item index, and the item
 *      with that index in the specified menu is modified.
 *    
 *    inData:
 *      The data to set. The caller should set the whichData field of
 *      this structure to indicate which data should be set. Only the
 *      fields of the structure corresponding to the non-zero whichData
 *      flags must be initialized; other fields are ignored.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemData( inMenu: MenuRef; inItem: MenuItemID; inIsCommandID: Boolean; const (*var*) inData: MenuItemDataRec ): OSStatus; external name '_SetMenuItemData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Dynamic menu item support (available in CarbonLib 1.1 and Mac OS X)               }
{  Dynamic menu item support allows a menu item to be redrawn while the menu is open   }
{  and visible to the user. Carbon contains automatic support for dynamic items based  }
{  on keyboard modifier state. If you need to implement your own variable item state   }
{  based on other system state, you can use these APIs to implement it.                }
{  To use the built-in support for dynamic items, you should create a menu containing  }
{  several contiguous items with the same command key but different text and modifier  }
{  keys. For example, you might have:                                                  }
{      Close       cmd-W                                                               }
{      Close All   cmd-option-W                                                        }
{  The Menu Manager automatically determines a dynamic menu group using the base       }
{  command key of a dynamic menu item ('W' in this case). Only a single item from      }
{  a dynamic group is visible at any time; the other items are hidden. The Menu        }
{  Manager uses the current keyboard modifiers to determine which item is visible.     }
{  It is also possible to have a dynamic menu item group that does not have command    }
{  keys, but only modifiers; for example, in the Finder's View menu, the Clean Up      }
{  and Clean Up Selection items have no command key, but are a dynamic menu item       }
{  group that changes visibility based on the Option modifier key.                     }
{  In this example, in your MENU resource, you would create the Close and Close All    }
{  items and give them each the letter 'W' as the command key; using an associated     }
{  xmnu resource, you would specify kMenuOptionModifier as the modifier for the        }
{  Close All item. You can do the same thing using InterfacerBuilder and nib-based     }
{  menus.                                                                              }
{  After loading your menu from the resource, you must set the kMenuItemAttrDynamic    }
{  flag for each dynamic item. In this example, you would use:                         }
{      ChangeMenuItemAttributes( menu, kCloseItem, kMenuItemAttrDynamic, 0 );          }
{      ChangeMenuItemAttributes( menu, kCloseAllItem, kMenuItemAttrDynamic, 0 );       }
{  The Menu Manager will now automatically display the correct item depending on       }
{  whether the Option key is pressed. The result from MenuSelect will be the item      }
{  number of the item that was visible when the menu closed.                           }
{  Note that:                                                                          }
{      - If you use InterfacerBuilder, you can set the kMenuItemAttrDynamic attribute  }
{        directly in the menu object in the nib, using the Inspector window for the    }
{        menu. You don't need to add the attribute after creating the menu.            }
{      - If your application requires Mac OS X 10.2 or CarbonLib 1.6 or later, you     }
{        can also use a version-one-format 'xmnu' resource, which allows settting      }
{        the menu item attributes in the 'xmnu'. Using a version one resource, you     }
{        can include the kMenuItemAttrDynamic attribute in the resource rather than    }
{        adding it after creating the menu.                                            }
{  If the Menu Manager's built-in support is not sufficient, you can also change the   }
{  attributes of an item yourself and use the UpdateInvalidMenuItems API to cause      }
{  the menu to redraw. Changes to a menu item (changing text, command key, style,      }
{  etc.) that occur while the menu is open will cause the menu item to be invalidated, }
{  but not redrawn. If you need to invalidate the item explicitly yourself, perhaps    }
{  because you have a custom MDEF that depends on state not accessed using Menu        }
{  Manager APIs, you can use the InvalidateMenuItems API. UpdateInvalidMenuItems will  }
{  scan the menu for invalid items and redraw each, clearing its invalid flag          }
{  afterwards.                                                                         }
{  If you need to change menu contents based on modifier key state without using the   }
{  built-in support in the Menu Manager, we recommend that you install a Carbon event  }
{  handler on your menu for the [kEventClassKeyboard, kEventRawKeyModifiersChanged]    }
{  event. Modifier key events are passed to the currently open menu before being sent  }
{  to the user focus target.                                                           }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  IsMenuItemInvalid()
 *  
 *  Summary:
 *    Determines if a menu item is invalid and should be redrawn.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose item to examine.
 *    
 *    inItem:
 *      The item to examine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuItemInvalid( inMenu: MenuRef; inItem: MenuItemIndex ): Boolean; external name '_IsMenuItemInvalid';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalidateMenuItems()
 *  
 *  Summary:
 *    Invalidates a group of menu items so that they will be redrawn
 *    when UpdateInvalidMenuItems is next called.
 *  
 *  Discussion:
 *    Menu items are automatically invalidated when their contents are
 *    changed using Menu Manager APIs while the menu is open. However,
 *    you might need to use this API if you have a custom MDEF that
 *    draws using state not contained in the menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose items to invalidate.
 *    
 *    inFirstItem:
 *      The first item to invalidate.
 *    
 *    inNumItems:
 *      The number of items to invalidate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvalidateMenuItems( inMenu: MenuRef; inFirstItem: MenuItemIndex; inNumItems: ItemCount ): OSStatus; external name '_InvalidateMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateInvalidMenuItems()
 *  
 *  Summary:
 *    Redraws the invalid items of an open menu.
 *  
 *  Discussion:
 *    It is not necessary to use UpdateInvalidMenuItems if you are
 *    using Carbon's built-in support for dynamic items based on
 *    modifier key state. However, if you are modifying items
 *    dynamically using your own implementation, you should call
 *    UpdateInvalidMenuItems after completing your modifications for a
 *    single menu. It will redraw any items that have been marked as
 *    invalid, and clear the invalid flag for those items.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to update.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateInvalidMenuItems( inMenu: MenuRef ): OSStatus; external name '_UpdateInvalidMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Standard font menu (Carbon and later)                                             }
{  These APIs allow you to create and use the standard font menu.                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

const
	kHierarchicalFontMenuOption = $00000001;

{$ifc not TARGET_CPU_64}
{
 *  CreateStandardFontMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateStandardFontMenu( menu: MenuRef; afterItem: MenuItemIndex; firstHierMenuID: MenuID; options: OptionBits; outHierMenuCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_CreateStandardFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateStandardFontMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateStandardFontMenu( menu: MenuRef; outHierMenuCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_UpdateStandardFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetFontFamilyFromMenuSelection()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetFontFamilyFromMenuSelection( menu: MenuRef; item: MenuItemIndex; var outFontFamily: FMFontFamily; var outStyle: FMFontStyle ): OSStatus; external name '_GetFontFamilyFromMenuSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Contextual Menu routines and constants                                            }
{  available with Conxtextual Menu extension 1.0 and later                             }
{——————————————————————————————————————————————————————————————————————————————————————}
{ Gestalt Selector for classic 68K apps only. }
{ CFM apps should weak link and check the symbols. }
{$endc} {not TARGET_CPU_64}

const
	gestaltContextualMenuAttr = FourCharCode('cmnu');
	gestaltContextualMenuUnusedBit = 0;
	gestaltContextualMenuTrapAvailable = 1;
	gestaltContextualMenuHasAttributeAndModifierKeys = 2; { Contextual Menu Manager supports keyContextualMenuAttributes and keyContextualMenuModifiers }
	gestaltContextualMenuHasUnicodeSupport = 3; { Contextual Menu Manager supports typeUnicodeText and typeCFStringRef }


{
 *  Summary:
 *    Values indicating what kind of help the application supports
 }
const
{
   * The application does not support any help. The Menu Manager will
   * put an appropriate help string into the contextual menu and
   * disable the Help item.
   }
	kCMHelpItemNoHelp = 0;

  {
   * The application supports Apple Guide help. The Menu Manager will
   * put the name of the main Guide file into the contextual menu and
   * enable the Help item.
   }
	kCMHelpItemAppleGuide = 1;

  {
   * The application supports some other form of help. In this case,
   * the application must also pass a valid string into the
   * inHelpItemString parameter of ContextualMenuSelect. This string
   * will be the text of the Help item in the contextual menu, and the
   * Help item will be enabled.
   }
	kCMHelpItemOtherHelp = 2;

  {
   * The application does not support any help. The Menu Manager will
   * remove the Help item from the contextual menu. This constant is
   * available in Mac OS X and CarbonLib 1.6, and later; however, in
   * CarbonLib it is translated to kCMHelpItemNoHelp, and the Help item
   * is only disabled, not removed.
   }
	kCMHelpItemRemoveHelp = 3;


{
 *  Summary:
 *    Values indicating what was chosen from a contextual menu
 }
const
{
   * The user did not choose an item from the contextual menu and the
   * application should do no further processing of the event.
   }
	kCMNothingSelected = 0;

  {
   * The user chose one of the application's items from the menu. The
   * application can examine the outMenuID and outMenuItem parameters
   * of ContextualMenuSelect to see what the menu selection was, and it
   * should then handle the selection appropriately.
   }
	kCMMenuItemSelected = 1;

  {
   * The user chose the Help item from the menu. The application should
   * open an Apple Guide database to a section appropriate for the
   * selection. If the application supports some other form of help, it
   * should be presented instead.
   }
	kCMShowHelpSelected = 3;


{
 *  Summary:
 *    AERecord keywords used by the ExamineContext method of a
 *    Contextual Menu plugin to specify menu item contents.
 }
const
{
   * Specifies the text of an item in a contextual menu. Data for this
   * parameter can be in one of several formats. In Mac OS 7/8/9.x and
   * Mac OS X 10.0 and 10.1, typeChar and typeIntlText are supported.
   * In Mac OS X 10.2 and later, typeStyledText, typeAEText,
   * typeUnicodeText, and typeCFStringRef are also supported. If you
   * provide data as typeCFStringRef, the Contextual Menu Manager will
   * automatically release the CFStringRef once the menu has been
   * displayed. If you need the CFStringRef to have a longer timetime,
   * your plugin should retain the CFStringRef before inserting it into
   * the AERecord.
   }
	keyContextualMenuName = FourCharCode('pnam');

  {
   * Specifies the command ID of an item in a contextual menu. Data for
   * this parameter should be typeSInt32.
   }
	keyContextualMenuCommandID = FourCharCode('cmcd');

  {
   * Specifies a contextual menu item with a submenu. Typically used
   * with AEPutKeyDesc to add an entire AEDesc containing the submenu
   * as the data for the parameter.
   }
	keyContextualMenuSubmenu = FourCharCode('cmsb');

  {
   * Specifies the menu item attributes of an item in a contextual
   * menu. Data for this parameter should be typeSInt32. Available in
   * Mac OS X 10.2 and later.
   }
	keyContextualMenuAttributes = FourCharCode('cmat');

  {
   * Specifies the modifier keys of an item in a contextual menu (see
   * kMenuShiftModifier, kMenuControlModifier, etc.) Data for this
   * parameter should be typeSInt32. Using this parameter together with
   * the keyContextualMenuAttributes parameter, it is possible to
   * create a contextual menu with dynamic items which change according
   * to the modifier keys pressed by the user. Available in Mac OS X
   * 10.2 and later.
   }
	keyContextualMenuModifiers = FourCharCode('cmmd');

{$ifc not TARGET_CPU_64}
{
 *  IsShowContextualMenuClick()
 *  
 *  Summary:
 *    Determines whether a particular EventRecord could invoke a
 *    contextual menu.
 *  
 *  Discussion:
 *    Applications should call IsShowContextualMenuClick when they
 *    receive non-null events. If IsShowContextualMenuClick returns
 *    true, your application should generate its own menu and Apple
 *    Event descriptor (AEDesc), and then call ContextualMenuSelect to
 *    display and track the contextual menu, and then handle the user's
 *    choice. 
 *    
 *    Apple recommends that you use the IsShowContextualMenuEvent API
 *    instead of this API. If you use the standard window Carbon event
 *    handler on your windows, you can also use a
 *    kEventControlContextualMenuClick event handler to be notified
 *    when a contextual menu click has occured; in this case, you do
 *    not need to call this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine.
 *  
 *  Result:
 *    Returns true if the application should display a contextual menu,
 *    false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function IsShowContextualMenuClick( const (*var*) inEvent: EventRecord ): Boolean; external name '_IsShowContextualMenuClick';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsShowContextualMenuEvent()
 *  
 *  Summary:
 *    Determines whether a particular EventRef could invoke a
 *    contextual menu.
 *  
 *  Discussion:
 *    This API is similar to IsShowContextualMenuClick, but takes a
 *    Carbon EventRef as its parameter instead of an EventRecord.
 *    
 *    
 *    In Mac OS X 10.3 and earlier, this API always returned false if
 *    the event kind was not kEventMouseDown,
 *    kEventWindowClickContentRgn, kEventWindowClickStructureRgn, or
 *    kEventWindowHandleContentClick. In Mac OS X 10.4 and later, this
 *    API no longer requires a specific event kind; it only requires
 *    that the event contain kEventParamMouseButton and
 *    kEventParamKeyModifiers parameters. 
 *    
 *    If you use the standard window Carbon event handler on your
 *    windows, you can also use a kEventControlContextualMenuClick
 *    event handler to be notified when a contextual menu click has
 *    occured; in this case, you do not need to call this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine.
 *  
 *  Result:
 *    Returns true if the application should display a contextual menu,
 *    false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsShowContextualMenuEvent( inEvent: EventRef ): Boolean; external name '_IsShowContextualMenuEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ContextualMenuSelect()
 *  
 *  Summary:
 *    Displays a contextual menu.
 *  
 *  Discussion:
 *    If IsShowContextualMenuClick returns true, you should call the
 *    ContextualMenuSelect API after generating your own menu and
 *    preparing an Apple Event descriptor (AEDesc) that describes the
 *    item for which your application is displaying a contextual menu.
 *    This descriptor may contain an object specifier or raw data and
 *    will be passed to all contextual menu plug-ins. 
 *    
 *    The system will add other items before displaying the contextual
 *    menu, and it will remove those items before returning, leaving
 *    the menu in its original state. 
 *    
 *    After all the system commands are added, the contextual menu is
 *    displayed and tracked. If the user selects one of the system
 *    items, it is handled by the system and the call returns as though
 *    the user didn't select anything from the menu. If the user
 *    selects any other item (or no item at all), the Menu Manager
 *    passes back appropriate values in the parameters
 *    outUserSelectionType, outMenuID, and outMenuItem. 
 *    
 *    Your application should provide visual feedback indicating the
 *    item that was clicked upon. For example, a click on an icon
 *    should highlight the icon, while a click on editable text should
 *    not eliminate the current selection. 
 *    
 *    If the outUserSelectionType parameter contains
 *    kCMMenuItemSelected, you should look at the outMenuID and
 *    outMenuItem parameters to determine what menu item the user chose
 *    and handle it appropriately. If the user selected
 *    kCMHelpItemSelected, you should open the proper Apple Guide
 *    sequence or other form of custom help. 
 *    
 *    There is a bug in ContextualMenuSelect in versions of Mac OS X
 *    prior to Mac OS X 10.4 that causes the value returned in the
 *    outUserSelectionType parameter to be set to kCMShowHelpSelected
 *    instead of kCMMenuItemSelected if the menu has a menu ID of zero
 *    and item three is selected. To avoid this problem on all versions
 *    of Mac OS X, simply use a non-zero menu ID. Note that when a new
 *    menu is created in InterfaceBuilder, its ID is automatically set
 *    to zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      A menu containing application commands to display. The caller
 *      creates this menu based on the current context, the mouse
 *      location, and the current selection (if it was the target of
 *      the mouse). If you pass nil, only system commands will be
 *      displayed. The menu should be added to the menu list as a
 *      pop-up menu (using the InsertMenu function).
 *    
 *    inGlobalLocation:
 *      The location (in global coordinates) of the mouse near which
 *      the menu is to be displayed.
 *    
 *    inReserved:
 *      Reserved for future use. Pass false for this parameter.
 *    
 *    inHelpType:
 *      An identifier specifying the type of help to be provided by the
 *      application; see kCMHelpItem constants.
 *    
 *    inHelpItemString:
 *      A string containing the text to be displayed for the help menu
 *      item. This string is unused unless you also pass the constant
 *      kCMHelpItemOtherHelp in the inHelpType parameter.
 *    
 *    inSelection:
 *      An object specifier for the current selection. This allows he
 *      system to examine the selection and add special system commands
 *      accordingly. Passing a value of nil indicates that no selection
 *      should be examined, and most likely, no special system actions
 *      will be included.
 *    
 *    outUserSelectionType:
 *      On exit, the value indicates what the user selected from the
 *      contextual menu; see kCMNothingSelected, kCMMenuItemSelected,
 *      and kCMShowHelpSelected.
 *    
 *    outMenuID:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu ID of the chosen item.
 *    
 *    outMenuItem:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu item chosen.
 *  
 *  Result:
 *    An OSStatus result code. ContextualMenuSelect returns the result
 *    code userCanceledErr and sets outUserSelectionType to
 *    kCMNothingSelected to indicate that the user did not select
 *    anything from the contextual menu and no further processing is
 *    needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function ContextualMenuSelect( inMenu: MenuRef; inGlobalLocation: Point; inReserved: Boolean; inHelpType: UInt32; {const} inHelpItemString:ConstStringPtr { can be NULL }; {const} inSelection: AEDescPtr { can be NULL }; var outUserSelectionType: UInt32; var outMenuID: MenuID; var outMenuItem: MenuItemIndex ): OSStatus; external name '_ContextualMenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {NOT TARGET_CPU_64}

{——————————————————————————————————————————————————————————————————————————————————————}
{  Contextual Menu Plugin Interface                                                    }
{  For Mac OS X 10.1, we support a new type of Contextual Menu Plugin: the CFPlugIn    }
{  based plugin.  Each plugin must be in a CFPlugIn in the Contextual Menu Items       }
{  folder in one of these paths:                                                       }
{      /System/Library/Contextual Menu Items/                                          }
{      /Library/Contextual Menu Items/                                                 }
{      ~/Library/Contextual Menu Items/                                                }
{  It must export the following functions using the following interface or a C++       }
{  interface inheriting from IUnknown and including similar functions.                 }
{——————————————————————————————————————————————————————————————————————————————————————}
(*
{ The Contextual Menu Manager will only load CFPlugIns of type kContextualMenuTypeID }
#define kContextualMenuTypeID ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x2F, 0x65, 0x22, 0xE9, 0x3E, 0x66, 0x11, 0xD5, \
  0x80, 0xA7, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 2F6522E9-3E66-11D5-80A7-003065B300BC }

{ Contextual Menu Plugins must implement this Contexual Menu Plugin Interface }
#define kContextualMenuInterfaceID    ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x32, 0x99, 0x7B, 0x62, 0x3E, 0x66, 0x11, 0xD5, \
  0xBE, 0xAB, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 32997B62-3E66-11D5-BEAB-003065B300BC }

#define CM_IUNKNOWN_C_GUTS \
   void *_reserved; \
 SInt32 (*QueryInterface)(void *thisPointer, CFUUIDBytes iid, void ** ppv); \
   UInt32 (*AddRef)(void *thisPointer); \
 UInt32 (*Release)(void *thisPointer)

{ The function table for the interface }
type
	ContextualMenuInterfaceStruct = record
CM_IUNKNOWN_C_GUTS;
    OSStatus ( *ExamineContext )(
          void*               thisInstance,
          const AEDesc*       inContext,
         AEDescList*         outCommandPairs );
 OSStatus ( *HandleSelection )(
         void*               thisInstance,
          AEDesc*             inContext,
         SInt32              inCommandID );
 void ( *PostMenuCleanup )(
         void*               thisInstance );
	end;
*)
{
 *  CMPluginExamineContext()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginExamineContext( thisInstance: UnivPtr; const (*var*) inContext: AEDesc; var outCommandPairs: AEDescList ): OSStatus; external name '_CMPluginExamineContext';


{
 *  CMPluginHandleSelection()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginHandleSelection( thisInstance: UnivPtr; var inContext: AEDesc; inCommandID: SInt32 ): OSStatus; external name '_CMPluginHandleSelection';
=======
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit Menus;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
>>>>>>> origin/fixes_2.4


{
 *  CMPluginPostMenuCleanup()
 *  
 *  Availability:
 *    Implemented by client
 }
procedure CMPluginPostMenuCleanup( thisInstance: UnivPtr ); external name '_CMPluginPostMenuCleanup';

<<<<<<< HEAD
=======
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

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}

>>>>>>> origin/fixes_2.4

{ previously in LowMem.h.  This functions return the menu ID of the hilited menu }
{$ifc not TARGET_CPU_64}
{
 *  LMGetTheMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetTheMenu: MenuID; external name '_LMGetTheMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD

{--------------------------------------------------------------------------------------}
{  • DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
=======
>>>>>>> graemeg/fixes_2_2
=======
=======
{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
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
>>>>>>> origin/fixes_2.4
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,AEDataModel,Appearance,CFBase,CFArray,CFAttributedString,CGContext,CTFont,ATSTypes,Events,QuickdrawTypes,TextCommon,Collections,CarbonEventsCore,HIObject;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


{
 *  Menu Manager
 }
{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Constants                                                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
{
    A Short Course on Menu Definition Functions
    
    A menu definition function is used to implement a custom visual appearance for a menu.
    Menu definition functions are still supported in Carbon, but the messages sent to a
    menu definition function in Carbon are different than for a non-Carbon application.
    
    In general, Apple recommends using the system menu definition whenever possible.
    Menu definition functions will continue to be supported, but it is not easy to write
    a correct menu definition, especially one that attempts to imitate the standard system
    menu appearance. If you require special features in your menu that the system menu
    definition does not support, please mail <toolbox@apple.com> and describe your requirements;
    we would much rather enhance the system menu definition than have you write a custom one.
<<<<<<< HEAD
<<<<<<< HEAD
    The use of menu definition functions is deprecated in Mac OS X; in Mac OS X 10.3 and
    later, Apple recommends that you use a custom HIView to draw menu content.
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    
    Menu definition functions before Carbon used the following messages:
    
        kMenuDrawMsg
        kMenuChooseMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuDrawItemMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    kMenuChooseMsg and kMenuDrawItemMsg are not supported in Carbon and are not sent to
    Carbon menu definitions. In Carbon, kMenuChooseMsg is replaced by kMenuFindItemMsg and
    kMenuHiliteItemMsg. Menu definition functions in Carbon use the following messages:
    
        kMenuInitMsg
        kMenuDisposeMsg
        kMenuFindItemMsg
        kMenuHiliteItemMsg
        kMenuDrawItemsMsg
        kMenuDrawMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    The rest of this documentation will focus on Carbon menu definitions only.
    
    Menu Definition Messages
    
        Carbon menu definition functions should support the following messages:
        
        kMenuInitMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       OSErr*
        
            Sent when a menu is created. This message gives the menu definition an opportunity
            to initialize its own state. If the menu definition encounters an error while
            initializing, it should set *whichItem to a non-zero error code; this will cause the
            Menu Manager to destroy the menu and return an error back from the API that was used
            to create the menu.
        
        kMenuDisposeMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       unused
            
            Sent when a menu is destroyed. This message gives the menu definition an opportunity
            to destroy its own data.
            
        kMenuFindItemMsg
            
            menuRect        menu bounds
            hitPt           point to hit-test
            whichItem       MDEFFindItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to know what item is under
            the mouse. The whichItem parameter is actually a pointer to a MenuTrackingData structure.
            On entry, the menu, virtualMenuTop, and virtualMenuBottom fields of this structure are
            valid. The menu definition should determine which item containst the given point, if any,
            and fill in the itemUnderMouse, itemSelected, and itemRect fields. If an item is found,
            the menu definition should always fill in the itemUnderMouse and itemRect fields. The
            menu definition should only fill in the itemSelected field if the item is available for
            selection; if it is unavailable (because it is disabled, or for some other reason), the
            menu definition should set the itemSelected field to zero.
            
            The values placed in the itemUnderMouse and itemSelected fields should be less than or
            equal to the number of items returned by CountMenuItems on this menu. The values placed
            in these two fields should be identical if both are non-zero. itemUnderMouse should always
            be non-zero if the mouse is actually over an item.
            
            The menu definition should not hilite the found item during this message. The Menu 
            Manager will send a separate kMenuHiliteItemMsg to request hiliting of the item.
            
            If the menu definition supports scrolling, it should scroll the menu during this message,
            and update the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData to
            indicate the menu's new scrolled position.
            
            If the menu definition uses QuickDraw to draw while scrolling, it should draw into the
            current port.
    
            If the menu definition uses CoreGraphics to draw while scrolling, it should use the
            CGContextRef passed in the context field of the MDEFHiliteItemData structure.
            
            Menu definitions must use the ScrollMenuImage API, if available, to scroll the menu contents.
            This API is available in CarbonLib 1.5 and later, and in Mac OS X 10.1 and later. ScrollMenuImage
            properly supports scrolling the alpha channel in the menu's image data. Use of QuickDraw's
            ScrollRect API to scroll the menu contents will result in the alpha channel being set to 0xFF
            (opaque) and the menu will no longer be transparent.
            
            The menu definition should not modify the menu field of the MenuTrackingData.
            
        kMenuHiliteItemMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFHiliteItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to hilite a newly selected
            item. The whichItem parameter is actually a pointer to a MDEFHiliteItemData structure.
            The menu definition should unhilite the item in the previousItem field, if non-zero,
            and hilite the item in the newItem field.
            
            Menu definitions should use the EraseMenuBackground API to erase the old menu contents
            before unhiliting a menu item, if the menu definition is using the Appearance Manager's
            menu drawing APIs. This is necessary because the background of a menu is transparent on
            Aqua, and if the old hilite is not erased first, it will show through the new unhilited
            menu background.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFHiliteItemData structure.
            
        kMenuDrawItemsMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawItemsData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw a portion of the
            menu. This message is used by the dynamic menu item support code in the Menu Manager;
            for example, if items five and six in a menu are a dynamic group, the Menu Manager will
            send a DrawItems message when the group's modifier key is pressed or released to redraw
            the appropriate item, but no other items in the menu.
            
            The whichItem parameter for this message is actually a pointer to a MDEFDrawItemsData
            structure. The menu definition should redraw the items starting with firstItem and
            ending with lastItem, inclusive.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawItemsData structure.
            
        kMenuDrawMsg
            
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw the entire menu.
            The whichItem parameter is actually a pointer to a MenuTrackingData structure. On entry,
            the menu field of this structure is valid. The menu definition should draw the menu and,
            if it supports scrolling, should also fill in the virtualMenuTop and virtualMenuBottom
            fields of the structure to indicate the menu's initial unscrolled position; typically, 
            virtualMenuTop would be set to the same value as the top coordinate of the menu rect,
            and virtualMenuBottom would be set to virtualMenuTop plus the virtual height of the menu.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawData structure.
            
        kMenuSizeMsg
        
            menuRect        unused
            hitPt           maximum width and height of the menu
            whichItem       unused
            
            Sent when the Menu Manager needs to determine the size of a menu. The menu definition
            should calculate the width and height of the menu and store the sizes into the menu with
            SetMenuWidth and SetMenuHeight.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the hitPt parameter to this message is the maximum width (hitPt.h) and height
            (hitPt.v) of the menu. The menu definition should ensure that the width and height that it
            places in the menu do not exceed these values. If the gestalt bit is not set, the menu
            definition should just use the main GDevice's width and height as constraints on the menu's
            width and height.
            
        kMenuPopUpMsg
        
            menuRect        on entry, constraints on the menu's position; on exit, menu bounds
            hitPt           requested menu location, with swapped coordinates
            whichItem       on entry, requested initial selection; on exit, virtual menu top
            
            Sent when the Menu Manager is about to display a popup menu. The menu definition should
            calculate the appropriate menu bounds to contain the menu based on the requested menu
            location and selected item. It should write the menuBounds into the rect given by the
            menuRect parameter.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the menuRect parameter on entry to this message contains a constraint rect,
            in global coordinates, outside of which the popup menu should not be positioned. The menu
            definition should take this constraint rect into account as it calculates the menu bounds.
            If the gestalt bit is not set, the menu definition should use the bounds of the GDevice
            containing the menu's top left corner as a constraint on the menu's position.
            
            The hitPt parameter is a requested location for the top left corner of the menu. The
            coordinates of this parameter are swapped from their normal order; the h field of the
            hitPt parameter contains the vertical coordinate, and the v field of hitPt contains
            the horizontal coordinate.
            
            On entry, the whichItem parameter points at a menu item index which is requested to be
            the initial selection when the menu is displayed. After calculating the menu's bounds,
            the menu definition should write the menu's virtual top coordinate into the location
            pointed at by the whichItem parameter. If displaying the menu at the requested location
            does not require scrolling, the virtual top will be the same as the menu bounds top;
            if the menu must scroll to fit in the requested location, the virtual top may be different.
            
        kMenuCalcItemMsg
        
            menuRect        on exit, item bounds
            hitPt           unused
            whichItem       the item whose rect to calculate
            
            Sent when the Menu Manager needs to know the bounds of a menu item. The menu definition
            should calculate the size of the item specified by the whichItem parameter, and store 
            the bounds in the rect specified by the menuRect parameter.
            
            Some sample menu definition code provided by Apple has previously shown an implementation
            of this message that always sets the top left corner of the item bounds to (0,0), regardless
            of the item's actual position in the menu. For best future compatibility, menu definitions
            should begin storing an item bounds that gives the item's actual position in the menu based
            on the menu's current virtual top. For example, if the virtual menu top starts at 20, then
            the menu definition would calculate an item bounds for the first item that starts at (0,20),
            an item bounds for the second item that starts at (0,40), and so on. The menu definition
            should call GetMenuTrackingData to get the menu's current virtual position, and use zero
            for the menu top if GetMenuTrackingData returns an error.
            
        kMenuThemeSavvyMsg
        
            menuRect        unused
            hitPt           unused
            whichItem       on exit, indicates theme-savvyness of the menu definition
            
            Sent by the Menu Manager to determine whether the MDEF uses the Appearance Manager 
            menu-drawing functions to draw its menu. If it does, the menu definition should return
            kThemeSavvyMenuResponse in the location pointed to by whichItem. If the menu definition
            draws its own custom content without using the Appearance Manager menu-drawing functions,
            it should ignore this message.
    
    Low-memory Global Replacements
    
        Pre-Carbon menu definitions needed to use several low-memory globals to communicate with the
        Menu Manager. These globals have all been replaced or made obsolete in Carbon, as follows:
        
        MenuDisable
            
            MenuDisable is now set automatically by the Menu Manager using the value returned in the
            itemUnderMouse field of the MenuTrackingData structure passed to kMenuFindItemMsg.
            
<<<<<<< HEAD
        TopMenuItem
        AtMenuBottom
        
            TopMenuItem and AtMenuBottom are now set automatically by the Menu Manager using the
            values returned in the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData
            structure passed to kMenuDrawMsg and kMenuFindItemMsg.
            
        mbSaveLoc
    
            This undocumented low-memory global was used by pre-Carbon menu definitions to store
            the bounding rect of the currently selected item and to avoid drawing glitches while
            the menu definition was scrolling the contents of a menu that had submenus. The Menu
            Manager now automatically sets the selected item bounds using the value returned in
            the itemRect field of the MenuTrackingData structure passed to kMenuFindItemMsg. In
            order to correctly support scrolling of menus with submenus, a menu definition should
            verify, before scrolling the menu contents, that no submenus of the scrolling menu are
            currently visible. A menu definition can use GetMenuTrackingData to verify this condition,
            as follows:
            
                Boolean SafeToScroll( MenuRef menuBeingScrolled )
                (
                    MenuTrackingData lastMenuData;
                    return GetMenuTrackingData( NULL, &lastMenuData ) == noErr
                           && lastMenuData.menu == menuBeingScrolled;
                )
            
            If SafeToScroll returns false, the menu definition should not scroll the menu.
}
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

const
{ menu defProc messages }
	kMenuDrawMsg = 0;
	kMenuSizeMsg = 2;
	kMenuPopUpMsg = 3;    { position the popup menu rect appropriately }
	kMenuCalcItemMsg = 5;
	kMenuThemeSavvyMsg = 7;    { is your MDEF theme-savvy?  If so, return kThemeSavvyMenuResponse in the whichItem parameter}
	kMenuInitMsg = 8;    { Return an error code in *whichItem to indicate success or failure. Only supported in Carbon. }
	kMenuDisposeMsg = 9;    { The menu is being destroyed. Only supported in Carbon.}
	kMenuFindItemMsg = 10;   { Determine which item is under the mouse. Only supported in Carbon.}
	kMenuHiliteItemMsg = 11;   { Hilite the specified item. Only supported in Carbon.}
	kMenuDrawItemsMsg = 12;   { Draw a range of items. Only supported in Carbon.}
	mDrawMsg = kMenuDrawMsg; { obsolete constant name}
	mSizeMsg = kMenuSizeMsg; { obsolete constant name}
	mPopUpMsg = kMenuPopUpMsg; { obsolete constant name}
	mCalcItemMsg = kMenuCalcItemMsg; { obsolete constant name}


const
	kThemeSavvyMenuResponse = $7473; { should be returned in *whichItem when handling kMenuThemeSavvyMsg}


{
 *  MDEFHiliteItemData
 *  
 *  Summary:
 *    The MDEFHiliteItemData structure contains information about which
 *    menu items should be hilited and unhilited as the user moves
 *    through the menus. It is used by menu definition functions, which
 *    receive a pointer to an MDEFHiliteItemData structure as the
 *    whichItem parameter during kMenuHiliteItemMsg.
 }
type
	MDEFHiliteItemData = record
{
   * The item that was previously selected. It should be redrawn in an
   * unhilited state. May be zero if no item was previously selected.
   }
		previousItem: MenuItemIndex;

  {
   * The item that is now selected. It should be redrawn in a hilited
   * state. May be zero if no item is now selected.
   }
		newItem: MenuItemIndex;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFHiliteItemDataPtr = ^MDEFHiliteItemData;
type
	HiliteMenuItemData = MDEFHiliteItemData;
	HiliteMenuItemDataPtr = MDEFHiliteItemDataPtr;

{
 *  MDEFDrawData
 *  
 *  Summary:
 *    The MDEFDrawData structure contains information needed to draw a
 *    menu. It is used by menu definition functions, which receive a
 *    pointer to an MDEFDrawData structure as the whichItem parameter
 *    during kMenuDrawMsg.
 }
type
	MDEFDrawData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the virtualMenuTop and virtualMenuBottom fields of this structure
   * while drawing the menu.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFDrawDataPtr = ^MDEFDrawData;

{
 *  MDEFFindItemData
 *  
 *  Summary:
 *    The MDEFFindItemData structure contains information needed to
 *    determine which item is currently selected by the user. It is
 *    used by menu definition functions, which receive a pointer to an
 *    MDEFDrawData structure as the whichItem parameter during
 *    kMenuFindItemMsg.
 }
type
	MDEFFindItemData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the itemSelected, itemUnderMouse, and itemRect fields of this
   * structure after determining which item is at the specified point.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into if it needs
   * to scroll the menu during the FindItem message. The Menu Manager
   * will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFFindItemDataPtr = ^MDEFFindItemData;

{
 *  MDEFDrawItemsData
 *  
 *  Summary:
 *    The MDEFDrawItemsData structure contains information about which
 *    menu items to redraw. It is used by menu definition functions,
 *    which receive a pointer to an MDEFDrawItemsData structure as the
 *    whichItem parameter during kMenuDrawItemsMsg.
 }
type
	MDEFDrawItemsData = record
{
   * The first item to draw.
   }
		firstItem: MenuItemIndex;

  {
   * The last item to draw.
   }
		lastItem: MenuItemIndex;

  {
   * Information about the menu's tracking state. The virtualMenuTop
   * and virtualMenuBottom fields in this structure will be the most
   * useful in handling the DrawItems message.
   }
		trackingData: MenuTrackingDataPtr;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF returns.
   }
		context: UnivPtr;
	end;
	MDEFDrawItemsDataPtr = ^MDEFDrawItemsData;
{ obsolete menu color table constants}
const
	mctAllItems = -98;  {search for all Items for the given ID}
	mctLastIDIndic = -99;   {last color table entry has this in ID field}

type
	MCEntry = record
		mctID: MenuID;                  {menu ID.  ID = 0 is the menu bar}
		mctItem: SInt16;                {menu Item. Item = 0 is a title}
		mctRGB1: RGBColor;                {usage depends on ID and Item}
		mctRGB2: RGBColor;                {usage depends on ID and Item}
		mctRGB3: RGBColor;                {usage depends on ID and Item}
		mctRGB4: RGBColor;                {usage depends on ID and Item}
		mctReserved: SInt16;            {reserved for internal use}
	end;
	MCEntryPtr = ^MCEntry;
type
	MCTable = array [0..0] of MCEntry;	{ARRAY [1..numEntries] of MCEntry}
	MCTablePtr = ^MCTable;
	MCTableHandle = ^MCTablePtr;
type
	MenuCRsrc = record
		numEntries: SInt16;             {number of entries}
		mcEntryRecs: MCTable;            {ARRAY [1..numEntries] of MCEntry}
	end;
	MenuCRsrcPtr = ^MenuCRsrc;
type
	MenuCRsrcHandle = ^MenuCRsrcPtr;


const
{
   * Menu item command ID to indicate a hierarchical menu; the item
   * icon ID is the hierarchical menu ID. This constant is deprecated.
   * Use SetMenuItemHierarchicalID or SetMenuItemHierarchicalMenu
   * instead of using this constant.
   }
	hMenuCmd = 27;

  {
   * A menu ID used with InsertMenu to insert a menu into the
   * hierarchical portion of the menubar. Rather than inserting menus
   * into the hierarchical portion of the menubar, we recommend that
   * your application use SetMenuItemHierarchicalMenu to directly
   * specify the submenu's MenuRef.
   }
	kInsertHierarchicalMenu = -1;

  {
   * The old name for kInsertHierarchicalMenu. This constant is
   * deprecated.
   }
	hierMenu = -1;

{$ifc not TARGET_CPU_64}
{
 *  NewMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateNewMenu instead of NewMenu.
 *  
 *  Summary:
 *    Creates a new menu.
=======
=======
        TopMenuItem
        AtMenuBottom
        
            TopMenuItem and AtMenuBottom are now set automatically by the Menu Manager using the
            values returned in the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData
            structure passed to kMenuDrawMsg and kMenuFindItemMsg.
            
        mbSaveLoc
    
            This undocumented low-memory global was used by pre-Carbon menu definitions to store
            the bounding rect of the currently selected item and to avoid drawing glitches while
            the menu definition was scrolling the contents of a menu that had submenus. The Menu
            Manager now automatically sets the selected item bounds using the value returned in
            the itemRect field of the MenuTrackingData structure passed to kMenuFindItemMsg. In
            order to correctly support scrolling of menus with submenus, a menu definition should
            verify, before scrolling the menu contents, that no submenus of the scrolling menu are
            currently visible. A menu definition can use GetMenuTrackingData to verify this condition,
            as follows:
            
                Boolean SafeToScroll( MenuRef menuBeingScrolled )
                (
                    MenuTrackingData lastMenuData;
                    return GetMenuTrackingData( NULL, &lastMenuData ) == noErr
                           && lastMenuData.menu == menuBeingScrolled;
                )
            
            If SafeToScroll returns false, the menu definition should not scroll the menu.
}
>>>>>>> origin/fixes_2_2
const
{ menu defProc messages }
	kMenuDrawMsg = 0;
	kMenuSizeMsg = 2;
	kMenuPopUpMsg = 3;    { position the popup menu rect appropriately }
	kMenuCalcItemMsg = 5;
	kMenuThemeSavvyMsg = 7;    { is your MDEF theme-savvy?  If so, return kThemeSavvyMenuResponse in the whichItem parameter}
	kMenuInitMsg = 8;    { Return an error code in *whichItem to indicate success or failure. Only supported in Carbon. }
	kMenuDisposeMsg = 9;    { The menu is being destroyed. Only supported in Carbon.}
	kMenuFindItemMsg = 10;   { Determine which item is under the mouse. Only supported in Carbon.}
	kMenuHiliteItemMsg = 11;   { Hilite the specified item. Only supported in Carbon.}
	kMenuDrawItemsMsg = 12;   { Draw a range of items. Only supported in Carbon.}
	mDrawMsg = kMenuDrawMsg; { obsolete constant name}
	mSizeMsg = kMenuSizeMsg; { obsolete constant name}
	mPopUpMsg = kMenuPopUpMsg; { obsolete constant name}
	mCalcItemMsg = kMenuCalcItemMsg; { obsolete constant name}


const
	kThemeSavvyMenuResponse = $7473; { should be returned in *whichItem when handling kMenuThemeSavvyMsg}

=======
>>>>>>> origin/fixes_2.4

const
{
   * Proc ID for a normal text menu. This constant is not typically
   * used.
   }
	textMenuProc = 0;

  {
   * This value may be passed to InsertMenuItem, InsertMenuItemText,
   * and InsertMenuItemTextWithCFString to indicate that the new item
   * should be inserted at the end of the menu. Note that you can also
   * just call AppendMenu[ItemText][WithCFString].
   }
	kHIMenuAppendItem = $0000FFFF;

const
	noMark = 0;     { mark symbol for SetItemMark; other mark symbols are defined in Fonts.h }

{ Constants for use with MacOS 8.0 (Appearance 1.0) and later}
const
	kMenuStdMenuProc = 63;
	kMenuStdMenuBarProc = 63;

{ For use with Get/SetMenuItemModifiers}
const
	kMenuNoModifiers = 0;    { Mask for no modifiers}
	kMenuShiftModifier = 1 shl 0; { Mask for shift key modifier}
	kMenuOptionModifier = 1 shl 1; { Mask for option key modifier}
	kMenuControlModifier = 1 shl 2; { Mask for control key modifier}
	kMenuNoCommandModifier = 1 shl 3; { Mask for no command key modifier}

{ For use with Get/SetMenuItemIconHandle}
const
	kMenuNoIcon = 0;    { No icon}
	kMenuIconType = 1;    { Type for ICON}
	kMenuShrinkIconType = 2;    { Type for ICON plotted 16 x 16}
	kMenuSmallIconType = 3;    { Type for SICN}
	kMenuColorIconType = 4;    { Type for cicn}
	kMenuIconSuiteType = 5;    { Type for Icon Suite}
	kMenuIconRefType = 6;    { Type for Icon Ref}
	kMenuCGImageRefType = 7;    { Type for a CGImageRef (Mac OS X only)}
	kMenuSystemIconSelectorType = 8;    { Type for an OSType identifying an IconRef registered with Icon Services under kSystemIconsCreator (Mac OS X 10.1 and later only)}
	kMenuIconResourceType = 9;     { Type for a CFStringRef with the full name of a .icns resource in the main bundle of the process (Mac OS X 10.1 and later only)}

{ For use with Get/SetMenuItemKeyGlyph}
const
	kMenuNullGlyph = $00; { Null (always glyph 1)}
	kMenuTabRightGlyph = $02; { Tab to the right key (for left-to-right script systems)}
	kMenuTabLeftGlyph = $03; { Tab to the left key (for right-to-left script systems)}
	kMenuEnterGlyph = $04; { Enter key}
	kMenuShiftGlyph = $05; { Shift key}
	kMenuControlGlyph = $06; { Control key}
	kMenuOptionGlyph = $07; { Option key}
	kMenuSpaceGlyph = $09; { Space (always glyph 3) key}
	kMenuDeleteRightGlyph = $0A; { Delete to the right key (for right-to-left script systems)}
	kMenuReturnGlyph = $0B; { Return key (for left-to-right script systems)}
	kMenuReturnR2LGlyph = $0C; { Return key (for right-to-left script systems)}
	kMenuNonmarkingReturnGlyph = $0D; { Nonmarking return key}
	kMenuPencilGlyph = $0F; { Pencil key}
	kMenuDownwardArrowDashedGlyph = $10; { Downward dashed arrow key}
	kMenuCommandGlyph = $11; { Command key}
	kMenuCheckmarkGlyph = $12; { Checkmark key}
	kMenuDiamondGlyph = $13; { Diamond key}
	kMenuAppleLogoFilledGlyph = $14; { Apple logo key (filled)}
	kMenuParagraphKoreanGlyph = $15; { Unassigned (paragraph in Korean)}
	kMenuDeleteLeftGlyph = $17; { Delete to the left key (for left-to-right script systems)}
	kMenuLeftArrowDashedGlyph = $18; { Leftward dashed arrow key}
	kMenuUpArrowDashedGlyph = $19; { Upward dashed arrow key}
	kMenuRightArrowDashedGlyph = $1A; { Rightward dashed arrow key}
	kMenuEscapeGlyph = $1B; { Escape key}
	kMenuClearGlyph = $1C; { Clear key}
	kMenuLeftDoubleQuotesJapaneseGlyph = $1D; { Unassigned (left double quotes in Japanese)}
	kMenuRightDoubleQuotesJapaneseGlyph = $1E; { Unassigned (right double quotes in Japanese)}
	kMenuTrademarkJapaneseGlyph = $1F; { Unassigned (trademark in Japanese)}
	kMenuBlankGlyph = $61; { Blank key}
	kMenuPageUpGlyph = $62; { Page up key}
	kMenuCapsLockGlyph = $63; { Caps lock key}
	kMenuLeftArrowGlyph = $64; { Left arrow key}
	kMenuRightArrowGlyph = $65; { Right arrow key}
	kMenuNorthwestArrowGlyph = $66; { Northwest arrow key}
	kMenuHelpGlyph = $67; { Help key}
	kMenuUpArrowGlyph = $68; { Up arrow key}
	kMenuSoutheastArrowGlyph = $69; { Southeast arrow key}
	kMenuDownArrowGlyph = $6A; { Down arrow key}
	kMenuPageDownGlyph = $6B; { Page down key}
	kMenuAppleLogoOutlineGlyph = $6C; { Apple logo key (outline)}
	kMenuContextualMenuGlyph = $6D; { Contextual menu key}
	kMenuPowerGlyph = $6E; { Power key}
	kMenuF1Glyph = $6F; { F1 key}
	kMenuF2Glyph = $70; { F2 key}
	kMenuF3Glyph = $71; { F3 key}
	kMenuF4Glyph = $72; { F4 key}
	kMenuF5Glyph = $73; { F5 key}
	kMenuF6Glyph = $74; { F6 key}
	kMenuF7Glyph = $75; { F7 key}
	kMenuF8Glyph = $76; { F8 key}
	kMenuF9Glyph = $77; { F9 key}
	kMenuF10Glyph = $78; { F10 key}
	kMenuF11Glyph = $79; { F11 key}
	kMenuF12Glyph = $7A; { F12 key}
	kMenuF13Glyph = $87; { F13 key}
	kMenuF14Glyph = $88; { F14 key}
	kMenuF15Glyph = $89; { F15 key}
	kMenuControlISOGlyph = $8A; { Control key (ISO standard)}
	kMenuEjectGlyph = $8C; { Eject key (available on Mac OS X 10.2 and later)}
	kMenuEisuGlyph = $8D; { Japanese eisu key (available in Mac OS X 10.4 and later)}
	kMenuKanaGlyph = $8E; { Japanese kana key (available in Mac OS X 10.4 and later)}
	kMenuF16Glyph = $8F; { F16 key (available in SnowLeopard and later)}
	kMenuF17Glyph = $90; { F17 key (available in SnowLeopard and later)}
	kMenuF18Glyph = $91; { F18 key (available in SnowLeopard and later)}
	kMenuF19Glyph = $92;  { F19 key (available in SnowLeopard and later)}


{
 *  MenuAttributes
 *  
 *  Summary:
 *    Menu attributes control behavior of the entire menu. They are
 *    used with the Get/ChangeMenuAttributes APIs.
 }
type
	MenuAttributes = UInt32;
const
{
   * No column space is allocated for the mark character when this menu
   * is drawn.
   }
	kMenuAttrExcludesMarkColumn = 1 shl 0;

  {
   * The menu title is automatically disabled when all items are
   * disabled.
   }
	kMenuAttrAutoDisable = 1 shl 2;

  {
   * The pencil glyph from the Keyboard font (kMenuPencilGlyph) is used
   * to draw the Control modifier key in menu keyboard equivalents.
   * This appearance is typically used only by Japanese input method
   * menus.
   }
	kMenuAttrUsePencilGlyph = 1 shl 3;

  {
   * The menu title is not drawn in the menubar, even when the menu is
   * inserted in the menubar. Useful for adding command keys that don't
   * correspond to a visible menu item; menu items with the desired
   * command keys can be added to the menu and inserted in the menubar
   * without making the menu visible. This attribute is available in
   * Mac OS X 10.2 and later.
   }
	kMenuAttrHidden = 1 shl 4;

  {
   * If menu item separators are present at the beginning or end of the
   * menu, or if multiple contiguous separators are present, the extra
   * separator items are marked as hidden to avoid extra blank space in
   * the menu. The menu is examined for extra separators whenever the
   * menu size is recalculated. This attribute is available in Mac OS X
   * 10.3 and later.
   }
	kMenuAttrCondenseSeparators = 1 shl 5;

  {
   * Disables automatic caching of the menu image by the Menu Manager.
   * Automatic caching is provided for all menus that use an HIView to
   * draw their content. Setting this attribute will prevent the Menu
   * Manager from caching the menu image; instead, the menu will be
   * drawn using the standard HIView drawing mechanism each time that
   * it is displayed. This attribute is available in Mac OS X 10.3 and
   * later.
   }
	kMenuAttrDoNotCacheImage = 1 shl 6;

  {
   * Disables substitution of command keys from the
   * NSUserKeyEquivalents dictionary. By default, all menu items are
   * checked for a match in the dictionary. Note that this attribute,
   * to be effective, should be added at the time that the menu is
   * created; once the menu has been searched for user command keys
   * (which occurs in CalcMenuSize, in GetItemCmd and
   * GetMenuItemCommandKey, and before command key matching), the
   * original command keys are replaced by the user command keys and
   * cannot be retrieved. For this reason, it is also not useful to
   * clear this attribute; the original command keys cannot be
   * restored. This attribute is available in Mac OS X 10.3 and later.
   }
	kMenuAttrDoNotUseUserCommandKeys = 1 shl 7;


{
 *  MenuItemAttributes
 *  
 *  Summary:
 *    Menu item attributes control behavior of individual menu items.
 *    They are used with the Get/ChangeMenuItemAttributes APIs.
 }
type
	MenuItemAttributes = UInt32;
const
{
   * This item is disabled.
   }
	kMenuItemAttrDisabled = 1 shl 0;

  {
   * This item's icon is disabled.
   }
	kMenuItemAttrIconDisabled = 1 shl 1;

  {
   * Allows the parent item of a submenu to be selectable.
   }
	kMenuItemAttrSubmenuParentChoosable = 1 shl 2;

  {
   * This item changes dynamically based on modifier key state.
   }
	kMenuItemAttrDynamic = 1 shl 3;

  {
   * This item is not part of the same dynamic group as the previous
   * item.
   }
	kMenuItemAttrNotPreviousAlternate = 1 shl 4;

  {
   * This item is not drawn when the menu is displayed. It is also not
   * included in command key matching, unless the item also has either
   * the Dynamic or IncludeInCmdKeyMatching attributes.
   }
	kMenuItemAttrHidden = 1 shl 5;

  {
   * This item is a separator; the text of the item is ignored.
   }
	kMenuItemAttrSeparator = 1 shl 6;

  {
   * This item is a menu section header; it is disabled and
   * unselectable.
   }
	kMenuItemAttrSectionHeader = 1 shl 7;

  {
   * Metacharacters in the text of this item (such as the dash) are
   * ignored.
   }
	kMenuItemAttrIgnoreMeta = 1 shl 8;

  {
   * This item is recognized by IsMenuKeyEvent when it is passed an
   * auto-repeat keyboard event.
   }
	kMenuItemAttrAutoRepeat = 1 shl 9;

  {
   * When MenuEvent and IsMenuKeyEvent compare this item's keyboard
   * equivalent against a keyboard event, they use the item's virtual
   * keycode equivalent rather than its character code equivalent.
   }
	kMenuItemAttrUseVirtualKey = 1 shl 10;

  {
   * This item is drawn in a customized fashion by the application.
   * Causes custom menu item drawing Carbon events to be sent. This
   * attribute is available in CarbonLib 1.4 and Mac OS X 10.1, and
   * later.
   }
	kMenuItemAttrCustomDraw = 1 shl 11;

  {
   * This item is examined during command key matching by MenuKey,
   * MenuEvent, and IsMenuKeyEvent. Normally, visible menu items are
   * included in command key matching, but hidden menu items are
   * excluded (unless the item also has the Dynamic menu item
   * attribute). The IncludeInCmdKeyMatching attribute can be used to
   * force a hidden, non-dynamic menu item to be included in command
   * key matching when it normally wouldn't. This attribute is
   * available in CarbonLib 1.6 and Mac OS X 10.2 and later.
   }
	kMenuItemAttrIncludeInCmdKeyMatching = 1 shl 12;

  {
   * This item is automatically disabled if, when
   * kEventCommandUpdateStatus is sent for this item, no handler is
   * installed or all handlers return eventNotHandledErr. A return
   * value from any handler of any value other than eventNotHandledErr
   * will prevent automatic disabling. This attribute is useful for
   * applications that use kEventCommandUpdateStatus events for all
   * menu item enabling; previously, an application would need to
   * install UpdateStatus handlers on its application target to disable
   * all items in the application that were unnecessary when no
   * document windows were open. By setting this attribute, all menu
   * items will be disabled automatically unless specifically enabled
   * by an UpdateStatus handler on a window, control, or application
   * target. This attribute is available in Mac OS X 10.3 and later.
   }
	kMenuItemAttrAutoDisable = 1 shl 13;

  {
   * During command key matching, the Menu Manager uses a cache of the
   * available command keys to locate the menu item that matches an
   * event. Before returning this item, the Menu Manager sends a
   * kEventMenuEnableItems event to the menu containing the item, and a
   * kEventCommandUpdateStatus event to each item in the menu, so that
   * the item can be properly enabled or disabled. For some
   * applications, updating the item status for each item in the menu
   * is quite expensive, and also unnecessary since only a single item
   * actually needs to be updated. Setting this attribute indicates to
   * the Menu Manager that it only needs to send a
   * kEventCommandUpdateStatus event to this menu item before returning
   * it from command key matching; kEventMenuEnableItems will not be
   * sent to the menu, and no other menu item will receive
   * kEventCommandUpdateStatus. This attribute is available in Mac OS X
   * 10.3 and later.
   }
	kMenuItemAttrUpdateSingleItem = 1 shl 14;


{
 *  MenuTrackingMode
 *  
 *  Summary:
 *    A menu tracking mode constant is part of the
 *    kEventMenuBeginTracking and kEventMenuChangeTrackingMode Carbon
 *    events. It indicates whether menus are being tracked using the
 *    mouse or keyboard.
 }
type
	MenuTrackingMode = UInt32;
const
{
   * Menus are being tracked using the mouse.
   }
	kMenuTrackingModeMouse = 1;

  {
   * Menus are being tracked using the keyboard.
   }
	kMenuTrackingModeKeyboard = 2;


{
 *  MenuEventOptions
 *  
 *  Summary:
 *    Menu event options control how the menus are searched for an item
 *    matching a particular keyboard event. They are used with the
 *    IsMenuKeyEvent API.
 }
type
	MenuEventOptions = UInt32;
const
{
   * Disabled items are examined for a match.
   }
	kMenuEventIncludeDisabledItems = $0001;

  {
   * Don't hilite the menu title if a match is found.
   }
	kMenuEventQueryOnly = $0002;

  {
   * Don't look for a match in submenus of the starting menu.
   }
	kMenuEventDontCheckSubmenus = $0004;

{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Types                                                                        }
{——————————————————————————————————————————————————————————————————————————————————————}
type
	MenuID = SInt16;
	MenuItemIndex = UInt16;
	MenuItemIndex_fix = MenuItemIndex; { used as field type when a record declaration contains a MenuItemIndex field identifier }
	MenuItemIndexPtr = ^MenuItemIndex;
	MenuCommand = UInt32;
type
	MenuRef = ^SInt32; { an opaque type }
	MenuRef_fix = MenuRef; { used as field type when a record declaration contains a MenuRef field identifier }
	MenuRefPtr = ^MenuRef;
{ MenuHandle is old name for MenuRef}
type
	MenuHandle = MenuRef;


{
   A MenuBarHandle is a handle to a MenuBarHeader. An instance of this structure is returned
   by the GetMenuBar and GetNewMBar APIs. It is typedef'd to a plain Handle to retain
   source compatibility with previous versions of this header file.
}
type
	MenuBarHandle = Handle;

{
 *  MenuBarHeader
 *  
 *  Summary:
 *    This structure is contained in a MenuBarHandle. It contains a
 *    list of the non-hierarchical menus that have been inserted into
 *    the menubar.
 *  
 *  Discussion:
 *    The MenuBarHandle is a dynamically sized object which cannot be
 *    directly expressed as a C or Pascal structure. First is the
 *    MenuBarHeader structure, followed by a dynamically sized array of
 *    MenuBarMenus, one for each menu. This array is followed by the
 *    HMenuBarHeader, followed by another dynamically sized array of
 *    HMenuBarMenus, one for each hierarchical menu.
 }
type
	MenuBarHeaderPtr = ^MenuBarHeader;
	MenuBarHeader = record
{
   * Offset in bytes from the start of the header to the last menu in
   * the array of MenuBarMenus.
   }
		lastMenu: UInt16;

  {
   * Global coordinate of the right edge of the rightmost menu; unused
   * in a MenuBarHandle returned by GetMenuBar or GetNewMBar.
   }
		lastRight: SInt16;

  {
   * The MBDF resource ID; unused in a MenuBarHandle returned by
   * GetMenuBar or GetNewMBar.
   }
		mbResID: SInt16;
	end;

{
 *  HMenuBarHeader
 *  
 *  Summary:
 *    This structure is contained in a MenuBarHandle. It contains a
 *    list of the hierarchical menus that have been inserted into the
 *    menubar with InsertMenu( menu, -1 ).
 *  
 *  Discussion:
 *    The hierarchical portion of the menubar follows the
 *    non-hierarchical portion in a menubar handle. The hierarchical
 *    portion begins with the HMenuBarHeader structure, followed by a
 *    dynamically sized array of HMenuBarMenus.
 }
type
	HMenuBarHeaderPtr = ^HMenuBarHeader;
	HMenuBarHeader = record
{
   * Offset in bytes from the start of the header to the last menu in
   * the array of HMenuBarMenus.
   }
		lastHMenu: UInt16;

  {
   * Saved bits behind the hilited menu title; unused in a
   * MenuBarHandle returned by GetMenuBar or GetNewMBar.
   }
		menuTitleBits: PixMapHandle;
	end;

{
 *  MenuBarMenu
 *  
 *  Summary:
 *    This structure contains a single menu in the menubar. It is an
 *    element in an array in the MenuBarHeader data strucuture.
 }
type
	MenuBarMenuPtr = ^MenuBarMenu;
	MenuBarMenu = record
{
   * A menu in the menubar.
   }
		menu: MenuRef;

  {
   * The global coordinate of the left edge of the menu title; unused
   * in a MenuBarHandle returned by GetMenuBar or GetNewMBar.
   }
		menuLeft: SInt16;
	end;

{
 *  HMenuBarMenu
 *  
 *  Summary:
 *    This structure contains a single hierarchical menu in the
 *    menubar. It is an element in an array in the HMenuBarHeader data
 *    strucuture.
 }
type
	HMenuBarMenuPtr = ^HMenuBarMenu;
	HMenuBarMenu = record
{
   * An hierarchical menu in the menubar.
   }
		menu: MenuRef;

  {
   * This field is currently unused.
   }
		reserved: SInt16;
	end;
{
 *  MenuTrackingData
 *  
 *  Summary:
 *    The MenuTrackingData structure contains information about a menu
 *    currently being displayed. It is used with the
 *    GetMenuTrackingData API.
 }
type
	MenuTrackingData = record
		menu: MenuRef;
		itemSelected: MenuItemIndex;
		itemUnderMouse: MenuItemIndex;
		itemRect: Rect;
		virtualMenuTop: SInt32;
		virtualMenuBottom: SInt32;
	end;
	MenuTrackingDataPtr = ^MenuTrackingData;

{
 *  Summary:
 *    A MenuItemDataFlags value indicates which fields of a
 *    MenuItemDataRec structure should be used by the
 *    Copy/SetMenuItemData APIs. All MenuItemDataFlags may be used when
 *    getting or setting the contents of a menu item; some may also be
 *    used when getting or setting information about the menu itself,
 *    if the item index given to Copy/SetMenuItemData is 0.
 }
const
{
   * Set or return the Str255 text of a menu using the
   * MenuItemDataRec.text field. If getting the text, the text field
   * must be initialized with a pointer to a Str255 variable before
   * calling CopyMenuItemData. If both kMenuItemDataText and
   * kMenuItemCFString are set on entry to CopyMenuItemData, the API
   * will determine whether the menu text was most recently set using a
   * Str255 or CFString, and return only that text format; the flags
   * value for the other format will be cleared. Valid for both menu
   * items and the menu title (if item number is 0). 
   * 
   * Note that this option is not supported in the 64-bit HIToolbox API.
   }
	kMenuItemDataText = 1 shl 0;

  {
   * Set or return the mark character of a menu item using the
   * MenuItemDataRec.mark field. Valid only for menu items.
   }
	kMenuItemDataMark = 1 shl 1;

  {
   * Set or return the command key of a menu item using the
   * MenuItemDataRec.cmdKey field. Valid only for menu items.
   }
	kMenuItemDataCmdKey = 1 shl 2;

  {
   * Set or return the command key glyph of a menu item using the
   * MenuItemDataRec.cmdKeyGlyph field. Valid only for menu items.
   }
	kMenuItemDataCmdKeyGlyph = 1 shl 3;

  {
   * Set or return the command key modifiers of a menu item using the
   * MenuItemDataRec.cmdKeyModifiers field. Valid only for menu items.
   }
	kMenuItemDataCmdKeyModifiers = 1 shl 4;

  {
   * Set or return the QuickDraw text style of a menu item using the
   * MenuItemDataRec.style field. Valid only for menu items.
   }
	kMenuItemDataStyle = 1 shl 5;

  {
   * Set or return the enable state of a menu using the
   * MenuItemDataRec.enabled field. Valid for both menu items and the
   * menu itself (if item number is 0).
   }
	kMenuItemDataEnabled = 1 shl 6;

  {
   * Set or return the enable state of a menu item icon using the
   * MenuItemDataRec.iconEnabled field. Valid only for menu items.
   }
	kMenuItemDataIconEnabled = 1 shl 7;

  {
   * Set or return the icon resource ID of a menu item using the
   * MenuItemDataRec.iconID field. Valid only for menu items. 
   * 
   * Note that this option is not supported in the 64-bit HIToolbox API.
   }
	kMenuItemDataIconID = 1 shl 8;

  {
   * Set or return the icon handle of a menu item using the
   * MenuItemDataRec.iconType and MenuItemDataRec.iconHandle fields.
   * Both fields must be initialized if setting the icon handle; both
   * fields will be returned when getting the handle. The iconType
   * field should contain one of the constants kMenuIconType,
   * kMenuShrinkIconType, kMenuSmallIconType, kMenuColorIconType,
   * kMenuIconSuiteType, kMenuIconRefType, kMenuCGImageRefType,
   * kMenuSystemIconSelectorType, or kMenuIconResourceType. An icon
   * handle may be a handle to an ICON resource, a SICN resource, a
   * cicn resource, an IconSuite, an IconRef, a CGImageRef, an OSType,
   * or a CFStringRef. Valid only for menu items. When used with
   * CopyMenuItemData, the caller does not need to release the returned
   * icon handle.
   }
	kMenuItemDataIconHandle = 1 shl 9;

  {
   * Set or return the command ID of a menu item using the
   * MenuItemDataRec.cmdID field. Valid only for menu items.
   }
	kMenuItemDataCommandID = 1 shl 10;

  {
   * Set or return the text encoding of a menu item using the
   * MenuItemDataRec.encoding field. Valid only for menu items.
   * 
   * 
   * Note that this option is not supported in the 64-bit HIToolbox API.
   }
	kMenuItemDataTextEncoding = 1 shl 11;

  {
   * Set or return the menu ID of a item's submenu using the
   * MenuItemDataRec.submenuID field. Valid only for menu items.
   }
	kMenuItemDataSubmenuID = 1 shl 12;

  {
   * Set or return the menu handle of an item's submenu using the
   * MenuItemDataRec.submenuHandle field. Valid only for menu items.
   * When used with CopyMenuItemData, the caller does not need to
   * release the returned MenuRef.
   }
	kMenuItemDataSubmenuHandle = 1 shl 13;

  {
   * Set or return the QuickDraw font ID of a menu item using the
   * MenuItemDataRec.fontID field. Valid only for menu items.
   }
	kMenuItemDataFontID = 1 shl 14;

  {
   * Set or return the refcon of a menu using the
   * MenuItemDataRec.refcon field. Valid for both menu items and the
   * menu itself (if item number is 0).
   }
	kMenuItemDataRefcon = 1 shl 15;

  {
   * Set or return the attributes of a menu using the
   * MenuItemDataRec.attr field. Valid for both menu items, in which
   * case the attr field should contain values from the
   * MenuItemAttributes enumeration, and for the menu itself (if item
   * number is 0), in which case the attr field should contain values
   * from the MenuAttributes enumeration.
   }
	kMenuItemDataAttributes = 1 shl 16;

  {
   * Set or return the CFString text of a menu using the
   * MenuItemDataRec.cfText field. If setting the CFString text, the
   * Menu Manager increments the refcount of the text; the caller may
   * safely release the text after setting it. If getting the CFString
   * text, the caller receives a copy of the text owned by the Menu
   * Manager; modifications on the text by the caller will have no
   * effect on the text of the menu, and the caller should release the
   * text when finished with it. If both kMenuItemDataText and
   * kMenuItemCFString are set on entry to CopyMenuItemData, the API
   * will determine whether the menu text was most recently set using a
   * Str255 or CFString, and return only that text format; the flags
   * value for the other format will be cleared. Valid for both menu
   * items and the menu title (if item number is 0). When used with
   * CopyMenuItemData, the caller must release the returned CFStringRef.
   }
	kMenuItemDataCFString = 1 shl 17;

  {
   * Set or return the properties of a menu using the
   * MenuItemDataRec.properties field. If setting the properties, the
   * properties field should contain a collection with the new
   * properties; existing menu properties with the same collection
   * creator and tag will be replaced by the new properties. If getting
   * the properties, the properties field should either be set to NULL
   * or to a valid Collection. If NULL, a new collection is allocated
   * by the CopyMenuItemData and returned in the properties field. If
   * not NULL, the entire contents of the collection are replaced by
   * the properties of the menu. Valid for both menu items and the menu
   * itself (if item number is 0).
   }
	kMenuItemDataProperties = 1 shl 18;

  {
   * Set or return the item indent level of a menu item using the
   * MenuItemDataRec.indent field. Valid only for menu items.
   }
	kMenuItemDataIndent = 1 shl 19;

  {
   * Set or return the virtual key code keyboard equivalent of a menu
   * item using the MenuItemDataRec.cmdVirtualKey field. Valid only for
   * menu items. On output, only valid if the item has the
   * kMenuItemAttrUseVirtualKeyCode attribute. Available in Mac OS X
   * and CarbonLib 1.1 and later.
   }
	kMenuItemDataCmdVirtualKey = 1 shl 20;

  {
   * Set or return the attributed text of a menu item using the
   * MenuItemDataRec.attributedText field. If setting the attributed
   * text, the Menu Manager increments the refcount of the text; the
   * caller may safely release the text after setting it. If getting
   * the attributed text, the caller receives a copy of the text owned
   * by the Menu Manager; modifications on the text by the caller will
   * have no effect on the text of the menu, and the caller should
   * release the text when finished with it. Valid only for menu items.
   * Available in Mac OS X 10.5 and later.
   }
	kMenuItemDataAttributedText = 1 shl 21;

  {
   * Set or return the CTFontRef of a menu using the
   * MenuItemDataRec.font field. If setting the font, the Menu Manager
   * increments the refcount of the font; the caller may safely release
   * the font after setting it. If getting the font, the caller
   * receives a retained instance of the font owned by the Menu
   * Manager; the caller should release the font when finished with it.
   * Valid for both menu items and the menu title (if item number is
   * 0). Available in Mac OS X 10.5 and later.
   }
	kMenuItemDataFont = 1 shl 22;
	kMenuItemDataAllDataVersionOne = $000FFFFF;
	kMenuItemDataAllDataVersionTwo = kMenuItemDataAllDataVersionOne or kMenuItemDataCmdVirtualKey;
	kMenuItemDataAllDataVersionThree = kMenuItemDataAllDataVersionTwo or kMenuItemDataAttributedText or kMenuItemDataFont;

type
	MenuItemDataFlags = UInt64;

{
 *  MenuItemDataRec
 *  
 *  Summary:
 *    The MenuItemDataRec structure is used to get and change aspects
 *    of a menu item. It is used with the Copy/SetMenuItemData APIs.
 *  
 *  Discussion:
 *    When using this structure with Copy/SetMenuItemData, the caller
 *    must first set the whichData field to a combination of
 *    MenuItemDataFlags indicating which specific data should be
 *    retrieved or set. Some fields also require initialization before
 *    calling CopyMenuItemData; see the individual MenuItemDataFlags
 *    documentation for details. When used with CopyMenuItemData, the
 *    whichData field will be updated on exit to indicate which data
 *    fields have actually been returned. If the caller requests data
 *    that does not exist for that menu or menu item, the corresponding
 *    bit in the whichData field will be cleared, and the field in the
 *    MenuItemDataRec for that data will be set to zero.
 }
type
	MenuItemDataRecPtr = ^MenuItemDataRec;
	MenuItemDataRec = record
		whichData: MenuItemDataFlags;
		text: StringPtr;
		mark: UniChar;
		cmdKey: UniChar;
		cmdKeyGlyph: UInt32;
		cmdKeyModifiers: UInt32;
		style: Style_fix;
		enabled: Boolean;
		iconEnabled: Boolean;
		filler1: UInt8;
		iconID: SInt32;
		iconType: UInt32;
		iconHandle: Handle;
		cmdID: MenuCommand;
		encoding: TextEncoding;
		submenuID: MenuID;
		submenuHandle: MenuRef;
		fontID: SInt32;
		refcon: URefCon;
		attr: OptionBits;
		cfText: CFStringRef;
		properties: Collection;
		indent: UInt32;
		cmdVirtualKey: UInt16;
		attributedText: CFAttributedStringRef;
		font: CTFontRef;
	end;
type
	MenuItemDataPtr = MenuItemDataRecPtr;
	MenuItemID = UInt32;

{
 *  MenuDefType
 *  
 *  Summary:
 *    Types of custom menu definitions.
 }
const
{
   * A custom menu definition using a function pointer based on the
   * pre-Carbon MDEF model. This type of custom menu definition is not
   * supported by 64-bit HIToolbox.
   }
	kMenuDefProcPtr = 0;

  {
   * A custom menu definition using an HIView subclass. Available in
   * Mac OS X 10.3 and later.
   }
	kMenuDefClassID = 1;

type
	MenuDefType = UInt32;
{$ifc TARGET_CPU_64}
type
	MenuDefUPP = UnivPtr;
{$elsec} {TARGET_CPU_64}

{
 *  MenuDefProcPtr
 *  
 *  Summary:
 *    Callback function for a custom menu definition.
 }
type
	MenuDefProcPtr = procedure( message: SInt16; theMenu: MenuRef; var menuRect: Rect; hitPt: Point; var whichItem: SInt16 );
	MenuDefUPP = MenuDefProcPtr;
{
 *  NewMenuDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewMenuDefUPP( userRoutine: MenuDefProcPtr ): MenuDefUPP; external name '_NewMenuDefUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeMenuDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeMenuDefUPP( userUPP: MenuDefUPP ); external name '_DisposeMenuDefUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  InvokeMenuDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeMenuDefUPP( message: SInt16; theMenu: MenuRef; var menuRect: Rect; hitPt: Point; var whichItem: SInt16; userUPP: MenuDefUPP ); external name '_InvokeMenuDefUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{$endc} {TARGET_CPU_64}


{
 *  MenuDefSpec
 *  
 *  Summary:
 *    Specifies a custom menu definition.
 }
type
	MenuDefSpec = record
{
   * The type of menu definition: either kMenuDefProcPtr or
   * kMenuDefClassID. kMenuDefClassID may only be used in Mac OS X 10.3
   * and later. On 64-bit HIToolbox, kMenuDefProcPtr is not supported.
   }
		defType: MenuDefType;
		case SInt16 of
		0: (
			defProc: MenuDefUPP;
			);
		1: (
			classID: CFStringRef;
			initEvent: EventRef;
			);
	end;
	MenuDefSpecPtr = ^MenuDefSpec;
{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Manipulation                                                                 }
{——————————————————————————————————————————————————————————————————————————————————————}
{$ifc not TARGET_CPU_64}
{
 *  [Mac]GetMenu()
 *  
 *  Summary:
 *    Creates a menu using the contents of a 'MENU' resource.
 *  
 *  Discussion:
 *    Apple recommends that you use Interface Builder to design your
 *    menus and other user interface components. Interface Builder
 *    stores its output in nib files rather then Resource Manager
 *    resources. Use the CreateMenuFromNib API to load a menu from a
 *    nib file.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    resourceID:
 *      The resource ID of the 'MENU' resource.
 *  
 *  Result:
 *    The newly created MenuRef. In CarbonLib and Mac OS X, a new menu
 *    will be created by each call to the GetMenu API; this is a change
 *    from the classic Mac OS API, in which GetMenu would return an
 *    existing menu if the menu had previously been created from the
 *    same resource.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
{$ifc TARGET_OS_MAC}
function MacGetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
{$endc} {TARGET_OS_MAC}
function GetMenu( resourceID: SInt16 ): MenuRef; external name '_GetMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DisposeMenu( theMenu: MenuRef ); external name '_DisposeMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuID( menu: MenuRef ): MenuID; external name '_GetMenuID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuWidth( menu: MenuRef ): SInt16; external name '_GetMenuWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuHeight( menu: MenuRef ): SInt16; external name '_GetMenuHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuDefinition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuDefinition( menu: MenuRef; outDefSpec: MenuDefSpecPtr ): OSStatus; external name '_GetMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuID( menu: MenuRef; menuID_: MenuID ); external name '_SetMenuID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuWidth()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuWidth( menu: MenuRef; width: SInt16 ); external name '_SetMenuWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
procedure SetMenuHeight( menu: MenuRef; height: SInt16 ); external name '_SetMenuHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuDefinition()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuDefinition( menu: MenuRef; const (*var*) defSpec: MenuDefSpec ): OSStatus; external name '_SetMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CalcMenuSize()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure CalcMenuSize( theMenu: MenuRef ); external name '_CalcMenuSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CountMenuItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
function CountMenuItems( theMenu: MenuRef ): UInt16; external name '_CountMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Routines available in Mac OS 8.5 and later, and on Mac OS 8.1 and later using CarbonLib 1.1 and later}

{
 *  GetMenuFont()
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID and size used to draw the items
 *    in a menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu's default font, or zero if the font
 *      has not been customized.
 *    
 *    outFontSize:
 *      On exit, contains the menu's default font size, or zero if the
 *      font size has not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuFont( menu: MenuRef; var outFontID: SInt16; var outFontSize: UInt16 ): OSStatus; external name '_GetMenuFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuFont()
 *  
 *  Summary:
 *    Sets the QuickDraw font ID and size used to draw the items in a
 *    menu.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuFont.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu whose font to set.
 *    
 *    inFontID:
 *      The menu's default font, or zero to use the standard font for
 *      menu items.
 *    
 *    inFontSize:
 *      The menu's default font size, or zero to use the standard font
 *      size for menu items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function SetMenuFont( menu: MenuRef; inFontID: SInt16; inFontSize: UInt16 ): OSStatus; external name '_SetMenuFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuExcludesMarkColumn()
 *  
 *  Summary:
 *    Returns whether the layout of a menu includes or excludes space
 *    for menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the GetMenuAttributes API and check
 *    for the kMenuAttrExcludesMarkColumn attribute instead of using
 *    this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to examine.
 *  
 *  Result:
 *    True if the menu layout does not include space for menu item
 *    marks, or false if it does include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuExcludesMarkColumn( menu: MenuRef ): Boolean; external name '_GetMenuExcludesMarkColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuExcludesMarkColumn()
 *  
 *  Summary:
 *    Sets whether the layout of a menu includes or excludes space for
 *    menu item marks.
 *  
 *  Discussion:
 *    Apple recommends that you use the ChangeMenuAttributes API with
 *    the kMenuAttrExcludesMarkColumn attribute instead of using this
 *    API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to modify.
 *    
 *    excludesMark:
 *      True if the menu layout should not include space for menu item
 *      marks, or false if it should include space for marks.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function SetMenuExcludesMarkColumn( menu: MenuRef; excludesMark: Boolean ): OSStatus; external name '_SetMenuExcludesMarkColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RegisterMenuDefinition()
 *  
 *  Summary:
 *    Registers or unregisters a binding between a resource ID and a
 *    menu definition function.
 *  
 *  Discussion:
 *    In the classic Mac OS Menu Manager, a 'MENU' resource can contain
 *    an embedded MDEF procID that is used by the Menu Manager as the
 *    resource ID of an 'MDEF' resource to measure and draw the menu.
 *    The 'MDEF' resource is loaded by the Menu Manager when you load
 *    the menu with GetMenu. Since MDEFs can no longer be packaged as
 *    code resources on Carbon, the procID can no longer refer directly
 *    to an MDEF resource. However, using RegisterMenuDefinition you
 *    can instead specify a UniversalProcPtr pointing to code in your
 *    application code fragment.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inResID:
 *      An MDEF proc ID, as used in a 'MENU' resource.
 *    
 *    inDefSpec:
 *      Specifies the MenuDefUPP that should be used for menus with the
 *      given MDEF proc ID. Passing NULL allows you to unregister the
 *      menu definition that had been associated with the given MDEF
 *      proc ID.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function RegisterMenuDefinition( inResID: SInt16; inDefSpec: MenuDefSpecPtr ): OSStatus; external name '_RegisterMenuDefinition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CreateNewMenu()
 *  
 *  Summary:
 *    Creates a new, untitled, empty menu.
 *  
 *  Discussion:
 *    CreateNewMenu is preferred over NewMenu because it allows you to
 *    specify the menu's attributes and it does not require you to
 *    specify a Str255-based menu title. To set the menu title, you can
 *    use either SetMenuTitle or SetMenuTitleWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenuID:
 *      The menu ID to use for the new menu. Zero is a valid menu ID in
 *      Carbon.
 *    
 *    inMenuAttributes:
 *      The menu attributes to use for the new menu.
 *    
 *    outMenuRef:
 *      On exit, contains the new menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateNewMenu( inMenuID: MenuID; inMenuAttributes: MenuAttributes; var outMenuRef: MenuRef ): OSStatus; external name '_CreateNewMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CreateCustomMenu()
 *  
 *  Summary:
 *    Creates a new, untitled, empty menu using a custom menu
 *    definition function.
 *  
 *  Discussion:
 *    Similar to CreateNewMenu, but also allows you to specify a custom
 *    menu definition function.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDefSpec:
 *      Specifies a custom menu definition function. defSpec->defType
 *      must be kMenuDefProcPtr or, on Mac OS X 10.3 and later,
 *      kMenuDefClassID.
 *    
 *    inMenuID:
 *      The menu ID to use for the new menu.
 *    
 *    inMenuAttributes:
 *      The menu attributes to use for the new menu.
 *    
 *    outMenuRef:
 *      On exit, contains the new menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateCustomMenu( const (*var*) inDefSpec: MenuDefSpec; inMenuID: MenuID; inMenuAttributes: MenuAttributes; var outMenuRef: MenuRef ): OSStatus; external name '_CreateCustomMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsValidMenu()
 *  
 *  Summary:
 *    Determines if a menu is valid.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to check for validity.
 *  
 *  Result:
 *    Indicates whether the menu is valid.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsValidMenu( inMenu: MenuRef ): Boolean; external name '_IsValidMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DuplicateMenu()
 *  
 *  Summary:
 *    Creates a new menu that is a copy of another menu.
 *  
 *  Discussion:
 *    Unlike RetainMenu, DuplicateMenu creates an entirely new menu
 *    that is an exact copy of the original menu. The MDEF for the new
 *    menu will receive an init message after the menu has been fully
 *    created.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSourceMenu:
 *      The menu to duplicate.
 *    
 *    outMenu:
 *      On exit, a copy of the source menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function DuplicateMenu( inSourceMenu: MenuRef; var outMenu: MenuRef ): OSStatus; external name '_DuplicateMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuTitleAsCFString()
 *  
 *  Summary:
 *    Returns a CFString containing the title of a menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose title to return.
 *    
 *    outString:
 *      On exit, a CFString containing the menu's title. This string
 *      must be released by the caller.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuTitleAsCFString( inMenu: MenuRef; var outString: CFStringRef ): OSStatus; external name '_CopyMenuTitleAsCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuTitleWithCFString()
 *  
 *  Summary:
 *    Sets the title of a menu to the text contained in a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    SetMenuTitleWithCFString, depending on whether the string is
 *    mutable or immutable. If the string is mutable, modifying the
 *    string after calling SetMenuTitleWithCFString will have no effect
 *    on the menu's actual title. The caller may release the string
 *    after calling SetMenuTitleWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose title to set.
 *    
 *    inString:
 *      The string containing the new menu title text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuTitleWithCFString( inMenu: MenuRef; inString: CFStringRef ): OSStatus; external name '_SetMenuTitleWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuTitleIcon()
 *  
 *  Summary:
 *    Sets the title of a menu to be an icon.
 *  
 *  Discussion:
 *    The Menu Manager takes ownership of the supplied icon after this
 *    call. When a menu with an title icon is disposed, the Menu
 *    Manager will dispose the icon also; the Menu Manager will also
 *    dispose of the current title icon when a new text or icon title
 *    is supplied for a menu. If an IconRef or CGImageRef is specified,
 *    the Menu Manager will increment its refcount, so you may freely
 *    release your reference to the icon or image without invalidating
 *    the Menu Manager's copy. The menubar will be invalidated by this
 *    call, and redrawn at the next opportunity.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose title to set.
 *    
 *    inType:
 *      The type of icon being used to specify the icon title; use
 *      kMenuNoIcon to remove the icon from the menu title. In Mac OS X
 *      10.2 and earlier, the supported types are kMenuIconSuiteType
 *      and kMenuIconRefType; Mac OS X 10.3 also supports
 *      kMenuCGImageRefType.
 *    
 *    inIcon:
 *      The icon; must be NULL if inType is kMenuNoIcon. The supported
 *      icon formats are IconSuiteRef, IconRef, and in Mac OS X 10.3
 *      and later, CGImageRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuTitleIcon( inMenu: MenuRef; inType: UInt32; inIcon: UnivPtr ): OSStatus; external name '_SetMenuTitleIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuTitleIcon()
 *  
 *  Summary:
 *    Retrieves the icon, if any, being used as the title of a menu.
 *  
 *  Discussion:
 *    This API does not increment a refcount on the returned icon. The
 *    caller should not release the icon.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose icon title to retrieve.
 *    
 *    outType:
 *      On exit, contains the type of icon being used as the title of
 *      the menu. Contains kMenuNoIcon if the menu does not have an
 *      icon title.
 *    
 *    outIcon:
 *      On exit, contains the IconSuiteRef, IconRef, or CGImageRef
 *      being used as the title of the menu, or NULL if the menu does
 *      not have an icon title. May be NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuTitleIcon( inMenu: MenuRef; outType: UInt32Ptr { can be NULL }; outIcon: UnivPtrPtr { can be NULL } ): OSStatus; external name '_GetMenuTitleIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalidateMenuSize()
 *  
 *  Summary:
 *    Invalidates the menu size so that it will be recalculated when
 *    next displayed.
 *  
 *  Discussion:
 *    The pre-Carbon technique for invalidating the menu size was to
 *    set the width and height to -1. Although this technique still
 *    works, for best compatibility it's preferable to use the
 *    InvalidateMenuSize API so that the Menu Manager has explicit
 *    notification that the menu is invalid.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose size to invalidate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvalidateMenuSize( inMenu: MenuRef ): OSStatus; external name '_InvalidateMenuSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuSizeInvalid()
 *  
 *  Summary:
 *    Determines if a menu's size is invalid and should be recalculated.
 *  
 *  Discussion:
 *    The pre-Carbon technique for determining if a menu's size is
 *    invalid was to check if the width or height was -1. This
 *    technique is not always reliable on Carbon due to implementation
 *    changes in the Menu Manager. You should now use IsMenuSizeInvalid
 *    instead.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose size to examine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuSizeInvalid( inMenu: MenuRef ): Boolean; external name '_IsMenuSizeInvalid';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HIMenuGetContentView()
 *  
 *  Summary:
 *    Returns the HIViewRef that will be used to draw menu content for
 *    this menu, if any.
 *  
 *  Discussion:
 *    If the content view has not yet been created, the Menu Manager
 *    will create the content view using the view class ID and
 *    initialization event associated with the menu. Note that the menu
 *    content view is not the same as the window content view; the menu
 *    content view is embedded inside the window content view. If the
 *    menu uses an MDEF instead of an HIView to draw its content, noErr
 *    is returned but the output HIViewRef is set to NULL.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu.
 *    
 *    inMenuType:
 *      The type of menu for which the menu content view should be
 *      returned. The same MenuRef may have multiple content views,
 *      depending on the menu type being displayed.
 *    
 *    outView:
 *      On exit, contains the view. May be set to NULL if the menu does
 *      not use an HIView to draw its content. The caller should not
 *      release this view.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HIMenuGetContentView( inMenu: MenuRef; inMenuType: ThemeMenuType; var outView: HIViewRef ): OSStatus; external name '_HIMenuGetContentView';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HIMenuSetFont()
 *  
 *  Summary:
 *    Sets the font to be used to draw a menu or a menu item.
 *  
 *  Discussion:
 *    Pass zero for the inItem parameter to set the default font for
 *    all of the items in a menu. Pass a non-zero value for the inItem
 *    parameter to set a font for a specific item; this overrides the
 *    default font for the menu. If a font is set for an entire menu,
 *    it will also apply to all submenus of the menu. Setting a font
 *    for a specific item does not cause that font to apply to the
 *    item's submenu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose font to set.
 *    
 *    inItem:
 *      The item whose font to set, or zero to customze the font for
 *      the entire menu.
 *    
 *    inFont:
 *      The font to use for the specified menu or item. May be NULL to
 *      remove the menu or item's current custom font. The font will be
 *      retained by the Menu Manager and released when the menu or item
 *      is destroyed, or when the font for the menu or item is changed
 *      to a different value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetFont( inMenu: MenuRef; inItem: MenuItemIndex; inFont: CTFontRef { can be NULL } ): OSStatus; external name '_HIMenuSetFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyFont()
 *  
 *  Summary:
 *    Retrieves the custom font that is used to draw a menu or a menu
 *    item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose font to retrieve.
 *    
 *    inItem:
 *      The item whose font to retrieve, or zero to retrieve the font
 *      for the entire menu.
 *    
 *    outFont:
 *      On exit, contains the font used for the specified menu or item.
 *      If the menu or item does not have a custom font, the resulting
 *      CTFontRef is set to NULL. The caller must release the
 *      CTFontRef, if it is non-NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyFont( inMenu: MenuRef; inItem: MenuItemIndex; var outFont: CTFontRef ): OSStatus; external name '_HIMenuCopyFont';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Insertion                                                               }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  DeleteMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DeleteMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_DeleteMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuItems()
 *  
 *  Summary:
 *    Copies menu items from one menu to another.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSourceMenu:
 *      The menu from which to copy items.
 *    
 *    inFirstItem:
 *      The first item to copy.
 *    
 *    inNumItems:
 *      The number of items to copy.
 *    
 *    inDestMenu:
 *      The menu to which to copy items.
 *    
 *    inInsertAfter:
 *      The menu item in the destination menu after which to insert the
 *      copied items. Pass 0 to insert the items at the beginning of
 *      the menu. This value of this parameter must not exceed the
 *      number of items in the destination menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuItems( inSourceMenu: MenuRef; inFirstItem: MenuItemIndex; inNumItems: ItemCount; inDestMenu: MenuRef; inInsertAfter: MenuItemIndex ): OSStatus; external name '_CopyMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DeleteMenuItems()
 *  
 *  Summary:
 *    Deletes multiple menu items.
 *  
 *  Discussion:
 *    This API is more efficient than calling DeleteMenuItem multiple
 *    times.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu from which to delete items.
 *    
 *    inFirstItem:
 *      The first item to delete.
 *    
 *    inNumItems:
 *      The number of items to delete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function DeleteMenuItems( inMenu: MenuRef; inFirstItem: MenuItemIndex; inNumItems: ItemCount ): OSStatus; external name '_DeleteMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AppendMenuItemTextWithCFString()
 *  
 *  Summary:
 *    Appends a new menu item with text from a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    AppendMenuItemWithTextCFString, depending on whether the string
 *    is mutable or immutable. If the string is mutable, modifying the
 *    string after calling AppendMenuItemTextWithCFString will have no
 *    effect on the menu item's actual text. The caller may release the
 *    string after calling AppendMenuItemTextWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to which to append the new item.
 *    
 *    inString:
 *      The text of the new item.
 *    
 *    inAttributes:
 *      The attributes of the new item.
 *    
 *    inCommandID:
 *      The command ID of the new item.
 *    
 *    outNewItem:
 *      On exit, the index of the new item. May be NULL if the caller
 *      does not need this information.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AppendMenuItemTextWithCFString( inMenu: MenuRef; inString: CFStringRef; inAttributes: MenuItemAttributes; inCommandID: MenuCommand; outNewItem: MenuItemIndexPtr { can be NULL } ): OSStatus; external name '_AppendMenuItemTextWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InsertMenuItemTextWithCFString()
 *  
 *  Summary:
 *    Inserts a new menu item with text from a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    InsertMenuItemWithCFString, depending on whether the string is
 *    mutable or immutable. If the string is mutable, modifying the
 *    string after calling InsertMenuItemWithCFString will have no
 *    effect on the menu item's actual text. The caller may release the
 *    string after calling InsertMenuItemWithCFString.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to insert the new item.
 *    
 *    inString:
 *      The text of the new item.
 *    
 *    inAfterItem:
 *      The item after which to insert the new item. Pass 0 to insert
 *      the item at the beginning of the menu. If inAfterItem is
 *      greater than the number of items in the menu, the item is
 *      inserted at the end of the menu.
 *    
 *    inAttributes:
 *      The attributes of the new item.
 *    
 *    inCommandID:
 *      The command ID of the new item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InsertMenuItemTextWithCFString( inMenu: MenuRef; inString: CFStringRef; inAfterItem: MenuItemIndex; inAttributes: MenuItemAttributes; inCommandID: MenuCommand ): OSStatus; external name '_InsertMenuItemTextWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Events                                                                       }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  HIMenuSetSelection()
 *  
 *  Summary:
 *    Sets the selected item in a menu.
 *  
 *  Discussion:
 *    The HIMenuSetSelection API may be called either before opening a
 *    menu, or while a menu is open. If called before opening a menu,
 *    when the menu is next opened, the specified item will be
 *    selected; this case only works for menus that use an HIView to
 *    display menu content, and does not work for MDEF-based menus. If
 *    called while a menu is open, the menu's selection will be changed
 *    to the specified item. Note that changing the selection of an
 *    open menu is not standard UI and may be confusing to the user;
 *    also, changing the selected item does not currently affect the
 *    state of hierarchical submenus of the previously selected or
 *    newly selected item, so Apple recommends that you do not change
 *    the selected state of an item with a hierarchical submenu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose selected item to set.
 *    
 *    inMenuType:
 *      The type of menu for which to set the selected item. Ignored
 *      unless the menu is already open; use GetMenuType to determine
 *      the menu's current type.
 *    
 *    inItem:
 *      The menu item that should be selected.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetSelection( inMenu: MenuRef; inMenuType: ThemeMenuType; inItem: MenuItemIndex ): OSStatus; external name '_HIMenuSetSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  MenuSelect()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MenuSelect( startPt: Point ): SInt32; external name '_MenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PopUpMenuSelect()
 *  
 *  Summary:
 *    Displays a pop-up menu at a specified location.
 *  
 *  Discussion:
 *    In Mac OS 9 and earlier, PopUpMenuSelect requires that the menu
 *    be inserted into the menubar using InsertMenu( menuRef,
 *    kInsertHierarchicalMenu ). CarbonLib 1.1 and later, and Mac OS X,
 *    do not have this requirement; a menu can be displayed by
 *    PopUpMenuSelect even if it is not inserted in the menubar.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menu:
 *      The menu to display.
 *    
 *    top:
 *      The vertical position, in global coordinates, of the top left
 *      corner of the selected item when the menu is opened.
 *    
 *    left:
 *      The horizontal position, in global coordinates, of the top left
 *      corner of the selected item when the menu is opened.
 *    
 *    popUpItem:
 *      The item that should be positioned at the global point
 *      specified by the top and left parameters. May be zero, in which
 *      case item one is positioned at the specified global point.
 *  
 *  Result:
 *    A 32-bit value whose high 16-bit word is the menu ID and whose
 *    low 16-bit word is the index of the menu item that was selected,
 *    or zero if no item was selected.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PopUpMenuSelect( menu: MenuRef; top: SInt16; left: SInt16; popUpItem: MenuItemIndex ): SInt32; external name '_PopUpMenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MenuChoice()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MenuChoice: SInt32; external name '_MenuChoice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MenuEvent()
 *  
 *  Summary:
 *    Determines whether an event should result in the selection of a
 *    menu item.
 *  
 *  Discussion:
 *    Apple recommends that you use the IsMenuKeyEvent API rather than
 *    this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine. Typically, this is a keyboard event.
 *  
 *  Result:
 *    If the keyboard event corresponds to a menu item's keyboard
 *    equivalent, then the result is a 32-bit value whose high 16-bit
 *    word is the menu ID and whose low 16-bit word is the index of the
 *    menu item that was found. Zero is returned if no item matched the
 *    event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function MenuEvent( const (*var*) inEvent: EventRecord ): UInt32; external name '_MenuEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuKeyEvent()
 *  
 *  Summary:
 *    Determines if an event corresponds to a menu command key.
 *  
 *  Discussion:
 *    By default, IsMenuKeyEvent searches the menus in the current menu
 *    bar and hilites the menu title of the menu containing the
 *    selected item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inStartMenu:
 *      IsMenuKeyEvent searches for matching menu items in this menu
 *      and all of its submenus. May be NULL to search the current menu
 *      bar contents.
 *    
 *    inEvent:
 *      The event to match against. Non-keyboard events are ignored.
 *    
 *    inOptions:
 *      Options controlling how to search. Pass kNilOptions for the
 *      default behavior.
 *    
 *    outMenu:
 *      On exit, the menu containing the matching item. May be NULL.
 *    
 *    outMenuItem:
 *      On exit, the menu item that matched. May be NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuKeyEvent( inStartMenu: MenuRef; inEvent: EventRef; inOptions: MenuEventOptions; outMenu: MenuRefPtr { can be NULL }; outMenuItem: MenuItemIndexPtr { can be NULL } ): Boolean; external name '_IsMenuKeyEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalidateMenuEnabling()
 *  
 *  Summary:
 *    Causes the menu enable state to be recalculated at the next
 *    convenient opportunity.
 *  
 *  Discussion:
 *    It is common for state changes in an application (for example,
 *    selection of text) to cause a change in the enabling of items in
 *    the application's menu (for example, the Copy menu item might
 *    become enabled). In a Carbon-event-savvy application, menu items
 *    are enabled or disabled in response to an
 *    kEventCommandUpdateStatus event; however, this event is normally
 *    only sent before a command key press or a click in the menubar.
 *    You can request an explicit recalculation of a menu's enable
 *    state with the InvalidateMenuEnabling API. The Carbon Event
 *    Manager will automatically invalidate the enable state of all
 *    top-level menus when a user event is dispatched, the user focus
 *    changes, or the active window changes, so in many cases you will
 *    not need to explicitly invalidate the menu enabling state.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      A menu to re-enable, or NULL if all menus in the root menu
 *      should be re-enabled.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function InvalidateMenuEnabling( inMenu: MenuRef ): OSStatus; external name '_InvalidateMenuEnabling';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}


{
 *  Summary:
 *    Menu dismissal causation constants
 }
const
{
   * The menu was dismissed by the selection of a menu item.
   }
	kHIMenuDismissedBySelection = 1;

  {
   * The menu was dismissed because the user canceled menu tracking.
   }
	kHIMenuDismissedByUserCancel = 2;

  {
   * The menu was dismissed by a mouse-down somewhere that did not
   * result in menu item selection.
   }
	kHIMenuDismissedByMouseDown = 3;

  {
   * The menu was dismissed by a mouse-up.
   }
	kHIMenuDismissedByMouseUp = 4;

  {
   * The menu was dismissed by a keyboard event.
   }
	kHIMenuDismissedByKeyEvent = 5;

  {
   * The menu was dismissed because the current application was no
   * longer frontmost.
   }
	kHIMenuDismissedByAppSwitch = 6;

  {
   * The menu was dismissed because menu tracking mode timed out.
   }
	kHIMenuDismissedByTimeout = 7;

  {
   * The menu was dismissed by the CancelMenuTracking API.
   }
	kHIMenuDismissedByCancelMenuTracking = 8;

  {
   * The menu was dismissed because the active window changed.
   }
	kHIMenuDismissedByActivationChange = 9;

  {
   * The menu was dismissed bcause the user focus window changed, or
   * because keyboard focus was removed from the current process.
   }
	kHIMenuDismissedByFocusChange = 10;

{$ifc not TARGET_CPU_64}
{
 *  CancelMenuTracking()
 *  
 *  Summary:
 *    Cancels a menu tracking session.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inRootMenu:
 *      The root menu of the menu tracking session that should be
 *      dismissed. For menubar tracking, use the result of AcquireRoot
 *      menu; for popup menu tracking, use the menu that was passed to
 *      PopUpMenuSelect.
 *    
 *    inImmediate:
 *      Whether the open menus should disappear immediately or fade out.
 *    
 *    inDismissalReason:
 *      Why the menu is being dismissed; this value will be added to
 *      the kEventMenuEndTracking event. On Mac OS X 10.5 and later,
 *      you may pass zero to indicate that
 *      kHIMenuDismissedByCancelMenuTracking should be passed to the
 *      EndTracking event.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function CancelMenuTracking( inRootMenu: MenuRef; inImmediate: Boolean; inDismissalReason: UInt32 ): OSStatus; external name '_CancelMenuTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Bar                                                                          }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  GetMBarHeight()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMBarHeight: SInt16; external name '_GetMBarHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]DrawMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DrawMenuBar; external name '_DrawMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacDrawMenuBar; external name '_DrawMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InvalMenuBar; external name '_InvalMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuBarInvalid()
 *  
 *  Summary:
 *    Determines if the menubar is invalid and should be redrawn.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    rootMenu:
 *      The root menu for the menubar to be examined. Pass NULL to
 *      check the state of the current menubar.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuBarInvalid( rootMenu: MenuRef ): Boolean; external name '_IsMenuBarInvalid';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HiliteMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure HiliteMenu( menuID_: MenuID ); external name '_HiliteMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetNewMBar()
 *  
 *  Summary:
 *    Creates multiple menus using the contents of a 'MBAR' resource.
 *  
 *  Discussion:
 *    Apple recommends that you use Interface Builder to design your
 *    menus and other user interface components. Interface Builder
 *    stores its output in nib files rather then Resource Manager
 *    resources. Use the CreateMenuBarFromNib API to load a menu from a
 *    nib file.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    menuBarID:
 *      The resource ID of the 'MBAR' resource.
 *  
 *  Result:
 *    The newly created menubar handle. Use SetMenuBar to place the
 *    contents of the menubar handle into the menubar. You must release
 *    the menubar handle with DisposeMenuBar when you are done with it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetNewMBar( menuBarID: SInt16 ): MenuBarHandle; external name '_GetNewMBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMenuBar: MenuBarHandle; external name '_GetMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMenuBar( mbar: MenuBarHandle ); external name '_SetMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DuplicateMenuBar()
 *  
 *  Summary:
 *    Duplicates a menubar handle.
 *  
 *  Discussion:
 *    This API should be used in Carbon applications when duplicating a
 *    handle returned from GetMenuBar or GetNewMBar. You should not use
 *    Memory Manager APIs (HandToHand, NewHandle, etc) to duplicate
 *    such a handle. This is necessary in Carbon so that the refcounts
 *    of the menus in the menubar handle can be incremented when the
 *    handle is duplicated.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMbar:
 *      The menubar handle to duplicate.
 *    
 *    outMbar:
 *      On exit, contains the new menubar handle.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function DuplicateMenuBar( inMbar: MenuBarHandle; var outMbar: MenuBarHandle ): OSStatus; external name '_DuplicateMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeMenuBar()
 *  
 *  Summary:
 *    Releases a menubar handle.
 *  
 *  Discussion:
 *    This API should be used in Carbon applications when releasing a
 *    handle returned from GetMenuBar or GetNewMBar. You should not use
 *    DisposeHandle to release such a handle. This is necessary in
 *    Carbon so that the refcounts of the menus in the menubar handle
 *    can be decremented when the handle is released.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMbar:
 *      The menubar handle to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function DisposeMenuBar( inMbar: MenuBarHandle ): OSStatus; external name '_DisposeMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMenuHandle( menuID_: MenuID ): MenuRef; external name '_GetMenuHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
function GetMenuRef( menuID_: MenuID ): MenuRef; external name '_GetMenuHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]InsertMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertMenu( theMenu: MenuRef; beforeID: MenuID ); external name '_InsertMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacInsertMenu( theMenu: MenuRef; beforeID: MenuID ); external name '_InsertMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  [Mac]DeleteMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DeleteMenu( menuID_: MenuID ); external name '_DeleteMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacDeleteMenu( menuID_: MenuID ); external name '_DeleteMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ClearMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure ClearMenuBar; external name '_ClearMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FlashMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure FlashMenuBar( menuID_: MenuID ); external name '_FlashMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuBarVisible()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function IsMenuBarVisible: Boolean; external name '_IsMenuBarVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ShowMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure ShowMenuBar; external name '_ShowMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HideMenuBar()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure HideMenuBar; external name '_HideMenuBar';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AcquireRootMenu()
 *  
 *  Summary:
 *    Get the menu whose contents are displayed in the menubar.
 *  
 *  Discussion:
 *    The refcount of the root menu is incremented by this API. The
 *    caller should release a refcount with ReleaseMenu when it’s done
 *    with the menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AcquireRootMenu: MenuRef; external name '_AcquireRootMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetRootMenu()
 *  
 *  Summary:
 *    Sets the menu whose contents are displayed in the menubar.
 *  
 *  Discussion:
 *    The refcount of the root menu is incremented by this API. The
 *    caller may release the menu after calling SetRootMenu.
 *    
 *    A root menu should contain one menu item for each top-level menu
 *    that should be displayed in the menubar. Each menu item should
 *    have a submenu that was installed with
 *    SetMenuItemHierarchicalMenu.
 *    
 *    SetRootMenu also sets the contents of the hierarchical portion of
 *    the menulist (the set of menus that were inserted with
 *    InsertMenu( menu, kInsertHierarchicalMenu). If a menu that was
 *    returned by AcquireRootMenu is passed to SetRootMenu, the
 *    hierarchical menulist is changed to include the menus that were
 *    in the hierarchical menulist when AcquireRootMenu was called. If
 *    a newly created menu is passed to SetRootMenu, the hierarchical
 *    menulist is cleared and has no menus in it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The new root menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetRootMenu( inMenu: MenuRef ): OSStatus; external name '_SetRootMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Accessors                                                               }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  [Mac]CheckMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
procedure CheckMenuItem( theMenu: MenuRef; item: MenuItemIndex; checked: Boolean ); external name '_CheckMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacCheckMenuItem( theMenu: MenuRef; item: MenuItemIndex; checked: Boolean ); external name '_CheckMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetItemMark()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemMark( theMenu: MenuRef; item: MenuItemIndex; markChar: CharParameter ); external name '_SetItemMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetItemMark()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }

procedure __GetItemMark( theMenu: MenuRef; item: MenuItemIndex; var markChar: UInt16 ); external name '_GetItemMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
		With GPC and FPC on Intel we have an endian issue, because markChar is expected to be 16-bit
}

procedure GetItemMark( theMenu: MenuRef; item: MenuItemIndex; var markChar: CharParameter ); inline;


{
 *  SetItemCmd()
 *  
 *  Summary:
 *    Sets a menu item's keyboard equivalent to a specified character
 *    in the application's text encoding.
 *  
 *  Discussion:
 *    Apple recommends that you use the SetMenuItemCommandKey API
 *    instead of this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose item to modify.
 *    
 *    item:
 *      The item whose keyboard equivalent to set.
 *    
 *    cmdChar:
 *      A character in the application's text encoding, or zero to
 *      remove the item's keyboard equivalent.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemCmd( theMenu: MenuRef; item: MenuItemIndex; cmdChar: CharParameter ); external name '_SetItemCmd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetItemCmd()
 *  
 *  Summary:
 *    Returns a menu item's keyboard equivalent in the application's
 *    text encoding.
 *  
 *  Discussion:
 *    Apple recommends that you use the GetMenuItemCommandKey API
 *    instead of this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose item to query.
 *    
 *    item:
 *      The item whose keyboard equivalent to return.
 *    
 *    cmdChar:
 *      On exit, contains the menu item's keyboard equivalent, or zero
 *      if the item has no keyboard equivalent.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure __GetItemCmd( theMenu: MenuRef; item: MenuItemIndex; var cmdChar: UInt16 ); external name '_GetItemCmd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
		With GPC and FPC on Intel we have an endian issue, because cmdChar is expected to be 16-bit
}

procedure GetItemCmd( theMenu: MenuRef; item: MenuItemIndex; var cmdChar: CharParameter ); inline;


{
 *  SetItemStyle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemStyle( theMenu: MenuRef; item: MenuItemIndex; chStyle: StyleParameter ); external name '_SetItemStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetItemStyle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetItemStyle( theMenu: MenuRef; item: MenuItemIndex; var chStyle: Style ); external name '_GetItemStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemCommandID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemCommandID( inMenu: MenuRef; inItem: MenuItemIndex; inCommandID: MenuCommand ): OSErr; external name '_SetMenuItemCommandID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemCommandID()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemCommandID( inMenu: MenuRef; inItem: MenuItemIndex; var outCommandID: MenuCommand ): OSErr; external name '_GetMenuItemCommandID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemModifiers()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemModifiers( inMenu: MenuRef; inItem: MenuItemIndex; inModifiers: SInt8 ): OSErr; external name '_SetMenuItemModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemModifiers()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemModifiers( inMenu: MenuRef; inItem: MenuItemIndex; var outModifiers: SInt8 ): OSErr; external name '_GetMenuItemModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemIconHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemIconHandle( inMenu: MenuRef; inItem: MenuItemIndex; inIconType: SInt8; inIconHandle: Handle ): OSErr; external name '_SetMenuItemIconHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemIconHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemIconHandle( inMenu: MenuRef; inItem: MenuItemIndex; var outIconType: SInt8; var outIconHandle: Handle ): OSErr; external name '_GetMenuItemIconHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)



{
 *  SetMenuItemFontID()
 *  
 *  Summary:
 *    Sets the QuickDraw font ID used to draw a menu item.
 *  
 *  Discussion:
 *    The font ID specified with this API overrides the font ID
 *    specified with the SetMenuFont API. 
 *    
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuSetFont instead of SetMenuItemFontID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item whose font to set.
 *    
 *    inItem:
 *      The menu item whose font to set.
 *    
 *    inFontID:
 *      The menu item's font, or zero to use the standard font for menu
 *      items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemFontID( inMenu: MenuRef; inItem: MenuItemIndex; inFontID: SInt16 ): OSErr; external name '_SetMenuItemFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemFontID()
 *  
 *  Summary:
 *    Retrieves the QuickDraw font ID used to draw a menu item.
 *  
 *  Discussion:
 *    This API will be deprecated in the next major release of Mac OS
 *    X. Use HIMenuCopyFont instead of GetMenuItemFontID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item whose font to retrieve.
 *    
 *    inItem:
 *      The menu item whose font to retrieve.
 *    
 *    outFontID:
 *      On exit, contains the menu item's font, or zero if the font has
 *      not been customized.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemFontID( inMenu: MenuRef; inItem: MenuItemIndex; var outFontID: SInt16 ): OSErr; external name '_GetMenuItemFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemRefCon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemRefCon( inMenu: MenuRef; inItem: MenuItemIndex; inRefCon: URefCon ): OSErr; external name '_SetMenuItemRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemRefCon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemRefCon( inMenu: MenuRef; inItem: MenuItemIndex; var outRefCon: URefCon ): OSErr; external name '_GetMenuItemRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemKeyGlyph()
 *  
 *  Summary:
 *    Sets the command key glyph code for a menu item.
 *  
 *  Discussion:
 *    A menu item's command key may be customized using a key glyph
 *    code; these codes are the kMenu*Glyph constants documented in
 *    Menus.h. In classic Mac OS, a glyph code is only used for
 *    display; it does not affect command key matching. In Carbon, a
 *    menu item's glyph code is used for command key matching if the
 *    menu item does not have a command key character or virtual
 *    keycode assigned to it. 
 *    
 *    In CarbonLib 1.2 and Mac OS X 10.0 and later, the Menu Manager
 *    will automatically draw the appropriate glyph for a menu item
 *    that has a virtual keycode command key assigned to it; it is not
 *    necessary to set both the virtual keycode and the glyph for an
 *    item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to change.
 *    
 *    inItem:
 *      The menu item to change.
 *    
 *    inGlyph:
 *      The new glyph code for the item, or zero to remove the item's
 *      glyph code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemKeyGlyph( inMenu: MenuRef; inItem: MenuItemIndex; inGlyph: SInt16 ): OSErr; external name '_SetMenuItemKeyGlyph';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemKeyGlyph()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemKeyGlyph( inMenu: MenuRef; inItem: MenuItemIndex; var outGlyph: SInt16 ): OSErr; external name '_GetMenuItemKeyGlyph';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Routines available in Mac OS 8.5 and later (supporting enabling/disabling of > 31 items)}

{
 *  [Mac]EnableMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure EnableMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_EnableMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacEnableMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_EnableMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisableMenuItem()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure DisableMenuItem( theMenu: MenuRef; item: MenuItemIndex ); external name '_DisableMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuItemEnabled()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function IsMenuItemEnabled( menu: MenuRef; item: MenuItemIndex ): Boolean; external name '_IsMenuItemEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EnableMenuItemIcon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure EnableMenuItemIcon( theMenu: MenuRef; item: MenuItemIndex ); external name '_EnableMenuItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisableMenuItemIcon()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
procedure DisableMenuItemIcon( theMenu: MenuRef; item: MenuItemIndex ); external name '_DisableMenuItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuItemIconEnabled()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function IsMenuItemIconEnabled( menu: MenuRef; item: MenuItemIndex ): Boolean; external name '_IsMenuItemIconEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemHierarchicalMenu()
 *  
 *  Summary:
 *    Attaches a submenu to a menu item.
 *  
 *  Discussion:
 *    Using SetMenuItemHierarchicalMenu, it is possible to directly
 *    specify the submenu for a menu item without specifying its menu
 *    ID. It is not necessary to insert the submenu into the
 *    hierarchical portion of the menubar, and it is not necessary for
 *    the submenu to have a unique menu ID; it is recommended that you
 *    use 0 as the menu ID for the submenu, and identify selections
 *    from the menu by command ID. The Menu Manager will increment the
 *    refcount of the submenu that you specify, and the submenu's
 *    refcount will be decremented automatically when the parent menu
 *    item is deleted or the parent menu is disposed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The parent menu.
 *    
 *    inItem:
 *      The parent item.
 *    
 *    inHierMenu:
 *      The submenu. You may pass NULL to remove any existing submenu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemHierarchicalMenu( inMenu: MenuRef; inItem: MenuItemIndex; inHierMenu: MenuRef { can be NULL } ): OSStatus; external name '_SetMenuItemHierarchicalMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemHierarchicalMenu()
 *  
 *  Summary:
 *    Returns the submenu attached to a menu item.
 *  
 *  Discussion:
 *    GetMenuItemHierarchicalMenu will return the submenu attached to a
 *    menu item regardless of how the submenu was specified. If the
 *    submenu was specified by menu ID (using SetItemCmd or
 *    SetMenuItemHierarchicalID), GetMenuItemHierarchicalMenu will
 *    return the currently installed menu with that ID, if any. The
 *    only case where GetMenuItemHierarchicalMenu will fail to return
 *    the item's submenu is when the submenu is specified by menu ID,
 *    but the submenu is not currently inserted in the menu bar.
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
function NewMenu( menuID_: MenuID; const (*var*) menuTitle: Str255 ): MenuRef; external name '_NewMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuTitleAsCFString instead of GetMenuTitle.
 *  
 *  Summary:
 *    Retrieves the title of a menu as a Pascal string.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The parent menu.
 *    
 *    inItem:
 *      The parent item.
 *    
 *    outHierMenu:
 *      On exit, the item's submenu, or NULL if it does not have one.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemHierarchicalMenu( inMenu: MenuRef; inItem: MenuItemIndex; var outHierMenu: MenuRef ): OSStatus; external name '_GetMenuItemHierarchicalMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HIMenuCopyParents()
 *  
 *  Summary:
 *    Returns the parent menus of a menu.
 *  
 *  Discussion:
 *    A menu may be a submenu of one or more other menus; for example,
 *    a Spelling menu might be a submenu of both an Edit menu contained
 *    in the menubar, and a contextual menu displayed when the user
 *    clicks in a document. HIMenuCopyParents returns an array of
 *    MenuRefs that are the parent menus of the specified menu. If the
 *    menu has only a single parent, the array will have only that
 *    single parent menu in it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose parent menus to return.
 *    
 *    outParents:
 *      On exit, contains an array of MenuRefs that are parents of the
 *      specified menu. If the menu does not have any parents, the
 *      array on exit will be NULL. If not NULL, the caller is
 *      responsible for releasing the array.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyParents( inMenu: MenuRef; var outParents: CFArrayRef ): OSStatus; external name '_HIMenuCopyParents';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  CopyMenuItemTextAsCFString()
 *  
 *  Summary:
 *    Returns a CFString containing the text of a menu item.
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
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
>>>>>>> graemeg/cpstrnew
function GetMenuTitle( menu: MenuRef; var title: Str255 ): StringPtr; external name '_GetMenuTitle';
=======
function SetMenuTitle( menu: MenuRef; const (*var*) title: Str255 ): OSStatus; external name '_SetMenuTitle';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuTitleWithCFString instead of SetMenuTitle.
 *  
 *  Summary:
 *    Sets the title of a menu to a Pascal string.
=======
 *  [Mac]AppendMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AppendMenuItemTextWithCFString instead of AppendMenu. Note
 *    that unlike AppendMenu, AppendMenuItemTextWithCFString does not
 *    interpret metacharacters in the text.
 *  
 *  Summary:
 *    Appends a new menu item to a menu, using a Pascal string for the
 *    item's text.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose text to return.
 *    
 *    outString:
 *      On exit, a CFString containing the item's text. This string
 *      must be released by the caller.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuItemTextAsCFString( inMenu: MenuRef; inItem: MenuItemIndex; var outString: CFStringRef ): OSStatus; external name '_CopyMenuItemTextAsCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemTextWithCFString()
 *  
 *  Summary:
 *    Sets the text of a menu item to the text contained in a CFString.
 *  
 *  Discussion:
 *    The Menu Manager will either make its own copy or just increment
 *    the refcount of the CFString before returning from
 *    SetMenuItemTextWithCFString, depending on whether the string is
 *    mutable or immutable. If the string is mutable, modifying the
 *    string after calling SetMenuItemTextWithCFString will have no
 *    effect on the menu item's actual text. The caller may release the
 *    string after calling SetMenuItemTextWithCFString.
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
<<<<<<< HEAD
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function SetMenuTitle( menu: MenuRef; const (*var*) title: Str255 ): OSStatus; external name '_SetMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  [Mac]AppendMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AppendMenuItemTextWithCFString instead of AppendMenu. Note
 *    that unlike AppendMenu, AppendMenuItemTextWithCFString does not
 *    interpret metacharacters in the text.
 *  
 *  Summary:
 *    Appends a new menu item to a menu, using a Pascal string for the
 *    item's text.
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure AppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacAppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  [Mac]InsertMenuItem()
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose text to set.
 *    
 *    inString:
 *      The string containing the new menu item text.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemTextWithCFString( inMenu: MenuRef; inItem: MenuItemIndex; inString: CFStringRef ): OSStatus; external name '_SetMenuItemTextWithCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemIndent()
 *  
 *  Summary:
 *    Gets the indent level of a menu item.
 *  
 *  Discussion:
 *    The indent level of an item is an amount of extra space added to
 *    the left of the item's icon or checkmark. The level is simply a
 *    number, starting at zero, which the Menu Manager multiplies by a
 *    constant to get the indent in pixels. The default indent level is
 *    zero.
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
=======
 *  Parameters:
 *    
 *    theMenu:
 *      The menu into which to insert the item.
 *    
 *    itemString:
 *      The text of the new item. This string is parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
>>>>>>> graemeg/cpstrnew
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
<<<<<<< HEAD
procedure AppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacAppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
<<<<<<< HEAD
<<<<<<< HEAD
 *  [Mac]InsertMenuItem()   *** DEPRECATED ***
=======
 *  HIMenuSetAttributedItemText()
 *  
 *  Summary:
 *    Sets the attributed text of a menu item.
 *  
 *  Discussion:
 *    A menu item may have two strings attached to it: a regular
 *    CFString and an attributed CFString. The regular string is
 *    modified by the SetMenuItemTextAsCFString API, and the attributed
 *    string is modified using this API. If an attributed string is
 *    provided, it will be used when measuring and drawing the menu
 *    item text. However, the regular string (if provided) is always
 *    used for type-selection; this allows the attributed string to
 *    contain extra characters that should be ignored during
 *    type-selection.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to set.
 *    
 *    inText:
 *      The new attributed text for the menu item. May be NULL if the
 *      item no longer need attributed text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; inText: CFAttributedStringRef { can be NULL } ): OSStatus; external name '_HIMenuSetAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyAttributedItemText()
 *  
 *  Summary:
 *    Retrieves the attributed text of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the specified item.
 *    
 *    inItem:
 *      The menu item whose attributed text to retrieve.
 *    
 *    outText:
 *      On exit, contains the item's attributed text, or NULL if the
 *      item has no attributed text.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyAttributedItemText( inMenu: MenuRef; inItem: MenuItemIndex; var outText: CFAttributedStringRef ): OSStatus; external name '_HIMenuCopyAttributedItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  GetMenuItemIndent()
 *  
 *  Summary:
 *    Gets the indent level of a menu item.
>>>>>>> origin/fixes_2.4
 *  
 *  Deprecated:
 *    Use InsertMenuItemTextWithCFString instead of InsertMenuItem.
 *    Note that unlike InsertMenuItem InsertMenuItemTextWithCFString
 *    does not interpret metacharacters in the text.
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Pascal string for
 *    the item text.
=======
 *  [Mac]InsertMenuItem()
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
>>>>>>> graemeg/cpstrnew
=======
procedure InsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacInsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AppendMenuItemText()
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose indent to retrieve.
 *    
 *    outIndent:
 *      On exit, the indent level of the item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemIndent( inMenu: MenuRef; inItem: MenuItemIndex; var outIndent: UInt32 ): OSStatus; external name '_GetMenuItemIndent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemIndent()
 *  
 *  Summary:
 *    Sets the indent level of a menu item.
 *  
 *  Discussion:
 *    The indent level of an item is an amount of extra space added to
 *    the left of the item's icon or checkmark. The level is simply a
 *    number, starting at zero, which the Menu Manager multiplies by a
 *    constant to get the indent in pixels. The default indent level is
 *    zero.
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
 *    theMenu:
 *      The menu into which to insert the item.
 *    
 *    itemString:
 *      The text of the new item. This string is parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacInsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose indent to set.
 *    
 *    inIndent:
 *      The new indent level of the item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemIndent( inMenu: MenuRef; inItem: MenuItemIndex; inIndent: UInt32 ): OSStatus; external name '_SetMenuItemIndent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  AppendMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AppendMenuItemTextWithCFString instead of AppendMenuItemText.
 *  
 *  Summary:
 *    Appends a menu item using a Pascal string for the item's text,
 *    without interpreting metacharacters in the text.
=======
 *  AppendMenuItemText()
>>>>>>> graemeg/cpstrnew
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function AppendMenuItemText( menu: MenuRef; const (*var*) inString: Str255 ): OSStatus; external name '_AppendMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use InsertMenuItemTextWithCFString instead of InsertMenuItemText.
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  GetMenuItemCommandKey()
 *  
 *  Summary:
 *    Gets the keyboard equivalent of a menu item.
 *  
 *  Discussion:
 *    A menu item's keyboard equivalent may be either a character code
 *    or a virtual keycode. An item's character code and virtual
 *    keycode are stored separately and may contain different values,
 *    but only one is used by the Menu Manager at any given time. When
 *    requesting a menu item's virtual keycode equivalent, you should
 *    first check that the item is using a virtual keycode by testing
 *    the kMenuItemAttrUseVirtualKey attribute for that item. If this
 *    attribute is not set, the item's virtual keycode is ignored by
 *    the Menu Manager. Note that zero is a valid virtual keycode, so
 *    you cannot test the returned keycode against zero to determine if
 *    the item is using a virtual keycode equivalent. You must test the
 *    kMenuItemAttrUseVirtualKey attribute.
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
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function AppendMenuItemText( menu: MenuRef; const (*var*) inString: Str255 ): OSStatus; external name '_AppendMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use InsertMenuItemTextWithCFString instead of InsertMenuItemText.
 *  
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
=======
 *  Parameters:
 *    
 *    menu:
 *      The menu into which to insert the item.
 *    
 *    inString:
 *      The text of the new item. This string is not parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh; the new item's text becomes exactly the
 *      specified text.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function InsertMenuItemText( menu: MenuRef; const (*var*) inString: Str255; afterItem: MenuItemIndex ): OSStatus; external name '_InsertMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of SetMenuItemText.
 *  
 *  Summary:
 *    Sets the text of a menu item using a Pascal string.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose keyboard equivalent to retrieve.
 *    
 *    inGetVirtualKey:
 *      Indicates whether to retrieve the item's character code or
 *      virtual keycode equivalent.
 *    
 *    outKey:
 *      On exit, the keyboard equivalent of the item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemCommandKey( inMenu: MenuRef; inItem: MenuItemIndex; inGetVirtualKey: Boolean; var outKey: UInt16 ): OSStatus; external name '_GetMenuItemCommandKey';
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemCommandKey()
 *  
 *  Summary:
 *    Sets the keyboard equivalent of a menu item.
 *  
 *  Discussion:
 *    A menu item's keyboard equivalent may be either a character code
 *    or a virtual keycode. The character code is always used to draw
 *    the item's keyboard equivalent in the menu, but either may be
 *    used for keyboard equivalent matching by MenuEvent and
 *    IsMenuKeyEvent, depending on whether the
 *    kMenuItemAttrUseVirtualKey item attribute is set. If
 *    SetMenuItemCommandKey is used to set the virtual keycode
 *    equivalent for a menu item, it also automatically sets the
 *    kMenuItemAttrUseVirtualKey item attribute. To make the menu item
 *    stop using the virtual keycode equivalent and use the character
 *    code equivalent instead, use ChangeMenuItemAttributes to clear
 *    the kMenuItemAttrUseVirtualKey item attribute.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Parameters:
 *    
 *    menu:
 *      The menu into which to insert the item.
 *    
 *    inString:
 *      The text of the new item. This string is not parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh; the new item's text becomes exactly the
 *      specified text.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function InsertMenuItemText( menu: MenuRef; const (*var*) inString: Str255; afterItem: MenuItemIndex ): OSStatus; external name '_InsertMenuItemText';
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; const (*var*) itemString: Str255 ); external name '_SetMenuItemText';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of SetMenuItemText.
 *  
 *  Summary:
 *    Sets the text of a menu item using a Pascal string.
=======
 *  GetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of GetMenuItemText.
 *  
 *  Summary:
 *    Retrieves the text of a menu item as a Pascal string.
>>>>>>> graemeg/cpstrnew
=======
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemCommandKey()
 *  
 *  Summary:
 *    Sets the keyboard equivalent of a menu item.
 *  
 *  Discussion:
 *    A menu item's keyboard equivalent may be either a character code
 *    or a virtual keycode. The character code is always used to draw
 *    the item's keyboard equivalent in the menu, but either a char
 *    code or a virtual key code may be used for keyboard equivalent
 *    matching by MenuEvent and IsMenuKeyEvent, depending on whether
 *    the kMenuItemAttrUseVirtualKey item attribute is set. If
 *    SetMenuItemCommandKey is used to set the virtual keycode
 *    equivalent for a menu item, it also automatically sets the
 *    kMenuItemAttrUseVirtualKey item attribute. To make the menu item
 *    stop using the virtual keycode equivalent and use the character
 *    code equivalent instead, use ChangeMenuItemAttributes to clear
 *    the kMenuItemAttrUseVirtualKey item attribute.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu containing the item.
 *    
 *    inItem:
 *      The item whose keyboard equivalent to set.
 *    
 *    inSetVirtualKey:
 *      Indicates whether to set the item's character code or virtual
 *      keycode equivalent. If setting a character code, the character
 *      is assumed to be in the application's text encoding.
 *    
 *    inKey:
 *      The item's new character code or virtual keycode equivalent.
 *      Note that the character code is not a Unicode value; rather,
 *      the character code is assumed to be expressed in the
 *      application's text encoding. In Mac OS X 10.3 and later, the
 *      Menu Manager also supports the following metavalues when
 *      inSetVirtualKey is false. When receiving one of these values,
 *      the Menu Manager automatically sets the item's keyboard
 *      equivalent glyph code to an appropriate value using
 *      SetMenuItemKeyGlyph. 
 *      
 *      NSF1FunctionKey..NSF15FunctionKey 
 *      NSHomeFunctionKey 
 *      kEnterCharCode 
 *      kEndCharCode 
 *      NSEndFunctionKey 
 *      kHelpCharCode 
 *      NSHelpFunctionKey 
 *      kBackspaceCharCode 
 *      kTabCharCode 
 *      kLineFeedCharCode 
 *      kReturnCharCode 
 *      kPageUpCharCode 
 *      NSPageUpFunctionKey 
 *      kPageDownCharCode 
 *      NSPageDownFunctionKey 
 *      NSBackTabCharacter 
 *      kEscapeCharCode 
 *      NSClearDisplayFunctionKey 
 *      kLeftArrowCharCode 
 *      NSLeftArrowFunctionKey 
 *      kRightArrowCharCode 
 *      NSRightArrowFunctionKey 
 *      kUpArrowCharCode 
 *      NSUpArrowFunctionKey 
 *      kDownArrowCharCode 
 *      NSDownArrowFunctionKey 
 *      kDeleteCharCode 
 *      
 *      In Mac OS X 10.4 and later, the Menu Manager also supports
 *      these values: 
 *      
 *      0x2190..0x2193 (left, up, right, and down-arrow)
 *       kSpaceCharCode 
 *      
 *      In Mac OS X 10.6 and later, the Menu Manager also supports
 *      these values: 
 *      NSF16FunctionKey..NSF19FunctionKey, NSDeleteFunctionKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemCommandKey( inMenu: MenuRef; inItem: MenuItemIndex; inSetVirtualKey: Boolean; inKey: UInt16 ): OSStatus; external name '_SetMenuItemCommandKey';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{ • Menu item views                                                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
{
<<<<<<< HEAD
 *  DeleteMCEntries()
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure SetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; const (*var*) itemString: Str255 ); external name '_SetMenuItemText';
=======
procedure GetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; var itemString: Str255 ); external name '_GetMenuItemText';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of GetMenuItemText.
 *  
 *  Summary:
 *    Retrieves the text of a menu item as a Pascal string.
=======
 *  SetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of setting an item's text
 *    as a Str255.
 *  
 *  Summary:
 *    Sets the text encoding that should be used to interpret a menu
 *    item's Pascal string-based text.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DeleteMCEntries( menuID_: MenuID; menuItem: SInt16 ); external name '_DeleteMCEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMCInfo()
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; var itemString: Str255 ); external name '_GetMenuItemText';
=======
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; inScriptID: TextEncoding ): OSErr; external name '_SetMenuItemTextEncoding';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of setting an item's text
 *    as a Str255.
 *  
 *  Summary:
 *    Sets the text encoding that should be used to interpret a menu
 *    item's Pascal string-based text.
=======
 *  GetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of retrieving a menu
 *    item's text as a Str255.
 *  
 *  Summary:
 *    Retrieves the text encoding that should be used to interpret a
 *    menu item's Pascal string-based text.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMCInfo: MCTableHandle; external name '_GetMCInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMCInfo()
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
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
<<<<<<< HEAD
function SetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; inScriptID: TextEncoding ): OSErr; external name '_SetMenuItemTextEncoding';
=======
function GetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; var outScriptID: TextEncoding ): OSErr; external name '_GetMenuItemTextEncoding';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetMenuItemTextEncoding()   *** DEPRECATED ***
=======
 *  HIMenuSetItemViewClass()
 *  
 *  Summary:
 *    Sets the HIObject class ID and initialization event that will be
 *    used to create a custom HIView for a menu item.
 *  
 *  Discussion:
 *    When a menu item has a custom HIObject class ID associated with
 *    it, the standard menu view automatically creates an HIView using
 *    that class ID as necessary and inserts the view as a subview of
 *    the standard menu view. The item view is send a
 *    kEventControlGetOptimalBounds event to request that it calculate
 *    its size, and if this size is larger than the size that the item
 *    would normally have, it is used instead. The view's frame is
 *    automatically positioned to match the frame of the item. 
 *    
 *    An item view should be an ordinary subclass of HIView. The item
 *    view is responsible for entirely replacing the standard menu item
 *    content; the standard content will not be drawn. When created, an
 *    item view's kEventHIObjectInitialize event will have two extra
 *    parameters, kEventParamMenuRef and kEventParamMenuItemIndex, if
 *    the view is intended for use in a menu. Note that a view should
 *    not cache the item index from the Init event, because the item
 *    index can change over time if menu items are inserted or deleted;
 *    instead, a view can determine its current menu item index at any
 *    time (except during Construct or Init handlers) by calling
 *    HIMenuItemViewGetEnclosingMenuItem. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
>>>>>>> origin/fixes_2.4
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of retrieving a menu
 *    item's text as a Str255.
 *  
<<<<<<< HEAD
 *  Summary:
 *    Retrieves the text encoding that should be used to interpret a
 *    menu item's Pascal string-based text.
=======
 *  SetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemIconHandle instead of SetItemIcon.
 *  
 *  Summary:
 *    Sets the resource ID of an icon resource to be displayed in a
 *    menu item.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMCInfo( menuCTbl: MCTableHandle ); external name '_SetMCInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeMCInfo()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Parameters:
 *    
 *    inMenu:
 *      The menu for which to set the item view class.
 *    
 *    inItem:
 *      The menu item for which to set the item view class.
 *    
 *    inClassID:
 *      The class ID of the menu item's custom view. May be NULL to
 *      clear the item's custom view class ID; however, any item view
 *      that had already been created for the item will remain embedded
 *      in the menu content view and will be used to measure and draw
 *      the item.
 *    
 *    inInitEvent:
 *      The initialization event of the menu item's custom view. May be
 *      NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuSetItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; inClassID: CFStringRef { can be NULL }; inInitEvent: EventRef { can be NULL } ): OSStatus; external name '_HIMenuSetItemViewClass';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuCopyItemViewClass()
 *  
 *  Summary:
 *    Returns the HIObject class ID and initialization event that are
 *    used to create a custom HIView for a menu item.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu for which to retrieve the item view class.
 *    
 *    inItem:
 *      The menu item for which to retrieve the item view class.
 *    
 *    outClassID:
 *      On exit, contains the item's custom view class ID, or NULL if
 *      none. The caller should release this string.
 *    
 *    outInitEvent:
 *      On exit, contains the item's custom view initialization event,
 *      or NULL if none. The caller should release this event.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; var outScriptID: TextEncoding ): OSErr; external name '_GetMenuItemTextEncoding';
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemIcon( theMenu: MenuRef; item: MenuItemIndex; iconIndex: SInt16 ); external name '_SetItemIcon';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetItemIcon()   *** DEPRECATED ***
=======
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuCopyItemViewClass( inMenu: MenuRef; inItem: MenuItemIndex; var outClassID: CFStringRef; outInitEvent: EventRefPtr { can be NULL } ): OSStatus; external name '_HIMenuCopyItemViewClass';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuGetItemView()
 *  
 *  Summary:
 *    Returns the HIViewRef that will be used to draw a menu item, if
 *    any.
 *  
 *  Discussion:
 *    If the item view has not yet been created, the Menu Manager will
 *    create the item view using the view class ID and initialization
 *    event specified by the HIMenuSetItemViewClass API. If the menu
 *    item does not have a custom view associated with it, noErr is
 *    returned but outView is set to NULL. 
 *    
 *    This API may only be called on menus that are implemented using
 *    an HIView. paramErr will be returned if an MDEF-based menu is
 *    provided.
>>>>>>> origin/fixes_2.4
 *  
 *  Deprecated:
 *    Use SetMenuItemIconHandle instead of SetItemIcon.
 *  
<<<<<<< HEAD
 *  Summary:
 *    Sets the resource ID of an icon resource to be displayed in a
 *    menu item.
=======
 *  GetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemIconHandle instead of GetItemIcon.
 *  
 *  Summary:
 *    Retrieves the resource ID of an icon resource to be displayed in
 *    a menu item.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DisposeMCInfo( menuCTbl: MCTableHandle ); external name '_DisposeMCInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMCEntry()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Parameters:
 *    
 *    inMenu:
 *      The menu.
 *    
 *    inItem:
 *      The menu item.
 *    
 *    inMenuType:
 *    
 *    outView:
 *      On exit, contains the view. May be set to NULL if the item does
 *      not have a custom item view. The caller should not release this
 *      view.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIMenuGetItemView( inMenu: MenuRef; inItem: MenuItemIndex; inMenuType: ThemeMenuType; var outView: HIViewRef ): OSStatus; external name '_HIMenuGetItemView';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIMenuItemViewGetEnclosingMenuItem()
 *  
 *  Summary:
 *    Locates the menu item that contains a menu item view or one of
 *    its subviews.
 *  
 *  Discussion:
 *    A custom HIView may behave differently depending on whether it's
 *    embedded in a menu or a regular window. This API will tell the
 *    view whether it's contained in a menu or a window. Note that this
 *    API will always return false if called from a view's
 *    kEventHIObjectConstruct or kEventHIObjectInitialize event
 *    handler, even if the view is being created for use in a menu; at
 *    Construct and Init time, the Menu Manager has not yet created the
 *    binding between the view and the item. A view can check for
 *    kEventParamMenuRef and kEventParamMenuItemIndex parameters in its
 *    kEventHIObjectInitialize handler to determine at Init time
 *    whether the view is being placed into a menu. 
 *    
 *    This API may be passed a menu item view or any subview of the
 *    item view. If passed a subview, it will search upwards through
 *    the view hierarchy to look for an item view.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure SetItemIcon( theMenu: MenuRef; item: MenuItemIndex; iconIndex: SInt16 ); external name '_SetItemIcon';
=======
procedure GetItemIcon( theMenu: MenuRef; item: MenuItemIndex; var iconIndex: SInt16 ); external name '_GetItemIcon';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemIconHandle instead of GetItemIcon.
 *  
 *  Summary:
 *    Retrieves the resource ID of an icon resource to be displayed in
 *    a menu item.
=======
 *  SetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemHierarchicalMenu instead of
 *    SetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Sets the menu ID of the submenu of a menu item.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *  Parameters:
 *    
 *    inItemView:
 *      The item view.
 *    
 *    outMenu:
 *      On exit, contains the menu in which the view is embedded. May
 *      be NULL if you don't need the menu.
 *    
 *    outItem:
 *      On exit, contains the menu item that the view is implementing.
 *      May be NULL if you don't need the item. Note that the item
 *      index that is returned may change over time if menu items are
 *      inserted or deleted; generally, a view should not cache the
 *      item index, but should call this API each time that it needs to
 *      know its current item.
 *  
 *  Result:
 *    true if the view is embedded in a menu, or false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function HIMenuItemViewGetEnclosingMenuItem( inItemView: HIViewRef; outMenu: MenuRefPtr { can be NULL }; outItem: MenuItemIndexPtr { can be NULL } ): Boolean; external name '_HIMenuItemViewGetEnclosingMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
<<<<<<< HEAD
 *  SetMCEntries()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  HIMenuItemViewCancelMenuTracking()
 *  
 *  Summary:
 *    Cancels menu tracking for the menu containing an item view.
 *  
 *  Discussion:
 *    In order to call the CancelMenuTracking API, you must know the
 *    menu that is the root for the menu tracking session. This root
 *    menu is often not available to a custom menu item view. This API
 *    allows a menu item view to cancel tracking without knowing the
 *    root menu.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inItemView:
 *      The item view.
 *    
 *    inImmediate:
 *      Whether the open menus should disappear immediately or fade out.
 *    
 *    inDismissalReason:
 *      Why the menu is being dismissed; this value will be added to
 *      the kEventMenuEndTracking event. If zero,
 *      kHIMenuDismissedByCancelMenuTracking is added to the
 *      EndTracking event.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetItemIcon( theMenu: MenuRef; item: MenuItemIndex; var iconIndex: SInt16 ); external name '_GetItemIcon';
=======
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; inHierID: MenuID ): OSErr; external name '_SetMenuItemHierarchicalID';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemHierarchicalMenu instead of
 *    SetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Sets the menu ID of the submenu of a menu item.
=======
 *  GetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemHierarchicalMenu instead of
 *    GetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Retrieves the menu ID of the submenu of a menu item.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *    Mac OS X:         in version 10.5 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
function HIMenuItemViewCancelMenuTracking( inItemView: HIViewRef; inImmediate: Boolean; inDismissalReason: UInt32 ): OSStatus; external name '_HIMenuItemViewCancelMenuTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{ • Properties  (Mac OS 8.5 and later)                                                 }
{ With the following property APIs, you can attach any piece of data you'd like to a   }
{ menu or menu item. Passing zero for the item number parameter indicates you'd like   }
{ to attach the data to the menu itself, and not to any specific menu item.            }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

const
	kMenuPropertyPersistent = $00000001; { whether this property gets saved when archiving the menu; however, ignored in Mac OS X 10.4}

{$ifc not TARGET_CPU_64}
{
 *  GetMenuItemProperty()
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
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
<<<<<<< HEAD
function SetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; inHierID: MenuID ): OSErr; external name '_SetMenuItemHierarchicalID';
=======
function GetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; var outHierID: MenuID ): OSErr; external name '_GetMenuItemHierarchicalID';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemHierarchicalMenu instead of
 *    GetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Retrieves the menu ID of the submenu of a menu item.
=======
 *  GetMenuRetainCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFGetRetainCount instead of
 *    GetMenuRetainCount.
 *  
 *  Summary:
 *    Returns the retain count of a menu.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuItemProperty( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; bufferSize: ByteCount; var actualSize: ByteCount; propertyBuffer: UnivPtr ): OSStatus; external name '_GetMenuItemProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemPropertySize()
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
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; var outHierID: MenuID ): OSErr; external name '_GetMenuItemHierarchicalID';
=======
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to return.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuRetainCount( inMenu: MenuRef ): ItemCount; external name '_GetMenuRetainCount';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetMenuRetainCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFGetRetainCount instead of
 *    GetMenuRetainCount.
 *  
 *  Summary:
 *    Returns the retain count of a menu.
=======
 *  RetainMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRetain instead of RetainMenu.
 *  
 *  Summary:
 *    Increments the retain count of a menu.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function GetMenuItemPropertySize( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; var size: ByteCount ): OSStatus; external name '_GetMenuItemPropertySize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemProperty()
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
 *  Parameters:
 *    
 *    inMenu:
<<<<<<< HEAD
 *      The menu whose retain count to return.
=======
 *      The menu whose retain count to increment.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function GetMenuRetainCount( inMenu: MenuRef ): ItemCount; external name '_GetMenuRetainCount';
=======
function RetainMenu( inMenu: MenuRef ): OSStatus; external name '_RetainMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  RetainMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRetain instead of RetainMenu.
 *  
 *  Summary:
 *    Increments the retain count of a menu.
=======
 *  ReleaseMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRelease instead of ReleaseMenu.
 *  
 *  Summary:
 *    Decrements the retain count of a menu.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function SetMenuItemProperty( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; propertySize: ByteCount; propertyData: {const} UnivPtr ): OSStatus; external name '_SetMenuItemProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveMenuItemProperty()
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
 *  Parameters:
 *    
 *    inMenu:
<<<<<<< HEAD
 *      The menu whose retain count to increment.
=======
 *      The menu whose retain count to decrement. If the retain count
 *      falls to zero, the menu is destroyed.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function RetainMenu( inMenu: MenuRef ): OSStatus; external name '_RetainMenu';
=======
function ReleaseMenu( inMenu: MenuRef ): OSStatus; external name '_ReleaseMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  ReleaseMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRelease instead of ReleaseMenu.
 *  
 *  Summary:
 *    Decrements the retain count of a menu.
=======
 *  InsertResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function RemoveMenuItemProperty( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType ): OSStatus; external name '_RemoveMenuItemProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemPropertyAttributes()
 *  
 *  Summary:
 *    Gets the attributes of a menu item property.
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
 *    inMenu:
 *      The menu whose retain count to decrement. If the retain count
 *      falls to zero, the menu is destroyed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function ReleaseMenu( inMenu: MenuRef ): OSStatus; external name '_ReleaseMenu';
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex ); external name '_InsertResMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  InsertResMenu()   *** DEPRECATED ***
=======
 *  AppendResMenu()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Inserts menu items containing the names of resources of a
=======
 *    Appends menu items containing the names of resources of a
>>>>>>> graemeg/cpstrnew
 *    specified resource type.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    item:
 *      The menu item.
 *    
 *    propertyCreator:
 *      The creator code of the property.
 *    
 *    propertyTag:
 *      The property tag.
 *    
 *    attributes:
 *      On exit, contains the attributes of the property.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemPropertyAttributes( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; var attributes: OptionBits ): OSStatus; external name '_GetMenuItemPropertyAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ChangeMenuItemPropertyAttributes()
 *  
 *  Summary:
 *    Changes the attributes of a menu item property.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure InsertResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex ); external name '_InsertResMenu';
=======
procedure AppendResMenu( theMenu: MenuRef; theType: ResType ); external name '_AppendResMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  AppendResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Appends menu items containing the names of resources of a
 *    specified resource type.
=======
 *  InsertFontResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateStandardFontMenu or a font panel window instead of
 *    InsertFontResMenu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of font resources.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure AppendResMenu( theMenu: MenuRef; theType: ResType ); external name '_AppendResMenu';
=======
procedure InsertFontResMenu( theMenu: MenuRef; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertFontResMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  InsertFontResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateStandardFontMenu or a font panel window instead of
 *    InsertFontResMenu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of font resources.
=======
 *  InsertIntlResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
>>>>>>> graemeg/cpstrnew
=======
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    item:
 *      The menu item.
 *    
 *    propertyCreator:
 *      The creator code of the property.
 *    
 *    propertyTag:
 *      The property tag.
 *    
 *    attributesToSet:
 *      The attributes to add to the menu item property.
 *    
 *    attributesToClear:
 *      The attributes to remove from the menu item property.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ChangeMenuItemPropertyAttributes( menu: MenuRef; item: MenuItemIndex; propertyCreator: OSType; propertyTag: OSType; attributesToSet: OptionBits; attributesToClear: OptionBits ): OSStatus; external name '_ChangeMenuItemPropertyAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Attributes (Carbon and later)                                                     }
{  Each menu and menu item has attribute flags.                                        }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  GetMenuAttributes()
 *  
 *  Summary:
 *    Gets the attributes of a menu.
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
<<<<<<< HEAD
procedure InsertFontResMenu( theMenu: MenuRef; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertFontResMenu';
=======
procedure InsertIntlResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertIntlResMenu';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  InsertIntlResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
=======
 *  MenuKey()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use MenuEvent, IsMenuKeyEvent, or a kEventCommandProcess Carbon
 *    event handler instead of MenuKey.
 *  
 *  Summary:
 *    Determines if a character matches a menu item command key.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    outAttributes:
 *      On exit, contains the attributes of the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuAttributes( menu: MenuRef; var outAttributes: MenuAttributes ): OSStatus; external name '_GetMenuAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ChangeMenuAttributes()
 *  
 *  Summary:
 *    Changes the attributes of a menu.
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
<<<<<<< HEAD
procedure InsertIntlResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertIntlResMenu';
=======
function MenuKey( ch: CharParameter ): SInt32; external name '_MenuKey';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  MenuKey()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use MenuEvent, IsMenuKeyEvent, or a kEventCommandProcess Carbon
 *    event handler instead of MenuKey.
 *  
 *  Summary:
 *    Determines if a character matches a menu item command key.
=======
 *  SetMenuFlashCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function has no effect on Mac OS X.
 *  
 *  Summary:
 *    Sets the number of times that a selected menu item will blink.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    setTheseAttributes:
 *      The attributes to add to the menu.
 *    
 *    clearTheseAttributes:
 *      The attributes to remove from the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ChangeMenuAttributes( menu: MenuRef; setTheseAttributes: MenuAttributes; clearTheseAttributes: MenuAttributes ): OSStatus; external name '_ChangeMenuAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuItemAttributes()
 *  
 *  Summary:
 *    Gets the attributes of a menu item.
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
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MenuKey( ch: CharParameter ): SInt32; external name '_MenuKey';
=======
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
procedure SetMenuFlashCount( count: SInt16 ); external name '_SetMenuFlashCount';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetMenuFlashCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function has no effect on Mac OS X.
 *  
 *  Summary:
 *    Sets the number of times that a selected menu item will blink.
=======
 *  InitContextualMenus()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X, it is not necessary to call InitContextualMenus.
 *  
 *  Summary:
 *    Adds the current process to the system registry of contextual
 *    menu clients.
 *  
 *  Discussion:
 *    On Mac OS 8.x and 9.x, your program should call the
 *    InitContextualMenus function early in your startup code to
 *    register your application as a contextual menu client. If you do
 *    not register your program, some system-level functions may
 *    respond as though your program does not use contextual menus. Not
 *    registering your program may also cause
 *    ProcessIsContextualMenuClient to return an incorrect value.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    item:
 *      The menu item.
 *    
 *    outAttributes:
 *      On exit, contains the attributes of the menu item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuItemAttributes( menu: MenuRef; item: MenuItemIndex; var outAttributes: MenuItemAttributes ): OSStatus; external name '_GetMenuItemAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ChangeMenuItemAttributes()
 *  
 *  Summary:
 *    Changes the attributes of a menu item.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
procedure SetMenuFlashCount( count: SInt16 ); external name '_SetMenuFlashCount';
=======
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function InitContextualMenus: OSStatus; external name '_InitContextualMenus';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  InitContextualMenus()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X, it is not necessary to call InitContextualMenus.
 *  
 *  Summary:
 *    Adds the current process to the system registry of contextual
 *    menu clients.
 *  
 *  Discussion:
 *    On Mac OS 8.x and 9.x, your program should call the
 *    InitContextualMenus function early in your startup code to
 *    register your application as a contextual menu client. If you do
 *    not register your program, some system-level functions may
 *    respond as though your program does not use contextual menus. Not
 *    registering your program may also cause
 *    ProcessIsContextualMenuClient to return an incorrect value.
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *  ProcessIsContextualMenuClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    There is no reason for an application to call this API.
 *  
 *  Summary:
 *    Determines whether the specified process is a contextual menu
 *    client.
 *  
 *  Discussion:
 *    On Mac OS 9, this API consults a global table of all processes
 *    that have registered with the Contextual Menu Manager by calling
 *    InitContextualMenus. On Mac OS X, this API ignores the inPSN
 *    parameter and always returns whether the current process is
 *    registered with the Contextual Menu Manager.
=======
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    menu:
 *      The menu.
 *    
 *    item:
 *      The menu item.
 *    
 *    setTheseAttributes:
 *      The attributes to add to the menu item.
 *    
 *    clearTheseAttributes:
 *      The attributes to remove from the menu item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ChangeMenuItemAttributes( menu: MenuRef; item: MenuItemIndex; setTheseAttributes: MenuItemAttributes; clearTheseAttributes: MenuItemAttributes ): OSStatus; external name '_ChangeMenuItemAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Mass menu item enabling and disabling (Carbon and later)                          }
{  Useful when rewriting code that modifies the enableFlags field directly.            }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  DisableAllMenuItems()
 *  
 *  Summary:
 *    Disables all items in a menu.
 *  
 *  Discussion:
 *    This API is equivalent to pre-Carbon code that masked the
 *    enableFlags field of the MenuInfo with 0x01. It disables all
 *    items (including items past item 31) but does not affect the
 *    state of the menu title.
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
 *    inPSN:
 *      The process to examine.
 *  
 *  Result:
 *    Whether the specified process (or, on Mac OS X, the current
 *    process) is registered with the Contextual Menu Manager.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
<<<<<<< HEAD
function InitContextualMenus: OSStatus; external name '_InitContextualMenus';
=======
function ProcessIsContextualMenuClient( var inPSN: ProcessSerialNumber ): Boolean; external name '_ProcessIsContextualMenuClient';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Color Tables                                                            }
{  Menu color manipulation is considered deprecated with the advent of the Appearance  }
{  Manager.  Avoid using these routines if possible                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
{
<<<<<<< HEAD
 *  ProcessIsContextualMenuClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    There is no reason for an application to call this API.
 *  
 *  Summary:
 *    Determines whether the specified process is a contextual menu
 *    client.
 *  
 *  Discussion:
 *    On Mac OS 9, this API consults a global table of all processes
 *    that have registered with the Contextual Menu Manager by calling
 *    InitContextualMenus. On Mac OS X, this API ignores the inPSN
 *    parameter and always returns whether the current process is
 *    registered with the Contextual Menu Manager.
=======
=======
>>>>>>> origin/fixes_2_2
 *    theMenu:
 *      The menu whose items to disable.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisableAllMenuItems( theMenu: MenuRef ); external name '_DisableAllMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EnableAllMenuItems()
 *  
 *  Summary:
 *    Enables all items in a menu.
 *  
 *  Discussion:
 *    This API is equivalent to pre-Carbon code that or'd the
 *    enableFlags field of the MenuInfo with 0xFFFFFFFE. It enables all
 *    items (including items past item 31) but does not affect the
 *    state of the menu title.
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
 *    inPSN:
 *      The process to examine.
<<<<<<< HEAD
 *  
 *  Result:
 *    Whether the specified process (or, on Mac OS X, the current
 *    process) is registered with the Contextual Menu Manager.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function ProcessIsContextualMenuClient( var inPSN: ProcessSerialNumber ): Boolean; external name '_ProcessIsContextualMenuClient';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Color Tables                                                            }
{  Menu color manipulation is considered deprecated with the advent of the Appearance  }
{  Manager.  Avoid using these routines if possible                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  DeleteMCEntries()   *** DEPRECATED ***
 *  
=======
 *  
 *  Result:
 *    Whether the specified process (or, on Mac OS X, the current
 *    process) is registered with the Contextual Menu Manager.
=======
 *  DeleteMCEntries()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Removes color table entries for a menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function ProcessIsContextualMenuClient( var inPSN: ProcessSerialNumber ): Boolean; external name '_ProcessIsContextualMenuClient';
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DeleteMCEntries( menuID_: MenuID; menuItem: SInt16 ); external name '_DeleteMCEntries';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Color Tables                                                            }
{  Menu color manipulation is considered deprecated with the advent of the Appearance  }
{  Manager.  Avoid using these routines if possible                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
{
<<<<<<< HEAD
 *  DeleteMCEntries()   *** DEPRECATED ***
 *  
>>>>>>> graemeg/cpstrnew
=======
 *  GetMCInfo()   *** DEPRECATED ***
 *  
>>>>>>> graemeg/cpstrnew
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Removes color table entries for a menu.
=======
 *    Retrieves color table entries for the current process.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    theMenu:
 *      The menu whose items to enable.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EnableAllMenuItems( theMenu: MenuRef ); external name '_EnableAllMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  MenuHasEnabledItems()
 *  
 *  Summary:
 *    Determines if any items in a menu are enabled.
 *  
 *  Discussion:
 *    This API is equivalent to pre-Carbon code that compared the
 *    enableFlags field of the MenuInfo with 0. It checks the enable
 *    state of all items to see if any are enabled, but ignores the
 *    state of the menu title. It will return true even if the menu
 *    title is disabled.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose items to examine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MenuHasEnabledItems( theMenu: MenuRef ): Boolean; external name '_MenuHasEnabledItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu tracking status (Carbon and later)                                           }
{  Get info about the selected menu item during menu tracking. Replaces direct access  }
{  to low-mem globals that previously held this info.                                  }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

{
 *  GetMenuTrackingData()
 *  
 *  Summary:
 *    Gets information about the menu currently selected by the user.
 *  
 *  Discussion:
 *    This API replaces direct access to the low-memory globals
 *    TopMenuItem, AtMenuBottom, MenuDisable, and mbSaveLoc. It is only
 *    valid to call this API while menu tracking is occurring. This API
 *    will most commonly be used by custom MDEFs.
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
<<<<<<< HEAD
procedure DeleteMCEntries( menuID_: MenuID; menuItem: SInt16 ); external name '_DeleteMCEntries';
=======
function GetMCInfo: MCTableHandle; external name '_GetMCInfo';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetMCInfo()   *** DEPRECATED ***
=======
 *  SetMCInfo()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Retrieves color table entries for the current process.
=======
 *    Sets color table info for the current process.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theMenu:
 *      The menu about which to get tracking information. May be NULL
 *      to get information about the menu that the user is currently
 *      selecting. If the menu is not currently open, menuNotFoundErr
 *      is returned.
 *    
 *    outData:
 *      On exit, contains tracking data about the menu. On CarbonLib,
 *      the itemRect field is not supported and is always set to an
 *      empty rect.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuTrackingData( theMenu: MenuRef; var outData: MenuTrackingData ): OSStatus; external name '_GetMenuTrackingData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  GetMenuType()
 *  
 *  Summary:
 *    Gets the display type (pulldown, hierarchical, or popup) of a
 *    menu.
 *  
 *  Discussion:
 *    This API may only be called when the menu is displayed. If the
 *    menu is not currently open, an error is returned. The display
 *    type of a menu may vary from one menu tracking session to
 *    another; for example, the same menu might be displayed as a
 *    pulldown menu and as a popup menu.
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
<<<<<<< HEAD
function GetMCInfo: MCTableHandle; external name '_GetMCInfo';
=======
procedure SetMCInfo( menuCTbl: MCTableHandle ); external name '_SetMCInfo';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetMCInfo()   *** DEPRECATED ***
=======
 *  DisposeMCInfo()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Sets color table info for the current process.
=======
 *    Releases the memory used by a menu color table.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theMenu:
 *      The menu whose type to get.
 *    
 *    outType:
 *      On exit, the type of the menu. The returned value will be one
 *      of the ThemeMenuType constants: kThemeMenuTypePullDown, PopUp,
 *      or Hierarchical. The kThemeMenuTypeInactive bit will never be
 *      set.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuType( theMenu: MenuRef; var outType: UInt16 ): OSStatus; external name '_GetMenuType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Universal command ID access (Carbon and later)                                    }
{  These APIs allow you to operate on menu items strictly by command ID, with no       }
{  knowledge of a menu item's index.                                                   }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  CountMenuItemsWithCommandID()
 *  
 *  Summary:
 *    Counts the menu items with a specified command ID.
 *  
 *  Discussion:
 *    In CarbonLib 1.0.x and 1.1, this API always returns zero or one;
 *    it stops after finding the first menu item with the specified
 *    command ID. In CarbonLib 1.2 and Mac OS X 10.0 and later, it
 *    counts all menu items with the specified command ID. In Mac OS X
 *    10.0 and CarbonLib 1.0 through 1.4, this API only searches
 *    top-level menus (menus visible in the menubar) and submenus of
 *    top-level menus. It does not search hierarchical menus that are
 *    inserted in the menubar but are not submenus of a top-level menus
 *    (for example, it does not search menus that are inserted for use
 *    in a popup menu control). In Mac OS X 10.1 and CarbonLib 1.5 and
 *    later, this API also searches inserted hierarchical menus.
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
<<<<<<< HEAD
procedure SetMCInfo( menuCTbl: MCTableHandle ); external name '_SetMCInfo';
=======
procedure DisposeMCInfo( menuCTbl: MCTableHandle ); external name '_DisposeMCInfo';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  DisposeMCInfo()   *** DEPRECATED ***
=======
 *  GetMCEntry()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Releases the memory used by a menu color table.
=======
 *    Returns a pointer to the menu color table entry for a menu item.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for items with the
 *      specified command ID. Pass NULL to hegin searching with the
 *      root menu. The search will descend into all submenus of this
 *      menu.
 *    
 *    inCommandID:
 *      The command ID for which to search.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CountMenuItemsWithCommandID( inMenu: MenuRef; inCommandID: MenuCommand ): ItemCount; external name '_CountMenuItemsWithCommandID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetIndMenuItemWithCommandID()
 *  
 *  Summary:
 *    Finds a menu item with a specified command ID.
 *  
 *  Discussion:
 *    This API searches the specified menu and its submenus for the
 *    n'th menu item with the specified command ID. In CarbonLib 1.0.x
 *    and 1.1, only the first menu item is returned. In CarbonLib 1.2
 *    and Mac OS X 10.0 and later, this API iterates over all menu
 *    items with the specified command ID. In Mac OS X 10.0 and
 *    CarbonLib 1.0 through 1.4, this API only searches top-level menus
 *    (menus visible in the menubar) and submenus of top-level menus.
 *    It does not search hierarchical menus that are inserted in the
 *    menubar but are not submenus of a top-level menus (for example,
 *    it does not search menus that are inserted for use in a popup
 *    menu control). In Mac OS X 10.1 and CarbonLib 1.5 and later, this
 *    API also searches inserted hierarchical menus.
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
<<<<<<< HEAD
procedure DisposeMCInfo( menuCTbl: MCTableHandle ); external name '_DisposeMCInfo';
=======
function GetMCEntry( menuID_: MenuID; menuItem: SInt16 ): MCEntryPtr; external name '_GetMCEntry';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetMCEntry()   *** DEPRECATED ***
=======
 *  SetMCEntries()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Returns a pointer to the menu color table entry for a menu item.
=======
 *    Sets entries in the menu color table for the current process.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for items with the
 *      specified command ID. Pass NULL to hegin searching with the
 *      root menu. The search will descend into all submenus of this
 *      menu.
 *    
 *    inCommandID:
 *      The command ID for which to search.
 *    
 *    inItemIndex:
 *      The 1-based index of the menu item to retrieve. In CarbonLib
 *      1.0.x and 1.1, this parameter must be 1. In CarbonLib 1.2 and
 *      Mac OS X 10.0, this parameter may vary from 1 to the number of
 *      menu items with the specified command ID.
 *    
 *    outMenu:
 *      On exit, the menu containing the menu item with the specified
 *      command ID.
 *    
 *    outIndex:
 *      On exit, the item index of the menu item with the specified
 *      command ID.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetIndMenuItemWithCommandID( inMenu: MenuRef; inCommandID: MenuCommand; inItemIndex: UInt32; outMenu: MenuRefPtr { can be NULL }; outIndex: MenuItemIndexPtr { can be NULL } ): OSStatus; external name '_GetIndMenuItemWithCommandID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  EnableMenuCommand()
 *  
 *  Summary:
 *    Enables the menu item with a specified command ID.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function GetMCEntry( menuID_: MenuID; menuItem: SInt16 ): MCEntryPtr; external name '_GetMCEntry';
=======
procedure SetMCEntries( numEntries: SInt16; menuCEntries: MCTablePtr ); external name '_SetMCEntries';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • MDEF support                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{
<<<<<<< HEAD
 *  SetMCEntries()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Sets entries in the menu color table for the current process.
=======
 *  EraseMenuBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use a custom HIView instead of a menu
 *    definition function to customize the appearance of a menu.
 *  
 *  Summary:
 *    Erases a portion of a menu background in preparation for further
 *    drawing.
 *  
 *  Discussion:
 *    It is necessary to erase the menu background before calling
 *    DrawThemeMenuBackground because some themes (such as Aqua on Mac
 *    OS X) draw the menu background using the alpha channel, and if
 *    the area underneath the menu background is not erased, portions
 *    of the old image will show through the menu background.
>>>>>>> graemeg/cpstrnew
=======
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to be enabled. If more than one
 *      item has this command ID, only the first will be enabled.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure EnableMenuCommand( inMenu: MenuRef; inCommandID: MenuCommand ); external name '_EnableMenuCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisableMenuCommand()
 *  
 *  Summary:
 *    Disables the menu item with a specified command ID.
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
 *    inMenu:
<<<<<<< HEAD
<<<<<<< HEAD
 *      The menu whose background to erase.
 *    
 *    inEraseRect:
 *      The bounds of the area to erase, in local coordinates to the
 *      current port.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, EraseMenuBackground will
 *      create a context based on the current port.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMCEntries( numEntries: SInt16; menuCEntries: MCTablePtr ); external name '_SetMCEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • MDEF support                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  EraseMenuBackground()   *** DEPRECATED ***
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function EraseMenuBackground( inMenu: MenuRef; const (*var*) inEraseRect: Rect; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_EraseMenuBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ScrollMenuImage()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use a custom HIView instead of a menu
 *    definition function to customize the appearance of a menu.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Erases a portion of a menu background in preparation for further
 *    drawing.
 *  
 *  Discussion:
 *    It is necessary to erase the menu background before calling
 *    DrawThemeMenuBackground because some themes (such as Aqua on Mac
 *    OS X) draw the menu background using the alpha channel, and if
 *    the area underneath the menu background is not erased, portions
 *    of the old image will show through the menu background.
=======
 *    Scrolls a portion of the menu image.
 *  
 *  Discussion:
 *    Menus on Mac OS X use an alpha channel, and QuickDraw does not
 *    support alpha channels. Therefore, scrolling a menu image with
 *    ScrollRect or other QuickDraw APIs does not work correctly; it
 *    results in the destruction of the alpha channel data. The
 *    ScrollMenuImage API uses CoreGraphics to move the menu image,
 *    preserving the alpha channel.
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to be disabled. If more than
 *      one item has this command ID, only the first will be disabled.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisableMenuCommand( inMenu: MenuRef; inCommandID: MenuCommand ); external name '_DisableMenuCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsMenuCommandEnabled()
 *  
 *  Summary:
 *    Determines if the menu item with a specified command ID is
 *    enabled.
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
 *    inMenu:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      The menu whose background to erase.
 *    
 *    inEraseRect:
 *      The bounds of the area to erase, in local coordinates to the
 *      current port.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, EraseMenuBackground will
 *      create a context based on the current port.
=======
 *      The menu whose image to scroll.
 *    
 *    inScrollRect:
 *      The bounds of the rect to scroll.
 *    
 *    inHScroll:
 *      The distance to scroll horizontally.
 *    
 *    inVScroll:
 *      The distance to scroll vertically.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, ScrollMenuImage will create a
 *      context based on the current port.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function EraseMenuBackground( inMenu: MenuRef; const (*var*) inEraseRect: Rect; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_EraseMenuBackground';
=======
function ScrollMenuImage( inMenu: MenuRef; const (*var*) inScrollRect: Rect; inHScroll: SInt32; inVScroll: SInt32; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_ScrollMenuImage';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure AppendResMenu( theMenu: MenuRef; theType: ResType ); external name '_AppendResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertFontResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateStandardFontMenu or a font panel window instead of
 *    InsertFontResMenu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of font resources.
=======
=======
>>>>>>> origin/fixes_2_2
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to examine. If more than one
 *      item has this command ID, only the first will be examined.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuCommandEnabled( inMenu: MenuRef; inCommandID: MenuCommand ): Boolean; external name '_IsMenuCommandEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuCommandMark()
 *  
 *  Summary:
 *    Locates the menu item with a specified command ID and sets its
 *    mark character.
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
procedure InsertFontResMenu( theMenu: MenuRef; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertFontResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}

type
	MenuBarDefProcPtr = function( selector: SInt16; message: SInt16; parameter1: SInt16; parameter2: SInt32 ): SInt32;
	MenuHookProcPtr = procedure;
	MBarHookProcPtr = function( var menuRect: Rect ): SInt16;
	MenuBarDefUPP = MenuBarDefProcPtr;
	MenuHookUPP = MenuHookProcPtr;
	MBarHookUPP = MBarHookProcPtr;
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  ScrollMenuImage()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use a custom HIView instead of a menu
 *    definition function to customize the appearance of a menu.
 *  
 *  Summary:
 *    Scrolls a portion of the menu image.
 *  
 *  Discussion:
 *    Menus on Mac OS X use an alpha channel, and QuickDraw does not
 *    support alpha channels. Therefore, scrolling a menu image with
 *    ScrollRect or other QuickDraw APIs does not work correctly; it
 *    results in the destruction of the alpha channel data. The
 *    ScrollMenuImage API uses CoreGraphics to move the menu image,
 *    preserving the alpha channel.
=======
 *  InsertIntlResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to be modified. If more than
 *      one item has this command ID, only the first will be modified.
 *    
 *    inMark:
 *      The new mark character. Although the type of this parameter is
 *      UniChar, currently only the low byte of this character will be
 *      used as the mark character, and it is interpreted using the
 *      application’s text encoding.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuCommandMark( inMenu: MenuRef; inCommandID: MenuCommand; inMark: UniChar ): OSStatus; external name '_SetMenuCommandMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuCommandMark()
 *  
 *  Summary:
 *    Locates the menu item with a specified command ID and returns its
 *    mark character.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item to be examined. If more than
 *      one item has this command ID, only the first will be examined.
 *    
 *    outMark:
 *      On exit, the menu item's mark character.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuCommandMark( inMenu: MenuRef; inCommandID: MenuCommand; var outMark: UniChar ): OSStatus; external name '_GetMenuCommandMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuCommandProperty()
 *  
 *  Summary:
 *    Retrives property data for a menu item with a specified command
 *    ID.
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
procedure InsertIntlResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertIntlResMenu';
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RetainMenu( inMenu: MenuRef ): OSStatus; external name '_RetainMenu';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  MenuKey()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use MenuEvent, IsMenuKeyEvent, or a kEventCommandProcess Carbon
 *    event handler instead of MenuKey.
 *  
 *  Summary:
 *    Determines if a character matches a menu item command key.
>>>>>>> graemeg/cpstrnew
=======
 *  ReleaseMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRelease instead of ReleaseMenu.
 *  
 *  Summary:
 *    Decrements the retain count of a menu.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *  NewMenuBarDefUPP()
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item containing the property. If
 *      more than one item has this command ID, only the first will be
 *      used.
 *    
 *    inPropertyCreator:
 *      The property creator.
 *    
 *    inPropertyTag:
 *      The property tag.
 *    
 *    inBufferSize:
 *      The size of the output buffer, in bytes.
 *    
 *    outActualSize:
 *      On exit, contains the actual size of the property data. May be
 *      NULL if you do not need this information.
 *    
 *    inPropertyBuffer:
 *      The address of a buffer in which to place the property data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuCommandProperty( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType; inBufferSize: ByteCount; outActualSize: ByteCountPtr { can be NULL }; inPropertyBuffer: UnivPtr ): OSStatus; external name '_GetMenuCommandProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetMenuCommandPropertySize()
 *  
 *  Summary:
 *    Retrives the size of property data for a menu item with a
 *    specified command ID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Parameters:
 *    
 *    inMenu:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      The menu whose image to scroll.
 *    
 *    inScrollRect:
 *      The bounds of the rect to scroll.
 *    
 *    inHScroll:
 *      The distance to scroll horizontally.
 *    
 *    inVScroll:
 *      The distance to scroll vertically.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, ScrollMenuImage will create a
 *      context based on the current port.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function ScrollMenuImage( inMenu: MenuRef; const (*var*) inScrollRect: Rect; inHScroll: SInt32; inVScroll: SInt32; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_ScrollMenuImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
<<<<<<< HEAD
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MenuKey( ch: CharParameter ): SInt32; external name '_MenuKey';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
 *      The menu whose retain count to decrement. If the retain count
 *      falls to zero, the menu is destroyed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function ReleaseMenu( inMenu: MenuRef ): OSStatus; external name '_ReleaseMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}
=======
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }
>>>>>>> graemeg/cpstrnew
=======
{
<<<<<<< HEAD
 *  SetMenuFlashCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function has no effect on Mac OS X.
 *  
 *  Summary:
 *    Sets the number of times that a selected menu item will blink.
=======
 *  InsertResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item containing the property. If
 *      more than one item has this command ID, only the first will be
 *      used.
 *    
 *    inPropertyCreator:
 *      The property creator.
 *    
 *    inPropertyTag:
 *      The property tag.
 *    
 *    outSize:
 *      On exit, contains the size of the property data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuCommandPropertySize( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType; var outSize: ByteCount ): OSStatus; external name '_GetMenuCommandPropertySize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuCommandProperty()
 *  
 *  Summary:
 *    Sets property data for a menu item with a specified command ID.
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
<<<<<<< HEAD
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
procedure SetMenuFlashCount( count: SInt16 ); external name '_SetMenuFlashCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
type
	MenuBarDefProcPtr = function( selector: SInt16; message: SInt16; parameter1: SInt16; parameter2: SIGNEDLONG ): SIGNEDLONG;
=======
{$endc} {not TARGET_CPU_64}

type
	MenuBarDefProcPtr = function( selector: SInt16; message: SInt16; parameter1: SInt16; parameter2: SInt32 ): SInt32;
>>>>>>> graemeg/cpstrnew
	MenuHookProcPtr = procedure;
	MBarHookProcPtr = function( var menuRect: Rect ): SInt16;
	MenuBarDefUPP = MenuBarDefProcPtr;
	MenuHookUPP = MenuHookProcPtr;
	MBarHookUPP = MBarHookProcPtr;
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  NewMenuBarDefUPP()
=======
 *  NewMenuHookUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
=======
 *  InitContextualMenus()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X, it is not necessary to call InitContextualMenus.
 *  
 *  Summary:
 *    Adds the current process to the system registry of contextual
 *    menu clients.
 *  
 *  Discussion:
 *    On Mac OS 8.x and 9.x, your program should call the
 *    InitContextualMenus function early in your startup code to
 *    register your application as a contextual menu client. If you do
 *    not register your program, some system-level functions may
 *    respond as though your program does not use contextual menus. Not
 *    registering your program may also cause
 *    ProcessIsContextualMenuClient to return an incorrect value.
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex ); external name '_InsertResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AppendResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Appends menu items containing the names of resources of a
 *    specified resource type.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item that will receive the property.
 *      If more than one item has this command ID, only the first will
 *      be modified.
 *    
 *    inPropertyCreator:
 *      The property creator.
 *    
 *    inPropertyTag:
 *      The property tag.
 *    
 *    inPropertySize:
 *      The size of the property data, in bytes.
 *    
 *    inPropertyData:
 *      The address of the property data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuCommandProperty( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType; inPropertySize: ByteCount; inPropertyData: {const} UnivPtr ): OSStatus; external name '_SetMenuCommandProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveMenuCommandProperty()
 *  
 *  Summary:
 *    Removes a property from a menu item with a specified command ID.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu in which to begin searching for the item. Pass NULL to
 *      begin searching with the root menu. The search will descend
 *      into all submenus of this menu.
 *    
 *    inCommandID:
 *      The command ID of the menu item from which the property will be
 *      removed. If more than one item has this command ID, only the
 *      first will be modified.
 *    
 *    inPropertyCreator:
 *      The property creator.
 *    
 *    inPropertyTag:
 *      The property tag.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function RemoveMenuCommandProperty( inMenu: MenuRef; inCommandID: MenuCommand; inPropertyCreator: OSType; inPropertyTag: OSType ): OSStatus; external name '_RemoveMenuCommandProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CopyMenuItemData()
 *  
 *  Summary:
 *    Returns multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to retrieve many attributes of a menu item
 *    simultaneously; for example, it might be used by a menu
 *    definition function that needs to know how to draw a menu item.
 *    It is more efficient to use this function than to use the
 *    accessor functions for the individual attributes of the menu.
 *    This function returns a copy of the data in the menu, so any data
 *    in the MenuItemDataRec that is dynamically allocated (for
 *    example, the CFString item text) should be released by the caller.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu from which to copy data. If inIsCommandID is true, you
 *      may pass NULL for this parameter to search for an item in the
 *      root menu; if inIsCommandID is false, this parameter must be a
 *      valid MenuRef.
 *    
 *    inItem:
 *      The item or command ID from which to copy data.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and data is copied from the first item in the
 *      menu with that command ID. If inIsCommandID is false, the
 *      inItem parameter is interpreted as a menu item index, and data
 *      is copied for that item in the specified menu.
 *    
 *    ioData:
 *      Data is copied from the item and placed here. On entry, the
 *      whichData field of this structure should be initialized to
 *      indicate which data the caller would like returned. Individual
 *      fields of the MenuItemDataRec structure may require
 *      pre-initialization also; see the individual MenuItemDataFlags
 *      documentation for details.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CopyMenuItemData( inMenu: MenuRef { can be NULL }; inItem: MenuItemID; inIsCommandID: Boolean; ioData: MenuItemDataPtr ): OSStatus; external name '_CopyMenuItemData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetMenuItemData()
 *  
 *  Summary:
 *    Sets multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to set many attributes of a menu item
 *    simultaneously. It is more efficient to use this function than to
 *    use the accessor functions for the individual attributes of the
 *    menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to modify.
 *    
 *    inItem:
 *      The item or command ID to modify.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and the first item in the menu with that
 *      command ID. is modified. If inIsCommandID is false, the inItem
 *      parameter is interpreted as a menu item index, and the item
 *      with that index in the specified menu is modified.
 *    
 *    inData:
 *      The data to set. The caller should set the whichData field of
 *      this structure to indicate which data should be set. Only the
 *      fields of the structure corresponding to the non-zero whichData
 *      flags must be initialized; other fields are ignored.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetMenuItemData( inMenu: MenuRef; inItem: MenuItemID; inIsCommandID: Boolean; const (*var*) inData: MenuItemDataRec ): OSStatus; external name '_SetMenuItemData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Dynamic menu item support (available in CarbonLib 1.1 and Mac OS X)               }
{  Dynamic menu item support allows a menu item to be redrawn while the menu is open   }
{  and visible to the user. Carbon contains automatic support for dynamic items based  }
{  on keyboard modifier state. If you need to implement your own variable item state   }
{  based on other system state, you can use these APIs to implement it.                }
{  To use the built-in support for dynamic items, you should create a menu containing  }
{  several contiguous items with the same command key but different text and modifier  }
{  keys. For example, you might have:                                                  }
{      Close       cmd-W                                                               }
{      Close All   cmd-option-W                                                        }
{  The Menu Manager automatically determines a dynamic menu group using the base       }
{  command key of a dynamic menu item ('W' in this case). Only a single item from      }
{  a dynamic group is visible at any time; the other items are hidden. The Menu        }
{  Manager uses the current keyboard modifiers to determine which item is visible.     }
{  It is also possible to have a dynamic menu item group that does not have command    }
{  keys, but only modifiers; for example, in the Finder's View menu, the Clean Up      }
{  and Clean Up Selection items have no command key, but are a dynamic menu item       }
{  group that changes visibility based on the Option modifier key.                     }
{  In this example, in your MENU resource, you would create the Close and Close All    }
{  items and give them each the letter 'W' as the command key; using an associated     }
{  xmnu resource, you would specify kMenuOptionModifier as the modifier for the        }
{  Close All item. You can do the same thing using InterfacerBuilder and nib-based     }
{  menus.                                                                              }
{  After loading your menu from the resource, you must set the kMenuItemAttrDynamic    }
{  flag for each dynamic item. In this example, you would use:                         }
{      ChangeMenuItemAttributes( menu, kCloseItem, kMenuItemAttrDynamic, 0 );          }
{      ChangeMenuItemAttributes( menu, kCloseAllItem, kMenuItemAttrDynamic, 0 );       }
{  The Menu Manager will now automatically display the correct item depending on       }
{  whether the Option key is pressed. The result from MenuSelect will be the item      }
{  number of the item that was visible when the menu closed.                           }
{  Note that:                                                                          }
{      - If you use InterfacerBuilder, you can set the kMenuItemAttrDynamic attribute  }
{        directly in the menu object in the nib, using the Inspector window for the    }
{        menu. You don't need to add the attribute after creating the menu.            }
{      - If your application requires Mac OS X 10.2 or CarbonLib 1.6 or later, you     }
{        can also use a version-one-format 'xmnu' resource, which allows settting      }
{        the menu item attributes in the 'xmnu'. Using a version one resource, you     }
{        can include the kMenuItemAttrDynamic attribute in the resource rather than    }
{        adding it after creating the menu.                                            }
{  If the Menu Manager's built-in support is not sufficient, you can also change the   }
{  attributes of an item yourself and use the UpdateInvalidMenuItems API to cause      }
{  the menu to redraw. Changes to a menu item (changing text, command key, style,      }
{  etc.) that occur while the menu is open will cause the menu item to be invalidated, }
{  but not redrawn. If you need to invalidate the item explicitly yourself, perhaps    }
{  because you have a custom MDEF that depends on state not accessed using Menu        }
{  Manager APIs, you can use the InvalidateMenuItems API. UpdateInvalidMenuItems will  }
{  scan the menu for invalid items and redraw each, clearing its invalid flag          }
{  afterwards.                                                                         }
{  If you need to change menu contents based on modifier key state without using the   }
{  built-in support in the Menu Manager, we recommend that you install a Carbon event  }
{  handler on your menu for the [kEventClassKeyboard, kEventRawKeyModifiersChanged]    }
{  event. Modifier key events are passed to the currently open menu before being sent  }
{  to the user focus target.                                                           }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  IsMenuItemInvalid()
 *  
 *  Summary:
 *    Determines if a menu item is invalid and should be redrawn.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose item to examine.
 *    
 *    inItem:
 *      The item to examine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsMenuItemInvalid( inMenu: MenuRef; inItem: MenuItemIndex ): Boolean; external name '_IsMenuItemInvalid';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  InvalidateMenuItems()
 *  
 *  Summary:
 *    Invalidates a group of menu items so that they will be redrawn
 *    when UpdateInvalidMenuItems is next called.
 *  
 *  Discussion:
 *    Menu items are automatically invalidated when their contents are
 *    changed using Menu Manager APIs while the menu is open. However,
 *    you might need to use this API if you have a custom MDEF that
 *    draws using state not contained in the menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose items to invalidate.
 *    
 *    inFirstItem:
 *      The first item to invalidate.
 *    
 *    inNumItems:
 *      The number of items to invalidate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function InvalidateMenuItems( inMenu: MenuRef; inFirstItem: MenuItemIndex; inNumItems: ItemCount ): OSStatus; external name '_InvalidateMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateInvalidMenuItems()
 *  
 *  Summary:
 *    Redraws the invalid items of an open menu.
 *  
 *  Discussion:
 *    It is not necessary to use UpdateInvalidMenuItems if you are
 *    using Carbon's built-in support for dynamic items based on
 *    modifier key state. However, if you are modifying items
 *    dynamically using your own implementation, you should call
 *    UpdateInvalidMenuItems after completing your modifications for a
 *    single menu. It will redraw any items that have been marked as
 *    invalid, and clear the invalid flag for those items.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu to update.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateInvalidMenuItems( inMenu: MenuRef ): OSStatus; external name '_UpdateInvalidMenuItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Standard font menu (Carbon and later)                                             }
{  These APIs allow you to create and use the standard font menu.                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

const
	kHierarchicalFontMenuOption = $00000001;

{$ifc not TARGET_CPU_64}
{
 *  CreateStandardFontMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateStandardFontMenu( menu: MenuRef; afterItem: MenuItemIndex; firstHierMenuID: MenuID; options: OptionBits; outHierMenuCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_CreateStandardFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UpdateStandardFontMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function UpdateStandardFontMenu( menu: MenuRef; outHierMenuCount: ItemCountPtr { can be NULL } ): OSStatus; external name '_UpdateStandardFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetFontFamilyFromMenuSelection()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function GetFontFamilyFromMenuSelection( menu: MenuRef; item: MenuItemIndex; var outFontFamily: FMFontFamily; var outStyle: FMFontStyle ): OSStatus; external name '_GetFontFamilyFromMenuSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Contextual Menu routines and constants                                            }
{  available with Conxtextual Menu extension 1.0 and later                             }
{——————————————————————————————————————————————————————————————————————————————————————}
{ Gestalt Selector for classic 68K apps only. }
{ CFM apps should weak link and check the symbols. }
{$endc} {not TARGET_CPU_64}

const
	gestaltContextualMenuAttr = FourCharCode('cmnu');
	gestaltContextualMenuUnusedBit = 0;
	gestaltContextualMenuTrapAvailable = 1;
	gestaltContextualMenuHasAttributeAndModifierKeys = 2; { Contextual Menu Manager supports keyContextualMenuAttributes and keyContextualMenuModifiers }
	gestaltContextualMenuHasUnicodeSupport = 3; { Contextual Menu Manager supports typeUnicodeText and typeCFStringRef }


{
 *  Summary:
 *    Values indicating what kind of help the application supports
 }
const
{
   * The application does not support any help. The Menu Manager will
   * put an appropriate help string into the contextual menu and
   * disable the Help item.
   }
	kCMHelpItemNoHelp = 0;

  {
   * The application supports Apple Guide help. The Menu Manager will
   * put the name of the main Guide file into the contextual menu and
   * enable the Help item.
   }
	kCMHelpItemAppleGuide = 1;

  {
   * The application supports some other form of help. In this case,
   * the application must also pass a valid string into the
   * inHelpItemString parameter of ContextualMenuSelect. This string
   * will be the text of the Help item in the contextual menu, and the
   * Help item will be enabled.
   }
	kCMHelpItemOtherHelp = 2;

  {
   * The application does not support any help. The Menu Manager will
   * remove the Help item from the contextual menu. This constant is
   * available in Mac OS X and CarbonLib 1.6, and later; however, in
   * CarbonLib it is translated to kCMHelpItemNoHelp, and the Help item
   * is only disabled, not removed.
   }
	kCMHelpItemRemoveHelp = 3;


{
 *  Summary:
 *    Values indicating what was chosen from a contextual menu
 }
const
{
   * The user did not choose an item from the contextual menu and the
   * application should do no further processing of the event.
   }
	kCMNothingSelected = 0;

  {
   * The user chose one of the application's items from the menu. The
   * application can examine the outMenuID and outMenuItem parameters
   * of ContextualMenuSelect to see what the menu selection was, and it
   * should then handle the selection appropriately.
   }
	kCMMenuItemSelected = 1;

  {
   * The user chose the Help item from the menu. The application should
   * open an Apple Guide database to a section appropriate for the
   * selection. If the application supports some other form of help, it
   * should be presented instead.
   }
	kCMShowHelpSelected = 3;


{
 *  Summary:
 *    AERecord keywords used by the ExamineContext method of a
 *    Contextual Menu plugin to specify menu item contents.
 }
const
{
   * Specifies the text of an item in a contextual menu. Data for this
   * parameter can be in one of several formats. In Mac OS 7/8/9.x and
   * Mac OS X 10.0 and 10.1, typeChar and typeIntlText are supported.
   * In Mac OS X 10.2 and later, typeStyledText, typeAEText,
   * typeUnicodeText, and typeCFStringRef are also supported. If you
   * provide data as typeCFStringRef, the Contextual Menu Manager will
   * automatically release the CFStringRef once the menu has been
   * displayed. If you need the CFStringRef to have a longer timetime,
   * your plugin should retain the CFStringRef before inserting it into
   * the AERecord.
   }
	keyContextualMenuName = FourCharCode('pnam');

  {
   * Specifies the command ID of an item in a contextual menu. Data for
   * this parameter should be typeSInt32.
   }
	keyContextualMenuCommandID = FourCharCode('cmcd');

  {
   * Specifies a contextual menu item with a submenu. Typically used
   * with AEPutKeyDesc to add an entire AEDesc containing the submenu
   * as the data for the parameter.
   }
	keyContextualMenuSubmenu = FourCharCode('cmsb');

  {
   * Specifies the menu item attributes of an item in a contextual
   * menu. Data for this parameter should be typeSInt32. Available in
   * Mac OS X 10.2 and later.
   }
	keyContextualMenuAttributes = FourCharCode('cmat');

  {
   * Specifies the modifier keys of an item in a contextual menu (see
   * kMenuShiftModifier, kMenuControlModifier, etc.) Data for this
   * parameter should be typeSInt32. Using this parameter together with
   * the keyContextualMenuAttributes parameter, it is possible to
   * create a contextual menu with dynamic items which change according
   * to the modifier keys pressed by the user. Available in Mac OS X
   * 10.2 and later.
   }
	keyContextualMenuModifiers = FourCharCode('cmmd');

{$ifc not TARGET_CPU_64}
{
 *  IsShowContextualMenuClick()
 *  
 *  Summary:
 *    Determines whether a particular EventRecord could invoke a
 *    contextual menu.
 *  
 *  Discussion:
 *    Applications should call IsShowContextualMenuClick when they
 *    receive non-null events. If IsShowContextualMenuClick returns
 *    true, your application should generate its own menu and Apple
 *    Event descriptor (AEDesc), and then call ContextualMenuSelect to
 *    display and track the contextual menu, and then handle the user's
 *    choice. 
 *    
 *    Apple recommends that you use the IsShowContextualMenuEvent API
 *    instead of this API. If you use the standard window Carbon event
 *    handler on your windows, you can also use a
 *    kEventControlContextualMenuClick event handler to be notified
 *    when a contextual menu click has occured; in this case, you do
 *    not need to call this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine.
 *  
 *  Result:
 *    Returns true if the application should display a contextual menu,
 *    false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function IsShowContextualMenuClick( const (*var*) inEvent: EventRecord ): Boolean; external name '_IsShowContextualMenuClick';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IsShowContextualMenuEvent()
 *  
 *  Summary:
 *    Determines whether a particular EventRef could invoke a
 *    contextual menu.
 *  
 *  Discussion:
 *    This API is similar to IsShowContextualMenuClick, but takes a
 *    Carbon EventRef as its parameter instead of an EventRecord.
 *    
 *    
 *    In Mac OS X 10.3 and earlier, this API always returned false if
 *    the event kind was not kEventMouseDown,
 *    kEventWindowClickContentRgn, kEventWindowClickStructureRgn, or
 *    kEventWindowHandleContentClick. In Mac OS X 10.4 and later, this
 *    API no longer requires a specific event kind; it only requires
 *    that the event contain kEventParamMouseButton and
 *    kEventParamKeyModifiers parameters. 
 *    
 *    If you use the standard window Carbon event handler on your
 *    windows, you can also use a kEventControlContextualMenuClick
 *    event handler to be notified when a contextual menu click has
 *    occured; in this case, you do not need to call this API.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inEvent:
 *      The event to examine.
 *  
 *  Result:
 *    Returns true if the application should display a contextual menu,
 *    false if not.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function IsShowContextualMenuEvent( inEvent: EventRef ): Boolean; external name '_IsShowContextualMenuEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ContextualMenuSelect()
 *  
 *  Summary:
 *    Displays a contextual menu.
 *  
 *  Discussion:
 *    If IsShowContextualMenuClick returns true, you should call the
 *    ContextualMenuSelect API after generating your own menu and
 *    preparing an Apple Event descriptor (AEDesc) that describes the
 *    item for which your application is displaying a contextual menu.
 *    This descriptor may contain an object specifier or raw data and
 *    will be passed to all contextual menu plug-ins. 
 *    
 *    The system will add other items before displaying the contextual
 *    menu, and it will remove those items before returning, leaving
 *    the menu in its original state. 
 *    
 *    After all the system commands are added, the contextual menu is
 *    displayed and tracked. If the user selects one of the system
 *    items, it is handled by the system and the call returns as though
 *    the user didn't select anything from the menu. If the user
 *    selects any other item (or no item at all), the Menu Manager
 *    passes back appropriate values in the parameters
 *    outUserSelectionType, outMenuID, and outMenuItem. 
 *    
 *    Your application should provide visual feedback indicating the
 *    item that was clicked upon. For example, a click on an icon
 *    should highlight the icon, while a click on editable text should
 *    not eliminate the current selection. 
 *    
 *    If the outUserSelectionType parameter contains
 *    kCMMenuItemSelected, you should look at the outMenuID and
 *    outMenuItem parameters to determine what menu item the user chose
 *    and handle it appropriately. If the user selected
 *    kCMHelpItemSelected, you should open the proper Apple Guide
 *    sequence or other form of custom help. 
 *    
 *    There is a bug in ContextualMenuSelect in versions of Mac OS X
 *    prior to Mac OS X 10.4 that causes the value returned in the
 *    outUserSelectionType parameter to be set to kCMShowHelpSelected
 *    instead of kCMMenuItemSelected if the menu has a menu ID of zero
 *    and item three is selected. To avoid this problem on all versions
 *    of Mac OS X, simply use a non-zero menu ID. Note that when a new
 *    menu is created in InterfaceBuilder, its ID is automatically set
 *    to zero.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      A menu containing application commands to display. The caller
 *      creates this menu based on the current context, the mouse
 *      location, and the current selection (if it was the target of
 *      the mouse). If you pass nil, only system commands will be
 *      displayed. The menu should be added to the menu list as a
 *      pop-up menu (using the InsertMenu function).
 *    
 *    inGlobalLocation:
 *      The location (in global coordinates) of the mouse near which
 *      the menu is to be displayed.
 *    
 *    inReserved:
 *      Reserved for future use. Pass false for this parameter.
 *    
 *    inHelpType:
 *      An identifier specifying the type of help to be provided by the
 *      application; see kCMHelpItem constants.
 *    
 *    inHelpItemString:
 *      A string containing the text to be displayed for the help menu
 *      item. This string is unused unless you also pass the constant
 *      kCMHelpItemOtherHelp in the inHelpType parameter.
 *    
 *    inSelection:
 *      An object specifier for the current selection. This allows he
 *      system to examine the selection and add special system commands
 *      accordingly. Passing a value of nil indicates that no selection
 *      should be examined, and most likely, no special system actions
 *      will be included.
 *    
 *    outUserSelectionType:
 *      On exit, the value indicates what the user selected from the
 *      contextual menu; see kCMNothingSelected, kCMMenuItemSelected,
 *      and kCMShowHelpSelected.
 *    
 *    outMenuID:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu ID of the chosen item.
 *    
 *    outMenuItem:
 *      On exit, if outUserSelectionType is set to kCMMenuItemSelected,
 *      the value is set to the menu item chosen.
 *  
 *  Result:
 *    An OSStatus result code. ContextualMenuSelect returns the result
 *    code userCanceledErr and sets outUserSelectionType to
 *    kCMNothingSelected to indicate that the user did not select
 *    anything from the contextual menu and no further processing is
 *    needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
 }
function ContextualMenuSelect( inMenu: MenuRef; inGlobalLocation: Point; inReserved: Boolean; inHelpType: UInt32; {const} inHelpItemString:ConstStringPtr { can be NULL }; {const} inSelection: AEDescPtr { can be NULL }; var outUserSelectionType: UInt32; var outMenuID: MenuID; var outMenuItem: MenuItemIndex ): OSStatus; external name '_ContextualMenuSelect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {NOT TARGET_CPU_64}

{——————————————————————————————————————————————————————————————————————————————————————}
{  Contextual Menu Plugin Interface                                                    }
{  For Mac OS X 10.1, we support a new type of Contextual Menu Plugin: the CFPlugIn    }
{  based plugin.  Each plugin must be in a CFPlugIn in the Contextual Menu Items       }
{  folder in one of these paths:                                                       }
{      /System/Library/Contextual Menu Items/                                          }
{      /Library/Contextual Menu Items/                                                 }
{      ~/Library/Contextual Menu Items/                                                }
{  It must export the following functions using the following interface or a C++       }
{  interface inheriting from IUnknown and including similar functions.                 }
{——————————————————————————————————————————————————————————————————————————————————————}
(*
{ The Contextual Menu Manager will only load CFPlugIns of type kContextualMenuTypeID }
#define kContextualMenuTypeID ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x2F, 0x65, 0x22, 0xE9, 0x3E, 0x66, 0x11, 0xD5, \
  0x80, 0xA7, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 2F6522E9-3E66-11D5-80A7-003065B300BC }

{ Contextual Menu Plugins must implement this Contexual Menu Plugin Interface }
#define kContextualMenuInterfaceID    ( CFUUIDGetConstantUUIDWithBytes( NULL, \
  0x32, 0x99, 0x7B, 0x62, 0x3E, 0x66, 0x11, 0xD5, \
  0xBE, 0xAB, 0x00, 0x30, 0x65, 0xB3, 0x00, 0xBC ) )
 { 32997B62-3E66-11D5-BEAB-003065B300BC }

#define CM_IUNKNOWN_C_GUTS \
   void *_reserved; \
 SInt32 (*QueryInterface)(void *thisPointer, CFUUIDBytes iid, void ** ppv); \
   UInt32 (*AddRef)(void *thisPointer); \
 UInt32 (*Release)(void *thisPointer)

{ The function table for the interface }
type
	ContextualMenuInterfaceStruct = record
CM_IUNKNOWN_C_GUTS;
    OSStatus ( *ExamineContext )(
          void*               thisInstance,
          const AEDesc*       inContext,
         AEDescList*         outCommandPairs );
 OSStatus ( *HandleSelection )(
         void*               thisInstance,
          AEDesc*             inContext,
         SInt32              inCommandID );
 void ( *PostMenuCleanup )(
         void*               thisInstance );
	end;
*)
{
 *  CMPluginExamineContext()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginExamineContext( thisInstance: UnivPtr; const (*var*) inContext: AEDesc; var outCommandPairs: AEDescList ): OSStatus; external name '_CMPluginExamineContext';


{
 *  CMPluginHandleSelection()
 *  
 *  Availability:
 *    Implemented by client
 }
function CMPluginHandleSelection( thisInstance: UnivPtr; var inContext: AEDesc; inCommandID: SInt32 ): OSStatus; external name '_CMPluginHandleSelection';


{
 *  CMPluginPostMenuCleanup()
 *  
 *  Availability:
 *    Implemented by client
 }
procedure CMPluginPostMenuCleanup( thisInstance: UnivPtr ); external name '_CMPluginPostMenuCleanup';


{ previously in LowMem.h.  This functions return the menu ID of the hilited menu }
{$ifc not TARGET_CPU_64}
{
 *  LMGetTheMenu()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetTheMenu: MenuID; external name '_LMGetTheMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  • DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
    A Short Course on Menu Definition Functions
    
    A menu definition function is used to implement a custom visual appearance for a menu.
    Menu definition functions are still supported in Carbon, but the messages sent to a
    menu definition function in Carbon are different than for a non-Carbon application.
    
    In general, Apple recommends using the system menu definition whenever possible.
    Menu definition functions will continue to be supported, but it is not easy to write
    a correct menu definition, especially one that attempts to imitate the standard system
    menu appearance. If you require special features in your menu that the system menu
    definition does not support, please mail <toolbox@apple.com> and describe your requirements;
    we would much rather enhance the system menu definition than have you write a custom one.
    The use of menu definition functions is deprecated in Mac OS X; in Mac OS X 10.3 and
    later, Apple recommends that you use a custom HIView to draw menu content.
    
    Menu definition functions before Carbon used the following messages:
    
        kMenuDrawMsg
        kMenuChooseMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuDrawItemMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    kMenuChooseMsg and kMenuDrawItemMsg are not supported in Carbon and are not sent to
    Carbon menu definitions. In Carbon, kMenuChooseMsg is replaced by kMenuFindItemMsg and
    kMenuHiliteItemMsg. Menu definition functions in Carbon use the following messages:
    
        kMenuInitMsg
        kMenuDisposeMsg
        kMenuFindItemMsg
        kMenuHiliteItemMsg
        kMenuDrawItemsMsg
        kMenuDrawMsg
        kMenuSizeMsg
        kMenuPopUpMsg
        kMenuCalcItemMsg
        kMenuThemeSavvyMsg
        
    The rest of this documentation will focus on Carbon menu definitions only.
    
    Menu Definition Messages
    
        Carbon menu definition functions should support the following messages:
        
        kMenuInitMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       OSErr*
        
            Sent when a menu is created. This message gives the menu definition an opportunity
            to initialize its own state. If the menu definition encounters an error while
            initializing, it should set *whichItem to a non-zero error code; this will cause the
            Menu Manager to destroy the menu and return an error back from the API that was used
            to create the menu.
        
        kMenuDisposeMsg
            
            menuRect        unused
            hitPt           unused
            whichItem       unused
            
            Sent when a menu is destroyed. This message gives the menu definition an opportunity
            to destroy its own data.
            
        kMenuFindItemMsg
            
            menuRect        menu bounds
            hitPt           point to hit-test
            whichItem       MDEFFindItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to know what item is under
            the mouse. The whichItem parameter is actually a pointer to a MenuTrackingData structure.
            On entry, the menu, virtualMenuTop, and virtualMenuBottom fields of this structure are
            valid. The menu definition should determine which item containst the given point, if any,
            and fill in the itemUnderMouse, itemSelected, and itemRect fields. If an item is found,
            the menu definition should always fill in the itemUnderMouse and itemRect fields. The
            menu definition should only fill in the itemSelected field if the item is available for
            selection; if it is unavailable (because it is disabled, or for some other reason), the
            menu definition should set the itemSelected field to zero.
            
            The values placed in the itemUnderMouse and itemSelected fields should be less than or
            equal to the number of items returned by CountMenuItems on this menu. The values placed
            in these two fields should be identical if both are non-zero. itemUnderMouse should always
            be non-zero if the mouse is actually over an item.
            
            The menu definition should not hilite the found item during this message. The Menu 
            Manager will send a separate kMenuHiliteItemMsg to request hiliting of the item.
            
            If the menu definition supports scrolling, it should scroll the menu during this message,
            and update the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData to
            indicate the menu's new scrolled position.
            
            If the menu definition uses QuickDraw to draw while scrolling, it should draw into the
            current port.
    
            If the menu definition uses CoreGraphics to draw while scrolling, it should use the
            CGContextRef passed in the context field of the MDEFHiliteItemData structure.
            
            Menu definitions must use the ScrollMenuImage API, if available, to scroll the menu contents.
            This API is available in CarbonLib 1.5 and later, and in Mac OS X 10.1 and later. ScrollMenuImage
            properly supports scrolling the alpha channel in the menu's image data. Use of QuickDraw's
            ScrollRect API to scroll the menu contents will result in the alpha channel being set to 0xFF
            (opaque) and the menu will no longer be transparent.
            
            The menu definition should not modify the menu field of the MenuTrackingData.
            
        kMenuHiliteItemMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFHiliteItemData*
            
            Sent when the Menu Manager is displaying a menu and needs to hilite a newly selected
            item. The whichItem parameter is actually a pointer to a MDEFHiliteItemData structure.
            The menu definition should unhilite the item in the previousItem field, if non-zero,
            and hilite the item in the newItem field.
            
            Menu definitions should use the EraseMenuBackground API to erase the old menu contents
            before unhiliting a menu item, if the menu definition is using the Appearance Manager's
            menu drawing APIs. This is necessary because the background of a menu is transparent on
            Aqua, and if the old hilite is not erased first, it will show through the new unhilited
            menu background.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFHiliteItemData structure.
            
        kMenuDrawItemsMsg
        
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawItemsData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw a portion of the
            menu. This message is used by the dynamic menu item support code in the Menu Manager;
            for example, if items five and six in a menu are a dynamic group, the Menu Manager will
            send a DrawItems message when the group's modifier key is pressed or released to redraw
            the appropriate item, but no other items in the menu.
            
            The whichItem parameter for this message is actually a pointer to a MDEFDrawItemsData
            structure. The menu definition should redraw the items starting with firstItem and
            ending with lastItem, inclusive.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawItemsData structure.
            
        kMenuDrawMsg
            
            menuRect        menu bounds
            hitPt           unused
            whichItem       MDEFDrawData*
            
            Sent when the Menu Manager is displaying a menu and needs to redraw the entire menu.
            The whichItem parameter is actually a pointer to a MenuTrackingData structure. On entry,
            the menu field of this structure is valid. The menu definition should draw the menu and,
            if it supports scrolling, should also fill in the virtualMenuTop and virtualMenuBottom
            fields of the structure to indicate the menu's initial unscrolled position; typically, 
            virtualMenuTop would be set to the same value as the top coordinate of the menu rect,
            and virtualMenuBottom would be set to virtualMenuTop plus the virtual height of the menu.
            
            If the menu definition uses QuickDraw to draw, it should draw into the current port.
    
            If the menu definition uses CoreGraphics to draw, it should use the CGContextRef passed
            in the context field of the MDEFDrawData structure.
            
        kMenuSizeMsg
        
            menuRect        unused
            hitPt           maximum width and height of the menu
            whichItem       unused
            
            Sent when the Menu Manager needs to determine the size of a menu. The menu definition
            should calculate the width and height of the menu and store the sizes into the menu with
            SetMenuWidth and SetMenuHeight.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the hitPt parameter to this message is the maximum width (hitPt.h) and height
            (hitPt.v) of the menu. The menu definition should ensure that the width and height that it
            places in the menu do not exceed these values. If the gestalt bit is not set, the menu
            definition should just use the main GDevice's width and height as constraints on the menu's
            width and height.
            
        kMenuPopUpMsg
        
            menuRect        on entry, constraints on the menu's position; on exit, menu bounds
            hitPt           requested menu location, with swapped coordinates
            whichItem       on entry, requested initial selection; on exit, virtual menu top
            
            Sent when the Menu Manager is about to display a popup menu. The menu definition should
            calculate the appropriate menu bounds to contain the menu based on the requested menu
            location and selected item. It should write the menuBounds into the rect given by the
            menuRect parameter.
            
            If the gestaltMenuMgrSendsMenuBoundsToDefProc bit is set in the Menu Manager's Gestalt
            value, then the menuRect parameter on entry to this message contains a constraint rect,
            in global coordinates, outside of which the popup menu should not be positioned. The menu
            definition should take this constraint rect into account as it calculates the menu bounds.
            If the gestalt bit is not set, the menu definition should use the bounds of the GDevice
            containing the menu's top left corner as a constraint on the menu's position.
            
            The hitPt parameter is a requested location for the top left corner of the menu. The
            coordinates of this parameter are swapped from their normal order; the h field of the
            hitPt parameter contains the vertical coordinate, and the v field of hitPt contains
            the horizontal coordinate.
            
            On entry, the whichItem parameter points at a menu item index which is requested to be
            the initial selection when the menu is displayed. After calculating the menu's bounds,
            the menu definition should write the menu's virtual top coordinate into the location
            pointed at by the whichItem parameter. If displaying the menu at the requested location
            does not require scrolling, the virtual top will be the same as the menu bounds top;
            if the menu must scroll to fit in the requested location, the virtual top may be different.
            
        kMenuCalcItemMsg
        
            menuRect        on exit, item bounds
            hitPt           unused
            whichItem       the item whose rect to calculate
            
            Sent when the Menu Manager needs to know the bounds of a menu item. The menu definition
            should calculate the size of the item specified by the whichItem parameter, and store 
            the bounds in the rect specified by the menuRect parameter.
            
            Some sample menu definition code provided by Apple has previously shown an implementation
            of this message that always sets the top left corner of the item bounds to (0,0), regardless
            of the item's actual position in the menu. For best future compatibility, menu definitions
            should begin storing an item bounds that gives the item's actual position in the menu based
            on the menu's current virtual top. For example, if the virtual menu top starts at 20, then
            the menu definition would calculate an item bounds for the first item that starts at (0,20),
            an item bounds for the second item that starts at (0,40), and so on. The menu definition
            should call GetMenuTrackingData to get the menu's current virtual position, and use zero
            for the menu top if GetMenuTrackingData returns an error.
            
        kMenuThemeSavvyMsg
        
            menuRect        unused
            hitPt           unused
            whichItem       on exit, indicates theme-savvyness of the menu definition
            
            Sent by the Menu Manager to determine whether the MDEF uses the Appearance Manager 
            menu-drawing functions to draw its menu. If it does, the menu definition should return
            kThemeSavvyMenuResponse in the location pointed to by whichItem. If the menu definition
            draws its own custom content without using the Appearance Manager menu-drawing functions,
            it should ignore this message.
    
    Low-memory Global Replacements
    
        Pre-Carbon menu definitions needed to use several low-memory globals to communicate with the
        Menu Manager. These globals have all been replaced or made obsolete in Carbon, as follows:
        
        MenuDisable
            
            MenuDisable is now set automatically by the Menu Manager using the value returned in the
            itemUnderMouse field of the MenuTrackingData structure passed to kMenuFindItemMsg.
            
        TopMenuItem
        AtMenuBottom
        
            TopMenuItem and AtMenuBottom are now set automatically by the Menu Manager using the
            values returned in the virtualMenuTop and virtualMenuBottom fields of the MenuTrackingData
            structure passed to kMenuDrawMsg and kMenuFindItemMsg.
            
        mbSaveLoc
    
            This undocumented low-memory global was used by pre-Carbon menu definitions to store
            the bounding rect of the currently selected item and to avoid drawing glitches while
            the menu definition was scrolling the contents of a menu that had submenus. The Menu
            Manager now automatically sets the selected item bounds using the value returned in
            the itemRect field of the MenuTrackingData structure passed to kMenuFindItemMsg. In
            order to correctly support scrolling of menus with submenus, a menu definition should
            verify, before scrolling the menu contents, that no submenus of the scrolling menu are
            currently visible. A menu definition can use GetMenuTrackingData to verify this condition,
            as follows:
            
                Boolean SafeToScroll( MenuRef menuBeingScrolled )
                (
                    MenuTrackingData lastMenuData;
                    return GetMenuTrackingData( NULL, &lastMenuData ) == noErr
                           && lastMenuData.menu == menuBeingScrolled;
                )
            
            If SafeToScroll returns false, the menu definition should not scroll the menu.
}
{$endc} {not TARGET_CPU_64}

const
{ menu defProc messages }
	kMenuDrawMsg = 0;
	kMenuSizeMsg = 2;
	kMenuPopUpMsg = 3;    { position the popup menu rect appropriately }
	kMenuCalcItemMsg = 5;
	kMenuThemeSavvyMsg = 7;    { is your MDEF theme-savvy?  If so, return kThemeSavvyMenuResponse in the whichItem parameter}
	kMenuInitMsg = 8;    { Return an error code in *whichItem to indicate success or failure. Only supported in Carbon. }
	kMenuDisposeMsg = 9;    { The menu is being destroyed. Only supported in Carbon.}
	kMenuFindItemMsg = 10;   { Determine which item is under the mouse. Only supported in Carbon.}
	kMenuHiliteItemMsg = 11;   { Hilite the specified item. Only supported in Carbon.}
	kMenuDrawItemsMsg = 12;   { Draw a range of items. Only supported in Carbon.}
	mDrawMsg = kMenuDrawMsg; { obsolete constant name}
	mSizeMsg = kMenuSizeMsg; { obsolete constant name}
	mPopUpMsg = kMenuPopUpMsg; { obsolete constant name}
	mCalcItemMsg = kMenuCalcItemMsg; { obsolete constant name}


const
	kThemeSavvyMenuResponse = $7473; { should be returned in *whichItem when handling kMenuThemeSavvyMsg}


{
 *  MDEFHiliteItemData
 *  
 *  Summary:
 *    The MDEFHiliteItemData structure contains information about which
 *    menu items should be hilited and unhilited as the user moves
 *    through the menus. It is used by menu definition functions, which
 *    receive a pointer to an MDEFHiliteItemData structure as the
 *    whichItem parameter during kMenuHiliteItemMsg.
 }
type
	MDEFHiliteItemData = record
{
   * The item that was previously selected. It should be redrawn in an
   * unhilited state. May be zero if no item was previously selected.
   }
		previousItem: MenuItemIndex;

  {
   * The item that is now selected. It should be redrawn in a hilited
   * state. May be zero if no item is now selected.
   }
		newItem: MenuItemIndex;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFHiliteItemDataPtr = ^MDEFHiliteItemData;
type
	HiliteMenuItemData = MDEFHiliteItemData;
	HiliteMenuItemDataPtr = MDEFHiliteItemDataPtr;

{
 *  MDEFDrawData
 *  
 *  Summary:
 *    The MDEFDrawData structure contains information needed to draw a
 *    menu. It is used by menu definition functions, which receive a
 *    pointer to an MDEFDrawData structure as the whichItem parameter
 *    during kMenuDrawMsg.
 }
type
	MDEFDrawData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the virtualMenuTop and virtualMenuBottom fields of this structure
   * while drawing the menu.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFDrawDataPtr = ^MDEFDrawData;

{
 *  MDEFFindItemData
 *  
 *  Summary:
 *    The MDEFFindItemData structure contains information needed to
 *    determine which item is currently selected by the user. It is
 *    used by menu definition functions, which receive a pointer to an
 *    MDEFDrawData structure as the whichItem parameter during
 *    kMenuFindItemMsg.
 }
type
	MDEFFindItemData = record
{
   * Information about the menu being drawn. The MDEF should fill in
   * the itemSelected, itemUnderMouse, and itemRect fields of this
   * structure after determining which item is at the specified point.
   }
		trackingData: MenuTrackingData;

  {
   * A CoreGraphics context that the MDEF should draw into if it needs
   * to scroll the menu during the FindItem message. The Menu Manager
   * will flush the context after the MDEF has returned.
   }
		context: UnivPtr;
	end;
	MDEFFindItemDataPtr = ^MDEFFindItemData;

{
 *  MDEFDrawItemsData
 *  
 *  Summary:
 *    The MDEFDrawItemsData structure contains information about which
 *    menu items to redraw. It is used by menu definition functions,
 *    which receive a pointer to an MDEFDrawItemsData structure as the
 *    whichItem parameter during kMenuDrawItemsMsg.
 }
type
	MDEFDrawItemsData = record
{
   * The first item to draw.
   }
		firstItem: MenuItemIndex;

  {
   * The last item to draw.
   }
		lastItem: MenuItemIndex;

  {
   * Information about the menu's tracking state. The virtualMenuTop
   * and virtualMenuBottom fields in this structure will be the most
   * useful in handling the DrawItems message.
   }
		trackingData: MenuTrackingDataPtr;

  {
   * A CoreGraphics context that the MDEF should draw into. The Menu
   * Manager will flush the context after the MDEF returns.
   }
		context: UnivPtr;
	end;
	MDEFDrawItemsDataPtr = ^MDEFDrawItemsData;
{ obsolete menu color table constants}
const
	mctAllItems = -98;  {search for all Items for the given ID}
	mctLastIDIndic = -99;   {last color table entry has this in ID field}

type
	MCEntry = record
		mctID: MenuID;                  {menu ID.  ID = 0 is the menu bar}
		mctItem: SInt16;                {menu Item. Item = 0 is a title}
		mctRGB1: RGBColor;                {usage depends on ID and Item}
		mctRGB2: RGBColor;                {usage depends on ID and Item}
		mctRGB3: RGBColor;                {usage depends on ID and Item}
		mctRGB4: RGBColor;                {usage depends on ID and Item}
		mctReserved: SInt16;            {reserved for internal use}
	end;
	MCEntryPtr = ^MCEntry;
type
	MCTable = array [0..0] of MCEntry;	{ARRAY [1..numEntries] of MCEntry}
	MCTablePtr = ^MCTable;
	MCTableHandle = ^MCTablePtr;
type
	MenuCRsrc = record
		numEntries: SInt16;             {number of entries}
		mcEntryRecs: MCTable;            {ARRAY [1..numEntries] of MCEntry}
	end;
	MenuCRsrcPtr = ^MenuCRsrc;
type
	MenuCRsrcHandle = ^MenuCRsrcPtr;


const
{
   * Menu item command ID to indicate a hierarchical menu; the item
   * icon ID is the hierarchical menu ID. This constant is deprecated.
   * Use SetMenuItemHierarchicalID or SetMenuItemHierarchicalMenu
   * instead of using this constant.
   }
	hMenuCmd = 27;

  {
   * A menu ID used with InsertMenu to insert a menu into the
   * hierarchical portion of the menubar. Rather than inserting menus
   * into the hierarchical portion of the menubar, we recommend that
   * your application use SetMenuItemHierarchicalMenu to directly
   * specify the submenu's MenuRef.
   }
	kInsertHierarchicalMenu = -1;

  {
   * The old name for kInsertHierarchicalMenu. This constant is
   * deprecated.
   }
	hierMenu = -1;

{$ifc not TARGET_CPU_64}
{
 *  NewMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateNewMenu instead of NewMenu.
 *  
 *  Summary:
 *    Creates a new menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewMenu( menuID_: MenuID; const (*var*) menuTitle: Str255 ): MenuRef; external name '_NewMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuTitleAsCFString instead of GetMenuTitle.
 *  
 *  Summary:
 *    Retrieves the title of a menu as a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function GetMenuTitle( menu: MenuRef; var title: Str255 ): StringPtr; external name '_GetMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuTitle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuTitleWithCFString instead of SetMenuTitle.
 *  
 *  Summary:
 *    Sets the title of a menu to a Pascal string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function SetMenuTitle( menu: MenuRef; const (*var*) title: Str255 ): OSStatus; external name '_SetMenuTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  [Mac]AppendMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use AppendMenuItemTextWithCFString instead of AppendMenu. Note
 *    that unlike AppendMenu, AppendMenuItemTextWithCFString does not
 *    interpret metacharacters in the text.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    This function is used to retrieve many attributes of a menu item
 *    simultaneously; for example, it might be used by a menu
 *    definition function that needs to know how to draw a menu item.
 *    It is more efficient to use this function than to use the
 *    accessor functions for the individual attributes of the menu.
 *    This function returns a copy of the data in the menu, so any data
 *    in the MenuItemDataRec that is dynamically allocated (for
<<<<<<< HEAD
 *    example, the CFString item text) should be released by the caller.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure AppendResMenu( theMenu: MenuRef; theType: ResType ); external name '_AppendResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertFontResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateStandardFontMenu or a font panel window instead of
 *    InsertFontResMenu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of font resources.
=======
=======
 *    example, the CFString item text) should be released by the caller.
=======
 *  Summary:
 *    Appends a new menu item to a menu, using a Pascal string for the
 *    item's text.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu from which to copy data. If inIsCommandID is true, you
 *      may pass NULL for this parameter to search for an item in the
 *      root menu; if inIsCommandID is false, this parameter must be a
 *      valid MenuRef.
 *    
 *    inItem:
 *      The item or command ID from which to copy data.
 *    
 *    inIsCommandID:
 *      Indicates whether inItem is a MenuItemIndex or MenuCommand. If
 *      inIsCommandID is true, the inItem parameter is interpreted as a
 *      menu command ID, and data is copied from the first item in the
 *      menu with that command ID. If inIsCommandID is false, the
 *      inItem parameter is interpreted as a menu item index, and data
 *      is copied for that item in the specified menu.
 *    
 *    ioData:
 *      Data is copied from the item and placed here. On entry, the
 *      whichData field of this structure should be initialized to
 *      indicate which data the caller would like returned. Individual
 *      fields of the MenuItemDataRec structure may require
 *      pre-initialization also; see the individual MenuItemDataFlags
 *      documentation for details.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure AppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacAppendMenu( menu: MenuRef; const (*var*) data: Str255 ); external name '_AppendMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  [Mac]InsertMenuItem()
 *  
 *  Summary:
<<<<<<< HEAD
 *    Sets multiple attributes of a menu item at once.
 *  
 *  Discussion:
 *    This function is used to set many attributes of a menu item
 *    simultaneously. It is more efficient to use this function than to
 *    use the accessor functions for the individual attributes of the
 *    menu.
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
procedure InsertFontResMenu( theMenu: MenuRef; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertFontResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertIntlResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
=======
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theMenu:
 *      The menu into which to insert the item.
 *    
 *    itemString:
 *      The text of the new item. This string is parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
procedure MacInsertMenuItem( theMenu: MenuRef; const (*var*) itemString: Str255; afterItem: MenuItemIndex ); external name '_InsertMenuItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  IsMenuItemInvalid()
 *  
 *  Summary:
 *    Determines if a menu item is invalid and should be redrawn.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  AppendMenuItemText()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertIntlResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertIntlResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MenuKey()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use MenuEvent, IsMenuKeyEvent, or a kEventCommandProcess Carbon
 *    event handler instead of MenuKey.
 *  
 *  Summary:
 *    Determines if a character matches a menu item command key.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose item to examine.
 *    
 *    inItem:
 *      The item to examine.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function AppendMenuItemText( menu: MenuRef; const (*var*) inString: Str255 ): OSStatus; external name '_AppendMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use InsertMenuItemTextWithCFString instead of InsertMenuItemText.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Menu items are automatically invalidated when their contents are
 *    changed using Menu Manager APIs while the menu is open. However,
 *    you might need to use this API if you have a custom MDEF that
 *    draws using state not contained in the menu.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Inserts a new menu item into a menu, using a Str255 for the item
 *    text.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
>>>>>>> graemeg/cpstrnew
 }
<<<<<<< HEAD
function InitContextualMenus: OSStatus; external name '_InitContextualMenus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
<<<<<<< HEAD
 *  NewMenuHookUPP()
=======
 *  NewMBarHookUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
<<<<<<< HEAD
=======
 *  ProcessIsContextualMenuClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    There is no reason for an application to call this API.
 *  
 *  Summary:
 *    Determines whether the specified process is a contextual menu
 *    client.
 *  
 *  Discussion:
 *    On Mac OS 9, this API consults a global table of all processes
 *    that have registered with the Contextual Menu Manager by calling
 *    InitContextualMenus. On Mac OS X, this API ignores the inPSN
 *    parameter and always returns whether the current process is
 *    registered with the Contextual Menu Manager.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    menu:
 *      The menu into which to insert the item.
 *    
 *    inString:
 *      The text of the new item. This string is not parsed for the
 *      meta-characters documented in the Menu Manager chapter of
 *      Inside Macintosh; the new item's text becomes exactly the
 *      specified text.
 *    
 *    afterItem:
 *      The menu item after which to insert the item. Pass 0 to insert
 *      the item at the beginning of the menu. If afterItem is greater
 *      than the number of items in the menu, the item is inserted at
 *      the end of the menu.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MenusLib 8.5 and later
 }
function InsertMenuItemText( menu: MenuRef; const (*var*) inString: Str255; afterItem: MenuItemIndex ): OSStatus; external name '_InsertMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

<<<<<<< HEAD
{
 *  UpdateInvalidMenuItems()
 *  
 *  Summary:
 *    Redraws the invalid items of an open menu.
 *  
 *  Discussion:
 *    It is not necessary to use UpdateInvalidMenuItems if you are
 *    using Carbon's built-in support for dynamic items based on
 *    modifier key state. However, if you are modifying items
 *    dynamically using your own implementation, you should call
 *    UpdateInvalidMenuItems after completing your modifications for a
 *    single menu. It will redraw any items that have been marked as
 *    invalid, and clear the invalid flag for those items.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{
 *  SetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of SetMenuItemText.
 *  
 *  Summary:
 *    Sets the text of a menu item using a Pascal string.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
 *    inPSN:
 *      The process to examine.
 *  
 *  Result:
 *    Whether the specified process (or, on Mac OS X, the current
 *    process) is registered with the Contextual Menu Manager.
=======
function MenuKey( ch: CharParameter ): SInt32; external name '_MenuKey';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuFlashCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function has no effect on Mac OS X.
 *  
 *  Summary:
 *    Sets the number of times that a selected menu item will blink.
=======
=======
>>>>>>> origin/fixes_2_2
 *    inMenu:
 *      The menu to update.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; const (*var*) itemString: Str255 ); external name '_SetMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  CreateStandardFontMenu()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
procedure SetMenuFlashCount( count: SInt16 ); external name '_SetMenuFlashCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InitContextualMenus()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X, it is not necessary to call InitContextualMenus.
 *  
 *  Summary:
 *    Adds the current process to the system registry of contextual
 *    menu clients.
 *  
 *  Discussion:
 *    On Mac OS 8.x and 9.x, your program should call the
 *    InitContextualMenus function early in your startup code to
 *    register your application as a contextual menu client. If you do
 *    not register your program, some system-level functions may
 *    respond as though your program does not use contextual menus. Not
 *    registering your program may also cause
 *    ProcessIsContextualMenuClient to return an incorrect value.
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
 }
function ProcessIsContextualMenuClient( var inPSN: ProcessSerialNumber ): Boolean; external name '_ProcessIsContextualMenuClient';
=======
 }
function InitContextualMenus: OSStatus; external name '_InitContextualMenus';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

<<<<<<< HEAD
{
<<<<<<< HEAD
 *  NewMBarHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
=======
>>>>>>> graemeg/cpstrnew
=======
 *  ProcessIsContextualMenuClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    There is no reason for an application to call this API.
 *  
 *  Summary:
 *    Determines whether the specified process is a contextual menu
 *    client.
 *  
 *  Discussion:
 *    On Mac OS 9, this API consults a global table of all processes
 *    that have registered with the Contextual Menu Manager by calling
 *    InitContextualMenus. On Mac OS X, this API ignores the inPSN
 *    parameter and always returns whether the current process is
 *    registered with the Contextual Menu Manager.
=======
=======
=======
 *  GetMenuItemText()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of GetMenuItemText.
 *  
 *  Summary:
 *    Retrieves the text of a menu item as a Pascal string.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetMenuItemText( theMenu: MenuRef; item: MenuItemIndex; var itemString: Str255 ); external name '_GetMenuItemText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemTextWithCFString instead of setting an item's text
 *    as a Str255.
 *  
 *  Summary:
 *    Sets the text encoding that should be used to interpret a menu
 *    item's Pascal string-based text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; inScriptID: TextEncoding ): OSErr; external name '_SetMenuItemTextEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetFontFamilyFromMenuSelection()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    inPSN:
 *      The process to examine.
 *  
 *  Result:
 *    Whether the specified process (or, on Mac OS X, the current
 *    process) is registered with the Contextual Menu Manager.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
>>>>>>> origin/cpstrnew
 }
function ProcessIsContextualMenuClient( var inPSN: ProcessSerialNumber ): Boolean; external name '_ProcessIsContextualMenuClient';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

<<<<<<< HEAD
{
 *  DisposeMenuBarDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
=======

{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Color Tables                                                            }
{  Menu color manipulation is considered deprecated with the advent of the Appearance  }
{  Manager.  Avoid using these routines if possible                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  DeleteMCEntries()   *** DEPRECATED ***
 *  
=======

{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Color Tables                                                            }
{  Menu color manipulation is considered deprecated with the advent of the Appearance  }
{  Manager.  Avoid using these routines if possible                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  DeleteMCEntries()   *** DEPRECATED ***
 *  
>>>>>>> origin/cpstrnew
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Removes color table entries for a menu.
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
>>>>>>> graemeg/cpstrnew
 }
procedure DeleteMCEntries( menuID_: MenuID; menuItem: SInt16 ); external name '_DeleteMCEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
<<<<<<< HEAD

{
<<<<<<< HEAD
 *  DisposeMenuHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }
=======
 *  GetMCInfo()   *** DEPRECATED ***
=======


{
 *  GetMCInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Retrieves color table entries for the current process.
=======
 *  GetMenuItemTextEncoding()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CopyMenuItemTextAsCFString instead of retrieving a menu
 *    item's text as a Str255.
 *  
 *  Summary:
 *    Retrieves the text encoding that should be used to interpret a
 *    menu item's Pascal string-based text.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMCInfo: MCTableHandle; external name '_GetMCInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMCInfo()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Retrieves color table entries for the current process.
=======
 *    Sets color table info for the current process.
>>>>>>> origin/cpstrnew
=======
 *  Availability:
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function GetMenuItemTextEncoding( inMenu: MenuRef; inItem: MenuItemIndex; var outScriptID: TextEncoding ): OSErr; external name '_GetMenuItemTextEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemIconHandle instead of SetItemIcon.
 *  
 *  Summary:
 *    Sets the resource ID of an icon resource to be displayed in a
 *    menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetItemIcon( theMenu: MenuRef; item: MenuItemIndex; iconIndex: SInt16 ); external name '_SetItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetItemIcon()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemIconHandle instead of GetItemIcon.
 *  
 *  Summary:
 *    Retrieves the resource ID of an icon resource to be displayed in
 *    a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure GetItemIcon( theMenu: MenuRef; item: MenuItemIndex; var iconIndex: SInt16 ); external name '_GetItemIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetMenuItemHierarchicalMenu instead of
 *    SetMenuItemHierarchicalID.
 *  
 *  Summary:
 *    Sets the menu ID of the submenu of a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppearanceLib 1.0 and later
 }
function SetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; inHierID: MenuID ): OSErr; external name '_SetMenuItemHierarchicalID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuItemHierarchicalID()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetMenuItemHierarchicalMenu instead of
 *    GetMenuItemHierarchicalID.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    On Mac OS 8.x and 9.x, your program should call the
 *    InitContextualMenus function early in your startup code to
 *    register your application as a contextual menu client. If you do
 *    not register your program, some system-level functions may
 *    respond as though your program does not use contextual menus. Not
 *    registering your program may also cause
 *    ProcessIsContextualMenuClient to return an incorrect value. On
 *    Mac OS X, it is not necessary to call InitContextualMenus.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Retrieves the menu ID of the submenu of a menu item.
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
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function GetMCInfo: MCTableHandle; external name '_GetMCInfo';
=======
procedure SetMCInfo( menuCTbl: MCTableHandle ); external name '_SetMCInfo';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetMCInfo()   *** DEPRECATED ***
=======
 *  DisposeMCInfo()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Sets color table info for the current process.
=======
 *    Releases the memory used by a menu color table.
>>>>>>> origin/cpstrnew
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
function GetMenuItemHierarchicalID( inMenu: MenuRef; inItem: MenuItemIndex; var outHierID: MenuID ): OSErr; external name '_GetMenuItemHierarchicalID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMenuRetainCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFGetRetainCount instead of
 *    GetMenuRetainCount.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Applications should call IsShowContextualMenuClick when they
 *    receive non-null events. If IsShowContextualMenuClick returns
 *    true, your application should generate its own menu and Apple
 *    Event descriptor (AEDesc), and then call ContextualMenuSelect to
 *    display and track the contextual menu, and then handle the user's
 *    choice. Some users may choose to use a two-button mouse with
 *    their Macintosh computer. This API does not return true for a
 *    right-click unless the mouse manufacturer has provided driver
 *    software that returns a control-left click in place of a right
 *    click. For proper recognition of a right-click gesture, you must
 *    use the IsShowContextualMenuEvent API.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Returns the retain count of a menu.
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
procedure SetMCInfo( menuCTbl: MCTableHandle ); external name '_SetMCInfo';
=======
procedure DisposeMCInfo( menuCTbl: MCTableHandle ); external name '_DisposeMCInfo';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  DisposeMCInfo()   *** DEPRECATED ***
=======
 *  GetMCEntry()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Releases the memory used by a menu color table.
=======
 *    Returns a pointer to the menu color table entry for a menu item.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to return.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetMenuRetainCount( inMenu: MenuRef ): ItemCount; external name '_GetMenuRetainCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RetainMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRetain instead of RetainMenu.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    This API is similar to IsShowContextualMenuClick, but takes a
 *    Carbon EventRef as its parameter instead of an EventRecord.
 *    EventRecords cannot express a right-mouse-click, but EventRefs
 *    can, so this API will return true for a right- click where
 *    IsShowContextualMenuClick will not. 
 *    
 *    In Mac OS X 10.3 and earlier, this API always returned false if
 *    the event kind was not kEventMouseDown,
 *    kEventWindowClickContentRgn, kEventWindowClickStructureRgn, or
 *    kEventWindowHandleContentClick. In Mac OS X 10.4 and later, this
 *    API no longer requires a specific event kind; it only requires
 *    that the event contain kEventParamMouseButton and
 *    kEventParamKeyModifiers parameters.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Summary:
 *    Increments the retain count of a menu.
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
procedure DisposeMCInfo( menuCTbl: MCTableHandle ); external name '_DisposeMCInfo';
=======
function GetMCEntry( menuID_: MenuID; menuItem: SInt16 ): MCEntryPtr; external name '_GetMCEntry';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetMCEntry()   *** DEPRECATED ***
=======
 *  SetMCEntries()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Returns a pointer to the menu color table entry for a menu item.
=======
 *    Sets entries in the menu color table for the current process.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to increment.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function RetainMenu( inMenu: MenuRef ): OSStatus; external name '_RetainMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ReleaseMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.2 and later, use CFRelease instead of ReleaseMenu.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    If IsShowContextualMenuClick returns true, you should call the
 *    ContextualMenuSelect API after generating your own menu and
 *    preparing an Apple Event descriptor (AEDesc) that describes the
 *    item for which your application is displaying a contextual menu.
 *    This descriptor may contain an object specifier or raw data and
 *    will be passed to all contextual menu plug-ins. The system will
 *    add other items before displaying the contextual menu, and it
 *    will remove those items before returning, leaving the menu in its
 *    original state. After all the system commands are added, the
 *    contextual menu is displayed and tracked. If the user selects one
 *    of the system items, it is handled by the system and the call
 *    returns as though the user didn't select anything from the menu.
 *    If the user selects any other item (or no item at all), the Menu
 *    Manager passes back appropriate values in the parameters
 *    outUserSelectionType, outMenuID, and outMenuItem. Your
 *    application should provide visual feedback indicating the item
 *    that was clicked upon. For example, a click on an icon should
 *    highlight the icon, while a click on editable text should not
<<<<<<< HEAD
 *    eliminate the current selection. If the outUserSelectionType
 *    parameter contains kCMMenuItemSelected, you should look at the
 *    outMenuID and outMenuItem parameters to determine what menu item
 *    the user chose and handle it appropriately. If the user selected
 *    kCMHelpItemSelected, you should open the proper Apple Guide
 *    sequence or other form of custom help.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function GetMCEntry( menuID_: MenuID; menuItem: SInt16 ): MCEntryPtr; external name '_GetMCEntry';
=======
procedure SetMCEntries( numEntries: SInt16; menuCEntries: MCTablePtr ); external name '_SetMCEntries';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • MDEF support                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{
<<<<<<< HEAD
 *  SetMCEntries()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Sets entries in the menu color table for the current process.
=======
 *  EraseMenuBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use a custom HIView instead of a menu
 *    definition function to customize the appearance of a menu.
 *  
 *  Summary:
 *    Erases a portion of a menu background in preparation for further
 *    drawing.
 *  
 *  Discussion:
 *    It is necessary to erase the menu background before calling
 *    DrawThemeMenuBackground because some themes (such as Aqua on Mac
 *    OS X) draw the menu background using the alpha channel, and if
 *    the area underneath the menu background is not erased, portions
 *    of the old image will show through the menu background.
>>>>>>> origin/cpstrnew
=======
=======
 *    eliminate the current selection. If the outUserSelectionType
 *    parameter contains kCMMenuItemSelected, you should look at the
 *    outMenuID and outMenuItem parameters to determine what menu item
 *    the user chose and handle it appropriately. If the user selected
 *    kCMHelpItemSelected, you should open the proper Apple Guide
 *    sequence or other form of custom help.
=======
 *  Summary:
 *    Decrements the retain count of a menu.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose retain count to decrement. If the retain count
 *      falls to zero, the menu is destroyed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function ReleaseMenu( inMenu: MenuRef ): OSStatus; external name '_ReleaseMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
<<<<<<< HEAD
 *  Discussion:
 *    On Mac OS 9, this API consults a global table of all processes
 *    that have registered with the Contextual Menu Manager by calling
 *    InitContextualMenus. On Mac OS X, this API ignores the inPSN
 *    parameter and always returns whether the current process is
 *    registered with the Contextual Menu Manager.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inMenu:
 *      The menu whose background to erase.
 *    
 *    inEraseRect:
 *      The bounds of the area to erase, in local coordinates to the
 *      current port.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, EraseMenuBackground will
 *      create a context based on the current port.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMCEntries( numEntries: SInt16; menuCEntries: MCTablePtr ); external name '_SetMCEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • MDEF support                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  EraseMenuBackground()   *** DEPRECATED ***
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function EraseMenuBackground( inMenu: MenuRef; const (*var*) inEraseRect: Rect; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_EraseMenuBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ScrollMenuImage()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use a custom HIView instead of a menu
 *    definition function to customize the appearance of a menu.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Erases a portion of a menu background in preparation for further
 *    drawing.
 *  
 *  Discussion:
 *    It is necessary to erase the menu background before calling
 *    DrawThemeMenuBackground because some themes (such as Aqua on Mac
 *    OS X) draw the menu background using the alpha channel, and if
 *    the area underneath the menu background is not erased, portions
 *    of the old image will show through the menu background.
=======
 *    Scrolls a portion of the menu image.
 *  
 *  Discussion:
 *    Menus on Mac OS X use an alpha channel, and QuickDraw does not
 *    support alpha channels. Therefore, scrolling a menu image with
 *    ScrollRect or other QuickDraw APIs does not work correctly; it
 *    results in the destruction of the alpha channel data. The
 *    ScrollMenuImage API uses CoreGraphics to move the menu image,
 *    preserving the alpha channel.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
<<<<<<< HEAD
 *      The menu whose background to erase.
 *    
 *    inEraseRect:
 *      The bounds of the area to erase, in local coordinates to the
 *      current port.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, EraseMenuBackground will
 *      create a context based on the current port.
=======
 *      The menu whose image to scroll.
 *    
 *    inScrollRect:
 *      The bounds of the rect to scroll.
 *    
 *    inHScroll:
 *      The distance to scroll horizontally.
 *    
 *    inVScroll:
 *      The distance to scroll vertically.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, ScrollMenuImage will create a
 *      context based on the current port.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function EraseMenuBackground( inMenu: MenuRef; const (*var*) inEraseRect: Rect; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_EraseMenuBackground';
=======
function ScrollMenuImage( inMenu: MenuRef; const (*var*) inScrollRect: Rect; inHScroll: SInt32; inVScroll: SInt32; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_ScrollMenuImage';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

type
	MenuBarDefProcPtr = function( selector: SInt16; message: SInt16; parameter1: SInt16; parameter2: SInt32 ): SInt32;
	MenuHookProcPtr = procedure;
	MBarHookProcPtr = function( var menuRect: Rect ): SInt16;
	MenuBarDefUPP = MenuBarDefProcPtr;
	MenuHookUPP = MenuHookProcPtr;
	MBarHookUPP = MBarHookProcPtr;
{
<<<<<<< HEAD
 *  ScrollMenuImage()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use a custom HIView instead of a menu
 *    definition function to customize the appearance of a menu.
 *  
 *  Summary:
 *    Scrolls a portion of the menu image.
 *  
 *  Discussion:
 *    Menus on Mac OS X use an alpha channel, and QuickDraw does not
 *    support alpha channels. Therefore, scrolling a menu image with
 *    ScrollRect or other QuickDraw APIs does not work correctly; it
 *    results in the destruction of the alpha channel data. The
 *    ScrollMenuImage API uses CoreGraphics to move the menu image,
 *    preserving the alpha channel.
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *  NewMenuBarDefUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose image to scroll.
 *    
 *    inScrollRect:
 *      The bounds of the rect to scroll.
 *    
 *    inHScroll:
 *      The distance to scroll horizontally.
 *    
 *    inVScroll:
 *      The distance to scroll vertically.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, ScrollMenuImage will create a
 *      context based on the current port.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function ScrollMenuImage( inMenu: MenuRef; const (*var*) inScrollRect: Rect; inHScroll: SInt32; inVScroll: SInt32; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_ScrollMenuImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }
>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}

type
	MenuBarDefProcPtr = function( selector: SInt16; message: SInt16; parameter1: SInt16; parameter2: SInt32 ): SInt32;
	MenuHookProcPtr = procedure;
	MBarHookProcPtr = function( var menuRect: Rect ): SInt16;
	MenuBarDefUPP = MenuBarDefProcPtr;
	MenuHookUPP = MenuHookProcPtr;
	MBarHookUPP = MBarHookProcPtr;
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  DisposeMBarHookUPP()
=======
 *  NewMenuBarDefUPP()
>>>>>>> graemeg/cpstrnew
=======
 *  NewMenuHookUPP()
>>>>>>> origin/cpstrnew
=======
=======
=======
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
>>>>>>> origin/fixes_2_2
 *    inPSN:
 *      The process to examine.
 *  
 *  Result:
 *    Whether the specified process (or, on Mac OS X, the current
 *    process) is registered with the Contextual Menu Manager.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex ); external name '_InsertResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AppendResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Appends menu items containing the names of resources of a
 *    specified resource type.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure AppendResMenu( theMenu: MenuRef; theType: ResType ); external name '_AppendResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertFontResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CreateStandardFontMenu or a font panel window instead of
 *    InsertFontResMenu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of font resources.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertFontResMenu( theMenu: MenuRef; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertFontResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InsertIntlResMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The most comment use for this API has been to create a font menu
 *    containing the names of 'FOND' resources. For this purpose, use
 *    CreateStandardFontMenu, or use a font panel window instead of a
 *    menu.
 *  
 *  Summary:
 *    Inserts menu items containing the names of resources of a
 *    specified resource type.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure InsertIntlResMenu( theMenu: MenuRef; theType: ResType; afterItem: MenuItemIndex; scriptFilter: SInt16 ); external name '_InsertIntlResMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MenuKey()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use MenuEvent, IsMenuKeyEvent, or a kEventCommandProcess Carbon
 *    event handler instead of MenuKey.
 *  
 *  Summary:
 *    Determines if a character matches a menu item command key.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MenuKey( ch: CharParameter ): SInt32; external name '_MenuKey';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMenuFlashCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function has no effect on Mac OS X.
 *  
 *  Summary:
 *    Sets the number of times that a selected menu item will blink.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.3 and later
 }
procedure SetMenuFlashCount( count: SInt16 ); external name '_SetMenuFlashCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  appendmenu()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }

{
<<<<<<< HEAD
<<<<<<< HEAD
 *  InvokeMenuBarDefUPP()
=======
 *  NewMenuHookUPP()
=======
 *  NewMBarHookUPP()
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *  InitContextualMenus()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    On Mac OS X, it is not necessary to call InitContextualMenus.
 *  
 *  Summary:
 *    Adds the current process to the system registry of contextual
 *    menu clients.
 *  
 *  Discussion:
 *    On Mac OS 8.x and 9.x, your program should call the
 *    InitContextualMenus function early in your startup code to
 *    register your application as a contextual menu client. If you do
 *    not register your program, some system-level functions may
 *    respond as though your program does not use contextual menus. Not
 *    registering your program may also cause
 *    ProcessIsContextualMenuClient to return an incorrect value.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
>>>>>>> origin/fixes_2.4
 }
function InitContextualMenus: OSStatus; external name '_InitContextualMenus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  insertmenuitem()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }

{
<<<<<<< HEAD
 *  NewMBarHookUPP()
>>>>>>> graemeg/cpstrnew
=======
 *  DisposeMenuBarDefUPP()
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *  ProcessIsContextualMenuClient()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    There is no reason for an application to call this API.
 *  
 *  Summary:
 *    Determines whether the specified process is a contextual menu
 *    client.
 *  
 *  Discussion:
 *    On Mac OS 9, this API consults a global table of all processes
 *    that have registered with the Contextual Menu Manager by calling
 *    InitContextualMenus. On Mac OS X, this API ignores the inPSN
 *    parameter and always returns whether the current process is
 *    registered with the Contextual Menu Manager.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPSN:
 *      The process to examine.
 *  
 *  Result:
 *    Whether the specified process (or, on Mac OS X, the current
 *    process) is registered with the Contextual Menu Manager.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ContextualMenu 1.0 and later
>>>>>>> origin/fixes_2.4
 }
function ProcessIsContextualMenuClient( var inPSN: ProcessSerialNumber ): Boolean; external name '_ProcessIsContextualMenuClient';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • Menu Item Color Tables                                                            }
{  Menu color manipulation is considered deprecated with the advent of the Appearance  }
{  Manager.  Avoid using these routines if possible                                    }
{——————————————————————————————————————————————————————————————————————————————————————}
{
<<<<<<< HEAD
 *  menuselect()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD

{
 *  InvokeMenuHookUPP()
=======

{
 *  DisposeMenuBarDefUPP()
=======

{
 *  DisposeMenuHookUPP()
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  setmenuitemtext()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }

{
<<<<<<< HEAD
 *  DisposeMenuHookUPP()
=======
 *  DisposeMBarHookUPP()
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  DeleteMCEntries()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Removes color table entries for a menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DeleteMCEntries( menuID_: MenuID; menuItem: SInt16 ); external name '_DeleteMCEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  getmenuitemtext()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }

{
<<<<<<< HEAD
 *  DisposeMBarHookUPP()
>>>>>>> graemeg/cpstrnew
=======
 *  InvokeMenuBarDefUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD

{
=======

{
 *  InvokeMenuBarDefUPP()
=======

{
 *  InvokeMenuHookUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
<<<<<<< HEAD
 *  InvokeMenuHookUPP()
=======
 *  InvokeMBarHookUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
<<<<<<< HEAD
 }

{
>>>>>>> graemeg/cpstrnew
 *  InvokeMBarHookUPP()
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
=======
>>>>>>> origin/cpstrnew
 }

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

{$ifc TARGET_OS_MAC}
{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetMCInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Retrieves color table entries for the current process.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMCInfo: MCTableHandle; external name '_GetMCInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMCInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Sets color table info for the current process.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMCInfo( menuCTbl: MCTableHandle ); external name '_SetMCInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DisposeMCInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Releases the memory used by a menu color table.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DisposeMCInfo( menuCTbl: MCTableHandle ); external name '_DisposeMCInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetMCEntry()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Returns a pointer to the menu color table entry for a menu item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetMCEntry( menuID_: MenuID; menuItem: SInt16 ): MCEntryPtr; external name '_GetMCEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetMCEntries()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support menu color tables; this API has no
 *    effect.
 *  
 *  Summary:
 *    Sets entries in the menu color table for the current process.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure SetMCEntries( numEntries: SInt16; menuCEntries: MCTablePtr ); external name '_SetMCEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————}
{  • MDEF support                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————}
{
 *  EraseMenuBackground()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use a custom HIView instead of a menu
 *    definition function to customize the appearance of a menu.
 *  
 *  Summary:
 *    Erases a portion of a menu background in preparation for further
 *    drawing.
 *  
 *  Discussion:
 *    It is necessary to erase the menu background before calling
 *    DrawThemeMenuBackground because some themes (such as Aqua on Mac
 *    OS X) draw the menu background using the alpha channel, and if
 *    the area underneath the menu background is not erased, portions
 *    of the old image will show through the menu background.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose background to erase.
 *    
 *    inEraseRect:
 *      The bounds of the area to erase, in local coordinates to the
 *      current port.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, EraseMenuBackground will
 *      create a context based on the current port.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function EraseMenuBackground( inMenu: MenuRef; const (*var*) inEraseRect: Rect; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_EraseMenuBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ScrollMenuImage()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    In Mac OS X 10.3 and later, use a custom HIView instead of a menu
 *    definition function to customize the appearance of a menu.
 *  
 *  Summary:
 *    Scrolls a portion of the menu image.
 *  
 *  Discussion:
 *    Menus on Mac OS X use an alpha channel, and QuickDraw does not
 *    support alpha channels. Therefore, scrolling a menu image with
 *    ScrollRect or other QuickDraw APIs does not work correctly; it
 *    results in the destruction of the alpha channel data. The
 *    ScrollMenuImage API uses CoreGraphics to move the menu image,
 *    preserving the alpha channel.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inMenu:
 *      The menu whose image to scroll.
 *    
 *    inScrollRect:
 *      The bounds of the rect to scroll.
 *    
 *    inHScroll:
 *      The distance to scroll horizontally.
 *    
 *    inVScroll:
 *      The distance to scroll vertically.
 *    
 *    inContext:
 *      The CG context to erase. If NULL, ScrollMenuImage will create a
 *      context based on the current port.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function ScrollMenuImage( inMenu: MenuRef; const (*var*) inScrollRect: Rect; inHScroll: SInt32; inVScroll: SInt32; inContext: CGContextRef { can be NULL } ): OSStatus; external name '_ScrollMenuImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

type
	MenuBarDefProcPtr = function( selector: SInt16; message: SInt16; parameter1: SInt16; parameter2: SInt32 ): SInt32;
	MenuHookProcPtr = procedure;
	MBarHookProcPtr = function( var menuRect: Rect ): SInt16;
	MenuBarDefUPP = MenuBarDefProcPtr;
	MenuHookUPP = MenuHookProcPtr;
	MBarHookUPP = MBarHookProcPtr;
{
 *  NewMenuBarDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  NewMenuHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  NewMBarHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  DisposeMenuBarDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  DisposeMenuHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  DisposeMBarHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  InvokeMenuBarDefUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  InvokeMenuHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  InvokeMBarHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}
{$ifc not TARGET_CPU_64}



procedure GetItemMark( theMenu: MenuRef; item: MenuItemIndex; var markChar: CharParameter ); inline;
var
	markCharInt: UInt16;
begin
	__GetItemMark( theMenu, item, markCharInt);
	markChar:= CharParameter( markCharInt)
end;

procedure GetItemCmd( theMenu: MenuRef; item: MenuItemIndex; var cmdChar: CharParameter ); inline;
var
	cmdCharInt: UInt16;
begin
	__GetItemCmd( theMenu, item, cmdCharInt);
	cmdChar:= CharParameter( cmdCharInt)
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}

end.

{$endc} {not MACOSALLINCLUDE}
=======

end.
>>>>>>> graemeg/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}

end.

{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
