{
 *  AXAttributeConstants.h
 *  HIServices
 *
 *  Created by John Louch on Wed Feb 25 2004.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Copyright (c) 2004, 2006 Apple Computer, Inc. All rights reserved.
 *
 }

<<<<<<< HEAD
{  Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
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
 *  Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
=======
 *  Copyright (c) 2004, 2006 Apple Computer, Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
 *
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

unit AXAttributeConstants;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
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
uses MacTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Attributes                                                                           }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{
	Quick reference:
	
	// informational attributes
	kAXRoleAttribute
	kAXSubroleAttribute
	kAXRoleDescriptionAttribute
	kAXTitleAttribute
	kAXDescriptionAttribute
	kAXHelpAttribute
	
	// hierarchy or relationship attributes
	kAXParentAttribute
	kAXChildrenAttribute
	kAXSelectedChildrenAttribute
	kAXVisibleChildrenAttribute
	kAXWindowAttribute
	kAXTopLevelUIElementAttribute
	kAXTitleUIElementAttribute
	kAXServesAsTitleForUIElementsAttribute
	kAXLinkedUIElementsAttribute
	
	// visual state attributes
	kAXEnabledAttribute
	kAXFocusedAttribute
	kAXPositionAttribute
	kAXSizeAttribute
	
	// value attributes
	kAXValueAttribute
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    kAXValueDescriptionAttribute
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    kAXValueDescriptionAttribute
>>>>>>> origin/fixes_2.4
	kAXMinValueAttribute
	kAXMaxValueAttribute
	kAXValueIncrementAttribute
	kAXValueWrapsAttribute
	kAXAllowedValuesAttribute
	
	// text-specific attributes
	kAXSelectedTextAttribute
	kAXSelectedTextRangeAttribute
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    kAXSelectedTextRangesAttribute
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    kAXSelectedTextRangesAttribute
>>>>>>> origin/fixes_2.4
	kAXVisibleCharacterRangeAttribute
	kAXNumberOfCharactersAttribute
	kAXSharedTextUIElementsAttribute
	kAXSharedCharacterRangeAttribute
	
	// window, sheet, or drawer-specific attributes
	kAXMainAttribute
	kAXMinimizedAttribute
	kAXCloseButtonAttribute
	kAXZoomButtonAttribute
	kAXMinimizeButtonAttribute
	kAXToolbarButtonAttribute
	kAXProxyAttribute
	kAXGrowAreaAttribute
	kAXModalAttribute
	kAXDefaultButtonAttribute
	kAXCancelButtonAttribute
	
	// menu or menu item-specific attributes
	kAXMenuItemCmdCharAttribute
	kAXMenuItemCmdVirtualKeyAttribute
	kAXMenuItemCmdGlyphAttribute
	kAXMenuItemCmdModifiersAttribute
	kAXMenuItemMarkCharAttribute
	kAXMenuItemPrimaryUIElementAttribute
	
	// application element-specific attributes
	kAXMenuBarAttribute
	kAXWindowsAttribute
	kAXFrontmostAttribute
	kAXHiddenAttribute
	kAXMainWindowAttribute
	kAXFocusedWindowAttribute
	kAXFocusedUIElementAttribute
<<<<<<< HEAD
<<<<<<< HEAD
	kAXExtrasMenuBarAttribute
 
=======
	
>>>>>>> graemeg/fixes_2_2
=======
	
>>>>>>> origin/fixes_2_2
	// date/time-specific attributes
	kAXHourFieldAttribute
	kAXMinuteFieldAttribute
	kAXSecondFieldAttribute
	kAXAMPMFieldAttribute
	kAXDayFieldAttribute
	kAXMonthFieldAttribute
	kAXYearFieldAttribute
	
	// table, outline, or browser-specific attributes
	kAXRowsAttribute
	kAXVisibleRowsAttribute
	kAXSelectedRowsAttribute
	kAXColumnsAttribute
	kAXVisibleColumnsAttribute
	kAXSelectedColumnsAttribute
	kAXSortDirectionAttribute
	kAXColumnHeaderUIElementsAttribute
	kAXIndexAttribute
	kAXDisclosingAttribute
	kAXDisclosedRowsAttribute
	kAXDisclosedByRowAttribute
	
	// matte-specific attributes
	kAXMatteHoleAttribute
	kAXMatteContentUIElementAttribute
	
	// ruler-specific attributes
	kAXMarkerUIElementsAttribute
	kAXUnitsAttribute
	kAXUnitDescriptionAttribute
	kAXMarkerTypeAttribute
	kAXMarkerTypeDescriptionAttribute
	
	// miscellaneous or role-specific attributes
	kAXHorizontalScrollBarAttribute
	kAXVerticalScrollBarAttribute
	kAXOrientationAttribute
	kAXHeaderAttribute
	kAXEditedAttribute
	kAXTabsAttribute
	kAXOverflowButtonAttribute
	kAXFilenameAttribute
	kAXExpandedAttribute
	kAXSelectedAttribute
	kAXSplittersAttribute
	kAXContentsAttribute
	kAXNextContentsAttribute
	kAXPreviousContentsAttribute
	kAXDocumentAttribute
	kAXIncrementorAttribute
	kAXDecrementButtonAttribute
	kAXIncrementButtonAttribute
	kAXColumnTitleAttribute
	kAXURLAttribute
	kAXLabelUIElementsAttribute
	kAXLabelValueAttribute
	kAXShownMenuUIElementAttribute
	kAXIsApplicationRunningAttribute
	kAXFocusedApplicationAttribute
}


{
	kAXRoleAttribute
	
	Identifies the basic type of an element.
	
	Value: A CFStringRef of one of the role strings defined in this header, or a new
	role string of your own invention. The string should not be localized, and it does
	not need to be human-readable. Instead of inventing new role strings, see if a
	custom element can be identified by an existing role string and a new subrole. See
	kAXSubroleAttribute.
	
	Writable? No.
	
	Required for all elements. Even in the worst case scenario where an element cannot
	figure out what its basic type is, it can still supply the value kAXUnknownRole.
	
	Carbon Accessorization Notes: If your HIObjectClass or Carbon Event handler provides
	the kAXRoleAttribute, it must also provide the kAXRoleDescriptionAttribute.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRoleAttribute CFSTRP('AXRole')}
{$endc}


{
	kAXSubroleAttribute
	
	More specifically identifies the type of an element beyond the basic type provided
	by kAXRoleAttribute.
	
	Value: A CFStringRef of one of the subrole strings defined in this header, or a new
	subrole string of your own invention. The string should not be localized, and it does
	not need to be human-readable.
	
	Writable? No.
	
	Required only when an element's kAXRoleAttribute alone doesn't provide an assistive
	application with enough information to convey the meaning of this element to the user.
	
	An example element that offers the kAXSubroleAttribute is a window's close box. Its
	kAXRoleAttribute value is kAXButtonRole and its kAXSubroleAttribute is
	kAXCloseButtonSubrole. It is of role kAXButtonRole because it offers no additional
	actions or attributes above and beyond what other kAXButtonRole elements provide; it
	was given a subrole in order to allow an assistive app to communicate the close box's
	semantic difference to the user.
	
	Carbon Accessorization Notes: If your HIObjectClass or Carbon Event handler provides
	the kAXSubroleAttribute, it must also provide the kAXRoleDescriptionAttribute.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSubroleAttribute CFSTRP('AXSubrole')}
{$endc}


{
	kAXRoleDescriptionAttribute
	
	A localized, human-readable string that an assistive application can present to the user
	as an explanation of an element's basic type or purpose. Examples would be "push button"
	or "secure text field". The string's language should match the language of the app that
	the element lives within. The string should be all lower-case and contain no punctuation.
	
	Two elements with the same kAXRoleAttribute and kAXSubroleAttribute should have the
	same kAXRoleDescriptionAttribute.
	
	Value: A localized, human-readable CFStringRef.
	
	Writable? No.
	
	Required for all elements. Even in the worst case scenario where an element cannot
	figure out what its basic type is, it can still supply the value "unknown".
	
	Carbon Accessorization Notes: The HIObjectClass or Carbon Event handler that provides
	the AXRole and/or AXSubrole for an element is the one that must also handle the
	AXRoleDescription attribute. If an HIObjectClass or Carbon Event handler does not
	provide either the AXRole or AXSubrole, it must not provide the AXRoleDescription.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRoleDescriptionAttribute CFSTRP('AXRoleDescription')}
{$endc}


{
	kAXHelpAttribute
	
	A localized, human-readable CFStringRef that offers help content for an element.
	This is often the same information that would be provided in a help tag for the element.
	
	Value: A localized, human-readable CFStringRef.
	
	Writable? No.
	
	Recommended for any element that has help data available.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXHelpAttribute CFSTRP('AXHelp')}
{$endc}


{
	kAXTitleAttribute
	
	The localized, human-readable string that is displayed as part of the element's
	normal visual interface. For example, a OK button's kAXTitleElement is the string
	"OK", and a menu item's kAXTitleElement is the text of the menu item.
	
	Value: A localized, human-readable CFStringRef.
	
	Writable? No.
	
	Required if the element draws a string as part of its normal visual interface.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXTitleAttribute CFSTRP('AXTitle')}
{$endc}


{
	kAXValueAttribute
	
	A catch-all attribute that represents a user modifiable setting of an element. For
	example, the contents of an editable text field, the position of a scroll bar thumb,
	and whether a check box is checked are all communicated by the kAXValueAttribute of
	their respective elements.
	
	Value: Varies, but will always be the same type for a given kind of element. Each
	role that offers kAXValueAttribute will specify the type of data that will be used
	for its value.
	
	Writable? Generally yes. However, it does not need to be writable if some other form
	of direct manipulation is more appropriate for causing a value change. For example,
	a kAXScrollBar's kAXValueAttribute is writable because it allows an efficient way
	for the user to get to a specific position in the element being scrolled. By
	contrast, a kAXCheckBox's kAXValueAttribute is not settable because underlying
	functionality of the check box widget relies on it being clicked on; therefore, it
	changes its own kAXValueAttribute appropriately in response to the kAXPressAction.
	
	Required for many user manipulatable elements, or those whose value state conveys
	important information.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXValueAttribute CFSTRP('AXValue')}
{$endc}


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
    kAXValueDescriptionAttribute
    
    Used to supplement kAXValueAttribute.  This attribute returns a string description that best 
    describes the current value stored in kAXValueAttribute.  This is useful for things like
    slider where the numeric value in kAXValueAttribute does not always convey enough information
    about the adjustment made on the slider.  As an example, a color slider that adjusts thru various  
    colors cannot be well-described by the numeric value in existing AXValueAttribute.  This is where 
    the kAXValueDescriptionAttribute comes in handy.  In this example, the developer can provide the  
    color information using this attribute.       
    
    Value: A localized, human-readable CFStringRef.
	
	Writable? No.
    
    Recommended for elements that support kAXValueAttribute.

}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXValueDescriptionAttribute CFSTRP('AXValueDescription')}
{$endc}


{
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kAXMinValueAttribute
	
	Only used in conjunction with kAXValueAttribute and kAXMaxValueAttribute, this
	attribute represents the minimum value that an element can display. This is useful
	for things like sliders and scroll bars, where the user needs to have an understanding
	of how much the kAXValueAttribute can vary.
	
	Value: Same data type as the element's kAXValueAttribute.
	
	Writable? No.
	
	Required for many user maniipulatable elements. See kAXValueAttribute for more
	details.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMinValueAttribute CFSTRP('AXMinValue')}
{$endc}


{
	kAXMaxValueAttribute
	
	Only used in conjunction with kAXValueAttribute and kAXMinValueAttribute, this
	attribute represents the maximum value that an element can display. This is useful
	for things like sliders and scroll bars, where the user needs to have an understanding
	of how much the kAXValueAttribute can vary.
	
	Value: Same data type as the element's kAXValueAttribute.
	
	Writable? No.
	
	Required for many user maniipulatable elements. See kAXValueAttribute for more
	details.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMaxValueAttribute CFSTRP('AXMaxValue')}
{$endc}


{
	kAXValueIncrementAttribute
	
	Only used in conjunction with kAXValueAttribute, this attribute represents the amount
	a value will change in one action on the given element. In particular, it is used on
	elements of role kAXIncrementorRole in order to give the user an idea of how much its
	value will change with a single click on the up or down arrow.
	
	Value: Same data type as the element's kAXValueAttribute.
	
	Writable? No.
	
	Recommended for kAXIncrementorRole and other similar elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXValueIncrementAttribute CFSTRP('AXValueIncrement')}
{$endc}


{
	kAXAllowedValuesAttribute
	
	An array of the allowed values for a slider or other widget that displays
	a large value range, but which can only be set to a small subset of values
	within that range.
	
	Value: A CFArrayRef of whatever type the element uses for its kAXValueAttribute.
	
	Writable? No.
	
	Recommended for sliders or other elements that can only be set to a small
	set of values.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXAllowedValuesAttribute CFSTRP('AXAllowedValues')}
{$endc}


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kAXPlaceholderValueAttribute
	
	The value of placeholder text as found in a text field.
	
	Value: A CFStringRef.
	
	Writable? No.
	
	Recommended for text fields and other elements that have a placeholder value.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXPlaceholderValueAttribute CFSTRP('AXPlaceholderValue')}
{$endc}


{
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kAXEnabledAttribute
	
	Indicates whether the element can be interacted with by the user. For example,
	a disabled push button's kAXEnabledAttribute will be false.
	
	Value: A CFBooleanRef. True means enabled, false means disabled.
	
	Writable? No.
	
	Required for all views, menus, and menu items. Not required for windows.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXEnabledAttribute CFSTRP('AXEnabled')}
{$endc}


{
	kAXFocusedAttribute
	
	Indicates whether the element is the current keyboard focus. It should be writable
	for any element that can accept keyboard focus, though you can only set the value
	of kAXFocusedAttribute to true. You cannot unfocus an element by setting the value
	to false. Only one element in a window's entire accessibility hierarchy should be
	marked as focused.
	
	Value: A CFBooleanRef. True means focused, false means not focused.
	
	Writable? Yes, for any focusable element. No in all other cases.
	
	Required for any focusable element. Not required for other elements, though it is
	often offered for non-focusable elements in a read-only fashion.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXFocusedAttribute CFSTRP('AXFocused')}
{$endc}


{
	kAXParentAttribute
	
	Indicates the element's container element in the visual element hierarchy. A push
	button's kAXParentElement might be a window element or a group. A sheet's
	kAXParentElement will be a window element. A window's kAXParentElement will be the
	application element. A menu item's kAXParentElement will be a menu element.
	
	Value: An AXUIElementRef.
	
	Writable? No.
	
	Required for every element except the application. Everything else in the visual
	element hierarchy must have a parent.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXParentAttribute CFSTRP('AXParent')}
{$endc}


{
	kAXChildrenAttribute
	
	Indicates the sub elements of a given element in the visual element hierarchy. A tab
	group's kAXChildrenAttribute is an array of tab radio button elements. A window's
	kAXChildrenAttribute is an array of the first-order views elements within the window.
	A menu's kAXChildrenAttribute is an array of the menu item elements.
	
	A given element may only be in the child array of one other element. If an element is
	in the child array of some other element, the element's kAXParentAttribute must be
	the other element.
	
	Value: A CFArrayRef of AXUIElementRefs.
	
	Writable? No.
	
	Required for elements that contain sub elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXChildrenAttribute CFSTRP('AXChildren')}
{$endc}


{
	kAXSelectedChildrenAttribute
	
	Indicates the selected sub elements of a given element in the visual element hierarchy.
	This is a the subset of the element's kAXChildrenAttribute that are selected. This is
	commonly used in lists so an assistive app can know which list item are selected.
	
	Value: A CFArrayRef of AXUIElementRefs.
	
	Writable? Only if there is no other way to manipulate the set of selected elements via
	accessibilty attributes or actions. Even if other ways exist, this attribute can be
	writable as a convenience to assistive applications and their users. If
	kAXSelectedChildrenAttribute is writable, a write request with a value of an empty
	array should deselect all selected children.
	
	Required for elements that contain selectable sub elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSelectedChildrenAttribute CFSTRP('AXSelectedChildren')}
{$endc}


{
	kAXVisibleChildrenAttribute
	
	Indicates the visible sub elements of a given element in the visual element hierarchy.
	This is a the subset of the element's kAXChildrenAttribute that a sighted user can
	see on the screen. In a list element, kAXVisibleChildrenAttribute would be an array
	of child elements that are currently scrolled into view.
	
	Value: A CFArrayRef of AXUIElementRefs.
	
	Writable? No.
	
	Recommended for elements whose child elements can be occluded or scrolled out of view.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVisibleChildrenAttribute CFSTRP('AXVisibleChildren')}
{$endc}


{
	kAXWindowAttribute
	
	A short cut for traversing an element's parent hierarchy until an element of role
	kAXWindowRole is found. Note that the value for kAXWindowAttribute should not be
	an element of role kAXSheetRole or kAXDrawerRole; instead, the value should be the
	element of kAXWindowRole that the sheet or drawer is attached to.
	
	Value: an AXUIElementRef of role kAXWindowRole.
	
	Writable? No.
	
	Required for any element that has an element of role kAXWindowRole somewhere
	in its parent chain.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXWindowAttribute CFSTRP('AXWindow')}
{$endc}


{
	kAXTopLevelUIElementAttribute
	
	This is very much like the kAXWindowAttribute, except that the value of this
	attribute can be an element with role kAXSheetRole or kAXDrawerRole. It is
	a short cut for traversing an element's parent hierarchy until an element of
	role kAXWindowRole, kAXSheetRole, or kAXDrawerRole is found.
	
	Value: An AXUIElementRef of role kAXWindowRole, kAXSheetRole, or kAXDrawerRole.
	
	Writable? No.
	
	Required for any element that has an appropriate element somewhere in its
	parent chain.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXTopLevelUIElementAttribute CFSTRP('AXTopLevelUIElement')}
{$endc}


{
	kAXPositionAttribute
	
	The global screen position of the top-left corner of an element.
	
	Value: An AXValueRef with type kAXValueCGPointType. 0,0 is the top-left
	corner of the screen that displays the menu bar. The value of the horizontal
	axis increases to the right. The value of the vertical axis increases
	downward. Units are pixels.
	
	Writable? Generally no. However, some elements that can be moved by the user
	through direct manipulation (like windows) should offer a writable position
	attribute.
	
	Required for all elements that are visible on the screen, which is virtually
	all elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXPositionAttribute CFSTRP('AXPosition')}
{$endc}


{
	kAXSizeAttribute
	
	The vertical and horizontal dimensions of the element.
	
	Value: An AXValueRef with type kAXValueCGSizeType. Units are pixels.
	
	Writable? Generally no. However, some elements that can be resized by the user
	through direct manipulation (like windows) should offer a writable size attribute.
	
	Required for all elements that are visible on the screen, which is virtually
	all elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSizeAttribute CFSTRP('AXSize')}
{$endc}


{
	kAXOrientationAttribute
	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	An indication of whether an element is drawn and/or interacted with in a
=======
	An indiciation of whether an element is drawn and/or interacted with in a
>>>>>>> graemeg/fixes_2_2
=======
	An indiciation of whether an element is drawn and/or interacted with in a
>>>>>>> origin/fixes_2_2
=======
	An indication of whether an element is drawn and/or interacted with in a
>>>>>>> origin/fixes_2.4
	vertical or horizontal manner. Elements such as scroll bars and sliders offer
	the kAXOrientationAttribute.
	
	Value: kAXHorizontalOrientationValue, kAXVerticalOrientationValue, or rarely
	kAXUnknownOrientationValue.
	
	Writable? No.
	
	Required for scroll bars, sliders, or other elements whose semantic or
	associative meaning changes based on their orientation.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXOrientationAttribute CFSTRP('AXOrientation')}
{$endc}


{
	kAXDescriptionAttribute
	
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	A localized, human-readable string that indicates an element's purpose in a way
	that is slightly more specific than the kAXRoleDescriptionAttribute, but which
	is less wordy than the kAXHelpAttribute. Typically, the description should be
	an adjective or short phrase that describes the element's usage. For example,
	the description of a slider in a font panel might be "font size". The string
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	An indication of an element's purpose, in a way that is slightly more specific
	than the kAXRoleDescriptionAttribute, but which is less wordy than the
	kAXHelpAttribute. In English, the description should typically be a concatenation
	of a usage adjective with the element's role description. For example, the
	description of a slider in a font panel might be "font size slider". The string
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	should be all lower-case and contain no punctuation.
	
	Value: A localized, human-readable CFStringRef.
	
	Writable? No.
	
	Recommended for all elements because it gives the user a concise indication of
	an element's purpose.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDescriptionAttribute CFSTRP('AXDescription')}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDescription CFSTRP('AXDescription')}
{$endc} // old name
=======
=======
>>>>>>> origin/fixes_2_2
// old name
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDescription CFSTRP('AXDescription')}
{$endc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{
	kAXSelectedTextAttribute
	
	The selected text of an editable text element.
	
	Value: A CFStringRef with the currently selected text of the element.
	
	Writable? No.
	
	Required for all editable text elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSelectedTextAttribute CFSTRP('AXSelectedText')}
{$endc}


{
	kAXSelectedTextRangeAttribute
	
	The range of characters (not bytes) that defines the current selection of an
	editable text element.
	
	Value: An AXValueRef of type kAXValueCFRange.
	
	Writable? Yes.
	
	Required for all editable text elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSelectedTextRangeAttribute CFSTRP('AXSelectedTextRange')}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
	kAXSelectedTextRangesAttribute
	
	An array of noncontiguous ranges of characters (not bytes) that defines the current selections of an
	editable text element.  
	
	Value: A CFArrayRef of kAXValueCFRanges.
	
	Writable? Yes.
	
	Recommended for text elements that support noncontiguous selections.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSelectedTextRangesAttribute CFSTRP('AXSelectedTextRanges')}
{$endc}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
	kAXVisibleCharacterRangeAttribute
	
	The range of characters (not bytes) that are scrolled into view in the editable
	text element.
	
	Value: An AXValueRef of type kAXValueCFRange.
	
	Writable? No.
	
	Required for elements of role kAXTextAreaRole. Not required for any other
	elements, including those of role kAXTextFieldRole.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVisibleCharacterRangeAttribute CFSTRP('AXVisibleCharacterRange')}
{$endc}


{
	kAXNumberOfCharactersAttribute
	
	The total number of characters (not bytes) in an editable text element.
	
	Value: CFNumberRef
	
	Writable? No.
	
	Required for editable text elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXNumberOfCharactersAttribute CFSTRP('AXNumberOfCharacters')}
{$endc}


{
	kAXSharedTextUIElementsAttribute
	
	Value: CFArrayRef of AXUIElementRefs
	
	Writable? No.
	
	Optional?
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSharedTextUIElementsAttribute CFSTRP('AXSharedTextUIElements')}
{$endc}


{
	kAXSharedCharacterRangeAttribute
	
	Value: AXValueRef of type kAXValueCFRangeType
	
	Writable? No.
	
	Optional?
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSharedCharacterRangeAttribute CFSTRP('AXSharedCharacterRange')}
{$endc}

{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXInsertionPointLineNumberAttribute CFSTRP('AXInsertionPointLineNumber')}
{$endc}

{
	kAXMainAttribute
	
	Whether a window is the main document window of an application. For an active
	app, the main window is the single active document window. For an inactive app,
	the main window is the single document window which would be active if the app
	were active. Main does not necessarily imply that the window has key focus.
	
	Value: A CFBooleanRef. True means the window is main. False means it is not.
	
	Writable? Yes.
	
	Required for all window elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMainAttribute CFSTRP('AXMain')}
{$endc}


{
	kAXMinimizedAttribute
	
	Whether a window is currently minimized to the dock.
	
	Value: A CFBooleanRef. True means minimized.
	
	Writable? Yes.
	
	Required for all window elements that can be minimized.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMinimizedAttribute CFSTRP('AXMinimized')}
{$endc}


{
	kAXCloseButtonAttribute
	
	A convenience attribute so assistive apps can quickly access a window's close
	button element.
	
	Value: An AXUIElementRef of the window's close button element.
	
	Writable? No.
	
	Required for all window elements that have a close button.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXCloseButtonAttribute CFSTRP('AXCloseButton')}
{$endc}


{
	kAXZoomButtonAttribute
	
	A convenience attribute so assistive apps can quickly access a window's zoom
	button element.
	
	Value: An AXUIElementRef of the window's zoom button element.
	
	Writable? No.
	
	Required for all window elements that have a zoom button.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXZoomButtonAttribute CFSTRP('AXZoomButton')}
{$endc}


{
	kAXMinimizeButtonAttribute
	
	A convenience attribute so assistive apps can quickly access a window's minimize
	button element.
	
	Value: An AXUIElementRef of the window's minimize button element.
	
	Writable? No.
	
	Required for all window elements that have a minimize button.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMinimizeButtonAttribute CFSTRP('AXMinimizeButton')}
{$endc}


{
	kAXToolbarButtonAttribute
	
	A convenience attribute so assistive apps can quickly access a window's toolbar
	button element.
	
	Value: An AXUIElementRef of the window's toolbar button element.
	
	Writable? No.
	
	Required for all window elements that have a toolbar button.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXToolbarButtonAttribute CFSTRP('AXToolbarButton')}
{$endc}


{
<<<<<<< HEAD
<<<<<<< HEAD
 kAXFullScreenButtonAttribute
 
 A convenience attribute so assistive apps can quickly access a window's full screen
 button element.
 
 Value: An AXUIElementRef of the window's full screen button element.
 
 Writable? No.
 
 Required for all window elements that have a full screen button.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXFullScreenButtonAttribute CFSTRP('AXFullScreenButton')}
{$endc}


{
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	kAXProxyAttribute
	
	A convenience attribute so assistive apps can quickly access a window's document
	proxy element.
	
	Value: An AXUIElementRef of the window's document proxy element.
	
	Writable? No.
	
	Required for all window elements that have a document proxy.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXProxyAttribute CFSTRP('AXProxy')}
{$endc}


{
	kAXGrowAreaAttribute
	
	A convenience attribute so assistive apps can quickly access a window's grow
	area element.
	
	Value: An AXUIElementRef of the window's grow area element.
	
	Writable? No.
	
	Required for all window elements that have a grow area.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXGrowAreaAttribute CFSTRP('AXGrowArea')}
{$endc}


{
	kAXModalAttribute
	
	Whether a window is modal.
	
	Value: A CFBooleanRef. True means the window is modal.
	
	Writable? No.
	
	Required for all window elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXModalAttribute CFSTRP('AXModal')}
{$endc}


{
	kAXDefaultButtonAttribute
	
	A convenience attribute so assistive apps can quickly access a window's default
	button element, if any.
	
	Value: An AXUIElementRef of the window's default button element.
	
	Writable? No.
	
	Required for all window elements that have a default button.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDefaultButtonAttribute CFSTRP('AXDefaultButton')}
{$endc}


{
	kAXCancelButtonAttribute
	
	A convenience attribute so assistive apps can quickly access a window's cancel
	button element, if any.
	
	Value: An AXUIElementRef of the window's cancel button element.
	
	Writable? No.
	
	Required for all window elements that have a cancel button.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXCancelButtonAttribute CFSTRP('AXCancelButton')}
{$endc}

// menu-specific attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMenuItemCmdCharAttribute CFSTRP('AXMenuItemCmdChar')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMenuItemCmdVirtualKeyAttribute CFSTRP('AXMenuItemCmdVirtualKey')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMenuItemCmdGlyphAttribute CFSTRP('AXMenuItemCmdGlyph')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMenuItemCmdModifiersAttribute CFSTRP('AXMenuItemCmdModifiers')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMenuItemMarkCharAttribute CFSTRP('AXMenuItemMarkChar')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMenuItemPrimaryUIElementAttribute CFSTRP('AXMenuItemPrimaryUIElement')}
{$endc}

// application-specific attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMenuBarAttribute CFSTRP('AXMenuBar')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXWindowsAttribute CFSTRP('AXWindows')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXFrontmostAttribute CFSTRP('AXFrontmost')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXHiddenAttribute CFSTRP('AXHidden')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMainWindowAttribute CFSTRP('AXMainWindow')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXFocusedWindowAttribute CFSTRP('AXFocusedWindow')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXFocusedUIElementAttribute CFSTRP('AXFocusedUIElement')}
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} 
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXExtrasMenuBarAttribute CFSTRP('AXExtrasMenuBar')}
{$endc}

=======
{$endc}


>>>>>>> graemeg/fixes_2_2
=======
{$endc}


>>>>>>> origin/fixes_2_2
{
	kAXHeaderAttribute
	
	A convenience attribute whose value is an element that is a header for another
	element. For example, an outline element has a header attribute whose value is
	a element of role AXGroup that contains the header buttons for each column.
	Used for things like tables, outlines, columns, etc.
	
	Value: An AXUIElementRef whose role varies.
	
	Writable? No.
	
	Recommended for elements that have header elements contained within them that an
	assistive application might want convenient access to.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXHeaderAttribute CFSTRP('AXHeader')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXEditedAttribute CFSTRP('AXEdited')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXValueWrapsAttribute CFSTRP('AXValueWraps')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXTabsAttribute CFSTRP('AXTabs')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXTitleUIElementAttribute CFSTRP('AXTitleUIElement')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXHorizontalScrollBarAttribute CFSTRP('AXHorizontalScrollBar')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVerticalScrollBarAttribute CFSTRP('AXVerticalScrollBar')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXOverflowButtonAttribute CFSTRP('AXOverflowButton')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXFilenameAttribute CFSTRP('AXFilename')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXExpandedAttribute CFSTRP('AXExpanded')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSelectedAttribute CFSTRP('AXSelected')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSplittersAttribute CFSTRP('AXSplitters')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXNextContentsAttribute CFSTRP('AXNextContents')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDocumentAttribute CFSTRP('AXDocument')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDecrementButtonAttribute CFSTRP('AXDecrementButton')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXIncrementButtonAttribute CFSTRP('AXIncrementButton')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXPreviousContentsAttribute CFSTRP('AXPreviousContents')}
{$endc}


{
	kAXContentsAttribute
	
	A convenience attribute so assistive apps can find interesting child elements
	of a given element, while at the same time avoiding non-interesting child
	elements. For example, the contents of a scroll area are the children that get
	scrolled, and not the horizontal and/or vertical scroll bars. The contents of
	a tab group does not include the tabs themselves.
	
	Value: A CFArrayRef of AXUIElementRefs.
	
	Writable? No.
	
	Recommended for elements that have children that act upon or are separate from
	other children.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXContentsAttribute CFSTRP('AXContents')}
{$endc}


{
	kAXIncrementorAttribute
	
	Convenience attribute that yields the incrementor of a time field or date
	field element.
	
	Value: A AXUIElementRef of role kAXIncrementorRole.
	
	Writable? No.
	
	Required for time field and date field elements that display an incrementor.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXIncrementorAttribute CFSTRP('AXIncrementor')}
{$endc}


{
	kAXHourFieldAttribute
	
	Convenience attribute that yields the hour field of a time field element.
	
	Value: A AXUIElementRef of role kAXTextFieldRole that is used to edit the
	hours in a time field element.
	
	Writable? No.
	
	Required for time field elements that display hours.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXHourFieldAttribute CFSTRP('AXHourField')}
{$endc}


{
	kAXMinuteFieldAttribute
	
	Convenience attribute that yields the minute field of a time field element.
	
	Value: A AXUIElementRef of role kAXTextFieldRole that is used to edit the
	minutes in a time field element.
	
	Writable? No.
	
	Required for time field elements that display minutes.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMinuteFieldAttribute CFSTRP('AXMinuteField')}
{$endc}


{
	kAXSecondFieldAttribute
	
	Convenience attribute that yields the seconds field of a time field element.
	
	Value: A AXUIElementRef of role kAXTextFieldRole that is used to edit the
	seconds in a time field element.
	
	Writable? No.
	
	Required for time field elements that display seconds.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSecondFieldAttribute CFSTRP('AXSecondField')}
{$endc}


{
	kAXAMPMFieldAttribute
	
	Convenience attribute that yields the AM/PM field of a time field element.
	
	Value: A AXUIElementRef of role kAXTextFieldRole that is used to edit the
	AM/PM setting in a time field element.
	
	Writable? No.
	
	Required for time field elements that displays an AM/PM setting.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXAMPMFieldAttribute CFSTRP('AXAMPMField')}
{$endc}


{
	kAXDayFieldAttribute
	
	Convenience attribute that yields the day field of a date field element.
	
	Value: A AXUIElementRef of role kAXTextFieldRole that is used to edit the
	day in a date field element.
	
	Writable? No.
	
	Required for date field elements that display days.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDayFieldAttribute CFSTRP('AXDayField')}
{$endc}


{
	kAXMonthFieldAttribute
	
	Convenience attribute that yields the month field of a date field element.
	
	Value: A AXUIElementRef of role kAXTextFieldRole that is used to edit the
	month in a date field element.
	
	Writable? No.
	
	Required for date field elements that display months.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMonthFieldAttribute CFSTRP('AXMonthField')}
{$endc}


{
	kAXYearFieldAttribute
	
	Convenience attribute that yields the year field of a date field element.
	
	Value: A AXUIElementRef of role kAXTextFieldRole that is used to edit the
	year in a date field element.
	
	Writable? No.
	
	Required for date field elements that display years.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXYearFieldAttribute CFSTRP('AXYearField')}
{$endc}


{
	kAXColumnTitleAttribute
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXColumnTitleAttribute CFSTRP('AXColumnTitles')}
{$endc}


{
	kAXURLAttribute
	
	Value: A CFURLRef.
	
	Writable? No.
	
	Required for elements that represent a disk or network item.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXURLAttribute CFSTRP('AXURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXLabelUIElementsAttribute CFSTRP('AXLabelUIElements')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXLabelValueAttribute CFSTRP('AXLabelValue')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXShownMenuUIElementAttribute CFSTRP('AXShownMenuUIElement')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXServesAsTitleForUIElementsAttribute CFSTRP('AXServesAsTitleForUIElements')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXLinkedUIElementsAttribute CFSTRP('AXLinkedUIElements')}
{$endc}

// table/outline view attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRowsAttribute CFSTRP('AXRows')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVisibleRowsAttribute CFSTRP('AXVisibleRows')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSelectedRowsAttribute CFSTRP('AXSelectedRows')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXColumnsAttribute CFSTRP('AXColumns')}
{$endc}


{
	kAXVisibleColumnsAttribute
	
	Indicates the visible column sub-elements of a kAXBrowserRole element.
	This is the subset of a browser's kAXColumnsAttribute where each column in the
	array is one that is currently scrolled into view within the browser. It does
	not include any columns that are currently scrolled out of view.
	
	Value: A CFArrayRef of AXUIElementRefs representing the columns of a browser.
	The columns will be grandchild elements of the browser, and will generally be
	of role kAXScrollArea.
	
	Writable? No.
	
	Required for all browser elements.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVisibleColumnsAttribute CFSTRP('AXVisibleColumns')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSelectedColumnsAttribute CFSTRP('AXSelectedColumns')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSortDirectionAttribute CFSTRP('AXSortDirection')}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXColumnHeaderUIElementsAttribute CFSTRP('AXColumnHeaderUIElements')}
{$endc}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXColumnHeaderUIElementsAttribute CFSTRP('AXColumnHeaderUIElements')}
{$endc}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

// row/column attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXIndexAttribute CFSTRP('AXIndex')}
{$endc}

// outline attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDisclosingAttribute CFSTRP('AXDisclosing')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDisclosedRowsAttribute CFSTRP('AXDisclosedRows')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDisclosedByRowAttribute CFSTRP('AXDisclosedByRow')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXDisclosureLevelAttribute CFSTRP('AXDisclosureLevel')}
{$endc}

// matte attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMatteHoleAttribute CFSTRP('AXMatteHole')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMatteContentUIElementAttribute CFSTRP('AXMatteContentUIElement')}
{$endc}

// ruler attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMarkerUIElementsAttribute CFSTRP('AXMarkerUIElements')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXUnitsAttribute CFSTRP('AXUnits')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXUnitDescriptionAttribute CFSTRP('AXUnitDescription')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMarkerTypeAttribute CFSTRP('AXMarkerType')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXMarkerTypeDescriptionAttribute CFSTRP('AXMarkerTypeDescription')}
{$endc}

// Dock attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXIsApplicationRunningAttribute CFSTRP('AXIsApplicationRunning')}
{$endc}

// search field attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSearchButtonAttribute CFSTRP('AXSearchButton')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXClearButtonAttribute CFSTRP('AXClearButton')}
{$endc}

// system-wide attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXFocusedApplicationAttribute CFSTRP('AXFocusedApplication')}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
// grid attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRowCountAttribute CFSTRP('AXRowCount')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXColumnCountAttribute CFSTRP('AXColumnCount')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXOrderedByRowAttribute CFSTRP('AXOrderedByRow')}
{$endc}

// level indicator attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXWarningValueAttribute CFSTRP('AXWarningValue')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXCriticalValueAttribute CFSTRP('AXCriticalValue')}
{$endc}

// cell-based table attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXSelectedCellsAttribute CFSTRP('AXSelectedCells')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVisibleCellsAttribute CFSTRP('AXVisibleCells')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRowHeaderUIElementsAttribute CFSTRP('AXRowHeaderUIElements')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXColumnHeaderUIElementsAttribute CFSTRP('AXColumnHeaderUIElements')}
{$endc}

// cell attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRowIndexRangeAttribute CFSTRP('AXRowIndexRange')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXColumnIndexRangeAttribute CFSTRP('AXColumnIndexRange')}
{$endc}

// layout area attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXHorizontalUnitsAttribute CFSTRP('AXHorizontalUnits')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVerticalUnitsAttribute CFSTRP('AXVerticalUnits')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXHorizontalUnitDescriptionAttribute CFSTRP('AXHorizontalUnitDescription')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVerticalUnitDescriptionAttribute CFSTRP('AXVerticalUnitDescription')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXHandlesAttribute CFSTRP('AXHandles')}
{$endc}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
// obsolete/unknown attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXTextAttribute CFSTRP('AXText')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXVisibleTextAttribute CFSTRP('AXVisibleText')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXIsEditableAttribute CFSTRP('AXIsEditable')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXColumnTitlesAttribute CFSTRP('AXColumnTitles')}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
// UI element identification attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXIdentifierAttribute CFSTRP('AXIdentifier')}
{$endc}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Parameterized Attributes                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

// Text Suite Parameterized Attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXLineForIndexParameterizedAttribute CFSTRP('AXLineForIndex')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRangeForLineParameterizedAttribute CFSTRP('AXRangeForLine')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXStringForRangeParameterizedAttribute CFSTRP('AXStringForRange')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRangeForPositionParameterizedAttribute CFSTRP('AXRangeForPosition')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRangeForIndexParameterizedAttribute CFSTRP('AXRangeForIndex')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXBoundsForRangeParameterizedAttribute CFSTRP('AXBoundsForRange')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXRTFForRangeParameterizedAttribute CFSTRP('AXRTFForRange')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXAttributedStringForRangeParameterizedAttribute CFSTRP('AXAttributedStringForRange')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXStyleRangeForIndexParameterizedAttribute CFSTRP('AXStyleRangeForIndex')}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
// cell-based table parameterized attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXCellForColumnAndRowParameterizedAttribute CFSTRP('AXCellForColumnAndRow')}
{$endc}

// layout area parameterized attributes
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXLayoutPointForScreenPointParameterizedAttribute CFSTRP('AXLayoutPointForScreenPoint')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXLayoutSizeForScreenSizeParameterizedAttribute CFSTRP('AXLayoutSizeForScreenSize')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXScreenPointForLayoutPointParameterizedAttribute CFSTRP('AXScreenPointForLayoutPoint')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kAXScreenSizeForLayoutSizeParameterizedAttribute CFSTRP('AXScreenSizeForLayoutSize')}
{$endc}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
<<<<<<< HEAD

end.
{$endc} {not MACOSALLINCLUDE}
=======

end.
>>>>>>> graemeg/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2_2
=======

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
