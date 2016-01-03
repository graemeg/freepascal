{
     File:       HIToolbox/MacHelp.h
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Contains:   Carbon Help Manager Interfaces.
 
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
 
     Copyright:  © 1998-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     Contains:   Macintosh Help Package Interfaces.
=======
     Contains:   Carbon Help Manager Interfaces.
>>>>>>> origin/fixes_2.4
 
     Version:    HIToolbox-437~1
 
<<<<<<< HEAD
     Copyright:  © 1998-2005 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1998-2008 by Apple Computer, Inc., all rights reserved
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

unit MacHelp;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CFBase,QuickdrawTypes,TextEdit,Controls,Dialogs,Events,MacWindows,Menus,HIObject,HIView;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,CFBase,Quickdraw,TextEdit,Controls,Dialogs,Events,MacWindows,Menus;
=======
uses MacTypes,CFBase,QuickdrawTypes,TextEdit,Controls,Dialogs,Events,MacWindows,Menus,HIObject,HIView;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,Quickdraw,TextEdit,Controls,Dialogs,Events,MacWindows,Menus;

>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Help Manager constants, etc.                                                     }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
const
	kMacHelpVersion = $0003;

const
	kHMHelpMenuID = -16490; { Resource ID and menu ID of help menu }

type
	HMContentRequest = SInt16;
const
	kHMSupplyContent = 0;
	kHMDisposeContent = 1;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

{
 *  HMContentType
 *  
 *  Summary:
 *    These constants should be used in the contentType field of the
 *    HMHelpContent structure.
 }
<<<<<<< HEAD
type
	HMContentType = UInt32;
const
{
   * No help content is provided.
   }
	kHMNoContent = FourCharCode('none');

  {
   * Help content is a CFStringRef in the HMHelpContent.u.tagCFString
   * field. 
   * 
   * The string is automatically retained by the Help Manager when
   * provided to the HMSetControl/Window/MenuItemHelpContent APIs, and
   * automatically released when the control/window/menu is destroyed.
   * When this type of help content is provided by a
   * control/window/menu title/menu item help content proc, then the
   * string is not automatically released, but the content proc will be
   * called again later with the kHMDisposeContent message to allow
   * your application to release the string, if appropriate.
   }
	kHMCFStringContent = FourCharCode('cfst');

  {
   * Help content is a localized string loaded automatically by the
   * Help Manager using CFCopyLocalizedString. The key that identifies
   * the localized string should be placed in the
   * HMHelpContent.u.tagCFString field. 
   * 
   * The string is automatically retained by the Help Manager when
   * provided to the HMSetControl/Window/MenuItemHelpContent APIs, and
   * automatically released when the control/window/menu is destroyed.
   * When this type of help content is provided by a
   * control/window/menu title/menu item help content proc, then the
   * string is not automatically released, but the content proc will be
   * called again later with the kHMDisposeContent message to allow
   * your application to release the string, if appropriate.
   }
	kHMCFStringLocalizedContent = FourCharCode('cfsl');

  {
   * Help content is a Pascal string in the application's text encoding
   * in the HMHelpContent.u.tagString field.
   }
	kHMPascalStrContent = FourCharCode('pstr');

  {
   * Help content is text in the application's text encoding loaded
   * from a 'TEXT' resource. The 'TEXT' resource ID should be placed in
   * the HMHelpContent.u.tagTextRes field. This help content type is
   * available in Mac OS X 10.2 and later.
   }
	kHMTextResContent = FourCharCode('text');

  {
   * Help content is a TEHandle in the HMHelpContent.u.tagTEHandle.
   * Note that Mac OS X ignores all style information contained in the
   * TEHandle and only uses the text. This help content type is
   * available in Mac OS X 10.2 and later. It is not supported in
   * 64-bit mode. 
   * 
   * When this type of help content is provided to the
   * HMSetControl/Window/MenuItemHelpContent APIs, the TEHandle is
   * _not_ released when the control/window/menu is destroyed. When
   * this type of help content is provided by a control/window/menu
   * title/menu item help content proc, then the content proc will be
   * called again later with the kHMDisposeContent message to allow
   * your application to release the TEHandle, if appropriate.
   }
	kHMTEHandleContent = FourCharCode('txth');

  {
   * Help content is a Pascal string in the application's text encoding
   * loaded from a 'STR#' resource. The 'STR#' resource ID and string
   * index should be placed in the HMHelpContent.u.tagStringRes field.
   * This help content type is not supported in 64-bit mode.
   }
	kHMStringResContent = FourCharCode('str#');

  {
   * Help content is a Pascal string in the application's text encoding
   * loaded from a 'STR ' resource. The 'STR ' resource ID should be
   * placed in the HMHelpContent.u.tagStrRes field. This help content
   * type is not supported in 64-bit mode.
   }
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
type
	HMContentType = UInt32;
const
{
   * No help content is provided.
   }
	kHMNoContent = FourCharCode('none');

  {
   * Help content is a CFStringRef in the HMHelpContent.u.tagCFString
   * field. 
   * 
   * The string is automatically retained by the Help Manager when
   * provided to the HMSetControl/Window/MenuItemHelpContent APIs, and
   * automatically released when the control/window/menu is destroyed.
   * When this type of help content is provided by a
   * control/window/menu title/menu item help content proc, then the
   * string is not automatically released, but the content proc will be
   * called again later with the kHMDisposeContent message to allow
   * your application to release the string, if appropriate.
   }
	kHMCFStringContent = FourCharCode('cfst');

  {
   * Help content is a localized string loaded automatically by the
   * Help Manager using CFCopyLocalizedString. The key that identifies
   * the localized string should be placed in the
   * HMHelpContent.u.tagCFString field. 
   * 
   * The string is automatically retained by the Help Manager when
   * provided to the HMSetControl/Window/MenuItemHelpContent APIs, and
   * automatically released when the control/window/menu is destroyed.
   * When this type of help content is provided by a
   * control/window/menu title/menu item help content proc, then the
   * string is not automatically released, but the content proc will be
   * called again later with the kHMDisposeContent message to allow
   * your application to release the string, if appropriate.
   }
	kHMCFStringLocalizedContent = FourCharCode('cfsl');

  {
   * Help content is a Pascal string in the application's text encoding
   * in the HMHelpContent.u.tagString field.
   }
	kHMPascalStrContent = FourCharCode('pstr');

  {
   * Help content is text in the application's text encoding loaded
   * from a 'TEXT' resource. The 'TEXT' resource ID should be placed in
   * the HMHelpContent.u.tagTextRes field. This help content type is
   * available in Mac OS X 10.2 and later.
   }
	kHMTextResContent = FourCharCode('text');

  {
   * Help content is a TEHandle in the HMHelpContent.u.tagTEHandle.
   * Note that Mac OS X ignores all style information contained in the
   * TEHandle and only uses the text. This help content type is
   * available in Mac OS X 10.2 and later. It is not supported in
   * 64-bit mode. 
   * 
   * When this type of help content is provided to the
   * HMSetControl/Window/MenuItemHelpContent APIs, the TEHandle is
   * _not_ released when the control/window/menu is destroyed. When
   * this type of help content is provided by a control/window/menu
   * title/menu item help content proc, then the content proc will be
   * called again later with the kHMDisposeContent message to allow
   * your application to release the TEHandle, if appropriate.
   }
	kHMTEHandleContent = FourCharCode('txth');

  {
   * Help content is a Pascal string in the application's text encoding
   * loaded from a 'STR#' resource. The 'STR#' resource ID and string
   * index should be placed in the HMHelpContent.u.tagStringRes field.
   * This help content type is not supported in 64-bit mode.
   }
	kHMStringResContent = FourCharCode('str#');
<<<<<<< HEAD
	kHMTEHandleContent = FourCharCode('txth'); { Supported by CarbonLib and Jaguar and later}
	kHMTextResContent = FourCharCode('text'); { Supported by CarbonLib and Jaguar and later}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

  {
   * Help content is a Pascal string in the application's text encoding
   * loaded from a 'STR ' resource. The 'STR ' resource ID should be
   * placed in the HMHelpContent.u.tagStrRes field. This help content
   * type is not supported in 64-bit mode.
   }
>>>>>>> origin/fixes_2.4
	kHMStrResContent = FourCharCode('str ');


{
 *  HMTagDisplaySide
 *  
 *  Discussion:
 *    Help tag display locations relative to absolute hot rect
 }
type
	HMTagDisplaySide = SInt16;
const
{
   * System default location
   }
	kHMDefaultSide = 0;

  {
   * Above, aligned with left or right depending on system script
   }
	kHMOutsideTopScriptAligned = 1;

  {
   * To the left, centered vertically
   }
	kHMOutsideLeftCenterAligned = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

  {
   * Below, aligned with left or right depending on system script
   }
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kHMOutsideBottomScriptAligned = 3;

  {
   * To the right, centered vertically
   }
	kHMOutsideRightCenterAligned = 4;

  {
   * Above, aligned with left
   }
	kHMOutsideTopLeftAligned = 5;

  {
   * Above, aligned with right
   }
	kHMOutsideTopRightAligned = 6;

  {
   * To the left, aligned with top
   }
	kHMOutsideLeftTopAligned = 7;

  {
   * To the left, aligned with bottom
   }
	kHMOutsideLeftBottomAligned = 8;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * Below, aligned with left
=======
   * To the right, aligned with top
>>>>>>> graemeg/fixes_2_2
=======
   * To the right, aligned with top
>>>>>>> origin/fixes_2_2
=======
   * Below, aligned with left
>>>>>>> origin/fixes_2.4
   }
	kHMOutsideBottomLeftAligned = 9;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * Below, aligned with right
   }
	kHMOutsideBottomRightAligned = 10;

  {
   * To the right, aligned with top
   }
	kHMOutsideRightTopAligned = 11;

  {
   * To the right, aligned with bottom
   }
=======
=======
>>>>>>> origin/fixes_2_2
   * To the right, aligned with bottom
=======
   * Below, aligned with right
>>>>>>> origin/fixes_2.4
   }
	kHMOutsideBottomRightAligned = 10;

  {
   * To the right, aligned with top
   }
	kHMOutsideRightTopAligned = 11;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

  {
   * To the right, aligned with bottom
   }
>>>>>>> origin/fixes_2.4
	kHMOutsideRightBottomAligned = 12;

  {
   * Above, centered horizontally
   }
	kHMOutsideTopCenterAligned = 13;

  {
   * Below, centered horizontally
   }
	kHMOutsideBottomCenterAligned = 14;

  {
   * Inside, aligned with right, centered vertically
   }
	kHMInsideRightCenterAligned = 15;

  {
   * Inside, aligned with left, centered vertically
   }
	kHMInsideLeftCenterAligned = 16;

  {
   * Inside, aligned with bottom, centered horizontally
   }
	kHMInsideBottomCenterAligned = 17;

  {
   * Inside, aligned with top, centered horizontally
   }
	kHMInsideTopCenterAligned = 18;

  {
   * Inside, aligned with top and left
   }
	kHMInsideTopLeftCorner = 19;

  {
   * Inside, aligned with top and right
   }
	kHMInsideTopRightCorner = 20;

  {
   * Inside, aligned with bottom and left
   }
	kHMInsideBottomLeftCorner = 21;

  {
   * Inside, aligned with bottom and right
   }
	kHMInsideBottomRightCorner = 22;

  {
   * Centered vertically and horizontally
   }
	kHMAbsoluteCenterAligned = 23;

{ Obsoleted constants HMTagDisplaySides, use the new ones, please }
const
	kHMTopSide = kHMOutsideTopScriptAligned;
	kHMLeftSide = kHMOutsideLeftCenterAligned;
	kHMBottomSide = kHMOutsideBottomScriptAligned;
	kHMRightSide = kHMOutsideRightCenterAligned;
	kHMTopLeftCorner = kHMOutsideTopLeftAligned;
	kHMTopRightCorner = kHMOutsideTopRightAligned;
	kHMLeftTopCorner = kHMOutsideLeftTopAligned;
	kHMLeftBottomCorner = kHMOutsideLeftBottomAligned;
	kHMBottomLeftCorner = kHMOutsideBottomLeftAligned;
	kHMBottomRightCorner = kHMOutsideBottomRightAligned;
	kHMRightTopCorner = kHMOutsideRightTopAligned;
	kHMRightBottomCorner = kHMOutsideRightBottomAligned;

type
	HMContentProvidedType = SInt16;
const
	kHMContentProvided = 0;
	kHMContentNotProvided = 1;
	kHMContentNotProvidedDontPropagate = 2;

const
	kHMMinimumContentIndex = 0;    { first entry in HMHelpContentRec.content is the minimum content }
	kHMMaximumContentIndex = 1;     { second entry in HMHelpContentRec.content is the maximum content }

const
	errHMIllegalContentForMinimumState = -10980; { unrecognized content type for minimum content }
	errHMIllegalContentForMaximumState = -10981; { unrecognized content type for maximum content }

{ obsolete names; will be removed}
const
	kHMIllegalContentForMinimumState = errHMIllegalContentForMinimumState;

const
	kHelpTagEventHandlerTag = FourCharCode('hevt');

type
	HMStringResType = record
		hmmResID: SInt16;
		hmmIndex: SInt16;
	end;
type
	HMHelpContent = record
		contentType: HMContentType;
		case SInt16 of
		0: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			tagCFString: CFStringRef;          { CFStringRef}
			);
		1: (
			tagString: Str255;          { Pascal String}
			);
		2: (
			tagStringRes: HMStringResType;          { STR# resource ID and index}
			);
		3: (
			tagTEHandle: TEHandle;          { TextEdit handle (NOT SUPPORTED ON MAC OS X)}
			);
		4: (
			tagTextRes: SInt16;          { TEXT/styl resource ID (NOT SUPPORTED ON MAC OS X)}
			);
		5: (
			tagStrRes: SInt16;          { STR resource ID}
=======
=======
>>>>>>> origin/fixes_2_2
			tagCFString:		CFStringRef;							{  CFStringRef }
=======
			tagCFString: CFStringRef;          { CFStringRef}
>>>>>>> origin/fixes_2.4
			);
		1: (
			tagString: Str255;          { Pascal String}
			);
		2: (
			tagStringRes: HMStringResType;          { STR# resource ID and index}
			);
		3: (
			tagTEHandle: TEHandle;          { TextEdit handle (NOT SUPPORTED ON MAC OS X)}
			);
		4: (
			tagTextRes: SInt16;          { TEXT/styl resource ID (NOT SUPPORTED ON MAC OS X)}
			);
		5: (
<<<<<<< HEAD
			tagStrRes:			SInt16;									{  STR resource ID }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
			tagStrRes: SInt16;          { STR resource ID}
>>>>>>> origin/fixes_2.4
			);
	end;
type
	HMHelpContentRec = record
		version: SInt32;
		absHotRect: Rect;
		tagSide: HMTagDisplaySide;
		content: array [0..1] of HMHelpContent;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
>>>>>>> origin/fixes_2.4
	HMHelpContentPtr = ^HMHelpContentRec;

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Callback procs                                       }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ }
type
	HMControlContentProcPtr = function( inControl: ControlRef; inGlobalMouse: Point; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr ): OSStatus;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	HMWindowContentProcPtr = function( inWindow: WindowRef; inGlobalMouse: Point; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr ): OSStatus;
	HMMenuTitleContentProcPtr = function( inMenu: MenuRef; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr ): OSStatus;
	HMMenuItemContentProcPtr = function( const (*var*) inTrackingData: MenuTrackingData; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr ): OSStatus;
	HMControlContentUPP = HMControlContentProcPtr;
	HMWindowContentUPP = HMWindowContentProcPtr;
	HMMenuTitleContentUPP = HMMenuTitleContentProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type
=======
>>>>>>> origin/fixes_2.4
	HMWindowContentProcPtr = function( inWindow: WindowRef; inGlobalMouse: Point; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr ): OSStatus;
	HMMenuTitleContentProcPtr = function( inMenu: MenuRef; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr ): OSStatus;
	HMMenuItemContentProcPtr = function( const (*var*) inTrackingData: MenuTrackingData; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr ): OSStatus;
	HMControlContentUPP = HMControlContentProcPtr;
	HMWindowContentUPP = HMWindowContentProcPtr;
	HMMenuTitleContentUPP = HMMenuTitleContentProcPtr;
<<<<<<< HEAD
type
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	HMMenuItemContentUPP = HMMenuItemContentProcPtr;
{
 *  NewHMControlContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewHMControlContentUPP( userRoutine: HMControlContentProcPtr ): HMControlContentUPP; external name '_NewHMControlContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewHMWindowContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewHMWindowContentUPP( userRoutine: HMWindowContentProcPtr ): HMWindowContentUPP; external name '_NewHMWindowContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewHMMenuTitleContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewHMMenuTitleContentUPP( userRoutine: HMMenuTitleContentProcPtr ): HMMenuTitleContentUPP; external name '_NewHMMenuTitleContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewHMMenuItemContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewHMMenuItemContentUPP( userRoutine: HMMenuItemContentProcPtr ): HMMenuItemContentUPP; external name '_NewHMMenuItemContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeHMControlContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeHMControlContentUPP( userUPP: HMControlContentUPP ); external name '_DisposeHMControlContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeHMWindowContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeHMWindowContentUPP( userUPP: HMWindowContentUPP ); external name '_DisposeHMWindowContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeHMMenuTitleContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeHMMenuTitleContentUPP( userUPP: HMMenuTitleContentUPP ); external name '_DisposeHMMenuTitleContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeHMMenuItemContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeHMMenuItemContentUPP( userUPP: HMMenuItemContentUPP ); external name '_DisposeHMMenuItemContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeHMControlContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeHMControlContentUPP( inControl: ControlRef; inGlobalMouse: Point; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr; userUPP: HMControlContentUPP ): OSStatus; external name '_InvokeHMControlContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeHMWindowContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeHMWindowContentUPP( inWindow: WindowRef; inGlobalMouse: Point; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr; userUPP: HMWindowContentUPP ): OSStatus; external name '_InvokeHMWindowContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeHMMenuTitleContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeHMMenuTitleContentUPP( inMenu: MenuRef; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr; userUPP: HMMenuTitleContentUPP ): OSStatus; external name '_InvokeHMMenuTitleContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeHMMenuItemContentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeHMMenuItemContentUPP( const (*var*) inTrackingData: MenuTrackingData; inRequest: HMContentRequest; var outContentProvided: HMContentProvidedType; ioHelpContent: HMHelpContentPtr; userUPP: HMMenuItemContentUPP ): OSStatus; external name '_InvokeHMMenuItemContentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ API                                                                                      }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Help Menu }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  HMGetHelpMenu()
 *  
 *  Summary:
 *    Returns a menu to which applications may add their own help items.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    outHelpMenu:
 *      On exit, contains the help menu.
 *    
 *    outFirstCustomItemIndex:
 *      On exit, contains the menu item index that will be used by the
 *      first item added by the application. This parameter may be NULL.
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
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function HMGetHelpMenu( var outHelpMenu: MenuRef; outFirstCustomItemIndex: MenuItemIndexPtr { can be NULL } ): OSStatus; external name '_HMGetHelpMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Installing/Retrieving Content }
{ Menu title and item help tags are not supported by CarbonLib. They are fully supported on Mac OS X. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Pass NULL for the inContent parameter of these APIs to remove help content from a control, window, or menu. }
=======
{ Pass NULL for the inContent parameter of HMSetControl/Window/MenuItemHelpContent to remove help content
       from a control, window, or menu. }
>>>>>>> graemeg/fixes_2_2
=======
{ Pass NULL for the inContent parameter of HMSetControl/Window/MenuItemHelpContent to remove help content
       from a control, window, or menu. }
>>>>>>> origin/fixes_2_2
=======
{ Pass NULL for the inContent parameter of these APIs to remove help content from a control, window, or menu. }
>>>>>>> origin/fixes_2.4
{
 *  HMSetControlHelpContent()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMSetControlHelpContent( inControl: ControlRef; {const} inContent: HMHelpContentPtr { can be NULL } ): OSStatus; external name '_HMSetControlHelpContent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMGetControlHelpContent()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMGetControlHelpContent( inControl: ControlRef; var outContent: HMHelpContentRec ): OSStatus; external name '_HMGetControlHelpContent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMSetWindowHelpContent()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMSetWindowHelpContent( inWindow: WindowRef; {const} inContent: HMHelpContentPtr { can be NULL } ): OSStatus; external name '_HMSetWindowHelpContent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMGetWindowHelpContent()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMGetWindowHelpContent( inWindow: WindowRef; var outContent: HMHelpContentRec ): OSStatus; external name '_HMGetWindowHelpContent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMSetMenuItemHelpContent()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMSetMenuItemHelpContent( inMenu: MenuRef; inItem: MenuItemIndex; {const} inContent: HMHelpContentPtr { can be NULL } ): OSStatus; external name '_HMSetMenuItemHelpContent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMGetMenuItemHelpContent()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMGetMenuItemHelpContent( inMenu: MenuRef; inItem: MenuItemIndex; var outContent: HMHelpContentRec ): OSStatus; external name '_HMGetMenuItemHelpContent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Installing/Retrieving Content Callbacks }
{
 *  HMInstallControlContentCallback()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMInstallControlContentCallback( inControl: ControlRef; inContentUPP: HMControlContentUPP ): OSStatus; external name '_HMInstallControlContentCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMInstallWindowContentCallback()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMInstallWindowContentCallback( inWindow: WindowRef; inContentUPP: HMWindowContentUPP ): OSStatus; external name '_HMInstallWindowContentCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMInstallMenuTitleContentCallback()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMInstallMenuTitleContentCallback( inMenu: MenuRef; inContentUPP: HMMenuTitleContentUPP ): OSStatus; external name '_HMInstallMenuTitleContentCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMInstallMenuItemContentCallback()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMInstallMenuItemContentCallback( inMenu: MenuRef; inContentUPP: HMMenuItemContentUPP ): OSStatus; external name '_HMInstallMenuItemContentCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMGetControlContentCallback()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMGetControlContentCallback( inControl: ControlRef; var outContentUPP: HMControlContentUPP ): OSStatus; external name '_HMGetControlContentCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMGetWindowContentCallback()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMGetWindowContentCallback( inWindow: WindowRef; var outContentUPP: HMWindowContentUPP ): OSStatus; external name '_HMGetWindowContentCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMGetMenuTitleContentCallback()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMGetMenuTitleContentCallback( inMenu: MenuRef; var outContentUPP: HMMenuTitleContentUPP ): OSStatus; external name '_HMGetMenuTitleContentCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMGetMenuItemContentCallback()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMGetMenuItemContentCallback( inMenu: MenuRef; var outContentUPP: HMMenuItemContentUPP ): OSStatus; external name '_HMGetMenuItemContentCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Enabling and Disabling Help Tags }
{
 *  HMAreHelpTagsDisplayed()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMAreHelpTagsDisplayed: Boolean; external name '_HMAreHelpTagsDisplayed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMSetHelpTagsDisplayed()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMSetHelpTagsDisplayed( inDisplayTags: Boolean ): OSStatus; external name '_HMSetHelpTagsDisplayed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMSetTagDelay()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMSetTagDelay( inDelay: Duration ): OSStatus; external name '_HMSetTagDelay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMGetTagDelay()
 *  
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
 *    Non-Carbon CFM:   not available
 }
function HMGetTagDelay( var outDelay: Duration ): OSStatus; external name '_HMGetTagDelay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{ Compatibility }
{
 *  HMSetMenuHelpFromBalloonRsrc()
 *  
 *  Summary:
 *    Not really implemented.
 *  
 *  Discussion:
 *    Though this API is exported from CarbonLib and Mac OS X, it is
 *    completely non-functional. We have no plans to implement it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function HMSetMenuHelpFromBalloonRsrc( inMenu: MenuRef; inHmnuRsrcID: SInt16 ): OSStatus; external name '_HMSetMenuHelpFromBalloonRsrc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMSetDialogHelpFromBalloonRsrc()
 *  
 *  Summary:
 *    Not really implemented.
 *  
 *  Discussion:
 *    Though this API is exported from CarbonLib and Mac OS X, it is
 *    completely non-functional. We have no plans to implement it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function HMSetDialogHelpFromBalloonRsrc( inDialog: DialogRef; inHdlgRsrcID: SInt16; inItemStart: SInt16 ): OSStatus; external name '_HMSetDialogHelpFromBalloonRsrc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ Displaying tags }
{
 *  HMDisplayTag()
 *  
 *  Summary:
 *    Displays a help tag at a user-defined location.
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Prior to Mac OS X 10.4, HMDisplayTag does not retain the help
 *    content that is passed to it, nor release it when the tag is
 *    closed. Your application must ensure that the help content
 *    remains valid as long as the tag may be visible (which
 *    effectively means that your application should never dispose of
 *    help content that is passed to HMDisplayTag). In Mac OS X 10.4
 *    and later, HMDisplayTag makes a copy of the content and releases
 *    the copy when the tag closes, so you can release the content
 *    after HMDisplayTag returns.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *    Note that HMDisplayTag does not currently retain the help content
 *    that is passed to it, nor release it when the tag is closed. Your
 *    application must ensure that the help content remains valid as
 *    long as the tag may be visible (which effectively means that your
 *    application should never dispose of help content that is passed
 *    to HMDisplayTag).
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inContent:
 *      HMHelpContentRec describing the help tag to be displayed.
 *  
 *  Result:
 *    An OSStatus code indicating success or failure.
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
 *    CarbonLib:        in CarbonLib 1.2 and later
 *    Non-Carbon CFM:   not available
 }
function HMDisplayTag( const (*var*) inContent: HMHelpContentRec ): OSStatus; external name '_HMDisplayTag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  HMHideTag()
 *  
 *  Summary:
 *    HMHideTag hides the currently visible help tag.  If there is no
 *    current help tag, this call does nothing.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    An OSStatus code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.2 and later
 *    Non-Carbon CFM:   not available
 }
function HMHideTag: OSStatus; external name '_HMHideTag';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_CPU_64}


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_CPU_64}


>>>>>>> origin/fixes_2.4
{
 *  Summary:
 *    Values for the inOptions parameter to HMHideTagWithOptions.
 }
const
{
   * Causes the tag to fade out when hidden. If this flag is not
   * specified, the tag is hidden without fading.
   }
	kHMHideTagFade = 1 shl 0;

  {
   * Causes the tag to begin hiding immediately. If this flag is not
   * specified, the tag is hidden after a short delay (currently 0.75
   * second).
   }
	kHMHideTagImmediately = 1 shl 1;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  HMHideTagWithOptions()
 *  
 *  Summary:
 *    Hides the current help tag, with various options to control how
 *    the tag is hidden.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inOptions:
 *      Options for how the tag should be hidden. kHMHideTagFade and
 *      kHMHideTagImmediately are the only available options.
 *  
 *  Result:
 *    An operating system result code. noErr is returned if there is no
 *    tag currently visible.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function HMHideTagWithOptions( inOptions: OptionBits ): OSStatus; external name '_HMHideTagWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


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
 *  HMSetMenuHelpFromBalloonRsrc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Though this API is exported from CarbonLib and Mac OS X, it is
 *    completely non-functional.
 *  
 *  Summary:
 *    Not really implemented.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function HMSetMenuHelpFromBalloonRsrc( inMenu: MenuRef; inHmnuRsrcID: SInt16 ): OSStatus; external name '_HMSetMenuHelpFromBalloonRsrc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  HMSetDialogHelpFromBalloonRsrc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Though this API is exported from CarbonLib and Mac OS X, it is
 *    completely non-functional.
 *  
 *  Summary:
 *    Not really implemented.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function HMSetDialogHelpFromBalloonRsrc( inDialog: DialogRef; inHdlgRsrcID: SInt16; inItemStart: SInt16 ): OSStatus; external name '_HMSetDialogHelpFromBalloonRsrc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {TARGET_CPU_64}
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
