{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/LowMem.h
 
     Contains:   Low Memory Accessor Interfaces.
                 The contents of this header file are deprecated.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1993-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1993-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       LowMem.p
=======
     File:       CarbonCore/LowMem.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Low Memory Accessor Interfaces.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1993-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1993-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit LowMem;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
uses MacTypes,Files;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD


=======


>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Files,MacMemory,OSUtils,Resources,Quickdraw,Controls,Events,Menus,Fonts,MacWindows;

=======
uses MacTypes,Files;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}

{*************************************************************************************

    SIMPLE LOWMEM ACCESSORS

*************************************************************************************}
{
    The following functions were moved to Quickdraw.h:
    
        LMSetDeviceList
        LMSetLastSPExtra
        LMGetWidthListHand
        LMSetWidthListHand
        LMGetScrHRes
        LMSetScrHRes
        LMSetScrVRes
        LMGetScrVRes
        LMGetHiliteMode
        LMSetHiliteMode
        LMGetMainDevice
        LMSetMainDevice
        LMGetDeviceList
        LMGetQDColors
        LMSetQDColors
        LMGetWidthPtr
        LMSetWidthPtr
        LMGetWidthTabHandle
        LMSetWidthTabHandle
        LMGetLastSPExtra
        LMGetLastFOND
        LMSetLastFOND
        LMGetFractEnable
        LMSetFractEnable
        LMGetTheGDevice
        LMSetTheGDevice
        LMGetCursorNew
        LMSetCursorNew
        LMGetHiliteRGB
        LMSetHiliteRGB
    
    The following functions were moved to TextEdit.h:
    
        LMGetWordRedraw
        LMSetWordRedraw

    The following functions were moved to Menus.h:
    
        LMGetTheMenu
    
    The following functions were moved to Events.h:
    
        LMGetKeyRepThresh
        LMSetKeyRepThresh
        LMGetKeyThresh
        LMSetKeyRepThresh
        LMGetKbdLast
        LMSetKbdLast
        LMGetKbdType
        LMSetKbdType

}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$ifc not TARGET_CPU_64}
{
 *  LMGetMemTop()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetMemTop: Ptr; external name '_LMGetMemTop';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{
 *  LMSetMemTop()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetMemTop( value: Ptr ); external name '_LMSetMemTop';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{
 *  LMGetBufPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBufPtr: Ptr; external name '_LMGetBufPtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{
 *  LMSetBufPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBufPtr( value: Ptr ); external name '_LMSetBufPtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{
 *  LMGetHeapEnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetHeapEnd: Ptr; external name '_LMGetHeapEnd';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2.4


{
 *  LMSetHeapEnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetHeapEnd( value: Ptr ); external name '_LMSetHeapEnd';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetHeapEnd( value: Ptr ); external name '_LMSetHeapEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  LMGetCPUFlag()   *** DEPRECATED ***
<<<<<<< HEAD
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetHeapEnd( value: Ptr ); external name '_LMSetHeapEnd';
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetHeapEnd( value: Ptr ); external name '_LMSetHeapEnd';
=======
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCPUFlag: UInt8; external name '_LMGetCPUFlag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetCPUFlag()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCPUFlag( value: ByteParameter ); external name '_LMSetCPUFlag';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMGetCPUFlag()   *** DEPRECATED ***
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetHeapEnd( value: Ptr ); external name '_LMSetHeapEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetCPUFlag()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCPUFlag: UInt8; external name '_LMGetCPUFlag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetCPUFlag()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCPUFlag( value: ByteParameter ); external name '_LMSetCPUFlag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetRndSeed()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetRndSeed: SInt32; external name '_LMGetRndSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetRndSeed()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetRndSeed( value: SInt32 ); external name '_LMSetRndSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSEvtEnb()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetCPUFlag: UInt8; external name '_LMGetCPUFlag';
=======
function LMGetSEvtEnb: UInt8; external name '_LMGetSEvtEnb';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMSetCPUFlag()   *** DEPRECATED ***
=======
 *  LMSetSEvtEnb()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetCPUFlag( value: ByteParameter ); external name '_LMSetCPUFlag';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMGetCPUFlag()   *** DEPRECATED ***
=======
 *  LMGetRndSeed()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetRndSeed: SInt32; external name '_LMGetRndSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetRndSeed()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetRndSeed( value: SInt32 ); external name '_LMSetRndSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSEvtEnb()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
=======
procedure LMSetSEvtEnb( value: ByteParameter ); external name '_LMSetSEvtEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBootDrive: SInt16; external name '_LMGetBootDrive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBootDrive( value: SInt16 ); external name '_LMSetBootDrive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  LMGetSdVolume()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetCPUFlag: UInt8; external name '_LMGetCPUFlag';
=======
function LMGetSEvtEnb: UInt8; external name '_LMGetSEvtEnb';
>>>>>>> origin/cpstrnew
=======
function LMGetSdVolume: UInt8; external name '_LMGetSdVolume';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  LMSetCPUFlag()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
 *  LMSetSEvtEnb()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
=======
 *  LMSetSdVolume()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetCPUFlag: UInt8; external name '_LMGetCPUFlag';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetCPUFlag()   *** DEPRECATED ***
=======
 *  LMGetRndSeed()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetRndSeed: SInt32; external name '_LMGetRndSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetRndSeed()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetRndSeed( value: SInt32 ); external name '_LMSetRndSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSEvtEnb()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
procedure LMSetSEvtEnb( value: ByteParameter ); external name '_LMSetSEvtEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBootDrive: SInt16; external name '_LMGetBootDrive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBootDrive( value: SInt16 ); external name '_LMSetBootDrive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  LMGetSdVolume()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure LMSetCPUFlag( value: ByteParameter ); external name '_LMSetCPUFlag';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetRndSeed()   *** DEPRECATED ***
=======
procedure LMSetCPUFlag( value: ByteParameter ); external name '_LMSetCPUFlag';
=======
function LMGetSEvtEnb: UInt8; external name '_LMGetSEvtEnb';
>>>>>>> graemeg/cpstrnew
=======
function LMGetSdVolume: UInt8; external name '_LMGetSdVolume';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  LMGetRndSeed()   *** DEPRECATED ***
=======
procedure LMSetSdVolume( value: ByteParameter ); external name '_LMSetSdVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetSoundPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundPtr: Ptr; external name '_LMGetSoundPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSoundPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundPtr( value: Ptr ); external name '_LMSetSoundPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetSoundBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundBase: Ptr; external name '_LMGetSoundBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetSoundBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundBase( value: Ptr ); external name '_LMSetSoundBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSoundLevel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundLevel: UInt8; external name '_LMGetSoundLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetSoundLevel()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetRndSeed: SInt32; external name '_LMGetRndSeed';
=======
procedure LMSetSoundLevel( value: ByteParameter ); external name '_LMSetSoundLevel';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  LMSetRndSeed()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
 *  LMGetCurPitch()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetRndSeed: SInt32; external name '_LMGetRndSeed';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetRndSeed()   *** DEPRECATED ***
=======
function LMGetCurPitch: SInt16; external name '_LMGetCurPitch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurPitch()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetRndSeed( value: SInt32 ); external name '_LMSetRndSeed';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetSEvtEnb()   *** DEPRECATED ***
=======
 *  LMSetSEvtEnb()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSEvtEnb( value: ByteParameter ); external name '_LMSetSEvtEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBootDrive: SInt16; external name '_LMGetBootDrive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBootDrive( value: SInt16 ); external name '_LMSetBootDrive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  LMGetSdVolume()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSEvtEnb: UInt8; external name '_LMGetSEvtEnb';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetSEvtEnb()   *** DEPRECATED ***
=======
function LMGetSdVolume: UInt8; external name '_LMGetSdVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSdVolume()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSEvtEnb( value: ByteParameter ); external name '_LMSetSEvtEnb';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{$endc} {not TARGET_CPU_64}

{
 *  LMGetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBootDrive: SInt16; external name '_LMGetBootDrive';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBootDrive( value: SInt16 ); external name '_LMSetBootDrive';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
procedure LMSetRndSeed( value: SInt32 ); external name '_LMSetRndSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSEvtEnb()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSEvtEnb: UInt8; external name '_LMGetSEvtEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSEvtEnb()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSEvtEnb( value: ByteParameter ); external name '_LMSetSEvtEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBootDrive: SInt16; external name '_LMGetBootDrive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetBootDrive()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBootDrive( value: SInt16 ); external name '_LMSetBootDrive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  LMGetSdVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSdVolume: UInt8; external name '_LMGetSdVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSdVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSdVolume( value: ByteParameter ); external name '_LMSetSdVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  LMGetSdVolume()   *** DEPRECATED ***
=======
 *  LMGetSoundPtr()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 }
function LMGetSdVolume: UInt8; external name '_LMGetSdVolume';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetSdVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSdVolume( value: ByteParameter ); external name '_LMSetSdVolume';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetSoundPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundPtr: Ptr; external name '_LMGetSoundPtr';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetSoundPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundPtr( value: Ptr ); external name '_LMSetSoundPtr';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetSoundBase()   *** DEPRECATED ***
=======
 }
function LMGetSoundPtr: Ptr; external name '_LMGetSoundPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSoundPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundPtr( value: Ptr ); external name '_LMSetSoundPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetSoundBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundBase: Ptr; external name '_LMGetSoundBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetSoundBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundBase( value: Ptr ); external name '_LMSetSoundBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSoundLevel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundLevel: UInt8; external name '_LMGetSoundLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetSoundLevel()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSoundBase: Ptr; external name '_LMGetSoundBase';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetSoundBase()   *** DEPRECATED ***
=======
procedure LMSetSoundLevel( value: ByteParameter ); external name '_LMSetSoundLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurPitch()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSoundBase( value: Ptr ); external name '_LMSetSoundBase';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetSoundLevel()   *** DEPRECATED ***
=======
function LMGetCurPitch: SInt16; external name '_LMGetCurPitch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurPitch()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
 *  LMSetSdVolume()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSdVolume( value: ByteParameter ); external name '_LMSetSdVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetSoundPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundPtr: Ptr; external name '_LMGetSoundPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSoundPtr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundPtr( value: Ptr ); external name '_LMSetSoundPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetSoundBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundBase: Ptr; external name '_LMGetSoundBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetSoundBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundBase( value: Ptr ); external name '_LMSetSoundBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSoundLevel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundLevel: UInt8; external name '_LMGetSoundLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetSoundLevel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundLevel( value: ByteParameter ); external name '_LMSetSoundLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurPitch()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurPitch: SInt16; external name '_LMGetCurPitch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurPitch()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurPitch( value: SInt16 ); external name '_LMSetCurPitch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetROM85()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetROM85()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetPortBUse()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetPortBUse()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetGNEFilter()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetGNEFilter()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetROMBase()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetROMBase()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetRAMBase()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetRAMBase()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetDSAlertTab()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetDSAlertTab()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
    NOTE:   LMGetABusVars and LMSetABusVars have been removed.
            Their implememtation in InterfaceLib was inconsistent
            with their prototypes here.  In InterfaceLib LMSetABusVars 
            would copy eight bytes and LMGetABusVars would return the
            value 0x02D8 instead of the long at that location.
            
            Use LMGetABusGlobals/LMSetABusGlobals to get/set the
            long at location 0x02D8 which is a pointer to the AppleTalk
            globals.  Use LMGetABusDCE/LMSetABusDCE to get/set the
            long at location 0x02DC which is the .MPP driver
            Device Control Entry. 
            
}
{
 *  LMGetABusGlobals()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMGetABusDCE()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMSetABusGlobals()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMSetABusDCE()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{$ifc not TARGET_CPU_64}
{
 *  LMGetScrDmpEnb()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetScrDmpEnb: UInt8; external name '_LMGetScrDmpEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetScrDmpEnb()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetScrDmpEnb( value: ByteParameter ); external name '_LMSetScrDmpEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetBufTgFNum()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetSoundLevel: UInt8; external name '_LMGetSoundLevel';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetSoundLevel()   *** DEPRECATED ***
=======
function LMGetBufTgFNum: SInt32; external name '_LMGetBufTgFNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFNum()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSoundLevel( value: ByteParameter ); external name '_LMSetSoundLevel';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetCurPitch()   *** DEPRECATED ***
=======
procedure LMSetBufTgFNum( value: SInt32 ); external name '_LMSetBufTgFNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgFFlg()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetCurPitch: SInt16; external name '_LMGetCurPitch';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetCurPitch()   *** DEPRECATED ***
=======
function LMGetBufTgFFlg: SInt16; external name '_LMGetBufTgFFlg';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFFlg()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetCurPitch( value: SInt16 ); external name '_LMSetCurPitch';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{$endc} {not TARGET_CPU_64}

{
 *  LMGetROM85()
=======
procedure LMSetCurPitch( value: SInt16 ); external name '_LMSetCurPitch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetROM85()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetROM85()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetPortBUse()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetPortBUse()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
 *  LMSetROM85()
=======
 *  LMGetGNEFilter()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
 *  LMGetPortBUse()
=======
 *  LMSetGNEFilter()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
 *  LMSetPortBUse()
=======
 *  LMGetROMBase()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
 *  LMGetGNEFilter()
=======
 *  LMSetROMBase()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
 *  LMSetGNEFilter()
=======
 *  LMGetRAMBase()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
 *  LMGetROMBase()
=======
 *  LMSetRAMBase()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
 *  LMSetROMBase()
=======
 *  LMGetDSAlertTab()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
 *  LMGetRAMBase()
=======
 *  LMSetDSAlertTab()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


<<<<<<< HEAD
{
 *  LMSetRAMBase()
=======
{
    NOTE:   LMGetABusVars and LMSetABusVars have been removed.
            Their implememtation in InterfaceLib was inconsistent
            with their prototypes here.  In InterfaceLib LMSetABusVars 
            would copy eight bytes and LMGetABusVars would return the
            value 0x02D8 instead of the long at that location.
            
            Use LMGetABusGlobals/LMSetABusGlobals to get/set the
            long at location 0x02D8 which is a pointer to the AppleTalk
            globals.  Use LMGetABusDCE/LMSetABusDCE to get/set the
            long at location 0x02DC which is the .MPP driver
            Device Control Entry. 
            
}
{
 *  LMGetABusGlobals()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
>>>>>>> graemeg/cpstrnew
 }


{
<<<<<<< HEAD
 *  LMGetDSAlertTab()
=======
 *  LMGetABusDCE()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
>>>>>>> graemeg/cpstrnew
 }


{
<<<<<<< HEAD
 *  LMSetDSAlertTab()
=======
 *  LMSetABusGlobals()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
>>>>>>> graemeg/cpstrnew
 }


{
<<<<<<< HEAD
    NOTE:   LMGetABusVars and LMSetABusVars have been removed.
            Their implememtation in InterfaceLib was inconsistent
            with their prototypes here.  In InterfaceLib LMSetABusVars 
            would copy eight bytes and LMGetABusVars would return the
            value 0x02D8 instead of the long at that location.
            
            Use LMGetABusGlobals/LMSetABusGlobals to get/set the
            long at location 0x02D8 which is a pointer to the AppleTalk
            globals.  Use LMGetABusDCE/LMSetABusDCE to get/set the
            long at location 0x02DC which is the .MPP driver
            Device Control Entry. 
            
}
{
 *  LMGetABusGlobals()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMGetABusDCE()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMSetABusGlobals()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMSetABusDCE()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{$ifc not TARGET_CPU_64}
{
 *  LMGetScrDmpEnb()   *** DEPRECATED ***
=======
procedure LMSetBufTgFFlg( value: SInt16 ); external name '_LMSetBufTgFFlg';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgFBkNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBufTgFBkNum: SInt16; external name '_LMGetBufTgFBkNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFBkNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBufTgFBkNum( value: SInt16 ); external name '_LMSetBufTgFBkNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgDate()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBufTgDate: SInt32; external name '_LMGetBufTgDate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgDate()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBufTgDate( value: SInt32 ); external name '_LMSetBufTgDate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetMinStack()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetMinStack: SInt32; external name '_LMGetMinStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetMinStack()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetMinStack( value: SInt32 ); external name '_LMSetMinStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetDefltStack()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetDefltStack: SInt32; external name '_LMGetDefltStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetDefltStack()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetDefltStack( value: SInt32 ); external name '_LMSetDefltStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetGZRootHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetGZRootHnd: Handle; external name '_LMGetGZRootHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetGZRootHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetGZRootHnd( value: Handle ); external name '_LMSetGZRootHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetGZMoveHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetGZMoveHnd: Handle; external name '_LMGetGZMoveHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetGZMoveHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetGZMoveHnd( value: Handle ); external name '_LMSetGZMoveHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetToExtFS()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetToExtFS: UniversalProcPtr; external name '_LMGetToExtFS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetToExtFS()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetToExtFS( value: UniversalProcPtr ); external name '_LMSetToExtFS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetJStash()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetJStash: UniversalProcPtr; external name '_LMGetJStash';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetJStash()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetJStash( value: UniversalProcPtr ); external name '_LMSetJStash';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurApRefNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApRefNum: FSIORefNum; external name '_LMGetCurApRefNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurApRefNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApRefNum( value: FSIORefNum ); external name '_LMSetCurApRefNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    If using the Thread Manager APIs, use ThreadCurrentStackSpace(). 
 *    If using pthreads(), get the stack with
 *    pthread_get_stackaddr_np().
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurStackBase: Ptr; external name '_LMGetCurStackBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support setting the stack base.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetScrDmpEnb: UInt8; external name '_LMGetScrDmpEnb';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetScrDmpEnb()   *** DEPRECATED ***
=======
procedure LMSetCurStackBase( value: Ptr ); external name '_LMSetCurStackBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetCurPageOption()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetScrDmpEnb( value: ByteParameter ); external name '_LMSetScrDmpEnb';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetBufTgFNum()   *** DEPRECATED ***
=======
=======
procedure LMSetSdVolume( value: ByteParameter ); external name '_LMSetSdVolume';
=======
function LMGetCurPageOption: SInt16; external name '_LMGetCurPageOption';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMGetSoundPtr()   *** DEPRECATED ***
=======
 *  LMSetCurPageOption()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSoundPtr: Ptr; external name '_LMGetSoundPtr';
=======
procedure LMSetCurPageOption( value: SInt16 ); external name '_LMSetCurPageOption';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMSetSoundPtr()   *** DEPRECATED ***
=======
 *  LMGetPrintErr()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSoundPtr( value: Ptr ); external name '_LMSetSoundPtr';
=======
function LMGetPrintErr: SInt16; external name '_LMGetPrintErr';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMGetSoundBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundBase: Ptr; external name '_LMGetSoundBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetSoundBase()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSoundBase( value: Ptr ); external name '_LMSetSoundBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSoundLevel()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSoundLevel: UInt8; external name '_LMGetSoundLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetSoundLevel()   *** DEPRECATED ***
=======
 *  LMSetPrintErr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetPrintErr( value: SInt16 ); external name '_LMSetPrintErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetApFontID()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the application font.  Use GetAppFont() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetApFontID: SInt16; external name '_LMGetApFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetApFontID()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set the id of the application font.  Don't use this call anymore;
 *    this functionality is not supported on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetApFontID( value: SInt16 ); external name '_LMSetApFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$ifc not TARGET_CPU_64}
{
 *  LMGetOneOne()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSoundLevel( value: ByteParameter ); external name '_LMSetSoundLevel';
=======
function LMGetOneOne: SInt32; external name '_LMGetOneOne';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  LMGetCurPitch()   *** DEPRECATED ***
=======
 *  LMSetOneOne()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetCurPitch: SInt16; external name '_LMGetCurPitch';
=======
procedure LMSetOneOne( value: SInt32 ); external name '_LMSetOneOne';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  LMSetCurPitch()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurPitch( value: SInt16 ); external name '_LMSetCurPitch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetROM85()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetROM85()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetPortBUse()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetPortBUse()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetGNEFilter()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetGNEFilter()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetROMBase()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetROMBase()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetRAMBase()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetRAMBase()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMGetDSAlertTab()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetDSAlertTab()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
    NOTE:   LMGetABusVars and LMSetABusVars have been removed.
            Their implememtation in InterfaceLib was inconsistent
            with their prototypes here.  In InterfaceLib LMSetABusVars 
            would copy eight bytes and LMGetABusVars would return the
            value 0x02D8 instead of the long at that location.
            
            Use LMGetABusGlobals/LMSetABusGlobals to get/set the
            long at location 0x02D8 which is a pointer to the AppleTalk
            globals.  Use LMGetABusDCE/LMSetABusDCE to get/set the
            long at location 0x02DC which is the .MPP driver
            Device Control Entry. 
            
}
{
 *  LMGetABusGlobals()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMGetABusDCE()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMSetABusGlobals()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
>>>>>>> graemeg/cpstrnew
 *  LMSetABusDCE()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{$ifc not TARGET_CPU_64}
{
 *  LMGetScrDmpEnb()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetScrDmpEnb: UInt8; external name '_LMGetScrDmpEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetScrDmpEnb()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetScrDmpEnb( value: ByteParameter ); external name '_LMSetScrDmpEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetBufTgFNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBufTgFNum: SInt32; external name '_LMGetBufTgFNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBufTgFNum( value: SInt32 ); external name '_LMSetBufTgFNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgFFlg()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetBufTgFFlg: SInt16; external name '_LMGetBufTgFFlg';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFFlg()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetBufTgFFlg( value: SInt16 ); external name '_LMSetBufTgFFlg';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgFBkNum()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetBufTgFNum: SInt32; external name '_LMGetBufTgFNum';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetBufTgFNum()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
function LMGetBufTgFBkNum: SInt16; external name '_LMGetBufTgFBkNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFBkNum()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure LMSetBufTgFNum( value: SInt32 ); external name '_LMSetBufTgFNum';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetBufTgFFlg()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
procedure LMSetBufTgFBkNum( value: SInt16 ); external name '_LMSetBufTgFBkNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgDate()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetBufTgFFlg: SInt16; external name '_LMGetBufTgFFlg';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetBufTgFFlg()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
function LMGetBufTgDate: SInt32; external name '_LMGetBufTgDate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgDate()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure LMSetBufTgFFlg( value: SInt16 ); external name '_LMSetBufTgFFlg';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetBufTgFBkNum()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
procedure LMSetBufTgDate( value: SInt32 ); external name '_LMSetBufTgDate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetMinStack()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetBufTgFBkNum: SInt16; external name '_LMGetBufTgFBkNum';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetBufTgFBkNum()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
function LMGetMinStack: SInt32; external name '_LMGetMinStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetMinStack()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure LMSetBufTgFBkNum( value: SInt16 ); external name '_LMSetBufTgFBkNum';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetBufTgDate()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
procedure LMSetMinStack( value: SInt32 ); external name '_LMSetMinStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetDefltStack()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetBufTgDate: SInt32; external name '_LMGetBufTgDate';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetBufTgDate()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
function LMGetDefltStack: SInt32; external name '_LMGetDefltStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetDefltStack()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure LMSetBufTgDate( value: SInt32 ); external name '_LMSetBufTgDate';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetMinStack()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetMinStack: SInt32; external name '_LMGetMinStack';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetMinStack()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetMinStack( value: SInt32 ); external name '_LMSetMinStack';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetDefltStack()   *** DEPRECATED ***
=======
procedure LMSetDefltStack( value: SInt32 ); external name '_LMSetDefltStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetGZRootHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetGZRootHnd: Handle; external name '_LMGetGZRootHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetGZRootHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetGZRootHnd( value: Handle ); external name '_LMSetGZRootHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetGZMoveHnd()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetDefltStack: SInt32; external name '_LMGetDefltStack';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetDefltStack()   *** DEPRECATED ***
=======
function LMGetGZMoveHnd: Handle; external name '_LMGetGZMoveHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetGZMoveHnd()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetDefltStack( value: SInt32 ); external name '_LMSetDefltStack';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetGZRootHnd()   *** DEPRECATED ***
=======
procedure LMSetDefltStack( value: SInt32 ); external name '_LMSetDefltStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetGZRootHnd()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetGZRootHnd: Handle; external name '_LMGetGZRootHnd';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetGZRootHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetGZRootHnd( value: Handle ); external name '_LMSetGZRootHnd';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetGZMoveHnd()   *** DEPRECATED ***
=======
procedure LMSetGZMoveHnd( value: Handle ); external name '_LMSetGZMoveHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetToExtFS()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetToExtFS: UniversalProcPtr; external name '_LMGetToExtFS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetToExtFS()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetToExtFS( value: UniversalProcPtr ); external name '_LMSetToExtFS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetJStash()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetGZMoveHnd: Handle; external name '_LMGetGZMoveHnd';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetGZMoveHnd()   *** DEPRECATED ***
=======
function LMGetJStash: UniversalProcPtr; external name '_LMGetJStash';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetJStash()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetGZMoveHnd( value: Handle ); external name '_LMSetGZMoveHnd';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetToExtFS()   *** DEPRECATED ***
=======
procedure LMSetJStash( value: UniversalProcPtr ); external name '_LMSetJStash';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurApRefNum()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetToExtFS: UniversalProcPtr; external name '_LMGetToExtFS';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetToExtFS()   *** DEPRECATED ***
=======
function LMGetCurApRefNum: FSIORefNum; external name '_LMGetCurApRefNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurApRefNum()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetToExtFS( value: UniversalProcPtr ); external name '_LMSetToExtFS';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetJStash()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetJStash: UniversalProcPtr; external name '_LMGetJStash';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetJStash()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetJStash( value: UniversalProcPtr ); external name '_LMSetJStash';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetCurApRefNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApRefNum: FSIORefNum; external name '_LMGetCurApRefNum';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetCurApRefNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApRefNum( value: FSIORefNum ); external name '_LMSetCurApRefNum';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    If using the Thread Manager APIs, use ThreadCurrentStackSpace(). 
 *    If using pthreads(), get the stack with
 *    pthread_get_stackaddr_np().
=======
procedure LMSetCurApRefNum( value: FSIORefNum ); external name '_LMSetCurApRefNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    If using the Thread Manager APIs, use ThreadCurrentStackSpace(). 
 *    If using pthreads(), get the stack with
 *    pthread_get_stackaddr_np().
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurStackBase: Ptr; external name '_LMGetCurStackBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support setting the stack base.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurStackBase( value: Ptr ); external name '_LMSetCurStackBase';
=======
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMGetCurPageOption()   *** DEPRECATED ***
=======
 *  LMSetGZRootHnd()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetCurPageOption: SInt16; external name '_LMGetCurPageOption';
=======
procedure LMSetGZRootHnd( value: Handle ); external name '_LMSetGZRootHnd';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMSetCurPageOption()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurPageOption( value: SInt16 ); external name '_LMSetCurPageOption';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetPrintErr()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetCurStackBase: Ptr; external name '_LMGetCurStackBase';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support setting the stack base.
=======
function LMGetPrintErr: SInt16; external name '_LMGetPrintErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetPrintErr()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetCurStackBase( value: Ptr ); external name '_LMSetCurStackBase';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetCurPageOption()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurPageOption: SInt16; external name '_LMGetCurPageOption';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetCurPageOption()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurPageOption( value: SInt16 ); external name '_LMSetCurPageOption';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetPrintErr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetPrintErr: SInt16; external name '_LMGetPrintErr';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetPrintErr()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetPrintErr( value: SInt16 ); external name '_LMSetPrintErr';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{$endc} {not TARGET_CPU_64}

{
 *  LMGetApFontID()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the application font.  Use GetAppFont() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetApFontID: SInt16; external name '_LMGetApFontID';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetApFontID()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set the id of the application font.  Don't use this call anymore;
 *    this functionality is not supported on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetApFontID( value: SInt16 ); external name '_LMSetApFontID';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{$ifc not TARGET_CPU_64}
{
 *  LMGetOneOne()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetOneOne: SInt32; external name '_LMGetOneOne';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetOneOne()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetOneOne( value: SInt32 ); external name '_LMSetOneOne';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetMinusOne()   *** DEPRECATED ***
=======
 *  LMGetGZMoveHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetGZMoveHnd: Handle; external name '_LMGetGZMoveHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetGZMoveHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetGZMoveHnd( value: Handle ); external name '_LMSetGZMoveHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetToExtFS()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetToExtFS: UniversalProcPtr; external name '_LMGetToExtFS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetToExtFS()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetToExtFS( value: UniversalProcPtr ); external name '_LMSetToExtFS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetJStash()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetJStash: UniversalProcPtr; external name '_LMGetJStash';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetJStash()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetJStash( value: UniversalProcPtr ); external name '_LMSetJStash';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurApRefNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApRefNum: FSIORefNum; external name '_LMGetCurApRefNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurApRefNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApRefNum( value: FSIORefNum ); external name '_LMSetCurApRefNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    If using the Thread Manager APIs, use ThreadCurrentStackSpace(). 
 *    If using pthreads(), get the stack with
 *    pthread_get_stackaddr_np().
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurStackBase: Ptr; external name '_LMGetCurStackBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support setting the stack base.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurStackBase( value: Ptr ); external name '_LMSetCurStackBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetCurPageOption()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurPageOption: SInt16; external name '_LMGetCurPageOption';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetCurPageOption()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurPageOption( value: SInt16 ); external name '_LMSetCurPageOption';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetPrintErr()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetMinusOne: SInt32; external name '_LMGetMinusOne';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetMinusOne()   *** DEPRECATED ***
=======
function LMGetPrintErr: SInt16; external name '_LMGetPrintErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetPrintErr()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetMinusOne( value: SInt32 ); external name '_LMSetMinusOne';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{$endc} {not TARGET_CPU_64}

{
 *  LMGetSysMap()
=======
=======
>>>>>>> graemeg/cpstrnew
procedure LMSetPrintErr( value: SInt16 ); external name '_LMSetPrintErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetApFontID()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the application font.  Use GetAppFont() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetApFontID: SInt16; external name '_LMGetApFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetApFontID()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set the id of the application font.  Don't use this call anymore;
 *    this functionality is not supported on Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetApFontID( value: SInt16 ); external name '_LMSetApFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$ifc not TARGET_CPU_64}
{
 *  LMGetOneOne()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetOneOne: SInt32; external name '_LMGetOneOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetOneOne()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetOneOne( value: SInt32 ); external name '_LMSetOneOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetMinusOne()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetMinusOne: SInt32; external name '_LMGetMinusOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetMinusOne()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetMinusOne( value: SInt32 ); external name '_LMSetMinusOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetSysMap()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysMap: SInt16; external name '_LMGetSysMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetSysMap()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysMap( value: SInt16 ); external name '_LMSetSysMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetResLoad()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetResLoad: UInt8; external name '_LMGetResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetResLoad()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetResLoad( value: ByteParameter ); external name '_LMSetResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetResErr()
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSysMap: SInt16; external name '_LMGetSysMap';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetSysMap()
=======
function LMGetResErr: SInt16; external name '_LMGetResErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetResErr()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSysMap( value: SInt16 ); external name '_LMSetSysMap';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetResLoad()
=======
procedure LMSetResErr( value: SInt16 ); external name '_LMSetResErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetTmpResLoad()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetResLoad: UInt8; external name '_LMGetResLoad';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetResLoad()
=======
 *  LMGetMinusOne()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetMinusOne: SInt32; external name '_LMGetMinusOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetMinusOne()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetMinusOne( value: SInt32 ); external name '_LMSetMinusOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetSysMap()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetResLoad( value: ByteParameter ); external name '_LMSetResLoad';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetResErr()
=======
>>>>>>> graemeg/cpstrnew
=======
function LMGetSysMap: SInt16; external name '_LMGetSysMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetSysMap()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetResErr: SInt16; external name '_LMGetResErr';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetResErr()
=======
procedure LMSetSysMap( value: SInt16 ); external name '_LMSetSysMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetResLoad()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetResErr( value: SInt16 ); external name '_LMSetResErr';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetTmpResLoad()
=======
function LMGetResLoad: UInt8; external name '_LMGetResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetResLoad()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetTmpResLoad: UInt8; external name '_LMGetTmpResLoad';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetTmpResLoad()
=======
procedure LMSetResLoad( value: ByteParameter ); external name '_LMSetResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetResErr()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetTmpResLoad( value: ByteParameter ); external name '_LMSetTmpResLoad';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetIntlSpec()
=======
function LMGetResErr: SInt16; external name '_LMGetResErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetResErr()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetIntlSpec: Ptr; external name '_LMGetIntlSpec';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetIntlSpec()
=======
procedure LMSetResErr( value: SInt16 ); external name '_LMSetResErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetTmpResLoad()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetIntlSpec( value: Ptr ); external name '_LMSetIntlSpec';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ LMGetWordRedraw and LMSetWordRedraw moved to TextEdit.h }
{$ifc not TARGET_CPU_64}
{
 *  LMGetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the system font family.  Use GetSysFont() in the
 *    Quickdraw framework instead of this.
=======
function LMGetTmpResLoad: UInt8; external name '_LMGetTmpResLoad';
=======
>>>>>>> graemeg/cpstrnew
=======
function LMGetTmpResLoad: UInt8; external name '_LMGetTmpResLoad';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetTmpResLoad()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetTmpResLoad( value: ByteParameter ); external name '_LMSetTmpResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetIntlSpec()
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetIntlSpec: Ptr; external name '_LMGetIntlSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetIntlSpec()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetIntlSpec( value: Ptr ); external name '_LMSetIntlSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ LMGetWordRedraw and LMSetWordRedraw moved to TextEdit.h }
{$ifc not TARGET_CPU_64}
{
 *  LMGetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the system font family.  Use GetSysFont() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysFontFam: SInt16; external name '_LMGetSysFontFam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMSetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set the id of the system font family.  This is not supported on
 *    Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysFontFam( value: SInt16 ); external name '_LMSetSysFontFam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetSysFontSize()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the size of the system font.  Use DefFontSize() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysFontSize: SInt16; external name '_LMGetSysFontSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetIntlSpec: Ptr; external name '_LMGetIntlSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetIntlSpec()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetIntlSpec( value: Ptr ); external name '_LMSetIntlSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ LMGetWordRedraw and LMSetWordRedraw moved to TextEdit.h }
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  LMSetSysFontSize()
 *  
 *  Summary:
 *    Set the size for the system font.  This is not supported on Mac
 *    OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysFontSize( value: SInt16 ); external name '_LMSetSysFontSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *  LMGetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the system font family.  Use GetSysFont() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysFontFam: SInt16; external name '_LMGetSysFontFam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}
>>>>>>> graemeg/cpstrnew

=======
{
 *  LMGetMemTop()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetMemTop: Ptr; external name '_LMGetMemTop';
{
 *  LMSetMemTop()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetMemTop(value: Ptr); external name '_LMSetMemTop';
{
 *  LMGetBufPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetBufPtr: Ptr; external name '_LMGetBufPtr';
{
 *  LMSetBufPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetBufPtr(value: Ptr); external name '_LMSetBufPtr';
{
 *  LMGetHeapEnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetHeapEnd: Ptr; external name '_LMGetHeapEnd';
{
 *  LMSetHeapEnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetHeapEnd(value: Ptr); external name '_LMSetHeapEnd';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetTheZone()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTheZone: THz; external name '_LMGetTheZone';
{
 *  LMSetTheZone()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTheZone(value: THz); external name '_LMSetTheZone';
{
 *  LMGetUTableBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetUTableBase: Ptr; external name '_LMGetUTableBase';
{
 *  LMSetUTableBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetUTableBase(value: Ptr); external name '_LMSetUTableBase';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetCPUFlag()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetCPUFlag: ByteParameter; external name '_LMGetCPUFlag';
{
 *  LMSetCPUFlag()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetCPUFlag(value: ByteParameter); external name '_LMSetCPUFlag';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetApplLimit()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetApplLimit: Ptr; external name '_LMGetApplLimit';
{
 *  LMSetApplLimit()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetApplLimit(value: Ptr); external name '_LMSetApplLimit';
{
 *  LMGetSysEvtMask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSysEvtMask: SInt16; external name '_LMGetSysEvtMask';
{  Carbon Usage: use SetEventMask }
{
 *  LMSetSysEvtMask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSysEvtMask(value: SInt16); external name '_LMSetSysEvtMask';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetRndSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetRndSeed: SInt32; external name '_LMGetRndSeed';
{
 *  LMSetRndSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetRndSeed(value: SInt32); external name '_LMSetRndSeed';
{
 *  LMGetSEvtEnb()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSEvtEnb: ByteParameter; external name '_LMGetSEvtEnb';
{
 *  LMSetSEvtEnb()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSEvtEnb(value: ByteParameter); external name '_LMSetSEvtEnb';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetTicks()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTicks: UInt32; external name '_LMGetTicks';
{
 *  LMSetTicks()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTicks(value: UInt32); external name '_LMSetTicks';
{
 *  LMGetVIA()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetVIA: Ptr; external name '_LMGetVIA';
{
 *  LMSetVIA()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetVIA(value: Ptr); external name '_LMSetVIA';
{
 *  LMGetSCCRd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSCCRd: Ptr; external name '_LMGetSCCRd';
{
 *  LMSetSCCRd()
=======
{
 *  LMGetMemTop()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetMemTop: Ptr; external name '_LMGetMemTop';
{
 *  LMSetMemTop()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetMemTop(value: Ptr); external name '_LMSetMemTop';
{
 *  LMGetBufPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetBufPtr: Ptr; external name '_LMGetBufPtr';
{
 *  LMSetBufPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetBufPtr(value: Ptr); external name '_LMSetBufPtr';
{
 *  LMGetHeapEnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetHeapEnd: Ptr; external name '_LMGetHeapEnd';
{
 *  LMSetHeapEnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetHeapEnd(value: Ptr); external name '_LMSetHeapEnd';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetTheZone()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTheZone: THz; external name '_LMGetTheZone';
{
 *  LMSetTheZone()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTheZone(value: THz); external name '_LMSetTheZone';
{
 *  LMGetUTableBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetUTableBase: Ptr; external name '_LMGetUTableBase';
{
 *  LMSetUTableBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetUTableBase(value: Ptr); external name '_LMSetUTableBase';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetCPUFlag()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetCPUFlag: ByteParameter; external name '_LMGetCPUFlag';
{
 *  LMSetCPUFlag()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetCPUFlag(value: ByteParameter); external name '_LMSetCPUFlag';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetApplLimit()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetApplLimit: Ptr; external name '_LMGetApplLimit';
{
 *  LMSetApplLimit()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetApplLimit(value: Ptr); external name '_LMSetApplLimit';
{
 *  LMGetSysEvtMask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSysEvtMask: SInt16; external name '_LMGetSysEvtMask';
{  Carbon Usage: use SetEventMask }
{
 *  LMSetSysEvtMask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSysEvtMask(value: SInt16); external name '_LMSetSysEvtMask';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetRndSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetRndSeed: SInt32; external name '_LMGetRndSeed';
{
 *  LMSetRndSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetRndSeed(value: SInt32); external name '_LMSetRndSeed';
{
 *  LMGetSEvtEnb()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSEvtEnb: ByteParameter; external name '_LMGetSEvtEnb';
{
 *  LMSetSEvtEnb()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSEvtEnb(value: ByteParameter); external name '_LMSetSEvtEnb';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetTicks()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTicks: UInt32; external name '_LMGetTicks';
{
 *  LMSetTicks()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTicks(value: UInt32); external name '_LMSetTicks';
{
 *  LMGetVIA()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetVIA: Ptr; external name '_LMGetVIA';
{
 *  LMSetVIA()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetVIA(value: Ptr); external name '_LMSetVIA';
{
 *  LMGetSCCRd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSCCRd: Ptr; external name '_LMGetSCCRd';
{
 *  LMSetSCCRd()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSCCRd(value: Ptr); external name '_LMSetSCCRd';
{
 *  LMGetSCCWr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSCCWr: Ptr; external name '_LMGetSCCWr';
{
 *  LMSetSCCWr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSCCWr(value: Ptr); external name '_LMSetSCCWr';
{
 *  LMGetSPValid()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSPValid: ByteParameter; external name '_LMGetSPValid';
{
 *  LMSetSPValid()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSPValid(value: ByteParameter); external name '_LMSetSPValid';
{
 *  LMGetSPATalkA()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSPATalkA: ByteParameter; external name '_LMGetSPATalkA';
{
 *  LMSetSPATalkA()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSPATalkA(value: ByteParameter); external name '_LMSetSPATalkA';
{
 *  LMGetSPATalkB()
=======
procedure LMSetCurPitch( value: SInt16 ); external name '_LMSetCurPitch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetROM85()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
 *  LMSetROM85()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSPATalkB: ByteParameter; external name '_LMGetSPATalkB';
{
 *  LMSetSPATalkB()
=======


{
 *  LMGetPortBUse()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSPATalkB(value: ByteParameter); external name '_LMSetSPATalkB';
{
 *  LMGetSPConfig()
=======


{
 *  LMSetPortBUse()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSPConfig: ByteParameter; external name '_LMGetSPConfig';
{
 *  LMSetSPConfig()
=======


{
 *  LMGetGNEFilter()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSPConfig(value: ByteParameter); external name '_LMSetSPConfig';
{
 *  LMGetSPPortA()
=======


{
 *  LMSetGNEFilter()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSPPortA: SInt16; external name '_LMGetSPPortA';
{
 *  LMSetSPPortA()
=======


{
 *  LMGetROMBase()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSPPortA(value: SInt16); external name '_LMSetSPPortA';
{
 *  LMGetSPPortB()
=======


{
 *  LMSetROMBase()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSPPortB: SInt16; external name '_LMGetSPPortB';
{
 *  LMSetSPPortB()
=======


{
 *  LMGetRAMBase()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSPPortB(value: SInt16); external name '_LMSetSPPortB';
{
 *  LMGetSPAlarm()
=======


{
 *  LMSetRAMBase()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSPAlarm: SInt32; external name '_LMGetSPAlarm';
{
 *  LMSetSPAlarm()
=======


{
 *  LMGetDSAlertTab()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetSPAlarm(value: SInt32); external name '_LMSetSPAlarm';
{
 *  LMGetSPFont()
=======


{
 *  LMSetDSAlertTab()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetSPFont: SInt16; external name '_LMGetSPFont';
{
 *  LMSetSPFont()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSPFont(value: SInt16); external name '_LMSetSPFont';
{
 *  LMGetSPKbd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSPKbd: ByteParameter; external name '_LMGetSPKbd';
{
 *  LMSetSPKbd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSPKbd(value: ByteParameter); external name '_LMSetSPKbd';
{
 *  LMGetSPPrint()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSPPrint: ByteParameter; external name '_LMGetSPPrint';
{
 *  LMSetSPPrint()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSPPrint(value: ByteParameter); external name '_LMSetSPPrint';
{
 *  LMGetSPVolCtl()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSPVolCtl: ByteParameter; external name '_LMGetSPVolCtl';
{
 *  LMSetSPVolCtl()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSPVolCtl(value: ByteParameter); external name '_LMSetSPVolCtl';
{
 *  LMGetSPClikCaret()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSPClikCaret: ByteParameter; external name '_LMGetSPClikCaret';
{
 *  LMSetSPClikCaret()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSPClikCaret(value: ByteParameter); external name '_LMSetSPClikCaret';
{
 *  LMGetSPMisc2()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSPMisc2: ByteParameter; external name '_LMGetSPMisc2';
{
 *  LMSetSPMisc2()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSPMisc2(value: ByteParameter); external name '_LMSetSPMisc2';
{  Carbon Usage: use GetDateTime }
{
 *  LMGetTime()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTime: SInt32; external name '_LMGetTime';
{  Carbon Usage: use SetDateTime }
{
 *  LMSetTime()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTime(value: SInt32); external name '_LMSetTime';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetBootDrive()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetBootDrive: SInt16; external name '_LMGetBootDrive';
{
 *  LMSetBootDrive()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetBootDrive(value: SInt16); external name '_LMSetBootDrive';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetSFSaveDisk()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSFSaveDisk: SInt16; external name '_LMGetSFSaveDisk';
{
 *  LMSetSFSaveDisk()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSFSaveDisk(value: SInt16); external name '_LMSetSFSaveDisk';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetMemErr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetMemErr: SInt16; external name '_LMGetMemErr';
{
 *  LMSetMemErr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetMemErr(value: SInt16); external name '_LMSetMemErr';
{
 *  LMGetSdVolume()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSdVolume: ByteParameter; external name '_LMGetSdVolume';
{
 *  LMSetSdVolume()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSdVolume(value: ByteParameter); external name '_LMSetSdVolume';
{
 *  LMGetSoundPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSoundPtr: Ptr; external name '_LMGetSoundPtr';
{
 *  LMSetSoundPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSoundPtr(value: Ptr); external name '_LMSetSoundPtr';
{
 *  LMGetSoundBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSoundBase: Ptr; external name '_LMGetSoundBase';
{
 *  LMSetSoundBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSoundBase(value: Ptr); external name '_LMSetSoundBase';
{
 *  LMGetSoundLevel()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSoundLevel: ByteParameter; external name '_LMGetSoundLevel';
{
 *  LMSetSoundLevel()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSoundLevel(value: ByteParameter); external name '_LMSetSoundLevel';
{
 *  LMGetCurPitch()
=======


{
    NOTE:   LMGetABusVars and LMSetABusVars have been removed.
            Their implememtation in InterfaceLib was inconsistent
            with their prototypes here.  In InterfaceLib LMSetABusVars 
            would copy eight bytes and LMGetABusVars would return the
            value 0x02D8 instead of the long at that location.
            
            Use LMGetABusGlobals/LMSetABusGlobals to get/set the
            long at location 0x02D8 which is a pointer to the AppleTalk
            globals.  Use LMGetABusDCE/LMSetABusDCE to get/set the
            long at location 0x02DC which is the .MPP driver
            Device Control Entry. 
            
}
{
 *  LMGetABusGlobals()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMGetABusDCE()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMSetABusGlobals()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{
 *  LMSetABusDCE()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }


{$ifc not TARGET_CPU_64}
{
 *  LMGetScrDmpEnb()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetCurPitch: SInt16; external name '_LMGetCurPitch';
{
 *  LMSetCurPitch()
=======
 }
function LMGetScrDmpEnb: UInt8; external name '_LMGetScrDmpEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetScrDmpEnb()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetCurPitch(value: SInt16); external name '_LMSetCurPitch';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetROM85()
=======
 }
procedure LMSetScrDmpEnb( value: ByteParameter ); external name '_LMSetScrDmpEnb';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetBufTgFNum()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetROM85: SInt16; external name '_LMGetROM85';
{
 *  LMSetROM85()
=======
function LMGetBufTgFNum: SInt32; external name '_LMGetBufTgFNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFNum()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetROM85(value: SInt16); external name '_LMSetROM85';
{
 *  LMGetPortBUse()
=======
procedure LMSetBufTgFNum( value: SInt32 ); external name '_LMSetBufTgFNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgFFlg()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetPortBUse: ByteParameter; external name '_LMGetPortBUse';
{
 *  LMSetPortBUse()
=======
function LMGetBufTgFFlg: SInt16; external name '_LMGetBufTgFFlg';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFFlg()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetPortBUse(value: ByteParameter); external name '_LMSetPortBUse';
{
 *  LMGetGNEFilter()
=======
procedure LMSetBufTgFFlg( value: SInt16 ); external name '_LMSetBufTgFFlg';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgFBkNum()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetGNEFilter: GetNextEventFilterUPP; external name '_LMGetGNEFilter';
{
 *  LMSetGNEFilter()
=======
function LMGetBufTgFBkNum: SInt16; external name '_LMGetBufTgFBkNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgFBkNum()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetGNEFilter(value: GetNextEventFilterUPP); external name '_LMSetGNEFilter';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetSysZone()
=======
procedure LMSetBufTgFBkNum( value: SInt16 ); external name '_LMSetBufTgFBkNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetBufTgDate()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSysZone: THz; external name '_LMGetSysZone';
{
 *  LMSetSysZone()
=======
 }
function LMGetBufTgDate: SInt32; external name '_LMGetBufTgDate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetBufTgDate()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSysZone(value: THz); external name '_LMSetSysZone';
{
 *  LMGetApplZone()
=======
 }
procedure LMSetBufTgDate( value: SInt32 ); external name '_LMSetBufTgDate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetMinStack()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetApplZone: THz; external name '_LMGetApplZone';
{
 *  LMSetApplZone()
=======
 }
function LMGetMinStack: SInt32; external name '_LMGetMinStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetMinStack()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetApplZone(value: THz); external name '_LMSetApplZone';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetROMBase()
=======
 }
procedure LMSetMinStack( value: SInt32 ); external name '_LMSetMinStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetDefltStack()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetROMBase: Ptr; external name '_LMGetROMBase';
{
 *  LMSetROMBase()
=======
function LMGetDefltStack: SInt32; external name '_LMGetDefltStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetDefltStack()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetROMBase(value: Ptr); external name '_LMSetROMBase';
{
 *  LMGetRAMBase()
=======
procedure LMSetDefltStack( value: SInt32 ); external name '_LMSetDefltStack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetGZRootHnd()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetRAMBase: Ptr; external name '_LMGetRAMBase';
{
 *  LMSetRAMBase()
=======
function LMGetGZRootHnd: Handle; external name '_LMGetGZRootHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetGZRootHnd()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetRAMBase(value: Ptr); external name '_LMSetRAMBase';
{
 *  LMGetDSAlertTab()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDSAlertTab: Ptr; external name '_LMGetDSAlertTab';
{
 *  LMSetDSAlertTab()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDSAlertTab(value: Ptr); external name '_LMSetDSAlertTab';
{
    NOTE:   LMGetABusVars and LMSetABusVars have been removed.
            Their implememtation in InterfaceLib was inconsistent
            with their prototypes here.  In InterfaceLib LMSetABusVars 
            would copy eight bytes and LMGetABusVars would return the
            value 0x02D8 instead of the long at that location.
            
            Use LMGetABusGlobals/LMSetABusGlobals to get/set the
            long at location 0x02D8 which is a pointer to the AppleTalk
            globals.  Use LMGetABusDCE/LMSetABusDCE to get/set the
            long at location 0x02DC which is the .MPP driver
            Device Control Entry. 
            
}
{
 *  LMGetABusGlobals()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetABusGlobals: Ptr; external name '_LMGetABusGlobals';
{
 *  LMGetABusDCE()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetABusDCE: Ptr; external name '_LMGetABusDCE';
{
 *  LMSetABusGlobals()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetABusGlobals(value: Ptr); external name '_LMSetABusGlobals';
{
 *  LMSetABusDCE()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetABusDCE(value: Ptr); external name '_LMSetABusDCE';
{  Carbon Usage: use GetDblTime }
{
 *  LMGetDoubleTime()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDoubleTime: UInt32; external name '_LMGetDoubleTime';
{
 *  LMSetDoubleTime()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDoubleTime(value: UInt32); external name '_LMSetDoubleTime';
{  Carbon Usage: use GetCaretTime }
{
 *  LMGetCaretTime()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCaretTime: UInt32; external name '_LMGetCaretTime';
{
 *  LMSetCaretTime()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCaretTime(value: UInt32); external name '_LMSetCaretTime';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetScrDmpEnb()
=======
procedure LMSetGZRootHnd( value: Handle ); external name '_LMSetGZRootHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetGZMoveHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetGZMoveHnd: Handle; external name '_LMGetGZMoveHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetGZMoveHnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetGZMoveHnd( value: Handle ); external name '_LMSetGZMoveHnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetToExtFS()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetToExtFS: UniversalProcPtr; external name '_LMGetToExtFS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetToExtFS()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetToExtFS( value: UniversalProcPtr ); external name '_LMSetToExtFS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetJStash()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetJStash: UniversalProcPtr; external name '_LMGetJStash';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetJStash()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetJStash( value: UniversalProcPtr ); external name '_LMSetJStash';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurApRefNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApRefNum: FSIORefNum; external name '_LMGetCurApRefNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurApRefNum()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApRefNum( value: FSIORefNum ); external name '_LMSetCurApRefNum';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    If using the Thread Manager APIs, use ThreadCurrentStackSpace(). 
 *    If using pthreads(), get the stack with
 *    pthread_get_stackaddr_np().
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurStackBase: Ptr; external name '_LMGetCurStackBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetCurStackBase()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not support setting the stack base.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurStackBase( value: Ptr ); external name '_LMSetCurStackBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetCurPageOption()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetScrDmpEnb: ByteParameter; external name '_LMGetScrDmpEnb';
{
 *  LMSetScrDmpEnb()
=======
function LMGetCurPageOption: SInt16; external name '_LMGetCurPageOption';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetCurPageOption()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetScrDmpEnb(value: ByteParameter); external name '_LMSetScrDmpEnb';
{
 *  LMGetBufTgFNum()
=======
procedure LMSetCurPageOption( value: SInt16 ); external name '_LMSetCurPageOption';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetPrintErr()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetBufTgFNum: SInt32; external name '_LMGetBufTgFNum';
{
 *  LMSetBufTgFNum()
=======
function LMGetPrintErr: SInt16; external name '_LMGetPrintErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetPrintErr()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetBufTgFNum(value: SInt32); external name '_LMSetBufTgFNum';
{
 *  LMGetBufTgFFlg()
=======
procedure LMSetPrintErr( value: SInt16 ); external name '_LMSetPrintErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetApFontID()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the application font.  Use GetAppFont() in the
 *    Quickdraw framework instead of this.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetBufTgFFlg: SInt16; external name '_LMGetBufTgFFlg';
{
 *  LMSetBufTgFFlg()
=======
function LMGetApFontID: SInt16; external name '_LMGetApFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetApFontID()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set the id of the application font.  Don't use this call anymore;
 *    this functionality is not supported on Mac OS X.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetBufTgFFlg(value: SInt16); external name '_LMSetBufTgFFlg';
{
 *  LMGetBufTgFBkNum()
=======
procedure LMSetApFontID( value: SInt16 ); external name '_LMSetApFontID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$ifc not TARGET_CPU_64}
{
 *  LMGetOneOne()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetBufTgFBkNum: SInt16; external name '_LMGetBufTgFBkNum';
{
 *  LMSetBufTgFBkNum()
=======
 }
function LMGetOneOne: SInt32; external name '_LMGetOneOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetOneOne()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetBufTgFBkNum(value: SInt16); external name '_LMSetBufTgFBkNum';
{
 *  LMGetBufTgDate()
=======
 }
procedure LMSetOneOne( value: SInt32 ); external name '_LMSetOneOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetMinusOne()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetBufTgDate: SInt32; external name '_LMGetBufTgDate';
{
 *  LMSetBufTgDate()
=======
 }
function LMGetMinusOne: SInt32; external name '_LMGetMinusOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetMinusOne()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetBufTgDate(value: SInt32); external name '_LMSetBufTgDate';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetLo3Bytes()
=======
 }
procedure LMSetMinusOne( value: SInt32 ); external name '_LMSetMinusOne';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMGetSysMap()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetLo3Bytes: SInt32; external name '_LMGetLo3Bytes';
{
 *  LMSetLo3Bytes()
=======
function LMGetSysMap: SInt16; external name '_LMGetSysMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetSysMap()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetLo3Bytes(value: SInt32); external name '_LMSetLo3Bytes';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetMinStack()
=======
procedure LMSetSysMap( value: SInt16 ); external name '_LMSetSysMap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetResLoad()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetMinStack: SInt32; external name '_LMGetMinStack';
{
 *  LMSetMinStack()
=======
function LMGetResLoad: UInt8; external name '_LMGetResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetResLoad()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetMinStack(value: SInt32); external name '_LMSetMinStack';
{
 *  LMGetDefltStack()
=======
procedure LMSetResLoad( value: ByteParameter ); external name '_LMSetResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetResErr()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetDefltStack: SInt32; external name '_LMGetDefltStack';
{
 *  LMSetDefltStack()
=======
function LMGetResErr: SInt16; external name '_LMGetResErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetResErr()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetDefltStack(value: SInt32); external name '_LMSetDefltStack';
{
 *  LMGetGZRootHnd()
=======
procedure LMSetResErr( value: SInt16 ); external name '_LMSetResErr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetTmpResLoad()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetGZRootHnd: Handle; external name '_LMGetGZRootHnd';
{
 *  LMSetGZRootHnd()
=======
 }
function LMGetTmpResLoad: UInt8; external name '_LMGetTmpResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetTmpResLoad()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetGZRootHnd(value: Handle); external name '_LMSetGZRootHnd';
{
 *  LMGetGZMoveHnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetGZMoveHnd: Handle; external name '_LMGetGZMoveHnd';
{
 *  LMSetGZMoveHnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetGZMoveHnd(value: Handle); external name '_LMSetGZMoveHnd';
{
   LMGetFCBSPtr, LMSetFCBSPtr and LMSetFSFCBLen are not supported with Mac OS 9
   and later. Access to information in File Control Blocks or Fork Control Blocks
   (FCBs) should, if at all possible, be made with the GetFCBInfo or GetForkCBInfo
   routines. See the Technote "FCBs, Now and Forever" or the Technical Q&A
   "Accessing File Control Blocks" for complete information on this subject.
   Direct access to FCBs is not allowed by Carbon. Non-Carbon programs that
   require direct access to FCBs should use the File System Manager (FSM) FCB accessor
   functions if FSM is available (use the Gestalt selector gestaltFSAttr to determine
   this). Non-Carbon programs needing direct access to FCBs when FSM is not available
   can define ENABLE_FCB_ARRAY_ACCESS to be true when compiling.
}
{$ifc undefined ENABLE_FCB_ARRAY_ACCESS}
{$setc ENABLE_FCB_ARRAY_ACCESS := 0}
{$endc}

{$ifc ENABLE_FCB_ARRAY_ACCESS}
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetFCBSPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetFCBSPtr: Ptr; external name '_LMGetFCBSPtr';
{
 *  LMSetFCBSPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetFCBSPtr(value: Ptr); external name '_LMSetFCBSPtr';
{
 *  LMSetFSFCBLen()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetFSFCBLen(value: SInt16); external name '_LMSetFSFCBLen';
{$endc}  {CALL_NOT_IN_CARBON}
{$endc}  {ENABLE_FCB_ARRAY_ACCESS}

{
   LMGetFSFCBLen is supported only for the purpose of determining that the HFS
   file system is available as documented in developer Technotes (the HFS file system
   is available in System 3.2 and later). There is no documented use of FSFCBLen
   other than testing it for a positive value.
}
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetFSFCBLen()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetFSFCBLen: SInt16; external name '_LMGetFSFCBLen';
{
 *  LMGetDefVCBPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDefVCBPtr: Ptr; external name '_LMGetDefVCBPtr';
{
 *  LMSetDefVCBPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDefVCBPtr(value: Ptr); external name '_LMSetDefVCBPtr';
{
 *  LMGetCurDirStore()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCurDirStore: SInt32; external name '_LMGetCurDirStore';
{
 *  LMSetCurDirStore()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCurDirStore(value: SInt32); external name '_LMSetCurDirStore';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetToExtFS()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetToExtFS: UniversalProcPtr; external name '_LMGetToExtFS';
{
 *  LMSetToExtFS()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetToExtFS(value: UniversalProcPtr); external name '_LMSetToExtFS';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetScrnBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetScrnBase: Ptr; external name '_LMGetScrnBase';
{
 *  LMSetScrnBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetScrnBase(value: Ptr); external name '_LMSetScrnBase';
{
 *  LMGetCrsrBusy()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCrsrBusy: ByteParameter; external name '_LMGetCrsrBusy';
{
 *  LMSetCrsrBusy()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCrsrBusy(value: ByteParameter); external name '_LMSetCrsrBusy';
{
 *  LMGetJournalRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetJournalRef: SInt16; external name '_LMGetJournalRef';
{
 *  LMSetJournalRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetJournalRef(value: SInt16); external name '_LMSetJournalRef';
{
 *  LMGetCrsrThresh()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCrsrThresh: SInt16; external name '_LMGetCrsrThresh';
{
 *  LMSetCrsrThresh()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCrsrThresh(value: SInt16); external name '_LMSetCrsrThresh';
{
 *  LMGetJFetch()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetJFetch: UniversalProcPtr; external name '_LMGetJFetch';
{
 *  LMSetJFetch()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetJFetch(value: UniversalProcPtr); external name '_LMSetJFetch';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetJStash()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetJStash: UniversalProcPtr; external name '_LMGetJStash';
{
 *  LMSetJStash()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetJStash(value: UniversalProcPtr); external name '_LMSetJStash';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetJIODone()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetJIODone: UniversalProcPtr; external name '_LMGetJIODone';
{
 *  LMSetJIODone()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetJIODone(value: UniversalProcPtr); external name '_LMSetJIODone';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetCurApRefNum()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetCurApRefNum: SInt16; external name '_LMGetCurApRefNum';
{
 *  LMSetCurApRefNum()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetCurApRefNum(value: SInt16); external name '_LMSetCurApRefNum';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetCurrentA5()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCurrentA5: Ptr; external name '_LMGetCurrentA5';
{
 *  LMSetCurrentA5()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCurrentA5(value: Ptr); external name '_LMSetCurrentA5';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetCurStackBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetCurStackBase: Ptr; external name '_LMGetCurStackBase';
{
 *  LMSetCurStackBase()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetCurStackBase(value: Ptr); external name '_LMSetCurStackBase';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetCurJTOffset()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCurJTOffset: SInt16; external name '_LMGetCurJTOffset';
{
 *  LMSetCurJTOffset()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCurJTOffset(value: SInt16); external name '_LMSetCurJTOffset';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetCurPageOption()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetCurPageOption: SInt16; external name '_LMGetCurPageOption';
{
 *  LMSetCurPageOption()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetCurPageOption(value: SInt16); external name '_LMSetCurPageOption';
{
 *  LMGetPrintErr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetPrintErr: SInt16; external name '_LMGetPrintErr';
{
 *  LMSetPrintErr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetPrintErr(value: SInt16); external name '_LMSetPrintErr';
{   Carbon Scrap Manager does not support low memory. }

{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetScrapSize()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetScrapSize: SInt32; external name '_LMGetScrapSize';
{
 *  LMSetScrapSize()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetScrapSize(value: SInt32); external name '_LMSetScrapSize';
{
 *  LMGetScrapHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetScrapHandle: Handle; external name '_LMGetScrapHandle';
{
 *  LMSetScrapHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetScrapHandle(value: Handle); external name '_LMSetScrapHandle';
{
 *  LMGetScrapCount()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetScrapCount: SInt16; external name '_LMGetScrapCount';
{
 *  LMSetScrapCount()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetScrapCount(value: SInt16); external name '_LMSetScrapCount';
{
 *  LMGetScrapState()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetScrapState: SInt16; external name '_LMGetScrapState';
{
 *  LMSetScrapState()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetScrapState(value: SInt16); external name '_LMSetScrapState';
{
 *  LMGetScrapName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetScrapName: StringPtr; external name '_LMGetScrapName';
{
 *  LMSetScrapName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetScrapName(value: StringPtr); external name '_LMSetScrapName';
{
 *  LMGetROMFont0()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetROMFont0: Handle; external name '_LMGetROMFont0';
{
 *  LMSetROMFont0()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetROMFont0(value: Handle); external name '_LMSetROMFont0';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetApFontID()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetApFontID: SInt16; external name '_LMGetApFontID';
{
 *  LMSetApFontID()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetApFontID(value: SInt16); external name '_LMSetApFontID';
{ Carbon versions of the Window Manager do not support LowMem. }
{  Carbon Usage: use GetWindowList }
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetWindowList()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetWindowList: WindowRef; external name '_LMGetWindowList';
{
 *  LMGetSaveUpdate()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSaveUpdate: SInt16; external name '_LMGetSaveUpdate';
{
 *  LMSetSaveUpdate()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSaveUpdate(value: SInt16); external name '_LMSetSaveUpdate';
{
 *  LMGetPaintWhite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetPaintWhite: SInt16; external name '_LMGetPaintWhite';
{  Carbon Usage : use InstallWindowContentPaintProc }
{
 *  LMSetPaintWhite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetPaintWhite(value: SInt16); external name '_LMSetPaintWhite';
{
 *  LMGetWMgrPort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetWMgrPort: GrafPtr; external name '_LMGetWMgrPort';
{
 *  LMSetWMgrPort()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetWMgrPort(value: GrafPtr); external name '_LMSetWMgrPort';
{  Carbon Usage: use GetGrayRgn }
{
 *  LMGetGrayRgn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetGrayRgn: RgnHandle; external name '_LMGetGrayRgn';
{
 *  LMGetDragHook()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDragHook: DragGrayRgnUPP; external name '_LMGetDragHook';
{
 *  LMSetDragHook()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDragHook(value: DragGrayRgnUPP); external name '_LMSetDragHook';
{
 *  LMSetWindowList()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetWindowList(value: WindowRef); external name '_LMSetWindowList';
{
 *  LMGetGhostWindow()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetGhostWindow: WindowRef; external name '_LMGetGhostWindow';
{
 *  LMSetGhostWindow()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetGhostWindow(value: WindowRef); external name '_LMSetGhostWindow';
{$endc}  {CALL_NOT_IN_CARBON}

{$ifc NOT OPAQUE_TOOLBOX_STRUCTS}
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetAuxWinHead()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetAuxWinHead: AuxWinHandle; external name '_LMGetAuxWinHead';
{
 *  LMSetAuxWinHead()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetAuxWinHead(value: AuxWinHandle); external name '_LMSetAuxWinHead';
{$endc}  {CALL_NOT_IN_CARBON}
{$endc}

{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetCurActivate()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCurActivate: WindowRef; external name '_LMGetCurActivate';
{
 *  LMSetCurActivate()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCurActivate(value: WindowRef); external name '_LMSetCurActivate';
{
 *  LMGetCurDeactive()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCurDeactive: WindowRef; external name '_LMGetCurDeactive';
{
 *  LMSetCurDeactive()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCurDeactive(value: WindowRef); external name '_LMSetCurDeactive';
{
 *  LMGetOldStructure()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetOldStructure: RgnHandle; external name '_LMGetOldStructure';
{
 *  LMSetOldStructure()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetOldStructure(value: RgnHandle); external name '_LMSetOldStructure';
{
 *  LMGetOldContent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetOldContent: RgnHandle; external name '_LMGetOldContent';
{
 *  LMSetOldContent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetOldContent(value: RgnHandle); external name '_LMSetOldContent';
{
 *  LMSetGrayRgn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetGrayRgn(value: RgnHandle); external name '_LMSetGrayRgn';
{
 *  LMGetSaveVisRgn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSaveVisRgn: RgnHandle; external name '_LMGetSaveVisRgn';
{
 *  LMSetSaveVisRgn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSaveVisRgn(value: RgnHandle); external name '_LMSetSaveVisRgn';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetOneOne()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetOneOne: SInt32; external name '_LMGetOneOne';
{
 *  LMSetOneOne()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetOneOne(value: SInt32); external name '_LMSetOneOne';
{
 *  LMGetMinusOne()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetMinusOne: SInt32; external name '_LMGetMinusOne';
{
 *  LMSetMinusOne()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetMinusOne(value: SInt32); external name '_LMSetMinusOne';
{  Carbon Usage: use GetMenuTrackingData }
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetTopMenuItem()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTopMenuItem: SInt16; external name '_LMGetTopMenuItem';
{  Carbon Usage: replaced by MDEF messages and GetMenuTrackingData API }
{
 *  LMSetTopMenuItem()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTopMenuItem(value: SInt16); external name '_LMSetTopMenuItem';
{  Carbon Usage: use GetMenuTrackingData }
{
 *  LMGetAtMenuBottom()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetAtMenuBottom: SInt16; external name '_LMGetAtMenuBottom';
{  Carbon Usage: replaced by MDEF messages and GetMenuTrackingData API }
{
 *  LMSetAtMenuBottom()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetAtMenuBottom(value: SInt16); external name '_LMSetAtMenuBottom';
{
   Carbon usage: use GetMenuBar (which returns a newly allocated handle in
   the same format as that returned by LMGetMenuList; dispose with DisposeMenuBar)
   or GetRootMenu.
}
{
 *  LMGetMenuList()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMenuList: Handle; external name '_LMGetMenuList';
{
 *  LMSetMenuList()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMenuList(value: Handle); external name '_LMSetMenuList';
{  Carbon usage: no replacement }
{
 *  LMGetMBarEnable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMBarEnable: SInt16; external name '_LMGetMBarEnable';
{
 *  LMSetMBarEnable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMBarEnable(value: SInt16); external name '_LMSetMBarEnable';
{  Carbon usage: no replacement }
{
 *  LMGetMenuFlash()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMenuFlash: SInt16; external name '_LMGetMenuFlash';
{
 *  LMSetMenuFlash()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMenuFlash(value: SInt16); external name '_LMSetMenuFlash';
{ LMGetTheMenu() moved to Menus.h }
{
 *  LMSetTheMenu()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTheMenu(value: SInt16); external name '_LMSetTheMenu';
{
 *  LMGetMBarHook()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMBarHook: MBarHookUPP; external name '_LMGetMBarHook';
{
 *  LMSetMBarHook()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMBarHook(value: MBarHookUPP); external name '_LMSetMBarHook';
{
 *  LMGetMenuHook()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMenuHook: MenuHookUPP; external name '_LMGetMenuHook';
{
 *  LMSetMenuHook()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMenuHook(value: MenuHookUPP); external name '_LMSetMenuHook';
{
 *  LMGetTopMapHndl()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTopMapHndl: Handle; external name '_LMGetTopMapHndl';
{
 *  LMSetTopMapHndl()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTopMapHndl(value: Handle); external name '_LMSetTopMapHndl';
{
 *  LMGetSysMapHndl()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSysMapHndl: Handle; external name '_LMGetSysMapHndl';
{
 *  LMSetSysMapHndl()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSysMapHndl(value: Handle); external name '_LMSetSysMapHndl';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetSysMap()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSysMap: SInt16; external name '_LMGetSysMap';
{
 *  LMSetSysMap()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSysMap(value: SInt16); external name '_LMSetSysMap';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetCurMap()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCurMap: SInt16; external name '_LMGetCurMap';
{
 *  LMSetCurMap()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetCurMap(value: SInt16); external name '_LMSetCurMap';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetResLoad()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetResLoad: ByteParameter; external name '_LMGetResLoad';
{
 *  LMSetResLoad()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetResLoad(value: ByteParameter); external name '_LMSetResLoad';
{
 *  LMGetResErr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetResErr: SInt16; external name '_LMGetResErr';
{
 *  LMSetResErr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetResErr(value: SInt16); external name '_LMSetResErr';
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetFScaleDisable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetFScaleDisable: ByteParameter; external name '_LMGetFScaleDisable';
{
 *  LMSetFScaleDisable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetFScaleDisable(value: ByteParameter); external name '_LMSetFScaleDisable';
{
 *  LMGetDeskHook()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDeskHook: UniversalProcPtr; external name '_LMGetDeskHook';
{
 *  LMSetDeskHook()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDeskHook(value: UniversalProcPtr); external name '_LMSetDeskHook';
{  Carbon Usage: Use TEGetDoTextHook. }
{
 *  LMGetTEDoText()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTEDoText: UniversalProcPtr; external name '_LMGetTEDoText';
{  Carbon Usage: Use TESetDoTextHook. }
{
 *  LMSetTEDoText()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTEDoText(value: UniversalProcPtr); external name '_LMSetTEDoText';
{  Carbon Usage: Use TEGetRecalcHook. }
{
 *  LMGetTERecal()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTERecal: UniversalProcPtr; external name '_LMGetTERecal';
{  Carbon Usage: Use TESetRecalcHook. }
{
 *  LMSetTERecal()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTERecal(value: UniversalProcPtr); external name '_LMSetTERecal';
{
 *  LMGetResumeProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetResumeProc: UniversalProcPtr; external name '_LMGetResumeProc';
{
 *  LMSetResumeProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetResumeProc(value: UniversalProcPtr); external name '_LMSetResumeProc';
{
 *  LMGetANumber()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetANumber: SInt16; external name '_LMGetANumber';
{
 *  LMSetANumber()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetANumber(value: SInt16); external name '_LMSetANumber';
{  Carbon Usage: Use GetAlertStage. }
{
 *  LMGetACount()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetACount: SInt16; external name '_LMGetACount';
{  Carbon Usage: Use ResetAlertStage. }
{
 *  LMSetACount()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetACount(value: SInt16); external name '_LMSetACount';
{
 *  LMGetDABeeper()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDABeeper: UniversalProcPtr; external name '_LMGetDABeeper';
{
 *  LMSetDABeeper()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDABeeper(value: UniversalProcPtr); external name '_LMSetDABeeper';
{  Carbon Usage: use TEGetScrapLength }
{
 *  LMGetTEScrpLength()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTEScrpLength: UInt16; external name '_LMGetTEScrpLength';
{  Carbon Usage: use TESetScrapLength }
{
 *  LMSetTEScrpLength()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTEScrpLength(value: UInt16); external name '_LMSetTEScrpLength';
{  Carbon Usage: use TEGetScrapHandle }
{
 *  LMGetTEScrpHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTEScrpHandle: Handle; external name '_LMGetTEScrpHandle';
{  Carbon Usage: use TESetScrapHandle }
{
 *  LMSetTEScrpHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTEScrpHandle(value: Handle); external name '_LMSetTEScrpHandle';
{
 *  LMGetAppParmHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetAppParmHandle: Handle; external name '_LMGetAppParmHandle';
{
 *  LMSetAppParmHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetAppParmHandle(value: Handle); external name '_LMSetAppParmHandle';
{
 *  LMGetDSErrCode()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDSErrCode: SInt16; external name '_LMGetDSErrCode';
{
 *  LMSetDSErrCode()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDSErrCode(value: SInt16); external name '_LMSetDSErrCode';
{
 *  LMGetResErrProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetResErrProc: ResErrUPP; external name '_LMGetResErrProc';
{
 *  LMSetResErrProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetResErrProc(value: ResErrUPP); external name '_LMSetResErrProc';
{
 *  LMGetDlgFont()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDlgFont: SInt16; external name '_LMGetDlgFont';
{  Carbon Usage: use SetDialogFont }
{
 *  LMSetDlgFont()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDlgFont(value: SInt16); external name '_LMSetDlgFont';
{
 *  LMGetATalkHk2()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetATalkHk2: Ptr; external name '_LMGetATalkHk2';
{
 *  LMSetATalkHk2()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetATalkHk2(value: Ptr); external name '_LMSetATalkHk2';
{
 *  LMGetHWCfgFlags()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetHWCfgFlags: SInt16; external name '_LMGetHWCfgFlags';
{
 *  LMSetHWCfgFlags()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetHWCfgFlags(value: SInt16); external name '_LMSetHWCfgFlags';
{  Carbon Usage: use GetMenuTrackingData }
{
 *  LMGetMenuDisable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMenuDisable: SInt32; external name '_LMGetMenuDisable';
{  Carbon Usage: use new MDEF messages }
{
 *  LMSetMenuDisable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMenuDisable(value: SInt32); external name '_LMSetMenuDisable';
{
 *  LMGetROMMapInsert()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetROMMapInsert: ByteParameter; external name '_LMGetROMMapInsert';
{
 *  LMSetROMMapInsert()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetROMMapInsert(value: ByteParameter); external name '_LMSetROMMapInsert';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  LMGetTmpResLoad()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetTmpResLoad: ByteParameter; external name '_LMGetTmpResLoad';
{
 *  LMSetTmpResLoad()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetTmpResLoad(value: ByteParameter); external name '_LMSetTmpResLoad';
{
 *  LMGetIntlSpec()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetIntlSpec: Ptr; external name '_LMGetIntlSpec';
{
 *  LMSetIntlSpec()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetIntlSpec(value: Ptr); external name '_LMSetIntlSpec';
{ LMGetWordRedraw and LMSetWordRedraw moved to TextEdit.h }
{
 *  LMGetSysFontFam()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSysFontFam: SInt16; external name '_LMGetSysFontFam';
{
 *  LMSetSysFontFam()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSysFontFam(value: SInt16); external name '_LMSetSysFontFam';
{
 *  LMGetSysFontSize()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetSysFontSize: SInt16; external name '_LMGetSysFontSize';
{
 *  LMSetSysFontSize()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure LMSetSysFontSize(value: SInt16); external name '_LMSetSysFontSize';
{  Carbon Usge: use GetMBarHeight }
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetMBarHeight()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMBarHeight: SInt16; external name '_LMGetMBarHeight';
{  Carbon Usage: use Hide/ShowMenuBar }
{
 *  LMSetMBarHeight()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMBarHeight(value: SInt16); external name '_LMSetMBarHeight';
{
 *  LMGetTESysJust()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTESysJust: SInt16; external name '_LMGetTESysJust';
{
 *  LMSetTESysJust()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTESysJust(value: SInt16); external name '_LMSetTESysJust';
{
 *  LMGetMMU32Bit()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMMU32Bit: ByteParameter; external name '_LMGetMMU32Bit';
{
 *  LMSetMMU32Bit()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMMU32Bit(value: ByteParameter); external name '_LMSetMMU32Bit';
{
 *  LMGetDeskCPat()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDeskCPat: PixPatHandle; external name '_LMGetDeskCPat';
{
 *  LMSetDeskCPat()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDeskCPat(value: PixPatHandle); external name '_LMSetDeskCPat';
{
 *  LMGetTimeDBRA()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTimeDBRA: SInt16; external name '_LMGetTimeDBRA';
{
 *  LMSetTimeDBRA()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTimeDBRA(value: SInt16); external name '_LMSetTimeDBRA';
{
 *  LMGetTimeSCCDB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTimeSCCDB: SInt16; external name '_LMGetTimeSCCDB';
{
 *  LMSetTimeSCCDB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTimeSCCDB(value: SInt16); external name '_LMSetTimeSCCDB';
{
 *  LMGetJVBLTask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetJVBLTask: UniversalProcPtr; external name '_LMGetJVBLTask';
{
 *  LMSetJVBLTask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetJVBLTask(value: UniversalProcPtr); external name '_LMSetJVBLTask';
{
 *  LMGetSynListHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSynListHandle: Handle; external name '_LMGetSynListHandle';
{
 *  LMSetSynListHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSynListHandle(value: Handle); external name '_LMSetSynListHandle';
{
 *  LMGetMenuCInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMenuCInfo: MCTableHandle; external name '_LMGetMenuCInfo';
{
 *  LMSetMenuCInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMenuCInfo(value: MCTableHandle); external name '_LMSetMenuCInfo';
{
 *  LMGetJDTInstall()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetJDTInstall: UniversalProcPtr; external name '_LMGetJDTInstall';
{
 *  LMSetJDTInstall()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetJDTInstall(value: UniversalProcPtr); external name '_LMSetJDTInstall';
{
 *  LMGetTimeSCSIDB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTimeSCSIDB: SInt16; external name '_LMGetTimeSCSIDB';
{
 *  LMSetTimeSCSIDB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTimeSCSIDB(value: SInt16); external name '_LMSetTimeSCSIDB';
{*************************************************************************************

    MORE COMPLEX LOWMEM ACCESSORS

*************************************************************************************}
{$endc}  {CALL_NOT_IN_CARBON}

{$ifc TARGET_CPU_68K AND NOT TARGET_RT_MAC_CFM}
{*************************************************************************************
    "BIG DATA"
    
        These lowmem accessors access big (> 4 bytes) values.
*************************************************************************************}
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetDeskPattern()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMGetDeskPattern(var deskPatternValue: Pattern); external name '_LMGetDeskPattern';
{
 *  LMSetDeskPattern()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDeskPattern(const (*var*) deskPatternValue: Pattern); external name '_LMSetDeskPattern';
{$endc}  {CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{*************************************************************************************
    "BLOCKMOVE ACCESSORS"
    
        These lowmem accessors use the BlockMove to set
*************************************************************************************}
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  LMGetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApName: StringPtr; external name '_LMGetCurApName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApName( curApNameValue: ConstStr31Param ); external name '_LMSetCurApName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysResName: StringPtr; external name '_LMGetSysResName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysResName( sysResNameValue: ConstStr15Param ); external name '_LMSetSysResName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Getting the name of the Finder isn't terribly useful.  If you
 *    need to find the Finder process, look it up with the CFBundle
 *    calls by bundle id ( "com.apple.finder" ).  Remove usage of this
 *    lowmem value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetFinderName: StringPtr; external name '_LMGetFinderName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Setting the name of the Finder application is not supported on
 *    Mac OS X.  Remove usage of this lowmem accessor.
=======
 *  LMSetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set the id of the system font family.  This is not supported on
 *    Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysFontFam( value: SInt16 ); external name '_LMSetSysFontFam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetSysFontSize()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the size of the system font.  Use DefFontSize() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysFontSize: SInt16; external name '_LMGetSysFontSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSysFontSize()
 *  
 *  Summary:
 *    Set the size for the system font.  This is not supported on Mac
 *    OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysFontSize( value: SInt16 ); external name '_LMSetSysFontSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{*************************************************************************************
    "BLOCKMOVE ACCESSORS"
    
        These lowmem accessors use the BlockMove to set
*************************************************************************************}
{$ifc not TARGET_CPU_64}
{
 *  LMGetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApName: StringPtr; external name '_LMGetCurApName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApName( curApNameValue: ConstStr31Param ); external name '_LMSetCurApName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetFinderName( finderNameValue: ConstStr15Param ); external name '_LMSetFinderName';
=======
function LMGetSysResName: StringPtr; external name '_LMGetSysResName';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetIntlSpec: Ptr; external name '_LMGetIntlSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetIntlSpec()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetIntlSpec( value: Ptr ); external name '_LMSetIntlSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew

{ LMGetWordRedraw and LMSetWordRedraw moved to TextEdit.h }
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  LMGetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
>>>>>>> graemeg/cpstrnew
=======
 *  LMSetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
>>>>>>> graemeg/cpstrnew
=======
 *  LMGetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the system font family.  Use GetSysFont() in the
 *    Quickdraw framework instead of this.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function LMGetSysFontFam: SInt16; external name '_LMGetSysFontFam';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
function LMGetSysFontFam: SInt16; external name '_LMGetSysFontFam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}

{
 *  LMSetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set the id of the system font family.  This is not supported on
 *    Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysFontFam( value: SInt16 ); external name '_LMSetSysFontFam';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  LMGetSysFontSize()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the size of the system font.  Use DefFontSize() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysFontSize: SInt16; external name '_LMGetSysFontSize';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  LMSetSysFontSize()
 *  
 *  Summary:
 *    Set the size for the system font.  This is not supported on Mac
 *    OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysFontSize( value: SInt16 ); external name '_LMSetSysFontSize';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
=======
procedure LMSetSysResName( sysResNameValue: ConstStr15Param ); external name '_LMSetSysResName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Getting the name of the Finder isn't terribly useful.  If you
 *    need to find the Finder process, look it up with the CFBundle
 *    calls by bundle id ( "com.apple.finder" ).  Remove usage of this
 *    lowmem value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetFinderName: StringPtr; external name '_LMGetFinderName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Setting the name of the Finder application is not supported on
 *    Mac OS X.  Remove usage of this lowmem accessor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetFinderName( finderNameValue: ConstStr15Param ); external name '_LMSetFinderName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTimeSCCDB(value: SInt16); external name '_LMSetTimeSCCDB';
{
 *  LMGetJVBLTask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetJVBLTask: UniversalProcPtr; external name '_LMGetJVBLTask';
{
 *  LMSetJVBLTask()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetJVBLTask(value: UniversalProcPtr); external name '_LMSetJVBLTask';
{
 *  LMGetSynListHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSynListHandle: Handle; external name '_LMGetSynListHandle';
{
 *  LMSetSynListHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetSynListHandle(value: Handle); external name '_LMSetSynListHandle';
{
 *  LMGetMenuCInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMenuCInfo: MCTableHandle; external name '_LMGetMenuCInfo';
{
 *  LMSetMenuCInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMenuCInfo(value: MCTableHandle); external name '_LMSetMenuCInfo';
{
 *  LMGetJDTInstall()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetJDTInstall: UniversalProcPtr; external name '_LMGetJDTInstall';
{
 *  LMSetJDTInstall()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetJDTInstall(value: UniversalProcPtr); external name '_LMSetJDTInstall';
{
 *  LMGetTimeSCSIDB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetTimeSCSIDB: SInt16; external name '_LMGetTimeSCSIDB';
{
 *  LMSetTimeSCSIDB()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetTimeSCSIDB(value: SInt16); external name '_LMSetTimeSCSIDB';
{*************************************************************************************

    MORE COMPLEX LOWMEM ACCESSORS

*************************************************************************************}
{$endc}  {CALL_NOT_IN_CARBON}

{$ifc TARGET_CPU_68K AND NOT TARGET_RT_MAC_CFM}
{*************************************************************************************
    "BIG DATA"
    
        These lowmem accessors access big (> 4 bytes) values.
*************************************************************************************}
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetDeskPattern()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMGetDeskPattern(var deskPatternValue: Pattern); external name '_LMGetDeskPattern';
{
 *  LMSetDeskPattern()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDeskPattern(const (*var*) deskPatternValue: Pattern); external name '_LMSetDeskPattern';
{$endc}  {CALL_NOT_IN_CARBON}
{*************************************************************************************
    "BLOCKMOVE ACCESSORS"
    
        These lowmem accessors use the BlockMove to set
*************************************************************************************}
>>>>>>> origin/fixes_2_2
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetCurApName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetCurApName: StringPtr; external name '_LMGetCurApName';
{
 *  LMGetSysResName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetSysResName: StringPtr; external name '_LMGetSysResName';
{
 *  LMGetFinderName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetFinderName: StringPtr; external name '_LMGetFinderName';
{$endc}  {CALL_NOT_IN_CARBON}
{*************************************************************************************
    "INDEXED ACCESSORS"
    
        These lowmem accessors take an index parameter to get/set an indexed
        lowmem global.
*************************************************************************************}
{************************************************************************************
    The DAString accessors are gone with Carbon. Please use ParamText and
    GetParamText instead.
*************************************************************************************}
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetDAStrings()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDAStrings(whichString: SInt16): StringHandle; external name '_LMGetDAStrings';
{
 *  LMSetDAStrings()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDAStrings(stringsValue: StringHandle; whichString: SInt16); external name '_LMSetDAStrings';
{$endc}  {CALL_NOT_IN_CARBON}

{$elsec}
{*************************************************************************************
    "BIG DATA"
    
        These lowmem accessors access big (> 4 bytes) values.
*************************************************************************************}

{$ifc CALL_NOT_IN_CARBON}
{
 *  LMGetDSAlertRect()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMGetDSAlertRect(var dsAlertRectValue: Rect); external name '_LMGetDSAlertRect';

{
 *  LMSetDSAlertRect()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDSAlertRect(const (*var*) dsAlertRectValue: Rect); external name '_LMSetDSAlertRect';

{
 *  LMGetDragPattern()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMGetDragPattern(var dragPatternValue: Pattern); external name '_LMGetDragPattern';

{
 *  LMSetDragPattern()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDragPattern(const (*var*) dragPatternValue: Pattern); external name '_LMSetDragPattern';

{
 *  LMGetDeskPattern()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMGetDeskPattern(var deskPatternValue: Pattern); external name '_LMGetDeskPattern';

{
 *  LMSetDeskPattern()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDeskPattern(const (*var*) deskPatternValue: Pattern); external name '_LMSetDeskPattern';

{
 *  LMGetEventQueue()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetEventQueue: QHdrPtr; external name '_LMGetEventQueue';

{
 *  LMSetEventQueue()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetEventQueue(eventQueueValue: QHdrPtr); external name '_LMSetEventQueue';


{
 *  LMGetVBLQueue()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetVBLQueue: QHdrPtr; external name '_LMGetVBLQueue';

{
 *  LMSetVBLQueue()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetVBLQueue(vblQueueValue: QHdrPtr); external name '_LMSetVBLQueue';

{
 *  LMGetDrvQHdr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDrvQHdr: QHdrPtr; external name '_LMGetDrvQHdr';

{
 *  LMSetDrvQHdr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDrvQHdr(drvQHdrValue: QHdrPtr); external name '_LMSetDrvQHdr';

{
 *  LMGetVCBQHdr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetVCBQHdr: QHdrPtr; external name '_LMGetVCBQHdr';

{
 *  LMSetVCBQHdr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetVCBQHdr(vcbQHdrValue: QHdrPtr); external name '_LMSetVCBQHdr';

{
 *  LMGetDTQueue()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetDTQueue: QHdrPtr; external name '_LMGetDTQueue';

{
 *  LMSetDTQueue()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetDTQueue(dtQueueValue: QHdrPtr); external name '_LMSetDTQueue';

{
 *  LMGetFSQHdr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetFSQHdr: QHdrPtr; external name '_LMGetFSQHdr';
>>>>>>> graemeg/fixes_2_2

{*************************************************************************************
    "BLOCKMOVE ACCESSORS"
    
        These lowmem accessors use the BlockMove to set
*************************************************************************************}
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  LMGetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApName: StringPtr; external name '_LMGetCurApName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApName( curApNameValue: ConstStr31Param ); external name '_LMSetCurApName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
function LMGetToolScratch: Ptr; external name '_LMGetToolScratch';
=======
function LMGetSysResName: StringPtr; external name '_LMGetSysResName';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMSetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
=======
 *  LMSetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetToolScratch( toolScratchValue: {const} UnivPtr ); external name '_LMSetToolScratch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{*************************************************************************************
    "INDEXED ACCESSORS"
    
        These lowmem accessors take an index parameter to get/set an indexed
        lowmem global.
*************************************************************************************}
{
 *  LMGetLvl2DT()   *** DEPRECATED ***
=======
procedure LMSetSysResName( sysResNameValue: ConstStr15Param ); external name '_LMSetSysResName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Getting the name of the Finder isn't terribly useful.  If you
 *    need to find the Finder process, look it up with the CFBundle
 *    calls by bundle id ( "com.apple.finder" ).  Remove usage of this
 *    lowmem value.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function LMGetLvl2DT( vectorNumber: SInt16 ): UniversalProcPtr; external name '_LMGetLvl2DT';
=======
function LMGetFinderName: StringPtr; external name '_LMGetFinderName';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
 *  LMSetLvl2DT()   *** DEPRECATED ***
=======
 *  LMSetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Setting the name of the Finder application is not supported on
 *    Mac OS X.  Remove usage of this lowmem accessor.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetLvl2DT( Lvl2DTValue: UniversalProcPtr; vectorNumber: SInt16 ); external name '_LMSetLvl2DT';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew


{*************************************************************************************
    "BLOCKMOVE ACCESSORS"
    
        These lowmem accessors use the BlockMove to set
*************************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  LMGetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApName: StringPtr; external name '_LMGetCurApName';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApName( curApNameValue: ConstStr31Param ); external name '_LMSetCurApName';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
=======
=======
procedure LMSetFinderName( finderNameValue: ConstStr15Param ); external name '_LMSetFinderName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetToolScratch: Ptr; external name '_LMGetToolScratch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetToolScratch( toolScratchValue: {const} UnivPtr ); external name '_LMSetToolScratch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
=======
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetFSQHdr: QHdrPtr; external name '_LMGetFSQHdr';

{*************************************************************************************
    "BLOCKMOVE ACCESSORS"
    
        These lowmem accessors use the BlockMove to set
*************************************************************************************}
>>>>>>> origin/fixes_2_2
{$endc}  {CALL_NOT_IN_CARBON}
{
 *  LMGetCurApName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LMGetCurApName: StringPtr; external name '_LMGetCurApName';

{
 *  LMSetCurApName()
=======
 }
procedure LMSetTmpResLoad( value: ByteParameter ); external name '_LMSetTmpResLoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMGetIntlSpec()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetIntlSpec: Ptr; external name '_LMGetIntlSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  LMSetIntlSpec()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetIntlSpec( value: Ptr ); external name '_LMSetIntlSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ LMGetWordRedraw and LMSetWordRedraw moved to TextEdit.h }
{$ifc not TARGET_CPU_64}
{
 *  LMGetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the id of the system font family.  Use GetSysFont() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysFontFam: SInt16; external name '_LMGetSysFontFam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{
 *  LMSetSysFontFam()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Set the id of the system font family.  This is not supported on
 *    Mac OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysFontFam( value: SInt16 ); external name '_LMSetSysFontFam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetSysFontSize()   *** DEPRECATED ***
 *  
 *  Summary:
 *    Get the size of the system font.  Use DefFontSize() in the
 *    Quickdraw framework instead of this.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysFontSize: SInt16; external name '_LMGetSysFontSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSysFontSize()
 *  
 *  Summary:
 *    Set the size for the system font.  This is not supported on Mac
 *    OS X.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysFontSize( value: SInt16 ); external name '_LMSetSysFontSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{*************************************************************************************
    "BLOCKMOVE ACCESSORS"
    
        These lowmem accessors use the BlockMove to set
*************************************************************************************}
{$ifc not TARGET_CPU_64}
{
 *  LMGetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetCurApName: StringPtr; external name '_LMGetCurApName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMSetCurApName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetCurApName( curApNameValue: ConstStr31Param ); external name '_LMSetCurApName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LMGetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysResName: StringPtr; external name '_LMGetSysResName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
procedure LMSetApplScratch(applScratchValue: UnivPtr); external name '_LMSetApplScratch';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*************************************************************************************
    "INDEXED ACCESSORS"
    
        These lowmem accessors take an index parameter to get/set an indexed
        lowmem global.
*************************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
{
 *  LMGetLvl2DT()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetLvl2DT( vectorNumber: SInt16 ): UniversalProcPtr; external name '_LMGetLvl2DT';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/cpstrnew


{ accesses "HiHeapMark"}
{
<<<<<<< HEAD
 *  LMGetHighHeapMark()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
 *  LMSetLvl2DT()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetSysResName: StringPtr; external name '_LMGetSysResName';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetSysResName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X does not have the concept of a "System" file name. 
 *    Remove usage of this lowmem value.
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetHighHeapMark: Ptr; external name '_LMGetHighHeapMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetHighHeapMark()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetSysResName( sysResNameValue: ConstStr15Param ); external name '_LMSetSysResName';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Getting the name of the Finder isn't terribly useful.  If you
 *    need to find the Finder process, look it up with the CFBundle
 *    calls by bundle id ( "com.apple.finder" ).  Remove usage of this
 *    lowmem value.
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetHighHeapMark( value: Ptr ); external name '_LMSetHighHeapMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ accesses "StkLowPt"}
{
 *  LMGetStackLowPoint()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======


{ accesses "HiHeapMark"}
{
 *  LMGetHighHeapMark()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetFinderName: StringPtr; external name '_LMGetFinderName';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Setting the name of the Finder application is not supported on
 *    Mac OS X.  Remove usage of this lowmem accessor.
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetStackLowPoint: Ptr; external name '_LMGetStackLowPoint';
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetHighHeapMark: Ptr; external name '_LMGetHighHeapMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetHighHeapMark()   *** DEPRECATED ***
=======
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetLvl2DT( Lvl2DTValue: UniversalProcPtr; vectorNumber: SInt16 ); external name '_LMSetLvl2DT';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
procedure LMSetSysResName( sysResNameValue: ConstStr15Param ); external name '_LMSetSysResName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4

{
 *  LMGetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Getting the name of the Finder isn't terribly useful.  If you
 *    need to find the Finder process, look it up with the CFBundle
 *    calls by bundle id ( "com.apple.finder" ).  Remove usage of this
 *    lowmem value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetFinderName: StringPtr; external name '_LMGetFinderName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetFinderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Setting the name of the Finder application is not supported on
 *    Mac OS X.  Remove usage of this lowmem accessor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetFinderName( finderNameValue: ConstStr15Param ); external name '_LMSetFinderName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMGetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetToolScratch: Ptr; external name '_LMGetToolScratch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetToolScratch( toolScratchValue: {const} UnivPtr ); external name '_LMSetToolScratch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{*************************************************************************************
    "INDEXED ACCESSORS"
    
        These lowmem accessors take an index parameter to get/set an indexed
        lowmem global.
*************************************************************************************}
{
 *  LMGetLvl2DT()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetLvl2DT( vectorNumber: SInt16 ): UniversalProcPtr; external name '_LMGetLvl2DT';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetLvl2DT()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetLvl2DT( Lvl2DTValue: UniversalProcPtr; vectorNumber: SInt16 ); external name '_LMSetLvl2DT';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
{$endc}  {CALL_NOT_IN_CARBON}
{$endc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

{*************************************************************************************
    "Missing Accessors"
    
        These lowmem accessors are not in the original InterfaceLib.  They were
        added to InterfaceLib in Mac OS 8.5.  In Universal Interfaces 3.2 they
        were defined via a macro. In you want to use these functions on a pre-8.5
        systems, you must write your own macros to override the function prototype
        or write your own implementation.
    
*************************************************************************************}


<<<<<<< HEAD
{ accesses "HiHeapMark"}
{
 *  LMGetHighHeapMark()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
<<<<<<< HEAD
procedure LMSetHighHeapMark( value: Ptr ); external name '_LMSetHighHeapMark';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
=======
function LMGetHighHeapMark: Ptr; external name '_LMGetHighHeapMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/cpstrnew


{ accesses "StkLowPt"}
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  LMSetStackLowPoint()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
 *  LMSetHighHeapMark()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetFinderName( finderNameValue: ConstStr15Param ); external name '_LMSetFinderName';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMGetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetToolScratch: Ptr; external name '_LMGetToolScratch';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetToolScratch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Mac OS X gives each process its own tool scratch area, so this
 *    accessor is fairly useless. If you really need to share 8 bytes
 *    of memory among various libraries, just make it a global and
 *    export it from one library and import it into the others.
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetHighHeapMark( value: Ptr ); external name '_LMSetHighHeapMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ accesses "StkLowPt"}
{
 *  LMGetStackLowPoint()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetToolScratch( toolScratchValue: {const} UnivPtr ); external name '_LMSetToolScratch';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{*************************************************************************************
    "INDEXED ACCESSORS"
    
        These lowmem accessors take an index parameter to get/set an indexed
        lowmem global.
*************************************************************************************}
{
 *  LMGetLvl2DT()   *** DEPRECATED ***
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetStackLowPoint( value: Ptr ); external name '_LMSetStackLowPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ accesses "FmtDefaults"}
{
 *  LMGetDiskFormatingHFSDefaults()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetStackLowPoint: Ptr; external name '_LMGetStackLowPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetStackLowPoint()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetLvl2DT( vectorNumber: SInt16 ): UniversalProcPtr; external name '_LMGetLvl2DT';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetLvl2DT()   *** DEPRECATED ***
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetDiskFormatingHFSDefaults: Ptr; external name '_LMGetDiskFormatingHFSDefaults';
=======
 *  LMGetStackLowPoint()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetStackLowPoint: Ptr; external name '_LMGetStackLowPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetStackLowPoint()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetStackLowPoint( value: Ptr ); external name '_LMSetStackLowPoint';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ accesses "FmtDefaults"}
{
<<<<<<< HEAD
 *  LMSetDiskFormatingHFSDefaults()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
 *  LMGetDiskFormatingHFSDefaults()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetLvl2DT( Lvl2DTValue: UniversalProcPtr; vectorNumber: SInt16 ); external name '_LMSetLvl2DT';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew

=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{*************************************************************************************
    "Missing Accessors"
    
        These lowmem accessors are not in the original InterfaceLib.  They were
        added to InterfaceLib in Mac OS 8.5.  In Universal Interfaces 3.2 they
        were defined via a macro. In you want to use these functions on a pre-8.5
        systems, you must write your own macros to override the function prototype
        or write your own implementation.
    
*************************************************************************************}


<<<<<<< HEAD
<<<<<<< HEAD
{ accesses "HiHeapMark"}
{
 *  LMGetHighHeapMark()   *** DEPRECATED ***
=======
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetStackLowPoint( value: Ptr ); external name '_LMSetStackLowPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ accesses "FmtDefaults"}
{
 *  LMGetDiskFormatingHFSDefaults()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetDiskFormatingHFSDefaults: Ptr; external name '_LMGetDiskFormatingHFSDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetDiskFormatingHFSDefaults()   *** DEPRECATED ***
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetHighHeapMark: Ptr; external name '_LMGetHighHeapMark';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetHighHeapMark()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetHighHeapMark( value: Ptr ); external name '_LMSetHighHeapMark';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{ accesses "StkLowPt"}
{
 *  LMGetStackLowPoint()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetStackLowPoint: Ptr; external name '_LMGetStackLowPoint';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetStackLowPoint()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetStackLowPoint( value: Ptr ); external name '_LMSetStackLowPoint';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{ accesses "FmtDefaults"}
{
 *  LMGetDiskFormatingHFSDefaults()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetDiskFormatingHFSDefaults: Ptr; external name '_LMGetDiskFormatingHFSDefaults';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  LMSetDiskFormatingHFSDefaults()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetDiskFormatingHFSDefaults( value: Ptr ); external name '_LMSetDiskFormatingHFSDefaults';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetDiskFormatingHFSDefaults( value: Ptr ); external name '_LMSetDiskFormatingHFSDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetDiskFormatingHFSDefaults( value: Ptr ); external name '_LMSetDiskFormatingHFSDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetDiskFormatingHFSDefaults( value: Ptr ); external name '_LMSetDiskFormatingHFSDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifc CALL_NOT_IN_CARBON}
{
 *  LMSetMouseTemp()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMouseTemp(value: Point); external name '_LMSetMouseTemp';
{  accesses "MTemp" }
{
 *  LMGetMouseTemp()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMouseTemp: Point; external name '_LMGetMouseTemp';
{
   accesses "MBState"
   Carbon Usage: use Button()
}
{
 *  LMGetMouseButtonState()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMouseButtonState: ByteParameter; external name '_LMGetMouseButtonState';
{
 *  LMSetMouseButtonState()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMouseButtonState(value: ByteParameter); external name '_LMSetMouseButtonState';
{
   accesses "RawMouse"
   Carbon Usage: use GetGlobalMouse
}
{
 *  LMGetRawMouseLocation()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetRawMouseLocation: Point; external name '_LMGetRawMouseLocation';
{  Carbon Usage: use GetGlobalMouse }
{
 *  LMSetRawMouseLocation()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetRawMouseLocation(value: Point); external name '_LMSetRawMouseLocation';
{
   accesses "Mouse"
   Carbon Usage: use GetGlobalMouse
}
{
 *  LMGetMouseLocation()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function LMGetMouseLocation: Point; external name '_LMGetMouseLocation';
{
 *  LMSetMouseLocation()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure LMSetMouseLocation(value: Point); external name '_LMSetMouseLocation';
{  accesses "TheCrsr" }
=======
{ accesses "HiHeapMark"}
>>>>>>> origin/fixes_2.4
{
 *  LMGetHighHeapMark()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetHighHeapMark: Ptr; external name '_LMGetHighHeapMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetHighHeapMark()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetHighHeapMark( value: Ptr ); external name '_LMSetHighHeapMark';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ accesses "StkLowPt"}
{
 *  LMGetStackLowPoint()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetStackLowPoint: Ptr; external name '_LMGetStackLowPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetStackLowPoint()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetStackLowPoint( value: Ptr ); external name '_LMSetStackLowPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ accesses "FmtDefaults"}
{
 *  LMGetDiskFormatingHFSDefaults()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function LMGetDiskFormatingHFSDefaults: Ptr; external name '_LMGetDiskFormatingHFSDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetDiskFormatingHFSDefaults()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
procedure LMSetDiskFormatingHFSDefaults( value: Ptr ); external name '_LMSetDiskFormatingHFSDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

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
