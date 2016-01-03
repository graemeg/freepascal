{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       Print/PMApplication.h
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Copyright  (c) 1998-2008 Apple Inc. All Rights Reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       PMApplication.p
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Version:    Technology: Mac OS X
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  © 1998-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     File:       Print/PMApplication.h
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Copyright  (c) 1998-2008 Apple Inc. All Rights Reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit PMApplication;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
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
uses MacTypes,Dialogs,QuickdrawTypes,PMDefinitions,CFBase;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew

{$ALIGN POWER}

<<<<<<< HEAD
=======

{$ALIGN POWER}

>>>>>>> graemeg/cpstrnew
{$ifc not TARGET_CPU_64}
=======
{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew

{$ALIGN POWER}

<<<<<<< HEAD
=======

{$ALIGN POWER}

{$ifc not TARGET_CPU_64}

{ Callbacks }

>>>>>>> graemeg/cpstrnew
=======
{$ifc not TARGET_CPU_64}

{ Callbacks }

>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{$ifc not TARGET_CPU_64}

{ Callbacks }

>>>>>>> origin/cpstrnew
type
	PMSheetDoneProcPtr = procedure( printSession: PMPrintSession; documentWindow: WindowRef; accepted: Boolean );
	PMSheetDoneUPP = PMSheetDoneProcPtr;

{
 *  NewPMSheetDoneUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewPMSheetDoneUPP( userRoutine: PMSheetDoneProcPtr ): PMSheetDoneUPP; external name '_NewPMSheetDoneUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposePMSheetDoneUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposePMSheetDoneUPP( userUPP: PMSheetDoneUPP ); external name '_DisposePMSheetDoneUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokePMSheetDoneUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokePMSheetDoneUPP( printSession: PMPrintSession; documentWindow: WindowRef; accepted: Boolean; userUPP: PMSheetDoneUPP ); external name '_InvokePMSheetDoneUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc PM_USE_SESSION_APIS}
{ Print loop }
{
 *  PMSessionBeginCGDocument()
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Summary:
 *    Begin a new print job that uses only drawing to a CoreGraphics
 *    context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocument. The functionality is identical to
 *    PMSessionBeginDocument except that during a print job, the caller
 *    cannot obtain a Quickdraw grafPort for the printing context but
 *    can only obtain a Quartz graphics context (CGContextRef). This
 *    function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginCGDocument( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  PMSessionEndDocument()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndDocument( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionBeginPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginPage( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *  
 *  Summary:
 *    Begin a new print job that uses only drawing to a CoreGraphics
 *    context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocument. The functionality is identical to
 *    PMSessionBeginDocument except that during a print job, the caller
 *    cannot obtain a Quickdraw grafPort for the printing context but
 *    can only obtain a Quartz graphics context (CGContextRef). This
 *    function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginCGDocument( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
 *  PMSessionEndDocument()
=======
 *  
 *  Summary:
 *    Begin a new print job that uses only drawing to a CoreGraphics
 *    context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocument. The functionality is identical to
 *    PMSessionBeginDocument except that during a print job, the caller
 *    cannot obtain a Quickdraw grafPort for the printing context but
 *    can only obtain a Quartz graphics context (CGContextRef). This
 *    function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginCGDocument( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Quickdraw,PMDefinitions,PMCore,Dialogs;
=======
uses MacTypes,Dialogs,QuickdrawTypes,PMDefinitions,CFBase;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{$ifc not TARGET_CPU_64}

{ Callbacks }

type
	PMSheetDoneProcPtr = procedure( printSession: PMPrintSession; documentWindow: WindowRef; accepted: Boolean );
	PMSheetDoneUPP = PMSheetDoneProcPtr;

{
 *  NewPMSheetDoneUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function NewPMSheetDoneUPP( userRoutine: PMSheetDoneProcPtr ): PMSheetDoneUPP; external name '_NewPMSheetDoneUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposePMSheetDoneUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposePMSheetDoneUPP( userUPP: PMSheetDoneUPP ); external name '_DisposePMSheetDoneUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokePMSheetDoneUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokePMSheetDoneUPP( printSession: PMPrintSession; documentWindow: WindowRef; accepted: Boolean; userUPP: PMSheetDoneUPP ); external name '_InvokePMSheetDoneUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc PM_USE_SESSION_APIS}
{ Print loop }
{
 *  PMSessionBeginCGDocument()
 *  
<<<<<<< HEAD
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionBeginDocument(printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat): OSStatus; external name '_PMSessionBeginDocument';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  PMSessionEndDocument()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndDocument( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionBeginPage()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 }
<<<<<<< HEAD
function PMSessionEndPage( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Dialogs
#pragma mark
*)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionEndDocument(printSession: PMPrintSession): OSStatus; external name '_PMSessionEndDocument';

{
 *  PMSessionBeginPage()
=======
 *  Summary:
 *    Begin a new print job that uses only drawing to a CoreGraphics
 *    context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocument. The functionality is identical to
 *    PMSessionBeginDocument except that during a print job, the caller
 *    cannot obtain a Quickdraw grafPort for the printing context but
 *    can only obtain a Quartz graphics context (CGContextRef). This
 *    function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginCGDocument( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  PMSessionEndDocument()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionEndPage(printSession: PMPrintSession): OSStatus; external name '_PMSessionEndPage';

{ Session Printing Dialogs }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
{
 *  PMSessionPageSetupDialog()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPageSetupDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMSessionPageSetupDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionPrintDialog()
=======
function PMSessionEndDocument( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionBeginPage()
>>>>>>> graemeg/cpstrnew
=======
 }
function PMSessionBeginPage( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionEndPage()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
function PMSessionPrintDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMSessionPrintDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionUseSheets()
 *  
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionUseSheets( printSession: PMPrintSession; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMSessionUseSheets';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMShowPageSetupDialogAsSheet()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPageSetupDialogAsSheet( printSession: PMPrintSession; pageFormat: PMPageFormat; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMShowPageSetupDialogAsSheet';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMShowPrintDialogWithOptions()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPrintDialogWithOptions( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; printDialogOptions: PMPrintDialogOptionFlags; var accepted: Boolean ): OSStatus; external name '_PMShowPrintDialogWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMShowPrintDialogWithOptionsAsSheet()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPrintDialogWithOptionsAsSheet( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; printDialogOptions: PMPrintDialogOptionFlags; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMShowPrintDialogWithOptionsAsSheet';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark Presets
#pragma mark
*)

{ Presets }
{
 *  PMSessionEnablePrinterPresets()
 *  
 *  Summary:
 *    Enable the use of printer presets in the print dialog.
<<<<<<< HEAD
 *  
 *  Discussion:
 *    Displaying the print dialog on a session after making this call
 *    will show the presets available for the specified graphics type.
 *  
 *  Parameters:
 *    
 *    session:
 *      The session that will be used to present the print dialog.
 *    
 *    graphicsType:
 *      The printer presets in the dialog should be suitable for
 *      rendering this type of graphic. Currently defined graphics
 *      types are: "Photo"
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEnablePrinterPresets( session: PMPrintSession; graphicsType: CFStringRef ): OSStatus; external name '_PMSessionEnablePrinterPresets';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{
 *  PMSessionDisablePrinterPresets()
 *  
 *  Summary:
 *    Disable the use of printer presets in the print dialog.
 *  
 *  Parameters:
 *    
 *    session:
 *      The session that will be used to present the print dialog.
 *  
=======
=======
function PMSessionBeginPage( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionEndPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndPage( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Dialogs
#pragma mark
*)

{
 *  PMSessionPageSetupDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPageSetupDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMSessionPageSetupDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionPrintDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPrintDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMSessionPrintDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionUseSheets()
 *  
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionUseSheets( printSession: PMPrintSession; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMSessionUseSheets';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMShowPageSetupDialogAsSheet()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPageSetupDialogAsSheet( printSession: PMPrintSession; pageFormat: PMPageFormat; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMShowPageSetupDialogAsSheet';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMShowPrintDialogWithOptions()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPrintDialogWithOptions( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; printDialogOptions: PMPrintDialogOptionFlags; var accepted: Boolean ): OSStatus; external name '_PMShowPrintDialogWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMShowPrintDialogWithOptionsAsSheet()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPrintDialogWithOptionsAsSheet( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; printDialogOptions: PMPrintDialogOptionFlags; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMShowPrintDialogWithOptionsAsSheet';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark Presets
#pragma mark
*)

{ Presets }
{
 *  PMSessionEnablePrinterPresets()
 *  
 *  Summary:
 *    Enable the use of printer presets in the print dialog.
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    Displaying the print dialog on a session after making this call
 *    will show the presets available for the specified graphics type.
 *  
=======
function PMSessionEndPage( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Dialogs
#pragma mark
*)

=======
>>>>>>> origin/fixes_2_2
{
 *  PMSessionPageSetupDialog()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPageSetupDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMSessionPageSetupDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionPrintDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPrintDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMSessionPrintDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionUseSheets()
 *  
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionUseSheets( printSession: PMPrintSession; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMSessionUseSheets';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMShowPageSetupDialogAsSheet()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPageSetupDialogAsSheet( printSession: PMPrintSession; pageFormat: PMPageFormat; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMShowPageSetupDialogAsSheet';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMShowPrintDialogWithOptions()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPrintDialogWithOptions( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; printDialogOptions: PMPrintDialogOptionFlags; var accepted: Boolean ): OSStatus; external name '_PMShowPrintDialogWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMShowPrintDialogWithOptionsAsSheet()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPrintDialogWithOptionsAsSheet( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; printDialogOptions: PMPrintDialogOptionFlags; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMShowPrintDialogWithOptionsAsSheet';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark Presets
#pragma mark
*)

{ Presets }
{
 *  PMSessionEnablePrinterPresets()
 *  
 *  Summary:
 *    Enable the use of printer presets in the print dialog.
 *  
 *  Discussion:
 *    Displaying the print dialog on a session after making this call
 *    will show the presets available for the specified graphics type.
 *  
 *  Parameters:
 *    
 *    session:
 *      The session that will be used to present the print dialog.
 *    
 *    graphicsType:
 *      The printer presets in the dialog should be suitable for
 *      rendering this type of graphic. Currently defined graphics
 *      types are: "Photo"
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEnablePrinterPresets( session: PMPrintSession; graphicsType: CFStringRef ): OSStatus; external name '_PMSessionEnablePrinterPresets';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{
 *  PMSessionDisablePrinterPresets()
 *  
 *  Summary:
 *    Disable the use of printer presets in the print dialog.
 *  
>>>>>>> origin/cpstrnew
 *  Parameters:
 *    
 *    session:
 *      The session that will be used to present the print dialog.
<<<<<<< HEAD
 *    
 *    graphicsType:
 *      The printer presets in the dialog should be suitable for
 *      rendering this type of graphic. Currently defined graphics
 *      types are: "Photo"
=======
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMSessionEnablePrinterPresets( session: PMPrintSession; graphicsType: CFStringRef ): OSStatus; external name '_PMSessionEnablePrinterPresets';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{
 *  PMSessionDisablePrinterPresets()
 *  
 *  Summary:
 *    Disable the use of printer presets in the print dialog.
 *  
 *  Parameters:
 *    
 *    session:
 *      The session that will be used to present the print dialog.
 *  
>>>>>>> graemeg/cpstrnew
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDisablePrinterPresets( session: PMPrintSession ): OSStatus; external name '_PMSessionDisablePrinterPresets';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

=======
function PMSessionDisablePrinterPresets( session: PMPrintSession ): OSStatus; external name '_PMSessionDisablePrinterPresets';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> origin/cpstrnew
{$endc} {PM_USE_SESSION_APIS}

{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndDocument( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionBeginPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginPage( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionEndPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndPage( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Dialogs
#pragma mark
*)

{
 *  PMSessionPageSetupDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPageSetupDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMSessionPageSetupDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionPrintDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPrintDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMSessionPrintDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionUseSheets()
 *  
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        in CarbonLib 1.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionUseSheets( printSession: PMPrintSession; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMSessionUseSheets';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMShowPageSetupDialogAsSheet()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPageSetupDialogAsSheet( printSession: PMPrintSession; pageFormat: PMPageFormat; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMShowPageSetupDialogAsSheet';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMShowPrintDialogWithOptions()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPrintDialogWithOptions( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; printDialogOptions: PMPrintDialogOptionFlags; var accepted: Boolean ): OSStatus; external name '_PMShowPrintDialogWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMShowPrintDialogWithOptionsAsSheet()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMShowPrintDialogWithOptionsAsSheet( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; printDialogOptions: PMPrintDialogOptionFlags; documentWindow: WindowRef; sheetDoneProc: PMSheetDoneUPP ): OSStatus; external name '_PMShowPrintDialogWithOptionsAsSheet';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark Presets
#pragma mark
*)

{ Presets }
{
 *  PMSessionEnablePrinterPresets()
 *  
 *  Summary:
 *    Enable the use of printer presets in the print dialog.
 *  
 *  Discussion:
 *    Displaying the print dialog on a session after making this call
 *    will show the presets available for the specified graphics type.
 *  
 *  Parameters:
 *    
 *    session:
 *      The session that will be used to present the print dialog.
 *    
 *    graphicsType:
 *      The printer presets in the dialog should be suitable for
 *      rendering this type of graphic. Currently defined graphics
 *      types are: "Photo"
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEnablePrinterPresets( session: PMPrintSession; graphicsType: CFStringRef ): OSStatus; external name '_PMSessionEnablePrinterPresets';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{
 *  PMSessionDisablePrinterPresets()
 *  
 *  Summary:
 *    Disable the use of printer presets in the print dialog.
 *  
 *  Parameters:
 *    
 *    session:
 *      The session that will be used to present the print dialog.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework  [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDisablePrinterPresets( session: PMPrintSession ): OSStatus; external name '_PMSessionDisablePrinterPresets';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

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
