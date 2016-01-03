{
     File:       Print/PMApplicationDeprecated.h
 
     Contains:   Deprecated Carbon Printing Manager Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	 Copyright  (c) 1998-2006, 2008, 2011 Apple Inc. All Rights Reserved.
=======
	 Copyright  (c) 1998-2006, 2008 Apple Inc. All Rights Reserved.
>>>>>>> graemeg/cpstrnew
=======
	 Copyright  (c) 1998-2006, 2008 Apple Inc. All Rights Reserved.
>>>>>>> graemeg/cpstrnew
=======
	 Copyright  (c) 1998-2006, 2008 Apple Inc. All Rights Reserved.
>>>>>>> graemeg/cpstrnew
=======
	 Copyright  (c) 1998-2006, 2008 Apple Inc. All Rights Reserved.
>>>>>>> origin/cpstrnew
=======
	 Copyright  (c) 1998-2006, 2008 Apple Inc. All Rights Reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit PMApplicationDeprecated;
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
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes,Dialogs,PMDefinitions,PMDefinitionsDeprecated;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{$ifc not TARGET_CPU_64}

{****************************************}
{ The following callbacks are deprecated }
{****************************************}
type
	PMItemProcPtr = procedure( theDialog: DialogRef; item: SInt16 );
	PMPrintDialogInitProcPtr = procedure( printSettings: PMPrintSettings; var theDialog: PMDialog );
	PMPageSetupDialogInitProcPtr = procedure( pageFormat: PMPageFormat; var theDialog: PMDialog );
	PMItemUPP = PMItemProcPtr;
	PMPrintDialogInitUPP = PMPrintDialogInitProcPtr;
	PMPageSetupDialogInitUPP = PMPageSetupDialogInitProcPtr;

{
 *  NewPMItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewPMItemUPP( userRoutine: PMItemProcPtr ): PMItemUPP; external name '_NewPMItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewPMPrintDialogInitUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewPMPrintDialogInitUPP( userRoutine: PMPrintDialogInitProcPtr ): PMPrintDialogInitUPP; external name '_NewPMPrintDialogInitUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewPMPageSetupDialogInitUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewPMPageSetupDialogInitUPP( userRoutine: PMPageSetupDialogInitProcPtr ): PMPageSetupDialogInitUPP; external name '_NewPMPageSetupDialogInitUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposePMItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposePMItemUPP( userUPP: PMItemUPP ); external name '_DisposePMItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposePMPrintDialogInitUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposePMPrintDialogInitUPP( userUPP: PMPrintDialogInitUPP ); external name '_DisposePMPrintDialogInitUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposePMPageSetupDialogInitUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposePMPageSetupDialogInitUPP( userUPP: PMPageSetupDialogInitUPP ); external name '_DisposePMPageSetupDialogInitUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokePMItemUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokePMItemUPP( theDialog: DialogRef; item: SInt16; userUPP: PMItemUPP ); external name '_InvokePMItemUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokePMPrintDialogInitUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokePMPrintDialogInitUPP( printSettings: PMPrintSettings; var theDialog: PMDialog; userUPP: PMPrintDialogInitUPP ); external name '_InvokePMPrintDialogInitUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokePMPageSetupDialogInitUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokePMPageSetupDialogInitUPP( pageFormat: PMPageFormat; var theDialog: PMDialog; userUPP: PMPageSetupDialogInitUPP ); external name '_InvokePMPageSetupDialogInitUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$ifc PM_USE_SESSION_APIS}

{
 *  PMSessionBeginDocument()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Instead use PMSessionBeginCGDocument.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginDocument( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMSessionPageSetupDialogInit()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPageSetupDialogInit( printSession: PMPrintSession; pageFormat: PMPageFormat; var newDialog: PMDialog ): OSStatus; external name '_PMSessionPageSetupDialogInit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionPrintDialogInit()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPrintDialogInit( printSession: PMPrintSession; printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var newDialog: PMDialog ): OSStatus; external name '_PMSessionPrintDialogInit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionPrintDialogMain()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPrintDialogMain( printSession: PMPrintSession; printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var accepted: Boolean; myInitProc: PMPrintDialogInitUPP ): OSStatus; external name '_PMSessionPrintDialogMain';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionPageSetupDialogMain()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPageSetupDialogMain( printSession: PMPrintSession; pageFormat: PMPageFormat; var accepted: Boolean; myInitProc: PMPageSetupDialogInitUPP ): OSStatus; external name '_PMSessionPageSetupDialogMain';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$elsec} {PM_USE_SESSION_APIS}

{
 *  PMBeginDocument()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionBeginDocument instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMBeginDocument( printSettings: PMPrintSettings; pageFormat: PMPageFormat; var printContext: PMPrintContext ): OSStatus; external name '_PMBeginDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMEndDocument()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionEndDocument instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMEndDocument( printContext: PMPrintContext ): OSStatus; external name '_PMEndDocument';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMBeginPage()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionBeginPage instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMBeginPage( printContext: PMPrintContext; const (*var*) pageFrame: PMRect ): OSStatus; external name '_PMBeginPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMEndPage()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionEndPage instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMEndPage( printContext: PMPrintContext ): OSStatus; external name '_PMEndPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPageSetupDialog()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionPageSetupDialog instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPageSetupDialog( pageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMPageSetupDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPrintDialog()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionPrintDialog instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintDialog( printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var accepted: Boolean ): OSStatus; external name '_PMPrintDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPageSetupDialogInit()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPageSetupDialogInit( pageFormat: PMPageFormat; var newDialog: PMDialog ): OSStatus; external name '_PMPageSetupDialogInit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPrintDialogInit()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintDialogInit( printSettings: PMPrintSettings; var newDialog: PMDialog ): OSStatus; external name '_PMPrintDialogInit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPrintDialogInitWithPageFormat()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintDialogInitWithPageFormat( printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var newDialog: PMDialog ): OSStatus; external name '_PMPrintDialogInitWithPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPrintDialogMain()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintDialogMain( printSettings: PMPrintSettings; constPageFormat: PMPageFormat; var accepted: Boolean; myInitProc: PMPrintDialogInitUPP ): OSStatus; external name '_PMPrintDialogMain';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPageSetupDialogMain()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPageSetupDialogMain( pageFormat: PMPageFormat; var accepted: Boolean; myInitProc: PMPageSetupDialogInitUPP ): OSStatus; external name '_PMPageSetupDialogMain';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {PM_USE_SESSION_APIS}

{
 *  PMGetDialogPtr()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDialogPtr( pmDialog_: PMDialog; var theDialog: DialogRef ): OSStatus; external name '_PMGetDialogPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

function PMGetDialogRef( pmDialog_: PMDialog; var theDialog: DialogRef ): OSStatus; external name '_PMGetDialogPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
{
 *  PMGetModalFilterProc()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetModalFilterProc( pmDialog_: PMDialog; var filterProc: ModalFilterUPP ): OSStatus; external name '_PMGetModalFilterProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSetModalFilterProc()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetModalFilterProc( pmDialog_: PMDialog; filterProc: ModalFilterUPP ): OSStatus; external name '_PMSetModalFilterProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetItemProc()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetItemProc( pmDialog_: PMDialog; var itemProc: PMItemUPP ): OSStatus; external name '_PMGetItemProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSetItemProc()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetItemProc( pmDialog_: PMDialog; itemProc: PMItemUPP ): OSStatus; external name '_PMSetItemProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetDialogAccepted()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDialogAccepted( pmDialog_: PMDialog; var process: Boolean ): OSStatus; external name '_PMGetDialogAccepted';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSetDialogAccepted()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetDialogAccepted( pmDialog_: PMDialog; process: Boolean ): OSStatus; external name '_PMSetDialogAccepted';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetDialogDone()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDialogDone( pmDialog_: PMDialog; var done: Boolean ): OSStatus; external name '_PMGetDialogDone';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSetDialogDone()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    You should create a PDE for your application instead of relying
 *    on this function.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetDialogDone( pmDialog_: PMDialog; done: Boolean ): OSStatus; external name '_PMSetDialogDone';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
