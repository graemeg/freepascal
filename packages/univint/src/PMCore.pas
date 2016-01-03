{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       PrintCore/PMCore.h
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Copyright (c) 1998-2006,2008 Apple Inc. All Rights Reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       PMCore.p
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Version:    Technology: Mac OS X
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  © 1998-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     File:       PrintCore/PMCore.h
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Copyright (c) 1998-2006,2008 Apple Inc. All Rights Reserved.
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

unit PMCore;
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
=======
>>>>>>> origin/fixes_2.4
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
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
uses MacTypes,CFBase,CFArray,MacErrors,Files,QuickdrawTypes,PMDefinitions,CFData,CFDictionary,CFString,CFURL,CGContext,CGDataProvider,CGImage;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD



=======



>>>>>>> graemeg/cpstrnew
{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

=======



{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

>>>>>>> origin/cpstrnew
{$ifc PM_USE_SESSION_APIS}
(*
#pragma mark
#pragma mark Retain/Release
#pragma mark
*)

{
 *  PMRetain()
<<<<<<< HEAD
=======



{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{$ifc PM_USE_SESSION_APIS}
(*
#pragma mark
#pragma mark Retain/Release
#pragma mark
*)

{
 *  PMRetain()
=======
 *  
 *  Summary:
 *    Increases a printing objects refcount by 1.
 *  
 *  Discussion:
 *    You should retain a printing object when you receive it from
 *    elsewhere (that is, you did not create or copy it) and you want
 *    it to persist. If you retain a printing object you are
 *    responsible for releasing it.
 *  
 *  Parameters:
 *    
 *    object:
 *      A Carbon Printing Manager object, such as a PMPrintSession
 *      object, a PMPageFormat object, a PMPrintSettings object, or
 *      PMPrinter object.
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Increases a printing objects refcount by 1.
 *  
 *  Discussion:
 *    You should retain a printing object when you receive it from
 *    elsewhere (that is, you did not create or copy it) and you want
 *    it to persist. If you retain a printing object you are
 *    responsible for releasing it.
 *  
 *  Parameters:
 *    
 *    object:
 *      A Carbon Printing Manager object, such as a PMPrintSession
 *      object, a PMPageFormat object, a PMPrintSettings object, or
 *      PMPrinter object.
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Increases a printing objects refcount by 1.
 *  
 *  Discussion:
 *    You should retain a printing object when you receive it from
 *    elsewhere (that is, you did not create or copy it) and you want
 *    it to persist. If you retain a printing object you are
 *    responsible for releasing it.
 *  
 *  Parameters:
 *    
 *    object:
 *      A Carbon Printing Manager object, such as a PMPrintSession
 *      object, a PMPageFormat object, a PMPrintSettings object, or
 *      PMPrinter object.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 }
function PMRetain( objct: PMObject ): OSStatus; external name '_PMRetain';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,CFBase,CFArray,MacErrors,Files,CFString,CFURL,Quickdraw,CMApplication,PMDefinitions;
=======
uses MacTypes,CFBase,CFArray,MacErrors,Files,QuickdrawTypes,PMDefinitions,CFData,CFDictionary,CFString,CFURL,CGContext,CGDataProvider,CGImage;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4



{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{$ifc PM_USE_SESSION_APIS}
(*
#pragma mark
#pragma mark Retain/Release
#pragma mark
*)

{
 *  PMRetain()
 *  
 *  Summary:
 *    Increases a printing objects refcount by 1.
 *  
 *  Discussion:
 *    You should retain a printing object when you receive it from
 *    elsewhere (that is, you did not create or copy it) and you want
 *    it to persist. If you retain a printing object you are
 *    responsible for releasing it.
 *  
 *  Parameters:
 *    
 *    object:
 *      A Carbon Printing Manager object, such as a PMPrintSession
 *      object, a PMPageFormat object, a PMPrintSettings object, or
 *      PMPrinter object.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMRetain(objct: PMObject): OSStatus; external name '_PMRetain';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PMRetain( objct: PMObject ): OSStatus; external name '_PMRetain';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  PMRelease()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Summary:
 *    Decreases a printing objects refcount by 1.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Discussion:
 *    Your application should use the PMRelease function to release any
 *    printing objects it creates or retains. When an object’s
 *    reference count reaches 0, the object is deallocated. To
 *    terminate a printing session created with the function
 *    PMCreateSession, pass the associated PMPrintSession object to
 *    PMRelease. To release printing objects created with the functions
 *    PMCreatePageFormat and PMCreatePrintSettings, pass the associated
 *    PMPageFormat and PMPrintSettings objects to PMRelease. You can
 *    pass a NULL object but be aware a kPMInvalidParameter error will
 *    be returned.
 *  
 *  Parameters:
 *    
 *    object:
 *      A Carbon Printing Manager object, such as a PMPrintSession
 *      object, a PMPageFormat object, a PMPrintSettings object, or
 *      PMPrinter object.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMRelease( objct: PMObject ): OSStatus; external name '_PMRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Session
#pragma mark
*)

{
 *  PMCreateSession()
 *  
 *  Summary:
 *    Creates and initializes a printing session object and creates a
 *    context for printing operations.
 *  
 *  Discussion:
 *    A session is created with a refcount of 1.
=======
 *  
=======
 *  
>>>>>>> origin/cpstrnew
 *  Discussion:
 *    Your application should use the PMRelease function to release any
 *    printing objects it creates or retains. When an object’s
 *    reference count reaches 0, the object is deallocated. To
 *    terminate a printing session created with the function
 *    PMCreateSession, pass the associated PMPrintSession object to
 *    PMRelease. To release printing objects created with the functions
 *    PMCreatePageFormat and PMCreatePrintSettings, pass the associated
 *    PMPageFormat and PMPrintSettings objects to PMRelease. You can
 *    pass a NULL object but be aware a kPMInvalidParameter error will
 *    be returned.
 *  
<<<<<<< HEAD
=======
 *  
=======
 *  
>>>>>>> graemeg/cpstrnew
=======
 *  Summary:
 *    Decreases a printing objects refcount by 1.
 *  
>>>>>>> origin/fixes_2.4
 *  Discussion:
 *    Your application should use the PMRelease function to release any
 *    printing objects it creates or retains. When an object’s
 *    reference count reaches 0, the object is deallocated. To
 *    terminate a printing session created with the function
 *    PMCreateSession, pass the associated PMPrintSession object to
 *    PMRelease. To release printing objects created with the functions
 *    PMCreatePageFormat and PMCreatePrintSettings, pass the associated
 *    PMPageFormat and PMPrintSettings objects to PMRelease. You can
 *    pass a NULL object but be aware a kPMInvalidParameter error will
 *    be returned.
 *  
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 *  Parameters:
 *    
 *    object:
 *      A Carbon Printing Manager object, such as a PMPrintSession
 *      object, a PMPageFormat object, a PMPrintSettings object, or
 *      PMPrinter object.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMCreateSession( var printSession: PMPrintSession ): OSStatus; external name '_PMCreateSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionError()
=======
 }
function PMRelease( objct: PMObject ): OSStatus; external name '_PMRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Session
#pragma mark
*)

{
 *  PMCreateSession()
 *  
 *  Summary:
 *    Creates and initializes a printing session object and creates a
 *    context for printing operations.
 *  
 *  Discussion:
 *    A session is created with a refcount of 1.
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionError( printSession: PMPrintSession ): OSStatus; external name '_PMSessionError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 }
function PMCreateSession( var printSession: PMPrintSession ): OSStatus; external name '_PMCreateSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionError()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionError( printSession: PMPrintSession ): OSStatus; external name '_PMSessionError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 }
function PMRelease( objct: PMObject ): OSStatus; external name '_PMRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 }
function PMRelease( objct: PMObject ): OSStatus; external name '_PMRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
(*
#pragma mark
#pragma mark Session
#pragma mark
*)
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


>>>>>>> graemeg/cpstrnew
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMSessionSetError()
=======
 }
function PMRelease( objct: PMObject ): OSStatus; external name '_PMRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 }
function PMRelease( objct: PMObject ): OSStatus; external name '_PMRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
(*
#pragma mark
#pragma mark Session
#pragma mark
*)

{
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  PMCreateSession()
 *  
 *  Summary:
 *    Creates and initializes a printing session object and creates a
 *    context for printing operations.
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Discussion:
 *    A session is created with a refcount of 1.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionSetError( printSession: PMPrintSession; printError: OSStatus ): OSStatus; external name '_PMSessionSetError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Session: Printing Loop
#pragma mark
*)

{
 *  PMSessionBeginCGDocumentNoDialog()
 *  
 *  Summary:
 *    Begin a new print job for client drawing to a CoreGraphics context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocumentNoDialog. The functionality is identical to
 *    PMSessionBeginDocumentNoDialog except that during a print job,
 *    the caller cannot obtain a Quickdraw grafPort for the printing
 *    context but can only obtain a Quartz graphics context
 *    (CGContextRef). This function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
 *  
=======
 }
function PMSessionSetError( printSession: PMPrintSession; printError: OSStatus ): OSStatus; external name '_PMSessionSetError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Session: Printing Loop
#pragma mark
*)

{
 *  PMSessionBeginCGDocumentNoDialog()
 *  
 *  Summary:
 *    Begin a new print job for client drawing to a CoreGraphics context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocumentNoDialog. The functionality is identical to
 *    PMSessionBeginDocumentNoDialog except that during a print job,
 *    the caller cannot obtain a Quickdraw grafPort for the printing
 *    context but can only obtain a Quartz graphics context
 *    (CGContextRef). This function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
=======
 *  PMCreateSession()
 *  
 *  Summary:
 *    Creates and initializes a printing session object and creates a
 *    context for printing operations.
 *  
 *  Discussion:
 *    A session is created with a refcount of 1.
>>>>>>> origin/fixes_2.4
 *  
>>>>>>> graemeg/cpstrnew
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionBeginCGDocumentNoDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSessionEndDocumentNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionEndDocumentNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 }
function PMSessionBeginCGDocumentNoDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSessionEndDocumentNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndDocumentNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew

{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }
{
 *  PMSessionBeginPageNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionBeginPageNoDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }
{
 *  PMSessionEndPageNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionEndPageNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionGetCGGraphicsContext()
 *  
 *  Summary:
 *    Return the CGContextRef for the current page in the printing
 *    session.
 *  
 *  Discussion:
 *    This function returns the CGContextRef for the printing session.
 *    This function must be called for each page. To use
 *    PMSessionGetCGGraphicsContext you MUST call
 *    PMSessionBeginCGDocument or PMSessionBeginCGDocumentNoDialog
 *    instead of PMSessionBeginDocument or
 *    PMSessionBeginDocumentNoDialog.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session for the print job.
 *    
 *    context:
 *      A pointer to a caller supplied CGContextRef variable. If this
 *      function succeeds then *context will be filled in with the
 *      printing CGContextRef.
 *  
=======
 }
function PMCreateSession( var printSession: PMPrintSession ): OSStatus; external name '_PMCreateSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionError()
 *  
=======
 *  
 *  Discussion:
 *    A session is created with a refcount of 1.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreateSession( var printSession: PMPrintSession ): OSStatus; external name '_PMCreateSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionError()
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreateSession( var printSession: PMPrintSession ): OSStatus; external name '_PMCreateSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionError()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionError( printSession: PMPrintSession ): OSStatus; external name '_PMSessionError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionSetError()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 }
function PMSessionError( printSession: PMPrintSession ): OSStatus; external name '_PMSessionError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionSetError()
 *  
>>>>>>> graemeg/cpstrnew
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMSessionError( printSession: PMPrintSession ): OSStatus; external name '_PMSessionError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionSetError()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetError( printSession: PMPrintSession; printError: OSStatus ): OSStatus; external name '_PMSessionSetError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
function PMSessionSetError( printSession: PMPrintSession; printError: OSStatus ): OSStatus; external name '_PMSessionSetError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
 }
function PMSessionSetError( printSession: PMPrintSession; printError: OSStatus ): OSStatus; external name '_PMSessionSetError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
(*
#pragma mark
#pragma mark Session: Printing Loop
#pragma mark
*)

{
 *  PMSessionBeginCGDocumentNoDialog()
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Summary:
 *    Begin a new print job for client drawing to a CoreGraphics context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocumentNoDialog. The functionality is identical to
 *    PMSessionBeginDocumentNoDialog except that during a print job,
 *    the caller cannot obtain a Quickdraw grafPort for the printing
 *    context but can only obtain a Quartz graphics context
 *    (CGContextRef). This function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
=======
 *  
 *  Discussion:
 *    A session is created with a refcount of 1.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreateSession( var printSession: PMPrintSession ): OSStatus; external name '_PMCreateSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionError()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionError( printSession: PMPrintSession ): OSStatus; external name '_PMSessionError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionSetError()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetError( printSession: PMPrintSession; printError: OSStatus ): OSStatus; external name '_PMSessionSetError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark
#pragma mark Session: Printing Loop
#pragma mark
*)

{
 *  PMSessionBeginCGDocumentNoDialog()
 *  
 *  Summary:
 *    Begin a new print job for client drawing to a CoreGraphics context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocumentNoDialog. The functionality is identical to
 *    PMSessionBeginDocumentNoDialog except that during a print job,
 *    the caller cannot obtain a Quickdraw grafPort for the printing
 *    context but can only obtain a Quartz graphics context
 *    (CGContextRef). This function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginCGDocumentNoDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

=======
 *  
 *  Summary:
 *    Begin a new print job for client drawing to a CoreGraphics context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocumentNoDialog. The functionality is identical to
 *    PMSessionBeginDocumentNoDialog except that during a print job,
 *    the caller cannot obtain a Quickdraw grafPort for the printing
 *    context but can only obtain a Quartz graphics context
 *    (CGContextRef). This function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginCGDocumentNoDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  PMSessionEndDocumentNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndDocumentNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD


{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }
{
 *  PMSessionBeginPageNoDialog()
>>>>>>> origin/cpstrnew
 *  
>>>>>>> graemeg/cpstrnew
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionGetCGGraphicsContext( printSession: PMPrintSession; var context: CGContextRef ): OSStatus; external name '_PMSessionGetCGGraphicsContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

(*
#pragma mark
#pragma mark Session: Other routines
#pragma mark -
*)

=======
 }
function PMSessionBeginPageNoDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======

>>>>>>> origin/fixes_2.4

{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
<<<<<<< HEAD
 }
{
 *  PMSessionEndPageNoDialog()
=======
 }
function PMSessionBeginCGDocumentNoDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSessionEndDocumentNoDialog()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMSessionEndPageNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *  
 *  Summary:
 *    Begin a new print job for client drawing to a CoreGraphics context.
 *  
 *  Discussion:
 *    This is an updated version of the function
 *    PMSessionBeginDocumentNoDialog. The functionality is identical to
 *    PMSessionBeginDocumentNoDialog except that during a print job,
 *    the caller cannot obtain a Quickdraw grafPort for the printing
 *    context but can only obtain a Quartz graphics context
 *    (CGContextRef). This function should be used in conjunction with
 *    PMSessionGetCGGraphicsContext instead of
 *    PMSessionGetGraphicsContext.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginCGDocumentNoDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginCGDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSessionEndDocumentNoDialog()
=======
 }
{
 *  PMSessionBeginPageNoDialog()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMSessionEndDocumentNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }
{
 *  PMSessionBeginPageNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginPageNoDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
function PMSessionBeginPageNoDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
<<<<<<< HEAD
 }
{
 *  PMSessionEndPageNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndPageNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/cpstrnew
{
 *  PMSessionGetCGGraphicsContext()
 *  
 *  Summary:
 *    Return the CGContextRef for the current page in the printing
 *    session.
=======
 }
{
 *  PMSessionEndPageNoDialog()
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    This function returns the CGContextRef for the printing session.
 *    This function must be called for each page. To use
 *    PMSessionGetCGGraphicsContext you MUST call
 *    PMSessionBeginCGDocument or PMSessionBeginCGDocumentNoDialog
 *    instead of PMSessionBeginDocument or
 *    PMSessionBeginDocumentNoDialog.
=======
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMRelease(objct: PMObject): OSStatus; external name '_PMRelease';

{ Session Print loop }
{**********************}
{ A session is created with a refcount of 1. }
{**********************}
{
 *  PMCreateSession()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMCreateSession(var printSession: PMPrintSession): OSStatus; external name '_PMCreateSession';

{ Session PMPageFormat }
{**********************}
{ A pageformat is created with a refcount of 1. }
{**********************}
{
 *  PMCreatePageFormat()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMCreatePageFormat(var pageFormat: PMPageFormat): OSStatus; external name '_PMCreatePageFormat';

{
 *  PMSessionDefaultPageFormat()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionDefaultPageFormat(printSession: PMPrintSession; pageFormat: PMPageFormat): OSStatus; external name '_PMSessionDefaultPageFormat';

{
 *  PMSessionValidatePageFormat()
=======
function PMSessionEndPageNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionGetCGGraphicsContext()
 *  
 *  Summary:
 *    Return the CGContextRef for the current page in the printing
 *    session.
 *  
 *  Discussion:
 *    This function returns the CGContextRef for the printing session.
 *    This function must be called for each page. To use
 *    PMSessionGetCGGraphicsContext you MUST call
 *    PMSessionBeginCGDocument or PMSessionBeginCGDocumentNoDialog
 *    instead of PMSessionBeginDocument or
 *    PMSessionBeginDocumentNoDialog.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session for the print job.
 *    
 *    context:
 *      A pointer to a caller supplied CGContextRef variable. If this
 *      function succeeds then *context will be filled in with the
 *      printing CGContextRef.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMSessionValidatePageFormat(printSession: PMPrintSession; pageFormat: PMPageFormat; result: BooleanPtr): OSStatus; external name '_PMSessionValidatePageFormat';
=======
function PMSessionGetCGGraphicsContext( printSession: PMPrintSession; var context: CGContextRef ): OSStatus; external name '_PMSessionGetCGGraphicsContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

(*
#pragma mark
#pragma mark Session: Other routines
#pragma mark -
*)
>>>>>>> origin/fixes_2.4

{ Session PMPrintSettings }
{**********************}
{ A printSettings is created with a refcount of 1. }
{**********************}
{
<<<<<<< HEAD
 *  PMCreatePrintSettings()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMCreatePrintSettings(var printSettings: PMPrintSettings): OSStatus; external name '_PMCreatePrintSettings';

{
 *  PMSessionDefaultPrintSettings()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionDefaultPrintSettings(printSession: PMPrintSession; printSettings: PMPrintSettings): OSStatus; external name '_PMSessionDefaultPrintSettings';

{
 *  PMSessionValidatePrintSettings()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionValidatePrintSettings(printSession: PMPrintSession; printSettings: PMPrintSettings; result: BooleanPtr): OSStatus; external name '_PMSessionValidatePrintSettings';

{ Session Classic support }
{
 *  PMSessionGeneral()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionGeneral(printSession: PMPrintSession; pData: Ptr): OSStatus; external name '_PMSessionGeneral';

{
 *  PMSessionConvertOldPrintRecord()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionConvertOldPrintRecord(printSession: PMPrintSession; printRecordHandle: Handle; var printSettings: PMPrintSettings; var pageFormat: PMPageFormat): OSStatus; external name '_PMSessionConvertOldPrintRecord';

{
 *  PMSessionMakeOldPrintRecord()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionMakeOldPrintRecord(printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; var printRecordHandle: Handle): OSStatus; external name '_PMSessionMakeOldPrintRecord';

{ Session Driver Information }
{
 *  PMPrinterGetDescriptionURL()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetDescriptionURL(printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef): OSStatus; external name '_PMPrinterGetDescriptionURL';

{
 *  PMSessionGetCurrentPrinter()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionGetCurrentPrinter(printSession: PMPrintSession; var currentPrinter: PMPrinter): OSStatus; external name '_PMSessionGetCurrentPrinter';

{
 *  PMPrinterGetLanguageInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetLanguageInfo(printer: PMPrinter; var info: PMLanguageInfo): OSStatus; external name '_PMPrinterGetLanguageInfo';

{
 *  PMPrinterGetDriverCreator()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetDriverCreator(printer: PMPrinter; var creator: OSType): OSStatus; external name '_PMPrinterGetDriverCreator';

{
 *  PMPrinterGetDriverReleaseInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetDriverReleaseInfo(printer: PMPrinter; var release: VersRec): OSStatus; external name '_PMPrinterGetDriverReleaseInfo';

{
 *  PMPrinterGetPrinterResolutionCount()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetPrinterResolutionCount(printer: PMPrinter; var count: UInt32): OSStatus; external name '_PMPrinterGetPrinterResolutionCount';

{
 *  PMPrinterGetPrinterResolution()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetPrinterResolution(printer: PMPrinter; tag: PMTag; var res: PMResolution): OSStatus; external name '_PMPrinterGetPrinterResolution';

{
 *  PMPrinterGetIndexedPrinterResolution()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetIndexedPrinterResolution(printer: PMPrinter; index: UInt32; var res: PMResolution): OSStatus; external name '_PMPrinterGetIndexedPrinterResolution';

{
 *  PMPrinterIsPostScriptCapable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Mac OS X:         in version 10.2 and later
 }
function PMPrinterIsPostScriptCapable(printer: PMPrinter): boolean; external name '_PMPrinterIsPostScriptCapable';

{
 *  PMPrinterGetMakeAndModelName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Mac OS X:         in version 10.2 and later
 }
function PMPrinterGetMakeAndModelName(printer: PMPrinter; var makeAndModel: CFStringRef): OSStatus; external name '_PMPrinterGetMakeAndModelName';

{ Session ColorSync & PostScript Support }
{
 *  PMSessionEnableColorSync()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionEnableColorSync(printSession: PMPrintSession): OSStatus; external name '_PMSessionEnableColorSync';

{
 *  PMSessionDisableColorSync()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionDisableColorSync(printSession: PMPrintSession): OSStatus; external name '_PMSessionDisableColorSync';

{
 *  PMSessionPostScriptBegin()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptBegin(printSession: PMPrintSession): OSStatus; external name '_PMSessionPostScriptBegin';

{
 *  PMSessionPostScriptEnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptEnd(printSession: PMPrintSession): OSStatus; external name '_PMSessionPostScriptEnd';

{
 *  PMSessionPostScriptHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptHandle(printSession: PMPrintSession; psHandle: Handle): OSStatus; external name '_PMSessionPostScriptHandle';

{
 *  PMSessionPostScriptData()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptData(printSession: PMPrintSession; psPtr: Ptr; len: Size): OSStatus; external name '_PMSessionPostScriptData';

{
 *  PMSessionPostScriptFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptFile(printSession: PMPrintSession; var psFile: FSSpec): OSStatus; external name '_PMSessionPostScriptFile';

{
 *  PMSessionSetPSInjectionData()
 *  
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMRelease(objct: PMObject): OSStatus; external name '_PMRelease';

{ Session Print loop }
{**********************}
{ A session is created with a refcount of 1. }
{**********************}
{
 *  PMCreateSession()
=======
function PMSessionSetPSInjectionData(printSession: PMPrintSession; printSettings: PMPrintSettings; injectionDictArray: CFArrayRef): OSStatus; external name '_PMSessionSetPSInjectionData';

{ Session Error }
{
 *  PMSessionError()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMCreateSession(var printSession: PMPrintSession): OSStatus; external name '_PMCreateSession';

{ Session PMPageFormat }
{**********************}
{ A pageformat is created with a refcount of 1. }
{**********************}
{
 *  PMCreatePageFormat()
=======
function PMSessionError(printSession: PMPrintSession): OSStatus; external name '_PMSessionError';

{
 *  PMSessionSetError()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMCreatePageFormat(var pageFormat: PMPageFormat): OSStatus; external name '_PMCreatePageFormat';

{
 *  PMSessionDefaultPageFormat()
=======
function PMSessionSetError(printSession: PMPrintSession; printError: OSStatus): OSStatus; external name '_PMSessionSetError';

{ Other Session routines }
{
 *  PMSessionGetDocumentFormatGeneration()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMSessionDefaultPageFormat(printSession: PMPrintSession; pageFormat: PMPageFormat): OSStatus; external name '_PMSessionDefaultPageFormat';

{
 *  PMSessionValidatePageFormat()
=======
function PMSessionGetDocumentFormatGeneration(printSession: PMPrintSession; var docFormats: CFArrayRef): OSStatus; external name '_PMSessionGetDocumentFormatGeneration';

{
 *  PMSessionSetDocumentFormatGeneration()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMSessionValidatePageFormat(printSession: PMPrintSession; pageFormat: PMPageFormat; result: BooleanPtr): OSStatus; external name '_PMSessionValidatePageFormat';

{ Session PMPrintSettings }
{**********************}
{ A printSettings is created with a refcount of 1. }
{**********************}
{
 *  PMCreatePrintSettings()
=======
function PMSessionSetDocumentFormatGeneration(printSession: PMPrintSession; docFormat: CFStringRef; graphicsContextTypes: CFArrayRef; options: CFTypeRef): OSStatus; external name '_PMSessionSetDocumentFormatGeneration';

{
 *  PMSessionGetDocumentFormatSupported()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMCreatePrintSettings(var printSettings: PMPrintSettings): OSStatus; external name '_PMCreatePrintSettings';

{
 *  PMSessionDefaultPrintSettings()
=======
function PMSessionGetDocumentFormatSupported(printSession: PMPrintSession; var docFormats: CFArrayRef; limit: UInt32): OSStatus; external name '_PMSessionGetDocumentFormatSupported';

{
 *  PMSessionIsDocumentFormatSupported()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMSessionDefaultPrintSettings(printSession: PMPrintSession; printSettings: PMPrintSettings): OSStatus; external name '_PMSessionDefaultPrintSettings';

{
 *  PMSessionValidatePrintSettings()
=======
function PMSessionIsDocumentFormatSupported(printSession: PMPrintSession; docFormat: CFStringRef; var supported: boolean): OSStatus; external name '_PMSessionIsDocumentFormatSupported';

{
 *  PMSessionGetGraphicsContext()
 *  
 *  Parameters:
 *    
 *    printSession:
 *      the session
 *    
 *    graphicsContextType:
 *      either kPMGraphicsContextQuickdraw or
 *      kPMGraphicsContextCoreGraphics
 *    
 *    graphicsContext:
 *      returns a GrafPtr or a CGContextRef
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMSessionValidatePrintSettings(printSession: PMPrintSession; printSettings: PMPrintSettings; result: BooleanPtr): OSStatus; external name '_PMSessionValidatePrintSettings';

{ Session Classic support }
{
 *  PMSessionGeneral()
=======
function PMSessionGetGraphicsContext(printSession: PMPrintSession; graphicsContextType: CFStringRef; var graphicsContext: UnivPtr): OSStatus; external name '_PMSessionGetGraphicsContext';

{
 *  PMSessionSetIdleProc()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
<<<<<<< HEAD
function PMSessionGeneral(printSession: PMPrintSession; pData: Ptr): OSStatus; external name '_PMSessionGeneral';

{
 *  PMSessionConvertOldPrintRecord()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionConvertOldPrintRecord(printSession: PMPrintSession; printRecordHandle: Handle; var printSettings: PMPrintSettings; var pageFormat: PMPageFormat): OSStatus; external name '_PMSessionConvertOldPrintRecord';

{
 *  PMSessionMakeOldPrintRecord()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionMakeOldPrintRecord(printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; var printRecordHandle: Handle): OSStatus; external name '_PMSessionMakeOldPrintRecord';

{ Session Driver Information }
{
 *  PMPrinterGetDescriptionURL()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetDescriptionURL(printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef): OSStatus; external name '_PMPrinterGetDescriptionURL';

{
 *  PMSessionGetCurrentPrinter()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionGetCurrentPrinter(printSession: PMPrintSession; var currentPrinter: PMPrinter): OSStatus; external name '_PMSessionGetCurrentPrinter';

{
 *  PMPrinterGetLanguageInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetLanguageInfo(printer: PMPrinter; var info: PMLanguageInfo): OSStatus; external name '_PMPrinterGetLanguageInfo';

{
 *  PMPrinterGetDriverCreator()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetDriverCreator(printer: PMPrinter; var creator: OSType): OSStatus; external name '_PMPrinterGetDriverCreator';

{
 *  PMPrinterGetDriverReleaseInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetDriverReleaseInfo(printer: PMPrinter; var release: VersRec): OSStatus; external name '_PMPrinterGetDriverReleaseInfo';

{
 *  PMPrinterGetPrinterResolutionCount()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetPrinterResolutionCount(printer: PMPrinter; var count: UInt32): OSStatus; external name '_PMPrinterGetPrinterResolutionCount';

{
 *  PMPrinterGetPrinterResolution()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetPrinterResolution(printer: PMPrinter; tag: PMTag; var res: PMResolution): OSStatus; external name '_PMPrinterGetPrinterResolution';

{
 *  PMPrinterGetIndexedPrinterResolution()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMPrinterGetIndexedPrinterResolution(printer: PMPrinter; index: UInt32; var res: PMResolution): OSStatus; external name '_PMPrinterGetIndexedPrinterResolution';

{
 *  PMPrinterIsPostScriptCapable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Mac OS X:         in version 10.2 and later
 }
function PMPrinterIsPostScriptCapable(printer: PMPrinter): boolean; external name '_PMPrinterIsPostScriptCapable';

{
 *  PMPrinterGetMakeAndModelName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Mac OS X:         in version 10.2 and later
 }
function PMPrinterGetMakeAndModelName(printer: PMPrinter; var makeAndModel: CFStringRef): OSStatus; external name '_PMPrinterGetMakeAndModelName';

{ Session ColorSync & PostScript Support }
{
 *  PMSessionEnableColorSync()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionEnableColorSync(printSession: PMPrintSession): OSStatus; external name '_PMSessionEnableColorSync';

{
 *  PMSessionDisableColorSync()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionDisableColorSync(printSession: PMPrintSession): OSStatus; external name '_PMSessionDisableColorSync';

{
 *  PMSessionPostScriptBegin()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptBegin(printSession: PMPrintSession): OSStatus; external name '_PMSessionPostScriptBegin';

{
 *  PMSessionPostScriptEnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptEnd(printSession: PMPrintSession): OSStatus; external name '_PMSessionPostScriptEnd';

{
 *  PMSessionPostScriptHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptHandle(printSession: PMPrintSession; psHandle: Handle): OSStatus; external name '_PMSessionPostScriptHandle';

{
 *  PMSessionPostScriptData()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptData(printSession: PMPrintSession; psPtr: Ptr; len: Size): OSStatus; external name '_PMSessionPostScriptData';

{
 *  PMSessionPostScriptFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionPostScriptFile(printSession: PMPrintSession; var psFile: FSSpec): OSStatus; external name '_PMSessionPostScriptFile';

{
 *  PMSessionSetPSInjectionData()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionSetPSInjectionData(printSession: PMPrintSession; printSettings: PMPrintSettings; injectionDictArray: CFArrayRef): OSStatus; external name '_PMSessionSetPSInjectionData';

{ Session Error }
{
 *  PMSessionError()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionError(printSession: PMPrintSession): OSStatus; external name '_PMSessionError';

{
 *  PMSessionSetError()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionSetError(printSession: PMPrintSession; printError: OSStatus): OSStatus; external name '_PMSessionSetError';

{ Other Session routines }
{
 *  PMSessionGetDocumentFormatGeneration()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionGetDocumentFormatGeneration(printSession: PMPrintSession; var docFormats: CFArrayRef): OSStatus; external name '_PMSessionGetDocumentFormatGeneration';

{
 *  PMSessionSetDocumentFormatGeneration()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionSetDocumentFormatGeneration(printSession: PMPrintSession; docFormat: CFStringRef; graphicsContextTypes: CFArrayRef; options: CFTypeRef): OSStatus; external name '_PMSessionSetDocumentFormatGeneration';

{
 *  PMSessionGetDocumentFormatSupported()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionGetDocumentFormatSupported(printSession: PMPrintSession; var docFormats: CFArrayRef; limit: UInt32): OSStatus; external name '_PMSessionGetDocumentFormatSupported';

{
 *  PMSessionIsDocumentFormatSupported()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionIsDocumentFormatSupported(printSession: PMPrintSession; docFormat: CFStringRef; var supported: boolean): OSStatus; external name '_PMSessionIsDocumentFormatSupported';

{
 *  PMSessionGetGraphicsContext()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Parameters:
 *    
 *    printSession:
<<<<<<< HEAD
 *      The session for the print job.
 *    
 *    context:
 *      A pointer to a caller supplied CGContextRef variable. If this
 *      function succeeds then *context will be filled in with the
 *      printing CGContextRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetCGGraphicsContext( printSession: PMPrintSession; var context: CGContextRef ): OSStatus; external name '_PMSessionGetCGGraphicsContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

(*
#pragma mark
#pragma mark Session: Other routines
#pragma mark -
*)
<<<<<<< HEAD
=======
function PMSessionEndDocumentNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew

>>>>>>> graemeg/cpstrnew
{
<<<<<<< HEAD
=======

{
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginPageNoDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }
{
 *  PMSessionEndPageNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndPageNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionGetCGGraphicsContext()
 *  
 *  Summary:
 *    Return the CGContextRef for the current page in the printing
 *    session.
 *  
 *  Discussion:
 *    This function returns the CGContextRef for the printing session.
 *    This function must be called for each page. To use
 *    PMSessionGetCGGraphicsContext you MUST call
 *    PMSessionBeginCGDocument or PMSessionBeginCGDocumentNoDialog
 *    instead of PMSessionBeginDocument or
 *    PMSessionBeginDocumentNoDialog.
=======
 *      the session
 *    
 *    graphicsContextType:
 *      either kPMGraphicsContextQuickdraw or
 *      kPMGraphicsContextCoreGraphics
 *    
 *    graphicsContext:
 *      returns a GrafPtr or a CGContextRef
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionGetGraphicsContext(printSession: PMPrintSession; graphicsContextType: CFStringRef; var graphicsContext: UnivPtr): OSStatus; external name '_PMSessionGetGraphicsContext';

{
 *  PMSessionSetIdleProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionSetIdleProc(printSession: PMPrintSession; idleProc: PMIdleUPP): OSStatus; external name '_PMSessionSetIdleProc';

{
 *  PMSessionSetDataInSession()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionSetDataInSession(printSession: PMPrintSession; key: CFStringRef; data: CFTypeRef): OSStatus; external name '_PMSessionSetDataInSession';

{
 *  PMSessionGetDataFromSession()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionGetDataFromSession(printSession: PMPrintSession; key: CFStringRef; var data: CFTypeRef): OSStatus; external name '_PMSessionGetDataFromSession';

{
 *  PMSessionCreatePrinterList()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionCreatePrinterList(printSession: PMPrintSession; var printerList: CFArrayRef; var currentIndex: CFIndex; var currentPrinter: PMPrinter): OSStatus; external name '_PMSessionCreatePrinterList';

{
 *  PMSessionSetCurrentPrinter()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionSetCurrentPrinter(session: PMPrintSession; printerName: CFStringRef): OSStatus; external name '_PMSessionSetCurrentPrinter';

{
 *  PMSessionSetDestination()
 *  
 *  Summary:
 *    Alter a print session and print settings so that an associated
 *    print job is sent to the provided destination type in the,
 *    optional, MIME document format.
 *  
 *  Discussion:
 *    This function is most useful when an application would like to
 *    write its print output to disk without requiring user
 *    interaction. The list of MIME types that can be sent to the
 *    provided destination can be obtained from
 *    PMSessionCopyOutputFormatList and one of these passed to this
 *    function.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Parameters:
 *    
 *    printSession:
<<<<<<< HEAD
 *      The session for the print job.
 *    
 *    context:
 *      A pointer to a caller supplied CGContextRef variable. If this
 *      function succeeds then *context will be filled in with the
 *      printing CGContextRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetCGGraphicsContext( printSession: PMPrintSession; var context: CGContextRef ): OSStatus; external name '_PMSessionGetCGGraphicsContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

(*
#pragma mark
#pragma mark Session: Other routines
#pragma mark -
*)

{
>>>>>>> origin/cpstrnew
=======
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file. It also holds the requested MIME output
 *      type.
 *    
 *    destType:
 *      The destiation type for a print job associated with the
 *      provided print session and print settings. Fax is currently not
 *      supported, but kPMDestinationPrinter, kPMDestinationFile, and
 *      kPMDestinationPreview can be set.
 *    
 *    destFormat:
 *      The MIME type to be generated for the provided destination
 *      type. This parameter can be NULL in which the default format
 *      for the requested destination type is used. To obtain a list of
 *      valid formats for a given destiation type, use the function
 *      PMSessionCopyOutputFormatList.
 *    
 *    destLocation:
 *      Some destination types support a destination location. The
 *      clearest example is the kPMDestinationFile destination type
 *      which allows a caller to also supply a file URL specifying
 *      where the output file is to be created.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionSetDestination(printSession: PMPrintSession; printSettings: PMPrintSettings; destType: PMDestinationType; destFormat: CFStringRef; destLocation: CFURLRef): OSStatus; external name '_PMSessionSetDestination';

{
>>>>>>> graemeg/fixes_2_2
 *  PMSessionGetDestinationType()
 *  
 *  Summary:
 *    Hand back the destination type that will be used for a print job
 *    with the specified print settings and print session.
 *  
 *  Discussion:
<<<<<<< HEAD
 *    Currently there are five destination types:
 *    kPMDestinationPrinter, kPMDestinationFile, kPMDestinationFax and
 *    kPMDestinationPreview, and kPMDestinationProcessPDF.
=======
 *    Currently there are four destination types:
 *    kPMDestinationPrinter, kPMDestinationFile, kPMDestinationFax and
 *    kPMDestinationPreview. The first three destination types are
 *    stored in the print settings. The switch for preview is stored in
 *    the print session and, if enabled, overrides the destination in
 *    the print setting. This function is preferred over
 *    PMGetDestination as the latter does not take a print session
 *    parameter and therefore can not indicate whether preview has been
 *    selected as the destination.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file.
 *    
 *    destTypeP:
 *      A pointer to a caller supplied PMDestinationType variable. If
 *      this function succeeds then *'destTypeP' will be filled in with
 *      the destination type for a print job that used the specified
 *      session and print settings. If this function fails, then
 *      *'destType' will be set to kPMDestinationInvalid.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 }
function PMSessionGetDestinationType( printSession: PMPrintSession; printSettings: PMPrintSettings; var destTypeP: PMDestinationType ): OSStatus; external name '_PMSessionGetDestinationType';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
  
=======
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionGetDestinationType(printSession: PMPrintSession; printSettings: PMPrintSettings; var destTypeP: PMDestinationType): OSStatus; external name '_PMSessionGetDestinationType';

>>>>>>> graemeg/fixes_2_2
{
 *  PMSessionCopyDestinationFormat()
 *  
 *  Summary:
 *    Hand back the destination output MIME type associated with the
 *    provided print session and print settings.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      A currently open print session.
 *    
 *    printSettings:
 *      The print settings that are to be searched.
 *    
 *    destFormatP:
 *      A pointer to a caller allocated CFStringRef variable. If this
 *      routine returns noErr then *'destFormatP' will either be a copy
 *      of a CFStringRef specifying the output format for the print
 *      job, or NULL indicating that the default output format will be
 *      used. If this function return an error, then *'destFormatP'
 *      will be set to NULL.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 }
function PMSessionCopyDestinationFormat( printSession: PMPrintSession; printSettings: PMPrintSettings; var destFormatP: CFStringRef ): OSStatus; external name '_PMSessionCopyDestinationFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionCopyDestinationFormat(printSession: PMPrintSession; printSettings: PMPrintSettings; var destFormatP: CFStringRef): OSStatus; external name '_PMSessionCopyDestinationFormat';
>>>>>>> graemeg/fixes_2_2

{
 *  PMSessionCopyDestinationLocation()
 *  
 *  Summary:
 *    Hand back the URL destination location given a print session and
 *    print settings.
 *  
 *  Discussion:
 *    Some destination type support a destination location which
 *    further defines where the output from a pritn job should be sent.
 *    The kPMDestinationFile destiation type, for example, will use a
 *    file URL to determine where a new file should be created.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      A currently open print session.
 *    
 *    printSettings:
 *      The print settings that are to be searched.
 *    
 *    destLocationP:
 *      A pointer to a caller allocated CFURLRef variable. If this
 *      routine returns noErr then *'outputFileP' will either be NULL
 *      indicating that the job is using the default destination
 *      location for the current destination type or a copy of a
 *      CFURLRef will be placed in *'destLocationP'. If this function
 *      returns an error then 'destLocationP' will be set to NULL.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionCopyDestinationLocation( printSession: PMPrintSession; printSettings: PMPrintSettings; var destLocationP: CFURLRef ): OSStatus; external name '_PMSessionCopyDestinationLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
 }
function PMSessionCopyDestinationLocation( printSession: PMPrintSession; printSettings: PMPrintSettings; var destLocationP: CFURLRef ): OSStatus; external name '_PMSessionCopyDestinationLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew

=======
 }
function PMSessionCopyDestinationLocation( printSession: PMPrintSession; printSettings: PMPrintSettings; var destLocationP: CFURLRef ): OSStatus; external name '_PMSessionCopyDestinationLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  PMSessionSetDestination()
 *  
 *  Summary:
 *    Alter a print session and print settings so that an associated
 *    print job is sent to the provided destination type in the,
 *    optional, MIME document format.
 *  
 *  Discussion:
 *    This function is most useful when an application would like to
 *    write its print output to disk without requiring user
 *    interaction. The list of MIME types that can be sent to the
 *    provided destination can be obtained from
 *    PMSessionCopyOutputFormatList and one of these passed to this
 *    function.
=======
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionCopyDestinationLocation(printSession: PMPrintSession; printSettings: PMPrintSettings; var destLocationP: CFURLRef): OSStatus; external name '_PMSessionCopyDestinationLocation';

{
 *  PMSessionCopyOutputFormatList()
 *  
 *  Summary:
 *    Hands back an an array of MIME types describing the possible
 *    output formats for the printer module associated with the current
 *    printer.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Parameters:
 *    
 *    printSession:
<<<<<<< HEAD
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file. It also holds the requested MIME output
 *      type.
 *    
 *    destType:
 *      The destiation type for a print job associated with the
 *      provided print session and print settings. Fax is currently not
 *      supported, but kPMDestinationPrinter, kPMDestinationFile, and
 *      kPMDestinationPreview can be set.
 *    
 *    destFormat:
 *      The MIME type to be generated for the provided destination
 *      type. This parameter can be NULL in which the default format
 *      for the requested destination type is used. To obtain a list of
 *      valid formats for a given destiation type, use the function
 *      PMSessionCopyOutputFormatList.
 *    
 *    destLocation:
 *      Some destination types support a destination location. The
 *      clearest example is the kPMDestinationFile destination type
 *      which allows a caller to also supply a file URL specifying
 *      where the output file is to be created.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetDestination( printSession: PMPrintSession; printSettings: PMPrintSettings; destType: PMDestinationType; destFormat: CFStringRef; destLocation: CFURLRef ): OSStatus; external name '_PMSessionSetDestination';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
  
{
 *  PMSessionSetDestination()
 *  
 *  Summary:
 *    Alter a print session and print settings so that an associated
 *    print job is sent to the provided destination type in the,
 *    optional, MIME document format.
 *  
 *  Discussion:
 *    This function is most useful when an application would like to
 *    write its print output to disk without requiring user
 *    interaction. The list of MIME types that can be sent to the
 *    provided destination can be obtained from
 *    PMSessionCopyOutputFormatList and one of these passed to this
 *    function.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file. It also holds the requested MIME output
 *      type.
 *    
 *    destType:
 *      The destiation type for a print job associated with the
 *      provided print session and print settings. Fax is currently not
 *      supported, but kPMDestinationPrinter, kPMDestinationFile, and
 *      kPMDestinationPreview can be set.
 *    
 *    destFormat:
 *      The MIME type to be generated for the provided destination
 *      type. This parameter can be NULL in which the default format
 *      for the requested destination type is used. To obtain a list of
 *      valid formats for a given destiation type, use the function
 *      PMSessionCopyOutputFormatList.
 *    
 *    destLocation:
 *      Some destination types support a destination location. The
 *      clearest example is the kPMDestinationFile destination type
 *      which allows a caller to also supply a file URL specifying
 *      where the output file is to be created.
=======
 *      This session's current printer's printer module will be queried
 *      for its supported output MIME types.
 *    
 *    destType:
 *      A print job can have one of several possible destination types.
 *      The list of valid output formats is dependent upon the
 *      destination type. This parameter specifies destination type of
 *      interest when retrieving the output formats list.
 *    
 *    documentFormatP:
 *      A pointer to a caller's CFArrayRef variable. If this routine
 *      completes successfully, then *'documentFormatP' will be set to
 *      a CFArrayRef containing CFStringRefs. Each CFStringRef in the
 *      array is a MIME type specifying a type of output that can be
 *      generated by the printer module associated with the current
 *      printer.
>>>>>>> graemeg/fixes_2_2
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
<<<<<<< HEAD
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetDestination( printSession: PMPrintSession; printSettings: PMPrintSettings; destType: PMDestinationType; destFormat: CFStringRef; destLocation: CFURLRef ): OSStatus; external name '_PMSessionSetDestination';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
  
=======
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCopyOutputFormatList( printSession: PMPrintSession; destType: PMDestinationType; var documentFormatP: CFArrayRef ): OSStatus; external name '_PMSessionCopyOutputFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


>>>>>>> origin/cpstrnew
{
 *  PMSessionSetDestination()
 *  
 *  Summary:
 *    Alter a print session and print settings so that an associated
 *    print job is sent to the provided destination type in the,
 *    optional, MIME document format.
 *  
 *  Discussion:
 *    This function is most useful when an application would like to
 *    write its print output to disk without requiring user
 *    interaction. The list of MIME types that can be sent to the
 *    provided destination can be obtained from
 *    PMSessionCopyOutputFormatList and one of these passed to this
 *    function.
=======
 *      version 10.2 and later. On Mac OS 8/9 using CarbonLib, this
 *      routine returns kPMNotImplemented
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionCopyOutputFormatList(printSession: PMPrintSession; destType: PMDestinationType; var documentFormatP: CFArrayRef): OSStatus; external name '_PMSessionCopyOutputFormatList';


{
 *  PMSessionCreatePageFormatList()
 *  
 *  Summary:
 *    Hand back a list of page format instances. Each page format
 *    instance describes a paper size available on the specified
 *    printer.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Parameters:
 *    
 *    printSession:
<<<<<<< HEAD
<<<<<<< HEAD
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file. It also holds the requested MIME output
 *      type.
=======
 *      A currently valid print session.
 *    
 *    printer:
 *      The printer whose page size list should be enumerated. 
>>>>>>> origin/cpstrnew
 *    
 *    destType:
 *      The destiation type for a print job associated with the
 *      provided print session and print settings. Fax is currently not
 *      supported, but kPMDestinationPrinter, kPMDestinationFile, and
 *      kPMDestinationPreview can be set.
 *    
 *    destFormat:
 *      The MIME type to be generated for the provided destination
 *      type. This parameter can be NULL in which the default format
 *      for the requested destination type is used. To obtain a list of
 *      valid formats for a given destiation type, use the function
 *      PMSessionCopyOutputFormatList.
 *    
 *    destLocation:
 *      Some destination types support a destination location. The
 *      clearest example is the kPMDestinationFile destination type
 *      which allows a caller to also supply a file URL specifying
 *      where the output file is to be created.
 *    
 *  Discussion:
 *	If you want to create the page format list for the session's current printer,
 *      pass the PMPrinter object returned by PMSessionGetCurrentPrinter() as the
 *	printer parameter. 
 *
=======
 *      A currently open print session.
 *    
 *    printer:
 *      The printer whose page size list should be enumerated. To get
 *      the session's current printer, see PMSessionGetCurrentPrinter().
 *    
 *    pageFormatList:
 *      If this function is successful then noErr will be returned and
 *      *'pageFormatList' will be set to a newly created CFArray. Each
 *      element in the array will be a PMPageFormat describing an
 *      available paper size for the specified printer. If this
 *      function fails then a non-zero error code will be returned and
 *      *'pageFormatList' will be set to NULL.
 *    
>>>>>>> graemeg/fixes_2_2
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
<<<<<<< HEAD
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionSetDestination( printSession: PMPrintSession; printSettings: PMPrintSettings; destType: PMDestinationType; destFormat: CFStringRef; destLocation: CFURLRef ): OSStatus; external name '_PMSessionSetDestination';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
  
{
 *  PMSessionCopyOutputFormatList()
 *  
 *  Summary:
 *    Hands back an an array of MIME types describing the possible
 *    output formats for the printer module associated with the current
 *    printer.
=======
=======
 *      version 10.2 and later. On Mac OS 8/9 using CarbonLib, this
 *      routine returns kPMNotImplemented
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionCreatePageFormatList(printSession: PMPrintSession; printer: PMPrinter; var pageFormatList: CFArrayRef): OSStatus; external name '_PMSessionCreatePageFormatList';

{
>>>>>>> graemeg/fixes_2_2
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }
{
<<<<<<< HEAD
 *  PMSessionBeginPageNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginPageNoDialog( printSession: PMPrintSession; pageFormat: PMPageFormat; pageFrame: PMRectPtr ): OSStatus; external name '_PMSessionBeginPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *  PMSessionBeginDocumentNoDialog()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionBeginDocumentNoDialog(printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat): OSStatus; external name '_PMSessionBeginDocumentNoDialog';
>>>>>>> graemeg/fixes_2_2

{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }
{
<<<<<<< HEAD
 *  PMSessionEndPageNoDialog()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEndPageNoDialog( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEndPageNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSessionGetCGGraphicsContext()
 *  
 *  Summary:
 *    Return the CGContextRef for the current page in the printing
 *    session.
 *  
 *  Discussion:
 *    This function returns the CGContextRef for the printing session.
 *    This function must be called for each page. To use
 *    PMSessionGetCGGraphicsContext you MUST call
 *    PMSessionBeginCGDocument or PMSessionBeginCGDocumentNoDialog
 *    instead of PMSessionBeginDocument or
 *    PMSessionBeginDocumentNoDialog.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session for the print job.
 *    
 *    context:
 *      A pointer to a caller supplied CGContextRef variable. If this
 *      function succeeds then *context will be filled in with the
 *      printing CGContextRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetCGGraphicsContext( printSession: PMPrintSession; var context: CGContextRef ): OSStatus; external name '_PMSessionGetCGGraphicsContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

(*
#pragma mark
#pragma mark Session: Other routines
#pragma mark -
*)

{
 *  PMSessionGetDestinationType()
 *  
 *  Summary:
 *    Hand back the destination type that will be used for a print job
 *    with the specified print settings and print session.
 *  
 *  Discussion:
 *    Currently there are five destination types:
 *    kPMDestinationPrinter, kPMDestinationFile, kPMDestinationFax and
 *    kPMDestinationPreview, and kPMDestinationProcessPDF.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printSession:
 *      This session's current printer's printer module will be queried
 *      for its supported output MIME types.
 *    
 *    destType:
 *      A print job can have one of several possible destination types.
 *      The list of valid output formats is dependent upon the
 *      destination type. This parameter specifies destination type of
 *      interest when retrieving the output formats list.
 *    
 *    documentFormatP:
 *      A pointer to a caller's CFArrayRef variable. If this routine
 *      completes successfully, then *'documentFormatP' will be set to
 *      a CFArrayRef containing CFStringRefs. Each CFStringRef in the
 *      array is a MIME type specifying a type of output that can be
 *      generated by the printer module associated with the current
 *      printer.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later. On Mac OS 8/9 using CarbonLib, this
 *      routine returns kPMNotImplemented
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionCopyOutputFormatList( printSession: PMPrintSession; destType: PMDestinationType; var documentFormatP: CFArrayRef ): OSStatus; external name '_PMSessionCopyOutputFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


=======
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetDestinationType( printSession: PMPrintSession; printSettings: PMPrintSettings; var destTypeP: PMDestinationType ): OSStatus; external name '_PMSessionGetDestinationType';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
  
>>>>>>> graemeg/cpstrnew
{
 *  PMSessionCreatePageFormatList()
 *  
 *  Summary:
 *    Hand back a list of page format instances. Each page format
 *    instance describes a paper size available on the specified
 *    printer.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      A currently valid print session.
 *    
<<<<<<< HEAD
=======
 *    destFormatP:
 *      A pointer to a caller allocated CFStringRef variable. If this
 *      routine returns noErr then *'destFormatP' will either be a copy
 *      of a CFStringRef specifying the output format for the print
 *      job, or NULL indicating that the default output format will be
 *      used. If this function return an error, then *'destFormatP'
 *      will be set to NULL.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCopyDestinationFormat( printSession: PMPrintSession; printSettings: PMPrintSettings; var destFormatP: CFStringRef ): OSStatus; external name '_PMSessionCopyDestinationFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionCopyDestinationLocation()
 *  
 *  Summary:
 *    Hand back the URL destination location given a print session and
 *    print settings.
 *  
 *  Discussion:
 *    Some destination type support a destination location which
 *    further defines where the output from a pritn job should be sent.
 *    The kPMDestinationFile destiation type, for example, will use a
 *    file URL to determine where a new file should be created.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      A currently open print session.
 *    
 *    printSettings:
 *      The print settings that are to be searched.
 *    
 *    destLocationP:
 *      A pointer to a caller allocated CFURLRef variable. If this
 *      routine returns noErr then *'outputFileP' will either be NULL
 *      indicating that the job is using the default destination
 *      location for the current destination type or a copy of a
 *      CFURLRef will be placed in *'destLocationP'. If this function
 *      returns an error then 'destLocationP' will be set to NULL.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCopyDestinationLocation( printSession: PMPrintSession; printSettings: PMPrintSettings; var destLocationP: CFURLRef ): OSStatus; external name '_PMSessionCopyDestinationLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionSetDestination()
 *  
 *  Summary:
 *    Alter a print session and print settings so that an associated
 *    print job is sent to the provided destination type in the,
 *    optional, MIME document format.
 *  
 *  Discussion:
 *    This function is most useful when an application would like to
 *    write its print output to disk without requiring user
 *    interaction. The list of MIME types that can be sent to the
 *    provided destination can be obtained from
 *    PMSessionCopyOutputFormatList and one of these passed to this
 *    function.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file. It also holds the requested MIME output
 *      type.
 *    
 *    destType:
 *      The destiation type for a print job associated with the
 *      provided print session and print settings. Fax is currently not
 *      supported, but kPMDestinationPrinter, kPMDestinationFile, and
 *      kPMDestinationPreview can be set.
 *    
 *    destFormat:
 *      The MIME type to be generated for the provided destination
 *      type. This parameter can be NULL in which the default format
 *      for the requested destination type is used. To obtain a list of
 *      valid formats for a given destiation type, use the function
 *      PMSessionCopyOutputFormatList.
 *    
 *    destLocation:
 *      Some destination types support a destination location. The
 *      clearest example is the kPMDestinationFile destination type
 *      which allows a caller to also supply a file URL specifying
 *      where the output file is to be created.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetDestination( printSession: PMPrintSession; printSettings: PMPrintSettings; destType: PMDestinationType; destFormat: CFStringRef; destLocation: CFURLRef ): OSStatus; external name '_PMSessionSetDestination';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
  
{
 *  PMSessionCopyOutputFormatList()
 *  
 *  Summary:
 *    Hands back an an array of MIME types describing the possible
 *    output formats for the printer module associated with the current
 *    printer.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      This session's current printer's printer module will be queried
 *      for its supported output MIME types.
 *    
 *    destType:
 *      A print job can have one of several possible destination types.
 *      The list of valid output formats is dependent upon the
 *      destination type. This parameter specifies destination type of
 *      interest when retrieving the output formats list.
 *    
 *    documentFormatP:
 *      A pointer to a caller's CFArrayRef variable. If this routine
 *      completes successfully, then *'documentFormatP' will be set to
 *      a CFArrayRef containing CFStringRefs. Each CFStringRef in the
 *      array is a MIME type specifying a type of output that can be
 *      generated by the printer module associated with the current
 *      printer.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later. On Mac OS 8/9 using CarbonLib, this
 *      routine returns kPMNotImplemented
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCopyOutputFormatList( printSession: PMPrintSession; destType: PMDestinationType; var documentFormatP: CFArrayRef ): OSStatus; external name '_PMSessionCopyOutputFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionCreatePageFormatList()
 *  
 *  Summary:
 *    Hand back a list of page format instances. Each page format
 *    instance describes a paper size available on the specified
 *    printer.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      A currently valid print session.
 *    
>>>>>>> graemeg/cpstrnew
 *    printer:
 *      The printer whose page size list should be enumerated. 
 *    
 *    pageFormatList:
 *      If this function is successful then noErr will be returned and
 *      *'pageFormatList' will be set to a newly created CFArray. Each
 *      element in the array will be a PMPageFormat describing an
 *      available paper size for the specified printer. If this
 *      function fails then a non-zero error code will be returned and
 *      *'pageFormatList' will be set to NULL.
 *    
 *  Discussion:
 *	If you want to create the page format list for the session's current printer,
 *      pass the PMPrinter object returned by PMSessionGetCurrentPrinter() as the
 *	printer parameter. 
 *
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later. On Mac OS 8/9 using CarbonLib, this
 *      routine returns kPMNotImplemented
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionCreatePageFormatList( printSession: PMPrintSession; printer: PMPrinter; var pageFormatList: CFArrayRef ): OSStatus; external name '_PMSessionCreatePageFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionCreatePrinterList()
 *
 *  See also: PMServerCreatePrinterList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionCreatePrinterList( printSession: PMPrintSession; var printerList: CFArrayRef; var currentIndex: CFIndex; var currentPrinter: PMPrinter ): OSStatus; external name '_PMSessionCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  PMSessionGetCurrentPrinter()
 *  
 *  Summary:
 *    Hand back the session's current printer.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionGetCurrentPrinter( printSession: PMPrintSession; var currentPrinter: PMPrinter ): OSStatus; external name '_PMSessionGetCurrentPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionSetCurrentPMPrinter()
 *  
 *  Summary:
 *    Set the session's current printer to a specified PMPrinter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionSetCurrentPMPrinter( session: PMPrintSession; printer: PMPrinter ): OSStatus; external name '_PMSessionSetCurrentPMPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMSessionGetDataFromSession()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionGetDataFromSession( printSession: PMPrintSession; key: CFStringRef; var data: CFTypeRef ): OSStatus; external name '_PMSessionGetDataFromSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionSetDataInSession()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionSetDataInSession( printSession: PMPrintSession; key: CFStringRef; data: CFTypeRef ): OSStatus; external name '_PMSessionSetDataInSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark -
#pragma mark PageFormat
#pragma mark
*)

{
 *  PMCreatePageFormat()
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Summary:
 *    Allocates memory for a new PMPageFormat object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A pageformat is created with a refcount of 1.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMCreatePageFormat( var pageFormat: PMPageFormat ): OSStatus; external name '_PMCreatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPageFormat()
=======
 *  
=======
 *  
>>>>>>> graemeg/cpstrnew
 *  Summary:
 *    Allocates memory for a new PMPageFormat object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A pageformat is created with a refcount of 1.
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionDefaultPageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionDefaultPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePageFormat()
=======
 }
function PMCreatePageFormat( var pageFormat: PMPageFormat ): OSStatus; external name '_PMCreatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPageFormat()
>>>>>>> graemeg/cpstrnew
=======
 }
function PMCreatePageFormat( var pageFormat: PMPageFormat ): OSStatus; external name '_PMCreatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDefaultPageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionDefaultPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePageFormat()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
 }
function PMSessionValidatePageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
 }
function PMCopyPageFormat( formatSrc: PMPageFormat; formatDest: PMPageFormat ): OSStatus; external name '_PMCopyPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMCreatePageFormatWithPMPaper()
 *  
 *  Summary:
 *    Create a pageformat with a specific paper.
 *  
 *  Parameters:
 *    
 *    pageFormat:
 *      On return, will contain the pageformat which was created
 *    
 *    paper:
 *      The paper that will be associate with the pageformat
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
>>>>>>> graemeg/cpstrnew
 }
function PMCreatePageFormatWithPMPaper( var pageFormat: PMPageFormat; paper: PMPaper ): OSStatus; external name '_PMCreatePageFormatWithPMPaper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPageFormatCreateDataRepresentation()
<<<<<<< HEAD
 *  
 *  Summary:
 *    Returns a data representation of a PMPageFormat object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
 *  
=======
 *  
 *  Summary:
 *    Returns a data representation of a PMPageFormat object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
 *  
>>>>>>> graemeg/cpstrnew
 *  Discussion:
 *    This function is similar to PMFlattenPageFormatToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPageFormatCreateWithDataRepresentation to create a PMPageFormat from 
 *    a CFDataRef created by this call.
<<<<<<< HEAD
=======
 }
function PMSessionDefaultPageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionDefaultPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPageFormat()
=======
 *  
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateDataRepresentation( pageFormat: PMPageFormat; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPageFormatCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPageFormatCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPageFormat object from a data representation created with
 *	  PMPageFormatCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateWithDataRepresentation( data: CFDataRef; var pageFormat: PMPageFormat ): OSStatus; external name '_PMPageFormatCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)



{
 *  PMGetAdjustedPageRect()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMCopyPageFormat( formatSrc: PMPageFormat; formatDest: PMPageFormat ): OSStatus; external name '_PMCopyPageFormat';
=======
function PMGetAdjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetAdjustedPageRect';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMCreatePageFormatWithPMPaper()
 *  
 *  Summary:
 *    Create a pageformat with a specific paper.
=======
 *  PMGetAdjustedPaperRect()
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    pageFormat:
 *      On return, will contain the pageformat which was created
 *    
 *    paper:
 *      The paper that will be associate with the pageformat
>>>>>>> graemeg/cpstrnew
 *  
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPageFormatCreateDataRepresentation( pageFormat: PMPageFormat; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPageFormatCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPageFormatCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPageFormat object from a data representation created with
 *	  PMPageFormatCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePageFormatWithPMPaper( var pageFormat: PMPageFormat; paper: PMPaper ): OSStatus; external name '_PMCreatePageFormatWithPMPaper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPageFormatCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPageFormat object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPageFormatToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPageFormatCreateWithDataRepresentation to create a PMPageFormat from 
 *    a CFDataRef created by this call.
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetAdjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetAdjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetOrientation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetOrientation( pageFormat: PMPageFormat; var orientation: PMOrientation ): OSStatus; external name '_PMGetOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetPageFormatExtendedData()
>>>>>>> graemeg/cpstrnew
 *  
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPageFormatCreateWithDataRepresentation( data: CFDataRef; var pageFormat: PMPageFormat ): OSStatus; external name '_PMPageFormatCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)



{
 *  PMGetAdjustedPageRect()
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; var size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMGetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPageFormatGetPrinterID()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *		Obtains the formatting printer for the pageformat.
 *
 *  Discussion:
 *		Will either return the formatting printer for the pageformat
 *		or will return NULL if the pageformat doesn't have that information.
 * 
 *  Parameters:
 *    
 *    pageFormat:
 *      The pageformat to obtain the information from.
 *
 *	  printerID:
 *		Where to store the name of the printer
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetAdjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetAdjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetAdjustedPaperRect()
=======
 }
function PMPageFormatCreateDataRepresentation( pageFormat: PMPageFormat; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPageFormatCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPageFormatCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPageFormat object from a data representation created with
 *	  PMPageFormatCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateWithDataRepresentation( data: CFDataRef; var pageFormat: PMPageFormat ): OSStatus; external name '_PMPageFormatCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)



{
 *  PMGetAdjustedPageRect()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetAdjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetAdjustedPaperRect';
=======
 }
function PMGetAdjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetAdjustedPageRect';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMGetOrientation()
=======
 *  PMGetAdjustedPaperRect()
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatGetPrinterID( pageFormat: PMPageFormat; var printerID: CFStringRef ): OSStatus; external name '_PMPageFormatGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMGetScale()
 *  
 *  Summary:
 *    Obtains the scaling factor currently applied to the page and
 *    paper rectangles.
 *  
 *  Discussion:
 *    A value of 100.0 means 100% (no scaling). 50.0 means 50% scaling
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetScale( pageFormat: PMPageFormat; var scale: Float64 ): OSStatus; external name '_PMGetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPageRect()
 *  
 *  Summary:
 *    Obtains the size of the imageable area in points, unaffected by
 *    orientation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the imageable area of the page without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPaperRect()
 *  
 *  Summary:
 *    Obtains a rectangle that specifies the size of the paper in
 *    points, unaffected by rotation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the physical size of the paper without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)



{**********************}
{ PMSetxxx calls only save the value inside the printing object. They make no assumption on the }
{ validity of the value. This should be done using PMValidatePageFormat/PMSessionValidatePageFormat }
{ Any dependent settings are also updated during a validate call. }
{ For example: }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 )  }
{ PMGetAdjustedPaperRect - returns the SAME rect as the first call  }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 ) }
{ PMValidatePageFormat or PMSessionValidatePageFormat }
{ PMGetAdjustedPaperRect - returns a rect thats scaled 500% from the first call }
{**********************}

{
 *  PMSetOrientation()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetOrientation( pageFormat: PMPageFormat; var orientation: PMOrientation ): OSStatus; external name '_PMGetOrientation';
=======
 }
function PMGetAdjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetAdjustedPaperRect';
>>>>>>> graemeg/cpstrnew
=======
 }
function PMSetOrientation( pageFormat: PMPageFormat; orientation: PMOrientation; lock: Boolean ): OSStatus; external name '_PMSetOrientation';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMGetPageFormatExtendedData()
=======
 *  PMGetOrientation()
>>>>>>> graemeg/cpstrnew
=======
 *  PMSetPageFormatExtendedData()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; var size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMGetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPageFormatGetPrinterID()
=======
 }
function PMGetOrientation( pageFormat: PMPageFormat; var orientation: PMOrientation ): OSStatus; external name '_PMGetOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 }
function PMSetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMSetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetScale()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetScale( pageFormat: PMPageFormat; scale: Float64 ): OSStatus; external name '_PMSetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

(*
#pragma mark -
#pragma mark PrintSettings
#pragma mark
*)

{$ifc PM_USE_SESSION_APIS}
{
<<<<<<< HEAD
 *  PMGetPageFormatExtendedData()
>>>>>>> graemeg/cpstrnew
=======
 *  PMCreatePrintSettings()
 *  
 *  Summary:
 *    Allocates memory for a new PMPrintSettings object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A printSettings is created with a refcount of 1.
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *		Obtains the formatting printer for the pageformat.
 *
 *  Discussion:
 *		Will either return the formatting printer for the pageformat
 *		or will return NULL if the pageformat doesn't have that information.
 * 
 *  Parameters:
 *    
 *    pageFormat:
 *      The pageformat to obtain the information from.
 *
 *	  printerID:
 *		Where to store the name of the printer
 *
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPageFormatGetPrinterID( pageFormat: PMPageFormat; var printerID: CFStringRef ): OSStatus; external name '_PMPageFormatGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMGetScale()
 *  
 *  Summary:
 *    Obtains the scaling factor currently applied to the page and
 *    paper rectangles.
 *  
 *  Discussion:
 *    A value of 100.0 means 100% (no scaling). 50.0 means 50% scaling
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; var size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMGetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPageFormatGetPrinterID()
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePrintSettings( var printSettings: PMPrintSettings ): OSStatus; external name '_PMCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDefaultPrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings ): OSStatus; external name '_PMSessionDefaultPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePrintSettings()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *		Obtains the formatting printer for the pageformat.
 *
 *  Discussion:
 *		Will either return the formatting printer for the pageformat
 *		or will return NULL if the pageformat doesn't have that information.
 * 
 *  Parameters:
 *    
 *    pageFormat:
 *      The pageformat to obtain the information from.
 *
 *	  printerID:
 *		Where to store the name of the printer
 *
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetScale( pageFormat: PMPageFormat; var scale: Float64 ): OSStatus; external name '_PMGetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPageRect()
 *  
 *  Summary:
 *    Obtains the size of the imageable area in points, unaffected by
 *    orientation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the imageable area of the page without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetUnadjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPageRect';
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatGetPrinterID( pageFormat: PMPageFormat; var printerID: CFStringRef ): OSStatus; external name '_PMPageFormatGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMGetScale()
 *  
 *  Summary:
 *    Obtains the scaling factor currently applied to the page and
 *    paper rectangles.
 *  
 *  Discussion:
 *    A value of 100.0 means 100% (no scaling). 50.0 means 50% scaling
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPrintSettings( settingSrc: PMPrintSettings; settingDest: PMPrintSettings ): OSStatus; external name '_PMCopyPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPrintSettings object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPrintSettingsToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPrintSettingsCreateWithDataRepresentation to create a PMPrintSettings from 
 *    a CFDataRef created by this call.
 *
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetScale( pageFormat: PMPageFormat; var scale: Float64 ): OSStatus; external name '_PMGetScale';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMGetUnadjustedPaperRect()
 *  
 *  Summary:
 *    Obtains a rectangle that specifies the size of the paper in
 *    points, unaffected by rotation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the physical size of the paper without regard to
=======
 *  PMGetUnadjustedPageRect()
 *  
 *  Summary:
 *    Obtains the size of the imageable area in points, unaffected by
 *    orientation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the imageable area of the page without regard to
>>>>>>> graemeg/cpstrnew
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetUnadjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)



{**********************}
{ PMSetxxx calls only save the value inside the printing object. They make no assumption on the }
{ validity of the value. This should be done using PMValidatePageFormat/PMSessionValidatePageFormat }
{ Any dependent settings are also updated during a validate call. }
{ For example: }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 )  }
{ PMGetAdjustedPaperRect - returns the SAME rect as the first call  }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 ) }
{ PMValidatePageFormat or PMSessionValidatePageFormat }
{ PMGetAdjustedPaperRect - returns a rect thats scaled 500% from the first call }
{**********************}

{
 *  PMSetOrientation()
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCreateDataRepresentation( printSettings: PMPrintSettings; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPrintSettingsCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPrintSettings object from a data representation created with
 *	  PMPrintSettingsCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *    
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCreateWithDataRepresentation( data: CFDataRef; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPrintSettingsCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


  
{
 *  PMGetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCollate( printSettings: PMPrintSettings; var collate: Boolean ): OSStatus; external name '_PMGetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMGetCopies()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetOrientation( pageFormat: PMPageFormat; orientation: PMOrientation; lock: Boolean ): OSStatus; external name '_PMSetOrientation';
=======
 *    Non-Carbon CFM:   not available
 }
function PMGetCopies( printSettings: PMPrintSettings; var copies: UInt32 ): OSStatus; external name '_PMGetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDuplex( printSettings: PMPrintSettings; var duplexSetting: PMDuplexMode ): OSStatus; external name '_PMGetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMGetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetFirstPage( printSettings: PMPrintSettings; var first: UInt32 ): OSStatus; external name '_PMGetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetLastPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetLastPage( printSettings: PMPrintSettings; var last: UInt32 ): OSStatus; external name '_PMGetLastPage';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMSetPageFormatExtendedData()
=======
 }
function PMGetUnadjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPaperRect()
 *  
 *  Summary:
 *    Obtains a rectangle that specifies the size of the paper in
 *    points, unaffected by rotation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the physical size of the paper without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)



{**********************}
{ PMSetxxx calls only save the value inside the printing object. They make no assumption on the }
{ validity of the value. This should be done using PMValidatePageFormat/PMSessionValidatePageFormat }
{ Any dependent settings are also updated during a validate call. }
{ For example: }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 )  }
{ PMGetAdjustedPaperRect - returns the SAME rect as the first call  }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 ) }
{ PMValidatePageFormat or PMSessionValidatePageFormat }
{ PMGetAdjustedPaperRect - returns a rect thats scaled 500% from the first call }
{**********************}

{
 *  PMSetOrientation()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMSetPageFormatExtendedData';
=======
 }
function PMSetOrientation( pageFormat: PMPageFormat; orientation: PMOrientation; lock: Boolean ): OSStatus; external name '_PMSetOrientation';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMSetScale()
=======
 *  PMSetPageFormatExtendedData()
>>>>>>> graemeg/cpstrnew
=======
 *  PMGetPageRange()
 *  
 *  Summary:
 *    Obtains the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The default page range is 1 - (all pages). The page range is
 *    something that is set by the application. It is NOT the first and
 *    last page to print. It serves as limits for setting the first and
 *    last page.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageRange( printSettings: PMPrintSettings; var minPage: UInt32; var maxPage: UInt32 ): OSStatus; external name '_PMGetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrintSettingsGetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetJobName( printSettings: PMPrintSettings; var name: CFStringRef ): OSStatus; external name '_PMPrintSettingsGetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  PMPrintSettingsGetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings to retrieve the value from
 *    
 *    key:
 *      The key to look for
 *    
 *    value:
 *      The return value. Its either the value for the key or NULL
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetValue( printSettings: PMPrintSettings; key: CFStringRef; var value: CFTypeRef ): OSStatus; external name '_PMPrintSettingsGetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)



{
 *  PMSetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCollate( printSettings: PMPrintSettings; collate: Boolean ): OSStatus; external name '_PMSetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMSetCopies()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetScale( pageFormat: PMPageFormat; scale: Float64 ): OSStatus; external name '_PMSetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark -
#pragma mark PrintSettings
#pragma mark
*)
=======
 }
function PMSetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMSetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew

{$ifc PM_USE_SESSION_APIS}
{
<<<<<<< HEAD
 *  PMCreatePrintSettings()
 *  
 *  Summary:
 *    Allocates memory for a new PMPrintSettings object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A printSettings is created with a refcount of 1.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMCreatePrintSettings( var printSettings: PMPrintSettings ): OSStatus; external name '_PMCreatePrintSettings';
=======
 *  PMSetScale()
=======
 }
function PMSetCopies( printSettings: PMPrintSettings; copies: UInt32; lock: Boolean ): OSStatus; external name '_PMSetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetDuplex( printSettings: PMPrintSettings; duplexSetting: PMDuplexMode ): OSStatus; external name '_PMSetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSetFirstPage()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMSetScale( pageFormat: PMPageFormat; scale: Float64 ): OSStatus; external name '_PMSetScale';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark -
#pragma mark PrintSettings
#pragma mark
*)

{$ifc PM_USE_SESSION_APIS}
{
<<<<<<< HEAD
 *  PMSessionDefaultPrintSettings()
=======
 *  PMCreatePrintSettings()
 *  
 *  Summary:
 *    Allocates memory for a new PMPrintSettings object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A printSettings is created with a refcount of 1.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionDefaultPrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings ): OSStatus; external name '_PMSessionDefaultPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePrintSettings()
=======
 }
function PMCreatePrintSettings( var printSettings: PMPrintSettings ): OSStatus; external name '_PMCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPrintSettings()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSessionValidatePrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMCopyPrintSettings( settingSrc: PMPrintSettings; settingDest: PMPrintSettings ): OSStatus; external name '_PMCopyPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPrintSettings object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
=======
 }
function PMSessionDefaultPrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings ): OSStatus; external name '_PMSessionDefaultPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPrintSettings()
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPrintSettingsToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPrintSettingsCreateWithDataRepresentation to create a PMPrintSettings from 
 *    a CFDataRef created by this call.
 *
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsCreateDataRepresentation( printSettings: PMPrintSettings; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPrintSettingsCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPrintSettings object from a data representation created with
 *	  PMPrintSettingsCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *    
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPrintSettings( settingSrc: PMPrintSettings; settingDest: PMPrintSettings ): OSStatus; external name '_PMCopyPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPrintSettings object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPrintSettingsToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPrintSettingsCreateWithDataRepresentation to create a PMPrintSettings from 
 *    a CFDataRef created by this call.
 *
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsCreateWithDataRepresentation( data: CFDataRef; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPrintSettingsCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
 }
function PMPrintSettingsCreateDataRepresentation( printSettings: PMPrintSettings; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPrintSettingsCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
function PMSetFirstPage( printSettings: PMPrintSettings; first: UInt32; lock: Boolean ): OSStatus; external name '_PMSetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetLastPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetLastPage( printSettings: PMPrintSettings; last: UInt32; lock: Boolean ): OSStatus; external name '_PMSetLastPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetPageRange()
 *  
 *  Summary:
 *    Sets the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The first and last page are immediately clipped to the new range.
 *    You may pass kPMPrintAllPages for the maxPage value to specified
 *    that all pages are available for printing.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPageRange( printSettings: PMPrintSettings; minPage: UInt32; maxPage: UInt32 ): OSStatus; external name '_PMSetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsSetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetJobName( printSettings: PMPrintSettings; name: CFStringRef ): OSStatus; external name '_PMPrintSettingsSetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrintSettingsSetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings in which to set the value
 *    
 *    key:
 *      The key to store the value in
 *    
 *    value:
 *      The value to store in the key. If NULL, any existing setting item with
 *      the specified key is removed.
 *    
 *    locked:
 *      A boolean value indicating whether the item being set should be
 *      locked. It is strongly recommended to pass false.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetValue( printSettings: PMPrintSettings; key: CFStringRef; value: CFTypeRef; locked: Boolean ): OSStatus; external name '_PMPrintSettingsSetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
  
  
{
 *  PMPrintSettingsCopyAsDictionary()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Represent these print settings as a dictionary.
 *    
 *    settingsDictionary:
 *      On exit, if successful *'settingsDictionary' will contain a reference to
 *      a CFDictionary describing the print settings. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'settingsDictionary' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCopyAsDictionary( printSettings: PMPrintSettings; var settingsDictionary: CFDictionaryRef ): OSStatus; external name '_PMPrintSettingsCopyAsDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrintSettingsCopyKeys()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Return the keys for items in this print settings.
 *    
 *    settingsKeys:
 *      On exit, if successful *'settingsKeys' will contain a reference to
 *      a CFArray describing the item keys in the print settings. Each of these
 *		keys may be passed to PMPrintSettingsGetValue() to obtain a value.
 *		The caller is responsible for releasing this reference. If this call returns
 *      an error, then *'settingsKeys' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCopyKeys( printSettings: PMPrintSettings; var settingsKeys: CFArrayRef ): OSStatus; external name '_PMPrintSettingsCopyKeys';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark -
#pragma mark Printer
#pragma mark
*)

{!
 *
 * PMCreateGenericPrinter
 *  Summary:
 *	Creates a generic PMPrinter
 *
 *  Parameters:
 *		printer:
 *		On return *printer contains the generic priner
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMCreateGenericPrinter( var printer: PMPrinter ): OSStatus; external name '_PMCreateGenericPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMServerCreatePrinterList()
 *  
 *  Summary:
 *    Hand back an array of PMPrinter objects that represents the list of printers
 *    known to the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server for which you want to obtain the list of printers.
 *      Use kPMServerLocal for the local print server.
 *    
 *    printerList:
 *      On return, *printerList contains the array of PMPrinter objects returned. The
 *	caller is responsible for releasing this array. On error, *printerList is NULL.
 *    
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMServerCreatePrinterList( server: PMServer; var printerList: CFArrayRef ): OSStatus; external name '_PMServerCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> graemeg/cpstrnew

>>>>>>> graemeg/cpstrnew


  
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMGetCollate()
=======
 *  PMServerLaunchPrinterBrowser()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Launch the printer browser, browsing the printers available for
 *    the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server whose available printers the browser should browse.
 *      Pass kPMServerLocal for the local print server.
 *    
 *    options:
 *      A CFDictionaryRef specifying how the browser should be presented. 
 *      Passing NULL presents the browser in the default fashion. 
 *
 *  Discussion:	The local print server allows its printers to be browsed but
 *		not all print servers do. Passing in a server whose printers cannot be
 *      browsed returns the error code kPMInvalidParameter.
 *    
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetCollate( printSettings: PMPrintSettings; var collate: Boolean ): OSStatus; external name '_PMGetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMGetCopies()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetCopies( printSettings: PMPrintSettings; var copies: UInt32 ): OSStatus; external name '_PMGetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetDuplex()
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMServerLaunchPrinterBrowser( server: PMServer; options: CFDictionaryRef ): OSStatus; external name '_PMServerLaunchPrinterBrowser';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterCreateFromPrinterID()
 *  
 *  Summary:
 *    Create a PMPrinter instance from the supplied printerID.
 *  
 *  Discussion:
 *    Returns a PMPrinter instance for the printer whose ID is
 *    printerID. The caller is responsible for releasing this instance
 *    with PMRelease. The PMPrinter instance returned will be NULL if
 *    there is no printer available which corresponds to the supplied
 *    printerID.
 *  
 *  Parameters:
 *    
 *    printerID:
 *      The printerID for the printer for which you want the PMPrinter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCreateFromPrinterID( printerID: CFStringRef ): PMPrinter; external name '_PMPrinterCreateFromPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyDescriptionURL()
 *  
 *  Summary:
 *    Hand back a URL to the printer's PostScript Printer Description (PPD) file in fileURL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose PPD will be returned.
 *    
 *    descriptionType:
 *      The type of description desired. Only kPMPPDDescriptionType is currently supported.
 *    
 *    fileURL:
 *      A pointer to storage for a CFURL reference that will be returned. The caller is responsible
 *      for releasing the reference returned. If this call returns an error, then *fileURL will
 *      be set to NULL.
 *
 *  Discussion:
 *    Only descriptionType of kPMPPDDescriptionType is supported, otherwise kPMInvalidParameter
 *    is returned.
 *
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetDuplex( printSettings: PMPrintSettings; var duplexSetting: PMDuplexMode ): OSStatus; external name '_PMGetDuplex';
=======
 }
function PMPrinterCopyDescriptionURL( printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef ): OSStatus; external name '_PMPrinterCopyDescriptionURL';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
<<<<<<< HEAD
 *  PMGetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetFirstPage( printSettings: PMPrintSettings; var first: UInt32 ): OSStatus; external name '_PMGetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetLastPage()
=======
 *  PMPrintSettingsCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPrintSettings object from a data representation created with
 *	  PMPrintSettingsCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *    
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCreateWithDataRepresentation( data: CFDataRef; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPrintSettingsCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


  
{
 *  PMGetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCollate( printSettings: PMPrintSettings; var collate: Boolean ): OSStatus; external name '_PMGetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMGetCopies()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCopies( printSettings: PMPrintSettings; var copies: UInt32 ): OSStatus; external name '_PMGetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDuplex( printSettings: PMPrintSettings; var duplexSetting: PMDuplexMode ): OSStatus; external name '_PMGetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMGetFirstPage()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetLastPage( printSettings: PMPrintSettings; var last: UInt32 ): OSStatus; external name '_PMGetLastPage';
=======
 }
function PMGetFirstPage( printSettings: PMPrintSettings; var first: UInt32 ): OSStatus; external name '_PMGetFirstPage';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMGetPageRange()
 *  
 *  Summary:
 *    Obtains the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The default page range is 1 - (all pages). The page range is
 *    something that is set by the application. It is NOT the first and
 *    last page to print. It serves as limits for setting the first and
 *    last page.
=======
 *  PMGetLastPage()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetPageRange( printSettings: PMPrintSettings; var minPage: UInt32; var maxPage: UInt32 ): OSStatus; external name '_PMGetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrintSettingsGetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsGetJobName( printSettings: PMPrintSettings; var name: CFStringRef ): OSStatus; external name '_PMPrintSettingsGetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  PMPrintSettingsGetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings to retrieve the value from
 *    
 *    key:
 *      The key to look for
 *    
 *    value:
 *      The return value. Its either the value for the key or NULL
=======
 }
function PMGetLastPage( printSettings: PMPrintSettings; var last: UInt32 ): OSStatus; external name '_PMGetLastPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetPageRange()
 *  
 *  Summary:
 *    Obtains the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The default page range is 1 - (all pages). The page range is
 *    something that is set by the application. It is NOT the first and
 *    last page to print. It serves as limits for setting the first and
 *    last page.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageRange( printSettings: PMPrintSettings; var minPage: UInt32; var maxPage: UInt32 ): OSStatus; external name '_PMGetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrintSettingsGetJobName()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsGetValue( printSettings: PMPrintSettings; key: CFStringRef; var value: CFTypeRef ): OSStatus; external name '_PMPrintSettingsGetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)



{
 *  PMSetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetCollate( printSettings: PMPrintSettings; collate: Boolean ): OSStatus; external name '_PMSetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMSetCopies()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetCopies( printSettings: PMPrintSettings; copies: UInt32; lock: Boolean ): OSStatus; external name '_PMSetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetDuplex( printSettings: PMPrintSettings; duplexSetting: PMDuplexMode ): OSStatus; external name '_PMSetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetFirstPage( printSettings: PMPrintSettings; first: UInt32; lock: Boolean ): OSStatus; external name '_PMSetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetLastPage()
=======
 }
function PMPrintSettingsGetJobName( printSettings: PMPrintSettings; var name: CFStringRef ): OSStatus; external name '_PMPrintSettingsGetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  PMPrintSettingsGetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings to retrieve the value from
 *    
 *    key:
 *      The key to look for
 *    
 *    value:
 *      The return value. Its either the value for the key or NULL
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetValue( printSettings: PMPrintSettings; key: CFStringRef; var value: CFTypeRef ): OSStatus; external name '_PMPrintSettingsGetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)



{
 *  PMSetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCollate( printSettings: PMPrintSettings; collate: Boolean ): OSStatus; external name '_PMSetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMSetCopies()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCopies( printSettings: PMPrintSettings; copies: UInt32; lock: Boolean ): OSStatus; external name '_PMSetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetDuplex( printSettings: PMPrintSettings; duplexSetting: PMDuplexMode ): OSStatus; external name '_PMSetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSetFirstPage()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetLastPage( printSettings: PMPrintSettings; last: UInt32; lock: Boolean ): OSStatus; external name '_PMSetLastPage';
=======
 }
function PMSetFirstPage( printSettings: PMPrintSettings; first: UInt32; lock: Boolean ): OSStatus; external name '_PMSetFirstPage';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMSetPageRange()
 *  
 *  Summary:
 *    Sets the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The first and last page are immediately clipped to the new range.
 *    You may pass kPMPrintAllPages for the maxPage value to specified
 *    that all pages are available for printing.
=======
 *  PMSetLastPage()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMSetPageRange( printSettings: PMPrintSettings; minPage: UInt32; maxPage: UInt32 ): OSStatus; external name '_PMSetPageRange';
=======
 }
function PMSetLastPage( printSettings: PMPrintSettings; last: UInt32; lock: Boolean ): OSStatus; external name '_PMSetLastPage';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrintSettingsSetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsSetJobName( printSettings: PMPrintSettings; name: CFStringRef ): OSStatus; external name '_PMPrintSettingsSetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrintSettingsSetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings in which to set the value
 *    
 *    key:
 *      The key to store the value in
 *    
 *    value:
 *      The value to store in the key. If NULL, any existing setting item with
 *      the specified key is removed.
 *    
 *    locked:
 *      A boolean value indicating whether the item being set should be
 *      locked. It is strongly recommended to pass false.
=======
 *  PMSetPageRange()
 *  
 *  Summary:
 *    Sets the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The first and last page are immediately clipped to the new range.
 *    You may pass kPMPrintAllPages for the maxPage value to specified
 *    that all pages are available for printing.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPageRange( printSettings: PMPrintSettings; minPage: UInt32; maxPage: UInt32 ): OSStatus; external name '_PMSetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsSetJobName()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsSetValue( printSettings: PMPrintSettings; key: CFStringRef; value: CFTypeRef; locked: Boolean ): OSStatus; external name '_PMPrintSettingsSetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
  
  
{
 *  PMPrintSettingsCopyAsDictionary()
=======
 }
function PMPrintSettingsSetJobName( printSettings: PMPrintSettings; name: CFStringRef ): OSStatus; external name '_PMPrintSettingsSetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrintSettingsSetValue()
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printSettings:
<<<<<<< HEAD
 *      Represent these print settings as a dictionary.
 *    
 *    settingsDictionary:
 *      On exit, if successful *'settingsDictionary' will contain a reference to
 *      a CFDictionary describing the print settings. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'settingsDictionary' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsCopyAsDictionary( printSettings: PMPrintSettings; var settingsDictionary: CFDictionaryRef ): OSStatus; external name '_PMPrintSettingsCopyAsDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrintSettingsCopyKeys()
=======
 *      The printsettings in which to set the value
 *    
 *    key:
 *      The key to store the value in
 *    
 *    value:
 *      The value to store in the key. If NULL, any existing setting item with
 *      the specified key is removed.
 *    
 *    locked:
 *      A boolean value indicating whether the item being set should be
 *      locked. It is strongly recommended to pass false.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetValue( printSettings: PMPrintSettings; key: CFStringRef; value: CFTypeRef; locked: Boolean ): OSStatus; external name '_PMPrintSettingsSetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
  
  
{
 *  PMPrintSettingsCopyAsDictionary()
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printSettings:
<<<<<<< HEAD
 *      Return the keys for items in this print settings.
 *    
 *    settingsKeys:
 *      On exit, if successful *'settingsKeys' will contain a reference to
 *      a CFArray describing the item keys in the print settings. Each of these
 *		keys may be passed to PMPrintSettingsGetValue() to obtain a value.
 *		The caller is responsible for releasing this reference. If this call returns
 *      an error, then *'settingsKeys' will be set to NULL.
=======
 *      Represent these print settings as a dictionary.
 *    
 *    settingsDictionary:
 *      On exit, if successful *'settingsDictionary' will contain a reference to
 *      a CFDictionary describing the print settings. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'settingsDictionary' will be set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsCopyKeys( printSettings: PMPrintSettings; var settingsKeys: CFArrayRef ): OSStatus; external name '_PMPrintSettingsCopyKeys';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark -
#pragma mark Printer
#pragma mark
*)

{!
 *
 * PMCreateGenericPrinter
 *  Summary:
 *	Creates a generic PMPrinter
 *
 *  Parameters:
 *		printer:
 *		On return *printer contains the generic priner
 *
=======
 }
function PMPrintSettingsCopyAsDictionary( printSettings: PMPrintSettings; var settingsDictionary: CFDictionaryRef ): OSStatus; external name '_PMPrintSettingsCopyAsDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrintSettingsCopyKeys()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Return the keys for items in this print settings.
 *    
 *    settingsKeys:
 *      On exit, if successful *'settingsKeys' will contain a reference to
 *      a CFArray describing the item keys in the print settings. Each of these
 *		keys may be passed to PMPrintSettingsGetValue() to obtain a value.
 *		The caller is responsible for releasing this reference. If this call returns
 *      an error, then *'settingsKeys' will be set to NULL.
 *  
>>>>>>> graemeg/cpstrnew
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMCreateGenericPrinter( var printer: PMPrinter ): OSStatus; external name '_PMCreateGenericPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMServerCreatePrinterList()
 *  
 *  Summary:
 *    Hand back an array of PMPrinter objects that represents the list of printers
 *    known to the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server for which you want to obtain the list of printers.
 *      Use kPMServerLocal for the local print server.
 *    
 *    printerList:
 *      On return, *printerList contains the array of PMPrinter objects returned. The
 *	caller is responsible for releasing this array. On error, *printerList is NULL.
 *    
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMServerCreatePrinterList( server: PMServer; var printerList: CFArrayRef ): OSStatus; external name '_PMServerCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMServerLaunchPrinterBrowser()
 *  
 *  Summary:
 *    Launch the printer browser, browsing the printers available for
 *    the specified print server.
=======
 }
function PMPrintSettingsCopyKeys( printSettings: PMPrintSettings; var settingsKeys: CFArrayRef ): OSStatus; external name '_PMPrintSettingsCopyKeys';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark -
#pragma mark Printer
#pragma mark
*)

{!
 *
 * PMCreateGenericPrinter
 *  Summary:
 *	Creates a generic PMPrinter
 *
 *  Parameters:
 *		printer:
 *		On return *printer contains the generic priner
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMCreateGenericPrinter( var printer: PMPrinter ): OSStatus; external name '_PMCreateGenericPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMServerCreatePrinterList()
 *  
 *  Summary:
 *    Hand back an array of PMPrinter objects that represents the list of printers
 *    known to the specified print server.
>>>>>>> graemeg/cpstrnew
 *
 *  Parameters:
 *    
 *    server:
<<<<<<< HEAD
 *      The print server whose available printers the browser should browse.
 *      Pass kPMServerLocal for the local print server.
 *    
 *    options:
 *      A CFDictionaryRef specifying how the browser should be presented. 
 *      Passing NULL presents the browser in the default fashion. 
 *
 *  Discussion:	The local print server allows its printers to be browsed but
 *		not all print servers do. Passing in a server whose printers cannot be
 *      browsed returns the error code kPMInvalidParameter.
 *    
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMServerLaunchPrinterBrowser( server: PMServer; options: CFDictionaryRef ): OSStatus; external name '_PMServerLaunchPrinterBrowser';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterCreateFromPrinterID()
 *  
 *  Summary:
 *    Create a PMPrinter instance from the supplied printerID.
 *  
 *  Discussion:
 *    Returns a PMPrinter instance for the printer whose ID is
 *    printerID. The caller is responsible for releasing this instance
 *    with PMRelease. The PMPrinter instance returned will be NULL if
 *    there is no printer available which corresponds to the supplied
 *    printerID.
 *  
 *  Parameters:
 *    
 *    printerID:
 *      The printerID for the printer for which you want the PMPrinter.
=======
 *      The print server for which you want to obtain the list of printers.
 *      Use kPMServerLocal for the local print server.
 *    
 *    printerList:
 *      On return, *printerList contains the array of PMPrinter objects returned. The
 *	caller is responsible for releasing this array. On error, *printerList is NULL.
 *    
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMServerCreatePrinterList( server: PMServer; var printerList: CFArrayRef ): OSStatus; external name '_PMServerCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMServerLaunchPrinterBrowser()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Launch the printer browser, browsing the printers available for
 *    the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server whose available printers the browser should browse.
 *      Pass kPMServerLocal for the local print server.
 *    
 *    options:
 *      A CFDictionaryRef specifying how the browser should be presented. 
 *      Passing NULL presents the browser in the default fashion. 
 *
 *  Discussion:	The local print server allows its printers to be browsed but
 *		not all print servers do. Passing in a server whose printers cannot be
 *      browsed returns the error code kPMInvalidParameter.
 *    
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterCreateFromPrinterID( printerID: CFStringRef ): PMPrinter; external name '_PMPrinterCreateFromPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyDescriptionURL()
 *  
 *  Summary:
 *    Hand back a URL to the printer's PostScript Printer Description (PPD) file in fileURL.
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMServerLaunchPrinterBrowser( server: PMServer; options: CFDictionaryRef ): OSStatus; external name '_PMServerLaunchPrinterBrowser';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterCreateFromPrinterID()
 *  
 *  Summary:
 *    Create a PMPrinter instance from the supplied printerID.
 *  
 *  Discussion:
 *    Returns a PMPrinter instance for the printer whose ID is
 *    printerID. The caller is responsible for releasing this instance
 *    with PMRelease. The PMPrinter instance returned will be NULL if
 *    there is no printer available which corresponds to the supplied
 *    printerID.
 *  
 *  Parameters:
 *    
 *    printerID:
 *      The printerID for the printer for which you want the PMPrinter.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose PPD will be returned.
 *    
 *    descriptionType:
 *      The type of description desired. Only kPMPPDDescriptionType is currently supported.
 *    
 *    fileURL:
 *      A pointer to storage for a CFURL reference that will be returned. The caller is responsible
 *      for releasing the reference returned. If this call returns an error, then *fileURL will
 *      be set to NULL.
 *
 *  Discussion:
 *    Only descriptionType of kPMPPDDescriptionType is supported, otherwise kPMInvalidParameter
 *    is returned.
 *
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterCopyDescriptionURL( printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef ): OSStatus; external name '_PMPrinterCopyDescriptionURL';
=======
 }
function PMPrinterCreateFromPrinterID( printerID: CFStringRef ): PMPrinter; external name '_PMPrinterCreateFromPrinterID';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterCopyDeviceURI()
 *  
 *  Summary:
 *    Hand back the URI of the printer's device.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose device URI is to be retrieved.
 *    
 *    deviceURI:
 *      On exit, if successful *'deviceURI' will contain a reference to
 *      a CFURL describing the printer's device. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'deviceURI' will be set to NULL.
=======
 *  PMPrinterCopyDescriptionURL()
 *  
 *  Summary:
 *    Hand back a URL to the printer's PostScript Printer Description (PPD) file in fileURL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose PPD will be returned.
 *    
 *    descriptionType:
 *      The type of description desired. Only kPMPPDDescriptionType is currently supported.
 *    
 *    fileURL:
 *      A pointer to storage for a CFURL reference that will be returned. The caller is responsible
 *      for releasing the reference returned. If this call returns an error, then *fileURL will
 *      be set to NULL.
 *
 *  Discussion:
 *    Only descriptionType of kPMPPDDescriptionType is supported, otherwise kPMInvalidParameter
 *    is returned.
 *
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterCopyDeviceURI( printer: PMPrinter; var deviceURI: CFURLRef ): OSStatus; external name '_PMPrinterCopyDeviceURI';
=======
 }
function PMPrinterCopyDescriptionURL( printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef ): OSStatus; external name '_PMPrinterCopyDescriptionURL';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterCopyHostName()
=======
 *  PMPrinterCopyDeviceURI()
 *  
 *  Summary:
 *    Hand back the URI of the printer's device.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose device URI is to be retrieved.
 *    
 *    deviceURI:
 *      On exit, if successful *'deviceURI' will contain a reference to
 *      a CFURL describing the printer's device. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'deviceURI' will be set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Hand back the host name of the print server hosting the printer's print queue.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterCopyHostName( printer: PMPrinter; var hostNameP: CFStringRef ): OSStatus; external name '_PMPrinterCopyHostName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterCopyPresets()
 *  
 *  Summary:
 *    Provides a list of print settings presets for the specified
 *    printer.
 *  
 *  Discussion:
 *    A printer may have associated with it a list of preset settings.
 *    Each setting is optimized for a particular printing situation.
 *    This function returns all of the presets for a given printer. To
 *    obtain more information about a particular preset see
 *    PMPresetGetAttributes().
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the presets for this printer.
 *    
 *    presetList:
 *      On exit, *'presetList' is set to reference an array of presets.
 *      The caller must call CFRelease when it no longer needs the
 *      array. Each element of the array is a PMPreset. If this
 *      function fails, returning a non-zero error code, then
 *      *'presetList' will be set to NULL.
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyDeviceURI( printer: PMPrinter; var deviceURI: CFURLRef ): OSStatus; external name '_PMPrinterCopyDeviceURI';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyHostName()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Hand back the host name of the print server hosting the printer's print queue.
 *
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterCopyPresets( printer: PMPrinter; var presetList: CFArrayRef ): OSStatus; external name '_PMPrinterCopyPresets';
=======
 }
function PMPrinterCopyHostName( printer: PMPrinter; var hostNameP: CFStringRef ): OSStatus; external name '_PMPrinterCopyHostName';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetCommInfo()
 *  
 *  Summary:
 *    Provides information about the comm channel characteristics for
 *    the printer.
 *  
 *  Discussion:
 *    This function is typically relevant only to PostScript capable
 *    printers. All PostScript printers, regardless of what
 *    communications channel is used to send data to them, support data
 *    in the range 0x20 - 0x7F. Many comm channels can support data
 *    outside this range. The Boolean returned in *supportsTransparentP
 *    indicates whether the comm channel to this printer supports bytes
 *    in the range 0x0 to 0x1F. The Boolean returned in
 *    *supportsEightBitP indicates whether the comm channel to this
 *    printer supports bytes with the high bit set, i.e. bytes in the
 *    range 0x80 - 0xFF.
=======
 *  PMPrinterCopyPresets()
 *  
 *  Summary:
 *    Provides a list of print settings presets for the specified
 *    printer.
 *  
 *  Discussion:
 *    A printer may have associated with it a list of preset settings.
 *    Each setting is optimized for a particular printing situation.
 *    This function returns all of the presets for a given printer. To
 *    obtain more information about a particular preset see
 *    PMPresetGetAttributes().
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
<<<<<<< HEAD
 *      Obtain the comm information for this printer.
 *    
 *    supportsControlCharRangeP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range
 *      0x0 - 0x1F
 *    
 *    supportsEightBitP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range 
 *      0x80 - 0xFF
=======
 *      Obtain the presets for this printer.
 *    
 *    presetList:
 *      On exit, *'presetList' is set to reference an array of presets.
 *      The caller must call CFRelease when it no longer needs the
 *      array. Each element of the array is a PMPreset. If this
 *      function fails, returning a non-zero error code, then
 *      *'presetList' will be set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetCommInfo( printer: PMPrinter; var supportsControlCharRangeP: Boolean; var supportsEightBitP: Boolean ): OSStatus; external name '_PMPrinterGetCommInfo';
=======
 }
function PMPrinterCopyPresets( printer: PMPrinter; var presetList: CFArrayRef ): OSStatus; external name '_PMPrinterCopyPresets';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetID()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetID( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetLocation()
=======
 *  PMPrinterGetCommInfo()
 *  
 *  Summary:
 *    Provides information about the comm channel characteristics for
 *    the printer.
 *  
 *  Discussion:
 *    This function is typically relevant only to PostScript capable
 *    printers. All PostScript printers, regardless of what
 *    communications channel is used to send data to them, support data
 *    in the range 0x20 - 0x7F. Many comm channels can support data
 *    outside this range. The Boolean returned in *supportsTransparentP
 *    indicates whether the comm channel to this printer supports bytes
 *    in the range 0x0 to 0x1F. The Boolean returned in
 *    *supportsEightBitP indicates whether the comm channel to this
 *    printer supports bytes with the high bit set, i.e. bytes in the
 *    range 0x80 - 0xFF.
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the comm information for this printer.
 *    
 *    supportsControlCharRangeP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range
 *      0x0 - 0x1F
 *    
 *    supportsEightBitP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range 
 *      0x80 - 0xFF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetCommInfo( printer: PMPrinter; var supportsControlCharRangeP: Boolean; var supportsEightBitP: Boolean ): OSStatus; external name '_PMPrinterGetCommInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetID()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetLocation( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetLocation';
=======
 }
function PMPrinterGetID( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetID';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetDriverCreator()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetDriverCreator( printer: PMPrinter; var creator: OSType ): OSStatus; external name '_PMPrinterGetDriverCreator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetDriverReleaseInfo()
=======
 *  PMPrinterGetLocation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLocation( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetDriverCreator()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetDriverReleaseInfo( printer: PMPrinter; var release: VersRec ): OSStatus; external name '_PMPrinterGetDriverReleaseInfo';
=======
 }
function PMPrinterGetDriverCreator( printer: PMPrinter; var creator: OSType ): OSStatus; external name '_PMPrinterGetDriverCreator';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetPrinterResolutionCount()
 *  
 *  Summary:
 *    Provides the number of available hardware resolution settings a printer supports. 
=======
 *  PMPrinterGetDriverReleaseInfo()
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the resolution count for this printer.
 *    
 *    countP:
 *      Storage for the returned count indicating the number of available hardware
 *      resolution settings for the specified printer.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetPrinterResolutionCount( printer: PMPrinter; var countP: UInt32 ): OSStatus; external name '_PMPrinterGetPrinterResolutionCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrinterGetIndexedPrinterResolution()
 *  
 *  Summary:
 *    Together with PMPrinterGetPrinterResolutionCount, allows iterating over the
 *    hardware resolution settings a printer supports. 
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer of interest.
 *    
 *    index:
 *      The index of the resolution to return. The minimum value for index is 1 and
 *      the maximum value of index is the count returned by PMPrinterGetPrinterResolutionCount.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetIndexedPrinterResolution( printer: PMPrinter; index: UInt32; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetIndexedPrinterResolution';
=======
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDriverReleaseInfo( printer: PMPrinter; var release: VersRec ): OSStatus; external name '_PMPrinterGetDriverReleaseInfo';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetOutputResolution()
 *  
 *  Summary:
 *    Obtain the printer hardware output resolution for the supplied
 *    printer and print settings. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterGetOutputResolution returns the best guess as to what printer
 *    resolution setting will be used for the destination print job. If the
 *    resolution setting cannot be reliably determined this function returns
 *    kPMKeyNotFound. 
 *  
 *    Most applications do not need to use this function since they draw the same
 *    content regardless of the destination device. For those few applications that
 *    do adjust their drawing based on the output device, they should only do so
 *    when the print job destination is kPMDestinationPrinter or kPMDestinationFax.
 *  
 *    This call should be used after displaying the print dialog to the user so
 *    that it correctly reflects settings changes performed prior to printing.
=======
 *  PMPrinterGetPrinterResolutionCount()
 *  
 *  Summary:
 *    Provides the number of available hardware resolution settings a printer supports. 
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
<<<<<<< HEAD
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings from which to obtain the printer hardware output resolution.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterSetOutputResolution()
 *  
 *  Summary:
 *    Set the printer hardware output resolution in the print settings
 *    for the supplied printer. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterSetOutputResolution configures the print settings to the closest
 *    resolution setting that can be used for the destination print job. Note
 *    that not all printers allow control of their resolution setting.
 *
 *    This function is rarely used since most applications do not set the output 
 *    resolution but instead use the setting supplied by the user in the print dialog.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings in which to set the hardware resolution.
 *    
 *    resolutionP:
 *      A pointer to the PMResolution to use to set the hardware output resolution.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterSetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; const (*var*) resolutionP: PMResolution ): OSStatus; external name '_PMPrinterSetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrinterGetLanguageInfo()
=======
 *      Obtain the resolution count for this printer.
 *    
 *    countP:
 *      Storage for the returned count indicating the number of available hardware
 *      resolution settings for the specified printer.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetPrinterResolutionCount( printer: PMPrinter; var countP: UInt32 ): OSStatus; external name '_PMPrinterGetPrinterResolutionCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrinterGetIndexedPrinterResolution()
 *  
 *  Summary:
 *    Together with PMPrinterGetPrinterResolutionCount, allows iterating over the
 *    hardware resolution settings a printer supports. 
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer of interest.
 *    
 *    index:
 *      The index of the resolution to return. The minimum value for index is 1 and
 *      the maximum value of index is the count returned by PMPrinterGetPrinterResolutionCount.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetIndexedPrinterResolution( printer: PMPrinter; index: UInt32; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetIndexedPrinterResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetOutputResolution()
 *  
 *  Summary:
 *    Obtain the printer hardware output resolution for the supplied
 *    printer and print settings. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterGetOutputResolution returns the best guess as to what printer
 *    resolution setting will be used for the destination print job. If the
 *    resolution setting cannot be reliably determined this function returns
 *    kPMKeyNotFound. 
 *  
 *    Most applications do not need to use this function since they draw the same
 *    content regardless of the destination device. For those few applications that
 *    do adjust their drawing based on the output device, they should only do so
 *    when the print job destination is kPMDestinationPrinter or kPMDestinationFax.
 *  
 *    This call should be used after displaying the print dialog to the user so
 *    that it correctly reflects settings changes performed prior to printing.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings from which to obtain the printer hardware output resolution.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetLanguageInfo( printer: PMPrinter; var info: PMLanguageInfo ): OSStatus; external name '_PMPrinterGetLanguageInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetMakeAndModelName()
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterSetOutputResolution()
 *  
 *  Summary:
 *    Set the printer hardware output resolution in the print settings
 *    for the supplied printer. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterSetOutputResolution configures the print settings to the closest
 *    resolution setting that can be used for the destination print job. Note
 *    that not all printers allow control of their resolution setting.
 *
 *    This function is rarely used since most applications do not set the output 
 *    resolution but instead use the setting supplied by the user in the print dialog.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings in which to set the hardware resolution.
 *    
 *    resolutionP:
 *      A pointer to the PMResolution to use to set the hardware output resolution.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetMakeAndModelName( printer: PMPrinter; var makeAndModel: CFStringRef ): OSStatus; external name '_PMPrinterGetMakeAndModelName';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetMimeTypes()
 *  
 *  Summary:
 *    Return the array of mime type supported by the printer for a
 *    given set of print settings.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job. The part of the print
 *      settings that effects the available mime type is the
 *      destination. This parameter can be NULL.
 *    
 *    mimeTypes:
 *      If this function returns without error then *'mimeTypes' is
 *      filled in with a reference to an array of CFStrings. Each
 *      CFString names a mime type supported by the printer with the
 *      specified print settings. The caller must not release this
 *      reference without first doing a retain. If this function
 *      returns an error then 'mimeTypes' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetMimeTypes( printer: PMPrinter; settings: PMPrintSettings; var mimeTypes: CFArrayRef ): OSStatus; external name '_PMPrinterGetMimeTypes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetName( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetName';
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterSetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; const (*var*) resolutionP: PMResolution ): OSStatus; external name '_PMPrinterSetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrinterGetLanguageInfo()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLanguageInfo( printer: PMPrinter; var info: PMLanguageInfo ): OSStatus; external name '_PMPrinterGetLanguageInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetMakeAndModelName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetMakeAndModelName( printer: PMPrinter; var makeAndModel: CFStringRef ): OSStatus; external name '_PMPrinterGetMakeAndModelName';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetPaperList()
 *  
 *  Summary:
 *    Returns the list of papers available for a given printer.
=======
 *  PMPrinterGetMimeTypes()
 *  
 *  Summary:
 *    Return the array of mime type supported by the printer for a
 *    given set of print settings.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
<<<<<<< HEAD
 *      Obtain the paper list for this printer.
 *    
 *    paperList:
 *      If successful noErr is returned and *paperList is a CFArray of
 *      PMPapers representing the list of papers available for the
 *      printer.
=======
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job. The part of the print
 *      settings that effects the available mime type is the
 *      destination. This parameter can be NULL.
 *    
 *    mimeTypes:
 *      If this function returns without error then *'mimeTypes' is
 *      filled in with a reference to an array of CFStrings. Each
 *      CFString names a mime type supported by the printer with the
 *      specified print settings. The caller must not release this
 *      reference without first doing a retain. If this function
 *      returns an error then 'mimeTypes' will be set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetPaperList( printer: PMPrinter; var paperList: CFArrayRef ): OSStatus; external name '_PMPrinterGetPaperList';
=======
 }
function PMPrinterGetMimeTypes( printer: PMPrinter; settings: PMPrintSettings; var mimeTypes: CFArrayRef ): OSStatus; external name '_PMPrinterGetMimeTypes';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetState()
=======
 *  PMPrinterGetName()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterGetState( printer: PMPrinter; var state: PMPrinterState ): OSStatus; external name '_PMPrinterGetState';
=======
 }
function PMPrinterGetName( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetName';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterIsDefault()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterIsDefault( printer: PMPrinter ): Boolean; external name '_PMPrinterIsDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsFavorite()
 *  
 *  Summary:
 *    Return true if the printer is in the user's favorite printer list.
=======
 *  PMPrinterGetPaperList()
 *  
 *  Summary:
 *    Returns the list of papers available for a given printer.
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the paper list for this printer.
 *    
 *    paperList:
 *      If successful noErr is returned and *paperList is a CFArray of
 *      PMPapers representing the list of papers available for the
 *      printer.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetPaperList( printer: PMPrinter; var paperList: CFArrayRef ): OSStatus; external name '_PMPrinterGetPaperList';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetState()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterIsFavorite( printer: PMPrinter ): Boolean; external name '_PMPrinterIsFavorite';
=======
 }
function PMPrinterGetState( printer: PMPrinter; var state: PMPrinterState ): OSStatus; external name '_PMPrinterGetState';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterIsPostScriptCapable()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterIsPostScriptCapable( printer: PMPrinter ): Boolean; external name '_PMPrinterIsPostScriptCapable';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{!
 * @function	PMPrinterIsPostScriptPrinter
 * @abstract	Set *isPSPrinter true if the printer is a PostScript printer.
 *
 * @discussion	A PostScript printer is one whose driver takes PostScript directly.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterIsPostScriptPrinter( printer: PMPrinter; var isPSPrinter: Boolean ): OSStatus; external name '_PMPrinterIsPostScriptPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrinterIsRemote()
 *  
 *  Summary:
 *    Hand back a boolean indicating whether the printer is hosted by remote print server.
 *  
 *  Discussion:
 *    If on return *isRemoteP is true, the print queue represents a printer hosted and
 *    managed by a remote print server.
 *
 *    If on return *isRemoteP is false, the print queue represents a directly connected
 *    printer, a network printer, or a remote printer that is locally managed. Consult
 *    the queue's device URI to determine the type of connection that is used to communicate
 *    with the printer.
 *
 *    Whether a printer is remote is derived from the CUPS printer-type attribute for the print queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterIsRemote( printer: PMPrinter; var isRemoteP: Boolean ): OSStatus; external name '_PMPrinterIsRemote';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterSetDefault()
 *  
 *  Summary:
 *    Set the default printer for the current user.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer to set as the default printer.
 *  
 *  Special considerations:
 *      It is not typical for an application to set the current default printer
 *      for the user; the printing system itself takes care of managing the default printer.
 *      This function should be used only in rare circumstances.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterSetDefault( printer: PMPrinter ): OSStatus; external name '_PMPrinterSetDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
 *  PMPrinterIsDefault()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsDefault( printer: PMPrinter ): Boolean; external name '_PMPrinterIsDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsFavorite()
 *  
 *  Summary:
 *    Return true if the printer is in the user's favorite printer list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsFavorite( printer: PMPrinter ): Boolean; external name '_PMPrinterIsFavorite';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsPostScriptCapable()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsPostScriptCapable( printer: PMPrinter ): Boolean; external name '_PMPrinterIsPostScriptCapable';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{!
 * @function	PMPrinterIsPostScriptPrinter
 * @abstract	Set *isPSPrinter true if the printer is a PostScript printer.
 *
 * @discussion	A PostScript printer is one whose driver takes PostScript directly.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsPostScriptPrinter( printer: PMPrinter; var isPSPrinter: Boolean ): OSStatus; external name '_PMPrinterIsPostScriptPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew

(*
#pragma mark
#pragma mark Preset
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMPresetCopyName()
 *  
 *  Summary:
 *    Hand back a copy of the localized name for the specified preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      The preset whose name is needed.
 *    
 *    name:
 *      On exit, if this routine succeeds, *'name' is filled in with a
 *      reference to a localized string with the preset's name. If this
 *      routine fails, then *'name' is set to NULL.
=======
 *  PMPrinterIsRemote()
 *  
 *  Summary:
 *    Hand back a boolean indicating whether the printer is hosted by remote print server.
 *  
 *  Discussion:
 *    If on return *isRemoteP is true, the print queue represents a printer hosted and
 *    managed by a remote print server.
 *
 *    If on return *isRemoteP is false, the print queue represents a directly connected
 *    printer, a network printer, or a remote printer that is locally managed. Consult
 *    the queue's device URI to determine the type of connection that is used to communicate
 *    with the printer.
 *
 *    Whether a printer is remote is derived from the CUPS printer-type attribute for the print queue.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPresetCopyName( preset: PMPreset; var name: CFStringRef ): OSStatus; external name '_PMPresetCopyName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPresetCreatePrintSettings()
 *  
 *  Summary:
 *    Create a print settings conforming to the specified print
 *    settings preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A preset specifying a set of initial print settings.
 *    
 *    session:
 *      A valid print session.
 *    
 *    printSettings:
 *      On exit, *'printSettings' is set to a newly created print
 *      settings that contains the settings specified by 'preset'. The
 *      caller is responsible for calling PMRelease when the print
 *      settings are no longer needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPresetCreatePrintSettings( preset: PMPreset; session: PMPrintSession; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPresetCreatePrintSettings';
=======
 }
function PMPrinterIsRemote( printer: PMPrinter; var isRemoteP: Boolean ): OSStatus; external name '_PMPrinterIsRemote';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPresetGetAttributes()
 *  
 *  Summary:
 *    Hand back the meta-data describing a given preset.
 *  
 *  Discussion:
 *    Each preset has associated with it a dictionary containing
 *    meta-data. The meta-data provides the preset's id, the preset's
 *    localized names, and descriptions of the environment for which
 *    the preset it intended.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A print settings preset as obtained from PMPrinterCopyPresets().
 *    
 *    attributes:
 *      On exit, *'attributes' is set to reference a dictionary
 *      containing the preset's meta-data. The caller is responsible
 *      for retaining this reference if it is to be used beyond the
 *      lifetime of 'preset'. If this function fails, returning a
 *      non-zero error code, then *'attributes' is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPresetGetAttributes( preset: PMPreset; var attributes: CFDictionaryRef ): OSStatus; external name '_PMPresetGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Paper
#pragma mark
*)
{
 *  PMGetPageFormatPaper()
 *  
 *  Summary:
 *    Returns the paper associated with a pageformat.
 *  
 *  Parameters:
 *    
 *    format:
 *      Obtain the paper for this pageformat.
 *    
 *    paper:
 *      If successful noErr is returned and *paper will contain a
 *      PMPaper object describing the current paper associated with the
 *      pageformat.
=======
 *  PMPrinterSetDefault()
 *  
 *  Summary:
 *    Set the default printer for the current user.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer to set as the default printer.
 *  
 *  Special considerations:
 *      It is not typical for an application to set the current default printer
 *      for the user; the printing system itself takes care of managing the default printer.
 *      This function should be used only in rare circumstances.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterSetDefault( printer: PMPrinter ): OSStatus; external name '_PMPrinterSetDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark Preset
#pragma mark
*)
{
 *  PMPresetCopyName()
 *  
 *  Summary:
 *    Hand back a copy of the localized name for the specified preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      The preset whose name is needed.
 *    
 *    name:
 *      On exit, if this routine succeeds, *'name' is filled in with a
 *      reference to a localized string with the preset's name. If this
 *      routine fails, then *'name' is set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMGetPageFormatPaper( format: PMPageFormat; var paper: PMPaper ): OSStatus; external name '_PMGetPageFormatPaper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperCreateCustom()
 *  
 *  Summary:
 *    Create a new custom paper instance.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The new paper size is appropriate for this printer.
 *    
 *    id:
 *      A unique identifier for this paper type.
 *    
 *    name:
 *      The name to display to the user for this paper type.
 *    
 *    width:
 *      The width, in points, of the paper.
 *    
 *    height:
 *      The height, in points, of the paper.
 *    
 *    margins:
 *      The unprintable margins on the paper.
 *    
 *    paperP:
 *      if this function is successful, returning noErr, then *'paperP'
 *      is set to be a reference to a newly created PMPaper instance.
 *      The caller is responsible for calling PMRelease when the
 *      instance is no longer needed. If this functions fails, it will
 *      return a non-zero error and set *'paperP' to NULL.
 *  
 *  Discussion:
 *		This function creates a new custom paper instance. To obtain one of the available
 *		built-in paper sizes for a given printer, use PMPrinterGetPaperList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperCreateCustom( printer: PMPrinter; id: CFStringRef; name: CFStringRef; width: Float64; height: Float64; const (*var*) margins: PMPaperMargins; var paperP: PMPaper ): OSStatus; external name '_PMPaperCreateCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetWidth()
 *  
 *  Summary:
 *    Returns the width for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the width for this paper.
 *    
 *    paperWidth:
 *      If successful noErr is returned and *paperWidth is set to the
 *      width of the paper.
=======
 }
function PMPresetCopyName( preset: PMPreset; var name: CFStringRef ): OSStatus; external name '_PMPresetCopyName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPresetCreatePrintSettings()
 *  
 *  Summary:
 *    Create a print settings conforming to the specified print
 *    settings preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A preset specifying a set of initial print settings.
 *    
 *    session:
 *      A valid print session.
 *    
 *    printSettings:
 *      On exit, *'printSettings' is set to a newly created print
 *      settings that contains the settings specified by 'preset'. The
 *      caller is responsible for calling PMRelease when the print
 *      settings are no longer needed.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperGetWidth( paper: PMPaper; var paperWidth: Float64 ): OSStatus; external name '_PMPaperGetWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCopyOutputFormatList( printSession: PMPrintSession; destType: PMDestinationType; var documentFormatP: CFArrayRef ): OSStatus; external name '_PMSessionCopyOutputFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
 *  PMPaperGetHeight()
 *  
 *  Summary:
 *    Returns the height for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the height for this paper.
 *    
 *    paperHeight:
 *      If successful noErr is returned and *paperHeight is set to the
 *      height of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperGetHeight( paper: PMPaper; var paperHeight: Float64 ): OSStatus; external name '_PMPaperGetHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetMargins()
 *  
 *  Summary:
 *    Returns the margins for a given paper.
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    paper:
 *      Obtain the margin information for this paper.
 *    
 *    paperMargins:
 *      If successful noErr is returned and *paperMargins is set to the
 *      margins of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperGetMargins( paper: PMPaper; var paperMargins: PMPaperMargins ): OSStatus; external name '_PMPaperGetMargins';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperGetID()
 *  
 *  Summary:
 *    Returns the id for a given paper.
 *  
 *  Parameters:
=======
 *    printSession:
 *      A currently valid print session.
 *    
 *    printer:
 *      The printer whose page size list should be enumerated. 
>>>>>>> graemeg/cpstrnew
 *    
 *    paper:
 *      Obtain the id for this paper.
 *    
<<<<<<< HEAD
 *    paperID:
 *      If successful noErr is returned and *paperID is set to the id
 *      of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperGetID( paper: PMPaper; var paperID: CFStringRef ): OSStatus; external name '_PMPaperGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetPPDPaperName()
 *  
 *  Summary:
 *    Returns the PPD name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the PPD name for this paper.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      PPD name of the paper. The returned result may be NULL.
 *  
 *  Discussion:
 *      The Mac OS X printing system uses a PostScript Printer Description (PPD)
 *      file to describe a given printer and print queue for that printer. The
 *      PPD name is the name that uniquely identifies a given paper
 *      to the printer to which the paper corresponds. 
 *    
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperGetPPDPaperName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetPPDPaperName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPaperCreateLocalizedName()
 *  
 *  Summary:
 *    Returns the localized name for a given paper. 
 *    Use of PMPaperCreateLocalizedName
 *    is recommended instead of PMPaperGetName.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the localized name for this paper.
 *    
 *    printer:
 *      The printer for which the localization should be performed.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      localized name of the paper. The returned paper name is that appropriate
 *      to display to a user. The returned result may be NULL.
 *      If non-NULL, the caller is responsible for releasing the name returned. 
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperCreateLocalizedName( paper: PMPaper; printer: PMPrinter; var paperName: CFStringRef ): OSStatus; external name '_PMPaperCreateLocalizedName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetPrinterID()
 *  
 *  Summary:
 *    Returns the printerID of the printer for which the paper corresponds.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the printer ID for this paper.
 *    
 *    printerID:
 *      If successful, noErr is returned and *printerID is set to the
 *      ID of the printer for which the paper corresponds. The returned
 *      result may be NULL. 
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperGetPrinterID( paper: PMPaper; var printerID: CFStringRef ): OSStatus; external name '_PMPaperGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperIsCustom()
 *  
 *  Summary:
 *    Returns true if the paper is a custom paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Determines if this is a custom paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperIsCustom( paper: PMPaper ): Boolean; external name '_PMPaperIsCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
 *  Discussion:
 *	If you want to create the page format list for the session's current printer,
 *      pass the PMPrinter object returned by PMSessionGetCurrentPrinter() as the
 *	printer parameter. 
 *
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later. On Mac OS 8/9 using CarbonLib, this
 *      routine returns kPMNotImplemented
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCreatePageFormatList( printSession: PMPrintSession; printer: PMPrinter; var pageFormatList: CFArrayRef ): OSStatus; external name '_PMSessionCreatePageFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionCreatePrinterList()
 *
 *  See also: PMServerCreatePrinterList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCreatePrinterList( printSession: PMPrintSession; var printerList: CFArrayRef; var currentIndex: CFIndex; var currentPrinter: PMPrinter ): OSStatus; external name '_PMSessionCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  PMSessionGetCurrentPrinter()
 *  
 *  Summary:
 *    Hand back the session's current printer.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetCurrentPrinter( printSession: PMPrintSession; var currentPrinter: PMPrinter ): OSStatus; external name '_PMSessionGetCurrentPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

(*
#pragma mark
#pragma mark PDF Workflow
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMWorkflowCopyItems()
 *  
 *  Summary:
 *    Hand back an array of dictionaries describing the PDF Workflow
 *    items installed on the system
 *  
 *  Parameters:
 *    
 *    workflowItems:
 *      If this function returns without error then *'workflowItems'
 *      will be filled in with a reference to an array. It is the
 *      caller's responsability to release the array when done with it.
 *      Each element in the array describes a PDF Workflow item or a
 *      folder holding workflow items. A dictionary describing a
 *      workflow item has, at least, the following keys and values:
 *      displayName - The user's diaplayable name for the workflow item
 *      itemURL - A CFURLRef pointing to the workflow item. A
 *      dictionary describing a workflow folder has at least the
 *      following keys: displayName - The user's diaplayable name for
 *      the workflow item folderURL - A CFURLRef pointing to the
 *      folder. items - A CFArrayRef describing the workflow items in
 *      the folder. If this function returns a non-zero error code then
 *      *'workflowItems' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMWorkflowCopyItems( var workflowItems: CFArrayRef ): OSStatus; external name '_PMWorkflowCopyItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithOptions()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    title:
 *      The user displayable name of the document.
 *    
 *    options:
 *      A string of CUPS style key-value pairs that may be passed to
 *      the PDF Workflow item. This parameter can be NULL in which case
 *      an empty string of options is used.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
=======
 *  PMSessionSetCurrentPMPrinter()
 *  
 *  Summary:
 *    Set the session's current printer to a specified PMPrinter.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMWorkflowSubmitPDFWithOptions( workflowItem: CFURLRef; title: CFStringRef; options: ConstCStringPtr; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithSettings()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    settings:
 *      The prints settings to apply to the PDF.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMWorkflowSubmitPDFWithSettings( workflowItem: CFURLRef; settings: PMPrintSettings; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Job Submission
#pragma mark
*)
{
 *  PMPrinterPrintWithProvider()
 *  
 *  Summary:
 *    Submit print data to a specified printer.
 *  
 *  Discussion:
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. This parameter can not
 *      be NULL. Use PMPrinterPrintWithFile() if auto-typing is desired.
 *    
 *    provider:
 *      The data provider that supplies the print data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterPrintWithProvider( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; provider: CGDataProviderRef ): OSStatus; external name '_PMPrinterPrintWithProvider';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterPrintWithFile()
 *  
 *  Summary:
 *    Submit a file for printing to a specified printer.
 *  
 *  Discussion:
 *
 *    One reason this function may fail is if the specified printer can
 *    not handle the file's mime type. Use PMPrinterGetMimeTypes() to
 *    check whether a mime type is supported.
 *  
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. If this parameter is
 *      NULL then the supplied file will be auto-typed.
 *    
 *    fileURL:
 *      A file URL specifying the file to be printed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterPrintWithFile( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; fileURL: CFURLRef ): OSStatus; external name '_PMPrinterPrintWithFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPrinterWritePostScriptToURL()
 *
 *  Summary:
 *    Convert an input file of the specified mimeType to printer ready PostScript for the destination printer.
 *
 *  Discussion:
 *
 *    The conversion of the input file to PostScript is performed before this function returns. This can take a significant
 *    amount of time for longer documents. The caller of PMPrinterWritePostScriptToURL may want to perform this operation
 *    on a thread other than the main application thread or fork a separate process for this purpose.
 *
 *    One reason this function may fail is if conversion from the input mimeType to PostScript cannot be performed.
 *
 *
 *  Parameters:
 *
 *    printer:
 *      The printer for which the printer ready PostScript will be generated.
 *
 *    settings:
 *      The print settings for the print job.
 *
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed.
 *
 *    mimeType:
 *      The mime type of the file to be printed. If NULL, the file is auto-typed.
 *
 *    sourceFileURL:
 *      A file URL specifying the input file to be converted to printer ready PostScript data. Only file based URLs
 *      are supported. This file is deleted when the conversion is completed.
 *
 *    destinationFileURL:
 *      A file URL specifying the destination file to be created. If the file already exists it will be overwritten. Only
 *      file based URLs are supported.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrinterWritePostScriptToURL( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; sourceFileURL: CFURLRef; destinationFileURL: CFURLRef ): OSStatus; external name '_PMPrinterWritePostScriptToURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsToOptions()
 *  
 *  Summary:
 *    Convert print settings to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'ed C string
 *      describing the passed in print settings. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsToOptions( settings: PMPrintSettings; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrintSettingsToOptionsWithPrinterAndPageFormat()
 *  
 *  Summary:
 *    Convert a print settings and page format to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string. Must not be NULL.
 *    
 *    printer:
 *      The printer to use for converting the print settings. Must not be NULL.
 *
 *    format:
 *      The page format for use in creating the cups options. Can be NULL.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'd C string
 *      describing the passed in print settings and format. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPrintSettingsToOptionsWithPrinterAndPageFormat( settings: PMPrintSettings; printer: PMPrinter; pageFormat: PMPageFormat; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptionsWithPrinterAndPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark PrinterCommandSupport
#pragma mark
*)

{
*  PMPrinterSendCommand()
*
*  Summary:
*    Submit a command file containing the specified string data to a print queue.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    commandString:
*      The contents of the command file as a CFStringRef.
*
*    jobTitle:
*      The title of the job associated with the command file. If NULL,
*      a job title will be automatically generated but it may not be
*      meaningful to a user.
*
*    options:
*      A dictionary containing options that apply to the job.
*      This parameter should be NULL; it is reserved for future expansion.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
}
function PMPrinterSendCommand( printer: PMPrinter; commandString: CFStringRef; jobTitle: CFStringRef; options: CFDictionaryRef ): OSStatus; external name '_PMPrinterSendCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{
*  PMPrinterCopyState()
*
*  Summary:
*    Copies the current "marker-change-time", "marker-colors",
*    "marker-high-levels", "marker-levels", "marker-low-levels",
*    "marker-message", "marker-names", "marker-types", "printer-state",
*    "printer-state-change-time", "printer-state-message", and
*    "printer-state-reasons" values for a print queue.  These attributes are
*    defined in the "CUPS Implementation of IPP" specification at:
*
*        http://www.cups.org/documentation.php/spec-ipp.html
*        http://localhost:631/help/spec-ipp.html
*
*    Each attribute is returned using the corresponding CoreFoundation
*    data type:
*
*        Attribute                    Type
*        ---------------------------  ------------------------
*        "marker-change-time"         CFDateRef
*        "marker-colors"              CFArrayRef + CFStringRef
*        "marker-high-levels"         CFArrayRef + CFNumberRef
*        "marker-levels"              CFArrayRef + CFNumberRef
*        "marker-low-levels"          CFArrayRef + CFNumberRef
*        "marker-message"             CFStringRef
*        "marker-names"               CFArrayRef + CFStringRef
*        "marker-types"               CFArrayRef + CFStringRef
*        "printer-state"              CFNumberRef
*        "printer-state-change-time"  CFDateRef
*        "printer-state-message"      CFStringRef
*        "printer-state-reasons"      CFArrayRef + CFStringRef
*
*  Localization Considerations:
*
*    The "marker-message" and "printer-state-message" attributes may be
*    localized by the printer driver using the language of the last processed
*    print job.
*
*    The "marker-names" and "printer-state-reasons" attributes are typically
*    English-language keywords that may be localized using the printer's PPD
*    file and CUPS ppdLocalizeMarkerName and ppdLocalizeIPPReason APIs,
*    respectively.
*
*    All other attributes are predefined keywords or values that are not
*    localized.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    stateDict:
*      A pointer to a CFDictionaryRef variable to hold the values.
*      You must release the dictionary when you are done using it.
*      The dictionary returned is not mutable.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
*  
*  Compatibility:
*  	Not appropriate for CUPS filters, drivers, and backends.
*  
}

function PMPrinterCopyState( printer: PMPrinter; var stateDict: CFDictionaryRef ): OSStatus; external name '_PMPrinterCopyState';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

(*
#pragma mark
#pragma mark PPD related
#pragma mark
*)
{
 *  PMCopyAvailablePPDs()
 *  
 *  Summary:
 *    Hand back the list of PPDs in the specified PPD domain.
 *  
 *  Parameters:
 *    
 *    domain:
 *      The domain to search for PPDs.
 *    
 *    ppds:
 *      If this function completes without error, *'ppds' is set to an
 *      array of CFURLs. Each CFURL specifies the location of a PPD
 *      file or a compressed PPD file. The caller is responsible for
 *      releasing the array. If this function returns a non-zero error
 *      code then *'ppds' is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMCopyAvailablePPDs( domain: PMPPDDomain; var ppds: CFArrayRef ): OSStatus; external name '_PMCopyAvailablePPDs';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMCopyLocalizedPPD()
 *  
 *  Summary:
 *    Hand back a reference to a localized PPD.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A PPD reference. Typically this is a CFURLRef returned from
 *      PMCopyAvailablePPDs().
 *    
 *    localizedPPD:
 *      If this function completes without error, *'localizedPPD' will
 *      be set to a CFURLRef referencing the PPD that should be used
 *      given the current user's language preferences. If this function
 *      returns an error then *'localizedPPD' will be set to to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMCopyLocalizedPPD( ppd: CFURLRef; var localizedPPD: CFURLRef ): OSStatus; external name '_PMCopyLocalizedPPD';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMCopyPPDData()
 *  
 *  Summary:
 *    Hand back the uncompressed PPD data for a PPD or compressed PPD
 *    file.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A reference to a PPD or compressed PPD file. This reference is
 *      usually obtained from PMCopyAvailablePPDs() or from
 *      PMCopyLocalizedPPD().
 *    
 *    data:
 *      If this function completes without error then *'data' is set to
 *      reference the uncompressed PPD data from the PPD file. If this
 *      function returns a non-zero error then *'data is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMCopyPPDData( ppd: CFURLRef; var data: CFDataRef ): OSStatus; external name '_PMCopyPPDData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
 }
function PMSessionSetCurrentPMPrinter( session: PMPrintSession; printer: PMPrinter ): OSStatus; external name '_PMSessionSetCurrentPMPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMSessionGetDataFromSession()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetDataFromSession( printSession: PMPrintSession; key: CFStringRef; var data: CFTypeRef ): OSStatus; external name '_PMSessionGetDataFromSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionSetDataInSession()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetDataInSession( printSession: PMPrintSession; key: CFStringRef; data: CFTypeRef ): OSStatus; external name '_PMSessionSetDataInSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark -
#pragma mark PageFormat
#pragma mark
*)

{
 *  PMCreatePageFormat()
 *  
=======
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCreatePageFormatList( printSession: PMPrintSession; printer: PMPrinter; var pageFormatList: CFArrayRef ): OSStatus; external name '_PMSessionCreatePageFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionCreatePrinterList()
 *
 *  See also: PMServerCreatePrinterList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCreatePrinterList( printSession: PMPrintSession; var printerList: CFArrayRef; var currentIndex: CFIndex; var currentPrinter: PMPrinter ): OSStatus; external name '_PMSessionCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  PMSessionGetCurrentPrinter()
 *  
 *  Summary:
 *    Hand back the session's current printer.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetCurrentPrinter( printSession: PMPrintSession; var currentPrinter: PMPrinter ): OSStatus; external name '_PMSessionGetCurrentPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionSetCurrentPMPrinter()
 *  
 *  Summary:
 *    Set the session's current printer to a specified PMPrinter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetCurrentPMPrinter( session: PMPrintSession; printer: PMPrinter ): OSStatus; external name '_PMSessionSetCurrentPMPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMSessionGetDataFromSession()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetDataFromSession( printSession: PMPrintSession; key: CFStringRef; var data: CFTypeRef ): OSStatus; external name '_PMSessionGetDataFromSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionSetDataInSession()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetDataInSession( printSession: PMPrintSession; key: CFStringRef; data: CFTypeRef ): OSStatus; external name '_PMSessionSetDataInSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark -
#pragma mark PageFormat
#pragma mark
*)

{
 *  PMCreatePageFormat()
 *  
>>>>>>> origin/cpstrnew
 *  Summary:
 *    Allocates memory for a new PMPageFormat object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A pageformat is created with a refcount of 1.
<<<<<<< HEAD
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePageFormat( var pageFormat: PMPageFormat ): OSStatus; external name '_PMCreatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDefaultPageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionDefaultPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPageFormat( formatSrc: PMPageFormat; formatDest: PMPageFormat ): OSStatus; external name '_PMCopyPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew

(*
#pragma mark
#pragma mark EPS related
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMCGImageCreateWithEPSDataProvider()
 *  
 *  Summary:
 *    Create an image reference that references both the PostScript
 *    contents of an EPS file and a preview (proxy) image for that EPS
 *    file.
 *  
 *  Discussion:
 *    For OS X 10.1.0, this function ignores the passed in data
 *    provider. The passed in image reference is retained and then
 *    returned. For 10.1.1 and later, then the data provider is used
 *    and the returned image reference is different than the passed in
 *    image reference, so please be careful with your use of these
 *    references. It is likely that the data will not be read from the
 *    EPS data provider until well after this function returns. The
 *    caller should be careful not to free the underlying EPS data
 *    until the provider's release routine is invoked. Similarly the
 *    preview image's data may be needed long after you think it should
 *    be. Do not free the image data until the image data provider's
 *    release data function has been called. To make sure these data
 *    providers are properly reference counted, release your reference
 *    the EPS data provider and on the EPS image preview when they are
 *    no longer needed by your application. For Mac OS X 10.2 and
 *    later, the contents of the EPS provider at the time of this call
 *    can be dumped to a file if you first do the following, BEFORE
 *    running your application. 
 *
 *	  From the command line in terminal:
 *		defaults write NSGlobalDomain com.apple.print.eps.testProvider /tmp/dump.eps
 *    
 *	  causes a dump of the EPS data into a file /tmp/dump.eps.
 *  
 *  Parameters:
 *    
 *    epsDataProvider:
 *      A Core Graphics data provider that can supply the PostScript
 *      contents of the EPS file. Post OS X 10.1, there will be some
 *      checking done on the EPS data provided to the
 *      PMCGImageCreateWithEPSDataProvider() call. It is important that
 *      the EPS data begin with the EPSF required header and bounding
 *      box DSC comments.
 *    
 *    epsPreview:
 *      A Core Graphics image reference to the proxy image for the EPS
 *      file. When the image reference result of this function is
 *      rendered on screen or printed to a printer that can not render
 *      PostScript this proxy image is drawn instead.
 *  
 *  Result:
 *    an image reference capable of rendering either the EPS content or
 *    the proxy image depending upon the capabilities of the targeted
 *    context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
=======
 *    Non-Carbon CFM:   not available
 }
function PMPresetCreatePrintSettings( preset: PMPreset; session: PMPrintSession; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPresetCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPresetGetAttributes()
 *  
 *  Summary:
 *    Hand back the meta-data describing a given preset.
 *  
 *  Discussion:
 *    Each preset has associated with it a dictionary containing
 *    meta-data. The meta-data provides the preset's id, the preset's
 *    localized names, and descriptions of the environment for which
 *    the preset it intended.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A print settings preset as obtained from PMPrinterCopyPresets().
 *    
 *    attributes:
 *      On exit, *'attributes' is set to reference a dictionary
 *      containing the preset's meta-data. The caller is responsible
 *      for retaining this reference if it is to be used beyond the
 *      lifetime of 'preset'. If this function fails, returning a
 *      non-zero error code, then *'attributes' is set to NULL.
=======
 *  PMPrinterCopyDeviceURI()
 *  
 *  Summary:
 *    Hand back the URI of the printer's device.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose device URI is to be retrieved.
 *    
 *    deviceURI:
 *      On exit, if successful *'deviceURI' will contain a reference to
 *      a CFURL describing the printer's device. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'deviceURI' will be set to NULL.
=======
 *  PMCreatePageFormatWithPMPaper()
 *  
 *  Summary:
 *    Create a pageformat with a specific paper.
 *  
 *  Parameters:
 *    
 *    pageFormat:
 *      On return, will contain the pageformat which was created
 *    
 *    paper:
 *      The paper that will be associate with the pageformat
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePageFormatWithPMPaper( var pageFormat: PMPageFormat; paper: PMPaper ): OSStatus; external name '_PMCreatePageFormatWithPMPaper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPageFormatCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPageFormat object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPageFormatToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPageFormatCreateWithDataRepresentation to create a PMPageFormat from 
 *    a CFDataRef created by this call.
 *  
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyDeviceURI( printer: PMPrinter; var deviceURI: CFURLRef ): OSStatus; external name '_PMPrinterCopyDeviceURI';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyHostName()
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateDataRepresentation( pageFormat: PMPageFormat; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPageFormatCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPageFormatCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPageFormat object from a data representation created with
 *	  PMPageFormatCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Hand back the host name of the print server hosting the printer's print queue.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPresetGetAttributes( preset: PMPreset; var attributes: CFDictionaryRef ): OSStatus; external name '_PMPresetGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateWithDataRepresentation( data: CFDataRef; var pageFormat: PMPageFormat ): OSStatus; external name '_PMPageFormatCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew

(*
#pragma mark
#pragma mark Paper
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMGetPageFormatPaper()
 *  
 *  Summary:
 *    Returns the paper associated with a pageformat.
 *  
 *  Parameters:
 *    
 *    format:
 *      Obtain the paper for this pageformat.
 *    
 *    paper:
 *      If successful noErr is returned and *paper will contain a
 *      PMPaper object describing the current paper associated with the
 *      pageformat.
=======
function PMPrinterCopyHostName( printer: PMPrinter; var hostNameP: CFStringRef ): OSStatus; external name '_PMPrinterCopyHostName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterCopyPresets()
 *  
 *  Summary:
 *    Provides a list of print settings presets for the specified
 *    printer.
 *  
 *  Discussion:
 *    A printer may have associated with it a list of preset settings.
 *    Each setting is optimized for a particular printing situation.
 *    This function returns all of the presets for a given printer. To
 *    obtain more information about a particular preset see
 *    PMPresetGetAttributes().
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the presets for this printer.
 *    
 *    presetList:
 *      On exit, *'presetList' is set to reference an array of presets.
 *      The caller must call CFRelease when it no longer needs the
 *      array. Each element of the array is a PMPreset. If this
 *      function fails, returning a non-zero error code, then
 *      *'presetList' will be set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMGetPageFormatPaper( format: PMPageFormat; var paper: PMPaper ): OSStatus; external name '_PMGetPageFormatPaper';
=======
function PMPrinterCopyPresets( printer: PMPrinter; var presetList: CFArrayRef ): OSStatus; external name '_PMPrinterCopyPresets';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPaperCreateCustom()
 *  
 *  Summary:
 *    Create a new custom paper instance.
=======
 *  PMPrinterGetCommInfo()
 *  
 *  Summary:
 *    Provides information about the comm channel characteristics for
 *    the printer.
 *  
 *  Discussion:
 *    This function is typically relevant only to PostScript capable
 *    printers. All PostScript printers, regardless of what
 *    communications channel is used to send data to them, support data
 *    in the range 0x20 - 0x7F. Many comm channels can support data
 *    outside this range. The Boolean returned in *supportsTransparentP
 *    indicates whether the comm channel to this printer supports bytes
 *    in the range 0x0 to 0x1F. The Boolean returned in
 *    *supportsEightBitP indicates whether the comm channel to this
 *    printer supports bytes with the high bit set, i.e. bytes in the
 *    range 0x80 - 0xFF.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
<<<<<<< HEAD
 *      The new paper size is appropriate for this printer.
 *    
 *    id:
 *      A unique identifier for this paper type.
 *    
 *    name:
 *      The name to display to the user for this paper type.
 *    
 *    width:
 *      The width, in points, of the paper.
 *    
 *    height:
 *      The height, in points, of the paper.
 *    
 *    margins:
 *      The unprintable margins on the paper.
 *    
 *    paperP:
 *      if this function is successful, returning noErr, then *'paperP'
 *      is set to be a reference to a newly created PMPaper instance.
 *      The caller is responsible for calling PMRelease when the
 *      instance is no longer needed. If this functions fails, it will
 *      return a non-zero error and set *'paperP' to NULL.
 *  
 *  Discussion:
 *		This function creates a new custom paper instance. To obtain one of the available
 *		built-in paper sizes for a given printer, use PMPrinterGetPaperList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperCreateCustom( printer: PMPrinter; id: CFStringRef; name: CFStringRef; width: Float64; height: Float64; const (*var*) margins: PMPaperMargins; var paperP: PMPaper ): OSStatus; external name '_PMPaperCreateCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetWidth()
 *  
 *  Summary:
 *    Returns the width for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the width for this paper.
 *    
 *    paperWidth:
 *      If successful noErr is returned and *paperWidth is set to the
 *      width of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetWidth( paper: PMPaper; var paperWidth: Float64 ): OSStatus; external name '_PMPaperGetWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetHeight()
 *  
 *  Summary:
 *    Returns the height for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the height for this paper.
 *    
 *    paperHeight:
 *      If successful noErr is returned and *paperHeight is set to the
 *      height of the paper.
=======
 *  PMGetAdjustedPageRect()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetAdjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetAdjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetAdjustedPaperRect()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetAdjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetAdjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetOrientation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetOrientation( pageFormat: PMPageFormat; var orientation: PMOrientation ): OSStatus; external name '_PMGetOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetPageFormatExtendedData()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; var size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMGetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPageFormatGetPrinterID()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *		Obtains the formatting printer for the pageformat.
 *
 *  Discussion:
 *		Will either return the formatting printer for the pageformat
 *		or will return NULL if the pageformat doesn't have that information.
 * 
 *  Parameters:
 *    
 *    pageFormat:
 *      The pageformat to obtain the information from.
 *
 *	  printerID:
 *		Where to store the name of the printer
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetHeight( paper: PMPaper; var paperHeight: Float64 ): OSStatus; external name '_PMPaperGetHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetMargins()
 *  
 *  Summary:
 *    Returns the margins for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the margin information for this paper.
 *    
 *    paperMargins:
 *      If successful noErr is returned and *paperMargins is set to the
 *      margins of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetMargins( paper: PMPaper; var paperMargins: PMPaperMargins ): OSStatus; external name '_PMPaperGetMargins';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperGetID()
 *  
 *  Summary:
 *    Returns the id for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the id for this paper.
 *    
 *    paperID:
 *      If successful noErr is returned and *paperID is set to the id
 *      of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetID( paper: PMPaper; var paperID: CFStringRef ): OSStatus; external name '_PMPaperGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetName()
 *  
 *  Summary:
 *    Returns the name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the name for this paper.
 *    
 *    paperName:
 *      If successful noErr is returned and *paperName is set to the
 *      name of the paper. The returned paper name is that appropriate
 *      to display to a user.
=======
 *      Obtain the comm information for this printer.
 *    
 *    supportsControlCharRangeP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range
 *      0x0 - 0x1F
 *    
 *    supportsEightBitP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range 
 *      0x80 - 0xFF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetCommInfo( printer: PMPrinter; var supportsControlCharRangeP: Boolean; var supportsEightBitP: Boolean ): OSStatus; external name '_PMPrinterGetCommInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetID()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetID( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetLocation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLocation( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetDriverCreator()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDriverCreator( printer: PMPrinter; var creator: OSType ): OSStatus; external name '_PMPrinterGetDriverCreator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatGetPrinterID( pageFormat: PMPageFormat; var printerID: CFStringRef ): OSStatus; external name '_PMPageFormatGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMGetScale()
 *  
 *  Summary:
 *    Obtains the scaling factor currently applied to the page and
 *    paper rectangles.
 *  
 *  Discussion:
 *    A value of 100.0 means 100% (no scaling). 50.0 means 50% scaling
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetScale( pageFormat: PMPageFormat; var scale: Float64 ): OSStatus; external name '_PMGetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPageRect()
 *  
 *  Summary:
 *    Obtains the size of the imageable area in points, unaffected by
 *    orientation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the imageable area of the page without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPaperRect()
 *  
 *  Summary:
 *    Obtains a rectangle that specifies the size of the paper in
 *    points, unaffected by rotation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the physical size of the paper without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)



{**********************}
{ PMSetxxx calls only save the value inside the printing object. They make no assumption on the }
{ validity of the value. This should be done using PMValidatePageFormat/PMSessionValidatePageFormat }
{ Any dependent settings are also updated during a validate call. }
{ For example: }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 )  }
{ PMGetAdjustedPaperRect - returns the SAME rect as the first call  }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 ) }
{ PMValidatePageFormat or PMSessionValidatePageFormat }
{ PMGetAdjustedPaperRect - returns a rect thats scaled 500% from the first call }
{**********************}

{
 *  PMSetOrientation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetOrientation( pageFormat: PMPageFormat; orientation: PMOrientation; lock: Boolean ): OSStatus; external name '_PMSetOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetPageFormatExtendedData()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMSetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetScale()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetScale( pageFormat: PMPageFormat; scale: Float64 ): OSStatus; external name '_PMSetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

(*
#pragma mark -
#pragma mark PrintSettings
#pragma mark
*)

{$ifc PM_USE_SESSION_APIS}
{
<<<<<<< HEAD
 *  PMPrinterGetDriverReleaseInfo()
=======
 *  PMCreatePrintSettings()
 *  
 *  Summary:
 *    Allocates memory for a new PMPrintSettings object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A printSettings is created with a refcount of 1.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrinterGetDriverReleaseInfo( printer: PMPrinter; var release: VersRec ): OSStatus; external name '_PMPrinterGetDriverReleaseInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetPrinterResolutionCount()
 *  
 *  Summary:
 *    Provides the number of available hardware resolution settings a printer supports. 
>>>>>>> graemeg/cpstrnew
=======
function PMCreatePrintSettings( var printSettings: PMPrintSettings ): OSStatus; external name '_PMCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPrintSettings()
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the resolution count for this printer.
 *    
 *    countP:
 *      Storage for the returned count indicating the number of available hardware
 *      resolution settings for the specified printer.
 *
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetPPDPaperName()
 *  
 *  Summary:
 *    Returns the PPD name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the PPD name for this paper.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      PPD name of the paper. The returned result may be NULL.
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDefaultPrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings ): OSStatus; external name '_PMSessionDefaultPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePrintSettings()
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *      The Mac OS X printing system uses a PostScript Printer Description (PPD)
 *      file to describe a given printer and print queue for that printer. The
 *      PPD name is the name that uniquely identifies a given paper
 *      to the printer to which the paper corresponds. 
 *    
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetPPDPaperName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetPPDPaperName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPaperCreateLocalizedName()
 *  
 *  Summary:
 *    Returns the localized name for a given paper. 
 *    Use of PMPaperCreateLocalizedName
 *    is recommended instead of PMPaperGetName.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the localized name for this paper.
 *    
 *    printer:
 *      The printer for which the localization should be performed.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      localized name of the paper. The returned paper name is that appropriate
 *      to display to a user. The returned result may be NULL.
 *      If non-NULL, the caller is responsible for releasing the name returned. 
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetPrinterResolutionCount( printer: PMPrinter; var countP: UInt32 ): OSStatus; external name '_PMPrinterGetPrinterResolutionCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrinterGetIndexedPrinterResolution()
 *  
 *  Summary:
 *    Together with PMPrinterGetPrinterResolutionCount, allows iterating over the
 *    hardware resolution settings a printer supports. 
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer of interest.
 *    
 *    index:
 *      The index of the resolution to return. The minimum value for index is 1 and
 *      the maximum value of index is the count returned by PMPrinterGetPrinterResolutionCount.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetIndexedPrinterResolution( printer: PMPrinter; index: UInt32; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetIndexedPrinterResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetOutputResolution()
 *  
 *  Summary:
 *    Obtain the printer hardware output resolution for the supplied
 *    printer and print settings. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterGetOutputResolution returns the best guess as to what printer
 *    resolution setting will be used for the destination print job. If the
 *    resolution setting cannot be reliably determined this function returns
 *    kPMKeyNotFound. 
 *  
 *    Most applications do not need to use this function since they draw the same
 *    content regardless of the destination device. For those few applications that
 *    do adjust their drawing based on the output device, they should only do so
 *    when the print job destination is kPMDestinationPrinter or kPMDestinationFax.
 *  
 *    This call should be used after displaying the print dialog to the user so
 *    that it correctly reflects settings changes performed prior to printing.
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPrintSettings( settingSrc: PMPrintSettings; settingDest: PMPrintSettings ): OSStatus; external name '_PMCopyPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPrintSettings object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPrintSettingsToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPrintSettingsCreateWithDataRepresentation to create a PMPrintSettings from 
 *    a CFDataRef created by this call.
 *
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCreateDataRepresentation( printSettings: PMPrintSettings; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPrintSettingsCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPrintSettings object from a data representation created with
 *	  PMPrintSettingsCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *    
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings from which to obtain the printer hardware output resolution.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
function PMPaperCreateLocalizedName( paper: PMPaper; printer: PMPrinter; var paperName: CFStringRef ): OSStatus; external name '_PMPaperCreateLocalizedName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetPrinterID()
 *  
 *  Summary:
 *    Returns the printerID of the printer for which the paper corresponds.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the printer ID for this paper.
 *    
 *    printerID:
 *      If successful, noErr is returned and *printerID is set to the
 *      ID of the printer for which the paper corresponds. The returned
 *      result may be NULL. 
=======
function PMPrinterGetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterSetOutputResolution()
 *  
 *  Summary:
 *    Set the printer hardware output resolution in the print settings
 *    for the supplied printer. 
=======
function PMPrintSettingsCreateWithDataRepresentation( data: CFDataRef; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPrintSettingsCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


  
{
 *  PMGetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCollate( printSettings: PMPrintSettings; var collate: Boolean ): OSStatus; external name '_PMGetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMGetCopies()
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterSetOutputResolution configures the print settings to the closest
 *    resolution setting that can be used for the destination print job. Note
 *    that not all printers allow control of their resolution setting.
 *
 *    This function is rarely used since most applications do not set the output 
 *    resolution but instead use the setting supplied by the user in the print dialog.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings in which to set the hardware resolution.
 *    
 *    resolutionP:
 *      A pointer to the PMResolution to use to set the hardware output resolution.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPaperGetPrinterID( paper: PMPaper; var printerID: CFStringRef ): OSStatus; external name '_PMPaperGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperIsCustom()
 *  
 *  Summary:
 *    Returns true if the paper is a custom paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Determines if this is a custom paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperIsCustom( paper: PMPaper ): Boolean; external name '_PMPaperIsCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
function PMPrinterSetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; const (*var*) resolutionP: PMResolution ): OSStatus; external name '_PMPrinterSetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrinterGetLanguageInfo()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLanguageInfo( printer: PMPrinter; var info: PMLanguageInfo ): OSStatus; external name '_PMPrinterGetLanguageInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCopies( printSettings: PMPrintSettings; var copies: UInt32 ): OSStatus; external name '_PMGetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDuplex( printSettings: PMPrintSettings; var duplexSetting: PMDuplexMode ): OSStatus; external name '_PMGetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMGetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetFirstPage( printSettings: PMPrintSettings; var first: UInt32 ): OSStatus; external name '_PMGetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew

(*
#pragma mark
#pragma mark PDF Workflow
#pragma mark
*)
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMWorkflowCopyItems()
 *  
 *  Summary:
 *    Hand back an array of dictionaries describing the PDF Workflow
 *    items installed on the system
 *  
 *  Parameters:
 *    
 *    workflowItems:
 *      If this function returns without error then *'workflowItems'
 *      will be filled in with a reference to an array. It is the
 *      caller's responsability to release the array when done with it.
 *      Each element in the array describes a PDF Workflow item or a
 *      folder holding workflow items. A dictionary describing a
 *      workflow item has, at least, the following keys and values:
 *      displayName - The user's diaplayable name for the workflow item
 *      itemURL - A CFURLRef pointing to the workflow item. A
 *      dictionary describing a workflow folder has at least the
 *      following keys: displayName - The user's diaplayable name for
 *      the workflow item folderURL - A CFURLRef pointing to the
 *      folder. items - A CFArrayRef describing the workflow items in
 *      the folder. If this function returns a non-zero error code then
 *      *'workflowItems' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowCopyItems( var workflowItems: CFArrayRef ): OSStatus; external name '_PMWorkflowCopyItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithOptions()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    title:
 *      The user displayable name of the document.
 *    
 *    options:
 *      A string of CUPS style key-value pairs that may be passed to
 *      the PDF Workflow item. This parameter can be NULL in which case
 *      an empty string of options is used.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
=======
 *  PMPrinterGetMakeAndModelName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetMakeAndModelName( printer: PMPrinter; var makeAndModel: CFStringRef ): OSStatus; external name '_PMPrinterGetMakeAndModelName';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetMimeTypes()
 *  
 *  Summary:
 *    Return the array of mime type supported by the printer for a
 *    given set of print settings.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job. The part of the print
 *      settings that effects the available mime type is the
 *      destination. This parameter can be NULL.
 *    
 *    mimeTypes:
 *      If this function returns without error then *'mimeTypes' is
 *      filled in with a reference to an array of CFStrings. Each
 *      CFString names a mime type supported by the printer with the
 *      specified print settings. The caller must not release this
 *      reference without first doing a retain. If this function
 *      returns an error then 'mimeTypes' will be set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMWorkflowSubmitPDFWithOptions( workflowItem: CFURLRef; title: CFStringRef; options: ConstCStringPtr; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithOptions';
=======
function PMPrinterGetMimeTypes( printer: PMPrinter; settings: PMPrintSettings; var mimeTypes: CFArrayRef ): OSStatus; external name '_PMPrinterGetMimeTypes';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMWorkflowSubmitPDFWithSettings()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    settings:
 *      The prints settings to apply to the PDF.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowSubmitPDFWithSettings( workflowItem: CFURLRef; settings: PMPrintSettings; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Job Submission
#pragma mark
*)
{
 *  PMPrinterPrintWithProvider()
 *  
 *  Summary:
 *    Submit print data to a specified printer.
 *  
 *  Discussion:
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. This parameter can not
 *      be NULL. Use PMPrinterPrintWithFile() if auto-typing is desired.
 *    
 *    provider:
 *      The data provider that supplies the print data.
=======
 *  PMPrinterGetName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetName( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetName';
=======
 *  PMGetLastPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetLastPage( printSettings: PMPrintSettings; var last: UInt32 ): OSStatus; external name '_PMGetLastPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetPageRange()
 *  
 *  Summary:
 *    Obtains the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The default page range is 1 - (all pages). The page range is
 *    something that is set by the application. It is NOT the first and
 *    last page to print. It serves as limits for setting the first and
 *    last page.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageRange( printSettings: PMPrintSettings; var minPage: UInt32; var maxPage: UInt32 ): OSStatus; external name '_PMGetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrintSettingsGetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetJobName( printSettings: PMPrintSettings; var name: CFStringRef ): OSStatus; external name '_PMPrintSettingsGetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  PMPrintSettingsGetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings to retrieve the value from
 *    
 *    key:
 *      The key to look for
 *    
 *    value:
 *      The return value. Its either the value for the key or NULL
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetValue( printSettings: PMPrintSettings; key: CFStringRef; var value: CFTypeRef ): OSStatus; external name '_PMPrintSettingsGetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)



{
 *  PMSetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCollate( printSettings: PMPrintSettings; collate: Boolean ): OSStatus; external name '_PMSetCollate';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetPaperList()
 *  
 *  Summary:
 *    Returns the list of papers available for a given printer.
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the paper list for this printer.
 *    
 *    paperList:
 *      If successful noErr is returned and *paperList is a CFArray of
 *      PMPapers representing the list of papers available for the
 *      printer.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrinterPrintWithProvider( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; provider: CGDataProviderRef ): OSStatus; external name '_PMPrinterPrintWithProvider';
=======
function PMPrinterGetPaperList( printer: PMPrinter; var paperList: CFArrayRef ): OSStatus; external name '_PMPrinterGetPaperList';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterPrintWithFile()
 *  
 *  Summary:
 *    Submit a file for printing to a specified printer.
 *  
 *  Discussion:
 *
 *    One reason this function may fail is if the specified printer can
 *    not handle the file's mime type. Use PMPrinterGetMimeTypes() to
 *    check whether a mime type is supported.
 *  
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. If this parameter is
 *      NULL then the supplied file will be auto-typed.
 *    
 *    fileURL:
 *      A file URL specifying the file to be printed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterPrintWithFile( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; fileURL: CFURLRef ): OSStatus; external name '_PMPrinterPrintWithFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPrinterWritePostScriptToURL()
 *
 *  Summary:
 *    Convert an input file of the specified mimeType to printer ready PostScript for the destination printer.
 *
 *  Discussion:
 *
 *    The conversion of the input file to PostScript is performed before this function returns. This can take a significant
 *    amount of time for longer documents. The caller of PMPrinterWritePostScriptToURL may want to perform this operation
 *    on a thread other than the main application thread or fork a separate process for this purpose.
 *
 *    One reason this function may fail is if conversion from the input mimeType to PostScript cannot be performed.
 *
 *
 *  Parameters:
 *
 *    printer:
 *      The printer for which the printer ready PostScript will be generated.
 *
 *    settings:
 *      The print settings for the print job.
 *
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed.
 *
 *    mimeType:
 *      The mime type of the file to be printed. If NULL, the file is auto-typed.
 *
 *    sourceFileURL:
 *      A file URL specifying the input file to be converted to printer ready PostScript data. Only file based URLs
 *      are supported.
 *
 *    destinationFileURL:
 *      A file URL specifying the destination file to be created. If the file already exists it will be overwritten. Only
 *      file based URLs are supported.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterWritePostScriptToURL( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; sourceFileURL: CFURLRef; destinationFileURL: CFURLRef ): OSStatus; external name '_PMPrinterWritePostScriptToURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsToOptions()
 *  
 *  Summary:
 *    Convert print settings to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'ed C string
 *      describing the passed in print settings. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsToOptions( settings: PMPrintSettings; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrintSettingsToOptionsWithPrinterAndPageFormat()
 *  
 *  Summary:
 *    Convert a print settings and page format to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string. Must not be NULL.
 *    
 *    printer:
 *      The printer to use for converting the print settings. Must not be NULL.
 *
 *    format:
 *      The page format for use in creating the cups options. Can be NULL.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'd C string
 *      describing the passed in print settings and format. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
=======
 *  PMPrinterGetState()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetState( printer: PMPrinter; var state: PMPrinterState ): OSStatus; external name '_PMPrinterGetState';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsDefault()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsDefault( printer: PMPrinter ): Boolean; external name '_PMPrinterIsDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsFavorite()
 *  
 *  Summary:
 *    Return true if the printer is in the user's favorite printer list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsFavorite( printer: PMPrinter ): Boolean; external name '_PMPrinterIsFavorite';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsPostScriptCapable()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsPostScriptCapable( printer: PMPrinter ): Boolean; external name '_PMPrinterIsPostScriptCapable';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{!
 * @function	PMPrinterIsPostScriptPrinter
 * @abstract	Set *isPSPrinter true if the printer is a PostScript printer.
 *
 * @discussion	A PostScript printer is one whose driver takes PostScript directly.
 *
 *  Availability:
>>>>>>> graemeg/cpstrnew
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrintSettingsToOptionsWithPrinterAndPageFormat( settings: PMPrintSettings; printer: PMPrinter; pageFormat: PMPageFormat; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptionsWithPrinterAndPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark PrinterCommandSupport
#pragma mark
*)

{
*  PMPrinterSendCommand()
*
*  Summary:
*    Submit a command file containing the specified string data to a print queue.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    commandString:
*      The contents of the command file as a CFStringRef.
*
*    jobTitle:
*      The title of the job associated with the command file. If NULL,
*      a job title will be automatically generated but it may not be
*      meaningful to a user.
*
*    options:
*      A dictionary containing options that apply to the job.
*      This parameter should be NULL; it is reserved for future expansion.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}
function PMPrinterSendCommand( printer: PMPrinter; commandString: CFStringRef; jobTitle: CFStringRef; options: CFDictionaryRef ): OSStatus; external name '_PMPrinterSendCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{
*  PMPrinterCopyState()
*
*  Summary:
*    Copies the current "marker-change-time", "marker-colors",
*    "marker-high-levels", "marker-levels", "marker-low-levels",
*    "marker-message", "marker-names", "marker-types", "printer-state",
*    "printer-state-change-time", "printer-state-message", and
*    "printer-state-reasons" values for a print queue.  These attributes are
*    defined in the "CUPS Implementation of IPP" specification at:
*
*        http://www.cups.org/documentation.php/spec-ipp.html
*        http://localhost:631/help/spec-ipp.html
*
*    Each attribute is returned using the corresponding CoreFoundation
*    data type:
*
*        Attribute                    Type
*        ---------------------------  ------------------------
*        "marker-change-time"         CFDateRef
*        "marker-colors"              CFArrayRef + CFStringRef
*        "marker-high-levels"         CFArrayRef + CFNumberRef
*        "marker-levels"              CFArrayRef + CFNumberRef
*        "marker-low-levels"          CFArrayRef + CFNumberRef
*        "marker-message"             CFStringRef
*        "marker-names"               CFArrayRef + CFStringRef
*        "marker-types"               CFArrayRef + CFStringRef
*        "printer-state"              CFNumberRef
*        "printer-state-change-time"  CFDateRef
*        "printer-state-message"      CFStringRef
*        "printer-state-reasons"      CFArrayRef + CFStringRef
*
*  Localization Considerations:
*
*    The "marker-message" and "printer-state-message" attributes may be
*    localized by the printer driver using the language of the last processed
*    print job.
*
*    The "marker-names" and "printer-state-reasons" attributes are typically
*    English-language keywords that may be localized using the printer's PPD
*    file and CUPS ppdLocalizeMarkerName and ppdLocalizeIPPReason APIs,
*    respectively.
*
*    All other attributes are predefined keywords or values that are not
*    localized.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    stateDict:
*      A pointer to a CFDictionaryRef variable to hold the values.
*      You must release the dictionary when you are done using it.
*      The dictionary returned is not mutable.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}
=======
function PMPrinterIsPostScriptPrinter( printer: PMPrinter; var isPSPrinter: Boolean ): OSStatus; external name '_PMPrinterIsPostScriptPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew

function PMPrinterCopyState( printer: PMPrinter; var stateDict: CFDictionaryRef ): OSStatus; external name '_PMPrinterCopyState';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

(*
#pragma mark
#pragma mark PPD related
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMCopyAvailablePPDs()
 *  
 *  Summary:
 *    Hand back the list of PPDs in the specified PPD domain.
 *  
 *  Parameters:
 *    
 *    domain:
 *      The domain to search for PPDs.
 *    
 *    ppds:
 *      If this function completes without error, *'ppds' is set to an
 *      array of CFURLs. Each CFURL specifies the location of a PPD
 *      file or a compressed PPD file. The caller is responsible for
 *      releasing the array. If this function returns a non-zero error
 *      code then *'ppds' is set to NULL.
=======
 *  PMPrinterIsRemote()
 *  
 *  Summary:
 *    Hand back a boolean indicating whether the printer is hosted by remote print server.
 *  
 *  Discussion:
 *    If on return *isRemoteP is true, the print queue represents a printer hosted and
 *    managed by a remote print server.
 *
 *    If on return *isRemoteP is false, the print queue represents a directly connected
 *    printer, a network printer, or a remote printer that is locally managed. Consult
 *    the queue's device URI to determine the type of connection that is used to communicate
 *    with the printer.
 *
 *    Whether a printer is remote is derived from the CUPS printer-type attribute for the print queue.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMCopyAvailablePPDs( domain: PMPPDDomain; var ppds: CFArrayRef ): OSStatus; external name '_PMCopyAvailablePPDs';
=======
function PMPrinterIsRemote( printer: PMPrinter; var isRemoteP: Boolean ): OSStatus; external name '_PMPrinterIsRemote';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMCopyLocalizedPPD()
 *  
 *  Summary:
 *    Hand back a reference to a localized PPD.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A PPD reference. Typically this is a CFURLRef returned from
 *      PMCopyAvailablePPDs().
 *    
 *    localizedPPD:
 *      If this function completes without error, *'localizedPPD' will
 *      be set to a CFURLRef referencing the PPD that should be used
 *      given the current user's language preferences. If this function
 *      returns an error then *'localizedPPD' will be set to to NULL.
=======
 *  PMSetCopies()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCopies( printSettings: PMPrintSettings; copies: UInt32; lock: Boolean ): OSStatus; external name '_PMSetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetDuplex( printSettings: PMPrintSettings; duplexSetting: PMDuplexMode ): OSStatus; external name '_PMSetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetFirstPage( printSettings: PMPrintSettings; first: UInt32; lock: Boolean ): OSStatus; external name '_PMSetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetLastPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetLastPage( printSettings: PMPrintSettings; last: UInt32; lock: Boolean ): OSStatus; external name '_PMSetLastPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetPageRange()
 *  
 *  Summary:
 *    Sets the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The first and last page are immediately clipped to the new range.
 *    You may pass kPMPrintAllPages for the maxPage value to specified
 *    that all pages are available for printing.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPageRange( printSettings: PMPrintSettings; minPage: UInt32; maxPage: UInt32 ): OSStatus; external name '_PMSetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsSetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetJobName( printSettings: PMPrintSettings; name: CFStringRef ): OSStatus; external name '_PMPrintSettingsSetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrintSettingsSetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings in which to set the value
 *    
 *    key:
 *      The key to store the value in
 *    
 *    value:
 *      The value to store in the key. If NULL, any existing setting item with
 *      the specified key is removed.
 *    
 *    locked:
 *      A boolean value indicating whether the item being set should be
 *      locked. It is strongly recommended to pass false.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetValue( printSettings: PMPrintSettings; key: CFStringRef; value: CFTypeRef; locked: Boolean ): OSStatus; external name '_PMPrintSettingsSetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
  
  
{
 *  PMPrintSettingsCopyAsDictionary()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Represent these print settings as a dictionary.
 *    
 *    settingsDictionary:
 *      On exit, if successful *'settingsDictionary' will contain a reference to
 *      a CFDictionary describing the print settings. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'settingsDictionary' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCopyAsDictionary( printSettings: PMPrintSettings; var settingsDictionary: CFDictionaryRef ): OSStatus; external name '_PMPrintSettingsCopyAsDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrintSettingsCopyKeys()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Return the keys for items in this print settings.
 *    
 *    settingsKeys:
 *      On exit, if successful *'settingsKeys' will contain a reference to
 *      a CFArray describing the item keys in the print settings. Each of these
 *		keys may be passed to PMPrintSettingsGetValue() to obtain a value.
 *		The caller is responsible for releasing this reference. If this call returns
 *      an error, then *'settingsKeys' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCopyKeys( printSettings: PMPrintSettings; var settingsKeys: CFArrayRef ): OSStatus; external name '_PMPrintSettingsCopyKeys';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark -
#pragma mark Printer
#pragma mark
*)

{!
 *
 * PMCreateGenericPrinter
 *  Summary:
 *	Creates a generic PMPrinter
 *
 *  Parameters:
 *		printer:
 *		On return *printer contains the generic priner
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMCreateGenericPrinter( var printer: PMPrinter ): OSStatus; external name '_PMCreateGenericPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMServerCreatePrinterList()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Hand back an array of PMPrinter objects that represents the list of printers
 *    known to the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server for which you want to obtain the list of printers.
 *      Use kPMServerLocal for the local print server.
 *    
 *    printerList:
 *      On return, *printerList contains the array of PMPrinter objects returned. The
 *	caller is responsible for releasing this array. On error, *printerList is NULL.
 *    
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyLocalizedPPD( ppd: CFURLRef; var localizedPPD: CFURLRef ): OSStatus; external name '_PMCopyLocalizedPPD';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
 *  PMPrinterSetDefault()
 *  
 *  Summary:
 *    Set the default printer for the current user.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer to set as the default printer.
 *  
 *  Special considerations:
 *      It is not typical for an application to set the current default printer
 *      for the user; the printing system itself takes care of managing the default printer.
 *      This function should be used only in rare circumstances.
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMServerCreatePrinterList( server: PMServer; var printerList: CFArrayRef ): OSStatus; external name '_PMServerCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMServerLaunchPrinterBrowser()
>>>>>>> graemeg/cpstrnew
 *  
 *  Summary:
 *    Launch the printer browser, browsing the printers available for
 *    the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server whose available printers the browser should browse.
 *      Pass kPMServerLocal for the local print server.
 *    
 *    options:
 *      A CFDictionaryRef specifying how the browser should be presented. 
 *      Passing NULL presents the browser in the default fashion. 
 *
 *  Discussion:	The local print server allows its printers to be browsed but
 *		not all print servers do. Passing in a server whose printers cannot be
 *      browsed returns the error code kPMInvalidParameter.
 *    
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrinterSetDefault( printer: PMPrinter ): OSStatus; external name '_PMPrinterSetDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function PMServerLaunchPrinterBrowser( server: PMServer; options: CFDictionaryRef ): OSStatus; external name '_PMServerLaunchPrinterBrowser';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew

(*
#pragma mark
#pragma mark Preset
#pragma mark
*)
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMCopyPPDData()
 *  
 *  Summary:
 *    Hand back the uncompressed PPD data for a PPD or compressed PPD
 *    file.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A reference to a PPD or compressed PPD file. This reference is
 *      usually obtained from PMCopyAvailablePPDs() or from
 *      PMCopyLocalizedPPD().
 *    
 *    data:
 *      If this function completes without error then *'data' is set to
 *      reference the uncompressed PPD data from the PPD file. If this
 *      function returns a non-zero error then *'data is set to NULL.
=======
 *  PMPresetCopyName()
 *  
 *  Summary:
 *    Hand back a copy of the localized name for the specified preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      The preset whose name is needed.
 *    
 *    name:
 *      On exit, if this routine succeeds, *'name' is filled in with a
 *      reference to a localized string with the preset's name. If this
 *      routine fails, then *'name' is set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMCopyPPDData( ppd: CFURLRef; var data: CFDataRef ): OSStatus; external name '_PMCopyPPDData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
function PMPresetCopyName( preset: PMPreset; var name: CFStringRef ): OSStatus; external name '_PMPresetCopyName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
 *  PMPrinterCreateFromPrinterID()
 *  
 *  Summary:
 *    Create a PMPrinter instance from the supplied printerID.
 *  
 *  Discussion:
 *    Returns a PMPrinter instance for the printer whose ID is
 *    printerID. The caller is responsible for releasing this instance
 *    with PMRelease. The PMPrinter instance returned will be NULL if
 *    there is no printer available which corresponds to the supplied
 *    printerID.
 *  
 *  Parameters:
 *    
 *    printerID:
 *      The printerID for the printer for which you want the PMPrinter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCreateFromPrinterID( printerID: CFStringRef ): PMPrinter; external name '_PMPrinterCreateFromPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

>>>>>>> graemeg/cpstrnew

(*
#pragma mark
#pragma mark EPS related
#pragma mark
*)
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMCGImageCreateWithEPSDataProvider()
 *  
 *  Summary:
 *    Create an image reference that references both the PostScript
 *    contents of an EPS file and a preview (proxy) image for that EPS
 *    file.
=======
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePageFormat( var pageFormat: PMPageFormat ): OSStatus; external name '_PMCreatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDefaultPageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionDefaultPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePageFormat()
>>>>>>> origin/cpstrnew
 *  
 *  Discussion:
 *    For OS X 10.1.0, this function ignores the passed in data
 *    provider. The passed in image reference is retained and then
 *    returned. For 10.1.1 and later, then the data provider is used
 *    and the returned image reference is different than the passed in
 *    image reference, so please be careful with your use of these
 *    references. It is likely that the data will not be read from the
 *    EPS data provider until well after this function returns. The
 *    caller should be careful not to free the underlying EPS data
 *    until the provider's release routine is invoked. Similarly the
 *    preview image's data may be needed long after you think it should
 *    be. Do not free the image data until the image data provider's
 *    release data function has been called. To make sure these data
 *    providers are properly reference counted, release your reference
 *    the EPS data provider and on the EPS image preview when they are
 *    no longer needed by your application. For Mac OS X 10.2 and
 *    later, the contents of the EPS provider at the time of this call
 *    can be dumped to a file if you first do the following, BEFORE
 *    running your application. 
 *
 *	  From the command line in terminal:
 *		defaults write NSGlobalDomain com.apple.print.eps.testProvider /tmp/dump.eps
 *    
 *	  causes a dump of the EPS data into a file /tmp/dump.eps.
 *  
 *  Parameters:
 *    
 *    epsDataProvider:
 *      A Core Graphics data provider that can supply the PostScript
 *      contents of the EPS file. Post OS X 10.1, there will be some
 *      checking done on the EPS data provided to the
 *      PMCGImageCreateWithEPSDataProvider() call. It is important that
 *      the EPS data begin with the EPSF required header and bounding
 *      box DSC comments.
 *    
 *    epsPreview:
 *      A Core Graphics image reference to the proxy image for the EPS
 *      file. When the image reference result of this function is
 *      rendered on screen or printed to a printer that can not render
 *      PostScript this proxy image is drawn instead.
 *  
 *  Result:
 *    an image reference capable of rendering either the EPS content or
 *    the proxy image depending upon the capabilities of the targeted
 *    context.
=======
 *  PMPrinterCopyDescriptionURL()
 *  
 *  Summary:
 *    Hand back a URL to the printer's PostScript Printer Description (PPD) file in fileURL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose PPD will be returned.
 *    
 *    descriptionType:
 *      The type of description desired. Only kPMPPDDescriptionType is currently supported.
 *    
 *    fileURL:
 *      A pointer to storage for a CFURL reference that will be returned. The caller is responsible
 *      for releasing the reference returned. If this call returns an error, then *fileURL will
 *      be set to NULL.
 *
 *  Discussion:
 *    Only descriptionType of kPMPPDDescriptionType is supported, otherwise kPMInvalidParameter
 *    is returned.
 *
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> graemeg/cpstrnew
=======
 *  PMPresetCreatePrintSettings()
 *  
 *  Summary:
 *    Create a print settings conforming to the specified print
 *    settings preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A preset specifying a set of initial print settings.
 *    
 *    session:
 *      A valid print session.
 *    
 *    printSettings:
 *      On exit, *'printSettings' is set to a newly created print
 *      settings that contains the settings specified by 'preset'. The
 *      caller is responsible for calling PMRelease when the print
 *      settings are no longer needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPresetCreatePrintSettings( preset: PMPreset; session: PMPrintSession; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPresetCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPresetGetAttributes()
 *  
 *  Summary:
 *    Hand back the meta-data describing a given preset.
 *  
 *  Discussion:
 *    Each preset has associated with it a dictionary containing
 *    meta-data. The meta-data provides the preset's id, the preset's
 *    localized names, and descriptions of the environment for which
 *    the preset it intended.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A print settings preset as obtained from PMPrinterCopyPresets().
 *    
 *    attributes:
 *      On exit, *'attributes' is set to reference a dictionary
 *      containing the preset's meta-data. The caller is responsible
 *      for retaining this reference if it is to be used beyond the
 *      lifetime of 'preset'. If this function fails, returning a
 *      non-zero error code, then *'attributes' is set to NULL.
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyDescriptionURL( printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef ): OSStatus; external name '_PMPrinterCopyDescriptionURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyDeviceURI()
 *  
 *  Summary:
 *    Hand back the URI of the printer's device.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose device URI is to be retrieved.
 *    
 *    deviceURI:
 *      On exit, if successful *'deviceURI' will contain a reference to
 *      a CFURL describing the printer's device. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'deviceURI' will be set to NULL.
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPageFormat( formatSrc: PMPageFormat; formatDest: PMPageFormat ): OSStatus; external name '_PMCopyPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMCreatePageFormatWithPMPaper()
 *  
 *  Summary:
 *    Create a pageformat with a specific paper.
 *  
 *  Parameters:
 *    
 *    pageFormat:
 *      On return, will contain the pageformat which was created
 *    
 *    paper:
 *      The paper that will be associate with the pageformat
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePageFormatWithPMPaper( var pageFormat: PMPageFormat; paper: PMPaper ): OSStatus; external name '_PMCreatePageFormatWithPMPaper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPageFormatCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPageFormat object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
>>>>>>> origin/cpstrnew
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPageFormatToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPageFormatCreateWithDataRepresentation to create a PMPageFormat from 
 *    a CFDataRef created by this call.
 *  
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyDeviceURI( printer: PMPrinter; var deviceURI: CFURLRef ): OSStatus; external name '_PMPrinterCopyDeviceURI';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyHostName()
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateDataRepresentation( pageFormat: PMPageFormat; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPageFormatCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPageFormatCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPageFormat object from a data representation created with
 *	  PMPageFormatCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Hand back the host name of the print server hosting the printer's print queue.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPresetGetAttributes( preset: PMPreset; var attributes: CFDictionaryRef ): OSStatus; external name '_PMPresetGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateWithDataRepresentation( data: CFDataRef; var pageFormat: PMPageFormat ): OSStatus; external name '_PMPageFormatCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/cpstrnew

(*
#pragma mark
#pragma mark Paper
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMGetPageFormatPaper()
 *  
 *  Summary:
 *    Returns the paper associated with a pageformat.
 *  
 *  Parameters:
 *    
 *    format:
 *      Obtain the paper for this pageformat.
 *    
 *    paper:
 *      If successful noErr is returned and *paper will contain a
 *      PMPaper object describing the current paper associated with the
 *      pageformat.
=======
function PMPrinterCopyHostName( printer: PMPrinter; var hostNameP: CFStringRef ): OSStatus; external name '_PMPrinterCopyHostName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterCopyPresets()
 *  
 *  Summary:
 *    Provides a list of print settings presets for the specified
 *    printer.
 *  
 *  Discussion:
 *    A printer may have associated with it a list of preset settings.
 *    Each setting is optimized for a particular printing situation.
 *    This function returns all of the presets for a given printer. To
 *    obtain more information about a particular preset see
 *    PMPresetGetAttributes().
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the presets for this printer.
 *    
 *    presetList:
 *      On exit, *'presetList' is set to reference an array of presets.
 *      The caller must call CFRelease when it no longer needs the
 *      array. Each element of the array is a PMPreset. If this
 *      function fails, returning a non-zero error code, then
 *      *'presetList' will be set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMGetPageFormatPaper( format: PMPageFormat; var paper: PMPaper ): OSStatus; external name '_PMGetPageFormatPaper';
=======
function PMPrinterCopyPresets( printer: PMPrinter; var presetList: CFArrayRef ): OSStatus; external name '_PMPrinterCopyPresets';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPaperCreateCustom()
 *  
 *  Summary:
 *    Create a new custom paper instance.
=======
 *  PMPrinterGetCommInfo()
 *  
 *  Summary:
 *    Provides information about the comm channel characteristics for
 *    the printer.
 *  
 *  Discussion:
 *    This function is typically relevant only to PostScript capable
 *    printers. All PostScript printers, regardless of what
 *    communications channel is used to send data to them, support data
 *    in the range 0x20 - 0x7F. Many comm channels can support data
 *    outside this range. The Boolean returned in *supportsTransparentP
 *    indicates whether the comm channel to this printer supports bytes
 *    in the range 0x0 to 0x1F. The Boolean returned in
 *    *supportsEightBitP indicates whether the comm channel to this
 *    printer supports bytes with the high bit set, i.e. bytes in the
 *    range 0x80 - 0xFF.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
<<<<<<< HEAD
 *      The new paper size is appropriate for this printer.
 *    
 *    id:
 *      A unique identifier for this paper type.
 *    
 *    name:
 *      The name to display to the user for this paper type.
 *    
 *    width:
 *      The width, in points, of the paper.
 *    
 *    height:
 *      The height, in points, of the paper.
 *    
 *    margins:
 *      The unprintable margins on the paper.
 *    
 *    paperP:
 *      if this function is successful, returning noErr, then *'paperP'
 *      is set to be a reference to a newly created PMPaper instance.
 *      The caller is responsible for calling PMRelease when the
 *      instance is no longer needed. If this functions fails, it will
 *      return a non-zero error and set *'paperP' to NULL.
 *  
 *  Discussion:
 *		This function creates a new custom paper instance. To obtain one of the available
 *		built-in paper sizes for a given printer, use PMPrinterGetPaperList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperCreateCustom( printer: PMPrinter; id: CFStringRef; name: CFStringRef; width: Float64; height: Float64; const (*var*) margins: PMPaperMargins; var paperP: PMPaper ): OSStatus; external name '_PMPaperCreateCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetWidth()
 *  
 *  Summary:
 *    Returns the width for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the width for this paper.
 *    
 *    paperWidth:
 *      If successful noErr is returned and *paperWidth is set to the
 *      width of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetWidth( paper: PMPaper; var paperWidth: Float64 ): OSStatus; external name '_PMPaperGetWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetHeight()
 *  
 *  Summary:
 *    Returns the height for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the height for this paper.
 *    
 *    paperHeight:
 *      If successful noErr is returned and *paperHeight is set to the
 *      height of the paper.
=======
 *  PMGetAdjustedPageRect()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetAdjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetAdjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetAdjustedPaperRect()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetAdjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetAdjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetOrientation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetOrientation( pageFormat: PMPageFormat; var orientation: PMOrientation ): OSStatus; external name '_PMGetOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetPageFormatExtendedData()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; var size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMGetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPageFormatGetPrinterID()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *		Obtains the formatting printer for the pageformat.
 *
 *  Discussion:
 *		Will either return the formatting printer for the pageformat
 *		or will return NULL if the pageformat doesn't have that information.
 * 
 *  Parameters:
 *    
 *    pageFormat:
 *      The pageformat to obtain the information from.
 *
 *	  printerID:
 *		Where to store the name of the printer
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetHeight( paper: PMPaper; var paperHeight: Float64 ): OSStatus; external name '_PMPaperGetHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetMargins()
 *  
 *  Summary:
 *    Returns the margins for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the margin information for this paper.
 *    
 *    paperMargins:
 *      If successful noErr is returned and *paperMargins is set to the
 *      margins of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetMargins( paper: PMPaper; var paperMargins: PMPaperMargins ): OSStatus; external name '_PMPaperGetMargins';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperGetID()
 *  
 *  Summary:
 *    Returns the id for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the id for this paper.
 *    
 *    paperID:
 *      If successful noErr is returned and *paperID is set to the id
 *      of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetID( paper: PMPaper; var paperID: CFStringRef ): OSStatus; external name '_PMPaperGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetName()
 *  
 *  Summary:
 *    Returns the name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the name for this paper.
 *    
 *    paperName:
 *      If successful noErr is returned and *paperName is set to the
 *      name of the paper. The returned paper name is that appropriate
 *      to display to a user.
=======
 *      Obtain the comm information for this printer.
 *    
 *    supportsControlCharRangeP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range
 *      0x0 - 0x1F
 *    
 *    supportsEightBitP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range 
 *      0x80 - 0xFF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetCommInfo( printer: PMPrinter; var supportsControlCharRangeP: Boolean; var supportsEightBitP: Boolean ): OSStatus; external name '_PMPrinterGetCommInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetID()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetID( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetLocation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLocation( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetDriverCreator()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDriverCreator( printer: PMPrinter; var creator: OSType ): OSStatus; external name '_PMPrinterGetDriverCreator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatGetPrinterID( pageFormat: PMPageFormat; var printerID: CFStringRef ): OSStatus; external name '_PMPageFormatGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMGetScale()
 *  
 *  Summary:
 *    Obtains the scaling factor currently applied to the page and
 *    paper rectangles.
 *  
 *  Discussion:
 *    A value of 100.0 means 100% (no scaling). 50.0 means 50% scaling
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetScale( pageFormat: PMPageFormat; var scale: Float64 ): OSStatus; external name '_PMGetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPageRect()
 *  
 *  Summary:
 *    Obtains the size of the imageable area in points, unaffected by
 *    orientation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the imageable area of the page without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPaperRect()
 *  
 *  Summary:
 *    Obtains a rectangle that specifies the size of the paper in
 *    points, unaffected by rotation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the physical size of the paper without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)



{**********************}
{ PMSetxxx calls only save the value inside the printing object. They make no assumption on the }
{ validity of the value. This should be done using PMValidatePageFormat/PMSessionValidatePageFormat }
{ Any dependent settings are also updated during a validate call. }
{ For example: }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 )  }
{ PMGetAdjustedPaperRect - returns the SAME rect as the first call  }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 ) }
{ PMValidatePageFormat or PMSessionValidatePageFormat }
{ PMGetAdjustedPaperRect - returns a rect thats scaled 500% from the first call }
{**********************}

{
 *  PMSetOrientation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetOrientation( pageFormat: PMPageFormat; orientation: PMOrientation; lock: Boolean ): OSStatus; external name '_PMSetOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetPageFormatExtendedData()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMSetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetScale()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetScale( pageFormat: PMPageFormat; scale: Float64 ): OSStatus; external name '_PMSetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

(*
#pragma mark -
#pragma mark PrintSettings
#pragma mark
*)

{$ifc PM_USE_SESSION_APIS}
{
<<<<<<< HEAD
 *  PMPrinterGetDriverReleaseInfo()
=======
 *  PMCreatePrintSettings()
 *  
 *  Summary:
 *    Allocates memory for a new PMPrintSettings object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A printSettings is created with a refcount of 1.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrinterGetDriverReleaseInfo( printer: PMPrinter; var release: VersRec ): OSStatus; external name '_PMPrinterGetDriverReleaseInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetPrinterResolutionCount()
 *  
 *  Summary:
 *    Provides the number of available hardware resolution settings a printer supports. 
>>>>>>> graemeg/cpstrnew
=======
function PMCreatePrintSettings( var printSettings: PMPrintSettings ): OSStatus; external name '_PMCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPrintSettings()
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the resolution count for this printer.
 *    
 *    countP:
 *      Storage for the returned count indicating the number of available hardware
 *      resolution settings for the specified printer.
 *
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetPPDPaperName()
 *  
 *  Summary:
 *    Returns the PPD name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the PPD name for this paper.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      PPD name of the paper. The returned result may be NULL.
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDefaultPrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings ): OSStatus; external name '_PMSessionDefaultPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePrintSettings()
>>>>>>> origin/cpstrnew
 *  
 *  Discussion:
 *      The Mac OS X printing system uses a PostScript Printer Description (PPD)
 *      file to describe a given printer and print queue for that printer. The
 *      PPD name is the name that uniquely identifies a given paper
 *      to the printer to which the paper corresponds. 
 *    
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetPPDPaperName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetPPDPaperName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPaperCreateLocalizedName()
 *  
 *  Summary:
 *    Returns the localized name for a given paper. 
 *    Use of PMPaperCreateLocalizedName
 *    is recommended instead of PMPaperGetName.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the localized name for this paper.
 *    
 *    printer:
 *      The printer for which the localization should be performed.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      localized name of the paper. The returned paper name is that appropriate
 *      to display to a user. The returned result may be NULL.
 *      If non-NULL, the caller is responsible for releasing the name returned. 
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetPrinterResolutionCount( printer: PMPrinter; var countP: UInt32 ): OSStatus; external name '_PMPrinterGetPrinterResolutionCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrinterGetIndexedPrinterResolution()
 *  
 *  Summary:
 *    Together with PMPrinterGetPrinterResolutionCount, allows iterating over the
 *    hardware resolution settings a printer supports. 
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer of interest.
 *    
 *    index:
 *      The index of the resolution to return. The minimum value for index is 1 and
 *      the maximum value of index is the count returned by PMPrinterGetPrinterResolutionCount.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetIndexedPrinterResolution( printer: PMPrinter; index: UInt32; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetIndexedPrinterResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetOutputResolution()
 *  
 *  Summary:
 *    Obtain the printer hardware output resolution for the supplied
 *    printer and print settings. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterGetOutputResolution returns the best guess as to what printer
 *    resolution setting will be used for the destination print job. If the
 *    resolution setting cannot be reliably determined this function returns
 *    kPMKeyNotFound. 
 *  
 *    Most applications do not need to use this function since they draw the same
 *    content regardless of the destination device. For those few applications that
 *    do adjust their drawing based on the output device, they should only do so
 *    when the print job destination is kPMDestinationPrinter or kPMDestinationFax.
 *  
 *    This call should be used after displaying the print dialog to the user so
 *    that it correctly reflects settings changes performed prior to printing.
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPrintSettings( settingSrc: PMPrintSettings; settingDest: PMPrintSettings ): OSStatus; external name '_PMCopyPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPrintSettings object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPrintSettingsToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPrintSettingsCreateWithDataRepresentation to create a PMPrintSettings from 
 *    a CFDataRef created by this call.
 *
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCreateDataRepresentation( printSettings: PMPrintSettings; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPrintSettingsCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPrintSettings object from a data representation created with
 *	  PMPrintSettingsCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *    
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings from which to obtain the printer hardware output resolution.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
function PMPaperCreateLocalizedName( paper: PMPaper; printer: PMPrinter; var paperName: CFStringRef ): OSStatus; external name '_PMPaperCreateLocalizedName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetPrinterID()
 *  
 *  Summary:
 *    Returns the printerID of the printer for which the paper corresponds.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the printer ID for this paper.
 *    
 *    printerID:
 *      If successful, noErr is returned and *printerID is set to the
 *      ID of the printer for which the paper corresponds. The returned
 *      result may be NULL. 
=======
function PMPrinterGetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterSetOutputResolution()
 *  
 *  Summary:
 *    Set the printer hardware output resolution in the print settings
 *    for the supplied printer. 
=======
function PMPrintSettingsCreateWithDataRepresentation( data: CFDataRef; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPrintSettingsCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


  
{
 *  PMGetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCollate( printSettings: PMPrintSettings; var collate: Boolean ): OSStatus; external name '_PMGetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMGetCopies()
>>>>>>> origin/cpstrnew
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterSetOutputResolution configures the print settings to the closest
 *    resolution setting that can be used for the destination print job. Note
 *    that not all printers allow control of their resolution setting.
 *
 *    This function is rarely used since most applications do not set the output 
 *    resolution but instead use the setting supplied by the user in the print dialog.
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings in which to set the hardware resolution.
 *    
 *    resolutionP:
 *      A pointer to the PMResolution to use to set the hardware output resolution.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPaperGetPrinterID( paper: PMPaper; var printerID: CFStringRef ): OSStatus; external name '_PMPaperGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperIsCustom()
 *  
 *  Summary:
 *    Returns true if the paper is a custom paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Determines if this is a custom paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperIsCustom( paper: PMPaper ): Boolean; external name '_PMPaperIsCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
function PMPrinterSetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; const (*var*) resolutionP: PMResolution ): OSStatus; external name '_PMPrinterSetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrinterGetLanguageInfo()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLanguageInfo( printer: PMPrinter; var info: PMLanguageInfo ): OSStatus; external name '_PMPrinterGetLanguageInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCopies( printSettings: PMPrintSettings; var copies: UInt32 ): OSStatus; external name '_PMGetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDuplex( printSettings: PMPrintSettings; var duplexSetting: PMDuplexMode ): OSStatus; external name '_PMGetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMGetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetFirstPage( printSettings: PMPrintSettings; var first: UInt32 ): OSStatus; external name '_PMGetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew

(*
#pragma mark
#pragma mark PDF Workflow
#pragma mark
*)
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMWorkflowCopyItems()
 *  
 *  Summary:
 *    Hand back an array of dictionaries describing the PDF Workflow
 *    items installed on the system
 *  
 *  Parameters:
 *    
 *    workflowItems:
 *      If this function returns without error then *'workflowItems'
 *      will be filled in with a reference to an array. It is the
 *      caller's responsability to release the array when done with it.
 *      Each element in the array describes a PDF Workflow item or a
 *      folder holding workflow items. A dictionary describing a
 *      workflow item has, at least, the following keys and values:
 *      displayName - The user's diaplayable name for the workflow item
 *      itemURL - A CFURLRef pointing to the workflow item. A
 *      dictionary describing a workflow folder has at least the
 *      following keys: displayName - The user's diaplayable name for
 *      the workflow item folderURL - A CFURLRef pointing to the
 *      folder. items - A CFArrayRef describing the workflow items in
 *      the folder. If this function returns a non-zero error code then
 *      *'workflowItems' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowCopyItems( var workflowItems: CFArrayRef ): OSStatus; external name '_PMWorkflowCopyItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithOptions()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    title:
 *      The user displayable name of the document.
 *    
 *    options:
 *      A string of CUPS style key-value pairs that may be passed to
 *      the PDF Workflow item. This parameter can be NULL in which case
 *      an empty string of options is used.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
=======
 *  PMPrinterGetMakeAndModelName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetMakeAndModelName( printer: PMPrinter; var makeAndModel: CFStringRef ): OSStatus; external name '_PMPrinterGetMakeAndModelName';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetMimeTypes()
 *  
 *  Summary:
 *    Return the array of mime type supported by the printer for a
 *    given set of print settings.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job. The part of the print
 *      settings that effects the available mime type is the
 *      destination. This parameter can be NULL.
 *    
 *    mimeTypes:
 *      If this function returns without error then *'mimeTypes' is
 *      filled in with a reference to an array of CFStrings. Each
 *      CFString names a mime type supported by the printer with the
 *      specified print settings. The caller must not release this
 *      reference without first doing a retain. If this function
 *      returns an error then 'mimeTypes' will be set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMWorkflowSubmitPDFWithOptions( workflowItem: CFURLRef; title: CFStringRef; options: ConstCStringPtr; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithOptions';
=======
function PMPrinterGetMimeTypes( printer: PMPrinter; settings: PMPrintSettings; var mimeTypes: CFArrayRef ): OSStatus; external name '_PMPrinterGetMimeTypes';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMWorkflowSubmitPDFWithSettings()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    settings:
 *      The prints settings to apply to the PDF.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowSubmitPDFWithSettings( workflowItem: CFURLRef; settings: PMPrintSettings; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Job Submission
#pragma mark
*)
{
 *  PMPrinterPrintWithProvider()
 *  
 *  Summary:
 *    Submit print data to a specified printer.
 *  
 *  Discussion:
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. This parameter can not
 *      be NULL. Use PMPrinterPrintWithFile() if auto-typing is desired.
 *    
 *    provider:
 *      The data provider that supplies the print data.
=======
 *  PMPrinterGetName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetName( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetName';
=======
 *  PMGetLastPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetLastPage( printSettings: PMPrintSettings; var last: UInt32 ): OSStatus; external name '_PMGetLastPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetPageRange()
 *  
 *  Summary:
 *    Obtains the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The default page range is 1 - (all pages). The page range is
 *    something that is set by the application. It is NOT the first and
 *    last page to print. It serves as limits for setting the first and
 *    last page.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageRange( printSettings: PMPrintSettings; var minPage: UInt32; var maxPage: UInt32 ): OSStatus; external name '_PMGetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrintSettingsGetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetJobName( printSettings: PMPrintSettings; var name: CFStringRef ): OSStatus; external name '_PMPrintSettingsGetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  PMPrintSettingsGetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings to retrieve the value from
 *    
 *    key:
 *      The key to look for
 *    
 *    value:
 *      The return value. Its either the value for the key or NULL
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetValue( printSettings: PMPrintSettings; key: CFStringRef; var value: CFTypeRef ): OSStatus; external name '_PMPrintSettingsGetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)



{
 *  PMSetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCollate( printSettings: PMPrintSettings; collate: Boolean ): OSStatus; external name '_PMSetCollate';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterGetPaperList()
 *  
 *  Summary:
 *    Returns the list of papers available for a given printer.
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the paper list for this printer.
 *    
 *    paperList:
 *      If successful noErr is returned and *paperList is a CFArray of
 *      PMPapers representing the list of papers available for the
 *      printer.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrinterPrintWithProvider( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; provider: CGDataProviderRef ): OSStatus; external name '_PMPrinterPrintWithProvider';
=======
function PMPrinterGetPaperList( printer: PMPrinter; var paperList: CFArrayRef ): OSStatus; external name '_PMPrinterGetPaperList';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterPrintWithFile()
 *  
 *  Summary:
 *    Submit a file for printing to a specified printer.
 *  
 *  Discussion:
 *
 *    One reason this function may fail is if the specified printer can
 *    not handle the file's mime type. Use PMPrinterGetMimeTypes() to
 *    check whether a mime type is supported.
 *  
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. If this parameter is
 *      NULL then the supplied file will be auto-typed.
 *    
 *    fileURL:
 *      A file URL specifying the file to be printed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterPrintWithFile( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; fileURL: CFURLRef ): OSStatus; external name '_PMPrinterPrintWithFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPrinterWritePostScriptToURL()
 *
 *  Summary:
 *    Convert an input file of the specified mimeType to printer ready PostScript for the destination printer.
 *
 *  Discussion:
 *
 *    The conversion of the input file to PostScript is performed before this function returns. This can take a significant
 *    amount of time for longer documents. The caller of PMPrinterWritePostScriptToURL may want to perform this operation
 *    on a thread other than the main application thread or fork a separate process for this purpose.
 *
 *    One reason this function may fail is if conversion from the input mimeType to PostScript cannot be performed.
 *
 *
 *  Parameters:
 *
 *    printer:
 *      The printer for which the printer ready PostScript will be generated.
 *
 *    settings:
 *      The print settings for the print job.
 *
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed.
 *
 *    mimeType:
 *      The mime type of the file to be printed. If NULL, the file is auto-typed.
 *
 *    sourceFileURL:
 *      A file URL specifying the input file to be converted to printer ready PostScript data. Only file based URLs
 *      are supported.
 *
 *    destinationFileURL:
 *      A file URL specifying the destination file to be created. If the file already exists it will be overwritten. Only
 *      file based URLs are supported.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterWritePostScriptToURL( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; sourceFileURL: CFURLRef; destinationFileURL: CFURLRef ): OSStatus; external name '_PMPrinterWritePostScriptToURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsToOptions()
 *  
 *  Summary:
 *    Convert print settings to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'ed C string
 *      describing the passed in print settings. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsToOptions( settings: PMPrintSettings; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrintSettingsToOptionsWithPrinterAndPageFormat()
 *  
 *  Summary:
 *    Convert a print settings and page format to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string. Must not be NULL.
 *    
 *    printer:
 *      The printer to use for converting the print settings. Must not be NULL.
 *
 *    format:
 *      The page format for use in creating the cups options. Can be NULL.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'd C string
 *      describing the passed in print settings and format. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
=======
 *  PMPrinterGetState()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetState( printer: PMPrinter; var state: PMPrinterState ): OSStatus; external name '_PMPrinterGetState';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsDefault()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsDefault( printer: PMPrinter ): Boolean; external name '_PMPrinterIsDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsFavorite()
 *  
 *  Summary:
 *    Return true if the printer is in the user's favorite printer list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsFavorite( printer: PMPrinter ): Boolean; external name '_PMPrinterIsFavorite';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsPostScriptCapable()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsPostScriptCapable( printer: PMPrinter ): Boolean; external name '_PMPrinterIsPostScriptCapable';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{!
 * @function	PMPrinterIsPostScriptPrinter
 * @abstract	Set *isPSPrinter true if the printer is a PostScript printer.
 *
 * @discussion	A PostScript printer is one whose driver takes PostScript directly.
 *
 *  Availability:
>>>>>>> graemeg/cpstrnew
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrintSettingsToOptionsWithPrinterAndPageFormat( settings: PMPrintSettings; printer: PMPrinter; pageFormat: PMPageFormat; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptionsWithPrinterAndPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark PrinterCommandSupport
#pragma mark
*)

{
*  PMPrinterSendCommand()
*
*  Summary:
*    Submit a command file containing the specified string data to a print queue.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    commandString:
*      The contents of the command file as a CFStringRef.
*
*    jobTitle:
*      The title of the job associated with the command file. If NULL,
*      a job title will be automatically generated but it may not be
*      meaningful to a user.
*
*    options:
*      A dictionary containing options that apply to the job.
*      This parameter should be NULL; it is reserved for future expansion.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}
function PMPrinterSendCommand( printer: PMPrinter; commandString: CFStringRef; jobTitle: CFStringRef; options: CFDictionaryRef ): OSStatus; external name '_PMPrinterSendCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{
*  PMPrinterCopyState()
*
*  Summary:
*    Copies the current "marker-change-time", "marker-colors",
*    "marker-high-levels", "marker-levels", "marker-low-levels",
*    "marker-message", "marker-names", "marker-types", "printer-state",
*    "printer-state-change-time", "printer-state-message", and
*    "printer-state-reasons" values for a print queue.  These attributes are
*    defined in the "CUPS Implementation of IPP" specification at:
*
*        http://www.cups.org/documentation.php/spec-ipp.html
*        http://localhost:631/help/spec-ipp.html
*
*    Each attribute is returned using the corresponding CoreFoundation
*    data type:
*
*        Attribute                    Type
*        ---------------------------  ------------------------
*        "marker-change-time"         CFDateRef
*        "marker-colors"              CFArrayRef + CFStringRef
*        "marker-high-levels"         CFArrayRef + CFNumberRef
*        "marker-levels"              CFArrayRef + CFNumberRef
*        "marker-low-levels"          CFArrayRef + CFNumberRef
*        "marker-message"             CFStringRef
*        "marker-names"               CFArrayRef + CFStringRef
*        "marker-types"               CFArrayRef + CFStringRef
*        "printer-state"              CFNumberRef
*        "printer-state-change-time"  CFDateRef
*        "printer-state-message"      CFStringRef
*        "printer-state-reasons"      CFArrayRef + CFStringRef
*
*  Localization Considerations:
*
*    The "marker-message" and "printer-state-message" attributes may be
*    localized by the printer driver using the language of the last processed
*    print job.
*
*    The "marker-names" and "printer-state-reasons" attributes are typically
*    English-language keywords that may be localized using the printer's PPD
*    file and CUPS ppdLocalizeMarkerName and ppdLocalizeIPPReason APIs,
*    respectively.
*
*    All other attributes are predefined keywords or values that are not
*    localized.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    stateDict:
*      A pointer to a CFDictionaryRef variable to hold the values.
*      You must release the dictionary when you are done using it.
*      The dictionary returned is not mutable.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}
=======
function PMPrinterIsPostScriptPrinter( printer: PMPrinter; var isPSPrinter: Boolean ): OSStatus; external name '_PMPrinterIsPostScriptPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew

function PMPrinterCopyState( printer: PMPrinter; var stateDict: CFDictionaryRef ): OSStatus; external name '_PMPrinterCopyState';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

(*
#pragma mark
#pragma mark PPD related
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMCopyAvailablePPDs()
 *  
 *  Summary:
 *    Hand back the list of PPDs in the specified PPD domain.
 *  
 *  Parameters:
 *    
 *    domain:
 *      The domain to search for PPDs.
 *    
 *    ppds:
 *      If this function completes without error, *'ppds' is set to an
 *      array of CFURLs. Each CFURL specifies the location of a PPD
 *      file or a compressed PPD file. The caller is responsible for
 *      releasing the array. If this function returns a non-zero error
 *      code then *'ppds' is set to NULL.
=======
 *  PMPrinterIsRemote()
 *  
 *  Summary:
 *    Hand back a boolean indicating whether the printer is hosted by remote print server.
 *  
 *  Discussion:
 *    If on return *isRemoteP is true, the print queue represents a printer hosted and
 *    managed by a remote print server.
 *
 *    If on return *isRemoteP is false, the print queue represents a directly connected
 *    printer, a network printer, or a remote printer that is locally managed. Consult
 *    the queue's device URI to determine the type of connection that is used to communicate
 *    with the printer.
 *
 *    Whether a printer is remote is derived from the CUPS printer-type attribute for the print queue.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMCopyAvailablePPDs( domain: PMPPDDomain; var ppds: CFArrayRef ): OSStatus; external name '_PMCopyAvailablePPDs';
=======
function PMPrinterIsRemote( printer: PMPrinter; var isRemoteP: Boolean ): OSStatus; external name '_PMPrinterIsRemote';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMCopyLocalizedPPD()
 *  
 *  Summary:
 *    Hand back a reference to a localized PPD.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A PPD reference. Typically this is a CFURLRef returned from
 *      PMCopyAvailablePPDs().
 *    
 *    localizedPPD:
 *      If this function completes without error, *'localizedPPD' will
 *      be set to a CFURLRef referencing the PPD that should be used
 *      given the current user's language preferences. If this function
 *      returns an error then *'localizedPPD' will be set to to NULL.
=======
 *  PMSetCopies()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCopies( printSettings: PMPrintSettings; copies: UInt32; lock: Boolean ): OSStatus; external name '_PMSetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetDuplex( printSettings: PMPrintSettings; duplexSetting: PMDuplexMode ): OSStatus; external name '_PMSetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetFirstPage( printSettings: PMPrintSettings; first: UInt32; lock: Boolean ): OSStatus; external name '_PMSetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetLastPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetLastPage( printSettings: PMPrintSettings; last: UInt32; lock: Boolean ): OSStatus; external name '_PMSetLastPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetPageRange()
 *  
 *  Summary:
 *    Sets the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The first and last page are immediately clipped to the new range.
 *    You may pass kPMPrintAllPages for the maxPage value to specified
 *    that all pages are available for printing.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPageRange( printSettings: PMPrintSettings; minPage: UInt32; maxPage: UInt32 ): OSStatus; external name '_PMSetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsSetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetJobName( printSettings: PMPrintSettings; name: CFStringRef ): OSStatus; external name '_PMPrintSettingsSetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrintSettingsSetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings in which to set the value
 *    
 *    key:
 *      The key to store the value in
 *    
 *    value:
 *      The value to store in the key. If NULL, any existing setting item with
 *      the specified key is removed.
 *    
 *    locked:
 *      A boolean value indicating whether the item being set should be
 *      locked. It is strongly recommended to pass false.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetValue( printSettings: PMPrintSettings; key: CFStringRef; value: CFTypeRef; locked: Boolean ): OSStatus; external name '_PMPrintSettingsSetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
  
  
{
 *  PMPrintSettingsCopyAsDictionary()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Represent these print settings as a dictionary.
 *    
 *    settingsDictionary:
 *      On exit, if successful *'settingsDictionary' will contain a reference to
 *      a CFDictionary describing the print settings. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'settingsDictionary' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCopyAsDictionary( printSettings: PMPrintSettings; var settingsDictionary: CFDictionaryRef ): OSStatus; external name '_PMPrintSettingsCopyAsDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrintSettingsCopyKeys()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Return the keys for items in this print settings.
 *    
 *    settingsKeys:
 *      On exit, if successful *'settingsKeys' will contain a reference to
 *      a CFArray describing the item keys in the print settings. Each of these
 *		keys may be passed to PMPrintSettingsGetValue() to obtain a value.
 *		The caller is responsible for releasing this reference. If this call returns
 *      an error, then *'settingsKeys' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCopyKeys( printSettings: PMPrintSettings; var settingsKeys: CFArrayRef ): OSStatus; external name '_PMPrintSettingsCopyKeys';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark -
#pragma mark Printer
#pragma mark
*)

{!
 *
 * PMCreateGenericPrinter
 *  Summary:
 *	Creates a generic PMPrinter
 *
 *  Parameters:
 *		printer:
 *		On return *printer contains the generic priner
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMCreateGenericPrinter( var printer: PMPrinter ): OSStatus; external name '_PMCreateGenericPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMServerCreatePrinterList()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Hand back an array of PMPrinter objects that represents the list of printers
 *    known to the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server for which you want to obtain the list of printers.
 *      Use kPMServerLocal for the local print server.
 *    
 *    printerList:
 *      On return, *printerList contains the array of PMPrinter objects returned. The
 *	caller is responsible for releasing this array. On error, *printerList is NULL.
 *    
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyLocalizedPPD( ppd: CFURLRef; var localizedPPD: CFURLRef ): OSStatus; external name '_PMCopyLocalizedPPD';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
 *  PMPrinterSetDefault()
 *  
 *  Summary:
 *    Set the default printer for the current user.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer to set as the default printer.
 *  
 *  Special considerations:
 *      It is not typical for an application to set the current default printer
 *      for the user; the printing system itself takes care of managing the default printer.
 *      This function should be used only in rare circumstances.
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMServerCreatePrinterList( server: PMServer; var printerList: CFArrayRef ): OSStatus; external name '_PMServerCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMServerLaunchPrinterBrowser()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Launch the printer browser, browsing the printers available for
 *    the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server whose available printers the browser should browse.
 *      Pass kPMServerLocal for the local print server.
 *    
 *    options:
 *      A CFDictionaryRef specifying how the browser should be presented. 
 *      Passing NULL presents the browser in the default fashion. 
 *
 *  Discussion:	The local print server allows its printers to be browsed but
 *		not all print servers do. Passing in a server whose printers cannot be
 *      browsed returns the error code kPMInvalidParameter.
 *    
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrinterSetDefault( printer: PMPrinter ): OSStatus; external name '_PMPrinterSetDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function PMServerLaunchPrinterBrowser( server: PMServer; options: CFDictionaryRef ): OSStatus; external name '_PMServerLaunchPrinterBrowser';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew

(*
#pragma mark
#pragma mark Preset
#pragma mark
*)
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMCopyPPDData()
 *  
 *  Summary:
 *    Hand back the uncompressed PPD data for a PPD or compressed PPD
 *    file.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A reference to a PPD or compressed PPD file. This reference is
 *      usually obtained from PMCopyAvailablePPDs() or from
 *      PMCopyLocalizedPPD().
 *    
 *    data:
 *      If this function completes without error then *'data' is set to
 *      reference the uncompressed PPD data from the PPD file. If this
 *      function returns a non-zero error then *'data is set to NULL.
=======
 *  PMPresetCopyName()
 *  
 *  Summary:
 *    Hand back a copy of the localized name for the specified preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      The preset whose name is needed.
 *    
 *    name:
 *      On exit, if this routine succeeds, *'name' is filled in with a
 *      reference to a localized string with the preset's name. If this
 *      routine fails, then *'name' is set to NULL.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMCopyPPDData( ppd: CFURLRef; var data: CFDataRef ): OSStatus; external name '_PMCopyPPDData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
function PMPresetCopyName( preset: PMPreset; var name: CFStringRef ): OSStatus; external name '_PMPresetCopyName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
 *  PMPrinterCreateFromPrinterID()
 *  
 *  Summary:
 *    Create a PMPrinter instance from the supplied printerID.
 *  
 *  Discussion:
 *    Returns a PMPrinter instance for the printer whose ID is
 *    printerID. The caller is responsible for releasing this instance
 *    with PMRelease. The PMPrinter instance returned will be NULL if
 *    there is no printer available which corresponds to the supplied
 *    printerID.
 *  
 *  Parameters:
 *    
 *    printerID:
 *      The printerID for the printer for which you want the PMPrinter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCreateFromPrinterID( printerID: CFStringRef ): PMPrinter; external name '_PMPrinterCreateFromPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

>>>>>>> origin/cpstrnew

(*
#pragma mark
#pragma mark EPS related
#pragma mark
*)
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  PMCGImageCreateWithEPSDataProvider()
 *  
 *  Summary:
 *    Create an image reference that references both the PostScript
 *    contents of an EPS file and a preview (proxy) image for that EPS
 *    file.
 *  
 *  Discussion:
 *    For OS X 10.1.0, this function ignores the passed in data
 *    provider. The passed in image reference is retained and then
 *    returned. For 10.1.1 and later, then the data provider is used
 *    and the returned image reference is different than the passed in
 *    image reference, so please be careful with your use of these
 *    references. It is likely that the data will not be read from the
 *    EPS data provider until well after this function returns. The
 *    caller should be careful not to free the underlying EPS data
 *    until the provider's release routine is invoked. Similarly the
 *    preview image's data may be needed long after you think it should
 *    be. Do not free the image data until the image data provider's
 *    release data function has been called. To make sure these data
 *    providers are properly reference counted, release your reference
 *    the EPS data provider and on the EPS image preview when they are
 *    no longer needed by your application. For Mac OS X 10.2 and
 *    later, the contents of the EPS provider at the time of this call
 *    can be dumped to a file if you first do the following, BEFORE
 *    running your application. 
 *
 *	  From the command line in terminal:
 *		defaults write NSGlobalDomain com.apple.print.eps.testProvider /tmp/dump.eps
 *    
 *	  causes a dump of the EPS data into a file /tmp/dump.eps.
 *  
 *  Parameters:
 *    
 *    epsDataProvider:
 *      A Core Graphics data provider that can supply the PostScript
 *      contents of the EPS file. Post OS X 10.1, there will be some
 *      checking done on the EPS data provided to the
 *      PMCGImageCreateWithEPSDataProvider() call. It is important that
 *      the EPS data begin with the EPSF required header and bounding
 *      box DSC comments.
 *    
 *    epsPreview:
 *      A Core Graphics image reference to the proxy image for the EPS
 *      file. When the image reference result of this function is
 *      rendered on screen or printed to a printer that can not render
 *      PostScript this proxy image is drawn instead.
 *  
 *  Result:
 *    an image reference capable of rendering either the EPS content or
 *    the proxy image depending upon the capabilities of the targeted
 *    context.
=======
 *  PMPrinterCopyDescriptionURL()
 *  
 *  Summary:
 *    Hand back a URL to the printer's PostScript Printer Description (PPD) file in fileURL.
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose PPD will be returned.
 *    
 *    descriptionType:
 *      The type of description desired. Only kPMPPDDescriptionType is currently supported.
 *    
 *    fileURL:
 *      A pointer to storage for a CFURL reference that will be returned. The caller is responsible
 *      for releasing the reference returned. If this call returns an error, then *fileURL will
 *      be set to NULL.
 *
 *  Discussion:
 *    Only descriptionType of kPMPPDDescriptionType is supported, otherwise kPMInvalidParameter
 *    is returned.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> graemeg/cpstrnew
=======
 *  PMPresetCreatePrintSettings()
 *  
 *  Summary:
 *    Create a print settings conforming to the specified print
 *    settings preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A preset specifying a set of initial print settings.
 *    
 *    session:
 *      A valid print session.
 *    
 *    printSettings:
 *      On exit, *'printSettings' is set to a newly created print
 *      settings that contains the settings specified by 'preset'. The
 *      caller is responsible for calling PMRelease when the print
 *      settings are no longer needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPresetCreatePrintSettings( preset: PMPreset; session: PMPrintSession; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPresetCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPresetGetAttributes()
 *  
 *  Summary:
 *    Hand back the meta-data describing a given preset.
 *  
 *  Discussion:
 *    Each preset has associated with it a dictionary containing
 *    meta-data. The meta-data provides the preset's id, the preset's
 *    localized names, and descriptions of the environment for which
 *    the preset it intended.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A print settings preset as obtained from PMPrinterCopyPresets().
 *    
 *    attributes:
 *      On exit, *'attributes' is set to reference a dictionary
 *      containing the preset's meta-data. The caller is responsible
 *      for retaining this reference if it is to be used beyond the
 *      lifetime of 'preset'. If this function fails, returning a
 *      non-zero error code, then *'attributes' is set to NULL.
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyDescriptionURL( printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef ): OSStatus; external name '_PMPrinterCopyDescriptionURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyDeviceURI()
 *  
 *  Summary:
 *    Hand back the URI of the printer's device.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose device URI is to be retrieved.
 *    
 *    deviceURI:
 *      On exit, if successful *'deviceURI' will contain a reference to
 *      a CFURL describing the printer's device. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'deviceURI' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyDeviceURI( printer: PMPrinter; var deviceURI: CFURLRef ): OSStatus; external name '_PMPrinterCopyDeviceURI';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyHostName()
>>>>>>> origin/cpstrnew
 *  
 *  Summary:
 *    Hand back the host name of the print server hosting the printer's print queue.
 *
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPresetGetAttributes( preset: PMPreset; var attributes: CFDictionaryRef ): OSStatus; external name '_PMPresetGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Paper
#pragma mark
*)
{
 *  PMGetPageFormatPaper()
 *  
 *  Summary:
 *    Returns the paper associated with a pageformat.
 *  
 *  Parameters:
 *    
 *    format:
 *      Obtain the paper for this pageformat.
 *    
 *    paper:
 *      If successful noErr is returned and *paper will contain a
 *      PMPaper object describing the current paper associated with the
 *      pageformat.
=======
function PMPrinterCopyHostName( printer: PMPrinter; var hostNameP: CFStringRef ): OSStatus; external name '_PMPrinterCopyHostName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterCopyPresets()
 *  
 *  Summary:
 *    Provides a list of print settings presets for the specified
 *    printer.
 *  
 *  Discussion:
 *    A printer may have associated with it a list of preset settings.
 *    Each setting is optimized for a particular printing situation.
 *    This function returns all of the presets for a given printer. To
 *    obtain more information about a particular preset see
 *    PMPresetGetAttributes().
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the presets for this printer.
 *    
 *    presetList:
 *      On exit, *'presetList' is set to reference an array of presets.
 *      The caller must call CFRelease when it no longer needs the
 *      array. Each element of the array is a PMPreset. If this
 *      function fails, returning a non-zero error code, then
 *      *'presetList' will be set to NULL.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMGetPageFormatPaper( format: PMPageFormat; var paper: PMPaper ): OSStatus; external name '_PMGetPageFormatPaper';
=======
function PMPrinterCopyPresets( printer: PMPrinter; var presetList: CFArrayRef ): OSStatus; external name '_PMPrinterCopyPresets';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPaperCreateCustom()
 *  
 *  Summary:
 *    Create a new custom paper instance.
=======
 *  PMPrinterGetCommInfo()
 *  
 *  Summary:
 *    Provides information about the comm channel characteristics for
 *    the printer.
 *  
 *  Discussion:
 *    This function is typically relevant only to PostScript capable
 *    printers. All PostScript printers, regardless of what
 *    communications channel is used to send data to them, support data
 *    in the range 0x20 - 0x7F. Many comm channels can support data
 *    outside this range. The Boolean returned in *supportsTransparentP
 *    indicates whether the comm channel to this printer supports bytes
 *    in the range 0x0 to 0x1F. The Boolean returned in
 *    *supportsEightBitP indicates whether the comm channel to this
 *    printer supports bytes with the high bit set, i.e. bytes in the
 *    range 0x80 - 0xFF.
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
<<<<<<< HEAD
 *      The new paper size is appropriate for this printer.
 *    
 *    id:
 *      A unique identifier for this paper type.
 *    
 *    name:
 *      The name to display to the user for this paper type.
 *    
 *    width:
 *      The width, in points, of the paper.
 *    
 *    height:
 *      The height, in points, of the paper.
 *    
 *    margins:
 *      The unprintable margins on the paper.
 *    
 *    paperP:
 *      if this function is successful, returning noErr, then *'paperP'
 *      is set to be a reference to a newly created PMPaper instance.
 *      The caller is responsible for calling PMRelease when the
 *      instance is no longer needed. If this functions fails, it will
 *      return a non-zero error and set *'paperP' to NULL.
 *  
 *  Discussion:
 *		This function creates a new custom paper instance. To obtain one of the available
 *		built-in paper sizes for a given printer, use PMPrinterGetPaperList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperCreateCustom( printer: PMPrinter; id: CFStringRef; name: CFStringRef; width: Float64; height: Float64; const (*var*) margins: PMPaperMargins; var paperP: PMPaper ): OSStatus; external name '_PMPaperCreateCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetWidth()
 *  
 *  Summary:
 *    Returns the width for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the width for this paper.
 *    
 *    paperWidth:
 *      If successful noErr is returned and *paperWidth is set to the
 *      width of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetWidth( paper: PMPaper; var paperWidth: Float64 ): OSStatus; external name '_PMPaperGetWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetHeight()
 *  
 *  Summary:
 *    Returns the height for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the height for this paper.
 *    
 *    paperHeight:
 *      If successful noErr is returned and *paperHeight is set to the
 *      height of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetHeight( paper: PMPaper; var paperHeight: Float64 ): OSStatus; external name '_PMPaperGetHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetMargins()
 *  
 *  Summary:
 *    Returns the margins for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the margin information for this paper.
 *    
 *    paperMargins:
 *      If successful noErr is returned and *paperMargins is set to the
 *      margins of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetMargins( paper: PMPaper; var paperMargins: PMPaperMargins ): OSStatus; external name '_PMPaperGetMargins';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperGetID()
 *  
 *  Summary:
 *    Returns the id for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the id for this paper.
 *    
 *    paperID:
 *      If successful noErr is returned and *paperID is set to the id
 *      of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetID( paper: PMPaper; var paperID: CFStringRef ): OSStatus; external name '_PMPaperGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetName()
 *  
 *  Summary:
 *    Returns the name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the name for this paper.
 *    
 *    paperName:
 *      If successful noErr is returned and *paperName is set to the
 *      name of the paper. The returned paper name is that appropriate
 *      to display to a user.
=======
 *      Obtain the comm information for this printer.
 *    
 *    supportsControlCharRangeP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range
 *      0x0 - 0x1F
 *    
 *    supportsEightBitP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range 
 *      0x80 - 0xFF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetCommInfo( printer: PMPrinter; var supportsControlCharRangeP: Boolean; var supportsEightBitP: Boolean ): OSStatus; external name '_PMPrinterGetCommInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetID()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetID( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetLocation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLocation( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetDriverCreator()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDriverCreator( printer: PMPrinter; var creator: OSType ): OSStatus; external name '_PMPrinterGetDriverCreator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetDriverReleaseInfo()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDriverReleaseInfo( printer: PMPrinter; var release: VersRec ): OSStatus; external name '_PMPrinterGetDriverReleaseInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetPrinterResolutionCount()
 *  
 *  Summary:
 *    Provides the number of available hardware resolution settings a printer supports. 
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the resolution count for this printer.
 *    
 *    countP:
 *      Storage for the returned count indicating the number of available hardware
 *      resolution settings for the specified printer.
 *
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetPPDPaperName()
 *  
 *  Summary:
 *    Returns the PPD name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the PPD name for this paper.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      PPD name of the paper. The returned result may be NULL.
 *  
 *  Discussion:
 *      The Mac OS X printing system uses a PostScript Printer Description (PPD)
 *      file to describe a given printer and print queue for that printer. The
 *      PPD name is the name that uniquely identifies a given paper
 *      to the printer to which the paper corresponds. 
 *    
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetPPDPaperName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetPPDPaperName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPaperCreateLocalizedName()
 *  
 *  Summary:
 *    Returns the localized name for a given paper. 
 *    Use of PMPaperCreateLocalizedName
 *    is recommended instead of PMPaperGetName.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the localized name for this paper.
 *    
 *    printer:
 *      The printer for which the localization should be performed.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      localized name of the paper. The returned paper name is that appropriate
 *      to display to a user. The returned result may be NULL.
 *      If non-NULL, the caller is responsible for releasing the name returned. 
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetPrinterResolutionCount( printer: PMPrinter; var countP: UInt32 ): OSStatus; external name '_PMPrinterGetPrinterResolutionCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrinterGetIndexedPrinterResolution()
 *  
 *  Summary:
 *    Together with PMPrinterGetPrinterResolutionCount, allows iterating over the
 *    hardware resolution settings a printer supports. 
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer of interest.
 *    
 *    index:
 *      The index of the resolution to return. The minimum value for index is 1 and
 *      the maximum value of index is the count returned by PMPrinterGetPrinterResolutionCount.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetIndexedPrinterResolution( printer: PMPrinter; index: UInt32; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetIndexedPrinterResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetOutputResolution()
 *  
 *  Summary:
 *    Obtain the printer hardware output resolution for the supplied
 *    printer and print settings. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterGetOutputResolution returns the best guess as to what printer
 *    resolution setting will be used for the destination print job. If the
 *    resolution setting cannot be reliably determined this function returns
 *    kPMKeyNotFound. 
 *  
 *    Most applications do not need to use this function since they draw the same
 *    content regardless of the destination device. For those few applications that
 *    do adjust their drawing based on the output device, they should only do so
 *    when the print job destination is kPMDestinationPrinter or kPMDestinationFax.
 *  
 *    This call should be used after displaying the print dialog to the user so
 *    that it correctly reflects settings changes performed prior to printing.
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings from which to obtain the printer hardware output resolution.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPaperCreateLocalizedName( paper: PMPaper; printer: PMPrinter; var paperName: CFStringRef ): OSStatus; external name '_PMPaperCreateLocalizedName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetPrinterID()
 *  
 *  Summary:
 *    Returns the printerID of the printer for which the paper corresponds.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the printer ID for this paper.
 *    
 *    printerID:
 *      If successful, noErr is returned and *printerID is set to the
 *      ID of the printer for which the paper corresponds. The returned
 *      result may be NULL. 
=======
function PMPrinterGetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterSetOutputResolution()
 *  
 *  Summary:
 *    Set the printer hardware output resolution in the print settings
 *    for the supplied printer. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterSetOutputResolution configures the print settings to the closest
 *    resolution setting that can be used for the destination print job. Note
 *    that not all printers allow control of their resolution setting.
 *
 *    This function is rarely used since most applications do not set the output 
 *    resolution but instead use the setting supplied by the user in the print dialog.
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings in which to set the hardware resolution.
 *    
 *    resolutionP:
 *      A pointer to the PMResolution to use to set the hardware output resolution.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPaperGetPrinterID( paper: PMPaper; var printerID: CFStringRef ): OSStatus; external name '_PMPaperGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperIsCustom()
 *  
 *  Summary:
 *    Returns true if the paper is a custom paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Determines if this is a custom paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperIsCustom( paper: PMPaper ): Boolean; external name '_PMPaperIsCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
function PMPrinterSetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; const (*var*) resolutionP: PMResolution ): OSStatus; external name '_PMPrinterSetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrinterGetLanguageInfo()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLanguageInfo( printer: PMPrinter; var info: PMLanguageInfo ): OSStatus; external name '_PMPrinterGetLanguageInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew

(*
#pragma mark
#pragma mark PDF Workflow
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMWorkflowCopyItems()
 *  
 *  Summary:
 *    Hand back an array of dictionaries describing the PDF Workflow
 *    items installed on the system
 *  
 *  Parameters:
 *    
 *    workflowItems:
 *      If this function returns without error then *'workflowItems'
 *      will be filled in with a reference to an array. It is the
 *      caller's responsability to release the array when done with it.
 *      Each element in the array describes a PDF Workflow item or a
 *      folder holding workflow items. A dictionary describing a
 *      workflow item has, at least, the following keys and values:
 *      displayName - The user's diaplayable name for the workflow item
 *      itemURL - A CFURLRef pointing to the workflow item. A
 *      dictionary describing a workflow folder has at least the
 *      following keys: displayName - The user's diaplayable name for
 *      the workflow item folderURL - A CFURLRef pointing to the
 *      folder. items - A CFArrayRef describing the workflow items in
 *      the folder. If this function returns a non-zero error code then
 *      *'workflowItems' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowCopyItems( var workflowItems: CFArrayRef ): OSStatus; external name '_PMWorkflowCopyItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithOptions()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    title:
 *      The user displayable name of the document.
 *    
 *    options:
 *      A string of CUPS style key-value pairs that may be passed to
 *      the PDF Workflow item. This parameter can be NULL in which case
 *      an empty string of options is used.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
=======
 *  PMPrinterGetMakeAndModelName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetMakeAndModelName( printer: PMPrinter; var makeAndModel: CFStringRef ): OSStatus; external name '_PMPrinterGetMakeAndModelName';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetMimeTypes()
 *  
 *  Summary:
 *    Return the array of mime type supported by the printer for a
 *    given set of print settings.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job. The part of the print
 *      settings that effects the available mime type is the
 *      destination. This parameter can be NULL.
 *    
 *    mimeTypes:
 *      If this function returns without error then *'mimeTypes' is
 *      filled in with a reference to an array of CFStrings. Each
 *      CFString names a mime type supported by the printer with the
 *      specified print settings. The caller must not release this
 *      reference without first doing a retain. If this function
 *      returns an error then 'mimeTypes' will be set to NULL.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMWorkflowSubmitPDFWithOptions( workflowItem: CFURLRef; title: CFStringRef; options: ConstCStringPtr; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithOptions';
=======
function PMPrinterGetMimeTypes( printer: PMPrinter; settings: PMPrintSettings; var mimeTypes: CFArrayRef ): OSStatus; external name '_PMPrinterGetMimeTypes';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMWorkflowSubmitPDFWithSettings()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    settings:
 *      The prints settings to apply to the PDF.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowSubmitPDFWithSettings( workflowItem: CFURLRef; settings: PMPrintSettings; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Job Submission
#pragma mark
*)
{
 *  PMPrinterPrintWithProvider()
 *  
 *  Summary:
 *    Submit print data to a specified printer.
 *  
 *  Discussion:
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. This parameter can not
 *      be NULL. Use PMPrinterPrintWithFile() if auto-typing is desired.
 *    
 *    provider:
 *      The data provider that supplies the print data.
=======
 *  PMPrinterGetName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetName( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetPaperList()
 *  
 *  Summary:
 *    Returns the list of papers available for a given printer.
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the paper list for this printer.
 *    
 *    paperList:
 *      If successful noErr is returned and *paperList is a CFArray of
 *      PMPapers representing the list of papers available for the
 *      printer.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrinterPrintWithProvider( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; provider: CGDataProviderRef ): OSStatus; external name '_PMPrinterPrintWithProvider';
=======
function PMPrinterGetPaperList( printer: PMPrinter; var paperList: CFArrayRef ): OSStatus; external name '_PMPrinterGetPaperList';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMPrinterPrintWithFile()
 *  
 *  Summary:
 *    Submit a file for printing to a specified printer.
 *  
 *  Discussion:
 *
 *    One reason this function may fail is if the specified printer can
 *    not handle the file's mime type. Use PMPrinterGetMimeTypes() to
 *    check whether a mime type is supported.
 *  
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. If this parameter is
 *      NULL then the supplied file will be auto-typed.
 *    
 *    fileURL:
 *      A file URL specifying the file to be printed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterPrintWithFile( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; fileURL: CFURLRef ): OSStatus; external name '_PMPrinterPrintWithFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPrinterWritePostScriptToURL()
 *
 *  Summary:
 *    Convert an input file of the specified mimeType to printer ready PostScript for the destination printer.
 *
 *  Discussion:
 *
 *    The conversion of the input file to PostScript is performed before this function returns. This can take a significant
 *    amount of time for longer documents. The caller of PMPrinterWritePostScriptToURL may want to perform this operation
 *    on a thread other than the main application thread or fork a separate process for this purpose.
 *
 *    One reason this function may fail is if conversion from the input mimeType to PostScript cannot be performed.
 *
 *
 *  Parameters:
 *
 *    printer:
 *      The printer for which the printer ready PostScript will be generated.
 *
 *    settings:
 *      The print settings for the print job.
 *
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed.
 *
 *    mimeType:
 *      The mime type of the file to be printed. If NULL, the file is auto-typed.
 *
 *    sourceFileURL:
 *      A file URL specifying the input file to be converted to printer ready PostScript data. Only file based URLs
 *      are supported.
 *
 *    destinationFileURL:
 *      A file URL specifying the destination file to be created. If the file already exists it will be overwritten. Only
 *      file based URLs are supported.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterWritePostScriptToURL( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; sourceFileURL: CFURLRef; destinationFileURL: CFURLRef ): OSStatus; external name '_PMPrinterWritePostScriptToURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsToOptions()
 *  
 *  Summary:
 *    Convert print settings to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'ed C string
 *      describing the passed in print settings. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsToOptions( settings: PMPrintSettings; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrintSettingsToOptionsWithPrinterAndPageFormat()
 *  
 *  Summary:
 *    Convert a print settings and page format to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string. Must not be NULL.
 *    
 *    printer:
 *      The printer to use for converting the print settings. Must not be NULL.
 *
 *    format:
 *      The page format for use in creating the cups options. Can be NULL.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'd C string
 *      describing the passed in print settings and format. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
=======
 *  PMPrinterGetState()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetState( printer: PMPrinter; var state: PMPrinterState ): OSStatus; external name '_PMPrinterGetState';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsDefault()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsDefault( printer: PMPrinter ): Boolean; external name '_PMPrinterIsDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsFavorite()
 *  
 *  Summary:
 *    Return true if the printer is in the user's favorite printer list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsFavorite( printer: PMPrinter ): Boolean; external name '_PMPrinterIsFavorite';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsPostScriptCapable()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsPostScriptCapable( printer: PMPrinter ): Boolean; external name '_PMPrinterIsPostScriptCapable';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{!
 * @function	PMPrinterIsPostScriptPrinter
 * @abstract	Set *isPSPrinter true if the printer is a PostScript printer.
 *
 * @discussion	A PostScript printer is one whose driver takes PostScript directly.
 *
 *  Availability:
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMPrintSettingsToOptionsWithPrinterAndPageFormat( settings: PMPrintSettings; printer: PMPrinter; pageFormat: PMPageFormat; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptionsWithPrinterAndPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark PrinterCommandSupport
#pragma mark
*)

{
*  PMPrinterSendCommand()
*
*  Summary:
*    Submit a command file containing the specified string data to a print queue.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    commandString:
*      The contents of the command file as a CFStringRef.
*
*    jobTitle:
*      The title of the job associated with the command file. If NULL,
*      a job title will be automatically generated but it may not be
*      meaningful to a user.
*
*    options:
*      A dictionary containing options that apply to the job.
*      This parameter should be NULL; it is reserved for future expansion.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}
function PMPrinterSendCommand( printer: PMPrinter; commandString: CFStringRef; jobTitle: CFStringRef; options: CFDictionaryRef ): OSStatus; external name '_PMPrinterSendCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{
*  PMPrinterCopyState()
*
*  Summary:
*    Copies the current "marker-change-time", "marker-colors",
*    "marker-high-levels", "marker-levels", "marker-low-levels",
*    "marker-message", "marker-names", "marker-types", "printer-state",
*    "printer-state-change-time", "printer-state-message", and
*    "printer-state-reasons" values for a print queue.  These attributes are
*    defined in the "CUPS Implementation of IPP" specification at:
*
*        http://www.cups.org/documentation.php/spec-ipp.html
*        http://localhost:631/help/spec-ipp.html
*
*    Each attribute is returned using the corresponding CoreFoundation
*    data type:
*
*        Attribute                    Type
*        ---------------------------  ------------------------
*        "marker-change-time"         CFDateRef
*        "marker-colors"              CFArrayRef + CFStringRef
*        "marker-high-levels"         CFArrayRef + CFNumberRef
*        "marker-levels"              CFArrayRef + CFNumberRef
*        "marker-low-levels"          CFArrayRef + CFNumberRef
*        "marker-message"             CFStringRef
*        "marker-names"               CFArrayRef + CFStringRef
*        "marker-types"               CFArrayRef + CFStringRef
*        "printer-state"              CFNumberRef
*        "printer-state-change-time"  CFDateRef
*        "printer-state-message"      CFStringRef
*        "printer-state-reasons"      CFArrayRef + CFStringRef
*
*  Localization Considerations:
*
*    The "marker-message" and "printer-state-message" attributes may be
*    localized by the printer driver using the language of the last processed
*    print job.
*
*    The "marker-names" and "printer-state-reasons" attributes are typically
*    English-language keywords that may be localized using the printer's PPD
*    file and CUPS ppdLocalizeMarkerName and ppdLocalizeIPPReason APIs,
*    respectively.
*
*    All other attributes are predefined keywords or values that are not
*    localized.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    stateDict:
*      A pointer to a CFDictionaryRef variable to hold the values.
*      You must release the dictionary when you are done using it.
*      The dictionary returned is not mutable.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}
=======
function PMPrinterIsPostScriptPrinter( printer: PMPrinter; var isPSPrinter: Boolean ): OSStatus; external name '_PMPrinterIsPostScriptPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew

function PMPrinterCopyState( printer: PMPrinter; var stateDict: CFDictionaryRef ): OSStatus; external name '_PMPrinterCopyState';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

(*
#pragma mark
#pragma mark PPD related
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMCopyAvailablePPDs()
 *  
 *  Summary:
 *    Hand back the list of PPDs in the specified PPD domain.
 *  
 *  Parameters:
 *    
 *    domain:
 *      The domain to search for PPDs.
 *    
 *    ppds:
 *      If this function completes without error, *'ppds' is set to an
 *      array of CFURLs. Each CFURL specifies the location of a PPD
 *      file or a compressed PPD file. The caller is responsible for
 *      releasing the array. If this function returns a non-zero error
 *      code then *'ppds' is set to NULL.
=======
 *  PMPrinterIsRemote()
 *  
 *  Summary:
 *    Hand back a boolean indicating whether the printer is hosted by remote print server.
 *  
 *  Discussion:
 *    If on return *isRemoteP is true, the print queue represents a printer hosted and
 *    managed by a remote print server.
 *
 *    If on return *isRemoteP is false, the print queue represents a directly connected
 *    printer, a network printer, or a remote printer that is locally managed. Consult
 *    the queue's device URI to determine the type of connection that is used to communicate
 *    with the printer.
 *
 *    Whether a printer is remote is derived from the CUPS printer-type attribute for the print queue.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMCopyAvailablePPDs( domain: PMPPDDomain; var ppds: CFArrayRef ): OSStatus; external name '_PMCopyAvailablePPDs';
=======
function PMPrinterIsRemote( printer: PMPrinter; var isRemoteP: Boolean ): OSStatus; external name '_PMPrinterIsRemote';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
 *  PMCopyLocalizedPPD()
 *  
 *  Summary:
 *    Hand back a reference to a localized PPD.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A PPD reference. Typically this is a CFURLRef returned from
 *      PMCopyAvailablePPDs().
 *    
 *    localizedPPD:
 *      If this function completes without error, *'localizedPPD' will
 *      be set to a CFURLRef referencing the PPD that should be used
 *      given the current user's language preferences. If this function
 *      returns an error then *'localizedPPD' will be set to to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyLocalizedPPD( ppd: CFURLRef; var localizedPPD: CFURLRef ): OSStatus; external name '_PMCopyLocalizedPPD';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
 *  PMPrinterSetDefault()
 *  
 *  Summary:
 *    Set the default printer for the current user.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer to set as the default printer.
 *  
 *  Special considerations:
 *      It is not typical for an application to set the current default printer
 *      for the user; the printing system itself takes care of managing the default printer.
 *      This function should be used only in rare circumstances.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterSetDefault( printer: PMPrinter ): OSStatus; external name '_PMPrinterSetDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew

(*
#pragma mark
#pragma mark Preset
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMCopyPPDData()
 *  
 *  Summary:
 *    Hand back the uncompressed PPD data for a PPD or compressed PPD
 *    file.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A reference to a PPD or compressed PPD file. This reference is
 *      usually obtained from PMCopyAvailablePPDs() or from
 *      PMCopyLocalizedPPD().
 *    
 *    data:
 *      If this function completes without error then *'data' is set to
 *      reference the uncompressed PPD data from the PPD file. If this
 *      function returns a non-zero error then *'data is set to NULL.
=======
 *  PMPresetCopyName()
 *  
 *  Summary:
 *    Hand back a copy of the localized name for the specified preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      The preset whose name is needed.
 *    
 *    name:
 *      On exit, if this routine succeeds, *'name' is filled in with a
 *      reference to a localized string with the preset's name. If this
 *      routine fails, then *'name' is set to NULL.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function PMCopyPPDData( ppd: CFURLRef; var data: CFDataRef ): OSStatus; external name '_PMCopyPPDData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
function PMPresetCopyName( preset: PMPreset; var name: CFStringRef ): OSStatus; external name '_PMPresetCopyName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> origin/cpstrnew

(*
#pragma mark
#pragma mark EPS related
#pragma mark
*)
{
<<<<<<< HEAD
 *  PMCGImageCreateWithEPSDataProvider()
 *  
 *  Summary:
 *    Create an image reference that references both the PostScript
 *    contents of an EPS file and a preview (proxy) image for that EPS
 *    file.
 *  
 *  Discussion:
 *    For OS X 10.1.0, this function ignores the passed in data
 *    provider. The passed in image reference is retained and then
 *    returned. For 10.1.1 and later, then the data provider is used
 *    and the returned image reference is different than the passed in
 *    image reference, so please be careful with your use of these
 *    references. It is likely that the data will not be read from the
 *    EPS data provider until well after this function returns. The
 *    caller should be careful not to free the underlying EPS data
 *    until the provider's release routine is invoked. Similarly the
 *    preview image's data may be needed long after you think it should
 *    be. Do not free the image data until the image data provider's
 *    release data function has been called. To make sure these data
 *    providers are properly reference counted, release your reference
 *    the EPS data provider and on the EPS image preview when they are
 *    no longer needed by your application. For Mac OS X 10.2 and
 *    later, the contents of the EPS provider at the time of this call
 *    can be dumped to a file if you first do the following, BEFORE
 *    running your application. 
 *
 *	  From the command line in terminal:
 *		defaults write NSGlobalDomain com.apple.print.eps.testProvider /tmp/dump.eps
 *    
 *	  causes a dump of the EPS data into a file /tmp/dump.eps.
 *  
 *  Parameters:
 *    
 *    epsDataProvider:
 *      A Core Graphics data provider that can supply the PostScript
 *      contents of the EPS file. Post OS X 10.1, there will be some
 *      checking done on the EPS data provided to the
 *      PMCGImageCreateWithEPSDataProvider() call. It is important that
 *      the EPS data begin with the EPSF required header and bounding
 *      box DSC comments.
 *    
 *    epsPreview:
 *      A Core Graphics image reference to the proxy image for the EPS
 *      file. When the image reference result of this function is
 *      rendered on screen or printed to a printer that can not render
 *      PostScript this proxy image is drawn instead.
 *  
 *  Result:
 *    an image reference capable of rendering either the EPS content or
 *    the proxy image depending upon the capabilities of the targeted
 *    context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> graemeg/cpstrnew
=======
 *  PMPresetCreatePrintSettings()
 *  
 *  Summary:
 *    Create a print settings conforming to the specified print
 *    settings preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A preset specifying a set of initial print settings.
 *    
 *    session:
 *      A valid print session.
 *    
 *    printSettings:
 *      On exit, *'printSettings' is set to a newly created print
 *      settings that contains the settings specified by 'preset'. The
 *      caller is responsible for calling PMRelease when the print
 *      settings are no longer needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPresetCreatePrintSettings( preset: PMPreset; session: PMPrintSession; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPresetCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPresetGetAttributes()
 *  
 *  Summary:
 *    Hand back the meta-data describing a given preset.
 *  
 *  Discussion:
 *    Each preset has associated with it a dictionary containing
 *    meta-data. The meta-data provides the preset's id, the preset's
 *    localized names, and descriptions of the environment for which
 *    the preset it intended.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A print settings preset as obtained from PMPrinterCopyPresets().
 *    
 *    attributes:
 *      On exit, *'attributes' is set to reference a dictionary
 *      containing the preset's meta-data. The caller is responsible
 *      for retaining this reference if it is to be used beyond the
 *      lifetime of 'preset'. If this function fails, returning a
 *      non-zero error code, then *'attributes' is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPresetGetAttributes( preset: PMPreset; var attributes: CFDictionaryRef ): OSStatus; external name '_PMPresetGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Paper
#pragma mark
*)
{
 *  PMGetPageFormatPaper()
 *  
 *  Summary:
 *    Returns the paper associated with a pageformat.
 *  
 *  Parameters:
 *    
 *    format:
 *      Obtain the paper for this pageformat.
 *    
 *    paper:
 *      If successful noErr is returned and *paper will contain a
 *      PMPaper object describing the current paper associated with the
 *      pageformat.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageFormatPaper( format: PMPageFormat; var paper: PMPaper ): OSStatus; external name '_PMGetPageFormatPaper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperCreateCustom()
 *  
 *  Summary:
 *    Create a new custom paper instance.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The new paper size is appropriate for this printer.
 *    
 *    id:
 *      A unique identifier for this paper type.
 *    
 *    name:
 *      The name to display to the user for this paper type.
 *    
 *    width:
 *      The width, in points, of the paper.
 *    
 *    height:
 *      The height, in points, of the paper.
 *    
 *    margins:
 *      The unprintable margins on the paper.
 *    
 *    paperP:
 *      if this function is successful, returning noErr, then *'paperP'
 *      is set to be a reference to a newly created PMPaper instance.
 *      The caller is responsible for calling PMRelease when the
 *      instance is no longer needed. If this functions fails, it will
 *      return a non-zero error and set *'paperP' to NULL.
 *  
 *  Discussion:
 *		This function creates a new custom paper instance. To obtain one of the available
 *		built-in paper sizes for a given printer, use PMPrinterGetPaperList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperCreateCustom( printer: PMPrinter; id: CFStringRef; name: CFStringRef; width: Float64; height: Float64; const (*var*) margins: PMPaperMargins; var paperP: PMPaper ): OSStatus; external name '_PMPaperCreateCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetWidth()
 *  
 *  Summary:
 *    Returns the width for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the width for this paper.
 *    
 *    paperWidth:
 *      If successful noErr is returned and *paperWidth is set to the
 *      width of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetWidth( paper: PMPaper; var paperWidth: Float64 ): OSStatus; external name '_PMPaperGetWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetHeight()
 *  
 *  Summary:
 *    Returns the height for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the height for this paper.
 *    
 *    paperHeight:
 *      If successful noErr is returned and *paperHeight is set to the
 *      height of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetHeight( paper: PMPaper; var paperHeight: Float64 ): OSStatus; external name '_PMPaperGetHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetMargins()
 *  
 *  Summary:
 *    Returns the margins for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the margin information for this paper.
 *    
 *    paperMargins:
 *      If successful noErr is returned and *paperMargins is set to the
 *      margins of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetMargins( paper: PMPaper; var paperMargins: PMPaperMargins ): OSStatus; external name '_PMPaperGetMargins';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperGetID()
 *  
 *  Summary:
 *    Returns the id for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the id for this paper.
 *    
 *    paperID:
 *      If successful noErr is returned and *paperID is set to the id
 *      of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetID( paper: PMPaper; var paperID: CFStringRef ): OSStatus; external name '_PMPaperGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetName()
 *  
 *  Summary:
 *    Returns the name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the name for this paper.
 *    
 *    paperName:
 *      If successful noErr is returned and *paperName is set to the
 *      name of the paper. The returned paper name is that appropriate
 *      to display to a user.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetPPDPaperName()
 *  
 *  Summary:
 *    Returns the PPD name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the PPD name for this paper.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      PPD name of the paper. The returned result may be NULL.
 *  
 *  Discussion:
 *      The Mac OS X printing system uses a PostScript Printer Description (PPD)
 *      file to describe a given printer and print queue for that printer. The
 *      PPD name is the name that uniquely identifies a given paper
 *      to the printer to which the paper corresponds. 
 *    
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetPPDPaperName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetPPDPaperName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPaperCreateLocalizedName()
 *  
 *  Summary:
 *    Returns the localized name for a given paper. 
 *    Use of PMPaperCreateLocalizedName
 *    is recommended instead of PMPaperGetName.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the localized name for this paper.
 *    
 *    printer:
 *      The printer for which the localization should be performed.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      localized name of the paper. The returned paper name is that appropriate
 *      to display to a user. The returned result may be NULL.
 *      If non-NULL, the caller is responsible for releasing the name returned. 
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperCreateLocalizedName( paper: PMPaper; printer: PMPrinter; var paperName: CFStringRef ): OSStatus; external name '_PMPaperCreateLocalizedName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetPrinterID()
 *  
 *  Summary:
 *    Returns the printerID of the printer for which the paper corresponds.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the printer ID for this paper.
 *    
 *    printerID:
 *      If successful, noErr is returned and *printerID is set to the
 *      ID of the printer for which the paper corresponds. The returned
 *      result may be NULL. 
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetPrinterID( paper: PMPaper; var printerID: CFStringRef ): OSStatus; external name '_PMPaperGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperIsCustom()
 *  
 *  Summary:
 *    Returns true if the paper is a custom paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Determines if this is a custom paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperIsCustom( paper: PMPaper ): Boolean; external name '_PMPaperIsCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark PDF Workflow
#pragma mark
*)
{
 *  PMWorkflowCopyItems()
 *  
 *  Summary:
 *    Hand back an array of dictionaries describing the PDF Workflow
 *    items installed on the system
 *  
 *  Parameters:
 *    
 *    workflowItems:
 *      If this function returns without error then *'workflowItems'
 *      will be filled in with a reference to an array. It is the
 *      caller's responsability to release the array when done with it.
 *      Each element in the array describes a PDF Workflow item or a
 *      folder holding workflow items. A dictionary describing a
 *      workflow item has, at least, the following keys and values:
 *      displayName - The user's diaplayable name for the workflow item
 *      itemURL - A CFURLRef pointing to the workflow item. A
 *      dictionary describing a workflow folder has at least the
 *      following keys: displayName - The user's diaplayable name for
 *      the workflow item folderURL - A CFURLRef pointing to the
 *      folder. items - A CFArrayRef describing the workflow items in
 *      the folder. If this function returns a non-zero error code then
 *      *'workflowItems' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowCopyItems( var workflowItems: CFArrayRef ): OSStatus; external name '_PMWorkflowCopyItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithOptions()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    title:
 *      The user displayable name of the document.
 *    
 *    options:
 *      A string of CUPS style key-value pairs that may be passed to
 *      the PDF Workflow item. This parameter can be NULL in which case
 *      an empty string of options is used.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowSubmitPDFWithOptions( workflowItem: CFURLRef; title: CFStringRef; options: ConstCStringPtr; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithSettings()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    settings:
 *      The prints settings to apply to the PDF.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowSubmitPDFWithSettings( workflowItem: CFURLRef; settings: PMPrintSettings; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Job Submission
#pragma mark
*)
{
 *  PMPrinterPrintWithProvider()
 *  
 *  Summary:
 *    Submit print data to a specified printer.
 *  
 *  Discussion:
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. This parameter can not
 *      be NULL. Use PMPrinterPrintWithFile() if auto-typing is desired.
 *    
 *    provider:
 *      The data provider that supplies the print data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterPrintWithProvider( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; provider: CGDataProviderRef ): OSStatus; external name '_PMPrinterPrintWithProvider';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterPrintWithFile()
 *  
 *  Summary:
 *    Submit a file for printing to a specified printer.
 *  
 *  Discussion:
 *
 *    One reason this function may fail is if the specified printer can
 *    not handle the file's mime type. Use PMPrinterGetMimeTypes() to
 *    check whether a mime type is supported.
 *  
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. If this parameter is
 *      NULL then the supplied file will be auto-typed.
 *    
 *    fileURL:
 *      A file URL specifying the file to be printed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterPrintWithFile( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; fileURL: CFURLRef ): OSStatus; external name '_PMPrinterPrintWithFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPrinterWritePostScriptToURL()
 *
 *  Summary:
 *    Convert an input file of the specified mimeType to printer ready PostScript for the destination printer.
 *
 *  Discussion:
 *
 *    The conversion of the input file to PostScript is performed before this function returns. This can take a significant
 *    amount of time for longer documents. The caller of PMPrinterWritePostScriptToURL may want to perform this operation
 *    on a thread other than the main application thread or fork a separate process for this purpose.
 *
 *    One reason this function may fail is if conversion from the input mimeType to PostScript cannot be performed.
 *
 *
 *  Parameters:
 *
 *    printer:
 *      The printer for which the printer ready PostScript will be generated.
 *
 *    settings:
 *      The print settings for the print job.
 *
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed.
 *
 *    mimeType:
 *      The mime type of the file to be printed. If NULL, the file is auto-typed.
 *
 *    sourceFileURL:
 *      A file URL specifying the input file to be converted to printer ready PostScript data. Only file based URLs
 *      are supported.
 *
 *    destinationFileURL:
 *      A file URL specifying the destination file to be created. If the file already exists it will be overwritten. Only
 *      file based URLs are supported.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterWritePostScriptToURL( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; sourceFileURL: CFURLRef; destinationFileURL: CFURLRef ): OSStatus; external name '_PMPrinterWritePostScriptToURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsToOptions()
 *  
 *  Summary:
 *    Convert print settings to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'ed C string
 *      describing the passed in print settings. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsToOptions( settings: PMPrintSettings; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrintSettingsToOptionsWithPrinterAndPageFormat()
 *  
 *  Summary:
 *    Convert a print settings and page format to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string. Must not be NULL.
 *    
 *    printer:
 *      The printer to use for converting the print settings. Must not be NULL.
 *
 *    format:
 *      The page format for use in creating the cups options. Can be NULL.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'd C string
 *      describing the passed in print settings and format. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsToOptionsWithPrinterAndPageFormat( settings: PMPrintSettings; printer: PMPrinter; pageFormat: PMPageFormat; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptionsWithPrinterAndPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark PrinterCommandSupport
#pragma mark
*)

{
*  PMPrinterSendCommand()
*
*  Summary:
*    Submit a command file containing the specified string data to a print queue.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    commandString:
*      The contents of the command file as a CFStringRef.
*
*    jobTitle:
*      The title of the job associated with the command file. If NULL,
*      a job title will be automatically generated but it may not be
*      meaningful to a user.
*
*    options:
*      A dictionary containing options that apply to the job.
*      This parameter should be NULL; it is reserved for future expansion.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}
function PMPrinterSendCommand( printer: PMPrinter; commandString: CFStringRef; jobTitle: CFStringRef; options: CFDictionaryRef ): OSStatus; external name '_PMPrinterSendCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{
*  PMPrinterCopyState()
*
*  Summary:
*    Copies the current "marker-change-time", "marker-colors",
*    "marker-high-levels", "marker-levels", "marker-low-levels",
*    "marker-message", "marker-names", "marker-types", "printer-state",
*    "printer-state-change-time", "printer-state-message", and
*    "printer-state-reasons" values for a print queue.  These attributes are
*    defined in the "CUPS Implementation of IPP" specification at:
*
*        http://www.cups.org/documentation.php/spec-ipp.html
*        http://localhost:631/help/spec-ipp.html
*
*    Each attribute is returned using the corresponding CoreFoundation
*    data type:
*
*        Attribute                    Type
*        ---------------------------  ------------------------
*        "marker-change-time"         CFDateRef
*        "marker-colors"              CFArrayRef + CFStringRef
*        "marker-high-levels"         CFArrayRef + CFNumberRef
*        "marker-levels"              CFArrayRef + CFNumberRef
*        "marker-low-levels"          CFArrayRef + CFNumberRef
*        "marker-message"             CFStringRef
*        "marker-names"               CFArrayRef + CFStringRef
*        "marker-types"               CFArrayRef + CFStringRef
*        "printer-state"              CFNumberRef
*        "printer-state-change-time"  CFDateRef
*        "printer-state-message"      CFStringRef
*        "printer-state-reasons"      CFArrayRef + CFStringRef
*
*  Localization Considerations:
*
*    The "marker-message" and "printer-state-message" attributes may be
*    localized by the printer driver using the language of the last processed
*    print job.
*
*    The "marker-names" and "printer-state-reasons" attributes are typically
*    English-language keywords that may be localized using the printer's PPD
*    file and CUPS ppdLocalizeMarkerName and ppdLocalizeIPPReason APIs,
*    respectively.
*
*    All other attributes are predefined keywords or values that are not
*    localized.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    stateDict:
*      A pointer to a CFDictionaryRef variable to hold the values.
*      You must release the dictionary when you are done using it.
*      The dictionary returned is not mutable.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}

function PMPrinterCopyState( printer: PMPrinter; var stateDict: CFDictionaryRef ): OSStatus; external name '_PMPrinterCopyState';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

(*
#pragma mark
#pragma mark PPD related
#pragma mark
*)
{
 *  PMCopyAvailablePPDs()
 *  
 *  Summary:
 *    Hand back the list of PPDs in the specified PPD domain.
 *  
 *  Parameters:
 *    
 *    domain:
 *      The domain to search for PPDs.
 *    
 *    ppds:
 *      If this function completes without error, *'ppds' is set to an
 *      array of CFURLs. Each CFURL specifies the location of a PPD
 *      file or a compressed PPD file. The caller is responsible for
 *      releasing the array. If this function returns a non-zero error
 *      code then *'ppds' is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyAvailablePPDs( domain: PMPPDDomain; var ppds: CFArrayRef ): OSStatus; external name '_PMCopyAvailablePPDs';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMCopyLocalizedPPD()
 *  
 *  Summary:
 *    Hand back a reference to a localized PPD.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A PPD reference. Typically this is a CFURLRef returned from
 *      PMCopyAvailablePPDs().
 *    
 *    localizedPPD:
 *      If this function completes without error, *'localizedPPD' will
 *      be set to a CFURLRef referencing the PPD that should be used
 *      given the current user's language preferences. If this function
 *      returns an error then *'localizedPPD' will be set to to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyLocalizedPPD( ppd: CFURLRef; var localizedPPD: CFURLRef ): OSStatus; external name '_PMCopyLocalizedPPD';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMCopyPPDData()
 *  
 *  Summary:
 *    Hand back the uncompressed PPD data for a PPD or compressed PPD
 *    file.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A reference to a PPD or compressed PPD file. This reference is
 *      usually obtained from PMCopyAvailablePPDs() or from
 *      PMCopyLocalizedPPD().
 *    
 *    data:
 *      If this function completes without error then *'data' is set to
 *      reference the uncompressed PPD data from the PPD file. If this
 *      function returns a non-zero error then *'data is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPPDData( ppd: CFURLRef; var data: CFDataRef ): OSStatus; external name '_PMCopyPPDData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark EPS related
#pragma mark
*)
{
 *  PMCGImageCreateWithEPSDataProvider()
 *  
 *  Summary:
 *    Create an image reference that references both the PostScript
 *    contents of an EPS file and a preview (proxy) image for that EPS
 *    file.
 *  
 *  Discussion:
 *    For OS X 10.1.0, this function ignores the passed in data
 *    provider. The passed in image reference is retained and then
 *    returned. For 10.1.1 and later, then the data provider is used
 *    and the returned image reference is different than the passed in
 *    image reference, so please be careful with your use of these
 *    references. It is likely that the data will not be read from the
 *    EPS data provider until well after this function returns. The
 *    caller should be careful not to free the underlying EPS data
 *    until the provider's release routine is invoked. Similarly the
 *    preview image's data may be needed long after you think it should
 *    be. Do not free the image data until the image data provider's
 *    release data function has been called. To make sure these data
 *    providers are properly reference counted, release your reference
 *    the EPS data provider and on the EPS image preview when they are
 *    no longer needed by your application. For Mac OS X 10.2 and
 *    later, the contents of the EPS provider at the time of this call
 *    can be dumped to a file if you first do the following, BEFORE
 *    running your application. 
 *
 *	  From the command line in terminal:
 *		defaults write NSGlobalDomain com.apple.print.eps.testProvider /tmp/dump.eps
 *    
 *	  causes a dump of the EPS data into a file /tmp/dump.eps.
 *  
 *  Parameters:
 *    
 *    epsDataProvider:
 *      A Core Graphics data provider that can supply the PostScript
 *      contents of the EPS file. Post OS X 10.1, there will be some
 *      checking done on the EPS data provided to the
 *      PMCGImageCreateWithEPSDataProvider() call. It is important that
 *      the EPS data begin with the EPSF required header and bounding
 *      box DSC comments.
 *    
 *    epsPreview:
 *      A Core Graphics image reference to the proxy image for the EPS
 *      file. When the image reference result of this function is
 *      rendered on screen or printed to a printer that can not render
 *      PostScript this proxy image is drawn instead.
 *  
 *  Result:
 *    an image reference capable of rendering either the EPS content or
 *    the proxy image depending upon the capabilities of the targeted
 *    context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
>>>>>>> origin/cpstrnew
 }
function PMCGImageCreateWithEPSDataProvider( epsDataProvider: CGDataProviderRef; epsPreview: CGImageRef ): CGImageRef; external name '_PMCGImageCreateWithEPSDataProvider';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
function PMSessionSetIdleProc(printSession: PMPrintSession; idleProc: PMIdleUPP): OSStatus; external name '_PMSessionSetIdleProc';

{
 *  PMSessionSetDataInSession()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionSetDataInSession(printSession: PMPrintSession; key: CFStringRef; data: CFTypeRef): OSStatus; external name '_PMSessionSetDataInSession';

{
 *  PMSessionGetDataFromSession()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PMSessionGetDataFromSession(printSession: PMPrintSession; key: CFStringRef; var data: CFTypeRef): OSStatus; external name '_PMSessionGetDataFromSession';

{
 *  PMSessionCreatePrinterList()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionCreatePrinterList(printSession: PMPrintSession; var printerList: CFArrayRef; var currentIndex: CFIndex; var currentPrinter: PMPrinter): OSStatus; external name '_PMSessionCreatePrinterList';

{
 *  PMSessionSetCurrentPrinter()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionSetCurrentPrinter(session: PMPrintSession; printerName: CFStringRef): OSStatus; external name '_PMSessionSetCurrentPrinter';

{
 *  PMSessionSetDestination()
 *  
 *  Summary:
 *    Alter a print session and print settings so that an associated
 *    print job is sent to the provided destination type in the,
 *    optional, MIME document format.
 *  
 *  Discussion:
 *    This function is most useful when an application would like to
 *    write its print output to disk without requiring user
 *    interaction. The list of MIME types that can be sent to the
 *    provided destination can be obtained from
 *    PMSessionCopyOutputFormatList and one of these passed to this
 *    function.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file. It also holds the requested MIME output
 *      type.
 *    
 *    destType:
 *      The destiation type for a print job associated with the
 *      provided print session and print settings. Fax is currently not
 *      supported, but kPMDestinationPrinter, kPMDestinationFile, and
 *      kPMDestinationPreview can be set.
 *    
 *    destFormat:
 *      The MIME type to be generated for the provided destination
 *      type. This parameter can be NULL in which the default format
 *      for the requested destination type is used. To obtain a list of
 *      valid formats for a given destiation type, use the function
 *      PMSessionCopyOutputFormatList.
 *    
 *    destLocation:
 *      Some destination types support a destination location. The
 *      clearest example is the kPMDestinationFile destination type
 *      which allows a caller to also supply a file URL specifying
 *      where the output file is to be created.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PMSessionSetDestination(printSession: PMPrintSession; printSettings: PMPrintSettings; destType: PMDestinationType; destFormat: CFStringRef; destLocation: CFURLRef): OSStatus; external name '_PMSessionSetDestination';

{
=======
>>>>>>> origin/fixes_2.4
 *  PMSessionGetDestinationType()
 *  
 *  Summary:
 *    Hand back the destination type that will be used for a print job
 *    with the specified print settings and print session.
 *  
 *  Discussion:
 *    Currently there are five destination types:
 *    kPMDestinationPrinter, kPMDestinationFile, kPMDestinationFax and
 *    kPMDestinationPreview, and kPMDestinationProcessPDF.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file.
 *    
 *    destTypeP:
 *      A pointer to a caller supplied PMDestinationType variable. If
 *      this function succeeds then *'destTypeP' will be filled in with
 *      the destination type for a print job that used the specified
 *      session and print settings. If this function fails, then
 *      *'destType' will be set to kPMDestinationInvalid.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetDestinationType( printSession: PMPrintSession; printSettings: PMPrintSettings; var destTypeP: PMDestinationType ): OSStatus; external name '_PMSessionGetDestinationType';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
  
{
 *  PMSessionCopyDestinationFormat()
 *  
 *  Summary:
 *    Hand back the destination output MIME type associated with the
 *    provided print session and print settings.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      A currently open print session.
 *    
 *    printSettings:
 *      The print settings that are to be searched.
 *    
 *    destFormatP:
 *      A pointer to a caller allocated CFStringRef variable. If this
 *      routine returns noErr then *'destFormatP' will either be a copy
 *      of a CFStringRef specifying the output format for the print
 *      job, or NULL indicating that the default output format will be
 *      used. If this function return an error, then *'destFormatP'
 *      will be set to NULL.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCopyDestinationFormat( printSession: PMPrintSession; printSettings: PMPrintSettings; var destFormatP: CFStringRef ): OSStatus; external name '_PMSessionCopyDestinationFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionCopyDestinationLocation()
 *  
 *  Summary:
 *    Hand back the URL destination location given a print session and
 *    print settings.
 *  
 *  Discussion:
 *    Some destination type support a destination location which
 *    further defines where the output from a pritn job should be sent.
 *    The kPMDestinationFile destiation type, for example, will use a
 *    file URL to determine where a new file should be created.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      A currently open print session.
 *    
 *    printSettings:
 *      The print settings that are to be searched.
 *    
 *    destLocationP:
 *      A pointer to a caller allocated CFURLRef variable. If this
 *      routine returns noErr then *'outputFileP' will either be NULL
 *      indicating that the job is using the default destination
 *      location for the current destination type or a copy of a
 *      CFURLRef will be placed in *'destLocationP'. If this function
 *      returns an error then 'destLocationP' will be set to NULL.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCopyDestinationLocation( printSession: PMPrintSession; printSettings: PMPrintSettings; var destLocationP: CFURLRef ): OSStatus; external name '_PMSessionCopyDestinationLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionSetDestination()
 *  
 *  Summary:
 *    Alter a print session and print settings so that an associated
 *    print job is sent to the provided destination type in the,
 *    optional, MIME document format.
 *  
 *  Discussion:
 *    This function is most useful when an application would like to
 *    write its print output to disk without requiring user
 *    interaction. The list of MIME types that can be sent to the
 *    provided destination can be obtained from
 *    PMSessionCopyOutputFormatList and one of these passed to this
 *    function.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      The session to be used for a print job. The session holds the
 *      preview setting which can override the destination type in the
 *      print settings.
 *    
 *    printSettings:
 *      The print settings to be used for a print job. The print
 *      settings specify whether a job will be directed toward a
 *      printer or to file. It also holds the requested MIME output
 *      type.
 *    
 *    destType:
 *      The destiation type for a print job associated with the
 *      provided print session and print settings. Fax is currently not
 *      supported, but kPMDestinationPrinter, kPMDestinationFile, and
 *      kPMDestinationPreview can be set.
 *    
 *    destFormat:
 *      The MIME type to be generated for the provided destination
 *      type. This parameter can be NULL in which the default format
 *      for the requested destination type is used. To obtain a list of
 *      valid formats for a given destiation type, use the function
 *      PMSessionCopyOutputFormatList.
 *    
 *    destLocation:
 *      Some destination types support a destination location. The
 *      clearest example is the kPMDestinationFile destination type
 *      which allows a caller to also supply a file URL specifying
 *      where the output file is to be created.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetDestination( printSession: PMPrintSession; printSettings: PMPrintSettings; destType: PMDestinationType; destFormat: CFStringRef; destLocation: CFURLRef ): OSStatus; external name '_PMSessionSetDestination';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
  
{
 *  PMSessionCopyOutputFormatList()
 *  
 *  Summary:
 *    Hands back an an array of MIME types describing the possible
 *    output formats for the printer module associated with the current
 *    printer.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      This session's current printer's printer module will be queried
 *      for its supported output MIME types.
 *    
 *    destType:
 *      A print job can have one of several possible destination types.
 *      The list of valid output formats is dependent upon the
 *      destination type. This parameter specifies destination type of
 *      interest when retrieving the output formats list.
 *    
 *    documentFormatP:
 *      A pointer to a caller's CFArrayRef variable. If this routine
 *      completes successfully, then *'documentFormatP' will be set to
 *      a CFArrayRef containing CFStringRefs. Each CFStringRef in the
 *      array is a MIME type specifying a type of output that can be
 *      generated by the printer module associated with the current
 *      printer.
 *    
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later. On Mac OS 8/9 using CarbonLib, this
 *      routine returns kPMNotImplemented
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCopyOutputFormatList( printSession: PMPrintSession; destType: PMDestinationType; var documentFormatP: CFArrayRef ): OSStatus; external name '_PMSessionCopyOutputFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionCreatePageFormatList()
 *  
 *  Summary:
 *    Hand back a list of page format instances. Each page format
 *    instance describes a paper size available on the specified
 *    printer.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      A currently valid print session.
 *    
 *    printer:
 *      The printer whose page size list should be enumerated. 
 *    
 *    pageFormatList:
 *      If this function is successful then noErr will be returned and
 *      *'pageFormatList' will be set to a newly created CFArray. Each
 *      element in the array will be a PMPageFormat describing an
 *      available paper size for the specified printer. If this
 *      function fails then a non-zero error code will be returned and
 *      *'pageFormatList' will be set to NULL.
 *    
 *  Discussion:
 *	If you want to create the page format list for the session's current printer,
 *      pass the PMPrinter object returned by PMSessionGetCurrentPrinter() as the
 *	printer parameter. 
 *
 *    SPECIAL_AVAILABILITY_NOTE:
 *      This routine is available in ApplicationsServices.framework in
 *      Mac OS X version 10.1 and later. On Mac OS X it is available to
 *      CFM applications through CarbonLib starting with Mac OS X
 *      version 10.2 and later. On Mac OS 8/9 using CarbonLib, this
 *      routine returns kPMNotImplemented
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCreatePageFormatList( printSession: PMPrintSession; printer: PMPrinter; var pageFormatList: CFArrayRef ): OSStatus; external name '_PMSessionCreatePageFormatList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  PMSessionCreatePrinterList()
 *
 *  See also: PMServerCreatePrinterList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionCreatePrinterList( printSession: PMPrintSession; var printerList: CFArrayRef; var currentIndex: CFIndex; var currentPrinter: PMPrinter ): OSStatus; external name '_PMSessionCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
<<<<<<< HEAD
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }
{
>>>>>>> origin/fixes_2_2
 *  PMSessionEndDocumentNoDialog()
=======
 *  PMSessionGetCurrentPrinter()
 *  
 *  Summary:
 *    Hand back the session's current printer.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetCurrentPrinter( printSession: PMPrintSession; var currentPrinter: PMPrinter ): OSStatus; external name '_PMSessionGetCurrentPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionSetCurrentPMPrinter()
 *  
 *  Summary:
 *    Set the session's current printer to a specified PMPrinter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetCurrentPMPrinter( session: PMPrintSession; printer: PMPrinter ): OSStatus; external name '_PMSessionSetCurrentPMPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMSessionGetDataFromSession()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetDataFromSession( printSession: PMPrintSession; key: CFStringRef; var data: CFTypeRef ): OSStatus; external name '_PMSessionGetDataFromSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionSetDataInSession()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetDataInSession( printSession: PMPrintSession; key: CFStringRef; data: CFTypeRef ): OSStatus; external name '_PMSessionSetDataInSession';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark -
#pragma mark PageFormat
#pragma mark
*)

{
 *  PMCreatePageFormat()
 *  
 *  Summary:
 *    Allocates memory for a new PMPageFormat object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A pageformat is created with a refcount of 1.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePageFormat( var pageFormat: PMPageFormat ): OSStatus; external name '_PMCreatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDefaultPageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionDefaultPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePageFormat( printSession: PMPrintSession; pageFormat: PMPageFormat; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPageFormat()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPageFormat( formatSrc: PMPageFormat; formatDest: PMPageFormat ): OSStatus; external name '_PMCopyPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMCreatePageFormatWithPMPaper()
 *  
 *  Summary:
 *    Create a pageformat with a specific paper.
 *  
 *  Parameters:
 *    
 *    pageFormat:
 *      On return, will contain the pageformat which was created
 *    
 *    paper:
 *      The paper that will be associate with the pageformat
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePageFormatWithPMPaper( var pageFormat: PMPageFormat; paper: PMPaper ): OSStatus; external name '_PMCreatePageFormatWithPMPaper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPageFormatCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPageFormat object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPageFormatToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPageFormatCreateWithDataRepresentation to create a PMPageFormat from 
 *    a CFDataRef created by this call.
 *  
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateDataRepresentation( pageFormat: PMPageFormat; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPageFormatCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPageFormatCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPageFormat object from a data representation created with
 *	  PMPageFormatCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatCreateWithDataRepresentation( data: CFDataRef; var pageFormat: PMPageFormat ): OSStatus; external name '_PMPageFormatCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)



{
 *  PMGetAdjustedPageRect()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetAdjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetAdjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetAdjustedPaperRect()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetAdjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetAdjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetOrientation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetOrientation( pageFormat: PMPageFormat; var orientation: PMOrientation ): OSStatus; external name '_PMGetOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetPageFormatExtendedData()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; var size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMGetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPageFormatGetPrinterID()
 *  
 *  Summary:
 *		Obtains the formatting printer for the pageformat.
 *
 *  Discussion:
 *		Will either return the formatting printer for the pageformat
 *		or will return NULL if the pageformat doesn't have that information.
 * 
 *  Parameters:
 *    
 *    pageFormat:
 *      The pageformat to obtain the information from.
 *
 *	  printerID:
 *		Where to store the name of the printer
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPageFormatGetPrinterID( pageFormat: PMPageFormat; var printerID: CFStringRef ): OSStatus; external name '_PMPageFormatGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMGetScale()
 *  
 *  Summary:
 *    Obtains the scaling factor currently applied to the page and
 *    paper rectangles.
 *  
 *  Discussion:
 *    A value of 100.0 means 100% (no scaling). 50.0 means 50% scaling
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetScale( pageFormat: PMPageFormat; var scale: Float64 ): OSStatus; external name '_PMGetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPageRect()
 *  
 *  Summary:
 *    Obtains the size of the imageable area in points, unaffected by
 *    orientation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the imageable area of the page without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPageRect( pageFormat: PMPageFormat; var pageRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetUnadjustedPaperRect()
 *  
 *  Summary:
 *    Obtains a rectangle that specifies the size of the paper in
 *    points, unaffected by rotation, resolution, or scaling.
 *  
 *  Discussion:
 *    This is the physical size of the paper without regard to
 *    resolution, orientation or scaling. Dimensions are returned as a 72dpi
 *    values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetUnadjustedPaperRect( pageFormat: PMPageFormat; var paperRect: PMRect ): OSStatus; external name '_PMGetUnadjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)



{**********************}
{ PMSetxxx calls only save the value inside the printing object. They make no assumption on the }
{ validity of the value. This should be done using PMValidatePageFormat/PMSessionValidatePageFormat }
{ Any dependent settings are also updated during a validate call. }
{ For example: }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 )  }
{ PMGetAdjustedPaperRect - returns the SAME rect as the first call  }
{ PMGetAdjustedPaperRect - returns a rect of a certain size }
{ PMSetScale( aPageFormat, 500.0 ) }
{ PMValidatePageFormat or PMSessionValidatePageFormat }
{ PMGetAdjustedPaperRect - returns a rect thats scaled 500% from the first call }
{**********************}

{
 *  PMSetOrientation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetOrientation( pageFormat: PMPageFormat; orientation: PMOrientation; lock: Boolean ): OSStatus; external name '_PMSetOrientation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetPageFormatExtendedData()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPageFormatExtendedData( pageFormat: PMPageFormat; dataID: OSType; size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMSetPageFormatExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetScale()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetScale( pageFormat: PMPageFormat; scale: Float64 ): OSStatus; external name '_PMSetScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

(*
#pragma mark -
#pragma mark PrintSettings
#pragma mark
*)

{$ifc PM_USE_SESSION_APIS}
{
 *  PMCreatePrintSettings()
 *  
 *  Summary:
 *    Allocates memory for a new PMPrintSettings object in your
 *    application’s memory space.
 *  
 *  Discussion:
 *    A printSettings is created with a refcount of 1.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCreatePrintSettings( var printSettings: PMPrintSettings ): OSStatus; external name '_PMCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionDefaultPrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDefaultPrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings ): OSStatus; external name '_PMSessionDefaultPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMSessionValidatePrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionValidatePrintSettings( printSession: PMPrintSession; printSettings: PMPrintSettings; result: BooleanPtr ): OSStatus; external name '_PMSessionValidatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMCopyPrintSettings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPrintSettings( settingSrc: PMPrintSettings; settingDest: PMPrintSettings ): OSStatus; external name '_PMCopyPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsCreateDataRepresentation()
 *  
 *  Summary:
 *    Returns a data representation of a PMPrintSettings object as a CFDataRef.
 *    The caller is responsible for releasing the CFData object returned.
 *  
 *  Discussion:
 *    This function is similar to PMFlattenPrintSettingsToCFData but allows you
 *    to specify the format of the data that is returned. Use  
 *    PMPrintSettingsCreateWithDataRepresentation to create a PMPrintSettings from 
 *    a CFDataRef created by this call.
 *
 *    See PMDataFormat for information about the available data formats.  
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCreateDataRepresentation( printSettings: PMPrintSettings; var data: CFDataRef; format: PMDataFormat ): OSStatus; external name '_PMPrintSettingsCreateDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsCreateWithDataRepresentation()
 *  
 *  Summary:
 *    Creates a PMPrintSettings object from a data representation created with
 *	  PMPrintSettingsCreateDataRepresentation. The caller is responsible
 *    for releasing the PMPrintSettings object returned with PMRelease.
 *    
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCreateWithDataRepresentation( data: CFDataRef; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPrintSettingsCreateWithDataRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


  
{
 *  PMGetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCollate( printSettings: PMPrintSettings; var collate: Boolean ): OSStatus; external name '_PMGetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMGetCopies()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetCopies( printSettings: PMPrintSettings; var copies: UInt32 ): OSStatus; external name '_PMGetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDuplex( printSettings: PMPrintSettings; var duplexSetting: PMDuplexMode ): OSStatus; external name '_PMGetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMGetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetFirstPage( printSettings: PMPrintSettings; var first: UInt32 ): OSStatus; external name '_PMGetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetLastPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetLastPage( printSettings: PMPrintSettings; var last: UInt32 ): OSStatus; external name '_PMGetLastPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMGetPageRange()
 *  
 *  Summary:
 *    Obtains the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The default page range is 1 - (all pages). The page range is
 *    something that is set by the application. It is NOT the first and
 *    last page to print. It serves as limits for setting the first and
 *    last page.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageRange( printSettings: PMPrintSettings; var minPage: UInt32; var maxPage: UInt32 ): OSStatus; external name '_PMGetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrintSettingsGetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetJobName( printSettings: PMPrintSettings; var name: CFStringRef ): OSStatus; external name '_PMPrintSettingsGetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  PMPrintSettingsGetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings to retrieve the value from
 *    
 *    key:
 *      The key to look for
 *    
 *    value:
 *      The return value. Its either the value for the key or NULL
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsGetValue( printSettings: PMPrintSettings; key: CFStringRef; var value: CFTypeRef ): OSStatus; external name '_PMPrintSettingsGetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)



{
 *  PMSetCollate()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCollate( printSettings: PMPrintSettings; collate: Boolean ): OSStatus; external name '_PMSetCollate';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMSetCopies()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetCopies( printSettings: PMPrintSettings; copies: UInt32; lock: Boolean ): OSStatus; external name '_PMSetCopies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetDuplex()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetDuplex( printSettings: PMPrintSettings; duplexSetting: PMDuplexMode ): OSStatus; external name '_PMSetDuplex';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMSetFirstPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetFirstPage( printSettings: PMPrintSettings; first: UInt32; lock: Boolean ): OSStatus; external name '_PMSetFirstPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetLastPage()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetLastPage( printSettings: PMPrintSettings; last: UInt32; lock: Boolean ): OSStatus; external name '_PMSetLastPage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMSetPageRange()
 *  
 *  Summary:
 *    Sets the valid range of pages that can be printed.
 *  
 *  Discussion:
 *    The first and last page are immediately clipped to the new range.
 *    You may pass kPMPrintAllPages for the maxPage value to specified
 *    that all pages are available for printing.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPageRange( printSettings: PMPrintSettings; minPage: UInt32; maxPage: UInt32 ): OSStatus; external name '_PMSetPageRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrintSettingsSetJobName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetJobName( printSettings: PMPrintSettings; name: CFStringRef ): OSStatus; external name '_PMPrintSettingsSetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrintSettingsSetValue()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      The printsettings in which to set the value
 *    
 *    key:
 *      The key to store the value in
 *    
 *    value:
 *      The value to store in the key. If NULL, any existing setting item with
 *      the specified key is removed.
 *    
 *    locked:
 *      A boolean value indicating whether the item being set should be
 *      locked. It is strongly recommended to pass false.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsSetValue( printSettings: PMPrintSettings; key: CFStringRef; value: CFTypeRef; locked: Boolean ): OSStatus; external name '_PMPrintSettingsSetValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
  
  
{
 *  PMPrintSettingsCopyAsDictionary()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Represent these print settings as a dictionary.
 *    
 *    settingsDictionary:
 *      On exit, if successful *'settingsDictionary' will contain a reference to
 *      a CFDictionary describing the print settings. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'settingsDictionary' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCopyAsDictionary( printSettings: PMPrintSettings; var settingsDictionary: CFDictionaryRef ): OSStatus; external name '_PMPrintSettingsCopyAsDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrintSettingsCopyKeys()
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      Return the keys for items in this print settings.
 *    
 *    settingsKeys:
 *      On exit, if successful *'settingsKeys' will contain a reference to
 *      a CFArray describing the item keys in the print settings. Each of these
 *		keys may be passed to PMPrintSettingsGetValue() to obtain a value.
 *		The caller is responsible for releasing this reference. If this call returns
 *      an error, then *'settingsKeys' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsCopyKeys( printSettings: PMPrintSettings; var settingsKeys: CFArrayRef ): OSStatus; external name '_PMPrintSettingsCopyKeys';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark -
#pragma mark Printer
#pragma mark
*)

{!
 *
 * PMCreateGenericPrinter
 *  Summary:
 *	Creates a generic PMPrinter
 *
 *  Parameters:
 *		printer:
 *		On return *printer contains the generic priner
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMCreateGenericPrinter( var printer: PMPrinter ): OSStatus; external name '_PMCreateGenericPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMServerCreatePrinterList()
 *  
 *  Summary:
 *    Hand back an array of PMPrinter objects that represents the list of printers
 *    known to the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server for which you want to obtain the list of printers.
 *      Use kPMServerLocal for the local print server.
 *    
 *    printerList:
 *      On return, *printerList contains the array of PMPrinter objects returned. The
 *	caller is responsible for releasing this array. On error, *printerList is NULL.
 *    
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMServerCreatePrinterList( server: PMServer; var printerList: CFArrayRef ): OSStatus; external name '_PMServerCreatePrinterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMServerLaunchPrinterBrowser()
 *  
 *  Summary:
 *    Launch the printer browser, browsing the printers available for
 *    the specified print server.
 *
 *  Parameters:
 *    
 *    server:
 *      The print server whose available printers the browser should browse.
 *      Pass kPMServerLocal for the local print server.
 *    
 *    options:
 *      A CFDictionaryRef specifying how the browser should be presented. 
 *      Passing NULL presents the browser in the default fashion. 
 *
 *  Discussion:	The local print server allows its printers to be browsed but
 *		not all print servers do. Passing in a server whose printers cannot be
 *      browsed returns the error code kPMInvalidParameter.
 *    
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMServerLaunchPrinterBrowser( server: PMServer; options: CFDictionaryRef ): OSStatus; external name '_PMServerLaunchPrinterBrowser';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterCreateFromPrinterID()
 *  
 *  Summary:
 *    Create a PMPrinter instance from the supplied printerID.
 *  
 *  Discussion:
 *    Returns a PMPrinter instance for the printer whose ID is
 *    printerID. The caller is responsible for releasing this instance
 *    with PMRelease. The PMPrinter instance returned will be NULL if
 *    there is no printer available which corresponds to the supplied
 *    printerID.
 *  
 *  Parameters:
 *    
 *    printerID:
 *      The printerID for the printer for which you want the PMPrinter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCreateFromPrinterID( printerID: CFStringRef ): PMPrinter; external name '_PMPrinterCreateFromPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyDescriptionURL()
 *  
 *  Summary:
 *    Hand back a URL to the printer's PostScript Printer Description (PPD) file in fileURL.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose PPD will be returned.
 *    
 *    descriptionType:
 *      The type of description desired. Only kPMPPDDescriptionType is currently supported.
 *    
 *    fileURL:
 *      A pointer to storage for a CFURL reference that will be returned. The caller is responsible
 *      for releasing the reference returned. If this call returns an error, then *fileURL will
 *      be set to NULL.
 *
 *  Discussion:
 *    Only descriptionType of kPMPPDDescriptionType is supported, otherwise kPMInvalidParameter
 *    is returned.
 *
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyDescriptionURL( printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef ): OSStatus; external name '_PMPrinterCopyDescriptionURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyDeviceURI()
 *  
 *  Summary:
 *    Hand back the URI of the printer's device.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose device URI is to be retrieved.
 *    
 *    deviceURI:
 *      On exit, if successful *'deviceURI' will contain a reference to
 *      a CFURL describing the printer's device. The caller is
 *      responsible for releasing this reference. If this call returns
 *      an error, then *'deviceURI' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyDeviceURI( printer: PMPrinter; var deviceURI: CFURLRef ): OSStatus; external name '_PMPrinterCopyDeviceURI';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PMPrinterCopyHostName()
 *  
 *  Summary:
 *    Hand back the host name of the print server hosting the printer's print queue.
 *
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyHostName( printer: PMPrinter; var hostNameP: CFStringRef ): OSStatus; external name '_PMPrinterCopyHostName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterCopyPresets()
 *  
 *  Summary:
 *    Provides a list of print settings presets for the specified
 *    printer.
 *  
 *  Discussion:
 *    A printer may have associated with it a list of preset settings.
 *    Each setting is optimized for a particular printing situation.
 *    This function returns all of the presets for a given printer. To
 *    obtain more information about a particular preset see
 *    PMPresetGetAttributes().
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the presets for this printer.
 *    
 *    presetList:
 *      On exit, *'presetList' is set to reference an array of presets.
 *      The caller must call CFRelease when it no longer needs the
 *      array. Each element of the array is a PMPreset. If this
 *      function fails, returning a non-zero error code, then
 *      *'presetList' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterCopyPresets( printer: PMPrinter; var presetList: CFArrayRef ): OSStatus; external name '_PMPrinterCopyPresets';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetCommInfo()
 *  
 *  Summary:
 *    Provides information about the comm channel characteristics for
 *    the printer.
 *  
 *  Discussion:
 *    This function is typically relevant only to PostScript capable
 *    printers. All PostScript printers, regardless of what
 *    communications channel is used to send data to them, support data
 *    in the range 0x20 - 0x7F. Many comm channels can support data
 *    outside this range. The Boolean returned in *supportsTransparentP
 *    indicates whether the comm channel to this printer supports bytes
 *    in the range 0x0 to 0x1F. The Boolean returned in
 *    *supportsEightBitP indicates whether the comm channel to this
 *    printer supports bytes with the high bit set, i.e. bytes in the
 *    range 0x80 - 0xFF.
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the comm information for this printer.
 *    
 *    supportsControlCharRangeP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range
 *      0x0 - 0x1F
 *    
 *    supportsEightBitP:
 *      Storage for the returned Boolean indicating whether the comm
 *      channel to this printer can accept data bytes in the range 
 *      0x80 - 0xFF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetCommInfo( printer: PMPrinter; var supportsControlCharRangeP: Boolean; var supportsEightBitP: Boolean ): OSStatus; external name '_PMPrinterGetCommInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetID()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetID( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetLocation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLocation( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetDriverCreator()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDriverCreator( printer: PMPrinter; var creator: OSType ): OSStatus; external name '_PMPrinterGetDriverCreator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetDriverReleaseInfo()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDriverReleaseInfo( printer: PMPrinter; var release: VersRec ): OSStatus; external name '_PMPrinterGetDriverReleaseInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetPrinterResolutionCount()
 *  
 *  Summary:
 *    Provides the number of available hardware resolution settings a printer supports. 
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the resolution count for this printer.
 *    
 *    countP:
 *      Storage for the returned count indicating the number of available hardware
 *      resolution settings for the specified printer.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetPrinterResolutionCount( printer: PMPrinter; var countP: UInt32 ): OSStatus; external name '_PMPrinterGetPrinterResolutionCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  PMPrinterGetIndexedPrinterResolution()
 *  
 *  Summary:
 *    Together with PMPrinterGetPrinterResolutionCount, allows iterating over the
 *    hardware resolution settings a printer supports. 
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer of interest.
 *    
 *    index:
 *      The index of the resolution to return. The minimum value for index is 1 and
 *      the maximum value of index is the count returned by PMPrinterGetPrinterResolutionCount.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetIndexedPrinterResolution( printer: PMPrinter; index: UInt32; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetIndexedPrinterResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetOutputResolution()
 *  
 *  Summary:
 *    Obtain the printer hardware output resolution for the supplied
 *    printer and print settings. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterGetOutputResolution returns the best guess as to what printer
 *    resolution setting will be used for the destination print job. If the
 *    resolution setting cannot be reliably determined this function returns
 *    kPMKeyNotFound. 
 *  
 *    Most applications do not need to use this function since they draw the same
 *    content regardless of the destination device. For those few applications that
 *    do adjust their drawing based on the output device, they should only do so
 *    when the print job destination is kPMDestinationPrinter or kPMDestinationFax.
 *  
 *    This call should be used after displaying the print dialog to the user so
 *    that it correctly reflects settings changes performed prior to printing.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings from which to obtain the printer hardware output resolution.
 *    
 *    resolutionP:
 *      Storage for the returned PMResolution data.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; var resolutionP: PMResolution ): OSStatus; external name '_PMPrinterGetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPrinterSetOutputResolution()
 *  
 *  Summary:
 *    Set the printer hardware output resolution in the print settings
 *    for the supplied printer. 
 *  
 *  Discussion:
 *    Some printers allow programmatic control of their hardware output
 *    resolution on a print job basis. The hardware resolution is determined
 *    by the combination of printer and print settings used for the print job.
 *    PMPrinterSetOutputResolution configures the print settings to the closest
 *    resolution setting that can be used for the destination print job. Note
 *    that not all printers allow control of their resolution setting.
 *
 *    This function is rarely used since most applications do not set the output 
 *    resolution but instead use the setting supplied by the user in the print dialog.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The destination printer.
 *    
 *    printSettings:
 *      The print settings in which to set the hardware resolution.
 *    
 *    resolutionP:
 *      A pointer to the PMResolution to use to set the hardware output resolution.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterSetOutputResolution( printer: PMPrinter; printSettings: PMPrintSettings; const (*var*) resolutionP: PMResolution ): OSStatus; external name '_PMPrinterSetOutputResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrinterGetLanguageInfo()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetLanguageInfo( printer: PMPrinter; var info: PMLanguageInfo ): OSStatus; external name '_PMPrinterGetLanguageInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  PMPrinterGetMakeAndModelName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetMakeAndModelName( printer: PMPrinter; var makeAndModel: CFStringRef ): OSStatus; external name '_PMPrinterGetMakeAndModelName';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetMimeTypes()
 *  
 *  Summary:
 *    Return the array of mime type supported by the printer for a
 *    given set of print settings.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job. The part of the print
 *      settings that effects the available mime type is the
 *      destination. This parameter can be NULL.
 *    
 *    mimeTypes:
 *      If this function returns without error then *'mimeTypes' is
 *      filled in with a reference to an array of CFStrings. Each
 *      CFString names a mime type supported by the printer with the
 *      specified print settings. The caller must not release this
 *      reference without first doing a retain. If this function
 *      returns an error then 'mimeTypes' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetMimeTypes( printer: PMPrinter; settings: PMPrintSettings; var mimeTypes: CFArrayRef ): OSStatus; external name '_PMPrinterGetMimeTypes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetName( printer: PMPrinter ): CFStringRef; external name '_PMPrinterGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterGetPaperList()
 *  
 *  Summary:
 *    Returns the list of papers available for a given printer.
 *  
 *  Parameters:
 *    
 *    printer:
 *      Obtain the paper list for this printer.
 *    
 *    paperList:
 *      If successful noErr is returned and *paperList is a CFArray of
 *      PMPapers representing the list of papers available for the
 *      printer.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetPaperList( printer: PMPrinter; var paperList: CFArrayRef ): OSStatus; external name '_PMPrinterGetPaperList';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterGetState()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetState( printer: PMPrinter; var state: PMPrinterState ): OSStatus; external name '_PMPrinterGetState';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsDefault()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsDefault( printer: PMPrinter ): Boolean; external name '_PMPrinterIsDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsFavorite()
 *  
 *  Summary:
 *    Return true if the printer is in the user's favorite printer list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsFavorite( printer: PMPrinter ): Boolean; external name '_PMPrinterIsFavorite';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  PMPrinterIsPostScriptCapable()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsPostScriptCapable( printer: PMPrinter ): Boolean; external name '_PMPrinterIsPostScriptCapable';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

{!
 * @function	PMPrinterIsPostScriptPrinter
 * @abstract	Set *isPSPrinter true if the printer is a PostScript printer.
 *
 * @discussion	A PostScript printer is one whose driver takes PostScript directly.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsPostScriptPrinter( printer: PMPrinter; var isPSPrinter: Boolean ): OSStatus; external name '_PMPrinterIsPostScriptPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrinterIsRemote()
 *  
 *  Summary:
 *    Hand back a boolean indicating whether the printer is hosted by remote print server.
 *  
 *  Discussion:
 *    If on return *isRemoteP is true, the print queue represents a printer hosted and
 *    managed by a remote print server.
 *
 *    If on return *isRemoteP is false, the print queue represents a directly connected
 *    printer, a network printer, or a remote printer that is locally managed. Consult
 *    the queue's device URI to determine the type of connection that is used to communicate
 *    with the printer.
 *
 *    Whether a printer is remote is derived from the CUPS printer-type attribute for the print queue.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterIsRemote( printer: PMPrinter; var isRemoteP: Boolean ): OSStatus; external name '_PMPrinterIsRemote';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterSetDefault()
 *  
 *  Summary:
 *    Set the default printer for the current user.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer to set as the default printer.
 *  
 *  Special considerations:
 *      It is not typical for an application to set the current default printer
 *      for the user; the printing system itself takes care of managing the default printer.
 *      This function should be used only in rare circumstances.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterSetDefault( printer: PMPrinter ): OSStatus; external name '_PMPrinterSetDefault';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark Preset
#pragma mark
*)
{
 *  PMPresetCopyName()
 *  
 *  Summary:
 *    Hand back a copy of the localized name for the specified preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      The preset whose name is needed.
 *    
 *    name:
 *      On exit, if this routine succeeds, *'name' is filled in with a
 *      reference to a localized string with the preset's name. If this
 *      routine fails, then *'name' is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPresetCopyName( preset: PMPreset; var name: CFStringRef ): OSStatus; external name '_PMPresetCopyName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPresetCreatePrintSettings()
 *  
 *  Summary:
 *    Create a print settings conforming to the specified print
 *    settings preset.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A preset specifying a set of initial print settings.
 *    
 *    session:
 *      A valid print session.
 *    
 *    printSettings:
 *      On exit, *'printSettings' is set to a newly created print
 *      settings that contains the settings specified by 'preset'. The
 *      caller is responsible for calling PMRelease when the print
 *      settings are no longer needed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPresetCreatePrintSettings( preset: PMPreset; session: PMPrintSession; var printSettings: PMPrintSettings ): OSStatus; external name '_PMPresetCreatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPresetGetAttributes()
 *  
 *  Summary:
 *    Hand back the meta-data describing a given preset.
 *  
 *  Discussion:
 *    Each preset has associated with it a dictionary containing
 *    meta-data. The meta-data provides the preset's id, the preset's
 *    localized names, and descriptions of the environment for which
 *    the preset it intended.
 *  
 *  Parameters:
 *    
 *    preset:
 *      A print settings preset as obtained from PMPrinterCopyPresets().
 *    
 *    attributes:
 *      On exit, *'attributes' is set to reference a dictionary
 *      containing the preset's meta-data. The caller is responsible
 *      for retaining this reference if it is to be used beyond the
 *      lifetime of 'preset'. If this function fails, returning a
 *      non-zero error code, then *'attributes' is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPresetGetAttributes( preset: PMPreset; var attributes: CFDictionaryRef ): OSStatus; external name '_PMPresetGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Paper
#pragma mark
*)
{
 *  PMGetPageFormatPaper()
 *  
 *  Summary:
 *    Returns the paper associated with a pageformat.
 *  
 *  Parameters:
 *    
 *    format:
 *      Obtain the paper for this pageformat.
 *    
 *    paper:
 *      If successful noErr is returned and *paper will contain a
 *      PMPaper object describing the current paper associated with the
 *      pageformat.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPageFormatPaper( format: PMPageFormat; var paper: PMPaper ): OSStatus; external name '_PMGetPageFormatPaper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperCreateCustom()
 *  
 *  Summary:
 *    Create a new custom paper instance.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The new paper size is appropriate for this printer.
 *    
 *    id:
 *      A unique identifier for this paper type.
 *    
 *    name:
 *      The name to display to the user for this paper type.
 *    
 *    width:
 *      The width, in points, of the paper.
 *    
 *    height:
 *      The height, in points, of the paper.
 *    
 *    margins:
 *      The unprintable margins on the paper.
 *    
 *    paperP:
 *      if this function is successful, returning noErr, then *'paperP'
 *      is set to be a reference to a newly created PMPaper instance.
 *      The caller is responsible for calling PMRelease when the
 *      instance is no longer needed. If this functions fails, it will
 *      return a non-zero error and set *'paperP' to NULL.
 *  
 *  Discussion:
 *		This function creates a new custom paper instance. To obtain one of the available
 *		built-in paper sizes for a given printer, use PMPrinterGetPaperList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperCreateCustom( printer: PMPrinter; id: CFStringRef; name: CFStringRef; width: Float64; height: Float64; const (*var*) margins: PMPaperMargins; var paperP: PMPaper ): OSStatus; external name '_PMPaperCreateCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetWidth()
 *  
 *  Summary:
 *    Returns the width for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the width for this paper.
 *    
 *    paperWidth:
 *      If successful noErr is returned and *paperWidth is set to the
 *      width of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetWidth( paper: PMPaper; var paperWidth: Float64 ): OSStatus; external name '_PMPaperGetWidth';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetHeight()
 *  
 *  Summary:
 *    Returns the height for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the height for this paper.
 *    
 *    paperHeight:
 *      If successful noErr is returned and *paperHeight is set to the
 *      height of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetHeight( paper: PMPaper; var paperHeight: Float64 ): OSStatus; external name '_PMPaperGetHeight';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetMargins()
 *  
 *  Summary:
 *    Returns the margins for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the margin information for this paper.
 *    
 *    paperMargins:
 *      If successful noErr is returned and *paperMargins is set to the
 *      margins of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetMargins( paper: PMPaper; var paperMargins: PMPaperMargins ): OSStatus; external name '_PMPaperGetMargins';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPaperGetID()
 *  
 *  Summary:
 *    Returns the id for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the id for this paper.
 *    
 *    paperID:
 *      If successful noErr is returned and *paperID is set to the id
 *      of the paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetID( paper: PMPaper; var paperID: CFStringRef ): OSStatus; external name '_PMPaperGetID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetName()
 *  
 *  Summary:
 *    Returns the name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the name for this paper.
 *    
 *    paperName:
 *      If successful noErr is returned and *paperName is set to the
 *      name of the paper. The returned paper name is that appropriate
 *      to display to a user.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPaperGetPPDPaperName()
 *  
 *  Summary:
 *    Returns the PPD name for a given paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the PPD name for this paper.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      PPD name of the paper. The returned result may be NULL.
 *  
 *  Discussion:
 *      The Mac OS X printing system uses a PostScript Printer Description (PPD)
 *      file to describe a given printer and print queue for that printer. The
 *      PPD name is the name that uniquely identifies a given paper
 *      to the printer to which the paper corresponds. 
 *    
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetPPDPaperName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetPPDPaperName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPaperCreateLocalizedName()
 *  
 *  Summary:
 *    Returns the localized name for a given paper. 
 *    Use of PMPaperCreateLocalizedName
 *    is recommended instead of PMPaperGetName.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the localized name for this paper.
 *    
 *    printer:
 *      The printer for which the localization should be performed.
 *    
 *    paperName:
 *      If successful, noErr is returned and *paperName is set to the
 *      localized name of the paper. The returned paper name is that appropriate
 *      to display to a user. The returned result may be NULL.
 *      If non-NULL, the caller is responsible for releasing the name returned. 
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperCreateLocalizedName( paper: PMPaper; printer: PMPrinter; var paperName: CFStringRef ): OSStatus; external name '_PMPaperCreateLocalizedName';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperGetPrinterID()
 *  
 *  Summary:
 *    Returns the printerID of the printer for which the paper corresponds.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Obtain the printer ID for this paper.
 *    
 *    printerID:
 *      If successful, noErr is returned and *printerID is set to the
 *      ID of the printer for which the paper corresponds. The returned
 *      result may be NULL. 
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperGetPrinterID( paper: PMPaper; var printerID: CFStringRef ): OSStatus; external name '_PMPaperGetPrinterID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  PMPaperIsCustom()
 *  
 *  Summary:
 *    Returns true if the paper is a custom paper.
 *  
 *  Parameters:
 *    
 *    paper:
 *      Determines if this is a custom paper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPaperIsCustom( paper: PMPaper ): Boolean; external name '_PMPaperIsCustom';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark PDF Workflow
#pragma mark
*)
{
 *  PMWorkflowCopyItems()
 *  
 *  Summary:
 *    Hand back an array of dictionaries describing the PDF Workflow
 *    items installed on the system
 *  
 *  Parameters:
 *    
 *    workflowItems:
 *      If this function returns without error then *'workflowItems'
 *      will be filled in with a reference to an array. It is the
 *      caller's responsability to release the array when done with it.
 *      Each element in the array describes a PDF Workflow item or a
 *      folder holding workflow items. A dictionary describing a
 *      workflow item has, at least, the following keys and values:
 *      displayName - The user's diaplayable name for the workflow item
 *      itemURL - A CFURLRef pointing to the workflow item. A
 *      dictionary describing a workflow folder has at least the
 *      following keys: displayName - The user's diaplayable name for
 *      the workflow item folderURL - A CFURLRef pointing to the
 *      folder. items - A CFArrayRef describing the workflow items in
 *      the folder. If this function returns a non-zero error code then
 *      *'workflowItems' will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowCopyItems( var workflowItems: CFArrayRef ): OSStatus; external name '_PMWorkflowCopyItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithOptions()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    title:
 *      The user displayable name of the document.
 *    
 *    options:
 *      A string of CUPS style key-value pairs that may be passed to
 *      the PDF Workflow item. This parameter can be NULL in which case
 *      an empty string of options is used.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowSubmitPDFWithOptions( workflowItem: CFURLRef; title: CFStringRef; options: ConstCStringPtr; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMWorkflowSubmitPDFWithSettings()
 *  
 *  Summary:
 *    Submit a PDF file for workflow processing.
 *  
 *  Discussion:
 *    The print dialog uses this function is conjunction with
 *    PMWorkflowGetItems to implement the PDF workflow button. Caller's
 *    can use PMWorkflowGetItems to obtain a CFURLRef that can be
 *    passed to PMWorkflowPDF or they can create a CFURLRef to another
 *    file system item.
 *  
 *  Parameters:
 *    
 *    workflowItem:
 *      A file system URL pointing to the workflow item that will
 *      handle the PDF file. Here are the different types of workflow
 *      items currently supported: Folder alias:   The PDF is moved to
 *      the resolved folder. Application or application alias: The
 *      application is sent an open event along with a reference to the
 *      PDF file. Compiled data fork AppleScript: The applescript is
 *      run with an open event along with a reference to the PDF file.
 *      executable tool: The tool is run with the following parameters:
 *      title options pdfFile
 *    
 *    settings:
 *      The prints settings to apply to the PDF.
 *    
 *    pdfFile:
 *      A file system URL pointing to the file to be processed by the
 *      workflow item.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMWorkflowSubmitPDFWithSettings( workflowItem: CFURLRef; settings: PMPrintSettings; pdfFile: CFURLRef ): OSStatus; external name '_PMWorkflowSubmitPDFWithSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark Job Submission
#pragma mark
*)
{
 *  PMPrinterPrintWithProvider()
 *  
 *  Summary:
 *    Submit print data to a specified printer.
 *  
 *  Discussion:
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. This parameter can not
 *      be NULL. Use PMPrinterPrintWithFile() if auto-typing is desired.
 *    
 *    provider:
 *      The data provider that supplies the print data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterPrintWithProvider( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; provider: CGDataProviderRef ): OSStatus; external name '_PMPrinterPrintWithProvider';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrinterPrintWithFile()
 *  
 *  Summary:
 *    Submit a file for printing to a specified printer.
 *  
 *  Discussion:
 *
 *    One reason this function may fail is if the specified printer can
 *    not handle the file's mime type. Use PMPrinterGetMimeTypes() to
 *    check whether a mime type is supported.
 *  
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Parameters:
 *    
 *    printer:
 *      The printer.
 *    
 *    settings:
 *      The print settings for the print job.
 *    
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed. This parameter can be NULL.
 *    
 *    mimeType:
 *      The mime type of the file to be printed. If this parameter is
 *      NULL then the supplied file will be auto-typed.
 *    
 *    fileURL:
 *      A file URL specifying the file to be printed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterPrintWithFile( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; fileURL: CFURLRef ): OSStatus; external name '_PMPrinterPrintWithFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
 *  PMPrinterWritePostScriptToURL()
 *
 *  Summary:
 *    Convert an input file of the specified mimeType to printer ready PostScript for the destination printer.
 *
 *  Discussion:
 *
 *    The conversion of the input file to PostScript is performed before this function returns. This can take a significant
 *    amount of time for longer documents. The caller of PMPrinterWritePostScriptToURL may want to perform this operation
 *    on a thread other than the main application thread or fork a separate process for this purpose.
 *
 *    One reason this function may fail is if conversion from the input mimeType to PostScript cannot be performed.
 *
 *
 *  Parameters:
 *
 *    printer:
 *      The printer for which the printer ready PostScript will be generated.
 *
 *    settings:
 *      The print settings for the print job.
 *
 *    format:
 *      The page format specifying the physical page size and orientation on which the document
 *      should be printed.
 *
 *    mimeType:
 *      The mime type of the file to be printed. If NULL, the file is auto-typed.
 *
 *    sourceFileURL:
 *      A file URL specifying the input file to be converted to printer ready PostScript data. Only file based URLs
 *      are supported.
 *
 *    destinationFileURL:
 *      A file URL specifying the destination file to be created. If the file already exists it will be overwritten. Only
 *      file based URLs are supported.
 *
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrinterWritePostScriptToURL( printer: PMPrinter; settings: PMPrintSettings; format: PMPageFormat; mimeType: CFStringRef; sourceFileURL: CFURLRef; destinationFileURL: CFURLRef ): OSStatus; external name '_PMPrinterWritePostScriptToURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  PMPrintSettingsToOptions()
 *  
 *  Summary:
 *    Convert print settings to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'ed C string
 *      describing the passed in print settings. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsToOptions( settings: PMPrintSettings; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMPrintSettingsToOptionsWithPrinterAndPageFormat()
 *  
 *  Summary:
 *    Convert a print settings and page format to a CUPS style options string.
 *  
 *  Parameters:
 *    
 *    settings:
 *      The print settings that should be converted to a CUPS style
 *      options string. Must not be NULL.
 *    
 *    printer:
 *      The printer to use for converting the print settings. Must not be NULL.
 *
 *    format:
 *      The page format for use in creating the cups options. Can be NULL.
 *    
 *    options:
 *      On exit *'options' will be filled in with a malloc'd C string
 *      describing the passed in print settings and format. It is the caller's
 *      responsibility to free this memory when done with it. If this
 *      function fails returning a non-zero error code then *'options'
 *      will be set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function PMPrintSettingsToOptionsWithPrinterAndPageFormat( settings: PMPrintSettings; printer: PMPrinter; pageFormat: PMPageFormat; var options: CStringPtr ): OSStatus; external name '_PMPrintSettingsToOptionsWithPrinterAndPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

(*
#pragma mark
#pragma mark PrinterCommandSupport
#pragma mark
*)

{
*  PMPrinterSendCommand()
*
*  Summary:
*    Submit a command file containing the specified string data to a print queue.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    commandString:
*      The contents of the command file as a CFStringRef.
*
*    jobTitle:
*      The title of the job associated with the command file. If NULL,
*      a job title will be automatically generated but it may not be
*      meaningful to a user.
*
*    options:
*      A dictionary containing options that apply to the job.
*      This parameter should be NULL; it is reserved for future expansion.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}
function PMPrinterSendCommand( printer: PMPrinter; commandString: CFStringRef; jobTitle: CFStringRef; options: CFDictionaryRef ): OSStatus; external name '_PMPrinterSendCommand';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{
*  PMPrinterCopyState()
*
*  Summary:
*    Copies the current "marker-change-time", "marker-colors",
*    "marker-high-levels", "marker-levels", "marker-low-levels",
*    "marker-message", "marker-names", "marker-types", "printer-state",
*    "printer-state-change-time", "printer-state-message", and
*    "printer-state-reasons" values for a print queue.  These attributes are
*    defined in the "CUPS Implementation of IPP" specification at:
*
*        http://www.cups.org/documentation.php/spec-ipp.html
*        http://localhost:631/help/spec-ipp.html
*
*    Each attribute is returned using the corresponding CoreFoundation
*    data type:
*
*        Attribute                    Type
*        ---------------------------  ------------------------
*        "marker-change-time"         CFDateRef
*        "marker-colors"              CFArrayRef + CFStringRef
*        "marker-high-levels"         CFArrayRef + CFNumberRef
*        "marker-levels"              CFArrayRef + CFNumberRef
*        "marker-low-levels"          CFArrayRef + CFNumberRef
*        "marker-message"             CFStringRef
*        "marker-names"               CFArrayRef + CFStringRef
*        "marker-types"               CFArrayRef + CFStringRef
*        "printer-state"              CFNumberRef
*        "printer-state-change-time"  CFDateRef
*        "printer-state-message"      CFStringRef
*        "printer-state-reasons"      CFArrayRef + CFStringRef
*
*  Localization Considerations:
*
*    The "marker-message" and "printer-state-message" attributes may be
*    localized by the printer driver using the language of the last processed
*    print job.
*
*    The "marker-names" and "printer-state-reasons" attributes are typically
*    English-language keywords that may be localized using the printer's PPD
*    file and CUPS ppdLocalizeMarkerName and ppdLocalizeIPPReason APIs,
*    respectively.
*
*    All other attributes are predefined keywords or values that are not
*    localized.
*
*  Parameters:
*
*    printer:
*      The destination print queue.
*
*    stateDict:
*      A pointer to a CFDictionaryRef variable to hold the values.
*      You must release the dictionary when you are done using it.
*      The dictionary returned is not mutable.
*
*  Availability:
*    Mac OS X:         in version 10.6 and later in ApplicationServices.framework
*    CarbonLib:        not available
}

function PMPrinterCopyState( printer: PMPrinter; var stateDict: CFDictionaryRef ): OSStatus; external name '_PMPrinterCopyState';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

(*
#pragma mark
#pragma mark PPD related
#pragma mark
*)
{
 *  PMCopyAvailablePPDs()
 *  
 *  Summary:
 *    Hand back the list of PPDs in the specified PPD domain.
 *  
 *  Parameters:
 *    
 *    domain:
 *      The domain to search for PPDs.
 *    
 *    ppds:
 *      If this function completes without error, *'ppds' is set to an
 *      array of CFURLs. Each CFURL specifies the location of a PPD
 *      file or a compressed PPD file. The caller is responsible for
 *      releasing the array. If this function returns a non-zero error
 *      code then *'ppds' is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyAvailablePPDs( domain: PMPPDDomain; var ppds: CFArrayRef ): OSStatus; external name '_PMCopyAvailablePPDs';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMCopyLocalizedPPD()
 *  
 *  Summary:
 *    Hand back a reference to a localized PPD.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A PPD reference. Typically this is a CFURLRef returned from
 *      PMCopyAvailablePPDs().
 *    
 *    localizedPPD:
 *      If this function completes without error, *'localizedPPD' will
 *      be set to a CFURLRef referencing the PPD that should be used
 *      given the current user's language preferences. If this function
 *      returns an error then *'localizedPPD' will be set to to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyLocalizedPPD( ppd: CFURLRef; var localizedPPD: CFURLRef ): OSStatus; external name '_PMCopyLocalizedPPD';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  PMCopyPPDData()
 *  
 *  Summary:
 *    Hand back the uncompressed PPD data for a PPD or compressed PPD
 *    file.
 *  
 *  Parameters:
 *    
 *    ppd:
 *      A reference to a PPD or compressed PPD file. This reference is
 *      usually obtained from PMCopyAvailablePPDs() or from
 *      PMCopyLocalizedPPD().
 *    
 *    data:
 *      If this function completes without error then *'data' is set to
 *      reference the uncompressed PPD data from the PPD file. If this
 *      function returns a non-zero error then *'data is set to NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMCopyPPDData( ppd: CFURLRef; var data: CFDataRef ): OSStatus; external name '_PMCopyPPDData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

(*
#pragma mark
#pragma mark EPS related
#pragma mark
*)
{
 *  PMCGImageCreateWithEPSDataProvider()
 *  
 *  Summary:
 *    Create an image reference that references both the PostScript
 *    contents of an EPS file and a preview (proxy) image for that EPS
 *    file.
 *  
 *  Discussion:
 *    For OS X 10.1.0, this function ignores the passed in data
 *    provider. The passed in image reference is retained and then
 *    returned. For 10.1.1 and later, then the data provider is used
 *    and the returned image reference is different than the passed in
 *    image reference, so please be careful with your use of these
 *    references. It is likely that the data will not be read from the
 *    EPS data provider until well after this function returns. The
 *    caller should be careful not to free the underlying EPS data
 *    until the provider's release routine is invoked. Similarly the
 *    preview image's data may be needed long after you think it should
 *    be. Do not free the image data until the image data provider's
 *    release data function has been called. To make sure these data
 *    providers are properly reference counted, release your reference
 *    the EPS data provider and on the EPS image preview when they are
 *    no longer needed by your application. For Mac OS X 10.2 and
 *    later, the contents of the EPS provider at the time of this call
 *    can be dumped to a file if you first do the following, BEFORE
 *    running your application. 
 *
 *	  From the command line in terminal:
 *		defaults write NSGlobalDomain com.apple.print.eps.testProvider /tmp/dump.eps
 *    
 *	  causes a dump of the EPS data into a file /tmp/dump.eps.
 *  
 *  Parameters:
 *    
 *    epsDataProvider:
 *      A Core Graphics data provider that can supply the PostScript
 *      contents of the EPS file. Post OS X 10.1, there will be some
 *      checking done on the EPS data provided to the
 *      PMCGImageCreateWithEPSDataProvider() call. It is important that
 *      the EPS data begin with the EPSF required header and bounding
 *      box DSC comments.
 *    
 *    epsPreview:
 *      A Core Graphics image reference to the proxy image for the EPS
 *      file. When the image reference result of this function is
 *      rendered on screen or printed to a printer that can not render
 *      PostScript this proxy image is drawn instead.
 *  
 *  Result:
 *    an image reference capable of rendering either the EPS content or
 *    the proxy image depending upon the capabilities of the targeted
 *    context.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMCGImageCreateWithEPSDataProvider( epsDataProvider: CGDataProviderRef; epsPreview: CGImageRef ): CGImageRef; external name '_PMCGImageCreateWithEPSDataProvider';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

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
