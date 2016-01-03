{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/QTML.h
 
     Contains:   QuickTime Cross-platform specific interfaces
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    QuickTime 7.7.1
 
     Copyright:  © 1997-2012 by Apple Inc., all rights reserved.
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       QTML.p
=======
     File:       QuickTime/QTML.h
>>>>>>> origin/fixes_2.4
 
     Contains:   QuickTime Cross-platform specific interfaces
 
     Version:    QuickTime 7.6.3
 
<<<<<<< HEAD
     Copyright:  © 1997-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
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
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit QTML;
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
>>>>>>> graemeg/cpstrnew
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
=======

>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
{$ifc TARGET_OS_MAC}

{ QuickTime is not available to 64-bit clients }

=======
{ QuickTime is not available to 64-bit clients }

>>>>>>> origin/cpstrnew
{$ifc not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Quickdraw,Events,MacMemory,MacWindows,OSUtils,Files;
=======
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{ QuickTime is not available to 64-bit clients }

<<<<<<< HEAD
{$ifc CALL_NOT_IN_CARBON}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  QTMLYieldCPU()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure QTMLYieldCPU; external name '_QTMLYieldCPU';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ QTMLYieldCPUTime flags}
const
	kQTMLHandlePortEvents = 1 shl 0; { ask for event handling during the yield}
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure QTMLYieldCPU; external name '_QTMLYieldCPU';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ QTMLYieldCPUTime flags}
const
	kQTMLHandlePortEvents = 1 shl 0; { ask for event handling during the yield}
>>>>>>> origin/fixes_2.4

{
 *  QTMLYieldCPUTime()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure QTMLYieldCPUTime( milliSeconds: SIGNEDLONG; flags: UNSIGNEDLONG ); external name '_QTMLYieldCPUTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	QTMLMutex = ^OpaqueQTMLMutex; { an opaque type }
	OpaqueQTMLMutex = record end;
	QTMLMutexPtr = ^QTMLMutex; { when a var xx:QTMLMutex parameter can be nil, it is changed to xx: QTMLMutexPtr }
	QTMLSyncVar = ^OpaqueQTMLSyncVar; { an opaque type }
	OpaqueQTMLSyncVar = record end;
=======
	QTMLMutex = ^SInt32; { an opaque type }
	QTMLMutexPtr = ^QTMLMutex; { when a var xx:QTMLMutex parameter can be nil, it is changed to xx: QTMLMutexPtr }
	QTMLSyncVar = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	QTMLMutex = ^SInt32; { an opaque type }
	QTMLMutexPtr = ^QTMLMutex; { when a var xx:QTMLMutex parameter can be nil, it is changed to xx: QTMLMutexPtr }
	QTMLSyncVar = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	QTMLMutex = ^SInt32; { an opaque type }
	QTMLMutexPtr = ^QTMLMutex; { when a var xx:QTMLMutex parameter can be nil, it is changed to xx: QTMLMutexPtr }
	QTMLSyncVar = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	QTMLMutex = ^SInt32; { an opaque type }
	QTMLMutexPtr = ^QTMLMutex; { when a var xx:QTMLMutex parameter can be nil, it is changed to xx: QTMLMutexPtr }
	QTMLSyncVar = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	QTMLSyncVarPtr = ^QTMLSyncVar;  { when a var xx:QTMLSyncVar parameter can be nil, it is changed to xx: QTMLSyncVarPtr }
{ InitializeQTML flags}
const
	kInitializeQTMLNoSoundFlag = 1 shl 0; { flag for requesting no sound when calling InitializeQTML}
	kInitializeQTMLUseGDIFlag = 1 shl 1; { flag for requesting GDI when calling InitializeQTML}
	kInitializeQTMLDisableDirectSound = 1 shl 2; { disables QTML's use of DirectSound}
	kInitializeQTMLUseExclusiveFullScreenModeFlag = 1 shl 3; { later than QTML 3.0: qtml starts up in exclusive full screen mode}
	kInitializeQTMLDisableDDClippers = 1 shl 4; { flag for requesting QTML not to use DirectDraw clipper objects; QTML 5.0 and later}
	kInitializeQTMLEnableDoubleBufferedSurface = 1 shl 6; { flag for requesting QuickTime use a double-buffered destination surface; QT6.4 and later}

{
 *  InitializeQTML()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure QTMLYieldCPUTime( milliSeconds: SIGNEDLONG; flags: UNSIGNEDLONG ); external name '_QTMLYieldCPUTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


type
	QTMLMutex = ^SInt32; { an opaque type }
	QTMLMutexPtr = ^QTMLMutex; { when a var xx:QTMLMutex parameter can be nil, it is changed to xx: QTMLMutexPtr }
	QTMLSyncVar = ^SInt32; { an opaque type }
	QTMLSyncVarPtr = ^QTMLSyncVar;  { when a var xx:QTMLSyncVar parameter can be nil, it is changed to xx: QTMLSyncVarPtr }
{ InitializeQTML flags}
const
<<<<<<< HEAD
	kInitializeQTMLNoSoundFlag	= $00000001;					{  flag for requesting no sound when calling InitializeQTML }
	kInitializeQTMLUseGDIFlag	= $00000002;					{  flag for requesting GDI when calling InitializeQTML }
	kInitializeQTMLDisableDirectSound = $00000004;				{  disables QTML's use of DirectSound }
	kInitializeQTMLUseExclusiveFullScreenModeFlag = $00000008;	{  later than QTML 3.0: qtml starts up in exclusive full screen mode }
	kInitializeQTMLDisableDDClippers = $00000010;				{  flag for requesting QTML not to use DirectDraw clipper objects; QTML 5.0 and later }

	kQTMLHandlePortEvents		= $00000001;					{  flag for requesting requesting QTML to handle events }
	kQTMLNoIdleEvents			= $00000002;					{  flag for requesting requesting QTML not to send Idle Events }

{$ifc CALL_NOT_IN_CARBON}
	{
	 *  InitializeQTML()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   not available
	 *    CarbonLib:        not available
	 *    Mac OS X:         not available
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function InitializeQTML(flag: SInt32): OSErr; external name '_InitializeQTML';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kInitializeQTMLNoSoundFlag = 1 shl 0; { flag for requesting no sound when calling InitializeQTML}
	kInitializeQTMLUseGDIFlag = 1 shl 1; { flag for requesting GDI when calling InitializeQTML}
	kInitializeQTMLDisableDirectSound = 1 shl 2; { disables QTML's use of DirectSound}
	kInitializeQTMLUseExclusiveFullScreenModeFlag = 1 shl 3; { later than QTML 3.0: qtml starts up in exclusive full screen mode}
	kInitializeQTMLDisableDDClippers = 1 shl 4; { flag for requesting QTML not to use DirectDraw clipper objects; QTML 5.0 and later}
	kInitializeQTMLEnableDoubleBufferedSurface = 1 shl 6; { flag for requesting QuickTime use a double-buffered destination surface; QT6.4 and later}
>>>>>>> origin/fixes_2.4

{
 *  InitializeQTML()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


{ CreatePortAssociation flags}
const
	kQTMLNoIdleEvents = 1 shl 1; { ask for a non-auto-idled port to be created}
	kQTMLNoDoubleBufferPort = 1 shl 2; { ask for QTML not to double-buffer this port}

const
	kQTMLIsDoubleBuffered = 'UsesDoubleBuffer';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
=======
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


{
 *  TerminateQTML()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ CreatePortAssociation flags}
const
	kQTMLNoIdleEvents = 1 shl 1; { ask for a non-auto-idled port to be created}
	kQTMLNoDoubleBufferPort = 1 shl 2; { ask for QTML not to double-buffer this port}

const
	kQTMLIsDoubleBuffered = 'UsesDoubleBuffer';
>>>>>>> origin/fixes_2.4
{
 *  CreatePortAssociation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function CreatePortAssociation(theWnd: UnivPtr; storage: Ptr; flags: SInt32): GrafPtr; external name '_CreatePortAssociation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  DestroyPortAssociation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure DestroyPortAssociation(cgp: CGrafPtr); external name '_DestroyPortAssociation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4


{
 *  QTMLGrabMutex()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure QTMLGrabMutex( mu: QTMLMutex ); external name '_QTMLGrabMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLGrabMutex(mu: QTMLMutex); external name '_QTMLGrabMutex';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure QTMLGrabMutex( mu: QTMLMutex ); external name '_QTMLGrabMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTMLTryGrabMutex()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 4.1 and later
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 4.1 and later
 }
function QTMLTryGrabMutex( mu: QTMLMutex ): Boolean; external name '_QTMLTryGrabMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 4.1 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTMLTryGrabMutex(mu: QTMLMutex): boolean; external name '_QTMLTryGrabMutex';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTMLTryGrabMutex( mu: QTMLMutex ): Boolean; external name '_QTMLTryGrabMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTMLReturnMutex()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure QTMLReturnMutex( mu: QTMLMutex ); external name '_QTMLReturnMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLReturnMutex(mu: QTMLMutex); external name '_QTMLReturnMutex';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure QTMLReturnMutex( mu: QTMLMutex ); external name '_QTMLReturnMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTMLCreateMutex()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTMLCreateMutex: QTMLMutex; external name '_QTMLCreateMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTMLCreateMutex: QTMLMutex; external name '_QTMLCreateMutex';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTMLDestroyMutex()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure QTMLDestroyMutex( mu: QTMLMutex ); external name '_QTMLDestroyMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 3.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLDestroyMutex(mu: QTMLMutex); external name '_QTMLDestroyMutex';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure QTMLDestroyMutex( mu: QTMLMutex ); external name '_QTMLDestroyMutex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  QTMLCreateSyncVar()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function QTMLCreateSyncVar: QTMLSyncVarPtr; external name '_QTMLCreateSyncVar';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTMLDestroySyncVar()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLDestroySyncVar(p: QTMLSyncVarPtr); external name '_QTMLDestroySyncVar';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTMLTestAndSetSyncVar()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function QTMLTestAndSetSyncVar(sync: QTMLSyncVarPtr): SInt32; external name '_QTMLTestAndSetSyncVar';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTMLWaitAndSetSyncVar()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLWaitAndSetSyncVar(sync: QTMLSyncVarPtr); external name '_QTMLWaitAndSetSyncVar';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTMLResetSyncVar()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLResetSyncVar(sync: QTMLSyncVarPtr); external name '_QTMLResetSyncVar';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4


{
 *  InitializeQHdr()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure InitializeQHdr(var qhdr_: QHdr); external name '_InitializeQHdr';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  TerminateQHdr()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure TerminateQHdr(var qhdr_: QHdr); external name '_TerminateQHdr';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4


{
 *  QTMLAcquireWindowList()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLAcquireWindowList; external name '_QTMLAcquireWindowList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTMLReleaseWindowList()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLReleaseWindowList; external name '_QTMLReleaseWindowList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
   These routines are here to support "interrupt level" code
      These are dangerous routines, only use if you know what you are doing.
}

{
 *  QTMLRegisterInterruptSafeThread()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function QTMLRegisterInterruptSafeThread(threadID: UInt32; threadInfo: UnivPtr): SInt32; external name '_QTMLRegisterInterruptSafeThread';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTMLUnregisterInterruptSafeThread()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function QTMLUnregisterInterruptSafeThread(threadID: UInt32): SInt32; external name '_QTMLUnregisterInterruptSafeThread';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4


{
 *  NativeEventToMacEvent()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


{$ifc TARGET_OS_WIN32}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


{$ifc TARGET_OS_WIN32}
<<<<<<< HEAD
{$ifc CALL_NOT_IN_CARBON}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  WinEventToMacEvent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function WinEventToMacEvent(winMsg: UnivPtr; var macEvent: EventRecord): SInt32; external name '_WinEventToMacEvent';

{
 *  IsTaskBarVisible()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
 *    M *    Mac OS X:         not available
 *    CarbonLib:        not available
>>>>>>> graemeg/cpstrnew
=======
 *    M *    Mac OS X:         not available
 *    CarbonLib:        not available
>>>>>>> graemeg/cpstrnew
=======
 *    M *    Mac OS X:         not available
 *    CarbonLib:        not available
>>>>>>> graemeg/cpstrnew
=======
 *    M *    Mac OS X:         not available
 *    CarbonLib:        not available
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
	kDDSurfaceLocked = 1 shl 0;
	kDDSurfaceStatic = 1 shl 1;
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
 *  QTGetDDObject()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

const
	kDDSurfaceLocked = 1 shl 0;
	kDDSurfaceStatic = 1 shl 1;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{
 *  QTGetDDObject()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         not available
=======
 *    M *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


const
	kDDSurfaceLocked = 1 shl 0;
	kDDSurfaceStatic = 1 shl 1;

{
 *  QTGetDDObject()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

<<<<<<< HEAD
{$endc}  {CALL_NOT_IN_CARBON}

const
	kDDSurfaceLocked			= $00000001;
	kDDSurfaceStatic			= $00000002;

{$ifc CALL_NOT_IN_CARBON}
	{
	 *  QTGetDDObject()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   not available
	 *    CarbonLib:        not available
	 *    Mac OS X:         not available
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function QTGetDDObject(var lpDDObject: UnivPtr): OSErr; external name '_QTGetDDObject';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  QTSetDDObject()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function QTSetDDObject(lpNewDDObject: UnivPtr): OSErr; external name '_QTSetDDObject';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTSetDDPrimarySurface()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function QTSetDDPrimarySurface(lpNewDDSurface: UnivPtr; flags: UInt32): OSErr; external name '_QTSetDDPrimarySurface';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4


{
 *  QTMLGetVolumeRootPath()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function QTMLGetVolumeRootPath(fullPath: CStringPtr; volumeRootPath: CStringPtr; volumeRootLen: UInt32): OSErr; external name '_QTMLGetVolumeRootPath';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4


{
 *  QTMLSetWindowWndProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
procedure QTMLSetWindowWndProc(theWindow: WindowRef; windowProc: UnivPtr); external name '_QTMLSetWindowWndProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTMLGetWindowWndProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


{$endc}  { TARGET_OS_WIN32 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

<<<<<<< HEAD
{$endc}  {CALL_NOT_IN_CARBON}
{$endc}  {TARGET_OS_WIN32}
{$ifc CALL_NOT_IN_CARBON}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$endc}  { TARGET_OS_WIN32 }

>>>>>>> origin/fixes_2.4
{
 *  QTMLGetCanonicalPathName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


const
	kFullNativePath = 0;
	kFileNameOnly = 1 shl 0;
	kDirectoryPathOnly = 1 shl 1;
	kUFSFullPathName = 1 shl 2;
	kTryVDIMask = 1 shl 3; {    Used in NativePathNameToFSSpec to specify to search VDI mountpoints}
	kFullPathSpecifiedMask = 1 shl 4; {    the passed in name is a fully qualified full path}

{
 *  FSSpecToNativePathName()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


const
	kErrorIfFileNotFound = 1 shl 31;

{
 *  NativePathNameToFSSpec()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


const
	kFullNativePath = 0;
	kFileNameOnly = 1 shl 0;
	kDirectoryPathOnly = 1 shl 1;
	kUFSFullPathName = 1 shl 2;
	kTryVDIMask = 1 shl 3; {    Used in NativePathNameToFSSpec to specify to search VDI mountpoints}
	kFullPathSpecifiedMask = 1 shl 4; {    the passed in name is a fully qualified full path}

{
 *  FSSpecToNativePathName()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }


const
	kErrorIfFileNotFound = 1 shl 31;

{
 *  NativePathNameToFSSpec()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }

<<<<<<< HEAD
{$ifc CALL_NOT_IN_CARBON}
	{
	 *  NativePathNameToFSSpec()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   not available
	 *    CarbonLib:        not available
	 *    Mac OS X:         not available
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function NativePathNameToFSSpec(inName: CStringPtr; var outFile: FSSpec; flags: SInt32): OSErr; external name '_NativePathNameToFSSpec';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  QTGetAliasInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 5.0 and later
 }

{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 5.0 and later
 }

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
