{
<<<<<<< HEAD
<<<<<<< HEAD
     File:       OSServices/OpenTransport.h
 
     Contains:   *** DEPRECATED *** Open Transport client interface file.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  (c) 1985-2011 Apple Inc. All rights reserved.
=======
     Version:    OpenTransport-110~114
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       OT/OpenTransport.h
 
     Contains:   Open Transport client interface file.
 
     Version:    OpenTransport-110~114
 
<<<<<<< HEAD
     Copyright:  © 1985-2005 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{      Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{      Pascal Translation Updated: Jonas Maebe <jonas@freepascal.org>, September 2012 }
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
{      Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
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

unit OpenTransport;
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
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
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
=======
{$elsec}
=======
{$elsec}
>>>>>>> origin/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes,MixedMode,MacErrors;
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


{
    All OpenTransport Manager APIs are deprecated in MacOSX 10.4, instead of using OpenTransport,
    consider using CFNetwork or socket library.
}
{
   The following table shows how to map from the old (pre-Universal
   Interfaces) header file name to the equivalent Universal Interfaces
   header file name.
    Old Header              New Header
    ----------              ----------
    cred.h                  OpenTransportProtocol.h
    dlpi.h                  OpenTransportProtocol.h
    miioccom.h              OpenTransportProtocol.h
    mistream.h              OpenTransportProtocol.h/OpenTransportKernel.h
    modnames.h              OpenTransportProtocol.h
    OpenTptAppleTalk.h      OpenTransportProviders.h
    OpenTptClient.h         OpenTransportProtocol.h
    OpenTptCommon.h         OpenTransportProtocol.h
    OpenTptConfig.h         OpenTransportProtocol.h
    OpenTptDevLinks.h       OpenTransportProviders.h
    OpenTptInternet.h       OpenTransportProviders.h
    OpenTptISDN.h           OpenTransportProviders.h
    OpenTptLinks.h          OpenTransportProviders.h
    OpenTptModule.h         OpenTransportKernel.h
    OpenTptPCISupport.h     OpenTransportKernel.h
    OpenTptSerial.h         OpenTransportProviders.h
    OpenTptXTI.h            OpenTransportUNIX.r
    OpenTransport.h         OpenTransport.h
    OpenTransport.r         OpenTransport.r
    OTConfig.r              OpenTransportProtocol.r
    OTDebug.h               OpenTransport.h
    OTSharedLibs.h          OpenTransportProviders.h
    strlog.h                OpenTransportProtocol.h/OpenTransportKernel.h
    stropts.h               OpenTransportProtocol.h/OpenTransportUNIX.h
    strstat.h               OpenTransportProtocol.h
    tihdr.h                 OpenTransportProtocol.h
}


{$ALIGN MAC68K}

{ ***** Setup Default Compiler Variables *****}

{
   OTKERNEL is used to indicate whether the code is being built
   for the kernel environment.  It defaults to 0.  If you include
   "OpenTransportKernel.h" before including this file,
   it will be 1 and you will only be able to see stuff available
   to kernel code.
}

{$ifc undefined OTKERNEL}
{$setc OTKERNEL := 0}
{$endc}

{
   OTUNIXERRORS determines whether this file defines a bunch of
   common UNIX error codes, like EPERM.  Typically, client code does
   not want to do this because of the possibility of a clash with
   other code modules, like the standard C libraries, that also
   defines these routines.  However, client code can turn it on to
   get these definitions.  This might be done by protocol stack
   infrastructure, or some other low-level code.
   "OpenTransportKernel.i" sets this flag before include
   "OpenTransport.h" because kernel modules typically need these
   error codes.  Note that kernel modules shouldn't be including
   standard C libraries, so this is rarely a problem.
   In general, the clash between OT and standard C definitions
   of these error codes is rarely a problem becasue both OT
   and the C libraries define them to have the same value.  But
   I'm sure this check is useful to some people.
}
{$ifc undefined OTUNIXERRORS}
{$setc OTUNIXERRORS := 0}
{$endc}

{
   OTDEBUG is used to control the behaviour of the OT debugging
   macros.  If you set it to non-zero, the macros will generate code
   that drops you into the debugger.  If you set it to 0, or leave it
   undefined, the macros are compiled out.
   Setting up this compiler variable is a little tricky because previous
   versions of the OT interfaces used a different variable, qDebug.
   We replaced qDebug with OTDEBUG because qDebug does not fit into
   the OT namespace.  But I didn't want to break a lot of currently
   building code.  The following tricky compiler variable footwork
   avoids this.
   There are four outcomes when this code is compiled, depending on
   whether qDebug and OTDEBUG are defined beforehand.  The following
   table shows the outcome in each case.
   qDebug     OTDEBUG    Outcome       Explanation  
   ------     -------    -------       -----------
   defined    defined    OTDEBUG wins  Mixed legacy and new code, we believe the new code.
   defined    undefined  qDebug wins   Legacy code.
   undefined  defined    OTDEBUG wins  New code.
   undefined  undefined  no debugging  No debugging.
}
{$ifc not undefined qDebug}
{$ifc undefined OTDEBUG}
{$setc OTDebug := qDebug}
{$endc}
{$endc}

{$ifc undefined OTDEBUG}
{$setc OTDEBUG := 0}
{$endc}

{  Carbon Applications have some restrictions on using OT }
{$ifc undefined OTCARBONAPPLICATION}
{$setc OTCARBONAPPLICATION := 0}
{$endc}

{
   ***** Normalise 68K Calling C Conventions *****
   Define special types that handle the difference in parameter passing
   between different Mac OS C compilers when generating 68K code.  OT
   exports C calling conventions routines, and various C compilers use
   various different conventions.  Differences in the placement of the result
   are covered above, where we output pragma pointers_in_D0.  The other big
   difference is how the compilers pass integer parameters less than 32 bits.
   The MPW compiler always extends these to 32 bits; other compilers simply
   push a value of the appropriate size.  We overcome this difference by
   defining special OTFooParam types, which are only used when passing
   sub 32 bit values to routines.  They are always defined to a 32 bit
   size, which makes all the compilers do the same thing.
   One weird consequence of this is that in more strict type checking
   languages (eg Pascal) OTBooleanParam is not compatible with Boolean.
   Sorry.
}


{
   Large tracts of OT source still uses boolean_p etc.
   So we continue to define the old types for Apple
   clients.  The long term fix is to remove all the
   uses of these type from the OT source, but that's
   beyond the scope of my work right now.
}
type
	OTUInt8Param = UInt8;
	OTUInt16Param = UInt16;
	OTSInt16Param = SInt16;
	OTSInt8Param = SInt8;
	OTBooleanParam = Boolean;

type
	OTByteCount = ByteCount;
	OTItemCount = ItemCount;
	OTInt32 = SInt32;
	OTUInt32 = UInt32;

{ ***** C++ Support *****}

{
   Setup _MDECL to be _cdecl when compiling C++ code with
   compilers that support it, or nothing otherwise.
}


{ ***** Shared Library Prefixes *****}


const
	kOTLibraryVersion = '1.1';

const
	kOTLibraryPrefix = 'OTLib$';
const
	kOTModulePrefix = 'OTModl$';
const
	kOTClientPrefix = 'OTClnt$';
const
	kOTKernelPrefix = 'OTKrnl$';

const
	kOTCFMClass = FourCharCode('otan');

{ ***** Miscellaneous Type Definitions *****}

{ A millisecond timeout value}
type
	OTTimeout = UInt32;
{ An ID number in connections/transactions     }
type
	OTSequence = SInt32;
{ An ID number for registered names            }
type
	OTNameID = SInt32;
{
   A protocol-specific reason code for failure.
   Usually a Unix-style positive error code.
}
type
	OTReason = SInt32;
{ Number of outstanding connection requests at a time.}
type
	OTQLen = UInt32;
{ Will become internationalizeable shortly (yeah, right).}
type
	OTClientName = UInt8Ptr;
{ The command code in STREAMS messages.}
type
	OTCommand = SInt32;
{ value describing a client}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	OTClient = UnivPtr;
=======
	OTClient = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	OTClient = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	OTClient = UnivPtr;
>>>>>>> origin/fixes_2.4
	OTClientPtr = ^OTClient;

{
    OT now defines its own version of the standard C "offsetof"
    macro so as to avoid including <stddef.h>.
}
// #define OTOffsetOf(structure,field) ((ByteCount)&((structure *) 0)->field)

{ ***** Debugging Macros *****}


const
	kOTFatalErr = 'FB ';
const
	kOTNonfatalErr = 'NB ';
const
	kOTExtFatalErr = 'FX ';
const
	kOTExtNonfatalErr = 'NX ';
const
	kOTUserFatalErr = 'UF ';
const
	kOTUserErr = 'UE ';
const
	kOTUserNonfatalErr = 'UE ';
const
	kOTInfoErr = 'IE ';
const
	kOTInfoBreak = 'IN ';


{
   ***** Flags Used When Opening Providers *****
   Important
   OT does not currently support any of these flags.  You should
   always pass 0 to a parameter of type OTOpenFlags.  If you need
   to modify the mode of operation of a provider, use OTSetBlocking,
   OTSetSynchronous, etc.
}
type
	OTOpenFlags = UInt32;
const
	kO_ASYNC = $01;
	kO_NDELAY = $04;
	kO_NONBLOCK = $04;


{$ifc CALL_NOT_IN_CARBON}
{
   BSD defines O_ASYNC, O_NDELAY and O_NONBLOCK in fcntl.h 
   Use kO_ASYNC, kO_NDELAY and kO_NONBLOCK in the unlikely event you need the OT value in Carbon
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	O_ASYNC = kO_ASYNC;
	O_NDELAY = kO_NDELAY;
	O_NONBLOCK = kO_NONBLOCK;
=======
	O_ASYNC = $01;
	O_NDELAY = $04;
	O_NONBLOCK = $04;
>>>>>>> graemeg/fixes_2_2
=======
	O_ASYNC = $01;
	O_NDELAY = $04;
	O_NONBLOCK = $04;
>>>>>>> origin/fixes_2_2
=======
	O_ASYNC = kO_ASYNC;
	O_NDELAY = kO_NDELAY;
	O_NONBLOCK = kO_NONBLOCK;
>>>>>>> origin/fixes_2.4

{$endc}  {CALL_NOT_IN_CARBON}

{ ***** UNIX-Style Error Codes *****}

type
	OTUnixErr = UInt16;
{
   These definitions are only compiled if you're building kernel code
   or you explicit request them by setting OTUNIXERRORS.  See the
   description of these compiler variables, given above.
}
{$ifc OTKERNEL OR OTUNIXERRORS}
{
   There may be some error code confusions with other compiler vendor header
   files - However, these match both MPW and AIX definitions.
}
{
   First we undefine the #defined ones we know about so that we can put them
   in an enum.  Of course, this is only going to work in C, but hopefully
   other languages won't have these symbols overloaded.
}

     
const
	EPERM = 1;    { Permission denied            }
	ENOENT = 2;    { No such file or directory       }
	ENORSRC = 3;    { No such resource               }
	EINTR = 4;    { Interrupted system service        }
	EIO = 5;    { I/O error                 }
	ENXIO = 6;    { No such device or address       }
	EBADF = 9;    { Bad file number                 }
	EAGAIN = 11;   { Try operation again later       }
	ENOMEM = 12;   { Not enough space               }
	EACCES = 13;   { Permission denied            }
	EFAULT = 14;   { Bad address                   }
	EBUSY = 16;   { Device or resource busy          }
	EEXIST = 17;   { File exists                   }
	ENODEV = 19;   { No such device               }
	EINVAL = 22;   { Invalid argument               }
	ENOTTY = 25;   { Not a character device          }
	EPIPE = 32;   { Broken pipe                   }
	ERANGE = 34;   { Math result not representable   }
	EDEADLK = 35;   { Call would block so was aborted       }
	EWOULDBLOCK = 35;   { Or a deadlock would occur       }
	EALREADY = 37;
	ENOTSOCK = 38;   { Socket operation on non-socket     }
	EDESTADDRREQ = 39;   { Destination address required      }
	EMSGSIZE = 40;   { Message too long               }
	EPROTOTYPE = 41;   { Protocol wrong type for socket     }
	ENOPROTOOPT = 42;   { Protocol not available          }
	EPROTONOSUPPORT = 43;   { Protocol not supported          }
	ESOCKTNOSUPPORT = 44;   { Socket type not supported       }
	EOPNOTSUPP = 45;   { Operation not supported on socket  }
	EADDRINUSE = 48;   { Address already in use          }
	EADDRNOTAVAIL = 49;   { Can't assign requested address     }
	ENETDOWN = 50;   { Network is down                 }
	ENETUNREACH = 51;   { Network is unreachable          }
	ENETRESET = 52;   { Network dropped connection on reset    }
	ECONNABORTED = 53;   { Software caused connection abort     }
	ECONNRESET = 54;   { Connection reset by peer          }
	ENOBUFS = 55;   { No buffer space available       }
	EISCONN = 56;   { Socket is already connected         }
	ENOTCONN = 57;   { Socket is not connected          }
	ESHUTDOWN = 58;   { Can't send after socket shutdown     }
	ETOOMANYREFS = 59;   { Too many references: can't splice  }
	ETIMEDOUT = 60;   { Connection timed out             }
	ECONNREFUSED = 61;   { Connection refused           }
	EHOSTDOWN = 64;   { Host is down                }
	EHOSTUNREACH = 65;   { No route to host               }
	EPROTO = 70;   { STREAMS protocol error          }
	ETIME = 71;
	ENOSR = 72;
	EBADMSG = 73;
	ECANCEL = 74;
	ENOSTR = 75;
	ENODATA = 76;
	EINPROGRESS = 77;
	ESRCH = 78;
	ENOMSG = 79;
	ELASTERRNO = 79;

{$endc}

{ ***** Open Transport/XTI Error codes *****}
type
	OTXTIErr = UInt16;
const
	TSUCCESS = 0;    { No Error occurred             }
	TBADADDR = 1;    { A Bad address was specified          }
	TBADOPT = 2;    { A Bad option was specified          }
	TACCES = 3;    { Missing access permission          }
	TBADF = 4;    { Bad provider reference           }
	TNOADDR = 5;    { No address was specified             }
	TOUTSTATE = 6;    { Call issued in wrong state          }
	TBADSEQ = 7;    { Sequence specified does not exist   }
	TSYSERR = 8;    { A system error occurred              }
	TLOOK = 9;    { An event occurred - call Look()         }
	TBADDATA = 10;   { An illegal amount of data was specified    }
	TBUFOVFLW = 11;   { Passed buffer not big enough          }
	TFLOW = 12;   { Provider is flow-controlled          }
	TNODATA = 13;   { No data available for reading       }
	TNODIS = 14;   { No disconnect indication available     }
	TNOUDERR = 15;   { No Unit Data Error indication available    }
	TBADFLAG = 16;   { A Bad flag value was supplied       }
	TNOREL = 17;   { No orderly release indication available    }
	TNOTSUPPORT = 18;   { Command is not supported             }
	TSTATECHNG = 19;   { State is changing - try again later       }
	TNOSTRUCTYPE = 20;   { Bad structure type requested for OTAlloc   }
	TBADNAME = 21;   { A bad endpoint name was supplied      }
	TBADQLEN = 22;   { A Bind to an in-use address with qlen > 0}
	TADDRBUSY = 23;   { Address requested is already in use       }
	TINDOUT = 24;   { Accept failed because of pending listen    }
	TPROVMISMATCH = 25;   { Tried to accept on incompatible endpoint   }
	TRESQLEN = 26;
	TRESADDR = 27;
	TQFULL = 28;
	TPROTO = 29;   { An unspecified provider error occurred }
	TBADSYNC = 30;   { A synchronous call at interrupt time     }
	TCANCELED = 31;   { The command was cancelled          }
	TLASTXTIERROR = 31;

{
   ***** Mac OS Error Codes *****
   Most OT client routines return an OSStatus error code, a 32 bit type
   defined in "MacTypes.h".  The OT-unique error code values are
   defined below.  Many of these are generated by remapping XTI error
   codes (Txxxx) and UNIX error codes (Exxxx) to a reserved range
   in the OSStatus space.
   Some routines return an OTResult type, indicating
   that the routine might fail with a negative error, succeed with noErr,
   or possible return a positive value indicating some status.
}

type
	OTResult = SInt32;

{
 * These map the Open Transport/XTI errors (the Txxxx error codes), and the
 * StdCLib Exxxx error codes into unique spaces in the Mac OS OSStatus space.
 }
// #define XTI2OSStatus(x)           (-3149 - (x))
// #define E2OSStatus(x)         (-3199 - (x))

// #define OSStatus2XTI(x)          ((OTXTIErr)(-3149 - (x)))
// #define OSStatus2E(x)         ((OTUnixErr)(-3199 - (x)))

// #define IsXTIError(x)           ((x) < -3149 && (x) >= (-3149 - TLASTXTIERROR))
// #define IsEError(x)             ((x) < -3199 && (x) >= (-3199 - ELASTERRNO))

{ ***** OTAddress *****}

{
   OTAddress type defines the standard header for all OT address formats.
   It consists of one 16 bit integer, which defines the address format
   used, followed by an arbitrary number of bytes which are protocol-specific.
   Conceptually, all OT address formats are subtypes of this type,
   extended with fields that are specific to the protocol.  For example,
   OTInetAddress starts with the OTAddressType field and then continues
   to include a host IP address and a port number.
}

const
	kOTGenericName = 0;     { Protocol specific data is just a string, interpreted in a protocol-specific fashion.}

type
	OTAddressType = UInt16;
	OTAddressPtr = ^OTAddress;
	OTAddress = record
		fAddressType: OTAddressType;           { The address format of this address...}
  	fAddress: packed array[0..0] of UInt8;            { ... followed by protocol specific address information.}
	end;
{
   ***** OTAlloc Constants *****
   Note:
   In general, Apple recommends that you avoid the OTAlloc call because
   using it extensively causes your program to allocate and deallocate
   many memory blocks, with each extra memory allocation costing time.
}
{
   OTStructType defines the structure type to be allocated using the OTAlloc
   call.
}
const
	T_BIND = 1;
	T_OPTMGMT = 2;
	T_CALL = 3;
	T_DIS = 4;
	T_UNITDATA = 5;
	T_UDERROR = 6;
	T_INFO = 7;
	T_REPLYDATA = 8;
	T_REQUESTDATA = 9;
	T_UNITREQUEST = 10;
	T_UNITREPLY = 11;


type
	OTStructType = UInt32;
{
   These values are used in the "fields" parameter of the OTAlloc call
   to define which fields of the structure should be allocated.
}
const
	T_ADDR = $01;
	T_OPT = $02;
	T_UDATA = $04;
	T_ALL = $FFFF;

type
	OTFieldsType = UInt32;
{ ***** OTFlags *****}
{
   This type is used to describe bitwise flags in OT data structures
   and parameters.  Think of it as the OT analogue to the OptionBits
   type in "MacTypes.h".
}

type
	OTFlags = UInt32;
{
   These flags are used when sending and receiving data.  The
   constants defined are masks.
}
const
	T_MORE = $0001; { More data to come in message     }
	T_EXPEDITED = $0002; { Data is expedited, if possible }
	T_ACKNOWLEDGED = $0004; { Acknowledge transaction         }
	T_PARTIALDATA = $0008; { Partial data - more coming     }
	T_NORECEIPT = $0010; { No event on transaction done     }
	T_TIMEDOUT = $0020; { Reply timed out              }

{ These flags are used in the TOptMgmt structure to request services.}

const
	T_NEGOTIATE = $0004;
	T_CHECK = $0008;
	T_DEFAULT = $0010;
	T_CURRENT = $0080;

{
   These flags are used in the TOptMgmt and TOption structures to
   return results.
}

const
	T_SUCCESS = $0020;
	T_FAILURE = $0040;
	T_PARTSUCCESS = $0100;
	T_READONLY = $0200;
	T_NOTSUPPORT = $0400;

{
   ***** OTBand *****
   A band is a STREAMS concepts which defines the priority of data
   on a stream.  Although this type is defined as a 32 bit number
   for efficiency's sake, bands actually only range from 0 to 255. 
   Typically band 0 is used for normal data and band 1 for expedited data.
}
type
	OTBand = UInt32;
{ ***** Object References *****}
{
   This deserves some explanation.  If you're compiling for
   C++, the C++ definitions of TEndpoint and TMapper at the
   end of this file are invoked, which lets the compiler
   know that they are both subclasses of TProvider.  This
   way the compiler will do the right subclass type checking,
   ie you will be able to pass an EndpointRef to a parameter
   of type ProviderRef, but not vice versa.
   On the other hand, if your compiling for straighth C,
   everything is defined as void.  This is somewhat dangerous,
   but it prevents you have to cast an EndpointRef to a
   ProviderRef every time you call a function that works
   on all forms of providers.
}
type
	ProviderRef = ^SInt32; { an opaque 32-bit type }
	ProviderRefPtr = ^ProviderRef;
	EndpointRef = ProviderRef; { an opaque 32-bit type }
	EndpointRefPtr = ^EndpointRef;
	MapperRef = ProviderRef; { an opaque 32-bit type }
	MapperRefPtr = ^MapperRef;

const
	kOTInvalidRef = nil;
	kOTInvalidProviderRef = nil;
	kOTInvalidEndpointRef = nil;
	kOTInvalidMapperRef = nil;
{ ***** Event Codes *****}
{
   OT event codes values for Open Transport.  These are the event codes that
   are sent to notification routine (notifiers).
}

type
	OTEventCode = UInt32;
{
   Events are divided into numerous categories:
   
   1. (0x0000xxxx) The core XTI events have identifiers of the form
      T_XXXX.  These signal that an XTI event has occured on a stream.
   2. (0x1000xxxx) Private events are reserved for protocol specific
      events.  Each protocol stack defines them as appropriate for
      its own usage.
   3. (0x2000xxxxx) Completion events have identifiers of the form
      T_XXXXCOMPLETE.  These signal the completion of some asynchronous
      API routine, and are only delivered if the endpoint is in asynchronous
      mode.
   4. (0x2100xxxx) Stream events are generally encountered when programming
      the raw streams API and indicate some event on a raw stream, or
      some other event of interest in the STREAMS kernel.
   5. (0x2200xxxx) Signal events indicate that a signal has arrived on
      a raw stream.  See "Signal Values" for details.
   6. (0x2300xxxx) General provider events that might be generated by any
      provider.
   7. (0x2400xxxx) System events sent to all providers.
   8. (0x2500xxxx) System events sent to registered clients.
   9. (0x2600xxxx) System events used by configurators.
  10. (0x2700xxxx) Events sent to registered OT clients.
}
{
   All event codes not described here are reserved by Apple.  If you receive
   an event code you do not understand, ignore it!
}

const
	T_LISTEN = $0001; { An connection request is available     }
	T_CONNECT = $0002; { Confirmation of a connect request  }
	T_DATA = $0004; { Standard data is available        }
	T_EXDATA = $0008; { Expedited data is available         }
	T_DISCONNECT = $0010; { A disconnect is available       }
	T_ERROR = $0020; { obsolete/unused in library        }
	T_UDERR = $0040; { A Unit Data Error has occurred     }
	T_ORDREL = $0080; { An orderly release is available       }
	T_GODATA = $0100; { Flow control lifted on standard data   }
	T_GOEXDATA = $0200; { Flow control lifted on expedited data}
	T_REQUEST = $0400; { An Incoming request is available     }
	T_REPLY = $0800; { An Incoming reply is available     }
	T_PASSCON = $1000; { State is now T_DATAXFER          }
	T_RESET = $2000; { Protocol has been reset          }
	kPRIVATEEVENT = $10000000; { Base of the private event range.}
	kCOMPLETEEVENT = $20000000; { Base of the completion event range.}
	T_BINDCOMPLETE = $20000001; { Bind call is complete          }
	T_UNBINDCOMPLETE = $20000002; { Unbind call is complete          }
	T_ACCEPTCOMPLETE = $20000003; { Accept call is complete          }
	T_REPLYCOMPLETE = $20000004; { SendReply call is complete        }
	T_DISCONNECTCOMPLETE = $20000005; { Disconnect call is complete         }
	T_OPTMGMTCOMPLETE = $20000006; { OptMgmt call is complete          }
	T_OPENCOMPLETE = $20000007; { An Open call is complete          }
	T_GETPROTADDRCOMPLETE = $20000008; { GetProtAddress call is complete       }
	T_RESOLVEADDRCOMPLETE = $20000009; { A ResolveAddress call is complet     }
	T_GETINFOCOMPLETE = $2000000A; { A GetInfo call is complete        }
	T_SYNCCOMPLETE = $2000000B; { A Sync call is complete          }
	T_MEMORYRELEASED = $2000000C; { No-copy memory was released         }
	T_REGNAMECOMPLETE = $2000000D; { A RegisterName call is complete       }
	T_DELNAMECOMPLETE = $2000000E; { A DeleteName call is complete   }
	T_LKUPNAMECOMPLETE = $2000000F; { A LookupName call is complete   }
	T_LKUPNAMERESULT = $20000010; { A LookupName is returning a name     }
	kOTSyncIdleEvent = $20000011; { Synchronous call Idle event         }
	kSTREAMEVENT = $21000000; { Base of the raw stream event range.}
	kOTReservedEvent1 = $21000001; { reserved for internal use by OT       }
	kGetmsgEvent = $21000002; { A GetMessage call is complete   }
	kStreamReadEvent = $21000003; { A Read call is complete          }
	kStreamWriteEvent = $21000004; { A Write call is complete          }
	kStreamIoctlEvent = $21000005; { An Ioctl call is complete       }
	kOTReservedEvent2 = $21000006; { reserved for internal use by OT       }
	kStreamOpenEvent = $21000007; { An OpenStream call is complete     }
	kPollEvent = $21000008; { A Poll call is complete          }
	kOTReservedEvent3 = $21000009; { reserved for internal use by OT       }
	kOTReservedEvent4 = $2100000A; { reserved for internal use by OT       }
	kOTReservedEvent5 = $2100000B; { reserved for internal use by OT       }
	kOTReservedEvent6 = $2100000C; { reserved for internal use by OT       }
	kOTReservedEvent7 = $2100000D; { reserved for internal use by OT       }
	kOTReservedEvent8 = $2100000E; { reserved for internal use by OT       }
	kSIGNALEVENT = $22000000; { A signal has arrived on a raw stream, see "Signal Values" below.}
	kPROTOCOLEVENT = $23000000; { Some event from the protocols   }
	kOTProviderIsDisconnected = $23000001; { Provider is temporarily off-line     }
	kOTProviderIsReconnected = $23000002; { Provider is now back on-line      }
	kOTProviderWillClose = $24000001; { Provider will close immediately       }
	kOTProviderIsClosed = $24000002; { Provider was closed              }
	kOTPortDisabled = $25000001; { Port is now disabled, result is 0, cookie is port ref }
	kOTPortEnabled = $25000002; { Port is now enabled, result is 0, cookie is port ref }
	kOTPortOffline = $25000003; { Port is now offline, result is 0, cookie is port ref }
	kOTPortOnline = $25000004; { Port is now online, result is 0, cookie is port ref }
	kOTClosePortRequest = $25000005; { Request to close/yield, result is reason, cookie is OTPortCloseStruct* }
	kOTYieldPortRequest = $25000005; { Request to close/yield, result is reason, cookie is OTPortCloseStruct* }
	kOTNewPortRegistered = $25000006; { New port has been registered, cookie is port ref }
	kOTPortNetworkChange = $25000007; { Port may have moved to a new network, result is 0, cookie is port ref }
	kOTConfigurationChanged = $26000001; { Protocol configuration changed     }
	kOTSystemSleep = $26000002;
	kOTSystemShutdown = $26000003;
	kOTSystemAwaken = $26000004;
	kOTSystemIdle = $26000005;
	kOTSystemSleepPrep = $26000006;
	kOTSystemShutdownPrep = $26000007;
	kOTSystemAwakenPrep = $26000008;
	kOTStackIsLoading = $27000001; { Sent before Open Transport attempts to load the TCP/IP protocol stack.}
	kOTStackWasLoaded = $27000002; { Sent after the TCP/IP stack has been successfully loaded.}
	kOTStackIsUnloading = $27000003; { Sent before Open Transport unloads the TCP/IP stack.}


{
   The following event codes are used internally by Open Transport
   but not documented to developers.  I had to remove them from the
   above list because Interfacer won't let me put a hard conditional
   inside an enum declaration.
}
const
	kOTDisablePortEvent = $21000001;
	kStreamCloseEvent = $21000006;
	kBackgroundStreamEvent = $21000009;
	kIoctlRecvFdEvent = $2100000A;
	kOTTryShutdownEvent = $2100000B; { probably not used by current OT (2.5)}
	kOTScheduleTerminationEvent = $2100000C;
	kOTEnablePortEvent = $2100000D;
	kOTNewPortRegisteredEvent = $2100000E;
	kOTPortOfflineEvent = $2100000F;
	kOTPortOnlineEvent = $21000010;
	kOTPortNetworkChangeEvent = $21000011;


{ ***** Event Classification Macros ***** }

// #define IsOTPrivateEvent(x)         (((x) & 0x70000000L) == kPRIVATEEVENT)
// #define IsOTCompleteEvent(x)     (((x) & 0x7f000000L) == kCOMPLETEEVENT)
// #define IsOTProtocolEvent(x)        (((x) & 0x7f000000L) == kPROTOCOLEVENT)
// #define IsOTStreamEvent(x)          (((x) & 0x7f000000L) == kSTREAMEVENT)
// #define IsOTSignalEvent(x)            (((x) & 0x7f000000L) == kSIGNALEVENT)
// #define GetOTEventCode(x)         (x)

{
   ***** Signal Values *****
   Signals that are generated by a raw stream.  When writing a notifier
   for a raw stream, add these values to kSIGNALEVENT to determine what
   event you are receiving.
}

const
	kSIGHUP = 1;
	kSIGURG = 16;
	kSIGPOLL = 30;

const
	SIGHUP = 1;
const
	SIGURG = 16;

{
   ***** Notifier Type Definition *****
   Open Transport notifiers must conform to the OTNotifyProcPtr prototype.
   Even though a OTNotifyUPP is a OTNotifyProcPtr on pre-Carbon system,
   use NewOTNotifyUPP() and friends to make your source code portable to OS X and Carbon.
}

type
	OTNotifyProcPtr = procedure( contextPtr: UnivPtr; code: OTEventCode; result: OTResult; cookie: UnivPtr );
	OTNotifyUPP = OTNotifyProcPtr;
{
 *  NewOTNotifyUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOTNotifyUPP( userRoutine: OTNotifyProcPtr ): OTNotifyUPP; external name '_NewOTNotifyUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeOTNotifyUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeOTNotifyUPP( userUPP: OTNotifyUPP ); external name '_DisposeOTNotifyUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeOTNotifyUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeOTNotifyUPP( contextPtr: UnivPtr; code: OTEventCode; result: OTResult; cookie: UnivPtr; userUPP: OTNotifyUPP ); external name '_InvokeOTNotifyUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

{ ***** Option Management Definitions *****}
{ The XTI Level number of a protocol.}
const
	XTI_GENERIC = $FFFF; { level for XTI options }

type
	OTXTILevel = UInt32;
{ The XTI name of a protocol option.}
type
	OTXTIName = UInt32;
{ XTI names for options used with XTI_GENERIC above}
const
	XTI_DEBUG = $0001;
	XTI_LINGER = $0080;
	XTI_RCVBUF = $1002;
	XTI_RCVLOWAT = $1004;
	XTI_SNDBUF = $1001;
	XTI_SNDLOWAT = $1003;
	XTI_PROTOTYPE = $1005;
	OPT_CHECKSUM = $0600; { Set checksumming = UInt32 - 0 or 1)}
	OPT_RETRYCNT = $0601; { Set a retry counter = UInt32 (0 = infinite)}
	OPT_INTERVAL = $0602; { Set a retry interval = UInt32 milliseconds}
	OPT_ENABLEEOM = $0603; { Enable the EOM indication = UInt8 (0 or 1)}
	OPT_SELFSEND = $0604; { Enable Self-sending on broadcasts = UInt32 (0 or 1)}
	OPT_SERVERSTATUS = $0605; { Set Server Status (format is proto dependent)}
	OPT_ALERTENABLE = $0606; { Enable/Disable protocol alerts}
	OPT_KEEPALIVE = $0008; { See t_keepalive structure}

{ ***** Ioctl Definitions *****}

{
   All OT ioctl numbers are formed using the MIOC_CMD macro,
   which divides the ioctl space by protocol space (the
   first parameter) and ioctl number within that protocol
   space (the second parameter).  This macro is only available
   to C users but it's relatively easy to synthesise its
   results in other languages.
}
// #define MIOC_CMD(t,v)   ((((t)&0xFF) << 8) | ((v)&0xFF))

{ The following is a registry of the ioctls protocol spaces.}

const
	MIOC_STREAMIO = 65;							{  Basic Stream ioctl() cmds - I_PUSH, I_LOOK, etc.  }
	MIOC_TMOD = 97;							{  ioctl's for tmod test module     }
	MIOC_STRLOG = 98;							{  ioctl's for Mentat's log device       }
	MIOC_ND = 99;							{  ioctl's for Mentat's nd device         }
	MIOC_ECHO = 100;							{  ioctl's for Mentat's echo device    }
	MIOC_TLI = 101;							{  ioctl's for Mentat's timod module   }
	MIOC_RESERVEDf = 102;							{  reserved, used by SVR4 FIOxxx    }
	MIOC_SAD = 103;							{  ioctl's for Mentat's sad module       }
	MIOC_ARP = 104;							{  ioctl's for Mentat's arp module       }
	MIOC_HAVOC = 72;							{  Havoc module ioctls.            }
	MIOC_RESERVEDi = 105;							{  reserved, used by SVR4 SIOCxxx      }
	MIOC_SIOC = 106;							{  sockio.h socket ioctl's            }
	MIOC_TCP = 107;							{  tcp.h ioctl's                 }
	MIOC_DLPI = 108;							{  dlpi.h additions              }
	MIOC_SOCKETS = 109;							{  Mentat sockmod ioctl's            }
	MIOC_IPX = 111;							{  ioctls for IPX                }
	MIOC_OT = 79;							{  ioctls for Open Transport        }
	MIOC_ATALK = 84;							{  ioctl's for AppleTalk           }
	MIOC_SRL = 85;							{  ioctl's for Serial            }
	MIOC_RESERVEDp = 112;							{  reserved, used by SVR4           }
	MIOC_RESERVEDr = 114;							{  reserved, used by SVR4           }
	MIOC_RESERVEDs = 115;							{  reserved, used by SVR4           }
	MIOC_CFIG = 122;							{  ioctl's for private configuration  }

{ OT specific ioctls.}

const
	I_OTGetMiscellaneousEvents = $4F01;						{  sign up for Misc Events               }
	I_OTSetFramingType = $4F02;						{  Set framing option for link           }
	kOTGetFramingValue = $FFFFFFFF;					{  Use this value to read framing         }
	I_OTSetRawMode = $4F03;						{  Set raw mode for link             }
	kOTSetRecvMode = $01;
	kOTSendErrorPacket = $02;
	I_OTConnect = $4F04;						{  Generic connect request for links    }
	I_OTDisconnect = $4F05;						{  Generic disconnect request for links      }
	I_OTScript = $4F06;						{  Send a script to a module           }

{ Structure for the I_OTScript Ioctl.}

type
	OTScriptInfoPtr = ^OTScriptInfo;
	OTScriptInfo = record
		fScriptType: UInt32;
		fTheScript: UnivPtr;
		fScriptLength: UInt32;
	end;
{
   ***** XTI States *****
   These are the potential values returned by OTGetEndpointState and OTSync
   which represent the XTI state of an endpoint.
}
type
	OTXTIStates = UInt32;
const
	T_UNINIT = 0;    { addition to standard xti.h }
	T_UNBND = 1;    { unbound                 }
	T_IDLE = 2;    { idle                }
	T_OUTCON = 3;    { outgoing connection pending    }
	T_INCON = 4;    { incoming connection pending    }
	T_DATAXFER = 5;    { data transfer          }
	T_OUTREL = 6;    { outgoing orderly release     }
	T_INREL = 7;     { incoming orderly release     }

{
   ***** General XTI Definitions *****
   These definitions are typically used during option management.
}

const
	T_YES = 1;
	T_NO = 0;
	T_UNUSED = -1;
	kT_NULL = 0;
	T_ABSREQ = $8000;

const
	kT_UNSPEC = $FFFFFFFD;
	T_ALLOPT = 0;

{
   T_NULL and T_UNSPEC have different values in BSD headers.  If you want the
   OT values, use kT_NULL or kT_UNSPEC.
}
{
   ***** OTConfiguration *****
   This is a "black box" structure used to define the configuration of a
   provider or endpoint.  This file defines a very limited set of operations
   on a configuration.  "OpenTransportClient.h" extends this with extra
   operations used by protocol stacks but not typically needed by clients.
}


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	OTConfigurationRef = ^SInt32; { an opaque type }
=======
	OTConfigurationRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	OTConfigurationRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	OTConfigurationRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	OTConfigurationRefPtr = ^OTConfigurationRef;

const
	kOTNoMemoryConfigurationPtr = OTConfigurationRef(0);
	kOTInvalidConfigurationPtr = OTConfigurationRef(-1);
{ ***** Option Management Structures *****}

{ This structure describes the contents of a single option in a buffer.}

type
	TOptionHeaderPtr = ^TOptionHeader;
	TOptionHeader = record
		len: ByteCount;                    { total length of option          }
                                              { = sizeof(TOptionHeader) + length     }
                                              {     of option value in bytes       }
		level: OTXTILevel;                  { protocol affected            }
		name: OTXTIName;                   { option name                   }
		status: UInt32;                 { status value                }
	end;
{
   This structure describes the contents of a single option in a buffer.
   It differs from TOptionHeader in that it includes the value field,
   which acts as an unbounded array representing the value of the option.
}
type
	TOptionPtr = ^TOption;
	TOption = record
		len: ByteCount;                    { total length of option          }
                                              { = sizeof(TOption) + length }
                                              {     of option value in bytes       }
		level: OTXTILevel;                  { protocol affected            }
		name: OTXTIName;                   { option name                   }
		status: UInt32;                 { status value                }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		value: array [0..0] of UInt32;					{ data goes here               }
=======
		value: array [0..0] of UInt32;					{  data goes here                }
>>>>>>> graemeg/fixes_2_2
=======
		value: array [0..0] of UInt32;					{  data goes here                }
>>>>>>> origin/fixes_2_2
=======
		value: array [0..0] of UInt32;					{ data goes here               }
>>>>>>> origin/fixes_2.4
	end;
{ Some useful constants when manipulating option buffers.}
const
	kOTOptionHeaderSize = SizeOf(TOptionHeader);
	kOTBooleanOptionDataSize = SizeOf(UInt32);
	kOTBooleanOptionSize = kOTOptionHeaderSize + kOTBooleanOptionDataSize;
	kOTOneByteOptionSize = kOTOptionHeaderSize + 1;
	kOTTwoByteOptionSize = kOTOptionHeaderSize + 2;
	kOTFourByteOptionSize = kOTOptionHeaderSize + SizeOf(UInt32);


{
    This macro will align return the value of "len", rounded up to the next
    4-byte boundary.
}

// #define T_ALIGN(len) (((UInt32)(len)+(sizeof(SInt32)-1)) & ~(sizeof(SInt32)-1))

{
   This macro will return the next option in the buffer, given the previous option
    in the buffer, returning NULL if there are no more.
    You start off by setting prevOption = (TOption*)theBuffer
  (Use OTNextOption for a more thorough check - it ensures the end
   of the option is in the buffer as well.)
}

// #define OPT_NEXTHDR(theBuffer, theBufLen, prevOption) \
//    (((char*)(prevOption) + T_ALIGN((prevOption)->len) < (char*)(theBuffer) + (theBufLen)) ?    \
//           (TOption*)((char*)(prevOption)+T_ALIGN((prevOption)->len))  \
//           : (TOption*)NULL)


{ t_kpalive is used with OPT_KEEPALIVE option.}

type
	t_kpalivePtr = ^t_kpalive;
	t_kpalive = record
		kp_onoff: SInt32;               { option on/off   }
		kp_timeout: SInt32;             { timeout in minutes }
	end;
{ t_linger is used with XTI_LINGER option.}
type
	t_lingerPtr = ^t_linger;
	t_linger = record
		l_onoff: SInt32;                { option on/off }
		l_linger: SInt32;               { linger time }
	end;
{
   ***** TEndpointInfo *****
   This structure is returned from the GetEndpointInfo call and contains
   information about an endpoint.  But first, some special flags and types.
}
{ Values returned in servtype field of TEndpointInfo.}

type
	OTServiceType = UInt32;
const
	T_COTS = 1;    { Connection-mode service                    }
	T_COTS_ORD = 2;    { Connection service with orderly release          }
	T_CLTS = 3;    { Connectionless-mode service                   }
	T_TRANS = 5;    { Connection-mode transaction service              }
	T_TRANS_ORD = 6;    { Connection transaction service with orderly release    }
	T_TRANS_CLTS = 7;     { Connectionless transaction service           }

{ Masks for the flags field of TEndpointInfo.}

const
	T_SENDZERO = $0001; { supports 0-length TSDU's          }
	T_XPG4_1 = $0002; { supports the GetProtAddress call     }
	T_CAN_SUPPORT_MDATA = $10000000; { support M_DATAs on packet protocols    }
	T_CAN_RESOLVE_ADDR = $40000000; { Supports ResolveAddress call      }
	T_CAN_SUPPLY_MIB = $20000000; { Supports SNMP MIB data          }

{
   Special-case values for in the tsdu, etsdu, connect, and discon
   fields of TEndpointInfo.
}

const
	T_INFINITE = -1;   { supports infinit amounts of data     }
	T_INVALID = -2;    { Does not support data transmission }


type
	OTDataSize = SInt32;
{ Now the TEndpointInfo structure proper.}
type
	TEndpointInfoPtr = ^TEndpointInfo;
	TEndpointInfo = record
		addr: OTDataSize;                   { Maximum size of an address        }
		options: OTDataSize;                { Maximum size of options          }
		tsdu: OTDataSize;                   { Standard data transmit unit size     }
		etsdu: OTDataSize;                  { Expedited data transmit unit size  }
		connect: OTDataSize;                { Maximum data size on connect      }
		discon: OTDataSize;                 { Maximum data size on disconnect       }
		servtype: OTServiceType;               { service type                }
		flags: UInt32;                  { Flags (see above for values)      }
	end;
{
   "OpenTransport.h" no longer defines "struct t_info".  We recommend
   that you use TEndpointInfo instead.  If this is impossible, use
   the definition of "struct t_info" in "OpenTransportXTI.h".
}
{ ***** OTPortRecord *****}

{ Unique identifier for a port.}


type
	OTPortRef = UInt32;
	OTPortRefPtr = ^OTPortRef;
const
	kOTInvalidPortRef = 0;

{ Valid values for the bus type element of an OTPortRef.}

type
	OTBusType = UInt8;
const
	kOTUnknownBusPort = 0;
	kOTMotherboardBus = 1;
	kOTNuBus = 2;
	kOTPCIBus = 3;
	kOTGeoPort = 4;
	kOTPCCardBus = 5;
	kOTFireWireBus = 6;
	kOTLastBusIndex = 15;

{
   A couple of special values for the device type element of an
   OTPortRef.  See "OpenTransportDevices.h" for the standard values.
}

type
	OTDeviceType = UInt16;
const
	kOTNoDeviceType = 0;
	kOTADEVDevice = 1;    { An Atalk ADEV   }
	kOTMDEVDevice = 2;    { A TCP/IP MDEV   }
	kOTLocalTalkDevice = 3;    { LocalTalk       }
	kOTIRTalkDevice = 4;    { IRTalk          }
	kOTTokenRingDevice = 5;    { Token Ring        }
	kOTISDNDevice = 6;    { ISDN             }
	kOTATMDevice = 7;    { ATM              }
	kOTSMDSDevice = 8;    { SMDS             }
	kOTSerialDevice = 9;    { Serial           }
	kOTEthernetDevice = 10;   { Ethernet          }
	kOTSLIPDevice = 11;   { SLIP Pseudo-device }
	kOTPPPDevice = 12;   { PPP Pseudo-device  }
	kOTModemDevice = 13;   { Modem Pseudo-Device    }
	kOTFastEthernetDevice = 14;   { 100 MB Ethernet       }
	kOTFDDIDevice = 15;   { FDDI             }
	kOTIrDADevice = 16;   { IrDA Infrared   }
	kOTATMSNAPDevice = 17;   { ATM SNAP emulation }
	kOTFibreChannelDevice = 18;   { Fibre Channel   }
	kOTFireWireDevice = 19;   { FireWire link Device   }
	kOTPseudoDevice = 1023; { used where no other defined device type will work}
	kOTLastDeviceIndex = 1022;

{ Special case values for the slot number element of an OTPortRef.}

const
	kOTLastSlotNumber = 255;
	kOTLastOtherNumber = 255;

type
	OTSlotNumber = UInt16;
{ Accessor functions for the various elements of the OTPortRef.}
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
 *  OTCreatePortRef()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTCreatePortRef( busType: OTBusType; devType: OTDeviceType; slot: OTSlotNumber; other: UInt16 ): OTPortRef; external name '_OTCreatePortRef';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTGetDeviceTypeFromPortRef()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTGetDeviceTypeFromPortRef( ref: OTPortRef ): OTDeviceType; external name '_OTGetDeviceTypeFromPortRef';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTGetBusTypeFromPortRef()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTGetBusTypeFromPortRef( ref: OTPortRef ): UInt16; external name '_OTGetBusTypeFromPortRef';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTGetSlotFromPortRef()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTGetSlotFromPortRef( ref: OTPortRef; var other: UInt16 ): OTSlotNumber; external name '_OTGetSlotFromPortRef';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSetDeviceTypeInPortRef()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTSetDeviceTypeInPortRef( ref: OTPortRef; devType: OTDeviceType ): OTPortRef; external name '_OTSetDeviceTypeInPortRef';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSetBusTypeInPortRef()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTSetBusTypeInPortRef( ref: OTPortRef; busType: OTBusType ): OTPortRef; external name '_OTSetBusTypeInPortRef';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
    Convenience macros for generating specific types of OTPortRefs.
}

// #define OTCreateNuBusPortRef(devType, slot, other)  \
//   OTCreatePortRef(kOTNuBus, devType, slot, other)
    
// #define OTCreatePCIPortRef(devType, slot, other)   \
//   OTCreatePortRef(kOTPCIBus, devType, slot, other)
   
// #define OTCreatePCCardPortRef(devType, slot, other)    \
//   OTCreatePortRef(kOTPCCardBus, devType, slot, other)

{ Name length definitions for various fields in OTPortRecord.}

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
	kMaxModuleNameLength = 31;   { max length of a STREAMS module name}
	kMaxModuleNameSize = kMaxModuleNameLength + 1;
	kMaxProviderNameLength = kMaxModuleNameLength + 4; { providers allow 4 characters for minor number}
	kMaxProviderNameSize = kMaxProviderNameLength + 1;
	kMaxSlotIDLength = 7;    { PCI slot names tend to be short}
	kMaxSlotIDSize = kMaxSlotIDLength + 1;
	kMaxResourceInfoLength = 31;   { max length of a configuration helper name}
	kMaxResourceInfoSize = 32;
	kMaxPortNameLength = kMaxModuleNameLength + 4; { max size allowed to define a port}
	kMaxPortNameSize = kMaxPortNameLength + 1;

{
   Masks for the fPortFlags field of OTPortRecord
   If no bits are set, the port is currently inactive.
}

const
	kOTPortIsActive = $00000001;
	kOTPortIsDisabled = $00000002;
	kOTPortIsUnavailable = $00000004;
	kOTPortIsOffline = $00000008;

{ Masks for the fInfoFlags field of the OTPortRecord.}

const
	kOTPortIsDLPI = $00000001;
	kOTPortIsTPI = $00000002;
	kOTPortCanYield = $00000004; { will not be set until the port is used for the first time}
	kOTPortCanArbitrate = $00000008; { will not be set until the port is used for the first time}
	kOTPortIsTransitory = $00000010;
	kOTPortAutoConnects = $00000020;
	kOTPortIsSystemRegistered = $00004000;
	kOTPortIsPrivate = $00008000;
	kOTPortIsAlias = $80000000;

{
   One OTPortRecord is created for each instance of a port.
   For Instance 'enet' identifies an ethernet port.
   A OTPortRecord for each ethernet card it finds, with an
   OTPortRef that will uniquely allow the driver to determine which
   port it is supposed to open on.
}

type
	OTPortRecordPtr = ^OTPortRecord;
	OTPortRecord = record
		fRef: OTPortRef;
		fPortFlags: UInt32;
		fInfoFlags: UInt32;
		fCapabilities: UInt32;
		fNumChildPorts: ItemCount;
		fChildPorts: OTPortRefPtr;
		fPortName: packed array [0..35] of char;
		fModuleName: packed array [0..31] of char;
		fSlotID: packed array [0..7] of char;
		fResourceInfo: packed array [0..31] of char;
		fReserved: packed array [0..163] of char;
	end;
{
   Routines for finding, registering and unregistering ports.
   IMPORTANT:
   These routines have two versions, one for the client and one
   for the kernel.  Make sure you use and link with the right ones.
}
{$ifc NOT OTKERNEL}
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
 *  OTGetIndexedPort()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }
function OTGetIndexedPort( var portRecord: OTPortRecord; index: OTItemCount ): Boolean; external name '_OTGetIndexedPort';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Index through the ports in the system}
{
 *  OTFindPort()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }
function OTFindPort( var portRecord: OTPortRecord; portName: ConstCStringPtr ): Boolean; external name '_OTFindPort';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Find an OTPortRecord for a port using it's name}
{
 *  OTFindPortByRef()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }
function OTFindPortByRef( var portRecord: OTPortRecord; ref: OTPortRef ): Boolean; external name '_OTFindPortByRef';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{ Find an OTPortRecord for a port using it's OTPortRef}
{$endc} {not TARGET_CPU_64}

=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Find an OTPortRecord for a port using it's OTPortRef}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  OTRegisterPort()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }


{
   Register a port. The name the port was registered under is returned in
   the fPortName field.
}
{
 *  OTUnregisterPort()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }


{
   Unregister the port with the given name (If you re-register the
   port, it may get a different name - use OTChangePortState if
   that is not desireable).  Since a single OTPortRef can be registered
   with several names, the API needs to use the portName rather than
   the OTPortRef to disambiguate.
}
{
 *  OTChangePortState()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }


{ Change the state of the port.}
{$endc}  { !OTKERNEL }

{ ***** Data Buffers *****}
{
   TNetbuf is the basic structure used to pass data back and forth
   between the Open Transport protocols and their clients
}

type
	TNetbufPtr = ^TNetbuf;
	TNetbuf = record
		maxlen: ByteCount;
		len: ByteCount;
		buf: UInt8Ptr;
	end;
{
   Some rarely used low-level routines in this file take a strbuf
   as a parameter.  This is the raw streams equivalent of a TNetbuf.
   The key difference is that the maxlen and len fields are signed,
   which allows you to specify extra operations by providing a
   negative value.
}


type
	strbufPtr = ^strbuf;
	strbuf = record
		maxlen: SInt32;                 { max buffer length }
		len: SInt32;                    { length of data }
		buf: UnivPtr;                    { pointer to buffer }
	end;
{
   OTData is used in a TNetbuf or netbuf to send
   non-contiguous data.  Set the 'len' field of the netbuf to the
   constant kNetbufDataIsOTData to signal that the 'buf' field of the
   netbuf actually points to one of these structures instead of a
   memory buffer.
}
type
	OTDataPtr = ^OTData;
	OTData = record
		fNext: UnivPtr;
		fData: UnivPtr;
		fLen: ByteCount;
	end;
const
	kNetbufDataIsOTData = $FFFFFFFE;


{
   OTBuffer is used for no-copy receives.  When receiving, you can
   set the receive length to kOTNetbufDataIsOTBufferStar and then
   pass the address of an OTBuffer* as the receive buffer.  OT will
   fill it out to point to a chain of OTBuffers.
   When you are done with it, you must call the OTReleaseBuffer function.
   For best performance, you need to call OTReleaseBuffer quickly.
   Only data netbufs may use this - no netbufs for addresses or options, or the like.
   Any OTBuffer returned to you by OT is read only!
   The astute will notice that this has a high correlation with the
   STREAMS msgb data type.  The fields are commented with their
   corresponding msgb field name.
}

type
	OTBufferPtr = ^OTBuffer;
	OTBuffer = record
		fLink: UnivPtr;                  { b_next}
		fLink2: UnivPtr;                 { b_prev}
		fNext: OTBufferPtr;                  { b_cont}
		fData: UInt8Ptr;                  { b_rptr}
		fLen: ByteCount;                   { b_wptr}
		fSave: UnivPtr;                  { b_datap}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		fBand: UInt8;                  { b_band}
		fType: UInt8;                  { b_pad1}
		fPad1: UInt8;
		fFlags: UInt8;                 { b_flag}
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		fBand: SInt8;                  { b_band}
		fType: SInt8;                  { b_pad1}
		fPad1: SInt8;
		fFlags: SInt8;                 { b_flag}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
const
	kOTNetbufDataIsOTBufferStar = $FFFFFFFD;

{
   OTBufferInfo is used with OTReadBuffer to keep track of where you
   are in the buffer, since the OTBuffer is "read-only".
}
{ Use the OTInitBuffer macro to initialise this structure from an OTBuffer chain.}
type
	OTBufferInfoPtr = ^OTBufferInfo;
	OTBufferInfo = record
		fBuffer: OTBufferPtr;
		fOffset: ByteCount;
		fPad: UInt8;
	end;

// #define OTInitBufferInfo(infoPtr, theBuffer)   \
//   (infoPtr)->fBuffer = theBuffer;             \
//   (infoPtr)->fPad = (theBuffer)->fPad1;       \
//   (infoPtr)->fOffset  = 0

{
   If the endpoint supports "raw mode" (the T_CAN_SUPPORT_MDATA bit will
   be set in the TEndpointInfo::flags field), then you specify the
   raw mode packet by putting the kOTNetbufIsRawMode value in
   the udata.addr.len field when calling OTSndUData and also set the
   udata.opt.len, udata.opt.buf, and udata.addr.buf fields to 0.
}

const
	kOTNetbufIsRawMode = $FFFFFFFF;

{
   ***** Standard XTI Parameter Types *****
   These structures are all used as parameters to the standard
   XTI routines.
}

{
   TBind holds binding information for calls to
   OTGetProtAddress, OTResolveAddress and OTBind.
}

type
	TBindPtr = ^TBind;
	TBind = record
		addr: TNetbuf;
		qlen: OTQLen;
	end;
{
   TDiscon is passed to RcvDisconnect to find out additional information
   about the disconnect.
}
type
	TDisconPtr = ^TDiscon;
	TDiscon = record
		udata: TNetbuf;
		reason: OTReason;
		sequence: OTSequence;
	end;
{
   TCall holds information about a connection and is a parameter to
   OTConnect, OTRcvConnect, OTListen, OTAccept, and OTSndDisconnect.
}
type
	TCallPtr = ^TCall;
	TCall = record
		addr: TNetbuf;
		opt: TNetbuf;
		udata: TNetbuf;
		sequence: OTSequence;
	end;
{ TUnitData describes a datagram in calls to OTSndUData and OTRcvUData.}
type
	TUnitDataPtr = ^TUnitData;
	TUnitData = record
		addr: TNetbuf;
		opt: TNetbuf;
		udata: TNetbuf;
	end;
{
   TUDErr is used to get information about a datagram error using
   OTRcvUDErr.
}
type
	TUDErrPtr = ^TUDErr;
	TUDErr = record
		addr: TNetbuf;
		opt: TNetbuf;
		error: SInt32;
	end;
{ TOptMgmt is passed to the OTOptionManagement call to read or set protocol}
type
	TOptMgmtPtr = ^TOptMgmt;
	TOptMgmt = record
		opt: TNetbuf;
		flags: OTFlags;
	end;
{
   ***** Transactional XTI Parameter Types *****
   These structures are all used as parameters to the OT's
   XTI-like routines for transaction protocols.
}
{
   TRequest is passed to OTSndRequest and OTRcvRequest that contains the information
   about the request.
}

type
	TRequestPtr = ^TRequest;
	TRequest = record
		data: TNetbuf;
		opt: TNetbuf;
		sequence: OTSequence;
	end;
{ TReply is passed to OTSndReply to send a reply to an incoming request.}
type
	TReplyPtr = ^TReply;
	TReply = record
		data: TNetbuf;
		opt: TNetbuf;
		sequence: OTSequence;
	end;
{
   TUnitRequest is passed to OTSndURequest and OTRcvURequest that contains
   the information about the request.
}
type
	TUnitRequestPtr = ^TUnitRequest;
	TUnitRequest = record
		addr: TNetbuf;
		opt: TNetbuf;
		udata: TNetbuf;
		sequence: OTSequence;
	end;
{ TUnitReply is passed to OTSndUReply to send a reply to an incoming request.}
type
	TUnitReplyPtr = ^TUnitReply;
	TUnitReply = record
		opt: TNetbuf;
		udata: TNetbuf;
		sequence: OTSequence;
	end;
{
   ***** Mapper Parameter Types *****
   These structures are all used as parameters to the OT's
   mapper routines.
}
{ TRegisterRequest holds the name to register in a call to OTRegisterName.}

type
	TRegisterRequestPtr = ^TRegisterRequest;
	TRegisterRequest = record
		name: TNetbuf;
		addr: TNetbuf;
		flags: OTFlags;
	end;
{
   TRegisterReply returns information about the registered name in a call
   to OTRegisterName.
}
type
	TRegisterReplyPtr = ^TRegisterReply;
	TRegisterReply = record
		addr: TNetbuf;
		nameid: OTNameID;
	end;
{ TLookupRequest holds the name to look up in a call to OTLookupName.}
type
	TLookupRequestPtr = ^TLookupRequest;
	TLookupRequest = record
		name: TNetbuf;
		addr: TNetbuf;
		maxcnt: UInt32;
		timeout: OTTimeout;
		flags: OTFlags;
	end;
{
   TLookupReply returns information about the found names after a call
   to OTLookupName.
}
type
	TLookupReplyPtr = ^TLookupReply;
	TLookupReply = record
		names: TNetbuf;
		rspcount: UInt32;
	end;
{
   TLookupBuffer describes the contents of the names buffer pointed
   to by the TLookupReply.
}
type
	TLookupBufferPtr = ^TLookupBuffer;
	TLookupBuffer = record
		fAddressLength: UInt16;
		fNameLength: UInt16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		fAddressBuffer: packed array[0..0] of UInt8;
=======
		fAddressBuffer: packed array[0..0] of SInt8;
>>>>>>> graemeg/fixes_2_2
=======
		fAddressBuffer: packed array[0..0] of SInt8;
>>>>>>> origin/fixes_2_2
=======
		fAddressBuffer: packed array[0..0] of UInt8;
>>>>>>> origin/fixes_2.4
	end;

{
    OTNextLookupBuffer allows you to step through a packed array
   of TLookupBuffers.
}

// #define OTNextLookupBuffer(buf)          \
//   ((TLookupBuffer*)                   \
//       ((char*)buf + ((OTOffsetOf(TLookupBuffer, fAddressBuffer) + buf->fAddressLength + buf->fNameLength + 3) & ~3)))

{ ***** Initializing and Shutting Down Open Transport *****}

{$ifc NOT OTKERNEL}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	OTClientContextPtr = ^SInt32; { an opaque type }
=======
	OTClientContextPtr = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	OTClientContextPtr = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	OTClientContextPtr = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	OTClientContextPtrPtr = ^OTClientContextPtr;
{
   For Carbon the InitOpenTransport interface has changed so it takes a flags parameter 
   and returns a client context pointer.
   The flag passed to indicates whether OT should be initialized for application use or for some other target
   (for example, plugins that run in an application context but not the application itself.)
   Applications that are not interested in the value of the client context pointer may pass NULL
   as outClientContext -- they will pass NULL to other routines that take a OTClientContextPtr.
}
type
	OTInitializationFlags = UInt32;
const
	kInitOTForApplicationMask = 1;
	kInitOTForExtensionMask = 2;

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
 *  InitOpenTransportInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function InitOpenTransportInContext( flags: OTInitializationFlags; outClientContext: OTClientContextPtrPtr { can be NULL } ): OSStatus; external name '_InitOpenTransportInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   Under Carbon, CloseOpenTransport takes a client context pointer.  Applications may pass NULL
   after calling InitOpenTransport(kInitOTForApplicationMask, ...).  Non-applications must always pass a
   valid client context.
}
{
 *  CloseOpenTransportInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure CloseOpenTransportInContext( clientContext: OTClientContextPtr ); external name '_CloseOpenTransportInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2_2

{$endc} {not TARGET_CPU_64}

{
 *  InitOpenTransport()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  InitOpenTransportUtilities()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  CloseOpenTransport()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTRegisterAsClient()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   This registers yourself as a client for any miscellaneous Open Transport
   notifications that come along. CloseOpenTransport will automatically do
   an OTUnregisterAsClient, if you have not already done so.
}
{
 *  OTUnregisterAsClient()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


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
 *  OTRegisterAsClientInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function OTRegisterAsClientInContext( name: OTClientName; proc: OTNotifyUPP; clientContext: OTClientContextPtr { can be NULL } ): OSStatus; external name '_OTRegisterAsClientInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTUnregisterAsClientInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   not available
 }
function OTUnregisterAsClientInContext( clientContext: OTClientContextPtr ): OSStatus; external name '_OTUnregisterAsClientInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{$endc}  { !OTKERNEL }

{ ***** Tasking Model *****}
{
   OTEnterInterrupt/OTLeaveInterrupt are normally used within the kernel to
   tell Open Transport we're at hardware interrupt time.  Clients can also
   them to do the same.
}

{
 *  OTEnterInterrupt()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTLeaveInterrupt()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTIsAtInterruptLevel()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTCanLoadLibraries()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
   All OT task callbacks use the same prototype, shown below.
   This is only a UPP for CFM-68K clients.
}

type
	OTProcessProcPtr = procedure( arg: UnivPtr );
	OTProcessUPP = OTProcessProcPtr;
{
 *  NewOTProcessUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOTProcessUPP( userRoutine: OTProcessProcPtr ): OTProcessUPP; external name '_NewOTProcessUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeOTProcessUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeOTProcessUPP( userUPP: OTProcessUPP ); external name '_DisposeOTProcessUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeOTProcessUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeOTProcessUPP( arg: UnivPtr; userUPP: OTProcessUPP ); external name '_InvokeOTProcessUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

{$ifc NOT OTKERNEL}
{
   Under Carbon, OTCreateDeferredTask takes a client context pointer.  Applications may pass NULL
   after calling InitOpenTransport(kInitOTForApplicationMask, ...).  Non-applications must always pass a
   valid client context.
}
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
 *  OTCreateDeferredTaskInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTCreateDeferredTaskInContext( upp: OTProcessUPP; arg: UnivPtr; clientContext: OTClientContextPtr { can be NULL } ): SIGNEDLONG; external name '_OTCreateDeferredTaskInContext';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTCreateDeferredTaskInContext( upp: OTProcessUPP; arg: UnivPtr; clientContext: OTClientContextPtr { can be NULL } ): SIGNEDLONG; external name '_OTCreateDeferredTaskInContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$endc} {not TARGET_CPU_64}

{$endc}  { !OTKERNEL }

{
   OT deferred tasks are often more convenience that standard Mac OS
   although they have no significant advantages beyond convenience.
}


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	OTDeferredTaskRef = SIGNEDLONG;
=======
	OTDeferredTaskRef = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	OTDeferredTaskRef = SInt32;
>>>>>>> origin/fixes_2_2
=======
	OTDeferredTaskRef = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
{
 *  OTCreateDeferredTask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


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
 *  OTScheduleDeferredTask()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTScheduleDeferredTask( dtCookie: OTDeferredTaskRef ): Boolean; external name '_OTScheduleDeferredTask';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2_2

{$endc} {not TARGET_CPU_64}

{
 *  OTScheduleInterruptTask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


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
 *  OTDestroyDeferredTask()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTDestroyDeferredTask( dtCookie: OTDeferredTaskRef ): OSStatus; external name '_OTDestroyDeferredTask';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{$ifc NOT OTKERNEL}
{
   OT system tasks allow you to schedule a procedure to be called
   at system task time.  Potentially useful, but it relies on someone
   calling SystemTask (or WaitNextEvent, which calls SystemTask).
   Not available to kernel code because relying on system task time
   to make progress is likely to result in deadlocks.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	OTSystemTaskRef = SIGNEDLONG;
=======
	OTSystemTaskRef = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	OTSystemTaskRef = SInt32;
>>>>>>> origin/fixes_2_2
=======
	OTSystemTaskRef = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
{
 *  OTCreateSystemTask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTDestroySystemTask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTScheduleSystemTask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTCancelSystemTask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


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
 *  OTCanMakeSyncCall()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTCanMakeSyncCall: Boolean; external name '_OTCanMakeSyncCall';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{$endc}  { !OTKERNEL }

{ ***** Interface to Providers *****}
{$ifc NOT OTKERNEL}
{
 *  OTAsyncOpenProvider()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTOpenProvider()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


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
 *  OTCloseProvider()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTCloseProvider( ref: ProviderRef ): OSStatus; external name '_OTCloseProvider';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  OTTransferProviderOwnership()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTWhoAmI()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTGetProviderPortRef()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


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
 *  OTIoctl()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTIoctl( ref: ProviderRef; cmd: UInt32; data: UnivPtr ): SInt32; external name '_OTIoctl';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  OTGetMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTGetPriorityMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTPutMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTPutPriorityMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


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
 *  OTSetAsynchronous()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTSetAsynchronous( ref: ProviderRef ): OSStatus; external name '_OTSetAsynchronous';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSetSynchronous()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTSetSynchronous( ref: ProviderRef ): OSStatus; external name '_OTSetSynchronous';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTIsSynchronous()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTIsSynchronous( ref: ProviderRef ): Boolean; external name '_OTIsSynchronous';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSetBlocking()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTSetBlocking( ref: ProviderRef ): OSStatus; external name '_OTSetBlocking';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSetNonBlocking()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTSetNonBlocking( ref: ProviderRef ): OSStatus; external name '_OTSetNonBlocking';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTIsBlocking()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTIsBlocking( ref: ProviderRef ): Boolean; external name '_OTIsBlocking';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTInstallNotifier()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTInstallNotifier( ref: ProviderRef; proc: OTNotifyUPP; contextPtr: UnivPtr ): OSStatus; external name '_OTInstallNotifier';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTUseSyncIdleEvents()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTUseSyncIdleEvents( ref: ProviderRef; useEvents: Boolean ): OSStatus; external name '_OTUseSyncIdleEvents';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTRemoveNotifier()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
procedure OTRemoveNotifier( ref: ProviderRef ); external name '_OTRemoveNotifier';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTLeaveNotifier()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
procedure OTLeaveNotifier( ref: ProviderRef ); external name '_OTLeaveNotifier';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTEnterNotifier()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTEnterNotifier( ref: ProviderRef ): Boolean; external name '_OTEnterNotifier';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTAckSends()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTAckSends( ref: ProviderRef ): OSStatus; external name '_OTAckSends';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTDontAckSends()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTDontAckSends( ref: ProviderRef ): OSStatus; external name '_OTDontAckSends';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTIsAckingSends()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTIsAckingSends( ref: ProviderRef ): Boolean; external name '_OTIsAckingSends';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTCancelSynchronousCalls()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTCancelSynchronousCalls( ref: ProviderRef; err: OSStatus ): OSStatus; external name '_OTCancelSynchronousCalls';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$endc} {not TARGET_CPU_64}

=======
{$endc} {not TARGET_CPU_64}

>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
{$endc} {not OTKERNEL}
=======
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
{$endc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not OTKERNEL}
>>>>>>> origin/fixes_2.4

{ ***** Interface to Endpoints *****}
{$ifc NOT OTKERNEL}
{ Open/Close}
{
   Under Carbon, the OpenEndpoint routines take a client context pointer.  Applications may pass NULL after
   calling InitOpenTransport(kInitOTForApplicationMask, ...).  Non-applications must always pass a
   valid client context.
}
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
 *  OTOpenEndpointInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTOpenEndpointInContext( config: OTConfigurationRef; oflag: OTOpenFlags; info: TEndpointInfoPtr { can be NULL }; var err: OSStatus; clientContext: OTClientContextPtr { can be NULL } ): EndpointRef; external name '_OTOpenEndpointInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTAsyncOpenEndpointInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTAsyncOpenEndpointInContext( config: OTConfigurationRef; oflag: OTOpenFlags; info: TEndpointInfoPtr { can be NULL }; upp: OTNotifyUPP; contextPtr: UnivPtr; clientContext: OTClientContextPtr { can be NULL } ): OSStatus; external name '_OTAsyncOpenEndpointInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  OTOpenEndpoint()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTAsyncOpenEndpoint()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ The following macros may be used by applications only.}
// #define OTOpenEndpoint(config, oflag, info, err)  OTOpenEndpointInContext(config, oflag, info, err, NULL)
// #define OTAsyncOpenEndpoint(config, oflag, info, proc, contextPtr)  OTAsyncOpenEndpointInContext(config, oflag, info, proc, contextPtr, NULL)

{ Misc Information}

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
 *  OTGetEndpointInfo()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTGetEndpointInfo( ref: EndpointRef; var info: TEndpointInfo ): OSStatus; external name '_OTGetEndpointInfo';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTGetEndpointState()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTGetEndpointState( ref: EndpointRef ): OTResult; external name '_OTGetEndpointState';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTLook()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTLook( ref: EndpointRef ): OTResult; external name '_OTLook';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2_2

{$endc} {not TARGET_CPU_64}

{
 *  OTSync()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


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
 *  OTCountDataBytes()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTCountDataBytes( ref: EndpointRef; var countPtr: OTByteCount ): OTResult; external name '_OTCountDataBytes';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTGetProtAddress()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTGetProtAddress( ref: EndpointRef; boundAddr: TBindPtr { can be NULL }; peerAddr: TBindPtr { can be NULL } ): OSStatus; external name '_OTGetProtAddress';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTResolveAddress()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTResolveAddress( ref: EndpointRef; var reqAddr: TBind; var retAddr: TBind; timeOut: OTTimeout ): OSStatus; external name '_OTResolveAddress';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Allocating structures}

{
   Note:
   In general, Apple recommends that you avoid the OTAlloc call because
   using it extensively causes your program to allocate and deallocate
   many memory blocks, with each extra memory allocation costing time.
}

{
   Under Carbon, OTAlloc takes a client context pointer.  Applications may pass NULL after
   calling InitOpenTransport(kInitOTForApplicationMask, ...).  Non-applications must always pass a
   valid client context.
}
{
 *  OTAllocInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTAllocInContext( ref: EndpointRef; structType: OTStructType; fields: UInt32; var err: OSStatus; clientContext: OTClientContextPtr { can be NULL } ): UnivPtr; external name '_OTAllocInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2_2

{$endc} {not TARGET_CPU_64}

{
 *  OTAlloc()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc OTCARBONAPPLICATION}
{ The following macro may be used by applications only.}
// #define OTAlloc(ref, structType, fields, err) OTAllocInContext(ref, structType, fields, err, NULL)
{$endc} {OTCARBONAPPLICATION}

{$ifc not TARGET_CPU_64}
=======
{ The following macro may be used by applications only.}
// #define OTAlloc(ref, structType, fields, err) OTAllocInContext(ref, structType, fields, err, NULL)

>>>>>>> graemeg/fixes_2_2
=======
=======
{$ifc OTCARBONAPPLICATION}
>>>>>>> origin/fixes_2.4
{ The following macro may be used by applications only.}
// #define OTAlloc(ref, structType, fields, err) OTAllocInContext(ref, structType, fields, err, NULL)
{$endc} {OTCARBONAPPLICATION}

<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  OTFree()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTFree( ptr: UnivPtr; structType: OTStructType ): OTResult; external name '_OTFree';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Option management}

{ It looks simple enough...}

{
 *  OTOptionManagement()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTOptionManagement( ref: EndpointRef; var req: TOptMgmt; var ret: TOptMgmt ): OSStatus; external name '_OTOptionManagement';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ ... but then the hidden complexity emerges.}

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
{
 *  OTCreateOptions()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }


{
 *  OTCreateOptionString()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }


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
 *  OTNextOption()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTNextOption( buffer: UInt8Ptr; buflen: UInt32; var prevOptPtr: TOptionPtr ): OSStatus; external name '_OTNextOption';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTFindOption()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTFindOption( buffer: UInt8Ptr; buflen: UInt32; level: OTXTILevel; name: OTXTIName ): TOptionPtr; external name '_OTFindOption';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Bind/Unbind}

{
 *  OTBind()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTBind( ref: EndpointRef; reqAddr: TBindPtr { can be NULL }; retAddr: TBindPtr { can be NULL } ): OSStatus; external name '_OTBind';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTUnbind()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTUnbind( ref: EndpointRef ): OSStatus; external name '_OTUnbind';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Connection creation/tear-down}

{
 *  OTConnect()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTConnect( ref: EndpointRef; var sndCall: TCall; rcvCall: TCallPtr { can be NULL } ): OSStatus; external name '_OTConnect';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTRcvConnect()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTRcvConnect( ref: EndpointRef; call: TCallPtr { can be NULL } ): OSStatus; external name '_OTRcvConnect';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTListen()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTListen( ref: EndpointRef; var call: TCall ): OSStatus; external name '_OTListen';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTAccept()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTAccept( listener: EndpointRef; worker: EndpointRef; var call: TCall ): OSStatus; external name '_OTAccept';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSndDisconnect()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTSndDisconnect( ref: EndpointRef; call: TCallPtr { can be NULL } ): OSStatus; external name '_OTSndDisconnect';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSndOrderlyDisconnect()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTSndOrderlyDisconnect( ref: EndpointRef ): OSStatus; external name '_OTSndOrderlyDisconnect';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTRcvDisconnect()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTRcvDisconnect( ref: EndpointRef; discon: TDisconPtr { can be NULL } ): OSStatus; external name '_OTRcvDisconnect';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTRcvOrderlyDisconnect()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTRcvOrderlyDisconnect( ref: EndpointRef ): OSStatus; external name '_OTRcvOrderlyDisconnect';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Connection-oriented send/receive}

{
 *  OTRcv()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTRcv( ref: EndpointRef; buf: UnivPtr; nbytes: OTByteCount; var flags: OTFlags ): OTResult; external name '_OTRcv';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSnd()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTSnd( ref: EndpointRef; buf: UnivPtr; nbytes: OTByteCount; flags: OTFlags ): OTResult; external name '_OTSnd';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Connectionless send/receive}

{
 *  OTSndUData()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTSndUData( ref: EndpointRef; var udata: TUnitData ): OSStatus; external name '_OTSndUData';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTRcvUData()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTRcvUData( ref: EndpointRef; var udata: TUnitData; var flags: OTFlags ): OSStatus; external name '_OTRcvUData';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTRcvUDErr()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTRcvUDErr( ref: EndpointRef; uderr: TUDErrPtr { can be NULL } ): OSStatus; external name '_OTRcvUDErr';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Connection-oriented transactions}

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
{
 *  OTSndRequest()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTRcvReply()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTSndReply()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTRcvRequest()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTCancelRequest()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTCancelReply()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{ Connectionless transactions}

{
 *  OTSndURequest()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTRcvUReply()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTSndUReply()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTRcvURequest()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTCancelURequest()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{
 *  OTCancelUReply()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }


{ Interface to Mappers}


{
   Under Carbon, the OpenMapper routines take a client context pointer.  Applications may pass NULL after
   calling InitOpenTransport(kInitOTForApplicationMask, ...).  Non-applications must always pass a
   valid client context.
}

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
 *  OTAsyncOpenMapperInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTAsyncOpenMapperInContext( config: OTConfigurationRef; oflag: OTOpenFlags; upp: OTNotifyUPP; contextPtr: UnivPtr; clientContext: OTClientContextPtr { can be NULL } ): OSStatus; external name '_OTAsyncOpenMapperInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTOpenMapperInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTOpenMapperInContext( config: OTConfigurationRef; oflag: OTOpenFlags; var err: OSStatus; clientContext: OTClientContextPtr { can be NULL } ): MapperRef; external name '_OTOpenMapperInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2_2

{$endc} {not TARGET_CPU_64}

{
 *  OTAsyncOpenMapper()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTOpenMapper()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ The following macros may be used by applications only.}
// #define OTAsyncOpenMapper(config, oflag, proc, contextPtr) OTAsyncOpenMapperInContext(config, oflag, proc, contextPtr, NULL)
// #define OTOpenMapper(config, oflag, err) OTOpenMapperInContext(config, oflag, err, NULL)

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
 *  OTRegisterName()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTRegisterName( ref: MapperRef; var req: TRegisterRequest; var reply: TRegisterReply ): OSStatus; external name '_OTRegisterName';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTDeleteName()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTDeleteName( ref: MapperRef; var name: TNetbuf ): OSStatus; external name '_OTDeleteName';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTDeleteNameByID()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTDeleteNameByID( ref: MapperRef; nameID: OTNameID ): OSStatus; external name '_OTDeleteNameByID';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTLookupName()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
function OTLookupName( ref: MapperRef; var req: TLookupRequest; var reply: TLookupReply ): OSStatus; external name '_OTLookupName';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Basic configuration manipulation}

{
 *  OTCreateConfiguration()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }
function OTCreateConfiguration( path: ConstCStringPtr ): OTConfigurationRef; external name '_OTCreateConfiguration';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTCloneConfiguration()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }
function OTCloneConfiguration( cfig: OTConfigurationRef ): OTConfigurationRef; external name '_OTCloneConfiguration';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTDestroyConfiguration()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }
procedure OTDestroyConfiguration( cfig: OTConfigurationRef ); external name '_OTDestroyConfiguration';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   This file defines a very limited set of operations
   on a configuration.  "OpenTransportClient.h" extends this with extra
   operations used by protocol stacks but not typically needed by clients.
}

{ Interrupt-safe memory allocators}

{
   Under Carbon, OTAllocMem takes a client context pointer.  Applications may pass NULL after
   calling InitOpenTransport(kInitOTForApplicationMask, ...).  Non-applications must always pass a
   valid client context.
}

{
 *  OTAllocMemInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTAllocMemInContext( size: OTByteCount; clientContext: OTClientContextPtr { can be NULL } ): UnivPtr; external name '_OTAllocMemInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

=======
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{
 *  OTAllocMem()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


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
 *  OTFreeMem()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientUtilLib 1.0 and later
 }
procedure OTFreeMem( mem: UnivPtr ); external name '_OTFreeMem';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{ The following macros may be used by applications only.}
// #define OTAllocMem(s) OTAllocMemInContext(s, NULL)

{ Miscellaneous and Generic Routines}

{
   Neither of these routines should be necessary to the correct
   operation of an OT program.  If you're calling them, think again.
}

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
 *  OTDelay()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
procedure OTDelay( seconds: UInt32 ); external name '_OTDelay';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTIdle()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTClientLib 1.0 and later
 }
procedure OTIdle; external name '_OTIdle';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}

{$endc} {not TARGET_CPU_64}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2_2

{$endc} {not TARGET_CPU_64}

{$endc}  { !OTKERNEL }

{
   ***** Open Transport Utility Routines *****
   All of these routines are available to both client and kernel.
}
{ Memory and String Routines}

{
   These are preferable, especially in the kernel case, to the standard
   C equivalents because they don't require you to link with StdCLib.
}

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
 *  OTMemcpy()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTMemcpy( dest: UnivPtr; src: {const} UnivPtr; nBytes: OTByteCount ); external name '_OTMemcpy';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTMemcmp()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTMemcmp( mem1: {const} UnivPtr; mem2: {const} UnivPtr; nBytes: OTByteCount ): Boolean; external name '_OTMemcmp';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTMemmove()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTMemmove( dest: UnivPtr; src: {const} UnivPtr; nBytes: OTByteCount ); external name '_OTMemmove';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTMemzero()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTMemzero( dest: UnivPtr; nBytes: OTByteCount ); external name '_OTMemzero';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTMemset()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTMemset( dest: UnivPtr; toSet: OTUInt8Param; nBytes: OTByteCount ); external name '_OTMemset';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTStrLength()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTStrLength( str: ConstCStringPtr ): OTByteCount; external name '_OTStrLength';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTStrCopy()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTStrCopy( var dest: char; src: ConstCStringPtr ); external name '_OTStrCopy';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTStrCat()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTStrCat( var dest: char; src: ConstCStringPtr ); external name '_OTStrCat';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTStrEqual()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTStrEqual( src1: ConstCStringPtr; src2: ConstCStringPtr ): Boolean; external name '_OTStrEqual';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Timer Utilities}

{
   OTGetTimeStamp returns time in "tick" numbers, stored in 64 bits.
   This timestamp can be used as a base number for calculating elapsed 
   time.
   OTSubtractTimeStamps returns a pointer to the "result" parameter.
    
   OTGetClockTimeInSecs returns time since Open Transport was initialized
   in seconds.
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

type
	OTTimeStamp = UnsignedWide;
	OTTimeStampPtr = ^OTTimeStamp;
{$ifc not TARGET_CPU_64}
=======
type
	OTTimeStamp = UnsignedWide;
	OTTimeStampPtr = ^OTTimeStamp;
>>>>>>> graemeg/fixes_2_2
=======
type
	OTTimeStamp = UnsignedWide;
	OTTimeStampPtr = ^OTTimeStamp;
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

type
	OTTimeStamp = UnsignedWide;
	OTTimeStampPtr = ^OTTimeStamp;
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  OTGetTimeStamp()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTGetTimeStamp( var currentTime: OTTimeStamp ); external name '_OTGetTimeStamp';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTSubtractTimeStamps()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTSubtractTimeStamps( var result: OTTimeStamp; var startTime: OTTimeStamp; var endEnd: OTTimeStamp ): OTTimeStampPtr; external name '_OTSubtractTimeStamps';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTTimeStampInMilliseconds()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTTimeStampInMilliseconds( var delta: OTTimeStamp ): UInt32; external name '_OTTimeStampInMilliseconds';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTTimeStampInMicroseconds()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTTimeStampInMicroseconds( var delta: OTTimeStamp ): UInt32; external name '_OTTimeStampInMicroseconds';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTElapsedMilliseconds()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTElapsedMilliseconds( var startTime: OTTimeStamp ): UInt32; external name '_OTElapsedMilliseconds';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTElapsedMicroseconds()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTElapsedMicroseconds( var startTime: OTTimeStamp ): UInt32; external name '_OTElapsedMicroseconds';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
 *  OTGetClockTimeInSecs()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTGetClockTimeInSecs: UInt32; external name '_OTGetClockTimeInSecs';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ ***** OT Link Element *****}

{
   When using OT linked lists, all pointers to other elements are
   represented by the OTLink structure.  When operating on link
   lists, you always pass in the address of the OTLink on which
   list elements are chained.
}

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
type
	OTLinkPtr = ^OTLink;
	OTLink = record
		fNext: OTLinkPtr;
	end;

{
    You can use this macro to map from an OTLink field to the
  structure in which it's embedded.
}
// #define OTGetLinkObject(link, struc, field)    \
//   ((struc*)((char*)(link) - OTOffsetOf(struc, field)))

{ OTLIFO}

{
   These are functions to implement a LIFO list that is interrupt-safe.
   The only function which is not is OTReverseList.  Normally, you create
   a LIFO list, populate it at interrupt time, and then use OTLIFOStealList
   to atomically remove the list, and OTReverseList to flip the list so that
   it is a FIFO list, which tends to be more useful.
}

type
	OTLIFOPtr = ^OTLIFO;
	OTLIFO = record
		fHead: OTLinkPtr;
	end;
{
   This function atomically enqueues the link onto the
   front of the list.
}
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
 *  OTLIFOEnqueue()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTLIFOEnqueue( var list: OTLIFO; var link: OTLink ); external name '_OTLIFOEnqueue';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   This function atomically dequeues the first element
   on the list.
}
{
 *  OTLIFODequeue()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTLIFODequeue( var list: OTLIFO ): OTLinkPtr; external name '_OTLIFODequeue';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   This function atomically empties the list and returns a
   pointer to the first element on the list.
}
{
 *  OTLIFOStealList()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTLIFOStealList( var list: OTLIFO ): OTLinkPtr; external name '_OTLIFOStealList';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   This function reverses a list that was stolen by
   OTLIFOStealList.  It is NOT atomic.  It returns the
   new starting list.
}
{
 *  OTReverseList()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTReverseList( var list: OTLink ): OTLinkPtr; external name '_OTReverseList';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ OTList}

{
   An OTList is a non-interrupt-safe list, but has more features than the
   OTLIFO list. It is a standard singly-linked list.
}

{
   The following is the prototype for a list element comparison function,
   which returns true if the element described by linkToCheck matches
   the client criteria (typically held in ref).
   This is only a UPP for CFM-68K clients.
}

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
type
	OTListSearchProcPtr = function( ref: {const} UnivPtr; var linkToCheck: OTLink ): Boolean;
	OTListSearchUPP = OTListSearchProcPtr;
{
 *  NewOTListSearchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOTListSearchUPP( userRoutine: OTListSearchProcPtr ): OTListSearchUPP; external name '_NewOTListSearchUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeOTListSearchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeOTListSearchUPP( userUPP: OTListSearchUPP ); external name '_DisposeOTListSearchUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeOTListSearchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeOTListSearchUPP( ref: {const} UnivPtr; var linkToCheck: OTLink; userUPP: OTListSearchUPP ): Boolean; external name '_InvokeOTListSearchUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2

type
	OTListPtr = ^OTList;
	OTList = record
		fHead: OTLinkPtr;
	end;
{ Add the link to the list at the front}
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
 *  OTAddFirst()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTAddFirst( var list: OTList; var link: OTLink ); external name '_OTAddFirst';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Add the link to the list at the end}
{
 *  OTAddLast()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTAddLast( var list: OTList; var link: OTLink ); external name '_OTAddLast';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Remove the first link from the list}
{
 *  OTRemoveFirst()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTRemoveFirst( var list: OTList ): OTLinkPtr; external name '_OTRemoveFirst';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Remove the last link from the list}
{
 *  OTRemoveLast()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTRemoveLast( var list: OTList ): OTLinkPtr; external name '_OTRemoveLast';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Return the first link from the list}
{
 *  OTGetFirst()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTGetFirst( var list: OTList ): OTLinkPtr; external name '_OTGetFirst';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Return the last link from the list}
{
 *  OTGetLast()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTGetLast( var list: OTList ): OTLinkPtr; external name '_OTGetLast';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Return true if the link is present in the list}
{
 *  OTIsInList()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTIsInList( var list: OTList; var link: OTLink ): Boolean; external name '_OTIsInList';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   Find a link in the list which matches the search criteria
   established by the search proc and the refPtr.  This is done
   by calling the search proc, passing it the refPtr and each
   link in the list, until the search proc returns true.
   NULL is returned if the search proc never returned true.
}
{
 *  OTFindLink()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTFindLink( var list: OTList; proc: OTListSearchUPP; ref: {const} UnivPtr ): OTLinkPtr; external name '_OTFindLink';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Remove the specified link from the list, returning true if it was found}
{
 *  OTRemoveLink()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTRemoveLink( var list: OTList; var link: OTLink ): Boolean; external name '_OTRemoveLink';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Similar to OTFindLink, but it also removes it from the list.}
{
 *  OTFindAndRemoveLink()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTFindAndRemoveLink( var list: OTList; proc: OTListSearchUPP; ref: {const} UnivPtr ): OTLinkPtr; external name '_OTFindAndRemoveLink';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Return the "index"th link in the list}
{
 *  OTGetIndexedLink()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTGetIndexedLink( var list: OTList; index: OTItemCount ): OTLinkPtr; external name '_OTGetIndexedLink';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ OTEnqueue/OTDequeue}

{
   These routines are atomic, mighty weird, and generally not
   worth the complexity.  If you need atomic list operations,
   use OTLIFO instead.
}

{
   This function puts "object" on the listHead, and places the
   previous value at listHead into the pointer at "object" plus
   linkOffset.
}
{
 *  OTEnqueue()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
procedure OTEnqueue( var listHead: UnivPtr; objct: UnivPtr; linkOffset: OTByteCount ); external name '_OTEnqueue';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   This function returns the head object of the list, and places
   the pointer at "object" + linkOffset into the listHead
}
{
 *  OTDequeue()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTDequeue( var listHead: UnivPtr; linkOffset: OTByteCount ): UnivPtr; external name '_OTDequeue';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Atomic Operations}

{
   Note:
   The Bit operations return the previous value of the bit (0 or non-zero).
   The memory pointed to must be a single byte and only bits 0 through 7 are
   valid.  Bit 0 corresponds to a mask of 0x01, and Bit 7 to a mask of 0x80.
}

{
   WARNING!
   void* and UInt32 locations MUST be on 4-byte boundaries.
   UInt16 locations must not cross a 4-byte boundary.
}

{
 *  OTAtomicSetBit()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTAtomicSetBit( bytePtr: UInt8Ptr; bitNumber: OTByteCount ): Boolean; external name '_OTAtomicSetBit';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   bset.b d0,(a0)
   sne d0
   moveq #1,d1
   and.l d1,d0
}
{
 *  OTAtomicClearBit()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTAtomicClearBit( bytePtr: UInt8Ptr; bitNumber: OTByteCount ): Boolean; external name '_OTAtomicClearBit';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   bclr.b d0,(a0)
   sne d0
   moveq #1,d1
   and.l d1,d0
}
{
 *  OTAtomicTestBit()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTAtomicTestBit( bytePtr: UInt8Ptr; bitNumber: OTByteCount ): Boolean; external name '_OTAtomicTestBit';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   btst.b d0,(a0)
   sne d0 *|
   moveq #1,d1
   and.l d1,d0 *|
}
{
 *  OTCompareAndSwapPtr()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTCompareAndSwapPtr( oldValue: UnivPtr; newValue: UnivPtr; var dest: UnivPtr ): Boolean; external name '_OTCompareAndSwapPtr';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   cas.l    d0,d1,(a0)  *|
   seq      d0          *|
   moveq #1,d1; and.l d1,d0 *|
}
{
 *  OTCompareAndSwap32()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTCompareAndSwap32( oldValue: UInt32; newValue: UInt32; var dest: UInt32 ): Boolean; external name '_OTCompareAndSwap32';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   cas.l    d0,d1,(a0)  *|
   seq      d0          *|
   moveq #1,d1; and.l d1,d0 *|
}
{
 *  OTCompareAndSwap16()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTCompareAndSwap16( oldValue: UInt32; newValue: UInt32; var dest: UInt16 ): Boolean; external name '_OTCompareAndSwap16';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   cas.w    d0,d1,(a0)  *|
   seq      d0          *|
   moveq #1,d1; and.l d1,d0 *|
}
{
 *  OTCompareAndSwap8()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTCompareAndSwap8( oldValue: UInt32; newValue: UInt32; var dest: UInt8 ): Boolean; external name '_OTCompareAndSwap8';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   cas.b    d0,d1,(a0)  *|
   seq      d0          *|
   moveq #1,d1; and.l d1,d0 *|
}
{
 *  OTAtomicAdd32()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTAtomicAdd32( toAdd: SInt32; var dest: SInt32 ): SInt32; external name '_OTAtomicAdd32';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{
   move.l   d0,a1       *|
   move.l   (a0),d1     *|
   move.l   d1,d0       *|
   add.l    a1,d0       *|
   cas.l    d1,d0,(a0)  *|
   bne.s    @1          *|
}
{
 *  OTAtomicAdd16()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTAtomicAdd16( toAdd: SInt32; var dest: SInt16 ): SInt16; external name '_OTAtomicAdd16';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Not used frequently enough to justify inlining.}
{
 *  OTAtomicAdd8()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }
function OTAtomicAdd8( toAdd: SInt32; var dest: SInt8 ): SInt8; external name '_OTAtomicAdd8';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Not used frequently enough to justify inlining.}
{ OTLock is just a convenience type with some convenient macros.}

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

type
	OTLock = UInt8;

// #define OTClearLock(lockPtr)   *(lockPtr) = 0
// #define OTAcquireLock(lockPtr)   (OTAtomicSetBit(lockPtr, 0) == 0)

{******************************************************************************
**
** FROM HERE ON DOWN ARE THE C++ Interfaces to Open Transport
**
*******************************************************************************}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======


end.
>>>>>>> graemeg/fixes_2_2
=======


end.
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
