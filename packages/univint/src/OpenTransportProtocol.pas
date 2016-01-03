{
<<<<<<< HEAD
<<<<<<< HEAD
     File:       OSServices/OpenTransportProtocol.h
 
     Contains:   *** DEPRECATED *** Definitions likely to be used by low-level protocol stack implementation.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  (c) 1993-2011 Apple Inc. and Mentat Inc. All rights reserved.
=======
     Version:    OpenTransport-110~114
 
     Copyright:  й 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  й 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  й 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  й 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       OT/OpenTransportProtocol.h
 
     Contains:   Definitions likely to be used by low-level protocol stack implementation.
 
     Version:    OpenTransport-110~114
 
<<<<<<< HEAD
     Copyright:  й 1993-2005 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  й 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{      Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{      Pascal Translation Updated: Jonas Maebe <jonas@freepascal.org>, September 2012 }
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
{      Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{      Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit OpenTransportProtocol;
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
<<<<<<< HEAD
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
=======
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
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
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
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
uses MacTypes,ConditionalMacros,Files,CodeFragments,OpenTransport;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{ this header is only supported on Mac OS X < 10.4, and Mac OS X < 10.4 does
  not support i386
}
{$ifc TARGET_OS_MAC and TARGET_CPU_PPC}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ALIGN MAC68K}

{ ***** Setup Default Compiler Variables *****}

{
   OTKERNEL is used to indicate whether the code is being built
   for the kernel environment.  It defaults to 0.  If you include
   "OpenTransportKernel.h" before including this file,
   it will be 1 and you will only be able to see stuff available
   to kernel code.
   As we've included "OpenTransport.h" and it defaults this variable
   to 0 if it's not already been defined, it should always be defined
   by the time we get here.  So we just assert that.  Assertions in
   header files!  Cool (-:
}

{$ifc undefined OTKERNEL}
{$setc OTKERNEL := 0}
{$endc}

{ ***** Shared Client Memory *****}
{$ifc NOT OTKERNEL}
{
   These allocators allocate memory in the shared client pool,
   which is shared between all clients and is not disposed when
   a particular client goes away.  See DTS Technote еее
   "Understanding Open Transport Memory Management" for details.
}
{
 *  OTAllocSharedClientMem()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTFreeSharedClientMem()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$endc}  { !OTKERNEL }

{ ***** UNIX Types *****}
{$ifc CALL_NOT_IN_CARBON}
{
   On UNIX, uid_t and gid_t are defined to be big enough
   to hold a user ID and group ID respectively.  As Mac OS
   has no such concepts, we just define them as UInt32 place
   holders.
}
type
	uid_t = UInt32;
	gid_t = UInt32;
{ Similarly, dev_t is a UNIX type for denoting a device number.}
type
	dev_t = UInt32;
{ ***** From the Mentat "strstat.h" *****}

{ module statistics structure }
type
	module_statPtr = ^module_stat;
	module_stat = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		ms_pcnt: SIGNEDLONG;                { count of calls to put proc }
		ms_scnt: SIGNEDLONG;                { count of calls to service proc }
		ms_ocnt: SIGNEDLONG;                { count of calls to open proc }
		ms_ccnt: SIGNEDLONG;                { count of calls to close proc }
		ms_acnt: SIGNEDLONG;                { count of calls to admin proc }
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		ms_pcnt: SInt32;                { count of calls to put proc }
		ms_scnt: SInt32;                { count of calls to service proc }
		ms_ocnt: SInt32;                { count of calls to open proc }
		ms_ccnt: SInt32;                { count of calls to close proc }
		ms_acnt: SInt32;                { count of calls to admin proc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
		ms_xptr: UnivPtr;                { pointer to private statistics }
		ms_xsize: SInt16;               { length of private statistics buffer }
	end;
{ ***** From the Mentat "cred.h" *****}

type
	credPtr = ^cred;
	cred = record
		cr_ref: UInt16;                 { reference count on processes using cred structures }
		cr_ngroups: UInt16;             { number of groups in cr_groups }
		cr_uid: uid_t;                 { effective user id }
		cr_gid: gid_t;                 { effective group id }
		cr_ruid: uid_t;                { real user id }
		cr_rgid: gid_t;                { real group id }
		cr_suid: uid_t;                { user id saved by exec }
		cr_sgid: gid_t;                { group id saved by exec }
  	cr_groups: array [0..0] of gid_t;           { supplementary groups list }
	end;
type
	cred_t = cred;
{ Free return structure for esballoc }
type
	FreeFuncType = procedure( var arg: char );
	free_rtnPtr = ^free_rtn;
	free_rtn = record
		free_func: FreeFuncType;              { Routine to call to free buffer }
		free_arg: UnivPtr;               { Parameter to free_func }
	end;
type
	frtn_t = free_rtn;
	frtn_tPtr = ^frtn_t;
{ data descriptor }
type
	databPtr = ^datab;
	datab_db_f = record
		case SInt16 of
		0: (
			freep:				databPtr;
			);
		1: (
			frtnp:				free_rtnPtr;
			);
	end;
	datab_db_fPtr = ^datab_db_f;
	datab = record
		db_f: datab_db_f;
		db_base: UInt8Ptr;                { first byte of buffer }
		db_lim: UInt8Ptr;                 { last byte+1 of buffer }
		db_ref: UInt8;                 { count of messages pointing to block}
		db_type: UInt8;                { message type }
		db_iswhat: UInt8;              { message status }
		db_filler2: UInt8;             { for spacing }
		db_size: UInt32;                { used internally }
		db_msgaddr: UInt8Ptr;             { used internally }
		db_filler: SInt32;
	end;


type
	dblk_t = datab;
	dblk_tPtr = ^dblk_t;

{ message block }
type
	msgbPtr = ^msgb;
	msgb = record
		b_next: struct msgb *;                 { next message on queue }
		b_prev: struct msgb *;                 { previous message on queue }
		b_cont: struct msgb *;                 { next message block of message }
		b_rptr: UInt8Ptr;                 { first unread data byte in buffer }
		b_wptr: UInt8Ptr;                 { first unwritten data byte }
		b_datap: databPtr;                { data block }
		b_band: UInt8;                 { message priority }
		b_pad1: UInt8;
		b_flag: UInt16;
	end;
type
	mblk_t = msgb;
	mblk_tPtr = ^mblk_t;
{ mblk flags }
const
	MSGMARK = $01; { last byte of message is tagged }
	MSGNOLOOP = $02; { don't pass message to write-side of stream }
	MSGDELIM = $04; { message is delimited }
	MSGNOGET = $08;

{ STREAMS environments are expected to define these constants in a public header file.}

const
	STRCTLSZ = 256;  { Maximum Control buffer size for messages   }
	STRMSGSZ = 8192;  { Maximum # data bytes for messages   }

{ Message types }
const
	QNORM = 0;
	M_DATA = 0;    { Ordinary data }
	M_PROTO = 1;    { Internal control info and data }
	M_BREAK = $08; { Request a driver to send a break }
	M_PASSFP = $09; { Used to pass a file pointer }
	M_SIG = $0B; { Requests a signal to be sent }
	M_DELAY = $0C; { Request a real-time delay }
	M_CTL = $0D; { For inter-module communication }
	M_IOCTL = $0E; { Used internally for I_STR requests }
	M_SETOPTS = $10; { Alters characteristics of Stream head }
	M_RSE = $11;  { Reserved for internal use }

{ MPS private type }
const
	M_MI = $40;
	M_MI_READ_RESET = 1;
	M_MI_READ_SEEK = 2;
	M_MI_READ_END = 4;

{ Priority messages types }
const
	QPCTL = $80;
	M_IOCACK = $81; { Positive ack of previous M_IOCTL }
	M_IOCNAK = $82; { Previous M_IOCTL failed }
	M_PCPROTO = $83; { Same as M_PROTO except for priority }
	M_PCSIG = $84; { Priority signal }
	M_FLUSH = $86; { Requests modules to flush queues }
	M_STOP = $87; { Request drivers to stop output }
	M_START = $88; { Request drivers to start output }
	M_HANGUP = $89; { Driver can no longer produce data }
	M_ERROR = $8A; { Reports downstream error condition }
	M_READ = $8B; { Reports client read at Stream head }
	M_COPYIN = $8C; { Requests the Stream to copy data in for a module }
	M_COPYOUT = $8D; { Requests the Stream to copy data out for a module }
	M_IOCDATA = $8E; { Status from M_COPYIN/M_COPYOUT message }
	M_PCRSE = $90; { Reserved for internal use }
	M_STOPI = $91; { Request drivers to stop input }
	M_STARTI = $92; { Request drivers to start input }
	M_HPDATA = $93;  { MPS-private type; high priority data }

{ Defines for flush messages }
const
	FLUSHALL = 1;
	FLUSHDATA = 0;


const
	NOERROR = -1;    { used in M_ERROR messages }

type
	sth_sPtr = ^sth_s;
	sth_s = record
		dummy:					UInt32;
	end;

	sqh_sPtr = ^sqh_s;
	sqh_s = record
		dummy:					UInt32;
	end;

	q_xtraPtr = ^q_xtra;
	q_xtra = record
		dummy:					UInt32;
	end;

{$ifc OTKERNEL}
{
   module_info is aligned differently on 68K than
   on PowerPC.  Yucky.  I can't defined a conditionalised
   pad field because a) you can't conditionalise specific
   fields in the interface definition language used to
   create Universal Interfaces, and b) lots of code 
   assigns C structured constants to global variables
   of this type, and these assignments break if you
   add an extra field to the type.  Instead, I
   set the alignment appropriately before defining the 
   structure.  The problem with doing that is that
   the interface definition language doesn't allow
   my to set the alignment in the middle of a file,
   so I have to do this via "pass throughs".  This
   works fine for the well known languages (C, Pascal),
   but may cause problems for other languages (Java,
   Asm).
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_CPU_PPC}
{$ALIGN POWER}
{$endc} {TARGET_CPU_PPC}
=======
{$ALIGN POWER}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN POWER}
>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_CPU_PPC}
{$ALIGN POWER}
{$endc} {TARGET_CPU_PPC}
>>>>>>> origin/fixes_2.4
type
	module_info = record
		mi_idnum: UInt16;               { module ID number }
		mi_idname: UnivPtr;              { module name }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		mi_minpsz: SIGNEDLONG;              { min pkt size, for developer use }
		mi_maxpsz: SIGNEDLONG;              { max pkt size, for developer use }
		mi_hiwat: UNSIGNEDLONG;               { hi-water mark, for flow control }
		mi_lowat: UNSIGNEDLONG;               { lo-water mark, for flow control }
<<<<<<< HEAD
	end;
	module_infoPtr = ^module_info;
{$ifc TARGET_CPU_PPC}
{$ALIGN MAC68K}
{$endc} {TARGET_CPU_PPC}
=======
=======
>>>>>>> origin/fixes_2_2
		mi_minpsz: SInt32;              { min pkt size, for developer use }
		mi_maxpsz: SInt32;              { max pkt size, for developer use }
		mi_hiwat: UInt32;               { hi-water mark, for flow control }
		mi_lowat: UInt32;               { lo-water mark, for flow control }
=======
>>>>>>> origin/fixes_2.4
	end;
	module_infoPtr = ^module_info;
{$ifc TARGET_CPU_PPC}
{$ALIGN MAC68K}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_CPU_PPC}
>>>>>>> origin/fixes_2.4


type
	queuePtr = ^queue;
	admin_t = function: OTInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	bufcall_t = procedure( size: SIGNEDLONG );
	bufcallp_t = procedure( size: SIGNEDLONG );
=======
	bufcall_t = procedure( size: SInt32 );
	bufcallp_t = procedure( size: SInt32 );
>>>>>>> graemeg/fixes_2_2
=======
	bufcall_t = procedure( size: SInt32 );
	bufcallp_t = procedure( size: SInt32 );
>>>>>>> origin/fixes_2_2
=======
	bufcall_t = procedure( size: SIGNEDLONG );
	bufcallp_t = procedure( size: SIGNEDLONG );
>>>>>>> origin/fixes_2.4
	closep_t = function( q: queuePtr; foo: OTInt32; var cred: cred_t ): OTInt32;
	old_closep_t = function( q: queuePtr ): OTInt32;
	openp_t = function( q: queuePtr; var dev: dev_t; foo: OTInt32; bar: OTInt32; var cred: cred_t ): OTInt32;
	openOld_t = function( q: queuePtr; dev: dev_t; foo: OTInt32; bar: OTInt32 ): OTInt32;
	old_openp_t = function( q: queuePtr; dev: dev_t; foo: OTInt32; bar: OTInt32 ): OTInt32;
	closeOld_t = function( q: queuePtr ): OTInt32;
	putp_t = function( q: queuePtr; var mp: msgb ): OTInt32;
	srvp_t = function( q: queuePtr ): OTInt32;
	qinitPtr = ^qinit;
	qinit = record
		qi_putp: putp_t;                { put procedure }
		qi_srvp: srvp_t;                { service procedure }
		qi_qopen: openp_t;               { called on each open or a push }
		qi_qclose: closep_t;              { called on last close or a pop }
		qi_qadmin: admin_t;              { reserved for future use }
		qi_minfo: module_infoPtr;               { information structure }
		qi_mstat: module_statPtr;               { statistics structure - optional }
	end;
{ defines module or driver }
	streamtabPtr = ^streamtab;
	streamtab = record
		st_rdinit: qinitPtr;              { defines read QUEUE }
		st_wrinit: qinitPtr;              { defines write QUEUE }
		st_muxrinit: qinitPtr;            { for multiplexing drivers only }
		st_muxwinit: qinitPtr;            { ditto }
	end;
	qbandPtr = ^qband;
	qband = record
		qb_next: struct qband *;                { next band for this queue }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		qb_count: UNSIGNEDLONG;               { weighted count of characters in this band }
		qb_first: msgbPtr;               { head of message queue }
		qb_last: msgbPtr;                { tail of message queue }
		qb_hiwat: UNSIGNEDLONG;               { high water mark }
		qb_lowat: UNSIGNEDLONG;               { low water mark }
=======
=======
>>>>>>> origin/fixes_2_2
		qb_count: UInt32;               { weighted count of characters in this band }
		qb_first: msgbPtr;               { head of message queue }
		qb_last: msgbPtr;                { tail of message queue }
		qb_hiwat: UInt32;               { high water mark }
		qb_lowat: UInt32;               { low water mark }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		qb_count: UNSIGNEDLONG;               { weighted count of characters in this band }
		qb_first: msgbPtr;               { head of message queue }
		qb_last: msgbPtr;                { tail of message queue }
		qb_hiwat: UNSIGNEDLONG;               { high water mark }
		qb_lowat: UNSIGNEDLONG;               { low water mark }
>>>>>>> origin/fixes_2.4
		qb_flag: UInt16;                { еее╩state }
		qb_pad1: SInt16;                { еее reserved }
	end;
	qband_t = qband;
	qband_tPtr = ^qband_t;
	queue_q_uPtr = ^queue_q_u;
	queue_q_u = record
		case SInt16 of
		0: (
			q_u_link:			queuePtr;								{  link to scheduling queue  }
			);
		1: (
			q_u_sqh_parent:		sqh_sPtr;
			);
	end;
	queue = record
		q_qinfo: qinitPtr;                { procedures and limits for queue }
		q_first: msgbPtr;                { head of message queue }
		q_last: msgbPtr;                 { tail of message queue }
		q_next: queuePtr;                 { next queue in Stream }
		q_u: queue_q_u;
		q_ptr: UnivPtr;                  { to private data structure }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		q_count: UNSIGNEDLONG;                { weighted count of characters on q }
		q_minpsz: SIGNEDLONG;               { min packet size accepted }
		q_maxpsz: SIGNEDLONG;               { max packet size accepted }
		q_hiwat: UNSIGNEDLONG;                { high water mark, for flow control }
		q_lowat: UNSIGNEDLONG;                { low water mark }
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		q_count: UInt32;                { weighted count of characters on q }
		q_minpsz: SInt32;               { min packet size accepted }
		q_maxpsz: SInt32;               { max packet size accepted }
		q_hiwat: UInt32;                { high water mark, for flow control }
		q_lowat: UInt32;                { low water mark }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
		q_bandp: qbandPtr;                { band information }
		q_flag: UInt16;                 { еее queue state }
		q_nband: UInt8;                { еее number of bands }
		q_pad1: SInt8;              { еее reserved }
		q_osx: q_xtraPtr;                  { Pointer to OS-dependent extra stuff }
		q_ffcp: queuePtr;                 { Forward flow control pointer }
		q_bfcp: queuePtr;                 { Backward flow control pointer }
	end;
	queue_t = queue;
	queue_tPtr = ^queue_t;

{ queue_t flag defines }
const
	QREADR = $01; { This queue is a read queue }
	QNOENB = $02; { Don't enable in putq }
	QFULL = $04; { The queue is full }
	QWANTR = $08; { The queue should be scheduled in the next putq }
	QWANTW = $10; { The stream should be back enabled when this queue drains }
	QUSE = $20; { The queue is allocated and ready for use }
	QENAB = $40; { The queue is scheduled (on the run queue) }
	QBACK = $80; { The queue has been back enabled }
	QOLD = $0100; { Module supports old style opens and closes }
	QHLIST = $0200; { The Stream head is doing something with this queue (Not supported by MPS) }
	QWELDED = $0400; { Mentat flag for welded queues }
	QUNWELDING = $0800; { Queue is scheduled to be unwelded }
	QPROTECTED = $1000; { Mentat flag for unsafe q access }
	QEXCOPENCLOSE = $2000; { Queue wants exclusive open/close calls }

{ qband_t flag defines }
const
	QB_FULL = $01; { The band is full }
	QB_WANTW = $02; { The stream should be back enabled when this band/queue drains }
	QB_BACK = $04;  { The queue has been back enabled }

{$elsec}
{
   Client code views a queue_t as a simple cookie.
   The real definition lives above and is only available
   to kernel code.
}
type
	queue_t = SInt32;
	queue_tPtr = ^queue_t;
{$endc}  {OTKERNEL}

{ structure contained in M_COPYIN/M_COPYOUT messages }
type
	caddr_t = ^char;
	copyreqPtr = ^copyreq;
	copyreq = record
		cq_cmd: SInt32;                 { ioctl command (from ioc_cmd) }
		cq_cr: credPtr;                  { pointer to full credentials }
		cq_id: UInt32;                  { ioctl id (from ioc_id) }
		cq_addr: caddr_t;                { address to copy data to/from }
		cq_size: UInt32;                { number of bytes to copy }
		cq_flag: SInt32;                { state }
		cq_private: mblk_tPtr;             { private state information }
		cq_filler: array [0..3] of SInt32;
	end;


{ copyreq defines }
const
	STRCANON = $01; { b_cont data block contains canonical format specifier }
	RECOPY = $02;  { perform I_STR copyin again this time using canonical format specifier }

{ structure contained in M_IOCDATA message block }
type
	copyrespPtr = ^copyresp;
	copyresp = record
		cp_cmd: SInt32;                 { ioctl command (from ioc_cmd) }
		cp_cr: credPtr;                  { pointer to full credentials }
		cp_id: UInt32;                  { ioctl id (from ioc_id) }
		cp_rval: caddr_t;                { status of request; 0 for success; error value for failure }
		cp_pad1: UInt32;
		cp_pad2: SInt32;
		cp_private: mblk_tPtr;             { private state information }
		cp_filler: array [0..3] of SInt32;
	end;

{ structure contained in an M_IOCTL message block }
type
	iocblkPtr = ^iocblk;
	iocblk = record
		ioc_cmd: SInt32;                { ioctl command type }
		ioc_cr: credPtr;                 { pointer to full credentials }
		ioc_id: UInt32;                 { ioctl id }
		ioc_count: UInt32;              { count of bytes in data field }
		ioc_error: SInt32;              { error code }
		ioc_rval: SInt32;               { return value }
		ioc_filler: array [0..3] of SInt32;
	end;


const
	TRANSPARENT = $FFFFFFFF;

{ Used in M_IOCTL mblks to muxes (ioc_cmd I_LINK) }
type
	linkblkPtr = ^linkblk;
	linkblk = record
		l_qtop: queue_tPtr;                 { lowest level write queue of upper stream }
		l_qbot: queue_tPtr;                 { highest level write queue of lower stream }
		l_index: SInt32;                { system-unique index for lower stream }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  	l_pad: array [0..4] of SIGNEDLONG;
=======
  	l_pad: array [0..4] of SInt32;
>>>>>>> graemeg/fixes_2_2
=======
  	l_pad: array [0..4] of SInt32;
>>>>>>> origin/fixes_2_2
=======
  	l_pad: array [0..4] of SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
{ structure contained in an M_PASSFP message block }
type
	strpfp = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		pass_file_cookie: UNSIGNEDLONG;       { file 'pointer' }
=======
		pass_file_cookie: UInt32;       { file 'pointer' }
>>>>>>> graemeg/fixes_2_2
=======
		pass_file_cookie: UInt32;       { file 'pointer' }
>>>>>>> origin/fixes_2_2
=======
		pass_file_cookie: UNSIGNEDLONG;       { file 'pointer' }
>>>>>>> origin/fixes_2.4
		pass_uid: UInt16;               { user id of sending stream }
		pass_gid: UInt16;
		pass_sth: sth_sPtr;               { Stream head pointer of passed stream }
	end;
{ structure contained in an M_SETOPTS message block }
type
	stroptions = packed record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		so_flags: UNSIGNEDLONG;               { options to set }
		so_readopt: SInt16;             { read option }
		so_wroff: UInt16;               { write offset }
		so_minpsz: SIGNEDLONG;              { minimum read packet size }
		so_maxpsz: SIGNEDLONG;              { maximum read packet size }
		so_hiwat: UNSIGNEDLONG;               { read queue high-water mark }
		so_lowat: UNSIGNEDLONG;               { read queue low-water mark }
		so_band: UInt8;                { band for water marks }
  	so_filler: packed array [0..2] of UInt8;           { added for alignment }
		so_poll_set: UNSIGNEDLONG;            { poll events to set }
		so_poll_clr: UNSIGNEDLONG;            { poll events to clear }
=======
=======
>>>>>>> origin/fixes_2_2
		so_flags: UInt32;               { options to set }
=======
		so_flags: UNSIGNEDLONG;               { options to set }
>>>>>>> origin/fixes_2.4
		so_readopt: SInt16;             { read option }
		so_wroff: UInt16;               { write offset }
		so_minpsz: SIGNEDLONG;              { minimum read packet size }
		so_maxpsz: SIGNEDLONG;              { maximum read packet size }
		so_hiwat: UNSIGNEDLONG;               { read queue high-water mark }
		so_lowat: UNSIGNEDLONG;               { read queue low-water mark }
		so_band: UInt8;                { band for water marks }
  	so_filler: packed array [0..2] of UInt8;           { added for alignment }
<<<<<<< HEAD
		so_poll_set: UInt32;            { poll events to set }
		so_poll_clr: UInt32;            { poll events to clear }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		so_poll_set: UNSIGNEDLONG;            { poll events to set }
		so_poll_clr: UNSIGNEDLONG;            { poll events to clear }
>>>>>>> origin/fixes_2.4
	end;
{ definitions for so_flags field }
const
	SO_ALL = $7FFF; { Update all options }
	SO_READOPT = $0001; { Set the read mode }
	SO_WROFF = $0002; { Insert an offset in write M_DATA mblks }
	SO_MINPSZ = $0004; { Change the min packet size on sth rq }
	SO_MAXPSZ = $0008; { Change the max packet size on sth rq }
	SO_HIWAT = $0010; { Change the high water mark on sth rq }
	SO_LOWAT = $0020; { Change the low water mark }
	SO_MREADON = $0040; { Request M_READ messages }
	SO_MREADOFF = $0080; { Don't gen M_READ messages }
	SO_NDELON = $0100; { old TTY semantics for O_NDELAY reads and writes }
	SO_NDELOFF = $0200; { STREAMS semantics for O_NDELAY reads and writes }
	SO_ISTTY = $0400; { Become a controlling tty }
	SO_ISNTTY = $0800; { No longer a controlling tty }
	SO_TOSTOP = $1000; { Stop on background writes }
	SO_TONSTOP = $2000; { Don't stop on background writes }
	SO_BAND = $4000; { Water marks are for a band }
	SO_POLL_SET = $8000; { Set events to poll }
	SO_POLL_CLR = $00010000; { Clear events to poll }

{ Buffer Allocation Priority }
const
	BPRI_LO = 1;
	BPRI_MED = 2;
	BPRI_HI = 3;

const
	INFPSZ = -1;


{* Test whether message is a data message }
// #define datamsg(type)   ((type) == M_DATA || (type) == M_PROTO || (type) == M_PCPROTO  ||  (type) == M_DELAY)

const
	CLONEOPEN = $02;
	MODOPEN = $01;
	OPENFAIL = -1;


{ Enumeration values for strqget and strqset }
type
	qfields = SInt32;
const
	QHIWAT = 0;
	QLOWAT = 1;
	QMAXPSZ = 2;
	QMINPSZ = 3;
	QCOUNT = 4;
	QFIRST = 5;
	QLAST = 6;
	QFLAG = 7;
	QBAD = 8;


type
	qfields_t = qfields;
{$endc} { CALL_NOT_IN_CARBON }

{ ***** From the Mentat "stropts.h" *****}


const
	I_NREAD = $4101;						{  return the number of bytes in 1st msg  }
	I_PUSH = $4102;						{  push module just below stream head  }
	I_POP = $4103;						{  pop module below stream head  }
	I_LOOK = $4104;						{  retrieve name of first stream module  }
	I_FLUSH = $4105;						{  flush all input and/or output queues  }
	I_SRDOPT = $4106;						{  set the read mode  }
	I_GRDOPT = $4107;						{  get the current read mode  }
	I_STR = $4108;						{  create an internal ioctl message       }
	I_SETSIG = $4109;						{  request SIGPOLL signal on events  }
	I_GETSIG = $410A;						{  query the registered events  }
	I_FIND = $410B;						{  check for module in stream           }
	I_LINK = $410C;						{  connect stream under mux fd  }
	I_UNLINK = $410D;						{  disconnect two streams  }
	I_PEEK = $410F;						{  peek at data on read queue  }
	I_FDINSERT = $4110;						{  create a message and send downstream  }
	I_SENDFD = $4111;						{  send an fd to a connected pipe stream  }
	I_RECVFD = $4112;						{  retrieve a file descriptor  }
	I_FLUSHBAND = $4113;						{  flush a particular input and/or output band  }
	I_SWROPT = $4114;						{  set the write mode  }
	I_GWROPT = $4115;						{  get the current write mode  }
	I_LIST = $4116;						{  get a list of all modules on a stream   }
	I_ATMARK = $4117;						{  check to see if the next message is "marked"  }
	I_CKBAND = $4118;						{  check for a message of a particular band  }
	I_GETBAND = $4119;						{  get the band of the next message to be read  }
	I_CANPUT = $411A;						{  check to see if a message may be passed on a stream  }
	I_SETCLTIME = $411B;						{  set the close timeout wait  }
	I_GETCLTIME = $411C;						{  get the current close timeout wait  }
	I_PLINK = $411D;						{  permanently connect a stream under a mux  }
	I_PUNLINK = $411E;						{  disconnect a permanent link  }
	I_GETMSG = $4128;						{  getmsg() system call  }
	I_PUTMSG = $4129;						{  putmsg() system call  }
	I_POLL = $412A;						{  poll() system call  }
	I_SETDELAY = $412B;						{  set blocking status  }
	I_GETDELAY = $412C;						{  get blocking status  }
	I_RUN_QUEUES = $412D;						{  sacrifice for the greater good  }
	I_GETPMSG = $412E;						{  getpmsg() system call  }
	I_PUTPMSG = $412F;						{  putpmsg() system call  }
	I_AUTOPUSH = $4130;						{  for systems that cannot do the autopush in open  }
	I_PIPE = $4131;						{  for pipe library call  }
	I_HEAP_REPORT = $4132;						{  get heap statistics  }
	I_FIFO = $4133;						{  for fifo library call  }

{ priority message request on putmsg() or strpeek }
const
	RS_HIPRI = $01;

{ flags for getpmsg and putpmsg }
const
	MSG_HIPRI = $01;
	MSG_BAND = $02; { Retrieve a message from a particular band }
	MSG_ANY = $04;  { Retrieve a message from any band }

{ return values from getmsg(), 0 indicates all ok }
const
	MORECTL = $01; { more control info available }
	MOREDATA = $02;  { more data available }


const
	FMNAMESZ = 31;    { maximum length of a module or device name }


{ Infinite poll wait time }
const
	INFTIM = $FFFFFFFF;

{ flush requests }
const
	FLUSHR = $01; { Flush the read queue }
	FLUSHW = $02; { Flush the write queue }
	FLUSHRW = FLUSHW or FLUSHR; { Flush both }

const
	FLUSHBAND = $40;  { Flush a particular band }

{
   Mentat's code does an #ifdef on this symbol, so we have to #define
   it as well as declare it as an enum.  But only for Apple builds because
   we don't want internal weirdness to propagate to developers.
}
// #define FLUSHBAND FLUSHBAND
{ I_FLUSHBAND }
type
	bandinfoPtr = ^bandinfo;
	bandinfo = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		bi_pri: UInt8;                 { Band to flush }
=======
		bi_pri: SInt8;                 { Band to flush }
>>>>>>> graemeg/fixes_2_2
=======
		bi_pri: SInt8;                 { Band to flush }
>>>>>>> origin/fixes_2_2
=======
		bi_pri: UInt8;                 { Band to flush }
>>>>>>> origin/fixes_2.4
		pad1: SInt8;
		bi_flag: SInt32;                { One of the above flush requests }
	end;
{ flags for I_ATMARK }
const
	ANYMARK = $01; { Check if message is marked }
	LASTMARK = $02;  { Check if this is the only message marked }

{ signal event masks }
const
	S_INPUT = $01; { A non-M_PCPROTO message has arrived }
	S_HIPRI = $02; { A priority (M_PCPROTO) message is available }
	S_OUTPUT = $04; { The write queue is no longer full }
	S_MSG = $08; { A signal message has reached the front of read queue }
	S_RDNORM = $10; { A non-priority message is available }
	S_RDBAND = $20; { A banded messsage is available }
	S_WRNORM = $40; { Same as S_OUTPUT }
	S_WRBAND = $80; { A priority band exists and is writable }
	S_ERROR = $0100; { Error message has arrived }
	S_HANGUP = $0200; { Hangup message has arrived }
	S_BANDURG = $0400; { Use SIGURG instead of SIGPOLL on S_RDBAND signals }

{ read mode bits for I_S|GRDOPT; choose one of the following }
const
	RNORM = $01; { byte-stream mode, default }
	RMSGD = $02; { message-discard mode }
	RMSGN = $04; { message-nondiscard mode }
	RFILL = $08;  { fill read buffer mode (PSE private) }

{ More read modes, these are bitwise or'ed with the modes above }
const
	RPROTNORM = $10; { Normal handling of M_PROTO/M_PCPROTO messages, default }
	RPROTDIS = $20; { Discard M_PROTO/M_PCPROTO message blocks }
	RPROTDAT = $40;  { Convert M_PROTO/M_PCPROTO message blocks into M_DATA }

{ write modes for I_S|GWROPT }
const
	SNDZERO = $01;  { Send a zero-length message downstream on a write of zero bytes }

const
	MUXID_ALL = -1;    { Unlink all lower streams for I_UNLINK and I_PUNLINK }

{
   strbuf is moved to "OpenTransport.h" because that header file
   exports provider routines that take it as a parameter.
}

{ structure of ioctl data on I_FDINSERT }
type
	strfdinsertPtr = ^strfdinsert;
	strfdinsert = record
		ctlbuf: strbuf;
		databuf: strbuf;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: SIGNEDLONG;                  { type of message, 0 or RS_HIPRI }
		fildes: SIGNEDLONG;                 { fd of other stream (FDCELL) }
=======
		flags: SInt32;                  { type of message, 0 or RS_HIPRI }
		fildes: SInt32;                 { fd of other stream (FDCELL) }
>>>>>>> graemeg/fixes_2_2
=======
		flags: SInt32;                  { type of message, 0 or RS_HIPRI }
		fildes: SInt32;                 { fd of other stream (FDCELL) }
>>>>>>> origin/fixes_2_2
=======
		flags: SIGNEDLONG;                  { type of message, 0 or RS_HIPRI }
		fildes: SIGNEDLONG;                 { fd of other stream (FDCELL) }
>>>>>>> origin/fixes_2.4
		offset: SInt32;                 { where to put other stream read qp }
	end;
{ I_LIST structures }
type
	str_mlistPtr = ^str_mlist;
	str_mlist = record
		l_name: packed array [0..31] of char;
	end;
type
	str_listPtr = ^str_list;
	str_list = record
		sl_nmods: SInt32;               { number of modules in sl_modlist array }
		sl_modlist: str_mlistPtr;
	end;
{ I_PEEK structure }
type
	strpeekPtr = ^strpeek;
	strpeek = record
		ctlbuf: strbuf;
		databuf: strbuf;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: SIGNEDLONG;                  { if RS_HIPRI, get priority messages only }
=======
		flags: SInt32;                  { if RS_HIPRI, get priority messages only }
>>>>>>> graemeg/fixes_2_2
=======
		flags: SInt32;                  { if RS_HIPRI, get priority messages only }
>>>>>>> origin/fixes_2_2
=======
		flags: SIGNEDLONG;                  { if RS_HIPRI, get priority messages only }
>>>>>>> origin/fixes_2.4
	end;
{ structure for getpmsg and putpmsg }
type
	strpmsgPtr = ^strpmsg;
	strpmsg = record
		ctlbuf: strbuf;
		databuf: strbuf;
		band: SInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: SIGNEDLONG;
=======
		flags: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		flags: SInt32;
>>>>>>> origin/fixes_2_2
=======
		flags: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
{ structure of ioctl data on I_RECVFD }
type
	strrecvfdPtr = ^strrecvfd;
	strrecvfd = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		fd: SIGNEDLONG;                     { new file descriptor (FDCELL) }
=======
		fd: SInt32;                     { new file descriptor (FDCELL) }
>>>>>>> graemeg/fixes_2_2
=======
		fd: SInt32;                     { new file descriptor (FDCELL) }
>>>>>>> origin/fixes_2_2
=======
		fd: SIGNEDLONG;                     { new file descriptor (FDCELL) }
>>>>>>> origin/fixes_2.4
		uid: UInt16;                    { user id of sending stream }
		gid: UInt16;
		fill: packed array [0..7] of char;
	end;
{ structure of ioctl data on I_STR }
type
	strioctlPtr = ^strioctl;
	strioctl = record
		ic_cmd: SInt32;                 { downstream command }
		ic_timout: SInt32;              { ACK/NAK timeout }
		ic_len: SInt32;                 { length of data arg }
		ic_dp: UnivPtr;                  { ptr to data arg }
	end;
{ ***** From the Mentat "strlog.h" *****}

type
	log_ctlPtr = ^log_ctl;
	log_ctl = record
		mid: SInt16;
		sid: SInt16;
		level: SInt8;
		pad1: SInt8;
		flags: SInt16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		ltime: SIGNEDLONG;
		ttime: SIGNEDLONG;
=======
		ltime: SInt32;
		ttime: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		ltime: SInt32;
		ttime: SInt32;
>>>>>>> origin/fixes_2_2
=======
		ltime: SIGNEDLONG;
		ttime: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
		seq_no: SInt32;
	end;
const
	SL_FATAL = $01; { Fatal error }
	SL_NOTIFY = $02; { Notify the system administrator }
	SL_ERROR = $04; { Pass message to error logger }
	SL_TRACE = $08; { Pass message to tracer }
	SL_CONSOLE = $00; { Console messages are disabled }
	SL_WARN = $20; { Warning }
	SL_NOTE = $40;  { Notice this message }

type
	trace_idsPtr = ^trace_ids;
	trace_ids = record
		ti_mid: SInt16;
		ti_sid: SInt16;
		ti_level: char;
	end;
const
	I_TRCLOG = $6201;
	I_ERRLOG = $6202;

const
	LOGMSGSZ = 128;

{ ***** From the Mentat "tihdr.h" *****}

{$ifc CALL_NOT_IN_CARBON}

{ TPI Primitives}


const
	T_BIND_REQ = 101;
	T_CONN_REQ = 102;           { connection request }
	T_CONN_RES = 103;           { respond to connection indication }
	T_DATA_REQ = 104;
	T_DISCON_REQ = 105;
	T_EXDATA_REQ = 106;
	T_INFO_REQ = 107;
	T_OPTMGMT_REQ = 108;
	T_ORDREL_REQ = 109;
	T_UNBIND_REQ = 110;
	T_UNITDATA_REQ = 111;
	T_ADDR_REQ = 112;           { Get address request              }
	T_UREQUEST_REQ = 113;       { UnitRequest (transaction) req    }
	T_REQUEST_REQ = 114;        { Request (CO transaction) req     }
	T_UREPLY_REQ = 115;         { UnitRequest (transaction) req    }
	T_REPLY_REQ = 116;          { REPLY (CO transaction) req       }
	T_CANCELREQUEST_REQ = 117;  { Cancel outgoing request          }
	T_CANCELREPLY_REQ = 118;    { Cancel incoming request          }
	T_REGNAME_REQ = 119;        { Request name registration        }
	T_DELNAME_REQ = 120;        { Request delete name registration }
	T_LKUPNAME_REQ = 121;       { Request name lookup              }

	T_BIND_ACK = 122;
	T_CONN_CON = 123;           { connection confirmation          }
	T_CONN_IND = 124;           { incoming connection indication   }
	T_DATA_IND = 125;
	T_DISCON_IND = 126;
	T_ERROR_ACK = 127;
	T_EXDATA_IND = 128;
	T_INFO_ACK = 129;
	T_OK_ACK = 130;
	T_OPTMGMT_ACK = 131;
	T_ORDREL_IND = 132;
	T_UNITDATA_IND = 133;
	T_UDERROR_IND = 134;
	T_ADDR_ACK = 135;           { Get address ack                  }
	T_UREQUEST_IND = 136;       { UnitRequest (transaction) ind    }
	T_REQUEST_IND = 137;        { Request (CO transaction) ind     }
	T_UREPLY_IND = 138;         { Incoming unit reply              }
	T_REPLY_IND = 139;          { Incoming reply                   }
	T_UREPLY_ACK = 140;         { outgoing Unit Reply is complete  }
	T_REPLY_ACK = 141;          { outgoing Reply is complete       }
	T_RESOLVEADDR_REQ = 142;
	T_RESOLVEADDR_ACK = 143;
	T_LKUPNAME_CON = 146;       { Results of name lookup           }
	T_LKUPNAME_RES = 147;       { Partial results of name lookup   }
	T_REGNAME_ACK = 148;        { Request name registration        }
	T_SEQUENCED_ACK = 149;      { Sequenced version of OK or ERROR ACK }

	T_EVENT_IND = 160;           { Miscellaneous event Indication       }

{ State values }
const
	TS_UNBND = 1;
	TS_WACK_BREQ = 2;
	TS_WACK_UREQ = 3;
	TS_IDLE = 4;
	TS_WACK_OPTREQ = 5;
	TS_WACK_CREQ = 6;
	TS_WCON_CREQ = 7;
	TS_WRES_CIND = 8;
	TS_WACK_CRES = 9;
	TS_DATA_XFER = 10;
	TS_WIND_ORDREL = 11;
	TS_WREQ_ORDREL = 12;
	TS_WACK_DREQ6 = 13;
	TS_WACK_DREQ7 = 14;
	TS_WACK_DREQ9 = 15;
	TS_WACK_DREQ10 = 16;
	TS_WACK_DREQ11 = 17;
	TS_WACK_ORDREL = 18;
	TS_NOSTATES = 19;
	TS_BAD_STATE = 19;

{ Transport events }
const
	TE_OPENED = 1;
	TE_BIND = 2;
	TE_OPTMGMT = 3;
	TE_UNBIND = 4;
	TE_CLOSED = 5;
	TE_CONNECT1 = 6;
	TE_CONNECT2 = 7;
	TE_ACCEPT1 = 8;
	TE_ACCEPT2 = 9;
	TE_ACCEPT3 = 10;
	TE_SND = 11;
	TE_SNDDIS1 = 12;
	TE_SNDDIS2 = 13;
	TE_SNDREL = 14;
	TE_SNDUDATA = 15;
	TE_LISTEN = 16;
	TE_RCVCONNECT = 17;
	TE_RCV = 18;
	TE_RCVDIS1 = 19;
	TE_RCVDIS2 = 20;
	TE_RCVDIS3 = 21;
	TE_RCVREL = 22;
	TE_RCVUDATA = 23;
	TE_RCVUDERR = 24;
	TE_PASS_CONN = 25;
	TE_BAD_EVENT = 26;

type
	T_addr_ackPtr = ^T_addr_ack;
	T_addr_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_ADDR_ACK }
		LOCADDR_length: SIGNEDLONG;
		LOCADDR_offset: SIGNEDLONG;
		REMADDR_length: SIGNEDLONG;
		REMADDR_offset: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_ADDR_ACK }
		LOCADDR_length: SInt32;
		LOCADDR_offset: SInt32;
		REMADDR_length: SInt32;
		REMADDR_offset: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_addr_reqPtr = ^T_addr_req;
	T_addr_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { Always T_ADDR_REQ }
=======
		PRIM_type: SInt32;              { Always T_ADDR_REQ }
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { Always T_ADDR_REQ }
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { Always T_ADDR_REQ }
>>>>>>> origin/fixes_2.4
	end;
type
	T_bind_ackPtr = ^T_bind_ack;
	T_bind_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_BIND_ACK }
		ADDR_length: SIGNEDLONG;
		ADDR_offset: SIGNEDLONG;
		CONIND_number: UNSIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_BIND_ACK }
		ADDR_length: SInt32;
		ADDR_offset: SInt32;
		CONIND_number: UInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_bind_reqPtr = ^T_bind_req;
	T_bind_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_BIND_REQ }
		ADDR_length: SIGNEDLONG;
		ADDR_offset: SIGNEDLONG;
		CONIND_number: UNSIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_BIND_REQ }
		ADDR_length: SInt32;
		ADDR_offset: SInt32;
		CONIND_number: UInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_conn_conPtr = ^T_conn_con;
	T_conn_con = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_CONN_CON }
		RES_length: SIGNEDLONG;             { responding address length }
		RES_offset: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_CONN_CON }
		RES_length: SInt32;             { responding address length }
		RES_offset: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_conn_indPtr = ^T_conn_ind;
	T_conn_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_CONN_IND }
		SRC_length: SIGNEDLONG;
		SRC_offset: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		SEQ_number: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_CONN_IND }
		SRC_length: SInt32;
		SRC_offset: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		SEQ_number: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_conn_reqPtr = ^T_conn_req;
	T_conn_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_CONN_REQ }
		DEST_length: SIGNEDLONG;
		DEST_offset: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_CONN_REQ }
		DEST_length: SInt32;
		DEST_offset: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_conn_resPtr = ^T_conn_res;
	T_conn_res = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_CONN_RES }
		QUEUE_ptr: queue_tPtr;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		SEQ_number: SIGNEDLONG;
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_CONN_RES }
		QUEUE_ptr: queue_tPtr;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		SEQ_number: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_CONN_RES }
		QUEUE_ptr: queue_tPtr;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		SEQ_number: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_data_indPtr = ^T_data_ind;
	T_data_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_DATA_IND }
		MORE_flag: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { always T_DATA_IND }
		MORE_flag: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_DATA_IND }
		MORE_flag: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_DATA_IND }
		MORE_flag: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_data_reqPtr = ^T_data_req;
	T_data_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_DATA_REQ }
		MORE_flag: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { always T_DATA_REQ }
		MORE_flag: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_DATA_REQ }
		MORE_flag: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_DATA_REQ }
		MORE_flag: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_discon_indPtr = ^T_discon_ind;
	T_discon_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_DISCON_IND }
		DISCON_reason: SIGNEDLONG;
		SEQ_number: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { always T_DISCON_IND }
		DISCON_reason: SInt32;
		SEQ_number: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_DISCON_IND }
		DISCON_reason: SInt32;
		SEQ_number: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_DISCON_IND }
		DISCON_reason: SIGNEDLONG;
		SEQ_number: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_discon_reqPtr = ^T_discon_req;
	T_discon_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_DISCON_REQ }
		SEQ_number: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { always T_DISCON_REQ }
		SEQ_number: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_DISCON_REQ }
		SEQ_number: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_DISCON_REQ }
		SEQ_number: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_exdata_indPtr = ^T_exdata_ind;
	T_exdata_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_EXDATA_IND }
		MORE_flag: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { always T_EXDATA_IND }
		MORE_flag: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_EXDATA_IND }
		MORE_flag: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_EXDATA_IND }
		MORE_flag: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_exdata_reqPtr = ^T_exdata_req;
	T_exdata_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_EXDATA_REQ }
		MORE_flag: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { always T_EXDATA_REQ }
		MORE_flag: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_EXDATA_REQ }
		MORE_flag: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_EXDATA_REQ }
		MORE_flag: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_error_ackPtr = ^T_error_ack;
	T_error_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_ERROR_ACK }
		ERROR_prim: SIGNEDLONG;             { primitive in error }
		TLI_error: SIGNEDLONG;
		UNIX_error: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_ERROR_ACK }
		ERROR_prim: SInt32;             { primitive in error }
		TLI_error: SInt32;
		UNIX_error: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_info_ackPtr = ^T_info_ack;
	T_info_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_INFO_ACK }
		TSDU_size: SIGNEDLONG;              { max TSDU size }
		ETSDU_size: SIGNEDLONG;             { max ETSDU size }
		CDATA_size: SIGNEDLONG;             { connect data size }
		DDATA_size: SIGNEDLONG;             { disconnect data size }
		ADDR_size: SIGNEDLONG;              { TSAP size }
		OPT_size: SIGNEDLONG;               { options size }
		TIDU_size: SIGNEDLONG;              { TIDU size }
		SERV_type: SIGNEDLONG;              { service type }
		CURRENT_state: SIGNEDLONG;          { current state }
		PROVIDER_flag: SIGNEDLONG;          { provider flags (see xti.h for defines) }
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_INFO_ACK }
		TSDU_size: SInt32;              { max TSDU size }
		ETSDU_size: SInt32;             { max ETSDU size }
		CDATA_size: SInt32;             { connect data size }
		DDATA_size: SInt32;             { disconnect data size }
		ADDR_size: SInt32;              { TSAP size }
		OPT_size: SInt32;               { options size }
		TIDU_size: SInt32;              { TIDU size }
		SERV_type: SInt32;              { service type }
		CURRENT_state: SInt32;          { current state }
		PROVIDER_flag: SInt32;          { provider flags (see xti.h for defines) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
{ Provider flags }
const
	SENDZERO = $0001; { supports 0-length TSDU's }
	XPG4_1 = $0002; { provider supports recent stuff }

type
	T_info_reqPtr = ^T_info_req;
	T_info_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_INFO_REQ }
=======
		PRIM_type: SInt32;              { always T_INFO_REQ }
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_INFO_REQ }
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_INFO_REQ }
>>>>>>> origin/fixes_2.4
	end;
type
	T_ok_ackPtr = ^T_ok_ack;
	T_ok_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_OK_ACK }
		CORRECT_prim: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { always T_OK_ACK }
		CORRECT_prim: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_OK_ACK }
		CORRECT_prim: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_OK_ACK }
		CORRECT_prim: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_optmgmt_ackPtr = ^T_optmgmt_ack;
	T_optmgmt_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_OPTMGMT_ACK }
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		MGMT_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_OPTMGMT_ACK }
		OPT_length: SInt32;
		OPT_offset: SInt32;
		MGMT_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_optmgmt_reqPtr = ^T_optmgmt_req;
	T_optmgmt_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_OPTMGMT_REQ }
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		MGMT_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_OPTMGMT_REQ }
		OPT_length: SInt32;
		OPT_offset: SInt32;
		MGMT_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_ordrel_indPtr = ^T_ordrel_ind;
	T_ordrel_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_ORDREL_IND }
=======
		PRIM_type: SInt32;              { always T_ORDREL_IND }
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_ORDREL_IND }
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_ORDREL_IND }
>>>>>>> origin/fixes_2.4
	end;
type
	T_ordrel_reqPtr = ^T_ordrel_req;
	T_ordrel_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_ORDREL_REQ }
=======
		PRIM_type: SInt32;              { always T_ORDREL_REQ }
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_ORDREL_REQ }
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_ORDREL_REQ }
>>>>>>> origin/fixes_2.4
	end;
type
	T_unbind_reqPtr = ^T_unbind_req;
	T_unbind_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_UNBIND_REQ }
=======
		PRIM_type: SInt32;              { always T_UNBIND_REQ }
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_UNBIND_REQ }
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_UNBIND_REQ }
>>>>>>> origin/fixes_2.4
	end;
type
	T_uderror_indPtr = ^T_uderror_ind;
	T_uderror_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_UDERROR_IND }
		DEST_length: SIGNEDLONG;
		DEST_offset: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		ERROR_type: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_UDERROR_IND }
		DEST_length: SInt32;
		DEST_offset: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		ERROR_type: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_unitdata_indPtr = ^T_unitdata_ind;
	T_unitdata_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_UNITDATA_IND }
		SRC_length: SIGNEDLONG;
		SRC_offset: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_UNITDATA_IND }
		SRC_length: SInt32;
		SRC_offset: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_unitdata_reqPtr = ^T_unitdata_req;
	T_unitdata_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_UNITDATA_REQ }
		DEST_length: SIGNEDLONG;
		DEST_offset: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_UNITDATA_REQ }
		DEST_length: SInt32;
		DEST_offset: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_resolveaddr_ackPtr = ^T_resolveaddr_ack;
	T_resolveaddr_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_RESOLVEADDR_ACK }
		SEQ_number: SIGNEDLONG;
		ADDR_length: SIGNEDLONG;
		ADDR_offset: SIGNEDLONG;
		ORIG_client: SIGNEDLONG;
		ORIG_data: SIGNEDLONG;
		TLI_error: SIGNEDLONG;
		UNIX_error: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_RESOLVEADDR_ACK }
		SEQ_number: SInt32;
		ADDR_length: SInt32;
		ADDR_offset: SInt32;
		ORIG_client: SInt32;
		ORIG_data: SInt32;
		TLI_error: SInt32;
		UNIX_error: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_resolveaddr_reqPtr = ^T_resolveaddr_req;
	T_resolveaddr_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_RESOLVEADDR_REQ }
		SEQ_number: SIGNEDLONG;
		ADDR_length: SIGNEDLONG;
		ADDR_offset: SIGNEDLONG;
		ORIG_client: SIGNEDLONG;
		ORIG_data: SIGNEDLONG;
		MAX_milliseconds: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_RESOLVEADDR_REQ }
		SEQ_number: SInt32;
		ADDR_length: SInt32;
		ADDR_offset: SInt32;
		ORIG_client: SInt32;
		ORIG_data: SInt32;
		MAX_milliseconds: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_unitreply_indPtr = ^T_unitreply_ind;
	T_unitreply_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_UREPLY_IND }
		SEQ_number: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		REP_flags: SIGNEDLONG;
		TLI_error: SIGNEDLONG;
		UNIX_error: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_UREPLY_IND }
		SEQ_number: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		REP_flags: SInt32;
		TLI_error: SInt32;
		UNIX_error: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_unitrequest_indPtr = ^T_unitrequest_ind;
	T_unitrequest_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_UREQUEST_IND }
		SEQ_number: SIGNEDLONG;
		SRC_length: SIGNEDLONG;
		SRC_offset: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		REQ_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_UREQUEST_IND }
		SEQ_number: SInt32;
		SRC_length: SInt32;
		SRC_offset: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		REQ_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_unitrequest_reqPtr = ^T_unitrequest_req;
	T_unitrequest_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_UREQUEST_REQ }
		SEQ_number: SIGNEDLONG;
		DEST_length: SIGNEDLONG;
		DEST_offset: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		REQ_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_UREQUEST_REQ }
		SEQ_number: SInt32;
		DEST_length: SInt32;
		DEST_offset: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		REQ_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_unitreply_reqPtr = ^T_unitreply_req;
	T_unitreply_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_UREPLY_REQ }
		SEQ_number: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		REP_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_UREPLY_REQ }
		SEQ_number: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		REP_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_unitreply_ackPtr = ^T_unitreply_ack;
	T_unitreply_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_UREPLY_ACK }
		SEQ_number: SIGNEDLONG;
		TLI_error: SIGNEDLONG;
		UNIX_error: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_UREPLY_ACK }
		SEQ_number: SInt32;
		TLI_error: SInt32;
		UNIX_error: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_cancelrequest_reqPtr = ^T_cancelrequest_req;
	T_cancelrequest_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { Always T_CANCELREQUEST_REQ }
		SEQ_number: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { Always T_CANCELREQUEST_REQ }
		SEQ_number: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { Always T_CANCELREQUEST_REQ }
		SEQ_number: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { Always T_CANCELREQUEST_REQ }
		SEQ_number: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_cancelreply_reqPtr = ^T_cancelreply_req;
	T_cancelreply_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { Always T_CANCELREPLY_REQ }
		SEQ_number: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { Always T_CANCELREPLY_REQ }
		SEQ_number: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { Always T_CANCELREPLY_REQ }
		SEQ_number: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { Always T_CANCELREPLY_REQ }
		SEQ_number: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
type
	T_reply_indPtr = ^T_reply_ind;
	T_reply_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_REPLY_IND }
		SEQ_number: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		REP_flags: SIGNEDLONG;
		TLI_error: SIGNEDLONG;
		UNIX_error: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_REPLY_IND }
		SEQ_number: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		REP_flags: SInt32;
		TLI_error: SInt32;
		UNIX_error: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_request_indPtr = ^T_request_ind;
	T_request_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_REQUEST_IND }
		SEQ_number: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		REQ_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_REQUEST_IND }
		SEQ_number: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		REQ_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_request_reqPtr = ^T_request_req;
	T_request_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_REQUEST_REQ }
		SEQ_number: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		REQ_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_REQUEST_REQ }
		SEQ_number: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		REQ_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_reply_reqPtr = ^T_reply_req;
	T_reply_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_REPLY_REQ }
		SEQ_number: SIGNEDLONG;
		OPT_length: SIGNEDLONG;
		OPT_offset: SIGNEDLONG;
		REP_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_REPLY_REQ }
		SEQ_number: SInt32;
		OPT_length: SInt32;
		OPT_offset: SInt32;
		REP_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_reply_ackPtr = ^T_reply_ack;
	T_reply_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_REPLY_ACK }
		SEQ_number: SIGNEDLONG;
		TLI_error: SIGNEDLONG;
		UNIX_error: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_REPLY_ACK }
		SEQ_number: SInt32;
		TLI_error: SInt32;
		UNIX_error: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_regname_reqPtr = ^T_regname_req;
	T_regname_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_REGNAME_REQ }
		SEQ_number: SIGNEDLONG;             { Reply is sequence ack }
		NAME_length: SIGNEDLONG;
		NAME_offset: SIGNEDLONG;
		ADDR_length: SIGNEDLONG;
		ADDR_offset: SIGNEDLONG;
		REQ_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_REGNAME_REQ }
		SEQ_number: SInt32;             { Reply is sequence ack }
		NAME_length: SInt32;
		NAME_offset: SInt32;
		ADDR_length: SInt32;
		ADDR_offset: SInt32;
		REQ_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_regname_ackPtr = ^T_regname_ack;
	T_regname_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_REGNAME_ACK     }
		SEQ_number: SIGNEDLONG;
		REG_id: SIGNEDLONG;
		ADDR_length: SIGNEDLONG;
		ADDR_offset: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_REGNAME_ACK     }
		SEQ_number: SInt32;
		REG_id: SInt32;
		ADDR_length: SInt32;
		ADDR_offset: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_delname_reqPtr = ^T_delname_req;
	T_delname_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_DELNAME_REQ }
		SEQ_number: SIGNEDLONG;             { Reply is sequence ack }
		NAME_length: SIGNEDLONG;
		NAME_offset: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_DELNAME_REQ }
		SEQ_number: SInt32;             { Reply is sequence ack }
		NAME_length: SInt32;
		NAME_offset: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_lkupname_reqPtr = ^T_lkupname_req;
	T_lkupname_req = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Always T_LKUPNAME_REQ }
		SEQ_number: SIGNEDLONG;             { Reply is sequence ack }
		NAME_length: SIGNEDLONG;            { ... or T_LKUPNAME_CON }
		NAME_offset: SIGNEDLONG;
		ADDR_length: SIGNEDLONG;
		ADDR_offset: SIGNEDLONG;
		MAX_number: SIGNEDLONG;
		MAX_milliseconds: SIGNEDLONG;
		REQ_flags: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Always T_LKUPNAME_REQ }
		SEQ_number: SInt32;             { Reply is sequence ack }
		NAME_length: SInt32;            { ... or T_LKUPNAME_CON }
		NAME_offset: SInt32;
		ADDR_length: SInt32;
		ADDR_offset: SInt32;
		MAX_number: SInt32;
		MAX_milliseconds: SInt32;
		REQ_flags: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_lkupname_conPtr = ^T_lkupname_con;
	T_lkupname_con = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { Either T_LKUPNAME_CON }
		SEQ_number: SIGNEDLONG;             { Or T_LKUPNAME_RES }
		NAME_length: SIGNEDLONG;
		NAME_offset: SIGNEDLONG;
		RSP_count: SIGNEDLONG;
		RSP_cumcount: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { Either T_LKUPNAME_CON }
		SEQ_number: SInt32;             { Or T_LKUPNAME_RES }
		NAME_length: SInt32;
		NAME_offset: SInt32;
		RSP_count: SInt32;
		RSP_cumcount: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_sequence_ackPtr = ^T_sequence_ack;
	T_sequence_ack = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		PRIM_type: SIGNEDLONG;              { always T_SEQUENCED_ACK     }
		ORIG_prim: SIGNEDLONG;              { original primitive        }
		SEQ_number: SIGNEDLONG;
		TLI_error: SIGNEDLONG;
		UNIX_error: SIGNEDLONG;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		PRIM_type: SInt32;              { always T_SEQUENCED_ACK     }
		ORIG_prim: SInt32;              { original primitive        }
		SEQ_number: SInt32;
		TLI_error: SInt32;
		UNIX_error: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T_event_indPtr = ^T_event_ind;
	T_event_ind = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		PRIM_type: SIGNEDLONG;              { always T_EVENT_IND        }
		EVENT_code: SIGNEDLONG;
		EVENT_cookie: SIGNEDLONG;
=======
		PRIM_type: SInt32;              { always T_EVENT_IND        }
		EVENT_code: SInt32;
		EVENT_cookie: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		PRIM_type: SInt32;              { always T_EVENT_IND        }
		EVENT_code: SInt32;
		EVENT_cookie: SInt32;
>>>>>>> origin/fixes_2_2
=======
		PRIM_type: SIGNEDLONG;              { always T_EVENT_IND        }
		EVENT_code: SIGNEDLONG;
		EVENT_cookie: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
	T_primitivesPtr = ^T_primitives;
	T_primitives = record
		case SInt16 of
		0: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			primType:			SIGNEDLONG;
=======
			primType:			SInt32;
>>>>>>> graemeg/fixes_2_2
=======
			primType:			SInt32;
>>>>>>> origin/fixes_2_2
=======
			primType:			SIGNEDLONG;
>>>>>>> origin/fixes_2.4
			);
		1: (
			taddrack:			T_addr_ack;
			);
		2: (
			tbindack:			T_bind_ack;
			);
		3: (
			tbindreq:			T_bind_req;
			);
		4: (
			tconncon:			T_conn_con;
			);
		5: (
			tconnind:			T_conn_ind;
			);
		6: (
			tconnreq:			T_conn_req;
			);
		7: (
			tconnres:			T_conn_res;
			);
		8: (
			tdataind:			T_data_ind;
			);
		9: (
			tdatareq:			T_data_req;
			);
		10: (
			tdisconind:			T_discon_ind;
			);
		11: (
			tdisconreq:			T_discon_req;
			);
		12: (
			texdataind:			T_exdata_ind;
			);
		13: (
			texdatareq:			T_exdata_req;
			);
		14: (
			terrorack:			T_error_ack;
			);
		15: (
			tinfoack:			T_info_ack;
			);
		16: (
			tinforeq:			T_info_req;
			);
		17: (
			tokack:				T_ok_ack;
			);
		18: (
			toptmgmtack:		T_optmgmt_ack;
			);
		19: (
			toptmgmtreq:		T_optmgmt_req;
			);
		20: (
			tordrelind:			T_ordrel_ind;
			);
		21: (
			tordrelreq:			T_ordrel_req;
			);
		22: (
			tunbindreq:			T_unbind_req;
			);
		23: (
			tuderrorind:		T_uderror_ind;
			);
		24: (
			tunitdataind:		T_unitdata_ind;
			);
		25: (
			tunitdatareq:		T_unitdata_req;
			);
		26: (
			tunitreplyind:		T_unitreply_ind;
			);
		27: (
			tunitrequestind:	T_unitrequest_ind;
			);
		28: (
			tunitrequestreq:	T_unitrequest_req;
			);
		29: (
			tunitreplyreq:		T_unitreply_req;
			);
		30: (
			tunitreplyack:		T_unitreply_ack;
			);
		31: (
			treplyind:			T_reply_ind;
			);
		32: (
			trequestind:		T_request_ind;
			);
		33: (
			trequestreq:		T_request_req;
			);
		34: (
			treplyreq:			T_reply_req;
			);
		35: (
			treplyack:			T_reply_ack;
			);
		36: (
			tcancelreqreq:		T_cancelrequest_req;
			);
		37: (
			tresolvereq:		T_resolveaddr_req;
			);
		38: (
			tresolveack:		T_resolveaddr_ack;
			);
		39: (
			tregnamereq:		T_regname_req;
			);
		40: (
			tregnameack:		T_regname_ack;
			);
		41: (
			tdelnamereq:		T_delname_req;
			);
		42: (
			tlkupnamereq:		T_lkupname_req;
			);
		43: (
			tlkupnamecon:		T_lkupname_con;
			);
		44: (
			tsequenceack:		T_sequence_ack;
			);
		45: (
			teventind:			T_event_ind;
			);
	end;
{ ***** From the Mentat "dlpi.h" *****}

{
   This header file has encoded the values so an existing driver
   or user which was written with the Logical Link Interface(LLI)
   can migrate to the DLPI interface in a binary compatible manner.
   Any fields which require a specific format or value are flagged
   with a comment containing the message LLI compatibility.
}

{ DLPI revision definition history}

const
	DL_CURRENT_VERSION = $02; { current version of dlpi }
	DL_VERSION_2 = $02;  { version of dlpi March 12,1991 }


const
	DL_INFO_REQ = $00; { Information Req, LLI compatibility }
	DL_INFO_ACK = $03; { Information Ack, LLI compatibility }
	DL_ATTACH_REQ = $0B; { Attach a PPA }
	DL_DETACH_REQ = $0C; { Detach a PPA }
	DL_BIND_REQ = $01; { Bind dlsap address, LLI compatibility }
	DL_BIND_ACK = $04; { Dlsap address bound, LLI compatibility }
	DL_UNBIND_REQ = $02; { Unbind dlsap address, LLI compatibility }
	DL_OK_ACK = $06; { Success acknowledgment, LLI compatibility }
	DL_ERROR_ACK = $05; { Error acknowledgment, LLI compatibility }
	DL_SUBS_BIND_REQ = $1B; { Bind Subsequent DLSAP address }
	DL_SUBS_BIND_ACK = $1C; { Subsequent DLSAP address bound }
	DL_SUBS_UNBIND_REQ = $15; { Subsequent unbind }
	DL_ENABMULTI_REQ = $1D; { Enable multicast addresses }
	DL_DISABMULTI_REQ = $1E; { Disable multicast addresses }
	DL_PROMISCON_REQ = $1F; { Turn on promiscuous mode }
	DL_PROMISCOFF_REQ = $20; { Turn off promiscuous mode }
	DL_UNITDATA_REQ = $07; { datagram send request, LLI compatibility }
	DL_UNITDATA_IND = $08; { datagram receive indication, LLI compatibility }
	DL_UDERROR_IND = $09; { datagram error indication, LLI compatibility }
	DL_UDQOS_REQ = $0A; { set QOS for subsequent datagram transmissions }
	DL_CONNECT_REQ = $0D; { Connect request }
	DL_CONNECT_IND = $0E; { Incoming connect indication }
	DL_CONNECT_RES = $0F; { Accept previous connect indication }
	DL_CONNECT_CON = $10; { Connection established }
	DL_TOKEN_REQ = $11; { Passoff token request }
	DL_TOKEN_ACK = $12; { Passoff token ack }
	DL_DISCONNECT_REQ = $13; { Disconnect request }
	DL_DISCONNECT_IND = $14; { Disconnect indication }
	DL_RESET_REQ = $17; { Reset service request }
	DL_RESET_IND = $18; { Incoming reset indication }
	DL_RESET_RES = $19; { Complete reset processing }
	DL_RESET_CON = $1A; { Reset processing complete }
	DL_DATA_ACK_REQ = $21; { data unit transmission request }
	DL_DATA_ACK_IND = $22; { Arrival of a command PDU }
	DL_DATA_ACK_STATUS_IND = $23; { Status indication of DATA_ACK_REQ}
	DL_REPLY_REQ = $24; { Request a DLSDU from the remote }
	DL_REPLY_IND = $25; { Arrival of a command PDU }
	DL_REPLY_STATUS_IND = $26; { Status indication of REPLY_REQ }
	DL_REPLY_UPDATE_REQ = $27; { Hold a DLSDU for transmission }
	DL_REPLY_UPDATE_STATUS_IND = $28; { Status of REPLY_UPDATE req }
	DL_XID_REQ = $29; { Request to send an XID PDU }
	DL_XID_IND = $2A; { Arrival of an XID PDU }
	DL_XID_RES = $2B; { request to send a response XID PDU}
	DL_XID_CON = $2C; { Arrival of a response XID PDU }
	DL_TEST_REQ = $2D; { TEST command request }
	DL_TEST_IND = $2E; { TEST response indication }
	DL_TEST_RES = $2F; { TEST response }
	DL_TEST_CON = $30; { TEST Confirmation }
	DL_PHYS_ADDR_REQ = $31; { Request to get physical addr }
	DL_PHYS_ADDR_ACK = $32; { Return physical addr }
	DL_SET_PHYS_ADDR_REQ = $33; { set physical addr }
	DL_GET_STATISTICS_REQ = $34; { Request to get statistics }
	DL_GET_STATISTICS_ACK = $35;  { Return statistics }

{ DLPI interface states}
const
	DL_UNATTACHED = $04; { PPA not attached }
	DL_ATTACH_PENDING = $05; { Waiting ack of DL_ATTACH_REQ }
	DL_DETACH_PENDING = $06; { Waiting ack of DL_DETACH_REQ }
	DL_UNBOUND = $00; { PPA attached, LLI compatibility }
	DL_BIND_PENDING = $01; { Waiting ack of DL_BIND_REQ, LLI compatibility }
	DL_UNBIND_PENDING = $02; { Waiting ack of DL_UNBIND_REQ, LLI compatibility }
	DL_IDLE = $03; { dlsap bound, awaiting use, LLI compatibility }
	DL_UDQOS_PENDING = $07; { Waiting ack of DL_UDQOS_REQ }
	DL_OUTCON_PENDING = $08; { outgoing connection, awaiting DL_CONN_CON }
	DL_INCON_PENDING = $09; { incoming connection, awaiting DL_CONN_RES }
	DL_CONN_RES_PENDING = $0A; { Waiting ack of DL_CONNECT_RES }
	DL_DATAXFER = $0B; { connection-oriented data transfer }
	DL_USER_RESET_PENDING = $0C; { user initiated reset, awaiting DL_RESET_CON }
	DL_PROV_RESET_PENDING = $0D; { provider initiated reset, awaiting DL_RESET_RES }
	DL_RESET_RES_PENDING = $0E; { Waiting ack of DL_RESET_RES }
	DL_DISCON8_PENDING = $0F; { Waiting ack of DL_DISC_REQ when in DL_OUTCON_PENDING }
	DL_DISCON9_PENDING = $10; { Waiting ack of DL_DISC_REQ when in DL_INCON_PENDING }
	DL_DISCON11_PENDING = $11; { Waiting ack of DL_DISC_REQ when in DL_DATAXFER }
	DL_DISCON12_PENDING = $12; { Waiting ack of DL_DISC_REQ when in DL_USER_RESET_PENDING }
	DL_DISCON13_PENDING = $13; { Waiting ack of DL_DISC_REQ when in DL_DL_PROV_RESET_PENDING }
	DL_SUBS_BIND_PND = $14; { Waiting ack of DL_SUBS_BIND_REQ }
	DL_SUBS_UNBIND_PND = $15;  { Waiting ack of DL_SUBS_UNBIND_REQ }

{ DL_ERROR_ACK error return values}

const
	DL_ACCESS = $02; { Improper permissions for request, LLI compatibility }
	DL_BADADDR = $01; { DLSAP address in improper format or invalid }
	DL_BADCORR = $05; { Sequence number not from outstanding DL_CONN_IND }
	DL_BADDATA = $06; { User data exceeded provider limit }
	DL_BADPPA = $08; { Specified PPA was invalid }
	DL_BADPRIM = $09; { Primitive received is not known by DLS provider }
	DL_BADQOSPARAM = $0A; { QOS parameters contained invalid values }
	DL_BADQOSTYPE = $0B; { QOS structure type is unknown or unsupported }
	DL_BADSAP = $00; { Bad LSAP selector, LLI compatibility }
	DL_BADTOKEN = $0C; { Token used not associated with an active stream }
	DL_BOUND = $0D; { Attempted second bind with dl_max_conind or    }
                                        {    dl_conn_mgmt > 0 on same DLSAP or PPA }
	DL_INITFAILED = $0E; { Physical Link initialization failed }
	DL_NOADDR = $0F; { Provider couldn't allocate alternate address }
	DL_NOTINIT = $10; { Physical Link not initialized }
	DL_OUTSTATE = $03; { Primitive issued in improper state, LLI compatibility }
	DL_SYSERR = $04; { UNIX system error occurred, LLI compatibility }
	DL_UNSUPPORTED = $07; { Requested service not supplied by provider }
	DL_UNDELIVERABLE = $11; { Previous data unit could not be delivered }
	DL_NOTSUPPORTED = $12; { Primitive is known but not supported by DLS provider }
	DL_TOOMANY = $13; { limit exceeded }
	DL_NOTENAB = $14; { Promiscuous mode not enabled }
	DL_BUSY = $15; { Other streams for a particular PPA in the post-attached state }
	DL_NOAUTO = $16; { Automatic handling of XID & TEST responses not supported }
	DL_NOXIDAUTO = $17; { Automatic handling of XID not supported }
	DL_NOTESTAUTO = $18; { Automatic handling of TEST not supported }
	DL_XIDAUTO = $19; { Automatic handling of XID response }
	DL_TESTAUTO = $1A; { AUtomatic handling of TEST response}
	DL_PENDING = $1B;  { pending outstanding connect indications }

{ DLPI media types supported}

const
	DL_CSMACD = $00; { IEEE 802.3 CSMA/CD network, LLI Compatibility }
	DL_TPB = $01; { IEEE 802.4 Token Passing Bus, LLI Compatibility }
	DL_TPR = $02; { IEEE 802.5 Token Passing Ring, LLI Compatibility }
	DL_METRO = $03; { IEEE 802.6 Metro Net, LLI Compatibility }
	DL_ETHER = $04; { Ethernet Bus, LLI Compatibility }
	DL_HDLC = $05; { ISO HDLC protocol support, bit synchronous }
	DL_CHAR = $06; { Character Synchronous protocol support, eg BISYNC }
	DL_CTCA = $07; { IBM Channel-to-Channel Adapter }
	DL_FDDI = $08; { Fiber Distributed data interface }
	DL_OTHER = $09;  { Any other medium not listed above }

{
   DLPI provider service supported.
   These must be allowed to be bitwise-OR for dl_service_mode in
   DL_INFO_ACK.
}
const
	DL_CODLS = $01; { support connection-oriented service }
	DL_CLDLS = $02; { support connectionless data link service }
	DL_ACLDLS = $04;  { support acknowledged connectionless service}

{
   DLPI provider style.
   The DLPI provider style which determines whether a provider
   requires a DL_ATTACH_REQ to inform the provider which PPA
   user messages should be sent/received on.
}

const
	DL_STYLE1 = $0500; { PPA is implicitly bound by open(2) }
	DL_STYLE2 = $0501; { PPA must be explicitly bound via DL_ATTACH_REQ }

{ DLPI Originator for Disconnect and Resets}

const
	DL_PROVIDER = $0700;
	DL_USER = $0701;

{ DLPI Disconnect Reasons}

const
	DL_CONREJ_DEST_UNKNOWN = $0800;
	DL_CONREJ_DEST_UNREACH_PERMANENT = $0801;
	DL_CONREJ_DEST_UNREACH_TRANSIENT = $0802;
	DL_CONREJ_QOS_UNAVAIL_PERMANENT = $0803;
	DL_CONREJ_QOS_UNAVAIL_TRANSIENT = $0804;
	DL_CONREJ_PERMANENT_COND = $0805;
	DL_CONREJ_TRANSIENT_COND = $0806;
	DL_DISC_ABNORMAL_CONDITION = $0807;
	DL_DISC_NORMAL_CONDITION = $0808;
	DL_DISC_PERMANENT_CONDITION = $0809;
	DL_DISC_TRANSIENT_CONDITION = $080A;
	DL_DISC_UNSPECIFIED = $080B;

{ DLPI Reset Reasons}

const
	DL_RESET_FLOW_CONTROL = $0900;
	DL_RESET_LINK_ERROR = $0901;
	DL_RESET_RESYNCH = $0902;

{ DLPI status values for acknowledged connectionless data transfer}

const
	DL_CMD_MASK = $0F; { mask for command portion of status }
	DL_CMD_OK = $00; { Command Accepted }
	DL_CMD_RS = $01; { Unimplemented or inactivated service }
	DL_CMD_UE = $05; { Data Link User interface error }
	DL_CMD_PE = $06; { Protocol error }
	DL_CMD_IP = $07; { Permanent implementation dependent error}
	DL_CMD_UN = $09; { Resources temporarily unavailable }
	DL_CMD_IT = $0F; { Temporary implementation dependent error }
	DL_RSP_MASK = $F0; { mask for response portion of status }
	DL_RSP_OK = $00; { Response DLSDU present }
	DL_RSP_RS = $10; { Unimplemented or inactivated service }
	DL_RSP_NE = $30; { Response DLSDU never submitted }
	DL_RSP_NR = $40; { Response DLSDU not requested }
	DL_RSP_UE = $50; { Data Link User interface error }
	DL_RSP_IP = $70; { Permanent implementation dependent error }
	DL_RSP_UN = $90; { Resources temporarily unavailable }
	DL_RSP_IT = $F0;  { Temporary implementation dependent error }

{ Service Class values for acknowledged connectionless data transfer}

const
	DL_RQST_RSP = $01; { Use acknowledge capability in MAC sublayer}
	DL_RQST_NORSP = $02;  { No acknowledgement service requested }

{ DLPI address type definition}

const
	DL_FACT_PHYS_ADDR = $01; { factory physical address }
	DL_CURR_PHYS_ADDR = $02;  { current physical address }

{ DLPI flag definitions}

const
	DL_POLL_FINAL = $01;  { if set,indicates poll/final bit set}

{ XID and TEST responses supported by the provider}

const
	DL_AUTO_XID = $01; { provider will respond to XID }
	DL_AUTO_TEST = $02;  { provider will respond to TEST }

{ Subsequent bind type}

const
	DL_PEER_BIND = $01; { subsequent bind on a peer addr }
	DL_HIERARCHICAL_BIND = $02;  { subs_bind on a hierarchical addr}

{ DLPI promiscuous mode definitions}

const
	DL_PROMISC_PHYS = $01; { promiscuous mode at phys level }
	DL_PROMISC_SAP = $02; { promiscous mode at sap level }
	DL_PROMISC_MULTI = $03;  { promiscuous mode for multicast }

{ M_DATA "raw" mode }
// #define DLIOCRAW MIOC_CMD(MIOC_DLPI,1)
{
   DLPI Quality Of Service definition for use in QOS structure definitions.
   The QOS structures are used in connection establishment, DL_INFO_ACK,
   and setting connectionless QOS values.
}
{
   Throughput
   
   This parameter is specified for both directions.
}

type
	dl_through_t = record
		dl_target_value: SInt32;        { desired bits/second desired }
		dl_accept_value: SInt32;        { min. acceptable bits/second }
	end;
{
   transit delay specification
   
   This parameter is specified for both directions.
   expressed in milliseconds assuming a DLSDU size of 128 octets.
   The scaling of the value to the current DLSDU size is provider dependent.
}
type
	dl_transdelay_tPtr = ^dl_transdelay_t;
	dl_transdelay_t = record
		dl_target_value: SInt32;        { desired value of service }
		dl_accept_value: SInt32;        { min. acceptable value of service }
	end;
{
   priority specification
   priority range is 0-100, with 0 being highest value.
}

type
	dl_priority_tPtr = ^dl_priority_t;
	dl_priority_t = record
		dl_min: SInt32;
		dl_max: SInt32;
	end;
{ protection specification}
const
	DL_NONE = $0B01; { no protection supplied }
	DL_MONITOR = $0B02; { protection against passive monitoring }
	DL_MAXIMUM = $0B03; { protection against modification, replay, addition, or deletion }

type
	dl_protect_tPtr = ^dl_protect_t;
	dl_protect_t = record
		dl_min: SInt32;
		dl_max: SInt32;
	end;
{
   Resilience specification
   probabilities are scaled by a factor of 10,000 with a time interval
   of 10,000 seconds.
}
type
	dl_resilience_tPtr = ^dl_resilience_t;
	dl_resilience_t = record
		dl_disc_prob: SInt32;           { probability of provider init DISC }
		dl_reset_prob: SInt32;          { probability of provider init RESET }
	end;
{
    QOS type definition to be used for negotiation with the
    remote end of a connection, or a connectionless unitdata request.
    There are two type definitions to handle the negotiation 
    process at connection establishment. The typedef dl_qos_range_t
    is used to present a range for parameters. This is used
    in the DL_CONNECT_REQ and DL_CONNECT_IND messages. The typedef
    dl_qos_sel_t is used to select a specific value for the QOS
    parameters. This is used in the DL_CONNECT_RES, DL_CONNECT_CON,
    and DL_INFO_ACK messages to define the selected QOS parameters
    for a connection.

    NOTE
    A DataLink provider which has unknown values for any of the fields
    will use a value of DL_UNKNOWN for all values in the fields.

    NOTE
    A QOS parameter value of DL_QOS_DONT_CARE informs the DLS
    provider the user requesting this value doesn't care 
    what the QOS parameter is set to. This value becomes the
    least possible value in the range of QOS parameters.
    The order of the QOS parameter range is then:

        DL_QOS_DONT_CARE < 0 < MAXIMUM QOS VALUE
}
const
	DL_UNKNOWN = -1;
	DL_QOS_DONT_CARE = -2;

{
    Every QOS structure has the first 4 bytes containing a type
    field, denoting the definition of the rest of the structure.
    This is used in the same manner has the dl_primitive variable
    is in messages.

    The following list is the defined QOS structure type values and structures.
}
const
	DL_QOS_CO_RANGE1 = $0101; { QOS range struct. for Connection modeservice }
	DL_QOS_CO_SEL1 = $0102; { QOS selection structure }
	DL_QOS_CL_RANGE1 = $0103; { QOS range struct. for connectionless}
	DL_QOS_CL_SEL1 = $0104; { QOS selection for connectionless mode}

type
	dl_qos_co_range1_tPtr = ^dl_qos_co_range1_t;
	dl_qos_co_range1_t = record
		dl_qos_type: UInt32;
		dl_rcv_throughput: dl_through_t;      { desired and acceptable}
		dl_rcv_trans_delay: dl_transdelay_t;     { desired and acceptable}
		dl_xmt_throughput: dl_through_t;
		dl_xmt_trans_delay: dl_transdelay_t;
		dl_priority: dl_priority_t;            { min and max values }
		dl_protection: dl_protect_t;          { min and max values }
		dl_residual_error: SInt32;
		dl_resilience: dl_resilience_t;
	end;
type
	dl_qos_co_sel1_tPtr = ^dl_qos_co_sel1_t;
	dl_qos_co_sel1_t = record
		dl_qos_type: UInt32;
		dl_rcv_throughput: SInt32;
		dl_rcv_trans_delay: SInt32;
		dl_xmt_throughput: SInt32;
		dl_xmt_trans_delay: SInt32;
		dl_priority: SInt32;
		dl_protection: SInt32;
		dl_residual_error: SInt32;
		dl_resilience: dl_resilience_t;
	end;
type
	dl_qos_cl_range1_tPtr = ^dl_qos_cl_range1_t;
	dl_qos_cl_range1_t = record
		dl_qos_type: UInt32;
		dl_trans_delay: dl_transdelay_t;
		dl_priority: dl_priority_t;
		dl_protection: dl_protect_t;
		dl_residual_error: SInt32;
	end;
type
	dl_qos_cl_sel1_tPtr = ^dl_qos_cl_sel1_t;
	dl_qos_cl_sel1_t = record
		dl_qos_type: UInt32;
		dl_trans_delay: SInt32;
		dl_priority: SInt32;
		dl_protection: SInt32;
		dl_residual_error: SInt32;
	end;
{
    DLPI interface primitive definitions.

    Each primitive is sent as a stream message. It is possible that
    the messages may be viewed as a sequence of bytes that have the
    following form without any padding. The structure definition
    of the following messages may have to change depending on the
    underlying hardware architecture and crossing of a hardware
    boundary with a different hardware architecture.

    Fields in the primitives having a name of the form
    dl_reserved cannot be used and have the value of
    binary zero, no bits turned on.

    Each message has the name defined followed by the
    stream message type (M_PROTO, M_PCPROTO, M_DATA)
 }
{ LOCAL MANAGEMENT SERVICE PRIMITIVES}

{ DL_INFO_REQ, M_PCPROTO type}

type
	dl_info_req_tPtr = ^dl_info_req_t;
	dl_info_req_t = record
		dl_primitive: UInt32;           { set to DL_INFO_REQ }
	end;
{ DL_INFO_ACK, M_PCPROTO type}
type
	dl_info_ack_tPtr = ^dl_info_ack_t;
	dl_info_ack_t = record
		dl_primitive: UInt32;           { set to DL_INFO_ACK }
		dl_max_sdu: UInt32;             { Max bytes in a DLSDU }
		dl_min_sdu: UInt32;             { Min bytes in a DLSDU }
		dl_addr_length: UInt32;         { length of DLSAP address }
		dl_mac_type: UInt32;            { type of medium supported}
		dl_reserved: UInt32;            { value set to zero }
		dl_current_state: UInt32;       { state of DLPI interface }
		dl_sap_length: SInt32;          { current length of SAP part of dlsap address }
		dl_service_mode: UInt32;        { CO, CL or ACL }
		dl_qos_length: UInt32;          { length of qos values }
		dl_qos_offset: UInt32;          { offset from beg. of block}
		dl_qos_range_length: UInt32;    { available range of qos }
		dl_qos_range_offset: UInt32;    { offset from beg. of block}
		dl_provider_style: UInt32;      { style1 or style2 }
		dl_addr_offset: UInt32;         { offset of the dlsap addr }
		dl_version: UInt32;             { version number }
		dl_brdcst_addr_length: UInt32;  { length of broadcast addr }
		dl_brdcst_addr_offset: UInt32;  { offset from beg. of block}
		dl_growth: UInt32;              { set to zero }
	end;
{ DL_ATTACH_REQ, M_PROTO type}
type
	dl_attach_req_tPtr = ^dl_attach_req_t;
	dl_attach_req_t = record
		dl_primitive: UInt32;           { set to DL_ATTACH_REQ}
		dl_ppa: UInt32;                 { id of the PPA }
	end;
{ DL_DETACH_REQ, M_PROTO type}
type
	dl_detach_req_tPtr = ^dl_detach_req_t;
	dl_detach_req_t = record
		dl_primitive: UInt32;           { set to DL_DETACH_REQ }
	end;
{ DL_BIND_REQ, M_PROTO type}
type
	dl_bind_req_tPtr = ^dl_bind_req_t;
	dl_bind_req_t = record
		dl_primitive: UInt32;           { set to DL_BIND_REQ }
		dl_sap: UInt32;                 { info to identify dlsap addr}
		dl_max_conind: UInt32;          { max # of outstanding con_ind}
		dl_service_mode: UInt16;        { CO, CL or ACL }
		dl_conn_mgmt: UInt16;           { if non-zero, is con-mgmt stream}
		dl_xidtest_flg: UInt32;         { if set to 1 indicates automatic initiation of test and xid frames }
	end;
{ DL_BIND_ACK, M_PCPROTO type}
type
	dl_bind_ack_tPtr = ^dl_bind_ack_t;
	dl_bind_ack_t = record
		dl_primitive: UInt32;           { DL_BIND_ACK }
		dl_sap: UInt32;                 { DLSAP addr info }
		dl_addr_length: UInt32;         { length of complete DLSAP addr }
		dl_addr_offset: UInt32;         { offset from beginning of M_PCPROTO}
		dl_max_conind: UInt32;          { allowed max. # of con-ind }
		dl_xidtest_flg: UInt32;         { responses supported by provider}
	end;
{ DL_SUBS_BIND_REQ, M_PROTO type}
type
	dl_subs_bind_req_tPtr = ^dl_subs_bind_req_t;
	dl_subs_bind_req_t = record
		dl_primitive: UInt32;           { DL_SUBS_BIND_REQ }
		dl_subs_sap_offset: UInt32;     { offset of subs_sap }
		dl_subs_sap_length: UInt32;     { length of subs_sap }
		dl_subs_bind_class: UInt32;     { peer or hierarchical }
	end;
{ DL_SUBS_BIND_ACK, M_PCPROTO type}
type
	dl_subs_bind_ack_tPtr = ^dl_subs_bind_ack_t;
	dl_subs_bind_ack_t = record
		dl_primitive: UInt32;           { DL_SUBS_BIND_ACK }
		dl_subs_sap_offset: UInt32;     { offset of subs_sap }
		dl_subs_sap_length: UInt32;     { length of subs_sap }
	end;
{ DL_UNBIND_REQ, M_PROTO type}
type
	dl_unbind_req_tPtr = ^dl_unbind_req_t;
	dl_unbind_req_t = record
		dl_primitive: UInt32;           { DL_UNBIND_REQ }
	end;
{ DL_SUBS_UNBIND_REQ, M_PROTO type}
type
	dl_subs_unbind_req_tPtr = ^dl_subs_unbind_req_t;
	dl_subs_unbind_req_t = record
		dl_primitive: UInt32;           { DL_SUBS_UNBIND_REQ }
		dl_subs_sap_offset: UInt32;     { offset of subs_sap }
		dl_subs_sap_length: UInt32;     { length of subs_sap }
	end;
{ DL_OK_ACK, M_PCPROTO type}
type
	dl_ok_ack_tPtr = ^dl_ok_ack_t;
	dl_ok_ack_t = record
		dl_primitive: UInt32;           { DL_OK_ACK }
		dl_correct_primitive: UInt32;   { primitive being acknowledged }
	end;
{ DL_ERROR_ACK, M_PCPROTO type}
type
	dl_error_ack_tPtr = ^dl_error_ack_t;
	dl_error_ack_t = record
		dl_primitive: UInt32;           { DL_ERROR_ACK }
		dl_error_primitive: UInt32;     { primitive in error }
		dl_errno: UInt32;               { DLPI error code }
		dl_unix_errno: UInt32;          { UNIX system error code }
	end;
{ DL_ENABMULTI_REQ, M_PROTO type}
type
	dl_enabmulti_req_tPtr = ^dl_enabmulti_req_t;
	dl_enabmulti_req_t = record
		dl_primitive: UInt32;           { DL_ENABMULTI_REQ }
		dl_addr_length: UInt32;         { length of multicast address }
		dl_addr_offset: UInt32;         { offset from beg. of M_PROTO block}
	end;
{ DL_DISABMULTI_REQ, M_PROTO type}
type
	dl_disabmulti_req_tPtr = ^dl_disabmulti_req_t;
	dl_disabmulti_req_t = record
		dl_primitive: UInt32;           { DL_DISABMULTI_REQ }
		dl_addr_length: UInt32;         { length of multicast address }
		dl_addr_offset: UInt32;         { offset from beg. of M_PROTO block}
	end;
{ DL_PROMISCON_REQ, M_PROTO type}
type
	dl_promiscon_req_tPtr = ^dl_promiscon_req_t;
	dl_promiscon_req_t = record
		dl_primitive: UInt32;           { DL_PROMISCON_REQ }
		dl_level: UInt32;               { physical,SAP level or ALL multicast}
	end;
{ DL_PROMISCOFF_REQ, M_PROTO type}
type
	dl_promiscoff_req_tPtr = ^dl_promiscoff_req_t;
	dl_promiscoff_req_t = record
		dl_primitive: UInt32;           { DL_PROMISCOFF_REQ }
		dl_level: UInt32;               { Physical,SAP level or ALL multicast}
	end;
{ Primitives to get and set the Physical address}
{ DL_PHYS_ADDR_REQ, M_PROTO type}

type
	dl_phys_addr_req_tPtr = ^dl_phys_addr_req_t;
	dl_phys_addr_req_t = record
		dl_primitive: UInt32;           { DL_PHYS_ADDR_REQ }
		dl_addr_type: UInt32;           { factory or current physical addr }
	end;
{ DL_PHYS_ADDR_ACK, M_PCPROTO type}
type
	dl_phys_addr_ack_tPtr = ^dl_phys_addr_ack_t;
	dl_phys_addr_ack_t = record
		dl_primitive: UInt32;           { DL_PHYS_ADDR_ACK }
		dl_addr_length: UInt32;         { length of the physical addr }
		dl_addr_offset: UInt32;         { offset from beg. of block }
	end;
{ DL_SET_PHYS_ADDR_REQ, M_PROTO type}
type
	dl_set_phys_addr_req_tPtr = ^dl_set_phys_addr_req_t;
	dl_set_phys_addr_req_t = record
		dl_primitive: UInt32;           { DL_SET_PHYS_ADDR_REQ }
		dl_addr_length: UInt32;         { length of physical addr }
		dl_addr_offset: UInt32;         { offset from beg. of block }
	end;
{ Primitives to get statistics}
{ DL_GET_STATISTICS_REQ, M_PROTO type}

type
	dl_get_statistics_req_tPtr = ^dl_get_statistics_req_t;
	dl_get_statistics_req_t = record
		dl_primitive: UInt32;           { DL_GET_STATISTICS_REQ }
	end;
{ DL_GET_STATISTICS_ACK, M_PCPROTO type}
type
	dl_get_statistics_ack_tPtr = ^dl_get_statistics_ack_t;
	dl_get_statistics_ack_t = record
		dl_primitive: UInt32;           { DL_GET_STATISTICS_ACK }
		dl_stat_length: UInt32;         { length of statistics structure}
		dl_stat_offset: UInt32;         { offset from beg. of block }
	end;
{ CONNECTION-ORIENTED SERVICE PRIMITIVES}

{ DL_CONNECT_REQ, M_PROTO type}

type
	dl_connect_req_tPtr = ^dl_connect_req_t;
	dl_connect_req_t = record
		dl_primitive: UInt32;           { DL_CONNECT_REQ }
		dl_dest_addr_length: UInt32;    { len. of dlsap addr}
		dl_dest_addr_offset: UInt32;    { offset }
		dl_qos_length: UInt32;          { len. of QOS parm val}
		dl_qos_offset: UInt32;          { offset }
		dl_growth: UInt32;              { set to zero }
	end;
{ DL_CONNECT_IND, M_PROTO type}
type
	dl_connect_ind_tPtr = ^dl_connect_ind_t;
	dl_connect_ind_t = record
		dl_primitive: UInt32;           { DL_CONNECT_IND }
		dl_correlation: UInt32;         { provider's correlation token}
		dl_called_addr_length: UInt32;  { length of called address }
		dl_called_addr_offset: UInt32;  { offset from beginning of block }
		dl_calling_addr_length: UInt32; { length of calling address }
		dl_calling_addr_offset: UInt32; { offset from beginning of block }
		dl_qos_length: UInt32;          { length of qos structure }
		dl_qos_offset: UInt32;          { offset from beginning of block }
		dl_growth: UInt32;              { set to zero }
	end;
{ DL_CONNECT_RES, M_PROTO type}
type
	dl_connect_res_tPtr = ^dl_connect_res_t;
	dl_connect_res_t = record
		dl_primitive: UInt32;           { DL_CONNECT_RES }
		dl_correlation: UInt32;         { provider's correlation token }
		dl_resp_token: UInt32;          { token associated with responding stream }
		dl_qos_length: UInt32;          { length of qos structure }
		dl_qos_offset: UInt32;          { offset from beginning of block }
		dl_growth: UInt32;              { set to zero }
	end;
{ DL_CONNECT_CON, M_PROTO type}
type
	dl_connect_con_tPtr = ^dl_connect_con_t;
	dl_connect_con_t = record
		dl_primitive: UInt32;           { DL_CONNECT_CON}
		dl_resp_addr_length: UInt32;    { length of responder's address }
		dl_resp_addr_offset: UInt32;    { offset from beginning of block}
		dl_qos_length: UInt32;          { length of qos structure }
		dl_qos_offset: UInt32;          { offset from beginning of block}
		dl_growth: UInt32;              { set to zero }
	end;
{ DL_TOKEN_REQ, M_PCPROTO type}
type
	dl_token_req_tPtr = ^dl_token_req_t;
	dl_token_req_t = record
		dl_primitive: UInt32;           { DL_TOKEN_REQ }
	end;
{ DL_TOKEN_ACK, M_PCPROTO type}
type
	dl_token_ack_tPtr = ^dl_token_ack_t;
	dl_token_ack_t = record
		dl_primitive: UInt32;           { DL_TOKEN_ACK }
		dl_token: UInt32;               { Connection response token associated with the stream }
	end;
{ DL_DISCONNECT_REQ, M_PROTO type}
type
	dl_disconnect_req_tPtr = ^dl_disconnect_req_t;
	dl_disconnect_req_t = record
		dl_primitive: UInt32;           { DL_DISCONNECT_REQ }
		dl_reason: UInt32;              {normal, abnormal, perm. or transient}
		dl_correlation: UInt32;         { association with connect_ind }
	end;
{ DL_DISCONNECT_IND, M_PROTO type}
type
	dl_disconnect_ind_tPtr = ^dl_disconnect_ind_t;
	dl_disconnect_ind_t = record
		dl_primitive: UInt32;           { DL_DISCONNECT_IND }
		dl_originator: UInt32;          { USER or PROVIDER }
		dl_reason: UInt32;              { permanent or transient }
		dl_correlation: UInt32;         { association with connect_ind }
	end;
{ DL_RESET_REQ, M_PROTO type}
type
	dl_reset_req_tPtr = ^dl_reset_req_t;
	dl_reset_req_t = record
		dl_primitive: UInt32;           { DL_RESET_REQ }
	end;
{ DL_RESET_IND, M_PROTO type}
type
	dl_reset_ind_tPtr = ^dl_reset_ind_t;
	dl_reset_ind_t = record
		dl_primitive: UInt32;           { DL_RESET_IND }
		dl_originator: UInt32;          { Provider or User }
		dl_reason: UInt32;              { flow control, link error or resynch}
	end;
{ DL_RESET_RES, M_PROTO type}
type
	dl_reset_res_tPtr = ^dl_reset_res_t;
	dl_reset_res_t = record
		dl_primitive: UInt32;           { DL_RESET_RES }
	end;
{ DL_RESET_CON, M_PROTO type}
type
	dl_reset_con_tPtr = ^dl_reset_con_t;
	dl_reset_con_t = record
		dl_primitive: UInt32;           { DL_RESET_CON }
	end;
{ CONNECTIONLESS SERVICE PRIMITIVES}
{ DL_UNITDATA_REQ, M_PROTO type, with M_DATA block(s)}

type
	dl_unitdata_req_tPtr = ^dl_unitdata_req_t;
	dl_unitdata_req_t = record
		dl_primitive: UInt32;           { DL_UNITDATA_REQ }
		dl_dest_addr_length: UInt32;    { DLSAP length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
		dl_priority: dl_priority_t;            { priority value }
	end;
{ DL_UNITDATA_IND, M_PROTO type, with M_DATA block(s)}
type
	dl_unitdata_ind_tPtr = ^dl_unitdata_ind_t;
	dl_unitdata_ind_t = record
		dl_primitive: UInt32;           { DL_UNITDATA_IND }
		dl_dest_addr_length: UInt32;    { DLSAP length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
		dl_src_addr_length: UInt32;     { DLSAP addr length of sending user}
		dl_src_addr_offset: UInt32;     { offset from beg. of block }
		dl_group_address: UInt32;       { set to one if multicast/broadcast}
	end;
{
   DL_UDERROR_IND, M_PROTO type
   (or M_PCPROTO type if LLI-based provider)
}
type
	dl_uderror_ind_tPtr = ^dl_uderror_ind_t;
	dl_uderror_ind_t = record
		dl_primitive: UInt32;           { DL_UDERROR_IND }
		dl_dest_addr_length: UInt32;    { Destination DLSAP }
		dl_dest_addr_offset: UInt32;    { Offset from beg. of block }
		dl_unix_errno: UInt32;          { unix system error code}
		dl_errno: UInt32;               { DLPI error code }
	end;
{ DL_UDQOS_REQ, M_PROTO type}
type
	dl_udqos_req_tPtr = ^dl_udqos_req_t;
	dl_udqos_req_t = record
		dl_primitive: UInt32;           { DL_UDQOS_REQ }
		dl_qos_length: UInt32;          { length in bytes of requested qos}
		dl_qos_offset: UInt32;          { offset from beg. of block }
	end;
{ Primitives to handle XID and TEST operations}
{ DL_TEST_REQ, M_PROTO type}

type
	dl_test_req_tPtr = ^dl_test_req_t;
	dl_test_req_t = record
		dl_primitive: UInt32;           { DL_TEST_REQ }
		dl_flag: UInt32;                { poll/final }
		dl_dest_addr_length: UInt32;    { DLSAP length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
	end;
{ DL_TEST_IND, M_PROTO type}
type
	dl_test_ind_tPtr = ^dl_test_ind_t;
	dl_test_ind_t = record
		dl_primitive: UInt32;           { DL_TEST_IND }
		dl_flag: UInt32;                { poll/final }
		dl_dest_addr_length: UInt32;    { dlsap length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
		dl_src_addr_length: UInt32;     { dlsap length of source user }
		dl_src_addr_offset: UInt32;     { offset from beg. of block }
	end;
{ DL_TEST_RES, M_PROTO type}
type
	dl_test_res_tPtr = ^dl_test_res_t;
	dl_test_res_t = record
		dl_primitive: UInt32;           { DL_TEST_RES }
		dl_flag: UInt32;                { poll/final }
		dl_dest_addr_length: UInt32;    { DLSAP length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
	end;
{ DL_TEST_CON, M_PROTO type}
type
	dl_test_con_tPtr = ^dl_test_con_t;
	dl_test_con_t = record
		dl_primitive: UInt32;           { DL_TEST_CON }
		dl_flag: UInt32;                { poll/final }
		dl_dest_addr_length: UInt32;    { dlsap length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
		dl_src_addr_length: UInt32;     { dlsap length of source user }
		dl_src_addr_offset: UInt32;     { offset from beg. of block }
	end;
{ DL_XID_REQ, M_PROTO type}
type
	dl_xid_req_tPtr = ^dl_xid_req_t;
	dl_xid_req_t = record
		dl_primitive: UInt32;           { DL_XID_REQ }
		dl_flag: UInt32;                { poll/final }
		dl_dest_addr_length: UInt32;    { dlsap length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
	end;
{ DL_XID_IND, M_PROTO type}
type
	dl_xid_ind_tPtr = ^dl_xid_ind_t;
	dl_xid_ind_t = record
		dl_primitive: UInt32;           { DL_XID_IND }
		dl_flag: UInt32;                { poll/final }
		dl_dest_addr_length: UInt32;    { dlsap length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
		dl_src_addr_length: UInt32;     { dlsap length of source user }
		dl_src_addr_offset: UInt32;     { offset from beg. of block }
	end;
{ DL_XID_RES, M_PROTO type}
type
	dl_xid_res_tPtr = ^dl_xid_res_t;
	dl_xid_res_t = record
		dl_primitive: UInt32;           { DL_XID_RES }
		dl_flag: UInt32;                { poll/final }
		dl_dest_addr_length: UInt32;    { DLSAP length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
	end;
{ DL_XID_CON, M_PROTO type}
type
	dl_xid_con_tPtr = ^dl_xid_con_t;
	dl_xid_con_t = record
		dl_primitive: UInt32;           { DL_XID_CON }
		dl_flag: UInt32;                { poll/final }
		dl_dest_addr_length: UInt32;    { dlsap length of dest. user }
		dl_dest_addr_offset: UInt32;    { offset from beg. of block }
		dl_src_addr_length: UInt32;     { dlsap length of source user }
		dl_src_addr_offset: UInt32;     { offset from beg. of block }
	end;
{ ACKNOWLEDGED CONNECTIONLESS SERVICE PRIMITIVES}

{ DL_DATA_ACK_REQ, M_PROTO type}

type
	dl_data_ack_req_tPtr = ^dl_data_ack_req_t;
	dl_data_ack_req_t = record
		dl_primitive: UInt32;           { DL_DATA_ACK_REQ }
		dl_correlation: UInt32;         { User's correlation token }
		dl_dest_addr_length: UInt32;    { length of destination addr }
		dl_dest_addr_offset: UInt32;    { offset from beginning of block }
		dl_src_addr_length: UInt32;     { length of source address }
		dl_src_addr_offset: UInt32;     { offset from beginning of block }
		dl_priority: UInt32;            { priority }
		dl_service_class: UInt32;       { DL_RQST_RSP or DL_RQST_NORSP }
	end;
{ DL_DATA_ACK_IND, M_PROTO type}
type
	dl_data_ack_ind_tPtr = ^dl_data_ack_ind_t;
	dl_data_ack_ind_t = record
		dl_primitive: UInt32;           { DL_DATA_ACK_IND }
		dl_dest_addr_length: UInt32;    { length of destination addr }
		dl_dest_addr_offset: UInt32;    { offset from beginning of block }
		dl_src_addr_length: UInt32;     { length of source address }
		dl_src_addr_offset: UInt32;     { offset from beginning of block }
		dl_priority: UInt32;            { priority for data unit transm. }
		dl_service_class: UInt32;       { DL_RQST_RSP or DL_RQST_NORSP }
	end;
{ DL_DATA_ACK_STATUS_IND, M_PROTO type}
type
	dl_data_ack_status_ind_tPtr = ^dl_data_ack_status_ind_t;
	dl_data_ack_status_ind_t = record
		dl_primitive: UInt32;           { DL_DATA_ACK_STATUS_IND }
		dl_correlation: UInt32;         { User's correlation token }
		dl_status: UInt32;              { success or failure of previous req}
	end;
{ DL_REPLY_REQ, M_PROTO type}
type
	dl_reply_req_tPtr = ^dl_reply_req_t;
	dl_reply_req_t = record
		dl_primitive: UInt32;           { DL_REPLY_REQ }
		dl_correlation: UInt32;         { User's correlation token }
		dl_dest_addr_length: UInt32;    { length of destination address }
		dl_dest_addr_offset: UInt32;    { offset from beginning of block }
		dl_src_addr_length: UInt32;     { source address length }
		dl_src_addr_offset: UInt32;     { offset from beginning of block }
		dl_priority: UInt32;            { priority for data unit transmission}
		dl_service_class: UInt32;
	end;
{ DL_REPLY_IND, M_PROTO type}
type
	dl_reply_ind_tPtr = ^dl_reply_ind_t;
	dl_reply_ind_t = record
		dl_primitive: UInt32;           { DL_REPLY_IND }
		dl_dest_addr_length: UInt32;    { length of destination address }
		dl_dest_addr_offset: UInt32;    { offset from beginning of block}
		dl_src_addr_length: UInt32;     { length of source address }
		dl_src_addr_offset: UInt32;     { offset from beginning of block }
		dl_priority: UInt32;            { priority for data unit transmission}
		dl_service_class: UInt32;       { DL_RQST_RSP or DL_RQST_NORSP }
	end;
{ DL_REPLY_STATUS_IND, M_PROTO type}
type
	dl_reply_status_ind_tPtr = ^dl_reply_status_ind_t;
	dl_reply_status_ind_t = record
		dl_primitive: UInt32;           { DL_REPLY_STATUS_IND }
		dl_correlation: UInt32;         { User's correlation token }
		dl_status: UInt32;              { success or failure of previous req}
	end;
{ DL_REPLY_UPDATE_REQ, M_PROTO type}
type
	dl_reply_update_req_tPtr = ^dl_reply_update_req_t;
	dl_reply_update_req_t = record
		dl_primitive: UInt32;           { DL_REPLY_UPDATE_REQ }
		dl_correlation: UInt32;         { user's correlation token }
		dl_src_addr_length: UInt32;     { length of source address }
		dl_src_addr_offset: UInt32;     { offset from beginning of block }
	end;
{ DL_REPLY_UPDATE_STATUS_IND, M_PROTO type}
type
	dl_reply_update_status_ind_tPtr = ^dl_reply_update_status_ind_t;
	dl_reply_update_status_ind_t = record
		dl_primitive: UInt32;           { DL_REPLY_UPDATE_STATUS_IND }
		dl_correlation: UInt32;         { User's correlation token }
		dl_status: UInt32;              { success or failure of previous req}
	end;
	DL_primitivesPtr = ^DL_primitives;
	DL_primitives = record
		case SInt16 of
		0: (
			dl_primitive:		UInt32;
			);
		1: (
			info_req:			dl_info_req_t;
			);
		2: (
			info_ack:			dl_info_ack_t;
			);
		3: (
			attach_req:			dl_attach_req_t;
			);
		4: (
			detach_req:			dl_detach_req_t;
			);
		5: (
			bind_req:			dl_bind_req_t;
			);
		6: (
			bind_ack:			dl_bind_ack_t;
			);
		7: (
			unbind_req:			dl_unbind_req_t;
			);
		8: (
			subs_bind_req:		dl_subs_bind_req_t;
			);
		9: (
			subs_bind_ack:		dl_subs_bind_ack_t;
			);
		10: (
			subs_unbind_req:	dl_subs_unbind_req_t;
			);
		11: (
			ok_ack:				dl_ok_ack_t;
			);
		12: (
			error_ack:			dl_error_ack_t;
			);
		13: (
			connect_req:		dl_connect_req_t;
			);
		14: (
			connect_ind:		dl_connect_ind_t;
			);
		15: (
			connect_res:		dl_connect_res_t;
			);
		16: (
			connect_con:		dl_connect_con_t;
			);
		17: (
			token_req:			dl_token_req_t;
			);
		18: (
			token_ack:			dl_token_ack_t;
			);
		19: (
			disconnect_req:		dl_disconnect_req_t;
			);
		20: (
			disconnect_ind:		dl_disconnect_ind_t;
			);
		21: (
			reset_req:			dl_reset_req_t;
			);
		22: (
			reset_ind:			dl_reset_ind_t;
			);
		23: (
			reset_res:			dl_reset_res_t;
			);
		24: (
			reset_con:			dl_reset_con_t;
			);
		25: (
			unitdata_req:		dl_unitdata_req_t;
			);
		26: (
			unitdata_ind:		dl_unitdata_ind_t;
			);
		27: (
			uderror_ind:		dl_uderror_ind_t;
			);
		28: (
			udqos_req:			dl_udqos_req_t;
			);
		29: (
			enabmulti_req:		dl_enabmulti_req_t;
			);
		30: (
			disabmulti_req:		dl_disabmulti_req_t;
			);
		31: (
			promiscon_req:		dl_promiscon_req_t;
			);
		32: (
			promiscoff_req:		dl_promiscoff_req_t;
			);
		33: (
			physaddr_req:		dl_phys_addr_req_t;
			);
		34: (
			physaddr_ack:		dl_phys_addr_ack_t;
			);
		35: (
			set_physaddr_req:	dl_set_phys_addr_req_t;
			);
		36: (
			get_statistics_req:	dl_get_statistics_req_t;
			);
		37: (
			get_statistics_ack:	dl_get_statistics_ack_t;
			);
		38: (
			test_req:			dl_test_req_t;
			);
		39: (
			test_ind:			dl_test_ind_t;
			);
		40: (
			test_res:			dl_test_res_t;
			);
		41: (
			test_con:			dl_test_con_t;
			);
		42: (
			xid_req:			dl_xid_req_t;
			);
		43: (
			xid_ind:			dl_xid_ind_t;
			);
		44: (
			xid_res:			dl_xid_res_t;
			);
		45: (
			xid_con:			dl_xid_con_t;
			);
		46: (
			data_ack_req:		dl_data_ack_req_t;
			);
		47: (
			data_ack_ind:		dl_data_ack_ind_t;
			);
		48: (
			data_ack_status_ind: dl_data_ack_status_ind_t;
			);
		49: (
			reply_req:			dl_reply_req_t;
			);
		50: (
			reply_ind:			dl_reply_ind_t;
			);
		51: (
			reply_status_ind:	dl_reply_status_ind_t;
			);
		52: (
			reply_update_req:	dl_reply_update_req_t;
			);
		53: (
			reply_update_status_ind: dl_reply_update_status_ind_t;
			);
	end;
const
	DL_INFO_REQ_SIZE = SizeOf(dl_info_req_t);
	DL_INFO_ACK_SIZE = SizeOf(dl_info_ack_t);
	DL_ATTACH_REQ_SIZE = SizeOf(dl_attach_req_t);
	DL_DETACH_REQ_SIZE = SizeOf(dl_detach_req_t);
	DL_BIND_REQ_SIZE = SizeOf(dl_bind_req_t);
	DL_BIND_ACK_SIZE = SizeOf(dl_bind_ack_t);
	DL_UNBIND_REQ_SIZE = SizeOf(dl_unbind_req_t);
	DL_SUBS_BIND_REQ_SIZE = SizeOf(dl_subs_bind_req_t);
	DL_SUBS_BIND_ACK_SIZE = SizeOf(dl_subs_bind_ack_t);
	DL_SUBS_UNBIND_REQ_SIZE = SizeOf(dl_subs_unbind_req_t);
	DL_OK_ACK_SIZE = SizeOf(dl_ok_ack_t);
	DL_ERROR_ACK_SIZE = SizeOf(dl_error_ack_t);
	DL_CONNECT_REQ_SIZE = SizeOf(dl_connect_req_t);
	DL_CONNECT_IND_SIZE = SizeOf(dl_connect_ind_t);
	DL_CONNECT_RES_SIZE = SizeOf(dl_connect_res_t);
	DL_CONNECT_CON_SIZE = SizeOf(dl_connect_con_t);
	DL_TOKEN_REQ_SIZE = SizeOf(dl_token_req_t);
	DL_TOKEN_ACK_SIZE = SizeOf(dl_token_ack_t);
	DL_DISCONNECT_REQ_SIZE = SizeOf(dl_disconnect_req_t);
	DL_DISCONNECT_IND_SIZE = SizeOf(dl_disconnect_ind_t);
	DL_RESET_REQ_SIZE = SizeOf(dl_reset_req_t);
	DL_RESET_IND_SIZE = SizeOf(dl_reset_ind_t);
	DL_RESET_RES_SIZE = SizeOf(dl_reset_res_t);
	DL_RESET_CON_SIZE = SizeOf(dl_reset_con_t);
	DL_UNITDATA_REQ_SIZE = SizeOf(dl_unitdata_req_t);
	DL_UNITDATA_IND_SIZE = SizeOf(dl_unitdata_ind_t);
	DL_UDERROR_IND_SIZE = SizeOf(dl_uderror_ind_t);
	DL_UDQOS_REQ_SIZE = SizeOf(dl_udqos_req_t);
	DL_ENABMULTI_REQ_SIZE = SizeOf(dl_enabmulti_req_t);
	DL_DISABMULTI_REQ_SIZE = SizeOf(dl_disabmulti_req_t);
	DL_PROMISCON_REQ_SIZE = SizeOf(dl_promiscon_req_t);
	DL_PROMISCOFF_REQ_SIZE = SizeOf(dl_promiscoff_req_t);
	DL_PHYS_ADDR_REQ_SIZE = SizeOf(dl_phys_addr_req_t);
	DL_PHYS_ADDR_ACK_SIZE = SizeOf(dl_phys_addr_ack_t);
	DL_SET_PHYS_ADDR_REQ_SIZE = SizeOf(dl_set_phys_addr_req_t);
	DL_GET_STATISTICS_REQ_SIZE = SizeOf(dl_get_statistics_req_t);
	DL_GET_STATISTICS_ACK_SIZE = SizeOf(dl_get_statistics_ack_t);
	DL_XID_REQ_SIZE = SizeOf(dl_xid_req_t);
	DL_XID_IND_SIZE = SizeOf(dl_xid_ind_t);
	DL_XID_RES_SIZE = SizeOf(dl_xid_res_t);
	DL_XID_CON_SIZE = SizeOf(dl_xid_con_t);
	DL_TEST_REQ_SIZE = SizeOf(dl_test_req_t);
	DL_TEST_IND_SIZE = SizeOf(dl_test_ind_t);
	DL_TEST_RES_SIZE = SizeOf(dl_test_res_t);
	DL_TEST_CON_SIZE = SizeOf(dl_test_con_t);
	DL_DATA_ACK_REQ_SIZE = SizeOf(dl_data_ack_req_t);
	DL_DATA_ACK_IND_SIZE = SizeOf(dl_data_ack_ind_t);
	DL_DATA_ACK_STATUS_IND_SIZE = SizeOf(dl_data_ack_status_ind_t);
	DL_REPLY_REQ_SIZE = SizeOf(dl_reply_req_t);
	DL_REPLY_IND_SIZE = SizeOf(dl_reply_ind_t);
	DL_REPLY_STATUS_IND_SIZE = SizeOf(dl_reply_status_ind_t);
	DL_REPLY_UPDATE_REQ_SIZE = SizeOf(dl_reply_update_req_t);
	DL_REPLY_UPDATE_STATUS_IND_SIZE = SizeOf(dl_reply_update_status_ind_t);

const
	DL_IOC_HDR_INFO = $6C0A;						{  Fast path request  }

{ ***** From the Mentat "modnames.h" *****}


const
	MI_AFU_NAME = 'afu';
const
	MI_AHARP_NAME = 'ahar';
const
	MI_AHENET_NAME = 'ahen';
const
	MI_ARP_NAME = 'arp';
const
	MI_ARPM_NAME = 'arpm';
const
	MI_COURMUX_NAME = 'courmux';
const
	MI_CLONE_NAME = 'clone';
const
	MI_DLB_NAME = 'dlb';
const
	MI_DLM_NAME = 'dlm';
const
	MI_DMODD_NAME = 'disdlpi';
const
	MI_DMODT_NAME = 'distpi';
const
	MI_DN_NAME = 'dn';
const
	MI_DNF_NAME = 'dnf';
const
	MI_DRVE_NAME = 'drve';
const
	MI_ECHO_NAME = 'echo';
const
	MI_ENXR_NAME = 'enxr';
const
	MI_RAWIP_NAME = 'rawip';
const
	MI_RAWIPM_NAME = 'rawipm';
const
	MI_HAVOC_NAME = 'havoc';
const
	MI_HAVOCM_NAME = 'havocm';
const
	MI_IP_NAME = 'ip';
const
	MI_IPM_NAME = 'ipm';
const
	MI_IPX_NAME = 'ipx';
const
	MI_LOG_NAME = 'log';
const
	MI_MODE_NAME = 'mode';
const
	MI_MUX_NAME = 'mux';
const
	MI_NECHO_NAME = 'necho';
const
	MI_NPEP_NAME = 'npep';
const
	MI_NULS_NAME = 'nuls';
const
	MI_NULZ_NAME = 'nulz';
const
	MI_PASS_NAME = 'pass';
const
	MI_PIPEMOD_NAME = 'pipemod';
const
	MI_SAD_NAME = 'sad';
const
	MI_SC_NAME = 'sc';
const
	MI_SOCKMOD_NAME = 'sockmod';
const
	MI_SPASS_NAME = 'spass';
const
	MI_SPX_NAME = 'spx';
const
	MI_STH_NAME = 'mi_sth';
const
	MI_TCP_NAME = 'tcp';
const
	MI_TCPM_NAME = 'tcpm';
const
	MI_TIMOD_NAME = 'timod';
const
	MI_TIRDWR_NAME = 'tirdwr';
const
	MI_TMOD_NAME = 'tmod';
const
	MI_TMUX_NAME = 'tmux';
const
	MI_TPIT_NAME = 'tpit';
const
	MI_TRSR_NAME = 'trsr';
const
	MI_TRXR_NAME = 'trxr';
const
	MI_UDP_NAME = 'udp';
const
	MI_UDPM_NAME = 'udpm';
const
	MI_WELD_NAME = 'mi_weld';
const
	MI_XDG_NAME = 'xdg';
const
	MI_XECHO_NAME = 'xecho';
const
	MI_XF_NAME = 'xf';
const
	MI_XFIPX_NAME = 'xfipx';
const
	MI_XFXNS_NAME = 'xfxns';
const
	MI_XPE_NAME = 'xpe';
const
	MI_XS_NAME = 'xs';
const
	MI_XTINDG_NAME = 'xtindg';
const
	MI_XTINVC_NAME = 'xtinvc';
const
	MI_XTM_NAME = 'xtm';
const
	MI_XTMIP_NAME = 'xtmip';

const
	MI_AFU_DEVICE = '/dev/afu';
const
	MI_ARP_DEVICE = '/dev/arp';
const
	MI_COURMUX_DEVICE = '/dev/courmux';
const
	MI_CLONE_DEVICE = '/dev/clone';
const
	MI_DLB_DEVICE = '/dev/dlb';
const
	MI_DN_DEVICE = '/dev/dn';
const
	MI_DNF_DEVICE = '/dev/dnf';
const
	MI_DRVE_DEVICE = '/dev/drve';
const
	MI_ECHO_DEVICE = '/dev/echo';
const
	MI_RAWIP_DEVICE = '/dev/rawip';
const
	MI_HAVOC_DEVICE = '/dev/havoc';
const
	MI_IP_DEVICE = '/dev/ip';
const
	MI_IPX_DEVICE = '/dev/ipx';
const
	MI_LOG_DEVICE = '/dev/log';
const
	MI_MODE_DEVICE = '/dev/mode';
const
	MI_MUX_DEVICE = '/dev/mux';
const
	MI_NECHO_DEVICE = '/dev/necho';
const
	MI_NPEP_DEVICE = '/dev/npep';
const
	MI_NULS_DEVICE = '/dev/nuls';
const
	MI_NULZ_DEVICE = '/dev/nulz';
const
	MI_SAD_DEVICE = '/dev/sad';
const
	MI_SPX_DEVICE = '/dev/spx';
const
	MI_TCP_DEVICE = '/dev/tcp';
const
	MI_TMUX_DEVICE = '/dev/tmux';
const
	MI_TMUX0_DEVICE = '/dev/tmux#0';
const
	MI_TMUX1_DEVICE = '/dev/tmux#1';
const
	MI_TPIT_DEVICE = '/dev/tpit';
const
	MI_UDP_DEVICE = '/dev/udp';
const
	MI_XDG_DEVICE = '/dev/xdg';
const
	MI_XECHO_DEVICE = '/dev/xecho';
const
	MI_XF_DEVICE = '/dev/xf';
const
	MI_XPE_DEVICE = '/dev/xpe';
const
	MI_XS_DEVICE = '/dev/xs';
const
	MI_XTINDG_DEVICE = '/dev/xtindg';
const
	MI_XTINVC_DEVICE = '/dev/xtinvc';


{ Streamtab entries }
// #define MI_AFU_STREAMTAB    afuinfo
// #define MI_AHARP_STREAMTAB  aharinfo
// #define MI_AHENET_STREAMTAB aheninfo
// #define MI_ARP_STREAMTAB    arpinfo
// #define MI_ARPM_STREAMTAB   arpminfo
// #define MI_COURMUX_STREAMTAB    courmuxinfo
// #define MI_CLONE_STREAMTAB  cloneinfo
// #define MI_DLB_STREAMTAB    dlbinfo
// #define MI_DLM_STREAMTAB    dlminfo
// #define MI_DMODD_STREAMTAB  dmoddinfo
// #define MI_DMODT_STREAMTAB  dmodtinfo
// #define MI_DN_STREAMTAB     dninfo
// #define MI_DNF_STREAMTAB    dnfinfo
// #define MI_DRVE_STREAMTAB   drveinfo
// #define MI_ECHO_STREAMTAB   echoinfo
// #define MI_ENXR_STREAMTAB   enxrinfo
// #define MI_HAVOC_STREAMTAB  hvcinfo
// #define MI_HAVOCM_STREAMTAB hvcminfo
// #define MI_IP_STREAMTAB     ipinfo
// #define MI_IPM_STREAMTAB    ipminfo
// #define MI_IPX_STREAMTAB    ipxinfo
// #define MI_LOG_STREAMTAB    loginfo
// #define MI_MODE_STREAMTAB   modeinfo
// #define MI_MUX_STREAMTAB    muxinfo
// #define MI_NECHO_STREAMTAB  nechoinfo
// #define MI_NPEP_STREAMTAB   npepinfo
// #define MI_NULS_STREAMTAB   nulsinfo
// #define MI_NULZ_STREAMTAB   nulzinfo
// #define MI_PASS_STREAMTAB   passinfo
// #define MI_PIPEMOD_STREAMTAB    pmodinfo
// #define MI_RAWIP_STREAMTAB  rawipinfo
// #define MI_RAWIPM_STREAMTAB rawipminfo
// #define MI_SAD_STREAMTAB    sadinfo
// #define MI_SC_STREAMTAB     scinfo
// #define MI_SOCKMOD_STREAMTAB    sockmodinfo
// #define MI_SPASS_STREAMTAB  spassinfo
// #define MI_SPX_STREAMTAB    spxinfo
// #define MI_STH_STREAMTAB    mi_sthinfo
// #define MI_TCP_STREAMTAB    tcpinfo
// #define MI_TCPM_STREAMTAB   tcpminfo
// #define MI_TIMOD_STREAMTAB  timodinfo
// #define MI_TIRDWR_STREAMTAB tirdwrinfo
// #define MI_TMOD_STREAMTAB   tmodinfo
// #define MI_TMUX_STREAMTAB   tmuxinfo
// #define MI_TPIT_STREAMTAB   tpitinfo
// #define MI_TRSR_STREAMTAB   trsrinfo
// #define MI_TRXR_STREAMTAB   trxrinfo
// #define MI_UDP_STREAMTAB    udpinfo
// #define MI_UDPM_STREAMTAB   udpminfo
// #define MI_WELD_STREAMTAB   mi_weldinfo
// #define MI_XDG_STREAMTAB    xdginfo
// #define MI_XECHO_STREAMTAB  xechoinfo
// #define MI_XF_STREAMTAB     xfinfo
// #define MI_XFIPX_STREAMTAB  xfipxinfo
// #define MI_XFXNS_STREAMTAB  xfxnsinfo
// #define MI_XPE_STREAMTAB    xpeinfo
// #define MI_XS_STREAMTAB     xsinfo
// #define MI_XTINDG_STREAMTAB xtindginfo
// #define MI_XTINVC_STREAMTAB xtinvcinfo
// #define MI_XTM_STREAMTAB    xtminfo
// #define MI_XTMIP_STREAMTAB  xtmipinfo


// #define MI_AFU_DEVFLAG      afudevflag
// #define MI_AHARP_DEVFLAG    ahardevflag
// #define MI_AHENET_DEVFLAG   ahendevflag
// #define MI_ARP_DEVFLAG      arpdevflag
// #define MI_ARPM_DEVFLAG     arpmdevflag
// #define MI_COURMUX_DEVFLAG  courmuxdevflag
// #define MI_CLONE_DEVFLAG    clonedevflag
// #define MI_DLB_DEVFLAG      dlbdevflag
// #define MI_DLM_DEVFLAG      dlmdevflag
// #define MI_DMODD_DEVFLAG    dmodddevflag
// #define MI_DMODT_DEVFLAG    dmodtdevflag
// #define MI_DN_DEVFLAG       dndevflag
// #define MI_DNF_DEVFLAG      dnfdevflag
// #define MI_DRVE_DEVFLAG     drvedevflag
// #define MI_ECHO_DEVFLAG     echodevflag
// #define MI_ENXR_DEVFLAG     enxrdevflag
// #define MI_HAVOC_DEVFLAG    hvcdevflag
// #define MI_HAVOCM_DEVFLAG   hvcmdevflag
// #define MI_IP_DEVFLAG       ipdevflag
// #define MI_IPM_DEVFLAG      ipmdevflag
// #define MI_IPX_DEVFLAG      ipxdevflag
// #define MI_LOG_DEVFLAG      logdevflag
// #define MI_MODE_DEVFLAG     modedevflag
// #define MI_MUX_DEVFLAG      muxdevflag
// #define MI_NECHO_DEVFLAG    nechodevflag
// #define MI_NPEP_DEVFLAG     npepdevflag
// #define MI_NULS_DEVFLAG     nulsdevflag
// #define MI_NULZ_DEVFLAG     nulzdevflag
// #define MI_PASS_DEVFLAG     passdevflag
// #define MI_PIPEMOD_DEVFLAG  pipemoddevflag
// #define MI_RAWIP_DEVFLAG    rawipdevflag
// #define MI_RAWIPM_DEVFLAG   rawipmdevflag

// #define MI_SAD_DEVFLAG      saddevflag
// #define MI_SC_DEVFLAG       scdevflag
// #define MI_SOCKMOD_DEVFLAG  sockmoddevflag
// #define MI_SPASS_DEVFLAG    spassdevflag
// #define MI_SPX_DEVFLAG      spxdevflag
// #define MI_TCP_DEVFLAG      tcpdevflag
// #define MI_TCPM_DEVFLAG     tcpmdevflag
// #define MI_TIMOD_DEVFLAG    timoddevflag
// #define MI_TIRDWR_DEVFLAG   tirdwrdevflag
// #define MI_TMOD_DEVFLAG     tmoddevflag
// #define MI_TMUX_DEVFLAG     tmuxdevflag
// #define MI_TPIT_DEVFLAG     tpitdevflag
// #define MI_TRSR_DEVFLAG     trsrdevflag
// #define MI_TRXR_DEVFLAG     trxrdevflag
// #define MI_UDP_DEVFLAG      udpdevflag
// #define MI_UDPM_DEVFLAG     udpmdevflag
// #define MI_XDG_DEVFLAG      xdgdevflag
// #define MI_XECHO_DEVFLAG    xechodevflag
// #define MI_XF_DEVFLAG       xfdevflag
// #define MI_XFIPX_DEVFLAG    xfipxdevflag
// #define MI_XFXNS_DEVFLAG    xfxnsdevflag
// #define MI_XPE_DEVFLAG      xpedevflag
// #define MI_XS_DEVFLAG       xsdevflag
// #define MI_XTINDG_DEVFLAG   xtindgdevflag
// #define MI_XTINVC_DEVFLAG   xtinvcdevflag
// #define MI_XTM_DEVFLAG      xtmdevflag
// #define MI_XTMIP_DEVFLAG    xtmipdevflag

// #define MI_AFU_SQLVL        SQLVL_QUEUEPAIR
// #define MI_AHARP_SQLVL      SQLVL_QUEUE
// #define MI_AHENET_SQLVL     SQLVL_QUEUE
// #define MI_ARP_SQLVL        SQLVL_MODULE
// #define MI_ARPM_SQLVL       SQLVL_MODULE
// #define MI_COURMUX_SQLVL    SQLVL_MODULE
// #define MI_CLONE_SQLVL      SQLVL_MODULE
// #define MI_DLB_SQLVL        SQLVL_QUEUE
// #define MI_DLM_SQLVL        SQLVL_QUEUE
// #define MI_DMODD_SQLVL      SQLVL_QUEUE
// #define MI_DMODT_SQLVL      SQLVL_QUEUE
// #define MI_DN_SQLVL         SQLVL_QUEUE
// #define MI_DNF_SQLVL        SQLVL_QUEUE
// #define MI_DRVE_SQLVL       SQLVL_QUEUEPAIR
// #define MI_ECHO_SQLVL       SQLVL_QUEUE
// #define MI_ENXR_SQLVL       SQLVL_QUEUE
// #define MI_RAWIP_SQLVL      SQLVL_QUEUE
// #define MI_RAWIPM_SQLVL     SQLVL_QUEUE
// #define MI_HAVOC_SQLVL      SQLVL_QUEUE
// #define MI_HAVOCM_SQLVL     SQLVL_QUEUE
// #define MI_IP_SQLVL         SQLVL_QUEUEPAIR
// #define MI_IPM_SQLVL        SQLVL_QUEUEPAIR
// #define MI_IPX_SQLVL        SQLVL_QUEUE
// #define MI_LOG_SQLVL        SQLVL_MODULE
// #define MI_MODE_SQLVL       SQLVL_QUEUEPAIR
// #define MI_MUX_SQLVL        SQLVL_MODULE
// #define MI_NECHO_SQLVL      SQLVL_QUEUE

// #define MI_NPEP_SQLVL       SQLVL_QUEUE
// #define MI_NULS_SQLVL       SQLVL_QUEUE
// #define MI_NULZ_SQLVL       SQLVL_QUEUE
// #define MI_PASS_SQLVL       SQLVL_QUEUE
// #define MI_PIPEMOD_SQLVL    SQLVL_QUEUE

// #define MI_SAD_SQLVL        SQLVL_MODULE
// #define MI_SC_SQLVL         SQLVL_QUEUE
// #define MI_SOCKMOD_SQLVL    SQLVL_QUEUEPAIR
// #define MI_SPASS_SQLVL      SQLVL_QUEUE
// #define MI_SPX_SQLVL        SQLVL_QUEUE
// #define MI_TCP_SQLVL        SQLVL_QUEUEPAIR
// #define MI_TCPM_SQLVL       SQLVL_QUEUEPAIR
// #define MI_TIMOD_SQLVL      SQLVL_QUEUEPAIR
// #define MI_TIRDWR_SQLVL     SQLVL_QUEUE
// #define MI_TMOD_SQLVL       SQLVL_QUEUEPAIR
// #define MI_TMUX_SQLVL       SQLVL_MODULE
// #define MI_TPIT_SQLVL       SQLVL_MODULE
// #define MI_TRSR_SQLVL       SQLVL_MODULE
// #define MI_TRXR_SQLVL       SQLVL_QUEUE
// #define MI_UDP_SQLVL        SQLVL_QUEUE
// #define MI_UDPM_SQLVL       SQLVL_QUEUE
// #define MI_XDG_SQLVL        SQLVL_QUEUE
// #define MI_XECHO_SQLVL      SQLVL_QUEUE
// #define MI_XF_SQLVL         SQLVL_MODULE
// #define MI_XFIPX_SQLVL      SQLVL_MODULE
// #define MI_XFXNS_SQLVL      SQLVL_MODULE
// #define MI_XPE_SQLVL        SQLVL_QUEUE
// #define MI_XS_SQLVL         SQLVL_QUEUEPAIR
// #define MI_XTINDG_SQLVL     SQLVL_QUEUEPAIR
// #define MI_XTINVC_SQLVL     SQLVL_QUEUEPAIR
// #define MI_XTM_SQLVL        SQLVL_QUEUEPAIR
// #define MI_XTMIP_SQLVL      SQLVL_QUEUEPAIR
{ ***** Raw Streams *****}


{
   Flags used in the fType field of OTReadInfo for functions.
   I've removed the terse and confusing comments in this header
   file.  For a full description, read "Open Transport Advanced
   Client Programming".
}

const
	kOTNoMessagesAvailable = $FFFFFFFF;
	kOTAnyMsgType = $FFFFFFFE;
	kOTDataMsgTypes = $FFFFFFFC;
	kOTMProtoMsgTypes = $FFFFFFFB;
	kOTOnlyMProtoMsgTypes = $FFFFFFFA;

{
   OTPutCommand, OTPutData, and OTPutWriteData flags.
   These equates must not conflict with any of the other putmsg flags,
   ie MSG_ANY, MSG_BAND, MSG_HIPRI, or RS_HIPRI.
   еее These should probably move into whereever their
   corresponding functions end up but, seeing as this
   is APPLE_ONLY, I'm not too concerned еее
}
const
	RS_EXDATA = $20;
	RS_ALLOWAGAIN = $40;
	RS_DELIMITMSG = $80;

{$ifc NOT OTKERNEL}
{ StreamRef is an opaque reference to a raw stream.}

type
	StreamRef = UnivPtr;
const
	kOTInvalidStreamRef = nil;
{ PollRef structure is used with the OTStreamPoll function.}
type
	PollRefPtr = ^PollRef;
	PollRef = record
		filler: SInt32;                 { holds a file descriptor an a UNIX system, replaced by ref (at end of structure) under OT}
		events: SInt16;
		revents: SInt16;
		ref: StreamRef;
	end;

{ Poll masks for use with OTStreamPoll: }

const
	POLLIN          = $001;   { A non-priority message is available                 }
	POLLPRI         = $002;   { A high priority message is available                }
	POLLOUT         = $004;   { The stream is writable for non-priority messages    }
	POLLERR         = $008;   { A error message has arrived                         }
	POLLHUP         = $010;   { A hangup has occurred                               }
	POLLNVAL        = $020;   { This fd is bogus                                    }
	POLLRDNORM      = $040;   { A non-priority message is available                 }
	POLLRDBAND      = $080;   { A priority message (band > 0) message is available  }
	POLLWRNORM      = $100;   { Same as POLLOUT                                     }
	POLLWRBAND      = $200;   { A priority band exists and is writable              }
	POLLMSG         = $400;   { A signal message has reached the front of the queue }

{ OTReadInfo structure is used with the various functions that read and peek at the stream head.}

type
	OTReadInfoPtr = ^OTReadInfo;
	OTReadInfo = record
		fType: UInt32;
		fCommand: OTCommand;
		fFiller: UInt32;                { For compatibility with OT 1.0 and 1.1 }
		fBytes: ByteCount;
		fError: OSStatus;
	end;
{ Opening and closing raw streams}
{
 *  OTStreamOpen()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTAsyncStreamOpen()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCreateStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTAsyncCreateStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamClose()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Polling a stream for activity}

{
 *  OTStreamPoll()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTAsyncStreamPoll()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Classic UNIX file descriptor operations}

{
 *  OTStreamRead()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamWrite()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamIoctl()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamPipe()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ there can be only 2!}
{ Notifiers and modes of operation}
{
 *  OTStreamInstallNotifier()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamRemoveNotifier()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamUseSyncIdleEvents()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamSetBlocking()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamSetNonBlocking()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamIsBlocking()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamSetSynchronous()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamSetAsynchronous()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamIsSynchronous()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ STREAMS primitives}

{
 *  OTStreamGetMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamGetPriorityMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamPutMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTStreamPutPriorityMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Miscellaneous stuff}

{
 *  OTStreamSetControlMask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   Opening endpoints and mappers on a Stream - these calls are synchronous, and may
   only be used at System Task time. Once the stream has been installed into a provider
   or endpoint, you should not continue to use STREAMS APIs on it
}

{
 *  OTOpenProviderOnStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTOpenEndpointOnStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   To quote an earlier version of this header file:
   
        Some functions that should only be used if
        you really know what you're doing.
}

{
 *  OTRemoveStreamFromProvider()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTPeekMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTReadMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTPutBackBuffer()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTPutBackPartialBuffer()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$endc}  { !OTKERNEL }

{$endc} { CALL_NOT_IN_CARBON }

{ ***** Port Utilities *****}
{$ifc NOT OTKERNEL}
{
   These types and routines are used during sophisticated
   port management.  High-level clients may get involved
   for things like request a port to be yielding, but typically
   this stuff is used by protocol infrastructure.
}
{
   OTPortCloseStruct is used when processing the kOTClosePortRequest
   and kOTYieldPortRequest events.
}

type
	OTPortCloseStructPtr = ^OTPortCloseStruct;
	OTPortCloseStruct = record
		fPortRef: OTPortRef;               { The port requested to be closed.}
		fTheProvider: ProviderRef;           { The provider using the port.}
		fDenyReason: OSStatus;            { Set to a negative number to deny the request}
	end;
{ OTClientList structure is used with the OTYieldPortRequest function.}
type
	OTClientListPtr = ^OTClientList;
	OTClientList = record
		fNumClients: ItemCount;
		fBuffer: packed array [0..3] of UInt8;
	end;
{
   Returns a buffer containing all of the clients that refused to yield the port.
   "size" is the total number of bytes @ buffer, including the fNumClients field.
}
{
 *  OTYieldPortRequest()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Send a notification to all Open Transport registered clients}
{
 *  OTNotifyAllClients()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Determine if "child" is a child port of "parent"}
{
 *  OTIsDependentPort()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$endc}  { !OTKERNEL }

{ ***** Timers ***** }
{
   STREAMS plug-ins code should not use these timers, instead
   they should use timer messages, ie mi_timer etc.
}

{$ifc NOT OTKERNEL}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	OTTimerTask = SIGNEDLONG;
=======
	OTTimerTask = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	OTTimerTask = SInt32;
>>>>>>> origin/fixes_2_2
=======
	OTTimerTask = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
{
   Under Carbon, OTCreateTimerTask takes a client context pointer.  Applications may pass NULL
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
 *  OTCreateTimerTaskInContext()   *** DEPRECATED ***
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function OTCreateTimerTaskInContext( upp: OTProcessUPP; arg: UnivPtr; clientContext: OTClientContextPtr ): SIGNEDLONG; external name '_OTCreateTimerTaskInContext';
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
function OTCreateTimerTaskInContext( upp: OTProcessUPP; arg: UnivPtr; clientContext: OTClientContextPtr ): SIGNEDLONG; external name '_OTCreateTimerTaskInContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$endc} {not TARGET_CPU_64}

{
 *  OTCreateTimerTask()
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
 *  OTCancelTimerTask()   *** DEPRECATED ***
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
function OTCancelTimerTask( timerTask: OTTimerTask ): Boolean; external name '_OTCancelTimerTask';
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
 *  OTDestroyTimerTask()   *** DEPRECATED ***
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
procedure OTDestroyTimerTask( timerTask: OTTimerTask ); external name '_OTDestroyTimerTask';
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
 *  OTScheduleTimerTask()   *** DEPRECATED ***
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
function OTScheduleTimerTask( timerTask: OTTimerTask; milliSeconds: OTTimeout ): Boolean; external name '_OTScheduleTimerTask';
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

{ The following macro may be used by applications only.}
// #define OTCreateTimerTask(upp, arg) OTCreateTimerTaskInContext(upp, arg, NULL)

{$endc}  { !OTKERNEL }

{ ***** Miscellaneous Helpful Functions *****}

{$ifc NOT OTKERNEL}
{
   These routines allow you to manipulate OT's buffer structures.
   If you use no-copy receives (described in "OpenTransport.h")
   you will need some of these routines, and may choose to use others.
   See "Open Tranport Advanced Client Programming" for documentation.
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
 *  OTBufferDataSize()   *** DEPRECATED ***
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
function OTBufferDataSize( var buffer: OTBuffer ): OTByteCount; external name '_OTBufferDataSize';
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
 *  OTReadBuffer()   *** DEPRECATED ***
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
function OTReadBuffer( var buffer: OTBufferInfo; dest: UnivPtr; var len: OTByteCount ): Boolean; external name '_OTReadBuffer';
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
 *  OTReleaseBuffer()   *** DEPRECATED ***
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
procedure OTReleaseBuffer( var buffer: OTBuffer ); external name '_OTReleaseBuffer';
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
 *  StoreIntoNetbuf()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  StoreMsgIntoNetbuf()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$endc}  { !OTKERNEL }

{ ***** OTConfiguration *****}
{$ifc CALL_NOT_IN_CARBON}
{$ifc NOT OTKERNEL}
{
   As promised in "OpenTransport.h", here are the routines
   for advanced operations on configurations.
}
{ Manipulating a configuration}

{
 *  OTCfigNewConfiguration()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigDeleteConfiguration()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigCloneConfiguration()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigPushNewSingleChild()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigPushParent()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigPushChild()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigPopChild()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigGetChild()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigSetPath()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigNewChild()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigAddChild()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigRemoveChild()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigSetPortRef()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigChangeProviderName()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Query a configuration}

{
 *  OTCfigNumberOfChildren()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigGetParent()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigGetOptionNetbuf()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigGetPortRef()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigGetInstallFlags()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigGetProviderName()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCfigIsPort()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$endc}  { !OTKERNEL }

{ ***** Configurators *****}
{
   The kOTConfiguratorInterfaceID define is what you need to add to your
   export file for the "interfaceID = " clause to export a configurator
   for ASLM.  Similarly, kOTConfiguratorCFMTag is used for CFM-based
   configurators.
}


// #define kOTConfiguratorInterfaceID  kOTClientPrefix "cfigMkr"
// #define kOTConfiguratorCFMTag       kOTClientPrefix "cfigMkr"

{$ifc NOT OTKERNEL}
type
	TOTConfiguratorRef = ^SInt32; { an opaque 32-bit type }
	TOTConfiguratorRefPtr = ^TOTConfiguratorRef;
{
   Typedef for the OTCanConfigure function, and the enum for which pass we're doing.
   The first (kOTSpecificConfigPass) is to give configurators a shot at the configuration
   before we start allowing the generic configurators to get into the act.
}
const
	kOTSpecificConfigPass = 0;
	kOTGenericConfigPass = 1;

type
	OTCanConfigureProcPtr = function( cfig: OTConfigurationRef; pass: UInt32 ): Boolean;
{ Typedef for the function to create and return a configurator object}
type
	OTCreateConfiguratorProcPtr = function( var cfigor: TOTConfiguratorRef ): OSStatus;
{
   Typedef for the "OTSetupConfigurator" function that your configurator library must export.
   The enum is for the type of configurator that it is.
}
const
	kOTSetupConfiguratorID = 'OTSetupConfigurator';
const
	kOTDefaultConfigurator = 0;
	kOTProtocolFamilyConfigurator = 1;
	kOTLinkDriverConfigurator = 2;

type
	OTSetupConfiguratorProcPtr = function( var canConfigure: OTCanConfigureProcPtr; var createConfigurator: OTCreateConfiguratorProcPtr; var configuratorType: UInt8 ): OSStatus;
{
   Procedure pointer definitions for the three key callbacks associated
   with a configurator, as established by OTNewConfigurator.
}
type
	OTCFConfigureProcPtr = function( cfigor: TOTConfiguratorRef; cfig: OTConfigurationRef ): OSStatus;
	OTCFCreateStreamProcPtr = function( cfigor: TOTConfiguratorRef; cfig: OTConfigurationRef; oFlags: OTOpenFlags; proc: OTNotifyUPP; contextPtr: UnivPtr ): OSStatus;
	OTCFHandleSystemEventProcPtr = procedure( cfigor: TOTConfiguratorRef; code: OTEventCode; result: OTResult; cookie: UnivPtr );
{
   Determine if this instance of your configurator is the "master"
   (the one that can create and destroy control streams)
}
{
 *  OTIsMasterConfigurator()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Get back the userData you passed in to OTNewConfigurator}
{
 *  OTGetConfiguratorUserData()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Create a configurator object for use by Open Transport}
{
 *  OTNewConfigurator()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Delete a configurator object created by OTNewConfigurator}
{
 *  OTDeleteConfigurator()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   A utility function to send notifications to the user - it takes care of calls
   from deferred tasks
}
{
 *  OTNotifyUser()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Call when the configurator unloads from memory}
{
 *  OTConfiguratorUnloaded()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   Call to create your control stream if you're not the master
   configurator.  You can also use the state machine function
   OTSMCreateControlStream(OTStateMachine*, OTConfigurationRef, TOTConfiguratorRef cfigor).
}
{
 *  OTCreateControlStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   A helpful function for the configurators to
   be able to recursively configure the children.
}
{
 *  OTConfigureChildren()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Allocate a bit in the system-wide control mask for streams.}
{
 *  OTNewControlMask()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Warning: These 2 APIs is going away}
{
 *  OTCloseProvidersByUseCount()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCloseProvidersByPortRef()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ These are the "real" APIs}
{
 *  OTCloseProviderByStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTCloseMatchingProviders()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ The following defines are for use in building an 'epcf' resource: }

{ Defines for fFlags field: }
const
	kIsReliable = $00000001;
	kIsNotReliable = $00000002;
	kSupportsOrderlyRelease = $00000004;

{ Defines for fProtocolType field: }
	kStream = $0001;
	kUStream = $0002;
	kTransaction = $0004;
	kUTransaction = $0008;
	kMapper = $0010;
	kGenericProtocol = $0020;

{ Defines for optionType field: }
const
	kBooleanOption = 0;
const
	kUnsignedValueOption = 1;
const
	kSignedValueOption = 2;
const
	kHexValueOption = 3;
const
	kPrintableStringOption = 4;
const
	kOctetStringOption = 5;

{ Defines for fUpperInterface and fLowerInterface: }
const
	kTPIInterface           = FourCharCode('TPI ');
	kDLPIInterface          = FourCharCode('DLPI');
	kMapperInterface        = FourCharCode('MAPR');
	kPrivateInterface       = -1;
const
	kNoInterface = 0;


{$endc}  { !OTKERNEL }

{$endc} { CALL_NOT_IN_CARBON }

{ ***** OTStateMachine *****}
{$ifc CALL_NOT_IN_CARBON}
{
   This utility set allows you to write an asynchronous chain of code that looks 
   somewhat like it is synchronous.  This is primarily used for plumbing 
   streams asynchronously, especially in configurators
}
{$ifc NOT OTKERNEL}
{ Alas, the state machine is only available to client code.  Sorry.}

{
   There are 12 or 8 bytes of reserved space at the front of
   the OTStateMachine structure, depending on whether you're
   building PowerPC or 68K code..  The OTStateMachineDataPad
   type compensates for this.
}

{$ifc TARGET_CPU_PPC}
type
	OTStateMachineDataPad = packed array [0..11] of UInt8;
{$elsec}
type
	OTStateMachineDataPad = packed array [0..7] of UInt8;
{$endc}  {TARGET_CPU_PPC}

{
   Forward define OTStateMachine so that OTStateProcPtr has
   access to it.
}
type
	OTStateMachinePtr = ^OTStateMachine;
{
   This type is is the required prototype of a state machine
   entry point.
}
	OTStateProcPtr = procedure( var sm: OTStateMachine );
{
   This type defines a routine that the state machine will
   call when the top level completes.
}
	OTSMCompleteProcPtr = procedure( contextPtr: UnivPtr );
{ And now for the state machine structure itself.}
	OTStateMachine = record
		fData: OTStateMachineDataPad;
		fCookie: UnivPtr;
		fCode: OTEventCode;
		fResult: OTResult;
	end;

// #define kOTSMBufferSize(callDepth) (80 + (callDepth * 8))
{
   For structSize, pass the size of your structure that you want associated with
   the state machine.  It can later be obtained by calling OTSMGetClientData.
   For bufSize, use the kOTSMBufferSize macro, plus the size of your structure
   to create a buffer on the stack. For synchronous calls, the stack buffer will
   be used (unless you pass in NULL).  The callDepth is the depth level of nested
   calls using OTSMCallStateProc.
}
{
 *  OTCreateStateMachine()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTDestroyStateMachine()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   OTSMCallStateProc used to take a parameter of type UInt16_p,
   which was defined to be the same as UInt32.  In an attempt
   to reduce the number of wacky types defined by the OT
   interfaces, we've changed these routines to just take a
   straight UInt32.  You should be warned that the current
   implementation does not support values outside of the
   range 0..32767.  The same applies to OTSMSetState.
}

{
 *  OTSMCallStateProc()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMGetState()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMSetState()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Fill out the fCookie, fCode, and fResult fields before calling!}
{
 *  OTSMComplete()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMPopCallback()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMWaitForComplete()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMCreateStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMOpenStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMIoctl()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMPutMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMGetMessage()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMReturnToCaller()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMGetClientData()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMInstallCompletionProc()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTSMCreateControlStream()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$endc}  { !OTKERNEL }

{ ***** Autopush Definitions *****}
{
   The autopush functionality for Open Transport is based on the names of
   devices and modules, rather than on the major number information like 
   SVR4.  This is so that autopush information can be set up for modules
   that are not yet loaded.
}


{ The name of the STREAMS driver you open and send the ioctls to.}
const
	kSADModuleName = 'sad';
{ Autopush ioctls.}
const
	I_SAD_SAP = $6701;						{  Set autopush information      }
	I_SAD_GAP = $6702;						{  Get autopush information      }
	I_SAD_VML = $6703;						{  Validate a list of modules (uses str_list structure)  }

{ Maximum number of modules autopushed on a driver.}

const
	kOTAutopushMax = 8;


{ ioctl structure used for SAD_SAP and SAD_GAP commands.}

type
	OTAutopushInfoPtr = ^OTAutopushInfo;
	OTAutopushInfo = record
		sap_cmd: UInt32;
		sap_device_name: packed array [0..31] of char;
		sap_minor: SInt32;
		sap_lastminor: SInt32;
		sap_npush: SInt32;
		sap_list: packed array [0..7,0..31] of char;
	end;
{ Command values for sap_cmd field of the above.}
const
	kSAP_ONE = 1;    { Configure a single minor device         }
	kSAP_RANGE = 2;    { Configure a range of minor devices     }
	kSAP_ALL = 3;    { Configure all minor devices          }
	kSAP_CLEAR = 4;     { Clear autopush information          }


{ ***** Configuration Helpers *****}

{
   These definitions are used by device driver and port scanner
   developers to provide a library giving client-side information about
   the registered ports, such as a user-visible name or an icon.
}

{ Configuration helper library prefix}

{
   This prefix is prepended to the string found in the "fResourceInfo"
   field of the OTPortRecord to build the actual library name of the
   configuration helper library.
}

const
	kPortConfigLibPrefix = 'OTPortCfg$';

{ Get user visible port name entry point.}

{
   This entry point returns the user visible name of the port.  If includeSlot
   is true, a slot distinguishing suffix (eg "slot X") should be added.  If
   includePort is true, a port distinguishing suffix (eg " port X") should be added for
   multiport cards.
}

const
	kOTGetUserPortNameID = 'OTGetUserPortName';
type
	OTGetPortNameProcPtr = procedure( var port: OTPortRecord; includeSlot: OTBooleanParam; includePort: OTBooleanParam; var userVisibleName: Str255 );
{ Get icon entry point.}
{
   This entry point returns the location of the icon for the port.  Return false if no
   icon is provided.
}

const
	kOTGetPortIconID = 'OTGetPortIcon';
type
	OTResourceLocatorPtr = ^OTResourceLocator;
	OTResourceLocator = record
		fFile: FSSpec;
		fResID: UInt16;
	end;
type
	OTGetPortIconProcPtr = function( var port: OTPortRecord; var iconLocation: OTResourceLocator ): Boolean;
{ ***** Application Access to Configuration Helpers *****}

{$ifc NOT OTKERNEL}
{
   These routines are used by clients to get information about ports.
   The canonical user of these routines is the OT control panel(s),
   but applications may want to use them as well (to display the list
   of available Ethernet cards, for example).
}
{  Returns a user friendly name for a port.}
{
 *  OTGetUserPortNameFromPortRef()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
    Returns the location for the icon familly representing the port.
    Returns false if the port has no icon.
}
{
 *  OTGetPortIconFromPortRef()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
    Gets the slot and other value for the default port of the given device type
    Returns false if there is no default port of that device type
}
{
 *  OTGetDefaultPort()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Returns true if the port can be used with the specified protocol.}
{
 *  OTIsPortCompatibleWith()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$endc}  { !OTKERNEL }

{$endc} { CALL_NOT_IN_CARBON }

{ ***** Common Utilities *****}
{
   The utilities defined in this section are available to both client
   and kernel code.  Cool huh?  These utilities differ from those
   provided in "OpenTransport.h" in that they are only available to native
   architecture clients.
}

{ Bitmap functions}

{ These functions atomically deal with a bitmap that is multiple-bytes long}

{
   Set the first clear bit in "bitMap", starting with bit "startBit",
   giving up after "numBits".  Returns the bit # that was set, or
   a kOTNotFoundErr if there was no clear bit available
}
{
 *  OTSetFirstClearBit()   *** DEPRECATED ***
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
function OTSetFirstClearBit( bitMap: UInt8Ptr; startBit: OTByteCount; numBits: OTByteCount ): OTResult; external name '_OTSetFirstClearBit';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Standard clear, set and test bit functions}
{
 *  OTClearBit()   *** DEPRECATED ***
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
function OTClearBit( bitMap: UInt8Ptr; bitNo: OTByteCount ): Boolean; external name '_OTClearBit';
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
 *  OTSetBit()   *** DEPRECATED ***
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
function OTSetBit( bitMap: UInt8Ptr; bitNo: OTByteCount ): Boolean; external name '_OTSetBit';
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
 *  OTTestBit()   *** DEPRECATED ***
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
function OTTestBit( bitMap: UInt8Ptr; bitNo: OTByteCount ): Boolean; external name '_OTTestBit';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ OTHashList}

{
   This implements a simple, but efficient hash list.  It is not
   thread-safe.
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
	OTHashProcPtr = function( var linkToHash: OTLink ): UInt32;
	OTHashSearchProcPtr = function( ref: {const} UnivPtr; var linkToCheck: OTLink ): Boolean;
	OTHashListPtr = ^OTHashList;
	OTHashList = record
		fHashProc: OTHashProcPtr;
		fHashTableSize: ByteCount;
		fHashBuckets: ^OTLinkPtr;
	end;
{
   Return the number of bytes of memory needed to create a hash list
   of at least "numEntries" entries.
}
{
 *  OTCalculateHashListMemoryNeeds()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
   Create an OTHashList from "memory".  Return an error if it
   couldn't be done.
}
{
 *  OTInitHashList()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTAddToHashList()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTRemoveLinkFromHashList()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTIsInHashList()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTFindInHashList()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTRemoveFromHashList()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{ Random functions}

{
   These implement a very simple random number generator, suitable
   for protocol implementations but not "cryptographically" random.
}

{
 *  OTGetRandomSeed()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTGetRandomNumber()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{ Concurrency Control}

{
   OTGate implements a cool concurrency control primitive.
   You're not going to understand it without reading the documentation!
   See "Open Transport Advanced Client Programming" for details.
   WARNING:
   This structure must be on a 4-byte boundary.
}

type
	OTGateProcPtr = function( var thisLink: OTLink ): Boolean;
	CFMLibraryInfoPtr = ^CFMLibraryInfo;
	CFMLibraryInfo = record
		fLIFO: OTLIFO;
		fList: OTList;
		fProc: OTGateProcPtr;
		fNumQueued: SInt32;
		fInside: SInt32;
	end;
{
 *  OTInitGate()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTEnterGate()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{
 *  OTLeaveGate()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in OTUtilityLib 1.0 and later
 }


{ ***** Shared Library Bonus Extras *****}

{$ifc CALL_NOT_IN_CARBON}
{
   These routines provide addition shared library support beyond
   that provided by the base shared library mechanism.
}
{
   Some flags which can be passed to the "loadFlags" parameter of the
   various CFM routines.  Not all flags can be used with all routines.
   See "Open Transport Advanced Client Programming" for details.
}

const
	kOTGetDataSymbol = 0;
	kOTGetCodeSymbol = 1;
	kOTLoadNewCopy = 2;
	kOTLoadACopy = 4;
	kOTFindACopy = 8;
	kOTLibMask = kOTLoadNewCopy or kOTLoadACopy or kOTFindACopy;
	kOTLoadLibResident = $20;

{ Finding all matching CFM libraries.}

{
   The routine OTFindCFMLibraries allows you to find all CFM libraries
   that match specific criteria.  The result is placed in a list
   of CFMLibraryInfo structures.  OT allocates those structures using
   a routine of type OTAllocMemProcPtr that you pass to OTFindCFMLibraries.
}

{
   A list of CFMLibraryInfo structures is returned by the OTFindCFMLibraries routine.
   The list is created out of the data that is passed to the function.
   
   IMPORTANT:
   Only the first 3 fields are valid when using OT 1.2 and older.
}

type
	CFMLibraryInfoPtr = ^CFMLibraryInfo;
	CFMLibraryInfo = record
		link: OTLink;                   { To link them all up on a list            }
		libName: UnivPtr;                { "C" String which is fragment name          }
		intlName: StringPtr;               { Pascal String which is internationalized name  }
		fileSpec: FSSpecPtr;               { location of fragment's file }
		pstring2: StringPtr;               { Secondary string from extended cfrg          }
		pstring3: StringPtr;               { Extra info from extended cfrg            }
	end;
{
   You must pass a routine of type OTAllocMemProcPtr to OTFindCFMLibraries
   which it calls to allocate memory for the CFMLibraryInfo structures.
}
type
	OTAllocMemProcPtr = function( size: OTByteCount ): UnivPtr;
{ Find CFM libraries of the specified kind and type}
{
 *  OTFindCFMLibraries()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Loading libraries and connecting to symbols.}

{ Load a CFM library by name}
{
 *  OTLoadCFMLibrary()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Load a CFM library and get a named pointer from it}
{
 *  OTGetCFMPointer()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Get a named pointer from a CFM library that's already loaded}
{
 *  OTGetCFMSymbol()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Release a connection to a CFM library}
{
 *  OTReleaseCFMConnection()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   You can call these routines in your CFM initialisation and termination
   routines to hold or unhold your libraries sections.
}
{
   Used in a CFM InitProc, will hold the executable code, if applicable.
   This can also be the InitProc of the library
}
{
 *  OTHoldThisCFMLibrary()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   Used in a CFM terminate proc, will unhold the executable code, if applicable.
   This can also be the terminate proc of the library
}
{
 *  OTUnholdThisCFMLibrary()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ ASLM Utilities}
{ Load an ASLM library}
{
 *  OTLoadASLMLibrary()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ Unload an ASLM library}
{
 *  OTUnloadASLMLibrary()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
   This is an ASLM utility routine.  You can get it by including
   "LibraryManagerUtilities.h", but since we only use a few ASLM utilities,
   we put the prototype here for convenience.
}

{
 *  UnloadUnusedLibraries()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{$ifc NOT OTKERNEL}
{******************************************************************************
** A few C++ objects for C++ fans
*******************************************************************************}
{$ifc CALL_NOT_IN_CARBON}


{$endc} { CALL_NOT_IN_CARBON }
{$endc}  { !OTKERNEL }

{$endc} { CALL_NOT_IN_CARBON }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_MAC and TARGET_CPU_PPC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======


end.
>>>>>>> graemeg/fixes_2_2
=======
=======
{$endc} {TARGET_OS_MAC and TARGET_CPU_PPC}
>>>>>>> origin/fixes_2.4

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
