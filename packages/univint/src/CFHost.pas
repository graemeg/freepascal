{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	 File:	   CFNetwork/CFHost.h
 
	 Contains:   CoreFoundation CFHost header
 
	 Copyright:  Copyright (c) 2001-2008, Apple Inc. All rights reserved.
 
	 Bugs?:	  For bug reports, consult the following page on
				 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					 http://bugs.freepascal.org
 
}
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated: Jonas Maebe <jonas@freepascal.org>, August 2015 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
					 http://www.freepascal.org/bugs.html
 
}
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
     File:       CFNetwork/CFHost.h
=======
	 File:	   CFNetwork/CFHost.h
>>>>>>> origin/fixes_2.4
 
	 Contains:   CoreFoundation CFHost header
 
	 Copyright:  Copyright (c) 2001-2008, Apple Inc. All rights reserved.
 
	 Bugs?:	  For bug reports, consult the following page on
				 the World Wide Web:
 
					 http://www.freepascal.org/bugs.html
 
}
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit CFHost;
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
=======
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
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
uses MacTypes,CFBase,CFData,CFArray,CFRunLoop,CFStream;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ALIGN MAC68K}
=======
{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew
=======
{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew
=======
{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew
=======
{$ALIGN POWER}
>>>>>>> origin/cpstrnew
=======
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}
>>>>>>> origin/fixes_2.4

{
 *  CFHostRef
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	This is the type of a reference to a host name or address lookup.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFHostRef = ^__CFHost; { an opaque type }
	__CFHost = record end;
=======
	CFHostRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFHostRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFHostRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFHostRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    This is the type of a reference to a host name or address lookup.
 }
type
	CFHostRef    = ^SInt32; { an opaque 32-bit type }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *	This is the type of a reference to a host name or address lookup.
 }
type
	CFHostRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{
 *  kCFStreamErrorDomainNetDB
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Errors listed in netdb.h
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainNetDB: SInt32; external name '_kCFStreamErrorDomainNetDB'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Errors listed in netdb.h
=======
 *	Errors listed in netdb.h
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainNetDB: SInt32; external name '_kCFStreamErrorDomainNetDB'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  kCFStreamErrorDomainSystemConfiguration
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Errors listed in SystemConfiguration/SystemConfiguration.h
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainSystemConfiguration: SInt32; external name '_kCFStreamErrorDomainSystemConfiguration'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Errors listed in SystemConfiguration/SystemConfiguration.h
=======
 *	Errors listed in SystemConfiguration/SystemConfiguration.h
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainSystemConfiguration: SInt32; external name '_kCFStreamErrorDomainSystemConfiguration'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostInfoType
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Host information types to be resolved.
=======
 *    Host information types to be resolved.
>>>>>>> graemeg/fixes_2_2
=======
 *    Host information types to be resolved.
>>>>>>> origin/fixes_2_2
=======
 *	Host information types to be resolved.
>>>>>>> origin/fixes_2.4
 }
type
	CFHostInfoType = SInt32;
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
   * Results value is a CFArray of CFData's (each being a struct
   * sockaddr)
   }
	kCFHostAddresses = 0;
=======
=======
>>>>>>> origin/fixes_2_2
  {
   * Results value is a CFArray of CFData's (each being a struct
   * sockaddr)
   }
  kCFHostAddresses              = 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
   * Results value is a CFArray of CFData's (each being a struct
   * sockaddr)
   }
	kCFHostAddresses = 0;
>>>>>>> origin/fixes_2.4

  {
   * Results value is a CFArray of CFString's
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFHostNames = 1;
=======
  kCFHostNames                  = 1;
>>>>>>> graemeg/fixes_2_2
=======
  kCFHostNames                  = 1;
>>>>>>> origin/fixes_2_2
=======
	kCFHostNames = 1;
>>>>>>> origin/fixes_2.4

  {
   * Results value is a CFData wrapping SCNetworkConnectionFlags
   * (defined in SystemConfiguration/SCNetwork.h)
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFHostReachability = 2;
=======
  kCFHostReachability           = 2;
>>>>>>> graemeg/fixes_2_2
=======
  kCFHostReachability           = 2;
>>>>>>> origin/fixes_2_2
=======
	kCFHostReachability = 2;
>>>>>>> origin/fixes_2.4


{
 *  CFHostClientContext
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Structure containing the user-defined data and callbacks for
 *	CFHost objects.
 }
type
	CFHostClientContext = record
{
   * The version number of the structure type being passed in as a
   * parameter to the CFHost client function. Valid version number is
   * currently 0.
   }
		version: CFIndex;

  {
   * An arbitrary pointer to client-defined data, which can be
   * associated with the host and is passed to the callbacks.
   }
		info: UnivPtr;

  {
   * The callback used to add a retain for the host on the info pointer
   * for the life of the host, and may be used for temporary references
   * the host needs to take. This callback returns the actual info
   * pointer to store in the host, almost always just the pointer
   * passed as the parameter.
   }
		retain: CFAllocatorRetainCallBack;

  {
   * The callback used to remove a retain previously added for the host
   * on the info pointer.
   }
		release: CFAllocatorReleaseCallBack;

  {
   * The callback used to create a descriptive string representation of
   * the info pointer (or the data pointed to by the info pointer) for
   * debugging purposes. This is used by the CFCopyDescription()
   * function.
   }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Structure containing the user-defined data and callbacks for
 *    CFHost objects.
=======
 *	Structure containing the user-defined data and callbacks for
 *	CFHost objects.
>>>>>>> origin/fixes_2.4
 }
type
	CFHostClientContext = record
{
   * The version number of the structure type being passed in as a
   * parameter to the CFHost client function. Valid version number is
   * currently 0.
   }
		version: CFIndex;

  {
   * An arbitrary pointer to client-defined data, which can be
   * associated with the host and is passed to the callbacks.
   }
		info: UnivPtr;

  {
   * The callback used to add a retain for the host on the info pointer
   * for the life of the host, and may be used for temporary references
   * the host needs to take. This callback returns the actual info
   * pointer to store in the host, almost always just the pointer
   * passed as the parameter.
   }
		retain: CFAllocatorRetainCallBack;

  {
   * The callback used to remove a retain previously added for the host
   * on the info pointer.
   }
		release: CFAllocatorReleaseCallBack;
<<<<<<< HEAD
	
		{
		 * The callback used to create a descriptive string representation of
		 * the info pointer (or the data pointed to by the info pointer) for
		 * debugging purposes. This is used by the CFCopyDescription()
		 * function.
		 }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

  {
   * The callback used to create a descriptive string representation of
   * the info pointer (or the data pointed to by the info pointer) for
   * debugging purposes. This is used by the CFCopyDescription()
   * function.
   }
>>>>>>> origin/fixes_2.4
		copyDescription: CFAllocatorCopyDescriptionCallBack;
	end;
	CFHostClientContextPtr = ^CFHostClientContext;

{
 *  CFHostClientCallBack
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Callback function which is called upon error or completion of an
 *	asynchronous resolve.
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  Host whose resolution is complete.
 *	
 *	typeInfo:
 *	  Enum representing which info resolution is complete.
 *	
 *	error:
 *	  Reference to an error structure if the resolution failed.
 *	
 *	info:
 *	  Client's info reference which was passed into the client
 *	  context.
 }
type
	CFHostClientCallBack = procedure( theHost: CFHostRef; typeInfo: CFHostInfoType; const (*var*) error: CFStreamError; info: UnivPtr );
=======
=======
>>>>>>> origin/fixes_2_2
 *    Callback function which is called upon error or completion of an
 *    asynchronous resolve.
=======
 *	Callback function which is called upon error or completion of an
 *	asynchronous resolve.
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  Host whose resolution is complete.
 *	
 *	typeInfo:
 *	  Enum representing which info resolution is complete.
 *	
 *	error:
 *	  Reference to an error structure if the resolution failed.
 *	
 *	info:
 *	  Client's info reference which was passed into the client
 *	  context.
 }
<<<<<<< HEAD
type CFHostClientCallBack = procedure( theHost: CFHostRef; typeInfo: CFHostInfoType; const (*var*) error: CFStreamError; info: UnivPtr );
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	CFHostClientCallBack = procedure( theHost: CFHostRef; typeInfo: CFHostInfoType; const (*var*) error: CFStreamError; info: UnivPtr );
>>>>>>> origin/fixes_2.4


{
 *  CFHostGetTypeID()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Returns the type identifier of all CFHost instances.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostGetTypeID: CFTypeID; external name '_CFHostGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Returns the type identifier of all CFHost instances.
=======
 *	Returns the type identifier of all CFHost instances.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostGetTypeID: CFTypeID; external name '_CFHostGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostCreateWithName()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Creates a new host object with the given name.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	allocator:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  host. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	hostname:
 *	  A CFStringRef representing the name of the host. Must be
 *	  non-NULL.  If this reference is not a valid CFStringRef, the
 *	  behavior is undefined.
 *  
 *  Result:
 *	A valid CFHostRef which may now be resolved, or NULL if
 *	unsuccessful.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostCreateWithName( allocator: CFAllocatorRef; hostname: CFStringRef ): CFHostRef; external name '_CFHostCreateWithName';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Creates a new host object with the given name.
=======
 *	Creates a new host object with the given name.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	allocator:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  host. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	hostname:
 *	  A CFStringRef representing the name of the host. Must be
 *	  non-NULL.  If this reference is not a valid CFStringRef, the
 *	  behavior is undefined.
 *  
 *  Result:
 *	A valid CFHostRef which may now be resolved, or NULL if
 *	unsuccessful.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostCreateWithName( allocator: CFAllocatorRef; hostname: CFStringRef ): CFHostRef; external name '_CFHostCreateWithName';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostCreateWithAddress()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Creates a new host object with the given address.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	allocator:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  host. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	addr:
 *	  A CFDataRef containing a struct sockaddr which is the address
 *	  of the host. Must be non-NULL.  If this reference is not a
 *	  valid CFDataRef, the behavior is undefined.
 *  
 *  Result:
 *	A valid CFHostRef which may now be resolved, or NULL if
 *	unsuccessful.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostCreateWithAddress( allocator: CFAllocatorRef; addr: CFDataRef ): CFHostRef; external name '_CFHostCreateWithAddress';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Creates a new host object with the given address.
=======
 *	Creates a new host object with the given address.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	allocator:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  host. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	addr:
 *	  A CFDataRef containing a struct sockaddr which is the address
 *	  of the host. Must be non-NULL.  If this reference is not a
 *	  valid CFDataRef, the behavior is undefined.
 *  
 *  Result:
 *	A valid CFHostRef which may now be resolved, or NULL if
 *	unsuccessful.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostCreateWithAddress( allocator: CFAllocatorRef; addr: CFDataRef ): CFHostRef; external name '_CFHostCreateWithAddress';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostCreateCopy()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Creates a new host object as a copy of host argument.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  new host. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	host:
 *	  A CFHostRef representing the original host. Must be non-NULL. 
 *	  If this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *  
 *  Result:
 *	A valid CFHostRef which contains a copy of all previously
 *	resolved data from the original.  NULL is returned in the case of
 *	failure.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostCreateCopy( alloc: CFAllocatorRef; host: CFHostRef ): CFHostRef; external name '_CFHostCreateCopy';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Creates a new host object as a copy of host argument.
=======
 *	Creates a new host object as a copy of host argument.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  new host. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	host:
 *	  A CFHostRef representing the original host. Must be non-NULL. 
 *	  If this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *  
 *  Result:
 *	A valid CFHostRef which contains a copy of all previously
 *	resolved data from the original.  NULL is returned in the case of
 *	failure.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostCreateCopy( alloc: CFAllocatorRef; host: CFHostRef ): CFHostRef; external name '_CFHostCreateCopy';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostStartInfoResolution()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Performs a lookup for the given host.  It will search for the
 *	requested information if there is no other active request. 
 *	Previously cached information of the given type will be released.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which should be resolved. Must be non-NULL. If
 *	  this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	info:
 *	  The enum representing the type of information to be retrieved. 
 *	  If the value is not a valid type, the behavior is undefined.
 *	
 *	error:
 *	  A reference to a CFStreamError structure which will be filled
 *	  with any error information should an error occur.  May be set
 *	  to NULL if error information is not wanted.
 *  
 *  Result:
 *	Returns TRUE on success and FALSE on failure.  In asynchronous
 *	mode, this function will return immediately.  In synchronous
 *	mode, it will block until the resolve has completed or until the
 *	resolve is cancelled.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostStartInfoResolution( theHost: CFHostRef; info: CFHostInfoType; error: CFStreamErrorPtr { can be NULL } ): Boolean; external name '_CFHostStartInfoResolution';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Performs a lookup for the given host.  It will search for the
 *    requested information if there is no other active request. 
 *    Previously cached information of the given type will be released.
=======
 *	Performs a lookup for the given host.  It will search for the
 *	requested information if there is no other active request. 
 *	Previously cached information of the given type will be released.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which should be resolved. Must be non-NULL. If
 *	  this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	info:
 *	  The enum representing the type of information to be retrieved. 
 *	  If the value is not a valid type, the behavior is undefined.
 *	
 *	error:
 *	  A reference to a CFStreamError structure which will be filled
 *	  with any error information should an error occur.  May be set
 *	  to NULL if error information is not wanted.
 *  
 *  Result:
 *	Returns TRUE on success and FALSE on failure.  In asynchronous
 *	mode, this function will return immediately.  In synchronous
 *	mode, it will block until the resolve has completed or until the
 *	resolve is cancelled.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function CFHostStartInfoResolution( theHost: CFHostRef; info: CFHostInfoType; error: CFStreamErrorPtr ): Boolean; external name '_CFHostStartInfoResolution';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CFHostStartInfoResolution( theHost: CFHostRef; info: CFHostInfoType; error: CFStreamErrorPtr { can be NULL } ): Boolean; external name '_CFHostStartInfoResolution';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostGetAddressing()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	Tries to retrieve the known addresses from the given host.
 *	Returns a CFArrayRef of addresses if known and there were some.
 *	NULL is returned otherwise.  Each address is a CFDataRef wrapping
 *	a struct sockaddr.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The function gets the data in a thread-safe manner, but the
 *	resulting data is not safe.  Since it is returned as a matter of
 *	a get opposed to a copy, the data is not safe if the host is
 *	being altered from another thread.
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which contains the relevant information. Must be
 *	  non-NULL. If this reference is not a valid CFHostRef, the
 *	  behavior is undefined.
 *	
 *	hasBeenResolved:
 *	  A reference to a Boolean which returns FALSE if the information
 *	  was not available (e.g. CFHostStartInfoResolution has not been
 *	  called), otherwise TRUE will be returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFHostGetAddressing( theHost: CFHostRef; hasBeenResolved: BooleanPtr { can be NULL } ): CFArrayRef; external name '_CFHostGetAddressing';
=======
function CFHostGetAddressing( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetAddressing';
>>>>>>> graemeg/cpstrnew
=======
function CFHostGetAddressing( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetAddressing';
>>>>>>> graemeg/cpstrnew
=======
function CFHostGetAddressing( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetAddressing';
>>>>>>> graemeg/cpstrnew
=======
function CFHostGetAddressing( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetAddressing';
>>>>>>> origin/cpstrnew
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Tries to retrieve the known addresses from the given host.
 *    Returns a CFArrayRef of addresses if known and there were some.
 *    NULL is returned otherwise.  Each address is a CFDataRef wrapping
 *    a struct sockaddr.
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The function gets the data in a thread-safe manner, but the
 *	resulting data is not safe.  Since it is returned as a matter of
 *	a get opposed to a copy, the data is not safe if the host is
 *	being altered from another thread.
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which contains the relevant information. Must be
 *	  non-NULL. If this reference is not a valid CFHostRef, the
 *	  behavior is undefined.
 *	
 *	hasBeenResolved:
 *	  A reference to a Boolean which returns FALSE if the information
 *	  was not available (e.g. CFHostStartInfoResolution has not been
 *	  called), otherwise TRUE will be returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostGetAddressing( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetAddressing';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostGetNames()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Tries to retrieve the names/aliases from the given host. Returns
 *	a CFArrayRef of names for the given host.  NULL is returned
 *	otherwise.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The function gets the data in a thread-safe manner, but the
 *	resulting data is not safe.  Since it is returned as a matter of
 *	a get opposed to a copy, the data is not safe if the host is
 *	being altered from another thread.
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which contains the relevant information. Must be
 *	  non-NULL. If this reference is not a valid CFHostRef, the
 *	  behavior is undefined.
 *	
 *	hasBeenResolved:
 *	  A reference to a Boolean which returns FALSE if the information
 *	  was not available (e.g. CFHostStartInfoResolution has not been
 *	  called), otherwise TRUE will be returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFHostGetNames( theHost: CFHostRef; hasBeenResolved: BooleanPtr { can be NULL } ): CFArrayRef; external name '_CFHostGetNames';
=======
function CFHostGetNames( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetNames';
>>>>>>> graemeg/cpstrnew
=======
function CFHostGetNames( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetNames';
>>>>>>> graemeg/cpstrnew
=======
function CFHostGetNames( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetNames';
>>>>>>> graemeg/cpstrnew
=======
function CFHostGetNames( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetNames';
>>>>>>> origin/cpstrnew
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Tries to retrieve the names/aliases from the given host. Returns
 *    a CFArrayRef of names for the given host.  NULL is returned
 *    otherwise.
=======
 *	Tries to retrieve the names/aliases from the given host. Returns
 *	a CFArrayRef of names for the given host.  NULL is returned
 *	otherwise.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The function gets the data in a thread-safe manner, but the
 *	resulting data is not safe.  Since it is returned as a matter of
 *	a get opposed to a copy, the data is not safe if the host is
 *	being altered from another thread.
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which contains the relevant information. Must be
 *	  non-NULL. If this reference is not a valid CFHostRef, the
 *	  behavior is undefined.
 *	
 *	hasBeenResolved:
 *	  A reference to a Boolean which returns FALSE if the information
 *	  was not available (e.g. CFHostStartInfoResolution has not been
 *	  called), otherwise TRUE will be returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostGetNames( theHost: CFHostRef; var hasBeenResolved: Boolean ): CFArrayRef; external name '_CFHostGetNames';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostGetReachability()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	Tries to retrieve the reachability of the given host. Returns a
 *	CFDataRef which wraps the reachability flags. NULL will be
 *	returned if the value has not been resolved. The possible values
 *	of these flags is declared in SystemConfiguration/SCNetwork.h.
 *	Returns FALSE if the information was not available, otherwise
 *	TRUE will be returned with the results containing the requested
 *	information.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The function gets the data in a thread-safe manner, but the
 *	resulting data is not safe.  Since it is returned as a matter of
 *	a get opposed to a copy, the data is not safe if the host is
 *	being altered from another thread.
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which contains the relevant information. Must be
 *	  non-NULL. If this reference is not a valid CFHostRef, the
 *	  behavior is undefined.
 *	
 *	hasBeenResolved:
 *	  A reference to a Boolean which returns FALSE if the information
 *	  was not available (e.g. CFHostStartInfoResolution has not been
 *	  called), otherwise TRUE will be returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostGetReachability( theHost: CFHostRef; hasBeenResolved: BooleanPtr { can be NULL } ): CFDataRef; external name '_CFHostGetReachability';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Tries to retrieve the reachability of the given host. Returns a
 *    CFDataRef which wraps the reachability flags. NULL will be
 *    returned if the value has not been resolved. The possible values
 *    of these flags is declared in SystemConfiguration/SCNetwork.h.
 *    Returns FALSE if the information was not available, otherwise
 *    TRUE will be returned with the results containing the requested
 *    information.
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The function gets the data in a thread-safe manner, but the
 *	resulting data is not safe.  Since it is returned as a matter of
 *	a get opposed to a copy, the data is not safe if the host is
 *	being altered from another thread.
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which contains the relevant information. Must be
 *	  non-NULL. If this reference is not a valid CFHostRef, the
 *	  behavior is undefined.
 *	
 *	hasBeenResolved:
 *	  A reference to a Boolean which returns FALSE if the information
 *	  was not available (e.g. CFHostStartInfoResolution has not been
 *	  called), otherwise TRUE will be returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function CFHostGetReachability( theHost: CFHostRef; hasBeenResolved: BooleanPtr ): CFDataRef; external name '_CFHostGetReachability';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CFHostGetReachability( theHost: CFHostRef; hasBeenResolved: BooleanPtr { can be NULL } ): CFDataRef; external name '_CFHostGetReachability';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostCancelInfoResolution()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Cancels an outstanding asynchronous or synchronous resolve.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which is currently resolving.  Must be non-NULL.
 *	  If this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	info:
 *	  The enum representing which resolution to be canceled.  If the
 *	  value is not a valid type, the behavior is undefined.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHostCancelInfoResolution( theHost: CFHostRef; info: CFHostInfoType ); external name '_CFHostCancelInfoResolution';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Cancels an outstanding asynchronous or synchronous resolve.
=======
 *	Cancels an outstanding asynchronous or synchronous resolve.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which is currently resolving.  Must be non-NULL.
 *	  If this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	info:
 *	  The enum representing which resolution to be canceled.  If the
 *	  value is not a valid type, the behavior is undefined.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHostCancelInfoResolution( theHost: CFHostRef; info: CFHostInfoType ); external name '_CFHostCancelInfoResolution';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostSetClient()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Associates a client context and callback function with a
 *	CFHostRef.  This is required for asynchronous usage.  If not set,
 *	resolve will take place synchronously.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which is getting a client.  Must be non-NULL. If
 *	  this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	clientCB:
 *	  A CFHostClientCallBack which will be called when the resolve
 *	  completes or is cancelled.  Use NULL to remove the client
 *	  association with a host object.
 *	
 *	clientContext:
 *	  A CFHostClientContext which is used to set the contextual
 *	  information associated with the host object.  The info pointer
 *	  from the struct will be passed to the callback function. If
 *	  setting a client, this value must be non-NULL.
 *  
 *  Result:
 *	Returns TRUE if the procedure was a success, otherwise it returns
 *	FALSE.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHostSetClient( theHost: CFHostRef; clientCB: CFHostClientCallBack { can be NULL }; clientContext: CFHostClientContextPtr { can be NULL } ): Boolean; external name '_CFHostSetClient';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Associates a client context and callback function with a
 *    CFHostRef.  This is required for asynchronous usage.  If not set,
 *    resolve will take place synchronously.
=======
 *	Associates a client context and callback function with a
 *	CFHostRef.  This is required for asynchronous usage.  If not set,
 *	resolve will take place synchronously.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which is getting a client.  Must be non-NULL. If
 *	  this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	clientCB:
 *	  A CFHostClientCallBack which will be called when the resolve
 *	  completes or is cancelled.  Use NULL to remove the client
 *	  association with a host object.
 *	
 *	clientContext:
 *	  A CFHostClientContext which is used to set the contextual
 *	  information associated with the host object.  The info pointer
 *	  from the struct will be passed to the callback function. If
 *	  setting a client, this value must be non-NULL.
 *  
 *  Result:
 *	Returns TRUE if the procedure was a success, otherwise it returns
 *	FALSE.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function CFHostSetClient( theHost: CFHostRef; clientCB: CFHostClientCallBack; clientContext: CFHostClientContextPtr ): Boolean; external name '_CFHostSetClient';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CFHostSetClient( theHost: CFHostRef; clientCB: CFHostClientCallBack { can be NULL }; clientContext: CFHostClientContextPtr { can be NULL } ): Boolean; external name '_CFHostSetClient';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostScheduleWithRunLoop()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Schedules the given host on a run loop and mode so the client
 *	will receive its callbacks on that loop and mode.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which is being scheduled.  Must be non-NULL. If
 *	  this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	runLoop:
 *	  A CFRunLoopRef on which the host should be scheduled. Must be
 *	  non-NULL.  If this reference is not a valid CFRunLoopRef, the
 *	  behavior is undefined.
 *	
 *	runLoopMode:
 *	  A CFStringRef which is the mode in which the run loop will be
 *	  running when notification occurs.  Must be non-NULL. If this
 *	  reference is not a valid CFStringRef, the behavior is undefined.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHostScheduleWithRunLoop( theHost: CFHostRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ); external name '_CFHostScheduleWithRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Schedules the given host on a run loop and mode so the client
 *    will receive its callbacks on that loop and mode.
=======
 *	Schedules the given host on a run loop and mode so the client
 *	will receive its callbacks on that loop and mode.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which is being scheduled.  Must be non-NULL. If
 *	  this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	runLoop:
 *	  A CFRunLoopRef on which the host should be scheduled. Must be
 *	  non-NULL.  If this reference is not a valid CFRunLoopRef, the
 *	  behavior is undefined.
 *	
 *	runLoopMode:
 *	  A CFStringRef which is the mode in which the run loop will be
 *	  running when notification occurs.  Must be non-NULL. If this
 *	  reference is not a valid CFStringRef, the behavior is undefined.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHostScheduleWithRunLoop( theHost: CFHostRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ); external name '_CFHostScheduleWithRunLoop';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFHostUnscheduleFromRunLoop()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Unschedules the given host from a run loop and mode so the client
 *	will not receive its callbacks on that loop and mode.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which is being unscheduled.  Must be non-NULL. If
 *	  this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	runLoop:
 *	  A CFRunLoopRef on which the host is scheduled and should now be
 *	  unscheduled.  Must be non-NULL.  If this reference is not a
 *	  valid CFRunLoopRef, the behavior is undefined.
 *	
 *	runLoopMode:
 *	  A CFStringRef which is the mode in which the host is scheduled
 *	  and should be unscheduled.  Must be non-NULL. If this reference
 *	  is not a valid CFStringRef, the behavior is undefined.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHostUnscheduleFromRunLoop( theHost: CFHostRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ); external name '_CFHostUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)


{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Unschedules the given host from a run loop and mode so the client
 *    will not receive its callbacks on that loop and mode.
=======
 *	Unschedules the given host from a run loop and mode so the client
 *	will not receive its callbacks on that loop and mode.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	theHost:
 *	  The CFHostRef which is being unscheduled.  Must be non-NULL. If
 *	  this reference is not a valid CFHostRef, the behavior is
 *	  undefined.
 *	
 *	runLoop:
 *	  A CFRunLoopRef on which the host is scheduled and should now be
 *	  unscheduled.  Must be non-NULL.  If this reference is not a
 *	  valid CFRunLoopRef, the behavior is undefined.
 *	
 *	runLoopMode:
 *	  A CFStringRef which is the mode in which the host is scheduled
 *	  and should be unscheduled.  Must be non-NULL. If this reference
 *	  is not a valid CFStringRef, the behavior is undefined.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHostUnscheduleFromRunLoop( theHost: CFHostRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ); external name '_CFHostUnscheduleFromRunLoop';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)


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
