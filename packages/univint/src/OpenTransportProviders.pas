{
<<<<<<< HEAD
<<<<<<< HEAD
     File:       OSServices/OpenTransportProviders.h
 
     Contains:   *** DEPRECATED *** This file contains provider-specific definitions for various built-in providers.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  (c) 1993-2011 Apple Inc. and Mentat Inc. All rights reserved.
=======
     Version:    OpenTransport-110~114
 
     Copyright:  © 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  © 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  © 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OpenTransport-110~114
 
     Copyright:  © 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       OT/OpenTransportProviders.h
 
     Contains:   This file contains provider-specific definitions for various built-in providers.
 
     Version:    OpenTransport-110~114
 
<<<<<<< HEAD
     Copyright:  © 1993-2005 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1993-2008 by Apple Computer, Inc. and Mentat Inc., all rights reserved.
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

unit OpenTransportProviders;
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
=======
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
{$elsec}
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
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
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
uses MacTypes,OpenTransport;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/cpstrnew

{ this header is only supported on Mac OS X < 10.4, and Mac OS X < 10.4 does
  not support i386
}
{$ifc TARGET_OS_MAC and TARGET_CPU_PPC}
=======
>>>>>>> graemeg/cpstrnew
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

{ this header is only supported on Mac OS X < 10.4, and Mac OS X < 10.4 does
  not support i386
}
{$ifc TARGET_OS_MAC and TARGET_CPU_PPC}

{ this header is only supported on Mac OS X < 10.4, and Mac OS X < 10.4 does
  not support i386
}
{$ifc TARGET_OS_MAC and TARGET_CPU_PPC}

{ this header is only supported on Mac OS X < 10.4, and Mac OS X < 10.4 does
  not support i386
}
{$ifc TARGET_OS_MAC and TARGET_CPU_PPC}
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2

{ this header is only supported on Mac OS X < 10.4, and Mac OS X < 10.4 does
  not support i386
}
{$ifc TARGET_OS_MAC and TARGET_CPU_PPC}

{$ALIGN MAC68K}

{
    All OpenTransport Manager APIs are deprecated in MacOSX 10.4, instead of using OpenTransport,
    consider using CFNetwork or socket library.
}
{ ***** TCP/IP *****}

{ Basic types}

type
	InetPort = UInt16;
	InetHost = UInt32;
{  Enums used as address type designations.}
const
	AF_INET = 2;
const
	AF_DNS = 42;


{
    Enum which can be used to bind to all IP interfaces
    rather than a specific one.
}

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kOTAnyInetAddress = 0;     { Wildcard}
=======
	kOTAnyInetAddress = 0;     { Wildcard }
>>>>>>> graemeg/fixes_2_2
=======
	kOTAnyInetAddress = 0;     { Wildcard }
>>>>>>> origin/fixes_2_2
=======
	kOTAnyInetAddress = 0;     { Wildcard}
>>>>>>> origin/fixes_2.4

{
   Define the InetSvcRef type.  This type needs special
   processing because in C++ it's a subclass of TProvider.
   See the definition of TEndpointRef in "OpenTransport.h"
   for the logic behind this definition.
}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	InetSvcRef = ProviderRef; { an opaque type }
=======
	InetSvcRef = ProviderRef; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	InetSvcRef = ProviderRef; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	InetSvcRef = ProviderRef; { an opaque type }
>>>>>>> origin/fixes_2.4
	InetSvcRefPtr = ^InetSvcRef;

const
	kDefaultInternetServicesPath = -3;
{ Shared library prefixes}


const
	kInetVersion = '3.1.1';
const
	kInetPrefix = 'ot:inet$';

{ Module Names}


const
	kDNRName = 'dnr';
const
	kTCPName = 'tcp';
const
	kUDPName = 'udp';
const
	kRawIPName = 'rawip';

{ XTI Options}

{ Protocol levels}

const
	INET_IP = $00;
	INET_TCP = $06;
	INET_UDP = $11;

{ TCP Level Options}

const
	TCP_NODELAY = $01;
	TCP_MAXSEG = $02;
	TCP_NOTIFY_THRESHOLD = $10; {* not a real XTI option }
	TCP_ABORT_THRESHOLD = $11; {* not a real XTI option }
	TCP_CONN_NOTIFY_THRESHOLD = $12; {* not a real XTI option }
	TCP_CONN_ABORT_THRESHOLD = $13; {* not a real XTI option }
	TCP_OOBINLINE = $14; {* not a real XTI option }
	TCP_URGENT_PTR_TYPE = $15; {* not a real XTI option }
	TCP_KEEPALIVE = $0008; { keepalive defined in OpenTransport.h }

const
	T_GARBAGE = 2;

{ UDP Level Options}

const
	UDP_CHECKSUM = $0600;
	UDP_RX_ICMP = $02;

{ IP Level Options}
const
	kIP_OPTIONS = $01;
	kIP_TOS = $02;
	kIP_TTL = $03;
	kIP_REUSEADDR = $04;
	kIP_DONTROUTE = $10;
	kIP_BROADCAST = $20;
	kIP_REUSEPORT = $0200;
	kIP_HDRINCL = $1002;
	kIP_RCVOPTS = $1005;
	kIP_RCVDSTADDR = $1007;
	kIP_MULTICAST_IF = $1010; { set/get IP multicast interface }
	kIP_MULTICAST_TTL = $1011; { set/get IP multicast timetolive    }
	kIP_MULTICAST_LOOP = $1012; { set/get IP multicast loopback  }
	kIP_ADD_MEMBERSHIP = $1013; { add an IP group membership     }
	kIP_DROP_MEMBERSHIP = $1014; { drop an IP group membership       }
	kIP_BROADCAST_IFNAME = $1015; { Set interface for broadcasts   }
	kIP_RCVIFADDR = $1016; { Set interface for broadcasts   }

const
	IP_OPTIONS = 1;
{
   BSD's netinet/in.h uses different values for the same IP_ logical constants.
   If you are using OT and want those values, prefix your use with k
   e.g. change IP_TTL to kIP_TTL in your source code
}
const
	DVMRP_INIT = 100;  { DVMRP-specific setsockopt commands, from ip_mroute.h}
	DVMRP_DONE = 101;
	DVMRP_ADD_VIF = 102;
	DVMRP_DEL_VIF = 103;
	DVMRP_ADD_LGRP = 104;
	DVMRP_DEL_LGRP = 105;
	DVMRP_ADD_MRT = 106;
	DVMRP_DEL_MRT = 107;


{ IP_TOS precdence levels}

const
	T_ROUTINE = 0;
	T_PRIORITY = 1;
	T_IMMEDIATE = 2;
	T_FLASH = 3;
	T_OVERRIDEFLASH = 4;
	T_CRITIC_ECP = 5;
	T_INETCONTROL = 6;
	T_NETCONTROL = 7;

{  IP_TOS type of service}

const
	T_NOTOS = $00;
	T_LDELAY = 1 shl 4;
	T_HITHRPT = 1 shl 3;
	T_HIREL = 1 shl 2;

// #define SET_TOS(prec,tos)   (((0x7 & (prec)) << 5) | (0x1c & (tos)))
{ IP Multicast option structures}

type
	TIPAddMulticastPtr = ^TIPAddMulticast;
	TIPAddMulticast = record
		multicastGroupAddress: InetHost;
		interfaceAddress: InetHost;
	end;
{ Protocol-specific events}
const
	T_DNRSTRINGTOADDRCOMPLETE = kPRIVATEEVENT + 1;
	T_DNRADDRTONAMECOMPLETE = kPRIVATEEVENT + 2;
	T_DNRSYSINFOCOMPLETE = kPRIVATEEVENT + 3;
	T_DNRMAILEXCHANGECOMPLETE = kPRIVATEEVENT + 4;
	T_DNRQUERYCOMPLETE = kPRIVATEEVENT + 5;

{ InetAddress}

type
	InetAddressPtr = ^InetAddress;
	InetAddress = record
		fAddressType: OTAddressType;           { always AF_INET}
		fPort: InetPort;                  { Port number }
		fHost: InetHost;                  { Host address in net byte order}
		fUnused: packed array [0..7] of UInt8;			{  Traditional unused bytes }
	end;
{ Domain Name Resolver (DNR) }
const
	kMaxHostAddrs = 10;
	kMaxSysStringLen = 32;
	kMaxHostNameLen = 255;


type
	InetDomainName = packed array [0..255] of char;
	InetHostInfoPtr = ^InetHostInfo;
	InetHostInfo = record
		name: InetDomainName;
		addrs: array [0..9] of InetHost;
	end;
type
	InetSysInfoPtr = ^InetSysInfo;
	InetSysInfo = record
		cpuType: packed array [0..31] of char;
		osType: packed array [0..31] of char;
	end;
type
	InetMailExchangePtr = ^InetMailExchange;
	InetMailExchange = record
		preference: UInt16;
		exchange: InetDomainName;
	end;
type
	DNSQueryInfoPtr = ^DNSQueryInfo;
	DNSQueryInfo = record
		qType: UInt16;
		qClass: UInt16;
		ttl: UInt32;
		name: InetDomainName;
		responseType: UInt16;           { answer, authority, or additional}
		resourceLen: UInt16;            { actual length of array which follows}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		resourceData: packed array [0..3] of char;        { size varies}
=======
		resourceData: packed array [0..3] of char;			{  size varies }
>>>>>>> graemeg/fixes_2_2
=======
		resourceData: packed array [0..3] of char;			{  size varies }
>>>>>>> origin/fixes_2_2
=======
		resourceData: packed array [0..3] of char;        { size varies}
>>>>>>> origin/fixes_2.4
	end;
{ DNSAddress}
{
   The DNSAddress format is optional and may be used in connects,
   datagram sends, and resolve address calls.   The name takes the 
   format "somewhere.com" or "somewhere.com:portnumber" where
   the ":portnumber" is optional.   The length of this structure
   is arbitrarily limited to the overall max length of a domain
   name (255 chars), although a longer one can be use successfully
   if you use this as a template for doing so.   However, the domain name 
   is still limited to 255 characters.
}

type
	DNSAddressPtr = ^DNSAddress;
	DNSAddress = record
		fAddressType: OTAddressType;           { always AF_DNS}
		fName: InetDomainName;
	end;
{ InetInterfaceInfo}
const
	kDefaultInetInterface = -1;

const
	kInetInterfaceInfoVersion = 3;

type
	InetInterfaceInfoPtr = ^InetInterfaceInfo;
	InetInterfaceInfo = record
		fAddress: InetHost;
		fNetmask: InetHost;
		fBroadcastAddr: InetHost;
		fDefaultGatewayAddr: InetHost;
		fDNSAddr: InetHost;
		fVersion: UInt16;
		fHWAddrLen: UInt16;
		fHWAddr: UInt8Ptr;
		fIfMTU: UInt32;
		fReservedPtrs: array [0..1] of Ptr;
		fDomainName: InetDomainName;
		fIPSecondaryCount: UInt32;
		fReserved: packed array [0..251] of UInt8;
	end;
{ InetDHCPOption}
const
	kAllDHCPOptions = -1;
	kDHCPLongOption = 126;
	kDHCPLongOptionReq = 127;

type
	InetDHCPOptionPtr = ^InetDHCPOption;
	InetDHCPOption = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		fOptionTag: UInt8;
		fOptionLen: UInt8;
		fOptionValue: UInt8;
=======
		fOptionTag: SInt8;
		fOptionLen: SInt8;
		fOptionValue: SInt8;
>>>>>>> graemeg/fixes_2_2
=======
		fOptionTag: SInt8;
		fOptionLen: SInt8;
		fOptionValue: SInt8;
>>>>>>> origin/fixes_2_2
=======
		fOptionTag: UInt8;
		fOptionLen: UInt8;
		fOptionValue: UInt8;
>>>>>>> origin/fixes_2.4
		pad: SInt8
	end;
{ TCP/IP Utility Routines}

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
 *  OTInitInetAddress()   *** DEPRECATED ***
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
procedure OTInitInetAddress( var addr: InetAddress; port: InetPort; host: InetHost ); external name '_OTInitInetAddress';
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
 *  OTInitDNSAddress()   *** DEPRECATED ***
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
function OTInitDNSAddress( var addr: DNSAddress; str: CStringPtr ): OTByteCount; external name '_OTInitDNSAddress';
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
 *  OTInetStringToHost()   *** DEPRECATED ***
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
function OTInetStringToHost( str: ConstCStringPtr; var host: InetHost ): OSStatus; external name '_OTInetStringToHost';
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
 *  OTInetHostToString()   *** DEPRECATED ***
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
procedure OTInetHostToString( host: InetHost; str: CStringPtr ); external name '_OTInetHostToString';
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
 *  OTInetGetInterfaceInfo()   *** DEPRECATED ***
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
function OTInetGetInterfaceInfo( var info: InetInterfaceInfo; val: SInt32 ): OSStatus; external name '_OTInetGetInterfaceInfo';
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
 *  OTInetGetSecondaryAddresses()   *** DEPRECATED ***
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
function OTInetGetSecondaryAddresses( var addr: InetHost; var count: UInt32; val: SInt32 ): OSStatus; external name '_OTInetGetSecondaryAddresses';
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
 *  OTInetGetDHCPConfigInfo()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ InetServices & DNR Calls}

{$ifc NOT OTKERNEL}
{
   Under Carbon, OTOpenInternetServices routines take a client context pointer.  Applications may pass NULL
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
 *  OTOpenInternetServicesInContext()   *** DEPRECATED ***
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
function OTOpenInternetServicesInContext( cfig: OTConfigurationRef; oflag: OTOpenFlags; var err: OSStatus; clientContext: OTClientContextPtr ): InetSvcRef; external name '_OTOpenInternetServicesInContext';
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
 *  OTAsyncOpenInternetServicesInContext()   *** DEPRECATED ***
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
function OTAsyncOpenInternetServicesInContext( cfig: OTConfigurationRef; oflag: OTOpenFlags; upp: OTNotifyUPP; contextPtr: UnivPtr; clientContext: OTClientContextPtr ): OSStatus; external name '_OTAsyncOpenInternetServicesInContext';
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
 *  OTOpenInternetServices()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTAsyncOpenInternetServices()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ The following macro may be used by applications only.}
// #define OTOpenInternetServices(cfig, oflags, err) OTOpenInternetServicesInContext(cfig, oflags, err, NULL)
// #define OTAsyncOpenInternetServices(cfig, oflags, proc, contextPtr)  OTAsyncOpenInternetServicesInContext(cfig, oflags, proc, contextPtr, NULL)

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
 *  OTInetStringToAddress()   *** DEPRECATED ***
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
function OTInetStringToAddress( ref: InetSvcRef; name: CStringPtr; var hinfo: InetHostInfo ): OSStatus; external name '_OTInetStringToAddress';
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
 *  OTInetAddressToName()   *** DEPRECATED ***
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
function OTInetAddressToName( ref: InetSvcRef; addr: InetHost; var name: InetDomainName ): OSStatus; external name '_OTInetAddressToName';
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
 *  OTInetSysInfo()   *** DEPRECATED ***
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
function OTInetSysInfo( ref: InetSvcRef; name: CStringPtr; var sysinfo: InetSysInfo ): OSStatus; external name '_OTInetSysInfo';
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
 *  OTInetMailExchange()   *** DEPRECATED ***
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
function OTInetMailExchange( ref: InetSvcRef; name: CStringPtr; var num: UInt16; mx: InetMailExchangePtr ): OSStatus; external name '_OTInetMailExchange';
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
 *  OTInetQuery()   *** DEPRECATED ***
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
function OTInetQuery( ref: InetSvcRef; name: CStringPtr; qClass: UInt16; qType: UInt16; buf: CStringPtr; buflen: OTByteCount; var argv: UnivPtr; argvlen: OTByteCount; flags: OTFlags ): OSStatus; external name '_OTInetQuery';
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

{ ***** AppleTalk *****}
{ Shared library prefixes}


const
	kATalkVersion = '1.1';
const
	kATalkPrefix = 'ot:atlk$';
const
	kATBinderID = 'ot:atbd$';

{******************************************************************************
** Module definitions
*******************************************************************************}
{ XTI Levels}

const
	ATK_DDP = FourCharCode('DDP ');
	ATK_AARP = FourCharCode('AARP');
	ATK_ATP = FourCharCode('ATP ');
	ATK_ADSP = FourCharCode('ADSP');
	ATK_ASP = FourCharCode('ASP ');
	ATK_PAP = FourCharCode('PAP ');
	ATK_NBP = FourCharCode('NBP ');
	ATK_ZIP = FourCharCode('ZIP ');

{ Module Names}


const
	kDDPName = 'ddp';
const
	kATPName = 'atp';
const
	kADSPName = 'adsp';
const
	kASPName = 'asp';
const
	kPAPName = 'pap';
const
	kNBPName = 'nbp';
const
	kZIPName = 'zip';
const
	kLTalkName = 'ltlk';
const
	kLTalkAName = 'ltlkA';
const
	kLTalkBName = 'ltlkB';

{
   Protocol-specific Options
   NOTE:
   All Protocols support OPT_CHECKSUM (Value is (unsigned long)T_YES/T_NO)
   ATP supports OPT_RETRYCNT (# Retries, 0 = try once) and
                OPT_INTERVAL (# Milliseconds to wait)
}

const
	DDP_OPT_HOPCOUNT = $2100; { DDP UnitDataReq Only - set hop count, Value is (unsigned long)  hop count   }

const
	DDP_OPT_CHECKSUM = $0600;
	DDP_OPT_SRCADDR = $2101; { DDP UnitDataReq Only - set src address, Value is DDPAddress }
	ATP_OPT_REPLYCNT = $2110; { AppleTalk - ATP Resp Pkt Ct Type, Value is (unsigned long)  pkt count }
	ATP_OPT_DATALEN = $2111; { AppleTalk - ATP Pkt Data Len Type, Value is (unsigned long) length }
	ATP_OPT_RELTIMER = $2112; { AppleTalk - ATP Release Timer Type, Value is (unsigned long) timer, (See Inside AppleTalk, second edition }
	ATP_OPT_TRANID = $2113; { Value is (unsigned long) Boolean, Used to request Transaction ID, Returned with Transaction ID on requests }
	PAP_OPT_OPENRETRY = $2120; { AppleTalk - PAP OpenConn Retry count, Value is (unsigned long) T_YES/T_NO }

{ Protocol-Specific Events}

{
   If you send the IOCTL: OTIoctl(I_OTGetMiscellaneousEvents, 1),
   you will receive the T_ATALKxxx events on your endpoint.
   NOTE: The endpoint does not need to be bound.
}

const
	kAppleTalkEvent = kPROTOCOLEVENT or $00010000;
	T_GETMYZONECOMPLETE = kAppleTalkEvent + 1;
	T_GETLOCALZONESCOMPLETE = kAppleTalkEvent + 2;
	T_GETZONELISTCOMPLETE = kAppleTalkEvent + 3;
	T_GETATALKINFOCOMPLETE = kAppleTalkEvent + 4;
	T_ATALKROUTERDOWNEVENT = kAppleTalkEvent + 51; { No routers have been seen for a while.  If the cookie is NULL, all routers are gone.  Otherwise, there is still an ARA router hanging around being used, and only the local cable has been  timed out.}
	T_ATALKROUTERUPEVENT = kAppleTalkEvent + 52; { We didn't have a router, but now one has come up. Cookie is NULL for a normal router coming up, non-NULL for an ARA router coming on-line}
	T_ATALKZONENAMECHANGEDEVENT = kAppleTalkEvent + 53; { A Zone name change was issued from the router, so our AppleTalk Zone has changed.}
	T_ATALKCONNECTIVITYCHANGEDEVENT = kAppleTalkEvent + 54; { An ARA connection was established (cookie != NULL), or was disconnected (cookie == NULL)}
	T_ATALKINTERNETAVAILABLEEVENT = kAppleTalkEvent + 55; { A router has appeared, and our address is in the startup range.  Cookie is hi/lo of new cable range.}
	T_ATALKCABLERANGECHANGEDEVENT = kAppleTalkEvent + 56; { A router has appeared, and it's incompatible with our current address.  Cookie is hi/lo of new cable range.}

const
	T_ATALKBADROUTEREVENT = kAppleTalkEvent + 70; { A bad router has appeared/disappeared on our network.}
	T_ALLNODESTAKENEVENT = kAppleTalkEvent + 71;
	T_FIXEDNODETAKENEVENT = kAppleTalkEvent + 72;
	T_MPPCOMPATCFIGEVENT = kAppleTalkEvent + 73;
	T_FIXEDNODEBADEVENT = kAppleTalkEvent + 74;

const
	kAllATalkRoutersDown = 0;    { This indicates that all routers are offline}
	kLocalATalkRoutersDown = -1;  { This indicates that all local routers went offline, but an ARA router is still active}
	kARARouterDisconnected = -2;   { This indicates that ARA was disconnected, do it's router went offline, and we have no local routers to fall back onto.}

const
	kARARouterOnline = -1;  { We had no local routers, but an ARA router is now online.}
	kATalkRouterOnline = 0;    { We had no routers, but a local router is now online}
	kLocalATalkRouterOnline = -2;   { We have an ARA router, but now we've seen a local router as well}

// #define IsAppleTalkEvent(x)         ((x) & 0xffff0000) == kAppleTalkEvent)
{ Protocol-specific IOCTLs}

const
	ATALK_IOC_FULLSELFSEND		= $542F;						{  Turn on/off full self-send (it's automatic for non-backward-compatible links) }
	ADSP_IOC_FORWARDRESET		= $543C;						{  ADSP Forward Reset }

{ Protocol-specific constants}

{ ECHO}

const
	kECHO_TSDU = 585;   { Max. # of data bytes.}

{ NBP}

const
	kNBPMaxNameLength = 32;
	kNBPMaxTypeLength = 32;
	kNBPMaxZoneLength = 32;
	kNBPSlushLength = 9;    { Extra space for @, : and a few escape chars}
	kNBPMaxEntityLength = kNBPMaxNameLength + kNBPMaxTypeLength + kNBPMaxZoneLength + 3;
	kNBPEntityBufferSize = kNBPMaxNameLength + kNBPMaxTypeLength + kNBPMaxZoneLength + kNBPSlushLength;
	kNBPWildCard = $3D; { NBP name and type match anything '='}
	kNBPImbeddedWildCard = $C5; { NBP name and type match some 'Å'}
	kNBPDefaultZone = $2A;  { NBP default zone '*'}

{ ZIP}

const
	kZIPMaxZoneLength = kNBPMaxZoneLength;

const
	kDDPAddressLength = 8;    { value to use in netbuf.len field, Maximum length of AppleTalk address}
	kNBPAddressLength = kNBPEntityBufferSize;
	kAppleTalkAddressLength = kDDPAddressLength + kNBPEntityBufferSize;


// #define OTCopyDDPAddress(addr, dest)               \
//   {                                               \
//       ((UInt32*)(dest))[0] = ((UInt32*)(addr))[0];    \
//       ((UInt32*)(dest))[1] = ((UInt32*)(addr))[1];    \
//   }

{******************************************************************************
** CLASS TAppleTalkServices
*******************************************************************************}
{$ifc NOT OTKERNEL}
{
   Define the ATSvcRef type.  This type needs special
   processing because in C++ it's a subclass of TProvider.
   See the definition of TEndpointRef in "OpenTransport.h"
   for the logic behind this definition.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ATSvcRef = ^SInt32; { an opaque type }
=======
	ATSvcRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	ATSvcRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	ATSvcRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	ATSvcRefPtr = ^ATSvcRef;

const
	kDefaultAppleTalkServicesPath = -3;
{
   Under Carbon, OpenAppleTalkServices routines take a client context pointer.  Applications may pass NULL
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
 *  OTAsyncOpenAppleTalkServicesInContext()   *** DEPRECATED ***
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
function OTAsyncOpenAppleTalkServicesInContext( cfig: OTConfigurationRef; flags: OTOpenFlags; proc: OTNotifyUPP; contextPtr: UnivPtr; clientContext: OTClientContextPtr ): OSStatus; external name '_OTAsyncOpenAppleTalkServicesInContext';
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
 *  OTOpenAppleTalkServicesInContext()   *** DEPRECATED ***
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
function OTOpenAppleTalkServicesInContext( cfig: OTConfigurationRef; flags: OTOpenFlags; var err: OSStatus; clientContext: OTClientContextPtr ): ATSvcRef; external name '_OTOpenAppleTalkServicesInContext';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

=======
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
 *  OTAsyncOpenAppleTalkServices()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  OTOpenAppleTalkServices()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{ The following macro may be used by applications only.}
// #define OTOpenAppleTalkServices(cfig, oflags, err) OTOpenAppleTalkServicesInContext(cfig, oflags, err, NULL)
// #define OTAsyncOpenAppleTalkServices(cfig, oflags, proc, contextPtr) OTAsyncOpenAppleTalkServicesInContext(cfig, oflags, proc, contextPtr, NULL)

{ Get the zone associated with the ATSvcRef}
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
 *  OTATalkGetMyZone()   *** DEPRECATED ***
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
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function OTATalkGetMyZone( ref: ATSvcRef; var zone: TNetbuf ): OSStatus; external name '_OTATalkGetMyZone';
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
   Get the list of available zones associated with the local cable
   of the ATSvcRef
}
{
 *  OTATalkGetLocalZones()   *** DEPRECATED ***
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
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function OTATalkGetLocalZones( ref: ATSvcRef; var zones: TNetbuf ): OSStatus; external name '_OTATalkGetLocalZones';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Get the list of all zones on the internet specified by the ATSvcRef}
{
 *  OTATalkGetZoneList()   *** DEPRECATED ***
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
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function OTATalkGetZoneList( ref: ATSvcRef; var zones: TNetbuf ): OSStatus; external name '_OTATalkGetZoneList';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Stores an AppleTalkInfo structure into the TNetbuf (see later in this file)}
{
 *  OTATalkGetInfo()   *** DEPRECATED ***
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
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   not available
 }
function OTATalkGetInfo( ref: ATSvcRef; var info: TNetbuf ): OSStatus; external name '_OTATalkGetInfo';
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

{ AppleTalk Addressing}
{
   The NBPEntity structure is used to manipulate NBP names without regard
   to issues of what kind of "special" characters are in the name.
   
   When stored as an address in an NBPAddress or DDPNBPAddress, they are 
   stored as a character string, which is currently just ASCII, but in the
   future may be UniChar, or some other internationalizable scripting set.
   The string following an NBPAddress or DDPNBPAddress is intended to be
   suitable for showing to users, whereas NBPEntity is not.
   WARNING: NBPAddress and DDPNBPAddress structures do not "know" the length
   of the address.  That must have been obtained as part of a Lookup or
   ResolveAddress call.
}

const
	AF_ATALK_FAMILY = $0100;
	AF_ATALK_DDP = $0100;
	AF_ATALK_DDPNBP = AF_ATALK_FAMILY + 1;
	AF_ATALK_NBP = AF_ATALK_FAMILY + 2;
	AF_ATALK_MNODE = AF_ATALK_FAMILY + 3;

type
	NBPEntityPtr = ^NBPEntity;
	NBPEntity = record
		fEntity: packed array [0..99] of UInt8; {one extra pad byte}
	end;
type
	DDPAddressPtr = ^DDPAddress;
	DDPAddress = record
		fAddressType: OTAddressType;           { One of the enums above}
		fNetwork: UInt16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		fNodeID: UInt8;
		fSocket: UInt8;
		fDDPType: UInt8;
		fPad: UInt8;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		fNodeID: SInt8;
		fSocket: SInt8;
		fDDPType: SInt8;
		fPad: SInt8;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	NBPAddressPtr = ^NBPAddress;
	NBPAddress = record
		fAddressType: OTAddressType;           { One of the enums above}
		fNBPNameBuffer: packed array [0..104] of UInt8;
	end;
type
	DDPNBPAddressPtr = ^DDPNBPAddress;
	DDPNBPAddress = record
		fAddressType: OTAddressType;           { One of the enums above}
		fNetwork: UInt16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		fNodeID: UInt8;
		fSocket: UInt8;
		fDDPType: UInt8;
		fPad: UInt8;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		fNodeID: SInt8;
		fSocket: SInt8;
		fDDPType: SInt8;
		fPad: SInt8;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
		fNBPNameBuffer: packed array [0..104] of UInt8;
	end;
{ These are some utility routines for dealing with NBP and DDP addresses. }

{ Functions to initialize the various AppleTalk Address types}
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
 *  OTInitDDPAddress()   *** DEPRECATED ***
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
procedure OTInitDDPAddress( var addr: DDPAddress; net: UInt16; node: ByteParameter; socket: ByteParameter; ddpType: ByteParameter ); external name '_OTInitDDPAddress';
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
 *  OTInitNBPAddress()   *** DEPRECATED ***
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
function OTInitNBPAddress( var addr: NBPAddress; name: ConstCStringPtr ): OTByteCount; external name '_OTInitNBPAddress';
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
 *  OTInitDDPNBPAddress()   *** DEPRECATED ***
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
function OTInitDDPNBPAddress( var addr: DDPNBPAddress; name: ConstCStringPtr; net: UInt16; node: ByteParameter; socket: ByteParameter; ddpType: ByteParameter ): OTByteCount; external name '_OTInitDDPNBPAddress';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Compare 2 DDP addresses for equality}
{
 *  OTCompareDDPAddresses()   *** DEPRECATED ***
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
function OTCompareDDPAddresses( const (*var*) addr1: DDPAddress; const (*var*) addr2: DDPAddress ): Boolean; external name '_OTCompareDDPAddresses';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Init an NBPEntity to a NULL name}
{
 *  OTInitNBPEntity()   *** DEPRECATED ***
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
procedure OTInitNBPEntity( var entity: NBPEntity ); external name '_OTInitNBPEntity';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Get the length an NBPEntity would have when stored as an address}
{
 *  OTGetNBPEntityLengthAsAddress()   *** DEPRECATED ***
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
function OTGetNBPEntityLengthAsAddress( const (*var*) entity: NBPEntity ): OTByteCount; external name '_OTGetNBPEntityLengthAsAddress';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Store an NBPEntity into an address buffer}
{
 *  OTSetAddressFromNBPEntity()   *** DEPRECATED ***
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
function OTSetAddressFromNBPEntity( nameBuf: UInt8Ptr; const (*var*) entity: NBPEntity ): OTByteCount; external name '_OTSetAddressFromNBPEntity';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Create an address buffer from a string (use -1 for len to use strlen)}
{
 *  OTSetAddressFromNBPString()   *** DEPRECATED ***
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
function OTSetAddressFromNBPString( addrBuf: UInt8Ptr; name: ConstCStringPtr; len: SInt32 ): OTByteCount; external name '_OTSetAddressFromNBPString';
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
   Create an NBPEntity from an address buffer. False is returned if
     the address was truncated.
}
{
 *  OTSetNBPEntityFromAddress()   *** DEPRECATED ***
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
function OTSetNBPEntityFromAddress( var entity: NBPEntity; addrBuf: UInt8Ptr; len: OTByteCount ): Boolean; external name '_OTSetNBPEntityFromAddress';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Routines to set a piece of an NBP entity from a character string}
{
 *  OTSetNBPName()   *** DEPRECATED ***
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
function OTSetNBPName( var entity: NBPEntity; name: ConstCStringPtr ): Boolean; external name '_OTSetNBPName';
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
 *  OTSetNBPType()   *** DEPRECATED ***
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
function OTSetNBPType( var entity: NBPEntity; typeVal: ConstCStringPtr ): Boolean; external name '_OTSetNBPType';
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
 *  OTSetNBPZone()   *** DEPRECATED ***
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
function OTSetNBPZone( var entity: NBPEntity; zone: ConstCStringPtr ): Boolean; external name '_OTSetNBPZone';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ Routines to extract pieces of an NBP entity}
{
 *  OTExtractNBPName()   *** DEPRECATED ***
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
procedure OTExtractNBPName( const (*var*) entity: NBPEntity; name: CStringPtr ); external name '_OTExtractNBPName';
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
 *  OTExtractNBPType()   *** DEPRECATED ***
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
procedure OTExtractNBPType( const (*var*) entity: NBPEntity; typeVal: CStringPtr ); external name '_OTExtractNBPType';
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
 *  OTExtractNBPZone()   *** DEPRECATED ***
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
procedure OTExtractNBPZone( const (*var*) entity: NBPEntity; zone: CStringPtr ); external name '_OTExtractNBPZone';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_4,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/fixes_2_2


{ AppleTalkInfo as used by the OTGetATalkInfo function}

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
	AppleTalkInfoPtr = ^AppleTalkInfo;
	AppleTalkInfo = record
		fOurAddress: DDPAddress;            { Our DDP address (network # & node)}
		fRouterAddress: DDPAddress;         { The address of a router on our cable}
		fCableRange: array [0..1] of UInt16;					{  The current cable range }
		fFlags: UInt16;                 { See below}
	end;
{ For the fFlags field in AppleTalkInfo}
const
	kATalkInfoIsExtended = $0001; { This is an extended (phase 2) network}
	kATalkInfoHasRouter = $0002; { This cable has a router}
	kATalkInfoOneZone = $0004; { This cable has only one zone}

{ ***** Ethernet *****}

{ Interface option flags}

{ Ethernet framing options}

const
	kOTFramingEthernet = $01;
	kOTFramingEthernetIPX = $02;
	kOTFraming8023 = $04;
	kOTFraming8022 = $08;

{
   These are obsolete and will be going away in OT 1.5.
   Hmmm, OT 1.5 got cancelled.  The status of these options
   is uncertain.
}

{ RawMode options}

const
	kOTRawRcvOn = 0;
	kOTRawRcvOff = 1;
	kOTRawRcvOnWithTimeStamp = 2;

const
	DL_PROMISC_OFF = 0;     { OPT_SETPROMISCUOUS value}

{ Module definitions}

{ Module IDs}

const
	kT8022ModuleID = 7100;
	kEnetModuleID = 7101;
	kTokenRingModuleID = 7102;
	kFDDIModuleID = 7103;

{ Module Names}


const
	kEnet8022Name = 'enet8022x';
const
	kEnetName = 'enet';
const
	kFastEnetName = 'fenet';
const
	kTokenRingName = 'tokn';
const
	kFDDIName = 'fddi';
const
	kIRTalkName = 'irtlk';
const
	kSMDSName = 'smds';
const
	kATMName = 'atm';
const
	kT8022Name = 'tpi8022x';
const
	kATMSNAPName = 'atmsnap';
const
	kFireWireName = 'firewire';
const
	kFibreChannelName = 'fibre';

{ Address Family}

const
	AF_8022 = 8200;  { Our 802.2 generic address family}

{ XTI Levels}

const
	LNK_ENET = FourCharCode('ENET');
	LNK_TOKN = FourCharCode('TOKN');
	LNK_FDDI = FourCharCode('FDDI');
	LNK_TPI = FourCharCode('LTPI');

{ Options}

const
	OPT_ADDMCAST = $1000;
	OPT_DELMCAST = $1001;
	OPT_RCVPACKETTYPE = $1002;
	OPT_RCVDESTADDR = $1003;
	OPT_SETRAWMODE = $1004;
	OPT_SETPROMISCUOUS = $1005;

type
	OTPacketType = UInt32;
const
	kETypeStandard = 0;
	kETypeMulticast = 1;
	kETypeBroadcast = 2;
	kETRawPacketBit = $80000000;
	kETTimeStampBit = $40000000;

{ Link related constants}

const
	kMulticastLength = 6;    { length of an ENET hardware addressaddress}
	k48BitAddrLength = 6;
	k8022DLSAPLength = 2;    { The protocol type is our DLSAP}
	k8022SNAPLength = 5;
	kEnetAddressLength = k48BitAddrLength + k8022DLSAPLength; { length of an address field used by the ENET enpoint}
                                        {    = k48BitAddrLength + sizeof(protocol type)}
	kSNAPSAP = $00AA; { Special DLSAPS for ENET}
	kIPXSAP = $00FF;
	kMax8022SAP = $00FE;
	k8022GlobalSAP = $00FF;
	kMinDIXSAP = 1501;
	kMaxDIXSAP = $FFFF;

{ Generic Address Structure}

type
	T8022AddressPtr = ^T8022Address;
	T8022Address = record
		fAddrFamily: OTAddressType;
		fHWAddr: packed array [0..5] of UInt8;
		fSAP: UInt16;
		fSNAP: packed array [0..4] of UInt8;
	end;
const
	k8022BasicAddressLength = SizeOf(OTAddressType) + k48BitAddrLength + sizeof(UInt16);
	k8022SNAPAddressLength = SizeOf(OTAddressType) + k48BitAddrLength + sizeof(UInt16) + k8022SNAPLength;

{ Some helpful stuff for dealing with 48 bit addresses}


// #define OTCompare48BitAddresses(p1, p2)                                                        \
//   (*(const UInt32*)((const UInt8*)(p1)) == *(const UInt32*)((const UInt8*)(p2)) &&        \
//    *(const UInt16*)(((const UInt8*)(p1))+4) == *(const UInt16*)(((const UInt8*)(p2))+4) )
// 
// #define OTCopy48BitAddress(p1, p2)                                             \
//   (*(UInt32*)((UInt8*)(p2)) = *(const UInt32*)((const UInt8*)(p1)),           \
//    *(UInt16*)(((UInt8*)(p2))+4) = *(const UInt16*)(((const UInt8*)(p1))+4) )
// 
// #define OTClear48BitAddress(p1)                                                 \
//   (*(UInt32*)((UInt8*)(p1)) = 0,                                              \
//    *(UInt16*)(((UInt8*)(p1))+4) = 0 )
// 
// #define OTCompare8022SNAP(p1, p2)                                                      \
//   (*(const UInt32*)((const UInt8*)(p1)) == *(const UInt32*)((const UInt8*)(p2)) &&    \
//    *(((const UInt8*)(p1))+4) == *(((const UInt8*)(p2))+4) )
// 
// #define OTCopy8022SNAP(p1, p2)                                               \
//   (*(UInt32*)((UInt8*)(p2)) = *(const UInt32*)((const UInt8*)(p1)),       \
//    *(((UInt8*)(p2))+4) = *(((const UInt8*)(p1))+4) )
// 
// #define OTIs48BitBroadcastAddress(p1)                   \
//   (*(UInt32*)((UInt8*)(p1)) == 0xffffffff &&          \
//    *(UInt16*)(((UInt8*)(p1))+4) == 0xffff )
// 
// #define OTSet48BitBroadcastAddress(p1)                   \
//   (*(UInt32*)((UInt8*)(p1)) = 0xffffffff,             \
//    *(UInt16*)(((UInt8*)(p1))+4) = 0xffff )
// 
// #define OTIs48BitZeroAddress(p1)              \
//   (*(UInt32*)((UInt8*)(p1)) == 0 &&           \
//    *(UInt16*)(((UInt8*)(p1))+4) == 0 )

{ Link related constants}

const
	kEnetPacketHeaderLength = (2 * k48BitAddrLength) + k8022DLSAPLength;
	kEnetTSDU = 1514; { The TSDU for ethernet.}
	kTokenRingTSDU = 4458; { The TSDU for TokenRing.}
	kFDDITSDU = 4458; { The TSDU for FDDI. }
	k8022SAPLength = 1;
	k8022BasicHeaderLength = 3;    { define the length of the header portion of an 802.2 packet.}
                                        { = SSAP+DSAP+ControlByte}
	k8022SNAPHeaderLength = k8022SNAPLength + k8022BasicHeaderLength;

{******************************************************************************
** Address Types recognized by the Enet DLPI
*******************************************************************************}
type
	EAddrType = UInt32;
const
	keaStandardAddress = 0;
	keaMulticast = 1;
	keaBroadcast = 2;
	keaBadAddress = 3;
	keaRawPacketBit = $80000000;
	keaTimeStampBit = $40000000;

{ Packet Header Structures}

type
	EnetPacketHeaderPtr = ^EnetPacketHeader;
	EnetPacketHeader = packed record
		fDestAddr: packed array [0..5] of UInt8;
		fSourceAddr: packed array [0..5] of UInt8;
		fProto: UInt16;
	end;
type
	T8022HeaderPtr = ^T8022Header;
	T8022Header = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		fDSAP: UInt8;
		fSSAP: UInt8;
		fCtrl: UInt8;
=======
		fDSAP: SInt8;
		fSSAP: SInt8;
		fCtrl: SInt8;
>>>>>>> graemeg/fixes_2_2
=======
		fDSAP: SInt8;
		fSSAP: SInt8;
		fCtrl: SInt8;
>>>>>>> origin/fixes_2_2
=======
		fDSAP: UInt8;
		fSSAP: UInt8;
		fCtrl: UInt8;
>>>>>>> origin/fixes_2.4
		pad: SInt8
	end;
type
	T8022SNAPHeaderPtr = ^T8022SNAPHeader;
	T8022SNAPHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		fDSAP: UInt8;
		fSSAP: UInt8;
		fCtrl: UInt8;
		fSNAP0: UInt8;
		fSNAP1: UInt8;
		fSNAP2: UInt8;
		fSNAP3: UInt8;
		fSNAP4: UInt8;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
		fDSAP: SInt8;
		fSSAP: SInt8;
		fCtrl: SInt8;
		fSNAP0: SInt8;
		fSNAP1: SInt8;
		fSNAP2: SInt8;
		fSNAP3: SInt8;
		fSNAP4: SInt8;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
type
	T8022FullPacketHeaderPtr = ^T8022FullPacketHeader;
	T8022FullPacketHeader = record
		fEnetPart: EnetPacketHeader;
		f8022Part: T8022SNAPHeader;
	end;
{ Define the lengths of the structures above}
const
	kT8022HeaderLength = 3;
	kT8022SNAPHeaderLength = 3 + k8022SNAPLength;
	kT8022FullPacketHeaderLength = kEnetPacketHeaderLength + kT8022SNAPHeaderLength;

{ ***** Serial *****}

{ Module Definitions}

{ XTI Level}

const
	COM_SERIAL = FourCharCode('SERL');

{ Version Number}


const
	kSerialABVersion = '1.1.1';

{ Module Names}


const
	kSerialABName = 'serialAB';
const
	kSerialName = 'serial';
const
	kSerialPortAName = 'serialA';
const
	kSerialPortBName = 'serialB';

const
	kSerialABModuleID = 7200;

const
	kOTSerialFramingAsync = $01; { Support Async serial mode         }
	kOTSerialFramingHDLC = $02; { Support HDLC synchronous serial mode   }
	kOTSerialFramingSDLC = $04; { Support SDLC synchronous serial mode   }
	kOTSerialFramingAsyncPackets = $08; { Support Async "packet" serial mode }
	kOTSerialFramingPPP = $10;  { Port does its own PPP framing - wants unframed packets from PPP }

{ IOCTL Calls for Serial Drivers}

const
	I_SetSerialDTR = $5500;						{  Set DTR (0 = off, 1 = on) }
	kOTSerialSetDTROff = 0;
	kOTSerialSetDTROn = 1;
	I_SetSerialBreak = $5501; { Send a break on the line - kOTSerialSetBreakOff = off, kOTSerialSetBreakOn = on,}
                                        { Any other number is the number of milliseconds to leave break on, then}
                                        { auto shutoff}
	kOTSerialSetBreakOn = $FFFFFFFF;
	kOTSerialSetBreakOff = 0;
	I_SetSerialXOffState = $5502; { Force XOFF state - 0 = Unconditionally clear XOFF state, 1 = unconditionally set it}
	kOTSerialForceXOffTrue = 1;
	kOTSerialForceXOffFalse = 0;
	I_SetSerialXOn = $5503; { Send an XON character, 0 = send only if in XOFF state, 1 = send always}
	kOTSerialSendXOnAlways = 1;
	kOTSerialSendXOnIfXOffTrue = 0;
	I_SetSerialXOff = $5504; { Send an XOFF character, 0 = send only if in XON state, 1 = send always}
	kOTSerialSendXOffAlways = 1;
	kOTSerialSendXOffIfXOnTrue = 0;

{ Option Management for Serial Drivers}

{
   These options are all 4-byte values.
   BaudRate is the baud rate.
   DataBits is the number of data bits.
   StopBits is the number of stop bits times 10.
   Parity is an enum
}

const
	SERIAL_OPT_BAUDRATE = $0100; { UInt32 }
	SERIAL_OPT_DATABITS = $0101; { UInt32 }
	SERIAL_OPT_STOPBITS = $0102; { UInt32 10, 15 or 20 for 1, 1.5 or 2    }
	SERIAL_OPT_PARITY = $0103; { UInt32 }
	SERIAL_OPT_STATUS = $0104; { UInt32 }
                                        { The "Status" option is a 4-byte value option that is ReadOnly}
                                        { It returns a bitmap of the current serial status}
	SERIAL_OPT_HANDSHAKE = $0105; { UInt32 }
                                        { The "Handshake" option defines what kind of handshaking the serial port}
                                        { will do for line flow control.  The value is a 32-bit value defined by}
                                        { the function or macro SerialHandshakeData below.}
                                        { For no handshake, or CTS handshake, the onChar and offChar parameters}
                                        { are ignored.}
	SERIAL_OPT_RCVTIMEOUT = $0106; { The "RcvTimeout" option defines how long the receiver should wait before delivering}
                                        { less than the RcvLoWat number of characters.  If RcvLoWat is 0, then the RcvTimeout}
                                        { is how long a gap to wait for before delivering characters.  This parameter is advisory,}
                                        { and serial drivers are free to deliver data whenever they deem it convenient.  For instance,}
                                        { many serial drivers will deliver data whenever 64 bytes have been received, since 64 bytes}
                                        { is the smallest STREAMS buffer size. Keep in mind that timeouts are quantized, so be sure to}
                                        { look at the return value of the option to determine what it was negotiated to.}
	SERIAL_OPT_ERRORCHARACTER = $0107; { This option defines how characters with parity errors are handled.}
                                        { A 0 value will disable all replacement.  A single character value in the low}
                                        { byte designates the replacement character.  When characters are received with a }
                                        { parity error, they are replaced by this specified character.  If a valid incoming}
                                        { character matches the replacement character, then the received character's msb is}
                                        { cleared. For this situation, the alternate character is used, if specified in bits}
                                        { 8 through 15 of the option long, with 0xff being place in bits 16 through 23.}
                                        { Whenever a valid character is received that matches the first replacement character,}
                                        { it is replaced with this alternate character.}
	SERIAL_OPT_EXTCLOCK = $0108; { The "ExtClock" requests an external clock.  A 0-value turns off external clocking.}
                                        { Any other value is a requested divisor for the external clock.  Be aware that}
                                        { not all serial implementations support an external clock, and that not all}
                                        { requested divisors will be supported if it does support an external clock.}
	SERIAL_OPT_BURSTMODE = $0109; { The "BurstMode" option informs the serial driver that it should continue looping,}
                                        { reading incoming characters, rather than waiting for an interrupt for each character.}
                                        { This option may not be supported by all Serial driver}
	SERIAL_OPT_DUMMY = $010A; { placeholder}

type
	ParityOptionValues = UInt32;
const
	kOTSerialNoParity = 0;
	kOTSerialOddParity = 1;
	kOTSerialEvenParity = 2;

const
	kOTSerialSwOverRunErr = $01;
	kOTSerialBreakOn = $08;
	kOTSerialParityErr = $10;
	kOTSerialOverrunErr = $20;
	kOTSerialFramingErr = $40;
	kOTSerialXOffSent = $00010000;
	kOTSerialDTRNegated = $00020000;
	kOTSerialCTLHold = $00040000;
	kOTSerialXOffHold = $00080000;
	kOTSerialOutputBreakOn = $01000000;

const
	kOTSerialXOnOffInputHandshake = 1;    { Want XOn/XOff handshake for incoming characters    }
	kOTSerialXOnOffOutputHandshake = 2;   { Want XOn/XOff handshake for outgoing characters    }
	kOTSerialCTSInputHandshake = 4;    { Want CTS handshake for incoming characters     }
	kOTSerialDTROutputHandshake = 8;     { Want DTR handshake for outoing characters   }


// #define OTSerialHandshakeData(type, onChar, offChar)    \
//       ((((UInt32)type) << 16) | (((UInt32)onChar) << 8) | offChar)
// 
// #define OTSerialSetErrorCharacter(rep) \
//   ((rep) & 0xff)
// 
// #define OTSerialSetErrorCharacterWithAlternate(rep, alternate)  \
//   ((((rep) & 0xff) | (((alternate) & 0xff) << 8)) | 0x80000000L)


{ Default attributes for the serial ports}

const
	kOTSerialDefaultBaudRate = 19200;
	kOTSerialDefaultDataBits = 8;
	kOTSerialDefaultStopBits = 10;
	kOTSerialDefaultParity = kOTSerialNoParity;
	kOTSerialDefaultHandshake = 0;
	kOTSerialDefaultOnChar = 17;
	kOTSerialDefaultOffChar = 19;
	kOTSerialDefaultSndBufSize = 1024;
	kOTSerialDefaultRcvBufSize = 1024;
	kOTSerialDefaultSndLoWat = 96;
	kOTSerialDefaultRcvLoWat = 1;
	kOTSerialDefaultRcvTimeout = 10;

{ ***** ISDN *****}

{ Module Definitions}

{ XTI Level}

const
	COM_ISDN = FourCharCode('ISDN');

{ Module Names}

const
	kISDNName = 'isdn';
const
	kISDNModuleID = 7300;


{ ISDN framing methods, set using the I_OTSetFramingType IOCTL}

const
	kOTISDNFramingTransparentSupported = $0010; { Support Transparent mode    }
	kOTISDNFramingHDLCSupported = $0020; { Support HDLC Synchronous mode  }
	kOTISDNFramingV110Supported = $0040; { Support V.110 Asynchronous mode    }
	kOTISDNFramingV14ESupported = $0080; { Support V.14 Asynchronous mode     }

{ Miscellaneous equates}

{ Disconnect reason codes (from Q.931)}

const
	kOTISDNUnallocatedNumber = 1;
	kOTISDNNoRouteToSpecifiedTransitNetwork = 2;
	kOTISDNNoRouteToDestination = 3;
	kOTISDNChannelUnacceptable = 6;
	kOTISDNNormal = 16;
	kOTISDNUserBusy = 17;
	kOTISDNNoUserResponding = 18;
	kOTISDNNoAnswerFromUser = 19;
	kOTISDNCallRejected = 21;
	kOTISDNNumberChanged = 22;
	kOTISDNNonSelectedUserClearing = 26;
	kOTISDNDestinationOutOfOrder = 27;
	kOTISDNInvalidNumberFormat = 28;
	kOTISDNFacilityRejected = 29;
	kOTISDNNormalUnspecified = 31;
	kOTISDNNoCircuitChannelAvailable = 34;
	kOTISDNNetworkOutOfOrder = 41;
	kOTISDNSwitchingEquipmentCongestion = 42;
	kOTISDNAccessInformationDiscarded = 43;
	kOTISDNRequestedCircuitChannelNotAvailable = 44;
	kOTISDNResourceUnavailableUnspecified = 45;
	kOTISDNQualityOfServiceUnvailable = 49;
	kOTISDNRequestedFacilityNotSubscribed = 50;
	kOTISDNBearerCapabilityNotAuthorized = 57;
	kOTISDNBearerCapabilityNotPresentlyAvailable = 58;
	kOTISDNCallRestricted = 59;
	kOTISDNServiceOrOptionNotAvilableUnspecified = 63;
	kOTISDNBearerCapabilityNotImplemented = 65;
	kOTISDNRequestedFacilityNotImplemented = 69;
	kOTISDNOnlyRestrictedDigitalBearer = 70;
	kOTISDNServiceOrOptionNotImplementedUnspecified = 79;
	kOTISDNCallIdentityNotUsed = 83;
	kOTISDNCallIdentityInUse = 84;
	kOTISDNNoCallSuspended = 85;
	kOTISDNCallIdentityCleared = 86;
	kOTISDNIncompatibleDestination = 88;
	kOTISDNInvalidTransitNetworkSelection = 91;
	kOTISDNInvalidMessageUnspecified = 95;
	kOTISDNMandatoryInformationElementIsMissing = 96;
	kOTISDNMessageTypeNonExistentOrNotImplemented = 97;
	kOTISDNInterworkingUnspecified = 127;

{ OTISDNAddress}

{
   The OTISDNAddress has the following format:
   "xxxxxx*yy"
   where 'x' is the phone number and 'y' is the sub address (if available
   in the network. The characters are coded in ASCII (IA5), and valid
   characters are: '0'-'9','#','*'.
   The max length of the each phone number is 21 characters (?) and the max
   subaddress length is network dependent.
   When using bonded channels the phone numbers are separated by '&'.
   The X.25 user data is preceded by '@'.
}

const
	kAF_ISDN = $2000;

{ BSD value for AF_ISDN conflicts, so OT Carbon clients must use kAF_ISDN}
const
	kOTISDNMaxPhoneSize = 32;
	kOTISDNMaxSubSize = 4;

type
	OTISDNAddressPtr = ^OTISDNAddress;
	OTISDNAddress = record
		fAddressType: OTAddressType;
		fPhoneLength: UInt16;
		fPhoneNumber: packed array [0..36] of char;
	end;
{ IOCTL Calls for ISDN}
{ ISDN shares the same ioctl space as serial.}

const
	MIOC_ISDN = 85;							{  ISDN ioctl() cmds  }

	I_OTISDNAlerting = $5564;						{  Send or receive an ALERTING message }
	I_OTISDNSuspend = $5565;						{  Send a SUSPEND message }
																{  The parameter is the Call Identity (Maximum 8 octets) }
	I_OTISDNSuspendAcknowledge = $5566;						{  Receive a SUSPEND ACKNOWLEDGE message }
	I_OTISDNSuspendReject = $5567;						{  Receive a SUSPEND REJECT message }
	I_OTISDNResume = $5568;						{  Send a RESUME message }
																{  The parameter is the Call Identity (Maximum 8 octets) }
	I_OTISDNResumeAcknowledge = $5569;						{  Receive a RESUME ACKNOWLEDGE message }
	I_OTISDNResumeReject = $556A;						{  Receive a RESUME REJECT message }
	I_OTISDNFaciltity = $556B;						{  Send or receive a FACILITY message }

	{  Connect user data size }

	kOTISDNMaxUserDataSize = 32;

{ Option management calls for ISDN}

const
	ISDN_OPT_COMMTYPE = $0200;
	ISDN_OPT_FRAMINGTYPE = $0201;
	ISDN_OPT_56KADAPTATION = $0202;

{ For ISDN_OPT_COMMTYPE...}

const
	kOTISDNTelephoneALaw = 1;    { G.711 A-law                }
	kOTISDNTelephoneMuLaw = 26;   { G.711 µ-law                }
	kOTISDNDigital64k = 13;   { unrestricted digital (default)     }
	kOTISDNDigital56k = 37;   { user rate 56Kb/s           }
	kOTISDNVideo64k = 41;   { video terminal at 64Kb/s    }
	kOTISDNVideo56k = 42;    { video terminal at 56Kb/s    }

{ For ISDN_OPT_FRAMINGTYPE...}

const
	kOTISDNFramingTransparent = $0010; { Transparent mode           }
	kOTISDNFramingHDLC = $0020; { HDLC synchronous mode (default)    }
	kOTISDNFramingV110 = $0040; { V.110 asynchronous mode       }
	kOTISDNFramingV14E = $0080; { V.14E asynchronous mode         }

{ For ISDN_OPT_56KADAPTATION...}

const
	kOTISDNNot56KAdaptation = false; { not 56K Adaptation (default)     }
	kOTISDN56KAdaptation = true;  { 56K Adaptation           }

{ Default options, you do not need to set these}

const
	kOTISDNDefaultCommType = kOTISDNDigital64k;
	kOTISDNDefaultFramingType = kOTISDNFramingHDLC;
	kOTISDNDefault56KAdaptation = kOTISDNNot56KAdaptation;


{******************************************************************************
*   Constants for Open Transport-based Remote Access/PPP API
*******************************************************************************}

{ OTCreateConfiguration name for PPP control endpoint}

const
	kPPPControlName = 'ppp';

{ XTI Level}

const
	COM_PPP = FourCharCode('PPPC');

{ Options limits}

const
	kPPPMaxIDLength = 255;
	kPPPMaxPasswordLength = 255;
	kPPPMaxDTEAddressLength = 127;
	kPPPMaxCallInfoLength = 255;


{ Various XTI option value constants}

const
	kPPPStateInitial = 1;
	kPPPStateClosed = 2;
	kPPPStateClosing = 3;
	kPPPStateOpening = 4;
	kPPPStateOpened = 5;

const
	kPPPConnectionStatusIdle = 1;
	kPPPConnectionStatusConnecting = 2;
	kPPPConnectionStatusConnected = 3;
	kPPPConnectionStatusDisconnecting = 4;

const
	kPPPMinMRU = 0;
	kPPPMaxMRU = 4500;

const
	kIPCPTCPHdrCompressionDisabled = 0;
	kIPCPTCPHdrCompressionEnabled = 1;

const
	kPPPCompressionDisabled = $00000000;
	kPPPProtoCompression = $00000001;
	kPPPAddrCompression = $00000002;

const
	kPPPNoOutAuthentication = 0;
	kPPPCHAPOrPAPOutAuthentication = 1;

const
	kCCReminderTimerDisabled = 0;
	kCCIPIdleTimerDisabled = 0;

const
	kPPPScriptTypeModem = 1;
	kPPPScriptTypeConnect = 2;
	kPPPMaxScriptSize = 32000;

const
	kE164Address = 1;
	kPhoneAddress = 1;
	kCompoundPhoneAddress = 2;
	kX121Address = 3;

const
	kPPPConnectionStatusDialogsFlag = $00000001;
	kPPPConnectionRemindersFlag = $00000002;
	kPPPConnectionFlashingIconFlag = $00000004;
	kPPPOutPasswordDialogsFlag = $00000008;
	kPPPAllAlertsDisabledFlag = $00000000;
	kPPPAllAlertsEnabledFlag = $0000000F;

const
	kPPPAsyncMapCharsNone = $00000000;
	kPPPAsyncMapCharsXOnXOff = $000A0000;
	kPPPAsyncMapCharsAll = $FFFFFFFF;


{ Option names}

const
	IPCP_OPT_GETREMOTEPROTOADDR = $00007000;
	IPCP_OPT_GETLOCALPROTOADDR = $00007001;
	IPCP_OPT_TCPHDRCOMPRESSION = $00007002;
	LCP_OPT_PPPCOMPRESSION = $00007003;
	LCP_OPT_MRU = $00007004;
	LCP_OPT_RCACCMAP = $00007005;
	LCP_OPT_TXACCMAP = $00007006;
	SEC_OPT_OUTAUTHENTICATION = $00007007;
	SEC_OPT_ID = $00007008;
	SEC_OPT_PASSWORD = $00007009;
	CC_OPT_REMINDERTIMER = $00007010;
	CC_OPT_IPIDLETIMER = $00007011;
	CC_OPT_DTEADDRESSTYPE = $00007012;
	CC_OPT_DTEADDRESS = $00007013;
	CC_OPT_CALLINFO = $00007014;
	CC_OPT_GETMISCINFO = $00007015;
	PPP_OPT_GETCURRENTSTATE = $00007016;
	LCP_OPT_ECHO = $00007017; { Available on Mac OS X only }
	CC_OPT_SERIALPORTNAME = $00007200;

{ Endpoint events}

const
	kPPPEvent = kPROTOCOLEVENT or $000F0000;
	kPPPConnectCompleteEvent = kPPPEvent + 1;
	kPPPSetScriptCompleteEvent = kPPPEvent + 2;
	kPPPDisconnectCompleteEvent = kPPPEvent + 3;
	kPPPDisconnectEvent = kPPPEvent + 4;
	kPPPIPCPUpEvent = kPPPEvent + 5;
	kPPPIPCPDownEvent = kPPPEvent + 6;
	kPPPLCPUpEvent = kPPPEvent + 7;
	kPPPLCPDownEvent = kPPPEvent + 8;
	kPPPLowerLayerUpEvent = kPPPEvent + 9;
	kPPPLowerLayerDownEvent = kPPPEvent + 10;
	kPPPAuthenticationStartedEvent = kPPPEvent + 11;
	kPPPAuthenticationFinishedEvent = kPPPEvent + 12;
	kPPPDCEInitStartedEvent = kPPPEvent + 13;
	kPPPDCEInitFinishedEvent = kPPPEvent + 14;
	kPPPDCECallStartedEvent = kPPPEvent + 15;
	kPPPDCECallFinishedEvent = kPPPEvent + 16;


{******************************************************************************
*   IOCTL constants for I_OTConnect, I_OTDisconnect and I_OTScript
*   are defined in OpenTransport.h
*******************************************************************************}

{******************************************************************************
*   PPPMRULimits
*******************************************************************************}
type
	PPPMRULimitsPtr = ^PPPMRULimits;
	PPPMRULimits = record
		mruSize: UInt32;                { proposed or actual}
		upperMRULimit: UInt32;
		lowerMRULimit: UInt32;
	end;

{******************************************************************************
*   CCMiscInfo
*******************************************************************************}
type
	CCMiscInfoPtr = ^CCMiscInfo;
	CCMiscInfo = record
		connectionStatus: UInt32;
		connectionTimeElapsed: UInt32;
		connectionTimeRemaining: UInt32;
		bytesTransmitted: UInt32;
		bytesReceived: UInt32;
		reserved: UInt32;
	end;

{******************************************************************************
*   LCPEcho
*******************************************************************************}
{ Set both fields to zero to disable sending of LCP echo requests}

type
	LCPEchoPtr = ^LCPEcho;
	LCPEcho = record
		retryCount: UInt32;
		retryPeriod: UInt32;            { in milliseconds}
	end;

{******************************************************************************
*   Bits used to tell kind of product
*******************************************************************************}
const
	kRAProductClientOnly = 2;
	kRAProductOnePortServer = 3;
	kRAProductManyPortServer = 4;


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


end.
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_MAC and TARGET_CPU_PPC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
