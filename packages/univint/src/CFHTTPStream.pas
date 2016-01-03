{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	 File:	   CFNetwork/CFHTTPStream.h
 
	 Contains:   CoreFoundation Network HTTP streams header
 
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
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
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
{   Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
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
     File:       CFNetwork/CFHTTPStream.h
=======
	 File:	   CFNetwork/CFHTTPStream.h
>>>>>>> origin/fixes_2.4
 
	 Contains:   CoreFoundation Network HTTP streams header
 
	 Copyright:  Copyright (c) 2001-2008, Apple Inc. All rights reserved.
 
	 Bugs?:	  For bug reports, consult the following page on
				 the World Wide Web:
 
					 http://www.freepascal.org/bugs.html
 
}
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
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

unit CFHTTPStream;
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
uses MacTypes,CFStream,CFBase,CFHTTPMessage;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}
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
 *  kCFStreamErrorDomainHTTP
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Discussion:
 *	Result code returned by HTTP server
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainHTTP: SInt32; external name '_kCFStreamErrorDomainHTTP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)

{
 *  CFStreamErrorHTTP
 *  
 *  Discussion:
 *	Errors from the kCFStreamErrorDomainHTTP domain.
 }
type
	CFStreamErrorHTTP = SInt32;
const
{
   * Could not parse the request/response.
   }
	kCFStreamErrorHTTPParseFailure = -1;

  {
   * A loop was detected during redirection.
   }
	kCFStreamErrorHTTPRedirectionLoop = -2;

  {
   * Could not retreive url for request/response.
   }
	kCFStreamErrorHTTPBadURL = -3;

{
 *  kCFStreamPropertyHTTPResponseHeader
 *  
 *  Discussion:
 *	Stream property key, for copy operations. Value is a
 *	CFHTTPMessage with 0 bytes data.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPResponseHeader: CFStringRef; external name '_kCFStreamPropertyHTTPResponseHeader'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPFinalURL
 *  
 *  Discussion:
 *	Stream property key, for copy operations. The response header
 *	value is the CFURL from the final request; will only differ from
 *	the URL in the original request if an autoredirection has
 *	occurred.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPFinalURL: CFStringRef; external name '_kCFStreamPropertyHTTPFinalURL'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

{
 *  kCFStreamPropertyHTTPFinalRequest
 *  
 *  Discussion:
 *	Stream property key, for copy operations. The value is the
 *	CFHTTPMessage transmitted by the stream, after all modifications
 *	(such as for authentication, connection policy, or redirection)
 *	have been made.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPFinalRequest: CFStringRef; external name '_kCFStreamPropertyHTTPFinalRequest'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFStreamPropertyHTTPProxy
 *  
 *  Discussion:
 *	Stream property key, for both set and copy operations. The value
 *	is a CFDictionary. HTTP proxy information is set the same way as
 *	SOCKS proxies (see CFSocketStream.h). Call
 *	CFReadStreamSetProperty() passing an HTTP stream and the property
 *	kCFStreamPropertyHTTPProxy. The value should include at least one
 *	Host/Port pair from the keys below. Use the dictionary returned
 *	by SystemConfiguration.framework to set the default values for
 *	the system. HTTP proxies are not applied automatically.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPProxy: CFStringRef; external name '_kCFStreamPropertyHTTPProxy'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPProxyHost
 *  
 *  Discussion:
 *	Proxy dictionary key. The hostname of an HTTP proxy. The value is
 *	a CFString. The key name matches kSCPropNetProxiesHTTPProxy.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPProxyHost: CFStringRef; external name '_kCFStreamPropertyHTTPProxyHost'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPProxyPort
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFNumber.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPProxyPort: CFStringRef; external name '_kCFStreamPropertyHTTPProxyPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)
{ matches kSCPropNetProxiesHTTPPort }


{
 *  kCFStreamPropertyHTTPSProxyHost
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFString.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPSProxyHost: CFStringRef; external name '_kCFStreamPropertyHTTPSProxyHost'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)
{ matches kSCPropNetProxiesHTTPSProxy }


{
 *  kCFStreamPropertyHTTPSProxyPort
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFNumber.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPSProxyPort: CFStringRef; external name '_kCFStreamPropertyHTTPSProxyPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)
{ matches kSCPropNetProxiesHTTPSPort }


{
 *  kCFStreamPropertyHTTPShouldAutoredirect
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Stream property key. Value is a CFBoolean. Redirection is not
=======
 *	Proxy dictionary key. Value is a CFBoolean. Redirection is not
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFBoolean. Redirection is not
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFBoolean. Redirection is not
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFBoolean. Redirection is not
>>>>>>> origin/cpstrnew
 *	performed by default.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPShouldAutoredirect: CFStringRef; external name '_kCFStreamPropertyHTTPShouldAutoredirect'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPAttemptPersistentConnection
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Stream property key. Value is a CFBoolean.  If this property is
=======
 *	Proxy dictionary key. Value is a CFBoolean.  If this property is
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFBoolean.  If this property is
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFBoolean.  If this property is
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFBoolean.  If this property is
>>>>>>> origin/cpstrnew
 *	set to kCFBooleanTrue, an HTTP stream will look for an
 *	appropriate extant persistent connection to use, and if it finds
 *	none, will try to create one. Persistent connections are not used
 *	by default.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPAttemptPersistentConnection: CFStringRef; external name '_kCFStreamPropertyHTTPAttemptPersistentConnection'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPRequestBytesWrittenCount
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Stream property key. Value is a CFNumber. This property can only
=======
 *	Proxy dictionary key. Value is a CFNumber. This property can only
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFNumber. This property can only
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFNumber. This property can only
>>>>>>> graemeg/cpstrnew
=======
 *	Proxy dictionary key. Value is a CFNumber. This property can only
>>>>>>> origin/cpstrnew
 *	be retrieved, not set. The number returned is the number of bytes
 *	from the body of the request that have been written to the
 *	underlying socket
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPRequestBytesWrittenCount: CFStringRef; external name '_kCFStreamPropertyHTTPRequestBytesWrittenCount'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)


{*******************}
{ Creation routines }
{*******************}
{
 *  CFReadStreamCreateForHTTPRequest()
 *  
 *  Discussion:
 *	Create an HTTP read stream for the response to the given request.
 *	When the stream is opened, it will begin transmitting the
 *	request. The bytes returned are the pure body bytes; the response
 *	header has been parsed off. To retrieve the response header, ask
 *	for kCFStreamPropertyHTTPResponseHeader, above, any time after
 *	the first bytes arrive on the stream (or when stream end is
 *	reported, if there are no data bytes). When an HTTP/1.1 server
 *	returns a chunked a response, the chunks will be formed into one
 *	continuous stream.
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  A pointer to the CFAllocator which should be used to allocate
 *	  memory for the CF read stream and its storage for values. If
 *	  this reference is not a valid CFAllocator, the behavior is
 *	  undefined.
 *	
 *	request:
 *	  A pointer to a CFHTTPMessage created by the
 *	  CFHTTPMessageCreateRequest function.
 *  
 *  Result:
 *	A pointer to the CF read stream created, or NULL if failed. It is
 *	caller's responsibilty to release the memory allocated for the
 *	read stream.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFReadStreamCreateForHTTPRequest( alloc: CFAllocatorRef; request: CFHTTPMessageRef ): CFReadStreamRef; external name '_CFReadStreamCreateForHTTPRequest';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)


{
 *  CFReadStreamCreateForStreamedHTTPRequest()
 *  
 *  Discussion:
 *	Creates a read stream for the response to the given
 *	requestHeaders plus requestBody. Use in preference to
 *	CFReadStreamCreateForHTTPRequest() when the body of the request
 *	is larger than you wish to be resident in memory.  Note that
 *	because streams cannot be reset, read streams created this way
 *	cannot have autoredirection enabled.  If the Content-Length
 *	header is set in requestHeaders, it is assumed that the caller
 *	got the length right and that requestBody will report
 *	end-of-stream after precisely Content-Length bytes have been read
 *	from it. If the Content-Length header is not set, the chunked
 *	transfer-encoding will be added to requestHeaders, and bytes read
 *	from requestBody will be transmitted chunked. The body of
 *	requestHeaders is ignored.
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  A pointer to the CFAllocator which should be used to allocate
 *	  memory for the CF read stream and its storage for values. If
 *	  this reference is not a valid CFAllocator, the behavior is
 *	  undefined.
 *	
 *	requestHeaders:
 *	  A pointer to a CFHTTPMessage created by the
 *	  CFHTTPMessageCreateRequest function. The body of requestHeaders
 *	  is ignored.
 *	
 *	requestBody:
 *	  A pointer to a CFReadStream.
 *  
 *  Result:
 *	A pointer to the CF read stream created, or NULL if failed. It is
 *	caller's responsibilty to release the memory allocated for the
 *	read stream.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFReadStreamCreateForStreamedHTTPRequest( alloc: CFAllocatorRef; requestHeaders: CFHTTPMessageRef; requestBody: CFReadStreamRef ): CFReadStreamRef; external name '_CFReadStreamCreateForStreamedHTTPRequest';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
{
 *  CFHTTPReadStreamSetRedirectsAutomatically()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *	Use the kCFStreamPropertyHTTPShouldAutoredirect property above
 *	instead.
 *  
 *  Discussion:
 *	Sets the redirection property on the http stream.
 *  
 *  Parameters:
 *	
 *	httpStream:
 *	  A pointer to the CFHTTPStream to be set.
 *	
 *	shouldAutoRedirect:
 *	  A boolean indicating whether to redirect or not.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework but deprecated in 10.3
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHTTPReadStreamSetRedirectsAutomatically( httpStream: CFReadStreamRef; shouldAutoRedirect: Boolean ); external name '_CFHTTPReadStreamSetRedirectsAutomatically';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_3,__IPHONE_NA,__IPHONE_NA) *)


{
 *  CFHTTPReadStreamSetProxy()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *	Use the kCFStreamPropertyHTTPProxy above instead.
 *  
 *  Discussion:
 *	Sets the redirection property on the http stream.
 *  
 *  Parameters:
 *	
 *	httpStream:
 *	  A pointer to the CFHTTPStream to be set.
 *	
 *	proxyHost:
 *	  The proxy hostname. A CFString value.
 *	
 *	proxyPort:
 *	  The port number. A CFNumber value.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework but deprecated in 10.3
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHTTPReadStreamSetProxy( httpStream: CFReadStreamRef; proxyHost: CFStringRef; proxyPort: CFIndex ); external name '_CFHTTPReadStreamSetProxy';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_3,__IPHONE_NA,__IPHONE_NA) *)


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Discussion:
 *	Result code returned by HTTP server
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainHTTP: SInt32; external name '_kCFStreamErrorDomainHTTP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)

{
 *  CFStreamErrorHTTP
 *  
 *  Discussion:
 *	Errors from the kCFStreamErrorDomainHTTP domain.
 }
type
	CFStreamErrorHTTP = SInt32;
const
{
   * Could not parse the request/response.
   }
	kCFStreamErrorHTTPParseFailure = -1;

  {
   * A loop was detected during redirection.
   }
	kCFStreamErrorHTTPRedirectionLoop = -2;

  {
   * Could not retreive url for request/response.
   }
	kCFStreamErrorHTTPBadURL = -3;

{
 *  kCFStreamPropertyHTTPResponseHeader
 *  
 *  Discussion:
 *	Stream property key, for copy operations. Value is a
 *	CFHTTPMessage with 0 bytes data.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPResponseHeader: CFStringRef; external name '_kCFStreamPropertyHTTPResponseHeader'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPFinalURL
 *  
 *  Discussion:
 *	Stream property key, for copy operations. The response header
 *	value is the CFURL from the final request; will only differ from
 *	the URL in the original request if an autoredirection has
 *	occurred.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPFinalURL: CFStringRef; external name '_kCFStreamPropertyHTTPFinalURL'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

{
 *  kCFStreamPropertyHTTPFinalRequest
 *  
 *  Discussion:
 *	Stream property key, for copy operations. The value is the
 *	CFHTTPMessage transmitted by the stream, after all modifications
 *	(such as for authentication, connection policy, or redirection)
 *	have been made.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPFinalRequest: CFStringRef; external name '_kCFStreamPropertyHTTPFinalRequest'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFStreamPropertyHTTPProxy
 *  
 *  Discussion:
 *	Stream property key, for both set and copy operations. The value
 *	is a CFDictionary. HTTP proxy information is set the same way as
 *	SOCKS proxies (see CFSocketStream.h). Call
 *	CFReadStreamSetProperty() passing an HTTP stream and the property
 *	kCFStreamPropertyHTTPProxy. The value should include at least one
 *	Host/Port pair from the keys below. Use the dictionary returned
 *	by SystemConfiguration.framework to set the default values for
 *	the system. HTTP proxies are not applied automatically.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPProxy: CFStringRef; external name '_kCFStreamPropertyHTTPProxy'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPProxyHost
 *  
 *  Discussion:
 *	Proxy dictionary key. The hostname of an HTTP proxy. The value is
 *	a CFString. The key name matches kSCPropNetProxiesHTTPProxy.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPProxyHost: CFStringRef; external name '_kCFStreamPropertyHTTPProxyHost'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPProxyPort
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFNumber.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPProxyPort: CFStringRef; external name '_kCFStreamPropertyHTTPProxyPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)
{ matches kSCPropNetProxiesHTTPPort }


{
 *  kCFStreamPropertyHTTPSProxyHost
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFString.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPSProxyHost: CFStringRef; external name '_kCFStreamPropertyHTTPSProxyHost'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)
{ matches kSCPropNetProxiesHTTPSProxy }


{
 *  kCFStreamPropertyHTTPSProxyPort
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFNumber.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPSProxyPort: CFStringRef; external name '_kCFStreamPropertyHTTPSProxyPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)
{ matches kSCPropNetProxiesHTTPSPort }


{
 *  kCFStreamPropertyHTTPShouldAutoredirect
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFBoolean. Redirection is not
 *	performed by default.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPShouldAutoredirect: CFStringRef; external name '_kCFStreamPropertyHTTPShouldAutoredirect'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPAttemptPersistentConnection
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFBoolean.  If this property is
 *	set to kCFBooleanTrue, an HTTP stream will look for an
 *	appropriate extant persistent connection to use, and if it finds
 *	none, will try to create one. Persistent connections are not used
 *	by default.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPAttemptPersistentConnection: CFStringRef; external name '_kCFStreamPropertyHTTPAttemptPersistentConnection'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  kCFStreamPropertyHTTPRequestBytesWrittenCount
 *  
 *  Discussion:
 *	Proxy dictionary key. Value is a CFNumber. This property can only
 *	be retrieved, not set. The number returned is the number of bytes
 *	from the body of the request that have been written to the
 *	underlying socket
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyHTTPRequestBytesWrittenCount: CFStringRef; external name '_kCFStreamPropertyHTTPRequestBytesWrittenCount'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)


{*******************}
{ Creation routines }
{*******************}
{
 *  CFReadStreamCreateForHTTPRequest()
 *  
 *  Discussion:
 *	Create an HTTP read stream for the response to the given request.
 *	When the stream is opened, it will begin transmitting the
 *	request. The bytes returned are the pure body bytes; the response
 *	header has been parsed off. To retrieve the response header, ask
 *	for kCFStreamPropertyHTTPResponseHeader, above, any time after
 *	the first bytes arrive on the stream (or when stream end is
 *	reported, if there are no data bytes). When an HTTP/1.1 server
 *	returns a chunked a response, the chunks will be formed into one
 *	continuous stream.
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  A pointer to the CFAllocator which should be used to allocate
 *	  memory for the CF read stream and its storage for values. If
 *	  this reference is not a valid CFAllocator, the behavior is
 *	  undefined.
 *	
 *	request:
 *	  A pointer to a CFHTTPMessage created by the
 *	  CFHTTPMessageCreateRequest function.
 *  
 *  Result:
 *	A pointer to the CF read stream created, or NULL if failed. It is
 *	caller's responsibilty to release the memory allocated for the
 *	read stream.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFReadStreamCreateForHTTPRequest( alloc: CFAllocatorRef; request: CFHTTPMessageRef ): CFReadStreamRef; external name '_CFReadStreamCreateForHTTPRequest';
(* __OSX_AVAILABLE_STARTING(__MAC_10_1,__IPHONE_2_0) *)


{
 *  CFReadStreamCreateForStreamedHTTPRequest()
 *  
 *  Discussion:
 *	Creates a read stream for the response to the given
 *	requestHeaders plus requestBody. Use in preference to
 *	CFReadStreamCreateForHTTPRequest() when the body of the request
 *	is larger than you wish to be resident in memory.  Note that
 *	because streams cannot be reset, read streams created this way
 *	cannot have autoredirection enabled.  If the Content-Length
 *	header is set in requestHeaders, it is assumed that the caller
 *	got the length right and that requestBody will report
 *	end-of-stream after precisely Content-Length bytes have been read
 *	from it. If the Content-Length header is not set, the chunked
 *	transfer-encoding will be added to requestHeaders, and bytes read
 *	from requestBody will be transmitted chunked. The body of
 *	requestHeaders is ignored.
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  A pointer to the CFAllocator which should be used to allocate
 *	  memory for the CF read stream and its storage for values. If
 *	  this reference is not a valid CFAllocator, the behavior is
 *	  undefined.
 *	
 *	requestHeaders:
 *	  A pointer to a CFHTTPMessage created by the
 *	  CFHTTPMessageCreateRequest function. The body of requestHeaders
 *	  is ignored.
 *	
 *	requestBody:
 *	  A pointer to a CFReadStream.
 *  
 *  Result:
 *	A pointer to the CF read stream created, or NULL if failed. It is
 *	caller's responsibilty to release the memory allocated for the
 *	read stream.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFReadStreamCreateForStreamedHTTPRequest( alloc: CFAllocatorRef; requestHeaders: CFHTTPMessageRef; requestBody: CFReadStreamRef ): CFReadStreamRef; external name '_CFReadStreamCreateForStreamedHTTPRequest';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
{
 *  CFHTTPReadStreamSetRedirectsAutomatically()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *	Use the kCFStreamPropertyHTTPShouldAutoredirect property above
 *	instead.
 *  
 *  Discussion:
 *	Sets the redirection property on the http stream.
 *  
 *  Parameters:
 *	
 *	httpStream:
 *	  A pointer to the CFHTTPStream to be set.
 *	
 *	shouldAutoRedirect:
 *	  A boolean indicating whether to redirect or not.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework but deprecated in 10.3
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHTTPReadStreamSetRedirectsAutomatically( httpStream: CFReadStreamRef; shouldAutoRedirect: Boolean ); external name '_CFHTTPReadStreamSetRedirectsAutomatically';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_3,__IPHONE_NA,__IPHONE_NA) *)


{
 *  CFHTTPReadStreamSetProxy()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *	Use the kCFStreamPropertyHTTPProxy above instead.
 *  
 *  Discussion:
 *	Sets the redirection property on the http stream.
 *  
 *  Parameters:
 *	
 *	httpStream:
 *	  A pointer to the CFHTTPStream to be set.
 *	
 *	proxyHost:
 *	  The proxy hostname. A CFString value.
 *	
 *	proxyPort:
 *	  The port number. A CFNumber value.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework but deprecated in 10.3
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFHTTPReadStreamSetProxy( httpStream: CFReadStreamRef; proxyHost: CFStringRef; proxyPort: CFIndex ); external name '_CFHTTPReadStreamSetProxy';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_3,__IPHONE_NA,__IPHONE_NA) *)


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
