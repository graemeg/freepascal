{
	 File:	   CFNetwork/CFHTTPAuthentication.h
 
	 Contains:   CoreFoundation Network HTTP authentication header
 
	 Copyright:  Copyright (c) 2001-2008, Apple Inc. All rights reserved.
 
	 Bugs?:	  For bug reports, consult the following page on
				 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
					 http://bugs.freepascal.org
 
}
{       Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
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
=======
>>>>>>> origin/fixes_2.4
					 http://www.freepascal.org/bugs.html
 
}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit CFHTTPAuthentication;
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
uses MacTypes, CFArray, CFBase, CFDictionary, CFHTTPMessage, CFStream;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{
 *  CFHTTPAuthenticationRef
 *  
 *  Discussion:
 *	This is the type of a reference to HTTP authentication
 *	information.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFHTTPAuthenticationRef = ^_CFHTTPAuthentication; { an opaque type }
	_CFHTTPAuthentication = record end;
=======
	CFHTTPAuthenticationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFHTTPAuthenticationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFHTTPAuthenticationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFHTTPAuthenticationRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFHTTPAuthenticationRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{
 *  CFStreamErrorHTTPAuthentication
 *  
 *  Discussion:
 *	Authentication errors which may be returned as a result of trying
 *	to apply authentication to a request.  These errors are in the
 *	kCFStreamErrorDomainHTTP domain.
 }
type
	CFStreamErrorHTTPAuthentication = SInt32;
const
{
   * The type of authentication to be applied to a request is not
   * supported.
   }
	kCFStreamErrorHTTPAuthenticationTypeUnsupported = -1000;

  {
   * The username was in a format not suitable for applying to the
   * request.
   }
	kCFStreamErrorHTTPAuthenticationBadUserName = -1001;

  {
   * The password was in a format not suitable for applying to the
   * request.
   }
	kCFStreamErrorHTTPAuthenticationBadPassword = -1002;


{
 *  kCFHTTPAuthenticationUsername
 *  
 *  Discussion:
 *	CFDictionary key, for CFHTTPMessageApplyCredentialDictionary. The
 *	username for authentication as a CFString.  Needs to be added if
 *	CFHTTPAuthenticationRequiresUserNameAndPassword returns TRUE.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFHTTPAuthenticationUsername: CFStringRef; external name '_kCFHTTPAuthenticationUsername'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFHTTPAuthenticationPassword
 *  
 *  Discussion:
 *	CFDictionary key, for CFHTTPMessageApplyCredentialDictionary. The
 *	password for authentication as a CFString.  Needs to be added if
 *	CFHTTPAuthenticationRequiresUserNameAndPassword returns TRUE.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFHTTPAuthenticationPassword: CFStringRef; external name '_kCFHTTPAuthenticationPassword'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFHTTPAuthenticationAccountDomain
 *  
 *  Discussion:
 *	CFDictionary key, for CFHTTPMessageApplyCredentialDictionary. The
 *	domain for authentication as a CFString.  Needs to be added if
 *	CFHTTPAuthenticationRequiresAccountDomain returns TRUE.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFHTTPAuthenticationAccountDomain: CFStringRef; external name '_kCFHTTPAuthenticationAccountDomain'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  CFHTTPAuthenticationGetTypeID()
 *  
 *  Discussion:
 *	Returns the type identifier of all CFHTTPAuthentication instances.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationGetTypeID: CFTypeID; external name '_CFHTTPAuthenticationGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationCreateFromResponse()
 *  
 *  Discussion:
 *	Based on a response of 401 or 407, this function will create a
 *	new authentication object which can be used for adding
 *	credentials to future requests.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  Allocator to use for creating authentication object
 *	
 *	response:
 *	  Failed response.
 *  
 *  Result:
 *	A freshly created authentication object useful for applying
 *	authentication credentials to new requests.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationCreateFromResponse( alloc: CFAllocatorRef; response: CFHTTPMessageRef ): CFHTTPAuthenticationRef; external name '_CFHTTPAuthenticationCreateFromResponse';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationIsValid()
 *  
 *  Discussion:
 *	Returns TRUE if the given authentication information was
 *	instantiated correctly and contains enough information in order
 *	to be applied to a request. If FALSE is returned, the object may
 *	still contain information which is useful to the user, e.g.
 *	unsupported method name.  An invalid object may be queried for
 *	information but may not be applied to a request.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	auth:
 *	  The authentication information being queried.
 *	
 *	error:
 *	  Reference to a CFStreamError which will be populated in the
 *	  case of an error in creation.  Pass NULL if not interested in
 *	  the failure reason.  The error domain will be
 *	  kCFStreamErrorDomainHTTP, and the error code will be one of
 *	  those defined in CFHTTPStream.h or one of those listed as
 *	  CFStreamErrorHTTPAuthentication.
 *  
 *  Result:
 *	TRUE or FALSE depending on whether the authentication object is
 *	good for applying credentials to further requests.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationIsValid( auth: CFHTTPAuthenticationRef; error: CFStreamErrorPtr { can be NULL } ): Boolean; external name '_CFHTTPAuthenticationIsValid';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationAppliesToRequest()
 *  
 *  Discussion:
 *	Returns TRUE if the given request requires credentials based upon
 *	the given authentication information.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	auth:
 *	  The authentication information being queried.
 *	
 *	request:
 *	  The request which is believed to need the given authentication.
 *  
 *  Result:
 *	TRUE if the given authentication information should be applied to
 *	the request, otherwise FALSE is returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationAppliesToRequest( auth: CFHTTPAuthenticationRef; request: CFHTTPMessageRef ): Boolean; external name '_CFHTTPAuthenticationAppliesToRequest';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationRequiresOrderedRequests()
 *  
 *  Discussion:
 *	Some authentication methods require that future requests must be
 *	performed in an ordered manner, so that information from a
 *	response can be added to a following request.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	auth:
 *	  The authentication information being queried.
 *  
 *  Result:
 *	Returns TRUE if the given authentication method requires ordered
 *	requests.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationRequiresOrderedRequests( auth: CFHTTPAuthenticationRef ): Boolean; external name '_CFHTTPAuthenticationRequiresOrderedRequests';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPMessageApplyCredentials()
 *  
 *  Discussion:
 *	Perform the authentication method required on the request using
 *	the given username and password.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	request:
 *	  The request which is to receive the credentials.
 *	
 *	auth:
 *	  The authentication information for the given request.
 *	
 *	username:
 *	  The username to use for performing the authentication.
 *	
 *	password:
 *	  The password to use for performing the authentication.
 *	
 *	error:
 *	  Reference to a CFStreamError which will be populated with the
 *	  error information should one occurr during the application of
 *	  the credentials. Pass NULL if not interested in the failure
 *	  reason.  The error domain will be kCFStreamErrorDomainHTTP, and
 *	  the error code will be one of those define in CFHTTPStream.h or
 *	  one of those listed as CFStreamErrorHTTPAuthentication.
 *  
 *  Result:
 *	TRUE will be returned if the application of the credentials to
 *	the request was successful, otherwise FALSE is returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPMessageApplyCredentials( request: CFHTTPMessageRef; auth: CFHTTPAuthenticationRef; username: CFStringRef; password: CFStringRef; error: CFStreamErrorPtr { can be NULL } ): Boolean; external name '_CFHTTPMessageApplyCredentials';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPMessageApplyCredentialDictionary()
 *  
 *  Discussion:
 *	Perform the authentication method required on the request using
 *	the given credential information.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	request:
 *	  The request which is to receive the credentials.
 *	
 *	auth:
 *	  The authentication information for the given request.
 *	
 *	dict:
 *	  A dictionary containing credentials to be applied to the
 *	  request.  Valid keys are declared above.
 *	
 *	error:
 *	  Reference to a CFStreamError which will be populated with the
 *	  error information should one occurr during the application of
 *	  the credentials. Pass NULL if not interested in the failure
 *	  reason.  The error domain will be kCFStreamErrorDomainHTTP, and
 *	  the error code will be one of those define in CFHTTPStream.h or
 *	  one of those listed as CFStreamErrorHTTPAuthentication.
 *  
 *  Result:
 *	TRUE will be returned if the application of the credentials to
 *	the request was successful, otherwise FALSE is returned.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPMessageApplyCredentialDictionary( request: CFHTTPMessageRef; auth: CFHTTPAuthenticationRef; dict: CFDictionaryRef; error: CFStreamErrorPtr { can be NULL } ): Boolean; external name '_CFHTTPMessageApplyCredentialDictionary';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationCopyRealm()
 *  
 *  Discussion:
 *	Some authentication techniques provide for namespaces on top of
 *	domains. This call will return the authentication information's
 *	namespace if there is one, otherwise it will return NULL.  This
 *	namespace is usually used for prompting the application user for
 *	a name and password.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	auth:
 *	  The authentication information being queried.
 *  
 *  Result:
 *	This call will return the authentication information's namespace
 *	if there is one, otherwise it will return NULL.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationCopyRealm( auth: CFHTTPAuthenticationRef ): CFStringRef; external name '_CFHTTPAuthenticationCopyRealm';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationCopyDomains()
 *  
 *  Discussion:
 *	Returns a list of domain URL's on which the given authentication
 *	should be applied. This function is provided mostly for
 *	informational purposes. CFHTTPAuthenticationAppliesToRequest
 *	should be used in order to check whether a request requires the
 *	authentication.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	auth:
 *	  The authentication information being queried.
 *  
 *  Result:
 *	Returns a list of domain URL's on which the given authentication
 *	should be applied.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationCopyDomains( auth: CFHTTPAuthenticationRef ): CFArrayRef; external name '_CFHTTPAuthenticationCopyDomains';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationCopyMethod()
 *  
 *  Discussion:
 *	Returns the method of authentication which will be performed when
 *	applying credentials.  The strongest method of authentication
 *	will be chosen in the case of multiple choices.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	auth:
 *	  The authentication information being queried.
 *  
 *  Result:
 *	Returns the method of authentication which will be performed when
 *	applying credentials.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationCopyMethod( auth: CFHTTPAuthenticationRef ): CFStringRef; external name '_CFHTTPAuthenticationCopyMethod';
(* __OSX_AVAILABLE_STARTING(__MAC_10_2,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationRequiresUserNameAndPassword()
 *  
 *  Discussion:
 *	Returns TRUE if the chosen authentication scheme requires a
 *	username and password.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	auth:
 *	  The authentication information being queried.
 *  
 *  Result:
 *	Returns TRUE if the chosen authentication scheme requires a
 *	username and password.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationRequiresUserNameAndPassword( auth: CFHTTPAuthenticationRef ): Boolean; external name '_CFHTTPAuthenticationRequiresUserNameAndPassword';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)


{
 *  CFHTTPAuthenticationRequiresAccountDomain()
 *  
 *  Discussion:
 *	Returns TRUE if the chosen authentication scheme requires a
 *	domain for authentication.  Currently, this will return TRUE for
 *	"NTLM" and FALSE for the other methods.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *	The API's to CFHTTPAuthenticationRef are thread-safe so long as
 *	multiple threads are not altering the same
 *	CFHTTPAuthenticationRef at the same time.
 *  
 *  Parameters:
 *	
 *	auth:
 *	  The authentication information being queried.
 *  
 *  Result:
 *	Returns TRUE if the chosen authentication scheme requires a
 *	domain for authentication.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFHTTPAuthenticationRequiresAccountDomain( auth: CFHTTPAuthenticationRef ): Boolean; external name '_CFHTTPAuthenticationRequiresAccountDomain';
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
