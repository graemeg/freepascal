{
	 File:	   CFNetwork/CFNetworkErrors.h
 
	 Contains:   CFNetwork error header
 
	 Copyright:  Copyright (c) 2006-2008, Apple Inc. All rights reserved.
 
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

unit CFNetworkErrorss;
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
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes, CFBase;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{GRP translation note: Double 's' unit name ending intentional to avoid GPC redeclaration error with 'CFNetworkErrors' type identifier.}

{
 *  kCFErrorDomainCFNetwork
 *  
 *  Discussion:
 *	Error domain for all errors originating in CFNetwork. Error codes
 *	may be interpreted using the list below.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFErrorDomainCFNetwork: CFStringRef; external name '_kCFErrorDomainCFNetwork'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFErrorDomainWinSock
 *  
 *  Discussion:
 *	On Windows, errors originating from WinSock are represented using
 *	this domain.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFErrorDomainWinSock: CFStringRef; external name '_kCFErrorDomainWinSock'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)


{
 *  CFNetworkErrors
 *  
 *  Discussion:
 *	The list of all public error codes returned under the error domain
 *	kCFErrorDomainCFNetwork
 }
type
	CFNetworkErrors = SInt32;
const
	kCFHostErrorHostNotFound = 1;
	kCFHostErrorUnknown = 2; // Query the kCFGetAddrInfoFailureKey to get the value returned from getaddrinfo; lookup in netdb.h
  // SOCKS errors; in all cases you may query kCFSOCKSStatusCodeKey to recover the status code returned by the server
	kCFSOCKSErrorUnknownClientVersion = 100;
	kCFSOCKSErrorUnsupportedServerVersion = 101; // Query the kCFSOCKSVersionKey to find the version requested by the server
  // SOCKS4-specific errors
	kCFSOCKS4ErrorRequestFailed = 110;  // request rejected or failed by the server
	kCFSOCKS4ErrorIdentdFailed = 111;  // request rejected because SOCKS server cannot connect to identd on the client
	kCFSOCKS4ErrorIdConflict = 112;  // request rejected because the client program and identd report different user-ids
	kCFSOCKS4ErrorUnknownStatusCode = 113;
  // SOCKS5-specific errors
	kCFSOCKS5ErrorBadState = 120;
	kCFSOCKS5ErrorBadResponseAddr = 121;
	kCFSOCKS5ErrorBadCredentials = 122;
	kCFSOCKS5ErrorUnsupportedNegotiationMethod = 123; // query kCFSOCKSNegotiationMethodKey to find the method requested
	kCFSOCKS5ErrorNoAcceptableMethod = 124;
  // FTP errors; query the kCFFTPStatusCodeKey to get the status code returned by the server
	kCFFTPErrorUnexpectedStatusCode = 200;
  // HTTP errors
	kCFErrorHTTPAuthenticationTypeUnsupported = 300;
	kCFErrorHTTPBadCredentials = 301;
	kCFErrorHTTPConnectionLost = 302;
	kCFErrorHTTPParseFailure = 303;
	kCFErrorHTTPRedirectionLoopDetected = 304;
	kCFErrorHTTPBadURL = 305;
	kCFErrorHTTPProxyConnectionFailure = 306;
	kCFErrorHTTPBadProxyCredentials = 307;
	kCFErrorPACFileError = 308;
	kCFErrorPACFileAuth = 309;
	kCFErrorHTTPSProxyConnectionFailure = 310;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFStreamErrorHTTPSProxyFailureUnexpectedResponseToCONNECTMethod = 311;
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
	
  // Error codes for CFURLConnection and CFURLProtocol
	kCFURLErrorUnknown = -998;
	kCFURLErrorCancelled = -999;
	kCFURLErrorBadURL = -1000;
	kCFURLErrorTimedOut = -1001;
	kCFURLErrorUnsupportedURL = -1002;
	kCFURLErrorCannotFindHost = -1003;
	kCFURLErrorCannotConnectToHost = -1004;
	kCFURLErrorNetworkConnectionLost = -1005;
	kCFURLErrorDNSLookupFailed = -1006;
	kCFURLErrorHTTPTooManyRedirects = -1007;
	kCFURLErrorResourceUnavailable = -1008;
	kCFURLErrorNotConnectedToInternet = -1009;
	kCFURLErrorRedirectToNonExistentLocation = -1010;
	kCFURLErrorBadServerResponse = -1011;
	kCFURLErrorUserCancelledAuthentication = -1012;
	kCFURLErrorUserAuthenticationRequired = -1013;
	kCFURLErrorZeroByteResource = -1014;
	kCFURLErrorCannotDecodeRawData = -1015;
	kCFURLErrorCannotDecodeContentData = -1016;
	kCFURLErrorCannotParseResponse = -1017;
	kCFURLErrorInternationalRoamingOff = -1018;
	kCFURLErrorCallIsActive = -1019;
	kCFURLErrorDataNotAllowed = -1020;
	kCFURLErrorRequestBodyStreamExhausted = -1021;
	kCFURLErrorFileDoesNotExist = -1100;
	kCFURLErrorFileIsDirectory = -1101;
	kCFURLErrorNoPermissionsToReadFile = -1102;
	kCFURLErrorDataLengthExceedsMaximum = -1103;
  // SSL errors
	kCFURLErrorSecureConnectionFailed = -1200;
	kCFURLErrorServerCertificateHasBadDate = -1201;
	kCFURLErrorServerCertificateUntrusted = -1202;
	kCFURLErrorServerCertificateHasUnknownRoot = -1203;
	kCFURLErrorServerCertificateNotYetValid = -1204;
	kCFURLErrorClientCertificateRejected = -1205;
	kCFURLErrorClientCertificateRequired = -1206;
	kCFURLErrorCannotLoadFromNetwork = -2000;
  // Download and file I/O errors
	kCFURLErrorCannotCreateFile = -3000;
	kCFURLErrorCannotOpenFile = -3001;
	kCFURLErrorCannotCloseFile = -3002;
	kCFURLErrorCannotWriteToFile = -3003;
	kCFURLErrorCannotRemoveFile = -3004;
	kCFURLErrorCannotMoveFile = -3005;
	kCFURLErrorDownloadDecodingFailedMidStream = -3006;
	kCFURLErrorDownloadDecodingFailedToComplete = -3007;
	
  // Cookie errors
	kCFHTTPCookieCannotParseCookieFile = -4000;

  // Errors originating from CFNetServices
	kCFNetServiceErrorUnknown = -72000;
	kCFNetServiceErrorCollision = -72001;
	kCFNetServiceErrorNotFound = -72002;
	kCFNetServiceErrorInProgress = -72003;
	kCFNetServiceErrorBadArgument = -72004;
	kCFNetServiceErrorCancel = -72005;
	kCFNetServiceErrorInvalid = -72006;
	kCFNetServiceErrorTimeout = -72007;
	kCFNetServiceErrorDNSServiceFailure = -73000; // An error from DNS discovery; look at kCFDNSServiceFailureKey to get the error number and interpret using dns_sd.h	


{ Keys used by CFNetwork to pass additional error information back to the user within CFError's userInfo dictionary }
{
 *  kCFURLErrorFailingURLErrorKey
 *  
 *  Discussion:
 *	When an NSURLConnection or NSURLDownload error occurs, this key's
 *  value is set to the URL which caused a load to fail
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.6 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFURLErrorFailingURLErrorKey: CFStringRef; external name '_kCFURLErrorFailingURLErrorKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_2) *)

{
 *  kCFURLErrorFailingURLStringErrorKey
 *  
 *  Discussion:
 *	When an NSURLConnection or NSURLDownload error occurs, this key's
 *  value is set to the CFString value of the URL which caused a load
 *  to fail
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.6 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFURLErrorFailingURLStringErrorKey: CFStringRef; external name '_kCFURLErrorFailingURLStringErrorKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_2) *)

{
 *  kCFGetAddrInfoFailureKey
 *  
 *  Discussion:
 *	When an error of kCFHostErrorUnknown is returned, this key's
 *	value is set to a CFNumber containing the raw error value
 *	returned by getaddrinfo()
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFGetAddrInfoFailureKey: CFStringRef; external name '_kCFGetAddrInfoFailureKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFSOCKSStatusCodeKey
 *  
 *  Discussion:
 *	When a SOCKS failure has occurred, this key's value is set to a
 *	CFString containing the status value returned by the SOCKS server.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFSOCKSStatusCodeKey: CFStringRef; external name '_kCFSOCKSStatusCodeKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFSOCKSVersionKey
 *  
 *  Discussion:
 *	When an error of kCFSOCKSErrorUnsupportedServerVersion is
 *	returned, this key's value is set to a CFString containing the
 *	version number requested by the server.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFSOCKSVersionKey: CFStringRef; external name '_kCFSOCKSVersionKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFSOCKSNegotiationMethodKey
 *  
 *  Discussion:
 *	When an error of kCFSOCKS5ErrorUnsupportedNegotiationMethod is
 *	returned, this key's value is set to a CFString containing the
 *	negotiation method requested by the server.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFSOCKSNegotiationMethodKey: CFStringRef; external name '_kCFSOCKSNegotiationMethodKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFDNSServiceFailureKey
 *  
 *  Discussion:
 *	When an error of kCFNetServicesErrorDNSServiceFailure is
 *	returned, this key's value is set to a CFNumber containing the
 *	value returned from DNS; interret it using the values dns_sd.h
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFDNSServiceFailureKey: CFStringRef; external name '_kCFDNSServiceFailureKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFFTPStatusCodeKey
 *  
 *  Discussion:
 *	When an error of kCFFTPErrorUnexpectedStatusCode is returned,
 *	this key's value is set to a CFString containing the status code
 *	returned by the server
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFFTPStatusCodeKey: CFStringRef; external name '_kCFFTPStatusCodeKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
