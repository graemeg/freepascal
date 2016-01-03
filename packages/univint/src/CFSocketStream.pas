{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	 File:	   CFNetwork/CFSocketStream.h
 
	 Contains:   CoreFoundation Network socket streams header
 
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
     File:       CFNetwork/CFSocketStream.h
=======
	 File:	   CFNetwork/CFSocketStream.h
>>>>>>> origin/fixes_2.4
 
	 Contains:   CoreFoundation Network socket streams header
 
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

unit CFSocketStream;
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	{$setc TARGET_CPU_X86 := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
{$elsec}
	{$error Neither __ppc__ nor __i386__ is defined.}
{$endc}
{$setc TARGET_CPU_PPC_64 := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifc defined FPC_BIG_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := TRUE}
	{$setc TARGET_RT_LITTLE_ENDIAN := FALSE}
{$elifc defined FPC_LITTLE_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := FALSE}
	{$setc TARGET_RT_LITTLE_ENDIAN := TRUE}
{$elsec}
<<<<<<< HEAD
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
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,CFStream,CFBase,CFHost,CFNetServices;
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  kCFStreamPropertySSLContext 
 *
 * The SSLContextRef used for both read and write operations on a
 * CFSocketStream.
 *
 * CFReadStreamCopyProperty or CFWriteStreamCopyProperty return an
 * appropriately reference counted SSLContextRef.  If the stream has
 * not yet been opened, this SSLContext may be configured directly
 * using the appropriate SecureTransport APIs.
 *
 * CFReadStreamSetProperty or CFWriteStreamSetProperty will allow you
 * to specify an SSLContextRef for a stream.  If the stream has not
 * been opened, the SSLContextRef will replace any existing
 * SSLContextRef and be used in the initial stream handshake.  If the
 * stream has been opened without SSL enabled, setting this property
 * will initiate an SSL handshake over the existing socket.
 *
 * If an SSL settings dictionary was set via
 * kCFStreamPropertySSLSettings, a SSLContextRef is created internally
 * and configured as per the dictionary.  However, if an SSLContextRef
 * is set after this, its configuration will take precedence over the
 * previously configured context.
 *
 * Reconfiguring an SSLContext after the stream it is bound to has
 * opened is unsupported.
 *
 * Note that this property is currently only supported on iOS 5.0 and
 * later.
 }
var kCFStreamPropertySSLContext: CFStringRef; external name '_kCFStreamPropertySSLContext'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_5_0) *)

{
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  kCFStreamPropertySSLPeerTrust
 *  
 *  Discussion:
 *	Stream property value for copy operations.  Returns a SecTrustRef
 *	which was a result of the SSL handshake.  This property is not valid before
 *  a stream is opened.  See Security/SecTrust.h for more information.
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 }
var kCFStreamPropertySSLPeerTrust: CFStringRef; external name '_kCFStreamPropertySSLPeerTrust'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFStreamSSLValidatesCertificateChain
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the certificate chain should be
 *	validated or not.  The value is kCFBooleanTrue by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLValidatesCertificateChain: CFStringRef; external name '_kCFStreamSSLValidatesCertificateChain'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
	
{
 *  kCFStreamPropertySSLSettings
 *  
 *  Discussion:
 *	Stream property key for set operations.  CFDictionaryRef filled
 *	with different security settings.  By default, there are no
 *	security settings.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySSLSettings: CFStringRef; external name '_kCFStreamPropertySSLSettings'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLLevel
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef being one of the security levels.  The value is
 *	kCFStreamSocketSecurityLevelNegotiatedSSL by default (not set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLLevel: CFStringRef; external name '_kCFStreamSSLLevel'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLPeerName
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef overriding the name used for certificate
 *	verification.  Set to kCFNull to prevent name verification. 
 *	Default is the host name with which the streams were created.  If
 *	no host name was used, no peer name will be used.
=======
 *  kCFStreamPropertySSLPeerTrust
 *  
 *  Discussion:
 *	Stream property value for copy operations.  Returns a SecTrustRef
 *	which was a result of the SSL handshake.  This property is not valid before
 *  a stream is opened.  See Security/SecTrust.h for more information.
 *  
 }
var kCFStreamPropertySSLPeerTrust: CFStringRef; external name '_kCFStreamPropertySSLPeerTrust'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
=======
 *  
 }
var kCFStreamPropertySSLPeerTrust: CFStringRef; external name '_kCFStreamPropertySSLPeerTrust'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
>>>>>>> graemeg/cpstrnew
 *  kCFStreamSSLValidatesCertificateChain
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the certificate chain should be
 *	validated or not.  The value is kCFBooleanTrue by default (not
 *	set).
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
 *  
 }
var kCFStreamPropertySSLPeerTrust: CFStringRef; external name '_kCFStreamPropertySSLPeerTrust'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFStreamSSLValidatesCertificateChain
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the certificate chain should be
 *	validated or not.  The value is kCFBooleanTrue by default (not
 *	set).
>>>>>>> graemeg/cpstrnew
=======
 *  kCFStreamPropertySSLPeerTrust
 *  
 *  Discussion:
 *	Stream property value for copy operations.  Returns a SecTrustRef
 *	which was a result of the SSL handshake.  This property is not valid before
 *  a stream is opened.  See Security/SecTrust.h for more information.
 *  
 }
var kCFStreamPropertySSLPeerTrust: CFStringRef; external name '_kCFStreamPropertySSLPeerTrust'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFStreamSSLValidatesCertificateChain
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the certificate chain should be
 *	validated or not.  The value is kCFBooleanTrue by default (not
 *	set).
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamSSLPeerName: CFStringRef; external name '_kCFStreamSSLPeerName'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFArrayRef of SecCertificateRefs, except for index [0], which is
 *	a SecIdentityRef.  See SSLSetCertificate in
 *	Security/SecureTransport.h for more information.
=======
var kCFStreamSSLValidatesCertificateChain: CFStringRef; external name '_kCFStreamSSLValidatesCertificateChain'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
	
{
 *  kCFStreamPropertySSLSettings
 *  
 *  Discussion:
 *	Stream property key for set operations.  CFDictionaryRef filled
 *	with different security settings.  By default, there are no
 *	security settings.
=======
var kCFStreamSSLValidatesCertificateChain: CFStringRef; external name '_kCFStreamSSLValidatesCertificateChain'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
	
{
 *  kCFStreamPropertySSLSettings
 *  
 *  Discussion:
 *	Stream property key for set operations.  CFDictionaryRef filled
 *	with different security settings.  By default, there are no
 *	security settings.
=======
var kCFStreamSSLValidatesCertificateChain: CFStringRef; external name '_kCFStreamSSLValidatesCertificateChain'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
	
{
 *  kCFStreamPropertySSLSettings
 *  
 *  Discussion:
 *	Stream property key for set operations.  CFDictionaryRef filled
 *	with different security settings.  By default, there are no
 *	security settings.
=======
var kCFStreamSSLValidatesCertificateChain: CFStringRef; external name '_kCFStreamSSLValidatesCertificateChain'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
	
{
 *  kCFStreamPropertySSLSettings
 *  
 *  Discussion:
 *	Stream property key for set operations.  CFDictionaryRef filled
 *	with different security settings.  By default, there are no
 *	security settings.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySSLSettings: CFStringRef; external name '_kCFStreamPropertySSLSettings'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLLevel
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef being one of the security levels.  The value is
 *	kCFStreamSocketSecurityLevelNegotiatedSSL by default (not set).
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
var kCFStreamPropertySSLSettings: CFStringRef; external name '_kCFStreamPropertySSLSettings'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLLevel
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef being one of the security levels.  The value is
 *	kCFStreamSocketSecurityLevelNegotiatedSSL by default (not set).
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLLevel: CFStringRef; external name '_kCFStreamSSLLevel'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLPeerName
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef overriding the name used for certificate
 *	verification.  Set to kCFNull to prevent name verification. 
 *	Default is the host name with which the streams were created.  If
 *	no host name was used, no peer name will be used.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamPropertySSLSettings: CFStringRef; external name '_kCFStreamPropertySSLSettings'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLLevel
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef being one of the security levels.  The value is
 *	kCFStreamSocketSecurityLevelNegotiatedSSL by default (not set).
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLLevel: CFStringRef; external name '_kCFStreamSSLLevel'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLPeerName
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef overriding the name used for certificate
 *	verification.  Set to kCFNull to prevent name verification. 
 *	Default is the host name with which the streams were created.  If
 *	no host name was used, no peer name will be used.
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLPeerName: CFStringRef; external name '_kCFStreamSSLPeerName'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFArrayRef of SecCertificateRefs, except for index [0], which is
 *	a SecIdentityRef.  See SSLSetCertificate in
 *	Security/SecureTransport.h for more information.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamPropertySSLSettings: CFStringRef; external name '_kCFStreamPropertySSLSettings'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLLevel
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef being one of the security levels.  The value is
 *	kCFStreamSocketSecurityLevelNegotiatedSSL by default (not set).
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLLevel: CFStringRef; external name '_kCFStreamSSLLevel'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLPeerName
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef overriding the name used for certificate
 *	verification.  Set to kCFNull to prevent name verification. 
 *	Default is the host name with which the streams were created.  If
 *	no host name was used, no peer name will be used.
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLPeerName: CFStringRef; external name '_kCFStreamSSLPeerName'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFArrayRef of SecCertificateRefs, except for index [0], which is
 *	a SecIdentityRef.  See SSLSetCertificate in
 *	Security/SecureTransport.h for more information.
=======
var kCFStreamSSLCertificates: CFStringRef; external name '_kCFStreamSSLCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLIsServer
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the connection is to act as a
 *	server in the SSL process or not.  The value is kCFBooleanFalse
 *	by default (not set).  If set to kCFBooleanTrue, there must be a
 *	valid value for the kCFStreamSSLCertificates key too.
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
var kCFStreamSSLCertificates: CFStringRef; external name '_kCFStreamSSLCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLIsServer
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the connection is to act as a
 *	server in the SSL process or not.  The value is kCFBooleanFalse
 *	by default (not set).  If set to kCFBooleanTrue, there must be a
 *	valid value for the kCFStreamSSLCertificates key too.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamSSLCertificates: CFStringRef; external name '_kCFStreamSSLCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLIsServer
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the connection is to act as a
 *	server in the SSL process or not.  The value is kCFBooleanFalse
 *	by default (not set).  If set to kCFBooleanTrue, there must be a
 *	valid value for the kCFStreamSSLCertificates key too.
=======
var kCFStreamSSLLevel: CFStringRef; external name '_kCFStreamSSLLevel'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
=======
var kCFStreamSSLIsServer: CFStringRef; external name '_kCFStreamSSLIsServer'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

	
{
 * The following properties are considered deprecated in Mac OS 10.6 and later.
 *
 * kCFStreamPropertySSLPeerCertificates: 
 *			The peer certificates are available as part of the SecTrustRef object.  See <Security/SecTrust.h>
 *
 * kCFStreamSSLAllowsExpiredCertificates:
 * kCFStreamSSLAllowsExpiredRoots:
 * kCFStreamSSLAllowsAnyRoot: 
 *			The SSL handshake flags which affect untrusted certificate chain evaluation are deprecated.
 *			Instead, use the single property kCFStreamSSLValidatesCertificateChain to disable certificate
 *			chain checking if the user has decided that it is appropriate to do so.
 }
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
{
<<<<<<< HEAD
 *  kCFStreamSSLPeerName
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef overriding the name used for certificate
 *	verification.  Set to kCFNull to prevent name verification. 
 *	Default is the host name with which the streams were created.  If
 *	no host name was used, no peer name will be used.
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLPeerName: CFStringRef; external name '_kCFStreamSSLPeerName'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFArrayRef of SecCertificateRefs, except for index [0], which is
 *	a SecIdentityRef.  See SSLSetCertificate in
=======
 *  kCFStreamPropertySSLPeerCertificates
 *  
 *  Discussion:
 *	Stream property key for copy operations.  CFArrayRef containing
 *	SecCertificateRefs. See SSLGetPeerCertificates in
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLIsServer: CFStringRef; external name '_kCFStreamSSLIsServer'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

	
{
 * The following properties are considered deprecated in Mac OS 10.6 and later.
 *
 * kCFStreamPropertySSLPeerCertificates: 
 *			The peer certificates are available as part of the SecTrustRef object.  See <Security/SecTrust.h>
 *
 * kCFStreamSSLAllowsExpiredCertificates:
 * kCFStreamSSLAllowsExpiredRoots:
 * kCFStreamSSLAllowsAnyRoot: 
 *			The SSL handshake flags which affect untrusted certificate chain evaluation are deprecated.
 *			Instead, use the single property kCFStreamSSLValidatesCertificateChain to disable certificate
 *			chain checking if the user has decided that it is appropriate to do so.
 }

{$ifc TARGET_OS_MAC}
{
 *  kCFStreamPropertySSLPeerCertificates
 *  
 *  Discussion:
 *	Stream property key for copy operations.  CFArrayRef containing
 *	SecCertificateRefs. See SSLGetPeerCertificates in
>>>>>>> origin/cpstrnew
 *	Security/SecureTransport.h for more information.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamSSLCertificates: CFStringRef; external name '_kCFStreamSSLCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLIsServer
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the connection is to act as a
 *	server in the SSL process or not.  The value is kCFBooleanFalse
 *	by default (not set).  If set to kCFBooleanTrue, there must be a
 *	valid value for the kCFStreamSSLCertificates key too.
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
var kCFStreamSSLIsServer: CFStringRef; external name '_kCFStreamSSLIsServer'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
=======
var kCFStreamPropertySSLPeerCertificates: CFStringRef; external name '_kCFStreamPropertySSLPeerCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew

	
{
<<<<<<< HEAD
 * The following properties are considered deprecated in Mac OS 10.6 and later.
 *
 * kCFStreamPropertySSLPeerCertificates: 
 *			The peer certificates are available as part of the SecTrustRef object.  See <Security/SecTrust.h>
 *
 * kCFStreamSSLAllowsExpiredCertificates:
 * kCFStreamSSLAllowsExpiredRoots:
 * kCFStreamSSLAllowsAnyRoot: 
 *			The SSL handshake flags which affect untrusted certificate chain evaluation are deprecated.
 *			Instead, use the single property kCFStreamSSLValidatesCertificateChain to disable certificate
 *			chain checking if the user has decided that it is appropriate to do so.
 }
<<<<<<< HEAD
=======
var kCFStreamSSLPeerName: CFStringRef; external name '_kCFStreamSSLPeerName'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamPropertySSLPeerCertificates: CFStringRef; external name '_kCFStreamPropertySSLPeerCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  kCFStreamSSLCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFArrayRef of SecCertificateRefs, except for index [0], which is
 *	a SecIdentityRef.  See SSLSetCertificate in
=======
 *  kCFStreamPropertySSLPeerCertificates
 *  
 *  Discussion:
 *	Stream property key for copy operations.  CFArrayRef containing
 *	SecCertificateRefs. See SSLGetPeerCertificates in
>>>>>>> graemeg/cpstrnew
 *	Security/SecureTransport.h for more information.
=======
 *  kCFStreamSSLAllowsExpiredCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired certificates should be
 *	allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
>>>>>>> graemeg/cpstrnew
=======
 *  kCFStreamSSLAllowsExpiredCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired certificates should be
 *	allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamSSLCertificates: CFStringRef; external name '_kCFStreamSSLCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
{
<<<<<<< HEAD
 *  kCFStreamPropertySSLPeerCertificates
 *  
 *  Discussion:
 *	Stream property key for copy operations.  CFArrayRef containing
 *	SecCertificateRefs. See SSLGetPeerCertificates in
 *	Security/SecureTransport.h for more information.
=======
 *  kCFStreamSSLIsServer
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the connection is to act as a
 *	server in the SSL process or not.  The value is kCFBooleanFalse
 *	by default (not set).  If set to kCFBooleanTrue, there must be a
 *	valid value for the kCFStreamSSLCertificates key too.
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLAllowsExpiredCertificates: CFStringRef; external name '_kCFStreamSSLAllowsExpiredCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsExpiredRoots
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired root certificates should
 *	be allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamPropertySSLPeerCertificates: CFStringRef; external name '_kCFStreamPropertySSLPeerCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
var kCFStreamSSLIsServer: CFStringRef; external name '_kCFStreamSSLIsServer'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
=======
var kCFStreamSSLAllowsExpiredRoots: CFStringRef; external name '_kCFStreamSSLAllowsExpiredRoots'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
>>>>>>> origin/cpstrnew

	
{
<<<<<<< HEAD
 * The following properties are considered deprecated in Mac OS 10.6 and later.
 *
 * kCFStreamPropertySSLPeerCertificates: 
 *			The peer certificates are available as part of the SecTrustRef object.  See <Security/SecTrust.h>
 *
 * kCFStreamSSLAllowsExpiredCertificates:
 * kCFStreamSSLAllowsExpiredRoots:
 * kCFStreamSSLAllowsAnyRoot: 
 *			The SSL handshake flags which affect untrusted certificate chain evaluation are deprecated.
 *			Instead, use the single property kCFStreamSSLValidatesCertificateChain to disable certificate
 *			chain checking if the user has decided that it is appropriate to do so.
 }
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamPropertySSLPeerCertificates: CFStringRef; external name '_kCFStreamPropertySSLPeerCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLAllowsExpiredCertificates: CFStringRef; external name '_kCFStreamSSLAllowsExpiredCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsExpiredRoots
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired root certificates should
 *	be allowed or not.  The value is kCFBooleanFalse by default (not
=======
 *  kCFStreamSSLAllowsAnyRoot
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether any root certificates should be
 *	allowed or not.  The value is kCFBooleanFalse by default (not
>>>>>>> origin/cpstrnew
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
var kCFStreamSSLAllowsExpiredRoots: CFStringRef; external name '_kCFStreamSSLAllowsExpiredRoots'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
 *  kCFStreamSSLAllowsExpiredCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired certificates should be
 *	allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
<<<<<<< HEAD
=======
 *  kCFStreamPropertySSLPeerCertificates
 *  
 *  Discussion:
 *	Stream property key for copy operations.  CFArrayRef containing
 *	SecCertificateRefs. See SSLGetPeerCertificates in
 *	Security/SecureTransport.h for more information.
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
 *  kCFStreamSSLAllowsAnyRoot
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether any root certificates should be
 *	allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamSSLAllowsExpiredCertificates: CFStringRef; external name '_kCFStreamSSLAllowsExpiredCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsExpiredRoots
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired root certificates should
 *	be allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLAllowsExpiredRoots: CFStringRef; external name '_kCFStreamSSLAllowsExpiredRoots'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsAnyRoot
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether any root certificates should be
=======
var kCFStreamPropertySSLPeerCertificates: CFStringRef; external name '_kCFStreamPropertySSLPeerCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsExpiredCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired certificates should be
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLAllowsExpiredCertificates: CFStringRef; external name '_kCFStreamSSLAllowsExpiredCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsExpiredRoots
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired root certificates should
 *	be allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLAllowsExpiredRoots: CFStringRef; external name '_kCFStreamSSLAllowsExpiredRoots'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsAnyRoot
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether any root certificates should be
>>>>>>> graemeg/cpstrnew
 *	allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStreamSSLAllowsAnyRoot: CFStringRef; external name '_kCFStreamSSLAllowsAnyRoot'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}

{	kCFStreamNetworkServiceType
 *
 * Discussion:
 * Property key to specify the type of service for the stream.  This
 * allows the system to properly handle the request with respect to
 * routing, suspension behavior and other networking related attributes
 * appropriate for the given service type.  The service types supported
 * are documented below.  Most streams should not need to set this
 * property.
 }
var kCFStreamNetworkServiceType: CFStringRef; external name '_kCFStreamNetworkServiceType'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_4_0) *)

{ supported network service types: }
var kCFStreamNetworkServiceTypeVoIP: CFStringRef; external name '_kCFStreamNetworkServiceTypeVoIP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_4_0) *)		// voice over IP control
var kCFStreamNetworkServiceTypeVideo: CFStringRef; external name '_kCFStreamNetworkServiceTypeVideo'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)		// interactive video
var kCFStreamNetworkServiceTypeBackground: CFStringRef; external name '_kCFStreamNetworkServiceTypeBackground'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)	// background
var kCFStreamNetworkServiceTypeVoice: CFStringRef; external name '_kCFStreamNetworkServiceTypeVoice'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)		// interactive voice data

{
 *  kCFStreamPropertyNoCellular
 *  
 *  Discussion:
 *  Stream property value, for both set and copy operations.
 *  The value is a CFBooleanRef which indicates whether the connection
 *  is allowed to use the built-in celluar radios.  A value of kCFBooleanTrue 
 *  disallows use of cellular interfaces.  kCFBooleanFalse (the default)
 *  allows use of cellular interfaces.
 *  
 }
var kCFStreamPropertyNoCellular: CFStringRef; external name '_kCFStreamPropertyNoCellular'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_5_0) *)

{	kCFStreamPropertyConnectionIsCellular
 *
 * Discussion:
 * Stream property key for copy operations.  Returns a CFBooleanRef value
 * of kCFBooleanTrue if the stream has connected using the built in cellular radios.
 * It returns kCFBooleanFalse if the stream is conneceted over a non-cellular
 * interface or has not yet established a connection.
 }
var kCFStreamPropertyConnectionIsCellular: CFStringRef; external name '_kCFStreamPropertyConnectionIsCellular'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_8, __IPHONE_6_0) *)
=======
var kCFStreamSSLAllowsExpiredCertificates: CFStringRef; external name '_kCFStreamSSLAllowsExpiredCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsExpiredRoots
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired root certificates should
 *	be allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLAllowsExpiredRoots: CFStringRef; external name '_kCFStreamSSLAllowsExpiredRoots'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsAnyRoot
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether any root certificates should be
 *	allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLAllowsAnyRoot: CFStringRef; external name '_kCFStreamSSLAllowsAnyRoot'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew

=======
var kCFStreamSSLAllowsAnyRoot: CFStringRef; external name '_kCFStreamSSLAllowsAnyRoot'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
{
=======
var kCFStreamSSLAllowsAnyRoot: CFStringRef; external name '_kCFStreamSSLAllowsAnyRoot'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}

{
>>>>>>> graemeg/cpstrnew
=======
var kCFStreamSSLAllowsAnyRoot: CFStringRef; external name '_kCFStreamSSLAllowsAnyRoot'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}
=======
>>>>>>> origin/fixes_2.4

{
>>>>>>> origin/cpstrnew
 *  kCFStreamErrorDomainWinSock
 *  
 *  Discussion:
 *	WinSock error domain.  On Win32 platforms, networking errors will
 *	come in this domain.  See <winsock2.h> for values.  Note that
 *	non-networking errors, like ENOMEM, will continue to come in the
 *	POSIX domain as on OS X.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainWinSock: CFIndex; external name '_kCFStreamErrorDomainWinSock'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
=======
=======
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
uses MacTypes,CFStream,CFBase,CFHost,CFNetServices;
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
{$ALIGN MAC68K}
=======
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

>>>>>>> origin/fixes_2.4

{
 *  kCFStreamPropertySSLPeerTrust
 *  
 *  Discussion:
 *	Stream property value for copy operations.  Returns a SecTrustRef
 *	which was a result of the SSL handshake.  This property is not valid before
 *  a stream is opened.  See Security/SecTrust.h for more information.
 *  
 }
var kCFStreamPropertySSLPeerTrust: CFStringRef; external name '_kCFStreamPropertySSLPeerTrust'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFStreamSSLValidatesCertificateChain
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the certificate chain should be
 *	validated or not.  The value is kCFBooleanTrue by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLValidatesCertificateChain: CFStringRef; external name '_kCFStreamSSLValidatesCertificateChain'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)
	
{
 *  kCFStreamPropertySSLSettings
 *  
 *  Discussion:
 *	Stream property key for set operations.  CFDictionaryRef filled
 *	with different security settings.  By default, there are no
 *	security settings.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySSLSettings: CFStringRef; external name '_kCFStreamPropertySSLSettings'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLLevel
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef being one of the security levels.  The value is
 *	kCFStreamSocketSecurityLevelNegotiatedSSL by default (not set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLLevel: CFStringRef; external name '_kCFStreamSSLLevel'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLPeerName
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFStringRef overriding the name used for certificate
 *	verification.  Set to kCFNull to prevent name verification. 
 *	Default is the host name with which the streams were created.  If
 *	no host name was used, no peer name will be used.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLPeerName: CFStringRef; external name '_kCFStreamSSLPeerName'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFArrayRef of SecCertificateRefs, except for index [0], which is
 *	a SecIdentityRef.  See SSLSetCertificate in
 *	Security/SecureTransport.h for more information.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLCertificates: CFStringRef; external name '_kCFStreamSSLCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{
 *  kCFStreamSSLIsServer
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether the connection is to act as a
 *	server in the SSL process or not.  The value is kCFBooleanFalse
 *	by default (not set).  If set to kCFBooleanTrue, there must be a
 *	valid value for the kCFStreamSSLCertificates key too.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLIsServer: CFStringRef; external name '_kCFStreamSSLIsServer'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

	
{
 * The following properties are considered deprecated in Mac OS 10.6 and later.
 *
 * kCFStreamPropertySSLPeerCertificates: 
 *			The peer certificates are available as part of the SecTrustRef object.  See <Security/SecTrust.h>
 *
 * kCFStreamSSLAllowsExpiredCertificates:
 * kCFStreamSSLAllowsExpiredRoots:
 * kCFStreamSSLAllowsAnyRoot: 
 *			The SSL handshake flags which affect untrusted certificate chain evaluation are deprecated.
 *			Instead, use the single property kCFStreamSSLValidatesCertificateChain to disable certificate
 *			chain checking if the user has decided that it is appropriate to do so.
 }

{$ifc TARGET_OS_MAC}
{
 *  kCFStreamPropertySSLPeerCertificates
 *  
 *  Discussion:
 *	Stream property key for copy operations.  CFArrayRef containing
 *	SecCertificateRefs. See SSLGetPeerCertificates in
 *	Security/SecureTransport.h for more information.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySSLPeerCertificates: CFStringRef; external name '_kCFStreamPropertySSLPeerCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsExpiredCertificates
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired certificates should be
 *	allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLAllowsExpiredCertificates: CFStringRef; external name '_kCFStreamSSLAllowsExpiredCertificates'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsExpiredRoots
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether expired root certificates should
 *	be allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLAllowsExpiredRoots: CFStringRef; external name '_kCFStreamSSLAllowsExpiredRoots'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  kCFStreamSSLAllowsAnyRoot
 *  
 *  Discussion:
 *	Security property key for kCFStreamPropertySSLSettings. 
 *	CFBooleanRef indicating whether any root certificates should be
 *	allowed or not.  The value is kCFBooleanFalse by default (not
 *	set).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSSLAllowsAnyRoot: CFStringRef; external name '_kCFStreamSSLAllowsAnyRoot'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  kCFStreamErrorDomainWinSock
 *  
 *  Discussion:
 *	WinSock error domain.  On Win32 platforms, networking errors will
 *	come in this domain.  See <winsock2.h> for values.  Note that
 *	non-networking errors, like ENOMEM, will continue to come in the
 *	POSIX domain as on OS X.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainWinSock: CFIndex; external name '_kCFStreamErrorDomainWinSock'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{
 *  kCFStreamErrorDomainSOCKS
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	SOCKS proxy error domain.  Errors formulated using inlines below.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainSOCKS: SInt32; external name '_kCFStreamErrorDomainSOCKS'; (* attribute const *)

{
CF_INLINE
SInt32 CFSocketStreamSOCKSGetErrorSubdomain(const CFStreamError* error)
	return ((error->error >> 16) & 0x0000FFFF);

CF_INLINE
SInt32 CFSocketStreamSOCKSGetError(CFStreamError* error)
	return (error->error & 0x0000FFFF);
}

const
	kCFStreamErrorSOCKSSubDomainNone = 0; { Error code is a general SOCKS error}
	kCFStreamErrorSOCKSSubDomainVersionCode = 1; { Error code is the version of SOCKS which the server wishes to use}
	kCFStreamErrorSOCKS4SubDomainResponse = 2; { Error code is the status code returned by the server}
	kCFStreamErrorSOCKS5SubDomainUserPass = 3; { Error code is the status code that the server returned}
	kCFStreamErrorSOCKS5SubDomainMethod = 4; { Error code is the server's desired negotiation method}
	kCFStreamErrorSOCKS5SubDomainResponse = 5; { Error code is the response code that the server returned in reply to the connection request}
=======
=======
>>>>>>> origin/fixes_2_2
 *    SOCKS proxy error domain.  Errors formulated using inlines below.
=======
 *	SOCKS proxy error domain.  Errors formulated using inlines below.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainSOCKS: SInt32; external name '_kCFStreamErrorDomainSOCKS'; (* attribute const *)

{
CF_INLINE
SInt32 CFSocketStreamSOCKSGetErrorSubdomain(const CFStreamError* error)
	return ((error->error >> 16) & 0x0000FFFF);

CF_INLINE
SInt32 CFSocketStreamSOCKSGetError(CFStreamError* error)
	return (error->error & 0x0000FFFF);
}

const
<<<<<<< HEAD
  kCFStreamErrorSOCKSSubDomainNone = 0; { Error code is a general SOCKS error }
  kCFStreamErrorSOCKSSubDomainVersionCode = 1; { Error code is the version of SOCKS which the server wishes to use }
  kCFStreamErrorSOCKS4SubDomainResponse = 2; { Error code is the status code returned by the server }
  kCFStreamErrorSOCKS5SubDomainUserPass = 3; { Error code is the status code that the server returned }
  kCFStreamErrorSOCKS5SubDomainMethod = 4; { Error code is the server's desired negotiation method }
  kCFStreamErrorSOCKS5SubDomainResponse = 5; { Error code is the response code that the server returned in reply to the connection request }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kCFStreamErrorSOCKSSubDomainNone = 0; { Error code is a general SOCKS error}
	kCFStreamErrorSOCKSSubDomainVersionCode = 1; { Error code is the version of SOCKS which the server wishes to use}
	kCFStreamErrorSOCKS4SubDomainResponse = 2; { Error code is the status code returned by the server}
	kCFStreamErrorSOCKS5SubDomainUserPass = 3; { Error code is the status code that the server returned}
	kCFStreamErrorSOCKS5SubDomainMethod = 4; { Error code is the server's desired negotiation method}
	kCFStreamErrorSOCKS5SubDomainResponse = 5; { Error code is the response code that the server returned in reply to the connection request}
>>>>>>> origin/fixes_2.4


{ kCFStreamErrorSOCKSSubDomainNone}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFStreamErrorSOCKS5BadResponseAddr = 1;
	kCFStreamErrorSOCKS5BadState = 2;
	kCFStreamErrorSOCKSUnknownClientVersion = 3;

{ kCFStreamErrorSOCKS4SubDomainResponse}
const
	kCFStreamErrorSOCKS4RequestFailed = 91; { request rejected or failed }
	kCFStreamErrorSOCKS4IdentdFailed = 92; { request rejected because SOCKS server cannot connect to identd on the client }
	kCFStreamErrorSOCKS4IdConflict = 93;   { request rejected because the client program and identd report different user-ids }

{ kCFStreamErrorSOCKS5SubDomainMethod}
const
	kSOCKS5NoAcceptableMethod = $FF;  { other values indicate the server's desired method }
=======
=======
>>>>>>> origin/fixes_2_2
  kCFStreamErrorSOCKS5BadResponseAddr = 1;
  kCFStreamErrorSOCKS5BadState  = 2;
  kCFStreamErrorSOCKSUnknownClientVersion = 3;
=======
	kCFStreamErrorSOCKS5BadResponseAddr = 1;
	kCFStreamErrorSOCKS5BadState = 2;
	kCFStreamErrorSOCKSUnknownClientVersion = 3;
>>>>>>> origin/fixes_2.4

{ kCFStreamErrorSOCKS4SubDomainResponse}
const
	kCFStreamErrorSOCKS4RequestFailed = 91; { request rejected or failed }
	kCFStreamErrorSOCKS4IdentdFailed = 92; { request rejected because SOCKS server cannot connect to identd on the client }
	kCFStreamErrorSOCKS4IdConflict = 93;   { request rejected because the client program and identd report different user-ids }

{ kCFStreamErrorSOCKS5SubDomainMethod}
const
<<<<<<< HEAD
  kSOCKS5NoAcceptableMethod     = $0FF;  { other values indicate the server's desired method }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kSOCKS5NoAcceptableMethod = $FF;  { other values indicate the server's desired method }
>>>>>>> origin/fixes_2.4


{
 *  kCFStreamPropertySOCKSProxy
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	Stream property key, for both set and copy operations.  To set a
 *	stream to use a SOCKS proxy, call CFReadStreamSetProperty or
 *	CFWriteStreamSetProperty with the property name set to
 *	kCFStreamPropertySOCKSProxy and the value being a dictionary with
 *	at least the following two keys: kCFStreamPropertySOCKSProxyHost
 *	and kCFStreamPropertySOCKSProxyPort.  The dictionary returned by
 *	SystemConfiguration for SOCKS proxies will work without
 *	alteration.
<<<<<<< HEAD
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
=======
=======
>>>>>>> origin/fixes_2_2
 *    Stream property key, for both set and copy operations.  To set a
 *    stream to use a SOCKS proxy, call CFReadStreamSetProperty or
 *    CFWriteStreamSetProperty with the property name set to
 *    kCFStreamPropertySOCKSProxy and the value being a dictionary with
 *    at least the following two keys: kCFStreamPropertySOCKSProxyHost
 *    and kCFStreamPropertySOCKSProxyPort.  The dictionary returned by
 *    SystemConfiguration for SOCKS proxies will work without
 *    alteration.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
var kCFStreamPropertySOCKSProxy: CFStringRef; external name '_kCFStreamPropertySOCKSProxy'; (* attribute const *)

{
 *  kCFStreamPropertySOCKSProxyHost
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	CFDictionary key for SOCKS proxy information.  The key
 *	kCFStreamPropertySOCKSProxyHost should contain a CFStringRef
 *	value representing the SOCKS proxy host.  Defined to match
 *	kSCPropNetProxiesSOCKSProxy
<<<<<<< HEAD
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
=======
=======
>>>>>>> origin/fixes_2_2
 *    CFDictinary key for SOCKS proxy information.  The key
 *    kCFStreamPropertySOCKSProxyHost should contain a CFStringRef
 *    value representing the SOCKS proxy host.  Defined to match
 *    kSCPropNetProxiesSOCKSProxy
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
var kCFStreamPropertySOCKSProxyHost: CFStringRef; external name '_kCFStreamPropertySOCKSProxyHost'; (* attribute const *)

{
 *  kCFStreamPropertySOCKSProxyPort
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	CFDictionary key for SOCKS proxy information.  The key
 *	kCFStreamPropertySOCKSProxyPort should contain a CFNumberRef
 *	which itself is of type kCFNumberSInt32Type.  This value should
 *	represent the port on which the proxy is listening.  Defined to
 *	match kSCPropNetProxiesSOCKSPort
<<<<<<< HEAD
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
=======
=======
>>>>>>> origin/fixes_2_2
 *    CFDictinary key for SOCKS proxy information.  The key
 *    kCFStreamPropertySOCKSProxyPort should contain a CFNumberRef
 *    which itself is of type kCFNumberSInt32Type.  This value should
 *    represent the port on which the proxy is listening.  Defined to
 *    match kSCPropNetProxiesSOCKSPort
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
var kCFStreamPropertySOCKSProxyPort: CFStringRef; external name '_kCFStreamPropertySOCKSProxyPort'; (* attribute const *)

{
 *  kCFStreamPropertySOCKSVersion
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	CFDictionary key for SOCKS proxy information.  By default, SOCKS5
 *	will be used unless there is a kCFStreamPropertySOCKSVersion key
 *	in the dictionary.  Its value must be
 *	kCFStreamSocketSOCKSVersion4 or kCFStreamSocketSOCKSVersion5 to
 *	set SOCKS4 or SOCKS5, respectively.
<<<<<<< HEAD
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
=======
=======
>>>>>>> origin/fixes_2_2
 *    CFDictinary key for SOCKS proxy information.  By default, SOCKS5
 *    will be used unless there is a kCFStreamPropertySOCKSVersion key
 *    in the dictionary.  Its value must be
 *    kCFStreamSocketSOCKSVersion4 or kCFStreamSocketSOCKSVersion5 to
 *    set SOCKS4 or SOCKS5, respectively.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
var kCFStreamPropertySOCKSVersion: CFStringRef; external name '_kCFStreamPropertySOCKSVersion'; (* attribute const *)

{
 *  kCFStreamSocketSOCKSVersion4
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	CFDictionary value for SOCKS proxy information.  Indcates that
 *	SOCKS will or is using version 4 of the SOCKS protocol.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
=======
=======
>>>>>>> origin/fixes_2_2
 *    CFDictionary value for SOCKS proxy information.  Indcates that
 *    SOCKS will or is using version 4 of the SOCKS protocol.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *	CFDictionary value for SOCKS proxy information.  Indcates that
 *	SOCKS will or is using version 4 of the SOCKS protocol.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
var kCFStreamSocketSOCKSVersion4: CFStringRef; external name '_kCFStreamSocketSOCKSVersion4'; (* attribute const *)

{
 *  kCFStreamSocketSOCKSVersion5
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	CFDictionary value for SOCKS proxy information.  Indcates that
 *	SOCKS will or is using version 5 of the SOCKS protocol.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
=======
=======
>>>>>>> origin/fixes_2_2
 *    CFDictionary value for SOCKS proxy information.  Indcates that
 *    SOCKS will or is using version 5 of the SOCKS protocol.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *	CFDictionary value for SOCKS proxy information.  Indcates that
 *	SOCKS will or is using version 5 of the SOCKS protocol.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
var kCFStreamSocketSOCKSVersion5: CFStringRef; external name '_kCFStreamSocketSOCKSVersion5'; (* attribute const *)

{
 *  kCFStreamPropertySOCKSUser
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	CFDictionary key for SOCKS proxy information.  To set a user name
 *	and/or password, if required, the dictionary must contain the
 *	key(s) kCFStreamPropertySOCKSUser and/or  
 *	kCFStreamPropertySOCKSPassword with the value being the user's
 *	name as a CFStringRef and/or the user's password as a
 *	CFStringRef, respectively.
<<<<<<< HEAD
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
=======
=======
>>>>>>> origin/fixes_2_2
 *    CFDictinary key for SOCKS proxy information.  To set a user name
 *    and/or password, if required, the dictionary must contain the
 *    key(s) kCFStreamPropertySOCKSUser and/or  
 *    kCFStreamPropertySOCKSPassword with the value being the user's
 *    name as a CFStringRef and/or the user's password as a
 *    CFStringRef, respectively.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
var kCFStreamPropertySOCKSUser: CFStringRef; external name '_kCFStreamPropertySOCKSUser'; (* attribute const *)

{
 *  kCFStreamPropertySOCKSPassword
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	CFDictionary key for SOCKS proxy information.  To set a user name
 *	and/or password, if required, the dictionary must contain the
 *	key(s) kCFStreamPropertySOCKSUser and/or  
 *	kCFStreamPropertySOCKSPassword with the value being the user's
 *	name as a CFStringRef and/or the user's password as a
 *	CFStringRef, respectively.
<<<<<<< HEAD
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySOCKSPassword: CFStringRef; external name '_kCFStreamPropertySOCKSPassword'; (* attribute const *)

{
 *  kCFStreamErrorDomainSSL
 *  
 *  Discussion:
 *	Errors located in Security/SecureTransport.h
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainSSL: SInt32; external name '_kCFStreamErrorDomainSSL'; (* attribute const *)

{
 *  kCFStreamPropertySocketSecurityLevel
 *  
 *  Discussion:
 *	Stream property key, for both set and copy operations. To set a
 *	stream to be secure, call CFReadStreamSetProperty or
 *	CFWriteStreamSetPropertywith the property name set to
 *	kCFStreamPropertySocketSecurityLevel and the value being one of
 *	the following values.  Streams may set a security level after
 *	open in order to allow on-the-fly securing of a stream.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySocketSecurityLevel: CFStringRef; external name '_kCFStreamPropertySocketSecurityLevel'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{
 *  kCFStreamSocketSecurityLevelNone
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use no security (default setting).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelNone: CFStringRef; external name '_kCFStreamSocketSecurityLevelNone'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelSSLv2
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use SSLv2 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelSSLv2: CFStringRef; external name '_kCFStreamSocketSecurityLevelSSLv2'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelSSLv3
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use SSLv3 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelSSLv3: CFStringRef; external name '_kCFStreamSocketSecurityLevelSSLv3'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelTLSv1
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use TLSv1 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelTLSv1: CFStringRef; external name '_kCFStreamSocketSecurityLevelTLSv1'; (* attribute const *)

{
=======
=======
>>>>>>> graemeg/cpstrnew

{
 *  kCFStreamSocketSecurityLevelNone
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use no security (default setting).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelNone: CFStringRef; external name '_kCFStreamSocketSecurityLevelNone'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelSSLv2
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use SSLv2 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelSSLv2: CFStringRef; external name '_kCFStreamSocketSecurityLevelSSLv2'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelSSLv3
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use SSLv3 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelSSLv3: CFStringRef; external name '_kCFStreamSocketSecurityLevelSSLv3'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelTLSv1
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use TLSv1 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelTLSv1: CFStringRef; external name '_kCFStreamSocketSecurityLevelTLSv1'; (* attribute const *)

{
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew

{
 *  kCFStreamSocketSecurityLevelNone
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use no security (default setting).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelNone: CFStringRef; external name '_kCFStreamSocketSecurityLevelNone'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelSSLv2
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use SSLv2 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelSSLv2: CFStringRef; external name '_kCFStreamSocketSecurityLevelSSLv2'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelSSLv3
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use SSLv3 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelSSLv3: CFStringRef; external name '_kCFStreamSocketSecurityLevelSSLv3'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelTLSv1
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use TLSv1 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelTLSv1: CFStringRef; external name '_kCFStreamSocketSecurityLevelTLSv1'; (* attribute const *)

{
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  kCFStreamSocketSecurityLevelNegotiatedSSL
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use TLS or SSL with fallback to lower versions. This
 *	is what HTTPS does, for instance.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelNegotiatedSSL: CFStringRef; external name '_kCFStreamSocketSecurityLevelNegotiatedSSL'; (* attribute const *)
{
 *  kCFStreamPropertyShouldCloseNativeSocket
 *  
 *  Discussion:
 *	Set the value to kCFBooleanTrue if the stream should close and
 *	release the underlying native socket when the stream is released.
 *	 Set the value to kCFBooleanFalse to keep the native socket from
 *	closing and releasing when the stream is released. If the stream
 *	was created with a native socket, the default property setting on
 *	the stream is kCFBooleanFalse. The
 *	kCFStreamPropertyShouldCloseNativeSocket can be set through
 *	CFReadStreamSetProperty or CFWriteStreamSetProperty.  The
 *	property can be copied through CFReadStreamCopyProperty or
 *	CFWriteStreamCopyProperty.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
=======
=======
>>>>>>> origin/fixes_2_2
 *    CFDictinary key for SOCKS proxy information.  To set a user name
 *    and/or password, if required, the dictionary must contain the
 *    key(s) kCFStreamPropertySOCKSUser and/or  
 *    kCFStreamPropertySOCKSPassword with the value being the user's
 *    name as a CFStringRef and/or the user's password as a
 *    CFStringRef, respectively.
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySOCKSPassword: CFStringRef; external name '_kCFStreamPropertySOCKSPassword'; (* attribute const *)

{
 *  kCFStreamErrorDomainSSL
 *  
 *  Discussion:
 *	Errors located in Security/SecureTransport.h
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamErrorDomainSSL: SInt32; external name '_kCFStreamErrorDomainSSL'; (* attribute const *)

{
 *  kCFStreamPropertySocketSecurityLevel
 *  
 *  Discussion:
 *	Stream property key, for both set and copy operations. To set a
 *	stream to be secure, call CFReadStreamSetProperty or
 *	CFWriteStreamSetPropertywith the property name set to
 *	kCFStreamPropertySocketSecurityLevel and the value being one of
 *	the following values.  Streams may set a security level after
 *	open in order to allow on-the-fly securing of a stream.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySocketSecurityLevel: CFStringRef; external name '_kCFStreamPropertySocketSecurityLevel'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelNone
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use no security (default setting).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelNone: CFStringRef; external name '_kCFStreamSocketSecurityLevelNone'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelSSLv2
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use SSLv2 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelSSLv2: CFStringRef; external name '_kCFStreamSocketSecurityLevelSSLv2'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelSSLv3
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use SSLv3 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelSSLv3: CFStringRef; external name '_kCFStreamSocketSecurityLevelSSLv3'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelTLSv1
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use TLSv1 security.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelTLSv1: CFStringRef; external name '_kCFStreamSocketSecurityLevelTLSv1'; (* attribute const *)

{
 *  kCFStreamSocketSecurityLevelNegotiatedSSL
 *  
 *  Discussion:
 *	Stream property value, for both set and copy operations.
 *	Indicates to use TLS or SSL with fallback to lower versions. This
 *	is what HTTPS does, for instance.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamSocketSecurityLevelNegotiatedSSL: CFStringRef; external name '_kCFStreamSocketSecurityLevelNegotiatedSSL'; (* attribute const *)
{
 *  kCFStreamPropertyShouldCloseNativeSocket
 *  
 *  Discussion:
 *	Set the value to kCFBooleanTrue if the stream should close and
 *	release the underlying native socket when the stream is released.
 *	 Set the value to kCFBooleanFalse to keep the native socket from
 *	closing and releasing when the stream is released. If the stream
 *	was created with a native socket, the default property setting on
 *	the stream is kCFBooleanFalse. The
 *	kCFStreamPropertyShouldCloseNativeSocket can be set through
 *	CFReadStreamSetProperty or CFWriteStreamSetProperty.  The
 *	property can be copied through CFReadStreamCopyProperty or
 *	CFWriteStreamCopyProperty.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *	Mac OS X:		 in version 10.2 and later
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
>>>>>>> origin/fixes_2.4
 }
var kCFStreamPropertyShouldCloseNativeSocket: CFStringRef; external name '_kCFStreamPropertyShouldCloseNativeSocket'; (* attribute const *)


{
 *  kCFStreamPropertySocketRemoteHost
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Stream property key for copy operations. Returns a CFHostRef if
 *	known, otherwise NULL.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySocketRemoteHost: CFStringRef; external name '_kCFStreamPropertySocketRemoteHost'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Stream property key for copy operations. Returns a CFHostRef if
 *    known, otherwise NULL.
=======
 *	Stream property key for copy operations. Returns a CFHostRef if
 *	known, otherwise NULL.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySocketRemoteHost: CFStringRef; external name '_kCFStreamPropertySocketRemoteHost'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  kCFStreamPropertySocketRemoteNetService
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Stream property key for copy operations. Returns a
 *	CFNetServiceRef if known, otherwise NULL.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySocketRemoteNetService: CFStringRef; external name '_kCFStreamPropertySocketRemoteNetService'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Stream property key for copy operations. Returns a
 *    CFNetServiceRef if known, otherwise NULL.
=======
 *	Stream property key for copy operations. Returns a
 *	CFNetServiceRef if known, otherwise NULL.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertySocketRemoteNetService: CFStringRef; external name '_kCFStreamPropertySocketRemoteNetService'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFStreamCreatePairWithSocketToCFHost()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Given a CFHostRef, this function will create a pair of streams
 *	suitable for connecting to the host.  If there is a failure
 *	during creation, the stream references will be set to NULL.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  streams. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	host:
 *	  A reference to a CFHost to which the streams are desired.  If
 *	  unresolved, the host will be resolved prior to connecting.
 *	
 *	port:
 *	  The port to which the connection should be established.
 *	
 *	readStream:
 *	  A pointer to a CFReadStreamRef which will be set to the new
 *	  read stream instance.  Can be set to NULL if not desired.
 *	
 *	writeStream:
 *	  A pointer to a CFWriteStreamRef which will be set to the new
 *	  write stream instance.  Can be set to NULL if not desired.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFStreamCreatePairWithSocketToCFHost( alloc: CFAllocatorRef; host: CFHostRef; port: SInt32; readStream: CFReadStreamRefPtr { can be NULL }; writeStream: CFWriteStreamRefPtr { can be NULL } ); external name '_CFStreamCreatePairWithSocketToCFHost';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Given a CFHostRef, this function will create a pair of streams
 *    suitable for connecting to the host.  If there is a failure
 *    during creation, the stream references will be set to NULL.
=======
 *	Given a CFHostRef, this function will create a pair of streams
 *	suitable for connecting to the host.  If there is a failure
 *	during creation, the stream references will be set to NULL.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
<<<<<<< HEAD
 *    
 *    alloc:
 *      The CFAllocator which should be used to allocate memory for the
 *      streams. If this reference is not a valid CFAllocator, the
 *      behavior is undefined.
 *    
 *    host:
 *      A reference to a CFHost to which the streams are desired.  If
 *      unresolved, the host will be resolved prior to connecting.
 *    
 *    port:
 *      The port to which the connection should be established.
 *    
 *    readStream:
 *      A pointer to a CFReadStreamRef which will be set to the new
 *      read stream instance.  Can be set to NULL if not desired.
 *    
 *    writeStream:
 *      A pointer to a CFWriteStreamRef which will be set to the new
 *      write stream instance.  Can be set to NULL if not desired.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
procedure CFStreamCreatePairWithSocketToCFHost( alloc: CFAllocatorRef; host: CFHostRef; port: UInt32; readStream: CFReadStreamRefPtr; writeStream: CFWriteStreamRefPtr ); external name '_CFStreamCreatePairWithSocketToCFHost';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *	
 *	alloc:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  streams. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	host:
 *	  A reference to a CFHost to which the streams are desired.  If
 *	  unresolved, the host will be resolved prior to connecting.
 *	
 *	port:
 *	  The port to which the connection should be established.
 *	
 *	readStream:
 *	  A pointer to a CFReadStreamRef which will be set to the new
 *	  read stream instance.  Can be set to NULL if not desired.
 *	
 *	writeStream:
 *	  A pointer to a CFWriteStreamRef which will be set to the new
 *	  write stream instance.  Can be set to NULL if not desired.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFStreamCreatePairWithSocketToCFHost( alloc: CFAllocatorRef; host: CFHostRef; port: SInt32; readStream: CFReadStreamRefPtr { can be NULL }; writeStream: CFWriteStreamRefPtr { can be NULL } ); external name '_CFStreamCreatePairWithSocketToCFHost';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4


{
 *  CFStreamCreatePairWithSocketToNetService()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Given a CFNetService, this function will create a pair of streams
 *	suitable for connecting to the service.  If there is a failure
 *	during creation, the stream references will be set to NULL.
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  streams. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	service:
 *	  A reference to a CFNetService to which the streams are desired.
 *	   If unresolved, the service will be resolved prior to
 *	  connecting.
 *	
 *	readStream:
 *	  A pointer to a CFReadStreamRef which will be set to the new
 *	  read stream instance.  Can be set to NULL if not desired.
 *	
 *	writeStream:
 *	  A pointer to a CFWriteStreamRef which will be set to the new
 *	  write stream instance.  Can be set to NULL if not desired.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFStreamCreatePairWithSocketToNetService( alloc: CFAllocatorRef; service: CFNetServiceRef; readStream: CFReadStreamRefPtr { can be NULL }; writeStream: CFWriteStreamRefPtr { can be NULL } ); external name '_CFStreamCreatePairWithSocketToNetService';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Given a CFNetService, this function will create a pair of streams
 *    suitable for connecting to the service.  If there is a failure
 *    during creation, the stream references will be set to NULL.
=======
 *	Given a CFNetService, this function will create a pair of streams
 *	suitable for connecting to the service.  If there is a failure
 *	during creation, the stream references will be set to NULL.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	alloc:
 *	  The CFAllocator which should be used to allocate memory for the
 *	  streams. If this reference is not a valid CFAllocator, the
 *	  behavior is undefined.
 *	
 *	service:
 *	  A reference to a CFNetService to which the streams are desired.
 *	   If unresolved, the service will be resolved prior to
 *	  connecting.
 *	
 *	readStream:
 *	  A pointer to a CFReadStreamRef which will be set to the new
 *	  read stream instance.  Can be set to NULL if not desired.
 *	
 *	writeStream:
 *	  A pointer to a CFWriteStreamRef which will be set to the new
 *	  write stream instance.  Can be set to NULL if not desired.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.3 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
procedure CFStreamCreatePairWithSocketToNetService( alloc: CFAllocatorRef; service: CFNetServiceRef; readStream: CFReadStreamRefPtr { can be NULL }; writeStream: CFWriteStreamRefPtr { can be NULL } ); external name '_CFStreamCreatePairWithSocketToNetService';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4
{
 *  CFStreamSocketSecurityProtocol
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	These enum values and CFSocketStreamPairSetSecurityProtocol have
 *	been deprecated in favor of CFReadStreamSetProperty and
 *	CFWriteStreamSetProperty with the previously mentioned property
 *	and values.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *    These enum values and CFSocketStreamPairSetSecurityProtocol have
 *    been deprecated in favor of CFReadStreamSetProperty and
 *    CFWriteStreamSetProperty with the previously mentioned property
 *    and values.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 }
type
	CFStreamSocketSecurityProtocol = SInt32;
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
   * DEPRECATED, use kCFStreamSocketSecurityLevelNone
   }
	kCFStreamSocketSecurityNone = 0;
=======
=======
>>>>>>> origin/fixes_2_2
  {
   * DEPRECATED, use kCFStreamSocketSecurityLevelNone
   }
  kCFStreamSocketSecurityNone   = 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
   * DEPRECATED, use kCFStreamSocketSecurityLevelNone
   }
	kCFStreamSocketSecurityNone = 0;
>>>>>>> origin/fixes_2.4

  {
   * DEPRECATED, use kCFStreamSocketSecurityLevelSSLv2
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFStreamSocketSecuritySSLv2 = 1;
=======
  kCFStreamSocketSecuritySSLv2  = 1;
>>>>>>> graemeg/fixes_2_2
=======
  kCFStreamSocketSecuritySSLv2  = 1;
>>>>>>> origin/fixes_2_2
=======
	kCFStreamSocketSecuritySSLv2 = 1;
>>>>>>> origin/fixes_2.4

  {
   * DEPRECATED, use kCFStreamSocketSecurityLevelSSLv3
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFStreamSocketSecuritySSLv3 = 2;
=======
  kCFStreamSocketSecuritySSLv3  = 2;
>>>>>>> graemeg/fixes_2_2
=======
  kCFStreamSocketSecuritySSLv3  = 2;
>>>>>>> origin/fixes_2_2
=======
	kCFStreamSocketSecuritySSLv3 = 2;
>>>>>>> origin/fixes_2.4

  {
   * DEPRECATED, use kCFStreamSocketSecurityLevelNegotiatedSSL
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFStreamSocketSecuritySSLv23 = 3;
=======
  kCFStreamSocketSecuritySSLv23 = 3;
>>>>>>> graemeg/fixes_2_2
=======
  kCFStreamSocketSecuritySSLv23 = 3;
>>>>>>> origin/fixes_2_2
=======
	kCFStreamSocketSecuritySSLv23 = 3;
>>>>>>> origin/fixes_2.4

  {
   * DEPRECATED, use kCFStreamSocketSecurityLevelTLSv1
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFStreamSocketSecurityTLSv1 = 4;
=======
  kCFStreamSocketSecurityTLSv1  = 4;
>>>>>>> graemeg/fixes_2_2
=======
  kCFStreamSocketSecurityTLSv1  = 4;
>>>>>>> origin/fixes_2_2
=======
	kCFStreamSocketSecurityTLSv1 = 4;
>>>>>>> origin/fixes_2.4

{
 *  CFSocketStreamPairSetSecurityProtocol()   *** DEPRECATED ***
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *	CFSocketStreamPairSetSecurityProtocol has been deprecated in
 *	favor of CFReadStreamSetProperty and CFWriteStreamSetProperty
 *	with the previously mentioned property and values.  Sets the
 *	security level on a pair of streams.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	socketReadStream:
 *	  Read stream reference which is to have its security level
 *	  changed.
 *	
 *	socketWriteStream:
 *	  Write stream reference which is to have its security level
 *	  changed.
 *	
 *	securityProtocol:
 *	  CFStreamSocketSecurityProtocol enum indicating the security
 *	  level to be set.
 *  
 *  Result:
 *	Returns TRUE if the settings were placed on the stream, FALSE
 *	otherwise.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework but deprecated in 10.2
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFSocketStreamPairSetSecurityProtocol( socketReadStream: CFReadStreamRef; socketWriteStream: CFWriteStreamRef; securityProtocol: CFStreamSocketSecurityProtocol ): Boolean; external name '_CFSocketStreamPairSetSecurityProtocol';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_2,__IPHONE_NA,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}

{
 *  kCFStreamPropertyProxyLocalBypass
 *  
 *  Discussion:
 *	CFDictionary key for proxy information.  It matches
 *	kSCPropNetProxiesExcludeSimpleHostnames defined in
 *	SCSchemaDefinitions.h.  CFNumber (0 or 1) indicating to bypass
 *	the proxies for simple hostnames (names without dots).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyProxyLocalBypass: CFStringRef; external name '_kCFStreamPropertyProxyLocalBypass'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    CFSocketStreamPairSetSecurityProtocol has been deprecated in
 *    favor of CFReadStreamSetProperty and CFWriteStreamSetProperty
 *    with the previously mentioned property and values.  Sets the
 *    security level on a pair of streams.
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *	Thread safe
 *  
 *  Parameters:
 *	
 *	socketReadStream:
 *	  Read stream reference which is to have its security level
 *	  changed.
 *	
 *	socketWriteStream:
 *	  Write stream reference which is to have its security level
 *	  changed.
 *	
 *	securityProtocol:
 *	  CFStreamSocketSecurityProtocol enum indicating the security
 *	  level to be set.
 *  
 *  Result:
 *	Returns TRUE if the settings were placed on the stream, FALSE
 *	otherwise.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.1 and later in CoreServices.framework but deprecated in 10.2
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFSocketStreamPairSetSecurityProtocol( socketReadStream: CFReadStreamRef; socketWriteStream: CFWriteStreamRef; securityProtocol: CFStreamSocketSecurityProtocol ): Boolean; external name '_CFSocketStreamPairSetSecurityProtocol';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_2,__IPHONE_NA,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}

{
 *  kCFStreamPropertyProxyLocalBypass
 *  
 *  Discussion:
 *	CFDictionary key for proxy information.  It matches
 *	kSCPropNetProxiesExcludeSimpleHostnames defined in
 *	SCSchemaDefinitions.h.  CFNumber (0 or 1) indicating to bypass
 *	the proxies for simple hostnames (names without dots).
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.4 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFStreamPropertyProxyLocalBypass: CFStringRef; external name '_kCFStreamPropertyProxyLocalBypass'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_2_0) *)

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
