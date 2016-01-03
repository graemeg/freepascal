{
	 File:	   CFNetwork/CFProxySupport.h
 
	 Contains:   Support for computing which proxy applies when
 
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

unit CFProxySupport;
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
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
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
uses MacTypes, CFArray, CFBase, CFDictionary, CFURL, CFError, CFRunLoop, CFStream;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{ 
	These APIs return arrays of dictionaries, where each dictionary describes a single proxy. 
	The arrays represent the order in which the proxies should be tried - try to download the URL
	using the first entry in the array, and if that fails, try using the second entry, and so on.

	The keys to the proxy dictionaries follow the function declarations; every proxy dictionary 
	will have an entry for kCFProxyTypeKey.  If the type is anything except 
	kCFProxyTypeAutoConfigurationURL, the dictionary will also have entries for the proxy's host
	and port (under kCFProxyHostNameKey and kCFProxyPortNumberKey respectively).  If the type is
	kCFProxyTypeAutoConfigurationURL, it will have an entry for kCFProxyAutoConfigurationURLKey.  
	
	The keys for username and password are optional and will only be present if the username 
	or password could be extracted from the information passed in (i.e. either the URL itself
	or the proxy dictionary supplied).  These APIs do not consult any external credential stores
	(such as the Keychain).
}


{!
	@function CFNetworkCopySystemProxySettings
	@discussion Returns a CFDictionary containing the current system internet proxy settings.
	@result Returns a dictionary containing key-value pairs that represent
		the current internet proxy settings.  See below for definitions of the keys and
		values.
		NULL if no proxy settings have been defined or if an error
		was encountered.
		The caller is responsible for releasing the returned dictionary.
}
function CFNetworkCopySystemProxySettings: CFDictionaryRef; external name '_CFNetworkCopySystemProxySettings';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)

	
{
 *  CFNetworkCopyProxiesForURL()
 *  
 *  Discussion:
 *	Given a URL and a proxy dictionary, determines the ordered list
 *	of proxies that should be used to download the given URL.
 *  
 *  Parameters:
 *	
 *	url:
 *	  The URL to be accessed
 *	
 *	proxySettings:
 *	  A dictionary describing the available proxy settings; the
 *	  dictionary's format should match the dictionary returned
 *	  by CFNetworkCopySystemProxySettings described below.
 *  
 *  Result:
 *	An array of dictionaries; each dictionary describes a single
 *	proxy.  See the comment at the top of this file for how to
 *	interpret the returned dictionaries.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFNetworkCopyProxiesForURL( url: CFURLRef; proxySettings: CFDictionaryRef ): CFArrayRef; external name '_CFNetworkCopyProxiesForURL';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)


{
 *  CFProxyAutoConfigurationResultCallback
 *  
 *  Discussion:
 *	Callback function to be called when a PAC file computation
 *	(initiated by either CFNetworkExecuteProxyAutoConfigurationScript
 *	or CFNetworkExecuteProxyAutoConfigurationURL) has completed.
 *  
 *  Parameters:
 *	
 *	client:
 *	  The client reference passed in to
 *	  CFNetworkExecuteProxyAutoConfigurationScript or
 *	  CFNetworkExecuteProxyAutoConfigurationURL
 *	
 *	proxyList:
 *	  Upon success, the list of proxies returned by the
 *	  autoconfiguration script.  The list has the same format as
 *	  returned by CFProxyCopyProxiesForURL, above, except that no
 *	  entry may be of type kCFProxyTypeAutoConfigurationURL.  Note
 *	  that if the client wishes to keep this list, they must retain
 *	  it when they receive this callback.
 *	
 *	error:
 *	  Upon failure, an error object explaining the failure.
 }
type
	CFProxyAutoConfigurationResultCallback = procedure( client: UnivPtr; proxyList: CFArrayRef; error: CFErrorRef );

{
 *  CFNetworkCopyProxiesForAutoConfigurationScript()
 *  
 *  Discussion:
 *	Synchronously executes the given proxy autoconfiguration script
 *	and returns a valid proxyList and NULL error upon success or a
 *	NULL proxyList and valid error on failure.
 *  
 *  Parameters:
 *	
 *	proxyAutoConfigurationScript:
 *	  A CFString containing the code of the script to be executed.
 *	
 *	targetURL:
 *	  The URL that should be input in to the autoconfiguration script.
 *	
 *	error:
 *	  A return argument that will contain a valid error in case of
 *	  failure.
 *  
 *  Result:
 *	An array of dictionaries describing the proxies returned by the
 *	script or NULL on failure.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFNetworkCopyProxiesForAutoConfigurationScript( proxyAutoConfigurationScript: CFStringRef; targetURL: CFURLRef; var error: CFErrorRef ): CFArrayRef; external name '_CFNetworkCopyProxiesForAutoConfigurationScript';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)


{
 *  CFNetworkExecuteProxyAutoConfigurationScript()
 *  
 *  Discussion:
 *	Begins the process of executing proxyAutoConfigurationScript to
 *	determine the correct proxy to use to retrieve targetURL.  The
 *	caller should schedule the returned run loop source; when the
 *	results are found, the caller's callback will be called via the
 *	run loop, passing a valid proxyList and NULL error upon success,
 *	or a NULL proxyList and valid error on failure.  The caller
 *	should invalidate the returned run loop source if it wishes to
 *	terminate the request before completion. The returned
 *	RunLoopSource will be removed from all run loops and modes on
 *	which it was scheduled after the callback returns.
 *  
 *  Parameters:
 *	
 *	proxyAutoConfigurationScript:
 *	  A CFString containing the code of the script to be executed.
 *	
 *	targetURL:
 *	  The URL that should be passed to the autoconfiguration script.
 *	
 *	cb:
 *	  A client callback to notify the caller of completion.
 *	
 *	clientContext:
 *	  a stream context containing a client info object and optionally
 *	  retain / release callbacks for said info object.
 *  
 *  Result:
 *	A CFRunLoopSource which the client can use to schedule execution
 *	of the AutoConfiguration Script.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFNetworkExecuteProxyAutoConfigurationScript( proxyAutoConfigurationScript: CFStringRef; targetURL: CFURLRef; cb: CFProxyAutoConfigurationResultCallback; var clientContext: CFStreamClientContext ): CFRunLoopSourceRef; external name '_CFNetworkExecuteProxyAutoConfigurationScript';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)


{
 *  CFNetworkExecuteProxyAutoConfigurationURL()
 *  
 *  Discussion:
 *	As CFNetworkExecuteProxyAutoConfigurationScript(), above, except
 *	that CFNetworkExecuteProxyAutoConfigurationURL will additionally
 *	download the contents of proxyAutoConfigURL, convert it to a
 *	JavaScript string, and then execute that script.
 *  Ownership for the returned CFRunLoopSourceRef follows the copy rule, 
 *  the client is responsible for releasing the object.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
function CFNetworkExecuteProxyAutoConfigurationURL( proxyAutoConfigURL: CFURLRef; targetURL: CFURLRef; cb: CFProxyAutoConfigurationResultCallback; var clientContext: CFStreamClientContext ): CFRunLoopSourceRef; external name '_CFNetworkExecuteProxyAutoConfigurationURL';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)


{
 *  kCFProxyTypeKey
 *  
 *  Discussion:
 *	Key for the type of proxy being represented; value will be one of
 *	the kCFProxyType constants listed below.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyTypeKey: CFStringRef; external name '_kCFProxyTypeKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFProxyHostNameKey
 *  
 *  Discussion:
 *	Key for the proxy's hostname; value is a CFString.  Note that
 *	this may be an IPv4 or IPv6 dotted-IP string.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyHostNameKey: CFStringRef; external name '_kCFProxyHostNameKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFProxyPortNumberKey
 *  
 *  Discussion:
 *	Key for the proxy's port number; value is a CFNumber specifying
 *	the port on which to contact the proxy
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyPortNumberKey: CFStringRef; external name '_kCFProxyPortNumberKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFProxyAutoConfigurationURLKey
 *  
 *  Discussion:
 *	Key for the proxy's PAC file location; this key is only present
 *	if the proxy's type is kCFProxyTypeAutoConfigurationURL.  Value
 *	is a CFURL specifying the location of a proxy auto-configuration
 *	file
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyAutoConfigurationURLKey: CFStringRef; external name '_kCFProxyAutoConfigurationURLKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  kCFProxyAutoConfigurationJavaScriptKey
 *  
 *  Discussion:
 *	Key for the proxy's PAC script
 *	The value is a CFString that contains the full JavaScript soure text for the PAC file.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyAutoConfigurationJavaScriptKey: CFStringRef; external name '_kCFProxyAutoConfigurationJavaScriptKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_3_0) *)


{
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
 *  kCFProxyUsernameKey
 *  
 *  Discussion:
 *	Key for the username to be used with the proxy; value is a
 *	CFString. Note that this key will only be present if the username
 *	could be extracted from the information passed in.  No external
 *	credential stores (like the Keychain) are consulted.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyUsernameKey: CFStringRef; external name '_kCFProxyUsernameKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{
 *  kCFProxyPasswordKey
 *  
 *  Discussion:
 *	Key for the password to be used with the proxy; value is a
 *	CFString. Note that this key will only be present if the username
 *	could be extracted from the information passed in.  No external
 *	credential stores (like the Keychain) are consulted.
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyPasswordKey: CFStringRef; external name '_kCFProxyPasswordKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)

{ 
	Possible values for kCFProxyTypeKey:
	kCFProxyTypeNone - no proxy should be used; contact the origin server directly
	kCFProxyTypeHTTP - the proxy is an HTTP proxy
	kCFProxyTypeHTTPS - the proxy is a tunneling proxy as used for HTTPS
	kCFProxyTypeSOCKS - the proxy is a SOCKS proxy
	kCFProxyTypeFTP - the proxy is an FTP proxy
	kCFProxyTypeAutoConfigurationURL - the proxy is specified by a proxy autoconfiguration (PAC) file
}
{
 *  kCFProxyTypeNone
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyTypeNone: CFStringRef; external name '_kCFProxyTypeNone'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
{
 *  kCFProxyTypeHTTP
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyTypeHTTP: CFStringRef; external name '_kCFProxyTypeHTTP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
{
 *  kCFProxyTypeHTTPS
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyTypeHTTPS: CFStringRef; external name '_kCFProxyTypeHTTPS'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
{
 *  kCFProxyTypeSOCKS
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyTypeSOCKS: CFStringRef; external name '_kCFProxyTypeSOCKS'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
{
 *  kCFProxyTypeFTP
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyTypeFTP: CFStringRef; external name '_kCFProxyTypeFTP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
{
 *  kCFProxyTypeAutoConfigurationURL
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyTypeAutoConfigurationURL: CFStringRef; external name '_kCFProxyTypeAutoConfigurationURL'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
 *
 *
 }
var kCFProxyTypeAutoConfigurationJavaScript: CFStringRef; external name '_kCFProxyTypeAutoConfigurationJavaScript'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_3_0) *)
	
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
{
 *  kCFProxyAutoConfigHTTPResponse
 *  
 *  Availability:
 *	Mac OS X:		 in version 10.5 and later in CoreServices.framework
 *	CarbonLib:		not available
 *	Non-Carbon CFM:   not available
 }
var kCFProxyAutoConfigurationHTTPResponseKey: CFStringRef; external name '_kCFProxyAutoConfigurationHTTPResponseKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_2_0) *)
	

{$ifc TARGET_OS_MAC}
{
 *  kCFNetworkProxiesExceptionsList
 *  
 *  Discussion:
 *	Key for the list of host name patterns that should bypass the proxy; value is a
 *	CFArray of CFStrings.  
 }
var kCFNetworkProxiesExceptionsList: CFStringRef; external name '_kCFNetworkProxiesExceptionsList'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesExcludeSimpleHostnames
 *  
 *  Discussion:
 *	Key whose value indicates if simple hostnames will be excluded; value is a
 *	CFNumber.  Simple hostnames will be excluded if the key is present and has a 
 *	non-zero value.
 }
var kCFNetworkProxiesExcludeSimpleHostnames: CFStringRef; external name '_kCFNetworkProxiesExcludeSimpleHostnames'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesFTPEnable
 *  
 *  Discussion:
 *	Key for the enabled status of the ftp proxy; value is a
 *	CFNumber.  The proxy is enabled if the key is present and has a non-zero value.
 }
var kCFNetworkProxiesFTPEnable: CFStringRef; external name '_kCFNetworkProxiesFTPEnable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesFTPPassive
 *  
 *  Discussion:
 *	Key for the state of passive mode for the ftp proxy; value is a
 *	CFNumber.  A value of one indicates that passive mode is enabled, a value
 *	of zero indicates that passive mode is not enabled.
 }
var kCFNetworkProxiesFTPPassive: CFStringRef; external name '_kCFNetworkProxiesFTPPassive'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesFTPPort
 *  
 *  Discussion:
 *	Key for the port number associated with the ftp proxy; value is a
 *	CFNumber which is the port number.
 }
var kCFNetworkProxiesFTPPort: CFStringRef; external name '_kCFNetworkProxiesFTPPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesFTPProxy
 *  
 *  Discussion:
 *	Key for the host name associated with the ftp proxy; value is a
 *	CFString which is the proxy host name.
 }
var kCFNetworkProxiesFTPProxy: CFStringRef; external name '_kCFNetworkProxiesFTPProxy'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesGopherEnable
 *  
 *  Discussion:
 *	Key for the enabled status of the gopher proxy; value is a
 *	CFNumber.  The proxy is enabled if the key is present and has a non-zero value.
 }
var kCFNetworkProxiesGopherEnable: CFStringRef; external name '_kCFNetworkProxiesGopherEnable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesGopherPort
 *  
 *  Discussion:
 *	Key for the port number associated with the gopher proxy; value is a
 *	CFNumber which is the port number.
 }
var kCFNetworkProxiesGopherPort: CFStringRef; external name '_kCFNetworkProxiesGopherPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesGopherProxy
 *  
 *  Discussion:
 *	Key for the host name associated with the gopher proxy; value is a
 *	CFString which is the proxy host name.
 }
var kCFNetworkProxiesGopherProxy: CFStringRef; external name '_kCFNetworkProxiesGopherProxy'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesHTTPEnable
 *  
 *  Discussion:
 *	Key for the enabled status of the HTTP proxy; value is a
 *	CFNumber.  The proxy is enabled if the key is present and has a non-zero value.
 }
var kCFNetworkProxiesHTTPEnable: CFStringRef; external name '_kCFNetworkProxiesHTTPEnable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)
{
 *  kCFNetworkProxiesHTTPPort
 *  
 *  Discussion:
 *	Key for the port number associated with the HTTP proxy; value is a
 *	CFNumber which is the port number.
 }
var kCFNetworkProxiesHTTPPort: CFStringRef; external name '_kCFNetworkProxiesHTTPPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)
{
 *  kCFNetworkProxiesHTTPProxy
 *  
 *  Discussion:
 *	Key for the host name associated with the HTTP proxy; value is a
 *	CFString which is the proxy host name.
 }
var kCFNetworkProxiesHTTPProxy: CFStringRef; external name '_kCFNetworkProxiesHTTPProxy'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)
{
 *  kCFNetworkProxiesHTTPSEnable
 *  
 *  Discussion:
 *	Key for the enabled status of the HTTPS proxy; value is a
 *	CFNumber.  The proxy is enabled if the key is present and has a non-zero value.
 }
var kCFNetworkProxiesHTTPSEnable: CFStringRef; external name '_kCFNetworkProxiesHTTPSEnable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesHTTPSPort
 *  
 *  Discussion:
 *	Key for the port number associated with the HTTPS proxy; value is a
 *	CFNumber which is the port number.
 }
var kCFNetworkProxiesHTTPSPort: CFStringRef; external name '_kCFNetworkProxiesHTTPSPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesHTTPSProxy
 *  
 *  Discussion:
 *	Key for the host name associated with the HTTPS proxy; value is a
 *	CFString which is the proxy host name.
 }
var kCFNetworkProxiesHTTPSProxy: CFStringRef; external name '_kCFNetworkProxiesHTTPSProxy'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesRTSPEnable
 *  
 *  Discussion:
 *	Key for the enabled status of the RTSP proxy; value is a
 *	CFNumber.  The proxy is enabled if the key is present and has a non-zero value.
 }
var kCFNetworkProxiesRTSPEnable: CFStringRef; external name '_kCFNetworkProxiesRTSPEnable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesRTSPPort
 *  
 *  Discussion:
 *	Key for the port number associated with the RTSP proxy; value is a
 *	CFNumber which is the port number.
 }
var kCFNetworkProxiesRTSPPort: CFStringRef; external name '_kCFNetworkProxiesRTSPPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesRTSPProxy
 *  
 *  Discussion:
 *	Key for the host name associated with the RTSP proxy; value is a
 *	CFString which is the proxy host name.
 }
var kCFNetworkProxiesRTSPProxy: CFStringRef; external name '_kCFNetworkProxiesRTSPProxy'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesSOCKSEnable
 *  
 *  Discussion:
 *	Key for the enabled status of the SOCKS proxy; value is a
 *	CFNumber.  The proxy is enabled if the key is present and has a non-zero value.
 }
var kCFNetworkProxiesSOCKSEnable: CFStringRef; external name '_kCFNetworkProxiesSOCKSEnable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesSOCKSPort
 *  
 *  Discussion:
 *	Key for the port number associated with the SOCKS proxy; value is a
 *	CFNumber which is the port number.
 }
var kCFNetworkProxiesSOCKSPort: CFStringRef; external name '_kCFNetworkProxiesSOCKSPort'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesSOCKSProxy
 *  
 *  Discussion:
 *	Key for the host name associated with the SOCKS proxy; value is a
 *	CFString which is the proxy host name.
 }
var kCFNetworkProxiesSOCKSProxy: CFStringRef; external name '_kCFNetworkProxiesSOCKSProxy'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{
 *  kCFNetworkProxiesProxyAutoConfigEnable
 *  
 *  Discussion:
 *	Key for the enabled status ProxyAutoConfig (PAC); value is a
 *	CFNumber.  ProxyAutoConfig is enabled if the key is present and has a non-zero value.
 }
var kCFNetworkProxiesProxyAutoConfigEnable: CFStringRef; external name '_kCFNetworkProxiesProxyAutoConfigEnable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  kCFNetworkProxiesProxyAutoConfigURLString
=======
 *  kCFNetworkProxiesSOCKSProxy
>>>>>>> graemeg/cpstrnew
=======
 *  kCFNetworkProxiesSOCKSProxy
>>>>>>> graemeg/cpstrnew
=======
 *  kCFNetworkProxiesSOCKSProxy
>>>>>>> graemeg/cpstrnew
=======
 *  kCFNetworkProxiesSOCKSProxy
>>>>>>> origin/cpstrnew
=======
 *  kCFNetworkProxiesSOCKSProxy
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *	Key for the url which indicates the location of the ProxyAutoConfig (PAC) file; value is a
 *	CFString which is url for the PAC file.
 }
var kCFNetworkProxiesProxyAutoConfigURLString: CFStringRef; external name '_kCFNetworkProxiesProxyAutoConfigURLString'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_2_0) *)
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * kCFNetworkProxiesProxyAutoConfigJavaScript
 *
 * Discussion:
 * Key for the string which is the full JavaScript source of the ProxyAutoConfig (PAC) script;  value is a
 * CFString with is the full text source of the PAC script.
 }
var kCFNetworkProxiesProxyAutoConfigJavaScript: CFStringRef; external name '_kCFNetworkProxiesProxyAutoConfigJavaScript'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7,__IPHONE_3_0) *)
	
{
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
 *  kCFNetworkProxiesProxyAutoDiscoveryEnable
 *  
 *  Discussion:
 *	Key for the enabled status of proxy auto discovery; value is a
 *	CFNumber.  Proxy auto discovery is enabled if the key is present and has a non-zero value.
 }
var kCFNetworkProxiesProxyAutoDiscoveryEnable: CFStringRef; external name '_kCFNetworkProxiesProxyAutoDiscoveryEnable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_6,__IPHONE_NA) *)
{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
