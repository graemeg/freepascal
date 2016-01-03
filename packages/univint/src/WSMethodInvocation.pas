{
     File:       OSServices/WSMethodInvocation.h
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Contains:   *** DEPRECATED *** WebServicesCore Method Invocation API
 
     Copyright:  (c) 2002-2011 Apple Inc. All rights reserved
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
     Contains:   WebServicesCore Method Invocation API
 
     Version:    OSServices-352~2
 
     Copyright:  © 2002-2008 by Apple Computer, Inc., all rights reserved
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
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
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
{       Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit WSMethodInvocation;
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
uses MacTypes,CFBase,CFArray,CFData,CFDictionary,CFRunLoop,CFXMLNode,WSTypes;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
    WSMethodInvocation
 }
{
    Dictionary entry if the invocation result is not a fault.  This is
    always available in method responses, although for SOAP messages,
    it may be more correctly to query the result dictionary for the
    specific field you're interested in.  What this really means is
    that the dictionary returned by the invocation may contain more
    than one value, wherein the result data is duplicated several
    times.  If you don't know what to ask for to dump the reply, you
    can ask for this key.  If you do know what you want, you should
    request that field expliclty.

    You can also specify the name of the reply parameter in the
    invocation using kWSMethodInvocationResultParameterName.  This will
    add an alias for the given name to the result dictionary such that
    kWSMethodInvocationResult will always return the correct
    parameter.  (This won't work for multi-value returns, however.)
}
var kWSMethodInvocationResult: CFStringRef; external name '_kWSMethodInvocationResult'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{
    Dictionary entries if the result is a fault
}

{ a CFString }
var kWSFaultString: CFStringRef; external name '_kWSFaultString'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ a CFNumber }
var kWSFaultCode: CFStringRef; external name '_kWSFaultCode'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ a CFString or CFDictionary, or NULL }
var kWSFaultExtra: CFStringRef; external name '_kWSFaultExtra'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{
    Dictionary entries if the result is a fault
}
var kWSFaultString: CFStringRef; external name '_kWSFaultString'; (* attribute const *)  { a CFString }
var kWSFaultCode: CFStringRef; external name '_kWSFaultCode'; (* attribute const *)    { a CFNumber }
var kWSFaultExtra: CFStringRef; external name '_kWSFaultExtra'; (* attribute const *)   { a CFString or CFDictionary, or NULL }
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
    If the result is a fault, and if the value of kWSFaultString in
    the reply dictionary is kWSNetworkStreamFaultString, then
    kWSFaultExtra will be a dictionary indicating the network error
    and kWSFaultCode is ignored in this case.  See
    <CoreFoundation/CFStream.h> for details on what the domain and
    error numbers mean.
}
var kWSNetworkStreamFaultString: CFStringRef; external name '_kWSNetworkStreamFaultString'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ A CFString (for debug purposes only) }
var kWSStreamErrorMessage: CFStringRef; external name '_kWSStreamErrorMessage'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ A CFNumberRef }
var kWSStreamErrorDomain: CFStringRef; external name '_kWSStreamErrorDomain'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ A CFNumberRef }
var kWSStreamErrorError: CFStringRef; external name '_kWSStreamErrorError'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
var kWSStreamErrorMessage: CFStringRef; external name '_kWSStreamErrorMessage'; (* attribute const *)           { A CFString (for debug purposes only) }
var kWSStreamErrorDomain: CFStringRef; external name '_kWSStreamErrorDomain'; (* attribute const *)            { A CFNumberRef }
var kWSStreamErrorError: CFStringRef; external name '_kWSStreamErrorError'; (* attribute const *)             { A CFNumberRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSStreamErrorMessage: CFStringRef; external name '_kWSStreamErrorMessage'; (* attribute const *)           { A CFString (for debug purposes only) }
var kWSStreamErrorDomain: CFStringRef; external name '_kWSStreamErrorDomain'; (* attribute const *)            { A CFNumberRef }
var kWSStreamErrorError: CFStringRef; external name '_kWSStreamErrorError'; (* attribute const *)             { A CFNumberRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSStreamErrorMessage: CFStringRef; external name '_kWSStreamErrorMessage'; (* attribute const *)           { A CFString (for debug purposes only) }
var kWSStreamErrorDomain: CFStringRef; external name '_kWSStreamErrorDomain'; (* attribute const *)            { A CFNumberRef }
var kWSStreamErrorError: CFStringRef; external name '_kWSStreamErrorError'; (* attribute const *)             { A CFNumberRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSStreamErrorMessage: CFStringRef; external name '_kWSStreamErrorMessage'; (* attribute const *)           { A CFString (for debug purposes only) }
var kWSStreamErrorDomain: CFStringRef; external name '_kWSStreamErrorDomain'; (* attribute const *)            { A CFNumberRef }
var kWSStreamErrorError: CFStringRef; external name '_kWSStreamErrorError'; (* attribute const *)             { A CFNumberRef }
>>>>>>> origin/cpstrnew
=======
var kWSStreamErrorMessage: CFStringRef; external name '_kWSStreamErrorMessage'; (* attribute const *)           { A CFString (for debug purposes only) }
var kWSStreamErrorDomain: CFStringRef; external name '_kWSStreamErrorDomain'; (* attribute const *)            { A CFNumberRef }
var kWSStreamErrorError: CFStringRef; external name '_kWSStreamErrorError'; (* attribute const *)             { A CFNumberRef }
>>>>>>> origin/fixes_2.4

{ 
    For HTTP[S] based invocations, you can specify a CFHTTPMessageRef
    as a property which will be used instead of creating a new
    outgoing message.  The CFHTTPMessageRef can contain header, proxy
    and authentication information.  The body of the message will be
    ignored and replaced with the outgoing, serialized invocation.

    After the invocation has executed, you can retrieve a copy of the
    actual CFHTTPMessageRef, containing the details of the invocation
    using kWSHTTPResponseMessage.  Attempting to retrieve the response
    message property before the invocation completes will result
    return NULL.
   
    See: <CFNetwork/CFHTTPMessage.h> for more information.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CFHTTPMessageRef }
var kWSHTTPMessage: CFStringRef; external name '_kWSHTTPMessage'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ CFHTTPMessageRef }
var kWSHTTPResponseMessage: CFStringRef; external name '_kWSHTTPResponseMessage'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
var kWSHTTPMessage: CFStringRef; external name '_kWSHTTPMessage'; (* attribute const *)                  { CFHTTPMessageRef }
var kWSHTTPResponseMessage: CFStringRef; external name '_kWSHTTPResponseMessage'; (* attribute const *)          { CFHTTPMessageRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSHTTPMessage: CFStringRef; external name '_kWSHTTPMessage'; (* attribute const *)                  { CFHTTPMessageRef }
var kWSHTTPResponseMessage: CFStringRef; external name '_kWSHTTPResponseMessage'; (* attribute const *)          { CFHTTPMessageRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSHTTPMessage: CFStringRef; external name '_kWSHTTPMessage'; (* attribute const *)                  { CFHTTPMessageRef }
var kWSHTTPResponseMessage: CFStringRef; external name '_kWSHTTPResponseMessage'; (* attribute const *)          { CFHTTPMessageRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSHTTPMessage: CFStringRef; external name '_kWSHTTPMessage'; (* attribute const *)                  { CFHTTPMessageRef }
var kWSHTTPResponseMessage: CFStringRef; external name '_kWSHTTPResponseMessage'; (* attribute const *)          { CFHTTPMessageRef }
>>>>>>> origin/cpstrnew
=======
var kWSHTTPMessage: CFStringRef; external name '_kWSHTTPMessage'; (* attribute const *)                  { CFHTTPMessageRef }
var kWSHTTPResponseMessage: CFStringRef; external name '_kWSHTTPResponseMessage'; (* attribute const *)          { CFHTTPMessageRef }
>>>>>>> origin/fixes_2.4
{
    To avoid having to create an entire CFHTTPMessageRef, these properties are
    individually settable.  If they are set, they will override any CFHTTPMessageRef
    previously specified.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ a CFDictionary of ( key (CFString), val (CFString) ) pairs }
var kWSHTTPExtraHeaders: CFStringRef; external name '_kWSHTTPExtraHeaders'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ "http/1.1" }
var kWSHTTPVersion: CFStringRef; external name '_kWSHTTPVersion'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ CFURLRef }
var kWSHTTPProxy: CFStringRef; external name '_kWSHTTPProxy'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ kCFBooleanFalse }
var kWSHTTPFollowsRedirects: CFStringRef; external name '_kWSHTTPFollowsRedirects'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
var kWSHTTPVersion: CFStringRef; external name '_kWSHTTPVersion'; (* attribute const *)                  { "http/1.1" }
var kWSHTTPExtraHeaders: CFStringRef; external name '_kWSHTTPExtraHeaders'; (* attribute const *)             { a CFDictionary of ( key (CFString), val (CFString) ) pairs }
var kWSHTTPProxy: CFStringRef; external name '_kWSHTTPProxy'; (* attribute const *)                    { CFURLRef }
var kWSHTTPFollowsRedirects: CFStringRef; external name '_kWSHTTPFollowsRedirects'; (* attribute const *)         { kCFBooleanFalse }
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
    SOCKS proxy support.  WSMethodInvocation uses the same flags as
    CFSocketStream.h in configuring SOCKS proxy support.  You can set
    the kCFStreamPropertySOCKSProxy property on the invocation and the
    value will be applied to the underlying stream.  See CFSocketStream.h
    for more information and valid keys.
}
{
    These debugging flags will populate the WSInvocationResultRef
    with some potentially useful debugging output.  The property
    name of the flag is the same as the the field in the result
    dictionary.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ kCFBooleanFalse }
var kWSDebugOutgoingHeaders: CFStringRef; external name '_kWSDebugOutgoingHeaders'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ kCFBooleanFalse }
var kWSDebugOutgoingBody: CFStringRef; external name '_kWSDebugOutgoingBody'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ kCFBooleanFalse }
var kWSDebugIncomingHeaders: CFStringRef; external name '_kWSDebugIncomingHeaders'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ kCFBooleanFalse }
var kWSDebugIncomingBody: CFStringRef; external name '_kWSDebugIncomingBody'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
var kWSDebugOutgoingHeaders: CFStringRef; external name '_kWSDebugOutgoingHeaders'; (* attribute const *)             { kCFBooleanFalse }
var kWSDebugOutgoingBody: CFStringRef; external name '_kWSDebugOutgoingBody'; (* attribute const *)                { kCFBooleanFalse }
var kWSDebugIncomingHeaders: CFStringRef; external name '_kWSDebugIncomingHeaders'; (* attribute const *)             { kCFBooleanFalse }
var kWSDebugIncomingBody: CFStringRef; external name '_kWSDebugIncomingBody'; (* attribute const *)                { kCFBooleanFalse }
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
    Extra properties for SOAP messages.  These apply to the message
    namespace and format itself.  Individual message elements can
    be modified using the kWSRecord constants below.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CFStringRef }
var kWSSOAPMethodNamespaceURI: CFStringRef; external name '_kWSSOAPMethodNamespaceURI'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ CFStringRef ( kWSSOAPStyleDoc, kWSSOAPStyleRPC ) }
var kWSSOAPBodyEncodingStyle: CFStringRef; external name '_kWSSOAPBodyEncodingStyle'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
var kWSSOAPStyleDoc: CFStringRef; external name '_kWSSOAPStyleDoc'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
var kWSSOAPStyleRPC: CFStringRef; external name '_kWSSOAPStyleRPC'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
var kWSSOAPMethodNamespaceURI: CFStringRef; external name '_kWSSOAPMethodNamespaceURI'; (* attribute const *)   { CFStringRef }
var kWSSOAPBodyEncodingStyle: CFStringRef; external name '_kWSSOAPBodyEncodingStyle'; (* attribute const *)    { CFStringRef ( kWSSOAPStyleDoc, kWSSOAPStyleRPC ) }
var kWSSOAPStyleDoc: CFStringRef; external name '_kWSSOAPStyleDoc'; (* attribute const *)
var kWSSOAPStyleRPC: CFStringRef; external name '_kWSSOAPStyleRPC'; (* attribute const *)
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
    For SOAP messages, this is an array of CFStringRefs which
    contain valid XML header elements that are sent with the
    message.  These are only applicable to the Header of a SOAP
    message.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CFArrayRef }
var kWSSOAPMessageHeaders: CFStringRef; external name '_kWSSOAPMessageHeaders'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
var kWSSOAPMessageHeaders: CFStringRef; external name '_kWSSOAPMessageHeaders'; (* attribute const *)               { CFArrayRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSSOAPMessageHeaders: CFStringRef; external name '_kWSSOAPMessageHeaders'; (* attribute const *)               { CFArrayRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSSOAPMessageHeaders: CFStringRef; external name '_kWSSOAPMessageHeaders'; (* attribute const *)               { CFArrayRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSSOAPMessageHeaders: CFStringRef; external name '_kWSSOAPMessageHeaders'; (* attribute const *)               { CFArrayRef }
>>>>>>> origin/cpstrnew
=======
var kWSSOAPMessageHeaders: CFStringRef; external name '_kWSSOAPMessageHeaders'; (* attribute const *)               { CFArrayRef }
>>>>>>> origin/fixes_2.4

{ 
    When serializing a record (dictionary) these keys present in
    the dictionary can modify the behavior of the serialization.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CFArrayRef of CFStringRef }
var kWSRecordParameterOrder: CFStringRef; external name '_kWSRecordParameterOrder'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ CFStringRef }
var kWSRecordNamespaceURI: CFStringRef; external name '_kWSRecordNamespaceURI'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
{ CFStringRef }
var kWSRecordType: CFStringRef; external name '_kWSRecordType'; (* attribute const *)
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
var kWSRecordParameterOrder: CFStringRef; external name '_kWSRecordParameterOrder'; (* attribute const *)     { CFArrayRef of CFStringRef }
var kWSRecordNamespaceURI: CFStringRef; external name '_kWSRecordNamespaceURI'; (* attribute const *)       { CFStringRef }
var kWSRecordType: CFStringRef; external name '_kWSRecordType'; (* attribute const *)               { CFStringRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSRecordParameterOrder: CFStringRef; external name '_kWSRecordParameterOrder'; (* attribute const *)     { CFArrayRef of CFStringRef }
var kWSRecordNamespaceURI: CFStringRef; external name '_kWSRecordNamespaceURI'; (* attribute const *)       { CFStringRef }
var kWSRecordType: CFStringRef; external name '_kWSRecordType'; (* attribute const *)               { CFStringRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSRecordParameterOrder: CFStringRef; external name '_kWSRecordParameterOrder'; (* attribute const *)     { CFArrayRef of CFStringRef }
var kWSRecordNamespaceURI: CFStringRef; external name '_kWSRecordNamespaceURI'; (* attribute const *)       { CFStringRef }
var kWSRecordType: CFStringRef; external name '_kWSRecordType'; (* attribute const *)               { CFStringRef }
>>>>>>> graemeg/cpstrnew
=======
var kWSRecordParameterOrder: CFStringRef; external name '_kWSRecordParameterOrder'; (* attribute const *)     { CFArrayRef of CFStringRef }
var kWSRecordNamespaceURI: CFStringRef; external name '_kWSRecordNamespaceURI'; (* attribute const *)       { CFStringRef }
var kWSRecordType: CFStringRef; external name '_kWSRecordType'; (* attribute const *)               { CFStringRef }
>>>>>>> origin/cpstrnew
=======
var kWSRecordParameterOrder: CFStringRef; external name '_kWSRecordParameterOrder'; (* attribute const *)     { CFArrayRef of CFStringRef }
var kWSRecordNamespaceURI: CFStringRef; external name '_kWSRecordNamespaceURI'; (* attribute const *)       { CFStringRef }
var kWSRecordType: CFStringRef; external name '_kWSRecordType'; (* attribute const *)               { CFStringRef }
>>>>>>> origin/fixes_2.4

{
    Specifies that the result parameter will be found as this name.  This
    forces the deserializer to alias the named output parameter to kWSMethodInvocationResult
}
var kWSMethodInvocationResultParameterName: CFStringRef; external name '_kWSMethodInvocationResultParameterName'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
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
    Specifies a timeout (as CFNumber) which specifies in seconds the
    amount of time to wait for the invocation to complete.  If the
    invocation times out before the server results are returned, 
    a fault will be returned with the error code errWSTimeoutError.  
}
var kWSMethodInvocationTimeoutValue: CFStringRef; external name '_kWSMethodInvocationTimeoutValue'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
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
 *  WSMethodInvocationRef
 *  
 *  Discussion:
 *    a WSMethodInvocationRef represents an object that can be executed
 *    to obtain a rsult from a web service. This is CFType and is
 *    therefore reference counted and and should be managed via
 *    CFRetain and CFRelease.
 }
type
	WSMethodInvocationRef = ^OpaqueWSMethodInvocationRef; { an opaque type }
	OpaqueWSMethodInvocationRef = record end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{
 *  WSMethodInvocationGetTypeID()   *** DEPRECATED ***
=======
{
 *  WSMethodInvocationGetTypeID()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationGetTypeID()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationGetTypeID()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationGetTypeID()
>>>>>>> origin/cpstrnew
=======
{
 *  WSMethodInvocationGetTypeID()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodInvocationGetTypeID: CFTypeID; external name '_WSMethodInvocationGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationCreate()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 *  WSMethodInvocationCreate()
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
 *  
 *  Discussion:
 *    Creates a web services method invocation object.  This object may
 *    be executed synchronously or scheduled on a run loop for
 *    asynchronous execution.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    url:
 *      the endpoint of the service
 *    
 *    methodName:
 *      the name of the method to be called
 *    
 *    protocol:
 *      a string, defined above, that determines the type of invocation
 *      object to create (XML-RPC vs. SOAP)
 *  
 *  Result:
 *    A WSMethodInvocationRef object that can be passed to
 *    WSMethodInvocationInvoke or scheduled with a run loop.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodInvocationCreate( url: CFURLRef; methodName: CFStringRef; protocol: CFStringRef ): WSMethodInvocationRef; external name '_WSMethodInvocationCreate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationCreateFromSerialization()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 *  WSMethodInvocationCreateFromSerialization()
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
 *  
 *  Discussion:
 *    Creates a web services method invocation object from a previously
 *    serialized contract.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    contract:
 *      the result of a previously serialized WSMethodInvocationRef
 *  
 *  Result:
 *    A WSMethodInvocationRef object that can be passed to
 *    WSMethodInvocationInvoke or scheduled with a run loop.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodInvocationCreateFromSerialization( contract: CFDataRef ): WSMethodInvocationRef; external name '_WSMethodInvocationCreateFromSerialization';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationCopySerialization()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 *  WSMethodInvocationCopySerialization()
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
 *  
 *  Discussion:
 *    Create a serialized version of the Method Invocation which can be
 *    reconstituted at a later time.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation to serialize
 *  
 *  Result:
 *    a CFDataRef
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodInvocationCopySerialization( invocation: WSMethodInvocationRef ): CFDataRef; external name '_WSMethodInvocationCopySerialization';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationSetParameters()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 *  WSMethodInvocationSetParameters()
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
 *  
 *  Discussion:
 *    Set the parameters for a method invocation.  The parameterOrder
 *    may be NULL, in which case the order of th parameters is
 *    undefined.  If it is not NULL and the parameters dictionary
 *    contains more parameters than are specified by the order, the
 *    behavior is undefined.  If the parameterOrder specifies more
 *    parameters than are present in the dictionary, the result is
 *    undefined.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation object
 *    
 *    parameters:
 *      a CFDictionaryRef of CFString keys and CFTypeRef values.
 *    
 *    parameterOrder:
 *      a CFArrayRef of CFString parameter names.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure WSMethodInvocationSetParameters( invocation: WSMethodInvocationRef; parameters: CFDictionaryRef; parameterOrder: CFArrayRef { can be NULL } ); external name '_WSMethodInvocationSetParameters';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationCopyParameters()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure WSMethodInvocationSetParameters( invocation: WSMethodInvocationRef; parameters: CFDictionaryRef; parameterOrder: CFArrayRef ); external name '_WSMethodInvocationSetParameters';      { can be NULL }


{
 *  WSMethodInvocationCopyParameters()
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
 *  
 *  Discussion:
 *    Copies the parameters from the invocation.  The resulting
 *    dictionary contains the parameter dictionary.  The parameterOrder
 *    output parameter, if not NULL, will contain the order used to
 *    serialize the parameters.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation
 *    
 *    parameterOrder:
 *      a pointer to a CFArray which will will receive the names, in
 *      their specified order, of the input parameter values.  This
 *      parameter may be NULL.
 *  
 *  Result:
 *    a CFDictionaryRef
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodInvocationCopyParameters( invocation: WSMethodInvocationRef; parameterOrder: CFArrayRefPtr { can be NULL } ): CFDictionaryRef; external name '_WSMethodInvocationCopyParameters';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationSetProperty()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodInvocationCopyParameters( invocation: WSMethodInvocationRef; var parameterOrder: CFArrayRef ): CFDictionaryRef; external name '_WSMethodInvocationCopyParameters';      { can be NULL }


{
 *  WSMethodInvocationSetProperty()
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
 *  
 *  Discussion:
 *    Add "properties" to a method invocation.  These properties can be
 *    user defined or one of the WebServicesCore declared properties
 *    (which may modify the behavior of the invocation.)  All
 *    WebServicesCore declared properties will start with the string
 *    "kWS", eg, kWSHTTPFollowsRedirects.  Properties are serialized
 *    along with the contract, so you may want to avoid sticking raw
 *    pointers in a CFNumber (for example).
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation
 *    
 *    propertyName:
 *      a CFStringRef name of the property to modify
 *    
 *    propertyValue:
 *      a CFTypeRef containing the new property value
 *  
 *  Result:
 *    none
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure WSMethodInvocationSetProperty( invocation: WSMethodInvocationRef; propertyName: CFStringRef; propertyValue: CFTypeRef ); external name '_WSMethodInvocationSetProperty';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationCopyProperty()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 *  WSMethodInvocationCopyProperty()
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
 *  
 *  Discussion:
 *    Return a property from a invocation.  If the result is NULL, the
 *    property doesn't exist.  Being a "Copy" call, you must release
 *    the result.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation
 *    
 *    propertyName:
 *      the name of the property to retreive
 *  
 *  Result:
 *    the CFTypeRef value of the property, or NULL if the property was
 *    not specified.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodInvocationCopyProperty( invocation: WSMethodInvocationRef; propertyName: CFStringRef ): CFTypeRef; external name '_WSMethodInvocationCopyProperty';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationInvoke()   *** DEPRECATED ***
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
 *  WSMethodInvocationInvoke()
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
 *  
 *  Discussion:
 *    Execute the invocation.  If the call was successful, the result
 *    will contain the result of the invocation. If for some reason the
 *    invocation failed, including out of memory or invalid parameter
 *    errors, then the result will contain a fault structure.  You must
 *    release the result when you're done with it.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation
 *  
 *  Result:
 *    a CFDictionaryRef containing the result of the execution or a
 *    fault, and optional debug information.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
{ CF_RETURNS_RETAINED }
function WSMethodInvocationInvoke( invocation: WSMethodInvocationRef ): CFDictionaryRef; external name '_WSMethodInvocationInvoke';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.2 and later
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodInvocationInvoke( invocation: WSMethodInvocationRef ): CFDictionaryRef; external name '_WSMethodInvocationInvoke';

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
    These calls implemented the asynchronous variant of the WSMethodInvocationInvoke.
    
    The strategy is to schedule the invocation on a given runloop.
    When the invocation completes, it calls the specified callback with
    the result of the execution.  The callback is responsible for
    releasing the result ref.  Your code is responsible for
    unscheduling the invocation from the run loop, whether it completes
    or not.
    
    You can re-schedule an invocation after it completes.
    
    When you unschedule with the run loop, the CallBack is not called.
    
    If a network error occurs, the kWSFaultString entry of the result
    will contain some textual description of the error, kWSFaultCode
    will contain kWSNetworkingFault and kWSFaultExtra will be a
    dictionary containing two CFNumber values called kWSStreamErrorDomain
    and kWSStreamErrorError.
}


{
 *  WSMethodInvocationCallBackProcPtr
 *  
 *  Discussion:
 *    Prototypes the callback made when an asynchronous invocation
 *    completes.  This callback is passed a reference to the invocation
 *    just completed, a pointer to private data, and a dictionary that
 *    contains the return value or falut for this invocation.  The
 *    callback is responsible for releasing the dictionary when it is
 *    no longer used.
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation just completed
 *    
 *    info:
 *      private callback data
 *    
 *    outRef:
 *      a CFDictionaryRef containing the result of the execution or a
 *      fault, and optional debug information.
 }
type
	WSMethodInvocationCallBackProcPtr = procedure( invocation: WSMethodInvocationRef; info: UnivPtr; outRef: CFDictionaryRef );
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	
	
{
 *  WSMethodInvocationSetCallBack()   *** DEPRECATED ***
=======
{
 *  WSMethodInvocationSetCallBack()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationSetCallBack()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationSetCallBack()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationSetCallBack()
>>>>>>> origin/cpstrnew
=======
{
 *  WSMethodInvocationSetCallBack()
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    sets the callback for an asynchronous method invocation.  Call
 *    with a clientCB and context of NULL to clear the invocation
 *    callback.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation
 *    
 *    clientCB:
 *      a ProcPtr to be called when the invocation completes.
 *    
 *    context:
 *      a pointer to a WSClientContext.  The structure will be copied.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure WSMethodInvocationSetCallBack( invocation: WSMethodInvocationRef; clientCB: WSMethodInvocationCallBackProcPtr; var context: WSClientContext ); external name '_WSMethodInvocationSetCallBack';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationScheduleWithRunLoop()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Schedules the invocation to execute on the run loop.
 *
 *  Mac OS X threading:
 *    Thread safe
 *   
 *  Parameters:
 *    
 *    invocation:
 *      the invocation.
 *    
 *    runLoop:
 *      the run loop upon which to scheduile the invocation.
 *    
 *    runLoopMode:
 *      the run loop mode.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
    @function       WSMethodInvocationScheduleWithRunLoop

    @discussion     Schedules the invocation to execute on the run loop.

    @param          invocation
                        the invocation

    @param          runLoop
                        the run loop upon which to scheduile the invocation

    @param          runLoopMode
                        the run loop mode
}
{
 *  WSMethodInvocationScheduleWithRunLoop()
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
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
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure WSMethodInvocationScheduleWithRunLoop( invocation: WSMethodInvocationRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ); external name '_WSMethodInvocationScheduleWithRunLoop';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)


{
 *  WSMethodInvocationUnscheduleFromRunLoop()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Unschedules the invocation from a given run loop and
 *    mode.  If the invocation has not yet completed,
 *    its callback will not be called.
 *
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation.
 *    
 *    runLoop:
 *      the run loop upon which to scheduile the invocation.
 *    
 *    runLoopMode:
 *      the run loop mode.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


{
    @function       WSMethodInvocationUnscheduleFromRunLoop

    @discussion     Unschedules the invocation from a given run loop and
                    mode.  If the invocation has not yet completed,
                    its callback will not be called.


    @param          invocation
                        the invocation

    @param          runLoop
                        the run loop upon which to scheduile the invocation

    @param          runLoopMode
                        the run loop mode
}
{
 *  WSMethodInvocationUnscheduleFromRunLoop()
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later
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
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure WSMethodInvocationUnscheduleFromRunLoop( invocation: WSMethodInvocationRef; runLoop: CFRunLoopRef; runLoopMode: CFStringRef ); external name '_WSMethodInvocationUnscheduleFromRunLoop';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
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
    Result interrogation.  

    If the result is a fault, look in the kWSFaultCode, kWSFaultString
    and kWSFaultExtra fields of the resulting dictionary.  If not a
    fault, kWSMethodInvocationResult will contain the result of the
    execution.  If debugging information was requested, it will be
    available in the dictionary as well.
}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  WSMethodResultIsFault()   *** DEPRECATED ***
=======
 *  WSMethodResultIsFault()
>>>>>>> graemeg/cpstrnew
=======
 *  WSMethodResultIsFault()
>>>>>>> graemeg/cpstrnew
=======
 *  WSMethodResultIsFault()
>>>>>>> graemeg/cpstrnew
=======
 *  WSMethodResultIsFault()
>>>>>>> origin/cpstrnew
=======
 *  WSMethodResultIsFault()
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    returns TRUE if the method invocation result contains a fault.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    methodResult:
 *      the result ref
 *  
 *  Result:
 *    TRUE if the result contains a fault condition
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function WSMethodResultIsFault( methodResult: CFDictionaryRef ): Boolean; external name '_WSMethodResultIsFault';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
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
    Serialization / Deserialization override support.
    
    You can add serialization and deserialization callbacks for custom
    types, or types not otherwise handled by the framework.  Note that these
    properties are *not* serialized if the invocation is serialized.
 }


{
 *  WSMethodInvocationSerializationProcPtr
 *  
 *  Discussion:
 *    Prototypes the callback function for a custom serialization proc.
 *     This callback is called whenever a type has the given CFTypeID. 
 *    The callback should return an XML snippet that will be understood
 *    by the server as a correct serialization for a given type.  If
 *    the callback returns NULL, the default serializer will be used.
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation currently being serialized
 *    
 *    obj:
 *      the CFTypeRef to be serialized
 *    
 *    info:
 *      private callback data
 *  
 *  Result:
 *    a CFStringRef containing valid XML.  The caller of this callback
 *    will release the string.
 }
type
	WSMethodInvocationSerializationProcPtr = function( invocation: WSMethodInvocationRef; obj: CFTypeRef; info: UnivPtr ): CFStringRef;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	
	
{
 *  WSMethodInvocationAddSerializationOverride()   *** DEPRECATED ***
=======
{
 *  WSMethodInvocationAddSerializationOverride()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationAddSerializationOverride()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationAddSerializationOverride()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationAddSerializationOverride()
>>>>>>> origin/cpstrnew
=======
{
 *  WSMethodInvocationAddSerializationOverride()
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    Specifies a callback which will be called to produce the XML that
 *    represents the serialization of a given type ref.  See
 *    WSDescription.h for a list of CFTypes for which there currently
 *    exist serializers.  If your callback returns NULL, the default
 *    serializer will be used.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation
 *    
 *    objType:
 *      the CFTypeID of the object
 *    
 *    serializationProc:
 *      the callback called
 *    
 *    context:
 *      a pointer to a WSClientContext.  The structure will be copied.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure WSMethodInvocationAddSerializationOverride( invocation: WSMethodInvocationRef; objType: CFTypeID; serializationProc: WSMethodInvocationSerializationProcPtr; var context: WSClientContext ); external name '_WSMethodInvocationAddSerializationOverride';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
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
 *  WSMethodInvocationDeserializationProcPtr
 *  
 *  Discussion:
 *    Prototypes the callback function for a custom deserializer.  This
 *    callback is passed a reference to the invocation currently being
 *    executed, the root of the response parse tree, the current node
 *    being deserialized, and a pointer to private data. The return
 *    result should be a valid CFTypeRef object (which will be released
 *    by the caller) or NULL to allow the default deserializer to act.
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation executing
 *    
 *    msgRoot:
 *      the root tree element
 *    
 *    deserializeRoot:
 *      the tree element that needs to be deserialied
 *    
 *    info:
 *      private callback data
 *  
 *  Result:
 *    a CFTypeRef representing the deserialized data, or NULL to allow
 *    the default deserializers to act.
 }
type
	WSMethodInvocationDeserializationProcPtr = function( invocation: WSMethodInvocationRef; msgRoot: CFXMLTreeRef; deserializeRoot: CFXMLTreeRef; info: UnivPtr ): CFTypeRef;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	
	
{
 *  WSMethodInvocationAddDeserializationOverride()   *** DEPRECATED ***
=======
{
 *  WSMethodInvocationAddDeserializationOverride()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationAddDeserializationOverride()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationAddDeserializationOverride()
>>>>>>> graemeg/cpstrnew
=======
{
 *  WSMethodInvocationAddDeserializationOverride()
>>>>>>> origin/cpstrnew
=======
{
 *  WSMethodInvocationAddDeserializationOverride()
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    Specifies a callback to be made when parsing an XML method
 *    response.  The callback should return a CFTypeRef containing the
 *    deserialized object value.  If the callback returns NULL, the
 *    default deserializer will be used.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    invocation:
 *      the invocation
 *    
 *    typeNamespace:
 *      the fully resolved namespace for a specific type.  If NULL, the
 *      default namespace will be used.  For example, this field could
 *      be: CFSTR("http://www.w3.org/2001/XMLSchema-instance").
 *    
 *    typeName:
 *      the non-qualified type name.  This parameter must not be NULL.
 *    
 *    deserializationProc:
 *      a ProcPtr to be called to perform the deserialization
 *    
 *    context:
 *      a pointer to a WSClientContext.  The structure will be copied.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later but deprecated in 10.8
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure WSMethodInvocationAddDeserializationOverride( invocation: WSMethodInvocationRef; typeNamespace: CFStringRef; typeName: CFStringRef; deserializationProc: WSMethodInvocationDeserializationProcPtr; var context: WSClientContext ); external name '_WSMethodInvocationAddDeserializationOverride';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2,__MAC_10_8,__IPHONE_NA,__IPHONE_NA) *)
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


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
