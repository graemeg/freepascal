{
 * Copyright (c) 2001-2002,2004 Apple Computer, Inc. All Rights Reserved.
 * 
 * @APPLE_LICENSE_HEADER_START@
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, August 2015 }
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
>>>>>>> origin/cpstrnew
=======
{	  Pascal Translation:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
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

unit AuthorizationPlugin;
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
uses MacTypes,Authorization;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
 *  AuthorizationPlugin.h
 *  AuthorizationPlugin -- APIs for implementing authorization plugins.
 }


{!
	@header AuthorizationPlugin
	
	The AuthorizationPlugin API allows the creation of plugins that can participate
	in authorization decisions.  Using the AuthorizationDB API the system can be configured
	to use these plugins.  Plugins are loaded into a separate process, the pluginhost, to 
	isolate the process of authorization from the client.  There are two types of pluginhosts.
	One runs as an anonymous user and can be used to communicate with the user, for example
	to ask for a password.  Another one runs with root privileges to perform privileged
	operations that may be required.

    A typical use is to implement additional policies that cannot be expressed in the
    authorization configuration.
    
    Plugins implement a handshake function called AuthorizationPluginCreate with which
    their interface (AuthorizationPluginInterface) and the engine's interface
    (AuthorizationCallbacks) are exchanged.  Plugins are asked to create 
    Mechanisms, which are the basic element as authorizations are performed.  
    
    Mechanisms are invoked when it is time for them to make a decision.  A decision is 
    made by setting a single result (AuthorizationResult).  Mechanisms in the 
    authorization can communicate auxiliary information by setting and/or getting hints 
    and setting and/or getting context data.  Hints are advisory and don't need to be
    looked at, nor are they preserved as part of the authorization result. Context data
    becomes part of the result of the authorization.
    
    Context data is tagged with a flag that describes whether the information is returned
    to the authorization client upon request (AuthorizationCopyInfo() in Authorization.h)
    or whether it's private to the mechanisms making a decision.
    
}


{!
	@typedef AuthorizationValue
    Auxiliary data is passed between the engine and the mechanism as AuthorizationValues
}
type
	AuthorizationValue = record
		length: size_t;
		data: UnivPtr;
	end;
	AuthorizationValuePtr = ^AuthorizationValue;
	
{!
    @typedef AuthorizationValueVector
    A vector of AuthorizationValues.  Used to communicate arguments passed from the 
    configuration file <code>authorization(5)</code>.
}
type
	AuthorizationValueVector = record
		count: UInt32;
		values: AuthorizationValuePtr;
	end;
	AuthorizationValueVectorPtr = ^AuthorizationValueVector;
	
{!
    @typedef
    Data produced as context during the authorization evaluation is tagged.  
    If data is set to be extractable (kAuthorizationContextFlagExtractable), it will be possible for the client of authorization to obtain the value of this attribute using AuthorizationCopyInfo().
    If data is marked as volatile (kAuthorizationContextFlagVolatile), this value will not be remembered in the AuthorizationRef.
    Sticky data (kAuthorizationContextFlagSticky) persists through a failed or interrupted evaluation. It can be used to propagate an error condition from a downstream plugin to an upstream one. It is not remembered in the AuthorizationRef.
}
type
	AuthorizationContextFlags = UInt32;
const
	kAuthorizationContextFlagExtractable = 1 shl 0;
	kAuthorizationContextFlagVolatile = 1 shl 1;
	kAuthorizationContextFlagSticky = 1 shl 2;


{!
	@typedef AuthorizationMechanismId
    The mechanism id specified in the configuration is passed to the plugin to create the appropriate mechanism.
}
type
	AuthorizationMechanismId = ConstAuthorizationString;

{!
    @typedef AuthorizationPluginId
	Not used by plugin writers.  Loaded plugins are identified by their name.
 }
type
	AuthorizationPluginId = ConstAuthorizationString;

{!
	@typedef AuthorizationPluginRef
	Handle passed back by the plugin writer when creating a plugin.  Any pluginhost will only instantiate one instance.  The handle is used when creating mechanisms.
}
type
	AuthorizationPluginRef = UnivPtr;

{!
	@typedef AuthorizationMechanismRef
	Handle passed back by the plugin writer when creating an an instance of a mechanism in a plugin.  One instance will be created for any authorization.
}
type
	AuthorizationMechanismRef = UnivPtr;

{!
	@typedef AuthorizationEngineRef
	Handle passed from the engine to an instance of a mechanism in a plugin (corresponds to a particular AuthorizationMechanismRef).
}
type
	__OpaqueAuthorizationEngine = Record end;
	AuthorizationEngineRef = ^__OpaqueAuthorizationEngine;

{!
	@typedef AuthorizationSessionId
	A unique value for an AuthorizationSession being evaluated, provided by the authorization engine.
    A session is represented by a top level call to an Authorization API.
}
type
	AuthorizationSessionId = UnivPtr;

{!
	@typedef AuthorizationResult
	Type for SetResult().  See AuthorizationResultConstants for all allowed values.
}
type
	AuthorizationResult = UInt32;

{!
    @enum AuthorizationResultConstants
	Possible values for SetResult() in AuthorizationCallbacks.
    
    @constant kAuthorizationResultAllow the operation succeeded and authorization should be granted as far as this mechanism is concerned.
    @constant kAuthorizationResultDeny the operation succeeded but authorization should be denied as far as this mechanism is concerned.
    @constant kAuthorizationResultUndefined the operation failed for some reason and should not be retried for this session.
    @constant kAuthorizationResultUserCanceled the user has requested that the evaluation be terminated.
}
const
	kAuthorizationResultAllow = 0;
	kAuthorizationResultDeny = 1;
	kAuthorizationResultUndefined = 2;
	kAuthorizationResultUserCanceled = 3;

{!
    @enum
    Version of the interface (AuthorizationPluginInterface) implemented by the plugin.
    The value is matched to the definition in this file.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAuthorizationPluginInterfaceVersion0 = 0;
	kAuthorizationPluginInterfaceVersion1 = 1; { new in OS X 10.9 }

	kAuthorizationPluginInterfaceVersion = kAuthorizationPluginInterfaceVersion1;
=======
	kAuthorizationPluginInterfaceVersion = 0;
>>>>>>> graemeg/cpstrnew
=======
	kAuthorizationPluginInterfaceVersion = 0;
>>>>>>> graemeg/cpstrnew
=======
	kAuthorizationPluginInterfaceVersion = 0;
>>>>>>> graemeg/cpstrnew
=======
	kAuthorizationPluginInterfaceVersion = 0;
>>>>>>> origin/cpstrnew
=======
	kAuthorizationPluginInterfaceVersion = 0;
>>>>>>> origin/fixes_2.4

{!
    @enum
    Version of the callback structure (AuthorizationCallbacks) passed to the plugin.
    The value is matched to the definition in this file.  The engine may provide a newer
    interface.
}
const
	kAuthorizationCallbacksVersion = 0;


{!
    @struct
    Callback API provided by the AuthorizationEngine. 
    
    @field version      Engine callback version.
    @field SetResult    Set a result after a call to AuthorizationSessionInvoke.
    @field RequestInterrupt Request authorization engine to interrupt all mechamisms invoked after this mechamism has called SessionSetResult and then call AuthorizationSessionInvoke again.
    @field DidDeactivate    Respond to the Deactivate request.
    @field GetContextValue  Read value from context.  AuthorizationValue does not own data.
    @field SetContextValue  Write value to context.  AuthorizationValue and data are copied.
    @field GetHintValue     Read value from hints. AuthorizationValue does not own data.
    @field SetHintValue     Write value to hints.  AuthorizationValue and data are copied.
    @field GetArguments     Read arguments passed.  AuthorizationValueVector does not own data.
    @field GetSessionId     Read SessionId.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AuthorizationCallbacks0 = record
=======
	AuthorizationCallbacks = record
>>>>>>> graemeg/cpstrnew
=======
	AuthorizationCallbacks = record
>>>>>>> graemeg/cpstrnew
=======
	AuthorizationCallbacks = record
>>>>>>> graemeg/cpstrnew
=======
	AuthorizationCallbacks = record
>>>>>>> origin/cpstrnew
=======
	AuthorizationCallbacks = record
>>>>>>> origin/fixes_2.4
{ Engine callback version. }
		version: UInt32;

    { Set a result after a call to AuthorizationSessionInvoke. }
{    OSStatus (*SetResult)(AuthorizationEngineRef inEngine, AuthorizationResult inResult); }
	 SetResult : function (inEngine : AuthorizationEngineRef; inResult : AuthorizationResult) : OSStatus;

    { Request authorization engine to interrupt all mechamisms invoked after 
        this mechamism has called SessionSetResult and then call 
        AuthorizationSessionInvoke again. }
     RequestInterrupt : function (inEngine : AuthorizationEngineRef) : OSStatus;
     
    { Respond to the Deactivate request. }
	 DidDeactivate : function (inEngine : AuthorizationEngineRef) : OSStatus;

    { Read value from context.  AuthorizationValue does not own data. }
        GetContextValue : function (inEngine			: AuthorizationEngineRef;
        							inKey				: AuthorizationString;
        							var outContextFlags : AuthorizationContextFlags;
        							var outValue		: AuthorizationValuePtr) : OSStatus;

    { Write value to context.  AuthorizationValue and data are copied. }
	 SetContextValue : function (inEngine 	: AuthorizationEngineRef;
	 							 inKey		: AuthorizationString;
	 							 inContextFlags	: AuthorizationContextFlags;
	 							 inValue		: AuthorizationValuePtr) : OSStatus;

    { Read value from hints. AuthorizationValue does not own data. }
	GetHintValue : function (inEngine		: AuthorizationEngineRef;
							 inKey			: AuthorizationString;
							 var outValue	: AuthorizationValuePtr) : OSStatus;
							 
    { Write value to hints.  AuthorizationValue and data are copied. }
	SetHintValue : function (inEngine	: AuthorizationEngineRef;
							 inKey		: AuthorizationString;
							 inValue	: AuthorizationValuePtr) : OSStatus;
							 
    { Read arguments passed.  AuthorizationValueVector does not own data. }
	GetArguments : function (inEngine	 		: AuthorizationEngineRef;
							 var outArguments	: AuthorizationValueVectorPtr) : OSStatus;
							 
    { Read SessionId. }
      GetSessionId : function (inEngine			: AuthorizationEngineRef;
      						   var outSessionId : AuthorizationSessionId) : OSStatus;
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AuthorizationCallbacks1 = record
{ Engine callback version. }
		version: UInt32;

    { Set a result after a call to AuthorizationSessionInvoke. }
{    OSStatus (*SetResult)(AuthorizationEngineRef inEngine, AuthorizationResult inResult); }
	 SetResult : function (inEngine : AuthorizationEngineRef; inResult : AuthorizationResult) : OSStatus;

    { Request authorization engine to interrupt all mechamisms invoked after 
        this mechamism has called SessionSetResult and then call 
        AuthorizationSessionInvoke again. }
     RequestInterrupt : function (inEngine : AuthorizationEngineRef) : OSStatus;
     
    { Respond to the Deactivate request. }
	 DidDeactivate : function (inEngine : AuthorizationEngineRef) : OSStatus;

    { Read value from context.  AuthorizationValue does not own data. }
        GetContextValue : function (inEngine			: AuthorizationEngineRef;
        							inKey				: AuthorizationString;
        							var outContextFlags : AuthorizationContextFlags;
        							var outValue		: AuthorizationValuePtr) : OSStatus;

    { Write value to context.  AuthorizationValue and data are copied. }
	 SetContextValue : function (inEngine 	: AuthorizationEngineRef;
	 							 inKey		: AuthorizationString;
	 							 inContextFlags	: AuthorizationContextFlags;
	 							 inValue		: AuthorizationValuePtr) : OSStatus;

    { Read value from hints. AuthorizationValue does not own data. }
	GetHintValue : function (inEngine		: AuthorizationEngineRef;
							 inKey			: AuthorizationString;
							 var outValue	: AuthorizationValuePtr) : OSStatus;
							 
    { Write value to hints.  AuthorizationValue and data are copied. }
	SetHintValue : function (inEngine	: AuthorizationEngineRef;
							 inKey		: AuthorizationString;
							 inValue	: AuthorizationValuePtr) : OSStatus;
							 
    { Read arguments passed.  AuthorizationValueVector does not own data. }
	GetArguments : function (inEngine	 		: AuthorizationEngineRef;
							 var outArguments	: AuthorizationValueVectorPtr) : OSStatus;
							 
    { Read SessionId. }
      GetSessionId : function (inEngine			: AuthorizationEngineRef;
      						   var outSessionId : AuthorizationSessionId) : OSStatus;

    { Read value from hints. AuthorizationValue does not own data. }
  GetImmutableHintValue: function (inEngine: AuthorizationEngineRef;
                     inKey: AuthorizationString;
                     var outValue: AuthorizationValuePtr): OSStatus; 

	end;
	
	{ support both interface version 0 and 1 }
	AuthorizationCallbacks = record
	  case byte of
	    0: (version0: AuthorizationCallbacks0);
	    1: (version1: AuthorizationCallbacks1);
	end;
	
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
	AuthorizationCallbacksPtr = ^AuthorizationCallbacks;

{!
    @struct
    Interface that must be implemented by each plugin. 
    
    @field version  Must be set to kAuthorizationPluginInterfaceVersion
    @field PluginDestroy    Plugin should clean up and release any resources it is holding.
    @field MechanismCreate  The plugin should create a mechanism named mechanismId.  The mechanism needs to use the AuthorizationEngineRef for the callbacks and pass back a   AuthorizationMechanismRef for itself.  MechanismDestroy will be called when it is no longer needed.
    @field MechanismInvoke  Invoke an instance of a mechanism.  It should call SetResult during or after returning from this function.
    @field MechanismDeactivate  Mechanism should respond with a DidDeactivate as soon as possible
    @field MechanismDestroy Mechanism should clean up and release any resources it is holding
}
type
	AuthorizationPluginInterface = record
{ Must be set to kAuthorizationPluginInterfaceVersion. }
		version: UInt32;

    { Notify a plugin that it is about to be unloaded so it get a chance to clean up and release any resources it is holding.  }
	PluginDestroy : function (inPlugin : AuthorizationPluginRef) : OSStatus;
	
    { The plugin should create a mechanism named mechanismId.  The mechanism needs to use the
        AuthorizationEngineRef for the callbacks and pass back an AuthorizationMechanismRef for
        itself.  MechanismDestroy will be called when it is no longer needed. }
 	MechanismCreate : function (inPlugin 			: AuthorizationPluginRef;
								inEngine 			: AuthorizationEngineRef;
								mechanismId			: AuthorizationMechanismId;
								var outMechanism	: AuthorizationMechanismRef) : OSStatus;
								
    { Invoke an instance of a mechanism.  It should call SetResult during or after returning from this function.  }
	MechanismInvoke : function (inMechanism : AuthorizationMechanismRef) : OSStatus;
	
    { Mechanism should respond with a DidDeactivate as soon as possible. }
	MechanismDeactivate : function (inMechanism : AuthorizationMechanismRef) : OSStatus;
	
    { Mechanism should clean up and release any resources it is holding. }
    MechanismDestroy : function (inMechanism : AuthorizationMechanismRef) : OSStatus;
	end;

	AuthorizationPluginInterfacePtr = ^AuthorizationPluginInterface;

{!
    @function AuthorizationPluginCreate

    Initialize a plugin after it gets loaded.  This is the main entry point to a plugin.  This function will only be called once.  
    After all Mechanism instances have been destroyed outPluginInterface->PluginDestroy will be called.

    @param callbacks (input) A pointer to an AuthorizationCallbacks which contains the callbacks implemented by the AuthorizationEngine.
    @param outPlugin (output) On successful completion should contain a valid AuthorizationPluginRef.  This will be passed in to any subsequent calls the engine makes to  outPluginInterface->MechanismCreate and outPluginInterface->PluginDestroy.
    @param outPluginInterface (output) On successful completion should contain a pointer to a AuthorizationPluginInterface that will stay valid until outPluginInterface->PluginDestroy is called. }
function AuthorizationPluginCreate( const callbacks: AuthorizationCallbacksPtr; var outPlugin: AuthorizationPluginRef; {const} var outPluginInterface: AuthorizationPluginInterfacePtr ): OSStatus; external name '_AuthorizationPluginCreate';

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
