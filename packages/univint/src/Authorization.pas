{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000-2004,2007 Apple Inc. All Rights Reserved.
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
{  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{  Pascal Translation Update:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, August 2015 }
=======
 {	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
 {	  Pascal Translation Update:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
 {    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 {	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
 {	  Pascal Translation Update:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
 {    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 {	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
 {	  Pascal Translation Update:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
 {    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 {	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
 {	  Pascal Translation Update:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
 {    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
 * Copyright (c) 2000-2002 Apple Computer, Inc. All Rights Reserved.
=======
 * Copyright (c) 2000-2004,2007 Apple Inc. All Rights Reserved.
>>>>>>> origin/fixes_2.4
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
 {	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
 {	  Pascal Translation Update:  Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
 {    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit Authorization;
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
 *  Authorization.h -- APIs for implementing access control in applications
 *  and daemons.
 }


<<<<<<< HEAD
=======
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{!
	@header Authorization
	Version 1.0 10/16/2000

	The Authorization API contains all the APIs that a application or tool that need pre-authorization or need an authorization desision made.
	
	A typical use cases are a preference panel that would start off calling AuthorizationCreate() (without UI) to get an authorization object.  Then call AuthorizationCopyRights() to figure out what is currently allowed.
	
	If any of the operations that the preference panel wishes to perform are currently not allowed the lock icon in the window would show up in the locked state.  Otherwise it would show up unlocked.
	
	When the user locks the lock AuthorizationFree() is called with the kAuthorizationFlagDestroyRights to destroy any authorization rights that have been aquired.
	
	When the user unlocks the lock AuthorizationCreate() is called with the kAuthorizationFlagInteractionAllowed and kAuthorizationFlagExtendRights flags to obtain all required rights.  The old authorization object can be freed by calling AuthorizationFree() with no flags.

}


{!
	@defined kAuthorizationEmptyEnvironment
	Parameter to specify to AuthorizationCreate when no environment is being provided.
}
const kAuthorizationEmptyEnvironment = nil;


{!
	@enum AuthorizationStatus
	Error codes returned by Authorization API.
}

{
    Note: the comments that appear after these errors are used to create SecErrorMessages.strings.
    The comments must not be multi-line, and should be in a form meaningful to an end user. If
    a different or additional comment is needed, it can be put in the header doc format, or on a
    line that does not start with errZZZ.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    errAuthorizationSuccess can't include a string as it's also errSecSuccess in libsecurity_keychain/lib/SecBase.h
}

const
	errAuthorizationSuccess = 0;
	errAuthorizationInvalidSet = -60001; { The authorization rights are invalid. }
	errAuthorizationInvalidRef = -60002; { The authorization reference is invalid. }
	errAuthorizationInvalidTag = -60003; { The authorization tag is invalid. }
	errAuthorizationInvalidPointer = -60004; { The returned authorization is invalid. }
	errAuthorizationDenied = -60005; { The authorization was denied. }
	errAuthorizationCanceled = -60006; { The authorization was cancelled by the user. }
	errAuthorizationInteractionNotAllowed = -60007; { The authorization was denied since no user interaction was possible. }
	errAuthorizationInternal = -60008; { Unable to obtain authorization for this operation. }
	errAuthorizationExternalizeNotAllowed = -60009; { The authorization is not allowed to be converted to an external format. }
	errAuthorizationInternalizeNotAllowed = -60010; { The authorization is not allowed to be created from an external format. }
	errAuthorizationInvalidFlags = -60011; { The provided option flag(s) are invalid for this authorization operation. }
	errAuthorizationToolExecuteFailure = -60031; { The specified program could not be executed. }
	errAuthorizationToolEnvironmentError = -60032; { An invalid status was returned during execution of a privileged tool. }
	errAuthorizationBadAddress = -60033; { The requested socket address is invalid (must be 0-1023 inclusive). }
=======
=======
>>>>>>> origin/fixes_2_2
}

const
	errAuthorizationSuccess					= 0;      { The operation completed successfully. }
	errAuthorizationInvalidSet				= -60001; { The set parameter is invalid. }
	errAuthorizationInvalidRef				= -60002; { The authorization parameter is invalid. }
	errAuthorizationInvalidTag				= -60003; { The tag parameter is invalid. }
	errAuthorizationInvalidPointer			= -60004; { The authorizedRights parameter is invalid. }
	errAuthorizationDenied					= -60005; { The authorization was denied. }
	errAuthorizationCanceled				= -60006; { The authorization was cancelled by the user. }
	errAuthorizationInteractionNotAllowed	= -60007; { The authorization was denied since no user interaction was possible. }
	errAuthorizationInternal                = -60008; { something else went wrong }
	errAuthorizationExternalizeNotAllowed	= -60009; { authorization externalization denied }
	errAuthorizationInternalizeNotAllowed	= -60010; { authorization internalization denied }
	errAuthorizationInvalidFlags            = -60011; { invalid option flag(s) }
	errAuthorizationToolExecuteFailure      = -60031; { cannot execute privileged tool }
	errAuthorizationToolEnvironmentError    = -60032; { privileged tool environment error }
	errAuthorizationBadAddress				= -60033; { invalid socket address requested }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

    errAuthorizationSuccess can't include a string as it's also errSecSuccess in libsecurity_keychain/lib/SecBase.h
}

const
	errAuthorizationSuccess = 0;
	errAuthorizationInvalidSet = -60001; { The authorization rights are invalid. }
	errAuthorizationInvalidRef = -60002; { The authorization reference is invalid. }
	errAuthorizationInvalidTag = -60003; { The authorization tag is invalid. }
	errAuthorizationInvalidPointer = -60004; { The returned authorization is invalid. }
	errAuthorizationDenied = -60005; { The authorization was denied. }
	errAuthorizationCanceled = -60006; { The authorization was cancelled by the user. }
	errAuthorizationInteractionNotAllowed = -60007; { The authorization was denied since no user interaction was possible. }
	errAuthorizationInternal = -60008; { Unable to obtain authorization for this operation. }
	errAuthorizationExternalizeNotAllowed = -60009; { The authorization is not allowed to be converted to an external format. }
	errAuthorizationInternalizeNotAllowed = -60010; { The authorization is not allowed to be created from an external format. }
	errAuthorizationInvalidFlags = -60011; { The provided option flag(s) are invalid for this authorization operation. }
	errAuthorizationToolExecuteFailure = -60031; { The specified program could not be executed. }
	errAuthorizationToolEnvironmentError = -60032; { An invalid status was returned during execution of a privileged tool. }
	errAuthorizationBadAddress = -60033; { The requested socket address is invalid (must be 0-1023 inclusive). }
>>>>>>> origin/fixes_2.4


{!
	@enum AuthorizationFlags
	Optional flags passed in to serveral Authorization APIs. See the description of AuthorizationCreate, AuthorizationCopyRights and AuthorizationFree for a description of how they affect those calls.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kAuthorizationFlagDefaults = 0;
	kAuthorizationFlagInteractionAllowed = 1 shl 0;
	kAuthorizationFlagExtendRights = 1 shl 1;
	kAuthorizationFlagPartialRights = 1 shl 2;
	kAuthorizationFlagDestroyRights = 1 shl 3;
	kAuthorizationFlagPreAuthorize = 1 shl 4;
<<<<<<< HEAD
	
	// private bits (do not use)
	kAuthorizationFlagNoData = 1 shl 20;
=======
=======
>>>>>>> origin/fixes_2_2
	kAuthorizationFlagDefaults              = 0;
	kAuthorizationFlagInteractionAllowed	= (1 shl 0);
	kAuthorizationFlagExtendRights			= (1 shl 1);
	kAuthorizationFlagPartialRights			= (1 shl 2);
	kAuthorizationFlagDestroyRights			= (1 shl 3);
	kAuthorizationFlagPreAuthorize			= (1 shl 4);
	
	// private bits (do not use)
	kAuthorizationFlagNoData                = (1 shl 20);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	
	// private bits (do not use)
	kAuthorizationFlagNoData = 1 shl 20;
>>>>>>> origin/fixes_2.4


{!
	@typedef AuthorizationFlags
	Optional flags passed in to AuthorizationCreate.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	AuthorizationFlags = UInt32;
=======
type AuthorizationFlags = UInt32;
>>>>>>> graemeg/fixes_2_2
=======
type AuthorizationFlags = UInt32;
>>>>>>> origin/fixes_2_2
=======
type
	AuthorizationFlags = UInt32;
>>>>>>> origin/fixes_2.4


{!
	@enum AuthorizationRightFlags
	Flags returned in the flags field of ItemSet Items when calling AuthorizationCopyRights().
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAuthorizationFlagCanNotPreAuthorize = 1 shl 0;
=======
	kAuthorizationFlagCanNotPreAuthorize = (1 shl 0);
>>>>>>> graemeg/fixes_2_2
=======
	kAuthorizationFlagCanNotPreAuthorize = (1 shl 0);
>>>>>>> origin/fixes_2_2
=======
	kAuthorizationFlagCanNotPreAuthorize = 1 shl 0;
>>>>>>> origin/fixes_2.4


{!
	@typedef AuthorizationRef
	Opaque reference to an authorization object.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AuthorizationRef = ^OpaqueAuthorization;
	OpaqueAuthorization = record end;
=======
	AuthorizationRef = ^SInt32;
>>>>>>> graemeg/cpstrnew
=======
	AuthorizationRef = ^SInt32;
>>>>>>> graemeg/cpstrnew
=======
	AuthorizationRef = ^SInt32;
>>>>>>> graemeg/cpstrnew
=======
	AuthorizationRef = ^SInt32;
>>>>>>> origin/cpstrnew
=======
	AuthorizationRef							= ^SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	AuthorizationRef							= ^SInt32;
>>>>>>> origin/fixes_2_2
=======
	AuthorizationRef = ^SInt32;
>>>>>>> origin/fixes_2.4


{!
	@typedef AuthorizationString
	A zero terminated string in UTF-8 encoding.
}
type AuthorizationString = CStringPtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	 ConstAuthorizationString = ConstCStringPtr;	{ will need later in AuthorizationPlugin }
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
	 ConstAuthorizationString = ConstCStringPtr;	{ will need later in AuthorizationPlugin }
>>>>>>> origin/fixes_2.4

{!
	@struct AuthorizationItem
	Each AuthorizationItem describes a single string-named item with optional
	parameter value. The value must be contiguous memory of valueLength bytes;
	internal structure is defined separately for each name.

	@field name name of the item, as an AuthorizationString. Mandatory.
	@field valueLength Number of bytes in parameter value. Must be 0 if no parameter value.
	@field value Pointer to the optional parameter value associated with name.
	Must be NULL if no parameter value.
	@field flags Reserved field. Must be set to 0 on creation. Do not modify after that.
}
type
	AuthorizationItem = record
		name: AuthorizationString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		valueLength: size_t;
		value: UnivPtr;
=======
		valueLength: UInt32;
		value: Ptr;
>>>>>>> graemeg/fixes_2_2
=======
		valueLength: UInt32;
		value: Ptr;
>>>>>>> origin/fixes_2_2
=======
		valueLength: size_t;
		value: UnivPtr;
>>>>>>> origin/fixes_2.4
		flags: UInt32;
	end;
	AuthorizationItemPtr = ^AuthorizationItem;


{!
	@struct AuthorizationItemSet
	An AuthorizationItemSet structure represents a set of zero or more AuthorizationItems.  Since it is a set it should not contain any identical AuthorizationItems.

	@field count Number of items identified by items.
	@field items Pointer to an array of items.
}
type
	AuthorizationItemSet = record
		count: UInt32;
		items: AuthorizationItemPtr;
	end;
	AuthorizationItemSetPtr = ^AuthorizationItemSet;

{!
	@struct AuthorizationExternalForm
	An AuthorizationExternalForm structure can hold the externalized form of
	an AuthorizationRef. As such, it can be transmitted across IPC channels
	to other processes, which can re-internalize it to recover a valid AuthorizationRef
	handle.
	The data contained in an AuthorizationExternalForm should be considered opaque.

	SECURITY NOTE: Applications should take care to not disclose the AuthorizationExternalForm to
	potential attackers since it would authorize rights to them.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAuthorizationExternalFormLength = 32;
=======
const kAuthorizationExternalFormLength = 32;
>>>>>>> graemeg/fixes_2_2
=======
const kAuthorizationExternalFormLength = 32;
>>>>>>> origin/fixes_2_2
=======
const
	kAuthorizationExternalFormLength = 32;
>>>>>>> origin/fixes_2.4

type
	AuthorizationExternalForm = record
		bytes: packed array[0..(kAuthorizationExternalFormLength)-1] of char;
	end;


{!
	@typedef AuthorizationRights
	An AuthorizationItemSet representing a set of rights each with an associated argument (value).
	Each argument value is as defined for the specific right they belong to.  Argument values may not contain pointers as the should be copyable to different address spaces.
}
type AuthorizationRights = AuthorizationItemSet;
type AuthorizationRightsPtr = ^AuthorizationRights;
type AuthorizationRightsPtrPtr = ^AuthorizationRightsPtr;


{!
	@typedef AuthorizationEnvironment
	An AuthorizationItemSet representing environmental information of potential use
	to authorization decisions.
}
type AuthorizationEnvironment = AuthorizationItemSet;
type AuthorizationEnvironmentPtr = ^AuthorizationEnvironment;


{!
    @function AuthorizationCreate
    Create a new autorization object which can be used in other authorization calls.  When the authorization is no longer needed AuthorizationFree should be called.

	When the kAuthorizationFlagInteractionAllowed flag is set, user interaction will happen when required.  Failing to set this flag will result in this call failing with a errAuthorizationInteractionNotAllowed status when interaction is required.

	Setting the kAuthorizationFlagExtendRights flag will extend the currently available rights. If this flag is set the returned AuthorizationRef will grant all the rights requested when errAuthorizationSuccess is returned. If this flag is not set the operation will almost certainly succeed, but no attempt will be made to make the requested rights availible.
		Call AuthorizationCopyRights to figure out which of the requested rights are granted by the returned AuthorizationRef.

	Setting the kAuthorizationFlagPartialRights flag will cause this call to succeed if only some of the requested rights are being granted by the returned AuthorizationRef. Unless this flag is set this API will fail if not all the requested rights could be obtained.

	Setting the kAuthorizationFlagDestroyRights flag will prevent any rights obtained during this call from being preserved after returning from this API (This is most useful when the authorization parameter is NULL and the caller doesn't want to affect the session state in any way).

	Setting the kAuthorizationFlagPreAuthorize flag will pre authorize the requested rights so that at a later time -- by calling AuthorizationMakeExternalForm() follow by AuthorizationCreateFromExternalForm() -- the obtained rights can be used in a different process.  Rights that can't be preauthorized will be treated as if they were authorized for the sake of returning an error (in other words if all rights are either authorized or could not be preauthorized this call will still succeed).
		The rights which could not be preauthorized are not currently authorized and may fail to authorize when a later call to AuthorizationCopyRights() is made, unless the kAuthorizationFlagExtendRights and kAuthorizationFlagInteractionAllowed flags are set.  Even then they might still fail if the user does not supply the correct credentials.
		The reason for passing in this flag is to provide correct audit trail information and to avoid unnecessary user interaction.

    @param rights (input/optional) An AuthorizationItemSet containing rights for which authorization is being requested.  If none are specified the resulting AuthorizationRef will authorize nothing at all.
    @param environment (input/optional) An AuthorizationItemSet containing enviroment state used when making the autorization decision.  See the AuthorizationEnvironment type for details.
    @param flags (input) options specified by the AuthorizationFlags enum.  set all unused bits to zero to allow for future expansion.
    @param authorization (output optional) A pointer to an AuthorizationRef to be returned.  When the returned AuthorizationRef is no longer needed AuthorizationFree should be called to prevent anyone from using the aquired rights.  If NULL is specified no new rights are returned, but the system will attempt to authorize all the requested rights and return the appropriate status.

    @result errAuthorizationSuccess 0 authorization or all requested rights succeeded.

	errAuthorizationDenied -60005 The authorization for one or more of the requested rights was denied.

	errAuthorizationCanceled -60006 The authorization was cancelled by the user.

	errAuthorizationInteractionNotAllowed -60007 The authorization was denied since no interaction with the user was allowed.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationCreate( rights: AuthorizationRightsPtr; environment: AuthorizationEnvironmentPtr; flags: AuthorizationFlags; var authorization: AuthorizationRef ): OSStatus; external name '_AuthorizationCreate';
=======
function AuthorizationCreate(rights: AuthorizationRightsPtr; environment: AuthorizationEnvironmentPtr; flags: AuthorizationFlags; var authorization: AuthorizationRef): OSStatus; external name '_AuthorizationCreate';
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationCreate(rights: AuthorizationRightsPtr; environment: AuthorizationEnvironmentPtr; flags: AuthorizationFlags; var authorization: AuthorizationRef): OSStatus; external name '_AuthorizationCreate';
>>>>>>> origin/fixes_2_2
=======
function AuthorizationCreate( rights: AuthorizationRightsPtr; environment: AuthorizationEnvironmentPtr; flags: AuthorizationFlags; var authorization: AuthorizationRef ): OSStatus; external name '_AuthorizationCreate';
>>>>>>> origin/fixes_2.4


{!
    @function AuthorizationFree
    Destroy an AutorizationRef object. If the kAuthorizationFlagDestroyRights flag is passed,
	any rights associated with the authorization are lost. Otherwise, only local resources
	are released, and the rights may still be available to other clients.

	Setting the kAuthorizationFlagDestroyRights flag will prevent any rights that were obtained by the specified authorization object to be preserved after returning from this API.  This effectivaly locks down all potentially shared authorizations.

    @param authorization (input) The authorization object on which this operation is performed.
	
	@param flags (input) Bit mask of option flags to this call.

    @result errAuthorizationSuccess 0 No error.

    errAuthorizationInvalidRef -60002 The authorization parameter is invalid.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationFree( authorization: AuthorizationRef; flags: AuthorizationFlags ): OSStatus; external name '_AuthorizationFree';

=======
function AuthorizationFree(authorization: AuthorizationRef;flags: AuthorizationFlags): OSStatus; external name '_AuthorizationFree';
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationFree(authorization: AuthorizationRef;flags: AuthorizationFlags): OSStatus; external name '_AuthorizationFree';
>>>>>>> origin/fixes_2_2
=======
function AuthorizationFree( authorization: AuthorizationRef; flags: AuthorizationFlags ): OSStatus; external name '_AuthorizationFree';

>>>>>>> origin/fixes_2.4

{!
	@function AuthorizationCopyRights
    Given a set of rights, return the subset that is currently authorized
    by the AuthorizationRef given.

	When the kAuthorizationFlagInteractionAllowed flag is set, user interaction will happen when required.  Failing to set this flag will result in this call failing with a errAuthorizationInteractionNotAllowed status when interaction is required.

	Setting the kAuthorizationFlagExtendRights flag will extend the currently available rights.

	Setting the kAuthorizationFlagPartialRights flag will cause this call to succeed if only some of the requested rights are being granted by the returned AuthorizationRef.  Unless this flag is set this API will fail if not all the requested rights could be obtained.

	Setting the kAuthorizationFlagDestroyRights flag will prevent any additional rights obtained during this call from being preserved after returning from this API.

	Setting the kAuthorizationFlagPreAuthorize flag will pre authorize the requested rights so that at a later time -- by calling AuthorizationMakeExternalForm() follow by AuthorizationCreateFromExternalForm() -- the obtained rights can be used in a different process.  Rights that can't be preauthorized will be treated as if they were authorized for the sake of returning an error (in other words if all rights are either authorized or could not be preauthorized this call will still succeed), and they will be returned in authorizedRights with their kAuthorizationFlagCanNotPreAuthorize bit in the flags field set to 1.
		The rights which could not be preauthorized are not currently authorized and may fail to authorize when a later call to AuthorizationCopyRights() is made, unless the kAuthorizationFlagExtendRights and kAuthorizationFlagInteractionAllowed flags are set.  Even then they might still fail if the user does not supply the correct credentials.
		The reason for passing in this flag is to provide correct audit trail information and to avoid unnecessary user interaction.

	Setting the kAuthorizationFlagPreAuthorize flag will pre authorize the requested rights so that at a later time -- by calling AuthorizationMakeExternalForm() follow by AuthorizationCreateFromExternalForm() -- the obtained rights can be used in a different process.  When this flags is specified rights that can't be preauthorized will be returned as if they were authorized with their kAuthorizationFlagCanNotPreAuthorize bit in the flags field set to 1.  These rights are not currently authorized and may fail to authorize later unless kAuthorizationFlagExtendRights and kAuthorizationFlagInteractionAllowed flags are set when the actual authorization is done.  And even then they might still fail if the user does not supply the correct credentials.

    @param authorization (input) The authorization object on which this operation is performed.
    @param rights (input) A rights set (see AuthorizationCreate).
    @param environment (input/optional) An AuthorizationItemSet containing enviroment state used when making the autorization decision.  See the AuthorizationEnvironment type for details.
    @param flags (input) options specified by the AuthorizationFlags enum.  set all unused bits to zero to allow for future expansion.
    @param authorizedRights (output/optional) A pointer to a newly allocated AuthorizationInfoSet in which the authorized subset of rights are returned (authorizedRights should be deallocated by calling AuthorizationFreeItemSet() when it is no longer needed).  If NULL the only information returned is the status.  Note that if the kAuthorizationFlagPreAuthorize flag was specified rights that could not be preauthorized are returned in authorizedRights, but their flags contains the kAuthorizationFlagCanNotPreAuthorize bit.

    @result errAuthorizationSuccess 0 No error.

	errAuthorizationInvalidRef -60002 The authorization parameter is invalid.

    errAuthorizationInvalidSet -60001 The rights parameter is invalid.

    errAuthorizationInvalidPointer -60004 The authorizedRights parameter is invalid.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationCopyRights( authorization: AuthorizationRef; const (*var*) rights: AuthorizationRights; environment: AuthorizationEnvironmentPtr; flags: AuthorizationFlags; authorizedRights: AuthorizationRightsPtrPtr ): OSStatus; external name '_AuthorizationCopyRights';
=======
function AuthorizationCopyRights(authorization: AuthorizationRef; const (*var*) rights: AuthorizationRights; environment: AuthorizationEnvironmentPtr; flags: AuthorizationFlags; authorizedRights: AuthorizationRightsPtrPtr): OSStatus; external name '_AuthorizationCopyRights';
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationCopyRights(authorization: AuthorizationRef; const (*var*) rights: AuthorizationRights; environment: AuthorizationEnvironmentPtr; flags: AuthorizationFlags; authorizedRights: AuthorizationRightsPtrPtr): OSStatus; external name '_AuthorizationCopyRights';
>>>>>>> origin/fixes_2_2
=======
function AuthorizationCopyRights( authorization: AuthorizationRef; const (*var*) rights: AuthorizationRights; environment: AuthorizationEnvironmentPtr; flags: AuthorizationFlags; authorizedRights: AuthorizationRightsPtrPtr ): OSStatus; external name '_AuthorizationCopyRights';
>>>>>>> origin/fixes_2.4

{!
	@function AuthorizationCopyInfo
	Returns sideband information (e.g. access credentials) obtained from a call to AuthorizationCreate.  The format of this data depends of the tag specified.
	
    @param authorization (input) The authorization object on which this operation is performed.
    @param tag (input/optional) An optional string tag specifing which sideband information should be returned.  When NULL is specified all available information is returned.
    @param flags (input) options specified by the AuthorizationFlags enum.  set all unused bits to zero to allow for future expansion.
    @param info (output) A pointer to a newly allocated AuthorizationInfoSet in which the requested sideband infomation is returned (info should be deallocated by calling AuthorizationFreeItemSet() when it is no longer needed).

    @result errAuthorizationSuccess 0 No error.

    errAuthorizationInvalidRef -60002 The authorization parameter is invalid.

    errAuthorizationInvalidTag -60003 The tag parameter is invalid.

    errAuthorizationInvalidPointer -60004 The info parameter is invalid.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationCopyInfo( authorization: AuthorizationRef; tag: AuthorizationString; var info: AuthorizationItemSetPtr ): OSStatus; external name '_AuthorizationCopyInfo';
=======
function AuthorizationCopyInfo(authorization: AuthorizationRef; tag: AuthorizationString; var info: AuthorizationItemSetPtr): OSStatus; external name '_AuthorizationCopyInfo';
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationCopyInfo(authorization: AuthorizationRef; tag: AuthorizationString; var info: AuthorizationItemSetPtr): OSStatus; external name '_AuthorizationCopyInfo';
>>>>>>> origin/fixes_2_2
=======
function AuthorizationCopyInfo( authorization: AuthorizationRef; tag: AuthorizationString; var info: AuthorizationItemSetPtr ): OSStatus; external name '_AuthorizationCopyInfo';
>>>>>>> origin/fixes_2.4

{!
	@function AuthorizationMakeExternalForm
	Turn an Authorization into an external "byte blob" form so it can be
	transmitted to another process.
	Note that *storing* the external form somewhere will probably not do what
	you want, since authorizations are bounded by sessions, processes, and possibly
	time limits. This is for online transmission of authorizations.
	
	@param authorization The (valid) authorization reference to externalize
	@param extForm Pointer to an AuthorizationExternalForm variable to fill.
	
        @result errAuthorizationSuccess 0 No error.

        errAuthorizationExternalizeNotAllowed -60009 Externalizing this authorization is not allowed.

        errAuthorizationInvalidRef -60002 The authorization parameter is invalid.


}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationMakeExternalForm( authorization: AuthorizationRef; var extForm: AuthorizationExternalForm ): OSStatus; external name '_AuthorizationMakeExternalForm';
=======
function AuthorizationMakeExternalForm(authorization: AuthorizationRef; var extForm: AuthorizationExternalForm): OSStatus; external name '_AuthorizationMakeExternalForm';
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationMakeExternalForm(authorization: AuthorizationRef; var extForm: AuthorizationExternalForm): OSStatus; external name '_AuthorizationMakeExternalForm';
>>>>>>> origin/fixes_2_2
=======
function AuthorizationMakeExternalForm( authorization: AuthorizationRef; var extForm: AuthorizationExternalForm ): OSStatus; external name '_AuthorizationMakeExternalForm';
>>>>>>> origin/fixes_2.4


{!
	@function AuthorizationCreateFromExternalForm
<<<<<<< HEAD
<<<<<<< HEAD
	Internalize the external "byte blob" form of an authorization reference.
=======
=======
>>>>>>> origin/fixes_2_2
	Turn an Authorization into an external "byte blob" form so it can be
	transmitted to another process.
	Note that *storing* the external form somewhere will probably not do what
	you want, since authorizations are bounded by sessions, processes, and possibly
	time limits. This is for online transmission of authorizations.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	
	@param extForm Pointer to an AuthorizationExternalForm value.
	@param authorization Will be filled with a valid AuthorizationRef on success.
	
	@result errAuthorizationInternalizeNotAllowed -60010 Internalizing this authorization is not allowed.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationCreateFromExternalForm( const (*var*) extForm: AuthorizationExternalForm; var authorization: AuthorizationRef ): OSStatus; external name '_AuthorizationCreateFromExternalForm';
=======
function AuthorizationCreateFromExternalForm(const (*var*) extForm: AuthorizationExternalForm; var authorization: AuthorizationRef): OSStatus; external name '_AuthorizationCreateFromExternalForm';
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationCreateFromExternalForm(const (*var*) extForm: AuthorizationExternalForm; var authorization: AuthorizationRef): OSStatus; external name '_AuthorizationCreateFromExternalForm';
>>>>>>> origin/fixes_2_2
=======
function AuthorizationCreateFromExternalForm( const (*var*) extForm: AuthorizationExternalForm; var authorization: AuthorizationRef ): OSStatus; external name '_AuthorizationCreateFromExternalForm';
>>>>>>> origin/fixes_2.4


{!
	@function AuthorizationFreeItemSet
	Release the memory allocated for an AuthorizationItemSet that was allocated
	by an API call.
	
    @param set The AuthorizationItemSet to deallocate.

    @result errAuthorizationSuccess 0 No error.

    errAuthorizationInvalidSet -60001 The set parameter is invalid.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationFreeItemSet( var setx: AuthorizationItemSet ): OSStatus; external name '_AuthorizationFreeItemSet';
=======
function AuthorizationFreeItemSet(var setx: AuthorizationItemSet): OSStatus; external name '_AuthorizationFreeItemSet';
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationFreeItemSet(var setx: AuthorizationItemSet): OSStatus; external name '_AuthorizationFreeItemSet';
>>>>>>> origin/fixes_2_2
=======
function AuthorizationFreeItemSet( var setx: AuthorizationItemSet ): OSStatus; external name '_AuthorizationFreeItemSet';
>>>>>>> origin/fixes_2.4


{!
	@function AuthorizationExecuteWithPrivileges
	Run an executable tool with enhanced privileges after passing
	suitable authorization procedures.
	
	@param authorization An authorization reference that is used to authorize
	access to the enhanced privileges. It is also passed to the tool for
	further access control.
	@param pathToTool Full pathname to the tool that should be executed
	with enhanced privileges.
	@param options Option bits (reserved). Must be zero.
	@param arguments An argv-style vector of strings to be passed to the tool.
	@param communicationsPipe Assigned a UNIX stdio FILE pointer for
	a bidirectional pipe to communicate with the tool. The tool will have
	this pipe as its standard I/O channels (stdin/stdout). If NULL, do not
	establish a communications pipe.
<<<<<<< HEAD
<<<<<<< HEAD

 	@discussion This function has been deprecated and should no longer be used.
 	Use a launchd-launched helper tool and/or the Service Mangement framework
 	for this functionality.
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 }
type
  Arg10000Type = array[0..10000] of CStringPtr;
  Arg10000TypePtr = ^Arg10000Type;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AuthorizationExecuteWithPrivileges( authorization: AuthorizationRef; pathToTool: CStringPtr; options: AuthorizationFlags; arguments: Arg10000TypePtr; communicationsPipe: UnivPtr ): OSStatus; external name '_AuthorizationExecuteWithPrivileges';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_7,__IPHONE_NA,__IPHONE_NA) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
function AuthorizationExecuteWithPrivileges(authorization: AuthorizationRef; pathToTool: CStringPtr; options: AuthorizationFlags; arguments: Arg10000TypePtr; communicationsPipe: UnivPtr): OSStatus; external name '_AuthorizationExecuteWithPrivileges';
>>>>>>> graemeg/fixes_2_2
=======
function AuthorizationExecuteWithPrivileges(authorization: AuthorizationRef; pathToTool: CStringPtr; options: AuthorizationFlags; arguments: Arg10000TypePtr; communicationsPipe: UnivPtr): OSStatus; external name '_AuthorizationExecuteWithPrivileges';
>>>>>>> origin/fixes_2_2
=======
function AuthorizationExecuteWithPrivileges( authorization: AuthorizationRef; pathToTool: CStringPtr; options: AuthorizationFlags; arguments: Arg10000TypePtr; communicationsPipe: UnivPtr ): OSStatus; external name '_AuthorizationExecuteWithPrivileges';
>>>>>>> origin/fixes_2.4
// communicationsPipe not yet supported


{!
	@function AuthorizationCopyPrivilegedReference
	From within a tool launched via the AuthorizationExecuteWithPrivileges function
	ONLY, retrieve the AuthorizationRef originally passed to that function.
	While AuthorizationExecuteWithPrivileges already verified the authorization to
	launch your tool, the tool may want to avail itself of any additional pre-authorizations
	the caller may have obtained through that reference.
<<<<<<< HEAD
<<<<<<< HEAD
 
	@discussion This function has been deprecated and should no longer be used.
	Use a launchd-launched helper tool and/or the Service Mangement framework
	for this functionality.
 }
function AuthorizationCopyPrivilegedReference( var authorization: AuthorizationRef; flags: AuthorizationFlags ): OSStatus; external name '_AuthorizationCopyPrivilegedReference';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_1,__MAC_10_7,__IPHONE_NA,__IPHONE_NA) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 }
function AuthorizationCopyPrivilegedReference( var authorization: AuthorizationRef; flags: AuthorizationFlags ): OSStatus; external name '_AuthorizationCopyPrivilegedReference';


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
