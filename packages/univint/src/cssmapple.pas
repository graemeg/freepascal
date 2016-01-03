{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2000-2013 Apple Inc. All Rights Reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 *
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 * Copyright (c) 2000-2004 Apple Computer, Inc. All Rights Reserved.
 * 
 * @APPLE_LICENSE_HEADER_START@
 * 
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
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *
=======
 * 
>>>>>>> graemeg/cpstrnew
=======
 * 
>>>>>>> graemeg/cpstrnew
=======
 * 
>>>>>>> graemeg/cpstrnew
=======
 * 
>>>>>>> origin/cpstrnew
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *
=======
 * 
>>>>>>> graemeg/cpstrnew
=======
 * 
>>>>>>> graemeg/cpstrnew
=======
 * 
>>>>>>> graemeg/cpstrnew
=======
 * 
>>>>>>> origin/cpstrnew
 * @APPLE_LICENSE_HEADER_END@
 *
 * cssmapple.h -- CSSM features specific to Apple's Implementation
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, August 2015 }

=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> origin/cpstrnew
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

unit cssmapple;
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
	{$setc TARGET_CPU_ARM64 := FALSE}
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
>>>>>>> origin/cpstrnew
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
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
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
uses MacTypes,MacOSXPosix,cssmerr,cssmtype,x509defs,certextensions;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$packrecords c}


{ Guids for standard Apple addin modules. }

{ CSSM itself: (87191ca0-0fc9-11d4-849a-000502b52122) }
var gGuidCssm: CSSM_GUID; external name '_gGuidCssm'; (* attribute const *)

{ File based DL (aka "Keychain DL"): (87191ca1-0fc9-11d4-849a-000502b52122) }
var gGuidAppleFileDL: CSSM_GUID; external name '_gGuidAppleFileDL'; (* attribute const *)

{ Core CSP (local space): (87191ca2-0fc9-11d4-849a-000502b52122) }
var gGuidAppleCSP: CSSM_GUID; external name '_gGuidAppleCSP'; (* attribute const *)

{ Secure CSP/DL (aka "Keychain CSPDL): (87191ca3-0fc9-11d4-849a-000502b52122) }
var gGuidAppleCSPDL: CSSM_GUID; external name '_gGuidAppleCSPDL'; (* attribute const *)

{ X509 Certificate CL: (87191ca4-0fc9-11d4-849a-000502b52122) }
var gGuidAppleX509CL: CSSM_GUID; external name '_gGuidAppleX509CL'; (* attribute const *)

{ X509 Certificate TP: (87191ca5-0fc9-11d4-849a-000502b52122) }
var gGuidAppleX509TP: CSSM_GUID; external name '_gGuidAppleX509TP'; (* attribute const *)

{ DLAP/OpenDirectory access DL: (87191ca6-0fc9-11d4-849a-000502b52122) }
var gGuidAppleLDAPDL: CSSM_GUID; external name '_gGuidAppleLDAPDL'; (* attribute const *)

{ TP for ".mac" related policies: (87191ca7-0fc9-11d4-849a-000502b52122) }
var gGuidAppleDotMacTP: CSSM_GUID; external name '_gGuidAppleDotMacTP'; (* attribute const *)

{ Smartcard CSP/DL: (87191ca8-0fc9-11d4-849a-000502b52122) }
var gGuidAppleSdCSPDL: CSSM_GUID; external name '_gGuidAppleSdCSPDL'; (* attribute const *)

{ DL for ".mac" certificate access: (87191ca9-0fc9-11d4-849a-000502b52122) }
var gGuidAppleDotMacDL: CSSM_GUID; external name '_gGuidAppleDotMacDL'; (* attribute const *)


{ Apple defined WORDID values }
const
	CSSM_WORDID_KEYCHAIN_PROMPT = CSSM_WORDID_VENDOR_START;
	CSSM_WORDID_KEYCHAIN_LOCK = CSSM_WORDID_VENDOR_START + 1;
	CSSM_WORDID_KEYCHAIN_CHANGE_LOCK = CSSM_WORDID_VENDOR_START + 2;
	CSSM_WORDID_PROCESS = CSSM_WORDID_VENDOR_START + 3;
	CSSM_WORDID__RESERVED_1 = CSSM_WORDID_VENDOR_START + 4;		{ was used in 10.2 test seeds; no longer in use }
	CSSM_WORDID_SYMMETRIC_KEY = CSSM_WORDID_VENDOR_START + 5;
	CSSM_WORDID_SYSTEM = CSSM_WORDID_VENDOR_START + 6;
	CSSM_WORDID_KEY = CSSM_WORDID_VENDOR_START + 7;
	CSSM_WORDID_PIN = CSSM_WORDID_VENDOR_START + 8;
	CSSM_WORDID_PREAUTH = CSSM_WORDID_VENDOR_START + 9;
	CSSM_WORDID_PREAUTH_SOURCE = CSSM_WORDID_VENDOR_START + 10;
	CSSM_WORDID_ASYMMETRIC_KEY = CSSM_WORDID_VENDOR_START + 11;
	CSSM_WORDID__FIRST_UNUSED = CSSM_WORDID_VENDOR_START + 12;

{ Apple defined ACL subject and credential types }
const
	CSSM_ACL_SUBJECT_TYPE_KEYCHAIN_PROMPT = CSSM_WORDID_KEYCHAIN_PROMPT;
	CSSM_ACL_SUBJECT_TYPE_PROCESS = CSSM_WORDID_PROCESS;
	CSSM_ACL_SUBJECT_TYPE_CODE_SIGNATURE = CSSM_WORDID_SIGNATURE;
	CSSM_ACL_SUBJECT_TYPE_COMMENT = CSSM_WORDID_COMMENT;
	CSSM_ACL_SUBJECT_TYPE_SYMMETRIC_KEY = CSSM_WORDID_SYMMETRIC_KEY;
	CSSM_ACL_SUBJECT_TYPE_PREAUTH = CSSM_WORDID_PREAUTH;
	CSSM_ACL_SUBJECT_TYPE_PREAUTH_SOURCE = CSSM_WORDID_PREAUTH_SOURCE;
	CSSM_ACL_SUBJECT_TYPE_ASYMMETRIC_KEY = CSSM_WORDID_ASYMMETRIC_KEY;

const
	CSSM_SAMPLE_TYPE_KEYCHAIN_PROMPT = CSSM_WORDID_KEYCHAIN_PROMPT;
	CSSM_SAMPLE_TYPE_KEYCHAIN_LOCK = CSSM_WORDID_KEYCHAIN_LOCK;
	CSSM_SAMPLE_TYPE_KEYCHAIN_CHANGE_LOCK = CSSM_WORDID_KEYCHAIN_CHANGE_LOCK;
	CSSM_SAMPLE_TYPE_PROCESS = CSSM_WORDID_PROCESS;
	CSSM_SAMPLE_TYPE_COMMENT = CSSM_WORDID_COMMENT;
	CSSM_SAMPLE_TYPE_RETRY_ID = CSSM_WORDID_PROPAGATE;
	CSSM_SAMPLE_TYPE_SYMMETRIC_KEY = CSSM_WORDID_SYMMETRIC_KEY;
	CSSM_SAMPLE_TYPE_PREAUTH = CSSM_WORDID_PREAUTH;
	CSSM_SAMPLE_TYPE_ASYMMETRIC_KEY = CSSM_WORDID_ASYMMETRIC_KEY;
	// there is no CSSM_SAMPLE_TYPE_PREAUTH_SOURCE


{ Apple-defined ACL authorization tags }
const
	CSSM_ACL_AUTHORIZATION_CHANGE_ACL = CSSM_ACL_AUTHORIZATION_TAG_VENDOR_DEFINED_START;
	CSSM_ACL_AUTHORIZATION_CHANGE_OWNER = CSSM_ACL_AUTHORIZATION_TAG_VENDOR_DEFINED_START + 1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> origin/cpstrnew
	// the "pre-auth" tags form a contiguous range of (up to) 64K pre-authorizations
	CSSM_ACL_AUTHORIZATION_PREAUTH_BASE = CSSM_ACL_AUTHORIZATION_TAG_VENDOR_DEFINED_START + $1000000;
	CSSM_ACL_AUTHORIZATION_PREAUTH_END = CSSM_ACL_AUTHORIZATION_PREAUTH_BASE + $10000;

{ pre-authorization conversions (auth-tag to slot and back) }
{
#define CSSM_ACL_AUTHORIZATION_PREAUTH(slot) \
		(CSSM_ACL_AUTHORIZATION_PREAUTH_BASE + (slot))
#define CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT(auth) \
		((auth) - CSSM_ACL_AUTHORIZATION_PREAUTH_BASE)
#define CSSM_ACL_AUTHORIZATION_IS_PREAUTH(auth) \
		((auth) >= CSSM_ACL_AUTHORIZATION_PREAUTH_BASE && \
		 (auth) < CSSM_ACL_AUTHORIZATION_PREAUTH_END)
}


function CSSM_ACL_AUTHORIZATION_PREAUTH(slot: UInt32): UInt32; inline;
function CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT(auth: UInt32): UInt32; inline;
function CSSM_ACL_AUTHORIZATION_IS_PREAUTH(auth: UInt32): Boolean; inline;


{ Parameters and structures for Apple-defined ACL subjects and samples }

const
{ types of code signatures - item 1 of CSSM_ACL_SUBJECT_TYPE_CODE_SIGNATURE subjects }
	CSSM_ACL_CODE_SIGNATURE_INVALID = 0; { standard OS X code signature }
	CSSM_ACL_CODE_SIGNATURE_OSX = 1;		{ standard OS X code signature }

{ ACL subjects of type PROCESS }

const
{ PROCESS_SUBJECT mask fields }
	CSSM_ACL_MATCH_UID = $01;			{ match userid against uid field }
	CSSM_ACL_MATCH_GID = $02;			{ match groupid against gid field }
	CSSM_ACL_MATCH_HONOR_ROOT = $100;	{ let root (uid 0) match any userid }
	CSSM_ACL_MATCH_BITS = CSSM_ACL_MATCH_UID or CSSM_ACL_MATCH_GID;

const
{ PROCESS_SUBJECT structure version field }
	CSSM_ACL_PROCESS_SELECTOR_CURRENT_VERSION = $101;

type
	cssm_acl_process_subject_selectorPtr = ^cssm_acl_process_subject_selector;
	cssm_acl_process_subject_selector = record
{ PROCESS_SUBJECT selector }
		version: UInt16;			{ version of this selector }
		mask: UInt16;			{ active fields mask }
		uid: UInt32;				{ effective user id match }
		gid: UInt32;				{ effective group id match }
	end;

{ ACL subjects of type KEYCHAIN_PROMPT }

const
{ KEYCHAIN_PROMPT structure version field }
	CSSM_ACL_KEYCHAIN_PROMPT_CURRENT_VERSION = $101;

const
{ KEYCHAIN_PROMPT operational flags }
	CSSM_ACL_KEYCHAIN_PROMPT_REQUIRE_PASSPHRASE = $0001; { require re-entering of passphrase }
	{ the following bits are ignored by 10.4 and earlier }
	CSSM_ACL_KEYCHAIN_PROMPT_UNSIGNED = $0010;			{ prompt for unsigned clients }
	CSSM_ACL_KEYCHAIN_PROMPT_UNSIGNED_ACT = $0020;		{ UNSIGNED bit overrides system default }
	CSSM_ACL_KEYCHAIN_PROMPT_INVALID = $0040;			{ prompt for invalid signed clients }
	CSSM_ACL_KEYCHAIN_PROMPT_INVALID_ACT = $0080;		{ INVALID bit overrides system default }

type
	cssm_acl_keychain_prompt_selectorPtr = ^cssm_acl_keychain_prompt_selector;
	cssm_acl_keychain_prompt_selector = record
{ KEYCHAIN_PROMPT selector }
		version: UInt16;			{ version of this selector }
		flags: UInt16;			{ flag bits }
	end;

{ ACL subjects of type CSSM_ACL_SUBJECT_TYPE_PREAUTH_SOURCE }
type
	CSSM_ACL_PREAUTH_TRACKING_STATE = UInt32;
const
{ preauth tracking state }
	CSSM_ACL_PREAUTH_TRACKING_COUNT_MASK = $ff;		{ mask for count status }
	CSSM_ACL_PREAUTH_TRACKING_BLOCKED = 0;		{ retries exhausted; the slot is blocked }
	{ 0 .. 255 is a count of (re)tries remaining }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> origin/cpstrnew
	{ bits or'ed into any count given }
	CSSM_ACL_PREAUTH_TRACKING_UNKNOWN = $40000000; { status of slot is unknown (ignore count) }
	CSSM_ACL_PREAUTH_TRACKING_AUTHORIZED = $80000000; { the slot is currently authorized (or'ed in) }


{ Apple defined values of a CSSM_DB_ACCESS_TYPE }
const
	CSSM_DB_ACCESS_RESET = $10000;	{ clear pre-authentications (or'ed bit) }


{ Apple defined algorithm IDs }
const
	CSSM_ALGID_APPLE_YARROW = CSSM_ALGID_VENDOR_DEFINED;
	CSSM_ALGID_AES = CSSM_ALGID_VENDOR_DEFINED + 1;				{ RijnDael }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_ALGID_FEE = CSSM_ALGID_VENDOR_DEFINED + 2;				{ FEE Key Generation }
=======
	CSSM_ALGID_FEE = CSSM_ALGID_VENDOR_DEFINED + 2;				{ FEE Key Generation } 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_ALGID_FEE = CSSM_ALGID_VENDOR_DEFINED + 2;				{ FEE Key Generation } 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_ALGID_FEE = CSSM_ALGID_VENDOR_DEFINED + 2;				{ FEE Key Generation } 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_ALGID_FEE = CSSM_ALGID_VENDOR_DEFINED + 2;				{ FEE Key Generation } 
>>>>>>> origin/cpstrnew
	CSSM_ALGID_FEE_MD5 = CSSM_ALGID_VENDOR_DEFINED + 3;			{ FEE/ElGamal signature w/ MD5 hash }
	CSSM_ALGID_FEE_SHA1 = CSSM_ALGID_VENDOR_DEFINED + 4;		{ FEE/ElGamal signature w/ SHA1 hash }
	CSSM_ALGID_FEED = CSSM_ALGID_VENDOR_DEFINED + 5;			{ 1:1 FEE asymmetric encryption }
	CSSM_ALGID_FEEDEXP = CSSM_ALGID_VENDOR_DEFINED + 6;			{ 2:1 FEE asymmetric encryption }
	CSSM_ALGID_ASC = CSSM_ALGID_VENDOR_DEFINED + 7;				{ Apple Secure Compression }
	CSSM_ALGID_SHA1HMAC_LEGACY = CSSM_ALGID_VENDOR_DEFINED + 8;	{ HMAC/SHA1, legacy compatible }
	CSSM_ALGID_KEYCHAIN_KEY = CSSM_ALGID_VENDOR_DEFINED + 9;	{ derive or manipulate keychain master keys }
	CSSM_ALGID_PKCS12_PBE_ENCR = CSSM_ALGID_VENDOR_DEFINED + 10;	{ PKCS12, encrypt/decrypt key }
	CSSM_ALGID_PKCS12_PBE_MAC = CSSM_ALGID_VENDOR_DEFINED + 11;	{ PKCS12, MAC key }
	CSSM_ALGID_SECURE_PASSPHRASE = CSSM_ALGID_VENDOR_DEFINED + 12;   { passphrase acquired by SecurityServer }
	CSSM_ALGID_PBE_OPENSSL_MD5 = CSSM_ALGID_VENDOR_DEFINED + 13; { traditional openssl key derivation }
	CSSM_ALGID_SHA256 = CSSM_ALGID_VENDOR_DEFINED + 14;			{ 256-bit SHA2 }
	CSSM_ALGID_SHA384 = CSSM_ALGID_VENDOR_DEFINED + 15;			{ 384-bit SHA2 }
	CSSM_ALGID_SHA512 = CSSM_ALGID_VENDOR_DEFINED + 16;			{ 512-bit SHA2 }
	CSSM_ALGID_ENTROPY_DEFAULT = CSSM_ALGID_VENDOR_DEFINED + 17;	{ default entropy source of (CSP) device, if any }
	CSSM_ALGID_SHA224 = CSSM_ALGID_VENDOR_DEFINED + 18;			{ SHA2, 224 bit }
	CSSM_ALGID_SHA224WithRSA = CSSM_ALGID_VENDOR_DEFINED + 19;	{ RSA signature on SHA224 digest }
	CSSM_ALGID_SHA256WithRSA = CSSM_ALGID_VENDOR_DEFINED + 20;	{ RSA signature on SHA256 digest }
	CSSM_ALGID_SHA384WithRSA = CSSM_ALGID_VENDOR_DEFINED + 21;	{ RSA signature on SHA384 digest }
	CSSM_ALGID_SHA512WithRSA = CSSM_ALGID_VENDOR_DEFINED + 22;	{ RSA signature on SHA512 digest }
	CSSM_ALGID_OPENSSH1 = CSSM_ALGID_VENDOR_DEFINED + 23;		{ OpenSSH v1 RSA key wrapping }
	CSSM_ALGID_SHA224WithECDSA = CSSM_ALGID_VENDOR_DEFINED + 24;	{ ECDSA signature on SHA224 digest }
	CSSM_ALGID_SHA256WithECDSA = CSSM_ALGID_VENDOR_DEFINED + 25;	{ ECDSA signature on SHA256 digest }
	CSSM_ALGID_SHA384WithECDSA = CSSM_ALGID_VENDOR_DEFINED + 26;	{ ECDSA signature on SHA384 digest }
	CSSM_ALGID_SHA512WithECDSA = CSSM_ALGID_VENDOR_DEFINED + 27;	{ ECDSA signature on SHA512 digest }
	CSSM_ALGID_ECDSA_SPECIFIED = CSSM_ALGID_VENDOR_DEFINED + 28;	{ ECDSA with separate digest algorithm specifier }
	CSSM_ALGID_ECDH_X963_KDF = CSSM_ALGID_VENDOR_DEFINED + 29;	{ ECDH with X9.63 key derivation }
	CSSM_ALGID__FIRST_UNUSED = CSSM_ALGID_VENDOR_DEFINED + 30;

{ Apple defined padding }
const
{ RFC 2246 section E.2 for SSLv2 rollback detection }
	CSSM_PADDING_APPLE_SSLv2 = CSSM_PADDING_VENDOR_DEFINED;


{ Apple defined keyblob formats }
const
	CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED = $80000000;
const
{ X509 SubjectPublicKeyInfo }
	CSSM_KEYBLOB_RAW_FORMAT_X509 = CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED;
	{ OpenSSH v1 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_KEYBLOB_RAW_FORMAT_OPENSSH = CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED + 1;
=======
	CSSM_KEYBLOB_RAW_FORMAT_OPENSSH = CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED + 1;		
>>>>>>> graemeg/cpstrnew
=======
	CSSM_KEYBLOB_RAW_FORMAT_OPENSSH = CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED + 1;		
>>>>>>> graemeg/cpstrnew
=======
	CSSM_KEYBLOB_RAW_FORMAT_OPENSSH = CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED + 1;		
>>>>>>> graemeg/cpstrnew
=======
	CSSM_KEYBLOB_RAW_FORMAT_OPENSSH = CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED + 1;		
>>>>>>> origin/cpstrnew
	{ openssl-style DSA private key }
	CSSM_KEYBLOB_RAW_FORMAT_OPENSSL = CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED + 2;
	{ OpenSSH v2 }
	CSSM_KEYBLOB_RAW_FORMAT_OPENSSH2 = CSSM_KEYBLOB_RAW_FORMAT_VENDOR_DEFINED + 3;

{ Apple adds some "common" error codes. CDSA does not define an official start value for this. }
const
	CSSM_CUSTOM_COMMON_ERROR_EXTENT = $00e0;
	CSSM_ERRCODE_NO_USER_INTERACTION = $00e0;
	CSSM_ERRCODE_USER_CANCELED = $00e1;
	CSSM_ERRCODE_SERVICE_NOT_AVAILABLE = $00e2;
	CSSM_ERRCODE_INSUFFICIENT_CLIENT_IDENTIFICATION = $00e3;
	CSSM_ERRCODE_DEVICE_RESET = $00e4;
	CSSM_ERRCODE_DEVICE_FAILED = $00e5;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_ERRCODE_IN_DARK_WAKE = $00e6;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
	CSSMERR_CSSM_NO_USER_INTERACTION = CSSM_CSSM_BASE_ERROR + CSSM_ERRCODE_NO_USER_INTERACTION;
	CSSMERR_AC_NO_USER_INTERACTION = CSSM_AC_BASE_ERROR + CSSM_ERRCODE_NO_USER_INTERACTION;
	CSSMERR_CSP_NO_USER_INTERACTION = CSSM_CSP_BASE_ERROR + CSSM_ERRCODE_NO_USER_INTERACTION;
	CSSMERR_CL_NO_USER_INTERACTION = CSSM_CL_BASE_ERROR + CSSM_ERRCODE_NO_USER_INTERACTION;
	CSSMERR_DL_NO_USER_INTERACTION = CSSM_DL_BASE_ERROR + CSSM_ERRCODE_NO_USER_INTERACTION;
	CSSMERR_TP_NO_USER_INTERACTION = CSSM_TP_BASE_ERROR + CSSM_ERRCODE_NO_USER_INTERACTION;
	CSSMERR_CSSM_USER_CANCELED = CSSM_CSSM_BASE_ERROR + CSSM_ERRCODE_USER_CANCELED;
	CSSMERR_AC_USER_CANCELED = CSSM_AC_BASE_ERROR + CSSM_ERRCODE_USER_CANCELED;
	CSSMERR_CSP_USER_CANCELED = CSSM_CSP_BASE_ERROR + CSSM_ERRCODE_USER_CANCELED;
	CSSMERR_CL_USER_CANCELED = CSSM_CL_BASE_ERROR + CSSM_ERRCODE_USER_CANCELED;
	CSSMERR_DL_USER_CANCELED = CSSM_DL_BASE_ERROR + CSSM_ERRCODE_USER_CANCELED;
	CSSMERR_TP_USER_CANCELED = CSSM_TP_BASE_ERROR + CSSM_ERRCODE_USER_CANCELED;
	CSSMERR_CSSM_SERVICE_NOT_AVAILABLE = CSSM_CSSM_BASE_ERROR + CSSM_ERRCODE_SERVICE_NOT_AVAILABLE;
	CSSMERR_AC_SERVICE_NOT_AVAILABLE = CSSM_AC_BASE_ERROR + CSSM_ERRCODE_SERVICE_NOT_AVAILABLE;
	CSSMERR_CSP_SERVICE_NOT_AVAILABLE = CSSM_CSP_BASE_ERROR + CSSM_ERRCODE_SERVICE_NOT_AVAILABLE;
	CSSMERR_CL_SERVICE_NOT_AVAILABLE = CSSM_CL_BASE_ERROR + CSSM_ERRCODE_SERVICE_NOT_AVAILABLE;
	CSSMERR_DL_SERVICE_NOT_AVAILABLE = CSSM_DL_BASE_ERROR + CSSM_ERRCODE_SERVICE_NOT_AVAILABLE;
	CSSMERR_TP_SERVICE_NOT_AVAILABLE = CSSM_TP_BASE_ERROR + CSSM_ERRCODE_SERVICE_NOT_AVAILABLE;
	CSSMERR_CSSM_INSUFFICIENT_CLIENT_IDENTIFICATION = CSSM_CSSM_BASE_ERROR + CSSM_ERRCODE_INSUFFICIENT_CLIENT_IDENTIFICATION;
	CSSMERR_AC_INSUFFICIENT_CLIENT_IDENTIFICATION = CSSM_AC_BASE_ERROR + CSSM_ERRCODE_INSUFFICIENT_CLIENT_IDENTIFICATION;
	CSSMERR_CSP_INSUFFICIENT_CLIENT_IDENTIFICATION = CSSM_CSP_BASE_ERROR + CSSM_ERRCODE_INSUFFICIENT_CLIENT_IDENTIFICATION;
	CSSMERR_CL_INSUFFICIENT_CLIENT_IDENTIFICATION = CSSM_CL_BASE_ERROR + CSSM_ERRCODE_INSUFFICIENT_CLIENT_IDENTIFICATION;
	CSSMERR_DL_INSUFFICIENT_CLIENT_IDENTIFICATION = CSSM_DL_BASE_ERROR + CSSM_ERRCODE_INSUFFICIENT_CLIENT_IDENTIFICATION;
	CSSMERR_TP_INSUFFICIENT_CLIENT_IDENTIFICATION = CSSM_TP_BASE_ERROR + CSSM_ERRCODE_INSUFFICIENT_CLIENT_IDENTIFICATION;
	CSSMERR_CSSM_DEVICE_RESET = CSSM_CSSM_BASE_ERROR + CSSM_ERRCODE_DEVICE_RESET;
	CSSMERR_AC_DEVICE_RESET = CSSM_AC_BASE_ERROR + CSSM_ERRCODE_DEVICE_RESET;
	CSSMERR_CSP_DEVICE_RESET = CSSM_CSP_BASE_ERROR + CSSM_ERRCODE_DEVICE_RESET;
	CSSMERR_CL_DEVICE_RESET = CSSM_CL_BASE_ERROR + CSSM_ERRCODE_DEVICE_RESET;
	CSSMERR_DL_DEVICE_RESET = CSSM_DL_BASE_ERROR + CSSM_ERRCODE_DEVICE_RESET;
	CSSMERR_TP_DEVICE_RESET = CSSM_TP_BASE_ERROR + CSSM_ERRCODE_DEVICE_RESET;
	CSSMERR_CSSM_DEVICE_FAILED = CSSM_CSSM_BASE_ERROR + CSSM_ERRCODE_DEVICE_FAILED;
	CSSMERR_AC_DEVICE_FAILED = CSSM_AC_BASE_ERROR + CSSM_ERRCODE_DEVICE_FAILED;
	CSSMERR_CSP_DEVICE_FAILED = CSSM_CSP_BASE_ERROR + CSSM_ERRCODE_DEVICE_FAILED;
	CSSMERR_CL_DEVICE_FAILED = CSSM_CL_BASE_ERROR + CSSM_ERRCODE_DEVICE_FAILED;
	CSSMERR_DL_DEVICE_FAILED = CSSM_DL_BASE_ERROR + CSSM_ERRCODE_DEVICE_FAILED;
	CSSMERR_TP_DEVICE_FAILED = CSSM_TP_BASE_ERROR + CSSM_ERRCODE_DEVICE_FAILED;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSMERR_CSSM_IN_DARK_WAKE = CSSM_CSSM_BASE_ERROR + CSSM_ERRCODE_IN_DARK_WAKE;
	CSSMERR_AC_IN_DARK_WAKE = CSSM_AC_BASE_ERROR + CSSM_ERRCODE_IN_DARK_WAKE;
	CSSMERR_CSP_IN_DARK_WAKE = CSSM_CSP_BASE_ERROR + CSSM_ERRCODE_IN_DARK_WAKE;
	CSSMERR_CL_IN_DARK_WAKE = CSSM_CL_BASE_ERROR + CSSM_ERRCODE_IN_DARK_WAKE;
	CSSMERR_DL_IN_DARK_WAKE = CSSM_DL_BASE_ERROR + CSSM_ERRCODE_IN_DARK_WAKE;
	CSSMERR_TP_IN_DARK_WAKE = CSSM_TP_BASE_ERROR + CSSM_ERRCODE_IN_DARK_WAKE;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ AppleCSPDL, AppleCSP private error codes. }
const
	CSSMERR_CSP_APPLE_ADD_APPLICATION_ACL_SUBJECT = CSSM_CSP_PRIVATE_ERROR + 0;
	{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	 * An attempt was made to use a public key which is incomplete due to
	 * the lack of algorithm-specific parameters.
	 }
	CSSMERR_CSP_APPLE_PUBLIC_KEY_INCOMPLETE = CSSM_CSP_PRIVATE_ERROR + 1;

	{ a code signature match failed }
	CSSMERR_CSP_APPLE_SIGNATURE_MISMATCH = CSSM_CSP_PRIVATE_ERROR + 2;

	{ Key StartDate/EndDate invalid }
	CSSMERR_CSP_APPLE_INVALID_KEY_START_DATE = CSSM_CSP_PRIVATE_ERROR + 3;
	CSSMERR_CSP_APPLE_INVALID_KEY_END_DATE = CSSM_CSP_PRIVATE_ERROR + 4;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	 * An attempt was made to use a public key which is incomplete due to 
	 * the lack of algorithm-specific parameters.
	 }
	CSSMERR_CSP_APPLE_PUBLIC_KEY_INCOMPLETE = CSSM_CSP_PRIVATE_ERROR + 1;
	
	{ a code signature match failed }
	CSSMERR_CSP_APPLE_SIGNATURE_MISMATCH = CSSM_CSP_PRIVATE_ERROR + 2;
	
	{ Key StartDate/EndDate invalid }
	CSSMERR_CSP_APPLE_INVALID_KEY_START_DATE = CSSM_CSP_PRIVATE_ERROR + 3;
	CSSMERR_CSP_APPLE_INVALID_KEY_END_DATE = CSSM_CSP_PRIVATE_ERROR + 4;
	
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
	{ Keychain Syncing error codes }
	CSSMERR_CSPDL_APPLE_DL_CONVERSION_ERROR = CSSM_CSP_PRIVATE_ERROR + 5;

	{ SSLv2 padding check: rollback attack detected }
	CSSMERR_CSP_APPLE_SSLv2_ROLLBACK = CSSM_CSP_PRIVATE_ERROR + 6;


{ AppleFileDL record types. }
const
	CSSM_DL_DB_RECORD_GENERIC_PASSWORD = CSSM_DB_RECORDTYPE_APP_DEFINED_START + 0;
	CSSM_DL_DB_RECORD_INTERNET_PASSWORD = CSSM_DB_RECORDTYPE_APP_DEFINED_START + 1;
	CSSM_DL_DB_RECORD_APPLESHARE_PASSWORD = CSSM_DB_RECORDTYPE_APP_DEFINED_START + 2;
	CSSM_DL_DB_RECORD_X509_CERTIFICATE = CSSM_DB_RECORDTYPE_APP_DEFINED_START + $1000;
	CSSM_DL_DB_RECORD_USER_TRUST = CSSM_DB_RECORDTYPE_APP_DEFINED_START + $1000 + 1;
	CSSM_DL_DB_RECORD_X509_CRL = CSSM_DB_RECORDTYPE_APP_DEFINED_START + $1000 + 2;
	CSSM_DL_DB_RECORD_UNLOCK_REFERRAL = CSSM_DB_RECORDTYPE_APP_DEFINED_START + $1000 + 3;
	CSSM_DL_DB_RECORD_EXTENDED_ATTRIBUTE = CSSM_DB_RECORDTYPE_APP_DEFINED_START + $1000 + 4;
	CSSM_DL_DB_RECORD_METADATA = CSSM_DB_RECORDTYPE_APP_DEFINED_START + $8000;

{ AppleFileDL extentions: passthrough ids }
const
// Toggle whether or not to autocommit after modifying the database.
	// The input parameter is a CSSM_BOOL, where TRUE turns autocommit on
	// and FALSE turns it off.
	CSSM_APPLEFILEDL_TOGGLE_AUTOCOMMIT = 0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	// Commit any pending changes to the database.
	CSSM_APPLEFILEDL_COMMIT = 1;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	
	// Commit any pending changes to the database.
	CSSM_APPLEFILEDL_COMMIT = 1;
	
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
	// Rollback and discard any pending changes to the database.
	CSSM_APPLEFILEDL_ROLLBACK = 2;

{ UNLOCK_REFERRAL "type" attribute values }
const
	CSSM_APPLE_UNLOCK_TYPE_KEY_DIRECT = 1;	// master secret key stored directly
	CSSM_APPLE_UNLOCK_TYPE_WRAPPED_PRIVATE = 2;		// master key wrapped by public key

{ Apple DL private error codes. }
const
{ The OpenParameters argument passed to CSSM_DL_DbCreate or CSSM_DL_DbOpen
	   was neither NULL nor a pointer to a valid CSSM_APPLEDL_OPEN_PARAMETERS
	   structure. }
	CSSMERR_APPLEDL_INVALID_OPEN_PARAMETERS = CSSM_DL_PRIVATE_ERROR + 0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	{ an operation failed because the disk was full }
	CSSMERR_APPLEDL_DISK_FULL = CSSM_DL_PRIVATE_ERROR + 1;

	{ an operation failed because a disk quota was exceeded }
	CSSMERR_APPLEDL_QUOTA_EXCEEDED = CSSM_DL_PRIVATE_ERROR + 2;

	{ an operation failed because a file was too large }
	CSSMERR_APPLEDL_FILE_TOO_BIG = CSSM_DL_PRIVATE_ERROR + 3;

    { a keychain database's internal information ("blob") is invalid }
	CSSMERR_APPLEDL_INVALID_DATABASE_BLOB = CSSM_DL_PRIVATE_ERROR + 4;
	CSSMERR_APPLEDL_INVALID_KEY_BLOB = CSSM_DL_PRIVATE_ERROR + 5;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	
	{ an operation failed because the disk was full }
	CSSMERR_APPLEDL_DISK_FULL = CSSM_DL_PRIVATE_ERROR + 1;
	
	{ an operation failed because a disk quota was exceeded }
	CSSMERR_APPLEDL_QUOTA_EXCEEDED = CSSM_DL_PRIVATE_ERROR + 2;
	
	{ an operation failed because a file was too large }
	CSSMERR_APPLEDL_FILE_TOO_BIG = CSSM_DL_PRIVATE_ERROR + 3;
    
    { a keychain database's internal information ("blob") is invalid }
	CSSMERR_APPLEDL_INVALID_DATABASE_BLOB = CSSM_DL_PRIVATE_ERROR + 4;
	CSSMERR_APPLEDL_INVALID_KEY_BLOB = CSSM_DL_PRIVATE_ERROR + 5;
    
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
    { the internal data format version for a database's internal information ("blob") is invalid }
	CSSMERR_APPLEDL_INCOMPATIBLE_DATABASE_BLOB = CSSM_DL_PRIVATE_ERROR + 6;
	CSSMERR_APPLEDL_INCOMPATIBLE_KEY_BLOB = CSSM_DL_PRIVATE_ERROR + 7;

{ Apple X509TP private error codes. }
const
{ Host name mismatch }
	CSSMERR_APPLETP_HOSTNAME_MISMATCH = CSSM_TP_PRIVATE_ERROR + 0;
	{ Non-understood extension with Critical flag true }
	CSSMERR_APPLETP_UNKNOWN_CRITICAL_EXTEN = CSSM_TP_PRIVATE_ERROR + 1;
	{ Basic Constraints extension required per policy, but not present }
	CSSMERR_APPLETP_NO_BASIC_CONSTRAINTS = CSSM_TP_PRIVATE_ERROR + 2;
	{ Invalid BasicConstraints.CA }
	CSSMERR_APPLETP_INVALID_CA = CSSM_TP_PRIVATE_ERROR + 3;
	{ Invalid Authority Key ID }
	CSSMERR_APPLETP_INVALID_AUTHORITY_ID = CSSM_TP_PRIVATE_ERROR + 4;
	{ Invalid Subject Key ID }
	CSSMERR_APPLETP_INVALID_SUBJECT_ID = CSSM_TP_PRIVATE_ERROR + 5;
	{ Invalid Key Usage for policy }
	CSSMERR_APPLETP_INVALID_KEY_USAGE = CSSM_TP_PRIVATE_ERROR + 6;
	{ Invalid Extended Key Usage for policy }
	CSSMERR_APPLETP_INVALID_EXTENDED_KEY_USAGE = CSSM_TP_PRIVATE_ERROR + 7;
	{ Invalid Subject/Authority Key ID Linkage }
	CSSMERR_APPLETP_INVALID_ID_LINKAGE = CSSM_TP_PRIVATE_ERROR + 8;
	{ PathLengthConstraint exceeded }
	CSSMERR_APPLETP_PATH_LEN_CONSTRAINT = CSSM_TP_PRIVATE_ERROR + 9;
	{ Cert group terminated at a root cert which did not self-verify }
	CSSMERR_APPLETP_INVALID_ROOT = CSSM_TP_PRIVATE_ERROR + 10;
	{ CRL expired/not valid yet }
	CSSMERR_APPLETP_CRL_EXPIRED = CSSM_TP_PRIVATE_ERROR + 11;
	CSSMERR_APPLETP_CRL_NOT_VALID_YET = CSSM_TP_PRIVATE_ERROR + 12;
	{ Cannot find appropriate CRL }
	CSSMERR_APPLETP_CRL_NOT_FOUND = CSSM_TP_PRIVATE_ERROR + 13;
	{ specified CRL server down }
	CSSMERR_APPLETP_CRL_SERVER_DOWN = CSSM_TP_PRIVATE_ERROR + 14;
	{ illegible CRL distribution point URL }
	CSSMERR_APPLETP_CRL_BAD_URI = CSSM_TP_PRIVATE_ERROR + 15;
	{ Unknown critical cert/CRL extension }
	CSSMERR_APPLETP_UNKNOWN_CERT_EXTEN = CSSM_TP_PRIVATE_ERROR + 16;
	CSSMERR_APPLETP_UNKNOWN_CRL_EXTEN = CSSM_TP_PRIVATE_ERROR + 17;
	{ CRL not verifiable to anchor or root }
	CSSMERR_APPLETP_CRL_NOT_TRUSTED = CSSM_TP_PRIVATE_ERROR + 18;
	{ CRL verified to untrusted root }
	CSSMERR_APPLETP_CRL_INVALID_ANCHOR_CERT = CSSM_TP_PRIVATE_ERROR + 19;
	{ CRL failed policy verification }
	CSSMERR_APPLETP_CRL_POLICY_FAIL = CSSM_TP_PRIVATE_ERROR + 20;
	{ IssuingDistributionPoint extension violation }
	CSSMERR_APPLETP_IDP_FAIL = CSSM_TP_PRIVATE_ERROR + 21;
	{ Cert not found at specified issuerAltName }
	CSSMERR_APPLETP_CERT_NOT_FOUND_FROM_ISSUER = CSSM_TP_PRIVATE_ERROR + 22;
	{ Bad cert obtained from specified issuerAltName }
	CSSMERR_APPLETP_BAD_CERT_FROM_ISSUER = CSSM_TP_PRIVATE_ERROR + 23;
	{ S/MIME Email address mismatch }
	CSSMERR_APPLETP_SMIME_EMAIL_ADDRS_NOT_FOUND = CSSM_TP_PRIVATE_ERROR + 24;
	{ Appropriate S/MIME ExtendedKeyUsage not found }
	CSSMERR_APPLETP_SMIME_BAD_EXT_KEY_USE = CSSM_TP_PRIVATE_ERROR + 25;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{ S/MIME KeyUsage incompatibility }
=======
	{ S/MIME KeyUsage incompatiblity }
>>>>>>> graemeg/cpstrnew
=======
	{ S/MIME KeyUsage incompatiblity }
>>>>>>> graemeg/cpstrnew
=======
	{ S/MIME KeyUsage incompatiblity }
>>>>>>> graemeg/cpstrnew
=======
	{ S/MIME KeyUsage incompatiblity }
>>>>>>> origin/cpstrnew
	CSSMERR_APPLETP_SMIME_BAD_KEY_USE = CSSM_TP_PRIVATE_ERROR + 26;
	{ S/MIME, cert with KeyUsage flagged !critical }
	CSSMERR_APPLETP_SMIME_KEYUSAGE_NOT_CRITICAL = CSSM_TP_PRIVATE_ERROR + 27;
	{ S/MIME, leaf with empty subject name and no email addrs
	 * in SubjectAltName }
	CSSMERR_APPLETP_SMIME_NO_EMAIL_ADDRS = CSSM_TP_PRIVATE_ERROR + 28;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{ S/MIME, leaf with empty subject name, SubjectAltName
=======
	{ S/MIME, leaf with empty subject name, SubjectAltName 
>>>>>>> graemeg/cpstrnew
=======
	{ S/MIME, leaf with empty subject name, SubjectAltName 
>>>>>>> graemeg/cpstrnew
=======
	{ S/MIME, leaf with empty subject name, SubjectAltName 
>>>>>>> graemeg/cpstrnew
=======
	{ S/MIME, leaf with empty subject name, SubjectAltName 
>>>>>>> origin/cpstrnew
	 * not critical }
	CSSMERR_APPLETP_SMIME_SUBJ_ALT_NAME_NOT_CRIT = CSSM_TP_PRIVATE_ERROR + 29;
	{ Appropriate SSL ExtendedKeyUsage not found }
	CSSMERR_APPLETP_SSL_BAD_EXT_KEY_USE = CSSM_TP_PRIVATE_ERROR + 30;
	{ unparseable OCSP response }
	CSSMERR_APPLETP_OCSP_BAD_RESPONSE = CSSM_TP_PRIVATE_ERROR + 31;
	{ unparseable OCSP request }
	CSSMERR_APPLETP_OCSP_BAD_REQUEST = CSSM_TP_PRIVATE_ERROR + 32;
	{ OCSP service unavailable }
	CSSMERR_APPLETP_OCSP_UNAVAILABLE = CSSM_TP_PRIVATE_ERROR + 33;
	{ OCSP status: cert unrecognized }
	CSSMERR_APPLETP_OCSP_STATUS_UNRECOGNIZED = CSSM_TP_PRIVATE_ERROR + 34;
	{ revocation check not successful for each cert }
	CSSMERR_APPLETP_INCOMPLETE_REVOCATION_CHECK = CSSM_TP_PRIVATE_ERROR + 35;
	{ general network error }
	CSSMERR_APPLETP_NETWORK_FAILURE = CSSM_TP_PRIVATE_ERROR + 36;
	{ OCSP response not verifiable to anchor or root }
	CSSMERR_APPLETP_OCSP_NOT_TRUSTED = CSSM_TP_PRIVATE_ERROR + 37;
	{ OCSP response verified to untrusted root }
	CSSMERR_APPLETP_OCSP_INVALID_ANCHOR_CERT = CSSM_TP_PRIVATE_ERROR + 38;
	{ OCSP response signature error }
	CSSMERR_APPLETP_OCSP_SIG_ERROR = CSSM_TP_PRIVATE_ERROR + 39;
	{ No signer for OCSP response found }
	CSSMERR_APPLETP_OCSP_NO_SIGNER = CSSM_TP_PRIVATE_ERROR + 40;
	{ OCSP responder status: malformed request }
	CSSMERR_APPLETP_OCSP_RESP_MALFORMED_REQ = CSSM_TP_PRIVATE_ERROR + 41;
	{ OCSP responder status: internal error }
	CSSMERR_APPLETP_OCSP_RESP_INTERNAL_ERR = CSSM_TP_PRIVATE_ERROR + 42;
	{ OCSP responder status: try later }
	CSSMERR_APPLETP_OCSP_RESP_TRY_LATER = CSSM_TP_PRIVATE_ERROR + 43;
	{ OCSP responder status: signature required }
	CSSMERR_APPLETP_OCSP_RESP_SIG_REQUIRED = CSSM_TP_PRIVATE_ERROR + 44;
	{ OCSP responder status: unauthorized }
	CSSMERR_APPLETP_OCSP_RESP_UNAUTHORIZED = CSSM_TP_PRIVATE_ERROR + 45;
	{ OCSP response nonce did not match request }
	CSSMERR_APPLETP_OCSP_NONCE_MISMATCH = CSSM_TP_PRIVATE_ERROR + 46;
	{ Illegal cert chain length for Code Signing  }
	CSSMERR_APPLETP_CS_BAD_CERT_CHAIN_LENGTH = CSSM_TP_PRIVATE_ERROR + 47;
	{ Missing Basic Constraints for Code Signing }
	CSSMERR_APPLETP_CS_NO_BASIC_CONSTRAINTS = CSSM_TP_PRIVATE_ERROR + 48;
	{ Bad PathLengthConstraint for Code Signing }
	CSSMERR_APPLETP_CS_BAD_PATH_LENGTH = CSSM_TP_PRIVATE_ERROR + 49;
	{ Missing ExtendedKeyUsage for Code Signing }
	CSSMERR_APPLETP_CS_NO_EXTENDED_KEY_USAGE = CSSM_TP_PRIVATE_ERROR + 50;
	{ Development style Code Signing Cert Detected }
	CSSMERR_APPLETP_CODE_SIGN_DEVELOPMENT = CSSM_TP_PRIVATE_ERROR + 51;
	{ Illegal cert chain length for Resource Signing  }
	CSSMERR_APPLETP_RS_BAD_CERT_CHAIN_LENGTH = CSSM_TP_PRIVATE_ERROR + 52;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{ Bad extended key usage for Resource Signing }
	CSSMERR_APPLETP_RS_BAD_EXTENDED_KEY_USAGE = CSSM_TP_PRIVATE_ERROR + 53;
	{ Trust Setting: deny }
	CSSMERR_APPLETP_TRUST_SETTING_DENY = CSSM_TP_PRIVATE_ERROR + 54;
	{ Invalid empty SubjectName }
	CSSMERR_APPLETP_INVALID_EMPTY_SUBJECT = CSSM_TP_PRIVATE_ERROR + 55;
	{ Unknown critical Qualified Cert Statement ID }
	CSSMERR_APPLETP_UNKNOWN_QUAL_CERT_STATEMENT = CSSM_TP_PRIVATE_ERROR + 56;
	{ Missing required extension }
	CSSMERR_APPLETP_MISSING_REQUIRED_EXTENSION = CSSM_TP_PRIVATE_ERROR + 57;
	{ Extended key usage not marked critical }
	CSSMERR_APPLETP_EXT_KEYUSAGE_NOT_CRITICAL = CSSM_TP_PRIVATE_ERROR + 58;
	{ Required name or identifier not present }
	CSSMERR_APPLETP_IDENTIFIER_MISSING = CSSM_TP_PRIVATE_ERROR + 59;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{ bad extended key usage for Resource Signing }
	CSSMERR_APPLETP_RS_BAD_EXTENDED_KEY_USAGE = CSSM_TP_PRIVATE_ERROR + 53;
	{ Trust Setting: deny }
	CSSMERR_APPLETP_TRUST_SETTING_DENY = CSSM_TP_PRIVATE_ERROR + 54;
	{ invalid empty SubjectName }
	CSSMERR_APPLETP_INVALID_EMPTY_SUBJECT = CSSM_TP_PRIVATE_ERROR + 55;
	{ unknown critical Qualified Cert Statement ID }
	CSSMERR_APPLETP_UNKNOWN_QUAL_CERT_STATEMENT = CSSM_TP_PRIVATE_ERROR + 56;
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

{ Apple .mac TP private error codes. }
const
{ cert request queued }
	CSSMERR_APPLE_DOTMAC_REQ_QUEUED = CSSM_TP_PRIVATE_ERROR + 100;
	{ cert request redirected }
	CSSMERR_APPLE_DOTMAC_REQ_REDIRECT = CSSM_TP_PRIVATE_ERROR + 101;
	{ general server-reported error }
	CSSMERR_APPLE_DOTMAC_REQ_SERVER_ERR = CSSM_TP_PRIVATE_ERROR + 102;
	{ server-reported parameter error }
	CSSMERR_APPLE_DOTMAC_REQ_SERVER_PARAM = CSSM_TP_PRIVATE_ERROR + 103;
	{ server-reported authorization error }
	CSSMERR_APPLE_DOTMAC_REQ_SERVER_AUTH = CSSM_TP_PRIVATE_ERROR + 104;
	{ server-reported unimplemented }
	CSSMERR_APPLE_DOTMAC_REQ_SERVER_UNIMPL = CSSM_TP_PRIVATE_ERROR + 105;
	{ server-reported not available }
	CSSMERR_APPLE_DOTMAC_REQ_SERVER_NOT_AVAIL = CSSM_TP_PRIVATE_ERROR + 106;
	{ server-reported already exists }
	CSSMERR_APPLE_DOTMAC_REQ_SERVER_ALREADY_EXIST = CSSM_TP_PRIVATE_ERROR + 107;
	{ server-reported service error }
	CSSMERR_APPLE_DOTMAC_REQ_SERVER_SERVICE_ERROR = CSSM_TP_PRIVATE_ERROR + 108;
	{ request already pending for specified user }
	CSSMERR_APPLE_DOTMAC_REQ_IS_PENDING = CSSM_TP_PRIVATE_ERROR + 109;
	{ no request pending for specified user }
	CSSMERR_APPLE_DOTMAC_NO_REQ_PENDING = CSSM_TP_PRIVATE_ERROR + 110;
	{ CSR failed to verify }
	CSSMERR_APPLE_DOTMAC_CSR_VERIFY_FAIL = CSSM_TP_PRIVATE_ERROR + 111;
	{ server reported failed consistency check }
	CSSMERR_APPLE_DOTMAC_FAILED_CONSISTENCY_CHECK = CSSM_TP_PRIVATE_ERROR + 112;

const
	CSSM_APPLEDL_OPEN_PARAMETERS_VERSION = 1;

type
	cssm_appledl_open_parameters_mask = SInt32;
const
	kCSSM_APPLEDL_MASK_MODE = 1 shl 0;

{ Pass a CSSM_APPLEDL_OPEN_PARAMETERS_PTR as the OpenParameters argument to
   CSSM_DL_DbCreate or CSSM_DL_DbOpen.  When using this struct, you must zero
   out the entire struct before setting any additional parameters to ensure
   forward compatibility.  }
type
	CSSM_APPLEDL_OPEN_PARAMETERS_PTR = ^cssm_appledl_open_parameters;
	CSSM_APPLEDL_OPEN_PARAMETERSPtr = ^cssm_appledl_open_parameters;
	cssm_appledl_open_parameters = record
		length: UInt32;	{ Should be sizeof(CSSM_APPLEDL_OPEN_PARAMETERS). }
		version: UInt32;	{ Should be CSSM_APPLEDL_OPEN_PARAMETERS_VERSION. }

	{ If no OpenParameters are specified, autoCommit is on (!CSSM_FALSE) by default.
	   When autoCommit is on (!CSSM_FALSE), changes made to the Db are written to disk
	   before returning from each function.
	   When autoCommit is off (CSSM_FALSE), changes made to the database are not guaranteed
	   to be written to disk until the Db is closed.  This is useful for bulk writes.
	   Be aware that if autoCommit is off, changes made in previous calls to the DL might
	   get rolled back if a new modification operation fails. }
		autoCommit: CSSM_BOOL;

	{ Mask marking which of the following fields are to be used. }
		mask: UInt32;

	{ When calling DbCreate, the initial mode to create the database file with; ignored on DbOpen.  You must set the kCSSM_APPLEDL_MASK_MODE bit in mask or mode is ignored.  }
		mode: mode_t;
	end;


{ AppleCSPDL passthough ids }
const
{ Tell the SecurityServer to lock the database specified by the DLDBHandle argument.
	   The InputParams and OutputParams arguments are ignored. }
	CSSM_APPLECSPDL_DB_LOCK = 0;

	{ Tell the SecurityServer to unlock the database specified by the DLDBHandle argument.
	   The InputParameters argument is a CSSM_DATA_PTR containing the password, or NULL if
	   the SecurityServer should prompt for the password.
	   The OutputParams argument is ignored.
	   The SecurityServer will put up UI (though the SecurityAgent) when this function is called
	   iff InputParameters is NULL.  }
	CSSM_APPLECSPDL_DB_UNLOCK = 1;

	{ Ask the SecurityServer to get the db settings specified for the database
	   specified by the DLDBHandle argument.  The settings are returned in the OutputParameters argument.
	   The OutputParameters argument is a pointer to a CSSM_APPLECSPDL_DB_SETTINGS_PARAMETERS_PTR.
	   Upon successful completion, the AppleCSPDL will have allocated a
	   CSSM_APPLECSPDL_DB_SETTINGS_PARAMETERS structure using the application-specified
	   allocators for the DL attachment specified by the DLDBHandle argument.  The structure will contain
	   the current database settings for the specified database.  The client should free the
	   CSSM_APPLECSPDL_DB_SETTINGS_PARAMETERS_PTR after it has finished using it.
	   The InputParameters argument is ignored.
	   The SecurityServer might put up UI (though the SecurityAgent) when this function is called.  }
	CSSM_APPLECSPDL_DB_GET_SETTINGS = 2;

	{ Tell the SecurityServer to set the db settings specified in InputParameters on the database
	   specified by the DLDBHandle argument.
	   The InputParameters argument is a const CSSM_APPLECSPDL_DB_SETTINGS_PARAMETERS * containing
	   the new settings for the specified database.
	   The OutputParams argument is ignored.
	   The SecurityServer might put up UI (though the SecurityAgent) when this function is called.  }
	CSSM_APPLECSPDL_DB_SET_SETTINGS = 3;

	{ Ask the SecurityServer whether the database specified by the DLDBHandle argument is locked.
	   The InputParameters argument is ignored.
	   The OutputParameters argument is a pointer to a CSSM_APPLECSPDL_DB_IS_LOCKED_PARAMETERS_PTR.
	   Upon successful completion, the AppleCSPDL will have allocated a
	   CSSM_APPLECSPDL_DB_IS_LOCKED_PARAMETERS structure using the application-specified
	   allocators for the DL attachment specified by the DLDBHandle argument.  The structure will contain
	   the current lock status for the specified database.  The client should free the
	   CSSM_APPLECSPDL_DB_IS_LOCKED_PARAMETERS_PTR after it has finished using it.
	   The SecurityServer will put up UI (though the SecurityAgent) when this function is called. }
	CSSM_APPLECSPDL_DB_IS_LOCKED = 4;

	{ Tell the SecurityServer to change the password for the database specified by
	   the DLDBHandle.

	   The InputParameters argument is a const CSSM_APPLECSPDL_DB_CHANGE_PASSWORD_PARAMETERS * containing
	   a CSSM_ACCESS_CREDENTIALS * which determines how the password will be changed.  If the
	   accessCredentials are NULL, the SecurityAgent will prompt for the old and the new password for the
	   specified database.  If credentials are specified, there should be 2 entries:
	   1. a 3-element list containing:
	   CSSM_WORDID_KEYCHAIN_LOCK, CSSM_SAMPLE_TYPE_PASSWORD, and the old password.
	   2. a 3-element list containing:
	   CSSM_WORDID_KEYCHAIN_CHANGE_LOCK, CSSM_SAMPLE_TYPE_PASSWORD, and the new password.

	   The OutputParams argument is ignored.
	   The SecurityServer might put up UI (though the SecurityAgent) when this function is called.  }
	CSSM_APPLECSPDL_DB_CHANGE_PASSWORD = 5;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	{ Return the SecurityServer database handle for the database specified by the DLDBHandle }
	CSSM_APPLECSPDL_DB_GET_HANDLE = 6;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	
	{ Return the SecurityServer database handle for the database specified by the DLDBHandle }
	CSSM_APPLECSPDL_DB_GET_HANDLE = 6;
	
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
	{ Given a CSSM_KEY for the CSPDL, return the SecurityServer key handle }
	CSSM_APPLESCPDL_CSP_GET_KEYHANDLE = 7;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_8 = 8;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_9 = 9;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_10 = 10;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_11 = 11;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_12 = 12;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_13 = 13;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_14 = 14;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_15 = 15;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_16 = 16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_APPLE_PRIVATE_CSPDL_CODE_17 = 17;
	CSSM_APPLE_PRIVATE_CSPDL_CODE_18 = 18;

	{ Given a CSSM_KEY_PTR in any format, obtain the SHA-1 hash of the
	 * associated key blob.
=======
	
	{ Given a CSSM_KEY_PTR in any format, obtain the SHA-1 hash of the 
	 * associated key blob. 
>>>>>>> graemeg/cpstrnew
=======
	
	{ Given a CSSM_KEY_PTR in any format, obtain the SHA-1 hash of the 
	 * associated key blob. 
>>>>>>> graemeg/cpstrnew
=======
	
	{ Given a CSSM_KEY_PTR in any format, obtain the SHA-1 hash of the 
	 * associated key blob. 
>>>>>>> graemeg/cpstrnew
=======
	
	{ Given a CSSM_KEY_PTR in any format, obtain the SHA-1 hash of the 
	 * associated key blob. 
>>>>>>> origin/cpstrnew
	 * Key is specified in CSSM_CSP_CreatePassThroughContext.
	 * Hash is allocated bythe CSP, in the App's memory, and returned
	 * in *outData. }
	CSSM_APPLECSP_KEYDIGEST = $100;


{ AppleCSPDL passthough parameters }
type
	CSSM_APPLECSPDL_DB_SETTINGS_PARAMETERS_PTR = ^cssm_applecspdl_db_settings_parameters;
	CSSM_APPLECSPDL_DB_SETTINGS_PARAMETERSPtr = ^cssm_applecspdl_db_settings_parameters;
	cssm_applecspdl_db_settings_parameters = record
		idleTimeout: UInt32;				// seconds idle timeout lock
		lockOnSleep: UInt8;				// lock database when system sleeps
	end;

{ AppleCSPDL passthough parameters }
type
	CSSM_APPLECSPDL_DB_IS_LOCKED_PARAMETERS_PTR = ^cssm_applecspdl_db_is_locked_parameters;
	CSSM_APPLECSPDL_DB_IS_LOCKED_PARAMETERSPtr = ^cssm_applecspdl_db_is_locked_parameters;
	cssm_applecspdl_db_is_locked_parameters = record
		isLocked: UInt8;				// True iff the database is locked
	end;

{ AppleCSPDL passthough parameters }
type
	CSSM_APPLECSPDL_DB_CHANGE_PASSWORD_PARAMETERS_PTR = ^cssm_applecspdl_db_change_password_parameters;
	CSSM_APPLECSPDL_DB_CHANGE_PASSWORD_PARAMETERSPtr = ^cssm_applecspdl_db_change_password_parameters;
	cssm_applecspdl_db_change_password_parameters = record
		accessCredentials: CSSM_ACCESS_CREDENTIALSPtr;
	end;

{ Custom wrapped key formats }
const
	CSSM_KEYBLOB_WRAPPED_FORMAT_APPLE_CUSTOM = 100;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_KEYBLOB_WRAPPED_FORMAT_OPENSSL = 101;			// traditional openssl
=======
	CSSM_KEYBLOB_WRAPPED_FORMAT_OPENSSL = 101;			// traditional openssl 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_KEYBLOB_WRAPPED_FORMAT_OPENSSL = 101;			// traditional openssl 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_KEYBLOB_WRAPPED_FORMAT_OPENSSL = 101;			// traditional openssl 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_KEYBLOB_WRAPPED_FORMAT_OPENSSL = 101;			// traditional openssl 
>>>>>>> origin/cpstrnew
	CSSM_KEYBLOB_WRAPPED_FORMAT_OPENSSH1 = 102;			// OpenSSH v1

{
 * Custom context attributes for AppleCSP.
 }
const
	CSSM_ATTRIBUTE_VENDOR_DEFINED = $800000;

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
	 * Public Key attribute for use with CSSM_ALGID_FEED.
	 }
	CSSM_ATTRIBUTE_PUBLIC_KEY = (CSSM_ATTRIBUTE_DATA_KEY or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 0));

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ 
	 * Public Key attribute for use with CSSM_ALGID_FEED.
	 }
	CSSM_ATTRIBUTE_PUBLIC_KEY = (CSSM_ATTRIBUTE_DATA_KEY or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 0));
			
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
	 * FEE key attributes.
	 * See CSSM_FEE_PRIME_TYPE_xxx, CSSM_FEE_CURVE_TYPE_xxx enums, below.
	 }
	CSSM_ATTRIBUTE_FEE_PRIME_TYPE = (CSSM_ATTRIBUTE_DATA_UINT32 or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 1));
	CSSM_ATTRIBUTE_FEE_CURVE_TYPE = (CSSM_ATTRIBUTE_DATA_UINT32 or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 2));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> origin/cpstrnew
	{
	 * Apple Secure Compression (ComCryption) optimization.
	 * See CSSM_ASC_OPTIMIZE_xxx, enums, below.
	 }
	CSSM_ATTRIBUTE_ASC_OPTIMIZATION = (CSSM_ATTRIBUTE_DATA_UINT32 or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 3));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> origin/cpstrnew
	{
	 * RSA blinding. Value is integer, nonzero (blinding on) or zero.
	 }
	CSSM_ATTRIBUTE_RSA_BLINDING = (CSSM_ATTRIBUTE_DATA_UINT32 or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 4));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> origin/cpstrnew
	{
	 * Additional public key from which to obtain algorithm-specific
	 * parameters.
	 }
	CSSM_ATTRIBUTE_PARAM_KEY = (CSSM_ATTRIBUTE_DATA_KEY or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 5));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> graemeg/cpstrnew
=======
			
>>>>>>> origin/cpstrnew
	{
	 * Prompt string for CSSM_ALGID_SECURE_PASSPHRASE key acquisition.
	 * Data is a UTF8-encoded external representation of a CFString. 
	 }
	CSSM_ATTRIBUTE_PROMPT = (CSSM_ATTRIBUTE_DATA_CSSM_DATA or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 6));

	{
	 * Alert panel title for CSSM_ALGID_SECURE_PASSPHRASE key acquisition.
	 * Data is a UTF8-encoded external representation of a CFString. 
	 }
	CSSM_ATTRIBUTE_ALERT_TITLE = (CSSM_ATTRIBUTE_DATA_CSSM_DATA or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 7));

	{
	 * Boolean to specify whether secure passphrase is being used to encrypt or to 
	 * recover data. In the former case the user will be prompted to enter the 
	 * passphrase twice. Value is integer, nonzero (verify passphrase) or zero.
	 }
	CSSM_ATTRIBUTE_VERIFY_PASSPHRASE = (CSSM_ATTRIBUTE_DATA_UINT32 or (CSSM_ATTRIBUTE_VENDOR_DEFINED + 8));

{
 * FEE key pair prime modulus types.
 }
const
	CSSM_FEE_PRIME_TYPE_DEFAULT = 0;	{ default per key size }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_FEE_PRIME_TYPE_MERSENNE = 1;		{ (2 ** q) - 1 }
=======
	CSSM_FEE_PRIME_TYPE_MERSENNE = 1;		{ (2 ** q) - 1Ê}
>>>>>>> graemeg/cpstrnew
=======
	CSSM_FEE_PRIME_TYPE_MERSENNE = 1;		{ (2 ** q) - 1Ê}
>>>>>>> graemeg/cpstrnew
=======
	CSSM_FEE_PRIME_TYPE_MERSENNE = 1;		{ (2 ** q) - 1Ê}
>>>>>>> graemeg/cpstrnew
=======
	CSSM_FEE_PRIME_TYPE_MERSENNE = 1;		{ (2 ** q) - 1Ê}
>>>>>>> origin/cpstrnew
	CSSM_FEE_PRIME_TYPE_FEE = 2;			{ (2 ** q) - k }
	CSSM_FEE_PRIME_TYPE_GENERAL = 3;			{ random prime }

{
 * FEE curve types. Comments refer to equation
 *
 *    y**2 = x**3 + c(x**2) + ax + b
 }
const
	CSSM_FEE_CURVE_TYPE_DEFAULT = 0;	{ default per key size }
	CSSM_FEE_CURVE_TYPE_MONTGOMERY = 1;		{ a==1, b==0 }
	CSSM_FEE_CURVE_TYPE_WEIERSTRASS = 2;	{ c==0. IEEE P1363 compliant. }
	CSSM_FEE_CURVE_TYPE_ANSI_X9_62 = 3;		{ ANSI X9.62 compatible }

{
 * Apple Secure Compression (ComCryption) optimization attributes.
 }
const
	CSSM_ASC_OPTIMIZE_DEFAULT = 0;
	CSSM_ASC_OPTIMIZE_SIZE = 1;				{ max compression (currently the default) }
	CSSM_ASC_OPTIMIZE_SECURITY = 2;			{ currently not implemented }
	CSSM_ASC_OPTIMIZE_TIME = 3;				{ min runtime }
	CSSM_ASC_OPTIMIZE_TIME_SIZE = 4;		{ implies loss of security }
	CSSM_ASC_OPTIMIZE_ASCII = 5;			{ optimized for ASCC text, not implemented }

{
 * Apple custom CSSM_KEYATTR_FLAGS.
 }
const
{
	 * When set, indicates a public key which is incomplete (though
	 * still valid) due to the lack of algorithm-specific parameters.
	 }
	CSSM_KEYATTR_PARTIAL = $00010000;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> origin/cpstrnew
	{
	 * When set, public keys are stored encrypted. Default is to store
	 * public keys in the clear. AppleCSPDL only.
	 }
	CSSM_KEYATTR_PUBLIC_KEY_ENCRYPT = $00020000;

{
 * Name/OID pair used in CSSM_APPLE_TP_CERT_REQUEST
 }
type
	CSSM_APPLE_TP_NAME_OIDPtr = ^CSSM_APPLE_TP_NAME_OID;
	CSSM_APPLE_TP_NAME_OID = record
		strng: {const} CStringPtr;
		oid: {const} CSSM_OIDPtr;
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
 * Certificate request passed to CSSM_TP_SubmitCredRequest() in the
 * CSSM_TP_AUTHORITY_REQUEST_TYPE.Requests field. Used for requesting
 * both locally-generated certs (CSSMOID_APPLE_TP_LOCAL_CERT_GEN) and
 * cert signing requests (CSSMOID_APPLE_TP_CSR_GEN).
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ 
 * Certificate request passed to CSSM_TP_SubmitCredRequest() in the
 * CSSM_TP_AUTHORITY_REQUEST_TYPE.Requests field. Used for requesting
 * both locally-generated certs (CSSMOID_APPLE_TP_LOCAL_CERT_GEN) and
 * cert signing requests (CSSMOID_APPLE_TP_CSR_GEN). 
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
 }
type
	CSSM_APPLE_TP_CERT_REQUESTPtr = ^CSSM_APPLE_TP_CERT_REQUEST;
	CSSM_APPLE_TP_CERT_REQUEST = record
		cspHand: CSSM_CSP_HANDLE;		// sign with this CSP
		clHand: CSSM_CL_HANDLE;			// and this CL
		serialNumber: UInt32;
		numSubjectNames: UInt32;// size subjectNames[]
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		subjectNames: CSSM_APPLE_TP_NAME_OIDPtr;

	{
	 * Issuer name can be expressed in the simplified CSSM_APPLE_TP_NAME_OID
	 * array, as is the subject name, or as an CSSM_X509_NAME, which is
	 * typically obtained from a signing cert.
	 * Exactly one of (issuerNames, issuerNameX509) must be non-NULL.
	 }
		numIssuerNames: UInt32;	// size issuerNames[]
		issuerNames: CSSM_APPLE_TP_NAME_OIDPtr;   // optional; NULL implies root
											//    (signer == subject)
		issuerNameX509: CSSM_X509_NAME_PTR;
		certPublicKey: {const} CSSM_KEYPtr;
		issuerPrivateKey: {const} CSSM_KEYPtr;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
		subjectNames: CSSM_APPLE_TP_NAME_OIDPtr;	
	
	{
	 * Issuer name can be expressed in the simplified CSSM_APPLE_TP_NAME_OID
	 * array, as is the subject name, or as an CSSM_X509_NAME, which is 
	 * typically obtained from a signing cert. 
	 * Exactly one of (issuerNames, issuerNameX509) must be non-NULL. 
	 }
		numIssuerNames: UInt32;	// size issuerNames[]
		issuerNames: CSSM_APPLE_TP_NAME_OIDPtr;   // optional; NULL implies root 
											//    (signer == subject)
		issuerNameX509: CSSM_X509_NAME_PTR;		
		certPublicKey: {const} CSSM_KEYPtr;
		issuerPrivateKey: {const} CSSM_KEYPtr;
	
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
	{ Unfortunately there is no practical way to map any algorithm
	 * to its appropriate OID, and we need both.... }
		signatureAlg: CSSM_ALGORITHMS;   // e.g., CSSM_ALGID_SHA1WithRSA
		signatureOid: CSSM_OID;	// e.g., CSSMOID_SHA1WithRSA
		notBefore: UInt32;		// relative to "now"
		notAfter: UInt32;
		numExtensions: UInt32;
		extensions: CE_DataAndTypePtr;	// optional
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	{
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> origin/cpstrnew
	 * Optional challenge string for CSSMOID_APPLE_TP_CSR_GEN.
	 }
		challengeString: {const} CStringPtr;
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy CSSMOID_APPLE_TP_SSL.
 * A pointer to, and length of, one of these is optionally placed in
=======
{ 
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy CSSMOID_APPLE_TP_SSL. 
 * A pointer to, and length of, one of these is optionally placed in 
>>>>>>> graemeg/cpstrnew
=======
{ 
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy CSSMOID_APPLE_TP_SSL. 
 * A pointer to, and length of, one of these is optionally placed in 
>>>>>>> graemeg/cpstrnew
=======
{ 
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy CSSMOID_APPLE_TP_SSL. 
 * A pointer to, and length of, one of these is optionally placed in 
>>>>>>> graemeg/cpstrnew
=======
{ 
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy CSSMOID_APPLE_TP_SSL. 
 * A pointer to, and length of, one of these is optionally placed in 
>>>>>>> origin/cpstrnew
 * CSSM_TP_VERIFY_CONTEXT.Cred->Policy.PolicyIds[n].FieldValue.
 }
const
	CSSM_APPLE_TP_SSL_OPTS_VERSION = 1;

{
 * Values for CSSM_APPLE_TP_SSL_OPTIONS.flags.
 *
 * Set this flag when evaluating a client cert.
 }
const
	CSSM_APPLE_TP_SSL_CLIENT = $00000001;

type
	CSSM_APPLE_TP_SSL_OPTIONSPtr = ^CSSM_APPLE_TP_SSL_OPTIONS;
	CSSM_APPLE_TP_SSL_OPTIONS = record
		Version: UInt32;        // CSSM_APPLE_TP_SSL_OPTS_VERSION

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{
	 * The domain name of the server (e.g., "store.apple.com".) In the
	 * SSL and TLS protocols, this must match the common name of the
=======
	{ 
	 * The domain name of the server (e.g., "store.apple.com".) In the 
	 * SSL and TLS protocols, this must match the common name of the 
>>>>>>> graemeg/cpstrnew
=======
	{ 
	 * The domain name of the server (e.g., "store.apple.com".) In the 
	 * SSL and TLS protocols, this must match the common name of the 
>>>>>>> graemeg/cpstrnew
=======
	{ 
	 * The domain name of the server (e.g., "store.apple.com".) In the 
	 * SSL and TLS protocols, this must match the common name of the 
>>>>>>> graemeg/cpstrnew
=======
	{ 
	 * The domain name of the server (e.g., "store.apple.com".) In the 
	 * SSL and TLS protocols, this must match the common name of the 
>>>>>>> origin/cpstrnew
	 * subject cert. Expressed as a C string, optionally NULL terminated
	 * if it is NULL terminated, the length field should include the NULL).
	 }
		ServerNameLen: UInt32;
		ServerName: {const} CStringPtr;    // optional
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> origin/cpstrnew
	{ new fields for struct version 1 }
		Flags: UInt32;
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy
 * CSSMOID_APPLE_TP_REVOCATION_CRL. A pointer to, and length of, one
 * of these is optionally placed in
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ 
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy 
 * CSSMOID_APPLE_TP_REVOCATION_CRL. A pointer to, and length of, one 
 * of these is optionally placed in 
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
 * CSSM_TP_VERIFY_CONTEXT.Cred->Policy.PolicyIds[n].FieldValue.
 }
const
	CSSM_APPLE_TP_CRL_OPTS_VERSION = 0;

type
	CSSM_APPLE_TP_CRL_OPT_FLAGS = UInt32;
const
// require CRL verification for each cert; default is "try"
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_TP_ACTION_REQUIRE_CRL_PER_CERT = $00000001;
=======
	CSSM_TP_ACTION_REQUIRE_CRL_PER_CERT = $00000001;	
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_REQUIRE_CRL_PER_CERT = $00000001;	
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_REQUIRE_CRL_PER_CERT = $00000001;	
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_REQUIRE_CRL_PER_CERT = $00000001;	
>>>>>>> origin/cpstrnew
	// enable fetch from network
	CSSM_TP_ACTION_FETCH_CRL_FROM_NET = $00000002;
	// if set and positive OCSP verify for given cert, no further revocation
	// checking need be done on that cert
	CSSM_TP_ACTION_CRL_SUFFICIENT = $00000004;
	// require CRL verification for certs which claim a CRL provider
	CSSM_TP_ACTION_REQUIRE_CRL_IF_PRESENT = $00000008;

type
	CSSM_APPLE_TP_CRL_OPTIONSPtr = ^CSSM_APPLE_TP_CRL_OPTIONS;
	CSSM_APPLE_TP_CRL_OPTIONS = record
		Version: UInt32;        // CSSM_APPLE_TP_CRL_OPTS_VERSION
		CrlFlags: CSSM_APPLE_TP_CRL_OPT_FLAGS;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	{
	 * When non-NULL, store CRLs fetched from net here.
	 * This is most likely a pointer to one of the
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	
	{
	 * When non-NULL, store CRLs fetched from net here.
	 * This is most likely a pointer to one of the  
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
	 * CSSM_TP_CALLERAUTH_CONTEXT.DBList entries but that
	 * is not a strict requirement.
	 }
		crlStore: CSSM_DL_DB_HANDLE_PTR;
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy
 * CSSMOID_APPLE_TP_SMIME. A pointer to, and length of, one
 * of these is optionally placed in
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ 
 * Options for X509TP's CSSM_TP_CertGroupVerify for policy 
 * CSSMOID_APPLE_TP_SMIME. A pointer to, and length of, one 
 * of these is optionally placed in 
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
 * CSSM_TP_VERIFY_CONTEXT.Cred->Policy.PolicyIds[n].FieldValue.
 }
const
	CSSM_APPLE_TP_SMIME_OPTS_VERSION = 0;
type
	CSSM_APPLE_TP_SMIME_OPTIONSPtr = ^CSSM_APPLE_TP_SMIME_OPTIONS;
	CSSM_APPLE_TP_SMIME_OPTIONS = record
		Version: UInt32;        // CSSM_APPLE_TP_SMIME_OPTS_VERSION

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{
=======
	{ 
>>>>>>> graemeg/cpstrnew
=======
	{ 
>>>>>>> graemeg/cpstrnew
=======
	{ 
>>>>>>> graemeg/cpstrnew
=======
	{ 
>>>>>>> origin/cpstrnew
	 * Intended usage of the leaf cert. The cert's KeyUsage extension,
	 * if present, must be a superset of this.
	 }
		IntendedUsage: CE_KeyUsage;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	{
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> origin/cpstrnew
	 * The email address of the sender. If there is an email address
	 * in the sender's cert, that email address must match this one.
	 * Both (email address in the cert, and this one) are optional.
	 * Expressed as a C string, optionally NULL terminated (i.e.,
	 * SenderEmail[SenderEmailLen - 1] may or may not be NULL).
	 }
		SenderEmailLen: UInt32;
		SenderEmail: {const} CStringPtr;    // optional
	end;


{
 * Optional ActionData for all X509TP CertGroupVerify policies.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * A pointer to, and length of, one of these is optionally placed in
=======
 * A pointer to, and length of, one of these is optionally placed in 
>>>>>>> graemeg/cpstrnew
=======
 * A pointer to, and length of, one of these is optionally placed in 
>>>>>>> graemeg/cpstrnew
=======
 * A pointer to, and length of, one of these is optionally placed in 
>>>>>>> graemeg/cpstrnew
=======
 * A pointer to, and length of, one of these is optionally placed in 
>>>>>>> origin/cpstrnew
 * CSSM_TP_VERIFY_CONTEXT.ActionData.
 }
type
	CSSM_APPLE_TP_ACTION_FLAGS = UInt32;
const
	CSSM_TP_ACTION_ALLOW_EXPIRED = $00000001;	// allow expired certs
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_TP_ACTION_LEAF_IS_CA = $00000002;	// first cert is a CA
=======
	CSSM_TP_ACTION_LEAF_IS_CA = $00000002;	// first cert is a CA 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_LEAF_IS_CA = $00000002;	// first cert is a CA 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_LEAF_IS_CA = $00000002;	// first cert is a CA 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_LEAF_IS_CA = $00000002;	// first cert is a CA 
>>>>>>> origin/cpstrnew
	CSSM_TP_ACTION_FETCH_CERT_FROM_NET = $00000004;	// enable net fetch of CA cert
	CSSM_TP_ACTION_ALLOW_EXPIRED_ROOT = $00000008; 	// allow expired roots
	CSSM_TP_ACTION_REQUIRE_REV_PER_CERT = $00000010; 	// require positive revocation
														//   check per cert
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CSSM_TP_ACTION_TRUST_SETTINGS = $00000020;	// use TrustSettings instead of
=======
	CSSM_TP_ACTION_TRUST_SETTINGS = $00000020;	// use TrustSettings instead of 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_TRUST_SETTINGS = $00000020;	// use TrustSettings instead of 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_TRUST_SETTINGS = $00000020;	// use TrustSettings instead of 
>>>>>>> graemeg/cpstrnew
=======
	CSSM_TP_ACTION_TRUST_SETTINGS = $00000020;	// use TrustSettings instead of 
>>>>>>> origin/cpstrnew
														//   anchors
	CSSM_TP_ACTION_IMPLICIT_ANCHORS = $00000040;	// properly self-signed certs are
														//   treated as anchors implicitly

const
	CSSM_APPLE_TP_ACTION_VERSION = 0;
type
	CSSM_APPLE_TP_ACTION_DATAPtr = ^CSSM_APPLE_TP_ACTION_DATA;
	CSSM_APPLE_TP_ACTION_DATA = record
		Version: UInt32; 		// CSSM_APPLE_TP_ACTION_VERSION
		ActionFlags: CSSM_APPLE_TP_ACTION_FLAGS;	// CSSM_TP_ACTION_ALLOW_EXPIRED, etc.
	end;

{
 * Per-cert evidence returned from CSSM_TP_CertGroupVerify.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * An array of these is presented in CSSM_TP_VERIFY_CONTEXT_RESULT.Evidence[2].
 * Same number of these as in the cert group in Evidence[1].
 }

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 * An array of these is presented in CSSM_TP_VERIFY_CONTEXT_RESULT.Evidence[2]. 
 * Same number of these as in the cert group in Evidence[1].
 }
 
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
{ First, an array of bits indicating various status of the cert. }
type
	CSSM_TP_APPLE_CERT_STATUS = UInt32;
const
	CSSM_CERT_STATUS_EXPIRED = $00000001;
	CSSM_CERT_STATUS_NOT_VALID_YET = $00000002;
	CSSM_CERT_STATUS_IS_IN_INPUT_CERTS = $00000004;
	CSSM_CERT_STATUS_IS_IN_ANCHORS = $00000008;
	CSSM_CERT_STATUS_IS_ROOT = $00000010;
	CSSM_CERT_STATUS_IS_FROM_NET = $00000020;
	{ settings found in per-user Trust Settings }
	CSSM_CERT_STATUS_TRUST_SETTINGS_FOUND_USER = $00000040;
	{ settings found in Admin Trust Settings }
	CSSM_CERT_STATUS_TRUST_SETTINGS_FOUND_ADMIN = $00000080;
	{ settings found in System Trust Settings }
	CSSM_CERT_STATUS_TRUST_SETTINGS_FOUND_SYSTEM = $00000100;
	{ Trust Settings result = Trust }
	CSSM_CERT_STATUS_TRUST_SETTINGS_TRUST = $00000200;
	{ Trust Settings result = Deny }
	CSSM_CERT_STATUS_TRUST_SETTINGS_DENY = $00000400;
	{ Per-cert error ignored due to Trust Settings }
	CSSM_CERT_STATUS_TRUST_SETTINGS_IGNORED_ERROR = $00000800;

type
	CSSM_TP_APPLE_EVIDENCE_INFOArrayPtr = ^CSSM_TP_APPLE_EVIDENCE_INFOPtr;
	CSSM_TP_APPLE_EVIDENCE_INFOPtr = ^CSSM_TP_APPLE_EVIDENCE_INFO;
	CSSM_TP_APPLE_EVIDENCE_INFO = record
		StatusBits: CSSM_TP_APPLE_CERT_STATUS;
		NumStatusCodes: UInt32;
		StatusCodes: CSSM_RETURNPtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	{ index into raw cert group or AnchorCerts depending on IS_IN_ANCHORS }
		Index: UInt32;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	
	{ index into raw cert group or AnchorCerts depending on IS_IN_ANCHORS }
		Index: UInt32;   
	
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
	{ nonzero if cert came from a DLDB }
		DlDbHandle: CSSM_DL_DB_HANDLE;
		UniqueRecord: CSSM_DB_UNIQUE_RECORD_PTR;
	end;

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * CSSM_TP_VERIFY_CONTEXT_RESULT.Evidence[0], basically defines which version/flavor
=======
 * CSSM_TP_VERIFY_CONTEXT_RESULT.Evidence[0], basically defines which version/flavor 
>>>>>>> graemeg/cpstrnew
=======
 * CSSM_TP_VERIFY_CONTEXT_RESULT.Evidence[0], basically defines which version/flavor 
>>>>>>> graemeg/cpstrnew
=======
 * CSSM_TP_VERIFY_CONTEXT_RESULT.Evidence[0], basically defines which version/flavor 
>>>>>>> graemeg/cpstrnew
=======
 * CSSM_TP_VERIFY_CONTEXT_RESULT.Evidence[0], basically defines which version/flavor 
>>>>>>> origin/cpstrnew
 * of remaining evidence is.
 }
const
	CSSM_TP_APPLE_EVIDENCE_VERSION = 0;
type
	CSSM_TP_APPLE_EVIDENCE_HEADERPtr = ^CSSM_TP_APPLE_EVIDENCE_HEADER;
	CSSM_TP_APPLE_EVIDENCE_HEADER = record
		Version: UInt32;
	end;


{
 * Apple-specific CSSM_EVIDENCE_FORM values
 *
 * The form of the evidence returns from CSSM_TP_CertGroupVerify is:
 *
 * EvidenceForm							contents of *Evidence
 * ------------  						---------------------
 * CSSM_EVIDENCE_FORM_APPLE_HEADER		CSSM_TP_APPLE_EVIDENCE_HEADER
 * CSSM_EVIDENCE_FORM_APPLE_CERTGROUP	CSSM_CERTGROUP
 * CSSM_EVIDENCE_FORM_APPLE_CERT_INFO	array of CSSM_TP_APPLE_EVIDENCE_INFO, size
 *											CSSM_CERTGROUP.NumCerts
 }

const
	CSSM_EVIDENCE_FORM_APPLE_CUSTOM = $80000000;
const
	CSSM_EVIDENCE_FORM_APPLE_HEADER = CSSM_EVIDENCE_FORM_APPLE_CUSTOM + 0;
	CSSM_EVIDENCE_FORM_APPLE_CERTGROUP = CSSM_EVIDENCE_FORM_APPLE_CUSTOM + 1;
	CSSM_EVIDENCE_FORM_APPLE_CERT_INFO = CSSM_EVIDENCE_FORM_APPLE_CUSTOM + 2;

{ AppleX509CL extensions: passthrough ids }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
=======
{ 
>>>>>>> graemeg/cpstrnew
=======
{ 
>>>>>>> graemeg/cpstrnew
=======
{ 
>>>>>>> graemeg/cpstrnew
=======
{ 
>>>>>>> origin/cpstrnew
	 * Obtain a signed Certificate Signing Request.
	 * Input = CSSM_APPLE_CL_CSR_REQUEST
	 * Output = allocated CSSM_DATA which points to a DER-encoded CSR.
	 }
	CSSM_APPLEX509CL_OBTAIN_CSR = 0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> graemeg/cpstrnew
=======
	
>>>>>>> origin/cpstrnew
	{
	 * Perform signature verify of a CSR.
	 * Input:  CSSM_DATA referring to a DER-encoded CSR.
	 * Output: Nothing, returns CSSMERR_CL_VERIFICATION_FAILURE on
	 *         on failure.
	 }
	CSSM_APPLEX509CL_VERIFY_CSR = 1;

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Used in CL's CSSM_APPLEX509_OBTAIN_CSR Passthrough. This is the
 * input; the output is a CSSM_DATA * containing the signed and
=======
 * Used in CL's CSSM_APPLEX509_OBTAIN_CSR Passthrough. This is the 
 * input; the output is a CSSM_DATA * containing the signed and 
>>>>>>> graemeg/cpstrnew
=======
 * Used in CL's CSSM_APPLEX509_OBTAIN_CSR Passthrough. This is the 
 * input; the output is a CSSM_DATA * containing the signed and 
>>>>>>> graemeg/cpstrnew
=======
 * Used in CL's CSSM_APPLEX509_OBTAIN_CSR Passthrough. This is the 
 * input; the output is a CSSM_DATA * containing the signed and 
>>>>>>> graemeg/cpstrnew
=======
 * Used in CL's CSSM_APPLEX509_OBTAIN_CSR Passthrough. This is the 
 * input; the output is a CSSM_DATA * containing the signed and 
>>>>>>> origin/cpstrnew
 * DER-encoded CSR.
 }
type
	CSSM_APPLE_CL_CSR_REQUESTPtr = ^CSSM_APPLE_CL_CSR_REQUEST;
	CSSM_APPLE_CL_CSR_REQUEST = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		subjectNameX509: CSSM_X509_NAME_PTR;

=======
		subjectNameX509: CSSM_X509_NAME_PTR;		
	
>>>>>>> graemeg/cpstrnew
=======
		subjectNameX509: CSSM_X509_NAME_PTR;		
	
>>>>>>> graemeg/cpstrnew
=======
		subjectNameX509: CSSM_X509_NAME_PTR;		
	
>>>>>>> graemeg/cpstrnew
=======
		subjectNameX509: CSSM_X509_NAME_PTR;		
	
>>>>>>> origin/cpstrnew
	{ Unfortunately there is no practical way to map any algorithm
	 * to its appropriate OID, and we need both.... }
		signatureAlg: CSSM_ALGORITHMS;   // e.g., CSSM_ALGID_SHA1WithRSA
		signatureOid: CSSM_OID;	// e.g., CSSMOID_SHA1WithRSA

		cspHand: CSSM_CSP_HANDLE;		// sign with this CSP
		subjectPublicKey: {const} CSSM_KEYPtr;
		subjectPrivateKey: {const} CSSM_KEYPtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

	{
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> graemeg/cpstrnew
=======
	
	{ 
>>>>>>> origin/cpstrnew
	 * Optional challenge string.
	 }
		challengeString: {const} CStringPtr;
	end;

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * When a CRL with no NextUpdate field is encountered, we use this time
=======
 * When a CRL with no NextUpdate field is encountered, we use this time 
>>>>>>> graemeg/cpstrnew
=======
 * When a CRL with no NextUpdate field is encountered, we use this time 
>>>>>>> graemeg/cpstrnew
=======
 * When a CRL with no NextUpdate field is encountered, we use this time 
>>>>>>> graemeg/cpstrnew
=======
 * When a CRL with no NextUpdate field is encountered, we use this time 
>>>>>>> origin/cpstrnew
 * as the NextUpdate attribute when storing in a DB. It represents the
 * virtual end of time in CSSM_TIMESTRING form.
 }
const
	CSSM_APPLE_CRL_END_OF_TIME = '99991231235959';

{
 * Default filesystem names and locations for SecurityServer features
 * (included here for lack of a better place)
 }
const
	kKeychainSuffix = '.keychain';
const
	kSystemKeychainName = 'System.keychain';
const
	kSystemKeychainDir = '/Library/Keychains/';
const
	kSystemUnlockFile = '/var/db/SystemKey';


procedure cssmPerror( how: ConstCStringPtr; error: CSSM_RETURN ); external name '_cssmPerror';

{ Convert between CSSM_OID and CSSM_ALGORITHMS }
function cssmOidToAlg( const (*var*) oid: CSSM_OID; var alg: CSSM_ALGORITHMS ): CBool; external name '_cssmOidToAlg';
function cssmAlgToOid( algId: CSSM_ALGORITHMS ): CSSM_OIDPtr; external name '_cssmAlgToOid';

{
 * The MacOS OSStatus space has an embedding for UNIX errno values, similar to
 * the way we embed CSSM_RETURN values in OSStatus. These are the base and limit
 * values for this embedding.
 }
const
	errSecErrnoBase = 100000;
const
	errSecErrnoLimit = 100255;

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

{$ifc TARGET_OS_MAC}



function CSSM_ACL_AUTHORIZATION_PREAUTH(slot: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH:=CSSM_ACL_AUTHORIZATION_PREAUTH_BASE + slot
end;

function CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT(auth: UInt32): UInt32; inline;
begin
  CSSM_ACL_AUTHORIZATION_PREAUTH_SLOT:=auth - CSSM_ACL_AUTHORIZATION_PREAUTH_BASE
end;

function CSSM_ACL_AUTHORIZATION_IS_PREAUTH(auth: UInt32): Boolean; inline;
begin
  CSSM_ACL_AUTHORIZATION_IS_PREAUTH:=
    (auth >= CSSM_ACL_AUTHORIZATION_PREAUTH_BASE) and
    (auth < CSSM_ACL_AUTHORIZATION_PREAUTH_END)
end;

{$endc} {TARGET_OS_MAC}

end.

{$endc} {not MACOSALLINCLUDE}
