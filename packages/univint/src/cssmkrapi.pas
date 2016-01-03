{
 * Copyright (c) 1999-2001,2004 Apple Computer, Inc. All Rights Reserved.
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
 *
 * cssmkrapi.h -- Application Programmers Interface for Key Recovery Modules
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

unit cssmkrapi;
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
uses MacTypes,cssmtype;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$packrecords c}

type
	CSSM_KRSP_HANDLE = UInt32; { Key Recovery Service Provider Handle }

type
	cssm_kr_name = record
		Type_: UInt8; { namespace type }
		Length: UInt8; { name string length }
		Name: CStringPtr; { name string }
	end;

type
	CSSM_KR_PROFILE_PTR = ^cssm_kr_profile;
	CSSM_KR_PROFILEPtr = ^cssm_kr_profile;
	cssm_kr_profile = record
		UserName: CSSM_KR_NAME; { name of the user }
		UserCertificate: CSSM_CERTGROUP_PTR; { public key certificate of the user }
		KRSCertChain: CSSM_CERTGROUP_PTR; { cert chain for the KRSP coordinator }
		LE_KRANum: UInt8; { number of KRA cert chains in the following list }
		LE_KRACertChainList: CSSM_CERTGROUP_PTR; { list of Law enforcement KRA certificate chains }
		ENT_KRANum: UInt8; { number of KRA cert chains in the following list }
		ENT_KRACertChainList: CSSM_CERTGROUP_PTR; { list of Enterprise KRA certificate chains }
		INDIV_KRANum: UInt8; { number of KRA cert chains in the following list }
		INDIV_KRACertChainList: CSSM_CERTGROUP_PTR; { list of Individual KRA certificate chains }
		INDIV_AuthenticationInfo: CSSM_DATA_PTR; { authentication information for individual key recovery }
		KRSPFlags: UInt32; { flag values to be interpreted by KRSP }
		KRSPExtensions: CSSM_DATA_PTR; { reserved for extensions specific to KRSPs }
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

type
	CSSM_KR_WRAPPEDPRODUCT_INFO_PTR = ^CSSM_KR_WRAPPEDPRODUCT_INFO;
	CSSM_KR_WRAPPEDPRODUCT_INFOPtr = ^CSSM_KR_WRAPPEDPRODUCT_INFO;
	CSSM_KR_WRAPPEDPRODUCT_INFO = record
		StandardVersion: CSSM_VERSION;
		StandardDescription: CSSM_STRING;
		ProductVersion: CSSM_VERSION;
		ProductDescription: CSSM_STRING;
		ProductVendor: CSSM_STRING;
		ProductFlags: UInt32;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

type
	CSSM_KRSUBSERVICE_PTR = ^cssm_krsubservice;
	CSSM_KRSUBSERVICEPtr = ^cssm_krsubservice;
	cssm_krsubservice = record
		SubServiceId: UInt32;
		Description: CStringPtr; { Description of this sub service }
		WrappedProduct: CSSM_KR_WRAPPEDPRODUCT_INFO;
	end;

type
	CSSM_KR_POLICY_TYPE = UInt32;
const
	CSSM_KR_INDIV_POLICY = $00000001;
const
	CSSM_KR_ENT_POLICY = $00000002;
const
	CSSM_KR_LE_MAN_POLICY = $00000003;
const
	CSSM_KR_LE_USE_POLICY = $00000004;

type
	CSSM_KR_POLICY_FLAGS = UInt32;

const
	CSSM_KR_INDIV = $00000001;
const
	CSSM_KR_ENT = $00000002;
const
	CSSM_KR_LE_MAN = $00000004;
const
	CSSM_KR_LE_USE = $00000008;
const
	CSSM_KR_LE = (CSSM_KR_LE_MAN or CSSM_KR_LE_USE);
const
	CSSM_KR_OPTIMIZE = $00000010;
const
	CSSM_KR_DROP_WORKFACTOR = $00000020;

type
	CSSM_KR_POLICY_LIST_ITEM_PTR = ^cssm_kr_policy_list_item;
	CSSM_KR_POLICY_LIST_ITEMPtr = ^cssm_kr_policy_list_item;
	cssm_kr_policy_list_item = record
		next: CSSM_KR_POLICY_LIST_ITEM_PTR;
		AlgorithmId: CSSM_ALGORITHMS;
		Mode: CSSM_ENCRYPT_MODE;
		MaxKeyLength: UInt32;
		MaxRounds: UInt32;
		WorkFactor: UInt8;
		PolicyFlags: CSSM_KR_POLICY_FLAGS;
		AlgClass: CSSM_CONTEXT_TYPE;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

type
	CSSM_KR_POLICY_INFO_PTR = ^cssm_kr_policy_info;
	CSSM_KR_POLICY_INFOPtr = ^cssm_kr_policy_info;
	cssm_kr_policy_info = record
		krbNotAllowed: CSSM_BOOL;
		numberOfEntries: UInt32;
		policyEntry: CSSM_KR_POLICY_LIST_ITEMPtr;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{ Key Recovery Module Mangement Operations }

function CSSM_KR_SetEnterpriseRecoveryPolicy( const (*var*) RecoveryPolicyFileName: CSSM_DATA; const (*var*) OldPassPhrase: CSSM_ACCESS_CREDENTIALS; const (*var*) NewPassPhrase: CSSM_ACCESS_CREDENTIALS ): CSSM_RETURN; external name '_CSSM_KR_SetEnterpriseRecoveryPolicy';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{ Key Recovery Context Operations }

function CSSM_KR_CreateRecoveryRegistrationContext( KRSPHandle: CSSM_KRSP_HANDLE; var NewContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_CreateRecoveryRegistrationContext';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_CreateRecoveryEnablementContext( KRSPHandle: CSSM_KRSP_HANDLE; const (*var*) LocalProfile: CSSM_KR_PROFILE; const (*var*) RemoteProfile: CSSM_KR_PROFILE; var NewContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_CreateRecoveryEnablementContext';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_CreateRecoveryRequestContext( KRSPHandle: CSSM_KRSP_HANDLE; const (*var*) LocalProfile: CSSM_KR_PROFILE; var NewContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_CreateRecoveryRequestContext';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_GetPolicyInfo( CCHandle: CSSM_CC_HANDLE; var EncryptionProhibited: CSSM_KR_POLICY_FLAGS; var WorkFactor: UInt32 ): CSSM_RETURN; external name '_CSSM_KR_GetPolicyInfo';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======

function CSSM_KR_CreateRecoveryEnablementContext( KRSPHandle: CSSM_KRSP_HANDLE; const (*var*) LocalProfile: CSSM_KR_PROFILE; const (*var*) RemoteProfile: CSSM_KR_PROFILE; var NewContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_CreateRecoveryEnablementContext';

function CSSM_KR_CreateRecoveryRequestContext( KRSPHandle: CSSM_KRSP_HANDLE; const (*var*) LocalProfile: CSSM_KR_PROFILE; var NewContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_CreateRecoveryRequestContext';

function CSSM_KR_GetPolicyInfo( CCHandle: CSSM_CC_HANDLE; var EncryptionProhibited: CSSM_KR_POLICY_FLAGS; var WorkFactor: UInt32 ): CSSM_RETURN; external name '_CSSM_KR_GetPolicyInfo';
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

function CSSM_KR_CreateRecoveryEnablementContext( KRSPHandle: CSSM_KRSP_HANDLE; const (*var*) LocalProfile: CSSM_KR_PROFILE; const (*var*) RemoteProfile: CSSM_KR_PROFILE; var NewContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_CreateRecoveryEnablementContext';

function CSSM_KR_CreateRecoveryRequestContext( KRSPHandle: CSSM_KRSP_HANDLE; const (*var*) LocalProfile: CSSM_KR_PROFILE; var NewContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_CreateRecoveryRequestContext';

function CSSM_KR_GetPolicyInfo( CCHandle: CSSM_CC_HANDLE; var EncryptionProhibited: CSSM_KR_POLICY_FLAGS; var WorkFactor: UInt32 ): CSSM_RETURN; external name '_CSSM_KR_GetPolicyInfo';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{ Key Recovery Registration Operations }

function CSSM_KR_RegistrationRequest( RecoveryRegistrationContext: CSSM_CC_HANDLE; const (*var*) KRInData: CSSM_DATA; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; KRFlags: CSSM_KR_POLICY_FLAGS; var EstimatedTime: SInt32; ReferenceHandle: CSSM_HANDLE_PTR ): CSSM_RETURN; external name '_CSSM_KR_RegistrationRequest';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_RegistrationRetrieve( KRSPHandle: CSSM_KRSP_HANDLE; ReferenceHandle: CSSM_HANDLE; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; var EstimatedTime: SInt32; KRProfile: CSSM_KR_PROFILE_PTR ): CSSM_RETURN; external name '_CSSM_KR_RegistrationRetrieve';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======

function CSSM_KR_RegistrationRetrieve( KRSPHandle: CSSM_KRSP_HANDLE; ReferenceHandle: CSSM_HANDLE; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; var EstimatedTime: SInt32; KRProfile: CSSM_KR_PROFILE_PTR ): CSSM_RETURN; external name '_CSSM_KR_RegistrationRetrieve';
>>>>>>> graemeg/cpstrnew
=======

function CSSM_KR_RegistrationRetrieve( KRSPHandle: CSSM_KRSP_HANDLE; ReferenceHandle: CSSM_HANDLE; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; var EstimatedTime: SInt32; KRProfile: CSSM_KR_PROFILE_PTR ): CSSM_RETURN; external name '_CSSM_KR_RegistrationRetrieve';
>>>>>>> graemeg/cpstrnew
=======

function CSSM_KR_RegistrationRetrieve( KRSPHandle: CSSM_KRSP_HANDLE; ReferenceHandle: CSSM_HANDLE; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; var EstimatedTime: SInt32; KRProfile: CSSM_KR_PROFILE_PTR ): CSSM_RETURN; external name '_CSSM_KR_RegistrationRetrieve';
>>>>>>> graemeg/cpstrnew
=======

function CSSM_KR_RegistrationRetrieve( KRSPHandle: CSSM_KRSP_HANDLE; ReferenceHandle: CSSM_HANDLE; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; var EstimatedTime: SInt32; KRProfile: CSSM_KR_PROFILE_PTR ): CSSM_RETURN; external name '_CSSM_KR_RegistrationRetrieve';
>>>>>>> origin/cpstrnew


{ Key Recovery Enablement Operations }

function CSSM_KR_GenerateRecoveryFields( KeyRecoveryContext: CSSM_CC_HANDLE; CCHandle: CSSM_CC_HANDLE; const (*var*) KRSPOptions: CSSM_DATA; KRFlags: CSSM_KR_POLICY_FLAGS; KRFields: CSSM_DATA_PTR; var NewCCHandle: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_GenerateRecoveryFields';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_ProcessRecoveryFields( KeyRecoveryContext: CSSM_CC_HANDLE; CryptoContext: CSSM_CC_HANDLE; const (*var*) KRSPOptions: CSSM_DATA; KRFlags: CSSM_KR_POLICY_FLAGS; const (*var*) KRFields: CSSM_DATA; var NewCryptoContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_ProcessRecoveryFields';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======

function CSSM_KR_ProcessRecoveryFields( KeyRecoveryContext: CSSM_CC_HANDLE; CryptoContext: CSSM_CC_HANDLE; const (*var*) KRSPOptions: CSSM_DATA; KRFlags: CSSM_KR_POLICY_FLAGS; const (*var*) KRFields: CSSM_DATA; var NewCryptoContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_ProcessRecoveryFields';
>>>>>>> graemeg/cpstrnew
=======

function CSSM_KR_ProcessRecoveryFields( KeyRecoveryContext: CSSM_CC_HANDLE; CryptoContext: CSSM_CC_HANDLE; const (*var*) KRSPOptions: CSSM_DATA; KRFlags: CSSM_KR_POLICY_FLAGS; const (*var*) KRFields: CSSM_DATA; var NewCryptoContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_ProcessRecoveryFields';
>>>>>>> graemeg/cpstrnew
=======

function CSSM_KR_ProcessRecoveryFields( KeyRecoveryContext: CSSM_CC_HANDLE; CryptoContext: CSSM_CC_HANDLE; const (*var*) KRSPOptions: CSSM_DATA; KRFlags: CSSM_KR_POLICY_FLAGS; const (*var*) KRFields: CSSM_DATA; var NewCryptoContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_ProcessRecoveryFields';
>>>>>>> graemeg/cpstrnew
=======

function CSSM_KR_ProcessRecoveryFields( KeyRecoveryContext: CSSM_CC_HANDLE; CryptoContext: CSSM_CC_HANDLE; const (*var*) KRSPOptions: CSSM_DATA; KRFlags: CSSM_KR_POLICY_FLAGS; const (*var*) KRFields: CSSM_DATA; var NewCryptoContext: CSSM_CC_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_ProcessRecoveryFields';
>>>>>>> origin/cpstrnew


{ Key Recovery Request Operations }

function CSSM_KR_RecoveryRequest( RecoveryRequestContext: CSSM_CC_HANDLE; const (*var*) KRInData: CSSM_DATA; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; var EstimatedTime: SInt32; ReferenceHandle: CSSM_HANDLE_PTR ): CSSM_RETURN; external name '_CSSM_KR_RecoveryRequest';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_RecoveryRetrieve( KRSPHandle: CSSM_KRSP_HANDLE; ReferenceHandle: CSSM_HANDLE; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; var EstimatedTime: SInt32; CacheHandle: CSSM_HANDLE_PTR; var NumberOfRecoveredKeys: UInt32 ): CSSM_RETURN; external name '_CSSM_KR_RecoveryRetrieve';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_GetRecoveredObject( KRSPHandle: CSSM_KRSP_HANDLE; CacheHandle: CSSM_HANDLE; IndexInResults: UInt32; CSPHandle: CSSM_CSP_HANDLE; const (*var*) CredAndAclEntry: CSSM_RESOURCE_CONTROL_CONTEXT; Flags: UInt32; RecoveredKey: CSSM_KEY_PTR; OtherInfo: CSSM_DATA_PTR ): CSSM_RETURN; external name '_CSSM_KR_GetRecoveredObject';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_RecoveryRequestAbort( KRSPHandle: CSSM_KRSP_HANDLE; CacheHandle: CSSM_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_RecoveryRequestAbort';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

function CSSM_KR_QueryPolicyInfo( KRSPHandle: CSSM_KRSP_HANDLE; AlgorithmID: CSSM_ALGORITHMS; Mode: CSSM_ENCRYPT_MODE; Class: CSSM_CONTEXT_TYPE; var PolicyInfoData: CSSM_KR_POLICY_INFO_PTR ): CSSM_RETURN; external name '_CSSM_KR_QueryPolicyInfo';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

function CSSM_KR_RecoveryRetrieve( KRSPHandle: CSSM_KRSP_HANDLE; ReferenceHandle: CSSM_HANDLE; const (*var*) AccessCredentials: CSSM_ACCESS_CREDENTIALS; var EstimatedTime: SInt32; CacheHandle: CSSM_HANDLE_PTR; var NumberOfRecoveredKeys: UInt32 ): CSSM_RETURN; external name '_CSSM_KR_RecoveryRetrieve';

function CSSM_KR_GetRecoveredObject( KRSPHandle: CSSM_KRSP_HANDLE; CacheHandle: CSSM_HANDLE; IndexInResults: UInt32; CSPHandle: CSSM_CSP_HANDLE; const (*var*) CredAndAclEntry: CSSM_RESOURCE_CONTROL_CONTEXT; Flags: UInt32; RecoveredKey: CSSM_KEY_PTR; OtherInfo: CSSM_DATA_PTR ): CSSM_RETURN; external name '_CSSM_KR_GetRecoveredObject';

function CSSM_KR_RecoveryRequestAbort( KRSPHandle: CSSM_KRSP_HANDLE; CacheHandle: CSSM_HANDLE ): CSSM_RETURN; external name '_CSSM_KR_RecoveryRequestAbort';

function CSSM_KR_QueryPolicyInfo( KRSPHandle: CSSM_KRSP_HANDLE; AlgorithmID: CSSM_ALGORITHMS; Mode: CSSM_ENCRYPT_MODE; Class: CSSM_CONTEXT_TYPE; var PolicyInfoData: CSSM_KR_POLICY_INFO_PTR ): CSSM_RETURN; external name '_CSSM_KR_QueryPolicyInfo';
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


{ Extensibility Functions }

function CSSM_KR_PassThrough( KRSPHandle: CSSM_KRSP_HANDLE; KeyRecoveryContext: CSSM_CC_HANDLE; CryptoContext: CSSM_CC_HANDLE; PassThroughId: UInt32; InputParams: {const} UnivPtr; OutputParams: UnivPtrPtr ): CSSM_RETURN; external name '_CSSM_KR_PassThrough';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
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
