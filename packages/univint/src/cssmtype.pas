{
 * Copyright (c) 1999-2002,2004 Apple Computer, Inc. All Rights Reserved.
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
 * cssmtype.h -- Common Security Services Manager Common Data Types
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

unit cssmtype;
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
uses MacTypes,cssmconfig;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$packrecords c}

{ Handle types. }
	
type
	CSSM_HANDLE = CSSM_INTPTR;
	CSSM_HANDLE_PTR = ^CSSM_INTPTR;
	CSSM_HANDLEPtr = ^CSSM_INTPTR;

type
	CSSM_LONG_HANDLE = UInt64;
	CSSM_LONG_HANDLE_PTR = ^UInt64;
	CSSM_LONG_HANDLEPtr = ^UInt64;

type
	CSSM_MODULE_HANDLE = CSSM_HANDLE;
	CSSM_MODULE_HANDLE_PTR = ^CSSM_HANDLE;
	CSSM_MODULE_HANDLEPtr = ^CSSM_HANDLE;

type
	CSSM_CC_HANDLE = CSSM_LONG_HANDLE; { Cryptographic Context Handle }

type
	CSSM_CSP_HANDLE = CSSM_MODULE_HANDLE; { Cryptographic Service Provider Handle }

type
	CSSM_TP_HANDLE = CSSM_MODULE_HANDLE; { Trust Policy Handle }

type
	CSSM_AC_HANDLE = CSSM_MODULE_HANDLE; { Authorization Computation Handle }

type
	CSSM_CL_HANDLE = CSSM_MODULE_HANDLE; { Certificate Library Handle }

type
	CSSM_DL_HANDLE = CSSM_MODULE_HANDLE; { Data Storage Library Handle }

type
	CSSM_DB_HANDLE = CSSM_MODULE_HANDLE; { Data Storage Database Handle }


{ invalid or NULL value for any CSSM_HANDLE type }
const
	CSSM_INVALID_HANDLE = 0;


{ Data Types for Core Services }

type
	CSSM_BOOL = SInt32;
const
	CSSM_FALSE = 0;
	CSSM_TRUE = 1;

{ The standard declares this as uint32 but we changed it to sint32 to match OSStatus. }
type
  CSSM_RETURNPtr = ^CSSM_RETURN;
	CSSM_RETURN = SInt32;
const
	CSSM_OK = 0;

const
	CSSM_MODULE_STRING_SIZE = 64;
type
	CSSM_STRING = array [0..CSSM_MODULE_STRING_SIZE + 4-1] of char;


type
	CSSM_DATA_PTR = ^cssm_data;
	CSSM_DATAPtr = ^cssm_data;
	CSSM_DATAArrayPtr = ^cssm_dataPtr;
	cssm_data = record
		Length: CSSM_SIZE; { in bytes }
		Data: UInt8Ptr;
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
	CSSM_GUID_PTR = ^cssm_guid;
	CSSM_GUIDPtr = ^cssm_guid;
	cssm_guid = record
		Data1: UInt32;
		Data2: UInt16;
		Data3: UInt16;
		Data4: array [0..8-1] of UInt8;
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
	CSSM_BITMASK = UInt32;
	CSSM_KEY_HIERARCHY = CSSM_BITMASK;
const
	CSSM_KEY_HIERARCHY_NONE = 0;
	CSSM_KEY_HIERARCHY_INTEG = 1;
	CSSM_KEY_HIERARCHY_EXPORT = 2;

type
	CSSM_PVC_MODE = CSSM_BITMASK;
const
	CSSM_PVC_NONE = 0;
	CSSM_PVC_APP = 1;
	CSSM_PVC_SP = 2;

type
	CSSM_PRIVILEGE_SCOPE = UInt32;
const
	CSSM_PRIVILEGE_SCOPE_NONE = 0;
	CSSM_PRIVILEGE_SCOPE_PROCESS = 1;
	CSSM_PRIVILEGE_SCOPE_THREAD = 2;

type
	CSSM_VERSION_PTR = ^cssm_version;
	CSSM_VERSIONPtr = ^cssm_version;
	cssm_version = record
		Major: UInt32;
		Minor: UInt32;
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
	CSSM_SERVICE_MASK = UInt32;
const
	CSSM_SERVICE_CSSM = $1;
	CSSM_SERVICE_CSP = $2;
	CSSM_SERVICE_DL = $4;
	CSSM_SERVICE_CL = $8;
	CSSM_SERVICE_TP = $10;
	CSSM_SERVICE_AC = $20;
	CSSM_SERVICE_KR = $40;

type
	CSSM_SERVICE_TYPE = CSSM_SERVICE_MASK;

type
	CSSM_SUBSERVICE_UID_PTR = ^cssm_subservice_uid;
	CSSM_SUBSERVICE_UIDPtr = ^cssm_subservice_uid;
	cssm_subservice_uid = record
		Guid: CSSM_GUID;
		Version: CSSM_VERSION;
		SubserviceId: UInt32;
		SubserviceType: CSSM_SERVICE_TYPE;
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
	CSSM_MODULE_EVENT = UInt32;
	CSSM_MODULE_EVENT_PTR = ^UInt32;
	CSSM_MODULE_EVENTPtr = ^UInt32;
const
	CSSM_NOTIFY_INSERT = 1;
	CSSM_NOTIFY_REMOVE = 2;
	CSSM_NOTIFY_FAULT = 3;

type
	CSSM_API_ModuleEventHandler = function( const (*var*) ModuleGuid: CSSM_GUID; AppNotifyCallbackCtx: UnivPtr; SubserviceId: UInt32; ServiceType: CSSM_SERVICE_TYPE; EventType: CSSM_MODULE_EVENT ): CSSM_RETURN;

type
	CSSM_ATTACH_FLAGS = UInt32;
const
	CSSM_ATTACH_READ_ONLY = $00000001;

{ Non-export privilege range: (0x00000000 - 0x7FFFFFFF) }
{ Vendor specific range: (0x80000000 - 0xFFFFFFFF) }
type
	CSSM_PRIVILEGE = UInt64;
	CSSM_USEE_TAG = CSSM_PRIVILEGE;
const
	CSSM_USEE_LAST = $FF;
	CSSM_USEE_NONE = 0;
	CSSM_USEE_DOMESTIC = 1;
	CSSM_USEE_FINANCIAL = 2;
	CSSM_USEE_KRLE = 3;
	CSSM_USEE_KRENT = 4;
	CSSM_USEE_SSL = 5;
	CSSM_USEE_AUTHENTICATION = 6;
	CSSM_USEE_KEYEXCH = 7;
	CSSM_USEE_MEDICAL = 8;
	CSSM_USEE_INSURANCE = 9;
	CSSM_USEE_WEAK = 10;

type
	CSSM_NET_ADDRESS_TYPE = UInt32;
const
	CSSM_ADDR_NONE = 0;
	CSSM_ADDR_CUSTOM = 1;
	CSSM_ADDR_URL = 2; { char* }
	CSSM_ADDR_SOCKADDR = 3;
	CSSM_ADDR_NAME = 4; { char* - qualified by access method }

type
	CSSM_NET_ADDRESS_PTR = ^cssm_net_address;
	CSSM_NET_ADDRESSPtr = ^cssm_net_address;
	cssm_net_address = record
		AddressType: CSSM_NET_ADDRESS_TYPE;
		Address: CSSM_DATA;
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
	CSSM_NET_PROTOCOL = UInt32;
const
	CSSM_NET_PROTO_NONE = 0;	{ local }
	CSSM_NET_PROTO_CUSTOM = 1;	{ proprietary implementation }
	CSSM_NET_PROTO_UNSPECIFIED = 2;	{ implementation default }
	CSSM_NET_PROTO_LDAP = 3;	{ light weight directory access protocol }
	CSSM_NET_PROTO_LDAPS = 4;	{ ldap/ssl where SSL initiates the connection }
	CSSM_NET_PROTO_LDAPNS = 5;	{ ldap where ldap negotiates an SSL session }
	CSSM_NET_PROTO_X500DAP = 6;	{ x.500 Directory access protocol }
	CSSM_NET_PROTO_FTP = 7;	{ ftp for cert/crl fetch }
	CSSM_NET_PROTO_FTPS = 8;	{ ftp/ssl/tls where SSL/TLS initiates the connection }
	CSSM_NET_PROTO_OCSP = 9;	{ online certificate status protocol }
	CSSM_NET_PROTO_CMP = 10;	{ the cert request protocol in PKIX3 }
	CSSM_NET_PROTO_CMPS = 11;	{ The ssl/tls derivative of CMP }

type
	CSSM_CALLBACK = function( OutData: CSSM_DATA_PTR; CallerCtx: UnivPtr ): CSSM_RETURN;

type
	CSSM_CRYPTO_DATA_PTR = ^cssm_crypto_data;
	CSSM_CRYPTO_DATAPtr = ^cssm_crypto_data;
	cssm_crypto_data = record
		Param: CSSM_DATA;
		Callback: CSSM_CALLBACK;
		CallerCtx: UnivPtr;
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
	CSSM_WORDID_TYPE = SInt32;
const
	CSSM_WORDID__UNK_ = -1; { not in dictionary }
	CSSM_WORDID__NLU_ = 0; { not yet looked up }
	CSSM_WORDID__STAR_ = 1;
	CSSM_WORDID_A = 2;
	CSSM_WORDID_ACL = 3;
	CSSM_WORDID_ALPHA = 4;
	CSSM_WORDID_B = 5;
	CSSM_WORDID_BER = 6;
	CSSM_WORDID_BINARY = 7;
	CSSM_WORDID_BIOMETRIC = 8;
	CSSM_WORDID_C = 9;
	CSSM_WORDID_CANCELED = 10;
	CSSM_WORDID_CERT = 11;
	CSSM_WORDID_COMMENT = 12;
	CSSM_WORDID_CRL = 13;
	CSSM_WORDID_CUSTOM = 14;
	CSSM_WORDID_D = 15;
	CSSM_WORDID_DATE = 16;
	CSSM_WORDID_DB_DELETE = 17;
	CSSM_WORDID_DB_EXEC_STORED_QUERY = 18;
	CSSM_WORDID_DB_INSERT = 19;
	CSSM_WORDID_DB_MODIFY = 20;
	CSSM_WORDID_DB_READ = 21;
	CSSM_WORDID_DBS_CREATE = 22;
	CSSM_WORDID_DBS_DELETE = 23;
	CSSM_WORDID_DECRYPT = 24;
	CSSM_WORDID_DELETE = 25;
	CSSM_WORDID_DELTA_CRL = 26;
	CSSM_WORDID_DER = 27;
	CSSM_WORDID_DERIVE = 28;
	CSSM_WORDID_DISPLAY = 29;
	CSSM_WORDID_DO = 30;
	CSSM_WORDID_DSA = 31;
	CSSM_WORDID_DSA_SHA1 = 32;
	CSSM_WORDID_E = 33;
	CSSM_WORDID_ELGAMAL = 34;
	CSSM_WORDID_ENCRYPT = 35;
	CSSM_WORDID_ENTRY = 36;
	CSSM_WORDID_EXPORT_CLEAR = 37;
	CSSM_WORDID_EXPORT_WRAPPED = 38;
	CSSM_WORDID_G = 39;
	CSSM_WORDID_GE = 40;
	CSSM_WORDID_GENKEY = 41;
	CSSM_WORDID_HASH = 42;
	CSSM_WORDID_HASHED_PASSWORD = 43;
	CSSM_WORDID_HASHED_SUBJECT = 44;
	CSSM_WORDID_HAVAL = 45;
	CSSM_WORDID_IBCHASH = 46;
	CSSM_WORDID_IMPORT_CLEAR = 47;
	CSSM_WORDID_IMPORT_WRAPPED = 48;
	CSSM_WORDID_INTEL = 49;
	CSSM_WORDID_ISSUER = 50;
	CSSM_WORDID_ISSUER_INFO = 51;
	CSSM_WORDID_K_OF_N = 52;
	CSSM_WORDID_KEA = 53;
	CSSM_WORDID_KEYHOLDER = 54;
	CSSM_WORDID_L = 55;
	CSSM_WORDID_LE = 56;
	CSSM_WORDID_LOGIN = 57;
	CSSM_WORDID_LOGIN_NAME = 58;
	CSSM_WORDID_MAC = 59;
	CSSM_WORDID_MD2 = 60;
	CSSM_WORDID_MD2WITHRSA = 61;
	CSSM_WORDID_MD4 = 62;
	CSSM_WORDID_MD5 = 63;
	CSSM_WORDID_MD5WITHRSA = 64;
	CSSM_WORDID_N = 65;
	CSSM_WORDID_NAME = 66;
	CSSM_WORDID_NDR = 67;
	CSSM_WORDID_NHASH = 68;
	CSSM_WORDID_NOT_AFTER = 69;
	CSSM_WORDID_NOT_BEFORE = 70;
	CSSM_WORDID_NULL = 71;
	CSSM_WORDID_NUMERIC = 72;
	CSSM_WORDID_OBJECT_HASH = 73;
	CSSM_WORDID_ONE_TIME = 74;
	CSSM_WORDID_ONLINE = 75;
	CSSM_WORDID_OWNER = 76;
	CSSM_WORDID_P = 77;
	CSSM_WORDID_PAM_NAME = 78;
	CSSM_WORDID_PASSWORD = 79;
	CSSM_WORDID_PGP = 80;
	CSSM_WORDID_PREFIX = 81;
	CSSM_WORDID_PRIVATE_KEY = 82;
	CSSM_WORDID_PROMPTED_BIOMETRIC = 83;
	CSSM_WORDID_PROMPTED_PASSWORD = 84;
	CSSM_WORDID_PROPAGATE = 85;
	CSSM_WORDID_PROTECTED_BIOMETRIC = 86;
	CSSM_WORDID_PROTECTED_PASSWORD = 87;
	CSSM_WORDID_PROTECTED_PIN = 88;
	CSSM_WORDID_PUBLIC_KEY = 89;
	CSSM_WORDID_PUBLIC_KEY_FROM_CERT = 90;
	CSSM_WORDID_Q = 91;
	CSSM_WORDID_RANGE = 92;
	CSSM_WORDID_REVAL = 93;
	CSSM_WORDID_RIPEMAC = 94;
	CSSM_WORDID_RIPEMD = 95;
	CSSM_WORDID_RIPEMD160 = 96;
	CSSM_WORDID_RSA = 97;
	CSSM_WORDID_RSA_ISO9796 = 98;
	CSSM_WORDID_RSA_PKCS = 99;
	CSSM_WORDID_RSA_PKCS_MD5 = 100;
	CSSM_WORDID_RSA_PKCS_SHA1 = 101;
	CSSM_WORDID_RSA_PKCS1 = 102;
	CSSM_WORDID_RSA_PKCS1_MD5 = 103;
	CSSM_WORDID_RSA_PKCS1_SHA1 = 104;
	CSSM_WORDID_RSA_PKCS1_SIG = 105;
	CSSM_WORDID_RSA_RAW = 106;
	CSSM_WORDID_SDSIV1 = 107;
	CSSM_WORDID_SEQUENCE = 108;
	CSSM_WORDID_SET = 109;
	CSSM_WORDID_SEXPR = 110;
	CSSM_WORDID_SHA1 = 111;
	CSSM_WORDID_SHA1WITHDSA = 112;
	CSSM_WORDID_SHA1WITHECDSA = 113;
	CSSM_WORDID_SHA1WITHRSA = 114;
	CSSM_WORDID_SIGN = 115;
	CSSM_WORDID_SIGNATURE = 116;
	CSSM_WORDID_SIGNED_NONCE = 117;
	CSSM_WORDID_SIGNED_SECRET = 118;
	CSSM_WORDID_SPKI = 119;
	CSSM_WORDID_SUBJECT = 120;
	CSSM_WORDID_SUBJECT_INFO = 121;
	CSSM_WORDID_TAG = 122;
	CSSM_WORDID_THRESHOLD = 123;
	CSSM_WORDID_TIME = 124;
	CSSM_WORDID_URI = 125;
	CSSM_WORDID_VERSION = 126;
	CSSM_WORDID_X509_ATTRIBUTE = 127;
	CSSM_WORDID_X509V1 = 128;
	CSSM_WORDID_X509V2 = 129;
	CSSM_WORDID_X509V3 = 130;
	CSSM_WORDID_X9_ATTRIBUTE = 131;
	CSSM_WORDID_VENDOR_START = $00010000;
	CSSM_WORDID_VENDOR_END = $7FFF0000;

type
	CSSM_LIST_ELEMENT_TYPE = UInt32;
	CSSM_LIST_ELEMENT_TYPE_PTR = ^UInt32;
	CSSM_LIST_ELEMENT_TYPEPtr = ^UInt32;
const
	CSSM_LIST_ELEMENT_DATUM = $00;
	CSSM_LIST_ELEMENT_SUBLIST = $01;
	CSSM_LIST_ELEMENT_WORDID = $02;

type
	CSSM_LIST_TYPE = UInt32;
	CSSM_LIST_TYPE_PTR = ^UInt32;
	CSSM_LIST_TYPEPtr = ^UInt32;
const
	CSSM_LIST_TYPE_UNKNOWN = 0;
	CSSM_LIST_TYPE_CUSTOM = 1;
	CSSM_LIST_TYPE_SEXPR = 2;

type
  CSSM_LIST_ELEMENT_PTR = ^cssm_list_element;
  CSSM_LIST_ELEMENTPtr = ^cssm_list_element;
	CSSM_LIST_PTR = ^cssm_list;
	CSSM_LISTPtr = ^cssm_list;
	cssm_list = record
		ListType: CSSM_LIST_TYPE;	{ type of this list }
		Head: CSSM_LIST_ELEMENT_PTR;	{ head of the list }
		Tail: CSSM_LIST_ELEMENT_PTR;	{ tail of the list }
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

  __embedded_cssm_list_element = record
    case Integer of
      0: (Sublist: CSSM_LIST);		{ sublist }
      1: (Word: CSSM_DATA);		{ a byte-string }
  end;

	cssm_list_element = record
		NextElement: cssm_list_element_ptr;	{ next list element }
		WordID: CSSM_WORDID_TYPE;	{ integer identifier associated }
								{ with a Word value }
		ElementType: CSSM_LIST_ELEMENT_TYPE;
		Element: __embedded_cssm_list_element;
	end;

type
	CSSM_TUPLE_PTR = ^CSSM_TUPLE;
	CSSM_TUPLEPtr = ^CSSM_TUPLE;
	CSSM_TUPLE = record
{ 5-tuple definition }
		Issuer: CSSM_LIST;			{ issuer, or empty if ACL }
		Subject: CSSM_LIST;			{ subject }
		Delegate: CSSM_BOOL;			{ permission to delegate }
		AuthorizationTag: CSSM_LIST;	{ authorization field }
		ValidityPeriod: CSSM_LIST;	{ validity information (dates) }
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
	CSSM_TUPLEGROUP_PTR = ^cssm_tuplegroup;
	CSSM_TUPLEGROUPPtr = ^cssm_tuplegroup;
	cssm_tuplegroup = record
		NumberOfTuples: UInt32;
		Tuples: CSSM_TUPLE_PTR;
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
	CSSM_SAMPLE_TYPE = CSSM_WORDID_TYPE;
const
	CSSM_SAMPLE_TYPE_PASSWORD = CSSM_WORDID_PASSWORD;
	CSSM_SAMPLE_TYPE_HASHED_PASSWORD = CSSM_WORDID_HASHED_PASSWORD;
	CSSM_SAMPLE_TYPE_PROTECTED_PASSWORD = CSSM_WORDID_PROTECTED_PASSWORD;
	CSSM_SAMPLE_TYPE_PROMPTED_PASSWORD = CSSM_WORDID_PROMPTED_PASSWORD;
	CSSM_SAMPLE_TYPE_SIGNED_NONCE = CSSM_WORDID_SIGNED_NONCE;
	CSSM_SAMPLE_TYPE_SIGNED_SECRET = CSSM_WORDID_SIGNED_SECRET;
	CSSM_SAMPLE_TYPE_BIOMETRIC = CSSM_WORDID_BIOMETRIC;
	CSSM_SAMPLE_TYPE_PROTECTED_BIOMETRIC = CSSM_WORDID_PROTECTED_BIOMETRIC;
	CSSM_SAMPLE_TYPE_PROMPTED_BIOMETRIC = CSSM_WORDID_PROMPTED_BIOMETRIC;
	CSSM_SAMPLE_TYPE_THRESHOLD = CSSM_WORDID_THRESHOLD;

type
	CSSM_SAMPLE_PTR = ^cssm_sample;
	CSSM_SAMPLEPtr = ^cssm_sample;
	cssm_sample = record
		TypedSample: CSSM_LIST;
		Verifier: {const} CSSM_SUBSERVICE_UIDPtr;
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
	CSSM_SAMPLEGROUP_PTR = ^cssm_samplegroup;
	CSSM_SAMPLEGROUPPtr = ^cssm_samplegroup;
	cssm_samplegroup = record
		NumberOfSamples: UInt32;
		Samples: {const} CSSM_SAMPLEPtr;
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
	CSSM_MALLOC = function( size: CSSM_SIZE; allocref: UnivPtr ): UnivPtr;

type
	CSSM_FREE = procedure( memblock: UnivPtr; allocref: UnivPtr );

type
	CSSM_REALLOC = function( memblock: UnivPtr; size: CSSM_SIZE; allocref: UnivPtr ): UnivPtr;

type
	CSSM_CALLOC = function( num: UInt32; size: CSSM_SIZE; allocref: UnivPtr ): UnivPtr;

type
	CSSM_MEMORY_FUNCS_PTR = ^cssm_memory_funcs;
	CSSM_MEMORY_FUNCSPtr = ^cssm_memory_funcs;
	cssm_memory_funcs = record
		malloc_func: CSSM_MALLOC;
		free_func: CSSM_FREE;
		realloc_func: CSSM_REALLOC;
		calloc_func: CSSM_CALLOC;
		AllocRef: UnivPtr;
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
	CSSM_API_MEMORY_FUNCS = CSSM_MEMORY_FUNCS;
	CSSM_API_MEMORY_FUNCS_PTR = ^CSSM_API_MEMORY_FUNCS;
	CSSM_API_MEMORY_FUNCSPtr = ^CSSM_API_MEMORY_FUNCS;

type
	CSSM_CHALLENGE_CALLBACK = function( const (*var*) Challenge: CSSM_LIST; Response: CSSM_SAMPLEGROUP_PTR; CallerCtx: UnivPtr; const (*var*) MemFuncs: CSSM_MEMORY_FUNCS ): CSSM_RETURN;

type
	CSSM_CERT_TYPE = UInt32;
	CSSM_CERT_TYPE_PTR = ^UInt32;
	CSSM_CERT_TYPEPtr = ^UInt32;
const
	CSSM_CERT_UNKNOWN = $00;
	CSSM_CERT_X_509v1 = $01;
	CSSM_CERT_X_509v2 = $02;
	CSSM_CERT_X_509v3 = $03;
	CSSM_CERT_PGP = $04;
	CSSM_CERT_SPKI = $05;
	CSSM_CERT_SDSIv1 = $06;
	CSSM_CERT_Intel = $08;
	CSSM_CERT_X_509_ATTRIBUTE = $09; { X.509 attribute cert }
	CSSM_CERT_X9_ATTRIBUTE = $0A; { X9 attribute cert }
	CSSM_CERT_TUPLE = $0B;
	CSSM_CERT_ACL_ENTRY = $0C;
	CSSM_CERT_MULTIPLE = $7FFE;
	CSSM_CERT_LAST = $7FFF;
	{ Applications wishing to define their own custom certificate
	   type should define and publicly document a uint32 value greater
	   than the CSSM_CL_CUSTOM_CERT_TYPE }
	CSSM_CL_CUSTOM_CERT_TYPE = $08000;

type
	CSSM_CERT_ENCODING = UInt32;
	CSSM_CERT_ENCODING_PTR = ^UInt32;
	CSSM_CERT_ENCODINGPtr = ^UInt32;
const
	CSSM_CERT_ENCODING_UNKNOWN = $00;
	CSSM_CERT_ENCODING_CUSTOM = $01;
	CSSM_CERT_ENCODING_BER = $02;
	CSSM_CERT_ENCODING_DER = $03;
	CSSM_CERT_ENCODING_NDR = $04;
	CSSM_CERT_ENCODING_SEXPR = $05;
	CSSM_CERT_ENCODING_PGP = $06;
	CSSM_CERT_ENCODING_MULTIPLE = $7FFE;
	CSSM_CERT_ENCODING_LAST = $7FFF;
	{ Applications wishing to define their own custom certificate
	   encoding should create a uint32 value greater than the
	   CSSM_CL_CUSTOM_CERT_ENCODING }
	CSSM_CL_CUSTOM_CERT_ENCODING = $8000;

type
	CSSM_ENCODED_CERT_PTR = ^cssm_encoded_cert;
	CSSM_ENCODED_CERTPtr = ^cssm_encoded_cert;
	cssm_encoded_cert = record
		CertType: CSSM_CERT_TYPE;			{ type of certificate }
		CertEncoding: CSSM_CERT_ENCODING;	{ encoding for this packed cert }
		CertBlob: CSSM_DATA;					{ packed cert }
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
	CSSM_CERT_PARSE_FORMAT = UInt32;
	CSSM_CERT_PARSE_FORMAT_PTR = ^UInt32;
	CSSM_CERT_PARSE_FORMATPtr = ^UInt32;
const
	CSSM_CERT_PARSE_FORMAT_NONE = $00;
	CSSM_CERT_PARSE_FORMAT_CUSTOM = $01; { void* }
	CSSM_CERT_PARSE_FORMAT_SEXPR = $02; { CSSM_LIST }
	CSSM_CERT_PARSE_FORMAT_COMPLEX = $03; { void* }
	CSSM_CERT_PARSE_FORMAT_OID_NAMED = $04; { CSSM_FIELDGROUP }
	CSSM_CERT_PARSE_FORMAT_TUPLE = $05; { CSSM_TUPLE }
	CSSM_CERT_PARSE_FORMAT_MULTIPLE = $7FFE;
{ multiple forms, each cert carries a
   parse format indicator }
	CSSM_CERT_PARSE_FORMAT_LAST = $7FFF;
{ Applications wishing to define their
   own custom parse format should create
   a * uint32 value greater than the
   CSSM_CL_CUSTOM_CERT_PARSE_FORMAT }
	CSSM_CL_CUSTOM_CERT_PARSE_FORMAT = $8000;

type
	CSSM_PARSED_CERT_PTR = ^cssm_parsed_cert;
	CSSM_PARSED_CERTPtr = ^cssm_parsed_cert;
	cssm_parsed_cert = record
		CertType: CSSM_CERT_TYPE; { certificate type }
		ParsedCertFormat: CSSM_CERT_PARSE_FORMAT;
    { struct of ParsedCert }
		ParsedCert: UnivPtr; { parsed cert (to be typecast) }
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
	CSSM_CERT_PAIR_PTR = ^cssm_cert_pair;
	CSSM_CERT_PAIRPtr = ^cssm_cert_pair;
	cssm_cert_pair = record
		EncodedCert: CSSM_ENCODED_CERT; { an encoded certificate blob }
		ParsedCert: CSSM_PARSED_CERT; { equivalent parsed certificate }
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
	CSSM_CERTGROUP_TYPE = UInt32;
	CSSM_CERTGROUP_TYPE_PTR = ^UInt32;
	CSSM_CERTGROUP_TYPEPtr = ^UInt32;
const
	CSSM_CERTGROUP_DATA = $00;
	CSSM_CERTGROUP_ENCODED_CERT = $01;
	CSSM_CERTGROUP_PARSED_CERT = $02;
	CSSM_CERTGROUP_CERT_PAIR = $03;

type
  __EmbeddedGroupListType = record
    case Integer of
      0: (CertList: CSSM_DATA_PTR); { legacy list of single type certificate blobs }
		  1: (EncodedCertList: CSSM_ENCODED_CERT_PTR);
        { list of multi-type certificate blobs }
		  2: (ParsedCertList: CSSM_PARSED_CERT_PTR);
        { list of multi-type parsed certs }
		  3: (PairCertList: CSSM_CERT_PAIR_PTR);
        {list of single or multi-type certs with two representations: blob and parsed }
	end;
	cssm_certgroup = record
		CertType: CSSM_CERT_TYPE;
		CertEncoding: CSSM_CERT_ENCODING;
		NumCerts: UInt32; { # of certificates in this list }
    CertGroupType: __EmbeddedGroupListType;
    { type of structure in the GroupList }
    Reserved: UnivPtr; { reserved for implementation dependent use }
  end;
  cssm_certgroup_ptr = ^cssm_certgroup;
  CSSM_certgroupPtr = ^cssm_certgroup;

type
	CSSM_BASE_CERTS_PTR = ^cssm_base_certs;
	CSSM_BASE_CERTSPtr = ^cssm_base_certs;
	cssm_base_certs = record
		TPHandle: CSSM_TP_HANDLE;
		CLHandle: CSSM_CL_HANDLE;
		Certs: CSSM_CERTGROUP;
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
	CSSM_ACCESS_CREDENTIALS_PTR = ^cssm_access_credentials;
	CSSM_ACCESS_CREDENTIALSPtr = ^cssm_access_credentials;
	cssm_access_credentials = record
		EntryTag: CSSM_STRING;
		BaseCerts: CSSM_BASE_CERTS;
		Samples: CSSM_SAMPLEGROUP;
		Callback: CSSM_CHALLENGE_CALLBACK;
		CallerCtx: UnivPtr;
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
	CSSM_ACL_SUBJECT_TYPE = SInt32;
const
	CSSM_ACL_SUBJECT_TYPE_ANY = CSSM_WORDID__STAR_;
	CSSM_ACL_SUBJECT_TYPE_THRESHOLD = CSSM_WORDID_THRESHOLD;
	CSSM_ACL_SUBJECT_TYPE_PASSWORD = CSSM_WORDID_PASSWORD;
	CSSM_ACL_SUBJECT_TYPE_PROTECTED_PASSWORD = CSSM_WORDID_PROTECTED_PASSWORD;
	CSSM_ACL_SUBJECT_TYPE_PROMPTED_PASSWORD = CSSM_WORDID_PROMPTED_PASSWORD;
	CSSM_ACL_SUBJECT_TYPE_PUBLIC_KEY = CSSM_WORDID_PUBLIC_KEY;
	CSSM_ACL_SUBJECT_TYPE_HASHED_SUBJECT = CSSM_WORDID_HASHED_SUBJECT;
	CSSM_ACL_SUBJECT_TYPE_BIOMETRIC = CSSM_WORDID_BIOMETRIC;
	CSSM_ACL_SUBJECT_TYPE_PROTECTED_BIOMETRIC = CSSM_WORDID_PROTECTED_BIOMETRIC;
	CSSM_ACL_SUBJECT_TYPE_PROMPTED_BIOMETRIC = CSSM_WORDID_PROMPTED_BIOMETRIC;
	CSSM_ACL_SUBJECT_TYPE_LOGIN_NAME = CSSM_WORDID_LOGIN_NAME;
	CSSM_ACL_SUBJECT_TYPE_EXT_PAM_NAME = CSSM_WORDID_PAM_NAME;

{ Authorization tag type }
type
	CSSM_ACL_AUTHORIZATION_TAG = SInt32;
	CSSM_ACL_AUTHORIZATION_TAGPtr = ^CSSM_ACL_AUTHORIZATION_TAG;
const
{ All vendor specific constants must be in the number range
	   starting at CSSM_ACL_AUTHORIZATION_TAG_VENDOR_DEFINED_START }
	CSSM_ACL_AUTHORIZATION_TAG_VENDOR_DEFINED_START = $00010000;
	{ No restrictions. Permission to perform all operations on
	   the resource or available to an ACL owner.  }
	CSSM_ACL_AUTHORIZATION_ANY = CSSM_WORDID__STAR_;
	{ Defined authorization tag values for CSPs }
	CSSM_ACL_AUTHORIZATION_LOGIN = CSSM_WORDID_LOGIN;
	CSSM_ACL_AUTHORIZATION_GENKEY = CSSM_WORDID_GENKEY;
	CSSM_ACL_AUTHORIZATION_DELETE = CSSM_WORDID_DELETE;
	CSSM_ACL_AUTHORIZATION_EXPORT_WRAPPED = CSSM_WORDID_EXPORT_WRAPPED;
	CSSM_ACL_AUTHORIZATION_EXPORT_CLEAR = CSSM_WORDID_EXPORT_CLEAR;
	CSSM_ACL_AUTHORIZATION_IMPORT_WRAPPED = CSSM_WORDID_IMPORT_WRAPPED;
	CSSM_ACL_AUTHORIZATION_IMPORT_CLEAR = CSSM_WORDID_IMPORT_CLEAR;
	CSSM_ACL_AUTHORIZATION_SIGN = CSSM_WORDID_SIGN;
	CSSM_ACL_AUTHORIZATION_ENCRYPT = CSSM_WORDID_ENCRYPT;
	CSSM_ACL_AUTHORIZATION_DECRYPT = CSSM_WORDID_DECRYPT;
	CSSM_ACL_AUTHORIZATION_MAC = CSSM_WORDID_MAC;
	CSSM_ACL_AUTHORIZATION_DERIVE = CSSM_WORDID_DERIVE;
	{ Defined authorization tag values for DLs }
	CSSM_ACL_AUTHORIZATION_DBS_CREATE = CSSM_WORDID_DBS_CREATE;
	CSSM_ACL_AUTHORIZATION_DBS_DELETE = CSSM_WORDID_DBS_DELETE;
	CSSM_ACL_AUTHORIZATION_DB_READ = CSSM_WORDID_DB_READ;
	CSSM_ACL_AUTHORIZATION_DB_INSERT = CSSM_WORDID_DB_INSERT;
	CSSM_ACL_AUTHORIZATION_DB_MODIFY = CSSM_WORDID_DB_MODIFY;
	CSSM_ACL_AUTHORIZATION_DB_DELETE = CSSM_WORDID_DB_DELETE;

type
	CSSM_AUTHORIZATIONGROUP_PTR = ^cssm_authorizationgroup;
	CSSM_AUTHORIZATIONGROUPPtr = ^cssm_authorizationgroup;
	cssm_authorizationgroup = record
		NumberOfAuthTags: UInt32;
		AuthTags: CSSM_ACL_AUTHORIZATION_TAGPtr;
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
	CSSM_ACL_VALIDITY_PERIOD_PTR = ^cssm_acl_validity_period;
	CSSM_ACL_VALIDITY_PERIODPtr = ^cssm_acl_validity_period;
	cssm_acl_validity_period = record
		StartDate: CSSM_DATA;
		EndDate: CSSM_DATA;
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
	CSSM_ACL_ENTRY_PROTOTYPE_PTR = ^cssm_acl_entry_prototype;
	CSSM_ACL_ENTRY_PROTOTYPEPtr = ^cssm_acl_entry_prototype;
	cssm_acl_entry_prototype = record
		TypedSubject: CSSM_LIST;
		Delegate: CSSM_BOOL;
		Authorization: CSSM_AUTHORIZATIONGROUP;
		TimeRange: CSSM_ACL_VALIDITY_PERIOD;
		EntryTag: CSSM_STRING;
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
	CSSM_ACL_OWNER_PROTOTYPE_PTR = ^cssm_acl_owner_prototype;
	CSSM_ACL_OWNER_PROTOTYPEPtr = ^cssm_acl_owner_prototype;
	cssm_acl_owner_prototype = record
		TypedSubject: CSSM_LIST;
		Delegate: CSSM_BOOL;
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
	CSSM_ACL_SUBJECT_CALLBACK = function( const (*var*) SubjectRequest: CSSM_LIST; SubjectResponse: CSSM_LIST_PTR; CallerContext: UnivPtr; const (*var*) MemFuncs: CSSM_MEMORY_FUNCS ): CSSM_RETURN;

type
	CSSM_ACL_ENTRY_INPUT_PTR = ^cssm_acl_entry_input;
	CSSM_ACL_ENTRY_INPUTPtr = ^cssm_acl_entry_input;
	cssm_acl_entry_input = record
		Prototype: CSSM_ACL_ENTRY_PROTOTYPE;
		Callback: CSSM_ACL_SUBJECT_CALLBACK;
		CallerContext: UnivPtr;
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
	CSSM_RESOURCE_CONTROL_CONTEXT_PTR = ^cssm_resource_control_context;
	CSSM_RESOURCE_CONTROL_CONTEXTPtr = ^cssm_resource_control_context;
	cssm_resource_control_context = record
		AccessCred: CSSM_ACCESS_CREDENTIALS_PTR;
		InitialAclEntry: CSSM_ACL_ENTRY_INPUT;
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
	CSSM_ACL_HANDLE = CSSM_HANDLE;

type
	CSSM_ACL_ENTRY_INFO_PTR = ^cssm_acl_entry_info;
	CSSM_ACL_ENTRY_INFOPtr = ^cssm_acl_entry_info;
	cssm_acl_entry_info = record
		EntryPublicInfo: CSSM_ACL_ENTRY_PROTOTYPE;
		EntryHandle: CSSM_ACL_HANDLE;
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
	CSSM_ACL_EDIT_MODE = UInt32;
const
	CSSM_ACL_EDIT_MODE_ADD = 1;
	CSSM_ACL_EDIT_MODE_DELETE = 2;
	CSSM_ACL_EDIT_MODE_REPLACE = 3;

type
	CSSM_ACL_EDIT_PTR = ^cssm_acl_edit;
	CSSM_ACL_EDITPtr = ^cssm_acl_edit;
	cssm_acl_edit = record
		EditMode: CSSM_ACL_EDIT_MODE;
		OldEntryHandle: CSSM_ACL_HANDLE;
		NewEntry: {const} CSSM_ACL_ENTRY_INPUTPtr;
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

{$ifc defined(WIN32)}
type
	CSSM_PROC_ADDR = FARPROC;
{$elsec}
type
	CSSM_PROC_ADDR = procedure;
{$endc}
  CSSM_PROC_ADDRPtr = ^CSSM_PROC_ADDR;
type
	CSSM_PROC_ADDR_PTR = CSSM_PROC_ADDRPtr;

type
	CSSM_FUNC_NAME_ADDR_PTR = ^cssm_func_name_addr;
	CSSM_FUNC_NAME_ADDRPtr = ^cssm_func_name_addr;
	cssm_func_name_addr = record
		Name: CSSM_STRING;
		Address: CSSM_PROC_ADDR;
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


{ Data Types for Cryptographic Services  }

type
	CSSM_DATE_PTR = ^cssm_date;
	CSSM_DATEPtr = ^cssm_date;
	cssm_date = record
		Year: array [0..4-1] of UInt8;
		Month: array [0..2-1] of UInt8;
		Day: array [0..2-1] of UInt8;
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
	CSSM_RANGE_PTR = ^cssm_range;
	CSSM_RANGEPtr = ^cssm_range;
	cssm_range = record
		Min: UInt32; { inclusive minimum value }
		Max: UInt32; { inclusive maximum value }
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
	CSSM_QUERY_SIZE_DATA_PTR = ^cssm_query_size_data;
	CSSM_QUERY_SIZE_DATAPtr = ^cssm_query_size_data;
	cssm_query_size_data = record
		SizeInputBlock: UInt32; { size of input data block }
		SizeOutputBlock: UInt32; { size of resulting output data block }
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
	CSSM_HEADERVERSION = UInt32;
const
	CSSM_KEYHEADER_VERSION = 2;

type
	CSSM_KEY_SIZE_PTR = ^cssm_key_size;
	CSSM_KEY_SIZEPtr = ^cssm_key_size;
	cssm_key_size = record
		LogicalKeySizeInBits: UInt32; { Logical key size in bits }
		EffectiveKeySizeInBits: UInt32; { Effective key size in bits }
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
	CSSM_KEYBLOB_TYPE = UInt32;
const
	CSSM_KEYBLOB_RAW = 0; { The blob is a clear, raw key }
	CSSM_KEYBLOB_REFERENCE = 2; { The blob is a reference to a key }
	CSSM_KEYBLOB_WRAPPED = 3; { The blob is a wrapped RAW key }
	CSSM_KEYBLOB_OTHER = $FFFFFFFF;

type
	CSSM_KEYBLOB_FORMAT = UInt32;
const
{ Raw Format }
	CSSM_KEYBLOB_RAW_FORMAT_NONE = 0;
	{ No further conversion need to be done }
	CSSM_KEYBLOB_RAW_FORMAT_PKCS1 = 1; { RSA PKCS1 V1.5 }
	CSSM_KEYBLOB_RAW_FORMAT_PKCS3 = 2; { RSA PKCS3 V1.5 }
	CSSM_KEYBLOB_RAW_FORMAT_MSCAPI = 3; { Microsoft CAPI V2.0 }
	CSSM_KEYBLOB_RAW_FORMAT_PGP = 4; { PGP V }
	CSSM_KEYBLOB_RAW_FORMAT_FIPS186 = 5; { US Gov. FIPS 186 - DSS V }
	CSSM_KEYBLOB_RAW_FORMAT_BSAFE = 6; { RSA Bsafe V3.0 }
	CSSM_KEYBLOB_RAW_FORMAT_CCA = 9; { CCA clear public key blob }
	CSSM_KEYBLOB_RAW_FORMAT_PKCS8 = 10; { RSA PKCS8 V1.2 }
	CSSM_KEYBLOB_RAW_FORMAT_SPKI = 11; { SPKI Specification }
	CSSM_KEYBLOB_RAW_FORMAT_OCTET_STRING = 12;
	CSSM_KEYBLOB_RAW_FORMAT_OTHER = $FFFFFFFF; { Other, CSP defined }
const
{ Wrapped Format }
	CSSM_KEYBLOB_WRAPPED_FORMAT_NONE = 0;
	{ No further conversion need to be done }
	CSSM_KEYBLOB_WRAPPED_FORMAT_PKCS8 = 1; { RSA PKCS8 V1.2 }
	CSSM_KEYBLOB_WRAPPED_FORMAT_PKCS7 = 2;
	CSSM_KEYBLOB_WRAPPED_FORMAT_MSCAPI = 3;
	CSSM_KEYBLOB_WRAPPED_FORMAT_OTHER = $FFFFFFFF; { Other, CSP defined }
const
{ Reference Format }
	CSSM_KEYBLOB_REF_FORMAT_INTEGER = 0;	{ Reference is a number or handle }
	CSSM_KEYBLOB_REF_FORMAT_STRING = 1;	{ Reference is a string or label }
	CSSM_KEYBLOB_REF_FORMAT_SPKI = 2;	{ Reference is an SPKI S-expression }
												{ to be evaluated to locate the key }
	CSSM_KEYBLOB_REF_FORMAT_OTHER = $FFFFFFFF;	{ Other, CSP defined }

type
	CSSM_KEYCLASS = UInt32;
const
	CSSM_KEYCLASS_PUBLIC_KEY = 0; { Key is public key }
	CSSM_KEYCLASS_PRIVATE_KEY = 1; { Key is private key }
	CSSM_KEYCLASS_SESSION_KEY = 2; { Key is session or symmetric key }
	CSSM_KEYCLASS_SECRET_PART = 3; { Key is part of secret key }
	CSSM_KEYCLASS_OTHER = $FFFFFFFF; { Other }

type
	CSSM_KEYATTR_FLAGS = UInt32;
const
{ Valid only during call to an API. Will never be valid when set in a key header }
	CSSM_KEYATTR_RETURN_DEFAULT = $00000000;
	CSSM_KEYATTR_RETURN_DATA = $10000000;
	CSSM_KEYATTR_RETURN_REF = $20000000;
	CSSM_KEYATTR_RETURN_NONE = $40000000;
	{ Valid during an API call and in a key header }
	CSSM_KEYATTR_PERMANENT = $00000001;
	CSSM_KEYATTR_PRIVATE = $00000002;
	CSSM_KEYATTR_MODIFIABLE = $00000004;
	CSSM_KEYATTR_SENSITIVE = $00000008;
	CSSM_KEYATTR_EXTRACTABLE = $00000020;
	{ Valid only in a key header generated by a CSP, not valid during an API call }
	CSSM_KEYATTR_ALWAYS_SENSITIVE = $00000010;
	CSSM_KEYATTR_NEVER_EXTRACTABLE = $00000040;

type
	CSSM_KEYUSE = UInt32;
const
	CSSM_KEYUSE_ANY = $80000000;
	CSSM_KEYUSE_ENCRYPT = $00000001;
	CSSM_KEYUSE_DECRYPT = $00000002;
	CSSM_KEYUSE_SIGN = $00000004;
	CSSM_KEYUSE_VERIFY = $00000008;
	CSSM_KEYUSE_SIGN_RECOVER = $00000010;
	CSSM_KEYUSE_VERIFY_RECOVER = $00000020;
	CSSM_KEYUSE_WRAP = $00000040;
	CSSM_KEYUSE_UNWRAP = $00000080;
	CSSM_KEYUSE_DERIVE = $00000100;

type
	CSSM_ALGORITHMS = UInt32;
const
	CSSM_ALGID_NONE = 0;
	CSSM_ALGID_CUSTOM = CSSM_ALGID_NONE + 1;
	CSSM_ALGID_DH = CSSM_ALGID_NONE + 2;
	CSSM_ALGID_PH = CSSM_ALGID_NONE + 3;
	CSSM_ALGID_KEA = CSSM_ALGID_NONE + 4;
	CSSM_ALGID_MD2 = CSSM_ALGID_NONE + 5;
	CSSM_ALGID_MD4 = CSSM_ALGID_NONE + 6;
	CSSM_ALGID_MD5 = CSSM_ALGID_NONE + 7;
	CSSM_ALGID_SHA1 = CSSM_ALGID_NONE + 8;
	CSSM_ALGID_NHASH = CSSM_ALGID_NONE + 9;
	CSSM_ALGID_HAVAL = CSSM_ALGID_NONE + 10;
	CSSM_ALGID_RIPEMD = CSSM_ALGID_NONE + 11;
	CSSM_ALGID_IBCHASH = CSSM_ALGID_NONE + 12;
	CSSM_ALGID_RIPEMAC = CSSM_ALGID_NONE + 13;
	CSSM_ALGID_DES = CSSM_ALGID_NONE + 14;
	CSSM_ALGID_DESX = CSSM_ALGID_NONE + 15;
	CSSM_ALGID_RDES = CSSM_ALGID_NONE + 16;
	CSSM_ALGID_3DES_3KEY_EDE = CSSM_ALGID_NONE + 17;
	CSSM_ALGID_3DES_2KEY_EDE = CSSM_ALGID_NONE + 18;
	CSSM_ALGID_3DES_1KEY_EEE = CSSM_ALGID_NONE + 19;
	CSSM_ALGID_3DES_3KEY = CSSM_ALGID_3DES_3KEY_EDE;
	CSSM_ALGID_3DES_3KEY_EEE = CSSM_ALGID_NONE + 20;
	CSSM_ALGID_3DES_2KEY = CSSM_ALGID_3DES_2KEY_EDE;
	CSSM_ALGID_3DES_2KEY_EEE = CSSM_ALGID_NONE + 21;
	CSSM_ALGID_3DES_1KEY = CSSM_ALGID_3DES_3KEY_EEE;
	CSSM_ALGID_IDEA = CSSM_ALGID_NONE + 22;
	CSSM_ALGID_RC2 = CSSM_ALGID_NONE + 23;
	CSSM_ALGID_RC5 = CSSM_ALGID_NONE + 24;
	CSSM_ALGID_RC4 = CSSM_ALGID_NONE + 25;
	CSSM_ALGID_SEAL = CSSM_ALGID_NONE + 26;
	CSSM_ALGID_CAST = CSSM_ALGID_NONE + 27;
	CSSM_ALGID_BLOWFISH = CSSM_ALGID_NONE + 28;
	CSSM_ALGID_SKIPJACK = CSSM_ALGID_NONE + 29;
	CSSM_ALGID_LUCIFER = CSSM_ALGID_NONE + 30;
	CSSM_ALGID_MADRYGA = CSSM_ALGID_NONE + 31;
	CSSM_ALGID_FEAL = CSSM_ALGID_NONE + 32;
	CSSM_ALGID_REDOC = CSSM_ALGID_NONE + 33;
	CSSM_ALGID_REDOC3 = CSSM_ALGID_NONE + 34;
	CSSM_ALGID_LOKI = CSSM_ALGID_NONE + 35;
	CSSM_ALGID_KHUFU = CSSM_ALGID_NONE + 36;
	CSSM_ALGID_KHAFRE = CSSM_ALGID_NONE + 37;
	CSSM_ALGID_MMB = CSSM_ALGID_NONE + 38;
	CSSM_ALGID_GOST = CSSM_ALGID_NONE + 39;
	CSSM_ALGID_SAFER = CSSM_ALGID_NONE + 40;
	CSSM_ALGID_CRAB = CSSM_ALGID_NONE + 41;
	CSSM_ALGID_RSA = CSSM_ALGID_NONE + 42;
	CSSM_ALGID_DSA = CSSM_ALGID_NONE + 43;
	CSSM_ALGID_MD5WithRSA = CSSM_ALGID_NONE + 44;
	CSSM_ALGID_MD2WithRSA = CSSM_ALGID_NONE + 45;
	CSSM_ALGID_ElGamal = CSSM_ALGID_NONE + 46;
	CSSM_ALGID_MD2Random = CSSM_ALGID_NONE + 47;
	CSSM_ALGID_MD5Random = CSSM_ALGID_NONE + 48;
	CSSM_ALGID_SHARandom = CSSM_ALGID_NONE + 49;
	CSSM_ALGID_DESRandom = CSSM_ALGID_NONE + 50;
	CSSM_ALGID_SHA1WithRSA = CSSM_ALGID_NONE + 51;
	CSSM_ALGID_CDMF = CSSM_ALGID_NONE + 52;
	CSSM_ALGID_CAST3 = CSSM_ALGID_NONE + 53;
	CSSM_ALGID_CAST5 = CSSM_ALGID_NONE + 54;
	CSSM_ALGID_GenericSecret = CSSM_ALGID_NONE + 55;
	CSSM_ALGID_ConcatBaseAndKey = CSSM_ALGID_NONE + 56;
	CSSM_ALGID_ConcatKeyAndBase = CSSM_ALGID_NONE + 57;
	CSSM_ALGID_ConcatBaseAndData = CSSM_ALGID_NONE + 58;
	CSSM_ALGID_ConcatDataAndBase = CSSM_ALGID_NONE + 59;
	CSSM_ALGID_XORBaseAndData = CSSM_ALGID_NONE + 60;
	CSSM_ALGID_ExtractFromKey = CSSM_ALGID_NONE + 61;
	CSSM_ALGID_SSL3PreMasterGen = CSSM_ALGID_NONE + 62;
	CSSM_ALGID_SSL3MasterDerive = CSSM_ALGID_NONE + 63;
	CSSM_ALGID_SSL3KeyAndMacDerive = CSSM_ALGID_NONE + 64;
	CSSM_ALGID_SSL3MD5_MAC = CSSM_ALGID_NONE + 65;
	CSSM_ALGID_SSL3SHA1_MAC = CSSM_ALGID_NONE + 66;
	CSSM_ALGID_PKCS5_PBKDF1_MD5 = CSSM_ALGID_NONE + 67;
	CSSM_ALGID_PKCS5_PBKDF1_MD2 = CSSM_ALGID_NONE + 68;
	CSSM_ALGID_PKCS5_PBKDF1_SHA1 = CSSM_ALGID_NONE + 69;
	CSSM_ALGID_WrapLynks = CSSM_ALGID_NONE + 70;
	CSSM_ALGID_WrapSET_OAEP = CSSM_ALGID_NONE + 71;
	CSSM_ALGID_BATON = CSSM_ALGID_NONE + 72;
	CSSM_ALGID_ECDSA = CSSM_ALGID_NONE + 73;
	CSSM_ALGID_MAYFLY = CSSM_ALGID_NONE + 74;
	CSSM_ALGID_JUNIPER = CSSM_ALGID_NONE + 75;
	CSSM_ALGID_FASTHASH = CSSM_ALGID_NONE + 76;
	CSSM_ALGID_3DES = CSSM_ALGID_NONE + 77;
	CSSM_ALGID_SSL3MD5 = CSSM_ALGID_NONE + 78;
	CSSM_ALGID_SSL3SHA1 = CSSM_ALGID_NONE + 79;
	CSSM_ALGID_FortezzaTimestamp = CSSM_ALGID_NONE + 80;
	CSSM_ALGID_SHA1WithDSA = CSSM_ALGID_NONE + 81;
	CSSM_ALGID_SHA1WithECDSA = CSSM_ALGID_NONE + 82;
	CSSM_ALGID_DSA_BSAFE = CSSM_ALGID_NONE + 83;
	CSSM_ALGID_ECDH = CSSM_ALGID_NONE + 84;
	CSSM_ALGID_ECMQV = CSSM_ALGID_NONE + 85;
	CSSM_ALGID_PKCS12_SHA1_PBE = CSSM_ALGID_NONE + 86;
	CSSM_ALGID_ECNRA = CSSM_ALGID_NONE + 87;
	CSSM_ALGID_SHA1WithECNRA = CSSM_ALGID_NONE + 88;
	CSSM_ALGID_ECES = CSSM_ALGID_NONE + 89;
	CSSM_ALGID_ECAES = CSSM_ALGID_NONE + 90;
	CSSM_ALGID_SHA1HMAC = CSSM_ALGID_NONE + 91;
	CSSM_ALGID_FIPS186Random = CSSM_ALGID_NONE + 92;
	CSSM_ALGID_ECC = CSSM_ALGID_NONE + 93;
	CSSM_ALGID_MQV = CSSM_ALGID_NONE + 94;
	CSSM_ALGID_NRA = CSSM_ALGID_NONE + 95;
	CSSM_ALGID_IntelPlatformRandom = CSSM_ALGID_NONE + 96;
	CSSM_ALGID_UTC = CSSM_ALGID_NONE + 97;
	CSSM_ALGID_HAVAL3 = CSSM_ALGID_NONE + 98;
	CSSM_ALGID_HAVAL4 = CSSM_ALGID_NONE + 99;
	CSSM_ALGID_HAVAL5 = CSSM_ALGID_NONE + 100;
	CSSM_ALGID_TIGER = CSSM_ALGID_NONE + 101;
	CSSM_ALGID_MD5HMAC = CSSM_ALGID_NONE + 102;
	CSSM_ALGID_PKCS5_PBKDF2 = CSSM_ALGID_NONE + 103;
	CSSM_ALGID_RUNNING_COUNTER = CSSM_ALGID_NONE + 104;
	CSSM_ALGID_LAST = CSSM_ALGID_NONE + $7FFFFFFF;
{ All algorithms IDs that are vendor specific, and not
   part of the CSSM specification should be defined relative
   to CSSM_ALGID_VENDOR_DEFINED. }
	CSSM_ALGID_VENDOR_DEFINED = CSSM_ALGID_NONE + $80000000;

type
	CSSM_ENCRYPT_MODE = UInt32;
const
	CSSM_ALGMODE_NONE = 0;
	CSSM_ALGMODE_CUSTOM = CSSM_ALGMODE_NONE + 1;
	CSSM_ALGMODE_ECB = CSSM_ALGMODE_NONE + 2;
	CSSM_ALGMODE_ECBPad = CSSM_ALGMODE_NONE + 3;
	CSSM_ALGMODE_CBC = CSSM_ALGMODE_NONE + 4;
	CSSM_ALGMODE_CBC_IV8 = CSSM_ALGMODE_NONE + 5;
	CSSM_ALGMODE_CBCPadIV8 = CSSM_ALGMODE_NONE + 6;
	CSSM_ALGMODE_CFB = CSSM_ALGMODE_NONE + 7;
	CSSM_ALGMODE_CFB_IV8 = CSSM_ALGMODE_NONE + 8;
	CSSM_ALGMODE_CFBPadIV8 = CSSM_ALGMODE_NONE + 9;
	CSSM_ALGMODE_OFB = CSSM_ALGMODE_NONE + 10;
	CSSM_ALGMODE_OFB_IV8 = CSSM_ALGMODE_NONE + 11;
	CSSM_ALGMODE_OFBPadIV8 = CSSM_ALGMODE_NONE + 12;
	CSSM_ALGMODE_COUNTER = CSSM_ALGMODE_NONE + 13;
	CSSM_ALGMODE_BC = CSSM_ALGMODE_NONE + 14;
	CSSM_ALGMODE_PCBC = CSSM_ALGMODE_NONE + 15;
	CSSM_ALGMODE_CBCC = CSSM_ALGMODE_NONE + 16;
	CSSM_ALGMODE_OFBNLF = CSSM_ALGMODE_NONE + 17;
	CSSM_ALGMODE_PBC = CSSM_ALGMODE_NONE + 18;
	CSSM_ALGMODE_PFB = CSSM_ALGMODE_NONE + 19;
	CSSM_ALGMODE_CBCPD = CSSM_ALGMODE_NONE + 20;
	CSSM_ALGMODE_PUBLIC_KEY = CSSM_ALGMODE_NONE + 21;
	CSSM_ALGMODE_PRIVATE_KEY = CSSM_ALGMODE_NONE + 22;
	CSSM_ALGMODE_SHUFFLE = CSSM_ALGMODE_NONE + 23;
	CSSM_ALGMODE_ECB64 = CSSM_ALGMODE_NONE + 24;
	CSSM_ALGMODE_CBC64 = CSSM_ALGMODE_NONE + 25;
	CSSM_ALGMODE_OFB64 = CSSM_ALGMODE_NONE + 26;
	CSSM_ALGMODE_CFB32 = CSSM_ALGMODE_NONE + 28;
	CSSM_ALGMODE_CFB16 = CSSM_ALGMODE_NONE + 29;
	CSSM_ALGMODE_CFB8 = CSSM_ALGMODE_NONE + 30;
	CSSM_ALGMODE_WRAP = CSSM_ALGMODE_NONE + 31;
	CSSM_ALGMODE_PRIVATE_WRAP = CSSM_ALGMODE_NONE + 32;
	CSSM_ALGMODE_RELAYX = CSSM_ALGMODE_NONE + 33;
	CSSM_ALGMODE_ECB128 = CSSM_ALGMODE_NONE + 34;
	CSSM_ALGMODE_ECB96 = CSSM_ALGMODE_NONE + 35;
	CSSM_ALGMODE_CBC128 = CSSM_ALGMODE_NONE + 36;
	CSSM_ALGMODE_OAEP_HASH = CSSM_ALGMODE_NONE + 37;
	CSSM_ALGMODE_PKCS1_EME_V15 = CSSM_ALGMODE_NONE + 38;
	CSSM_ALGMODE_PKCS1_EME_OAEP = CSSM_ALGMODE_NONE + 39;
	CSSM_ALGMODE_PKCS1_EMSA_V15 = CSSM_ALGMODE_NONE + 40;
	CSSM_ALGMODE_ISO_9796 = CSSM_ALGMODE_NONE + 41;
	CSSM_ALGMODE_X9_31 = CSSM_ALGMODE_NONE + 42;
	CSSM_ALGMODE_LAST = CSSM_ALGMODE_NONE + $7FFFFFFF;
{ All algorithms modes that are vendor specific, and
   not part of the CSSM specification should be defined
   relative to CSSM_ALGMODE_VENDOR_DEFINED. }
	CSSM_ALGMODE_VENDOR_DEFINED = CSSM_ALGMODE_NONE + $80000000;

type
	CSSM_KEYHEADER_PTR = ^cssm_keyheader;
	CSSM_KEYHEADERPtr = ^cssm_keyheader;
	cssm_keyheader = record
		HeaderVersion: CSSM_HEADERVERSION; { Key header version }
		CspId: CSSM_GUID; { GUID of CSP generating the key }
		BlobType: CSSM_KEYBLOB_TYPE; { See BlobType enum }
		Format: CSSM_KEYBLOB_FORMAT; { Raw or Reference format }
		AlgorithmId: CSSM_ALGORITHMS; { Algorithm ID of key }
		KeyClass: CSSM_KEYCLASS; { Public/Private/Secret, etc. }
		LogicalKeySizeInBits: UInt32; { Logical key size in bits }
		KeyAttr: CSSM_KEYATTR_FLAGS; { Attribute flags }
		KeyUsage: CSSM_KEYUSE; { Key use flags }
		StartDate: CSSM_DATE; { Effective date of key }
		EndDate: CSSM_DATE; { Expiration date of key }
		WrapAlgorithmId: CSSM_ALGORITHMS; { == CSSM_ALGID_NONE if clear key }
		WrapMode: CSSM_ENCRYPT_MODE; { if alg supports multiple wrapping modes }
		Reserved: UInt32;
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
	CSSM_KEY_PTR = ^cssm_key;
	CSSM_KEYPtr = ^cssm_key;
	cssm_key = record
		KeyHeader: CSSM_KEYHEADER; { Fixed length key header }
		KeyData: CSSM_DATA; { Variable length key data }
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
	CSSM_WRAP_KEY = CSSM_KEY;
	CSSM_WRAP_KEY_PTR = ^CSSM_KEY;
	CSSM_WRAP_KEYPtr = ^CSSM_KEY;

type
	CSSM_CSPTYPE = UInt32;
const
	CSSM_CSP_SOFTWARE = 1;
	CSSM_CSP_HARDWARE = CSSM_CSP_SOFTWARE + 1;
	CSSM_CSP_HYBRID = CSSM_CSP_SOFTWARE + 2;

{ From DL. }
type
	CSSM_DL_DB_HANDLE_PTR = ^cssm_dl_db_handle;
	CSSM_DL_DB_HANDLEPtr = ^cssm_dl_db_handle;
	cssm_dl_db_handle = record
		DLHandle: CSSM_DL_HANDLE;
		DBHandle: CSSM_DB_HANDLE;
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
	CSSM_CONTEXT_TYPE = UInt32;
const
	CSSM_ALGCLASS_NONE = 0;
	CSSM_ALGCLASS_CUSTOM = CSSM_ALGCLASS_NONE + 1;
	CSSM_ALGCLASS_SIGNATURE = CSSM_ALGCLASS_NONE + 2;
	CSSM_ALGCLASS_SYMMETRIC = CSSM_ALGCLASS_NONE + 3;
	CSSM_ALGCLASS_DIGEST = CSSM_ALGCLASS_NONE + 4;
	CSSM_ALGCLASS_RANDOMGEN = CSSM_ALGCLASS_NONE + 5;
	CSSM_ALGCLASS_UNIQUEGEN = CSSM_ALGCLASS_NONE + 6;
	CSSM_ALGCLASS_MAC = CSSM_ALGCLASS_NONE + 7;
	CSSM_ALGCLASS_ASYMMETRIC = CSSM_ALGCLASS_NONE + 8;
	CSSM_ALGCLASS_KEYGEN = CSSM_ALGCLASS_NONE + 9;
	CSSM_ALGCLASS_DERIVEKEY = CSSM_ALGCLASS_NONE + 10;

{ Attribute data type tags }
const
	CSSM_ATTRIBUTE_DATA_NONE = $00000000;
	CSSM_ATTRIBUTE_DATA_UINT32 = $10000000;
	CSSM_ATTRIBUTE_DATA_CSSM_DATA = $20000000;
	CSSM_ATTRIBUTE_DATA_CRYPTO_DATA = $30000000;
	CSSM_ATTRIBUTE_DATA_KEY = $40000000;
	CSSM_ATTRIBUTE_DATA_STRING = $50000000;
	CSSM_ATTRIBUTE_DATA_DATE = $60000000;
	CSSM_ATTRIBUTE_DATA_RANGE = $70000000;
	CSSM_ATTRIBUTE_DATA_ACCESS_CREDENTIALS = $80000000;
	CSSM_ATTRIBUTE_DATA_VERSION = $01000000;
	CSSM_ATTRIBUTE_DATA_DL_DB_HANDLE = $02000000;
	CSSM_ATTRIBUTE_DATA_KR_PROFILE = $03000000;
	CSSM_ATTRIBUTE_TYPE_MASK = $FF000000;

type
	CSSM_ATTRIBUTE_TYPE = UInt32;
const
	CSSM_ATTRIBUTE_NONE = 0;
	CSSM_ATTRIBUTE_CUSTOM = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 1;
	CSSM_ATTRIBUTE_DESCRIPTION = CSSM_ATTRIBUTE_DATA_STRING or 2;
	CSSM_ATTRIBUTE_KEY = CSSM_ATTRIBUTE_DATA_KEY or 3;
	CSSM_ATTRIBUTE_INIT_VECTOR = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 4;
	CSSM_ATTRIBUTE_SALT = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 5;
	CSSM_ATTRIBUTE_PADDING = CSSM_ATTRIBUTE_DATA_UINT32 or 6;
	CSSM_ATTRIBUTE_RANDOM = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 7;
	CSSM_ATTRIBUTE_SEED = CSSM_ATTRIBUTE_DATA_CRYPTO_DATA or 8;
	CSSM_ATTRIBUTE_PASSPHRASE = CSSM_ATTRIBUTE_DATA_CRYPTO_DATA or 9;
	CSSM_ATTRIBUTE_KEY_LENGTH = CSSM_ATTRIBUTE_DATA_UINT32 or 10;
	CSSM_ATTRIBUTE_KEY_LENGTH_RANGE = CSSM_ATTRIBUTE_DATA_RANGE or 11;
	CSSM_ATTRIBUTE_BLOCK_SIZE = CSSM_ATTRIBUTE_DATA_UINT32 or 12;
	CSSM_ATTRIBUTE_OUTPUT_SIZE = CSSM_ATTRIBUTE_DATA_UINT32 or 13;
	CSSM_ATTRIBUTE_ROUNDS = CSSM_ATTRIBUTE_DATA_UINT32 or 14;
	CSSM_ATTRIBUTE_IV_SIZE = CSSM_ATTRIBUTE_DATA_UINT32 or 15;
	CSSM_ATTRIBUTE_ALG_PARAMS = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 16;
	CSSM_ATTRIBUTE_LABEL = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 17;
	CSSM_ATTRIBUTE_KEY_TYPE = CSSM_ATTRIBUTE_DATA_UINT32 or 18;
	CSSM_ATTRIBUTE_MODE = CSSM_ATTRIBUTE_DATA_UINT32 or 19;
	CSSM_ATTRIBUTE_EFFECTIVE_BITS = CSSM_ATTRIBUTE_DATA_UINT32 or 20;
	CSSM_ATTRIBUTE_START_DATE = CSSM_ATTRIBUTE_DATA_DATE or 21;
	CSSM_ATTRIBUTE_END_DATE = CSSM_ATTRIBUTE_DATA_DATE or 22;
	CSSM_ATTRIBUTE_KEYUSAGE = CSSM_ATTRIBUTE_DATA_UINT32 or 23;
	CSSM_ATTRIBUTE_KEYATTR = CSSM_ATTRIBUTE_DATA_UINT32 or 24;
	CSSM_ATTRIBUTE_VERSION = CSSM_ATTRIBUTE_DATA_VERSION or 25;
	CSSM_ATTRIBUTE_PRIME = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 26;
	CSSM_ATTRIBUTE_BASE = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 27;
	CSSM_ATTRIBUTE_SUBPRIME = CSSM_ATTRIBUTE_DATA_CSSM_DATA or 28;
	CSSM_ATTRIBUTE_ALG_ID = CSSM_ATTRIBUTE_DATA_UINT32 or 29;
	CSSM_ATTRIBUTE_ITERATION_COUNT = CSSM_ATTRIBUTE_DATA_UINT32 or 30;
	CSSM_ATTRIBUTE_ROUNDS_RANGE = CSSM_ATTRIBUTE_DATA_RANGE or 31;
	CSSM_ATTRIBUTE_KRPROFILE_LOCAL = CSSM_ATTRIBUTE_DATA_KR_PROFILE or 32;
	CSSM_ATTRIBUTE_KRPROFILE_REMOTE = CSSM_ATTRIBUTE_DATA_KR_PROFILE or 33;
	CSSM_ATTRIBUTE_CSP_HANDLE = CSSM_ATTRIBUTE_DATA_UINT32 or 34;
	CSSM_ATTRIBUTE_DL_DB_HANDLE = CSSM_ATTRIBUTE_DATA_DL_DB_HANDLE or 35;
	CSSM_ATTRIBUTE_ACCESS_CREDENTIALS = CSSM_ATTRIBUTE_DATA_ACCESS_CREDENTIALS or 36;
	CSSM_ATTRIBUTE_PUBLIC_KEY_FORMAT = CSSM_ATTRIBUTE_DATA_UINT32 or 37;
	CSSM_ATTRIBUTE_PRIVATE_KEY_FORMAT = CSSM_ATTRIBUTE_DATA_UINT32 or 38;
	CSSM_ATTRIBUTE_SYMMETRIC_KEY_FORMAT = CSSM_ATTRIBUTE_DATA_UINT32 or 39;
	CSSM_ATTRIBUTE_WRAPPED_KEY_FORMAT = CSSM_ATTRIBUTE_DATA_UINT32 or 40;

type
	CSSM_PADDING = UInt32;
const
	CSSM_PADDING_NONE = 0;
	CSSM_PADDING_CUSTOM = CSSM_PADDING_NONE + 1;
	CSSM_PADDING_ZERO = CSSM_PADDING_NONE + 2;
	CSSM_PADDING_ONE = CSSM_PADDING_NONE + 3;
	CSSM_PADDING_ALTERNATE = CSSM_PADDING_NONE + 4;
	CSSM_PADDING_FF = CSSM_PADDING_NONE + 5;
	CSSM_PADDING_PKCS5 = CSSM_PADDING_NONE + 6;
	CSSM_PADDING_PKCS7 = CSSM_PADDING_NONE + 7;
	CSSM_PADDING_CIPHERSTEALING = CSSM_PADDING_NONE + 8;
	CSSM_PADDING_RANDOM = CSSM_PADDING_NONE + 9;
	CSSM_PADDING_PKCS1 = CSSM_PADDING_NONE + 10;
{ All padding types that are vendor specific, and not
   part of the CSSM specification should be defined
   relative to CSSM_PADDING_VENDOR_DEFINED. }
	CSSM_PADDING_VENDOR_DEFINED = CSSM_PADDING_NONE + $80000000;

type
	CSSM_KEY_TYPE = CSSM_ALGORITHMS;

type
  __embedded_cssm_context_attribute = record
		case Integer of
		  0: (String_: CStringPtr);
		  1: (Uint32: UInt32_fix);
  		2: (AccessCredentials: CSSM_ACCESS_CREDENTIALS_PTR);
		  3: (Key: CSSM_KEY_PTR);
		  4: (Data: CSSM_DATA_PTR);
		  5: (Padding: CSSM_PADDING);
		  6: (Date: CSSM_DATE_PTR);
		  7: (Range: CSSM_RANGE_PTR);
		  8: (CryptoData: CSSM_CRYPTO_DATA_PTR);
  		9: (Version: CSSM_VERSION_PTR);
		  10: (DLDBHandle: CSSM_DL_DB_HANDLE_PTR);
  		11: (KRProfile: UnivPtr {cssm_kr_profile_ptr});
  end;
  
  CSSM_CONTEXT_ATTRIBUTE_PTR = ^cssm_context_attribute;
  CSSM_CONTEXT_ATTRIBUTEPtr = ^cssm_context_attribute;
	cssm_context_attribute = record
		AttributeType: CSSM_ATTRIBUTE_TYPE;
		AttributeLength: UInt32;
		Attribute: __embedded_cssm_context_attribute;
	end;

type
	CSSM_CONTEXT_PTR = ^cssm_context;
	CSSM_CONTEXTPtr = ^cssm_context;
	cssm_context = record
		ContextType: CSSM_CONTEXT_TYPE;
		AlgorithmType: CSSM_ALGORITHMS;
		NumberOfAttributes: UInt32;
		ContextAttributes: CSSM_CONTEXT_ATTRIBUTE_PTR;
		CSPHandle: CSSM_CSP_HANDLE;
		Privileged: CSSM_BOOL;
	  EncryptionProhibited: uint32 {CSSM_KR_POLICY_FLAGS} ;
		WorkFactor: UInt32;
		Reserved: UInt32;	{ reserved for future use }
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
	CSSM_SC_FLAGS = UInt32;
const
	CSSM_CSP_TOK_RNG = $00000001;
	CSSM_CSP_TOK_CLOCK_EXISTS = $00000040;

type
	CSSM_CSP_READER_FLAGS = UInt32;
const
	CSSM_CSP_RDR_TOKENPRESENT = $00000001;
	{ Token is present in reader/slot }
	CSSM_CSP_RDR_EXISTS = $00000002;
	{ Device is a reader with a
	   removable token }
	CSSM_CSP_RDR_HW = $00000004;
	{ Slot is a hardware slot }

type
	CSSM_CSP_FLAGS = UInt32;
const
	CSSM_CSP_TOK_WRITE_PROTECTED = $00000002;
	CSSM_CSP_TOK_LOGIN_REQUIRED = $00000004;
	CSSM_CSP_TOK_USER_PIN_INITIALIZED = $00000008;
	CSSM_CSP_TOK_PROT_AUTHENTICATION = $00000100;
	CSSM_CSP_TOK_USER_PIN_EXPIRED = $00100000;
	CSSM_CSP_TOK_SESSION_KEY_PASSWORD = $00200000;
	CSSM_CSP_TOK_PRIVATE_KEY_PASSWORD = $00400000;
	CSSM_CSP_STORES_PRIVATE_KEYS = $01000000;
	CSSM_CSP_STORES_PUBLIC_KEYS = $02000000;
	CSSM_CSP_STORES_SESSION_KEYS = $04000000;
	CSSM_CSP_STORES_CERTIFICATES = $08000000;
	CSSM_CSP_STORES_GENERIC = $10000000;

type
	CSSM_PKCS_OAEP_MGF = UInt32;
const
	CSSM_PKCS_OAEP_MGF_NONE = 0;
	CSSM_PKCS_OAEP_MGF1_SHA1 = CSSM_PKCS_OAEP_MGF_NONE + 1;
	CSSM_PKCS_OAEP_MGF1_MD5 = CSSM_PKCS_OAEP_MGF_NONE + 2;

type
	CSSM_PKCS_OAEP_PSOURCE = UInt32;
const
	CSSM_PKCS_OAEP_PSOURCE_NONE = 0;
	CSSM_PKCS_OAEP_PSOURCE_Pspecified = CSSM_PKCS_OAEP_PSOURCE_NONE + 1;

type
	CSSM_PKCS1_OAEP_PARAMS_PTR = ^cssm_pkcs1_oaep_params;
	CSSM_PKCS1_OAEP_PARAMSPtr = ^cssm_pkcs1_oaep_params;
	cssm_pkcs1_oaep_params = record
		HashAlgorithm: UInt32;
		HashParams: CSSM_DATA;
		MGF: CSSM_PKCS_OAEP_MGF;
		MGFParams: CSSM_DATA;
		PSource: CSSM_PKCS_OAEP_PSOURCE;
		PSourceParams: CSSM_DATA;
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
	CSSM_CSP_OPERATIONAL_STATISTICS_PTR = ^cssm_csp_operational_statistics;
	CSSM_CSP_OPERATIONAL_STATISTICSPtr = ^cssm_csp_operational_statistics;
	cssm_csp_operational_statistics = record
		UserAuthenticated: CSSM_BOOL;
    { CSSM_TRUE if the user is logged in to the token, CSSM_FALSE otherwise. }
		DeviceFlags: CSSM_CSP_FLAGS;
		TokenMaxSessionCount: UInt32; { Exported by Cryptoki modules. }
		TokenOpenedSessionCount: UInt32;
		TokenMaxRWSessionCount: UInt32;
		TokenOpenedRWSessionCount: UInt32;
		TokenTotalPublicMem: UInt32; { Storage space statistics. }
		TokenFreePublicMem: UInt32;
		TokenTotalPrivateMem: UInt32;
		TokenFreePrivateMem: UInt32;
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

{ Indicates that the statistical value can not be revealed or is not
   relevant for a CSP }
const
	CSSM_VALUE_NOT_AVAILABLE = $FFFFFFFF;

type
	CSSM_PKCS5_PBKDF1_PARAMS_PTR = ^cssm_pkcs5_pbkdf1_params;
	CSSM_PKCS5_PBKDF1_PARAMSPtr = ^cssm_pkcs5_pbkdf1_params;
	cssm_pkcs5_pbkdf1_params = record
		Passphrase: CSSM_DATA;
		InitVector: CSSM_DATA;
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
	CSSM_PKCS5_PBKDF2_PRF = UInt32;
const
	CSSM_PKCS5_PBKDF2_PRF_HMAC_SHA1 = 0;

type
	CSSM_PKCS5_PBKDF2_PARAMS_PTR = ^cssm_pkcs5_pbkdf2_params;
	CSSM_PKCS5_PBKDF2_PARAMSPtr = ^cssm_pkcs5_pbkdf2_params;
	cssm_pkcs5_pbkdf2_params = record
		Passphrase: CSSM_DATA;
		PseudoRandomFunction: CSSM_PKCS5_PBKDF2_PRF;
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
	CSSM_KEA_DERIVE_PARAMS_PTR = ^cssm_kea_derive_params;
	CSSM_KEA_DERIVE_PARAMSPtr = ^cssm_kea_derive_params;
	cssm_kea_derive_params = record
		Rb: CSSM_DATA;
		Yb: CSSM_DATA;
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


{ Data Types for Trust Policy Services  }

type
	CSSM_TP_AUTHORITY_ID_PTR = ^cssm_tp_authority_id;
	CSSM_TP_AUTHORITY_IDPtr = ^cssm_tp_authority_id;
	cssm_tp_authority_id = record
		AuthorityCert: CSSM_DATAPtr;
		AuthorityLocation: CSSM_NET_ADDRESS_PTR;
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
	CSSM_TP_AUTHORITY_REQUEST_TYPE = UInt32;
	CSSM_TP_AUTHORITY_REQUEST_TYPE_PTR = ^UInt32;
	CSSM_TP_AUTHORITY_REQUEST_TYPEPtr = ^UInt32;
const
	CSSM_TP_AUTHORITY_REQUEST_CERTISSUE = $01;
	CSSM_TP_AUTHORITY_REQUEST_CERTREVOKE = $02;
	CSSM_TP_AUTHORITY_REQUEST_CERTSUSPEND = $03;
	CSSM_TP_AUTHORITY_REQUEST_CERTRESUME = $04;
	CSSM_TP_AUTHORITY_REQUEST_CERTVERIFY = $05;
	CSSM_TP_AUTHORITY_REQUEST_CERTNOTARIZE = $06;
	CSSM_TP_AUTHORITY_REQUEST_CERTUSERECOVER = $07;
	CSSM_TP_AUTHORITY_REQUEST_CRLISSUE = $100;

type
	CSSM_TP_VERIFICATION_RESULTS_CALLBACK = function( ModuleHandle: CSSM_MODULE_HANDLE; CallerCtx: UnivPtr; VerifiedCert: CSSM_DATA_PTR ): CSSM_RETURN;

{ From CL }
type
	CSSM_OID = CSSM_DATA;
	CSSM_OID_PTR = ^CSSM_DATA;
	CSSM_OIDPtr = ^CSSM_DATA;

type
	CSSM_FIELD_PTR = ^cssm_field;
	CSSM_FIELDPtr = ^cssm_field;
	cssm_field = record
		FieldOid: CSSM_OID;
		FieldValue: CSSM_DATA;
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

{ TP Again. }
type
	CSSM_TP_POLICYINFO_PTR = ^cssm_tp_policyinfo;
	CSSM_TP_POLICYINFOPtr = ^cssm_tp_policyinfo;
	cssm_tp_policyinfo = record
		NumberOfPolicyIds: UInt32;
		PolicyIds: CSSM_FIELD_PTR;
		PolicyControl: UnivPtr;
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
	CSSM_TP_SERVICES = UInt32;
const
{ bit masks for additional Authority services available through TP }
	CSSM_TP_KEY_ARCHIVE = $0001; { archive cert & keys }
	CSSM_TP_CERT_PUBLISH = $0002; { register cert in directory }
	CSSM_TP_CERT_NOTIFY_RENEW = $0004; { notify at renewal time }
	CSSM_TP_CERT_DIR_UPDATE = $0008; { update cert registry entry }
	CSSM_TP_CRL_DISTRIBUTE = $0010; { push CRL to everyone }

type
	CSSM_TP_ACTION = UInt32;
const
	CSSM_TP_ACTION_DEFAULT = 0;

type
	CSSM_TP_STOP_ON = UInt32;
const
	CSSM_TP_STOP_ON_POLICY = 0; { use the pre-defined stopping criteria }
	CSSM_TP_STOP_ON_NONE = 1; { evaluate all condition whether TRUE or FALSE }
	CSSM_TP_STOP_ON_FIRST_PASS = 2; { stop evaluation at first TRUE }
	CSSM_TP_STOP_ON_FIRST_FAIL = 3; { stop evaluation at first FALSE }

type
	CSSM_TIMESTRING = CStringPtr;

{ From DL. }
type
	CSSM_DL_DB_LIST_PTR = ^cssm_dl_db_list;
	CSSM_DL_DB_LISTPtr = ^cssm_dl_db_list;
	cssm_dl_db_list = record
		NumHandles: UInt32;
		DLDBHandle: CSSM_DL_DB_HANDLE_PTR;
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

{ TP Again. }
type
	CSSM_TP_CALLERAUTH_CONTEXT_PTR = ^cssm_tp_callerauth_context;
	CSSM_TP_CALLERAUTH_CONTEXTPtr = ^cssm_tp_callerauth_context;
	cssm_tp_callerauth_context = record
		Policy: CSSM_TP_POLICYINFO;
		VerifyTime: CSSM_TIMESTRING;
		VerificationAbortOn: CSSM_TP_STOP_ON;
		CallbackWithVerifiedCert: CSSM_TP_VERIFICATION_RESULTS_CALLBACK;
		NumberOfAnchorCerts: UInt32;
		AnchorCerts: CSSM_DATA_PTR;
		DBList: CSSM_DL_DB_LIST_PTR;
		CallerCredentials: CSSM_ACCESS_CREDENTIALS_PTR;
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
	CSSM_CRL_PARSE_FORMAT = UInt32;
	CSSM_CRL_PARSE_FORMAT_PTR = ^UInt32;
	CSSM_CRL_PARSE_FORMATPtr = ^UInt32;
const
	CSSM_CRL_PARSE_FORMAT_NONE = $00;
	CSSM_CRL_PARSE_FORMAT_CUSTOM = $01;
	CSSM_CRL_PARSE_FORMAT_SEXPR = $02;
	CSSM_CRL_PARSE_FORMAT_COMPLEX = $03;
	CSSM_CRL_PARSE_FORMAT_OID_NAMED = $04;
	CSSM_CRL_PARSE_FORMAT_TUPLE = $05;
	CSSM_CRL_PARSE_FORMAT_MULTIPLE = $7FFE;
	CSSM_CRL_PARSE_FORMAT_LAST = $7FFF;
	{ Applications wishing to define their own custom parse
	   format should create a uint32 value greater than the
	   CSSM_CL_CUSTOM_CRL_PARSE_FORMAT }
	CSSM_CL_CUSTOM_CRL_PARSE_FORMAT = $8000;

{ From CL. }
type
	CSSM_CRL_TYPE = UInt32;
	CSSM_CRL_TYPE_PTR = ^UInt32;
	CSSM_CRL_TYPEPtr = ^UInt32;
const
	CSSM_CRL_TYPE_UNKNOWN = $00;
	CSSM_CRL_TYPE_X_509v1 = $01;
	CSSM_CRL_TYPE_X_509v2 = $02;
	CSSM_CRL_TYPE_SPKI = $03;
	CSSM_CRL_TYPE_MULTIPLE = $7FFE;

type
	CSSM_CRL_ENCODING = UInt32;
	CSSM_CRL_ENCODING_PTR = ^UInt32;
	CSSM_CRL_ENCODINGPtr = ^UInt32;
const
	CSSM_CRL_ENCODING_UNKNOWN = $00;
	CSSM_CRL_ENCODING_CUSTOM = $01;
	CSSM_CRL_ENCODING_BER = $02;
	CSSM_CRL_ENCODING_DER = $03;
	CSSM_CRL_ENCODING_BLOOM = $04;
	CSSM_CRL_ENCODING_SEXPR = $05;
	CSSM_CRL_ENCODING_MULTIPLE = $7FFE;

type
	CSSM_ENCODED_CRL_PTR = ^cssm_encoded_crl;
	CSSM_ENCODED_CRLPtr = ^cssm_encoded_crl;
	cssm_encoded_crl = record
		CrlType: CSSM_CRL_TYPE; { type of CRL }
		CrlEncoding: CSSM_CRL_ENCODING; { encoding for this packed CRL }
		CrlBlob: CSSM_DATA; { packed CRL }
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

{ TP Again. }
type
	CSSM_PARSED_CRL_PTR = ^cssm_parsed_crl;
	CSSM_PARSED_CRLPtr = ^cssm_parsed_crl;
	cssm_parsed_crl = record
		CrlType: CSSM_CRL_TYPE; { CRL type }
		ParsedCrlFormat: CSSM_CRL_PARSE_FORMAT;
    { struct of ParsedCrl }
		ParsedCrl: UnivPtr; { parsed CRL (to be typecast) }
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
	CSSM_CRL_PAIR_PTR = ^cssm_crl_pair;
	CSSM_CRL_PAIRPtr = ^cssm_crl_pair;
	cssm_crl_pair = record
		EncodedCrl: CSSM_ENCODED_CRL; { an encoded CRL blob }
		ParsedCrl: CSSM_PARSED_CRL; { equivalent parsed CRL }
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
	CSSM_CRLGROUP_TYPE = UInt32;
	CSSM_CRLGROUP_TYPE_PTR = ^UInt32;
	CSSM_CRLGROUP_TYPEPtr = ^UInt32;
const
	CSSM_CRLGROUP_DATA = $00;
	CSSM_CRLGROUP_ENCODED_CRL = $01;
	CSSM_CRLGROUP_PARSED_CRL = $02;
	CSSM_CRLGROUP_CRL_PAIR = $03;

type
  __embedded_cssm_crlgroup = record
    case Integer of
		  0: (CrlList: CSSM_DATA_PTR);					{ CRL blob }
		  1: (EncodedCrlList: CSSM_ENCODED_CRL_PTR);	{ CRL blob w/ separate type }
		  2: (ParsedCrlList: CSSM_PARSED_CRL_PTR);		{ bushy, parsed CRL }
		  3: (PairCrlList: CSSM_CRL_PAIR_PTR);
  end;

  CSSM_CRLGROUP_PTR = ^cssm_crlgroup;
  CSSM_CRLGROUPPtr = ^cssm_crlgroup;
	cssm_crlgroup = record
		CrlType: CSSM_CRL_TYPE;
		CrlEncoding: CSSM_CRL_ENCODING;
		NumberOfCrls: UInt32;
		GroupCrlList: __embedded_cssm_crlgroup;
	end;

type
	CSSM_FIELDGROUP_PTR = ^cssm_fieldgroup;
	CSSM_FIELDGROUPPtr = ^cssm_fieldgroup;
	cssm_fieldgroup = record
		NumberOfFields: SInt32;		{ number of fields in the array }
		Fields: CSSM_FIELD_PTR;	{ array of fields }
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
	CSSM_EVIDENCE_FORM = UInt32;
const
	CSSM_EVIDENCE_FORM_UNSPECIFIC = $0;
	CSSM_EVIDENCE_FORM_CERT = $1;
	CSSM_EVIDENCE_FORM_CRL = $2;
	CSSM_EVIDENCE_FORM_CERT_ID = $3;
	CSSM_EVIDENCE_FORM_CRL_ID = $4;
	CSSM_EVIDENCE_FORM_VERIFIER_TIME = $5;
	CSSM_EVIDENCE_FORM_CRL_THISTIME = $6;
	CSSM_EVIDENCE_FORM_CRL_NEXTTIME = $7;
	CSSM_EVIDENCE_FORM_POLICYINFO = $8;
	CSSM_EVIDENCE_FORM_TUPLEGROUP = $9;

type
	CSSM_EVIDENCE_PTR = ^cssm_evidence;
	CSSM_EVIDENCEPtr = ^cssm_evidence;
	cssm_evidence = record
		EvidenceForm: CSSM_EVIDENCE_FORM;
		Evidence: UnivPtr; { Evidence content }
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
	CSSM_TP_VERIFY_CONTEXT_PTR = ^cssm_tp_verify_context;
	CSSM_TP_VERIFY_CONTEXTPtr = ^cssm_tp_verify_context;
	cssm_tp_verify_context = record
		Action: CSSM_TP_ACTION;
		ActionData: CSSM_DATA;
		Crls: CSSM_CRLGROUP;
		Cred: CSSM_TP_CALLERAUTH_CONTEXT_PTR;
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
	CSSM_TP_VERIFY_CONTEXT_RESULT_PTR = ^cssm_tp_verify_context_result;
	CSSM_TP_VERIFY_CONTEXT_RESULTPtr = ^cssm_tp_verify_context_result;
	cssm_tp_verify_context_result = record
		NumberOfEvidences: UInt32;
		Evidence: CSSM_EVIDENCE_PTR;
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
	CSSM_TP_REQUEST_SET_PTR = ^cssm_tp_request_set;
	CSSM_TP_REQUEST_SETPtr = ^cssm_tp_request_set;
	cssm_tp_request_set = record
		NumberOfRequests: UInt32;
		Requests: UnivPtr;
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
	CSSM_TP_RESULT_SET_PTR = ^cssm_tp_result_set;
	CSSM_TP_RESULT_SETPtr = ^cssm_tp_result_set;
	cssm_tp_result_set = record
		NumberOfResults: UInt32;
		Results: UnivPtr;
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
	CSSM_TP_CONFIRM_STATUS = UInt32;
	CSSM_TP_CONFIRM_STATUS_PTR = ^UInt32;
	CSSM_TP_CONFIRM_STATUSPtr = ^UInt32;
const
	CSSM_TP_CONFIRM_STATUS_UNKNOWN = $0;
	{ indeterminate }
	CSSM_TP_CONFIRM_ACCEPT = $1;
	{ accept results of executing a
	   submit-retrieve function pair }
	CSSM_TP_CONFIRM_REJECT = $2;
	{ reject results of executing a
	   submit-retrieve function pair }

type
	CSSM_TP_CONFIRM_RESPONSE_PTR = ^cssm_tp_confirm_response;
	CSSM_TP_CONFIRM_RESPONSEPtr = ^cssm_tp_confirm_response;
	cssm_tp_confirm_response = record
		NumberOfResponses: UInt32;
		Responses: CSSM_TP_CONFIRM_STATUS_PTR;
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

const
	CSSM_ESTIMATED_TIME_UNKNOWN = -1;

const
	CSSM_ELAPSED_TIME_UNKNOWN = -1;
	CSSM_ELAPSED_TIME_COMPLETE = -2;

type
	CSSM_TP_CERTISSUE_INPUT_PTR = ^cssm_tp_certissue_input;
	CSSM_TP_CERTISSUE_INPUTPtr = ^cssm_tp_certissue_input;
	cssm_tp_certissue_input = record
		CSPSubserviceUid: CSSM_SUBSERVICE_UID;
		CLHandle: CSSM_CL_HANDLE;
		NumberOfTemplateFields: UInt32;
		SubjectCertFields: CSSM_FIELD_PTR;
		MoreServiceRequests: CSSM_TP_SERVICES;
		NumberOfServiceControls: UInt32;
		ServiceControls: CSSM_FIELD_PTR;
		UserCredentials: CSSM_ACCESS_CREDENTIALS_PTR;
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
	CSSM_TP_CERTISSUE_STATUS = UInt32;
const
	CSSM_TP_CERTISSUE_STATUS_UNKNOWN = $0;
	{ indeterminate }
	CSSM_TP_CERTISSUE_OK = $1;
	{ cert issued as requested }
	CSSM_TP_CERTISSUE_OKWITHCERTMODS = $2;
	{ cert issued but cert contents were
	   updated by the issuing authority }
	CSSM_TP_CERTISSUE_OKWITHSERVICEMODS = $3;
	{ cert issued but some requested backend
	   services were not performed by the
	   issuing authority }
	CSSM_TP_CERTISSUE_REJECTED = $4;
	{ cert was not issued due to some error
	   condition }
	CSSM_TP_CERTISSUE_NOT_AUTHORIZED = $5;
	{ cert was not issued, the request was
	   not authorized }
	CSSM_TP_CERTISSUE_WILL_BE_REVOKED = $6;
	{ cert was issued, but TP has initiated
	   a revocation of the certificate }

type
	CSSM_TP_CERTISSUE_OUTPUT_PTR = ^cssm_tp_certissue_output;
	CSSM_TP_CERTISSUE_OUTPUTPtr = ^cssm_tp_certissue_output;
	cssm_tp_certissue_output = record
		IssueStatus: CSSM_TP_CERTISSUE_STATUS;
		CertGroup: CSSM_CERTGROUP_PTR;
		PerformedServiceRequests: CSSM_TP_SERVICES;
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
	CSSM_TP_CERTCHANGE_ACTION = UInt32;
const
	CSSM_TP_CERTCHANGE_NONE = $0; { no change }
	CSSM_TP_CERTCHANGE_REVOKE = $1; { Revoke the certificate }
{ This action type indicates a request to revoke a single
   certificate. Notice of the revocation operation remains
   in affect until the certificate itself expires. Revocation
   should be used to permanently remove a certificate from use. }
	CSSM_TP_CERTCHANGE_HOLD = $2; { Hold/suspend the certificate }
{ This action type indicates a request to suspend a
   single certificate. A suspension operation implies
   that the requester intends, at some time in the future,
   to request that the certificate be released from hold,
   making it available for use again. Placing a hold on
   a certificate does not obligate the requester to
   request a release. In practice, a certificate may
   remain on hold until the certificate itself expires.
   Revocation should be used to permanently remove a
   certificate from use. }
	CSSM_TP_CERTCHANGE_RELEASE = $3; { Release the held certificate }
{ This action type indicates a request to release a
   single certificate currently on hold. A release
   operation makes a certificate available for use again.
   Revocation should be used to permanently remove a
   certificate from use. }

type
	CSSM_TP_CERTCHANGE_REASON = UInt32;
const
	CSSM_TP_CERTCHANGE_REASON_UNKNOWN = $0;
	{ unspecified }
	CSSM_TP_CERTCHANGE_REASON_KEYCOMPROMISE = $1;
	{ Subject key believed to be compromised }
	CSSM_TP_CERTCHANGE_REASON_CACOMPROMISE = $2;
	{ CA’s key believed to be compromised }
	CSSM_TP_CERTCHANGE_REASON_CEASEOPERATION = $3;
	{ certificate holder ceases operation under
	   the jurisdiction of this certificate }
	CSSM_TP_CERTCHANGE_REASON_AFFILIATIONCHANGE = $4;
	{ certificate holder has moved from this
	   jurisdiction }
	CSSM_TP_CERTCHANGE_REASON_SUPERCEDED = $5;
	{ certificate holder as issued a new, superceding
	   certificate }
	CSSM_TP_CERTCHANGE_REASON_SUSPECTEDCOMPROMISE = $6;
	{ certificate could be compromised }
	CSSM_TP_CERTCHANGE_REASON_HOLDRELEASE = $7;
	{ certificate holder resumes operation under the
	   jurisdiction of this certificate }

type
	CSSM_TP_CERTCHANGE_INPUT_PTR = ^cssm_tp_certchange_input;
	CSSM_TP_CERTCHANGE_INPUTPtr = ^cssm_tp_certchange_input;
	cssm_tp_certchange_input = record
		Action: CSSM_TP_CERTCHANGE_ACTION;
		Reason: CSSM_TP_CERTCHANGE_REASON;
		CLHandle: CSSM_CL_HANDLE;
		Cert: CSSM_DATA_PTR;
		ChangeInfo: CSSM_FIELD_PTR;
		StartTime: CSSM_TIMESTRING;
		CallerCredentials: CSSM_ACCESS_CREDENTIALS_PTR;
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
	CSSM_TP_CERTCHANGE_STATUS = UInt32;
const
	CSSM_TP_CERTCHANGE_STATUS_UNKNOWN = $0;
	{ indeterminate }
	CSSM_TP_CERTCHANGE_OK = $1;
	{ cert state was successfully changed
	   beginning at the specified time }
	CSSM_TP_CERTCHANGE_OKWITHNEWTIME = $2;
	{ cert state was successfully changed,
	   at a modified effective time }
	CSSM_TP_CERTCHANGE_WRONGCA = $3;
	{ cert state was not changed, the
	   selected CA is not authorized to
	   change the cert state }
	CSSM_TP_CERTCHANGE_REJECTED = $4;
	{ cert state was not changed due to some
	   error condition }
	CSSM_TP_CERTCHANGE_NOT_AUTHORIZED = $5;
	{ cert state was not changed, the
	   requester is not authorized to change
	   the cert state }

type
	CSSM_TP_CERTCHANGE_OUTPUT_PTR = ^cssm_tp_certchange_output;
	CSSM_TP_CERTCHANGE_OUTPUTPtr = ^cssm_tp_certchange_output;
	cssm_tp_certchange_output = record
		ActionStatus: CSSM_TP_CERTCHANGE_STATUS;
		RevokeInfo: CSSM_FIELD;
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
	CSSM_TP_CERTVERIFY_INPUT_PTR = ^cssm_tp_certverify_input;
	CSSM_TP_CERTVERIFY_INPUTPtr = ^cssm_tp_certverify_input;
	cssm_tp_certverify_input = record
		CLHandle: CSSM_CL_HANDLE;
		Cert: CSSM_DATA_PTR;
		VerifyContext: CSSM_TP_VERIFY_CONTEXT_PTR;
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
	CSSM_TP_CERTVERIFY_STATUS = UInt32;
const
	CSSM_TP_CERTVERIFY_UNKNOWN = $0;
	CSSM_TP_CERTVERIFY_VALID = $1;
	CSSM_TP_CERTVERIFY_INVALID = $2;
	CSSM_TP_CERTVERIFY_REVOKED = $3;
	CSSM_TP_CERTVERIFY_SUSPENDED = $4;
	CSSM_TP_CERTVERIFY_EXPIRED = $5;
	CSSM_TP_CERTVERIFY_NOT_VALID_YET = $6;
	CSSM_TP_CERTVERIFY_INVALID_AUTHORITY = $7;
	CSSM_TP_CERTVERIFY_INVALID_SIGNATURE = $8;
	CSSM_TP_CERTVERIFY_INVALID_CERT_VALUE = $9;
	CSSM_TP_CERTVERIFY_INVALID_CERTGROUP = $A;
	CSSM_TP_CERTVERIFY_INVALID_POLICY = $B;
	CSSM_TP_CERTVERIFY_INVALID_POLICY_IDS = $C;
	CSSM_TP_CERTVERIFY_INVALID_BASIC_CONSTRAINTS = $D;
	CSSM_TP_CERTVERIFY_INVALID_CRL_DIST_PT = $E;
	CSSM_TP_CERTVERIFY_INVALID_NAME_TREE = $F;
	CSSM_TP_CERTVERIFY_UNKNOWN_CRITICAL_EXT = $10;

type
	CSSM_TP_CERTVERIFY_OUTPUT_PTR = ^cssm_tp_certverify_output;
	CSSM_TP_CERTVERIFY_OUTPUTPtr = ^cssm_tp_certverify_output;
	cssm_tp_certverify_output = record
		VerifyStatus: CSSM_TP_CERTVERIFY_STATUS;
		NumberOfEvidence: UInt32;
		Evidence: CSSM_EVIDENCE_PTR;
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
	CSSM_TP_CERTNOTARIZE_INPUT_PTR = ^cssm_tp_certnotarize_input;
	CSSM_TP_CERTNOTARIZE_INPUTPtr = ^cssm_tp_certnotarize_input;
	cssm_tp_certnotarize_input = record
		CLHandle: CSSM_CL_HANDLE;
		NumberOfFields: UInt32;
		MoreFields: CSSM_FIELD_PTR;
		SignScope: CSSM_FIELD_PTR;
		ScopeSize: UInt32;
		MoreServiceRequests: CSSM_TP_SERVICES;
		NumberOfServiceControls: UInt32;
		ServiceControls: CSSM_FIELD_PTR;
		UserCredentials: CSSM_ACCESS_CREDENTIALS_PTR;
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
	CSSM_TP_CERTNOTARIZE_STATUS = UInt32;
const
	CSSM_TP_CERTNOTARIZE_STATUS_UNKNOWN = $0;
	{ indeterminate }
	CSSM_TP_CERTNOTARIZE_OK = $1;
	{ cert fields were added and the result was
	   notarized as requested }
	CSSM_TP_CERTNOTARIZE_OKWITHOUTFIELDS = $2;
	{ non-conflicting cert fields were added,
	   conflicting cert fields were ignored,
	   and the result was notarized as requested }
	CSSM_TP_CERTNOTARIZE_OKWITHSERVICEMODS = $3;
	{ cert fields were added and the result was
	   notarized as requested, but some requested
	   backend services were not performed by the
	   notary }
	CSSM_TP_CERTNOTARIZE_REJECTED = $4;
	{ cert was not notarized due to some error
	   condition }
	CSSM_TP_CERTNOTARIZE_NOT_AUTHORIZED = $5;
	{ cert was not notarized, the request was
	   not authorized }

type
	CSSM_TP_CERTNOTARIZE_OUTPUT_PTR = ^cssm_tp_certnotarize_output;
	CSSM_TP_CERTNOTARIZE_OUTPUTPtr = ^cssm_tp_certnotarize_output;
	cssm_tp_certnotarize_output = record
		NotarizeStatus: CSSM_TP_CERTNOTARIZE_STATUS;
		NotarizedCertGroup: CSSM_CERTGROUP_PTR;
		PerformedServiceRequests: CSSM_TP_SERVICES;
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
	CSSM_TP_CERTRECLAIM_INPUT_PTR = ^cssm_tp_certreclaim_input;
	CSSM_TP_CERTRECLAIM_INPUTPtr = ^cssm_tp_certreclaim_input;
	cssm_tp_certreclaim_input = record
		CLHandle: CSSM_CL_HANDLE;
		NumberOfSelectionFields: UInt32;
		SelectionFields: CSSM_FIELD_PTR;
		UserCredentials: CSSM_ACCESS_CREDENTIALS_PTR;
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
	CSSM_TP_CERTRECLAIM_STATUS = UInt32;
const
	CSSM_TP_CERTRECLAIM_STATUS_UNKNOWN = $0;
	{ indeterminate }
	CSSM_TP_CERTRECLAIM_OK = $1;
	{ a set of one or more certificates were
	   returned by the CA for local recovery
	   of the associated private key }
	CSSM_TP_CERTRECLAIM_NOMATCH = $2;
	{ no certificates owned by the requester
	   were found matching the specified
	   selection fields }
	CSSM_TP_CERTRECLAIM_REJECTED = $3;
	{ certificate reclamation failed due
	   to some error condition }
	CSSM_TP_CERTRECLAIM_NOT_AUTHORIZED = $4;
	{ certificate reclamation was not
	   performed, the request was not
	   authorized }

type
	CSSM_TP_CERTRECLAIM_OUTPUT_PTR = ^cssm_tp_certreclaim_output;
	CSSM_TP_CERTRECLAIM_OUTPUTPtr = ^cssm_tp_certreclaim_output;
	cssm_tp_certreclaim_output = record
		ReclaimStatus: CSSM_TP_CERTRECLAIM_STATUS;
		ReclaimedCertGroup: CSSM_CERTGROUP_PTR;
		KeyCacheHandle: CSSM_LONG_HANDLE;
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
	CSSM_TP_CRLISSUE_INPUT_PTR = ^cssm_tp_crlissue_input;
	CSSM_TP_CRLISSUE_INPUTPtr = ^cssm_tp_crlissue_input;
	cssm_tp_crlissue_input = record
		CLHandle: CSSM_CL_HANDLE;
		CrlIdentifier: UInt32;
		CrlThisTime: CSSM_TIMESTRING;
		PolicyIdentifier: CSSM_FIELD_PTR;
		CallerCredentials: CSSM_ACCESS_CREDENTIALS_PTR;
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
	CSSM_TP_CRLISSUE_STATUS = UInt32;
const
	CSSM_TP_CRLISSUE_STATUS_UNKNOWN = $0;
	{ indeterminate }
	CSSM_TP_CRLISSUE_OK = $1;
	{ a copy of the most current CRL was
	   issued as requested and the time for
	   issuing the next CRL is also returned }
	CSSM_TP_CRLISSUE_NOT_CURRENT = $2;
	{ either no CRL has been issued since
	   the CRL identified in the request, or
	   it is not time to issue an updated CRL.
	   no CRL has been returned, but the time
	   for issuing the next CRL is included
	   in the results }
	CSSM_TP_CRLISSUE_INVALID_DOMAIN = $3;
	{ CRL domain was not recognized or was
	   outside the CA jurisdiction, no CRL or
	   time for the next CRL has been
	   returned. }
	CSSM_TP_CRLISSUE_UNKNOWN_IDENTIFIER = $4;
	{ unrecognized CRL identifier, no CRL or
	   time for the next CRL has been
	   returned. }
	CSSM_TP_CRLISSUE_REJECTED = $5;
	{ CRL was not issued due to some error
	   condition, no CRL or time for the next
	   CRL has been returned. }
	CSSM_TP_CRLISSUE_NOT_AUTHORIZED = $6;
	{ CRL was not issued, the request was
	   not authorized, no CRL or time for the
	   next CRL has been returned. }

type
	CSSM_TP_CRLISSUE_OUTPUT_PTR = ^cssm_tp_crlissue_output;
	CSSM_TP_CRLISSUE_OUTPUTPtr = ^cssm_tp_crlissue_output;
	cssm_tp_crlissue_output = record
		IssueStatus: CSSM_TP_CRLISSUE_STATUS;
		Crl: CSSM_ENCODED_CRL_PTR;
		CrlNextTime: CSSM_TIMESTRING;
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
	CSSM_TP_FORM_TYPE = UInt32;
const
	CSSM_TP_FORM_TYPE_GENERIC = $0;
	CSSM_TP_FORM_TYPE_REGISTRATION = $1;

{ Data Types for Certificate Library Services  }

type
	CSSM_CL_TEMPLATE_TYPE = UInt32;
const
	CSSM_CL_TEMPLATE_INTERMEDIATE_CERT = 1;
	{ for X509 certificates, a fully-formed
	   encoded certificate with empty signature field }
	CSSM_CL_TEMPLATE_PKIX_CERTTEMPLATE = 2;
	{ as defined in RFC2511, section 5 CertTemplate }

type
	CSSM_CERT_BUNDLE_TYPE = UInt32;
const
	CSSM_CERT_BUNDLE_UNKNOWN = $00;
	CSSM_CERT_BUNDLE_CUSTOM = $01;
	CSSM_CERT_BUNDLE_PKCS7_SIGNED_DATA = $02;
	CSSM_CERT_BUNDLE_PKCS7_SIGNED_ENVELOPED_DATA = $03;
	CSSM_CERT_BUNDLE_PKCS12 = $04;
	CSSM_CERT_BUNDLE_PFX = $05;
	CSSM_CERT_BUNDLE_SPKI_SEQUENCE = $06;
	CSSM_CERT_BUNDLE_PGP_KEYRING = $07;
	CSSM_CERT_BUNDLE_LAST = $7FFF;
	{ Applications wishing to define their own custom certificate
	   bundle type should define and publicly document a uint32
	   value greater than CSSM_CL_CUSTOM_CERT_BUNDLE_TYPE }
	CSSM_CL_CUSTOM_CERT_BUNDLE_TYPE = $8000;

type
	CSSM_CERT_BUNDLE_ENCODING = UInt32;
const
	CSSM_CERT_BUNDLE_ENCODING_UNKNOWN = $00;
	CSSM_CERT_BUNDLE_ENCODING_CUSTOM = $01;
	CSSM_CERT_BUNDLE_ENCODING_BER = $02;
	CSSM_CERT_BUNDLE_ENCODING_DER = $03;
	CSSM_CERT_BUNDLE_ENCODING_SEXPR = $04;
	CSSM_CERT_BUNDLE_ENCODING_PGP = $05;

type
	CSSM_CERT_BUNDLE_HEADER_PTR = ^cssm_cert_bundle_header;
	CSSM_CERT_BUNDLE_HEADERPtr = ^cssm_cert_bundle_header;
	cssm_cert_bundle_header = record
		BundleType: CSSM_CERT_BUNDLE_TYPE;
		BundleEncoding: CSSM_CERT_BUNDLE_ENCODING;
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
	CSSM_CERT_BUNDLE_PTR = ^cssm_cert_bundle;
	CSSM_CERT_BUNDLEPtr = ^cssm_cert_bundle;
	cssm_cert_bundle = record
		BundleHeader: CSSM_CERT_BUNDLE_HEADER;
		Bundle: CSSM_DATA;
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

const
	CSSM_FIELDVALUE_COMPLEX_DATA_TYPE = $FFFFFFFF;

{ Data Types for Data Storage Library Services  }

type
	CSSM_DB_ATTRIBUTE_NAME_FORMAT = UInt32;
	CSSM_DB_ATTRIBUTE_NAME_FORMAT_PTR = ^UInt32;
	CSSM_DB_ATTRIBUTE_NAME_FORMATPtr = ^UInt32;
const
	CSSM_DB_ATTRIBUTE_NAME_AS_STRING = 0;
	CSSM_DB_ATTRIBUTE_NAME_AS_OID = 1;
	CSSM_DB_ATTRIBUTE_NAME_AS_INTEGER = 2;

type
	CSSM_DB_ATTRIBUTE_FORMAT = UInt32;
	CSSM_DB_ATTRIBUTE_FORMAT_PTR = ^UInt32;
	CSSM_DB_ATTRIBUTE_FORMATPtr = ^UInt32;
const
	CSSM_DB_ATTRIBUTE_FORMAT_STRING = 0;
	CSSM_DB_ATTRIBUTE_FORMAT_SINT32 = 1;
	CSSM_DB_ATTRIBUTE_FORMAT_UINT32 = 2;
	CSSM_DB_ATTRIBUTE_FORMAT_BIG_NUM = 3;
	CSSM_DB_ATTRIBUTE_FORMAT_REAL = 4;
	CSSM_DB_ATTRIBUTE_FORMAT_TIME_DATE = 5;
	CSSM_DB_ATTRIBUTE_FORMAT_BLOB = 6;
	CSSM_DB_ATTRIBUTE_FORMAT_MULTI_UINT32 = 7;
	CSSM_DB_ATTRIBUTE_FORMAT_COMPLEX = 8;

type
  __embeddded_cssm_db_attribute_info = record
    case Integer of
      0: (AttributeName: CStringPtr);		{ e.g., "record label" }
      1: (AttributeOID: CSSM_OID);		{ e.g., CSSMOID_RECORDLABEL }
      2: (AttributeID: UInt32);			{ e.g., FourCharCode('recl') }
  end;
 
  CSSM_DB_ATTRIBUTE_INFO_PTR = ^cssm_db_attribute_info;
  CSSM_DB_ATTRIBUTE_INFOPtr = ^cssm_db_attribute_info;
	cssm_db_attribute_info = record
    AttributeNameFormat: CSSM_DB_ATTRIBUTE_NAME_FORMAT;
    AttributeFormat: __embeddded_cssm_db_attribute_info;
	end;

type
	CSSM_DB_ATTRIBUTE_DATA_PTR = ^cssm_db_attribute_data;
	CSSM_DB_ATTRIBUTE_DATAPtr = ^cssm_db_attribute_data;
	cssm_db_attribute_data = record
		Info: CSSM_DB_ATTRIBUTE_INFO;
		NumberOfValues: UInt32;
		Value: CSSM_DATA_PTR;
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
	CSSM_DB_RECORDTYPE = UInt32;
const
{ Schema Management Name Space Range Definition}
	CSSM_DB_RECORDTYPE_SCHEMA_START = $00000000;
	CSSM_DB_RECORDTYPE_SCHEMA_END = CSSM_DB_RECORDTYPE_SCHEMA_START + 4;
	{ Open Group Application Name Space Range Definition}
	CSSM_DB_RECORDTYPE_OPEN_GROUP_START = $0000000A;
	CSSM_DB_RECORDTYPE_OPEN_GROUP_END = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 8;
	{ Industry At Large Application Name Space Range Definition }
	CSSM_DB_RECORDTYPE_APP_DEFINED_START = $80000000;
	CSSM_DB_RECORDTYPE_APP_DEFINED_END = $ffffffff;
	{ Record Types defined in the Schema Management Name Space }
	CSSM_DL_DB_SCHEMA_INFO = CSSM_DB_RECORDTYPE_SCHEMA_START + 0;
	CSSM_DL_DB_SCHEMA_INDEXES = CSSM_DB_RECORDTYPE_SCHEMA_START + 1;
	CSSM_DL_DB_SCHEMA_ATTRIBUTES = CSSM_DB_RECORDTYPE_SCHEMA_START + 2;
	CSSM_DL_DB_SCHEMA_PARSING_MODULE = CSSM_DB_RECORDTYPE_SCHEMA_START + 3;
	{ Record Types defined in the Open Group Application Name Space }
	CSSM_DL_DB_RECORD_ANY = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 0;
	CSSM_DL_DB_RECORD_CERT = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 1;
	CSSM_DL_DB_RECORD_CRL = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 2;
	CSSM_DL_DB_RECORD_POLICY = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 3;
	CSSM_DL_DB_RECORD_GENERIC = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 4;
	CSSM_DL_DB_RECORD_PUBLIC_KEY = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 5;
	CSSM_DL_DB_RECORD_PRIVATE_KEY = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 6;
	CSSM_DL_DB_RECORD_SYMMETRIC_KEY = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 7;
	CSSM_DL_DB_RECORD_ALL_KEYS = CSSM_DB_RECORDTYPE_OPEN_GROUP_START + 8;

const
	CSSM_DB_CERT_USE_TRUSTED = $00000001;	{ application-defined as trusted }
	CSSM_DB_CERT_USE_SYSTEM = $00000002;	{ the CSSM system cert }
	CSSM_DB_CERT_USE_OWNER = $00000004;	{ private key owned by system user}
	CSSM_DB_CERT_USE_REVOKED = $00000008;	{ revoked cert -15913 used w CRL APIs }
	CSSM_DB_CERT_USE_SIGNING = $00000010;	{ use cert for signing only }
	CSSM_DB_CERT_USE_PRIVACY = $00000020;	{ use cert for confidentiality only }

type
	CSSM_DB_RECORD_ATTRIBUTE_INFO_PTR = ^cssm_db_record_attribute_info;
	CSSM_DB_RECORD_ATTRIBUTE_INFOPtr = ^cssm_db_record_attribute_info;
	cssm_db_record_attribute_info = record
		DataRecordType: CSSM_DB_RECORDTYPE;
		NumberOfAttributes: UInt32;
		AttributeInfo: CSSM_DB_ATTRIBUTE_INFO_PTR;
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
	CSSM_DB_RECORD_ATTRIBUTE_DATA_PTR = ^cssm_db_record_attribute_data;
	CSSM_DB_RECORD_ATTRIBUTE_DATAPtr = ^cssm_db_record_attribute_data;
	cssm_db_record_attribute_data = record
		DataRecordType: CSSM_DB_RECORDTYPE;
		SemanticInformation: UInt32;
		NumberOfAttributes: UInt32;
		AttributeData: CSSM_DB_ATTRIBUTE_DATA_PTR;
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
	CSSM_DB_PARSING_MODULE_INFO_PTR = ^cssm_db_parsing_module_info;
	CSSM_DB_PARSING_MODULE_INFOPtr = ^cssm_db_parsing_module_info;
	cssm_db_parsing_module_info = record
		RecordType: CSSM_DB_RECORDTYPE;
		ModuleSubserviceUid: CSSM_SUBSERVICE_UID;
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
	CSSM_DB_INDEX_TYPE = UInt32;
const
	CSSM_DB_INDEX_UNIQUE = 0;
	CSSM_DB_INDEX_NONUNIQUE = 1;

type
	CSSM_DB_INDEXED_DATA_LOCATION = UInt32;
const
	CSSM_DB_INDEX_ON_UNKNOWN = 0;
	CSSM_DB_INDEX_ON_ATTRIBUTE = 1;
	CSSM_DB_INDEX_ON_RECORD = 2;

type
	CSSM_DB_INDEX_INFO_PTR = ^cssm_db_index_info;
	CSSM_DB_INDEX_INFOPtr = ^cssm_db_index_info;
	cssm_db_index_info = record
		IndexType: CSSM_DB_INDEX_TYPE;
		IndexedDataLocation: CSSM_DB_INDEXED_DATA_LOCATION;
		Info: CSSM_DB_ATTRIBUTE_INFO;
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
	CSSM_DB_UNIQUE_RECORD_PTR = ^cssm_db_unique_record;
	CSSM_DB_UNIQUE_RECORDPtr = ^cssm_db_unique_record;
	cssm_db_unique_record = record
		RecordLocator: CSSM_DB_INDEX_INFO;
		RecordIdentifier: CSSM_DATA;
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
	CSSM_DB_RECORD_INDEX_INFO_PTR = ^cssm_db_record_index_info;
	CSSM_DB_RECORD_INDEX_INFOPtr = ^cssm_db_record_index_info;
	cssm_db_record_index_info = record
		DataRecordType: CSSM_DB_RECORDTYPE;
		NumberOfIndexes: UInt32;
		IndexInfo: CSSM_DB_INDEX_INFO_PTR;
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
	CSSM_DB_ACCESS_TYPE = UInt32;
	CSSM_DB_ACCESS_TYPE_PTR = ^UInt32;
	CSSM_DB_ACCESS_TYPEPtr = ^UInt32;
const
	CSSM_DB_ACCESS_READ = $00001;
	CSSM_DB_ACCESS_WRITE = $00002;
	CSSM_DB_ACCESS_PRIVILEGED = $00004; { versus user mode }

type
	CSSM_DB_MODIFY_MODE = UInt32;
const
	CSSM_DB_MODIFY_ATTRIBUTE_NONE = 0;
	CSSM_DB_MODIFY_ATTRIBUTE_ADD = CSSM_DB_MODIFY_ATTRIBUTE_NONE + 1;
	CSSM_DB_MODIFY_ATTRIBUTE_DELETE = CSSM_DB_MODIFY_ATTRIBUTE_NONE + 2;
	CSSM_DB_MODIFY_ATTRIBUTE_REPLACE = CSSM_DB_MODIFY_ATTRIBUTE_NONE + 3;

type
	CSSM_DBINFO_PTR = ^cssm_dbinfo;
	CSSM_DBINFOPtr = ^cssm_dbinfo;
	cssm_dbinfo = record
{ meta information about each record type stored in this
    data store including meta information about record
    attributes and indexes }
		NumberOfRecordTypes: UInt32;
		DefaultParsingModules: CSSM_DB_PARSING_MODULE_INFO_PTR;
		RecordAttributeNames: CSSM_DB_RECORD_ATTRIBUTE_INFO_PTR;
		RecordIndexes: CSSM_DB_RECORD_INDEX_INFO_PTR;
    { access restrictions for opening this data store }
		IsLocal: CSSM_BOOL;
		AccessPath: CStringPtr; { URL, dir path, etc. }
		Reserved: UnivPtr;
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
	CSSM_DB_OPERATOR = UInt32;
	CSSM_DB_OPERATOR_PTR = ^UInt32;
	CSSM_DB_OPERATORPtr = ^UInt32;
const
	CSSM_DB_EQUAL = 0;
	CSSM_DB_NOT_EQUAL = 1;
	CSSM_DB_LESS_THAN = 2;
	CSSM_DB_GREATER_THAN = 3;
	CSSM_DB_CONTAINS = 4;
	CSSM_DB_CONTAINS_INITIAL_SUBSTRING = 5;
	CSSM_DB_CONTAINS_FINAL_SUBSTRING = 6;

type
	CSSM_DB_CONJUNCTIVE = UInt32;
	CSSM_DB_CONJUNCTIVE_PTR = ^UInt32;
	CSSM_DB_CONJUNCTIVEPtr = ^UInt32;
const
	CSSM_DB_NONE = 0;
	CSSM_DB_AND = 1;
	CSSM_DB_OR = 2;

type
	CSSM_SELECTION_PREDICATE_PTR = ^cssm_selection_predicate;
	CSSM_SELECTION_PREDICATEPtr = ^cssm_selection_predicate;
	cssm_selection_predicate = record
		DbOperator: CSSM_DB_OPERATOR;
		Attribute: CSSM_DB_ATTRIBUTE_DATA;
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

const
	CSSM_QUERY_TIMELIMIT_NONE = 0;

const
	CSSM_QUERY_SIZELIMIT_NONE = 0;

type
	CSSM_QUERY_LIMITS_PTR = ^cssm_query_limits;
	CSSM_QUERY_LIMITSPtr = ^cssm_query_limits;
	cssm_query_limits = record
		TimeLimit: UInt32; { in seconds }
		SizeLimit: UInt32; { max. number of records to return }
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
	CSSM_QUERY_FLAGS = UInt32;
const
	CSSM_QUERY_RETURN_DATA = $01;

type
	CSSM_QUERY_PTR = ^cssm_query;
	CSSM_QUERYPtr = ^cssm_query;
	cssm_query = record
		RecordType: CSSM_DB_RECORDTYPE;
		Conjunctive: CSSM_DB_CONJUNCTIVE;
		NumSelectionPredicates: UInt32;
		SelectionPredicate: CSSM_SELECTION_PREDICATE_PTR;
		QueryLimits: CSSM_QUERY_LIMITS;
		QueryFlags: CSSM_QUERY_FLAGS;
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
	CSSM_DLTYPE = UInt32;
	CSSM_DLTYPE_PTR = ^UInt32;
	CSSM_DLTYPEPtr = ^UInt32;
const
	CSSM_DL_UNKNOWN = 0;
	CSSM_DL_CUSTOM = 1;
	CSSM_DL_LDAP = 2;
	CSSM_DL_ODBC = 3;
	CSSM_DL_PKCS11 = 4;
	CSSM_DL_FFS = 5; { flat file system }
	CSSM_DL_MEMORY = 6;
	CSSM_DL_REMOTEDIR = 7;

type
	CSSM_DL_CUSTOM_ATTRIBUTES = UnivPtr;
	CSSM_DL_LDAP_ATTRIBUTES = UnivPtr;
	CSSM_DL_ODBC_ATTRIBUTES = UnivPtr;
	CSSM_DL_FFS_ATTRIBUTES = UnivPtr;

type
	CSSM_DL_PKCS11_ATTRIBUTE_PTR = ^cssm_dl_pkcs11_attributes;
	CSSM_DL_PKCS11_ATTRIBUTEPtr = ^cssm_dl_pkcs11_attributes;
	cssm_dl_pkcs11_attributes = record
		DeviceAccessFlags: UInt32;
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

const
	CSSM_DB_DATASTORES_UNKNOWN = $FFFFFFFF;

type
  CSSM_NAME_LIST_PTR = ^cssm_name_list;
  CSSM_NAME_LISTPtr = ^cssm_name_list;
 cssm_name_list = record
		NumStrings: UInt32;
		String_: CStringPtrPtr;
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
	CSSM_DB_RETRIEVAL_MODES = UInt32;
const
	CSSM_DB_TRANSACTIONAL_MODE = 0;
	CSSM_DB_FILESYSTEMSCAN_MODE = 1;

type
	CSSM_DB_SCHEMA_ATTRIBUTE_INFO_PTR = ^cssm_db_schema_attribute_info;
	CSSM_DB_SCHEMA_ATTRIBUTE_INFOPtr = ^cssm_db_schema_attribute_info;
	cssm_db_schema_attribute_info = record
		AttributeId: UInt32;
		AttributeName: CStringPtr;
		AttributeNameID: CSSM_OID;
		DataType: CSSM_DB_ATTRIBUTE_FORMAT;
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
	CSSM_DB_SCHEMA_INDEX_INFO_PTR = ^cssm_db_schema_index_info;
	CSSM_DB_SCHEMA_INDEX_INFOPtr = ^cssm_db_schema_index_info;
	cssm_db_schema_index_info = record
		AttributeId: UInt32;
		IndexId: UInt32;
		IndexType: CSSM_DB_INDEX_TYPE;
		IndexedDataLocation: CSSM_DB_INDEXED_DATA_LOCATION;
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

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
