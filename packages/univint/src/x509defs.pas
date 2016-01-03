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
 * x509defs.h -- Data structures for X509 Certificate Library field values
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

unit x509defs;
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
	CSSM_BER_TAG = UInt8;
const
	BER_TAG_UNKNOWN = 0;
const
	BER_TAG_BOOLEAN = 1;
const
	BER_TAG_INTEGER = 2;
const
	BER_TAG_BIT_STRING = 3;
const
	BER_TAG_OCTET_STRING = 4;
const
	BER_TAG_NULL = 5;
const
	BER_TAG_OID = 6;
const
	BER_TAG_OBJECT_DESCRIPTOR = 7;
const
	BER_TAG_EXTERNAL = 8;
const
	BER_TAG_REAL = 9;
const
	BER_TAG_ENUMERATED = 10;
{ 12 to 15 are reserved for future versions of the recommendation }
const
	BER_TAG_PKIX_UTF8_STRING = 12;
const
	BER_TAG_SEQUENCE = 16;
const
	BER_TAG_SET = 17;
const
	BER_TAG_NUMERIC_STRING = 18;
const
	BER_TAG_PRINTABLE_STRING = 19;
const
	BER_TAG_T61_STRING = 20;
const
  BER_TAG_TELETEX_STRING = BER_TAG_T61_STRING;
const
	BER_TAG_VIDEOTEX_STRING = 21;
const
	BER_TAG_IA5_STRING = 22;
const
	BER_TAG_UTC_TIME = 23;
const
	BER_TAG_GENERALIZED_TIME = 24;
const
	BER_TAG_GRAPHIC_STRING = 25;
const
	BER_TAG_ISO646_STRING = 26;
const
	BER_TAG_GENERAL_STRING = 27;
const
  BER_TAG_VISIBLE_STRING = BER_TAG_ISO646_STRING;
{ 28 - are reserved for future versions of the recommendation }
const
	BER_TAG_PKIX_UNIVERSAL_STRING = 28;
const
	BER_TAG_PKIX_BMP_STRING = 30;


{ Data Structures for X.509 Certificates }

type
	CSSM_X509_ALGORITHM_IDENTIFIER_PTR = ^cssm_x509_algorithm_identifier;
	CSSM_X509_ALGORITHM_IDENTIFIERPtr = ^cssm_x509_algorithm_identifier;
	cssm_x509_algorithm_identifier = record
		algorithm: CSSM_OID;
		parameters: CSSM_DATA;
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

{ X509 Distinguished name structure }
type
	CSSM_X509_TYPE_VALUE_PAIR_PTR = ^cssm_x509_type_value_pair;
	CSSM_X509_TYPE_VALUE_PAIRPtr = ^cssm_x509_type_value_pair;
	cssm_x509_type_value_pair = record
		typ: CSSM_OID;
		valueType: CSSM_BER_TAG; { The Tag to be used when }
    {this value is BER encoded }
		value: CSSM_DATA;
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
	CSSM_X509_RDN_PTR = ^cssm_x509_rdn;
	CSSM_X509_RDNPtr = ^cssm_x509_rdn;
	cssm_x509_rdn = record
		numberOfPairs: UInt32;
		AttributeTypeAndValue: CSSM_X509_TYPE_VALUE_PAIR_PTR;
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
	CSSM_X509_NAME_PTR = ^cssm_x509_name;
	CSSM_X509_NAMEPtr = ^cssm_x509_name;
	cssm_x509_name = record
		numberOfRDNs: UInt32;
		RelativeDistinguishedName: CSSM_X509_RDN_PTR;
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

{ Public key info struct }
type
	CSSM_X509_SUBJECT_PUBLIC_KEY_INFO_PTR = ^cssm_x509_subject_public_key_info;
	CSSM_X509_SUBJECT_PUBLIC_KEY_INFOPtr = ^cssm_x509_subject_public_key_info;
	cssm_x509_subject_public_key_info = record
		algorithm: CSSM_X509_ALGORITHM_IDENTIFIER;
		subjectPublicKey: CSSM_DATA;
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
	CSSM_X509_TIME_PTR = ^cssm_x509_time;
	CSSM_X509_TIMEPtr = ^cssm_x509_time;
	cssm_x509_time = record
		timeType: CSSM_BER_TAG;
		time: CSSM_DATA;
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

{ Validity struct }
type
	CSSM_X509_VALIDITY_PTR = ^CSSM_X509_VALIDITY;
	CSSM_X509_VALIDITYPtr = ^CSSM_X509_VALIDITY;
	CSSM_X509_VALIDITY = record
		notBefore: CSSM_X509_TIME;
		notAfter: CSSM_X509_TIME;
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
  CSSM_X509_OPTION_PRESENT = CSSM_TRUE;
  CSSM_X509_OPTION_NOT_PRESENT = CSSM_FALSE;

type
	CSSM_X509_OPTION = CSSM_BOOL;

type
	CSSM_X509EXT_BASICCONSTRAINTS_PTR = ^cssm_x509ext_basicConstraints;
	CSSM_X509EXT_BASICCONSTRAINTSPtr = ^cssm_x509ext_basicConstraints;
	cssm_x509ext_basicConstraints = record
		cA: CSSM_BOOL;
		pathLenConstraintPresent: CSSM_X509_OPTION;
		pathLenConstraint: UInt32;
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
  CSSM_X509EXT_DATA_FORMAT = UInt32;

const
  CSSM_X509_DATAFORMAT_ENCODED = 0;
  CSSM_X509_DATAFORMAT_PARSED = 1;
  CSSM_X509_DATAFORMAT_PAIR = 2;

type
	CSSM_X509EXT_TAGandVALUE_PTR = ^CSSM_X509EXT_TAGandVALUE;
	CSSM_X509EXT_TAGandVALUEPtr = ^CSSM_X509EXT_TAGandVALUE;
	CSSM_X509EXT_TAGandVALUE = record
		typ: CSSM_BER_TAG;
		value: CSSM_DATA;
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
	CSSM_X509EXT_PAIR_PTR = ^cssm_x509ext_pair;
	CSSM_X509EXT_PAIRPtr = ^cssm_x509ext_pair;
	cssm_x509ext_pair = record
		tagAndValue: CSSM_X509EXT_TAGandVALUE;
		parsedValue: UnivPtr;
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

{ Extension structure }
type
  __embedded_cssm_x509_extension = record
    case Integer of
      0: (tagAndValue: CSSM_X509EXT_TAGandVALUEPtr);
      1: (parsedValue: UnivPtr);
      2: (valuePair: CSSM_X509EXT_PAIRPtr);
  end;

  CSSM_X509_EXTENSION_PTR = ^cssm_x509_extension;
  CSSM_X509_EXTENSIONPtr = ^cssm_x509_extension;
	cssm_x509_extension = record
		extnId: CSSM_OID;
		critical: CSSM_BOOL;
		format: CSSM_X509EXT_DATA_FORMAT;
		value: __embedded_cssm_x509_extension;
    BERvalue: CSSM_DATA ;
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
	CSSM_X509_EXTENSIONS_PTR = ^cssm_x509_extensions;
	CSSM_X509_EXTENSIONSPtr = ^cssm_x509_extensions;
	cssm_x509_extensions = record
		numberOfExtensions: UInt32;
		extensions: CSSM_X509_EXTENSION_PTR;
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

{ X509V3 certificate structure }
type
	CSSM_X509_TBS_CERTIFICATE_PTR = ^cssm_x509_tbs_certificate;
	CSSM_X509_TBS_CERTIFICATEPtr = ^cssm_x509_tbs_certificate;
	cssm_x509_tbs_certificate = record
		version: CSSM_DATA;
		serialNumber: CSSM_DATA;
		signature: CSSM_X509_ALGORITHM_IDENTIFIER;
		issuer: CSSM_X509_NAME;
		validity: CSSM_X509_VALIDITY;
		subject: CSSM_X509_NAME;
		subjectPublicKeyInfo: CSSM_X509_SUBJECT_PUBLIC_KEY_INFO;
		issuerUniqueIdentifier: CSSM_DATA;
		subjectUniqueIdentifier: CSSM_DATA;
		extensions: CSSM_X509_EXTENSIONS;
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

{ Signature structure }
type
	CSSM_X509_SIGNATURE_PTR = ^cssm_x509_signature;
	CSSM_X509_SIGNATUREPtr = ^cssm_x509_signature;
	cssm_x509_signature = record
		algorithmIdentifier: CSSM_X509_ALGORITHM_IDENTIFIER;
		encrypted: CSSM_DATA;
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

{ Signed certificate structure }
type
	CSSM_X509_SIGNED_CERTIFICATE_PTR = ^cssm_x509_signed_certificate;
	CSSM_X509_SIGNED_CERTIFICATEPtr = ^cssm_x509_signed_certificate;
	cssm_x509_signed_certificate = record
		certificate: CSSM_X509_TBS_CERTIFICATE;
		signature: CSSM_X509_SIGNATURE;
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
	CSSM_X509EXT_POLICYQUALIFIERINFO_PTR = ^cssm_x509ext_policyQualifierInfo;
	CSSM_X509EXT_POLICYQUALIFIERINFOPtr = ^cssm_x509ext_policyQualifierInfo;
	cssm_x509ext_policyQualifierInfo = record
		policyQualifierId: CSSM_OID;
		value: CSSM_DATA;
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
	CSSM_X509EXT_POLICYQUALIFIERS_PTR = ^cssm_x509ext_policyQualifiers;
	CSSM_X509EXT_POLICYQUALIFIERSPtr = ^cssm_x509ext_policyQualifiers;
	cssm_x509ext_policyQualifiers = record
		numberOfPolicyQualifiers: UInt32;
		policyQualifier: CSSM_X509EXT_POLICYQUALIFIERINFOPtr;
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
	CSSM_X509EXT_POLICYINFO_PTR = ^cssm_x509ext_policyInfo;
	CSSM_X509EXT_POLICYINFOPtr = ^cssm_x509ext_policyInfo;
	cssm_x509ext_policyInfo = record
		policyIdentifier: CSSM_OID;
		policyQualifiers: CSSM_X509EXT_POLICYQUALIFIERS;
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


{ Data Structures for X.509 Certificate Revocations Lists }

{ x509V2 entry in the CRL revokedCertificates sequence }
type
	CSSM_X509_REVOKED_CERT_ENTRY_PTR = ^cssm_x509_revoked_cert_entry;
	CSSM_X509_REVOKED_CERT_ENTRYPtr = ^cssm_x509_revoked_cert_entry;
	cssm_x509_revoked_cert_entry = record
		certificateSerialNumber: CSSM_DATA;
		revocationDate: CSSM_X509_TIME;
		extensions: CSSM_X509_EXTENSIONS;
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
	CSSM_X509_REVOKED_CERT_LIST_PTR = ^cssm_x509_revoked_cert_list;
	CSSM_X509_REVOKED_CERT_LISTPtr = ^cssm_x509_revoked_cert_list;
	cssm_x509_revoked_cert_list = record
		numberOfRevokedCertEntries: UInt32;
		revokedCertEntry: CSSM_X509_REVOKED_CERT_ENTRY_PTR;
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

{ x509v2 Certificate Revocation List (CRL) (unsigned) structure }
type
	CSSM_X509_TBS_CERTLIST_PTR = ^cssm_x509_tbs_certlist;
	CSSM_X509_TBS_CERTLISTPtr = ^cssm_x509_tbs_certlist;
	cssm_x509_tbs_certlist = record
		version: CSSM_DATA;
		signature: CSSM_X509_ALGORITHM_IDENTIFIER;
		issuer: CSSM_X509_NAME;
		thisUpdate: CSSM_X509_TIME;
		nextUpdate: CSSM_X509_TIME;
		revokedCertificates: CSSM_X509_REVOKED_CERT_LIST_PTR;
		extensions: CSSM_X509_EXTENSIONS;
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
	CSSM_X509_SIGNED_CRL_PTR = ^cssm_x509_signed_crl;
	CSSM_X509_SIGNED_CRLPtr = ^cssm_x509_signed_crl;
	cssm_x509_signed_crl = record
		tbsCertList: CSSM_X509_TBS_CERTLIST;
		signature: CSSM_X509_SIGNATURE;
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
