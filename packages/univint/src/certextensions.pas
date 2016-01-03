{
 * Copyright (c) 2000-2004 Apple Computer, Inc. All Rights Reserved.
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
 * CertExtensions.h -- X.509 Cert Extensions as C structs
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

unit certextensions;
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes,cssmtype,x509defs;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$packrecords c}

{**
 *** Structs for declaring extension-specific data. 
 **}

{
 * GeneralName, used in AuthorityKeyID, SubjectAltName, and 
 * IssuerAltName. 
 *
 * For now, we just provide explicit support for the types which are
 * represented as IA5Strings, OIDs, and octet strings. Constructed types
 * such as EDIPartyName and x400Address are not explicitly handled
 * right now and must be encoded and decoded by the caller. (See exception
 * for Name and OtherName, below). In those cases the CE_GeneralName.name.Data field 
 * represents the BER contents octets; CE_GeneralName.name.Length is the 
 * length of the contents; the tag of the field is not needed - the BER 
 * encoding uses context-specific implicit tagging. The berEncoded field 
 * is set to CSSM_TRUE in these case. Simple types have berEncoded = CSSM_FALSE. 
 *
 * In the case of a GeneralName in the form of a Name, we parse the Name
 * into a CSSM_X509_NAME and place a pointer to the CSSM_X509_NAME in the
 * CE_GeneralName.name.Data field. CE_GeneralName.name.Length is set to 
 * sizeof(CSSM_X509_NAME). In this case berEncoded is false. 
 *
 * In the case of a GeneralName in the form of a OtherName, we parse the fields
 * into a CE_OtherName and place a pointer to the CE_OtherName in the
 * CE_GeneralName.name.Data field. CE_GeneralName.name.Length is set to 
 * sizeof(CE_OtherName). In this case berEncoded is false. 
 *
 *      GeneralNames ::= SEQUENCE SIZE (1..MAX) OF GeneralName
 *
 *      GeneralName ::= CHOICE (
 *           otherName                       [0]     OtherName
 *           rfc822Name                      [1]     IA5String,
 *           dNSName                         [2]     IA5String,
 *           x400Address                     [3]     ORAddress,
 *           directoryName                   [4]     Name,
 *           ediPartyName                    [5]     EDIPartyName,
 *           uniformResourceIdentifier       [6]     IA5String,
 *           iPAddress                       [7]     OCTET STRING,
 *           registeredID                    [8]     OBJECT IDENTIFIER)
 *
 *      OtherName ::= SEQUENCE (
 *           type-id    OBJECT IDENTIFIER,
 *           value      [0] EXPLICIT ANY DEFINED BY type-id )
 *
 *      EDIPartyName ::= SEQUENCE (
 *           nameAssigner            [0]     DirectoryString OPTIONAL,
 *           partyName               [1]     DirectoryString )
 }
type
  CE_GeneralNameType = UInt32;
const
	GNT_OtherName = 0;
	GNT_RFC822Name = 1;
	GNT_DNSName = 2;
	GNT_X400Address = 3;
	GNT_DirectoryName = 4;
	GNT_EdiPartyName = 5;
	GNT_URI = 6;
	GNT_IPAddress = 7;
	GNT_RegisteredID = 8;

type
	CE_OtherNamePtr = ^CE_OtherName;
	CE_OtherName = record
		typeId: CSSM_OID;
		value: CSSM_DATA;		// unparsed, BER-encoded
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
  CE_GeneralNamePtr = ^CE_GeneralName;
	CE_GeneralName = record
		nameType: CE_GeneralNameType;	// GNT_RFC822Name, etc.
		berEncoded: CSSM_BOOL;
		name: CSSM_DATA;
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
  CE_GeneralNamesPtr = ^CE_GeneralNames;
	CE_GeneralNames = record
		numNames: UInt32;
		generalName: CE_GeneralNamePtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	end;
	(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
	end;	
>>>>>>> graemeg/cpstrnew
=======
	end;	
>>>>>>> graemeg/cpstrnew
=======
	end;	
>>>>>>> graemeg/cpstrnew
=======
	end;	
>>>>>>> origin/cpstrnew

{
 * id-ce-authorityKeyIdentifier OBJECT IDENTIFIER ::=  ( id-ce 35 )
 *
 *   AuthorityKeyIdentifier ::= SEQUENCE (
 *     keyIdentifier             [0] KeyIdentifier           OPTIONAL,
 *     authorityCertIssuer       [1] GeneralNames            OPTIONAL,
 *     authorityCertSerialNumber [2] CertificateSerialNumber OPTIONAL  )
 *
 *   KeyIdentifier ::= OCTET STRING
 *
 * CSSM OID = CSSMOID_AuthorityKeyIdentifier
 }
type
  CE_AuthorityKeyIDPtr = ^CE_AuthorityKeyID;
	CE_AuthorityKeyID = record
		keyIdentifierPresent: CSSM_BOOL;
		keyIdentifier: CSSM_DATA;
		generalNamesPresent: CSSM_BOOL;
		generalNames: CE_GeneralNamesPtr;
		serialNumberPresent: CSSM_BOOL;
		serialNumber: CSSM_DATA;
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

{
 * id-ce-subjectKeyIdentifier OBJECT IDENTIFIER ::=  ( id-ce 14 )
 *   SubjectKeyIdentifier ::= KeyIdentifier
 *
 * CSSM OID = CSSMOID_SubjectKeyIdentifier
 }
type
	CE_SubjectKeyID = CSSM_DATA;
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

{
 * id-ce-keyUsage OBJECT IDENTIFIER ::=  ( id-ce 15 )
 *
 *     KeyUsage ::= BIT STRING (
 *          digitalSignature        (0),
 *          nonRepudiation          (1),
 *          keyEncipherment         (2),
 *          dataEncipherment        (3),
 *          keyAgreement            (4),
 *          keyCertSign             (5),
 *          cRLSign                 (6),
 *          encipherOnly            (7),
 *          decipherOnly            (8) )
 *
 * CSSM OID = CSSMOID_KeyUsage
 *
 }
type
	CE_KeyUsage = UInt16;
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
	CE_KU_DigitalSignature = $8000;
const
	CE_KU_NonRepudiation = $4000;
const
	CE_KU_KeyEncipherment = $2000;
const
	CE_KU_DataEncipherment = $1000;
const
	CE_KU_KeyAgreement = $0800;
const
	CE_KU_KeyCertSign = $0400;
const
	CE_KU_CRLSign = $0200;
const
	CE_KU_EncipherOnly = $0100;
const
	CE_KU_DecipherOnly = $0080;

{
 *  id-ce-cRLReason OBJECT IDENTIFIER ::= ( id-ce 21 )
 *
 *   -- reasonCode ::= ( CRLReason )
 *
 *   CRLReason ::= ENUMERATED (
 *  	unspecified             (0),
 *      keyCompromise           (1),
 *     	cACompromise            (2),
 *    	affiliationChanged      (3),
 *   	superseded              (4),
 *  	cessationOfOperation    (5),
 * 		certificateHold         (6),
 *		removeFromCRL           (8) )
 *
 * CSSM OID = CSSMOID_CrlReason
 *
 }
type
	CE_CrlReason = UInt32;
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
	CE_CR_Unspecified = 0;
const
	CE_CR_KeyCompromise = 1;
const
	CE_CR_CACompromise = 2;
const
	CE_CR_AffiliationChanged = 3;
const
	CE_CR_Superseded = 4;
const
	CE_CR_CessationOfOperation = 5;
const
	CE_CR_CertificateHold = 6;
const
	CE_CR_RemoveFromCRL = 8;

{
 * id-ce-subjectAltName OBJECT IDENTIFIER ::=  ( id-ce 17 )
 *
 *      SubjectAltName ::= GeneralNames
 *
 * CSSM OID = CSSMOID_SubjectAltName
 *
 * GeneralNames defined above.
 }

{
 *  id-ce-extKeyUsage OBJECT IDENTIFIER ::= (id-ce 37)
 *
 *   ExtKeyUsageSyntax ::= SEQUENCE SIZE (1..MAX) OF KeyPurposeId*
 *
 *  KeyPurposeId ::= OBJECT IDENTIFIER
 *
 * CSSM OID = CSSMOID_ExtendedKeyUsage
 }
type
  CE_ExtendedKeyUsagePtr = ^CE_ExtendedKeyUsage;
	CE_ExtendedKeyUsage = record
		numPurposes: UInt32;
		purposes: CSSM_OID_PTR;		// in Intel pre-encoded format
	end;

{
 * id-ce-basicConstraints OBJECT IDENTIFIER ::=  ( id-ce 19 )
 *
 * BasicConstraints ::= SEQUENCE (
 *       cA                      BOOLEAN DEFAULT FALSE,
 *       pathLenConstraint       INTEGER (0..MAX) OPTIONAL )
 *
 * CSSM OID = CSSMOID_BasicConstraints
 }
type
  CE_BasicConstraintsPtr = ^CE_BasicConstraints;
	CE_BasicConstraints = record
		cA: CSSM_BOOL;
		pathLenConstraintPresent: CSSM_BOOL;
		pathLenConstraint: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	end;
	(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
	end;	
>>>>>>> graemeg/cpstrnew
=======
	end;	
>>>>>>> graemeg/cpstrnew
=======
	end;	
>>>>>>> graemeg/cpstrnew
=======
	end;	
>>>>>>> origin/cpstrnew

{
 * id-ce-certificatePolicies OBJECT IDENTIFIER ::=  ( id-ce 32 )
 *
 *   certificatePolicies ::= SEQUENCE SIZE (1..MAX) OF PolicyInformation
 *
 *   PolicyInformation ::= SEQUENCE (
 *        policyIdentifier   CertPolicyId,
 *        policyQualifiers   SEQUENCE SIZE (1..MAX) OF
 *                                PolicyQualifierInfo OPTIONAL )
 *
 *   CertPolicyId ::= OBJECT IDENTIFIER
 *
 *   PolicyQualifierInfo ::= SEQUENCE (
 *        policyQualifierId  PolicyQualifierId,
 *        qualifier          ANY DEFINED BY policyQualifierId ) 
 *
 *   -- policyQualifierIds for Internet policy qualifiers
 *
 *   id-qt          OBJECT IDENTIFIER ::=  ( id-pkix 2 )
 *   id-qt-cps      OBJECT IDENTIFIER ::=  ( id-qt 1 )
 *   id-qt-unotice  OBJECT IDENTIFIER ::=  ( id-qt 2 )
 *
 *   PolicyQualifierId ::=
 *        OBJECT IDENTIFIER ( id-qt-cps | id-qt-unotice )
 *
 *   Qualifier ::= CHOICE (
 *        cPSuri           CPSuri,
 *        userNotice       UserNotice )
 *
 *   CPSuri ::= IA5String
 *
 *   UserNotice ::= SEQUENCE (
 *        noticeRef        NoticeReference OPTIONAL,
 *        explicitText     DisplayText OPTIONAL)
 *
 *   NoticeReference ::= SEQUENCE (
 *        organization     DisplayText,
 *        noticeNumbers    SEQUENCE OF INTEGER )
 *
 *   DisplayText ::= CHOICE (
 *        visibleString    VisibleString  (SIZE (1..200)),
 *        bmpString        BMPString      (SIZE (1..200)),
 *        utf8String       UTF8String     (SIZE (1..200)) )
 *
 *  CSSM OID = CSSMOID_CertificatePolicies
 *
 * We only support down to the level of Qualifier, and then only the CPSuri
 * choice. UserNotice is transmitted to and from this library as a raw
 * CSSM_DATA containing the BER-encoded UserNotice sequence. 
 }

type
  CE_PolicyQualifierInfoPtr = ^CE_PolicyQualifierInfo;
	CE_PolicyQualifierInfo = record
		policyQualifierId: CSSM_OID;			// CSSMOID_QT_CPS, CSSMOID_QT_UNOTICE
		qualifier: CSSM_DATA;					// CSSMOID_QT_CPS: IA5String contents
											// CSSMOID_QT_UNOTICE : Sequence contents
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
  CE_PolicyInformationPtr = ^CE_PolicyInformation;
	CE_PolicyInformation = record
		certPolicyId: CSSM_OID;
		numPolicyQualifiers: UInt32;	// size of *policyQualifiers;
		policyQualifiers: CE_PolicyQualifierInfoPtr;
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
  CE_CertPoliciesPtr = ^CE_CertPolicies;
	CE_CertPolicies = record
		numPolicies: UInt32;			// size of *policies;
		policies: CE_PolicyInformationPtr;
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

{
 * netscape-cert-type, a bit string.
 *
 * CSSM OID = CSSMOID_NetscapeCertType
 *
 * Bit fields defined in oidsattr.h: CE_NCT_SSL_Client, etc.
 }
type
	CE_NetscapeCertType = UInt16;
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

{
 * CRLDistributionPoints.
 *
 *   id-ce-cRLDistributionPoints OBJECT IDENTIFIER ::=  ( id-ce 31 )
 *
 *   cRLDistributionPoints ::= (
 *        CRLDistPointsSyntax )
 *
 *   CRLDistPointsSyntax ::= SEQUENCE SIZE (1..MAX) OF DistributionPoint
 *
 *   NOTE: RFC 2459 claims that the tag for the optional DistributionPointName
 *   is IMPLICIT as shown here, but in practice it is EXPLICIT. It has to be -
 *   because the underlying type also uses an implicit tag for distinguish
 *   between CHOICEs.
 *
 *   DistributionPoint ::= SEQUENCE (
 *        distributionPoint       [0]     DistributionPointName OPTIONAL,
 *        reasons                 [1]     ReasonFlags OPTIONAL,
 *        cRLIssuer               [2]     GeneralNames OPTIONAL )
 *
 *   DistributionPointName ::= CHOICE (
 *        fullName                [0]     GeneralNames,
 *        nameRelativeToCRLIssuer [1]     RelativeDistinguishedName )
 *
 *   ReasonFlags ::= BIT STRING (
 *        unused                  (0),
 *        keyCompromise           (1),
 *        cACompromise            (2),
 *        affiliationChanged      (3),
 *        superseded              (4),
 *        cessationOfOperation    (5),
 *        certificateHold         (6) )
 *
 * CSSM OID = CSSMOID_CrlDistributionPoints
 }
 
{
 * Note that this looks similar to CE_CrlReason, but that's an enum and this
 * is an OR-able bit string.
 }
type
	CE_CrlDistReasonFlags = UInt8;
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
	CE_CD_Unspecified = $80;
const
	CE_CD_KeyCompromise = $40;
const
	CE_CD_CACompromise = $20;
const
	CE_CD_AffiliationChanged = $10;
const
	CE_CD_Superseded = $08;
const
	CE_CD_CessationOfOperation = $04;
const
	CE_CD_CertificateHold = $02;

type
  CE_CrlDistributionPointNameType = UInt32;
const
	CE_CDNT_FullName = 0;
	CE_CDNT_NameRelativeToCrlIssuer = 1;

type
  __embedded_dpn = record
    case Integer of
      0: (fullName: CE_GeneralNamesPtr);
      1: (rdn: CSSM_X509_RDN_PTR);
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

	CE_DistributionPointNamePtr = ^CE_DistributionPointName;
	CE_DistributionPointName = record
		nameType: CE_CrlDistributionPointNameType;
		dpn: __embedded_dpn;
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

{
 * The top-level CRLDistributionPoint.
 * All fields are optional; NULL pointers indicate absence. 
 }
type
	CE_CRLDistributionPointPtr = ^CE_CRLDistributionPoint;
	CE_CRLDistributionPoint = record
		distPointName: CE_DistributionPointNamePtr;
		reasonsPresent: CSSM_BOOL;
		reasons: CE_CrlDistReasonFlags;
		crlIssuer: CE_GeneralNamesPtr;
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
	CE_CRLDistPointsSyntaxPtr = ^CE_CRLDistPointsSyntax;
	CE_CRLDistPointsSyntax = record
		numDistPoints: UInt32;
		distPoints: CE_CRLDistributionPointPtr;
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

{ 
 * Authority Information Access and Subject Information Access.
 *
 * CSSM OID = CSSMOID_AuthorityInfoAccess
 * CSSM OID = CSSMOID_SubjectInfoAccess
 *
 * SubjAuthInfoAccessSyntax  ::=
 *		SEQUENCE SIZE (1..MAX) OF AccessDescription
 * 
 * AccessDescription  ::=  SEQUENCE (
 *		accessMethod          OBJECT IDENTIFIER,
 *		accessLocation        GeneralName  )
 }
type
	CE_AccessDescriptionPtr = ^CE_AccessDescription;
	CE_AccessDescription = record
		accessMethod: CSSM_OID;
		accessLocation: CE_GeneralName;
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
	CE_AuthorityInfoAccessPtr = ^CE_AuthorityInfoAccess;
	CE_AuthorityInfoAccess = record
		numAccessDescriptions: UInt32;
		accessDescriptions: CE_AccessDescriptionPtr;
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

{
 * Qualified Certificate Statement support, per RFC 3739.
 *
 * First, NameRegistrationAuthorities, a component of
 * SemanticsInformation; it's the same as a GeneralNames - 
 * a sequence of GeneralName. 
 }
type
  CE_NameRegistrationAuthoritiesPtr = ^CE_NameRegistrationAuthorities;
	CE_NameRegistrationAuthorities = CE_GeneralNames;
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

{
 * SemanticsInformation, identified as the qcType field
 * of a CE_QC_Statement for statementId value id-qcs-pkixQCSyntax-v2.
 * Both fields optional; at least one must be present. 
 }
type
	CE_SemanticsInformationPtr = ^CE_SemanticsInformation;
	CE_SemanticsInformation = record
		semanticsIdentifier: CSSM_OIDPtr;	
		nameRegistrationAuthorities: CE_NameRegistrationAuthoritiesPtr;
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

{ 
 * One Qualified Certificate Statement. 
 * The statementId OID is required; zero or one of (semanticsInfo, 
 * otherInfo) can be valid, depending on the value of statementId. 
 * For statementId id-qcs-pkixQCSyntax-v2 (CSSMOID_OID_QCS_SYNTAX_V2), 
 * the semanticsInfo field may be present; otherwise, DER-encoded
 * information may be present in otherInfo. Both semanticsInfo and
 * otherInfo are optional. 
 }
type
	CE_QC_StatementPtr = ^CE_QC_Statement;
	CE_QC_Statement = record
		statementId: CSSM_OID;
		semanticsInfo: CE_SemanticsInformationPtr;
		otherInfo: CSSM_DATAPtr;
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

{
 * The top-level Qualified Certificate Statements extension.
 }
type
	CE_QC_StatementsPtr = ^CE_QC_Statements;
	CE_QC_Statements = record
		numQCStatements: UInt32;
		qcStatements: CE_QC_StatementPtr;
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

{** CRL extensions **}

{
 * cRLNumber, an integer.
 *
 * CSSM OID = CSSMOID_CrlNumber
 }
type
	CE_CrlNumber = UInt32;

{
 * deltaCRLIndicator, an integer.
 *
 * CSSM OID = CSSMOID_DeltaCrlIndicator
 }
type
	CE_DeltaCrl = UInt32;

{
 * IssuingDistributionPoint
 *
 * id-ce-issuingDistributionPoint OBJECT IDENTIFIER ::= ( id-ce 28 )
 *
 * issuingDistributionPoint ::= SEQUENCE (
 *      distributionPoint       [0] DistributionPointName OPTIONAL,
 *		onlyContainsUserCerts   [1] BOOLEAN DEFAULT FALSE,
 *      onlyContainsCACerts     [2] BOOLEAN DEFAULT FALSE,
 *      onlySomeReasons         [3] ReasonFlags OPTIONAL,
 *      indirectCRL             [4] BOOLEAN DEFAULT FALSE )
 *
 * CSSM OID = CSSMOID_IssuingDistributionPoint
 }
type
	CE_IssuingDistributionPointPtr = ^CE_IssuingDistributionPoint;
	CE_IssuingDistributionPoint = record
		distPointName: CE_DistributionPointNamePtr;		// optional
		onlyUserCertsPresent: CSSM_BOOL;
		onlyUserCerts: CSSM_BOOL;
		onlyCACertsPresent: CSSM_BOOL;
		onlyCACerts: CSSM_BOOL;
		onlySomeReasonsPresent: CSSM_BOOL;
		onlySomeReasons: CE_CrlDistReasonFlags;
		indirectCrlPresent: CSSM_BOOL;
		indirectCrl: CSSM_BOOL;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

{
 * NameConstraints
 *
 * id-ce-nameConstraints OBJECT IDENTIFIER ::=  ( id-ce 30 )
 *
 *     NameConstraints ::= SEQUENCE (
 *          permittedSubtrees       [0]     GeneralSubtrees OPTIONAL,
 *          excludedSubtrees        [1]     GeneralSubtrees OPTIONAL )
 *
 *     GeneralSubtrees ::= SEQUENCE SIZE (1..MAX) OF GeneralSubtree
 *
 *     GeneralSubtree ::= SEQUENCE (
 *          base                    GeneralName,
 *          minimum         [0]     BaseDistance DEFAULT 0,
 *          maximum         [1]     BaseDistance OPTIONAL )
 *
 *     BaseDistance ::= INTEGER (0..MAX)
 }
type
  CE_GeneralSubtreePtr = ^CE_GeneralSubtree;
  CE_GeneralSubtree = record
		base: CE_GeneralNamesPtr;
		minimum: UInt32; // default=0
		maximumPresent: CSSM_BOOL;
		maximum: UInt32; // optional
  end;
  (* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

type
  CE_GeneralSubtreesPtr = ^CE_GeneralSubtrees;
  CE_GeneralSubtrees = record
		numSubtrees: UInt32;
		subtrees: CE_GeneralSubtreePtr;
  end;
  (* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

type
  CE_NameConstraintsPtr = ^CE_NameConstraints;
  CE_NameConstraints = record
		permitted: CE_GeneralSubtreesPtr; // optional
		excluded: CE_GeneralSubtreesPtr;  // optional
  end;
  (* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

{
 * PolicyMappings
 *
 * id-ce-policyMappings OBJECT IDENTIFIER ::=  ( id-ce 33 )
 *
 *     PolicyMappings ::= SEQUENCE SIZE (1..MAX) OF SEQUENCE (
 *          issuerDomainPolicy      CertPolicyId,
 *          subjectDomainPolicy     CertPolicyId )
 *
 * Note that both issuer and subject policy OIDs are required,
 * and are stored by value in this structure.
 }
type
  CE_PolicyMappingPtr = ^CE_PolicyMapping;
  CE_PolicyMapping = record
		issuerDomainPolicy: CSSM_OID;
		subjectDomainPolicy: CSSM_OID;
  end;
  (* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

type
  CE_PolicyMappingsPtr = ^CE_PolicyMappings;
  CE_PolicyMappings = record
		numPolicyMappings: UInt32;
		policyMappings: CE_PolicyMappingPtr;
  end;
  (* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

{
 * PolicyConstraints
 *
 * id-ce-policyConstraints OBJECT IDENTIFIER ::=  ( id-ce 36 )
 *
 *     PolicyConstraints ::= SEQUENCE (
 *          requireExplicitPolicy   [0]     SkipCerts OPTIONAL,
 *          inhibitPolicyMapping    [1]     SkipCerts OPTIONAL )
 *
 *      SkipCerts ::= INTEGER (0..MAX)
 }
type
  CE_PolicyConstraintsPtr = ^CE_PolicyConstraints;
  CE_PolicyConstraints = record
		requireExplicitPolicyPresent: CSSM_BOOL;
		requireExplicitPolicy: UInt32; // optional
		inhibitPolicyMappingPresent: CSSM_BOOL;
		inhibitPolicyMapping: UInt32;  // optional
  end;
  (* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)

{
 * InhibitAnyPolicy, an integer.
 *
 * CSSM OID = CSSMOID_InhibitAnyPolicy
 }
type
  CE_InhibitAnyPolicy = UInt32;
	(* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{
 * An enumerated list identifying one of the above per-extension
 * structs.
 }
type
  CE_DataType = UInt32;
const
	DT_AuthorityKeyID = 0;			// CE_AuthorityKeyID
	DT_SubjectKeyID = 1;			// CE_SubjectKeyID
	DT_KeyUsage = 2;				// CE_KeyUsage
	DT_SubjectAltName = 3;			// implies CE_GeneralName
	DT_IssuerAltName = 4;			// implies CE_GeneralName
	DT_ExtendedKeyUsage = 5;		// CE_ExtendedKeyUsage
	DT_BasicConstraints = 6;		// CE_BasicConstraints
	DT_CertPolicies = 7;			// CE_CertPolicies
	DT_NetscapeCertType = 8;		// CE_NetscapeCertType
	DT_CrlNumber = 9;				// CE_CrlNumber
	DT_DeltaCrl = 10;				// CE_DeltaCrl
	DT_CrlReason = 11;				// CE_CrlReason
	DT_CrlDistributionPoints = 12;	// CE_CRLDistPointsSyntax
	DT_IssuingDistributionPoint = 13;// CE_IssuingDistributionPoint
	DT_AuthorityInfoAccess = 14;		// CE_AuthorityInfoAccess
	DT_Other = 15;					// unknown, raw data as a CSSM_DATA
	DT_QC_Statements = 16;			// CE_QC_Statements
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	DT_NameConstraints = 17;			// CE_NameConstraints
	DT_PolicyMappings = 18;			// CE_PolicyMappings
	DT_PolicyConstraints = 19;		// CE_PolicyConstraints
	DT_InhibitAnyPolicy = 20;			// CE_InhibitAnyPolicy
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{
 * One unified representation of all the cert adn CRL extensions we know about.
 }
type
  CE_DataPtr = ^CE_Data;
  CE_Data = record
    case CE_DataType of
      DT_AuthorityKeyID: (authorityKeyID: CE_AuthorityKeyID);
      DT_SubjectKeyID: (subjectKeyID: CE_SubjectKeyID);
      DT_KeyUsage: (keyUsage: CE_KeyUsage);
      DT_SubjectAltName: (subjectAltName: CE_GeneralNames);
      DT_IssuerAltName: (issuerAltName: CE_GeneralNames);
      DT_ExtendedKeyUsage: (extendedKeyUsage: CE_ExtendedKeyUsage);
      DT_BasicConstraints: (basicConstraints: CE_BasicConstraints);
      DT_CertPolicies: (certPolicies: CE_CertPolicies);
      DT_NetscapeCertType: (netscapeCertType: CE_NetscapeCertType);
      DT_CrlNumber: (crlNumber: CE_CrlNumber);
      DT_DeltaCrl: (deltaCrl: CE_DeltaCrl);
      DT_CrlReason: (crlReason: CE_CrlReason);
      DT_CrlDistributionPoints: (crlDistPoints: CE_CRLDistPointsSyntax);
      DT_IssuingDistributionPoint: (issuingDistPoint: CE_IssuingDistributionPoint);
      DT_AuthorityInfoAccess: (authorityInfoAccess: CE_AuthorityInfoAccess);
      DT_QC_Statements: (qualifiedCertStatements: CE_QC_Statements);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      DT_NameConstraints: (nameConstraints: CE_NameConstraints);
      DT_PolicyMappings: (policyMappings: CE_PolicyMappings);
      DT_PolicyConstraints: (policyConstraints: CE_PolicyConstraints);
      DT_InhibitAnyPolicy: (inhibitAnyPolicy: CE_InhibitAnyPolicy);
      65535: (rawData: CSSM_DATA); // unknown, not decoded
  end;
  (* DEPRECATED_IN_MAC_OS_X_VERSION_10_7_AND_LATER *)
=======
      65535: (rawData: CSSM_DATA); // unknown, not decoded
  end;
>>>>>>> graemeg/cpstrnew
=======
      65535: (rawData: CSSM_DATA); // unknown, not decoded
  end;
>>>>>>> graemeg/cpstrnew
=======
      65535: (rawData: CSSM_DATA); // unknown, not decoded
  end;
>>>>>>> graemeg/cpstrnew
=======
      65535: (rawData: CSSM_DATA); // unknown, not decoded
  end;
>>>>>>> origin/cpstrnew


type
	CE_DataAndTypePtr = ^CE_DataAndType;
	CE_DataAndType = record
		typ: CE_DataType;
		extension: CE_Data;
		critical: CSSM_BOOL;
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
