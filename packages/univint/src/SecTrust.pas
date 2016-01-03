{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2002-2010,2012-2013 Apple Inc. All Rights Reserved.
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
 * Copyright (c) 2002-2009 Apple Inc. All Rights Reserved.
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
 * @APPLE_LICENSE_HEADER_END@
 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, August 2015 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 * 
 * @APPLE_LICENSE_HEADER_END@
 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
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
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit SecTrust;
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
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,SecBase,cssmtype,cssmapple,CFBase,CFArray,CFData,CFDate,CFDictionary;
=======
uses MacTypes,SecBase,cssmtype,cssmapple,CFBase,CFArray,CFData,CFDate;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,SecBase,cssmtype,cssmapple,CFBase,CFArray,CFData,CFDate;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,SecBase,cssmtype,cssmapple,CFBase,CFArray,CFData,CFDate;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,SecBase,cssmtype,cssmapple,CFBase,CFArray,CFData,CFDate;
>>>>>>> origin/cpstrnew
{$endc} {not MACOSALLINCLUDE}


{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @header SecTrust
    The functions and data types in SecTrust implement trust computation
    and allow the caller to apply trust decisions to the evaluation.
 }


{!
    @typedef SecTrustResultType
    @abstract Specifies the trust result type.
    @discussion SecTrustResultType results have two dimensions.  They specify
    both whether evaluation suceeded and whether this is because of a user
    decision.  The commonly expected result is kSecTrustResultUnspecified,
    which indicates a positive result that wasn't decided by the user.  The
    common failure is kSecTrustResultRecoverableTrustFailure, which means a
    negative result.  kSecTrustResultProceed and kSecTrustResultDeny are the
    positive and negative result respectively when decided by the user.  User
    decisions are persisted through the use of SecTrustCopyExceptions() and
    SecTrustSetExceptions().  Finally, kSecTrustResultFatalTrustFailure is a
    negative result that must not be circumvented.
    @constant kSecTrustResultInvalid Indicates an invalid setting or result.
    This result usually means that SecTrustEvaluate has not yet been called.
    @constant kSecTrustResultProceed Indicates you may proceed.  This value
    may be returned by the SecTrustEvaluate function or stored as part of
    the user trust settings.
    @constant kSecTrustResultConfirm Indicates confirmation with the user
    is required before proceeding.  Important: this value is no longer returned
    or supported by SecTrustEvaluate or the SecTrustSettings API starting in
    OS X 10.5; its use is deprecated in OS X 10.9 and later, as well as in iOS.
    @constant kSecTrustResultDeny Indicates a user-configured deny; do not
    proceed. This value may be returned by the SecTrustEvaluate function
    or stored as part of the user trust settings.
    @constant kSecTrustResultUnspecified Indicates the evaluation succeeded
    and the certificate is implicitly trusted, but user intent was not
    explicitly specified.  This value may be returned by the SecTrustEvaluate
    function or stored as part of the user trust settings.
    @constant kSecTrustResultRecoverableTrustFailure Indicates a trust policy
    failure which can be overridden by the user.  This value may be returned
    by the SecTrustEvaluate function but not stored as part of the user
    trust settings.
    @constant kSecTrustResultFatalTrustFailure Indicates a trust failure
    which cannot be overridden by the user.  This value may be returned by the
    SecTrustEvaluate function but not stored as part of the user trust
    settings.
    @constant kSecTrustResultOtherError Indicates a failure other than that
    of trust evaluation. This value may be returned by the SecTrustEvaluate
    function but not stored as part of the user trust settings.
 }

type
	SecTrustResultType = UInt32;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	@header SecTrust
	The functions and data types in SecTrust implement trust computation and allow the user to apply trust decisions to the trust configuration.
}


{!
	@typedef SecTrustResultType
	@abstract Specifies the trust result type.
	@constant kSecTrustResultInvalid Indicates an invalid setting or result.
	@constant kSecTrustResultProceed Indicates you may proceed.  This value may be returned by the SecTrustEvaluate function or stored as part of the user trust settings. 
	@constant kSecTrustResultConfirm Indicates confirmation with the user is required before proceeding.  This value may be returned by the SecTrustEvaluate function or stored as part of the user trust settings. 
	@constant kSecTrustResultDeny Indicates a user-configured deny; do not proceed. This value may be returned by the SecTrustEvaluate function or stored as part of the user trust settings. 
	@constant kSecTrustResultUnspecified Indicates user intent is unknown. This value may be returned by the SecTrustEvaluate function or stored as part of the user trust settings. 
	@constant kSecTrustResultRecoverableTrustFailure Indicates a trust framework failure; retry after fixing inputs. This value may be returned by the SecTrustEvaluate function but not stored as part of the user trust settings. 
	@constant kSecTrustResultFatalTrustFailure Indicates a trust framework failure; no "easy" fix. This value may be returned by the SecTrustEvaluate function but not stored as part of the user trust settings.
	@constant kSecTrustResultOtherError Indicates a failure other than that of trust evaluation. This value may be returned by the SecTrustEvaluate function but not stored as part of the user trust settings.
 }
type
	SecTrustResultType = UInt32;

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
const
	kSecTrustResultInvalid = 0;
	kSecTrustResultProceed = 1;
	kSecTrustResultConfirm = 2;
	kSecTrustResultDeny = 3;
	kSecTrustResultUnspecified = 4;
	kSecTrustResultRecoverableTrustFailure = 5;
	kSecTrustResultFatalTrustFailure = 6;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kSecTrustResultOtherError = 7; (* CF_ENUM_DEPRECATED(10_0, 10_9, NA, NA) *)

{!
    @typedef SecTrustRef
    @abstract CFType used for performing X.509 certificate trust evaluations.
 }
type
	SecTrustRef = ^OpaqueSecTrustRef; { an opaque type }
	{ already defined in SecBase 
	__SecTrust = record end; }

{!
    @enum Trust Property Constants
    @discussion Predefined key constants used to obtain values in a
        per-certificate dictionary of trust evaluation results,
        as retrieved from a call to SecTrustCopyProperties.
    @constant kSecPropertyTypeTitle Specifies a key whose value is a
        CFStringRef containing the title (display name) of this certificate.
    @constant kSecPropertyTypeError Specifies a key whose value is a
        CFStringRef containing the reason for a trust evaluation failure.
 }
var kSecPropertyTypeTitle: CFTypeRef; external name '_kSecPropertyTypeTitle'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_7_0) *)
var kSecPropertyTypeError: CFTypeRef; external name '_kSecPropertyTypeError'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_7_0) *)

{!
    @enum Trust Result Constants
    @discussion Predefined key constants used to obtain values in a
        dictionary of trust evaluation results for a certificate chain,
        as retrieved from a call to SecTrustCopyResult.
    @constant kSecTrustEvaluationDate
        This key will be present if a trust evaluation has been performed
        and results are available. Its value is a CFDateRef representing
        when the evaluation for this trust object took place.
    @constant kSecTrustExtendedValidation
        This key will be present and have a value of kCFBooleanTrue
        if this chain was validated for EV.
    @constant kSecTrustOrganizationName
        Organization name field of subject of leaf certificate. This
        field is meant to be displayed to the user as the validated
        name of the company or entity that owns the certificate if the
        kSecTrustExtendedValidation key is present.
    @constant kSecTrustResultValue
        This key will be present if a trust evaluation has been performed.
        Its value is a CFNumberRef representing the SecTrustResultType result
        for the evaluation.
    @constant kSecTrustRevocationChecked
        This key will be present iff this chain had its revocation checked.
        The value will be a kCFBooleanTrue if revocation checking was
        successful and none of the certificates in the chain were revoked.
        The value will be kCFBooleanFalse if no current revocation status
        could be obtained for one or more certificates in the chain due
        to connection problems or timeouts.  This is a hint to a client
        to retry revocation checking at a later time.
    @constant kSecTrustRevocationValidUntilDate
        This key will be present iff kSecTrustRevocationChecked has a
        value of kCFBooleanTrue. The value will be a CFDateRef representing
        the earliest date at which the revocation info for one of the
        certificates in this chain might change.
 }
var kSecTrustEvaluationDate: CFTypeRef; external name '_kSecTrustEvaluationDate'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)
var kSecTrustExtendedValidation: CFTypeRef; external name '_kSecTrustExtendedValidation'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)
var kSecTrustOrganizationName: CFTypeRef; external name '_kSecTrustOrganizationName'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)
var kSecTrustResultValue: CFTypeRef; external name '_kSecTrustResultValue'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)
var kSecTrustRevocationChecked: CFTypeRef; external name '_kSecTrustRevocationChecked'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)
var kSecTrustRevocationValidUntilDate: CFTypeRef; external name '_kSecTrustRevocationValidUntilDate'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)


{!
    @function SecTrustGetTypeID
    @abstract Returns the type identifier of SecTrust instances.
    @result The CFTypeID of SecTrust instances.
 }
function SecTrustGetTypeID: CFTypeID; external name '_SecTrustGetTypeID';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{!
    @function SecTrustCreateWithCertificates
    @abstract Creates a trust object based on the given certificates and
    policies.
    @param certificates The group of certificates to verify.  This can either
    be a CFArrayRef of SecCertificateRef objects or a single SecCertificateRef
    @param policies An array of one or more policies. You may pass a
    SecPolicyRef to represent a single policy.
    @param trust On return, a pointer to the trust management reference.
    @result A result code.  See "Security Error Codes" (SecBase.h).
    @discussion If multiple policies are passed in, all policies must verify
    for the chain to be considered valid.
 }
function SecTrustCreateWithCertificates( certificates: CFTypeRef; policies: CFTypeRef; var trustref: SecTrustRef ): OSStatus; external name '_SecTrustCreateWithCertificates';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{!
    @function SecTrustSetPolicies
    @abstract Set the policies for which trust should be verified.
    @param trust A trust reference.
    @param policies An array of one or more policies. You may pass a
    SecPolicyRef to represent a single policy.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function will invalidate the existing trust result,
    requiring a fresh evaluation for the newly-set policies.
 }
function SecTrustSetPolicies( trust: SecTrustRef; policies: CFTypeRef ): OSStatus; external name '_SecTrustSetPolicies';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_6_0) *)

{!
    @function SecTrustCopyPolicies
    @abstract Returns an array of policies used for this evaluation.
    @param trust  A reference to a trust object.
    @param policies On return, an array of policies used by this trust.
    Call the CFRelease function to release this reference.
    @result A result code. See "Security Error Codes" (SecBase.h).
 }
function SecTrustCopyPolicies( trust: SecTrustRef; var policies: CFArrayRef ): OSStatus; external name '_SecTrustCopyPolicies';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_7_0) *)

{!
    @function SecTrustSetNetworkFetchAllowed
    @abstract Specifies whether a trust evaluation is permitted to fetch missing
    intermediate certificates from the network.
    @param trust A trust reference.
    @param allowFetch If true, and a certificate's issuer is not present in the
    trust reference but its network location is known, the evaluation is permitted
    to attempt to download it automatically. Pass false to disable network fetch
    for this trust evaluation.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion By default, network fetch of missing certificates is enabled if
    the trust evaluation includes the SSL policy, otherwise it is disabled.
 }
function SecTrustSetNetworkFetchAllowed( trust: SecTrustRef; allowFetch: Boolean ): OSStatus; external name '_SecTrustSetNetworkFetchAllowed';
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)

{!
    @function SecTrustGetNetworkFetchAllowed
    @abstract Returns whether a trust evaluation is permitted to fetch missing
    intermediate certificates from the network.
    @param trust A trust reference.
    @param allowFetch On return, the boolean pointed to by this parameter is
    set to true if the evaluation is permitted to download missing certificates.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion By default, network fetch of missing certificates is enabled if
    the trust evaluation includes the SSL policy, otherwise it is disabled.
 }
function SecTrustGetNetworkFetchAllowed( trust: SecTrustRef; var allowFetch: Boolean ): OSStatus; external name '_SecTrustGetNetworkFetchAllowed';
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)

{!
    @function SecTrustSetAnchorCertificates
    @abstract Sets the anchor certificates for a given trust.
    @param trust A reference to a trust object.
    @param anchorCertificates An array of anchor certificates.
    @result A result code.  See "Security Error Codes" (SecBase.h).
    @discussion Calling this function without also calling
    SecTrustSetAnchorCertificatesOnly() will disable trusting any
    anchors other than the ones in anchorCertificates.
 }
function SecTrustSetAnchorCertificates( trust: SecTrustRef; anchorCertificates: CFArrayRef ): OSStatus; external name '_SecTrustSetAnchorCertificates';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{!
    @function SecTrustSetAnchorCertificatesOnly
    @abstract Reenables trusting anchor certificates in addition to those
    passed in via the SecTrustSetAnchorCertificates API.
    @param trust A reference to a trust object.
    @param anchorCertificatesOnly If true, disables trusting any anchors other
    than the ones passed in via SecTrustSetAnchorCertificates().  If false,
    the built in anchor certificates are also trusted.
    @result A result code.  See "Security Error Codes" (SecBase.h).
 }
function SecTrustSetAnchorCertificatesOnly( trust: SecTrustRef; anchorCertificatesOnly: Boolean ): OSStatus; external name '_SecTrustSetAnchorCertificatesOnly';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_2_0) *)

{!
    @function SecTrustCopyCustomAnchorCertificates
    @abstract Returns an array of custom anchor certificates used by a given
    trust, as set by a prior call to SecTrustSetAnchorCertificates, or NULL if
    no custom anchors have been specified.
    @param trust  A reference to a trust object.
    @param anchors On return, an array of custom anchor certificates (roots)
    used by this trust, or NULL if no custom anchors have been specified. Call
    the CFRelease function to release this reference.
    @result A result code. See "Security Error Codes" (SecBase.h).
 }
function SecTrustCopyCustomAnchorCertificates( trust: SecTrustRef; var anchors: CFArrayRef ): OSStatus; external name '_SecTrustCopyCustomAnchorCertificates';
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_7_0) *)

{!
    @function SecTrustSetVerifyDate
    @abstract Set the date for which the trust should be verified.
    @param trust A reference to a trust object.
    @param verifyDate The date for which to verify trust.
    @result A result code.  See "Security Error Codes" (SecBase.h).
    @discussion This function lets you evaluate certificate validity for a
    given date (for example, to determine if a signature was valid on the date
    it was signed, even if the certificate has since expired.) If this function
    is not called, the time at which SecTrustEvaluate() is called is used
    implicitly as the verification time.
 }
function SecTrustSetVerifyDate( trust: SecTrustRef; verifyDate: CFDateRef ): OSStatus; external name '_SecTrustSetVerifyDate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{!
    @function SecTrustGetVerifyTime
    @abstract Returns the verify time.
    @param trust A reference to the trust object being verified.
    @result A CFAbsoluteTime value representing the time at which certificates
    should be checked for validity.
    @discussion This function retrieves the verification time for the given
    trust reference, as set by a prior call to SecTrustSetVerifyDate(). If the
    verification time has not been set, this function returns a value of 0,
    indicating that the current date/time is implicitly used for verification.
 }
function SecTrustGetVerifyTime( trust: SecTrustRef ): CFAbsoluteTime; external name '_SecTrustGetVerifyTime';
(* __OSX_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_2_0) *)

{!
    @function SecTrustEvaluate
    @abstract Evaluates a trust reference synchronously.
    @param trust A reference to the trust object to evaluate.
    @param result A pointer to a result type.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function will completely evaluate trust before returning,
    possibly including network access to fetch intermediate certificates or to
    perform revocation checking. Since this function can block during those
    operations, you should call it from within a function that is placed on a
    dispatch queue, or in a separate thread from your application's main
    run loop. Alternatively, you can use the SecTrustEvaluateAsync function.
 }
function SecTrustEvaluate( trust: SecTrustRef; var result: SecTrustResultType ): OSStatus; external name '_SecTrustEvaluate';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)


{!
    @function SecTrustGetTrustResult
    @param trust A reference to a trust object.
    @param result A pointer to the result from the most recent call to
    SecTrustEvaluate for this trust reference. If SecTrustEvaluate has not been
    called or trust parameters have changed, the result is kSecTrustResultInvalid.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function replaces SecTrustGetResult for the purpose of
    obtaining the current evaluation result of a given trust reference.
 }
function SecTrustGetTrustResult( trust: SecTrustRef; var result: SecTrustResultType ): OSStatus; external name '_SecTrustGetTrustResult';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_7_0) *)

{!
    @function SecTrustCopyPublicKey
    @abstract Return the public key for a leaf certificate after it has
    been evaluated.
    @param trust A reference to the trust object which has been evaluated.
    @result The certificate's public key, or NULL if it the public key could
    not be extracted (this can happen with DSA certificate chains if the
    parameters in the chain cannot be found).  The caller is responsible
    for calling CFRelease on the returned key when it is no longer needed.
 }
function SecTrustCopyPublicKey( trust: SecTrustRef ): SecKeyRef; external name '_SecTrustCopyPublicKey';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_2_0) *)

{!
    @function SecTrustGetCertificateCount
    @abstract Returns the number of certificates in an evaluated certificate
    chain.
    @param trust A reference to a trust object.
    @result The number of certificates in the trust chain, including the anchor.
    @discussion Important: if the trust reference has not yet been evaluated,
    this function will evaluate it first before returning. If speed is critical,
    you may want to call SecTrustGetTrustResult first to make sure that a
    result other than kSecTrustResultInvalid is present for the trust object.
 }
function SecTrustGetCertificateCount( trust: SecTrustRef ): CFIndex; external name '_SecTrustGetCertificateCount';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_2_0) *)

{!
    @function SecTrustGetCertificateAtIndex
    @abstract Returns a certificate from the trust chain.
    @param trust Reference to a trust object.
    @param ix The index of the requested certificate.  Indices run from 0
    (leaf) to the anchor (or last certificate found if no anchor was found).
    The leaf cert (index 0) is always present regardless of whether the trust
    reference has been evaluated or not.
    @result A SecCertificateRef for the requested certificate.
 }
function SecTrustGetCertificateAtIndex( trust: SecTrustRef; ix: CFIndex ): SecCertificateRef; external name '_SecTrustGetCertificateAtIndex';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_2_0) *)

{!
    @function SecTrustCopyExceptions
    @abstract Returns an opaque cookie which will allow future evaluations
    of the current certificate to succeed.
    @param trust A reference to an evaluated trust object.
    @result An opaque cookie which when passed to SecTrustSetExceptions() will
    cause a call to SecTrustEvaluate() return kSecTrustResultProceed.  This
    will happen upon subsequent evaluation of the current certificate unless
    some new error starts happening that wasn't being reported when the cookie
    was returned from this function (for example, if the certificate expires
    then evaluation will start failing again until a new cookie is obtained.)
    @discussion Normally this API should only be called once the errors have
    been presented to the user and the user decided to trust the current
    certificate chain regardless of the errors being presented, for the
    current application/server/protocol combination.
 }
function SecTrustCopyExceptions( trust: SecTrustRef ): CFDataRef; external name '_SecTrustCopyExceptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_4_0) *)

{!
    @function SecTrustSetExceptions
    @abstract Set a trust cookie to be used for evaluating this certificate chain.
    @param trust A reference to a trust object.
    @param exceptions An exceptions cookie as returned by a call to
    SecTrustCopyExceptions() in the past.
    @result Upon calling SecTrustEvaluate(), any failures that where present at the
    time the exceptions object was created are ignored, and instead of returning
    kSecTrustResultRecoverableTrustFailure, kSecTrustResultProceed will be returned
    (if the certificate for which exceptions was created matches the current leaf
    certificate).
    @result Returns true if the exceptions cookies was valid and matches the current
    leaf certificate, false otherwise.  This function will invalidate the existing
    trust result, requiring a subsequent evaluation for the newly-set exceptions.
    Note that this function returning true doesn't mean the caller can skip calling
    SecTrustEvaluate, as there may be new errors since the exceptions cookie was
    created (for example, a certificate may have subsequently expired.)
    @discussion Clients of this interface will need to establish the context of this
    exception to later decide when this exception cookie is to be used.
    Examples of this context would be the server we are connecting to, the ssid
    of the wireless network for which this cert is needed, the account for which
    this cert should be considered valid, and so on.
 }
function SecTrustSetExceptions( trust: SecTrustRef; exceptions: CFDataRef ): CBool; external name '_SecTrustSetExceptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_4_0) *)

{!
    @function SecTrustCopyProperties
    @abstract Return a property array for this trust evaluation.
    @param trust A reference to a trust object. If the trust has not been
    evaluated, the returned property array will be empty.
    @result A property array. It is the caller's responsibility to CFRelease
    the returned array when it is no longer needed.
    @discussion This function returns an ordered array of CFDictionaryRef
    instances for each certificate in the chain. Indices run from 0 (leaf) to
    the anchor (or last certificate found if no anchor was found.) See the
    "Trust Property Constants" section for a list of currently defined keys.
 }
function SecTrustCopyProperties( trust: SecTrustRef ): CFArrayRef; external name '_SecTrustCopyProperties';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_2_0) *)

{!
    @function SecTrustCopyResult
    @abstract Returns a dictionary containing information about the
    evaluated certificate chain for use by clients.
    @param trust A reference to a trust object.
    @result A dictionary with various fields that can be displayed to the user,
    or NULL if no additional info is available or the trust has not yet been
    validated.  The caller is responsible for calling CFRelease on the value
    returned when it is no longer needed.
    @discussion Returns a dictionary for the overall trust evaluation. See the
    "Trust Result Constants" section for a list of currently defined keys.
 }
function SecTrustCopyResult( trust: SecTrustRef ): CFDictionaryRef; external name '_SecTrustCopyResult';
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)

{!
    @function SecTrustSetOCSPResponse
    @abstract Attach OCSPResponse data to a trust object.
    @param trust A reference to a trust object.
    @param responseData This may be either a CFData object containing a single
    DER-encoded OCSPResponse (per RFC 2560), or a CFArray of these.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion Allows the caller to provide OCSPResponse data (which may be
    obtained during a TLS/SSL handshake, per RFC 3546) as input to a trust
    evaluation. If this data is available, it can obviate the need to contact
    an OCSP server for current revocation information.
 }
function SecTrustSetOCSPResponse( trust: SecTrustRef; responseData: CFTypeRef ): OSStatus; external name '_SecTrustSetOCSPResponse';
(* __OSX_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)


{
 *  Legacy functions (OS X only)
 }
{$ifc TARGET_OS_MAC}

{!
    @typedef SecTrustUserSetting
    @abstract Specifies a user-specified trust setting value.
    @discussion Deprecated in OS X 10.9. User trust settings are managed by
    functions in SecTrustSettings.h (starting with OS X 10.5), and by the
    SecTrustCopyExceptions and SecTrustSetExceptions functions (starting with
    iOS 4 and OS X 10.9). The latter two functions are recommended for both OS X
    and iOS, as they avoid the need to explicitly specify these values.
 }
type
  SecTrustUserSetting = SecTrustResultType;
  (* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_9, __IPHONE_NA, __IPHONE_NA) *)

{!
    @typedef SecTrustOptionFlags
    @abstract Options for customizing trust evaluation.
    @constant kSecTrustOptionAllowExpired Allow expired certificates.
    @constant kSecTrustOptionLeafIsCA Allow CA as leaf certificate.
    @constant kSecTrustOptionFetchIssuerFromNet Allow network fetch of CA cert.
    @constant kSecTrustOptionAllowExpiredRoot Allow expired roots.
    @constant kSecTrustOptionRequireRevPerCert Require positive revocation
    check per certificate.
    @constant kSecTrustOptionUseTrustSettings Use TrustSettings instead of
    anchors.
    @constant kSecTrustOptionImplicitAnchors Properly self-signed certs are
    treated as anchors implicitly.
 }
type
	SecTrustOptionFlagsPtr = ^SecTrustOptionFlags;
	SecTrustOptionFlags = UInt32;
const
	kSecTrustOptionAllowExpired = $00000001;
	kSecTrustOptionLeafIsCA = $00000002;
	kSecTrustOptionFetchIssuerFromNet = $00000004;
	kSecTrustOptionAllowExpiredRoot = $00000008;
	kSecTrustOptionRequireRevPerCert = $00000010;
	kSecTrustOptionUseTrustSettings = $00000020;
	kSecTrustOptionImplicitAnchors = $00000040;

{!
    @function SecTrustSetOptions
    @abstract Sets optional flags for customizing a trust evaluation.
    @param trustRef A trust reference.
    @param options Flags to change evaluation behavior for this trust.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function is not available on iOS. Use SecTrustSetExceptions
    and SecTrustCopyExceptions to modify default trust results, and
    SecTrustSetNetworkFetchAllowed to specify whether missing CA certificates
    can be fetched from the network.
 }
function SecTrustSetOptions( trustRef: SecTrustRef; options: SecTrustOptionFlags ): OSStatus; external name '_SecTrustSetOptions';
(* __OSX_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_NA) *)

{!
    @function SecTrustSetParameters
    @abstract Sets the action and action data for a trust object.
    @param trustRef The reference to the trust to change.
    @param action A trust action.
    @param actionData A reference to data associated with this action.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function is deprecated in OS X 10.7 and later, where it
    was replaced by SecTrustSetOptions, and is not available on iOS. Your code
    should use SecTrustSetExceptions and SecTrustCopyExceptions to modify default
    trust results, and SecTrustSetNetworkFetchAllowed to specify whether missing
    CA certificates can be fetched from the network.
 }
function SecTrustSetParameters( trustRef: SecTrustRef; action: CSSM_TP_ACTION; actionData: CFDataRef ): OSStatus; external name '_SecTrustSetParameters';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

{!
    @function SecTrustSetKeychains
    @abstract Sets the keychains for a given trust object.
    @param trust A reference to a trust object.
    @param keychainOrArray A reference to an array of keychains to search, a
    single keychain, or NULL to use the default keychain search list.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion By default, the user's keychain search list and the system
    anchors keychain are searched for certificates to complete the chain. You
    can specify a zero-element array if you do not want any keychains searched.
    Note: this function is not applicable to iOS.
 }
function SecTrustSetKeychains( trust: SecTrustRef; keychainOrArray: CFTypeRef ): OSStatus; external name '_SecTrustSetKeychains';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{!
    @function SecTrustGetResult
    @abstract Returns detailed information on the outcome of an evaluation.
    @param trustRef A reference to a trust object.
    @param result A pointer to the result from the call to SecTrustEvaluate.
    @param certChain On return, a pointer to the certificate chain used to
    validate the input certificate. Call the CFRelease function to release
    this pointer.
    @param statusChain On return, a pointer to the status of the certificate
    chain. Do not attempt to free this pointer; it remains valid until the
    trust is destroyed or the next call to SecTrustEvaluate.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function is deprecated in OS X 10.7 and later,
    and is not available on iOS.
    To get the complete certificate chain, use SecTrustGetCertificateCount and
    SecTrustGetCertificateAtIndex. To get detailed status information for each
    certificate, use SecTrustCopyProperties. To get the overall trust result
    for the evaluation, use SecTrustGetTrustResult.
 }
function SecTrustGetResult( trustRef: SecTrustRef; var result: SecTrustResultType; var certChain: CFArrayRef; statusChain: CSSM_TP_APPLE_EVIDENCE_INFOArrayPtr): OSStatus; external name '_SecTrustGetResult';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

{!
    @function SecTrustGetCssmResult
    @abstract Gets the CSSM trust result.
    @param trust A reference to a trust.
    @param result On return, a pointer to the CSSM trust result.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function is deprecated in OS X 10.7 and later,
    and is not available on iOS.
    To get detailed status information for each certificate, use
    SecTrustCopyProperties. To get the overall trust result for the evaluation,
    use SecTrustGetTrustResult.
 }
function SecTrustGetCssmResult( trust: SecTrustRef; var result: CSSM_TP_VERIFY_CONTEXT_RESULT_PTR ): OSStatus; external name '_SecTrustGetCssmResult';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

{!
    @function SecTrustGetCssmResultCode
    @abstract Gets the result code from the most recent call to SecTrustEvaluate
    for the specified trust.
    @param trust A reference to a trust.
    @param resultCode On return, the result code produced by the most recent
    evaluation of the given trust (cssmerr.h). The value of resultCode is
    undefined if SecTrustEvaluate has not been called.
    @result A result code. See "Security Error Codes" (SecBase.h). Returns
    errSecTrustNotAvailable if SecTrustEvaluate has not been called for the
    specified trust.
    @discussion This function is deprecated in OS X 10.7 and later,
    and is not available on iOS.
    To get detailed status information for each certificate, use
    SecTrustCopyProperties. To get the overall trust result for the evaluation,
    use SecTrustGetTrustResult.
 }
function SecTrustGetCssmResultCode( trust: SecTrustRef; var resultCode: OSStatus ): OSStatus; external name '_SecTrustGetCssmResultCode';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

{!
    @function SecTrustGetTPHandle
    @abstract Gets the CSSM trust handle
    @param trust A reference to a trust.
    @param handle On return, a CSSM trust handle.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function is deprecated in OS X 10.7 and later.
 }
function SecTrustGetTPHandle( trust: SecTrustRef; var handle: CSSM_TP_HANDLE ): OSStatus; external name '_SecTrustGetTPHandle';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_7, __IPHONE_NA, __IPHONE_NA) *)

{!
    @function SecTrustCopyAnchorCertificates
    @abstract Returns an array of default anchor (root) certificates used by
    the system.
    @param anchors On return, an array containing the system's default anchors
    (roots). Call the CFRelease function to release this pointer.
    @result A result code. See "Security Error Codes" (SecBase.h).
    @discussion This function is not available on iOS, as certificate data
    for system-trusted roots is currently unavailable on that platform.
 }
function SecTrustCopyAnchorCertificates( var anchors: CFArrayRef ): OSStatus; external name '_SecTrustCopyAnchorCertificates';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kSecTrustResultOtherError = 7;

{!
	@typedef SecTrustUserSetting
	@abstract Specifies user-specified trust settings.
}
type
	SecTrustUserSetting = SecTrustResultType;

{!
	@typedef SecTrustRef
	@abstract A pointer to an opaque trust management structure.
}
type
	SecTrustRef = ^OpaqueSecTrustRef; { an opaque type }
	{ already defined in SecBase 
	OpaqueSecTrustRef = record end; }

{!
	@function SecTrustGetTypeID
	@abstract Returns the type identifier of SecTrust instances.
	@result The CFTypeID of SecTrust instances.
}
function SecTrustGetTypeID: CFTypeID; external name '_SecTrustGetTypeID';

{!
	@function SecTrustCreateWithCertificates
	@abstract Creates a trust based on the given certificates and policies.
    @param certificates The group of certificates to verify.
    @param policies An array of one or more policies. You may pass a SecPolicyRef to represent a single policy.
	@param trustRef On return, a pointer to the trust management reference.
	@result A result code. See "Security Error Codes" (SecBase.h).
}
function SecTrustCreateWithCertificates( certificates: CFArrayRef; policies: CFTypeRef; var trustRef: SecTrustRef ): OSStatus; external name '_SecTrustCreateWithCertificates';

{!
    @function SecTrustSetPolicies
    @abstract Set (replace) the set of policies to evaluate.
    @param trust The reference to the trust to change.
    @param policies An array of one or more policies. A single SecPolicyRef may also be passed, representing an array of one policy.
    @result A result code. See "Security Error Codes" (SecBase.h).
}    
function SecTrustSetPolicies( trust: SecTrustRef; policies: CFTypeRef ): OSStatus; external name '_SecTrustSetPolicies';

{$ifc TARGET_OS_MAC}
{!
	@function SecTrustSetParameters
	@abstract Sets the action and action data for a trust.
	@param trustRef The reference to the trust to change.
	@param action A CSSM trust action.
	@param actionData A reference to action data.
	@result A result code. See "Security Error Codes" (SecBase.h).
 }
function SecTrustSetParameters( trustRef: SecTrustRef; action: CSSM_TP_ACTION; actionData: CFDataRef ): OSStatus; external name '_SecTrustSetParameters';
{$endc} {TARGET_OS_MAC}

{!
	@function SecTrustSetAnchorCertificates
	@abstract Sets the anchor certificates for a given trust.
	@param trust A reference to a trust.
	@param anchorCertificates An array of anchor certificates.
	@result A result code. See "Security Error Codes" (SecBase.h).
}
function SecTrustSetAnchorCertificates( trust: SecTrustRef; anchorCertificates: CFArrayRef ): OSStatus; external name '_SecTrustSetAnchorCertificates';

{!
	@function SecTrustSetAnchorCertificatesOnly
	@abstract Reenables trusting anchor certificates in addition to those passed in
    via the SecTrustSetAnchorCertificates API.
	@param trust A reference to a trust object.
	@param anchorCertificatesOnly If true, disables trusting any anchors other
    than the ones passed in via SecTrustSetAnchorCertificates().  If false,
    the built in anchor certificates are also trusted.
	@result A result code.  See "Security Error Codes" (SecBase.h).
}
function SecTrustSetAnchorCertificatesOnly( trust: SecTrustRef; anchorCertificatesOnly: Boolean ): OSStatus; external name '_SecTrustSetAnchorCertificatesOnly';

{!
	@function SecTrustSetKeychains
	@abstract Sets the keychains for a given trust.
	@param trust A reference to a trust.
    @param keychainOrArray A reference to an array of keychains to search, a single keychain or NULL to search the user's default keychain search list.
	@result A result code. See "Security Error Codes" (SecBase.h).
}
function SecTrustSetKeychains( trust: SecTrustRef; keychainOrArray: CFTypeRef ): OSStatus; external name '_SecTrustSetKeychains';

{!
	@function SecTrustSetVerifyDate
	@abstract Specifies the date for verification of a given trust.
	@param trust A reference to the trust to verify.
	@param verifyDate The date to verify.
	@result A result code. See "Security Error Codes" (SecBase.h).
}
function SecTrustSetVerifyDate( trust: SecTrustRef; verifyDate: CFDateRef ): OSStatus; external name '_SecTrustSetVerifyDate';

{!
	@function SecTrustGetVerifyTime
	@abstract Returns the verify time.
	@param trust A reference to the trust object to verify.
	@result A CFAbsoluteTime value representing the time at which certificates
	should be checked for validity.
}
function SecTrustGetVerifyTime( trust: SecTrustRef ): CFAbsoluteTime; external name '_SecTrustGetVerifyTime';

{!
	@function SecTrustEvaluate
	@abstract Evaluates a trust.
	@param trust A reference to the trust to evaluate.
	@param result A pointer to a result type.
	@result A result code. See "Security Error Codes" (SecBase.h).	
}
function SecTrustEvaluate( trust: SecTrustRef; var result: SecTrustResultType ): OSStatus; external name '_SecTrustEvaluate';

{$ifc TARGET_OS_MAC}
{!
	@function SecTrustGetResult
	@abstract Returns detail information on the outcome of a call to SecTrustEvaluate.
	@param trustRef A reference to a trust.
	@param result A pointer to the result from the call to SecTrustEvaluate.
	@param certChain On return, a pointer to the certificate chain used to validate the input certificate. Call the CFRelease function to release this pointer.
	@param statusChain On return, a pointer to the status of the certificate chain. Do not attempt to free this pointer; it remains valid until the trust is destroyed or the next call to SecTrustEvaluate.
	@result A result code. See "Security Error Codes" (SecBase.h).
}
function SecTrustGetResult( trustRef: SecTrustRef; var result: SecTrustResultType; var certChain: CFArrayRef; statusChain: CSSM_TP_APPLE_EVIDENCE_INFOArrayPtr): OSStatus; external name '_SecTrustGetResult';

{!
	@function SecTrustGetCssmResult
	@abstract Gets the CSSM trust result.
	@param trust A reference to a trust.
	@param result On return, a pointer to the CSSM trust result.
	@result A result code. See "Security Error Codes" (SecBase.h).
}
function SecTrustGetCssmResult( trust: SecTrustRef; var result: CSSM_TP_VERIFY_CONTEXT_RESULT_PTR ): OSStatus; external name '_SecTrustGetCssmResult';

{!
	@function SecTrustGetCssmResultCode
	@abstract Gets the result code from the most recent call to SecTrustEvaluate for the specified trust.
	@param trust A reference to a trust.
	@param resultCode On return, the result code produced by the most recent evaluation of the given trust (cssmerr.h). The value of resultCode is undefined if SecTrustEvaluate has not been called.
	@result A result code. See "Security Error Codes" (SecBase.h). Returns errSecTrustNotAvailable if SecTrustEvaluate has not been called for the specified trust.
}
function SecTrustGetCssmResultCode( trust: SecTrustRef; var resultCode: OSStatus ): OSStatus; external name '_SecTrustGetCssmResultCode';

{!
	@function SecTrustGetTPHandle
	@abstract Gets the CSSM trust handle
	@param trust A reference to a trust.
	@param handle On return, a CSSM trust handle.
	@result A result code. See "Security Error Codes" (SecBase.h).
}
function SecTrustGetTPHandle( trust: SecTrustRef; var handle: CSSM_TP_HANDLE ): OSStatus; external name '_SecTrustGetTPHandle';
{$endc} {TARGET_OS_MAC}

{!
    @function SecTrustCopyPolicies
    @abstract Returns an array of policies used by a given trust.
    @param trust  A reference to a trust.
    @param policies On return, an array of policies used by this trust. Call the CFRelease function to release this reference.
    @result A result code. See "Security Error Codes" (SecBase.h).
}    
function SecTrustCopyPolicies( trust: SecTrustRef; var policies: CFArrayRef ): OSStatus; external name '_SecTrustCopyPolicies';

{!
    @function SecTrustCopyCustomAnchorCertificates
    @abstract Returns an array of custom anchor certificates used by a given trust, as set by a prior call to SecTrustSetAnchorCertificates, or NULL if no custom anchors have been specified.
    @param trust  A reference to a trust.
    @param anchors On return, an array of custom anchor certificates (roots) used by this trust, or NULL if no custom anchors have been specified. Call the CFRelease function to release this reference.
    @result A result code. See "Security Error Codes" (SecBase.h).
	@availability Mac OS X version 10.5.
}    
function SecTrustCopyCustomAnchorCertificates( trust: SecTrustRef; var anchors: CFArrayRef ): OSStatus; external name '_SecTrustCopyCustomAnchorCertificates';

{!
    @function SecTrustCopyAnchorCertificates
    @abstract Returns an array of default anchor (root) certificates used by the system.
    @param anchors On return, an array containing the system's default anchors (roots). Call the CFRelease function to release this pointer.
    @result A result code. See "Security Error Codes" (SecBase.h).
}
function SecTrustCopyAnchorCertificates( var anchors: CFArrayRef ): OSStatus; external name '_SecTrustCopyAnchorCertificates';

{$ifc TARGET_OS_MAC}
{!
	@function SecTrustGetCSSMAnchorCertificates
	@abstract Retrieves the CSSM anchor certificates.
	@param cssmAnchors A pointer to an array of anchor certificates.
	@param cssmAnchorCount A pointer to the number of certificates in anchors.
	@result A result code. See "Security Error Codes" (SecBase.h).
	@availability Mac OS X version 10.4. Deprecated in Mac OS X version 10.5.
}
function SecTrustGetCSSMAnchorCertificates( {const} cssmAnchors: CSSM_DATAArrayPtr; var cssmAnchorCount: UInt32 ): OSStatus; external name '_SecTrustGetCSSMAnchorCertificates';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER *)

{!
	@function SecTrustGetUserTrust
	@abstract Gets the user-specified trust settings of a certificate and policy.
	@param certificate A reference to a certificate.
	@param policy A reference to a policy.
	@param trustSetting On return, a pointer to the user specified trust settings.
	@result A result code. See "Security Error Codes" (SecBase.h).
	@availability Mac OS X version 10.4. Deprecated in Mac OS X version 10.5.
}
function SecTrustGetUserTrust( certificate: SecCertificateRef; policy: SecPolicyRef; var trustSetting: SecTrustUserSetting ): OSStatus; external name '_SecTrustGetUserTrust';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER *)

{!
	@function SecTrustSetUserTrust
	@abstract Sets the user-specified trust settings of a certificate and policy.
	@param certificate A reference to a certificate.
	@param policy A reference to a policy.
	@param trustSetting The user-specified trust settings.
	@result A result code. See "Security Error Codes" (SecBase.h).
	@availability Mac OS X version 10.4. Deprecated in Mac OS X version 10.5.
	@discussion as of Mac OS version 10.5, this will result in a call to 
	 SecTrustSettingsSetTrustSettings(). 
}
function SecTrustSetUserTrust( certificate: SecCertificateRef; policy: SecPolicyRef; trustSetting: SecTrustUserSetting ): OSStatus; external name '_SecTrustSetUserTrust';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER *)
{$endc} {TARGET_OS_MAC}
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
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
