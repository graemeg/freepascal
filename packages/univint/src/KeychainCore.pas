{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       OSServices/KeychainCore.h
=======
     File:       SecurityCore/KeychainCore.h
>>>>>>> graemeg/cpstrnew
=======
     File:       SecurityCore/KeychainCore.h
>>>>>>> graemeg/cpstrnew
=======
     File:       SecurityCore/KeychainCore.h
>>>>>>> graemeg/cpstrnew
 
     Contains:   *** DEPRECATED *** Keychain low-level Interfaces
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  (c) 2000-2011 Apple Inc. All rights reserved.
=======
     Version:    SecurityCore-36064~226
 
     Copyright:  � 2000-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    SecurityCore-36064~226
 
     Copyright:  � 2000-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    SecurityCore-36064~226
 
     Copyright:  � 2000-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
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

unit KeychainCore;
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
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
uses MacTypes,Files,Aliases,CodeFragments,MacErrors,Processes,Events,SecBase;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

(*
  Also defined in SecBase

{ Data structures and types }
type
	SecKeychainRef = ^OpaqueSecKeychainRef; { an opaque type }
	OpaqueSecKeychainRef = record end;
	SecKeychainRefPtr = ^SecKeychainRef;  { when a var xx:SecKeychainRef parameter can be nil, it is changed to xx: SecKeychainRefPtr }
	SecKeychainItemRef = ^OpaqueSecKeychainItemRef; { an opaque type }
	OpaqueSecKeychainItemRef = record end;
	SecKeychainItemRefPtr = ^SecKeychainItemRef;  { when a var xx:SecKeychainItemRef parameter can be nil, it is changed to xx: SecKeychainItemRefPtr }
	SecKeychainSearchRef = ^OpaqueSecKeychainSearchRef; { an opaque type }
	OpaqueSecKeychainSearchRef = record end;
	SecKeychainSearchRefPtr = ^SecKeychainSearchRef;  { when a var xx:SecKeychainSearchRef parameter can be nil, it is changed to xx: SecKeychainSearchRefPtr }
	SecKeychainAttrType = OSType;
	SecKeychainStatus = UInt32;
	SecKeychainAttribute = record
		tag: SecKeychainAttrType;                   { 4-byte attribute tag }
		length: UInt32;                 { Length of attribute data }
		data: UnivPtr;                   { Pointer to attribute data }
	end;
	SecKeychainAttributePtr = ^SecKeychainAttribute;
type
	SecKeychainAttributeListPtr = ^SecKeychainAttributeList;
	SecKeychainAttributeList = record
		count: UInt32;                  { How many attributes in the array }
		attr: SecKeychainAttributePtr;                { Pointer to first attribute in array }
	end;
*)

type
	KCRef = SecKeychainRef;
	KCItemRef = SecKeychainItemRef;
	KCSearchRef = SecKeychainSearchRef;
	KCRefPtr = ^KCRef;
	KCItemRefPtr = ^KCItemRef;
	KCSearchRefPtr = ^KCSearchRef;
	KCAttribute = SecKeychainAttribute;
	KCAttributePtr = ^KCAttribute;
	KCAttributeList = SecKeychainAttributeList;
	KCAttributeListPtr = ^KCAttributeList;
	KCAttrType = SecKeychainAttrType;
	KCStatus = SecKeychainStatus;
	KCEvent = UInt16;
const
	kIdleKCEvent = 0;    { null event }
	kLockKCEvent = 1;    { a keychain was locked }
	kUnlockKCEvent = 2;    { a keychain was unlocked }
	kAddKCEvent = 3;    { an item was added to a keychain }
	kDeleteKCEvent = 4;    { an item was deleted from a keychain }
	kUpdateKCEvent = 5;    { an item was updated }
	kPasswordChangedKCEvent = 6;    { the keychain identity was changed }
	kSystemKCEvent = 8;    { the keychain client can process events }
	kDefaultChangedKCEvent = 9;    { the default keychain was changed }
	kDataAccessKCEvent = 10;   { a process has accessed a keychain item's data }
	kKeychainListChangedKCEvent = 11;    { the list of keychains has changed }

type
	KCEventMask = UInt16;
const
	kIdleKCEventMask = 1 shl kIdleKCEvent;
	kLockKCEventMask = 1 shl kLockKCEvent;
	kUnlockKCEventMask = 1 shl kUnlockKCEvent;
	kAddKCEventMask = 1 shl kAddKCEvent;
	kDeleteKCEventMask = 1 shl kDeleteKCEvent;
	kUpdateKCEventMask = 1 shl kUpdateKCEvent;
	kPasswordChangedKCEventMask = 1 shl kPasswordChangedKCEvent;
	kSystemEventKCEventMask = 1 shl kSystemKCEvent;
	kDefaultChangedKCEventMask = 1 shl kDefaultChangedKCEvent;
	kDataAccessKCEventMask = 1 shl kDataAccessKCEvent;
	kEveryKCEventMask = $FFFF; { all of the above}

type
	AFPServerSignature = packed array [0..15] of UInt8;
	AFPServerSignaturePtr = ^AFPServerSignature; { when a VAR xx: AFPServerSignature parameter can be nil, it is changed to xx: AFPServerSignaturePtr }
	KCPublicKeyHash = packed array [0..19] of UInt8;
type
	KCCallbackInfoPtr = ^KCCallbackInfo;
	KCCallbackInfo = record
		version: UInt32;
		item: KCItemRef;
		processID: array [0..1] of SInt32;           { unavailable on Mac OS X}
		event: array [0..4] of SInt32;               { unavailable on Mac OS X}
		keychain: KCRef;
	end;
const
	kUnlockStateKCStatus = 1;
	kRdPermKCStatus = 2;
	kWrPermKCStatus = 4;


const
	kCertificateKCItemClass = FourCharCode('cert'); { Certificate }
	kAppleSharePasswordKCItemClass = FourCharCode('ashp'); { Appleshare password }
	kInternetPasswordKCItemClass = FourCharCode('inet'); { Internet password }
	kGenericPasswordKCItemClass = FourCharCode('genp'); { Generic password }


type
	KCItemClass = FourCharCode;
const
{ Common attributes }
	kClassKCItemAttr = FourCharCode('clas'); { Item class (KCItemClass) }
	kCreationDateKCItemAttr = FourCharCode('cdat'); { Date the item was created (UInt32) }
	kModDateKCItemAttr = FourCharCode('mdat'); { Last time the item was updated (UInt32) }
	kDescriptionKCItemAttr = FourCharCode('desc'); { User-visible description string (string) }
	kCommentKCItemAttr = FourCharCode('icmt'); { User's comment about the item (string) }
	kCreatorKCItemAttr = FourCharCode('crtr'); { Item's creator (OSType) }
	kTypeKCItemAttr = FourCharCode('type'); { Item's type (OSType) }
	kScriptCodeKCItemAttr = FourCharCode('scrp'); { Script code for all strings (ScriptCode) }
	kLabelKCItemAttr = FourCharCode('labl'); { Item label (string) }
	kInvisibleKCItemAttr = FourCharCode('invi'); { Invisible (boolean) }
	kNegativeKCItemAttr = FourCharCode('nega'); { Negative (boolean) }
	kCustomIconKCItemAttr = FourCharCode('cusi'); { Custom icon (boolean) }
	kAccountKCItemAttr = FourCharCode('acct'); { User account (string) }
                                        { Unique Generic password attributes }
	kServiceKCItemAttr = FourCharCode('svce'); { Service (string) }
	kGenericKCItemAttr = FourCharCode('gena'); { User-defined attribute (untyped bytes) }
                                        { Unique Internet password attributes }
	kSecurityDomainKCItemAttr = FourCharCode('sdmn'); { Security domain (string) }
	kServerKCItemAttr = FourCharCode('srvr'); { Server's domain name or IP address (string) }
	kAuthTypeKCItemAttr = FourCharCode('atyp'); { Authentication Type (KCAuthType) }
	kPortKCItemAttr = FourCharCode('port'); { Port (UInt16) }
	kPathKCItemAttr = FourCharCode('path'); { Path (string) }
                                        { Unique Appleshare password attributes }
	kVolumeKCItemAttr = FourCharCode('vlme'); { Volume (string) }
	kAddressKCItemAttr = FourCharCode('addr'); { Server address (IP or domain name) or zone name (string) }
	kSignatureKCItemAttr = FourCharCode('ssig'); { Server signature block (AFPServerSignature) }
                                        { Unique AppleShare and Internet attributes }
	kProtocolKCItemAttr = FourCharCode('ptcl'); { Protocol (KCProtocolType) }
                                        { Certificate attributes }
	kSubjectKCItemAttr = FourCharCode('subj'); { Subject distinguished name (DER-encoded data) }
	kCommonNameKCItemAttr = FourCharCode('cn  '); { Common Name (UTF8-encoded string) }
	kIssuerKCItemAttr = FourCharCode('issu'); { Issuer distinguished name (DER-encoded data) }
	kSerialNumberKCItemAttr = FourCharCode('snbr'); { Certificate serial number (DER-encoded data) }
	kEMailKCItemAttr = FourCharCode('mail'); { E-mail address (ASCII-encoded string) }
	kPublicKeyHashKCItemAttr = FourCharCode('hpky'); { Hash of public key (KCPublicKeyHash), 20 bytes max. }
	kIssuerURLKCItemAttr = FourCharCode('iurl'); { URL of the certificate issuer (ASCII-encoded string) }
                                        { Shared by keys and certificates }
	kEncryptKCItemAttr = FourCharCode('encr'); { Encrypt (Boolean) }
	kDecryptKCItemAttr = FourCharCode('decr'); { Decrypt (Boolean) }
	kSignKCItemAttr = FourCharCode('sign'); { Sign (Boolean) }
	kVerifyKCItemAttr = FourCharCode('veri'); { Verify (Boolean) }
	kWrapKCItemAttr = FourCharCode('wrap'); { Wrap (Boolean) }
	kUnwrapKCItemAttr = FourCharCode('unwr'); { Unwrap (Boolean) }
	kStartDateKCItemAttr = FourCharCode('sdat'); { Start Date (UInt32) }
	kEndDateKCItemAttr = FourCharCode('edat'); { End Date (UInt32) }
<<<<<<< HEAD

type
	KCItemAttr = FourCharCode;
const
	kKCAuthTypeNTLM = FourCharCode('ntlm');
	kKCAuthTypeMSN = FourCharCode('msna');
	kKCAuthTypeDPA = FourCharCode('dpaa');
	kKCAuthTypeRPA = FourCharCode('rpaa');
	kKCAuthTypeHTTPDigest = FourCharCode('httd');
	kKCAuthTypeDefault = FourCharCode('dflt');

type
	KCAuthType = FourCharCode;
const
	kKCProtocolTypeFTP = FourCharCode('ftp ');
	kKCProtocolTypeFTPAccount = FourCharCode('ftpa');
	kKCProtocolTypeHTTP = FourCharCode('http');
	kKCProtocolTypeIRC = FourCharCode('irc ');
	kKCProtocolTypeNNTP = FourCharCode('nntp');
	kKCProtocolTypePOP3 = FourCharCode('pop3');
	kKCProtocolTypeSMTP = FourCharCode('smtp');
	kKCProtocolTypeSOCKS = FourCharCode('sox ');
	kKCProtocolTypeIMAP = FourCharCode('imap');
	kKCProtocolTypeLDAP = FourCharCode('ldap');
	kKCProtocolTypeAppleTalk = FourCharCode('atlk');
	kKCProtocolTypeAFP = FourCharCode('afp ');
	kKCProtocolTypeTelnet = FourCharCode('teln');

type
	KCProtocolType = FourCharCode;
	KCCertAddOptions = UInt32;
const
	kSecOptionReserved = $000000FF; { First byte reserved for SecOptions flags }
	kCertUsageShift = 8;    { start at bit 8 }
	kCertUsageSigningAdd = 1 shl (kCertUsageShift + 0);
	kCertUsageSigningAskAndAdd = 1 shl (kCertUsageShift + 1);
	kCertUsageVerifyAdd = 1 shl (kCertUsageShift + 2);
	kCertUsageVerifyAskAndAdd = 1 shl (kCertUsageShift + 3);
	kCertUsageEncryptAdd = 1 shl (kCertUsageShift + 4);
	kCertUsageEncryptAskAndAdd = 1 shl (kCertUsageShift + 5);
	kCertUsageDecryptAdd = 1 shl (kCertUsageShift + 6);
	kCertUsageDecryptAskAndAdd = 1 shl (kCertUsageShift + 7);
	kCertUsageKeyExchAdd = 1 shl (kCertUsageShift + 8);
	kCertUsageKeyExchAskAndAdd = 1 shl (kCertUsageShift + 9);
	kCertUsageRootAdd = 1 shl (kCertUsageShift + 10);
	kCertUsageRootAskAndAdd = 1 shl (kCertUsageShift + 11);
	kCertUsageSSLAdd = 1 shl (kCertUsageShift + 12);
	kCertUsageSSLAskAndAdd = 1 shl (kCertUsageShift + 13);
	kCertUsageAllAdd = $7FFFFF00;
<<<<<<< HEAD

type
	KCVerifyStopOn = UInt16;
const
	kPolicyKCStopOn = 0;
	kNoneKCStopOn = 1;
	kFirstPassKCStopOn = 2;
	kFirstFailKCStopOn = 3;

type
	KCCertSearchOptions = UInt32;
<<<<<<< HEAD
const
	kCertSearchShift = 0;    { start at bit 0 }
	kCertSearchSigningIgnored = 0;
	kCertSearchSigningAllowed = 1 shl (kCertSearchShift + 0);
	kCertSearchSigningDisallowed = 1 shl (kCertSearchShift + 1);
	kCertSearchSigningMask = ((kCertSearchSigningAllowed) or (kCertSearchSigningDisallowed));
	kCertSearchVerifyIgnored = 0;
	kCertSearchVerifyAllowed = 1 shl (kCertSearchShift + 2);
	kCertSearchVerifyDisallowed = 1 shl (kCertSearchShift + 3);
	kCertSearchVerifyMask = ((kCertSearchVerifyAllowed) or (kCertSearchVerifyDisallowed));
	kCertSearchEncryptIgnored = 0;
	kCertSearchEncryptAllowed = 1 shl (kCertSearchShift + 4);
	kCertSearchEncryptDisallowed = 1 shl (kCertSearchShift + 5);
	kCertSearchEncryptMask = ((kCertSearchEncryptAllowed) or (kCertSearchEncryptDisallowed));
	kCertSearchDecryptIgnored = 0;
	kCertSearchDecryptAllowed = 1 shl (kCertSearchShift + 6);
	kCertSearchDecryptDisallowed = 1 shl (kCertSearchShift + 7);
	kCertSearchDecryptMask = ((kCertSearchDecryptAllowed) or (kCertSearchDecryptDisallowed));
	kCertSearchWrapIgnored = 0;
	kCertSearchWrapAllowed = 1 shl (kCertSearchShift + 8);
	kCertSearchWrapDisallowed = 1 shl (kCertSearchShift + 9);
	kCertSearchWrapMask = ((kCertSearchWrapAllowed) or (kCertSearchWrapDisallowed));
	kCertSearchUnwrapIgnored = 0;
	kCertSearchUnwrapAllowed = 1 shl (kCertSearchShift + 10);
	kCertSearchUnwrapDisallowed = 1 shl (kCertSearchShift + 11);
	kCertSearchUnwrapMask = ((kCertSearchUnwrapAllowed) or (kCertSearchUnwrapDisallowed));
	kCertSearchPrivKeyRequired = 1 shl (kCertSearchShift + 12);
	kCertSearchAny = 0;

{ Other constants }
const
	kAnyPort = 0;

const
=======
const
	kCertSearchShift = 0;    { start at bit 0 }
	kCertSearchSigningIgnored = 0;
	kCertSearchSigningAllowed = 1 shl (kCertSearchShift + 0);
	kCertSearchSigningDisallowed = 1 shl (kCertSearchShift + 1);
	kCertSearchSigningMask = ((kCertSearchSigningAllowed) or (kCertSearchSigningDisallowed));
	kCertSearchVerifyIgnored = 0;
	kCertSearchVerifyAllowed = 1 shl (kCertSearchShift + 2);
	kCertSearchVerifyDisallowed = 1 shl (kCertSearchShift + 3);
	kCertSearchVerifyMask = ((kCertSearchVerifyAllowed) or (kCertSearchVerifyDisallowed));
	kCertSearchEncryptIgnored = 0;
	kCertSearchEncryptAllowed = 1 shl (kCertSearchShift + 4);
	kCertSearchEncryptDisallowed = 1 shl (kCertSearchShift + 5);
	kCertSearchEncryptMask = ((kCertSearchEncryptAllowed) or (kCertSearchEncryptDisallowed));
	kCertSearchDecryptIgnored = 0;
	kCertSearchDecryptAllowed = 1 shl (kCertSearchShift + 6);
	kCertSearchDecryptDisallowed = 1 shl (kCertSearchShift + 7);
	kCertSearchDecryptMask = ((kCertSearchDecryptAllowed) or (kCertSearchDecryptDisallowed));
	kCertSearchWrapIgnored = 0;
	kCertSearchWrapAllowed = 1 shl (kCertSearchShift + 8);
	kCertSearchWrapDisallowed = 1 shl (kCertSearchShift + 9);
	kCertSearchWrapMask = ((kCertSearchWrapAllowed) or (kCertSearchWrapDisallowed));
	kCertSearchUnwrapIgnored = 0;
	kCertSearchUnwrapAllowed = 1 shl (kCertSearchShift + 10);
	kCertSearchUnwrapDisallowed = 1 shl (kCertSearchShift + 11);
	kCertSearchUnwrapMask = ((kCertSearchUnwrapAllowed) or (kCertSearchUnwrapDisallowed));
	kCertSearchPrivKeyRequired = 1 shl (kCertSearchShift + 12);
	kCertSearchAny = 0;

{ Other constants }
const
	kAnyPort = 0;

const
>>>>>>> graemeg/cpstrnew
=======

type
	KCVerifyStopOn = UInt16;
const
	kPolicyKCStopOn = 0;
	kNoneKCStopOn = 1;
	kFirstPassKCStopOn = 2;
	kFirstFailKCStopOn = 3;

type
	KCCertSearchOptions = UInt32;
const
	kCertSearchShift = 0;    { start at bit 0 }
	kCertSearchSigningIgnored = 0;
	kCertSearchSigningAllowed = 1 shl (kCertSearchShift + 0);
	kCertSearchSigningDisallowed = 1 shl (kCertSearchShift + 1);
	kCertSearchSigningMask = ((kCertSearchSigningAllowed) or (kCertSearchSigningDisallowed));
	kCertSearchVerifyIgnored = 0;
	kCertSearchVerifyAllowed = 1 shl (kCertSearchShift + 2);
	kCertSearchVerifyDisallowed = 1 shl (kCertSearchShift + 3);
	kCertSearchVerifyMask = ((kCertSearchVerifyAllowed) or (kCertSearchVerifyDisallowed));
	kCertSearchEncryptIgnored = 0;
	kCertSearchEncryptAllowed = 1 shl (kCertSearchShift + 4);
	kCertSearchEncryptDisallowed = 1 shl (kCertSearchShift + 5);
	kCertSearchEncryptMask = ((kCertSearchEncryptAllowed) or (kCertSearchEncryptDisallowed));
	kCertSearchDecryptIgnored = 0;
	kCertSearchDecryptAllowed = 1 shl (kCertSearchShift + 6);
	kCertSearchDecryptDisallowed = 1 shl (kCertSearchShift + 7);
	kCertSearchDecryptMask = ((kCertSearchDecryptAllowed) or (kCertSearchDecryptDisallowed));
	kCertSearchWrapIgnored = 0;
	kCertSearchWrapAllowed = 1 shl (kCertSearchShift + 8);
	kCertSearchWrapDisallowed = 1 shl (kCertSearchShift + 9);
	kCertSearchWrapMask = ((kCertSearchWrapAllowed) or (kCertSearchWrapDisallowed));
	kCertSearchUnwrapIgnored = 0;
	kCertSearchUnwrapAllowed = 1 shl (kCertSearchShift + 10);
	kCertSearchUnwrapDisallowed = 1 shl (kCertSearchShift + 11);
	kCertSearchUnwrapMask = ((kCertSearchUnwrapAllowed) or (kCertSearchUnwrapDisallowed));
	kCertSearchPrivKeyRequired = 1 shl (kCertSearchShift + 12);
	kCertSearchAny = 0;

{ Other constants }
const
	kAnyPort = 0;

const
>>>>>>> graemeg/cpstrnew
=======

type
	KCItemAttr = FourCharCode;
const
	kKCAuthTypeNTLM = FourCharCode('ntlm');
	kKCAuthTypeMSN = FourCharCode('msna');
	kKCAuthTypeDPA = FourCharCode('dpaa');
	kKCAuthTypeRPA = FourCharCode('rpaa');
	kKCAuthTypeHTTPDigest = FourCharCode('httd');
	kKCAuthTypeDefault = FourCharCode('dflt');

type
	KCAuthType = FourCharCode;
const
	kKCProtocolTypeFTP = FourCharCode('ftp ');
	kKCProtocolTypeFTPAccount = FourCharCode('ftpa');
	kKCProtocolTypeHTTP = FourCharCode('http');
	kKCProtocolTypeIRC = FourCharCode('irc ');
	kKCProtocolTypeNNTP = FourCharCode('nntp');
	kKCProtocolTypePOP3 = FourCharCode('pop3');
	kKCProtocolTypeSMTP = FourCharCode('smtp');
	kKCProtocolTypeSOCKS = FourCharCode('sox ');
	kKCProtocolTypeIMAP = FourCharCode('imap');
	kKCProtocolTypeLDAP = FourCharCode('ldap');
	kKCProtocolTypeAppleTalk = FourCharCode('atlk');
	kKCProtocolTypeAFP = FourCharCode('afp ');
	kKCProtocolTypeTelnet = FourCharCode('teln');

type
	KCProtocolType = FourCharCode;
	KCCertAddOptions = UInt32;
const
	kSecOptionReserved = $000000FF; { First byte reserved for SecOptions flags }
	kCertUsageShift = 8;    { start at bit 8 }
	kCertUsageSigningAdd = 1 shl (kCertUsageShift + 0);
	kCertUsageSigningAskAndAdd = 1 shl (kCertUsageShift + 1);
	kCertUsageVerifyAdd = 1 shl (kCertUsageShift + 2);
	kCertUsageVerifyAskAndAdd = 1 shl (kCertUsageShift + 3);
	kCertUsageEncryptAdd = 1 shl (kCertUsageShift + 4);
	kCertUsageEncryptAskAndAdd = 1 shl (kCertUsageShift + 5);
	kCertUsageDecryptAdd = 1 shl (kCertUsageShift + 6);
	kCertUsageDecryptAskAndAdd = 1 shl (kCertUsageShift + 7);
	kCertUsageKeyExchAdd = 1 shl (kCertUsageShift + 8);
	kCertUsageKeyExchAskAndAdd = 1 shl (kCertUsageShift + 9);
	kCertUsageRootAdd = 1 shl (kCertUsageShift + 10);
	kCertUsageRootAskAndAdd = 1 shl (kCertUsageShift + 11);
	kCertUsageSSLAdd = 1 shl (kCertUsageShift + 12);
	kCertUsageSSLAskAndAdd = 1 shl (kCertUsageShift + 13);
	kCertUsageAllAdd = $7FFFFF00;

type
	KCVerifyStopOn = UInt16;
const
	kPolicyKCStopOn = 0;
	kNoneKCStopOn = 1;
	kFirstPassKCStopOn = 2;
	kFirstFailKCStopOn = 3;

type
	KCCertSearchOptions = UInt32;
const
	kCertSearchShift = 0;    { start at bit 0 }
	kCertSearchSigningIgnored = 0;
	kCertSearchSigningAllowed = 1 shl (kCertSearchShift + 0);
	kCertSearchSigningDisallowed = 1 shl (kCertSearchShift + 1);
	kCertSearchSigningMask = ((kCertSearchSigningAllowed) or (kCertSearchSigningDisallowed));
	kCertSearchVerifyIgnored = 0;
	kCertSearchVerifyAllowed = 1 shl (kCertSearchShift + 2);
	kCertSearchVerifyDisallowed = 1 shl (kCertSearchShift + 3);
	kCertSearchVerifyMask = ((kCertSearchVerifyAllowed) or (kCertSearchVerifyDisallowed));
	kCertSearchEncryptIgnored = 0;
	kCertSearchEncryptAllowed = 1 shl (kCertSearchShift + 4);
	kCertSearchEncryptDisallowed = 1 shl (kCertSearchShift + 5);
	kCertSearchEncryptMask = ((kCertSearchEncryptAllowed) or (kCertSearchEncryptDisallowed));
	kCertSearchDecryptIgnored = 0;
	kCertSearchDecryptAllowed = 1 shl (kCertSearchShift + 6);
	kCertSearchDecryptDisallowed = 1 shl (kCertSearchShift + 7);
	kCertSearchDecryptMask = ((kCertSearchDecryptAllowed) or (kCertSearchDecryptDisallowed));
	kCertSearchWrapIgnored = 0;
	kCertSearchWrapAllowed = 1 shl (kCertSearchShift + 8);
	kCertSearchWrapDisallowed = 1 shl (kCertSearchShift + 9);
	kCertSearchWrapMask = ((kCertSearchWrapAllowed) or (kCertSearchWrapDisallowed));
	kCertSearchUnwrapIgnored = 0;
	kCertSearchUnwrapAllowed = 1 shl (kCertSearchShift + 10);
	kCertSearchUnwrapDisallowed = 1 shl (kCertSearchShift + 11);
	kCertSearchUnwrapMask = ((kCertSearchUnwrapAllowed) or (kCertSearchUnwrapDisallowed));
	kCertSearchPrivKeyRequired = 1 shl (kCertSearchShift + 12);
	kCertSearchAny = 0;

{ Other constants }
const
	kAnyPort = 0;

const
>>>>>>> graemeg/cpstrnew
	kAnyProtocol = 0;
	kAnyAuthType = 0;

{ Opening and getting information about the Keychain Manager }
{
 *  KCGetKeychainManagerVersion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainGetVersion
 *  
 *  Availability:
 *    Mac OS X:         not available but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCGetKeychainManagerVersion( var returnVers: UInt32 ): OSStatus; external name '_KCGetKeychainManagerVersion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew


{$ifc TARGET_RT_MAC_CFM}
{
        KeychainManagerAvailable() is a macro/inline available only in C/C++.  
        To get the same functionality from pascal or assembly, you need
        to test if KCGetKeychainManagerVersion function is not NULL.  For instance:
        
            gKeychainManagerAvailable = FALSE;
            IF @KCGetKeychainManagerVersion <> kUnresolvedCFragSymbolAddress THEN
                gKeychainManagerAvailable = TRUE;
            end
    
    }
{$elsec}
  {$ifc TARGET_RT_MAC_MACHO}
    { Keychain is always available on OS X }
//    #define KeychainManagerAvailable()  (true)
  {$endc}
{$endc}  {  }

{ Managing the Human Interface }
{
 *  KCSetInteractionAllowed()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainSetUserInteractionAllowed
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCSetInteractionAllowed( state: Boolean ): OSStatus; external name '_KCSetInteractionAllowed';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCIsInteractionAllowed()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainGetUserInteractionAllowed
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCIsInteractionAllowed: Boolean; external name '_KCIsInteractionAllowed';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{ Creating references to keychains }
{$ifc not TARGET_CPU_64}
{
 *  KCMakeKCRefFromFSSpec()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainOpen
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCMakeKCRefFromFSSpec( var keychainFSSpec: FSSpec; var keychain: KCRef ): OSStatus; external name '_KCMakeKCRefFromFSSpec';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_5,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{$endc} {not TARGET_CPU_64}

{
 *  KCMakeKCRefFromFSRef()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainOpen
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCMakeKCRefFromFSRef( var keychainFSRef: FSRef; var keychain: KCRef ): OSStatus; external name '_KCMakeKCRefFromFSRef';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCMakeKCRefFromAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainOpen
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCMakeKCRefFromAlias( keychainAlias: AliasHandle; var keychain: KCRef ): OSStatus; external name '_KCMakeKCRefFromAlias';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCMakeAliasFromKCRef()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainOpen
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCMakeAliasFromKCRef( keychain: KCRef; var keychainAlias: AliasHandle ): OSStatus; external name '_KCMakeAliasFromKCRef';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCReleaseKeychain()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CFRelease when releasing SecKeychainRef objects
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCReleaseKeychain( var keychain: KCRef ): OSStatus; external name '_KCReleaseKeychain';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{ Specifying the default keychain }
{
 *  KCGetDefaultKeychain()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainCopyDefault
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCGetDefaultKeychain( var keychain: KCRef ): OSStatus; external name '_KCGetDefaultKeychain';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCSetDefaultKeychain()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainSetDefault
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCSetDefaultKeychain( keychain: KCRef ): OSStatus; external name '_KCSetDefaultKeychain';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{ Getting information about a keychain }
{
 *  KCGetStatus()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainGetStatus
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCGetStatus( keychain: KCRef { can be NULL }; var keychainStatus: UInt32 ): OSStatus; external name '_KCGetStatus';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCGetKeychain()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemCopyKeychain
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCGetKeychain( item: KCItemRef; var keychain: KCRef ): OSStatus; external name '_KCGetKeychain';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCGetKeychainName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainGetPath
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCGetKeychainName( keychain: KCRef; keychainName: StringPtr ): OSStatus; external name '_KCGetKeychainName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{ Enumerating available keychains }
{
 *  KCCountKeychains()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainCopySearchList
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCCountKeychains: UInt16; external name '_KCCountKeychains';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCGetIndKeychain()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainCopySearchList
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCGetIndKeychain( index: UInt16; var keychain: KCRef ): OSStatus; external name '_KCGetIndKeychain';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


type
	KCCallbackProcPtr = function( keychainEvent: KCEvent; var info: KCCallbackInfo; userContext: UnivPtr ): OSStatus;
	KCCallbackUPP = KCCallbackProcPtr;
<<<<<<< HEAD
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  NewKCCallbackUPP()   *** DEPRECATED ***
=======
 *  NewKCCallbackUPP()
>>>>>>> graemeg/cpstrnew
=======
 *  NewKCCallbackUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewKCCallbackUPP( userRoutine: KCCallbackProcPtr ): KCCallbackUPP; external name '_NewKCCallbackUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  DisposeKCCallbackUPP()   *** DEPRECATED ***
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{
 *  DisposeKCCallbackUPP()
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

=======
>>>>>>> graemeg/cpstrnew
{
 *  NewKCCallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
<<<<<<< HEAD
 }
procedure DisposeKCCallbackUPP( userUPP: KCCallbackUPP ); external name '_DisposeKCCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{
 *  InvokeKCCallbackUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeKCCallbackUPP( userUPP: KCCallbackUPP ); external name '_DisposeKCCallbackUPP';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

{
 *  InvokeKCCallbackUPP()   *** DEPRECATED ***
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{
 *  InvokeKCCallbackUPP()
>>>>>>> graemeg/cpstrnew
=======
 }
function NewKCCallbackUPP( userRoutine: KCCallbackProcPtr ): KCCallbackUPP; external name '_NewKCCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{
 *  DisposeKCCallbackUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeKCCallbackUPP( keychainEvent: KCEvent; var info: KCCallbackInfo; userContext: UnivPtr; userUPP: KCCallbackUPP ): OSStatus; external name '_InvokeKCCallbackUPP';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew

{ High-level interface for retrieving passwords }
{
 *  KCFindAppleSharePassword()   *** DEPRECATED ***
=======
function InvokeKCCallbackUPP( keychainEvent: KCEvent; var info: KCCallbackInfo; userContext: UnivPtr; userUPP: KCCallbackUPP ): OSStatus; external name '_InvokeKCCallbackUPP';
=======
procedure DisposeKCCallbackUPP( userUPP: KCCallbackUPP ); external name '_DisposeKCCallbackUPP';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{ High-level interface for retrieving passwords }
{
<<<<<<< HEAD
 *  KCFindAppleSharePassword()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainFindInternetPassword
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCFindAppleSharePassword( serverSignature: AFPServerSignaturePtr { can be NULL }; serverAddress: ConstStringPtr { can be NULL }; serverName: ConstStringPtr { can be NULL }; volumeName: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindAppleSharePassword';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  KCFindInternetPassword()   *** DEPRECATED ***
=======
 *  InvokeKCCallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeKCCallbackUPP( keychainEvent: KCEvent; var info: KCCallbackInfo; userContext: UnivPtr; userUPP: KCCallbackUPP ): OSStatus; external name '_InvokeKCCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{ High-level interface for retrieving passwords }
{
 *  KCFindAppleSharePassword()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    Use SecKeychainFindInternetPassword
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
<<<<<<< HEAD
function KCFindInternetPassword( serverName: ConstStringPtr { can be NULL }; securityDomain: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; port: UInt16; protocol: OSType; authType: OSType; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindInternetPassword';
=======
function KCFindAppleSharePassword( serverSignature: AFPServerSignaturePtr { can be NULL }; serverAddress: ConstStringPtr { can be NULL }; serverName: ConstStringPtr { can be NULL }; volumeName: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindAppleSharePassword';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  KCFindInternetPasswordWithPath()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
 *  KCFindInternetPassword()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    Use SecKeychainFindInternetPassword
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCFindAppleSharePassword( serverSignature: AFPServerSignaturePtr { can be NULL }; serverAddress: ConstStringPtr { can be NULL }; serverName: ConstStringPtr { can be NULL }; volumeName: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindAppleSharePassword';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCFindInternetPassword()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainFindInternetPassword
=======
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCFindInternetPasswordWithPath( serverName: ConstStringPtr { can be NULL }; securityDomain: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; path: ConstStringPtr { can be NULL }; port: UInt16; protocol: OSType; authType: OSType; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindInternetPasswordWithPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  KCFindGenericPassword()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainFindGenericPassword
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCFindInternetPassword( serverName: ConstStringPtr { can be NULL }; securityDomain: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; port: UInt16; protocol: OSType; authType: OSType; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindInternetPassword';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  KCFindInternetPasswordWithPath()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainFindInternetPassword
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
<<<<<<< HEAD
function KCFindInternetPassword( serverName: ConstStringPtr { can be NULL }; securityDomain: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; port: UInt16; protocol: OSType; authType: OSType; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindInternetPassword';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCFindInternetPasswordWithPath()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainFindInternetPassword
=======
function KCFindGenericPassword( serviceName: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindGenericPassword';
=======
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCFindInternetPasswordWithPath( serverName: ConstStringPtr { can be NULL }; securityDomain: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; path: ConstStringPtr { can be NULL }; port: UInt16; protocol: OSType; authType: OSType; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindInternetPasswordWithPath';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{ Keychain Manager callbacks }
{
<<<<<<< HEAD
 *  KCAddCallback()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainAddCallback
>>>>>>> graemeg/cpstrnew
=======
 *  KCFindGenericPassword()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainFindGenericPassword
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCFindInternetPasswordWithPath( serverName: ConstStringPtr { can be NULL }; securityDomain: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; path: ConstStringPtr { can be NULL }; port: UInt16; protocol: OSType; authType: OSType; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindInternetPasswordWithPath';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCFindGenericPassword()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainFindGenericPassword
=======
=======
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCFindGenericPassword( serviceName: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindGenericPassword';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{ Keychain Manager callbacks }
{
 *  KCAddCallback()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainAddCallback
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> graemeg/cpstrnew
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCAddCallback( callbackProc: KCCallbackUPP; eventMask: KCEventMask; userContext: UnivPtr ): OSStatus; external name '_KCAddCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  KCRemoveCallback()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainRemoveCallback
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function KCFindGenericPassword( serviceName: ConstStringPtr { can be NULL }; accountName: ConstStringPtr { can be NULL }; maxLength: UInt32; passwordData: UnivPtr; var actualLength: UInt32; item: KCItemRefPtr { can be NULL } ): OSStatus; external name '_KCFindGenericPassword';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{ Keychain Manager callbacks }
{
 *  KCAddCallback()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainAddCallback
=======
function KCRemoveCallback( callbackProc: KCCallbackUPP ): OSStatus; external name '_KCRemoveCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


=======
function KCRemoveCallback( callbackProc: KCCallbackUPP ): OSStatus; external name '_KCRemoveCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
{ Creating and editing a keychain item }
{
 *  KCNewItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemCreateFromContent
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function KCAddCallback( callbackProc: KCCallbackUPP; eventMask: KCEventMask; userContext: UnivPtr ): OSStatus; external name '_KCAddCallback';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCRemoveCallback()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainRemoveCallback
=======
function KCNewItem( itemClass: KCItemClass; itemCreator: OSType; length: UInt32; data: {const} UnivPtr; var item: KCItemRef ): OSStatus; external name '_KCNewItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
=======
function KCNewItem( itemClass: KCItemClass; itemCreator: OSType; length: UInt32; data: {const} UnivPtr; var item: KCItemRef ): OSStatus; external name '_KCNewItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
>>>>>>> graemeg/cpstrnew
 *  KCSetAttribute()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemModifyAttributesAndData
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function KCRemoveCallback( callbackProc: KCCallbackUPP ): OSStatus; external name '_KCRemoveCallback';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
function KCSetAttribute( item: KCItemRef; var attr: KCAttribute ): OSStatus; external name '_KCSetAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
function KCSetAttribute( item: KCItemRef; var attr: KCAttribute ): OSStatus; external name '_KCSetAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

>>>>>>> graemeg/cpstrnew


=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
{ Creating and editing a keychain item }
{
<<<<<<< HEAD
<<<<<<< HEAD
 *  KCNewItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemCreateFromContent
=======
=======
>>>>>>> graemeg/cpstrnew
 *  KCGetAttribute()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemCopyAttributesAndData
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function KCNewItem( itemClass: KCItemClass; itemCreator: OSType; length: UInt32; data: {const} UnivPtr; var item: KCItemRef ): OSStatus; external name '_KCNewItem';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCSetAttribute()   *** DEPRECATED ***
=======
function KCGetAttribute( item: KCItemRef; var attr: KCAttribute; var actualLength: UInt32 ): OSStatus; external name '_KCGetAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  KCSetData()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
=======
function KCGetAttribute( item: KCItemRef; var attr: KCAttribute; var actualLength: UInt32 ): OSStatus; external name '_KCGetAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  KCSetData()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    Use SecKeychainItemModifyAttributesAndData
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function KCSetAttribute( item: KCItemRef; var attr: KCAttribute ): OSStatus; external name '_KCSetAttribute';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
function KCSetData( item: KCItemRef; length: UInt32; data: {const} UnivPtr ): OSStatus; external name '_KCSetData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
function KCSetData( item: KCItemRef; length: UInt32; data: {const} UnivPtr ): OSStatus; external name '_KCSetData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  KCGetAttribute()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemCopyAttributesAndData
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCGetAttribute( item: KCItemRef; var attr: KCAttribute; var actualLength: UInt32 ): OSStatus; external name '_KCGetAttribute';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCSetData()   *** DEPRECATED ***
 *  
=======
 *  KCUpdateItem()   *** DEPRECATED ***
 *  
>>>>>>> graemeg/cpstrnew
=======
 *  KCUpdateItem()   *** DEPRECATED ***
 *  
>>>>>>> graemeg/cpstrnew
 *  Deprecated:
 *    Use SecKeychainItemModifyAttributesAndData
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 }
function KCSetData( item: KCItemRef; length: UInt32; data: {const} UnivPtr ): OSStatus; external name '_KCSetData';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{ Managing keychain items }
{
 *  KCUpdateItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemModifyAttributesAndData
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCUpdateItem( item: KCItemRef ): OSStatus; external name '_KCUpdateItem';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
 }
function KCUpdateItem( item: KCItemRef ): OSStatus; external name '_KCUpdateItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
 }
function KCUpdateItem( item: KCItemRef ): OSStatus; external name '_KCUpdateItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCReleaseItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CFRelease when releasing SecKeychainItemRef objects
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCReleaseItem( var item: KCItemRef ): OSStatus; external name '_KCReleaseItem';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCCopyItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemCreateCopy
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 2.0 and later
 }
function KCCopyItem( item: KCItemRef; destKeychain: KCRef; var copy: KCItemRef ): OSStatus; external name '_KCCopyItem';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)


=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
{ Searching and enumerating keychain items }
{
 *  KCFindFirstItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainSearchCreateFromAttributes /
 *    SecKeychainSearchCopyNext
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCFindFirstItem( keychain: KCRef { can be NULL }; {const} attrList: KCAttributeListPtr { can be NULL }; var search: KCSearchRef; var item: KCItemRef ): OSStatus; external name '_KCFindFirstItem';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCFindNextItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainSearchCopyNext
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCFindNextItem( search: KCSearchRef; var item: KCItemRef ): OSStatus; external name '_KCFindNextItem';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCReleaseSearch()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CFRelease when releasing SecKeychainSearchRef objects
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCReleaseSearch( var search: KCSearchRef ): OSStatus; external name '_KCReleaseSearch';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
{ Managing keychain items }
{
 *  KCDeleteItem()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemDelete
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCDeleteItem( item: KCItemRef ): OSStatus; external name '_KCDeleteItem';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew


{
 *  KCGetData()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainItemCopyAttributesAndData
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCGetData( item: KCItemRef; maxLength: UInt32; data: UnivPtr; var actualLength: UInt32 ): OSStatus; external name '_KCGetData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)


=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


>>>>>>> graemeg/cpstrnew
{ Locking a keychain }
{
 *  KCLock()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SecKeychainLock
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in KeychainLib 1.0 and later
 }
function KCLock( keychain: KCRef ): OSStatus; external name '_KCLock';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0,__MAC_10_6,__IPHONE_NA,__IPHONE_NA) *)

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> graemeg/cpstrnew

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
