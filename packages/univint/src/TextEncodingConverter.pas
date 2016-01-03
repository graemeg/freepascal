{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/TextEncodingConverter.h
 
     Contains:   Text Encoding Conversion Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1994-2011 Apple Inc. All rights reserved.
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1994-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1994-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1994-2008 Apple Inc. All rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1994-2008 Apple Inc. All rights reserved.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       TextEncodingConverter.p
=======
     File:       CarbonCore/TextEncodingConverter.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Text Encoding Conversion Interfaces.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1994-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1994-2008 Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
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
                     http://www.freepascal.org/bugs.html
 
}
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

unit TextEncodingConverter;
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
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
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
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,TextCommon,CFBase;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
=======
uses MacTypes,TextCommon;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,TextCommon;
>>>>>>> origin/fixes_2_2
=======
uses MacTypes,TextCommon,CFBase;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
<<<<<<< HEAD
type
	TECPluginSignature = OSType;
	TECPluginVersion = UInt32;
{ plugin signatures }
const
	kTECSignature = FourCharCode('encv');
	kTECUnicodePluginSignature = FourCharCode('puni');
	kTECJapanesePluginSignature = FourCharCode('pjpn');
	kTECChinesePluginSignature = FourCharCode('pzho');
	kTECKoreanPluginSignature = FourCharCode('pkor');
<<<<<<< HEAD


=======


>>>>>>> origin/cpstrnew
{ converter object reference }
type
	TECObjectRef = ^SInt32; { an opaque type }
	TECObjectRefPtr = ^TECObjectRef;  { when a var xx:TECObjectRef parameter can be nil, it is changed to xx: TECObjectRefPtr }
	TECSnifferObjectRef = ^SInt32; { an opaque type }
	TECSnifferObjectRefPtr = ^TECSnifferObjectRef;  { when a var xx:TECSnifferObjectRef parameter can be nil, it is changed to xx: TECSnifferObjectRefPtr }
	TECPluginSig = OSType;
	TECConversionInfoPtr = ^TECConversionInfo;
	TECConversionInfo = record
		sourceEncoding: TextEncoding;
		destinationEncoding: TextEncoding;
		reserved1: UInt16;
		reserved2: UInt16;
	end;

{
 *  TECInternetNameUsageMask
 *  
 *  Discussion:
 *    Mask values that control the mapping between TextEncoding and
 *    IANA charset name or MIB enum.
 }
type
	TECInternetNameUsageMask = UInt32;
const
{ Use one of the following}

  {
   * Use the default type of mapping given other usage information
   * (none currently defined).
   }
	kTECInternetNameDefaultUsageMask = 0;

  {
   * Use the closest possible match between TextEncoding value and IANA
   * charset name or MIB enum
   }
	kTECInternetNameStrictUsageMask = 1;

  {
   * When mapping from IANA charset name or MIB enum to TextEncoding,
   * map to the largest superset of the encoding specified by the
   * charset name or MIB enum (i.e. be tolerant). When mapping from
   * TextEncoding to IANA charset name or MIB enum, typically map to
   * the most generic or widely recognized charset name or MIB enum.
   }
	kTECInternetNameTolerantUsageMask = 2;

{ Special values for MIB enums }
const
	kTEC_MIBEnumDontCare = -1;

{ Additional control flags for TECSetBasicOptions }
const
	kTECDisableFallbacksBit = 16;
	kTECDisableLooseMappingsBit = 17;

const
	kTECDisableFallbacksMask = 1 shl kTECDisableFallbacksBit;
	kTECDisableLooseMappingsMask = 1 shl kTECDisableLooseMappingsBit;


{ return number of encodings types supported by user's configuraton of the encoding converter }
{
 *  TECCountAvailableTextEncodings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
function TECCountAvailableTextEncodings( var numberEncodings: ItemCount ): OSStatus; external name '_TECCountAvailableTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2

{$ALIGN MAC68K}

type
	TECPluginSignature = OSType;
	TECPluginVersion = UInt32;
{ plugin signatures }
const
	kTECSignature = FourCharCode('encv');
	kTECUnicodePluginSignature = FourCharCode('puni');
	kTECJapanesePluginSignature = FourCharCode('pjpn');
	kTECChinesePluginSignature = FourCharCode('pzho');
	kTECKoreanPluginSignature = FourCharCode('pkor');


{ converter object reference }
type
	TECObjectRef = ^SInt32; { an opaque type }
	TECObjectRefPtr = ^TECObjectRef;  { when a var xx:TECObjectRef parameter can be nil, it is changed to xx: TECObjectRefPtr }
	TECSnifferObjectRef = ^SInt32; { an opaque type }
	TECSnifferObjectRefPtr = ^TECSnifferObjectRef;  { when a var xx:TECSnifferObjectRef parameter can be nil, it is changed to xx: TECSnifferObjectRefPtr }
	TECPluginSig = OSType;
	TECConversionInfoPtr = ^TECConversionInfo;
	TECConversionInfo = record
		sourceEncoding: TextEncoding;
		destinationEncoding: TextEncoding;
		reserved1: UInt16;
		reserved2: UInt16;
	end;

<<<<<<< HEAD
	{	 return number of encodings types supported by user's configuraton of the encoding converter 	}
	{
	 *  TECCountAvailableTextEncodings()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function TECCountAvailableTextEncodings(var numberEncodings: ItemCount): OSStatus; external name '_TECCountAvailableTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  TECInternetNameUsageMask
 *  
 *  Discussion:
 *    Mask values that control the mapping between TextEncoding and
 *    IANA charset name or MIB enum.
 }
type
	TECInternetNameUsageMask = UInt32;
const
{ Use one of the following}

  {
   * Use the default type of mapping given other usage information
   * (none currently defined).
   }
	kTECInternetNameDefaultUsageMask = 0;

  {
   * Use the closest possible match between TextEncoding value and IANA
   * charset name or MIB enum
   }
	kTECInternetNameStrictUsageMask = 1;

  {
   * When mapping from IANA charset name or MIB enum to TextEncoding,
   * map to the largest superset of the encoding specified by the
   * charset name or MIB enum (i.e. be tolerant). When mapping from
   * TextEncoding to IANA charset name or MIB enum, typically map to
   * the most generic or widely recognized charset name or MIB enum.
   }
	kTECInternetNameTolerantUsageMask = 2;

{ Special values for MIB enums }
const
	kTEC_MIBEnumDontCare = -1;

{ Additional control flags for TECSetBasicOptions }
const
	kTECDisableFallbacksBit = 16;
	kTECDisableLooseMappingsBit = 17;

const
	kTECDisableFallbacksMask = 1 shl kTECDisableFallbacksBit;
	kTECDisableLooseMappingsMask = 1 shl kTECDisableLooseMappingsBit;


{ return number of encodings types supported by user's configuraton of the encoding converter }
{
 *  TECCountAvailableTextEncodings()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
function TECCountAvailableTextEncodings( var numberEncodings: ItemCount ): OSStatus; external name '_TECCountAvailableTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ fill in an array of type TextEncoding passed in by the user with types of encodings the current configuration of the encoder can handle. }
{
 *  TECGetAvailableTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECGetAvailableTextEncodings( availableEncodings: {variable-size-array} TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus; external name '_TECGetAvailableTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECGetAvailableTextEncodings(availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount): OSStatus; external name '_TECGetAvailableTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetAvailableTextEncodings( availableEncodings: {variable-size-array} TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus; external name '_TECGetAvailableTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ return number of from-to encoding conversion pairs supported  }
{
 *  TECCountDirectTextEncodingConversions()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCountDirectTextEncodingConversions( var numberOfEncodings: ItemCount ): OSStatus; external name '_TECCountDirectTextEncodingConversions';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCountDirectTextEncodingConversions(var numberOfEncodings: ItemCount): OSStatus; external name '_TECCountDirectTextEncodingConversions';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCountDirectTextEncodingConversions( var numberOfEncodings: ItemCount ): OSStatus; external name '_TECCountDirectTextEncodingConversions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ fill in an array of type TextEncodingPair passed in by the user with types of encoding pairs the current configuration of the encoder can handle. }
{
 *  TECGetDirectTextEncodingConversions()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECGetDirectTextEncodingConversions( availableConversions: {variable-size-array} TECConversionInfoPtr; maxAvailableConversions: ItemCount; var actualAvailableConversions: ItemCount ): OSStatus; external name '_TECGetDirectTextEncodingConversions';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECGetDirectTextEncodingConversions(availableConversions: TECConversionInfoPtr; maxAvailableConversions: ItemCount; var actualAvailableConversions: ItemCount): OSStatus; external name '_TECGetDirectTextEncodingConversions';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetDirectTextEncodingConversions( availableConversions: {variable-size-array} TECConversionInfoPtr; maxAvailableConversions: ItemCount; var actualAvailableConversions: ItemCount ): OSStatus; external name '_TECGetDirectTextEncodingConversions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ return number of encodings a given encoding can be converter into }
{
 *  TECCountDestinationTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCountDestinationTextEncodings( inputEncoding: TextEncoding; var numberOfEncodings: ItemCount ): OSStatus; external name '_TECCountDestinationTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCountDestinationTextEncodings(inputEncoding: TextEncoding; var numberOfEncodings: ItemCount): OSStatus; external name '_TECCountDestinationTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCountDestinationTextEncodings( inputEncoding: TextEncoding; var numberOfEncodings: ItemCount ): OSStatus; external name '_TECCountDestinationTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ fill in an array of type TextEncodingPair passed in by the user with types of encodings pairs the current configuration of the encoder can handle. }
{
 *  TECGetDestinationTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECGetDestinationTextEncodings( inputEncoding: TextEncoding; destinationEncodings: {variable-size-array} TextEncodingPtr; maxDestinationEncodings: ItemCount; var actualDestinationEncodings: ItemCount ): OSStatus; external name '_TECGetDestinationTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECGetDestinationTextEncodings(inputEncoding: TextEncoding; destinationEncodings: TextEncodingPtr; maxDestinationEncodings: ItemCount; var actualDestinationEncodings: ItemCount): OSStatus; external name '_TECGetDestinationTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetDestinationTextEncodings( inputEncoding: TextEncoding; destinationEncodings: {variable-size-array} TextEncodingPtr; maxDestinationEncodings: ItemCount; var actualDestinationEncodings: ItemCount ): OSStatus; external name '_TECGetDestinationTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ get info about a text encoding }
{
 *  TECGetTextEncodingInternetName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
function TECGetTextEncodingInternetName( textEncoding_: TextEncoding; var encodingName: Str255 ): OSStatus; external name '_TECGetTextEncodingInternetName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
<<<<<<< HEAD
function TECGetTextEncodingInternetName(textEncoding_: TextEncoding; var encodingName: Str255): OSStatus; external name '_TECGetTextEncodingInternetName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetTextEncodingInternetName( textEncoding_: TextEncoding; var encodingName: Str255 ): OSStatus; external name '_TECGetTextEncodingInternetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECGetTextEncodingFromInternetName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
function TECGetTextEncodingFromInternetName( var textEncoding_: TextEncoding; const (*var*) encodingName: Str255 ): OSStatus; external name '_TECGetTextEncodingFromInternetName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
<<<<<<< HEAD
function TECGetTextEncodingFromInternetName(var textEncoding_: TextEncoding; const (*var*) encodingName: Str255): OSStatus; external name '_TECGetTextEncodingFromInternetName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetTextEncodingFromInternetName( var textEncoding_: TextEncoding; const (*var*) encodingName: Str255 ): OSStatus; external name '_TECGetTextEncodingFromInternetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ create/dispose converters }
{
 *  TECCreateConverter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
function TECCreateConverter( var newEncodingConverter: TECObjectRef; inputEncoding: TextEncoding; outputEncoding: TextEncoding ): OSStatus; external name '_TECCreateConverter';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
<<<<<<< HEAD
function TECCreateConverter(var newEncodingConverter: TECObjectRef; inputEncoding: TextEncoding; outputEncoding: TextEncoding): OSStatus; external name '_TECCreateConverter';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCreateConverter( var newEncodingConverter: TECObjectRef; inputEncoding: TextEncoding; outputEncoding: TextEncoding ): OSStatus; external name '_TECCreateConverter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECCreateConverterFromPath()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCreateConverterFromPath( var newEncodingConverter: TECObjectRef; {const} inPath: {variable-size-array} TextEncodingPtr; inEncodings: ItemCount ): OSStatus; external name '_TECCreateConverterFromPath';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCreateConverterFromPath(var newEncodingConverter: TECObjectRef; inPath: TextEncodingPtr; inEncodings: ItemCount): OSStatus; external name '_TECCreateConverterFromPath';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCreateConverterFromPath( var newEncodingConverter: TECObjectRef; {const} inPath: {variable-size-array} TextEncodingPtr; inEncodings: ItemCount ): OSStatus; external name '_TECCreateConverterFromPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECDisposeConverter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
function TECDisposeConverter( newEncodingConverter: TECObjectRef ): OSStatus; external name '_TECDisposeConverter';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
<<<<<<< HEAD
function TECDisposeConverter(newEncodingConverter: TECObjectRef): OSStatus; external name '_TECDisposeConverter';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECDisposeConverter( newEncodingConverter: TECObjectRef ): OSStatus; external name '_TECDisposeConverter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ convert text encodings }
{
 *  TECClearConverterContextInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECClearConverterContextInfo( encodingConverter: TECObjectRef ): OSStatus; external name '_TECClearConverterContextInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECClearConverterContextInfo(encodingConverter: TECObjectRef): OSStatus; external name '_TECClearConverterContextInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECClearConverterContextInfo( encodingConverter: TECObjectRef ): OSStatus; external name '_TECClearConverterContextInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECConvertText()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECConvertText( encodingConverter: TECObjectRef; inputBuffer: ConstTextPtr; inputBufferLength: ByteCount; var actualInputLength: ByteCount; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount ): OSStatus; external name '_TECConvertText';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECConvertText(encodingConverter: TECObjectRef; inputBuffer: ConstTextPtr; inputBufferLength: ByteCount; var actualInputLength: ByteCount; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount): OSStatus; external name '_TECConvertText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECConvertText( encodingConverter: TECObjectRef; inputBuffer: ConstTextPtr; inputBufferLength: ByteCount; var actualInputLength: ByteCount; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount ): OSStatus; external name '_TECConvertText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECFlushText()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECFlushText( encodingConverter: TECObjectRef; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount ): OSStatus; external name '_TECFlushText';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECFlushText(encodingConverter: TECObjectRef; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount): OSStatus; external name '_TECFlushText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECFlushText( encodingConverter: TECObjectRef; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount ): OSStatus; external name '_TECFlushText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ one-to-many routines }
{
 *  TECCountSubTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCountSubTextEncodings( inputEncoding: TextEncoding; var numberOfEncodings: ItemCount ): OSStatus; external name '_TECCountSubTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCountSubTextEncodings(inputEncoding: TextEncoding; var numberOfEncodings: ItemCount): OSStatus; external name '_TECCountSubTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCountSubTextEncodings( inputEncoding: TextEncoding; var numberOfEncodings: ItemCount ): OSStatus; external name '_TECCountSubTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECGetSubTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECGetSubTextEncodings( inputEncoding: TextEncoding; subEncodings: {variable-size-array} TextEncodingPtr; maxSubEncodings: ItemCount; var actualSubEncodings: ItemCount ): OSStatus; external name '_TECGetSubTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECGetSubTextEncodings(inputEncoding: TextEncoding; subEncodings: TextEncodingPtr; maxSubEncodings: ItemCount; var actualSubEncodings: ItemCount): OSStatus; external name '_TECGetSubTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetSubTextEncodings( inputEncoding: TextEncoding; subEncodings: {variable-size-array} TextEncodingPtr; maxSubEncodings: ItemCount; var actualSubEncodings: ItemCount ): OSStatus; external name '_TECGetSubTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECGetEncodingList()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Parameters:
 *    
 *    encodingConverter:
 *      The encodingConverter to return the encoding list for
 *    
 *    numEncodings:
 *      On exit, the number of encodings in encodingList
 *    
 *    encodingList:
 *      On exit, a handle containing numEncodings values of type
 *      TextEncoding, for each known encoding.  Do not dispose of this
 *      handle.
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
function TECGetEncodingList( encodingConverter: TECObjectRef; var numEncodings: ItemCount; var encodingList: Handle ): OSStatus; external name '_TECGetEncodingList';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.1 and later
 }
<<<<<<< HEAD
function TECGetEncodingList(encodingConverter: TECObjectRef; var numEncodings: ItemCount; var encodingList: Handle): OSStatus; external name '_TECGetEncodingList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetEncodingList( encodingConverter: TECObjectRef; var numEncodings: ItemCount; var encodingList: Handle ): OSStatus; external name '_TECGetEncodingList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECCreateOneToManyConverter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCreateOneToManyConverter( var newEncodingConverter: TECObjectRef; inputEncoding: TextEncoding; numOutputEncodings: ItemCount; {const} outputEncodings: {variable-size-array} TextEncodingPtr ): OSStatus; external name '_TECCreateOneToManyConverter';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCreateOneToManyConverter(var newEncodingConverter: TECObjectRef; inputEncoding: TextEncoding; numOutputEncodings: ItemCount; outputEncodings: TextEncodingPtr): OSStatus; external name '_TECCreateOneToManyConverter';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCreateOneToManyConverter( var newEncodingConverter: TECObjectRef; inputEncoding: TextEncoding; numOutputEncodings: ItemCount; {const} outputEncodings: {variable-size-array} TextEncodingPtr ): OSStatus; external name '_TECCreateOneToManyConverter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECConvertTextToMultipleEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECConvertTextToMultipleEncodings( encodingConverter: TECObjectRef; inputBuffer: ConstTextPtr; inputBufferLength: ByteCount; var actualInputLength: ByteCount; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount; outEncodingsBuffer: {variable-size-array} TextEncodingRunPtr; maxOutEncodingRuns: ItemCount; var actualOutEncodingRuns: ItemCount ): OSStatus; external name '_TECConvertTextToMultipleEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECConvertTextToMultipleEncodings(encodingConverter: TECObjectRef; inputBuffer: ConstTextPtr; inputBufferLength: ByteCount; var actualInputLength: ByteCount; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount; var outEncodingsBuffer: TextEncodingRun; maxOutEncodingRuns: ItemCount; var actualOutEncodingRuns: ItemCount): OSStatus; external name '_TECConvertTextToMultipleEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECConvertTextToMultipleEncodings( encodingConverter: TECObjectRef; inputBuffer: ConstTextPtr; inputBufferLength: ByteCount; var actualInputLength: ByteCount; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount; outEncodingsBuffer: {variable-size-array} TextEncodingRunPtr; maxOutEncodingRuns: ItemCount; var actualOutEncodingRuns: ItemCount ): OSStatus; external name '_TECConvertTextToMultipleEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECFlushMultipleEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECFlushMultipleEncodings( encodingConverter: TECObjectRef; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount; outEncodingsBuffer: {variable-size-array} TextEncodingRunPtr; maxOutEncodingRuns: ItemCount; var actualOutEncodingRuns: ItemCount ): OSStatus; external name '_TECFlushMultipleEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECFlushMultipleEncodings(encodingConverter: TECObjectRef; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount; var outEncodingsBuffer: TextEncodingRun; maxOutEncodingRuns: ItemCount; var actualOutEncodingRuns: ItemCount): OSStatus; external name '_TECFlushMultipleEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECFlushMultipleEncodings( encodingConverter: TECObjectRef; outputBuffer: TextPtr; outputBufferLength: ByteCount; var actualOutputLength: ByteCount; outEncodingsBuffer: {variable-size-array} TextEncodingRunPtr; maxOutEncodingRuns: ItemCount; var actualOutEncodingRuns: ItemCount ): OSStatus; external name '_TECFlushMultipleEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ international internet info }
{
 *  TECCountWebTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCountWebTextEncodings( locale: RegionCode; var numberEncodings: ItemCount ): OSStatus; external name '_TECCountWebTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCountWebTextEncodings(locale: RegionCode; var numberEncodings: ItemCount): OSStatus; external name '_TECCountWebTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCountWebTextEncodings( locale: RegionCode; var numberEncodings: ItemCount ): OSStatus; external name '_TECCountWebTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECGetWebTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECGetWebTextEncodings( locale: RegionCode; availableEncodings: {variable-size-array} TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus; external name '_TECGetWebTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECGetWebTextEncodings(locale: RegionCode; availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount): OSStatus; external name '_TECGetWebTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetWebTextEncodings( locale: RegionCode; availableEncodings: {variable-size-array} TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus; external name '_TECGetWebTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECCountMailTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCountMailTextEncodings( locale: RegionCode; var numberEncodings: ItemCount ): OSStatus; external name '_TECCountMailTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCountMailTextEncodings(locale: RegionCode; var numberEncodings: ItemCount): OSStatus; external name '_TECCountMailTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCountMailTextEncodings( locale: RegionCode; var numberEncodings: ItemCount ): OSStatus; external name '_TECCountMailTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECGetMailTextEncodings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECGetMailTextEncodings( locale: RegionCode; availableEncodings: {variable-size-array} TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus; external name '_TECGetMailTextEncodings';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECGetMailTextEncodings(locale: RegionCode; availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount): OSStatus; external name '_TECGetMailTextEncodings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetMailTextEncodings( locale: RegionCode; availableEncodings: {variable-size-array} TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus; external name '_TECGetMailTextEncodings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ examine text encodings }
{
 *  TECCountAvailableSniffers()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCountAvailableSniffers( var numberOfEncodings: ItemCount ): OSStatus; external name '_TECCountAvailableSniffers';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCountAvailableSniffers(var numberOfEncodings: ItemCount): OSStatus; external name '_TECCountAvailableSniffers';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCountAvailableSniffers( var numberOfEncodings: ItemCount ): OSStatus; external name '_TECCountAvailableSniffers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECGetAvailableSniffers()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECGetAvailableSniffers( availableSniffers: {variable-size-array} TextEncodingPtr; maxAvailableSniffers: ItemCount; var actualAvailableSniffers: ItemCount ): OSStatus; external name '_TECGetAvailableSniffers';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECGetAvailableSniffers(availableSniffers: TextEncodingPtr; maxAvailableSniffers: ItemCount; var actualAvailableSniffers: ItemCount): OSStatus; external name '_TECGetAvailableSniffers';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECGetAvailableSniffers( availableSniffers: {variable-size-array} TextEncodingPtr; maxAvailableSniffers: ItemCount; var actualAvailableSniffers: ItemCount ): OSStatus; external name '_TECGetAvailableSniffers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECCreateSniffer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECCreateSniffer( var encodingSniffer: TECSnifferObjectRef; {const} testEncodings: {variable-size-array} TextEncodingPtr; numTextEncodings: ItemCount ): OSStatus; external name '_TECCreateSniffer';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECCreateSniffer(var encodingSniffer: TECSnifferObjectRef; testEncodings: TextEncodingPtr; numTextEncodings: ItemCount): OSStatus; external name '_TECCreateSniffer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECCreateSniffer( var encodingSniffer: TECSnifferObjectRef; {const} testEncodings: {variable-size-array} TextEncodingPtr; numTextEncodings: ItemCount ): OSStatus; external name '_TECCreateSniffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECSniffTextEncoding()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECSniffTextEncoding( encodingSniffer: TECSnifferObjectRef; inputBuffer: ConstTextPtr; inputBufferLength: ByteCount; testEncodings: {variable-size-array} TextEncodingPtr; numTextEncodings: ItemCount; numErrsArray: {variable-size-array} ItemCountPtr; maxErrs: ItemCount; numFeaturesArray: {variable-size-array} ItemCountPtr; maxFeatures: ItemCount ): OSStatus; external name '_TECSniffTextEncoding';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECSniffTextEncoding(encodingSniffer: TECSnifferObjectRef; inputBuffer: TextPtr; inputBufferLength: ByteCount; testEncodings: TextEncodingPtr; numTextEncodings: ItemCount; var numErrsArray: ItemCount; maxErrs: ItemCount; var numFeaturesArray: ItemCount; maxFeatures: ItemCount): OSStatus; external name '_TECSniffTextEncoding';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECSniffTextEncoding( encodingSniffer: TECSnifferObjectRef; inputBuffer: ConstTextPtr; inputBufferLength: ByteCount; testEncodings: {variable-size-array} TextEncodingPtr; numTextEncodings: ItemCount; numErrsArray: {variable-size-array} ItemCountPtr; maxErrs: ItemCount; numFeaturesArray: {variable-size-array} ItemCountPtr; maxFeatures: ItemCount ): OSStatus; external name '_TECSniffTextEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECDisposeSniffer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECDisposeSniffer( encodingSniffer: TECSnifferObjectRef ): OSStatus; external name '_TECDisposeSniffer';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
<<<<<<< HEAD
function TECDisposeSniffer(encodingSniffer: TECSnifferObjectRef): OSStatus; external name '_TECDisposeSniffer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TECDisposeSniffer( encodingSniffer: TECSnifferObjectRef ): OSStatus; external name '_TECDisposeSniffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TECClearSnifferContextInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECClearSnifferContextInfo( encodingSniffer: TECSnifferObjectRef ): OSStatus; external name '_TECClearSnifferContextInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  TECSetBasicOptions()
 *  
 *  Summary:
 *    Sets encodingConverter options affecting
 *    TECConvertText[ToMultipleEncodings].
 *  
 *  Parameters:
 *    
 *    encodingConverter:
 *      The high-level encoding converter object created by
 *      TECCreateConverter or TECCreateOneToManyConverter whose
 *      behavior is to be modified by the options specified in
 *      controlFlags.
 *    
 *    controlFlags:
 *      A bit mask specifying the desired options. The following mask
 *      constants are valid for this parameter; multiple mask constants
 *      may be ORed together to set multiple options; passing 0 for
 *      this parameter clears all options: 
 *      
 *      kUnicodeForceASCIIRangeMask, kUnicodeNoHalfwidthCharsMask
 *      (defined in UnicodeConverter.h) 
 *      
 *      kTECDisableFallbacksMask, kTECDisableLooseMappingsMask (defined
 *      above) - loose and fallback mappings are both enabled by
 *      default for the TextEncodingConverter.h conversion APIs
 *      (TECConvertText, TECConvertTextToMultipleEncodings), unlike the
 *      behavior of the conversion APIs in UnicodeConverter.h. These
 *      options may be used to disable loose and/or fallback mappings
 *      for the TextEncodingConverter.h conversion APIs.
 *  
 *  Result:
 *    The function returns paramErr for invalid masks,
 *    kTECCorruptConverterErr for an invalid encodingConverter, noErr
 *    otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.5 and later
 }
function TECSetBasicOptions( encodingConverter: TECObjectRef; controlFlags: OptionBits ): OSStatus; external name '_TECSetBasicOptions';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew


{ Map TextEncoding values to/from IANA charset names and/or MIB enums, with usage control }
{
 *  TECCopyTextEncodingInternetNameAndMIB()
 *  
 *  Summary:
 *    Converts a TextEncoding value to an IANA charset name and/or a
 *    MIB enum value
 *  
 *  Discussion:
 *    Given a TextEncoding value, this function maps it to an IANA
 *    charset name (if encodingNamePtr is non-NULL) and/or a MIB enum
 *    value (if mibEnumPtr is non-NULL), as specified by the usage
 *    parameter.
 *  
 *  Parameters:
 *    
 *    textEncoding:
 *      A TextEncoding value to map to a charset name and/or MIB enum.
 *    
 *    usage:
 *      Specifies the type of mapping desired (see
 *      TECInternetNameUsageMask above).
 *    
 *    encodingNamePtr:
 *      If non-NULL, is a pointer to a CStringRef for an immutable
 *      CFString created by this function; when the caller is finished
 *      with it, the caller must dispose of it by calling CFRelease.
 *    
 *    mibEnumPtr:
 *      If non-NULL, is a pointer to an SInt32 that will be set to the
 *      appropriate MIB enum value, or to 0 (or kTEC_MIBEnumDontCare)
 *      if there is no appropriate MIB enum value (valid MIB enums
 *      begin at 3).
 *  
 *  Result:
 *    The function returns paramErr if encodingNamePtr and mibEnumPtr
 *    are both NULL. It returns kTextUnsupportedEncodingErr if it has
 *    no data for the supplied textEncoding. It returns noErr if it
 *    found useful data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function TECCopyTextEncodingInternetNameAndMIB( textEncoding_: TextEncoding; usage: TECInternetNameUsageMask; encodingNamePtr: CFStringRefPtr { can be NULL }; mibEnumPtr: SInt32Ptr { can be NULL } ): OSStatus; external name '_TECCopyTextEncodingInternetNameAndMIB';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  TECGetTextEncodingFromInternetNameOrMIB()
 *  
 *  Summary:
 *    Converts an IANA charset name or a MIB enum value to a
 *    TextEncoding value
 *  
 *  Discussion:
 *    If encodingName is non-NULL, this function treats it as an IANA
 *    charset name and maps it to a TextEncoding value; in this case
 *    mibEnum is ignored, and may be set to kTEC_MIBEnumDontCare.
 *    Otherwise, this function maps the mibEnum to a TextEncoding
 *    value. In either case, the mapping is controlled by the usage
 *    parameter. The textEncodingPtr parameter must be non-NULL.
 *  
 *  Result:
 *    The function returns paramErr if textEncodingPtr is NULL. It
 *    returns kTextUnsupportedEncodingErr if it has no data for the
 *    supplied encodingName or mibEnum. It returns noErr if it found
 *    useful data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function TECGetTextEncodingFromInternetNameOrMIB( var textEncodingPtr: TextEncoding; usage: TECInternetNameUsageMask; encodingName: CFStringRef; mibEnum: SInt32 ): OSStatus; external name '_TECGetTextEncodingFromInternetNameOrMIB';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.2 and later
 }
function TECClearSnifferContextInfo( encodingSniffer: TECSnifferObjectRef ): OSStatus; external name '_TECClearSnifferContextInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TECSetBasicOptions()
 *  
 *  Summary:
 *    Sets encodingConverter options affecting
 *    TECConvertText[ToMultipleEncodings].
 *  
 *  Parameters:
 *    
 *    encodingConverter:
 *      The high-level encoding converter object created by
 *      TECCreateConverter or TECCreateOneToManyConverter whose
 *      behavior is to be modified by the options specified in
 *      controlFlags.
 *    
 *    controlFlags:
 *      A bit mask specifying the desired options. The following mask
 *      constants are valid for this parameter; multiple mask constants
 *      may be ORed together to set multiple options; passing 0 for
 *      this parameter clears all options: 
 *      
 *      kUnicodeForceASCIIRangeMask, kUnicodeNoHalfwidthCharsMask
 *      (defined in UnicodeConverter.h) 
 *      
 *      kTECDisableFallbacksMask, kTECDisableLooseMappingsMask (defined
 *      above) - loose and fallback mappings are both enabled by
 *      default for the TextEncodingConverter.h conversion APIs
 *      (TECConvertText, TECConvertTextToMultipleEncodings), unlike the
 *      behavior of the conversion APIs in UnicodeConverter.h. These
 *      options may be used to disable loose and/or fallback mappings
 *      for the TextEncodingConverter.h conversion APIs.
 *  
 *  Result:
 *    The function returns paramErr for invalid masks,
 *    kTECCorruptConverterErr for an invalid encodingConverter, noErr
 *    otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   in TextEncodingConverter 1.5 and later
 }
function TECSetBasicOptions( encodingConverter: TECObjectRef; controlFlags: OptionBits ): OSStatus; external name '_TECSetBasicOptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ Map TextEncoding values to/from IANA charset names and/or MIB enums, with usage control }
{
 *  TECCopyTextEncodingInternetNameAndMIB()
 *  
 *  Summary:
 *    Converts a TextEncoding value to an IANA charset name and/or a
 *    MIB enum value
 *  
 *  Discussion:
 *    Given a TextEncoding value, this function maps it to an IANA
 *    charset name (if encodingNamePtr is non-NULL) and/or a MIB enum
 *    value (if mibEnumPtr is non-NULL), as specified by the usage
 *    parameter.
 *  
 *  Parameters:
 *    
 *    textEncoding:
 *      A TextEncoding value to map to a charset name and/or MIB enum.
 *    
 *    usage:
 *      Specifies the type of mapping desired (see
 *      TECInternetNameUsageMask above).
 *    
 *    encodingNamePtr:
 *      If non-NULL, is a pointer to a CStringRef for an immutable
 *      CFString created by this function; when the caller is finished
 *      with it, the caller must dispose of it by calling CFRelease.
 *    
 *    mibEnumPtr:
 *      If non-NULL, is a pointer to an SInt32 that will be set to the
 *      appropriate MIB enum value, or to 0 (or kTEC_MIBEnumDontCare)
 *      if there is no appropriate MIB enum value (valid MIB enums
 *      begin at 3).
 *  
 *  Result:
 *    The function returns paramErr if encodingNamePtr and mibEnumPtr
 *    are both NULL. It returns kTextUnsupportedEncodingErr if it has
 *    no data for the supplied textEncoding. It returns noErr if it
 *    found useful data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function TECCopyTextEncodingInternetNameAndMIB( textEncoding_: TextEncoding; usage: TECInternetNameUsageMask; encodingNamePtr: CFStringRefPtr { can be NULL }; mibEnumPtr: SInt32Ptr { can be NULL } ): OSStatus; external name '_TECCopyTextEncodingInternetNameAndMIB';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TECGetTextEncodingFromInternetNameOrMIB()
 *  
 *  Summary:
 *    Converts an IANA charset name or a MIB enum value to a
 *    TextEncoding value
 *  
 *  Discussion:
 *    If encodingName is non-NULL, this function treats it as an IANA
 *    charset name and maps it to a TextEncoding value; in this case
 *    mibEnum is ignored, and may be set to kTEC_MIBEnumDontCare.
 *    Otherwise, this function maps the mibEnum to a TextEncoding
 *    value. In either case, the mapping is controlled by the usage
 *    parameter. The textEncodingPtr parameter must be non-NULL.
 *  
 *  Result:
 *    The function returns paramErr if textEncodingPtr is NULL. It
 *    returns kTextUnsupportedEncodingErr if it has no data for the
 *    supplied encodingName or mibEnum. It returns noErr if it found
 *    useful data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function TECGetTextEncodingFromInternetNameOrMIB( var textEncodingPtr: TextEncoding; usage: TECInternetNameUsageMask; encodingName: CFStringRef; mibEnum: SInt32 ): OSStatus; external name '_TECGetTextEncodingFromInternetNameOrMIB';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


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
