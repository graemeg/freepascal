{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/UnicodeConverter.h
 
     Contains:   Types, constants, and prototypes for Unicode Converter
 
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
     File:       UnicodeConverter.p
=======
     File:       CarbonCore/UnicodeConverter.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Types, constants, and prototypes for Unicode Converter
 
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

unit UnicodeConverter;
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
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes,TextCommon;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,TextCommon,MixedMode;
=======
uses MacTypes,TextCommon;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,TextCommon,MixedMode;

>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ Unicode conversion contexts: }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TextToUnicodeInfo = ^SInt32; { an opaque type }
	TextToUnicodeInfoPtr = ^TextToUnicodeInfo;  { when a var xx:TextToUnicodeInfo parameter can be nil, it is changed to xx: TextToUnicodeInfoPtr }
	UnicodeToTextInfo = ^SInt32; { an opaque type }
	UnicodeToTextInfoPtr = ^UnicodeToTextInfo;  { when a var xx:UnicodeToTextInfo parameter can be nil, it is changed to xx: UnicodeToTextInfoPtr }
	UnicodeToTextRunInfo = ^SInt32; { an opaque type }
	UnicodeToTextRunInfoPtr = ^UnicodeToTextRunInfo;  { when a var xx:UnicodeToTextRunInfo parameter can be nil, it is changed to xx: UnicodeToTextRunInfoPtr }
	ConstTextToUnicodeInfo = TextToUnicodeInfo;
	ConstUnicodeToTextInfo = UnicodeToTextInfo;
{ UnicodeMapVersion type & values }
type
	UnicodeMapVersion = SInt32;
const
	kUnicodeUseLatestMapping = -1;
	kUnicodeUseHFSPlusMapping = 4;

{ Types used in conversion }
type
	UnicodeMapping = record
		unicodeEncoding: TextEncoding;
		otherEncoding: TextEncoding;
		mappingVersion: UnicodeMapVersion;
	end;
	UnicodeMappingPtr = ^UnicodeMapping;
type
	ConstUnicodeMappingPtr = ^UnicodeMapping;
{ Control flags for ConvertFromUnicodeToText and ConvertFromTextToUnicode }
const
	kUnicodeUseFallbacksBit = 0;
	kUnicodeKeepInfoBit = 1;
	kUnicodeDirectionalityBits = 2;
	kUnicodeVerticalFormBit = 4;
	kUnicodeLooseMappingsBit = 5;
	kUnicodeStringUnterminatedBit = 6;
	kUnicodeTextRunBit = 7;
	kUnicodeKeepSameEncodingBit = 8;
	kUnicodeForceASCIIRangeBit = 9;
	kUnicodeNoHalfwidthCharsBit = 10;
	kUnicodeTextRunHeuristicsBit = 11;
	kUnicodeMapLineFeedToReturnBit = 12;  {    if kUnicodeUseExternalEncodingFormBit is not set, }
                                        {    input/output UTF-16 (and UTF-32) is assumed to be in native endian. }
                                        {    if kUnicodeUseExternalEncodingFormBit is set, }
                                        {    input UTF-16 (and UTF-32) is assumed to be in big endian }
                                        {    unless it begins with a byte-order-mark, }
                                        {    and output UTF-16 (and UTF-32) will be in big endian. }
	kUnicodeUseExternalEncodingFormBit = 13; {    Bits 16-17 are defined in TextEncodingConverter.h for TECSetBasicOptions }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
	kUnicodeUseFallbacksMask = 1 shl kUnicodeUseFallbacksBit;
	kUnicodeKeepInfoMask = 1 shl kUnicodeKeepInfoBit;
	kUnicodeDirectionalityMask = 3 shl kUnicodeDirectionalityBits;
	kUnicodeVerticalFormMask = 1 shl kUnicodeVerticalFormBit;
	kUnicodeLooseMappingsMask = 1 shl kUnicodeLooseMappingsBit;
	kUnicodeStringUnterminatedMask = 1 shl kUnicodeStringUnterminatedBit;
	kUnicodeTextRunMask = 1 shl kUnicodeTextRunBit;
	kUnicodeKeepSameEncodingMask = 1 shl kUnicodeKeepSameEncodingBit;
	kUnicodeForceASCIIRangeMask = 1 shl kUnicodeForceASCIIRangeBit;
	kUnicodeNoHalfwidthCharsMask = 1 shl kUnicodeNoHalfwidthCharsBit;
	kUnicodeTextRunHeuristicsMask = 1 shl kUnicodeTextRunHeuristicsBit;
	kUnicodeMapLineFeedToReturnMask = 1 shl kUnicodeMapLineFeedToReturnBit; {    if kUnicodeUseExternalEncodingFormBit is not set, }
                                        {    input/output UTF-16 (and UTF-32) is assumed to be in native endian. }
                                        {    if kUnicodeUseExternalEncodingFormBit is set, }
                                        {    input UTF-16 (and UTF-32) is assumed to be in big endian }
                                        {    unless it begins with a byte-order-mark, }
                                        {    and output UTF-16 (and UTF-32) will be in big endian. }
	kUnicodeUseExternalEncodingFormMask = 1 shl kUnicodeUseExternalEncodingFormBit;

{ Values for kUnicodeDirectionality field }
const
	kUnicodeDefaultDirection = 0;
	kUnicodeLeftToRight = 1;
	kUnicodeRightToLeft = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ Directionality masks for control flags }
const
	kUnicodeDefaultDirectionMask = kUnicodeDefaultDirection shl kUnicodeDirectionalityBits;
	kUnicodeLeftToRightMask = kUnicodeLeftToRight shl kUnicodeDirectionalityBits;
	kUnicodeRightToLeftMask = kUnicodeRightToLeft shl kUnicodeDirectionalityBits;


{ Control flags for TruncateForUnicodeToText: }
{
   Now TruncateForUnicodeToText uses control flags from the same set as used by
   ConvertFromTextToUnicode, ConvertFromUnicodeToText, etc., but only
   kUnicodeStringUnterminatedMask is meaningful for TruncateForUnicodeToText.
   
   Previously two special control flags were defined for TruncateForUnicodeToText:
        kUnicodeTextElementSafeBit = 0
        kUnicodeRestartSafeBit = 1
   However, neither of these was implemented.
   Instead of implementing kUnicodeTextElementSafeBit, we now use
   kUnicodeStringUnterminatedMask since it accomplishes the same thing and avoids
   having special flags just for TruncateForUnicodeToText
   Also, kUnicodeRestartSafeBit is unnecessary, since restart-safeness is handled by
   setting kUnicodeKeepInfoBit with ConvertFromUnicodeToText.
   If TruncateForUnicodeToText is called with one or both of the old special control
   flags set (bits 0 or 1), it will not generate a paramErr, but the old bits have no
   effect on its operation.
}

=======

=======

const
	kUnicodeUseFallbacksMask = 1 shl kUnicodeUseFallbacksBit;
	kUnicodeKeepInfoMask = 1 shl kUnicodeKeepInfoBit;
	kUnicodeDirectionalityMask = 3 shl kUnicodeDirectionalityBits;
	kUnicodeVerticalFormMask = 1 shl kUnicodeVerticalFormBit;
	kUnicodeLooseMappingsMask = 1 shl kUnicodeLooseMappingsBit;
	kUnicodeStringUnterminatedMask = 1 shl kUnicodeStringUnterminatedBit;
	kUnicodeTextRunMask = 1 shl kUnicodeTextRunBit;
	kUnicodeKeepSameEncodingMask = 1 shl kUnicodeKeepSameEncodingBit;
	kUnicodeForceASCIIRangeMask = 1 shl kUnicodeForceASCIIRangeBit;
	kUnicodeNoHalfwidthCharsMask = 1 shl kUnicodeNoHalfwidthCharsBit;
	kUnicodeTextRunHeuristicsMask = 1 shl kUnicodeTextRunHeuristicsBit;
	kUnicodeMapLineFeedToReturnMask = 1 shl kUnicodeMapLineFeedToReturnBit; {    if kUnicodeUseExternalEncodingFormBit is not set, }
                                        {    input/output UTF-16 (and UTF-32) is assumed to be in native endian. }
                                        {    if kUnicodeUseExternalEncodingFormBit is set, }
                                        {    input UTF-16 (and UTF-32) is assumed to be in big endian }
                                        {    unless it begins with a byte-order-mark, }
                                        {    and output UTF-16 (and UTF-32) will be in big endian. }
	kUnicodeUseExternalEncodingFormMask = 1 shl kUnicodeUseExternalEncodingFormBit;

{ Values for kUnicodeDirectionality field }
const
	kUnicodeDefaultDirection = 0;
	kUnicodeLeftToRight = 1;
	kUnicodeRightToLeft = 2;

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
{ Directionality masks for control flags }
const
	kUnicodeDefaultDirectionMask = kUnicodeDefaultDirection shl kUnicodeDirectionalityBits;
	kUnicodeLeftToRightMask = kUnicodeLeftToRight shl kUnicodeDirectionalityBits;
	kUnicodeRightToLeftMask = kUnicodeRightToLeft shl kUnicodeDirectionalityBits;


{ Control flags for TruncateForUnicodeToText: }
{
   Now TruncateForUnicodeToText uses control flags from the same set as used by
   ConvertFromTextToUnicode, ConvertFromUnicodeToText, etc., but only
   kUnicodeStringUnterminatedMask is meaningful for TruncateForUnicodeToText.
   
   Previously two special control flags were defined for TruncateForUnicodeToText:
        kUnicodeTextElementSafeBit = 0
        kUnicodeRestartSafeBit = 1
   However, neither of these was implemented.
   Instead of implementing kUnicodeTextElementSafeBit, we now use
   kUnicodeStringUnterminatedMask since it accomplishes the same thing and avoids
   having special flags just for TruncateForUnicodeToText
   Also, kUnicodeRestartSafeBit is unnecessary, since restart-safeness is handled by
   setting kUnicodeKeepInfoBit with ConvertFromUnicodeToText.
   If TruncateForUnicodeToText is called with one or both of the old special control
   flags set (bits 0 or 1), it will not generate a paramErr, but the old bits have no
   effect on its operation.
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
{ Filter bits for filter field in QueryUnicodeMappings and CountUnicodeMappings: }
const
	kUnicodeMatchUnicodeBaseBit = 0;
	kUnicodeMatchUnicodeVariantBit = 1;
	kUnicodeMatchUnicodeFormatBit = 2;
	kUnicodeMatchOtherBaseBit = 3;
	kUnicodeMatchOtherVariantBit = 4;
	kUnicodeMatchOtherFormatBit = 5;

const
	kUnicodeMatchUnicodeBaseMask = 1 shl kUnicodeMatchUnicodeBaseBit;
	kUnicodeMatchUnicodeVariantMask = 1 shl kUnicodeMatchUnicodeVariantBit;
	kUnicodeMatchUnicodeFormatMask = 1 shl kUnicodeMatchUnicodeFormatBit;
	kUnicodeMatchOtherBaseMask = 1 shl kUnicodeMatchOtherBaseBit;
	kUnicodeMatchOtherVariantMask = 1 shl kUnicodeMatchOtherVariantBit;
	kUnicodeMatchOtherFormatMask = 1 shl kUnicodeMatchOtherFormatBit;

{ Control flags for SetFallbackUnicodeToText }
const
	kUnicodeFallbackSequencingBits = 0;

const
	kUnicodeFallbackSequencingMask = 3 shl kUnicodeFallbackSequencingBits;
	kUnicodeFallbackInterruptSafeMask = 1 shl 2; { To indicate that caller fallback routine doesn’t move memory}

{ values for kUnicodeFallbackSequencing field }
const
	kUnicodeFallbackDefaultOnly = 0;
	kUnicodeFallbackCustomOnly = 1;
	kUnicodeFallbackDefaultFirst = 2;
	kUnicodeFallbackCustomFirst = 3;
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
{ Caller-supplied entry point to a fallback handler }
type
	UnicodeToTextFallbackProcPtr = function( iSrcUniStr: UniCharPtr; iSrcUniStrLen: ByteCount; var oSrcConvLen: ByteCount; oDestStr: TextPtr; iDestStrLen: ByteCount; var oDestConvLen: ByteCount; iInfoPtr: LogicalAddress; iUnicodeMappingPtr: ConstUnicodeMappingPtr ): OSStatus;
	UnicodeToTextFallbackUPP = UnicodeToTextFallbackProcPtr;
{
 *  NewUnicodeToTextFallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewUnicodeToTextFallbackUPP( userRoutine: UnicodeToTextFallbackProcPtr ): UnicodeToTextFallbackUPP; external name '_NewUnicodeToTextFallbackUPP';
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
 *  DisposeUnicodeToTextFallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeUnicodeToTextFallbackUPP( userUPP: UnicodeToTextFallbackUPP ); external name '_DisposeUnicodeToTextFallbackUPP';
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
 *  InvokeUnicodeToTextFallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeUnicodeToTextFallbackUPP( iSrcUniStr: UniCharPtr; iSrcUniStrLen: ByteCount; var oSrcConvLen: ByteCount; oDestStr: TextPtr; iDestStrLen: ByteCount; var oDestConvLen: ByteCount; iInfoPtr: LogicalAddress; iUnicodeMappingPtr: ConstUnicodeMappingPtr; userUPP: UnicodeToTextFallbackUPP ): OSStatus; external name '_InvokeUnicodeToTextFallbackUPP';
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

{ Function prototypes }
=======
=======
>>>>>>> origin/fixes_2_2
	TextToUnicodeInfo    = ^SInt32; { an opaque 32-bit type }
=======
	TextToUnicodeInfo = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	TextToUnicodeInfoPtr = ^TextToUnicodeInfo;  { when a var xx:TextToUnicodeInfo parameter can be nil, it is changed to xx: TextToUnicodeInfoPtr }
	UnicodeToTextInfo = ^SInt32; { an opaque type }
	UnicodeToTextInfoPtr = ^UnicodeToTextInfo;  { when a var xx:UnicodeToTextInfo parameter can be nil, it is changed to xx: UnicodeToTextInfoPtr }
	UnicodeToTextRunInfo = ^SInt32; { an opaque type }
	UnicodeToTextRunInfoPtr = ^UnicodeToTextRunInfo;  { when a var xx:UnicodeToTextRunInfo parameter can be nil, it is changed to xx: UnicodeToTextRunInfoPtr }
	ConstTextToUnicodeInfo = TextToUnicodeInfo;
	ConstUnicodeToTextInfo = UnicodeToTextInfo;
{ UnicodeMapVersion type & values }
type
	UnicodeMapVersion = SInt32;
const
	kUnicodeUseLatestMapping = -1;
	kUnicodeUseHFSPlusMapping = 4;

{ Types used in conversion }
type
	UnicodeMapping = record
		unicodeEncoding: TextEncoding;
		otherEncoding: TextEncoding;
		mappingVersion: UnicodeMapVersion;
	end;
	UnicodeMappingPtr = ^UnicodeMapping;
type
	ConstUnicodeMappingPtr = ^UnicodeMapping;
{ Control flags for ConvertFromUnicodeToText and ConvertFromTextToUnicode }
const
	kUnicodeUseFallbacksBit = 0;
	kUnicodeKeepInfoBit = 1;
	kUnicodeDirectionalityBits = 2;
	kUnicodeVerticalFormBit = 4;
	kUnicodeLooseMappingsBit = 5;
	kUnicodeStringUnterminatedBit = 6;
	kUnicodeTextRunBit = 7;
	kUnicodeKeepSameEncodingBit = 8;
	kUnicodeForceASCIIRangeBit = 9;
	kUnicodeNoHalfwidthCharsBit = 10;
	kUnicodeTextRunHeuristicsBit = 11;
	kUnicodeMapLineFeedToReturnBit = 12;  {    if kUnicodeUseExternalEncodingFormBit is not set, }
                                        {    input/output UTF-16 (and UTF-32) is assumed to be in native endian. }
                                        {    if kUnicodeUseExternalEncodingFormBit is set, }
                                        {    input UTF-16 (and UTF-32) is assumed to be in big endian }
                                        {    unless it begins with a byte-order-mark, }
                                        {    and output UTF-16 (and UTF-32) will be in big endian. }
	kUnicodeUseExternalEncodingFormBit = 13; {    Bits 16-17 are defined in TextEncodingConverter.h for TECSetBasicOptions }

const
	kUnicodeUseFallbacksMask = 1 shl kUnicodeUseFallbacksBit;
	kUnicodeKeepInfoMask = 1 shl kUnicodeKeepInfoBit;
	kUnicodeDirectionalityMask = 3 shl kUnicodeDirectionalityBits;
	kUnicodeVerticalFormMask = 1 shl kUnicodeVerticalFormBit;
	kUnicodeLooseMappingsMask = 1 shl kUnicodeLooseMappingsBit;
	kUnicodeStringUnterminatedMask = 1 shl kUnicodeStringUnterminatedBit;
	kUnicodeTextRunMask = 1 shl kUnicodeTextRunBit;
	kUnicodeKeepSameEncodingMask = 1 shl kUnicodeKeepSameEncodingBit;
	kUnicodeForceASCIIRangeMask = 1 shl kUnicodeForceASCIIRangeBit;
	kUnicodeNoHalfwidthCharsMask = 1 shl kUnicodeNoHalfwidthCharsBit;
	kUnicodeTextRunHeuristicsMask = 1 shl kUnicodeTextRunHeuristicsBit;
	kUnicodeMapLineFeedToReturnMask = 1 shl kUnicodeMapLineFeedToReturnBit; {    if kUnicodeUseExternalEncodingFormBit is not set, }
                                        {    input/output UTF-16 (and UTF-32) is assumed to be in native endian. }
                                        {    if kUnicodeUseExternalEncodingFormBit is set, }
                                        {    input UTF-16 (and UTF-32) is assumed to be in big endian }
                                        {    unless it begins with a byte-order-mark, }
                                        {    and output UTF-16 (and UTF-32) will be in big endian. }
	kUnicodeUseExternalEncodingFormMask = 1 shl kUnicodeUseExternalEncodingFormBit;

{ Values for kUnicodeDirectionality field }
const
	kUnicodeDefaultDirection = 0;
	kUnicodeLeftToRight = 1;
	kUnicodeRightToLeft = 2;

{ Directionality masks for control flags }
const
	kUnicodeDefaultDirectionMask = kUnicodeDefaultDirection shl kUnicodeDirectionalityBits;
	kUnicodeLeftToRightMask = kUnicodeLeftToRight shl kUnicodeDirectionalityBits;
	kUnicodeRightToLeftMask = kUnicodeRightToLeft shl kUnicodeDirectionalityBits;


{ Control flags for TruncateForUnicodeToText: }
{
   Now TruncateForUnicodeToText uses control flags from the same set as used by
   ConvertFromTextToUnicode, ConvertFromUnicodeToText, etc., but only
   kUnicodeStringUnterminatedMask is meaningful for TruncateForUnicodeToText.
   
   Previously two special control flags were defined for TruncateForUnicodeToText:
        kUnicodeTextElementSafeBit = 0
        kUnicodeRestartSafeBit = 1
   However, neither of these was implemented.
   Instead of implementing kUnicodeTextElementSafeBit, we now use
   kUnicodeStringUnterminatedMask since it accomplishes the same thing and avoids
   having special flags just for TruncateForUnicodeToText
   Also, kUnicodeRestartSafeBit is unnecessary, since restart-safeness is handled by
   setting kUnicodeKeepInfoBit with ConvertFromUnicodeToText.
   If TruncateForUnicodeToText is called with one or both of the old special control
   flags set (bits 0 or 1), it will not generate a paramErr, but the old bits have no
   effect on its operation.
}

{ Filter bits for filter field in QueryUnicodeMappings and CountUnicodeMappings: }
const
	kUnicodeMatchUnicodeBaseBit = 0;
	kUnicodeMatchUnicodeVariantBit = 1;
	kUnicodeMatchUnicodeFormatBit = 2;
	kUnicodeMatchOtherBaseBit = 3;
	kUnicodeMatchOtherVariantBit = 4;
	kUnicodeMatchOtherFormatBit = 5;

const
	kUnicodeMatchUnicodeBaseMask = 1 shl kUnicodeMatchUnicodeBaseBit;
	kUnicodeMatchUnicodeVariantMask = 1 shl kUnicodeMatchUnicodeVariantBit;
	kUnicodeMatchUnicodeFormatMask = 1 shl kUnicodeMatchUnicodeFormatBit;
	kUnicodeMatchOtherBaseMask = 1 shl kUnicodeMatchOtherBaseBit;
	kUnicodeMatchOtherVariantMask = 1 shl kUnicodeMatchOtherVariantBit;
	kUnicodeMatchOtherFormatMask = 1 shl kUnicodeMatchOtherFormatBit;

{ Control flags for SetFallbackUnicodeToText }
const
	kUnicodeFallbackSequencingBits = 0;

const
	kUnicodeFallbackSequencingMask = 3 shl kUnicodeFallbackSequencingBits;
	kUnicodeFallbackInterruptSafeMask = 1 shl 2; { To indicate that caller fallback routine doesn’t move memory}

{ values for kUnicodeFallbackSequencing field }
const
	kUnicodeFallbackDefaultOnly = 0;
	kUnicodeFallbackCustomOnly = 1;
	kUnicodeFallbackDefaultFirst = 2;
	kUnicodeFallbackCustomFirst = 3;


{ Caller-supplied entry point to a fallback handler }
type
	UnicodeToTextFallbackProcPtr = function( iSrcUniStr: UniCharPtr; iSrcUniStrLen: ByteCount; var oSrcConvLen: ByteCount; oDestStr: TextPtr; iDestStrLen: ByteCount; var oDestConvLen: ByteCount; iInfoPtr: LogicalAddress; iUnicodeMappingPtr: ConstUnicodeMappingPtr ): OSStatus;
	UnicodeToTextFallbackUPP = UnicodeToTextFallbackProcPtr;
{
 *  NewUnicodeToTextFallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewUnicodeToTextFallbackUPP( userRoutine: UnicodeToTextFallbackProcPtr ): UnicodeToTextFallbackUPP; external name '_NewUnicodeToTextFallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeUnicodeToTextFallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeUnicodeToTextFallbackUPP( userUPP: UnicodeToTextFallbackUPP ); external name '_DisposeUnicodeToTextFallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeUnicodeToTextFallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeUnicodeToTextFallbackUPP( iSrcUniStr: UniCharPtr; iSrcUniStrLen: ByteCount; var oSrcConvLen: ByteCount; oDestStr: TextPtr; iDestStrLen: ByteCount; var oDestConvLen: ByteCount; iInfoPtr: LogicalAddress; iUnicodeMappingPtr: ConstUnicodeMappingPtr; userUPP: UnicodeToTextFallbackUPP ): OSStatus; external name '_InvokeUnicodeToTextFallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Function prototypes }
>>>>>>> origin/fixes_2.4
{
 *  CreateTextToUnicodeInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function CreateTextToUnicodeInfo( iUnicodeMapping: ConstUnicodeMappingPtr; var oTextToUnicodeInfo: TextToUnicodeInfo ): OSStatus; external name '_CreateTextToUnicodeInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function CreateTextToUnicodeInfo(iUnicodeMapping: ConstUnicodeMappingPtr; var oTextToUnicodeInfo: TextToUnicodeInfo): OSStatus; external name '_CreateTextToUnicodeInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CreateTextToUnicodeInfo( iUnicodeMapping: ConstUnicodeMappingPtr; var oTextToUnicodeInfo: TextToUnicodeInfo ): OSStatus; external name '_CreateTextToUnicodeInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CreateTextToUnicodeInfoByEncoding()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function CreateTextToUnicodeInfoByEncoding( iEncoding: TextEncoding; var oTextToUnicodeInfo: TextToUnicodeInfo ): OSStatus; external name '_CreateTextToUnicodeInfoByEncoding';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function CreateTextToUnicodeInfoByEncoding(iEncoding: TextEncoding; var oTextToUnicodeInfo: TextToUnicodeInfo): OSStatus; external name '_CreateTextToUnicodeInfoByEncoding';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CreateTextToUnicodeInfoByEncoding( iEncoding: TextEncoding; var oTextToUnicodeInfo: TextToUnicodeInfo ): OSStatus; external name '_CreateTextToUnicodeInfoByEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CreateUnicodeToTextInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function CreateUnicodeToTextInfo( iUnicodeMapping: ConstUnicodeMappingPtr; var oUnicodeToTextInfo: UnicodeToTextInfo ): OSStatus; external name '_CreateUnicodeToTextInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function CreateUnicodeToTextInfo(iUnicodeMapping: ConstUnicodeMappingPtr; var oUnicodeToTextInfo: UnicodeToTextInfo): OSStatus; external name '_CreateUnicodeToTextInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CreateUnicodeToTextInfo( iUnicodeMapping: ConstUnicodeMappingPtr; var oUnicodeToTextInfo: UnicodeToTextInfo ): OSStatus; external name '_CreateUnicodeToTextInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CreateUnicodeToTextInfoByEncoding()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function CreateUnicodeToTextInfoByEncoding( iEncoding: TextEncoding; var oUnicodeToTextInfo: UnicodeToTextInfo ): OSStatus; external name '_CreateUnicodeToTextInfoByEncoding';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function CreateUnicodeToTextInfoByEncoding(iEncoding: TextEncoding; var oUnicodeToTextInfo: UnicodeToTextInfo): OSStatus; external name '_CreateUnicodeToTextInfoByEncoding';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CreateUnicodeToTextInfoByEncoding( iEncoding: TextEncoding; var oUnicodeToTextInfo: UnicodeToTextInfo ): OSStatus; external name '_CreateUnicodeToTextInfoByEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CreateUnicodeToTextRunInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function CreateUnicodeToTextRunInfo( iNumberOfMappings: ItemCount; {const} iUnicodeMappings: {variable-size-array} UnicodeMappingPtr; var oUnicodeToTextInfo: UnicodeToTextRunInfo ): OSStatus; external name '_CreateUnicodeToTextRunInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function CreateUnicodeToTextRunInfo(iNumberOfMappings: ItemCount; const (*var*) iUnicodeMappings: UnicodeMapping; var oUnicodeToTextInfo: UnicodeToTextRunInfo): OSStatus; external name '_CreateUnicodeToTextRunInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CreateUnicodeToTextRunInfo( iNumberOfMappings: ItemCount; {const} iUnicodeMappings: {variable-size-array} UnicodeMappingPtr; var oUnicodeToTextInfo: UnicodeToTextRunInfo ): OSStatus; external name '_CreateUnicodeToTextRunInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CreateUnicodeToTextRunInfoByEncoding()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function CreateUnicodeToTextRunInfoByEncoding( iNumberOfEncodings: ItemCount; {const} iEncodings: {variable-size-array} TextEncodingPtr; var oUnicodeToTextInfo: UnicodeToTextRunInfo ): OSStatus; external name '_CreateUnicodeToTextRunInfoByEncoding';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function CreateUnicodeToTextRunInfoByEncoding(iNumberOfEncodings: ItemCount; iEncodings: TextEncodingPtr; var oUnicodeToTextInfo: UnicodeToTextRunInfo): OSStatus; external name '_CreateUnicodeToTextRunInfoByEncoding';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CreateUnicodeToTextRunInfoByEncoding( iNumberOfEncodings: ItemCount; {const} iEncodings: {variable-size-array} TextEncodingPtr; var oUnicodeToTextInfo: UnicodeToTextRunInfo ): OSStatus; external name '_CreateUnicodeToTextRunInfoByEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CreateUnicodeToTextRunInfoByScriptCode()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function CreateUnicodeToTextRunInfoByScriptCode( iNumberOfScriptCodes: ItemCount; {const} iScripts: {variable-size-array} ScriptCodePtr; var oUnicodeToTextInfo: UnicodeToTextRunInfo ): OSStatus; external name '_CreateUnicodeToTextRunInfoByScriptCode';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function CreateUnicodeToTextRunInfoByScriptCode(iNumberOfScriptCodes: ItemCount; iScripts: ScriptCodePtr; var oUnicodeToTextInfo: UnicodeToTextRunInfo): OSStatus; external name '_CreateUnicodeToTextRunInfoByScriptCode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CreateUnicodeToTextRunInfoByScriptCode( iNumberOfScriptCodes: ItemCount; {const} iScripts: {variable-size-array} ScriptCodePtr; var oUnicodeToTextInfo: UnicodeToTextRunInfo ): OSStatus; external name '_CreateUnicodeToTextRunInfoByScriptCode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Change the TextToUnicodeInfo to another mapping. }
{
 *  ChangeTextToUnicodeInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ChangeTextToUnicodeInfo( ioTextToUnicodeInfo: TextToUnicodeInfo; iUnicodeMapping: ConstUnicodeMappingPtr ): OSStatus; external name '_ChangeTextToUnicodeInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ChangeTextToUnicodeInfo(ioTextToUnicodeInfo: TextToUnicodeInfo; iUnicodeMapping: ConstUnicodeMappingPtr): OSStatus; external name '_ChangeTextToUnicodeInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ChangeTextToUnicodeInfo( ioTextToUnicodeInfo: TextToUnicodeInfo; iUnicodeMapping: ConstUnicodeMappingPtr ): OSStatus; external name '_ChangeTextToUnicodeInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Change the UnicodeToTextInfo to another mapping. }
{
 *  ChangeUnicodeToTextInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ChangeUnicodeToTextInfo( ioUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeMapping: ConstUnicodeMappingPtr ): OSStatus; external name '_ChangeUnicodeToTextInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ChangeUnicodeToTextInfo(ioUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeMapping: ConstUnicodeMappingPtr): OSStatus; external name '_ChangeUnicodeToTextInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ChangeUnicodeToTextInfo( ioUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeMapping: ConstUnicodeMappingPtr ): OSStatus; external name '_ChangeUnicodeToTextInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  DisposeTextToUnicodeInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function DisposeTextToUnicodeInfo( var ioTextToUnicodeInfo: TextToUnicodeInfo ): OSStatus; external name '_DisposeTextToUnicodeInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function DisposeTextToUnicodeInfo(var ioTextToUnicodeInfo: TextToUnicodeInfo): OSStatus; external name '_DisposeTextToUnicodeInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function DisposeTextToUnicodeInfo( var ioTextToUnicodeInfo: TextToUnicodeInfo ): OSStatus; external name '_DisposeTextToUnicodeInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  DisposeUnicodeToTextInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function DisposeUnicodeToTextInfo( var ioUnicodeToTextInfo: UnicodeToTextInfo ): OSStatus; external name '_DisposeUnicodeToTextInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function DisposeUnicodeToTextInfo(var ioUnicodeToTextInfo: UnicodeToTextInfo): OSStatus; external name '_DisposeUnicodeToTextInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function DisposeUnicodeToTextInfo( var ioUnicodeToTextInfo: UnicodeToTextInfo ): OSStatus; external name '_DisposeUnicodeToTextInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  DisposeUnicodeToTextRunInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function DisposeUnicodeToTextRunInfo( var ioUnicodeToTextRunInfo: UnicodeToTextRunInfo ): OSStatus; external name '_DisposeUnicodeToTextRunInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function DisposeUnicodeToTextRunInfo(var ioUnicodeToTextRunInfo: UnicodeToTextRunInfo): OSStatus; external name '_DisposeUnicodeToTextRunInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function DisposeUnicodeToTextRunInfo( var ioUnicodeToTextRunInfo: UnicodeToTextRunInfo ): OSStatus; external name '_DisposeUnicodeToTextRunInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ConvertFromTextToUnicode()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ConvertFromTextToUnicode( iTextToUnicodeInfo: TextToUnicodeInfo; iSourceLen: ByteCount; iSourceStr: ConstLogicalAddress; iControlFlags: OptionBits; iOffsetCount: ItemCount; {const} iOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; oOffsetCount: ItemCountPtr { can be NULL }; oOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; iOutputBufLen: ByteCount; var oSourceRead: ByteCount; var oUnicodeLen: ByteCount; oUnicodeStr: {variable-size-array} UniCharPtr ): OSStatus; external name '_ConvertFromTextToUnicode';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ConvertFromTextToUnicode(iTextToUnicodeInfo: TextToUnicodeInfo; iSourceLen: ByteCount; iSourceStr: ConstLogicalAddress; iControlFlags: OptionBits; iOffsetCount: ItemCount; iOffsetArray: ByteOffsetPtr; oOffsetCount: ItemCountPtr; oOffsetArray: ByteOffsetPtr; iOutputBufLen: ByteCount; var oSourceRead: ByteCount; var oUnicodeLen: ByteCount; oUnicodeStr: UniCharPtr): OSStatus; external name '_ConvertFromTextToUnicode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ConvertFromTextToUnicode( iTextToUnicodeInfo: TextToUnicodeInfo; iSourceLen: ByteCount; iSourceStr: ConstLogicalAddress; iControlFlags: OptionBits; iOffsetCount: ItemCount; {const} iOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; oOffsetCount: ItemCountPtr { can be NULL }; oOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; iOutputBufLen: ByteCount; var oSourceRead: ByteCount; var oUnicodeLen: ByteCount; oUnicodeStr: {variable-size-array} UniCharPtr ): OSStatus; external name '_ConvertFromTextToUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ConvertFromUnicodeToText()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ConvertFromUnicodeToText( iUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeLen: ByteCount; {const} iUnicodeStr: {variable-size-array} UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; {const} iOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; oOffsetCount: ItemCountPtr { can be NULL }; oOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress ): OSStatus; external name '_ConvertFromUnicodeToText';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ConvertFromUnicodeToText(iUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeLen: ByteCount; iUnicodeStr: UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; iOffsetArray: ByteOffsetPtr; oOffsetCount: ItemCountPtr; oOffsetArray: ByteOffsetPtr; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress): OSStatus; external name '_ConvertFromUnicodeToText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ConvertFromUnicodeToText( iUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeLen: ByteCount; {const} iUnicodeStr: {variable-size-array} UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; {const} iOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; oOffsetCount: ItemCountPtr { can be NULL }; oOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress ): OSStatus; external name '_ConvertFromUnicodeToText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ConvertFromUnicodeToTextRun()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ConvertFromUnicodeToTextRun( iUnicodeToTextInfo: UnicodeToTextRunInfo; iUnicodeLen: ByteCount; {const} iUnicodeStr: {variable-size-array} UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; {const} iOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; oOffsetCount: ItemCountPtr { can be NULL }; oOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress; iEncodingRunBufLen: ItemCount; var oEncodingRunOutLen: ItemCount; oEncodingRuns: {variable-size-array} TextEncodingRunPtr ): OSStatus; external name '_ConvertFromUnicodeToTextRun';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ConvertFromUnicodeToTextRun(iUnicodeToTextInfo: UnicodeToTextRunInfo; iUnicodeLen: ByteCount; iUnicodeStr: UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; iOffsetArray: ByteOffsetPtr; oOffsetCount: ItemCountPtr; oOffsetArray: ByteOffsetPtr; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress; iEncodingRunBufLen: ItemCount; var oEncodingRunOutLen: ItemCount; var oEncodingRuns: TextEncodingRun): OSStatus; external name '_ConvertFromUnicodeToTextRun';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ConvertFromUnicodeToTextRun( iUnicodeToTextInfo: UnicodeToTextRunInfo; iUnicodeLen: ByteCount; {const} iUnicodeStr: {variable-size-array} UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; {const} iOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; oOffsetCount: ItemCountPtr { can be NULL }; oOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress; iEncodingRunBufLen: ItemCount; var oEncodingRunOutLen: ItemCount; oEncodingRuns: {variable-size-array} TextEncodingRunPtr ): OSStatus; external name '_ConvertFromUnicodeToTextRun';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  ConvertFromUnicodeToScriptCodeRun()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ConvertFromUnicodeToScriptCodeRun( iUnicodeToTextInfo: UnicodeToTextRunInfo; iUnicodeLen: ByteCount; {const} iUnicodeStr: {variable-size-array} UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; {const} iOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; oOffsetCount: ItemCountPtr { can be NULL }; oOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress; iScriptRunBufLen: ItemCount; var oScriptRunOutLen: ItemCount; oScriptCodeRuns: {variable-size-array} ScriptCodeRunPtr ): OSStatus; external name '_ConvertFromUnicodeToScriptCodeRun';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ConvertFromUnicodeToScriptCodeRun(iUnicodeToTextInfo: UnicodeToTextRunInfo; iUnicodeLen: ByteCount; iUnicodeStr: UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; iOffsetArray: ByteOffsetPtr; oOffsetCount: ItemCountPtr; oOffsetArray: ByteOffsetPtr; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress; iScriptRunBufLen: ItemCount; var oScriptRunOutLen: ItemCount; var oScriptCodeRuns: ScriptCodeRun): OSStatus; external name '_ConvertFromUnicodeToScriptCodeRun';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ConvertFromUnicodeToScriptCodeRun( iUnicodeToTextInfo: UnicodeToTextRunInfo; iUnicodeLen: ByteCount; {const} iUnicodeStr: {variable-size-array} UniCharPtr; iControlFlags: OptionBits; iOffsetCount: ItemCount; {const} iOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; oOffsetCount: ItemCountPtr { can be NULL }; oOffsetArray: {variable-size-array} ByteOffsetPtr { can be NULL }; iOutputBufLen: ByteCount; var oInputRead: ByteCount; var oOutputLen: ByteCount; oOutputStr: LogicalAddress; iScriptRunBufLen: ItemCount; var oScriptRunOutLen: ItemCount; oScriptCodeRuns: {variable-size-array} ScriptCodeRunPtr ): OSStatus; external name '_ConvertFromUnicodeToScriptCodeRun';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Truncate a multibyte string at a safe place. }
{
 *  TruncateForTextToUnicode()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function TruncateForTextToUnicode( iTextToUnicodeInfo: ConstTextToUnicodeInfo; iSourceLen: ByteCount; iSourceStr: ConstLogicalAddress; iMaxLen: ByteCount; var oTruncatedLen: ByteCount ): OSStatus; external name '_TruncateForTextToUnicode';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function TruncateForTextToUnicode(iTextToUnicodeInfo: ConstTextToUnicodeInfo; iSourceLen: ByteCount; iSourceStr: ConstLogicalAddress; iMaxLen: ByteCount; var oTruncatedLen: ByteCount): OSStatus; external name '_TruncateForTextToUnicode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TruncateForTextToUnicode( iTextToUnicodeInfo: ConstTextToUnicodeInfo; iSourceLen: ByteCount; iSourceStr: ConstLogicalAddress; iMaxLen: ByteCount; var oTruncatedLen: ByteCount ): OSStatus; external name '_TruncateForTextToUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Truncate a Unicode string at a safe place. }
{
 *  TruncateForUnicodeToText()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function TruncateForUnicodeToText( iUnicodeToTextInfo: ConstUnicodeToTextInfo; iSourceLen: ByteCount; iSourceStr: ConstUniCharPtr; iControlFlags: OptionBits; iMaxLen: ByteCount; var oTruncatedLen: ByteCount ): OSStatus; external name '_TruncateForUnicodeToText';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function TruncateForUnicodeToText(iUnicodeToTextInfo: ConstUnicodeToTextInfo; iSourceLen: ByteCount; iSourceStr: ConstUniCharPtr; iControlFlags: OptionBits; iMaxLen: ByteCount; var oTruncatedLen: ByteCount): OSStatus; external name '_TruncateForUnicodeToText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TruncateForUnicodeToText( iUnicodeToTextInfo: ConstUnicodeToTextInfo; iSourceLen: ByteCount; iSourceStr: ConstUniCharPtr; iControlFlags: OptionBits; iMaxLen: ByteCount; var oTruncatedLen: ByteCount ): OSStatus; external name '_TruncateForUnicodeToText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Convert a Pascal string to Unicode string. }
{
 *  ConvertFromPStringToUnicode()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ConvertFromPStringToUnicode( iTextToUnicodeInfo: TextToUnicodeInfo; const (*var*) iPascalStr: Str255; iOutputBufLen: ByteCount; var oUnicodeLen: ByteCount; oUnicodeStr: UniCharPtr ): OSStatus; external name '_ConvertFromPStringToUnicode';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ConvertFromPStringToUnicode(iTextToUnicodeInfo: TextToUnicodeInfo; const (*var*) iPascalStr: Str255; iOutputBufLen: ByteCount; var oUnicodeLen: ByteCount; oUnicodeStr: UniCharPtr): OSStatus; external name '_ConvertFromPStringToUnicode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ConvertFromPStringToUnicode( iTextToUnicodeInfo: TextToUnicodeInfo; const (*var*) iPascalStr: Str255; iOutputBufLen: ByteCount; var oUnicodeLen: ByteCount; oUnicodeStr: UniCharPtr ): OSStatus; external name '_ConvertFromPStringToUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Convert a Unicode string to Pascal string. }
{
 *  ConvertFromUnicodeToPString()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ConvertFromUnicodeToPString( iUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeLen: ByteCount; {const} iUnicodeStr: {variable-size-array} UniCharPtr; var oPascalStr: Str255 ): OSStatus; external name '_ConvertFromUnicodeToPString';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ConvertFromUnicodeToPString(iUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeLen: ByteCount; iUnicodeStr: UniCharPtr; var oPascalStr: Str255): OSStatus; external name '_ConvertFromUnicodeToPString';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ConvertFromUnicodeToPString( iUnicodeToTextInfo: UnicodeToTextInfo; iUnicodeLen: ByteCount; {const} iUnicodeStr: {variable-size-array} UniCharPtr; var oPascalStr: Str255 ): OSStatus; external name '_ConvertFromUnicodeToPString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Count the available conversion mappings. }
{
 *  CountUnicodeMappings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function CountUnicodeMappings( iFilter: OptionBits; iFindMapping: ConstUnicodeMappingPtr; var oActualCount: ItemCount ): OSStatus; external name '_CountUnicodeMappings';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function CountUnicodeMappings(iFilter: OptionBits; iFindMapping: ConstUnicodeMappingPtr; var oActualCount: ItemCount): OSStatus; external name '_CountUnicodeMappings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CountUnicodeMappings( iFilter: OptionBits; iFindMapping: ConstUnicodeMappingPtr; var oActualCount: ItemCount ): OSStatus; external name '_CountUnicodeMappings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Get a list of the available conversion mappings. }
{
 *  QueryUnicodeMappings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function QueryUnicodeMappings( iFilter: OptionBits; iFindMapping: ConstUnicodeMappingPtr; iMaxCount: ItemCount; var oActualCount: ItemCount; oReturnedMappings: {variable-size-array} UnicodeMappingPtr ): OSStatus; external name '_QueryUnicodeMappings';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function QueryUnicodeMappings(iFilter: OptionBits; iFindMapping: ConstUnicodeMappingPtr; iMaxCount: ItemCount; var oActualCount: ItemCount; var oReturnedMappings: UnicodeMapping): OSStatus; external name '_QueryUnicodeMappings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QueryUnicodeMappings( iFilter: OptionBits; iFindMapping: ConstUnicodeMappingPtr; iMaxCount: ItemCount; var oActualCount: ItemCount; oReturnedMappings: {variable-size-array} UnicodeMappingPtr ): OSStatus; external name '_QueryUnicodeMappings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Setup the fallback handler for converting Unicode To Text. }
{
 *  SetFallbackUnicodeToText()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function SetFallbackUnicodeToText( iUnicodeToTextInfo: UnicodeToTextInfo; iFallback: UnicodeToTextFallbackUPP; iControlFlags: OptionBits; iInfoPtr: LogicalAddress ): OSStatus; external name '_SetFallbackUnicodeToText';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function SetFallbackUnicodeToText(iUnicodeToTextInfo: UnicodeToTextInfo; iFallback: UnicodeToTextFallbackUPP; iControlFlags: OptionBits; iInfoPtr: LogicalAddress): OSStatus; external name '_SetFallbackUnicodeToText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SetFallbackUnicodeToText( iUnicodeToTextInfo: UnicodeToTextInfo; iFallback: UnicodeToTextFallbackUPP; iControlFlags: OptionBits; iInfoPtr: LogicalAddress ): OSStatus; external name '_SetFallbackUnicodeToText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Setup the fallback handler for converting Unicode To TextRuns. }
{
 *  SetFallbackUnicodeToTextRun()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function SetFallbackUnicodeToTextRun( iUnicodeToTextRunInfo: UnicodeToTextRunInfo; iFallback: UnicodeToTextFallbackUPP; iControlFlags: OptionBits; iInfoPtr: LogicalAddress ): OSStatus; external name '_SetFallbackUnicodeToTextRun';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function SetFallbackUnicodeToTextRun(iUnicodeToTextRunInfo: UnicodeToTextRunInfo; iFallback: UnicodeToTextFallbackUPP; iControlFlags: OptionBits; iInfoPtr: LogicalAddress): OSStatus; external name '_SetFallbackUnicodeToTextRun';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function SetFallbackUnicodeToTextRun( iUnicodeToTextRunInfo: UnicodeToTextRunInfo; iFallback: UnicodeToTextFallbackUPP; iControlFlags: OptionBits; iInfoPtr: LogicalAddress ): OSStatus; external name '_SetFallbackUnicodeToTextRun';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Re-initialize all state information kept by the context objects. }
{
 *  ResetTextToUnicodeInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.3 and later
 }
function ResetTextToUnicodeInfo( ioTextToUnicodeInfo: TextToUnicodeInfo ): OSStatus; external name '_ResetTextToUnicodeInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.3 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.3 and later
 }
<<<<<<< HEAD
function ResetTextToUnicodeInfo(ioTextToUnicodeInfo: TextToUnicodeInfo): OSStatus; external name '_ResetTextToUnicodeInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ResetTextToUnicodeInfo( ioTextToUnicodeInfo: TextToUnicodeInfo ): OSStatus; external name '_ResetTextToUnicodeInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Re-initialize all state information kept by the context objects. }
{
 *  ResetUnicodeToTextInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ResetUnicodeToTextInfo( ioUnicodeToTextInfo: UnicodeToTextInfo ): OSStatus; external name '_ResetUnicodeToTextInfo';
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
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
<<<<<<< HEAD
function ResetUnicodeToTextInfo(ioUnicodeToTextInfo: UnicodeToTextInfo): OSStatus; external name '_ResetUnicodeToTextInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ResetUnicodeToTextInfo( ioUnicodeToTextInfo: UnicodeToTextInfo ): OSStatus; external name '_ResetUnicodeToTextInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Re-initialize all state information kept by the context objects in TextRun conversions. }
{
 *  ResetUnicodeToTextRunInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ResetUnicodeToTextRunInfo( ioUnicodeToTextRunInfo: UnicodeToTextRunInfo ): OSStatus; external name '_ResetUnicodeToTextRunInfo';
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
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in UnicodeConverter 1.1 and later
 }
function ResetUnicodeToTextRunInfo( ioUnicodeToTextRunInfo: UnicodeToTextRunInfo ): OSStatus; external name '_ResetUnicodeToTextRunInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
