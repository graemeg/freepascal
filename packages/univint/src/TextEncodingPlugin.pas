{
<<<<<<< HEAD
     File:       CarbonCore/TextEncodingPlugin.h
 
     Contains:   Required interface for Text Encoding Converter-Plugins
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  � 1996-2011 by Apple Inc. All rights reserved.
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  � 1996-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  � 1996-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  � 1996-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    CarbonCore-859.2~1
 
     Copyright:  � 1996-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
     File:       TextEncodingPlugin.p
 
     Contains:   Required interface for Text Encoding Converter-Plugins
 
     Version:    Technology: Mac OS 8
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  � 1996-2002 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
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
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/cpstrnew
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
                     http://www.freepascal.org/bugs.html
 
}


{
    Modified for use with Free Pascal
    Version 210
    Please report any bugs to <gpc@microbizz.nl>
}

>>>>>>> graemeg/fixes_2_2
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit TextEncodingPlugin;
interface
<<<<<<< HEAD
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> graemeg/fixes_2_2

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
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
=======
>>>>>>> graemeg/fixes_2_2
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/fixes_2_2

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
>>>>>>> graemeg/cpstrnew
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
<<<<<<< HEAD
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
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
=======
{$elsec}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
	{$setc TARGET_CPU_X86 := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
{$elsec}
	{$error Neither __ppc__ nor __i386__ is defined.}
{$endc}
{$setc TARGET_CPU_PPC_64 := FALSE}
>>>>>>> graemeg/fixes_2_2

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
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,TextCommon,TextEncodingConverter;
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/fixes_2_2

{$ALIGN MAC68K}

{
  ####################################################################################
        Constants
  ####################################################################################
}
{
<<<<<<< HEAD
   This constant is needed for MacOS X development only. It is the name in which the
   function to grab the plugin's dispatch table must go by. 
}
const
	kTECMacOSXDispatchTableNameString = 'ConverterPluginGetPluginDispatchTable';
{ These constant are needed for TEC plugins.}
const
	kTECAvailableEncodingsResType = FourCharCode('cven');
	kTECAvailableSniffersResType = FourCharCode('cvsf');
	kTECSubTextEncodingsResType = FourCharCode('cvsb');
	kTECConversionInfoResType = FourCharCode('cvif');
	kTECMailEncodingsResType = FourCharCode('cvml');
	kTECWebEncodingsResType = FourCharCode('cvwb');
	kTECInternetNamesResType = FourCharCode('cvmm');

const
	kTECPluginType = FourCharCode('ecpg');
	kTECPluginCreator = FourCharCode('encv');
	kTECPluginOneToOne = FourCharCode('otoo');
	kTECPluginOneToMany = FourCharCode('otom');
	kTECPluginManyToOne = FourCharCode('mtoo');
	kTECPluginSniffObj = FourCharCode('snif');

const
	verUnspecified = 32767;
	kTECResourceID = 128;

{
=======
>>>>>>> graemeg/fixes_2_2
  ####################################################################################
        Structs
  ####################################################################################
}

<<<<<<< HEAD
{ These structs are needed for TEC plugins.}

type
	TextEncodingRecPtr = ^TextEncodingRec;
	TextEncodingRec = record
		base: UInt32;
		variant: UInt32;
		format: UInt32;
	end;
{ supported encodings & sniffers lists, type TECEncodingsListRec }
type
	TECEncodingsListRecPtr = ^TECEncodingsListRec;
	TECEncodingsListRec = record
		count: UInt32;
		encodings: TextEncodingRec;              { first of many}
	end;
type
	TECEncodingsListPtr = TECEncodingsListRecPtr;
	TECEncodingsListHandle = ^TECEncodingsListPtr;
{ sub encodings list - type TECSubTextEncodingsRec }
type
	TECSubTextEncodingRecPtr =^TECSubTextEncodingRec;
	TECSubTextEncodingRec = record
		offset: UInt32;                 { offset to next variable-length record}
		searchEncoding: TextEncodingRec;         { the encoding}
		count: UInt32;
		subEncodings: TextEncodingRec;           { first of many sub encodings for searchEncoding}
	end;
type
	TECSubTextEncodingsRecPtr = ^TECSubTextEncodingsRec;
	TECSubTextEncodingsRec = record
		count: UInt32;
		subTextEncodingRec: TECSubTextEncodingRec;  { first of many}
	end;
type
	TECSubTextEncodingsPtr = TECSubTextEncodingsRecPtr;
	TECSubTextEncodingsHandle = ^TECSubTextEncodingsPtr;
{ conversions pairs list - type TECEncodingPairsRec }
type
	TECEncodingPairRecPtr = ^TECEncodingPairRec;
	TECEncodingPairRec = record
		source: TextEncodingRec;
		dest: TextEncodingRec;
	end;
type
	TECEncodingPairs = record
		encodingPair: TECEncodingPairRec;
		flags: UInt32;                  { 'flags' name is not really used yet (JKC 9/5/97)}
		speed: UInt32;                  { 'speed' name is not really used yet (JKC 9/5/97)}
	end;
type
	TECEncodingPairsRecPtr =^TECEncodingPairsRec;
	TECEncodingPairsRec = record
		count: UInt32;
		encodingPairs: TECEncodingPairs;
	end;
type
	TECEncodingPairsPtr = TECEncodingPairsRecPtr;
	TECEncodingPairsHandle = ^TECEncodingPairsPtr;
{ mail & web encodings lists - type TECLocaleToEncodingsListRec }
type
	TECLocaleListToEncodingListRecPtr = ^TECLocaleListToEncodingListRec;
	TECLocaleListToEncodingListRec = record
		offset: UInt32;                 { offset to next variable-length record}
		count: UInt32;
		locales: RegionCode;                { first in list of locales}
                                              { TECEncodingListRec encodingList;     // after local variable length array}
	end;
type
	TECLocaleListToEncodingListPtr = TECLocaleListToEncodingListRecPtr;
	TECLocaleToEncodingsListRecPtr = ^TECLocaleToEncodingsListRec;
	TECLocaleToEncodingsListRec = record
		count: UInt32;
		localeListToEncodingList: TECLocaleListToEncodingListRec; { language of name}
	end;
type
	TECLocaleToEncodingsListPtr = TECLocaleToEncodingsListRecPtr;
	TECLocaleToEncodingsListHandle = ^TECLocaleToEncodingsListPtr;
{ internet names list - type TECInternetNamesRec }
type
	TECInternetNameRecPtr = ^TECInternetNameRec;
	TECInternetNameRec = record
		offset: UInt32;                 { offset to next variable-length record}
		searchEncoding: TextEncodingRec;         { named encoding}
		encodingNameLength: UInt8;
		encodingName: array [0..0] of UInt8;        { first byte of many }
	end;
type
	TECInternetNamesRecPtr = ^TECInternetNamesRec;
	TECInternetNamesRec = record
		count: UInt32;
		InternetNames: TECInternetNameRec;          { first of many}
	end;
type
	TECInternetNamesPtr = TECInternetNamesRecPtr;
	TECInternetNamesHandle = ^TECInternetNamesPtr;
{ plugin context record }
type
	TECBufferContextRecPtr = ^TECBufferContextRec;
	TECBufferContextRec = record
		textInputBuffer: ConstTextPtr;
		textInputBufferEnd: ConstTextPtr;
		textOutputBuffer: TextPtr;
		textOutputBufferEnd: TextPtr;

		encodingInputBuffer: ConstTextEncodingRunPtr;
		encodingInputBufferEnd: ConstTextEncodingRunPtr;
		encodingOutputBuffer: TextEncodingRunPtr;
		encodingOutputBufferEnd: TextEncodingRunPtr;
	end;
type
	TECPluginStateRecPtr = ^TECPluginStateRec;
	TECPluginStateRec = record
		state1: UInt8;
		state2: UInt8;
		state3: UInt8;
		state4: UInt8;

		longState1: UInt32;
		longState2: UInt32;
		longState3: UInt32;
		longState4: UInt32;
	end;
type
	TECConverterContextRecPtr = ^TECConverterContextRec;
	TECConverterContextRec = record
{ public - manipulated externally and by plugin}
		pluginRec: Ptr;
		sourceEncoding: TextEncoding;
		destEncoding: TextEncoding;
		reserved1: UInt32;
		reserved2: UInt32;
		bufferContext: TECBufferContextRec;
                                              { private - manipulated only within Plugin}
		contextRefCon: URefCon;
		conversionProc: ProcPtr;
		flushProc: ProcPtr;
		clearContextInfoProc: ProcPtr;
		options1: UInt32;
		options2: UInt32;
		pluginState: TECPluginStateRec;
	end;
type
	TECSnifferContextRecPtr = ^TECSnifferContextRec;
	TECSnifferContextRec = record
{ public - manipulated externally}
		pluginRec: Ptr;
		encoding: TextEncoding;
		maxErrors: ItemCount;
		maxFeatures: ItemCount;
		textInputBuffer: TextPtr;
		textInputBufferEnd: TextPtr;
		numFeatures: ItemCount;
		numErrors: ItemCount;
                                              { private - manipulated only within Plugin}
		contextRefCon: URefCon;
		sniffProc: ProcPtr;
		clearContextInfoProc: ProcPtr;
		pluginState: TECPluginStateRec;
	end;
{
  ####################################################################################
        Functional Messages
  ####################################################################################
}

type
	TECPluginNewEncodingConverterPtr = function( var newEncodingConverter: TECObjectRef; var plugContext: TECConverterContextRec; inputEncoding: TextEncoding; outputEncoding: TextEncoding ): OSStatus;
	TECPluginClearContextInfoPtr = function( encodingConverter: TECObjectRef; var plugContext: TECConverterContextRec ): OSStatus;
	TECPluginConvertTextEncodingPtr = function( encodingConverter: TECObjectRef; var plugContext: TECConverterContextRec ): OSStatus;
	TECPluginFlushConversionPtr = function( encodingConverter: TECObjectRef; var plugContext: TECConverterContextRec ): OSStatus;
	TECPluginDisposeEncodingConverterPtr = function( newEncodingConverter: TECObjectRef; var plugContext: TECConverterContextRec ): OSStatus;
	TECPluginNewEncodingSnifferPtr = function( var encodingSniffer: TECSnifferObjectRef; var snifContext: TECSnifferContextRec; inputEncoding: TextEncoding ): OSStatus;
	TECPluginClearSnifferContextInfoPtr = function( encodingSniffer: TECSnifferObjectRef; var snifContext: TECSnifferContextRec ): OSStatus;
	TECPluginSniffTextEncodingPtr = function( encodingSniffer: TECSnifferObjectRef; var snifContext: TECSnifferContextRec ): OSStatus;
	TECPluginDisposeEncodingSnifferPtr = function( encodingSniffer: TECSnifferObjectRef; var snifContext: TECSnifferContextRec ): OSStatus;
	TECPluginGetCountAvailableTextEncodingsPtr = function( availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus;
	TECPluginGetCountAvailableTextEncodingPairsPtr = function( availableEncodings: TECConversionInfoPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus;
	TECPluginGetCountDestinationTextEncodingsPtr = function( inputEncoding: TextEncoding; destinationEncodings: TextEncodingPtr; maxDestinationEncodings: ItemCount; var actualDestinationEncodings: ItemCount ): OSStatus;
	TECPluginGetCountSubTextEncodingsPtr = function( inputEncoding: TextEncoding; subEncodings: TextEncodingPtr; maxSubEncodings: ItemCount; var actualSubEncodings: ItemCount ): OSStatus;
	TECPluginGetCountAvailableSniffersPtr = function( availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus;
	TECPluginGetTextEncodingInternetNamePtr = function( textEncoding_: TextEncoding; var encodingName: Str255 ): OSStatus;
	TECPluginGetTextEncodingFromInternetNamePtr = function( var textEncoding_: TextEncoding; encodingName: Str255 ): OSStatus;
	TECPluginGetCountWebEncodingsPtr = function( availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus;
	TECPluginGetCountMailEncodingsPtr = function( availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount ): OSStatus;
{
  ####################################################################################
        Dispatch Table Definition
  ####################################################################################
}

const
	kTECPluginDispatchTableVersion1 = $00010000; { 1.0 through 1.0.3 releases}
	kTECPluginDispatchTableVersion1_1 = $00010001; { 1.1 releases}
	kTECPluginDispatchTableVersion1_2 = $00010002; { 1.2 releases}
	kTECPluginDispatchTableCurrentVersion = kTECPluginDispatchTableVersion1_2;
=======
type
	TECBufferContextRecPtr = ^TECBufferContextRec;
	TECBufferContextRec = record
		textInputBuffer:		TextPtr;
		textInputBufferEnd:		TextPtr;
		textOutputBuffer:		TextPtr;
		textOutputBufferEnd:	TextPtr;
		encodingInputBuffer:	TextEncodingRunPtr;
		encodingInputBufferEnd:	TextEncodingRunPtr;
		encodingOutputBuffer:	TextEncodingRunPtr;
		encodingOutputBufferEnd: TextEncodingRunPtr;
	end;

	TECPluginStateRecPtr = ^TECPluginStateRec;
	TECPluginStateRec = record
		state1:					SInt8;
		state2:					SInt8;
		state3:					SInt8;
		state4:					SInt8;
		longState1:				UInt32;
		longState2:				UInt32;
		longState3:				UInt32;
		longState4:				UInt32;
	end;

	TECConverterContextRecPtr = ^TECConverterContextRec;
	TECConverterContextRec = record
																		{  public - manipulated externally and by plugin }
		pluginRec:				Ptr;
		sourceEncoding:			TextEncoding;
		destEncoding:			TextEncoding;
		reserved1:				UInt32;
		reserved2:				UInt32;
		bufferContext:			TECBufferContextRec;
																		{  private - manipulated only within Plugin }
		contextRefCon:			UInt32;
		conversionProc:			ProcPtr;
		flushProc:				ProcPtr;
		clearContextInfoProc:	ProcPtr;
		options1:				UInt32;
		options2:				UInt32;
		pluginState:			TECPluginStateRec;
	end;

	TECSnifferContextRecPtr = ^TECSnifferContextRec;
	TECSnifferContextRec = record
																		{  public - manipulated externally }
		pluginRec:				Ptr;
		encoding:				TextEncoding;
		maxErrors:				ItemCount;
		maxFeatures:			ItemCount;
		textInputBuffer:		TextPtr;
		textInputBufferEnd:		TextPtr;
		numFeatures:			ItemCount;
		numErrors:				ItemCount;
																		{  private - manipulated only within Plugin }
		contextRefCon:			UInt32;
		sniffProc:				ProcPtr;
		clearContextInfoProc:	ProcPtr;
		pluginState:			TECPluginStateRec;
	end;

	{
	  ####################################################################################
	        Functional Messages
	  ####################################################################################
	}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginNewEncodingConverterPtr = function(var newEncodingConverter: TECObjectRef; var plugContext: TECConverterContextRec; inputEncoding: TextEncoding; outputEncoding: TextEncoding): OSStatus;
{$elsec}
	TECPluginNewEncodingConverterPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginClearContextInfoPtr = function(encodingConverter: TECObjectRef; var plugContext: TECConverterContextRec): OSStatus;
{$elsec}
	TECPluginClearContextInfoPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginConvertTextEncodingPtr = function(encodingConverter: TECObjectRef; var plugContext: TECConverterContextRec): OSStatus;
{$elsec}
	TECPluginConvertTextEncodingPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginFlushConversionPtr = function(encodingConverter: TECObjectRef; var plugContext: TECConverterContextRec): OSStatus;
{$elsec}
	TECPluginFlushConversionPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginDisposeEncodingConverterPtr = function(newEncodingConverter: TECObjectRef; var plugContext: TECConverterContextRec): OSStatus;
{$elsec}
	TECPluginDisposeEncodingConverterPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginNewEncodingSnifferPtr = function(var encodingSniffer: TECSnifferObjectRef; var snifContext: TECSnifferContextRec; inputEncoding: TextEncoding): OSStatus;
{$elsec}
	TECPluginNewEncodingSnifferPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginClearSnifferContextInfoPtr = function(encodingSniffer: TECSnifferObjectRef; var snifContext: TECSnifferContextRec): OSStatus;
{$elsec}
	TECPluginClearSnifferContextInfoPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginSniffTextEncodingPtr = function(encodingSniffer: TECSnifferObjectRef; var snifContext: TECSnifferContextRec): OSStatus;
{$elsec}
	TECPluginSniffTextEncodingPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginDisposeEncodingSnifferPtr = function(encodingSniffer: TECSnifferObjectRef; var snifContext: TECSnifferContextRec): OSStatus;
{$elsec}
	TECPluginDisposeEncodingSnifferPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetCountAvailableTextEncodingsPtr = function(availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount): OSStatus;
{$elsec}
	TECPluginGetCountAvailableTextEncodingsPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetCountAvailableTextEncodingPairsPtr = function(availableEncodings: TECConversionInfoPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount): OSStatus;
{$elsec}
	TECPluginGetCountAvailableTextEncodingPairsPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetCountDestinationTextEncodingsPtr = function(inputEncoding: TextEncoding; destinationEncodings: TextEncodingPtr; maxDestinationEncodings: ItemCount; var actualDestinationEncodings: ItemCount): OSStatus;
{$elsec}
	TECPluginGetCountDestinationTextEncodingsPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetCountSubTextEncodingsPtr = function(inputEncoding: TextEncoding; subEncodings: TextEncodingPtr; maxSubEncodings: ItemCount; var actualSubEncodings: ItemCount): OSStatus;
{$elsec}
	TECPluginGetCountSubTextEncodingsPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetCountAvailableSniffersPtr = function(availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount): OSStatus;
{$elsec}
	TECPluginGetCountAvailableSniffersPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetTextEncodingInternetNamePtr = function(textEncoding_: TextEncoding; var encodingName: Str255): OSStatus;
{$elsec}
	TECPluginGetTextEncodingInternetNamePtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetTextEncodingFromInternetNamePtr = function(var textEncoding_: TextEncoding; encodingName: Str255): OSStatus;
{$elsec}
	TECPluginGetTextEncodingFromInternetNamePtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetCountWebEncodingsPtr = function(availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount): OSStatus;
{$elsec}
	TECPluginGetCountWebEncodingsPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	TECPluginGetCountMailEncodingsPtr = function(availableEncodings: TextEncodingPtr; maxAvailableEncodings: ItemCount; var actualAvailableEncodings: ItemCount): OSStatus;
{$elsec}
	TECPluginGetCountMailEncodingsPtr = ProcPtr;
{$endc}

	{
	  ####################################################################################
	        Dispatch Table Definition
	  ####################################################################################
	}


const
	kTECPluginDispatchTableVersion1 = $00010000;				{  1.0 through 1.0.3 releases }
	kTECPluginDispatchTableVersion1_1 = $00010001;				{  1.1 releases }
	kTECPluginDispatchTableVersion1_2 = $00010002;				{  1.2 releases }
	kTECPluginDispatchTableCurrentVersion = $00010002;

>>>>>>> graemeg/fixes_2_2

type
	TECPluginDispatchTablePtr = ^TECPluginDispatchTable;
	TECPluginDispatchTable = record
<<<<<<< HEAD
		version: TECPluginVersion;
		compatibleVersion: TECPluginVersion;
		PluginID: TECPluginSignature;

		PluginNewEncodingConverter: TECPluginNewEncodingConverterPtr;
		PluginClearContextInfo: TECPluginClearContextInfoPtr;
		PluginConvertTextEncoding: TECPluginConvertTextEncodingPtr;
		PluginFlushConversion: TECPluginFlushConversionPtr;
		PluginDisposeEncodingConverter: TECPluginDisposeEncodingConverterPtr;

=======
		version:				TECPluginVersion;
		compatibleVersion:		TECPluginVersion;
		PluginID:				TECPluginSignature;
		PluginNewEncodingConverter: TECPluginNewEncodingConverterPtr;
		PluginClearContextInfo:	TECPluginClearContextInfoPtr;
		PluginConvertTextEncoding: TECPluginConvertTextEncodingPtr;
		PluginFlushConversion:	TECPluginFlushConversionPtr;
		PluginDisposeEncodingConverter: TECPluginDisposeEncodingConverterPtr;
>>>>>>> graemeg/fixes_2_2
		PluginNewEncodingSniffer: TECPluginNewEncodingSnifferPtr;
		PluginClearSnifferContextInfo: TECPluginClearSnifferContextInfoPtr;
		PluginSniffTextEncoding: TECPluginSniffTextEncodingPtr;
		PluginDisposeEncodingSniffer: TECPluginDisposeEncodingSnifferPtr;
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
		PluginGetCountAvailableTextEncodings: TECPluginGetCountAvailableTextEncodingsPtr;
		PluginGetCountAvailableTextEncodingPairs: TECPluginGetCountAvailableTextEncodingPairsPtr;
		PluginGetCountDestinationTextEncodings: TECPluginGetCountDestinationTextEncodingsPtr;
		PluginGetCountSubTextEncodings: TECPluginGetCountSubTextEncodingsPtr;
		PluginGetCountAvailableSniffers: TECPluginGetCountAvailableSniffersPtr;
		PluginGetCountWebTextEncodings: TECPluginGetCountWebEncodingsPtr;
		PluginGetCountMailTextEncodings: TECPluginGetCountMailEncodingsPtr;
<<<<<<< HEAD

		PluginGetTextEncodingInternetName: TECPluginGetTextEncodingInternetNamePtr;
		PluginGetTextEncodingFromInternetName: TECPluginGetTextEncodingFromInternetNamePtr;
	end;
{
   The last prototype here is for MacOS X plugins only. TEC Plugins in MacOS X need to export a
   a function called ConverterPluginGetPluginDispatchTable with the following prototype:
   extern TECPluginDispatchTable *ConverterPluginGetPluginDispatchTable( void )
   This function will need to return a pointer to the plugin's function dispatch table 
   when called. It is important that the function be called exactly 
   "ConverterPluginGetPluginDispatchTable". TECPluginGetPluginDispatchTablePtr is a 
   function pointer to this function.
}
type
	TECPluginGetPluginDispatchTablePtr = function: TECPluginDispatchTablePtr;

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
		PluginGetTextEncodingInternetName: TECPluginGetTextEncodingInternetNamePtr;
		PluginGetTextEncodingFromInternetName: TECPluginGetTextEncodingFromInternetNamePtr;
	end;


{$ALIGN MAC68K}


end.
>>>>>>> graemeg/fixes_2_2
