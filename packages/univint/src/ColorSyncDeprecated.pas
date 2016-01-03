{ ColorSync- ColorSyncDeprecated.h
 * Copyright (c)  2008 Apple Inc.
 * All rights reserved.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/cpstrnew
=======
{       Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2007 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/fixes_2.4
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

unit ColorSyncDeprecated;
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> origin/fixes_2.4
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
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
uses MacTypes,Files,QDCMCommon,CFBase,CFData,CFDictionary;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

//#pragma mark --- CMBase.h ---

//#pragma mark --- CMICCProfile.h ----

{$ALIGN MAC68K}

{ ICC Profile version constants  }
const
	cmICCProfileVersion4 = $04000000;
	cmICCProfileVersion2 = $02000000;
	cmICCProfileVersion21 = $02100000;
	cmCS2ProfileVersion = cmICCProfileVersion2;
	cmCS1ProfileVersion = $00000100; { ColorSync 1.0 profile version }

{ Current Major version number }
const
	cmProfileMajorVersionMask = $FF000000;
	cmCurrentProfileMajorVersion = $02000000;

{ magic cookie number for anonymous file ID }
const
	cmMagicNumber = FourCharCode('acsp');


{**********************************************************************}
{************** ColorSync 2.0 profile specification *******************}
{**********************************************************************}
{*** flags field  ***}
const
	cmICCReservedFlagsMask = $0000FFFF; { these bits of the flags field are defined and reserved by ICC }
	cmEmbeddedMask = $00000001; { if bit 0 is 0 then not embedded profile, if 1 then embedded profile }
	cmEmbeddedUseMask = $00000002; { if bit 1 is 0 then ok to use anywhere, if 1 then ok to use as embedded profile only }
	cmBlackPointCompensationMask = $00000004; { if bit 2 is 1 then CMM will enable Black Point Compensation if applicable}
	cmCMSReservedFlagsMask = $FFFF0000; { these bits of the flags field are defined and reserved by CMS vendor }
	cmQualityMask = $00030000; { if bits 16-17 is 0 then normal, if 1 then draft, if 2 then best }
	cmInterpolationMask = $00040000; { if bit 18 is 0 then interpolation, if 1 then lookup only }
	cmGamutCheckingMask = $00080000; { if bit 19 is 0 then create gamut checking info, if 1 then no gamut checking info }

{ copyright-protection flag options }
const
	cmEmbeddedProfile = 0;    { 0 is not embedded profile, 1 is embedded profile }
	cmEmbeddedUse = 1;     { 0 is to use anywhere, 1 is to use as embedded profile only }

{ speed and quality flag options }
const
	cmNormalMode = 0;    { it uses the least significent two bits in the high word of flag }
	cmDraftMode = 1;    { it should be evaulated like this: right shift 16 bits first, mask off the }
	cmBestMode = 2;     { high 14 bits, and then compare with the enum to determine the option value }

{ black point compensation flag option }
const
	cmBlackPointCompensation = 1;     { 0 do not apply Black Point Compensation, 1 apply }


{*** deviceAttributes fields ***}
{ deviceAttributes[0] is defined by and reserved for device vendors }
{ deviceAttributes[1] is defined by and reserved for ICC }
{ The following bits of deviceAttributes[1] are currently defined }
const
	cmReflectiveTransparentMask = $00000001; { if bit 0 is 0 then reflective media, if 1 then transparency media }
	cmGlossyMatteMask = $00000002; { if bit 1 is 0 then glossy, if 1 then matte }

{ device/media attributes element values  }
const
	cmReflective = 0;    { if bit 0 is 0 then reflective media, if 1 then transparency media }
	cmGlossy = 1;     { if bit 1 is 0 then glossy, if 1 then matte }


{*** renderingIntent field ***}
const
	cmPerceptual = 0;    { Photographic images }
	cmRelativeColorimetric = 1;    { Logo Colors }
	cmSaturation = 2;    { Business graphics }
	cmAbsoluteColorimetric = 3;     { Logo Colors }


{ data type element values }
const
	cmAsciiData = 0;
	cmBinaryData = 1;

{ screen encodings  }
const
	cmPrtrDefaultScreens = 0;    { Use printer default screens.  0 is false, 1 is ture }
	cmLinesPer = 1;     { 0 is LinesPerCm, 1 is LinesPerInch }

{ 2.0 tag type information }
const
	cmNumHeaderElements = 10;

{ public tags }
const
	cmAToB0Tag = FourCharCode('A2B0');
	cmAToB1Tag = FourCharCode('A2B1');
	cmAToB2Tag = FourCharCode('A2B2');
	cmBlueColorantTag = FourCharCode('bXYZ');
	cmBlueTRCTag = FourCharCode('bTRC');
	cmBToA0Tag = FourCharCode('B2A0');
	cmBToA1Tag = FourCharCode('B2A1');
	cmBToA2Tag = FourCharCode('B2A2');
	cmCalibrationDateTimeTag = FourCharCode('calt');
	cmChromaticAdaptationTag = FourCharCode('chad');
	cmCharTargetTag = FourCharCode('targ');
	cmCopyrightTag = FourCharCode('cprt');
	cmDeviceMfgDescTag = FourCharCode('dmnd');
	cmDeviceModelDescTag = FourCharCode('dmdd');
	cmGamutTag = FourCharCode('gamt');
	cmGrayTRCTag = FourCharCode('kTRC');
	cmGreenColorantTag = FourCharCode('gXYZ');
	cmGreenTRCTag = FourCharCode('gTRC');
	cmLuminanceTag = FourCharCode('lumi');
	cmMeasurementTag = FourCharCode('meas');
	cmMediaBlackPointTag = FourCharCode('bkpt');
	cmMediaWhitePointTag = FourCharCode('wtpt');
	cmNamedColorTag = FourCharCode('ncol');
	cmNamedColor2Tag = FourCharCode('ncl2');
	cmPreview0Tag = FourCharCode('pre0');
	cmPreview1Tag = FourCharCode('pre1');
	cmPreview2Tag = FourCharCode('pre2');
	cmProfileDescriptionTag = FourCharCode('desc');
	cmProfileSequenceDescTag = FourCharCode('pseq');
	cmPS2CRD0Tag = FourCharCode('psd0');
	cmPS2CRD1Tag = FourCharCode('psd1');
	cmPS2CRD2Tag = FourCharCode('psd2');
	cmPS2CRD3Tag = FourCharCode('psd3');
	cmPS2CSATag = FourCharCode('ps2s');
	cmPS2RenderingIntentTag = FourCharCode('ps2i');
	cmRedColorantTag = FourCharCode('rXYZ');
	cmRedTRCTag = FourCharCode('rTRC');
	cmScreeningDescTag = FourCharCode('scrd');
	cmScreeningTag = FourCharCode('scrn');
	cmTechnologyTag = FourCharCode('tech');
	cmUcrBgTag = FourCharCode('bfd ');
	cmViewingConditionsDescTag = FourCharCode('vued');
	cmViewingConditionsTag = FourCharCode('view');

{ custom tags }
const
	cmPS2CRDVMSizeTag = FourCharCode('psvm');
	cmVideoCardGammaTag = FourCharCode('vcgt');
	cmMakeAndModelTag = FourCharCode('mmod');
	cmProfileDescriptionMLTag = FourCharCode('dscm');
	cmNativeDisplayInfoTag = FourCharCode('ndin');

{ public type signatures }
const
	cmSigCrdInfoType = FourCharCode('crdi');
	cmSigCurveType = FourCharCode('curv');
	cmSigDataType = FourCharCode('data');
	cmSigDateTimeType = FourCharCode('dtim');
	cmSigLut16Type = FourCharCode('mft2');
	cmSigLut8Type = FourCharCode('mft1');
	cmSigMeasurementType = FourCharCode('meas');
	cmSigMultiFunctA2BType = FourCharCode('mAB ');
	cmSigMultiFunctB2AType = FourCharCode('mBA ');
	cmSigNamedColorType = FourCharCode('ncol');
	cmSigNamedColor2Type = FourCharCode('ncl2');
	cmSigParametricCurveType = FourCharCode('para');
	cmSigProfileDescriptionType = FourCharCode('desc');
	cmSigProfileSequenceDescType = FourCharCode('pseq');
	cmSigScreeningType = FourCharCode('scrn');
	cmSigS15Fixed16Type = FourCharCode('sf32');
	cmSigSignatureType = FourCharCode('sig ');
	cmSigTextType = FourCharCode('text');
	cmSigU16Fixed16Type = FourCharCode('uf32');
	cmSigU1Fixed15Type = FourCharCode('uf16');
	cmSigUInt8Type = FourCharCode('ui08');
	cmSigUInt16Type = FourCharCode('ui16');
	cmSigUInt32Type = FourCharCode('ui32');
	cmSigUInt64Type = FourCharCode('ui64');
	cmSigUcrBgType = FourCharCode('bfd ');
	cmSigUnicodeTextType = FourCharCode('utxt');
	cmSigViewingConditionsType = FourCharCode('view');
	cmSigXYZType = FourCharCode('XYZ ');

{ custom type signatures }
const
	cmSigPS2CRDVMSizeType = FourCharCode('psvm');
	cmSigVideoCardGammaType = FourCharCode('vcgt');
	cmSigMakeAndModelType = FourCharCode('mmod');
	cmSigNativeDisplayInfoType = FourCharCode('ndin');
	cmSigMultiLocalizedUniCodeType = FourCharCode('mluc');


{ technology tag descriptions }
const
	cmTechnologyDigitalCamera = FourCharCode('dcam');
	cmTechnologyFilmScanner = FourCharCode('fscn');
	cmTechnologyReflectiveScanner = FourCharCode('rscn');
	cmTechnologyInkJetPrinter = FourCharCode('ijet');
	cmTechnologyThermalWaxPrinter = FourCharCode('twax');
	cmTechnologyElectrophotographicPrinter = FourCharCode('epho');
	cmTechnologyElectrostaticPrinter = FourCharCode('esta');
	cmTechnologyDyeSublimationPrinter = FourCharCode('dsub');
	cmTechnologyPhotographicPaperPrinter = FourCharCode('rpho');
	cmTechnologyFilmWriter = FourCharCode('fprn');
	cmTechnologyVideoMonitor = FourCharCode('vidm');
	cmTechnologyVideoCamera = FourCharCode('vidc');
	cmTechnologyProjectionTelevision = FourCharCode('pjtv');
	cmTechnologyCRTDisplay = FourCharCode('CRT ');
	cmTechnologyPMDisplay = FourCharCode('PMD ');
	cmTechnologyAMDisplay = FourCharCode('AMD ');
	cmTechnologyPhotoCD = FourCharCode('KPCD');
	cmTechnologyPhotoImageSetter = FourCharCode('imgs');
	cmTechnologyGravure = FourCharCode('grav');
	cmTechnologyOffsetLithography = FourCharCode('offs');
	cmTechnologySilkscreen = FourCharCode('silk');
	cmTechnologyFlexography = FourCharCode('flex');


{ Measurement type encodings }
{ Measurement Flare }
const
	cmFlare0 = $00000000;
	cmFlare100 = $00000001;

{ Measurement Geometry }
const
	cmGeometryUnknown = $00000000;
	cmGeometry045or450 = $00000001;
	cmGeometry0dord0 = $00000002;

{ Standard Observer    }
const
	cmStdobsUnknown = $00000000;
	cmStdobs1931TwoDegrees = $00000001;
	cmStdobs1964TenDegrees = $00000002;

{ Standard Illuminant }
const
	cmIlluminantUnknown = $00000000;
	cmIlluminantD50 = $00000001;
	cmIlluminantD65 = $00000002;
	cmIlluminantD93 = $00000003;
	cmIlluminantF2 = $00000004;
	cmIlluminantD55 = $00000005;
	cmIlluminantA = $00000006;
	cmIlluminantEquiPower = $00000007;
	cmIlluminantF8 = $00000008;

{ Spot Function Value }
const
	cmSpotFunctionUnknown = 0;
	cmSpotFunctionDefault = 1;
	cmSpotFunctionRound = 2;
	cmSpotFunctionDiamond = 3;
	cmSpotFunctionEllipse = 4;
	cmSpotFunctionLine = 5;
	cmSpotFunctionSquare = 6;
	cmSpotFunctionCross = 7;

{ Color Space Signatures }
const
	cmXYZData = FourCharCode('XYZ ');
	cmLabData = FourCharCode('Lab ');
	cmLuvData = FourCharCode('Luv ');
	cmYCbCrData = FourCharCode('YCbr');
	cmYxyData = FourCharCode('Yxy ');
	cmRGBData = FourCharCode('RGB ');
	cmSRGBData = FourCharCode('sRGB');
	cmGrayData = FourCharCode('GRAY');
	cmHSVData = FourCharCode('HSV ');
	cmHLSData = FourCharCode('HLS ');
	cmCMYKData = FourCharCode('CMYK');
	cmCMYData = FourCharCode('CMY ');
	cmMCH5Data = FourCharCode('MCH5');
	cmMCH6Data = FourCharCode('MCH6');
	cmMCH7Data = FourCharCode('MCH7');
	cmMCH8Data = FourCharCode('MCH8');
	cm3CLRData = FourCharCode('3CLR');
	cm4CLRData = FourCharCode('4CLR');
	cm5CLRData = FourCharCode('5CLR');
	cm6CLRData = FourCharCode('6CLR');
	cm7CLRData = FourCharCode('7CLR');
	cm8CLRData = FourCharCode('8CLR');
	cm9CLRData = FourCharCode('9CLR');
	cm10CLRData = FourCharCode('ACLR');
	cm11CLRData = FourCharCode('BCLR');
	cm12CLRData = FourCharCode('CCLR');
	cm13CLRData = FourCharCode('DCLR');
	cm14CLRData = FourCharCode('ECLR');
	cm15CLRData = FourCharCode('FCLR');
	cmNamedData = FourCharCode('NAME');

{ profileClass enumerations }
const
	cmInputClass = FourCharCode('scnr');
	cmDisplayClass = FourCharCode('mntr');
	cmOutputClass = FourCharCode('prtr');
	cmLinkClass = FourCharCode('link');
	cmAbstractClass = FourCharCode('abst');
	cmColorSpaceClass = FourCharCode('spac');
	cmNamedColorClass = FourCharCode('nmcl');

{ platform enumerations }
const
	cmMacintosh = FourCharCode('APPL');
	cmMicrosoft = FourCharCode('MSFT');
	cmSolaris = FourCharCode('SUNW');
	cmSiliconGraphics = FourCharCode('SGI ');
	cmTaligent = FourCharCode('TGNT');

{ parametric curve type enumerations }
const
	cmParametricType0 = 0;    { Y = X^gamma }
	cmParametricType1 = 1;    { Y = (aX+b)^gamma     [X>=-b/a],  Y = 0    [X<-b/a] }
	cmParametricType2 = 2;    { Y = (aX+b)^gamma + c [X>=-b/a],  Y = c    [X<-b/a] }
	cmParametricType3 = 3;    { Y = (aX+b)^gamma     [X>=d],     Y = cX   [X<d]    }
	cmParametricType4 = 4;     { Y = (aX+b)^gamma + e [X>=d],     Y = cX+f [X<d]    }


{ ColorSync 1.0 elements }
const
	cmCS1ChromTag = FourCharCode('chrm');
	cmCS1TRCTag = FourCharCode('trc ');
	cmCS1NameTag = FourCharCode('name');
	cmCS1CustTag = FourCharCode('cust');

{ General element data types }
type
	CMDateTimePtr = ^CMDateTime;
	CMDateTime = record
		year: UInt16;
		month: UInt16;
		dayOfTheMonth: UInt16;
		hours: UInt16;
		minutes: UInt16;
		seconds: UInt16;
	end;

type
	CMFixedXYColorPtr = ^CMFixedXYColor;
	CMFixedXYColor = record
		x: Fixed;
		y: Fixed;
	end;

type
	CMFixedXYZColorPtr = ^CMFixedXYZColor;
	CMFixedXYZColor = record
		X: Fixed;
		Y: Fixed;
		Z: Fixed;
	end;

type
	CMXYZComponent = UInt16;

type
	CMXYZColorPtr = ^CMXYZColor;
	CMXYZColor = record
		X: CMXYZComponent;
		Y: CMXYZComponent;
		Z: CMXYZComponent;
	end;

{ Typedef for Profile MD5 message digest }
{ Derived from the RSA Data Security, Inc. MD5 Message-Digest Algorithm }

     CMProfileMD5 = packed array[0..15] of UInt8;
     CMProfileMD5Ptr = ^CMProfileMD5;

{
 *  CMProfileMD5AreEqual()
 *  
 *  Availability:       available as macro/inline
 }
//  #define CMProfileMD5AreEqual(a, b) (\
//    ((long*)a)[0]==((long*)b)[0] && ((long*)a)[1]==((long*)b)[1] && \
//  ((long*)a)[2]==((long*)b)[2] && ((long*)a)[3]==((long*)b)[3])


type
	CM2HeaderPtr = ^CM2Header;
	CM2Header = record
		size: UInt32;                   { This is the total size of the Profile }
		CMMType: OSType;                { CMM signature,  Registered with CS2 consortium  }
		profileVersion: UInt32;         { Version of CMProfile format }
		profileClass: OSType;           { input, display, output, devicelink, abstract, or color conversion profile type }
		dataColorSpace: OSType;         { color space of data }
		profileConnectionSpace: OSType; { profile connection color space }
		dateTime: CMDateTime;               { date and time of profile creation }
		CS2profileSignature: OSType;    { 'acsp' constant ColorSync 2.0 file ID }
		platform: OSType;               { primary profile platform, Registered with CS2 consortium }
		flags: UInt32;                  { profile flags }
		deviceManufacturer: OSType;     { Registered with ICC consortium }
		deviceModel: UInt32;            { Registered with ICC consortium }
		deviceAttributes: array [0..1] of UInt32;    { Attributes[0] is for device vendors, [1] is for ICC }
		renderingIntent: UInt32;        { preferred rendering intent of tagged object }
		white: CMFixedXYZColor;                  { profile illuminant }
		creator: OSType;                { profile creator }
		reserved: array [0..43] of SInt8;           { reserved for future use }
	end;

type
	CM4HeaderPtr = ^CM4Header;
	CM4Header = record
		size: UInt32;                   { This is the total size of the Profile }
		CMMType: OSType;                { CMM signature,  Registered with CS2 consortium  }
		profileVersion: UInt32;         { Version of CMProfile format }
		profileClass: OSType;           { input, display, output, devicelink, abstract, or color conversion profile type }
		dataColorSpace: OSType;         { color space of data }
		profileConnectionSpace: OSType; { profile connection color space }
		dateTime: CMDateTime;               { date and time of profile creation }
		CS2profileSignature: OSType;    { 'acsp' constant ColorSync 2.0 file ID }
		platform: OSType;               { primary profile platform, Registered with CS2 consortium }
		flags: UInt32;                  { profile flags }
		deviceManufacturer: OSType;     { Registered with ICC consortium }
		deviceModel: UInt32;            { Registered with ICC consortium }
		deviceAttributes: array [0..1] of UInt32;    { Attributes[0] is for device vendors, [1] is for ICC }
		renderingIntent: UInt32;        { preferred rendering intent of tagged object }
		white: CMFixedXYZColor;                  { profile illuminant }
		creator: OSType;                { profile creator }
		digest: CMProfileMD5;                 { Profile message digest }
		reserved: array [0..27] of SInt8;           { reserved for future use }
	end;

type
	CMTagRecordPtr = ^CMTagRecord;
	CMTagRecord = record
		tag: OSType;                    { Registered with CS2 consortium }
		elementOffset: UInt32;          { Relative to start of CMProfile }
		elementSize: UInt32;
	end;

type
	CMTagElemTablePtr = ^CMTagElemTable;
	CMTagElemTable = record
		count: UInt32;
		tagList: array [0..0] of CMTagRecord;             { variable size, determined by count }
	end;

type
	CM2ProfilePtr = ^CM2Profile;
	CM2Profile = record
		header: CM2Header;
		tagTable: CMTagElemTable;
		elemData: SInt8;            { variable size data for tagged element storage }
	end;

	CM2ProfileHandle					= ^CM2ProfilePtr;
	{	 Tag Type Definitions 	}
type
	CMAdaptationMatrixTypePtr = ^CMAdaptationMatrixType;
	CMAdaptationMatrixType = record
		typeDescriptor: OSType;         { 'sf32' = cmSigS15Fixed16Type }
		reserved: UInt32;               { fill with 0x00 }
		adaptationMatrix: array [0..8] of Fixed;    { fixed size of nine matrix entries }
	end;

	CMCurveTypePtr = ^CMCurveType;
	CMCurveType = record
		typeDescriptor: OSType;         { 'curv' = cmSigCurveType }
		reserved: UInt32;               { fill with 0x00 }
		countValue: UInt32;             { number of entries in table that follows }
		data: array [0..0] of UInt16;                { variable size, determined by countValue }
	end;

type
	CMDataTypePtr = ^CMDataType;
	CMDataType = record
		typeDescriptor: OSType;         { 'data' = cmSigDataType}
		reserved: UInt32;               { fill with 0x00 }
		dataFlag: UInt32;               { 0 = ASCII, 1 = binary }
		data: array [0..0] of char;                { variable size, determined by tag element size }
	end;

type
	CMDateTimeTypePtr = ^CMDateTimeType;
	CMDateTimeType = record
		typeDescriptor: OSType;         { 'dtim' = cmSigDateTimeType }
		reserved: UInt32;               { fill with 0x00 }
		dateTime: CMDateTime;               { }
	end;

type
	CMLut16TypePtr = ^CMLut16Type;
	CMLut16Type = record
		typeDescriptor: OSType;         { 'mft2' = cmSigLut16Type }
		reserved: UInt32;               { fill with 0x00 }
		inputChannels: UInt8;          { Number of input channels }
		outputChannels: UInt8;         { Number of output channels }
		gridPoints: UInt8;             { Number of clutTable grid points }
		reserved2: UInt8;              { fill with 0x00 }
		matrix: array [0..2,0..2] of Fixed;           { }
		inputTableEntries: UInt16;      { Number of entries in 1-D input luts }
		outputTableEntries: UInt16;     { Number of entries in 1-D output luts }
		inputTable: array [0..0] of UInt16;          { variable size, determined by inputChannels*inputTableEntries }
	end;

type
	CMLut8TypePtr = ^CMLut8Type;
	CMLut8Type = record
		typeDescriptor: OSType;         { 'mft1' = cmSigLut8Type }
		reserved: UInt32;               { fill with 0x00 }
		inputChannels: UInt8;          { Number of input channels }
		outputChannels: UInt8;         { Number of output channels }
		gridPoints: UInt8;             { Number of clutTable grid points }
		reserved2: UInt8;              { fill with 0x00 }
		matrix: array [0..2,0..2] of Fixed;           { }
		inputTable: array [0..0] of UInt8;          { variable size, determined by inputChannels*256 }
	end;

type
	CMMultiFunctLutTypePtr = ^CMMultiFunctLutType;
	CMMultiFunctLutType = record
		typeDescriptor: OSType;         { 'mAB ' = cmSigMultiFunctA2BType or 'mBA ' = cmSigMultiFunctB2AType }
		reserved: UInt32;               { fill with 0x00 }
		inputChannels: UInt8;          { Number of input channels }
		outputChannels: UInt8;         { Number of output channels }
		reserved2: UInt16;              { fill with 0x00 }
		offsetBcurves: UInt32;          { offset to first "B" curve }
		offsetMatrix: UInt32;           { offset to 3x4 matrix }
		offsetMcurves: UInt32;          { offset to first "M" curve }
		offsetCLUT: UInt32;             { offset to multi-dimensional LUT of type CMMultiFunctCLUTType }
		offsetAcurves: UInt32;          { offset to first "A" curve }
		data: array [0..0] of UInt8;                { variable size }
	end;

	CMMultiFunctLutA2BType = CMMultiFunctLutType;
	CMMultiFunctLutA2BTypePtr = ^CMMultiFunctLutA2BType;
	CMMultiFunctLutB2AType = CMMultiFunctLutType;
	CMMultiFunctLutB2ATypePtr = ^CMMultiFunctLutB2AType;
	
	CMMultiFunctCLUTTypePtr = ^CMMultiFunctCLUTType;
	CMMultiFunctCLUTType = record
		gridPoints: packed array[0..15] of UInt8;         { grigpoints for each input channel dimension (remaining are 0) }
		entrySize: SInt8;              { bytes per lut enrty (1 or 2) }
		reserved: array[0..2] of SInt8;            { fill with 0x00 }
		data: SInt8;                { variable size, determined by above }
		pad: SInt8;                                  { pad byte needed for correct record size. Critical to accessing CMMultiFunctLutType's variable sized data field contents. }
	end;

type
	CMMeasurementTypePtr = ^CMMeasurementType;
	CMMeasurementType = record
		typeDescriptor: OSType;         { 'meas' = cmSigMeasurementType }
		reserved: UInt32;               { fill with 0x00 }
		standardObserver: UInt32;       { cmStdobsUnknown, cmStdobs1931TwoDegrees, cmStdobs1964TenDegrees }
		backingXYZ: CMFixedXYZColor;             { absolute XYZ values of backing }
		geometry: UInt32;               { cmGeometryUnknown, cmGeometry045or450 (0/45), cmGeometry0dord0 (0/d or d/0) }
		flare: UInt32;                  { cmFlare0, cmFlare100 }
		illuminant: UInt32;             { cmIlluminantUnknown, cmIlluminantD50, ... }
	end;

type
	CMNamedColorTypePtr = ^CMNamedColorType;
	CMNamedColorType = record
		typeDescriptor: OSType;         { 'ncol' = cmSigNamedColorType }
		reserved: UInt32;               { fill with 0x00 }
		vendorFlag: UInt32;             { }
		count: UInt32;                  { count of named colors in array that follows }
		prefixName: array [0..0] of UInt8;          { variable size, max = 32 }
	end;

type
	CMNamedColor2EntryTypePtr = ^CMNamedColor2EntryType;
	CMNamedColor2EntryType = record
		rootName: packed array [0..31] of UInt8;			{ 32 byte field.  7 bit ASCII null terminated }
		PCSColorCoords: array [0..2] of UInt16;					{ Lab or XYZ color }
		DeviceColorCoords: array [0..0] of UInt16;					{ variable size }
	end;

type
	CMNamedColor2TypePtr = ^CMNamedColor2Type;
	CMNamedColor2Type = record
		typeDescriptor: OSType;         { 'ncl2' = cmSigNamedColor2Type }
		reserved: UInt32;               { fill with 0x00 }
		vendorFlag: UInt32;             { lower 16 bits reserved for ICC use }
		count: UInt32;                  { count of named colors in array that follows }
		deviceChannelCount: UInt32;     { number of device channels, 0 indicates no device value available }
		prefixName: packed array [0..31] of UInt8;			{ Fixed 32 byte size.  7 bit ASCII null terminated }
		suffixName: packed array [0..31] of UInt8;			{ Fixed 32 byte size.  7 bit ASCII null terminated }
		data: SInt8;									{ variable size data for CMNamedColor2EntryType }
	end;

type
	CMNativeDisplayInfoPtr = ^CMNativeDisplayInfo;
	CMNativeDisplayInfo = record
		dataSize: UInt32;               { Size of this structure }
		redPhosphor: CMFixedXYColor;            { Phosphors - native cromaticity values of the display  }
		greenPhosphor: CMFixedXYColor;
		bluePhosphor: CMFixedXYColor;
		whitePoint: CMFixedXYColor;
		redGammaValue: Fixed;          { Gammas - native gamma values of the display }
		greenGammaValue: Fixed;
		blueGammaValue: Fixed;
                                              {  Gamma tables - if if gammaChannels is not zero, }
                                              {  native gamma tables are preferred over values }
                                              {  redGammaValue, greenGammaValue, blueGammaValue }
		gammaChannels: UInt16;          { # of gamma channels (1 or 3) }
		gammaEntryCount: UInt16;        { 1-based number of entries per channel }
		gammaEntrySize: UInt16;         { size in bytes of each entry }
		gammaData: array [0..0] of SInt8;           { variable size, determined by channels*entryCount*entrySize }
	end;

type
	CMNativeDisplayInfoTypePtr = ^CMNativeDisplayInfoType;
	CMNativeDisplayInfoType = record
		typeDescriptor: OSType;         { 'ndin' = cmSigNativeDisplayInfoType }
		reserved: UInt32;               { fill with 0x00 }
		nativeDisplayInfo: CMNativeDisplayInfo;      { data of type CMNativeDisplayInfo }
	end;

type
	CMParametricCurveTypePtr = ^CMParametricCurveType;
	CMParametricCurveType = record
		typeDescriptor: OSType;         { 'para' = cmSigParametricCurveType }
		reserved: UInt32;               { fill with 0x00 }
		functionType: UInt16;           { cmParametricType0, cmParametricType1, etc. }
		reserved2: UInt16;              { fill with 0x00 }
		value: array [0..0] of Fixed;               { variable size, determined by functionType }
	end;

type
	CMTextDescriptionType = packed record
		typeDescriptor: OSType;         { 'desc' = cmSigProfileDescriptionType }
		reserved: UInt32;               { fill with 0x00 }
		ASCIICount: UInt32;             { Count of bytes (including null terminator)  }
		ASCIIName: packed array [0..1] of UInt8;           { variable size, determined by ASCIICount.  7 bit ASCII null terminated }
	end;

type
	CMTextTypePtr = ^CMTextType;
	CMTextType = record
		typeDescriptor: OSType;         { 'text' = cmSigTextType }
		reserved: UInt32;               { fill with 0x00 }
		text: array [0..0] of UInt8;                { variable size, determined by tag element size }
	end;

type
	CMUnicodeTextTypePtr = ^CMUnicodeTextType;
	CMUnicodeTextType = record
		typeDescriptor: OSType;         { 'utxt' = cmSigUnicodeTextType }
		reserved: UInt32;               { fill with 0x00 }
		text: array [0..0] of UniChar;                { variable size, determined by tag element size  }
	end;

type
	CMScreeningChannelRecPtr = ^CMScreeningChannelRec;
	CMScreeningChannelRec = record
		frequency: Fixed;
		angle: Fixed;
		spotFunction: UInt32;
	end;

type
	CMScreeningTypePtr = ^CMScreeningType;
	CMScreeningType = record
		typeDescriptor: OSType;         { 'scrn' = cmSigScreeningType }
		reserved: UInt32;               { fill with 0x00 }
		screeningFlag: UInt32;          { bit 0 : use printer default screens, bit 1 : inch/cm }
		channelCount: UInt32;           { }
		channelInfo: array [0..0] of CMScreeningChannelRec;      { variable size, determined by channelCount }
	end;

type
	CMSignatureTypePtr = ^CMSignatureType;
	CMSignatureType = record
		typeDescriptor: OSType;         { 'sig ' = cmSigSignatureType }
		reserved: UInt32;               { fill with 0x00 }
		signature: OSType;
	end;

type
	CMS15Fixed16ArrayTypePtr = ^CMS15Fixed16ArrayType;
	CMS15Fixed16ArrayType = record
		typeDescriptor: OSType;         { 'sf32' = cmSigS15Fixed16Type }
		reserved: UInt32;               { fill with 0x00 }
		value: array [0..0] of Fixed;               { variable size, determined by tag element size }
	end;

type
	CMU16Fixed16ArrayTypePtr = ^CMU16Fixed16ArrayType;
	CMU16Fixed16ArrayType = record
		typeDescriptor: OSType;         { 'uf32' = cmSigU16Fixed16Type }
		reserved: UInt32;               { fill with 0x00 }
		value: array [0..0] of UInt32;               { variable size, determined by tag element size }
	end;

type
	CMUInt8ArrayTypePtr = ^CMUInt8ArrayType;
	CMUInt8ArrayType = record
		typeDescriptor: OSType;         { 'ui08' = cmSigUInt8Type }
		reserved: UInt32;               { fill with 0x00 }
		value: array [0..0] of UInt8;               { variable size, determined by tag element size }
	end;

type
	CMUInt16ArrayTypePtr = ^CMUInt16ArrayType;
	CMUInt16ArrayType = record
		typeDescriptor: OSType;         { 'ui16' = cmSigUInt16Type }
		reserved: UInt32;               { fill with 0x00 }
		value: array [0..0] of UInt16;               { variable size, determined by tag element size }
	end;

type
	CMUInt32ArrayTypePtr = ^CMUInt32ArrayType;
	CMUInt32ArrayType = record
		typeDescriptor: OSType;         { 'ui32' = cmSigUInt32Type }
		reserved: UInt32;               { fill with 0x00 }
		value: array [0..0] of UInt32;               { variable size, determined by tag element size }
	end;

type
	CMUInt64ArrayTypePtr = ^CMUInt64ArrayType;
	CMUInt64ArrayType = record
		typeDescriptor: OSType;         { 'ui64' = cmSigUInt64Type }
		reserved: UInt32;               { fill with 0x00 }
		value: array [0..0] of UInt32;               { variable size, determined by tag element size }
	end;

type
	CMViewingConditionsTypePtr = ^CMViewingConditionsType;
	CMViewingConditionsType = record
		typeDescriptor: OSType;         { 'view' = cmSigViewingConditionsType }
		reserved: UInt32;               { fill with 0x00 }
		illuminant: CMFixedXYZColor;             { absolute XYZs of illuminant  in cd/m^2 }
		surround: CMFixedXYZColor;               { absolute XYZs of surround in cd/m^2 }
		stdIlluminant: UInt32;          { see definitions of std illuminants }
	end;

type
	CMXYZTypePtr = ^CMXYZType;
	CMXYZType = record
		typeDescriptor: OSType;         { 'XYZ ' = cmSigXYZType }
		reserved: UInt32;               { fill with 0x00 }
		XYZ: array [0..0] of CMFixedXYZColor;                 { variable size, determined by tag element size }
	end;

type
	CMProfileSequenceDescTypePtr = ^CMProfileSequenceDescType;
	CMProfileSequenceDescType = record
		typeDescriptor: OSType;         { 'pseq' = cmProfileSequenceDescTag }
		reserved: UInt32;               { fill with 0x00 }
		count: UInt32;                  { Number of descriptions }
		data: array [0..0] of SInt8;                { variable size data explained in ICC spec }
	end;

type
	CMUcrBgTypePtr = ^CMUcrBgType;
	CMUcrBgType = record
		typeDescriptor: OSType;         { 'bfd ' = cmSigUcrBgType }
		reserved: UInt32;               { fill with 0x00 }
		ucrCount: UInt32;               { Number of UCR entries }
		ucrValues: array [0..0] of UInt16;           { variable size, determined by ucrCount }
	end;

type
	{	 Private Tag Type Definitions 	}
	CMIntentCRDVMSizePtr = ^CMIntentCRDVMSize;
	CMIntentCRDVMSize = record
		renderingIntent: UInt32;        { rendering intent }
		VMSize: UInt32;                 { VM size taken up by the CRD }
	end;

type
	CMPS2CRDVMSizeTypePtr = ^CMPS2CRDVMSizeType;
	CMPS2CRDVMSizeType = record
		typeDescriptor: OSType;         { 'psvm' = cmSigPS2CRDVMSizeType }
		reserved: UInt32;               { fill with 0x00 }
		count: UInt32;                  { number of intent entries }
		intentCRD: array [0..0] of CMIntentCRDVMSize;           { variable size, determined by count }
	end;


const
	cmVideoCardGammaTableType = 0;
	cmVideoCardGammaFormulaType = 1;

type
	CMVideoCardGammaTablePtr = ^CMVideoCardGammaTable;
	CMVideoCardGammaTable = record
		channels: UInt16;               { # of gamma channels (1 or 3) }
		entryCount: UInt16;             { 1-based number of entries per channel }
		entrySize: UInt16;              { size in bytes of each entry }
		data: array [0..0] of SInt8;                { variable size, determined by channels*entryCount*entrySize }
	end;

type
	CMVideoCardGammaFormulaPtr = ^CMVideoCardGammaFormula;
	CMVideoCardGammaFormula = record
		redGamma: Fixed;               { must be > 0.0 }
		redMin: Fixed;                 { must be > 0.0 and < 1.0 }
		redMax: Fixed;                 { must be > 0.0 and < 1.0 }
		greenGamma: Fixed;             { must be > 0.0 }
		greenMin: Fixed;               { must be > 0.0 and < 1.0 }
		greenMax: Fixed;               { must be > 0.0 and < 1.0 }
		blueGamma: Fixed;              { must be > 0.0 }
		blueMin: Fixed;                { must be > 0.0 and < 1.0 }
		blueMax: Fixed;                { must be > 0.0 and < 1.0 }
	end;

type
	CMVideoCardGammaPtr = ^CMVideoCardGamma;
	CMVideoCardGamma = record
		tagType: UInt32;
		case SInt16 of
		0: (
			table: CMVideoCardGammaTable;
			);
		1: (
			formula: CMVideoCardGammaFormula;
			);
	end;

type
	CMVideoCardGammaTypePtr = ^CMVideoCardGammaType;
	CMVideoCardGammaType = record
		typeDescriptor: OSType;         { 'vcgt' = cmSigVideoCardGammaType }
		reserved: UInt32;               { fill with 0x00 }
		gamma: CMVideoCardGamma;
	end;

type
	CMMakeAndModelPtr = ^CMMakeAndModel;
	CMMakeAndModel = record
		manufacturer: OSType;
		model: UInt32;
		serialNumber: UInt32;
		manufactureDate: UInt32;
		reserved1: UInt32;              { fill with 0x00 }
		reserved2: UInt32;              { fill with 0x00 }
		reserved3: UInt32;              { fill with 0x00 }
		reserved4: UInt32;              { fill with 0x00 }
	end;

type
	CMMakeAndModelTypePtr = ^CMMakeAndModelType;
	CMMakeAndModelType = record
		typeDescriptor: OSType;         { 'mmod' = cmSigMakeAndModelType }
		reserved: UInt32;               { fill with 0x00 }
		makeAndModel: CMMakeAndModel;
	end;

type
	CMMultiLocalizedUniCodeEntryRecPtr = ^CMMultiLocalizedUniCodeEntryRec;
	CMMultiLocalizedUniCodeEntryRec = record
		languageCode: packed array [0..1] of char;        { language code from ISO-639 }
		regionCode: packed array [0..1] of char;          { region code from ISO-3166 }
		textLength: UInt32;             { the length in bytes of the string }
		textOffset: UInt32;             { the offset from the start of tag in bytes }
	end;

type
	CMMultiLocalizedUniCodeTypePtr = ^CMMultiLocalizedUniCodeType;
	CMMultiLocalizedUniCodeType = record
		typeDescriptor: OSType;         { 'mluc' = cmSigMultiLocalizedUniCodeType }
		reserved: UInt32;               { fill with 0x00 }
		entryCount: UInt32;             { 1-based number of name records that follow }
		entrySize: UInt32;              { size in bytes of name records that follow }
                                              { variable-length data for storage of CMMultiLocalizedUniCodeEntryRec }
                                              { variable-length data for storage of Unicode strings}
	end;

{$ifc not TARGET_CPU_64}
{**********************************************************************}
{************** ColorSync 1.0 profile specification *******************}
{**********************************************************************}
const
	cmGrayResponse = 0;
	cmRedResponse = 1;
	cmGreenResponse = 2;
	cmBlueResponse = 3;
	cmCyanResponse = 4;
	cmMagentaResponse = 5;
	cmYellowResponse = 6;
	cmUcrResponse = 7;
	cmBgResponse = 8;
	cmOnePlusLastResponse = 9;


{ Device types }
const
	cmMonitorDevice = FourCharCode('mntr');
	cmScannerDevice = FourCharCode('scnr');
	cmPrinterDevice = FourCharCode('prtr');


type
	CMIStringPtr = ^CMIString;
	CMIString = record
		theScript: ScriptCode;
		theString: Str63;
	end;

{ Profile options }
const
	cmPerceptualMatch = $0000; { Default. For photographic images }
	cmColorimetricMatch = $0001; { Exact matching when possible }
	cmSaturationMatch = $0002; { For solid colors }

{ Profile flags }
const
	cmNativeMatchingPreferred = $00000001; { Default to native not preferred }
	cmTurnOffCache = $00000002; { Default to turn on CMM cache }


type
	CMMatchOption						= SInt32;
	CMMatchFlag							= SInt32;
type
	CMHeaderPtr = ^CMHeader;
	CMHeader = record
		CMMType: OSType;
		applProfileVersion: UInt32;
		dataType: OSType;
		deviceType: OSType;
		deviceManufacturer: OSType;
		deviceModel: UInt32;
		deviceAttributes: array [0..1] of UInt32;
		profileNameOffset: UInt32;
		customDataOffset: UInt32;
		flags: CMMatchFlag;
		options: CMMatchOption;
		white: CMXYZColor;
		black: CMXYZColor;
	end;

type
	CMProfileChromaticitiesPtr = ^CMProfileChromaticities;
	CMProfileChromaticities = record
		red: CMXYZColor;
		green: CMXYZColor;
		blue: CMXYZColor;
		cyan: CMXYZColor;
		magenta: CMXYZColor;
		yellow: CMXYZColor;
	end;

type
	CMProfileResponsePtr = ^CMProfileResponse;
	CMProfileResponse = record
		counts: array [0..8] of UInt16;
		data: array [0..0] of UInt16;                { Variable size }
	end;

type
	CMProfilePtr = ^CMProfile;
	CMProfile = record
		header: CMHeader;
		profile: CMProfileChromaticities;
		response: CMProfileResponse;
		profileName: CMIString;
		customData: array [0..0] of SInt8;          { Variable size }
	end;

	CMProfileHandle						= ^CMProfilePtr;

{$endc} {not TARGET_CPU_64}

// #pragma mark --- CMTypes.h ----

type
	CMError = OSStatus;
{ Abstract data type for memory-based Profile }
	OpaqueCMProfileRef = record end;
	CMProfileRef = ^OpaqueCMProfileRef; { an opaque type }
	CMProfileRefPtr = ^CMProfileRef;  { when a var xx:CMProfileRef parameter can be nil, it is changed to xx: CMProfileRefPtr }
	
{$ifc not TARGET_CPU_64}
{ Abstract data type for Profile search result }
	OpaqueCMProfileSearchRef = record end;
	CMProfileSearchRef = ^OpaqueCMProfileSearchRef; { an opaque type }
	CMProfileSearchRefPtr = ^CMProfileSearchRef;  { when a var xx:CMProfileSearchRef parameter can be nil, it is changed to xx: CMProfileSearchRefPtr }

{ Abstract data type for BeginMatching(É) reference }
	OpaqueCMMatchRef = record end;
	CMMatchRef = ^OpaqueCMMatchRef; { an opaque type }
	CMMatchRefPtr = ^CMMatchRef;  { when a var xx:CMMatchRef parameter can be nil, it is changed to xx: CMMatchRefPtr }
{$endc} {not TARGET_CPU_64}

{ Abstract data type for ColorWorld reference }
	OpaqueCMWorldRef = record end;
	CMWorldRef = ^OpaqueCMWorldRef; { an opaque type }
	CMWorldRefPtr = ^CMWorldRef;  { when a var xx:CMWorldRef parameter can be nil, it is changed to xx: CMWorldRefPtr }

{ Data type for ColorSync DisplayID reference }
{ On 8 & 9 this is a AVIDType }
{ On X this is a CGSDisplayID }
	CMDisplayIDType = UInt32;
	CMChromaticAdaptation = UInt32;
const
	cmLinearChromaticAdaptation = 1;
	cmVonKriesChromaticAdaptation = 2;
	cmBradfordChromaticAdaptation = 3;


type
	CMFlattenProcPtr = function(command: SInt32; var size: SIGNEDLONG; data: UnivPtr; refCon: UnivPtr): OSErr;

	{	 Caller-supplied progress function for NCMMConcatInit & NCMMNewLinkProfile routines 	}
	CMConcatCallBackProcPtr = function(progress: SInt32; refCon: UnivPtr): boolean;

	{	 Caller-supplied filter function for Profile search 	}
	CMProfileFilterProcPtr = function(prof: CMProfileRef; refCon: UnivPtr): boolean;

	{	 Caller-supplied function for profile access 	}
	CMProfileAccessProcPtr = function(command: SInt32; offset: SInt32; var size: SInt32; data: UnivPtr; refCon: UnivPtr): OSErr;

	CMFlattenUPP = CMFlattenProcPtr;
	CMConcatCallBackUPP = CMConcatCallBackProcPtr;
	CMProfileFilterUPP = CMProfileFilterProcPtr;
	CMProfileAccessUPP = CMProfileAccessProcPtr;

{
 *  NewCMFlattenUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCMFlattenUPP(userRoutine: CMFlattenProcPtr): CMFlattenUPP; external name '_NewCMFlattenUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  NewCMConcatCallBackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCMConcatCallBackUPP(userRoutine: CMConcatCallBackProcPtr): CMConcatCallBackUPP; external name '_NewCMConcatCallBackUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  NewCMProfileFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCMProfileFilterUPP(userRoutine: CMProfileFilterProcPtr): CMProfileFilterUPP; external name '_NewCMProfileFilterUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  NewCMProfileAccessUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCMProfileAccessUPP(userRoutine: CMProfileAccessProcPtr): CMProfileAccessUPP; external name '_NewCMProfileAccessUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeCMFlattenUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeCMFlattenUPP(userUPP: CMFlattenUPP); external name '_DisposeCMFlattenUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeCMConcatCallBackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeCMConcatCallBackUPP(userUPP: CMConcatCallBackUPP); external name '_DisposeCMConcatCallBackUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeCMProfileFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeCMProfileFilterUPP(userUPP: CMProfileFilterUPP); external name '_DisposeCMProfileFilterUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeCMProfileAccessUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeCMProfileAccessUPP(userUPP: CMProfileAccessUPP); external name '_DisposeCMProfileAccessUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeCMFlattenUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeCMFlattenUPP(command: SInt32; var size: SIGNEDLONG; data: UnivPtr; refCon: UnivPtr; userRoutine: CMFlattenUPP): OSErr; external name '_InvokeCMFlattenUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeCMConcatCallBackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeCMConcatCallBackUPP(progress: SInt32; refCon: UnivPtr; userRoutine: CMConcatCallBackUPP): boolean; external name '_InvokeCMConcatCallBackUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeCMProfileFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeCMProfileFilterUPP(prof: CMProfileRef; refCon: UnivPtr; userRoutine: CMProfileFilterUPP): boolean; external name '_InvokeCMProfileFilterUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeCMProfileAccessUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeCMProfileAccessUPP(command: SInt32; offset: SInt32; var size: SInt32; data: UnivPtr; refCon: UnivPtr; userRoutine: CMProfileAccessUPP): OSErr; external name '_InvokeCMProfileAccessUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


//#pragma mark --- CMApplication.h ----


{$setc _DECLARE_CS_QD_API_ := 0} { Mac OS X ColorSync QuickDraw API are located in QuickDraw.p[.pas] }

{$ifc not TARGET_CPU_64}
{$ALIGN MAC68K}
{$elsec}
{$packrecords c}
{$endc}


const
	kDefaultCMMSignature = FourCharCode('appl');

{$ifc not TARGET_CPU_64}
{ PicComment IDs }
const
	cmBeginProfile = 220;
	cmEndProfile = 221;
	cmEnableMatching = 222;
	cmDisableMatching = 223;
	cmComment = 224;

{ PicComment selectors for cmComment }
const
	cmBeginProfileSel = 0;
	cmContinueProfileSel = 1;
	cmEndProfileSel = 2;
	cmProfileIdentifierSel = 3;

{ Defines for version 1.0 CMProfileSearchRecord.fieldMask }
const
	cmMatchCMMType = $00000001;
	cmMatchApplProfileVersion = $00000002;
	cmMatchDataType = $00000004;
	cmMatchDeviceType = $00000008;
	cmMatchDeviceManufacturer = $00000010;
	cmMatchDeviceModel = $00000020;
	cmMatchDeviceAttributes = $00000040;
	cmMatchFlags = $00000080;
	cmMatchOptions = $00000100;
	cmMatchWhite = $00000200;
	cmMatchBlack = $00000400;

{ Defines for version 2.0 CMSearchRecord.searchMask }
const
	cmMatchAnyProfile = $00000000;
	cmMatchProfileCMMType = $00000001;
	cmMatchProfileClass = $00000002;
	cmMatchDataColorSpace = $00000004;
	cmMatchProfileConnectionSpace = $00000008;
	cmMatchManufacturer = $00000010;
	cmMatchModel = $00000020;
	cmMatchAttributes = $00000040;
	cmMatchProfileFlags = $00000080;

{$endc} {not TARGET_CPU_64}

{ Flags for PostScript-related functions }
const
	cmPS7bit = 1;
	cmPS8bit = 2;

{$ifc not TARGET_CPU_64}
{ Flags for profile embedding functions }
const
	cmEmbedWholeProfile = $00000000;
	cmEmbedProfileIdentifier = $00000001;
{$endc} {not TARGET_CPU_64}

{ Commands for CMFlattenUPP() }
const
	cmOpenReadSpool = 1;
	cmOpenWriteSpool = 2;
	cmReadSpool = 3;
	cmWriteSpool = 4;
	cmCloseSpool = 5;

{ Commands for CMAccessUPP() }
const
	cmOpenReadAccess = 1;
	cmOpenWriteAccess = 2;
	cmReadAccess = 3;
	cmWriteAccess = 4;
	cmCloseAccess = 5;
	cmCreateNewAccess = 6;
	cmAbortWriteAccess = 7;
	cmBeginAccess = 8;
	cmEndAccess = 9;

{ Use types for CMGet/SetDefaultProfileByUse() }
const
	cmInputUse = FourCharCode('inpt');
	cmOutputUse = FourCharCode('outp');
	cmDisplayUse = FourCharCode('dply');
	cmProofUse = FourCharCode('pruf');


{ Union of 1.0 2.0, and 4.0 profile header variants }
type
	CMAppleProfileHeaderPtr = ^CMAppleProfileHeader;
	CMAppleProfileHeader = record
		case SInt16 of
{$ifc not TARGET_CPU_64}
		0: (
			cm1: CMHeader;
			);
{$endc} {not TARGET_CPU_64}
		1: (
			cm2: CM2Header;
			);
		2: (
			cm4: CM4Header;
			);
	end;

{ CWConcatColorWorld() definitions }
type
	CMConcatProfileSetPtr = ^CMConcatProfileSet;
	CMConcatProfileSet = record
		keyIndex: UInt16;               { Zero-based }
		count: UInt16;                  { Min 1 }
		profileSet: array [0..0] of CMProfileRef;          { Variable. Ordered from Source -> Dest }
	end;

{ NCWConcatColorWorld() definitions }
type
	NCMConcatProfileSpecPtr = ^NCMConcatProfileSpec;
	NCMConcatProfileSpec = record
		renderingIntent: UInt32;        { renderingIntent override }
		transformTag: UInt32;           { transform enumerations defined below }
		profile: CMProfileRef;                { profile }
	end;

type
	NCMConcatProfileSetPtr = ^NCMConcatProfileSet;
	NCMConcatProfileSet = record
		cmm: OSType;                    { e.g. 'KCMS', 'appl', ...  uniquely ids the cmm, or 0000 }
		flags: UInt32;                  { specify quality, lookup only, no gamut checking ... }
		flagsMask: UInt32;              { which bits of 'flags' to use to override profile }
		profileCount: UInt32;           { how many ProfileSpecs in the following set }
		profileSpecs: array [0..0] of NCMConcatProfileSpec;      { Variable. Ordered from Source -> Dest }
	end;

const
	kNoTransform = 0;    { Not used }
	kUseAtoB = 1;    { Use 'A2B*' tag from this profile or equivalent }
	kUseBtoA = 2;    { Use 'B2A*' tag from this profile or equivalent }
	kUseBtoB = 3;    { Use 'pre*' tag from this profile or equivalent }
                                        { For typical device profiles the following synonyms may be useful }
	kDeviceToPCS = kUseAtoB; { Device Dependent to Device Independent }
	kPCSToDevice = kUseBtoA; { Device Independent to Device Dependent }
	kPCSToPCS = kUseBtoB; { Independent, through device's gamut }
	kUseProfileIntent = -1; { For renderingIntent in NCMConcatProfileSpec    }


{ ColorSync color data types }
type
	CMRGBColorPtr = ^CMRGBColor;
	CMRGBColor = record
		red: UInt16;                    { 0..65535 }
		green: UInt16;
		blue: UInt16;
	end;

type
	CMCMYKColorPtr = ^CMCMYKColor;
	CMCMYKColor = record
		cyan: UInt16;                   { 0..65535 }
		magenta: UInt16;
		yellow: UInt16;
		black: UInt16;
	end;

type
	CMCMYColorPtr = ^CMCMYColor;
	CMCMYColor = record
		cyan: UInt16;                   { 0..65535 }
		magenta: UInt16;
		yellow: UInt16;
	end;

type
	CMHLSColorPtr = ^CMHLSColor;
	CMHLSColor = record
		hue: UInt16;                    { 0..65535. Fraction of circle. Red at 0 }
		lightness: UInt16;              { 0..65535 }
		saturation: UInt16;             { 0..65535 }
	end;

type
	CMHSVColorPtr = ^CMHSVColor;
	CMHSVColor = record
		hue: UInt16;                    { 0..65535. Fraction of circle. Red at 0 }
		saturation: UInt16;             { 0..65535 }
		value: UInt16;                  { 0..65535 }
	end;

type
	CMLabColorPtr = ^CMLabColor;
	CMLabColor = record
		L: UInt16;                      { 0..65535 maps to 0..100 }
		a: UInt16;                      { 0..65535 maps to -128..127.996 }
		b: UInt16;                      { 0..65535 maps to -128..127.996 }
	end;

type
	CMLuvColorPtr = ^CMLuvColor;
	CMLuvColor = record
		L: UInt16;                      { 0..65535 maps to 0..100 }
		u: UInt16;                      { 0..65535 maps to -128..127.996 }
		v: UInt16;                      { 0..65535 maps to -128..127.996 }
	end;

type
	CMYxyColorPtr = ^CMYxyColor;
	CMYxyColor = record
		capY: UInt16;                   { 0..65535 maps to 0..1 }
		x: UInt16;                      { 0..65535 maps to 0..1 }
		y: UInt16;                      { 0..65535 maps to 0..1 }
	end;

type
	CMGrayColorPtr = ^CMGrayColor;
	CMGrayColor = record
		gray: UInt16;                   { 0..65535 }
	end;

type
	CMMultichannel5ColorPtr = ^CMMultichannel5Color;
	CMMultichannel5Color = record
		components: packed array [0..4] of UInt8;          { 0..255 }
		pad: UInt8; {pad byte so record size equals Apple gcc struct size}
	end;

type
	CMMultichannel6ColorPtr = ^CMMultichannel6Color;
	CMMultichannel6Color = record
		components: packed array [0..5] of UInt8;          { 0..255 }
	end;

type
	CMMultichannel7ColorPtr = ^CMMultichannel7Color;
	CMMultichannel7Color = record
		components: packed array [0..6] of UInt8;          { 0..255 }
		pad: UInt8; {pad byte so record size equals Apple gcc struct size}
	end;

type
	CMMultichannel8ColorPtr = ^CMMultichannel8Color;
	CMMultichannel8Color = record
		components: packed array [0..7] of UInt8;          { 0..255 }
	end;

type
	CMNamedColorPtr = ^CMNamedColor;
	CMNamedColor = record
		namedColorIndex: UInt32;        { 0..a lot }
	end;

	CMColorPtr = ^CMColor;
	CMColor = record
		case SInt16 of
		0: (
			rgb: CMRGBColor;
			);
		1: (
			hsv: CMHSVColor;
			);
		2: (
			hls: CMHLSColor;
			);
		3: (
			XYZ: CMXYZColor;
			);
		4: (
			Lab: CMLabColor;
			);
		5: (
			Luv: CMLuvColor;
			);
		6: (
			Yxy: CMYxyColor;
			);
		7: (
			cmyk: CMCMYKColor;
			);
		8: (
			cmy: CMCMYColor;
			);
		9: (
			gray: CMGrayColor;
			);
		10: (
			mc5: CMMultichannel5Color;
			);
		11: (
			mc6: CMMultichannel6Color;
			);
		12: (
			mc7: CMMultichannel7Color;
			);
		13: (
			mc8: CMMultichannel8Color;
			);
		14: (
			namedColor: CMNamedColor;
			);
	end;

{$ifc not TARGET_CPU_64}
{ GetIndexedProfile() search definition}
type
	CMProfileSearchRecordPtr = ^CMProfileSearchRecord;
	CMProfileSearchRecord = record
		header: CMHeader;
		fieldMask: UInt32;
		reserved: array [0..1] of UInt32;
	end;

	CMProfileSearchRecordHandle			= ^CMProfileSearchRecordPtr;

type
{ CMNewProfileSearch() search definition }
	CMSearchRecordPtr = ^CMSearchRecord;
	CMSearchRecord = record
		CMMType: OSType;
		profileClass: OSType;
		dataColorSpace: OSType;
		profileConnectionSpace: OSType;
		deviceManufacturer: UInt32;
		deviceModel: UInt32;
		deviceAttributes: array [0..1] of UInt32;
		profileFlags: UInt32;
		searchMask: UInt32;
		filter: CMProfileFilterUPP;
	end;
{$endc} {not TARGET_CPU_64}

{ CMMIterateUPP() structure }
type
	CMMInfoPtr = ^CMMInfo;
	CMMInfo = record
		dataSize: size_t;               { Size of this structure - compatibility}
		CMMType: OSType;                { Signature, e.g. 'appl', 'HDM ' or 'KCMS'}
		CMMMfr: OSType;                 { Vendor, e.g. 'appl'}
		CMMVersion: UInt32;             { CMM version number}
		ASCIIName: packed array [0..31] of UInt8;          { pascal string - name}
		ASCIIDesc: packed array [0..255] of UInt8;         { pascal string - description or copyright}
		UniCodeNameCount: UniCharCount;       { count of UniChars in following array}
		UniCodeName: array [0..31] of UniChar;        { the name in UniCode chars}
		UniCodeDescCount: UniCharCount;       { count of UniChars in following array}
		UniCodeDesc: array [0..255] of UniChar;       { the description in UniCode chars}
	end;

{ GetCWInfo() structures }

{$ifc not TARGET_CPU_64}

type
	CMMInfoRecordPtr = ^CMMInfoRecord;
	CMMInfoRecord = record
		CMMType: OSType;
		CMMVersion: SIGNEDLONG;
	end;

type
	CMCWInfoRecordPtr = ^CMCWInfoRecord;
	CMCWInfoRecord = record
		cmmCount: UInt32;
		cmmInfo: array [0..1] of CMMInfoRecord;
	end;

{ profile identifier structures }
type
	CMProfileIdentifierPtr = ^CMProfileIdentifier;
	CMProfileIdentifier = record
		profileHeader: CM2Header;
		calibrationDate: CMDateTime;
		ASCIIProfileDescriptionLen: UInt32;
		ASCIIProfileDescription: array [0..0] of char; { variable length }
end;
{$endc} {not TARGET_CPU_64}

{ colorspace masks }
const
	cmColorSpaceSpaceMask = $0000003F;
	cmColorSpacePremulAlphaMask = $00000040;
	cmColorSpaceAlphaMask = $00000080;
	cmColorSpaceSpaceAndAlphaMask = $000000FF;
	cmColorSpacePackingMask = $0000FF00;
	cmColorSpaceEncodingMask = $000F0000;
	cmColorSpaceReservedMask = $FFF00000;

{ packing formats }
const
	cmNoColorPacking = $0000;
	cmWord5ColorPacking = $0500;
	cmWord565ColorPacking = $0600;
	cmLong8ColorPacking = $0800;
	cmLong10ColorPacking = $0A00;
	cmAlphaFirstPacking = $1000;
	cmOneBitDirectPacking = $0B00;
	cmAlphaLastPacking = $0000;
	cm8_8ColorPacking = $2800;
	cm16_8ColorPacking = $2000;
	cm24_8ColorPacking = $2100;
	cm32_8ColorPacking = cmLong8ColorPacking;
	cm40_8ColorPacking = $2200;
	cm48_8ColorPacking = $2300;
	cm56_8ColorPacking = $2400;
	cm64_8ColorPacking = $2500;
	cm32_16ColorPacking = $2600;
	cm48_16ColorPacking = $2900;
	cm64_16ColorPacking = $2A00;
	cm32_32ColorPacking = $2700;
	cmLittleEndianPacking = $4000;
	cmReverseChannelPacking = $8000;

{ channel encoding format }
const
	cmSRGB16ChannelEncoding = $00010000; { used for sRGB64 encoding ( ±3.12 format)}

{ general colorspaces }
const
	cmNoSpace = $0000;
	cmRGBSpace = $0001;
	cmCMYKSpace = $0002;
	cmHSVSpace = $0003;
	cmHLSSpace = $0004;
	cmYXYSpace = $0005;
	cmXYZSpace = $0006;
	cmLUVSpace = $0007;
	cmLABSpace = $0008;
	cmReservedSpace1 = $0009;
	cmGraySpace = $000A;
	cmReservedSpace2 = $000B;
	cmGamutResultSpace = $000C;
	cmNamedIndexedSpace = $0010;
	cmMCFiveSpace = $0011;
	cmMCSixSpace = $0012;
	cmMCSevenSpace = $0013;
	cmMCEightSpace = $0014;
	cmAlphaPmulSpace = $0040;
	cmAlphaSpace = $0080;
	cmRGBASpace = cmRGBSpace + cmAlphaSpace;
	cmGrayASpace = cmGraySpace + cmAlphaSpace;
	cmRGBAPmulSpace = cmRGBASpace + cmAlphaPmulSpace;
	cmGrayAPmulSpace = cmGrayASpace + cmAlphaPmulSpace;

{ supported CMBitmapColorSpaces - Each of the following is a }
{ combination of a general colospace and a packing formats. }
{ Each can also be or'd with cmReverseChannelPacking. }
const
	cmGray8Space = cmGraySpace + cm8_8ColorPacking;
	cmGray16Space = cmGraySpace;
	cmGray16LSpace = cmGraySpace + cmLittleEndianPacking;
	cmGrayA16Space = cmGrayASpace + cm16_8ColorPacking;
	cmGrayA32Space = cmGrayASpace;
	cmGrayA32LSpace = cmGrayASpace + cmLittleEndianPacking;
	cmGrayA16PmulSpace = cmGrayAPmulSpace + cm16_8ColorPacking;
	cmGrayA32PmulSpace = cmGrayAPmulSpace;
	cmGrayA32LPmulSpace = cmGrayAPmulSpace + cmLittleEndianPacking;
	cmRGB16Space = cmRGBSpace + cmWord5ColorPacking;
	cmRGB16LSpace = cmRGBSpace + cmWord5ColorPacking + cmLittleEndianPacking;
	cmRGB565Space = cmRGBSpace + cmWord565ColorPacking;
	cmRGB565LSpace = cmRGBSpace + cmWord565ColorPacking + cmLittleEndianPacking;
	cmRGB24Space = cmRGBSpace + cm24_8ColorPacking;
	cmRGB32Space = cmRGBSpace + cm32_8ColorPacking;
	cmRGB48Space = cmRGBSpace + cm48_16ColorPacking;
	cmRGB48LSpace = cmRGBSpace + cm48_16ColorPacking + cmLittleEndianPacking;
	cmARGB32Space = cmRGBASpace + cm32_8ColorPacking + cmAlphaFirstPacking;
	cmARGB64Space = cmRGBASpace + cm64_16ColorPacking + cmAlphaFirstPacking;
	cmARGB64LSpace = cmRGBASpace + cm64_16ColorPacking + cmAlphaFirstPacking + cmLittleEndianPacking;
	cmRGBA32Space = cmRGBASpace + cm32_8ColorPacking + cmAlphaLastPacking;
	cmRGBA64Space = cmRGBASpace + cm64_16ColorPacking + cmAlphaLastPacking;
	cmRGBA64LSpace = cmRGBASpace + cm64_16ColorPacking + cmAlphaLastPacking + cmLittleEndianPacking;
	cmARGB32PmulSpace = cmRGBAPmulSpace + cm32_8ColorPacking + cmAlphaFirstPacking;
	cmARGB64PmulSpace = cmRGBAPmulSpace + cm64_16ColorPacking + cmAlphaFirstPacking;
	cmARGB64LPmulSpace = cmRGBAPmulSpace + cm64_16ColorPacking + cmAlphaFirstPacking + cmLittleEndianPacking;
	cmRGBA32PmulSpace = cmRGBAPmulSpace + cm32_8ColorPacking + cmAlphaLastPacking;
	cmRGBA64PmulSpace = cmRGBAPmulSpace + cm64_16ColorPacking + cmAlphaLastPacking;
	cmRGBA64LPmulSpace = cmRGBAPmulSpace + cm64_16ColorPacking + cmAlphaLastPacking + cmLittleEndianPacking;
	cmCMYK32Space = cmCMYKSpace + cm32_8ColorPacking;
	cmCMYK64Space = cmCMYKSpace + cm64_16ColorPacking;
	cmCMYK64LSpace = cmCMYKSpace + cm64_16ColorPacking + cmLittleEndianPacking;
	cmHSV32Space = cmHSVSpace + cmLong10ColorPacking;
	cmHLS32Space = cmHLSSpace + cmLong10ColorPacking;
	cmYXY32Space = cmYXYSpace + cmLong10ColorPacking;
	cmXYZ24Space = cmXYZSpace + cm24_8ColorPacking;
	cmXYZ32Space = cmXYZSpace + cmLong10ColorPacking;
	cmXYZ48Space = cmXYZSpace + cm48_16ColorPacking;
	cmXYZ48LSpace = cmXYZSpace + cm48_16ColorPacking + cmLittleEndianPacking;
	cmLUV32Space = cmLUVSpace + cmLong10ColorPacking;
	cmLAB24Space = cmLABSpace + cm24_8ColorPacking;
	cmLAB32Space = cmLABSpace + cmLong10ColorPacking;
	cmLAB48Space = cmLABSpace + cm48_16ColorPacking;
	cmLAB48LSpace = cmLABSpace + cm48_16ColorPacking + cmLittleEndianPacking;
	cmGamutResult1Space = cmOneBitDirectPacking + cmGamutResultSpace;
	cmNamedIndexed32Space = cm32_32ColorPacking + cmNamedIndexedSpace;
	cmNamedIndexed32LSpace = cm32_32ColorPacking + cmNamedIndexedSpace + cmLittleEndianPacking;
	cmMCFive8Space = cm40_8ColorPacking + cmMCFiveSpace;
	cmMCSix8Space = cm48_8ColorPacking + cmMCSixSpace;
	cmMCSeven8Space = cm56_8ColorPacking + cmMCSevenSpace;
	cmMCEight8Space = cm64_8ColorPacking + cmMCEightSpace;


type
	CMBitmapColorSpace = UInt32;

type
	CMBitmapPtr = ^CMBitmap;
	CMBitmap = record
		image: CStringPtr;
		width: size_t;
		height: size_t;
		rowBytes: size_t;
		pixelSize: size_t;
		space: CMBitmapColorSpace;
		user1: UInt32;
		user2: UInt32;
	end;


{ Profile Locations }

const
{$ifc not TARGET_CPU_64}
	CS_MAX_PATH = 256;
{$elsec}
  CS_MAX_PATH = 1024;
{$endc} {not TARGET_CPU_64}


const
	cmNoProfileBase = 0;
{$ifc not TARGET_CPU_64}
	cmFileBasedProfile = 1;
	cmHandleBasedProfile = 2;
	cmPtrBasedProfile = 3;
	cmProcedureBasedProfile = 4;
{$endc} {not TARGET_CPU_64}
	cmPathBasedProfile = 5;
	cmBufferBasedProfile = 6;


{$ifc not TARGET_CPU_64}
{ This structure is deprecated in Mac OS X 10.5. Use CMPathLocation instead.}
type
	CMFileLocationPtr = ^CMFileLocation;
	CMFileLocation = record
		spec: FSSpec;
	end;
{$endc} {not TARGET_CPU_64}

type
	CMHandleLocationPtr = ^CMHandleLocation;
	CMHandleLocation = record
		h: Handle;
	end;

{$ifc not TARGET_CPU_64}
{ This structure is deprecated in Mac OS X 10.5. Use CMBufferLocation instead.}
type
	CMPtrLocationPtr = ^CMPtrLocation;
	CMPtrLocation = record
		p: Ptr;
	end;

{ This structure is deprecated in Mac OS X 10.5.}
type
	CMProcedureLocationPtr = ^CMProcedureLocation;
	CMProcedureLocation = record
		proc: CMProfileAccessUPP;
		refCon: UnivPtr;
	end;
{$endc} {not TARGET_CPU_64}

type
	CMPathLocationPtr = ^CMPathLocation;
	CMPathLocation = record
		path: packed array [0..CS_MAX_PATH] of char;
	end;

type
	CMBufferLocationPtr = ^CMBufferLocation;
	CMBufferLocation = record
		buffer: UnivPtr;
		size: UInt32;
	end;

type
	CMProfLocPtr = ^CMProfLoc;
	CMProfLoc = record
		case SInt16 of
{$ifc not TARGET_CPU_64}
		0: (
			fileLoc: CMFileLocation;
			);
{$endc} {not TARGET_CPU_64}
		1: (
			handleLoc: CMHandleLocation;
			);
{$ifc not TARGET_CPU_64}
		2: (
			ptrLoc: CMPtrLocation;
			);
		3: (
			procLoc: CMProcedureLocation;
			);
{$endc} {not TARGET_CPU_64}
		4: (
			pathLoc: CMPathLocation;
			);
		5: (
			bufferLoc: CMBufferLocation;
			);
	end;

type
	CMProfileLocationPtr = ^CMProfileLocation;
	CMProfileLocation = record
		locType: SInt16;
		u: CMProfLoc;
	end;

const
	cmOriginalProfileLocationSize = 72;
	cmCurrentProfileLocationSize = SizeOf(CMProfileLocation);


{ Struct and enums used for Profile iteration }

const
	cmProfileIterateDataVersion1 = $00010000;
	cmProfileIterateDataVersion2 = $00020000; { Added makeAndModel}
	cmProfileIterateDataVersion3 = $00030000; { Added MD5 digest}
	cmProfileIterateDataVersion4 = $00040000;  { Only path based locations}

type
	CMProfileIterateDataPtr = ^CMProfileIterateData;
	CMProfileIterateData = record
		dataVersion: UInt32;            { cmProfileIterateDataVersion2 }
		header: CM2Header;
		code: ScriptCode;
		name: Str255;
		location: CMProfileLocation;
		uniCodeNameCount: UniCharCount;
		uniCodeName: UniCharPtr;
		asciiName: UInt8Ptr;
		makeAndModel: CMMakeAndModelPtr;
		digest: CMProfileMD5Ptr;                 { Derived from the RSA Data Security, Inc. MD5 Message-Digest Algorithm }
	end;


{ Caller-supplied callback function for Profile iteration }

type
	CMProfileIterateProcPtr = function( var iterateData: CMProfileIterateData; refCon: UnivPtr ): OSErr;

type
	CMProfileIterateUPP = CMProfileIterateProcPtr;


function NewCMProfileIterateUPP( userRoutine: CMProfileIterateProcPtr ): CMProfileIterateUPP; external name '_NewCMProfileIterateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

function InvokeCMProfileIterateUPP( var iterateData: CMProfileIterateData; refCon: UnivPtr; userUPP: CMProfileIterateUPP ): OSErr; external name '_InvokeCMProfileIterateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

procedure DisposeCMProfileIterateUPP( userUPP: CMProfileIterateUPP ); external name '_DisposeCMProfileIterateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{ Caller-supplied callback function for CMM iteration }

type
	CMMIterateProcPtr = function( var iterateData: CMMInfo; refCon: UnivPtr ): OSErr;

type
	CMMIterateUPP = CMMIterateProcPtr;

function NewCMMIterateUPP( userRoutine: CMMIterateProcPtr ): CMMIterateUPP; external name '_NewCMMIterateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

function InvokeCMMIterateUPP( var iterateData: CMMInfo; refCon: UnivPtr; userUPP: CMMIterateUPP ): OSErr; external name '_InvokeCMMIterateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

procedure DisposeCMMIterateUPP( userUPP: CMMIterateUPP ); external name '_DisposeCMMIterateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


type
	CMLabToLabProcPtr = procedure( var L: Float32; var a: Float32; var b: Float32; refcon: UnivPtr );


{ Creating Profiles }

function CMNewProfile( var prof: CMProfileRef; const (*var*) theProfile: CMProfileLocation ): CMError; external name '_CMNewProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5. Use NCWNewLinkProfile instead.}
function CWNewLinkProfile( var prof: CMProfileRef; const (*var*) targetLocation: CMProfileLocation; var profileSet: CMConcatProfileSet ): CMError; external name '_CWNewLinkProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


function NCWNewLinkProfile( var prof: CMProfileRef; const (*var*) targetLocation: CMProfileLocation; var profileSet: NCMConcatProfileSet; proc: CMConcatCallBackUPP; refCon: UnivPtr ): CMError; external name '_NCWNewLinkProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
    @function    CMMakeProfile
    @abstract    Make a display or abstract profile.
    @discussion  Adds appropriate tags to a profile to make display or abstract
                 profile based on an specification dictionary.
	One key in the specification dictionary must be "profileType" 
	which must have a CFString value of "abstractLab", "displayRGB" 
	or "displayID".  It can also contain the keys/values:
	  <PRE>
	  "description"    CFString (optional)
	  "copyright"      CFString (optional)
	  </PRE>
	For profileType of "abstractLab", the dictionary
	should also contain the keys/values:
	  <PRE>
	  "gridPoints"     CFNumber(SInt32) (should be odd)
	  "proc"           CFNumber(SInt64) (coerced from a LabToLabProcPtr)
	  "refcon"         CFNumber(SInt64) (coerced from a void*) (optional) 
	  </PRE>
	For profileType of "displayRGB", the dictionary
	should also contain the keys/values:
	  <PRE>
	  "targetGamma"    CFNumber(Float)  (e.g. 1.8)  (optional)
	  "targetWhite"    CFNumber(SInt32) (e.g. 6500) (optional)
	  "gammaR"         CFNumber(Float)  (e.g. 2.5)
	  "gammaG"         CFNumber(Float)  (e.g. 2.5)
	  "gammaB"         CFNumber(Float)  (e.g. 2.5)
	  "tableChans"     CFNumber(SInt32) (1 or 3) (optional)
	  "tableEntries"   CFNumber(SInt32) (e.g 16 or 255) (optional)
	  "tableEntrySize" CFNumber(SInt32) (1 or 2) (optional)
	  "tableData"      CFData (lut in RRRGGGBBB order) (optional)
	  
	  either
	  "phosphorRx"     CFNumber(Float)
	  "phosphorRy"     CFNumber(Float)
	  "phosphorGx"     CFNumber(Float)
	  "phosphorGy"     CFNumber(Float)
	  "phosphorBx"     CFNumber(Float)
	  "phosphorBy"     CFNumber(Float)
	  or
	  "phosphorSet"    CFString ("WideRGB", "700/525/450nm", "P22-EBU", "HDTV", 
	                             "CCIR709", "sRGB", "AdobeRGB98" or "Trinitron")
	  either
	  "whitePointx"    CFNumber(Float)
	  "whitePointy"    CFNumber(Float)
	  or
	  "whiteTemp"      CFNumber(SInt32)  (e.g. 5000, 6500, 9300)
	  </PRE>
	For profileType of "displayID", the dictionary
	should also contain the keys/values:
	  <PRE>
	  "targetGamma"    CFNumber(Float)  (e.g. 1.8)  (optional)
	  "targetWhite"    CFNumber(SInt32) (e.g. 6500) (optional)
	  "displayID       CFNumber(SInt32)
	  Optionally, the keys/values for "displayRGB" can be
	  provided to override the values from the display.
	  </PRE>
    
    @param       prof       (in) the profile to modify
    @param       spec       (in) specification dictionary
}
function CMMakeProfile( prof: CMProfileRef; spec: CFDictionaryRef ): CMError; external name '_CMMakeProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{ Accessing Profiles }

function CMOpenProfile( var prof: CMProfileRef; const (*var*) theProfile: CMProfileLocation ): CMError; external name '_CMOpenProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMCloseProfile( prof: CMProfileRef ): CMError; external name '_CMCloseProfile';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMUpdateProfile( prof: CMProfileRef ): CMError; external name '_CMUpdateProfile';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMCopyProfile( var targetProf: CMProfileRef; const (*var*) targetLocation: CMProfileLocation; srcProf: CMProfileRef ): CMError; external name '_CMCopyProfile';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMValidateProfile( prof: CMProfileRef; var valid: Boolean; var preferredCMMnotfound: Boolean ): CMError; external name '_CMValidateProfile';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMCloseProfile( prof: CMProfileRef ): CMError; external name '_CMCloseProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMUpdateProfile( prof: CMProfileRef ): CMError; external name '_CMUpdateProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMCopyProfile( var targetProf: CMProfileRef; const (*var*) targetLocation: CMProfileLocation; srcProf: CMProfileRef ): CMError; external name '_CMCopyProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMValidateProfile( prof: CMProfileRef; var valid: Boolean; var preferredCMMnotfound: Boolean ): CMError; external name '_CMValidateProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5. Use NCMGetProfileLocation instead.}
function CMGetProfileLocation( prof: CMProfileRef; var location: CMProfileLocation ): CMError; external name '_CMGetProfileLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


function NCMGetProfileLocation( prof: CMProfileRef; var theProfile: CMProfileLocation; var locationSize: UInt32 ): CMError; external name '_NCMGetProfileLocation';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
    @function    CMProfileCopyICCData
    @abstract    Return a copy of the icc data specified by `prof'.
    @param       allocator  (in) The object to be used to allocate memory for the data
    @param       prof       (in) The profile to query
 }
function CMProfileCopyICCData( allocator: CFAllocatorRef; prof: CMProfileRef ): CFDataRef; external name '_CMProfileCopyICCData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5. Use CMCopyProfile instead.}
function CMFlattenProfile( prof: CMProfileRef; flags: UInt32; proc: CMFlattenUPP; refCon: UnivPtr; var preferredCMMnotfound: Boolean ): CMError; external name '_CMFlattenProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5.}
function NCMUnflattenProfile( var targetLocation: CMProfileLocation; proc: CMFlattenUPP; refCon: UnivPtr; var preferredCMMnotfound: Boolean ): CMError; external name '_NCMUnflattenProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


function CMGetProfileHeader( prof: CMProfileRef; var header: CMAppleProfileHeader ): CMError; external name '_CMGetProfileHeader';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMSetProfileHeader( prof: CMProfileRef; const (*var*) header: CMAppleProfileHeader ): CMError; external name '_CMSetProfileHeader';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMCloneProfileRef( prof: CMProfileRef ): CMError; external name '_CMCloneProfileRef';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetProfileRefCount( prof: CMProfileRef; var count: SIGNEDLONG ): CMError; external name '_CMGetProfileRefCount';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMProfileModified( prof: CMProfileRef; var modified: Boolean ): CMError; external name '_CMProfileModified';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetProfileMD5( prof: CMProfileRef; digest: CMProfileMD5 ): CMError; external name '_CMGetProfileMD5';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMSetProfileHeader( prof: CMProfileRef; const (*var*) header: CMAppleProfileHeader ): CMError; external name '_CMSetProfileHeader';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMCloneProfileRef( prof: CMProfileRef ): CMError; external name '_CMCloneProfileRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetProfileRefCount( prof: CMProfileRef; var count: SIGNEDLONG ): CMError; external name '_CMGetProfileRefCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMProfileModified( prof: CMProfileRef; var modified: Boolean ): CMError; external name '_CMProfileModified';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetProfileMD5( prof: CMProfileRef; digest: CMProfileMD5 ): CMError; external name '_CMGetProfileMD5';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4


{ Accessing Profile Elements }

function CMCountProfileElements( prof: CMProfileRef; var elementCount: UInt32 ): CMError; external name '_CMCountProfileElements';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMProfileElementExists( prof: CMProfileRef; tag: OSType; var found: Boolean ): CMError; external name '_CMProfileElementExists';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetProfileElement( prof: CMProfileRef; tag: OSType; var elementSize: UInt32; elementData: UnivPtr ): CMError; external name '_CMGetProfileElement';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMSetProfileElement( prof: CMProfileRef; tag: OSType; elementSize: UInt32; elementData: {const} UnivPtr ): CMError; external name '_CMSetProfileElement';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMSetProfileElementSize( prof: CMProfileRef; tag: OSType; elementSize: UInt32 ): CMError; external name '_CMSetProfileElementSize';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMSetProfileElementReference( prof: CMProfileRef; elementTag: OSType; referenceTag: OSType ): CMError; external name '_CMSetProfileElementReference';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetPartialProfileElement( prof: CMProfileRef; tag: OSType; offset: UInt32; var byteCount: UInt32; elementData: UnivPtr ): CMError; external name '_CMGetPartialProfileElement';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMSetPartialProfileElement( prof: CMProfileRef; tag: OSType; offset: UInt32; byteCount: UInt32; elementData: {const} UnivPtr ): CMError; external name '_CMSetPartialProfileElement';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetIndProfileElementInfo( prof: CMProfileRef; index: UInt32; var tag: OSType; var elementSize: UInt32; var refs: Boolean ): CMError; external name '_CMGetIndProfileElementInfo';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetIndProfileElement( prof: CMProfileRef; index: UInt32; var elementSize: UInt32; elementData: UnivPtr ): CMError; external name '_CMGetIndProfileElement';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMRemoveProfileElement( prof: CMProfileRef; tag: OSType ): CMError; external name '_CMRemoveProfileElement';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMProfileElementExists( prof: CMProfileRef; tag: OSType; var found: Boolean ): CMError; external name '_CMProfileElementExists';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetProfileElement( prof: CMProfileRef; tag: OSType; var elementSize: UInt32; elementData: UnivPtr ): CMError; external name '_CMGetProfileElement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMSetProfileElement( prof: CMProfileRef; tag: OSType; elementSize: UInt32; elementData: {const} UnivPtr ): CMError; external name '_CMSetProfileElement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMSetProfileElementSize( prof: CMProfileRef; tag: OSType; elementSize: UInt32 ): CMError; external name '_CMSetProfileElementSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMSetProfileElementReference( prof: CMProfileRef; elementTag: OSType; referenceTag: OSType ): CMError; external name '_CMSetProfileElementReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetPartialProfileElement( prof: CMProfileRef; tag: OSType; offset: UInt32; var byteCount: UInt32; elementData: UnivPtr ): CMError; external name '_CMGetPartialProfileElement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMSetPartialProfileElement( prof: CMProfileRef; tag: OSType; offset: UInt32; byteCount: UInt32; elementData: {const} UnivPtr ): CMError; external name '_CMSetPartialProfileElement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetIndProfileElementInfo( prof: CMProfileRef; index: UInt32; var tag: OSType; var elementSize: UInt32; var refs: Boolean ): CMError; external name '_CMGetIndProfileElementInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetIndProfileElement( prof: CMProfileRef; index: UInt32; var elementSize: UInt32; elementData: UnivPtr ): CMError; external name '_CMGetIndProfileElement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMRemoveProfileElement( prof: CMProfileRef; tag: OSType ): CMError; external name '_CMRemoveProfileElement';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4


{ Accessing Profile Descriptions }

{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5. Use CMCopyProfileDescriptionString instead.}
function CMGetScriptProfileDescription( prof: CMProfileRef; var name: Str255; var code: ScriptCode ): CMError; external name '_CMGetScriptProfileDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


function CMGetProfileDescriptions( prof: CMProfileRef; aName: CStringPtr; var aCount: UInt32; var mName: Str255; var mCode: ScriptCode; var uName: UniChar; var uCount: UniCharCount ): CMError; external name '_CMGetProfileDescriptions';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMSetProfileDescriptions( prof: CMProfileRef; aName: ConstCStringPtr; aCount: UInt32; const (*var*) mName: Str255; mCode: ScriptCode; uName: ConstUniCharPtr; uCount: UniCharCount ): CMError; external name '_CMSetProfileDescriptions';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMCopyProfileLocalizedStringDictionary( prof: CMProfileRef; tag: OSType; var theDict: CFDictionaryRef ): CMError; external name '_CMCopyProfileLocalizedStringDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


function CMSetProfileLocalizedStringDictionary( prof: CMProfileRef; tag: OSType; theDict: CFDictionaryRef ): CMError; external name '_CMSetProfileLocalizedStringDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

function CMCopyProfileLocalizedString( prof: CMProfileRef; tag: OSType; reqLocale: CFStringRef; var locale: CFStringRef; var str: CFStringRef ): CMError; external name '_CMCopyProfileLocalizedString';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMSetProfileDescriptions( prof: CMProfileRef; aName: ConstCStringPtr; aCount: UInt32; const (*var*) mName: Str255; mCode: ScriptCode; uName: ConstUniCharPtr; uCount: UniCharCount ): CMError; external name '_CMSetProfileDescriptions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMCopyProfileLocalizedStringDictionary( prof: CMProfileRef; tag: OSType; var theDict: CFDictionaryRef ): CMError; external name '_CMCopyProfileLocalizedStringDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


function CMSetProfileLocalizedStringDictionary( prof: CMProfileRef; tag: OSType; theDict: CFDictionaryRef ): CMError; external name '_CMSetProfileLocalizedStringDictionary';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


function CMCopyProfileLocalizedString( prof: CMProfileRef; tag: OSType; reqLocale: CFStringRef; var locale: CFStringRef; var str: CFStringRef ): CMError; external name '_CMCopyProfileLocalizedString';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4

{!
    @function    CMCopyProfileDescriptionString
    @abstract    Returns the name of a profile as a CFString.
    @discussion  If the profile is multi-localized, the best localized name for the current process is returned.
    @param       prof       (in) the profile to query
    @param       str        (out) returns the name
}
function CMCopyProfileDescriptionString( prof: CMProfileRef; var str: CFStringRef ): CMError; external name '_CMCopyProfileDescriptionString';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ Accessing Name-Class Profiles }

function CMGetNamedColorInfo( prof: CMProfileRef; var deviceChannels: UInt32; var deviceColorSpace: OSType; var PCSColorSpace: OSType; var count: UInt32; prefix: StringPtr; suffix: StringPtr ): CMError; external name '_CMGetNamedColorInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetNamedColorValue( prof: CMProfileRef; name: StringPtr; var deviceColor: CMColor; var PCSColor: CMColor ): CMError; external name '_CMGetNamedColorValue';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetIndNamedColorValue( prof: CMProfileRef; index: UInt32; var deviceColor: CMColor; var PCSColor: CMColor ): CMError; external name '_CMGetIndNamedColorValue';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetNamedColorIndex( prof: CMProfileRef; name: StringPtr; var index: UInt32 ): CMError; external name '_CMGetNamedColorIndex';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetNamedColorName( prof: CMProfileRef; index: UInt32; name: StringPtr ): CMError; external name '_CMGetNamedColorName';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetNamedColorValue( prof: CMProfileRef; name: StringPtr; var deviceColor: CMColor; var PCSColor: CMColor ): CMError; external name '_CMGetNamedColorValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetIndNamedColorValue( prof: CMProfileRef; index: UInt32; var deviceColor: CMColor; var PCSColor: CMColor ): CMError; external name '_CMGetIndNamedColorValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetNamedColorIndex( prof: CMProfileRef; name: StringPtr; var index: UInt32 ): CMError; external name '_CMGetNamedColorIndex';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetNamedColorName( prof: CMProfileRef; index: UInt32; name: StringPtr ): CMError; external name '_CMGetNamedColorName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4


{ Working with ColorWorlds }

function NCWNewColorWorld( var cw: CMWorldRef; src: CMProfileRef; dst: CMProfileRef ): CMError; external name '_NCWNewColorWorld';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
function CWConcatColorWorld( var cw: CMWorldRef; var profileSet: CMConcatProfileSet ): CMError; external name '_CWConcatColorWorld';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4
{$endc} {not TARGET_CPU_64}


function NCWConcatColorWorld( var cw: CMWorldRef; var profileSet: NCMConcatProfileSet; proc: CMConcatCallBackUPP; refCon: UnivPtr ): CMError; external name '_NCWConcatColorWorld';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5.}
function CMGetCWInfo( cw: CMWorldRef; var info: CMCWInfoRecord ): CMError; external name '_CMGetCWInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


procedure CWDisposeColorWorld( cw: CMWorldRef ); external name '_CWDisposeColorWorld';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CWMatchColors( cw: CMWorldRef; var myColors: CMColor; count: size_t ): CMError; external name '_CWMatchColors';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CWCheckColors( cw: CMWorldRef; var myColors: CMColor; count: size_t; var result: UInt8 ): CMError; external name '_CWCheckColors';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CWMatchColors( cw: CMWorldRef; var myColors: CMColor; count: size_t ): CMError; external name '_CWMatchColors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CWCheckColors( cw: CMWorldRef; var myColors: CMColor; count: size_t; var result: UInt8 ): CMError; external name '_CWCheckColors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4

{$ifc not _DECLARE_CS_QD_API_}

function CWMatchBitmap( cw: CMWorldRef; var bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var matchedBitmap: CMBitmap ): CMError; external name '_CWMatchBitmap';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CWCheckBitmap( cw: CMWorldRef; const (*var*) bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitmap: CMBitmap ): CMError; external name '_CWCheckBitmap';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CWCheckBitmap( cw: CMWorldRef; const (*var*) bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitmap: CMBitmap ): CMError; external name '_CWCheckBitmap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4

{$endc} {not _DECLARE_CS_QD_API_}

function CWGetCMMSignature( cw: CMWorldRef ): UInt32; external name '_CWGetCMMSignature';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

{ OpenGL support }
const
	cmTextureRGBtoRGBX8 = 0;    { RGB to  8-bit RGBx texture}
	cmTextureRGBtoRGBX16 = 1;    { RGB to 16-bit RGBx texture}
	cmTextureRGBtoRGBXFloat32 = 2;     { RGB to 32-bit float RGBx texture }

{!
    @function    CWFillLookupTexture
    @abstract    Fills a 3d lookup texture from a colorworld.
    @discussion  The resulting table is suitable for use in OpenGL to 
                 accelerate color management in hardware.
    @param       cw             (in) the colorworld to use
    @param       gridPoints     (in) number of grid points per channel in the texture
    @param       format         (in) format of pixels in texture (e.g. cmTextureRGBtoRGBX8)
    @param       dataSize       (in) size in bytes of texture data to fill
    @param       data           (in/out) pointer to texture data to fill
}
function CWFillLookupTexture( cw: CMWorldRef; gridPoints: UInt32; format: UInt32; dataSize: UInt32; data: UnivPtr ): CMError; external name '_CWFillLookupTexture';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{ Accessing Special Profiles }

function CMGetSystemProfile( var prof: CMProfileRef ): CMError; external name '_CMGetSystemProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5. Use CMSetProfileByAVID instead.}
function CMSetSystemProfile( const (*var*) profileFileSpec: FSSpec ): CMError; external name '_CMSetSystemProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5. Use CMSetProfileByAVID instead.}
function NCMSetSystemProfile( const (*var*) profLoc: CMProfileLocation ): CMError; external name '_NCMSetSystemProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


function CMGetDefaultProfileBySpace( dataColorSpace: OSType; var prof: CMProfileRef ): CMError; external name '_CMGetDefaultProfileBySpace';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5.}
function CMSetDefaultProfileBySpace( dataColorSpace: OSType; prof: CMProfileRef ): CMError; external name '_CMSetDefaultProfileBySpace';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


function CMGetDefaultProfileByUse( use: OSType; var prof: CMProfileRef ): CMError; external name '_CMGetDefaultProfileByUse';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}
{ This function is deprecated in Mac OS X 10.5.}
function CMSetDefaultProfileByUse( use: OSType; prof: CMProfileRef ): CMError; external name '_CMSetDefaultProfileByUse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


function CMGetProfileByAVID( theID: CMDisplayIDType; var prof: CMProfileRef ): CMError; external name '_CMGetProfileByAVID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMSetProfileByAVID( theID: CMDisplayIDType; prof: CMProfileRef ): CMError; external name '_CMSetProfileByAVID';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetGammaByAVID( theID: CMDisplayIDType; var gamma: CMVideoCardGamma; var size: UInt32 ): CMError; external name '_CMGetGammaByAVID';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMSetGammaByAVID( theID: CMDisplayIDType; var gamma: CMVideoCardGamma ): CMError; external name '_CMSetGammaByAVID';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMSetProfileByAVID( theID: CMDisplayIDType; prof: CMProfileRef ): CMError; external name '_CMSetProfileByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetGammaByAVID( theID: CMDisplayIDType; var gamma: CMVideoCardGamma; var size: UInt32 ): CMError; external name '_CMGetGammaByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMSetGammaByAVID( theID: CMDisplayIDType; var gamma: CMVideoCardGamma ): CMError; external name '_CMSetGammaByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4


{ Searching for Profiles }

function CMIterateColorSyncFolder( proc: CMProfileIterateUPP; var seed: UInt32; var count: UInt32; refCon: UnivPtr ): CMError; external name '_CMIterateColorSyncFolder';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$ifc not TARGET_CPU_64}

{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
function CMGetColorSyncFolderSpec( vRefNum: SInt16; createFolder: Boolean; var foundVRefNum: SInt16; var foundDirID: SIGNEDLONG ): CMError; external name '_CMGetColorSyncFolderSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
function CMNewProfileSearch( var searchSpec: CMSearchRecord; refCon: UnivPtr; var count: UInt32; var searchResult: CMProfileSearchRef ): CMError; external name '_CMNewProfileSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
function CMUpdateProfileSearch( search: CMProfileSearchRef; refCon: UnivPtr; var count: UInt32 ): CMError; external name '_CMUpdateProfileSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
procedure CMDisposeProfileSearch( search: CMProfileSearchRef ); external name '_CMDisposeProfileSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
function CMSearchGetIndProfile( search: CMProfileSearchRef; index: UInt32; var prof: CMProfileRef ): CMError; external name '_CMSearchGetIndProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
function CMSearchGetIndProfileFileSpec( search: CMProfileSearchRef; index: UInt32; var spec: FSSpec ): CMError; external name '_CMSearchGetIndProfileFileSpec';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
function CMCreateProfileIdentifier( prof: CMProfileRef; ident: CMProfileIdentifierPtr; var size: UInt32 ): CMError; external name '_CMCreateProfileIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
function CMProfileIdentifierFolderSearch( ident: CMProfileIdentifierPtr; var matchedCount: UInt32; var searchResult: CMProfileSearchRef ): CMError; external name '_CMProfileIdentifierFolderSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMIterateColorSyncFolder instead.}
function CMProfileIdentifierListSearch( ident: CMProfileIdentifierPtr; var profileList: CMProfileRef; listSize: UInt32; var matchedCount: UInt32; var matchedList: CMProfileRef ): CMError; external name '_CMProfileIdentifierListSearch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{$endc} {not TARGET_CPU_64}


{ Utilities }

{$ifc not TARGET_CPU_64}

function CMGetPreferredCMM( var cmmType: OSType; var prefCMMnotfound: Boolean ): CMError; external name '_CMGetPreferredCMM';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMSetPreferredCMM( cmmType: OSType ): CMError; external name '_CMSetPreferredCMM';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{$endc} {not TARGET_CPU_64}

function CMIterateCMMInfo( proc: CMMIterateUPP; var count: UInt32; refCon: UnivPtr ): CMError; external name '_CMIterateCMMInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetColorSyncVersion( var version: UInt32 ): CMError; external name '_CMGetColorSyncVersion';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMLaunchControlPanel( flags: UInt32 ): CMError; external name '_CMLaunchControlPanel';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetColorSyncVersion( var version: UInt32 ): CMError; external name '_CMGetColorSyncVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMLaunchControlPanel( flags: UInt32 ): CMError; external name '_CMLaunchControlPanel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4


{ Converting Colors }

{$ifc not TARGET_CPU_64}

{ This function is deprecated in Mac OS X 10.5. Use CMConvertXYZFloatBitmap instead.}
function CMConvertXYZToLab( const (*var*) src: CMColor; const (*var*) white: CMXYZColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertXYZToLab';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertXYZFloatBitmap instead.}
function CMConvertLabToXYZ( const (*var*) src: CMColor; const (*var*) white: CMXYZColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertLabToXYZ';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertXYZFloatBitmap instead.}
function CMConvertXYZToLuv( const (*var*) src: CMColor; const (*var*) white: CMXYZColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertXYZToLuv';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertXYZFloatBitmap instead.}
function CMConvertLuvToXYZ( const (*var*) src: CMColor; const (*var*) white: CMXYZColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertLuvToXYZ';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertXYZFloatBitmap instead.}
function CMConvertXYZToYxy( const (*var*) src: CMColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertXYZToYxy';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertXYZFloatBitmap instead.}
function CMConvertYxyToXYZ( const (*var*) src: CMColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertYxyToXYZ';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertRGBFloatBitmap instead.}
function CMConvertRGBToHLS( const (*var*) src: CMColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertRGBToHLS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertRGBFloatBitmap instead.}
function CMConvertHLSToRGB( const (*var*) src: CMColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertHLSToRGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertRGBFloatBitmap instead.}
function CMConvertRGBToHSV( const (*var*) src: CMColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertRGBToHSV';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertRGBFloatBitmap instead.}
function CMConvertHSVToRGB( const (*var*) src: CMColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertHSVToRGB';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertRGBFloatBitmap instead.}
function CMConvertRGBToGray( const (*var*) src: CMColor; var dst: CMColor; count: size_t ): CMError; external name '_CMConvertRGBToGray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5.}
function CMConvertXYZToFixedXYZ( const (*var*) src: CMXYZColor; var dst: CMFixedXYZColor; count: size_t ): CMError; external name '_CMConvertXYZToFixedXYZ';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5.}
function CMConvertFixedXYZToXYZ( const (*var*) src: CMFixedXYZColor; var dst: CMXYZColor; count: size_t ): CMError; external name '_CMConvertFixedXYZToXYZ';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ This function is deprecated in Mac OS X 10.5. Use CMConvertXYZFloatBitmap instead.}
function CMConvertXYZToXYZ( const (*var*) src: CMColor; const (*var*) srcIlluminant: CMXYZColor; var dst: CMColor; const (*var*) dstIlluminant: CMXYZColor; method: CMChromaticAdaptation; count: size_t ): CMError; external name '_CMConvertXYZToXYZ';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{$endc} {not TARGET_CPU_64}


{ Working with PostScript }

function CMGetPS2ColorSpace( srcProf: CMProfileRef; flags: UInt32; proc: CMFlattenUPP; refCon: UnivPtr; var preferredCMMnotfound: Boolean ): CMError; external name '_CMGetPS2ColorSpace';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetPS2ColorRenderingIntent( srcProf: CMProfileRef; flags: UInt32; proc: CMFlattenUPP; refCon: UnivPtr; var preferredCMMnotfound: Boolean ): CMError; external name '_CMGetPS2ColorRenderingIntent';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetPS2ColorRendering( srcProf: CMProfileRef; dstProf: CMProfileRef; flags: UInt32; proc: CMFlattenUPP; refCon: UnivPtr; var preferredCMMnotfound: Boolean ): CMError; external name '_CMGetPS2ColorRendering';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)


function CMGetPS2ColorRenderingVMSize( srcProf: CMProfileRef; dstProf: CMProfileRef; var vmSize: UInt32; var preferredCMMnotfound: Boolean ): CMError; external name '_CMGetPS2ColorRenderingVMSize';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetPS2ColorRenderingIntent( srcProf: CMProfileRef; flags: UInt32; proc: CMFlattenUPP; refCon: UnivPtr; var preferredCMMnotfound: Boolean ): CMError; external name '_CMGetPS2ColorRenderingIntent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetPS2ColorRendering( srcProf: CMProfileRef; dstProf: CMProfileRef; flags: UInt32; proc: CMFlattenUPP; refCon: UnivPtr; var preferredCMMnotfound: Boolean ): CMError; external name '_CMGetPS2ColorRendering';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function CMGetPS2ColorRenderingVMSize( srcProf: CMProfileRef; dstProf: CMProfileRef; var vmSize: UInt32; var preferredCMMnotfound: Boolean ): CMError; external name '_CMGetPS2ColorRenderingVMSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4


{ Notifications }

{
 *  Clients can register for notifications of ColorSync preference changes by
 *  using the kCMPrefsChangedNotification key. This notification will be sent
 *  if the user changes ColorSync preferences such as:
 *      the default profile by colors space, (CMSetDefaultProfileBySpace)
 *      the default profile by device useage, (CMSetDefaultProfileByUse)
 *      or the preferred CMM.
 *  See <CMDeviceIntegration.h> for more notifications that can be sent.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMPrefsChangedNotification CFSTRP('AppleColorSyncPreferencesChangedNotification')}
{$endc}


//#pragma mark --- CMFloatBitmap.h ----

{$ifc not TARGET_CPU_64}
{$ALIGN POWER}
{$endc} {not TARGET_CPU_64}


type
	CMFloatBitmapFlags = SInt32;
const
	kCMFloatBitmapFlagsNone = 0;
	kCMFloatBitmapFlagsAlpha = 1;
	kCMFloatBitmapFlagsAlphaPremul = 2;
	kCMFloatBitmapFlagsRangeClipped = 4;

{!
    @struct     CMFloatBitmap
    @abstract       A new struture that defines and arbritrary map of float color values.
    @discussion     The struture defines a pixel array of dimensions [height][width][chans] 
                    where 'chans' is the number of channels in the colorspace plus an optional one for alpha.
                    The actual memory pointed to by the structure can contain a variety of possible arrangements. 
                    The actual data values can be chuncky or planar. The channels can by in any order.
<PRE>

Examples:
a) float* p contains a 640w by 480h bitmap of chunky RGB data
    CMFloatBitmap map = ( 0,         // version
                (p, p+1, p+2),       // base addrs of R,G,B
                480, 640,            // height, width
                640*3,               // rowStride
                3,                   // colStride
                cmRGBData,
                kCMFloatBitmapFlagsNone);
b) float* p contains a 640w by 480h bitmap of chunky BGRA data
    CMFloatBitmap map = ( 0,         // version
                (p+2, p+1, p, p+3),  // base addrs of R,G,B,A
                480, 640,            // height, width
                640*4,               // rowStride
                3,                   // colStride
                cmRGBData,
                kCMFloatBitmapFlagsAlpha);
c) float* p contains a 640w by 480h bitmap of planar CMYK data
    CMFloatBitmap map = ( 0,        // version
                (p, p+640*480 , p+2*640*480 , p+3*640*480),
                480, 640,           // height, width
                640,                // rowStride
                1,                  // colStride
                cmCMYKData,
                kCMFloatBitmapFlagsNone);
</PRE>
        
    @field      version     The version number of this structure to allow for future expansion.
                            Should contain 0 for now.
    
    @field      buffers     The base address for each channel in canonical order.
                            The canonical order for RGB is R,G,B. CMYK is C,M,Y,K etc.
                            A maximum of sixteen channels is supported.
                            Another way to think of this is 
                                buffers[c] = &(pixelArray[0][0][c])
                                
    @field      height      The height (in pixels) of the bitmap.

    @field      width       The width (in pixels) of the bitmap.

    @field      rowStride   The number of floats to skip to move from one row to the next.
                            This is typically (width*chans) for chunky pixel buffers or (width) for planar.
                            Can be negative if the image is vertically flipped.

    @field      colStride   The number of floats to skip to move from one column to the next.
                            This is typically (chans) for chunky pixel buffers or (1) for planar.
                            Can be negative if the image is horizontally flipped.

    @field      space       The colorspace of the data (e.g cmRGBdata cmCMYKData)

    @field      flags       Holds bits to specify the alpha type of the data.
                            The remaining bits are reserved for future use.

}
type
	CMFloatBitmapPtr = ^CMFloatBitmap;
	CMFloatBitmap = record
		version: UNSIGNEDLONG;
		buffers: array [0..15] of Float32Ptr;
		height: size_t;
		width: size_t;
		rowStride: SIGNEDLONG;
		colStride: SIGNEDLONG;
		space: OSType;
		flags: CMFloatBitmapFlags;
	end;
 
type
	IlluminantArray = array [0..2] of Float32;
 
{ D50 illuminant (0.9642, 1.0000, 0.8249) }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCMIlluminantD50: IlluminantArray; external name '_kCMIlluminantD50'; (* attribute const *)                                  (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
var kCMIlluminantD50: IlluminantArray; external name '_kCMIlluminantD50'; (* attribute const *)                                  (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
var kCMIlluminantD50: IlluminantArray; external name '_kCMIlluminantD50'; (* attribute const *)                                  (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
var kCMIlluminantD50: IlluminantArray; external name '_kCMIlluminantD50'; (* attribute const *)                                  (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
var kCMIlluminantD50: IlluminantArray; external name '_kCMIlluminantD50'; (* attribute const *)                                  (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
var kCMIlluminantD50: IlluminantArray; external name '_kCMIlluminantD50'; (* attribute const *)                                  (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4
 
 
{ D65 illuminant (0.9504, 1.0000, 1.0889)}
var kCMIlluminantD65: IlluminantArray; external name '_kCMIlluminantD65'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4
 
 

{!
    @function   CMFloatBitmapMakeChunky
    @abstract   A handy funtion to fill in a CMFloatBitmap.
    @discussion Returns a filled in CMFloatBitmap structure given a single buffer of chunky data with no alpha.
    @param      buffer  (in) address of interleaved data
    @param      height  (in) height of bitmap in pixels
    @param      width   (in) width of bitmap in pixels
    @param      space   (in) colorsapce of the data
    @result     a filled in CMFloatBitmap
}
function CMFloatBitmapMakeChunky( var buffer: Float32; height: size_t; width: size_t; space: OSType ): CMFloatBitmap; external name '_CMFloatBitmapMakeChunky';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
    @function   CMConvertXYZFloatBitmap
    @abstract   Used to convert CMFloatBitmaps between the related colorspaces XYZ, Yxy, Lab, and Luv.
    @discussion The buffer data from the source CMFloatBitmap is converted into the buffer data
                specified the destination CMFloatBitmap.  Converion "in-place" is allowed.
    @param      src     (in) description of source data buffer to convert from
    @param      srcIlluminantXYZ    (in) required if src->space is XYZ or Yxy
    @param      dst     (in,out) description of destination data buffer to convert to
    @param      dstIlluminantXYZ    (in) required if dst->space is XYZ or Yxy
    @param      method  (in) the chromatic adaptation method to use
}
function CMConvertXYZFloatBitmap( const (*var*) src: CMFloatBitmap; const (*var*) srcIlluminantXYZ: IlluminantArray; var dst: CMFloatBitmap; const (*var*) dstIlluminantXYZ: IlluminantArray; method: CMChromaticAdaptation ): CMError; external name '_CMConvertXYZFloatBitmap';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
    @function   CMConvertRGBFloatBitmap
    @abstract   Used to convert CMFloatBitmaps between the related colorspaces RGB, HSV, and HLS.
    @discussion The buffer data from the source CMFloatBitmap is converted into the buffer data
                specified the destination CMFloatBitmap.  Converion "in-place" is allowed.
    @param      src     (in) description of source data buffer to convert from
    @param      dst     (in,out) description of destination data buffer to convert to
}
function CMConvertRGBFloatBitmap( const (*var*) src: CMFloatBitmap; var dst: CMFloatBitmap ): CMError; external name '_CMConvertRGBFloatBitmap';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
    @function   CMMatchFloatBitmap
    @abstract   Used to convert CMFloatBitmaps using a CMWorldRef.
    @discussion The buffer data from the source CMFloatBitmap is converted into the buffer data
                specified the destination CMFloatBitmap.  Converion "in-place" is allowed.
    @param      cw      (in) the CMWorldRef to convert with
    @param      src     (in) description of source data buffer to convert from
    @param      dst     (in,out) description of destination data buffer to convert to
}
function CMMatchFloatBitmap( cw: CMWorldRef; const (*var*) src: CMFloatBitmap; var dst: CMFloatBitmap ): CMError; external name '_CMMatchFloatBitmap';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4


// #pragma mark --- CMMComponent.h ----


procedure CWColorWorldSetProperty( cw: CMWorldRef; key: CFStringRef; value: CFTypeRef ); external name '_CWColorWorldSetProperty';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


function CWColorWorldGetProperty( cw: CMWorldRef; key: CFStringRef ): UnivPtr; external name '_CWColorWorldGetProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


function CWColorWorldGetProperty( cw: CMWorldRef; key: CFStringRef ): UnivPtr; external name '_CWColorWorldGetProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4

{
   The following declarations specify the calling conventions for CMM entry-points on Mac OS X.
}

{ Required }
function CMM_ConcatColorWorld( cw: CMWorldRef; var profileSet: NCMConcatProfileSet; proc: CMConcatCallBackUPP; refCon: UnivPtr ): CMError; external name '_CMM_ConcatColorWorld';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)

function CMM_MatchColors( cw: CMWorldRef; var colors: CMColor; count: UInt32 ): CMError; external name '_CMM_MatchColors';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)

function CMM_CheckColors( cw: CMWorldRef; var colors: CMColor; count: UInt32; var result: UInt8 ): CMError; external name '_CMM_CheckColors';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew

function CMM_MatchColors( cw: CMWorldRef; var colors: CMColor; count: UInt32 ): CMError; external name '_CMM_MatchColors';

function CMM_CheckColors( cw: CMWorldRef; var colors: CMColor; count: UInt32; var result: UInt8 ): CMError; external name '_CMM_CheckColors';
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

function CMM_MatchColors( cw: CMWorldRef; var colors: CMColor; count: UInt32 ): CMError; external name '_CMM_MatchColors';

function CMM_CheckColors( cw: CMWorldRef; var colors: CMColor; count: UInt32; var result: UInt8 ): CMError; external name '_CMM_CheckColors';
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2.4

function CMM_MatchColors( cw: CMWorldRef; var colors: CMColor; count: UInt32 ): CMError; external name '_CMM_MatchColors';

function CMM_CheckColors( cw: CMWorldRef; var colors: CMColor; count: UInt32; var result: UInt8 ): CMError; external name '_CMM_CheckColors';
<<<<<<< HEAD
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

{ Optional }

function CMM_ValidateProfile( prof: CMProfileRef; var valid: Boolean ): CMError; external name '_CMM_ValidateProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
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

{$ifc not _DECLARE_CS_QD_API_}

function CMM_MatchBitmap( cw: CMWorldRef; var bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var matchedBitmap: CMBitmap ): CMError; external name '_CMM_MatchBitmap';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)

function CMM_CheckBitmap( cw: CMWorldRef; const (*var*) bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitmap: CMBitmap ): CMError; external name '_CMM_CheckBitmap';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======

function CMM_CheckBitmap( cw: CMWorldRef; const (*var*) bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitmap: CMBitmap ): CMError; external name '_CMM_CheckBitmap';
>>>>>>> graemeg/cpstrnew
=======

function CMM_CheckBitmap( cw: CMWorldRef; const (*var*) bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitmap: CMBitmap ): CMError; external name '_CMM_CheckBitmap';
>>>>>>> graemeg/cpstrnew
=======

function CMM_CheckBitmap( cw: CMWorldRef; const (*var*) bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitmap: CMBitmap ): CMError; external name '_CMM_CheckBitmap';
>>>>>>> graemeg/cpstrnew
=======

function CMM_CheckBitmap( cw: CMWorldRef; const (*var*) bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitmap: CMBitmap ): CMError; external name '_CMM_CheckBitmap';
>>>>>>> origin/cpstrnew
=======

function CMM_CheckBitmap( cw: CMWorldRef; const (*var*) bitmap: CMBitmap; progressProc: CMBitmapCallBackUPP; refCon: UnivPtr; var resultBitmap: CMBitmap ): CMError; external name '_CMM_CheckBitmap';
>>>>>>> origin/fixes_2.4

{$endc} {not _DECLARE_CS_QD_API_}

function CMM_MatchFloatBitmap( cw: CMWorldRef; const (*var*) bitmap: CMFloatBitmap; var resultBitmap: CMFloatBitmap ): CMError; external name '_CMM_MatchFloatBitmap';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)

function CMM_CreateLinkProfile( prof: CMProfileRef; var profileSet: NCMConcatProfileSet; proc: CMConcatCallBackUPP; refCon: UnivPtr ): CMError; external name '_CMM_CreateLinkProfile';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)

function CMM_GetProperty( cw: CMWorldRef; requestedKey: CFStringRef ): CFTypeRef; external name '_CMM_GetProperty';
(* DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/fixes_2.4

function CMM_CreateLinkProfile( prof: CMProfileRef; var profileSet: NCMConcatProfileSet; proc: CMConcatCallBackUPP; refCon: UnivPtr ): CMError; external name '_CMM_CreateLinkProfile';

function CMM_GetProperty( cw: CMWorldRef; requestedKey: CFStringRef ): CFTypeRef; external name '_CMM_GetProperty';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

function CMM_CreateLinkProfile( prof: CMProfileRef; var profileSet: NCMConcatProfileSet; proc: CMConcatCallBackUPP; refCon: UnivPtr ): CMError; external name '_CMM_CreateLinkProfile';

function CMM_GetProperty( cw: CMWorldRef; requestedKey: CFStringRef ): CFTypeRef; external name '_CMM_GetProperty';
>>>>>>> graemeg/cpstrnew
=======

function CMM_CreateLinkProfile( prof: CMProfileRef; var profileSet: NCMConcatProfileSet; proc: CMConcatCallBackUPP; refCon: UnivPtr ): CMError; external name '_CMM_CreateLinkProfile';

function CMM_GetProperty( cw: CMWorldRef; requestedKey: CFStringRef ): CFTypeRef; external name '_CMM_GetProperty';
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4


//#pragma mark --- CMScriptingPlugin.h ----

{$ifc not TARGET_CPU_64}


const
{ ColorSync Scripting Errors }
	cmspInvalidImageFile = -4220; { Plugin cannot handle this image file type }
	cmspInvalidImageSpace = -4221; { Plugin cannot create an image file of this colorspace }
	cmspInvalidProfileEmbed = -4222; { Specific invalid profile errors }
	cmspInvalidProfileSource = -4223;
	cmspInvalidProfileDest = -4224;
	cmspInvalidProfileProof = -4225;
	cmspInvalidProfileLink = -4226;


{*** embedFlags field  ***}
{ reserved for future use: currently 0 }

{*** matchFlags field  ***}
const
	cmspFavorEmbeddedMask = $00000001; { if bit 0 is 0 then use srcProf profile, if 1 then use profile embedded in image if present}


function CMValidImage( const (*var*) spec: FSSpec ): CMError; external name '_CMValidImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMGetImageSpace( const (*var*) spec: FSSpec; var space: OSType ): CMError; external name '_CMGetImageSpace';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMEmbedImage( const (*var*) specFrom: FSSpec; const (*var*) specInto: FSSpec; repl: Boolean; embProf: CMProfileRef ): CMError; external name '_CMEmbedImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMUnembedImage( const (*var*) specFrom: FSSpec; const (*var*) specInto: FSSpec; repl: Boolean ): CMError; external name '_CMUnembedImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMMatchImage( const (*var*) specFrom: FSSpec; const (*var*) specInto: FSSpec; repl: Boolean; qual: UInt32; srcProf: CMProfileRef; srcIntent: UInt32; dstProf: CMProfileRef ): CMError; external name '_CMMatchImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMProofImage( const (*var*) specFrom: FSSpec; const (*var*) specInto: FSSpec; repl: Boolean; qual: UInt32; srcProf: CMProfileRef; srcIntent: UInt32; dstProf: CMProfileRef; prfProf: CMProfileRef ): CMError; external name '_CMProofImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMLinkImage( const (*var*) specFrom: FSSpec; const (*var*) specInto: FSSpec; repl: Boolean; qual: UInt32; lnkProf: CMProfileRef; lnkIntent: UInt32 ): CMError; external name '_CMLinkImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMCountImageProfiles( const (*var*) spec: FSSpec; var count: UInt32 ): CMError; external name '_CMCountImageProfiles';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMGetIndImageProfile( const (*var*) spec: FSSpec; index: UInt32; var prof: CMProfileRef ): CMError; external name '_CMGetIndImageProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function CMSetIndImageProfile( const (*var*) specFrom: FSSpec; const (*var*) specInto: FSSpec; repl: Boolean; index: UInt32; prof: CMProfileRef ): CMError; external name '_CMSetIndImageProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

//#pragma mark -- CMDeviceIntegration.h


{$ifc not TARGET_CPU_64}
{$ALIGN MAC68K}
{$elsec}
{$packrecords c}
{$endc} {not TARGET_CPU_64}
{
    The current versions of the data structure
    containing information on registered devices.
}
const
	cmDeviceInfoVersion1 = $00010000;
	cmDeviceProfileInfoVersion1 = $00010000;
	cmDeviceProfileInfoVersion2 = $00020000;

const
	cmCurrentDeviceInfoVersion = cmDeviceInfoVersion1;
	cmCurrentProfileInfoVersion = cmDeviceProfileInfoVersion1;

{
    Certain APIs require a device ID or profile ID.  
    In some cases, a "default ID" can be used.
}
const
	cmDefaultDeviceID = 0;
	cmDefaultProfileID = 0;

{
    Possible values for device states accessible by the
    CMGetDeviceState() and CMSetDeviceState() APIs.
}
const
	cmDeviceStateDefault = $00000000;
	cmDeviceStateOffline = $00000001;
	cmDeviceStateBusy = $00000002;
	cmDeviceStateForceNotify = $80000000;
	cmDeviceStateDeviceRsvdBits = $00FF0000;
	cmDeviceStateAppleRsvdBits = $FF00FFFF;

{
    Possible values for flags passed to the
    CMIterateDeviceProfiles() API.
    
    "Factory" profiles are registered via the
    CMSetDeviceFactoryProfiles() API.
    
    "Custom" profiles are those which are meant to take
    the place of the factory profiles, as a result of
    customization or calibration.  These profiles are
    registered via the CMSetDeviceProfiles() API.
    
    To retrieve all of the the former for all devices,
    use cmIterateFactoryDeviceProfiles as the flags
    value when calling CMIterateDeviceProfiles().
    
    To retrieve only the latter for all devices, use
    the cmIterateCustomDeviceProfiles, as the flags
    value when calling CMIterateDeviceProfiles().
    
    To get the profiles in use for all devices, use
    cmIterateCurrentDeviceProfiles as the flags value.
    This will replace the factory profiles with any
    overrides, yielding the currently used set.
    
    To get all profiles, without replacement, use
    cmIterateAllDeviceProfiles. 
}
const
	cmIterateFactoryDeviceProfiles = $00000001;
	cmIterateCustomDeviceProfiles = $00000002;
	cmIterateCurrentDeviceProfiles = $00000003;
	cmIterateAllDeviceProfiles = $00000004;
	cmIterateDeviceProfilesMask = $0000000F;

{
    Errors returned by CMDeviceIntegration APIs
}
const
	cmDeviceDBNotFoundErr = -4227; { Prefs not found/loaded }
	cmDeviceAlreadyRegistered = -4228; { Re-registration of device }
	cmDeviceNotRegistered = -4229; { Device not found }
	cmDeviceProfilesNotFound = -4230; { Profiles not found }
	cmInternalCFErr = -4231; { CoreFoundation failure }
	cmPrefsSynchError = -4232;  { CFPreferencesSynchronize failed }


{
   Clients can register for notifications of device changes:
      Notification         Description                           Sent by API
    ----------------      -----------                           -----------
      DeviceRegistered      a device was registered               CMRegisterColorDevice()  
      DeviceUnregistered    a device was unregistered             CMUnregisterColorDevice()
      DeviceOnline          a device's state changed to Online    CMSetDeviceState()
      DeviceOffline         a device's state changed to Offline   CMSetDeviceState()
      DeviceState           a device's state changed              CMSetDeviceState()
      DefaultDevice         a class' default device changed       CMSetDefaultDevice()
      DeviceProfiles        a device's profiles changed           CMSetDeviceFactoryProfiles(), CMSetDeviceProfiles()
      DefaultDeviceProfile  a device's default profile ID changed CMSetDeviceDefaultProfileID()
      DisplayDeviceProfiles a display device's profiles changed   CMSetDeviceFactoryProfiles(), CMSetDeviceProfiles()
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDeviceRegisteredNotification CFSTRP('CMDeviceRegisteredNotification')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDeviceUnregisteredNotification CFSTRP('CMDeviceUnregisteredNotification')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDeviceOnlineNotification CFSTRP('CMDeviceOnlineNotification')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDeviceOfflineNotification CFSTRP('CMDeviceOfflineNotification')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDeviceStateNotification CFSTRP('CMDeviceStateNotification')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDefaultDeviceNotification CFSTRP('CMDefaultDeviceNotification')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDeviceProfilesNotification CFSTRP('CMDeviceProfilesNotification')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDefaultDeviceProfileNotification CFSTRP('CMDefaultDeviceProfileNotification')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCMDisplayDeviceProfilesNotification CFSTRP('CMDisplayDeviceProfilesNotification')}
{$endc}

{
    Device state data.
}
type
	CMDeviceState = UInt32;

{
    A CMDeviceID must be unique within a device's class.
}

type
	CMDeviceID = UInt32;

{
    A CMDeviceProfileID must only be unique per device.
}
type
	CMDeviceProfileID = UInt32;
	CMDeviceProfileIDPtr = ^CMDeviceProfileID;
{
    DeviceClass type.
}
const
	cmScannerDeviceClass = FourCharCode('scnr');
	cmCameraDeviceClass = FourCharCode('cmra');
	cmDisplayDeviceClass = FourCharCode('mntr');
	cmPrinterDeviceClass = FourCharCode('prtr');
	cmProofDeviceClass = FourCharCode('pruf');

type
	CMDeviceClass = OSType;

{
    CMDeviceScope
    Structure specifying a device's or a device setting's scope.
}
type
	CMDeviceScopePtr = ^CMDeviceScope;
	CMDeviceScope = record
		deviceUser: CFStringRef;             { kCFPreferencesCurrentUser | _AnyUser }
		deviceHost: CFStringRef;             { kCFPreferencesCurrentHost | _AnyHost }
	end;
	CMDeviceProfileScope = CMDeviceScope;
	CMDeviceProfileScopePtr = ^CMDeviceProfileScope;

{
    CMDeviceInfo
    Structure containing information on a given device.
}
type
	CMDeviceInfoPtr = ^CMDeviceInfo;
	CMDeviceInfo = record
		dataVersion: UInt32;            { cmDeviceInfoVersion1 }
		deviceClass: CMDeviceClass;            { device class }
		deviceID: CMDeviceID;               { device ID }
		deviceScope: CMDeviceScope;            { device's scope }
		deviceState: CMDeviceState;            { Device State flags }
		defaultProfileID: CMDeviceProfileID;       { Can change }
		deviceName: CFDictionaryRefPtr;             { Ptr to storage for CFDictionary of }
                                              { localized device names (could be nil) }
		profileCount: UInt32;           { Count of registered profiles }
		reserved: UInt32;               { Reserved for use by ColorSync }
	end;

{
    CMDeviceProfileInfo
    Structure containing information on a device profile.
}
type
	CMDeviceProfileInfoPtr = ^CMDeviceProfileInfo;
	CMDeviceProfileInfo = record
		dataVersion: UInt32;            { cmDeviceProfileInfoVersion1 }
		profileID: CMDeviceProfileID;              { The identifier for this profile }
		profileLoc: CMProfileLocation;             { The profile's location }
		profileName: CFDictionaryRef;            { CFDictionary of localized profile names }
		reserved: UInt32;               { Reserved for use by ColorSync }
	end;

type
	NCMDeviceProfileInfoPtr = ^NCMDeviceProfileInfo;
	NCMDeviceProfileInfo = record
		dataVersion: UInt32;            { cmDeviceProfileInfoVersion2 }
		profileID: CMDeviceProfileID;              { The identifier for this profile }
		profileLoc: CMProfileLocation;             { The profile's location }
		profileName: CFDictionaryRef;            { CFDictionary of localized profile names }
		profileScope: CMDeviceProfileScope;         { The scope this profile applies to }
		reserved: UInt32;               { Reserved for use by ColorSync }
	end;


{
    CMDeviceProfileArray
    Structure containing the profiles for a device.
}

type
	CMDeviceProfileArrayPtr = ^CMDeviceProfileArray;
	CMDeviceProfileArray = record
		profileCount: UInt32;           { Count of profiles in array }
		profiles: array [0..0] of CMDeviceProfileInfo;           { The profile info records }
	end;

{
    Caller-supplied iterator functions
}

type
	CMIterateDeviceInfoProcPtr = function( const (*var*) deviceInfo: CMDeviceInfo; refCon: UnivPtr ): OSErr;
	CMIterateDeviceProfileProcPtr = function( const (*var*) deviceInfo: CMDeviceInfo; const (*var*) profileInfo: NCMDeviceProfileInfo; refCon: UnivPtr ): OSErr;


{
    Device Registration
}

{!
    @function    CMRegisterColorDevice
    @abstract    Registers a device with ColorSync
    @discussion  For a device to be recognized by ColorSync it needs to register itself 
                    via this API.  After calling this API, the CMSetDeviceFactoryProfiles
                    API should be called to specify the initial modes and profiles for the
                    device. Registration need only happen once, when the device is installed.
    @param       deviceClass    (in) Device class to add
    @param       deviceID       (in) Device id to add
    @param       deviceName     (in) Dictionary containing localized names
    @param       deviceScope    (in) Scope where information should be stored
}
function CMRegisterColorDevice( deviceClass: CMDeviceClass; deviceID: CMDeviceID; deviceName: CFDictionaryRef; const (*var*) deviceScope: CMDeviceScope ): CMError; external name '_CMRegisterColorDevice';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
    @function    CMUnregisterColorDevice
    @abstract    Unregisters a device with ColorSync
    @discussion  When a device is no longer to be used on a system (as opposed to
                    just being offline), it should be unregistered. If a device is
                    temporariy shut down or disconnected it need not be unregistered
                    unless the device driver knows that it will not be used (e.g. being
                    deinstalled) or cannot access the device profiles without the device.
    @param       deviceClass    (in) Device class to remove
    @param       deviceID       (in) Device id to remove
}
function CMUnregisterColorDevice( deviceClass: CMDeviceClass; deviceID: CMDeviceID ): CMError; external name '_CMUnregisterColorDevice';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
    Default Device accessors
}

{!
    @function    CMSetDefaultDevice
    @abstract    Specifeis the default device of a class
    @param       deviceClass    (in) Device class to modify
    @param       deviceID       (in) Device id to make default
}
function CMSetDefaultDevice( deviceClass: CMDeviceClass; deviceID: CMDeviceID ): CMError; external name '_CMSetDefaultDevice';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
    @function    CMGetDefaultDevice
    @abstract    Returns the default device of a class
    @param       deviceClass    (in) Device class to query
    @param       deviceID       (out) Returns default device for class
}
function CMGetDefaultDevice( deviceClass: CMDeviceClass; var deviceID: CMDeviceID ): CMError; external name '_CMGetDefaultDevice';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
    Device Profile Registration & Access
}

{!
    @function    CMSetDeviceFactoryProfiles
    @abstract    Registers a device's factory profiles with ColorSync
    @discussion  This API establishes the profiles used by a particular device for 
                    it's various modes. It is meant to be called once, right after
                    device registration to notify ColorSync of the device's profiles.
    @param       deviceClass    (in) Device class to modify
    @param       deviceID       (in) Device id to modify
    @param       defaultProfID  (in) The id of the default profile
    @param       deviceProfiles (in) List of profile IDs, names, and locations
}
function CMSetDeviceFactoryProfiles( deviceClass: CMDeviceClass; deviceID: CMDeviceID; defaultProfID: CMDeviceProfileID; const (*var*) deviceProfiles: CMDeviceProfileArray ): CMError; external name '_CMSetDeviceFactoryProfiles';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function    CMGetDeviceFactoryProfiles
    @abstract    Returns all the device's factory profiles
    @discussion  This API allows the caller to retrieve the original profiles for a device.
                    These may differ from the profiles currently in use for that device in the
                    case where factory profiles have been overriden with custom profiles.
    @param       deviceClass    (in) Device class to query
    @param       deviceID       (in) Device id to query (can be cmDefaultDeviceID)
    @param       defaultProfID  (out) Returns id of default mode (optional)
    @param       arraySize      (in/out) Size of buffer passed in / Returns size of array in bytes
    @param       deviceProfiles (out) Returns list of profile IDs, names, and locations
}
function CMGetDeviceFactoryProfiles( deviceClass: CMDeviceClass; deviceID: CMDeviceID; defaultProfID: CMDeviceProfileIDPtr { can be NULL }; var arraySize: UInt32; var deviceProfiles: CMDeviceProfileArray ): CMError; external name '_CMGetDeviceFactoryProfiles';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function    CMSetDeviceProfiles
    @abstract    Specifies custom overide profiles for a device
    @discussion  This API provides a way to overide the factory profiles of a device for
                    a particular mode or modes. To set custom profiles, the profileScope and 
                    deviceProfiles params must be valid. To remove all custom profiles of a 
                    device, pass in nil for the profileScope and deviceProfiles parameters.
    @param       deviceClass    (in) Device class to change
    @param       deviceID       (in) Device id to change (can be cmDefaultDeviceID)
    @param       profileScope   (in) Scope where information should be stored (or nil to remove all)
    @param       deviceProfiles (in) Profiles to set (or nil to remove all)
}
{$ifc not TARGET_CPU_64}
function CMSetDeviceProfiles( deviceClass: CMDeviceClass; deviceID: CMDeviceID; {const} profileScope: CMDeviceProfileScopePtr { can be NULL }; {const} deviceProfiles: CMDeviceProfileArrayPtr { can be NULL } ): CMError; external name '_CMSetDeviceProfiles';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


{!
    @function    CMGetDeviceProfiles
    @abstract    Returns all the device's current profiles
    @discussion  This API allows the caller to retrieve the current profiles for a device.
    @param       deviceClass    (in) Device class to query
    @param       deviceID       (in) Device id to query (can be cmDefaultDeviceID)
    @param       arraySize      (in/out) Size of buffer passed in / Returns size of array in bytes
    @param       deviceProfiles (out) Returns list of profile IDs, names, and locations
}
{$ifc not TARGET_CPU_64}
function CMGetDeviceProfiles( deviceClass: CMDeviceClass; deviceID: CMDeviceID; var arraySize: UInt32; var deviceProfiles: CMDeviceProfileArray ): CMError; external name '_CMGetDeviceProfiles';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
{$endc} {not TARGET_CPU_64}


{!
    @function    CMSetDeviceDefaultProfileID
    @abstract    Specifies a device's default profile mode
    @discussion  This API allows the caller to change the default profile ID for a device.
                    The initial default is established when CMSetDeviceFactoryProfiles is called. 
    @param       deviceClass    (in) Device class to modify
    @param       deviceID       (in) Device id to modify (can be cmDefaultDeviceID)
    @param       defaultProfID  (in) New device default 
}
function CMSetDeviceDefaultProfileID( deviceClass: CMDeviceClass; deviceID: CMDeviceID; defaultProfID: CMDeviceProfileID ): CMError; external name '_CMSetDeviceDefaultProfileID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function    CMGetDeviceDefaultProfileID
    @abstract    Returns the default profile ID for a device
    @param       deviceClass    (in) Device class to query
    @param       deviceID       (in) Device id to query (can be cmDefaultDeviceID)
    @param       defaultProfID  (out) Returns id of default profile
}
function CMGetDeviceDefaultProfileID( deviceClass: CMDeviceClass; deviceID: CMDeviceID; var defaultProfID: CMDeviceProfileID ): CMError; external name '_CMGetDeviceDefaultProfileID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function    CMSetDeviceProfile
    @abstract    Specifies a custom overide profile for a device
    @discussion  This API provides a way to change one of the profiles used by a 
                    device for a particular mode. It can be called after
                    device registration by calibration applications to reset a device's
                    profile from its factory default to a custom calibrated profile.
    @param       deviceClass    (in) Device class to modify
    @param       deviceID       (in) Device id to modify (can be cmDefaultDeviceID)
    @param       profileScope   (in) Scope where information should be stored
    @param       profileID      (in) Profile id to modify (can be cmDefaultProfileID)
    @param       profileLoc     (in) New profile location 
}
function CMSetDeviceProfile( deviceClass: CMDeviceClass; deviceID: CMDeviceID; const (*var*) profileScope: CMDeviceProfileScope; profileID: CMDeviceProfileID; const (*var*) profileLoc: CMProfileLocation ): CMError; external name '_CMSetDeviceProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function    CMGetDeviceProfile
    @abstract    Returns the location of the current profile for a 
                    given device class, device ID, and profile ID
    @param       deviceClass    (in) Device class to query
    @param       deviceID       (in) Device id to query (can be cmDefaultDeviceID)
    @param       profileID      (in) Profile id to query (can be cmDefaultDeviceID)
    @param       profileLoc (out) Returns profile location
}
function CMGetDeviceProfile( deviceClass: CMDeviceClass; deviceID: CMDeviceID; profileID: CMDeviceProfileID; var profileLoc: CMProfileLocation ): CMError; external name '_CMGetDeviceProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
    Other Device State/Info accessors
}

{!
    @function    CMSetDeviceState
    @abstract    Specifies the state of a device
    @discussion  This API provides access for the device management layer to
                    update the state of a particular device. For example, a device
                    can be offline, busy, calibrated, etc. The state data passed in
                    replaces the old state data with the new value.
    @param       deviceClass    (in) Device class to modify
    @param       deviceID       (in) Device id to modify (can be cmDefaultDeviceID)
    @param       deviceState    (in) New device state 
}
function CMSetDeviceState( deviceClass: CMDeviceClass; deviceID: CMDeviceID; deviceState: CMDeviceState ): CMError; external name '_CMSetDeviceState';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function    CMGetDeviceState
    @abstract    Returns the state of a device. 
    @param       deviceClass    (in) Device class to query
    @param       deviceID       (in) Device id to query (can be cmDefaultDeviceID)
    @param       deviceState    (out) Returns device state
}
function CMGetDeviceState( deviceClass: CMDeviceClass; deviceID: CMDeviceID; var deviceState: CMDeviceState ): CMError; external name '_CMGetDeviceState';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function    CMGetDeviceInfo
    @abstract    Returns information about a device. 
    @discussion  This API returns information about a registered device.
                    If, on input, deviceInfo->deviceName is nil then the name is not returned.
                    If the caller wants the device name dictionary returned, then the caller
                    should provide in deviceInfo->deviceName the address where this API should 
                    store the CFDictionaryRef. The caller is responsible for disposing of the 
                    name dictionary.
    @param       deviceClass    (in) Device class to query
    @param       deviceID       (in) Device id to query (can be cmDefaultDeviceID)
    @param       deviceInfo     (in/out) Returns device information
}
function CMGetDeviceInfo( deviceClass: CMDeviceClass; deviceID: CMDeviceID; var deviceInfo: CMDeviceInfo ): CMError; external name '_CMGetDeviceInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
    Device Info & Profile Iterators
}

{!
    @function    CMIterateColorDevices
    @abstract    Returns information about all devices to a callback procedure. 
    @discussion  This API allows the caller to get device information about all 
                    registered color devices.  If provided, the supplied proceedure will be
                    called once for each registered device, passing in the device info and 
                    the supplied refcon.
                    If the caller passes in a pointer to a seed value that is the same as
                    the current seed value, then the callback proc is not called.
    @param       proc           (in) Client callback proc (optional)
    @param       seed           (in/out) seed value (optional)
    @param       count          (out) Returns count of devices (optional)
    @param       refCon         (in) Passed to callback proc (optional)
}
function CMIterateColorDevices( proc: CMIterateDeviceInfoProcPtr { can be NULL }; seed: UInt32Ptr { can be NULL }; count: UInt32Ptr { can be NULL }; refCon: UnivPtr { can be NULL } ): CMError; external name '_CMIterateColorDevices';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{!
    @function    CMIterateDeviceProfiles
    @abstract    Returns information about profiles of all devices to a callback procedure. 
    @discussion  This API allows the caller to get device information about profiles of all 
                    registered color devices.  If provided, the supplied proceedure will be
                    called once for each registered device, passing in the device info, the 
                    profile info and the supplied refcon.
                    If the caller passes in a pointer to a seed value that is the same as
                    the current seed value, then the callback proc is not called.
    @param       proc           (in) Client callback proc (optional)
    @param       seed           (in/out) seed value (optional)
    @param       count          (out) Returns count of devices (optional)
    @param       flags          (in) Options for which set of profiles are to be iterated.
                                        It can have the following possible values:
                                        cmIterateFactoryDeviceProfiles, cmIterateCustomDeviceProfiles, 
                                        cmIterateCurrentDeviceProfiles, cmIterateAllDeviceProfiles or 0.
                                        The flag value 0 behaves like cmIterateCurrentDeviceProfiles.
    @param       refCon         (in) Passed to callback proc (optional)
}
function CMIterateDeviceProfiles( proc: CMIterateDeviceProfileProcPtr { can be NULL }; seed: UInt32Ptr { can be NULL }; count: UInt32Ptr { can be NULL }; flags: UInt32; refCon: UnivPtr { can be NULL } ): CMError; external name '_CMIterateDeviceProfiles';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4
{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
