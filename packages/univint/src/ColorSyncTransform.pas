{
 * ColorSync - ColorSyncTransform.h
 * Copyright (c)  2008 Apple Inc.
 * All rights reserved.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/cpstrnew
=======
{   Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit ColorSyncTransform;
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
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes,CFBase,CFArray,CFDictionary;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ColorSyncTransformRef = ^OpaqueColorSyncTransformRef; { an opaque type }
	OpaqueColorSyncTransformRef = record end;
=======
	ColorSyncTransformRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ColorSyncTransformRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ColorSyncTransformRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ColorSyncTransformRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	ColorSyncTransformRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

function ColorSyncTransformGetTypeID: CFTypeID; external name '_ColorSyncTransformGetTypeID';
   {
    * returns the CFTypeID for ColorSyncTransforms.
    }

function ColorSyncTransformCreate( profileSequence: CFArrayRef; options: CFDictionaryRef ): ColorSyncTransformRef; external name '_ColorSyncTransformCreate';
   {
    *   profileSequence  - array of dictionaries, each one containing a profile object and the
    *                      information on the usage of the profile in the transform.
    *               
    *               Required keys:
    *               ==============
    *                      kColorSyncProfile           : ColorSyncProfileRef
    *                      kColorSyncRenderingIntent   : CFStringRef defining rendering intent
    *                      kColorSyncTransformTag      : CFStringRef defining which tags to use 
    *               Optional key:
    *               =============
    *                      kColorSyncBlackPointCompensation : CFBooleanRef to enable/disable BPC
    *   
    *   options      - dictionary with additional public global options (e.g. preferred CMM, quality,
    *                       etc... It can also contain custom options that are CMM specific.
    *
    *   returns ColorSyncTransformRef or NULL in case of failure
    }


function ColorSyncTransformCopyProperty( transform: ColorSyncTransformRef; key: CFTypeRef; options: CFDictionaryRef ): CFTypeRef; external name '_ColorSyncTransformCopyProperty';
   {
    *   transform - transform from which to copy the property
    *   key       - CFTypeRef to be used as a key to identify the property
    }


procedure ColorSyncTransformSetProperty( transform: ColorSyncTransformRef; key: CFTypeRef; property: CFTypeRef ); external name '_ColorSyncTransformSetProperty';
   {
    *   transform - transform in which to set the property
    *   key       - CFTypeRef to be used as a key to identify the property
    *   property  - CFTypeRef to be set as the property
    }

type
	ColorSyncDataDepth = SInt32;
const
	kColorSync1BitGamut = 1;
	kColorSync8BitInteger = 2;
	kColorSync16BitInteger = 3;
	kColorSync16BitFloat = 4;
	kColorSync32BitInteger = 5;
	kColorSync32BitNamedColorIndex = 6;
	kColorSync32BitFloat = 7;

type
	ColorSyncAlphaInfo = SInt32;
const
	kColorSyncAlphaNone = 0;               { For example, RGB. }
	kColorSyncAlphaPremultipliedLast = 1;  { For example, premultiplied RGBA }
	kColorSyncAlphaPremultipliedFirst = 2; { For example, premultiplied ARGB }
	kColorSyncAlphaLast = 3;               { For example, non-premultiplied RGBA }
	kColorSyncAlphaFirst = 4;              { For example, non-premultiplied ARGB }
	kColorSyncAlphaNoneSkipLast = 5;       { For example, RBGX. }
	kColorSyncAlphaNoneSkipFirst = 6;      { For example, XRGB. }

const
	kColorSyncAlphaInfoMask = $1F;
	kColorSyncByteOrderMask = $7000;
	kColorSyncByteOrderDefault = 0 shl 12;
	kColorSyncByteOrder16Little = 1 shl 12;
	kColorSyncByteOrder32Little = 2 shl 12;
	kColorSyncByteOrder16Big = 3 shl 12;
	kColorSyncByteOrder32Big = 4 shl 12;


type
	ColorSyncDataLayout = UInt32;

function ColorSyncTransformConvert( transform: ColorSyncTransformRef; width: size_t; height: size_t; dst: UnivPtr; dstDepth: ColorSyncDataDepth; dstLayout: ColorSyncDataLayout; dstBytesPerRow: size_t; src: {const} UnivPtr; srcDepth: ColorSyncDataDepth; srcLayout: ColorSyncDataLayout; srcBytesPerRow: size_t; options: CFDictionaryRef ): CBool; external name '_ColorSyncTransformConvert';
   {
    *   transform         - transform to be used for converting color
    *   width             - width of the image in pixels
    *   height            - height of the image in pixels
    *   dst               - a pointer to the destination where the results will be written.
    *   dstDepth          - describes the bit depth and type of the destination color components
    *   dstFormat         - describes the format and byte packing of the destination pixels
    *   dstBytesPerRow    - number of bytes in the row of data
    *   src               - a pointer to the data to be converted.
    *   srcDepth          - describes the bit depth and type of the source color components
    *   srcFormat         - describes the format and byte packing of the source pixels
    *   srcBytesPerRow    - number of bytes in the row of data
    *
    *   returns true if conversion was successful or false otherwise 
    }


{ Keys and values for profile specific info and options }
var kColorSyncProfile: CFStringRef; external name '_kColorSyncProfile'; (* attribute const *)
var kColorSyncRenderingIntent: CFStringRef; external name '_kColorSyncRenderingIntent'; (* attribute const *)

    { Legal values for kColorSyncRenderingIntent }
var kColorSyncRenderingIntentPerceptual: CFStringRef; external name '_kColorSyncRenderingIntentPerceptual'; (* attribute const *)
var kColorSyncRenderingIntentRelative: CFStringRef; external name '_kColorSyncRenderingIntentRelative'; (* attribute const *)
var kColorSyncRenderingIntentSaturation: CFStringRef; external name '_kColorSyncRenderingIntentSaturation'; (* attribute const *)
var kColorSyncRenderingIntentAbsolute: CFStringRef; external name '_kColorSyncRenderingIntentAbsolute'; (* attribute const *)
var kColorSyncRenderingIntentUseProfileHeader: CFStringRef; external name '_kColorSyncRenderingIntentUseProfileHeader'; (* attribute const *)

var kColorSyncTransformTag: CFStringRef; external name '_kColorSyncTransformTag'; (* attribute const *)

    { Legal values for kColorSyncTransformTag }
var kColorSyncTransformDeviceToPCS: CFStringRef; external name '_kColorSyncTransformDeviceToPCS'; (* attribute const *)
var kColorSyncTransformPCSToPCS: CFStringRef; external name '_kColorSyncTransformPCSToPCS'; (* attribute const *)
var kColorSyncTransformPCSToDevice: CFStringRef; external name '_kColorSyncTransformPCSToDevice'; (* attribute const *)
var kColorSyncTransformDeviceToDevice: CFStringRef; external name '_kColorSyncTransformDeviceToDevice'; (* attribute const *)
var kColorSyncTransformGamutCheck: CFStringRef; external name '_kColorSyncTransformGamutCheck'; (* attribute const *)

var kColorSyncBlackPointCompensation: CFStringRef; external name '_kColorSyncBlackPointCompensation'; (* attribute const *)

{ Global transform options }
var kColorSyncPreferredCMM: CFStringRef; external name '_kColorSyncPreferredCMM'; (* attribute const *)       { ColorSyncCMMRef of the preferred CMM }
var kColorSyncConvertQuality: CFStringRef; external name '_kColorSyncConvertQuality'; (* attribute const *)

    { Legal values for kColorSyncConvertQuality }
var kColorSyncBestQuality: CFStringRef; external name '_kColorSyncBestQuality'; (* attribute const *)      { do not coalesce profile transforms (default) }
var kColorSyncNormalQuality: CFStringRef; external name '_kColorSyncNormalQuality'; (* attribute const *)    { coalesce all transforms }
var kColorSyncDraftQuality: CFStringRef; external name '_kColorSyncDraftQuality'; (* attribute const *)     { coalesce all transforms, do not interpolate }

{ Conversion options }
var kColorSyncConvertThreadCount: CFStringRef; external name '_kColorSyncConvertThreadCount'; (* attribute const *) { applies to large amounts of data; 0 for number of CPUs }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kColorSyncConvertUseVectorUnit: CFStringRef; external name '_kColorSyncConvertUseVectorUnit'; (* attribute const *) { applies to large amounts of data; CFBooleanRef value; default true}
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

{ Public keys for copying transform properties }

var kColorSyncTranformInfo: CFStringRef; external name '_kColorSyncTranformInfo'; (* attribute const *)         { dictionary with the following keys }
var kColorSyncTransformCreator: CFStringRef; external name '_kColorSyncTransformCreator'; (* attribute const *) { name of the CMM that created the transform }
var kColorSyncTransformSrcSpace: CFStringRef; external name '_kColorSyncTransformSrcSpace'; (* attribute const *)
var kColorSyncTransformDstSpace: CFStringRef; external name '_kColorSyncTransformDstSpace'; (* attribute const *)

{
 * =============================
 *
 *  Code Fragment Support
 *
 * =============================
 *
 * ColorSync can return parameters for standard components of color conversion,
 * i.e. tone rendering curves (TRCs), 3x4 matrices (3x3 + 3x1), multi-dimensional
 * interpolation tables and Black Point Compensation.
 * The parameters are wrapped in CFArray or CFData objects specific to the
 * component type and placed in a CFDictionary under a key that identifies the 
 * type of the component. The complete code fragment is a CFArray of component
 * dictionaries that are placed in the in the order they have to be executed.
 * 
 * A code fragment is created by calling ColorSyncTransformCopyProperty with the key 
 * specifying the type of the code fragment to be created. NULL pointer will be
 * returnde if the requested code fragment cannot be created. The code fragment
 * will be stored in the ColorSyncTransform list of properties and can be removed by 
 * by calling ColorSyncTransformSetProperty with NULL value.
 }

{
 * Types of Code Fragments:
 *
 * 1. Full conversion: all non-NULL components based on all the tags from the
 *                     sequence of profiles passed to create the ColorWorld with
 *                     an exception of adjacent matrices that can be collapsed to
 *                     one matrix.
 * 2. Parametric:      same as above, except that the returned code fragment consists
 *                     only of parametric curves, matrices and BPC components.
 * 3. Simplified:      Full conversion is collapsed to N input TRCs one
 *                     multi-dimensional table and M output TRCs.
 }
 
var kColorSyncTransformFullConversionData: CFStringRef; external name '_kColorSyncTransformFullConversionData'; (* attribute const *)         { CFSTR("com.apple.cmm.FullConversion") }
var kColorSyncTransformSimplifiedConversionData: CFStringRef; external name '_kColorSyncTransformSimplifiedConversionData'; (* attribute const *)   { CFSTR("com.apple.cmm.SimplifiedConversion") }
var kColorSyncTransformParametricConversionData: CFStringRef; external name '_kColorSyncTransformParametricConversionData'; (* attribute const *)   { CFSTR("com.apple.cmm.ParametricConversion") }

{
 * Matrix: represented as a CFArray of three CFArrays of four CFNumbers (Float32)
 *         each, performin the following matrix operation
 *         y[3] = 3x3 matrix *x[3] + 3x1 vector (last column)
 }
var kColorSyncConversionMatrix: CFStringRef; external name '_kColorSyncConversionMatrix'; (* attribute const *)        { CFSTR("com.apple.cmm.Matrix") }

{
 * Tone Rendering Curves:
 *
 * 1. Parametric curves: represented as a CFArray of seven CFNumbers (Float32)
 *                       [gamma a b c d e f]
 *  Curve Type 0  Y = X^gamma
 *  Curve Type 1  Y = (aX+b)^gamma     [X >= -b/a],  Y = 0  [X < -b/a]
 *  Curve Type 2  Y = (aX+b)^gamma + c [X >= -b/a],  Y = c  [X < -b/a]
 *  Curve Type 3  Y = (aX+b)^gamma     [X >= d],     Y = cX [X < d]
 *  Curve Type 4  Y = (aX+b)^gamma + e [X >= d],     Y = cX + f [X < d]
 *
 * 2. 1-dimensional lookup with interpolation:
 *                  represented as CFData containing a Float32 table[gridPoints]
 }
 
var kColorSyncConversionParamCurve0: CFStringRef; external name '_kColorSyncConversionParamCurve0'; (* attribute const *)   { CFSTR("com.apple.cmm.ParamCurve0") }
var kColorSyncConversionParamCurve1: CFStringRef; external name '_kColorSyncConversionParamCurve1'; (* attribute const *)   { CFSTR("com.apple.cmm.ParamCurve1") }
var kColorSyncConversionParamCurve2: CFStringRef; external name '_kColorSyncConversionParamCurve2'; (* attribute const *)   { CFSTR("com.apple.cmm.ParamCurve2") }
var kColorSyncConversionParamCurve3: CFStringRef; external name '_kColorSyncConversionParamCurve3'; (* attribute const *)   { CFSTR("com.apple.cmm.ParamCurve3") }
var kColorSyncConversionParamCurve4: CFStringRef; external name '_kColorSyncConversionParamCurve4'; (* attribute const *)   { CFSTR("com.apple.cmm.ParamCurve4") }
var kColorSyncConversion1DLut: CFStringRef; external name '_kColorSyncConversion1DLut'; (* attribute const *)         { CFSTR("com.apple.cmm.1D-LUT") }
var kColorSyncConversionGridPoints: CFStringRef; external name '_kColorSyncConversionGridPoints'; (* attribute const *)    { CFSTR("com.apple.cmm.GridPointCount") }
var kColorSyncConversionChannelID: CFStringRef; external name '_kColorSyncConversionChannelID'; (* attribute const *)     { CFSTR("com.apple.cmm.ChannelID") }

{
 * Multi-dimensional lookup with interpolation:
 *
 *       represented as CFData containing a Float32 table for N inputs, M outputs 
 *       and P gridPoints in each direction. The dimensioncorresponding to the
 *       first input channel varies least rapidly, and the dimension corresponding
 *       to the last input channel varies most rapidly. Each grid point value contains
 *       M Float32 numbers, one for each of output channels (M is the number of outputs).
 }
 

var kColorSyncConversion3DLut: CFStringRef; external name '_kColorSyncConversion3DLut'; (* attribute const *)      { CFSTR("com.apple.cmm.3D-LUT") }
var kColorSyncConversionInpChan: CFStringRef; external name '_kColorSyncConversionInpChan'; (* attribute const *)    { CFSTR("com.apple.cmm.InputChannels") }
var kColorSyncConversionOutChan: CFStringRef; external name '_kColorSyncConversionOutChan'; (* attribute const *)    { CFSTR("com.apple.cmm.OutputChannels") }

{
 * Black Point Compensation: represented as an CFArray of CFNumbers (Float32)
 *
 * 1. Scaling in Luminance: CFArray containing two numbers.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * 2. Scaling in XYZ: CFArray containing six numbers.
=======
 * 2. Sc aling inn XYZ: CFArray containing six numbers.
>>>>>>> graemeg/cpstrnew
=======
 * 2. Sc aling inn XYZ: CFArray containing six numbers.
>>>>>>> graemeg/cpstrnew
=======
 * 2. Sc aling inn XYZ: CFArray containing six numbers.
>>>>>>> graemeg/cpstrnew
=======
 * 2. Sc aling inn XYZ: CFArray containing six numbers.
>>>>>>> origin/cpstrnew
=======
 * 2. Sc aling inn XYZ: CFArray containing six numbers.
>>>>>>> origin/fixes_2.4
 }
 
var kColorSyncConversionBPC: CFStringRef; external name '_kColorSyncConversionBPC'; (* attribute const *)   { CFSTR("com.apple.cmm.BPC") }

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
