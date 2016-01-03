{
 * ColorSync - ColorSyncProfile.h
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

unit ColorSyncProfile;
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
uses MacTypes,CFBase,CFArray,CFData,CFDictionary,CFError,CFUUID;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ColorSyncProfileRef = ^OpaqueColorSyncProfileRef; { an opaque type }
	OpaqueColorSyncProfileRef = record end;
=======
	ColorSyncProfileRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ColorSyncProfileRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ColorSyncProfileRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ColorSyncProfileRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	ColorSyncProfileRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

type
	ColorSyncProfile_ = record end;
	ColorSyncMutableProfileRef = ^ColorSyncProfile_;

var kColorSyncGenericGrayProfile: CFStringRef; external name '_kColorSyncGenericGrayProfile'; (* attribute const *)          { com.apple.ColorSync.GenericGray  }
var kColorSyncGenericGrayGamma22Profile: CFStringRef; external name '_kColorSyncGenericGrayGamma22Profile'; (* attribute const *)   { com.apple.ColorSync.GenericGrayGamma2.2  }
var kColorSyncGenericRGBProfile: CFStringRef; external name '_kColorSyncGenericRGBProfile'; (* attribute const *)           { com.apple.ColorSync.GenericRGB   }
var kColorSyncGenericCMYKProfile: CFStringRef; external name '_kColorSyncGenericCMYKProfile'; (* attribute const *)          { com.apple.ColorSync.GenericCMYK  }
var kColorSyncSRGBProfile: CFStringRef; external name '_kColorSyncSRGBProfile'; (* attribute const *)                 { com.apple.ColorSync.sRGB         }
var kColorSyncAdobeRGB1998Profile: CFStringRef; external name '_kColorSyncAdobeRGB1998Profile'; (* attribute const *)         { com.apple.ColorSync.AdobeRGB1998 }
var kColorSyncGenericLabProfile: CFStringRef; external name '_kColorSyncGenericLabProfile'; (* attribute const *)           { com.apple.ColorSync.GenericLab }
var kColorSyncGenericXYZProfile: CFStringRef; external name '_kColorSyncGenericXYZProfile'; (* attribute const *)           { com.apple.ColorSync.GenericXYZ }

var kColorSyncProfileHeader: CFStringRef; external name '_kColorSyncProfileHeader'; (* attribute const *)      { com.apple.ColorSync.ProfileHeader }
var kColorSyncProfileClass: CFStringRef; external name '_kColorSyncProfileClass'; (* attribute const *)       { com.apple.ColorSync.ProfileClass }
var kColorSyncProfileColorSpace: CFStringRef; external name '_kColorSyncProfileColorSpace'; (* attribute const *)  { com.apple.ColorSync.ProfileColorSpace }
var kColorSyncProfilePCS: CFStringRef; external name '_kColorSyncProfilePCS'; (* attribute const *)         { com.apple.ColorSync.PCS }
var kColorSyncProfileURL: CFStringRef; external name '_kColorSyncProfileURL'; (* attribute const *)         { com.apple.ColorSync.ProfileURL }
var kColorSyncProfileDescription: CFStringRef; external name '_kColorSyncProfileDescription'; (* attribute const *) { com.apple.ColorSync.ProfileDescription }
var kColorSyncProfileMD5Digest: CFStringRef; external name '_kColorSyncProfileMD5Digest'; (* attribute const *)   { com.apple.ColorSync.ProfileMD5Digest }

function ColorSyncProfileGetTypeID: CFTypeID; external name '_ColorSyncProfileGetTypeID';
   {
    * returns the CFTypeID for ColorSyncProfiles.
    }

function ColorSyncProfileCreate( data: CFDataRef; var error: CFErrorRef ): ColorSyncProfileRef; external name '_ColorSyncProfileCreate';
   {
    *   data   - profile data
    *   error  - (optional) pointer to the error that will be returned in case of failure
    *   
    *   returns ColorSyncProfileRef or NULL in case of failure
    }

function ColorSyncProfileCreateWithURL( url: CFURLRef; var error: CFErrorRef ): ColorSyncProfileRef; external name '_ColorSyncProfileCreateWithURL';
   {
    *   url    - URL to the profile data (caller needs to have privileges to read url).
    *   error  - (optional) pointer to the error that will be returned in case of failure
    *   
    *   returns ColorSyncProfileRef or NULL in case of failure
    }

function ColorSyncProfileCreateWithName( name: CFStringRef ): ColorSyncProfileRef; external name '_ColorSyncProfileCreateWithName';
   {
    *   name    - predefined profile name
    *   
    *   returns ColorSyncProfileRef or NULL in case of failure
    }

function ColorSyncProfileCreateWithDisplayID( displayID: UInt32 ): ColorSyncProfileRef; external name '_ColorSyncProfileCreateWithDisplayID';
   {
    *   displayID - system-wide unique display ID (defined by IOKIt); pass 0 for main display.
    *   
    *   returns ColorSyncProfileRef or NULL in case of failure
    }

function ColorSyncProfileCreateDeviceProfile( deviceClass: CFStringRef; deviceID: CFUUIDRef; profileID: CFTypeRef ): ColorSyncProfileRef; external name '_ColorSyncProfileCreateDeviceProfile';
   {
    *   deviceClass - ColorSync device class 
    *   deviceID    - deviceID registered with ColorSync
    *   profileID   - profileID registered with ColorSync; pass kColorSyncDeviceDefaultProfileID to get the default profile.
    *   
    *   See ColorSyncDevice.h for more info on deviceClass, deviceID and profileID
    *
    *   returns ColorSyncProfileRef or NULL in case of failure
    }

function ColorSyncProfileCreateMutable: ColorSyncMutableProfileRef; external name '_ColorSyncProfileCreateMutable';
   {
    *   returns empty ColorSyncMutableProfileRef or NULL in case of failure
    }

function ColorSyncProfileCreateMutableCopy( prof: ColorSyncProfileRef ): ColorSyncMutableProfileRef; external name '_ColorSyncProfileCreateMutableCopy';
   {
    *  prof  - profile from which profile data will be copied to the created profile.
    *   
    *   returns ColorSyncMutableProfileRef or NULL in case of failure
    }

function ColorSyncProfileCreateLink( profileInfo: CFArrayRef; options: CFDictionaryRef ): ColorSyncProfileRef; external name '_ColorSyncProfileCreateLink';
   {
    *   profileInfo  - array of dictionaries, each one containing a profile object and the
    *                       information on the usage of the profile in the transform.
    *               
    *               Required keys:
    *               ==============
    *                      kColorSyncProfile           : ColorSyncProfileRef
    *                      kColorSyncRenderingIntent   : CFStringRef defining rendering intent
    *                      kColorSyncTransformTag      : CFStringRef defining which tags to use 
    *               Optional key:
    *               =============
    *                    kColorSyncBlackPointCompensation : CFBooleanRef to enable/disable BPC
    *   
    *   options      - dictionary with additional public global options (e.g. preferred CMM, quality,
    *                       etc... It can also contain custom options that are CMM specific.
    *
    *   returns ColorSyncProfileRef or NULL in case of failure
    }

function ColorSyncProfileVerify( prof: ColorSyncProfileRef; var errors: CFErrorRef; var warnings: CFErrorRef ): CBool; external name '_ColorSyncProfileVerify';
   {
    *   prof    - profile to be verified
    *
    *   errors  - returns error strings in case problems are found which 
    *                  would prevent use of the profile.
    *  
    *   warnings - returns warning strings indicating problems due to lack of
    *                       conformance with the ICC specification, but not preventing
    *                       use of the profile.
    *
    *   returns true if profile can be used or false otherwise
    }

function ColorSyncProfileEstimateGammaWithDisplayID( {const} displayID: SInt32; var error: CFErrorRef ): Float32; external name '_ColorSyncProfileEstimateGammaWithDisplayID';
   {
    *   displayID - system-wide unique display ID (defined by IOKIt)
    *   error     - (optional) pointer to the error that will be returned in
    *               case of failure
    *   
    *   returns non-zero value if success or 0.0 in case of error.
    }

function ColorSyncProfileEstimateGamma( prof: ColorSyncProfileRef; var error: CFErrorRef ): Float32; external name '_ColorSyncProfileEstimateGamma';
    {
    *   prof    - profile to perform estimation on
    *   error   - (optional) pointer to the error that will be returned in
    *             case of failure
    *   
    *   returns non-zero value if success or 0.0 in case of error
    }

const
	COLORSYNC_MD5_LENGTH = 16;

type
	ColorSyncMD5 = record
		digest: array [0..COLORSYNC_MD5_LENGTH-1] of 	UInt8;
	end;

function ColorSyncProfileGetMD5( prof: ColorSyncProfileRef ): ColorSyncMD5; external name '_ColorSyncProfileGetMD5';
   { 
    *   returns MD5 digest for the profile calculated as defined by
    *           ICC specification or a "zero" signature (filled with zeros)
    *           in case of failure
    }

function ColorSyncProfileCopyData( prof: ColorSyncProfileRef; var error: CFErrorRef ): CFDataRef; external name '_ColorSyncProfileCopyData';
   {
    *   prof    - profile to copy the flattened data from
    *   error  - (optional) pointer to the error that will be returned in case of failure
    *   
    *   returns CFDataRef if success or NULL in case of failure 
    }

function ColorSyncProfileGetURL( prof: ColorSyncProfileRef; var error: CFErrorRef ): CFURLRef; external name '_ColorSyncProfileGetURL';
   {
    *   prof   - profile to get URL from
    *   error  - (optional) pointer to the error that will be returned in case of failure
    *   
    *   returns CFURLRef if success or NULL in case of failure 
    }

function ColorSyncProfileCopyHeader( prof: ColorSyncProfileRef ): CFDataRef; external name '_ColorSyncProfileCopyHeader';
   {
    *   prof    - profile from which to copy the header
    *   
    *   returns CFDataRef containing profile header (in host endianess) or NULL in case of failure 
    }

procedure ColorSyncProfileSetHeader( prof: ColorSyncMutableProfileRef; header: CFDataRef ); external name '_ColorSyncProfileSetHeader';
   {
    *   prof        - profile in which to set the header
    *   header  - CFDataRef containing the header data (must be in host endianess)
    }

function ColorSyncProfileCopyDescriptionString( prof: ColorSyncProfileRef ): CFStringRef; external name '_ColorSyncProfileCopyDescriptionString';
   {
    *   prof    - profile from which to copy description string
    *   
    *   returns CFStringRef containing profile description localized to current locale
    }

function ColorSyncProfileCopyTagSignatures( prof: ColorSyncProfileRef ): CFArrayRef; external name '_ColorSyncProfileCopyTagSignatures';
   {
    *   prof    - profile from which to copy tag signatures
    *   
    *   returns CFArray with signatures (CFStringRef) of tags in the profile 
    }

function ColorSyncProfileContainsTag( prof: ColorSyncProfileRef; signature: CFStringRef ): CBool; external name '_ColorSyncProfileContainsTag';
   {
    *   prof        - profile in which to search for the tag
    *   signature   - signature of the tag to be searched for
    *   
    *   returns true if tag exists or false if does not 
    }

function ColorSyncProfileCopyTag( prof: ColorSyncProfileRef; signature: CFStringRef ): CFDataRef; external name '_ColorSyncProfileCopyTag';
   {
    *   prof             - profile from which to copy the tag
    *   signature   - signature of the tag to be copied
    *   
    *   returns CFDataRef containing tag data or NULL in case of failure 
    }

procedure ColorSyncProfileSetTag( prof: ColorSyncMutableProfileRef; signature: CFStringRef; data: CFDataRef ); external name '_ColorSyncProfileSetTag';
   {
    *   prof           - profile in which to set the tag
    *   signature - signature of the tag to be set in the profile
    *   data          - CFDataRef containing the tag data
    }

procedure ColorSyncProfileRemoveTag( prof: ColorSyncMutableProfileRef; signature: CFStringRef ); external name '_ColorSyncProfileRemoveTag';
   {
    *   prof              - profile from which to remove the tag
    *   signature    - signature of the tag to be removed
    *   
    *   returns true if success or false in case of failure 
    }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function ColorSyncProfileGetDisplayTransferFormulaFromVCGT( profile: ColorSyncProfileRef; var redMin: Float32; var redMax: Float32; var redGamma: Float32; var greenMin: Float32; var greenMax: Float32; var greenGamma: Float32; var blueMin: Float32; var blueMax: Float32; var blueGamma: Float32 ): CBool; external name '_ColorSyncProfileGetDisplayTransferFormulaFromVCGT';
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)
   {
    * An utility function converting vcgt tag (if vcgt tag exists in the profile and conversion possible)
    * to formula components used by CGSetDisplayTransferByFormula.
    }

function ColorSyncProfileCreateDisplayTransferTablesFromVCGT( profile: ColorSyncProfileRef; var nSamplesPerChannel: size_t ): CFDataRef; external name '_ColorSyncProfileCreateDisplayTransferTablesFromVCGT';
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)
   {
    * An utility function creating three tables of floats (redTable, greenTable, blueTable)
    * each of size nSamplesPerChannel, packed into contiguous memory contained in the CFDataRef
    *  to be returned from the vcgt tag of the profile (if vcgt tag exists in the profile).
    * Used by CGSetDisplayTransferByTable.
    }


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
type
	ColorSyncProfileIterateCallback = function( profileInfo: CFDictionaryRef; userInfo: UnivPtr ): CBool;
   {
    * Notes:
    *   1. Only validated profiles will be passed to the caller
    *   2. if the ColorSyncProfileIterateCallback returns false, the iteration stops
    }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure ColorSyncIterateInstalledProfiles( callBack: ColorSyncProfileIterateCallback; seed: UInt32Ptr; userInfo: UnivPtr; var error: CFErrorRef ); external name '_ColorSyncIterateInstalledProfiles';
   {
    * callBack - pointer to a client provided function (can be NULL)
    * seed     - pointer to a cache seed owned by the client (can be NULL)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure ColorSyncIterateInstalledProfiles( callBack: ColorSyncProfileIterateCallback; var seed: UInt32; userInfo: UnivPtr; var error: CFErrorRef ); external name '_ColorSyncIterateInstalledProfiles';
   {
    * callBack - pointer to a client provided function (can be NULL)
    * seed     - pointer to a cache seed owned by the client
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
    * error    - (optional) pointer to the error that will be returned in case of failure
    *
    }

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
