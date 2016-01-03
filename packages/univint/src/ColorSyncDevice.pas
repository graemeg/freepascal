{
 * ColorSync - ColorSyncDevice.h
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

unit ColorSyncDevice;
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
=======
>>>>>>> origin/fixes_2.4
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
uses MacTypes,ColorSyncProfile,CFBase,CFDictionary,CFUUID;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
var kColorSyncDeviceID: CFStringRef; external name '_kColorSyncDeviceID'; (* attribute const *)                { CFUUIDRef }
var kColorSyncDeviceClass: CFStringRef; external name '_kColorSyncDeviceClass'; (* attribute const *)             { one of the below : }
var kColorSyncCameraDeviceClass: CFStringRef; external name '_kColorSyncCameraDeviceClass'; (* attribute const *)   { cmra }
var kColorSyncDisplayDeviceClass: CFStringRef; external name '_kColorSyncDisplayDeviceClass'; (* attribute const *)  { mntr }
var kColorSyncPrinterDeviceClass: CFStringRef; external name '_kColorSyncPrinterDeviceClass'; (* attribute const *)  { prtr }
var kColorSyncScannerDeviceClass: CFStringRef; external name '_kColorSyncScannerDeviceClass'; (* attribute const *)  { scnr }

var kColorSyncDeviceProfileURL: CFStringRef; external name '_kColorSyncDeviceProfileURL'; (* attribute const *)

var kColorSyncDeviceDescription: CFStringRef; external name '_kColorSyncDeviceDescription'; (* attribute const *)       { CFString with a name in current locale }
var kColorSyncDeviceDescriptions: CFStringRef; external name '_kColorSyncDeviceDescriptions'; (* attribute const *)      { CFDictionary with localized names }

var kColorSyncFactoryProfiles: CFStringRef; external name '_kColorSyncFactoryProfiles'; (* attribute const *)         { CFDictionary containing factory profile info }
var kColorSyncCustomProfiles: CFStringRef; external name '_kColorSyncCustomProfiles'; (* attribute const *)          { CFDictionary containing custom profile info }

var kColorSyncDeviceModeDescription: CFStringRef; external name '_kColorSyncDeviceModeDescription'; (* attribute const *)   { CFString, e.g. Glossy, Best Quality }
var kColorSyncDeviceModeDescriptions: CFStringRef; external name '_kColorSyncDeviceModeDescriptions'; (* attribute const *)  { CFDictionary with localized mode names }
var kColorSyncDeviceDefaultProfileID: CFStringRef; external name '_kColorSyncDeviceDefaultProfileID'; (* attribute const *)  { see below }
var kColorSyncDeviceHostScope: CFStringRef; external name '_kColorSyncDeviceHostScope'; (* attribute const *)         { kCFPreferences(Current,Any)Host }
var kColorSyncDeviceUserScope: CFStringRef; external name '_kColorSyncDeviceUserScope'; (* attribute const *)         { kCFPreferences(Current,Any)User }
var kColorSyncProfileHostScope: CFStringRef; external name '_kColorSyncProfileHostScope'; (* attribute const *)        { kCFPreferences(Current,Any)Host }
var kColorSyncProfileUserScope: CFStringRef; external name '_kColorSyncProfileUserScope'; (* attribute const *)        { kCFPreferences(Current,Any)User }

var kColorSyncDeviceProfileIsFactory: CFStringRef; external name '_kColorSyncDeviceProfileIsFactory'; (* attribute const *)  { Present in ColorSyncDeviceProfileInfo dictionary.}
                                                        { See ColorSyncDeviceProfileIterateCallback below. }
var kColorSyncDeviceProfileIsDefault: CFStringRef; external name '_kColorSyncDeviceProfileIsDefault'; (* attribute const *)  { ditto }
var kColorSyncDeviceProfileIsCurrent: CFStringRef; external name '_kColorSyncDeviceProfileIsCurrent'; (* attribute const *)  { ditto }
var kColorSyncDeviceProfileID: CFStringRef; external name '_kColorSyncDeviceProfileID'; (* attribute const *)         { ditto }

var kColorSyncDeviceRegisteredNotification: CFStringRef; external name '_kColorSyncDeviceRegisteredNotification'; (* attribute const *)        { com.apple.ColorSync.DeviceRegisteredNotification }
var kColorSyncDeviceUnregisteredNotification: CFStringRef; external name '_kColorSyncDeviceUnregisteredNotification'; (* attribute const *)      { com.apple.ColorSync.DeviceUnregisteredNotification }
var kColorSyncDeviceProfilesNotification: CFStringRef; external name '_kColorSyncDeviceProfilesNotification'; (* attribute const *)         { com.apple.ColorSync.DeviceProfilesNotification }
var kColorSyncDisplayDeviceProfilesNotification: CFStringRef; external name '_kColorSyncDisplayDeviceProfilesNotification'; (* attribute const *)   { com.apple.ColorSync.DisplayProfileNotification }

function ColorSyncRegisterDevice( deviceClass: CFStringRef; deviceID: CFUUIDRef; deviceInfo: CFDictionaryRef ): CBool; external name '_ColorSyncRegisterDevice';
   {
    *   deviceInfo  -a dictionary containing information needed to register a device.
    *   ----------------------------------------------------------------------------
    *               
    *               Required keys:
    *               ==============
    *                   kColorSyncDeviceDescriptions: CFDictionary with localized names of the device.
    *                                                 Localization keys must be five character strings
    *                                                 containing language code and region code in the
    *                                                 lc_RG format and it must contain (at least) the "en_US" locale.
    *                   kColorSyncFactoryProfiles   : CFDictionary with factory profile info CFDictionaries
    *                                                 The keys are the profile IDs and the values
    *                                                 are the profile info dictionaries.
    *               Optional keys:
    *               ==============
    *                   kColorSyncDeviceHostScope   : host scope of the device;
    *                                                 one of kCFPreferences(Current,Any)Host;
    *                                                 if unspecified kCFPreferencesCurrentHost is
    *                                                 assumed.
    *                   kColorSyncDeviceUserScope   : user scope of the device;
    *                                                 one of kCFPreferences(Current,Any)User;
    *                                                 if unspecified kCFPreferencesCurrentUser is
    *                                                 assumed.
    *
    *           factory profiles dictionary - value for the key kColorSyncFactoryProfiles in deviceInfo
    *           -------------------------------------------------------------------------------------
    *               Required keys and values:
    *               ========================
    *                   Each profile is identified by a ProfileID (of CFStringRef type) which used as the key.
    *                   Value associated with the key is a  profile info dictionary
    *                   that describes an individual device profile.
    *
    *                   kColorSyncDeviceDefaultProfileID: the associated value must be one of the ProfileID
    *                                                     present in the dictionary. Presence of this 
    *                                                     key is not required if there is only one factory profile.
    *
    *                   profile info CFDictionary
    *                   --------------------------------
    *                   Required keys:
    *                   ==============
    *                       kColorSyncDeviceProfileURL      :CFURLRef of the profile to be registered
    *                       kColorSyncDeviceModeDescriptions:CFDictionary with localized device mode
    *                                                        names for the profile. Localization keys 
    *                                                        must be five character strings containing
    *                                                        language code and region code in the lc_RG 
    *                                                        format and it must contain (at least) the
    *                                                        "en_US" locale.
    *                                                        E.g. "en_US" "Glossy Paper with best quality"
    *
    * Example of deviceInfo dictionary:
    *
    *   <<
    *       kColorSyncDeviceDescriptions   <<
    *                                           en_US  My Little Printer
    *                                           de_DE  Mein Kleiner Drucker
    *                                           fr_FR  Mon petit immprimeur
    *                                           ...
    *                                       >>
    *       kColorSyncFactoryProfiles       <<
    *                                           CFSTR("Profile 1")  <<
    *                                                                   kColorSyncDeviceProfileURL    (CFURLRef)
    *
    *                                                                   kColorSyncDeviceModeDescriptions    <<
    *                                                                                                           en_US Glossy Paper
    *                                                                                                           de_DE Glanzpapier
    *                                                                                                           fr_FR Papier glace
    *                                                                                                           ...
    *                                                                                                       >>
    *                                           ...
    *                                           
    *                                           kColorSyncDeviceDefaultProfileID  CFSTR("Profile 1")
    *                                       >>
    *       kColorSyncDeviceUserScope   kCFPreferencesAnyUser
    *
    *       kColorSyncDeviceHostScope   kCFPreferencesCurrentHost
    *   <<
    *
    * Notes:    1. Scope for factory profiles is exactly the same as the device scope.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    *           2. Pass kCFNull in lieu of the profile URL or no URl key/value pair at all if 
=======
    *           2. Pass CFNullRef in lieu of the profile URL or no URl key/value pair at all if 
>>>>>>> graemeg/cpstrnew
=======
    *           2. Pass CFNullRef in lieu of the profile URL or no URl key/value pair at all if 
>>>>>>> graemeg/cpstrnew
=======
    *           2. Pass CFNullRef in lieu of the profile URL or no URl key/value pair at all if 
>>>>>>> graemeg/cpstrnew
=======
    *           2. Pass CFNullRef in lieu of the profile URL or no URl key/value pair at all if 
>>>>>>> origin/cpstrnew
=======
    *           2. Pass CFNullRef in lieu of the profile URL or no URl key/value pair at all if 
>>>>>>> origin/fixes_2.4
    *              factory profile is not available. This will enable setting custom profile.
    *           3. For the reasons of compatibility with legacy API, it is recommended that the
    *              profile keys are created as CFStrings from uint32_t numbers as follows:
    *              CFStringRef key = CFStringCreateWithFormat(NULL, NULL, CFSTR("%u"), (uint32_t) i);
    *
    *   returns true on success and false in case of failure
    }

function ColorSyncUnregisterDevice( deviceClass: CFStringRef; deviceID: CFUUIDRef ): CBool; external name '_ColorSyncUnregisterDevice';
   {
    *   Unregister a device of given deviceClass and deviceID.
    *
    *   returns true on success and false in case of failure
    }

function ColorSyncDeviceSetCustomProfiles( deviceClass: CFStringRef; deviceID: CFUUIDRef; profileInfo: CFDictionaryRef ): CBool; external name '_ColorSyncDeviceSetCustomProfiles';
   {
    *                   profileInfo is a CFDictionary containing the information about 
    *                   custom profiles to be set in lieu of factory profiles.
    *                   Required keys:
    *                   ==============
    *                       ProfileIDs which must be the subset of the ProfileIDs that device was registered with
    *                       or kColorSyncDeviceDefaultProfileID for setting custom default profile.
    *
    *                   Required values:
    *                   ==============
    *                       CFURLRef of the profile to be set as a custom profile.
    *
    *                   Optional keys:
    *                   ==============
    *                       kColorSyncProfileHostScope  : host scope of the profile;
    *                                                     one of kCFPreferences(Current,Any)Host;
    *                                                     if unspecified kCFPreferencesCurrentHost
    *                                                     is assumed.
    *                       kColorSyncProfileUserScope  : user scope of the profile;
    *                                                     one of kCFPreferences(Current,Any)User;
    *                                                     if unspecified kCFPreferencesCurrentUser
    *                                                     is assumed.
    *
    *
    * Notes:    1. Profile scope for custom profiles cannot exceed scope of the factory profiles.
    *           2. There is only one host scope and user scope per dictionary (i.e. per call)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    *           3. Pass kCFNull in lieu of the profile URL to unset the custom profile and
=======
    *           3. Pass CFNullRef in lieu of the profile URL to unset the custom profile and
>>>>>>> graemeg/cpstrnew
=======
    *           3. Pass CFNullRef in lieu of the profile URL to unset the custom profile and
>>>>>>> graemeg/cpstrnew
=======
    *           3. Pass CFNullRef in lieu of the profile URL to unset the custom profile and
>>>>>>> graemeg/cpstrnew
=======
    *           3. Pass CFNullRef in lieu of the profile URL to unset the custom profile and
>>>>>>> origin/cpstrnew
=======
    *           3. Pass CFNullRef in lieu of the profile URL to unset the custom profile and
>>>>>>> origin/fixes_2.4
    *              reset the current profile to the factory profile.
    *
    *   returns true on success and false in case of failure
    }

function ColorSyncDeviceCopyDeviceInfo( deviceClass: CFStringRef; devID: CFUUIDRef ): CFDictionaryRef; external name '_ColorSyncDeviceCopyDeviceInfo';
   {
    *   Returns a dictionary with the following keys and values resolved for the current host and current user.
    *
    *   <<
    *       kColorSyncDeviceClass                   (camera, display, printer, scanner)
    *       kColorSyncDeviceID                      (CFUUIDRef registered with ColorSync)
    *       kColorSyncDeviceDescription             (localized device description)
    *       kColorSyncFactoryProfiles  (dictionary) <<
    *                                                   (ProfileID)    (dictionary) <<
    *                                                                                   kColorSyncDeviceProfileURL      (CFURLRef or kCFNull)
    *                                                                                   kColorSyncDeviceModeDescription (localized mode description)
    *                                                                               >>
    *                                                    ...
    *                                                   kColorSyncDeviceDefaultProfileID (ProfileID)
    *                                               >>
    *       kColorSyncCustomProfiles  (dictionary) <<
    *                                                   (ProfileID)    (CFURLRef or kCFNull)
    *                                                   ...
    *                                              <<
    *       kColorSyncDeviceUserScope              (kCFPreferencesAnyUser or kCFPreferencesCurrentUser)
    *       kColorSyncDeviceHostScope              (kCFPreferencesAnyHost or kCFPreferencesCurrentHost)
    *   >>
    }
    
type
	ColorSyncDeviceProfileIterateCallback = function( colorSyncDeviceProfileInfo: CFDictionaryRef; userInfo: UnivPtr ): CBool;
   {
    *   colorSyncDeviceProfileInfo contains the following keys: 
    *   <<
    *       kColorSyncDeviceClass                   (camera, display, printer, scanner)
    *       kColorSyncDeviceID                      (CFUUIDRef registered with ColorSync)
    *       kColorSyncDeviceDescription             (localized device description)
    *       kColorSyncDeviceModeDescription         (localized device mode description)
    *       kColorSyncDeviceProfileID               (ProfileID registered with ColorSync)
    *       kColorSyncDeviceProfileURL              (CFURLRef registered with ColorSync)
    *       kColorSyncDeviceProfileIsFactory        (kCFBooleanTrue or kCFBooleanFalse)
    *       kColorSyncDeviceProfileIsDefault        (kCFBooleanTrue or kCFBooleanFalse)
    *       kColorSyncDeviceProfileIsCurrent        (kCFBooleanTrue or kCFBooleanFalse)
    *   >>
    }
                                                       
procedure ColorSyncIterateDeviceProfiles( callBack: ColorSyncDeviceProfileIterateCallback; userInfo: UnivPtr ); external name '_ColorSyncIterateDeviceProfiles';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    {
     * A utility function converting displayID to CFUUIDRef
     }
function CGDisplayCreateUUIDFromDisplayID( displayID: UInt32 ): CFUUIDRef; external name '_CGDisplayCreateUUIDFromDisplayID';
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)
    
    {
     * A utility function converting first 32 bits of CFUUIDRef to displayID
     }
function CGDisplayGetDisplayIDFromUUID( uuid: CFUUIDRef ): UInt32; external name '_CGDisplayGetDisplayIDFromUUID';
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)

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
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
