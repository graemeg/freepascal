{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       LSInfo.h
 
     Contains:   Public interfaces for LaunchServices.framework
 
     Copyright:  (c) 2001-2012 by Apple Inc. All rights reserved.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
     File:       LaunchServices/LSInfo.h
 
     Contains:   Public interfaces for LaunchServices.framework
 
     Version:    LaunchServices-360.3~1
 
     Copyright:  © 2001-2008 by Apple Computer, Inc., all rights reserved.
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
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                     http://www.freepascal.org/bugs.html
 
}
{	 Pascal Translation: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
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

unit LSInfo;
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
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
uses MacTypes,CFBase,CFArray,CFDictionary,CFURL,Files;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}


{$ALIGN MAC68K}

{ ======================================================================================================== }
{ LaunchServices Type & Constants                                                                          }
{ ======================================================================================================== }

const
	kLSAppInTrashErr = -10660; { The app cannot be run when inside a Trash folder}
	kLSExecutableIncorrectFormat = -10661; { No compatible executable was found}
	kLSAttributeNotFoundErr = -10662; { An item attribute value could not be found with the specified name}
	kLSAttributeNotSettableErr = -10663; { The attribute is not settable}
	kLSIncompatibleApplicationVersionErr = -10664; { The app is incompatible with the current OS}
	kLSNoRosettaEnvironmentErr = -10665; { The Rosetta environment was required not available}
	kLSUnknownErr = -10810; { Unexpected internal error}
	kLSNotAnApplicationErr = -10811; { Item needs to be an application, but is not}
	kLSNotInitializedErr = -10812; { Not used in 10.2 and later}
	kLSDataUnavailableErr = -10813; { E.g. no kind string}
	kLSApplicationNotFoundErr = -10814; { E.g. no application claims the file}
	kLSUnknownTypeErr = -10815; { Don't know anything about the type of the item}
	kLSDataTooOldErr = -10816; { Not used in 10.3 and later}
	kLSDataErr = -10817; { Not used in 10.4 and later}
	kLSLaunchInProgressErr = -10818; { E.g. launching an already launching application}
	kLSNotRegisteredErr = -10819; { Not used in 10.3 and later}
	kLSAppDoesNotClaimTypeErr = -10820; { Not used in 10.3 and later}
	kLSAppDoesNotSupportSchemeWarning = -10821; { Not used in 10.2 and later}
	kLSServerCommunicationErr = -10822; { The server process (registration and recent items) is not available}
	kLSCannotSetInfoErr = -10823; { The extension visibility on this item cannot be changed}
	kLSNoRegistrationInfoErr = -10824; { The item contains no registration info}
	kLSIncompatibleSystemVersionErr = -10825; { The app cannot run on the current OS version}
	kLSNoLaunchPermissionErr = -10826; { User doesn't have permission to launch the app (managed networks)}
	kLSNoExecutableErr = -10827; { The executable is missing}
	kLSNoClassicEnvironmentErr = -10828; { The Classic environment was required but is not available}
	kLSMultipleSessionsNotSupportedErr = -10829; { The app cannot run simultaneously in two different sessions}

type
	LSInitializeFlags = OptionBits;
const
	kLSInitializeDefaults = $00000001;

const
	kLSMinCatInfoBitmap = kFSCatInfoNodeFlags or kFSCatInfoParentDirID or kFSCatInfoFinderInfo or kFSCatInfoFinderXInfo; { do not use }

{ #define kLSInvalidExtensionIndex ULONG_MAX }
const
	kLSInvalidExtensionIndex = high(UNSIGNEDLONG);	// Index returned from LSGetExtensionInfo when name has no extension
	
type
	LSRequestedInfo = OptionBits;
const
	kLSRequestExtension = $00000001; { thread-safe in 10.0}
	kLSRequestTypeCreator = $00000002; { thread-safe in 10.0}
	kLSRequestBasicFlagsOnly = $00000004; { thread-safe in 10.2}
	kLSRequestAppTypeFlags = $00000008; { thread-safe in 10.2}
	kLSRequestAllFlags = $00000010; { thread-safe in 10.2}
	kLSRequestIconAndKind = $00000020; { thread-safe in 10.2}
	kLSRequestExtensionFlagsOnly = $00000040; { thread-safe in 10.0}
	kLSRequestAllInfo = $FFFFFFFF; { thread-safe in 10.2}

type
	LSItemInfoFlags = OptionBits;
const
	kLSItemInfoIsPlainFile = $00000001; { Not a directory, volume, or symlink}
	kLSItemInfoIsPackage = $00000002; { Packaged directory}
	kLSItemInfoIsApplication = $00000004; { Single-file or packaged application}
	kLSItemInfoIsContainer = $00000008; { Directory (includes packages) or volume}
	kLSItemInfoIsAliasFile = $00000010; { Alias file (includes sym links)}
	kLSItemInfoIsSymlink = $00000020; { UNIX sym link}
	kLSItemInfoIsInvisible = $00000040; { Invisible by any known mechanism}
	kLSItemInfoIsNativeApp = $00000080; { Carbon or Cocoa native app}
	kLSItemInfoIsClassicApp = $00000100; { CFM/68K Classic app}
	kLSItemInfoAppPrefersNative = $00000200; { Carbon app that prefers to be launched natively}
	kLSItemInfoAppPrefersClassic = $00000400; { Carbon app that prefers to be launched in Classic}
	kLSItemInfoAppIsScriptable = $00000800; { App can be scripted}
	kLSItemInfoIsVolume = $00001000; { Item is a volume}
	kLSItemInfoExtensionIsHidden = $00100000; { Item has a hidden extension}

type
	LSRolesMask = OptionBits;
const
	kLSRolesNone = $00000001; { no claim is made about support for this type/scheme}
	kLSRolesViewer = $00000002; { claim to view items of this type}
	kLSRolesEditor = $00000004; { claim to edit items of this type/scheme}
	kLSRolesShell = $00000008; { claim to execute items of this type}
	kLSRolesAll = $FFFFFFFF; { claim to do it all}

{$ifc not TARGET_CPU_64}
type
	LSKindID = UInt32;
const
	kLSUnknownKindID = 0;

{$endc}	{ TARGET_CPU_64 }

const
	kLSUnknownType = 0;
	kLSUnknownCreator = 0;

type
	LSItemInfoRecord = record
		flags: LSItemInfoFlags;
		filetype: OSType;
		creator: OSType;
		extension: CFStringRef;              { release when finished}
{$ifc not TARGET_CPU_64}
		iconFileName: CFStringRef;           { not for general use}
		kindID: LSKindID;                 { not for general use}
{$endc}	{ TARGET_CPU_64 }
	end;

type
	LSAcceptanceFlags = OptionBits;
const
	kLSAcceptDefault = $00000001;
	kLSAcceptAllowLoginUI = $00000002; { show UI to log in if necessary}


{ ======================================================================================================== }
{ LaunchServices API                                                                                       }
{ ======================================================================================================== }

{
 *  LSInit()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    LSInit is deprecated. Do not use.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSInit( inFlags: LSInitializeFlags ): OSStatus; external name '_LSInit';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/fixes_2.4


{
 *  LSTerm()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    LSTerm is deprecated. It does nothing.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSTerm: OSStatus; external name '_LSTerm';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/fixes_2.4


{
 *  LSCopyItemInfoForRef()
 *  
 *  Summary:
 *    Return information about an item.
 *  
 *  Discussion:
 *    Returns as much or as little information as requested about
 *    inItemRef.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inItemRef:
 *      The FSRef of the item about which information is requested.
 *    
 *    inWhichInfo:
 *      Flags indicating which information to return
 *    
 *    outItemInfo:
 *      Information is returned in this structure. Must not be NULL
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyItemInfoForRef( const (*var*) inItemRef: FSRef; inWhichInfo: LSRequestedInfo; var outItemInfo: LSItemInfoRecord ): OSStatus; external name '_LSCopyItemInfoForRef';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSCopyItemInfoForURL()
 *  
 *  Summary:
 *    Return information about an item.
 *  
 *  Discussion:
 *    Returns as much or as little information as requested about inURL.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inURL:
 *      The CFURLRef of the item about which information is requested.
 *    
 *    inWhichInfo:
 *      Flags indicating which information to return
 *    
 *    outItemInfo:
 *      Information is returned in this structure. Must not be NULL
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyItemInfoForURL( inURL: CFURLRef; inWhichInfo: LSRequestedInfo; var outItemInfo: LSItemInfoRecord ): OSStatus; external name '_LSCopyItemInfoForURL';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSGetExtensionInfo()
 *  
 *  Summary:
 *    Get information about the extension for a file system name.
 *  
 *  Discussion:
 *    Returns the starting index of the extension (not including the
 *    period) or kLSInvalidExtensionIndex if the input name has no
 *    extension.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inNameLen:
 *      The number of the UniChars in inNameBuffer.
 *    
 *    inNameBuffer:
 *      The buffer containing the name's Unicode characters.
 *    
 *    outExtStartIndex:
 *      On success, the starting index of the extension if there is one
 *      (not including the period). Set to kLSInvalidExtensionIndex if
 *      inNameBuffer does not contain a valid extension.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSGetExtensionInfo( inNameLen: UniCharCount; {const} inNameBuffer: {variable-size-array} UniCharPtr; var outExtStartIndex: UniCharCount ): OSStatus; external name '_LSGetExtensionInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
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
 *  LSCopyDisplayNameForRef()
 *  
 *  Summary:
 *    Get the display name for an FSRef.
 *  
 *  Discussion:
 *    Return a copy of the display name for an FSRef. Takes into
 *    consideration whether this item has a hidden extension or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inRef:
 *      The FSRef for which the display name is desired.
 *    
 *    outDisplayName:
 *      Pointer to the CFString into which the display name should be
 *      copied. Callers must dispose of the resulting CFString.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyDisplayNameForRef( const (*var*) inRef: FSRef; var outDisplayName: CFStringRef ): OSStatus; external name '_LSCopyDisplayNameForRef';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
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
 *  LSCopyDisplayNameForURL()
 *  
 *  Summary:
 *    Get the display name for a CFURLRef.
 *  
 *  Discussion:
 *    Return a copy of the display name for a CFURLRef. Takes into
 *    consideration whether this item has a hidden extension or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inURL:
 *      The URL for which the display name is desired.
 *    
 *    outDisplayName:
 *      Pointer to the CFString into which the display name should be
 *      copied. Callers must dispose of the resulting CFString.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyDisplayNameForURL( inURL: CFURLRef; var outDisplayName: CFStringRef ): OSStatus; external name '_LSCopyDisplayNameForURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
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
 *  LSSetExtensionHiddenForRef()
 *  
 *  Summary:
 *    Sets whether the extension for an FSRef is hidden or not.
 *  
 *  Discussion:
 *    Sets the necessary file system state to indicate that the
 *    extension for inRef is hidden, as in the Finder. You can
 *    determine if an FSRef's extension is hidden using
 *    LSCopyItemInfoForRef.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inRef:
 *      The FSRef for which the extension is to be hidden or shown.
 *    
 *    inHide:
 *      True to hide inRef's extension, false to show it.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSSetExtensionHiddenForRef( const (*var*) inRef: FSRef; inHide: Boolean ): OSStatus; external name '_LSSetExtensionHiddenForRef';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
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
 *  LSSetExtensionHiddenForURL()
 *  
 *  Summary:
 *    Sets whether the extension for a CFURLRef is hidden or not.
 *  
 *  Discussion:
 *    Sets the necessary file system state to indicate that the
 *    extension for inURL is hidden, as in the Finder. You can
 *    determine if a CFURLRef's extension is hidden using
 *    LSCopyItemInfoForURL.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inURL:
 *      The CFURLRef for which the extension is to be hidden or shown.
 *    
 *    inHide:
 *      True to hide inURL's extension, false to show it.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSSetExtensionHiddenForURL( inURL: CFURLRef; inHide: Boolean ): OSStatus; external name '_LSSetExtensionHiddenForURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_1, __IPHONE_NA) *)
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
 *  LSCopyKindStringForRef()
 *  
 *  Summary:
 *    Get the kind string for an item.
 *  
 *  Discussion:
 *    Returns the kind string as used in the Finder and elsewhere for
 *    inFSRef. The CFStringRef must be released after use.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inFSRef:
 *      The item for which the kind string is requested.
 *    
 *    outKindString:
 *      A CFStringRef* to receive the copied kind string object. This
 *      CFStringRef must be released eventually.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyKindStringForRef( const (*var*) inFSRef: FSRef; var outKindString: CFStringRef ): OSStatus; external name '_LSCopyKindStringForRef';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSCopyKindStringForURL()
 *  
 *  Summary:
 *    Get the kind string for an item.
 *  
 *  Discussion:
 *    Returns the kind string as used in the Finder and elsewhere for
 *    inURL. The CFStringRef must be released after use.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inURL:
 *      The item for which the kind string is requested.
 *    
 *    outKindString:
 *      A CFStringRef* to receive the copied kind string object. This
 *      CFStringRef must be released eventually.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyKindStringForURL( inURL: CFURLRef; var outKindString: CFStringRef ): OSStatus; external name '_LSCopyKindStringForURL';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSCopyKindStringForTypeInfo()
 *  
 *  Summary:
 *    Return the kind string for items like the provided info
 *  
 *  Discussion:
 *    Returns the kind string as shown in the Finder for the those
 *    items whose type, creator, and/or extension match the provided
 *    information. The kind string returned will be the one that most
 *    closely describes all the information provided. The kind string
 *    is subject to the document binding preferences that have been
 *    specified by the user. For example, if a creator is specified but
 *    the user has asked for files with the given
 *    creator/type/extension combination to open in an application with
 *    a different creator, the kind string will be loaded from the
 *    user's preferred application.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inType:
 *      The OSType file type for which you want a kind string. Specify
 *      kLSUnknownType if no file type information is available.
 *    
 *    inCreator:
 *      The OSType creator for which you want a kind string. Specify
 *      kLSUnknownCreator if no creator information is available.
 *    
 *    inExtension:
 *      The extension for which you want a kind string. Specify NULL if
 *      no extension information is available.
 *    
 *    outKindString:
 *      A CFStringRef* to receive the copied kind string object. This
 *      CFStringRef must be released eventually.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyKindStringForTypeInfo( inType: OSType; inCreator: OSType; inExtension: CFStringRef { can be NULL }; var outKindString: CFStringRef ): OSStatus; external name '_LSCopyKindStringForTypeInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSCopyKindStringForMIMEType()
 *  
 *  Summary:
 *    Get the kind string for the specified MIME type.
 *  
 *  Discussion:
 *    Returns the localized kind string describing the specified MIME
 *    type.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inMIMEType:
 *      The string specifying the MIME type.
 *    
 *    outKindString:
 *      A CFStringRef* to receive the copied kind string object. This
 *      CFStringRef must be released eventually.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyKindStringForMIMEType( inMIMEType: CFStringRef; var outKindString: CFStringRef ): OSStatus; external name '_LSCopyKindStringForMIMEType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSGetApplicationForItem()
 *  
 *  Summary:
 *    Return the application used to open an item.
 *  
 *  Discussion:
 *    Consults the binding tables to return the application that would
 *    be used to open inItemRef if it were double-clicked in the
 *    Finder. This application will be the user-specified override if
 *    appropriate or the default otherwise. If no application is known
 *    to LaunchServices suitable for opening this item,
 *    kLSApplicationNotFoundErr will be returned.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inItemRef:
 *      The FSRef of the item for which the application is requested.
 *    
 *    inRoleMask:
 *      Whether to return the editor or viewer for inItemRef. If you
 *      don't care which, use kLSRolesAll.
 *    
 *    outAppRef:
 *      Filled in with the FSRef of the application if not NULL.
 *    
 *    outAppURL:
 *      Filled in with the CFURLRef of the application if not NULL.
 *      THIS FUNCTION, DESPITE ITS NAME, RETAINS THE URL REFERENCE ON
 *      BEHALF OF THE CALLER. THE CALLER MUST EVENTUALLY RELEASE THE
 *      RETURNED URL REFERENCE.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSGetApplicationForItem( const (*var*) inItemRef: FSRef; inRoleMask: LSRolesMask; outAppRef: FSRefPtr { can be NULL }; outAppURL: CFURLRefPtr { can be NULL } ): OSStatus; external name '_LSGetApplicationForItem';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSGetApplicationForInfo()
 *  
 *  Summary:
 *    Return the application used to open items with particular data.
 *  
 *  Discussion:
 *    Consults the binding tables to return the application that would
 *    be used to open items with type, creator, and/or extension as
 *    provided if they were double-clicked in the Finder. This
 *    application will be the default for items like this if one has
 *    been set. If no application is known to LaunchServices suitable
 *    for opening such items, kLSApplicationNotFoundErr will be
 *    returned. Not all three input parameters can be NULL at the same
 *    time nor can both output parameters be NULL at the same time.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inType:
 *      The file type to consider. Can be kLSUnknownType.
 *    
 *    inCreator:
 *      The file creator to consider. Can be kLSUnknownCreator.
 *    
 *    inExtension:
 *      The file name extension to consider. Can be NULL.
 *    
 *    inRoleMask:
 *      Whether to return the editor or viewer for inItemRef. If you
 *      don't care which, use kLSRolesAll.
 *    
 *    outAppRef:
 *      Filled in with the FSRef of the application if not NULL.
 *    
 *    outAppURL:
 *      Filled in with the CFURLRef of the application if not NULL.
 *      THIS FUNCTION, DESPITE ITS NAME, RETAINS THE URL REFERENCE ON
 *      BEHALF OF THE CALLER. THE CALLER MUST EVENTUALLY RELEASE THE
 *      RETURNED URL REFERENCE.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSGetApplicationForInfo( inType: OSType; inCreator: OSType; inExtension: CFStringRef { can be NULL }; inRoleMask: LSRolesMask; outAppRef: FSRefPtr { can be NULL }; outAppURL: CFURLRefPtr { can be NULL } ): OSStatus; external name '_LSGetApplicationForInfo';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSCopyApplicationForMIMEType()
 *  
 *  Summary:
 *    Return the application used to handle data with the specified
 *    MIME type.
 *  
 *  Discussion:
 *    The returned application URL will be the user's preferred handler
 *    for the MIME type if one has been set. If no user preferred
 *    application has been set, Launch Services will select a default
 *    handler for the MIME type. If no application is known to handle
 *    the MIME type, kLSApplicationNotFoundErr will be returned.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inMIMEType:
 *      The string specifying the MIME type.
 *    
 *    inRoleMask:
 *      A role mask that the chosen application must satisfy. Use
 *      kLSRolesAll if the role is not important.
 *    
 *    outAppURL:
 *      Receives the copied CFURLRef, which must be released by the
 *      caller.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyApplicationForMIMEType( inMIMEType: CFStringRef; inRoleMask: LSRolesMask; var outAppURL: CFURLRef ): OSStatus; external name '_LSCopyApplicationForMIMEType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSGetApplicationForURL()
 *  
 *  Summary:
 *    Return the application used to open an item.
 *  
 *  Discussion:
 *    Consults the binding tables to return the application that would
 *    be used to open inURL if it were double-clicked in the Finder.
 *    This application will be the user-specified override if
 *    appropriate or the default otherwise. If no application is known
 *    to LaunchServices suitable for opening this item,
 *    kLSApplicationNotFoundErr will be returned.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inURL:
 *      The CFURLRef of the item for which the application is requested.
 *    
 *    inRoleMask:
 *      Whether to return the editor or viewer for inItemRef. If you
 *      don't care which, use kLSRolesAll.
 *    
 *    outAppRef:
 *      Filled in with the FSRef of the application if not NULL.
 *    
 *    outAppURL:
 *      Filled in with the CFURLRef of the application if not NULL.
 *      THIS FUNCTION, DESPITE ITS NAME, RETAINS THE URL REFERENCE ON
 *      BEHALF OF THE CALLER. THE CALLER MUST EVENTUALLY RELEASE THE
 *      RETURNED URL REFERENCE.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSGetApplicationForURL( inURL: CFURLRef; inRoleMask: LSRolesMask; outAppRef: FSRefPtr { can be NULL }; outAppURL: CFURLRefPtr { can be NULL } ): OSStatus; external name '_LSGetApplicationForURL';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSFindApplicationForInfo()
 *  
 *  Summary:
 *    Locate a specific application.
 *  
 *  Discussion:
 *    Returns the application with the corresponding input information.
 *    The registry of applications is consulted first in order of
 *    bundleID, then creator, then name. All comparisons are case
 *    insensitive and 'ties' are decided first by version, then by
 *    native vs. Classic.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inCreator:
 *      The file creator to consider. Can be kLSUnknownCreator.
 *    
 *    inBundleID:
 *      The bundle ID to consider. Can be NULL.
 *    
 *    inName:
 *      The name to consider. Can be NULL. Must include any extensions
 *      that are part of the file system name, e.g. '.app'.
 *    
 *    outAppRef:
 *      Filled in with the FSRef of the application if not NULL.
 *    
 *    outAppURL:
 *      Filled in with the CFURLRef of the application if not NULL.
 *      THIS FUNCTION, DESPITE ITS NAME, RETAINS THE URL REFERENCE ON
 *      BEHALF OF THE CALLER. THE CALLER MUST EVENTUALLY RELEASE THE
 *      RETURNED URL REFERENCE.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSFindApplicationForInfo( inCreator: OSType; inBundleID: CFStringRef { can be NULL }; inName: CFStringRef { can be NULL }; outAppRef: FSRefPtr { can be NULL }; outAppURL: CFURLRefPtr { can be NULL } ): OSStatus; external name '_LSFindApplicationForInfo';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSCanRefAcceptItem()
 *  
 *  Summary:
 *    Determine whether an item can accept another item.
 *  
 *  Discussion:
 *    Returns in outAcceptsItem whether inTargetRef can accept
 *    inItemFSRef as in a drag and drop operation. If inRoleMask is
 *    other than kLSRolesAll then make sure inTargetRef claims to
 *    fulfill the requested role.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inItemFSRef:
 *      FSRef of the item about which acceptance is requested.
 *    
 *    inTargetRef:
 *      FSRef of the potential target.
 *    
 *    inRoleMask:
 *      The role(s) the target must claim in order to consider
 *      acceptance.
 *    
 *    inFlags:
 *      Use kLSAcceptDefault.
 *    
 *    outAcceptsItem:
 *      Filled in with result. Must not be NULL.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCanRefAcceptItem( const (*var*) inItemFSRef: FSRef; const (*var*) inTargetRef: FSRef; inRoleMask: LSRolesMask; inFlags: LSAcceptanceFlags; var outAcceptsItem: Boolean ): OSStatus; external name '_LSCanRefAcceptItem';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSCanURLAcceptURL()
 *  
 *  Summary:
 *    Determine whether an item can accept another item.
 *  
 *  Discussion:
 *    Returns in outAcceptsItem whether inTargetURL can accept
 *    inItemURL as in a drag and drop operation. If inRoleMask is other
 *    than kLSRolesAll then make sure inTargetRef claims to fulfill the
 *    requested role.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inItemURL:
 *      CFURLRef of the item about which acceptance is requested.
 *    
 *    inTargetURL:
 *      CFURLRef of the potential target.
 *    
 *    inRoleMask:
 *      The role(s) the target must claim in order to consider
 *      acceptance.
 *    
 *    inFlags:
 *      Use kLSAcceptDefault.
 *    
 *    outAcceptsItem:
 *      Filled in with result. Must not be NULL.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCanURLAcceptURL( inItemURL: CFURLRef; inTargetURL: CFURLRef; inRoleMask: LSRolesMask; inFlags: LSAcceptanceFlags; var outAcceptsItem: Boolean ): OSStatus; external name '_LSCanURLAcceptURL';
<<<<<<< HEAD
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSRegisterURL()
 *  
 *  Discussion:
 *    If the specified URL refers to an application or other bundle
 *    claiming to handle documents or URLs, add the bundle's document
 *    and URL claims to the Launch Services database.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    inURL:
 *      The CFURLRef of the item (a directory or file) to be registered.
 *    
 *    inUpdate:
 *      When false, LSRegisterURL does not register the item if it has
 *      already been registered and the current modification date of
 *      the item has not changed from when it was last registered. When
 *      true, the item's registered info is updated, even if the
 *      modification has not changed.
 *  
 *  Result:
 *    An OSStatus value: noErr - Success kLSNoRegistrationInfoErr - The
 *    item does not contain info requiring registration kLSDataErr -
 *    The item's property list info is malformed.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSRegisterURL( inURL: CFURLRef; inUpdate: Boolean ): OSStatus; external name '_LSRegisterURL';
<<<<<<< HEAD
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSRegisterFSRef()
 *  
 *  Discussion:
 *    If the specified FSRef refers to an application or other bundle
 *    claiming to handle documents or URLs, add the bundle's document
 *    and URL claims to the Launch Services database.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    inRef:
 *      The FSRef of the item to be registered.
 *    
 *    inUpdate:
 *      When false, LSRegisterFSRef does not register the item if it
 *      has already been registered and the current modification date
 *      of the item has not changed from when it was last registered.
 *      When true, the item's registered info is updated, even if the
 *      modification has not changed.
 *  
 *  Result:
 *    An OSStatus value: noErr - Success kLSNoRegistrationInfoErr - The
 *    item does not contain info requiring registration kLSDataErr -
 *    The item's property list info is malformed.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSRegisterFSRef( const (*var*) inRef: FSRef; inUpdate: Boolean ): OSStatus; external name '_LSRegisterFSRef';
<<<<<<< HEAD
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSCopyApplicationURLsForURL()
 *  
 *  Discussion:
 *    Returns an array of URLs to applications that offer the requested
 *    role(s) for the input item.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    inURL:
 *      The CFURLRef of the item for which all suitable applications
 *      are desired. If the URL is a file URL, it is treated as a
 *      document, and applications are selected based on the document's
 *      type information. Otherwise, applications are selected based on
 *      the URL's scheme.
 *    
 *    inRoleMask:
 *      The role(s) which must intersect with the role provided by an
 *      application for the specified item in order for the application
 *      to be included in the result. Pass kLSRolesAll if any role is
 *      acceptable.
 *  
 *  Result:
 *    An array of CFURLRefs, one for each application which can open
 *    inURL with at least one of the roles in inRoleMask, or NULL if no
 *    applications can open the item. When an array is returned, you
 *    must eventually release it.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSCopyApplicationURLsForURL( inURL: CFURLRef; inRoleMask: LSRolesMask ): CFArrayRef; external name '_LSCopyApplicationURLsForURL';
<<<<<<< HEAD
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{ ================================================================================== }
{   API for retrieving item attributes                                               }
{ ================================================================================== }
{ 
 *  Attributes Names
 *
 *  kLSItemContentType
 *
 *    The item's content type identifier (a uniform type identifier string)
 *    Value type CFStringRef
 *
 *
 *  kLSItemFileType
 *
 *    The item's file type (OSType)
 *    Value type CFStringRef
 *
 *
 *  kLSItemFileCreator
 *
 *    The item's file creator (OSType)
 *    Value type CFStringRef 
 *
 *
 *  kLSItemExtension
 *
 *    The item's filename extension
 *    Value type CFStringRef 
 *
 *
 *  kLSItemDisplayName
 *
 *    The item's name as displayed to the user
 *    (The display name reflects localization and
 *    extension hiding which may be in effect)
 *    Value type CFStringRef 
 *
 *
 *  kLSItemDisplayKind
 *
 *    The localized kind string describing this item's type
 *    Value type CFStringRef 
 *
 *
 *  kLSItemRoleHandlerDisplayName
 *
 *    The display name of the application set to handle (open) this item
 *    (subject to the role mask)
 *    value type CFStringRef 
 *
 *
 *  kLSItemIsInvisible
 *
 *    True if the item is normally hidden from users
 *    Value type CFBooleanRef 
 *
 *
 *  kLSItemExtensionIsHidden
 *
 *    True if the item's extension is set to be hidden
 *    Value type CFBooleanRef 
 *
 *
 *  kLSItemQuarantineProperties (can be set)
 *
 *    A dictionary of quarantine properties. See LSQuarantine.h for
 *    quarantine property definitions. This attribute is not
 *    present if the item is not quarantined.
 *    Value type CFDictionaryRef. May be NULL.
 }
{
 *  kLSItemContentType
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemContentType: CFStringRef; external name '_kLSItemContentType'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemFileType
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemFileType: CFStringRef; external name '_kLSItemFileType'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemFileCreator
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemFileCreator: CFStringRef; external name '_kLSItemFileCreator'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemExtension
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemExtension: CFStringRef; external name '_kLSItemExtension'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemDisplayName
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemDisplayName: CFStringRef; external name '_kLSItemDisplayName'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemDisplayKind
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemDisplayKind: CFStringRef; external name '_kLSItemDisplayKind'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemRoleHandlerDisplayName
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemRoleHandlerDisplayName: CFStringRef; external name '_kLSItemRoleHandlerDisplayName'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemIsInvisible
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemIsInvisible: CFStringRef; external name '_kLSItemIsInvisible'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemExtensionIsHidden
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemExtensionIsHidden: CFStringRef; external name '_kLSItemExtensionIsHidden'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
{
 *  kLSItemQuarantineProperties
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kLSItemQuarantineProperties: CFStringRef; external name '_kLSItemQuarantineProperties'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
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
{
 *  LSCopyItemAttribute()
 *  
 *  Discussion:
 *    Assigns the value of the specified item's attribute (or NULL, if
 *    the item has no such attribute or an error occurs) to *outValue.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    inItem:
 *      The FSRef of the item
 *    
 *    inRoles:
 *      The role(s), at least one of which must be provided by the
 *      application selected when computing attributes related to
 *      document binding (such as kLSItemRoleHandlerDisplayName). Pass
 *      kLSRolesAll if any role is acceptable.
 *    
 *    inAttributeName:
 *      The name of the attribute to copy
 *    
 *    outValue:
 *      Receives the attribute value
 *  
 *  Result:
 *    an OSStatus value. Returns kLSAttributeNotFoundErr if the item
 *    does not have the requested attribute.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSCopyItemAttribute( const (*var*) inItem: FSRef; inRoles: LSRolesMask; inAttributeName: CFStringRef; var outValue: CFTypeRef ): OSStatus; external name '_LSCopyItemAttribute';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSCopyItemAttributes()
 *  
 *  Discussion:
 *    Creates a dictionary containing the specified attribute values
 *    and assigns it to *outValues. The output dictionary keys are the
 *    attribute names. The CFTypeID of each value in the dictionary
 *    varies by attribute. See each attribute name constant for a
 *    description of its value type. An attribute key will be absent
 *    from the values dictionary if the item has no such attribute.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    inItem:
 *      The FSRef of the item
 *    
 *    inRoles:
 *      The role(s), at least one of which must be provided by the
 *      application selected when computing attributes related to
 *      document binding (such as kLSItemRoleHandlerDisplayName). Pass
 *      kLSRolesAll if any role is acceptable.
 *    
 *    inAttributeNames:
 *      The array of attribute names
 *    
 *    outValues:
 *      Receives the dictionary of attribure name-value pairs
 *  
 *  Result:
 *    an OSStatus value.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSCopyItemAttributes( const (*var*) inItem: FSRef; inRoles: LSRolesMask; inAttributeNames: CFArrayRef; var outValues: CFDictionaryRef ): OSStatus; external name '_LSCopyItemAttributes';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSSetItemAttribute()
 *  
 *  Discussion:
 *    Sets the value of a settable item's attribute. Currently, only
 *    the kLSItemQuarantineProperties attribute may be set.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inItem:
 *      The FSRef of the item
 *    
 *    inRoles:
 *      Reserved for future use. To ensure compatibility, pass
 *      kLSRolesAll for this argument.
 *    
 *    inAttributeName:
 *      The name of the attribute to set
 *    
 *    inValue:
 *      The new value of the attribute. If NULL, removes the attribute
 *      from the item.
 *  
 *  Result:
 *    an OSStatus value. Returns kLSAttributeNotSettableErr if the
 *    attribute is read-only.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSSetItemAttribute( const (*var*) inItem: FSRef; inRoles: LSRolesMask; inAttributeName: CFStringRef; inValue: CFTypeRef { can be NULL } ): OSStatus; external name '_LSSetItemAttribute';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
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


{ ================================================================================== }
{   API for accessing content and URL handler preferences                            }
{ ================================================================================== }

{
 *  LSCopyDefaultRoleHandlerForContentType
 *  
 *  Returns the application bundle identifier of the default handler 
 *  for the specified content type (UTI), in the specified role(s).
 *  For any role, specify kLSRolesAll. Returns NULL if no handler
 *  is available.
 }
{
 *  LSCopyDefaultRoleHandlerForContentType()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSCopyDefaultRoleHandlerForContentType( inContentType: CFStringRef; inRole: LSRolesMask ): CFStringRef; external name '_LSCopyDefaultRoleHandlerForContentType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSCopyAllRoleHandlersForContentType
 *  
 *  Returns an array of application bundle identifiers for
 *  applications capable of handling the specified content type 
 *  (UTI) with the specified role(s). Application content handling 
 *  capabilities are determined according to the kCFBundleDocumentTypes 
 *  listed in an application's Info.plist). For any role, specify kLSRolesAll. 
 *  Returns NULL if no handlers are available.
 }
{
 *  LSCopyAllRoleHandlersForContentType()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSCopyAllRoleHandlersForContentType( inContentType: CFStringRef; inRole: LSRolesMask ): CFArrayRef; external name '_LSCopyAllRoleHandlersForContentType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSSetDefaultRoleHandlerForContentType
 *  
 *  Sets the user's preferred handler for the specified content
 *  type (UTI) in the specified role(s). For all roles, specify
 *  kLSRolesAll. The handler is specified as an application
 *  bundle identifier.
 }
{
 *  LSSetDefaultRoleHandlerForContentType()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSSetDefaultRoleHandlerForContentType( inContentType: CFStringRef; inRole: LSRolesMask; inHandlerBundleID: CFStringRef ): OSStatus; external name '_LSSetDefaultRoleHandlerForContentType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSHandlerOptions
 *
 *  Options controlling how content handlers are selected.
 *
 *    kLSHandlerOptionsDefault - by default, Launch Services will
 *        use a content item's creator (when available) to select a handler
 *    kLSHandlerOptionsIgnoreCreator - Launch Services will ignore content item 
 *        creator information when selecting a role handler for the specified 
 *        content type
 }
type
	LSHandlerOptions = OptionBits;
const
	kLSHandlerOptionsDefault = 0;
	kLSHandlerOptionsIgnoreCreator = 1;


{
 *  LSGetHandlerOptionsForContentType
 *  
 *  Get the handler options for the specified content type (UTI).
 }
{
 *  LSGetHandlerOptionsForContentType()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSGetHandlerOptionsForContentType( inContentType: CFStringRef ): LSHandlerOptions; external name '_LSGetHandlerOptionsForContentType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSSetHandlerOptionsForContentType
 *  
 *  Set the handler options for the specified content type (UTI).
 }
{
 *  LSSetHandlerOptionsForContentType()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSSetHandlerOptionsForContentType( inContentType: CFStringRef; inOptions: LSHandlerOptions ): OSStatus; external name '_LSSetHandlerOptionsForContentType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSCopyDefaultHandlerForURLScheme
 *  
 *  Returns the bundle identifier of the default handler for
 *  the specified URL scheme. Returns NULL if no handler
 *  is available.
 }
{
 *  LSCopyDefaultHandlerForURLScheme()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSCopyDefaultHandlerForURLScheme( inURLScheme: CFStringRef ): CFStringRef; external name '_LSCopyDefaultHandlerForURLScheme';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSCopyAllHandlersForURLScheme
 *  
 *  Returns an array of application bundle identifiers for
 *  applications capable of handling the specified URL scheme. 
 *  URL handling capability is determined according to the 
 *  kCFBundleURLTypes listed in an application's Info.plist).
 *  Returns NULL if no handlers are available.
 }
{
 *  LSCopyAllHandlersForURLScheme()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSCopyAllHandlersForURLScheme( inURLScheme: CFStringRef ): CFArrayRef; external name '_LSCopyAllHandlersForURLScheme';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{
 *  LSSetDefaultHandlerForURLScheme
 *  
 *  Sets the user's preferred handler for the specified URL
 *  scheme. The handler is specified as an application
 *  bundle identifier.
 }
{
 *  LSSetDefaultHandlerForURLScheme()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSSetDefaultHandlerForURLScheme( inURLScheme: CFStringRef; inHandlerBundleID: CFStringRef ): OSStatus; external name '_LSSetDefaultHandlerForURLScheme';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
