{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QD/Displays.h
 
     Contains:   Display Manager Interfaces.
 
     Version:    Quickdraw-262~1
 
     Copyright:  © 1993-2008 by Apple Inc. all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       Displays.p
=======
     File:       QD/Displays.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Display Manager Interfaces.
 
     Version:    Quickdraw-262~1
 
<<<<<<< HEAD
     Copyright:  © 1993-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1993-2008 by Apple Inc. all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit Displays;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
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
=======
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
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
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,QuickdrawTypes,ColorSyncDeprecated,AEDataModel,ConditionalMacros,Components,Video,AppleEvents,Events,Processes,Dialogs;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

=======


>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{******************* DEPRECATION NOTICE *********************
 *
 * The DisplayMgr API is being deprecated, and should be replaced
 * by the CGDirectDisplay API in the CoreGraphics framework in 
 * ApplicationServices.framework.
 *
 ************************************************************}

type
	DMProcessInfoPtr = UnivPtr;
	DMModalFilterUPP = UnivPtr;
const
{ AppleEvents Core Suite }
	kAESystemConfigNotice = FourCharCode('cnfg'); { Core Suite types }
	kAEDisplayNotice = FourCharCode('dspl');
	kAEDisplaySummary = FourCharCode('dsum');
	keyDMConfigVersion = FourCharCode('dmcv');
	keyDMConfigFlags = FourCharCode('dmcf');
	keyDMConfigReserved = FourCharCode('dmcr');
	keyDisplayID = FourCharCode('dmid');
	keyDisplayComponent = FourCharCode('dmdc');
	keyDisplayDevice = FourCharCode('dmdd');
	keyDisplayFlags = FourCharCode('dmdf');
	keyDisplayMode = FourCharCode('dmdm');
	keyDisplayModeReserved = FourCharCode('dmmr');
	keyDisplayReserved = FourCharCode('dmdr');
	keyDisplayMirroredId = FourCharCode('dmmi');
	keyDeviceFlags = FourCharCode('dddf');
	keyDeviceDepthMode = FourCharCode('dddm');
	keyDeviceRect = FourCharCode('dddr');
	keyPixMapRect = FourCharCode('dpdr');
	keyPixMapHResolution = FourCharCode('dphr');
	keyPixMapVResolution = FourCharCode('dpvr');
	keyPixMapPixelType = FourCharCode('dppt');
	keyPixMapPixelSize = FourCharCode('dpps');
	keyPixMapCmpCount = FourCharCode('dpcc');
	keyPixMapCmpSize = FourCharCode('dpcs');
	keyPixMapAlignment = FourCharCode('dppa');
	keyPixMapResReserved = FourCharCode('dprr');
	keyPixMapReserved = FourCharCode('dppr');
	keyPixMapColorTableSeed = FourCharCode('dpct');
	keySummaryMenubar = FourCharCode('dsmb');
	keySummaryChanges = FourCharCode('dsch');
	keyDisplayOldConfig = FourCharCode('dold');
	keyDisplayNewConfig = FourCharCode('dnew');

const
	dmOnlyActiveDisplays = true;
	dmAllDisplays = false;
<<<<<<< HEAD


const
{ DMSendDependentNotification notifyClass }
	kDependentNotifyClassShowCursor = FourCharCode('shcr'); { When display mgr shows a hidden cursor during an unmirror }
	kDependentNotifyClassDriverOverride = FourCharCode('ndrv'); { When a driver is overridden }
	kDependentNotifyClassDisplayMgrOverride = FourCharCode('dmgr'); { When display manager is upgraded }
	kDependentNotifyClassProfileChanged = FourCharCode('prof'); { When DMSetProfileByAVID is called }


const
{ Switch Flags }
	kNoSwitchConfirmBit = 0;    { Flag indicating that there is no need to confirm a switch to this mode }
	kDepthNotAvailableBit = 1;    { Current depth not available in new mode }
	kShowModeBit = 3;    { Show this mode even though it requires a confirm. }
	kModeNotResizeBit = 4;    { Do not use this mode to resize display (for cards that mode drives a different connector). }
	kNeverShowModeBit = 5;     { This mode should not be shown in the user interface. }

{    Summary Change Flags (sticky bits indicating an operation was performed)
    For example, moving a display then moving it back will still set the kMovedDisplayBit.
}
const
	kBeginEndConfigureBit = 0;
	kMovedDisplayBit = 1;
	kSetMainDisplayBit = 2;
	kSetDisplayModeBit = 3;
	kAddDisplayBit = 4;
	kRemoveDisplayBit = 5;
	kNewDisplayBit = 6;
	kDisposeDisplayBit = 7;
	kEnabledDisplayBit = 8;
	kDisabledDisplayBit = 9;
	kMirrorDisplayBit = 10;
	kUnMirrorDisplayBit = 11;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


const
{ Notification Messages for extended call back routines }
	kDMNotifyRequestConnectionProbe = 0;  { Like kDMNotifyRequestDisplayProbe only not for smart displays (used in wake before all busses are awake) }
	kDMNotifyInstalled = 1;    { At install time }
	kDMNotifyEvent = 2;    { Post change time }
	kDMNotifyRemoved = 3;    { At remove time }
	kDMNotifyPrep = 4;    { Pre change time }
	kDMNotifyExtendEvent = 5;    { Allow registrees to extend apple event before it is sent }
	kDMNotifyDependents = 6;    { Minor notification check without full update }
	kDMNotifySuspendConfigure = 7;    { Temporary end of configuration }
	kDMNotifyResumeConfigure = 8;    { Resume configuration }
	kDMNotifyRequestDisplayProbe = 9;    { Request smart displays re-probe (used in sleep and hot plugging) }
	kDMNotifyDisplayWillSleep = 10;   { Mac OS X only }
	kDMNotifyDisplayDidWake = 11;   { Mac OS X only }
                                        { Notification Flags }
	kExtendedNotificationProc = 1 shl 16;


{ types for notifyType }
const
	kFullNotify = 0;    { This is the appleevent whole nine yards notify }
	kFullDependencyNotify = 1;     { Only sends to those who want to know about interrelated functionality (used for updating UI) }

{ DisplayID/DeviceID constants }
const
	kDummyDeviceID = $00FF; { This is the ID of the dummy display, used when the last “real” display is disabled.}
	kInvalidDisplayID = $0000; { This is the invalid ID}
	kFirstDisplayID = $0100;

const
{ bits for panelListFlags }
	kAllowDuplicatesBit = 0;

const
{ bits for nameFlags }
	kSuppressNumberBit = 0;
	kSuppressNumberMask = 1;
	kForceNumberBit = 1;
	kForceNumberMask = 2;
	kSuppressNameBit = 2;
	kSuppressNameMask = 4;

{ DMGetNameByAVID masks}
const
	kDMSupressNumbersMask = 1 shl 0; { Supress the numbers and return only names}
	kDMForceNumbersMask = 1 shl 1; { Force numbers to always be shown (even on single display configs)}
	kDMSupressNameMask = 1 shl 2; { Supress the names and return only numbers.}


{ Constants for fidelity checks }
const
	kNoFidelity = 0;
	kMinimumFidelity = 1;
	kDefaultFidelity = 500;  { I'm just picking a number for Apple default panels and engines}
	kDefaultManufacturerFidelity = 1000;  { I'm just picking a number for Manufacturer's panels and engines (overrides apple defaults)}

const
	kAnyPanelType = 0;    { Pass to DMNewEngineList for list of all panels (as opposed to specific types)}
	kAnyEngineType = 0;    { Pass to DMNewEngineList for list of all engines}
	kAnyDeviceType = 0;    { Pass to DMNewDeviceList for list of all devices}
	kAnyPortType = 0;     { Pass to DMNewDevicePortList for list of all devices}

{ portListFlags for DM_NewDevicePortList }
const
{ Should offline devices be put into the port list (such as dummy display) }
	kPLIncludeOfflineDevicesBit = 0;
=======
>>>>>>> graemeg/cpstrnew


{ confirmFlags for DMConfirmConfiguration }
const
	kForceConfirmBit = 0;    { Force a confirm dialog }
	kForceConfirmMask = 1 shl kForceConfirmBit;


{ Flags for displayModeFlags }
const
<<<<<<< HEAD
=======
{ Notification Messages for extended call back routines }
	kDMNotifyRequestConnectionProbe = 0;  { Like kDMNotifyRequestDisplayProbe only not for smart displays (used in wake before all busses are awake) }
	kDMNotifyInstalled = 1;    { At install time }
	kDMNotifyEvent = 2;    { Post change time }
	kDMNotifyRemoved = 3;    { At remove time }
	kDMNotifyPrep = 4;    { Pre change time }
	kDMNotifyExtendEvent = 5;    { Allow registrees to extend apple event before it is sent }
	kDMNotifyDependents = 6;    { Minor notification check without full update }
	kDMNotifySuspendConfigure = 7;    { Temporary end of configuration }
	kDMNotifyResumeConfigure = 8;    { Resume configuration }
	kDMNotifyRequestDisplayProbe = 9;    { Request smart displays re-probe (used in sleep and hot plugging) }
	kDMNotifyDisplayWillSleep = 10;   { Mac OS X only }
	kDMNotifyDisplayDidWake = 11;   { Mac OS X only }
                                        { Notification Flags }
	kExtendedNotificationProc = 1 shl 16;


{ types for notifyType }
const
	kFullNotify = 0;    { This is the appleevent whole nine yards notify }
	kFullDependencyNotify = 1;     { Only sends to those who want to know about interrelated functionality (used for updating UI) }

{ DisplayID/DeviceID constants }
const
	kDummyDeviceID = $00FF; { This is the ID of the dummy display, used when the last “real” display is disabled.}
	kInvalidDisplayID = $0000; { This is the invalid ID}
	kFirstDisplayID = $0100;

const
{ bits for panelListFlags }
	kAllowDuplicatesBit = 0;

const
{ bits for nameFlags }
	kSuppressNumberBit = 0;
	kSuppressNumberMask = 1;
	kForceNumberBit = 1;
	kForceNumberMask = 2;
	kSuppressNameBit = 2;
	kSuppressNameMask = 4;

{ DMGetNameByAVID masks}
const
	kDMSupressNumbersMask = 1 shl 0; { Supress the numbers and return only names}
	kDMForceNumbersMask = 1 shl 1; { Force numbers to always be shown (even on single display configs)}
	kDMSupressNameMask = 1 shl 2; { Supress the names and return only numbers.}
=======
>>>>>>> graemeg/cpstrnew


{ Constants for fidelity checks }
const
<<<<<<< HEAD
=======
{ Notification Messages for extended call back routines }
	kDMNotifyRequestConnectionProbe = 0;  { Like kDMNotifyRequestDisplayProbe only not for smart displays (used in wake before all busses are awake) }
	kDMNotifyInstalled = 1;    { At install time }
	kDMNotifyEvent = 2;    { Post change time }
	kDMNotifyRemoved = 3;    { At remove time }
	kDMNotifyPrep = 4;    { Pre change time }
	kDMNotifyExtendEvent = 5;    { Allow registrees to extend apple event before it is sent }
	kDMNotifyDependents = 6;    { Minor notification check without full update }
	kDMNotifySuspendConfigure = 7;    { Temporary end of configuration }
	kDMNotifyResumeConfigure = 8;    { Resume configuration }
	kDMNotifyRequestDisplayProbe = 9;    { Request smart displays re-probe (used in sleep and hot plugging) }
	kDMNotifyDisplayWillSleep = 10;   { Mac OS X only }
	kDMNotifyDisplayDidWake = 11;   { Mac OS X only }
                                        { Notification Flags }
	kExtendedNotificationProc = 1 shl 16;


{ types for notifyType }
const
	kFullNotify = 0;    { This is the appleevent whole nine yards notify }
	kFullDependencyNotify = 1;     { Only sends to those who want to know about interrelated functionality (used for updating UI) }

{ DisplayID/DeviceID constants }
const
	kDummyDeviceID = $00FF; { This is the ID of the dummy display, used when the last “real” display is disabled.}
	kInvalidDisplayID = $0000; { This is the invalid ID}
	kFirstDisplayID = $0100;

const
{ bits for panelListFlags }
	kAllowDuplicatesBit = 0;

const
{ bits for nameFlags }
	kSuppressNumberBit = 0;
	kSuppressNumberMask = 1;
	kForceNumberBit = 1;
	kForceNumberMask = 2;
	kSuppressNameBit = 2;
	kSuppressNameMask = 4;

{ DMGetNameByAVID masks}
const
	kDMSupressNumbersMask = 1 shl 0; { Supress the numbers and return only names}
	kDMForceNumbersMask = 1 shl 1; { Force numbers to always be shown (even on single display configs)}
	kDMSupressNameMask = 1 shl 2; { Supress the names and return only numbers.}


{ Constants for fidelity checks }
const
>>>>>>> graemeg/cpstrnew
	kNoFidelity = 0;
	kMinimumFidelity = 1;
	kDefaultFidelity = 500;  { I'm just picking a number for Apple default panels and engines}
	kDefaultManufacturerFidelity = 1000;  { I'm just picking a number for Manufacturer's panels and engines (overrides apple defaults)}

const
	kAnyPanelType = 0;    { Pass to DMNewEngineList for list of all panels (as opposed to specific types)}
	kAnyEngineType = 0;    { Pass to DMNewEngineList for list of all engines}
	kAnyDeviceType = 0;    { Pass to DMNewDeviceList for list of all devices}
	kAnyPortType = 0;     { Pass to DMNewDevicePortList for list of all devices}

{ portListFlags for DM_NewDevicePortList }
const
{ Should offline devices be put into the port list (such as dummy display) }
	kPLIncludeOfflineDevicesBit = 0;
=======
>>>>>>> graemeg/cpstrnew


{ confirmFlags for DMConfirmConfiguration }
const
<<<<<<< HEAD
=======
=======


const
{ DMSendDependentNotification notifyClass }
	kDependentNotifyClassShowCursor = FourCharCode('shcr'); { When display mgr shows a hidden cursor during an unmirror }
	kDependentNotifyClassDriverOverride = FourCharCode('ndrv'); { When a driver is overridden }
	kDependentNotifyClassDisplayMgrOverride = FourCharCode('dmgr'); { When display manager is upgraded }
	kDependentNotifyClassProfileChanged = FourCharCode('prof'); { When DMSetProfileByAVID is called }


const
{ Switch Flags }
	kNoSwitchConfirmBit = 0;    { Flag indicating that there is no need to confirm a switch to this mode }
	kDepthNotAvailableBit = 1;    { Current depth not available in new mode }
	kShowModeBit = 3;    { Show this mode even though it requires a confirm. }
	kModeNotResizeBit = 4;    { Do not use this mode to resize display (for cards that mode drives a different connector). }
	kNeverShowModeBit = 5;     { This mode should not be shown in the user interface. }

{    Summary Change Flags (sticky bits indicating an operation was performed)
    For example, moving a display then moving it back will still set the kMovedDisplayBit.
}
const
	kBeginEndConfigureBit = 0;
	kMovedDisplayBit = 1;
	kSetMainDisplayBit = 2;
	kSetDisplayModeBit = 3;
	kAddDisplayBit = 4;
	kRemoveDisplayBit = 5;
	kNewDisplayBit = 6;
	kDisposeDisplayBit = 7;
	kEnabledDisplayBit = 8;
	kDisabledDisplayBit = 9;
	kMirrorDisplayBit = 10;
	kUnMirrorDisplayBit = 11;


const
>>>>>>> origin/cpstrnew
{ Notification Messages for extended call back routines }
	kDMNotifyRequestConnectionProbe = 0;  { Like kDMNotifyRequestDisplayProbe only not for smart displays (used in wake before all busses are awake) }
	kDMNotifyInstalled = 1;    { At install time }
	kDMNotifyEvent = 2;    { Post change time }
	kDMNotifyRemoved = 3;    { At remove time }
	kDMNotifyPrep = 4;    { Pre change time }
	kDMNotifyExtendEvent = 5;    { Allow registrees to extend apple event before it is sent }
	kDMNotifyDependents = 6;    { Minor notification check without full update }
	kDMNotifySuspendConfigure = 7;    { Temporary end of configuration }
	kDMNotifyResumeConfigure = 8;    { Resume configuration }
	kDMNotifyRequestDisplayProbe = 9;    { Request smart displays re-probe (used in sleep and hot plugging) }
	kDMNotifyDisplayWillSleep = 10;   { Mac OS X only }
	kDMNotifyDisplayDidWake = 11;   { Mac OS X only }
                                        { Notification Flags }
	kExtendedNotificationProc = 1 shl 16;


{ types for notifyType }
const
	kFullNotify = 0;    { This is the appleevent whole nine yards notify }
	kFullDependencyNotify = 1;     { Only sends to those who want to know about interrelated functionality (used for updating UI) }

{ DisplayID/DeviceID constants }
const
	kDummyDeviceID = $00FF; { This is the ID of the dummy display, used when the last “real” display is disabled.}
	kInvalidDisplayID = $0000; { This is the invalid ID}
	kFirstDisplayID = $0100;

const
{ bits for panelListFlags }
	kAllowDuplicatesBit = 0;

const
{ bits for nameFlags }
	kSuppressNumberBit = 0;
	kSuppressNumberMask = 1;
	kForceNumberBit = 1;
	kForceNumberMask = 2;
	kSuppressNameBit = 2;
	kSuppressNameMask = 4;

{ DMGetNameByAVID masks}
const
	kDMSupressNumbersMask = 1 shl 0; { Supress the numbers and return only names}
	kDMForceNumbersMask = 1 shl 1; { Force numbers to always be shown (even on single display configs)}
	kDMSupressNameMask = 1 shl 2; { Supress the names and return only numbers.}


{ Constants for fidelity checks }
const
	kNoFidelity = 0;
	kMinimumFidelity = 1;
	kDefaultFidelity = 500;  { I'm just picking a number for Apple default panels and engines}
	kDefaultManufacturerFidelity = 1000;  { I'm just picking a number for Manufacturer's panels and engines (overrides apple defaults)}

const
	kAnyPanelType = 0;    { Pass to DMNewEngineList for list of all panels (as opposed to specific types)}
	kAnyEngineType = 0;    { Pass to DMNewEngineList for list of all engines}
	kAnyDeviceType = 0;    { Pass to DMNewDeviceList for list of all devices}
	kAnyPortType = 0;     { Pass to DMNewDevicePortList for list of all devices}

{ portListFlags for DM_NewDevicePortList }
const
{ Should offline devices be put into the port list (such as dummy display) }
	kPLIncludeOfflineDevicesBit = 0;


{ confirmFlags for DMConfirmConfiguration }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kForceConfirmBit = 0;    { Force a confirm dialog }
	kForceConfirmMask = 1 shl kForceConfirmBit;


{ Flags for displayModeFlags }
const
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
	kDisplayModeListNotPreferredBit = 0;
	kDisplayModeListNotPreferredMask = 1 shl kDisplayModeListNotPreferredBit;


{ Flags for itemFlags }
const
	kComponentListNotPreferredBit = 0;
	kComponentListNotPreferredMask = 1 shl kComponentListNotPreferredBit;

const
	kDisplayTimingInfoVersionZero = 1;
	kDisplayTimingInfoReservedCountVersionZero = 16;
	kDisplayModeEntryVersionZero = 0;    { displayModeVersion - original version}
	kDisplayModeEntryVersionOne = 1;     { displayModeVersion - added displayModeOverrideInfo}


const
	kMakeAndModelReservedCount = 4;     { Number of reserved fields}


{ Display Gestalt for DMDisplayGestalt}
const
	kDisplayGestaltDisplayCommunicationAttr = FourCharCode('comm');
	kDisplayGestaltForbidI2CMask = 1 shl 0; { Some displays have firmware problems if they get I2C communication.  If this bit is set, then I2C communication is forbidden}
	kDisplayGestaltUseI2CPowerMask = 1 shl 1; { Some displays require I2C power settings (most use DPMS).}
	kDisplayGestaltCalibratorAttr = FourCharCode('cali');
	kDisplayGestaltBrightnessAffectsGammaMask = 1 shl 0; { Used by default calibrator (should we show brightness panel) }
	kDisplayGestaltViewAngleAffectsGammaMask = 1 shl 1; { Currently not used by color sync}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew


type
	DMFidelityType = UInt32;
{
   AVID is an ID for ports and devices the old DisplayID type
    is carried on for compatibility
}
<<<<<<< HEAD


type
=======


type
	DMFidelityType = UInt32;
{
   AVID is an ID for ports and devices the old DisplayID type
    is carried on for compatibility
}


type
>>>>>>> graemeg/cpstrnew
=======


type
	DMFidelityType = UInt32;
{
   AVID is an ID for ports and devices the old DisplayID type
    is carried on for compatibility
}


type
>>>>>>> graemeg/cpstrnew
=======


type
>>>>>>> graemeg/cpstrnew
=======


type
	DMFidelityType = UInt32;
{
   AVID is an ID for ports and devices the old DisplayID type
    is carried on for compatibility
}


type
>>>>>>> origin/cpstrnew
	DMListType = UnivPtr;
	DMListIndexType = UInt32;
	AVPowerStateRec = VDPowerStateRec;
	AVPowerStateRecPtr = ^AVPowerStateRec;
	AVPowerStatePtr = VDPowerStateRecPtr;
	DMDisplayTimingInfoRecPtr = ^DMDisplayTimingInfoRec;
	DMDisplayTimingInfoRec = record
		timingInfoVersion: UInt32;
		timingInfoAttributes: UInt32;   { Flags }
		timingInfoRelativeQuality: SInt32; { quality of the timing }
		timingInfoRelativeDefault: SInt32; { relative default of the timing }

		timingInfoReserved: array [0..15] of UInt32;  
	end;
type
	DMDisplayTimingInfoPtr = ^DMDisplayTimingInfoRec;

type
	DMComponentListEntryRecPtr = ^DMComponentListEntryRec;
	DMComponentListEntryRec = record
		itemID: DisplayIDType;                 { DisplayID Manager}
		itemComponent: Component;          { Component Manager}
		itemDescription: ComponentDescription;      { We can always construct this if we use something beyond the compontent mgr.}

		itemClass: ResType;              { Class of group to put this panel (eg geometry/color/etc for panels, brightness/contrast for engines, video out/sound/etc for devices)}
		itemFidelity: DMFidelityType;           { How good is this item for the specified search?}
		itemSubClass: ResType;           { Subclass of group to put this panel.  Can use to do sub-grouping (eg volume for volume panel and mute panel)}
		itemSort: Point;               { Set to 0 - future to sort the items in a sub group.}

		itemFlags: UInt32;              { Set to 0 (future expansion)}
		itemReserved: ResType;           { What kind of code does the itemReference point to  (right now - kPanelEntryTypeComponentMgr only)}
		itemFuture1: UInt32;            { Set to 0 (future expansion - probably an alternate code style)}
		itemFuture2: UInt32;            { Set to 0 (future expansion - probably an alternate code style)}
		itemFuture3: UInt32;            { Set to 0 (future expansion - probably an alternate code style)}
		itemFuture4: UInt32;            { Set to 0 (future expansion - probably an alternate code style)}
	end;
type
	DMComponentListEntryPtr = DMComponentListEntryRecPtr;
{ ••• Move AVLocationRec to AVComponents.i AFTER AVComponents.i is created}
type
	AVLocationRecPtr = ^AVLocationRec;
	AVLocationRec = record
		locationConstant: UInt32;       { Set to 0 (future expansion - probably an alternate code style)}
	end;
type
	AVLocationPtr = AVLocationRecPtr;
	DMDepthInfoRecPtr = ^DMDepthInfoRec;
	DMDepthInfoRec = record
		depthSwitchInfo: VDSwitchInfoPtr;        { This is the switch mode to choose this timing/depth }
		depthVPBlock: VPBlockPtr;           { VPBlock (including size, depth and format) }
		depthFlags: UInt32;             { VDVideoParametersInfoRec.csDepthFlags  }
		depthReserved1: UInt32;         { Reserved }
		depthReserved2: UInt32;         { Reserved }
	end;
type
	DMDepthInfoPtr = DMDepthInfoRecPtr;
	DMDepthInfoBlockRecPtr = ^DMDepthInfoBlockRec;
	DMDepthInfoBlockRec = record
		depthBlockCount: UInt32;        { How many depths are there? }
		depthVPBlock: DMDepthInfoPtr;           { Array of DMDepthInfoRec }
		depthBlockFlags: UInt32;        { Reserved }
		depthBlockReserved1: UInt32;    { Reserved }
		depthBlockReserved2: UInt32;    { Reserved }
	end;
type
	DMDepthInfoBlockPtr = DMDepthInfoBlockRecPtr;
	DMDisplayModeListEntryRecPtr = ^DMDisplayModeListEntryRec;
	DMDisplayModeListEntryRec = record
		displayModeFlags: UInt32;
		displayModeSwitchInfo: VDSwitchInfoPtr;
		displayModeResolutionInfo: VDResolutionInfoPtr;
		displayModeTimingInfo: VDTimingInfoPtr;
		displayModeDepthBlockInfo: DMDepthInfoBlockPtr; { Information about all the depths}
		displayModeVersion: UInt32;     { What version is this record (now kDisplayModeEntryVersionOne)}
		displayModeName: StringPtr;        { Name of the timing mode}
		displayModeDisplayInfo: DMDisplayTimingInfoPtr; { Information from the display.}
	end;
type
	DMDisplayModeListEntryPtr = DMDisplayModeListEntryRecPtr;

type
	DependentNotifyRecPtr = ^DependentNotifyRec;
	DependentNotifyRec = record
		notifyType: ResType;             { What type was the engine that made the change (may be zero)}
		notifyClass: ResType;            { What class was the change (eg geometry, color etc)}
		notifyPortID: DisplayIDType;           { Which device was touched (kInvalidDisplayID -> all or none)}
		notifyComponent: ComponentInstance;        { What engine did it (may be 0)?}

		notifyVersion: UInt32;          { Set to 0 (future expansion)}
		notifyFlags: UInt32;            { Set to 0 (future expansion)}
		notifyReserved: UInt32;         { Set to 0 (future expansion)}
		notifyFuture: UInt32;           { Set to 0 (future expansion)}
	end;
type
	DependentNotifyPtr = DependentNotifyRecPtr;

type
	DMMakeAndModelRecPtr = ^DMMakeAndModelRec;
	DMMakeAndModelRec = record
		manufacturer: ResType;
		model: UInt32;
		serialNumber: UInt32;
		manufactureDate: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD

		makeReserved: array [0..3] of UInt32;
	end;
type
	DMMakeAndModelPtr = DMMakeAndModelRecPtr;
{ DMNewDisplayList displayListIncludeFlags}
const
	kIncludeOnlineActiveDisplaysMask = 1 shl 0;
	kIncludeOnlineDisabledDisplaysMask = 1 shl 1;
	kIncludeOfflineDisplaysMask = 1 shl 2;
	kIncludeOfflineDummyDisplaysMask = 1 shl 3;
	kIncludeHardwareMirroredDisplaysMask = 1 shl 4;
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/cpstrnew
=======
=======

		makeReserved: array [0..3] of UInt32;
	end;
type
	DMMakeAndModelPtr = DMMakeAndModelRecPtr;
{ DMNewDisplayList displayListIncludeFlags}
const
	kIncludeOnlineActiveDisplaysMask = 1 shl 0;
	kIncludeOnlineDisabledDisplaysMask = 1 shl 1;
	kIncludeOfflineDisplaysMask = 1 shl 2;
	kIncludeOfflineDummyDisplaysMask = 1 shl 3;
	kIncludeHardwareMirroredDisplaysMask = 1 shl 4;

>>>>>>> graemeg/cpstrnew

const
{ modeListFlags for DMNewDisplayModeList }
	kDMModeListIncludeAllModesMask = 1 shl 0; { Include all timing modes not _explicitly_ excluded (see other bits)}
	kDMModeListIncludeOfflineModesMask = 1 shl 1;
	kDMModeListExcludeDriverModesMask = 1 shl 2; { Exclude old-style timing modes (cscGetNextResolution/kDisplayModeIDFindFirstResolution modes)}
	kDMModeListExcludeDisplayModesMask = 1 shl 3; { Exclude timing modes that come from the display (always arbritrary timing modes)}
	kDMModeListExcludeCustomModesMask = 1 shl 4; { Exclude custom modes that came neither from the driver or display (need a better name)}
	kDMModeListPreferStretchedModesMask = 1 shl 5; { Prefer modes that are stretched over modes that are letterboxed when setting kDisplayModeListNotPreferredBit}
	kDMModeListPreferSafeModesMask = 1 shl 6; { Prefer modes that are safe over modes that are not when setting kDisplayModeListNotPreferredBit}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
const
{ modeListFlags for DMNewDisplayModeList }
	kDMModeListIncludeAllModesMask = 1 shl 0; { Include all timing modes not _explicitly_ excluded (see other bits)}
	kDMModeListIncludeOfflineModesMask = 1 shl 1;
	kDMModeListExcludeDriverModesMask = 1 shl 2; { Exclude old-style timing modes (cscGetNextResolution/kDisplayModeIDFindFirstResolution modes)}
	kDMModeListExcludeDisplayModesMask = 1 shl 3; { Exclude timing modes that come from the display (always arbritrary timing modes)}
	kDMModeListExcludeCustomModesMask = 1 shl 4; { Exclude custom modes that came neither from the driver or display (need a better name)}
	kDMModeListPreferStretchedModesMask = 1 shl 5; { Prefer modes that are stretched over modes that are letterboxed when setting kDisplayModeListNotPreferredBit}
	kDMModeListPreferSafeModesMask = 1 shl 6; { Prefer modes that are safe over modes that are not when setting kDisplayModeListNotPreferredBit}

=======
>>>>>>> graemeg/cpstrnew

=======

		makeReserved: array [0..3] of UInt32;
	end;
type
	DMMakeAndModelPtr = DMMakeAndModelRecPtr;
{ DMNewDisplayList displayListIncludeFlags}
const
	kIncludeOnlineActiveDisplaysMask = 1 shl 0;
	kIncludeOnlineDisabledDisplaysMask = 1 shl 1;
	kIncludeOfflineDisplaysMask = 1 shl 2;
	kIncludeOfflineDummyDisplaysMask = 1 shl 3;
	kIncludeHardwareMirroredDisplaysMask = 1 shl 4;


const
{ modeListFlags for DMNewDisplayModeList }
	kDMModeListIncludeAllModesMask = 1 shl 0; { Include all timing modes not _explicitly_ excluded (see other bits)}
	kDMModeListIncludeOfflineModesMask = 1 shl 1;
	kDMModeListExcludeDriverModesMask = 1 shl 2; { Exclude old-style timing modes (cscGetNextResolution/kDisplayModeIDFindFirstResolution modes)}
	kDMModeListExcludeDisplayModesMask = 1 shl 3; { Exclude timing modes that come from the display (always arbritrary timing modes)}
	kDMModeListExcludeCustomModesMask = 1 shl 4; { Exclude custom modes that came neither from the driver or display (need a better name)}
	kDMModeListPreferStretchedModesMask = 1 shl 5; { Prefer modes that are stretched over modes that are letterboxed when setting kDisplayModeListNotPreferredBit}
	kDMModeListPreferSafeModesMask = 1 shl 6; { Prefer modes that are safe over modes that are not when setting kDisplayModeListNotPreferredBit}


>>>>>>> origin/cpstrnew
{ DMNewDisplayList displayListFlags}
=======
uses MacTypes,Quickdraw,CMTypes,AEDataModel,ConditionalMacros,Components,Video,CMApplication,AppleEvents,Events,Processes,Dialogs;


{$ALIGN MAC68K}


type
	DMProcessInfoPtr					= ProcessSerialNumberPtr;
	DMModalFilterUPP					= ModalFilterUPP;


const
																{  AppleEvents Core Suite  }
	kAESystemConfigNotice		= FourCharCode('cnfg');						{  Core Suite types  }
	kAEDisplayNotice			= FourCharCode('dspl');
	kAEDisplaySummary			= FourCharCode('dsum');
	keyDMConfigVersion			= FourCharCode('dmcv');
	keyDMConfigFlags			= FourCharCode('dmcf');
	keyDMConfigReserved			= FourCharCode('dmcr');
	keyDisplayID				= FourCharCode('dmid');
	keyDisplayComponent			= FourCharCode('dmdc');
	keyDisplayDevice			= FourCharCode('dmdd');
	keyDisplayFlags				= FourCharCode('dmdf');
	keyDisplayMode				= FourCharCode('dmdm');
	keyDisplayModeReserved		= FourCharCode('dmmr');
	keyDisplayReserved			= FourCharCode('dmdr');
	keyDisplayMirroredId		= FourCharCode('dmmi');
	keyDeviceFlags				= FourCharCode('dddf');
	keyDeviceDepthMode			= FourCharCode('dddm');
	keyDeviceRect				= FourCharCode('dddr');
	keyPixMapRect				= FourCharCode('dpdr');
	keyPixMapHResolution		= FourCharCode('dphr');
	keyPixMapVResolution		= FourCharCode('dpvr');
	keyPixMapPixelType			= FourCharCode('dppt');
	keyPixMapPixelSize			= FourCharCode('dpps');
	keyPixMapCmpCount			= FourCharCode('dpcc');
	keyPixMapCmpSize			= FourCharCode('dpcs');
	keyPixMapAlignment			= FourCharCode('dppa');
	keyPixMapResReserved		= FourCharCode('dprr');
	keyPixMapReserved			= FourCharCode('dppr');
	keyPixMapColorTableSeed		= FourCharCode('dpct');
	keySummaryMenubar			= FourCharCode('dsmb');
	keySummaryChanges			= FourCharCode('dsch');
	keyDisplayOldConfig			= FourCharCode('dold');
	keyDisplayNewConfig			= FourCharCode('dnew');

	dmOnlyActiveDisplays		= true;
	dmAllDisplays				= false;


																{  DMSendDependentNotification notifyClass  }
	kDependentNotifyClassShowCursor = FourCharCode('shcr');					{  When display mgr shows a hidden cursor during an unmirror  }
	kDependentNotifyClassDriverOverride = FourCharCode('ndrv');				{  When a driver is overridden  }
	kDependentNotifyClassDisplayMgrOverride = FourCharCode('dmgr');			{  When display manager is upgraded  }
	kDependentNotifyClassProfileChanged = FourCharCode('prof');				{  When DMSetProfileByAVID is called  }


																{  Switch Flags  }
	kNoSwitchConfirmBit			= 0;							{  Flag indicating that there is no need to confirm a switch to this mode  }
	kDepthNotAvailableBit		= 1;							{  Current depth not available in new mode  }
	kShowModeBit				= 3;							{  Show this mode even though it requires a confirm.  }
	kModeNotResizeBit			= 4;							{  Do not use this mode to resize display (for cards that mode drives a different connector).  }
	kNeverShowModeBit			= 5;							{  This mode should not be shown in the user interface.  }

	{	    Summary Change Flags (sticky bits indicating an operation was performed)
	    For example, moving a display then moving it back will still set the kMovedDisplayBit.
		}
	kBeginEndConfigureBit		= 0;
	kMovedDisplayBit			= 1;
	kSetMainDisplayBit			= 2;
	kSetDisplayModeBit			= 3;
	kAddDisplayBit				= 4;
	kRemoveDisplayBit			= 5;
	kNewDisplayBit				= 6;
	kDisposeDisplayBit			= 7;
	kEnabledDisplayBit			= 8;
	kDisabledDisplayBit			= 9;
	kMirrorDisplayBit			= 10;
	kUnMirrorDisplayBit			= 11;


																{  Notification Messages for extended call back routines  }
	kDMNotifyRequestConnectionProbe = 0;						{  Like kDMNotifyRequestDisplayProbe only not for smart displays (used in wake before all busses are awake)  }
	kDMNotifyInstalled			= 1;							{  At install time  }
	kDMNotifyEvent				= 2;							{  Post change time  }
	kDMNotifyRemoved			= 3;							{  At remove time  }
	kDMNotifyPrep				= 4;							{  Pre change time  }
	kDMNotifyExtendEvent		= 5;							{  Allow registrees to extend apple event before it is sent  }
	kDMNotifyDependents			= 6;							{  Minor notification check without full update  }
	kDMNotifySuspendConfigure	= 7;							{  Temporary end of configuration  }
	kDMNotifyResumeConfigure	= 8;							{  Resume configuration  }
	kDMNotifyRequestDisplayProbe = 9;							{  Request smart displays re-probe (used in sleep and hot plugging)  }
																{  Notification Flags  }
	kExtendedNotificationProc	= $00010000;


	{	 types for notifyType 	}
	kFullNotify					= 0;							{  This is the appleevent whole nine yards notify  }
	kFullDependencyNotify		= 1;							{  Only sends to those who want to know about interrelated functionality (used for updating UI)  }

	{	 DisplayID/DeviceID constants 	}
	kDummyDeviceID				= $00FF;						{  This is the ID of the dummy display, used when the last “real” display is disabled. }
	kInvalidDisplayID			= $0000;						{  This is the invalid ID }
	kFirstDisplayID				= $0100;

																{  bits for panelListFlags  }
	kAllowDuplicatesBit			= 0;

																{  bits for nameFlags  }
	kSuppressNumberBit			= 0;
	kSuppressNumberMask			= 1;
	kForceNumberBit				= 1;
	kForceNumberMask			= 2;
	kSuppressNameBit			= 2;
	kSuppressNameMask			= 4;

	{  DMGetNameByAVID masks }
	kDMSupressNumbersMask		= $01;							{  Supress the numbers and return only names }
	kDMForceNumbersMask			= $02;							{  Force numbers to always be shown (even on single display configs) }
	kDMSupressNameMask			= $04;							{  Supress the names and return only numbers. }


	{	 Constants for fidelity checks 	}
	kNoFidelity					= 0;
	kMinimumFidelity			= 1;
	kDefaultFidelity			= 500;							{  I'm just picking a number for Apple default panels and engines }
	kDefaultManufacturerFidelity = 1000;						{  I'm just picking a number for Manufacturer's panels and engines (overrides apple defaults) }

	kAnyPanelType				= 0;							{  Pass to DMNewEngineList for list of all panels (as opposed to specific types) }
	kAnyEngineType				= 0;							{  Pass to DMNewEngineList for list of all engines }
	kAnyDeviceType				= 0;							{  Pass to DMNewDeviceList for list of all devices }
	kAnyPortType				= 0;							{  Pass to DMNewDevicePortList for list of all devices }

	{	 portListFlags for DM_NewDevicePortList 	}
																{  Should offline devices be put into the port list (such as dummy display)  }
	kPLIncludeOfflineDevicesBit	= 0;


	{	 confirmFlags for DMConfirmConfiguration 	}
	kForceConfirmBit			= 0;							{  Force a confirm dialog  }
	kForceConfirmMask			= $01;


	{	 Flags for displayModeFlags 	}
	kDisplayModeListNotPreferredBit = 0;
	kDisplayModeListNotPreferredMask = $01;


	{	 Flags for itemFlags 	}
	kComponentListNotPreferredBit = 0;
	kComponentListNotPreferredMask = $01;

	kDisplayTimingInfoVersionZero = 1;
	kDisplayTimingInfoReservedCountVersionZero = 16;
	kDisplayModeEntryVersionZero = 0;							{  displayModeVersion - original version }
	kDisplayModeEntryVersionOne	= 1;							{  displayModeVersion - added displayModeOverrideInfo }


	kMakeAndModelReservedCount	= 4;							{  Number of reserved fields }


	{  Display Gestalt for DMDisplayGestalt }
	kDisplayGestaltDisplayCommunicationAttr = FourCharCode('comm');
	kDisplayGestaltForbidI2CMask = $01;							{  Some displays have firmware problems if they get I2C communication.  If this bit is set, then I2C communication is forbidden }
	kDisplayGestaltUseI2CPowerMask = $02;						{  Some displays require I2C power settings (most use DPMS). }
	kDisplayGestaltCalibratorAttr = FourCharCode('cali');
	kDisplayGestaltBrightnessAffectsGammaMask = $01;			{  Used by default calibrator (should we show brightness panel)  }
	kDisplayGestaltViewAngleAffectsGammaMask = $02;				{  Currently not used by color sync }


type
	DMFidelityType						= UInt32;
	{
	   AVID is an ID for ports and devices the old DisplayID type
	    is carried on for compatibility
	}


	DMListType							= Ptr;
	DMListIndexType						= UInt32;
	AVPowerStateRec						= VDPowerStateRec;
	AVPowerStateRecPtr 					= ^AVPowerStateRec;
	AVPowerStatePtr						= ^VDPowerStateRec;
	DMDisplayTimingInfoRecPtr = ^DMDisplayTimingInfoRec;
	DMDisplayTimingInfoRec = record
		timingInfoVersion:		UInt32;
		timingInfoAttributes:	UInt32;									{  Flags  }
		timingInfoRelativeQuality: SInt32;								{  quality of the timing  }
		timingInfoRelativeDefault: SInt32;								{  relative default of the timing  }
		timingInfoReserved:		array [0..15] of UInt32;				{  Reserved  }
	end;

	DMDisplayTimingInfoPtr				= ^DMDisplayTimingInfoRec;

	DMComponentListEntryRecPtr = ^DMComponentListEntryRec;
	DMComponentListEntryRec = record
		itemID:					DisplayIDType;							{  DisplayID Manager }
		itemComponent:			Component;								{  Component Manager }
		itemDescription:		ComponentDescription;					{  We can always construct this if we use something beyond the compontent mgr. }
		itemClass:				ResType;								{  Class of group to put this panel (eg geometry/color/etc for panels, brightness/contrast for engines, video out/sound/etc for devices) }
		itemFidelity:			DMFidelityType;							{  How good is this item for the specified search? }
		itemSubClass:			ResType;								{  Subclass of group to put this panel.  Can use to do sub-grouping (eg volume for volume panel and mute panel) }
		itemSort:				Point;									{  Set to 0 - future to sort the items in a sub group. }
		itemFlags:				UInt32;									{  Set to 0 (future expansion) }
		itemReserved:			ResType;								{  What kind of code does the itemReference point to  (right now - kPanelEntryTypeComponentMgr only) }
		itemFuture1:			UInt32;									{  Set to 0 (future expansion - probably an alternate code style) }
		itemFuture2:			UInt32;									{  Set to 0 (future expansion - probably an alternate code style) }
		itemFuture3:			UInt32;									{  Set to 0 (future expansion - probably an alternate code style) }
		itemFuture4:			UInt32;									{  Set to 0 (future expansion - probably an alternate code style) }
	end;

	DMComponentListEntryPtr				= ^DMComponentListEntryRec;
	{  ••• Move AVLocationRec to AVComponents.i AFTER AVComponents.i is created }
	AVLocationRecPtr = ^AVLocationRec;
	AVLocationRec = record
		locationConstant:		UInt32;									{  Set to 0 (future expansion - probably an alternate code style) }
	end;

	AVLocationPtr						= ^AVLocationRec;
	DMDepthInfoRecPtr = ^DMDepthInfoRec;
	DMDepthInfoRec = record
		depthSwitchInfo:		VDSwitchInfoPtr;						{  This is the switch mode to choose this timing/depth  }
		depthVPBlock:			VPBlockPtr;								{  VPBlock (including size, depth and format)  }
		depthFlags:				UInt32;									{  VDVideoParametersInfoRec.csDepthFlags   }
		depthReserved1:			UInt32;									{  Reserved  }
		depthReserved2:			UInt32;									{  Reserved  }
	end;

	DMDepthInfoPtr						= ^DMDepthInfoRec;
	DMDepthInfoBlockRecPtr = ^DMDepthInfoBlockRec;
	DMDepthInfoBlockRec = record
		depthBlockCount:		UInt32;									{  How many depths are there?  }
		depthVPBlock:			DMDepthInfoPtr;							{  Array of DMDepthInfoRec  }
		depthBlockFlags:		UInt32;									{  Reserved  }
		depthBlockReserved1:	UInt32;									{  Reserved  }
		depthBlockReserved2:	UInt32;									{  Reserved  }
	end;

	DMDepthInfoBlockPtr					= ^DMDepthInfoBlockRec;
	DMDisplayModeListEntryRecPtr = ^DMDisplayModeListEntryRec;
	DMDisplayModeListEntryRec = record
		displayModeFlags:		UInt32;
		displayModeSwitchInfo:	VDSwitchInfoPtr;
		displayModeResolutionInfo: VDResolutionInfoPtr;
		displayModeTimingInfo:	VDTimingInfoPtr;
		displayModeDepthBlockInfo: DMDepthInfoBlockPtr;					{  Information about all the depths }
		displayModeVersion:		UInt32;									{  What version is this record (now kDisplayModeEntryVersionOne) }
		displayModeName:		StringPtr;								{  Name of the timing mode }
		displayModeDisplayInfo:	DMDisplayTimingInfoPtr;					{  Information from the display. }
	end;

	DMDisplayModeListEntryPtr			= ^DMDisplayModeListEntryRec;

	DependentNotifyRecPtr = ^DependentNotifyRec;
	DependentNotifyRec = record
		notifyType:				ResType;								{  What type was the engine that made the change (may be zero) }
		notifyClass:			ResType;								{  What class was the change (eg geometry, color etc) }
		notifyPortID:			DisplayIDType;							{  Which device was touched (kInvalidDisplayID -> all or none) }
		notifyComponent:		ComponentInstance;						{  What engine did it (may be 0)? }
		notifyVersion:			UInt32;									{  Set to 0 (future expansion) }
		notifyFlags:			UInt32;									{  Set to 0 (future expansion) }
		notifyReserved:			UInt32;									{  Set to 0 (future expansion) }
		notifyFuture:			UInt32;									{  Set to 0 (future expansion) }
	end;

	DependentNotifyPtr					= ^DependentNotifyRec;

	DMMakeAndModelRecPtr = ^DMMakeAndModelRec;
	DMMakeAndModelRec = record
		manufacturer:			ResType;
		model:					UInt32;
		serialNumber:			UInt32;
		manufactureDate:		UInt32;
		makeReserved:			array [0..3] of UInt32;
	end;

	DMMakeAndModelPtr					= ^DMMakeAndModelRec;
	{  DMNewDisplayList displayListIncludeFlags }

const
	kIncludeOnlineActiveDisplaysMask = $01;
	kIncludeOnlineDisabledDisplaysMask = $02;
	kIncludeOfflineDisplaysMask	= $04;
	kIncludeOfflineDummyDisplaysMask = $08;
	kIncludeHardwareMirroredDisplaysMask = $10;


																{  modeListFlags for DMNewDisplayModeList  }
	kDMModeListIncludeAllModesMask = $01;						{  Include all timing modes not _explicitly_ excluded (see other bits) }
	kDMModeListIncludeOfflineModesMask = $02;
	kDMModeListExcludeDriverModesMask = $04;					{  Exclude old-style timing modes (cscGetNextResolution/kDisplayModeIDFindFirstResolution modes) }
	kDMModeListExcludeDisplayModesMask = $08;					{  Exclude timing modes that come from the display (always arbritrary timing modes) }
	kDMModeListExcludeCustomModesMask = $10;					{  Exclude custom modes that came neither from the driver or display (need a better name) }
	kDMModeListPreferStretchedModesMask = $20;					{  Prefer modes that are stretched over modes that are letterboxed when setting kDisplayModeListNotPreferredBit }
	kDMModeListPreferSafeModesMask = $40;						{  Prefer modes that are safe over modes that are not when setting kDisplayModeListNotPreferredBit }
	kDMModeListExcludeDriverScaledModesMask = $80;				{  Exclude modes that are scaled by the driver (usually on fixed resolution displays) }


	{  DMNewDisplayList displayListFlags }

>>>>>>> graemeg/fixes_2_2
type
	DisplayListEntryRecPtr = ^DisplayListEntryRec;
	DisplayListEntryRec = record
		displayListEntryGDevice: GDHandle;
		displayListEntryDisplayID: DisplayIDType;
<<<<<<< HEAD
		displayListEntryIncludeFlags: UInt32; { Reason this entry was included}
		displayListEntryReserved1: UInt32;

		displayListEntryReserved2: UInt32; { Zero}
		displayListEntryReserved3: UInt32; { Zero}
		displayListEntryReserved4: UInt32; { Zero}
		displayListEntryReserved5: UInt32; { Zero}
	end;
type
	DisplayListEntryPtr = DisplayListEntryRecPtr;
	DMProfileListEntryRecPtr = ^DMProfileListEntryRec;
	DMProfileListEntryRec = record
		profileRef: UnivPtr;             { was CMProfileRef}
		profileReserved1: Ptr;       { Reserved}
		profileReserved2: Ptr;       { Reserved}
		profileReserved3: Ptr;       { Reserved}
	end;
type
	DMProfileListEntryPtr = DMProfileListEntryRecPtr;
	DMNotificationProcPtr = procedure( var theEvent: AppleEvent );
	DMExtendedNotificationProcPtr = procedure( userData: UnivPtr; theMessage: SInt16; notifyData: UnivPtr );
	DMComponentListIteratorProcPtr = procedure( userData: UnivPtr; itemIndex: DMListIndexType; componentInfo: DMComponentListEntryPtr );
	DMDisplayModeListIteratorProcPtr = procedure( userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DMDisplayModeListEntryPtr );
	DMProfileListIteratorProcPtr = procedure( userData: UnivPtr; itemIndex: DMListIndexType; profileInfo: DMProfileListEntryPtr );
	DMDisplayListIteratorProcPtr = procedure( userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DisplayListEntryPtr );
	DMNotificationUPP = DMNotificationProcPtr;
	DMExtendedNotificationUPP = DMExtendedNotificationProcPtr;
	DMComponentListIteratorUPP = DMComponentListIteratorProcPtr;
	DMDisplayModeListIteratorUPP = DMDisplayModeListIteratorProcPtr;
	DMProfileListIteratorUPP = DMProfileListIteratorProcPtr;
	DMDisplayListIteratorUPP = DMDisplayListIteratorProcPtr;
{
 *  NewDMNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMNotificationUPP( userRoutine: DMNotificationProcPtr ): DMNotificationUPP; external name '_NewDMNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
		displayListEntryIncludeFlags: UInt32;							{  Reason this entry was included }
		displayListEntryReserved1: UInt32;
		displayListEntryReserved2: UInt32;								{  Zero }
		displayListEntryReserved3: UInt32;								{  Zero }
		displayListEntryReserved4: UInt32;								{  Zero }
		displayListEntryReserved5: UInt32;								{  Zero }
	end;

	DisplayListEntryPtr					= ^DisplayListEntryRec;
	DMProfileListEntryRecPtr = ^DMProfileListEntryRec;
	DMProfileListEntryRec = record
		profileRef:				CMProfileRef;
		profileReserved1:		Ptr;									{  Reserved }
		profileReserved2:		Ptr;									{  Reserved }
		profileReserved3:		Ptr;									{  Reserved }
	end;

	DMProfileListEntryPtr				= ^DMProfileListEntryRec;
{$ifc TYPED_FUNCTION_POINTERS}
	DMNotificationProcPtr = procedure(var theEvent: AppleEvent);
{$elsec}
	DMNotificationProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	DMExtendedNotificationProcPtr = procedure(userData: UnivPtr; theMessage: SInt16; notifyData: UnivPtr);
{$elsec}
	DMExtendedNotificationProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	DMComponentListIteratorProcPtr = procedure(userData: UnivPtr; itemIndex: DMListIndexType; componentInfo: DMComponentListEntryPtr);
{$elsec}
	DMComponentListIteratorProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	DMDisplayModeListIteratorProcPtr = procedure(userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DMDisplayModeListEntryPtr);
{$elsec}
	DMDisplayModeListIteratorProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	DMProfileListIteratorProcPtr = procedure(userData: UnivPtr; itemIndex: DMListIndexType; profileInfo: DMProfileListEntryPtr);
{$elsec}
	DMProfileListIteratorProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	DMDisplayListIteratorProcPtr = procedure(userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DisplayListEntryPtr);
{$elsec}
	DMDisplayListIteratorProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	DMNotificationUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DMNotificationUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	DMExtendedNotificationUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DMExtendedNotificationUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	DMComponentListIteratorUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DMComponentListIteratorUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	DMDisplayModeListIteratorUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DMDisplayModeListIteratorUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	DMProfileListIteratorUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DMProfileListIteratorUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	DMDisplayListIteratorUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DMDisplayListIteratorUPP = UniversalProcPtr;
{$endc}	

const
	uppDMNotificationProcInfo = $000000C0;
	uppDMExtendedNotificationProcInfo = $00000EC0;
	uppDMComponentListIteratorProcInfo = $00000FC0;
	uppDMDisplayModeListIteratorProcInfo = $00000FC0;
	uppDMProfileListIteratorProcInfo = $00000FC0;
	uppDMDisplayListIteratorProcInfo = $00000FC0;
	{
	 *  NewDMNotificationUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewDMNotificationUPP(userRoutine: DMNotificationProcPtr): DMNotificationUPP; external name '_NewDMNotificationUPP'; { old name was NewDMNotificationProc }
>>>>>>> graemeg/fixes_2_2
{
 *  NewDMExtendedNotificationUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMExtendedNotificationUPP( userRoutine: DMExtendedNotificationProcPtr ): DMExtendedNotificationUPP; external name '_NewDMExtendedNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function NewDMExtendedNotificationUPP(userRoutine: DMExtendedNotificationProcPtr): DMExtendedNotificationUPP; external name '_NewDMExtendedNotificationUPP'; { old name was NewDMExtendedNotificationProc }
>>>>>>> graemeg/fixes_2_2
{
 *  NewDMComponentListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMComponentListIteratorUPP( userRoutine: DMComponentListIteratorProcPtr ): DMComponentListIteratorUPP; external name '_NewDMComponentListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function NewDMComponentListIteratorUPP(userRoutine: DMComponentListIteratorProcPtr): DMComponentListIteratorUPP; external name '_NewDMComponentListIteratorUPP'; { old name was NewDMComponentListIteratorProc }
>>>>>>> graemeg/fixes_2_2
{
 *  NewDMDisplayModeListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMDisplayModeListIteratorUPP( userRoutine: DMDisplayModeListIteratorProcPtr ): DMDisplayModeListIteratorUPP; external name '_NewDMDisplayModeListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function NewDMDisplayModeListIteratorUPP(userRoutine: DMDisplayModeListIteratorProcPtr): DMDisplayModeListIteratorUPP; external name '_NewDMDisplayModeListIteratorUPP'; { old name was NewDMDisplayModeListIteratorProc }
>>>>>>> graemeg/fixes_2_2
{
 *  NewDMProfileListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMProfileListIteratorUPP( userRoutine: DMProfileListIteratorProcPtr ): DMProfileListIteratorUPP; external name '_NewDMProfileListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function NewDMProfileListIteratorUPP(userRoutine: DMProfileListIteratorProcPtr): DMProfileListIteratorUPP; external name '_NewDMProfileListIteratorUPP'; { old name was NewDMProfileListIteratorProc }
>>>>>>> graemeg/fixes_2_2
{
 *  NewDMDisplayListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMDisplayListIteratorUPP( userRoutine: DMDisplayListIteratorProcPtr ): DMDisplayListIteratorUPP; external name '_NewDMDisplayListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function NewDMDisplayListIteratorUPP(userRoutine: DMDisplayListIteratorProcPtr): DMDisplayListIteratorUPP; external name '_NewDMDisplayListIteratorUPP'; { old name was NewDMDisplayListIteratorProc }
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeDMNotificationUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMNotificationUPP( userUPP: DMNotificationUPP ); external name '_DisposeDMNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeDMNotificationUPP(userUPP: DMNotificationUPP); external name '_DisposeDMNotificationUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeDMExtendedNotificationUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMExtendedNotificationUPP( userUPP: DMExtendedNotificationUPP ); external name '_DisposeDMExtendedNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeDMExtendedNotificationUPP(userUPP: DMExtendedNotificationUPP); external name '_DisposeDMExtendedNotificationUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeDMComponentListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMComponentListIteratorUPP( userUPP: DMComponentListIteratorUPP ); external name '_DisposeDMComponentListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeDMComponentListIteratorUPP(userUPP: DMComponentListIteratorUPP); external name '_DisposeDMComponentListIteratorUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeDMDisplayModeListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMDisplayModeListIteratorUPP( userUPP: DMDisplayModeListIteratorUPP ); external name '_DisposeDMDisplayModeListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeDMDisplayModeListIteratorUPP(userUPP: DMDisplayModeListIteratorUPP); external name '_DisposeDMDisplayModeListIteratorUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeDMProfileListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMProfileListIteratorUPP( userUPP: DMProfileListIteratorUPP ); external name '_DisposeDMProfileListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeDMProfileListIteratorUPP(userUPP: DMProfileListIteratorUPP); external name '_DisposeDMProfileListIteratorUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeDMDisplayListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMDisplayListIteratorUPP( userUPP: DMDisplayListIteratorUPP ); external name '_DisposeDMDisplayListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeDMDisplayListIteratorUPP(userUPP: DMDisplayListIteratorUPP); external name '_DisposeDMDisplayListIteratorUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeDMNotificationUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMNotificationUPP( var theEvent: AppleEvent; userUPP: DMNotificationUPP ); external name '_InvokeDMNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure InvokeDMNotificationUPP(var theEvent: AppleEvent; userRoutine: DMNotificationUPP); external name '_InvokeDMNotificationUPP'; { old name was CallDMNotificationProc }
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeDMExtendedNotificationUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMExtendedNotificationUPP( userData: UnivPtr; theMessage: SInt16; notifyData: UnivPtr; userUPP: DMExtendedNotificationUPP ); external name '_InvokeDMExtendedNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure InvokeDMExtendedNotificationUPP(userData: UnivPtr; theMessage: SInt16; notifyData: UnivPtr; userRoutine: DMExtendedNotificationUPP); external name '_InvokeDMExtendedNotificationUPP'; { old name was CallDMExtendedNotificationProc }
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeDMComponentListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMComponentListIteratorUPP( userData: UnivPtr; itemIndex: DMListIndexType; componentInfo: DMComponentListEntryPtr; userUPP: DMComponentListIteratorUPP ); external name '_InvokeDMComponentListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure InvokeDMComponentListIteratorUPP(userData: UnivPtr; itemIndex: DMListIndexType; componentInfo: DMComponentListEntryPtr; userRoutine: DMComponentListIteratorUPP); external name '_InvokeDMComponentListIteratorUPP'; { old name was CallDMComponentListIteratorProc }
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeDMDisplayModeListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMDisplayModeListIteratorUPP( userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DMDisplayModeListEntryPtr; userUPP: DMDisplayModeListIteratorUPP ); external name '_InvokeDMDisplayModeListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure InvokeDMDisplayModeListIteratorUPP(userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DMDisplayModeListEntryPtr; userRoutine: DMDisplayModeListIteratorUPP); external name '_InvokeDMDisplayModeListIteratorUPP'; { old name was CallDMDisplayModeListIteratorProc }
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeDMProfileListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMProfileListIteratorUPP( userData: UnivPtr; itemIndex: DMListIndexType; profileInfo: DMProfileListEntryPtr; userUPP: DMProfileListIteratorUPP ); external name '_InvokeDMProfileListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure InvokeDMProfileListIteratorUPP(userData: UnivPtr; itemIndex: DMListIndexType; profileInfo: DMProfileListEntryPtr; userRoutine: DMProfileListIteratorUPP); external name '_InvokeDMProfileListIteratorUPP'; { old name was CallDMProfileListIteratorProc }
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeDMDisplayListIteratorUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMDisplayListIteratorUPP( userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DisplayListEntryPtr; userUPP: DMDisplayListIteratorUPP ); external name '_InvokeDMDisplayListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$ifc not TARGET_CPU_64}
=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure InvokeDMDisplayListIteratorUPP(userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DisplayListEntryPtr; userRoutine: DMDisplayListIteratorUPP); external name '_InvokeDMDisplayListIteratorUPP'; { old name was CallDMDisplayListIteratorProc }
{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  DMDisplayGestalt()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMDisplayGestalt(theDisplayID: DisplayIDType; displayGestaltSelector: ResType; var displayGestaltResponse: UInt32): OSErr; external name '_DMDisplayGestalt';
>>>>>>> graemeg/fixes_2_2
{
 *  DMUseScreenPrefs()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMUseScreenPrefs(usePrefs: boolean; displayState: Handle): OSErr; external name '_DMUseScreenPrefs';
>>>>>>> graemeg/fixes_2_2
{
 *  DMSuspendConfigure()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMSuspendConfigure(displayState: Handle; reserved1: UInt32): OSErr; external name '_DMSuspendConfigure';
>>>>>>> graemeg/fixes_2_2
{
 *  DMResumeConfigure()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMResumeConfigure(displayState: Handle; reserved1: UInt32): OSErr; external name '_DMResumeConfigure';
>>>>>>> graemeg/fixes_2_2
{
 *  DMSetGammaByAVID()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMSetGammaByAVID(gammaAVID: AVIDType; setGammaFlags: UInt32; theGamma: GammaTblHandle): OSErr; external name '_DMSetGammaByAVID';
>>>>>>> graemeg/fixes_2_2
{
 *  DMGetGammaByAVID()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMGetGammaByAVID(gammaAVID: AVIDType; getGammaFlags: UInt32; var theGamma: GammaTblHandle): OSErr; external name '_DMGetGammaByAVID';
>>>>>>> graemeg/fixes_2_2
{
 *  DMGetMakeAndModelByAVID()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMGetMakeAndModelByAVID(theAVID: AVIDType; theMakeAndModel: DMMakeAndModelPtr): OSErr; external name '_DMGetMakeAndModelByAVID';
>>>>>>> graemeg/fixes_2_2
{
 *  DMNewDisplayList()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMNewDisplayList(displayListIncludeFlags: UInt32; reserved1: UInt32; reserved2: UInt32; var theCount: DMListIndexType; var theDisplayList: DMListType): OSErr; external name '_DMNewDisplayList';
>>>>>>> graemeg/fixes_2_2
{
 *  DMGetIndexedDisplayFromList()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMGetIndexedDisplayFromList(theDisplayList: DMListType; itemIndex: DMListIndexType; reserved: UInt32; listIterator: DMDisplayListIteratorUPP; userData: UnivPtr): OSErr; external name '_DMGetIndexedDisplayFromList';
>>>>>>> graemeg/fixes_2_2
{
 *  DMNewProfileListByAVID()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMNewProfileListByAVID(theAVID: AVIDType; reserved: UInt32; var profileCount: DMListIndexType; var profileList: DMListType): OSErr; external name '_DMNewProfileListByAVID';
>>>>>>> graemeg/fixes_2_2
{
 *  DMGetIndexedProfileFromList()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMGetFirstScreenDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMGetFirstScreenDevice( activeOnly: Boolean ): GDHandle; external name '_DMGetFirstScreenDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetNextScreenDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMGetNextScreenDevice( theDevice: GDHandle; activeOnly: Boolean ): GDHandle; external name '_DMGetNextScreenDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDrawDesktopRect()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DMDrawDesktopRect( var globalRect: Rect ); external name '_DMDrawDesktopRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDrawDesktopRegion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DMDrawDesktopRegion( globalRgn: RgnHandle ); external name '_DMDrawDesktopRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMBeginConfigureDisplays()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMBeginConfigureDisplays( var displayState: Handle ): OSErr; external name '_DMBeginConfigureDisplays';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMEndConfigureDisplays()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMEndConfigureDisplays( displayState: Handle ): OSErr; external name '_DMEndConfigureDisplays';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMAddDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMAddDisplay( newDevice: GDHandle; driver: SInt16; mode: UInt32; reserved: UInt32; displayID: UInt32; displayComponent: Component; displayState: Handle ): OSErr; external name '_DMAddDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMMoveDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMMoveDisplay( moveDevice: GDHandle; x: SInt16; y: SInt16; displayState: Handle ): OSErr; external name '_DMMoveDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDisableDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMDisableDisplay( disableDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMDisableDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMEnableDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMEnableDisplay( enableDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMEnableDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRemoveDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMRemoveDisplay( removeDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMRemoveDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetMainDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMSetMainDisplay( newMainDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMSetMainDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetDisplayMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMSetDisplayMode( theDevice: GDHandle; mode: UInt32; var depthMode: UInt32; reserved: SIGNEDLONG; displayState: Handle ): OSErr; external name '_DMSetDisplayMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMCheckDisplayMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMCheckDisplayMode( theDevice: GDHandle; mode: UInt32; depthMode: UInt32; var switchFlags: UInt32; reserved: UInt32; var modeOk: Boolean ): OSErr; external name '_DMCheckDisplayMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDeskRegion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMGetDeskRegion( var desktopRegion: RgnHandle ): OSErr; external name '_DMGetDeskRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRegisterNotifyProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMRegisterNotifyProc( notificationProc: DMNotificationUPP; whichPSN: DMProcessInfoPtr ): OSErr; external name '_DMRegisterNotifyProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRemoveNotifyProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMRemoveNotifyProc( notificationProc: DMNotificationUPP; whichPSN: DMProcessInfoPtr ): OSErr; external name '_DMRemoveNotifyProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMQDIsMirroringCapable()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMQDIsMirroringCapable( var qdIsMirroringCapable: Boolean ): OSErr; external name '_DMQDIsMirroringCapable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMCanMirrorNow()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMCanMirrorNow( var canMirrorNow: Boolean ): OSErr; external name '_DMCanMirrorNow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMIsMirroringOn()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMIsMirroringOn( var isMirroringOn: Boolean ): OSErr; external name '_DMIsMirroringOn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMMirrorDevices()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMMirrorDevices( gD1: GDHandle; gD2: GDHandle; displayState: Handle ): OSErr; external name '_DMMirrorDevices';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMUnmirrorDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMUnmirrorDevice( gDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMUnmirrorDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetNextMirroredDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMGetNextMirroredDevice( gDevice: GDHandle; var mirroredDevice: GDHandle ): OSErr; external name '_DMGetNextMirroredDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMBlockMirroring()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMBlockMirroring: OSErr; external name '_DMBlockMirroring';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMUnblockMirroring()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMUnblockMirroring: OSErr; external name '_DMUnblockMirroring';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DMGetIndexedProfileFromList(profileList: DMListType; itemIndex: DMListIndexType; reserved: UInt32; listIterator: DMProfileListIteratorUPP; userData: UnivPtr): OSErr; external name '_DMGetIndexedProfileFromList';
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  DMGetFirstScreenDevice()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMGetFirstScreenDevice(activeOnly: boolean): GDHandle; external name '_DMGetFirstScreenDevice';
{
 *  DMGetNextScreenDevice()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMGetNextScreenDevice(theDevice: GDHandle; activeOnly: boolean): GDHandle; external name '_DMGetNextScreenDevice';
{
 *  DMDrawDesktopRect()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DMDrawDesktopRect(var globalRect: Rect); external name '_DMDrawDesktopRect';
{
 *  DMDrawDesktopRegion()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DMDrawDesktopRegion(globalRgn: RgnHandle); external name '_DMDrawDesktopRegion';
{
 *  DMBeginConfigureDisplays()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMBeginConfigureDisplays(var displayState: Handle): OSErr; external name '_DMBeginConfigureDisplays';
{
 *  DMEndConfigureDisplays()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMEndConfigureDisplays(displayState: Handle): OSErr; external name '_DMEndConfigureDisplays';
{
 *  DMAddDisplay()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMAddDisplay(newDevice: GDHandle; driver: SInt16; mode: UInt32; reserved: UInt32; displayID: UInt32; displayComponent: Component; displayState: Handle): OSErr; external name '_DMAddDisplay';
{
 *  DMMoveDisplay()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMMoveDisplay(moveDevice: GDHandle; x: SInt16; y: SInt16; displayState: Handle): OSErr; external name '_DMMoveDisplay';
{
 *  DMDisableDisplay()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMDisableDisplay(disableDevice: GDHandle; displayState: Handle): OSErr; external name '_DMDisableDisplay';
{
 *  DMEnableDisplay()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMEnableDisplay(enableDevice: GDHandle; displayState: Handle): OSErr; external name '_DMEnableDisplay';
{
 *  DMRemoveDisplay()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMRemoveDisplay(removeDevice: GDHandle; displayState: Handle): OSErr; external name '_DMRemoveDisplay';
{
 *  DMSetMainDisplay()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMSetMainDisplay(newMainDevice: GDHandle; displayState: Handle): OSErr; external name '_DMSetMainDisplay';
{
 *  DMSetDisplayMode()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMSetDisplayMode(theDevice: GDHandle; mode: UInt32; var depthMode: UInt32; reserved: UInt32; displayState: Handle): OSErr; external name '_DMSetDisplayMode';
{
 *  DMCheckDisplayMode()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMCheckDisplayMode(theDevice: GDHandle; mode: UInt32; depthMode: UInt32; var switchFlags: UInt32; reserved: UInt32; var modeOk: boolean): OSErr; external name '_DMCheckDisplayMode';
{
 *  DMGetDeskRegion()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMGetDeskRegion(var desktopRegion: RgnHandle): OSErr; external name '_DMGetDeskRegion';
{
 *  DMRegisterNotifyProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMRegisterNotifyProc(notificationProc: DMNotificationUPP; whichPSN: DMProcessInfoPtr): OSErr; external name '_DMRegisterNotifyProc';
{
 *  DMRemoveNotifyProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMRemoveNotifyProc(notificationProc: DMNotificationUPP; whichPSN: DMProcessInfoPtr): OSErr; external name '_DMRemoveNotifyProc';
{
 *  DMQDIsMirroringCapable()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMQDIsMirroringCapable(var qdIsMirroringCapable: boolean): OSErr; external name '_DMQDIsMirroringCapable';
{
 *  DMCanMirrorNow()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMCanMirrorNow(var canMirrorNow: boolean): OSErr; external name '_DMCanMirrorNow';
{
 *  DMIsMirroringOn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMIsMirroringOn(var isMirroringOn: boolean): OSErr; external name '_DMIsMirroringOn';
{
 *  DMMirrorDevices()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMMirrorDevices(gD1: GDHandle; gD2: GDHandle; displayState: Handle): OSErr; external name '_DMMirrorDevices';
{
 *  DMUnmirrorDevice()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMUnmirrorDevice(gDevice: GDHandle; displayState: Handle): OSErr; external name '_DMUnmirrorDevice';
{
 *  DMGetNextMirroredDevice()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMGetNextMirroredDevice(gDevice: GDHandle; var mirroredDevice: GDHandle): OSErr; external name '_DMGetNextMirroredDevice';
{
 *  DMBlockMirroring()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMBlockMirroring: OSErr; external name '_DMBlockMirroring';
{
 *  DMUnblockMirroring()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DMUnblockMirroring: OSErr; external name '_DMUnblockMirroring';
{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  DMGetDisplayMgrA5World()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }


{
 *  DMGetDisplayIDByGDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMGetDisplayIDByGDevice( displayDevice: GDHandle; var displayID: DisplayIDType; failToMain: Boolean ): OSErr; external name '_DMGetDisplayIDByGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetGDeviceByDisplayID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMGetGDeviceByDisplayID( displayID: DisplayIDType; var displayDevice: GDHandle; failToMain: Boolean ): OSErr; external name '_DMGetGDeviceByDisplayID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetDisplayComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMSetDisplayComponent( theDevice: GDHandle; displayComponent: Component ): OSErr; external name '_DMSetDisplayComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDisplayComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMGetDisplayComponent( theDevice: GDHandle; var displayComponent: Component ): OSErr; external name '_DMGetDisplayComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMNewDisplay( var newDevice: GDHandle; driverRefNum: SInt16; mode: UInt32; reserved: UInt32; displayID: DisplayIDType; displayComponent: Component; displayState: Handle ): OSErr; external name '_DMNewDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDisposeDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMDisposeDisplay( disposeDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMDisposeDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMResolveDisplayComponents()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMResolveDisplayComponents: OSErr; external name '_DMResolveDisplayComponents';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRegisterExtendedNotifyProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.0 and later
 }
function DMRegisterExtendedNotifyProc( notifyProc: DMExtendedNotificationUPP; notifyUserData: UnivPtr; nofifyOnFlags: UInt16; whichPSN: DMProcessInfoPtr ): OSErr; external name '_DMRegisterExtendedNotifyProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRemoveExtendedNotifyProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.0 and later
 }
function DMRemoveExtendedNotifyProc( notifyProc: DMExtendedNotificationUPP; notifyUserData: UnivPtr; whichPSN: DMProcessInfoPtr; removeFlags: UInt16 ): OSErr; external name '_DMRemoveExtendedNotifyProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVPanelList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVPanelList( displayID: DisplayIDType; panelType: ResType; minimumFidelity: DMFidelityType; panelListFlags: UInt32; reserved: UInt32; var thePanelCount: DMListIndexType; var thePanelList: DMListType ): OSErr; external name '_DMNewAVPanelList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVEngineList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVEngineList( displayID: DisplayIDType; engineType: ResType; minimumFidelity: DMFidelityType; engineListFlags: UInt32; reserved: UInt32; var engineCount: DMListIndexType; var engineList: DMListType ): OSErr; external name '_DMNewAVEngineList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVDeviceList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVDeviceList( deviceType: ResType; deviceListFlags: UInt32; reserved: UInt32; var deviceCount: DMListIndexType; var deviceList: DMListType ): OSErr; external name '_DMNewAVDeviceList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVPortListByPortType()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVPortListByPortType( subType: ResType; portListFlags: UInt32; reserved: UInt32; var devicePortCount: DMListIndexType; var theDevicePortList: DMListType ): OSErr; external name '_DMNewAVPortListByPortType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetIndexedComponentFromList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetIndexedComponentFromList( panelList: DMListType; itemIndex: DMListIndexType; reserved: UInt32; listIterator: DMComponentListIteratorUPP; userData: UnivPtr ): OSErr; external name '_DMGetIndexedComponentFromList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDisposeList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.0 and later
 }
function DMDisposeList( panelList: DMListType ): OSErr; external name '_DMDisposeList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetNameByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetNameByAVID( theID: AVIDType; nameFlags: UInt32; var name: Str255 ): OSErr; external name '_DMGetNameByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVIDByPortComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVIDByPortComponent( thePortComponent: Component; portKind: ResType; reserved: UInt32; var newID: AVIDType ): OSErr; external name '_DMNewAVIDByPortComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetPortComponentByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetPortComponentByAVID( thePortID: DisplayIDType; var thePortComponent: Component; var theDesciption: ComponentDescription; var thePortKind: ResType ): OSErr; external name '_DMGetPortComponentByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSendDependentNotification()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.0 and later
 }
function DMSendDependentNotification( notifyType: ResType; notifyClass: ResType; displayID: AVIDType; notifyComponent: ComponentInstance ): OSErr; external name '_DMSendDependentNotification';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDisposeAVComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMDisposeAVComponent( theAVComponent: Component ): OSErr; external name '_DMDisposeAVComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSaveScreenPrefs()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMSaveScreenPrefs( reserved1: UInt32; saveFlags: UInt32; reserved2: UInt32 ): OSErr; external name '_DMSaveScreenPrefs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVIDByDeviceComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVIDByDeviceComponent( theDeviceComponent: Component; portKind: ResType; reserved: UInt32; var newID: DisplayIDType ): OSErr; external name '_DMNewAVIDByDeviceComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVPortListByDeviceAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVPortListByDeviceAVID( theID: AVIDType; minimumFidelity: DMFidelityType; portListFlags: UInt32; reserved: UInt32; var devicePortCount: DMListIndexType; var theDevicePortList: DMListType ): OSErr; external name '_DMNewAVPortListByDeviceAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDeviceComponentByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetDeviceComponentByAVID( theDeviceID: AVIDType; var theDeviceComponent: Component; var theDesciption: ComponentDescription; var theDeviceKind: ResType ): OSErr; external name '_DMGetDeviceComponentByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewDisplayModeList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewDisplayModeList( displayID: DisplayIDType; modeListFlags: UInt32; reserved: UInt32; var thePanelCount: DMListIndexType; var thePanelList: DMListType ): OSErr; external name '_DMNewDisplayModeList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetIndexedDisplayModeFromList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetIndexedDisplayModeFromList( panelList: DMListType; itemIndex: DMListIndexType; reserved: UInt32; listIterator: DMDisplayModeListIteratorUPP; userData: UnivPtr ): OSErr; external name '_DMGetIndexedDisplayModeFromList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetGraphicInfoByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetGraphicInfoByAVID( theID: AVIDType; var theAVPcit: PicHandle; var theAVIconSuite: Handle; var theAVLocation: AVLocationRec ): OSErr; external name '_DMGetGraphicInfoByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetAVPowerState()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetAVPowerState( theID: AVIDType; getPowerState: AVPowerStatePtr; reserved1: UInt32 ): OSErr; external name '_DMGetAVPowerState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetAVPowerState()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMSetAVPowerState( theID: AVIDType; setPowerState: AVPowerStatePtr; powerFlags: UInt32; displayState: Handle ): OSErr; external name '_DMSetAVPowerState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDeviceAVIDByPortAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetDeviceAVIDByPortAVID( portAVID: AVIDType; var deviceAVID: AVIDType ): OSErr; external name '_DMGetDeviceAVIDByPortAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetEnableByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetEnableByAVID( theAVID: AVIDType; var isAVIDEnabledNow: Boolean; var canChangeEnableNow: Boolean ): OSErr; external name '_DMGetEnableByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetEnableByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMSetEnableByAVID( theAVID: AVIDType; doEnable: Boolean; displayState: Handle ): OSErr; external name '_DMSetEnableByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDisplayMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetDisplayMode( theDevice: GDHandle; switchInfo: VDSwitchInfoPtr ): OSErr; external name '_DMGetDisplayMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMConfirmConfiguration()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }
function DMConfirmConfiguration( filterProc: DMModalFilterUPP; confirmFlags: UInt32; reserved: UInt32; displayState: Handle ): OSErr; external name '_DMConfirmConfiguration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
uses MacTypes,Quickdraw,CMTypes,AEDataModel,ConditionalMacros,Components,Video,CMApplication,AppleEvents,Events,Processes,Dialogs;
=======
uses MacTypes,QuickdrawTypes,ColorSyncDeprecated,AEDataModel,ConditionalMacros,Components,Video,AppleEvents,Events,Processes,Dialogs;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{******************* DEPRECATION NOTICE *********************
 *
 * The DisplayMgr API is being deprecated, and should be replaced
 * by the CGDirectDisplay API in the CoreGraphics framework in 
 * ApplicationServices.framework.
 *
 ************************************************************}

type
	DMProcessInfoPtr = UnivPtr;
	DMModalFilterUPP = UnivPtr;
const
{ AppleEvents Core Suite }
	kAESystemConfigNotice = FourCharCode('cnfg'); { Core Suite types }
	kAEDisplayNotice = FourCharCode('dspl');
	kAEDisplaySummary = FourCharCode('dsum');
	keyDMConfigVersion = FourCharCode('dmcv');
	keyDMConfigFlags = FourCharCode('dmcf');
	keyDMConfigReserved = FourCharCode('dmcr');
	keyDisplayID = FourCharCode('dmid');
	keyDisplayComponent = FourCharCode('dmdc');
	keyDisplayDevice = FourCharCode('dmdd');
	keyDisplayFlags = FourCharCode('dmdf');
	keyDisplayMode = FourCharCode('dmdm');
	keyDisplayModeReserved = FourCharCode('dmmr');
	keyDisplayReserved = FourCharCode('dmdr');
	keyDisplayMirroredId = FourCharCode('dmmi');
	keyDeviceFlags = FourCharCode('dddf');
	keyDeviceDepthMode = FourCharCode('dddm');
	keyDeviceRect = FourCharCode('dddr');
	keyPixMapRect = FourCharCode('dpdr');
	keyPixMapHResolution = FourCharCode('dphr');
	keyPixMapVResolution = FourCharCode('dpvr');
	keyPixMapPixelType = FourCharCode('dppt');
	keyPixMapPixelSize = FourCharCode('dpps');
	keyPixMapCmpCount = FourCharCode('dpcc');
	keyPixMapCmpSize = FourCharCode('dpcs');
	keyPixMapAlignment = FourCharCode('dppa');
	keyPixMapResReserved = FourCharCode('dprr');
	keyPixMapReserved = FourCharCode('dppr');
	keyPixMapColorTableSeed = FourCharCode('dpct');
	keySummaryMenubar = FourCharCode('dsmb');
	keySummaryChanges = FourCharCode('dsch');
	keyDisplayOldConfig = FourCharCode('dold');
	keyDisplayNewConfig = FourCharCode('dnew');

const
	dmOnlyActiveDisplays = true;
	dmAllDisplays = false;


const
{ DMSendDependentNotification notifyClass }
	kDependentNotifyClassShowCursor = FourCharCode('shcr'); { When display mgr shows a hidden cursor during an unmirror }
	kDependentNotifyClassDriverOverride = FourCharCode('ndrv'); { When a driver is overridden }
	kDependentNotifyClassDisplayMgrOverride = FourCharCode('dmgr'); { When display manager is upgraded }
	kDependentNotifyClassProfileChanged = FourCharCode('prof'); { When DMSetProfileByAVID is called }


const
{ Switch Flags }
	kNoSwitchConfirmBit = 0;    { Flag indicating that there is no need to confirm a switch to this mode }
	kDepthNotAvailableBit = 1;    { Current depth not available in new mode }
	kShowModeBit = 3;    { Show this mode even though it requires a confirm. }
	kModeNotResizeBit = 4;    { Do not use this mode to resize display (for cards that mode drives a different connector). }
	kNeverShowModeBit = 5;     { This mode should not be shown in the user interface. }

{    Summary Change Flags (sticky bits indicating an operation was performed)
    For example, moving a display then moving it back will still set the kMovedDisplayBit.
}
const
	kBeginEndConfigureBit = 0;
	kMovedDisplayBit = 1;
	kSetMainDisplayBit = 2;
	kSetDisplayModeBit = 3;
	kAddDisplayBit = 4;
	kRemoveDisplayBit = 5;
	kNewDisplayBit = 6;
	kDisposeDisplayBit = 7;
	kEnabledDisplayBit = 8;
	kDisabledDisplayBit = 9;
	kMirrorDisplayBit = 10;
	kUnMirrorDisplayBit = 11;


const
{ Notification Messages for extended call back routines }
	kDMNotifyRequestConnectionProbe = 0;  { Like kDMNotifyRequestDisplayProbe only not for smart displays (used in wake before all busses are awake) }
	kDMNotifyInstalled = 1;    { At install time }
	kDMNotifyEvent = 2;    { Post change time }
	kDMNotifyRemoved = 3;    { At remove time }
	kDMNotifyPrep = 4;    { Pre change time }
	kDMNotifyExtendEvent = 5;    { Allow registrees to extend apple event before it is sent }
	kDMNotifyDependents = 6;    { Minor notification check without full update }
	kDMNotifySuspendConfigure = 7;    { Temporary end of configuration }
	kDMNotifyResumeConfigure = 8;    { Resume configuration }
	kDMNotifyRequestDisplayProbe = 9;    { Request smart displays re-probe (used in sleep and hot plugging) }
	kDMNotifyDisplayWillSleep = 10;   { Mac OS X only }
	kDMNotifyDisplayDidWake = 11;   { Mac OS X only }
                                        { Notification Flags }
	kExtendedNotificationProc = 1 shl 16;


{ types for notifyType }
const
	kFullNotify = 0;    { This is the appleevent whole nine yards notify }
	kFullDependencyNotify = 1;     { Only sends to those who want to know about interrelated functionality (used for updating UI) }

{ DisplayID/DeviceID constants }
const
	kDummyDeviceID = $00FF; { This is the ID of the dummy display, used when the last “real” display is disabled.}
	kInvalidDisplayID = $0000; { This is the invalid ID}
	kFirstDisplayID = $0100;

const
{ bits for panelListFlags }
	kAllowDuplicatesBit = 0;

const
{ bits for nameFlags }
	kSuppressNumberBit = 0;
	kSuppressNumberMask = 1;
	kForceNumberBit = 1;
	kForceNumberMask = 2;
	kSuppressNameBit = 2;
	kSuppressNameMask = 4;

{ DMGetNameByAVID masks}
const
	kDMSupressNumbersMask = 1 shl 0; { Supress the numbers and return only names}
	kDMForceNumbersMask = 1 shl 1; { Force numbers to always be shown (even on single display configs)}
	kDMSupressNameMask = 1 shl 2; { Supress the names and return only numbers.}


{ Constants for fidelity checks }
const
	kNoFidelity = 0;
	kMinimumFidelity = 1;
	kDefaultFidelity = 500;  { I'm just picking a number for Apple default panels and engines}
	kDefaultManufacturerFidelity = 1000;  { I'm just picking a number for Manufacturer's panels and engines (overrides apple defaults)}

const
	kAnyPanelType = 0;    { Pass to DMNewEngineList for list of all panels (as opposed to specific types)}
	kAnyEngineType = 0;    { Pass to DMNewEngineList for list of all engines}
	kAnyDeviceType = 0;    { Pass to DMNewDeviceList for list of all devices}
	kAnyPortType = 0;     { Pass to DMNewDevicePortList for list of all devices}

{ portListFlags for DM_NewDevicePortList }
const
{ Should offline devices be put into the port list (such as dummy display) }
	kPLIncludeOfflineDevicesBit = 0;


{ confirmFlags for DMConfirmConfiguration }
const
	kForceConfirmBit = 0;    { Force a confirm dialog }
	kForceConfirmMask = 1 shl kForceConfirmBit;


{ Flags for displayModeFlags }
const
	kDisplayModeListNotPreferredBit = 0;
	kDisplayModeListNotPreferredMask = 1 shl kDisplayModeListNotPreferredBit;


{ Flags for itemFlags }
const
	kComponentListNotPreferredBit = 0;
	kComponentListNotPreferredMask = 1 shl kComponentListNotPreferredBit;

const
	kDisplayTimingInfoVersionZero = 1;
	kDisplayTimingInfoReservedCountVersionZero = 16;
	kDisplayModeEntryVersionZero = 0;    { displayModeVersion - original version}
	kDisplayModeEntryVersionOne = 1;     { displayModeVersion - added displayModeOverrideInfo}


const
	kMakeAndModelReservedCount = 4;     { Number of reserved fields}


{ Display Gestalt for DMDisplayGestalt}
const
	kDisplayGestaltDisplayCommunicationAttr = FourCharCode('comm');
	kDisplayGestaltForbidI2CMask = 1 shl 0; { Some displays have firmware problems if they get I2C communication.  If this bit is set, then I2C communication is forbidden}
	kDisplayGestaltUseI2CPowerMask = 1 shl 1; { Some displays require I2C power settings (most use DPMS).}
	kDisplayGestaltCalibratorAttr = FourCharCode('cali');
	kDisplayGestaltBrightnessAffectsGammaMask = 1 shl 0; { Used by default calibrator (should we show brightness panel) }
	kDisplayGestaltViewAngleAffectsGammaMask = 1 shl 1; { Currently not used by color sync}


type
	DMFidelityType = UInt32;
{
   AVID is an ID for ports and devices the old DisplayID type
    is carried on for compatibility
}


type
	DMListType = UnivPtr;
	DMListIndexType = UInt32;
	AVPowerStateRec = VDPowerStateRec;
	AVPowerStateRecPtr = ^AVPowerStateRec;
	AVPowerStatePtr = VDPowerStateRecPtr;
	DMDisplayTimingInfoRecPtr = ^DMDisplayTimingInfoRec;
	DMDisplayTimingInfoRec = record
		timingInfoVersion: UInt32;
		timingInfoAttributes: UInt32;   { Flags }
		timingInfoRelativeQuality: SInt32; { quality of the timing }
		timingInfoRelativeDefault: SInt32; { relative default of the timing }

		timingInfoReserved: array [0..15] of UInt32;  
	end;
type
	DMDisplayTimingInfoPtr = ^DMDisplayTimingInfoRec;

type
	DMComponentListEntryRecPtr = ^DMComponentListEntryRec;
	DMComponentListEntryRec = record
		itemID: DisplayIDType;                 { DisplayID Manager}
		itemComponent: Component;          { Component Manager}
		itemDescription: ComponentDescription;      { We can always construct this if we use something beyond the compontent mgr.}

		itemClass: ResType;              { Class of group to put this panel (eg geometry/color/etc for panels, brightness/contrast for engines, video out/sound/etc for devices)}
		itemFidelity: DMFidelityType;           { How good is this item for the specified search?}
		itemSubClass: ResType;           { Subclass of group to put this panel.  Can use to do sub-grouping (eg volume for volume panel and mute panel)}
		itemSort: Point;               { Set to 0 - future to sort the items in a sub group.}

		itemFlags: UInt32;              { Set to 0 (future expansion)}
		itemReserved: ResType;           { What kind of code does the itemReference point to  (right now - kPanelEntryTypeComponentMgr only)}
		itemFuture1: UInt32;            { Set to 0 (future expansion - probably an alternate code style)}
		itemFuture2: UInt32;            { Set to 0 (future expansion - probably an alternate code style)}
		itemFuture3: UInt32;            { Set to 0 (future expansion - probably an alternate code style)}
		itemFuture4: UInt32;            { Set to 0 (future expansion - probably an alternate code style)}
	end;
type
	DMComponentListEntryPtr = DMComponentListEntryRecPtr;
{ ••• Move AVLocationRec to AVComponents.i AFTER AVComponents.i is created}
type
	AVLocationRecPtr = ^AVLocationRec;
	AVLocationRec = record
		locationConstant: UInt32;       { Set to 0 (future expansion - probably an alternate code style)}
	end;
type
	AVLocationPtr = AVLocationRecPtr;
	DMDepthInfoRecPtr = ^DMDepthInfoRec;
	DMDepthInfoRec = record
		depthSwitchInfo: VDSwitchInfoPtr;        { This is the switch mode to choose this timing/depth }
		depthVPBlock: VPBlockPtr;           { VPBlock (including size, depth and format) }
		depthFlags: UInt32;             { VDVideoParametersInfoRec.csDepthFlags  }
		depthReserved1: UInt32;         { Reserved }
		depthReserved2: UInt32;         { Reserved }
	end;
type
	DMDepthInfoPtr = DMDepthInfoRecPtr;
	DMDepthInfoBlockRecPtr = ^DMDepthInfoBlockRec;
	DMDepthInfoBlockRec = record
		depthBlockCount: UInt32;        { How many depths are there? }
		depthVPBlock: DMDepthInfoPtr;           { Array of DMDepthInfoRec }
		depthBlockFlags: UInt32;        { Reserved }
		depthBlockReserved1: UInt32;    { Reserved }
		depthBlockReserved2: UInt32;    { Reserved }
	end;
type
	DMDepthInfoBlockPtr = DMDepthInfoBlockRecPtr;
	DMDisplayModeListEntryRecPtr = ^DMDisplayModeListEntryRec;
	DMDisplayModeListEntryRec = record
		displayModeFlags: UInt32;
		displayModeSwitchInfo: VDSwitchInfoPtr;
		displayModeResolutionInfo: VDResolutionInfoPtr;
		displayModeTimingInfo: VDTimingInfoPtr;
		displayModeDepthBlockInfo: DMDepthInfoBlockPtr; { Information about all the depths}
		displayModeVersion: UInt32;     { What version is this record (now kDisplayModeEntryVersionOne)}
		displayModeName: StringPtr;        { Name of the timing mode}
		displayModeDisplayInfo: DMDisplayTimingInfoPtr; { Information from the display.}
	end;
type
	DMDisplayModeListEntryPtr = DMDisplayModeListEntryRecPtr;

type
	DependentNotifyRecPtr = ^DependentNotifyRec;
	DependentNotifyRec = record
		notifyType: ResType;             { What type was the engine that made the change (may be zero)}
		notifyClass: ResType;            { What class was the change (eg geometry, color etc)}
		notifyPortID: DisplayIDType;           { Which device was touched (kInvalidDisplayID -> all or none)}
		notifyComponent: ComponentInstance;        { What engine did it (may be 0)?}

		notifyVersion: UInt32;          { Set to 0 (future expansion)}
		notifyFlags: UInt32;            { Set to 0 (future expansion)}
		notifyReserved: UInt32;         { Set to 0 (future expansion)}
		notifyFuture: UInt32;           { Set to 0 (future expansion)}
	end;
type
	DependentNotifyPtr = DependentNotifyRecPtr;

type
	DMMakeAndModelRecPtr = ^DMMakeAndModelRec;
	DMMakeAndModelRec = record
		manufacturer: ResType;
		model: UInt32;
		serialNumber: UInt32;
		manufactureDate: UInt32;

		makeReserved: array [0..3] of UInt32;
	end;
type
	DMMakeAndModelPtr = DMMakeAndModelRecPtr;
{ DMNewDisplayList displayListIncludeFlags}
const
	kIncludeOnlineActiveDisplaysMask = 1 shl 0;
	kIncludeOnlineDisabledDisplaysMask = 1 shl 1;
	kIncludeOfflineDisplaysMask = 1 shl 2;
	kIncludeOfflineDummyDisplaysMask = 1 shl 3;
	kIncludeHardwareMirroredDisplaysMask = 1 shl 4;


const
{ modeListFlags for DMNewDisplayModeList }
	kDMModeListIncludeAllModesMask = 1 shl 0; { Include all timing modes not _explicitly_ excluded (see other bits)}
	kDMModeListIncludeOfflineModesMask = 1 shl 1;
	kDMModeListExcludeDriverModesMask = 1 shl 2; { Exclude old-style timing modes (cscGetNextResolution/kDisplayModeIDFindFirstResolution modes)}
	kDMModeListExcludeDisplayModesMask = 1 shl 3; { Exclude timing modes that come from the display (always arbritrary timing modes)}
	kDMModeListExcludeCustomModesMask = 1 shl 4; { Exclude custom modes that came neither from the driver or display (need a better name)}
	kDMModeListPreferStretchedModesMask = 1 shl 5; { Prefer modes that are stretched over modes that are letterboxed when setting kDisplayModeListNotPreferredBit}
	kDMModeListPreferSafeModesMask = 1 shl 6; { Prefer modes that are safe over modes that are not when setting kDisplayModeListNotPreferredBit}


{ DMNewDisplayList displayListFlags}
type
	DisplayListEntryRecPtr = ^DisplayListEntryRec;
	DisplayListEntryRec = record
		displayListEntryGDevice: GDHandle;
		displayListEntryDisplayID: DisplayIDType;
		displayListEntryIncludeFlags: UInt32; { Reason this entry was included}
		displayListEntryReserved1: UInt32;

		displayListEntryReserved2: UInt32; { Zero}
		displayListEntryReserved3: UInt32; { Zero}
		displayListEntryReserved4: UInt32; { Zero}
		displayListEntryReserved5: UInt32; { Zero}
	end;
type
	DisplayListEntryPtr = DisplayListEntryRecPtr;
	DMProfileListEntryRecPtr = ^DMProfileListEntryRec;
	DMProfileListEntryRec = record
		profileRef: UnivPtr;             { was CMProfileRef}
		profileReserved1: Ptr;       { Reserved}
		profileReserved2: Ptr;       { Reserved}
		profileReserved3: Ptr;       { Reserved}
	end;
type
	DMProfileListEntryPtr = DMProfileListEntryRecPtr;
	DMNotificationProcPtr = procedure( var theEvent: AppleEvent );
	DMExtendedNotificationProcPtr = procedure( userData: UnivPtr; theMessage: SInt16; notifyData: UnivPtr );
	DMComponentListIteratorProcPtr = procedure( userData: UnivPtr; itemIndex: DMListIndexType; componentInfo: DMComponentListEntryPtr );
	DMDisplayModeListIteratorProcPtr = procedure( userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DMDisplayModeListEntryPtr );
	DMProfileListIteratorProcPtr = procedure( userData: UnivPtr; itemIndex: DMListIndexType; profileInfo: DMProfileListEntryPtr );
	DMDisplayListIteratorProcPtr = procedure( userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DisplayListEntryPtr );
	DMNotificationUPP = DMNotificationProcPtr;
	DMExtendedNotificationUPP = DMExtendedNotificationProcPtr;
	DMComponentListIteratorUPP = DMComponentListIteratorProcPtr;
	DMDisplayModeListIteratorUPP = DMDisplayModeListIteratorProcPtr;
	DMProfileListIteratorUPP = DMProfileListIteratorProcPtr;
	DMDisplayListIteratorUPP = DMDisplayListIteratorProcPtr;
{
 *  NewDMNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMNotificationUPP( userRoutine: DMNotificationProcPtr ): DMNotificationUPP; external name '_NewDMNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewDMExtendedNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMExtendedNotificationUPP( userRoutine: DMExtendedNotificationProcPtr ): DMExtendedNotificationUPP; external name '_NewDMExtendedNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewDMComponentListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMComponentListIteratorUPP( userRoutine: DMComponentListIteratorProcPtr ): DMComponentListIteratorUPP; external name '_NewDMComponentListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewDMDisplayModeListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMDisplayModeListIteratorUPP( userRoutine: DMDisplayModeListIteratorProcPtr ): DMDisplayModeListIteratorUPP; external name '_NewDMDisplayModeListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewDMProfileListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMProfileListIteratorUPP( userRoutine: DMProfileListIteratorProcPtr ): DMProfileListIteratorUPP; external name '_NewDMProfileListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewDMDisplayListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDMDisplayListIteratorUPP( userRoutine: DMDisplayListIteratorProcPtr ): DMDisplayListIteratorUPP; external name '_NewDMDisplayListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeDMNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMNotificationUPP( userUPP: DMNotificationUPP ); external name '_DisposeDMNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeDMExtendedNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMExtendedNotificationUPP( userUPP: DMExtendedNotificationUPP ); external name '_DisposeDMExtendedNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeDMComponentListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMComponentListIteratorUPP( userUPP: DMComponentListIteratorUPP ); external name '_DisposeDMComponentListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeDMDisplayModeListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMDisplayModeListIteratorUPP( userUPP: DMDisplayModeListIteratorUPP ); external name '_DisposeDMDisplayModeListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeDMProfileListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMProfileListIteratorUPP( userUPP: DMProfileListIteratorUPP ); external name '_DisposeDMProfileListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeDMDisplayListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDMDisplayListIteratorUPP( userUPP: DMDisplayListIteratorUPP ); external name '_DisposeDMDisplayListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeDMNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMNotificationUPP( var theEvent: AppleEvent; userUPP: DMNotificationUPP ); external name '_InvokeDMNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeDMExtendedNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMExtendedNotificationUPP( userData: UnivPtr; theMessage: SInt16; notifyData: UnivPtr; userUPP: DMExtendedNotificationUPP ); external name '_InvokeDMExtendedNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeDMComponentListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMComponentListIteratorUPP( userData: UnivPtr; itemIndex: DMListIndexType; componentInfo: DMComponentListEntryPtr; userUPP: DMComponentListIteratorUPP ); external name '_InvokeDMComponentListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeDMDisplayModeListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMDisplayModeListIteratorUPP( userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DMDisplayModeListEntryPtr; userUPP: DMDisplayModeListIteratorUPP ); external name '_InvokeDMDisplayModeListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeDMProfileListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMProfileListIteratorUPP( userData: UnivPtr; itemIndex: DMListIndexType; profileInfo: DMProfileListEntryPtr; userUPP: DMProfileListIteratorUPP ); external name '_InvokeDMProfileListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeDMDisplayListIteratorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDMDisplayListIteratorUPP( userData: UnivPtr; itemIndex: DMListIndexType; displaymodeInfo: DisplayListEntryPtr; userUPP: DMDisplayListIteratorUPP ); external name '_InvokeDMDisplayListIteratorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$ifc not TARGET_CPU_64}
{
 *  DMDisplayGestalt()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMUseScreenPrefs()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMSuspendConfigure()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMResumeConfigure()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMSetGammaByAVID()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMGetGammaByAVID()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMGetMakeAndModelByAVID()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMNewDisplayList()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMGetIndexedDisplayFromList()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMNewProfileListByAVID()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMGetIndexedProfileFromList()
 *  
 *  Availability:
 *    Mac OS X:         not available [32-bit only]
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }


{
 *  DMGetFirstScreenDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMGetFirstScreenDevice( activeOnly: Boolean ): GDHandle; external name '_DMGetFirstScreenDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetNextScreenDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMGetNextScreenDevice( theDevice: GDHandle; activeOnly: Boolean ): GDHandle; external name '_DMGetNextScreenDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDrawDesktopRect()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DMDrawDesktopRect( var globalRect: Rect ); external name '_DMDrawDesktopRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDrawDesktopRegion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DMDrawDesktopRegion( globalRgn: RgnHandle ); external name '_DMDrawDesktopRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMBeginConfigureDisplays()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMBeginConfigureDisplays( var displayState: Handle ): OSErr; external name '_DMBeginConfigureDisplays';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMEndConfigureDisplays()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMEndConfigureDisplays( displayState: Handle ): OSErr; external name '_DMEndConfigureDisplays';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMAddDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMAddDisplay( newDevice: GDHandle; driver: SInt16; mode: UInt32; reserved: UInt32; displayID: UInt32; displayComponent: Component; displayState: Handle ): OSErr; external name '_DMAddDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMMoveDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMMoveDisplay( moveDevice: GDHandle; x: SInt16; y: SInt16; displayState: Handle ): OSErr; external name '_DMMoveDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDisableDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMDisableDisplay( disableDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMDisableDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMEnableDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMEnableDisplay( enableDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMEnableDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRemoveDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMRemoveDisplay( removeDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMRemoveDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetMainDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMSetMainDisplay( newMainDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMSetMainDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetDisplayMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMSetDisplayMode( theDevice: GDHandle; mode: UInt32; var depthMode: UInt32; reserved: SIGNEDLONG; displayState: Handle ): OSErr; external name '_DMSetDisplayMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMCheckDisplayMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMCheckDisplayMode( theDevice: GDHandle; mode: UInt32; depthMode: UInt32; var switchFlags: UInt32; reserved: UInt32; var modeOk: Boolean ): OSErr; external name '_DMCheckDisplayMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDeskRegion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMGetDeskRegion( var desktopRegion: RgnHandle ): OSErr; external name '_DMGetDeskRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRegisterNotifyProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMRegisterNotifyProc( notificationProc: DMNotificationUPP; whichPSN: DMProcessInfoPtr ): OSErr; external name '_DMRegisterNotifyProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRemoveNotifyProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMRemoveNotifyProc( notificationProc: DMNotificationUPP; whichPSN: DMProcessInfoPtr ): OSErr; external name '_DMRemoveNotifyProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMQDIsMirroringCapable()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMQDIsMirroringCapable( var qdIsMirroringCapable: Boolean ): OSErr; external name '_DMQDIsMirroringCapable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMCanMirrorNow()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMCanMirrorNow( var canMirrorNow: Boolean ): OSErr; external name '_DMCanMirrorNow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMIsMirroringOn()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMIsMirroringOn( var isMirroringOn: Boolean ): OSErr; external name '_DMIsMirroringOn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMMirrorDevices()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMMirrorDevices( gD1: GDHandle; gD2: GDHandle; displayState: Handle ): OSErr; external name '_DMMirrorDevices';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMUnmirrorDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMUnmirrorDevice( gDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMUnmirrorDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetNextMirroredDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMGetNextMirroredDevice( gDevice: GDHandle; var mirroredDevice: GDHandle ): OSErr; external name '_DMGetNextMirroredDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMBlockMirroring()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMBlockMirroring: OSErr; external name '_DMBlockMirroring';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMUnblockMirroring()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMUnblockMirroring: OSErr; external name '_DMUnblockMirroring';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDisplayMgrA5World()
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
=======
 *    Mac OS X:         not available [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }


{
 *  DMGetDisplayIDByGDevice()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMGetDisplayIDByGDevice( displayDevice: GDHandle; var displayID: DisplayIDType; failToMain: Boolean ): OSErr; external name '_DMGetDisplayIDByGDevice';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetGDeviceByDisplayID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMGetGDeviceByDisplayID( displayID: DisplayIDType; var displayDevice: GDHandle; failToMain: Boolean ): OSErr; external name '_DMGetGDeviceByDisplayID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetDisplayComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMSetDisplayComponent( theDevice: GDHandle; displayComponent: Component ): OSErr; external name '_DMSetDisplayComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDisplayComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMGetDisplayComponent( theDevice: GDHandle; var displayComponent: Component ): OSErr; external name '_DMGetDisplayComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMNewDisplay( var newDevice: GDHandle; driverRefNum: SInt16; mode: UInt32; reserved: UInt32; displayID: DisplayIDType; displayComponent: Component; displayState: Handle ): OSErr; external name '_DMNewDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDisposeDisplay()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.5 and later
 }
function DMDisposeDisplay( disposeDevice: GDHandle; displayState: Handle ): OSErr; external name '_DMDisposeDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMResolveDisplayComponents()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DMResolveDisplayComponents: OSErr; external name '_DMResolveDisplayComponents';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRegisterExtendedNotifyProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.0 and later
 }
function DMRegisterExtendedNotifyProc( notifyProc: DMExtendedNotificationUPP; notifyUserData: UnivPtr; nofifyOnFlags: UInt16; whichPSN: DMProcessInfoPtr ): OSErr; external name '_DMRegisterExtendedNotifyProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMRemoveExtendedNotifyProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.0 and later
 }
function DMRemoveExtendedNotifyProc( notifyProc: DMExtendedNotificationUPP; notifyUserData: UnivPtr; whichPSN: DMProcessInfoPtr; removeFlags: UInt16 ): OSErr; external name '_DMRemoveExtendedNotifyProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVPanelList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVPanelList( displayID: DisplayIDType; panelType: ResType; minimumFidelity: DMFidelityType; panelListFlags: UInt32; reserved: UInt32; var thePanelCount: DMListIndexType; var thePanelList: DMListType ): OSErr; external name '_DMNewAVPanelList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVEngineList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVEngineList( displayID: DisplayIDType; engineType: ResType; minimumFidelity: DMFidelityType; engineListFlags: UInt32; reserved: UInt32; var engineCount: DMListIndexType; var engineList: DMListType ): OSErr; external name '_DMNewAVEngineList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVDeviceList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVDeviceList( deviceType: ResType; deviceListFlags: UInt32; reserved: UInt32; var deviceCount: DMListIndexType; var deviceList: DMListType ): OSErr; external name '_DMNewAVDeviceList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVPortListByPortType()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVPortListByPortType( subType: ResType; portListFlags: UInt32; reserved: UInt32; var devicePortCount: DMListIndexType; var theDevicePortList: DMListType ): OSErr; external name '_DMNewAVPortListByPortType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetIndexedComponentFromList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetIndexedComponentFromList( panelList: DMListType; itemIndex: DMListIndexType; reserved: UInt32; listIterator: DMComponentListIteratorUPP; userData: UnivPtr ): OSErr; external name '_DMGetIndexedComponentFromList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDisposeList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.0 and later
 }
function DMDisposeList( panelList: DMListType ): OSErr; external name '_DMDisposeList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetNameByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetNameByAVID( theID: AVIDType; nameFlags: UInt32; var name: Str255 ): OSErr; external name '_DMGetNameByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVIDByPortComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVIDByPortComponent( thePortComponent: Component; portKind: ResType; reserved: UInt32; var newID: AVIDType ): OSErr; external name '_DMNewAVIDByPortComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetPortComponentByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetPortComponentByAVID( thePortID: DisplayIDType; var thePortComponent: Component; var theDesciption: ComponentDescription; var thePortKind: ResType ): OSErr; external name '_DMGetPortComponentByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSendDependentNotification()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.0 and later
 }
function DMSendDependentNotification( notifyType: ResType; notifyClass: ResType; displayID: AVIDType; notifyComponent: ComponentInstance ): OSErr; external name '_DMSendDependentNotification';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMDisposeAVComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMDisposeAVComponent( theAVComponent: Component ): OSErr; external name '_DMDisposeAVComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSaveScreenPrefs()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMSaveScreenPrefs( reserved1: UInt32; saveFlags: UInt32; reserved2: UInt32 ): OSErr; external name '_DMSaveScreenPrefs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVIDByDeviceComponent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVIDByDeviceComponent( theDeviceComponent: Component; portKind: ResType; reserved: UInt32; var newID: DisplayIDType ): OSErr; external name '_DMNewAVIDByDeviceComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewAVPortListByDeviceAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewAVPortListByDeviceAVID( theID: AVIDType; minimumFidelity: DMFidelityType; portListFlags: UInt32; reserved: UInt32; var devicePortCount: DMListIndexType; var theDevicePortList: DMListType ): OSErr; external name '_DMNewAVPortListByDeviceAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDeviceComponentByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetDeviceComponentByAVID( theDeviceID: AVIDType; var theDeviceComponent: Component; var theDesciption: ComponentDescription; var theDeviceKind: ResType ): OSErr; external name '_DMGetDeviceComponentByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMNewDisplayModeList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMNewDisplayModeList( displayID: DisplayIDType; modeListFlags: UInt32; reserved: UInt32; var thePanelCount: DMListIndexType; var thePanelList: DMListType ): OSErr; external name '_DMNewDisplayModeList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetIndexedDisplayModeFromList()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetIndexedDisplayModeFromList( panelList: DMListType; itemIndex: DMListIndexType; reserved: UInt32; listIterator: DMDisplayModeListIteratorUPP; userData: UnivPtr ): OSErr; external name '_DMGetIndexedDisplayModeFromList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetGraphicInfoByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetGraphicInfoByAVID( theID: AVIDType; var theAVPcit: PicHandle; var theAVIconSuite: Handle; var theAVLocation: AVLocationRec ): OSErr; external name '_DMGetGraphicInfoByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetAVPowerState()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetAVPowerState( theID: AVIDType; getPowerState: AVPowerStatePtr; reserved1: UInt32 ): OSErr; external name '_DMGetAVPowerState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetAVPowerState()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMSetAVPowerState( theID: AVIDType; setPowerState: AVPowerStatePtr; powerFlags: UInt32; displayState: Handle ): OSErr; external name '_DMSetAVPowerState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDeviceAVIDByPortAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetDeviceAVIDByPortAVID( portAVID: AVIDType; var deviceAVID: AVIDType ): OSErr; external name '_DMGetDeviceAVIDByPortAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetEnableByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetEnableByAVID( theAVID: AVIDType; var isAVIDEnabledNow: Boolean; var canChangeEnableNow: Boolean ): OSErr; external name '_DMGetEnableByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMSetEnableByAVID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMSetEnableByAVID( theAVID: AVIDType; doEnable: Boolean; displayState: Handle ): OSErr; external name '_DMSetEnableByAVID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMGetDisplayMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib68k 2.0 and later
 }
function DMGetDisplayMode( theDevice: GDHandle; switchInfo: VDSwitchInfoPtr ): OSErr; external name '_DMGetDisplayMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DMConfirmConfiguration()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DisplayLib 2.1 and later
 }
function DMConfirmConfiguration( filterProc: DMModalFilterUPP; confirmFlags: UInt32; reserved: UInt32; displayState: Handle ): OSErr; external name '_DMConfirmConfiguration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

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
