{
<<<<<<< HEAD
     File:       HIServices/Icons.h
 
     Contains:   Icon Utilities and Icon Services Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    HIServices-416~44
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> origin/cpstrnew
 
     Copyright:  � 1990-2008 by Apple Computer, Inc. All rights reserved
=======
     File:       Icons.p
 
     Contains:   Icon Utilities and Icon Services Interfaces.
 
     Version:    Technology: Mac OS 9.x
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  � 1990-2002 by Apple Computer, Inc. All rights reserved
>>>>>>> graemeg/fixes_2_2
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}

<<<<<<< HEAD
{  Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{    Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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

{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
                     http://www.freepascal.org/bugs.html
 
}

{    Pascal Translation Updated:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }

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

unit Icons;
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elsec}
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
<<<<<<< HEAD
uses MacTypes,CFBase,CGGeometry,QuickdrawTypes,IconStorage,IconsCore,Files,CGContext,HIShape;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ Alignment type values. }
const
	kAlignNone = $00;
	kAlignVerticalCenter = $01;
	kAlignTop = $02;
	kAlignBottom = $03;
	kAlignHorizontalCenter = $04;
	kAlignAbsoluteCenter = kAlignVerticalCenter or kAlignHorizontalCenter;
	kAlignCenterTop = kAlignTop or kAlignHorizontalCenter;
	kAlignCenterBottom = kAlignBottom or kAlignHorizontalCenter;
	kAlignLeft = $08;
	kAlignCenterLeft = kAlignVerticalCenter or kAlignLeft;
	kAlignTopLeft = kAlignTop or kAlignLeft;
	kAlignBottomLeft = kAlignBottom or kAlignLeft;
	kAlignRight = $0C;
	kAlignCenterRight = kAlignVerticalCenter or kAlignRight;
	kAlignTopRight = kAlignTop or kAlignRight;
	kAlignBottomRight = kAlignBottom or kAlignRight;

{ Obsolete. Use names defined above. }
const
	atNone = kAlignNone;
	atVerticalCenter = kAlignVerticalCenter;
	atTop = kAlignTop;
	atBottom = kAlignBottom;
	atHorizontalCenter = kAlignHorizontalCenter;
	atAbsoluteCenter = kAlignAbsoluteCenter;
	atCenterTop = kAlignCenterTop;
	atCenterBottom = kAlignCenterBottom;
	atLeft = kAlignLeft;
	atCenterLeft = kAlignCenterLeft;
	atTopLeft = kAlignTopLeft;
	atBottomLeft = kAlignBottomLeft;
	atRight = kAlignRight;
	atCenterRight = kAlignCenterRight;
	atTopRight = kAlignTopRight;
	atBottomRight = kAlignBottomRight;

type
	IconAlignmentType = SInt16;
{ Transform type values. }
const
	kTransformNone = $00;
	kTransformDisabled = $01;
	kTransformOffline = $02;
	kTransformOpen = $03;
	kTransformLabel1 = $0100;
	kTransformLabel2 = $0200;
	kTransformLabel3 = $0300;
	kTransformLabel4 = $0400;
	kTransformLabel5 = $0500;
	kTransformLabel6 = $0600;
	kTransformLabel7 = $0700;
	kTransformSelected = $4000;
	kTransformSelectedDisabled = kTransformSelected or kTransformDisabled;
	kTransformSelectedOffline = kTransformSelected or kTransformOffline;
	kTransformSelectedOpen = kTransformSelected or kTransformOpen;

{ Obsolete. Use names defined above. }
const
	ttNone = kTransformNone;
	ttDisabled = kTransformDisabled;
	ttOffline = kTransformOffline;
	ttOpen = kTransformOpen;
	ttLabel1 = kTransformLabel1;
	ttLabel2 = kTransformLabel2;
	ttLabel3 = kTransformLabel3;
	ttLabel4 = kTransformLabel4;
	ttLabel5 = kTransformLabel5;
	ttLabel6 = kTransformLabel6;
	ttLabel7 = kTransformLabel7;
	ttSelected = kTransformSelected;
	ttSelectedDisabled = kTransformSelectedDisabled;
	ttSelectedOffline = kTransformSelectedOffline;
	ttSelectedOpen = kTransformSelectedOpen;

type
	IconTransformType = SInt16;
{ Selector mask values. }
const
	kSelectorLarge1Bit = $00000001;
	kSelectorLarge4Bit = $00000002;
	kSelectorLarge8Bit = $00000004;
	kSelectorLarge32Bit = $00000008;
	kSelectorLarge8BitMask = $00000010;
	kSelectorSmall1Bit = $00000100;
	kSelectorSmall4Bit = $00000200;
	kSelectorSmall8Bit = $00000400;
	kSelectorSmall32Bit = $00000800;
	kSelectorSmall8BitMask = $00001000;
	kSelectorMini1Bit = $00010000;
	kSelectorMini4Bit = $00020000;
	kSelectorMini8Bit = $00040000;
	kSelectorHuge1Bit = $01000000;
	kSelectorHuge4Bit = $02000000;
	kSelectorHuge8Bit = $04000000;
	kSelectorHuge32Bit = $08000000;
	kSelectorHuge8BitMask = $10000000;
	kSelectorAllLargeData = $000000FF;
	kSelectorAllSmallData = $0000FF00;
	kSelectorAllMiniData = $00FF0000;
	kSelectorAllHugeData = $FF000000;
	kSelectorAll1BitData = kSelectorLarge1Bit or kSelectorSmall1Bit or kSelectorMini1Bit or kSelectorHuge1Bit;
	kSelectorAll4BitData = kSelectorLarge4Bit or kSelectorSmall4Bit or kSelectorMini4Bit or kSelectorHuge4Bit;
	kSelectorAll8BitData = kSelectorLarge8Bit or kSelectorSmall8Bit or kSelectorMini8Bit or kSelectorHuge8Bit;
	kSelectorAll32BitData = kSelectorLarge32Bit or kSelectorSmall32Bit or kSelectorHuge32Bit;
	kSelectorAllAvailableData = $FFFFFFFF;

type
	IconSelectorValue = UInt32;
{ Obsolete. Use names defined above. }
const
	svLarge1Bit = kSelectorLarge1Bit;
	svLarge4Bit = kSelectorLarge4Bit;
	svLarge8Bit = kSelectorLarge8Bit;
	svSmall1Bit = kSelectorSmall1Bit;
	svSmall4Bit = kSelectorSmall4Bit;
	svSmall8Bit = kSelectorSmall8Bit;
	svMini1Bit = kSelectorMini1Bit;
	svMini4Bit = kSelectorMini4Bit;
	svMini8Bit = kSelectorMini8Bit;
	svAllLargeData = kSelectorAllLargeData;
	svAllSmallData = kSelectorAllSmallData;
	svAllMiniData = kSelectorAllMiniData;
	svAll1BitData = kSelectorAll1BitData;
	svAll4BitData = kSelectorAll4BitData;
	svAll8BitData = kSelectorAll8BitData;
	svAllAvailableData = kSelectorAllAvailableData;

type
	IconActionProcPtr = function( theType: ResType; var theIcon: Handle; yourDataPtr: UnivPtr ): OSErr;
	IconGetterProcPtr = function( theType: ResType; yourDataPtr: UnivPtr ): Handle;
	IconActionUPP = IconActionProcPtr;
	IconGetterUPP = IconGetterProcPtr;
{
 *  NewIconActionUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewIconActionUPP( userRoutine: IconActionProcPtr ): IconActionUPP; external name '_NewIconActionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
uses MacTypes,CFBase,CGGeometry,Quickdraw,Files,CodeFragments,CGContext;


{$ALIGN MAC68K}

{ The following are icons for which there are both icon suites and SICNs. }
{ Avoid using icon resources if possible. Use IconServices instead. }

const
	kGenericDocumentIconResource = -4000;
	kGenericStationeryIconResource = -3985;
	kGenericEditionFileIconResource = -3989;
	kGenericApplicationIconResource = -3996;
	kGenericDeskAccessoryIconResource = -3991;
	kGenericFolderIconResource	= -3999;
	kPrivateFolderIconResource	= -3994;
	kFloppyIconResource			= -3998;
	kTrashIconResource			= -3993;
	kGenericRAMDiskIconResource	= -3988;
	kGenericCDROMIconResource	= -3987;

	{	 The following are icons for which there are SICNs only. 	}
	{	 Avoid using icon resources if possible. Use IconServices instead. 	}
	kDesktopIconResource		= -3992;
	kOpenFolderIconResource		= -3997;
	kGenericHardDiskIconResource = -3995;
	kGenericFileServerIconResource = -3972;
	kGenericSuitcaseIconResource = -3970;
	kGenericMoverObjectIconResource = -3969;

	{	 The following are icons for which there are icon suites only. 	}
	{	 Avoid using icon resources if possible. Use IconServices instead. 	}
	kGenericPreferencesIconResource = -3971;
	kGenericQueryDocumentIconResource = -16506;
	kGenericExtensionIconResource = -16415;
	kSystemFolderIconResource	= -3983;
	kHelpIconResource			= -20271;
	kAppleMenuFolderIconResource = -3982;

	{	 Obsolete. Use named constants defined above. 	}
	genericDocumentIconResource	= -4000;
	genericStationeryIconResource = -3985;
	genericEditionFileIconResource = -3989;
	genericApplicationIconResource = -3996;
	genericDeskAccessoryIconResource = -3991;
	genericFolderIconResource	= -3999;
	privateFolderIconResource	= -3994;
	floppyIconResource			= -3998;
	trashIconResource			= -3993;
	genericRAMDiskIconResource	= -3988;
	genericCDROMIconResource	= -3987;
	desktopIconResource			= -3992;
	openFolderIconResource		= -3997;
	genericHardDiskIconResource	= -3995;
	genericFileServerIconResource = -3972;
	genericSuitcaseIconResource	= -3970;
	genericMoverObjectIconResource = -3969;
	genericPreferencesIconResource = -3971;
	genericQueryDocumentIconResource = -16506;
	genericExtensionIconResource = -16415;
	systemFolderIconResource	= -3983;
	appleMenuFolderIconResource	= -3982;

	{	 Avoid using icon resources if possible. Use IconServices instead. 	}
	kStartupFolderIconResource	= -3981;
	kOwnedFolderIconResource	= -3980;
	kDropFolderIconResource		= -3979;
	kSharedFolderIconResource	= -3978;
	kMountedFolderIconResource	= -3977;
	kControlPanelFolderIconResource = -3976;
	kPrintMonitorFolderIconResource = -3975;
	kPreferencesFolderIconResource = -3974;
	kExtensionsFolderIconResource = -3973;
	kFontsFolderIconResource	= -3968;
	kFullTrashIconResource		= -3984;

	{	 Obsolete. Use named constants defined above. 	}
	startupFolderIconResource	= -3981;
	ownedFolderIconResource		= -3980;
	dropFolderIconResource		= -3979;
	sharedFolderIconResource	= -3978;
	mountedFolderIconResource	= -3977;
	controlPanelFolderIconResource = -3976;
	printMonitorFolderIconResource = -3975;
	preferencesFolderIconResource = -3974;
	extensionsFolderIconResource = -3973;
	fontsFolderIconResource		= -3968;
	fullTrashIconResource		= -3984;

	{	 The following icon types can only be used as an icon element 	}
	{	 inside a 'icns' icon family 	}
    kIconServices256PixelDataARGB = FourCharCode('ic08');
	kThumbnail32BitData			= FourCharCode('it32');
	kThumbnail8BitMask			= FourCharCode('t8mk');

	kHuge1BitMask				= FourCharCode('ich#');
	kHuge4BitData				= FourCharCode('ich4');
	kHuge8BitData				= FourCharCode('ich8');
	kHuge32BitData				= FourCharCode('ih32');
	kHuge8BitMask				= FourCharCode('h8mk');

	{	 The following icon types can be used as a resource type 	}
	{	 or as an icon element type inside a 'icns' icon family 	}
	kLarge1BitMask				= FourCharCode('ICN#');
	kLarge4BitData				= FourCharCode('icl4');
	kLarge8BitData				= FourCharCode('icl8');
	kLarge32BitData				= FourCharCode('il32');
	kLarge8BitMask				= FourCharCode('l8mk');
	kSmall1BitMask				= FourCharCode('ics#');
	kSmall4BitData				= FourCharCode('ics4');
	kSmall8BitData				= FourCharCode('ics8');
	kSmall32BitData				= FourCharCode('is32');
	kSmall8BitMask				= FourCharCode('s8mk');
	kMini1BitMask				= FourCharCode('icm#');
	kMini4BitData				= FourCharCode('icm4');
	kMini8BitData				= FourCharCode('icm8');

	{	  Icon Variants 	}
	{	 These can be used as an element of an 'icns' icon family 	}
	{	 or as a parameter to GetIconRefVariant 	}
	kTileIconVariant			= FourCharCode('tile');
	kRolloverIconVariant		= FourCharCode('over');
	kDropIconVariant			= FourCharCode('drop');
	kOpenIconVariant			= FourCharCode('open');
	kOpenDropIconVariant		= FourCharCode('odrp');

	{	 Obsolete. Use names defined above. 	}
	large1BitMask				= FourCharCode('ICN#');
	large4BitData				= FourCharCode('icl4');
	large8BitData				= FourCharCode('icl8');
	small1BitMask				= FourCharCode('ics#');
	small4BitData				= FourCharCode('ics4');
	small8BitData				= FourCharCode('ics8');
	mini1BitMask				= FourCharCode('icm#');
	mini4BitData				= FourCharCode('icm4');
	mini8BitData				= FourCharCode('icm8');

	{	 Alignment type values. 	}
	kAlignNone					= $00;
	kAlignVerticalCenter		= $01;
	kAlignTop					= $02;
	kAlignBottom				= $03;
	kAlignHorizontalCenter		= $04;
	kAlignAbsoluteCenter		= $05;
	kAlignCenterTop				= $06;
	kAlignCenterBottom			= $07;
	kAlignLeft					= $08;
	kAlignCenterLeft			= $09;
	kAlignTopLeft				= $0A;
	kAlignBottomLeft			= $0B;
	kAlignRight					= $0C;
	kAlignCenterRight			= $0D;
	kAlignTopRight				= $0E;
	kAlignBottomRight			= $0F;

	{	 Obsolete. Use names defined above. 	}
	atNone						= $00;
	atVerticalCenter			= $01;
	atTop						= $02;
	atBottom					= $03;
	atHorizontalCenter			= $04;
	atAbsoluteCenter			= $05;
	atCenterTop					= $06;
	atCenterBottom				= $07;
	atLeft						= $08;
	atCenterLeft				= $09;
	atTopLeft					= $0A;
	atBottomLeft				= $0B;
	atRight						= $0C;
	atCenterRight				= $0D;
	atTopRight					= $0E;
	atBottomRight				= $0F;


type
	IconAlignmentType					= SInt16;
	{	 Transform type values. 	}

const
	kTransformNone				= $00;
	kTransformDisabled			= $01;
	kTransformOffline			= $02;
	kTransformOpen				= $03;
	kTransformLabel1			= $0100;
	kTransformLabel2			= $0200;
	kTransformLabel3			= $0300;
	kTransformLabel4			= $0400;
	kTransformLabel5			= $0500;
	kTransformLabel6			= $0600;
	kTransformLabel7			= $0700;
	kTransformSelected			= $4000;
	kTransformSelectedDisabled	= $4001;
	kTransformSelectedOffline	= $4002;
	kTransformSelectedOpen		= $4003;

	{	 Obsolete. Use names defined above. 	}
	ttNone						= $00;
	ttDisabled					= $01;
	ttOffline					= $02;
	ttOpen						= $03;
	ttLabel1					= $0100;
	ttLabel2					= $0200;
	ttLabel3					= $0300;
	ttLabel4					= $0400;
	ttLabel5					= $0500;
	ttLabel6					= $0600;
	ttLabel7					= $0700;
	ttSelected					= $4000;
	ttSelectedDisabled			= $4001;
	ttSelectedOffline			= $4002;
	ttSelectedOpen				= $4003;


type
	IconTransformType					= SInt16;
	{	 Selector mask values. 	}

const
	kSelectorLarge1Bit			= $00000001;
	kSelectorLarge4Bit			= $00000002;
	kSelectorLarge8Bit			= $00000004;
	kSelectorLarge32Bit			= $00000008;
	kSelectorLarge8BitMask		= $00000010;
	kSelectorSmall1Bit			= $00000100;
	kSelectorSmall4Bit			= $00000200;
	kSelectorSmall8Bit			= $00000400;
	kSelectorSmall32Bit			= $00000800;
	kSelectorSmall8BitMask		= $00001000;
	kSelectorMini1Bit			= $00010000;
	kSelectorMini4Bit			= $00020000;
	kSelectorMini8Bit			= $00040000;
	kSelectorHuge1Bit			= $01000000;
	kSelectorHuge4Bit			= $02000000;
	kSelectorHuge8Bit			= $04000000;
	kSelectorHuge32Bit			= $08000000;
	kSelectorHuge8BitMask		= $10000000;
	kSelectorAllLargeData		= $000000FF;
	kSelectorAllSmallData		= $0000FF00;
	kSelectorAllMiniData		= $00FF0000;
	kSelectorAllHugeData		= $FF000000;
	kSelectorAll1BitData		= $01010101;
	kSelectorAll4BitData		= $02020202;
	kSelectorAll8BitData		= $04040404;
	kSelectorAll32BitData		= $08000808;
	kSelectorAllAvailableData	= $FFFFFFFF;


	{	 Obsolete. Use names defined above. 	}
	svLarge1Bit					= $00000001;
	svLarge4Bit					= $00000002;
	svLarge8Bit					= $00000004;
	svSmall1Bit					= $00000100;
	svSmall4Bit					= $00000200;
	svSmall8Bit					= $00000400;
	svMini1Bit					= $00010000;
	svMini4Bit					= $00020000;
	svMini8Bit					= $00040000;
	svAllLargeData				= $000000FF;
	svAllSmallData				= $0000FF00;
	svAllMiniData				= $00FF0000;
	svAll1BitData				= $01010101;
	svAll4BitData				= $02020202;
	svAll8BitData				= $04040404;
	svAllAvailableData			= $FFFFFFFF;


type
	IconSelectorValue					= UInt32;
{$ifc TYPED_FUNCTION_POINTERS}
	IconActionProcPtr = function(theType: ResType; var theIcon: Handle; yourDataPtr: UnivPtr): OSErr;
{$elsec}
	IconActionProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	IconGetterProcPtr = function(theType: ResType; yourDataPtr: UnivPtr): Handle;
{$elsec}
	IconGetterProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	IconActionUPP = ^SInt32; { an opaque UPP }
{$elsec}
	IconActionUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	IconGetterUPP = ^SInt32; { an opaque UPP }
{$elsec}
	IconGetterUPP = UniversalProcPtr;
{$endc}	

const
	uppIconActionProcInfo = $00000FE0;
	uppIconGetterProcInfo = $000003F0;
	{
	 *  NewIconActionUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewIconActionUPP(userRoutine: IconActionProcPtr): IconActionUPP; external name '_NewIconActionUPP'; { old name was NewIconActionProc }
>>>>>>> graemeg/fixes_2_2
{
 *  NewIconGetterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
 { old name was NewIconGetterProc }
 function NewIconGetterUPP( userRoutine: IconGetterProcPtr ): IconGetterUPP; external name '_NewIconGetterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function NewIconGetterUPP(userRoutine: IconGetterProcPtr): IconGetterUPP; external name '_NewIconGetterUPP'; { old name was NewIconGetterProc }
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeIconActionUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeIconActionUPP( userUPP: IconActionUPP ); external name '_DisposeIconActionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeIconActionUPP(userUPP: IconActionUPP); external name '_DisposeIconActionUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeIconGetterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeIconGetterUPP( userUPP: IconGetterUPP ); external name '_DisposeIconGetterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeIconGetterUPP(userUPP: IconGetterUPP); external name '_DisposeIconGetterUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeIconActionUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
 { old name was CallIconActionProc }
 function InvokeIconActionUPP( theType: ResType; var theIcon: Handle; yourDataPtr: UnivPtr; userUPP: IconActionUPP ): OSErr; external name '_InvokeIconActionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function InvokeIconActionUPP(theType: ResType; var theIcon: Handle; yourDataPtr: UnivPtr; userRoutine: IconActionUPP): OSErr; external name '_InvokeIconActionUPP'; { old name was CallIconActionProc }
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeIconGetterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeIconGetterUPP( theType: ResType; yourDataPtr: UnivPtr; userUPP: IconGetterUPP ): Handle; external name '_InvokeIconGetterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc not TARGET_CPU_64}
type
	IconGetter = IconGetterProcPtr;
	IconAction = IconActionProcPtr;
	{  CIconHandle, GetCIcon(), PlotCIcon(), and DisposeCIcon() moved here from Quickdraw.h }
	 CIcon = record
		iconPMap: PixMap;               {the icon's pixMap}
		iconMask: BitMap;               {the icon's mask}
		iconBMap: BitMap;               {the icon's bitMap}
		iconData: Handle;               {the icon's data}
		iconMaskData: array [0..0] of SInt16;        {icon's mask and BitMap data}
	end;
	CIconPtr = ^CIcon;
type
	CIconHandle = ^CIconPtr;
	CIconHandle_fix = ^CIconPtr; { used as field type when a record declaration contains a CIconHandle field identifier }
{
    Note:   IconSuiteRef and IconCacheRef should be an abstract types, 
            but too much source code already relies on them being of type Handle.
}
type
	IconSuiteRef = Handle;
	IconCacheRef = Handle;

{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}

{
 *  GetCIcon()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetCIcon( iconID: SInt16 ): CIconHandle; external name '_GetCIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotCIcon()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure PlotCIcon( const (*var*) theRect: Rect; theIcon: CIconHandle ); external name '_PlotCIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DisposeCIcon()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure DisposeCIcon( theIcon: CIconHandle ); external name '_DisposeCIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetIcon()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetIcon( iconID: SInt16 ): Handle; external name '_GetIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotIcon()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure PlotIcon( const (*var*) theRect: Rect; theIcon: Handle ); external name '_PlotIcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotIconID()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotIconID( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theResID: SInt16 ): OSErr; external name '_PlotIconID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NewIconSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewIconSuite( var theIconSuite: IconSuiteRef ): OSErr; external name '_NewIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AddIconToSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AddIconToSuite( theIconData: Handle; theSuite: IconSuiteRef; theType: ResType ): OSErr; external name '_AddIconToSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetIconFromSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetIconFromSuite( var theIconData: Handle; theSuite: IconSuiteRef; theType: ResType ): OSErr; external name '_GetIconFromSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ForEachIconDo()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ForEachIconDo( theSuite: IconSuiteRef; selector: IconSelectorValue; action: IconActionUPP; yourDataPtr: UnivPtr ): OSErr; external name '_ForEachIconDo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetIconSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetIconSuite( var theIconSuite: IconSuiteRef; theResID: SInt16; selector: IconSelectorValue ): OSErr; external name '_GetIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DisposeIconSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DisposeIconSuite( theIconSuite: IconSuiteRef; disposeData: Boolean ): OSErr; external name '_DisposeIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotIconSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotIconSuite( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIconSuite: IconSuiteRef ): OSErr; external name '_PlotIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  MakeIconCache()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MakeIconCache( var theCache: IconCacheRef; makeIcon: IconGetterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MakeIconCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  LoadIconCache()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LoadIconCache( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIconCache: IconCacheRef ): OSErr; external name '_LoadIconCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotIconMethod()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotIconMethod( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_PlotIconMethod';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetLabel()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetLabel( labelNumber: SInt16; var labelColor: RGBColor; var labelString: Str255 ): OSErr; external name '_GetLabel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PtInIconID()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PtInIconID( testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; iconID: SInt16 ): Boolean; external name '_PtInIconID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PtInIconSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PtInIconSuite( testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconSuite: IconSuiteRef ): Boolean; external name '_PtInIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PtInIconMethod()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PtInIconMethod( testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr ): Boolean; external name '_PtInIconMethod';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RectInIconID()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RectInIconID( const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; iconID: SInt16 ): Boolean; external name '_RectInIconID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RectInIconSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RectInIconSuite( const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconSuite: IconSuiteRef ): Boolean; external name '_RectInIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RectInIconMethod()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RectInIconMethod( const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr ): Boolean; external name '_RectInIconMethod';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IconIDToRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function IconIDToRgn( theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; iconID: SInt16 ): OSErr; external name '_IconIDToRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IconSuiteToRgn()   *** DEPRECATED ***
<<<<<<< HEAD
 *  
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function IconSuiteToRgn( theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconSuite: IconSuiteRef ): OSErr; external name '_IconSuiteToRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  IconMethodToRgn()   *** DEPRECATED ***
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 }
function IconSuiteToRgn( theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconSuite: IconSuiteRef ): OSErr; external name '_IconSuiteToRgn';
=======
 }
function IconMethodToRgn( theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_IconMethodToRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetSuiteLabel()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetSuiteLabel( theSuite: IconSuiteRef; theLabel: SInt16 ): OSErr; external name '_SetSuiteLabel';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  IconMethodToRgn()   *** DEPRECATED ***
 *  
=======
 *  GetSuiteLabel()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetSuiteLabel( theSuite: IconSuiteRef ): SInt16; external name '_GetSuiteLabel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetIconCacheData()   *** DEPRECATED ***
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 }
function IconMethodToRgn( theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_IconMethodToRgn';
=======
 }
function GetIconCacheData( theCache: IconCacheRef; var theData: UnivPtr ): OSErr; external name '_GetIconCacheData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetIconCacheData()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetIconCacheData( theCache: IconCacheRef; theData: UnivPtr ): OSErr; external name '_SetIconCacheData';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetSuiteLabel()   *** DEPRECATED ***
 *  
=======
 *  GetIconCacheProc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetIconCacheProc( theCache: IconCacheRef; var theProc: IconGetterUPP ): OSErr; external name '_GetIconCacheProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetIconCacheProc()   *** DEPRECATED ***
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
<<<<<<< HEAD
 }
function SetSuiteLabel( theSuite: IconSuiteRef; theLabel: SInt16 ): OSErr; external name '_SetSuiteLabel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
 }
function SetIconCacheProc( theCache: IconCacheRef; theProc: IconGetterUPP ): OSErr; external name '_SetIconCacheProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotIconHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotIconHandle( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIcon: Handle ): OSErr; external name '_PlotIconHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotSICNHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotSICNHandle( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theSICN: Handle ): OSErr; external name '_PlotSICNHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotCIconHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotCIconHandle( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theCIcon: CIconHandle ): OSErr; external name '_PlotCIconHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
{
 *  GetSuiteLabel()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======

{
 *  PlotIconRefFlags
 *  
 *  Discussion:
 *    Flags that can be passed to the PlotIconRefInContext routine.
 }
type
	PlotIconRefFlags = UInt32;
const
{
   * Draw the icon image and alpha channel.
   }
	kPlotIconRefNormalFlags = 0;

  {
   * Only draw the alpha channel.
   }
	kPlotIconRefNoImage = 1 shl 1;

  {
   * Only draw the image.
   }
	kPlotIconRefNoMask = 1 shl 2;

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function InvokeIconGetterUPP(theType: ResType; yourDataPtr: UnivPtr; userRoutine: IconGetterUPP): Handle; external name '_InvokeIconGetterUPP'; { old name was CallIconGetterProc }
type
	IconGetter							= IconGetterProcPtr;
	IconAction							= IconActionProcPtr;
	{  CIconHandle, GetCIcon(), PlotCIcon(), and DisposeCIcon() moved here from Quickdraw.h }
	CIconPtr = ^CIcon;
	CIcon = record
		iconPMap:				PixMap;									{ the icon's pixMap }
		iconMask:				BitMap;									{ the icon's mask }
		iconBMap:				BitMap;									{ the icon's bitMap }
		iconData:				Handle;									{ the icon's data }
		iconMaskData:			array [0..0] of SInt16;					{ icon's mask and BitMap data }
	end;

	CIconHandle							= ^CIconPtr;
	CIconHandle_fix                     = CIconHandle; { used as field type when a record declaration contains a CIconHandle field identifier }
	{
	 *  GetCIcon()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function GetCIcon(iconID: SInt16): CIconHandle; external name '_GetCIcon';
{
 *  PlotCIcon()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure PlotCIcon(const (*var*) theRect: Rect; theIcon: CIconHandle); external name '_PlotCIcon';
{
 *  DisposeCIcon()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeCIcon(theIcon: CIconHandle); external name '_DisposeCIcon';
{  GetIcon and PlotIcon moved here from ToolUtils }
{
 *  GetIcon()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIcon(iconID: SInt16): Handle; external name '_GetIcon';
{
 *  PlotIcon()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure PlotIcon(const (*var*) theRect: Rect; theIcon: Handle); external name '_PlotIcon';
{
    Note:   IconSuiteRef and IconCacheRef should be an abstract types, 
            but too much source code already relies on them being of type Handle.
}

type
	IconSuiteRef						= Handle;
	IconCacheRef						= Handle;
	{  IconRefs are 32-bit values identifying cached icon data. IconRef 0 is invalid. }
	IconRef     = ^SInt32;  { an opaque 32-bit type }
	IconRef_fix = IconRef;  { used as field type when a record declaration contains a IconRef field identifier }
	IconRefPtr  = ^IconRef; { when a var xx:IconRef parameter can be nil, it is changed to xx: IconRefPtr }
	{
	 *  PlotIconID()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function PlotIconID(const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theResID: SInt16): OSErr; external name '_PlotIconID';
{
 *  NewIconSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function NewIconSuite(var theIconSuite: IconSuiteRef): OSErr; external name '_NewIconSuite';
{
 *  AddIconToSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AddIconToSuite(theIconData: Handle; theSuite: IconSuiteRef; theType: ResType): OSErr; external name '_AddIconToSuite';
{
 *  GetIconFromSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconFromSuite(var theIconData: Handle; theSuite: IconSuiteRef; theType: ResType): OSErr; external name '_GetIconFromSuite';
{
 *  ForEachIconDo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ForEachIconDo(theSuite: IconSuiteRef; selector: IconSelectorValue; action: IconActionUPP; yourDataPtr: UnivPtr): OSErr; external name '_ForEachIconDo';
{
 *  GetIconSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconSuite(var theIconSuite: IconSuiteRef; theResID: SInt16; selector: IconSelectorValue): OSErr; external name '_GetIconSuite';
{
 *  DisposeIconSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function DisposeIconSuite(theIconSuite: IconSuiteRef; disposeData: boolean): OSErr; external name '_DisposeIconSuite';
{
 *  PlotIconSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PlotIconSuite(const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIconSuite: IconSuiteRef): OSErr; external name '_PlotIconSuite';
{
 *  MakeIconCache()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function MakeIconCache(var theCache: IconCacheRef; makeIcon: IconGetterUPP; yourDataPtr: UnivPtr): OSErr; external name '_MakeIconCache';
{
 *  LoadIconCache()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function LoadIconCache(const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIconCache: IconCacheRef): OSErr; external name '_LoadIconCache';
{
 *  PlotIconMethod()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PlotIconMethod(const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr): OSErr; external name '_PlotIconMethod';
{
 *  GetLabel()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetLabel(labelNumber: SInt16; var labelColor: RGBColor; var labelString: Str255): OSErr; external name '_GetLabel';
{
 *  PtInIconID()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PtInIconID(testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; iconID: SInt16): boolean; external name '_PtInIconID';
{
 *  PtInIconSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PtInIconSuite(testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconSuite: IconSuiteRef): boolean; external name '_PtInIconSuite';
{
 *  PtInIconMethod()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PtInIconMethod(testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr): boolean; external name '_PtInIconMethod';
{
 *  RectInIconID()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function RectInIconID(const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; iconID: SInt16): boolean; external name '_RectInIconID';
{
 *  RectInIconSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function RectInIconSuite(const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconSuite: IconSuiteRef): boolean; external name '_RectInIconSuite';
{
 *  RectInIconMethod()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function RectInIconMethod(const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr): boolean; external name '_RectInIconMethod';
{
 *  IconIDToRgn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IconIDToRgn(theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; iconID: SInt16): OSErr; external name '_IconIDToRgn';
{
 *  IconSuiteToRgn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IconSuiteToRgn(theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconSuite: IconSuiteRef): OSErr; external name '_IconSuiteToRgn';
{
 *  IconMethodToRgn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IconMethodToRgn(theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; theMethod: IconGetterUPP; yourDataPtr: UnivPtr): OSErr; external name '_IconMethodToRgn';
{
 *  SetSuiteLabel()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function SetSuiteLabel(theSuite: IconSuiteRef; theLabel: SInt16): OSErr; external name '_SetSuiteLabel';
{
 *  GetSuiteLabel()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetSuiteLabel(theSuite: IconSuiteRef): SInt16; external name '_GetSuiteLabel';
{
 *  GetIconCacheData()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconCacheData(theCache: IconCacheRef; var theData: UnivPtr): OSErr; external name '_GetIconCacheData';
{
 *  SetIconCacheData()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function SetIconCacheData(theCache: IconCacheRef; theData: UnivPtr): OSErr; external name '_SetIconCacheData';
{
 *  GetIconCacheProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconCacheProc(theCache: IconCacheRef; var theProc: IconGetterUPP): OSErr; external name '_GetIconCacheProc';
{
 *  SetIconCacheProc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function SetIconCacheProc(theCache: IconCacheRef; theProc: IconGetterUPP): OSErr; external name '_SetIconCacheProc';
{
 *  PlotIconHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PlotIconHandle(const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIcon: Handle): OSErr; external name '_PlotIconHandle';
{
 *  PlotSICNHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PlotSICNHandle(const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theSICN: Handle): OSErr; external name '_PlotSICNHandle';
{
 *  PlotCIconHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PlotCIconHandle(const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theCIcon: CIconHandle): OSErr; external name '_PlotCIconHandle';
{
   IconServices is an efficient mechanism to share icon data amongst multiple 
   clients. It avoids duplication of data; it provides efficient caching, 
   releasing memory when the icon data is no longer needed; it can provide
   the appropriate icon for any filesystem object; it can provide commonly 
   used icons (caution, note, help...); it is Appearance-savvy: the icons
   are switched when appropriate.
   IconServices refer to cached icon data using IconRef, a 32-bit opaque
   value. IconRefs are reference counted. When there are no more "owners" 
   of an IconRef, the memory used by the icon bitmap is disposed of.
   Two files of same type and creator with no custom icon will have the same IconRef.
   Files with custom icons will have their own IconRef.
}

{
   Use the special creator kSystemIconsCreator to get "standard" icons 
   that are not associated with a file, such as the help icon.
   Note that all lowercase creators are reserved by Apple.
}

const
	kSystemIconsCreator			= FourCharCode('macs');


	{
	   Type of the predefined/generic icons. For example, the call:
	      err = GetIconRef(kOnSystemDisk, kSystemIconsCreator, kHelpIcon, &iconRef);
	   will retun in iconRef the IconRef for the standard help icon.
	}

	{	 Generic Finder icons 	}
	kClipboardIcon				= FourCharCode('CLIP');
	kClippingUnknownTypeIcon	= FourCharCode('clpu');
	kClippingPictureTypeIcon	= FourCharCode('clpp');
	kClippingTextTypeIcon		= FourCharCode('clpt');
	kClippingSoundTypeIcon		= FourCharCode('clps');
	kDesktopIcon				= FourCharCode('desk');
	kFinderIcon					= FourCharCode('FNDR');
	kComputerIcon					= FourCharCode('root');
	kFontSuitcaseIcon			= FourCharCode('FFIL');
	kFullTrashIcon				= FourCharCode('ftrh');
	kGenericApplicationIcon		= FourCharCode('APPL');
	kGenericCDROMIcon			= FourCharCode('cddr');
	kGenericControlPanelIcon	= FourCharCode('APPC');
	kGenericControlStripModuleIcon = FourCharCode('sdev');
	kGenericComponentIcon		= FourCharCode('thng');
	kGenericDeskAccessoryIcon	= FourCharCode('APPD');
	kGenericDocumentIcon		= FourCharCode('docu');
	kGenericEditionFileIcon		= FourCharCode('edtf');
	kGenericExtensionIcon		= FourCharCode('INIT');
	kGenericFileServerIcon		= FourCharCode('srvr');
	kGenericFontIcon			= FourCharCode('ffil');
	kGenericFontScalerIcon		= FourCharCode('sclr');
	kGenericFloppyIcon			= FourCharCode('flpy');
	kGenericHardDiskIcon		= FourCharCode('hdsk');
	kGenericIDiskIcon			= FourCharCode('idsk');
	kGenericRemovableMediaIcon	= FourCharCode('rmov');
	kGenericMoverObjectIcon		= FourCharCode('movr');
	kGenericPCCardIcon			= FourCharCode('pcmc');
	kGenericPreferencesIcon		= FourCharCode('pref');
	kGenericQueryDocumentIcon	= FourCharCode('qery');
	kGenericRAMDiskIcon			= FourCharCode('ramd');
	kGenericSharedLibaryIcon	= FourCharCode('shlb');
	kGenericStationeryIcon		= FourCharCode('sdoc');
	kGenericSuitcaseIcon		= FourCharCode('suit');
	kGenericURLIcon				= FourCharCode('gurl');
	kGenericWORMIcon			= FourCharCode('worm');
	kInternationalResourcesIcon	= FourCharCode('ifil');
	kKeyboardLayoutIcon			= FourCharCode('kfil');
	kSoundFileIcon				= FourCharCode('sfil');
	kSystemSuitcaseIcon			= FourCharCode('zsys');
	kTrashIcon					= FourCharCode('trsh');
	kTrueTypeFontIcon			= FourCharCode('tfil');
	kTrueTypeFlatFontIcon		= FourCharCode('sfnt');
	kTrueTypeMultiFlatFontIcon	= FourCharCode('ttcf');
	kUserIDiskIcon				= FourCharCode('udsk');
	kUnknownFSObjectIcon				= FourCharCode('unfs');
	kInternationResourcesIcon	= FourCharCode('ifil');						{  old misspelling }

	{	 Internet locations 	}
	kInternetLocationHTTPIcon	= FourCharCode('ilht');
	kInternetLocationFTPIcon	= FourCharCode('ilft');
	kInternetLocationAppleShareIcon = FourCharCode('ilaf');
	kInternetLocationAppleTalkZoneIcon = FourCharCode('ilat');
	kInternetLocationFileIcon	= FourCharCode('ilfi');
	kInternetLocationMailIcon	= FourCharCode('ilma');
	kInternetLocationNewsIcon	= FourCharCode('ilnw');
	kInternetLocationNSLNeighborhoodIcon = FourCharCode('ilns');
	kInternetLocationGenericIcon = FourCharCode('ilge');

	{	 Folders 	}
	kGenericFolderIcon			= FourCharCode('fldr');
	kDropFolderIcon				= FourCharCode('dbox');
	kMountedFolderIcon			= FourCharCode('mntd');
	kOpenFolderIcon				= FourCharCode('ofld');
	kOwnedFolderIcon			= FourCharCode('ownd');
	kPrivateFolderIcon			= FourCharCode('prvf');
	kSharedFolderIcon			= FourCharCode('shfl');

	{	 Sharing Privileges icons 	}
	kSharingPrivsNotApplicableIcon = FourCharCode('shna');
	kSharingPrivsReadOnlyIcon	= FourCharCode('shro');
	kSharingPrivsReadWriteIcon	= FourCharCode('shrw');
	kSharingPrivsUnknownIcon	= FourCharCode('shuk');
	kSharingPrivsWritableIcon	= FourCharCode('writ');


	{	 Users and Groups icons 	}
	kUserFolderIcon				= FourCharCode('ufld');
	kWorkgroupFolderIcon		= FourCharCode('wfld');
	kGuestUserIcon				= FourCharCode('gusr');
	kUserIcon					= FourCharCode('user');
	kOwnerIcon					= FourCharCode('susr');
	kGroupIcon					= FourCharCode('grup');

	{	 Special folders 	}
	kAppearanceFolderIcon		= FourCharCode('appr');
	kAppleExtrasFolderIcon		= FourCharCode('aex�');
	kAppleMenuFolderIcon		= FourCharCode('amnu');
	kApplicationsFolderIcon		= FourCharCode('apps');
	kApplicationSupportFolderIcon = FourCharCode('asup');
	kAssistantsFolderIcon		= FourCharCode('ast�');
	kColorSyncFolderIcon		= FourCharCode('prof');
	kContextualMenuItemsFolderIcon = FourCharCode('cmnu');
	kControlPanelDisabledFolderIcon = FourCharCode('ctrD');
	kControlPanelFolderIcon		= FourCharCode('ctrl');
	kControlStripModulesFolderIcon = FourCharCode('sdv�');
	kDocumentsFolderIcon		= FourCharCode('docs');
	kExtensionsDisabledFolderIcon = FourCharCode('extD');
	kExtensionsFolderIcon		= FourCharCode('extn');
	kFavoritesFolderIcon		= FourCharCode('favs');
	kFontsFolderIcon			= FourCharCode('font');
	kHelpFolderIcon				= FourCharCode('�hlp');
	kInternetFolderIcon			= FourCharCode('int�');
	kInternetPlugInFolderIcon	= FourCharCode('�net');
	kInternetSearchSitesFolderIcon = FourCharCode('issf');
	kLocalesFolderIcon			= FourCharCode('�loc');
	kMacOSReadMeFolderIcon		= FourCharCode('mor�');
	kPublicFolderIcon			= FourCharCode('pubf');
	kPreferencesFolderIcon		= FourCharCode('prf�');
	kPrinterDescriptionFolderIcon = FourCharCode('ppdf');
	kPrinterDriverFolderIcon	= FourCharCode('�prd');
	kPrintMonitorFolderIcon		= FourCharCode('prnt');
	kRecentApplicationsFolderIcon = FourCharCode('rapp');
	kRecentDocumentsFolderIcon	= FourCharCode('rdoc');
	kRecentServersFolderIcon	= FourCharCode('rsrv');
	kScriptingAdditionsFolderIcon = FourCharCode('�scr');
	kSharedLibrariesFolderIcon	= FourCharCode('�lib');
	kScriptsFolderIcon			= FourCharCode('scr�');
	kShutdownItemsDisabledFolderIcon = FourCharCode('shdD');
	kShutdownItemsFolderIcon	= FourCharCode('shdf');
	kSpeakableItemsFolder		= FourCharCode('spki');
	kStartupItemsDisabledFolderIcon = FourCharCode('strD');
	kStartupItemsFolderIcon		= FourCharCode('strt');
	kSystemExtensionDisabledFolderIcon = FourCharCode('macD');
	kSystemFolderIcon			= FourCharCode('macs');
	kTextEncodingsFolderIcon	= FourCharCode('�tex');
	kUsersFolderIcon			= FourCharCode('usr�');
	kUtilitiesFolderIcon		= FourCharCode('uti�');
	kVoicesFolderIcon			= FourCharCode('fvoc');
	kSystemFolderXIcon			= FourCharCode('macx');

	{	 Badges 	}
	kAppleScriptBadgeIcon		= FourCharCode('scrp');
	kLockedBadgeIcon			= FourCharCode('lbdg');
	kMountedBadgeIcon			= FourCharCode('mbdg');
	kSharedBadgeIcon			= FourCharCode('sbdg');
	kAliasBadgeIcon				= FourCharCode('abdg');
	kAlertCautionBadgeIcon		= FourCharCode('cbdg');

	{	 Alert icons 	}
	kAlertNoteIcon				= FourCharCode('note');
	kAlertCautionIcon			= FourCharCode('caut');
	kAlertStopIcon				= FourCharCode('stop');

	{	 Networking icons 	}
	kAppleTalkIcon				= FourCharCode('atlk');
	kAppleTalkZoneIcon			= FourCharCode('atzn');
	kAFPServerIcon				= FourCharCode('afps');
	kFTPServerIcon				= FourCharCode('ftps');
	kHTTPServerIcon				= FourCharCode('htps');
	kGenericNetworkIcon			= FourCharCode('gnet');
	kIPFileServerIcon			= FourCharCode('isrv');

	{	 Toolbar icons 	}
	kToolbarCustomizeIcon		= FourCharCode('tcus');
	kToolbarDeleteIcon			= FourCharCode('tdel');
	kToolbarFavoritesIcon		= FourCharCode('tfav');
	kToolbarHomeIcon			= FourCharCode('thom');

	{	 Other icons 	}
	kAppleLogoIcon				= FourCharCode('capl');
	kAppleMenuIcon				= FourCharCode('sapl');
	kBackwardArrowIcon			= FourCharCode('baro');
	kFavoriteItemsIcon			= FourCharCode('favr');
	kForwardArrowIcon			= FourCharCode('faro');
	kGridIcon					= FourCharCode('grid');
	kHelpIcon					= FourCharCode('help');
	kKeepArrangedIcon			= FourCharCode('arng');
	kLockedIcon					= FourCharCode('lock');
	kNoFilesIcon				= FourCharCode('nfil');
	kNoFolderIcon				= FourCharCode('nfld');
	kNoWriteIcon				= FourCharCode('nwrt');
	kProtectedApplicationFolderIcon = FourCharCode('papp');
	kProtectedSystemFolderIcon	= FourCharCode('psys');
	kRecentItemsIcon			= FourCharCode('rcnt');
	kShortcutIcon				= FourCharCode('shrt');
	kSortAscendingIcon			= FourCharCode('asnd');
	kSortDescendingIcon			= FourCharCode('dsnd');
	kUnlockedIcon				= FourCharCode('ulck');
	kConnectToIcon				= FourCharCode('cnct');
	kGenericWindowIcon			= FourCharCode('gwin');
	kQuestionMarkIcon			= FourCharCode('ques');
	kDeleteAliasIcon			= FourCharCode('dali');
	kEjectMediaIcon				= FourCharCode('ejec');
	kBurningIcon				= FourCharCode('burn');
	kRightContainerArrowIcon	= FourCharCode('rcar');


	{	  IconServicesUsageFlags 	}

type
	IconServicesUsageFlags				= UInt32;

const
	kIconServicesNormalUsageFlag    = $00000000;
	kIconServicesNoBadgeFlag        = $00000001;     // available on Panther and later
	kIconServicesUpdateIfNeededFlag = $00000002;     // available on Panther and later


	{	
	  kIconServicesCatalogInfoMask - Minimal bitmask for use with
	    GetIconRefFromFileInfo(). Use this mask with FSGetCatalogInfo
	    before calling GetIconRefFromFileInfo().
		}
	kIconServicesCatalogInfoMask = $0008181E;


	{
	 *  PlotIconRefFlags
	 *  
	 *  Discussion:
	 *    Flags that can be passed to the PlotIconRefInContext routine.
	 	}

type
	PlotIconRefFlags 			= UInt32;
const
	kPlotIconRefNormalFlags		= 0;
	kPlotIconRefNoImage			= $02;
	kPlotIconRefNoMask			= $04;


	{
	    IconFamily 'icns' resources contain an entire IconFamily (all sizes and depths).  
	   For custom icons, icns IconFamily resources of the custom icon resource ID are fetched first before
	   the classic custom icons (individual 'ics#, ICN#, etc) are fetched.  If the fetch of the icns resource
	   succeeds then the icns is looked at exclusively for the icon data.
	   For custom icons, new icon features such as 32-bit deep icons are only fetched from the icns resource.
	   This is to avoid incompatibilities with cut & paste of new style icons with an older version of the
	   MacOS Finder.
	   DriverGestalt is called with code kdgMediaIconSuite by IconServices after calling FSM to determine a
	   driver icon for a particular device.  The result of the kdgMediaIconSuite call to DriverGestalt should
	   be a pointer an an IconFamily.  In this manner driver vendors can provide rich, detailed drive icons
	   instead of the 1-bit variety previously supported.
	}
	kIconFamilyType				= FourCharCode('icns');


type
	IconFamilyElementPtr = ^IconFamilyElement;
	IconFamilyElement = record
		elementType:			OSType;									{  'ICN#', 'icl8', etc... }
		elementSize:			Size;									{  Size of this element }
		elementData:			SInt8;
	end;

	IconFamilyResourcePtr = ^IconFamilyResource;
	IconFamilyResource = record
		resourceType:			OSType;									{  Always 'icns' }
		resourceSize:			Size;									{  Total size of this resource }
		elements:				array [0..0] of IconFamilyElement;
	end;

	IconFamilyPtr						= ^IconFamilyResource;
	IconFamilyHandle					= ^IconFamilyPtr;
	{
	  ==============================================================================
	   Initialization and Termination
	  ==============================================================================
	}

	{
	   IconServicesInit
	   
	   Call this routine once per classic 68K application initialization.
	   This routine does not need to be called at boot time.
	}

{$ifc CALL_NOT_IN_CARBON}
	{
	 *  IconServicesInit()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
	 *    CarbonLib:        not available
	 *    Mac OS X:         not available
	 	}
function IconServicesInit(initBlockPtr: CFragInitBlockPtr): OSErr; external name '_IconServicesInit';
{
   IconServicesTerminate:
   
   Call this routine once from the termination of a classic 68K application.
   This routine does not need to be called at boot time.
}

{
 *  IconServicesTerminate()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure IconServicesTerminate; external name '_IconServicesTerminate';
{$endc}  {CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2

{
  ==============================================================================
   Converting data structures
  ==============================================================================
}


{
   IconRefToIconFamily
   This routines returns a new IconFamily that contains the data corresponding
   to the specified IconRef.
}

{
 *  IconRefToIconFamily()
<<<<<<< HEAD
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetSuiteLabel( theSuite: IconSuiteRef ): SInt16; external name '_GetSuiteLabel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetIconCacheData()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IconRefToIconFamily( theIconRef: IconRef; whichIcons: IconSelectorValue; var iconFamily: IconFamilyHandle ): OSErr; external name '_IconRefToIconFamily';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IconRefToIconFamily(theIconRef: IconRef; whichIcons: IconSelectorValue; var iconFamily: IconFamilyHandle): OSErr; external name '_IconRefToIconFamily';
>>>>>>> graemeg/fixes_2_2
{
   IconFamilyToIconSuite
   This routine transfers the data from an icon family handle into an icon suite.
}

<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  IconFamilyToIconSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IconFamilyToIconSuite( iconFamily: IconFamilyHandle; whichIcons: IconSelectorValue; var iconSuite: IconSuiteRef ): OSErr; external name '_IconFamilyToIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


=======
{
 *  IconFamilyToIconSuite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IconFamilyToIconSuite(iconFamily: IconFamilyHandle; whichIcons: IconSelectorValue; var iconSuite: IconSuiteRef): OSErr; external name '_IconFamilyToIconSuite';
>>>>>>> graemeg/fixes_2_2
{
   IconSuiteToIconFamily
   This routine transfers the data in an icon suite into an icon family.
}

{
<<<<<<< HEAD
 *  IconSuiteToIconFamily()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IconSuiteToIconFamily( iconSuite: IconSuiteRef; whichIcons: IconSelectorValue; var iconFamily: IconFamilyHandle ): OSErr; external name '_IconSuiteToIconFamily';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


=======
 *  IconSuiteToIconFamily()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IconSuiteToIconFamily(iconSuite: IconSuiteRef; whichIcons: IconSelectorValue; var iconFamily: IconFamilyHandle): OSErr; external name '_IconSuiteToIconFamily';
>>>>>>> graemeg/fixes_2_2
{
   SetIconFamilyData
   Change the data of an icon family. The data is copied.
   The type can be one of the icon type, or 'PICT'.
   The data will be compressed if needed.
}

<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{
 *  SetIconFamilyData()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetIconCacheData( theCache: IconCacheRef; var theData: UnivPtr ): OSErr; external name '_GetIconCacheData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function SetIconFamilyData( iconFamily: IconFamilyHandle; iconType: OSType; h: Handle ): OSErr; external name '_SetIconFamilyData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
{
 *  SetIconFamilyData()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function SetIconFamilyData(iconFamily: IconFamilyHandle; iconType: OSType; h: Handle): OSErr; external name '_SetIconFamilyData';
>>>>>>> graemeg/fixes_2_2
{
   GetIconFamilyData
   Return a copy of the data in the icon family.
   The type can be one of the icon type, or 'PICT'
   The data will be returned uncompressed.
   The handle (h) will be resized as appropriate. If no data of the 
   requested type is present, the handle size will be set to 0.
}
<<<<<<< HEAD
>>>>>>> origin/cpstrnew

{
 *  SetIconCacheData()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetIconCacheData( theCache: IconCacheRef; theData: UnivPtr ): OSErr; external name '_SetIconCacheData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function GetIconFamilyData( iconFamily: IconFamilyHandle; iconType: OSType; h: Handle ): OSErr; external name '_GetIconFamilyData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
   PlotIconRef
   
   This routine plots the IconRef.  It mostly takes the same parameters as 
   PlotIconSuite. Pass kIconServicesNormalUsageFlag as a default value for 
   IconServicesUsageFlags.
}
>>>>>>> origin/cpstrnew

{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  GetIconCacheProc()   *** DEPRECATED ***
=======
 *  PlotIconRef()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetIconCacheProc( theCache: IconCacheRef; var theProc: IconGetterUPP ): OSErr; external name '_GetIconCacheProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetIconCacheProc()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function SetIconCacheProc( theCache: IconCacheRef; theProc: IconGetterUPP ): OSErr; external name '_SetIconCacheProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

=======
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function PlotIconRef( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): OSErr; external name '_PlotIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ PlotIconRefInContext}
{$endc} {not TARGET_CPU_64}

{
 *  PlotIconRefInContext()
 *  
 *  Discussion:
 *    This routines plots an IconRef using Quartz/CoreGraphics.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    inContext:
 *      The graphics context to use.
 *    
 *    inRect:
 *      The rect to plot the icon in.
 *    
 *    inAlign:
 *      The icon alignment.
 *    
 *    inTransform:
 *      The icon transform.
 *    
 *    inLabelColor:
 *      The icon label color.
 *    
 *    inFlags:
 *      The drawing flags to use (usually kPlotIconRefNormalFlags).
 *    
 *    inIconRef:
 *      The IconRef to plot.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function PlotIconRefInContext( inContext: CGContextRef; const (*var*) inRect: CGRect; inAlign: IconAlignmentType; inTransform: IconTransformType; inLabelColor: RGBColorPtr; inFlags: PlotIconRefFlags; inIconRef: IconRef ): OSStatus; external name '_PlotIconRefInContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
   PtInIconRef
   
   This routine indicates if testPt would hit the icon designated by iconRef.
   It mostly takes the same parameters as PtInIconSuite.
   Pass kIconServicesNormalUsageFlag as a default value for IconServicesUsageFlags.
   This call is deprecated. Please use IconRefContainsCGPoint() instead.
}
>>>>>>> origin/cpstrnew


{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  PlotIconHandle()   *** DEPRECATED ***
 *  
=======
 *  PtInIconRef()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function PtInIconRef( const (*var*) testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): Boolean; external name '_PtInIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
   RectInIconRef
   
   This routine indicates if testRect would intersect the icon designated by iconRef.
   It mostly takes the same parameters as RectInIconSuite.
   Pass kIconServicesNormalUsageFlag as a default value for IconServicesUsageFlags.
   This call is deprecated. Please use IconRefIntersectsCGRect() instead.
}


{
 *  RectInIconRef()   *** DEPRECATED ***
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotIconHandle( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIcon: Handle ): OSErr; external name '_PlotIconHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotSICNHandle()   *** DEPRECATED ***
=======
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function RectInIconRef( const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): Boolean; external name '_RectInIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
   IconRefToRgn
   
   This routine returns a region for the icon.
   It mostly takes the same parameters as IconSuiteToRgn.
   Pass kIconServicesNormalUsageFlag as a default value for IconServicesUsageFlags.
   This call is deprecated. Please use IconRefToHIShape() instead.
}

{
 *  IconRefToRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IconRefToRgn( theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): OSErr; external name '_IconRefToRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{
 *  IconRefContainsCGPoint()
 *  
 *  Discussion:
 *    Tests whether a specified point falls within an icon�s mask.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    testPt:
 *      A pointer to the location that IconServices tests to see
 *      whether is falls within the mask of indicated icon.
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses to determine which icon is hit-tested. Use the same CGRect
 *      value as when the icon was last drawn.
 *    
 *    align:
 *      A value that specifies how the indicated icon is aligned within
 *      the rectangle specified in the iconRect parameter. Use the same
 *      IconAlignmentType value as when the icon was last drawn.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
 *    theIconRef:
 *      The icon to be tested.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function IconRefContainsCGPoint( const (*var*) testPt: CGPoint; const (*var*) iconRect: CGRect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): Boolean; external name '_IconRefContainsCGPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  IconRefIntersectsCGRect()
 *  
 *  Discussion:
 *    Tests whether a specified rectangle falls within an icon�s mask.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotSICNHandle( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theSICN: Handle ): OSErr; external name '_PlotSICNHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PlotCIconHandle()   *** DEPRECATED ***
=======
 *  Parameters:
 *    
 *    testRect:
 *      A pointer to the rectangle that IconServices tests to see
 *      whether is falls within the mask of indicated icon.
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses to determine which icon is hit-tested. Use the same CGRect
 *      value as when the icon was last drawn.
 *    
 *    align:
 *      A value that specifies how the indicated icon is aligned within
 *      the rectangle specified in the iconRect parameter. Use the same
 *      IconAlignmentType value as when the icon was last drawn.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
 *    theIconRef:
 *      The icon to be tested.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function IconRefIntersectsCGRect( const (*var*) testRect: CGRect; const (*var*) iconRect: CGRect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): Boolean; external name '_IconRefIntersectsCGRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  IconRefToHIShape()
 *  
 *  Discussion:
 *    Converts an IconRef-derived icon into an HIShape.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function PlotCIconHandle( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theCIcon: CIconHandle ): OSErr; external name '_PlotCIconHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}


{
 *  PlotIconRefFlags
 *  
 *  Discussion:
 *    Flags that can be passed to the PlotIconRefInContext routine.
 }
type
	PlotIconRefFlags = UInt32;
const
{
   * Draw the icon image and alpha channel.
   }
	kPlotIconRefNormalFlags = 0;

  {
   * Only draw the alpha channel.
   }
	kPlotIconRefNoImage = 1 shl 1;

  {
   * Only draw the image.
   }
	kPlotIconRefNoMask = 1 shl 2;


{
  ==============================================================================
   Converting data structures
  ==============================================================================
}


{
   IconRefToIconFamily
   This routines returns a new IconFamily that contains the data corresponding
   to the specified IconRef.
}

{
 *  IconRefToIconFamily()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IconRefToIconFamily( theIconRef: IconRef; whichIcons: IconSelectorValue; var iconFamily: IconFamilyHandle ): OSErr; external name '_IconRefToIconFamily';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
   IconFamilyToIconSuite
   This routine transfers the data from an icon family handle into an icon suite.
}

{$ifc not TARGET_CPU_64}
{
 *  IconFamilyToIconSuite()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IconFamilyToIconSuite( iconFamily: IconFamilyHandle; whichIcons: IconSelectorValue; var iconSuite: IconSuiteRef ): OSErr; external name '_IconFamilyToIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
   IconSuiteToIconFamily
   This routine transfers the data in an icon suite into an icon family.
}

{
 *  IconSuiteToIconFamily()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IconSuiteToIconFamily( iconSuite: IconSuiteRef; whichIcons: IconSelectorValue; var iconFamily: IconFamilyHandle ): OSErr; external name '_IconSuiteToIconFamily';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
   SetIconFamilyData
   Change the data of an icon family. The data is copied.
   The type can be one of the icon type, or 'PICT'.
   The data will be compressed if needed.
}

{$endc} {not TARGET_CPU_64}

{
 *  SetIconFamilyData()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function SetIconFamilyData( iconFamily: IconFamilyHandle; iconType: OSType; h: Handle ): OSErr; external name '_SetIconFamilyData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
   GetIconFamilyData
   Return a copy of the data in the icon family.
   The type can be one of the icon type, or 'PICT'
   The data will be returned uncompressed.
   The handle (h) will be resized as appropriate. If no data of the 
   requested type is present, the handle size will be set to 0.
}

{
 *  GetIconFamilyData()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function GetIconFamilyData( iconFamily: IconFamilyHandle; iconType: OSType; h: Handle ): OSErr; external name '_GetIconFamilyData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======

{
 *  GetIconFamilyData()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconFamilyData(iconFamily: IconFamilyHandle; iconType: OSType; h: Handle): OSErr; external name '_GetIconFamilyData';
{
  ==============================================================================
   Reference counting
  ==============================================================================
}


{
   GetIconRefOwners
   
   This routine returns the reference count for the IconRef, or number of owners.
   
   A valid IconRef always has at least one owner.
}

{
 *  GetIconRefOwners()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconRefOwners(theIconRef: IconRef; var owners: UInt16): OSErr; external name '_GetIconRefOwners';
{
   AcquireIconRef
   This routine increments the reference count for the IconRef
}

{
 *  AcquireIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AcquireIconRef(theIconRef: IconRef): OSErr; external name '_AcquireIconRef';
{
   ReleaseIconRef
   
   This routine decrements the reference count for the IconRef.
   
   When the reference count reaches 0, all memory allocated for the icon
   is disposed. Any subsequent use of the IconRef is invalid.
}

{
 *  ReleaseIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ReleaseIconRef(theIconRef: IconRef): OSErr; external name '_ReleaseIconRef';
{
  ==============================================================================
   Getting an IconRef
  ==============================================================================
}


{
   GetIconRefFromFile
   
   This routine returns an icon ref for the specified file, folder or volume.
   The label information is provided separately, since two files with the same icon 
   but a different label would share the same iconRef. The label can be used in 
   PlotIconRef() for example.
   
   Use this routine if you have no information about the file system object. If 
   you have already done a GetCatInfo on the file and want to save some I/O, 
   call GetIconRefFromFolder() if you know it's a folder with no custom icon or 
   call GetIconRef() if it's a file with no custom icon.
   This routine increments the reference count of the returned IconRef. Call 
   ReleaseIconRef() when you're done with it.
}

{
 *  GetIconRefFromFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconRefFromFile(const (*var*) theFile: FSSpec; var theIconRef: IconRef; var theLabel: SInt16): OSErr; external name '_GetIconRefFromFile';
{
   GetIconRef
   
   This routine returns an icon ref for an icon in the desktop database or
   for a registered icon.
   The system registers a set of icon such as the help icon with the creator 
   code kSystemIconsCreator. See above for a list of the registered system types.
   The vRefNum is used as a hint on where to look for the icon first. Use 
   kOnSystemDisk if you don't know what to pass.
   This routine increments the reference count of the returned IconRef. Call 
   ReleaseIconRef() when you're done with it.
}

{
 *  GetIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconRef(vRefNum: SInt16; creator: OSType; iconType: OSType; var theIconRef: IconRef): OSErr; external name '_GetIconRef';
{
   GetIconRefFromFolder
   
   This routine returns an icon ref for a folder with no custom icon.
   Use the more generic, but slightly slower, GetIconRefFromFile() if
   you don't already have the necessary info about the file.
   Attributes should be CInfoPBRec.dirInfo.ioFlAttrib for this folder.
   Access privileges should be CInfoPBRec.dirInfo.ioACUser for this folder.
   This routine increments the reference count of the IconRef. Call ReleaseIconRef() 
   when you're done with it.
}

{
 *  GetIconRefFromFolder()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconRefFromFolder(vRefNum: SInt16; parentFolderID: SInt32; folderID: SInt32; attributes: SInt8; accessPrivileges: SInt8; var theIconRef: IconRef): OSErr; external name '_GetIconRefFromFolder';
{  GetIconRefFromFileInfo }
{
 *  GetIconRefFromFileInfo()
 *  
 *  Summary:
 *    This routine returns an IconRef for a file with minimal file I/O.
 *  
 *  Discussion:
 *    To minimize file operations, FSGetCatalogInfo should be called
 *    prior to calling this routine. The FSCatalogInfo should
 *    correspond to kIconServicesCatalogInfoMask The name should be
 *    fetched and passed in. If either the name or the correct catalog
 *    info is not passed in, this routine will do file operations for
 *    this information instead.
 *  
 *  Parameters:
 *    
 *    inRef:
 *      An FSRef for the target file
 *    
 *    inFileNameLength:
 *      The length of the name of the target file
 *    
 *    inFileName:
 *      The name of the target file
 *    
 *    inWhichInfo:
 *      The mask of file info already acquired.
 *    
 *    inCatalogInfo:
 *      The catalog info already acquired.
 *    
 *    inUsageFlags:
 *      The usage flags for this call (use
 *      kIconServicesNormalUsageFlag).
 *    
 *    outIconRef:
 *      The output IconRef for the routine.
 *    
 *    outLabel:
 *      The output label for the icon/file.
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Mac OS X:         in version 10.1 and later
 }
function GetIconRefFromFileInfo(const (*var*) inRef: FSRef; inFileNameLength: UniCharCount; inFileName: UniCharPtr; inWhichInfo: FSCatalogInfoBitmap; inCatalogInfo: {Const}FSCatalogInfoPtr; inUsageFlags: IconServicesUsageFlags; var outIconRef: IconRef; var outLabel: SInt16): OSStatus; external name '_GetIconRefFromFileInfo';

{ GetIconRefFromTypeInfo}
{
 *  GetIconRefFromTypeInfo()
 *  
 *  Summary:
 *    Create an IconRef for a type information.
 *  
 *  Discussion:
 *    Creates IconRef based on provided type info. Any of the input
 *    parameters can be zero (meaning it is unknown). Returns generic
 *    document icon in case if all parameters are zeroes. Calling the
 *    routine with non zero inCreator and inType and zero inExtension
 *    and inMIMEType is equivalent to GetIconRef(kOnSystemDisk,
 *    inCreator, inType).
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inCreator:
 *      The creator.
 *    
 *    inType:
 *      The type.
 *    
 *    inExtension:
 *      The extension.
 *    
 *    inMIMEType:
 *      The MIME type.
 *    
 *    inUsageFlags:
 *      The usage flags for this call (use
 *      kIconServicesNormalUsageFlag).
 *    
 *    outIconRef:
 *      The output IconRef for the routine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function GetIconRefFromTypeInfo( inCreator: OSType; inType: OSType; inExtension: CFStringRef; inMIMEType: CFStringRef; inUsageFlags: IconServicesUsageFlags; var outIconRef: IconRef ): OSErr; external name '_GetIconRefFromTypeInfo';

{ GetIconRefFromIconFamilyPtr}
{
 *  GetIconRefFromIconFamilyPtr()
 *  
 *  Summary:
 *    Create an IconRef for the IconFamilyPtr.
 *  
 *  Discussion:
 *    This routine creates IconRef for the IconFamilyPtr.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inIconFamilyPtr:
 *      The icon data
 *    
 *    inSize:
 *      The icon data size
 *    
 *    outIconRef:
 *      The output IconRef for the routine.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function GetIconRefFromIconFamilyPtr( const (*var*) inIconFamilyPtr: IconFamilyResource; inSize: Size; var outIconRef: IconRef ): OSStatus; external name '_GetIconRefFromIconFamilyPtr';

{
  ==============================================================================
   Adding and modifying IconRef
  ==============================================================================
}


{
   RegisterIconRefFromIconFamily
   This routine adds a new entry to the IconRef registry. Other clients will be 
   able to access it using the (creator, iconType) pair specified here.
   Lower-case creators are reserved for the system.
   Consider using RegisterIconRefFromResource() if possible, since the data 
   registered using RegisterIconRefFromFamily() cannot be purged.
   The iconFamily data is copied and the caller is reponsible for disposing of it.
   This routine increments the reference count of the IconRef. Call ReleaseIconRef() 
   when you're done with it.
}

{
 *  RegisterIconRefFromIconFamily()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function RegisterIconRefFromIconFamily(creator: OSType; iconType: OSType; iconFamily: IconFamilyHandle; var theIconRef: IconRef): OSErr; external name '_RegisterIconRefFromIconFamily';
{
   RegisterIconRefFromResource
   
   Registers an IconRef from a resouce file.  
   Lower-case creators are reserved for the system.
   The icon data to be fetched is either classic icon data or an icon family.  
   The 'icns' icon family is searched for before the classic icon data.
   This routine increments the reference count of the IconRef. Call ReleaseIconRef() 
   when you're done with it.
}

{
 *  RegisterIconRefFromResource()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function RegisterIconRefFromResource(creator: OSType; iconType: OSType; const (*var*) resourceFile: FSSpec; resourceID: SInt16; var theIconRef: IconRef): OSErr; external name '_RegisterIconRefFromResource';
{  RegisterIconRefFromFSRef }
{
 *  RegisterIconRefFromFSRef()
 *  
 *  Discussion:
 *    This routine registers an IconRef from a ".icns" file and
 *    associates it with a creator/type pair.
 *  
 *  Parameters:
 *    
 *    creator:
 *      The creator code for the icns file.
 *    
 *    iconType:
 *      The type code for the icns file
 *    
 *    iconFile:
 *      The FSRef of the icns file.
 *    
 *    theIconRef:
 *      The output IconRef for the routine.
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Mac OS X:         in version 10.1 and later
 }
function RegisterIconRefFromFSRef(creator: OSType; iconType: OSType; const (*var*) iconFile: FSRef; var theIconRef: IconRef): OSStatus; external name '_RegisterIconRefFromFSRef';

{
   UnregisterIconRef
   
   Removes the specified icon from the icon cache (if there are no users of it).  
   If some clients are using this iconRef, then the IconRef will be removed when the 
   last user calls ReleaseIconRef.
}

{
 *  UnregisterIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function UnregisterIconRef(creator: OSType; iconType: OSType): OSErr; external name '_UnregisterIconRef';
{
   UpdateIconRef
   
   Call this routine to force an update of the data for iconRef.
   
   For example after changing an icon in the desktop database or changing the custom 
   icon of a file. Note that after _adding_ a custom icon to file or folder, you 
   need to call GetIconRefFromFile() to get a new IconRef specific to this file. 
   
   This routine does nothing if the IconRef is a registered icon.
}

{
 *  UpdateIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function UpdateIconRef(theIconRef: IconRef): OSErr; external name '_UpdateIconRef';
{
   OverrideIconRefFromResource
   
   This routines replaces the bitmaps of the specified IconRef with the ones
   in the specified resource file.
}

{
 *  OverrideIconRefFromResource()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function OverrideIconRefFromResource(theIconRef: IconRef; const (*var*) resourceFile: FSSpec; resourceID: SInt16): OSErr; external name '_OverrideIconRefFromResource';
{
   OverrideIconRef
   
   This routines replaces the bitmaps of the specified IconRef with the ones
   from the new IconRef.
}

{
 *  OverrideIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function OverrideIconRef(oldIconRef: IconRef; newIconRef: IconRef): OSErr; external name '_OverrideIconRef';
{
   RemoveIconRefOverride
   This routine remove an override if one was applied to the icon and 
   reverts back to the original bitmap data.
}

{
 *  RemoveIconRefOverride()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function RemoveIconRefOverride(theIconRef: IconRef): OSErr; external name '_RemoveIconRefOverride';
{
  ==============================================================================
   Creating composite IconRef
  ==============================================================================
}


{
   CompositeIconRef
   
   Superimposes an IconRef on top of another one
}

{
 *  CompositeIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function CompositeIconRef(backgroundIconRef: IconRef; foregroundIconRef: IconRef; var compositeIconRef: IconRef): OSErr; external name '_CompositeIconRef';
{
   IsIconRefComposite
   Indicates if a given icon ref is a composite of two other icon refs (and which ones)
   If it isn't a composite, backgroundIconRef and foreGroundIconRef will be 0.
}

{
 *  IsIconRefComposite()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IsIconRefComposite(compositeIconRef: IconRef; var backgroundIconRef: IconRef; var foregroundIconRef: IconRef): OSErr; external name '_IsIconRefComposite';
{
  ==============================================================================
   Using IconRef
  ==============================================================================
}

{
   IsValidIconRef
   Return true if the iconRef passed in is a valid icon ref
}

{
 *  IsValidIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IsValidIconRef(theIconRef: IconRef): boolean; external name '_IsValidIconRef';
>>>>>>> graemeg/fixes_2_2
{
   PlotIconRef
   
   This routine plots the IconRef.  It mostly takes the same parameters as 
   PlotIconSuite. Pass kIconServicesNormalUsageFlag as a default value for 
   IconServicesUsageFlags.
}

<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  PlotIconRef()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function PlotIconRef( const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): OSErr; external name '_PlotIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ PlotIconRefInContext}
{$endc} {not TARGET_CPU_64}

=======
{
 *  PlotIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PlotIconRef(const (*var*) theRect: Rect; align: IconAlignmentType; transform: IconTransformType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef): OSErr; external name '_PlotIconRef';
{  PlotIconRefInContext }
>>>>>>> graemeg/fixes_2_2
{
 *  PlotIconRefInContext()
 *  
 *  Discussion:
 *    This routines plots an IconRef using Quartz/CoreGraphics.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
=======
>>>>>>> graemeg/fixes_2_2
 *  Parameters:
 *    
 *    inContext:
 *      The graphics context to use.
 *    
 *    inRect:
 *      The rect to plot the icon in.
 *    
 *    inAlign:
 *      The icon alignment.
 *    
 *    inTransform:
 *      The icon transform.
 *    
 *    inLabelColor:
 *      The icon label color.
 *    
 *    inFlags:
 *      The drawing flags to use (usually kPlotIconRefNormalFlags).
 *    
 *    inIconRef:
 *      The IconRef to plot.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function PlotIconRefInContext( inContext: CGContextRef; const (*var*) inRect: CGRect; inAlign: IconAlignmentType; inTransform: IconTransformType; inLabelColor: RGBColorPtr; inFlags: PlotIconRefFlags; inIconRef: IconRef ): OSStatus; external name '_PlotIconRefInContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
=======
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Mac OS X:         in version 10.1 and later
 }
function PlotIconRefInContext(inContext: CGContextRef; const (*var*) inRect: CGRect; inAlign: IconAlignmentType; inTransform: IconTransformType; inLabelColor: RGBColorPtr; inFlags: PlotIconRefFlags; inIconRef: IconRef): OSStatus; external name '_PlotIconRefInContext';
>>>>>>> graemeg/fixes_2_2


{
   PtInIconRef
   
   This routine indicates if testPt would hit the icon designated by iconRef.
   It mostly takes the same parameters as PtInIconSuite.
   Pass kIconServicesNormalUsageFlag as a default value for IconServicesUsageFlags.
<<<<<<< HEAD
   This call is deprecated. Please use IconRefContainsCGPoint() instead.
}


{$ifc not TARGET_CPU_64}
{
 *  PtInIconRef()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function PtInIconRef( const (*var*) testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): Boolean; external name '_PtInIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


=======
}


{
 *  PtInIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function PtInIconRef(const (*var*) testPt: Point; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef): boolean; external name '_PtInIconRef';
>>>>>>> graemeg/fixes_2_2
{
   RectInIconRef
   
   This routine indicates if testRect would intersect the icon designated by iconRef.
   It mostly takes the same parameters as RectInIconSuite.
   Pass kIconServicesNormalUsageFlag as a default value for IconServicesUsageFlags.
<<<<<<< HEAD
   This call is deprecated. Please use IconRefIntersectsCGRect() instead.
=======
>>>>>>> graemeg/fixes_2_2
}


{
<<<<<<< HEAD
 *  RectInIconRef()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function RectInIconRef( const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): Boolean; external name '_RectInIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


=======
 *  RectInIconRef()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function RectInIconRef(const (*var*) testRect: Rect; const (*var*) iconRect: Rect; align: IconAlignmentType; iconServicesUsageFlags_: IconServicesUsageFlags; theIconRef: IconRef): boolean; external name '_RectInIconRef';
>>>>>>> graemeg/fixes_2_2
{
   IconRefToRgn
   
   This routine returns a region for the icon.
   It mostly takes the same parameters as IconSuiteToRgn.
   Pass kIconServicesNormalUsageFlag as a default value for IconServicesUsageFlags.
<<<<<<< HEAD
   This call is deprecated. Please use IconRefToHIShape() instead.
}

{
 *  IconRefToRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IconRefToRgn( theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): OSErr; external name '_IconRefToRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}
=======
 *  Parameters:
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses as the bounding box of the region.
 *    
 *    align:
 *      The value which determines how Icon Services aligns the region
 *      within the rectangle.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
 *    theIconRef:
 *      The icon to be tested.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function IconRefToHIShape( const (*var*) iconRect: CGRect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): HIShapeRef; external name '_IconRefToHIShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


=======
}

{
 *  IconRefToRgn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IconRefToRgn(theRgn: RgnHandle; const (*var*) iconRect: Rect; align: IconAlignmentType; iconServicesUsageFlags_: IconServicesUsageFlags; theIconRef: IconRef): OSErr; external name '_IconRefToRgn';
>>>>>>> graemeg/fixes_2_2
{
   GetIconSizesFromIconRef
   
   This routine returns an IconSelectorValue indicating the depths and sizes of 
   icon data which are actually available.  It takes an IconSelectorValue 
   indicating which sizes/depths the caller is interested and returns an 
   IconSelectorValue indicating which sizes/depths exist.
   
   Caution:
   
   This is potentially an extremely expensive call as IconServices may be forced 
   to attempt fetching the data for the IconRef's sizes/depths which may result 
   in hitting the local disk or even the network to obtain the data to determine 
   which sizes/depths actually exist.
   Pass kIconServicesNormalUsageFlag as a default value for IconServicesUsageFlags.
   
   This call is deprecated. Please use IsDataAvailableInIconRef() instead.
}
<<<<<<< HEAD
>>>>>>> origin/cpstrnew

{$ifc not TARGET_CPU_64}
{
 *  IconRefContainsCGPoint()
 *  
 *  Discussion:
 *    Tests whether a specified point falls within an icon�s mask.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    testPt:
 *      A pointer to the location that IconServices tests to see
 *      whether is falls within the mask of indicated icon.
<<<<<<< HEAD
<<<<<<< HEAD
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses to determine which icon is hit-tested. Use the same CGRect
 *      value as when the icon was last drawn.
 *    
 *    align:
 *      A value that specifies how the indicated icon is aligned within
 *      the rectangle specified in the iconRect parameter. Use the same
 *      IconAlignmentType value as when the icon was last drawn.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
=======
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses to determine which icon is hit-tested. Use the same CGRect
 *      value as when the icon was last drawn.
 *    
 *    align:
 *      A value that specifies how the indicated icon is aligned within
 *      the rectangle specified in the iconRect parameter. Use the same
 *      IconAlignmentType value as when the icon was last drawn.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
>>>>>>> graemeg/cpstrnew
=======
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses to determine which icon is hit-tested. Use the same CGRect
 *      value as when the icon was last drawn.
 *    
 *    align:
 *      A value that specifies how the indicated icon is aligned within
 *      the rectangle specified in the iconRect parameter. Use the same
 *      IconAlignmentType value as when the icon was last drawn.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
>>>>>>> graemeg/cpstrnew
 *    theIconRef:
 *      The icon to be tested.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function IconRefContainsCGPoint( const (*var*) testPt: CGPoint; const (*var*) iconRect: CGRect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): Boolean; external name '_IconRefContainsCGPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  IconRefIntersectsCGRect()
 *  
 *  Discussion:
 *    Tests whether a specified rectangle falls within an icon�s mask.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    testRect:
 *      A pointer to the rectangle that IconServices tests to see
 *      whether is falls within the mask of indicated icon.
<<<<<<< HEAD
<<<<<<< HEAD
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses to determine which icon is hit-tested. Use the same CGRect
 *      value as when the icon was last drawn.
 *    
 *    align:
 *      A value that specifies how the indicated icon is aligned within
 *      the rectangle specified in the iconRect parameter. Use the same
 *      IconAlignmentType value as when the icon was last drawn.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
=======
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses to determine which icon is hit-tested. Use the same CGRect
 *      value as when the icon was last drawn.
 *    
 *    align:
 *      A value that specifies how the indicated icon is aligned within
 *      the rectangle specified in the iconRect parameter. Use the same
 *      IconAlignmentType value as when the icon was last drawn.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
>>>>>>> graemeg/cpstrnew
=======
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses to determine which icon is hit-tested. Use the same CGRect
 *      value as when the icon was last drawn.
 *    
 *    align:
 *      A value that specifies how the indicated icon is aligned within
 *      the rectangle specified in the iconRect parameter. Use the same
 *      IconAlignmentType value as when the icon was last drawn.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
>>>>>>> graemeg/cpstrnew
 *    theIconRef:
 *      The icon to be tested.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function IconRefIntersectsCGRect( const (*var*) testRect: CGRect; const (*var*) iconRect: CGRect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): Boolean; external name '_IconRefIntersectsCGRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  IconRefToHIShape()
 *  
 *  Discussion:
 *    Converts an IconRef-derived icon into an HIShape.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    iconRect:
 *      A pointer to the rectangle defining the area that Icon Services
 *      uses as the bounding box of the region.
<<<<<<< HEAD
<<<<<<< HEAD
 *    
 *    align:
 *      The value which determines how Icon Services aligns the region
 *      within the rectangle.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
=======
 *    
 *    align:
 *      The value which determines how Icon Services aligns the region
 *      within the rectangle.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
>>>>>>> graemeg/cpstrnew
=======
 *    
 *    align:
 *      The value which determines how Icon Services aligns the region
 *      within the rectangle.
 *    
 *    iconServicesUsageFlags:
 *      Reserved for future use. Pass the kIconServicesNormalUsageFlag
 *      constant in this parameter.
 *    
>>>>>>> graemeg/cpstrnew
 *    theIconRef:
 *      The icon to be tested.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function IconRefToHIShape( const (*var*) iconRect: CGRect; align: IconAlignmentType; theIconServicesUsageFlags: IconServicesUsageFlags; theIconRef: IconRef ): HIShapeRef; external name '_IconRefToHIShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function GetIconSizesFromIconRef( iconSelectorInput: IconSelectorValue; var iconSelectorOutputPtr: IconSelectorValue; iconServicesUsageFlags_: IconServicesUsageFlags; theIconRef: IconRef ): OSErr; external name '_GetIconSizesFromIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
   IsIconRefMaskEmpty
   Returns true if the mask for this icon is blank
}
>>>>>>> origin/cpstrnew

{$endc} {TARGET_CPU_64}

{
<<<<<<< HEAD
   GetIconSizesFromIconRef
   
   This routine returns an IconSelectorValue indicating the depths and sizes of 
   icon data which are actually available.  It takes an IconSelectorValue 
   indicating which sizes/depths the caller is interested and returns an 
   IconSelectorValue indicating which sizes/depths exist.
   
   Caution:
   
   This is potentially an extremely expensive call as IconServices may be forced 
   to attempt fetching the data for the IconRef's sizes/depths which may result 
   in hitting the local disk or even the network to obtain the data to determine 
   which sizes/depths actually exist.
   Pass kIconServicesNormalUsageFlag as a default value for IconServicesUsageFlags.
   
   This call is deprecated. Please use IsDataAvailableInIconRef() instead.
}

{$ifc not TARGET_CPU_64}
{
 *  GetIconSizesFromIconRef()   *** DEPRECATED ***
=======
 *  IsIconRefMaskEmpty()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function GetIconSizesFromIconRef( iconSelectorInput: IconSelectorValue; var iconSelectorOutputPtr: IconSelectorValue; iconServicesUsageFlags_: IconServicesUsageFlags; theIconRef: IconRef ): OSErr; external name '_GetIconSizesFromIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

<<<<<<< HEAD
<<<<<<< HEAD

{
   IsIconRefMaskEmpty
   Returns true if the mask for this icon is blank
}
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IsIconRefMaskEmpty( iconRef_: IconRef ): Boolean; external name '_IsIconRefMaskEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew

{$endc} {TARGET_CPU_64}

{
<<<<<<< HEAD
 *  IsIconRefMaskEmpty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IsIconRefMaskEmpty( iconRef_: IconRef ): Boolean; external name '_IsIconRefMaskEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
=======

{
   IsIconRefMaskEmpty
   Returns true if the mask for this icon is blank
}

{$endc} {TARGET_CPU_64}

{
 *  IsIconRefMaskEmpty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IsIconRefMaskEmpty( iconRef_: IconRef ): Boolean; external name '_IsIconRefMaskEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
>>>>>>> graemeg/cpstrnew
=======

{
=======

{
 *  GetIconSizesFromIconRef()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconSizesFromIconRef(iconSelectorInput: IconSelectorValue; var iconSelectorOutputPtr: IconSelectorValue; iconServicesUsageFlags_: IconServicesUsageFlags; theIconRef: IconRef): OSErr; external name '_GetIconSizesFromIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

{ IsDataAvailableInIconRef}
{
 *  IsDataAvailableInIconRef()
 *  
 *  Summary:
 *    Check if IconRef has specific data.
 *  
 *  Discussion:
 *    This routine returns true if inIconKind icon data is availabe or
 *    can be created.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inIconKind:
 *      The icon data kind
 *    
 *    inIconRef:
 *      The IconRef to test.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function IsDataAvailableInIconRef( inIconKind: OSType; inIconRef: IconRef ): Boolean; external name '_IsDataAvailableInIconRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{
  ==============================================================================
   Flushing IconRef data
  ==============================================================================
}


{
   FlushIconRefs
   
   Making this call will dispose of all the data for the specified icons if 
   the data can be reacquired, for example if the data is provided from a resource.
   '****' is a wildcard for all types or all creators.
}

{
 *  FlushIconRefs()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function FlushIconRefs(creator: OSType; iconType: OSType): OSErr; external name '_FlushIconRefs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

{
   FlushIconRefsByVolume
   
   This routine disposes of the data for the icons related to the indicated volume
   if this data can be reacquired, for example if the data is provided from a 
   resource.
}

{
 *  FlushIconRefsByVolume()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function FlushIconRefsByVolume(vRefNum: SInt16): OSErr; external name '_FlushIconRefsByVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

{
  ==============================================================================
   Controling custom icons
  ==============================================================================
}


{
   SetCustomIconsEnabled
   
   Enable or disable custom icons on the specified volume.
}

{
 *  SetCustomIconsEnabled()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function SetCustomIconsEnabled(vRefNum: SInt16; enableCustomIcons: boolean): OSErr; external name '_SetCustomIconsEnabled';
{
   GetCustomIconsEnabled
   
   Return true if custom icons are enabled on the specified volume, false otherwise.
}

{
 *  GetCustomIconsEnabled()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetCustomIconsEnabled(vRefNum: SInt16; var customIconsEnabled: boolean): OSErr; external name '_GetCustomIconsEnabled';
{
>>>>>>> graemeg/fixes_2_2
   IsIconRefMaskEmpty
   Returns true if the mask for this icon is blank
}

<<<<<<< HEAD
{$endc} {TARGET_CPU_64}

{
 *  IsIconRefMaskEmpty()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 }
function IsIconRefMaskEmpty( iconRef_: IconRef ): Boolean; external name '_IsIconRefMaskEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
{
 *  IsIconRefMaskEmpty()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 8.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function IsIconRefMaskEmpty(iconRef_: IconRef): boolean; external name '_IsIconRefMaskEmpty';
{
>>>>>>> graemeg/fixes_2_2
   GetIconRefVariant
   Icon variants allows different images to be used with different icon state.
   For example, the 'open' variant for a folder could be represented with
   an open folder.
   Given an icon ref and a desired variant, this routine returns an icon
   ref (which may be the same as the input icon ref) and a transformation 
   which should be used with PlotIconRef() to render the icon appropriately.
   The returned icon ref should be used to do hit-testing.
}

{
 *  GetIconRefVariant()
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in IconServicesLib 9.0 and later
 }
function GetIconRefVariant( inIconRef: IconRef; inVariant: OSType; var outTransform: IconTransformType ): IconRef; external name '_GetIconRefVariant';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 9.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function GetIconRefVariant(inIconRef: IconRef; inVariant: OSType; var outTransform: IconTransformType): IconRef; external name '_GetIconRefVariant';
{
  ==============================================================================
   Icon files (.icns files)
  ==============================================================================
}


{
   RegisterIconRefFromIconFile
   This routine adds a new entry to the IconRef registry. Other clients will be 
   able to access it using the (creator, iconType) pair specified here.
   Lower-case creators are reserved for the system.
   If the creator is kSystemIconsCreator and the iconType is 0, a new IconRef
   is always returned. Otherwise, if the creator and type have already been
   registered, the previously registered IconRef is returned.
   This routine increments the reference count of the IconRef. Call ReleaseIconRef() 
   when you're done with it.
}

{
 *  RegisterIconRefFromIconFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 9.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function RegisterIconRefFromIconFile(creator: OSType; iconType: OSType; const (*var*) iconFile: FSSpec; var theIconRef: IconRef): OSErr; external name '_RegisterIconRefFromIconFile';
{
   ReadIconFile
   Read the specified icon file into the icon family handle.
   The caller is responsible for disposing the iconFamily
}

{
 *  ReadIconFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 9.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ReadIconFile(const (*var*) iconFile: FSSpec; var iconFamily: IconFamilyHandle): OSErr; external name '_ReadIconFile';
{  ReadIconFromFSRef }
{
 *  ReadIconFromFSRef()
 *  
 *  Discussion:
 *    This routine reads an icon (icns) file into memory.
 *  
 *  Parameters:
 *    
 *    ref:
 *      The FSRef for the icon file.
 *    
 *    iconFamily:
 *      The handle for the icon family.
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Mac OS X:         in version 10.1 and later
 }
function ReadIconFromFSRef(const (*var*) ref: FSRef; var iconFamily: IconFamilyHandle): OSStatus; external name '_ReadIconFromFSRef';


{
   WriteIconFile
   Write the iconFamily handle to the specified file
}

{
 *  WriteIconFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in IconServicesLib 9.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function WriteIconFile(iconFamily: IconFamilyHandle; const (*var*) iconFile: FSSpec): OSErr; external name '_WriteIconFile';
{$ALIGN MAC68K}


end.
>>>>>>> graemeg/fixes_2_2
