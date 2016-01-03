{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/QuickTimeStreaming.h
 
     Contains:   QuickTime Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    QuickTime 7.7.1
 
     Copyright:  © 1990-2012 by Apple Inc., all rights reserved
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       QuickTimeStreaming.p
=======
     File:       QuickTime/QuickTimeStreaming.h
>>>>>>> origin/fixes_2.4
 
     Contains:   QuickTime Interfaces.
 
     Version:    QuickTime 7.6.3
 
<<<<<<< HEAD
     Copyright:  © 1990-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
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
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
=======
>>>>>>> origin/fixes_2_2
                     http://www.freepascal.org/bugs.html
 
}
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit QuickTimeStreaming;
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
=======
>>>>>>> origin/fixes_2.4
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> origin/fixes_2.4
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
uses MacTypes,Components,Events,Files,QuickdrawTypes,Movies,ImageCompression,QuickTimeComponents;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/cpstrnew

=======


>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

const
	kQTSInfiniteDuration = $7FFFFFFF;
	kQTSUnknownDuration = $00000000;
	kQTSNormalForwardRate = $00010000;
	kQTSStoppedRate = $00000000;
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Files,Events,ImageCompression,Quickdraw,Components,MacErrors,Movies,QuickTimeComponents;
=======
uses MacTypes,Components,Events,Files,QuickdrawTypes,Movies,ImageCompression,QuickTimeComponents;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kQTSInfiniteDuration = $7FFFFFFF;
	kQTSUnknownDuration = $00000000;
	kQTSNormalForwardRate = $00010000;
	kQTSStoppedRate = $00000000;
>>>>>>> origin/fixes_2.4

type
	QTSPresentationRecordPtr = ^QTSPresentationRecord;
	QTSPresentationRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		data: array [0..1-1] of SIGNEDLONG;
=======
		data: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		data: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		data: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		data: array [0..0] of SInt32;
>>>>>>> origin/cpstrnew
	end;
type
	QTSPresentation = ^QTSPresentationRecord;
	QTSStreamRecordPtr = ^QTSStreamRecord;
	QTSStreamRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		data: array [0..1-1] of SIGNEDLONG;
=======
		data: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		data: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		data: array [0..0] of SInt32;
>>>>>>> graemeg/cpstrnew
=======
		data: array [0..0] of SInt32;
>>>>>>> origin/cpstrnew
	end;
type
	QTSStream = ^QTSStreamRecord;
	QTSEditEntryPtr = ^QTSEditEntry;
	QTSEditEntry = record
		presentationDuration: TimeValue64;
		streamStartTime: TimeValue64;
		streamRate: Fixed;
	end;
type
	QTSEditList = record
		numEdits: SInt32;
		edits: array [0..0] of QTSEditEntry;
	end;
	QTSEditListPtr = ^QTSEditList;
type
	QTSEditListHandle = ^QTSEditListPtr;
	QTSNotificationProcPtr = function( inErr: ComponentResult; inNotificationType: OSType; inNotificationParams: UnivPtr; inRefCon: UnivPtr ): ComponentResult;
	QTSNotificationUPP = QTSNotificationProcPtr;
{-----------------------------------------
    Get / Set Info
-----------------------------------------}
const
	kQTSGetURLLink = FourCharCode('gull'); { QTSGetURLLinkRecord* }

{ get and set }
const
	kQTSTargetBufferDurationInfo = FourCharCode('bufr'); { Fixed* in seconds; expected, not actual }
	kQTSDurationInfo = FourCharCode('dura'); { QTSDurationAtom* }
	kQTSSoundLevelMeteringEnabledInfo = FourCharCode('mtrn'); { Boolean* }
	kQTSSoundLevelMeterInfo = FourCharCode('levm'); { LevelMeterInfoPtr }
	kQTSSourceTrackIDInfo = FourCharCode('otid'); { UInt32* }
	kQTSSourceLayerInfo = FourCharCode('olyr'); { UInt16* }
	kQTSSourceLanguageInfo = FourCharCode('olng'); { UInt16* }
	kQTSSourceTrackFlagsInfo = FourCharCode('otfl'); { SInt32* }
	kQTSSourceDimensionsInfo = FourCharCode('odim'); { QTSDimensionParams* }
	kQTSSourceVolumesInfo = FourCharCode('ovol'); { QTSVolumesParams* }
	kQTSSourceMatrixInfo = FourCharCode('omat'); { MatrixRecord* }
	kQTSSourceClipRectInfo = FourCharCode('oclp'); { Rect* }
	kQTSSourceGraphicsModeInfo = FourCharCode('ogrm'); { QTSGraphicsModeParams* }
	kQTSSourceScaleInfo = FourCharCode('oscl'); { Point* }
	kQTSSourceBoundingRectInfo = FourCharCode('orct'); { Rect* }
	kQTSSourceUserDataInfo = FourCharCode('oudt'); { UserData }
	kQTSSourceInputMapInfo = FourCharCode('oimp'); { QTAtomContainer }
	kQTSInfo_DataProc = FourCharCode('datp'); { QTSDataProcParams* }
	kQTSInfo_SendDataExtras = FourCharCode('dext'); { QTSSendDataExtrasParams* }
	kQTSInfo_HintTrackID = FourCharCode('htid'); { long* }
	kQTSInfo_URL = FourCharCode('url '); { Handle*, cstring in handle }
	kQTSInfo_Authentication = FourCharCode('auup'); { QTSAuthenticationParams }
	kQTSInfo_MediaPacketizer = FourCharCode('rmpk'); { ComponentInstance }

{ get only }
const
	kQTSStatisticsInfo = FourCharCode('stat'); { QTSStatisticsParams* }
	kQTSMinStatusDimensionsInfo = FourCharCode('mstd'); { QTSDimensionParams* }
	kQTSNormalStatusDimensionsInfo = FourCharCode('nstd'); { QTSDimensionParams* }
	kQTSTotalDataRateInfo = FourCharCode('drtt'); { UInt32*, add to what's there }
	kQTSTotalDataRateInInfo = FourCharCode('drti'); { UInt32*, add to what's there }
	kQTSTotalDataRateOutInfo = FourCharCode('drto'); { UInt32*, add to what's there }
	kQTSLostPercentInfo = FourCharCode('lpct'); { QTSLostPercentParams*, add to what's there }
	kQTSNumViewersInfo = FourCharCode('nviw'); { UInt32* }
	kQTSMediaTypeInfo = FourCharCode('mtyp'); { OSType* }
	kQTSNameInfo = FourCharCode('name'); { QTSNameParams* }
	kQTSCanHandleSendDataType = FourCharCode('chsd'); { QTSCanHandleSendDataTypeParams* }
	kQTSAnnotationsInfo = FourCharCode('meta'); { QTAtomContainer }
	kQTSRemainingBufferTimeInfo = FourCharCode('btms'); { UInt32* remaining buffer time before playback, in microseconds }
	kQTSInfo_SettingsText = FourCharCode('sttx'); { QTSSettingsTextParams* }
	kQTSInfo_AverageFrameRate = FourCharCode('fps '); { UnsignedFixed* }
=======
=======
>>>>>>> origin/fixes_2_2
		data:					array [0..0] of SInt32;
=======
		data: array [0..0] of SInt32;
>>>>>>> origin/fixes_2.4
	end;
type
	QTSPresentation = ^QTSPresentationRecord;
	QTSStreamRecordPtr = ^QTSStreamRecord;
	QTSStreamRecord = record
		data: array [0..0] of SInt32;
	end;
type
	QTSStream = ^QTSStreamRecord;
	QTSEditEntryPtr = ^QTSEditEntry;
	QTSEditEntry = record
		presentationDuration: TimeValue64;
		streamStartTime: TimeValue64;
		streamRate: Fixed;
	end;
type
	QTSEditList = record
		numEdits: SInt32;
		edits: array [0..0] of QTSEditEntry;
	end;
	QTSEditListPtr = ^QTSEditList;
type
	QTSEditListHandle = ^QTSEditListPtr;
	QTSNotificationProcPtr = function( inErr: ComponentResult; inNotificationType: OSType; inNotificationParams: UnivPtr; inRefCon: UnivPtr ): ComponentResult;
	QTSNotificationUPP = QTSNotificationProcPtr;
{-----------------------------------------
    Get / Set Info
-----------------------------------------}
const
	kQTSGetURLLink = FourCharCode('gull'); { QTSGetURLLinkRecord* }

{ get and set }
const
	kQTSTargetBufferDurationInfo = FourCharCode('bufr'); { Fixed* in seconds; expected, not actual }
	kQTSDurationInfo = FourCharCode('dura'); { QTSDurationAtom* }
	kQTSSoundLevelMeteringEnabledInfo = FourCharCode('mtrn'); { Boolean* }
	kQTSSoundLevelMeterInfo = FourCharCode('levm'); { LevelMeterInfoPtr }
	kQTSSourceTrackIDInfo = FourCharCode('otid'); { UInt32* }
	kQTSSourceLayerInfo = FourCharCode('olyr'); { UInt16* }
	kQTSSourceLanguageInfo = FourCharCode('olng'); { UInt16* }
	kQTSSourceTrackFlagsInfo = FourCharCode('otfl'); { SInt32* }
	kQTSSourceDimensionsInfo = FourCharCode('odim'); { QTSDimensionParams* }
	kQTSSourceVolumesInfo = FourCharCode('ovol'); { QTSVolumesParams* }
	kQTSSourceMatrixInfo = FourCharCode('omat'); { MatrixRecord* }
	kQTSSourceClipRectInfo = FourCharCode('oclp'); { Rect* }
	kQTSSourceGraphicsModeInfo = FourCharCode('ogrm'); { QTSGraphicsModeParams* }
	kQTSSourceScaleInfo = FourCharCode('oscl'); { Point* }
	kQTSSourceBoundingRectInfo = FourCharCode('orct'); { Rect* }
	kQTSSourceUserDataInfo = FourCharCode('oudt'); { UserData }
	kQTSSourceInputMapInfo = FourCharCode('oimp'); { QTAtomContainer }
	kQTSInfo_DataProc = FourCharCode('datp'); { QTSDataProcParams* }
	kQTSInfo_SendDataExtras = FourCharCode('dext'); { QTSSendDataExtrasParams* }
	kQTSInfo_HintTrackID = FourCharCode('htid'); { long* }
	kQTSInfo_URL = FourCharCode('url '); { Handle*, cstring in handle }
	kQTSInfo_Authentication = FourCharCode('auup'); { QTSAuthenticationParams }
	kQTSInfo_MediaPacketizer = FourCharCode('rmpk'); { ComponentInstance }

{ get only }
const
<<<<<<< HEAD
	kQTSGetURLLink				= FourCharCode('gull');						{  QTSGetURLLinkRecord*  }

	{	 get and set 	}
	kQTSTargetBufferDurationInfo = FourCharCode('bufr');						{  Fixed* in seconds; expected, not actual  }
	kQTSDurationInfo			= FourCharCode('dura');						{  QTSDurationAtom*  }
	kQTSSoundLevelMeteringEnabledInfo = FourCharCode('mtrn');					{  Boolean*  }
	kQTSSoundLevelMeterInfo		= FourCharCode('levm');						{  LevelMeterInfoPtr  }
	kQTSSourceTrackIDInfo		= FourCharCode('otid');						{  UInt32*  }
	kQTSSourceLayerInfo			= FourCharCode('olyr');						{  UInt16*  }
	kQTSSourceLanguageInfo		= FourCharCode('olng');						{  UInt16*  }
	kQTSSourceTrackFlagsInfo	= FourCharCode('otfl');						{  SInt32*  }
	kQTSSourceDimensionsInfo	= FourCharCode('odim');						{  QTSDimensionParams*  }
	kQTSSourceVolumesInfo		= FourCharCode('ovol');						{  QTSVolumesParams*  }
	kQTSSourceMatrixInfo		= FourCharCode('omat');						{  MatrixRecord*  }
	kQTSSourceClipRectInfo		= FourCharCode('oclp');						{  Rect*  }
	kQTSSourceGraphicsModeInfo	= FourCharCode('ogrm');						{  QTSGraphicsModeParams*  }
	kQTSSourceScaleInfo			= FourCharCode('oscl');						{  Point*  }
	kQTSSourceBoundingRectInfo	= FourCharCode('orct');						{  Rect*  }
	kQTSSourceUserDataInfo		= FourCharCode('oudt');						{  UserData  }
	kQTSSourceInputMapInfo		= FourCharCode('oimp');						{  QTAtomContainer  }
	kQTSInfo_DataProc			= FourCharCode('datp');						{  QTSDataProcParams*  }
	kQTSInfo_SendDataExtras		= FourCharCode('dext');						{  QTSSendDataExtrasParams*  }
	kQTSInfo_HintTrackID		= FourCharCode('htid');						{  long*  }
	kQTSInfo_URL				= FourCharCode('url ');						{  Handle*, cstring in handle  }
	kQTSInfo_Authentication		= FourCharCode('auup');						{  QTSAuthenticationParams  }
	kQTSInfo_MediaPacketizer	= FourCharCode('rmpk');						{  ComponentInstance  }

	{	 get only 	}
	kQTSStatisticsInfo			= FourCharCode('stat');						{  QTSStatisticsParams*  }
	kQTSMinStatusDimensionsInfo	= FourCharCode('mstd');						{  QTSDimensionParams*  }
	kQTSNormalStatusDimensionsInfo = FourCharCode('nstd');					{  QTSDimensionParams*  }
	kQTSTotalDataRateInfo		= FourCharCode('drtt');						{  UInt32*, add to what's there  }
	kQTSTotalDataRateInInfo		= FourCharCode('drti');						{  UInt32*, add to what's there  }
	kQTSTotalDataRateOutInfo	= FourCharCode('drto');						{  UInt32*, add to what's there  }
	kQTSLostPercentInfo			= FourCharCode('lpct');						{  QTSLostPercentParams*, add to what's there  }
	kQTSNumViewersInfo			= FourCharCode('nviw');						{  UInt32*  }
	kQTSMediaTypeInfo			= FourCharCode('mtyp');						{  OSType*  }
	kQTSNameInfo				= FourCharCode('name');						{  QTSNameParams*  }
	kQTSCanHandleSendDataType	= FourCharCode('chsd');						{  QTSCanHandleSendDataTypeParams*  }
	kQTSAnnotationsInfo			= FourCharCode('meta');						{  QTAtomContainer  }
	kQTSRemainingBufferTimeInfo	= FourCharCode('btms');						{  UInt32* remaining buffer time before playback, in microseconds  }
	kQTSInfo_SettingsText		= FourCharCode('sttx');						{  QTSSettingsTextParams*  }
	kQTSInfo_AverageFrameRate	= FourCharCode('fps ');						{  UnsignedFixed*  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSStatisticsInfo = FourCharCode('stat'); { QTSStatisticsParams* }
	kQTSMinStatusDimensionsInfo = FourCharCode('mstd'); { QTSDimensionParams* }
	kQTSNormalStatusDimensionsInfo = FourCharCode('nstd'); { QTSDimensionParams* }
	kQTSTotalDataRateInfo = FourCharCode('drtt'); { UInt32*, add to what's there }
	kQTSTotalDataRateInInfo = FourCharCode('drti'); { UInt32*, add to what's there }
	kQTSTotalDataRateOutInfo = FourCharCode('drto'); { UInt32*, add to what's there }
	kQTSLostPercentInfo = FourCharCode('lpct'); { QTSLostPercentParams*, add to what's there }
	kQTSNumViewersInfo = FourCharCode('nviw'); { UInt32* }
	kQTSMediaTypeInfo = FourCharCode('mtyp'); { OSType* }
	kQTSNameInfo = FourCharCode('name'); { QTSNameParams* }
	kQTSCanHandleSendDataType = FourCharCode('chsd'); { QTSCanHandleSendDataTypeParams* }
	kQTSAnnotationsInfo = FourCharCode('meta'); { QTAtomContainer }
	kQTSRemainingBufferTimeInfo = FourCharCode('btms'); { UInt32* remaining buffer time before playback, in microseconds }
	kQTSInfo_SettingsText = FourCharCode('sttx'); { QTSSettingsTextParams* }
	kQTSInfo_AverageFrameRate = FourCharCode('fps '); { UnsignedFixed* }
>>>>>>> origin/fixes_2.4


type
	QTSAuthenticationParamsPtr = ^QTSAuthenticationParams;
	QTSAuthenticationParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: SInt32;
		userID: ConstCStringPtr;                 { caller disposes of pointer}
		password: ConstCStringPtr;               { caller disposes of pointer}
	end;
const
	kQTSTargetBufferDurationTimeScale = 1000;

type
	QTSPanelFilterParamsPtr = ^QTSPanelFilterParams;
	QTSPanelFilterParams = record
		version: SInt32;
		inStream: QTSStream;
		inPanelType: OSType;
		inPanelSubType: OSType;
		details: QTAtomSpec;
	end;
{ return true to keep this panel}
type
	QTSPanelFilterProcPtr = function( var inParams: QTSPanelFilterParams; inRefCon: UnivPtr ): Boolean;
	QTSPanelFilterUPP = QTSPanelFilterProcPtr;
const
	kQTSSettingsTextSummary = FourCharCode('set1');
	kQTSSettingsTextDetails = FourCharCode('setd');
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;
		userID:					ConstCStringPtr;						{  caller disposes of pointer }
		password:				ConstCStringPtr;						{  caller disposes of pointer }
=======
		flags: SInt32;
		userID: ConstCStringPtr;                 { caller disposes of pointer}
		password: ConstCStringPtr;               { caller disposes of pointer}
>>>>>>> origin/fixes_2.4
	end;
const
	kQTSTargetBufferDurationTimeScale = 1000;

type
	QTSPanelFilterParamsPtr = ^QTSPanelFilterParams;
	QTSPanelFilterParams = record
		version: SInt32;
		inStream: QTSStream;
		inPanelType: OSType;
		inPanelSubType: OSType;
		details: QTAtomSpec;
	end;
{ return true to keep this panel}
type
	QTSPanelFilterProcPtr = function( var inParams: QTSPanelFilterParams; inRefCon: UnivPtr ): Boolean;
	QTSPanelFilterUPP = QTSPanelFilterProcPtr;
const
<<<<<<< HEAD
	kQTSSettingsTextSummary		= FourCharCode('set1');
	kQTSSettingsTextDetails		= FourCharCode('setd');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSSettingsTextSummary = FourCharCode('set1');
	kQTSSettingsTextDetails = FourCharCode('setd');
>>>>>>> origin/fixes_2.4

type
	QTSSettingsTextParamsPtr = ^QTSSettingsTextParams;
	QTSSettingsTextParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		flags: SInt32;                  { None yet defined}
		inSettingsSelector: OSType;     { which kind of setting you want from enum above}
		outSettingsAsText: Handle;      { QTS allocates; Caller disposes}
		inPanelFilterProc: QTSPanelFilterUPP;      { To get a subset filter with this   }
		inPanelFilterProcRefCon: UnivPtr;
<<<<<<< HEAD
	end;
type
	QTSCanHandleSendDataTypeParamsPtr = ^QTSCanHandleSendDataTypeParams;
	QTSCanHandleSendDataTypeParams = record
		modifierTypeOrInputID: SInt32;
		isModifierType: Boolean;
		returnedCanHandleSendDataType: Boolean; { callee sets to true if it can handle it}
	end;
type
	QTSNameParams = record
		maxNameLength: SInt32;
		requestedLanguage: SInt32;
		returnedActualLanguage: SInt32;
		returnedName: UInt8Ptr;           { pascal string; caller supplies}
	end;
type
	QTSLostPercentParamsPtr = ^QTSLostPercentParams;
	QTSLostPercentParams = record
		receivedPkts: UInt32;
		lostPkts: UInt32;
		percent: Fixed;
	end;
type
	QTSDimensionParamsPtr = ^QTSDimensionParams;
	QTSDimensionParams = record
		width: Fixed;
		height: Fixed;
	end;
type
	QTSVolumesParamsPtr = ^QTSVolumesParams;
	QTSVolumesParams = record
		leftVolume: SInt16;
		rightVolume: SInt16;
	end;
type
	QTSGraphicsModeParamsPtr = ^QTSGraphicsModeParams;
	QTSGraphicsModeParams = record
		graphicsMode: SInt16;
		opColor: RGBColor;
	end;
type
	QTSGetURLLinkRecordPtr = ^QTSGetURLLinkRecord;
	QTSGetURLLinkRecord = record
		displayWhere: Point;
		returnedURLLink: Handle;
	end;
const
	kQTSDataProcParamsVersion1 = 1;

const
	kQTSDataProcType_MediaSample = FourCharCode('mdia');
	kQTSDataProcType_HintSample = FourCharCode('hint');
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;									{  None yet defined }
		inSettingsSelector:		OSType;									{  which kind of setting you want from enum above }
		outSettingsAsText:		Handle;									{  QTS allocates; Caller disposes }
		inPanelFilterProc:		QTSPanelFilterUPP;						{  To get a subset filter with this    }
		inPanelFilterProcRefCon: Ptr;
=======
>>>>>>> origin/fixes_2.4
	end;
type
	QTSCanHandleSendDataTypeParamsPtr = ^QTSCanHandleSendDataTypeParams;
	QTSCanHandleSendDataTypeParams = record
		modifierTypeOrInputID: SInt32;
		isModifierType: Boolean;
		returnedCanHandleSendDataType: Boolean; { callee sets to true if it can handle it}
	end;
type
	QTSNameParams = record
		maxNameLength: SInt32;
		requestedLanguage: SInt32;
		returnedActualLanguage: SInt32;
		returnedName: UInt8Ptr;           { pascal string; caller supplies}
	end;
type
	QTSLostPercentParamsPtr = ^QTSLostPercentParams;
	QTSLostPercentParams = record
		receivedPkts: UInt32;
		lostPkts: UInt32;
		percent: Fixed;
	end;
type
	QTSDimensionParamsPtr = ^QTSDimensionParams;
	QTSDimensionParams = record
		width: Fixed;
		height: Fixed;
	end;
type
	QTSVolumesParamsPtr = ^QTSVolumesParams;
	QTSVolumesParams = record
		leftVolume: SInt16;
		rightVolume: SInt16;
	end;
type
	QTSGraphicsModeParamsPtr = ^QTSGraphicsModeParams;
	QTSGraphicsModeParams = record
		graphicsMode: SInt16;
		opColor: RGBColor;
	end;
type
	QTSGetURLLinkRecordPtr = ^QTSGetURLLinkRecord;
	QTSGetURLLinkRecord = record
		displayWhere: Point;
		returnedURLLink: Handle;
	end;
const
	kQTSDataProcParamsVersion1 = 1;

const
	kQTSDataProcType_MediaSample = FourCharCode('mdia');
<<<<<<< HEAD
	kQTSDataProcType_HintSample	= FourCharCode('hint');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSDataProcType_HintSample = FourCharCode('hint');
>>>>>>> origin/fixes_2.4

type
	QTSDataProcParamsPtr = ^QTSDataProcParams;
	QTSDataProcParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		version: SInt32;
		flags: SInt32;
		stream: QTSStream;
		procType: OSType;
		proc: QTSNotificationUPP;
		procRefCon: UnivPtr;
<<<<<<< HEAD
	end;
=======
=======
>>>>>>> origin/fixes_2_2
		version:				SInt32;
		flags:					SInt32;
		stream:					QTSStream;
		procType:				OSType;
		proc:					QTSNotificationUPP;
		procRefCon:				Ptr;
	end;


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	end;
>>>>>>> origin/fixes_2.4
const
	kQTSDataProcSelector_SampleData = FourCharCode('samp');
	kQTSDataProcSelector_UserData = FourCharCode('user');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kQTSSampleDataCallbackParamsVersion1 = 1;

type
	QTSSampleDataCallbackParamsPtr = ^QTSSampleDataCallbackParams;
	QTSSampleDataCallbackParams = record
		version: SInt32;
		flags: SInt32;
		stream: QTSStream;
		procType: OSType;
		mediaType: OSType;
		mediaTimeScale: TimeScale;
		sampleDesc: SampleDescriptionHandle;
		sampleDescSeed: UInt32;
		sampleTime: TimeValue64;
		duration: TimeValue64;               { could be 0 }
		sampleFlags: SInt32;
		dataLength: UInt32;
		data: {const} UnivPtr;
	end;
const
	kQTSUserDataCallbackParamsVersion1 = 1;

type
	QTSUserDataCallbackParamsPtr = ^QTSUserDataCallbackParams;
	QTSUserDataCallbackParams = record
		version: SInt32;
		flags: SInt32;
		stream: QTSStream;
		procType: OSType;
		userDataType: OSType;
		userDataHandle: Handle;         { caller must make copy if it wants to keep the data around}
	end;
const
	kQTSSendDataExtrasParamsVersion1 = 1;

type
	QTSSendDataExtrasParamsPtr = ^QTSSendDataExtrasParams;
	QTSSendDataExtrasParams = record
		version: SInt32;
		flags: SInt32;
		procType: OSType;
	end;
type
	QTSModalFilterProcPtr = function( inDialog: DialogPtr; const (*var*) inEvent: EventRecord; var ioItemHit: SInt16; inRefCon: UnivPtr ): Boolean;
	QTSModalFilterUPP = QTSModalFilterProcPtr;
{-----------------------------------------
    Characteristics
-----------------------------------------}
{ characteristics in Movies.h work here too }
const
	kQTSSupportsPerStreamControlCharacteristic = FourCharCode('psct');

type
	QTSVideoParamsPtr = ^QTSVideoParams;
	QTSVideoParams = record
		width: Fixed;
		height: Fixed;
		matrix: MatrixRecord;
		gWorld: CGrafPtr;
		gdHandle: GDHandle_fix;
		clip: RgnHandle;
		graphicsMode: SInt16;
		opColor: RGBColor;
	end;
type
	QTSAudioParamsPtr = ^QTSAudioParams;
	QTSAudioParams = record
		leftVolume: SInt16;
		rightVolume: SInt16;
		bassLevel: SInt16;
		trebleLevel: SInt16;
		frequencyBandsCount: SInt16;
		frequencyBands: UnivPtr;
		levelMeteringEnabled: Boolean;
	end;
type
	QTSMediaParamsPtr = ^QTSMediaParams;
	QTSMediaParams = record
		v: QTSVideoParams;
		a: QTSAudioParams;
	end;
const
	kQTSMustDraw = 1 shl 3;
	kQTSAtEnd = 1 shl 4;
	kQTSPreflightDraw = 1 shl 5;
	kQTSSyncDrawing = 1 shl 6;

{ media task result flags }
const
	kQTSDidDraw = 1 shl 0;
	kQTSNeedsToDraw = 1 shl 2;
	kQTSDrawAgain = 1 shl 3;
	kQTSPartialDraw = 1 shl 4;

{============================================================================
        Notifications
============================================================================}
{ ------ notification types ------ }
const
	kQTSNullNotification = FourCharCode('null'); { NULL }
	kQTSErrorNotification = FourCharCode('err '); { QTSErrorParams*, optional }
	kQTSNewPresDetectedNotification = FourCharCode('newp'); { QTSNewPresDetectedParams* }
	kQTSPresBeginChangingNotification = FourCharCode('prcb'); { NULL }
	kQTSPresDoneChangingNotification = FourCharCode('prcd'); { NULL }
	kQTSPresentationChangedNotification = FourCharCode('prch'); { NULL }
	kQTSNewStreamNotification = FourCharCode('stnw'); { QTSNewStreamParams* }
	kQTSStreamBeginChangingNotification = FourCharCode('stcb'); { QTSStream }
	kQTSStreamDoneChangingNotification = FourCharCode('stcd'); { QTSStream }
	kQTSStreamChangedNotification = FourCharCode('stch'); { QTSStreamChangedParams* }
	kQTSStreamGoneNotification = FourCharCode('stgn'); { QTSStreamGoneParams* }
	kQTSPreviewAckNotification = FourCharCode('pvak'); { QTSStream }
	kQTSPrerollAckNotification = FourCharCode('pack'); { QTSStream }
	kQTSStartAckNotification = FourCharCode('sack'); { QTSStream }
	kQTSStopAckNotification = FourCharCode('xack'); { QTSStream }
	kQTSStatusNotification = FourCharCode('stat'); { QTSStatusParams* }
	kQTSURLNotification = FourCharCode('url '); { QTSURLParams* }
	kQTSDurationNotification = FourCharCode('dura'); { QTSDurationAtom* }
	kQTSNewPresentationNotification = FourCharCode('nprs'); { QTSPresentation }
	kQTSPresentationGoneNotification = FourCharCode('xprs'); { QTSPresentation }
	kQTSPresentationDoneNotification = FourCharCode('pdon'); { NULL }
	kQTSBandwidthAlertNotification = FourCharCode('bwal'); { QTSBandwidthAlertParams* }
	kQTSAnnotationsChangedNotification = FourCharCode('meta'); { NULL }


{ flags for QTSErrorParams }
const
	kQTSFatalErrorFlag = $00000001;
=======
=======
>>>>>>> origin/fixes_2_2
=======
const
>>>>>>> origin/fixes_2.4
	kQTSSampleDataCallbackParamsVersion1 = 1;

type
	QTSSampleDataCallbackParamsPtr = ^QTSSampleDataCallbackParams;
	QTSSampleDataCallbackParams = record
		version: SInt32;
		flags: SInt32;
		stream: QTSStream;
		procType: OSType;
		mediaType: OSType;
		mediaTimeScale: TimeScale;
		sampleDesc: SampleDescriptionHandle;
		sampleDescSeed: UInt32;
		sampleTime: TimeValue64;
		duration: TimeValue64;               { could be 0 }
		sampleFlags: SInt32;
		dataLength: UInt32;
		data: {const} UnivPtr;
	end;
const
	kQTSUserDataCallbackParamsVersion1 = 1;

type
	QTSUserDataCallbackParamsPtr = ^QTSUserDataCallbackParams;
	QTSUserDataCallbackParams = record
		version: SInt32;
		flags: SInt32;
		stream: QTSStream;
		procType: OSType;
		userDataType: OSType;
		userDataHandle: Handle;         { caller must make copy if it wants to keep the data around}
	end;
const
	kQTSSendDataExtrasParamsVersion1 = 1;

type
	QTSSendDataExtrasParamsPtr = ^QTSSendDataExtrasParams;
	QTSSendDataExtrasParams = record
		version: SInt32;
		flags: SInt32;
		procType: OSType;
	end;
type
	QTSModalFilterProcPtr = function( inDialog: DialogPtr; const (*var*) inEvent: EventRecord; var ioItemHit: SInt16; inRefCon: UnivPtr ): Boolean;
	QTSModalFilterUPP = QTSModalFilterProcPtr;
{-----------------------------------------
    Characteristics
-----------------------------------------}
{ characteristics in Movies.h work here too }
const
	kQTSSupportsPerStreamControlCharacteristic = FourCharCode('psct');

type
	QTSVideoParamsPtr = ^QTSVideoParams;
	QTSVideoParams = record
		width: Fixed;
		height: Fixed;
		matrix: MatrixRecord;
		gWorld: CGrafPtr;
		gdHandle: GDHandle_fix;
		clip: RgnHandle;
		graphicsMode: SInt16;
		opColor: RGBColor;
	end;
type
	QTSAudioParamsPtr = ^QTSAudioParams;
	QTSAudioParams = record
		leftVolume: SInt16;
		rightVolume: SInt16;
		bassLevel: SInt16;
		trebleLevel: SInt16;
		frequencyBandsCount: SInt16;
		frequencyBands: UnivPtr;
		levelMeteringEnabled: Boolean;
	end;
type
	QTSMediaParamsPtr = ^QTSMediaParams;
	QTSMediaParams = record
		v: QTSVideoParams;
		a: QTSAudioParams;
	end;
const
	kQTSMustDraw = 1 shl 3;
	kQTSAtEnd = 1 shl 4;
	kQTSPreflightDraw = 1 shl 5;
	kQTSSyncDrawing = 1 shl 6;

{ media task result flags }
const
	kQTSDidDraw = 1 shl 0;
	kQTSNeedsToDraw = 1 shl 2;
	kQTSDrawAgain = 1 shl 3;
	kQTSPartialDraw = 1 shl 4;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{============================================================================
        Notifications
============================================================================}
{ ------ notification types ------ }
const
	kQTSNullNotification = FourCharCode('null'); { NULL }
	kQTSErrorNotification = FourCharCode('err '); { QTSErrorParams*, optional }
	kQTSNewPresDetectedNotification = FourCharCode('newp'); { QTSNewPresDetectedParams* }
	kQTSPresBeginChangingNotification = FourCharCode('prcb'); { NULL }
	kQTSPresDoneChangingNotification = FourCharCode('prcd'); { NULL }
	kQTSPresentationChangedNotification = FourCharCode('prch'); { NULL }
	kQTSNewStreamNotification = FourCharCode('stnw'); { QTSNewStreamParams* }
	kQTSStreamBeginChangingNotification = FourCharCode('stcb'); { QTSStream }
	kQTSStreamDoneChangingNotification = FourCharCode('stcd'); { QTSStream }
	kQTSStreamChangedNotification = FourCharCode('stch'); { QTSStreamChangedParams* }
	kQTSStreamGoneNotification = FourCharCode('stgn'); { QTSStreamGoneParams* }
	kQTSPreviewAckNotification = FourCharCode('pvak'); { QTSStream }
	kQTSPrerollAckNotification = FourCharCode('pack'); { QTSStream }
	kQTSStartAckNotification = FourCharCode('sack'); { QTSStream }
	kQTSStopAckNotification = FourCharCode('xack'); { QTSStream }
	kQTSStatusNotification = FourCharCode('stat'); { QTSStatusParams* }
	kQTSURLNotification = FourCharCode('url '); { QTSURLParams* }
	kQTSDurationNotification = FourCharCode('dura'); { QTSDurationAtom* }
	kQTSNewPresentationNotification = FourCharCode('nprs'); { QTSPresentation }
	kQTSPresentationGoneNotification = FourCharCode('xprs'); { QTSPresentation }
	kQTSPresentationDoneNotification = FourCharCode('pdon'); { NULL }
	kQTSBandwidthAlertNotification = FourCharCode('bwal'); { QTSBandwidthAlertParams* }
	kQTSAnnotationsChangedNotification = FourCharCode('meta'); { NULL }


{ flags for QTSErrorParams }
const
	kQTSFatalErrorFlag = $00000001;
>>>>>>> origin/fixes_2.4

type
	QTSErrorParamsPtr = ^QTSErrorParams;
	QTSErrorParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		errorString: ConstCStringPtr;
		flags: SInt32;
	end;
type
	QTSNewPresDetectedParamsPtr = ^QTSNewPresDetectedParams;
	QTSNewPresDetectedParams = record
		data: UnivPtr;
	end;
type
	QTSNewStreamParamsPtr = ^QTSNewStreamParams;
	QTSNewStreamParams = record
		stream: QTSStream;
	end;
type
	QTSStreamChangedParamsPtr = ^QTSStreamChangedParams;
	QTSStreamChangedParams = record
		stream: QTSStream;
		mediaComponent: ComponentInstance;         { could be NULL }
	end;
type
	QTSStreamGoneParamsPtr = ^QTSStreamGoneParams;
	QTSStreamGoneParams = record
		stream: QTSStream;
	end;
type
	QTSStatusParamsPtr = ^QTSStatusParams;
	QTSStatusParams = record
		status: UInt32;
		statusString: ConstCStringPtr;
		detailedStatus: UInt32;
		detailedStatusString: ConstCStringPtr;
	end;
type
	QTSInfoParamsPtr = ^QTSInfoParams;
	QTSInfoParams = record
		infoType: OSType;
		infoParams: UnivPtr;
	end;
type
	QTSURLParamsPtr = ^QTSURLParams;
	QTSURLParams = record
		urlLength: UInt32;
		url: ConstCStringPtr;
	end;
const
	kQTSBandwidthAlertNeedToStop = 1 shl 0;
	kQTSBandwidthAlertRestartAt = 1 shl 1;
=======
=======
>>>>>>> origin/fixes_2_2
		errorString:			ConstCStringPtr;
		flags:					SInt32;
=======
		errorString: ConstCStringPtr;
		flags: SInt32;
>>>>>>> origin/fixes_2.4
	end;
type
	QTSNewPresDetectedParamsPtr = ^QTSNewPresDetectedParams;
	QTSNewPresDetectedParams = record
		data: UnivPtr;
	end;
type
	QTSNewStreamParamsPtr = ^QTSNewStreamParams;
	QTSNewStreamParams = record
		stream: QTSStream;
	end;
type
	QTSStreamChangedParamsPtr = ^QTSStreamChangedParams;
	QTSStreamChangedParams = record
		stream: QTSStream;
		mediaComponent: ComponentInstance;         { could be NULL }
	end;
type
	QTSStreamGoneParamsPtr = ^QTSStreamGoneParams;
	QTSStreamGoneParams = record
		stream: QTSStream;
	end;
type
	QTSStatusParamsPtr = ^QTSStatusParams;
	QTSStatusParams = record
		status: UInt32;
		statusString: ConstCStringPtr;
		detailedStatus: UInt32;
		detailedStatusString: ConstCStringPtr;
	end;
type
	QTSInfoParamsPtr = ^QTSInfoParams;
	QTSInfoParams = record
		infoType: OSType;
		infoParams: UnivPtr;
	end;
type
	QTSURLParamsPtr = ^QTSURLParams;
	QTSURLParams = record
		urlLength: UInt32;
		url: ConstCStringPtr;
	end;
const
<<<<<<< HEAD
	kQTSBandwidthAlertNeedToStop = $01;
	kQTSBandwidthAlertRestartAt	= $02;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSBandwidthAlertNeedToStop = 1 shl 0;
	kQTSBandwidthAlertRestartAt = 1 shl 1;
>>>>>>> origin/fixes_2.4

type
	QTSBandwidthAlertParamsPtr = ^QTSBandwidthAlertParams;
	QTSBandwidthAlertParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: SInt32;
		restartAt: TimeValue;              { new field in QT 4.1}
		reserved: UnivPtr;
	end;
{============================================================================
        Presentation
============================================================================}
{-----------------------------------------
     Flags
-----------------------------------------}
{ flags for NewPresentationFromData }
const
	kQTSAutoModeFlag = $00000001;
	kQTSDontShowStatusFlag = $00000008;
	kQTSSendMediaFlag = $00010000;
	kQTSReceiveMediaFlag = $00020000;
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;
		restartAt:				TimeValue;								{  new field in QT 4.1 }
		reserved:				Ptr;
=======
		flags: SInt32;
		restartAt: TimeValue;              { new field in QT 4.1}
		reserved: UnivPtr;
>>>>>>> origin/fixes_2.4
	end;
{============================================================================
        Presentation
============================================================================}
{-----------------------------------------
     Flags
-----------------------------------------}
{ flags for NewPresentationFromData }
const
<<<<<<< HEAD
	kQTSAutoModeFlag			= $00000001;
	kQTSDontShowStatusFlag		= $00000008;
	kQTSSendMediaFlag			= $00010000;
	kQTSReceiveMediaFlag		= $00020000;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSAutoModeFlag = $00000001;
	kQTSDontShowStatusFlag = $00000008;
	kQTSSendMediaFlag = $00010000;
	kQTSReceiveMediaFlag = $00020000;
>>>>>>> origin/fixes_2.4

type
	QTSNewPresentationParamsPtr = ^QTSNewPresentationParams;
	QTSNewPresentationParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		dataType: OSType;
		data: {const} UnivPtr;
		dataLength: UInt32;
		editList: QTSEditListHandle;
		flags: SInt32;
		timeScale: TimeScale_fix;              { set to 0 for default timescale }
		mediaParams: QTSMediaParamsPtr;
		notificationProc: QTSNotificationUPP;
		notificationRefCon: UnivPtr;
<<<<<<< HEAD
	end;
type
	QTSPresParamsPtr = ^QTSPresParams;
	QTSPresParams = record
		version: UInt32;
		editList: QTSEditListHandle;
		flags: SInt32;
		timeScale: TimeScale_fix;              { set to 0 for default timescale }
		mediaParams: QTSMediaParamsPtr;
		notificationProc: QTSNotificationUPP;
		notificationRefCon: UnivPtr;
	end;
const
	kQTSPresParamsVersion1 = 1;
=======
=======
>>>>>>> origin/fixes_2_2
		dataType:				OSType;
		data:					Ptr;
		dataLength:				UInt32;
		editList:				QTSEditListHandle;
		flags:					SInt32;
		timeScale:				TimeScale_fix;	{  set to 0 for default timescale  }
		mediaParams:			QTSMediaParamsPtr;
		notificationProc:		QTSNotificationUPP;
		notificationRefCon:		Ptr;
=======
>>>>>>> origin/fixes_2.4
	end;
type
	QTSPresParamsPtr = ^QTSPresParams;
	QTSPresParams = record
		version: UInt32;
		editList: QTSEditListHandle;
		flags: SInt32;
		timeScale: TimeScale_fix;              { set to 0 for default timescale }
		mediaParams: QTSMediaParamsPtr;
		notificationProc: QTSNotificationUPP;
		notificationRefCon: UnivPtr;
	end;
const
<<<<<<< HEAD
	kQTSPresParamsVersion1		= 1;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSPresParamsVersion1 = 1;
>>>>>>> origin/fixes_2.4

type
	QTSPresIdleParamsPtr = ^QTSPresIdleParams;
	QTSPresIdleParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		stream: QTSStream;
		movieTimeToDisplay: TimeValue64;
		flagsIn: SInt32;
		flagsOut: SInt32;
<<<<<<< HEAD
	end;
const
	kQTSExportFlag_ShowDialog = $00000001;

const
	kQTSExportParamsVersion1 = 1;
=======
=======
>>>>>>> origin/fixes_2_2
		stream:					QTSStream;
		movieTimeToDisplay:		TimeValue64;
		flagsIn:				SInt32;
		flagsOut:				SInt32;
=======
>>>>>>> origin/fixes_2.4
	end;
const
	kQTSExportFlag_ShowDialog = $00000001;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kQTSExportParamsVersion1 = 1;
>>>>>>> origin/fixes_2.4

type
	QTSExportParamsPtr = ^QTSExportParams;
	QTSExportParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		version: SInt32;
		exportType: OSType;
		exportExtraData: UnivPtr;
		destinationContainerType: OSType;
		destinationContainerData: UnivPtr;
		destinationContainerExtras: UnivPtr;
		flagsIn: SInt32;
		flagsOut: SInt32;
		filterProc: QTSModalFilterUPP;
		filterProcRefCon: UnivPtr;
		exportComponent: Component;        { NULL unless you want to override }
	end;
{-----------------------------------------
    Toolbox Init/Close
-----------------------------------------}
{ all "apps" must call this }
{
 *  InitializeQTS()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function InitializeQTS: OSErr; external name '_InitializeQTS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
		version:				SInt32;
		exportType:				OSType;
		exportExtraData:		Ptr;
=======
		version: SInt32;
		exportType: OSType;
		exportExtraData: UnivPtr;
>>>>>>> origin/fixes_2.4
		destinationContainerType: OSType;
		destinationContainerData: UnivPtr;
		destinationContainerExtras: UnivPtr;
		flagsIn: SInt32;
		flagsOut: SInt32;
		filterProc: QTSModalFilterUPP;
		filterProcRefCon: UnivPtr;
		exportComponent: Component;        { NULL unless you want to override }
	end;
{-----------------------------------------
    Toolbox Init/Close
-----------------------------------------}
{ all "apps" must call this }
{
 *  InitializeQTS()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function InitializeQTS: OSErr; external name '_InitializeQTS';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  TerminateQTS()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function TerminateQTS: OSErr; external name '_TerminateQTS';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function TerminateQTS: OSErr; external name '_TerminateQTS';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{-----------------------------------------
    Presentation Functions
-----------------------------------------}
{
 *  QTSNewPresentation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSNewPresentation( const (*var*) inParams: QTSNewPresentationParams; var outPresentation: QTSPresentation ): OSErr; external name '_QTSNewPresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSNewPresentation(const (*var*) inParams: QTSNewPresentationParams; var outPresentation: QTSPresentation): OSErr; external name '_QTSNewPresentation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSNewPresentation( const (*var*) inParams: QTSNewPresentationParams; var outPresentation: QTSPresentation ): OSErr; external name '_QTSNewPresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSNewPresentationFromData()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
function QTSNewPresentationFromData( inDataType: OSType; inData: {const} UnivPtr; (*const*) var inDataLength: SInt64; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation ): OSErr; external name '_QTSNewPresentationFromData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
function QTSNewPresentationFromData(inDataType: OSType; inData: UnivPtr; (*const*) var inDataLength: SInt64; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation): OSErr; external name '_QTSNewPresentationFromData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  QTSNewPresentationFromFile()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
function QTSNewPresentationFromFile( const (*var*) inFileSpec: FSSpec; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation ): OSErr; external name '_QTSNewPresentationFromFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
<<<<<<< HEAD
function QTSNewPresentationFromFile(const (*var*) inFileSpec: FSSpec; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation): OSErr; external name '_QTSNewPresentationFromFile';
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
function QTSNewPresentationFromFile(const (*var*) inFileSpec: FSSpec; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation): OSErr; external name '_QTSNewPresentationFromFile';
>>>>>>> origin/fixes_2_2
=======
function QTSNewPresentationFromFile( const (*var*) inFileSpec: FSSpec; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation ): OSErr; external name '_QTSNewPresentationFromFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSNewPresentationFromDataRef()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
function QTSNewPresentationFromDataRef( inDataRef: Handle; inDataRefType: OSType; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation ): OSErr; external name '_QTSNewPresentationFromDataRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
<<<<<<< HEAD
function QTSNewPresentationFromDataRef(inDataRef: Handle; inDataRefType: OSType; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation): OSErr; external name '_QTSNewPresentationFromDataRef';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSNewPresentationFromDataRef( inDataRef: Handle; inDataRefType: OSType; const (*var*) inPresParams: QTSPresParams; var outPresentation: QTSPresentation ): OSErr; external name '_QTSNewPresentationFromDataRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSDisposePresentation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSDisposePresentation( inPresentation: QTSPresentation; inFlags: SInt32 ): OSErr; external name '_QTSDisposePresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSDisposePresentation(inPresentation: QTSPresentation; inFlags: SInt32): OSErr; external name '_QTSDisposePresentation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSDisposePresentation( inPresentation: QTSPresentation; inFlags: SInt32 ): OSErr; external name '_QTSDisposePresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresExport()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
function QTSPresExport( inPresentation: QTSPresentation; inStream: QTSStream; var inExportParams: QTSExportParams ): OSErr; external name '_QTSPresExport';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresExport(inPresentation: QTSPresentation; inStream: QTSStream; var inExportParams: QTSExportParams): OSErr; external name '_QTSPresExport';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresExport( inPresentation: QTSPresentation; inStream: QTSStream; var inExportParams: QTSExportParams ): OSErr; external name '_QTSPresExport';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresIdle()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
procedure QTSPresIdle( inPresentation: QTSPresentation; var ioParams: QTSPresIdleParams ); external name '_QTSPresIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
procedure QTSPresIdle(inPresentation: QTSPresentation; var ioParams: QTSPresIdleParams); external name '_QTSPresIdle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure QTSPresIdle( inPresentation: QTSPresentation; var ioParams: QTSPresIdleParams ); external name '_QTSPresIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresInvalidateRegion()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresInvalidateRegion( inPresentation: QTSPresentation; inRegion: RgnHandle ): OSErr; external name '_QTSPresInvalidateRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresInvalidateRegion(inPresentation: QTSPresentation; inRegion: RgnHandle): OSErr; external name '_QTSPresInvalidateRegion';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresInvalidateRegion( inPresentation: QTSPresentation; inRegion: RgnHandle ): OSErr; external name '_QTSPresInvalidateRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{-----------------------------------------
    Presentation Configuration
-----------------------------------------}
{
 *  QTSPresSetFlags()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetFlags( inPresentation: QTSPresentation; inFlags: SInt32; inFlagsMask: SInt32 ): OSErr; external name '_QTSPresSetFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetFlags(inPresentation: QTSPresentation; inFlags: SInt32; inFlagsMask: SInt32): OSErr; external name '_QTSPresSetFlags';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetFlags( inPresentation: QTSPresentation; inFlags: SInt32; inFlagsMask: SInt32 ): OSErr; external name '_QTSPresSetFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetFlags()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetFlags( inPresentation: QTSPresentation; var outFlags: SInt32 ): OSErr; external name '_QTSPresGetFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetFlags(inPresentation: QTSPresentation; var outFlags: SInt32): OSErr; external name '_QTSPresGetFlags';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetFlags( inPresentation: QTSPresentation; var outFlags: SInt32 ): OSErr; external name '_QTSPresGetFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetTimeBase()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetTimeBase( inPresentation: QTSPresentation; var outTimeBase: TimeBase ): OSErr; external name '_QTSPresGetTimeBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetTimeBase(inPresentation: QTSPresentation; var outTimeBase: TimeBase): OSErr; external name '_QTSPresGetTimeBase';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetTimeBase( inPresentation: QTSPresentation; var outTimeBase: TimeBase ): OSErr; external name '_QTSPresGetTimeBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetTimeScale()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetTimeScale( inPresentation: QTSPresentation; var outTimeScale: TimeScale ): OSErr; external name '_QTSPresGetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetTimeScale(inPresentation: QTSPresentation; var outTimeScale: TimeScale): OSErr; external name '_QTSPresGetTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetTimeScale( inPresentation: QTSPresentation; var outTimeScale: TimeScale ): OSErr; external name '_QTSPresGetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetInfo( inPresentation: QTSPresentation; inStream: QTSStream; inSelector: OSType; ioParam: UnivPtr ): OSErr; external name '_QTSPresSetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetInfo(inPresentation: QTSPresentation; inStream: QTSStream; inSelector: OSType; ioParam: UnivPtr): OSErr; external name '_QTSPresSetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetInfo( inPresentation: QTSPresentation; inStream: QTSStream; inSelector: OSType; ioParam: UnivPtr ): OSErr; external name '_QTSPresSetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetInfo( inPresentation: QTSPresentation; inStream: QTSStream; inSelector: OSType; ioParam: UnivPtr ): OSErr; external name '_QTSPresGetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetInfo(inPresentation: QTSPresentation; inStream: QTSStream; inSelector: OSType; ioParam: UnivPtr): OSErr; external name '_QTSPresGetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetInfo( inPresentation: QTSPresentation; inStream: QTSStream; inSelector: OSType; ioParam: UnivPtr ): OSErr; external name '_QTSPresGetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresHasCharacteristic()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresHasCharacteristic( inPresentation: QTSPresentation; inStream: QTSStream; inCharacteristic: OSType; var outHasIt: Boolean ): OSErr; external name '_QTSPresHasCharacteristic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresHasCharacteristic(inPresentation: QTSPresentation; inStream: QTSStream; inCharacteristic: OSType; var outHasIt: boolean): OSErr; external name '_QTSPresHasCharacteristic';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresHasCharacteristic( inPresentation: QTSPresentation; inStream: QTSStream; inCharacteristic: OSType; var outHasIt: Boolean ): OSErr; external name '_QTSPresHasCharacteristic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetNotificationProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetNotificationProc( inPresentation: QTSPresentation; inNotificationProc: QTSNotificationUPP; inRefCon: UnivPtr ): OSErr; external name '_QTSPresSetNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetNotificationProc(inPresentation: QTSPresentation; inNotificationProc: QTSNotificationUPP; inRefCon: UnivPtr): OSErr; external name '_QTSPresSetNotificationProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetNotificationProc( inPresentation: QTSPresentation; inNotificationProc: QTSNotificationUPP; inRefCon: UnivPtr ): OSErr; external name '_QTSPresSetNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetNotificationProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetNotificationProc( inPresentation: QTSPresentation; var outNotificationProc: QTSNotificationUPP; var outRefCon: UnivPtr ): OSErr; external name '_QTSPresGetNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetNotificationProc(inPresentation: QTSPresentation; var outNotificationProc: QTSNotificationUPP; var outRefCon: UnivPtr): OSErr; external name '_QTSPresGetNotificationProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetNotificationProc( inPresentation: QTSPresentation; var outNotificationProc: QTSNotificationUPP; var outRefCon: UnivPtr ): OSErr; external name '_QTSPresGetNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{-----------------------------------------
    Presentation Control
-----------------------------------------}
{
 *  QTSPresPreview()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSPresPreview( inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inTimeValue: TimeValue64; inRate: Fixed; inFlags: SInt32 ): OSErr; external name '_QTSPresPreview';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresPreview(inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inTimeValue: TimeValue64; inRate: Fixed; inFlags: SInt32): OSErr; external name '_QTSPresPreview';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresPreview( inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inTimeValue: TimeValue64; inRate: Fixed; inFlags: SInt32 ): OSErr; external name '_QTSPresPreview';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresPreroll()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresPreroll( inPresentation: QTSPresentation; inStream: QTSStream; inTimeValue: UInt32; inRate: Fixed; inFlags: SInt32 ): OSErr; external name '_QTSPresPreroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresPreroll(inPresentation: QTSPresentation; inStream: QTSStream; inTimeValue: UInt32; inRate: Fixed; inFlags: SInt32): OSErr; external name '_QTSPresPreroll';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresPreroll( inPresentation: QTSPresentation; inStream: QTSStream; inTimeValue: UInt32; inRate: Fixed; inFlags: SInt32 ): OSErr; external name '_QTSPresPreroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresPreroll64()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPresPreroll64( inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inPrerollTime: TimeValue64; inRate: Fixed; inFlags: SInt32 ): OSErr; external name '_QTSPresPreroll64';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPresPreroll64(inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inPrerollTime: TimeValue64; inRate: Fixed; inFlags: SInt32): OSErr; external name '_QTSPresPreroll64';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresPreroll64( inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inPrerollTime: TimeValue64; inRate: Fixed; inFlags: SInt32 ): OSErr; external name '_QTSPresPreroll64';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresStart()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresStart( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32 ): OSErr; external name '_QTSPresStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresStart(inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32): OSErr; external name '_QTSPresStart';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresStart( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32 ): OSErr; external name '_QTSPresStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSkipTo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSkipTo( inPresentation: QTSPresentation; inTimeValue: UInt32 ): OSErr; external name '_QTSPresSkipTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSkipTo(inPresentation: QTSPresentation; inTimeValue: UInt32): OSErr; external name '_QTSPresSkipTo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSkipTo( inPresentation: QTSPresentation; inTimeValue: UInt32 ): OSErr; external name '_QTSPresSkipTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSkipTo64()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPresSkipTo64( inPresentation: QTSPresentation; (*const*) var inTimeValue: TimeValue64 ): OSErr; external name '_QTSPresSkipTo64';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPresSkipTo64(inPresentation: QTSPresentation; (*const*) var inTimeValue: TimeValue64): OSErr; external name '_QTSPresSkipTo64';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSkipTo64( inPresentation: QTSPresentation; (*const*) var inTimeValue: TimeValue64 ): OSErr; external name '_QTSPresSkipTo64';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresStop()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresStop( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32 ): OSErr; external name '_QTSPresStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresStop(inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32): OSErr; external name '_QTSPresStop';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresStop( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32 ): OSErr; external name '_QTSPresStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{============================================================================
        Streams
============================================================================}
{-----------------------------------------
    Stream Functions
-----------------------------------------}
{
 *  QTSPresNewStream()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresNewStream( inPresentation: QTSPresentation; inDataType: OSType; inData: {const} UnivPtr; inDataLength: UInt32; inFlags: SInt32; var outStream: QTSStream ): OSErr; external name '_QTSPresNewStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresNewStream(inPresentation: QTSPresentation; inDataType: OSType; inData: UnivPtr; inDataLength: UInt32; inFlags: SInt32; var outStream: QTSStream): OSErr; external name '_QTSPresNewStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresNewStream( inPresentation: QTSPresentation; inDataType: OSType; inData: {const} UnivPtr; inDataLength: UInt32; inFlags: SInt32; var outStream: QTSStream ): OSErr; external name '_QTSPresNewStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSDisposeStream()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSDisposeStream( inStream: QTSStream; inFlags: SInt32 ): OSErr; external name '_QTSDisposeStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSDisposeStream(inStream: QTSStream; inFlags: SInt32): OSErr; external name '_QTSDisposeStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSDisposeStream( inStream: QTSStream; inFlags: SInt32 ): OSErr; external name '_QTSDisposeStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetNumStreams()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetNumStreams( inPresentation: QTSPresentation ): UInt32; external name '_QTSPresGetNumStreams';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetNumStreams(inPresentation: QTSPresentation): UInt32; external name '_QTSPresGetNumStreams';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetNumStreams( inPresentation: QTSPresentation ): UInt32; external name '_QTSPresGetNumStreams';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetIndStream()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetIndStream( inPresentation: QTSPresentation; inIndex: UInt32 ): QTSStream; external name '_QTSPresGetIndStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetIndStream(inPresentation: QTSPresentation; inIndex: UInt32): QTSStream; external name '_QTSPresGetIndStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetIndStream( inPresentation: QTSPresentation; inIndex: UInt32 ): QTSStream; external name '_QTSPresGetIndStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSGetStreamPresentation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSGetStreamPresentation( inStream: QTSStream ): QTSPresentation; external name '_QTSGetStreamPresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSGetStreamPresentation(inStream: QTSStream): QTSPresentation; external name '_QTSGetStreamPresentation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSGetStreamPresentation( inStream: QTSStream ): QTSPresentation; external name '_QTSGetStreamPresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetPreferredRate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetPreferredRate( inPresentation: QTSPresentation; inRate: Fixed; inFlags: SInt32 ): OSErr; external name '_QTSPresSetPreferredRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetPreferredRate(inPresentation: QTSPresentation; inRate: Fixed; inFlags: SInt32): OSErr; external name '_QTSPresSetPreferredRate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetPreferredRate( inPresentation: QTSPresentation; inRate: Fixed; inFlags: SInt32 ): OSErr; external name '_QTSPresSetPreferredRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetPreferredRate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetPreferredRate( inPresentation: QTSPresentation; var outRate: Fixed ): OSErr; external name '_QTSPresGetPreferredRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetPreferredRate(inPresentation: QTSPresentation; var outRate: Fixed): OSErr; external name '_QTSPresGetPreferredRate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetPreferredRate( inPresentation: QTSPresentation; var outRate: Fixed ): OSErr; external name '_QTSPresGetPreferredRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetEnable()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetEnable( inPresentation: QTSPresentation; inStream: QTSStream; inEnableMode: Boolean ): OSErr; external name '_QTSPresSetEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetEnable(inPresentation: QTSPresentation; inStream: QTSStream; inEnableMode: boolean): OSErr; external name '_QTSPresSetEnable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetEnable( inPresentation: QTSPresentation; inStream: QTSStream; inEnableMode: Boolean ): OSErr; external name '_QTSPresSetEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetEnable()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetEnable( inPresentation: QTSPresentation; inStream: QTSStream; var outEnableMode: Boolean ): OSErr; external name '_QTSPresGetEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetEnable(inPresentation: QTSPresentation; inStream: QTSStream; var outEnableMode: boolean): OSErr; external name '_QTSPresGetEnable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetEnable( inPresentation: QTSPresentation; inStream: QTSStream; var outEnableMode: Boolean ): OSErr; external name '_QTSPresGetEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetPresenting()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetPresenting( inPresentation: QTSPresentation; inStream: QTSStream; inPresentingMode: Boolean ): OSErr; external name '_QTSPresSetPresenting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetPresenting(inPresentation: QTSPresentation; inStream: QTSStream; inPresentingMode: boolean): OSErr; external name '_QTSPresSetPresenting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetPresenting( inPresentation: QTSPresentation; inStream: QTSStream; inPresentingMode: Boolean ): OSErr; external name '_QTSPresSetPresenting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetPresenting()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetPresenting( inPresentation: QTSPresentation; inStream: QTSStream; var outPresentingMode: Boolean ): OSErr; external name '_QTSPresGetPresenting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetPresenting(inPresentation: QTSPresentation; inStream: QTSStream; var outPresentingMode: boolean): OSErr; external name '_QTSPresGetPresenting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetPresenting( inPresentation: QTSPresentation; inStream: QTSStream; var outPresentingMode: Boolean ): OSErr; external name '_QTSPresGetPresenting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetActiveSegment()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPresSetActiveSegment( inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inStartTime: TimeValue64; (*const*) var inDuration: TimeValue64 ): OSErr; external name '_QTSPresSetActiveSegment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPresSetActiveSegment(inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inStartTime: TimeValue64; (*const*) var inDuration: TimeValue64): OSErr; external name '_QTSPresSetActiveSegment';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetActiveSegment( inPresentation: QTSPresentation; inStream: QTSStream; (*const*) var inStartTime: TimeValue64; (*const*) var inDuration: TimeValue64 ): OSErr; external name '_QTSPresSetActiveSegment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetActiveSegment()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPresGetActiveSegment( inPresentation: QTSPresentation; inStream: QTSStream; var outStartTime: TimeValue64; var outDuration: TimeValue64 ): OSErr; external name '_QTSPresGetActiveSegment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPresGetActiveSegment(inPresentation: QTSPresentation; inStream: QTSStream; var outStartTime: TimeValue64; var outDuration: TimeValue64): OSErr; external name '_QTSPresGetActiveSegment';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetActiveSegment( inPresentation: QTSPresentation; inStream: QTSStream; var outStartTime: TimeValue64; var outDuration: TimeValue64 ): OSErr; external name '_QTSPresGetActiveSegment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetPlayHints()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetPlayHints( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFlagsMask: SInt32 ): OSErr; external name '_QTSPresSetPlayHints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetPlayHints(inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFlagsMask: SInt32): OSErr; external name '_QTSPresSetPlayHints';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetPlayHints( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFlagsMask: SInt32 ): OSErr; external name '_QTSPresSetPlayHints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetPlayHints()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetPlayHints( inPresentation: QTSPresentation; inStream: QTSStream; var outFlags: SInt32 ): OSErr; external name '_QTSPresGetPlayHints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetPlayHints(inPresentation: QTSPresentation; inStream: QTSStream; var outFlags: SInt32): OSErr; external name '_QTSPresGetPlayHints';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetPlayHints( inPresentation: QTSPresentation; inStream: QTSStream; var outFlags: SInt32 ): OSErr; external name '_QTSPresGetPlayHints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{-----------------------------------------
    Stream Spatial Functions
-----------------------------------------}
{
 *  QTSPresSetGWorld()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetGWorld( inPresentation: QTSPresentation; inStream: QTSStream; inGWorld: CGrafPtr; inGDHandle: GDHandle ): OSErr; external name '_QTSPresSetGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetGWorld(inPresentation: QTSPresentation; inStream: QTSStream; inGWorld: CGrafPtr; inGDHandle: GDHandle): OSErr; external name '_QTSPresSetGWorld';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetGWorld( inPresentation: QTSPresentation; inStream: QTSStream; inGWorld: CGrafPtr; inGDHandle: GDHandle ): OSErr; external name '_QTSPresSetGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetGWorld()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetGWorld( inPresentation: QTSPresentation; inStream: QTSStream; var outGWorld: CGrafPtr; var outGDHandle: GDHandle ): OSErr; external name '_QTSPresGetGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetGWorld(inPresentation: QTSPresentation; inStream: QTSStream; var outGWorld: CGrafPtr; var outGDHandle: GDHandle): OSErr; external name '_QTSPresGetGWorld';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetGWorld( inPresentation: QTSPresentation; inStream: QTSStream; var outGWorld: CGrafPtr; var outGDHandle: GDHandle ): OSErr; external name '_QTSPresGetGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetClip()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetClip( inPresentation: QTSPresentation; inStream: QTSStream; inClip: RgnHandle ): OSErr; external name '_QTSPresSetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetClip(inPresentation: QTSPresentation; inStream: QTSStream; inClip: RgnHandle): OSErr; external name '_QTSPresSetClip';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetClip( inPresentation: QTSPresentation; inStream: QTSStream; inClip: RgnHandle ): OSErr; external name '_QTSPresSetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetClip()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetClip( inPresentation: QTSPresentation; inStream: QTSStream; var outClip: RgnHandle ): OSErr; external name '_QTSPresGetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetClip(inPresentation: QTSPresentation; inStream: QTSStream; var outClip: RgnHandle): OSErr; external name '_QTSPresGetClip';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetClip( inPresentation: QTSPresentation; inStream: QTSStream; var outClip: RgnHandle ): OSErr; external name '_QTSPresGetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetMatrix()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetMatrix( inPresentation: QTSPresentation; inStream: QTSStream; const (*var*) inMatrix: MatrixRecord ): OSErr; external name '_QTSPresSetMatrix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetMatrix(inPresentation: QTSPresentation; inStream: QTSStream; const (*var*) inMatrix: MatrixRecord): OSErr; external name '_QTSPresSetMatrix';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetMatrix( inPresentation: QTSPresentation; inStream: QTSStream; const (*var*) inMatrix: MatrixRecord ): OSErr; external name '_QTSPresSetMatrix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetMatrix()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetMatrix( inPresentation: QTSPresentation; inStream: QTSStream; var outMatrix: MatrixRecord ): OSErr; external name '_QTSPresGetMatrix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetMatrix(inPresentation: QTSPresentation; inStream: QTSStream; var outMatrix: MatrixRecord): OSErr; external name '_QTSPresGetMatrix';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetMatrix( inPresentation: QTSPresentation; inStream: QTSStream; var outMatrix: MatrixRecord ): OSErr; external name '_QTSPresGetMatrix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetDimensions()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetDimensions( inPresentation: QTSPresentation; inStream: QTSStream; inWidth: Fixed; inHeight: Fixed ): OSErr; external name '_QTSPresSetDimensions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresGetDimensions()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetDimensions( inPresentation: QTSPresentation; inStream: QTSStream; var outWidth: Fixed; var outHeight: Fixed ): OSErr; external name '_QTSPresGetDimensions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresSetGraphicsMode()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetGraphicsMode( inPresentation: QTSPresentation; inStream: QTSStream; inMode: SInt16; const (*var*) inOpColor: RGBColor ): OSErr; external name '_QTSPresSetGraphicsMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresGetGraphicsMode()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetGraphicsMode( inPresentation: QTSPresentation; inStream: QTSStream; var outMode: SInt16; var outOpColor: RGBColor ): OSErr; external name '_QTSPresGetGraphicsMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresGetPicture()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetPicture( inPresentation: QTSPresentation; inStream: QTSStream; var outPicture: PicHandle ): OSErr; external name '_QTSPresGetPicture';
<<<<<<< HEAD
=======
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetPicture( inPresentation: QTSPresentation; inStream: QTSStream; var outPicture: PicHandle ): OSErr; external name '_QTSPresGetPicture';
=======
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetPicture( inPresentation: QTSPresentation; inStream: QTSStream; var outPicture: PicHandle ): OSErr; external name '_QTSPresGetPicture';
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresSetVisualContext()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetVisualContext( inPresentation: QTSPresentation; inStream: QTSStream; inVisualContext: QTVisualContextRef ): OSErr; external name '_QTSPresSetVisualContext';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  QTSPresSetVisualContext()
=======
 *  QTSPresGetVisualContext()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetVisualContext( inPresentation: QTSPresentation; inStream: QTSStream; inVisualContext: QTVisualContextRef ): OSErr; external name '_QTSPresSetVisualContext';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  QTSPresSetVisualContext()
=======
 *  QTSPresGetVisualContext()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetVisualContext( inPresentation: QTSPresentation; inStream: QTSStream; inVisualContext: QTVisualContextRef ): OSErr; external name '_QTSPresSetVisualContext';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  QTSPresSetVisualContext()
=======
 *  QTSPresGetVisualContext()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
<<<<<<< HEAD
 *    Windows:          in QTSClient.lib 4.0 and later
 }
=======
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetPicture( inPresentation: QTSPresentation; inStream: QTSStream; var outPicture: PicHandle ): OSErr; external name '_QTSPresGetPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresSetVisualContext()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
>>>>>>> graemeg/cpstrnew
function QTSPresSetVisualContext( inPresentation: QTSPresentation; inStream: QTSStream; inVisualContext: QTVisualContextRef ): OSErr; external name '_QTSPresSetVisualContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresGetVisualContext()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
=======
 *    Windows:          in QTSClient.lib 4.0 and later
 }
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function QTSPresGetVisualContext( inPresentation: QTSPresentation; inStream: QTSStream; var outVisualContext: QTVisualContextRef ): OSErr; external name '_QTSPresGetVisualContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetDimensions( inPresentation: QTSPresentation; inStream: QTSStream; inWidth: Fixed; inHeight: Fixed ): OSErr; external name '_QTSPresSetDimensions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresGetDimensions()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetDimensions( inPresentation: QTSPresentation; inStream: QTSStream; var outWidth: Fixed; var outHeight: Fixed ): OSErr; external name '_QTSPresGetDimensions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresSetGraphicsMode()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetGraphicsMode( inPresentation: QTSPresentation; inStream: QTSStream; inMode: SInt16; const (*var*) inOpColor: RGBColor ): OSErr; external name '_QTSPresSetGraphicsMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresGetGraphicsMode()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetGraphicsMode( inPresentation: QTSPresentation; inStream: QTSStream; var outMode: SInt16; var outOpColor: RGBColor ): OSErr; external name '_QTSPresGetGraphicsMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresGetPicture()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetPicture( inPresentation: QTSPresentation; inStream: QTSStream; var outPicture: PicHandle ): OSErr; external name '_QTSPresGetPicture';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresSetVisualContext()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetVisualContext( inPresentation: QTSPresentation; inStream: QTSStream; inVisualContext: QTVisualContextRef ): OSErr; external name '_QTSPresSetVisualContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSPresGetVisualContext()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetPicture(inPresentation: QTSPresentation; inStream: QTSStream; var outPicture: PicHandle): OSErr; external name '_QTSPresGetPicture';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetVisualContext( inPresentation: QTSPresentation; inStream: QTSStream; var outVisualContext: QTVisualContextRef ): OSErr; external name '_QTSPresGetVisualContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{-----------------------------------------
    Stream Sound Functions
-----------------------------------------}
{
 *  QTSPresSetVolumes()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresSetVolumes( inPresentation: QTSPresentation; inStream: QTSStream; inLeftVolume: SInt16; inRightVolume: SInt16 ): OSErr; external name '_QTSPresSetVolumes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresSetVolumes(inPresentation: QTSPresentation; inStream: QTSStream; inLeftVolume: SInt16; inRightVolume: SInt16): OSErr; external name '_QTSPresSetVolumes';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetVolumes( inPresentation: QTSPresentation; inStream: QTSStream; inLeftVolume: SInt16; inRightVolume: SInt16 ): OSErr; external name '_QTSPresSetVolumes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetVolumes()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSPresGetVolumes( inPresentation: QTSPresentation; inStream: QTSStream; var outLeftVolume: SInt16; var outRightVolume: SInt16 ): OSErr; external name '_QTSPresGetVolumes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSPresGetVolumes(inPresentation: QTSPresentation; inStream: QTSStream; var outLeftVolume: SInt16; var outRightVolume: SInt16): OSErr; external name '_QTSPresGetVolumes';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetVolumes( inPresentation: QTSPresentation; inStream: QTSStream; var outLeftVolume: SInt16; var outRightVolume: SInt16 ): OSErr; external name '_QTSPresGetVolumes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{-----------------------------------------
    Sourcing
-----------------------------------------}
{
 *  QTSPresGetSettingsAsText()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 }
function QTSPresGetSettingsAsText( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inSettingsType: OSType; var outText: Handle; inPanelFilterProc: QTSPanelFilterUPP; inPanelFilterProcRefCon: UnivPtr ): OSErr; external name '_QTSPresGetSettingsAsText';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
=======
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 }
<<<<<<< HEAD
function QTSPresGetSettingsAsText(inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inSettingsType: OSType; var outText: Handle; inPanelFilterProc: QTSPanelFilterUPP; inPanelFilterProcRefCon: UnivPtr): OSErr; external name '_QTSPresGetSettingsAsText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetSettingsAsText( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inSettingsType: OSType; var outText: Handle; inPanelFilterProc: QTSPanelFilterUPP; inPanelFilterProcRefCon: UnivPtr ): OSErr; external name '_QTSPresGetSettingsAsText';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSettingsDialog()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSPresSettingsDialog( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFilterProc: QTSModalFilterUPP; inFilterProcRefCon: UnivPtr ): OSErr; external name '_QTSPresSettingsDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresSettingsDialog(inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFilterProc: QTSModalFilterUPP; inFilterProcRefCon: UnivPtr): OSErr; external name '_QTSPresSettingsDialog';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSettingsDialog( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFilterProc: QTSModalFilterUPP; inFilterProcRefCon: UnivPtr ): OSErr; external name '_QTSPresSettingsDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSettingsDialogWithFilters()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 }
function QTSPresSettingsDialogWithFilters( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFilterProc: QTSModalFilterUPP; inFilterProcRefCon: UnivPtr; inPanelFilterProc: QTSPanelFilterUPP; inPanelFilterProcRefCon: UnivPtr ): OSErr; external name '_QTSPresSettingsDialogWithFilters';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
=======
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 }
<<<<<<< HEAD
function QTSPresSettingsDialogWithFilters(inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFilterProc: QTSModalFilterUPP; inFilterProcRefCon: UnivPtr; inPanelFilterProc: QTSPanelFilterUPP; inPanelFilterProcRefCon: UnivPtr): OSErr; external name '_QTSPresSettingsDialogWithFilters';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSettingsDialogWithFilters( inPresentation: QTSPresentation; inStream: QTSStream; inFlags: SInt32; inFilterProc: QTSModalFilterUPP; inFilterProcRefCon: UnivPtr; inPanelFilterProc: QTSPanelFilterUPP; inPanelFilterProcRefCon: UnivPtr ): OSErr; external name '_QTSPresSettingsDialogWithFilters';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresSetSettings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSPresSetSettings( inPresentation: QTSPresentation; inStream: QTSStream; inSettings: QTAtomSpecPtr; inFlags: SInt32 ): OSErr; external name '_QTSPresSetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresSetSettings(inPresentation: QTSPresentation; inStream: QTSStream; inSettings: QTAtomSpecPtr; inFlags: SInt32): OSErr; external name '_QTSPresSetSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresSetSettings( inPresentation: QTSPresentation; inStream: QTSStream; inSettings: QTAtomSpecPtr; inFlags: SInt32 ): OSErr; external name '_QTSPresSetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetSettings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSPresGetSettings( inPresentation: QTSPresentation; inStream: QTSStream; var outSettings: QTAtomContainer; inFlags: SInt32 ): OSErr; external name '_QTSPresGetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresGetSettings(inPresentation: QTSPresentation; inStream: QTSStream; var outSettings: QTAtomContainer; inFlags: SInt32): OSErr; external name '_QTSPresGetSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetSettings( inPresentation: QTSPresentation; inStream: QTSStream; var outSettings: QTAtomContainer; inFlags: SInt32 ): OSErr; external name '_QTSPresGetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresAddSourcer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSPresAddSourcer( inPresentation: QTSPresentation; inStream: QTSStream; inSourcer: ComponentInstance; inFlags: SInt32 ): OSErr; external name '_QTSPresAddSourcer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresAddSourcer(inPresentation: QTSPresentation; inStream: QTSStream; inSourcer: ComponentInstance; inFlags: SInt32): OSErr; external name '_QTSPresAddSourcer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresAddSourcer( inPresentation: QTSPresentation; inStream: QTSStream; inSourcer: ComponentInstance; inFlags: SInt32 ): OSErr; external name '_QTSPresAddSourcer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresRemoveSourcer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSPresRemoveSourcer( inPresentation: QTSPresentation; inStream: QTSStream; inSourcer: ComponentInstance; inFlags: SInt32 ): OSErr; external name '_QTSPresRemoveSourcer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresRemoveSourcer(inPresentation: QTSPresentation; inStream: QTSStream; inSourcer: ComponentInstance; inFlags: SInt32): OSErr; external name '_QTSPresRemoveSourcer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresRemoveSourcer( inPresentation: QTSPresentation; inStream: QTSStream; inSourcer: ComponentInstance; inFlags: SInt32 ): OSErr; external name '_QTSPresRemoveSourcer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetNumSourcers()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSPresGetNumSourcers( inPresentation: QTSPresentation; inStream: QTSStream ): UInt32; external name '_QTSPresGetNumSourcers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresGetNumSourcers(inPresentation: QTSPresentation; inStream: QTSStream): UInt32; external name '_QTSPresGetNumSourcers';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetNumSourcers( inPresentation: QTSPresentation; inStream: QTSStream ): UInt32; external name '_QTSPresGetNumSourcers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPresGetIndSourcer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSPresGetIndSourcer( inPresentation: QTSPresentation; inStream: QTSStream; inIndex: UInt32; var outSourcer: ComponentInstance ): OSErr; external name '_QTSPresGetIndSourcer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
<<<<<<< HEAD
function QTSPresGetIndSourcer(inPresentation: QTSPresentation; inStream: QTSStream; inIndex: UInt32; var outSourcer: ComponentInstance): OSErr; external name '_QTSPresGetIndSourcer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPresGetIndSourcer( inPresentation: QTSPresentation; inStream: QTSStream; inIndex: UInt32; var outSourcer: ComponentInstance ): OSErr; external name '_QTSPresGetIndSourcer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{============================================================================
        Misc
============================================================================}
{ flags for Get/SetNetworkAppName }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kQTSNetworkAppNameIsFullNameFlag = $00000001;

{
 *  QTSSetNetworkAppName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSSetNetworkAppName( inAppName: ConstCStringPtr; inFlags: SInt32 ): OSErr; external name '_QTSSetNetworkAppName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2

const
	kQTSNetworkAppNameIsFullNameFlag = $00000001;

	{
	 *  QTSSetNetworkAppName()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.1 and later
	 	}
function QTSSetNetworkAppName(inAppName: ConstCStringPtr; inFlags: SInt32): OSErr; external name '_QTSSetNetworkAppName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kQTSNetworkAppNameIsFullNameFlag = $00000001;

{
 *  QTSSetNetworkAppName()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSSetNetworkAppName( inAppName: ConstCStringPtr; inFlags: SInt32 ): OSErr; external name '_QTSSetNetworkAppName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSGetNetworkAppName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSGetNetworkAppName( inFlags: SInt32; var outCStringPtr: CStringPtr ): OSErr; external name '_QTSGetNetworkAppName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSGetNetworkAppName(inFlags: SInt32; var outCStringPtr: CStringPtr): OSErr; external name '_QTSGetNetworkAppName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSGetNetworkAppName( inFlags: SInt32; var outCStringPtr: CStringPtr ): OSErr; external name '_QTSGetNetworkAppName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4

{-----------------------------------------
    Statistics Utilities
-----------------------------------------}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	QTSStatHelperRecordPtr = ^QTSStatHelperRecord;
	QTSStatHelperRecord = record
		data: array [0..0] of SInt32;
	end;
type
	QTSStatHelper = QTSStatHelperRecordPtr;
const
	kQTSInvalidStatHelper = 0;

{ flags for QTSStatHelperNextParams }
const
	kQTSStatHelperReturnPascalStringsFlag = $00000001;

type
	QTSStatHelperNextParamsPtr = ^QTSStatHelperNextParams;
	QTSStatHelperNextParams = record
		flags: SInt32;
		returnedStatisticsType: OSType;
		returnedStream: QTSStream;
		maxStatNameLength: UInt32;
		returnedStatName: CStringPtr;       { NULL if you don't want it}
		maxStatStringLength: UInt32;
		returnedStatString: CStringPtr;     { NULL if you don't want it}
		maxStatUnitLength: UInt32;
		returnedStatUnit: CStringPtr;       { NULL if you don't want it}
	end;
type
	QTSStatisticsParamsPtr = ^QTSStatisticsParams;
	QTSStatisticsParams = record
		statisticsType: OSType;
		container: QTAtomContainer;
		parentAtom: QTAtom;
		flags: SInt32;
	end;
{ general statistics types }
const
	kQTSAllStatisticsType = FourCharCode('all ');
	kQTSShortStatisticsType = FourCharCode('shrt');
	kQTSSummaryStatisticsType = FourCharCode('summ');

{ statistics flags }
const
	kQTSGetNameStatisticsFlag = $00000001;
	kQTSDontGetDataStatisticsFlag = $00000002;
	kQTSUpdateAtomsStatisticsFlag = $00000004;
	kQTSGetUnitsStatisticsFlag = $00000008;
	kQTSUpdateAllIfNecessaryStatisticsFlag = $00010000;

{ statistics atom types }
const
	kQTSStatisticsStreamAtomType = FourCharCode('strm');
	kQTSStatisticsNameAtomType = FourCharCode('name'); { chars only, no length or terminator }
	kQTSStatisticsDataFormatAtomType = FourCharCode('frmt'); { OSType }
	kQTSStatisticsDataAtomType = FourCharCode('data');
	kQTSStatisticsUnitsAtomType = FourCharCode('unit'); { OSType }
	kQTSStatisticsUnitsNameAtomType = FourCharCode('unin'); { chars only, no length or terminator }

{ statistics data formats }
const
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
type
	QTSStatHelperRecordPtr = ^QTSStatHelperRecord;
	QTSStatHelperRecord = record
		data: array [0..0] of SInt32;
	end;
type
	QTSStatHelper = QTSStatHelperRecordPtr;
const
	kQTSInvalidStatHelper = 0;

{ flags for QTSStatHelperNextParams }
const
	kQTSStatHelperReturnPascalStringsFlag = $00000001;

type
	QTSStatHelperNextParamsPtr = ^QTSStatHelperNextParams;
	QTSStatHelperNextParams = record
		flags: SInt32;
		returnedStatisticsType: OSType;
		returnedStream: QTSStream;
		maxStatNameLength: UInt32;
		returnedStatName: CStringPtr;       { NULL if you don't want it}
		maxStatStringLength: UInt32;
		returnedStatString: CStringPtr;     { NULL if you don't want it}
		maxStatUnitLength: UInt32;
		returnedStatUnit: CStringPtr;       { NULL if you don't want it}
	end;
type
	QTSStatisticsParamsPtr = ^QTSStatisticsParams;
	QTSStatisticsParams = record
		statisticsType: OSType;
		container: QTAtomContainer;
		parentAtom: QTAtom;
		flags: SInt32;
	end;
{ general statistics types }
const
	kQTSAllStatisticsType = FourCharCode('all ');
	kQTSShortStatisticsType = FourCharCode('shrt');
	kQTSSummaryStatisticsType = FourCharCode('summ');

{ statistics flags }
const
	kQTSGetNameStatisticsFlag = $00000001;
	kQTSDontGetDataStatisticsFlag = $00000002;
	kQTSUpdateAtomsStatisticsFlag = $00000004;
	kQTSGetUnitsStatisticsFlag = $00000008;
	kQTSUpdateAllIfNecessaryStatisticsFlag = $00010000;

{ statistics atom types }
const
	kQTSStatisticsStreamAtomType = FourCharCode('strm');
	kQTSStatisticsNameAtomType = FourCharCode('name'); { chars only, no length or terminator }
	kQTSStatisticsDataFormatAtomType = FourCharCode('frmt'); { OSType }
	kQTSStatisticsDataAtomType = FourCharCode('data');
	kQTSStatisticsUnitsAtomType = FourCharCode('unit'); { OSType }
	kQTSStatisticsUnitsNameAtomType = FourCharCode('unin'); { chars only, no length or terminator }

<<<<<<< HEAD
	{	 statistics data formats 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ statistics data formats }
const
>>>>>>> origin/fixes_2.4
	kQTSStatisticsSInt32DataFormat = FourCharCode('si32');
	kQTSStatisticsUInt32DataFormat = FourCharCode('ui32');
	kQTSStatisticsSInt16DataFormat = FourCharCode('si16');
	kQTSStatisticsUInt16DataFormat = FourCharCode('ui16');
	kQTSStatisticsFixedDataFormat = FourCharCode('fixd');
	kQTSStatisticsUnsignedFixedDataFormat = FourCharCode('ufix');
	kQTSStatisticsStringDataFormat = FourCharCode('strg');
	kQTSStatisticsOSTypeDataFormat = FourCharCode('ostp');
	kQTSStatisticsRectDataFormat = FourCharCode('rect');
	kQTSStatisticsPointDataFormat = FourCharCode('pont');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ statistics units types }
const
	kQTSStatisticsNoUnitsType = 0;
=======
	{	 statistics units types 	}
	kQTSStatisticsNoUnitsType	= 0;
>>>>>>> graemeg/fixes_2_2
=======
	{	 statistics units types 	}
	kQTSStatisticsNoUnitsType	= 0;
>>>>>>> origin/fixes_2_2
=======
{ statistics units types }
const
	kQTSStatisticsNoUnitsType = 0;
>>>>>>> origin/fixes_2.4
	kQTSStatisticsPercentUnitsType = FourCharCode('pcnt');
	kQTSStatisticsBitsPerSecUnitsType = FourCharCode('bps ');
	kQTSStatisticsFramesPerSecUnitsType = FourCharCode('fps ');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{ specific statistics types }
const
	kQTSTotalDataRateStat = FourCharCode('drtt');
	kQTSTotalDataRateInStat = FourCharCode('drti');
	kQTSTotalDataRateOutStat = FourCharCode('drto');
	kQTSNetworkIDStringStat = FourCharCode('nids');

{
 *  QTSNewStatHelper()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSNewStatHelper( inPresentation: QTSPresentation; inStream: QTSStream; inStatType: OSType; inFlags: SInt32; var outStatHelper: QTSStatHelper ): OSErr; external name '_QTSNewStatHelper';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD

=======
=======
>>>>>>> origin/fixes_2_2
	{	 specific statistics types 	}
	kQTSTotalDataRateStat		= FourCharCode('drtt');
	kQTSTotalDataRateInStat		= FourCharCode('drti');
	kQTSTotalDataRateOutStat	= FourCharCode('drto');
	kQTSNetworkIDStringStat		= FourCharCode('nids');

	{
	 *  QTSNewStatHelper()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.0 and later
	 	}
function QTSNewStatHelper(inPresentation: QTSPresentation; inStream: QTSStream; inStatType: OSType; inFlags: SInt32; var outStatHelper: QTSStatHelper): OSErr; external name '_QTSNewStatHelper';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4

{
 *  QTSDisposeStatHelper()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSDisposeStatHelper( inStatHelper: QTSStatHelper ): OSErr; external name '_QTSDisposeStatHelper';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSDisposeStatHelper(inStatHelper: QTSStatHelper): OSErr; external name '_QTSDisposeStatHelper';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSDisposeStatHelper( inStatHelper: QTSStatHelper ): OSErr; external name '_QTSDisposeStatHelper';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSStatHelperGetStats()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSStatHelperGetStats( inStatHelper: QTSStatHelper ): OSErr; external name '_QTSStatHelperGetStats';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSStatHelperGetStats(inStatHelper: QTSStatHelper): OSErr; external name '_QTSStatHelperGetStats';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSStatHelperGetStats( inStatHelper: QTSStatHelper ): OSErr; external name '_QTSStatHelperGetStats';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSStatHelperResetIter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSStatHelperResetIter( inStatHelper: QTSStatHelper ): OSErr; external name '_QTSStatHelperResetIter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSStatHelperResetIter(inStatHelper: QTSStatHelper): OSErr; external name '_QTSStatHelperResetIter';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSStatHelperResetIter( inStatHelper: QTSStatHelper ): OSErr; external name '_QTSStatHelperResetIter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSStatHelperNext()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSStatHelperNext( inStatHelper: QTSStatHelper; var ioParams: QTSStatHelperNextParams ): Boolean; external name '_QTSStatHelperNext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSStatHelperNext(inStatHelper: QTSStatHelper; var ioParams: QTSStatHelperNextParams): boolean; external name '_QTSStatHelperNext';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSStatHelperNext( inStatHelper: QTSStatHelper; var ioParams: QTSStatHelperNextParams ): Boolean; external name '_QTSStatHelperNext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSStatHelperGetNumStats()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSStatHelperGetNumStats( inStatHelper: QTSStatHelper ): UInt32; external name '_QTSStatHelperGetNumStats';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSStatHelperGetNumStats(inStatHelper: QTSStatHelper): UInt32; external name '_QTSStatHelperGetNumStats';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSStatHelperGetNumStats( inStatHelper: QTSStatHelper ): UInt32; external name '_QTSStatHelperGetNumStats';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ used by components to put statistics into the atom container }
{
 *  QTSGetOrMakeStatAtomForStream()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSGetOrMakeStatAtomForStream( inContainer: QTAtomContainer; inStream: QTSStream; var outParentAtom: QTAtom ): OSErr; external name '_QTSGetOrMakeStatAtomForStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSGetOrMakeStatAtomForStream(inContainer: QTAtomContainer; inStream: QTSStream; var outParentAtom: QTAtom): OSErr; external name '_QTSGetOrMakeStatAtomForStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSGetOrMakeStatAtomForStream( inContainer: QTAtomContainer; inStream: QTSStream; var outParentAtom: QTAtom ): OSErr; external name '_QTSGetOrMakeStatAtomForStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSInsertStatistic()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSInsertStatistic( inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inStatData: UnivPtr; inStatDataLength: UInt32; inStatDataFormat: OSType; inFlags: SInt32 ): OSErr; external name '_QTSInsertStatistic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSInsertStatistic(inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inStatData: UnivPtr; inStatDataLength: UInt32; inStatDataFormat: OSType; inFlags: SInt32): OSErr; external name '_QTSInsertStatistic';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSInsertStatistic( inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inStatData: UnivPtr; inStatDataLength: UInt32; inStatDataFormat: OSType; inFlags: SInt32 ): OSErr; external name '_QTSInsertStatistic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSInsertStatisticName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSInsertStatisticName( inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inStatName: ConstCStringPtr; inStatNameLength: UInt32 ): OSErr; external name '_QTSInsertStatisticName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSInsertStatisticName(inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inStatName: ConstCStringPtr; inStatNameLength: UInt32): OSErr; external name '_QTSInsertStatisticName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

{
 *  QTSInsertStatisticUnits()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSInsertStatisticUnits( inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inUnitsType: OSType; inUnitsName: ConstCStringPtr; inUnitsNameLength: UInt32 ): OSErr; external name '_QTSInsertStatisticUnits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSInsertStatisticName( inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inStatName: ConstCStringPtr; inStatNameLength: UInt32 ): OSErr; external name '_QTSInsertStatisticName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSInsertStatisticUnits()
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSInsertStatisticUnits(inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inUnitsType: OSType; inUnitsName: ConstCStringPtr; inUnitsNameLength: UInt32): OSErr; external name '_QTSInsertStatisticUnits';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSInsertStatisticUnits( inContainer: QTAtomContainer; inParentAtom: QTAtom; inStatType: OSType; inUnitsType: OSType; inUnitsName: ConstCStringPtr; inUnitsNameLength: UInt32 ): OSErr; external name '_QTSInsertStatisticUnits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{============================================================================
        Data Formats
============================================================================}
{-----------------------------------------
    Data Types
-----------------------------------------}
{ universal data types }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kQTSNullDataType = FourCharCode('NULL');
	kQTSUnknownDataType = FourCharCode('huh?');
	kQTSAtomContainerDataType = FourCharCode('qtac'); { QTAtomContainer }
	kQTSAtomDataType = FourCharCode('qtat'); { QTSAtomContainerDataStruct* }
	kQTSAliasDataType = FourCharCode('alis');
	kQTSFileDataType = FourCharCode('fspc'); { FSSpec* }
	kQTSFileSpecDataType = FourCharCode('fspc'); { FSSpec* }
	kQTSHandleDataType = FourCharCode('hndl'); { Handle* }
	kQTSDataRefDataType = FourCharCode('dref'); { DataReferencePtr }

{ these data types are specific to presentations }
const
	kQTSRTSPDataType = FourCharCode('rtsp');
	kQTSSDPDataType = FourCharCode('sdp ');

{-----------------------------------------
    Atom IDs
-----------------------------------------}
const
	kQTSAtomType_Presentation = FourCharCode('pres');
	kQTSAtomType_PresentationHeader = FourCharCode('phdr'); { QTSPresentationHeaderAtom }
	kQTSAtomType_MediaStream = FourCharCode('mstr');
	kQTSAtomType_MediaStreamHeader = FourCharCode('mshd'); { QTSMediaStreamHeaderAtom }
	kQTSAtomType_MediaDescriptionText = FourCharCode('mdes'); { chars, no length }
	kQTSAtomType_ClipRect = FourCharCode('clip'); { QTSClipRectAtom }
	kQTSAtomType_Duration = FourCharCode('dura'); { QTSDurationAtom }
	kQTSAtomType_BufferTime = FourCharCode('bufr'); { QTSBufferTimeAtom }
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
const
	kQTSNullDataType = FourCharCode('NULL');
	kQTSUnknownDataType = FourCharCode('huh?');
	kQTSAtomContainerDataType = FourCharCode('qtac'); { QTAtomContainer }
	kQTSAtomDataType = FourCharCode('qtat'); { QTSAtomContainerDataStruct* }
	kQTSAliasDataType = FourCharCode('alis');
	kQTSFileDataType = FourCharCode('fspc'); { FSSpec* }
	kQTSFileSpecDataType = FourCharCode('fspc'); { FSSpec* }
	kQTSHandleDataType = FourCharCode('hndl'); { Handle* }
	kQTSDataRefDataType = FourCharCode('dref'); { DataReferencePtr }

{ these data types are specific to presentations }
const
	kQTSRTSPDataType = FourCharCode('rtsp');
	kQTSSDPDataType = FourCharCode('sdp ');

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{-----------------------------------------
    Atom IDs
-----------------------------------------}
const
	kQTSAtomType_Presentation = FourCharCode('pres');
	kQTSAtomType_PresentationHeader = FourCharCode('phdr'); { QTSPresentationHeaderAtom }
	kQTSAtomType_MediaStream = FourCharCode('mstr');
	kQTSAtomType_MediaStreamHeader = FourCharCode('mshd'); { QTSMediaStreamHeaderAtom }
	kQTSAtomType_MediaDescriptionText = FourCharCode('mdes'); { chars, no length }
	kQTSAtomType_ClipRect = FourCharCode('clip'); { QTSClipRectAtom }
	kQTSAtomType_Duration = FourCharCode('dura'); { QTSDurationAtom }
	kQTSAtomType_BufferTime = FourCharCode('bufr'); { QTSBufferTimeAtom }
>>>>>>> origin/fixes_2.4

type
	QTSAtomContainerDataStructPtr = ^QTSAtomContainerDataStruct;
	QTSAtomContainerDataStruct = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		container: QTAtomContainer;
		parentAtom: QTAtom;
	end;
{ flags for QTSPresentationHeaderAtom }
const
	kQTSPresHeaderTypeIsData = $00000100;
	kQTSPresHeaderDataIsHandle = $00000200;
=======
=======
>>>>>>> origin/fixes_2_2
		container:				QTAtomContainer;
		parentAtom:				QTAtom;
=======
		container: QTAtomContainer;
		parentAtom: QTAtom;
>>>>>>> origin/fixes_2.4
	end;
{ flags for QTSPresentationHeaderAtom }
const
<<<<<<< HEAD
	kQTSPresHeaderTypeIsData	= $00000100;
	kQTSPresHeaderDataIsHandle	= $00000200;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSPresHeaderTypeIsData = $00000100;
	kQTSPresHeaderDataIsHandle = $00000200;
>>>>>>> origin/fixes_2.4

type
	QTSPresentationHeaderAtomPtr = ^QTSPresentationHeaderAtom;
	QTSPresentationHeaderAtom = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		versionAndFlags: SInt32;
		conductorOrDataType: OSType;
		dataAtomType: OSType;           { where the data really is}
	end;
type
	QTSMediaStreamHeaderAtomPtr = ^QTSMediaStreamHeaderAtom;
	QTSMediaStreamHeaderAtom = record
		versionAndFlags: SInt32;
		mediaTransportType: OSType;
		mediaTransportDataAID: OSType;  { where the data really is}
	end;
type
	QTSBufferTimeAtomPtr = ^QTSBufferTimeAtom;
	QTSBufferTimeAtom = record
		versionAndFlags: SInt32;
		bufferTime: Fixed;
	end;
type
	QTSDurationAtomPtr = ^QTSDurationAtom;
	QTSDurationAtom = record
		versionAndFlags: SInt32;
		timeScale: TimeScale_fix;
		duration: TimeValue64;
	end;
type
	QTSClipRectAtomPtr = ^QTSClipRectAtom;
	QTSClipRectAtom = record
		versionAndFlags: SInt32;
		clipRect: Rect;
	end;
=======
=======
>>>>>>> origin/fixes_2_2
		versionAndFlags:		SInt32;
		conductorOrDataType:	OSType;
		dataAtomType:			OSType;									{  where the data really is }
=======
		versionAndFlags: SInt32;
		conductorOrDataType: OSType;
		dataAtomType: OSType;           { where the data really is}
>>>>>>> origin/fixes_2.4
	end;
type
	QTSMediaStreamHeaderAtomPtr = ^QTSMediaStreamHeaderAtom;
	QTSMediaStreamHeaderAtom = record
		versionAndFlags: SInt32;
		mediaTransportType: OSType;
		mediaTransportDataAID: OSType;  { where the data really is}
	end;
type
	QTSBufferTimeAtomPtr = ^QTSBufferTimeAtom;
	QTSBufferTimeAtom = record
		versionAndFlags: SInt32;
		bufferTime: Fixed;
	end;
type
	QTSDurationAtomPtr = ^QTSDurationAtom;
	QTSDurationAtom = record
		versionAndFlags: SInt32;
		timeScale: TimeScale_fix;
		duration: TimeValue64;
	end;
type
	QTSClipRectAtomPtr = ^QTSClipRectAtom;
	QTSClipRectAtom = record
		versionAndFlags: SInt32;
		clipRect: Rect;
	end;
<<<<<<< HEAD


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kQTSEmptyEditStreamStartTime = -1;


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	QTSStatus = UInt32;
const
	kQTSNullStatus = 0;
	kQTSUninitializedStatus = 1;
	kQTSConnectingStatus = 2;
	kQTSOpeningConnectionDetailedStatus = 3;
	kQTSMadeConnectionDetailedStatus = 4;
	kQTSNegotiatingStatus = 5;
=======
=======
>>>>>>> origin/fixes_2_2
	QTSStatus							= UInt32;

=======
	QTSStatus = UInt32;
>>>>>>> origin/fixes_2.4
const
	kQTSNullStatus = 0;
	kQTSUninitializedStatus = 1;
	kQTSConnectingStatus = 2;
	kQTSOpeningConnectionDetailedStatus = 3;
	kQTSMadeConnectionDetailedStatus = 4;
<<<<<<< HEAD
	kQTSNegotiatingStatus		= 5;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSNegotiatingStatus = 5;
>>>>>>> origin/fixes_2.4
	kQTSGettingDescriptionDetailedStatus = 6;
	kQTSGotDescriptionDetailedStatus = 7;
	kQTSSentSetupCmdDetailedStatus = 8;
	kQTSReceivedSetupResponseDetailedStatus = 9;
	kQTSSentPlayCmdDetailedStatus = 10;
	kQTSReceivedPlayResponseDetailedStatus = 11;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kQTSBufferingStatus = 12;
	kQTSPlayingStatus = 13;
	kQTSPausedStatus = 14;
	kQTSAutoConfiguringStatus = 15;
	kQTSDownloadingStatus = 16;
	kQTSBufferingWithTimeStatus = 17;
	kQTSWaitingDisconnectStatus = 100;

{-----------------------------------------
    QuickTime Preferences Types
-----------------------------------------}
const
	kQTSConnectionPrefsType = FourCharCode('stcm'); { root atom that all other atoms are contained in}
                                        {    kQTSNotUsedForProxyPrefsType = 'nopr',     //        comma-delimited list of URLs that are never used for proxies}
	kQTSConnectionMethodPrefsType = FourCharCode('mthd'); {      connection method (OSType that matches one of the following three)}
	kQTSDirectConnectPrefsType = FourCharCode('drct'); {       used if direct connect (QTSDirectConnectPrefsRecord)}
                                        {    kQTSRTSPProxyPrefsType =     'rtsp',   //   used if RTSP Proxy (QTSProxyPrefsRecord)}
	kQTSSOCKSPrefsType = FourCharCode('sock'); {       used if SOCKS Proxy (QTSProxyPrefsRecord)}
<<<<<<< HEAD

const
	kQTSDirectConnectHTTPProtocol = FourCharCode('http');
	kQTSDirectConnectRTSPProtocol = FourCharCode('rtsp');

type
	QTSDirectConnectPrefsRecordPtr = ^QTSDirectConnectPrefsRecord;
	QTSDirectConnectPrefsRecord = record
		tcpPortID: UInt32;
		protocol: OSType;
	end;
type
	QTSProxyPrefsRecordPtr = ^QTSProxyPrefsRecord;
	QTSProxyPrefsRecord = record
		serverNameStr: Str255;
		portID: UInt32;
	end;
const
  kQTSTransAndProxyPrefsVersNum = 2;       { prefs atom format version }
const
	kConnectionActive = 1 shl 0;
	kConnectionUseSystemPref = 1 shl 1;
=======
=======
>>>>>>> origin/fixes_2_2
	kQTSBufferingStatus			= 12;
	kQTSPlayingStatus			= 13;
	kQTSPausedStatus			= 14;
	kQTSAutoConfiguringStatus	= 15;
	kQTSDownloadingStatus		= 16;
	kQTSBufferingWithTimeStatus	= 17;
	kQTSWaitingDisconnectStatus	= 100;

	{	-----------------------------------------
	    QuickTime Preferences Types
	-----------------------------------------	}
	kQTSConnectionPrefsType		= FourCharCode('stcm');						{  root atom that all other atoms are contained in }
																{     kQTSNotUsedForProxyPrefsType = 'nopr',     //        comma-delimited list of URLs that are never used for proxies }
	kQTSConnectionMethodPrefsType = FourCharCode('mthd');						{       connection method (OSType that matches one of the following three) }
	kQTSDirectConnectPrefsType	= FourCharCode('drct');						{        used if direct connect (QTSDirectConnectPrefsRecord) }
																{     kQTSRTSPProxyPrefsType =     'rtsp',   //   used if RTSP Proxy (QTSProxyPrefsRecord) }
	kQTSSOCKSPrefsType			= FourCharCode('sock');						{        used if SOCKS Proxy (QTSProxyPrefsRecord) }
=======
>>>>>>> origin/fixes_2.4

const
	kQTSDirectConnectHTTPProtocol = FourCharCode('http');
	kQTSDirectConnectRTSPProtocol = FourCharCode('rtsp');

type
	QTSDirectConnectPrefsRecordPtr = ^QTSDirectConnectPrefsRecord;
	QTSDirectConnectPrefsRecord = record
		tcpPortID: UInt32;
		protocol: OSType;
	end;
type
	QTSProxyPrefsRecordPtr = ^QTSProxyPrefsRecord;
	QTSProxyPrefsRecord = record
		serverNameStr: Str255;
		portID: UInt32;
	end;
const
<<<<<<< HEAD
	kConnectionActive			= $00000001;
	kConnectionUseSystemPref	= $00000002;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  kQTSTransAndProxyPrefsVersNum = 2;       { prefs atom format version }
const
	kConnectionActive = 1 shl 0;
	kConnectionUseSystemPref = 1 shl 1;
>>>>>>> origin/fixes_2.4

type
	QTSTransportPrefPtr = ^QTSTransportPref;
	QTSTransportPref = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		protocol: OSType;               { udp, http, tcp, etc}
		portID: SInt32;                 { port to use for this connection type}
		flags: UInt32;                  { connection flags}
		seed: UInt32;                   { seed value last time this setting was read from system prefs}
<<<<<<< HEAD
	end;
const
	kProxyActive = 1 shl 0;
	kProxyUseSystemPref = 1 shl 1;
=======
=======
>>>>>>> origin/fixes_2_2
		protocol:				OSType;									{  udp, http, tcp, etc }
		portID:					SInt32;									{  port to use for this connection type }
		flags:					UInt32;									{  connection flags }
		seed:					UInt32;									{  seed value last time this setting was read from system prefs }
=======
>>>>>>> origin/fixes_2.4
	end;
const
<<<<<<< HEAD
	kProxyActive				= $00000001;
	kProxyUseSystemPref			= $00000002;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kProxyActive = 1 shl 0;
	kProxyUseSystemPref = 1 shl 1;
>>>>>>> origin/fixes_2.4

type
	QTSProxyPrefPtr = ^QTSProxyPref;
	QTSProxyPref = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		flags: UInt32;                  { proxy flags}
		portID: SInt32;                 { port to use for this connection type}
		seed: UInt32;                   { seed value last time this setting was read from system prefs}
		serverNameStr: Str255;          { proxy server url}
<<<<<<< HEAD
	end;
const
	kNoProxyUseSystemPref = 1 shl 0;
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					UInt32;									{  proxy flags }
		portID:					SInt32;									{  port to use for this connection type }
		seed:					UInt32;									{  seed value last time this setting was read from system prefs }
		serverNameStr:			Str255;									{  proxy server url }
=======
>>>>>>> origin/fixes_2.4
	end;
const
<<<<<<< HEAD
	kNoProxyUseSystemPref		= $00000001;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kNoProxyUseSystemPref = 1 shl 0;
>>>>>>> origin/fixes_2.4

type
	QTSNoProxyPrefPtr = ^QTSNoProxyPref;
	QTSNoProxyPref = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: UInt32;                  { no-proxy flags}
		seed: UInt32;                   { seed value last time this setting was read from system prefs}
    urlList: array [0..0] of SInt8;             { NULL terminated, comma delimited list of urls}
	end;
const
	kQTSInstantOnFlag_Enable = 1 shl 0; { instant on is enabled (read/write)}
	kQTSInstantOnFlag_Permitted = 1 shl 1; { instant on is possible (read only)}
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					UInt32;									{  no-proxy flags }
		seed:					UInt32;									{  seed value last time this setting was read from system prefs }
		urlList:				SInt8;									{  NULL terminated, comma delimited list of urls }
=======
		flags: UInt32;                  { no-proxy flags}
		seed: UInt32;                   { seed value last time this setting was read from system prefs}
    urlList: array [0..0] of SInt8;             { NULL terminated, comma delimited list of urls}
>>>>>>> origin/fixes_2.4
	end;
const
<<<<<<< HEAD
	kQTSInstantOnFlag_Enable	= $00000001;					{  instant on is enabled (read/write) }
	kQTSInstantOnFlag_Permitted	= $00000002;					{  instant on is possible (read only) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSInstantOnFlag_Enable = 1 shl 0; { instant on is enabled (read/write)}
	kQTSInstantOnFlag_Permitted = 1 shl 1; { instant on is possible (read only)}
>>>>>>> origin/fixes_2.4


type
	QTSInstantOnPrefPtr = ^QTSInstantOnPref;
	QTSInstantOnPref = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: SInt32;                  { flags}
		factor: SInt32;                 {    0-100; default is 50}
	end;
const
	kQTSTransAndProxyAtomType = FourCharCode('strp'); { transport/proxy prefs root atom}
	kQTSConnectionPrefsVersion = FourCharCode('vers'); {   prefs format version}
	kQTSTransportPrefsAtomType = FourCharCode('trns'); {   tranport prefs root atom}
	kQTSConnectionAtomType = FourCharCode('conn'); {     connection prefs atom type, one for each transport type}
	kQTSUDPTransportType = FourCharCode('udp '); {     udp transport prefs}
	kQTSHTTPTransportType = FourCharCode('http'); {     http transport prefs}
	kQTSTCPTransportType = FourCharCode('tcp '); {     tcp transport prefs    }
	kQTSProxyPrefsAtomType = FourCharCode('prxy'); {   proxy prefs root atom}
	kQTSHTTPProxyPrefsType = FourCharCode('http'); {     http proxy settings}
	kQTSRTSPProxyPrefsType = FourCharCode('rtsp'); {     rtsp proxy settings}
	kQTSSOCKSProxyPrefsType = FourCharCode('sock'); {     socks proxy settings}
	kQTSProxyUserInfoPrefsType = FourCharCode('user'); {   proxy username/password root atom}
	kQTSDontProxyPrefsAtomType = FourCharCode('nopr'); {   no-proxy prefs root atom}
	kQTSDontProxyDataType = FourCharCode('data'); {     no proxy settings}
	kQTSInstantOnPrefsAtomType = FourCharCode('inon'); { instant on prefs}

{
 *  QTSPrefsAddProxySetting()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPrefsAddProxySetting( proxyType: OSType; portID: SInt32; flags: UInt32; seed: UInt32; var srvrURL: Str255 ): OSErr; external name '_QTSPrefsAddProxySetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;									{  flags }
		factor:					SInt32;									{     0-100; default is 50 }
=======
		flags: SInt32;                  { flags}
		factor: SInt32;                 {    0-100; default is 50}
>>>>>>> origin/fixes_2.4
	end;
const
	kQTSTransAndProxyAtomType = FourCharCode('strp'); { transport/proxy prefs root atom}
	kQTSConnectionPrefsVersion = FourCharCode('vers'); {   prefs format version}
	kQTSTransportPrefsAtomType = FourCharCode('trns'); {   tranport prefs root atom}
	kQTSConnectionAtomType = FourCharCode('conn'); {     connection prefs atom type, one for each transport type}
	kQTSUDPTransportType = FourCharCode('udp '); {     udp transport prefs}
	kQTSHTTPTransportType = FourCharCode('http'); {     http transport prefs}
	kQTSTCPTransportType = FourCharCode('tcp '); {     tcp transport prefs    }
	kQTSProxyPrefsAtomType = FourCharCode('prxy'); {   proxy prefs root atom}
	kQTSHTTPProxyPrefsType = FourCharCode('http'); {     http proxy settings}
	kQTSRTSPProxyPrefsType = FourCharCode('rtsp'); {     rtsp proxy settings}
	kQTSSOCKSProxyPrefsType = FourCharCode('sock'); {     socks proxy settings}
	kQTSProxyUserInfoPrefsType = FourCharCode('user'); {   proxy username/password root atom}
	kQTSDontProxyPrefsAtomType = FourCharCode('nopr'); {   no-proxy prefs root atom}
	kQTSDontProxyDataType = FourCharCode('data'); {     no proxy settings}
	kQTSInstantOnPrefsAtomType = FourCharCode('inon'); { instant on prefs}

{
 *  QTSPrefsAddProxySetting()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPrefsAddProxySetting( proxyType: OSType; portID: SInt32; flags: UInt32; seed: UInt32; var srvrURL: Str255 ): OSErr; external name '_QTSPrefsAddProxySetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	kQTSTransAndProxyAtomType	= FourCharCode('strp');						{  transport/proxy prefs root atom }
	kQTSConnectionPrefsVersion	= FourCharCode('vers');						{    prefs format version }
	kQTSTransportPrefsAtomType	= FourCharCode('trns');						{    tranport prefs root atom }
	kQTSConnectionAtomType		= FourCharCode('conn');						{      connection prefs atom type, one for each transport type }
	kQTSUDPTransportType		= FourCharCode('udp ');						{      udp transport prefs }
	kQTSHTTPTransportType		= FourCharCode('http');						{      http transport prefs }
	kQTSTCPTransportType		= FourCharCode('tcp ');						{      tcp transport prefs     }
	kQTSProxyPrefsAtomType		= FourCharCode('prxy');						{    proxy prefs root atom }
	kQTSHTTPProxyPrefsType		= FourCharCode('http');						{      http proxy settings }
	kQTSRTSPProxyPrefsType		= FourCharCode('rtsp');						{      rtsp proxy settings }
	kQTSSOCKSProxyPrefsType		= FourCharCode('sock');						{      socks proxy settings }
	kQTSProxyUserInfoPrefsType	= FourCharCode('user');						{    proxy username/password root atom }
	kQTSDontProxyPrefsAtomType	= FourCharCode('nopr');						{    no-proxy prefs root atom }
	kQTSDontProxyDataType		= FourCharCode('data');						{      no proxy settings }
	kQTSInstantOnPrefsAtomType	= FourCharCode('inon');						{  instant on prefs }

	{
	 *  QTSPrefsAddProxySetting()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.1 and later
	 	}
function QTSPrefsAddProxySetting(proxyType: OSType; portID: SInt32; flags: UInt32; seed: UInt32; var srvrURL: Str255): OSErr; external name '_QTSPrefsAddProxySetting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsFindProxyByType()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPrefsFindProxyByType( proxyType: OSType; flags: UInt32; flagsMask: UInt32; var proxyHndl: UnivPtr; var count: SInt16 ): OSErr; external name '_QTSPrefsFindProxyByType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPrefsFindProxyByType(proxyType: OSType; flags: UInt32; flagsMask: UInt32; var proxyHndl: UnivPtr; var count: SInt16): OSErr; external name '_QTSPrefsFindProxyByType';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsFindProxyByType( proxyType: OSType; flags: UInt32; flagsMask: UInt32; var proxyHndl: UnivPtr; var count: SInt16 ): OSErr; external name '_QTSPrefsFindProxyByType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsAddConnectionSetting()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPrefsAddConnectionSetting( protocol: OSType; portID: SInt32; flags: UInt32; seed: UInt32 ): OSErr; external name '_QTSPrefsAddConnectionSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPrefsAddConnectionSetting(protocol: OSType; portID: SInt32; flags: UInt32; seed: UInt32): OSErr; external name '_QTSPrefsAddConnectionSetting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsAddConnectionSetting( protocol: OSType; portID: SInt32; flags: UInt32; seed: UInt32 ): OSErr; external name '_QTSPrefsAddConnectionSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsFindConnectionByType()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPrefsFindConnectionByType( protocol: OSType; flags: UInt32; flagsMask: UInt32; var connectionHndl: UnivPtr; var count: SInt16 ): OSErr; external name '_QTSPrefsFindConnectionByType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPrefsFindConnectionByType(protocol: OSType; flags: UInt32; flagsMask: UInt32; var connectionHndl: UnivPtr; var count: SInt16): OSErr; external name '_QTSPrefsFindConnectionByType';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsFindConnectionByType( protocol: OSType; flags: UInt32; flagsMask: UInt32; var connectionHndl: UnivPtr; var count: SInt16 ): OSErr; external name '_QTSPrefsFindConnectionByType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsGetActiveConnection()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPrefsGetActiveConnection( protocol: OSType; var connectInfo: QTSTransportPref ): OSErr; external name '_QTSPrefsGetActiveConnection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPrefsGetActiveConnection(protocol: OSType; var connectInfo: QTSTransportPref): OSErr; external name '_QTSPrefsGetActiveConnection';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsGetActiveConnection( protocol: OSType; var connectInfo: QTSTransportPref ): OSErr; external name '_QTSPrefsGetActiveConnection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsGetNoProxyURLs()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPrefsGetNoProxyURLs( var noProxyHndl: UnivPtr ): OSErr; external name '_QTSPrefsGetNoProxyURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPrefsGetNoProxyURLs(var noProxyHndl: UnivPtr): OSErr; external name '_QTSPrefsGetNoProxyURLs';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsGetNoProxyURLs( var noProxyHndl: UnivPtr ): OSErr; external name '_QTSPrefsGetNoProxyURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsSetNoProxyURLs()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
function QTSPrefsSetNoProxyURLs( urls: CStringPtr; flags: UInt32; seed: UInt32 ): OSErr; external name '_QTSPrefsSetNoProxyURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.1 and later
 *    Windows:          in QTSClient.lib 4.1 and later
 }
<<<<<<< HEAD
function QTSPrefsSetNoProxyURLs(urls: CStringPtr; flags: UInt32; seed: UInt32): OSErr; external name '_QTSPrefsSetNoProxyURLs';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsSetNoProxyURLs( urls: CStringPtr; flags: UInt32; seed: UInt32 ): OSErr; external name '_QTSPrefsSetNoProxyURLs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsAddProxyUserInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 *    Windows:          in QTSClient.lib 5.0.1 and later
 }
function QTSPrefsAddProxyUserInfo( proxyType: OSType; flags: SInt32; flagsMask: SInt32; username: StringPtr; password: StringPtr ): OSErr; external name '_QTSPrefsAddProxyUserInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
=======
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 *    Windows:          in QTSClient.lib 5.0.1 and later
 }
<<<<<<< HEAD
function QTSPrefsAddProxyUserInfo(proxyType: OSType; flags: SInt32; flagsMask: SInt32; username: StringPtr; password: StringPtr): OSErr; external name '_QTSPrefsAddProxyUserInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsAddProxyUserInfo( proxyType: OSType; flags: SInt32; flagsMask: SInt32; username: StringPtr; password: StringPtr ): OSErr; external name '_QTSPrefsAddProxyUserInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsFindProxyUserInfoByType()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 *    Windows:          in QTSClient.lib 5.0.1 and later
 }
function QTSPrefsFindProxyUserInfoByType( proxyType: OSType; flags: SInt32; flagsMask: SInt32; username: StringPtr; password: StringPtr ): OSErr; external name '_QTSPrefsFindProxyUserInfoByType';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
=======
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 *    Windows:          in QTSClient.lib 5.0.1 and later
 }
<<<<<<< HEAD
function QTSPrefsFindProxyUserInfoByType(proxyType: OSType; flags: SInt32; flagsMask: SInt32; username: StringPtr; password: StringPtr): OSErr; external name '_QTSPrefsFindProxyUserInfoByType';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsFindProxyUserInfoByType( proxyType: OSType; flags: SInt32; flagsMask: SInt32; username: StringPtr; password: StringPtr ): OSErr; external name '_QTSPrefsFindProxyUserInfoByType';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsGetInstantOnSettings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 6.0 and later
 *    Windows:          in QTSClient.lib 6.0 and later
 }
function QTSPrefsGetInstantOnSettings( var outPref: QTSInstantOnPref; inFlags: SInt32 ): OSErr; external name '_QTSPrefsGetInstantOnSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 6.0 and later
 *    Windows:          in QTSClient.lib 6.0 and later
 }
<<<<<<< HEAD
function QTSPrefsGetInstantOnSettings(var outPref: QTSInstantOnPref; inFlags: SInt32): OSErr; external name '_QTSPrefsGetInstantOnSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsGetInstantOnSettings( var outPref: QTSInstantOnPref; inFlags: SInt32 ): OSErr; external name '_QTSPrefsGetInstantOnSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSPrefsSetInstantOnSettings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 6.0 and later
 *    Windows:          in QTSClient.lib 6.0 and later
 }
function QTSPrefsSetInstantOnSettings( var inPref: QTSInstantOnPref; inFlags: SInt32 ): OSErr; external name '_QTSPrefsSetInstantOnSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 6.0 and later
 *    Windows:          in QTSClient.lib 6.0 and later
 }
<<<<<<< HEAD
function QTSPrefsSetInstantOnSettings(var inPref: QTSInstantOnPref; inFlags: SInt32): OSErr; external name '_QTSPrefsSetInstantOnSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSPrefsSetInstantOnSettings( var inPref: QTSInstantOnPref; inFlags: SInt32 ): OSErr; external name '_QTSPrefsSetInstantOnSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4


{============================================================================
        Memory Management Services
============================================================================}
{
   These routines allocate normal pointers and handles,
   but do the correct checking, etc.
   Dispose using the normal DisposePtr and DisposeHandle
   Call these routines for one time memory allocations.
   You do not need to set any hints to use these calls.
}

{
 *  QTSNewPtr()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSNewPtr( inByteCount: UInt32; inFlags: SInt32; var outFlags: SInt32 ): Ptr; external name '_QTSNewPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSNewPtr(inByteCount: UInt32; inFlags: SInt32; var outFlags: SInt32): Ptr; external name '_QTSNewPtr';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSNewPtr( inByteCount: UInt32; inFlags: SInt32; var outFlags: SInt32 ): Ptr; external name '_QTSNewPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSNewHandle()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSNewHandle( inByteCount: UInt32; inFlags: SInt32; var outFlags: SInt32 ): Handle; external name '_QTSNewHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ flags in}
const
	kQTSMemAllocClearMem = $00000001;
	kQTSMemAllocDontUseTempMem = $00000002;
	kQTSMemAllocTryTempMemFirst = $00000004;
	kQTSMemAllocDontUseSystemMem = $00000008;
	kQTSMemAllocTrySystemMemFirst = $00000010;
	kQTSMemAllocHoldMemory = $00001000;
	kQTSMemAllocIsInterruptTime = $01010000; { currently not supported for alloc}

{ flags out}
const
	kQTSMemAllocAllocatedInTempMem = $00000001;
	kQTSMemAllocAllocatedInSystemMem = $00000002;

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	QTSMemPtr = ^OpaqueQTSMemPtr; { an opaque type }
	OpaqueQTSMemPtr = record end;
=======
	QTSMemPtr = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	QTSMemPtr = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	QTSMemPtr = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	QTSMemPtr = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	QTSMemPtrPtr = ^QTSMemPtr;  { when a var xx:QTSMemPtr parameter can be nil, it is changed to xx: QTSMemPtrPtr }
{
   These routines are for buffers that will be recirculated
   you must use QTReleaseMemPtr instead of DisposePtr
   QTSReleaseMemPtr can be used at interrupt time
   but QTSAllocMemPtr currently cannot 
}
{
 *  QTSAllocMemPtr()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSAllocMemPtr( inByteCount: UInt32; inFlags: SInt32 ): QTSMemPtr; external name '_QTSAllocMemPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSNewHandle( inByteCount: UInt32; inFlags: SInt32; var outFlags: SInt32 ): Handle; external name '_QTSNewHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ flags in}
const
	kQTSMemAllocClearMem = $00000001;
	kQTSMemAllocDontUseTempMem = $00000002;
	kQTSMemAllocTryTempMemFirst = $00000004;
	kQTSMemAllocDontUseSystemMem = $00000008;
	kQTSMemAllocTrySystemMemFirst = $00000010;
	kQTSMemAllocHoldMemory = $00001000;
	kQTSMemAllocIsInterruptTime = $01010000; { currently not supported for alloc}

{ flags out}
const
	kQTSMemAllocAllocatedInTempMem = $00000001;
	kQTSMemAllocAllocatedInSystemMem = $00000002;

type
	QTSMemPtr = ^SInt32; { an opaque type }
	QTSMemPtrPtr = ^QTSMemPtr;  { when a var xx:QTSMemPtr parameter can be nil, it is changed to xx: QTSMemPtrPtr }
<<<<<<< HEAD
	{
	   These routines are for buffers that will be recirculated
	   you must use QTReleaseMemPtr instead of DisposePtr
	   QTSReleaseMemPtr can be used at interrupt time
	   but QTSAllocMemPtr currently cannot 
	}
	{
	 *  QTSAllocMemPtr()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.0 and later
	 	}
function QTSAllocMemPtr(inByteCount: UInt32; inFlags: SInt32): QTSMemPtr; external name '_QTSAllocMemPtr';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
   These routines are for buffers that will be recirculated
   you must use QTReleaseMemPtr instead of DisposePtr
   QTSReleaseMemPtr can be used at interrupt time
   but QTSAllocMemPtr currently cannot 
}
{
 *  QTSAllocMemPtr()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSAllocMemPtr( inByteCount: UInt32; inFlags: SInt32 ): QTSMemPtr; external name '_QTSAllocMemPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSReleaseMemPtr()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
procedure QTSReleaseMemPtr( inMemPtr: QTSMemPtr; inFlags: SInt32 ); external name '_QTSReleaseMemPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
procedure QTSReleaseMemPtr(inMemPtr: QTSMemPtr; inFlags: SInt32); external name '_QTSReleaseMemPtr';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure QTSReleaseMemPtr( inMemPtr: QTSMemPtr; inFlags: SInt32 ); external name '_QTSReleaseMemPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{============================================================================
        Buffer Management Services
============================================================================}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kQTSStreamBufferVersion1 = 1;
=======
=======
>>>>>>> origin/fixes_2_2

const
	kQTSStreamBufferVersion1	= 1;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kQTSStreamBufferVersion1 = 1;
>>>>>>> origin/fixes_2.4

type
	QTSStreamBufferPtr = ^QTSStreamBuffer;
	QTSStreamBuffer = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		reserved1: QTSStreamBufferPtr;
		reserved2: QTSStreamBufferPtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		next: QTSStreamBufferPtr;              { next message block in a message }
		rptr: UInt8Ptr;                   { first byte with real data in the DataBuffer }
		wptr: UInt8Ptr;                   { last+1 byte with real data in the DataBuffer }
		version: SInt32;
		metadata: array [0..4-1] of UInt32;            { usage defined by message sender }
		flags: SInt32;                  { reserved }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
		reserved1: QTSStreamBufferPtr;
		reserved2: QTSStreamBufferPtr;
>>>>>>> origin/fixes_2.4
		next: QTSStreamBufferPtr;              {  next message block in a message  }
		rptr: UInt8Ptr;                   {  first byte with real data in the DataBuffer  }
		wptr: UInt8Ptr;                   {  last+1 byte with real data in the DataBuffer  }
		version: SInt32;
		metadata: array [0..3] of UInt32;            {  usage defined by message sender  }
		flags: SInt32;                  {  reserved  }
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
		reserved3: SIGNEDLONG;
		reserved4: SIGNEDLONG;
		reserved5: SIGNEDLONG;
		moreMeta: array [0..7] of UInt32;
<<<<<<< HEAD
	end;
{ flags for QTSDuplicateMessage}
=======
=======
>>>>>>> origin/fixes_2_2
		reserved1:				QTSStreamBufferPtr;
		reserved2:				QTSStreamBufferPtr;
		next:					QTSStreamBufferPtr;						{  next message block in a message  }
		rptr:					Ptr;									{  first byte with real data in the DataBuffer  }
		wptr:					Ptr;									{  last+1 byte with real data in the DataBuffer  }
		version:				SInt32;
		metadata:				array [0..3] of UInt32;					{  usage defined by message sender  }
		flags:					SInt32;									{  reserved  }
		reserved3:				SInt32;
		reserved4:				SInt32;
		reserved5:				SInt32;
		moreMeta:				array [0..7] of UInt32;
	end;

	{  flags for QTSDuplicateMessage }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	end;
{ flags for QTSDuplicateMessage}
>>>>>>> origin/fixes_2.4
const
	kQTSDuplicateBufferFlag_CopyData = $00000001;
	kQTSDuplicateBufferFlag_FlattenMessage = $00000002;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  QTSNewStreamBuffer()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
function QTSNewStreamBuffer( inDataSize: UInt32; inFlags: SInt32; var outStreamBuffer: UnivPtr ): OSErr; external name '_QTSNewStreamBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  QTSNewStreamBuffer()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
	 *    CarbonLib:        in CarbonLib 1.3 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 5.0 and later
	 	}
function QTSNewStreamBuffer(inDataSize: UInt32; inFlags: SInt32; var outStreamBuffer: UnivPtr): OSErr; external name '_QTSNewStreamBuffer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  QTSFreeMessage()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
procedure QTSFreeMessage( var inMessage: QTSStreamBuffer ); external name '_QTSFreeMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
procedure QTSFreeMessage(var inMessage: QTSStreamBuffer); external name '_QTSFreeMessage';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure QTSFreeMessage( var inMessage: QTSStreamBuffer ); external name '_QTSFreeMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
    kQTSDuplicateBufferFlag_CopyData - forces a copy of the data itself
    kQTSCopyBufferFlag_FlattenMessage - copies the data if it needs to be flattened
    QTSDuplicateMessage never frees the old message
}
{
 *  QTSDuplicateMessage()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
function QTSDuplicateMessage( var inMessage: QTSStreamBuffer; inFlags: SInt32; var outDuplicatedMessage: UnivPtr ): OSErr; external name '_QTSDuplicateMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
<<<<<<< HEAD
function QTSDuplicateMessage(var inMessage: QTSStreamBuffer; inFlags: SInt32; var outDuplicatedMessage: UnivPtr): OSErr; external name '_QTSDuplicateMessage';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSDuplicateMessage( var inMessage: QTSStreamBuffer; inFlags: SInt32; var outDuplicatedMessage: UnivPtr ): OSErr; external name '_QTSDuplicateMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSMessageLength()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSMessageLength( var inMessage: QTSStreamBuffer ): UInt32; external name '_QTSMessageLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSMessageLength(var inMessage: QTSStreamBuffer): UInt32; external name '_QTSMessageLength';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSMessageLength( var inMessage: QTSStreamBuffer ): UInt32; external name '_QTSMessageLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSStreamBufferDataInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
procedure QTSStreamBufferDataInfo( var inStreamBuffer: QTSStreamBuffer; var outDataStart: UnivPtr; var outDataMaxLength: UInt32 ); external name '_QTSStreamBufferDataInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ ---- old calls (don't use these)}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 *    Windows:          in QTSClient.lib 5.0 and later
 }
procedure QTSStreamBufferDataInfo( var inStreamBuffer: QTSStreamBuffer; var outDataStart: UnivPtr; var outDataMaxLength: UInt32 ); external name '_QTSStreamBufferDataInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
{  ---- old calls (don't use these) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ ---- old calls (don't use these)}
>>>>>>> origin/fixes_2.4

{
 *  QTSAllocBuffer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSAllocBuffer( inSize: SInt32 ): QTSStreamBufferPtr; external name '_QTSAllocBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSAllocBuffer(inSize: SInt32): QTSStreamBufferPtr; external name '_QTSAllocBuffer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSAllocBuffer( inSize: SInt32 ): QTSStreamBufferPtr; external name '_QTSAllocBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSDupMessage()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSDupMessage( var inMessage: QTSStreamBuffer ): QTSStreamBufferPtr; external name '_QTSDupMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSDupMessage(var inMessage: QTSStreamBuffer): QTSStreamBufferPtr; external name '_QTSDupMessage';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSDupMessage( var inMessage: QTSStreamBuffer ): QTSStreamBufferPtr; external name '_QTSDupMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSCopyMessage()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSCopyMessage( var inMessage: QTSStreamBuffer ): QTSStreamBufferPtr; external name '_QTSCopyMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSCopyMessage(var inMessage: QTSStreamBuffer): QTSStreamBufferPtr; external name '_QTSCopyMessage';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSCopyMessage( var inMessage: QTSStreamBuffer ): QTSStreamBufferPtr; external name '_QTSCopyMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSFlattenMessage()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSFlattenMessage( var inMessage: QTSStreamBuffer ): QTSStreamBufferPtr; external name '_QTSFlattenMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSFlattenMessage(var inMessage: QTSStreamBuffer): QTSStreamBufferPtr; external name '_QTSFlattenMessage';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSFlattenMessage( var inMessage: QTSStreamBuffer ): QTSStreamBufferPtr; external name '_QTSFlattenMessage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{============================================================================
        Misc
============================================================================}
{
 *  QTSGetErrorString()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSGetErrorString( inErrorCode: SInt32; inMaxErrorStringLength: UInt32; outErrorString: CStringPtr; inFlags: SInt32 ): Boolean; external name '_QTSGetErrorString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSGetErrorString(inErrorCode: SInt32; inMaxErrorStringLength: UInt32; outErrorString: CStringPtr; inFlags: SInt32): boolean; external name '_QTSGetErrorString';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSGetErrorString( inErrorCode: SInt32; inMaxErrorStringLength: UInt32; outErrorString: CStringPtr; inFlags: SInt32 ): Boolean; external name '_QTSGetErrorString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSInitializeMediaParams()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 *    Windows:          in QTSClient.lib 5.0.1 and later
 }
function QTSInitializeMediaParams( var inMediaParams: QTSMediaParams ): OSErr; external name '_QTSInitializeMediaParams';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{ UPP call backs }
{
 *  NewQTSNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTSNotificationUPP( userRoutine: QTSNotificationProcPtr ): QTSNotificationUPP; external name '_NewQTSNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
=======
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 *    Windows:          in QTSClient.lib 5.0.1 and later
 }
function QTSInitializeMediaParams( var inMediaParams: QTSMediaParams ): OSErr; external name '_QTSInitializeMediaParams';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{ UPP call backs }
{
 *  NewQTSNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTSNotificationUPP( userRoutine: QTSNotificationProcPtr ): QTSNotificationUPP; external name '_NewQTSNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	uppQTSNotificationProcInfo = $00003FF0;
	uppQTSPanelFilterProcInfo = $000003D0;
	uppQTSModalFilterProcInfo = $00003FD0;
	{
	 *  NewQTSNotificationUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.3 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewQTSNotificationUPP(userRoutine: QTSNotificationProcPtr): QTSNotificationUPP; external name '_NewQTSNotificationUPP'; { old name was NewQTSNotificationProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewQTSPanelFilterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTSPanelFilterUPP( userRoutine: QTSPanelFilterProcPtr ): QTSPanelFilterUPP; external name '_NewQTSPanelFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewQTSPanelFilterUPP(userRoutine: QTSPanelFilterProcPtr): QTSPanelFilterUPP; external name '_NewQTSPanelFilterUPP'; { old name was NewQTSPanelFilterProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewQTSPanelFilterUPP( userRoutine: QTSPanelFilterProcPtr ): QTSPanelFilterUPP; external name '_NewQTSPanelFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  NewQTSModalFilterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTSModalFilterUPP( userRoutine: QTSModalFilterProcPtr ): QTSModalFilterUPP; external name '_NewQTSModalFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewQTSModalFilterUPP(userRoutine: QTSModalFilterProcPtr): QTSModalFilterUPP; external name '_NewQTSModalFilterUPP'; { old name was NewQTSModalFilterProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewQTSModalFilterUPP( userRoutine: QTSModalFilterProcPtr ): QTSModalFilterUPP; external name '_NewQTSModalFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeQTSNotificationUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTSNotificationUPP( userUPP: QTSNotificationUPP ); external name '_DisposeQTSNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTSNotificationUPP(userUPP: QTSNotificationUPP); external name '_DisposeQTSNotificationUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTSNotificationUPP( userUPP: QTSNotificationUPP ); external name '_DisposeQTSNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeQTSPanelFilterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTSPanelFilterUPP( userUPP: QTSPanelFilterUPP ); external name '_DisposeQTSPanelFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTSPanelFilterUPP(userUPP: QTSPanelFilterUPP); external name '_DisposeQTSPanelFilterUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTSPanelFilterUPP( userUPP: QTSPanelFilterUPP ); external name '_DisposeQTSPanelFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeQTSModalFilterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTSModalFilterUPP( userUPP: QTSModalFilterUPP ); external name '_DisposeQTSModalFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTSModalFilterUPP(userUPP: QTSModalFilterUPP); external name '_DisposeQTSModalFilterUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTSModalFilterUPP( userUPP: QTSModalFilterUPP ); external name '_DisposeQTSModalFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTSNotificationUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTSNotificationUPP( inErr: ComponentResult; inNotificationType: OSType; inNotificationParams: UnivPtr; inRefCon: UnivPtr; userUPP: QTSNotificationUPP ): ComponentResult; external name '_InvokeQTSNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeQTSNotificationUPP(inErr: ComponentResult; inNotificationType: OSType; inNotificationParams: UnivPtr; inRefCon: UnivPtr; userRoutine: QTSNotificationUPP): ComponentResult; external name '_InvokeQTSNotificationUPP'; { old name was CallQTSNotificationProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeQTSNotificationUPP( inErr: ComponentResult; inNotificationType: OSType; inNotificationParams: UnivPtr; inRefCon: UnivPtr; userUPP: QTSNotificationUPP ): ComponentResult; external name '_InvokeQTSNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTSPanelFilterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTSPanelFilterUPP( var inParams: QTSPanelFilterParams; inRefCon: UnivPtr; userUPP: QTSPanelFilterUPP ): Boolean; external name '_InvokeQTSPanelFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeQTSPanelFilterUPP(var inParams: QTSPanelFilterParams; inRefCon: UnivPtr; userRoutine: QTSPanelFilterUPP): boolean; external name '_InvokeQTSPanelFilterUPP'; { old name was CallQTSPanelFilterProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeQTSPanelFilterUPP( var inParams: QTSPanelFilterParams; inRefCon: UnivPtr; userUPP: QTSPanelFilterUPP ): Boolean; external name '_InvokeQTSPanelFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTSModalFilterUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTSModalFilterUPP( inDialog: DialogPtr; const (*var*) inEvent: EventRecord; var ioItemHit: SInt16; inRefCon: UnivPtr; userUPP: QTSModalFilterUPP ): Boolean; external name '_InvokeQTSModalFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTSModalFilterUPP( inDialog: DialogPtr; const (*var*) inEvent: EventRecord; var ioItemHit: SInt16; inRefCon: UnivPtr; userUPP: QTSModalFilterUPP ): Boolean; external name '_InvokeQTSModalFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {TARGET_CPU_64}

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
