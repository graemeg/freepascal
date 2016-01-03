{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/QTStreamingComponents.h
 
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
     File:       QTStreamingComponents.p
=======
     File:       QuickTime/QTStreamingComponents.h
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

unit QTStreamingComponents;
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
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
{$elsec}
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
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
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
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
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
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes,Components,Dialogs,Movies,QuickTimeStreaming;
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

{$ALIGN MAC68K}


{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

{============================================================================
        Stream Sourcer
============================================================================}
const
	kQTSSourcerType = FourCharCode('srcr');

type
	QTSSourcer = ComponentInstance;
const
	kQTSSGChannelSourcerType = FourCharCode('sgch');
	kQTSMovieTrackSourcerType = FourCharCode('trak');
	kQTSPushDataSourcerType = FourCharCode('push');

{ flags for sourcer data }
const
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Dialogs,Components,Movies,QuickTimeStreaming;
=======
uses MacTypes,Components,Dialogs,Movies,QuickTimeStreaming;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

{============================================================================
        Stream Sourcer
============================================================================}
const
	kQTSSourcerType = FourCharCode('srcr');

type
	QTSSourcer = ComponentInstance;
const
	kQTSSGChannelSourcerType = FourCharCode('sgch');
	kQTSMovieTrackSourcerType = FourCharCode('trak');
	kQTSPushDataSourcerType = FourCharCode('push');

<<<<<<< HEAD
	{	 flags for sourcer data 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ flags for sourcer data }
const
>>>>>>> origin/fixes_2.4
	kQTSSourcerDataFlag_SyncSample = $00000001;
	kQTSPushDataSourcerFlag_SampleTimeIsValid = $80000000;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kQTSSourcerInitParamsVersion1 = 1;

type
	QTSSourcerInitParamsPtr = ^QTSSourcerInitParams;
	QTSSourcerInitParams = record
		version: SInt32;
		flags: SInt32;
		dataType: OSType;
		data: UnivPtr;
		dataLength: UInt32;
	end;
{
 *  QTSNewSourcer()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSNewSourcer( params: UnivPtr; const (*var*) inInitParams: QTSSourcerInitParams; inFlags: SInt32; var outSourcer: ComponentInstance ): OSErr; external name '_QTSNewSourcer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ info selectors for sourcers - get and set }
const
	kQTSInfo_Track = FourCharCode('trak'); { QTSTrackParams* }
	kQTSInfo_Loop = FourCharCode('loop'); { QTSLoopParams* }
	kQTSInfo_SourcerTiming = FourCharCode('stim'); { QTSSourcerTimingParams* }
	kQTSInfo_TargetFrameRate = FourCharCode('tfps'); { Fixed * in frames per second }
	kQTSInfo_PushData = FourCharCode('push'); { QTSPushDataParams* }
	kQTSInfo_SourcerCallbackProc = FourCharCode('scbp'); { QTSSourcerCallbackProcParams* }
	kQTSInfo_TargetDataRate = FourCharCode('tdrt'); { UInt32 * in bytes per second }
	kQTSInfo_AudioAutoGainOnOff = FourCharCode('agc '); { Boolean*  - error if unavailable}
	kQTSInfo_AudioGain = FourCharCode('gain'); { Fixed* kFixed1 is unity gain }
	kQTSInfo_CroppedInputRect = FourCharCode('crpr'); { Rect* - defined relative to kQTSInfo_FullInputRect below }
	kQTSInfo_SpatialSettings = FourCharCode('sptl'); { pointer to SCSpatialSettings struct}
	kQTSInfo_TemporalSettings = FourCharCode('tprl'); { pointer to SCTemporalSettings struct}
	kQTSInfo_DataRateSettings = FourCharCode('drat'); { pointer to SCDataRateSettings struct}
	kQTSInfo_CodecFlags = FourCharCode('cflg'); { pointer to CodecFlags}
	kQTSInfo_CodecSettings = FourCharCode('cdec'); { pointer to Handle}
	kQTSInfo_ForceKeyValue = FourCharCode('ksim'); { pointer to long}
	kQTSInfo_SoundSampleRate = FourCharCode('ssrt'); { pointer to UnsignedFixed}
	kQTSInfo_SoundSampleSize = FourCharCode('ssss'); { pointer to short}
	kQTSInfo_SoundChannelCount = FourCharCode('sscc'); { pointer to short}
	kQTSInfo_SoundCompression = FourCharCode('ssct'); { pointer to OSType}
	kQTSInfo_CompressionList = FourCharCode('ctyl'); { pointer to OSType Handle}
	kQTSInfo_VideoHue = FourCharCode('hue '); { UInt16* }
	kQTSInfo_VideoSaturation = FourCharCode('satr'); { UInt16* }
	kQTSInfo_VideoContrast = FourCharCode('trst'); { UInt16* }
	kQTSInfo_VideoBrightness = FourCharCode('brit'); { UInt16* }
	kQTSInfo_VideoSharpness = FourCharCode('shrp'); { UInt16* }
	kQTSInfo_TimeScale = FourCharCode('scal'); { UInt32* }
	kQTSInfo_SGChannelDeviceName = FourCharCode('innm'); { Handle* }
	kQTSInfo_SGChannelDeviceList = FourCharCode('srdl'); { SGDeviceList* }
	kQTSInfo_SGChannelDeviceInput = FourCharCode('sdii'); { short* }
	kQTSInfo_SGChannelSettings = FourCharCode('sesg'); { QTSSGChannelSettingsParams }
	kQTSInfo_PreviewWhileRecordingMode = FourCharCode('srpr'); { Boolean* }
	kQTSInfo_CompressionParams = FourCharCode('sccp'); { QTAtomContainer* }

{ info selectors for sourcers - get only}
const
	kQTSInfo_SGChannel = FourCharCode('sgch'); { SGChannel* }
	kQTSInfo_SGChannelInputName = FourCharCode('srnm'); { Handle* }
	kQTSInfo_FullInputRect = FourCharCode('fulr'); { Rect* }

{ loop flags }
const
	kQTSLoopFlag_Loop = $00000001;

const
	kQTSLoopParamsVersion1 = 1;
=======
=======
>>>>>>> origin/fixes_2_2
=======
const
>>>>>>> origin/fixes_2.4
	kQTSSourcerInitParamsVersion1 = 1;

type
	QTSSourcerInitParamsPtr = ^QTSSourcerInitParams;
	QTSSourcerInitParams = record
		version: SInt32;
		flags: SInt32;
		dataType: OSType;
		data: UnivPtr;
		dataLength: UInt32;
	end;
{
 *  QTSNewSourcer()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSNewSourcer( params: UnivPtr; const (*var*) inInitParams: QTSSourcerInitParams; inFlags: SInt32; var outSourcer: ComponentInstance ): OSErr; external name '_QTSNewSourcer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ info selectors for sourcers - get and set }
const
	kQTSInfo_Track = FourCharCode('trak'); { QTSTrackParams* }
	kQTSInfo_Loop = FourCharCode('loop'); { QTSLoopParams* }
	kQTSInfo_SourcerTiming = FourCharCode('stim'); { QTSSourcerTimingParams* }
	kQTSInfo_TargetFrameRate = FourCharCode('tfps'); { Fixed * in frames per second }
	kQTSInfo_PushData = FourCharCode('push'); { QTSPushDataParams* }
	kQTSInfo_SourcerCallbackProc = FourCharCode('scbp'); { QTSSourcerCallbackProcParams* }
	kQTSInfo_TargetDataRate = FourCharCode('tdrt'); { UInt32 * in bytes per second }
	kQTSInfo_AudioAutoGainOnOff = FourCharCode('agc '); { Boolean*  - error if unavailable}
	kQTSInfo_AudioGain = FourCharCode('gain'); { Fixed* kFixed1 is unity gain }
	kQTSInfo_CroppedInputRect = FourCharCode('crpr'); { Rect* - defined relative to kQTSInfo_FullInputRect below }
	kQTSInfo_SpatialSettings = FourCharCode('sptl'); { pointer to SCSpatialSettings struct}
	kQTSInfo_TemporalSettings = FourCharCode('tprl'); { pointer to SCTemporalSettings struct}
	kQTSInfo_DataRateSettings = FourCharCode('drat'); { pointer to SCDataRateSettings struct}
	kQTSInfo_CodecFlags = FourCharCode('cflg'); { pointer to CodecFlags}
	kQTSInfo_CodecSettings = FourCharCode('cdec'); { pointer to Handle}
	kQTSInfo_ForceKeyValue = FourCharCode('ksim'); { pointer to long}
	kQTSInfo_SoundSampleRate = FourCharCode('ssrt'); { pointer to UnsignedFixed}
	kQTSInfo_SoundSampleSize = FourCharCode('ssss'); { pointer to short}
	kQTSInfo_SoundChannelCount = FourCharCode('sscc'); { pointer to short}
	kQTSInfo_SoundCompression = FourCharCode('ssct'); { pointer to OSType}
	kQTSInfo_CompressionList = FourCharCode('ctyl'); { pointer to OSType Handle}
	kQTSInfo_VideoHue = FourCharCode('hue '); { UInt16* }
	kQTSInfo_VideoSaturation = FourCharCode('satr'); { UInt16* }
	kQTSInfo_VideoContrast = FourCharCode('trst'); { UInt16* }
	kQTSInfo_VideoBrightness = FourCharCode('brit'); { UInt16* }
	kQTSInfo_VideoSharpness = FourCharCode('shrp'); { UInt16* }
	kQTSInfo_TimeScale = FourCharCode('scal'); { UInt32* }
	kQTSInfo_SGChannelDeviceName = FourCharCode('innm'); { Handle* }
	kQTSInfo_SGChannelDeviceList = FourCharCode('srdl'); { SGDeviceList* }
	kQTSInfo_SGChannelDeviceInput = FourCharCode('sdii'); { short* }
	kQTSInfo_SGChannelSettings = FourCharCode('sesg'); { QTSSGChannelSettingsParams }
	kQTSInfo_PreviewWhileRecordingMode = FourCharCode('srpr'); { Boolean* }
	kQTSInfo_CompressionParams = FourCharCode('sccp'); { QTAtomContainer* }

{ info selectors for sourcers - get only}
const
	kQTSInfo_SGChannel = FourCharCode('sgch'); { SGChannel* }
	kQTSInfo_SGChannelInputName = FourCharCode('srnm'); { Handle* }
	kQTSInfo_FullInputRect = FourCharCode('fulr'); { Rect* }

{ loop flags }
const
	kQTSLoopFlag_Loop = $00000001;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kQTSLoopParamsVersion1 = 1;
>>>>>>> origin/fixes_2.4

type
	QTSLoopParamsPtr = ^QTSLoopParams;
	QTSLoopParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		version: SInt32;
		flags: SInt32;
		loopFlags: SInt32;
		flagsMask: SInt32;
		numLoops: SInt32;
<<<<<<< HEAD
	end;
const
	kQTSTrackParamsVersion1 = 1;
=======
=======
>>>>>>> origin/fixes_2_2
		version:				SInt32;
		flags:					SInt32;
		loopFlags:				SInt32;
		flagsMask:				SInt32;
		numLoops:				SInt32;
=======
>>>>>>> origin/fixes_2.4
	end;
const
<<<<<<< HEAD
	kQTSTrackParamsVersion1		= 1;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSTrackParamsVersion1 = 1;
>>>>>>> origin/fixes_2.4

type
	QTSTrackParamsPtr = ^QTSTrackParams;
	QTSTrackParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		version: SInt32;
		flags: SInt32;
		track: Track_fix;
		trackStartOffset: TimeValue64;       { to start other than at the beginning otherwise set to 0}
		duration: TimeValue64;               { to limit the duration otherwise set to 0}
		loopParams: QTSLoopParamsPtr;             { set to NULL if not using; default is no looping }
<<<<<<< HEAD
	end;
const
	kQTSSourcerTimingParamsVersion1 = 1;

type
	QTSSourcerTimingParamsPtr = ^QTSSourcerTimingParams;
	QTSSourcerTimingParams = record
		version: SInt32;
		flags: SInt32;
		timeScale_: TimeScale;
		presentationStartTime: TimeValue64;
		presentationEndTime: TimeValue64;
		presentationCurrentTime: TimeValue64;
		localStartTime: TimeValue64;
		localEndTime: TimeValue64;
		localCurrentTime: TimeValue64;
	end;
const
	kQTSPushDataParamsVersion1 = 1;

const
	kQTSPushDataFlag_SampleTimeIsValid = $00000001;
	kQTSPushDataFlag_DurationIsValid = $00000002;

type
	QTSPushDataParamsPtr = ^QTSPushDataParams;
	QTSPushDataParams = record
		version: SInt32;
		flags: SInt32;
		sampleDescription: SampleDescriptionHandle; { caller owns the handle }
		sampleDescSeed: UInt32;
		sampleTime: TimeValue64;             { also set flag if you set this }
		duration: TimeValue64;               { also set flag if you set this }
		dataLength: UInt32;
		dataPtr: UnivPtr;                { this does not have to be a real macintosh Ptr }
	end;
=======
=======
>>>>>>> origin/fixes_2_2
		version:				SInt32;
		flags:					SInt32;
		track:					Track_fix;
		trackStartOffset:		TimeValue64;							{  to start other than at the beginning otherwise set to 0 }
		duration:				TimeValue64;							{  to limit the duration otherwise set to 0 }
		loopParams:				QTSLoopParamsPtr;						{  set to NULL if not using; default is no looping  }
=======
>>>>>>> origin/fixes_2.4
	end;
const
	kQTSSourcerTimingParamsVersion1 = 1;

type
	QTSSourcerTimingParamsPtr = ^QTSSourcerTimingParams;
	QTSSourcerTimingParams = record
		version: SInt32;
		flags: SInt32;
		timeScale_: TimeScale;
		presentationStartTime: TimeValue64;
		presentationEndTime: TimeValue64;
		presentationCurrentTime: TimeValue64;
		localStartTime: TimeValue64;
		localEndTime: TimeValue64;
		localCurrentTime: TimeValue64;
	end;
const
	kQTSPushDataParamsVersion1 = 1;

const
	kQTSPushDataFlag_SampleTimeIsValid = $00000001;
	kQTSPushDataFlag_DurationIsValid = $00000002;

type
	QTSPushDataParamsPtr = ^QTSPushDataParams;
	QTSPushDataParams = record
		version: SInt32;
		flags: SInt32;
		sampleDescription: SampleDescriptionHandle; { caller owns the handle }
		sampleDescSeed: UInt32;
		sampleTime: TimeValue64;             { also set flag if you set this }
		duration: TimeValue64;               { also set flag if you set this }
		dataLength: UInt32;
		dataPtr: UnivPtr;                { this does not have to be a real macintosh Ptr }
	end;
<<<<<<< HEAD


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kQTSSourcerCallbackProcParamsVersion1 = 1;


type
	QTSSourcerCallbackProcParamsPtr = ^QTSSourcerCallbackProcParams;
	QTSSourcerCallbackProcParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		version: SInt32;
		flags: SInt32;
		proc: QTSNotificationUPP;
		refCon: UnivPtr;
<<<<<<< HEAD
	end;
{ track sourcer callback selectors}
const
	kQTSSourcerCallback_Done = FourCharCode('done'); { QTSSourcerDoneParams* }
<<<<<<< HEAD


=======


>>>>>>> origin/cpstrnew
{ push data sourcer callback selectors}
const
	kQTSPushDataSourcerCallback_HasCharacteristic = $050D; { QTSPushDataHasCharacteristicParams* }
	kQTSPushDataSourcerCallback_SetInfo = $0507; { QTSPushDataInfoParams* }
	kQTSPushDataSourcerCallback_GetInfo = $0508; { QTSPushDataInfoParams* }
=======
=======
>>>>>>> origin/fixes_2_2
		version:				SInt32;
		flags:					SInt32;
		proc:					QTSNotificationUPP;
		refCon:					Ptr;
=======
>>>>>>> origin/fixes_2.4
	end;
{ track sourcer callback selectors}
const
	kQTSSourcerCallback_Done = FourCharCode('done'); { QTSSourcerDoneParams* }


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ push data sourcer callback selectors}
const
	kQTSPushDataSourcerCallback_HasCharacteristic = $050D; { QTSPushDataHasCharacteristicParams* }
	kQTSPushDataSourcerCallback_SetInfo = $0507; { QTSPushDataInfoParams* }
	kQTSPushDataSourcerCallback_GetInfo = $0508; { QTSPushDataInfoParams* }
>>>>>>> origin/fixes_2.4

type
	QTSPushDataHasCharacteristicParamsPtr = ^QTSPushDataHasCharacteristicParams;
	QTSPushDataHasCharacteristicParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		version: SInt32;
		flags: SInt32;
		characteristic: OSType;
		returnedHasIt: Boolean;
		reserved1: SInt8;
		reserved2: SInt8;
		reserved3: SInt8;
<<<<<<< HEAD
	end;
type
	QTSPushDataInfoParamsPtr = ^QTSPushDataInfoParams;
	QTSPushDataInfoParams = record
		version: SInt32;
		flags: SInt32;
		selector: OSType;
		ioParams: UnivPtr;
	end;
const
	kQTSSourcerDoneParamsVersion1 = 1;

type
	QTSSourcerDoneParamsPtr = ^QTSSourcerDoneParams;
	QTSSourcerDoneParams = record
		version: SInt32;
		flags: SInt32;
		sourcer: ComponentInstance;
	end;
type
	QTSSGChannelSettingsParamsPtr = ^QTSSGChannelSettingsParams;
	QTSSGChannelSettingsParams = record
		settings: UserData;
		flags: SInt32;
	end;

{-----------------------------------------
    Stream Sourcer Selectors
-----------------------------------------}
const
	kQTSSourcerInitializeSelect = $0500;
	kQTSSourcerSetEnableSelect = $0503;
	kQTSSourcerGetEnableSelect = $0504;
	kQTSSourcerSetInfoSelect = $0507;
	kQTSSourcerGetInfoSelect = $0508;
	kQTSSourcerSetTimeScaleSelect = $050E;
	kQTSSourcerGetTimeScaleSelect = $050F;
	kQTSSourcerIdleSelect = $0516;

{-----------------------------------------
    Stream Sourcer Prototypes
-----------------------------------------}
{
 *  QTSSourcerInitialize()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 }
function QTSSourcerInitialize( inSourcer: QTSSourcer; const (*var*) inInitParams: QTSSourcerInitParams ): ComponentResult; external name '_QTSSourcerInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
		version:				SInt32;
		flags:					SInt32;
		characteristic:			OSType;
		returnedHasIt:			boolean;
		reserved1:				SInt8;
		reserved2:				SInt8;
		reserved3:				SInt8;
=======
>>>>>>> origin/fixes_2.4
	end;
type
	QTSPushDataInfoParamsPtr = ^QTSPushDataInfoParams;
	QTSPushDataInfoParams = record
		version: SInt32;
		flags: SInt32;
		selector: OSType;
		ioParams: UnivPtr;
	end;
const
	kQTSSourcerDoneParamsVersion1 = 1;

type
	QTSSourcerDoneParamsPtr = ^QTSSourcerDoneParams;
	QTSSourcerDoneParams = record
		version: SInt32;
		flags: SInt32;
		sourcer: ComponentInstance;
	end;
type
	QTSSGChannelSettingsParamsPtr = ^QTSSGChannelSettingsParams;
	QTSSGChannelSettingsParams = record
		settings: UserData;
		flags: SInt32;
	end;

{-----------------------------------------
    Stream Sourcer Selectors
-----------------------------------------}
const
	kQTSSourcerInitializeSelect = $0500;
	kQTSSourcerSetEnableSelect = $0503;
	kQTSSourcerGetEnableSelect = $0504;
	kQTSSourcerSetInfoSelect = $0507;
	kQTSSourcerGetInfoSelect = $0508;
	kQTSSourcerSetTimeScaleSelect = $050E;
	kQTSSourcerGetTimeScaleSelect = $050F;
<<<<<<< HEAD
	kQTSSourcerIdleSelect		= $0516;

	{	-----------------------------------------
	    Stream Sourcer Prototypes
	-----------------------------------------	}
	{
	 *  QTSSourcerInitialize()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
	 *    CarbonLib:        in CarbonLib 1.6 and later
	 *    Mac OS X:         in version 10.1 and later
	 	}
function QTSSourcerInitialize(inSourcer: QTSSourcer; const (*var*) inInitParams: QTSSourcerInitParams): ComponentResult; external name '_QTSSourcerInitialize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTSSourcerIdleSelect = $0516;

{-----------------------------------------
    Stream Sourcer Prototypes
-----------------------------------------}
{
 *  QTSSourcerInitialize()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0.1 and later
 }
function QTSSourcerInitialize( inSourcer: QTSSourcer; const (*var*) inInitParams: QTSSourcerInitParams ): ComponentResult; external name '_QTSSourcerInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTSSourcerIdle()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSSourcerIdle( inSourcer: QTSSourcer; (*const*) var inTime: TimeValue64; inFlags: SInt32; var outFlags: SInt32 ): ComponentResult; external name '_QTSSourcerIdle';
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
function QTSSourcerIdle(inSourcer: QTSSourcer; (*const*) var inTime: TimeValue64; inFlags: SInt32; var outFlags: SInt32): ComponentResult; external name '_QTSSourcerIdle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSSourcerIdle( inSourcer: QTSSourcer; (*const*) var inTime: TimeValue64; inFlags: SInt32; var outFlags: SInt32 ): ComponentResult; external name '_QTSSourcerIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTSSourcerSetEnable()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSSourcerSetEnable( inSourcer: QTSSourcer; inEnableMode: Boolean; inFlags: SInt32 ): ComponentResult; external name '_QTSSourcerSetEnable';
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
function QTSSourcerSetEnable(inSourcer: QTSSourcer; inEnableMode: boolean; inFlags: SInt32): ComponentResult; external name '_QTSSourcerSetEnable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSSourcerSetEnable( inSourcer: QTSSourcer; inEnableMode: Boolean; inFlags: SInt32 ): ComponentResult; external name '_QTSSourcerSetEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTSSourcerGetEnable()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSSourcerGetEnable( inSourcer: QTSSourcer; var outEnableMode: Boolean; inFlags: SInt32 ): ComponentResult; external name '_QTSSourcerGetEnable';
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
function QTSSourcerGetEnable(inSourcer: QTSSourcer; var outEnableMode: boolean; inFlags: SInt32): ComponentResult; external name '_QTSSourcerGetEnable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSSourcerGetEnable( inSourcer: QTSSourcer; var outEnableMode: Boolean; inFlags: SInt32 ): ComponentResult; external name '_QTSSourcerGetEnable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTSSourcerSetTimeScale()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSSourcerSetTimeScale( inSourcer: QTSSourcer; inTimeScale: TimeScale ): ComponentResult; external name '_QTSSourcerSetTimeScale';
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
function QTSSourcerSetTimeScale(inSourcer: QTSSourcer; inTimeScale: TimeScale): ComponentResult; external name '_QTSSourcerSetTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSSourcerSetTimeScale( inSourcer: QTSSourcer; inTimeScale: TimeScale ): ComponentResult; external name '_QTSSourcerSetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTSSourcerGetTimeScale()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSSourcerGetTimeScale( inSourcer: QTSSourcer; var outTimeScale: TimeScale ): ComponentResult; external name '_QTSSourcerGetTimeScale';
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
function QTSSourcerGetTimeScale(inSourcer: QTSSourcer; var outTimeScale: TimeScale): ComponentResult; external name '_QTSSourcerGetTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSSourcerGetTimeScale( inSourcer: QTSSourcer; var outTimeScale: TimeScale ): ComponentResult; external name '_QTSSourcerGetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTSSourcerSetInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSSourcerSetInfo( inSourcer: QTSSourcer; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_QTSSourcerSetInfo';
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
function QTSSourcerSetInfo(inSourcer: QTSSourcer; inSelector: OSType; ioParams: UnivPtr): ComponentResult; external name '_QTSSourcerSetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSSourcerSetInfo( inSourcer: QTSSourcer; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_QTSSourcerSetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTSSourcerGetInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QTStreamLib 5.0 and later
 }
function QTSSourcerGetInfo( inSourcer: QTSSourcer; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_QTSSourcerGetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew

const
	kQTSInfo_InputDeviceName = FourCharCode('innm'); { Handle* }
	kQTSInfo_InputSourceName = FourCharCode('srnm'); { Handle* }
=======
>>>>>>> origin/cpstrnew

<<<<<<< HEAD

<<<<<<< HEAD
{============================================================================
        Stream Handler
============================================================================}

=======
=======
>>>>>>> origin/cpstrnew
const
	kQTSInfo_InputDeviceName = FourCharCode('innm'); { Handle* }
	kQTSInfo_InputSourceName = FourCharCode('srnm'); { Handle* }


{============================================================================
        Stream Handler
============================================================================}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{
    Server edits are only valid for the current chunk
}
type
	SHServerEditParametersPtr = ^SHServerEditParameters;
	SHServerEditParameters = record
		version: UInt32;
		editRate: Fixed;
		dataStartTime_mediaAxis: TimeValue64;
		dataEndTime_mediaAxis: TimeValue64;
	end;
const
	kSHNoChunkDispatchFlags = 0;
	kSHChunkFlagSyncSample = 1 shl 2;
	kSHChunkFlagDataLoss = 1 shl 4;
	kSHChunkFlagExtended = 1 shl 5;
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
function QTSSourcerGetInfo( inSourcer: QTSSourcer; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_QTSSourcerGetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


const
	kQTSInfo_InputDeviceName = FourCharCode('innm'); { Handle* }
	kQTSInfo_InputSourceName = FourCharCode('srnm'); { Handle* }


{============================================================================
        Stream Handler
============================================================================}

{
    Server edits are only valid for the current chunk
}
type
	SHServerEditParametersPtr = ^SHServerEditParameters;
	SHServerEditParameters = record
		version: UInt32;
		editRate: Fixed;
		dataStartTime_mediaAxis: TimeValue64;
		dataEndTime_mediaAxis: TimeValue64;
	end;
const
<<<<<<< HEAD
	kSHNoChunkDispatchFlags		= 0;
	kSHChunkFlagSyncSample		= $04;
	kSHChunkFlagDataLoss		= $10;
	kSHChunkFlagExtended		= $20;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kSHNoChunkDispatchFlags = 0;
	kSHChunkFlagSyncSample = 1 shl 2;
	kSHChunkFlagDataLoss = 1 shl 4;
	kSHChunkFlagExtended = 1 shl 5;
>>>>>>> origin/fixes_2.4

type
	SHChunkRecordPtr = ^SHChunkRecord;
	SHChunkRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		version: UInt32;
		reserved1: SIGNEDLONG;
		flags: SInt32;
		dataSize: UInt32;
		dataPtr: UInt8Ptr;
		reserved2: SIGNEDLONG;
		reserved3: SIGNEDLONG;
		presentationTime: TimeValue64;

		reserved4: SIGNEDLONG;
		reserved5: SIGNEDLONG;
		serverEditParameters: {const} SHServerEditParametersPtr;
		reserved6: SIGNEDLONG;
		reserved7: SIGNEDLONG;
<<<<<<< HEAD
	end;
const
	kSHNumExtendedDataLongs = 10;

const
	kSHExtendedChunkFlag_HasSampleCount = 1 shl 0;
	kSHExtendedChunkFlag_HasFrameLengths = 1 shl 1;
=======
=======
>>>>>>> origin/fixes_2_2
		version:				UInt32;
		reserved1:				SInt32;
		flags:					SInt32;
		dataSize:				UInt32;
		dataPtr:				Ptr;
		reserved2:				SInt32;
		reserved3:				SInt32;
		presentationTime:		TimeValue64;
		reserved4:				SInt32;
		reserved5:				SInt32;
		serverEditParameters:	SHServerEditParametersPtr;
		reserved6:				SInt32;
		reserved7:				SInt32;
=======
>>>>>>> origin/fixes_2.4
	end;
const
	kSHNumExtendedDataLongs = 10;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kSHExtendedChunkFlag_HasSampleCount = 1 shl 0;
	kSHExtendedChunkFlag_HasFrameLengths = 1 shl 1;
>>>>>>> origin/fixes_2.4

type
	SHExtendedChunkRecordPtr = ^SHExtendedChunkRecord;
	SHExtendedChunkRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		chunk: SHChunkRecord;
		extendedFlags: SInt32;
		extendedData: array [0..9] of SInt32;
	end;


{============================================================================
        RTP Components
============================================================================}
<<<<<<< HEAD

type
	RTPSSRC = UInt32;
const
	kRTPInvalidSSRC = 0;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ RTP standard content encodings for audio }
const
	kRTPPayload_PCMU = 0;    { 8kHz PCM mu-law mono }
	kRTPPayload_1016 = 1;    { 8kHz CELP (Fed Std 1016) mono }
	kRTPPayload_G721 = 2;    { 8kHz G.721 ADPCM mono }
	kRTPPayload_GSM = 3;    { 8kHz GSM mono }
	kRTPPayload_G723 = 4;    { 8kHz G.723 ADPCM mono }
	kRTPPayload_DVI_8 = 5;    { 8kHz Intel DVI ADPCM mono }
	kRTPPayload_DVI_16 = 6;    { 16kHz Intel DVI ADPCM mono }
	kRTPPayload_LPC = 7;    { 8kHz LPC }
	kRTPPayload_PCMA = 8;    { 8kHz PCM a-law mono }
	kRTPPayload_L16_44_2 = 10;   { 44.1kHz 16-bit linear stereo }
	kRTPPayload_L16_44_1 = 11;   { 44.1kHz 16-bit linear mono }
	kRTPPayload_PureVoice = 12;   { 8kHz PureVoice mono (QCELP) }
	kRTPPayload_MPEGAUDIO = 14;   { MPEG I and II audio }
	kRTPPayload_DVI_11 = 16;   { 11kHz Intel DVI ADPCM mono }
	kRTPPayload_DVI_22 = 17;    { 22kHz Intel DVI ADPCM mono }

{ RTP standard content encodings for video }
const
	kRTPPayload_CELLB = 25;   { Sun CellB }
	kRTPPayload_JPEG = 26;   { JPEG }
	kRTPPayload_CUSEEME = 27;   { Cornell CU-SeeMe }
	kRTPPayload_NV = 28;   { Xerox PARC nv }
	kRTPPayload_PICWIN = 29;   { BBN Picture Window }
	kRTPPayload_CPV = 30;   { Bolter CPV }
	kRTPPayload_H261 = 31;   { CCITT H.261 }
	kRTPPayload_MPEGVIDEO = 32;   { MPEG I and II video }
	kRTPPayload_H263 = 34;    { CCITT H.263 }

{ Other RTP standard content encodings }
const
	kRTPPayload_MPEG2T = 33;    { MPEG 2 Transport }

{ Dynamic encodings }
const
	kRTPPayload_FirstDynamic = 96;
	kRTPPayload_LastDynamic = 127;
	kRTPPayload_Unknown = $FF;


{
-----------------------------------------
    RTP Info selectors
-----------------------------------------
}
{ ----- these are get and set ----- }
const
	kRTPInfo_SSRC = FourCharCode('ssrc'); { UInt32* }
	kRTPInfo_NextSeqNum = FourCharCode('rnsn'); { UInt16* }

{-----------------------------------------
    RTP Statistics
-----------------------------------------}
const
<<<<<<< HEAD
<<<<<<< HEAD
=======

{ RTP standard content encodings for audio }
const
	kRTPPayload_PCMU = 0;    { 8kHz PCM mu-law mono }
	kRTPPayload_1016 = 1;    { 8kHz CELP (Fed Std 1016) mono }
	kRTPPayload_G721 = 2;    { 8kHz G.721 ADPCM mono }
	kRTPPayload_GSM = 3;    { 8kHz GSM mono }
	kRTPPayload_G723 = 4;    { 8kHz G.723 ADPCM mono }
	kRTPPayload_DVI_8 = 5;    { 8kHz Intel DVI ADPCM mono }
	kRTPPayload_DVI_16 = 6;    { 16kHz Intel DVI ADPCM mono }
	kRTPPayload_LPC = 7;    { 8kHz LPC }
	kRTPPayload_PCMA = 8;    { 8kHz PCM a-law mono }
	kRTPPayload_L16_44_2 = 10;   { 44.1kHz 16-bit linear stereo }
	kRTPPayload_L16_44_1 = 11;   { 44.1kHz 16-bit linear mono }
	kRTPPayload_PureVoice = 12;   { 8kHz PureVoice mono (QCELP) }
	kRTPPayload_MPEGAUDIO = 14;   { MPEG I and II audio }
	kRTPPayload_DVI_11 = 16;   { 11kHz Intel DVI ADPCM mono }
	kRTPPayload_DVI_22 = 17;    { 22kHz Intel DVI ADPCM mono }

{ RTP standard content encodings for video }
const
	kRTPPayload_CELLB = 25;   { Sun CellB }
	kRTPPayload_JPEG = 26;   { JPEG }
	kRTPPayload_CUSEEME = 27;   { Cornell CU-SeeMe }
	kRTPPayload_NV = 28;   { Xerox PARC nv }
	kRTPPayload_PICWIN = 29;   { BBN Picture Window }
	kRTPPayload_CPV = 30;   { Bolter CPV }
	kRTPPayload_H261 = 31;   { CCITT H.261 }
	kRTPPayload_MPEGVIDEO = 32;   { MPEG I and II video }
	kRTPPayload_H263 = 34;    { CCITT H.263 }

{ Other RTP standard content encodings }
const
	kRTPPayload_MPEG2T = 33;    { MPEG 2 Transport }

{ Dynamic encodings }
const
	kRTPPayload_FirstDynamic = 96;
	kRTPPayload_LastDynamic = 127;
	kRTPPayload_Unknown = $FF;


{
-----------------------------------------
    RTP Info selectors
-----------------------------------------
}
{ ----- these are get and set ----- }
const
	kRTPInfo_SSRC = FourCharCode('ssrc'); { UInt32* }
	kRTPInfo_NextSeqNum = FourCharCode('rnsn'); { UInt16* }

{-----------------------------------------
    RTP Statistics
-----------------------------------------}
const
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======

type
	RTPSSRC = UInt32;
const
	kRTPInvalidSSRC = 0;


{ RTP standard content encodings for audio }
const
	kRTPPayload_PCMU = 0;    { 8kHz PCM mu-law mono }
	kRTPPayload_1016 = 1;    { 8kHz CELP (Fed Std 1016) mono }
	kRTPPayload_G721 = 2;    { 8kHz G.721 ADPCM mono }
	kRTPPayload_GSM = 3;    { 8kHz GSM mono }
	kRTPPayload_G723 = 4;    { 8kHz G.723 ADPCM mono }
	kRTPPayload_DVI_8 = 5;    { 8kHz Intel DVI ADPCM mono }
	kRTPPayload_DVI_16 = 6;    { 16kHz Intel DVI ADPCM mono }
	kRTPPayload_LPC = 7;    { 8kHz LPC }
	kRTPPayload_PCMA = 8;    { 8kHz PCM a-law mono }
	kRTPPayload_L16_44_2 = 10;   { 44.1kHz 16-bit linear stereo }
	kRTPPayload_L16_44_1 = 11;   { 44.1kHz 16-bit linear mono }
	kRTPPayload_PureVoice = 12;   { 8kHz PureVoice mono (QCELP) }
	kRTPPayload_MPEGAUDIO = 14;   { MPEG I and II audio }
	kRTPPayload_DVI_11 = 16;   { 11kHz Intel DVI ADPCM mono }
	kRTPPayload_DVI_22 = 17;    { 22kHz Intel DVI ADPCM mono }

{ RTP standard content encodings for video }
const
	kRTPPayload_CELLB = 25;   { Sun CellB }
	kRTPPayload_JPEG = 26;   { JPEG }
	kRTPPayload_CUSEEME = 27;   { Cornell CU-SeeMe }
	kRTPPayload_NV = 28;   { Xerox PARC nv }
	kRTPPayload_PICWIN = 29;   { BBN Picture Window }
	kRTPPayload_CPV = 30;   { Bolter CPV }
	kRTPPayload_H261 = 31;   { CCITT H.261 }
	kRTPPayload_MPEGVIDEO = 32;   { MPEG I and II video }
	kRTPPayload_H263 = 34;    { CCITT H.263 }

{ Other RTP standard content encodings }
const
	kRTPPayload_MPEG2T = 33;    { MPEG 2 Transport }

{ Dynamic encodings }
const
	kRTPPayload_FirstDynamic = 96;
	kRTPPayload_LastDynamic = 127;
	kRTPPayload_Unknown = $FF;


{
-----------------------------------------
    RTP Info selectors
-----------------------------------------
}
{ ----- these are get and set ----- }
const
	kRTPInfo_SSRC = FourCharCode('ssrc'); { UInt32* }
	kRTPInfo_NextSeqNum = FourCharCode('rnsn'); { UInt16* }

{-----------------------------------------
    RTP Statistics
-----------------------------------------}
const
>>>>>>> origin/fixes_2.4
	kRTPTotalReceivedPktsStat = FourCharCode('trcp');
	kRTPTotalLostPktsStat = FourCharCode('tlsp');
	kRTPTotalProcessedPktsStat = FourCharCode('tprp');
	kRTPTotalDroppedPktsStat = FourCharCode('tdrp');
<<<<<<< HEAD
=======
		chunk:					SHChunkRecord;
		extendedFlags:			SInt32;
		extendedData:			array [0..9] of SInt32;
	end;


	{	============================================================================
	        RTP Components
	============================================================================	}
	RTPSSRC								= UInt32;

const
	kRTPInvalidSSRC				= 0;
=======
		chunk:					SHChunkRecord;
		extendedFlags:			SInt32;
		extendedData:			array [0..9] of SInt32;
	end;


	{	============================================================================
	        RTP Components
	============================================================================	}
	RTPSSRC								= UInt32;

const
	kRTPInvalidSSRC				= 0;
>>>>>>> origin/fixes_2_2


	{	 RTP standard content encodings for audio 	}
	kRTPPayload_PCMU			= 0;							{  8kHz PCM mu-law mono  }
	kRTPPayload_1016			= 1;							{  8kHz CELP (Fed Std 1016) mono  }
	kRTPPayload_G721			= 2;							{  8kHz G.721 ADPCM mono  }
	kRTPPayload_GSM				= 3;							{  8kHz GSM mono  }
	kRTPPayload_G723			= 4;							{  8kHz G.723 ADPCM mono  }
	kRTPPayload_DVI_8			= 5;							{  8kHz Intel DVI ADPCM mono  }
	kRTPPayload_DVI_16			= 6;							{  16kHz Intel DVI ADPCM mono  }
	kRTPPayload_LPC				= 7;							{  8kHz LPC  }
	kRTPPayload_PCMA			= 8;							{  8kHz PCM a-law mono  }
	kRTPPayload_L16_44_2		= 10;							{  44.1kHz 16-bit linear stereo  }
	kRTPPayload_L16_44_1		= 11;							{  44.1kHz 16-bit linear mono  }
	kRTPPayload_PureVoice		= 12;							{  8kHz PureVoice mono (QCELP)  }
	kRTPPayload_MPEGAUDIO		= 14;							{  MPEG I and II audio  }
	kRTPPayload_DVI_11			= 16;							{  11kHz Intel DVI ADPCM mono  }
	kRTPPayload_DVI_22			= 17;							{  22kHz Intel DVI ADPCM mono  }

	{	 RTP standard content encodings for video 	}
	kRTPPayload_CELLB			= 25;							{  Sun CellB  }
	kRTPPayload_JPEG			= 26;							{  JPEG  }
	kRTPPayload_CUSEEME			= 27;							{  Cornell CU-SeeMe  }
	kRTPPayload_NV				= 28;							{  Xerox PARC nv  }
	kRTPPayload_PICWIN			= 29;							{  BBN Picture Window  }
	kRTPPayload_CPV				= 30;							{  Bolter CPV  }
	kRTPPayload_H261			= 31;							{  CCITT H.261  }
	kRTPPayload_MPEGVIDEO		= 32;							{  MPEG I and II video  }
	kRTPPayload_H263			= 34;							{  CCITT H.263  }

	{	 Other RTP standard content encodings 	}
	kRTPPayload_MPEG2T			= 33;							{  MPEG 2 Transport  }

	{	 Dynamic encodings 	}
	kRTPPayload_FirstDynamic	= 96;
	kRTPPayload_LastDynamic		= 127;
	kRTPPayload_Unknown			= $FF;


	{	
	-----------------------------------------
	    RTP Info selectors
	-----------------------------------------
		}
	{	 ----- these are get and set ----- 	}
	kRTPInfo_SSRC				= FourCharCode('ssrc');						{  UInt32*  }
	kRTPInfo_NextSeqNum			= FourCharCode('rnsn');						{  UInt16*  }

	{	-----------------------------------------
	    RTP Statistics
	-----------------------------------------	}
	kRTPTotalReceivedPktsStat	= FourCharCode('trcp');
	kRTPTotalLostPktsStat		= FourCharCode('tlsp');
	kRTPTotalProcessedPktsStat	= FourCharCode('tprp');
	kRTPTotalDroppedPktsStat	= FourCharCode('tdrp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kRTPBadHeaderDroppedPktsStat = FourCharCode('bhdp');
	kRTPOurHeaderDroppedPktsStat = FourCharCode('ohdp');
	kRTPNotReceivingSenderDroppedPktsStat = FourCharCode('nsdp');
	kRTPNotProcessingDroppedPktsStat = FourCharCode('npdp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kRTPBadSeqDroppedPktsStat = FourCharCode('bsdp');
	kRTPArriveTooLatePktsStat = FourCharCode('artl');
	kRTPWaitForSeqDroppedPktsStat = FourCharCode('wsdp');
	kRTPBadStateDroppedPktsStat = FourCharCode('stdp');
	kRTPBadPayloadDroppedPktsStat = FourCharCode('bpdp');
	kRTPNoTimeScaleDroppedPktsStat = FourCharCode('ntdp');
	kRTPDupSeqNumDroppedPktsStat = FourCharCode('dsdp');
	kRTPLostPktsPercentStat = FourCharCode('lspp');
	kRTPDroppedPktsPercentStat = FourCharCode('dppp');
	kRTPTotalUnprocessedPktsPercentStat = FourCharCode('tupp');
	kRTPRTCPDataRateStat = FourCharCode('rrcd');
	kRTPPayloadIDStat = FourCharCode('rpid');
	kRTPPayloadNameStat = FourCharCode('rpnm');
	kRTPNumPktsInQueueStat = FourCharCode('rnpq');
	kRTPTotalPktsInQueueStat = FourCharCode('rtpq');
	kRTPTotalOutOfOrderPktsStat = FourCharCode('rtoo');
	kRTPRetransmissionStat = FourCharCode('rrtx');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/cpstrnew
=======
=======

>>>>>>> graemeg/cpstrnew

{-----------------------------------------
    Payload Info
-----------------------------------------}
const
	kRTPPayloadSpeedTag = FourCharCode('sped'); { 0-255, 255 is fastest}
	kRTPPayloadLossRecoveryTag = FourCharCode('loss'); { 0-255, 0 can't handle any loss, 128 can handle 50% packet loss}
	kRTPPayloadConformanceTag = FourCharCode('conf'); { more than one of these can be present}

>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
{-----------------------------------------
    Payload Info
-----------------------------------------}
const
	kRTPPayloadSpeedTag = FourCharCode('sped'); { 0-255, 255 is fastest}
	kRTPPayloadLossRecoveryTag = FourCharCode('loss'); { 0-255, 0 can't handle any loss, 128 can handle 50% packet loss}
	kRTPPayloadConformanceTag = FourCharCode('conf'); { more than one of these can be present}
=======
=======
>>>>>>> origin/fixes_2_2
	kRTPBadSeqDroppedPktsStat	= FourCharCode('bsdp');
	kRTPArriveTooLatePktsStat	= FourCharCode('artl');
=======
	kRTPBadSeqDroppedPktsStat = FourCharCode('bsdp');
	kRTPArriveTooLatePktsStat = FourCharCode('artl');
>>>>>>> origin/fixes_2.4
	kRTPWaitForSeqDroppedPktsStat = FourCharCode('wsdp');
	kRTPBadStateDroppedPktsStat = FourCharCode('stdp');
	kRTPBadPayloadDroppedPktsStat = FourCharCode('bpdp');
	kRTPNoTimeScaleDroppedPktsStat = FourCharCode('ntdp');
	kRTPDupSeqNumDroppedPktsStat = FourCharCode('dsdp');
	kRTPLostPktsPercentStat = FourCharCode('lspp');
	kRTPDroppedPktsPercentStat = FourCharCode('dppp');
	kRTPTotalUnprocessedPktsPercentStat = FourCharCode('tupp');
	kRTPRTCPDataRateStat = FourCharCode('rrcd');
	kRTPPayloadIDStat = FourCharCode('rpid');
	kRTPPayloadNameStat = FourCharCode('rpnm');
	kRTPNumPktsInQueueStat = FourCharCode('rnpq');
	kRTPTotalPktsInQueueStat = FourCharCode('rtpq');
	kRTPTotalOutOfOrderPktsStat = FourCharCode('rtoo');
	kRTPRetransmissionStat = FourCharCode('rrtx');


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{-----------------------------------------
    Payload Info
-----------------------------------------}
const
	kRTPPayloadSpeedTag = FourCharCode('sped'); { 0-255, 255 is fastest}
	kRTPPayloadLossRecoveryTag = FourCharCode('loss'); { 0-255, 0 can't handle any loss, 128 can handle 50% packet loss}
	kRTPPayloadConformanceTag = FourCharCode('conf'); { more than one of these can be present}
>>>>>>> origin/fixes_2.4

type
	RTPPayloadCharacteristicPtr = ^RTPPayloadCharacteristic;
	RTPPayloadCharacteristic = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		tag: OSType;
		value: SIGNEDLONG;
	end;
{
    pass RTPPayloadSortRequest to QTSFindMediaPacketizer or QTSFindMediaPacketizerForTrack.
    define the characteristics to sort by. tag is key to sort on. value is positive for ascending
    sort (low value first), negative for descending sort (high value first).
}
type
	RTPPayloadSortRequest = record
		characteristicCount: SIGNEDLONG;
		characteristic: array [0..0] of RTPPayloadCharacteristic; { tag is key to sort on, value is + for ascending, - for descending}
	end;
	RTPPayloadSortRequestPtr = ^RTPPayloadSortRequest;
{ flags for RTPPayloadInfo }
const
	kRTPPayloadTypeStaticFlag = $00000001;
	kRTPPayloadTypeDynamicFlag = $00000002;

type
	RTPPayloadInfo = record
		payloadFlags: SIGNEDLONG;
		payloadID: UInt8;
		reserved1: SInt8;
		reserved2: SInt8;
		reserved3: SInt8;
    payloadName: array [0..1] of SInt8;
	end;
	RTPPayloadInfoPtr = ^RTPPayloadInfo;
type
	RTPPayloadInfoHandle = ^RTPPayloadInfoPtr;
{============================================================================
        RTP Reassembler
============================================================================}
type
	RTPReassembler = ComponentInstance;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
const
	kRTPReassemblerType = FourCharCode('rtpr');

const
<<<<<<< HEAD
=======
const
	kRTPReassemblerType = FourCharCode('rtpr');

const
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kRTPBaseReassemblerType = FourCharCode('gnrc');
	kRTP261ReassemblerType = FourCharCode('h261');
	kRTP263ReassemblerType = FourCharCode('h263');
	kRTP263PlusReassemblerType = FourCharCode('263+');
	kRTPAudioReassemblerType = FourCharCode('soun');
	kRTPQTReassemblerType = FourCharCode('qtim');
	kRTPPureVoiceReassemblerType = FourCharCode('Qclp');
	kRTPJPEGReassemblerType = FourCharCode('jpeg');
	kRTPQDesign2ReassemblerType = FourCharCode('QDM2');
	kRTPSorensonReassemblerType = FourCharCode('SVQ1');
	kRTPMP3ReassemblerType = FourCharCode('mp3 ');
	kRTPMPEG4AudioReassemblerType = FourCharCode('mp4a');
	kRTPMPEG4VideoReassemblerType = FourCharCode('mp4v');

type
	RTPRssmInitParamsPtr = ^RTPRssmInitParams;
	RTPRssmInitParams = record
		ssrc: RTPSSRC;
		payloadType: UInt8;
		reserved1: UInt8;
		reserved2: UInt8;
		reserved3: UInt8;
		timeBase_: TimeBase;
		timeScale_: TimeScale;
	end;
type
	RTPDescParamsPtr = ^RTPDescParams;
	RTPDescParams = record
		container: QTAtomContainer;
		presentationParentAtom: QTAtom;
		streamParentAtom: QTAtom;
	end;
type
	RTPRssmMoreInitParamsPtr = ^RTPRssmMoreInitParams;
	RTPRssmMoreInitParams = record
		initParams: RTPRssmInitParams;
		version: SInt32;
		desc: RTPDescParams;
	end;
=======
=======
>>>>>>> origin/fixes_2_2
		tag:					OSType;
		value:					SInt32;
=======
		tag: OSType;
		value: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
	end;
{
    pass RTPPayloadSortRequest to QTSFindMediaPacketizer or QTSFindMediaPacketizerForTrack.
    define the characteristics to sort by. tag is key to sort on. value is positive for ascending
    sort (low value first), negative for descending sort (high value first).
}
type
	RTPPayloadSortRequest = record
		characteristicCount: SIGNEDLONG;
		characteristic: array [0..0] of RTPPayloadCharacteristic; { tag is key to sort on, value is + for ascending, - for descending}
	end;
	RTPPayloadSortRequestPtr = ^RTPPayloadSortRequest;
{ flags for RTPPayloadInfo }
const
	kRTPPayloadTypeStaticFlag = $00000001;
	kRTPPayloadTypeDynamicFlag = $00000002;

type
	RTPPayloadInfo = record
		payloadFlags: SIGNEDLONG;
		payloadID: UInt8;
		reserved1: SInt8;
		reserved2: SInt8;
		reserved3: SInt8;
    payloadName: array [0..1] of SInt8;
	end;
	RTPPayloadInfoPtr = ^RTPPayloadInfo;
type
	RTPPayloadInfoHandle = ^RTPPayloadInfoPtr;
{============================================================================
        RTP Reassembler
============================================================================}
type
	RTPReassembler = ComponentInstance;
const
	kRTPReassemblerType = FourCharCode('rtpr');

const
	kRTPBaseReassemblerType = FourCharCode('gnrc');
	kRTP261ReassemblerType = FourCharCode('h261');
	kRTP263ReassemblerType = FourCharCode('h263');
	kRTP263PlusReassemblerType = FourCharCode('263+');
	kRTPAudioReassemblerType = FourCharCode('soun');
	kRTPQTReassemblerType = FourCharCode('qtim');
	kRTPPureVoiceReassemblerType = FourCharCode('Qclp');
	kRTPJPEGReassemblerType = FourCharCode('jpeg');
	kRTPQDesign2ReassemblerType = FourCharCode('QDM2');
	kRTPSorensonReassemblerType = FourCharCode('SVQ1');
	kRTPMP3ReassemblerType = FourCharCode('mp3 ');
	kRTPMPEG4AudioReassemblerType = FourCharCode('mp4a');
	kRTPMPEG4VideoReassemblerType = FourCharCode('mp4v');

type
	RTPRssmInitParamsPtr = ^RTPRssmInitParams;
	RTPRssmInitParams = record
		ssrc: RTPSSRC;
		payloadType: UInt8;
		reserved1: UInt8;
		reserved2: UInt8;
		reserved3: UInt8;
		timeBase_: TimeBase;
		timeScale_: TimeScale;
	end;
type
	RTPDescParamsPtr = ^RTPDescParams;
	RTPDescParams = record
		container: QTAtomContainer;
		presentationParentAtom: QTAtom;
		streamParentAtom: QTAtom;
	end;
type
	RTPRssmMoreInitParamsPtr = ^RTPRssmMoreInitParams;
	RTPRssmMoreInitParams = record
		initParams: RTPRssmInitParams;
		version: SInt32;
		desc: RTPDescParams;
	end;
<<<<<<< HEAD


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kRTPRssmMoreInitParamsVersion1 = 1;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ get/set info selectors}
const
	kRTPRssmInfo_MoreInitParams = FourCharCode('rrmi');
=======
	{  get/set info selectors }
	kRTPRssmInfo_MoreInitParams	= FourCharCode('rrmi');
>>>>>>> graemeg/fixes_2_2
=======
	{  get/set info selectors }
	kRTPRssmInfo_MoreInitParams	= FourCharCode('rrmi');
>>>>>>> origin/fixes_2_2
=======
{ get/set info selectors}
const
	kRTPRssmInfo_MoreInitParams = FourCharCode('rrmi');
>>>>>>> origin/fixes_2.4


type
	RTPRssmPacketPtr = ^RTPRssmPacket;
	RTPRssmPacket = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		next: RTPRssmPacketPtr;
		prev: RTPRssmPacketPtr;
		streamBuffer: QTSStreamBufferPtr;
		paramsFilledIn: Boolean;
		reserved: UInt8;
		sequenceNum: UInt16;
		transportHeaderLength: UInt32;  { filled in by base}
		payloadHeaderLength: UInt32;    { derived adjusts this }
		dataLength: UInt32;
		serverEditParams: SHServerEditParameters;
		timeStamp: TimeValue64;              { lower 32 bits is original rtp timestamp}
		chunkFlags: SInt32;             { these are or'd together}
		flags: SInt32;
<<<<<<< HEAD
	end;
{ flags for RTPRssmPacket struct}
=======
=======
>>>>>>> origin/fixes_2_2
		next:					RTPRssmPacketPtr;
		prev:					RTPRssmPacketPtr;
		streamBuffer:			QTSStreamBufferPtr;
		paramsFilledIn:			boolean;
		reserved:				SInt8;
		sequenceNum:			UInt16;
		transportHeaderLength:	UInt32;									{  filled in by base }
		payloadHeaderLength:	UInt32;									{  derived adjusts this  }
		dataLength:				UInt32;
		serverEditParams:		SHServerEditParameters;
		timeStamp:				TimeValue64;							{  lower 32 bits is original rtp timestamp }
		chunkFlags:				SInt32;									{  these are or'd together }
		flags:					SInt32;
	end;

	{  flags for RTPRssmPacket struct }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	end;
{ flags for RTPRssmPacket struct}
>>>>>>> origin/fixes_2.4
const
	kRTPRssmPacketHasMarkerBitSet = $00000001;
	kRTPRssmPacketHasServerEditFlag = $00010000;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ flags for RTPRssmSendStreamBufferRange}
const
	kRTPRssmCanRefStreamBuffer = $00000001;

{ flags for RTPRssmSendPacketList}
const
	kRTPRssmLostSomePackets = $00000001;

{ flags for RTPRssmSetFlags}
const
=======
=======
>>>>>>> origin/fixes_2_2
	{  flags for RTPRssmSendStreamBufferRange }
	kRTPRssmCanRefStreamBuffer	= $00000001;
=======
{ flags for RTPRssmSendStreamBufferRange}
const
	kRTPRssmCanRefStreamBuffer = $00000001;
>>>>>>> origin/fixes_2.4

{ flags for RTPRssmSendPacketList}
const
	kRTPRssmLostSomePackets = $00000001;

<<<<<<< HEAD
	{  flags for RTPRssmSetFlags }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ flags for RTPRssmSetFlags}
const
>>>>>>> origin/fixes_2.4
	kRTPRssmEveryPacketAChunkFlag = $00000001;
	kRTPRssmQueueAndUseMarkerBitFlag = $00000002;
	kRTPRssmTrackLostPacketsFlag = $00010000;
	kRTPRssmNoReorderingRequiredFlag = $00020000;


type
	RTPSendStreamBufferRangeParamsPtr = ^RTPSendStreamBufferRangeParams;
	RTPSendStreamBufferRangeParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		streamBuffer: QTSStreamBufferPtr;
		presentationTime: TimeValue64;
		chunkStartPosition: UInt32;
		numDataBytes: UInt32;
		chunkFlags: SInt32;
		flags: SInt32;
		serverEditParams: {const} SHServerEditParametersPtr; { NULL if no edit}
<<<<<<< HEAD
	end;
{ characteristics}
=======
=======
>>>>>>> origin/fixes_2_2
		streamBuffer:			QTSStreamBufferPtr;
		presentationTime:		TimeValue64;
		chunkStartPosition:		UInt32;
		numDataBytes:			UInt32;
		chunkFlags:				SInt32;
		flags:					SInt32;
		serverEditParams:		SHServerEditParametersPtr;				{  NULL if no edit }
	end;

	{  characteristics }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	end;
{ characteristics}
>>>>>>> origin/fixes_2.4
const
	kRTPCharacteristic_RequiresOrderedPackets = FourCharCode('rrop');
	kRTPCharacteristic_TimeStampsNotMonoIncreasing = FourCharCode('tsmi');


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kRTPReassemblerInfoResType = FourCharCode('rsmi');

type
	RTPReassemblerInfo = record
		characteristicCount: SIGNEDLONG;
		characteristic: array [0..0] of RTPPayloadCharacteristic;

                                              { after the last characteristic, the payload name (defined by the MediaPacketizerPayloadInfo}
                                              { structure) is present. }
	end;
	RTPReassemblerInfoPtr = ^RTPReassemblerInfo;
type
	RTPReassemblerInfoHandle = ^RTPReassemblerInfoPtr;
	{	 RTPReassemblerInfoElement structs are padded to 32 bits 	}
=======
=======
>>>>>>> origin/fixes_2_2
	kRTPReassemblerInfoResType	= FourCharCode('rsmi');

=======
const
	kRTPReassemblerInfoResType = FourCharCode('rsmi');
>>>>>>> origin/fixes_2.4

type
	RTPReassemblerInfo = record
		characteristicCount: SIGNEDLONG;
		characteristic: array [0..0] of RTPPayloadCharacteristic;

                                              { after the last characteristic, the payload name (defined by the MediaPacketizerPayloadInfo}
                                              { structure) is present. }
	end;
	RTPReassemblerInfoPtr = ^RTPReassemblerInfo;
type
	RTPReassemblerInfoHandle = ^RTPReassemblerInfoPtr;
	{	 RTPReassemblerInfoElement structs are padded to 32 bits 	}
<<<<<<< HEAD

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kRTPReassemblerInfoPadUpToBytes = 4;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
 *  QTSFindReassemblerForPayloadID()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSFindReassemblerForPayloadID( inPayloadID: UInt8; var inSortInfo: RTPPayloadSortRequest; var outReassemblerList: QTAtomContainer ): OSErr; external name '_QTSFindReassemblerForPayloadID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  QTSFindReassemblerForPayloadID()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.0 and later
	 	}
function QTSFindReassemblerForPayloadID(inPayloadID: ByteParameter; var inSortInfo: RTPPayloadSortRequest; var outReassemblerList: QTAtomContainer): OSErr; external name '_QTSFindReassemblerForPayloadID';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  QTSFindReassemblerForPayloadName()
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
function QTSFindReassemblerForPayloadName( inPayloadName: ConstCStringPtr; var inSortInfo: RTPPayloadSortRequest; var outReassemblerList: QTAtomContainer ): OSErr; external name '_QTSFindReassemblerForPayloadName';
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
function QTSFindReassemblerForPayloadName(inPayloadName: ConstCStringPtr; var inSortInfo: RTPPayloadSortRequest; var outReassemblerList: QTAtomContainer): OSErr; external name '_QTSFindReassemblerForPayloadName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSFindReassemblerForPayloadName( inPayloadName: ConstCStringPtr; var inSortInfo: RTPPayloadSortRequest; var outReassemblerList: QTAtomContainer ): OSErr; external name '_QTSFindReassemblerForPayloadName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{-----------------------------------------
    RTP Reassembler Selectors
-----------------------------------------}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kRTPRssmSetCapabilitiesSelect = $0100;
	kRTPRssmGetCapabilitiesSelect = $0101;
	kRTPRssmSetPayloadHeaderLengthSelect = $0102;
	kRTPRssmGetPayloadHeaderLengthSelect = $0103;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kRTPRssmSetTimeScaleSelect = $0104;
	kRTPRssmGetTimeScaleSelect = $0105;
=======
	kRTPRssmSetTimeScaleSelect	= $0104;
	kRTPRssmGetTimeScaleSelect	= $0105;
>>>>>>> graemeg/fixes_2_2
=======
	kRTPRssmSetTimeScaleSelect	= $0104;
	kRTPRssmGetTimeScaleSelect	= $0105;
>>>>>>> origin/fixes_2_2
=======
	kRTPRssmSetTimeScaleSelect = $0104;
	kRTPRssmGetTimeScaleSelect = $0105;
>>>>>>> origin/fixes_2.4
	kRTPRssmNewStreamHandlerSelect = $0106;
	kRTPRssmSetStreamHandlerSelect = $0107;
	kRTPRssmGetStreamHandlerSelect = $0108;
	kRTPRssmSendStreamHandlerChangedSelect = $0109;
	kRTPRssmSetSampleDescriptionSelect = $010A;
	kRTPRssmGetChunkAndIncrRefCountSelect = $010D;
	kRTPRssmSendChunkAndDecrRefCountSelect = $010E;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kRTPRssmSendLostChunkSelect = $010F;
	kRTPRssmSendStreamBufferRangeSelect = $0110;
	kRTPRssmClearCachedPackets = $0111;
=======
	kRTPRssmSendLostChunkSelect	= $010F;
	kRTPRssmSendStreamBufferRangeSelect = $0110;
	kRTPRssmClearCachedPackets	= $0111;
>>>>>>> graemeg/fixes_2_2
=======
	kRTPRssmSendLostChunkSelect	= $010F;
	kRTPRssmSendStreamBufferRangeSelect = $0110;
	kRTPRssmClearCachedPackets	= $0111;
>>>>>>> origin/fixes_2_2
=======
	kRTPRssmSendLostChunkSelect = $010F;
	kRTPRssmSendStreamBufferRangeSelect = $0110;
	kRTPRssmClearCachedPackets = $0111;
>>>>>>> origin/fixes_2.4
	kRTPRssmFillPacketListParamsSelect = $0113;
	kRTPRssmReleasePacketListSelect = $0114;
	kRTPRssmIncrChunkRefCountSelect = $0115;
	kRTPRssmDecrChunkRefCountSelect = $0116;
	kRTPRssmGetExtChunkAndIncrRefCountSelect = $0117;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kRTPRssmInitializeSelect = $0500;
=======
	kRTPRssmInitializeSelect	= $0500;
>>>>>>> graemeg/fixes_2_2
=======
	kRTPRssmInitializeSelect	= $0500;
>>>>>>> origin/fixes_2_2
=======
	kRTPRssmInitializeSelect = $0500;
>>>>>>> origin/fixes_2.4
	kRTPRssmHandleNewPacketSelect = $0501;
	kRTPRssmComputeChunkSizeSelect = $0502;
	kRTPRssmAdjustPacketParamsSelect = $0503;
	kRTPRssmCopyDataToChunkSelect = $0504;
	kRTPRssmSendPacketListSelect = $0505;
	kRTPRssmGetTimeScaleFromPacketSelect = $0506;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kRTPRssmSetInfoSelect = $0509;
	kRTPRssmGetInfoSelect = $050A;
	kRTPRssmHasCharacteristicSelect = $050B;
	kRTPRssmResetSelect = $050C;

{-----------------------------------------
    RTP Reassembler functions - base to derived
-----------------------------------------}

{
 *  RTPRssmInitialize()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function RTPRssmInitialize( rtpr: RTPReassembler; var inInitParams: RTPRssmInitParams ): ComponentResult; external name '_RTPRssmInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
	kRTPRssmSetInfoSelect		= $0509;
	kRTPRssmGetInfoSelect		= $050A;
	kRTPRssmHasCharacteristicSelect = $050B;
	kRTPRssmResetSelect			= $050C;

	{	-----------------------------------------
	    RTP Reassembler functions - base to derived
	-----------------------------------------	}

	{
	 *  RTPRssmInitialize()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.0 and later
	 	}
function RTPRssmInitialize(rtpr: RTPReassembler; var inInitParams: RTPRssmInitParams): ComponentResult; external name '_RTPRssmInitialize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kRTPRssmSetInfoSelect = $0509;
	kRTPRssmGetInfoSelect = $050A;
	kRTPRssmHasCharacteristicSelect = $050B;
	kRTPRssmResetSelect = $050C;

{-----------------------------------------
    RTP Reassembler functions - base to derived
-----------------------------------------}

>>>>>>> origin/fixes_2.4
{
 *  RTPRssmInitialize()
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
function RTPRssmHandleNewPacket( rtpr: RTPReassembler; var inStreamBuffer: QTSStreamBuffer; inNumWraparounds: SInt32 ): ComponentResult; external name '_RTPRssmHandleNewPacket';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function RTPRssmInitialize( rtpr: RTPReassembler; var inInitParams: RTPRssmInitParams ): ComponentResult; external name '_RTPRssmInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RTPRssmHandleNewPacket()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function RTPRssmHandleNewPacket(rtpr: RTPReassembler; var inStreamBuffer: QTSStreamBuffer; inNumWraparounds: SInt32): ComponentResult; external name '_RTPRssmHandleNewPacket';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmHandleNewPacket( rtpr: RTPReassembler; var inStreamBuffer: QTSStreamBuffer; inNumWraparounds: SInt32 ): ComponentResult; external name '_RTPRssmHandleNewPacket';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmComputeChunkSize()
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
function RTPRssmComputeChunkSize( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inFlags: SInt32; var outChunkDataSize: UInt32 ): ComponentResult; external name '_RTPRssmComputeChunkSize';
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
function RTPRssmComputeChunkSize(rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inFlags: SInt32; var outChunkDataSize: UInt32): ComponentResult; external name '_RTPRssmComputeChunkSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmComputeChunkSize( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inFlags: SInt32; var outChunkDataSize: UInt32 ): ComponentResult; external name '_RTPRssmComputeChunkSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmAdjustPacketParams()
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
function RTPRssmAdjustPacketParams( rtpr: RTPReassembler; var inPacket: RTPRssmPacket; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmAdjustPacketParams';
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
function RTPRssmAdjustPacketParams(rtpr: RTPReassembler; var inPacket: RTPRssmPacket; inFlags: SInt32): ComponentResult; external name '_RTPRssmAdjustPacketParams';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmAdjustPacketParams( rtpr: RTPReassembler; var inPacket: RTPRssmPacket; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmAdjustPacketParams';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmCopyDataToChunk()
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
function RTPRssmCopyDataToChunk( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inMaxChunkDataSize: UInt32; var inChunk: SHChunkRecord; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmCopyDataToChunk';
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
function RTPRssmCopyDataToChunk(rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inMaxChunkDataSize: UInt32; var inChunk: SHChunkRecord; inFlags: SInt32): ComponentResult; external name '_RTPRssmCopyDataToChunk';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmCopyDataToChunk( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inMaxChunkDataSize: UInt32; var inChunk: SHChunkRecord; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmCopyDataToChunk';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSendPacketList()
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
function RTPRssmSendPacketList( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; (*const*) var inLastChunkPresentationTime: TimeValue64; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmSendPacketList';
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
function RTPRssmSendPacketList(rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; (*const*) var inLastChunkPresentationTime: TimeValue64; inFlags: SInt32): ComponentResult; external name '_RTPRssmSendPacketList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSendPacketList( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; (*const*) var inLastChunkPresentationTime: TimeValue64; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmSendPacketList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmGetTimeScaleFromPacket()
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
function RTPRssmGetTimeScaleFromPacket( rtpr: RTPReassembler; var inStreamBuffer: QTSStreamBuffer; var outTimeScale: TimeScale ): ComponentResult; external name '_RTPRssmGetTimeScaleFromPacket';
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
function RTPRssmGetTimeScaleFromPacket(rtpr: RTPReassembler; var inStreamBuffer: QTSStreamBuffer; var outTimeScale: TimeScale): ComponentResult; external name '_RTPRssmGetTimeScaleFromPacket';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmGetTimeScaleFromPacket( rtpr: RTPReassembler; var inStreamBuffer: QTSStreamBuffer; var outTimeScale: TimeScale ): ComponentResult; external name '_RTPRssmGetTimeScaleFromPacket';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSetInfo()
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
function RTPRssmSetInfo( rtpr: RTPReassembler; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPRssmSetInfo';
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
function RTPRssmSetInfo(rtpr: RTPReassembler; inSelector: OSType; ioParams: UnivPtr): ComponentResult; external name '_RTPRssmSetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSetInfo( rtpr: RTPReassembler; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPRssmSetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmGetInfo()
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
function RTPRssmGetInfo( rtpr: RTPReassembler; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPRssmGetInfo';
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
function RTPRssmGetInfo(rtpr: RTPReassembler; inSelector: OSType; ioParams: UnivPtr): ComponentResult; external name '_RTPRssmGetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmGetInfo( rtpr: RTPReassembler; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPRssmGetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmHasCharacteristic()
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
function RTPRssmHasCharacteristic( rtpr: RTPReassembler; inCharacteristic: OSType; var outHasIt: Boolean ): ComponentResult; external name '_RTPRssmHasCharacteristic';
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
function RTPRssmHasCharacteristic(rtpr: RTPReassembler; inCharacteristic: OSType; var outHasIt: boolean): ComponentResult; external name '_RTPRssmHasCharacteristic';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmHasCharacteristic( rtpr: RTPReassembler; inCharacteristic: OSType; var outHasIt: Boolean ): ComponentResult; external name '_RTPRssmHasCharacteristic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmReset()
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
function RTPRssmReset( rtpr: RTPReassembler; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmReset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{-----------------------------------------
    RTP Reassembler functions - derived to base
-----------------------------------------}
{ ----- setup}
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
function RTPRssmReset( rtpr: RTPReassembler; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmReset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{-----------------------------------------
    RTP Reassembler functions - derived to base
-----------------------------------------}
<<<<<<< HEAD
{  ----- setup }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ ----- setup}
>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSetCapabilities()
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
function RTPRssmSetCapabilities( rtpr: RTPReassembler; inFlags: SInt32; inFlagsMask: SInt32 ): ComponentResult; external name '_RTPRssmSetCapabilities';
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
function RTPRssmSetCapabilities(rtpr: RTPReassembler; inFlags: SInt32; inFlagsMask: SInt32): ComponentResult; external name '_RTPRssmSetCapabilities';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSetCapabilities( rtpr: RTPReassembler; inFlags: SInt32; inFlagsMask: SInt32 ): ComponentResult; external name '_RTPRssmSetCapabilities';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmGetCapabilities()
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
function RTPRssmGetCapabilities( rtpr: RTPReassembler; var outFlags: SInt32 ): ComponentResult; external name '_RTPRssmGetCapabilities';
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
function RTPRssmGetCapabilities(rtpr: RTPReassembler; var outFlags: SInt32): ComponentResult; external name '_RTPRssmGetCapabilities';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmGetCapabilities( rtpr: RTPReassembler; var outFlags: SInt32 ): ComponentResult; external name '_RTPRssmGetCapabilities';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSetPayloadHeaderLength()
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
function RTPRssmSetPayloadHeaderLength( rtpr: RTPReassembler; inPayloadHeaderLength: UInt32 ): ComponentResult; external name '_RTPRssmSetPayloadHeaderLength';
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
function RTPRssmSetPayloadHeaderLength(rtpr: RTPReassembler; inPayloadHeaderLength: UInt32): ComponentResult; external name '_RTPRssmSetPayloadHeaderLength';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSetPayloadHeaderLength( rtpr: RTPReassembler; inPayloadHeaderLength: UInt32 ): ComponentResult; external name '_RTPRssmSetPayloadHeaderLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmGetPayloadHeaderLength()
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
function RTPRssmGetPayloadHeaderLength( rtpr: RTPReassembler; var outPayloadHeaderLength: UInt32 ): ComponentResult; external name '_RTPRssmGetPayloadHeaderLength';
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
function RTPRssmGetPayloadHeaderLength(rtpr: RTPReassembler; var outPayloadHeaderLength: UInt32): ComponentResult; external name '_RTPRssmGetPayloadHeaderLength';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmGetPayloadHeaderLength( rtpr: RTPReassembler; var outPayloadHeaderLength: UInt32 ): ComponentResult; external name '_RTPRssmGetPayloadHeaderLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSetTimeScale()
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
function RTPRssmSetTimeScale( rtpr: RTPReassembler; inSHTimeScale: TimeScale ): ComponentResult; external name '_RTPRssmSetTimeScale';
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
function RTPRssmSetTimeScale(rtpr: RTPReassembler; inSHTimeScale: TimeScale): ComponentResult; external name '_RTPRssmSetTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSetTimeScale( rtpr: RTPReassembler; inSHTimeScale: TimeScale ): ComponentResult; external name '_RTPRssmSetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmGetTimeScale()
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
function RTPRssmGetTimeScale( rtpr: RTPReassembler; var outSHTimeScale: TimeScale ): ComponentResult; external name '_RTPRssmGetTimeScale';
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
function RTPRssmGetTimeScale(rtpr: RTPReassembler; var outSHTimeScale: TimeScale): ComponentResult; external name '_RTPRssmGetTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmGetTimeScale( rtpr: RTPReassembler; var outSHTimeScale: TimeScale ): ComponentResult; external name '_RTPRssmGetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmNewStreamHandler()
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
function RTPRssmNewStreamHandler( rtpr: RTPReassembler; inSHType: OSType; inSampleDescription: SampleDescriptionHandle; inSHTimeScale: TimeScale; var outHandler: ComponentInstance ): ComponentResult; external name '_RTPRssmNewStreamHandler';
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
function RTPRssmNewStreamHandler(rtpr: RTPReassembler; inSHType: OSType; inSampleDescription: SampleDescriptionHandle; inSHTimeScale: TimeScale; var outHandler: ComponentInstance): ComponentResult; external name '_RTPRssmNewStreamHandler';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmNewStreamHandler( rtpr: RTPReassembler; inSHType: OSType; inSampleDescription: SampleDescriptionHandle; inSHTimeScale: TimeScale; var outHandler: ComponentInstance ): ComponentResult; external name '_RTPRssmNewStreamHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSetStreamHandler()
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
function RTPRssmSetStreamHandler( rtpr: RTPReassembler; inStreamHandler: ComponentInstance ): ComponentResult; external name '_RTPRssmSetStreamHandler';
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
function RTPRssmSetStreamHandler(rtpr: RTPReassembler; inStreamHandler: ComponentInstance): ComponentResult; external name '_RTPRssmSetStreamHandler';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSetStreamHandler( rtpr: RTPReassembler; inStreamHandler: ComponentInstance ): ComponentResult; external name '_RTPRssmSetStreamHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmGetStreamHandler()
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
function RTPRssmGetStreamHandler( rtpr: RTPReassembler; var outStreamHandler: ComponentInstance ): ComponentResult; external name '_RTPRssmGetStreamHandler';
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
function RTPRssmGetStreamHandler(rtpr: RTPReassembler; var outStreamHandler: ComponentInstance): ComponentResult; external name '_RTPRssmGetStreamHandler';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmGetStreamHandler( rtpr: RTPReassembler; var outStreamHandler: ComponentInstance ): ComponentResult; external name '_RTPRssmGetStreamHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSendStreamHandlerChanged()
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
function RTPRssmSendStreamHandlerChanged( rtpr: RTPReassembler ): ComponentResult; external name '_RTPRssmSendStreamHandlerChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function RTPRssmSendStreamHandlerChanged(rtpr: RTPReassembler): ComponentResult; external name '_RTPRssmSendStreamHandlerChanged';
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function RTPRssmSendStreamHandlerChanged(rtpr: RTPReassembler): ComponentResult; external name '_RTPRssmSendStreamHandlerChanged';
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSendStreamHandlerChanged( rtpr: RTPReassembler ): ComponentResult; external name '_RTPRssmSendStreamHandlerChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSetSampleDescription()
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
function RTPRssmSetSampleDescription( rtpr: RTPReassembler; inSampleDescription: SampleDescriptionHandle ): ComponentResult; external name '_RTPRssmSetSampleDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ ----- manually sending chunks}
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
function RTPRssmSetSampleDescription(rtpr: RTPReassembler; inSampleDescription: SampleDescriptionHandle): ComponentResult; external name '_RTPRssmSetSampleDescription';
{  ----- manually sending chunks }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSetSampleDescription( rtpr: RTPReassembler; inSampleDescription: SampleDescriptionHandle ): ComponentResult; external name '_RTPRssmSetSampleDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ ----- manually sending chunks}
>>>>>>> origin/fixes_2.4
{
 *  RTPRssmGetChunkAndIncrRefCount()
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
function RTPRssmGetChunkAndIncrRefCount( rtpr: RTPReassembler; inChunkDataSize: UInt32; (*const*) var inChunkPresentationTime: TimeValue64; var outChunk: UnivPtr ): ComponentResult; external name '_RTPRssmGetChunkAndIncrRefCount';
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
function RTPRssmGetChunkAndIncrRefCount(rtpr: RTPReassembler; inChunkDataSize: UInt32; (*const*) var inChunkPresentationTime: TimeValue64; var outChunk: UnivPtr): ComponentResult; external name '_RTPRssmGetChunkAndIncrRefCount';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmGetChunkAndIncrRefCount( rtpr: RTPReassembler; inChunkDataSize: UInt32; (*const*) var inChunkPresentationTime: TimeValue64; var outChunk: UnivPtr ): ComponentResult; external name '_RTPRssmGetChunkAndIncrRefCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmGetExtChunkAndIncrRefCount()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QTStreamLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function RTPRssmGetExtChunkAndIncrRefCount( rtpr: RTPReassembler; inChunkDataSize: UInt32; (*const*) var inChunkPresentationTime: TimeValue64; inFlags: SInt32; var outChunk: UnivPtr ): ComponentResult; external name '_RTPRssmGetExtChunkAndIncrRefCount';
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
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function RTPRssmGetExtChunkAndIncrRefCount(rtpr: RTPReassembler; inChunkDataSize: UInt32; (*const*) var inChunkPresentationTime: TimeValue64; inFlags: SInt32; var outChunk: UnivPtr): ComponentResult; external name '_RTPRssmGetExtChunkAndIncrRefCount';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmGetExtChunkAndIncrRefCount( rtpr: RTPReassembler; inChunkDataSize: UInt32; (*const*) var inChunkPresentationTime: TimeValue64; inFlags: SInt32; var outChunk: UnivPtr ): ComponentResult; external name '_RTPRssmGetExtChunkAndIncrRefCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSendChunkAndDecrRefCount()
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
function RTPRssmSendChunkAndDecrRefCount( rtpr: RTPReassembler; var inChunk: SHChunkRecord; const (*var*) inServerEdit: SHServerEditParameters ): ComponentResult; external name '_RTPRssmSendChunkAndDecrRefCount';
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
function RTPRssmSendChunkAndDecrRefCount(rtpr: RTPReassembler; var inChunk: SHChunkRecord; const (*var*) inServerEdit: SHServerEditParameters): ComponentResult; external name '_RTPRssmSendChunkAndDecrRefCount';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSendChunkAndDecrRefCount( rtpr: RTPReassembler; var inChunk: SHChunkRecord; const (*var*) inServerEdit: SHServerEditParameters ): ComponentResult; external name '_RTPRssmSendChunkAndDecrRefCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSendLostChunk()
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
function RTPRssmSendLostChunk( rtpr: RTPReassembler; (*const*) var inChunkPresentationTime: TimeValue64 ): ComponentResult; external name '_RTPRssmSendLostChunk';
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
function RTPRssmSendLostChunk(rtpr: RTPReassembler; (*const*) var inChunkPresentationTime: TimeValue64): ComponentResult; external name '_RTPRssmSendLostChunk';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSendLostChunk( rtpr: RTPReassembler; (*const*) var inChunkPresentationTime: TimeValue64 ): ComponentResult; external name '_RTPRssmSendLostChunk';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmSendStreamBufferRange()
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
function RTPRssmSendStreamBufferRange( rtpr: RTPReassembler; var inParams: RTPSendStreamBufferRangeParams ): ComponentResult; external name '_RTPRssmSendStreamBufferRange';
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
function RTPRssmSendStreamBufferRange(rtpr: RTPReassembler; var inParams: RTPSendStreamBufferRangeParams): ComponentResult; external name '_RTPRssmSendStreamBufferRange';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmSendStreamBufferRange( rtpr: RTPReassembler; var inParams: RTPSendStreamBufferRangeParams ): ComponentResult; external name '_RTPRssmSendStreamBufferRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmClearCachedPackets()
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
function RTPRssmClearCachedPackets( rtpr: RTPReassembler; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmClearCachedPackets';
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
function RTPRssmClearCachedPackets(rtpr: RTPReassembler; inFlags: SInt32): ComponentResult; external name '_RTPRssmClearCachedPackets';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmClearCachedPackets( rtpr: RTPReassembler; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmClearCachedPackets';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmFillPacketListParams()
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
function RTPRssmFillPacketListParams( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inNumWraparounds: SInt32; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmFillPacketListParams';
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
function RTPRssmFillPacketListParams(rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inNumWraparounds: SInt32; inFlags: SInt32): ComponentResult; external name '_RTPRssmFillPacketListParams';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmFillPacketListParams( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket; inNumWraparounds: SInt32; inFlags: SInt32 ): ComponentResult; external name '_RTPRssmFillPacketListParams';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmReleasePacketList()
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
function RTPRssmReleasePacketList( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket ): ComponentResult; external name '_RTPRssmReleasePacketList';
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
function RTPRssmReleasePacketList(rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket): ComponentResult; external name '_RTPRssmReleasePacketList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmReleasePacketList( rtpr: RTPReassembler; var inPacketListHead: RTPRssmPacket ): ComponentResult; external name '_RTPRssmReleasePacketList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmIncrChunkRefCount()
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
function RTPRssmIncrChunkRefCount( rtpr: RTPReassembler; var inChunk: SHChunkRecord ): ComponentResult; external name '_RTPRssmIncrChunkRefCount';
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
function RTPRssmIncrChunkRefCount(rtpr: RTPReassembler; var inChunk: SHChunkRecord): ComponentResult; external name '_RTPRssmIncrChunkRefCount';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPRssmIncrChunkRefCount( rtpr: RTPReassembler; var inChunk: SHChunkRecord ): ComponentResult; external name '_RTPRssmIncrChunkRefCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPRssmDecrChunkRefCount()
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
function RTPRssmDecrChunkRefCount( rtpr: RTPReassembler; var inChunk: SHChunkRecord ): ComponentResult; external name '_RTPRssmDecrChunkRefCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{============================================================================
        RTP Media Packetizer
============================================================================}
const
	kRTPMediaPacketizerType = FourCharCode('rtpm');

type
	RTPMediaPacketizer = ComponentInstance;
const
	kRTPBaseMediaPacketizerType = FourCharCode('gnrc');
	kRTP261MediaPacketizerType = FourCharCode('h261');
	kRTP263PlusMediaPacketizerType = FourCharCode('263+');
	kRTPAudioMediaPacketizerType = FourCharCode('soun');
	kRTPQTMediaPacketizerType = FourCharCode('qtim');
	kRTPPureVoiceMediaPacketizerType = FourCharCode('Qclp');
	kRTPJPEGMediaPacketizerType = FourCharCode('jpeg');
	kRTPQDesign2MediaPacketizerType = FourCharCode('QDM2');
	kRTPSorensonMediaPacketizerType = FourCharCode('SVQ1');
	kRTPMP3MediaPacketizerType = FourCharCode('mp3 ');
	kRTPMPEG4AudioMediaPacketizerType = FourCharCode('mp4a');
	kRTPMPEG4VideoMediaPacketizerType = FourCharCode('mp4v');
	kRTPAMRMediaPacketizerType = FourCharCode('amr ');

type
	RTPMPSampleRef = UInt32;
	RTPMPDataReleaseProcPtr = procedure( var inData: UInt8; inRefCon: UnivPtr );
	RTPMPDataReleaseUPP = RTPMPDataReleaseProcPtr;
const
	kMediaPacketizerCanPackEditRate = 1 shl 0;
	kMediaPacketizerCanPackLayer = 1 shl 1;
	kMediaPacketizerCanPackVolume = 1 shl 2;
	kMediaPacketizerCanPackBalance = 1 shl 3;
	kMediaPacketizerCanPackGraphicsMode = 1 shl 4;
	kMediaPacketizerCanPackEmptyEdit = 1 shl 5;


type
	MediaPacketizerRequirements = record
		mediaType: OSType;              { media type supported (0 for all)}
		dataFormat: OSType;             { data format (e.g., compression) supported (0 for all)}
		capabilityFlags: UInt32;        { ability to handle non-standard track characteristics}
		canPackMatrixType: UInt8;      { can pack any matrix type up to this (identityMatrixType for identity only)}
		reserved1: UInt8;
		reserved2: UInt8;
		reserved3: UInt8;
	end;
	MediaPacketizerRequirementsPtr = ^MediaPacketizerRequirements;
type
	MediaPacketizerInfo = record
		mediaType: OSType;              { media type supported (0 for all)}
		dataFormat: OSType;             { data format (e.g., compression) supported (0 for all)}
		vendor: OSType;                 { manufacturer of this packetizer (e.g., 'appl' for Apple)}
		capabilityFlags: UInt32;        { ability to handle non-standard track characteristics}
		canPackMatrixType: UInt8;      { can pack any matrix type up to this (identityMatrixType for identity only)}
		reserved1: UInt8;
		reserved2: UInt8;
		reserved3: UInt8;
		characteristicCount: SIGNEDLONG;
		characteristic: array [0..0] of RTPPayloadCharacteristic;

                                              { after the last characteristic, the payload name (defined by the RTPPayloadInfo}
                                              { structure) is present. }
	end;
	MediaPacketizerInfoPtr = ^MediaPacketizerInfo;
type
	MediaPacketizerInfoHandle = ^MediaPacketizerInfoPtr;
{ MediaPacketizerInfo structs are padded to 32 bits }
const
	kMediaPacketizerInfoPadUpToBytes = 4;

const
	kRTPMediaPacketizerInfoRezType = FourCharCode('pcki');


{
 *  QTSFindMediaPacketizer()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSFindMediaPacketizer( inPacketizerinfo: MediaPacketizerRequirementsPtr; inSampleDescription: SampleDescriptionHandle; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer ): OSErr; external name '_QTSFindMediaPacketizer';
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
function RTPRssmDecrChunkRefCount( rtpr: RTPReassembler; var inChunk: SHChunkRecord ): ComponentResult; external name '_RTPRssmDecrChunkRefCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{============================================================================
        RTP Media Packetizer
============================================================================}
const
	kRTPMediaPacketizerType = FourCharCode('rtpm');

type
	RTPMediaPacketizer = ComponentInstance;
const
	kRTPBaseMediaPacketizerType = FourCharCode('gnrc');
	kRTP261MediaPacketizerType = FourCharCode('h261');
	kRTP263PlusMediaPacketizerType = FourCharCode('263+');
	kRTPAudioMediaPacketizerType = FourCharCode('soun');
	kRTPQTMediaPacketizerType = FourCharCode('qtim');
	kRTPPureVoiceMediaPacketizerType = FourCharCode('Qclp');
	kRTPJPEGMediaPacketizerType = FourCharCode('jpeg');
	kRTPQDesign2MediaPacketizerType = FourCharCode('QDM2');
	kRTPSorensonMediaPacketizerType = FourCharCode('SVQ1');
	kRTPMP3MediaPacketizerType = FourCharCode('mp3 ');
	kRTPMPEG4AudioMediaPacketizerType = FourCharCode('mp4a');
	kRTPMPEG4VideoMediaPacketizerType = FourCharCode('mp4v');
	kRTPAMRMediaPacketizerType = FourCharCode('amr ');

type
	RTPMPSampleRef = UInt32;
	RTPMPDataReleaseProcPtr = procedure( var inData: UInt8; inRefCon: UnivPtr );
	RTPMPDataReleaseUPP = RTPMPDataReleaseProcPtr;
const
	kMediaPacketizerCanPackEditRate = 1 shl 0;
	kMediaPacketizerCanPackLayer = 1 shl 1;
	kMediaPacketizerCanPackVolume = 1 shl 2;
	kMediaPacketizerCanPackBalance = 1 shl 3;
	kMediaPacketizerCanPackGraphicsMode = 1 shl 4;
	kMediaPacketizerCanPackEmptyEdit = 1 shl 5;


type
	MediaPacketizerRequirements = record
		mediaType: OSType;              { media type supported (0 for all)}
		dataFormat: OSType;             { data format (e.g., compression) supported (0 for all)}
		capabilityFlags: UInt32;        { ability to handle non-standard track characteristics}
		canPackMatrixType: UInt8;      { can pack any matrix type up to this (identityMatrixType for identity only)}
		reserved1: UInt8;
		reserved2: UInt8;
		reserved3: UInt8;
	end;
	MediaPacketizerRequirementsPtr = ^MediaPacketizerRequirements;
type
	MediaPacketizerInfo = record
		mediaType: OSType;              { media type supported (0 for all)}
		dataFormat: OSType;             { data format (e.g., compression) supported (0 for all)}
		vendor: OSType;                 { manufacturer of this packetizer (e.g., 'appl' for Apple)}
		capabilityFlags: UInt32;        { ability to handle non-standard track characteristics}
		canPackMatrixType: UInt8;      { can pack any matrix type up to this (identityMatrixType for identity only)}
		reserved1: UInt8;
		reserved2: UInt8;
		reserved3: UInt8;
		characteristicCount: SIGNEDLONG;
		characteristic: array [0..0] of RTPPayloadCharacteristic;

                                              { after the last characteristic, the payload name (defined by the RTPPayloadInfo}
                                              { structure) is present. }
	end;
	MediaPacketizerInfoPtr = ^MediaPacketizerInfo;
type
	MediaPacketizerInfoHandle = ^MediaPacketizerInfoPtr;
{ MediaPacketizerInfo structs are padded to 32 bits }
const
	kMediaPacketizerInfoPadUpToBytes = 4;

<<<<<<< HEAD
	{
	 *  QTSFindMediaPacketizer()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.0 and later
	 	}
function QTSFindMediaPacketizer(inPacketizerinfo: MediaPacketizerRequirementsPtr; inSampleDescription: SampleDescriptionHandle; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer): OSErr; external name '_QTSFindMediaPacketizer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kRTPMediaPacketizerInfoRezType = FourCharCode('pcki');

>>>>>>> origin/fixes_2.4

{
 *  QTSFindMediaPacketizer()
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
function QTSFindMediaPacketizerForTrack( inTrack: Track; inSampleDescriptionIndex: SIGNEDLONG; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer ): OSErr; external name '_QTSFindMediaPacketizerForTrack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function QTSFindMediaPacketizer( inPacketizerinfo: MediaPacketizerRequirementsPtr; inSampleDescription: SampleDescriptionHandle; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer ): OSErr; external name '_QTSFindMediaPacketizer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  QTSFindMediaPacketizerForTrack()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function QTSFindMediaPacketizerForTrack(inTrack: Track; inSampleDescriptionIndex: SInt32; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer): OSErr; external name '_QTSFindMediaPacketizerForTrack';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSFindMediaPacketizerForTrack( inTrack: Track; inSampleDescriptionIndex: SIGNEDLONG; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer ): OSErr; external name '_QTSFindMediaPacketizerForTrack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSFindMediaPacketizerForPayloadID()
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
function QTSFindMediaPacketizerForPayloadID( payloadID: SIGNEDLONG; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer ): OSErr; external name '_QTSFindMediaPacketizerForPayloadID';
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
function QTSFindMediaPacketizerForPayloadID(payloadID: SInt32; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer): OSErr; external name '_QTSFindMediaPacketizerForPayloadID';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTSFindMediaPacketizerForPayloadID( payloadID: SIGNEDLONG; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer ): OSErr; external name '_QTSFindMediaPacketizerForPayloadID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTSFindMediaPacketizerForPayloadName()
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
function QTSFindMediaPacketizerForPayloadName( payloadName: ConstCStringPtr; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer ): OSErr; external name '_QTSFindMediaPacketizerForPayloadName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ flags for RTPMPInitialize}
const
	kRTPMPRealtimeModeFlag = $00000001;

{ flags for RTPMPSampleDataParams}
const
	kRTPMPSyncSampleFlag = $00000001;
	kRTPMPRespectDurationFlag = $00000002;
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
function QTSFindMediaPacketizerForPayloadName( payloadName: ConstCStringPtr; inSortInfo: RTPPayloadSortRequestPtr; var outPacketizerList: QTAtomContainer ): OSErr; external name '_QTSFindMediaPacketizerForPayloadName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ flags for RTPMPInitialize}
const
	kRTPMPRealtimeModeFlag = $00000001;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ flags for RTPMPSampleDataParams}
const
	kRTPMPSyncSampleFlag = $00000001;
	kRTPMPRespectDurationFlag = $00000002;
>>>>>>> origin/fixes_2.4

type
	RTPMPSampleDataParamsPtr = ^RTPMPSampleDataParams;
	RTPMPSampleDataParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		version: UInt32;
		timeStamp: UInt32;
		duration: UInt32;               { 0 = unknown duration}
		playOffset: UInt32;
		playRate: Fixed;
		flags: SInt32;
		sampleDescSeed: UInt32;
		sampleDescription: Handle;
		sampleRef: RTPMPSampleRef;
		dataLength: UInt32;
		data: {const} UInt8Ptr;
		releaseProc: RTPMPDataReleaseUPP;
		refCon: UnivPtr;
<<<<<<< HEAD
	end;
{ out flags for idle, RTPMPSetSampleData, and RTPMPFlush}
const
	kRTPMPStillProcessingData = $00000001; { not done with data you've got}
=======
=======
>>>>>>> origin/fixes_2_2
		version:				UInt32;
		timeStamp:				UInt32;
		duration:				UInt32;									{  0 = unknown duration }
		playOffset:				UInt32;
		playRate:				Fixed;
		flags:					SInt32;
		sampleDescSeed:			UInt32;
		sampleDescription:		Handle;
		sampleRef:				RTPMPSampleRef;
		dataLength:				UInt32;
		data:					Ptr;
		releaseProc:			RTPMPDataReleaseUPP;
		refCon:					Ptr;
=======
>>>>>>> origin/fixes_2.4
	end;
{ out flags for idle, RTPMPSetSampleData, and RTPMPFlush}
const
<<<<<<< HEAD
	kRTPMPStillProcessingData	= $00000001;					{  not done with data you've got }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kRTPMPStillProcessingData = $00000001; { not done with data you've got}
>>>>>>> origin/fixes_2.4

type
	RTPMPPayloadTypeParamsPtr = ^RTPMPPayloadTypeParams;
	RTPMPPayloadTypeParams = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		flags: UInt32;
		payloadNumber: UInt32;
		nameLength: SInt16;             { in: size of payloadName buffer (counting null terminator) -- this will be reset to needed length and paramErr returned if too small }
		payloadName: CStringPtr;            { caller must provide buffer }
<<<<<<< HEAD
	end;
{-----------------------------------------
    RTP Media Packetizer Info selectors
-----------------------------------------}
{ info selectors - get only }
const
	kRTPMPPayloadTypeInfo = FourCharCode('rtpp'); { RTPMPPayloadTypeParams* }
	kRTPMPRTPTimeScaleInfo = FourCharCode('rtpt'); { TimeScale* }
	kRTPMPRequiredSampleDescriptionInfo = FourCharCode('sdsc'); { SampleDescriptionHandle* }
	kRTPMPMinPayloadSize = FourCharCode('mins'); { UInt32* in bytes, does not include rtp header; default is 0 }
	kRTPMPMinPacketDuration = FourCharCode('mind'); { UInt3* in milliseconds; default is no min required }
	kRTPMPSuggestedRepeatPktCountInfo = FourCharCode('srpc'); { UInt32* }
	kRTPMPSuggestedRepeatPktSpacingInfo = FourCharCode('srps'); { UInt32* in milliseconds }
	kRTPMPMaxPartialSampleSizeInfo = FourCharCode('mpss'); { UInt32* in bytes }
	kRTPMPPreferredBufferDelayInfo = FourCharCode('prbd'); { UInt32* in milliseconds }
	kRTPMPPayloadNameInfo = FourCharCode('name'); { StringPtr }
	kRTPInfo_FormatString = FourCharCode('fmtp'); { char **, caller allocates ptr, callee disposes }

{-----------------------------------------
    RTP Media Packetizer Characteristics
-----------------------------------------}
{ also supports relevant ones in Movies.h and QTSToolbox.h }
const
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					UInt32;
		payloadNumber:			UInt32;
		nameLength:				SInt16;								{  in: size of payloadName buffer (counting null terminator) -- this will be reset to needed length and paramErr returned if too small  }
		payloadName:			CStringPtr;								{  caller must provide buffer  }
=======
>>>>>>> origin/fixes_2.4
	end;
{-----------------------------------------
    RTP Media Packetizer Info selectors
-----------------------------------------}
{ info selectors - get only }
const
	kRTPMPPayloadTypeInfo = FourCharCode('rtpp'); { RTPMPPayloadTypeParams* }
	kRTPMPRTPTimeScaleInfo = FourCharCode('rtpt'); { TimeScale* }
	kRTPMPRequiredSampleDescriptionInfo = FourCharCode('sdsc'); { SampleDescriptionHandle* }
	kRTPMPMinPayloadSize = FourCharCode('mins'); { UInt32* in bytes, does not include rtp header; default is 0 }
	kRTPMPMinPacketDuration = FourCharCode('mind'); { UInt3* in milliseconds; default is no min required }
	kRTPMPSuggestedRepeatPktCountInfo = FourCharCode('srpc'); { UInt32* }
	kRTPMPSuggestedRepeatPktSpacingInfo = FourCharCode('srps'); { UInt32* in milliseconds }
	kRTPMPMaxPartialSampleSizeInfo = FourCharCode('mpss'); { UInt32* in bytes }
	kRTPMPPreferredBufferDelayInfo = FourCharCode('prbd'); { UInt32* in milliseconds }
	kRTPMPPayloadNameInfo = FourCharCode('name'); { StringPtr }
	kRTPInfo_FormatString = FourCharCode('fmtp'); { char **, caller allocates ptr, callee disposes }

{-----------------------------------------
    RTP Media Packetizer Characteristics
-----------------------------------------}
{ also supports relevant ones in Movies.h and QTSToolbox.h }
const
<<<<<<< HEAD
	kRTPMPPayloadTypeInfo		= FourCharCode('rtpp');						{  RTPMPPayloadTypeParams*  }
	kRTPMPRTPTimeScaleInfo		= FourCharCode('rtpt');						{  TimeScale*  }
	kRTPMPRequiredSampleDescriptionInfo = FourCharCode('sdsc');				{  SampleDescriptionHandle*  }
	kRTPMPMinPayloadSize		= FourCharCode('mins');						{  UInt32* in bytes, does not include rtp header; default is 0  }
	kRTPMPMinPacketDuration		= FourCharCode('mind');						{  UInt3* in milliseconds; default is no min required  }
	kRTPMPSuggestedRepeatPktCountInfo = FourCharCode('srpc');					{  UInt32*  }
	kRTPMPSuggestedRepeatPktSpacingInfo = FourCharCode('srps');				{  UInt32* in milliseconds  }
	kRTPMPMaxPartialSampleSizeInfo = FourCharCode('mpss');					{  UInt32* in bytes  }
	kRTPMPPreferredBufferDelayInfo = FourCharCode('prbd');					{  UInt32* in milliseconds  }
	kRTPMPPayloadNameInfo		= FourCharCode('name');						{  StringPtr  }
	kRTPInfo_FormatString		= FourCharCode('fmtp');						{  char **, caller allocates ptr, callee disposes  }

	{	-----------------------------------------
	    RTP Media Packetizer Characteristics
	-----------------------------------------	}
	{	 also supports relevant ones in Movies.h and QTSToolbox.h 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kRTPMPNoSampleDataRequiredCharacteristic = FourCharCode('nsdr');
	kRTPMPHasUserSettingsDialogCharacteristic = FourCharCode('sdlg');
	kRTPMPPrefersReliableTransportCharacteristic = FourCharCode('rely');
	kRTPMPRequiresOutOfBandDimensionsCharacteristic = FourCharCode('robd');
	kRTPMPReadsPartialSamplesCharacteristic = FourCharCode('rpsp');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{-----------------------------------------
    RTP Media Packetizer selectors
-----------------------------------------}
const
	kRTPMPInitializeSelect = $0500;
	kRTPMPPreflightMediaSelect = $0501;
	kRTPMPIdleSelect = $0502;
	kRTPMPSetSampleDataSelect = $0503;
	kRTPMPFlushSelect = $0504;
	kRTPMPResetSelect = $0505;
	kRTPMPSetInfoSelect = $0506;
	kRTPMPGetInfoSelect = $0507;
	kRTPMPSetTimeScaleSelect = $0508;
	kRTPMPGetTimeScaleSelect = $0509;
	kRTPMPSetTimeBaseSelect = $050A;
	kRTPMPGetTimeBaseSelect = $050B;
<<<<<<< HEAD
	kRTPMPHasCharacteristicSelect = $050C;
	kRTPMPSetPacketBuilderSelect = $050E;
	kRTPMPGetPacketBuilderSelect = $050F;
	kRTPMPSetMediaTypeSelect = $0510;
	kRTPMPGetMediaTypeSelect = $0511;
	kRTPMPSetMaxPacketSizeSelect = $0512;
	kRTPMPGetMaxPacketSizeSelect = $0513;
	kRTPMPSetMaxPacketDurationSelect = $0514;
	kRTPMPGetMaxPacketDurationSelect = $0515; { for export component and apps who want to}
                                        { access dialogs for Media-specific settings}
                                        { (such as Pure Voice interleave factor)}
	kRTPMPDoUserDialogSelect = $0516;
	kRTPMPSetSettingsFromAtomContainerAtAtomSelect = $0517;
	kRTPMPGetSettingsIntoAtomContainerAtAtomSelect = $0518;
	kRTPMPGetSettingsAsTextSelect = $0519;
	kRTPMPGetSettingsSelect = $051C;
	kRTPMPSetSettingsSelect = $051D;

{-----------------------------------------
    RTP Media Packetizer functions
-----------------------------------------}

{
 *  RTPMPInitialize()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function RTPMPInitialize( rtpm: RTPMediaPacketizer; inFlags: SInt32 ): ComponentResult; external name '_RTPMPInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
	{	-----------------------------------------
	    RTP Media Packetizer selectors
	-----------------------------------------	}
	kRTPMPInitializeSelect		= $0500;
	kRTPMPPreflightMediaSelect	= $0501;
	kRTPMPIdleSelect			= $0502;
	kRTPMPSetSampleDataSelect	= $0503;
	kRTPMPFlushSelect			= $0504;
	kRTPMPResetSelect			= $0505;
	kRTPMPSetInfoSelect			= $0506;
	kRTPMPGetInfoSelect			= $0507;
	kRTPMPSetTimeScaleSelect	= $0508;
	kRTPMPGetTimeScaleSelect	= $0509;
	kRTPMPSetTimeBaseSelect		= $050A;
	kRTPMPGetTimeBaseSelect		= $050B;
=======
>>>>>>> origin/fixes_2.4
	kRTPMPHasCharacteristicSelect = $050C;
	kRTPMPSetPacketBuilderSelect = $050E;
	kRTPMPGetPacketBuilderSelect = $050F;
	kRTPMPSetMediaTypeSelect = $0510;
	kRTPMPGetMediaTypeSelect = $0511;
	kRTPMPSetMaxPacketSizeSelect = $0512;
	kRTPMPGetMaxPacketSizeSelect = $0513;
	kRTPMPSetMaxPacketDurationSelect = $0514;
	kRTPMPGetMaxPacketDurationSelect = $0515; { for export component and apps who want to}
                                        { access dialogs for Media-specific settings}
                                        { (such as Pure Voice interleave factor)}
	kRTPMPDoUserDialogSelect = $0516;
	kRTPMPSetSettingsFromAtomContainerAtAtomSelect = $0517;
	kRTPMPGetSettingsIntoAtomContainerAtAtomSelect = $0518;
	kRTPMPGetSettingsAsTextSelect = $0519;
<<<<<<< HEAD
	kRTPMPGetSettingsSelect		= $051A;
	kRTPMPSetSettingsSelect		= $051B;

	{	-----------------------------------------
	    RTP Media Packetizer functions
	-----------------------------------------	}

	{
	 *  RTPMPInitialize()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.0 and later
	 	}
function RTPMPInitialize(rtpm: RTPMediaPacketizer; inFlags: SInt32): ComponentResult; external name '_RTPMPInitialize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kRTPMPGetSettingsSelect = $051C;
	kRTPMPSetSettingsSelect = $051D;

{-----------------------------------------
    RTP Media Packetizer functions
-----------------------------------------}

{
 *  RTPMPInitialize()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function RTPMPInitialize( rtpm: RTPMediaPacketizer; inFlags: SInt32 ): ComponentResult; external name '_RTPMPInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{ return noErr if you can handle this media }
{
 *  RTPMPPreflightMedia()
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
function RTPMPPreflightMedia( rtpm: RTPMediaPacketizer; inMediaType: OSType; inSampleDescription: SampleDescriptionHandle ): ComponentResult; external name '_RTPMPPreflightMedia';
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
function RTPMPPreflightMedia(rtpm: RTPMediaPacketizer; inMediaType: OSType; inSampleDescription: SampleDescriptionHandle): ComponentResult; external name '_RTPMPPreflightMedia';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPPreflightMedia( rtpm: RTPMediaPacketizer; inMediaType: OSType; inSampleDescription: SampleDescriptionHandle ): ComponentResult; external name '_RTPMPPreflightMedia';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   do work here if you need to - give up time periodically
   if you're doing time consuming operations
}
{
 *  RTPMPIdle()
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
function RTPMPIdle( rtpm: RTPMediaPacketizer; inFlags: SInt32; var outFlags: SInt32 ): ComponentResult; external name '_RTPMPIdle';
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
function RTPMPIdle(rtpm: RTPMediaPacketizer; inFlags: SInt32; var outFlags: SInt32): ComponentResult; external name '_RTPMPIdle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPIdle( rtpm: RTPMediaPacketizer; inFlags: SInt32; var outFlags: SInt32 ): ComponentResult; external name '_RTPMPIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   caller owns the RTPMPSampleDataParams struct
   media Packetizer must copy any fields of the struct it wants to keep
   media Packetizer must call release proc when done with the data
   you can do the processing work here if it does not take up too
   much cpu time - otherwise do it in idle
}
{
 *  RTPMPSetSampleData()
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
function RTPMPSetSampleData( rtpm: RTPMediaPacketizer; const (*var*) inSampleData: RTPMPSampleDataParams; var outFlags: SInt32 ): ComponentResult; external name '_RTPMPSetSampleData';
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
function RTPMPSetSampleData(rtpm: RTPMediaPacketizer; const (*var*) inSampleData: RTPMPSampleDataParams; var outFlags: SInt32): ComponentResult; external name '_RTPMPSetSampleData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetSampleData( rtpm: RTPMediaPacketizer; const (*var*) inSampleData: RTPMPSampleDataParams; var outFlags: SInt32 ): ComponentResult; external name '_RTPMPSetSampleData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   send everything you have buffered - you will get idles while
   you set the kRTPMPStillProcessingData flag here and in idle
}
{
 *  RTPMPFlush()
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
function RTPMPFlush( rtpm: RTPMediaPacketizer; inFlags: SInt32; var outFlags: SInt32 ): ComponentResult; external name '_RTPMPFlush';
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
function RTPMPFlush(rtpm: RTPMediaPacketizer; inFlags: SInt32; var outFlags: SInt32): ComponentResult; external name '_RTPMPFlush';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPFlush( rtpm: RTPMediaPacketizer; inFlags: SInt32; var outFlags: SInt32 ): ComponentResult; external name '_RTPMPFlush';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   dispose of anything buffered and get rid of state
   do not send the buffered data (because presumably
   there is no connection for you to send on)
   state should be the same as if you were just initialized
}
{
 *  RTPMPReset()
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
function RTPMPReset( rtpm: RTPMediaPacketizer; inFlags: SInt32 ): ComponentResult; external name '_RTPMPReset';
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
function RTPMPReset(rtpm: RTPMediaPacketizer; inFlags: SInt32): ComponentResult; external name '_RTPMPReset';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPReset( rtpm: RTPMediaPacketizer; inFlags: SInt32 ): ComponentResult; external name '_RTPMPReset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{-----------------------------------------
    RTP Media Packetizer get / set functions
-----------------------------------------}
{
 *  RTPMPSetInfo()
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
function RTPMPSetInfo( rtpm: RTPMediaPacketizer; inSelector: OSType; ioParams: {const} UnivPtr ): ComponentResult; external name '_RTPMPSetInfo';
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
function RTPMPSetInfo(rtpm: RTPMediaPacketizer; inSelector: OSType; ioParams: UnivPtr): ComponentResult; external name '_RTPMPSetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetInfo( rtpm: RTPMediaPacketizer; inSelector: OSType; ioParams: {const} UnivPtr ): ComponentResult; external name '_RTPMPSetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetInfo()
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
function RTPMPGetInfo( rtpm: RTPMediaPacketizer; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPMPGetInfo';
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
function RTPMPGetInfo(rtpm: RTPMediaPacketizer; inSelector: OSType; ioParams: UnivPtr): ComponentResult; external name '_RTPMPGetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetInfo( rtpm: RTPMediaPacketizer; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPMPGetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPSetTimeScale()
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
function RTPMPSetTimeScale( rtpm: RTPMediaPacketizer; inTimeScale: TimeScale ): ComponentResult; external name '_RTPMPSetTimeScale';
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
function RTPMPSetTimeScale(rtpm: RTPMediaPacketizer; inTimeScale: TimeScale): ComponentResult; external name '_RTPMPSetTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetTimeScale( rtpm: RTPMediaPacketizer; inTimeScale: TimeScale ): ComponentResult; external name '_RTPMPSetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetTimeScale()
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
function RTPMPGetTimeScale( rtpm: RTPMediaPacketizer; var outTimeScale: TimeScale ): ComponentResult; external name '_RTPMPGetTimeScale';
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
function RTPMPGetTimeScale(rtpm: RTPMediaPacketizer; var outTimeScale: TimeScale): ComponentResult; external name '_RTPMPGetTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetTimeScale( rtpm: RTPMediaPacketizer; var outTimeScale: TimeScale ): ComponentResult; external name '_RTPMPGetTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPSetTimeBase()
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
function RTPMPSetTimeBase( rtpm: RTPMediaPacketizer; inTimeBase: TimeBase ): ComponentResult; external name '_RTPMPSetTimeBase';
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
function RTPMPSetTimeBase(rtpm: RTPMediaPacketizer; inTimeBase: TimeBase): ComponentResult; external name '_RTPMPSetTimeBase';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetTimeBase( rtpm: RTPMediaPacketizer; inTimeBase: TimeBase ): ComponentResult; external name '_RTPMPSetTimeBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetTimeBase()
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
function RTPMPGetTimeBase( rtpm: RTPMediaPacketizer; var outTimeBase: TimeBase ): ComponentResult; external name '_RTPMPGetTimeBase';
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
function RTPMPGetTimeBase(rtpm: RTPMediaPacketizer; var outTimeBase: TimeBase): ComponentResult; external name '_RTPMPGetTimeBase';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetTimeBase( rtpm: RTPMediaPacketizer; var outTimeBase: TimeBase ): ComponentResult; external name '_RTPMPGetTimeBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPHasCharacteristic()
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
function RTPMPHasCharacteristic( rtpm: RTPMediaPacketizer; inSelector: OSType; var outHasIt: Boolean ): ComponentResult; external name '_RTPMPHasCharacteristic';
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
function RTPMPHasCharacteristic(rtpm: RTPMediaPacketizer; inSelector: OSType; var outHasIt: boolean): ComponentResult; external name '_RTPMPHasCharacteristic';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPHasCharacteristic( rtpm: RTPMediaPacketizer; inSelector: OSType; var outHasIt: Boolean ): ComponentResult; external name '_RTPMPHasCharacteristic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPSetPacketBuilder()
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
function RTPMPSetPacketBuilder( rtpm: RTPMediaPacketizer; inPacketBuilder: ComponentInstance ): ComponentResult; external name '_RTPMPSetPacketBuilder';
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
function RTPMPSetPacketBuilder(rtpm: RTPMediaPacketizer; inPacketBuilder: ComponentInstance): ComponentResult; external name '_RTPMPSetPacketBuilder';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetPacketBuilder( rtpm: RTPMediaPacketizer; inPacketBuilder: ComponentInstance ): ComponentResult; external name '_RTPMPSetPacketBuilder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetPacketBuilder()
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
function RTPMPGetPacketBuilder( rtpm: RTPMediaPacketizer; var outPacketBuilder: ComponentInstance ): ComponentResult; external name '_RTPMPGetPacketBuilder';
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
function RTPMPGetPacketBuilder(rtpm: RTPMediaPacketizer; var outPacketBuilder: ComponentInstance): ComponentResult; external name '_RTPMPGetPacketBuilder';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetPacketBuilder( rtpm: RTPMediaPacketizer; var outPacketBuilder: ComponentInstance ): ComponentResult; external name '_RTPMPGetPacketBuilder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPSetMediaType()
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
function RTPMPSetMediaType( rtpm: RTPMediaPacketizer; inMediaType: OSType ): ComponentResult; external name '_RTPMPSetMediaType';
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
function RTPMPSetMediaType(rtpm: RTPMediaPacketizer; inMediaType: OSType): ComponentResult; external name '_RTPMPSetMediaType';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetMediaType( rtpm: RTPMediaPacketizer; inMediaType: OSType ): ComponentResult; external name '_RTPMPSetMediaType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetMediaType()
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
function RTPMPGetMediaType( rtpm: RTPMediaPacketizer; var outMediaType: OSType ): ComponentResult; external name '_RTPMPGetMediaType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ size is in bytes}
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
function RTPMPGetMediaType(rtpm: RTPMediaPacketizer; var outMediaType: OSType): ComponentResult; external name '_RTPMPGetMediaType';
{  size is in bytes }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetMediaType( rtpm: RTPMediaPacketizer; var outMediaType: OSType ): ComponentResult; external name '_RTPMPGetMediaType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ size is in bytes}
>>>>>>> origin/fixes_2.4
{
 *  RTPMPSetMaxPacketSize()
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
function RTPMPSetMaxPacketSize( rtpm: RTPMediaPacketizer; inMaxPacketSize: UInt32 ): ComponentResult; external name '_RTPMPSetMaxPacketSize';
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
function RTPMPSetMaxPacketSize(rtpm: RTPMediaPacketizer; inMaxPacketSize: UInt32): ComponentResult; external name '_RTPMPSetMaxPacketSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetMaxPacketSize( rtpm: RTPMediaPacketizer; inMaxPacketSize: UInt32 ): ComponentResult; external name '_RTPMPSetMaxPacketSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetMaxPacketSize()
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
function RTPMPGetMaxPacketSize( rtpm: RTPMediaPacketizer; var outMaxPacketSize: UInt32 ): ComponentResult; external name '_RTPMPGetMaxPacketSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ duration is in milliseconds}
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
function RTPMPGetMaxPacketSize(rtpm: RTPMediaPacketizer; var outMaxPacketSize: UInt32): ComponentResult; external name '_RTPMPGetMaxPacketSize';
{  duration is in milliseconds }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetMaxPacketSize( rtpm: RTPMediaPacketizer; var outMaxPacketSize: UInt32 ): ComponentResult; external name '_RTPMPGetMaxPacketSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ duration is in milliseconds}
>>>>>>> origin/fixes_2.4
{
 *  RTPMPSetMaxPacketDuration()
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
function RTPMPSetMaxPacketDuration( rtpm: RTPMediaPacketizer; inMaxPacketDuration: UInt32 ): ComponentResult; external name '_RTPMPSetMaxPacketDuration';
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
function RTPMPSetMaxPacketDuration(rtpm: RTPMediaPacketizer; inMaxPacketDuration: UInt32): ComponentResult; external name '_RTPMPSetMaxPacketDuration';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetMaxPacketDuration( rtpm: RTPMediaPacketizer; inMaxPacketDuration: UInt32 ): ComponentResult; external name '_RTPMPSetMaxPacketDuration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetMaxPacketDuration()
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
function RTPMPGetMaxPacketDuration( rtpm: RTPMediaPacketizer; var outMaxPacketDuration: UInt32 ): ComponentResult; external name '_RTPMPGetMaxPacketDuration';
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
function RTPMPGetMaxPacketDuration(rtpm: RTPMediaPacketizer; var outMaxPacketDuration: UInt32): ComponentResult; external name '_RTPMPGetMaxPacketDuration';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetMaxPacketDuration( rtpm: RTPMediaPacketizer; var outMaxPacketDuration: UInt32 ): ComponentResult; external name '_RTPMPGetMaxPacketDuration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPDoUserDialog()
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
function RTPMPDoUserDialog( rtpm: RTPMediaPacketizer; inFilterUPP: ModalFilterUPP; var canceled: Boolean ): ComponentResult; external name '_RTPMPDoUserDialog';
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
function RTPMPDoUserDialog(rtpm: RTPMediaPacketizer; inFilterUPP: ModalFilterUPP; var canceled: boolean): ComponentResult; external name '_RTPMPDoUserDialog';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPDoUserDialog( rtpm: RTPMediaPacketizer; inFilterUPP: ModalFilterUPP; var canceled: Boolean ): ComponentResult; external name '_RTPMPDoUserDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPSetSettingsFromAtomContainerAtAtom()
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
function RTPMPSetSettingsFromAtomContainerAtAtom( rtpm: RTPMediaPacketizer; inContainer: QTAtomContainer; inParentAtom: QTAtom ): ComponentResult; external name '_RTPMPSetSettingsFromAtomContainerAtAtom';
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
function RTPMPSetSettingsFromAtomContainerAtAtom(rtpm: RTPMediaPacketizer; inContainer: QTAtomContainer; inParentAtom: QTAtom): ComponentResult; external name '_RTPMPSetSettingsFromAtomContainerAtAtom';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPSetSettingsFromAtomContainerAtAtom( rtpm: RTPMediaPacketizer; inContainer: QTAtomContainer; inParentAtom: QTAtom ): ComponentResult; external name '_RTPMPSetSettingsFromAtomContainerAtAtom';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetSettingsIntoAtomContainerAtAtom()
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
function RTPMPGetSettingsIntoAtomContainerAtAtom( rtpm: RTPMediaPacketizer; inOutContainer: QTAtomContainer; inParentAtom: QTAtom ): ComponentResult; external name '_RTPMPGetSettingsIntoAtomContainerAtAtom';
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
function RTPMPGetSettingsIntoAtomContainerAtAtom(rtpm: RTPMediaPacketizer; inOutContainer: QTAtomContainer; inParentAtom: QTAtom): ComponentResult; external name '_RTPMPGetSettingsIntoAtomContainerAtAtom';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetSettingsIntoAtomContainerAtAtom( rtpm: RTPMediaPacketizer; inOutContainer: QTAtomContainer; inParentAtom: QTAtom ): ComponentResult; external name '_RTPMPGetSettingsIntoAtomContainerAtAtom';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetSettingsAsText()
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
function RTPMPGetSettingsAsText( rtpm: RTPMediaPacketizer; var text: Handle ): ComponentResult; external name '_RTPMPGetSettingsAsText';
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
function RTPMPGetSettingsAsText(rtpm: RTPMediaPacketizer; var text: Handle): ComponentResult; external name '_RTPMPGetSettingsAsText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetSettingsAsText( rtpm: RTPMediaPacketizer; var text: Handle ): ComponentResult; external name '_RTPMPGetSettingsAsText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPGetSettings()
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
function RTPMPGetSettings( rtpm: RTPMediaPacketizer; var outSettings: QTAtomContainer; inFlags: SInt32 ): ComponentResult; external name '_RTPMPGetSettings';
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
function RTPMPGetSettings(rtpm: RTPMediaPacketizer; var outSettings: QTAtomContainer; inFlags: SInt32): ComponentResult; external name '_RTPMPGetSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPMPGetSettings( rtpm: RTPMediaPacketizer; var outSettings: QTAtomContainer; inFlags: SInt32 ): ComponentResult; external name '_RTPMPGetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPMPSetSettings()
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
function RTPMPSetSettings( rtpm: RTPMediaPacketizer; inSettings: QTAtomSpecPtr; inFlags: SInt32 ): ComponentResult; external name '_RTPMPSetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{============================================================================
        RTP Packet Builder
============================================================================}
const
	kRTPPacketBuilderType = FourCharCode('rtpb');


type
	RTPPacketBuilder = ComponentInstance;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	RTPPacketGroupRef = ^OpaqueRTPPacketGroupRef; { an opaque type }
	OpaqueRTPPacketGroupRef = record end;
	RTPPacketRef = ^OpaqueRTPPacketRef; { an opaque type }
	OpaqueRTPPacketRef = record end;
	RTPPacketRepeatedDataRef = ^OpaqueRTPPacketRepeatedDataRef; { an opaque type }
	OpaqueRTPPacketRepeatedDataRef = record end;
=======
	RTPPacketGroupRef = ^SInt32; { an opaque type }
	RTPPacketRef = ^SInt32; { an opaque type }
	RTPPacketRepeatedDataRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	RTPPacketGroupRef = ^SInt32; { an opaque type }
	RTPPacketRef = ^SInt32; { an opaque type }
	RTPPacketRepeatedDataRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	RTPPacketGroupRef = ^SInt32; { an opaque type }
	RTPPacketRef = ^SInt32; { an opaque type }
	RTPPacketRepeatedDataRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	RTPPacketGroupRef = ^SInt32; { an opaque type }
	RTPPacketRef = ^SInt32; { an opaque type }
	RTPPacketRepeatedDataRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
{ flags for RTPPBBegin/EndPacket, RTPPBBegin/EndPacketGroup}
const
	kRTPPBSetMarkerFlag = $00000001;
	kRTPPBRepeatPacketFlag = $00000002;
	kRTPPBSyncSampleFlag = $00010000;
	kRTPPBBFrameFlag = $00020000;
	kRTPPBDontSendFlag = $10000000; { when set in EndPacketGroup, will not add group}

const
	kRTPPBUnknownPacketMediaDataLength = 0;

{ flags for RTPPBGetSampleData}
const
	kRTPPBEndOfDataFlag = $00000001;


type
	RTPPBCallbackProcPtr = procedure( inSelector: OSType; ioParams: UnivPtr; inRefCon: UnivPtr );
	RTPPBCallbackUPP = RTPPBCallbackProcPtr;
{-----------------------------------------
    RTP Packet Builder selectors
-----------------------------------------}
const
	kRTPPBBeginPacketGroupSelect = $0500;
	kRTPPBEndPacketGroupSelect = $0501;
	kRTPPBBeginPacketSelect = $0502;
	kRTPPBEndPacketSelect = $0503;
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
function RTPMPSetSettings( rtpm: RTPMediaPacketizer; inSettings: QTAtomSpecPtr; inFlags: SInt32 ): ComponentResult; external name '_RTPMPSetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{============================================================================
        RTP Packet Builder
============================================================================}
const
	kRTPPacketBuilderType = FourCharCode('rtpb');


type
	RTPPacketBuilder = ComponentInstance;
	RTPPacketGroupRef = ^SInt32; { an opaque type }
	RTPPacketRef = ^SInt32; { an opaque type }
	RTPPacketRepeatedDataRef = ^SInt32; { an opaque type }
{ flags for RTPPBBegin/EndPacket, RTPPBBegin/EndPacketGroup}
const
	kRTPPBSetMarkerFlag = $00000001;
	kRTPPBRepeatPacketFlag = $00000002;
	kRTPPBSyncSampleFlag = $00010000;
	kRTPPBBFrameFlag = $00020000;
	kRTPPBDontSendFlag = $10000000; { when set in EndPacketGroup, will not add group}

const
	kRTPPBUnknownPacketMediaDataLength = 0;

{ flags for RTPPBGetSampleData}
const
	kRTPPBEndOfDataFlag = $00000001;


type
	RTPPBCallbackProcPtr = procedure( inSelector: OSType; ioParams: UnivPtr; inRefCon: UnivPtr );
	RTPPBCallbackUPP = RTPPBCallbackProcPtr;
{-----------------------------------------
    RTP Packet Builder selectors
-----------------------------------------}
const
	kRTPPBBeginPacketGroupSelect = $0500;
<<<<<<< HEAD
	kRTPPBEndPacketGroupSelect	= $0501;
	kRTPPBBeginPacketSelect		= $0502;
	kRTPPBEndPacketSelect		= $0503;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kRTPPBEndPacketGroupSelect = $0501;
	kRTPPBBeginPacketSelect = $0502;
	kRTPPBEndPacketSelect = $0503;
>>>>>>> origin/fixes_2.4
	kRTPPBAddPacketLiteralDataSelect = $0504;
	kRTPPBAddPacketSampleDataSelect = $0505;
	kRTPPBAddPacketRepeatedDataSelect = $0506;
	kRTPPBReleaseRepeatedDataSelect = $0507;
	kRTPPBSetPacketSequenceNumberSelect = $0508;
	kRTPPBGetPacketSequenceNumberSelect = $0509;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kRTPPBSetCallbackSelect = $050A;
	kRTPPBGetCallbackSelect = $050B;
	kRTPPBSetInfoSelect = $050C;
	kRTPPBGetInfoSelect = $050D;
<<<<<<< HEAD
	kRTPPBSetPacketTimeStampOffsetSelect = $050E;
	kRTPPBGetPacketTimeStampOffsetSelect = $050F;
	kRTPPBAddPacketSampleData64Select = $0510;
	kRTPPBGetSampleDataSelect = $0511;
	kRTPPBAddRepeatPacketSelect = $0512;

{-----------------------------------------
    RTP Packet Builder functions
-----------------------------------------}
{
 *  RTPPBBeginPacketGroup()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function RTPPBBeginPacketGroup( rtpb: RTPPacketBuilder; inFlags: SInt32; inTimeStamp: UInt32; var outPacketGroup: RTPPacketGroupRef ): ComponentResult; external name '_RTPPBBeginPacketGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
	kRTPPBSetCallbackSelect		= $050A;
	kRTPPBGetCallbackSelect		= $050B;
	kRTPPBSetInfoSelect			= $050C;
	kRTPPBGetInfoSelect			= $050D;
	kRTPPBSetPacketTimeStampOffsetSelect = $050E;
	kRTPPBGetPacketTimeStampOffsetSelect = $050F;
	kRTPPBAddPacketSampleData64Select = $0510;
	kRTPPBGetSampleDataSelect	= $0511;
	kRTPPBAddRepeatPacketSelect	= $0512;

	{	-----------------------------------------
	    RTP Packet Builder functions
	-----------------------------------------	}
	{
	 *  RTPPBBeginPacketGroup()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in QTSClient.lib 4.0 and later
	 	}
function RTPPBBeginPacketGroup(rtpb: RTPPacketBuilder; inFlags: SInt32; inTimeStamp: UInt32; var outPacketGroup: RTPPacketGroupRef): ComponentResult; external name '_RTPPBBeginPacketGroup';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kRTPPBSetPacketTimeStampOffsetSelect = $050E;
	kRTPPBGetPacketTimeStampOffsetSelect = $050F;
	kRTPPBAddPacketSampleData64Select = $0510;
	kRTPPBGetSampleDataSelect = $0511;
	kRTPPBAddRepeatPacketSelect = $0512;

{-----------------------------------------
    RTP Packet Builder functions
-----------------------------------------}
>>>>>>> origin/fixes_2.4
{
 *  RTPPBBeginPacketGroup()
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
function RTPPBEndPacketGroup( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef ): ComponentResult; external name '_RTPPBEndPacketGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
function RTPPBBeginPacketGroup( rtpb: RTPPacketBuilder; inFlags: SInt32; inTimeStamp: UInt32; var outPacketGroup: RTPPacketGroupRef ): ComponentResult; external name '_RTPPBBeginPacketGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RTPPBEndPacketGroup()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QTStreamLib 4.0 and later
 *    Windows:          in QTSClient.lib 4.0 and later
 }
<<<<<<< HEAD
function RTPPBEndPacketGroup(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef): ComponentResult; external name '_RTPPBEndPacketGroup';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBEndPacketGroup( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef ): ComponentResult; external name '_RTPPBEndPacketGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBBeginPacket()
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
function RTPPBBeginPacket( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacketMediaDataLength: UInt32; var outPacket: RTPPacketRef ): ComponentResult; external name '_RTPPBBeginPacket';
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
function RTPPBBeginPacket(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacketMediaDataLength: UInt32; var outPacket: RTPPacketRef): ComponentResult; external name '_RTPPBBeginPacket';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBBeginPacket( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacketMediaDataLength: UInt32; var outPacket: RTPPacketRef ): ComponentResult; external name '_RTPPBBeginPacket';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBEndPacket()
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
function RTPPBEndPacket( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTransmissionTimeOffset: UInt32; inDuration: UInt32 ): ComponentResult; external name '_RTPPBEndPacket';
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
function RTPPBEndPacket(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTransmissionTimeOffset: UInt32; inDuration: UInt32): ComponentResult; external name '_RTPPBEndPacket';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBEndPacket( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTransmissionTimeOffset: UInt32; inDuration: UInt32 ): ComponentResult; external name '_RTPPBEndPacket';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   non-NULL RTPPacketRepeatedDataRef means this data will be repeated later
   pb must return a repeated data ref
}
{
 *  RTPPBAddPacketLiteralData()
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
function RTPPBAddPacketLiteralData( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inData: UInt8; inDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBAddPacketLiteralData';
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
function RTPPBAddPacketLiteralData(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inData: UInt8; inDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef): ComponentResult; external name '_RTPPBAddPacketLiteralData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBAddPacketLiteralData( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inData: UInt8; inDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBAddPacketLiteralData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   non-NULL RTPPacketRepeatedDataRef means this data will be repeated later
   pb must return a repeated data ref
}
{
 *  RTPPBAddPacketSampleData()
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
function RTPPBAddPacketSampleData( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inSampleDataParams: RTPMPSampleDataParams; inSampleOffset: UInt32; inSampleDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBAddPacketSampleData';
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
function RTPPBAddPacketSampleData(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inSampleDataParams: RTPMPSampleDataParams; inSampleOffset: UInt32; inSampleDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef): ComponentResult; external name '_RTPPBAddPacketSampleData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBAddPacketSampleData( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inSampleDataParams: RTPMPSampleDataParams; inSampleOffset: UInt32; inSampleDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBAddPacketSampleData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   non-NULL RTPPacketRepeatedDataRef means this data will be repeated later
   pb must return a repeated data ref
}
{
 *  RTPPBAddPacketSampleData64()
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
function RTPPBAddPacketSampleData64( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inSampleDataParams: RTPMPSampleDataParams; (*const*) var inSampleOffset: UInt64; inSampleDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBAddPacketSampleData64';
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
function RTPPBAddPacketSampleData64(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inSampleDataParams: RTPMPSampleDataParams; (*const*) var inSampleOffset: UInt64; inSampleDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef): ComponentResult; external name '_RTPPBAddPacketSampleData64';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBAddPacketSampleData64( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var inSampleDataParams: RTPMPSampleDataParams; (*const*) var inSampleOffset: UInt64; inSampleDataLength: UInt32; var outDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBAddPacketSampleData64';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   call to add the repeated data using the ref you got from
   RTPPBAddPacketLiteralData or RTPPBAddPacketSampleData
}
{
 *  RTPPBAddPacketRepeatedData()
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
function RTPPBAddPacketRepeatedData( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBAddPacketRepeatedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ call when done with repeated data}
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
function RTPPBAddPacketRepeatedData(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inDataRef: RTPPacketRepeatedDataRef): ComponentResult; external name '_RTPPBAddPacketRepeatedData';
{  call when done with repeated data }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBAddPacketRepeatedData( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBAddPacketRepeatedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ call when done with repeated data}
>>>>>>> origin/fixes_2.4
{
 *  RTPPBReleaseRepeatedData()
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
function RTPPBReleaseRepeatedData( rtpb: RTPPacketBuilder; inDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBReleaseRepeatedData';
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
function RTPPBReleaseRepeatedData(rtpb: RTPPacketBuilder; inDataRef: RTPPacketRepeatedDataRef): ComponentResult; external name '_RTPPBReleaseRepeatedData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBReleaseRepeatedData( rtpb: RTPPacketBuilder; inDataRef: RTPPacketRepeatedDataRef ): ComponentResult; external name '_RTPPBReleaseRepeatedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   seq number is just relative seq number
   don't call if you don't care when seq # is used
}
{
 *  RTPPBSetPacketSequenceNumber()
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
function RTPPBSetPacketSequenceNumber( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inSequenceNumber: UInt32 ): ComponentResult; external name '_RTPPBSetPacketSequenceNumber';
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
function RTPPBSetPacketSequenceNumber(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inSequenceNumber: UInt32): ComponentResult; external name '_RTPPBSetPacketSequenceNumber';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBSetPacketSequenceNumber( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inSequenceNumber: UInt32 ): ComponentResult; external name '_RTPPBSetPacketSequenceNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBGetPacketSequenceNumber()
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
function RTPPBGetPacketSequenceNumber( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var outSequenceNumber: UInt32 ): ComponentResult; external name '_RTPPBGetPacketSequenceNumber';
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
function RTPPBGetPacketSequenceNumber(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var outSequenceNumber: UInt32): ComponentResult; external name '_RTPPBGetPacketSequenceNumber';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBGetPacketSequenceNumber( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var outSequenceNumber: UInt32 ): ComponentResult; external name '_RTPPBGetPacketSequenceNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBSetPacketTimeStampOffset()
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
function RTPPBSetPacketTimeStampOffset( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTimeStampOffset: SInt32 ): ComponentResult; external name '_RTPPBSetPacketTimeStampOffset';
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
function RTPPBSetPacketTimeStampOffset(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTimeStampOffset: SInt32): ComponentResult; external name '_RTPPBSetPacketTimeStampOffset';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBSetPacketTimeStampOffset( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTimeStampOffset: SInt32 ): ComponentResult; external name '_RTPPBSetPacketTimeStampOffset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBGetPacketTimeStampOffset()
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
function RTPPBGetPacketTimeStampOffset( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var outTimeStampOffset: SInt32 ): ComponentResult; external name '_RTPPBGetPacketTimeStampOffset';
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
function RTPPBGetPacketTimeStampOffset(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var outTimeStampOffset: SInt32): ComponentResult; external name '_RTPPBGetPacketTimeStampOffset';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBGetPacketTimeStampOffset( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; var outTimeStampOffset: SInt32 ): ComponentResult; external name '_RTPPBGetPacketTimeStampOffset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBAddRepeatPacket()
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
function RTPPBAddRepeatPacket( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTransmissionOffset: TimeValue; inSequenceNumber: UInt32 ): ComponentResult; external name '_RTPPBAddRepeatPacket';
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
function RTPPBAddRepeatPacket(rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTransmissionOffset: TimeValue; inSequenceNumber: UInt32): ComponentResult; external name '_RTPPBAddRepeatPacket';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBAddRepeatPacket( rtpb: RTPPacketBuilder; inFlags: SInt32; inPacketGroup: RTPPacketGroupRef; inPacket: RTPPacketRef; inTransmissionOffset: TimeValue; inSequenceNumber: UInt32 ): ComponentResult; external name '_RTPPBAddRepeatPacket';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
   used for communicating with the caller of the media packetizers if needed
   NOT used for communicating with the media packetizers themselves
}
{
 *  RTPPBSetCallback()
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
function RTPPBSetCallback( rtpb: RTPPacketBuilder; inCallback: RTPPBCallbackUPP; inRefCon: UnivPtr ): ComponentResult; external name '_RTPPBSetCallback';
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
function RTPPBSetCallback(rtpb: RTPPacketBuilder; inCallback: RTPPBCallbackUPP; inRefCon: UnivPtr): ComponentResult; external name '_RTPPBSetCallback';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBSetCallback( rtpb: RTPPacketBuilder; inCallback: RTPPBCallbackUPP; inRefCon: UnivPtr ): ComponentResult; external name '_RTPPBSetCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBGetCallback()
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
function RTPPBGetCallback( rtpb: RTPPacketBuilder; var outCallback: RTPPBCallbackUPP; var outRefCon: UnivPtr ): ComponentResult; external name '_RTPPBGetCallback';
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
function RTPPBGetCallback(rtpb: RTPPacketBuilder; var outCallback: RTPPBCallbackUPP; var outRefCon: UnivPtr): ComponentResult; external name '_RTPPBGetCallback';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBGetCallback( rtpb: RTPPacketBuilder; var outCallback: RTPPBCallbackUPP; var outRefCon: UnivPtr ): ComponentResult; external name '_RTPPBGetCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBSetInfo()
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
function RTPPBSetInfo( rtpb: RTPPacketBuilder; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPPBSetInfo';
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
function RTPPBSetInfo(rtpb: RTPPacketBuilder; inSelector: OSType; ioParams: UnivPtr): ComponentResult; external name '_RTPPBSetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBSetInfo( rtpb: RTPPacketBuilder; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPPBSetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBGetInfo()
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
function RTPPBGetInfo( rtpb: RTPPacketBuilder; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPPBGetInfo';
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
function RTPPBGetInfo(rtpb: RTPPacketBuilder; inSelector: OSType; ioParams: UnivPtr): ComponentResult; external name '_RTPPBGetInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function RTPPBGetInfo( rtpb: RTPPacketBuilder; inSelector: OSType; ioParams: UnivPtr ): ComponentResult; external name '_RTPPBGetInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  RTPPBGetSampleData()
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
function RTPPBGetSampleData( rtpb: RTPPacketBuilder; var inParams: RTPMPSampleDataParams; (*const*) var inStartOffset: UInt64; var outDataBuffer: UInt8; inBytesToRead: UInt32; var outBytesRead: UInt32; var outFlags: SInt32 ): ComponentResult; external name '_RTPPBGetSampleData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ UPP call backs }
{
 *  NewRTPMPDataReleaseUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewRTPMPDataReleaseUPP( userRoutine: RTPMPDataReleaseProcPtr ): RTPMPDataReleaseUPP; external name '_NewRTPMPDataReleaseUPP';
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
function RTPPBGetSampleData( rtpb: RTPPacketBuilder; var inParams: RTPMPSampleDataParams; (*const*) var inStartOffset: UInt64; var outDataBuffer: UInt8; inBytesToRead: UInt32; var outBytesRead: UInt32; var outFlags: SInt32 ): ComponentResult; external name '_RTPPBGetSampleData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ UPP call backs }
{
 *  NewRTPMPDataReleaseUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewRTPMPDataReleaseUPP( userRoutine: RTPMPDataReleaseProcPtr ): RTPMPDataReleaseUPP; external name '_NewRTPMPDataReleaseUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	uppRTPMPDataReleaseProcInfo = $000003C0;
	uppRTPPBCallbackProcInfo = $00000FC0;
	{
	 *  NewRTPMPDataReleaseUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.3 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewRTPMPDataReleaseUPP(userRoutine: RTPMPDataReleaseProcPtr): RTPMPDataReleaseUPP; external name '_NewRTPMPDataReleaseUPP'; { old name was NewRTPMPDataReleaseProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewRTPPBCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewRTPPBCallbackUPP( userRoutine: RTPPBCallbackProcPtr ): RTPPBCallbackUPP; external name '_NewRTPPBCallbackUPP';
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
function NewRTPPBCallbackUPP(userRoutine: RTPPBCallbackProcPtr): RTPPBCallbackUPP; external name '_NewRTPPBCallbackUPP'; { old name was NewRTPPBCallbackProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewRTPPBCallbackUPP( userRoutine: RTPPBCallbackProcPtr ): RTPPBCallbackUPP; external name '_NewRTPPBCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeRTPMPDataReleaseUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeRTPMPDataReleaseUPP( userUPP: RTPMPDataReleaseUPP ); external name '_DisposeRTPMPDataReleaseUPP';
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
procedure DisposeRTPMPDataReleaseUPP(userUPP: RTPMPDataReleaseUPP); external name '_DisposeRTPMPDataReleaseUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeRTPMPDataReleaseUPP( userUPP: RTPMPDataReleaseUPP ); external name '_DisposeRTPMPDataReleaseUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeRTPPBCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeRTPPBCallbackUPP( userUPP: RTPPBCallbackUPP ); external name '_DisposeRTPPBCallbackUPP';
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
procedure DisposeRTPPBCallbackUPP(userUPP: RTPPBCallbackUPP); external name '_DisposeRTPPBCallbackUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeRTPPBCallbackUPP( userUPP: RTPPBCallbackUPP ); external name '_DisposeRTPPBCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeRTPMPDataReleaseUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeRTPMPDataReleaseUPP( var inData: UInt8; inRefCon: UnivPtr; userUPP: RTPMPDataReleaseUPP ); external name '_InvokeRTPMPDataReleaseUPP';
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
procedure InvokeRTPMPDataReleaseUPP(var inData: UInt8; inRefCon: UnivPtr; userRoutine: RTPMPDataReleaseUPP); external name '_InvokeRTPMPDataReleaseUPP'; { old name was CallRTPMPDataReleaseProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvokeRTPMPDataReleaseUPP( var inData: UInt8; inRefCon: UnivPtr; userUPP: RTPMPDataReleaseUPP ); external name '_InvokeRTPMPDataReleaseUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeRTPPBCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeRTPPBCallbackUPP( inSelector: OSType; ioParams: UnivPtr; inRefCon: UnivPtr; userUPP: RTPPBCallbackUPP ); external name '_InvokeRTPPBCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

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
procedure InvokeRTPPBCallbackUPP( inSelector: OSType; ioParams: UnivPtr; inRefCon: UnivPtr; userUPP: RTPPBCallbackUPP ); external name '_InvokeRTPPBCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
