{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/MediaHandlers.h
 
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
     File:       MediaHandlers.p
=======
     File:       QuickTime/MediaHandlers.h
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

unit MediaHandlers;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,Components,Events,ImageCompression,Movies,QuickdrawTypes,Sound;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======


{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}
>>>>>>> origin/cpstrnew

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

type
	PrePrerollCompleteProcPtr = procedure( mh: MediaHandler; err: OSErr; refcon: UnivPtr );
	PrePrerollCompleteUPP = PrePrerollCompleteProcPtr;
{ derived media handler capability flags }
const
	handlerHasSpatial = 1 shl 0;
	handlerCanClip = 1 shl 1;
	handlerCanMatte = 1 shl 2;
	handlerCanTransferMode = 1 shl 3;
	handlerNeedsBuffer = 1 shl 4;
	handlerNoIdle = 1 shl 5;
	handlerNoScheduler = 1 shl 6;
	handlerWantsTime = 1 shl 7;
	handlerCGrafPortOnly = 1 shl 8;
	handlerCanSend = 1 shl 9;
	handlerCanHandleComplexMatrix = 1 shl 10;
	handlerWantsDestinationPixels = 1 shl 11;
	handlerCanSendImageData = 1 shl 12;
	handlerCanPicSave = 1 shl 13;

{ media task flags }
const
	mMustDraw = 1 shl 3;
	mAtEnd = 1 shl 4;
	mPreflightDraw = 1 shl 5;
	mSyncDrawing = 1 shl 6;
	mPrecompositeOnly = 1 shl 9;
	mSoundOnly = 1 shl 10;
	mDoIdleActionsBeforeDraws = 1 shl 11;
	mDisableIdleActions = 1 shl 12;

{ media task result flags }
const
	mDidDraw = 1 shl 0;
	mNeedsToDraw = 1 shl 2;
	mDrawAgain = 1 shl 3;
	mPartialDraw = 1 shl 4;
	mWantIdleActions = 1 shl 5;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

const
	forceUpdateRedraw = 1 shl 0;
	forceUpdateNewBuffer = 1 shl 1;

{ media hit test flags }
const
	mHitTestBounds = 1 shl 0; {    point must only be within targetRefCon's bounding box }
	mHitTestImage = 1 shl 1; {  point must be within the shape of the targetRefCon's image }
	mHitTestInvisible = 1 shl 2; {  invisible targetRefCon's may be hit tested }
	mHitTestIsClick = 1 shl 3; {  for codecs that want mouse events }

{ media is opaque flags }
const
	mOpaque = 1 shl 0;
	mInvisible = 1 shl 1;
<<<<<<< HEAD

=======
=======
>>>>>>> graemeg/cpstrnew

const
	forceUpdateRedraw = 1 shl 0;
	forceUpdateNewBuffer = 1 shl 1;

{ media hit test flags }
const
	mHitTestBounds = 1 shl 0; {    point must only be within targetRefCon's bounding box }
	mHitTestImage = 1 shl 1; {  point must be within the shape of the targetRefCon's image }
	mHitTestInvisible = 1 shl 2; {  invisible targetRefCon's may be hit tested }
	mHitTestIsClick = 1 shl 3; {  for codecs that want mouse events }

{ media is opaque flags }
const
	mOpaque = 1 shl 0;
	mInvisible = 1 shl 1;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
{ MediaSetPublicInfo/MediaGetPublicInfo selectors }
const
	kMediaQTIdleFrequencySelector = FourCharCode('idfq');

type
	GetMovieCompleteParamsPtr = ^GetMovieCompleteParams;
	GetMovieCompleteParams = record
		version: SInt16;
		theMovie: Movie;
		theTrack: Track;
		theMedia: Media;
		movieScale: TimeScale;
		mediaScale: TimeScale;
		movieDuration: TimeValue;
		trackDuration: TimeValue;
		mediaDuration: TimeValue;
		effectiveRate: Fixed;
		timeBase: TimeBase_fix;
		volume: SInt16;
		width: Fixed;
		height: Fixed;
		trackMovieMatrix: MatrixRecord;
		moviePort: CGrafPtr;
		movieGD: GDHandle;
		trackMatte: PixMapHandle;
		inputMap: QTAtomContainer;
		mediaContextID: QTMediaContextID;
	end;
const
	kMediaVideoParamBrightness = 1;
	kMediaVideoParamContrast = 2;
	kMediaVideoParamHue = 3;
	kMediaVideoParamSharpness = 4;
	kMediaVideoParamSaturation = 5;
	kMediaVideoParamBlackLevel = 6;
	kMediaVideoParamWhiteLevel = 7;

{ These are for MediaGetInfo() and MediaSetInfo().}
const
	kMHInfoEncodedFrameRate = FourCharCode('orat'); { Parameter is a MHInfoEncodedFrameRateRecord*.}

{ This holds the frame rate at which the track was encoded.}
type
	MHInfoEncodedFrameRateRecordPtr = ^MHInfoEncodedFrameRateRecord;
	MHInfoEncodedFrameRateRecord = record
		encodedFrameRate: Fixed;
	end;

type
	dataHandlePtr = ^Handle;
	dataHandleHandle = ^dataHandlePtr;

type
	QTCustomActionTargetRecordPtr = ^QTCustomActionTargetRecord;
	QTCustomActionTargetRecord = record
		movie: Movie_fix;
		doMCActionCallbackProc: DoMCActionUPP;
		callBackRefcon: SIGNEDLONG;
		track: Track_fix;
		trackObjectRefCon: SIGNEDLONG;
		defaultTrack: Track_fix;
		defaultObjectRefCon: SIGNEDLONG;
		reserved1: SIGNEDLONG;
		reserved2: SIGNEDLONG;
	end;
type
	QTCustomActionTargetPtr = ^QTCustomActionTargetRecord;
	MediaEQSpectrumBandsRecord = record
		count: SInt16;
		frequency: UnsignedFixedPtr;              { pointer to array of frequencies}
	end;
	MediaEQSpectrumBandsRecordPtr = ^MediaEQSpectrumBandsRecord;
{
 *  CallComponentExecuteWiredAction()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function CallComponentExecuteWiredAction( ci: ComponentInstance; actionContainer: QTAtomContainer; actionAtom: QTAtom; target: QTCustomActionTargetPtr; event: QTEventRecordPtr ): ComponentResult; external name '_CallComponentExecuteWiredAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Quickdraw,Events,ImageCompression,ConditionalMacros,Components,Sound,Movies;
=======
uses MacTypes,Components,Events,ImageCompression,Movies,QuickdrawTypes,Sound;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

type
	PrePrerollCompleteProcPtr = procedure( mh: MediaHandler; err: OSErr; refcon: UnivPtr );
	PrePrerollCompleteUPP = PrePrerollCompleteProcPtr;
{ derived media handler capability flags }
const
	handlerHasSpatial = 1 shl 0;
	handlerCanClip = 1 shl 1;
	handlerCanMatte = 1 shl 2;
	handlerCanTransferMode = 1 shl 3;
	handlerNeedsBuffer = 1 shl 4;
	handlerNoIdle = 1 shl 5;
	handlerNoScheduler = 1 shl 6;
	handlerWantsTime = 1 shl 7;
	handlerCGrafPortOnly = 1 shl 8;
	handlerCanSend = 1 shl 9;
	handlerCanHandleComplexMatrix = 1 shl 10;
	handlerWantsDestinationPixels = 1 shl 11;
	handlerCanSendImageData = 1 shl 12;
	handlerCanPicSave = 1 shl 13;

{ media task flags }
const
	mMustDraw = 1 shl 3;
	mAtEnd = 1 shl 4;
	mPreflightDraw = 1 shl 5;
	mSyncDrawing = 1 shl 6;
	mPrecompositeOnly = 1 shl 9;
	mSoundOnly = 1 shl 10;
	mDoIdleActionsBeforeDraws = 1 shl 11;
	mDisableIdleActions = 1 shl 12;

{ media task result flags }
const
	mDidDraw = 1 shl 0;
	mNeedsToDraw = 1 shl 2;
	mDrawAgain = 1 shl 3;
	mPartialDraw = 1 shl 4;
	mWantIdleActions = 1 shl 5;

const
	forceUpdateRedraw = 1 shl 0;
	forceUpdateNewBuffer = 1 shl 1;

{ media hit test flags }
const
	mHitTestBounds = 1 shl 0; {    point must only be within targetRefCon's bounding box }
	mHitTestImage = 1 shl 1; {  point must be within the shape of the targetRefCon's image }
	mHitTestInvisible = 1 shl 2; {  invisible targetRefCon's may be hit tested }
	mHitTestIsClick = 1 shl 3; {  for codecs that want mouse events }

{ media is opaque flags }
const
	mOpaque = 1 shl 0;
	mInvisible = 1 shl 1;

{ MediaSetPublicInfo/MediaGetPublicInfo selectors }
const
	kMediaQTIdleFrequencySelector = FourCharCode('idfq');

type
	GetMovieCompleteParamsPtr = ^GetMovieCompleteParams;
	GetMovieCompleteParams = record
		version: SInt16;
		theMovie: Movie;
		theTrack: Track;
		theMedia: Media;
		movieScale: TimeScale;
		mediaScale: TimeScale;
		movieDuration: TimeValue;
		trackDuration: TimeValue;
		mediaDuration: TimeValue;
		effectiveRate: Fixed;
		timeBase: TimeBase_fix;
		volume: SInt16;
		width: Fixed;
		height: Fixed;
		trackMovieMatrix: MatrixRecord;
		moviePort: CGrafPtr;
		movieGD: GDHandle;
		trackMatte: PixMapHandle;
		inputMap: QTAtomContainer;
		mediaContextID: QTMediaContextID;
	end;
const
	kMediaVideoParamBrightness = 1;
	kMediaVideoParamContrast = 2;
	kMediaVideoParamHue = 3;
	kMediaVideoParamSharpness = 4;
	kMediaVideoParamSaturation = 5;
	kMediaVideoParamBlackLevel = 6;
	kMediaVideoParamWhiteLevel = 7;

{ These are for MediaGetInfo() and MediaSetInfo().}
const
	kMHInfoEncodedFrameRate = FourCharCode('orat'); { Parameter is a MHInfoEncodedFrameRateRecord*.}

{ This holds the frame rate at which the track was encoded.}
type
	MHInfoEncodedFrameRateRecordPtr = ^MHInfoEncodedFrameRateRecord;
	MHInfoEncodedFrameRateRecord = record
		encodedFrameRate: Fixed;
	end;

type
	dataHandlePtr = ^Handle;
	dataHandleHandle = ^dataHandlePtr;

type
	QTCustomActionTargetRecordPtr = ^QTCustomActionTargetRecord;
	QTCustomActionTargetRecord = record
		movie: Movie_fix;
		doMCActionCallbackProc: DoMCActionUPP;
		callBackRefcon: SIGNEDLONG;
		track: Track_fix;
		trackObjectRefCon: SIGNEDLONG;
		defaultTrack: Track_fix;
		defaultObjectRefCon: SIGNEDLONG;
		reserved1: SIGNEDLONG;
		reserved2: SIGNEDLONG;
	end;
type
	QTCustomActionTargetPtr = ^QTCustomActionTargetRecord;
	MediaEQSpectrumBandsRecord = record
		count: SInt16;
		frequency: UnsignedFixedPtr;              { pointer to array of frequencies}
	end;
	MediaEQSpectrumBandsRecordPtr = ^MediaEQSpectrumBandsRecord;
{
 *  CallComponentExecuteWiredAction()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function CallComponentExecuteWiredAction( ci: ComponentInstance; actionContainer: QTAtomContainer; actionAtom: QTAtom; target: QTCustomActionTargetPtr; event: QTEventRecordPtr ): ComponentResult; external name '_CallComponentExecuteWiredAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
	{
	 *  CallComponentExecuteWiredAction()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.0.2 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 4.0 and later
	 	}
function CallComponentExecuteWiredAction(ci: ComponentInstance; actionContainer: QTAtomContainer; actionAtom: QTAtom; target: QTCustomActionTargetPtr; event: QTEventRecordPtr): ComponentResult; external name '_CallComponentExecuteWiredAction';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ MediaCallRange2 }
{ These are unique to each type of media handler }
{ They are also included in the public interfaces }


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Flags for MediaSetChunkManagementFlags}
const
	kEmptyPurgableChunksOverAllowance = 1;

{
 *  MediaSetChunkManagementFlags()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaSetChunkManagementFlags( mh: MediaHandler; flags: UInt32; flagsMask: UInt32 ): ComponentResult; external name '_MediaSetChunkManagementFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
{  Flags for MediaSetChunkManagementFlags }

const
	kEmptyPurgableChunksOverAllowance = 1;

	{
	 *  MediaSetChunkManagementFlags()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
	 *    CarbonLib:        in CarbonLib 1.6 and later
	 *    Mac OS X:         in version 10.2 and later
	 *    Windows:          in qtmlClient.lib 6.0 and later
	 	}
function MediaSetChunkManagementFlags(mh: MediaHandler; flags: UInt32; flagsMask: UInt32): ComponentResult; external name '_MediaSetChunkManagementFlags';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Flags for MediaSetChunkManagementFlags}
const
	kEmptyPurgableChunksOverAllowance = 1;

>>>>>>> origin/fixes_2.4
{
 *  MediaSetChunkManagementFlags()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaGetChunkManagementFlags( mh: MediaHandler; var flags: UInt32 ): ComponentResult; external name '_MediaGetChunkManagementFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaSetChunkManagementFlags( mh: MediaHandler; flags: UInt32; flagsMask: UInt32 ): ComponentResult; external name '_MediaSetChunkManagementFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  MediaGetChunkManagementFlags()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function MediaGetChunkManagementFlags(mh: MediaHandler; var flags: UInt32): ComponentResult; external name '_MediaGetChunkManagementFlags';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetChunkManagementFlags( mh: MediaHandler; var flags: UInt32 ): ComponentResult; external name '_MediaGetChunkManagementFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetPurgeableChunkMemoryAllowance()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaSetPurgeableChunkMemoryAllowance( mh: MediaHandler; allowance: Size ): ComponentResult; external name '_MediaSetPurgeableChunkMemoryAllowance';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function MediaSetPurgeableChunkMemoryAllowance(mh: MediaHandler; allowance: Size): ComponentResult; external name '_MediaSetPurgeableChunkMemoryAllowance';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetPurgeableChunkMemoryAllowance( mh: MediaHandler; allowance: Size ): ComponentResult; external name '_MediaSetPurgeableChunkMemoryAllowance';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetPurgeableChunkMemoryAllowance()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaGetPurgeableChunkMemoryAllowance( mh: MediaHandler; var allowance: Size ): ComponentResult; external name '_MediaGetPurgeableChunkMemoryAllowance';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function MediaGetPurgeableChunkMemoryAllowance(mh: MediaHandler; var allowance: Size): ComponentResult; external name '_MediaGetPurgeableChunkMemoryAllowance';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetPurgeableChunkMemoryAllowance( mh: MediaHandler; var allowance: Size ): ComponentResult; external name '_MediaGetPurgeableChunkMemoryAllowance';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaEmptyAllPurgeableChunks()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaEmptyAllPurgeableChunks( mh: MediaHandler ): ComponentResult; external name '_MediaEmptyAllPurgeableChunks';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function MediaEmptyAllPurgeableChunks(mh: MediaHandler): ComponentResult; external name '_MediaEmptyAllPurgeableChunks';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaEmptyAllPurgeableChunks( mh: MediaHandler ): ComponentResult; external name '_MediaEmptyAllPurgeableChunks';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{**** These are the calls for dealing with the Generic media handler ****}
{
 *  MediaInitialize()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaInitialize( mh: MediaHandler; var gmc: GetMovieCompleteParams ): ComponentResult; external name '_MediaInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaInitialize(mh: MediaHandler; var gmc: GetMovieCompleteParams): ComponentResult; external name '_MediaInitialize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaInitialize( mh: MediaHandler; var gmc: GetMovieCompleteParams ): ComponentResult; external name '_MediaInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetHandlerCapabilities()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetHandlerCapabilities( mh: MediaHandler; flags: SIGNEDLONG; flagsMask: SIGNEDLONG ): ComponentResult; external name '_MediaSetHandlerCapabilities';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetHandlerCapabilities(mh: MediaHandler; flags: SInt32; flagsMask: SInt32): ComponentResult; external name '_MediaSetHandlerCapabilities';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetHandlerCapabilities( mh: MediaHandler; flags: SIGNEDLONG; flagsMask: SIGNEDLONG ): ComponentResult; external name '_MediaSetHandlerCapabilities';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaIdle()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaIdle( mh: MediaHandler; atMediaTime: TimeValue; flagsIn: SIGNEDLONG; var flagsOut: SIGNEDLONG; const (*var*) movieTime: TimeRecord ): ComponentResult; external name '_MediaIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaIdle(mh: MediaHandler; atMediaTime: TimeValue; flagsIn: SInt32; var flagsOut: SInt32; const (*var*) movieTime: TimeRecord): ComponentResult; external name '_MediaIdle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaIdle( mh: MediaHandler; atMediaTime: TimeValue; flagsIn: SIGNEDLONG; var flagsOut: SIGNEDLONG; const (*var*) movieTime: TimeRecord ): ComponentResult; external name '_MediaIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetMediaInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetMediaInfo( mh: MediaHandler; h: Handle ): ComponentResult; external name '_MediaGetMediaInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetMediaInfo(mh: MediaHandler; h: Handle): ComponentResult; external name '_MediaGetMediaInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetMediaInfo( mh: MediaHandler; h: Handle ): ComponentResult; external name '_MediaGetMediaInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaPutMediaInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaPutMediaInfo( mh: MediaHandler; h: Handle ): ComponentResult; external name '_MediaPutMediaInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaPutMediaInfo(mh: MediaHandler; h: Handle): ComponentResult; external name '_MediaPutMediaInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaPutMediaInfo( mh: MediaHandler; h: Handle ): ComponentResult; external name '_MediaPutMediaInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetActive()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetActive( mh: MediaHandler; enableMedia: Boolean ): ComponentResult; external name '_MediaSetActive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetActive(mh: MediaHandler; enableMedia: boolean): ComponentResult; external name '_MediaSetActive';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetActive( mh: MediaHandler; enableMedia: Boolean ): ComponentResult; external name '_MediaSetActive';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetRate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetRate( mh: MediaHandler; rate: Fixed ): ComponentResult; external name '_MediaSetRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetRate(mh: MediaHandler; rate: Fixed): ComponentResult; external name '_MediaSetRate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetRate( mh: MediaHandler; rate: Fixed ): ComponentResult; external name '_MediaSetRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGGetStatus()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGGetStatus( mh: MediaHandler; var statusErr: ComponentResult ): ComponentResult; external name '_MediaGGetStatus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGGetStatus(mh: MediaHandler; var statusErr: ComponentResult): ComponentResult; external name '_MediaGGetStatus';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGGetStatus( mh: MediaHandler; var statusErr: ComponentResult ): ComponentResult; external name '_MediaGGetStatus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaTrackEdited()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaTrackEdited( mh: MediaHandler ): ComponentResult; external name '_MediaTrackEdited';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaTrackEdited(mh: MediaHandler): ComponentResult; external name '_MediaTrackEdited';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaTrackEdited( mh: MediaHandler ): ComponentResult; external name '_MediaTrackEdited';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetMediaTimeScale()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetMediaTimeScale( mh: MediaHandler; newTimeScale: TimeScale ): ComponentResult; external name '_MediaSetMediaTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetMediaTimeScale(mh: MediaHandler; newTimeScale: TimeScale): ComponentResult; external name '_MediaSetMediaTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetMediaTimeScale( mh: MediaHandler; newTimeScale: TimeScale ): ComponentResult; external name '_MediaSetMediaTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetMovieTimeScale()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetMovieTimeScale( mh: MediaHandler; newTimeScale: TimeScale ): ComponentResult; external name '_MediaSetMovieTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetMovieTimeScale(mh: MediaHandler; newTimeScale: TimeScale): ComponentResult; external name '_MediaSetMovieTimeScale';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetMovieTimeScale( mh: MediaHandler; newTimeScale: TimeScale ): ComponentResult; external name '_MediaSetMovieTimeScale';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetGWorld()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetGWorld( mh: MediaHandler; aPort: CGrafPtr; aGD: GDHandle ): ComponentResult; external name '_MediaSetGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetGWorld(mh: MediaHandler; aPort: CGrafPtr; aGD: GDHandle): ComponentResult; external name '_MediaSetGWorld';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetGWorld( mh: MediaHandler; aPort: CGrafPtr; aGD: GDHandle ): ComponentResult; external name '_MediaSetGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetDimensions()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetDimensions( mh: MediaHandler; width: Fixed; height: Fixed ): ComponentResult; external name '_MediaSetDimensions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetDimensions(mh: MediaHandler; width: Fixed; height: Fixed): ComponentResult; external name '_MediaSetDimensions';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  MediaSetClip()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetClip( mh: MediaHandler; theClip: RgnHandle ): ComponentResult; external name '_MediaSetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetClip(mh: MediaHandler; theClip: RgnHandle): ComponentResult; external name '_MediaSetClip';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetClip( mh: MediaHandler; theClip: RgnHandle ): ComponentResult; external name '_MediaSetClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetMatrix()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetMatrix( mh: MediaHandler; var trackMovieMatrix: MatrixRecord ): ComponentResult; external name '_MediaSetMatrix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetMatrix(mh: MediaHandler; var trackMovieMatrix: MatrixRecord): ComponentResult; external name '_MediaSetMatrix';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetMatrix( mh: MediaHandler; var trackMovieMatrix: MatrixRecord ): ComponentResult; external name '_MediaSetMatrix';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetTrackOpaque()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetTrackOpaque( mh: MediaHandler; var trackIsOpaque: Boolean ): ComponentResult; external name '_MediaGetTrackOpaque';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetTrackOpaque(mh: MediaHandler; var trackIsOpaque: boolean): ComponentResult; external name '_MediaGetTrackOpaque';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetTrackOpaque( mh: MediaHandler; var trackIsOpaque: Boolean ): ComponentResult; external name '_MediaGetTrackOpaque';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetGraphicsMode()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetGraphicsMode( mh: MediaHandler; mode: SIGNEDLONG; const (*var*) opColor: RGBColor ): ComponentResult; external name '_MediaSetGraphicsMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetGraphicsMode(mh: MediaHandler; mode: SInt32; const (*var*) opColor: RGBColor): ComponentResult; external name '_MediaSetGraphicsMode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetGraphicsMode( mh: MediaHandler; mode: SIGNEDLONG; const (*var*) opColor: RGBColor ): ComponentResult; external name '_MediaSetGraphicsMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetGraphicsMode()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetGraphicsMode( mh: MediaHandler; var mode: SIGNEDLONG; var opColor: RGBColor ): ComponentResult; external name '_MediaGetGraphicsMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetGraphicsMode(mh: MediaHandler; var mode: SInt32; var opColor: RGBColor): ComponentResult; external name '_MediaGetGraphicsMode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetGraphicsMode( mh: MediaHandler; var mode: SIGNEDLONG; var opColor: RGBColor ): ComponentResult; external name '_MediaGetGraphicsMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGSetVolume()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGSetVolume( mh: MediaHandler; volume: SInt16 ): ComponentResult; external name '_MediaGSetVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGSetVolume(mh: MediaHandler; volume: SInt16): ComponentResult; external name '_MediaGSetVolume';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGSetVolume( mh: MediaHandler; volume: SInt16 ): ComponentResult; external name '_MediaGSetVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetSoundBalance()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetSoundBalance( mh: MediaHandler; balance: SInt16 ): ComponentResult; external name '_MediaSetSoundBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetSoundBalance(mh: MediaHandler; balance: SInt16): ComponentResult; external name '_MediaSetSoundBalance';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetSoundBalance( mh: MediaHandler; balance: SInt16 ): ComponentResult; external name '_MediaSetSoundBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSoundBalance()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetSoundBalance( mh: MediaHandler; var balance: SInt16 ): ComponentResult; external name '_MediaGetSoundBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetSoundBalance(mh: MediaHandler; var balance: SInt16): ComponentResult; external name '_MediaGetSoundBalance';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSoundBalance( mh: MediaHandler; var balance: SInt16 ): ComponentResult; external name '_MediaGetSoundBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetNextBoundsChange()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetNextBoundsChange( mh: MediaHandler; var when: TimeValue ): ComponentResult; external name '_MediaGetNextBoundsChange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetNextBoundsChange(mh: MediaHandler; var when: TimeValue): ComponentResult; external name '_MediaGetNextBoundsChange';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetNextBoundsChange( mh: MediaHandler; var when: TimeValue ): ComponentResult; external name '_MediaGetNextBoundsChange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSrcRgn()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetSrcRgn( mh: MediaHandler; rgn: RgnHandle; atMediaTime: TimeValue ): ComponentResult; external name '_MediaGetSrcRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetSrcRgn(mh: MediaHandler; rgn: RgnHandle; atMediaTime: TimeValue): ComponentResult; external name '_MediaGetSrcRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSrcRgn( mh: MediaHandler; rgn: RgnHandle; atMediaTime: TimeValue ): ComponentResult; external name '_MediaGetSrcRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaPreroll()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaPreroll( mh: MediaHandler; time: TimeValue; rate: Fixed ): ComponentResult; external name '_MediaPreroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaPreroll(mh: MediaHandler; time: TimeValue; rate: Fixed): ComponentResult; external name '_MediaPreroll';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaPreroll( mh: MediaHandler; time: TimeValue; rate: Fixed ): ComponentResult; external name '_MediaPreroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSampleDescriptionChanged()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSampleDescriptionChanged( mh: MediaHandler; index: SIGNEDLONG ): ComponentResult; external name '_MediaSampleDescriptionChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSampleDescriptionChanged(mh: MediaHandler; index: SInt32): ComponentResult; external name '_MediaSampleDescriptionChanged';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSampleDescriptionChanged( mh: MediaHandler; index: SIGNEDLONG ): ComponentResult; external name '_MediaSampleDescriptionChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaHasCharacteristic()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaHasCharacteristic( mh: MediaHandler; characteristic: OSType; var hasIt: Boolean ): ComponentResult; external name '_MediaHasCharacteristic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaHasCharacteristic(mh: MediaHandler; characteristic: OSType; var hasIt: boolean): ComponentResult; external name '_MediaHasCharacteristic';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaHasCharacteristic( mh: MediaHandler; characteristic: OSType; var hasIt: Boolean ): ComponentResult; external name '_MediaHasCharacteristic';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetOffscreenBufferSize()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetOffscreenBufferSize( mh: MediaHandler; var bounds: Rect; depth: SInt16; ctab: CTabHandle ): ComponentResult; external name '_MediaGetOffscreenBufferSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetOffscreenBufferSize(mh: MediaHandler; var bounds: Rect; depth: SInt16; ctab: CTabHandle): ComponentResult; external name '_MediaGetOffscreenBufferSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetOffscreenBufferSize( mh: MediaHandler; var bounds: Rect; depth: SInt16; ctab: CTabHandle ): ComponentResult; external name '_MediaGetOffscreenBufferSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetHints()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetHints( mh: MediaHandler; hints: SIGNEDLONG ): ComponentResult; external name '_MediaSetHints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetHints(mh: MediaHandler; hints: SInt32): ComponentResult; external name '_MediaSetHints';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetHints( mh: MediaHandler; hints: SIGNEDLONG ): ComponentResult; external name '_MediaSetHints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetName( mh: MediaHandler; var name: Str255; requestedLanguage: SIGNEDLONG; var actualLanguage: SIGNEDLONG ): ComponentResult; external name '_MediaGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetName(mh: MediaHandler; var name: Str255; requestedLanguage: SInt32; var actualLanguage: SInt32): ComponentResult; external name '_MediaGetName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetName( mh: MediaHandler; var name: Str255; requestedLanguage: SIGNEDLONG; var actualLanguage: SIGNEDLONG ): ComponentResult; external name '_MediaGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaForceUpdate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaForceUpdate( mh: MediaHandler; forceUpdateFlags: SIGNEDLONG ): ComponentResult; external name '_MediaForceUpdate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaForceUpdate(mh: MediaHandler; forceUpdateFlags: SInt32): ComponentResult; external name '_MediaForceUpdate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaForceUpdate( mh: MediaHandler; forceUpdateFlags: SIGNEDLONG ): ComponentResult; external name '_MediaForceUpdate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetDrawingRgn()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetDrawingRgn( mh: MediaHandler; var partialRgn: RgnHandle ): ComponentResult; external name '_MediaGetDrawingRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetDrawingRgn(mh: MediaHandler; var partialRgn: RgnHandle): ComponentResult; external name '_MediaGetDrawingRgn';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetDrawingRgn( mh: MediaHandler; var partialRgn: RgnHandle ): ComponentResult; external name '_MediaGetDrawingRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGSetActiveSegment()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGSetActiveSegment( mh: MediaHandler; activeStart: TimeValue; activeDuration: TimeValue ): ComponentResult; external name '_MediaGSetActiveSegment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGSetActiveSegment(mh: MediaHandler; activeStart: TimeValue; activeDuration: TimeValue): ComponentResult; external name '_MediaGSetActiveSegment';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGSetActiveSegment( mh: MediaHandler; activeStart: TimeValue; activeDuration: TimeValue ): ComponentResult; external name '_MediaGSetActiveSegment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaInvalidateRegion()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaInvalidateRegion( mh: MediaHandler; invalRgn: RgnHandle ): ComponentResult; external name '_MediaInvalidateRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaInvalidateRegion(mh: MediaHandler; invalRgn: RgnHandle): ComponentResult; external name '_MediaInvalidateRegion';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaInvalidateRegion( mh: MediaHandler; invalRgn: RgnHandle ): ComponentResult; external name '_MediaInvalidateRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetNextStepTime()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetNextStepTime( mh: MediaHandler; flags: SInt16; mediaTimeIn: TimeValue; var mediaTimeOut: TimeValue; rate: Fixed ): ComponentResult; external name '_MediaGetNextStepTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetNextStepTime(mh: MediaHandler; flags: SInt16; mediaTimeIn: TimeValue; var mediaTimeOut: TimeValue; rate: Fixed): ComponentResult; external name '_MediaGetNextStepTime';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetNextStepTime( mh: MediaHandler; flags: SInt16; mediaTimeIn: TimeValue; var mediaTimeOut: TimeValue; rate: Fixed ): ComponentResult; external name '_MediaGetNextStepTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetNonPrimarySourceData()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetNonPrimarySourceData( mh: MediaHandler; inputIndex: SIGNEDLONG; dataDescriptionSeed: SIGNEDLONG; dataDescription: Handle; data: UnivPtr; dataSize: SIGNEDLONG; asyncCompletionProc: ICMCompletionProcRecordPtr; transferProc: ICMConvertDataFormatUPP; refCon: UnivPtr ): ComponentResult; external name '_MediaSetNonPrimarySourceData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetNonPrimarySourceData(mh: MediaHandler; inputIndex: SInt32; dataDescriptionSeed: SInt32; dataDescription: Handle; data: UnivPtr; dataSize: SInt32; asyncCompletionProc: ICMCompletionProcRecordPtr; transferProc: ICMConvertDataFormatUPP; refCon: UnivPtr): ComponentResult; external name '_MediaSetNonPrimarySourceData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetNonPrimarySourceData( mh: MediaHandler; inputIndex: SIGNEDLONG; dataDescriptionSeed: SIGNEDLONG; dataDescription: Handle; data: UnivPtr; dataSize: SIGNEDLONG; asyncCompletionProc: ICMCompletionProcRecordPtr; transferProc: ICMConvertDataFormatUPP; refCon: UnivPtr ): ComponentResult; external name '_MediaSetNonPrimarySourceData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaChangedNonPrimarySource()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaChangedNonPrimarySource( mh: MediaHandler; inputIndex: SIGNEDLONG ): ComponentResult; external name '_MediaChangedNonPrimarySource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaChangedNonPrimarySource(mh: MediaHandler; inputIndex: SInt32): ComponentResult; external name '_MediaChangedNonPrimarySource';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaChangedNonPrimarySource( mh: MediaHandler; inputIndex: SIGNEDLONG ): ComponentResult; external name '_MediaChangedNonPrimarySource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaTrackReferencesChanged()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaTrackReferencesChanged( mh: MediaHandler ): ComponentResult; external name '_MediaTrackReferencesChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaTrackReferencesChanged(mh: MediaHandler): ComponentResult; external name '_MediaTrackReferencesChanged';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaTrackReferencesChanged( mh: MediaHandler ): ComponentResult; external name '_MediaTrackReferencesChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSampleDataPointer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetSampleDataPointer( mh: MediaHandler; sampleNum: SIGNEDLONG; var dataPtr: Ptr; var dataSize: SIGNEDLONG; var sampleDescIndex: SIGNEDLONG ): ComponentResult; external name '_MediaGetSampleDataPointer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetSampleDataPointer(mh: MediaHandler; sampleNum: SInt32; var dataPtr: Ptr; var dataSize: SInt32; var sampleDescIndex: SInt32): ComponentResult; external name '_MediaGetSampleDataPointer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSampleDataPointer( mh: MediaHandler; sampleNum: SIGNEDLONG; var dataPtr: Ptr; var dataSize: SIGNEDLONG; var sampleDescIndex: SIGNEDLONG ): ComponentResult; external name '_MediaGetSampleDataPointer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaReleaseSampleDataPointer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaReleaseSampleDataPointer( mh: MediaHandler; sampleNum: SIGNEDLONG ): ComponentResult; external name '_MediaReleaseSampleDataPointer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaReleaseSampleDataPointer(mh: MediaHandler; sampleNum: SInt32): ComponentResult; external name '_MediaReleaseSampleDataPointer';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaReleaseSampleDataPointer( mh: MediaHandler; sampleNum: SIGNEDLONG ): ComponentResult; external name '_MediaReleaseSampleDataPointer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaTrackPropertyAtomChanged()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaTrackPropertyAtomChanged( mh: MediaHandler ): ComponentResult; external name '_MediaTrackPropertyAtomChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaTrackPropertyAtomChanged(mh: MediaHandler): ComponentResult; external name '_MediaTrackPropertyAtomChanged';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaTrackPropertyAtomChanged( mh: MediaHandler ): ComponentResult; external name '_MediaTrackPropertyAtomChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetTrackInputMapReference()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetTrackInputMapReference( mh: MediaHandler; inputMap: QTAtomContainer ): ComponentResult; external name '_MediaSetTrackInputMapReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetTrackInputMapReference(mh: MediaHandler; inputMap: QTAtomContainer): ComponentResult; external name '_MediaSetTrackInputMapReference';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetTrackInputMapReference( mh: MediaHandler; inputMap: QTAtomContainer ): ComponentResult; external name '_MediaSetTrackInputMapReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetVideoParam()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetVideoParam( mh: MediaHandler; whichParam: SIGNEDLONG; var value: UInt16 ): ComponentResult; external name '_MediaSetVideoParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetVideoParam(mh: MediaHandler; whichParam: SInt32; var value: UInt16): ComponentResult; external name '_MediaSetVideoParam';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetVideoParam( mh: MediaHandler; whichParam: SIGNEDLONG; var value: UInt16 ): ComponentResult; external name '_MediaSetVideoParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetVideoParam()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetVideoParam( mh: MediaHandler; whichParam: SIGNEDLONG; var value: UInt16 ): ComponentResult; external name '_MediaGetVideoParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetVideoParam(mh: MediaHandler; whichParam: SInt32; var value: UInt16): ComponentResult; external name '_MediaGetVideoParam';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetVideoParam( mh: MediaHandler; whichParam: SIGNEDLONG; var value: UInt16 ): ComponentResult; external name '_MediaGetVideoParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaCompare()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaCompare( mh: MediaHandler; var isOK: Boolean; srcMedia: Media; srcMediaComponent: ComponentInstance ): ComponentResult; external name '_MediaCompare';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaCompare(mh: MediaHandler; var isOK: boolean; srcMedia: Media; srcMediaComponent: ComponentInstance): ComponentResult; external name '_MediaCompare';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaCompare( mh: MediaHandler; var isOK: Boolean; srcMedia: Media; srcMediaComponent: ComponentInstance ): ComponentResult; external name '_MediaCompare';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetClock()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetClock( mh: MediaHandler; var clock: ComponentInstance ): ComponentResult; external name '_MediaGetClock';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetClock(mh: MediaHandler; var clock: ComponentInstance): ComponentResult; external name '_MediaGetClock';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetClock( mh: MediaHandler; var clock: ComponentInstance ): ComponentResult; external name '_MediaGetClock';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetSoundOutputComponent()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetSoundOutputComponent( mh: MediaHandler; outputComponent: Component ): ComponentResult; external name '_MediaSetSoundOutputComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetSoundOutputComponent(mh: MediaHandler; outputComponent: Component): ComponentResult; external name '_MediaSetSoundOutputComponent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetSoundOutputComponent( mh: MediaHandler; outputComponent: Component ): ComponentResult; external name '_MediaSetSoundOutputComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSoundOutputComponent()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetSoundOutputComponent( mh: MediaHandler; var outputComponent: Component ): ComponentResult; external name '_MediaGetSoundOutputComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetSoundOutputComponent(mh: MediaHandler; var outputComponent: Component): ComponentResult; external name '_MediaGetSoundOutputComponent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSoundOutputComponent( mh: MediaHandler; var outputComponent: Component ): ComponentResult; external name '_MediaGetSoundOutputComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetSoundLocalizationData()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetSoundLocalizationData( mh: MediaHandler; data: Handle ): ComponentResult; external name '_MediaSetSoundLocalizationData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetSoundLocalizationData(mh: MediaHandler; data: Handle): ComponentResult; external name '_MediaSetSoundLocalizationData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetSoundLocalizationData( mh: MediaHandler; data: Handle ): ComponentResult; external name '_MediaSetSoundLocalizationData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetInvalidRegion()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetInvalidRegion( mh: MediaHandler; rgn: RgnHandle ): ComponentResult; external name '_MediaGetInvalidRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetInvalidRegion(mh: MediaHandler; rgn: RgnHandle): ComponentResult; external name '_MediaGetInvalidRegion';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetInvalidRegion( mh: MediaHandler; rgn: RgnHandle ): ComponentResult; external name '_MediaGetInvalidRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSampleDescriptionB2N()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSampleDescriptionB2N( mh: MediaHandler; sampleDescriptionH: SampleDescriptionHandle ): ComponentResult; external name '_MediaSampleDescriptionB2N';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSampleDescriptionB2N(mh: MediaHandler; sampleDescriptionH: SampleDescriptionHandle): ComponentResult; external name '_MediaSampleDescriptionB2N';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSampleDescriptionB2N( mh: MediaHandler; sampleDescriptionH: SampleDescriptionHandle ): ComponentResult; external name '_MediaSampleDescriptionB2N';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSampleDescriptionN2B()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSampleDescriptionN2B( mh: MediaHandler; sampleDescriptionH: SampleDescriptionHandle ): ComponentResult; external name '_MediaSampleDescriptionN2B';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSampleDescriptionN2B(mh: MediaHandler; sampleDescriptionH: SampleDescriptionHandle): ComponentResult; external name '_MediaSampleDescriptionN2B';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSampleDescriptionN2B( mh: MediaHandler; sampleDescriptionH: SampleDescriptionHandle ): ComponentResult; external name '_MediaSampleDescriptionN2B';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaQueueNonPrimarySourceData()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaQueueNonPrimarySourceData( mh: MediaHandler; inputIndex: SIGNEDLONG; dataDescriptionSeed: SIGNEDLONG; dataDescription: Handle; data: UnivPtr; dataSize: SIGNEDLONG; asyncCompletionProc: ICMCompletionProcRecordPtr; const (*var*) frameTime: ICMFrameTimeRecord; transferProc: ICMConvertDataFormatUPP; refCon: UnivPtr ): ComponentResult; external name '_MediaQueueNonPrimarySourceData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaQueueNonPrimarySourceData(mh: MediaHandler; inputIndex: SInt32; dataDescriptionSeed: SInt32; dataDescription: Handle; data: UnivPtr; dataSize: SInt32; asyncCompletionProc: ICMCompletionProcRecordPtr; const (*var*) frameTime: ICMFrameTimeRecord; transferProc: ICMConvertDataFormatUPP; refCon: UnivPtr): ComponentResult; external name '_MediaQueueNonPrimarySourceData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaQueueNonPrimarySourceData( mh: MediaHandler; inputIndex: SIGNEDLONG; dataDescriptionSeed: SIGNEDLONG; dataDescription: Handle; data: UnivPtr; dataSize: SIGNEDLONG; asyncCompletionProc: ICMCompletionProcRecordPtr; const (*var*) frameTime: ICMFrameTimeRecord; transferProc: ICMConvertDataFormatUPP; refCon: UnivPtr ): ComponentResult; external name '_MediaQueueNonPrimarySourceData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaFlushNonPrimarySourceData()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaFlushNonPrimarySourceData( mh: MediaHandler; inputIndex: SIGNEDLONG ): ComponentResult; external name '_MediaFlushNonPrimarySourceData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaFlushNonPrimarySourceData(mh: MediaHandler; inputIndex: SInt32): ComponentResult; external name '_MediaFlushNonPrimarySourceData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaFlushNonPrimarySourceData( mh: MediaHandler; inputIndex: SIGNEDLONG ): ComponentResult; external name '_MediaFlushNonPrimarySourceData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetURLLink()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetURLLink( mh: MediaHandler; displayWhere: Point; var urlLink: Handle ): ComponentResult; external name '_MediaGetURLLink';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetURLLink(mh: MediaHandler; displayWhere: Point; var urlLink: Handle): ComponentResult; external name '_MediaGetURLLink';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetURLLink( mh: MediaHandler; displayWhere: Point; var urlLink: Handle ): ComponentResult; external name '_MediaGetURLLink';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaMakeMediaTimeTable()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaMakeMediaTimeTable( mh: MediaHandler; var offsets: SIGNEDLONGPtr; startTime: TimeValue; endTime: TimeValue; timeIncrement: TimeValue; firstDataRefIndex: SInt16; lastDataRefIndex: SInt16; var retDataRefSkew: SIGNEDLONG ): ComponentResult; external name '_MediaMakeMediaTimeTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaMakeMediaTimeTable(mh: MediaHandler; var offsets: SInt32Ptr; startTime: TimeValue; endTime: TimeValue; timeIncrement: TimeValue; firstDataRefIndex: SInt16; lastDataRefIndex: SInt16; var retDataRefSkew: SInt32): ComponentResult; external name '_MediaMakeMediaTimeTable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaMakeMediaTimeTable( mh: MediaHandler; var offsets: SIGNEDLONGPtr; startTime: TimeValue; endTime: TimeValue; timeIncrement: TimeValue; firstDataRefIndex: SInt16; lastDataRefIndex: SInt16; var retDataRefSkew: SIGNEDLONG ): ComponentResult; external name '_MediaMakeMediaTimeTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaHitTestForTargetRefCon()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaHitTestForTargetRefCon( mh: MediaHandler; flags: SIGNEDLONG; loc: Point; var targetRefCon: SIGNEDLONG ): ComponentResult; external name '_MediaHitTestForTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaHitTestForTargetRefCon(mh: MediaHandler; flags: SInt32; loc: Point; var targetRefCon: SInt32): ComponentResult; external name '_MediaHitTestForTargetRefCon';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaHitTestForTargetRefCon( mh: MediaHandler; flags: SIGNEDLONG; loc: Point; var targetRefCon: SIGNEDLONG ): ComponentResult; external name '_MediaHitTestForTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaHitTestTargetRefCon()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaHitTestTargetRefCon( mh: MediaHandler; targetRefCon: SIGNEDLONG; flags: SIGNEDLONG; loc: Point; var wasHit: Boolean ): ComponentResult; external name '_MediaHitTestTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaHitTestTargetRefCon(mh: MediaHandler; targetRefCon: SInt32; flags: SInt32; loc: Point; var wasHit: boolean): ComponentResult; external name '_MediaHitTestTargetRefCon';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaHitTestTargetRefCon( mh: MediaHandler; targetRefCon: SIGNEDLONG; flags: SIGNEDLONG; loc: Point; var wasHit: Boolean ): ComponentResult; external name '_MediaHitTestTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetActionsForQTEvent()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaGetActionsForQTEvent( mh: MediaHandler; event: QTEventRecordPtr; targetRefCon: SIGNEDLONG; var container: QTAtomContainer; var atom: QTAtom ): ComponentResult; external name '_MediaGetActionsForQTEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaGetActionsForQTEvent(mh: MediaHandler; event: QTEventRecordPtr; targetRefCon: SInt32; var container: QTAtomContainer; var atom: QTAtom): ComponentResult; external name '_MediaGetActionsForQTEvent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetActionsForQTEvent( mh: MediaHandler; event: QTEventRecordPtr; targetRefCon: SIGNEDLONG; var container: QTAtomContainer; var atom: QTAtom ): ComponentResult; external name '_MediaGetActionsForQTEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaDisposeTargetRefCon()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaDisposeTargetRefCon( mh: MediaHandler; targetRefCon: SIGNEDLONG ): ComponentResult; external name '_MediaDisposeTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaDisposeTargetRefCon(mh: MediaHandler; targetRefCon: SInt32): ComponentResult; external name '_MediaDisposeTargetRefCon';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaDisposeTargetRefCon( mh: MediaHandler; targetRefCon: SIGNEDLONG ): ComponentResult; external name '_MediaDisposeTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaTargetRefConsEqual()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaTargetRefConsEqual( mh: MediaHandler; firstRefCon: SIGNEDLONG; secondRefCon: SIGNEDLONG; var equal: Boolean ): ComponentResult; external name '_MediaTargetRefConsEqual';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaTargetRefConsEqual(mh: MediaHandler; firstRefCon: SInt32; secondRefCon: SInt32; var equal: boolean): ComponentResult; external name '_MediaTargetRefConsEqual';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaTargetRefConsEqual( mh: MediaHandler; firstRefCon: SIGNEDLONG; secondRefCon: SIGNEDLONG; var equal: Boolean ): ComponentResult; external name '_MediaTargetRefConsEqual';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetActionsCallback()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaSetActionsCallback( mh: MediaHandler; actionsCallbackProc: ActionsUPP; refcon: UnivPtr ): ComponentResult; external name '_MediaSetActionsCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaSetActionsCallback(mh: MediaHandler; actionsCallbackProc: ActionsUPP; refcon: UnivPtr): ComponentResult; external name '_MediaSetActionsCallback';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetActionsCallback( mh: MediaHandler; actionsCallbackProc: ActionsUPP; refcon: UnivPtr ): ComponentResult; external name '_MediaSetActionsCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaPrePrerollBegin()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaPrePrerollBegin( mh: MediaHandler; time: TimeValue; rate: Fixed; completeProc: PrePrerollCompleteUPP; refcon: UnivPtr ): ComponentResult; external name '_MediaPrePrerollBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaPrePrerollBegin(mh: MediaHandler; time: TimeValue; rate: Fixed; completeProc: PrePrerollCompleteUPP; refcon: UnivPtr): ComponentResult; external name '_MediaPrePrerollBegin';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaPrePrerollBegin( mh: MediaHandler; time: TimeValue; rate: Fixed; completeProc: PrePrerollCompleteUPP; refcon: UnivPtr ): ComponentResult; external name '_MediaPrePrerollBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaPrePrerollCancel()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaPrePrerollCancel( mh: MediaHandler; refcon: UnivPtr ): ComponentResult; external name '_MediaPrePrerollCancel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaPrePrerollCancel(mh: MediaHandler; refcon: UnivPtr): ComponentResult; external name '_MediaPrePrerollCancel';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaPrePrerollCancel( mh: MediaHandler; refcon: UnivPtr ): ComponentResult; external name '_MediaPrePrerollCancel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaEnterEmptyEdit()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaEnterEmptyEdit( mh: MediaHandler ): ComponentResult; external name '_MediaEnterEmptyEdit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaEnterEmptyEdit(mh: MediaHandler): ComponentResult; external name '_MediaEnterEmptyEdit';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaEnterEmptyEdit( mh: MediaHandler ): ComponentResult; external name '_MediaEnterEmptyEdit';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaCurrentMediaQueuedData()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function MediaCurrentMediaQueuedData( mh: MediaHandler; var milliSecs: SIGNEDLONG ): ComponentResult; external name '_MediaCurrentMediaQueuedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function MediaCurrentMediaQueuedData(mh: MediaHandler; var milliSecs: SInt32): ComponentResult; external name '_MediaCurrentMediaQueuedData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaCurrentMediaQueuedData( mh: MediaHandler; var milliSecs: SIGNEDLONG ): ComponentResult; external name '_MediaCurrentMediaQueuedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetEffectiveVolume()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaGetEffectiveVolume( mh: MediaHandler; var volume: SInt16 ): ComponentResult; external name '_MediaGetEffectiveVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaGetEffectiveVolume(mh: MediaHandler; var volume: SInt16): ComponentResult; external name '_MediaGetEffectiveVolume';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetEffectiveVolume( mh: MediaHandler; var volume: SInt16 ): ComponentResult; external name '_MediaGetEffectiveVolume';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaResolveTargetRefCon()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaResolveTargetRefCon( mh: MediaHandler; container: QTAtomContainer; atom: QTAtom; var targetRefCon: SIGNEDLONG ): ComponentResult; external name '_MediaResolveTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaResolveTargetRefCon(mh: MediaHandler; container: QTAtomContainer; atom: QTAtom; var targetRefCon: SInt32): ComponentResult; external name '_MediaResolveTargetRefCon';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaResolveTargetRefCon( mh: MediaHandler; container: QTAtomContainer; atom: QTAtom; var targetRefCon: SIGNEDLONG ): ComponentResult; external name '_MediaResolveTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSoundLevelMeteringEnabled()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaGetSoundLevelMeteringEnabled( mh: MediaHandler; var enabled: Boolean ): ComponentResult; external name '_MediaGetSoundLevelMeteringEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaGetSoundLevelMeteringEnabled(mh: MediaHandler; var enabled: boolean): ComponentResult; external name '_MediaGetSoundLevelMeteringEnabled';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSoundLevelMeteringEnabled( mh: MediaHandler; var enabled: Boolean ): ComponentResult; external name '_MediaGetSoundLevelMeteringEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetSoundLevelMeteringEnabled()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaSetSoundLevelMeteringEnabled( mh: MediaHandler; enable: Boolean ): ComponentResult; external name '_MediaSetSoundLevelMeteringEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaSetSoundLevelMeteringEnabled(mh: MediaHandler; enable: boolean): ComponentResult; external name '_MediaSetSoundLevelMeteringEnabled';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetSoundLevelMeteringEnabled( mh: MediaHandler; enable: Boolean ): ComponentResult; external name '_MediaSetSoundLevelMeteringEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSoundLevelMeterInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaGetSoundLevelMeterInfo( mh: MediaHandler; levelInfo: LevelMeterInfoPtr ): ComponentResult; external name '_MediaGetSoundLevelMeterInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaGetSoundLevelMeterInfo(mh: MediaHandler; levelInfo: LevelMeterInfoPtr): ComponentResult; external name '_MediaGetSoundLevelMeterInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSoundLevelMeterInfo( mh: MediaHandler; levelInfo: LevelMeterInfoPtr ): ComponentResult; external name '_MediaGetSoundLevelMeterInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetEffectiveSoundBalance()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaGetEffectiveSoundBalance( mh: MediaHandler; var balance: SInt16 ): ComponentResult; external name '_MediaGetEffectiveSoundBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaGetEffectiveSoundBalance(mh: MediaHandler; var balance: SInt16): ComponentResult; external name '_MediaGetEffectiveSoundBalance';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetEffectiveSoundBalance( mh: MediaHandler; var balance: SInt16 ): ComponentResult; external name '_MediaGetEffectiveSoundBalance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetScreenLock()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaSetScreenLock( mh: MediaHandler; lockIt: Boolean ): ComponentResult; external name '_MediaSetScreenLock';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaSetScreenLock(mh: MediaHandler; lockIt: boolean): ComponentResult; external name '_MediaSetScreenLock';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetScreenLock( mh: MediaHandler; lockIt: Boolean ): ComponentResult; external name '_MediaSetScreenLock';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetDoMCActionCallback()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaSetDoMCActionCallback( mh: MediaHandler; doMCActionCallbackProc: DoMCActionUPP; refcon: UnivPtr ): ComponentResult; external name '_MediaSetDoMCActionCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaSetDoMCActionCallback(mh: MediaHandler; doMCActionCallbackProc: DoMCActionUPP; refcon: UnivPtr): ComponentResult; external name '_MediaSetDoMCActionCallback';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetDoMCActionCallback( mh: MediaHandler; doMCActionCallbackProc: DoMCActionUPP; refcon: UnivPtr ): ComponentResult; external name '_MediaSetDoMCActionCallback';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetErrorString()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaGetErrorString( mh: MediaHandler; theError: ComponentResult; var errorString: Str255 ): ComponentResult; external name '_MediaGetErrorString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaGetErrorString(mh: MediaHandler; theError: ComponentResult; var errorString: Str255): ComponentResult; external name '_MediaGetErrorString';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetErrorString( mh: MediaHandler; theError: ComponentResult; var errorString: Str255 ): ComponentResult; external name '_MediaGetErrorString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSoundEqualizerBands()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaGetSoundEqualizerBands( mh: MediaHandler; spectrumInfo: MediaEQSpectrumBandsRecordPtr ): ComponentResult; external name '_MediaGetSoundEqualizerBands';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaGetSoundEqualizerBands(mh: MediaHandler; spectrumInfo: MediaEQSpectrumBandsRecordPtr): ComponentResult; external name '_MediaGetSoundEqualizerBands';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSoundEqualizerBands( mh: MediaHandler; spectrumInfo: MediaEQSpectrumBandsRecordPtr ): ComponentResult; external name '_MediaGetSoundEqualizerBands';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetSoundEqualizerBands()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaSetSoundEqualizerBands( mh: MediaHandler; spectrumInfo: MediaEQSpectrumBandsRecordPtr ): ComponentResult; external name '_MediaSetSoundEqualizerBands';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaSetSoundEqualizerBands(mh: MediaHandler; spectrumInfo: MediaEQSpectrumBandsRecordPtr): ComponentResult; external name '_MediaSetSoundEqualizerBands';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetSoundEqualizerBands( mh: MediaHandler; spectrumInfo: MediaEQSpectrumBandsRecordPtr ): ComponentResult; external name '_MediaSetSoundEqualizerBands';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSoundEqualizerBandLevels()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaGetSoundEqualizerBandLevels( mh: MediaHandler; var bandLevels: UInt8 ): ComponentResult; external name '_MediaGetSoundEqualizerBandLevels';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaGetSoundEqualizerBandLevels(mh: MediaHandler; var bandLevels: UInt8): ComponentResult; external name '_MediaGetSoundEqualizerBandLevels';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSoundEqualizerBandLevels( mh: MediaHandler; var bandLevels: UInt8 ): ComponentResult; external name '_MediaGetSoundEqualizerBandLevels';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaDoIdleActions()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaDoIdleActions( mh: MediaHandler ): ComponentResult; external name '_MediaDoIdleActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaDoIdleActions(mh: MediaHandler): ComponentResult; external name '_MediaDoIdleActions';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaDoIdleActions( mh: MediaHandler ): ComponentResult; external name '_MediaDoIdleActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetSoundBassAndTreble()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaSetSoundBassAndTreble( mh: MediaHandler; bass: SInt16; treble: SInt16 ): ComponentResult; external name '_MediaSetSoundBassAndTreble';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaSetSoundBassAndTreble(mh: MediaHandler; bass: SInt16; treble: SInt16): ComponentResult; external name '_MediaSetSoundBassAndTreble';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetSoundBassAndTreble( mh: MediaHandler; bass: SInt16; treble: SInt16 ): ComponentResult; external name '_MediaSetSoundBassAndTreble';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetSoundBassAndTreble()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaGetSoundBassAndTreble( mh: MediaHandler; var bass: SInt16; var treble: SInt16 ): ComponentResult; external name '_MediaGetSoundBassAndTreble';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaGetSoundBassAndTreble(mh: MediaHandler; var bass: SInt16; var treble: SInt16): ComponentResult; external name '_MediaGetSoundBassAndTreble';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetSoundBassAndTreble( mh: MediaHandler; var bass: SInt16; var treble: SInt16 ): ComponentResult; external name '_MediaGetSoundBassAndTreble';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaTimeBaseChanged()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function MediaTimeBaseChanged( mh: MediaHandler ): ComponentResult; external name '_MediaTimeBaseChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function MediaTimeBaseChanged(mh: MediaHandler): ComponentResult; external name '_MediaTimeBaseChanged';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaTimeBaseChanged( mh: MediaHandler ): ComponentResult; external name '_MediaTimeBaseChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaMCIsPlayerEvent()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.1 and later
 *    Windows:          in qtmlClient.lib 4.1 and later
 }
function MediaMCIsPlayerEvent( mh: MediaHandler; const (*var*) e: EventRecord; var handledIt: Boolean ): ComponentResult; external name '_MediaMCIsPlayerEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.1 and later
 *    Windows:          in qtmlClient.lib 4.1 and later
 }
<<<<<<< HEAD
function MediaMCIsPlayerEvent(mh: MediaHandler; const (*var*) e: EventRecord; var handledIt: boolean): ComponentResult; external name '_MediaMCIsPlayerEvent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaMCIsPlayerEvent( mh: MediaHandler; const (*var*) e: EventRecord; var handledIt: Boolean ): ComponentResult; external name '_MediaMCIsPlayerEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetMediaLoadState()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.1 and later
 *    Windows:          in qtmlClient.lib 4.1 and later
 }
function MediaGetMediaLoadState( mh: MediaHandler; var mediaLoadState: SIGNEDLONG ): ComponentResult; external name '_MediaGetMediaLoadState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 4.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.1 and later
 *    Windows:          in qtmlClient.lib 4.1 and later
 }
<<<<<<< HEAD
function MediaGetMediaLoadState(mh: MediaHandler; var mediaLoadState: SInt32): ComponentResult; external name '_MediaGetMediaLoadState';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetMediaLoadState( mh: MediaHandler; var mediaLoadState: SIGNEDLONG ): ComponentResult; external name '_MediaGetMediaLoadState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaVideoOutputChanged()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
function MediaVideoOutputChanged( mh: MediaHandler; vout: ComponentInstance ): ComponentResult; external name '_MediaVideoOutputChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
<<<<<<< HEAD
function MediaVideoOutputChanged(mh: MediaHandler; vout: ComponentInstance): ComponentResult; external name '_MediaVideoOutputChanged';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaVideoOutputChanged( mh: MediaHandler; vout: ComponentInstance ): ComponentResult; external name '_MediaVideoOutputChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaEmptySampleCache()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
function MediaEmptySampleCache( mh: MediaHandler; sampleNum: SIGNEDLONG; sampleCount: SIGNEDLONG ): ComponentResult; external name '_MediaEmptySampleCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
<<<<<<< HEAD
function MediaEmptySampleCache(mh: MediaHandler; sampleNum: SInt32; sampleCount: SInt32): ComponentResult; external name '_MediaEmptySampleCache';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaEmptySampleCache( mh: MediaHandler; sampleNum: SIGNEDLONG; sampleCount: SIGNEDLONG ): ComponentResult; external name '_MediaEmptySampleCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetPublicInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
function MediaGetPublicInfo( mh: MediaHandler; infoSelector: OSType; infoDataPtr: UnivPtr; var ioDataSize: Size ): ComponentResult; external name '_MediaGetPublicInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
<<<<<<< HEAD
function MediaGetPublicInfo(mh: MediaHandler; infoSelector: OSType; infoDataPtr: UnivPtr; var ioDataSize: Size): ComponentResult; external name '_MediaGetPublicInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetPublicInfo( mh: MediaHandler; infoSelector: OSType; infoDataPtr: UnivPtr; var ioDataSize: Size ): ComponentResult; external name '_MediaGetPublicInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetPublicInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
function MediaSetPublicInfo( mh: MediaHandler; infoSelector: OSType; infoDataPtr: UnivPtr; dataSize: Size ): ComponentResult; external name '_MediaSetPublicInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
<<<<<<< HEAD
function MediaSetPublicInfo(mh: MediaHandler; infoSelector: OSType; infoDataPtr: UnivPtr; dataSize: Size): ComponentResult; external name '_MediaSetPublicInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetPublicInfo( mh: MediaHandler; infoSelector: OSType; infoDataPtr: UnivPtr; dataSize: Size ): ComponentResult; external name '_MediaSetPublicInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGetUserPreferredCodecs()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
function MediaGetUserPreferredCodecs( mh: MediaHandler; var userPreferredCodecs: CodecComponentHandle ): ComponentResult; external name '_MediaGetUserPreferredCodecs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
<<<<<<< HEAD
function MediaGetUserPreferredCodecs(mh: MediaHandler; var userPreferredCodecs: CodecComponentHandle): ComponentResult; external name '_MediaGetUserPreferredCodecs';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGetUserPreferredCodecs( mh: MediaHandler; var userPreferredCodecs: CodecComponentHandle ): ComponentResult; external name '_MediaGetUserPreferredCodecs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaSetUserPreferredCodecs()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
function MediaSetUserPreferredCodecs( mh: MediaHandler; userPreferredCodecs: CodecComponentHandle ): ComponentResult; external name '_MediaSetUserPreferredCodecs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Keyboard Focus Support}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeLib 5.0 and later
 *    Windows:          in qtmlClient.lib 5.0 and later
 }
<<<<<<< HEAD
function MediaSetUserPreferredCodecs(mh: MediaHandler; userPreferredCodecs: CodecComponentHandle): ComponentResult; external name '_MediaSetUserPreferredCodecs';
{  Keyboard Focus Support }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaSetUserPreferredCodecs( mh: MediaHandler; userPreferredCodecs: CodecComponentHandle ): ComponentResult; external name '_MediaSetUserPreferredCodecs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Keyboard Focus Support}
>>>>>>> origin/fixes_2.4

{
 *  MediaRefConSetProperty()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaRefConSetProperty( mh: MediaHandler; refCon: SIGNEDLONG; propertyType: SIGNEDLONG; propertyValue: UnivPtr ): ComponentResult; external name '_MediaRefConSetProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function MediaRefConSetProperty(mh: MediaHandler; refCon: SInt32; propertyType: SInt32; propertyValue: UnivPtr): ComponentResult; external name '_MediaRefConSetProperty';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaRefConSetProperty( mh: MediaHandler; refCon: SIGNEDLONG; propertyType: SIGNEDLONG; propertyValue: UnivPtr ): ComponentResult; external name '_MediaRefConSetProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaRefConGetProperty()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaRefConGetProperty( mh: MediaHandler; refCon: SIGNEDLONG; propertyType: SIGNEDLONG; propertyValue: UnivPtr ): ComponentResult; external name '_MediaRefConGetProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function MediaRefConGetProperty(mh: MediaHandler; refCon: SInt32; propertyType: SInt32; propertyValue: UnivPtr): ComponentResult; external name '_MediaRefConGetProperty';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaRefConGetProperty( mh: MediaHandler; refCon: SIGNEDLONG; propertyType: SIGNEDLONG; propertyValue: UnivPtr ): ComponentResult; external name '_MediaRefConGetProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaNavigateTargetRefCon()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaNavigateTargetRefCon( mh: MediaHandler; navigation: SIGNEDLONG; var refCon: SIGNEDLONG ): ComponentResult; external name '_MediaNavigateTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function MediaNavigateTargetRefCon(mh: MediaHandler; navigation: SInt32; var refCon: SInt32): ComponentResult; external name '_MediaNavigateTargetRefCon';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaNavigateTargetRefCon( mh: MediaHandler; navigation: SIGNEDLONG; var refCon: SIGNEDLONG ): ComponentResult; external name '_MediaNavigateTargetRefCon';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGGetIdleManager()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaGGetIdleManager( mh: MediaHandler; var pim: IdleManager ): ComponentResult; external name '_MediaGGetIdleManager';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
<<<<<<< HEAD
function MediaGGetIdleManager(mh: MediaHandler; var pim: IdleManager): ComponentResult; external name '_MediaGGetIdleManager';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function MediaGGetIdleManager( mh: MediaHandler; var pim: IdleManager ): ComponentResult; external name '_MediaGGetIdleManager';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  MediaGSetIdleManager()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaGSetIdleManager( mh: MediaHandler; im: IdleManager ): ComponentResult; external name '_MediaGSetIdleManager';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  MediaGGetLatency()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 (or QuickTime 6.4) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 6.5 and later
 }
function MediaGGetLatency( mh: MediaHandler; var latency: TimeRecord ): ComponentResult; external name '_MediaGGetLatency';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ Aperture Mode Support}

{
 *  MediaSetTrackApertureModeDimensionsUsingSampleDescription()
 *  
 *  Summary:
 *    Sets the three aperture mode dimension properties on the track,
 *    calculating the values using the provided sample description.
 *  
 *  Parameters:
 *    
 *    mh:
 *      [in] The media handler.
 *    
 *    sampleDesc:
 *      [in] The sample description handle.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 (or QuickTime 7.1) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function MediaSetTrackApertureModeDimensionsUsingSampleDescription( mh: MediaHandler; sampleDesc: SampleDescriptionHandle ): ComponentResult; external name '_MediaSetTrackApertureModeDimensionsUsingSampleDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  MediaGetApertureModeClipRectForSampleDescriptionIndex()
 *  
 *  Summary:
 *    Calculates a source clip rectangle appropriate for the current
 *    aperture mode and the given sample description.
 *  
 *  Discussion:
 *    If the track's aperture mode is kQTApertureMode_CleanAperture,
 *    the rectangle should be the clean aperture as described by the
 *    sample description (see
 *    kICMImageDescriptionPropertyID_CleanApertureClipRect); otherwise
 *    it should be the full dimensions of the sample description.
 *  
 *  Parameters:
 *    
 *    mh:
 *      [in] The media handler.
 *    
 *    sampleDescIndex:
 *      [in] Indicates the sample description index of sample
 *      description in the media
 *    
 *    clipFixedRectOut:
 *      [out] Points to a variable to receive the clip rectangle.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 (or QuickTime 7.1) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function MediaGetApertureModeClipRectForSampleDescriptionIndex( mh: MediaHandler; sampleDescIndex: SIGNEDLONG; var clipFixedRectOut: FixedRect ): ComponentResult; external name '_MediaGetApertureModeClipRectForSampleDescriptionIndex';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  MediaGetApertureModeMatrixForSampleDescriptionIndex()
 *  
 *  Summary:
 *    Calculates a matrix appropriate for the current aperture mode and
 *    the given sample description.
 *  
 *  Discussion:
 *    If the track's aperture mode is kQTApertureMode_CleanAperture or
 *    kQTApertureMode_ProductionAperture, the matrix should scale
 *    horizontally to compensate for the pixel aspect ratio. Otherwise
 *    the matrix should be identity. If the track's aperture mode is
 *    kQTApertureMode_CleanAperture, the matrix should translate the
 *    top-left point of the clean aperture to the origin. (See
 *    kICMImageDescriptionPropertyID_CleanApertureMatrix and
 *    kICMImageDescriptionPropertyID_ProductionApertureMatrix.)
 *  
 *  Parameters:
 *    
 *    mh:
 *      [in] The media handler.
 *    
 *    sampleDescIndex:
 *      [in] Indicates the sample description index of sample
 *      description in the media
 *    
 *    matrixOut:
 *      [out] Points to a variable to receive the matrix.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 (or QuickTime 7.1) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function MediaGetApertureModeMatrixForSampleDescriptionIndex( mh: MediaHandler; sampleDescIndex: SIGNEDLONG; var matrixOut: MatrixRecord ): ComponentResult; external name '_MediaGetApertureModeMatrixForSampleDescriptionIndex';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  MediaGenerateApertureModeDimensions()
 *  
 *  Summary:
 *    Examines a movie and sets up track aperture mode dimensions.
 *  
 *  Discussion:
 *    If the sample descriptions tracks lack tags describing clean
 *    aperture and pixel aspect ratio information, the media data may
 *    be scanned to see if the correct values can be divined and
 *    attached. Then the aperture mode dimensions should be calculated
 *    and set, as by
 *    MediaSetTrackApertureModeDimensionsUsingSampleDescription.
 *  
 *  Parameters:
 *    
 *    mh:
 *      [in] The media handler.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 (or QuickTime 7.1) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function MediaGenerateApertureModeDimensions( mh: MediaHandler ): ComponentResult; external name '_MediaGenerateApertureModeDimensions';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{ selectors for component calls }
const
	kCallComponentExecuteWiredActionSelect = -9;
	kMediaSetChunkManagementFlagsSelect = $0415;
	kMediaGetChunkManagementFlagsSelect = $0416;
	kMediaSetPurgeableChunkMemoryAllowanceSelect = $0417;
	kMediaGetPurgeableChunkMemoryAllowanceSelect = $0418;
	kMediaEmptyAllPurgeableChunksSelect = $0419;
	kMediaInitializeSelect = $0501;
	kMediaSetHandlerCapabilitiesSelect = $0502;
	kMediaIdleSelect = $0503;
	kMediaGetMediaInfoSelect = $0504;
	kMediaPutMediaInfoSelect = $0505;
	kMediaSetActiveSelect = $0506;
	kMediaSetRateSelect = $0507;
	kMediaGGetStatusSelect = $0508;
	kMediaTrackEditedSelect = $0509;
	kMediaSetMediaTimeScaleSelect = $050A;
	kMediaSetMovieTimeScaleSelect = $050B;
	kMediaSetGWorldSelect = $050C;
	kMediaSetDimensionsSelect = $050D;
	kMediaSetClipSelect = $050E;
	kMediaSetMatrixSelect = $050F;
	kMediaGetTrackOpaqueSelect = $0510;
	kMediaSetGraphicsModeSelect = $0511;
	kMediaGetGraphicsModeSelect = $0512;
	kMediaGSetVolumeSelect = $0513;
	kMediaSetSoundBalanceSelect = $0514;
	kMediaGetSoundBalanceSelect = $0515;
	kMediaGetNextBoundsChangeSelect = $0516;
	kMediaGetSrcRgnSelect = $0517;
	kMediaPrerollSelect = $0518;
	kMediaSampleDescriptionChangedSelect = $0519;
	kMediaHasCharacteristicSelect = $051A;
	kMediaGetOffscreenBufferSizeSelect = $051B;
	kMediaSetHintsSelect = $051C;
	kMediaGetNameSelect = $051D;
	kMediaForceUpdateSelect = $051E;
	kMediaGetDrawingRgnSelect = $051F;
	kMediaGSetActiveSegmentSelect = $0520;
	kMediaInvalidateRegionSelect = $0521;
	kMediaGetNextStepTimeSelect = $0522;
	kMediaSetNonPrimarySourceDataSelect = $0523;
	kMediaChangedNonPrimarySourceSelect = $0524;
	kMediaTrackReferencesChangedSelect = $0525;
	kMediaGetSampleDataPointerSelect = $0526;
	kMediaReleaseSampleDataPointerSelect = $0527;
	kMediaTrackPropertyAtomChangedSelect = $0528;
	kMediaSetTrackInputMapReferenceSelect = $0529;
	kMediaSetVideoParamSelect = $052B;
	kMediaGetVideoParamSelect = $052C;
	kMediaCompareSelect = $052D;
	kMediaGetClockSelect = $052E;
	kMediaSetSoundOutputComponentSelect = $052F;
	kMediaGetSoundOutputComponentSelect = $0530;
	kMediaSetSoundLocalizationDataSelect = $0531;
	kMediaGetInvalidRegionSelect = $053C;
	kMediaSampleDescriptionB2NSelect = $053E;
	kMediaSampleDescriptionN2BSelect = $053F;
	kMediaQueueNonPrimarySourceDataSelect = $0540;
	kMediaFlushNonPrimarySourceDataSelect = $0541;
	kMediaGetURLLinkSelect = $0543;
	kMediaMakeMediaTimeTableSelect = $0545;
	kMediaHitTestForTargetRefConSelect = $0546;
	kMediaHitTestTargetRefConSelect = $0547;
	kMediaGetActionsForQTEventSelect = $0548;
	kMediaDisposeTargetRefConSelect = $0549;
	kMediaTargetRefConsEqualSelect = $054A;
	kMediaSetActionsCallbackSelect = $054B;
	kMediaPrePrerollBeginSelect = $054C;
	kMediaPrePrerollCancelSelect = $054D;
	kMediaEnterEmptyEditSelect = $054F;
	kMediaCurrentMediaQueuedDataSelect = $0550;
	kMediaGetEffectiveVolumeSelect = $0551;
	kMediaResolveTargetRefConSelect = $0552;
	kMediaGetSoundLevelMeteringEnabledSelect = $0553;
	kMediaSetSoundLevelMeteringEnabledSelect = $0554;
	kMediaGetSoundLevelMeterInfoSelect = $0555;
	kMediaGetEffectiveSoundBalanceSelect = $0556;
	kMediaSetScreenLockSelect = $0557;
	kMediaSetDoMCActionCallbackSelect = $0558;
	kMediaGetErrorStringSelect = $0559;
	kMediaGetSoundEqualizerBandsSelect = $055A;
	kMediaSetSoundEqualizerBandsSelect = $055B;
	kMediaGetSoundEqualizerBandLevelsSelect = $055C;
	kMediaDoIdleActionsSelect = $055D;
	kMediaSetSoundBassAndTrebleSelect = $055E;
	kMediaGetSoundBassAndTrebleSelect = $055F;
	kMediaTimeBaseChangedSelect = $0560;
	kMediaMCIsPlayerEventSelect = $0561;
	kMediaGetMediaLoadStateSelect = $0562;
	kMediaVideoOutputChangedSelect = $0563;
	kMediaEmptySampleCacheSelect = $0564;
	kMediaGetPublicInfoSelect = $0565;
	kMediaSetPublicInfoSelect = $0566;
	kMediaGetUserPreferredCodecsSelect = $0567;
	kMediaSetUserPreferredCodecsSelect = $0568;
	kMediaRefConSetPropertySelect = $0569;
	kMediaRefConGetPropertySelect = $056A;
	kMediaNavigateTargetRefConSelect = $056B;
	kMediaGGetIdleManagerSelect = $056C;
	kMediaGSetIdleManagerSelect = $056D;
	kMediaGGetLatencySelect = $0571;
	kMediaSetTrackApertureModeDimensionsUsingSampleDescriptionSelect = $0579;
	kMediaGetApertureModeClipRectForSampleDescriptionIndexSelect = $057A;
	kMediaGetApertureModeMatrixForSampleDescriptionIndexSelect = $057B;
	kMediaGenerateApertureModeDimensionsSelect = $057C;
{
 *  NewPrePrerollCompleteUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewPrePrerollCompleteUPP( userRoutine: PrePrerollCompleteProcPtr ): PrePrerollCompleteUPP; external name '_NewPrePrerollCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
=======
 *    Mac OS X:         in version 10.2 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   in QuickTimeLib 6.0 and later
 *    Windows:          in qtmlClient.lib 6.0 and later
 }
function MediaGSetIdleManager( mh: MediaHandler; im: IdleManager ): ComponentResult; external name '_MediaGSetIdleManager';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  MediaGGetLatency()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 (or QuickTime 6.4) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 6.5 and later
 }
function MediaGGetLatency( mh: MediaHandler; var latency: TimeRecord ): ComponentResult; external name '_MediaGGetLatency';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{ Aperture Mode Support}

{
 *  MediaSetTrackApertureModeDimensionsUsingSampleDescription()
 *  
 *  Summary:
 *    Sets the three aperture mode dimension properties on the track,
 *    calculating the values using the provided sample description.
 *  
 *  Parameters:
 *    
 *    mh:
 *      [in] The media handler.
 *    
 *    sampleDesc:
 *      [in] The sample description handle.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 (or QuickTime 7.1) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function MediaSetTrackApertureModeDimensionsUsingSampleDescription( mh: MediaHandler; sampleDesc: SampleDescriptionHandle ): ComponentResult; external name '_MediaSetTrackApertureModeDimensionsUsingSampleDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  MediaGetApertureModeClipRectForSampleDescriptionIndex()
 *  
 *  Summary:
 *    Calculates a source clip rectangle appropriate for the current
 *    aperture mode and the given sample description.
 *  
 *  Discussion:
 *    If the track's aperture mode is kQTApertureMode_CleanAperture,
 *    the rectangle should be the clean aperture as described by the
 *    sample description (see
 *    kICMImageDescriptionPropertyID_CleanApertureClipRect); otherwise
 *    it should be the full dimensions of the sample description.
 *  
 *  Parameters:
 *    
 *    mh:
 *      [in] The media handler.
 *    
 *    sampleDescIndex:
 *      [in] Indicates the sample description index of sample
 *      description in the media
 *    
 *    clipFixedRectOut:
 *      [out] Points to a variable to receive the clip rectangle.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 (or QuickTime 7.1) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function MediaGetApertureModeClipRectForSampleDescriptionIndex( mh: MediaHandler; sampleDescIndex: SIGNEDLONG; var clipFixedRectOut: FixedRect ): ComponentResult; external name '_MediaGetApertureModeClipRectForSampleDescriptionIndex';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  MediaGetApertureModeMatrixForSampleDescriptionIndex()
 *  
 *  Summary:
 *    Calculates a matrix appropriate for the current aperture mode and
 *    the given sample description.
 *  
 *  Discussion:
 *    If the track's aperture mode is kQTApertureMode_CleanAperture or
 *    kQTApertureMode_ProductionAperture, the matrix should scale
 *    horizontally to compensate for the pixel aspect ratio. Otherwise
 *    the matrix should be identity. If the track's aperture mode is
 *    kQTApertureMode_CleanAperture, the matrix should translate the
 *    top-left point of the clean aperture to the origin. (See
 *    kICMImageDescriptionPropertyID_CleanApertureMatrix and
 *    kICMImageDescriptionPropertyID_ProductionApertureMatrix.)
 *  
 *  Parameters:
 *    
 *    mh:
 *      [in] The media handler.
 *    
 *    sampleDescIndex:
 *      [in] Indicates the sample description index of sample
 *      description in the media
 *    
 *    matrixOut:
 *      [out] Points to a variable to receive the matrix.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 (or QuickTime 7.1) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function MediaGetApertureModeMatrixForSampleDescriptionIndex( mh: MediaHandler; sampleDescIndex: SIGNEDLONG; var matrixOut: MatrixRecord ): ComponentResult; external name '_MediaGetApertureModeMatrixForSampleDescriptionIndex';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  MediaGenerateApertureModeDimensions()
 *  
 *  Summary:
 *    Examines a movie and sets up track aperture mode dimensions.
 *  
 *  Discussion:
 *    If the sample descriptions tracks lack tags describing clean
 *    aperture and pixel aspect ratio information, the media data may
 *    be scanned to see if the correct values can be divined and
 *    attached. Then the aperture mode dimensions should be calculated
 *    and set, as by
 *    MediaSetTrackApertureModeDimensionsUsingSampleDescription.
 *  
 *  Parameters:
 *    
 *    mh:
 *      [in] The media handler.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 (or QuickTime 7.1) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function MediaGenerateApertureModeDimensions( mh: MediaHandler ): ComponentResult; external name '_MediaGenerateApertureModeDimensions';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{ selectors for component calls }
const
<<<<<<< HEAD
	uppPrePrerollCompleteProcInfo = $00000EC0;
{$ifc CALL_NOT_IN_CARBON}
	{
	 *  NewPrePrerollCompleteUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        not available
	 *    Mac OS X:         not available
	 	}
function NewPrePrerollCompleteUPP(userRoutine: PrePrerollCompleteProcPtr): PrePrerollCompleteUPP; external name '_NewPrePrerollCompleteUPP'; { old name was NewPrePrerollCompleteProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kCallComponentExecuteWiredActionSelect = -9;
	kMediaSetChunkManagementFlagsSelect = $0415;
	kMediaGetChunkManagementFlagsSelect = $0416;
	kMediaSetPurgeableChunkMemoryAllowanceSelect = $0417;
	kMediaGetPurgeableChunkMemoryAllowanceSelect = $0418;
	kMediaEmptyAllPurgeableChunksSelect = $0419;
	kMediaInitializeSelect = $0501;
	kMediaSetHandlerCapabilitiesSelect = $0502;
	kMediaIdleSelect = $0503;
	kMediaGetMediaInfoSelect = $0504;
	kMediaPutMediaInfoSelect = $0505;
	kMediaSetActiveSelect = $0506;
	kMediaSetRateSelect = $0507;
	kMediaGGetStatusSelect = $0508;
	kMediaTrackEditedSelect = $0509;
	kMediaSetMediaTimeScaleSelect = $050A;
	kMediaSetMovieTimeScaleSelect = $050B;
	kMediaSetGWorldSelect = $050C;
	kMediaSetDimensionsSelect = $050D;
	kMediaSetClipSelect = $050E;
	kMediaSetMatrixSelect = $050F;
	kMediaGetTrackOpaqueSelect = $0510;
	kMediaSetGraphicsModeSelect = $0511;
	kMediaGetGraphicsModeSelect = $0512;
	kMediaGSetVolumeSelect = $0513;
	kMediaSetSoundBalanceSelect = $0514;
	kMediaGetSoundBalanceSelect = $0515;
	kMediaGetNextBoundsChangeSelect = $0516;
	kMediaGetSrcRgnSelect = $0517;
	kMediaPrerollSelect = $0518;
	kMediaSampleDescriptionChangedSelect = $0519;
	kMediaHasCharacteristicSelect = $051A;
	kMediaGetOffscreenBufferSizeSelect = $051B;
	kMediaSetHintsSelect = $051C;
	kMediaGetNameSelect = $051D;
	kMediaForceUpdateSelect = $051E;
	kMediaGetDrawingRgnSelect = $051F;
	kMediaGSetActiveSegmentSelect = $0520;
	kMediaInvalidateRegionSelect = $0521;
	kMediaGetNextStepTimeSelect = $0522;
	kMediaSetNonPrimarySourceDataSelect = $0523;
	kMediaChangedNonPrimarySourceSelect = $0524;
	kMediaTrackReferencesChangedSelect = $0525;
	kMediaGetSampleDataPointerSelect = $0526;
	kMediaReleaseSampleDataPointerSelect = $0527;
	kMediaTrackPropertyAtomChangedSelect = $0528;
	kMediaSetTrackInputMapReferenceSelect = $0529;
	kMediaSetVideoParamSelect = $052B;
	kMediaGetVideoParamSelect = $052C;
	kMediaCompareSelect = $052D;
	kMediaGetClockSelect = $052E;
	kMediaSetSoundOutputComponentSelect = $052F;
	kMediaGetSoundOutputComponentSelect = $0530;
	kMediaSetSoundLocalizationDataSelect = $0531;
	kMediaGetInvalidRegionSelect = $053C;
	kMediaSampleDescriptionB2NSelect = $053E;
	kMediaSampleDescriptionN2BSelect = $053F;
	kMediaQueueNonPrimarySourceDataSelect = $0540;
	kMediaFlushNonPrimarySourceDataSelect = $0541;
	kMediaGetURLLinkSelect = $0543;
	kMediaMakeMediaTimeTableSelect = $0545;
	kMediaHitTestForTargetRefConSelect = $0546;
	kMediaHitTestTargetRefConSelect = $0547;
	kMediaGetActionsForQTEventSelect = $0548;
	kMediaDisposeTargetRefConSelect = $0549;
	kMediaTargetRefConsEqualSelect = $054A;
	kMediaSetActionsCallbackSelect = $054B;
	kMediaPrePrerollBeginSelect = $054C;
	kMediaPrePrerollCancelSelect = $054D;
	kMediaEnterEmptyEditSelect = $054F;
	kMediaCurrentMediaQueuedDataSelect = $0550;
	kMediaGetEffectiveVolumeSelect = $0551;
	kMediaResolveTargetRefConSelect = $0552;
	kMediaGetSoundLevelMeteringEnabledSelect = $0553;
	kMediaSetSoundLevelMeteringEnabledSelect = $0554;
	kMediaGetSoundLevelMeterInfoSelect = $0555;
	kMediaGetEffectiveSoundBalanceSelect = $0556;
	kMediaSetScreenLockSelect = $0557;
	kMediaSetDoMCActionCallbackSelect = $0558;
	kMediaGetErrorStringSelect = $0559;
	kMediaGetSoundEqualizerBandsSelect = $055A;
	kMediaSetSoundEqualizerBandsSelect = $055B;
	kMediaGetSoundEqualizerBandLevelsSelect = $055C;
	kMediaDoIdleActionsSelect = $055D;
	kMediaSetSoundBassAndTrebleSelect = $055E;
	kMediaGetSoundBassAndTrebleSelect = $055F;
	kMediaTimeBaseChangedSelect = $0560;
	kMediaMCIsPlayerEventSelect = $0561;
	kMediaGetMediaLoadStateSelect = $0562;
	kMediaVideoOutputChangedSelect = $0563;
	kMediaEmptySampleCacheSelect = $0564;
	kMediaGetPublicInfoSelect = $0565;
	kMediaSetPublicInfoSelect = $0566;
	kMediaGetUserPreferredCodecsSelect = $0567;
	kMediaSetUserPreferredCodecsSelect = $0568;
	kMediaRefConSetPropertySelect = $0569;
	kMediaRefConGetPropertySelect = $056A;
	kMediaNavigateTargetRefConSelect = $056B;
	kMediaGGetIdleManagerSelect = $056C;
	kMediaGSetIdleManagerSelect = $056D;
	kMediaGGetLatencySelect = $0571;
	kMediaSetTrackApertureModeDimensionsUsingSampleDescriptionSelect = $0579;
	kMediaGetApertureModeClipRectForSampleDescriptionIndexSelect = $057A;
	kMediaGetApertureModeMatrixForSampleDescriptionIndexSelect = $057B;
	kMediaGenerateApertureModeDimensionsSelect = $057C;
{
 *  NewPrePrerollCompleteUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewPrePrerollCompleteUPP( userRoutine: PrePrerollCompleteProcPtr ): PrePrerollCompleteUPP; external name '_NewPrePrerollCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposePrePrerollCompleteUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposePrePrerollCompleteUPP( userUPP: PrePrerollCompleteUPP ); external name '_DisposePrePrerollCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposePrePrerollCompleteUPP(userUPP: PrePrerollCompleteUPP); external name '_DisposePrePrerollCompleteUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposePrePrerollCompleteUPP( userUPP: PrePrerollCompleteUPP ); external name '_DisposePrePrerollCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokePrePrerollCompleteUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokePrePrerollCompleteUPP( mh: MediaHandler; err: OSErr; refcon: UnivPtr; userUPP: PrePrerollCompleteUPP ); external name '_InvokePrePrerollCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokePrePrerollCompleteUPP( mh: MediaHandler; err: OSErr; refcon: UnivPtr; userUPP: PrePrerollCompleteUPP ); external name '_InvokePrePrerollCompleteUPP';
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
