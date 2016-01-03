{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/ImageCodec.h
 
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
     File:       ImageCodec.p
=======
     File:       QuickTime/ImageCodec.h
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
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
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
=======
>>>>>>> origin/fixes_2_2
                     http://www.freepascal.org/bugs.html
 
}
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit ImageCodec;
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
uses MacTypes,Components,Dialogs,Events,ImageCompression,Movies,OSUtils,QuickdrawTypes,QDOffscreen,CFDictionary;
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

{
   The following GX types were previously in GXTypes.h, but that header
   is not available in any Mac OS X framework. 
}
type
	gxPoint = record
		x: Fixed;
		y: Fixed;
	end;
type
	gxPathPtr = ^gxPath;
	gxPath = record
		vectors: SIGNEDLONG;
		controlBits: array [0..0] of SIGNEDLONG;
		vector: array [0..0] of gxPoint;
	end;
type
	gxPathsPtr = ^gxPaths;
	gxPaths = record
		contours: SIGNEDLONG;
    contour: array [0..0] of gxPath;
	end;
{  codec capabilities flags    }
const
	codecCanScale = 1 shl 0;
	codecCanMask = 1 shl 1;
	codecCanMatte = 1 shl 2;
	codecCanTransform = 1 shl 3;
	codecCanTransferMode = 1 shl 4;
	codecCanCopyPrev = 1 shl 5;
	codecCanSpool = 1 shl 6;
	codecCanClipVertical = 1 shl 7;
	codecCanClipRectangular = 1 shl 8;
	codecCanRemapColor = 1 shl 9;
	codecCanFastDither = 1 shl 10;
	codecCanSrcExtract = 1 shl 11;
	codecCanCopyPrevComp = 1 shl 12;
	codecCanAsync = 1 shl 13;
	codecCanMakeMask = 1 shl 14;
	codecCanShift = 1 shl 15;
	codecCanAsyncWhen = 1 shl 16;
	codecCanShieldCursor = 1 shl 17;
	codecCanManagePrevBuffer = 1 shl 18;
	codecHasVolatileBuffer = 1 shl 19; { codec requires redraw after window movement }
	codecWantsRegionMask = 1 shl 20;
	codecImageBufferIsOnScreen = 1 shl 21; { old def of codec using overlay surface, = ( codecIsDirectToScreenOnly | codecUsesOverlaySurface | codecImageBufferIsOverlaySurface | codecSrcMustBeImageBuffer ) }
	codecWantsDestinationPixels = 1 shl 22;
	codecWantsSpecialScaling = 1 shl 23;
	codecHandlesInputs = 1 shl 24;
	codecCanDoIndirectSurface = 1 shl 25; { codec can handle indirect surface (GDI) }
	codecIsSequenceSensitive = 1 shl 26;
	codecRequiresOffscreen = 1 shl 27;
	codecRequiresMaskBits = 1 shl 28;
	codecCanRemapResolution = 1 shl 29;
	codecIsDirectToScreenOnly = 1 shl 30; { codec can only decompress data to the screen }
	codecCanLockSurface = 1 shl 31; { codec can lock destination surface, icm doesn't lock for you }

{  codec capabilities flags2   }
const
	codecUsesOverlaySurface = 1 shl 0; { codec uses overlay surface }
	codecImageBufferIsOverlaySurface = 1 shl 1; { codec image buffer is overlay surface, the bits in the buffer are on the screen }
	codecSrcMustBeImageBuffer = 1 shl 2; { codec can only source data from an image buffer }
	codecImageBufferIsInAGPMemory = 1 shl 4; { codec image buffer is in AGP space, byte writes are OK }
	codecImageBufferIsInPCIMemory = 1 shl 5; { codec image buffer is across a PCI bus; byte writes are bad }
	codecImageBufferMemoryFlagsValid = 1 shl 6; { set by ImageCodecNewImageBufferMemory/NewImageGWorld to indicate that it set the AGP/PCI flags (supported in QuickTime 6.0 and later) }
	codecDrawsHigherQualityScaled = 1 shl 7; { codec will draw higher-quality image if it performs scaling (eg, wipe effect with border) }
	codecSupportsOutOfOrderDisplayTimes = 1 shl 8; { codec supports frames queued in one order for display in a different order, eg, IPB content }
	codecSupportsScheduledBackwardsPlaybackWithDifferenceFrames = 1 shl 9; { codec can use additional buffers to minimise redecoding during backwards playback }
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Events,QDOffscreen,OSUtils,Dialogs,Quickdraw,Components,GXTypes,ImageCompression,Movies;
=======
uses MacTypes,Components,Dialogs,Events,ImageCompression,Movies,OSUtils,QuickdrawTypes,QDOffscreen,CFDictionary;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

{
   The following GX types were previously in GXTypes.h, but that header
   is not available in any Mac OS X framework. 
}
type
	gxPoint = record
		x: Fixed;
		y: Fixed;
	end;
type
	gxPathPtr = ^gxPath;
	gxPath = record
		vectors: SIGNEDLONG;
		controlBits: array [0..0] of SIGNEDLONG;
		vector: array [0..0] of gxPoint;
	end;
type
	gxPathsPtr = ^gxPaths;
	gxPaths = record
		contours: SIGNEDLONG;
    contour: array [0..0] of gxPath;
	end;
{  codec capabilities flags    }
const
<<<<<<< HEAD
	codecCanScale				= $00000001;
	codecCanMask				= $00000002;
	codecCanMatte				= $00000004;
	codecCanTransform			= $00000008;
	codecCanTransferMode		= $00000010;
	codecCanCopyPrev			= $00000020;
	codecCanSpool				= $00000040;
	codecCanClipVertical		= $00000080;
	codecCanClipRectangular		= $00000100;
	codecCanRemapColor			= $00000200;
	codecCanFastDither			= $00000400;
	codecCanSrcExtract			= $00000800;
	codecCanCopyPrevComp		= $00001000;
	codecCanAsync				= $00002000;
	codecCanMakeMask			= $00004000;
	codecCanShift				= $00008000;
	codecCanAsyncWhen			= $00010000;
	codecCanShieldCursor		= $00020000;
	codecCanManagePrevBuffer	= $00040000;
	codecHasVolatileBuffer		= $00080000;					{  codec requires redraw after window movement  }
	codecWantsRegionMask		= $00100000;
	codecImageBufferIsOnScreen	= $00200000;					{  old def of codec using overlay surface, = ( codecIsDirectToScreenOnly | codecUsesOverlaySurface | codecImageBufferIsOverlaySurface | codecSrcMustBeImageBuffer )  }
	codecWantsDestinationPixels	= $00400000;
	codecWantsSpecialScaling	= $00800000;
	codecHandlesInputs			= $01000000;
	codecCanDoIndirectSurface	= $02000000;					{  codec can handle indirect surface (GDI)  }
	codecIsSequenceSensitive	= $04000000;
	codecRequiresOffscreen		= $08000000;
	codecRequiresMaskBits		= $10000000;
	codecCanRemapResolution		= $20000000;
	codecIsDirectToScreenOnly	= $40000000;					{  codec can only decompress data to the screen  }
	codecCanLockSurface			= $80000000;					{  codec can lock destination surface, icm doesn't lock for you  }

	{	  codec capabilities flags2   	}
	codecUsesOverlaySurface		= $00000001;					{  codec uses overlay surface  }
	codecImageBufferIsOverlaySurface = $00000002;				{  codec image buffer is overlay surface, the bits in the buffer are on the screen  }
	codecSrcMustBeImageBuffer	= $00000004;					{  codec can only source data from an image buffer  }
	codecImageBufferIsInAGPMemory = $00000010;					{  codec image buffer is in AGP space, byte writes are OK  }
	codecImageBufferIsInPCIMemory = $00000020;					{  codec image buffer is across a PCI bus; byte writes are bad  }
	codecImageBufferMemoryFlagsValid = $00000040;				{  set by ImageCodecNewImageBufferMemory/NewImageGWorld to indicate that it set the AGP/PCI flags (supported in QuickTime 6.0 and later)  }
	codecDrawsHigherQualityScaled = $00000080;					{  codec will draw higher-quality image if it performs scaling (eg, wipe effect with border)  }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	codecCanScale = 1 shl 0;
	codecCanMask = 1 shl 1;
	codecCanMatte = 1 shl 2;
	codecCanTransform = 1 shl 3;
	codecCanTransferMode = 1 shl 4;
	codecCanCopyPrev = 1 shl 5;
	codecCanSpool = 1 shl 6;
	codecCanClipVertical = 1 shl 7;
	codecCanClipRectangular = 1 shl 8;
	codecCanRemapColor = 1 shl 9;
	codecCanFastDither = 1 shl 10;
	codecCanSrcExtract = 1 shl 11;
	codecCanCopyPrevComp = 1 shl 12;
	codecCanAsync = 1 shl 13;
	codecCanMakeMask = 1 shl 14;
	codecCanShift = 1 shl 15;
	codecCanAsyncWhen = 1 shl 16;
	codecCanShieldCursor = 1 shl 17;
	codecCanManagePrevBuffer = 1 shl 18;
	codecHasVolatileBuffer = 1 shl 19; { codec requires redraw after window movement }
	codecWantsRegionMask = 1 shl 20;
	codecImageBufferIsOnScreen = 1 shl 21; { old def of codec using overlay surface, = ( codecIsDirectToScreenOnly | codecUsesOverlaySurface | codecImageBufferIsOverlaySurface | codecSrcMustBeImageBuffer ) }
	codecWantsDestinationPixels = 1 shl 22;
	codecWantsSpecialScaling = 1 shl 23;
	codecHandlesInputs = 1 shl 24;
	codecCanDoIndirectSurface = 1 shl 25; { codec can handle indirect surface (GDI) }
	codecIsSequenceSensitive = 1 shl 26;
	codecRequiresOffscreen = 1 shl 27;
	codecRequiresMaskBits = 1 shl 28;
	codecCanRemapResolution = 1 shl 29;
	codecIsDirectToScreenOnly = 1 shl 30; { codec can only decompress data to the screen }
	codecCanLockSurface = 1 shl 31; { codec can lock destination surface, icm doesn't lock for you }

{  codec capabilities flags2   }
const
	codecUsesOverlaySurface = 1 shl 0; { codec uses overlay surface }
	codecImageBufferIsOverlaySurface = 1 shl 1; { codec image buffer is overlay surface, the bits in the buffer are on the screen }
	codecSrcMustBeImageBuffer = 1 shl 2; { codec can only source data from an image buffer }
	codecImageBufferIsInAGPMemory = 1 shl 4; { codec image buffer is in AGP space, byte writes are OK }
	codecImageBufferIsInPCIMemory = 1 shl 5; { codec image buffer is across a PCI bus; byte writes are bad }
	codecImageBufferMemoryFlagsValid = 1 shl 6; { set by ImageCodecNewImageBufferMemory/NewImageGWorld to indicate that it set the AGP/PCI flags (supported in QuickTime 6.0 and later) }
	codecDrawsHigherQualityScaled = 1 shl 7; { codec will draw higher-quality image if it performs scaling (eg, wipe effect with border) }
	codecSupportsOutOfOrderDisplayTimes = 1 shl 8; { codec supports frames queued in one order for display in a different order, eg, IPB content }
	codecSupportsScheduledBackwardsPlaybackWithDifferenceFrames = 1 shl 9; { codec can use additional buffers to minimise redecoding during backwards playback }
>>>>>>> origin/fixes_2.4

type
	CodecCapabilitiesPtr = ^CodecCapabilities;
	CodecCapabilities = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		flags: SIGNEDLONG;
		wantedPixelSize: SInt16;
		extendWidth: SInt16;
		extendHeight: SInt16;
		bandMin: SInt16;
		bandInc: SInt16;
		pad: SInt16;
		time: UNSIGNEDLONG;
		flags2: SIGNEDLONG;                 { field new in QuickTime 4.0 }
<<<<<<< HEAD
	end;
{  codec condition flags   }
const
	codecConditionFirstBand = 1 shl 0;
	codecConditionLastBand = 1 shl 1;
	codecConditionFirstFrame = 1 shl 2;
	codecConditionNewDepth = 1 shl 3;
	codecConditionNewTransform = 1 shl 4;
	codecConditionNewSrcRect = 1 shl 5;
	codecConditionNewMask = 1 shl 6;
	codecConditionNewMatte = 1 shl 7;
	codecConditionNewTransferMode = 1 shl 8;
	codecConditionNewClut = 1 shl 9;
	codecConditionNewAccuracy = 1 shl 10;
	codecConditionNewDestination = 1 shl 11;
	codecConditionFirstScreen = 1 shl 12;
	codecConditionDoCursor = 1 shl 13;
	codecConditionCatchUpDiff = 1 shl 14;
	codecConditionMaskMayBeChanged = 1 shl 15;
	codecConditionToBuffer = 1 shl 16;
	codecConditionCodecChangedMask = 1 shl 31;


const
	codecInfoResourceType = FourCharCode('cdci'); { codec info resource type }
	codecInterfaceVersion = 2;     { high word returned in component GetVersion }

type
	CDSequenceDataSourceQueueEntry = record
		nextBusy: UnivPtr;

		descSeed: SIGNEDLONG;
		dataDesc: Handle;
		data: UnivPtr;
		dataSize: SIGNEDLONG;

		useCount: SIGNEDLONG;

		frameTime: TimeValue;
		frameDuration: TimeValue;
		timeScale: TimeValue;
	end;
	CDSequenceDataSourceQueueEntryPtr = ^CDSequenceDataSourceQueueEntry;
type
	CDSequenceDataSource = record
		recordSize: SIGNEDLONG;

		next: UnivPtr;

		seqID: ImageSequence;
		sourceID: ImageSequenceDataSource;
		sourceType: OSType;
		sourceInputNumber: SIGNEDLONG;
		dataPtr: UnivPtr;
		dataDescription: Handle;
		changeSeed: SIGNEDLONG;
		transferProc: ICMConvertDataFormatUPP;
		transferRefcon: UnivPtr;
		dataSize: SIGNEDLONG;

                                              { fields available in QT 3 and later }

		dataQueue: QHdrPtr;              { queue of CDSequenceDataSourceQueueEntry structures}

		originalDataPtr: UnivPtr;
		originalDataSize: SIGNEDLONG;
		originalDataDescription: Handle;
		originalDataDescriptionSeed: SIGNEDLONG;
	end;
	CDSequenceDataSourcePtr = ^CDSequenceDataSource;
type
	ICMFrameTimeInfo = record
		startTime: wide;
		scale: SIGNEDLONG;
		duration: SIGNEDLONG;
	end;
	ICMFrameTimeInfoPtr = ^ICMFrameTimeInfo;
type
	CodecCompressParams = record
		sequenceID: ImageSequence;             { precompress,bandcompress }
		imageDescription: ImageDescriptionHandle;   { precompress,bandcompress }
		data: Ptr;
		bufferSize: SIGNEDLONG;
		frameNumber: SIGNEDLONG;
		startLine: SIGNEDLONG;
		stopLine: SIGNEDLONG;
		conditionFlags: SIGNEDLONG;
		callerFlags: CodecFlags;
		capabilities: CodecCapabilitiesPtr;           { precompress,bandcompress }
		progressProcRecord: ICMProgressProcRecord;
		completionProcRecord: ICMCompletionProcRecord;
		flushProcRecord: ICMFlushProcRecord;

		srcPixMap: PixMap;              { precompress,bandcompress }
		prevPixMap: PixMap;
		spatialQuality: CodecQ;
		temporalQuality: CodecQ;
		similarity: Fixed;
		dataRateParams: DataRateParamsPtr;
		reserved: SIGNEDLONG;

                                              { The following fields only exist for QuickTime 2.1 and greater }
		majorSourceChangeSeed: UInt16;
		minorSourceChangeSeed: UInt16;
		sourceData: CDSequenceDataSourcePtr;

                                              { The following fields only exist for QuickTime 2.5 and greater }
		preferredPacketSizeInBytes: SIGNEDLONG;

                                              { The following fields only exist for QuickTime 3.0 and greater }
		requestedBufferWidth: SIGNEDLONG;   { must set codecWantsSpecialScaling to indicate this field is valid}
		requestedBufferHeight: SIGNEDLONG;  { must set codecWantsSpecialScaling to indicate this field is valid}

                                              { The following fields only exist for QuickTime 4.0 and greater }
		wantedSourcePixelType: OSType;

                                              { The following fields only exist for QuickTime 5.0 and greater }
		compressedDataSize: SIGNEDLONG;     { if nonzero, this overrides (*imageDescription)->dataSize}
		taskWeight: UInt32;             { preferred weight for MP tasks implementing this operation}
		taskName: OSType;               { preferred name (type) for MP tasks implementing this operation}
	end;
type
	CodecDecompressParamsPtr = ^CodecDecompressParams;
	CodecDecompressParams = record
		sequenceID: ImageSequence;             { predecompress,banddecompress }
		imageDescription: ImageDescriptionHandle;   { predecompress,banddecompress }
		data: Ptr;
		bufferSize: SIGNEDLONG;
		frameNumber: SIGNEDLONG;
		startLine: SIGNEDLONG;
		stopLine: SIGNEDLONG;
		conditionFlags: SIGNEDLONG;
		callerFlags: CodecFlags;
		capabilities: CodecCapabilitiesPtr;           { predecompress,banddecompress }
		progressProcRecord: ICMProgressProcRecord;
		completionProcRecord: ICMCompletionProcRecord;
		dataProcRecord: ICMDataProcRecord;

		port: CGrafPtr;                   { predecompress,banddecompress }
		dstPixMap: PixMap;              { predecompress,banddecompress }
		maskBits: BitMapPtr;
		mattePixMap: PixMapPtr;
		srcRect: Rect;                { predecompress,banddecompress }
		matrix: MatrixRecordPtr;                 { predecompress,banddecompress }
		accuracy: CodecQ;               { predecompress,banddecompress }
		transferMode: SInt16;           { predecompress,banddecompress }
		frameTime: ICMFrameTimePtr;              { banddecompress }
    reserved: array [0..0] of SIGNEDLONG;

                                              { The following fields only exist for QuickTime 2.0 and greater }
		matrixFlags: SInt8;            { high bit set if 2x resize }
		matrixType: SInt8;
		dstRect: Rect;                { only valid for simple transforms }

                                              { The following fields only exist for QuickTime 2.1 and greater }
		majorSourceChangeSeed: UInt16;
		minorSourceChangeSeed: UInt16;
		sourceData: CDSequenceDataSourcePtr;

		maskRegion: RgnHandle;

                                              { The following fields only exist for QuickTime 2.5 and greater }
		wantedDestinationPixelTypes: OSTypeHandle; { Handle to 0-terminated list of OSTypes }

		screenFloodMethod: SIGNEDLONG;
		screenFloodValue: SIGNEDLONG;
		preferredOffscreenPixelSize: SInt16;

                                              { The following fields only exist for QuickTime 3.0 and greater }
		syncFrameTime: ICMFrameTimeInfoPtr;         { banddecompress }
		needUpdateOnTimeChange: Boolean; { banddecompress }
		enableBlackLining: Boolean;
		needUpdateOnSourceChange: Boolean; { band decompress }
		pad: Boolean;

		unused: SIGNEDLONG;

		finalDestinationPort: CGrafPtr;

		requestedBufferWidth: SIGNEDLONG;   { must set codecWantsSpecialScaling to indicate this field is valid}
		requestedBufferHeight: SIGNEDLONG;  { must set codecWantsSpecialScaling to indicate this field is valid}

                                              { The following fields only exist for QuickTime 4.0 and greater }
		displayableAreaOfRequestedBuffer: Rect; { set in predecompress}
		requestedSingleField: Boolean;
		needUpdateOnNextIdle: Boolean;
		pad2: array [0..1] of Boolean;
		bufferGammaLevel: Fixed;

                                              { The following fields only exist for QuickTime 5.0 and greater }
		taskWeight: UInt32;             { preferred weight for MP tasks implementing this operation}
		taskName: OSType;               { preferred name (type) for MP tasks implementing this operation}

                                              { The following fields only exist for QuickTime 6.0 and greater }
		pad3: Boolean;
		destinationBufferMemoryPreference: UInt8; { a codec's PreDecompress/Preflight call can set this to express a preference about what kind of memory its destination buffer should go into.  no guarantees.}
		codecBufferMemoryPreference: UInt8; { may indicate preferred kind of memory that NewImageGWorld/NewImageBufferMemory should create its buffer in, if applicable.}
		onlyUseCodecIfItIsInUserPreferredCodecList: Boolean; { set to prevent this codec from being used unless it is in the userPreferredCodec list}

		mediaContextID: QTMediaContextID;

                                              { The following fields only exist for QuickTime 6.5 and greater }
		deinterlaceRequest: UInt8;     { set by the ICM before PreDecompress/Preflight }
		deinterlaceAnswer: UInt8;      { codec should set this in PreDecompress/Preflight if it will satisfy the deinterlaceRequest }

                                              { The following fields only exist for QuickTime 7.0 and greater }
		pad4: array[0..1] of UInt8;
		reserved2: SIGNEDLONG;
		reserved3: UInt32;
		reserved4: SIGNEDLONG;
		reserved5: UnivPtr;
		reserved6: UnivPtr;
		reserved7: UnivPtr;
		reserved8: UnivPtr;
	end;
const
	matrixFlagScale2x = 1 shl 7;
	matrixFlagScale1x = 1 shl 6;
	matrixFlagScaleHalf = 1 shl 5;

const
	kScreenFloodMethodNone = 0;
	kScreenFloodMethodKeyColor = 1;
	kScreenFloodMethodAlpha = 2;

const
	kFlushLastQueuedFrame = 0;
	kFlushFirstQueuedFrame = 1;

const
	kNewImageGWorldErase = 1 shl 0;

{ values for destinationBufferMemoryPreference and codecBufferMemoryPreference }
const
	kICMImageBufferNoPreference = 0;
	kICMImageBufferPreferMainMemory = 1;
	kICMImageBufferPreferVideoMemory = 2;

{ values for deinterlaceRequest and deinterlaceAnswer }
const
	kICMNoDeinterlacing = 0;
	kICMDeinterlaceFields = 1;

type
	ImageCodecTimeTriggerProcPtr = procedure( refcon: UnivPtr );
	ImageCodecDrawBandCompleteProcPtr = procedure( refcon: UnivPtr; drawBandResult: ComponentResult; drawBandCompleteFlags: UInt32 );
	ImageCodecTimeTriggerUPP = ImageCodecTimeTriggerProcPtr;
	ImageCodecDrawBandCompleteUPP = ImageCodecDrawBandCompleteProcPtr;
{
 *  NewImageCodecTimeTriggerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewImageCodecTimeTriggerUPP( userRoutine: ImageCodecTimeTriggerProcPtr ): ImageCodecTimeTriggerUPP; external name '_NewImageCodecTimeTriggerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;
		wantedPixelSize:		SInt16;
		extendWidth:			SInt16;
		extendHeight:			SInt16;
		bandMin:				SInt16;
		bandInc:				SInt16;
		pad:					SInt16;
		time:					UInt32;
		flags2:					SInt32;								{  field new in QuickTime 4.0  }
=======
>>>>>>> origin/fixes_2.4
	end;
{  codec condition flags   }
const
	codecConditionFirstBand = 1 shl 0;
	codecConditionLastBand = 1 shl 1;
	codecConditionFirstFrame = 1 shl 2;
	codecConditionNewDepth = 1 shl 3;
	codecConditionNewTransform = 1 shl 4;
	codecConditionNewSrcRect = 1 shl 5;
	codecConditionNewMask = 1 shl 6;
	codecConditionNewMatte = 1 shl 7;
	codecConditionNewTransferMode = 1 shl 8;
	codecConditionNewClut = 1 shl 9;
	codecConditionNewAccuracy = 1 shl 10;
	codecConditionNewDestination = 1 shl 11;
	codecConditionFirstScreen = 1 shl 12;
	codecConditionDoCursor = 1 shl 13;
	codecConditionCatchUpDiff = 1 shl 14;
	codecConditionMaskMayBeChanged = 1 shl 15;
	codecConditionToBuffer = 1 shl 16;
	codecConditionCodecChangedMask = 1 shl 31;


const
	codecInfoResourceType = FourCharCode('cdci'); { codec info resource type }
	codecInterfaceVersion = 2;     { high word returned in component GetVersion }

type
	CDSequenceDataSourceQueueEntry = record
		nextBusy: UnivPtr;

		descSeed: SIGNEDLONG;
		dataDesc: Handle;
		data: UnivPtr;
		dataSize: SIGNEDLONG;

		useCount: SIGNEDLONG;

		frameTime: TimeValue;
		frameDuration: TimeValue;
		timeScale: TimeValue;
	end;
	CDSequenceDataSourceQueueEntryPtr = ^CDSequenceDataSourceQueueEntry;
type
	CDSequenceDataSource = record
		recordSize: SIGNEDLONG;

		next: UnivPtr;

		seqID: ImageSequence;
		sourceID: ImageSequenceDataSource;
		sourceType: OSType;
		sourceInputNumber: SIGNEDLONG;
		dataPtr: UnivPtr;
		dataDescription: Handle;
		changeSeed: SIGNEDLONG;
		transferProc: ICMConvertDataFormatUPP;
		transferRefcon: UnivPtr;
		dataSize: SIGNEDLONG;

                                              { fields available in QT 3 and later }

		dataQueue: QHdrPtr;              { queue of CDSequenceDataSourceQueueEntry structures}

		originalDataPtr: UnivPtr;
		originalDataSize: SIGNEDLONG;
		originalDataDescription: Handle;
		originalDataDescriptionSeed: SIGNEDLONG;
	end;
	CDSequenceDataSourcePtr = ^CDSequenceDataSource;
type
	ICMFrameTimeInfo = record
		startTime: wide;
		scale: SIGNEDLONG;
		duration: SIGNEDLONG;
	end;
	ICMFrameTimeInfoPtr = ^ICMFrameTimeInfo;
type
	CodecCompressParams = record
		sequenceID: ImageSequence;             { precompress,bandcompress }
		imageDescription: ImageDescriptionHandle;   { precompress,bandcompress }
		data: Ptr;
		bufferSize: SIGNEDLONG;
		frameNumber: SIGNEDLONG;
		startLine: SIGNEDLONG;
		stopLine: SIGNEDLONG;
		conditionFlags: SIGNEDLONG;
		callerFlags: CodecFlags;
		capabilities: CodecCapabilitiesPtr;           { precompress,bandcompress }
		progressProcRecord: ICMProgressProcRecord;
		completionProcRecord: ICMCompletionProcRecord;
		flushProcRecord: ICMFlushProcRecord;

		srcPixMap: PixMap;              { precompress,bandcompress }
		prevPixMap: PixMap;
		spatialQuality: CodecQ;
		temporalQuality: CodecQ;
		similarity: Fixed;
		dataRateParams: DataRateParamsPtr;
		reserved: SIGNEDLONG;

                                              { The following fields only exist for QuickTime 2.1 and greater }
		majorSourceChangeSeed: UInt16;
		minorSourceChangeSeed: UInt16;
		sourceData: CDSequenceDataSourcePtr;

                                              { The following fields only exist for QuickTime 2.5 and greater }
		preferredPacketSizeInBytes: SIGNEDLONG;

                                              { The following fields only exist for QuickTime 3.0 and greater }
		requestedBufferWidth: SIGNEDLONG;   { must set codecWantsSpecialScaling to indicate this field is valid}
		requestedBufferHeight: SIGNEDLONG;  { must set codecWantsSpecialScaling to indicate this field is valid}

                                              { The following fields only exist for QuickTime 4.0 and greater }
		wantedSourcePixelType: OSType;

                                              { The following fields only exist for QuickTime 5.0 and greater }
		compressedDataSize: SIGNEDLONG;     { if nonzero, this overrides (*imageDescription)->dataSize}
		taskWeight: UInt32;             { preferred weight for MP tasks implementing this operation}
		taskName: OSType;               { preferred name (type) for MP tasks implementing this operation}
	end;
type
	CodecDecompressParamsPtr = ^CodecDecompressParams;
	CodecDecompressParams = record
		sequenceID: ImageSequence;             { predecompress,banddecompress }
		imageDescription: ImageDescriptionHandle;   { predecompress,banddecompress }
		data: Ptr;
		bufferSize: SIGNEDLONG;
		frameNumber: SIGNEDLONG;
		startLine: SIGNEDLONG;
		stopLine: SIGNEDLONG;
		conditionFlags: SIGNEDLONG;
		callerFlags: CodecFlags;
		capabilities: CodecCapabilitiesPtr;           { predecompress,banddecompress }
		progressProcRecord: ICMProgressProcRecord;
		completionProcRecord: ICMCompletionProcRecord;
		dataProcRecord: ICMDataProcRecord;

		port: CGrafPtr;                   { predecompress,banddecompress }
		dstPixMap: PixMap;              { predecompress,banddecompress }
		maskBits: BitMapPtr;
		mattePixMap: PixMapPtr;
		srcRect: Rect;                { predecompress,banddecompress }
		matrix: MatrixRecordPtr;                 { predecompress,banddecompress }
		accuracy: CodecQ;               { predecompress,banddecompress }
		transferMode: SInt16;           { predecompress,banddecompress }
		frameTime: ICMFrameTimePtr;              { banddecompress }
    reserved: array [0..0] of SIGNEDLONG;

                                              { The following fields only exist for QuickTime 2.0 and greater }
		matrixFlags: SInt8;            { high bit set if 2x resize }
		matrixType: SInt8;
		dstRect: Rect;                { only valid for simple transforms }

                                              { The following fields only exist for QuickTime 2.1 and greater }
		majorSourceChangeSeed: UInt16;
		minorSourceChangeSeed: UInt16;
		sourceData: CDSequenceDataSourcePtr;

		maskRegion: RgnHandle;

                                              { The following fields only exist for QuickTime 2.5 and greater }
		wantedDestinationPixelTypes: OSTypeHandle; { Handle to 0-terminated list of OSTypes }

		screenFloodMethod: SIGNEDLONG;
		screenFloodValue: SIGNEDLONG;
		preferredOffscreenPixelSize: SInt16;

                                              { The following fields only exist for QuickTime 3.0 and greater }
		syncFrameTime: ICMFrameTimeInfoPtr;         { banddecompress }
		needUpdateOnTimeChange: Boolean; { banddecompress }
		enableBlackLining: Boolean;
		needUpdateOnSourceChange: Boolean; { band decompress }
		pad: Boolean;

		unused: SIGNEDLONG;

		finalDestinationPort: CGrafPtr;

		requestedBufferWidth: SIGNEDLONG;   { must set codecWantsSpecialScaling to indicate this field is valid}
		requestedBufferHeight: SIGNEDLONG;  { must set codecWantsSpecialScaling to indicate this field is valid}

                                              { The following fields only exist for QuickTime 4.0 and greater }
		displayableAreaOfRequestedBuffer: Rect; { set in predecompress}
		requestedSingleField: Boolean;
		needUpdateOnNextIdle: Boolean;
		pad2: array [0..1] of Boolean;
		bufferGammaLevel: Fixed;

                                              { The following fields only exist for QuickTime 5.0 and greater }
		taskWeight: UInt32;             { preferred weight for MP tasks implementing this operation}
		taskName: OSType;               { preferred name (type) for MP tasks implementing this operation}

                                              { The following fields only exist for QuickTime 6.0 and greater }
		pad3: Boolean;
		destinationBufferMemoryPreference: UInt8; { a codec's PreDecompress/Preflight call can set this to express a preference about what kind of memory its destination buffer should go into.  no guarantees.}
		codecBufferMemoryPreference: UInt8; { may indicate preferred kind of memory that NewImageGWorld/NewImageBufferMemory should create its buffer in, if applicable.}
		onlyUseCodecIfItIsInUserPreferredCodecList: Boolean; { set to prevent this codec from being used unless it is in the userPreferredCodec list}

		mediaContextID: QTMediaContextID;

                                              { The following fields only exist for QuickTime 6.5 and greater }
		deinterlaceRequest: UInt8;     { set by the ICM before PreDecompress/Preflight }
		deinterlaceAnswer: UInt8;      { codec should set this in PreDecompress/Preflight if it will satisfy the deinterlaceRequest }

                                              { The following fields only exist for QuickTime 7.0 and greater }
		pad4: array[0..1] of UInt8;
		reserved2: SIGNEDLONG;
		reserved3: UInt32;
		reserved4: SIGNEDLONG;
		reserved5: UnivPtr;
		reserved6: UnivPtr;
		reserved7: UnivPtr;
		reserved8: UnivPtr;
	end;
const
	matrixFlagScale2x = 1 shl 7;
	matrixFlagScale1x = 1 shl 6;
	matrixFlagScaleHalf = 1 shl 5;

const
	kScreenFloodMethodNone = 0;
	kScreenFloodMethodKeyColor = 1;
	kScreenFloodMethodAlpha = 2;

const
	kFlushLastQueuedFrame = 0;
	kFlushFirstQueuedFrame = 1;

const
	kNewImageGWorldErase = 1 shl 0;

{ values for destinationBufferMemoryPreference and codecBufferMemoryPreference }
const
	kICMImageBufferNoPreference = 0;
	kICMImageBufferPreferMainMemory = 1;
	kICMImageBufferPreferVideoMemory = 2;

{ values for deinterlaceRequest and deinterlaceAnswer }
const
	kICMNoDeinterlacing = 0;
	kICMDeinterlaceFields = 1;

type
	ImageCodecTimeTriggerProcPtr = procedure( refcon: UnivPtr );
	ImageCodecDrawBandCompleteProcPtr = procedure( refcon: UnivPtr; drawBandResult: ComponentResult; drawBandCompleteFlags: UInt32 );
	ImageCodecTimeTriggerUPP = ImageCodecTimeTriggerProcPtr;
	ImageCodecDrawBandCompleteUPP = ImageCodecDrawBandCompleteProcPtr;
{
 *  NewImageCodecTimeTriggerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewImageCodecTimeTriggerUPP( userRoutine: ImageCodecTimeTriggerProcPtr ): ImageCodecTimeTriggerUPP; external name '_NewImageCodecTimeTriggerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{$ifc OPAQUE_UPP_TYPES}
	ImageCodecTimeTriggerUPP = ^SInt32; { an opaque UPP }
{$elsec}
	ImageCodecTimeTriggerUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	ImageCodecDrawBandCompleteUPP = ^SInt32; { an opaque UPP }
{$elsec}
	ImageCodecDrawBandCompleteUPP = UniversalProcPtr;
{$endc}	

const
	uppImageCodecTimeTriggerProcInfo = $000000C0;
	uppImageCodecDrawBandCompleteProcInfo = $00000FC0;
	{
	 *  NewImageCodecTimeTriggerUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0.2 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewImageCodecTimeTriggerUPP(userRoutine: ImageCodecTimeTriggerProcPtr): ImageCodecTimeTriggerUPP; external name '_NewImageCodecTimeTriggerUPP'; { old name was NewImageCodecTimeTriggerProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewImageCodecDrawBandCompleteUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewImageCodecDrawBandCompleteUPP( userRoutine: ImageCodecDrawBandCompleteProcPtr ): ImageCodecDrawBandCompleteUPP; external name '_NewImageCodecDrawBandCompleteUPP';
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
function NewImageCodecDrawBandCompleteUPP(userRoutine: ImageCodecDrawBandCompleteProcPtr): ImageCodecDrawBandCompleteUPP; external name '_NewImageCodecDrawBandCompleteUPP'; { old name was NewImageCodecDrawBandCompleteProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewImageCodecDrawBandCompleteUPP( userRoutine: ImageCodecDrawBandCompleteProcPtr ): ImageCodecDrawBandCompleteUPP; external name '_NewImageCodecDrawBandCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeImageCodecTimeTriggerUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeImageCodecTimeTriggerUPP( userUPP: ImageCodecTimeTriggerUPP ); external name '_DisposeImageCodecTimeTriggerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeImageCodecTimeTriggerUPP(userUPP: ImageCodecTimeTriggerUPP); external name '_DisposeImageCodecTimeTriggerUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeImageCodecTimeTriggerUPP( userUPP: ImageCodecTimeTriggerUPP ); external name '_DisposeImageCodecTimeTriggerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeImageCodecDrawBandCompleteUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeImageCodecDrawBandCompleteUPP( userUPP: ImageCodecDrawBandCompleteUPP ); external name '_DisposeImageCodecDrawBandCompleteUPP';
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
procedure DisposeImageCodecDrawBandCompleteUPP(userUPP: ImageCodecDrawBandCompleteUPP); external name '_DisposeImageCodecDrawBandCompleteUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeImageCodecDrawBandCompleteUPP( userUPP: ImageCodecDrawBandCompleteUPP ); external name '_DisposeImageCodecDrawBandCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeImageCodecTimeTriggerUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeImageCodecTimeTriggerUPP( refcon: UnivPtr; userUPP: ImageCodecTimeTriggerUPP ); external name '_InvokeImageCodecTimeTriggerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure InvokeImageCodecTimeTriggerUPP(refcon: UnivPtr; userRoutine: ImageCodecTimeTriggerUPP); external name '_InvokeImageCodecTimeTriggerUPP'; { old name was CallImageCodecTimeTriggerProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvokeImageCodecTimeTriggerUPP( refcon: UnivPtr; userUPP: ImageCodecTimeTriggerUPP ); external name '_InvokeImageCodecTimeTriggerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeImageCodecDrawBandCompleteUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeImageCodecDrawBandCompleteUPP( refcon: UnivPtr; drawBandResult: ComponentResult; drawBandCompleteFlags: UInt32; userUPP: ImageCodecDrawBandCompleteUPP ); external name '_InvokeImageCodecDrawBandCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

type
	ImageSubCodecDecompressCapabilitiesPtr = ^ImageSubCodecDecompressCapabilities;
	ImageSubCodecDecompressCapabilities = record
		recordSize: SIGNEDLONG;             { sizeof(ImageSubCodecDecompressCapabilities)}
		decompressRecordSize: SIGNEDLONG;   { size of your codec's decompress record}
		canAsync: Boolean;               { default true}
		pad0: UInt8;
                                              { The following field only exists for QuickTime 4.1 and greater }
		suggestedQueueSize: UInt16;
                                              { The following field only exists for QuickTime 4.0 and greater }
		canProvideTrigger: Boolean;

                                              { The following fields only exist for QuickTime 5.0 and greater }
		subCodecFlushesScreen: Boolean;  { only used on Mac OS X}
		subCodecCallsDrawBandComplete: Boolean;
		pad2: array[0..1] of UInt8;

                                              { The following fields only exist for QuickTime 5.0.1 and greater }
		isChildCodec: Boolean;           { set by base codec before calling Initialize}
		reserved1: UInt8;
		pad4: array[0..1] of UInt8;

                                              { The following fields only exist for QuickTime 7.0 and greater }
		subCodecIsMultiBufferAware: Boolean; { set if subcodec always draws using ImageSubCodecDecompressRecord.baseAddr/rowBytes passed to ImageCodecDrawBand, and always writes every pixel in the buffer}
		subCodecSupportsOutOfOrderDisplayTimes: Boolean;
		subCodecSupportsScheduledBackwardsPlaybackWithDifferenceFrames: Boolean;
		subCodecNeedsHelpReportingNonDisplayableFrames: Boolean;
		baseCodecShouldCallDecodeBandForAllFrames: Boolean;

		pad5: array[0..1] of UInt8;
		subCodecSupportsDrawInDecodeOrder: Boolean; { indicates that it's okay for the subcodec to get a single DrawBand call for each frame in decode order even when frames need reordering.  (This will only happen when other circumstances allow it.)}
		subCodecSupportsDecodeSmoothing: Boolean; { Frame-reordering subcodecs should set this to indicate that they can safely decode a non-droppable frame before drawing the previous non-droppable frame.  This enables smoother playback in GWorlds.}
		pad6: array[0..3] of UInt8;
	end;
const
	kCodecFrameTypeUnknown = 0;
	kCodecFrameTypeKey = 1;
	kCodecFrameTypeDifference = 2;
	kCodecFrameTypeDroppableDifference = 3;

type
	ImageSubCodecDecompressRecordPtr = ^ImageSubCodecDecompressRecord;
	ImageSubCodecDecompressRecord = record
		baseAddr: Ptr;
		rowBytes: SIGNEDLONG;
		codecData: Ptr;
		progressProcRecord: ICMProgressProcRecord;
		dataProcRecord: ICMDataProcRecord;
		userDecompressRecord: UnivPtr;   { pointer to codec-specific per-band data}
		frameType: UInt8;
		inhibitMP: Boolean;              { set this in BeginBand to tell the base decompressor not to call DrawBand from an MP task for this frame.  (Only has any effect for MP-capable subcodecs.  New in QuickTime 5.0.)}
		pad2: array[0..11] of UInt8;
		priv: array[0..1] of SIGNEDLONG;

                                              { The following fields only exist for QuickTime 5.0 and greater }
		drawBandCompleteUPP: ImageCodecDrawBandCompleteUPP; { only used if subcodec set subCodecCallsDrawBandComplete; if drawBandCompleteUPP is non-nil, codec must call it when a frame is finished, but may return from DrawBand before the frame is finished. }
		drawBandCompleteRefCon: UnivPtr; { Note: do not call drawBandCompleteUPP directly from a hardware interrupt; instead, use DTInstall to run a function at deferred task time, and call drawBandCompleteUPP from that. }

                                              { The following fields only exist for QuickTime 7.0 and greater }
		reserved1: UnivPtr;
		reserved2: SIGNEDLONG;
		reserved3: SIGNEDLONG;
	end;
{
  These data structures are used by code that wants to pass planar pixmap 
   information around.
  The structure below gives the basic idea of what is being done.
  Normal instances of code will use a fixed number of planes (eg YUV420 uses 
   three planes, Y, U and V). Each such code instance will define its own
   version of the PlanarPixMapInfo struct counting the number of planes it 
   needs along with defining constants that specify the meanings of each
   plane.
}
type
	PlanarComponentInfo = record
		offset: SInt32;
		rowBytes: UInt32;
	end;
type
	PlanarPixMapInfoPtr = ^PlanarPixMapInfo;
	PlanarPixMapInfo = record
		componentInfo: array [0..0] of PlanarComponentInfo;
	end;
type
	PlanarPixmapInfoSorensonYUV9Ptr = ^PlanarPixmapInfoSorensonYUV9;
	PlanarPixmapInfoSorensonYUV9 = record
		componentInfoY: PlanarComponentInfo;
		componentInfoU: PlanarComponentInfo;
		componentInfoV: PlanarComponentInfo;
	end;
type
	PlanarPixmapInfoYUV420Ptr = ^PlanarPixmapInfoYUV420;
	PlanarPixmapInfoYUV420 = record
		componentInfoY: PlanarComponentInfo;
		componentInfoCb: PlanarComponentInfo;
		componentInfoCr: PlanarComponentInfo;
	end;
const
	codecSuggestedBufferSentinel = FourCharCode('sent'); { codec public resource containing suggested data pattern to put past end of data buffer }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

=======
>>>>>>> origin/cpstrnew

{ name of parameters or effect -- placed in root container, required }
<<<<<<< HEAD
const
	kParameterTitleName = FourCharCode('name');
	kParameterTitleID = 1;

{ codec sub-type of parameters or effect -- placed in root container, required }
const
	kParameterWhatName = FourCharCode('what');
	kParameterWhatID = 1;

{ effect version -- placed in root container, optional, but recommended }
const
	kParameterVersionName = FourCharCode('vers');
	kParameterVersionID = 1;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{ is effect repeatable -- placed in root container, optional, default is TRUE}
const
	kParameterRepeatableName = FourCharCode('pete');
	kParameterRepeatableID = 1;

{ name of parameters or effect -- placed in root container, required }
const
	kParameterTitleName = FourCharCode('name');
	kParameterTitleID = 1;
=======
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
{ codec sub-type of parameters or effect -- placed in root container, required }
const
<<<<<<< HEAD
	kParameterRepeatableTrue = 1;
	kParameterRepeatableFalse = 0;

=======
const
	kParameterTitleName = FourCharCode('name');
	kParameterTitleID = 1;

{ codec sub-type of parameters or effect -- placed in root container, required }
const
	kParameterWhatName = FourCharCode('what');
	kParameterWhatID = 1;

{ effect version -- placed in root container, optional, but recommended }
const
	kParameterVersionName = FourCharCode('vers');
	kParameterVersionID = 1;

{ is effect repeatable -- placed in root container, optional, default is TRUE}
const
	kParameterRepeatableName = FourCharCode('pete');
	kParameterRepeatableID = 1;

const
	kParameterRepeatableTrue = 1;
	kParameterRepeatableFalse = 0;

>>>>>>> origin/fixes_2.4
{ substitution codec in case effect is missing -- placed in root container, recommended }
const
	kParameterAlternateCodecName = FourCharCode('subs');
	kParameterAlternateCodecID = 1;
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
{ maximum number of sources -- placed in root container, required }
const
	kParameterSourceCountName = FourCharCode('srcs');
	kParameterSourceCountID = 1;

{ EFFECT CLASSES}

{
   The effect major class defines the major grouping of the effect.
   Major classes are defined only by Apple and are not extendable by third
   parties.  Major classes are used for filtering of the effect list by
   applications, but do not define what UI sub-group may or may not be
   presented to the user.  For example, the major class may be a transition,
   but the minor class may be a wipe.  
}

{
   Effects that fail to include a
   kEffectMajorClassType will be classified as kMiscMajorClass.
}
const
	kEffectMajorClassType = FourCharCode('clsa');
	kEffectMajorClassID = 1;

const
	kGeneratorMajorClass = FourCharCode('genr'); { zero source effects}
	kFilterMajorClass = FourCharCode('filt'); { one source effects}
	kTransitionMajorClass = FourCharCode('tran'); { multisource morph effects }
	kCompositorMajorClass = FourCharCode('comp'); { multisource layer effects}
	kMiscMajorClass = FourCharCode('misc'); { all other effects}

{
   The effect minor class defines the grouping of effects for the purposes
   of UI.  Apple defines a set of minor classes and will extend it over
   time.  Apple also provides strings within the UI for minor classes
   that it defines.  Third party developers may either classify
   their effects as a type defined by Apple, or may define their own
   minor class.  Effects which define a minor class of their own
   must also then supply a kEffectMinorClassNameType atom.
}

{
=======
	kParameterWhatName = FourCharCode('what');
	kParameterWhatID = 1;

{ effect version -- placed in root container, optional, but recommended }
const
	kParameterVersionName = FourCharCode('vers');
	kParameterVersionID = 1;

{ is effect repeatable -- placed in root container, optional, default is TRUE}
const
	kParameterRepeatableName = FourCharCode('pete');
	kParameterRepeatableID = 1;

const
	kParameterRepeatableTrue = 1;
	kParameterRepeatableFalse = 0;

{ substitution codec in case effect is missing -- placed in root container, recommended }
const
	kParameterAlternateCodecName = FourCharCode('subs');
	kParameterAlternateCodecID = 1;

{ maximum number of sources -- placed in root container, required }
const
	kParameterSourceCountName = FourCharCode('srcs');
	kParameterSourceCountID = 1;

{ EFFECT CLASSES}

{
   The effect major class defines the major grouping of the effect.
   Major classes are defined only by Apple and are not extendable by third
   parties.  Major classes are used for filtering of the effect list by
   applications, but do not define what UI sub-group may or may not be
   presented to the user.  For example, the major class may be a transition,
   but the minor class may be a wipe.  
}

=======
{ name of parameters or effect -- placed in root container, required }
const
	kParameterTitleName = FourCharCode('name');
	kParameterTitleID = 1;

{ codec sub-type of parameters or effect -- placed in root container, required }
const
	kParameterWhatName = FourCharCode('what');
	kParameterWhatID = 1;

{ effect version -- placed in root container, optional, but recommended }
const
	kParameterVersionName = FourCharCode('vers');
	kParameterVersionID = 1;

{ is effect repeatable -- placed in root container, optional, default is TRUE}
const
	kParameterRepeatableName = FourCharCode('pete');
	kParameterRepeatableID = 1;

const
	kParameterRepeatableTrue = 1;
	kParameterRepeatableFalse = 0;

{ substitution codec in case effect is missing -- placed in root container, recommended }
const
	kParameterAlternateCodecName = FourCharCode('subs');
	kParameterAlternateCodecID = 1;

{ maximum number of sources -- placed in root container, required }
const
	kParameterSourceCountName = FourCharCode('srcs');
	kParameterSourceCountID = 1;

{ EFFECT CLASSES}

{
   The effect major class defines the major grouping of the effect.
   Major classes are defined only by Apple and are not extendable by third
   parties.  Major classes are used for filtering of the effect list by
   applications, but do not define what UI sub-group may or may not be
   presented to the user.  For example, the major class may be a transition,
   but the minor class may be a wipe.  
}

>>>>>>> graemeg/cpstrnew
{
   Effects that fail to include a
   kEffectMajorClassType will be classified as kMiscMajorClass.
}
const
	kEffectMajorClassType = FourCharCode('clsa');
	kEffectMajorClassID = 1;

const
	kGeneratorMajorClass = FourCharCode('genr'); { zero source effects}
	kFilterMajorClass = FourCharCode('filt'); { one source effects}
	kTransitionMajorClass = FourCharCode('tran'); { multisource morph effects }
	kCompositorMajorClass = FourCharCode('comp'); { multisource layer effects}
	kMiscMajorClass = FourCharCode('misc'); { all other effects}

{
   The effect minor class defines the grouping of effects for the purposes
   of UI.  Apple defines a set of minor classes and will extend it over
   time.  Apple also provides strings within the UI for minor classes
   that it defines.  Third party developers may either classify
   their effects as a type defined by Apple, or may define their own
   minor class.  Effects which define a minor class of their own
   must also then supply a kEffectMinorClassNameType atom.
}

{
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
   If a kEffectMinorClassNameType atom is present, but
   the minor type is one defined by Apple, the Apple supplied
   string will be used in the UI.
}
<<<<<<< HEAD
<<<<<<< HEAD

{
   Effects that fail to supply a kEffectMinorClassType will be 
   classified as kMiscMinorClass.
}
const
	kEffectMinorClassType = FourCharCode('clsi');
	kEffectMinorClassID = 1;
	kEffectMinorClassNameType = FourCharCode('clsn');
	kEffectMinorClassNameID = 1;

=======

{
   Effects that fail to supply a kEffectMinorClassType will be 
   classified as kMiscMinorClass.
}
const
	kEffectMinorClassType = FourCharCode('clsi');
	kEffectMinorClassID = 1;
	kEffectMinorClassNameType = FourCharCode('clsn');
	kEffectMinorClassNameID = 1;

>>>>>>> graemeg/cpstrnew
=======

{
   Effects that fail to supply a kEffectMinorClassType will be 
   classified as kMiscMinorClass.
}
const
	kEffectMinorClassType = FourCharCode('clsi');
	kEffectMinorClassID = 1;
	kEffectMinorClassNameType = FourCharCode('clsn');
	kEffectMinorClassNameID = 1;

>>>>>>> graemeg/cpstrnew
const
=======
{ name of parameters or effect -- placed in root container, required }
const
	kParameterTitleName = FourCharCode('name');
	kParameterTitleID = 1;

{ codec sub-type of parameters or effect -- placed in root container, required }
const
	kParameterWhatName = FourCharCode('what');
	kParameterWhatID = 1;

{ effect version -- placed in root container, optional, but recommended }
const
	kParameterVersionName = FourCharCode('vers');
	kParameterVersionID = 1;

{ is effect repeatable -- placed in root container, optional, default is TRUE}
const
	kParameterRepeatableName = FourCharCode('pete');
	kParameterRepeatableID = 1;

const
	kParameterRepeatableTrue = 1;
	kParameterRepeatableFalse = 0;

{ substitution codec in case effect is missing -- placed in root container, recommended }
const
	kParameterAlternateCodecName = FourCharCode('subs');
	kParameterAlternateCodecID = 1;

{ maximum number of sources -- placed in root container, required }
const
	kParameterSourceCountName = FourCharCode('srcs');
	kParameterSourceCountID = 1;

{ EFFECT CLASSES}

{
   The effect major class defines the major grouping of the effect.
   Major classes are defined only by Apple and are not extendable by third
   parties.  Major classes are used for filtering of the effect list by
   applications, but do not define what UI sub-group may or may not be
   presented to the user.  For example, the major class may be a transition,
   but the minor class may be a wipe.  
}

{
   Effects that fail to include a
   kEffectMajorClassType will be classified as kMiscMajorClass.
}
const
	kEffectMajorClassType = FourCharCode('clsa');
	kEffectMajorClassID = 1;

const
	kGeneratorMajorClass = FourCharCode('genr'); { zero source effects}
	kFilterMajorClass = FourCharCode('filt'); { one source effects}
	kTransitionMajorClass = FourCharCode('tran'); { multisource morph effects }
	kCompositorMajorClass = FourCharCode('comp'); { multisource layer effects}
	kMiscMajorClass = FourCharCode('misc'); { all other effects}

{
   The effect minor class defines the grouping of effects for the purposes
   of UI.  Apple defines a set of minor classes and will extend it over
   time.  Apple also provides strings within the UI for minor classes
   that it defines.  Third party developers may either classify
   their effects as a type defined by Apple, or may define their own
   minor class.  Effects which define a minor class of their own
   must also then supply a kEffectMinorClassNameType atom.
}

{
   If a kEffectMinorClassNameType atom is present, but
   the minor type is one defined by Apple, the Apple supplied
   string will be used in the UI.
}

{
   Effects that fail to supply a kEffectMinorClassType will be 
   classified as kMiscMinorClass.
}
const
	kEffectMinorClassType = FourCharCode('clsi');
	kEffectMinorClassID = 1;
	kEffectMinorClassNameType = FourCharCode('clsn');
	kEffectMinorClassNameID = 1;

const
>>>>>>> graemeg/cpstrnew
=======
{ name of parameters or effect -- placed in root container, required }
const
	kParameterTitleName = FourCharCode('name');
	kParameterTitleID = 1;

{ codec sub-type of parameters or effect -- placed in root container, required }
const
	kParameterWhatName = FourCharCode('what');
	kParameterWhatID = 1;

{ effect version -- placed in root container, optional, but recommended }
const
	kParameterVersionName = FourCharCode('vers');
	kParameterVersionID = 1;

{ is effect repeatable -- placed in root container, optional, default is TRUE}
const
	kParameterRepeatableName = FourCharCode('pete');
	kParameterRepeatableID = 1;

const
	kParameterRepeatableTrue = 1;
	kParameterRepeatableFalse = 0;

{ substitution codec in case effect is missing -- placed in root container, recommended }
const
	kParameterAlternateCodecName = FourCharCode('subs');
	kParameterAlternateCodecID = 1;

{ maximum number of sources -- placed in root container, required }
const
	kParameterSourceCountName = FourCharCode('srcs');
	kParameterSourceCountID = 1;

{ EFFECT CLASSES}

{
   The effect major class defines the major grouping of the effect.
   Major classes are defined only by Apple and are not extendable by third
   parties.  Major classes are used for filtering of the effect list by
   applications, but do not define what UI sub-group may or may not be
   presented to the user.  For example, the major class may be a transition,
   but the minor class may be a wipe.  
}

{
   Effects that fail to include a
   kEffectMajorClassType will be classified as kMiscMajorClass.
}
const
	kEffectMajorClassType = FourCharCode('clsa');
	kEffectMajorClassID = 1;

const
	kGeneratorMajorClass = FourCharCode('genr'); { zero source effects}
	kFilterMajorClass = FourCharCode('filt'); { one source effects}
	kTransitionMajorClass = FourCharCode('tran'); { multisource morph effects }
	kCompositorMajorClass = FourCharCode('comp'); { multisource layer effects}
	kMiscMajorClass = FourCharCode('misc'); { all other effects}

{
   The effect minor class defines the grouping of effects for the purposes
   of UI.  Apple defines a set of minor classes and will extend it over
   time.  Apple also provides strings within the UI for minor classes
   that it defines.  Third party developers may either classify
   their effects as a type defined by Apple, or may define their own
   minor class.  Effects which define a minor class of their own
   must also then supply a kEffectMinorClassNameType atom.
}

{
   If a kEffectMinorClassNameType atom is present, but
   the minor type is one defined by Apple, the Apple supplied
   string will be used in the UI.
}

{
   Effects that fail to supply a kEffectMinorClassType will be 
   classified as kMiscMinorClass.
}
const
	kEffectMinorClassType = FourCharCode('clsi');
	kEffectMinorClassID = 1;
	kEffectMinorClassNameType = FourCharCode('clsn');
	kEffectMinorClassNameID = 1;

const
>>>>>>> origin/cpstrnew
	kGeneratorMinorClass = FourCharCode('genr'); { "Generators"}
	kRenderMinorClass = FourCharCode('rend'); { "Render"}
	kFilterMinorClass = FourCharCode('filt'); { "Filters"}
	kArtisticMinorClass = FourCharCode('arts'); { "Artistic}
	kBlurMinorClass = FourCharCode('blur'); { "Blur"}
	kSharpenMinorClass = FourCharCode('shrp'); { "Sharpen"}
	kDistortMinorClass = FourCharCode('dist'); { "Distort"}
	kNoiseMinorClass = FourCharCode('nois'); { "Noise"}
	kAdjustmentMinorClass = FourCharCode('adst'); { "Adjustments"}
	kTransitionMinorClass = FourCharCode('tran'); { "Transitions"}
	kWipeMinorClass = FourCharCode('wipe'); { "Wipes"}
	k3DMinorClass = FourCharCode('pzre'); { "3D Transitions"}
	kCompositorMinorClass = FourCharCode('comp'); { "Compositors"}
	kEffectsMinorClass = FourCharCode('fxfx'); { "Special Effects"}
	kMiscMinorClass = FourCharCode('misc'); { "Miscellaneous"}

{
   Effects can define a number of "preset" values which will be presented to the user
   in a simplified UI.  Each preset is an atom within the parameter description list
   and must have an atom ID from 1 going up sequentially.  Inside of this atom are three other
   atoms containing:
    1) the name of the preset as a Pascal string
    2) a preview picture for the preset, 86 x 64 pixels in size
    3) the ENTIRE set of parameter values needed to create a sample of this preset.
}
const
	kEffectPresetType = FourCharCode('peff');
	kPresetNameType = FourCharCode('pnam');
	kPresetNameID = 1;
	kPresetPreviewPictureType = FourCharCode('ppct');
	kPresetPreviewPictureID = 1;
	kPresetSettingsType = FourCharCode('psst');
	kPresetSettingsID = 1;

const
	kParameterDependencyName = FourCharCode('deep');
	kParameterDependencyID = 1;

const
	kParameterListDependsUponColorProfiles = FourCharCode('prof');
	kParameterListDependsUponFonts = FourCharCode('font');

type
	ParameterDependancyRecordPtr = ^ParameterDependancyRecord;
	ParameterDependancyRecord = record
		dependCount: SIGNEDLONG;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		depends: array [0..0] of OSType;
=======
    depends: array [0..0] of OSType;
>>>>>>> graemeg/cpstrnew
=======
    depends: array [0..0] of OSType;
>>>>>>> graemeg/cpstrnew
=======
    depends: array [0..0] of OSType;
>>>>>>> graemeg/cpstrnew
=======
    depends: array [0..0] of OSType;
>>>>>>> origin/cpstrnew
	end;
{
   enumeration list in container -- placed in root container, optional unless used by a
   parameter in the list
}
const
	kParameterEnumList = FourCharCode('enum');
=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure InvokeImageCodecDrawBandCompleteUPP(refcon: UnivPtr; drawBandResult: ComponentResult; drawBandCompleteFlags: UInt32; userRoutine: ImageCodecDrawBandCompleteUPP); external name '_InvokeImageCodecDrawBandCompleteUPP'; { old name was CallImageCodecDrawBandCompleteProc }
type
	ImageSubCodecDecompressCapabilitiesPtr = ^ImageSubCodecDecompressCapabilities;
	ImageSubCodecDecompressCapabilities = record
		recordSize:				SInt32;								{  sizeof(ImageSubCodecDecompressCapabilities) }
		decompressRecordSize:	SInt32;								{  size of your codec's decompress record }
		canAsync:				boolean;								{  default true }
		pad0:					SInt8;
																		{  The following field only exists for QuickTime 4.1 and greater  }
		suggestedQueueSize:		UInt16;
																		{  The following field only exists for QuickTime 4.0 and greater  }
		canProvideTrigger:		boolean;
																		{  The following fields only exist for QuickTime 5.0 and greater  }
		subCodecFlushesScreen:	boolean;								{  only used on Mac OS X }
		subCodecCallsDrawBandComplete: boolean;
		pad2:					SInt8;
																		{  The following fields only exist for QuickTime 5.0.1 and greater  }
		isChildCodec:			boolean;								{  set by base codec before calling Initialize }
		pad3,pad4,pad5:			SInt8;
	end;


const
	kCodecFrameTypeUnknown		= 0;
	kCodecFrameTypeKey			= 1;
	kCodecFrameTypeDifference	= 2;
	kCodecFrameTypeDroppableDifference = 3;


type
	ImageSubCodecDecompressRecordPtr = ^ImageSubCodecDecompressRecord;
	ImageSubCodecDecompressRecord = record
		baseAddr:				Ptr;
		rowBytes:				SInt32;
		codecData:				Ptr;
		progressProcRecord:		ICMProgressProcRecord;
		dataProcRecord:			ICMDataProcRecord;
		userDecompressRecord:	Ptr;									{  pointer to codec-specific per-band data }
		frameType:				SInt8;
		inhibitMP:				boolean;								{  set this in BeginBand to tell the base decompressor not to call DrawBand from an MP task for this frame.  (Only has any effect for MP-capable subcodecs.  New in QuickTime 5.0.) }
		pad:					packed array [0..1] of UInt8;
		priv:					array [0..1] of SInt32;
																		{  The following fields only exist for QuickTime 5.0 and greater  }
		drawBandCompleteUPP:	ImageCodecDrawBandCompleteUPP;			{  only used if subcodec set subCodecCallsDrawBandComplete; if drawBandCompleteUPP is non-nil, codec must call it when a frame is finished, but may return from DrawBand before the frame is finished.  }
		drawBandCompleteRefCon:	Ptr;									{  Note: do not call drawBandCompleteUPP directly from a hardware interrupt; instead, use DTInstall to run a function at deferred task time, and call drawBandCompleteUPP from that.  }
	end;

	{
	  These data structures are used by code that wants to pass planar pixmap 
	   information around.
	  The structure below gives the basic idea of what is being done.
	  Normal instances of code will use a fixed number of planes (eg YUV420 uses 
	   three planes, Y, U and V). Each such code instance will define its own
	   version of the PlanarPixMapInfo struct counting the number of planes it 
	   needs along with defining constants that specify the meanings of each
	   plane.
	}
	PlanarComponentInfoPtr = ^PlanarComponentInfo;
	PlanarComponentInfo = record
		offset:					SInt32;
		rowBytes:				UInt32;
	end;

	PlanarPixMapInfoPtr = ^PlanarPixMapInfo;
	PlanarPixMapInfo = record
		componentInfo:			array [0..0] of PlanarComponentInfo;
	end;

	PlanarPixmapInfoSorensonYUV9Ptr = ^PlanarPixmapInfoSorensonYUV9;
	PlanarPixmapInfoSorensonYUV9 = record
		componentInfoY:			PlanarComponentInfo;
		componentInfoU:			PlanarComponentInfo;
		componentInfoV:			PlanarComponentInfo;
	end;

	PlanarPixmapInfoYUV420Ptr = ^PlanarPixmapInfoYUV420;
	PlanarPixmapInfoYUV420 = record
		componentInfoY:			PlanarComponentInfo;
		componentInfoCb:		PlanarComponentInfo;
		componentInfoCr:		PlanarComponentInfo;
	end;


const
	codecSuggestedBufferSentinel = FourCharCode('sent');						{  codec public resource containing suggested data pattern to put past end of data buffer  }


	{  name of parameters or effect -- placed in root container, required  }
	kParameterTitleName			= FourCharCode('name');
	kParameterTitleID			= 1;

	{  codec sub-type of parameters or effect -- placed in root container, required  }
	kParameterWhatName			= FourCharCode('what');
	kParameterWhatID			= 1;
=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure InvokeImageCodecDrawBandCompleteUPP(refcon: UnivPtr; drawBandResult: ComponentResult; drawBandCompleteFlags: UInt32; userRoutine: ImageCodecDrawBandCompleteUPP); external name '_InvokeImageCodecDrawBandCompleteUPP'; { old name was CallImageCodecDrawBandCompleteProc }
type
	ImageSubCodecDecompressCapabilitiesPtr = ^ImageSubCodecDecompressCapabilities;
	ImageSubCodecDecompressCapabilities = record
		recordSize:				SInt32;								{  sizeof(ImageSubCodecDecompressCapabilities) }
		decompressRecordSize:	SInt32;								{  size of your codec's decompress record }
		canAsync:				boolean;								{  default true }
		pad0:					SInt8;
																		{  The following field only exists for QuickTime 4.1 and greater  }
		suggestedQueueSize:		UInt16;
																		{  The following field only exists for QuickTime 4.0 and greater  }
		canProvideTrigger:		boolean;
																		{  The following fields only exist for QuickTime 5.0 and greater  }
		subCodecFlushesScreen:	boolean;								{  only used on Mac OS X }
		subCodecCallsDrawBandComplete: boolean;
		pad2:					SInt8;
																		{  The following fields only exist for QuickTime 5.0.1 and greater  }
		isChildCodec:			boolean;								{  set by base codec before calling Initialize }
		pad3,pad4,pad5:			SInt8;
	end;


const
	kCodecFrameTypeUnknown		= 0;
	kCodecFrameTypeKey			= 1;
	kCodecFrameTypeDifference	= 2;
	kCodecFrameTypeDroppableDifference = 3;


type
	ImageSubCodecDecompressRecordPtr = ^ImageSubCodecDecompressRecord;
	ImageSubCodecDecompressRecord = record
		baseAddr:				Ptr;
		rowBytes:				SInt32;
		codecData:				Ptr;
		progressProcRecord:		ICMProgressProcRecord;
		dataProcRecord:			ICMDataProcRecord;
		userDecompressRecord:	Ptr;									{  pointer to codec-specific per-band data }
		frameType:				SInt8;
		inhibitMP:				boolean;								{  set this in BeginBand to tell the base decompressor not to call DrawBand from an MP task for this frame.  (Only has any effect for MP-capable subcodecs.  New in QuickTime 5.0.) }
		pad:					packed array [0..1] of UInt8;
		priv:					array [0..1] of SInt32;
																		{  The following fields only exist for QuickTime 5.0 and greater  }
		drawBandCompleteUPP:	ImageCodecDrawBandCompleteUPP;			{  only used if subcodec set subCodecCallsDrawBandComplete; if drawBandCompleteUPP is non-nil, codec must call it when a frame is finished, but may return from DrawBand before the frame is finished.  }
		drawBandCompleteRefCon:	Ptr;									{  Note: do not call drawBandCompleteUPP directly from a hardware interrupt; instead, use DTInstall to run a function at deferred task time, and call drawBandCompleteUPP from that.  }
	end;

	{
	  These data structures are used by code that wants to pass planar pixmap 
	   information around.
	  The structure below gives the basic idea of what is being done.
	  Normal instances of code will use a fixed number of planes (eg YUV420 uses 
	   three planes, Y, U and V). Each such code instance will define its own
	   version of the PlanarPixMapInfo struct counting the number of planes it 
	   needs along with defining constants that specify the meanings of each
	   plane.
	}
	PlanarComponentInfoPtr = ^PlanarComponentInfo;
	PlanarComponentInfo = record
		offset:					SInt32;
		rowBytes:				UInt32;
	end;

	PlanarPixMapInfoPtr = ^PlanarPixMapInfo;
	PlanarPixMapInfo = record
		componentInfo:			array [0..0] of PlanarComponentInfo;
	end;

	PlanarPixmapInfoSorensonYUV9Ptr = ^PlanarPixmapInfoSorensonYUV9;
	PlanarPixmapInfoSorensonYUV9 = record
		componentInfoY:			PlanarComponentInfo;
		componentInfoU:			PlanarComponentInfo;
		componentInfoV:			PlanarComponentInfo;
	end;

	PlanarPixmapInfoYUV420Ptr = ^PlanarPixmapInfoYUV420;
	PlanarPixmapInfoYUV420 = record
		componentInfoY:			PlanarComponentInfo;
		componentInfoCb:		PlanarComponentInfo;
		componentInfoCr:		PlanarComponentInfo;
	end;


const
	codecSuggestedBufferSentinel = FourCharCode('sent');						{  codec public resource containing suggested data pattern to put past end of data buffer  }


	{  name of parameters or effect -- placed in root container, required  }
	kParameterTitleName			= FourCharCode('name');
	kParameterTitleID			= 1;

	{  codec sub-type of parameters or effect -- placed in root container, required  }
	kParameterWhatName			= FourCharCode('what');
	kParameterWhatID			= 1;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ maximum number of sources -- placed in root container, required }
const
	kParameterSourceCountName = FourCharCode('srcs');
	kParameterSourceCountID = 1;

{ EFFECT CLASSES}

{
   The effect major class defines the major grouping of the effect.
   Major classes are defined only by Apple and are not extendable by third
   parties.  Major classes are used for filtering of the effect list by
   applications, but do not define what UI sub-group may or may not be
   presented to the user.  For example, the major class may be a transition,
   but the minor class may be a wipe.  
}

{
   Effects that fail to include a
   kEffectMajorClassType will be classified as kMiscMajorClass.
}
const
	kEffectMajorClassType = FourCharCode('clsa');
	kEffectMajorClassID = 1;

const
	kGeneratorMajorClass = FourCharCode('genr'); { zero source effects}
	kFilterMajorClass = FourCharCode('filt'); { one source effects}
	kTransitionMajorClass = FourCharCode('tran'); { multisource morph effects }
	kCompositorMajorClass = FourCharCode('comp'); { multisource layer effects}
	kMiscMajorClass = FourCharCode('misc'); { all other effects}

{
   The effect minor class defines the grouping of effects for the purposes
   of UI.  Apple defines a set of minor classes and will extend it over
   time.  Apple also provides strings within the UI for minor classes
   that it defines.  Third party developers may either classify
   their effects as a type defined by Apple, or may define their own
   minor class.  Effects which define a minor class of their own
   must also then supply a kEffectMinorClassNameType atom.
}

{
   If a kEffectMinorClassNameType atom is present, but
   the minor type is one defined by Apple, the Apple supplied
   string will be used in the UI.
}

{
   Effects that fail to supply a kEffectMinorClassType will be 
   classified as kMiscMinorClass.
}
const
	kEffectMinorClassType = FourCharCode('clsi');
	kEffectMinorClassID = 1;
	kEffectMinorClassNameType = FourCharCode('clsn');
	kEffectMinorClassNameID = 1;

const
	kGeneratorMinorClass = FourCharCode('genr'); { "Generators"}
	kRenderMinorClass = FourCharCode('rend'); { "Render"}
	kFilterMinorClass = FourCharCode('filt'); { "Filters"}
	kArtisticMinorClass = FourCharCode('arts'); { "Artistic}
	kBlurMinorClass = FourCharCode('blur'); { "Blur"}
	kSharpenMinorClass = FourCharCode('shrp'); { "Sharpen"}
	kDistortMinorClass = FourCharCode('dist'); { "Distort"}
	kNoiseMinorClass = FourCharCode('nois'); { "Noise"}
	kAdjustmentMinorClass = FourCharCode('adst'); { "Adjustments"}
	kTransitionMinorClass = FourCharCode('tran'); { "Transitions"}
	kWipeMinorClass = FourCharCode('wipe'); { "Wipes"}
	k3DMinorClass = FourCharCode('pzre'); { "3D Transitions"}
	kCompositorMinorClass = FourCharCode('comp'); { "Compositors"}
	kEffectsMinorClass = FourCharCode('fxfx'); { "Special Effects"}
	kMiscMinorClass = FourCharCode('misc'); { "Miscellaneous"}

{
   Effects can define a number of "preset" values which will be presented to the user
   in a simplified UI.  Each preset is an atom within the parameter description list
   and must have an atom ID from 1 going up sequentially.  Inside of this atom are three other
   atoms containing:
    1) the name of the preset as a Pascal string
    2) a preview picture for the preset, 86 x 64 pixels in size
    3) the ENTIRE set of parameter values needed to create a sample of this preset.
}
const
	kEffectPresetType = FourCharCode('peff');
	kPresetNameType = FourCharCode('pnam');
	kPresetNameID = 1;
	kPresetPreviewPictureType = FourCharCode('ppct');
	kPresetPreviewPictureID = 1;
	kPresetSettingsType = FourCharCode('psst');
	kPresetSettingsID = 1;

const
	kParameterDependencyName = FourCharCode('deep');
	kParameterDependencyID = 1;

const
	kParameterListDependsUponColorProfiles = FourCharCode('prof');
	kParameterListDependsUponFonts = FourCharCode('font');

type
	ParameterDependancyRecordPtr = ^ParameterDependancyRecord;
	ParameterDependancyRecord = record
		dependCount: SIGNEDLONG;
    depends: array [0..0] of OSType;
	end;
{
   enumeration list in container -- placed in root container, optional unless used by a
   parameter in the list
}
const
<<<<<<< HEAD
	kParameterEnumList			= FourCharCode('enum');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kParameterEnumList = FourCharCode('enum');
>>>>>>> origin/fixes_2.4

type
	EnumValuePairPtr = ^EnumValuePair;
	EnumValuePair = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		value: SIGNEDLONG;
		name: Str255;
	end;
type
	EnumListRecordPtr = ^EnumListRecord;
	EnumListRecord = record
		enumCount: SIGNEDLONG;              { number of enumeration items to follow}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		values: array [0..0] of EnumValuePair;              { values and names for them, packed }
=======
    values: array [0..0] of EnumValuePair;              { values and names for them, packed }
>>>>>>> graemeg/cpstrnew
=======
    values: array [0..0] of EnumValuePair;              { values and names for them, packed }
>>>>>>> graemeg/cpstrnew
=======
    values: array [0..0] of EnumValuePair;              { values and names for them, packed }
>>>>>>> graemeg/cpstrnew
=======
    values: array [0..0] of EnumValuePair;              { values and names for them, packed }
>>>>>>> origin/cpstrnew
	end;
{ atom type of parameter}
const
	kParameterAtomTypeAndID = FourCharCode('type');

const
	kNoAtom = FourCharCode('none'); { atom type for no data got/set}
	kAtomNoFlags = $00000000;
	kAtomNotInterpolated = $00000001; { atom can never be interpolated}
	kAtomInterpolateIsOptional = $00000002; { atom can be interpolated, but it is an advanced user operation}
	kAtomMayBeIndexed = $00000004; { more than one value of atom can exist with accending IDs (ie, lists of colors)}
=======
=======
>>>>>>> origin/fixes_2_2
		value:					SInt32;
		name:					Str255;
=======
		value: SIGNEDLONG;
		name: Str255;
>>>>>>> origin/fixes_2.4
	end;
type
	EnumListRecordPtr = ^EnumListRecord;
	EnumListRecord = record
		enumCount: SIGNEDLONG;              { number of enumeration items to follow}
    values: array [0..0] of EnumValuePair;              { values and names for them, packed }
	end;
{ atom type of parameter}
const
	kParameterAtomTypeAndID = FourCharCode('type');

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kNoAtom = FourCharCode('none'); { atom type for no data got/set}
	kAtomNoFlags = $00000000;
	kAtomNotInterpolated = $00000001; { atom can never be interpolated}
	kAtomInterpolateIsOptional = $00000002; { atom can be interpolated, but it is an advanced user operation}
	kAtomMayBeIndexed = $00000004; { more than one value of atom can exist with accending IDs (ie, lists of colors)}
>>>>>>> origin/fixes_2.4

type
	ParameterAtomTypeAndIDPtr = ^ParameterAtomTypeAndID;
	ParameterAtomTypeAndID = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		atomType: QTAtomType;               { type of atom this data comes from/goes into}
		atomID: QTAtomID;                 { ID of atom this data comes from/goes into}
		atomFlags: SIGNEDLONG;              { options for this atom}
		atomName: Str255;               { name of this value type}
<<<<<<< HEAD
	end;
{ optional specification of mapping between parameters and properties}
const
	kParameterProperty = FourCharCode('prop');

type
	ParameterProperty = record
		propertyClass: OSType;          { class to set for this property (0 for default which is specified by caller)}
		propertyID: OSType;             { id to set for this property (default is the atomType)}
	end;
{ data type of a parameter}
const
	kParameterDataType = FourCharCode('data');

const
	kParameterTypeDataLong = kTweenTypeLong; { integer value}
	kParameterTypeDataFixed = kTweenTypeFixed; { fixed point value}
	kParameterTypeDataRGBValue = kTweenTypeRGBColor; { RGBColor data}
	kParameterTypeDataDouble = kTweenTypeQTFloatDouble; { IEEE 64 bit floating point value}
	kParameterTypeDataText = FourCharCode('text'); { editable text item}
	kParameterTypeDataEnum = FourCharCode('enum'); { enumerated lookup value}
	kParameterTypeDataBitField = FourCharCode('bool'); { bit field value (something that holds boolean(s))}
	kParameterTypeDataImage = FourCharCode('imag'); { reference to an image via Picture data}
=======
=======
>>>>>>> origin/fixes_2_2
		atomType:				QTAtomType;								{  type of atom this data comes from/goes into }
		atomID:					QTAtomID;								{  ID of atom this data comes from/goes into }
		atomFlags:				SInt32;								{  options for this atom }
		atomName:				Str255;									{  name of this value type }
=======
>>>>>>> origin/fixes_2.4
	end;
{ optional specification of mapping between parameters and properties}
const
	kParameterProperty = FourCharCode('prop');

type
	ParameterProperty = record
		propertyClass: OSType;          { class to set for this property (0 for default which is specified by caller)}
		propertyID: OSType;             { id to set for this property (default is the atomType)}
	end;
{ data type of a parameter}
const
	kParameterDataType = FourCharCode('data');

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kParameterTypeDataLong = kTweenTypeLong; { integer value}
	kParameterTypeDataFixed = kTweenTypeFixed; { fixed point value}
	kParameterTypeDataRGBValue = kTweenTypeRGBColor; { RGBColor data}
	kParameterTypeDataDouble = kTweenTypeQTFloatDouble; { IEEE 64 bit floating point value}
	kParameterTypeDataText = FourCharCode('text'); { editable text item}
	kParameterTypeDataEnum = FourCharCode('enum'); { enumerated lookup value}
	kParameterTypeDataBitField = FourCharCode('bool'); { bit field value (something that holds boolean(s))}
	kParameterTypeDataImage = FourCharCode('imag'); { reference to an image via Picture data}
>>>>>>> origin/fixes_2.4

type
	ParameterDataTypePtr = ^ParameterDataType;
	ParameterDataType = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		dataType: OSType;               { type of data this item is stored as}
	end;
{
   alternate (optional) data type -- main data type always required.  
   Must be modified or deleted when modifying main data type.
   Main data type must be modified when alternate is modified.
}
const
	kParameterAlternateDataType = FourCharCode('alt1');
	kParameterTypeDataColorValue = FourCharCode('cmlr'); { CMColor data (supported on machines with ColorSync)}
	kParameterTypeDataCubic = FourCharCode('cubi'); { cubic bezier(s) (no built-in support)}
	kParameterTypeDataNURB = FourCharCode('nurb'); { nurb(s) (no built-in support)}
=======
=======
>>>>>>> origin/fixes_2_2
		dataType:				OSType;									{  type of data this item is stored as }
=======
		dataType: OSType;               { type of data this item is stored as}
>>>>>>> origin/fixes_2.4
	end;
{
   alternate (optional) data type -- main data type always required.  
   Must be modified or deleted when modifying main data type.
   Main data type must be modified when alternate is modified.
}
const
<<<<<<< HEAD
	kParameterAlternateDataType	= FourCharCode('alt1');
	kParameterTypeDataColorValue = FourCharCode('cmlr');						{  CMColor data (supported on machines with ColorSync) }
	kParameterTypeDataCubic		= FourCharCode('cubi');						{  cubic bezier(s) (no built-in support) }
	kParameterTypeDataNURB		= FourCharCode('nurb');						{  nurb(s) (no built-in support) }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kParameterAlternateDataType = FourCharCode('alt1');
	kParameterTypeDataColorValue = FourCharCode('cmlr'); { CMColor data (supported on machines with ColorSync)}
	kParameterTypeDataCubic = FourCharCode('cubi'); { cubic bezier(s) (no built-in support)}
	kParameterTypeDataNURB = FourCharCode('nurb'); { nurb(s) (no built-in support)}
>>>>>>> origin/fixes_2.4

type
	ParameterAlternateDataEntryPtr = ^ParameterAlternateDataEntry;
	ParameterAlternateDataEntry = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		dataType: OSType;               { type of data this item is stored as}
		alternateAtom: QTAtomType;          { where to store}
	end;
type
	ParameterAlternateDataTypePtr = ^ParameterAlternateDataType;
	ParameterAlternateDataType = record
		numEntries: SIGNEDLONG;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		entries: array [0..0] of ParameterAlternateDataEntry;
=======
    entries: array [0..0] of ParameterAlternateDataEntry;
>>>>>>> graemeg/cpstrnew
=======
    entries: array [0..0] of ParameterAlternateDataEntry;
>>>>>>> graemeg/cpstrnew
=======
    entries: array [0..0] of ParameterAlternateDataEntry;
>>>>>>> graemeg/cpstrnew
=======
    entries: array [0..0] of ParameterAlternateDataEntry;
>>>>>>> origin/cpstrnew
	end;
{ legal values for the parameter}
const
	kParameterDataRange = FourCharCode('rang');

const
	kNoMinimumLongFixed = $7FFFFFFF; { ignore minimum/maxiumum values}
	kNoMaximumLongFixed = $80000000;
	kNoScaleLongFixed = 0;    { don't perform any scaling of value}
	kNoPrecision = -1;  { allow as many digits as format}

{ 'text'}
type
	StringRangeRecordPtr = ^StringRangeRecord;
	StringRangeRecord = record
		maxChars: SIGNEDLONG;               { maximum length of string}
		maxLines: SIGNEDLONG;               { number of editing lines to use (1 typical, 0 to default)}
	end;
{ 'long'}
type
	LongRangeRecordPtr = ^LongRangeRecord;
	LongRangeRecord = record
		minValue: SIGNEDLONG;               { no less than this}
		maxValue: SIGNEDLONG;               { no more than this}
		scaleValue: SIGNEDLONG;             { muliply content by this going in, divide going out}
		precisionDigits: SIGNEDLONG;        { # digits of precision when editing via typing}
	end;
{ 'enum'}
type
	EnumRangeRecordPtr = ^EnumRangeRecord;
	EnumRangeRecord = record
		enumID: SIGNEDLONG;                 { 'enum' list in root container to search within}
	end;
{ 'fixd'}
type
	FixedRangeRecordPtr = ^FixedRangeRecord;
	FixedRangeRecord = record
		minValue: Fixed;               { no less than this}
		maxValue: Fixed;               { no more than this}
		scaleValue: Fixed;             { muliply content by this going in, divide going out}
		precisionDigits: SIGNEDLONG;        { # digits of precision when editing via typing}
	end;
{ 'doub'}


{$push}
{$r-}
{$q-}
const
  kNoMinimumDouble=0.0/0.0;                   { ignore minimum/maxiumum values }
  kNoMaximumDouble=0.0/0.0;
{$pop}
const
  kNoScaleDouble = 0.0;                     { don't perform any scaling of value }
type
	DoubleRangeRecordPtr = ^DoubleRangeRecord;
	DoubleRangeRecord = record
		minValue: QTFloatDouble;           { no less than this }
		maxValue: QTFloatDouble;           { no more than this }
		scaleValue: QTFloatDouble;         { muliply content by this going in, divide going out }
		precisionDigits: SIGNEDLONG;    { # digits of precision when editing via typing }
	end;
    
{ 'bool'   }
type
	BooleanRangeRecordPtr = ^BooleanRangeRecord;
	BooleanRangeRecord = record
		maskValue: SIGNEDLONG;              { value to mask on/off to set/clear the boolean}
	end;
{ 'rgb '}
type
	RGBRangeRecordPtr = ^RGBRangeRecord;
	RGBRangeRecord = record
		minColor: RGBColor;
		maxColor: RGBColor;
	end;
{ 'imag'}
const
	kParameterImageNoFlags = 0;
	kParameterImageIsPreset = 1;

const
	kStandardPresetGroup = FourCharCode('pset');
=======
=======
>>>>>>> origin/fixes_2_2
		dataType:				OSType;									{  type of data this item is stored as }
		alternateAtom:			QTAtomType;								{  where to store }
=======
		dataType: OSType;               { type of data this item is stored as}
		alternateAtom: QTAtomType;          { where to store}
>>>>>>> origin/fixes_2.4
	end;
type
	ParameterAlternateDataTypePtr = ^ParameterAlternateDataType;
	ParameterAlternateDataType = record
		numEntries: SIGNEDLONG;
    entries: array [0..0] of ParameterAlternateDataEntry;
	end;
{ legal values for the parameter}
const
	kParameterDataRange = FourCharCode('rang');

const
	kNoMinimumLongFixed = $7FFFFFFF; { ignore minimum/maxiumum values}
	kNoMaximumLongFixed = $80000000;
	kNoScaleLongFixed = 0;    { don't perform any scaling of value}
	kNoPrecision = -1;  { allow as many digits as format}

{ 'text'}
type
	StringRangeRecordPtr = ^StringRangeRecord;
	StringRangeRecord = record
		maxChars: SIGNEDLONG;               { maximum length of string}
		maxLines: SIGNEDLONG;               { number of editing lines to use (1 typical, 0 to default)}
	end;
{ 'long'}
type
	LongRangeRecordPtr = ^LongRangeRecord;
	LongRangeRecord = record
		minValue: SIGNEDLONG;               { no less than this}
		maxValue: SIGNEDLONG;               { no more than this}
		scaleValue: SIGNEDLONG;             { muliply content by this going in, divide going out}
		precisionDigits: SIGNEDLONG;        { # digits of precision when editing via typing}
	end;
{ 'enum'}
type
	EnumRangeRecordPtr = ^EnumRangeRecord;
	EnumRangeRecord = record
		enumID: SIGNEDLONG;                 { 'enum' list in root container to search within}
	end;
{ 'fixd'}
type
	FixedRangeRecordPtr = ^FixedRangeRecord;
	FixedRangeRecord = record
		minValue: Fixed;               { no less than this}
		maxValue: Fixed;               { no more than this}
		scaleValue: Fixed;             { muliply content by this going in, divide going out}
		precisionDigits: SIGNEDLONG;        { # digits of precision when editing via typing}
	end;
{ 'doub'}


{$push}
{$r-}
{$q-}
const
  kNoMinimumDouble=0.0/0.0;                   { ignore minimum/maxiumum values }
  kNoMaximumDouble=0.0/0.0;
{$pop}
const
  kNoScaleDouble = 0.0;                     { don't perform any scaling of value }
type
	DoubleRangeRecordPtr = ^DoubleRangeRecord;
	DoubleRangeRecord = record
		minValue: QTFloatDouble;           { no less than this }
		maxValue: QTFloatDouble;           { no more than this }
		scaleValue: QTFloatDouble;         { muliply content by this going in, divide going out }
		precisionDigits: SIGNEDLONG;    { # digits of precision when editing via typing }
	end;
    
{ 'bool'   }
type
	BooleanRangeRecordPtr = ^BooleanRangeRecord;
	BooleanRangeRecord = record
		maskValue: SIGNEDLONG;              { value to mask on/off to set/clear the boolean}
	end;
{ 'rgb '}
type
	RGBRangeRecordPtr = ^RGBRangeRecord;
	RGBRangeRecord = record
		minColor: RGBColor;
		maxColor: RGBColor;
	end;
{ 'imag'}
const
	kParameterImageNoFlags = 0;
	kParameterImageIsPreset = 1;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kStandardPresetGroup = FourCharCode('pset');
>>>>>>> origin/fixes_2.4

type
	ImageRangeRecordPtr = ^ImageRangeRecord;
	ImageRangeRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		imageFlags: SIGNEDLONG;
		fileType: OSType;               { file type to contain the preset group (normally kStandardPresetGroup)}
		replacedAtoms: SIGNEDLONG;          { # atoms at this level replaced by this preset group}
	end;
{ union of all of the above}

type
	ParameterRangeRecord = record
	  case SInt16 of
	    0: (
        longRange: LongRangeRecord;
        );
      1: (
        enumRange: EnumRangeRecord;
        );
      2: (
        fixedRange: FixedRangeRecord;
        );
      3: (
        doubleRange: DoubleRangeRecord;
        );
      4: (
        stringRange: StringRangeRecord;
        );
      5: (
        booleanRange: BooleanRangeRecord;
        );
      6: (
        rgbRange: RGBRangeRecord;
        );
      7: (
        imageRange: ImageRangeRecord;
        );
	end;
  
{ UI behavior of a parameter}
const
	kParameterDataBehavior = FourCharCode('ditl');

const
{ items edited via typing}
	kParameterItemEditText = FourCharCode('edit'); { edit text box}
	kParameterItemEditLong = FourCharCode('long'); { long number editing box}
	kParameterItemEditFixed = FourCharCode('fixd'); { fixed point number editing box}
	kParameterItemEditDouble = FourCharCode('doub'); { double number editing box}
                                        { items edited via control(s)}
	kParameterItemPopUp = FourCharCode('popu'); { pop up value for enum types}
	kParameterItemRadioCluster = FourCharCode('radi'); { radio cluster for enum types}
	kParameterItemCheckBox = FourCharCode('chex'); { check box for booleans}
	kParameterItemControl = FourCharCode('cntl'); { item controlled via a standard control of some type}
                                        { special user items}
	kParameterItemLine = FourCharCode('line'); { line}
	kParameterItemColorPicker = FourCharCode('pick'); { color swatch & picker}
	kParameterItemGroupDivider = FourCharCode('divi'); { start of a new group of items}
	kParameterItemStaticText = FourCharCode('stat'); { display "parameter name" as static text}
	kParameterItemDragImage = FourCharCode('imag'); { allow image display, along with drag and drop}
                                        { flags valid for lines and groups}
	kGraphicsNoFlags = $00000000; { no options for graphics}
	kGraphicsFlagsGray = $00000001; { draw lines with gray}
                                        { flags valid for groups}
	kGroupNoFlags = $00000000; { no options for group -- may be combined with graphics options             }
	kGroupAlignText = $00010000; { edit text items in group have the same size}
	kGroupSurroundBox = $00020000; { group should be surrounded with a box}
	kGroupMatrix = $00040000; { side-by-side arrangement of group is okay}
	kGroupNoName = $00080000; { name of group should not be displayed above box}
                                        { flags valid for popup/radiocluster/checkbox/control}
	kDisableControl = $00000001;
	kDisableWhenNotEqual = $00000000 + kDisableControl;
	kDisableWhenEqual = $00000010 + kDisableControl;
	kDisableWhenLessThan = $00000020 + kDisableControl;
	kDisableWhenGreaterThan = $00000030 + kDisableControl; { flags valid for controls}
	kCustomControl = $00100000; { flags valid for popups}
	kPopupStoreAsString = $00010000;

type
	ControlBehaviors = record
		groupID: QTAtomID;                { group under control of this item}
		controlValue: SIGNEDLONG;           { control value for comparison purposes}
		customType: QTAtomType;             { custom type identifier, for kCustomControl}
		customID: QTAtomID;               { custom type ID, for kCustomControl}
	end;
type
	ParameterDataBehaviorPtr = ^ParameterDataBehavior;
	ParameterDataBehavior = record
		behaviorType: OSType;
		behaviorFlags: SIGNEDLONG;
		case SInt16 of
			0: (
				controls: ControlBehaviors;
				);
	end;
{ higher level purpose of a parameter or set of parameters}
const
	kParameterDataUsage = FourCharCode('use ');

const
	kParameterUsagePixels = FourCharCode('pixl');
	kParameterUsageRectangle = FourCharCode('rect');
	kParameterUsagePoint = FourCharCode('xy  ');
	kParameterUsage3DPoint = FourCharCode('xyz ');
	kParameterUsageDegrees = FourCharCode('degr');
	kParameterUsageRadians = FourCharCode('rads');
	kParameterUsagePercent = FourCharCode('pcnt');
	kParameterUsageSeconds = FourCharCode('secs');
	kParameterUsageMilliseconds = FourCharCode('msec');
	kParameterUsageMicroseconds = FourCharCode('µsec');
	kParameterUsage3by3Matrix = FourCharCode('3by3');
	kParameterUsageCircularDegrees = FourCharCode('degc');
	kParameterUsageCircularRadians = FourCharCode('radc');

type
	ParameterDataUsagePtr = ^ParameterDataUsage;
	ParameterDataUsage = record
		usageType: OSType;              { higher level purpose of the data or group}
	end;
{ default value(s) for a parameter}
const
	kParameterDataDefaultItem = FourCharCode('dflt');

	{	 atoms that help to fill in data within the info window 	}
	kParameterInfoLongName = FourCharCode('©nam');
	kParameterInfoCopyright = FourCharCode('©cpy');
	kParameterInfoDescription	= FourCharCode('©inf');
	kParameterInfoWindowTitle	= FourCharCode('©wnt');
	kParameterInfoPicture = FourCharCode('©pix');
	kParameterInfoManufacturer = FourCharCode('©man');
	kParameterInfoIDs = 1;

	{ flags for ImageCodecValidateParameters }
	kParameterValidationNoFlags = $00000000;
=======
=======
>>>>>>> origin/fixes_2_2
		imageFlags:				SInt32;
		fileType:				OSType;									{  file type to contain the preset group (normally kStandardPresetGroup) }
		replacedAtoms:			SInt32;								{  # atoms at this level replaced by this preset group }
=======
		imageFlags: SIGNEDLONG;
		fileType: OSType;               { file type to contain the preset group (normally kStandardPresetGroup)}
		replacedAtoms: SIGNEDLONG;          { # atoms at this level replaced by this preset group}
>>>>>>> origin/fixes_2.4
	end;
{ union of all of the above}

type
	ParameterRangeRecord = record
	  case SInt16 of
	    0: (
        longRange: LongRangeRecord;
        );
      1: (
        enumRange: EnumRangeRecord;
        );
      2: (
        fixedRange: FixedRangeRecord;
        );
      3: (
        doubleRange: DoubleRangeRecord;
        );
      4: (
        stringRange: StringRangeRecord;
        );
      5: (
        booleanRange: BooleanRangeRecord;
        );
      6: (
        rgbRange: RGBRangeRecord;
        );
      7: (
        imageRange: ImageRangeRecord;
        );
	end;
  
{ UI behavior of a parameter}
const
	kParameterDataBehavior = FourCharCode('ditl');

const
{ items edited via typing}
	kParameterItemEditText = FourCharCode('edit'); { edit text box}
	kParameterItemEditLong = FourCharCode('long'); { long number editing box}
	kParameterItemEditFixed = FourCharCode('fixd'); { fixed point number editing box}
	kParameterItemEditDouble = FourCharCode('doub'); { double number editing box}
                                        { items edited via control(s)}
	kParameterItemPopUp = FourCharCode('popu'); { pop up value for enum types}
	kParameterItemRadioCluster = FourCharCode('radi'); { radio cluster for enum types}
	kParameterItemCheckBox = FourCharCode('chex'); { check box for booleans}
	kParameterItemControl = FourCharCode('cntl'); { item controlled via a standard control of some type}
                                        { special user items}
	kParameterItemLine = FourCharCode('line'); { line}
	kParameterItemColorPicker = FourCharCode('pick'); { color swatch & picker}
	kParameterItemGroupDivider = FourCharCode('divi'); { start of a new group of items}
	kParameterItemStaticText = FourCharCode('stat'); { display "parameter name" as static text}
	kParameterItemDragImage = FourCharCode('imag'); { allow image display, along with drag and drop}
                                        { flags valid for lines and groups}
	kGraphicsNoFlags = $00000000; { no options for graphics}
	kGraphicsFlagsGray = $00000001; { draw lines with gray}
                                        { flags valid for groups}
	kGroupNoFlags = $00000000; { no options for group -- may be combined with graphics options             }
	kGroupAlignText = $00010000; { edit text items in group have the same size}
	kGroupSurroundBox = $00020000; { group should be surrounded with a box}
	kGroupMatrix = $00040000; { side-by-side arrangement of group is okay}
	kGroupNoName = $00080000; { name of group should not be displayed above box}
                                        { flags valid for popup/radiocluster/checkbox/control}
	kDisableControl = $00000001;
	kDisableWhenNotEqual = $00000000 + kDisableControl;
	kDisableWhenEqual = $00000010 + kDisableControl;
	kDisableWhenLessThan = $00000020 + kDisableControl;
	kDisableWhenGreaterThan = $00000030 + kDisableControl; { flags valid for controls}
	kCustomControl = $00100000; { flags valid for popups}
	kPopupStoreAsString = $00010000;

type
	ControlBehaviors = record
		groupID: QTAtomID;                { group under control of this item}
		controlValue: SIGNEDLONG;           { control value for comparison purposes}
		customType: QTAtomType;             { custom type identifier, for kCustomControl}
		customID: QTAtomID;               { custom type ID, for kCustomControl}
	end;
type
	ParameterDataBehaviorPtr = ^ParameterDataBehavior;
	ParameterDataBehavior = record
		behaviorType: OSType;
		behaviorFlags: SIGNEDLONG;
		case SInt16 of
			0: (
				controls: ControlBehaviors;
				);
	end;
{ higher level purpose of a parameter or set of parameters}
const
	kParameterDataUsage = FourCharCode('use ');

const
	kParameterUsagePixels = FourCharCode('pixl');
	kParameterUsageRectangle = FourCharCode('rect');
	kParameterUsagePoint = FourCharCode('xy  ');
	kParameterUsage3DPoint = FourCharCode('xyz ');
	kParameterUsageDegrees = FourCharCode('degr');
	kParameterUsageRadians = FourCharCode('rads');
	kParameterUsagePercent = FourCharCode('pcnt');
	kParameterUsageSeconds = FourCharCode('secs');
	kParameterUsageMilliseconds = FourCharCode('msec');
	kParameterUsageMicroseconds = FourCharCode('µsec');
	kParameterUsage3by3Matrix = FourCharCode('3by3');
	kParameterUsageCircularDegrees = FourCharCode('degc');
	kParameterUsageCircularRadians = FourCharCode('radc');

type
	ParameterDataUsagePtr = ^ParameterDataUsage;
	ParameterDataUsage = record
		usageType: OSType;              { higher level purpose of the data or group}
	end;
{ default value(s) for a parameter}
const
<<<<<<< HEAD
	kParameterDataDefaultItem	= FourCharCode('dflt');

	{	 atoms that help to fill in data within the info window 	}
	kParameterInfoLongName		= FourCharCode('©nam');
	kParameterInfoCopyright		= FourCharCode('©cpy');
	kParameterInfoDescription	= FourCharCode('©inf');
	kParameterInfoWindowTitle	= FourCharCode('©wnt');
	kParameterInfoPicture		= FourCharCode('©pix');
	kParameterInfoManufacturer	= FourCharCode('©man');
	kParameterInfoIDs			= 1;

<<<<<<< HEAD
	{	 flags for ImageCodecValidateParameters 	}
	kParameterValidationNoFlags	= $00000000;
>>>>>>> graemeg/fixes_2_2
	kParameterValidationFinalValidation = $00000001;


type
<<<<<<< HEAD
	QTParameterValidationOptions = SIGNEDLONG;
{ QTAtomTypes for atoms in image compressor settings containers}
const
	kImageCodecSettingsFieldCount = FourCharCode('fiel'); { Number of fields (UInt8) }
	kImageCodecSettingsFieldOrdering = FourCharCode('fdom'); { Ordering of fields (UInt8)}
=======
	QTParameterValidationOptions		= SInt32;
	{  QTAtomTypes for atoms in image compressor settings containers }

const
	kImageCodecSettingsFieldCount = FourCharCode('fiel');						{  Number of fields (UInt8)  }
	kImageCodecSettingsFieldOrdering = FourCharCode('fdom');					{  Ordering of fields (UInt8) }
>>>>>>> graemeg/fixes_2_2
	kImageCodecSettingsFieldOrderingF1F2 = 1;
	kImageCodecSettingsFieldOrderingF2F1 = 2;


<<<<<<< HEAD
{
 *  Summary:
 *    Additional Image Description Extensions
 }
const
{
   * Image description extension describing the color properties.
   }
	kColorInfoImageDescriptionExtension = FourCharCode('colr');

  {
   * Image description extension describing the pixel aspect ratio.
   }
	kPixelAspectRatioImageDescriptionExtension = FourCharCode('pasp'); { big-endian PixelAspectRatioImageDescriptionExtension }

  {
   * Image description extension describing the clean aperture.
   }
	kCleanApertureImageDescriptionExtension = FourCharCode('clap'); { big-endian CleanApertureImageDescriptionExtension }

  {
   * Specifies the offset in bytes from the start of one pixel row to
   * the next. Only valid for chunky pixel formats. If present, this
   * image description extension overrides other conventions for
   * calculating rowBytes.
   }
	kQTRowBytesImageDescriptionExtension = FourCharCode('rowb'); { big-endian SInt32 }

{ Color Info Image Description Extension types}
const
	kVideoColorInfoImageDescriptionExtensionType = FourCharCode('nclc'); { For video color descriptions (defined below)    }
	kICCProfileColorInfoImageDescriptionExtensionType = FourCharCode('prof'); { For ICC Profile color descriptions (not defined here)}


{ Video Color Info Image Description Extensions}
type
	NCLCColorInfoImageDescriptionExtension = record
		colorParamType: OSType;         { Type of color parameter 'nclc'               }
		primaries: UInt16;              { CIE 1931 xy chromaticity coordinates          }
		transferFunction: UInt16;       { Nonlinear transfer function from RGB to ErEgEb }
		matrix: UInt16;                 { Matrix from ErEgEb to EyEcbEcr           }
	end;
{ Primaries}
const
	kQTPrimaries_ITU_R709_2 = 1;    { ITU-R BT.709-2, SMPTE 274M-1995, and SMPTE 296M-1997 }
	kQTPrimaries_Unknown = 2;    { Unknown }
	kQTPrimaries_EBU_3213 = 5;    { EBU Tech. 3213 (1981) }
	kQTPrimaries_SMPTE_C = 6;     { SMPTE C Primaries from SMPTE RP 145-1993 }

{ Transfer Function}
const
	kQTTransferFunction_ITU_R709_2 = 1;   { Recommendation ITU-R BT.709-2, SMPTE 274M-1995, SMPTE 296M-1997, SMPTE 293M-1996 and SMPTE 170M-1994 }
	kQTTransferFunction_Unknown = 2;    { Unknown }
	kQTTransferFunction_SMPTE_240M_1995 = 7; { SMPTE 240M-1995 and interim color implementation of SMPTE 274M-1995 }

{ Matrix}
const
	kQTMatrix_ITU_R_709_2 = 1;    { Recommendation ITU-R BT.709-2 (1125/60/2:1 only), SMPTE 274M-1995 and SMPTE 296M-1997 }
	kQTMatrix_Unknown = 2;    { Unknown }
	kQTMatrix_ITU_R_601_4 = 6;    { Recommendation ITU-R BT.601-4, Recommendation ITU-R BT.470-4 System B and G, SMPTE 170M-1994 and SMPTE 293M-1996 }
	kQTMatrix_SMPTE_240M_1995 = 7;     { SMPTE 240M-1995 and interim color implementation of SMPTE 274M-1995 }


{ Field/Frame Info Image Description (this remaps to FieldInfoImageDescriptionExtension)}
type
	FieldInfoImageDescriptionExtension2Ptr = ^FieldInfoImageDescriptionExtension2;
	FieldInfoImageDescriptionExtension2 = record
		fields: UInt8;
		detail: UInt8;
	end;
const
	kQTFieldsProgressiveScan = 1;
	kQTFieldsInterlaced = 2;

const
	kQTFieldDetailUnknown = 0;
=======
=======
	{	 flags for ImageCodecValidateParameters 	}
	kParameterValidationNoFlags	= $00000000;
=======
	kParameterDataDefaultItem = FourCharCode('dflt');

	{	 atoms that help to fill in data within the info window 	}
	kParameterInfoLongName = FourCharCode('©nam');
	kParameterInfoCopyright = FourCharCode('©cpy');
	kParameterInfoDescription	= FourCharCode('©inf');
	kParameterInfoWindowTitle	= FourCharCode('©wnt');
	kParameterInfoPicture = FourCharCode('©pix');
	kParameterInfoManufacturer = FourCharCode('©man');
	kParameterInfoIDs = 1;

	{ flags for ImageCodecValidateParameters }
	kParameterValidationNoFlags = $00000000;
>>>>>>> origin/fixes_2.4
	kParameterValidationFinalValidation = $00000001;


type
	QTParameterValidationOptions = SIGNEDLONG;
{ QTAtomTypes for atoms in image compressor settings containers}
const
	kImageCodecSettingsFieldCount = FourCharCode('fiel'); { Number of fields (UInt8) }
	kImageCodecSettingsFieldOrdering = FourCharCode('fdom'); { Ordering of fields (UInt8)}
	kImageCodecSettingsFieldOrderingF1F2 = 1;
	kImageCodecSettingsFieldOrderingF2F1 = 2;


<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
	{  Additional Image Description Extensions }
	kColorInfoImageDescriptionExtension = FourCharCode('colr');				{  image description extension describing the color properties     }
	kPixelAspectRatioImageDescriptionExtension = FourCharCode('pasp');		{  image description extension describing the pixel aspect ratio }
	kCleanApertureImageDescriptionExtension = FourCharCode('clap');			{  image description extension describing the pixel aspect ratio }


	{  Color Info Image Description Extension types }
	kVideoColorInfoImageDescriptionExtensionType = FourCharCode('nclc');		{  For video color descriptions (defined below)     }
	kICCProfileColorInfoImageDescriptionExtensionType = FourCharCode('prof');	{  For ICC Profile color descriptions (not defined here) }

=======
{
 *  Summary:
 *    Additional Image Description Extensions
 }
const
{
   * Image description extension describing the color properties.
   }
	kColorInfoImageDescriptionExtension = FourCharCode('colr');

  {
   * Image description extension describing the pixel aspect ratio.
   }
	kPixelAspectRatioImageDescriptionExtension = FourCharCode('pasp'); { big-endian PixelAspectRatioImageDescriptionExtension }

  {
   * Image description extension describing the clean aperture.
   }
	kCleanApertureImageDescriptionExtension = FourCharCode('clap'); { big-endian CleanApertureImageDescriptionExtension }

  {
   * Specifies the offset in bytes from the start of one pixel row to
   * the next. Only valid for chunky pixel formats. If present, this
   * image description extension overrides other conventions for
   * calculating rowBytes.
   }
	kQTRowBytesImageDescriptionExtension = FourCharCode('rowb'); { big-endian SInt32 }

{ Color Info Image Description Extension types}
const
	kVideoColorInfoImageDescriptionExtensionType = FourCharCode('nclc'); { For video color descriptions (defined below)    }
	kICCProfileColorInfoImageDescriptionExtensionType = FourCharCode('prof'); { For ICC Profile color descriptions (not defined here)}
>>>>>>> origin/fixes_2.4


{ Video Color Info Image Description Extensions}
type
	NCLCColorInfoImageDescriptionExtension = record
		colorParamType: OSType;         { Type of color parameter 'nclc'               }
		primaries: UInt16;              { CIE 1931 xy chromaticity coordinates          }
		transferFunction: UInt16;       { Nonlinear transfer function from RGB to ErEgEb }
		matrix: UInt16;                 { Matrix from ErEgEb to EyEcbEcr           }
	end;
{ Primaries}
const
	kQTPrimaries_ITU_R709_2 = 1;    { ITU-R BT.709-2, SMPTE 274M-1995, and SMPTE 296M-1997 }
	kQTPrimaries_Unknown = 2;    { Unknown }
	kQTPrimaries_EBU_3213 = 5;    { EBU Tech. 3213 (1981) }
	kQTPrimaries_SMPTE_C = 6;     { SMPTE C Primaries from SMPTE RP 145-1993 }

{ Transfer Function}
const
	kQTTransferFunction_ITU_R709_2 = 1;   { Recommendation ITU-R BT.709-2, SMPTE 274M-1995, SMPTE 296M-1997, SMPTE 293M-1996 and SMPTE 170M-1994 }
	kQTTransferFunction_Unknown = 2;    { Unknown }
	kQTTransferFunction_SMPTE_240M_1995 = 7; { SMPTE 240M-1995 and interim color implementation of SMPTE 274M-1995 }

{ Matrix}
const
	kQTMatrix_ITU_R_709_2 = 1;    { Recommendation ITU-R BT.709-2 (1125/60/2:1 only), SMPTE 274M-1995 and SMPTE 296M-1997 }
	kQTMatrix_Unknown = 2;    { Unknown }
	kQTMatrix_ITU_R_601_4 = 6;    { Recommendation ITU-R BT.601-4, Recommendation ITU-R BT.470-4 System B and G, SMPTE 170M-1994 and SMPTE 293M-1996 }
	kQTMatrix_SMPTE_240M_1995 = 7;     { SMPTE 240M-1995 and interim color implementation of SMPTE 274M-1995 }


{ Field/Frame Info Image Description (this remaps to FieldInfoImageDescriptionExtension)}
type
	FieldInfoImageDescriptionExtension2Ptr = ^FieldInfoImageDescriptionExtension2;
	FieldInfoImageDescriptionExtension2 = record
		fields: UInt8;
		detail: UInt8;
	end;
const
	kQTFieldsProgressiveScan = 1;
	kQTFieldsInterlaced = 2;

<<<<<<< HEAD
	kQTFieldDetailUnknown		= 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kQTFieldDetailUnknown = 0;
>>>>>>> origin/fixes_2.4
	kQTFieldDetailTemporalTopFirst = 1;
	kQTFieldDetailTemporalBottomFirst = 6;
	kQTFieldDetailSpatialFirstLineEarly = 9;
	kQTFieldDetailSpatialFirstLineLate = 14;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Pixel Aspect Ratio Image Description Extensions}
type
	PixelAspectRatioImageDescriptionExtensionPtr = ^PixelAspectRatioImageDescriptionExtension;
	PixelAspectRatioImageDescriptionExtension = record
		hSpacing: UInt32;               { Horizontal Spacing }
		vSpacing: UInt32;               { Vertical Spacing }
	end;
{ Clean Aperture Image Description Extensions}
type
	CleanApertureImageDescriptionExtensionPtr = ^CleanApertureImageDescriptionExtension;
	CleanApertureImageDescriptionExtension = record
		cleanApertureWidthN: UInt32;    { width of clean aperture, numerator, denominator }
		cleanApertureWidthD: UInt32;
		cleanApertureHeightN: UInt32;   { height of clean aperture, numerator, denominator}
		cleanApertureHeightD: UInt32;
		horizOffN: SInt32;              { horizontal offset of clean aperture center minus (width-1)/2, numerator, denominator }
		horizOffD: UInt32;
		vertOffN: SInt32;               { vertical offset of clean aperture center minus (height-1)/2, numerator, denominator }
		vertOffD: UInt32;
	end;
type
	ImageCodecMPDrawBandProcPtr = function( refcon: UnivPtr; var drp: ImageSubCodecDecompressRecord ): ComponentResult;
	ImageCodecMPDrawBandUPP = ImageCodecMPDrawBandProcPtr;
{
 *  NewImageCodecMPDrawBandUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewImageCodecMPDrawBandUPP( userRoutine: ImageCodecMPDrawBandProcPtr ): ImageCodecMPDrawBandUPP; external name '_NewImageCodecMPDrawBandUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
	{  Pixel Aspect Ratio Image Description Extensions }

=======
{ Pixel Aspect Ratio Image Description Extensions}
>>>>>>> origin/fixes_2.4
type
	PixelAspectRatioImageDescriptionExtensionPtr = ^PixelAspectRatioImageDescriptionExtension;
	PixelAspectRatioImageDescriptionExtension = record
		hSpacing: UInt32;               { Horizontal Spacing }
		vSpacing: UInt32;               { Vertical Spacing }
	end;
{ Clean Aperture Image Description Extensions}
type
	CleanApertureImageDescriptionExtensionPtr = ^CleanApertureImageDescriptionExtension;
	CleanApertureImageDescriptionExtension = record
		cleanApertureWidthN: UInt32;    { width of clean aperture, numerator, denominator }
		cleanApertureWidthD: UInt32;
		cleanApertureHeightN: UInt32;   { height of clean aperture, numerator, denominator}
		cleanApertureHeightD: UInt32;
		horizOffN: SInt32;              { horizontal offset of clean aperture center minus (width-1)/2, numerator, denominator }
		horizOffD: UInt32;
		vertOffN: SInt32;               { vertical offset of clean aperture center minus (height-1)/2, numerator, denominator }
		vertOffD: UInt32;
	end;
type
	ImageCodecMPDrawBandProcPtr = function( refcon: UnivPtr; var drp: ImageSubCodecDecompressRecord ): ComponentResult;
	ImageCodecMPDrawBandUPP = ImageCodecMPDrawBandProcPtr;
{
 *  NewImageCodecMPDrawBandUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewImageCodecMPDrawBandUPP( userRoutine: ImageCodecMPDrawBandProcPtr ): ImageCodecMPDrawBandUPP; external name '_NewImageCodecMPDrawBandUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{$ifc TYPED_FUNCTION_POINTERS}
	ImageCodecMPDrawBandProcPtr = function(refcon: UnivPtr; var drp: ImageSubCodecDecompressRecord): ComponentResult;
{$elsec}
	ImageCodecMPDrawBandProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	ImageCodecMPDrawBandUPP = ^SInt32; { an opaque UPP }
{$elsec}
	ImageCodecMPDrawBandUPP = UniversalProcPtr;
{$endc}	

const
	uppImageCodecMPDrawBandProcInfo = $000003F0;
	{
	 *  NewImageCodecMPDrawBandUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewImageCodecMPDrawBandUPP(userRoutine: ImageCodecMPDrawBandProcPtr): ImageCodecMPDrawBandUPP; external name '_NewImageCodecMPDrawBandUPP'; { old name was NewImageCodecMPDrawBandProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  DisposeImageCodecMPDrawBandUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeImageCodecMPDrawBandUPP( userUPP: ImageCodecMPDrawBandUPP ); external name '_DisposeImageCodecMPDrawBandUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeImageCodecMPDrawBandUPP(userUPP: ImageCodecMPDrawBandUPP); external name '_DisposeImageCodecMPDrawBandUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeImageCodecMPDrawBandUPP( userUPP: ImageCodecMPDrawBandUPP ); external name '_DisposeImageCodecMPDrawBandUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeImageCodecMPDrawBandUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeImageCodecMPDrawBandUPP( refcon: UnivPtr; var drp: ImageSubCodecDecompressRecord; userUPP: ImageCodecMPDrawBandUPP ): ComponentResult; external name '_InvokeImageCodecMPDrawBandUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{  codec selectors 0-127 are reserved by Apple }
{  codec selectors 128-191 are subtype specific }
{  codec selectors 192-255 are vendor specific }
{  codec selectors 256-511 are available for general use }
{  codec selectors 512-1023 are reserved by Apple }
{  codec selectors 1024-32767 are available for general use }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeImageCodecMPDrawBandUPP( refcon: UnivPtr; var drp: ImageSubCodecDecompressRecord; userUPP: ImageCodecMPDrawBandUPP ): ComponentResult; external name '_InvokeImageCodecMPDrawBandUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{  codec selectors 0-127 are reserved by Apple }
{  codec selectors 128-191 are subtype specific }
{  codec selectors 192-255 are vendor specific }
<<<<<<< HEAD
{  codec selectors 256-32767 are available for general use }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{  codec selectors 256-511 are available for general use }
{  codec selectors 512-1023 are reserved by Apple }
{  codec selectors 1024-32767 are available for general use }
>>>>>>> origin/fixes_2.4
{  negative selectors are reserved by the Component Manager }
{
 *  ImageCodecGetCodecInfo()
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
function ImageCodecGetCodecInfo( ci: ComponentInstance; var info: CodecInfo ): ComponentResult; external name '_ImageCodecGetCodecInfo';
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
function ImageCodecGetCodecInfo(ci: ComponentInstance; var info: CodecInfo): ComponentResult; external name '_ImageCodecGetCodecInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetCodecInfo( ci: ComponentInstance; var info: CodecInfo ): ComponentResult; external name '_ImageCodecGetCodecInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetCompressionTime()
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
function ImageCodecGetCompressionTime( ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; var spatialQuality: CodecQ; var temporalQuality: CodecQ; var time: UNSIGNEDLONG ): ComponentResult; external name '_ImageCodecGetCompressionTime';
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
function ImageCodecGetCompressionTime(ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; var spatialQuality: CodecQ; var temporalQuality: CodecQ; var time: UInt32): ComponentResult; external name '_ImageCodecGetCompressionTime';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetCompressionTime( ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; var spatialQuality: CodecQ; var temporalQuality: CodecQ; var time: UNSIGNEDLONG ): ComponentResult; external name '_ImageCodecGetCompressionTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetMaxCompressionSize()
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
function ImageCodecGetMaxCompressionSize( ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; quality: CodecQ; var size: SIGNEDLONG ): ComponentResult; external name '_ImageCodecGetMaxCompressionSize';
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
function ImageCodecGetMaxCompressionSize(ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; quality: CodecQ; var size: SInt32): ComponentResult; external name '_ImageCodecGetMaxCompressionSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetMaxCompressionSize( ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; quality: CodecQ; var size: SIGNEDLONG ): ComponentResult; external name '_ImageCodecGetMaxCompressionSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecPreCompress()
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
function ImageCodecPreCompress( ci: ComponentInstance; var params: CodecCompressParams ): ComponentResult; external name '_ImageCodecPreCompress';
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
function ImageCodecPreCompress(ci: ComponentInstance; var params: CodecCompressParams): ComponentResult; external name '_ImageCodecPreCompress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecPreCompress( ci: ComponentInstance; var params: CodecCompressParams ): ComponentResult; external name '_ImageCodecPreCompress';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecBandCompress()
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
function ImageCodecBandCompress( ci: ComponentInstance; var params: CodecCompressParams ): ComponentResult; external name '_ImageCodecBandCompress';
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
function ImageCodecBandCompress(ci: ComponentInstance; var params: CodecCompressParams): ComponentResult; external name '_ImageCodecBandCompress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecBandCompress( ci: ComponentInstance; var params: CodecCompressParams ): ComponentResult; external name '_ImageCodecBandCompress';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecPreDecompress()
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
function ImageCodecPreDecompress( ci: ComponentInstance; var params: CodecDecompressParams ): ComponentResult; external name '_ImageCodecPreDecompress';
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
function ImageCodecPreDecompress(ci: ComponentInstance; var params: CodecDecompressParams): ComponentResult; external name '_ImageCodecPreDecompress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecPreDecompress( ci: ComponentInstance; var params: CodecDecompressParams ): ComponentResult; external name '_ImageCodecPreDecompress';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecBandDecompress()
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
function ImageCodecBandDecompress( ci: ComponentInstance; var params: CodecDecompressParams ): ComponentResult; external name '_ImageCodecBandDecompress';
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
function ImageCodecBandDecompress(ci: ComponentInstance; var params: CodecDecompressParams): ComponentResult; external name '_ImageCodecBandDecompress';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecBandDecompress( ci: ComponentInstance; var params: CodecDecompressParams ): ComponentResult; external name '_ImageCodecBandDecompress';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecBusy()
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
function ImageCodecBusy( ci: ComponentInstance; seq: ImageSequence ): ComponentResult; external name '_ImageCodecBusy';
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
function ImageCodecBusy(ci: ComponentInstance; seq: ImageSequence): ComponentResult; external name '_ImageCodecBusy';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecBusy( ci: ComponentInstance; seq: ImageSequence ): ComponentResult; external name '_ImageCodecBusy';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetCompressedImageSize()
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
function ImageCodecGetCompressedImageSize( ci: ComponentInstance; desc: ImageDescriptionHandle; data: Ptr; bufferSize: SIGNEDLONG; dataProc: ICMDataProcRecordPtr; var dataSize: SIGNEDLONG ): ComponentResult; external name '_ImageCodecGetCompressedImageSize';
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
function ImageCodecGetCompressedImageSize(ci: ComponentInstance; desc: ImageDescriptionHandle; data: Ptr; bufferSize: SInt32; dataProc: ICMDataProcRecordPtr; var dataSize: SInt32): ComponentResult; external name '_ImageCodecGetCompressedImageSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetCompressedImageSize( ci: ComponentInstance; desc: ImageDescriptionHandle; data: Ptr; bufferSize: SIGNEDLONG; dataProc: ICMDataProcRecordPtr; var dataSize: SIGNEDLONG ): ComponentResult; external name '_ImageCodecGetCompressedImageSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetSimilarity()
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
function ImageCodecGetSimilarity( ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; desc: ImageDescriptionHandle; data: Ptr; var similarity: Fixed ): ComponentResult; external name '_ImageCodecGetSimilarity';
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
function ImageCodecGetSimilarity(ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; desc: ImageDescriptionHandle; data: Ptr; var similarity: Fixed): ComponentResult; external name '_ImageCodecGetSimilarity';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetSimilarity( ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; desc: ImageDescriptionHandle; data: Ptr; var similarity: Fixed ): ComponentResult; external name '_ImageCodecGetSimilarity';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecTrimImage()
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
function ImageCodecTrimImage( ci: ComponentInstance; Desc: ImageDescriptionHandle; inData: Ptr; inBufferSize: SIGNEDLONG; dataProc: ICMDataProcRecordPtr; outData: Ptr; outBufferSize: SIGNEDLONG; flushProc: ICMFlushProcRecordPtr; var trimRect: Rect; progressProc: ICMProgressProcRecordPtr ): ComponentResult; external name '_ImageCodecTrimImage';
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
function ImageCodecTrimImage(ci: ComponentInstance; Desc: ImageDescriptionHandle; inData: Ptr; inBufferSize: SInt32; dataProc: ICMDataProcRecordPtr; outData: Ptr; outBufferSize: SInt32; flushProc: ICMFlushProcRecordPtr; var trimRect: Rect; progressProc: ICMProgressProcRecordPtr): ComponentResult; external name '_ImageCodecTrimImage';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecTrimImage( ci: ComponentInstance; Desc: ImageDescriptionHandle; inData: Ptr; inBufferSize: SIGNEDLONG; dataProc: ICMDataProcRecordPtr; outData: Ptr; outBufferSize: SIGNEDLONG; flushProc: ICMFlushProcRecordPtr; var trimRect: Rect; progressProc: ICMProgressProcRecordPtr ): ComponentResult; external name '_ImageCodecTrimImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecRequestSettings()
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
function ImageCodecRequestSettings( ci: ComponentInstance; settings: Handle; var rp: Rect; filterProc: ModalFilterUPP ): ComponentResult; external name '_ImageCodecRequestSettings';
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
function ImageCodecRequestSettings(ci: ComponentInstance; settings: Handle; var rp: Rect; filterProc: ModalFilterUPP): ComponentResult; external name '_ImageCodecRequestSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecRequestSettings( ci: ComponentInstance; settings: Handle; var rp: Rect; filterProc: ModalFilterUPP ): ComponentResult; external name '_ImageCodecRequestSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetSettings()
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
function ImageCodecGetSettings( ci: ComponentInstance; settings: Handle ): ComponentResult; external name '_ImageCodecGetSettings';
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
function ImageCodecGetSettings(ci: ComponentInstance; settings: Handle): ComponentResult; external name '_ImageCodecGetSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetSettings( ci: ComponentInstance; settings: Handle ): ComponentResult; external name '_ImageCodecGetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecSetSettings()
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
function ImageCodecSetSettings( ci: ComponentInstance; settings: Handle ): ComponentResult; external name '_ImageCodecSetSettings';
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
function ImageCodecSetSettings(ci: ComponentInstance; settings: Handle): ComponentResult; external name '_ImageCodecSetSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecSetSettings( ci: ComponentInstance; settings: Handle ): ComponentResult; external name '_ImageCodecSetSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecFlush()
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
function ImageCodecFlush( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecFlush';
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
function ImageCodecFlush(ci: ComponentInstance): ComponentResult; external name '_ImageCodecFlush';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecFlush( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecFlush';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecSetTimeCode()
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
function ImageCodecSetTimeCode( ci: ComponentInstance; timeCodeFormat: UnivPtr; timeCodeTime: UnivPtr ): ComponentResult; external name '_ImageCodecSetTimeCode';
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
function ImageCodecSetTimeCode(ci: ComponentInstance; timeCodeFormat: UnivPtr; timeCodeTime: UnivPtr): ComponentResult; external name '_ImageCodecSetTimeCode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecSetTimeCode( ci: ComponentInstance; timeCodeFormat: UnivPtr; timeCodeTime: UnivPtr ): ComponentResult; external name '_ImageCodecSetTimeCode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecIsImageDescriptionEquivalent()
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
function ImageCodecIsImageDescriptionEquivalent( ci: ComponentInstance; newDesc: ImageDescriptionHandle; var equivalent: Boolean ): ComponentResult; external name '_ImageCodecIsImageDescriptionEquivalent';
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
function ImageCodecIsImageDescriptionEquivalent(ci: ComponentInstance; newDesc: ImageDescriptionHandle; var equivalent: boolean): ComponentResult; external name '_ImageCodecIsImageDescriptionEquivalent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecIsImageDescriptionEquivalent( ci: ComponentInstance; newDesc: ImageDescriptionHandle; var equivalent: Boolean ): ComponentResult; external name '_ImageCodecIsImageDescriptionEquivalent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecNewMemory()
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
function ImageCodecNewMemory( ci: ComponentInstance; var data: Ptr; dataSize: Size; dataUse: SIGNEDLONG; memoryGoneProc: ICMMemoryDisposedUPP; refCon: UnivPtr ): ComponentResult; external name '_ImageCodecNewMemory';
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
function ImageCodecNewMemory(ci: ComponentInstance; var data: Ptr; dataSize: Size; dataUse: SInt32; memoryGoneProc: ICMMemoryDisposedUPP; refCon: UnivPtr): ComponentResult; external name '_ImageCodecNewMemory';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecNewMemory( ci: ComponentInstance; var data: Ptr; dataSize: Size; dataUse: SIGNEDLONG; memoryGoneProc: ICMMemoryDisposedUPP; refCon: UnivPtr ): ComponentResult; external name '_ImageCodecNewMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDisposeMemory()
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
function ImageCodecDisposeMemory( ci: ComponentInstance; data: Ptr ): ComponentResult; external name '_ImageCodecDisposeMemory';
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
function ImageCodecDisposeMemory(ci: ComponentInstance; data: Ptr): ComponentResult; external name '_ImageCodecDisposeMemory';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDisposeMemory( ci: ComponentInstance; data: Ptr ): ComponentResult; external name '_ImageCodecDisposeMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecHitTestData()
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
function ImageCodecHitTestData( ci: ComponentInstance; desc: ImageDescriptionHandle; data: UnivPtr; dataSize: Size; where: Point; var hit: Boolean ): ComponentResult; external name '_ImageCodecHitTestData';
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
function ImageCodecHitTestData(ci: ComponentInstance; desc: ImageDescriptionHandle; data: UnivPtr; dataSize: Size; where: Point; var hit: boolean): ComponentResult; external name '_ImageCodecHitTestData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecHitTestData( ci: ComponentInstance; desc: ImageDescriptionHandle; data: UnivPtr; dataSize: Size; where: Point; var hit: Boolean ): ComponentResult; external name '_ImageCodecHitTestData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecNewImageBufferMemory()
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
function ImageCodecNewImageBufferMemory( ci: ComponentInstance; var params: CodecDecompressParams; flags: SIGNEDLONG; memoryGoneProc: ICMMemoryDisposedUPP; refCon: UnivPtr ): ComponentResult; external name '_ImageCodecNewImageBufferMemory';
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
function ImageCodecNewImageBufferMemory(ci: ComponentInstance; var params: CodecDecompressParams; flags: SInt32; memoryGoneProc: ICMMemoryDisposedUPP; refCon: UnivPtr): ComponentResult; external name '_ImageCodecNewImageBufferMemory';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecNewImageBufferMemory( ci: ComponentInstance; var params: CodecDecompressParams; flags: SIGNEDLONG; memoryGoneProc: ICMMemoryDisposedUPP; refCon: UnivPtr ): ComponentResult; external name '_ImageCodecNewImageBufferMemory';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecExtractAndCombineFields()
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
function ImageCodecExtractAndCombineFields( ci: ComponentInstance; fieldFlags: SIGNEDLONG; data1: UnivPtr; dataSize1: SIGNEDLONG; desc1: ImageDescriptionHandle; data2: UnivPtr; dataSize2: SIGNEDLONG; desc2: ImageDescriptionHandle; outputData: UnivPtr; var outDataSize: SIGNEDLONG; descOut: ImageDescriptionHandle ): ComponentResult; external name '_ImageCodecExtractAndCombineFields';
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
function ImageCodecExtractAndCombineFields(ci: ComponentInstance; fieldFlags: SInt32; data1: UnivPtr; dataSize1: SInt32; desc1: ImageDescriptionHandle; data2: UnivPtr; dataSize2: SInt32; desc2: ImageDescriptionHandle; outputData: UnivPtr; var outDataSize: SInt32; descOut: ImageDescriptionHandle): ComponentResult; external name '_ImageCodecExtractAndCombineFields';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecExtractAndCombineFields( ci: ComponentInstance; fieldFlags: SIGNEDLONG; data1: UnivPtr; dataSize1: SIGNEDLONG; desc1: ImageDescriptionHandle; data2: UnivPtr; dataSize2: SIGNEDLONG; desc2: ImageDescriptionHandle; outputData: UnivPtr; var outDataSize: SIGNEDLONG; descOut: ImageDescriptionHandle ): ComponentResult; external name '_ImageCodecExtractAndCombineFields';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetMaxCompressionSizeWithSources()
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
function ImageCodecGetMaxCompressionSizeWithSources( ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; quality: CodecQ; sourceData: CDSequenceDataSourcePtr; var size: SIGNEDLONG ): ComponentResult; external name '_ImageCodecGetMaxCompressionSizeWithSources';
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
function ImageCodecGetMaxCompressionSizeWithSources(ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; quality: CodecQ; sourceData: CDSequenceDataSourcePtr; var size: SInt32): ComponentResult; external name '_ImageCodecGetMaxCompressionSizeWithSources';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetMaxCompressionSizeWithSources( ci: ComponentInstance; src: PixMapHandle; const (*var*) srcRect: Rect; depth: SInt16; quality: CodecQ; sourceData: CDSequenceDataSourcePtr; var size: SIGNEDLONG ): ComponentResult; external name '_ImageCodecGetMaxCompressionSizeWithSources';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecSetTimeBase()
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
function ImageCodecSetTimeBase( ci: ComponentInstance; base: UnivPtr ): ComponentResult; external name '_ImageCodecSetTimeBase';
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
function ImageCodecSetTimeBase(ci: ComponentInstance; base: UnivPtr): ComponentResult; external name '_ImageCodecSetTimeBase';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecSetTimeBase( ci: ComponentInstance; base: UnivPtr ): ComponentResult; external name '_ImageCodecSetTimeBase';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecSourceChanged()
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
function ImageCodecSourceChanged( ci: ComponentInstance; majorSourceChangeSeed: UInt32; minorSourceChangeSeed: UInt32; sourceData: CDSequenceDataSourcePtr; var flagsOut: SIGNEDLONG ): ComponentResult; external name '_ImageCodecSourceChanged';
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
function ImageCodecSourceChanged(ci: ComponentInstance; majorSourceChangeSeed: UInt32; minorSourceChangeSeed: UInt32; sourceData: CDSequenceDataSourcePtr; var flagsOut: SInt32): ComponentResult; external name '_ImageCodecSourceChanged';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecSourceChanged( ci: ComponentInstance; majorSourceChangeSeed: UInt32; minorSourceChangeSeed: UInt32; sourceData: CDSequenceDataSourcePtr; var flagsOut: SIGNEDLONG ): ComponentResult; external name '_ImageCodecSourceChanged';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecFlushFrame()
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
function ImageCodecFlushFrame( ci: ComponentInstance; flags: UInt32 ): ComponentResult; external name '_ImageCodecFlushFrame';
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
function ImageCodecFlushFrame(ci: ComponentInstance; flags: UInt32): ComponentResult; external name '_ImageCodecFlushFrame';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecFlushFrame( ci: ComponentInstance; flags: UInt32 ): ComponentResult; external name '_ImageCodecFlushFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetSettingsAsText()
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
function ImageCodecGetSettingsAsText( ci: ComponentInstance; var text: Handle ): ComponentResult; external name '_ImageCodecGetSettingsAsText';
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
function ImageCodecGetSettingsAsText(ci: ComponentInstance; var text: Handle): ComponentResult; external name '_ImageCodecGetSettingsAsText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetSettingsAsText( ci: ComponentInstance; var text: Handle ): ComponentResult; external name '_ImageCodecGetSettingsAsText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetParameterListHandle()
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
function ImageCodecGetParameterListHandle( ci: ComponentInstance; var parameterDescriptionHandle: Handle ): ComponentResult; external name '_ImageCodecGetParameterListHandle';
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
function ImageCodecGetParameterListHandle(ci: ComponentInstance; var parameterDescriptionHandle: Handle): ComponentResult; external name '_ImageCodecGetParameterListHandle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetParameterListHandle( ci: ComponentInstance; var parameterDescriptionHandle: Handle ): ComponentResult; external name '_ImageCodecGetParameterListHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetParameterList()
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
function ImageCodecGetParameterList( ci: ComponentInstance; var parameterDescription: QTAtomContainer ): ComponentResult; external name '_ImageCodecGetParameterList';
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
function ImageCodecGetParameterList(ci: ComponentInstance; var parameterDescription: QTAtomContainer): ComponentResult; external name '_ImageCodecGetParameterList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetParameterList( ci: ComponentInstance; var parameterDescription: QTAtomContainer ): ComponentResult; external name '_ImageCodecGetParameterList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecCreateStandardParameterDialog()
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
function ImageCodecCreateStandardParameterDialog( ci: ComponentInstance; parameterDescription: QTAtomContainer; parameters: QTAtomContainer; dialogOptions: QTParameterDialogOptions; existingDialog: DialogPtr; existingUserItem: SInt16; var createdDialog: QTParameterDialog ): ComponentResult; external name '_ImageCodecCreateStandardParameterDialog';
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
function ImageCodecCreateStandardParameterDialog(ci: ComponentInstance; parameterDescription: QTAtomContainer; parameters: QTAtomContainer; dialogOptions: QTParameterDialogOptions; existingDialog: DialogPtr; existingUserItem: SInt16; var createdDialog: QTParameterDialog): ComponentResult; external name '_ImageCodecCreateStandardParameterDialog';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecCreateStandardParameterDialog( ci: ComponentInstance; parameterDescription: QTAtomContainer; parameters: QTAtomContainer; dialogOptions: QTParameterDialogOptions; existingDialog: DialogPtr; existingUserItem: SInt16; var createdDialog: QTParameterDialog ): ComponentResult; external name '_ImageCodecCreateStandardParameterDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecIsStandardParameterDialogEvent()
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
function ImageCodecIsStandardParameterDialogEvent( ci: ComponentInstance; var pEvent: EventRecord; createdDialog: QTParameterDialog ): ComponentResult; external name '_ImageCodecIsStandardParameterDialogEvent';
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
function ImageCodecIsStandardParameterDialogEvent(ci: ComponentInstance; var pEvent: EventRecord; createdDialog: QTParameterDialog): ComponentResult; external name '_ImageCodecIsStandardParameterDialogEvent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecIsStandardParameterDialogEvent( ci: ComponentInstance; var pEvent: EventRecord; createdDialog: QTParameterDialog ): ComponentResult; external name '_ImageCodecIsStandardParameterDialogEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDismissStandardParameterDialog()
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
function ImageCodecDismissStandardParameterDialog( ci: ComponentInstance; createdDialog: QTParameterDialog ): ComponentResult; external name '_ImageCodecDismissStandardParameterDialog';
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
function ImageCodecDismissStandardParameterDialog(ci: ComponentInstance; createdDialog: QTParameterDialog): ComponentResult; external name '_ImageCodecDismissStandardParameterDialog';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDismissStandardParameterDialog( ci: ComponentInstance; createdDialog: QTParameterDialog ): ComponentResult; external name '_ImageCodecDismissStandardParameterDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecStandardParameterDialogDoAction()
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
function ImageCodecStandardParameterDialogDoAction( ci: ComponentInstance; createdDialog: QTParameterDialog; action: SIGNEDLONG; params: UnivPtr ): ComponentResult; external name '_ImageCodecStandardParameterDialogDoAction';
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
function ImageCodecStandardParameterDialogDoAction(ci: ComponentInstance; createdDialog: QTParameterDialog; action: SInt32; params: UnivPtr): ComponentResult; external name '_ImageCodecStandardParameterDialogDoAction';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{
 *  ImageCodecNewImageGWorld()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function ImageCodecNewImageGWorld( ci: ComponentInstance; var params: CodecDecompressParams; var newGW: GWorldPtr; flags: SIGNEDLONG ): ComponentResult; external name '_ImageCodecNewImageGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function ImageCodecNewImageGWorld(ci: ComponentInstance; var params: CodecDecompressParams; var newGW: GWorldPtr; flags: SInt32): ComponentResult; external name '_ImageCodecNewImageGWorld';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
function ImageCodecStandardParameterDialogDoAction( ci: ComponentInstance; createdDialog: QTParameterDialog; action: SIGNEDLONG; params: UnivPtr ): ComponentResult; external name '_ImageCodecStandardParameterDialogDoAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDisposeImageGWorld()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function ImageCodecDisposeImageGWorld( ci: ComponentInstance; theGW: GWorldPtr ): ComponentResult; external name '_ImageCodecDisposeImageGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
=======
function ImageCodecNewImageGWorld( ci: ComponentInstance; var params: CodecDecompressParams; var newGW: GWorldPtr; flags: SIGNEDLONG ): ComponentResult; external name '_ImageCodecNewImageGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDisposeImageGWorld()
 *  
 *  Availability:
<<<<<<< HEAD
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
function ImageCodecDisposeImageGWorld(ci: ComponentInstance; theGW: GWorldPtr): ComponentResult; external name '_ImageCodecDisposeImageGWorld';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDisposeImageGWorld( ci: ComponentInstance; theGW: GWorldPtr ): ComponentResult; external name '_ImageCodecDisposeImageGWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecHitTestDataWithFlags()
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
function ImageCodecHitTestDataWithFlags( ci: ComponentInstance; desc: ImageDescriptionHandle; data: UnivPtr; dataSize: Size; where: Point; var hit: SIGNEDLONG; hitFlags: SIGNEDLONG ): ComponentResult; external name '_ImageCodecHitTestDataWithFlags';
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
function ImageCodecHitTestDataWithFlags(ci: ComponentInstance; desc: ImageDescriptionHandle; data: UnivPtr; dataSize: Size; where: Point; var hit: SInt32; hitFlags: SInt32): ComponentResult; external name '_ImageCodecHitTestDataWithFlags';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecHitTestDataWithFlags( ci: ComponentInstance; desc: ImageDescriptionHandle; data: UnivPtr; dataSize: Size; where: Point; var hit: SIGNEDLONG; hitFlags: SIGNEDLONG ): ComponentResult; external name '_ImageCodecHitTestDataWithFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecValidateParameters()
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
function ImageCodecValidateParameters( ci: ComponentInstance; parameters: QTAtomContainer; validationFlags: QTParameterValidationOptions; errorString: StringPtr ): ComponentResult; external name '_ImageCodecValidateParameters';
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
function ImageCodecValidateParameters(ci: ComponentInstance; parameters: QTAtomContainer; validationFlags: QTParameterValidationOptions; errorString: StringPtr): ComponentResult; external name '_ImageCodecValidateParameters';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecValidateParameters( ci: ComponentInstance; parameters: QTAtomContainer; validationFlags: QTParameterValidationOptions; errorString: StringPtr ): ComponentResult; external name '_ImageCodecValidateParameters';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetBaseMPWorkFunction()
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
function ImageCodecGetBaseMPWorkFunction( ci: ComponentInstance; var workFunction: ComponentMPWorkFunctionUPP; var refCon: UnivPtr; drawProc: ImageCodecMPDrawBandUPP; drawProcRefCon: UnivPtr ): ComponentResult; external name '_ImageCodecGetBaseMPWorkFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  ImageCodecLockBits()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 (or QuickTime 6.4) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 6.5 and later
 }
function ImageCodecLockBits( ci: ComponentInstance; port: CGrafPtr ): ComponentResult; external name '_ImageCodecLockBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  ImageCodecUnlockBits()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 (or QuickTime 6.4) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 6.5 and later
 }
function ImageCodecUnlockBits( ci: ComponentInstance; port: CGrafPtr ): ComponentResult; external name '_ImageCodecUnlockBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


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
function ImageCodecGetBaseMPWorkFunction(ci: ComponentInstance; var workFunction: ComponentMPWorkFunctionUPP; var refCon: UnivPtr; drawProc: ImageCodecMPDrawBandUPP; drawProcRefCon: UnivPtr): ComponentResult; external name '_ImageCodecGetBaseMPWorkFunction';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetBaseMPWorkFunction( ci: ComponentInstance; var workFunction: ComponentMPWorkFunctionUPP; var refCon: UnivPtr; drawProc: ImageCodecMPDrawBandUPP; drawProcRefCon: UnivPtr ): ComponentResult; external name '_ImageCodecGetBaseMPWorkFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  ImageCodecLockBits()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 (or QuickTime 6.4) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 6.5 and later
 }
function ImageCodecLockBits( ci: ComponentInstance; port: CGrafPtr ): ComponentResult; external name '_ImageCodecLockBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  ImageCodecUnlockBits()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 (or QuickTime 6.4) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib 6.5 and later
 }
function ImageCodecUnlockBits( ci: ComponentInstance; port: CGrafPtr ): ComponentResult; external name '_ImageCodecUnlockBits';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecRequestGammaLevel()
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
function ImageCodecRequestGammaLevel( ci: ComponentInstance; srcGammaLevel: Fixed; dstGammaLevel: Fixed; var codecCanMatch: SIGNEDLONG ): ComponentResult; external name '_ImageCodecRequestGammaLevel';
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
function ImageCodecRequestGammaLevel(ci: ComponentInstance; srcGammaLevel: Fixed; dstGammaLevel: Fixed; var codecCanMatch: SInt32): ComponentResult; external name '_ImageCodecRequestGammaLevel';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecRequestGammaLevel( ci: ComponentInstance; srcGammaLevel: Fixed; dstGammaLevel: Fixed; var codecCanMatch: SIGNEDLONG ): ComponentResult; external name '_ImageCodecRequestGammaLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetSourceDataGammaLevel()
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
function ImageCodecGetSourceDataGammaLevel( ci: ComponentInstance; var sourceDataGammaLevel: Fixed ): ComponentResult; external name '_ImageCodecGetSourceDataGammaLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ (Selector 42 skipped) }
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
function ImageCodecGetSourceDataGammaLevel(ci: ComponentInstance; var sourceDataGammaLevel: Fixed): ComponentResult; external name '_ImageCodecGetSourceDataGammaLevel';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetSourceDataGammaLevel( ci: ComponentInstance; var sourceDataGammaLevel: Fixed ): ComponentResult; external name '_ImageCodecGetSourceDataGammaLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ (Selector 42 skipped) }
>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetDecompressLatency()
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
function ImageCodecGetDecompressLatency( ci: ComponentInstance; var latency: TimeRecord ): ComponentResult; external name '_ImageCodecGetDecompressLatency';
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
function ImageCodecGetDecompressLatency(ci: ComponentInstance; var latency: TimeRecord): ComponentResult; external name '_ImageCodecGetDecompressLatency';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetDecompressLatency( ci: ComponentInstance; var latency: TimeRecord ): ComponentResult; external name '_ImageCodecGetDecompressLatency';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecMergeFloatingImageOntoWindow()
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
function ImageCodecMergeFloatingImageOntoWindow( ci: ComponentInstance; flags: UInt32 ): ComponentResult; external name '_ImageCodecMergeFloatingImageOntoWindow';
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
function ImageCodecMergeFloatingImageOntoWindow(ci: ComponentInstance; flags: UInt32): ComponentResult; external name '_ImageCodecMergeFloatingImageOntoWindow';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecMergeFloatingImageOntoWindow( ci: ComponentInstance; flags: UInt32 ): ComponentResult; external name '_ImageCodecMergeFloatingImageOntoWindow';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecRemoveFloatingImage()
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
function ImageCodecRemoveFloatingImage( ci: ComponentInstance; flags: UInt32 ): ComponentResult; external name '_ImageCodecRemoveFloatingImage';
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
function ImageCodecRemoveFloatingImage(ci: ComponentInstance; flags: UInt32): ComponentResult; external name '_ImageCodecRemoveFloatingImage';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecRemoveFloatingImage( ci: ComponentInstance; flags: UInt32 ): ComponentResult; external name '_ImageCodecRemoveFloatingImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecGetDITLForSize()
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
function ImageCodecGetDITLForSize( ci: ComponentInstance; var ditl: Handle; var requestedSize: Point ): ComponentResult; external name '_ImageCodecGetDITLForSize';
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
function ImageCodecGetDITLForSize(ci: ComponentInstance; var ditl: Handle; var requestedSize: Point): ComponentResult; external name '_ImageCodecGetDITLForSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecGetDITLForSize( ci: ComponentInstance; var ditl: Handle; var requestedSize: Point ): ComponentResult; external name '_ImageCodecGetDITLForSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDITLInstall()
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
function ImageCodecDITLInstall( ci: ComponentInstance; d: DialogRef; itemOffset: SInt16 ): ComponentResult; external name '_ImageCodecDITLInstall';
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
function ImageCodecDITLInstall(ci: ComponentInstance; d: DialogRef; itemOffset: SInt16): ComponentResult; external name '_ImageCodecDITLInstall';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDITLInstall( ci: ComponentInstance; d: DialogRef; itemOffset: SInt16 ): ComponentResult; external name '_ImageCodecDITLInstall';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDITLEvent()
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
function ImageCodecDITLEvent( ci: ComponentInstance; d: DialogRef; itemOffset: SInt16; const (*var*) theEvent: EventRecord; var itemHit: SInt16; var handled: Boolean ): ComponentResult; external name '_ImageCodecDITLEvent';
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
function ImageCodecDITLEvent(ci: ComponentInstance; d: DialogRef; itemOffset: SInt16; const (*var*) theEvent: EventRecord; var itemHit: SInt16; var handled: boolean): ComponentResult; external name '_ImageCodecDITLEvent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDITLEvent( ci: ComponentInstance; d: DialogRef; itemOffset: SInt16; const (*var*) theEvent: EventRecord; var itemHit: SInt16; var handled: Boolean ): ComponentResult; external name '_ImageCodecDITLEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDITLItem()
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
function ImageCodecDITLItem( ci: ComponentInstance; d: DialogRef; itemOffset: SInt16; itemNum: SInt16 ): ComponentResult; external name '_ImageCodecDITLItem';
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
function ImageCodecDITLItem(ci: ComponentInstance; d: DialogRef; itemOffset: SInt16; itemNum: SInt16): ComponentResult; external name '_ImageCodecDITLItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDITLItem( ci: ComponentInstance; d: DialogRef; itemOffset: SInt16; itemNum: SInt16 ): ComponentResult; external name '_ImageCodecDITLItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDITLRemove()
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
function ImageCodecDITLRemove( ci: ComponentInstance; d: DialogRef; itemOffset: SInt16 ): ComponentResult; external name '_ImageCodecDITLRemove';
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
function ImageCodecDITLRemove(ci: ComponentInstance; d: DialogRef; itemOffset: SInt16): ComponentResult; external name '_ImageCodecDITLRemove';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDITLRemove( ci: ComponentInstance; d: DialogRef; itemOffset: SInt16 ): ComponentResult; external name '_ImageCodecDITLRemove';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDITLValidateInput()
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
function ImageCodecDITLValidateInput( ci: ComponentInstance; var ok: Boolean ): ComponentResult; external name '_ImageCodecDITLValidateInput';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{ (Selector 52 skipped) }
{ (Selector 53 skipped) }
{
 *  ImageCodecGetPreferredChunkSizeAndAlignment()
 *  
 *  Summary:
 *    Returns the preferences of an image decompressor for the chunking
 *    of image data within a container, e.g. a movie file.
 *  
 *  Discussion:
 *    If you are writing image data to a container, you can optimize
 *    the subsequent loading of the image data for playback and other
 *    operations by chunking multiple samples of image data together.
 *    This function can be called to determine whether an image
 *    decompressor has special chunking preferences.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    desc:
 *      The image description for the image data to be chunked.
 *    
 *    frameRate:
 *      Mean frame rate in frames per second as in SCTemporalSettings;
 *      0 if not known or not applicable.
 *    
 *    dataRate:
 *      Mean data rate in bytes per second as in SCDataRateSettings; 0
 *      if not known.
 *    
 *    bytesPerChunk:
 *      Points to a variable to receive the preferred maximum size in
 *      bytes of each chunk of image data. It is not safe to pass NULL
 *      for this parameter. The codec may indicate that it has no
 *      preference regarding chunk sizing by setting the variable to 0.
 *    
 *    alignment:
 *      Points to a variable to receive the preferred boundary for
 *      chunk alignment in bytes, e.g. 512. It is not safe to pass NULL
 *      for this parameter. The codec may indicate that it has no
 *      preference regarding chunk alignment by setting the variable to
 *      0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecGetPreferredChunkSizeAndAlignment( ci: ComponentInstance; desc: ImageDescriptionHandle; frameRate: Fixed; dataRate: UInt32; var bytesPerChunk: SIGNEDLONG; var alignment: SIGNEDLONG ): ComponentResult; external name '_ImageCodecGetPreferredChunkSizeAndAlignment';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ B-Frame Capable Image Compressor Component API }
{
 *  ImageCodecPrepareToCompressFrames()
 *  
 *  Summary:
 *    Prepares the compressor to receive frames.
 *  
 *  Discussion:
 *    The compressor should record session and retain
 *    compressionSessionOptions for use in later calls. 
 *    The compressor may modify imageDescription at this point. 
 *     The compressor should create and return pixel buffer attributes,
 *    which the ICM will release. 
 *    (Note: this replaces ImageCodecPreCompress.)
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    session:
 *      The compressor session reference. The compressor should store
 *      this in its globals; it will need it when calling the ICM back
 *      (eg, to call ICMEncodedFrameCreateMutable and
 *      ICMCompressorSessionEmitEncodedFrame). 
 *      This is not a CF type. Do not call CFRetain or CFRelease on it.
 *    
 *    compressionSessionOptions:
 *      The session options from the client. The compressor should
 *      retain this and use the settings to guide compression.
 *    
 *    imageDescription:
 *      The image description. The compressor may add image description
 *      extensions.
 *    
 *    reserved:
 *      Reserved for future use.  Ignore this parameter.
 *    
 *    compressorPixelBufferAttributesOut:
 *      The compressor should create a pixel buffer attributes
 *      dictionary and set compressorPixelBufferAttributesOut to it. 
 *      The ICM will release it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecPrepareToCompressFrames( ci: ComponentInstance; session: ICMCompressorSessionRef; compressionSessionOptions: ICMCompressionSessionOptionsRef; imageDescription: ImageDescriptionHandle; reserved: UnivPtr; var compressorPixelBufferAttributesOut: CFDictionaryRef ): ComponentResult; external name '_ImageCodecPrepareToCompressFrames';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecEncodeFrame()
 *  
 *  Summary:
 *    Presents the compressor with a frame to encode.
 *  
 *  Discussion:
 *    The compressor may encode the frame immediately or queue it for
 *    later encoding. If the compressor queues the frame for later
 *    decode, it must retain it (by calling
 *    ICMCompressorSourceFrameRetain) and release it when it is done
 *    with it (by calling ICMCompressorSourceFrameRelease). 
 *    Pixel buffers are guaranteed to conform to the pixel buffer
 *    attributes returned by ImageCodecPrepareToCompressFrames. 
 *     During multipass encoding, if the compressor requested the
 *    kICMCompressionPassMode_NoSourceFrames flag, the source frame
 *    pixel buffers may be NULL. 
 *    (Note: this replaces ImageCodecBandCompress.)
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    sourceFrame:
 *      The source frame to encode.
 *    
 *    flags:
 *      Reserved; ignore.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecEncodeFrame( ci: ComponentInstance; sourceFrame: ICMCompressorSourceFrameRef; flags: UInt32 ): ComponentResult; external name '_ImageCodecEncodeFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecCompleteFrame()
 *  
 *  Summary:
 *    Directs the compressor to finish with a queued source frame,
 *    either emitting or dropping it.
 *  
 *  Discussion:
 *    This frame does not necessarily need to be the first or only
 *    source frame emitted or dropped during this call, but the
 *    compressor must call either ICMCompressorSessionDropFrame or
 *    ICMCompressorSessionEmitEncodedFrame with this frame before
 *    returning. 
 *    The ICM will call this function to force frames to be encoded for
 *    the following reasons: (a) the maximum frame delay count or
 *    maximum frame delay time in the compressionSessionOptions does
 *    not permit frames to be queued; (b) the client has called
 *    ICMCompressionSessionCompleteFrames.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    sourceFrame:
 *      The source frame that must be completed.
 *    
 *    flags:
 *      Reserved; ignore.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecCompleteFrame( ci: ComponentInstance; sourceFrame: ICMCompressorSourceFrameRef; flags: UInt32 ): ComponentResult; external name '_ImageCodecCompleteFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecBeginPass()
 *  
 *  Summary:
 *    Notifies the compressor that it should operate in multipass mode
 *    and use the given multipass storage.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    passModeFlags:
 *      Indicates how the compressor should operate in this pass. 
 *       If the kICMCompressionPassMode_WriteToMultiPassStorage flag is
 *      set, the compressor may gather information of interest and
 *      store it in multiPassStorage. 
 *      If the kICMCompressionPassMode_ReadFromMultiPassStorage flag is
 *      set, the compressor may retrieve information from
 *      multiPassStorage. 
 *      If the kICMCompressionPassMode_OutputEncodedFrames flag is set,
 *      the compressor must encode or drop every frame by calling
 *      ICMCompressorSessionDropFrame or
 *      ICMCompressorSessionEmitEncodedFrame. If that flag is not set,
 *      the compressor should not call these routines.
 *    
 *    flags:
 *      Reserved.  Ignore this parameter.
 *    
 *    multiPassStorage:
 *      The multipass storage object that the compressor should use to
 *      store and retrieve information between passes.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecBeginPass( ci: ComponentInstance; passModeFlags: ICMCompressionPassModeFlags; flags: UInt32; multiPassStorage: ICMMultiPassStorageRef ): ComponentResult; external name '_ImageCodecBeginPass';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecEndPass()
 *  
 *  Summary:
 *    Notifies the compressor that a pass is over.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecEndPass( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecEndPass';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecProcessBetweenPasses()
 *  
 *  Summary:
 *    Gives the compressor an opportunity to perform processing between
 *    passes.
 *  
 *  Discussion:
 *    This function will be called repeatedly until it returns true in
 *    *interpassProcessingDoneOut. 
 *    The compressor may read and write to multiPassStorage. 
 *    The compressor should indicate which type of pass it would prefer
 *    to perform next by setting *requestedNextPassTypeOut.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    multiPassStorage:
 *      The multipass storage object that the compressor should use to
 *      store and retrieve information between passes.
 *    
 *    interpassProcessingDoneOut:
 *      Points to a Boolean. Set this to false if you want your
 *      ImageCodecProcessBetweenPasses function to be called again to
 *      perform more processing, true if not.
 *    
 *    requestedNextPassModeFlagsOut:
 *      Set *requestedNextPassModeFlagsOut to indicate the type of pass
 *      that should be performed next: 
 *      To recommend a repeated analysis pass, set it to
 *      kICMCompressionPassMode_ReadFromMultiPassStorage |
 *      kICMCompressionPassMode_WriteToMultiPassStorage. 
 *      To recommend a final encoding pass, set it to
 *      kICMCompressionPassMode_ReadFromMultiPassStorage |
 *      kICMCompressionPassMode_OutputEncodedFrames. 
 *      If source frame buffers are not necessary for the recommended
 *      pass (eg, because all the required data has been copied into
 *      multipass storage), set kICMCompressionPassMode_NoSourceFrames.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecProcessBetweenPasses( ci: ComponentInstance; multiPassStorage: ICMMultiPassStorageRef; var interpassProcessingDoneOut: Boolean; var requestedNextPassModeFlagsOut: ICMCompressionPassModeFlags ): ComponentResult; external name '_ImageCodecProcessBetweenPasses';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ (Selector 61 skipped) }
{ (Selector 62 skipped) }
{ (Selector 63 skipped) }
{ (Selector 64 skipped) }
{ (Selector 65 skipped) }
{ (Selector 66 skipped) }
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
function ImageCodecDITLValidateInput(ci: ComponentInstance; var ok: boolean): ComponentResult; external name '_ImageCodecDITLValidateInput';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDITLValidateInput( ci: ComponentInstance; var ok: Boolean ): ComponentResult; external name '_ImageCodecDITLValidateInput';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{ (Selector 52 skipped) }
{ (Selector 53 skipped) }
{
 *  ImageCodecGetPreferredChunkSizeAndAlignment()
 *  
 *  Summary:
 *    Returns the preferences of an image decompressor for the chunking
 *    of image data within a container, e.g. a movie file.
 *  
 *  Discussion:
 *    If you are writing image data to a container, you can optimize
 *    the subsequent loading of the image data for playback and other
 *    operations by chunking multiple samples of image data together.
 *    This function can be called to determine whether an image
 *    decompressor has special chunking preferences.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    desc:
 *      The image description for the image data to be chunked.
 *    
 *    frameRate:
 *      Mean frame rate in frames per second as in SCTemporalSettings;
 *      0 if not known or not applicable.
 *    
 *    dataRate:
 *      Mean data rate in bytes per second as in SCDataRateSettings; 0
 *      if not known.
 *    
 *    bytesPerChunk:
 *      Points to a variable to receive the preferred maximum size in
 *      bytes of each chunk of image data. It is not safe to pass NULL
 *      for this parameter. The codec may indicate that it has no
 *      preference regarding chunk sizing by setting the variable to 0.
 *    
 *    alignment:
 *      Points to a variable to receive the preferred boundary for
 *      chunk alignment in bytes, e.g. 512. It is not safe to pass NULL
 *      for this parameter. The codec may indicate that it has no
 *      preference regarding chunk alignment by setting the variable to
 *      0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecGetPreferredChunkSizeAndAlignment( ci: ComponentInstance; desc: ImageDescriptionHandle; frameRate: Fixed; dataRate: UInt32; var bytesPerChunk: SIGNEDLONG; var alignment: SIGNEDLONG ): ComponentResult; external name '_ImageCodecGetPreferredChunkSizeAndAlignment';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ B-Frame Capable Image Compressor Component API }
{
 *  ImageCodecPrepareToCompressFrames()
 *  
 *  Summary:
 *    Prepares the compressor to receive frames.
 *  
 *  Discussion:
 *    The compressor should record session and retain
 *    compressionSessionOptions for use in later calls. 
 *    The compressor may modify imageDescription at this point. 
 *     The compressor should create and return pixel buffer attributes,
 *    which the ICM will release. 
 *    (Note: this replaces ImageCodecPreCompress.)
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    session:
 *      The compressor session reference. The compressor should store
 *      this in its globals; it will need it when calling the ICM back
 *      (eg, to call ICMEncodedFrameCreateMutable and
 *      ICMCompressorSessionEmitEncodedFrame). 
 *      This is not a CF type. Do not call CFRetain or CFRelease on it.
 *    
 *    compressionSessionOptions:
 *      The session options from the client. The compressor should
 *      retain this and use the settings to guide compression.
 *    
 *    imageDescription:
 *      The image description. The compressor may add image description
 *      extensions.
 *    
 *    reserved:
 *      Reserved for future use.  Ignore this parameter.
 *    
 *    compressorPixelBufferAttributesOut:
 *      The compressor should create a pixel buffer attributes
 *      dictionary and set compressorPixelBufferAttributesOut to it. 
 *      The ICM will release it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecPrepareToCompressFrames( ci: ComponentInstance; session: ICMCompressorSessionRef; compressionSessionOptions: ICMCompressionSessionOptionsRef; imageDescription: ImageDescriptionHandle; reserved: UnivPtr; var compressorPixelBufferAttributesOut: CFDictionaryRef ): ComponentResult; external name '_ImageCodecPrepareToCompressFrames';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecEncodeFrame()
 *  
 *  Summary:
 *    Presents the compressor with a frame to encode.
 *  
 *  Discussion:
 *    The compressor may encode the frame immediately or queue it for
 *    later encoding. If the compressor queues the frame for later
 *    decode, it must retain it (by calling
 *    ICMCompressorSourceFrameRetain) and release it when it is done
 *    with it (by calling ICMCompressorSourceFrameRelease). 
 *    Pixel buffers are guaranteed to conform to the pixel buffer
 *    attributes returned by ImageCodecPrepareToCompressFrames. 
 *     During multipass encoding, if the compressor requested the
 *    kICMCompressionPassMode_NoSourceFrames flag, the source frame
 *    pixel buffers may be NULL. 
 *    (Note: this replaces ImageCodecBandCompress.)
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    sourceFrame:
 *      The source frame to encode.
 *    
 *    flags:
 *      Reserved; ignore.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecEncodeFrame( ci: ComponentInstance; sourceFrame: ICMCompressorSourceFrameRef; flags: UInt32 ): ComponentResult; external name '_ImageCodecEncodeFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecCompleteFrame()
 *  
 *  Summary:
 *    Directs the compressor to finish with a queued source frame,
 *    either emitting or dropping it.
 *  
 *  Discussion:
 *    This frame does not necessarily need to be the first or only
 *    source frame emitted or dropped during this call, but the
 *    compressor must call either ICMCompressorSessionDropFrame or
 *    ICMCompressorSessionEmitEncodedFrame with this frame before
 *    returning. 
 *    The ICM will call this function to force frames to be encoded for
 *    the following reasons: (a) the maximum frame delay count or
 *    maximum frame delay time in the compressionSessionOptions does
 *    not permit frames to be queued; (b) the client has called
 *    ICMCompressionSessionCompleteFrames.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    sourceFrame:
 *      The source frame that must be completed.
 *    
 *    flags:
 *      Reserved; ignore.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecCompleteFrame( ci: ComponentInstance; sourceFrame: ICMCompressorSourceFrameRef; flags: UInt32 ): ComponentResult; external name '_ImageCodecCompleteFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecBeginPass()
 *  
 *  Summary:
 *    Notifies the compressor that it should operate in multipass mode
 *    and use the given multipass storage.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    passModeFlags:
 *      Indicates how the compressor should operate in this pass. 
 *       If the kICMCompressionPassMode_WriteToMultiPassStorage flag is
 *      set, the compressor may gather information of interest and
 *      store it in multiPassStorage. 
 *      If the kICMCompressionPassMode_ReadFromMultiPassStorage flag is
 *      set, the compressor may retrieve information from
 *      multiPassStorage. 
 *      If the kICMCompressionPassMode_OutputEncodedFrames flag is set,
 *      the compressor must encode or drop every frame by calling
 *      ICMCompressorSessionDropFrame or
 *      ICMCompressorSessionEmitEncodedFrame. If that flag is not set,
 *      the compressor should not call these routines.
 *    
 *    flags:
 *      Reserved.  Ignore this parameter.
 *    
 *    multiPassStorage:
 *      The multipass storage object that the compressor should use to
 *      store and retrieve information between passes.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecBeginPass( ci: ComponentInstance; passModeFlags: ICMCompressionPassModeFlags; flags: UInt32; multiPassStorage: ICMMultiPassStorageRef ): ComponentResult; external name '_ImageCodecBeginPass';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecEndPass()
 *  
 *  Summary:
 *    Notifies the compressor that a pass is over.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecEndPass( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecEndPass';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  ImageCodecProcessBetweenPasses()
 *  
 *  Summary:
 *    Gives the compressor an opportunity to perform processing between
 *    passes.
 *  
 *  Discussion:
 *    This function will be called repeatedly until it returns true in
 *    *interpassProcessingDoneOut. 
 *    The compressor may read and write to multiPassStorage. 
 *    The compressor should indicate which type of pass it would prefer
 *    to perform next by setting *requestedNextPassTypeOut.
 *  
 *  Parameters:
 *    
 *    ci:
 *      Component instance / instance globals.
 *    
 *    multiPassStorage:
 *      The multipass storage object that the compressor should use to
 *      store and retrieve information between passes.
 *    
 *    interpassProcessingDoneOut:
 *      Points to a Boolean. Set this to false if you want your
 *      ImageCodecProcessBetweenPasses function to be called again to
 *      perform more processing, true if not.
 *    
 *    requestedNextPassModeFlagsOut:
 *      Set *requestedNextPassModeFlagsOut to indicate the type of pass
 *      that should be performed next: 
 *      To recommend a repeated analysis pass, set it to
 *      kICMCompressionPassMode_ReadFromMultiPassStorage |
 *      kICMCompressionPassMode_WriteToMultiPassStorage. 
 *      To recommend a final encoding pass, set it to
 *      kICMCompressionPassMode_ReadFromMultiPassStorage |
 *      kICMCompressionPassMode_OutputEncodedFrames. 
 *      If source frame buffers are not necessary for the recommended
 *      pass (eg, because all the required data has been copied into
 *      multipass storage), set kICMCompressionPassMode_NoSourceFrames.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function ImageCodecProcessBetweenPasses( ci: ComponentInstance; multiPassStorage: ICMMultiPassStorageRef; var interpassProcessingDoneOut: Boolean; var requestedNextPassModeFlagsOut: ICMCompressionPassModeFlags ): ComponentResult; external name '_ImageCodecProcessBetweenPasses';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ (Selector 61 skipped) }
{ (Selector 62 skipped) }
{ (Selector 63 skipped) }
{ (Selector 64 skipped) }
{ (Selector 65 skipped) }
{ (Selector 66 skipped) }
>>>>>>> origin/fixes_2.4
{
 *  ImageCodecPreflight()
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
function ImageCodecPreflight( ci: ComponentInstance; var params: CodecDecompressParams ): ComponentResult; external name '_ImageCodecPreflight';
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
function ImageCodecPreflight(ci: ComponentInstance; var params: CodecDecompressParams): ComponentResult; external name '_ImageCodecPreflight';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecPreflight( ci: ComponentInstance; var params: CodecDecompressParams ): ComponentResult; external name '_ImageCodecPreflight';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecInitialize()
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
function ImageCodecInitialize( ci: ComponentInstance; var cap: ImageSubCodecDecompressCapabilities ): ComponentResult; external name '_ImageCodecInitialize';
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
function ImageCodecInitialize(ci: ComponentInstance; var cap: ImageSubCodecDecompressCapabilities): ComponentResult; external name '_ImageCodecInitialize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecInitialize( ci: ComponentInstance; var cap: ImageSubCodecDecompressCapabilities ): ComponentResult; external name '_ImageCodecInitialize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecBeginBand()
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
function ImageCodecBeginBand( ci: ComponentInstance; var params: CodecDecompressParams; var drp: ImageSubCodecDecompressRecord; flags: SIGNEDLONG ): ComponentResult; external name '_ImageCodecBeginBand';
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
function ImageCodecBeginBand(ci: ComponentInstance; var params: CodecDecompressParams; var drp: ImageSubCodecDecompressRecord; flags: SInt32): ComponentResult; external name '_ImageCodecBeginBand';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecBeginBand( ci: ComponentInstance; var params: CodecDecompressParams; var drp: ImageSubCodecDecompressRecord; flags: SIGNEDLONG ): ComponentResult; external name '_ImageCodecBeginBand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDrawBand()
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
function ImageCodecDrawBand( ci: ComponentInstance; var drp: ImageSubCodecDecompressRecord ): ComponentResult; external name '_ImageCodecDrawBand';
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
function ImageCodecDrawBand(ci: ComponentInstance; var drp: ImageSubCodecDecompressRecord): ComponentResult; external name '_ImageCodecDrawBand';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDrawBand( ci: ComponentInstance; var drp: ImageSubCodecDecompressRecord ): ComponentResult; external name '_ImageCodecDrawBand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecEndBand()
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
function ImageCodecEndBand( ci: ComponentInstance; var drp: ImageSubCodecDecompressRecord; result: OSErr; flags: SIGNEDLONG ): ComponentResult; external name '_ImageCodecEndBand';
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
function ImageCodecEndBand(ci: ComponentInstance; var drp: ImageSubCodecDecompressRecord; result: OSErr; flags: SInt32): ComponentResult; external name '_ImageCodecEndBand';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecEndBand( ci: ComponentInstance; var drp: ImageSubCodecDecompressRecord; result: OSErr; flags: SIGNEDLONG ): ComponentResult; external name '_ImageCodecEndBand';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecQueueStarting()
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
function ImageCodecQueueStarting( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecQueueStarting';
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
function ImageCodecQueueStarting(ci: ComponentInstance): ComponentResult; external name '_ImageCodecQueueStarting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecQueueStarting( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecQueueStarting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecQueueStopping()
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
function ImageCodecQueueStopping( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecQueueStopping';
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
function ImageCodecQueueStopping(ci: ComponentInstance): ComponentResult; external name '_ImageCodecQueueStopping';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecQueueStopping( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecQueueStopping';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecDroppingFrame()
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
function ImageCodecDroppingFrame( ci: ComponentInstance; const (*var*) drp: ImageSubCodecDecompressRecord ): ComponentResult; external name '_ImageCodecDroppingFrame';
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
function ImageCodecDroppingFrame(ci: ComponentInstance; const (*var*) drp: ImageSubCodecDecompressRecord): ComponentResult; external name '_ImageCodecDroppingFrame';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecDroppingFrame( ci: ComponentInstance; const (*var*) drp: ImageSubCodecDecompressRecord ): ComponentResult; external name '_ImageCodecDroppingFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecScheduleFrame()
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
function ImageCodecScheduleFrame( ci: ComponentInstance; const (*var*) drp: ImageSubCodecDecompressRecord; triggerProc: ImageCodecTimeTriggerUPP; triggerProcRefCon: UnivPtr ): ComponentResult; external name '_ImageCodecScheduleFrame';
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
function ImageCodecScheduleFrame(ci: ComponentInstance; const (*var*) drp: ImageSubCodecDecompressRecord; triggerProc: ImageCodecTimeTriggerUPP; triggerProcRefCon: UnivPtr): ComponentResult; external name '_ImageCodecScheduleFrame';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecScheduleFrame( ci: ComponentInstance; const (*var*) drp: ImageSubCodecDecompressRecord; triggerProc: ImageCodecTimeTriggerUPP; triggerProcRefCon: UnivPtr ): ComponentResult; external name '_ImageCodecScheduleFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecCancelTrigger()
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
function ImageCodecCancelTrigger( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecCancelTrigger';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ImageCodecDecodeBand()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib version 10.4 (or QuickTime 7.0) and later
 }
function ImageCodecDecodeBand( ci: ComponentInstance; var drp: ImageSubCodecDecompressRecord; flags: UNSIGNEDLONG ): ComponentResult; external name '_ImageCodecDecodeBand';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ selectors for component calls }
const
	kImageCodecGetCodecInfoSelect = $0000;
	kImageCodecGetCompressionTimeSelect = $0001;
	kImageCodecGetMaxCompressionSizeSelect = $0002;
	kImageCodecPreCompressSelect = $0003;
	kImageCodecBandCompressSelect = $0004;
	kImageCodecPreDecompressSelect = $0005;
	kImageCodecBandDecompressSelect = $0006;
	kImageCodecBusySelect = $0007;
	kImageCodecGetCompressedImageSizeSelect = $0008;
	kImageCodecGetSimilaritySelect = $0009;
	kImageCodecTrimImageSelect = $000A;
	kImageCodecRequestSettingsSelect = $000B;
	kImageCodecGetSettingsSelect = $000C;
	kImageCodecSetSettingsSelect = $000D;
	kImageCodecFlushSelect = $000E;
	kImageCodecSetTimeCodeSelect = $000F;
	kImageCodecIsImageDescriptionEquivalentSelect = $0010;
	kImageCodecNewMemorySelect = $0011;
	kImageCodecDisposeMemorySelect = $0012;
	kImageCodecHitTestDataSelect = $0013;
	kImageCodecNewImageBufferMemorySelect = $0014;
	kImageCodecExtractAndCombineFieldsSelect = $0015;
	kImageCodecGetMaxCompressionSizeWithSourcesSelect = $0016;
	kImageCodecSetTimeBaseSelect = $0017;
	kImageCodecSourceChangedSelect = $0018;
	kImageCodecFlushFrameSelect = $0019;
	kImageCodecGetSettingsAsTextSelect = $001A;
	kImageCodecGetParameterListHandleSelect = $001B;
	kImageCodecGetParameterListSelect = $001C;
	kImageCodecCreateStandardParameterDialogSelect = $001D;
	kImageCodecIsStandardParameterDialogEventSelect = $001E;
	kImageCodecDismissStandardParameterDialogSelect = $001F;
	kImageCodecStandardParameterDialogDoActionSelect = $0020;
	kImageCodecNewImageGWorldSelect = $0021;
	kImageCodecDisposeImageGWorldSelect = $0022;
	kImageCodecHitTestDataWithFlagsSelect = $0023;
	kImageCodecValidateParametersSelect = $0024;
	kImageCodecGetBaseMPWorkFunctionSelect = $0025;
	kImageCodecLockBitsSelect = $0026;
	kImageCodecUnlockBitsSelect = $0027;
	kImageCodecRequestGammaLevelSelect = $0028;
	kImageCodecGetSourceDataGammaLevelSelect = $0029;
	kImageCodecGetDecompressLatencySelect = $002B;
	kImageCodecMergeFloatingImageOntoWindowSelect = $002C;
	kImageCodecRemoveFloatingImageSelect = $002D;
	kImageCodecGetDITLForSizeSelect = $002E;
	kImageCodecDITLInstallSelect = $002F;
	kImageCodecDITLEventSelect = $0030;
	kImageCodecDITLItemSelect = $0031;
	kImageCodecDITLRemoveSelect = $0032;
	kImageCodecDITLValidateInputSelect = $0033;
	kImageCodecGetPreferredChunkSizeAndAlignmentSelect = $0036;
	kImageCodecPrepareToCompressFramesSelect = $0037;
	kImageCodecEncodeFrameSelect = $0038;
	kImageCodecCompleteFrameSelect = $0039;
	kImageCodecBeginPassSelect = $003A;
	kImageCodecEndPassSelect = $003B;
	kImageCodecProcessBetweenPassesSelect = $003C;
	kImageCodecPreflightSelect = $0200;
	kImageCodecInitializeSelect = $0201;
	kImageCodecBeginBandSelect = $0202;
	kImageCodecDrawBandSelect = $0203;
	kImageCodecEndBandSelect = $0204;
	kImageCodecQueueStartingSelect = $0205;
	kImageCodecQueueStoppingSelect = $0206;
	kImageCodecDroppingFrameSelect = $0207;
	kImageCodecScheduleFrameSelect = $0208;
	kImageCodecCancelTriggerSelect = $0209;
	kImageCodecDecodeBandSelect = $020F;


const
	kMotionJPEGTag = FourCharCode('mjpg');
	kJPEGQuantizationTablesImageDescriptionExtension = FourCharCode('mjqt');
	kJPEGHuffmanTablesImageDescriptionExtension = FourCharCode('mjht');
	kFieldInfoImageDescriptionExtension = FourCharCode('fiel'); { image description extension describing the field count and field orderings}

const
	kFieldOrderUnknown = 0;
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
function ImageCodecCancelTrigger( ci: ComponentInstance ): ComponentResult; external name '_ImageCodecCancelTrigger';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ImageCodecDecodeBand()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 (or QuickTime 7.0) and later in QuickTime.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 *    Windows:          in qtmlClient.lib version 10.4 (or QuickTime 7.0) and later
 }
function ImageCodecDecodeBand( ci: ComponentInstance; var drp: ImageSubCodecDecompressRecord; flags: UNSIGNEDLONG ): ComponentResult; external name '_ImageCodecDecodeBand';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ selectors for component calls }
const
	kImageCodecGetCodecInfoSelect = $0000;
	kImageCodecGetCompressionTimeSelect = $0001;
	kImageCodecGetMaxCompressionSizeSelect = $0002;
	kImageCodecPreCompressSelect = $0003;
	kImageCodecBandCompressSelect = $0004;
	kImageCodecPreDecompressSelect = $0005;
	kImageCodecBandDecompressSelect = $0006;
	kImageCodecBusySelect = $0007;
	kImageCodecGetCompressedImageSizeSelect = $0008;
	kImageCodecGetSimilaritySelect = $0009;
	kImageCodecTrimImageSelect = $000A;
	kImageCodecRequestSettingsSelect = $000B;
	kImageCodecGetSettingsSelect = $000C;
	kImageCodecSetSettingsSelect = $000D;
	kImageCodecFlushSelect = $000E;
	kImageCodecSetTimeCodeSelect = $000F;
	kImageCodecIsImageDescriptionEquivalentSelect = $0010;
	kImageCodecNewMemorySelect = $0011;
	kImageCodecDisposeMemorySelect = $0012;
	kImageCodecHitTestDataSelect = $0013;
	kImageCodecNewImageBufferMemorySelect = $0014;
	kImageCodecExtractAndCombineFieldsSelect = $0015;
	kImageCodecGetMaxCompressionSizeWithSourcesSelect = $0016;
	kImageCodecSetTimeBaseSelect = $0017;
	kImageCodecSourceChangedSelect = $0018;
	kImageCodecFlushFrameSelect = $0019;
	kImageCodecGetSettingsAsTextSelect = $001A;
	kImageCodecGetParameterListHandleSelect = $001B;
	kImageCodecGetParameterListSelect = $001C;
	kImageCodecCreateStandardParameterDialogSelect = $001D;
	kImageCodecIsStandardParameterDialogEventSelect = $001E;
	kImageCodecDismissStandardParameterDialogSelect = $001F;
	kImageCodecStandardParameterDialogDoActionSelect = $0020;
	kImageCodecNewImageGWorldSelect = $0021;
	kImageCodecDisposeImageGWorldSelect = $0022;
	kImageCodecHitTestDataWithFlagsSelect = $0023;
	kImageCodecValidateParametersSelect = $0024;
	kImageCodecGetBaseMPWorkFunctionSelect = $0025;
	kImageCodecLockBitsSelect = $0026;
	kImageCodecUnlockBitsSelect = $0027;
	kImageCodecRequestGammaLevelSelect = $0028;
	kImageCodecGetSourceDataGammaLevelSelect = $0029;
	kImageCodecGetDecompressLatencySelect = $002B;
	kImageCodecMergeFloatingImageOntoWindowSelect = $002C;
	kImageCodecRemoveFloatingImageSelect = $002D;
	kImageCodecGetDITLForSizeSelect = $002E;
	kImageCodecDITLInstallSelect = $002F;
	kImageCodecDITLEventSelect = $0030;
	kImageCodecDITLItemSelect = $0031;
	kImageCodecDITLRemoveSelect = $0032;
	kImageCodecDITLValidateInputSelect = $0033;
	kImageCodecGetPreferredChunkSizeAndAlignmentSelect = $0036;
	kImageCodecPrepareToCompressFramesSelect = $0037;
	kImageCodecEncodeFrameSelect = $0038;
	kImageCodecCompleteFrameSelect = $0039;
	kImageCodecBeginPassSelect = $003A;
	kImageCodecEndPassSelect = $003B;
	kImageCodecProcessBetweenPassesSelect = $003C;
	kImageCodecPreflightSelect = $0200;
	kImageCodecInitializeSelect = $0201;
	kImageCodecBeginBandSelect = $0202;
	kImageCodecDrawBandSelect = $0203;
	kImageCodecEndBandSelect = $0204;
	kImageCodecQueueStartingSelect = $0205;
	kImageCodecQueueStoppingSelect = $0206;
	kImageCodecDroppingFrameSelect = $0207;
	kImageCodecScheduleFrameSelect = $0208;
	kImageCodecCancelTriggerSelect = $0209;
	kImageCodecDecodeBandSelect = $020F;


const
	kMotionJPEGTag = FourCharCode('mjpg');
	kJPEGQuantizationTablesImageDescriptionExtension = FourCharCode('mjqt');
	kJPEGHuffmanTablesImageDescriptionExtension = FourCharCode('mjht');
	kFieldInfoImageDescriptionExtension = FourCharCode('fiel'); { image description extension describing the field count and field orderings}

<<<<<<< HEAD
<<<<<<< HEAD
	kFieldOrderUnknown			= 0;
>>>>>>> graemeg/fixes_2_2
	kFieldsStoredF1F2DisplayedF1F2 = 1;
	kFieldsStoredF1F2DisplayedF2F1 = 2;
	kFieldsStoredF2F1DisplayedF1F2 = 5;
	kFieldsStoredF2F1DisplayedF2F1 = 6;

<<<<<<< HEAD
type
	MotionJPEGApp1MarkerPtr = ^MotionJPEGApp1Marker;
	MotionJPEGApp1Marker = record
		unused: SIGNEDLONG;
		tag: SIGNEDLONG;
		fieldSize: SIGNEDLONG;
		paddedFieldSize: SIGNEDLONG;
		offsetToNextField: SIGNEDLONG;
		qTableOffset: SIGNEDLONG;
		huffmanTableOffset: SIGNEDLONG;
		sofOffset: SIGNEDLONG;
		sosOffset: SIGNEDLONG;
		soiOffset: SIGNEDLONG;
	end;
type
	FieldInfoImageDescriptionExtensionPtr = ^FieldInfoImageDescriptionExtension;
	FieldInfoImageDescriptionExtension = record
		fieldCount: UInt8;
		fieldOrderings: UInt8;
	end;

{
 *  QTPhotoSetSampling()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTPhotoSetSampling( codec: ComponentInstance; yH: SInt16; yV: SInt16; cbH: SInt16; cbV: SInt16; crH: SInt16; crV: SInt16 ): ComponentResult; external name '_QTPhotoSetSampling';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
	kFieldOrderUnknown			= 0;
=======
const
	kFieldOrderUnknown = 0;
>>>>>>> origin/fixes_2.4
	kFieldsStoredF1F2DisplayedF1F2 = 1;
	kFieldsStoredF1F2DisplayedF2F1 = 2;
	kFieldsStoredF2F1DisplayedF1F2 = 5;
	kFieldsStoredF2F1DisplayedF2F1 = 6;

<<<<<<< HEAD
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
type
	MotionJPEGApp1MarkerPtr = ^MotionJPEGApp1Marker;
	MotionJPEGApp1Marker = record
		unused: SIGNEDLONG;
		tag: SIGNEDLONG;
		fieldSize: SIGNEDLONG;
		paddedFieldSize: SIGNEDLONG;
		offsetToNextField: SIGNEDLONG;
		qTableOffset: SIGNEDLONG;
		huffmanTableOffset: SIGNEDLONG;
		sofOffset: SIGNEDLONG;
		sosOffset: SIGNEDLONG;
		soiOffset: SIGNEDLONG;
	end;
type
	FieldInfoImageDescriptionExtensionPtr = ^FieldInfoImageDescriptionExtension;
	FieldInfoImageDescriptionExtension = record
		fieldCount: UInt8;
		fieldOrderings: UInt8;
	end;

{
 *  QTPhotoSetSampling()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function QTPhotoSetSampling( codec: ComponentInstance; yH: SInt16; yV: SInt16; cbH: SInt16; cbV: SInt16; crH: SInt16; crV: SInt16 ): ComponentResult; external name '_QTPhotoSetSampling';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
	{
	 *  QTPhotoSetSampling()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 2.5 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function QTPhotoSetSampling(codec: ComponentInstance; yH: SInt16; yV: SInt16; cbH: SInt16; cbV: SInt16; crH: SInt16; crV: SInt16): ComponentResult; external name '_QTPhotoSetSampling';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  QTPhotoSetRestartInterval()
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
function QTPhotoSetRestartInterval( codec: ComponentInstance; restartInterval: UInt16 ): ComponentResult; external name '_QTPhotoSetRestartInterval';
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
function QTPhotoSetRestartInterval(codec: ComponentInstance; restartInterval: UInt16): ComponentResult; external name '_QTPhotoSetRestartInterval';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTPhotoSetRestartInterval( codec: ComponentInstance; restartInterval: UInt16 ): ComponentResult; external name '_QTPhotoSetRestartInterval';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTPhotoDefineHuffmanTable()
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
function QTPhotoDefineHuffmanTable( codec: ComponentInstance; componentNumber: SInt16; isDC: Boolean; lengthCounts: UInt8Ptr; values: UInt8Ptr ): ComponentResult; external name '_QTPhotoDefineHuffmanTable';
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
function QTPhotoDefineHuffmanTable(codec: ComponentInstance; componentNumber: SInt16; isDC: boolean; lengthCounts: UInt8Ptr; values: UInt8Ptr): ComponentResult; external name '_QTPhotoDefineHuffmanTable';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTPhotoDefineHuffmanTable( codec: ComponentInstance; componentNumber: SInt16; isDC: Boolean; lengthCounts: UInt8Ptr; values: UInt8Ptr ): ComponentResult; external name '_QTPhotoDefineHuffmanTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  QTPhotoDefineQuantizationTable()
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
function QTPhotoDefineQuantizationTable( codec: ComponentInstance; componentNumber: SInt16; table: UInt8Ptr ): ComponentResult; external name '_QTPhotoDefineQuantizationTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ selectors for component calls }
const
	kQTPhotoSetSamplingSelect = $0100;
	kQTPhotoSetRestartIntervalSelect = $0101;
	kQTPhotoDefineHuffmanTableSelect = $0102;
	kQTPhotoDefineQuantizationTableSelect = $0103;

{
 *  Summary:
 *    Properties for image compressor components
 }
const
{
   * Property class for image compressor components.
   }
	kQTPropertyClass_ImageCompressor = FourCharCode('imco');


{
 *  Summary:
 *    Enforced properties for image compressor components
 *  
 *  Discussion:
 *    Image compressors that sometimes or always restrict image
 *    dimensions, clean aperture and/or pixel aspect ratio should
 *    support these properties. 
 *    If these properties can change dynamically for a compressor (eg,
 *    in response to user interaction) then the properties should be
 *    listenable, and the compressor should call the listeners whenever
 *    the properties change. (In this case, the component's
 *    GetComponentPropertyInfo function should set the
 *    kComponentPropertyFlagWillNotifyListeners flag.) 
 *    If a compressor has a mode in which these properties are
 *    flexible, then when the component is in that mode, (a) the
 *    component's GetComponentProperty function should return
 *    kQTPropertyAskLaterErr for these properties, and (b) the
 *    component's GetComponentPropertyInfo function should set the
 *    kComponentPropertyFlagCanGetLater flag for these properties.
 }
const
{
   * The encoded width enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedEncodedWidth = FourCharCode('enwi'); { SInt32, Read/Sometimes Listen }

  {
   * The encoded height enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedEncodedHeight = FourCharCode('enhe'); { SInt32, Read/Sometimes Listen }

  {
   * The clean aperture enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedCleanAperture = FourCharCode('encl'); { CleanApertureImageDescriptionExtension, Read/Sometimes Listen }

  {
   * The pixel aspect ratio enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedPixelAspectRatio = FourCharCode('enpa'); { PixelAspectRatioImageDescriptionExtension, Read/Sometimes Listen }

  {
   * The number and order of fields enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedFieldInfo = FourCharCode('enfi'); { FieldInfoImageDescriptionExtension2, Read/Sometimes Listen }


{
 *  Summary:
 *    DV Compressor Component Properties.
 }
const
{
   * Property class for DV compressors.  (Applicable to DV25, DV50,
   * NTSC, PAL, PROPAL.)
   }
	kQTPropertyClass_DVCompressor = FourCharCode('dvco');

  {
   * If set, indicates that the compressed frames should be marked as
   * progressive-scan. By default, this flag is clear, meaning that
   * frames should be marked as interlaced.
   }
	kDVCompressorPropertyID_ProgressiveScan = FourCharCode('prog'); { Boolean, Read/Write }

  {
   * If set, indicates that the compressor should use a 16:9 picture
   * aspect ratio. If clear, the compressor will use the default 4:3
   * picture aspect ratio.
   }
	kDVCompressorPropertyID_AspectRatio16x9 = FourCharCode('16x9'); { Boolean, Read/Write }


{ source identifier -- placed in root container of description, one or more required }
const
	kEffectSourceName = FourCharCode('src ');


{ source type -- placed in the input map to identify the source kind }
const
	kEffectDataSourceType = FourCharCode('dtst');

{  default effect types }
const
	kEffectRawSource = 0;    { the source is raw image data}
	kEffectGenericType = FourCharCode('geff'); { generic effect for combining others}
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
function QTPhotoDefineQuantizationTable( codec: ComponentInstance; componentNumber: SInt16; table: UInt8Ptr ): ComponentResult; external name '_QTPhotoDefineQuantizationTable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ selectors for component calls }
const
	kQTPhotoSetSamplingSelect = $0100;
	kQTPhotoSetRestartIntervalSelect = $0101;
	kQTPhotoDefineHuffmanTableSelect = $0102;
	kQTPhotoDefineQuantizationTableSelect = $0103;

{
 *  Summary:
 *    Properties for image compressor components
 }
const
{
   * Property class for image compressor components.
   }
	kQTPropertyClass_ImageCompressor = FourCharCode('imco');


{
 *  Summary:
 *    Enforced properties for image compressor components
 *  
 *  Discussion:
 *    Image compressors that sometimes or always restrict image
 *    dimensions, clean aperture and/or pixel aspect ratio should
 *    support these properties. 
 *    If these properties can change dynamically for a compressor (eg,
 *    in response to user interaction) then the properties should be
 *    listenable, and the compressor should call the listeners whenever
 *    the properties change. (In this case, the component's
 *    GetComponentPropertyInfo function should set the
 *    kComponentPropertyFlagWillNotifyListeners flag.) 
 *    If a compressor has a mode in which these properties are
 *    flexible, then when the component is in that mode, (a) the
 *    component's GetComponentProperty function should return
 *    kQTPropertyAskLaterErr for these properties, and (b) the
 *    component's GetComponentPropertyInfo function should set the
 *    kComponentPropertyFlagCanGetLater flag for these properties.
 }
const
{
   * The encoded width enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedEncodedWidth = FourCharCode('enwi'); { SInt32, Read/Sometimes Listen }

  {
   * The encoded height enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedEncodedHeight = FourCharCode('enhe'); { SInt32, Read/Sometimes Listen }

  {
   * The clean aperture enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedCleanAperture = FourCharCode('encl'); { CleanApertureImageDescriptionExtension, Read/Sometimes Listen }

  {
   * The pixel aspect ratio enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedPixelAspectRatio = FourCharCode('enpa'); { PixelAspectRatioImageDescriptionExtension, Read/Sometimes Listen }

  {
   * The number and order of fields enforced for compressed frames.
   }
	kICMImageCompressorPropertyID_EnforcedFieldInfo = FourCharCode('enfi'); { FieldInfoImageDescriptionExtension2, Read/Sometimes Listen }


{
 *  Summary:
 *    DV Compressor Component Properties.
 }
const
{
   * Property class for DV compressors.  (Applicable to DV25, DV50,
   * NTSC, PAL, PROPAL.)
   }
	kQTPropertyClass_DVCompressor = FourCharCode('dvco');

  {
   * If set, indicates that the compressed frames should be marked as
   * progressive-scan. By default, this flag is clear, meaning that
   * frames should be marked as interlaced.
   }
	kDVCompressorPropertyID_ProgressiveScan = FourCharCode('prog'); { Boolean, Read/Write }

  {
   * If set, indicates that the compressor should use a 16:9 picture
   * aspect ratio. If clear, the compressor will use the default 4:3
   * picture aspect ratio.
   }
	kDVCompressorPropertyID_AspectRatio16x9 = FourCharCode('16x9'); { Boolean, Read/Write }


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ source identifier -- placed in root container of description, one or more required }
const
	kEffectSourceName = FourCharCode('src ');


{ source type -- placed in the input map to identify the source kind }
const
	kEffectDataSourceType = FourCharCode('dtst');

{  default effect types }
const
	kEffectRawSource = 0;    { the source is raw image data}
	kEffectGenericType = FourCharCode('geff'); { generic effect for combining others}
>>>>>>> origin/fixes_2.4

type
	EffectSourcePtr = ^EffectSource;
	SourceDataPtr = ^SourceData;
	SourceData = record
		case SInt16 of
		0: (
			image:				CDSequenceDataSourcePtr;
			);
		1: (
			effect:				EffectSourcePtr;
			);
	end;


	EffectSource = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		effectType: SIGNEDLONG;             { type of effect or kEffectRawSource if raw ICM data}
		data: Ptr;                   { track data for this effect}
		source: SourceData;                 { source/effect pointers}
		next: EffectSourcePtr;                   { the next source for the parent effect}

                                              { fields added for QuickTime 4.0}
		lastTranslatedFrameTime: TimeValue; { start frame time of last converted frame, may be -1}
		lastFrameDuration: TimeValue;      { duration of the last converted frame, may be zero}
		lastFrameTimeScale: TimeValue;     { time scale of this source frame, only has meaning if above fields are valid}
<<<<<<< HEAD
	end;

type
	ICMFrameTimeRecord_QT3Ptr = ^ICMFrameTimeRecord_QT3;
	ICMFrameTimeRecord_QT3 = record
		value: wide;                  { frame display time}
		scale: SIGNEDLONG;                  { timescale of value/duration fields}
		base: UnivPtr;                   { timebase}

		duration: SIGNEDLONG;               { duration frame is to be displayed (0 if unknown)}
		rate: Fixed;                   { rate of timebase relative to wall-time}

		recordSize: SIGNEDLONG;             { total number of bytes in ICMFrameTimeRecord}

		frameNumber: SIGNEDLONG;            { number of frame, zero if not known}

		flags: SIGNEDLONG;

		virtualStartTime: wide;       { conceptual start time}
		virtualDuration: SIGNEDLONG;        { conceptual duration}
	end;
type
	EffectsFrameParamsPtr = ^EffectsFrameParams;
	EffectsFrameParams = record
		frameTime: ICMFrameTimeRecord_QT3;          { timing data (uses non-extended ICMFrameTimeRecord)}
		effectDuration: SIGNEDLONG;         { the duration of a single effect frame}
		doAsync: Boolean;                { set to true if the effect can go async}
		pad: array[0..2] of UInt8;
		source: EffectSourcePtr;                 { ptr to the source input tree}
		refCon: UnivPtr;                 { storage for the effect}
	end;

{
 *  ImageCodecEffectSetup()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function ImageCodecEffectSetup( effect: ComponentInstance; var p: CodecDecompressParams ): ComponentResult; external name '_ImageCodecEffectSetup';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
		effectType:				SInt32;								{  type of effect or kEffectRawSource if raw ICM data }
		data:					Ptr;									{  track data for this effect }
		source:					SourceData;								{  source/effect pointers }
		next:					EffectSourcePtr;						{  the next source for the parent effect }
																		{  fields added for QuickTime 4.0 }
		lastTranslatedFrameTime: TimeValue;								{  start frame time of last converted frame, may be -1 }
		lastFrameDuration:		TimeValue;								{  duration of the last converted frame, may be zero }
		lastFrameTimeScale:		TimeValue;								{  time scale of this source frame, only has meaning if above fields are valid }
=======
>>>>>>> origin/fixes_2.4
	end;

type
	ICMFrameTimeRecord_QT3Ptr = ^ICMFrameTimeRecord_QT3;
	ICMFrameTimeRecord_QT3 = record
		value: wide;                  { frame display time}
		scale: SIGNEDLONG;                  { timescale of value/duration fields}
		base: UnivPtr;                   { timebase}

		duration: SIGNEDLONG;               { duration frame is to be displayed (0 if unknown)}
		rate: Fixed;                   { rate of timebase relative to wall-time}

		recordSize: SIGNEDLONG;             { total number of bytes in ICMFrameTimeRecord}

		frameNumber: SIGNEDLONG;            { number of frame, zero if not known}

		flags: SIGNEDLONG;

		virtualStartTime: wide;       { conceptual start time}
		virtualDuration: SIGNEDLONG;        { conceptual duration}
	end;
type
	EffectsFrameParamsPtr = ^EffectsFrameParams;
	EffectsFrameParams = record
		frameTime: ICMFrameTimeRecord_QT3;          { timing data (uses non-extended ICMFrameTimeRecord)}
		effectDuration: SIGNEDLONG;         { the duration of a single effect frame}
		doAsync: Boolean;                { set to true if the effect can go async}
		pad: array[0..2] of UInt8;
		source: EffectSourcePtr;                 { ptr to the source input tree}
		refCon: UnivPtr;                 { storage for the effect}
	end;

{
 *  ImageCodecEffectSetup()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function ImageCodecEffectSetup( effect: ComponentInstance; var p: CodecDecompressParams ): ComponentResult; external name '_ImageCodecEffectSetup';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
	{
	 *  ImageCodecEffectSetup()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function ImageCodecEffectSetup(effect: ComponentInstance; var p: CodecDecompressParams): ComponentResult; external name '_ImageCodecEffectSetup';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  ImageCodecEffectBegin()
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
function ImageCodecEffectBegin( effect: ComponentInstance; var p: CodecDecompressParams; ePtr: EffectsFrameParamsPtr ): ComponentResult; external name '_ImageCodecEffectBegin';
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
function ImageCodecEffectBegin(effect: ComponentInstance; var p: CodecDecompressParams; ePtr: EffectsFrameParamsPtr): ComponentResult; external name '_ImageCodecEffectBegin';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecEffectBegin( effect: ComponentInstance; var p: CodecDecompressParams; ePtr: EffectsFrameParamsPtr ): ComponentResult; external name '_ImageCodecEffectBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecEffectRenderFrame()
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
function ImageCodecEffectRenderFrame( effect: ComponentInstance; p: EffectsFrameParamsPtr ): ComponentResult; external name '_ImageCodecEffectRenderFrame';
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
function ImageCodecEffectRenderFrame(effect: ComponentInstance; p: EffectsFrameParamsPtr): ComponentResult; external name '_ImageCodecEffectRenderFrame';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecEffectRenderFrame( effect: ComponentInstance; p: EffectsFrameParamsPtr ): ComponentResult; external name '_ImageCodecEffectRenderFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecEffectConvertEffectSourceToFormat()
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
function ImageCodecEffectConvertEffectSourceToFormat( effect: ComponentInstance; sourceToConvert: EffectSourcePtr; requestedDesc: ImageDescriptionHandle ): ComponentResult; external name '_ImageCodecEffectConvertEffectSourceToFormat';
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
function ImageCodecEffectConvertEffectSourceToFormat(effect: ComponentInstance; sourceToConvert: EffectSourcePtr; requestedDesc: ImageDescriptionHandle): ComponentResult; external name '_ImageCodecEffectConvertEffectSourceToFormat';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecEffectConvertEffectSourceToFormat( effect: ComponentInstance; sourceToConvert: EffectSourcePtr; requestedDesc: ImageDescriptionHandle ): ComponentResult; external name '_ImageCodecEffectConvertEffectSourceToFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecEffectCancel()
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
function ImageCodecEffectCancel( effect: ComponentInstance; p: EffectsFrameParamsPtr ): ComponentResult; external name '_ImageCodecEffectCancel';
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
function ImageCodecEffectCancel(effect: ComponentInstance; p: EffectsFrameParamsPtr): ComponentResult; external name '_ImageCodecEffectCancel';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecEffectCancel( effect: ComponentInstance; p: EffectsFrameParamsPtr ): ComponentResult; external name '_ImageCodecEffectCancel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecEffectGetSpeed()
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
function ImageCodecEffectGetSpeed( effect: ComponentInstance; parameters: QTAtomContainer; var pFPS: Fixed ): ComponentResult; external name '_ImageCodecEffectGetSpeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


const
	kSMPTENoFlag = 0;
	kSMPTESmoothEdgeFlag = $01; { smooth edges of the stroke}
	kSMPTEStrokeEdgeFlag = $02;  { stroke edge with color}

type
	SMPTEFlags = SIGNEDLONG;
	SMPTEFrameReference = SIGNEDLONG;
const
	kSlideHorizontalWipe = 1;
	kSlideVerticalWipe = 2;
	kTopLeftWipe = 3;
	kTopRightWipe = 4;
	kBottomRightWipe = 5;
	kBottomLeftWipe = 6;
	kFourCornerWipe = 7;
	kFourBoxWipe = 8;
	kBarnVerticalWipe = 21;
	kBarnHorizontalWipe = 22;
	kTopCenterWipe = 23;
	kRightCenterWipe = 24;
	kBottomCenterWipe = 25;
	kLeftCenterWipe = 26;
	kDiagonalLeftDownWipe = 41;
	kDiagonalRightDownWipe = 42;
	kTopBottomBowTieWipe = 43;
	kLeftRightBowTieWipe = 44;
	kDiagonalLeftOutWipe = 45;
	kDiagonalRightOutWipe = 46;
	kDiagonalCrossWipe = 47;
	kDiagonalBoxWipe = 48;
	kFilledVWipe = 61;
	kFilledVRightWipe = 62;
	kFilledVBottomWipe = 63;
	kFilledVLeftWipe = 64;
	kHollowVWipe = 65;
	kHollowVRightWipe = 66;
	kHollowVBottomWipe = 67;
	kHollowVLeftWipe = 68;
	kVerticalZigZagWipe = 71;
	kHorizontalZigZagWipe = 72;
	kVerticalBarnZigZagWipe = 73;
	kHorizontalBarnZigZagWipe = 74;

const
	kRectangleWipe = 101;
	kDiamondWipe = 102;
	kTriangleWipe = 103;
	kTriangleRightWipe = 104;
	kTriangleUpsideDownWipe = 105;
	kTriangleLeftWipe = 106;
	kSpaceShipWipe = 107;
	kSpaceShipRightWipe = 108;
	kSpaceShipUpsideDownWipe = 109;
	kSpaceShipLeftWipe = 110;
	kPentagonWipe = 111;
	kPentagonUpsideDownWipe = 112;
	kHexagonWipe = 113;
	kHexagonSideWipe = 114;
	kCircleWipe = 119;
	kOvalWipe = 120;
	kOvalSideWipe = 121;
	kCatEyeWipe = 122;
	kCatEyeSideWipe = 123;
	kRoundRectWipe = 124;
	kRoundRectSideWipe = 125;
	kFourPointStarWipe = 127;
	kFivePointStarWipe = 128;
	kStarOfDavidWipe = 129;
	kHeartWipe = 130;
	kKeyholeWipe = 131;

const
	kRotatingTopWipe = 201;
	kRotatingRightWipe = 202;
	kRotatingBottomWipe = 203;
	kRotatingLeftWipe = 204;
	kRotatingTopBottomWipe = 205;
	kRotatingLeftRightWipe = 206;
	kRotatingQuadrantWipe = 207;
	kTopToBottom180Wipe = 211;
	kRightToLeft180Wipe = 212;
	kTopToBottom90Wipe = 213;
	kRightToLeft90Wipe = 214;
	kTop180Wipe = 221;
	kRight180Wipe = 222;
	kBottom180Wipe = 223;
	kLeft180Wipe = 224;
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
function ImageCodecEffectGetSpeed( effect: ComponentInstance; parameters: QTAtomContainer; var pFPS: Fixed ): ComponentResult; external name '_ImageCodecEffectGetSpeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


const
	kSMPTENoFlag = 0;
	kSMPTESmoothEdgeFlag = $01; { smooth edges of the stroke}
	kSMPTEStrokeEdgeFlag = $02;  { stroke edge with color}

type
<<<<<<< HEAD
	SMPTEFlags							= SInt32;
	SMPTEFrameReference					= SInt32;

const
	kSlideHorizontalWipe		= 1;
	kSlideVerticalWipe			= 2;
	kTopLeftWipe				= 3;
	kTopRightWipe				= 4;
	kBottomRightWipe			= 5;
	kBottomLeftWipe				= 6;
	kFourCornerWipe				= 7;
	kFourBoxWipe				= 8;
	kBarnVerticalWipe			= 21;
	kBarnHorizontalWipe			= 22;
	kTopCenterWipe				= 23;
	kRightCenterWipe			= 24;
	kBottomCenterWipe			= 25;
	kLeftCenterWipe				= 26;
	kDiagonalLeftDownWipe		= 41;
	kDiagonalRightDownWipe		= 42;
	kTopBottomBowTieWipe		= 43;
	kLeftRightBowTieWipe		= 44;
	kDiagonalLeftOutWipe		= 45;
	kDiagonalRightOutWipe		= 46;
	kDiagonalCrossWipe			= 47;
	kDiagonalBoxWipe			= 48;
	kFilledVWipe				= 61;
	kFilledVRightWipe			= 62;
	kFilledVBottomWipe			= 63;
	kFilledVLeftWipe			= 64;
	kHollowVWipe				= 65;
	kHollowVRightWipe			= 66;
	kHollowVBottomWipe			= 67;
	kHollowVLeftWipe			= 68;
	kVerticalZigZagWipe			= 71;
	kHorizontalZigZagWipe		= 72;
	kVerticalBarnZigZagWipe		= 73;
	kHorizontalBarnZigZagWipe	= 74;

	kRectangleWipe				= 101;
	kDiamondWipe				= 102;
	kTriangleWipe				= 103;
	kTriangleRightWipe			= 104;
	kTriangleUpsideDownWipe		= 105;
	kTriangleLeftWipe			= 106;
	kSpaceShipWipe				= 107;
	kSpaceShipRightWipe			= 108;
	kSpaceShipUpsideDownWipe	= 109;
	kSpaceShipLeftWipe			= 110;
	kPentagonWipe				= 111;
	kPentagonUpsideDownWipe		= 112;
	kHexagonWipe				= 113;
	kHexagonSideWipe			= 114;
	kCircleWipe					= 119;
	kOvalWipe					= 120;
	kOvalSideWipe				= 121;
	kCatEyeWipe					= 122;
	kCatEyeSideWipe				= 123;
	kRoundRectWipe				= 124;
	kRoundRectSideWipe			= 125;
	kFourPointStarWipe			= 127;
	kFivePointStarWipe			= 128;
	kStarOfDavidWipe			= 129;
	kHeartWipe					= 130;
	kKeyholeWipe				= 131;

	kRotatingTopWipe			= 201;
	kRotatingRightWipe			= 202;
	kRotatingBottomWipe			= 203;
	kRotatingLeftWipe			= 204;
	kRotatingTopBottomWipe		= 205;
	kRotatingLeftRightWipe		= 206;
	kRotatingQuadrantWipe		= 207;
	kTopToBottom180Wipe			= 211;
	kRightToLeft180Wipe			= 212;
	kTopToBottom90Wipe			= 213;
	kRightToLeft90Wipe			= 214;
	kTop180Wipe					= 221;
	kRight180Wipe				= 222;
	kBottom180Wipe				= 223;
	kLeft180Wipe				= 224;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	SMPTEFlags = SIGNEDLONG;
	SMPTEFrameReference = SIGNEDLONG;
const
	kSlideHorizontalWipe = 1;
	kSlideVerticalWipe = 2;
	kTopLeftWipe = 3;
	kTopRightWipe = 4;
	kBottomRightWipe = 5;
	kBottomLeftWipe = 6;
	kFourCornerWipe = 7;
	kFourBoxWipe = 8;
	kBarnVerticalWipe = 21;
	kBarnHorizontalWipe = 22;
	kTopCenterWipe = 23;
	kRightCenterWipe = 24;
	kBottomCenterWipe = 25;
	kLeftCenterWipe = 26;
	kDiagonalLeftDownWipe = 41;
	kDiagonalRightDownWipe = 42;
	kTopBottomBowTieWipe = 43;
	kLeftRightBowTieWipe = 44;
	kDiagonalLeftOutWipe = 45;
	kDiagonalRightOutWipe = 46;
	kDiagonalCrossWipe = 47;
	kDiagonalBoxWipe = 48;
	kFilledVWipe = 61;
	kFilledVRightWipe = 62;
	kFilledVBottomWipe = 63;
	kFilledVLeftWipe = 64;
	kHollowVWipe = 65;
	kHollowVRightWipe = 66;
	kHollowVBottomWipe = 67;
	kHollowVLeftWipe = 68;
	kVerticalZigZagWipe = 71;
	kHorizontalZigZagWipe = 72;
	kVerticalBarnZigZagWipe = 73;
	kHorizontalBarnZigZagWipe = 74;

const
	kRectangleWipe = 101;
	kDiamondWipe = 102;
	kTriangleWipe = 103;
	kTriangleRightWipe = 104;
	kTriangleUpsideDownWipe = 105;
	kTriangleLeftWipe = 106;
	kSpaceShipWipe = 107;
	kSpaceShipRightWipe = 108;
	kSpaceShipUpsideDownWipe = 109;
	kSpaceShipLeftWipe = 110;
	kPentagonWipe = 111;
	kPentagonUpsideDownWipe = 112;
	kHexagonWipe = 113;
	kHexagonSideWipe = 114;
	kCircleWipe = 119;
	kOvalWipe = 120;
	kOvalSideWipe = 121;
	kCatEyeWipe = 122;
	kCatEyeSideWipe = 123;
	kRoundRectWipe = 124;
	kRoundRectSideWipe = 125;
	kFourPointStarWipe = 127;
	kFivePointStarWipe = 128;
	kStarOfDavidWipe = 129;
	kHeartWipe = 130;
	kKeyholeWipe = 131;

const
	kRotatingTopWipe = 201;
	kRotatingRightWipe = 202;
	kRotatingBottomWipe = 203;
	kRotatingLeftWipe = 204;
	kRotatingTopBottomWipe = 205;
	kRotatingLeftRightWipe = 206;
	kRotatingQuadrantWipe = 207;
	kTopToBottom180Wipe = 211;
	kRightToLeft180Wipe = 212;
	kTopToBottom90Wipe = 213;
	kRightToLeft90Wipe = 214;
	kTop180Wipe = 221;
	kRight180Wipe = 222;
	kBottom180Wipe = 223;
	kLeft180Wipe = 224;
>>>>>>> origin/fixes_2.4
	kCounterRotatingTopBottomWipe = 225;
	kCounterRotatingLeftRightWipe = 226;
	kDoubleRotatingTopBottomWipe = 227;
	kDoubleRotatingLeftRightWipe = 228;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kVOpenTopWipe = 231;
	kVOpenRightWipe = 232;
	kVOpenBottomWipe = 233;
	kVOpenLeftWipe = 234;
	kVOpenTopBottomWipe = 235;
	kVOpenLeftRightWipe = 236;
	kRotatingTopLeftWipe = 241;
	kRotatingBottomLeftWipe = 242;
	kRotatingBottomRightWipe = 243;
	kRotatingTopRightWipe = 244;
<<<<<<< HEAD
	kRotatingTopLeftBottomRightWipe = 245;
	kRotatingBottomLeftTopRightWipe = 246;
	kRotatingTopLeftRightWipe = 251;
	kRotatingLeftTopBottomWipe = 252;
	kRotatingBottomLeftRightWipe = 253;
	kRotatingRightTopBottomWipe = 254;
=======
=======
>>>>>>> origin/fixes_2_2
	kVOpenTopWipe				= 231;
	kVOpenRightWipe				= 232;
	kVOpenBottomWipe			= 233;
	kVOpenLeftWipe				= 234;
	kVOpenTopBottomWipe			= 235;
	kVOpenLeftRightWipe			= 236;
	kRotatingTopLeftWipe		= 241;
	kRotatingBottomLeftWipe		= 242;
	kRotatingBottomRightWipe	= 243;
	kRotatingTopRightWipe		= 244;
=======
>>>>>>> origin/fixes_2.4
	kRotatingTopLeftBottomRightWipe = 245;
	kRotatingBottomLeftTopRightWipe = 246;
	kRotatingTopLeftRightWipe = 251;
	kRotatingLeftTopBottomWipe = 252;
	kRotatingBottomLeftRightWipe = 253;
<<<<<<< HEAD
	kRotatingRightTopBottomWipe	= 254;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kRotatingRightTopBottomWipe = 254;
>>>>>>> origin/fixes_2.4
	kRotatingDoubleCenterRightWipe = 261;
	kRotatingDoubleCenterTopWipe = 262;
	kRotatingDoubleCenterTopBottomWipe = 263;
	kRotatingDoubleCenterLeftRightWipe = 264;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const
	kHorizontalMatrixWipe = 301;
	kVerticalMatrixWipe = 302;
	kTopLeftDiagonalMatrixWipe = 303;
	kTopRightDiagonalMatrixWipe = 304;
<<<<<<< HEAD
	kBottomRightDiagonalMatrixWipe = 305;
	kBottomLeftDiagonalMatrixWipe = 306;
	kClockwiseTopLeftMatrixWipe = 310;
=======
=======
>>>>>>> origin/fixes_2_2
	kHorizontalMatrixWipe		= 301;
	kVerticalMatrixWipe			= 302;
	kTopLeftDiagonalMatrixWipe	= 303;
	kTopRightDiagonalMatrixWipe	= 304;
	kBottomRightDiagonalMatrixWipe = 305;
	kBottomLeftDiagonalMatrixWipe = 306;
	kClockwiseTopLeftMatrixWipe	= 310;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kBottomRightDiagonalMatrixWipe = 305;
	kBottomLeftDiagonalMatrixWipe = 306;
	kClockwiseTopLeftMatrixWipe = 310;
>>>>>>> origin/fixes_2.4
	kClockwiseTopRightMatrixWipe = 311;
	kClockwiseBottomRightMatrixWipe = 312;
	kClockwiseBottomLeftMatrixWipe = 313;
	kCounterClockwiseTopLeftMatrixWipe = 314;
	kCounterClockwiseTopRightMatrixWipe = 315;
	kCounterClockwiseBottomRightMatrixWipe = 316;
	kCounterClockwiseBottomLeftMatrixWipe = 317;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kVerticalStartTopMatrixWipe = 320;
=======
	kVerticalStartTopMatrixWipe	= 320;
>>>>>>> graemeg/fixes_2_2
=======
	kVerticalStartTopMatrixWipe	= 320;
>>>>>>> origin/fixes_2_2
=======
	kVerticalStartTopMatrixWipe = 320;
>>>>>>> origin/fixes_2.4
	kVerticalStartBottomMatrixWipe = 321;
	kVerticalStartTopOppositeMatrixWipe = 322;
	kVerticalStartBottomOppositeMatrixWipe = 323;
	kHorizontalStartLeftMatrixWipe = 324;
	kHorizontalStartRightMatrixWipe = 325;
	kHorizontalStartLeftOppositeMatrixWipe = 326;
	kHorizontalStartRightOppositeMatrixWipe = 327;
	kDoubleDiagonalTopRightMatrixWipe = 328;
	kDoubleDiagonalBottomRightMatrixWipe = 329;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kDoubleSpiralTopMatixWipe = 340;
	kDoubleSpiralBottomMatixWipe = 341;
	kDoubleSpiralLeftMatixWipe = 342;
	kDoubleSpiralRightMatixWipe = 343;
=======
=======
>>>>>>> origin/fixes_2_2
	kDoubleSpiralTopMatixWipe	= 340;
	kDoubleSpiralBottomMatixWipe = 341;
	kDoubleSpiralLeftMatixWipe	= 342;
	kDoubleSpiralRightMatixWipe	= 343;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kDoubleSpiralTopMatixWipe = 340;
	kDoubleSpiralBottomMatixWipe = 341;
	kDoubleSpiralLeftMatixWipe = 342;
	kDoubleSpiralRightMatixWipe = 343;
>>>>>>> origin/fixes_2.4
	kQuadSpiralVerticalMatixWipe = 344;
	kQuadSpiralHorizontalMatixWipe = 345;
	kVerticalWaterfallLeftMatrixWipe = 350;
	kVerticalWaterfallRightMatrixWipe = 351;
	kHorizontalWaterfallLeftMatrixWipe = 352;
	kHorizontalWaterfallRightMatrixWipe = 353;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kRandomWipe = 409;  { non-SMPTE standard numbers}
	kRandomWipeGroupWipe = 501;
	kRandomIrisGroupWipe = 502;
	kRandomRadialGroupWipe = 503;
	kRandomMatrixGroupWipe = 504;
<<<<<<< HEAD

type
	SMPTEWipeType = UNSIGNEDLONG;
{
 *  ImageCodecEffectPrepareSMPTEFrame()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function ImageCodecEffectPrepareSMPTEFrame( effect: ComponentInstance; destPixMap: PixMapPtr; var returnValue: SMPTEFrameReference ): ComponentResult; external name '_ImageCodecEffectPrepareSMPTEFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
	kRandomWipe					= 409;							{  non-SMPTE standard numbers }
	kRandomWipeGroupWipe		= 501;
	kRandomIrisGroupWipe		= 502;
	kRandomRadialGroupWipe		= 503;
	kRandomMatrixGroupWipe		= 504;


type
	SMPTEWipeType						= UInt32;
	{
	 *  ImageCodecEffectPrepareSMPTEFrame()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
	 *    CarbonLib:        in CarbonLib 1.0.2 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 4.0 and later
	 	}
function ImageCodecEffectPrepareSMPTEFrame(effect: ComponentInstance; destPixMap: PixMapPtr; var returnValue: SMPTEFrameReference): ComponentResult; external name '_ImageCodecEffectPrepareSMPTEFrame';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

type
	SMPTEWipeType = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
{
 *  ImageCodecEffectPrepareSMPTEFrame()
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
function ImageCodecEffectDisposeSMPTEFrame( effect: ComponentInstance; frameRef: SMPTEFrameReference ): ComponentResult; external name '_ImageCodecEffectDisposeSMPTEFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
function ImageCodecEffectPrepareSMPTEFrame( effect: ComponentInstance; destPixMap: PixMapPtr; var returnValue: SMPTEFrameReference ): ComponentResult; external name '_ImageCodecEffectPrepareSMPTEFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ImageCodecEffectDisposeSMPTEFrame()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in QuickTimeLib 4.0 and later
 *    Windows:          in qtmlClient.lib 4.0 and later
 }
<<<<<<< HEAD
function ImageCodecEffectDisposeSMPTEFrame(effect: ComponentInstance; frameRef: SMPTEFrameReference): ComponentResult; external name '_ImageCodecEffectDisposeSMPTEFrame';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ImageCodecEffectDisposeSMPTEFrame( effect: ComponentInstance; frameRef: SMPTEFrameReference ): ComponentResult; external name '_ImageCodecEffectDisposeSMPTEFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  ImageCodecEffectRenderSMPTEFrame()
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
function ImageCodecEffectRenderSMPTEFrame( effect: ComponentInstance; destPixMap: PixMapPtr; frameRef: SMPTEFrameReference; effectPercentageEven: Fixed; effectPercentageOdd: Fixed; var pSourceRect: Rect; var matrixP: MatrixRecord; effectNumber: SMPTEWipeType; xRepeat: SIGNEDLONG; yRepeat: SIGNEDLONG; flags: SMPTEFlags; penWidth: Fixed; strokeValue: SIGNEDLONG ): ComponentResult; external name '_ImageCodecEffectRenderSMPTEFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ selectors for component calls }
const
	kImageCodecEffectSetupSelect = $0300;
	kImageCodecEffectBeginSelect = $0301;
	kImageCodecEffectRenderFrameSelect = $0302;
	kImageCodecEffectConvertEffectSourceToFormatSelect = $0303;
	kImageCodecEffectCancelSelect = $0304;
	kImageCodecEffectGetSpeedSelect = $0305;
	kImageCodecEffectPrepareSMPTEFrameSelect = $0100;
	kImageCodecEffectDisposeSMPTEFrameSelect = $0101;
	kImageCodecEffectRenderSMPTEFrameSelect = $0102;


{ curve atom types and data structures }
const
	kCurvePathAtom = FourCharCode('path');
	kCurveEndAtom = FourCharCode('zero');
	kCurveAntialiasControlAtom = FourCharCode('anti');
	kCurveAntialiasOff = 0;
	kCurveAntialiasOn = $FFFFFFFF;
	kCurveFillTypeAtom = FourCharCode('fill');
	kCurvePenThicknessAtom = FourCharCode('pent');
	kCurveMiterLimitAtom = FourCharCode('mitr');
	kCurveJoinAttributesAtom = FourCharCode('join');
	kCurveMinimumDepthAtom = FourCharCode('mind');
	kCurveDepthAlwaysOffscreenMask = $80000000;
	kCurveTransferModeAtom = FourCharCode('xfer');
	kCurveGradientAngleAtom = FourCharCode('angl');
	kCurveGradientRadiusAtom = FourCharCode('radi');
	kCurveGradientOffsetAtom = FourCharCode('cent');

const
	kCurveARGBColorAtom = FourCharCode('argb');
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
function ImageCodecEffectRenderSMPTEFrame( effect: ComponentInstance; destPixMap: PixMapPtr; frameRef: SMPTEFrameReference; effectPercentageEven: Fixed; effectPercentageOdd: Fixed; var pSourceRect: Rect; var matrixP: MatrixRecord; effectNumber: SMPTEWipeType; xRepeat: SIGNEDLONG; yRepeat: SIGNEDLONG; flags: SMPTEFlags; penWidth: Fixed; strokeValue: SIGNEDLONG ): ComponentResult; external name '_ImageCodecEffectRenderSMPTEFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ selectors for component calls }
const
	kImageCodecEffectSetupSelect = $0300;
	kImageCodecEffectBeginSelect = $0301;
	kImageCodecEffectRenderFrameSelect = $0302;
	kImageCodecEffectConvertEffectSourceToFormatSelect = $0303;
	kImageCodecEffectCancelSelect = $0304;
	kImageCodecEffectGetSpeedSelect = $0305;
	kImageCodecEffectPrepareSMPTEFrameSelect = $0100;
	kImageCodecEffectDisposeSMPTEFrameSelect = $0101;
	kImageCodecEffectRenderSMPTEFrameSelect = $0102;


{ curve atom types and data structures }
const
	kCurvePathAtom = FourCharCode('path');
	kCurveEndAtom = FourCharCode('zero');
	kCurveAntialiasControlAtom = FourCharCode('anti');
	kCurveAntialiasOff = 0;
	kCurveAntialiasOn = $FFFFFFFF;
	kCurveFillTypeAtom = FourCharCode('fill');
	kCurvePenThicknessAtom = FourCharCode('pent');
	kCurveMiterLimitAtom = FourCharCode('mitr');
	kCurveJoinAttributesAtom = FourCharCode('join');
	kCurveMinimumDepthAtom = FourCharCode('mind');
	kCurveDepthAlwaysOffscreenMask = $80000000;
	kCurveTransferModeAtom = FourCharCode('xfer');
	kCurveGradientAngleAtom = FourCharCode('angl');
	kCurveGradientRadiusAtom = FourCharCode('radi');
	kCurveGradientOffsetAtom = FourCharCode('cent');

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kCurveARGBColorAtom = FourCharCode('argb');
>>>>>>> origin/fixes_2.4

type
	ARGBColorPtr = ^ARGBColor;
	ARGBColor = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		alpha: UInt16;
		red: UInt16;
		green: UInt16;
		blue: UInt16;
<<<<<<< HEAD
	end;
const
	kCurveGradientRecordAtom = FourCharCode('grad');
=======
=======
>>>>>>> origin/fixes_2_2
		alpha:					UInt16;
		red:					UInt16;
		green:					UInt16;
		blue:					UInt16;
=======
>>>>>>> origin/fixes_2.4
	end;
const
<<<<<<< HEAD
	kCurveGradientRecordAtom	= FourCharCode('grad');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kCurveGradientRecordAtom = FourCharCode('grad');
>>>>>>> origin/fixes_2.4

type
	GradientColorRecordPtr = ^GradientColorRecord;
	GradientColorRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		thisColor: ARGBColor;
		endingPercentage: Fixed;
	end;
type
	GradientColorPtr = GradientColorRecordPtr;
const
	kCurveGradientTypeAtom = FourCharCode('grdt');

{ currently supported gradient types }
const
	kLinearGradient = 0;
	kCircularGradient = 1;

type
	GradientType = SIGNEDLONG;
{
 *  CurveGetLength()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CurveGetLength( effect: ComponentInstance; var target: gxPaths; index: SIGNEDLONG; var wideLength: wide ): ComponentResult; external name '_CurveGetLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
		thisColor:				ARGBColor;
		endingPercentage:		Fixed;
=======
		thisColor: ARGBColor;
		endingPercentage: Fixed;
>>>>>>> origin/fixes_2.4
	end;
type
	GradientColorPtr = GradientColorRecordPtr;
const
	kCurveGradientTypeAtom = FourCharCode('grdt');

{ currently supported gradient types }
const
	kLinearGradient = 0;
	kCircularGradient = 1;

type
	GradientType = SIGNEDLONG;
{
 *  CurveGetLength()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CurveGetLength( effect: ComponentInstance; var target: gxPaths; index: SIGNEDLONG; var wideLength: wide ): ComponentResult; external name '_CurveGetLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
type
	GradientType						= SInt32;
	{
	 *  CurveGetLength()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in QuickTimeLib 3.0 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 *    Windows:          in qtmlClient.lib 3.0 and later
	 	}
function CurveGetLength(effect: ComponentInstance; var target: gxPaths; index: SInt32; var wideLength: wide): ComponentResult; external name '_CurveGetLength';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  CurveLengthToPoint()
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
function CurveLengthToPoint( effect: ComponentInstance; var target: gxPaths; index: SIGNEDLONG; length: Fixed; var location: FixedPoint; var tangent: FixedPoint ): ComponentResult; external name '_CurveLengthToPoint';
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
function CurveLengthToPoint(effect: ComponentInstance; var target: gxPaths; index: SInt32; length: Fixed; var location: FixedPoint; var tangent: FixedPoint): ComponentResult; external name '_CurveLengthToPoint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveLengthToPoint( effect: ComponentInstance; var target: gxPaths; index: SIGNEDLONG; length: Fixed; var location: FixedPoint; var tangent: FixedPoint ): ComponentResult; external name '_CurveLengthToPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveNewPath()
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
function CurveNewPath( effect: ComponentInstance; var pPath: Handle ): ComponentResult; external name '_CurveNewPath';
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
function CurveNewPath(effect: ComponentInstance; var pPath: Handle): ComponentResult; external name '_CurveNewPath';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveNewPath( effect: ComponentInstance; var pPath: Handle ): ComponentResult; external name '_CurveNewPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveCountPointsInPath()
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
function CurveCountPointsInPath( effect: ComponentInstance; var aPath: gxPaths; contourIndex: UNSIGNEDLONG; var pCount: UNSIGNEDLONG ): ComponentResult; external name '_CurveCountPointsInPath';
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
function CurveCountPointsInPath(effect: ComponentInstance; var aPath: gxPaths; contourIndex: UInt32; var pCount: UInt32): ComponentResult; external name '_CurveCountPointsInPath';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveCountPointsInPath( effect: ComponentInstance; var aPath: gxPaths; contourIndex: UNSIGNEDLONG; var pCount: UNSIGNEDLONG ): ComponentResult; external name '_CurveCountPointsInPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveGetPathPoint()
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
function CurveGetPathPoint( effect: ComponentInstance; var aPath: gxPaths; contourIndex: UNSIGNEDLONG; pointIndex: UNSIGNEDLONG; var thePoint: gxPoint; var ptIsOnPath: Boolean ): ComponentResult; external name '_CurveGetPathPoint';
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
function CurveGetPathPoint(effect: ComponentInstance; var aPath: gxPaths; contourIndex: UInt32; pointIndex: UInt32; var thePoint: gxPoint; var ptIsOnPath: boolean): ComponentResult; external name '_CurveGetPathPoint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveGetPathPoint( effect: ComponentInstance; var aPath: gxPaths; contourIndex: UNSIGNEDLONG; pointIndex: UNSIGNEDLONG; var thePoint: gxPoint; var ptIsOnPath: Boolean ): ComponentResult; external name '_CurveGetPathPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveInsertPointIntoPath()
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
function CurveInsertPointIntoPath( effect: ComponentInstance; var aPoint: gxPoint; thePath: Handle; contourIndex: UNSIGNEDLONG; pointIndex: UNSIGNEDLONG; ptIsOnPath: Boolean ): ComponentResult; external name '_CurveInsertPointIntoPath';
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
function CurveInsertPointIntoPath(effect: ComponentInstance; var aPoint: gxPoint; thePath: Handle; contourIndex: UInt32; pointIndex: UInt32; ptIsOnPath: boolean): ComponentResult; external name '_CurveInsertPointIntoPath';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveInsertPointIntoPath( effect: ComponentInstance; var aPoint: gxPoint; thePath: Handle; contourIndex: UNSIGNEDLONG; pointIndex: UNSIGNEDLONG; ptIsOnPath: Boolean ): ComponentResult; external name '_CurveInsertPointIntoPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveSetPathPoint()
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
function CurveSetPathPoint( effect: ComponentInstance; var aPath: gxPaths; contourIndex: UNSIGNEDLONG; pointIndex: UNSIGNEDLONG; var thePoint: gxPoint; ptIsOnPath: Boolean ): ComponentResult; external name '_CurveSetPathPoint';
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
function CurveSetPathPoint(effect: ComponentInstance; var aPath: gxPaths; contourIndex: UInt32; pointIndex: UInt32; var thePoint: gxPoint; ptIsOnPath: boolean): ComponentResult; external name '_CurveSetPathPoint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveSetPathPoint( effect: ComponentInstance; var aPath: gxPaths; contourIndex: UNSIGNEDLONG; pointIndex: UNSIGNEDLONG; var thePoint: gxPoint; ptIsOnPath: Boolean ): ComponentResult; external name '_CurveSetPathPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveGetNearestPathPoint()
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
function CurveGetNearestPathPoint( effect: ComponentInstance; var aPath: gxPaths; var thePoint: FixedPoint; var contourIndex: UNSIGNEDLONG; var pointIndex: UNSIGNEDLONG; var theDelta: Fixed ): ComponentResult; external name '_CurveGetNearestPathPoint';
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
function CurveGetNearestPathPoint(effect: ComponentInstance; var aPath: gxPaths; var thePoint: FixedPoint; var contourIndex: UInt32; var pointIndex: UInt32; var theDelta: Fixed): ComponentResult; external name '_CurveGetNearestPathPoint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveGetNearestPathPoint( effect: ComponentInstance; var aPath: gxPaths; var thePoint: FixedPoint; var contourIndex: UNSIGNEDLONG; var pointIndex: UNSIGNEDLONG; var theDelta: Fixed ): ComponentResult; external name '_CurveGetNearestPathPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurvePathPointToLength()
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
function CurvePathPointToLength( ci: ComponentInstance; var aPath: gxPaths; startDist: Fixed; endDist: Fixed; var thePoint: FixedPoint; var pLength: Fixed ): ComponentResult; external name '_CurvePathPointToLength';
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
function CurvePathPointToLength(ci: ComponentInstance; var aPath: gxPaths; startDist: Fixed; endDist: Fixed; var thePoint: FixedPoint; var pLength: Fixed): ComponentResult; external name '_CurvePathPointToLength';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurvePathPointToLength( ci: ComponentInstance; var aPath: gxPaths; startDist: Fixed; endDist: Fixed; var thePoint: FixedPoint; var pLength: Fixed ): ComponentResult; external name '_CurvePathPointToLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveCreateVectorStream()
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
function CurveCreateVectorStream( effect: ComponentInstance; var pStream: Handle ): ComponentResult; external name '_CurveCreateVectorStream';
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
function CurveCreateVectorStream(effect: ComponentInstance; var pStream: Handle): ComponentResult; external name '_CurveCreateVectorStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveCreateVectorStream( effect: ComponentInstance; var pStream: Handle ): ComponentResult; external name '_CurveCreateVectorStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveAddAtomToVectorStream()
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
function CurveAddAtomToVectorStream( effect: ComponentInstance; atomType: OSType; atomSize: Size; pAtomData: UnivPtr; vectorStream: Handle ): ComponentResult; external name '_CurveAddAtomToVectorStream';
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
function CurveAddAtomToVectorStream(effect: ComponentInstance; atomType: OSType; atomSize: Size; pAtomData: UnivPtr; vectorStream: Handle): ComponentResult; external name '_CurveAddAtomToVectorStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveAddAtomToVectorStream( effect: ComponentInstance; atomType: OSType; atomSize: Size; pAtomData: UnivPtr; vectorStream: Handle ): ComponentResult; external name '_CurveAddAtomToVectorStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveAddPathAtomToVectorStream()
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
function CurveAddPathAtomToVectorStream( effect: ComponentInstance; pathData: Handle; vectorStream: Handle ): ComponentResult; external name '_CurveAddPathAtomToVectorStream';
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
function CurveAddPathAtomToVectorStream(effect: ComponentInstance; pathData: Handle; vectorStream: Handle): ComponentResult; external name '_CurveAddPathAtomToVectorStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveAddPathAtomToVectorStream( effect: ComponentInstance; pathData: Handle; vectorStream: Handle ): ComponentResult; external name '_CurveAddPathAtomToVectorStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveAddZeroAtomToVectorStream()
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
function CurveAddZeroAtomToVectorStream( effect: ComponentInstance; vectorStream: Handle ): ComponentResult; external name '_CurveAddZeroAtomToVectorStream';
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
function CurveAddZeroAtomToVectorStream(effect: ComponentInstance; vectorStream: Handle): ComponentResult; external name '_CurveAddZeroAtomToVectorStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CurveAddZeroAtomToVectorStream( effect: ComponentInstance; vectorStream: Handle ): ComponentResult; external name '_CurveAddZeroAtomToVectorStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CurveGetAtomDataFromVectorStream()
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
function CurveGetAtomDataFromVectorStream( effect: ComponentInstance; vectorStream: Handle; atomType: SIGNEDLONG; var dataSize: SIGNEDLONG; var dataPtr: Ptr ): ComponentResult; external name '_CurveGetAtomDataFromVectorStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ selectors for component calls }
const
	kCurveGetLengthSelect = $0100;
	kCurveLengthToPointSelect = $0101;
	kCurveNewPathSelect = $0102;
	kCurveCountPointsInPathSelect = $0103;
	kCurveGetPathPointSelect = $0104;
	kCurveInsertPointIntoPathSelect = $0105;
	kCurveSetPathPointSelect = $0106;
	kCurveGetNearestPathPointSelect = $0107;
	kCurvePathPointToLengthSelect = $0108;
	kCurveCreateVectorStreamSelect = $0109;
	kCurveAddAtomToVectorStreamSelect = $010A;
	kCurveAddPathAtomToVectorStreamSelect = $010B;
	kCurveAddZeroAtomToVectorStreamSelect = $010C;
	kCurveGetAtomDataFromVectorStreamSelect = $010D;
{ UPP call backs }

{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
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
function CurveGetAtomDataFromVectorStream( effect: ComponentInstance; vectorStream: Handle; atomType: SIGNEDLONG; var dataSize: SIGNEDLONG; var dataPtr: Ptr ): ComponentResult; external name '_CurveGetAtomDataFromVectorStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ selectors for component calls }
const
	kCurveGetLengthSelect = $0100;
	kCurveLengthToPointSelect = $0101;
	kCurveNewPathSelect = $0102;
	kCurveCountPointsInPathSelect = $0103;
	kCurveGetPathPointSelect = $0104;
	kCurveInsertPointIntoPathSelect = $0105;
	kCurveSetPathPointSelect = $0106;
	kCurveGetNearestPathPointSelect = $0107;
	kCurvePathPointToLengthSelect = $0108;
	kCurveCreateVectorStreamSelect = $0109;
	kCurveAddAtomToVectorStreamSelect = $010A;
	kCurveAddPathAtomToVectorStreamSelect = $010B;
	kCurveAddZeroAtomToVectorStreamSelect = $010C;
	kCurveGetAtomDataFromVectorStreamSelect = $010D;
{ UPP call backs }

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
