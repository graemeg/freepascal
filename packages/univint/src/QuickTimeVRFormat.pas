{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/QuickTimeVRFormat.h
 
     Contains:   QuickTime VR interfaces
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    QuickTime 7.7.1
 
     Copyright:  © 1997-2012 by Apple Inc., all rights reserved.
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       QuickTimeVRFormat.p
=======
     File:       QuickTime/QuickTimeVRFormat.h
>>>>>>> origin/fixes_2.4
 
     Contains:   QuickTime VR interfaces
 
     Version:    QuickTime 7.6.3
 
<<<<<<< HEAD
     Copyright:  © 1997-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1997-2008 by Apple Inc., all rights reserved.
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

unit QuickTimeVRFormat;
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
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
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
=======
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
uses MacTypes,Movies,QuickTimeVR;
{$endc} {not MACOSALLINCLUDE}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc not TARGET_CPU_64}

<<<<<<< HEAD

{ File Format Version numbers }
const
	kQTVRMajorVersion = 2;
const
	kQTVRMinorVersion = 0;

{ User data type for the Movie Controller type specifier}
const
	kQTControllerType = kQTVRControllerSubType; { Atom & ID of where our}
	kQTControllerID = 1;     { ...controller name is stored}

{ VRWorld atom types}
const
	kQTVRWorldHeaderAtomType = FourCharCode('vrsc');
	kQTVRImagingParentAtomType = FourCharCode('imgp');
	kQTVRPanoImagingAtomType = FourCharCode('impn');
	kQTVRObjectImagingAtomType = FourCharCode('imob');
	kQTVRNodeParentAtomType = FourCharCode('vrnp');
	kQTVRNodeIDAtomType = FourCharCode('vrni');
	kQTVRNodeLocationAtomType = FourCharCode('nloc');
	kQTVRCursorParentAtomType = FourCharCode('vrcp'); { New with 2.1}
	kQTVRCursorAtomType = FourCharCode('CURS'); { New with 2.1}
	kQTVRColorCursorAtomType = FourCharCode('crsr'); { New with 2.1}

{ NodeInfo atom types}
const
	kQTVRNodeHeaderAtomType = FourCharCode('ndhd');
	kQTVRHotSpotParentAtomType = FourCharCode('hspa');
	kQTVRHotSpotAtomType = FourCharCode('hots');
	kQTVRHotSpotInfoAtomType = FourCharCode('hsin');
	kQTVRLinkInfoAtomType = FourCharCode('link');

{ Miscellaneous atom types}
const
	kQTVRStringAtomType = FourCharCode('vrsg');
	kQTVRStringEncodingAtomType = FourCharCode('vrse'); { New with 2.1}
	kQTVRPanoSampleDataAtomType = FourCharCode('pdat');
	kQTVRObjectInfoAtomType = FourCharCode('obji');
	kQTVRImageTrackRefAtomType = FourCharCode('imtr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRHotSpotTrackRefAtomType = FourCharCode('hstr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRAngleRangeAtomType = FourCharCode('arng');
	kQTVRTrackRefArrayAtomType = FourCharCode('tref');
	kQTVRPanConstraintAtomType = FourCharCode('pcon');
	kQTVRTiltConstraintAtomType = FourCharCode('tcon');
	kQTVRFOVConstraintAtomType = FourCharCode('fcon');
	kQTVRCubicViewAtomType = FourCharCode('cuvw'); { New with 5.0}
	kQTVRCubicFaceDataAtomType = FourCharCode('cufa'); { New with 5.0}

const
	kQTVRObjectInfoAtomID = 1;
	kQTVRObjectImageTrackRefAtomID = 1;   { New with 2.1, it adds a track reference to select between multiple image tracks}
	kQTVRObjectHotSpotTrackRefAtomID = 1;  { New with 2.1, it adds a track reference to select between multiple hotspot tracks}

{ Track reference types}
const
	kQTVRImageTrackRefType = FourCharCode('imgt');
	kQTVRHotSpotTrackRefType = FourCharCode('hott');

{ Old hot spot types}
const
	kQTVRHotSpotNavigableType = FourCharCode('navg');

{ Valid bits used in QTVRLinkHotSpotAtom}
const
	kQTVRValidPan = 1 shl 0;
	kQTVRValidTilt = 1 shl 1;
	kQTVRValidFOV = 1 shl 2;
	kQTVRValidViewCenter = 1 shl 3;

=======

{ File Format Version numbers }
const
	kQTVRMajorVersion = 2;
const
	kQTVRMinorVersion = 0;

=======

{$ifc not TARGET_CPU_64}


{ File Format Version numbers }
const
	kQTVRMajorVersion = 2;
const
	kQTVRMinorVersion = 0;

>>>>>>> graemeg/cpstrnew
{ User data type for the Movie Controller type specifier}
const
	kQTControllerType = kQTVRControllerSubType; { Atom & ID of where our}
	kQTControllerID = 1;     { ...controller name is stored}

{ VRWorld atom types}
const
	kQTVRWorldHeaderAtomType = FourCharCode('vrsc');
	kQTVRImagingParentAtomType = FourCharCode('imgp');
	kQTVRPanoImagingAtomType = FourCharCode('impn');
	kQTVRObjectImagingAtomType = FourCharCode('imob');
	kQTVRNodeParentAtomType = FourCharCode('vrnp');
	kQTVRNodeIDAtomType = FourCharCode('vrni');
	kQTVRNodeLocationAtomType = FourCharCode('nloc');
	kQTVRCursorParentAtomType = FourCharCode('vrcp'); { New with 2.1}
	kQTVRCursorAtomType = FourCharCode('CURS'); { New with 2.1}
	kQTVRColorCursorAtomType = FourCharCode('crsr'); { New with 2.1}

{ NodeInfo atom types}
const
	kQTVRNodeHeaderAtomType = FourCharCode('ndhd');
	kQTVRHotSpotParentAtomType = FourCharCode('hspa');
	kQTVRHotSpotAtomType = FourCharCode('hots');
	kQTVRHotSpotInfoAtomType = FourCharCode('hsin');
	kQTVRLinkInfoAtomType = FourCharCode('link');

{ Miscellaneous atom types}
const
	kQTVRStringAtomType = FourCharCode('vrsg');
	kQTVRStringEncodingAtomType = FourCharCode('vrse'); { New with 2.1}
	kQTVRPanoSampleDataAtomType = FourCharCode('pdat');
	kQTVRObjectInfoAtomType = FourCharCode('obji');
	kQTVRImageTrackRefAtomType = FourCharCode('imtr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRHotSpotTrackRefAtomType = FourCharCode('hstr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRAngleRangeAtomType = FourCharCode('arng');
	kQTVRTrackRefArrayAtomType = FourCharCode('tref');
	kQTVRPanConstraintAtomType = FourCharCode('pcon');
	kQTVRTiltConstraintAtomType = FourCharCode('tcon');
	kQTVRFOVConstraintAtomType = FourCharCode('fcon');
	kQTVRCubicViewAtomType = FourCharCode('cuvw'); { New with 5.0}
	kQTVRCubicFaceDataAtomType = FourCharCode('cufa'); { New with 5.0}

const
	kQTVRObjectInfoAtomID = 1;
	kQTVRObjectImageTrackRefAtomID = 1;   { New with 2.1, it adds a track reference to select between multiple image tracks}
	kQTVRObjectHotSpotTrackRefAtomID = 1;  { New with 2.1, it adds a track reference to select between multiple hotspot tracks}

{ Track reference types}
const
	kQTVRImageTrackRefType = FourCharCode('imgt');
	kQTVRHotSpotTrackRefType = FourCharCode('hott');

{ Old hot spot types}
const
	kQTVRHotSpotNavigableType = FourCharCode('navg');

=======

{$ifc not TARGET_CPU_64}


{ File Format Version numbers }
const
	kQTVRMajorVersion = 2;
const
	kQTVRMinorVersion = 0;

{ User data type for the Movie Controller type specifier}
const
	kQTControllerType = kQTVRControllerSubType; { Atom & ID of where our}
	kQTControllerID = 1;     { ...controller name is stored}

{ VRWorld atom types}
const
	kQTVRWorldHeaderAtomType = FourCharCode('vrsc');
	kQTVRImagingParentAtomType = FourCharCode('imgp');
	kQTVRPanoImagingAtomType = FourCharCode('impn');
	kQTVRObjectImagingAtomType = FourCharCode('imob');
	kQTVRNodeParentAtomType = FourCharCode('vrnp');
	kQTVRNodeIDAtomType = FourCharCode('vrni');
	kQTVRNodeLocationAtomType = FourCharCode('nloc');
	kQTVRCursorParentAtomType = FourCharCode('vrcp'); { New with 2.1}
	kQTVRCursorAtomType = FourCharCode('CURS'); { New with 2.1}
	kQTVRColorCursorAtomType = FourCharCode('crsr'); { New with 2.1}

{ NodeInfo atom types}
const
	kQTVRNodeHeaderAtomType = FourCharCode('ndhd');
	kQTVRHotSpotParentAtomType = FourCharCode('hspa');
	kQTVRHotSpotAtomType = FourCharCode('hots');
	kQTVRHotSpotInfoAtomType = FourCharCode('hsin');
	kQTVRLinkInfoAtomType = FourCharCode('link');

{ Miscellaneous atom types}
const
	kQTVRStringAtomType = FourCharCode('vrsg');
	kQTVRStringEncodingAtomType = FourCharCode('vrse'); { New with 2.1}
	kQTVRPanoSampleDataAtomType = FourCharCode('pdat');
	kQTVRObjectInfoAtomType = FourCharCode('obji');
	kQTVRImageTrackRefAtomType = FourCharCode('imtr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRHotSpotTrackRefAtomType = FourCharCode('hstr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRAngleRangeAtomType = FourCharCode('arng');
	kQTVRTrackRefArrayAtomType = FourCharCode('tref');
	kQTVRPanConstraintAtomType = FourCharCode('pcon');
	kQTVRTiltConstraintAtomType = FourCharCode('tcon');
	kQTVRFOVConstraintAtomType = FourCharCode('fcon');
	kQTVRCubicViewAtomType = FourCharCode('cuvw'); { New with 5.0}
	kQTVRCubicFaceDataAtomType = FourCharCode('cufa'); { New with 5.0}

const
	kQTVRObjectInfoAtomID = 1;
	kQTVRObjectImageTrackRefAtomID = 1;   { New with 2.1, it adds a track reference to select between multiple image tracks}
	kQTVRObjectHotSpotTrackRefAtomID = 1;  { New with 2.1, it adds a track reference to select between multiple hotspot tracks}

{ Track reference types}
const
	kQTVRImageTrackRefType = FourCharCode('imgt');
	kQTVRHotSpotTrackRefType = FourCharCode('hott');

{ Old hot spot types}
const
	kQTVRHotSpotNavigableType = FourCharCode('navg');

>>>>>>> graemeg/cpstrnew
{ Valid bits used in QTVRLinkHotSpotAtom}
const
	kQTVRValidPan = 1 shl 0;
	kQTVRValidTilt = 1 shl 1;
	kQTVRValidFOV = 1 shl 2;
	kQTVRValidViewCenter = 1 shl 3;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{ Values for flags field in QTVRPanoSampleAtom}
const
	kQTVRPanoFlagHorizontal = 1 shl 0;
	kQTVRPanoFlagLast = 1 shl 31;


{ Values for locationFlags field in QTVRNodeLocationAtom}
const
	kQTVRSameFile = 0;


{$endc} {not TARGET_CPU_64}

=======

{$ifc not TARGET_CPU_64}


{ File Format Version numbers }
const
	kQTVRMajorVersion = 2;
const
	kQTVRMinorVersion = 0;

{ User data type for the Movie Controller type specifier}
const
	kQTControllerType = kQTVRControllerSubType; { Atom & ID of where our}
	kQTControllerID = 1;     { ...controller name is stored}

{ VRWorld atom types}
const
	kQTVRWorldHeaderAtomType = FourCharCode('vrsc');
	kQTVRImagingParentAtomType = FourCharCode('imgp');
	kQTVRPanoImagingAtomType = FourCharCode('impn');
	kQTVRObjectImagingAtomType = FourCharCode('imob');
	kQTVRNodeParentAtomType = FourCharCode('vrnp');
	kQTVRNodeIDAtomType = FourCharCode('vrni');
	kQTVRNodeLocationAtomType = FourCharCode('nloc');
	kQTVRCursorParentAtomType = FourCharCode('vrcp'); { New with 2.1}
	kQTVRCursorAtomType = FourCharCode('CURS'); { New with 2.1}
	kQTVRColorCursorAtomType = FourCharCode('crsr'); { New with 2.1}

{ NodeInfo atom types}
const
	kQTVRNodeHeaderAtomType = FourCharCode('ndhd');
	kQTVRHotSpotParentAtomType = FourCharCode('hspa');
	kQTVRHotSpotAtomType = FourCharCode('hots');
	kQTVRHotSpotInfoAtomType = FourCharCode('hsin');
	kQTVRLinkInfoAtomType = FourCharCode('link');

{ Miscellaneous atom types}
const
	kQTVRStringAtomType = FourCharCode('vrsg');
	kQTVRStringEncodingAtomType = FourCharCode('vrse'); { New with 2.1}
	kQTVRPanoSampleDataAtomType = FourCharCode('pdat');
	kQTVRObjectInfoAtomType = FourCharCode('obji');
	kQTVRImageTrackRefAtomType = FourCharCode('imtr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRHotSpotTrackRefAtomType = FourCharCode('hstr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRAngleRangeAtomType = FourCharCode('arng');
	kQTVRTrackRefArrayAtomType = FourCharCode('tref');
	kQTVRPanConstraintAtomType = FourCharCode('pcon');
	kQTVRTiltConstraintAtomType = FourCharCode('tcon');
	kQTVRFOVConstraintAtomType = FourCharCode('fcon');
	kQTVRCubicViewAtomType = FourCharCode('cuvw'); { New with 5.0}
	kQTVRCubicFaceDataAtomType = FourCharCode('cufa'); { New with 5.0}

const
	kQTVRObjectInfoAtomID = 1;
	kQTVRObjectImageTrackRefAtomID = 1;   { New with 2.1, it adds a track reference to select between multiple image tracks}
	kQTVRObjectHotSpotTrackRefAtomID = 1;  { New with 2.1, it adds a track reference to select between multiple hotspot tracks}

{ Track reference types}
const
	kQTVRImageTrackRefType = FourCharCode('imgt');
	kQTVRHotSpotTrackRefType = FourCharCode('hott');

{ Old hot spot types}
const
	kQTVRHotSpotNavigableType = FourCharCode('navg');

{ Valid bits used in QTVRLinkHotSpotAtom}
const
	kQTVRValidPan = 1 shl 0;
	kQTVRValidTilt = 1 shl 1;
	kQTVRValidFOV = 1 shl 2;
	kQTVRValidViewCenter = 1 shl 3;


{ Values for flags field in QTVRPanoSampleAtom}
const
	kQTVRPanoFlagHorizontal = 1 shl 0;
	kQTVRPanoFlagLast = 1 shl 31;


{ Values for locationFlags field in QTVRNodeLocationAtom}
const
	kQTVRSameFile = 0;


{$endc} {not TARGET_CPU_64}

>>>>>>> origin/cpstrnew
{ Header for QTVR track's Sample Description record (vrWorld atom container is appended)}
type
	QTVRSampleDescription = record
		descSize: UInt32;               { total size of the QTVRSampleDescription}
		descType: UInt32;               { must be 'qtvr'}

		reserved1: UInt32;              { must be zero}
		reserved2: UInt16;              { must be zero}
		dataRefIndex: UInt16;           { must be zero}

		data: UInt32;                   { Will be extended to hold vrWorld QTAtomContainer}
	end;
	QTVRSampleDescriptionPtr = ^QTVRSampleDescription;
type
	QTVRSampleDescriptionHandle = ^QTVRSampleDescriptionPtr;

{$ifc not TARGET_CPU_64}

{
  =================================================================================================
   Definitions and structures used in the VRWorld QTAtomContainer
  -------------------------------------------------------------------------------------------------
}

type
	QTVRStringAtom = record
		stringUsage: UInt16;
		stringLength: UInt16;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		theString: packed array [0..3] of UInt8;           { field previously named "string" }
=======
		theString: packed array [0..3] of UInt8;			{  field previously named "string" }
>>>>>>> graemeg/cpstrnew
=======
		theString: packed array [0..3] of UInt8;			{  field previously named "string" }
>>>>>>> graemeg/cpstrnew
=======
		theString: packed array [0..3] of UInt8;			{  field previously named "string" }
>>>>>>> graemeg/cpstrnew
=======
		theString: packed array [0..3] of UInt8;			{  field previously named "string" }
>>>>>>> origin/cpstrnew
	end;
	QTVRStringAtomPtr = ^QTVRStringAtom;

type
	QTVRWorldHeaderAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		nameAtomID: QTAtomID;
		defaultNodeID: UInt32;
		vrWorldFlags: UInt32;

		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRWorldHeaderAtomPtr = ^QTVRWorldHeaderAtom;

{ Valid bits used in QTVRPanoImagingAtom}
const
	kQTVRValidCorrection = 1 shl 0;
	kQTVRValidQuality = 1 shl 1;
	kQTVRValidDirectDraw = 1 shl 2;
	kQTVRValidFirstExtraProperty = 1 shl 3;

type
	QTVRPanoImagingAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		imagingMode: UInt32;
		imagingValidFlags: UInt32;

		correction: UInt32;
		quality: UInt32;
		directDraw: UInt32;
		imagingProperties: array [0..5] of UInt32;   { for future properties}

		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRPanoImagingAtomPtr = ^QTVRPanoImagingAtom;
type
	QTVRNodeLocationAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		nodeType: OSType;
		locationFlags: UInt32;
		locationData: UInt32;

		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRNodeLocationAtomPtr = ^QTVRNodeLocationAtom;
{
  =================================================================================================
   Definitions and structures used in the Nodeinfo QTAtomContainer
  -------------------------------------------------------------------------------------------------
}

type
	QTVRNodeHeaderAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		nodeType: OSType;
		nodeID: QTAtomID;
		nameAtomID: QTAtomID;
		commentAtomID: QTAtomID;

		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRNodeHeaderAtomPtr = ^QTVRNodeHeaderAtom;
type
	QTVRAngleRangeAtom = record
		minimumAngle: Float32;
		maximumAngle: Float32;
	end;
	QTVRAngleRangeAtomPtr = ^QTVRAngleRangeAtom;
type
	QTVRHotSpotInfoAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		hotSpotType: OSType;
		nameAtomID: QTAtomID;
		commentAtomID: QTAtomID;

		cursorID: array [0..2] of SInt32;

                                              { canonical view for this hot spot}
		bestPan: Float32;
		bestTilt: Float32;
		bestFOV: Float32;
		bestViewCenter: QTVRFloatPoint;

                                              { Bounding box for this hot spot}
		hotSpotRect: Rect;

		flags: UInt32;
		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRHotSpotInfoAtomPtr = ^QTVRHotSpotInfoAtom;
type
	QTVRLinkHotSpotAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		toNodeID: UInt32;

		fromValidFlags: UInt32;
		fromPan: Float32;
		fromTilt: Float32;
		fromFOV: Float32;
		fromViewCenter: QTVRFloatPoint;

		toValidFlags: UInt32;
		toPan: Float32;
		toTilt: Float32;
		toFOV: Float32;
		toViewCenter: QTVRFloatPoint;

		distance: Float32;

		flags: UInt32;
		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRLinkHotSpotAtomPtr = ^QTVRLinkHotSpotAtom;
{
  =================================================================================================
   Definitions and structures used in Panorama and Object tracks
  -------------------------------------------------------------------------------------------------
}

type
	QTVRPanoSampleAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		imageRefTrackIndex: UInt32;     { track reference index of the full res image track}
		hotSpotRefTrackIndex: UInt32;   { track reference index of the full res hot spot track}

		minPan: Float32;
		maxPan: Float32;
		minTilt: Float32;
		maxTilt: Float32;
		minFieldOfView: Float32;
		maxFieldOfView: Float32;

		defaultPan: Float32;
		defaultTilt: Float32;
		defaultFieldOfView: Float32;

                                              { Info for highest res version of image track}
		imageSizeX: UInt32;             { pixel width of the panorama (e.g. 768)}
		imageSizeY: UInt32;             { pixel height of the panorama (e.g. 2496)}
		imageNumFramesX: UInt16;        { diced frames wide (e.g. 1)}
		imageNumFramesY: UInt16;        { diced frames high (e.g. 24)}

                                              { Info for highest res version of hotSpot track}
		hotSpotSizeX: UInt32;           { pixel width of the hot spot panorama (e.g. 768)}
		hotSpotSizeY: UInt32;           { pixel height of the hot spot panorama (e.g. 2496)}
		hotSpotNumFramesX: UInt16;      { diced frames wide (e.g. 1)}
		hotSpotNumFramesY: UInt16;      { diced frames high (e.g. 24)}

		flags: UInt32;
		panoType: OSType;
		reserved2: UInt32;
	end;
	QTVRPanoSampleAtomPtr = ^QTVRPanoSampleAtom;
{
   View atom for cubes (since same fields in QTVRPanoSampleAtom are set to special
   values for backwards compatibility and hence are ignored by the cubic engine)
}
type
	QTVRCubicViewAtom = record
		minPan: Float32;
		maxPan: Float32;
		minTilt: Float32;
		maxTilt: Float32;
		minFieldOfView: Float32;
		maxFieldOfView: Float32;

		defaultPan: Float32;
		defaultTilt: Float32;
		defaultFieldOfView: Float32;
	end;
	QTVRCubicViewAtomPtr = ^QTVRCubicViewAtom;
type
	QTVRCubicFaceData = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		orientation: array [0..4-1] of Float32;         { WXYZ quaternion of absolute orientation}
		center: array [0..2-1] of Float32;              { Center of image relative to center of projection (default = (0,0)) in normalized units}
=======
		orientation: array [0..3] of Float32;         {  WXYZ quaternion of absolute orientation }
		center: array [0..1] of Float32;              {  Center of image relative to center of projection (default = (0,0)) in normalized units }
>>>>>>> graemeg/cpstrnew
=======
		orientation: array [0..3] of Float32;         {  WXYZ quaternion of absolute orientation }
		center: array [0..1] of Float32;              {  Center of image relative to center of projection (default = (0,0)) in normalized units }
>>>>>>> graemeg/cpstrnew
=======
		orientation: array [0..3] of Float32;         {  WXYZ quaternion of absolute orientation }
		center: array [0..1] of Float32;              {  Center of image relative to center of projection (default = (0,0)) in normalized units }
>>>>>>> graemeg/cpstrnew
=======
		orientation: array [0..3] of Float32;         {  WXYZ quaternion of absolute orientation }
		center: array [0..1] of Float32;              {  Center of image relative to center of projection (default = (0,0)) in normalized units }
>>>>>>> origin/cpstrnew
		aspect: Float32;                 { aspect>1 => tall pixels; aspect <1 => squat pixels (default = 1)}
		skew: Float32;                   { skew x by y (default = 0)}
	end;
	QTVRCubicFaceDataPtr = ^QTVRCubicFaceData;
{ Special resolution values for the Image Track Reference Atoms. Use only one value per track reference.}
const
	kQTVRFullTrackRes = kQTVRFullRes;
	kQTVRHalfTrackRes = kQTVRHalfRes;
	kQTVRQuarterTrackRes = kQTVRQuarterRes;
	kQTVRPreviewTrackRes = $8000;
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Files,ConditionalMacros,Movies,QuickTimeVR;
=======
uses MacTypes,Movies,QuickTimeVR;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}


{ File Format Version numbers }
const
	kQTVRMajorVersion = 2;
const
	kQTVRMinorVersion = 0;

{ User data type for the Movie Controller type specifier}
const
	kQTControllerType = kQTVRControllerSubType; { Atom & ID of where our}
	kQTControllerID = 1;     { ...controller name is stored}

{ VRWorld atom types}
const
	kQTVRWorldHeaderAtomType = FourCharCode('vrsc');
	kQTVRImagingParentAtomType = FourCharCode('imgp');
	kQTVRPanoImagingAtomType = FourCharCode('impn');
	kQTVRObjectImagingAtomType = FourCharCode('imob');
	kQTVRNodeParentAtomType = FourCharCode('vrnp');
	kQTVRNodeIDAtomType = FourCharCode('vrni');
	kQTVRNodeLocationAtomType = FourCharCode('nloc');
	kQTVRCursorParentAtomType = FourCharCode('vrcp'); { New with 2.1}
	kQTVRCursorAtomType = FourCharCode('CURS'); { New with 2.1}
	kQTVRColorCursorAtomType = FourCharCode('crsr'); { New with 2.1}

{ NodeInfo atom types}
const
	kQTVRNodeHeaderAtomType = FourCharCode('ndhd');
	kQTVRHotSpotParentAtomType = FourCharCode('hspa');
	kQTVRHotSpotAtomType = FourCharCode('hots');
	kQTVRHotSpotInfoAtomType = FourCharCode('hsin');
	kQTVRLinkInfoAtomType = FourCharCode('link');

{ Miscellaneous atom types}
const
	kQTVRStringAtomType = FourCharCode('vrsg');
	kQTVRStringEncodingAtomType = FourCharCode('vrse'); { New with 2.1}
	kQTVRPanoSampleDataAtomType = FourCharCode('pdat');
	kQTVRObjectInfoAtomType = FourCharCode('obji');
	kQTVRImageTrackRefAtomType = FourCharCode('imtr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRHotSpotTrackRefAtomType = FourCharCode('hstr'); { Parent is kQTVRObjectInfoAtomType. Required if track ref is not 1 as required by 2.0 format.}
	kQTVRAngleRangeAtomType = FourCharCode('arng');
	kQTVRTrackRefArrayAtomType = FourCharCode('tref');
	kQTVRPanConstraintAtomType = FourCharCode('pcon');
	kQTVRTiltConstraintAtomType = FourCharCode('tcon');
	kQTVRFOVConstraintAtomType = FourCharCode('fcon');
	kQTVRCubicViewAtomType = FourCharCode('cuvw'); { New with 5.0}
	kQTVRCubicFaceDataAtomType = FourCharCode('cufa'); { New with 5.0}

const
	kQTVRObjectInfoAtomID = 1;
	kQTVRObjectImageTrackRefAtomID = 1;   { New with 2.1, it adds a track reference to select between multiple image tracks}
	kQTVRObjectHotSpotTrackRefAtomID = 1;  { New with 2.1, it adds a track reference to select between multiple hotspot tracks}

{ Track reference types}
const
	kQTVRImageTrackRefType = FourCharCode('imgt');
	kQTVRHotSpotTrackRefType = FourCharCode('hott');

{ Old hot spot types}
const
	kQTVRHotSpotNavigableType = FourCharCode('navg');

{ Valid bits used in QTVRLinkHotSpotAtom}
const
	kQTVRValidPan = 1 shl 0;
	kQTVRValidTilt = 1 shl 1;
	kQTVRValidFOV = 1 shl 2;
	kQTVRValidViewCenter = 1 shl 3;


{ Values for flags field in QTVRPanoSampleAtom}
const
	kQTVRPanoFlagHorizontal = 1 shl 0;
	kQTVRPanoFlagLast = 1 shl 31;


{ Values for locationFlags field in QTVRNodeLocationAtom}
const
	kQTVRSameFile = 0;


{$endc} {not TARGET_CPU_64}

{ Header for QTVR track's Sample Description record (vrWorld atom container is appended)}
type
	QTVRSampleDescription = record
		descSize: UInt32;               { total size of the QTVRSampleDescription}
		descType: UInt32;               { must be 'qtvr'}

		reserved1: UInt32;              { must be zero}
		reserved2: UInt16;              { must be zero}
		dataRefIndex: UInt16;           { must be zero}

		data: UInt32;                   { Will be extended to hold vrWorld QTAtomContainer}
	end;
	QTVRSampleDescriptionPtr = ^QTVRSampleDescription;
type
	QTVRSampleDescriptionHandle = ^QTVRSampleDescriptionPtr;

{$ifc not TARGET_CPU_64}

{
  =================================================================================================
   Definitions and structures used in the VRWorld QTAtomContainer
  -------------------------------------------------------------------------------------------------
}

type
	QTVRStringAtom = record
		stringUsage: UInt16;
		stringLength: UInt16;
		theString: packed array [0..3] of UInt8;			{  field previously named "string" }
	end;
	QTVRStringAtomPtr = ^QTVRStringAtom;

type
	QTVRWorldHeaderAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		nameAtomID: QTAtomID;
		defaultNodeID: UInt32;
		vrWorldFlags: UInt32;

		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRWorldHeaderAtomPtr = ^QTVRWorldHeaderAtom;

{ Valid bits used in QTVRPanoImagingAtom}
const
	kQTVRValidCorrection = 1 shl 0;
	kQTVRValidQuality = 1 shl 1;
	kQTVRValidDirectDraw = 1 shl 2;
	kQTVRValidFirstExtraProperty = 1 shl 3;

type
	QTVRPanoImagingAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		imagingMode: UInt32;
		imagingValidFlags: UInt32;

		correction: UInt32;
		quality: UInt32;
		directDraw: UInt32;
		imagingProperties: array [0..5] of UInt32;   { for future properties}

		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRPanoImagingAtomPtr = ^QTVRPanoImagingAtom;
type
	QTVRNodeLocationAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		nodeType: OSType;
		locationFlags: UInt32;
		locationData: UInt32;

		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRNodeLocationAtomPtr = ^QTVRNodeLocationAtom;
{
  =================================================================================================
   Definitions and structures used in the Nodeinfo QTAtomContainer
  -------------------------------------------------------------------------------------------------
}

type
	QTVRNodeHeaderAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		nodeType: OSType;
		nodeID: QTAtomID;
		nameAtomID: QTAtomID;
		commentAtomID: QTAtomID;

		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRNodeHeaderAtomPtr = ^QTVRNodeHeaderAtom;
type
	QTVRAngleRangeAtom = record
		minimumAngle: Float32;
		maximumAngle: Float32;
	end;
	QTVRAngleRangeAtomPtr = ^QTVRAngleRangeAtom;
type
	QTVRHotSpotInfoAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		hotSpotType: OSType;
		nameAtomID: QTAtomID;
		commentAtomID: QTAtomID;

		cursorID: array [0..2] of SInt32;

                                              { canonical view for this hot spot}
		bestPan: Float32;
		bestTilt: Float32;
		bestFOV: Float32;
		bestViewCenter: QTVRFloatPoint;

                                              { Bounding box for this hot spot}
		hotSpotRect: Rect;

		flags: UInt32;
		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRHotSpotInfoAtomPtr = ^QTVRHotSpotInfoAtom;
type
	QTVRLinkHotSpotAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		toNodeID: UInt32;

		fromValidFlags: UInt32;
		fromPan: Float32;
		fromTilt: Float32;
		fromFOV: Float32;
		fromViewCenter: QTVRFloatPoint;

		toValidFlags: UInt32;
		toPan: Float32;
		toTilt: Float32;
		toFOV: Float32;
		toViewCenter: QTVRFloatPoint;

		distance: Float32;

		flags: UInt32;
		reserved1: UInt32;
		reserved2: UInt32;
	end;
	QTVRLinkHotSpotAtomPtr = ^QTVRLinkHotSpotAtom;
{
  =================================================================================================
   Definitions and structures used in Panorama and Object tracks
  -------------------------------------------------------------------------------------------------
}

type
	QTVRPanoSampleAtom = record
		majorVersion: UInt16;
		minorVersion: UInt16;

		imageRefTrackIndex: UInt32;     { track reference index of the full res image track}
		hotSpotRefTrackIndex: UInt32;   { track reference index of the full res hot spot track}

		minPan: Float32;
		maxPan: Float32;
		minTilt: Float32;
		maxTilt: Float32;
		minFieldOfView: Float32;
		maxFieldOfView: Float32;

		defaultPan: Float32;
		defaultTilt: Float32;
		defaultFieldOfView: Float32;

                                              { Info for highest res version of image track}
		imageSizeX: UInt32;             { pixel width of the panorama (e.g. 768)}
		imageSizeY: UInt32;             { pixel height of the panorama (e.g. 2496)}
		imageNumFramesX: UInt16;        { diced frames wide (e.g. 1)}
		imageNumFramesY: UInt16;        { diced frames high (e.g. 24)}

                                              { Info for highest res version of hotSpot track}
		hotSpotSizeX: UInt32;           { pixel width of the hot spot panorama (e.g. 768)}
		hotSpotSizeY: UInt32;           { pixel height of the hot spot panorama (e.g. 2496)}
		hotSpotNumFramesX: UInt16;      { diced frames wide (e.g. 1)}
		hotSpotNumFramesY: UInt16;      { diced frames high (e.g. 24)}

		flags: UInt32;
		panoType: OSType;
		reserved2: UInt32;
	end;
	QTVRPanoSampleAtomPtr = ^QTVRPanoSampleAtom;
{
   View atom for cubes (since same fields in QTVRPanoSampleAtom are set to special
   values for backwards compatibility and hence are ignored by the cubic engine)
}
type
	QTVRCubicViewAtom = record
		minPan: Float32;
		maxPan: Float32;
		minTilt: Float32;
		maxTilt: Float32;
		minFieldOfView: Float32;
		maxFieldOfView: Float32;

		defaultPan: Float32;
		defaultTilt: Float32;
		defaultFieldOfView: Float32;
	end;
	QTVRCubicViewAtomPtr = ^QTVRCubicViewAtom;
type
	QTVRCubicFaceData = record
		orientation: array [0..3] of Float32;         {  WXYZ quaternion of absolute orientation }
		center: array [0..1] of Float32;              {  Center of image relative to center of projection (default = (0,0)) in normalized units }
		aspect: Float32;                 { aspect>1 => tall pixels; aspect <1 => squat pixels (default = 1)}
		skew: Float32;                   { skew x by y (default = 0)}
	end;
	QTVRCubicFaceDataPtr = ^QTVRCubicFaceData;
{ Special resolution values for the Image Track Reference Atoms. Use only one value per track reference.}
const
<<<<<<< HEAD
	kQTVRFullTrackRes			= $00000001;
	kQTVRHalfTrackRes			= $00000002;
	kQTVRQuarterTrackRes		= $00000004;
	kQTVRPreviewTrackRes		= $8000;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTVRFullTrackRes = kQTVRFullRes;
	kQTVRHalfTrackRes = kQTVRHalfRes;
	kQTVRQuarterTrackRes = kQTVRQuarterRes;
	kQTVRPreviewTrackRes = $8000;
>>>>>>> origin/fixes_2.4

type
	QTVRTrackRefEntryPtr = ^QTVRTrackRefEntry;
	QTVRTrackRefEntry = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		trackRefType: UInt32;
		trackResolution: UInt16;
		trackRefIndex: UInt32;
	end;
{
  =================================================================================================
   Object File format 2.0
  -------------------------------------------------------------------------------------------------
}
const
	kQTVRObjectAnimateViewFramesOn = 1 shl 0;
	kQTVRObjectPalindromeViewFramesOn = 1 shl 1;
	kQTVRObjectStartFirstViewFrameOn = 1 shl 2;
	kQTVRObjectAnimateViewsOn = 1 shl 3;
	kQTVRObjectPalindromeViewsOn = 1 shl 4;
	kQTVRObjectSyncViewToFrameRate = 1 shl 5;
	kQTVRObjectDontLoopViewFramesOn = 1 shl 6;
	kQTVRObjectPlayEveryViewFrameOn = 1 shl 7;
	kQTVRObjectStreamingViewsOn = 1 shl 8;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

const
	kQTVRObjectWrapPanOn = 1 shl 0;
	kQTVRObjectWrapTiltOn = 1 shl 1;
	kQTVRObjectCanZoomOn = 1 shl 2;
	kQTVRObjectReverseHControlOn = 1 shl 3;
	kQTVRObjectReverseVControlOn = 1 shl 4;
	kQTVRObjectSwapHVControlOn = 1 shl 5;
	kQTVRObjectTranslationOn = 1 shl 6;

const
<<<<<<< HEAD
=======

const
	kQTVRObjectWrapPanOn = 1 shl 0;
	kQTVRObjectWrapTiltOn = 1 shl 1;
	kQTVRObjectCanZoomOn = 1 shl 2;
	kQTVRObjectReverseHControlOn = 1 shl 3;
	kQTVRObjectReverseVControlOn = 1 shl 4;
	kQTVRObjectSwapHVControlOn = 1 shl 5;
	kQTVRObjectTranslationOn = 1 shl 6;

const
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kGrabberScrollerUI = 1;    { "Object" }
	kOldJoyStickUI = 2;    {  "1.0 Object as Scene"     }
	kJoystickUI = 3;    { "Object In Scene"}
	kGrabberUI = 4;    { "Grabber only"}
	kAbsoluteUI = 5;     { "Absolute pointer"}


type
	QTVRObjectSampleAtom = record
		majorVersion: UInt16;           { kQTVRMajorVersion}
		minorVersion: UInt16;           { kQTVRMinorVersion}
		movieType: UInt16;              { ObjectUITypes}
		viewStateCount: UInt16;         { The number of view states 1 based}
		defaultViewState: UInt16;       { The default view state number. The number must be 1 to viewStateCount}
		mouseDownViewState: UInt16;     { The mouse down view state.   The number must be 1 to viewStateCount}
		viewDuration: UInt32;           { The duration of each view including all animation frames in a view}
		columns: UInt32;                { Number of columns in movie}
		rows: UInt32;                   { Number rows in movie}
		mouseMotionScale: Float32;       { 180.0 for kStandardObject or kQTVRObjectInScene, actual degrees for kOldNavigableMovieScene.}
		minPan: Float32;                 { Start   horizontal pan angle in degrees}
		maxPan: Float32;                 { End     horizontal pan angle in degrees}
		defaultPan: Float32;             { Initial horizontal pan angle in degrees (poster view)}
		minTilt: Float32;                { Start   vertical   pan angle in degrees}
		maxTilt: Float32;                { End     vertical   pan angle in degrees}
		defaultTilt: Float32;            { Initial vertical   pan angle in degrees (poster view)  }
		minFieldOfView: Float32;         { minimum field of view setting (appears as the maximum zoom effect) must be >= 1}
		fieldOfView: Float32;            { the field of view range must be >= 1}
		defaultFieldOfView: Float32;     { must be in minFieldOfView and maxFieldOfView range inclusive}
		defaultViewCenterH: Float32;
		defaultViewCenterV: Float32;

		viewRate: Float32;
		frameRate: Float32;
		animationSettings: UInt32;      { 32 reserved bit fields}
		controlSettings: UInt32;        { 32 reserved bit fields}
	end;
	QTVRObjectSampleAtomPtr = ^QTVRObjectSampleAtom;
{
  =================================================================================================
   QuickTime VR Authoring Components
  -------------------------------------------------------------------------------------------------
}

{
   ComponentDescription constants for QTVR Export components   
    (componentType = MovieExportType; componentSubType = MovieFileType)
}
const
	kQTVRFlattenerManufacturer = FourCharCode('vrwe'); { aka QTVRFlattenerType}
	kQTVRSplitterManufacturer = FourCharCode('vrsp');
	kQTVRObjExporterManufacturer = FourCharCode('vrob');

{ QuickTime VR Flattener atom types}
const
	kQTVRFlattenerSettingsParentAtomType = FourCharCode('VRWe'); { parent of settings atoms (other than compression)}
	kQTVRFlattenerPreviewResAtomType = FourCharCode('PRes'); { preview resolution Int16}
	kQTVRFlattenerImportSpecAtomType = FourCharCode('ISpe'); { import file spec FSSpec}
	kQTVRFlattenerCreatePreviewAtomType = FourCharCode('Prev'); { Boolean}
	kQTVRFlattenerImportPreviewAtomType = FourCharCode('IPre'); { Boolean}
	kQTVRFlattenerBlurPreviewAtomType = FourCharCode('Blur'); { Boolean}

{ QuickTime VR Splitter atom types}
const
	kQTVRSplitterSettingsParentAtomType = FourCharCode('VRSp'); { parent of settings atoms (other than compression)}
	kQTVRSplitterGenerateHTMLAtomType = FourCharCode('Ghtm'); { Boolean}
	kQTVRSplitterOverwriteFilesAtomType = FourCharCode('Owfi'); { Boolean}
	kQTVRSplitterUseFlattenerAtomType = FourCharCode('Usef'); { Boolean}
	kQTVRSplitterShowControllerAtomType = FourCharCode('Shco'); { Boolean}
	kQTVRSplitterTargetMyselfAtomType = FourCharCode('Tgtm'); { Boolean}

{ QuickTime VR Object Exporter atom types}
const
	kQTVRObjExporterSettingsBlockSize = FourCharCode('bsiz'); { block size for compression}
	kQTVRObjExporterSettingsTargetSize = FourCharCode('tsiz'); { target file size}


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
		trackRefType:			UInt32;
		trackResolution:		UInt16;
		trackRefIndex:			UInt32;
=======
		trackRefType: UInt32;
		trackResolution: UInt16;
		trackRefIndex: UInt32;
>>>>>>> origin/fixes_2.4
	end;
{
  =================================================================================================
   Object File format 2.0
  -------------------------------------------------------------------------------------------------
}
const
	kQTVRObjectAnimateViewFramesOn = 1 shl 0;
	kQTVRObjectPalindromeViewFramesOn = 1 shl 1;
	kQTVRObjectStartFirstViewFrameOn = 1 shl 2;
	kQTVRObjectAnimateViewsOn = 1 shl 3;
	kQTVRObjectPalindromeViewsOn = 1 shl 4;
	kQTVRObjectSyncViewToFrameRate = 1 shl 5;
	kQTVRObjectDontLoopViewFramesOn = 1 shl 6;
	kQTVRObjectPlayEveryViewFrameOn = 1 shl 7;
	kQTVRObjectStreamingViewsOn = 1 shl 8;

const
	kQTVRObjectWrapPanOn = 1 shl 0;
	kQTVRObjectWrapTiltOn = 1 shl 1;
	kQTVRObjectCanZoomOn = 1 shl 2;
	kQTVRObjectReverseHControlOn = 1 shl 3;
	kQTVRObjectReverseVControlOn = 1 shl 4;
	kQTVRObjectSwapHVControlOn = 1 shl 5;
	kQTVRObjectTranslationOn = 1 shl 6;

const
	kGrabberScrollerUI = 1;    { "Object" }
	kOldJoyStickUI = 2;    {  "1.0 Object as Scene"     }
	kJoystickUI = 3;    { "Object In Scene"}
	kGrabberUI = 4;    { "Grabber only"}
	kAbsoluteUI = 5;     { "Absolute pointer"}


type
	QTVRObjectSampleAtom = record
		majorVersion: UInt16;           { kQTVRMajorVersion}
		minorVersion: UInt16;           { kQTVRMinorVersion}
		movieType: UInt16;              { ObjectUITypes}
		viewStateCount: UInt16;         { The number of view states 1 based}
		defaultViewState: UInt16;       { The default view state number. The number must be 1 to viewStateCount}
		mouseDownViewState: UInt16;     { The mouse down view state.   The number must be 1 to viewStateCount}
		viewDuration: UInt32;           { The duration of each view including all animation frames in a view}
		columns: UInt32;                { Number of columns in movie}
		rows: UInt32;                   { Number rows in movie}
		mouseMotionScale: Float32;       { 180.0 for kStandardObject or kQTVRObjectInScene, actual degrees for kOldNavigableMovieScene.}
		minPan: Float32;                 { Start   horizontal pan angle in degrees}
		maxPan: Float32;                 { End     horizontal pan angle in degrees}
		defaultPan: Float32;             { Initial horizontal pan angle in degrees (poster view)}
		minTilt: Float32;                { Start   vertical   pan angle in degrees}
		maxTilt: Float32;                { End     vertical   pan angle in degrees}
		defaultTilt: Float32;            { Initial vertical   pan angle in degrees (poster view)  }
		minFieldOfView: Float32;         { minimum field of view setting (appears as the maximum zoom effect) must be >= 1}
		fieldOfView: Float32;            { the field of view range must be >= 1}
		defaultFieldOfView: Float32;     { must be in minFieldOfView and maxFieldOfView range inclusive}
		defaultViewCenterH: Float32;
		defaultViewCenterV: Float32;

		viewRate: Float32;
		frameRate: Float32;
		animationSettings: UInt32;      { 32 reserved bit fields}
		controlSettings: UInt32;        { 32 reserved bit fields}
	end;
	QTVRObjectSampleAtomPtr = ^QTVRObjectSampleAtom;
{
  =================================================================================================
   QuickTime VR Authoring Components
  -------------------------------------------------------------------------------------------------
}

{
   ComponentDescription constants for QTVR Export components   
    (componentType = MovieExportType; componentSubType = MovieFileType)
}
const
	kQTVRFlattenerManufacturer = FourCharCode('vrwe'); { aka QTVRFlattenerType}
	kQTVRSplitterManufacturer = FourCharCode('vrsp');
	kQTVRObjExporterManufacturer = FourCharCode('vrob');

{ QuickTime VR Flattener atom types}
const
	kQTVRFlattenerSettingsParentAtomType = FourCharCode('VRWe'); { parent of settings atoms (other than compression)}
	kQTVRFlattenerPreviewResAtomType = FourCharCode('PRes'); { preview resolution Int16}
	kQTVRFlattenerImportSpecAtomType = FourCharCode('ISpe'); { import file spec FSSpec}
	kQTVRFlattenerCreatePreviewAtomType = FourCharCode('Prev'); { Boolean}
	kQTVRFlattenerImportPreviewAtomType = FourCharCode('IPre'); { Boolean}
	kQTVRFlattenerBlurPreviewAtomType = FourCharCode('Blur'); { Boolean}

{ QuickTime VR Splitter atom types}
const
	kQTVRSplitterSettingsParentAtomType = FourCharCode('VRSp'); { parent of settings atoms (other than compression)}
	kQTVRSplitterGenerateHTMLAtomType = FourCharCode('Ghtm'); { Boolean}
	kQTVRSplitterOverwriteFilesAtomType = FourCharCode('Owfi'); { Boolean}
	kQTVRSplitterUseFlattenerAtomType = FourCharCode('Usef'); { Boolean}
	kQTVRSplitterShowControllerAtomType = FourCharCode('Shco'); { Boolean}
	kQTVRSplitterTargetMyselfAtomType = FourCharCode('Tgtm'); { Boolean}

{ QuickTime VR Object Exporter atom types}
const
	kQTVRObjExporterSettingsBlockSize = FourCharCode('bsiz'); { block size for compression}
	kQTVRObjExporterSettingsTargetSize = FourCharCode('tsiz'); { target file size}


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
