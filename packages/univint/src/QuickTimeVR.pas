{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/QuickTimeVR.h
 
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
     File:       QuickTimeVR.p
=======
     File:       QuickTime/QuickTimeVR.h
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

unit QuickTimeVR;
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,MacWindows,QuickdrawTypes,Movies;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew

=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	QTVRInstance = ^OpaqueQTVRInstance; { an opaque type }
	OpaqueQTVRInstance = record end;
	QTVRInstancePtr = ^QTVRInstance;  { when a var xx:QTVRInstance parameter can be nil, it is changed to xx: QTVRInstancePtr }

{ Released API Version numbers }
const
  kQTVRAPIMajorVersion05 = $05;
  kQTVRAPIMajorVersion02 = $02;
  kQTVRAPIMinorVersion00 = $00;
  kQTVRAPIMinorVersion01 = $01;
  kQTVRAPIMinorVersion10 = $10;
  kQTVRAPIMinorVersion20 = $20;

{ Version numbers for the API described in this header }
const
=======
=======
>>>>>>> graemeg/cpstrnew
	QTVRInstance = ^SInt32; { an opaque type }
	QTVRInstancePtr = ^QTVRInstance;  { when a var xx:QTVRInstance parameter can be nil, it is changed to xx: QTVRInstancePtr }

{ Released API Version numbers }
<<<<<<< HEAD
=======
	QTVRInstance = ^SInt32; { an opaque type }
	QTVRInstancePtr = ^QTVRInstance;  { when a var xx:QTVRInstance parameter can be nil, it is changed to xx: QTVRInstancePtr }

{ Released API Version numbers }
>>>>>>> graemeg/cpstrnew
const
  kQTVRAPIMajorVersion05 = $05;
  kQTVRAPIMajorVersion02 = $02;
  kQTVRAPIMinorVersion00 = $00;
  kQTVRAPIMinorVersion01 = $01;
  kQTVRAPIMinorVersion10 = $10;
  kQTVRAPIMinorVersion20 = $20;

{ Version numbers for the API described in this header }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
const
=======
	QTVRInstance = ^SInt32; { an opaque type }
	QTVRInstancePtr = ^QTVRInstance;  { when a var xx:QTVRInstance parameter can be nil, it is changed to xx: QTVRInstancePtr }

{ Released API Version numbers }
const
>>>>>>> origin/cpstrnew
  kQTVRAPIMajorVersion05 = $05;
  kQTVRAPIMajorVersion02 = $02;
  kQTVRAPIMinorVersion00 = $00;
  kQTVRAPIMinorVersion01 = $01;
  kQTVRAPIMinorVersion10 = $10;
  kQTVRAPIMinorVersion20 = $20;

{ Version numbers for the API described in this header }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  kQTVRAPIMajorVersion = kQTVRAPIMajorVersion05;
  kQTVRAPIMinorVersion = kQTVRAPIMinorVersion00;


const
	kQTVRControllerSubType = FourCharCode('ctyp');
	kQTVRQTVRType = FourCharCode('qtvr');
	kQTVRPanoramaType = FourCharCode('pano');
	kQTVRObjectType = FourCharCode('obje');
	kQTVROldPanoType = FourCharCode('STpn'); { Used in QTVR 1.0 release}
	kQTVROldObjectType = FourCharCode('stna'); { Used in QTVR 1.0 release}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew

  kQTVRUnknownType = FourCharCode('????'); { Unknown node type }

{ QTVR hot spot types}
const
	kQTVRHotSpotLinkType = FourCharCode('link');
	kQTVRHotSpotURLType = FourCharCode('url ');
	kQTVRHotSpotUndefinedType = FourCharCode('undf');

{ Special Values for nodeID in QTVRGoToNodeID}
const
	kQTVRCurrentNode = 0;
	kQTVRPreviousNode = $80000000;
	kQTVRDefaultNode = $80000001;

<<<<<<< HEAD
=======

  kQTVRUnknownType = FourCharCode('????'); { Unknown node type }

{ QTVR hot spot types}
const
	kQTVRHotSpotLinkType = FourCharCode('link');
	kQTVRHotSpotURLType = FourCharCode('url ');
	kQTVRHotSpotUndefinedType = FourCharCode('undf');

{ Special Values for nodeID in QTVRGoToNodeID}
const
	kQTVRCurrentNode = 0;
	kQTVRPreviousNode = $80000000;
	kQTVRDefaultNode = $80000001;

>>>>>>> graemeg/cpstrnew
{ Panorama correction modes used for the kQTVRImagingCorrection imaging property}
const
	kQTVRNoCorrection = 0;
	kQTVRPartialCorrection = 1;
	kQTVRFullCorrection = 2;

=======
{ Panorama correction modes used for the kQTVRImagingCorrection imaging property}
const
	kQTVRNoCorrection = 0;
	kQTVRPartialCorrection = 1;
	kQTVRFullCorrection = 2;

>>>>>>> graemeg/cpstrnew
=======

  kQTVRUnknownType = FourCharCode('????'); { Unknown node type }

{ QTVR hot spot types}
const
	kQTVRHotSpotLinkType = FourCharCode('link');
	kQTVRHotSpotURLType = FourCharCode('url ');
	kQTVRHotSpotUndefinedType = FourCharCode('undf');

{ Special Values for nodeID in QTVRGoToNodeID}
const
	kQTVRCurrentNode = 0;
	kQTVRPreviousNode = $80000000;
	kQTVRDefaultNode = $80000001;

{ Panorama correction modes used for the kQTVRImagingCorrection imaging property}
const
	kQTVRNoCorrection = 0;
	kQTVRPartialCorrection = 1;
	kQTVRFullCorrection = 2;

>>>>>>> graemeg/cpstrnew
=======

  kQTVRUnknownType = FourCharCode('????'); { Unknown node type }

{ QTVR hot spot types}
const
	kQTVRHotSpotLinkType = FourCharCode('link');
	kQTVRHotSpotURLType = FourCharCode('url ');
	kQTVRHotSpotUndefinedType = FourCharCode('undf');

{ Special Values for nodeID in QTVRGoToNodeID}
const
	kQTVRCurrentNode = 0;
	kQTVRPreviousNode = $80000000;
	kQTVRDefaultNode = $80000001;

{ Panorama correction modes used for the kQTVRImagingCorrection imaging property}
const
	kQTVRNoCorrection = 0;
	kQTVRPartialCorrection = 1;
	kQTVRFullCorrection = 2;

>>>>>>> origin/cpstrnew
{ Imaging Modes used by QTVRSetImagingProperty, QTVRGetImagingProperty, QTVRUpdate, QTVRBeginUpdate}
type
	QTVRImagingMode = UInt32;
const
	kQTVRStatic = 1;
	kQTVRMotion = 2;
	kQTVRCurrentMode = 0;    { Special Value for QTVRUpdate}
	kQTVRAllModes = 100;   { Special value for QTVRSetProperty}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew

{ Imaging Properties used by QTVRSetImagingProperty, QTVRGetImagingProperty}
const
	kQTVRImagingCorrection = 1;
	kQTVRImagingQuality = 2;
	kQTVRImagingDirectDraw = 3;
	kQTVRImagingCurrentMode = 100;   { Get Only}

{ OR the above with kImagingDefaultValue to get/set the default value}
const
	kImagingDefaultValue = $80000000;

{ Transition Types used by QTVRSetTransitionProperty, QTVREnableTransition}
const
	kQTVRTransitionSwing = 1;
<<<<<<< HEAD

=======

{ Imaging Properties used by QTVRSetImagingProperty, QTVRGetImagingProperty}
const
	kQTVRImagingCorrection = 1;
	kQTVRImagingQuality = 2;
	kQTVRImagingDirectDraw = 3;
	kQTVRImagingCurrentMode = 100;   { Get Only}

{ OR the above with kImagingDefaultValue to get/set the default value}
const
	kImagingDefaultValue = $80000000;

{ Transition Types used by QTVRSetTransitionProperty, QTVREnableTransition}
const
	kQTVRTransitionSwing = 1;

>>>>>>> graemeg/cpstrnew
{ Transition Properties QTVRSetTransitionProperty}
const
	kQTVRTransitionSpeed = 1;
	kQTVRTransitionDirection = 2;

{ Constraint values used to construct value returned by GetConstraintStatus}
const
	kQTVRUnconstrained = 0;
	kQTVRCantPanLeft = 1 shl 0;
	kQTVRCantPanRight = 1 shl 1;
	kQTVRCantPanUp = 1 shl 2;
	kQTVRCantPanDown = 1 shl 3;
	kQTVRCantZoomIn = 1 shl 4;
	kQTVRCantZoomOut = 1 shl 5;
	kQTVRCantTranslateLeft = 1 shl 6;
	kQTVRCantTranslateRight = 1 shl 7;
	kQTVRCantTranslateUp = 1 shl 8;
	kQTVRCantTranslateDown = 1 shl 9;

{ Object-only mouse mode values used to construct value returned by QTVRGetCurrentMouseMode}
const
	kQTVRPanning = 1 shl 0; { standard objects, "object only" controllers}
	kQTVRTranslating = 1 shl 1; { all objects}
	kQTVRZooming = 1 shl 2; { all objects}
	kQTVRScrolling = 1 shl 3; { standard object arrow scrollers and joystick object}
	kQTVRSelecting = 1 shl 4; { object absolute controller}
<<<<<<< HEAD

=======

{ Imaging Properties used by QTVRSetImagingProperty, QTVRGetImagingProperty}
const
	kQTVRImagingCorrection = 1;
	kQTVRImagingQuality = 2;
	kQTVRImagingDirectDraw = 3;
	kQTVRImagingCurrentMode = 100;   { Get Only}

{ OR the above with kImagingDefaultValue to get/set the default value}
const
	kImagingDefaultValue = $80000000;

{ Transition Types used by QTVRSetTransitionProperty, QTVREnableTransition}
const
	kQTVRTransitionSwing = 1;

{ Transition Properties QTVRSetTransitionProperty}
const
	kQTVRTransitionSpeed = 1;
	kQTVRTransitionDirection = 2;

{ Constraint values used to construct value returned by GetConstraintStatus}
const
	kQTVRUnconstrained = 0;
	kQTVRCantPanLeft = 1 shl 0;
	kQTVRCantPanRight = 1 shl 1;
	kQTVRCantPanUp = 1 shl 2;
	kQTVRCantPanDown = 1 shl 3;
	kQTVRCantZoomIn = 1 shl 4;
	kQTVRCantZoomOut = 1 shl 5;
	kQTVRCantTranslateLeft = 1 shl 6;
	kQTVRCantTranslateRight = 1 shl 7;
	kQTVRCantTranslateUp = 1 shl 8;
	kQTVRCantTranslateDown = 1 shl 9;

{ Object-only mouse mode values used to construct value returned by QTVRGetCurrentMouseMode}
const
	kQTVRPanning = 1 shl 0; { standard objects, "object only" controllers}
	kQTVRTranslating = 1 shl 1; { all objects}
	kQTVRZooming = 1 shl 2; { all objects}
	kQTVRScrolling = 1 shl 3; { standard object arrow scrollers and joystick object}
	kQTVRSelecting = 1 shl 4; { object absolute controller}

>>>>>>> graemeg/cpstrnew
{ Properties for use with QTVRSetInteractionProperty/GetInteractionProperty}
const
	kQTVRInteractionMouseClickHysteresis = 1; { pixels within which the mouse is considered not to have moved (UInt16)}
	kQTVRInteractionMouseClickTimeout = 2; { ticks after which a mouse click times out and turns into panning (UInt32)}
	kQTVRInteractionPanTiltSpeed = 3;    { control the relative pan/tilt speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionZoomSpeed = 4;    { control the relative zooming speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionTranslateOnMouseDown = 101; { Holding MouseDown with this setting translates zoomed object movies (Boolean)}
	kQTVRInteractionMouseMotionScale = 102; { The maximum angle of rotation caused by dragging across the display window. (* float)}
	kQTVRInteractionNudgeMode = 103;   { A QTVRNudgeMode: rotate, translate, or the same as the current mouse mode. Requires QTVR 2.1}

{ OR the above with kQTVRInteractionDefaultValue to get/set the default value}
const
	kQTVRInteractionDefaultValue = $80000000;
=======
>>>>>>> origin/cpstrnew

{ Imaging Properties used by QTVRSetImagingProperty, QTVRGetImagingProperty}
const
	kQTVRImagingCorrection = 1;
	kQTVRImagingQuality = 2;
	kQTVRImagingDirectDraw = 3;
	kQTVRImagingCurrentMode = 100;   { Get Only}

<<<<<<< HEAD
{ Geometry constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieGeometry = 0;
	kQTVRVerticalCylinder = FourCharCode('vcyl');
	kQTVRHorizontalCylinder = FourCharCode('hcyl');
	kQTVRCube = FourCharCode('cube');

{ Resolution constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
=======

{ Properties for use with QTVRSetInteractionProperty/GetInteractionProperty}
const
	kQTVRInteractionMouseClickHysteresis = 1; { pixels within which the mouse is considered not to have moved (UInt16)}
	kQTVRInteractionMouseClickTimeout = 2; { ticks after which a mouse click times out and turns into panning (UInt32)}
	kQTVRInteractionPanTiltSpeed = 3;    { control the relative pan/tilt speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionZoomSpeed = 4;    { control the relative zooming speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionTranslateOnMouseDown = 101; { Holding MouseDown with this setting translates zoomed object movies (Boolean)}
	kQTVRInteractionMouseMotionScale = 102; { The maximum angle of rotation caused by dragging across the display window. (* float)}
	kQTVRInteractionNudgeMode = 103;   { A QTVRNudgeMode: rotate, translate, or the same as the current mouse mode. Requires QTVR 2.1}

{ OR the above with kQTVRInteractionDefaultValue to get/set the default value}
const
	kQTVRInteractionDefaultValue = $80000000;


{ Geometry constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieGeometry = 0;
	kQTVRVerticalCylinder = FourCharCode('vcyl');
	kQTVRHorizontalCylinder = FourCharCode('hcyl');
	kQTVRCube = FourCharCode('cube');

{ Resolution constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
>>>>>>> graemeg/cpstrnew
	kQTVRDefaultRes = 0;
	kQTVRFullRes = 1 shl 0;
	kQTVRHalfRes = 1 shl 1;
	kQTVRQuarterRes = 1 shl 2;

{ QTVR-specific pixelFormat constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieDepth = 0;

{ Cache Size Pref constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings}
const
	kQTVRMinimumCache = -1;
	kQTVRSuggestedCache = 0;
	kQTVRFullCache = 1;

{ Angular units used by QTVRSetAngularUnits}
type
	QTVRAngularUnits = UInt32;
<<<<<<< HEAD
const
	kQTVRDegrees = 0;
	kQTVRRadians = 1;

{ Values for enableFlag parameter in QTVREnableHotSpot}
const
	kQTVRHotSpotID = 0;
	kQTVRHotSpotType = 1;
	kQTVRAllHotSpots = 2;

{ Values for viewParameter for QTVRSet/GetViewParameter}
<<<<<<< HEAD
const
	kQTVRPanAngle = $0100; { default units; &float, &float}
	kQTVRTiltAngle = $0101; { default units; &float, &float}
	kQTVRFieldOfViewAngle = $0103; { default units; &float, &float}
	kQTVRViewCenter = $0104; { pixels (per object movies); &QTVRFloatPoint, &QTVRFloatPoint}
	kQTVRHotSpotsVisible = $0200; { Boolean, &Boolean}

{ Values for flagsIn for QTVRSet/GetViewParameter}
const
	kQTVRValueIsRelative = 1 shl 0; { Is the value absolute or relative to the current value?}
	kQTVRValueIsRate = 1 shl 1; { Is the value absolute or a rate of change to be applied?}
	kQTVRValueIsUserPrefRelative = 1 shl 2; { Is the value a percentage of the user rate pref?}

=======
const
	kQTVRPanAngle = $0100; { default units; &float, &float}
	kQTVRTiltAngle = $0101; { default units; &float, &float}
	kQTVRFieldOfViewAngle = $0103; { default units; &float, &float}
	kQTVRViewCenter = $0104; { pixels (per object movies); &QTVRFloatPoint, &QTVRFloatPoint}
	kQTVRHotSpotsVisible = $0200; { Boolean, &Boolean}

{ Values for flagsIn for QTVRSet/GetViewParameter}
const
	kQTVRValueIsRelative = 1 shl 0; { Is the value absolute or relative to the current value?}
	kQTVRValueIsRate = 1 shl 1; { Is the value absolute or a rate of change to be applied?}
	kQTVRValueIsUserPrefRelative = 1 shl 2; { Is the value a percentage of the user rate pref?}

>>>>>>> graemeg/cpstrnew
{ Values for kind parameter in QTVRGet/SetConstraints, QTVRGetViewingLimits}
const
	kQTVRPan = 0;
	kQTVRTilt = 1;
	kQTVRFieldOfView = 2;
	kQTVRViewCenterH = 4;    { WrapAndConstrain only}
	kQTVRViewCenterV = 5;     { WrapAndConstrain only}

=======

{ Transition Properties QTVRSetTransitionProperty}
const
	kQTVRTransitionSpeed = 1;
	kQTVRTransitionDirection = 2;

{ Constraint values used to construct value returned by GetConstraintStatus}
const
	kQTVRUnconstrained = 0;
	kQTVRCantPanLeft = 1 shl 0;
	kQTVRCantPanRight = 1 shl 1;
	kQTVRCantPanUp = 1 shl 2;
	kQTVRCantPanDown = 1 shl 3;
	kQTVRCantZoomIn = 1 shl 4;
	kQTVRCantZoomOut = 1 shl 5;
	kQTVRCantTranslateLeft = 1 shl 6;
	kQTVRCantTranslateRight = 1 shl 7;
	kQTVRCantTranslateUp = 1 shl 8;
	kQTVRCantTranslateDown = 1 shl 9;

{ Object-only mouse mode values used to construct value returned by QTVRGetCurrentMouseMode}
const
	kQTVRPanning = 1 shl 0; { standard objects, "object only" controllers}
	kQTVRTranslating = 1 shl 1; { all objects}
	kQTVRZooming = 1 shl 2; { all objects}
	kQTVRScrolling = 1 shl 3; { standard object arrow scrollers and joystick object}
	kQTVRSelecting = 1 shl 4; { object absolute controller}

{ Properties for use with QTVRSetInteractionProperty/GetInteractionProperty}
const
	kQTVRInteractionMouseClickHysteresis = 1; { pixels within which the mouse is considered not to have moved (UInt16)}
	kQTVRInteractionMouseClickTimeout = 2; { ticks after which a mouse click times out and turns into panning (UInt32)}
	kQTVRInteractionPanTiltSpeed = 3;    { control the relative pan/tilt speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionZoomSpeed = 4;    { control the relative zooming speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionTranslateOnMouseDown = 101; { Holding MouseDown with this setting translates zoomed object movies (Boolean)}
	kQTVRInteractionMouseMotionScale = 102; { The maximum angle of rotation caused by dragging across the display window. (* float)}
	kQTVRInteractionNudgeMode = 103;   { A QTVRNudgeMode: rotate, translate, or the same as the current mouse mode. Requires QTVR 2.1}

{ OR the above with kQTVRInteractionDefaultValue to get/set the default value}
const
	kQTVRInteractionDefaultValue = $80000000;


{ Geometry constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieGeometry = 0;
	kQTVRVerticalCylinder = FourCharCode('vcyl');
	kQTVRHorizontalCylinder = FourCharCode('hcyl');
	kQTVRCube = FourCharCode('cube');

{ Resolution constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRDefaultRes = 0;
	kQTVRFullRes = 1 shl 0;
	kQTVRHalfRes = 1 shl 1;
	kQTVRQuarterRes = 1 shl 2;

{ QTVR-specific pixelFormat constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieDepth = 0;

{ Cache Size Pref constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings}
const
	kQTVRMinimumCache = -1;
	kQTVRSuggestedCache = 0;
	kQTVRFullCache = 1;

{ Angular units used by QTVRSetAngularUnits}
type
	QTVRAngularUnits = UInt32;
const
	kQTVRDegrees = 0;
	kQTVRRadians = 1;

{ Values for enableFlag parameter in QTVREnableHotSpot}
const
	kQTVRHotSpotID = 0;
	kQTVRHotSpotType = 1;
	kQTVRAllHotSpots = 2;

=======
const
	kQTVRDegrees = 0;
	kQTVRRadians = 1;

{ Values for enableFlag parameter in QTVREnableHotSpot}
const
	kQTVRHotSpotID = 0;
	kQTVRHotSpotType = 1;
	kQTVRAllHotSpots = 2;

>>>>>>> graemeg/cpstrnew
{ Values for viewParameter for QTVRSet/GetViewParameter}
const
	kQTVRPanAngle = $0100; { default units; &float, &float}
	kQTVRTiltAngle = $0101; { default units; &float, &float}
	kQTVRFieldOfViewAngle = $0103; { default units; &float, &float}
	kQTVRViewCenter = $0104; { pixels (per object movies); &QTVRFloatPoint, &QTVRFloatPoint}
	kQTVRHotSpotsVisible = $0200; { Boolean, &Boolean}

{ Values for flagsIn for QTVRSet/GetViewParameter}
const
	kQTVRValueIsRelative = 1 shl 0; { Is the value absolute or relative to the current value?}
	kQTVRValueIsRate = 1 shl 1; { Is the value absolute or a rate of change to be applied?}
	kQTVRValueIsUserPrefRelative = 1 shl 2; { Is the value a percentage of the user rate pref?}
<<<<<<< HEAD

{ Values for kind parameter in QTVRGet/SetConstraints, QTVRGetViewingLimits}
const
	kQTVRPan = 0;
	kQTVRTilt = 1;
	kQTVRFieldOfView = 2;
	kQTVRViewCenterH = 4;    { WrapAndConstrain only}
	kQTVRViewCenterV = 5;     { WrapAndConstrain only}

>>>>>>> graemeg/cpstrnew
=======

{ Values for kind parameter in QTVRGet/SetConstraints, QTVRGetViewingLimits}
const
	kQTVRPan = 0;
	kQTVRTilt = 1;
	kQTVRFieldOfView = 2;
	kQTVRViewCenterH = 4;    { WrapAndConstrain only}
	kQTVRViewCenterV = 5;     { WrapAndConstrain only}

>>>>>>> graemeg/cpstrnew
=======
{ OR the above with kImagingDefaultValue to get/set the default value}
const
	kImagingDefaultValue = $80000000;

{ Transition Types used by QTVRSetTransitionProperty, QTVREnableTransition}
const
	kQTVRTransitionSwing = 1;

{ Transition Properties QTVRSetTransitionProperty}
const
	kQTVRTransitionSpeed = 1;
	kQTVRTransitionDirection = 2;

{ Constraint values used to construct value returned by GetConstraintStatus}
const
	kQTVRUnconstrained = 0;
	kQTVRCantPanLeft = 1 shl 0;
	kQTVRCantPanRight = 1 shl 1;
	kQTVRCantPanUp = 1 shl 2;
	kQTVRCantPanDown = 1 shl 3;
	kQTVRCantZoomIn = 1 shl 4;
	kQTVRCantZoomOut = 1 shl 5;
	kQTVRCantTranslateLeft = 1 shl 6;
	kQTVRCantTranslateRight = 1 shl 7;
	kQTVRCantTranslateUp = 1 shl 8;
	kQTVRCantTranslateDown = 1 shl 9;

{ Object-only mouse mode values used to construct value returned by QTVRGetCurrentMouseMode}
const
	kQTVRPanning = 1 shl 0; { standard objects, "object only" controllers}
	kQTVRTranslating = 1 shl 1; { all objects}
	kQTVRZooming = 1 shl 2; { all objects}
	kQTVRScrolling = 1 shl 3; { standard object arrow scrollers and joystick object}
	kQTVRSelecting = 1 shl 4; { object absolute controller}

{ Properties for use with QTVRSetInteractionProperty/GetInteractionProperty}
const
	kQTVRInteractionMouseClickHysteresis = 1; { pixels within which the mouse is considered not to have moved (UInt16)}
	kQTVRInteractionMouseClickTimeout = 2; { ticks after which a mouse click times out and turns into panning (UInt32)}
	kQTVRInteractionPanTiltSpeed = 3;    { control the relative pan/tilt speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionZoomSpeed = 4;    { control the relative zooming speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionTranslateOnMouseDown = 101; { Holding MouseDown with this setting translates zoomed object movies (Boolean)}
	kQTVRInteractionMouseMotionScale = 102; { The maximum angle of rotation caused by dragging across the display window. (* float)}
	kQTVRInteractionNudgeMode = 103;   { A QTVRNudgeMode: rotate, translate, or the same as the current mouse mode. Requires QTVR 2.1}

{ OR the above with kQTVRInteractionDefaultValue to get/set the default value}
const
	kQTVRInteractionDefaultValue = $80000000;


{ Geometry constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieGeometry = 0;
	kQTVRVerticalCylinder = FourCharCode('vcyl');
	kQTVRHorizontalCylinder = FourCharCode('hcyl');
	kQTVRCube = FourCharCode('cube');

{ Resolution constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRDefaultRes = 0;
	kQTVRFullRes = 1 shl 0;
	kQTVRHalfRes = 1 shl 1;
	kQTVRQuarterRes = 1 shl 2;

{ QTVR-specific pixelFormat constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieDepth = 0;

{ Cache Size Pref constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings}
const
	kQTVRMinimumCache = -1;
	kQTVRSuggestedCache = 0;
	kQTVRFullCache = 1;

{ Angular units used by QTVRSetAngularUnits}
type
	QTVRAngularUnits = UInt32;
const
	kQTVRDegrees = 0;
	kQTVRRadians = 1;

{ Values for enableFlag parameter in QTVREnableHotSpot}
const
	kQTVRHotSpotID = 0;
	kQTVRHotSpotType = 1;
	kQTVRAllHotSpots = 2;

{ Values for viewParameter for QTVRSet/GetViewParameter}
const
	kQTVRPanAngle = $0100; { default units; &float, &float}
	kQTVRTiltAngle = $0101; { default units; &float, &float}
	kQTVRFieldOfViewAngle = $0103; { default units; &float, &float}
	kQTVRViewCenter = $0104; { pixels (per object movies); &QTVRFloatPoint, &QTVRFloatPoint}
	kQTVRHotSpotsVisible = $0200; { Boolean, &Boolean}

{ Values for flagsIn for QTVRSet/GetViewParameter}
const
	kQTVRValueIsRelative = 1 shl 0; { Is the value absolute or relative to the current value?}
	kQTVRValueIsRate = 1 shl 1; { Is the value absolute or a rate of change to be applied?}
	kQTVRValueIsUserPrefRelative = 1 shl 2; { Is the value a percentage of the user rate pref?}

{ Values for kind parameter in QTVRGet/SetConstraints, QTVRGetViewingLimits}
const
	kQTVRPan = 0;
	kQTVRTilt = 1;
	kQTVRFieldOfView = 2;
	kQTVRViewCenterH = 4;    { WrapAndConstrain only}
	kQTVRViewCenterV = 5;     { WrapAndConstrain only}

>>>>>>> origin/cpstrnew
{ Values for setting parameter in QTVRSetAnimationSetting, QTVRGetAnimationSetting}
type
	QTVRObjectAnimationSetting = UInt32;
const
{ View Frame Animation Settings}
	kQTVRPalindromeViewFrames = 1;
	kQTVRStartFirstViewFrame = 2;
	kQTVRDontLoopViewFrames = 3;
	kQTVRPlayEveryViewFrame = 4;    { Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
                                        { View Animation Settings}
	kQTVRSyncViewToFrameRate = 16;
	kQTVRPalindromeViews = 17;
	kQTVRPlayStreamingViews = 18;    { Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}

type
	QTVRControlSetting = UInt32;
const
	kQTVRWrapPan = 1;
	kQTVRWrapTilt = 2;
	kQTVRCanZoom = 3;
	kQTVRReverseHControl = 4;
	kQTVRReverseVControl = 5;
	kQTVRSwapHVControl = 6;
	kQTVRTranslation = 7;

type
	QTVRViewStateType = UInt32;
const
	kQTVRDefault = 0;
	kQTVRCurrent = 2;
	kQTVRMouseDown = 3;

type
	QTVRNudgeControl = UInt32;
const
	kQTVRRight = 0;
	kQTVRUpRight = 45;
	kQTVRUp = 90;
	kQTVRUpLeft = 135;
	kQTVRLeft = 180;
	kQTVRDownLeft = 225;
	kQTVRDown = 270;
	kQTVRDownRight = 315;

type
	QTVRNudgeMode = UInt32;
const
	kQTVRNudgeRotate = 0;
	kQTVRNudgeTranslate = 1;
	kQTVRNudgeSameAsMouse = 2;


{ Flags to control elements of the QTVR control bar (set via mcActionSetFlags) }
const
	mcFlagQTVRSuppressBackBtn = 1 shl 16;
	mcFlagQTVRSuppressZoomBtns = 1 shl 17;
	mcFlagQTVRSuppressHotSpotBtn = 1 shl 18;
	mcFlagQTVRSuppressTranslateBtn = 1 shl 19;
	mcFlagQTVRSuppressHelpText = 1 shl 20;
	mcFlagQTVRSuppressHotSpotNames = 1 shl 21;
	mcFlagQTVRExplicitFlagSet = 1 shl 31; { bits 0->30 should be interpreted as "explicit on" for the corresponding suppression bits}

{ Cursor types used in type field of QTVRCursorRecord}
const
	kQTVRUseDefaultCursor = 0;
	kQTVRStdCursorType = 1;
	kQTVRColorCursorType = 2;

{ Values for flags parameter in QTVRMouseOverHotSpot callback}
const
	kQTVRHotSpotEnter = 0;
	kQTVRHotSpotWithin = 1;
	kQTVRHotSpotLeave = 2;

{ Values for flags parameter in QTVRSetPrescreenImagingCompleteProc}
const
	kQTVRPreScreenEveryIdle = 1 shl 0; { Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}

{ Values for flags field of areasOfInterest in QTVRSetBackBufferImagingProc}
const
	kQTVRBackBufferEveryUpdate = 1 shl 0;
	kQTVRBackBufferEveryIdle = 1 shl 1;
	kQTVRBackBufferAlwaysRefresh = 1 shl 2;
	kQTVRBackBufferHorizontal = 1 shl 3; { Requires that backbuffer proc be long-rowBytes aware (gestaltQDHasLongRowBytes)}

{ Values for flagsIn parameter in QTVRBackBufferImaging callback}
const
	kQTVRBackBufferRectVisible = 1 shl 0;
	kQTVRBackBufferWasRefreshed = 1 shl 1;

{ Values for flagsOut parameter in QTVRBackBufferImaging callback}
const
	kQTVRBackBufferFlagDidDraw = 1 shl 0;
	kQTVRBackBufferFlagLastFlag = 1 shl 31;

{ QTVRCursorRecord used in QTVRReplaceCursor}
type
	QTVRCursorRecordPtr = ^QTVRCursorRecord;
	QTVRCursorRecord = record
		theType: UInt16;                { field was previously named "type"}
		rsrcID: SInt16;
		handle: Handle_fix;
	end;
type
	QTVRFloatPointPtr = ^QTVRFloatPoint;
	QTVRFloatPoint = record
		x: Float32;
		y: Float32;
	end;
{ Struct used for areasOfInterest parameter in QTVRSetBackBufferImagingProc}
type
	QTVRAreaOfInterestPtr = ^QTVRAreaOfInterest;
	QTVRAreaOfInterest = record
		panAngle: Float32;
		tiltAngle: Float32;
		width: Float32;
		height: Float32;
		flags: UInt32;
	end;
{
  =================================================================================================
   Callback routines 
  -------------------------------------------------------------------------------------------------
}

type
	QTVRLeavingNodeProcPtr = function( qtvr: QTVRInstance; fromNodeID: UInt32; toNodeID: UInt32; var cancel: Boolean; refCon: SInt32 ): OSErr;
	QTVREnteringNodeProcPtr = function( qtvr: QTVRInstance; nodeID: UInt32; refCon: SInt32 ): OSErr;
	QTVRMouseOverHotSpotProcPtr = function( qtvr: QTVRInstance; hotSpotID: UInt32; flags: UInt32; refCon: SInt32 ): OSErr;
	QTVRImagingCompleteProcPtr = function( qtvr: QTVRInstance; refCon: SInt32 ): OSErr;
	QTVRBackBufferImagingProcPtr = function( qtvr: QTVRInstance; var drawRect: Rect; areaIndex: UInt16; flagsIn: UInt32; var flagsOut: UInt32; refCon: SInt32 ): OSErr;
	QTVRLeavingNodeUPP = QTVRLeavingNodeProcPtr;
	QTVREnteringNodeUPP = QTVREnteringNodeProcPtr;
	QTVRMouseOverHotSpotUPP = QTVRMouseOverHotSpotProcPtr;
	QTVRImagingCompleteUPP = QTVRImagingCompleteProcPtr;
	QTVRBackBufferImagingUPP = QTVRBackBufferImagingProcPtr;
{
 *  NewQTVRLeavingNodeUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTVRLeavingNodeUPP( userRoutine: QTVRLeavingNodeProcPtr ): QTVRLeavingNodeUPP; external name '_NewQTVRLeavingNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Quickdraw,Movies;
=======
uses MacTypes,MacWindows,QuickdrawTypes,Movies;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ QuickTime is not available to 64-bit clients }

{$ifc not TARGET_CPU_64}

type
	QTVRInstance = ^SInt32; { an opaque type }
	QTVRInstancePtr = ^QTVRInstance;  { when a var xx:QTVRInstance parameter can be nil, it is changed to xx: QTVRInstancePtr }

{ Released API Version numbers }
const
  kQTVRAPIMajorVersion05 = $05;
  kQTVRAPIMajorVersion02 = $02;
  kQTVRAPIMinorVersion00 = $00;
  kQTVRAPIMinorVersion01 = $01;
  kQTVRAPIMinorVersion10 = $10;
  kQTVRAPIMinorVersion20 = $20;

{ Version numbers for the API described in this header }
const
  kQTVRAPIMajorVersion = kQTVRAPIMajorVersion05;
  kQTVRAPIMinorVersion = kQTVRAPIMinorVersion00;


const
	kQTVRControllerSubType = FourCharCode('ctyp');
	kQTVRQTVRType = FourCharCode('qtvr');
	kQTVRPanoramaType = FourCharCode('pano');
	kQTVRObjectType = FourCharCode('obje');
	kQTVROldPanoType = FourCharCode('STpn'); { Used in QTVR 1.0 release}
	kQTVROldObjectType = FourCharCode('stna'); { Used in QTVR 1.0 release}

  kQTVRUnknownType = FourCharCode('????'); { Unknown node type }

{ QTVR hot spot types}
const
	kQTVRHotSpotLinkType = FourCharCode('link');
	kQTVRHotSpotURLType = FourCharCode('url ');
	kQTVRHotSpotUndefinedType = FourCharCode('undf');

{ Special Values for nodeID in QTVRGoToNodeID}
const
	kQTVRCurrentNode = 0;
	kQTVRPreviousNode = $80000000;
	kQTVRDefaultNode = $80000001;

{ Panorama correction modes used for the kQTVRImagingCorrection imaging property}
const
	kQTVRNoCorrection = 0;
	kQTVRPartialCorrection = 1;
	kQTVRFullCorrection = 2;

{ Imaging Modes used by QTVRSetImagingProperty, QTVRGetImagingProperty, QTVRUpdate, QTVRBeginUpdate}
type
	QTVRImagingMode = UInt32;
const
	kQTVRStatic = 1;
	kQTVRMotion = 2;
	kQTVRCurrentMode = 0;    { Special Value for QTVRUpdate}
	kQTVRAllModes = 100;   { Special value for QTVRSetProperty}

{ Imaging Properties used by QTVRSetImagingProperty, QTVRGetImagingProperty}
const
	kQTVRImagingCorrection = 1;
	kQTVRImagingQuality = 2;
	kQTVRImagingDirectDraw = 3;
	kQTVRImagingCurrentMode = 100;   { Get Only}

{ OR the above with kImagingDefaultValue to get/set the default value}
const
	kImagingDefaultValue = $80000000;

{ Transition Types used by QTVRSetTransitionProperty, QTVREnableTransition}
const
	kQTVRTransitionSwing = 1;

{ Transition Properties QTVRSetTransitionProperty}
const
	kQTVRTransitionSpeed = 1;
	kQTVRTransitionDirection = 2;

{ Constraint values used to construct value returned by GetConstraintStatus}
const
	kQTVRUnconstrained = 0;
	kQTVRCantPanLeft = 1 shl 0;
	kQTVRCantPanRight = 1 shl 1;
	kQTVRCantPanUp = 1 shl 2;
	kQTVRCantPanDown = 1 shl 3;
	kQTVRCantZoomIn = 1 shl 4;
	kQTVRCantZoomOut = 1 shl 5;
	kQTVRCantTranslateLeft = 1 shl 6;
	kQTVRCantTranslateRight = 1 shl 7;
	kQTVRCantTranslateUp = 1 shl 8;
	kQTVRCantTranslateDown = 1 shl 9;

{ Object-only mouse mode values used to construct value returned by QTVRGetCurrentMouseMode}
const
	kQTVRPanning = 1 shl 0; { standard objects, "object only" controllers}
	kQTVRTranslating = 1 shl 1; { all objects}
	kQTVRZooming = 1 shl 2; { all objects}
	kQTVRScrolling = 1 shl 3; { standard object arrow scrollers and joystick object}
	kQTVRSelecting = 1 shl 4; { object absolute controller}

{ Properties for use with QTVRSetInteractionProperty/GetInteractionProperty}
const
	kQTVRInteractionMouseClickHysteresis = 1; { pixels within which the mouse is considered not to have moved (UInt16)}
	kQTVRInteractionMouseClickTimeout = 2; { ticks after which a mouse click times out and turns into panning (UInt32)}
	kQTVRInteractionPanTiltSpeed = 3;    { control the relative pan/tilt speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionZoomSpeed = 4;    { control the relative zooming speed from 1 (slowest) to 10 (fastest). (UInt32) Default is 5;}
	kQTVRInteractionTranslateOnMouseDown = 101; { Holding MouseDown with this setting translates zoomed object movies (Boolean)}
	kQTVRInteractionMouseMotionScale = 102; { The maximum angle of rotation caused by dragging across the display window. (* float)}
	kQTVRInteractionNudgeMode = 103;   { A QTVRNudgeMode: rotate, translate, or the same as the current mouse mode. Requires QTVR 2.1}

{ OR the above with kQTVRInteractionDefaultValue to get/set the default value}
const
	kQTVRInteractionDefaultValue = $80000000;


{ Geometry constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieGeometry = 0;
	kQTVRVerticalCylinder = FourCharCode('vcyl');
	kQTVRHorizontalCylinder = FourCharCode('hcyl');
	kQTVRCube = FourCharCode('cube');

{ Resolution constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRDefaultRes = 0;
	kQTVRFullRes = 1 shl 0;
	kQTVRHalfRes = 1 shl 1;
	kQTVRQuarterRes = 1 shl 2;

{ QTVR-specific pixelFormat constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings, QTVRGetBackBufferMemInfo}
const
	kQTVRUseMovieDepth = 0;

{ Cache Size Pref constants used in QTVRSetBackBufferPrefs, QTVRGetBackBufferSettings}
const
	kQTVRMinimumCache = -1;
	kQTVRSuggestedCache = 0;
	kQTVRFullCache = 1;

{ Angular units used by QTVRSetAngularUnits}
type
	QTVRAngularUnits = UInt32;
const
	kQTVRDegrees = 0;
	kQTVRRadians = 1;

{ Values for enableFlag parameter in QTVREnableHotSpot}
const
	kQTVRHotSpotID = 0;
	kQTVRHotSpotType = 1;
	kQTVRAllHotSpots = 2;

{ Values for viewParameter for QTVRSet/GetViewParameter}
const
	kQTVRPanAngle = $0100; { default units; &float, &float}
	kQTVRTiltAngle = $0101; { default units; &float, &float}
	kQTVRFieldOfViewAngle = $0103; { default units; &float, &float}
	kQTVRViewCenter = $0104; { pixels (per object movies); &QTVRFloatPoint, &QTVRFloatPoint}
	kQTVRHotSpotsVisible = $0200; { Boolean, &Boolean}

{ Values for flagsIn for QTVRSet/GetViewParameter}
const
	kQTVRValueIsRelative = 1 shl 0; { Is the value absolute or relative to the current value?}
	kQTVRValueIsRate = 1 shl 1; { Is the value absolute or a rate of change to be applied?}
	kQTVRValueIsUserPrefRelative = 1 shl 2; { Is the value a percentage of the user rate pref?}

{ Values for kind parameter in QTVRGet/SetConstraints, QTVRGetViewingLimits}
const
	kQTVRPan = 0;
	kQTVRTilt = 1;
	kQTVRFieldOfView = 2;
	kQTVRViewCenterH = 4;    { WrapAndConstrain only}
	kQTVRViewCenterV = 5;     { WrapAndConstrain only}

{ Values for setting parameter in QTVRSetAnimationSetting, QTVRGetAnimationSetting}
type
	QTVRObjectAnimationSetting = UInt32;
const
{ View Frame Animation Settings}
	kQTVRPalindromeViewFrames = 1;
	kQTVRStartFirstViewFrame = 2;
	kQTVRDontLoopViewFrames = 3;
	kQTVRPlayEveryViewFrame = 4;    { Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
                                        { View Animation Settings}
	kQTVRSyncViewToFrameRate = 16;
	kQTVRPalindromeViews = 17;
	kQTVRPlayStreamingViews = 18;    { Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}

type
	QTVRControlSetting = UInt32;
const
	kQTVRWrapPan = 1;
	kQTVRWrapTilt = 2;
	kQTVRCanZoom = 3;
	kQTVRReverseHControl = 4;
	kQTVRReverseVControl = 5;
	kQTVRSwapHVControl = 6;
	kQTVRTranslation = 7;

type
	QTVRViewStateType = UInt32;
const
	kQTVRDefault = 0;
	kQTVRCurrent = 2;
	kQTVRMouseDown = 3;

type
	QTVRNudgeControl = UInt32;
const
	kQTVRRight = 0;
	kQTVRUpRight = 45;
	kQTVRUp = 90;
	kQTVRUpLeft = 135;
	kQTVRLeft = 180;
	kQTVRDownLeft = 225;
	kQTVRDown = 270;
	kQTVRDownRight = 315;

type
	QTVRNudgeMode = UInt32;
const
	kQTVRNudgeRotate = 0;
	kQTVRNudgeTranslate = 1;
	kQTVRNudgeSameAsMouse = 2;


{ Flags to control elements of the QTVR control bar (set via mcActionSetFlags) }
const
	mcFlagQTVRSuppressBackBtn = 1 shl 16;
	mcFlagQTVRSuppressZoomBtns = 1 shl 17;
	mcFlagQTVRSuppressHotSpotBtn = 1 shl 18;
	mcFlagQTVRSuppressTranslateBtn = 1 shl 19;
	mcFlagQTVRSuppressHelpText = 1 shl 20;
	mcFlagQTVRSuppressHotSpotNames = 1 shl 21;
	mcFlagQTVRExplicitFlagSet = 1 shl 31; { bits 0->30 should be interpreted as "explicit on" for the corresponding suppression bits}

{ Cursor types used in type field of QTVRCursorRecord}
const
	kQTVRUseDefaultCursor = 0;
	kQTVRStdCursorType = 1;
	kQTVRColorCursorType = 2;

{ Values for flags parameter in QTVRMouseOverHotSpot callback}
const
	kQTVRHotSpotEnter = 0;
	kQTVRHotSpotWithin = 1;
	kQTVRHotSpotLeave = 2;

{ Values for flags parameter in QTVRSetPrescreenImagingCompleteProc}
const
	kQTVRPreScreenEveryIdle = 1 shl 0; { Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}

{ Values for flags field of areasOfInterest in QTVRSetBackBufferImagingProc}
const
	kQTVRBackBufferEveryUpdate = 1 shl 0;
	kQTVRBackBufferEveryIdle = 1 shl 1;
	kQTVRBackBufferAlwaysRefresh = 1 shl 2;
	kQTVRBackBufferHorizontal = 1 shl 3; { Requires that backbuffer proc be long-rowBytes aware (gestaltQDHasLongRowBytes)}

{ Values for flagsIn parameter in QTVRBackBufferImaging callback}
const
	kQTVRBackBufferRectVisible = 1 shl 0;
	kQTVRBackBufferWasRefreshed = 1 shl 1;

{ Values for flagsOut parameter in QTVRBackBufferImaging callback}
const
	kQTVRBackBufferFlagDidDraw = 1 shl 0;
	kQTVRBackBufferFlagLastFlag = 1 shl 31;

{ QTVRCursorRecord used in QTVRReplaceCursor}
type
	QTVRCursorRecordPtr = ^QTVRCursorRecord;
	QTVRCursorRecord = record
		theType: UInt16;                { field was previously named "type"}
		rsrcID: SInt16;
		handle: Handle_fix;
	end;
type
	QTVRFloatPointPtr = ^QTVRFloatPoint;
	QTVRFloatPoint = record
		x: Float32;
		y: Float32;
	end;
{ Struct used for areasOfInterest parameter in QTVRSetBackBufferImagingProc}
type
	QTVRAreaOfInterestPtr = ^QTVRAreaOfInterest;
	QTVRAreaOfInterest = record
		panAngle: Float32;
		tiltAngle: Float32;
		width: Float32;
		height: Float32;
		flags: UInt32;
	end;
{
  =================================================================================================
   Callback routines 
  -------------------------------------------------------------------------------------------------
}

type
	QTVRLeavingNodeProcPtr = function( qtvr: QTVRInstance; fromNodeID: UInt32; toNodeID: UInt32; var cancel: Boolean; refCon: SInt32 ): OSErr;
	QTVREnteringNodeProcPtr = function( qtvr: QTVRInstance; nodeID: UInt32; refCon: SInt32 ): OSErr;
	QTVRMouseOverHotSpotProcPtr = function( qtvr: QTVRInstance; hotSpotID: UInt32; flags: UInt32; refCon: SInt32 ): OSErr;
	QTVRImagingCompleteProcPtr = function( qtvr: QTVRInstance; refCon: SInt32 ): OSErr;
	QTVRBackBufferImagingProcPtr = function( qtvr: QTVRInstance; var drawRect: Rect; areaIndex: UInt16; flagsIn: UInt32; var flagsOut: UInt32; refCon: SInt32 ): OSErr;
	QTVRLeavingNodeUPP = QTVRLeavingNodeProcPtr;
	QTVREnteringNodeUPP = QTVREnteringNodeProcPtr;
	QTVRMouseOverHotSpotUPP = QTVRMouseOverHotSpotProcPtr;
	QTVRImagingCompleteUPP = QTVRImagingCompleteProcPtr;
	QTVRBackBufferImagingUPP = QTVRBackBufferImagingProcPtr;
{
 *  NewQTVRLeavingNodeUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTVRLeavingNodeUPP( userRoutine: QTVRLeavingNodeProcPtr ): QTVRLeavingNodeUPP; external name '_NewQTVRLeavingNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	uppQTVRLeavingNodeProcInfo = $0000FFE0;
	uppQTVREnteringNodeProcInfo = $00000FE0;
	uppQTVRMouseOverHotSpotProcInfo = $00003FE0;
	uppQTVRImagingCompleteProcInfo = $000003E0;
	uppQTVRBackBufferImagingProcInfo = $0003FBE0;
	{
	 *  NewQTVRLeavingNodeUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewQTVRLeavingNodeUPP(userRoutine: QTVRLeavingNodeProcPtr): QTVRLeavingNodeUPP; external name '_NewQTVRLeavingNodeUPP'; { old name was NewQTVRLeavingNodeProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewQTVREnteringNodeUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTVREnteringNodeUPP( userRoutine: QTVREnteringNodeProcPtr ): QTVREnteringNodeUPP; external name '_NewQTVREnteringNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewQTVREnteringNodeUPP(userRoutine: QTVREnteringNodeProcPtr): QTVREnteringNodeUPP; external name '_NewQTVREnteringNodeUPP'; { old name was NewQTVREnteringNodeProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewQTVREnteringNodeUPP( userRoutine: QTVREnteringNodeProcPtr ): QTVREnteringNodeUPP; external name '_NewQTVREnteringNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  NewQTVRMouseOverHotSpotUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTVRMouseOverHotSpotUPP( userRoutine: QTVRMouseOverHotSpotProcPtr ): QTVRMouseOverHotSpotUPP; external name '_NewQTVRMouseOverHotSpotUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewQTVRMouseOverHotSpotUPP(userRoutine: QTVRMouseOverHotSpotProcPtr): QTVRMouseOverHotSpotUPP; external name '_NewQTVRMouseOverHotSpotUPP'; { old name was NewQTVRMouseOverHotSpotProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewQTVRMouseOverHotSpotUPP( userRoutine: QTVRMouseOverHotSpotProcPtr ): QTVRMouseOverHotSpotUPP; external name '_NewQTVRMouseOverHotSpotUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  NewQTVRImagingCompleteUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTVRImagingCompleteUPP( userRoutine: QTVRImagingCompleteProcPtr ): QTVRImagingCompleteUPP; external name '_NewQTVRImagingCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewQTVRImagingCompleteUPP(userRoutine: QTVRImagingCompleteProcPtr): QTVRImagingCompleteUPP; external name '_NewQTVRImagingCompleteUPP'; { old name was NewQTVRImagingCompleteProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewQTVRImagingCompleteUPP( userRoutine: QTVRImagingCompleteProcPtr ): QTVRImagingCompleteUPP; external name '_NewQTVRImagingCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  NewQTVRBackBufferImagingUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTVRBackBufferImagingUPP( userRoutine: QTVRBackBufferImagingProcPtr ): QTVRBackBufferImagingUPP; external name '_NewQTVRBackBufferImagingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewQTVRBackBufferImagingUPP(userRoutine: QTVRBackBufferImagingProcPtr): QTVRBackBufferImagingUPP; external name '_NewQTVRBackBufferImagingUPP'; { old name was NewQTVRBackBufferImagingProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewQTVRBackBufferImagingUPP( userRoutine: QTVRBackBufferImagingProcPtr ): QTVRBackBufferImagingUPP; external name '_NewQTVRBackBufferImagingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeQTVRLeavingNodeUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTVRLeavingNodeUPP( userUPP: QTVRLeavingNodeUPP ); external name '_DisposeQTVRLeavingNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTVRLeavingNodeUPP(userUPP: QTVRLeavingNodeUPP); external name '_DisposeQTVRLeavingNodeUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTVRLeavingNodeUPP( userUPP: QTVRLeavingNodeUPP ); external name '_DisposeQTVRLeavingNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeQTVREnteringNodeUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTVREnteringNodeUPP( userUPP: QTVREnteringNodeUPP ); external name '_DisposeQTVREnteringNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTVREnteringNodeUPP(userUPP: QTVREnteringNodeUPP); external name '_DisposeQTVREnteringNodeUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTVREnteringNodeUPP( userUPP: QTVREnteringNodeUPP ); external name '_DisposeQTVREnteringNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeQTVRMouseOverHotSpotUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTVRMouseOverHotSpotUPP( userUPP: QTVRMouseOverHotSpotUPP ); external name '_DisposeQTVRMouseOverHotSpotUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTVRMouseOverHotSpotUPP(userUPP: QTVRMouseOverHotSpotUPP); external name '_DisposeQTVRMouseOverHotSpotUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTVRMouseOverHotSpotUPP( userUPP: QTVRMouseOverHotSpotUPP ); external name '_DisposeQTVRMouseOverHotSpotUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeQTVRImagingCompleteUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTVRImagingCompleteUPP( userUPP: QTVRImagingCompleteUPP ); external name '_DisposeQTVRImagingCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTVRImagingCompleteUPP(userUPP: QTVRImagingCompleteUPP); external name '_DisposeQTVRImagingCompleteUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTVRImagingCompleteUPP( userUPP: QTVRImagingCompleteUPP ); external name '_DisposeQTVRImagingCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeQTVRBackBufferImagingUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTVRBackBufferImagingUPP( userUPP: QTVRBackBufferImagingUPP ); external name '_DisposeQTVRBackBufferImagingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTVRBackBufferImagingUPP(userUPP: QTVRBackBufferImagingUPP); external name '_DisposeQTVRBackBufferImagingUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTVRBackBufferImagingUPP( userUPP: QTVRBackBufferImagingUPP ); external name '_DisposeQTVRBackBufferImagingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTVRLeavingNodeUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTVRLeavingNodeUPP( qtvr: QTVRInstance; fromNodeID: UInt32; toNodeID: UInt32; var cancel: Boolean; refCon: SInt32; userUPP: QTVRLeavingNodeUPP ): OSErr; external name '_InvokeQTVRLeavingNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeQTVRLeavingNodeUPP(qtvr: QTVRInstance; fromNodeID: UInt32; toNodeID: UInt32; var cancel: boolean; refCon: SInt32; userRoutine: QTVRLeavingNodeUPP): OSErr; external name '_InvokeQTVRLeavingNodeUPP'; { old name was CallQTVRLeavingNodeProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeQTVRLeavingNodeUPP( qtvr: QTVRInstance; fromNodeID: UInt32; toNodeID: UInt32; var cancel: Boolean; refCon: SInt32; userUPP: QTVRLeavingNodeUPP ): OSErr; external name '_InvokeQTVRLeavingNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTVREnteringNodeUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTVREnteringNodeUPP( qtvr: QTVRInstance; nodeID: UInt32; refCon: SInt32; userUPP: QTVREnteringNodeUPP ): OSErr; external name '_InvokeQTVREnteringNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeQTVREnteringNodeUPP(qtvr: QTVRInstance; nodeID: UInt32; refCon: SInt32; userRoutine: QTVREnteringNodeUPP): OSErr; external name '_InvokeQTVREnteringNodeUPP'; { old name was CallQTVREnteringNodeProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeQTVREnteringNodeUPP( qtvr: QTVRInstance; nodeID: UInt32; refCon: SInt32; userUPP: QTVREnteringNodeUPP ): OSErr; external name '_InvokeQTVREnteringNodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTVRMouseOverHotSpotUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTVRMouseOverHotSpotUPP( qtvr: QTVRInstance; hotSpotID: UInt32; flags: UInt32; refCon: SInt32; userUPP: QTVRMouseOverHotSpotUPP ): OSErr; external name '_InvokeQTVRMouseOverHotSpotUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeQTVRMouseOverHotSpotUPP(qtvr: QTVRInstance; hotSpotID: UInt32; flags: UInt32; refCon: SInt32; userRoutine: QTVRMouseOverHotSpotUPP): OSErr; external name '_InvokeQTVRMouseOverHotSpotUPP'; { old name was CallQTVRMouseOverHotSpotProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeQTVRMouseOverHotSpotUPP( qtvr: QTVRInstance; hotSpotID: UInt32; flags: UInt32; refCon: SInt32; userUPP: QTVRMouseOverHotSpotUPP ): OSErr; external name '_InvokeQTVRMouseOverHotSpotUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTVRImagingCompleteUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTVRImagingCompleteUPP( qtvr: QTVRInstance; refCon: SInt32; userUPP: QTVRImagingCompleteUPP ): OSErr; external name '_InvokeQTVRImagingCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeQTVRImagingCompleteUPP(qtvr: QTVRInstance; refCon: SInt32; userRoutine: QTVRImagingCompleteUPP): OSErr; external name '_InvokeQTVRImagingCompleteUPP'; { old name was CallQTVRImagingCompleteProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeQTVRImagingCompleteUPP( qtvr: QTVRInstance; refCon: SInt32; userUPP: QTVRImagingCompleteUPP ): OSErr; external name '_InvokeQTVRImagingCompleteUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTVRBackBufferImagingUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQTVRBackBufferImagingUPP( qtvr: QTVRInstance; var drawRect: Rect; areaIndex: UInt16; flagsIn: UInt32; var flagsOut: UInt32; refCon: SInt32; userUPP: QTVRBackBufferImagingUPP ): OSErr; external name '_InvokeQTVRBackBufferImagingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeQTVRBackBufferImagingUPP(qtvr: QTVRInstance; var drawRect: Rect; areaIndex: UInt16; flagsIn: UInt32; var flagsOut: UInt32; refCon: SInt32; userRoutine: QTVRBackBufferImagingUPP): OSErr; external name '_InvokeQTVRBackBufferImagingUPP'; { old name was CallQTVRBackBufferImagingProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeQTVRBackBufferImagingUPP( qtvr: QTVRInstance; var drawRect: Rect; areaIndex: UInt16; flagsIn: UInt32; var flagsOut: UInt32; refCon: SInt32; userUPP: QTVRBackBufferImagingUPP ): OSErr; external name '_InvokeQTVRBackBufferImagingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
  =================================================================================================
    QTVR Intercept Struct, Callback, Routine Descriptors 
  -------------------------------------------------------------------------------------------------
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	QTVRProcSelector = UInt32;
const
	kQTVRSetPanAngleSelector = $2000;
	kQTVRSetTiltAngleSelector = $2001;
	kQTVRSetFieldOfViewSelector = $2002;
	kQTVRSetViewCenterSelector = $2003;
	kQTVRMouseEnterSelector = $2004;
	kQTVRMouseWithinSelector = $2005;
	kQTVRMouseLeaveSelector = $2006;
	kQTVRMouseDownSelector = $2007;
	kQTVRMouseStillDownSelector = $2008;
	kQTVRMouseUpSelector = $2009;
	kQTVRTriggerHotSpotSelector = $200A;
	kQTVRGetHotSpotTypeSelector = $200B; { Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
	kQTVRSetViewParameterSelector = $200C; { Requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00)}
	kQTVRGetViewParameterSelector = $200D; { Requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00)}
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
type
	QTVRProcSelector = UInt32;
const
<<<<<<< HEAD
	kQTVRSetPanAngleSelector	= $2000;
	kQTVRSetTiltAngleSelector	= $2001;
	kQTVRSetFieldOfViewSelector	= $2002;
	kQTVRSetViewCenterSelector	= $2003;
	kQTVRMouseEnterSelector		= $2004;
	kQTVRMouseWithinSelector	= $2005;
	kQTVRMouseLeaveSelector		= $2006;
	kQTVRMouseDownSelector		= $2007;
	kQTVRMouseStillDownSelector	= $2008;
	kQTVRMouseUpSelector		= $2009;
	kQTVRTriggerHotSpotSelector	= $200A;
	kQTVRGetHotSpotTypeSelector	= $200B;						{  Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10) }
	kQTVRSetViewParameterSelector = $200C;						{  Requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00) }
	kQTVRGetViewParameterSelector = $200D;						{  Requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00) }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTVRSetPanAngleSelector = $2000;
	kQTVRSetTiltAngleSelector = $2001;
	kQTVRSetFieldOfViewSelector = $2002;
	kQTVRSetViewCenterSelector = $2003;
	kQTVRMouseEnterSelector = $2004;
	kQTVRMouseWithinSelector = $2005;
	kQTVRMouseLeaveSelector = $2006;
	kQTVRMouseDownSelector = $2007;
	kQTVRMouseStillDownSelector = $2008;
	kQTVRMouseUpSelector = $2009;
	kQTVRTriggerHotSpotSelector = $200A;
	kQTVRGetHotSpotTypeSelector = $200B; { Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
	kQTVRSetViewParameterSelector = $200C; { Requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00)}
	kQTVRGetViewParameterSelector = $200D; { Requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00)}
>>>>>>> origin/fixes_2.4

type
	QTVRInterceptRecordPtr = ^QTVRInterceptRecord;
	QTVRInterceptRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		reserved1: SInt32;
		selector: SInt32;

		reserved2: SInt32;
		reserved3: SInt32;

		paramCount: SInt32;
		parameter: array [0..5] of UnivPtr;
	end;
type
	QTVRInterceptPtr = QTVRInterceptRecordPtr;
{ Prototype for Intercept Proc callback}
type
	QTVRInterceptProcPtr = procedure( qtvr: QTVRInstance; qtvrMsg: QTVRInterceptPtr; refCon: SInt32; var cancel: Boolean );
	QTVRInterceptUPP = QTVRInterceptProcPtr;
{
 *  NewQTVRInterceptUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTVRInterceptUPP( userRoutine: QTVRInterceptProcPtr ): QTVRInterceptUPP; external name '_NewQTVRInterceptUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
		reserved1:				SInt32;
		selector:				SInt32;
		reserved2:				SInt32;
		reserved3:				SInt32;
		paramCount:				SInt32;
		parameter:				array [0..5] of Ptr;
	end;
=======
		reserved1: SInt32;
		selector: SInt32;
>>>>>>> origin/fixes_2.4

		reserved2: SInt32;
		reserved3: SInt32;

		paramCount: SInt32;
		parameter: array [0..5] of UnivPtr;
	end;
type
	QTVRInterceptPtr = QTVRInterceptRecordPtr;
{ Prototype for Intercept Proc callback}
type
	QTVRInterceptProcPtr = procedure( qtvr: QTVRInstance; qtvrMsg: QTVRInterceptPtr; refCon: SInt32; var cancel: Boolean );
	QTVRInterceptUPP = QTVRInterceptProcPtr;
{
 *  NewQTVRInterceptUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQTVRInterceptUPP( userRoutine: QTVRInterceptProcPtr ): QTVRInterceptUPP; external name '_NewQTVRInterceptUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	uppQTVRInterceptProcInfo = $00003FC0;
	{
	 *  NewQTVRInterceptUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.1 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewQTVRInterceptUPP(userRoutine: QTVRInterceptProcPtr): QTVRInterceptUPP; external name '_NewQTVRInterceptUPP'; { old name was NewQTVRInterceptProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  DisposeQTVRInterceptUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQTVRInterceptUPP( userUPP: QTVRInterceptUPP ); external name '_DisposeQTVRInterceptUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeQTVRInterceptUPP(userUPP: QTVRInterceptUPP); external name '_DisposeQTVRInterceptUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeQTVRInterceptUPP( userUPP: QTVRInterceptUPP ); external name '_DisposeQTVRInterceptUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeQTVRInterceptUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQTVRInterceptUPP( qtvr: QTVRInstance; qtvrMsg: QTVRInterceptPtr; refCon: SInt32; var cancel: Boolean; userUPP: QTVRInterceptUPP ); external name '_InvokeQTVRInterceptUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure InvokeQTVRInterceptUPP(qtvr: QTVRInstance; qtvrMsg: QTVRInterceptPtr; refCon: SInt32; var cancel: boolean; userRoutine: QTVRInterceptUPP); external name '_InvokeQTVRInterceptUPP'; { old name was CallQTVRInterceptProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvokeQTVRInterceptUPP( qtvr: QTVRInstance; qtvrMsg: QTVRInterceptPtr; refCon: SInt32; var cancel: Boolean; userUPP: QTVRInterceptUPP ); external name '_InvokeQTVRInterceptUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
  =================================================================================================
    Initialization QTVR calls 
  -------------------------------------------------------------------------------------------------
   Requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10) and only work on Non-Macintosh platforms
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{$ifc NOT TARGET_OS_MAC}
{$ifc CALL_NOT_IN_CARBON}
{
 *  InitializeQTVR()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 *    Windows:          in QTVR.lib 2.1 and later
 }
function InitializeQTVR: OSErr; external name '_InitializeQTVR';

{
 *  TerminateQTVR()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 *    Windows:          in QTVR.lib 2.1 and later
 }
function TerminateQTVR: OSErr; external name '_TerminateQTVR';

{$endc}  {CALL_NOT_IN_CARBON}
{$endc}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
  =================================================================================================
    General QTVR calls 
  -------------------------------------------------------------------------------------------------
}
{
 *  QTVRGetQTVRTrack()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetQTVRTrack( theMovie: Movie; index: SInt32 ): Track; external name '_QTVRGetQTVRTrack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetQTVRTrack(theMovie: Movie; index: SInt32): Track; external name '_QTVRGetQTVRTrack';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetQTVRTrack( theMovie: Movie; index: SInt32 ): Track; external name '_QTVRGetQTVRTrack';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetQTVRInstance()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetQTVRInstance( var qtvr: QTVRInstance; qtvrTrack: Track; mc: MovieController ): OSErr; external name '_QTVRGetQTVRInstance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetQTVRInstance(var qtvr: QTVRInstance; qtvrTrack: Track; mc: MovieController): OSErr; external name '_QTVRGetQTVRInstance';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetQTVRInstance( var qtvr: QTVRInstance; qtvrTrack: Track; mc: MovieController ): OSErr; external name '_QTVRGetQTVRInstance';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Viewing Angles and Zooming 
  -------------------------------------------------------------------------------------------------
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ QTVRSetViewParameter requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00)}
=======
{  QTVRSetViewParameter requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00) }
>>>>>>> graemeg/fixes_2_2
=======
{  QTVRSetViewParameter requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00) }
>>>>>>> origin/fixes_2_2
=======
{ QTVRSetViewParameter requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00)}
>>>>>>> origin/fixes_2.4
{
 *  QTVRSetViewParameter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 5.0 and later
 *    Windows:          in QTVR.lib 5.0 and later
 }
function QTVRSetViewParameter( qtvr: QTVRInstance; viewParameter: UInt32; value: UnivPtr; flagsIn: UInt32 ): OSErr; external name '_QTVRSetViewParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ QTVRGetViewParameter requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00)}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 5.0 and later
 *    Windows:          in QTVR.lib 5.0 and later
 }
function QTVRSetViewParameter( qtvr: QTVRInstance; viewParameter: UInt32; value: UnivPtr; flagsIn: UInt32 ): OSErr; external name '_QTVRSetViewParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
{  QTVRGetViewParameter requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ QTVRGetViewParameter requires QTVR 5.0 (kQTVRAPIMajorVersion05 + kQTVRAPIMinorVersion00)}
>>>>>>> origin/fixes_2.4
{
 *  QTVRGetViewParameter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 5.0 and later
 *    Windows:          in QTVR.lib 5.0 and later
 }
function QTVRGetViewParameter( qtvr: QTVRInstance; viewParameter: UInt32; value: UnivPtr; flagsIn: UInt32; var flagsOut: UInt32 ): OSErr; external name '_QTVRGetViewParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 5.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 5.0 and later
 *    Windows:          in QTVR.lib 5.0 and later
 }
<<<<<<< HEAD
function QTVRGetViewParameter(qtvr: QTVRInstance; viewParameter: UInt32; value: UnivPtr; flagsIn: UInt32; var flagsOut: UInt32): OSErr; external name '_QTVRGetViewParameter';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetViewParameter( qtvr: QTVRInstance; viewParameter: UInt32; value: UnivPtr; flagsIn: UInt32; var flagsOut: UInt32 ): OSErr; external name '_QTVRGetViewParameter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetPanAngle()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetPanAngle( qtvr: QTVRInstance; panAngle: Float32 ): OSErr; external name '_QTVRSetPanAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetPanAngle(qtvr: QTVRInstance; panAngle: Single): OSErr; external name '_QTVRSetPanAngle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetPanAngle( qtvr: QTVRInstance; panAngle: Float32 ): OSErr; external name '_QTVRSetPanAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetPanAngle()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetPanAngle( qtvr: QTVRInstance ): Float32; external name '_QTVRGetPanAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetPanAngle(qtvr: QTVRInstance): Single; external name '_QTVRGetPanAngle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

{
 *  QTVRSetTiltAngle()
 *  
 *  Availability:
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetTiltAngle( qtvr: QTVRInstance; tiltAngle: Float32 ): OSErr; external name '_QTVRSetTiltAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
=======
function QTVRGetPanAngle( qtvr: QTVRInstance ): Float32; external name '_QTVRGetPanAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetTiltAngle()
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetTiltAngle(qtvr: QTVRInstance; tiltAngle: Single): OSErr; external name '_QTVRSetTiltAngle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetTiltAngle( qtvr: QTVRInstance; tiltAngle: Float32 ): OSErr; external name '_QTVRSetTiltAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetTiltAngle()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetTiltAngle( qtvr: QTVRInstance ): Float32; external name '_QTVRGetTiltAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetTiltAngle(qtvr: QTVRInstance): Single; external name '_QTVRGetTiltAngle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetTiltAngle( qtvr: QTVRInstance ): Float32; external name '_QTVRGetTiltAngle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetFieldOfView()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetFieldOfView( qtvr: QTVRInstance; fieldOfView: Float32 ): OSErr; external name '_QTVRSetFieldOfView';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetFieldOfView(qtvr: QTVRInstance; fieldOfView: Single): OSErr; external name '_QTVRSetFieldOfView';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetFieldOfView( qtvr: QTVRInstance; fieldOfView: Float32 ): OSErr; external name '_QTVRSetFieldOfView';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetFieldOfView()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetFieldOfView( qtvr: QTVRInstance ): Float32; external name '_QTVRGetFieldOfView';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetFieldOfView(qtvr: QTVRInstance): Single; external name '_QTVRGetFieldOfView';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetFieldOfView( qtvr: QTVRInstance ): Float32; external name '_QTVRGetFieldOfView';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRShowDefaultView()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRShowDefaultView( qtvr: QTVRInstance ): OSErr; external name '_QTVRShowDefaultView';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
{ Object Specific}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRShowDefaultView( qtvr: QTVRInstance ): OSErr; external name '_QTVRShowDefaultView';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{  Object Specific }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ Object Specific}
>>>>>>> origin/fixes_2.4
{
 *  QTVRSetViewCenter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetViewCenter( qtvr: QTVRInstance; const (*var*) viewCenter: QTVRFloatPoint ): OSErr; external name '_QTVRSetViewCenter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetViewCenter(qtvr: QTVRInstance; const (*var*) viewCenter: QTVRFloatPoint): OSErr; external name '_QTVRSetViewCenter';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetViewCenter( qtvr: QTVRInstance; const (*var*) viewCenter: QTVRFloatPoint ): OSErr; external name '_QTVRSetViewCenter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetViewCenter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetViewCenter( qtvr: QTVRInstance; var viewCenter: QTVRFloatPoint ): OSErr; external name '_QTVRGetViewCenter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetViewCenter(qtvr: QTVRInstance; var viewCenter: QTVRFloatPoint): OSErr; external name '_QTVRGetViewCenter';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetViewCenter( qtvr: QTVRInstance; var viewCenter: QTVRFloatPoint ): OSErr; external name '_QTVRGetViewCenter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRNudge()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRNudge( qtvr: QTVRInstance; direction: QTVRNudgeControl ): OSErr; external name '_QTVRNudge';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
{ QTVRInteractionNudge requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRNudge( qtvr: QTVRInstance; direction: QTVRNudgeControl ): OSErr; external name '_QTVRNudge';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{  QTVRInteractionNudge requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ QTVRInteractionNudge requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
>>>>>>> origin/fixes_2.4
{
 *  QTVRInteractionNudge()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRInteractionNudge( qtvr: QTVRInstance; direction: QTVRNudgeControl ): OSErr; external name '_QTVRInteractionNudge';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRInteractionNudge(qtvr: QTVRInstance; direction: QTVRNudgeControl): OSErr; external name '_QTVRInteractionNudge';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRInteractionNudge( qtvr: QTVRInstance; direction: QTVRNudgeControl ): OSErr; external name '_QTVRInteractionNudge';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Scene and Node Location Information 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRGetVRWorld()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetVRWorld( qtvr: QTVRInstance; var VRWorld: QTAtomContainer ): OSErr; external name '_QTVRGetVRWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetVRWorld(qtvr: QTVRInstance; var VRWorld: QTAtomContainer): OSErr; external name '_QTVRGetVRWorld';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetVRWorld( qtvr: QTVRInstance; var VRWorld: QTAtomContainer ): OSErr; external name '_QTVRGetVRWorld';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetNodeInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetNodeInfo( qtvr: QTVRInstance; nodeID: UInt32; var nodeInfo: QTAtomContainer ): OSErr; external name '_QTVRGetNodeInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetNodeInfo(qtvr: QTVRInstance; nodeID: UInt32; var nodeInfo: QTAtomContainer): OSErr; external name '_QTVRGetNodeInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetNodeInfo( qtvr: QTVRInstance; nodeID: UInt32; var nodeInfo: QTAtomContainer ): OSErr; external name '_QTVRGetNodeInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGoToNodeID()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGoToNodeID( qtvr: QTVRInstance; nodeID: UInt32 ): OSErr; external name '_QTVRGoToNodeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGoToNodeID(qtvr: QTVRInstance; nodeID: UInt32): OSErr; external name '_QTVRGoToNodeID';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGoToNodeID( qtvr: QTVRInstance; nodeID: UInt32 ): OSErr; external name '_QTVRGoToNodeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetCurrentNodeID()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetCurrentNodeID( qtvr: QTVRInstance ): UInt32; external name '_QTVRGetCurrentNodeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetCurrentNodeID(qtvr: QTVRInstance): UInt32; external name '_QTVRGetCurrentNodeID';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetCurrentNodeID( qtvr: QTVRInstance ): UInt32; external name '_QTVRGetCurrentNodeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetNodeType()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetNodeType( qtvr: QTVRInstance; nodeID: UInt32 ): OSType; external name '_QTVRGetNodeType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetNodeType(qtvr: QTVRInstance; nodeID: UInt32): SInt32; external name '_QTVRGetNodeType';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetNodeType( qtvr: QTVRInstance; nodeID: UInt32 ): OSType; external name '_QTVRGetNodeType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Hot Spot related calls 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRPtToHotSpotID()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRPtToHotSpotID( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32 ): OSErr; external name '_QTVRPtToHotSpotID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
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
{ QTVRGetHotSpotType requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRPtToHotSpotID( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32 ): OSErr; external name '_QTVRPtToHotSpotID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
{  QTVRGetHotSpotType requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ QTVRGetHotSpotType requires QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
>>>>>>> origin/fixes_2.4
{
 *  QTVRGetHotSpotType()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetHotSpotType( qtvr: QTVRInstance; hotSpotID: UInt32; var hotSpotType: OSType ): OSErr; external name '_QTVRGetHotSpotType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetHotSpotType(qtvr: QTVRInstance; hotSpotID: UInt32; var hotSpotType: OSType): OSErr; external name '_QTVRGetHotSpotType';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetHotSpotType( qtvr: QTVRInstance; hotSpotID: UInt32; var hotSpotType: OSType ): OSErr; external name '_QTVRGetHotSpotType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRTriggerHotSpot()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRTriggerHotSpot( qtvr: QTVRInstance; hotSpotID: UInt32; nodeInfo: QTAtomContainer; selectedAtom: QTAtom ): OSErr; external name '_QTVRTriggerHotSpot';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRTriggerHotSpot(qtvr: QTVRInstance; hotSpotID: UInt32; nodeInfo: QTAtomContainer; selectedAtom: QTAtom): OSErr; external name '_QTVRTriggerHotSpot';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRTriggerHotSpot( qtvr: QTVRInstance; hotSpotID: UInt32; nodeInfo: QTAtomContainer; selectedAtom: QTAtom ): OSErr; external name '_QTVRTriggerHotSpot';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetMouseOverHotSpotProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetMouseOverHotSpotProc( qtvr: QTVRInstance; mouseOverHotSpotProc: QTVRMouseOverHotSpotUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRSetMouseOverHotSpotProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetMouseOverHotSpotProc(qtvr: QTVRInstance; mouseOverHotSpotProc: QTVRMouseOverHotSpotUPP; refCon: SInt32; flags: UInt32): OSErr; external name '_QTVRSetMouseOverHotSpotProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetMouseOverHotSpotProc( qtvr: QTVRInstance; mouseOverHotSpotProc: QTVRMouseOverHotSpotUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRSetMouseOverHotSpotProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVREnableHotSpot()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVREnableHotSpot( qtvr: QTVRInstance; enableFlag: UInt32; hotSpotValue: UInt32; enable: Boolean ): OSErr; external name '_QTVREnableHotSpot';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVREnableHotSpot(qtvr: QTVRInstance; enableFlag: UInt32; hotSpotValue: UInt32; enable: boolean): OSErr; external name '_QTVREnableHotSpot';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVREnableHotSpot( qtvr: QTVRInstance; enableFlag: UInt32; hotSpotValue: UInt32; enable: Boolean ): OSErr; external name '_QTVREnableHotSpot';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetVisibleHotSpots()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetVisibleHotSpots( qtvr: QTVRInstance; hotSpots: Handle ): UInt32; external name '_QTVRGetVisibleHotSpots';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetVisibleHotSpots(qtvr: QTVRInstance; hotSpots: Handle): UInt32; external name '_QTVRGetVisibleHotSpots';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetVisibleHotSpots( qtvr: QTVRInstance; hotSpots: Handle ): UInt32; external name '_QTVRGetVisibleHotSpots';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetHotSpotRegion()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetHotSpotRegion( qtvr: QTVRInstance; hotSpotID: UInt32; hotSpotRegion: RgnHandle ): OSErr; external name '_QTVRGetHotSpotRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetHotSpotRegion(qtvr: QTVRInstance; hotSpotID: UInt32; hotSpotRegion: RgnHandle): OSErr; external name '_QTVRGetHotSpotRegion';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetHotSpotRegion( qtvr: QTVRInstance; hotSpotID: UInt32; hotSpotRegion: RgnHandle ): OSErr; external name '_QTVRGetHotSpotRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Event & Cursor Handling Calls 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRSetMouseOverTracking()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetMouseOverTracking( qtvr: QTVRInstance; enable: Boolean ): OSErr; external name '_QTVRSetMouseOverTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetMouseOverTracking(qtvr: QTVRInstance; enable: boolean): OSErr; external name '_QTVRSetMouseOverTracking';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetMouseOverTracking( qtvr: QTVRInstance; enable: Boolean ): OSErr; external name '_QTVRSetMouseOverTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetMouseOverTracking()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetMouseOverTracking( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetMouseOverTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetMouseOverTracking(qtvr: QTVRInstance): boolean; external name '_QTVRGetMouseOverTracking';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetMouseOverTracking( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetMouseOverTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetMouseDownTracking()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetMouseDownTracking( qtvr: QTVRInstance; enable: Boolean ): OSErr; external name '_QTVRSetMouseDownTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetMouseDownTracking(qtvr: QTVRInstance; enable: boolean): OSErr; external name '_QTVRSetMouseDownTracking';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetMouseDownTracking( qtvr: QTVRInstance; enable: Boolean ): OSErr; external name '_QTVRSetMouseDownTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetMouseDownTracking()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetMouseDownTracking( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetMouseDownTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetMouseDownTracking(qtvr: QTVRInstance): boolean; external name '_QTVRGetMouseDownTracking';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetMouseDownTracking( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetMouseDownTracking';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRMouseEnter()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseEnter( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseEnter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRMouseEnter(qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef): OSErr; external name '_QTVRMouseEnter';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRMouseEnter( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseEnter';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRMouseWithin()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseWithin( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseWithin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRMouseWithin(qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef): OSErr; external name '_QTVRMouseWithin';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRMouseWithin( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseWithin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRMouseLeave()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseLeave( qtvr: QTVRInstance; pt: Point; w: WindowRef ): OSErr; external name '_QTVRMouseLeave';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRMouseLeave(qtvr: QTVRInstance; pt: Point; w: WindowRef): OSErr; external name '_QTVRMouseLeave';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRMouseLeave( qtvr: QTVRInstance; pt: Point; w: WindowRef ): OSErr; external name '_QTVRMouseLeave';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRMouseDown()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseDown( qtvr: QTVRInstance; pt: Point; when: UInt32; modifiers: UInt16; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseDown';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRMouseDown(qtvr: QTVRInstance; pt: Point; when: UInt32; modifiers: UInt16; var hotSpotID: UInt32; w: WindowRef): OSErr; external name '_QTVRMouseDown';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRMouseDown( qtvr: QTVRInstance; pt: Point; when: UInt32; modifiers: UInt16; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseDown';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRMouseStillDown()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseStillDown( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseStillDown';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRMouseStillDown(qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef): OSErr; external name '_QTVRMouseStillDown';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRMouseStillDown( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseStillDown';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRMouseUp()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseUp( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseUp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ These require QTVR 2.01 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion01)}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseUp( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef ): OSErr; external name '_QTVRMouseUp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{  These require QTVR 2.01 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion01) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ These require QTVR 2.01 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion01)}
>>>>>>> origin/fixes_2.4
{
 *  QTVRMouseStillDownExtended()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseStillDownExtended( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef; when: UInt32; modifiers: UInt16 ): OSErr; external name '_QTVRMouseStillDownExtended';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRMouseStillDownExtended(qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef; when: UInt32; modifiers: UInt16): OSErr; external name '_QTVRMouseStillDownExtended';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRMouseStillDownExtended( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef; when: UInt32; modifiers: UInt16 ): OSErr; external name '_QTVRMouseStillDownExtended';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRMouseUpExtended()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRMouseUpExtended( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef; when: UInt32; modifiers: UInt16 ): OSErr; external name '_QTVRMouseUpExtended';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRMouseUpExtended(qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef; when: UInt32; modifiers: UInt16): OSErr; external name '_QTVRMouseUpExtended';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRMouseUpExtended( qtvr: QTVRInstance; pt: Point; var hotSpotID: UInt32; w: WindowRef; when: UInt32; modifiers: UInt16 ): OSErr; external name '_QTVRMouseUpExtended';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Intercept Routines 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRInstallInterceptProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRInstallInterceptProc( qtvr: QTVRInstance; selector: QTVRProcSelector; interceptProc: QTVRInterceptUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRInstallInterceptProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRInstallInterceptProc(qtvr: QTVRInstance; selector: QTVRProcSelector; interceptProc: QTVRInterceptUPP; refCon: SInt32; flags: UInt32): OSErr; external name '_QTVRInstallInterceptProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRInstallInterceptProc( qtvr: QTVRInstance; selector: QTVRProcSelector; interceptProc: QTVRInterceptUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRInstallInterceptProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRCallInterceptedProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRCallInterceptedProc( qtvr: QTVRInstance; var qtvrMsg: QTVRInterceptRecord ): OSErr; external name '_QTVRCallInterceptedProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRCallInterceptedProc(qtvr: QTVRInstance; var qtvrMsg: QTVRInterceptRecord): OSErr; external name '_QTVRCallInterceptedProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRCallInterceptedProc( qtvr: QTVRInstance; var qtvrMsg: QTVRInterceptRecord ): OSErr; external name '_QTVRCallInterceptedProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Object Movie Specific Calls 
  -------------------------------------------------------------------------------------------------
   QTVRGetCurrentMouseMode requires QTRVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)
}
{
 *  QTVRGetCurrentMouseMode()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetCurrentMouseMode( qtvr: QTVRInstance ): UInt32; external name '_QTVRGetCurrentMouseMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetCurrentMouseMode(qtvr: QTVRInstance): UInt32; external name '_QTVRGetCurrentMouseMode';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetCurrentMouseMode( qtvr: QTVRInstance ): UInt32; external name '_QTVRGetCurrentMouseMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetFrameRate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetFrameRate( qtvr: QTVRInstance; rate: Float32 ): OSErr; external name '_QTVRSetFrameRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetFrameRate(qtvr: QTVRInstance; rate: Single): OSErr; external name '_QTVRSetFrameRate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetFrameRate( qtvr: QTVRInstance; rate: Float32 ): OSErr; external name '_QTVRSetFrameRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetFrameRate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetFrameRate( qtvr: QTVRInstance ): Float32; external name '_QTVRGetFrameRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetFrameRate(qtvr: QTVRInstance): Single; external name '_QTVRGetFrameRate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetFrameRate( qtvr: QTVRInstance ): Float32; external name '_QTVRGetFrameRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetViewRate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetViewRate( qtvr: QTVRInstance; rate: Float32 ): OSErr; external name '_QTVRSetViewRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetViewRate(qtvr: QTVRInstance; rate: Single): OSErr; external name '_QTVRSetViewRate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetViewRate( qtvr: QTVRInstance; rate: Float32 ): OSErr; external name '_QTVRSetViewRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetViewRate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetViewRate( qtvr: QTVRInstance ): Float32; external name '_QTVRGetViewRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetViewRate(qtvr: QTVRInstance): Single; external name '_QTVRGetViewRate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetViewRate( qtvr: QTVRInstance ): Float32; external name '_QTVRGetViewRate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetViewCurrentTime()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetViewCurrentTime( qtvr: QTVRInstance; time: TimeValue ): OSErr; external name '_QTVRSetViewCurrentTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetViewCurrentTime(qtvr: QTVRInstance; time: TimeValue): OSErr; external name '_QTVRSetViewCurrentTime';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetViewCurrentTime( qtvr: QTVRInstance; time: TimeValue ): OSErr; external name '_QTVRSetViewCurrentTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetViewCurrentTime()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetViewCurrentTime( qtvr: QTVRInstance ): TimeValue; external name '_QTVRGetViewCurrentTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetViewCurrentTime(qtvr: QTVRInstance): TimeValue; external name '_QTVRGetViewCurrentTime';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetViewCurrentTime( qtvr: QTVRInstance ): TimeValue; external name '_QTVRGetViewCurrentTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetCurrentViewDuration()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetCurrentViewDuration( qtvr: QTVRInstance ): TimeValue; external name '_QTVRGetCurrentViewDuration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetCurrentViewDuration(qtvr: QTVRInstance): TimeValue; external name '_QTVRGetCurrentViewDuration';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetCurrentViewDuration( qtvr: QTVRInstance ): TimeValue; external name '_QTVRGetCurrentViewDuration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
   View State Calls - QTVR Object Only
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRSetViewState()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetViewState( qtvr: QTVRInstance; viewStateType: QTVRViewStateType; state: UInt16 ): OSErr; external name '_QTVRSetViewState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetViewState(qtvr: QTVRInstance; viewStateType: QTVRViewStateType; state: UInt16): OSErr; external name '_QTVRSetViewState';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetViewState( qtvr: QTVRInstance; viewStateType: QTVRViewStateType; state: UInt16 ): OSErr; external name '_QTVRSetViewState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetViewState()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetViewState( qtvr: QTVRInstance; viewStateType: QTVRViewStateType; var state: UInt16 ): OSErr; external name '_QTVRGetViewState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetViewState(qtvr: QTVRInstance; viewStateType: QTVRViewStateType; var state: UInt16): OSErr; external name '_QTVRGetViewState';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetViewState( qtvr: QTVRInstance; viewStateType: QTVRViewStateType; var state: UInt16 ): OSErr; external name '_QTVRGetViewState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetViewStateCount()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetViewStateCount( qtvr: QTVRInstance ): UInt16; external name '_QTVRGetViewStateCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetViewStateCount(qtvr: QTVRInstance): UInt16; external name '_QTVRGetViewStateCount';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetViewStateCount( qtvr: QTVRInstance ): UInt16; external name '_QTVRGetViewStateCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetAnimationSetting()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetAnimationSetting( qtvr: QTVRInstance; setting: QTVRObjectAnimationSetting; enable: Boolean ): OSErr; external name '_QTVRSetAnimationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetAnimationSetting(qtvr: QTVRInstance; setting: QTVRObjectAnimationSetting; enable: boolean): OSErr; external name '_QTVRSetAnimationSetting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetAnimationSetting( qtvr: QTVRInstance; setting: QTVRObjectAnimationSetting; enable: Boolean ): OSErr; external name '_QTVRSetAnimationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetAnimationSetting()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetAnimationSetting( qtvr: QTVRInstance; setting: QTVRObjectAnimationSetting; var enable: Boolean ): OSErr; external name '_QTVRGetAnimationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetAnimationSetting(qtvr: QTVRInstance; setting: QTVRObjectAnimationSetting; var enable: boolean): OSErr; external name '_QTVRGetAnimationSetting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetAnimationSetting( qtvr: QTVRInstance; setting: QTVRObjectAnimationSetting; var enable: Boolean ): OSErr; external name '_QTVRGetAnimationSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetControlSetting()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetControlSetting( qtvr: QTVRInstance; setting: QTVRControlSetting; enable: Boolean ): OSErr; external name '_QTVRSetControlSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetControlSetting(qtvr: QTVRInstance; setting: QTVRControlSetting; enable: boolean): OSErr; external name '_QTVRSetControlSetting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetControlSetting( qtvr: QTVRInstance; setting: QTVRControlSetting; enable: Boolean ): OSErr; external name '_QTVRSetControlSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetControlSetting()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetControlSetting( qtvr: QTVRInstance; setting: QTVRControlSetting; var enable: Boolean ): OSErr; external name '_QTVRGetControlSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetControlSetting(qtvr: QTVRInstance; setting: QTVRControlSetting; var enable: boolean): OSErr; external name '_QTVRGetControlSetting';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetControlSetting( qtvr: QTVRInstance; setting: QTVRControlSetting; var enable: Boolean ): OSErr; external name '_QTVRGetControlSetting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVREnableFrameAnimation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVREnableFrameAnimation( qtvr: QTVRInstance; enable: Boolean ): OSErr; external name '_QTVREnableFrameAnimation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVREnableFrameAnimation(qtvr: QTVRInstance; enable: boolean): OSErr; external name '_QTVREnableFrameAnimation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVREnableFrameAnimation( qtvr: QTVRInstance; enable: Boolean ): OSErr; external name '_QTVREnableFrameAnimation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetFrameAnimation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetFrameAnimation( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetFrameAnimation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetFrameAnimation(qtvr: QTVRInstance): boolean; external name '_QTVRGetFrameAnimation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetFrameAnimation( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetFrameAnimation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVREnableViewAnimation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVREnableViewAnimation( qtvr: QTVRInstance; enable: Boolean ): OSErr; external name '_QTVREnableViewAnimation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVREnableViewAnimation(qtvr: QTVRInstance; enable: boolean): OSErr; external name '_QTVREnableViewAnimation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVREnableViewAnimation( qtvr: QTVRInstance; enable: Boolean ): OSErr; external name '_QTVREnableViewAnimation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetViewAnimation()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetViewAnimation( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetViewAnimation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetViewAnimation(qtvr: QTVRInstance): boolean; external name '_QTVRGetViewAnimation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetViewAnimation( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetViewAnimation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
  =================================================================================================
    Imaging Characteristics 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRSetVisible()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetVisible( qtvr: QTVRInstance; visible: Boolean ): OSErr; external name '_QTVRSetVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetVisible(qtvr: QTVRInstance; visible: boolean): OSErr; external name '_QTVRSetVisible';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetVisible( qtvr: QTVRInstance; visible: Boolean ): OSErr; external name '_QTVRSetVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetVisible()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetVisible( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetVisible(qtvr: QTVRInstance): boolean; external name '_QTVRGetVisible';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetVisible( qtvr: QTVRInstance ): Boolean; external name '_QTVRGetVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetImagingProperty()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetImagingProperty( qtvr: QTVRInstance; imagingMode: QTVRImagingMode; imagingProperty: UInt32; propertyValue: SInt32 ): OSErr; external name '_QTVRSetImagingProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetImagingProperty(qtvr: QTVRInstance; imagingMode: QTVRImagingMode; imagingProperty: UInt32; propertyValue: SInt32): OSErr; external name '_QTVRSetImagingProperty';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetImagingProperty( qtvr: QTVRInstance; imagingMode: QTVRImagingMode; imagingProperty: UInt32; propertyValue: SInt32 ): OSErr; external name '_QTVRSetImagingProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetImagingProperty()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetImagingProperty( qtvr: QTVRInstance; imagingMode: QTVRImagingMode; imagingProperty: UInt32; var propertyValue: SInt32 ): OSErr; external name '_QTVRGetImagingProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetImagingProperty(qtvr: QTVRInstance; imagingMode: QTVRImagingMode; imagingProperty: UInt32; var propertyValue: SInt32): OSErr; external name '_QTVRGetImagingProperty';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetImagingProperty( qtvr: QTVRInstance; imagingMode: QTVRImagingMode; imagingProperty: UInt32; var propertyValue: SInt32 ): OSErr; external name '_QTVRGetImagingProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRUpdate()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRUpdate( qtvr: QTVRInstance; imagingMode: QTVRImagingMode ): OSErr; external name '_QTVRUpdate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRUpdate(qtvr: QTVRInstance; imagingMode: QTVRImagingMode): OSErr; external name '_QTVRUpdate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRUpdate( qtvr: QTVRInstance; imagingMode: QTVRImagingMode ): OSErr; external name '_QTVRUpdate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRBeginUpdateStream()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRBeginUpdateStream( qtvr: QTVRInstance; imagingMode: QTVRImagingMode ): OSErr; external name '_QTVRBeginUpdateStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRBeginUpdateStream(qtvr: QTVRInstance; imagingMode: QTVRImagingMode): OSErr; external name '_QTVRBeginUpdateStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRBeginUpdateStream( qtvr: QTVRInstance; imagingMode: QTVRImagingMode ): OSErr; external name '_QTVRBeginUpdateStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVREndUpdateStream()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVREndUpdateStream( qtvr: QTVRInstance ): OSErr; external name '_QTVREndUpdateStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVREndUpdateStream(qtvr: QTVRInstance): OSErr; external name '_QTVREndUpdateStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVREndUpdateStream( qtvr: QTVRInstance ): OSErr; external name '_QTVREndUpdateStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetTransitionProperty()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetTransitionProperty( qtvr: QTVRInstance; transitionType: UInt32; transitionProperty: UInt32; transitionValue: SInt32 ): OSErr; external name '_QTVRSetTransitionProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetTransitionProperty(qtvr: QTVRInstance; transitionType: UInt32; transitionProperty: UInt32; transitionValue: SInt32): OSErr; external name '_QTVRSetTransitionProperty';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetTransitionProperty( qtvr: QTVRInstance; transitionType: UInt32; transitionProperty: UInt32; transitionValue: SInt32 ): OSErr; external name '_QTVRSetTransitionProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVREnableTransition()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVREnableTransition( qtvr: QTVRInstance; transitionType: UInt32; enable: Boolean ): OSErr; external name '_QTVREnableTransition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVREnableTransition(qtvr: QTVRInstance; transitionType: UInt32; enable: boolean): OSErr; external name '_QTVREnableTransition';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVREnableTransition( qtvr: QTVRInstance; transitionType: UInt32; enable: Boolean ): OSErr; external name '_QTVREnableTransition';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Basic Conversion and Math Routines 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRSetAngularUnits()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetAngularUnits( qtvr: QTVRInstance; units: QTVRAngularUnits ): OSErr; external name '_QTVRSetAngularUnits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetAngularUnits(qtvr: QTVRInstance; units: QTVRAngularUnits): OSErr; external name '_QTVRSetAngularUnits';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetAngularUnits( qtvr: QTVRInstance; units: QTVRAngularUnits ): OSErr; external name '_QTVRSetAngularUnits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetAngularUnits()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetAngularUnits( qtvr: QTVRInstance ): QTVRAngularUnits; external name '_QTVRGetAngularUnits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Pano specific routines}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetAngularUnits( qtvr: QTVRInstance ): QTVRAngularUnits; external name '_QTVRGetAngularUnits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
{  Pano specific routines }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Pano specific routines}
>>>>>>> origin/fixes_2.4
{
 *  QTVRPtToAngles()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRPtToAngles( qtvr: QTVRInstance; pt: Point; var panAngle: Float32; var tiltAngle: Float32 ): OSErr; external name '_QTVRPtToAngles';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRPtToAngles(qtvr: QTVRInstance; pt: Point; var panAngle: Single; var tiltAngle: Single): OSErr; external name '_QTVRPtToAngles';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRPtToAngles( qtvr: QTVRInstance; pt: Point; var panAngle: Float32; var tiltAngle: Float32 ): OSErr; external name '_QTVRPtToAngles';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRCoordToAngles()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRCoordToAngles( qtvr: QTVRInstance; var coord: QTVRFloatPoint; var panAngle: Float32; var tiltAngle: Float32 ): OSErr; external name '_QTVRCoordToAngles';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRCoordToAngles(qtvr: QTVRInstance; var coord: QTVRFloatPoint; var panAngle: Single; var tiltAngle: Single): OSErr; external name '_QTVRCoordToAngles';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRCoordToAngles( qtvr: QTVRInstance; var coord: QTVRFloatPoint; var panAngle: Float32; var tiltAngle: Float32 ): OSErr; external name '_QTVRCoordToAngles';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRAnglesToCoord()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRAnglesToCoord( qtvr: QTVRInstance; panAngle: Float32; tiltAngle: Float32; var coord: QTVRFloatPoint ): OSErr; external name '_QTVRAnglesToCoord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
{ Object specific routines}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRAnglesToCoord( qtvr: QTVRInstance; panAngle: Float32; tiltAngle: Float32; var coord: QTVRFloatPoint ): OSErr; external name '_QTVRAnglesToCoord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
{  Object specific routines }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Object specific routines}
>>>>>>> origin/fixes_2.4
{
 *  QTVRPanToColumn()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRPanToColumn( qtvr: QTVRInstance; panAngle: Float32 ): SInt16; external name '_QTVRPanToColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
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
{ zero based   }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRPanToColumn( qtvr: QTVRInstance; panAngle: Float32 ): SInt16; external name '_QTVRPanToColumn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
{  zero based    }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ zero based   }
>>>>>>> origin/fixes_2.4
{
 *  QTVRColumnToPan()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRColumnToPan( qtvr: QTVRInstance; column: SInt16 ): Float32; external name '_QTVRColumnToPan';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
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
{ zero based   }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRColumnToPan( qtvr: QTVRInstance; column: SInt16 ): Float32; external name '_QTVRColumnToPan';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{  zero based    }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ zero based   }
>>>>>>> origin/fixes_2.4
{
 *  QTVRTiltToRow()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRTiltToRow( qtvr: QTVRInstance; tiltAngle: Float32 ): SInt16; external name '_QTVRTiltToRow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ zero based   }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRTiltToRow( qtvr: QTVRInstance; tiltAngle: Float32 ): SInt16; external name '_QTVRTiltToRow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
{  zero based    }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ zero based   }
>>>>>>> origin/fixes_2.4
{
 *  QTVRRowToTilt()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRRowToTilt( qtvr: QTVRInstance; row: SInt16 ): Float32; external name '_QTVRRowToTilt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
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
{ zero based               }
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRRowToTilt( qtvr: QTVRInstance; row: SInt16 ): Float32; external name '_QTVRRowToTilt';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


<<<<<<< HEAD
{  zero based                }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ zero based               }
>>>>>>> origin/fixes_2.4
{
 *  QTVRWrapAndConstrain()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRWrapAndConstrain( qtvr: QTVRInstance; kind: SInt16; value: Float32; var result: Float32 ): OSErr; external name '_QTVRWrapAndConstrain';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRWrapAndConstrain(qtvr: QTVRInstance; kind: SInt16; value: Single; var result: Single): OSErr; external name '_QTVRWrapAndConstrain';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRWrapAndConstrain( qtvr: QTVRInstance; kind: SInt16; value: Float32; var result: Float32 ): OSErr; external name '_QTVRWrapAndConstrain';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
  =================================================================================================
    Interaction Routines 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRSetEnteringNodeProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetEnteringNodeProc( qtvr: QTVRInstance; enteringNodeProc: QTVREnteringNodeUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRSetEnteringNodeProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetEnteringNodeProc(qtvr: QTVRInstance; enteringNodeProc: QTVREnteringNodeUPP; refCon: SInt32; flags: UInt32): OSErr; external name '_QTVRSetEnteringNodeProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetEnteringNodeProc( qtvr: QTVRInstance; enteringNodeProc: QTVREnteringNodeUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRSetEnteringNodeProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetLeavingNodeProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetLeavingNodeProc( qtvr: QTVRInstance; leavingNodeProc: QTVRLeavingNodeUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRSetLeavingNodeProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetLeavingNodeProc(qtvr: QTVRInstance; leavingNodeProc: QTVRLeavingNodeUPP; refCon: SInt32; flags: UInt32): OSErr; external name '_QTVRSetLeavingNodeProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetLeavingNodeProc( qtvr: QTVRInstance; leavingNodeProc: QTVRLeavingNodeUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRSetLeavingNodeProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetInteractionProperty()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetInteractionProperty( qtvr: QTVRInstance; proprty: UInt32; value: UnivPtr ): OSErr; external name '_QTVRSetInteractionProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetInteractionProperty(qtvr: QTVRInstance; proprty: UInt32; value: UnivPtr): OSErr; external name '_QTVRSetInteractionProperty';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetInteractionProperty( qtvr: QTVRInstance; proprty: UInt32; value: UnivPtr ): OSErr; external name '_QTVRSetInteractionProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetInteractionProperty()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetInteractionProperty( qtvr: QTVRInstance; proprty: UInt32; value: UnivPtr ): OSErr; external name '_QTVRGetInteractionProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetInteractionProperty(qtvr: QTVRInstance; proprty: UInt32; value: UnivPtr): OSErr; external name '_QTVRGetInteractionProperty';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetInteractionProperty( qtvr: QTVRInstance; proprty: UInt32; value: UnivPtr ): OSErr; external name '_QTVRGetInteractionProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRReplaceCursor()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRReplaceCursor( qtvr: QTVRInstance; var cursRecord: QTVRCursorRecord ): OSErr; external name '_QTVRReplaceCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRReplaceCursor(qtvr: QTVRInstance; var cursRecord: QTVRCursorRecord): OSErr; external name '_QTVRReplaceCursor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRReplaceCursor( qtvr: QTVRInstance; var cursRecord: QTVRCursorRecord ): OSErr; external name '_QTVRReplaceCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Viewing Limits and Constraints 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRGetViewingLimits()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetViewingLimits( qtvr: QTVRInstance; kind: UInt16; var minValue: Float32; var maxValue: Float32 ): OSErr; external name '_QTVRGetViewingLimits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetViewingLimits(qtvr: QTVRInstance; kind: UInt16; var minValue: Single; var maxValue: Single): OSErr; external name '_QTVRGetViewingLimits';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetViewingLimits( qtvr: QTVRInstance; kind: UInt16; var minValue: Float32; var maxValue: Float32 ): OSErr; external name '_QTVRGetViewingLimits';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetConstraintStatus()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetConstraintStatus( qtvr: QTVRInstance ): UInt32; external name '_QTVRGetConstraintStatus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetConstraintStatus(qtvr: QTVRInstance): UInt32; external name '_QTVRGetConstraintStatus';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetConstraintStatus( qtvr: QTVRInstance ): UInt32; external name '_QTVRGetConstraintStatus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetConstraints()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetConstraints( qtvr: QTVRInstance; kind: UInt16; var minValue: Float32; var maxValue: Float32 ): OSErr; external name '_QTVRGetConstraints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetConstraints(qtvr: QTVRInstance; kind: UInt16; var minValue: Single; var maxValue: Single): OSErr; external name '_QTVRGetConstraints';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetConstraints( qtvr: QTVRInstance; kind: UInt16; var minValue: Float32; var maxValue: Float32 ): OSErr; external name '_QTVRGetConstraints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetConstraints()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetConstraints( qtvr: QTVRInstance; kind: UInt16; minValue: Float32; maxValue: Float32 ): OSErr; external name '_QTVRSetConstraints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetConstraints(qtvr: QTVRInstance; kind: UInt16; minValue: Single; maxValue: Single): OSErr; external name '_QTVRSetConstraints';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetConstraints( qtvr: QTVRInstance; kind: UInt16; minValue: Float32; maxValue: Float32 ): OSErr; external name '_QTVRSetConstraints';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
  =================================================================================================
    Back Buffer Memory Management 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRGetAvailableResolutions()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetAvailableResolutions( qtvr: QTVRInstance; var resolutionsMask: UInt16 ): OSErr; external name '_QTVRGetAvailableResolutions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
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
{ These require QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetAvailableResolutions( qtvr: QTVRInstance; var resolutionsMask: UInt16 ): OSErr; external name '_QTVRGetAvailableResolutions';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{  These require QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10) }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ These require QTVR 2.1 (kQTVRAPIMajorVersion02 + kQTVRAPIMinorVersion10)}
>>>>>>> origin/fixes_2.4
{
 *  QTVRGetBackBufferMemInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetBackBufferMemInfo( qtvr: QTVRInstance; geometry: UInt32; resolution: UInt16; cachePixelFormat: UInt32; var minCacheBytes: SInt32; var suggestedCacheBytes: SInt32; var fullCacheBytes: SInt32 ): OSErr; external name '_QTVRGetBackBufferMemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetBackBufferMemInfo(qtvr: QTVRInstance; geometry: UInt32; resolution: UInt16; cachePixelFormat: UInt32; var minCacheBytes: SInt32; var suggestedCacheBytes: SInt32; var fullCacheBytes: SInt32): OSErr; external name '_QTVRGetBackBufferMemInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetBackBufferMemInfo( qtvr: QTVRInstance; geometry: UInt32; resolution: UInt16; cachePixelFormat: UInt32; var minCacheBytes: SInt32; var suggestedCacheBytes: SInt32; var fullCacheBytes: SInt32 ): OSErr; external name '_QTVRGetBackBufferMemInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRGetBackBufferSettings()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRGetBackBufferSettings( qtvr: QTVRInstance; var geometry: UInt32; var resolution: UInt16; var cachePixelFormat: UInt32; var cacheSize: SInt16 ): OSErr; external name '_QTVRGetBackBufferSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRGetBackBufferSettings(qtvr: QTVRInstance; var geometry: UInt32; var resolution: UInt16; var cachePixelFormat: UInt32; var cacheSize: SInt16): OSErr; external name '_QTVRGetBackBufferSettings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRGetBackBufferSettings( qtvr: QTVRInstance; var geometry: UInt32; var resolution: UInt16; var cachePixelFormat: UInt32; var cacheSize: SInt16 ): OSErr; external name '_QTVRGetBackBufferSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetBackBufferPrefs()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetBackBufferPrefs( qtvr: QTVRInstance; geometry: UInt32; resolution: UInt16; cachePixelFormat: UInt32; cacheSize: SInt16 ): OSErr; external name '_QTVRSetBackBufferPrefs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.1 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetBackBufferPrefs(qtvr: QTVRInstance; geometry: UInt32; resolution: UInt16; cachePixelFormat: UInt32; cacheSize: SInt16): OSErr; external name '_QTVRSetBackBufferPrefs';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetBackBufferPrefs( qtvr: QTVRInstance; geometry: UInt32; resolution: UInt16; cachePixelFormat: UInt32; cacheSize: SInt16 ): OSErr; external name '_QTVRSetBackBufferPrefs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
  =================================================================================================
    Buffer Access 
  -------------------------------------------------------------------------------------------------
}

{
 *  QTVRSetPrescreenImagingCompleteProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetPrescreenImagingCompleteProc( qtvr: QTVRInstance; imagingCompleteProc: QTVRImagingCompleteUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRSetPrescreenImagingCompleteProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetPrescreenImagingCompleteProc(qtvr: QTVRInstance; imagingCompleteProc: QTVRImagingCompleteUPP; refCon: SInt32; flags: UInt32): OSErr; external name '_QTVRSetPrescreenImagingCompleteProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetPrescreenImagingCompleteProc( qtvr: QTVRInstance; imagingCompleteProc: QTVRImagingCompleteUPP; refCon: SInt32; flags: UInt32 ): OSErr; external name '_QTVRSetPrescreenImagingCompleteProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRSetBackBufferImagingProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRSetBackBufferImagingProc( qtvr: QTVRInstance; backBufferImagingProc: QTVRBackBufferImagingUPP; numAreas: UInt16; areasOfInterest: {variable-size-array} QTVRAreaOfInterestPtr; refCon: SInt32 ): OSErr; external name '_QTVRSetBackBufferImagingProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
<<<<<<< HEAD
function QTVRSetBackBufferImagingProc(qtvr: QTVRInstance; backBufferImagingProc: QTVRBackBufferImagingUPP; numAreas: UInt16; var areasOfInterest: QTVRAreaOfInterest; refCon: SInt32): OSErr; external name '_QTVRSetBackBufferImagingProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function QTVRSetBackBufferImagingProc( qtvr: QTVRInstance; backBufferImagingProc: QTVRBackBufferImagingUPP; numAreas: UInt16; areasOfInterest: {variable-size-array} QTVRAreaOfInterestPtr; refCon: SInt32 ): OSErr; external name '_QTVRSetBackBufferImagingProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  QTVRRefreshBackBuffer()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRRefreshBackBuffer( qtvr: QTVRInstance; flags: UInt32 ): OSErr; external name '_QTVRRefreshBackBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
=======
 *    Mac OS X:         in version 10.0 and later in QuickTime.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in QuickTimeVRLib 2.0 and later
 *    Windows:          in QTVR.lib 2.1 and later
 }
function QTVRRefreshBackBuffer( qtvr: QTVRInstance; flags: UInt32 ): OSErr; external name '_QTVRRefreshBackBuffer';
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
