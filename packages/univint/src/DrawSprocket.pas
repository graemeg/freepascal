{
<<<<<<< HEAD
     File:       DrawSprocket/DrawSprocket.h
 
     Contains:   Games Sprockets: DrawSprocket interfaces
 
     Version:    DrawSprocket-2.0.85~65
 
     Copyright:  � 1999-2008 by Apple Computer, Inc., all rights reserved.
=======
     File:       DrawSprocket.p
 
     Contains:   Games Sprockets: DrawSprocket interfaces
 
     Version:    Technology: Draw Sprocket 1.7
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  � 1996-2002 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
                     http://www.freepascal.org/bugs.html
 
}


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

unit DrawSprocket;
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
>>>>>>> origin/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
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
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
uses MacTypes,Video,Events,QuickdrawTypes,QDOffscreen,Displays,MacErrors;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{******************* DEPRECATION NOTICE *********************
 *
 * The DrawSprocket API is being deprecated, and should be replaced
 * by the CGDirectDisplay API in the CoreGraphics framework in 
 * ApplicationServices.framework.
 *
 ************************************************************}

=======
uses MacTypes,Video,Events,Quickdraw,QDOffscreen,Displays,MacErrors;

{$ALIGN POWER}

>>>>>>> graemeg/fixes_2_2
{
********************************************************************************
** constants
********************************************************************************
}
<<<<<<< HEAD
type
	DSpDepthMask = SInt32;
const
	kDSpDepthMask_1 = 1 shl 0;
	kDSpDepthMask_2 = 1 shl 1;
	kDSpDepthMask_4 = 1 shl 2;
	kDSpDepthMask_8 = 1 shl 3;
	kDSpDepthMask_16 = 1 shl 4;
	kDSpDepthMask_32 = 1 shl 5;
	kDSpDepthMask_All = -1;

type
	DSpColorNeeds = SInt32;
const
	kDSpColorNeeds_DontCare = 0;
	kDSpColorNeeds_Request = 1;
	kDSpColorNeeds_Require = 2;

type
	DSpContextState = SInt32;
const
	kDSpContextState_Active = 0;
	kDSpContextState_Paused = 1;
	kDSpContextState_Inactive = 2;

{ kDSpContextOption_QD3DAccel not yet implemented }
type
	DSpContextOption = SInt32;
const
{    kDSpContextOption_QD3DAccel       = 1<<0,}
	kDSpContextOption_PageFlip = 1 shl 1;
	kDSpContextOption_DontSyncVBL = 1 shl 2;
	kDSpContextOption_Stereoscopic = 1 shl 3;

type
	DSpAltBufferOption = SInt32;
const
	kDSpAltBufferOption_RowBytesEqualsWidth = 1 shl 0;

type
	DSpBufferKind = SInt32;
const
	kDSpBufferKind_Normal = 0;

type
	DSpBlitMode = SInt32;
const
	kDSpBlitMode_Plain = 0;
	kDSpBlitMode_SrcKey = 1 shl 0;
	kDSpBlitMode_DstKey = 1 shl 1;
	kDSpBlitMode_Interpolation = 1 shl 2;

{
********************************************************************************
** data types
********************************************************************************
}
type
	DSpAltBufferReference = ^OpaqueDSpAltBufferReference; { an opaque type }
	OpaqueDSpAltBufferReference = record end;
	DSpAltBufferReferencePtr = ^DSpAltBufferReference;  { when a var xx:DSpAltBufferReference parameter can be nil, it is changed to xx: DSpAltBufferReferencePtr }
	DSpContextReference = ^OpaqueDSpContextReference; { an opaque type }
	OpaqueDSpContextReference = record end;
	DSpContextReferencePtr = ^DSpContextReference;  { when a var xx:DSpContextReference parameter can be nil, it is changed to xx: DSpContextReferencePtr }
	DSpContextReferenceConst = ^OpaqueDSpContextReference;
	DSpContextReferenceConstPtr = ^DSpContextReferenceConst;  { when a var xx:DSpContextReferenceConst parameter can be nil, it is changed to xx: DSpContextReferenceConstPtr }
const
	kDSpEveryContext = nil;
type
	DSpEventProcPtr = function( var inEvent: EventRecord ): Boolean;
	DSpCallbackProcPtr = function( inContext: DSpContextReference; inRefCon: UnivPtr ): Boolean;
	DSpEventUPP = DSpEventProcPtr;
	DSpCallbackUPP = DSpCallbackProcPtr;
{
 *  NewDSpEventUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

=======

type
	DSpDepthMask 				= SInt32;
const
	kDSpDepthMask_1				= $01;
	kDSpDepthMask_2				= $02;
	kDSpDepthMask_4				= $04;
	kDSpDepthMask_8				= $08;
	kDSpDepthMask_16			= $10;
	kDSpDepthMask_32			= $20;
	kDSpDepthMask_All			= -1;


type
	DSpColorNeeds 				= SInt32;
const
	kDSpColorNeeds_DontCare		= 0;
	kDSpColorNeeds_Request		= 1;
	kDSpColorNeeds_Require		= 2;


type
	DSpContextState 			= SInt32;
const
	kDSpContextState_Active		= 0;
	kDSpContextState_Paused		= 1;
	kDSpContextState_Inactive	= 2;

	{	 kDSpContextOption_QD3DAccel not yet implemented 	}

type
	DSpContextOption 			= SInt32;
const
																{     kDSpContextOption_QD3DAccel       = 1<<0, }
	kDSpContextOption_PageFlip	= $02;
	kDSpContextOption_DontSyncVBL = $04;
	kDSpContextOption_Stereoscopic = $08;


type
	DSpAltBufferOption 			= SInt32;
const
	kDSpAltBufferOption_RowBytesEqualsWidth = $01;


type
	DSpBufferKind 				= SInt32;
const
	kDSpBufferKind_Normal		= 0;


type
	DSpBlitMode 				= SInt32;
const
	kDSpBlitMode_Plain			= 0;
	kDSpBlitMode_SrcKey			= $01;
	kDSpBlitMode_DstKey			= $02;
	kDSpBlitMode_Interpolation	= $04;

	{	
	********************************************************************************
	** data types
	********************************************************************************
		}

type
	DSpAltBufferReference    = ^SInt32; { an opaque 32-bit type }
	DSpAltBufferReferencePtr = ^DSpAltBufferReference;  { when a var xx:DSpAltBufferReference parameter can be nil, it is changed to xx: DSpAltBufferReferencePtr }
	DSpContextReference    = ^SInt32; { an opaque 32-bit type }
	DSpContextReferencePtr = ^DSpContextReference;  { when a var xx:DSpContextReference parameter can be nil, it is changed to xx: DSpContextReferencePtr }
	DSpContextReferenceConst    = ^SInt32; { an opaque 32-bit type }
	DSpContextReferenceConstPtr = ^DSpContextReferenceConst;  { when a var xx:DSpContextReferenceConst parameter can be nil, it is changed to xx: DSpContextReferenceConstPtr }
{$ifc TYPED_FUNCTION_POINTERS}
	DSpEventProcPtr = function(var inEvent: EventRecord): boolean;
{$elsec}
	DSpEventProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	DSpCallbackProcPtr = function(inContext: DSpContextReference; inRefCon: UnivPtr): boolean;
{$elsec}
	DSpCallbackProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	DSpEventUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DSpEventUPP = DSpEventProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	DSpCallbackUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DSpCallbackUPP = DSpCallbackProcPtr;
{$endc}	

const
	uppDSpEventProcInfo = $000000D1;
	uppDSpCallbackProcInfo = $000003D1;
{$ifc CALL_NOT_IN_CARBON}
	{
	 *  NewDSpEventUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        not available
	 *    Mac OS X:         not available
	 	}
function NewDSpEventUPP(userRoutine: DSpEventProcPtr): DSpEventUPP; external name '_NewDSpEventUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  NewDSpCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function NewDSpCallbackUPP(userRoutine: DSpCallbackProcPtr): DSpCallbackUPP; external name '_NewDSpCallbackUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeDSpEventUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure DisposeDSpEventUPP(userUPP: DSpEventUPP); external name '_DisposeDSpEventUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  DisposeDSpCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure DisposeDSpCallbackUPP(userUPP: DSpCallbackUPP); external name '_DisposeDSpCallbackUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeDSpEventUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function InvokeDSpEventUPP(var inEvent: EventRecord; userRoutine: DSpEventUPP): boolean; external name '_InvokeDSpEventUPP';
>>>>>>> graemeg/fixes_2_2
{
 *  InvokeDSpCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }
=======
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function InvokeDSpCallbackUPP(inContext: DSpContextReference; inRefCon: UnivPtr; userRoutine: DSpCallbackUPP): boolean; external name '_InvokeDSpCallbackUPP';
{$endc}  {CALL_NOT_IN_CARBON}

>>>>>>> graemeg/fixes_2_2

type
	DSpContextAttributesPtr = ^DSpContextAttributes;
	DSpContextAttributes = record
<<<<<<< HEAD
		frequency: Fixed;
		displayWidth: UInt32;
		displayHeight: UInt32;
		reserved1: UInt32;
		reserved2: UInt32;
		colorNeeds: UInt32;
		colorTable: CTabHandle;
		contextOptions: OptionBits;
		backBufferDepthMask: OptionBits;
		displayDepthMask: OptionBits;
		backBufferBestDepth: UInt32;
		displayBestDepth: UInt32;
		pageCount: UInt32;
		filler1,filler2,filler3: SInt8;
		gameMustConfirmSwitch: Boolean;
		reserved3: array [0..4-1] of UInt32;
	end;
type
	DSpAltBufferAttributesPtr = ^DSpAltBufferAttributes;
	DSpAltBufferAttributes = record
		width: UInt32;
		height: UInt32;
		options: DSpAltBufferOption;
		reserved: array [0..4-1] of UInt32;
	end;
type
	DSpBlitInfoPtr = ^DSpBlitInfo;
	DSpBlitDoneProc = procedure( info: DSpBlitInfoPtr );
	DSpBlitInfo = record
		completionFlag: Boolean;
		filler1, filler2, filler3: SInt8;
		completionProc: DSpBlitDoneProc;
		srcContext: DSpContextReference;
		srcBuffer: CGrafPtr;
		srcRect: Rect;
		srcKey: UInt32;

		dstContext: DSpContextReference;
		dstBuffer: CGrafPtr;
		dstRect: Rect;
		dstKey: UInt32;

		mode: DSpBlitMode;
		reserved: array [0..4-1] of UInt32;
	end;
{
********************************************************************************
** function prototypes
********************************************************************************
}

{
** global operations
}
{
 *  DSpStartup()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpStartup: OSStatus; external name '_DSpStartup';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpShutdown()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpShutdown: OSStatus; external name '_DSpShutdown';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpGetVersion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 }
function DSpGetVersion: NumVersion; external name '_DSpGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpGetFirstContext()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpGetFirstContext( inDisplayID: DisplayIDType; var outContext: DSpContextReference ): OSStatus; external name '_DSpGetFirstContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpGetNextContext()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpGetNextContext( inCurrentContext: DSpContextReference; var outContext: DSpContextReference ): OSStatus; external name '_DSpGetNextContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpGetCurrentContext()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 }
function DSpGetCurrentContext( inDisplayID: DisplayIDType; var outContext: DSpContextReference ): OSStatus; external name '_DSpGetCurrentContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpFindBestContext()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpFindBestContext( inDesiredAttributes: DSpContextAttributesPtr; var outContext: DSpContextReference ): OSStatus; external name '_DSpFindBestContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpFindBestContextOnDisplayID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 }
function DSpFindBestContextOnDisplayID( inDesiredAttributes: DSpContextAttributesPtr; var outContext: DSpContextReference; inDisplayID: DisplayIDType ): OSStatus; external name '_DSpFindBestContextOnDisplayID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
		frequency:				Fixed;
		displayWidth:			UInt32;
		displayHeight:			UInt32;
		reserved1:				UInt32;
		reserved2:				UInt32;
		colorNeeds:				UInt32;
		colorTable:				CTabHandle;
		contextOptions:			OptionBits;
		backBufferDepthMask:	OptionBits;
		displayDepthMask:		OptionBits;
		backBufferBestDepth:	UInt32;
		displayBestDepth:		UInt32;
		pageCount:				UInt32;
		filler1,filler2,filler3:		SInt8;
		gameMustConfirmSwitch:	boolean;
		reserved3:				array [0..3] of UInt32;
	end;

	DSpAltBufferAttributesPtr = ^DSpAltBufferAttributes;
	DSpAltBufferAttributes = record
		width:					UInt32;
		height:					UInt32;
		options:				DSpAltBufferOption;
		reserved:				array [0..3] of UInt32;
	end;

	DSpBlitInfoPtr = ^DSpBlitInfo;
{$ifc TYPED_FUNCTION_POINTERS}
	DSpBlitDoneProc = procedure(info: DSpBlitInfoPtr);
{$elsec}
	DSpBlitDoneProc = ProcPtr;
{$endc}

	DSpBlitInfo = record
		completionFlag:			boolean;
		filler1, filler2, filler3:	SInt8;
		completionProc:			DSpBlitDoneProc;
		srcContext:				DSpContextReference;
		srcBuffer:				CGrafPtr;
		srcRect:				Rect;
		srcKey:					UInt32;
		dstContext:				DSpContextReference;
		dstBuffer:				CGrafPtr;
		dstRect:				Rect;
		dstKey:					UInt32;
		mode:					DSpBlitMode;
		reserved:				array [0..3] of UInt32;
	end;

	{	
	********************************************************************************
	** function prototypes
	********************************************************************************
		}

	{	
	** global operations
		}
	{
	 *  DSpStartup()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
	 *    CarbonLib:        not available
	 *    Mac OS X:         in version 10.0 and later
	 	}
function DSpStartup: OSStatus; external name '_DSpStartup';

{
 *  DSpShutdown()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpShutdown: OSStatus; external name '_DSpShutdown';

{
 *  DSpGetVersion()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpGetVersion: NumVersion; external name '_DSpGetVersion';

{
 *  DSpGetFirstContext()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpGetFirstContext(inDisplayID: DisplayIDType; var outContext: DSpContextReference): OSStatus; external name '_DSpGetFirstContext';

{
 *  DSpGetNextContext()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpGetNextContext(inCurrentContext: DSpContextReference; var outContext: DSpContextReference): OSStatus; external name '_DSpGetNextContext';

{
 *  DSpGetCurrentContext()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpGetCurrentContext(inDisplayID: DisplayIDType; var outContext: DSpContextReference): OSStatus; external name '_DSpGetCurrentContext';

{
 *  DSpFindBestContext()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpFindBestContext(inDesiredAttributes: DSpContextAttributesPtr; var outContext: DSpContextReference): OSStatus; external name '_DSpFindBestContext';

{
 *  DSpFindBestContextOnDisplayID()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpFindBestContextOnDisplayID(inDesiredAttributes: DSpContextAttributesPtr; var outContext: DSpContextReference; inDisplayID: DisplayIDType): OSStatus; external name '_DSpFindBestContextOnDisplayID';

{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  DSpCanUserSelectContext()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpCanUserSelectContext(inDesiredAttributes: DSpContextAttributesPtr; var outUserCanSelectContext: boolean): OSStatus; external name '_DSpCanUserSelectContext';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpUserSelectContext()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }


{
 *  DSpProcessEvent()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpProcessEvent( var inEvent: EventRecord; var outEventWasProcessed: Boolean ): OSStatus; external name '_DSpProcessEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpSetBlankingColor()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpSetBlankingColor( const (*var*) inRGBColor: RGBColor ): OSStatus; external name '_DSpSetBlankingColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpSetDebugMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpSetDebugMode( inDebugMode: Boolean ): OSStatus; external name '_DSpSetDebugMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpFindContextFromPoint()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpFindContextFromPoint( inGlobalPoint: Point; var outContext: DSpContextReference ): OSStatus; external name '_DSpFindContextFromPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpGetMouse()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpGetMouse( var outGlobalPoint: Point ): OSStatus; external name '_DSpGetMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpUserSelectContext(inDesiredAttributes: DSpContextAttributesPtr; inDialogDisplayLocation: DisplayIDType; inEventProc: DSpEventUPP; var outContext: DSpContextReference): OSStatus; external name '_DSpUserSelectContext';

{$endc}  {CALL_NOT_IN_CARBON}

{
 *  DSpProcessEvent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpProcessEvent(var inEvent: EventRecord; var outEventWasProcessed: boolean): OSStatus; external name '_DSpProcessEvent';

{
 *  DSpSetBlankingColor()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpSetBlankingColor(const (*var*) inRGBColor: RGBColor): OSStatus; external name '_DSpSetBlankingColor';

{
 *  DSpSetDebugMode()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpSetDebugMode(inDebugMode: boolean): OSStatus; external name '_DSpSetDebugMode';

{
 *  DSpFindContextFromPoint()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpFindContextFromPoint(inGlobalPoint: Point; var outContext: DSpContextReference): OSStatus; external name '_DSpFindContextFromPoint';

{
 *  DSpGetMouse()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpGetMouse(var outGlobalPoint: Point): OSStatus; external name '_DSpGetMouse';
>>>>>>> graemeg/fixes_2_2

{
** alternate buffer operations
}
<<<<<<< HEAD
=======
{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  DSpAltBuffer_New()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpAltBuffer_New(inContext: DSpContextReference; inVRAMBuffer: boolean; var inAttributes: DSpAltBufferAttributes; var outAltBuffer: DSpAltBufferReference): OSStatus; external name '_DSpAltBuffer_New';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpAltBuffer_Dispose()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpAltBuffer_Dispose(inAltBuffer: DSpAltBufferReference): OSStatus; external name '_DSpAltBuffer_Dispose';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpAltBuffer_InvalRect()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpAltBuffer_InvalRect(inAltBuffer: DSpAltBufferReference; const (*var*) inInvalidRect: Rect): OSStatus; external name '_DSpAltBuffer_InvalRect';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpAltBuffer_GetCGrafPtr()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpAltBuffer_GetCGrafPtr(inAltBuffer: DSpAltBufferReference; inBufferKind: DSpBufferKind; var outCGrafPtr: CGrafPtr; var outGDevice: GDHandle): OSStatus; external name '_DSpAltBuffer_GetCGrafPtr';
>>>>>>> graemeg/fixes_2_2

{
** context operations
}
{ general }
<<<<<<< HEAD
{
 *  DSpContext_GetAttributes()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_GetAttributes( inContext: DSpContextReferenceConst; outAttributes: DSpContextAttributesPtr ): OSStatus; external name '_DSpContext_GetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_Reserve()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_Reserve( inContext: DSpContextReference; inDesiredAttributes: DSpContextAttributesPtr ): OSStatus; external name '_DSpContext_Reserve';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_Queue()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 }
function DSpContext_Queue( inParentContext: DSpContextReference; inChildContext: DSpContextReference; inDesiredAttributes: DSpContextAttributesPtr ): OSStatus; external name '_DSpContext_Queue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_Switch()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 }
function DSpContext_Switch( inOldContext: DSpContextReference; inNewContext: DSpContextReference ): OSStatus; external name '_DSpContext_Switch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_Release()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_Release( inContext: DSpContextReference ): OSStatus; external name '_DSpContext_Release';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_Dispose()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function DSpContext_Dispose( inContext: DSpContextReference ): OSStatus; external name '_DSpContext_Dispose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_GetDisplayID()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_GetDisplayID( inContext: DSpContextReferenceConst; var outDisplayID: DisplayIDType ): OSStatus; external name '_DSpContext_GetDisplayID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_GlobalToLocal()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_GlobalToLocal( inContext: DSpContextReferenceConst; var ioPoint: Point ): OSStatus; external name '_DSpContext_GlobalToLocal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_LocalToGlobal()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_LocalToGlobal( inContext: DSpContextReferenceConst; var ioPoint: Point ): OSStatus; external name '_DSpContext_LocalToGlobal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  DSpContext_GetAttributes()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_GetAttributes(inContext: DSpContextReferenceConst; outAttributes: DSpContextAttributesPtr): OSStatus; external name '_DSpContext_GetAttributes';

{
 *  DSpContext_Reserve()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_Reserve(inContext: DSpContextReference; inDesiredAttributes: DSpContextAttributesPtr): OSStatus; external name '_DSpContext_Reserve';

{
 *  DSpContext_Queue()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_Queue(inParentContext: DSpContextReference; inChildContext: DSpContextReference; inDesiredAttributes: DSpContextAttributesPtr): OSStatus; external name '_DSpContext_Queue';

{
 *  DSpContext_Switch()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.7 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_Switch(inOldContext: DSpContextReference; inNewContext: DSpContextReference): OSStatus; external name '_DSpContext_Switch';

{
 *  DSpContext_Release()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_Release(inContext: DSpContextReference): OSStatus; external name '_DSpContext_Release';

{
 *  DSpContext_Dispose()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_Dispose(inContext: DSpContextReference): OSStatus; external name '_DSpContext_Dispose';

{
 *  DSpContext_GetDisplayID()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_GetDisplayID(inContext: DSpContextReferenceConst; var outDisplayID: DisplayIDType): OSStatus; external name '_DSpContext_GetDisplayID';

{
 *  DSpContext_GlobalToLocal()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_GlobalToLocal(inContext: DSpContextReferenceConst; var ioPoint: Point): OSStatus; external name '_DSpContext_GlobalToLocal';

{
 *  DSpContext_LocalToGlobal()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_LocalToGlobal(inContext: DSpContextReferenceConst; var ioPoint: Point): OSStatus; external name '_DSpContext_LocalToGlobal';

{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  DSpContext_SetVBLProc()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_SetVBLProc(inContext: DSpContextReference; inProcPtr: DSpCallbackUPP; inRefCon: UnivPtr): OSStatus; external name '_DSpContext_SetVBLProc';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpContext_GetFlattenedSize()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_GetFlattenedSize(inContext: DSpContextReference; var outFlatContextSize: UInt32): OSStatus; external name '_DSpContext_GetFlattenedSize';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpContext_Flatten()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_Flatten(inContext: DSpContextReference; outFlatContext: UnivPtr): OSStatus; external name '_DSpContext_Flatten';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpContext_Restore()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }


{
 *  DSpContext_GetMonitorFrequency()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_GetMonitorFrequency( inContext: DSpContextReferenceConst; var outFrequency: Fixed ): OSStatus; external name '_DSpContext_GetMonitorFrequency';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_Restore(inFlatContext: UnivPtr; var outRestoredContext: DSpContextReference): OSStatus; external name '_DSpContext_Restore';

{$endc}  {CALL_NOT_IN_CARBON}

{
 *  DSpContext_GetMonitorFrequency()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_GetMonitorFrequency(inContext: DSpContextReferenceConst; var outFrequency: Fixed): OSStatus; external name '_DSpContext_GetMonitorFrequency';

{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  DSpContext_SetMaxFrameRate()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_SetMaxFrameRate(inContext: DSpContextReference; inMaxFPS: UInt32): OSStatus; external name '_DSpContext_SetMaxFrameRate';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpContext_GetMaxFrameRate()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }


{
 *  DSpContext_SetState()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_SetState( inContext: DSpContextReference; inState: DSpContextState ): OSStatus; external name '_DSpContext_SetState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_GetState()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_GetState( inContext: DSpContextReferenceConst; var outState: DSpContextState ): OSStatus; external name '_DSpContext_GetState';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_IsBusy()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_IsBusy( inContext: DSpContextReferenceConst; var outBusyFlag: Boolean ): OSStatus; external name '_DSpContext_IsBusy';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ dirty rectangles }
=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_GetMaxFrameRate(inContext: DSpContextReferenceConst; var outMaxFPS: UInt32): OSStatus; external name '_DSpContext_GetMaxFrameRate';

{$endc}  {CALL_NOT_IN_CARBON}

{
 *  DSpContext_SetState()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_SetState(inContext: DSpContextReference; inState: DSpContextState): OSStatus; external name '_DSpContext_SetState';

{
 *  DSpContext_GetState()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_GetState(inContext: DSpContextReferenceConst; var outState: DSpContextState): OSStatus; external name '_DSpContext_GetState';

{
 *  DSpContext_IsBusy()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_IsBusy(inContext: DSpContextReferenceConst; var outBusyFlag: boolean): OSStatus; external name '_DSpContext_IsBusy';

{ dirty rectangles }
{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  DSpContext_SetDirtyRectGridSize()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_SetDirtyRectGridSize(inContext: DSpContextReference; inCellPixelWidth: UInt32; inCellPixelHeight: UInt32): OSStatus; external name '_DSpContext_SetDirtyRectGridSize';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpContext_GetDirtyRectGridSize()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_GetDirtyRectGridSize(inContext: DSpContextReferenceConst; var outCellPixelWidth: UInt32; var outCellPixelHeight: UInt32): OSStatus; external name '_DSpContext_GetDirtyRectGridSize';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpContext_GetDirtyRectGridUnits()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_GetDirtyRectGridUnits(inContext: DSpContextReferenceConst; var outCellPixelWidth: UInt32; var outCellPixelHeight: UInt32): OSStatus; external name '_DSpContext_GetDirtyRectGridUnits';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpContext_InvalBackBufferRect()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_InvalBackBufferRect(inContext: DSpContextReference; const (*var*) inRect: Rect): OSStatus; external name '_DSpContext_InvalBackBufferRect';
>>>>>>> graemeg/fixes_2_2

{ underlays }
{
 *  DSpContext_SetUnderlayAltBuffer()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_SetUnderlayAltBuffer(inContext: DSpContextReference; inNewUnderlay: DSpAltBufferReference): OSStatus; external name '_DSpContext_SetUnderlayAltBuffer';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpContext_GetUnderlayAltBuffer()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }


{ gamma }
{
 *  DSpContext_FadeGammaOut()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_FadeGammaOut( inContext: DSpContextReference; var inZeroIntensityColor: RGBColor ): OSStatus; external name '_DSpContext_FadeGammaOut';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_FadeGammaIn()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_FadeGammaIn( inContext: DSpContextReference; var inZeroIntensityColor: RGBColor ): OSStatus; external name '_DSpContext_FadeGammaIn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_FadeGamma()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_FadeGamma( inContext: DSpContextReference; inPercentOfOriginalIntensity: SInt32; var inZeroIntensityColor: RGBColor ): OSStatus; external name '_DSpContext_FadeGamma';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ buffering }
{
 *  DSpContext_SwapBuffers()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_SwapBuffers( inContext: DSpContextReference; inBusyProc: DSpCallbackUPP; inUserRefCon: UnivPtr ): OSStatus; external name '_DSpContext_SwapBuffers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_GetBackBuffer()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_GetBackBuffer( inContext: DSpContextReference; inBufferKind: DSpBufferKind; var outBackBuffer: CGrafPtr ): OSStatus; external name '_DSpContext_GetBackBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_GetFrontBuffer()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.1 and later
 }
function DSpContext_GetFrontBuffer( inContext: DSpContextReferenceConst; var outFrontBuffer: CGrafPtr ): OSStatus; external name '_DSpContext_GetFrontBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ clut operations }
{
 *  DSpContext_SetCLUTEntries()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_SetCLUTEntries( inContext: DSpContextReference; const (*var*) inEntries: ColorSpec; inStartingEntry: UInt16; inLastEntry: UInt16 ): OSStatus; external name '_DSpContext_SetCLUTEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DSpContext_GetCLUTEntries()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in DrawSprocket.framework but deprecated in 10.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 }
function DSpContext_GetCLUTEntries( inContext: DSpContextReferenceConst; var outEntries: ColorSpec; inStartingEntry: UInt16; inLastEntry: UInt16 ): OSStatus; external name '_DSpContext_GetCLUTEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ blit operations }
=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpContext_GetUnderlayAltBuffer(inContext: DSpContextReferenceConst; var outUnderlay: DSpAltBufferReference): OSStatus; external name '_DSpContext_GetUnderlayAltBuffer';

{ gamma }
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  DSpContext_FadeGammaOut()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_FadeGammaOut(inContext: DSpContextReference; var inZeroIntensityColor: RGBColor): OSStatus; external name '_DSpContext_FadeGammaOut';

{
 *  DSpContext_FadeGammaIn()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_FadeGammaIn(inContext: DSpContextReference; var inZeroIntensityColor: RGBColor): OSStatus; external name '_DSpContext_FadeGammaIn';

{
 *  DSpContext_FadeGamma()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_FadeGamma(inContext: DSpContextReference; inPercentOfOriginalIntensity: SInt32; var inZeroIntensityColor: RGBColor): OSStatus; external name '_DSpContext_FadeGamma';

{ buffering }
{
 *  DSpContext_SwapBuffers()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_SwapBuffers(inContext: DSpContextReference; inBusyProc: DSpCallbackUPP; inUserRefCon: UnivPtr): OSStatus; external name '_DSpContext_SwapBuffers';

{
 *  DSpContext_GetBackBuffer()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_GetBackBuffer(inContext: DSpContextReference; inBufferKind: DSpBufferKind; var outBackBuffer: CGrafPtr): OSStatus; external name '_DSpContext_GetBackBuffer';

{
 *  DSpContext_GetFrontBuffer()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_GetFrontBuffer(inContext: DSpContextReferenceConst; var outFrontBuffer: CGrafPtr): OSStatus; external name '_DSpContext_GetFrontBuffer';

{ clut operations }
{
 *  DSpContext_SetCLUTEntries()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_SetCLUTEntries(inContext: DSpContextReference; const (*var*) inEntries: ColorSpec; inStartingEntry: UInt16; inLastEntry: UInt16): OSStatus; external name '_DSpContext_SetCLUTEntries';

{
 *  DSpContext_GetCLUTEntries()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in DrawSprocketLib 1.0 and later
 *    CarbonLib:        not available
 *    Mac OS X:         in version 10.0 and later
 }
function DSpContext_GetCLUTEntries(inContext: DSpContextReferenceConst; var outEntries: ColorSpec; inStartingEntry: UInt16; inLastEntry: UInt16): OSStatus; external name '_DSpContext_GetCLUTEntries';

{ blit operations }
{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  DSpBlit_Faster()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.1 and later
 }

=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpBlit_Faster(inBlitInfo: DSpBlitInfoPtr; inAsyncFlag: boolean): OSStatus; external name '_DSpBlit_Faster';
>>>>>>> graemeg/fixes_2_2

{
 *  DSpBlit_Fastest()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DrawSprocketLib 1.1 and later
 }


{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
 *    Non-Carbon CFM:   in DrawSprocketLib 1.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function DSpBlit_Fastest(inBlitInfo: DSpBlitInfoPtr; inAsyncFlag: boolean): OSStatus; external name '_DSpBlit_Fastest';


{$endc}  {CALL_NOT_IN_CARBON}

{$ALIGN MAC68K}


end.
>>>>>>> graemeg/fixes_2_2
